---
title: Single-ball toss — nominated catch state via a Toss action (MVP Phase 8 expansion)
created: 2026-07-24
status: active
related_logbook:
  - 2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md
  - 2026-07-01-rung2b-kinematic-release.md
  - 2026-06-30-rung2a-single-ball-tilt-throw.md
  - 2026-06-30-rung1-clean-single-catch.md
  - 2026-06-27-throw-aim-band-limit-and-closed-loop-catch.md
  - 2026-07-04-codesign-catch-continuous-velocity-matched.md
related_code:
  - sim/reload_gate.py::ReloadGate
  - sim/hand/trajectory.py::HandThrowSequence
  - sim/hand/planner.py::ThrowCatchPlanner
  - sim/juggle_tilt.py::tilt_to_throw
  - controller/ballistics.py::compute_launch_velocity
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py::build_catch
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
---

# Plan — Single-ball toss to a nominated catch state (`Toss.action`)

**Branch:** `mvp-trajectory-bringup`
**Expands:** `plans/active/mvp-trajectory-bringup.md` § Phase 8 (the stretch
"self-toss loop" sketch becomes this staged plan; the level-platform MVP throw is
retained as Tier 8a, and nominated-catch-position throws become Tier 8b).
**Carries forward (merged 2026-07-24 from `demo/bb-led-two-ball-juggle`):**
the online-juggle tilt-ladder primitives and their evidence —
`plans/active/bb-online-juggle-tilt-rearchitecture.md` Rungs 0–2b
(tilt-aim geometry, single-catch-at-position, throw-to-target, 12/12 self-catch
with kinematic release).

## Context

Phase 7 (BB→Jugglebot reload) is mid-hardware-bringup at 15/19 catches with the
open-loop-platform pivot landed; the catch pipeline (announcement → tracker
correlation → `arm_catch` latch → `catch/dynamic_target` → `build_catch` →
emitter → pump) is hardware-proven. The merged tilt-ladder work proved, in sim,
the two primitives Phase 8 needs beyond that pipeline: a throw to a nominated
landing point (Rung 2a: tilt-aimed, ≤33 mm inside the scoped box) and a
throw→self-catch loop that converges (Rung 2b: 12/12 on all seeds — but only
with a *kinematic* release; the contact-detach path has landing-vs-origin
sensitivity ≈2.7).

The gap is hosting: every ladder primitive drives the platform through sim-side
kinematic realisation (`realize_tilted` → plant), not through the production
trajectory stack the hardware runs. This plan re-hosts the throw on the
production stack behind a `Toss` action, gates it with a `reload_gate.py`-style
production-in-the-loop sim harness, and stages hardware bring-up.

**Ball sourcing precondition:** a toss needs a ball already seated in the cup —
the `Reload` action is the loader. The operator sequence is Reload → Toss
(→ Toss …), which also means every toss session inherits the Phase-7 session
disciplines (can-bridge Teensy reboot before each sitting; `uptime_ms` logged
with any timing measurement; tracker-corruption verdicts judged by eye).

## Architecture

### Tiering — one action, two capability tiers

- **Tier 8a — toss-at-position (platform level).** The platform *translates to
  the nominated catch (x, y) before the throw* (within the ±150 mm workspace),
  then stays level and quiescent through release; the hand stroke supplies all
  throw energy (`traj_type=0`, `event_vel` from ballistics). Throw and catch are
  co-located; catch z and flight time (⇔ throw height) vary freely. This covers
  "vary all of catch(x, y, z) and throw height" without tilt, and is the
  hardware-first tier.
- **Tier 8b — displaced throw→catch (tilt-aimed).** Throw from A, catch at B:
  the platform pre-tilts, the slider ejects along the tilted cup axis
  (`lateral = |v|·sinθ`, Rung 2a), then the platform translate-to-reaches B for
  the catch (Rung 1 / `build_catch`). Lateral take-off velocity is **never**
  demanded from platform translation — that is the band-limit wall the ladder
  hit (−3 dB ≈ 5 Hz, `logbook/2026-06-27-online-replanning-architecture-and-cup-bandlimit.md`;
  wall diagnosis in `logbook/2026-06-27-throw-aim-band-limit-and-closed-loop-catch.md`).

The action goal is identical for both tiers; a tier parameter (config default +
per-goal override) selects capability. Tier 8b is rejected loudly
(`REJECTED_TIER`) until Phase 4 lands.

### `Toss.action` (new, `jugglebot_interfaces`)

```
# Jugglebot self-toss: throw the seated ball to a nominated catch state and
# catch it. Runs within ACTIVE:TRAJECTORY (no mode switch — leaving the mode
# mid-sequence is the documented abort), mirrors Reload.action choreography.
geometry_msgs/Point catch_position   # nominated catch point, mm, STOW-relative
                                     # platform frame (same convention as
                                     # TimedTarget.pose; z 170 = ACTIVE plane)
float64 flight_time_s                # ballistic flight time; sets throw height
                                     # (apex = g·T²/8 above release for the
                                     # vertical toss); 0 => config default
float64 throw_delay_s                # delay from goal-accept to release; 0 => default
float64 catch_vel_scale              # 0 => config default; same semantics and
                                     # clamp [0.3, 1.5] as Reload.action
---
bool success                         # True iff OUR announced ball was CAUGHT
                                     # (tracker-id-correlated, as Reload)
string outcome                       # CAUGHT | REJECTED_<code> | ABORTED_<code> | MISSED | MISSED_<code>
float64 catch_error_mm               # tracker last-KF horizontal miss (NaN unknown)
float64 achieved_flight_s            # measured release→catch-plane time (NaN unknown)
---
string phase                         # CHECKING | POSITIONING | PREPARING | THROWING |
                                     # BALL_IN_FLIGHT | CATCHING | SETTLING
```

**Frame convention (single conversion point).** The goal is STOW-relative
platform frame for operator consistency with `TimedTarget`/`go_to_pose`; the
coordinator converts once to global mm for the self-`ThrowAnnouncement`
(`landing_position` is global, per the message spec). The Phase-7 first sitting
was rejected wholesale by a z frame double-add
(`logbook/2026-07-23` first-sitting entry) — the conversion lives in exactly one
tested function, with a regression test pinning both frames' values for one
worked example.

### Choreography (mirrors Reload; throw side is new)

1. **CHECKING** — preconditions: ACTIVE + TRAJECTORY streaming a hold, mocap
   fresh, **ball seated** (hand at catch-rest with ball evidence; a toss with an
   empty cup is `REJECTED_NO_BALL`), no reload/toss already active.
2. **POSITIONING** — profiled `go_to_pose` to the nominated catch (x, y) at the
   toss-ready z (Tier 8a: throw site = catch site; Tier 8b: throw site A).
3. **PREPARING** — compute the release state (release position from
   hand-at-release geometry; `event_vel` from
   `controller.ballistics.compute_launch_velocity` with `flight_time_s`); raise
   the `trajectory/arm_catch` latch and confirm; publish the
   self-`ThrowAnnouncement` (`thrower_name='jugglebot'`, predicted landing =
   the nominated catch state) so the existing correlation → catch path closes
   the loop unchanged.
4. **THROWING** — dispatch the hand stroke (`SetHandTrajCmd traj_type=0`,
   `event_delay`, `event_vel`) via a **telemetry-verified ladder** — the Phase-7
   fourth sitting measured the hand ack lying both ways ~59%; a hand dispatch is
   never blind-re-sent, its outcome is read back from hand telemetry
   (`logbook/2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md`).
5. **BALL_IN_FLIGHT** — the platform holds open-loop (the Phase-7 pivot: the
   pre-positioned/pre-tilted pose is held; live tracking never moves the
   platform). Hand-stroke *catch* timing stays tracker-driven through
   `catch_coordinator`'s latch-gated arm, exactly the reload path.
6. **CATCHING → SETTLING** — as Reload (seat, verdict, lower latch, recenter).
7. **Abort** — ball still seated (pre-release): retract to park, recenter,
   lower latch. Ball in flight: the catch attempt continues (aborting a catch
   mid-flight drops a ball on the robot); cancellation is honoured at the next
   phase boundary. Enumerated per-phase in the sequencer tests.

**Firmware alternative noted, not chosen:** the Teensy accepts `traj_type=2`
(throw and catch, same ball, firmware-timed). Single-dispatch is attractive on
paper but abandons the tracker-driven catch timing the reload arc validated
(announced/predicted landings run early vs. the real flight) and forfeits the
per-phase telemetry ladder. It stays available as a bench diagnostic.

### Sim gate — `sim/toss_gate.py` (production-in-the-loop)

A sibling of `sim/reload_gate.py`, sharing its helpers (extraction into a
common module only where reuse is mechanical): real `build_catch` +
`KnotEmitter` + `SetpointPump` in the loop, MuJoCo plant, seeded §3-style noise.
The throw is realised as a **kinematic release** at the computed release state
(`Ball.ballistic_release`, merged from the ladder) with seeded release-velocity
noise — the contact-detach path mis-models the real cup (sensitivity ≈2.7 in
sim vs. visibly smooth hardware catches; the sim contact model, not the robot,
is the low-fidelity element). A contact-physics variant runs as a non-gating
diagnostic column. Release-noise magnitude is a placeholder until Phase 5's T0
bench characterisation measures the real scatter; the gate re-runs with the
measured value before any hardware catch attempt.

Sweep axes: catch (x, y) across the workspace (centre + 4 corners at ±60 mm to
start), catch z (±30 mm about ACTIVE), flight time 0.55–1.10 s (throw speeds
≈2.7–5.4 m/s, within the 7 m/s hand ceiling with margin). Gate criterion (from
the MVP plan): **≥ 9/10 toss-and-catch cycles at 2–3 m/s** per swept point,
zero feasibility violations, all knots pump-accepted.

## Implementation Phase Summary

| Phase | Scope | Gate | Status |
|---|---|---|---|
| 0 | Post-merge reconciliation (suite green, doc pointers) | full pytest | IN PROGRESS (2026-07-24 merge) |
| 1 | `Toss.action` + `toss_sequencer` FSM + coordinator wiring (Tier 8a, REJECTED_TIER for 8b) | full pytest | NOT STARTED |
| 2 | `sim/toss_gate.py` + Tier-8a sweep | Self-toss gate ≥9/10 | NOT STARTED |
| 3 | Real-ordering trace (multi-node choreography) | trace review | NOT STARTED |
| 4 | Tier 8b — tilt-aimed displaced throw on the production stack | gate sweep extension | NOT STARTED |
| 5 | Hardware bring-up T0–T4 (operator-run) | staged PASS criteria | NOT STARTED |

## Implementation Phases

### Phase 0 — Post-merge reconciliation
The 2026-07-24 merge of `demo/bb-led-two-ball-juggle` lands the ladder
primitives, probes, tests, and logbook history in the working branch. Full
suite green is the phase gate. Doc pointers: `sim/JUGGLE_DEMO.md` gains a
banner distinguishing the (paused) offline demo from the online ladder;
`plans/active/bb-led-two-ball-juggle-demo.md` § status gains a merged-location
note; the 4 ladder entries never indexed on the demo branch
(`2026-06-26-contact-mechanics-integration`, `2026-07-03-catch-control-formulation-design-basis`,
`2026-07-03-motion-quality-review`, `2026-07-03-p2-selfcatch-reunification-tension`)
get their `logbook/INDEX.md` rows backfilled. No code changes.

### Phase 1 — Action + sequencer + coordinator (Tier 8a)
- `Toss.action` as specified; `toss_sequencer.py` (pure Python FSM, mirroring
  `reload_sequencer.py` structure and its per-phase abort enumeration).
- Coordinator: either a `toss_coordinator_node` or an extension of
  `reload_coordinator_node` — **decision at implementation time** after
  measuring shared-client surface (both need `arm_catch`, hand ladder,
  `go_to_pose`; a shared mixin is likely; a merged "ball ops" node is accepted
  if the client duplication is >70%).
- The release-state computation (frame conversion + ballistics) lands as pure
  functions in `motion/` with unit tests pinning worked examples (magnitude
  sanity: 0.8 s flight ⇒ v≈3.9 m/s, apex ≈0.78 m).
- `catch_coordinator` accepts `thrower_name='jugglebot'` announcements through
  the existing correlation path (verify — the reload arc suggests it is already
  thrower-agnostic; add the test either way).
- Full pytest; commit; `/log feature`.

### Phase 2 — Sim toss gate (Tier 8a)
`sim/toss_gate.py` per Architecture; CI smoke (small-N) in `tests/sim/`;
sweep report to `temp/reports/`. Gate pass recorded with the
(date, command, result) triple. Commit; logbook entry.

### Phase 3 — Real-ordering trace
Mocked-ROS unit tests are blind to cross-process choreography (the Phase-7
audit found five BLOCKING ordering bugs exactly there). Before hardware: a
recorded real-ordering trace of Reload → Toss on the live launch (bench, no
ball — a dry trace: the harness raises a trace-only waiver for the
ball-evidence precondition so the full post-CHECKING choreography emits and
is observed without physical outcomes; an unpowered bench cannot show
hand-telemetry ball evidence, which is why a seated real ball is not the
mechanism, and every powered-session runbook leaves the waiver unset; the
un-waived `REJECTED_NO_BALL` refusal path is captured as its own short
trace), reviewed against the sequencer's assumed ordering.

### Phase 4 — Tier 8b (tilt-aimed displaced throw)
- Port `tilt_to_throw` into `motion/trajectory/tilt_geometry.py` (pure Python;
  grep-before-refactor across the sim callers; `sim/juggle_tilt.py` re-exports
  to avoid a sim churn).
- Pre-tilt + eject along the tilted axis at release; translate-to-reach catch
  at B through the unchanged catch path. Carry the Rung-2a open item: the
  ±100 mm directional asymmetry in the reliable box is unresolved — the gate
  sweep maps it before hardware.
- Gate sweep extension: displaced targets on the Rung-2a reliable box, then the
  asymmetry map. Commit; logbook entry.

### Phase 5 — Hardware bring-up (operator-run, staged)
- **T0 — bench hand-throw characterisation (no catch).** Standard session
  balls; `event_vel` ladder starting at the minimum useful throw speed; no
  catch is attempted, so every ball ends on the floor — the routine
  missed-reload outcome, cleared between throws. Measures: commanded
  `event_vel` vs. achieved release speed (mocap), release scatter (feeds the
  gate noise model), release-position repeatability. This is the
  empirical-probe-before-thresholds discipline applied to the throw.
- **T1 — single vertical toss-and-catch**, low height (flight ≈0.6 s), centre
  workspace. PASS: ≥3/5 caught.
- **T2 — height ladder** at centre: flight 0.6 → 0.9 → 1.1 s. PASS per step: ≥3/5.
- **T3 — toss-at-position**: workspace corners at ±60 mm, one height. PASS: ≥3/5 each.
- **T4 — Tier 8b displaced throw→catch** (only after Phase 4's gate): A→B at
  100 mm separation. PASS: ≥3/5.
- Session disciplines: can-bridge Teensy reboot pre-session; `uptime_ms` logged;
  `/diagnose --latest` after every session; one truthful outcome line per toss.

## Testing Plan
- Pure math (ballistics, frames, sequencer) in `tests/motion/`; node behaviour
  in `tests/ros/` (mock-ROS); gate harness smoke in `tests/sim/`.
- The production-in-the-loop invariant (every emitted knot pump-accepted)
  re-asserted inside `toss_gate` as in `reload_gate`.
- Full `pytest tests/ -q` before every commit; ci-deep at Phase 2 and Phase 4
  exits, cited with the (date, command, result) triple.

## Risk register
- **Release fidelity is the headline unknown.** The sim gate's kinematic
  release assumes the firmware stroke delivers `event_vel` cleanly; T0 exists
  to measure the truth. If real scatter is large, Tier-8a catches absorb it
  (co-located, tracker-driven catch); Tier 8b tightens the requirement — the
  knife-edge finding transfers as a hardware risk, not a sim artifact.
- **ERR_TIMEOUT epidemic (open, Phase-7)** — more hand dispatches per action
  means more exposure; the telemetry ladder is the mitigation; the standing
  investigation stays open.
- **Tracker verdict corruption (open, Phase-7)** — CAUGHT may read MISSED;
  hardware PASS counts are judged by eye + tracker-id evidence, as in the
  fourth sitting.
- **Teensy-uptime tracking lag (open)** — all timing-sensitive measurements
  (achieved flight time, catch error) are only meaningful with a fresh
  can-bridge boot; `uptime_ms` is logged alongside every session artefact.

## Notes for Collaborators
- Rung evidence and design bases live in the merged ladder logbook entries
  (2026-06-24 → 2026-07-04) and `plans/active/bb-online-juggle-tilt-rearchitecture.md`;
  that plan remains the authority for the two-ball frontier (Rung 3), which is
  **out of scope here** (MVP Phase 9 re-plans it against this plan's evidence).
- The offline juggle demo (`sim/juggle_demo.py`, CasADi optimiser) is paused,
  not deleted; nothing in this plan depends on it.
- MPC is untouched: the toss runs entirely on the trajectory stack.
