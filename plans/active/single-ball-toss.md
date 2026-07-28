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
with any timing measurement; tracker-corruption verdicts judged by eye — **on the
reload path only** since C-POSSESS-1 landed 2026-07-28; self-toss verdicts are now
expected to be right, so a self-toss `MISSED` is a finding).

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
float64 throw_height_m               # apex height above release (m) — the
                                     # juggling-relevant variable; converted to
                                     # flight T = sqrt(8·h/g) (h ∝ T²); 0 =>
                                     # config default (~0.78 m)
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
   fresh, hand at catch-rest (**hand-parked** — a physical-hazard gate, reliably
   from hand telemetry), no reload/toss already active. Ball possession is **NOT
   gated by default** (`toss_require_ball_evidence: false` — no ball-in-cup
   sensor; the operator guarantees the ball); `REJECTED_NO_BALL` fires only when
   that config gate is enabled.
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
(`Ball.ballistic_release`, merged from the ladder; the gate implements
the release via `Ball.release` — the kinematic-hold-mode ejector, since
`ballistic_release`’s contract requires contact-carry mode — see the
Phase-2 logbook entry’s Discussion) with seeded release-velocity
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
| 0 | Post-merge reconciliation (suite green, doc pointers) | full pytest | COMPLETE (2026-07-25) |
| 1 | `Toss.action` + `toss_sequencer` FSM + coordinator wiring (Tier 8a, REJECTED_TIER for 8b) | full pytest | COMPLETE (2026-07-25; tier is config-only — the locked goal has no tier field, see the phase logbook entry) |
| 2 | `sim/toss_gate.py` + Tier-8a sweep | Self-toss gate ≥9/10 | COMPLETE (2026-07-25 — PASS both binding bands, 262/290 core_clean overall; long-flight advisory tail fails under the placeholder noise, re-run after T0) |
| 3 | Real-ordering trace (multi-node choreography) | trace review | **VALIDATED (2026-07-25)** — reject + dry captures run; ordering all-green; 3 pollution/pre-position invariants corrected as a contract fix; `check --dry` on `toss_trace_2026-07-25_15-24-25` = 12 PASS / 0 FAIL |
| 4 | Tier 8b — tilt-aimed displaced throw on the production stack | gate sweep extension | COMPLETE (2026-07-25 — 8b binding ring PASS 9/9; asymmetry map landed; ships behind `JB_OP_TOSS_TIER='8a'`) |
| 5 | Hardware bring-up T0–T4 (operator-run) | staged PASS criteria | RUNBOOK READY (`tests/hardware/session_phase8_toss_hardware.md`, 2026-07-25); operator captures pending |

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
recorded real-ordering trace of a **standalone Toss** — no Reload leg and no
BB (BB cannot throw with an empty magazine and would auto-reload; the throw
side is the new, unproven choreography, and it emits regardless of whether a
ball is present) — on the live launch (**powered** no-ball bench — a dry
trace: the harness raises a trace-only waiver for the ball-evidence
precondition so the full post-CHECKING choreography emits and is observed
without physical outcomes; every ball-flying session runbook leaves the waiver
unset; the un-waived `REJECTED_NO_BALL` refusal path is captured as its own
short trace), reviewed against the sequencer's assumed ordering.
*(Amended 2026-07-25: the original "unpowered bench" wording is
unreachable — ACTIVE activation faults against unpowered ODrives and the
10 Hz mode republish reject the goal `REJECTED_WRONG_MODE` before the
waiver ever matters; with power, every precondition passes naturally except
possession, which is exactly what the waiver covers. Note the dry capture
therefore fires a real empty-cup throw stroke — the runbook frames it as a
real actuation and recommends folding it into the T0 sitting. See the
Phase-3 prep logbook entry.)* Tooling: `tests/hardware/toss_trace_recorder.py`
(recorder + offline invariant checker, DT-1..14 / RJ-1..4) +
`tests/hardware/session_phase8_toss_trace.md` (runbook).

The full real-ball **Reload → Toss** chain trace (the powered operator
sequence with a caught, seated ball) is deferred to the first Phase-5
sitting; the recorder already subscribes to the Reload action wires so the
same harness captures it there without change.

**Outcome (2026-07-25): VALIDATED.** Run on a powered no-ball bench.
*Capture R* (un-waived `REJECTED_NO_BALL`): 4/4 RJ invariants PASS — zero
choreography, proving the CHECKING chain (mode/streaming/mocap/hand-fresh/
hand-parked) all passed. *Capture D* (waived dry choreography): every ordering
invariant PASS (DT-1/2/5/6/8/10/11). Two invariants (DT-9 stroke↔BALL_IN_FLIGHT
timing, DT-12 outcome form) were confirmed as ball-in-hand artifacts by a
ball/no-ball A/B — both flipped FAIL→PASS with the hand empty, which positively
validates the release-timing model. **Checker contract correction:** DT-7,
DT-13, DT-14 were mis-specified — they assumed a no-ball toss leaves the tracker
quiescent, but a *self-announced* toss always seeds its own predicted-ball track
(`ball_prediction_node` synthesises a `destination=jugglebot` `BallState` from
the ThrowAnnouncement — `tracking=0`, no mocap detection — which drives the
catch pre-position). Corrected to distinguish that intrinsic track from a REAL
detection (`tracking=1`)/foreign track; adversarially verified (the corrected
DT-14 still fails on the real-ball run, catching `tracking=1 id=35`). Final:
`check --dry` on `temp/logs/toss_trace_2026-07-25_15-24-25.jsonl` = **12 PASS /
0 FAIL** (2 in-bundle AMBIGUOUS tolerated). Two real tosses occurred during the
arc (a ball was present) — both MISSED — standing as early Phase-5 T0/T1 data.
See `logbook/2026-07-25-toss-phase3-trace-validated.md`.

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

**LANDED 2026-07-25** (see the Phase-4 logbook entry). `tilt_to_throw` ported
(sim re-exports); `compute_release_state_tilted` (swing-compensated pre-tilt
at A + tilt-aimed launch, loud `ThrowTiltInfeasible` clamp gate); the
coordinator pre-positions **tilted at A**, raises `catch/pretilt_hold` (a new
gate suppressing the stock announcement pre-tilt — otherwise the platform
translates A→B and un-tilts *before* release), and publishes the ONE deferred
A→B reach at `t_release`. The 8b binding ring (50 mm, T=0.80) passed 9/9; the
asymmetry map (detach column, at the T=0.80 s flight) shows a
**+y-hemisphere weakness** (−y hemisphere + −x seat to 100 mm; +y/NW fail at
70 mm; the T=0.60 s sub-map is worse everywhere and is not a T4 basis). **Decisions for T4:** (a)
the throw-site A is config (`JB_OP_TOSS_THROW_SITE_MM`) — the locked goal has
no A/tier field, so the plan's "per-goal override" stays a goal-schema
question; (b) 8b ships behind `JB_OP_TOSS_TIER='8a'`; (c) the **T4-at-100 mm**
fork is open — aim into the −y hemisphere/−x if attempting 100 mm, or re-stage
T4 to ≤70 mm (the shipped cap); raising the 80 mm reach envelope or moving its
center to B is a safety-critical `trajectory_node` change not taken this run;
(d) the Phase-3 dry-trace needs an **8b addendum** confirming `pretilt_hold`
reaches `catch_coordinator` before the announcement, before T4.

### Phase 5 — Hardware bring-up (operator-run, staged)

Operator runbook: `tests/hardware/session_phase8_toss_hardware.md` (exact
commands, per-rung PASS/ABORT, preflight). The `Toss.action` goal now nominates
a **throw height** (`throw_height_m`) not a flight time, and the ball-evidence
gate is OFF by default (the operator guarantees a loaded ball — no waiver) —
see `logbook/2026-07-25-toss-action-height-and-operator-guaranteed-ball.md`. The
runbook operationalises T0 via the Toss action (measure the outgoing release from
QTM; the catch is not scored) rather than a separate throw-only command. The
rungs below are the specification; the runbook is the authority.

- **T0 — bench hand-throw characterisation (no catch).** Standard session
  balls; `event_vel` ladder starting at the minimum useful throw speed; no
  catch is attempted, so every ball ends on the floor — the routine
  missed-reload outcome, cleared between throws. Measures: commanded
  `event_vel` vs. achieved release speed (mocap), release scatter (feeds the
  gate noise model), release-position repeatability. This is the
  empirical-probe-before-thresholds discipline applied to the throw.
- **T1 — single vertical toss-and-catch**, low height (flight ≈0.7 s — see
  the risk register's kind-1 time-budget entry; 0.6 s is firmware-marginal),
  centre workspace. PASS: ≥3/5 caught.
- **T2 — height ladder** at centre: flight 0.6 → 0.9 → 1.1 s. PASS per step: ≥3/5.
- **T3 — toss-at-position**: workspace corners at ±60 mm, one height. PASS: ≥3/5 each.
- **T4 — Tier 8b displaced throw→catch** (only after Phase 4's gate AND the
  8b dry-trace addendum): A→B separation. The shipped gate caps displacement
  at **70 mm** (the clean box ∩ the 80 mm reach envelope). The plan's 100 mm
  target is direction-dependent per the Phase-4 asymmetry map — if attempted,
  aim into the **−y hemisphere or −x** at the **T≥0.80 s** flight (sim-robust
  to 100 mm there), never +y/NW (fails at 70 mm); otherwise stage T4 at ≤70 mm. Set
  `JB_OP_TOSS_TIER='8b'` and `JB_OP_TOSS_THROW_SITE_MM` for the session.
  PASS: ≥3/5.
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
- **Tracker verdict corruption (open, Phase-7) — PARTLY RETIRED 2026-07-28.**
  Split by path since contract **C-POSSESS-1**
  (`ros_ws/docs/ball_possession_contract.md`): **self-toss** verdicts are now
  correct (17/17 on the 2026-07-27 capture, where the shipped gate scored 0/17),
  so a self-toss `MISSED` on a watched catch is a **finding**, not expected noise.
  **Reload** verdicts still read `MISSED` on a real catch — the split-track
  mis-association is still open — and PASS counts on that path are still judged by
  eye + tracker-id evidence, as in the fourth sitting.
- **Teensy-uptime tracking lag (open)** — all timing-sensitive measurements
  (achieved flight time, catch error) are only meaningful with a fresh
  can-bridge boot; `uptime_ms` is logged alongside every session artefact.
- **Firmware kind-1 time-budget race at short flights (found in Phase 1's
  control analysis, 2026-07-25).** The tracker-driven catch arm arrives
  mid-throw-decel; the Teensy windup budget silently drops it (Serial-only,
  lying success ack) for flight ≲0.6 s at realistic pipeline latencies.
  Hardware T1/T2 should treat **0.7 s as the flight floor** (the plan's
  0.6 s start is marginal — clean only for pipeline delay ≤20 ms). The sim
  gate does not model the firmware budget; Phase 2's report annotates the
  hardware-marginal band. See the Phase 1 logbook entry.

## Notes for Collaborators
- Rung evidence and design bases live in the merged ladder logbook entries
  (2026-06-24 → 2026-07-04) and `plans/active/bb-online-juggle-tilt-rearchitecture.md`;
  that plan remains the authority for the two-ball frontier (Rung 3), which is
  **out of scope here** (MVP Phase 9 re-plans it against this plan's evidence).
- The offline juggle demo (`sim/juggle_demo.py`, CasADi optimiser) is paused,
  not deleted; nothing in this plan depends on it.
- MPC is untouched: the toss runs entirely on the trajectory stack.
