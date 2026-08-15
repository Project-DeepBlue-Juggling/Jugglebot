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
disciplines (~~can-bridge Teensy reboot before each sitting~~ — retired
2026-08-15, FW 14 fixed the uptime lag; `uptime_ms` logged
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
   from hand telemetry), no reload/toss already active. Ball evidence **IS**
   gated by default since 2026-08-10 (`toss_require_ball_evidence: true`): the
   node reads the hand ball sensor LIVE, so a valid-empty cup is
   `REJECTED_NO_BALL` and a sensor that cannot answer is `REJECTED_BALL_UNKNOWN`
   (it refuses — see `ros_ws/docs/ball_possession_contract.md` § 3.3). The
   pre-2026-08-10 reading of this step — *"NOT gated by default
   (`toss_require_ball_evidence: false` — no ball-in-cup sensor; the operator
   guarantees the ball)"* — is what the config `false` now restores as the
   operator's escape hatch.
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
| 5 | Hardware bring-up T0–T4 (operator-run) | staged PASS criteria | RUNBOOK READY (`tests/hardware/session_phase8_toss_hardware.md`, 2026-07-25); T4 **VALIDATED 2026-07-27** (11/11 displaced throws accepted at the then-70 mm cap); T0–T3 captures pending |
| **E** | **Displaced throws to ±150 mm** — throw site from the live pose, reach envelope re-scoped (**C-REACH-1**), cap re-based, stay-at-pose on CAUGHT | full pytest + the re-run 8b gate + the § SECTION DISP ladder | **LANDED 2026-07-29 (code complete, NOT YET FLOWN)** — see § Phase E Outcome. Ships with one **known limitation**: chaining is refused at the cap (works below ~146 mm) |
| **F** | **`TossContinuous` — repeated toss-catch cycles with a configurable dwell** (the programme finale, operator decision (c)): new action + pure-Python session FSM, `stop_on_miss` default TRUE, per-cycle accounting | full pytest + the CS-1..CS-6 trace checker (synthetic matrix clean) + the § SECTION CONT ladder | **LANDED 2026-07-29 (code complete, NOT YET RUN)** — see § Phase F Outcome |

## Implementation Phases

### Phase 0 — Post-merge reconciliation
The 2026-07-24 merge of `demo/bb-led-two-ball-juggle` lands the ladder
primitives, probes, tests, and logbook history in the working branch. Full
suite green is the phase gate. Doc pointers: `sim/JUGGLE_DEMO.md` gains a
banner distinguishing the (paused) offline demo from the online ladder;
`plans/archived/2026-08-15 bb-led-two-ball-juggle-demo.md` § status gains a merged-location
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

> **Phase E (2026-07-29) supersedes three of Phase 4's shipped decisions.** The
> throw site A is no longer config (`JB_OP_TOSS_THROW_SITE_MM` is retired — A is the
> platform's live commanded pose); the 70 mm cap is now the config key
> `toss_max_displacement_mm` at 150 mm; and the "raising the 80 mm reach envelope or
> moving its center to B is a safety-critical `trajectory_node` change not taken this
> run" item below **was taken**, as contract C-REACH-1 — the centre moved to B, the
> radius did not move. Decisions (b) and (d) below are the ones that stand.

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
  to 100 mm there), never +y/NW (fails at 70 mm); otherwise stage T4 at ≤70 mm.
  > **REFUTED 2026-07-29 — do NOT inherit the "+y/NW fails" guidance above.** The
  > 2026-07-25 directional map it rests on was produced by a seeded sim bug (every
  > trial in a cell ran the same seed, so each cell was ONE trial replicated). The
  > re-run at the current machine (`python sim/toss_gate.py --tier 8b
  > --trials-per-point 10 --seed 0`, run 2026-07-29, `temp/reports/toss_8b_phaseE_seed0.json`)
  > measures **+y at 10/10 (70 mm), 10/10 (100 mm), 9/10 (150 mm)** and **NW at
  > 10/10 at every radius**. The only sub-9/10 cell anywhere is **SW at the
  > 150 mm cap (8/10)**. Phase E therefore ships a simple scalar cap with no
  > direction-awareness; the one watch-direction lives in the runbook ladder.
  Set
  `JB_OP_TOSS_TIER='8b'` for the session (**amended 2026-08-10: no longer the
  shipped default — `toss_tier` went back to `"8a"` on 2026-08-10 by owner
  decision after the tilt-map validation and an 8a toss retest, so this flip is
  now required, not a no-op; see
  `logbook/2026-08-10-tilt-cal-c0-blockers-level-noise-and-leg0-spinout.md`
  § *Arc wrap-up*.** It was the shipped default from
  2026-07-28; `JB_OP_TOSS_THROW_SITE_MM` is retired — Phase E throws from the live
  pose, so pre-position with `go_to_pose` instead). PASS: ≥3/5.
  **Superseded for anything past 70 mm** by Phase E's § SECTION DISP ladder.
- Session disciplines: can-bridge Teensy reboot pre-session; `uptime_ms` logged;
  `/diagnose --latest` after every session; one truthful outcome line per toss.

### Phase E — displaced throws to ±150 mm (operator decision (d), 2026-07-28)

The operator's goal, stated 2026-07-28: *throw across large translations — at
least ±150 mm at z = 170 — from oblique platform positions, with the platform
STAYING at its catch pose after a caught toss so sessions chain A → B → C.* Four
sub-changes, one phase, because they interlock: three of them are individually
inert and only the fourth makes any of them usable.

**1. The throw site A is the platform's LIVE commanded pose.** Sourced from a new
`trajectory/commanded_position` topic (`geometry_msgs/Point`, 5 Hz off
trajectory_node's status timer — deliberately not a `jugglebot_interfaces` type, so
this needs no interface-package rebuild). `jugglebot_operational.toss_throw_site_mm`
is **retired** and `compute_release_state_tilted`'s `throw_site_xy_mm` became a
REQUIRED argument — one enforcement point for *A is never defaulted*. No fresh read
⇒ `REJECTED_POSE_UNKNOWN`, fail-closed, because a guessed A is not merely a wrong
number: POSITIONING would obediently translate the platform to it before throwing.
A stale read is nonetheless not a correctness bug — A is *nominated*, and the
platform is commanded to the pre-tilt pose derived from it, so the throw site is
self-consistent by construction.

**2. The reach envelope is re-scoped, not weakened — contract C-REACH-1**
(`ros_ws/docs/catch_reach_envelope.md`). The envelope bounds *unrequested* drift;
centring it on the pose held at arming silently also capped *requested* reach, so a
deliberate A→B toss past 80 mm was rejected `WORKSPACE` **mid-flight, ball
airborne** — 4/4 on hardware, bag `2026-07-27_16-07-30`. It now centres on the
**declared catch point B** (`catch/reach_center`, published one FSM tick before the
arm raise, consumed-and-cleared by the next `arm_catch` call of any kind). Drift
stays bounded at 80 mm about B; requested displacement is gated pre-throw by its
own contract. The RELOAD path declares nothing, so its centre is still the
commanded pose and its choreography is byte-identical.

*Mechanism chosen over the two alternatives, by failure mode.* Centring on the
first accepted post-arm target needs no wire change and was the plan's lean — but
on the reload path the first target can be a corrupt tracker landing estimate (18
corrupt reload tracks in the 2026-07-27 capture; `reload_sequencer` records 27–46
envelope `WORKSPACE` rejects per flight in the 2026-07-23 session), and under that
mechanism the corruption would *define its own envelope*. Widening the envelope to
~250 mm was rejected outright: it loosens the drift bound by exactly the amount it
buys reach, which is the coupling this contract exists to break.

**3. The cap is config-keyed and re-based on its own evidence.**
`jugglebot_operational.toss_max_displacement_mm` = **150 mm** (was a hard-coded
70). The old 70 was *the 80 mm envelope ∩ the Rung-2a clean box*; C-REACH-1 removed
the envelope half, so the cap needed evidence rather than an inheritance.
`tools/probes/displaced_reach_frontier.py` (2026-07-29) measures the production
planner's real A→B frontier over 8 directions: **125 mm at T = 0.55 s, 175 mm at
0.60, ~225 mm from 0.70 s up**, and every direction that stays inside the ±150 box
is accepted from every extremity throw site at T ≥ 0.70. So the closed-form quintic
bound in `toss_sequencer` is **conservative below T ≈ 0.75 s and optimistic above
it**, and 150 mm keeps a ~1.5× margin on the real frontier while keeping that
optimism harmless. The cap is a simple scalar, not direction-aware: the 2026-07-25
directional map is stale vintage (pre-2026-07-26 moving-rim machine) and the
re-run map is characterisation, so direction guidance lives in the runbook ladder
where an operator can act on it.

**4. A CAUGHT toss STAYS at its catch pose.**
`jugglebot_operational.toss_stay_at_pose_on_caught` = `true`; the terminal is the
RECENTER ladder minus `go_home` — no new mechanism, no new setpoint, just not
calling `go_home` and letting the emitter's terminal hold do what it already does.
That is what makes A → B → C chain, because the next goal reads its throw site from
the live pose. Every NOT-caught terminal keeps `SAFE_ABORT`'s retract + latch-lower
+ `go_home` unchanged: a miss leaves a loose ball and possibly a hand at the top of
its stroke, and chaining is exactly what the miss already ended.

**The seam this opens, closed loudly.** The RELOAD catch is hard-fixed at the
workspace centre and the reload never pre-positions, so a reload commanded from an
off-centre park would arm an envelope centred off (0,0) and reject the incoming BB
ball mid-flight. `reload_sequencer` now refuses `REJECTED_NOT_CENTERED` in CHECKING
— before `ACTION_PRIME_HAND`, so nothing moves and BB is never asked. Refusal
rather than an auto-return: an auto-`go_home` would inject new commanded motion into
the shipping reload choreography, which no hardware session has run.

**Hardware ladder:** `tests/hardware/session_anomaly_fixes.md` § SECTION DISP —
DISP-1 (degenerate B == A at centre, never flown) → DISP-2 (degenerate at
`(140, −140)`, never flown) → DISP-3 (70 mm, parity with the validated T4) → DISP-4
(100 mm) → DISP-5 (150 mm) → DISP-6 (the chained session) → DISP-7 (the reload
refusal). Each rung carries numeric PASS/ABORT and names what the sim map does not
cover.

**Known residual, instrumented not designed away.** Under STAY the platform holds
the catch's RECEIVE TILT (up to ~3.6° at the cap) indefinitely rather than returning
to level. The catch already holds that tilt through its quiescent settle with the
ball in the cup, so this extends a hardware-observed state — but it extends it
without bound, which the machine has never done. Runbook row **DISP-5.6** is a
REPORT row that measures it, and the escape hatch is one config flip.

### Phase E — Outcome (landed 2026-07-29)

**Commits:** `ad80ff0` (the phase), `38a33cf` (the sim/toss_gate.py asymmetry-map
seed bugfix, its own logical unit).

**Tests:** `python -m pytest tests/ -q`, run 2026-07-29 on the Jetson in the project
venv: **4140 passed, 3 xfailed, 196 warnings in 1409.60 s (0:23:29)**, exit 0.
Baseline at HEAD `956a4b8` was 4096 passed / 3 xfailed; xfail count unchanged, and
the net +41 is accounted for in the logbook entry's Verification section.

**Reviewer findings adjudicated at finalize.** Two independent lenses (physics,
contract) converged on the same defect and it is real — see the KNOWN LIMITATION
below. A third (regression) surfaced a PROVEN safety-relevant gap in the new reload
gate, fixed in this phase: `REJECTED_NOT_CENTERED`'s tolerance was the bare 80 mm
envelope radius, leaving **zero** budget for the reload pre-tilt's own centroid
swing. `compute_catch_orientation` clamps at `MAX_TILT_DEG = 12°` for every real BB
arrival (18–40° off vertical), so that swing **saturates** at
`64.78·sin(12°) = 13.47 mm` on every reload — measured invariant across 18/25/30/40°
arrivals. Parks in `(66.5, 80] mm` were therefore ADMITTED by the gate and then
rejected `WORKSPACE` by the envelope, *after* `ACTION_SEND_THROW`, with BB's
countdown started and the ball unsavable — precisely the mid-flight rejection the
gate was added to make impossible. The tolerance is now
`envelope − hand_catch_offset·sin(MAX_TILT_DEG)` = 66.53 mm, derived rather than
magic, pinned by
`test_centered_tolerance_leaves_room_for_the_reload_pretilt_swing`. It costs nothing
in normal operation (a `go_home`'d platform reads 0 mm) and a refusal moves nothing.

**⚠ KNOWN LIMITATION — chaining is refused AT the cap.** The phase's headline
capability works below ~146 mm and is refused at 150 mm, in every direction. The
catch deliberately parks the platform **centroid** outside `B` so the **cup** lands
**on** `B` (`centroid = landing − hand_catch_offset·platform_z`). Measured through
the production chain at `B = (−150, 0, 170)`, `T = 0.80 s`: the cup ends at exactly
`(−150.00, 0)`, the centroid at `(−153.10, 0)`. `trajectory/commanded_position`
publishes the **centroid**, so the next goal reads `A = −153.10` and both surviving
gates — the `±150 mm` planning box on `A` and the `150 mm` cap on `|B − A|` — are
applied to a value 3.10 mm outside nominal. The offset is
`hand_catch_offset·sin(receive tilt)` = 2.07 % of displacement, so it crosses the
box at `|B| ≈ 147 mm`. Every refusal is loud, pre-throw and moves nothing; the
remedy is one `go_home`.
**UPDATE 2026-08-14 — chaining refusal DISSOLVED by the box/cap separation**:
the planning box is now the config key `toss_workspace_xy_mm` (default 160 =
cap + 10 > cap × 1.03, i.e. above the 2.07 % divergence at the cap edge), so the
parked centroid sits inside the box and chained goals at the cap are ADMITTED;
back-to-centre from a cap-edge park still refuses on the `|B − A|` cap, which is
a genuinely requested displacement. Pinned by
`tests/ros/test_toss_sequencer.py::test_chaining_at_the_cap_box_dissolves_the_frame_divergence`,
documented as C-REACH-1 residual 7 (updated in place). The frame decision below
is therefore no longer *forced* by DISP-5/DISP-6 — those rungs run as written at
the shipped box — but the centroid-vs-cup question itself stays open.

**Open question the operator owns — the throw-site frame (centroid vs cup).**
`trajectory/commanded_position` publishes the commanded centroid;
`compute_release_state_tilted` documents `throw_site_xy_mm` as the **cup**/release
xy. They agree exactly whenever the platform is level and diverge by the cup swing
when it is not — which is exactly the state `STAY` now creates. The aim stays
self-consistent (A is nominated, and POSITIONING makes it true), so this is not an
aim error; it is a **gate-frame** error. Three candidate fixes, none takeable
unilaterally: (a) publish or derive the **cup** xy — the root-cause fix, but it needs
the commanded *rotation*, which the phase deliberately kept off the wire because a
cup xy derived from the corrected rotation re-opens C-LEVEL-1's double-correction
hazard for a consumer that feeds it back as a request (the toss does); (b) widen the
A-side box and the cap by one cup-swing allowance — but the uncertainty is
bidirectional, so this loosens an evidence-based cap; (c) lower the cap to ~145 mm —
which violates the operator's stated "at least ±150 mm" ask. **Decide before the
DISP sitting**, because DISP-5 and DISP-6 are written against the limitation.

**Deferred operator handoff.** The whole § SECTION DISP ladder (DISP-0 … DISP-7)
is unflown. Prerequisites: `colcon build --packages-select jugglebot` **and a
relaunch** — the launch runs the INSTALLED copy, so until the rebuild the robot
executes the 70 mm cap, the config throw site and the `go_home` CAUGHT terminal
while the repo says otherwise (row DISP-0 greps the installed copy for exactly
this). **No firmware flash and no `jugglebot_interfaces` rebuild** for this phase:
both new topics carry `geometry_msgs/Point`, and no sketch reads either new
generated constant.

### Phase F — `TossContinuous`: repeated toss-catch cycles (operator decision (c), 2026-07-28)

The operator's ask: `toss_continuous {catch_position, throw_height_m, num_throws,
dwell_time_s}` — repeated toss-catch cycles with a configurable dwell, as the
bridge between validated single tosses and 2-ball juggling. `stop_on_miss`
defaults **TRUE**.

**It adds no capability, and that is the design.** Every cycle is an ordinary
`Toss`, built by `_build_toss_cycle` and ticked by `_run_toss_cycle` — the same
two methods the single `Toss` now uses, extracted so there is exactly one copy of
the cycle machinery. A session cannot drift from the toss the hardware ladder
validated, because there is nothing to drift from. What the outer FSM
(`toss_session.py`, pure Python, no ROS) owns is exactly three things: *when* the
next cycle starts, *whether* it starts, and the per-cycle accounting a sitting is
scored from.

**The found fact it rests on.** The firmware catch stroke ENDS at 0 rev
(`Trajectory.h` `buildCatch`, `xA = {x3, x5, x6, 0.f}`, line 267) and a kind-0
throw stroke STARTS at 0 rev (`buildThrow`, `xA = {0.f, x1, x2, x3}`, line 251):
the catch is its own re-park and the throw is its own catch-prime, so **a caught
ball needs no hand move between cycles**. Corroborated on the 2026-07-27 sitting —
hand `pos_meas` within ±0.045 rev of park at the CAUGHT instant on all 17
self-tosses, worst excursion 0.069 rev over the following 3 s against the ±0.5 rev
park band (7.2×). The platform side is free because Phase E's `ACTION_STAY` leaves
the machine at its catch pose and `trajectory/commanded_position` reports it.

**Five session invariants**, stated in the module docstring and pinned by tests:
S1 at most one live cycle; S2 the session commands no motion of its own; S3
`stop_on_miss` stops at the cycle boundary and introduces no new abort point; S4
cancellation obeys the per-cycle phase rules verbatim; S5 the dwell is a quiescent
wait, never a stretched `throw_delay`.

**The dwell floor is DERIVED, and the brief's 2.0 s figure is unachievable.**
Dwell is previous SCHEDULED LANDING → next RELEASE, so
`cycle_start(N+1) = landing(N) + dwell − throw_delay` and the floor is
`throw_delay + margin`. `throw_delay` cannot go below `MIN_TOSS_THROW_DELAY_S` =
3.5 s (the toss FSM's own `REJECTED_CANT_MAKE_LEAD`; verified against the real FSM
— 3.49 s rejects, 3.50 s dispatches), and the margin covers the handoff the
machine cannot avoid: the CAUGHT verdict lands at `landing + 0.202–0.442 s`
(median 0.209; 17/17 self-tosses, `logbook/2026-07-28-caught-gate-xy-plausibility.md`)
plus two 50 ms node ticks = 0.542 s worst measured, so 0.6 s ships. **Absolute
floor 4.10 s; 5.60 s at the 5.0 s default delay; config default dwell 6.0 s.** A
2.0 s dwell would need `throw_delay ≤ 1.458 s`, i.e. 2.042 s below the FSM floor.
Lowering that floor is a change to the arming/release timing of a
hardware-validated path — a safety fork, deliberately not taken; it is the lever a
future phase pulls, and it needs a bench measurement of the real
positioning + prepare budget, not an argument.

**S5's choice, by failure mode.** Stretching `throw_delay` to absorb the dwell
satisfies the same arithmetic, and was rejected because it leaves `catch/armed`
RAISED for the whole dwell with a ball resting in the cup — `catch_coordinator`'s
reactive catch path live over a loaded cup for the entire gap, so any tracked ball
entering the volume can command platform motion; because an armed dwell looks
identical to an about-to-throw machine, so the operator's intervention window is
one in which the robot is armed; and because it moves every cycle's internal
timing off the profile the hardware measured. Waiting keeps each cycle
bit-identical to a validated single toss.

**Phase E's KNOWN LIMITATION becomes a pre-throw refusal.** The catch parks the
platform CENTROID a cup-swing outside `B` so the CUP lands ON `B`, and the wire
publishes the centroid — so near the ±150 mm planning box a chained session would
throw one ball, catch it, and then refuse cycle 2 `REJECTED_WORKSPACE` with the
platform parked off-box and the ball in the cup. `_predicted_chain_site_mm` runs
the SAME `predicted_catch_command` policy the deferred reach publishes from and
refuses `REJECTED_CHAIN_UNREACHABLE` before anything moves. Measured frontier:
**|B| ≤ 146.5 mm chains, |B| ≥ 147.0 mm does not**, and the residual then
COLLAPSES (cycle 2 `149.017`, cycle 3 `145.938`, cycle 4 `146.001` at B = 146) —
so **a fixed-B chain converges and cycle 2 is the only one at risk**. Note the
binding gate is the ±150 box on `A`, not the 150 mm displacement cap: the residual
|B−A| never exceeds 3.1 mm.

**Possession across the dwell is stated, not designed away.** Nothing re-verifies
that the ball is still in the cup between catch and next throw: C-POSSESS-1's
verdict is minted at ARRIVAL (the tracker declares CAUGHT because the marker
vanished), so a post-CAUGHT bounce-out during a dwell leaves the latch set and the
next cycle fires an empty stroke — benign, but the verdict is wrong — and
`stop_on_miss` does not close it because the bounce-out happens *after* a CAUGHT.
A session multiplies that exposure by `num_throws`. The seam that closes it needs
no wire change: the coordinator already routes every possession question through
`_possession_confirmed` → `_possession_source`, and the ball-in-cup hand sensor
becomes the PRIMARY source there. Recorded in
`ros_ws/docs/ball_possession_contract.md` § 7.

**v1 is ONE catch point for the whole session.** A per-cycle waypoint list is
explicitly out of scope and is the obvious v2: it needs Phase E's throw-site frame
question settled first, and a fixed B is the well-conditioned case (measured, it
converges).

**Hardware ladder:** `tests/hardware/session_anomaly_fixes.md` § SECTION CONT —
CONT-0 (pre-flight) → CONT-STEP-0 (the zero-code operator dispatch-loop baseline,
which produces the comparison data) → CONT-STEP-1 (the no-ball DRY TRACE, 3
cycles, `stop_on_miss: false`, scored by the new CS-1..CS-6 trace invariants) →
CONT-2/3 (3 then 5 cycles at 0.60 m) → CONT-4 (5 at 0.80 m, **gated on
`platform_fw_version = 2`** — unattended repetition multiplies exposure to the
end-stop overshoot the Phase-D decel fix removes) → CONT-5 (a chained displaced
session at 70 mm).

### Phase F — Outcome (landed 2026-07-29)

**Code complete, nothing has run.** Action:
`ros_ws/src/jugglebot_interfaces/action/TossContinuous.action`. FSM:
`ros_ws/src/jugglebot/jugglebot/toss_session.py`. Node: the third action server on
`reload_coordinator_node`, plus the `_build_toss_cycle` / `_run_toss_cycle`
extraction. Config: `jugglebot_operational.toss_session_dwell_default_s` (6.0),
`_dwell_margin_s` (0.6), `_max_throws` (20). Trace invariants: **CS-1..CS-6** in
`tests/hardware/toss_trace_recorder.py check --continuous`, verified both
directions by `tools/probes/toss_trace_synth.py` — **two** happy cases, one per
session terminal shape (`cont_happy`, every cycle CAUGHT / `ACTION_STAY`;
`cont_dry`, every cycle MISSED / `SAFE_ABORT`, which is the shape the mandatory
CONT-STEP-1 pre-flight produces), plus one violation trace per invariant.

**A real defect the tests caught before it shipped.** The session's `success` was
computed from the counts alone, so a `num_throws = 0` goal satisfied
`0 == 0 and 0 == 0` **vacuously** and the node called `goal_handle.succeed()` on a
REJECTED goal. Fixed at the class level rather than the case: `success` now also
requires the `COMPLETED` terminal, so no terminal that is not a clean completion
can report success however the counters land.

**A second, pre-existing defect surfaced and fixed.** `tools/probes/toss_trace_synth.py`'s
`viol_dt7` case had gone **vacuous** — DT-7 stopped being an "exactly one
dynamic_target" count when it was reworked to bound the pre-position STREAM to
30 mm of the nominated point, so injecting a duplicate AT (0, 0, 170) was no
longer a violation. The verification matrix reported `viol_dt7 PASS/MISMATCH` at
HEAD (confirmed by running the HEAD checker against the HEAD probe). The case now
injects a target that WANDERS to (95, 0, 170), which is what DT-7 actually guards.
The full 30-case matrix is clean. It is NOT split into its own commit, though the
repo's logical-unit rule would normally earn it one: it lives in the same `CASES`
table and the same generator this phase rewrote, so the two are not independently
revertible, and a clean matrix is a *prerequisite* for trusting the new CS cases.

**Ten reviewer findings verified and fixed at finalize; four adjudicated
otherwise.** The two that mattered most converged from independent lenses on the
same defect: **the CS-1..CS-6 trace checker this phase ships as the bench gate had
only ever been validated against an all-CAUGHT session, while runbook CONT-STEP-1
makes an all-MISSED dry trace the MANDATORY capture before any ball flies.** That
ladder disarms BEFORE it retracts, so the cycle's own mandated retract — ~10 rev on
a dry trace, since no catch stroke re-parks the hand — lands inside the dwell
window CS-3 measures. Building the missing `cont_dry` case found two real defects
in sequence: CS-3's hand bound was DT-5's *idle-window* 0.05 rev, which is 1.12x
the worst real post-disarm residual (0.0446 rev, measured over the 16 toss-shaped
gaps of `temp/logs/toss_trace_2026-07-27_15-39-50.jsonl`) — a coin-flip false STOP
on a hard-STOP row; and the first repair, anchoring on the first in-park sample,
still FAILed `cont_dry` because the 0.5 rev park band is wide enough that the tail
of the retract ramp is inside it. Landed formulation is **no-ascent with a running
minimum** at a measured 0.15 rev (3.4x the worst real ascent, 66x below the ~9.96
rev auto-prime it exists to catch), plus a new failure for a window where the hand
never reaches park; validated three ways (`cont_dry` 6/6 PASS, the new
`viol_cs3_hand` FAILs CS-3 with zero collateral, 16/16 real windows pass).

**The extraction regressed the single Toss, in the direction nobody expected.**
`_execute_toss`'s `rclpy`-shutdown branch terminalised NOTHING on the goal handle
at HEAD; after the extraction the discarded `exit_kind` let it fall through to
`goal_handle.abort()`. The *session* handled shutdown correctly and its test
asserted parity with "the single Toss does the same" — which the same change had
made false, and the pre-existing shutdown test never asserted the handle. Fixed,
and the invariant is now pinned on the single Toss.

**The dwell floor sized only the handoff that commands nothing.**
`dwell_margin_s = 0.6` covers the CAUGHT handoff (a verdict, two ticks). A MISSED
cycle the session continues past hands over through a whole `SAFE_ABORT` ladder
whose every rung returns on a SERVICE ACK — `_go_home()` returns when a 2.0 s
recentre profile has been *installed*. At the shipped defaults the naive
arithmetic already starts cycle N+1 **1.7 s before the recentre lands**. Landed
fix: `DEFAULT_SESSION_MISS_CLEANUP_S = 2.80 s` as a FLOOR on landing -> next cycle
start after any non-success cycle — it can only lengthen a gap, never shorten one,
so it makes the docstring's existing "lateness is absorbed" claim true by
construction. Also landed: `REJECTED_THROW_DELAY`, without which the advertised
4.10 s absolute floor was an arithmetic identity rather than a floor;
`catch/reach_center` removed from CS-3's silence set (it is published in the same
FSM tick as the window's own end boundary, 131 us apart on the real bag, and CS-4
already covers the whole gap more strictly); CS-6's docstring corrected to what it
proves; and `ball_possession_contract.md` § 7.1's "no edit to the coordinator"
upgrade claim refuted and replaced with the two edits retention actually needs.

**Deferred, with the trace recorded**: the unprotected pre-`try` region of
`_execute_toss_continuous` (structurally real, reachability NOT-PROVEN by both
reviewers who raised it, pre-existing in `_execute_toss`, and the safe restructure
needs `session is None` guards on two handlers — the wrong trade at finalize).

**Deployment: `colcon build --packages-select jugglebot_interfaces jugglebot` +
relaunch — BOTH packages, mandatorily.** This is the first phase since Phase E to
need the interfaces build, and its failure mode is worse than the msg-field cases
the runbook already documents: `reload_coordinator_node` imports `TossContinuous`
at module scope, so a stale interfaces package raises `ImportError` before the
node is constructed and **all three** ball-op actions disappear — `Reload` and
`Toss` included. Runbook row CONT-0.3 is the 3-second check. **No firmware flash**
(config regeneration added three `constexpr` to the delivered
`hardware_config.h`; no sketch reads them). **The two-package build is now
mandatory in EVERY section of the runbook, not just § SECTION CONT**, because a
`jugglebot`-only build now takes `Reload` and `Toss` down along with the session.
The review found four per-section blocks affirmatively saying "no
`jugglebot_interfaces` rebuild"; underneath them **every** build instruction in
the file was the single-package command. All 20 were swept —
`grep -c 'packages-select jugglebot\b'` now returns 0.

**Verification.** `python tools/probes/toss_trace_synth.py --all --verify`, run
2026-07-29: **30/30 cases OK, matrix CLEAN** (RED at HEAD on `viol_dt7`). Full
suite: `pytest tests/ -q`, run 2026-07-29 on the Jetson in the project venv (07:47:07 -> 08:11:09): **4254 passed, 3 xfailed, 198 warnings in 1435.48 s (0:23:55)**, exit 0 (+114 on the `3332bc6` baseline of 4140 — accounted EXACTLY: `--collect-only` reports 70 tests in `test_toss_session.py` and 44 in `test_toss_continuous_node.py`, both new files; xfail unchanged at 3, so no test was weakened to reach green). Commits: **`5e8db9a`** (the single logical unit: the action, the FSM, the node extraction, the finalize fixes, the instrument, and the narrative docs). Logbook:
`logbook/2026-07-25-toss-continuous-action.md`.

**Operator handoff, deferred in full.** Nothing in this phase has run on hardware.
`tests/hardware/session_anomaly_fixes.md` § SECTION CONT is the authority and runs
LAST — after § SECTION POSS (whose verdicts it consumes) and § SECTION DISP (whose
`STAY` terminal makes chaining possible). CONT-0 pre-flight → CONT-STEP-0
(zero-code dispatch-loop baseline; **if it comes back 1-of-3 caught, abort the
section and go fix the single toss** — a session cannot be more reliable than the
toss it repeats) → CONT-STEP-1 (the no-ball dry trace, the one sanctioned use of
`stop_on_miss: false`) → the live ladder CONT-2..CONT-5, with CONT-4 gated on
`platform_fw_version = 2`.

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
- **ERR_TIMEOUT epidemic (CLOSED 2026-08-09, FW 10)** — retained as a note: the
  telemetry ladder mitigation stays as defense-in-depth against host-side
  RpcTimeout.
- **Tracker verdict corruption (open, Phase-7) — PARTLY RETIRED 2026-07-28.**
  Split by path since contract **C-POSSESS-1**
  (`ros_ws/docs/ball_possession_contract.md`): **self-toss** verdicts are now
  correct (17/17 on the 2026-07-27 capture, where the shipped gate scored 0/17),
  so a self-toss `MISSED` on a watched catch is a **finding**, not expected noise.
  **Reload** verdicts still read `MISSED` on a real catch — the split-track
  mis-association is still open — and PASS counts on that path are still judged by
  eye + tracker-id evidence, as in the fourth sitting.
- **Teensy-uptime tracking lag (CLOSED 2026-08-15)** — root cause was the vendored
  FlexCAN_T4 `_available` RX-ring leak; FW 14 fixed it and validation at 5.8 h and
  15.2 h of continuous uptime holds the lag at 10–20 ms with zero lead-clamp duty
  (`logbook/2026-08-15-fw14-validated-arc-closed.md`). Timing-sensitive
  measurements no longer require a fresh can-bridge boot. `uptime_ms` is still
  logged alongside every session artefact — it is now the regression detector, not
  a caveat — and `/link_status` carries an alarmed `latency_monitor` row.
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
