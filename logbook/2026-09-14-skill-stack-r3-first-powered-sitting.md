---
title: "R3 first powered sitting — every throw was caught but announced NO_LANDING (the announcer, not the tracker, was starved), 'leg 6' was the hand, two MAX_DEVIATION latches with different causes, and a +25% release-speed overshoot the learner's box cannot reach"
type: investigation
date: 2026-09-14
status: in-progress
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/docs/choreography.md
  - sim/skills_gate.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_reload_coordinator_node.py
  - tests/motion/test_skills_executor.py
  - tests/hardware/session_skills_r3.md
subsystem:
  - ros
  - motion
  - can
  - tracking
tags:
  - safety
  - testing
  - performance
---

# R3 first powered sitting

## Summary

The R3 hardware gate's first powered sitting (2026-09-13 evening) threw nothing
onto the gate: every single-throw attempt was physically caught by the
operator but the node reported `NO_LANDING`, the five-throw chain caught two
balls with 14–23% too-long flights before aborting, a `MAX_DEVIATION` guard
latch fired twice for two different reasons, and reload from Ball Butler could
not run because its provider service was deleted at R1 and never re-wired.
Two of six root causes were reached only after withdrawing an initial
hypothesis. Six fixes landed 2026-09-14 (Jetson-side, no flash); six items are
filed, not fixed, including a firmware contract gap and an owner-level
operating-point decision on the release-speed overshoot. The R3 hardware gate
is still outstanding — nothing in this session flew a clean sitting against
the runsheet's pre-registered gates.

## Context

Runsheet: `tests/hardware/session_skills_r3.md` (sim triple filled, Findings A
and B resolved per `logbook/2026-09-13-skill-stack-r3-learner-single-site.md`).
Bag: `/home/jetson/Desktop/rosbags/2026-09-13_22-57-18/`. Launch log (tee'd):
`temp/logs/launch_r2gate_20260913_2257.log` (473 lines, ROS-epoch timestamps
1789304241..654). `plant_id r3-20260913`. Three parallel investigation agents
worked the bag and log against a shared context file (guard/latch, reload,
tracker/catch); this entry synthesises their reports.

## Timeline

All times ROS epoch seconds, `1789304xxx`.

- 4313 — `skills/check`: ladder `REFUSED NOT_LEVELLED`; box `REFUSED` (swept at
  `leg_vel` 300, live limit 1000). Operator levelled and set limits.
- 4436–4537 — five single self-tosses (`n_throws=1`), each: pre-level, REST
  1.625 s, THROW 0.825 s, then `END NO_LANDING: the tracker has no landing for
  ball 0 by the deadline` / `OUTCOME ball 0: no landing estimate was ever
  observed — no row`. Operator reports all five physically caught. No
  `/throw_announcements` published for any of the five (the bag holds only 3,
  all from the five-throw run below).
- 4566 — `n_throws=5` run: THROW skill 1 fresh origin; catches spliced but
  most re-sends refused (`SPLICE_TOO_LATE`, `LIMIT_JERK` 152877/199999/355440,
  `LIMIT_ACC` 5346, `LIMIT_VEL` 366); two memory rows appended
  (`caught=False`): landing (−3, 4) mm / flight 1.053 s and (−10, 31) mm /
  flight 0.975 s, vs commanded flight 0.857 s. 4571.365 `END
  ABORTED_NO_RELEASE`.
- 4572.13 — **Latch 1**: `teensy_bridge`: "Teensy guard FAULT LATCHED:
  fault_state=MAX_DEVIATION (leg 6 first to cross, dev=+2.506 rev at trip)
  live_dev=[6 zeros]"; `trajectory_node` guard descent to measured
  x=−49.6 y=−2.7 z=170.0.
- 4585.3 — cleared; hold seeded at measured (−49.6, −2.8, 170).
- 4591.9 — Reload attempt 1: `REJECTED_WRONG_MODE` (mode was STANDBY). 4598 —
  operator selects trajectory mode.
- 4601.3 — Reload attempt 2: possession `UNKNOWN [SENSOR_NO_LANDING]`;
  `smooth_move_hand service unavailable` ×4 prime dispatch failures → ×4 safe-
  abort retract dispatch failures (≈30 s) → `Reload ABORTED_PRIME_FAILED`.
- 4643.3 — operator restarts `n_throws=5`: pre-level; REST skill 0 fresh
  origin from rest (seed at rest, platform 0 mm/s, hand 0.065 rev/s), install
  plan 49.1 ms.
- 4644.33 — **Latch 2**: `MAX_DEVIATION (leg 6 first to cross, dev=−2.920 rev
  at trip)`; descent to measured x=−20.0 y=−0.3 z=170.0. `END GUARD_LATCHED at
  skill 1 (THROW)`.

## Investigation

### Thread 1 — every throw was caught, every throw reported NO_LANDING

`ball_tracker_node`'s `/balls` carried a CONFIRMED track with a populated
830 mm landing for all five single-throw windows regardless (balls
60/71/72/73/75) — the tracker itself was never the problem. The break is one
level up: `motion/skills/segments.py::_plan_throw` never sets `release_t_s`,
so `trajectory_node` answers a plain THROW with `t_release_mono = 0.0`;
`skill_node._maybe_announce` guards on that field and returns early on `0.0`
→ no `ThrowAnnouncement` published → `_correlation[ball_id]` never created →
`_tracker()` returns `None` forever → the executor refuses `NO_LANDING` at
the deadline. Only a CATCH-with-throw (which does set `release_t_s`)
announces — matching the bag's 3 announcements, all from the five-throw run.
The mocked unit test had been overriding `t_release_mono=50.0` for a THROW, a
value the node never actually produces on this path (mocked-ROS blindness).

### Thread 2 — "leg 6" is the hand, and the two latches have different mechanisms

Firmware source pins the axis: `fault_machine.cpp:437-448` sets
`md_leg = HAND_AXIS` for a hand-lane deviation trip;
`canbridge_config.h:96` `HAND_AXIS = NodeId::JUGGLEBOT_HAND` (= 6). Legs are
0..5 (`fault_machine.cpp:462`, `i < NUM_LEGS`), so the six-entry `live_dev`
array is legs 0..5 only and says nothing about the hand — the trip message's
"leg 6" wording is a firmware defect. Threshold:
`MAX_DEVIATION_HAND_REV = 2.5` (`canbridge_config.h:252`, 79.1 mm); leg
threshold is 1.0 rev. Residual = raw pre-clamp interpolated command minus
age-extrapolated feedback (`leg_interp.cpp:773-779`).

**Latch 1 command-vs-measured (trip snapshot, `/link_status` KeyValues, 10 Hz):**

| field | value |
|---|---|
| trip time | 1789304572.134 |
| dev (rev) | +2.5065 |
| u0 (commanded, rev) | 5.8732 |
| encoder (rev) | 3.3667 |

Mechanism: the last accepted catch-with-throw plan kept streaming past
`END ABORTED_NO_RELEASE` (571.365) — two more full hand strokes ran
afterward: 0.3↔9.6 rev, four strokes in 1.7 s (~0.4–0.5 s apart, far faster
than the 1.44 s beat). The trip stroke ramped `vel_ff` 0 → 127.7 rev/s in
37 ms (≈3450 rev/s², at the session ceiling `hand_acc_limit_rps2` 3500,
`config/hardware_config.yaml:1071`); `iq_meas` saturated at 47.82 A; peak
`vel_meas` 161.4 rev/s. The immediately preceding identical-amplitude stroke
(570.836) ran at 2865 rev/s² / 19.9 A and tracked to 0.23 rev. No mechanical
stop was hit (measured span 0.079–9.967 rev of a 10.701 rev stroke).

**Latch 2 command-vs-measured (trip snapshot):**

| field | value |
|---|---|
| trip time | 1789304644.332 |
| dev (rev) | −2.9198 |
| u0 (commanded, rev) | 5.1887 |
| encoder (rev) | 8.1086 |

Mechanism: from 572.127 to 643.63 the wire hand command was frozen at 9.426
rev while the hand physically drooped to 8.665 (0.76 rev sag against
gravity). At 643.666 the hand lane re-activates
(`leg_interp.cpp:857` arms `s_hand_recover_slewing`, base = encoder 8.665);
the emitted command ramps at `RECOVER_SLEW_ACCEL_RPS2` 5.0 to the
`RECOVER_SLEW_VEL_RPS` 1.0 cap (`canbridge_config.h:370-371`). Meanwhile the
raw REST command descends 9.426 → park over 1.65 s (peak ≈9 rev/s). The guard
residual is computed at `leg_interp.cpp:773`, **before** the slew overwrite at
`:941-967` — it sees the raw plan against the encoder, not the slewed emitted
command. `dev` crosses −2.5 at ≈0.62 s; trip at 0.71 s, with encoder
8.1086 = 8.665 − 1.0×0.573 (exactly the 1.0 rev/s slew cap × elapsed time).
The clamped wire value never reaches the wire either — the slew overwrites
it, so the 100 Hz echo shows a clean ramp and hides the fault entirely.

Both latches are real MAX_DEVIATION trips on the same axis, but they are not
the same defect: Latch 1 is the actuator physically unable to track a command
this session's acceleration ceiling allows (the electrical/kinematic limit);
Latch 2 is the guard flagging a gap that the bridge's own recovery-slew
logic created by design.

### Thread 3 — reload from Ball Butler

`smooth_move_hand`'s provider (`teensy_bridge_node._svc_smooth_move_hand`)
was deleted at R1 (`1e2c0c9`, 2026-09-11 18:06:03) along with
`set_hand_traj_cmd`/`set_hand_source`, per the R1 comment at
`teensy_bridge_node.py:1826-1833`: "the hand is now the 7th lane of the
streamed Setpoint, with no RPC in the loop and no latch to switch." Nothing
in `jugglebot_launch.py` provides `smooth_move_hand` any more
(`git grep create_service hand` → only `set_hand_state`/`set_hand_gains`
remain). `reload_coordinator_node.py` (`create_client` at :2227,
`_smooth_move_hand` at :10378, prime at :10440, retract at :10463) and
`catch_coordinator_node.py:356` both still hold clients for it — neither was
updated at R1 (the grep-before-refactor gap). Reload's `REJECTED_WRONG_MODE`
gate and `REJECTED_NOT_CENTERED` gate both worked as designed; the sequence
died in `_step_checking` only after the mode was corrected, on the deleted
service.

## Discussion

**(a) Withdrawn hypothesis — "the tracker never saw the ball."** The first
reading of `NO_LANDING` across all five single throws was that the tracker
had lost the balls. It had not: `/balls` carried a CONFIRMED, landing-
populated track for every one of the five windows. The correlation layer in
`skill_node`, not the tracker, was starved — `_maybe_announce` never fired
because `_plan_throw` never populates `release_t_s` for a plain THROW. This
matters for where future sessions look first: a `NO_LANDING` refusal on this
path is now known to implicate the announcer before the tracker.

**(b) Reframe — "leg 6" is the hand axis.** The guard message's own wording
("leg 6 first to cross") reads as a leg fault, and the six-entry `live_dev`
array (all zeros both times) reinforces that reading by omission — it only
covers legs 0..5 and says nothing about axis 6. Both trips were, in fact, the
hand. This is fixed in this session (Fix item 3) rather than filed, because
the message actively misleads an operator into debugging the wrong six
actuators.

**(c) Why the two latches have different causes, and why Latch 2's cause is
filed rather than fixed.** Latch 1 is a genuine tracking failure: the
commanded acceleration (3450 rev/s², at the session ceiling) pushed the motor
into current saturation (47.82 A) and the hand fell 2.5 rev behind a command
it physically could not follow — the guard did its job. Latch 2 is different
in kind: the guard measures the **raw pre-slew plan** against the encoder,
but the bridge's own recovery-slew logic (armed after a hand-lane
reactivation) deliberately limits what it actually emits to ≤1 rev/s. The
gap the guard trips on is a gap the bridge itself created by design, not a
tracking failure of the actuator. Fixing this means changing what the
firmware's residual is measured against (post-slew emitted command) or
suppressing/counting the guard while `s_hand_recover_slewing` is armed — a
firmware change requiring a flash, and an invariant change ("the guard never
measures a gap the bridge introduced") that deserves its own decision rather
than a same-session patch. It is filed, not fixed, pending an owner flash
decision.

**(d) The +25% release-speed overshoot, and why the learner's box cannot
reach it — an owner decision, explicitly requested.** The chained run's two
completed throws were both fast and long. Announced release speed was
|v|=4.166 m/s (127.9 rev/s at the hand); the measured ball apex (2206 mm
world, i.e. 1.38 m above the 830 mm catch plane) implies an actual release
speed of 5.20 m/s (159.5 rev/s) — a 25% overshoot — and the hand's own
measured peak, 161 rev/s at 4572.047, matches that figure, not the announced
one. Flight time came out 14–23% long: 0.975 s and 1.053 s measured against a
commanded 0.857 s. The coherent story ties directly to Thread 2 Latch 1: the
stroke saturates current, the hand lags the commanded profile, and the
position loop then catches the plan up at whatever speed closes the gap —
that catch-up speed, not the planned release speed, is what launches the
ball. The learner's admissible flight window is 0.750–0.857 s (§3 of the R3
sim entry); centering a systematically +23% flight would need a target
around u≈0.70 s, which is outside that box entirely — the learner cannot
correct this by choosing a different command, because no command in its
admissible range produces the observed physics. Two candidate directions
exist and neither is exercised yet: lower the session's hand acceleration
ceiling so the stroke never saturates (the 2865 rev/s² stroke immediately
before the trip tracked cleanly to 0.23 rev, a existence proof that a lower
ceiling tracks); or accept the faster release and re-sweep the admissible box
at the new operating point (which likely means the catch apex has to drop,
echoing R2's note that "the hand binds at 0.9 m"). This is an operating-point
decision for the owner, not a code fix — and per the working-agreement on
hardware investigations: if your physical intuition about the hand's
current/acceleration margin at this ceiling disagrees with this framing,
that is load-bearing signal — say so before either direction is chosen.

**(e) Tradeoff accepted in Unit D — refuse the retired reload rather than
port it.** `reload_coordinator_node._execute_reload` now refuses every goal
up front with `REJECTED_RELOAD_RETIRED_R1(...)` instead of restoring
`smooth_move_hand` or rebuilding its prime/retract semantics against the
streamed hand lane. This matches the plan's own § 1 item 7 ("the Ball
Butler reload's reactive catch is deleted with the stroke engine and
operator placement is R3's reset") and the recorded R4 direction (reload
returns as a CATCH skill: `bb/reload` + `bb/throw_at_target` at the current
site plus the executor's standalone CATCH from the announcement). Porting the
old FSM-era prime/retract machinery onto the streamed lane now would be
throwaway work against a design that is superseded at R4; refusing cleanly
buys an honest error message today at the cost of reload staying unusable
until R4 lands its replacement.

## Fix

Landed 2026-09-14 (Jetson-side; no firmware flash):

1. **`skill_node.py`** — a THROW's announcement now derives from
   `t_event_mono` (the release *is* its own event) instead of gating on
   `t_release_mono`; tests made wire-faithful (`t_release_mono=0.0` for a
   THROW, matching what the node actually produces). Fixes Thread 1.
2. **`skill_node.py`** — `blas_threads.check_blas_threads()` runs at
   start-up (runsheet row 13 was previously unsatisfiable — nothing checked
   it at the point the runsheet claimed).
3. **`teensy_bridge_node.py`** — the `MAX_DEVIATION` message now names the
   tripped axis via `_axis_label` (e.g. "hand first to cross") instead of
   always reading "leg N"; test added. Fixes Thread 2(b).
4. **`motion/skills/executor.py`** (Unit A) — an ended attempt (abort or
   `skills/stop`) with a future release still streaming installs one
   `trajectory/hold`, so a killed attempt cannot keep driving strokes past
   its own end (the mechanism behind Latch 1). The pending-instant check is
   kept on the wire's perf-counter clock — the implementer's first cut
   compared it against the ROS tick clock and would never have fired, caught
   in main-session review before landing. Four tests added.
5. **`motion/skills/executor.py` + `skill_node.py`** (Unit B) — new
   `Observations.hand_at_park` (`|meas − JB_OP_HAND_RETRACT_REV| ≤
   HOMING_HAND_PARK_BAND_REV` = 0.5 rev) plumbed through
   `precondition_refusals(..., fresh_origin, skip_mocap)`; the opening skill
   (index 0, non-CATCH) is now gated on a fresh-origin REST too.
   `REJECTED_HAND_NOT_PARKED` now covers tracking error OR off-park, with the
   recovery verb (DEACTIVATE then ACTIVATE re-parks through the bridge's own
   slew) stated in `skills/check`'s refusal text. `INVARIANTS.md` row
   updated; `sim/skills_gate.py` passes `hand_at_park=True`. Four tests
   added.
6. **`reload_coordinator_node.py`** (Unit D) — `_execute_reload` refuses
   every goal up front with `REJECTED_RELOAD_RETIRED_R1(...)`; three tests of
   the now-unreachable FSM body skipped with reason.
   Fixes Thread 3 per the accepted tradeoff (d). (`ros_ws/docs/choreography.md`
   was regenerated in the same unit, but for Unit A: it picks up `skill_node`
   as a new `trajectory/hold` client; `reload_coordinator_node`'s own hold
   client is live and unrelated.)

## Catch-cleanliness signals (bag evidence, 8 catches)

Extracted from `/balls` (≈5 ms) and `/hand_telemetry` (≈10 ms) around each of
the 8 physically-caught balls (5 singles + 3 of the 5-throw chain). Ball
z-velocity at the 830 mm plane crossing, hand axis z-velocity at that same
instant, time from crossing to the first SEATED (`ball_held_raw &
ball_held_valid`) edge within 1 s, bounce edges in the 300 ms after SEATED,
and peak `|iq|` / peak `|vel_meas − vel_ff_cmd|` within ±150 ms of crossing:

| catch | ball | ball vz (m/s) | hand vz (mm/s) | Δt cross→SEATED | bounce edges | peak \|iq\| (A) | peak \|vel dev\| (mm/s) |
|---|---|---|---|---|---|---|---|
| single_0 | 60 | −5.12 | 2.0 | no SEATED within 1 s | — | 2.00 | 12.3 |
| single_1 | 71 | −4.57 | 6.6 | 0.081 s | 0 | 2.43 | 39.5 |
| single_2 | 72 | −5.01 | 1.5 | 0.067 s | 0 | 21.03 | 396.2 |
| single_3 | 73 | −4.30 | −2.2 | 0.023 s | 2 | 1.96 | 120.9 |
| single_4 | 75 | −4.75 | 1.4 | 0.286 s | 0 | 1.84 | 30.7 |
| multi (76) | 76 | −3.77 | 6.6 | no SEATED within 1 s | — | 19.89 | 582.3 |
| multi (77) | 77 | −4.26 | −8.8 | 0.092 s | 1 | 34.77 | 806.6 |
| multi (79) | 79 | −4.52 | −0.4 | no SEATED within 1 s | — | 30.07 | 828.3 |

3 of 8 physically-caught balls (single_0, multi 76, multi 79) never produced
a SEATED edge within 1 s of the plane crossing despite the operator reporting
a catch. Clean singles run peak `|iq|` ≈2 A and peak velocity deviation
12–40 mm/s; single_2 and all three chained catches run 19–35 A and
400–830 mm/s — an order of magnitude rougher, consistent with the chained
run's tracking-lag/catch-up dynamic in Discussion (d). Both memory rows from
the chained run were labelled `caught=False` even though ball 77 produced a
SEATED edge 0.092 s after crossing — filed below. True closing velocity
(platform task-space, not `/hand_telemetry`) was not extracted this session.

## Verification

- (2026-09-14, `pytest tests/ros/test_skill_node.py
  tests/motion/test_skills_executor.py tests/ros/test_reload_coordinator_node.py
  tests/ros/test_teensy_bridge_node_read.py -q`, 249 passed, 3 skipped)
- Full gate (2026-09-14, `./run_tests.sh --full`): **PASS** — parallel phase 5949 passed, 9 skipped, 1 xfailed in 281.12s, serial phase 6 passed. A first run failed ONE test, `tests/motion/test_unified_cycle_splice.py::test_splice_at_the_terminal_knot_is_extend_bit_for_bit` (`assert 0.0 < 0.0` on `plan_wall_s`): traced to `tests/ros/test_install_segment.py:251`, whose `monkeypatch.setattr(time, 'perf_counter', …)` INSIDE `_frozen_perf()` snapshotted the frozen lambda as the original and restored it after the context manager had put the real clock back — the frozen clock then leaked to every later test on that xdist worker (wall times read 0.0). Fixed by setting the attribute directly (the context manager's exit restores the real clock); pair run `pytest tests/ros/test_install_segment.py tests/motion/test_unified_cycle_splice.py -q` 27 passed; the second full run is the PASS above.

## Filed / Follow-ups

1. **Firmware: the hand deviation residual is measured against the raw plan,
   not the emitted (post-slew) command** — Thread 2 / Discussion (c). Guard
   never measures a gap the bridge itself created. Fix candidates: residual
   against emitted `h_pos` in `leg_interp.cpp`, or suppress/count while
   `s_hand_recover_slewing` is armed. Requires a flash; owner decision.
2. **`trajectory_node`'s guard descent collapses the legs onto measured state
   but leaves the hand command frozen** — the root cause of the 58 s stall
   against gravity between Latch 1 and Latch 2.
3. **Hand acceleration ceiling (3500 rev/s²) saturates the motor and drives
   the +25% release-speed overshoot** — Discussion (d); operating-point
   decision (owner) plus an admissible-box re-sweep once decided.
4. **The possession SEATED label misses real catches** — 3 of 8 bag-confirmed
   catches produced no SEATED edge within 1 s, and both chained-run memory
   rows read `caught=False` despite at least one (ball 77) SEATED 0.092 s
   after crossing. The learner uses the outcome `y` regardless of this label,
   but the label feeds the gate's pass/fail count. `CAUGHT_WINDOW_S = 0.15`'s
   anchor needs a look.
5. **Fresh-origin detection is `idx == 0 and kind != CATCH`** — a
   mid-schedule fresh-origin REST (none occurred in tonight's schedules)
   would not be gated by Fix item 5's precondition.
6. **Reload as a CATCH skill (R4 build item)** is what "reload from Ball
   Butler while away from home" actually needs — `bb/reload` +
   `bb/throw_at_target` at the current site plus the executor's standalone
   CATCH from the announcement. Not started; Discussion (e).
7. **`catch_coordinator_node.py:356,858` keeps the same dead
   `smooth_move_hand` client** (the second of Thread 3's "two coordinators");
   untouched here — the node is on R4's `fsm-final` deletion list and its
   prime path is not reachable from the skill stack.

## Status

in-progress — the R3 hardware gate is still outstanding. Nothing in this
sitting flew a clean run against the runsheet's pre-registered gates; a
second powered sitting is needed once Filed items 1 and 3 have owner
decisions (or are explicitly deferred past the next sitting).
