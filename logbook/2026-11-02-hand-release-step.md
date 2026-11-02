---
title: "C-HAND-5: the Hand's acceleration steps at the release, so the launch and the brake run near the constant-acceleration ideal (Apex 1.05 peak 3484 -> 2362 rev/s^2; Apex 1.42 refused -> planned at 3138); not flown"
type: feature
date: 2026-11-02
status: in-progress
phase: "JB-114 R1 (the planner's own brake)"
subsystem: motion
related_entries:
  - 2026-11-01-sitting-debrief.md
  - 2026-11-01-hand-limit-4500-and-stroke-floor.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cycle_plan.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/motion/test_release_step.py
  - tools/probes/release_step_shape.py
  - tools/probes/hand_stall_stop.py
  - config/generated/admissible_box.yaml
  - tests/motion/test_validate_cycle.py
  - tests/motion/test_unified_cycle.py
  - tests/motion/test_unified_cycle_splice.py
  - tests/motion/test_skills_segments.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_pose_planner.py
  - tests/motion/test_validate_cycle_budget.py
  - tests/sim/test_skills_gate.py
  - logbook/INDEX.md
---

# C-HAND-5: the release step

**Summary.** The cup QP pinned `acc == g` at every release knot (the CasADi
transcription, kept for parity), so the cup's acceleration was continuous
through the release. On the 25 ms knot grid that forces the launch to ramp from
+A to -g over its last knot and the brake to ramp from -g to -B over its first.
Both peaks ran well above the constant-acceleration ideal `v^2 / 2d` (kappa):
launch 1.81, brake 1.56-1.68 at Apex 1.05. The 4500 rev/s^2 hand limit then
capped the Apex near 1.3, while the motor still had current to spare.

Now the release may step: the launch keeps its acceleration up to the release,
and the next window starts from a brake seed recorded on the release mark.
`validate_cycle` admits the step at that knot only. Owner request 2026-11-01
("progress with improving the planner's braking profile"; current limits up to
80 A bursts are fine). This is JB-114's R1. The bumper is pinned for later as a
safety stop.

## Background: R0 (2026-10-30, read-only study)

R0 was an Opus probe study kept in a session scratchpad. Its conclusions are
folded in here so they survive the reboot.

- The cause is the release pin plus the knot grid. It is not the time budget
  (SETTLE tail, Dwell and holds were each varied: kappa unchanged), not the
  objective (only +5-9 %), not the jerk rows below Apex 1.3, and not the planner
  mode.
- A grid LP with a step allowed reaches kappa 0.97-1.00 on both sides.
- The C2 tolerance (`HAND_C2_TOL_MARGIN`, C2FF 2026-09-14) was written against
  off-knot stream-sampling artefacts (299-679 rev/s^2 steps), not a planned
  release.
- The legacy firmware Throw had exactly this step (constant ascent, then constant
  decel) and tracked at eta 0.95.
- No entry ever chose the front-loaded brake.

## Discussion

**Hypothesis reframed mid-implementation: a vertical step was wrong on a tilted
cup.**
- The first build seeded the next window with `(0, 0, -B)`. On the hop throws
  (tilted release) the gate refused `LIMIT_JERK` at 199-203k mm/s^3 against a
  150-200k cap, 31 ms after the release.
- The slider runs along the cup axis, so a vertical cup step on a tilted cup has
  a sideways part, and only the Platform can make it.
- The shipped form:
  - The release rows pin the DIRECTION of the cup's specific force, `(acc - g)`
    along the take-off direction `u`. That direction is the cup axis the release
    tilt is pinned to (`_throw_tilt_for`).
  - Its size is held at or above the contact floor (`release_seated`).
  - The seed is `a_R = a_L + s*u`.
- The step is then a pure slider step, the Platform's acceleration stays
  continuous, and `a_R - g` is along `u`, so the post-release detach cone already
  holds at knot 0. After the fix, every LIMIT_JERK failure in the planner files
  was gone.

**The seed is a number on the release mark, not a QP variable.**
- A free knot-0 acceleration would let the next window's QP choose the brake
  directly. But it changes the program shape of every post-release window and of
  the pose planner's augmentation.
- An analytic seed changes one boundary value, and the QP continues from it with
  continuous acceleration. Recording it on `ReleaseMark.brake_accel_mm_s2` when
  the throw is planned means one number reaches every consumer: the SETTLE tail,
  the next segment's window, an interior splice on the release knot (all through
  `release_state_at_knot`), and the gate (`extend` declares the seam knot).
- Sizing:
  - `B = 1.10 * v_z^2 / 2d` to the box top. Factor 1.0 / 1.05 / 1.10 / 1.15 gave
    a 3252 / 3172 / 3138 / 3196 rev/s^2 plan peak at Apex 1.42; at 1.10 the
    launch, not the brake, binds.
  - Clamped to `[2 g, A*u_z]` with `A = min(hand limit, firmware stream STOP
    3500)`: the cap is the slider's (a vertical brake B is a slider deceleration
    B / u_z; the audit caught a vertical-frame cap, inactive on today's
    near-vertical self-throws). Below 1 g the cup would carry the ball on. A
    plan braking harder than the stall stop would outrun it.

**Accepted: the launch stays at kappa ~1.3.**
- The mean-square-acceleration objective spreads the launch: its peak is
  mid-launch and it ends at +1441 rev/s^2 at Apex 1.05.
- A flatter launch needs a peak-minimising objective or a per-throw acceleration
  box, more design than this unit, and 1.3 already plans Apex 1.5.

**The gate's exemption is declared, not inferred.**
- `CyclePlan.release_step_knots` names the knots, carried through `extend`, the
  head, the tail, the gate view and the bump slice. A path that drops it makes
  the gate refuse, the safe direction.
- At a declared knot the step must be downward (an upward step would drive the
  cup into the ball). Every other knot keeps the exact C2 tolerance.
- The gate's contact window still closes at `n - 1`. On a joined plan the release
  knot's right-hand acceleration is the brake, far below the floor, so the seated
  bound at the release is a QP row (`release_seated`), not a gate window.

**Firmware read (FW 35; no change needed for the step itself).**
- The U2b torque feedforward is LIVE: `hand_torque_ff_gain` 0.7 in
  `jugglebot_launch.py`.
- It evaluates `a_cmd` analytically per Hermite span (`leg_interp.cpp` Mode 1),
  so the feedforward steps exactly on the knot boundary: about -0.21 N m at Apex
  1.42, inside the 0.234 N m clamp.
- The scheduled stall stop takes its entry state from the span's own analytic
  `(p, v, a)` (`sched_advance` / `span_eval`), not the frame's `accel` field.
- Frame handovers compare old and new curves at the same instant on the same
  span, so they see no step.
- The LEGACY hand ladder (Mode 1/2/3, non-scheduled frames) is different. Its
  Mode-2 Taylor extrapolation runs from the frame's `accel` field and a
  finite-difference jerk estimate, both of which straddle a step.
  - `tests/sim/test_skills_gate.py::test_the_hand_lane_decays_on_the_falling_edge`
    cuts at the knot of peak hand speed, which is now the release, and the
    extrapolated velocity reversed (not monotone).
  - A scheduled hand group supersedes that ladder (`leg_interp.cpp:1298`), and
    skill plans stream scheduled frames (the K = 0.7 feedforward is only active
    then). So the ladder is unreachable while every frame carries the full
    scheduled knot set; a `HAS_SCHED` downgrade (`setpoint_pump.py` clears it
    when any of u1/u2/v1/v2 or the knot stamp is missing) falls back to it.
    That case is recorded on JB-145.
  - The test now pins the ladder's wind-down on a continuous-release plan
    (`throw_plan_continuous`).

**Found, not caused: the stall stop cannot fit near a release.**
- `tools/probes/hand_stall_stop.py` models the scheduled lane exhausting its
  cover at each knot around a release: the last span is continued 20 ms (grace),
  then the C2 stop runs at 3500 rev/s^2 / 350k rev/s^3.
- With TODAY's continuous release, exhaustion one knot before the release ends
  the stop at 15.5 rev (Apex 1.05) and 20.0 rev (Apex 1.42), against the 10.501
  clip and the 10.701 hard stop.
- With the step, the worst knot moves to the release itself: 14.6 and 18.5 rev,
  slightly better.
- A clamped entry acceleration and a 4500 stop still leave 12.8 rev, because the
  grace alone travels about 3 rev at 150 rev/s and the planned brake already uses
  the whole stroke above the release.
- This needs at least three missed frames (more than 70 ms; the worst measured
  emit gap is 47 ms). It is filed as JB-145. The bumper (JB-114) is the
  physical backstop.

## Verification

- New contract tests (`tests/motion/test_release_step.py`, 12 tests): the kappa
  pins at Apex 1.05, Apex 1.42 planned stepped and refused continuous, the axial
  step on a tilted throw, the declared / undeclared / upward gate cases, the knot
  shift, and the seed clamps.
- Offline reach (release 860, hand 4500, `tools/probes/release_step_shape.py`):

  | Apex | Peak continuous | Peak stepped | Launch / brake kappa (stepped, STEADY) |
  |---|---|---|---|
  | 1.05 | 3484 | 2362 | 1.30 / 1.10 |
  | 1.20 | 3956 | 2686 | 1.28 / 1.14 (factor 1.05 run) |
  | 1.42 | refused (4859) | 3138 | 1.28 / 1.11 |
  | 1.50 | refused (UNVERIFIED) | 3310 (factor 1.05 run) | – |

  Leg peaks are unchanged on the same-site cell.
- Admissible box re-sweep (2026-11-02 11:49-14:17,
  `WT=<worktree> OUT=temp/sweeps/hstep bash sweep_hstep.sh`):
  - The runner is the h4500 runner (`temp/sweeps/sweep_h4500.sh`, 26 families,
    hand 4500, BLAS 1) plus three pose Columns families at Apex 1.00-1.40
    (`temp/sweeps/hstep/`, logs kept for JB-143).
  - Rounds a and b were byte-identical (CMP_STATUS 0). The box went from 82 to
    100 cells at gate_hash 252300b435d8, re-stamped 1749cca8bc23 by a one-round verify after the audit fixes (all 29 families identical).
  - Unchanged: 50 cells, including all 38 pose plain-Columns, turned, high-low
    and oval cells, bit-identical (pinned by
    `test_release_step_resweep_kept_the_pose_cells_and_added_columns_to_apex_1_40`).
  - Added: all 18 high Columns cells are admissible.
  - Moved (not pinned): 32 cells. The Hop cells in BOTH planner modes (8 pose,
    8 level) moved their landing rectangles and Apex floors (0.648 to 0.578),
    the selection rule JB-143 replaces; the LEVEL planner's self_toss (2),
    Columns (4) and oval (10) cells.
  - Level oval at Dwell 0.45 lost its 0.95 and 1.10 cells to LIMIT_JERK (885
    refusals). The stepped release holds the launch's specific force along a
    tilted cross-Site take-off direction, which the level planner's Platform
    has to translate for. Level oval at Dwell 0.50 / 0.55 narrowed its commanded
    Apex floor (0.61 to 0.95 / 0.86).
  - The pose planner (the default) keeps its Columns, oval, turned and
    high-low cells bit-identical; only its Hop cells moved. The level planner is
    a fallback; its oval narrowing is accepted and recorded here.
- Full tier (2026-11-02, `./run_tests.sh --full`, on the final tree with the
  re-stamped box): **7981 passed, 55 skipped (parallel, 391 s) + 8 passed
  (serial, 34 s), RESULT PASS**.
- Contract tests (2026-11-02,
  `pytest -q tests/motion/test_release_step.py`): **13 passed**.

## Hardware

Not flown. The first Sitting is a Columns Apex ladder (Sep. 125, 0.95 -> 1.05
-> 1.15 -> 1.25 -> 1.35, 20 Throws each); its `## Sitting` section is on JB-144
(the release-step Feature), with stop rules on Hand iq over 54 A, MAX_DEVIATION
or a stream STOP, LANE_REFUSED and any HAND_LIMIT END. The box sweep adds pose
Columns cells at Apex 1.00-1.40 for it. The Learner's Memory was built on the
continuous shape, so expect a landing bias on the first Throws.
