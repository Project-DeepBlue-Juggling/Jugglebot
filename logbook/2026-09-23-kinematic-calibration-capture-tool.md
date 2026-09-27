---
title: "Kinematic calibration capture tool: generated tilted sweep, production-gate dry-run, no-motion rehearsal, frame sanity check"
type: feature
date: 2026-09-23
status: in-progress
phase: "kinematic-calibration — § 6 step 2"
related_plan: kinematic-calibration.md
files_changed:
  - tests/hardware/kincal_capture.py
  - tests/sim/test_kincal_capture.py
  - plans/active/kinematic-calibration.md
subsystem:
  - motion
  - tracking
tags:
  - kinematics
  - hardware-test
---

# Kinematic calibration: capture tool

**What.** `tests/hardware/kincal_capture.py` drives the plan's § 5 sweep through
`trajectory/go_to_pose` and writes the CSV that `tools/kincal_fit.py` reads. Each dwell
records:
- the mocap Platform pose (Base frame)
- the six leg encoder revolutions
- the command, alongside

It is request-only, like `tilt_cal_grid.py`: it never arms, changes mode, sets limits or
commands the hand, and the operator does the re-home at a prompt. Seed 1 generates:
- 125 main poses over five z-levels (100–250 mm), every one tilted 6–10°, edge-weighted
  to about 0.65 of each pose's own reachable radius
- 10 hold-outs
- 10 § 3 repeat groups, each approached twice from each of two opposite sides through a
  30 mm via pose
- a 10-pose § 4 re-home subset, visited before and after the re-home

That is 185 dwells in 18.8 min. `--check` is the per-session homing check: 8 poses in
0.7 min.

**Why these choices.**
- *The dry-run runs `planner.build_move`, the same gate `go_to_pose` runs, on every move,
  and reports every refusal at once.* The moves are requested at lean 0: lean shapes only
  the transit, so the node builds exactly the plan the dry-run checked. The slow
  durations (≥ 3 s, 60 mm/s, 4°/s) keep the legs far from the unshaped-traverse latch.
- *z is shifted back into the Base frame.* `mocap_node` publishes the Platform body's z
  minus `GEOM_INITIAL_HEIGHT_MM` (574.3). The tool adds that value back and records it in
  the CSV header, because the calibration will change it.
- *A frame sanity check at the first dwell,* which is tilted. It checks mocap position
  against the command (25 mm), attitude (2°) and encoder revolutions against the config
  IK (0.3 rev). It aborts on:
  - a missing z shift
  - a transposed or wrong-order quaternion
  - a leg sign error

  Each of these would otherwise surface only as a nonsense fit.
- *A dwell is recorded only if its window is still,* meaning mocap spread ≤ 0.5 mm and
  encoder spread ≤ 0.003 rev. The tool retries up to three windows, then skips the pose
  and names it in the meta, rather than writing a moving sample.

## Verification

- `pytest tests/sim/test_kincal_capture.py -q` (2026-09-23): **23 passed in 4.94 s**. The
  end-to-end test flies the generated sweep on a synthetic robot with a known geometry
  error, through the tool's own CSV writer, into `kincal_fit.analyse`. Result: § 8 PASS,
  hold-out < 0.3 mm RMS against > 3 mm under the config geometry, repeats STATIC,
  re-home PASS. A 2.5 mm re-home error reads FAIL. Each of the four frame faults is
  caught.
- `python tests/hardware/kincal_capture.py --dry-run` (venv, 2026-09-23): **0
  refusal(s)**, 185 dwells, 18.8 min; with `--check`, 0 refusals.
- `--rehearse --no-gate` under system python3.8 with ROS sourced and the stack down
  (2026-09-23): the ROS layer imported and subscribed, and all six preflight problems
  were reported together. **Not yet rehearsed against a live stack**; that comes before
  the sitting, on the loaded Jetson.

## 2026-09-27: two fixes from the first attempt at the robot

The rehearsal passed, then the capture aborted after the first move each time with
`no hold 5 s after the planned end (plan_kind=move)`. `trajectory/status` never
reports a finished move as `hold`. The finished plan stays `move`, and
`plan_time_remaining_s` falls to 0; that pair is the node's own in-flight test
(`trajectory_node._active_move_in_flight`). The tool now takes a move as finished when
a status published after the planned end reports it done (`move_finished`). A test
fails if the node's in-flight test ever changes. Separately, one attempt's preflight
judged `/link_status` and `/robot_state` missing after a fixed 1 s listen. It now
listens until every stream has arrived, up to the timeout. The rehearsal could not
see either problem: the first needs a move, and the second is discovery timing.

- `pytest tests/sim/test_kincal_capture.py -q` (2026-09-27): **29 passed in 3.03 s**.
- Gate `./run_tests.sh` (2026-09-27 13:06, log
  `temp/logs/kincal_arrivalfix_gate_1306.log`): **5408 passed, 8 skipped in
  377.94 s**; serial tail 3 passed.

## 2026-09-27: the first two sweeps; three more fixes

Two sweeps ran. **Run 1** (`kincal_sweep_20260927_130935`) stalled. At 13:12:22 mocap
and all six encoders froze, 20 mm short of z100_11, with no fault raised. The command
kept advancing through two more poses until the accumulated deviation reached 1 rev.
The Teensy guard then latched MAX_DEVIATION at 13:12:37 (leg 2 +1.03 rev;
`live_dev` [+0.03, −0.40, +1.03, +0.56, −0.02, +0.88]). The tool recorded three
frozen dwells, because mocap and encoders agree with each other and no check compared
them with the command. The bag (`~/Desktop/rosbags/2026-09-27_13-08-04`) shows the
cause, and it was **not a jam**. During the stall all six legs stayed in closed loop
with no errors, at gravity-hold iq of about 2 A, and the telemetry stayed live. At
13:12:20.62 `/link_status` `sched_stops` went 162 → 163, and `sched_refused` then
climbed from 50 at about 40/s, the full stream rate, while `setpoints_sent` kept
advancing. That is the can-bridge scheduler's designed latched hold (`leg_interp.cpp`
`sched_advance` / `sched_apply`):
1. A stream gap exhausted the knot cover mid-move.
2. The scheduler stopped and held.
3. The resumed stream did not join the stopped curve within the resume tolerance.
4. Every frame after that was refused, and the legs kept holding.

The only signal is the counter. `skill_node` watches it for the hand lane
(2026-09-18), but nothing watches it on the `go_to_pose` path, so it ran silent until
the guard's deviation check tripped 17 s later. What caused the gap (a Jetson stall
or UDP loss) is not established. `robot_state_stale_skips` ticked in the same
seconds.
**Run 2** (`…_131410`) recorded 97 dwells through z212, then go_to_pose refused
z250_23 with WORKSPACE (leg 1, 275.0 mm). The dry-run had passed that pose because
the node tilts every request by the C-LEVEL-1 levelling correction, about 0.8° here
(`_pose_from_msg`), and the dry-run did not. Leg 1 moves from 272.4 mm to 275.2 mm.

Fixes in `kincal_capture.py`:
- **Reach allows 1.5° of correction in any direction** (`LEVEL_ALLOWANCE_DEG`, an
  8-direction ring). The regenerated seed-1 sweep still has 185 dwells.
- **Every dwell is checked against the command** (`tracking_problems`, 0.15 rev).
  Good dwells read ≤ 0.055 rev and run 1's frozen dwells read 0.33–0.78 rev. On
  failure the capture aborts without recording the dwell.
- **WORKSPACE and UNREACHABLE refusals skip the pose; any other refusal aborts.** A
  refused via also skips its dwell. More than 10 refusals abort.
- **`sched_refused` rising during a move aborts at once** (`sched_refused_problem`,
  compared with the value when the move was accepted). The operator sees the cause,
  not a deviation 15 s later.

**Carried, not fixed here:** the leg lane's latched hold is silent to
`trajectory_node`. That belongs to the skill-stack arc and is surfaced to the owner.

The preview fit on run 2's 97 rows is **NOT PASSED**, and the reason is in the fit
tool, not the data. The freeze decision reads posterior sd scaled by √χ². A misfit
(χ² 27) therefore inflates every sd about 5×: 22 parameters froze, all six L0
included, and freezing them worsens the misfit. Fitting with nothing frozen gives a
leg residual of 0.37 mm RMS (χ² 1.7) and a hold-out of 1.39 mm RMS / 1.86 mm max,
against 11.9 mm under the config geometry. Separately, L0 and base-node z trade
against each other (L0 +8..+11 mm, base z −4..−10 mm). The fix is deferred to a
separate change agreed with the owner; see the plan.

- `pytest tests/sim/test_kincal_capture.py -q` (2026-09-27): **42 passed in 39.09 s**.
- `python tests/hardware/kincal_capture.py --dry-run` (2026-09-27): **0 refusal(s)**,
  185 dwells, 18.7 min.
