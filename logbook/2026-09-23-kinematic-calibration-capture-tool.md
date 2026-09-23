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
