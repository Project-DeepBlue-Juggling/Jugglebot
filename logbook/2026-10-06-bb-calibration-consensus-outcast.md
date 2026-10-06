---
title: BB calibration — one drifting marker no longer refuses the calibration; the axis is the consensus of at least 5 of 7 markers and the outcast is named
type: bugfix
date: 2026-10-06
status: resolved
phase: two-ball-skill-stack — R5 (operator surface)
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - tests/ros/test_bb_calibration_consensus.py
  - tests/ros/test_perception_console_lines.py
subsystem:
  - mocap
  - ball-butler
---

# BB calibration: consensus axis, named outcast

## Symptom

The 2026-10-06 20:19 sitting refused 7 of 9 BB calibrations ("Circle centres deviate up to
3.15–4.55 mm from axis", gate 3.0 mm); the 2026-10-05 22:27 launch refused 4 of 5. The
owner got a pass by raising BB's hand, to keep the held ball away from the base markers.

## Diagnosis (from `/mocap_data` in both bags — `/bb/markers` is not recorded)

Replaying the solver on all 14 windows: **every rejection was QTM `Ball Butler - 1`** (one of
the two markers added 2026-09-27). Its centre deviation fell steadily through each session
(4.56 → 1.06 mm on 10-06; 4.02 → 2.44 on 10-05) while the other six stayed within 2.3 mm.
At the identical parked pose (yaw 0°), M1's distance to M4 read 72.9 → 72.5 → 72.0 → 70.3 mm
over the sitting, then held; during motion it read ~70.2 throughout. No unlabelled marker
came within ~120 mm of M1. **The hand raise did not cause the pass**: M1's trend runs
smoothly through it (3.31, 3.21, 2.68), attempt 9 passed with the hand at 0 mm, and 10-05's
fifth attempt passed with the hand never raised. Physical cause open (mount creep / warm-up
are the candidates; owner to inspect the M1 mount).

With M1 out of the fit, BB's position agreed within 0.8 mm across all 9 attempts: the
refusals guarded nothing.

## Discussion

- **Hand-raise step at the start of calibration — rejected**: the data does not support it (above).
- **Tighten the gate to ≤ 2 mm — rejected**: with the outcast removed, the healthy six still
  spread 1.29–2.30 mm (pose-dependent QTM error of ~1 mm, levered up by 80–120° arcs); 2.0 mm
  refuses 9 of the 14 recorded attempts. Kept at 3.0 (`MAX_AXIS_DEVIATION_MM`).
- **Drop M1/M2 from the axis fit permanently — not chosen**: consensus handles any single
  marker going bad, not only the one that did this time, and names it.
- **Consensus rule**: the largest subset (all, then one fewer, …, never below
  `MIN_AGREEING_MARKERS` = 5) whose centres all lie within 3.0 mm of their own weighted axis.
  Exhaustive over subsets (≤ 29 fits from 7 cached circles), not greedy: a bad marker drags
  the full-set mean so a good one can look worst. With ≤ 5 markers fitted there is no
  majority, so the old all-must-agree rule applies unchanged.
- **Kept closed**: an outcast is excluded from the plane height as well as the axis; an
  outcast **yaw anchor** (QTM 4) refuses, because the yaw offset IS its parked angle (3 mm
  at 117 mm ≈ 1.5° ≈ 80 mm sideways at 3 m, with no downstream error).
- **Shortening the 0° hold (owner ask) — deferred**: the hold is BB firmware
  `CALIB_PAUSE_MS = 2500` (+ ~1.5 s settle confirm), and `calculate_yaw_offset` averages the
  last 200 marker samples / 10 yaw readings (~1 s) of it, so 0.5 s would average motion into
  the yaw offset. The BallButler checkout also holds unfamiliar uncommitted edits and has
  diverged from origin, so nothing was built from it.

## Fix

`find_rotation_axis` takes `max_axis_dev_mm` / `min_agreeing`, marks outcasts with status
`'outcast'` and their distance from the consensus axis, and its refusal now lists every
marker's deviation (failures used to log only the max). `mocap_node` appends
`· outcast Marker N (d mm off axis)` to the `BB calibrated:` line; the reason rides the
existing per-marker WARN.

## Verification

- Replay of all 14 recorded windows (scratchpad probe, 2026-10-06): **14/14 pass**, M1
  named in each of the 11 former refusals (3.44–4.98 mm), position spread ±0.4 mm per night.
- New tests fail on the old solver (5 of 8) and pass on the new one.
- Gate (2026-10-06, `./run_tests.sh`, log `temp/logs/gate_bb_consensus_20261006.log`):
  **6061 passed, 1 failed** — `test_skills_executor.py::test_the_window_closes_before_this_balls_next_release`,
  from another session's uncommitted `executor.py`; on a detached worktree of HEAD with only
  this change applied, that file + the BB tests: **205/205 pass** (same date).
- `python ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py`: 47/47.
