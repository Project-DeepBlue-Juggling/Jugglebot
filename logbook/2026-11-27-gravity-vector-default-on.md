---
title: "The Tracker's measured gravity vector [43.0, 83.0] mm/s^2 is the config default (owner decision after the Sitting of 2026-11-27)"
type: feature
date: 2026-11-27
status: complete
phase: "FEAT-27 kinematic calibration acceptance"
related_plan: ""
related_issues: []
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - tests/tracking/test_flight_fit_gravity_tilt.py
  - tests/ros/test_ball_tracker_gravity_tilt.py
subsystem: [tracking, config]
---

## What changed

`ball_tracking.gravity_world_xy_mmps2` in `config/hardware_config.yaml` goes from `[0.0, 0.0]` to the measured
`[43.0, 83.0]`, and the generated copies follow (`python config/generate_config.py`). The node parameter
`gravity_world_xy_mmps2` still overrides it live, so `"[0.0, 0.0]"` is the A/B arm now. The firmware headers carry the
constant but no firmware reads it. Two tests that pinned the default at zero now pin the measured value.

## Why

Owner decision, 2026-11-27. At that day's Sitting 96 Columns Throws flew with the vector on: 94 caught, landing rms
29.0 mm against 29.2 mm over 728 earlier Throws, and the owner judged the Catches clearly smoother. The debrief's
numbers show no harm and cannot show the gain (the off arm was one Attempt of 17 Throws), so the default rests on the
offline measurement (1,139 free flights) and the owner's observation. The plumb-line check of the value is still
owed and stays in FEAT-27's Sitting section.

## What changes on the robot

Every landing the Tracker publishes moves by the vector's effect: about (+8, +21) mm in the Robot frame for an early
Fit and (+3, +11) mm for the final Fit, relative to the zero default. Catch aims follow. The jugglebot Memory already
holds about 95 Columns rows learned with the vector on; its oval rows were learned with it off, so the oval's plant
offset will re-learn over the first Attempts (the Columns offset moved 7.6 mm in y over 59 rows on 2026-11-27).
Nothing in the planner, the setpoint stream or the feedforward changes; no timing changes.

## Verification

- 2026-11-27, `python -m pytest -q tests/sim/test_imu_clock_check.py tests/ros/test_ball_tracker_gravity_tilt.py tests/tracking`:
  115 passed. `test_config_default_is_the_measured_vector` fails on the old config value (it was the zero pin, turned
  round).
- The full gate is cited in the commit message.
