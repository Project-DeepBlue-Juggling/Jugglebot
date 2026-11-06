---
title: "JB-182: motor_guard interpolation tests read the wall clock; freeze it for the interpolation step"
type: bugfix
date: 2026-11-07
status: complete
phase: "Night Shift 2026-11-07 JB-182"
files_changed:
  - tests/motion/test_motor_guard.py
  - logbook/2026-11-07-jb182-motor-guard-test-frozen-clock.md
---

# JB-182: frozen clock for the motor_guard interpolation tests

`test_normal_interpolation_unchanged` failed one full gate on 2026-11-06 (a 0.19 mrev miss against atol=1e-4) and passed on the same commit the next run. It backdated `_mpc_base_timestamp` by `dt_test` and asserted linear interpolation at exactly `dt_test`, so any scheduling delay before the `perf_counter()` read inside `_interpolate_and_send` moved the answer. The safety claim (plain linear within the window) does not need real time.

Fix (test only; `motor_guard.py` untouched): a helper `_interpolate_at_frozen_age` patches `time.perf_counter` to a constant `t0`, sets the base timestamp to `t0 - age`, and runs one step. The `time.sleep(0.02)` stays, because the finite-difference velocity is built from real elapsed time between the two commands; only the interpolation step is frozen. `test_cubic_interpolation_with_jerk` had the identical defect (atol=1e-5, a comment admitting the jitter) and uses the same helper. Other `perf_counter() - ` sites were left alone: the stale-feedback and stale-command ones use margins of 10 ms or more, and the ones past the extrapolation or segment-end clamp saturate, so the result does not depend on the age (atol 1e-6 sites at 0.025 and 0.040 s, the 0.150/0.160 s sites). The decay-window site (atol 0.05 on velocity) has a tolerance wide enough for load.

## Verification

- 2026-11-07, unfrozen original with a `time.sleep(0.05)` after the timestamp line, `pytest -q tests/motion/test_motor_guard.py -k "normal_interpolation_unchanged or cubic_interpolation_with_jerk"`: 2 failed.
- 2026-11-07, frozen version with the same 0.05 s sleep inside the helper, same command: 2 passed. Sleep then removed.
- 2026-11-07, `pytest -q tests/motion/test_motor_guard.py`: 50 passed in 25.04 s.
