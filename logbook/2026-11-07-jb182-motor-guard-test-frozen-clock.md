---
title: "BUG-71: motor_guard interpolation tests read the wall clock; freeze it for the interpolation step"
type: bugfix
date: 2026-11-07
status: complete
phase: "Night Shift 2026-11-07 BUG-71"
files_changed:
  - tests/motion/test_motor_guard.py
  - logbook/2026-11-07-jb182-motor-guard-test-frozen-clock.md
---

# BUG-71: frozen clock for the motor_guard interpolation tests

`test_normal_interpolation_unchanged` failed one full gate on 2026-11-06 (a 0.19 mrev miss against atol=1e-4) and passed on the same commit the next run. It backdated `_mpc_base_timestamp` by `dt_test` and asserted linear interpolation at exactly `dt_test`, so any scheduling delay before the `perf_counter()` read inside `_interpolate_and_send` moved the answer. The safety claim (plain linear within the window) does not need real time.

Fix (test only; `motor_guard.py` untouched): a helper `_interpolate_at_frozen_age` patches `time.perf_counter` to a constant `t0`, sets the base timestamp to `t0 - age`, and runs one step. The `time.sleep(0.02)` stays, because the finite-difference velocity is built from real elapsed time between the two commands; only the interpolation step is frozen. `test_cubic_interpolation_with_jerk` had the identical defect (atol=1e-5, a comment admitting the jitter) and uses the same helper. `test_decay_boundary_continuity` (the `MAX_EXTRAP_DT_S +/- eps` pair, velocity atol 1e-3, about 0.06 ms of delay per rev/s) and `test_extrapolation_vel_decays` (atol 0.05 at 5 rev/s, exceeded at about 0.6 ms) had the same defect and are now frozen too: four tests in all. In the latter the tracking-clamp feedback is built from `dt_test` and the frozen age is the same `dt_test`, so the two stay consistent. The four moved out of the manual `main()` runner, which omits `monkeypatch` tests by design.

Other `perf_counter() - ` sites were left alone after reading each. Stale-feedback and stale-command sites use margins of 10 ms or more. Sites past the extrapolation or segment-end clamp saturate (atol 1e-6 at 0.025 and 0.040 s, the 0.150/0.160 s sites). The `- T` chord site is safe because `s = min(dt/T, 1.0)` saturates. The Hermite sites at T and T/2 are safe (saturation, and a bracketing range). The `MAX_EXTRAP_DT_S` stroke-clamp sites (`test_stroke_clamp` twice, `test_stroke_clamp_workspace_status`) drive 100 rev/s into the stroke limit: the outcome is the clamped limit, zeroed feedforwards and HARD_LIMIT status, none of which depends on the age, so they are not age-sensitive and stay unfrozen.

## Verification

- 2026-11-07, unfrozen original with a `time.sleep(0.05)` after the timestamp line, `pytest -q tests/motion/test_motor_guard.py -k "normal_interpolation_unchanged or cubic_interpolation_with_jerk"`: 2 failed.
- 2026-11-07, frozen version with the same 0.05 s sleep inside the helper, same command: 2 passed. Sleep then removed.
- 2026-11-07, frozen helper with `time.sleep(0.05)` added after the timestamp line, `pytest -q tests/motion/test_motor_guard.py -k "normal_interpolation_unchanged or cubic_interpolation_with_jerk or extrapolation_vel_decays or decay_boundary_continuity"`: 4 passed. Sleep then removed.
- 2026-11-07, `pytest -q tests/motion/test_motor_guard.py tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_front_matter.py`: 64 passed in 26.75 s.
- 2026-11-07, `PYTHONPATH=ros_ws/src/jugglebot:. python tests/motion/test_motor_guard.py`: ALL 41 TESTS PASSED (no TypeError).
