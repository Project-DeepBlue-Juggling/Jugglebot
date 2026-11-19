---
title: "FEAT-59 slice 1: retire the dormant vel-FF plumbing check with its 16 tests (TKT-83 option 2); the ILC measurement tools are left for a day Session"
type: refactor
date: 2026-11-20
status: in_progress
phase: night-shift
subsystem: tests
related_items: [FEAT-59, TKT-83]
files_changed:
  - tools/check_vel_ff_plumbing.py
  - tests/sim/test_check_vel_ff_plumbing.py
  - tests/sim/test_tilt_cal_analyse.py
  - logbook/2026-11-20-retire-check-vel-ff-plumbing.md
---

# FEAT-59 slice 1: retire the vel-FF plumbing check

## What and why

The owner chose TKT-83 option 2: retire the ILC measurement tools and the vel-FF plumbing check, each in the same commit as its tests, and keep `toss_release`. This branch does the vel-FF half. `tools/check_vel_ff_plumbing.py` subscribes to motor_guard's :5556 telemetry. Its own docstring has called it dormant since 2026-08-01, because nothing in the production launch publishes :5556 after the MPC removal. It goes with `tests/sim/test_check_vel_ff_plumbing.py` (16 tests).

Importer grep (2026-11-20, `grep -rln check_vel_ff_plumbing --exclude-dir=.git --exclude-dir=temp .`): the tool, its own test, a docstring in `tests/sim/test_tilt_cal_analyse.py` (reworded here), the FEAT-59 Item, two archived plans and one 2026-08-03 logbook entry. History is not rewritten. Nothing else imports it.

## Left for a day Session

- **The ILC measurement tools.** Their test is `tests/ros/test_ilc_measurement_*.py`, and the modules and README rows live in the probe directory under `tools/`. The Night Shift's command guard refuses any command that names that directory or the test file, so the night cannot delete them. The README rows go in the same commit.
- **The merge.** This branch removes tests. The merge gate's condition (i) holds every branch that collects fewer tests than its base, and a day Session merges it.

## Verification

(2026-11-20, `night_shift_nonet.sh timeout 600 python -m pytest tests/sim/test_tilt_cal_analyse.py tests/sim/test_logbook_front_matter.py -q -p no:cacheprovider`): **41 passed in 4.90 s**. The full suite was not run tonight; it runs when a day Session merges the branch.
