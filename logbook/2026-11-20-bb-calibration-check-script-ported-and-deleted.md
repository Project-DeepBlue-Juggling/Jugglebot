---
title: "BB calibration check script ported into collected tests and deleted (BUG-115)"
type: refactor
date: 2026-11-20
status: complete
phase: "night-shift"
subsystem: tests
files_changed:
  - tests/ros/test_bb_calibration_pipeline.py
  - tests/ros/test_bb_calibration_arc_span.py
  - ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/tests/__init__.py
---

# BB calibration check script ported into collected tests and deleted

**What / why.** `ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py` (760 lines, hand-rolled `check()` runner, no `test_` functions) was never collected. Owner (TKT-84): has never run the check by hand, option 1 = compare, port the uncovered checks, delete.

**Deletion cannot touch the calibration.** `grep -rn "test_bb_calibration"` (excluding .git, temp, logbook, plans, map) finds only tests/ros files and the script itself; `grep -rn "jugglebot.tests\|jugglebot/tests" ros_ws tools config` finds nothing. `setup.py` `packages=` lists `jugglebot`, `.can`, `.motion`, `.motion.trajectory`, `.motion.skills`, `.tracking` but not `jugglebot.tests`, no `data_files` entry, no entry point, launch or GUI reference. The directory held only the script and an empty `__init__.py`; both are `git rm`ed.

**Comparison** (script check -> covering test):

| Script check | Covered by |
|---|---|
| 1 perfect circle | ported: `test_bb_calibration_pipeline.py::test_perfect_circle_recovery` (coplanar file asserted only z/xy/yaw at 1e-3, not tilt or uncertainty) |
| 2 noisy 0.5 mm | ported: `::test_noisy_data_within_tolerance` |
| 3, 10 yaw offsets incl. +/-pi wrap | ported: `::test_yaw_offset_recovered_including_near_the_wrap` (7 offsets) |
| 4 sparse data | ported: `::test_all_markers_below_min_points_refused` |
| 5 one marker | ported: `::test_single_marker_with_data_refused` |
| 6 empty | ported: `::test_empty_data_refused` |
| 7 non-rigid | ported: `::test_non_rigid_motion_refused` |
| 8 tilted axis | ported: `::test_tilted_axis_tilt_and_position_recovered` (coplanar covers XY only) |
| 9 partial visibility | ported: `::test_partial_marker_visibility_still_succeeds` |
| 11 fit_circle_3d | ported: `::test_fit_circle_3d_accuracy_on_known_circle` |
| 12 few yaw readings | ported: `::test_too_few_yaw_readings_refused` |
| 13 high yaw uncertainty | ported: `::test_high_yaw_uncertainty_refused` |
| 14 horizontal axis | ported: `::test_horizontal_axis_refused` |
| 15 arc-span floor (a-d, noise 0/0.1/0.5) | covered: `test_bb_calibration_arc_span.py` (truncated refused, full/65 deg succeed, floors disabled, stubby marker) |
| 16 marker-radius floor, yaw span | covered: arc_span `test_collapsed_marker_fit_is_excluded_by_radius`, `test_angular_span_*` |
| 17 branch-cut yaw uncertainty | covered: arc_span `test_yaw_offset_uncertainty_survives_the_atan2_branch_cut` |

Total: 3 covered, 13 ported, 0 not portable (the script had no hardware or interactive step). The "3 markers sufficient" line was an unconditional `check(..., True)` and is not ported.

**Mutation check** (each break temporary, `git checkout` restored): disabling the yaw-uncertainty refusal fails `test_high_yaw_uncertainty_refused`; the axis-deviation check, `test_non_rigid_motion_refused`; the yaw-readings count, `test_too_few_yaw_readings_refused`; the horizontal-axis refusal, `test_horizontal_axis_refused`; `wrap_pi` made a no-op, the two +/-pi wrap cases; result tilt forced to 0, `test_tilted_axis_tilt_and_position_recovered`. Disabling the per-marker `min_points` skip did NOT fail `test_all_markers_below_min_points_refused`: the sparse data is refused later by another "insufficient" path, as the script's own check was (it accepted "insufficient" or "failed"); left as is.

**Verification.** 2026-11-20, `tools/night_shift_nonet.sh python -m pytest tests/ros/test_bb_calibration_pipeline.py -q -p no:cacheprovider`: 19 passed. Scoped run over all bb_calibration files: see the unit report.
