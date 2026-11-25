---
title: "GUI chart toolbar: the IMU charts show/hide button sits with the ODrive and all-charts show/hide buttons, right of the mm/rev divider (FEAT-66)"
type: feature
date: 2026-11-26
status: complete
phase: "Night Shift 2026-11-26"
related_plan: ""
related_issues: [FEAT-66]
files_changed:
  - ros_ws/gui/index.html
  - ros_ws/gui/css/charts.css
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/imu-charts.js
  - tests/ros/test_gui_imu_charts.py
subsystem: [gui]
tags:
  - night-shift
---

# IMU charts button moved next to the ODrive charts button

**What.** The owner asked for the IMU, ODrive and all-charts show/hide buttons in one section, with the IMU charts button to the right of the divider after the mm/rev button and to the left of "Hide ODrive charts" (FEAT-66). The ODrive, all-charts and units buttons are created at runtime (`telemetry-charts.js` `initChartGridToggle`), so the IMU button is moved there by id, between the units divider and the ODrive button. `imu-charts.js` finds it by id, so its click handler and label ("IMU charts" / "Hide IMU charts") are unchanged.

The capture tools (capture dropdown, jump, filter, spectrum) stay where they were. The rule after them now has an id and hides with them, so a hidden IMU group leaves no stray rule at the left of the toolbar. A CSS rule makes the `hidden` attribute win over `.chart-toolbar-divider`'s `display`.

**Not checked by eye.** The tests pin the DOM order and the wiring only. Whether the toolbar looks right, including how it wraps on a narrow window with the IMU tools shown, needs a look in the GUI.

## Verification

- New test `tests/ros/test_gui_imu_charts.py::test_the_imu_show_hide_button_sits_with_the_odrive_one_feat66`. It fails on the base: none of its asserted strings exist on `origin/main` (2026-11-26, `git show origin/main:ros_ws/gui/js/telemetry-charts.js` grep count 0).
- 2026-11-26, `tools/night_shift_nonet.sh timeout 600 python -m pytest tests/ros/test_gui_axis_ticks.py tests/ros/test_gui_callout_layout.py tests/ros/test_gui_geometry.py tests/ros/test_gui_imu_charts.py tests/ros/test_gui_replay_chart.py tests/ros/test_gui_span_events.py tests/ros/test_gui_traffic_reset.py -q -p no:cacheprovider`: **234 passed in 13.13 s**. These are every test that reads the four changed GUI files (`grep -rln` over `tests/`).
