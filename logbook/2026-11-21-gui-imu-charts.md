---
title: "GUI: IMU charts group (linear acceleration and angular rate from Platform IMU captures) on the same time axis as the motor charts, with two new gui_server routes"
type: feature
date: 2026-11-21
status: complete
phase: "IDEA-20 tooling"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/gui/imu_api.py
  - ros_ws/gui/gui_server.py
  - ros_ws/gui/js/imu-charts.js
  - ros_ws/gui/js/imu-charts-core.js
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/index.html
  - ros_ws/gui/css/charts.css
  - tests/ros/test_gui_imu_api.py
  - tests/ros/test_gui_imu_charts.py
  - tests/ros/js/imu_charts_harness.js
  - tests/ros/test_replay_stdlib.py
subsystem: [gui, platform-imu]
tags: [gui, charts, imu, replay]
---

# GUI: IMU charts

## What and why

The owner wanted to look at the Platform IMU captures (IDEA-20) himself, with only ssh and VS Code access to the
Jetson, so the captures are drawn in the operator GUI. A new chart group, off by default, sits as a row above the
motor (ODrive) chart grid: linear acceleration (x, y, z and magnitude, in g) and angular rate (x, y, z, deg/s), in the
sensor frame because the mount calibration is not done. A switch turns the right chart into angular acceleration
(10 ms boxcar, then a central difference; never across invalid samples). A dropdown lists the captures of the current
launch (LIVE) or of the recording being replayed, matched by time.

Decisions the owner made (2026-11-20): choosing a capture pauses the motor charts and moves every chart to its span,
after which zoom, pan, presets and hover are shared by all eleven charts; the newest capture is followed in LIVE
unless an older one was picked by hand, and the capture under the playhead in REPLAY; the IMU row adds 200 px to the
chart panel instead of taking height from the motor grid.

## How

- `gui_server.py` gains `GET /api/imu/captures` (list from the `.json` sidecars, optional `from`/`to` overlap filter)
  and `GET /api/imu/captures/<stem>/data` (a little-endian binary of times, acceleration, rate and the valid flags).
  `imu_api.py` reads the `.npz` with `zipfile` and a small `.npy` parser because the server runs on the system
  Python, which has no numpy. Capture times are `header.freeze_wall_us` and `n_samples_written / fit.rate_hz`;
  `span_wall_s` is the requested window and is not used.
- `telemetry-charts.js` exports a small hook (`onSharedXRange`, `showRangeOnAllCharts`, `selectRangeOnAllCharts`,
  `seekReplayTo`, `attachZoomPanControls`, `SYNC_KEY`). With the IMU group hidden the listener only stores the range.
- `imu-charts-core.js` holds the logic as pure functions (parse, gaps, decimation, derivative, selection and follow
  rules); `imu-charts.js` is the DOM and uPlot layer.

## Known limits

- `gui_server.py` changes need `sudo systemctl restart jugglebot-gui`; until then the group says so.
- R (reset view) does not resume a paused chart; that is the existing motor-chart behaviour. Space resumes.
- Showing the group in LIVE with the charts scrolling pauses them on the newest capture (the owner's time-linking
  choice applied to the default selection).
- Not exercised: a live launch (no ROS was up), the light theme. While the group is shown in LIVE the IMU charts
  redraw at the motor charts' 20 Hz even when the capture is out of view.

## Verification

- Headless Chromium 156 over the DevTools protocol against the worktree server and the real recording
  `2026-11-20_22-53-34` (2026-11-20/21): hidden by default with no `/api/imu` request; four captures listed; the
  selection follows the playhead; hover and wheel zoom shared with the motor charts; series pills, the angular
  acceleration switch, Jump to capture, hide, and leaving and re-entering replay all pass. Two defects were found
  there and fixed (an exception on every pill click; the toolbar collapsing).
- Tests: see the commit message for the (date, command, result) triple.
