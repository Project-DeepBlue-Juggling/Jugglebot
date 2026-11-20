---
title: "GUI IMU charts follow-ups: independent heights for the IMU and ODrive charts, the replay playhead on the IMU charts, the ODrive hide button renamed"
type: feature
date: 2026-11-21
status: complete
phase: "IDEA-20 tooling"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/gui/js/imu-charts.js
  - ros_ws/gui/js/imu-charts-core.js
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/index.html
  - ros_ws/gui/css/charts.css
  - ros_ws/gui/css/theme.css
  - tests/ros/test_gui_imu_charts.py
  - tests/ros/js/imu_charts_harness.js
subsystem: [gui, platform-imu]
tags: [gui, charts, imu, replay]
---

# GUI IMU charts follow-ups

Owner requests after first use (2026-11-21), following `2026-11-21-gui-imu-charts`.

- **Independent heights.** A divider between the IMU row and the motor grid sets the IMU row's height alone (100 to
  500 px, default 200, double-click resets, stored under `jugglebot-imu-chart-height`); the existing top handle sets
  the motor grid's height alone. The top handle's drag was corrected for the IMU row's height, otherwise the grid
  jumped by about 205 px on the first move. The 3D view keeps at least 150 px by shrinking the IMU row; a very tall
  motor grid can still push it below that.
- **Playhead.** In REPLAY the current-frame line is drawn on both IMU charts with the motor charts' own
  `drawReplayPlayhead`, now exported; nothing is drawn in LIVE.
- **Button.** "Hide charts" / "Show charts" is now "Hide ODrive charts" / "Show ODrive charts".

## Verification

Headless Chromium against the worktree server and recording `2026-11-20_22-53-34` (2026-11-21): divider drag changed
the IMU row 200 to 280 px with the motor cells unchanged; the top handle changed the motor grid 242 to 305 px with the
IMU row unchanged; both heights survived a page refresh; the playhead line sits at the same position on all eleven charts;
no console errors. Tests: see the commit message.
