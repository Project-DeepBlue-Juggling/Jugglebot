---
title: "GUI IMU charts: the top handle no longer un-hides the ODrive grid, and Throw / Catch markers cover the whole capture in replay"
type: bugfix
date: 2026-11-22
status: complete
phase: "IDEA-20 tooling"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/imu-charts.js
  - ros_ws/gui/js/event-store.js
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/replay/span-events.js
  - ros_ws/gui/css/charts.css
  - tests/ros/test_gui_span_events.py
  - tests/ros/js/span_events_harness.js
subsystem: [gui, platform-imu]
tags: [gui, charts, imu, replay]
---

# GUI IMU charts: hidden ODrive grid and future markers

Two owner reports after the 2026-11-21 entries (`2026-11-21-gui-imu-charts-followups`).

- **Hidden grid stayed hidden.** Cause: the top-handle drag called `clearChartGridHidden()` on every pointer move, so the
  ODrive grid reappeared whenever the IMU group was the only one showing. The drag now clears it only when no IMU group is
  shown. The IMU-only panel also took its height from its content (`height: auto`), so the drag had nothing to size: it
  now uses the single-group height (`--chart-panel-height`, `jugglebot-chart-height`) with the IMU row filling it.
  IMU hidden with the ODrive grid shown never touched the IMU group, and was checked.
- **Markers beyond the playhead.** The event store holds Throws and Catches only up to the playhead (they arrive as it
  advances and a backward seek trims them), while the IMU charts show the whole capture. `replay/span-events.js` reads
  `/balls` of the selected capture (plus 3 s each side, the events are stamped before the message that reports them)
  with filtered lite chunk loads, runs it through a fresh `ball-events.js` mapper, and `event-store.js` keeps the result
  in a separate span list that only the IMU charts merge in. The Event Log, the ODrive charts and the replay trim never
  see it. Chosen over feeding the ring: the ring is trimmed by seeks and would put future events in the Log.

Verification:
- 2026-11-22, `flock /tmp/jugglebot-tests.lock pytest tests/ros -k "gui or replay" -q`: **860 passed, 1 skipped in 210.62 s**.
- 2026-11-22, headless Chromium over CDP, replay `2026-11-20_22-53-34`, capture `AEE1` (31.0 s): span markers 43 with the
  playhead at the capture start and at its end (ring markers inside the capture: 1 and 42); Event Log gained 0. ODrive
  grid stayed hidden through a synthetic top-handle drag (IMU row 219 to 282 px), a window resize and a page refresh.
