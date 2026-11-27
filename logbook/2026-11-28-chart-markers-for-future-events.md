---
title: "ODrive charts draw the replay span's Throw/Catch markers beyond the playhead (BUG-150)"
type: bugfix
date: 2026-11-28
status: complete
phase: "night-shift"
subsystem: gui
files_changed:
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/event-store.js
  - tests/ros/test_gui_span_events.py
---

**What / why.** In replay the event store's rings hold only the past (events arrive as the playhead advances and a backward seek trims them). The IMU charts opted in to a separate span list (`setSpanEvents`, read straight from the recording) so their Throw/Catch markers show on both sides of the playhead; the ODrive charts were deliberately excluded (`spanMarkerCharts` WeakSet), so their markers vanished for every event not yet reached. The store query itself (`getEventsInRange`) has no "now" clamp; the cull was the missing opt-in. `eventsForChart` now always passes `{span: true}`, so hit-testing and painting on every chart (ODrive and IMU) use one query path; the WeakSet is gone. Zoom and scroll are untouched.

**Left open.** The span list is only filled when an IMU capture is selected (`imu-charts.js` `syncSpanMarkers`), so with no capture selected the ODrive charts still show only the past. In live mode a marker beyond the chart's right edge (xMax = now) is simply off-screen, not culled. Scanning the span independent of the IMU selection is a separate change. How it looks on the live GUI is something only eyes can judge.

**Verification.** 2026-11-28, `tests/ros/test_gui_span_events.py` with the old `telemetry-charts.js` restored: 1 failed, 7 passed (the source scan pinning that every chart queries with `span: true`); with the fix, see the run below.
