---
title: "GUI chart grid: a short column fills the full height, and every chart in a column plots at one height"
type: feature
date: 2026-10-02
status: resolved
files_changed:
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/css/charts.css
  - ros_ws/gui/css/panels.css
subsystem:
  - gui
---

# GUI chart grid fill and equal plot heights

## Summary

Hiding a chart used to leave a blank slot at the bottom of its column. Hiding BB Hand left
Hand and BB Pitch in two of three rows. Now a column holding fewer charts than the others
stretches them evenly over the full height. Separately, the bottom chart of each column draws
the time-axis labels (28 px against a 4 px stub elsewhere), which took 24 px from its plot
area. Every chart in a column now plots at the same height. Also: the Event Log's timestamp
column was a fixed 72 px, narrower than `HH:MM:SS.mmm`, so labels ran into it.

## Design

- `applyChartLayout()` keeps the column count and the column-major fill order, but places
  every visible cell explicitly. The row-track count is the LCM of the column lengths, and a
  chart in a column of m spans tracks/m. With columns of 3, 3 and 2 there are 6 tracks; each
  chart in a full column spans 2 and each in the short column spans 3.
- A fixed track of `X_AXIS_SIZE - X_AXIS_STUB_SIZE - row-gap` px sits under the grid, and each
  column's bottom chart also spans it. Those two constants now also set the uPlot x-axis size,
  so the layout and the axis cannot drift apart.
- The Event Log timestamp column is `max-content` (monospace, so every row is the same width)
  with an 8 px gap.

## Verification

On 2026-10-02, the real page was driven in headless Chromium over CDP: visibility pills were
clicked and every visible cell's `.u-over` (plot area) was measured. With all 9 charts visible,
every plot is 61 px (the bottom row was 24 px shorter before). With BB Hand hidden, Hand and BB
Pitch are 99 px each. With both BB charts hidden, Hand is 212 px. Hiding Leg 0 and reloading
the page also measured correctly. The Event Log's timestamp ends 8 px before its label.
`pytest tests/ros/test_gui_geometry.py -q` (2026-10-02): 135 passed.
