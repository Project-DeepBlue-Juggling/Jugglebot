---
title: "GUI: every chart axis shows at least two ticks at every size and zoom; sub-second time labels (BUG-37)"
type: bugfix
date: 2026-11-01
status: complete
phase: gui
files_changed:
  - ros_ws/gui/js/axis-ticks.js
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/can-traffic.js
  - tests/ros/js/axis_ticks_harness.js
  - tests/ros/test_gui_axis_ticks.py
  - tests/ros/test_gui_replay_chart.py
  - tests/ros/test_gui_traffic_reset.py
  - map/bug-some-gui-chart-zoom-levels-result-in-only-a-single-tick-mark.md
  - docs/project-map.md
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - ux
---

# BUG-37: at least two ticks on every chart axis

**Cause.** uPlot picks the smallest ladder step whose on-screen spacing is at least `space` px
(defaults 30 px y, 50 px x). On a short chart, or a zoom that leaves a narrow span, that minimum
is a large fraction of the plot, so the chosen step can be most of the span and only one
multiple of it lands in range.

**Fix.** New DOM-free `ros_ws/gui/js/axis-ticks.js`. `axis.space` becomes
`min(default, plotDim / (2F))`, where F is the largest neighbour ratio in the step ladder. The
chosen step is then below half the span, and any interval two steps long holds two multiples.
The numeric ladder has F = 2. uPlot's seconds ladder jumps 1 s -> 5 s and 1 min -> 5 min (F = 5),
so `TIME_INCRS` is that ladder plus 2 s and 2 min (F = 2.5). Big charts keep the default spacing.
Applied to every axis of both uPlot builders: the telemetry grid (x plus each value axis) and the
CAN-traffic charts (x and rate). Owner chose all axes, all charts (2026-11-01). `timeTickLabels`
adds fractional seconds once the tick step drops below 1 s, so two ticks never both read
`12:34:56`. It replaces the two hand-rolled HH:MM:SS formatters.

## Verification

- 2026-11-01, `pytest tests/ros/test_gui_axis_ticks.py -q`: 7 passed in 0.54 s. The harness
  replays uPlot's step choice over plot sizes 20-1600 px and spans from 1e-3 to 1e4 (numeric) and
  4 ms to 2 h (time). Worst case is >= 2 ticks with the fix; the same sweep on stock settings
  reproduces a single tick. Also checked: the ladder ratios equal the constants, the labels, and
  both builders' wiring.
- The replay-chart and traffic-reset sandboxes copy the chart modules verbatim, so they now copy
  `axis-ticks.js` too (the first gate run errored on the missing import).
- Not run: a live browser pass.
