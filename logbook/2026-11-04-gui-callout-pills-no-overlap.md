---
title: "GUI chart hover callout pills no longer overlap: a pure 1-D label layout spreads pills of nearby curves"
type: bugfix
date: 2026-11-04
status: complete
phase: "BUG-61"
files_changed:
  - ros_ws/gui/js/callout-layout.js
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/js/can-traffic.js
  - tests/ros/test_gui_callout_layout.py
  - tests/ros/test_gui_traffic_reset.py
  - tests/ros/test_gui_replay_chart.py
  - logbook/2026-11-04-gui-callout-pills-no-overlap.md
  - logbook/INDEX.md
subsystem: gui
---

# BUG-61: hover callout pills overlap

**What.** Each hover pill was placed at its own curve's y. Curves closer than one pill height (position and temperature, say) stacked their pills and every pill behind the front one was invisible; uPlot's cursor points are off, so the pills are the readout. New pure module `ros_ws/gui/js/callout-layout.js` (`layoutCallouts(desired, h, plotH)`): sort by desired y, sweep down pushing each pill below the previous, sweep up from the bottom if the last overflows, stack from the top if they cannot fit. Order by y is preserved. `telemetry-charts.js` and `can-traffic.js` `updateCallouts` now compute every visible pill's y first, lay them out, then set `top`; pill height is measured (`offsetHeight`, 16 px fallback). Pill text is unchanged: only the vertical position moves, and a pill whose curve is off-scale is now held at the plot edge instead of drawn outside it.

**Why.** Pure and DOM-free so the geometry is testable under node. No robot, control or telemetry path is touched; GUI display only. Two existing tests that copy the chart modules into a sandbox now also copy the new sibling module.

**Verification.** 2026-11-04, `pytest -q tests/ros/test_gui_callout_layout.py tests/ros/test_gui_console.py tests/ros/test_gui_axis_ticks.py tests/ros/test_gui_clock_contract.py tests/ros/test_gui_geometry.py tests/ros/test_gui_replay_chart.py tests/ros/test_gui_traffic_reset.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py`: **196 passed in 10.04 s**. The new behaviour tests were confirmed red against an identity layout (4 of 8 failed: close pills, input order, bottom cluster, too many) and green with the real one (8 passed, same day, `pytest -q tests/ros/test_gui_callout_layout.py`). Not seen on a live browser chart.
