---
title: "GUI: paused-chart cursor drift was uPlot holding ring-buffer views the store shifts under it; armed-but-hidden chart pills now dimmed"
type: bugfix
date: 2026-09-07
status: resolved
phase: "gui — operator tooling"
files_changed:
  - ros_ws/gui/js/telemetry-charts.js
  - ros_ws/gui/css/charts.css
subsystem:
  - gui
related_entries:
  - 2026-09-06-gui-armed-indication-coords-target-chart-isolate.md
---

# GUI: paused-chart cursor drift fixed; armed-but-hidden pills dimmed

## Summary

Two owner-requested follow-ups to the 2026-09-06 GUI work, `ros_ws/gui/` only, no
Python touched.

1. **Cursor drift while zoomed (bug).** Symptom: zoomed into a time slice, sweeping the
   mouse, the data-value cursor/callouts start walking forward in time on their own;
   Resume + re-zoom clears it briefly. Cause: `ChartDataStore.getAlignedData()` returned
   zero-copy `subarray` **views** into the Float64Array ring buffer, and once the store is
   full (`CACHE_WINDOW_SEC` 600 s × 20 Hz = 12 000 samples) every `push()` does
   `copyWithin(0, 1)` — an in-place left shift of the whole buffer. Zoom-to-history pauses
   the charts, and `repaintAllCharts()` returns early while paused, so uPlot keeps the view
   it was last handed: the canvas is never redrawn (curves look right) but
   `u.cursor.idx → u.data[0][idx]` and `updateCallouts()` now read a timestamp one sample
   later per push — 50 ms per tick, i.e. real-time drift. That is why it only appears
   "sometimes": the page has to have streamed for 10 min before the buffer is full, and
   re-zooming (`reloadChartDataFromStores` → `setData`) hands out a fresh view that goes
   stale again within seconds.
   Fix: `getAlignedData(signalKeys, windowStart, copy = false)` uses `slice` when `copy`
   is true; the three call sites pass `paused` (initial data in `buildAllCharts`, the forced
   repaint in `repaintAllCharts`) or `true` (`reloadChartDataFromStores`, paused-only path).
   **The audit caught that this alone left the primary flow open**: the Pause button / Space
   goes through `pauseCharts()`, which flipped the flag and returned, so uPlot kept the live
   views from the last repaint and the drift survived for pause-then-hover (only the
   pause-via-zoom/pan and rebuild paths were covered). `pauseCharts()` now captures
   `getViewAnchor()` *before* flipping `paused` (once paused the anchor is
   `pausedWindowEnd`, and wall-clock now would jump a stale stream's window past its data),
   then forces one repaint so the chart is handed an owned copy at the same window it was
   showing. Also from the audit: `reloadChartDataFromStores` is now rAF-coalesced
   (latest-wins), because the middle-drag pan calls it per mousemove and each call now
   copies — up to ~2.6 MB at the 600 s span.
   Live mode keeps the zero-copy views deliberately: every push schedules a rAF repaint that
   re-issues `setData` before the next push can shift the buffer, and copying there would
   cost up to ~8 MB per repaint at the 600 s span. The invariant is stated in the
   `getAlignedData` docstring: **uPlot may hold a view only if the next push is guaranteed
   to be followed by a repaint before the chart is read.**

2. **Armed-but-hidden pill.** `.signal-toggle.armed` overrode the active/inactive
   distinction, so a hidden chart's pill for a CLOSED_LOOP axis looked identical to a
   visible one's. New rule `.signal-toggle.armed:not(.active) { opacity: 0.5; }` keeps the
   violet (axis state still reads) but dims it to the same weight the `.disabled` pill uses.
   Because `opacity` dims the whole subtree, the audit had the transients exempted
   (`:not(.hold-active):not(.hold-confirmed):not(.chart-pick-flash)`): the long-press sweep
   is itself a 0.25-opacity `::before`, and the 3D-pick flash is the only feedback when a
   pick lands on a hidden chart — both now pop to full strength while they run.

## Verification

- Mechanism probe (2026-09-07, `node -e` replicating `push`/`copyWithin` with a 5-slot
  buffer): a held `subarray` view read `[2,3,4]` at set time and `[4,5,6]` after two pushes;
  the `slice` copy still read `[2,3,4]`.
- Syntax (2026-09-07, `node --input-type=module --check < ros_ws/gui/js/telemetry-charts.js`): OK.
- Scoped tests (2026-09-07, `pytest tests/ros/test_gui_geometry.py tests/ros/test_gui_fk_golden.py tests/firmware/test_config_drift.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q`, after the audit fixes): **186 passed in 7.23 s**. No `*.py` / `*.yaml` changed, so the full gate is not triggered.
- Audit (2026-09-07, one `audit-reporter` pass over `git diff -- ros_ws/gui`): 1 BLOCKING (the
  `pauseCharts` hole), 1 MEDIUM (uncoalesced copying pan), 1 LOW (opacity on the transients)
  — all three applied as suggested; the auditor's enumeration of every other `u.data` holder
  (`nanGaps`, delta callouts, collapsed/backgrounded-tab early returns) found no further gap.
  Not re-audited after the fixes (owner: one audit per batch).
- Not yet loaded in a browser this session — the owner's next page load is the visual check
  for the dimmed pill; the drift needs a >10 min stream, then pause + zoom + hover.
