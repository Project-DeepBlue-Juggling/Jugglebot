---
title: GUI replay - uPlot tick-loop runaway on near-constant large-magnitude y data
type: bugfix
date: 2026-10-11
status: resolved
phase: "gui-rosbag-replay - Phase 4 follow-up (memory, block 2)"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/2026-10-11-gui-replay-uplot-tick-runaway.md
  - logbook/INDEX.md
  - ros_ws/gui/js/telemetry-charts.js
  - tests/ros/js/replay_chart_harness.js
  - tests/ros/test_gui_replay_chart.py
subsystem:
  - gui
tags:
  - performance
  - testing
---

# GUI replay: uPlot tick-loop runaway

## Summary

After the first memory block (`2026-10-10-gui-replay-memory-and-main-thread`) the owner still read 2.0 GB for the tab. The cause was not resident data: a y-range padded by 5 % of a span only a few ulps wide, at |v| of 1e9 or more, makes uPlot 1.6.31 choose a tick step below half an ulp, so its tick loop never advances and the splits array grows to V8's limit. Fixed by a relative-span floor on every telemetry y scale (`yRangeFor`).

## Symptom

Owner, Win10 Chrome 154, 2026-10-11, after block 1 landed: tab reads 2.0 GB while scrubbing during playback and 1.3 GB paused, with some lag when panning quickly.

## Hypothesis

Withdrawn. Two readings were held before measuring: (1) worker heap from materialising ~7k message objects per slot; (2) typed-array garbage from per-frame chart rebuilds. Both predicted a large worker or ArrayBuffer share.

## Measurement

Headless Chromium on the Jetson, per-process split (renderer RssAnon, main JS heap, worker heap, ArrayBuffer backing, remainder, separate GPU process) at rest and over a 90 s play + seek/pan storm. Bags: `2026-10-02_12-43-28` (busy) and `2026-06-17_15-13-42` (7 min). Scripts and raw JSON in the session scratchpad (`mem/m3/`: `mem5.mjs`, `before_*`/`after_*` mem5 JSON, `probe_err.mjs`, `ulp.mjs`).

| Quantity | Measured |
|---|---|
| Renderer total | ~0.3-0.5 GB |
| Main JS heap used | 42-43 MB (18 MB after GC) |
| Worker heap used / reserved | 9-12 MB / ~70 MB (same on both bags) |
| ArrayBuffers | ~100 MB main, ~20 MB worker |
| Remainder (allocator high-water) | ~130-160 MB |
| GPU process | 226 MB, separate |

Both hypotheses fail: nothing measured is anywhere near 1.3-2 GB. The one outlier came from a dry run: in a single draw the main heap jumped from ~30 MB to 1,119 MB, with `RangeError: Invalid array length` at `splits` (uPlot's `numAxisSplits`), a 6.4 s stall, and a second event took the heap to 2.15 GB. A standalone page with real uPlot reproduced it with y data like `[1.79e9, 1.79e9 + 4.8e-7]`.

## Discussion

Mechanism, in root-cause terms. The y range was `[min - 0.05*span, max + 0.05*span]`. When the span is a few ulps at |v| around 1e9, the 5 % pad is below half an ulp, so the padded bounds round back to the data. uPlot then picks a tick increment smaller than half an ulp, and `value += incr` does not change `value`: the loop in `numAxisSplits` never advances and pushes into `splits` until V8's maximum array length throws. The failure class is "any pad or step derived from a span that can be below the float resolution of the values", not one signal.

It is intermittent because it depends on which near-constant large-magnitude series is in view (and it needs the range to collapse in the visible window), which is why seven headless storms never hit it in-app. V8 keeps the grown heap committed after the exception, which is why the owner's tab rested at 1.3 GB paused rather than dropping back.

Why the first two hypotheses went: the measured heaps were small, so worker materialisation and chart-rebuild garbage could not account for the gigabyte.

Not done, deliberately. Worker streaming slot build: worker reservation is ~70 MB on both bags and does not scale with messages per slot, so streaming may not shrink it; evidence too weak for the rewrite. Chart-store scratch reuse: contradicts the store's immutable-snapshot contract (`test_buffer_is_immutable_across_rebuilds`). Digester concurrency: it already runs one lite load at a time (`status().inflight` is a slot index, not a count).

## Fix

`ros_ws/gui/js/telemetry-charts.js`: `Y_MIN_REL_SPAN = 1e-9` and `yRangeFor(dataMin, dataMax, padFloor)`. Null or non-finite input gives `[0, 1]`; a span at or below `|v| * 1e-9` takes the flat-data pad around the midpoint; otherwise the old 5 % pad, bit-identical to before. Routed through every telemetry y scale, live and replay.

## Verification

- 2026-10-11, `python -m pytest tests/ros -k "gui or replay" -q`: 454 passed, 1 skipped.
- New `tests/ros/test_gui_replay_chart.py::test_y_range_never_stalls_uplots_tick_loop`: a harness mirror of uPlot's `incr`/`findIncr`/`numAxisSplits` with stall detection. The old formula stalls on the repro inputs; the new one terminates; ordinary inputs are unchanged; null/+-Inf/NaN give `[0, 1]`.
- 2026-10-11, real uPlot in Chromium, six repro cases: 1.1 GB heap plus the exception before; 2-5 ticks and a 1-2 MB heap after.
- Split after the fix is unchanged within noise (busy bag storm renderer mean 425 MB, max 471 MB).
- Full gate (`./run_tests.sh`, run 2026-10-11 in the replay worktree): parallel **6566 passed, 9 skipped in 279.59s (0:04:39)**; serial 3 passed, 6617 deselected in 10.08s; PASS.

## Outcome

Resolved at software level: the runaway cannot occur for any input, and the heap split shows no other large resident cost. Unverified: the owner's Win10 run; which plotted signal triggers it in the app (a field at |v| >= 1e9 with a few-ulp spread is likely, an epoch-seconds stamp the obvious candidate, but unconfirmed). Pan lag remains; the owner accepts it for now and it is noted for Phase 6.
