---
title: GUI replay Phase 3 owner feedback - banner inset, chart resize, playhead line, scrub lag
type: bugfix
date: 2026-10-10
status: resolved
phase: "gui-rosbag-replay - Phase 3 (replay UI)"
related_plan: gui-rosbag-replay.md
files_changed:
  - ros_ws/gui/css/charts.css
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/js/replay/engine.js
  - ros_ws/gui/js/replay/ui/trackbar.js
  - ros_ws/gui/js/telemetry-charts.js
  - tests/ros/js/replay_chart_harness.js
  - tests/ros/js/replay_engine_harness.js
  - tests/ros/js/replay_ui_trackbar_harness.js
  - tests/ros/test_gui_replay_chart.py
  - tests/ros/test_gui_replay_engine.py
  - tests/ros/test_gui_replay_ui_trackbar.py
  - logbook/2026-10-10-gui-replay-phase3-owner-feedback.md
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - testing
---

# GUI replay Phase 3 owner feedback

## Summary

The owner's first real session (2026-10-10, Win10 desktop Chrome over Ethernet) worked and raised five points; four are fixed here, the fifth dissolves under a Phase 4 decision.

## Problem

1. Replay banners sat under the expanded state-machine strip.
2. Charts resized when zoomed.
3. No playhead marker inside the charts.
4. Scrubbing the trackbar lagged.
5. No ETA / no partial viewing while a recording converts (dissolves: ticket 06's direct-MCAP decision, Phase 4, removes the convert-first wait).

## Root Cause

1. The replay banners lacked the `--minimap-width` inset rule the live overlays have (`css/replay.css`).
2. `#chart-window-select` sized to its selected option, so a zoom changed its width and wrapped the toolbar, resizing the charts. Pre-existing in live mode too.
3. Not built yet.
4. `engine.scrub` ran the full reset plus latest-record dispatch of every state topic per mousemove: 22.9 ms mean, 94.8 ms max handler time per step (headless).

## Fix

- (1) Added the `--minimap-width` inset to the banners in `css/replay.css`.
- (2) Fixed width on `#chart-window-select` in `css/charts.css`.
- (3) A uPlot draw hook installed on replay entry draws a line in every chart from the playhead VALUE (not the box centre); colour is the trackbar's `--replay-playhead` variable (`telemetry-charts.js`, exported `playheadCanvasX`).
- (4) `engine.scrub` is now the light path (playhead only); a drag must end in `seek`, documented in the `engine.js` header. The trackbar coalesces to one scrub per animation frame and runs one `seek` on release or after a ~150 ms rest.

## Verification

- Tests added/changed: `test_playhead_line_hook_installs_draws_from_value_and_removes` and `test_chart_dom_contracts_for_feedback_unit_a` (`test_gui_replay_chart.py`); `test_scrub_is_light_and_seek_is_the_full_pipeline` (`test_gui_replay_engine.py`); `test_drag_coalesces_to_one_scrub_per_frame_then_seeks_on_release` and `test_drag_rest_runs_the_full_seek_once` (`test_gui_replay_ui_trackbar.py`).
- 2026-10-10, `python -m pytest tests/ros -k "gui or replay" -q`: 371 passed, 1 skipped.
- 2026-10-10, headless smoke driver `smoke3.mjs`: all checks PASS (a, 1-6, b-h) after each unit.
- 2026-10-10, drag measurement (60 mousemoves 10% to 80% then release, headless Chromium): handler per step before mean 22.9 ms / max 94.8 ms, 1 seek; after mean 0.27 ms / max 1.5 ms, 1 seek.
- Full gate (`./run_tests.sh`, run 2026-10-10 in the replay worktree): parallel **6415 passed, 9 skipped in 267.04 s**; serial 3 passed in 10.22 s; PASS.

## Outcome

Four points fixed; point 5 deferred to Phase 4. Unverified: real-GPU frame time (headless frames ~440 ms under software WebGL); the chart-store rebuild per arriving chunk during a drag is not coalesced; the owner's narrower window may still wrap the toolbar (next suspect `#signal-toggles`).
