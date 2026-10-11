---
title: GUI replay Phase 5 - balls and ribbon trails in the 3D scene, live and replay
type: feature
date: 2026-10-11
status: resolved
phase: "gui-rosbag-replay - Phase 5 (balls and trails)"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/2026-10-11-gui-replay-trails-phase5.md
  - logbook/INDEX.md
  - plans/active/gui-rosbag-replay.md
  - ros_ws/gui/css/viewer.css
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/marker-palette.js
  - ros_ws/gui/js/mocap-markers.js
  - ros_ws/gui/js/replay/mode.js
  - ros_ws/gui/js/replay/policy.js
  - ros_ws/gui/js/replay/trail-window.js
  - ros_ws/gui/js/replay/wiring.js
  - ros_ws/gui/js/trail-feed.js
  - ros_ws/gui/js/trail-settings-ui.js
  - ros_ws/gui/js/trail-settings.js
  - ros_ws/gui/js/trails-scene.js
  - ros_ws/gui/js/trails.js
  - ros_ws/gui/replay/schema.py
  - tests/ros/js/replay_mode_harness.js
  - tests/ros/js/replay_trail_window_harness.js
  - tests/ros/js/trail_feed_harness.js
  - tests/ros/js/trail_layer_harness.js
  - tests/ros/test_gui_replay_mode.py
  - tests/ros/test_gui_replay_trail_window.py
  - tests/ros/test_gui_trail_feed.py
  - tests/ros/test_gui_trail_layer.py
  - tests/ros/test_gui_trails_wiring.py
subsystem:
  - gui
tags:
  - feature
  - testing
---

# GUI replay Phase 5 - balls and ribbon trails in the 3D scene, live and replay

## Summary

The 3D scene now draws the tracked balls (`/balls`) as spheres and a fading ribbon trail behind every ball and every
mocap marker, live and in replay, with one "Trail length" setting (0..5 s, default 1 s) in the scene menu beside the
camera presets. This builds the owner's picks on the ticket 04 prototype (ribbon, dim AND thin with age, label colours
for markers, a hue per ball id, fade over the tail). Replay feeds the trails from the resident chunk columns at
recorded rate, not from the engine's gated dispatch. The design is § Phase 5 of `plans/active/gui-rosbag-replay.md`.

## Discussion

The survey before the build forced four decisions the prototype could not see, because it ran on synthetic data.
Each is recorded with its cause in the plan (D1-D4).

- **D1, ball tracks end on staleness as well as absence.** `ball_tracker_node` publishes `/balls` only while a ball
  is tracked, so the last ball never sees a message without its id. A track also ends, at its last message time, after
  150 ms of silence. The audit then found the same rule must run at the start of every `/balls` message, or a rebuilt
  replay window that spans a gap ends the old track at the new message's time.
- **D2, 10 ms per-track decimation.** Real `/mocap_data` is recorded at about 197 Hz (busy bag `2026-10-02_12-43-28`),
  twice what the prototype's ring was sized for. Decimating keeps the 640-sample ring and halves the work.
- **D3, the duplicate-trail rule.** The tracked ball is also an unlabelled mocap marker. An unlabelled marker within
  40 mm of a fresh ball gets no trail and no nearest-neighbour slot.
- **D4, one trail writer per mode.** Replay dispatches `/mocap_data` through the live handler, so its trail lines are
  skipped while `clock.isReplay()`. The columns window is then the only writer and supplies the render time, since a
  scrub never moves `clock.now()`.

Two tradeoffs were accepted. The layer re-uploads a written track's whole buffer (about 70 KB) instead of a per-sample
update range, because the range objects allocate per push. And trail times are float32 relative to a track's first
sample, which loses fade resolution only for a track alive for about a day.

The smoke on the busy bag overturned one sizing assumption. The bag carries 21 persistent marker tracks and peaks at
46 live tracks at a 5 s tail and 8x, so the 32-track pool dropped hundreds of pushes. The pool is 64.

## Fix

- New modules: `trails.js` (the ribbon layer), `trail-feed.js` (keying and D1-D3), `trail-settings.js` and its UI row,
  `marker-palette.js` (the label palette moved out of `mocap-markers.js`), `trails-scene.js` (singleton, ball spheres,
  one `onFrame` render), `replay/trail-window.js` (append for a forward step within the tail, rebuild otherwise).
- `main.js` subscribes `/balls` at 20 ms and feeds the trails live only; `schema.py` moves `/balls` from PLANNED to
  SUBSCRIBED. `mode.js` creates the window on entry, forwards both engine hooks, and makes the seek pre-roll cover the
  tail.
- Audit fixes (one pass, 2026-10-11): tail 0 no longer hides the replay ball spheres (the window feeds at least the
  150 ms staleness span); a vacuous layer test now pins "no line across a revived track's gap"; the nearest-neighbour
  search no longer reuses a slot within one message; a wall-clock bound left the parallel gate.

## Verification

- Scoped (`pytest tests/ros/test_gui_trail_*.py tests/ros/test_gui_replay_trail_window.py tests/ros/test_gui_trails_wiring.py tests/ros/test_gui_replay_mode.py tests/ros/test_gui_clock_contract.py -q`, run 2026-10-11): **49 passed**.
- Headless Chromium smoke on the busy bag (run 2026-10-11, driver kept in the session scratchpad): 7 of 8 items PASS
  (live settings row, seeks, 1x and 8x play, reverse and scrub, tail 0 and 5 s, exit); heap after GC 6.5 MB before
  entry, 27.0 MB after 30 s of play, 8.8 MB after exit. The paused-replay allocation check was inconclusive: garbage
  grows equally with trails on and off, so it comes from elsewhere. The screenshot shows a coloured ribbon arcing from
  Ball Butler with the ball sphere at its head.
- Replay rebuild of a 5 s window on the busy bag under node: median 1.44 ms, max 8.8 ms (1,926 records).
- Full gate (`./run_tests.sh`, run 2026-10-11 in the `Jugglebot-replay` worktree, log `temp/logs/gate_phase5_2026-10-11.log`): **6601 passed, 9 skipped; serial 3 passed; RESULT PASS** (306 s).

## Outcome

Phase 5 is software-complete. Open for the owner: a live throw to check that D3's 40 mm gate holds at the live
throttles (`/mocap_data` 50 ms, `/balls` 20 ms stamp at receipt, so a fast ball may sit beyond 40 mm of its own
marker and grow a duplicate trail; the audit could not confirm this without live data), and a look at the colours
and widths in the owner's own browser.
