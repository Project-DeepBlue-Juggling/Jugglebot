---
title: GUI - trail slider in the live command bar; the disconnected "Not connected" banner removed
type: feature
date: 2026-10-11
status: resolved
phase: "gui-rosbag-replay - Phase 5b follow-up (owner ask)"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/2026-10-11-gui-trail-slider-in-command-bar.md
  - logbook/INDEX.md
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/css/viewer.css
  - ros_ws/gui/index.html
  - ros_ws/gui/js/replay/ui/lobby.js
  - ros_ws/gui/js/trail-settings-ui.js
  - tests/ros/js/replay_ui_lobby_harness.js
  - tests/ros/test_gui_replay_ui_lobby.py
  - tests/ros/test_gui_trails_wiring.py
subsystem:
  - gui
tags:
  - feature
---

# GUI - trail slider in the live command bar; the disconnected "Not connected" banner removed

## Summary

Owner ask (2026-10-11): "during a live session, the Trail slider should be in the same banner as the 'Activate'
'Home' etc. buttons (though still right-aligned)" and "remove the 'Not connected to the robot ....' banner when there
is no live session and no recording is loaded". The one `#trail-overlay` element now moves between two parents on the
`replay-active` class `lobby.js` sets: inside `#command-overlay`, right-aligned, outside replay; above the replay dock
in replay, as before. The banner showed only in the disconnected lobby, so it is deleted with its offset rule; the
separate backend-unreachable banner stays.

## Verification

- Scoped (`pytest tests/ros -q -k "gui or replay"`, run 2026-10-11): 515 passed, 1 skipped.
- Headless Chromium (run 2026-10-11): lobby with no banner and the slider in the bar; replay unchanged; the live
  button row forced visible: 1600 px one row, no overlap. At 1200 px a fixed 280 px slider wrapped the five buttons
  to three rows, so the slider's width scales with the bar (30 %, 180..280 px): two rows, no overlap. A connected
  session cannot run headless; the live row was simulated by dropping the lobby class.
- Full gate (`./run_tests.sh`, run 2026-10-11 in the `Jugglebot-replay` worktree, log `temp/logs/gate_trail_bar_2026-10-11.log`): **6630 passed, 9 skipped; serial 3 passed; RESULT PASS**.
