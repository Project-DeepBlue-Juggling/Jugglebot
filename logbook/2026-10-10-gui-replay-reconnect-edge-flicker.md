---
title: GUI replay panels flickered - rosbridge reconnect-loop edges leaked past the replay mode switch
type: bugfix
date: 2026-10-10
status: resolved
phase: "gui-rosbag-replay - Phase 2 (browser engine)"
related_plan: gui-rosbag-replay.md
files_changed:
  - ros_ws/gui/js/ros-bridge.js
  - ros_ws/gui/js/replay/mode.js
  - tests/ros/js/replay_mode_harness.js
  - tests/ros/test_gui_replay_mode.py
  - logbook/2026-10-10-gui-replay-reconnect-edge-flicker.md
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - testing
---

# GUI replay panels flickered - reconnect-loop edges leaked past the replay mode switch

## Summary

The owner's Phase 2 dev-page session (2026-10-10, Chrome, ROS stack down) showed every panel flickering between placeholders and data during replay. With rosbridge down, the reconnect loop in `ros_ws/gui/js/ros-bridge.js` emits `connecting` / `disconnected` edges every ~2 s. `replay/mode.js` intercepted only the `connected` edge, so each `disconnected` reached `main.js` (`blankDisconnectedState`, `stopTopicDiscovery`, RosLink(false)) and `state-minimap.js` (snapshot and echo-sample wipe); the next replay dispatch then repainted them.

## Fix

The class is every `onConnectionStateChange` listener, and there are exactly two (`main.js`, `state-minimap.js`), so the fix is one enforcement point rather than per-listener guards: `ros-bridge.js` gains `setStateSuppressor(fn)`. While `fn()` holds, a non-`connected` edge is recorded for `getConnectionState()` but not delivered to listeners. `mode.js` installs `() => phase !== 'idle'`. The `connected` pre-notify exit path and the exit sequence's own final blanking are unchanged.

## Verification

- 2026-10-10, `pytest tests/ros/test_gui_replay_mode.py tests/ros/test_gui_replay_bridge.py tests/ros/test_gui_replay_fence.py tests/ros/test_gui_replay_engine.py -q` -> 36 passed.
- New `test_reconnect_loop_edges_never_reach_listeners_during_replay` (harness block 8 in `replay_mode_harness.js`) failed before the fix (6 edges leaked) and passes after.
- Known gap: not yet re-checked in a real browser; the owner re-tests after the next service restart. Unchecked edge: exiting replay while the state is `connecting` leaves listeners last told `disconnected`.
- Full gate (`./run_tests.sh`, run 2026-10-10 in the replay worktree): parallel **6332 passed, 9 skipped in 310.99 s**; serial 3 passed in 11.07 s; PASS.
