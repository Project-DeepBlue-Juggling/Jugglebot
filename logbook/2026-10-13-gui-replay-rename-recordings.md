---
title: GUI - rename recordings from the replay picker; the "hidden while disconnected" note removed
type: feature
date: 2026-10-13
status: resolved
phase: "gui-rosbag-replay - follow-up (owner ask)"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/2026-10-13-gui-replay-rename-recordings.md
  - logbook/INDEX.md
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/js/replay/ui/format.js
  - ros_ws/gui/js/replay/ui/lobby.js
  - ros_ws/gui/js/replay/ui/picker.js
  - ros_ws/gui/replay/api.py
  - ros_ws/gui/replay/recordings.py
  - tests/ros/_replay_srv.py
  - tests/ros/js/replay_ui_lobby_harness.js
  - tests/ros/test_gui_replay_ui_lobby.py
  - tests/ros/test_replay_rename.py
subsystem:
  - gui
tags:
  - feature
---

# GUI - rename recordings from the replay picker; the "hidden while disconnected" note removed

## Summary

Owner ask (2026-10-13): rename recordings from the GUI, renaming the actual files, and drop the lobby's
"Home / Level / Activate are hidden while disconnected" note.

- **Rename** renames the files on disk. A ✎ on each picker row (none on in-progress rows) opens an inline box (Enter / save,
  Esc / cancel) and calls `POST /api/replay/recordings/<id>/rename {"name": ...}`. The folder becomes
  `<timestamp>_<slug>`, and the `.mcap` is renamed to match when it carries the old id as its prefix, as rosbag2
  writes it. Every reference to the old `.mcap` filename in `metadata.yaml` is rewritten, so `ros2 bag play` still
  opens the bag. The cached overview and any remembered overview failure move to the new id. A blank name restores
  the bare timestamp. A second rename replaces the name; it does not stack.
- **Why the timestamp stays (owner choice):** the id is the folder name. The newest-first sort, the JS
  `dateFromId` fallback and other tools (`tools/probes/*`, `~/Desktop/rosbags/<timestamp>`) all key on that prefix.
  `valid_id` now accepts an optional `_[A-Za-z0-9][A-Za-z0-9._-]{0,79}` suffix. `slugify` maps any other run of
  characters to `-`.
- **Refusals:** the target already exists (409), in progress (409), the overview pass is running or queued for that
  id (409, it holds the bag and writes under the id), unknown (404), bad body (400). Order is mcap → metadata →
  folder, and each step is undone if a later one fails.
- **Note removed:** the lobby is just the "Open recording…" button; the unused `.replay-note` rule is gone.
  `test_replay_mock.html` (historical visual spec) is unchanged.

## Verification

- Scoped (`pytest tests/ros -q -k "replay or gui" -n 4`, scratch venv in a cloud container, run 2026-10-13):
  560 passed, 3 skipped; 2 failed + 16 errors, all teensy_bridge/orchestrator modules that fail identically with
  this change stashed (`quaternion` and ROS stubs absent in that container). New: `tests/ros/test_replay_rename.py`
  23 passed; `test_gui_replay_ui_lobby.py` 24 passed.
- Headless Chromium (run 2026-10-13): picker against a real `gui_server.py` on three fake bags; ✎ → typed
  "three ball PB" → Enter renamed the folder to `2026-10-12_23-24-02_three-ball-PB`, the `.mcap` and
  `metadata.yaml` to match, and the list reloaded.
- **Not run:** `./run_tests.sh` (needs the PDJ venv on the Jetson). The GUI service needs a restart to pick
  up the new route.
