---
title: GUI - rename recordings (picker and the open recording) with the name as the row headline; the "hidden while disconnected" note removed
type: feature
date: 2026-10-13
status: resolved
phase: "gui-rosbag-replay - follow-up (owner ask)"
related_plan: gui-rosbag-replay.md
files_changed:
  - logbook/2026-10-13-gui-replay-rename-recordings.md
  - logbook/INDEX.md
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/js/replay/mcap-decode.js
  - ros_ws/gui/js/replay/mcap-worker.js
  - ros_ws/gui/js/replay/sources.js
  - ros_ws/gui/js/replay/ui/format.js
  - ros_ws/gui/js/replay/ui/index.js
  - ros_ws/gui/js/replay/ui/lobby.js
  - ros_ws/gui/js/replay/ui/picker.js
  - ros_ws/gui/js/replay/ui/rename.js
  - ros_ws/gui/js/replay/ui/trackbar.js
  - ros_ws/gui/replay/api.py
  - ros_ws/gui/replay/recordings.py
  - tests/ros/_replay_srv.py
  - tests/ros/js/replay_rename_live_harness.js
  - tests/ros/js/replay_ui_lobby_harness.js
  - tests/ros/test_gui_replay_rename_live.py
  - tests/ros/test_gui_replay_ui_lobby.py
  - tests/ros/test_gui_replay_ui_trackbar.py
  - tests/ros/test_replay_rename.py
subsystem:
  - gui
tags:
  - feature
---

# GUI - rename recordings (picker and the open recording) with the name as the row headline; the "hidden while disconnected" note removed

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
- **Refusals:** the target already exists (409), in progress (409), unknown (404), bad body (400). (The first
  version also refused while the overview pass was queued or running for the id; the follow-up below lifts that.) Order is mcap → metadata →
  folder, and each step is undone if a later one fails.
- **Note removed:** the lobby is just the "Open recording…" button; the unused `.replay-note` rule is gone.
  `test_replay_mock.html` (historical visual spec) is unchanged.

## Follow-up (same day, owner ask): rename the open recording; the name as the row headline

- **The open recording can be renamed.** Name + ✎ sit beside "Exit replay" in the dock (owner's placement).
  The browser streams the bag by URL, and the URL carries the id. So after the POST, `McapSource.retarget(newId)` moves the
  source's id, the overview poll URL, and the worker's Range URL (out-of-band `{op:'retarget'}` →
  `httpReadable.setUrl`). The ETag still matches because a rename keeps size + mtime_ns. Copy link then names the
  new id.
- **The server answers old ids.** `ReplayBackend._aliases` (old → new, in memory) lets the file and overview routes
  serve a request that was in flight across the rename, or a tab that has not retargeted. A real recording always
  wins over an alias, so renaming back to the bare timestamp cannot loop. The aliases are lost when the service restarts.
- **The overview pass no longer blocks a rename.** Opening a bag queues its pass, so the old 409 would have refused
  almost every rename of an open bag. Now a queued id is re-pointed. A running pass is re-pointed too: `overview_state`
  reports it as computing under the new id, and `_drain` moves its output to the new id when it finishes. A failure
  across a rename is not remembered, so the next poll requeues it. `_drain` skips a pass whose cache is already
  valid.
- **The name is the row headline** (owner's choice from two layouts): a named row leads with the name (larger, cyan),
  with the date under it. An unnamed row keeps the date with "(unnamed)" under it. The first column is 250 px wide
  (was 190). The filter matches "three ball" against `three-ball`.

## Verification

- Scoped (`pytest tests/ros -q -k "replay or gui" -n 4`, scratch venv in a cloud container, run 2026-10-13):
  560 passed, 3 skipped; 2 failed + 16 errors, all teensy_bridge/orchestrator modules that fail identically with
  this change stashed (`quaternion` and ROS stubs absent in that container). New: `tests/ros/test_replay_rename.py`
  23 passed; `test_gui_replay_ui_lobby.py` 24 passed.
- Headless Chromium (run 2026-10-13): picker against a real `gui_server.py` on three fake bags; ✎ → typed
  "three ball PB" → Enter renamed the folder to `2026-10-12_23-24-02_three-ball-PB`, the `.mcap` and
  `metadata.yaml` to match, and the list reloaded.
- Follow-up, scoped (`pytest tests/ros tests/sim/test_logbook_front_matter.py tests/sim/test_plans_index.py -k
  "replay or gui or logbook or plans" -n 4`, same cloud venv, run 2026-10-13): 658 passed, 3 skipped; the same 2
  failed + 16 errors as above (missing deps, not this change). New: `test_gui_replay_rename_live.py` 4 passed,
  `test_replay_rename.py` 27 passed (4 new), lobby 25 passed.
- Follow-up, headless Chromium against `gui_server.py` with the venv overview worker (run 2026-10-13): a 120 s
  synthetic bag (`_replay_fixture.write_bag`) opened by deep link. ✎ → "three ball PB" → Enter renamed the folder,
  the `.mcap` and `metadata.yaml`. Every later request went to `/recordings/<new id>/file`, and the overview JSON landed under
  the new id. A seek + play kept playing with no error toast. One page error (`reading 'topics'` of null) and the
  seek landing at 0.4 s reproduced identically on the never-renamed bag, so they belong to the synthetic fixture or
  headless setup, not this change (not chased).
- **Not run:** `./run_tests.sh` (needs the PDJ venv on the Jetson). The GUI service needs a restart to pick
  up the new route.
