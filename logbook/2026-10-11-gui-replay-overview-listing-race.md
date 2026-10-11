---
title: GUI replay - overview listing said "computing" while the route already served the overview
type: bugfix
date: 2026-10-11
status: resolved
phase: "gui-rosbag-replay - Phase 6 (found by the gate)"
related_plan: gui-rosbag-replay.md
files_changed:
  - ros_ws/gui/replay/api.py
  - tests/ros/test_replay_overview.py
  - logbook/2026-10-11-gui-replay-overview-listing-race.md
  - logbook/INDEX.md
---

# GUI replay: overview listing race

**What.** The Phase 6 gate failed `tests/ros/test_replay_overview.py::test_route_202_then_200_cached_then_recomputed_on_mtime_change`
once (`assert 'computing' == 'ready'`); five isolated reruns of the file passed. The failure path:

1. The overview pass subprocess `os.replace()`s its JSON into the cache, then exits.
2. In that window, `_overview` already serves 200 from the valid cache.
3. `_drain` clears `_running` only after `subprocess.run` returns.
4. `overview_state` checked `_running == rid` BEFORE the cache, so the recordings listing still said `computing`.

Under gate load the window was wide enough for the test to read the listing inside it. In production, the picker's
overview badge could lag the strip by that window.

**Fix.** `overview_state` checks the cache first, the same order as `_overview`. `_cached_overview` validates the
cache against the bag's size and mtime, so a re-pass after the bag changed still reports `computing`. A deterministic
regression test sets `_running` by hand over a valid cache and asserts `ready`.

**Verification.**
- 2026-10-11, `python -m pytest tests/ros/test_replay_overview.py tests/ros/test_replay_api.py -q`: 19 passed.
- 2026-10-11, full gate: see the Phase 6 entry
  ([2026-10-11-gui-replay-deep-links-and-marker-history](2026-10-11-gui-replay-deep-links-and-marker-history.md)).

**Deploy.** This is a Python change, so it is inert until the OWNER restarts `jugglebot-gui.service`.
