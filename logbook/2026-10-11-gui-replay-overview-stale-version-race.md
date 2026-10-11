---
title: GUI replay - a pass still exiting for an older file version reported "computing" and swallowed the requeue
type: bugfix
date: 2026-10-11
status: resolved
phase: "gui-rosbag-replay - archival gate (found by the gate)"
related_plan: gui-rosbag-replay.md
files_changed:
  - ros_ws/gui/replay/api.py
  - tests/ros/test_replay_overview.py
  - logbook/2026-10-11-gui-replay-overview-stale-version-race.md
  - logbook/INDEX.md
---

# GUI replay: overview pass state is now keyed to the file version

**What.** The overview queue's `_running` named a recording id but not which version of the file. Between the
pass's `os.replace()` and `_drain` clearing `_running`, a change to the file made the listing report `computing`
for a version nobody was computing. `_enqueue` also answered `computing` without queueing, so the new version only
got queued on a later poll. `_running_ver = (size_bytes, mtime_ns)` is now recorded once the pass has resolved the
file. `_running_for(info)` matches only that version, or any version while the pass is still resolving it.

**Why.** The archival gate failed with `test_route_202_then_200_cached_then_recomputed_on_mtime_change`
(`'computing' == 'none'`, after the `utime`). That is the mirror image of the race fixed in
[2026-10-11-gui-replay-overview-listing-race](2026-10-11-gui-replay-overview-listing-race.md), and it surfaced under
the heavier load of a peer session's tests (the gate ran 447 s against the usual ~294 s). The earlier fix made the
listing check the cache first; with a stale cache the check falls through to the running flag, which is where this
race bit.

**Verification.**
- Deterministic regression test `test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue`.
  It pins the old-version running state with no drain thread, and checks that the listing says `none`, that a GET
  queues with `202 queued`, and that a version still being resolved reads `computing`.
- `pytest tests/ros/test_replay_overview.py -q` (run 2026-10-11): **11 passed**.
- Gate: `./run_tests.sh` (run 2026-10-11, with the archival edits and a peer session's uncommitted skill_node work in the tree): **6666 passed, 9 skipped**, serial **3 passed**.
- Takes effect on the box at the next `jugglebot-gui.service` restart.
