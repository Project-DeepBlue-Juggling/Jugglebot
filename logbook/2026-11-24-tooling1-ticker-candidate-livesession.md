---
title: "Three tooling fixes: the nightly ticker calls a missed 02:00 slot STALE (BUG-100), a candidate-build merge-abort test (BUG-117), the live-session guard matches node process shapes not any argv (BUG-85)"
type: bugfix
date: 2026-11-24
status: complete
phase: "tooling"
related_plan: ""
related_issues: []
files_changed:
  - tools/nightly_ticker.sh
  - tests/sim/test_nightly_ticker.py
  - tests/sim/test_night_shift_candidate.py
  - tools/lib/live_session.sh
  - tests/sim/test_live_session_guard.py
subsystem: [tooling]
---

## What and why

**BUG-100 (missed slot).** `tools/nightly_ticker.sh` called a status STALE only after two days, so one missed 02:00 run was invisible. The LOWERED path now also reports STALE when the status iso-date is before the latest scheduled slot: the most recent 02:00 local time at or before `now - 1800 s`, less 300 s timer skew. The grace keeps a run still in progress at 02:10 (previous status is the previous slot's) from reading STALE; a late Persistent catch-up run and a DEFERRED status (written at budget expiry) carry later dates and pass. `NIGHTLY_TICKER_NOW` (epoch) is a fake clock, `NIGHTLY_SLOT_HHMM/GRACE_S/EARLY_S` tune it. Only the LOWERED verdict gains it: CLAIMED and UNRAISED print exactly as before (an unclaimed old result is CLAIMED once, then the next check says STALE). Claim expiry (84e05d53) is untouched.

**BUG-117.** New test `test_conflict_is_aborted_so_a_later_branch_still_merges`: BUG-4 conflicts with BUG-1, BUG-5 is tried after it and must be MERGED. The worktree is removed at the end of a run, so the effect is observed through the later merge, not through MERGE_HEAD afterwards.

**BUG-85 (cheap fix only).** With the default regex, a `pgrep -f` hit is now only a candidate, confirmed by its `/proc/<pid>/cmdline` shape (`_live_argv_is_robot`): (1) argv[0], or `python*` argv[1]/[2] (after `-u`/`-B`), has basename exactly `teensy_bridge_node|trajectory_node|orchestrator_node`; or `python -m jugglebot.<node>`; (2) the same positions with `jugglebot_launch.py`; (3) `ros2` in those positions, then `launch`, then `jugglebot` or a `jugglebot_launch.py` path. Shapes were taken from the launch files and logs: `~/.ros/log/*/launch.log` lists `teensy_bridge_node-11` etc. as launch-started executables, and Foxy console-scripts run as `python3 .../lib/jugglebot/<node> --ros-args`; no robot launch was running to read a live `ps`, so that is unchecked against a live process (the tests spawn the same argv shapes). An overridden `LIVE_SESSION_RE` keeps the old any-argv match; the default regex string is unchanged. Not matched any more (the point): `sed/grep/tail/vim` of a node file, a shell whose command text names a node, `ros2 topic list teensy_bridge_node`, `ros2 launch other_pkg`. Residual: an argv shape not listed (for example a node started via `ros2 run jugglebot trajectory_node` still shows as its child entry point, matched by 1) would be a false negative; the lock-file design (BUG-101) is not built.

## Verification

- 2026-11-24, `pytest tests/sim/test_nightly_ticker.py tests/sim/test_pull_status.py -q`: 19 passed. Mutation (slot comparison replaced by `false`): `test_missed_slot_is_stale_after_grace` fails; restored, 10/10.
- 2026-11-24, `pytest tests/sim/test_night_shift_candidate.py -q`: 10 passed. Mutation (the `git merge --abort` line commented out): the new test fails (no MERGED BUG-5 row); restored.
- 2026-11-24, `pytest tests/sim/test_live_session_guard.py -q`: 17 passed. Mutation (shape check replaced by `true`): `test_default_regex_ignores_a_command_naming_a_node_end_to_end` fails; restored.
- 2026-11-24, `pytest tests/sim/test_live_session_guard.py tests/sim/test_night_shift_runner.py -q -k "live or session or defer"`: 20 passed. Full gate not run (orchestrator integrates).
