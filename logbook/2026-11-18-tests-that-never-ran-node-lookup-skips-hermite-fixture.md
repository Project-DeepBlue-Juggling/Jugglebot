---
title: "59 GUI tests that skipped every night now run: one shared Node lookup, two permanent skips deleted, a committed fixture for the hermite cross-check (BUG-103)"
type: bugfix
date: 2026-11-18
status: complete
phase: "maintainer review"
subsystem: tests
files_changed:
  - tests/ros/_node.py
  - tests/ros/test_gui_*.py and tests/ros/test_replay_*.py (38 files, lookup only)
  - tests/motion/test_unified_cycle.py
  - tests/firmware/test_hermite_xref.py
  - tests/fixtures/hermite_xref/mpc_20260524_153227.csv
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - run_tests.sh
  - map/bug-tests-that-never-run-or-cannot-fail-two-permanent-skips-node.md
  - logbook/INDEX.md
---

## What and why

Three findings of BUG-103, tests that looked like protection and were not. Its other two
findings (the uncollected calibration script, the loose ROS mocks) were split into their
own Items the same day.

1. **Node lookup.** Thirty-eight files under `tests/ros/` locate Node. Thirty carried a
   copy of a finder that also searches `~/.nvm`; eight used a bare `shutil.which`. The
   04:00 systemd run has no Node on `PATH`, so those eight files (59 tests) skipped on
   every nightly run from 2026-10-17 to 2026-11-16 (31 nights of junit). All 38 now
   import `find_node` from `tests/ros/_node.py`. The 59 tests had not rotted: they pass.
2. **Skip reasons in the gate.** `run_tests.sh` passes `-rs` in both phases, so a mass
   skip is printed with its reason instead of being a bare count.
3. **Two permanent skips deleted** from `tests/motion/test_unified_cycle.py`, with the
   `_replan_last_catch` helper only they used. They were unconditional skips since
   2026-09-24 whose own reason said the port did not reproduce the claim. What they
   meant to pin: (a) the wider gate range at a truncated-head splice is covered by the
   running `test_unified_cycle_splice.py::test_a_truncated_head_splice_gates_the_straddle_at_k_s_minus_1`;
   (b) the union-range label is behaviour the code no longer has (the skip reason
   measured `report_range_knots is None` on every input). A comment in
   `unified_cycle.py` cited the deleted test as the one driving `_VALIDATE_STENCIL_KNOTS`;
   it now cites the running one. Comment only, no behaviour change.
4. **Hermite cross-check fixture.** `test_recorded_stream_matches_motor_guard` replayed
   the newest `temp/logs/mpc_*.csv` (gitignored logs of the MPC chain deleted
   2026-09-01) and skipped when there was none. It now replays a committed fixture: the
   200-row recording the xref README's acceptance figure was measured on, cut to the
   twelve columns the replay reads. Full file and fixture give the same result,
   `(0.0, 0.0, 200)`. The skip path is gone and the assertion is `n == 200`.

## Verification

- 2026-11-18, `env PATH=/usr/bin:/bin python -m pytest tests/ros/test_gui_clock.py tests/ros/test_gui_vendor_pin.py tests/ros/test_gui_replay_ui_hide.py -q -rs`: **30 passed, 0 skipped**.
- 2026-11-18, the eight formerly skipping files plus two others with nvm stripped from `PATH`: **93 passed, 0 skipped**.
- 2026-11-18, `flock /tmp/jugglebot-tests.lock ./run_tests.sh` from a scratch worktree:
  **10016 passed, 1 failed, 61 skipped, 1 xfailed in 718 s; serial 7 passed**. The one
  failure is `tests/sim/test_no_legacy_item_ids.py::test_the_tree_holds_no_old_item_id`,
  already failing on `main` (BUG-109, the Night Shift history ledger) and untouched here.
  Skips fell from 118 on the 2026-11-16 nightly; ten of the 61 are worktree artefacts
  (no `temp/logs`, no sibling BallButler checkout) that the new `-rs` output names.
