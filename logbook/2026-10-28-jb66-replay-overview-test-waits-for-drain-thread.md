---
title: "JB-66: the replay_overview requeue test faked backend state while the first pass's drain thread was still alive"
type: bugfix
date: 2026-10-28
status: resolved
phase: "JB-66 - replay_overview test load flake"
files_changed:
  - tests/ros/test_replay_overview.py
subsystem:
  - gui
  - tests
tags:
  - load-flake
  - replay
---

## Summary

`test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue` failed with `assert [] == ['2026-10-01_10-00-00']` in at least four gates (`temp/logs/gate_iplan10_20261019.log`, `run_tests_full_phase2.log`, `gate_archivals_20261023.log`, and the 2026-10-27 run in the item).

How it fails:
1. `poll()` returns at the first 200. The pass `os.replace()`s its output before it exits, so the first pass's real drain thread is still alive at that point.
2. The test then fakes `_running`/`_running_ver`/`_thread = object()`.
3. If the dying thread reaches its post-pass block after that, it clears all three under the lock.
4. The test's GET then sees `_thread is None` and starts a real drain thread, which pops the requeued Sitting before the queue assertion runs.

**This was a fixture artefact, not a backend race.** In production that thread is the real one, and a GET that queues the new version and starts a fresh drain thread is the correct behaviour. A GUI user never saw anything wrong, so `ros_ws/gui/replay/api.py` is unchanged.

Fix: a `drained(srv)` helper waits until `_thread` is back to None, re-reading it under the lock and joining the live thread. The two tests that fake drain state after `poll()` now call it before faking anything. The second test, `test_listing_reports_ready_when_the_cache_is_valid_while_the_pass_is_still_running`, never failed, because "ready" wins regardless. But the same thread could clear its faked `_running`, so it could pass without checking what it claims to.

## Verification

- Plain CPU load does not open the window. On 2026-10-28, eight `while True` loaders with the test run 8 times in a loop gave 8/8 pass.
- Deterministic reproduction (2026-10-28, temporary probe, deleted): wrap `replay.api.subprocess.run` to sleep 1.5 s after the worker returns, fake the state, `join()` the captured live thread, GET, then sleep 0.2 s. Result: **`AssertionError: assert [] == ['2026-10-01_10-00-00']`**, the gate's exact failure. After the join, `_running=None _thread=None`: the faked state had been cleared.
- After the fix, on 2026-10-28, `pytest tests/ros/test_replay_overview.py -q` run three times: **11 passed** each time.
- Gate: the pre-commit `./run_tests.sh` triple is in this entry's commit message.
