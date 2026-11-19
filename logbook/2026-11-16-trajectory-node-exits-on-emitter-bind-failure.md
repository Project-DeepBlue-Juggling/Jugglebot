---
title: "trajectory_node exits non-zero when its emitter cannot bind the :5557 command socket (BUG-96)"
type: bugfix
date: 2026-11-16
status: complete
phase: night-shift
subsystem: skill-stack
related_items: [BUG-96]
files_changed:
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/ros/test_trajectory_node_bind_failure.py
  - logbook/INDEX.md
---

## What

On a bind failure `_emitter_loop` logged "The emitter will NOT start" and returned, but the
node kept spinning and accepting installs, so the operator and the GUI saw a healthy planner.
The emitter thread now sets `emitter_bind_failed` and calls `rclpy.shutdown()`. The emitter
thread starts inside `__init__`, so on real Foxy the failure almost always lands before `main()`
builds the executor, and `MultiThreadedExecutor()` on a dead context raises with `on_shutdown()`
never run (the segment-solver worker would not be closed). `main()` therefore checks the flag
right after constructing the node: it runs `on_shutdown()` and `destroy_node()` and calls
`sys.exit(1)` without building the executor, whenever the bind has already failed by then (the common ordering). The post-spin `sys.exit(1)`
stays for the rarer ordering where the failure lands mid-spin (a timer callback raising out of
`spin()` in that window still exits non-zero by traceback; accepted, same exit code). The error text
still names the likely stale trajectory_node. `sys.exit` inside the thread was avoided because
it would only end that thread. The Item's second option (an alive flag plus refused installs)
was not needed: the exit is simpler and adds no new state to test.

## 2026-11-19: the four audit findings applied (TKT-80)

1. The bind is asynchronous, so `main()` now waits (3 s bound, `_BIND_WAIT_S`) on a `_emit_bind_done` Event that the emitter thread sets on both outcomes (after the flag and the shutdown on failure; after the bind on success). On timeout we proceed rather than fail: a hung bind must not hang startup and may yet succeed. The executor is now built inside the `try/finally`, and an exception from it or from `spin()` with the flag set is converted to the exit-1 path, so a late failure always runs `on_shutdown()` and exits 1, not by traceback.
2. New test for a bind failure during `spin()` (post-spin `sys.exit(1)`), plus one for failure during executor construction.
3. The early exit's `on_shutdown()` is wrapped in `try/finally` so `destroy_node()` and the shutdown still happen if it raises.
4. The early-exit tests now spy on the segment-solver worker's `close()` instead of stubbing `on_shutdown`.

Lifecycle only: nothing on the emit path, its timing or knot content changed.

## 2026-11-20: the two open audit findings applied (TKT-86 option 1)

1. `main()`'s `except Exception` (taken only when the bind-failure flag is set) now logs the swallowed exception with its traceback through the node logger (falling back to `traceback.print_exc()` if the logger fails) before the exit-1 path. Exit code and ordering unchanged; `import traceback` added.
2. Three new tests in `test_trajectory_node_bind_failure.py`: a REAL emitter thread with a failing bind (main's bounded wait sees the verdict, no executor built, worker `close()` ran, exit 1); a bind that BLOCKS with `_BIND_WAIT_S` patched to 0.01 (main times out, builds the executor, the late failure is released from `spin()` and exits 1); and the logged-exception test for item 1. Joins use a 30 s ceiling and no wall-clock asserts.

## Control-system check

The exit happens only when the bind failed, i.e. before any setpoint was ever emitted, so no
stream is interrupted; the firmware's setpoint staleness Watchdog remains the safety authority.
No timing, rate or emission code on the success path changed (the edit sits before the period
and deadline setup, in the failure branch only).

## Launch

`jugglebot_launch.py` declares `trajectory_node` with no `respawn`, so the operator sees the
error line, then the launch's "process has died ... exit code 1" for trajectory_node, instead of
a silent healthy-looking node. A respawn would just collide with the stale holder again.

## Verification

2026-11-16, `night_shift_nonet.sh nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest -q tests/ros/test_trajectory_node_bind_failure.py tests/ros/test_trajectory_node.py tests/ros/test_trajectory_node_rt_param.py`: 161 passed in 14.07s. Two of the three new tests fail with the source change stashed.

2026-11-17 (deterministic exit, TKT-77 option 1), same wrapper and command over `tests/ros/test_trajectory_node*.py`: 174 passed in 10.00s. The new `test_main_early_exit_when_bind_fails_before_executor` (executor constructor raises, never built, `on_shutdown` called, exit 1) fails with the early-exit check disabled. Success path untouched: the added block is guarded by the bind-failure flag, so the executor, emitter and 40 Hz path are unchanged when the bind succeeds.

2026-11-19 (TKT-80 findings), `night_shift_nonet.sh timeout 600 python -m pytest tests/ros/test_trajectory_node_bind_failure.py tests/ros/ -k "trajectory_node" -q -p no:cacheprovider`: 195 passed in 37.89s (8 of the 195 are `test_trajectory_node_bind_failure.py`). With the source reverted to the branch HEAD, 3 of the 8 tests in the bind-failure file fail (`on_shutdown`-raises, late failure during executor construction, bind-done Event); the spin-time test passes on both, since the post-spin exit already existed and was merely unpinned.

2026-11-20 (TKT-86 option 1), `night_shift_nonet.sh timeout 900 python -m pytest tests/ros/test_trajectory_node_bind_failure.py tests/ros/test_trajectory_node.py tests/ros/test_trajectory_node_console.py tests/ros/test_trajectory_node_rt_param.py -q -p no:cacheprovider`: 186 passed in 14.72s. With the source change reverted only the logged-exception test fails; the two real-thread tests pin existing branch behaviour (the wait and the late-failure path) and pass on both.
