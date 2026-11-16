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
`sys.exit(1)` without building the executor, deterministically. The post-spin `sys.exit(1)`
stays for the rarer ordering where the failure lands mid-spin (a timer callback raising out of
`spin()` in that window still exits non-zero by traceback; accepted, same exit code). The error text
still names the likely stale trajectory_node. `sys.exit` inside the thread was avoided because
it would only end that thread. The Item's second option (an alive flag plus refused installs)
was not needed: the exit is simpler and adds no new state to test.

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
