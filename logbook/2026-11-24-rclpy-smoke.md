---
title: "A real-rclpy smoke tier (FEAT-55) and the BUG-47 reproduction: the bridge lingers 20 s after SIGINT because the executor worker is still inside the homing monitor loop"
type: feature
date: 2026-11-24
status: complete
phase: "offline work"
related_plan: ""
related_issues: []
files_changed:
  - tests/rclpy_smoke/__init__.py
  - tests/rclpy_smoke/_child.py
  - tests/rclpy_smoke/test_rclpy_smoke.py
  - logbook/2026-11-24-rclpy-smoke.md
subsystem: [tests, ros, can-bridge]
---

## Summary

First tests that run nodes under real Foxy rclpy (every `tests/ros/` test runs on mocks). Each test starts one node in a child process (`ROS_DOMAIN_ID` random 80..101, `ROS_LOCALHOST_ONLY=1`, node source from this tree, interfaces from `ros_ws/install`), waits for a topic or service in the graph, sends SIGINT and bounds the exit. Marked `nightly` and `serial` (the `live_session` guard matches any process whose command line names a node, and other tests spawn such processes under xdist, so only the serial tail gives a stable answer); skips cleanly without Foxy or built interfaces, and skips while `live_session` reports a live session. Production code is unchanged.

Safety of the bridge child: `teensy_bridge_node.main()` builds its own client from config constants (real board address, local bind 0.0.0.0:5005/5006). `_child.py` replaces `teensy_bridge_node.TeensyLinkClient` with a factory that ignores the node's arguments and binds 127.0.0.1 on ephemeral ports against the test's own `FakeTeensy` (shared with `tests/teensy_link`); it exits 99 before any socket exists if the loopback peer variables are missing (`test_bridge_child_refuses_without_loopback_override`).

## BUG-47 reproduction

`test_bridge_node_exit_with_home_action_in_flight_bug47` (xfail strict): the fake ACKs HOME then goes silent, a `home_motors` goal is accepted, SIGINT follows at once. The process exits **21.0 s and 20.8 s after SIGINT** (two runs). A SIGUSR1 `faulthandler` dump at +1.4 s shows the main thread already in `concurrent.futures.thread._python_exit` (the atexit join, so `main()` had returned) and the executor worker inside `rclpy/action/server.py _execute_goal` -> `_home_action_execute` -> `_run_home`. `_run_home`'s per-axis monitor loop (`mon.step` / `time.sleep(poll_dt)`, timeout `min(20, HOMING_MOTOR_TIMEOUT_S - 5)` plus 5 s hard deadline) never checks any stop flag, and `on_shutdown()` stopping the client does not end it (the loop polls cached telemetry, not RPC); it ended only when the 20 s homing timeout fired. The hypothesis in BUG-47 is right in effect and wrong in detail: the wait is the interpreter-exit join of the executor's worker thread, not `executor.shutdown()`. With a callback that fails fast (silent peer, HOME times out) the same node exits in 1.0 s.

Proposed fix, not applied (safety path, and aborting homing needs a decision about what the axis does): a shutdown event set at the top of `on_shutdown()`, checked in `_run_home`'s loop, which abandons the monitor and commands the homing axis to IDLE.

Also seen: SIGINT during the constructor's boot-state read (up to ~5 s on a dead link) lands outside `main()`'s `try` and dies with a KeyboardInterrupt traceback (exit -2), skipping `on_shutdown`.

## Verification

- 2026-11-24, `pytest tests/rclpy_smoke -q -p no:cacheprovider` in the worktree, with no live session: 3 passed, 1 xfailed in 31.5 s (earlier full run 53.1 s).
- Mutation: probing `cone/nope` instead of `cone/timing_result` made `test_catch_correlation_node_comes_up_and_exits_on_sigint` fail (restored). The BUG-47 test fails its 15 s bound unmutated (hence the strict xfail).
- No child left: `pgrep -af _child.py` returned nothing after the runs.
