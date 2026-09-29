---
title: "skill_node died mid-attempt on the first missed throw: one rclpy log call site used two severities (Foxy raises ValueError); the test mock now enforces the rule"
type: bugfix
date: 2026-09-29
status: resolved
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/launch_console.py
  - tests/ros/conftest.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_trajectory_node_console.py
  - tests/ros/test_launch_console.py
subsystem:
  - ros
tags:
  - safety
  - testing
---

# skill_node log-severity crash

## Problem

The owner's first launch on operator console phase 2 (`2bb2c030`, 2026-09-29 23:35) lost
skill_node mid-attempt. At 23:36:06 `_on_tick` → `_drain_reports` → `_log_at` raised `ValueError:
Logger severity cannot be changed between calls.`. The timer callback's exception ended
`executor.spin()`, and the process exited with code 1 (`~/.ros/log/2026-09-29-23-35-29-*/launch.log`).
Every installed segment is rest-terminal, so trajectory_node streamed the installed plan to rest.
But nothing dispatched or stopped anything after that until a relaunch.

## Root Cause

Foxy's `RcutilsLogger.log` keys a context on the CALL SITE (function, file, line, bytecode
offset) and raises if a later call from that site uses a different severity, logger name or
filter kwargs. `_log_at` dispatched INFO/WARN/ERROR through one call expression, so the first
MISSED throw (WARN) after a CAUGHT one (INFO) raised. trajectory_node's clock-offset refresh had
the same shape (`log = warning if big else debug; log(msg)`). It would have killed the leg-path
node on the first refresh step of 100 µs or more; none had occurred (max 3.3 µs over 320).

No test saw it because `tests/ros/conftest.py`'s `MockLogger` accepted any call. The phase-2
tests also captured lines through substituted lambdas, which bypass the logger entirely.

## Fix

- `_log_at` and the clock-offset refresh use one call site per severity.
- **The class is closed at the test layer:** `MockLogger` now enforces rclpy's rule (severity and
  filter kwargs fixed per call site, `ValueError` otherwise), so every node test through the mock
  logger checks it. The whole `tests/ros` tree passed under enforcement (2087 passed), and a scan
  of `ros_ws/src` + `teensy_link` found no other call site that picks its log method at runtime.
- Tests: the mock's own guard; caught → missed → caught throw lines then INFO/ERROR/WARN end lines
  through the enforcing mock (verified RED on the old `_log_at`, with the production message); a
  DEBUG → WARNING → DEBUG clock-offset sequence.
- Same commit, console: after launch's Ctrl-C line, a process exit is a yellow "exited during
  shutdown (...)" rather than a red PROCESS DIED. `ros2 bag record` exits 2 on every Ctrl-C
  (ros2cli returns SIGINT's number). A death before Ctrl-C stays red.

## Verification

- Gate, `./run_tests.sh` in the console worktree (on `2bb2c030`), run 2026-09-29 23:49–23:53:
  **PASS, 5622 passed, 9 skipped; serial 3 passed**
  (`temp/logs/gate_log_severity_fix_20260929.log` in that worktree).

## Notes

- The owner's other report, no shutdown lines and no bag line after Ctrl-C with record:=true, was
  not a launch fault. `launch.log` shows a clean shutdown (every node finished cleanly, bag
  `metadata.yaml` written 23:36:18). The screen lost it because the shell's SIGINT also kills a
  plain `| tee`. `tee -i` ignores it and copies until launch exits. The record:=false run that
  showed its shutdown was not tee'd.
