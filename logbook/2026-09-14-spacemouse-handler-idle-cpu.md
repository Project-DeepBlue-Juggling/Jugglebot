---
title: spacemouse_handler — mode-scoped timer removes always-on idle CPU
type: optimization
date: 2026-09-14
status: resolved
files_changed:
  - ros_ws/src/jugglebot/jugglebot/spacemouse_handler.py
  - tests/ros/test_spacemouse_handler.py
  - tests/ros/test_ros_mock_hygiene.py
  - tests/ros/conftest.py
subsystem:
  - ros
tags:
  - performance
  - spacemouse
  - hid
---

# spacemouse_handler — mode-scoped timer removes always-on idle CPU

## Summary

`spacemouse_handler` held a 100 Hz timer and a `while rclpy.ok():
spin_once(timeout_sec=0.01)` loop unconditionally, whether or not
SPACEMOUSE was the active control mode — `pidstat` measured this at
7.3-8.1% of a core in the live launch with no device connected
(2026-09-13; 5.9% standalone on an idle Jetson). The cost was the timer + spin loop itself, not HID
scanning (`pyspacemouse.open()` with no device measured 0.36 ms). The
node now creates the timer and opens the device only while SPACEMOUSE
is the active mode (per `control_mode_topic`); outside it, the node's
only live entity is that subscription and nothing reads HID.

## Approach

`control_mode_callback` is now the lifecycle boundary: entering
SPACEMOUSE resets the reconnect limiter (so the first open attempt is
immediate) and creates the 100 Hz tick timer; leaving destroys the
timer and closes the device if open. The 100 Hz tick itself is
unchanged from before — same transform, same home-pose-on-absent/lost
behaviour — it just only exists while in mode, so the old
not-in-mode "drain the HID buffer" branch is dead code and was
deleted. `main()` now calls `rclpy.spin(node)` instead of the
`spin_once` poll loop.

**Control-flow check.** In SPACEMOUSE mode the publish stream is
identical to today: 100 Hz, same axis transform, home pose while the
device is absent or lost — `trajectory_node` (the follower) sees no
change. Outside the mode, `trajectory_node._on_platform_pose` already
drops `platform_pose_topic` (it only accepts a follower mode whose
`publisher` field matches), so removing the idle work changes nothing
downstream; if this node goes silent entirely, `follower_input_loss_s`
(0.4 s) is the existing backstop. One behavioural nuance: the device
now opens fresh at mode entry rather than possibly already being open,
so `pyspacemouse` state starts at zero axes (== the home pose) until
the first HID report arrives, where previously an already-open device
kept its last state.

Two mock-harness gaps surfaced writing the tests: the mocked `Node`
(`tests/ros/conftest.py`) had no `destroy_timer` (real `rclpy.Node`
does — this is the first node in the tree to need it), and `rclpy.spin`
wasn't defined on the mock `rclpy` module either. Per-test node
instances get `destroy_timer` bound as a plain function (never a
`Mock`, per `test_ros_mock_hygiene.py`'s hygiene contract) rather than
touching the shared `MockNode` class; `rclpy.spin` is monkeypatched
with `raising=False` for the one test that needs it, mirroring
`test_skill_node.py`'s existing `rclpy.spin` pin. Both
`test_ros_mock_hygiene.py` and `conftest.py` also got a docstring
reword: their examples cited `spacemouse_handler.main`'s `spin_once`
loop by name, and that loop is now gone — reworded generically to
"any `while rclpy.ok(): spin_once` driver".

## Verification

- (2026-09-14, `pytest tests/ros/test_spacemouse_handler.py
  tests/ros/test_ros_mock_hygiene.py tests/ros/test_choreography_map.py
  -q`): **42 passed in 8.98s**.
- (2026-09-14, `pytest tests/ros/ -q`, full scoped ROS suite for
  collateral breakage from the two docstring edits): **2801 passed, 4
  skipped in 207.99s**.
- (2026-09-14, pidstat recipe from the brief — `ROS_DOMAIN_ID=87`,
  node run standalone with a 10 Hz `control_mode_topic` publisher
  holding it in STANDBY, `pidstat -u -p $SM 1 40`): **0.62% average
  CPU** (0.55% usr + 0.07% sys), vs 5.9% for the old loop under the same
  standalone recipe (2026-09-13; 7.3-8.1% in the live launch). All started processes confirmed killed
  via `pgrep -af`.

## Operator guidance

After merge: `colcon build --packages-select jugglebot`, source,
relaunch. The plug-in smoke test is the operator's to run — entering
SPACEMOUSE mode moves the platform home.
