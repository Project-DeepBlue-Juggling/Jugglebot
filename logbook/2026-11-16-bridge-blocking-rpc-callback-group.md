---
title: "teensy_bridge_node: blocking-RPC services and version polls move to their own callback group (BUG-98)"
type: bugfix
date: 2026-11-16
status: complete
phase: night-shift
related_items:
  - BUG-98
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_blocking_rpc_group.py
  - tests/ros/test_teensy_bridge_node_recover.py
  - logbook/2026-11-16-bridge-blocking-rpc-callback-group.md
---

## What and why

BUG-98 claimed blocking RPC services share a callback group with the 100 Hz publishers and the 1 Hz link-loss check. **Confirmed by reading, with one correction.** `main()` runs a `MultiThreadedExecutor` (`teensy_bridge_node.py` main), so threads exist, but the node-default `MutuallyExclusiveCallbackGroup` still serializes everything left in it: `_publish_robot_state`, `_publish_link_status` and `_health_check` (all `create_timer` without a group) shared it with `reboot_odrives`, `set_hand_state`, `set_hand_gains`, `bb/reload`, `bb/reset`, `bb/calibrate` and both 1 Hz version polls (`_version_check_poll`, `_bb_version_check_poll`, whose own comment said it shared `_publish_robot_state`'s group). Each parks in `_call_rpc` (0.5 s x 3 by default), so a dead bridge stalled telemetry and the link-loss latch for up to 1.5 s. Correction: `odrive_command` was already moved to the `/recover` group on 2026-09-23, so it is not touched. The firmware owns the real safety; this is an observability/timing fix.

Fix: one new `MutuallyExclusiveCallbackGroup`, `_blocking_rpc_cbgroup`, holds those six services and the two version polls. Mutually exclusive (not reentrant) on purpose: the two polls both write the unlocked `self._versions` / `_fw_unreleased`, and keeping them in one exclusive group preserves "never concurrent with each other". The `_publish_link_status` reader (default group) now runs concurrently with those writers; it only does single-entry dict `.get()` (GIL-atomic) and renders `?` for a missing axis, so the worst case is a one-tick advisory string. Verdict bools stay under `self._lock`. Comments updated at the group, the tracker declaration and the poll.

Sitting note: this changes live executor threading, so the branch is held for a Sitting; flown behaviour to check is a normal launch with `/robot_state` at 100 Hz and `ros2 service call` of `bb/reload` / `set_hand_state` with the bridge link cut, watching `/robot_state` and the link-loss latch keep ticking.

## Verification

- Tests: `tests/ros/test_teensy_bridge_node_blocking_rpc_group.py` pins that the six services and both polls are in `_blocking_rpc_cbgroup`, the three publisher/health timers stay in the default group, and no service is left in the default group. Two existing assertions (`reboot_odrives` was `callback_group is None`) in `test_teensy_bridge_node_recover.py` updated.
- The three new tests FAIL with the source change stashed (2026-11-16, `night_shift_nonet.sh nice -n 19 python -m pytest -q tests/ros/test_teensy_bridge_node_blocking_rpc_group.py` with `teensy_bridge_node.py` at the base: 3 failed), pass with it.
- 2026-11-16, `night_shift_nonet.sh nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest -q tests/ros/test_teensy_bridge_node_blocking_rpc_group.py tests/ros/test_teensy_bridge_node_recover.py tests/ros/test_teensy_bridge_node_version.py tests/ros/test_teensy_bridge_node_rpc.py tests/ros/test_teensy_bridge_node_bb.py tests/ros/test_teensy_bridge_node_hand.py tests/ros/test_orchestrator_conduit_contract.py`: **128 passed in 29.73 s**.
- Not run: `./run_tests.sh` (tier 1 for `ros_ws/`; the Night Shift gate owns it). The test mocks rclpy, so real executor scheduling is only exercised at the Sitting.
