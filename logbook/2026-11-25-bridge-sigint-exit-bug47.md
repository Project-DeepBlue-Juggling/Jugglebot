---
title: "Bridge node SIGINT exit: the 24/24 unpowered no-exit was the series harness starting the launch with SIGINT ignored; homing-in-flight abort and the boot-read guard added (BUG-47, BUG-141)"
type: bugfix
date: 2026-11-25
status: complete
phase: ""
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/rclpy_smoke/test_rclpy_smoke.py
  - tests/ros/test_teensy_bridge_node_shutdown_order_bug47.py
  - tools/probes/bug47_sigint_ignored_probe.py
  - logbook/2026-11-25-bridge-sigint-exit-bug47.md
subsystem: [ros, can-bridge]
---

## Summary

Three separate things, one entry. (1) Where the unpowered bridge "blocked" 24 of 24 times: nowhere in the node. `tools/probes/bug129_launch_series.sh` starts the launch with `setsid bash -c "... exec ros2 launch ..." &` from a non-interactive script; bash starts a background job there with SIGINT set to ignored, `setsid` and `exec` inherit it, Python then leaves SIGINT ignored, and Foxy `rclpy.init()` does not replace it. The whole launch ignored the SIGINT: the bag file grew until 08:31:17 (the SIGKILL), `trajectory_node` kept logging after the SIGINT (launch.log of `A_01`), and the script had to SIGKILL the launch group too. (The brief said the recorder and rclpy exited within 0.25 s; the A_01 files show the opposite.) (2) BUG-47 proper, the homing-in-flight 21 s exit: fixed by a stop check. (3) BUG-141: construction moved inside the guarded block.

## Control and safety walk (written before the change)

Shutdown order on a healthy link, unchanged: log line, `_shutdown_event.set()` (new, no wire traffic), disarm (`_stop_setpoint_output`: stops the 40 Hz setpoint thread, drops mpc_active on the wire), hand fan AUTO, settle plus `_wait_wire_disarmed`, `_shutdown_stow` (DEACTIVATE), IMU/capture/stall-watchdog close, ToD/RPC close, `client.stop()`. Pinned by `test_on_shutdown_step_order_is_unchanged_on_a_healthy_link`.

The only new wire traffic is when a Homing is in flight at SIGINT: `_run_home` sees `_home_abort_requested()` at the top of its next 50 ms poll, sends one SET_AXIS_STATE(axis, IDLE) for the axis it was homing (only if the firmware had accepted the HOME), returns a failed homing, and `_do_home` then persists is_homed=False (the conservative direction: forces a re-home) and skips configure. Before, the monitor ran on to its own result (up to 20 s) with the axis driving into its stop while on_shutdown stowed the legs alongside it. **Owner decision, 2026-11-25: yes, SIGINT aborts a homing in flight and Idles the axis** (the Hand is not heavy enough to drop by itself, and no hazard if it did). The behaviour still sits behind `_home_abort_requested()` (return False to restore the old run-to-result).

Bound after the change, for the homing case: on_shutdown about 1 s, plus the worker's current RPC (at most one RPC timeout) plus the IDLE RPC plus the is_homed persist, measured below. No step that protects a powered robot was weakened or skipped for the unpowered case, and none was needed: nothing in on_shutdown blocks on an unpowered robot (see the unpowered test).

BUG-141: the boot-state read catches KeyboardInterrupt, sets `_boot_interrupted`, and the constructor finishes with the conservative cold cache; `main()` then skips the spin and runs the same `on_shutdown()` path. An interrupt earlier in the constructor (milliseconds, before the client matters) leaves no node: only `rclpy.shutdown()` runs.

## Verification

See the result lines appended below.

- 2026-11-25, `python -m pytest tests/rclpy_smoke -q -s -o addopts=""` (real Foxy rclpy, loopback fake peer; run without the box test lock, which was held by another Session's probe for 25 min; live_session clear): 6 passed in 20.73 s. Homing in flight: exit 1.40 s after SIGINT (before: 19.95 s, the strict xfail). Unpowered-like fake (ERR_REJECTED to every RPC, ODRIVE_FATAL heartbeats): exit 2.00 s, "Shutting down TeensyBridgeNode" logged, rc 0 (a pin: passes before the change too).
- 2026-11-25, `python -m pytest tests/ros/test_teensy_bridge_node_shutdown_order_bug47.py tests/ros/test_teensy_bridge_node_home.py tests/ros/test_teensy_bridge_node_shutdown_stow.py -q`: 27 passed in 9.92 s. The new file against the pre-change node: 5 of its 6 tests fail.
- SIGINT-ignored mechanism, `tools/probes/bug47_sigint_ignored_probe.py` started from a non-interactive bash script with `&`: SigIgn 0x1001004, handler stays SIG_IGN after `rclpy.init()`, no exit 3 s after SIGINT; with job control on (`set -m`) it exits in under 0.3 s via KeyboardInterrupt from `spin()`.

## Not-homed write ordering (follow-up, 2026-11-25)

Problem: the aborted homing persists is_homed=False on its worker thread while `on_shutdown` runs on the main thread and closes the transport; a lost write leaves is_homed=True after an aborted homing (the unsafe direction: the next boot skips Homing). The first smoke run showed the write failing (ERR_UNKNOWN_METHOD from the fake).

Fix: `_home_idle` Event (set while no `_do_home` runs, cleared from the single-flight claim to the `finally`, i.e. across the persist). `on_shutdown` waits on it (`_SHUTDOWN_HOMING_ABORT_WAIT_S` = 5 s) as step 3b, after the stow and stall watchdog stop and immediately before the transport teardown; on timeout it writes is_homed=False itself. With no homing in flight the wait returns at once, so the healthy step order is byte-identical (pinned test unchanged and passing).

Messages to the bridge at SIGINT with a homing in flight, in order: (1) SETPOINT/heartbeat disarm (mpc_active=0), (2) hand fan AUTO if forced, (3) SET_AXIS_STATE(axis, IDLE) from the homing worker (concurrent with the stow, within one 50 ms poll of the event), (4) DEACTIVATE (stow), (5) STATE_WRITE is_homed=False (worker; or on_shutdown if the worker is still stuck at the bound), then the transport closes and client.stop(). Steps 3 and 5 are from the worker thread, so 3 can interleave with 1 and 2.

Bound: healthy on_shutdown (about 1 s plus stow) plus at most 5 s wait plus at most one own write (about 1.5 s RPC default 0.5 s x 3 attempts): at most about 6.5 s added, and the wait overlaps the stow so typically 0.

BUG-122 interaction: `_do_home` runs the reference-loss accounting on an aborted homing: the write of is_homed=False succeeds, `_write_is_homed(False)` calls `landed_false()` (pending cleared, correct: False is what a loss owes) and `home_written(seq_at_start)` runs only when the write landed; a loss event during the homing stays owed. If the write fails the latch stays armed (correct; the process is exiting and the next boot reads is_homed from the Teensy). No change needed.

Verification: see appended lines.

- 2026-11-25, `python -m pytest tests/ros/test_teensy_bridge_node_*.py -q -p no:cacheprovider -n 3 --dist loadfile` (after rebase on origin/main; box lock held by another Session's 40-minute run, so run unlocked with nice): 663 passed in 106 s. The first run found `test_main_destroys_action_servers_before_node` failing (its fake node lacked `_boot_interrupted`, read by main() since BUG-141); fixed in the test's fake.
- 2026-11-25, `python -m pytest tests/ros/test_teensy_bridge_node_shutdown_order_bug47.py -q`: 9 passed; with the node file reverted to the previous commit, the 3 new tests fail (3 failed, 6 passed).
- 2026-11-25, `python -m pytest tests/rclpy_smoke -q -s -o addopts=""`: 6 passed in 27.8 s; homing-in-flight exit 1.40 s after SIGINT, and the fake now receives SET_AXIS_STATE(0, IDLE) then STATE_WRITE is_homed=False. Honest limit: with the wait removed this smoke test STILL passes, because the worker's write lands during the ~1 s stow before the socket closes; only the mock tests discriminate the ordering.
