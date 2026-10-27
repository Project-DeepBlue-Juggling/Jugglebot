---
id: JB-75
kind: bug
title: can-bridge SETPOINT_STALE latches on an arm edge when the last setpoint is from an earlier stream
status: inbox
created: '2026-10-27'
updated: '2026-10-27T12:25:11Z'
owners: []
priority: medium
blocked_by: []
links:
- ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
- ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
filed_by: claude
---

The can-bridge guard can latch SETPOINT_STALE about 0.1 s after an arm edge, while the stream is holding still and before any motion. The cause is that `s_last_setpoint_us` (leg_interp.cpp:137) survives across arms: `interp_reset()` has no runtime caller. On a bridge that has streamed before (`ever_cmd` true), the 10 Hz check in fault_machine.cpp:440 (`ever_cmd && s_mpc_active && age > SETPOINT_STALENESS_US`) sees the previous stream's age from the moment `mpc_active` goes true. The bridge node sends the first new setpoint only after the arm: 37 ms after it on 2026-10-27 and 16 ms on 2026-10-24. If the fault tick lands inside that window, the guard latches.

Seen on 2026-10-27 during the lead-clamp P3 Sitting (bridge FW 33, not power-cycled since the P2 bench session, uptime 5523 s at launch):
- `setpoint output ARMED` at 1793102549.376, then `FAULT LATCHED: fault_state=SETPOINT_STALE` at .493.
- The `/leg_setpoint_echo` samples start at .412.
- Recovered with clear_errors, then re-armed at 563.2 without another latch.
- Launch log: /home/jetson/.ros/log/2026-10-27-22-59-39-920154-jetson-2913798/launch.log. Bag: /home/jetson/Desktop/rosbags/2026-10-27_22-59-40.

It fails safe: output is suppressed and the recovery is clean. The fault tick landing in the window is inferred from the timing, not observed directly. The mechanism predates FW 31 and has nothing to do with the lead clamp. Over the 9 earlier FW 30 arms with a 16 ms window, never seeing a latch is plausible by chance.

Possible fix: stamp `s_last_setpoint_us` (or start a grace window of one staleness bound) on the `mpc_active` false -> true edge. A dead stream still trips 250 ms after the arm.
