---
id: BUG-129
kind: bug
title: teensy_bridge_node freezes for about 340 ms once in some launches; Armed, that latches SETPOINT_STALE
status: in_progress
created: '2026-11-22'
updated: '2026-11-23T04:19:59Z'
owners: []
priority: high
effort: m
hardware: none
owner: none
radius: subsystem
diagnosis: hypothesis
area: can-bridge
blocked_by: []
links:
- map/idea-platform-accelerometer.md
filed_by: claude
night_skip: waiting-trigger
night_skip_on: '2026-11-24'
night_skip_note: Waits on launches with shm_clean (landed 2026-11-23) and one with bridge_shm:=false; the stall watchdog dump decides it.
---

## What happened

Sitting 2026-11-22 (IDEA-20 sheet, step 7), 21:05:34: Armed and holding still in ACTIVE:TRAJECTORY, the can-bridge guard latched SETPOINT_STALE (250 ms limit) the moment the operator called /platform_imu/upload. Cause found in the bag: every teensy_bridge_node topic stopped for 339 to 398 ms (21:05:33.565 to 33.907; /robot_state gap 342.7 ms) while Mocap and trajectory_node ran on time. The upload request was handled about 57 ms after the freeze ended, so the upload did not cause it.

## Ruled out

Ruled out: the upload handler (non-blocking, 0 READs sent), a garbage-collection pause (0.45 ms in that window), the code new on 2026-11-21/22 (the same freeze predates it), the mode (only the arm state matters, STANDBY is exposed too).

## History

History: bags since 2026-11-18 hold four one-off bridge freezes of 328 to 349 ms, at most one per launch. In the two with a known trigger the freeze ended just before the first ros2 command-line call to the bridge (today, and 2026-11-20 12:50:15 with the bridge disarmed, so no latch).

## Trigger

Trigger hypothesis NOT reproduced: a no-power launch on 2026-11-22 21:48 with three spaced `ros2 service call /platform_imu/set_upload_after_attempt` calls (first 18 s after the firmware checks) shows no /robot_state gap over 42.5 ms (bag 2026-11-22_21-48-38). So the first-call trigger is unconfirmed; what differs (launch age, Armed streaming, drive power, which service) is unknown.

## Risk and next step

Risk: a freeze over 250 ms while Armed latches the guard; mid-Attempt it becomes a scheduler STOP and then a latch. The response on 2026-11-22 was as designed (no Leg moved more than 0.0002 rev, clean recovery).

Do not change the 250 ms limit or the 70/80 ms lane covers for this. Next step: find what blocks the whole process for a third of a second (per-thread CPU and a stack sample of teensy_bridge_node around a freeze; BUG-112 wants the same instrument). Write-up: temp/debrief/2026-11-22/stale.md; logbook 2026-11-22-sitting-debrief.

## Progress

- 2026-11-23: 2026-11-23: stall watchdog landed and caught a real 359 ms stall at rest (14:07:33). The dump shows one thread inside rclpy publish (robot_state) and every other thread idle: a publish blocked in the middleware holding the GIL. Kernel pause ruled out for this event. Trigger still unknown; six new-participant actions did not reproduce it.
- 2026-11-23: Sitting 2026-11-23: one 313 ms stall at launch plus 1 s (unarmed), none while Armed. Third unprovoked dump with the same stack: the robot_state publish inside rclpy, all other threads idle.
- 2026-11-23: Experiment landed (logbook 2026-11-23-bridge-shm-clean-at-launch): the launch now removes stale Fast DDS shared-memory files before any node starts (shm_clean, on by default); bridge_shm:=false puts the bridge on UDP only (off by default, the decisive half, not yet tried). The watchdog keeps counting.
