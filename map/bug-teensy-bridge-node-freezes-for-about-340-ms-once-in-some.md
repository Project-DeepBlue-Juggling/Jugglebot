---
id: BUG-129
kind: bug
title: teensy_bridge_node freezes for about 340 ms once in some launches; Armed, that latches SETPOINT_STALE
status: inbox
created: '2026-11-22'
updated: '2026-11-22T13:24:43Z'
owners: []
priority: high
effort: m
hardware: none
owner: none
radius: subsystem
diagnosis: unknown
area: can-bridge
blocked_by: []
links:
- map/idea-platform-accelerometer.md
filed_by: claude
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
