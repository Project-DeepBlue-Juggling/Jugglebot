---
id: IDEA-25
kind: idea
title: Minimise the Platform IMU upload time after an Attempt
status: inbox
created: '2026-11-19'
updated: '2026-11-18T13:11:51Z'
owners: []
priority: low
blocked_by: []
links:
- plans/active/platform-imu.md
parent: two-ball-oval
filed_by: claude
---

Keep this area open (owner, 2026-11-19): once the MVP works, explore shortening the post-Attempt IMU upload. Three levers: tighter sample packing or compression; a slower setpoint heartbeat while holding (the 500 Hz stream stays as is for now: it feeds the ODrive watchdog and avoids a rate switch on the Leg path); a higher measured rate from the P4 ramp.
