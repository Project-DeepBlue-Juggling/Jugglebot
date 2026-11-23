---
id: IDEA-28
kind: idea
title: Measure QTM's true latency with a physical edge seen by QTM and the bridge
status: inbox
created: '2026-11-23'
updated: '2026-11-23T10:30:18Z'
owners: []
priority: low
blocked_by: []
links:
- logbook/2026-11-23-bridge-stamp-on-robot-state-and-hand-telemetry.md
filed_by: claude
---

QTM's exposure-to-packet latency has never been measured. Offline fits against the executed command bound it to about 4 to 13 ms, and a Ball Butler yaw cross-check disagrees with the command route.

## Wanted

- Put one physical edge (an LED or strobe) in view of both QTM and a bridge-stamped input, and read the delay between the two stamps.

## Then

- Decide whether to correct the live Mocap stamp. That moves Tracker landing times, trips the bb_calibration 13 ms lag advisory and invalidates the Ball Butler release constants, so it needs a Ball Butler re-centring Sitting (BUG-134).
