---
id: BUG-133
kind: bug
title: Ball Butler firmware builds against the unpatched FlexCAN_T4 and stamps the sync at dispatch
status: inbox
created: '2026-11-23'
updated: '2026-11-23T07:12:10Z'
owners: []
priority: medium
effort: m
hardware: robot
radius: subsystem
diagnosis: confirmed
area: ball-butler
blocked_by: []
links:
- logbook/2026-11-23-platform-imu-clock-early-rx-ring-leak.md
filed_by: claude
night_skip: out-of-reach
night_skip_on: '2026-11-24'
night_skip_note: The fix is in the BallButler repo's firmware, not in this clone, and needs a flash; its deciding measurement needs Ball Butler uptime per Throw.
---

Ball Butler firmware (BallButler repo, `ball_butler_main`) builds against upstream FlexCAN_T4 master and stamps the 0x7DD sync at dispatch. The Platform had the same defect, diagnosed in BUG-132 (logbook entry 2026-11-23-platform-imu-clock-early-rx-ring-leak).

## Findings

- Same unprotected RX pop, so stranded frames turn the RX ring into a delay line: the wall clock reads early and the Throw command arrives late by the same delay, so Ball Butler would release late.
- Cost is about 2.6 ms per stranded frame (about 390 frames/s on its bus), growing with uptime. Inferred growth is 0.3 to 8 ms per hour; saturated it is about 0.65 s.
- On 2026-10-12 the release landed 3.9 ms before `release_wall` (sd 2.6), so the delay was about zero then. Ball Butler is power-cycled each Sitting, which resets it.

## Proposed fix

- Build against the can-bridge's patched copy (pop with the RX interrupt masked), and stamp 0x7DD at reception.

## Deciding measurement

- Release minus `release_wall` per Throw, against Ball Butler uptime. A slow ratchet toward late in 2.6 ms steps convicts it.
