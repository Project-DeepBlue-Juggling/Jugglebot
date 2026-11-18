---
id: IDEA-25
kind: idea
title: Minimise the Platform IMU upload time after an Attempt
status: inbox
created: '2026-11-19'
updated: '2026-11-18T15:51:27Z'
owners: []
priority: low
effort: m
hardware: robot
owner: decision
radius: local
area: platform-teensy
blocked_by: []
links:
- plans/active/platform-imu.md
parent: two-ball-oval
filed_by: claude
---

Keep this area open (owner, 2026-11-19): once the MVP works, explore shortening the post-Attempt IMU upload. Three levers: tighter sample packing or compression; a slower setpoint heartbeat while holding (the 500 Hz stream stays as is for now: it feeds the ODrive watchdog and avoids a rate switch on the Leg path); a higher measured rate from the P4 ramp.

## Feasibility (2026-11-19)

- What it would take: once P4's rate ramp has measured the clean frame rate, pick one lever. A sample is 6 x int16 = 12 bytes against 8 data bytes per classic CAN frame, so the payload is the bound: a 12-bit or delta pack fits about 1.5 samples a frame, cutting the 64000 data frames of a full window to about 43000. A slower setpoint heartbeat while holding frees bus time but touches the Leg path and the ODrive watchdog. A higher rate needs only the measured number.
- Effort: m (packing in the Platform Teensy firmware and the Jetson decoder, with native and decode tests). Hardware: robot (the bus rate is measurable only on the Platform). The owner chooses the lever.
- One risk: packing changes the wire format of a CRC-checked upload, so firmware and decoder out of step corrupt captures silently unless the header carries a format version.
- Waits on: the MVP (plan `platform-imu.md` P1 to P4) working, as the owner set when filing.
