---
id: IDEA-25
kind: idea
title: Minimise the Platform IMU upload time after an Attempt
status: backlog
created: '2026-11-19'
updated: '2026-11-20T12:38:26Z'
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
- logbook/2026-11-20-sitting-debrief-robot-2.md
parent: two-ball-oval
filed_by: claude
night_skip: waiting-trigger
night_skip_on: '2026-11-19'
night_skip_note: Feasibility written 2026-11-19; waits for the Platform IMU MVP (plan platform-imu.md P1 to P4) to work.
---

Keep this area open (owner, 2026-11-19): once the MVP works, explore shortening the post-Attempt IMU upload. Three levers: tighter sample packing or compression; a slower setpoint heartbeat while holding (the 500 Hz stream stays as is for now: it feeds the ODrive watchdog and avoids a rate switch on the Leg path); a higher measured rate from the P4 ramp.

## Feasibility (2026-11-19)

- What it would take: once P4's rate ramp has measured the clean frame rate, pick one lever. A sample is 6 x int16 = 12 bytes against 8 data bytes per classic CAN frame, so the payload is the bound: a 12-bit or delta pack fits about 1.5 samples a frame, cutting the 64000 data frames of a full window to about 43000. A slower setpoint heartbeat while holding frees bus time but touches the Leg path and the ODrive watchdog. A higher rate needs only the measured number.
- Effort: m (packing in the Platform Teensy firmware and the Jetson decoder, with native and decode tests). Hardware: robot (the bus rate is measurable only on the Platform). The owner chooses the lever.
- One risk: packing changes the wire format of a CRC-checked upload, so firmware and decoder out of step corrupt captures silently unless the header carries a format version.
- Waits on: the MVP (plan `platform-imu.md` P1 to P4) working, as the owner set when filing.

## Progress

- 2026-11-20: Sitting 2026-11-20: Armed uploads took 79 s (ring) and 84 s (32 s span) against 56 s at rest, with 14651-19974 duplicate frames (158 at rest), up to 6 block timeouts, CAN1 at 74-77 %. A juggle goal was refused 7 s into a 37 s upload. Diagnosis in logbook 2026-11-20-sitting-debrief-robot-2.
