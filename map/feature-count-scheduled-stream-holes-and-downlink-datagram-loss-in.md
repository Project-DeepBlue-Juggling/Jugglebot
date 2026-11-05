---
id: JB-174
kind: feature
title: Count scheduled-stream holes and downlink datagram loss in the can-bridge heartbeat
status: inbox
created: '2026-11-05'
updated: '2026-11-05T12:31:16Z'
owners: []
priority: high
effort: m
hardware: robot
owner: none
radius: normative
area: firmware
blocked_by: []
links:
- map/bug-firmware-scheduler-stop-fires-on-a-clean-host-stream-and.md
- map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
filed_by: claude
---

On 2026-11-05 three Hand-only STOPs ended Attempts with every host-side counter clean, so a Hand frame lost in the downlink is invisible today (JB-166, JB-149).

## Evidence
- Drop counters 0, frame ages at most 28 ms, every numbered frame sent (seq vs setpoints_sent constant), no host stamp hole.

## Wanted
- Uplink `s_sched_frames` and the udp_link downlink `seq_gaps` in the can-bridge heartbeat.
- A count of frames received without HAS_HAND / HAS_SCHED.
- A promotion stamp-hole counter per group.
- A per-frame host ring (seq, flags, stamp).
- PROTOCOL version bump; flash with the launch down.
