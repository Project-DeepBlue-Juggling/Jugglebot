---
id: BUG-134
kind: bug
title: 'Reference stamps run late: robot state is stamped at publish and Mocap omits QTM latency'
status: inbox
created: '2026-11-23'
updated: '2026-11-23T07:12:19Z'
owners: []
priority: medium
effort: m
radius: cross-cutting
diagnosis: confirmed
area: tracking
blocked_by: []
links:
- logbook/2026-11-23-platform-imu-clock-early-rx-ring-leak.md
filed_by: claude
---

Reference stamps run late, so neither Mocap nor the encoders can validate a bridge-clock stream (the Platform IMU) to better than about 15 ms. Found while diagnosing BUG-132 (logbook entry 2026-11-23-platform-imu-clock-early-rx-ring-leak).

## Findings

- `/robot_state` and `/hand_telemetry` are stamped at publish on the Jetson, 12 to 14 ms after the bridge telemetry tick (measured by restamping to the tick: IMU lag against encoders moved from about -19 ms to about -3 ms). The encoder sample is a further 0 to 10 ms older than the tick (about 5.5 ms assumed from code).
- The Mocap stamp is QTM frame time plus the minimum receive latency and omits QTM's own latency, about 14 to 16 ms inferred and never measured.

## Proposed fix

- Stamp `/robot_state` from the bridge tick time (`tm.t_teensy_us`) and export the encoder sample age from the bridge firmware.
- Measure QTM's exposure-to-packet latency and add it in `qtm_clock_sync.py`.
- Survey the consumers of these stamps first.
