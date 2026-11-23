---
id: BUG-134
kind: bug
title: 'Reference stamps run late: robot state is stamped at publish and Mocap omits QTM latency'
status: complete
created: '2026-11-23'
updated: '2026-11-23T11:04:34Z'
owners: []
priority: medium
effort: m
hardware: none
owner: none
radius: cross-cutting
diagnosis: confirmed
area: tracking
blocked_by: []
links:
- logbook/2026-11-23-platform-imu-clock-early-rx-ring-leak.md
- logbook/2026-11-23-bridge-stamp-on-robot-state-and-hand-telemetry.md
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

## Done and open (2026-11-23)

- Done: `/robot_state` and `/hand_telemetry` gain appended `bridge_stamp` and `bridge_stamp_valid`; `timestamp` is unchanged. Offline helpers in `tools/probes/_bag_time.py` restamp old bags by pairing with `/leg_cmd_executed`.
- Mocap: the envelope stamp runs 13.05 ms (sd 1.39) after the executed command. That constant lives in analysis tools only; the live stamp is not changed.
- Open: the encoder sample age needs can-bridge FW 50 (FEAT-64). QTM's true latency is unmeasured and lies between about 4 and 13 ms (IDEA-28).

## Progress

- 2026-11-23: Offline helpers and appended bridge_stamp fields landed in the worktree; live Mocap stamp left alone (detail below)
