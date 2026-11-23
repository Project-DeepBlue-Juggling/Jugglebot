---
id: FEAT-64
kind: feature
title: Can-bridge FW 50 exports the per-axis encoder sample age
status: inbox
created: '2026-11-23'
updated: '2026-11-23T10:30:14Z'
owners: []
priority: low
blocked_by: []
links:
- logbook/2026-11-23-bridge-stamp-on-robot-state-and-hand-telemetry.md
filed_by: claude
---

Today `/robot_state` samples carry an encoder sample that is 0 to 10 ms older than the bridge telemetry tick, and the age is not exported (about 5.5 ms assumed from code, probably nearer 4 ms).

## Wanted

- Can-bridge FW 50 exports the per-axis encoder sample age in an additive message paired to its tick by `t_teensy_us`, the way `BbYawEstimate` is. No PROTOCOL bump.

## Why

- It removes the last 0 to 10 ms per axis from every offline comparison against `/robot_state`.
- It gives the true Leg tracking lag behind the command, which in turn settles how much of the 13 ms Mocap envelope offset is QTM latency (BUG-134).
