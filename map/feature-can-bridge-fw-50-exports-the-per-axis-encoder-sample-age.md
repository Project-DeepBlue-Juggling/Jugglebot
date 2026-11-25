---
id: FEAT-64
kind: feature
title: Can-bridge FW 50 exports the per-axis encoder sample age
status: inbox
created: '2026-11-23'
updated: '2026-11-23T15:48:46Z'
owners: []
priority: low
effort: m
hardware: robot
owner: none
radius: subsystem
area: can-bridge
blocked_by: []
links:
- logbook/2026-11-23-bridge-stamp-on-robot-state-and-hand-telemetry.md
filed_by: claude
night_skip: frozen-path
night_skip_on: '2026-11-24'
night_skip_note: Can-bridge firmware (FW 50) plus a decoder and a flash; a day Session builds it.
---

Today `/robot_state` samples carry an encoder sample that is 0 to 10 ms older than the bridge telemetry tick, and the age is not exported (about 5.5 ms assumed from code, probably nearer 4 ms).

## Wanted

- Can-bridge FW 50 exports the per-axis encoder sample age in an additive message paired to its tick by `t_teensy_us`, the way `BbYawEstimate` is. No PROTOCOL bump.

## Why

- It removes the last 0 to 10 ms per axis from every offline comparison against `/robot_state`.
- It gives the true Leg tracking lag behind the command, which in turn settles how much of the 13 ms Mocap envelope offset is QTM latency (BUG-134).

## Progress

- 2026-11-25: built as ENCODER_AGE 0x96 and flashed inside FW 52 (20:54; decoded by the host, additive, PROTOCOL 14). Open until the uplink is read on a powered robot.
