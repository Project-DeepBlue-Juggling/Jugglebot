---
id: JB-23
kind: bug
title: 'Emitter gap: one 47 ms window over sitting 8 against the 35 ms max target'
status: inbox
created: '2026-10-24'
updated: '2026-11-01T12:18:36Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: hands
radius: subsystem
diagnosis: unknown
area: skill-stack
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
filed_by: claude
---

Over sitting 8 the emitter showed p50 25.9 / p99 29.9 / max 47.0 ms on `/trajectory/status`: the p99 < 30 ms target is met, the max < 35 ms target is not. The solve is no longer in the emitter's interpreter; find what still holds it for 20 ms (GC, logging, the `/trajectory/status` publish). Trigger: a latched lane or a missed knot traced to a gap.

## Sitting

- setup: armed
- why: the 47 ms emitter window has only one sample (sitting 8); a fresh read at today's launch says whether the max is still over the 35 ms target.
- do:
  - Fly one 50-throw goal of any Pattern and read the emitter p50 / p99 / max from `/trajectory/status` in the launch log.
- pass: emitter max below 35 ms over the goal.
- record: the emitter p50, p99 and max lines, and the goal flown.
- minutes: 10

## Progress

- 2026-11-01: 2026-11-01 Sitting: 28/29 goal emit gap p50 26.2 / p99 30.9 / max 32.8 ms (pass on the goal). Launch-wide max 47.8 ms: one 48 ms emitter wake gap WARN at L1264 in the 16/17 goal.
