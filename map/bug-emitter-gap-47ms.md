---
id: JB-23
kind: bug
title: 'Emitter gap: one 47 ms window over sitting 8 against the 35 ms max target'
status: inbox
created: '2026-10-24'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: medium
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
filed_by: claude
---

Over sitting 8 the emitter showed p50 25.9 / p99 29.9 / max 47.0 ms on `/trajectory/status`: the p99 < 30 ms target is met, the max < 35 ms target is not. The solve is no longer in the emitter's interpreter; find what still holds it for 20 ms (GC, logging, the `/trajectory/status` publish). Trigger: a latched lane or a missed knot traced to a gap.
