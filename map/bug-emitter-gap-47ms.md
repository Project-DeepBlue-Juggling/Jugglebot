---
id: BUG-4
kind: bug
title: 'Emitter gap: one 47 ms window over sitting 8 against the 35 ms max target'
status: complete
created: '2026-10-24'
updated: '2026-11-02T08:49:47Z'
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
- logbook/2026-11-02-sitting-debrief.md
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

## Sitting result (2026-11-02)

- verdict: pass
- notes: Should be addressable using data from BUG-42/BUG-52 Sitting. I watched this topic during a few attempts and max_emit_gap_ms never appeared to be above ~35 ms, but the frequency of that echo-ing was quite slow

## Progress

- 2026-11-01: 2026-11-01 Sitting: 28/29 goal emit gap p50 26.2 / p99 30.9 / max 32.8 ms (pass on the goal). Launch-wide max 47.8 ms: one 48 ms emitter wake gap WARN at L1264 in the 16/17 goal.
- 2026-11-02: Superseded 2026-11-02 by BUG-58: the 50-throw goal read 33.0 ms (pass), but two long goals hit 81.7 and 40.9 ms and the hop 63.2 ms; the 81.7 ms gap ended the 58/58 goal by LANE_REFUSED. Owner: fold into the starvation Item.
