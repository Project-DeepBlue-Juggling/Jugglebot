---
id: JB-191
kind: bug
title: 'Two-ball Columns runs end on a missed Catch of Ball 2 with no collision: 13 of 23 ends, mean run 22 Throws'
status: inbox
created: '2026-11-06'
updated: '2026-11-06T05:59:07Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
diagnosis: unknown
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief-2.md
- bug-mid-air-ball-collisions-end-half-of-two-ball-columns
filed_by: claude
---

Two-ball Columns runs of 2026-11-06 mostly end on a smooth missed Catch of Ball 2, not on a collision (JB-186).

## Findings

- 35 endurance goals (Columns, Apex 0.95 to 1.10, Sep. 125 to 133, 101 Throws asked): median 15 Throws, best 73, about 4.5 ends per 100 Throws.
- End classes: 13 smooth missed Catches (2.0 per 100 Throws), 8 collisions (JB-186), 2 ambiguous, 5 drive faults, 2 lane refusals after a GC pause (JB-149), 5 refused before the first Throw.
- 21 of the 23 MISSED_CATCH first ENDs are the Catch of Ball 2 (the negative-x Site), although both balls' Throws land equally well (lateral RMS 29.6 against 31.9 mm).
- In 9 of them the last fit was within 40 mm of the aim and the cup read EMPTY: bounce-out, a false EMPTY (the JB-169 question mid-run) or a late fit, not resolved (needs `/hand_telemetry`).
- At the P2 Sitting three of five ends were the same class, each with the Catch aim clamped at the lateral authority (10 mm outward three times, 35 mm inward twice).

## Open questions

- Not diagnosed.
- Owner decision if the diagnosis points there: the outward authority (10 mm) is a limit, not changed.

## Evidence

`temp/probes/sitting_debrief_2026-11-06b/unit_endurance.md` sections 1 and 2, `unit_stack.md` section 3.
