---
id: BUG-78
kind: bug
title: 'Two-ball Columns runs end on a missed Catch of Ball 2 with no collision: 13 of 23 ends, mean run 22 Throws'
status: in_progress
created: '2026-11-06'
updated: '2026-11-08T00:13:24Z'
owners: []
priority: high
effort: m
hardware: robot
owner: none
radius: subsystem
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief-2.md
- bug-mid-air-ball-collisions-end-half-of-two-ball-columns
- logbook/2026-11-06-sitting-debrief-3.md
filed_by: claude
---

Two-ball Columns runs of 2026-11-06 mostly end on a smooth missed Catch of Ball 2, not on a collision (BUG-74).

## Findings

- 35 endurance goals (Columns, Apex 0.95 to 1.10, Sep. 125 to 133, 101 Throws asked): median 15 Throws, best 73, about 4.5 ends per 100 Throws.
- End classes: 13 smooth missed Catches (2.0 per 100 Throws), 8 collisions (BUG-74), 2 ambiguous, 5 drive faults, 2 lane refusals after a GC pause (BUG-58), 5 refused before the first Throw.
- 21 of the 23 MISSED_CATCH first ENDs are the Catch of Ball 2 (the negative-x Site), although both balls' Throws land equally well (lateral RMS 29.6 against 31.9 mm).
- In 9 of them the last fit was within 40 mm of the aim and the cup read EMPTY: bounce-out, a false EMPTY (the BUG-67 question mid-run) or a late fit, not resolved (needs `/hand_telemetry`).
- At the P2 Sitting three of five ends were the same class, each with the Catch aim clamped at the lateral authority (10 mm outward three times, 35 mm inward twice).

## Open questions

- Not diagnosed.
- Owner decision if the diagnosis points there: the outward authority (10 mm) is a limit, not changed.

## Evidence

`temp/probes/sitting_debrief_2026-11-06b/unit_endurance.md` sections 1 and 2, `unit_stack.md` section 3.

## Progress

- 2026-11-06: 2026-11-06 20:20 Sitting: goals 2 and 4 are smooth Ball 2 misses with the ball deflected at Cup height: sideways speed jumps to 1.8 and 0.3 m/s within 30 ms, cup raw 0 throughout, so a false EMPTY and a late fit are out for these two. Ball 45 and 37 mm off the Platform origin at z 1000 (caught median 19, p90 36), Platform at 406 and 784 mm/s. Rim, Cone or edge: mocap cannot say.
- 2026-11-06: Across the endurance set and 20:20: every smooth no-collision end (15 of 15) is a Ball 2 Catch, and all 4 Ball 1 ends are collisions. Open: whether offset or Platform arrival error drives the strike; /trajectory/commanded_position was not read. Evidence: temp/probes/sitting_debrief_2026-11-06c/unit_ends.md.
- 2026-11-08: 2026-11-07: of 20 non-collision Ball 2 ends, 13 had a last fit beyond the Catch authority and 4 inside reach with the Cup empty. Owner: Cup rim 85 mm dia; in turned 90 deg Columns every strike hit the Platform top (top of the Hand's stroke); in other Patterns some misses hit the Hand, some other Platform parts. No seated-ball baseline in the bag.
