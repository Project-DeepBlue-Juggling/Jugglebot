---
id: BUG-146
kind: bug
title: Aimed Throw from a level rest is refused HAND_LIMIT_C2 past about 6.3 deg of release tilt
status: inbox
created: '2026-11-26'
updated: '2026-11-26T01:17:45Z'
owners: []
priority: medium
effort: m
hardware: none
owner: none
radius: subsystem
diagnosis: unknown
area: skill-stack
blocked_by: []
links: []
parent: human-partner-juggling
filed_by: claude
---

An aimed THROW from a level rest is refused `HAND_LIMIT_C2` once the release tilt passes about 6.3 to 6.9 deg. Found 2026-11-26 by the PWAY-11 P4 design sweep (`executor.check_segment`, the context of `tools/probes/human_catch_reach.py`).

## Findings

- The refusal reads "peak hand knot-acceleration discontinuity 5.02 rev/s^2 > 4.50". It appears in all 8 directions, at Apex 1.5, 1.8 and 2.1 m, and for windows of 0.4, 0.6 and 0.8 s.
- It caps aimed reach at 250, 400 and 550 mm for those Apex heights. Leg jerk margin at the edge is still 0.04 to 0.41, so the Legs do not bind.
- The tolerance's own comment in `controller/feasibility.py` says a plan that needs a looser value has a real C2 defect.
- Root cause not investigated. Raw rows: temp/probes/pway11-p4/ (sweepA.jsonl, reach_table.md).

## Wanted

- Find where the Hand acceleration jump comes from as tilt grows (the tilt-coupled Hand compensation at a knot is the first suspect, unchecked), and fix the plan, not the tolerance.
