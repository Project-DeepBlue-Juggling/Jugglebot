---
id: JB-189
kind: bug
title: A Throw from a faulted Hand stroke is written to the Memory as caught
status: complete
created: '2026-11-06'
updated: '2026-11-06T18:16:09Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: learner
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief-2.md
- logbook/2026-11-07-jb189-no-memory-row-from-a-faulted-stroke.md
parent: learner-generalisation
filed_by: claude
---

P2 Sitting 2026-11-06 goal 10: the Throw the lead clamp cut short (Apex 0.318 m against 1.00 commanded, released about 80 ms before the Hand drive faulted) is a row in `temp/learn/jugglebot/memory_v2.csv` with `caught=True`.

## Findings

- The plant model's inlier rule rejects it, so the fit is unaffected, but the row is on disk and any reader that trusts `caught` will use it.

## Wanted

- Candidate rule: a Throw whose stroke overlapped a guard latch, a lead-clamp episode or a drive fault writes no row (or a flagged one).

## Evidence

`temp/probes/sitting_debrief_2026-11-06b/unit_A_C.md` (goal 10) and `unit_hand.md` section 1.
