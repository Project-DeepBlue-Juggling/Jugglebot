---
id: JB-25
kind: bug
title: NOT_SETTLED retry is dead in the field
status: inbox
created: '2026-10-24'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: low
effort: m
hardware: robot
owner: decision
radius: subsystem
diagnosis: confirmed
area: ball-butler
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
parent: ball-butler
filed_by: claude
---

The `NOT_SETTLED` retry never fires in the field; it fails closed as `REJECTED_BB(THROW_ABORTED_NOT_SETTLED)`. A real retry is design work (bb-self-cancel finding F1). Trigger: a Ball Butler verdict race observed.

## Options (2026-11-01)

1. Retire the retry: delete the dead `retry_ctx` branch and its tests, keep the fail-closed end. Smallest honest state; a yaw-settle false positive costs one attempt (4 of 19 on 2026-09-30).
2. Retry after the swap: let the retry re-fire against the LIVE executor's schedule (recompute `delay_s` from the installed bridge, re-record the release window) once per attempt. Design work on the swap path; needs a robot Sitting to see a real `NOT_SETTLED`.
3. Fix at the source: tighten Ball Butler's yaw-settle gate (BallButler firmware) so the false positives stop, and keep the host fail-closed. Lives in the BallButler repo.
4. Leave as is (fails closed, documented); close the Item as "won't fix until a verdict race is observed", which is its own trigger.

Recommendation: 1 now (dead code that reads as a live guarantee is the hazard) and 3 when the Ball Butler firmware is next touched; 2 only if the false-positive rate returns.

A question ticket carries this to the owner; the answer under its `## Answer` re-opens this Item.
