---
id: JB-125
kind: ticket
title: 'NOT_SETTLED retry: delete the dead branch, retry after the announcement swap, fix Ball Butler''s yaw-settle gate, or leave it fail-closed?'
status: open
created: '2026-11-01'
updated: '2026-10-31T19:09:40Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: ball-butler
blocked_by: []
links: []
parent: bug-not-settled-retry-dead-in-the-field
type: question
---

JB-25: the once-per-attempt `THROW_ABORTED_NOT_SETTLED` retry in `skill_node._on_bb_throw_outcome` never fires in the field, because the announcement swap retires the `reload` ctx before Ball Butler's terminal verdict arrives; the verdict then fails closed as `REJECTED_BB(THROW_ABORTED_NOT_SETTLED)` (bb-self-cancel finding F1, 2026-10-13). A real retry is design work, and the Item is low priority with no verdict race observed since.

Night Shift 2026-11-01 drafted the options below into the Item's `## Options (2026-11-01)` section and needs the owner to pick the direction before any code.

## Options

1. Retire the retry: delete the dead `retry_ctx` branch and its tests, keep the fail-closed end. Smallest honest state; a yaw-settle false positive costs one attempt (4 of 19 on 2026-09-30).
2. Retry after the swap: let the retry re-fire against the LIVE executor's schedule (recompute `delay_s` from the installed bridge, re-record the release window) once per attempt. Design work on the swap path; needs a robot Sitting to see a real `NOT_SETTLED`.
3. Fix at the source: tighten Ball Butler's yaw-settle gate (BallButler firmware) so the false positives stop, and keep the host fail-closed. Lives in the BallButler repo.
4. Leave as is (fails closed, documented); close the Item as "won't fix until a verdict race is observed", which is its own trigger.

Recommendation: 1 now (dead code that reads as a live guarantee is the hazard) and 3 when the Ball Butler firmware is next touched; 2 only if the false-positive rate returns.

## What each answer unblocks

- 1: a Night Shift unit (hardware none) deleting the branch and its tests, re-rating the Item.
- 2: a design unit for the owner's Session and a robot Sitting section on this Item.
- 3: an Item in the BallButler repo; this one closes as superseded.
- 4: the Item is re-rated and parked.
