---
id: JB-27
kind: bug
title: Sim `--reload` gate LIMIT_JERK regression bisected to 08ec3fe6 (2026-09-30)
status: complete
created: '2026-10-24'
updated: '2026-11-01T02:54:09Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
diagnosis: confirmed
area: sim
blocked_by: []
links:
- plans/archived/columns-empty-cup-two-feed.md
filed_by: claude
---

The sim `--reload` gate was good at 8364808c (5/5 makes, dwell 0.30 s); after 08ec3fe6 it gets 1 make and `LIMIT_JERK` in the continuation throws. Candidate fix: set lateral authorities in the sim gate (plan section 6). Owner decision pending.

## Options (2026-11-01)

1. Treat it as a gate-configuration gap: set the sim gate's lateral authorities to the launch defaults (20 mm learner, 35 / 10 mm catch) and re-run the scenario; if it returns to 5/5 the Item closes with the gate fixed.
2. Treat it as a planner regression: bisect inside 08ec3fe6 (the feed-catch segment shape) with the sim scenario as the oracle and fix the planner; needs a design look, since hardware did not show it.
3. Both, in order: option 1 first (one evening, offline); if the scenario stays red, option 2.
4. Retire the `--reload` scenario from the sim gate as superseded by the Columns scenarios flown since.

Recommendation: 3. Option 1 is cheap and offline; its result decides whether 2 is needed.

A question ticket carries this to the owner; the answer under its `## Answer` re-opens this Item.

## Progress

- 2026-11-01: JB-128 option 3, step 1: sim `--reload` gate now runs the launch's lateral authorities; 2 makes per seed; LIMIT_JERK ends no longer reproduce on main; no planner bisect needed
