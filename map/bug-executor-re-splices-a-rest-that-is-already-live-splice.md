---
id: JB-122
kind: bug
title: Executor re-splices a REST that is already live; splice budget fixed at 158 ms while REST solve grows with window
status: complete
created: '2026-10-31'
updated: '2026-10-31T08:18:58Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-10-31-hold-ladder-sitting-the-hold-model-was-wrong.md
parent: two-ball-oval
filed_by: claude-books-2026-10-31
---

Software defect found on 2026-10-31 (goal 13): the first skill of a schedule, a long opening REST, is installed once and then spliced a second time after the Ball Butler announcement swap, and the second solve outruns a fixed splice budget.

## Evidence

- The opening REST homed the Hand from 8.62 to 0.307 rev over 6.5 s after a `LANE_REFUSED` end.
- First install: solve 469.6 ms, rebased by the fresh-origin rule; that worked.
- After the `ThrowAnnouncement`, the executor swap re-dispatched the same REST as a splice at k_s 43: solve 323 ms against a 158 ms budget (`SOLVE_BUDGET_KNOTS` 6 plus `LEAD_KNOTS` 9), wire already at knot 49.
- Result: `END SPLICE_TOO_LATE at skill 0 (REST)`, 0 of 0 caught, Ball Butler cancelled.
- A normal 1.75 s REST solves in 20-83 ms; solve cost grows about 8x for a 3.7x longer window while the budget stays constant.
- Not the closed handoff-solve-chaining class (a CATCH handoff splice).

## Fix candidates

1. Skip re-dispatch of a REST that is already live and identical after the executor swap.
2. Scale the splice budget with the REST window length.
3. Do the Ball Butler announcement swap before the first REST solve.
4. Home the Hand as its own move, then a short REST. The Hand return itself is addressed separately by the opening-REST change of 2026-10-31.

## Progress

- 2026-10-31: Fix candidate 2 (REST splice lead scaled with the window) implemented; candidate 1 (skip re-splicing a live REST) rejected: the swap installs a different terminal.
