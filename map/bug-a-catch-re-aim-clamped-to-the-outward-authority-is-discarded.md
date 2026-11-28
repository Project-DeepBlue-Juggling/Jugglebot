---
id: BUG-161
kind: bug
title: A Catch re-aim clamped to the outward authority is discarded by the re-send tolerance at 10 mm
status: complete
created: '2026-11-28'
updated: '2026-11-28T07:58:23Z'
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
- logbook/2026-11-28-first-flight-of-the-learner-merge-and-the-15-mm-catch-authority-trial.md
- logbook/2026-11-28-clamped-catch-re-aim-judged-on-the-fit-demand.md
parent: learner-generalisation
filed_by: claude
---

A Catch re-aim that the outward authority clamps to a length at or below `resend_pos_tol_mm` is discarded as within tolerance, so the Cup never moves toward the ball.

## Findings

- `resend_pos_tol_mm` defaults to 10.0 (`executor.py`, tests `moved_mm <= self.resend_pos_tol_mm`), the same as the default `catch_outward_authority_mm` of 10.0.
- A re-aim clamped to exactly 10.0 mm outward is logged RESEND-SKIPPED or BUMP-DECLINED WITHIN-TOLERANCE and the Cup stays put.
- Seen in Attempts 5, 6 and 9 of the 2026-11-28 12:21 bag and Attempts 3, 7 and 10 of the 2026-11-27 19:13 bag, every one a lost Feed.
- With the authority at 15 mm (2026-11-28 12:31 bag) the re-aim was taken.

## Status

A fix is in progress this Session (2026-11-28). Left open until it lands and its logbook entry is written.

## Progress

- 2026-11-28: Fixed 2026-11-28: both re-aim paths judge a clamp-shortened move on the Fit's demand (C-CATCH-2 amended). Not yet flown; first flight is BUG-160's Sitting.
