---
id: JB-176
kind: bug
title: A caught Throw loses its learner row when the Tracker fit has moved to the ball's next flight (+1 beat)
status: inbox
created: '2026-11-05'
updated: '2026-11-05T12:31:17Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links: []
filed_by: claude
---

In the first fed Attempt of the 2026-11-05 22:46 Sitting, Throws 3, 4, 5, 6 and 8 were CAUGHT but gave no learner row (TRACKER-IDENTITY-REFUSED): the fitted landing was +1.29 to +1.31 s from the scheduled one, band +/-0.434 s.

## Findings
- +1.30 s is exactly one beat (flight 0.903 s + Dwell 0.40 s): the Tracker fit is the same ball's next flight.
- `executor.py` `_release_landing` gates on landing time only.
- The learner lost 5 of 9 rows in that Attempt (silent warm-up loss).

## Hypothesis (confirm in the bag first)
- Outcome finalisation runs late (cup seat evidence about 0.4 s after landing), after the ball is re-announced; match the fit by its own release time to `pend.t_release_s` and skip the next-flight fit.
