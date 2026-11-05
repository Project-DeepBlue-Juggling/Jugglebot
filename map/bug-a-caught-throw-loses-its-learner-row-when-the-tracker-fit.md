---
id: JB-176
kind: bug
title: A row holding its own fit logs a false 'no evidence or row' refusal when the Tracker moves to the ball's next flight
status: complete
created: '2026-11-05'
updated: '2026-11-05T19:57:59Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-06-jb176-false-refusal-line.md
filed_by: claude
---

In the first fed Attempt of the 2026-11-05 22:46 Sitting, Throws 3, 4, 5, 6 and 8 were CAUGHT but gave no learner row (TRACKER-IDENTITY-REFUSED): the fitted landing was +1.29 to +1.31 s from the scheduled one, band +/-0.434 s.

## Findings
- +1.30 s is exactly one beat (flight 0.903 s + Dwell 0.40 s): the Tracker fit is the same ball's next flight.
- `executor.py` `_release_landing` gates on landing time only.
- The learner lost 5 of 9 rows in that Attempt (silent warm-up loss).

## Hypothesis (confirm in the bag first)
- Outcome finalisation runs late (cup seat evidence about 0.4 s after landing), after the ball is re-announced; match the fit by its own release time to `pend.t_release_s` and skip the next-flight fit.

## Findings (2026-11-06)

Read from the 2026-11-05 22:46 bag and launch log: the hypothesis is half right, and the "5 of 9 rows lost" is wrong.

- Right: each refused fit is the same ball's next flight. Finalise falls 0.115-0.131 s after the next release.
- Wrong: Throws 4, 5, 6 and 8 did get rows (`memory row appended` is logged 2-29 ms before each refusal line). Only Throw 3 lost its row.
- So the line "no evidence or row from it" is false on those four: the row already held its own fit.
- Throw 3's row loss has a different cause, tracked as JB-181 (a hypothesis there).
- The fix here is only the misleading line and the inflated `n_rejected`. The identity gate is correct and stays as it is.
