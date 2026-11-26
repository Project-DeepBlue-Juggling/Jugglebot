---
id: TKT-100
kind: ticket
title: 'FEAT-65 held again: the x and y shrinkage law misses your gate on the full Memory; build a recency-weighted variant, merge as the quiet-command law, or drop it?'
status: open
created: '2026-11-27'
updated: '2026-11-26T17:23:46Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: learner
blocked_by: []
links: []
parent: feature-learner-a-noise-aware-shrinkage-estimator-in-place-of-the
type: question
---

FEAT-65 is rebuilt on night-shift/2026-11-26-FEAT-65 as you asked in TKT-97 (option 1): shrinkage on x and y only, the recent-16 mean kept on Apex. Last night's four audit findings are fixed. On the Memory copy as it now stands (5,455 rows), the law misses your gate on x and y.

| Law | rms x / y / Apex (mm) | jitter sd x / y / Apex (mm) |
|---|---|---|
| Model only | 21.58 / 20.44 / 16.44 | 1.00 / 0.96 / 1.01 |
| Production | 20.50 / 20.00 / 15.90 | 2.38 / 2.33 / 1.89 |
| x/y shrinkage, Apex recent-16 | 21.74 / 21.59 / 15.90 | 1.13 / 1.04 / 1.89 |

Command jitter is 53 and 56 % lower on x and y, but rms is 1.24 and 1.59 mm worse than production, and y is worse than the model alone. The all-rows mean appears to carry stale rows that the recent-16 mean tracks. The audit left one finding open: the replay test's "no worse than the old law" allowance was widened to 2.0 mm so the new law passes; it should go back to 1.0 mm as a strict xfail until you accept the law.

## Options

1. A windowed or recency-weighted shrinkage on x and y (the review's leaky law): the next night builds it on the branch, replays it, and holds it again.
2. Merge as the quiet-command law, accepting the rms loss (a day Session, after restoring the test as a recorded acceptance).
3. Drop it: the branch is left for you to delete and FEAT-65 goes back to open.

## What each answer unblocks

1: the next night builds and replays the variant. 2: a day Session merges. 3: the Item reopens without a branch.

Recommendation: 3, or 1 if you still want a quieter command: on the full Memory no all-rows law has beaten the recent-16 mean on rms.
