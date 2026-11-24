---
id: TKT-94
kind: ticket
title: 'FEAT-65: confirm the E2 gate for the shrinkage estimator, and may the night merge it or must a day Session?'
status: complete
created: '2026-11-25'
updated: '2026-11-24T22:13:01Z'
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

FEAT-65 replaces the Learner's mean of the 16 most recent rows with a noise-aware shrinkage estimator (plan E2(a)). It changes every commanded u, and its acceptance gate is still marked "proposed, owner to confirm" (plan § 7 question 1). The night has a Memory copy dated 2026-11-21 in its clone, so it can run the sequential replay offline.

## Options

1. Confirm the proposed gate: replay rms no worse than production on every axis, command jitter at least 30 % lower, sim gate green. The night builds it with continuity and empty-Memory tests and the replay, and holds it for a day Session to re-run the replay on the live Memory and merge.
2. The same, but the night may merge it when the gate script says MERGE and the replay on its Memory copy meets the criterion; the first Columns Attempt is the flight check.
3. A different criterion (name the numbers), then as 1.
4. Build the leaky-integrator variant instead (replay rms 20.76 / 19.95 / 18.10 mm, jitter 7.6 / 5.6 / 2.7 mm), then as 1.
5. Keep it for a day Session: E2 as one piece of work, with BUG-138 and BUG-139.

## Recommendation

1. The replay differences between laws are 0.1 to 0.3 mm with no confidence interval, so a day Session should look at the live-Memory replay before the robot flies a new law on every Throw.

## Answer

Agreed; go with Option 1.
