---
id: TKT-97
kind: ticket
title: 'FEAT-65 held: the replay misses your Apex gate; shrinkage on x and y only, add a leaky term, merge as is, or drop?'
status: open
created: '2026-11-26'
updated: '2026-11-25T17:40:34Z'
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

FEAT-65 is built on night-shift/2026-11-26-FEAT-65 as you asked (TKT-94 option 1): the Learner's residual is now a kernel mean over all in-kernel rows with a per-axis gain n_eff / (n_eff + sigma_noise^2 / sigma_prior^2) (Kish n_eff, sigma_noise 18 mm, sigma_prior 10 mm). The night's sequential replay of the 2026-11-21 Memory copy (2,077 rows scored, a prediction-quality test, not closed loop) does NOT meet your gate.

| Law | rms x / y / Apex (mm) | command jitter sd x / y / Apex (mm) |
|---|---|---|
| Production | 21.32 / 20.27 / 18.28 | 2.21 / 2.11 / 2.65 |
| Shrinkage | 21.43 / 20.27 / 18.95 | 0.99 / 0.75 / 2.02 |

Jitter is 55, 65 and 24 % lower (x, y, Apex), but Apex rms is 0.67 mm worse than production and worse than the model alone (18.69); x is +0.11 mm, inside the review's tie level. No sigma from 12 to 80 mm rescues Apex: the all-rows mean averages away slow Apex structure that the most-recent rows track. The 2026-11-24 review's own numbers for this law (Apex 18.48 rms, 0.8 jitter) were not reproduced; its script lives only in your checkout.

The night's audit also left four behaviour-affecting findings open on the branch: the nightly-tier `tests/motion/test_learner_replay.py` pins the production law's numbers and was not updated (it would fail `--full`), and three new tests are weaker than they claim (a bound the cap also satisfies, a seed-dependent no-step check, a moved test Site). Any option below that keeps the branch includes fixing those.

## Options

1. Apply the shrinkage law to x and y only and keep the most-recent-rows law on Apex; the night builds that and replays it, then holds it as before.
2. Add a leaky (recency-weighted) term to the all-rows mean, the review's fifth law, and replay; then as 1.
3. Merge as is, as the quiet-command law, accepting the Apex loss (a day Session).
4. Drop it: E2(a) was already retired from the plan; keep FEAT-65 as an Item until the launch scatter is down.

## What each answer unblocks

1 or 2: the next night builds the variant on the held branch, replays it against your gate, and holds it for the day Session you asked for. 3: a day Session merges it after its own live-Memory replay. 4: the branch is left for you to delete and FEAT-65 goes back to open.

Recommendation: 1, because x and y gain the quiet command at no cost while Apex keeps the law that tracks its slow drift.
