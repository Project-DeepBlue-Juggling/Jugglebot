---
id: FEAT-65
kind: feature
title: 'Learner: a noise-aware shrinkage estimator in place of the mean of the 16 most recent rows'
status: in_progress
created: '2026-11-24'
updated: '2026-11-25T17:40:29Z'
owners:
- night-shift
priority: medium
effort: m
hardware: none
owner: decision
radius: subsystem
area: learner
blocked_by: []
links: []
parent: learner-generalisation
filed_by: claude
---

Replace the Learner's mean of the 16 most recent support rows with a noise-aware shrinkage estimator over all in-kernel rows. The goal is a quieter command at equal landing error, not smaller scatter: the predictability ceiling is 2 to 4 % of rms on x. Plan step E2(a) in `learner-efficacy.md`; evidence in `logbook/2026-11-24-learner-efficacy-review-and-scatter-decomposition.md`.

## Why

- Defect A: the support is the most recent k, not the nearest k (`learner.py:270`). 49 % of queries get less than half the evidence nearest-k would give; 25 % get a kernel weight sum below 4 although 12 or more near-exact rows exist.
- The proportional law with alpha at most 0.889 leaves 11 % of any residual uncorrected.
- A shuffled-error control shows noise alone reproduces about 70 % of the production correction's median magnitude.

## Replay (rms x / y / Apex mm; command jitter x / y / Apex mm)

| Law | rms | jitter |
|---|---|---|
| Model only | 21.71 / 20.26 / 18.54 | 0 |
| Production | 20.97 / 20.28 / 18.14 | 8.5 / 5.7 / 4.7 |
| Nearest-16 | 21.12 / 20.44 / 18.46 | 9.7 / 7.1 / 4.3 |
| Shrinkage | 20.91 / 20.07 / 18.48 | 6.2 / 5.5 / 0.8 |
| Shrinkage plus leaky integrator | 20.76 / 19.95 / 18.10 | 7.6 / 5.6 / 2.7 |

The rms differences between laws are 0.1 to 0.3 mm with no confidence interval, so they are ties except against model-only. This is a prediction-quality test, not a closed-loop simulation.

## Estimator

- Kernel mean over all past rows; gain n_eff / (n_eff + sigma_noise squared / sigma_prior squared), sigma_prior 10 mm.

## Gate (proposed, owner to confirm)

- Sequential replay rms no worse than production on every axis, and command jitter at least 30 % lower.
- Sim gate green; first flown as a normal Columns Attempt.
- Safety-bearing: it changes every commanded u. Walk one Throw cycle through the command path before changing it (the Learner runs at command time, not in the 40 Hz tick).

## Options (2026-11-25)

Night Shift 2026-11-25 drafted these; TKT-94 carries them and its answer re-opens this Item.

1. Confirm the proposed gate; the night builds it with the replay on its Memory copy and holds it for a day Session to merge.
2. As 1, but the night may merge on the gate's MERGE when its replay meets the criterion.
3. A different criterion, then as 1.
4. Build the leaky-integrator variant instead, then as 1.
5. Keep E2 for a day Session.

Recommendation: 1, because the replay differences between laws are 0.1 to 0.3 mm with no confidence interval.

## Progress

- 2026-11-26: Night Shift 2026-11-26: held on night-shift/2026-11-26-FEAT-65: built per TKT-94; replay misses the Apex gate (+0.67 mm rms), jitter 24 to 65 percent lower; unblocked by your answer in TKT-97
