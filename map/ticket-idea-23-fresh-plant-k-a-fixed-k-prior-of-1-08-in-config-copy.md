---
id: TKT-72
kind: ticket
title: 'IDEA-23 fresh-plant K: a fixed K prior of 1.08 in config, copy K from the trained plant, warm-start from the Memory mean, or leave the identity prior?'
status: open
created: '2026-11-14'
updated: '2026-11-13T17:51:08Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: learner
blocked_by: []
links: []
parent: idea-seed-a-fresh-plant-k-from-the-trained-plant
type: question
---

A fresh plant starts with apex gain K = 1.0, but 9 fresh starts on 2026-11-13 measured 1.077 +/- 0.015, so the first Throw flies about +68 mm high. `plant_model.py`'s own header measured a global K of 1.081 over 39 Sittings.

## Options

1. **A fixed K prior mean of 1.08 in config:** auditable, independent of any one plant.
2. **Copy K from the trained plant** at fresh-plant creation.
3. **Warm-start from the Memory CSV mean.**
4. **Leave the identity prior:** the learner fits K within two rows.

## What each answer unblocks

- 1, 2 or 3: the next night builds it on a branch; a dress rehearsal on a fresh plant judges it at a Sitting. It changes plan T-R2's empty-Memory-is-identity rule.
- 4: IDEA-23 closes.

Recommendation: 1. It matches both the corpus and the fresh starts, cannot inherit a bad plant, and takes the first Throw from about +68 mm to about +10 mm.
