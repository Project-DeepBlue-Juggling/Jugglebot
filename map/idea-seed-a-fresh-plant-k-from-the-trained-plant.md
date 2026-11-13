---
id: IDEA-23
kind: idea
title: seed a fresh plant K from the trained plant
status: inbox
created: '2026-11-13'
updated: '2026-11-13T17:50:23Z'
owners: []
priority: low
effort: s
hardware: robot
owner: decision
radius: subsystem
area: learner
blocked_by: []
links:
- map/idea-learner-context-cannot-tell-a-throw-from-rest-from-a-carried.md
- map/feature-p4-ramp-sitting-columns-canary-at-600-8000-and-20-a-then-the.md
filed_by: claude
---

A fresh plant starts with K = 1.0 (identity prior), but 9 fresh starts over the Sitting of 2026-11-13 measured K 1.077 +/- 0.015. The first Throw on a fresh plant therefore flies 1.077 +/- 0.015 times the commanded Apex (+51 to +92 mm, mean about +68 mm, flight about 3.7 % longer), which is exactly what the learner then fits (K 1.063 after one row, 1.078 after two).

## Wanted

- Seed a fresh plant K from the trained plant, or warm-start from the Memory CSV mean (about 1.07).
- Why it matters: three of the four oval ends on the fresh plant carried a 12 to 58 ms late arrival from this error, and every fresh-plant row of the Sitting ended early Attempts on it.
- A control change on the Throw Apex: it needs an owner decision and a dress rehearsal on a fresh plant.

## Feasibility (2026-11-14)

- What it takes: `plant_model.py` reports a parameter's prior mean while it is uncertain, and K's prior mean is 1.0, so an empty Memory is exactly the identity (plan T-R2). Seeding means a different prior mean for K at plant creation; the RLS update is unchanged.
- The same module's header already measured a global K of 1.081 over 39 Sittings (1.070 to 1.095 per Sitting), consistent with 1.077 +/- 0.015 on 2026-11-13.
- Effort s; hardware robot (a dress rehearsal on a fresh plant); owner decision (a control change on the Throw Apex).
- Risk: a seed copied from a trained plant carries that plant's error into every fresh start, and a fresh plant exists partly to escape a bad Memory.

## Options (2026-11-14)

1. **A fixed K prior mean in config, 1.08:** auditable, independent of any one plant.
2. **Copy K from the trained plant** at fresh-plant creation.
3. **Warm-start from the Memory CSV mean** over its last rows.
4. **Leave the identity prior:** the learner fits K within two rows.

Recommendation: 1. It matches both the 39-Sitting corpus and the 9 fresh starts, cannot inherit a bad plant, and takes the first Throw from about +68 mm to about +10 mm. It changes the empty-Memory-is-identity rule, so plan T-R2's wording changes with it.
