---
id: BUG-137
kind: bug
title: 'Learner far-Site guard fails on long single-Site memories: the slope turns confident with no excitation'
status: complete
created: '2026-11-24'
updated: '2026-11-24T16:22:22Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: learner
blocked_by: []
links:
- logbook/2026-11-25-plant-model-far-site-guard-unforgotten-information.md
parent: learner-generalisation
filed_by: claude
---

The Learner's far-Site guard (the rule that a Site far from the data gets no offset) fails on a long single-Site Memory. The plant model's slope turns confident with no excitation, and an invented slope is applied at a Site with no rows. Found in the PWAY-2 review, `logbook/2026-11-24-learner-efficacy-review-and-scatter-decomposition.md`. Plan step E2(c) in `learner-efficacy.md`.

## Evidence

- In the plant model, b forgets (lambda 0.99) and s does not, so the sd of s falls from 0.245 to 0.013 with no excitation.
- Past about 150 single-Site rows the slope is applied as if it were known.
- A real oval-only Memory (bug89) applies -18.7 mm at x = +0.125 m with no rows there.

## Test gap

- `test_plant_model.py:206` checks the guard only at 60 rows, where the slope is still uncertain.
- The plan's statement that a Site far from the data gets no offset is false in the long-memory regime.

## Wanted

- Fix the guard so a Site far from the data gets no offset in the long-memory regime.
- Pin it with a test on a long single-Site Memory that asserts no offset at a far Site.
- The numbers and file:line references come from an agent's code review and were not re-read when this Item was filed (hence diagnosis: hypothesis).
