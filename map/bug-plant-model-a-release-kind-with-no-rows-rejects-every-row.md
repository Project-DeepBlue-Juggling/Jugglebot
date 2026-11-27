---
id: BUG-153
kind: bug
title: 'Plant model: a release kind with no rows rejects every row more than about 120 mm off its prior, so a cold carried kind cannot learn the oval''s 179 mm offset'
status: inbox
created: '2026-11-28'
updated: '2026-11-27T13:55:33Z'
owners: []
priority: low
effort: s
hardware: none
area: learner
blocked_by: []
links: []
parent: learner-generalisation
filed_by: claude-e2
---

A release kind at its prior measures the plant model's 100 mm lateral gate against an offset of 0. The Apex gate widens while K is uncertain; the lateral one does not, and nothing re-seeds.

## Evidence

- Probe, 5 seeds, 300 rows each: offsets up to 120 mm are learned (294 to 300 rows accepted); at 150 mm 0 to 128 are accepted; at 179 mm, the oval's measured offset, 0 of 300.
- The live Memory does not hit it: carried accepted 1083 of 1085 rows on 2026-11-27.
- Pinned by a strict xfail in `tests/motion/test_plant_model.py` (the 179 mm case) beside a passing 120 mm test.

## Options

1. Accept a row after about five consecutive rejections whose innovations agree within the band; one wild row resets the run.
2. A wider prior sd on b for the carried kind.
3. A widening term on the lateral gate like the Apex gate's; alone it reaches only about 112 to 127 mm.

Any of these moves the model command, which the Learner's 30 mm cap does not bound, so it needs a replay. It matters only when the oval flies on a cold carried kind. Found at the BUG-138 merge, `logbook/2026-11-26-plant-model-lateral-law-per-release-kind.md`.
