---
id: TKT-83
kind: ticket
title: 'FEAT-59: retire all three unused modules with their tests, retire the ILC and vel-FF tools only and keep toss_release, or leave all three?'
status: open
created: '2026-11-19'
updated: '2026-11-18T15:54:04Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: tests-tooling
blocked_by: []
links: []
parent: feature-retire-the-tests-of-unused-modules-together-with-the-modules
type: question
---

FEAT-59 retires about 180 tests that cover code nothing live imports, deleting each module in the same commit as its tests. It needs your call on whether each of the three modules is really finished with.

## Options

1. **Retire all three now:** `toss_release`, the ILC measurement tools and the vel-FF plumbing check, with their tests. The live comments that cite `toss_release` and the `ball_possession.py` log message that names `toss_release.stow_to_global_mm` are reworded in the same commit.
2. **Retire the ILC and vel-FF tools only; keep `toss_release`:** five live modules and one log message still name it as the stow-to-global conversion reference.
3. **Leave all three:** they cost about 30 CPU-seconds a run.

## What each answer unblocks

- 1 or 2: the next night builds the deletion on a branch, with the importer grep quoted, and merges it if the gate says MERGE.
- 3: FEAT-59 is dropped.

Recommendation: 2. Both tools say they are dormant. Retiring `toss_release` means rewording its live references, so it is a separate small refactor.
