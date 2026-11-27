---
id: BUG-139
kind: bug
title: 'Plant model slope value stays corrupted after a long single-Site Memory: one row at a second Site applies a line up to 59 mm off'
status: complete
created: '2026-11-25'
updated: '2026-11-27T13:55:49Z'
owners:
- night-shift
priority: medium
effort: s
hardware: none
owner: decision
radius: subsystem
diagnosis: confirmed
area: learner
blocked_by: []
links:
- bug-learner-far-site-guard-fails-on-long-single-site-memories
- logbook/2026-11-27-plant-model-process-noise-on-b.md
parent: learner-generalisation
filed_by: claude
---

The residue of BUG-137, found by the Night Shift unit on 2026-11-25. The fix makes every at-prior decision read a no-forgetting covariance, so a single-Site Memory now applies 0 at a far Site. The forgetting filter's VALUE of s is still corrupted by the information its per-parameter forgetting manufactures. That was already so before the fix.

## Evidence

- After 1500 single-Site rows and then 60 rows at a second Site, the far-Site line is off by up to about 25 mm. At 300 single-Site rows it is about 12 mm; after only 5 second-Site rows it is about 50 mm. Probe: 6 seeds, `_one_site` and `_second_site` in `tests/motion/test_plant_model.py`.
- Pinned as `test_bug137_residue_the_slope_value_recovers_after_a_long_single_site_memory` in the same file, `xfail(strict=True)`.
- The live Memory copy (5455 rows, 2026-11-21) is not affected at any flown Site: its replay applies identical offsets with and without the BUG-137 fix.

## Why it needs a decision

The fix changes the forgetting itself, so it moves the P1 probe's pin (`learner_replay.py`, the "band+struct" variant).

## Options

1. Process noise on b (a random walk, P_bb += q per row) in place of P / sqrt(lambda lambda). This forgets b without creating information about s.
2. Directional forgetting: forget only in the excited direction.
3. Forget s as well (one lambda for every parameter). The probe rejected this: s falls back to its prior after 410 to 1958 single-Site rows.
4. Leave it. The guard now holds the far Site at the identity, so the residue matters only in the first rows after a long single-Site run.

Logbook: `logbook/2026-11-25-plant-model-far-site-guard-unforgotten-information.md`. Plan step E2 of `learner-efficacy.md`.

## Progress

- 2026-11-27: Night Shift 2026-11-27: held on night-shift/2026-11-27-BUG-139: process noise on b (TKT-95 option 1), xfail passes, 0 audit findings open; unblocked by a day Session merge
- 2026-11-28: Merged with BUG-138 (process noise inside each release kind's lateral filter) after the full gate and the sim gate.
