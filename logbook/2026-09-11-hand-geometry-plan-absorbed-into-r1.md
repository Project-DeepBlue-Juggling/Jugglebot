---
title: "Hand geometry correction archived — its measurement is absorbed into skill-stack R1 as one measured key, its branch stays unmerged, and a second owner reading confirms the scale to 0.01 %"
type: refactor
date: 2026-09-11
status: resolved
phase: "two-ball-skill-stack — between R0 and R1 (board change + R1 prompt amendment)"
related_plan: two-ball-skill-stack.md
files_changed:
  - plans/archived/hand-geometry-correction.md (moved from plans/active/, archival note added)
  - plans/active/INDEX.md
  - plans/archived/INDEX.md
  - plans/active/two-ball-skill-stack.md
  - plans/active/PROMPT-skill-stack-r1.md
  - logbook/2026-09-08-fw18-bundle-hand-clip-homing-counters-rename.md (path reference re-pointed)
  - logbook/2026-09-11-hand-geometry-plan-absorbed-into-r1.md
  - logbook/INDEX.md
subsystem:
  - motion
  - config
tags:
  - docs
---

# Hand geometry correction archived, measurement absorbed into R1

Board change and prompt amendment only; no code. Owner question 2026-09-11: complete
`hand-geometry-correction.md` before R1? Answer: no, as written it builds on the two
things R1 deletes (its 70-file G2 build targets the FSM toss stack and the Platform
Teensy stroke engine; its Platform-reflash, ILC-refit and G3 obligations all attach to
them). What matters is one physical number, the hand's millimetres per revolution, and it
belongs in the prior, not the learner: a 3 % rev-to-metre error also moves the cup's
actual height at the catch by about 9 mm at catch-prime, which the learner's outcome
(landing xy, flight) never sees.

The owner re-measured the stroke 2026-09-11 by sliding the hand to each stop and reading the
encoder: top 10.691, bottom −0.118 rev (2026-09-06: 10.701 / −0.107). Spans 10.809 vs
10.808 rev over 352.0 mm — the scale agrees to 0.01 %; the 0.011 rev end shift is
homing-zero variation (~0.36 mm). R1 adopts the mean, **32.567 mm/rev**, as a single
measured key `hand_mm_per_rev`, deleting `linear_gain_factor` and `hand_spool_radius_m`
rather than re-valuing them; the R1 runbook gains a stop-to-stop confirmation row on the
streamed lane (span 10.81 ± 0.01 rev) that replaces G3. The branch and worktree stay as
the record and are not merged.

## Verification

Docs only, no `*.py` or `*.yaml` touched. Named tests (2026-09-11): `pytest
tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py
tests/sim/test_logbook_search.py -q` — result recorded in the commit message. The path
sweep for the moved plan: one tracked file before, zero after. The phase-cadence audit for
this change is R1's end-of-rung audit (the per-phase rule; R0's ran on 2026-09-10).
