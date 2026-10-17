---
title: Plans board cleared - eleven plans archived at the close of the skill-stack arc
type: refactor
date: 2026-10-14
status: resolved
phase: "two-ball-skill-stack — close-out"
related_plan: skill-stack-dress-rehearsals-and-residue.md
related:
  - 2026-10-14-skill-stack-dress-rehearsals-flown.md
  - 2026-10-13-skill-stack-r6-close-out.md
files_changed:
  - plans/archived/ (eleven plans moved by git mv, filenames unchanged)
  - plans/active/INDEX.md
  - plans/parked/INDEX.md
  - plans/archived/INDEX.md
  - logbook/2026-10-14-skill-stack-possession-precondition-at-accept.md
  - logbook/INDEX.md
  - CLAUDE.md
  - attic/README.md
  - controller/README.md
  - logbook/README.md
  - .scratch/r4-throw-precision/ (removed from the tree)
subsystem:
  - docs
tags:
  - archival
  - docs
---

# Plans board cleared

## Summary

Owner decision, 2026-10-14, at the close of the skill-stack arc: "archive all the plans, clean everything up". Eleven plans moved to `plans/archived/` by `git mv` (filenames unchanged, per the filename-never-changes rule). Each carries `status` / `archived:` frontmatter and an "## Archival note (2026-10-14)" naming what landed and "Would revive it".

- **From `plans/active/` (6):** `skill-stack-dress-rehearsals-and-residue` (completed; plan-reviewer READY, zero residual xfails), `columns-tracking-ff-and-shaping`, `odrive-config-drift-assertion`, `leg-bus-frame-drops` (superseded, nothing implemented), `kinematic-calibration` (superseded; steps 1-5 landed, 6-7 never run), `bridge-clock-frequency-discipline` (superseded; phases 1-2 landed, 3-5 not).
- **From `plans/parked/` (5; now empty by design):** `accel-ff-inertia`, `learned-ff-residuals`, `clapboard-can3-integration` (superseded, nothing implemented; the clapboard branch stays as the record), `levelling-frame-contract`, `refactor-2026-07` (completed).
- **Left on the active board:** `leg-gain-tuning-methodology.md` - a normative reference, not a plan.

## Live findings carried forward

The rehearsals plan's § 2 is retitled "Open residue, unscheduled", every row tagged (a) done / (b) open, worth a plan / (c) trivial. The (b) rows are the live findings: `SPLICE_TOO_LATE` self-ends (5 ends at sitting 8, 3 of the 12 fed attempts) and the 47 ms emit-gap max against the < 35 ms target.

## Related edits

- `2026-10-14-skill-stack-possession-precondition-at-accept.md` flipped to resolved with rehearsal row 21's evidence.
- CLAUDE.md's arc paragraph re-pointed; path-qualified `plans/active|parked/` references re-pointed in live docs, deliberately left in dated logbook entries and code comments (history).
- The tracked-by-mistake `.scratch/r4-throw-precision/` map removed from the tree.

## Verification

2026-10-14, `pytest -q -p no:cacheprovider tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py`: 107 passed, 2 skipped in 1.43 s. The full-gate triple is recorded in the commit message.
