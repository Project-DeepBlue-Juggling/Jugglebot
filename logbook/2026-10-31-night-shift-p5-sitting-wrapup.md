---
title: "Night Shift P5: the /sitting-wrapup command (per-Item merges of passed held branches, never the candidate branch; single-writer check; records, follow-ups, logbook, Runbook regeneration) and the protocol's unaddressed-results step"
type: feature
date: 2026-10-31
status: complete
phase: "night-shift P5"
related_plan:
  - night-shift.md
related_entries:
  - 2026-10-31-night-shift-p4-sitting-sections-and-runbook.md
files_changed:
  - .claude/commands/sitting-wrapup.md
  - .claude/commands/night-shift.md
  - tests/sim/test_sitting_wrapup_protocol.py
subsystem:
  - docs
  - tests-tooling
tags:
  - night-shift
  - runbook
---

# Night Shift P5: Sitting wrap-up command

## What

`/sitting-wrapup <date> [--dry-run]` is the one prompt that closes a Sitting flown from the
Next Sitting Runbook: it reads every `## Sitting result (<date>)` newer than the last wrap-up,
merges each passed Item's held branch into `main` one at a time (a conflict aborts that merge
and leaves the branch held with the files named; the candidate branch itself is never merged),
closes passed Items and keeps failed ones `in_progress` with the note, returns the live
checkout to `main`, rebuilds `ros_ws` if a merged branch touched it, commits `docs/records.md`,
files follow-ups from the notes, writes the wrap-up logbook entry, regenerates the Runbook, runs
the per-commit gate and makes one commit with the trailer. It runs in `~/Desktop/Jugglebot`
under the single-writer rule (fetch and status first, pause on anything unexpected, never
stash, never force). The Night Shift protocol gains its one P5 step: `## Unaddressed Sitting
results` lists results newer than the last wrap-up, and the orchestrator never merges a
Sitting's branches itself. `tests/sim/test_sitting_wrapup_protocol.py` pins that the command
file names every close-out promise and that the generated Runbook points at it.

## Verification

- Scoped, 2026-10-31: `pytest -q tests/sim/test_sitting_wrapup_protocol.py` 24 passed in 0.45 s (22 at first; two promises added after the audit: the `--waive c` gate re-run and the held-branch lookup).
- Full gate before the commit, 2026-10-31 (`./run_tests.sh --full`): run 2026-10-31 00:56-01:04 on the final tree with no other load: **parallel 7764 passed, 65 skipped in 482.66 s; serial 8 passed in 32.55 s; exit 0** (an earlier run at 00:37, under five concurrent agents, had 5 parallel failures that passed scoped and 2 serial wall-clock failures, all load).
- R6 (the first Runbook-driven Sitting) is the hardware verification and is not this entry's.
