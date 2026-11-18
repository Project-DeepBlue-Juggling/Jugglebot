---
title: "The Night Shift plan is archived; path references follow it to plans/archived/"
type: refactor
date: 2026-11-18
status: complete
phase: "night-shift close-out"
subsystem: tooling
related_plan: night-shift.md
files_changed:
  - plans/archived/night-shift.md
  - CLAUDE.md
  - .claude/commands/night-shift.md
  - map/pathway-night-shift.md
  - tools/night_shift.sh
  - tools/night_shift_candidate.sh
  - tools/night_shift_gate.py
  - tools/systemd/README.md
  - tests/sim/test_night_shift_candidate.py
  - tests/sim/test_night_shift_runner.py
  - tests/sim/test_sitting_debrief_protocol.py
---

# The Night Shift plan is archived

## What changed

On the owner's ask, `night-shift.md` moved from `plans/active/` to `plans/archived/` with its filename unchanged (`status: completed`, `completed: 2026-11-13`, `archived: 2026-11-18`) and an archival note. All six steps were complete and its hardening plan was archived earlier the same day. The archive review returned READY TO ARCHIVE; its three wording notes (a stale P1 Outcome line, a stale "C9 pending" line, two integration checks with no dated record) are answered in the archival note and the body is left as written.

Ten files named the plan by its old path. Each reference was rewritten to `plans/archived/night-shift.md`: two in `CLAUDE.md`, one in the Night Shift command, the Pathway Item, the systemd README, and comment or docstring lines in three `tools/night_shift*` files and three test files. No code changed. Pathway PWAY-9 stays active; closing a Pathway is the owner's decision.

## Verification

- 2026-11-18, the path sweep (`grep -rn "plans/active/night-shift" .` outside `logbook/`, `plans/`, `docs/night-shift/`, the generated project map and build output): no match after the edit.
- The gate run on this tree is cited in the commit message.
