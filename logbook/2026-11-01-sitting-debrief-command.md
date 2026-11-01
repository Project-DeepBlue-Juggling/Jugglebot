---
title: "/sitting-wrapup becomes /sitting-debrief: an analysis phase (inventory, parallel verdict and diagnosis units, crosswalk, write-backs) runs before the unchanged paperwork"
type: feature
date: 2026-11-01
status: complete
phase: "project map"
related_entries:
  - 2026-10-31-night-shift-p5-sitting-wrapup.md
files_changed:
  - .claude/commands/sitting-debrief.md
  - tests/sim/test_sitting_debrief_protocol.py
  - mapkit/sitting.py
  - docs/runbook.md
  - tools/night_shift_gate.py
  - .claude/commands/night-shift.md
  - plans/active/night-shift.md
  - logbook/INDEX.md
---

# /sitting-debrief

**What and why.** The owner closed Sittings with `/sitting-wrapup` (paperwork only) and did the
data analysis by hand in a separate Session. One command now does both: section 1 inventories
every goal and attempt end in the new launch logs and bags, asks one `AskUserQuestion`
(verdict source, fix scope, physical observations), runs parallel scratchpad units (Runbook
verdicts, one diagnosis per attempt-killer class testing the open Item's hypothesis first,
a crosswalk of plan steps and Items), then writes back to Items, plans, patterns and the
logbook. Sections 2 to 6 are the old wrap-up steps, unchanged. `--dry-run` stops after the
verdict table. Threshold changes (limits, margins, guards) come back as owner decisions.

**Rename.** Live references renamed (command, test, `mapkit/sitting.py` Close-out line,
`docs/runbook.md` regenerated, gate docstring, `night-shift.md`, the plan). The logbook entry
name is now `<date>-sitting-debrief.md`; the last-debrief lookup accepts both
`*-sitting-debrief.md` and the older `*-sitting-wrapup.md`. Historical logbook entries keep the
old name.

**Verification.** 2026-11-01, `pytest tests/sim/test_sitting_debrief_protocol.py
tests/sim/test_night_shift_gate.py tests/sim/test_mapkit*.py tests/sim/test_logbook_front_matter.py
tests/sim/test_docs_links.py tests/sim/test_plans_index.py -q`: see the commit message for the result.
