---
title: "The Runbook becomes an index; the Sitting sheet is the only ordering authority (close-out C11 a)"
type: refactor
date: 2026-11-14
status: complete
related_plan:
  - night-shift-closeout.md
files_changed:
  - mapkit/sitting.py
  - docs/runbook.md
  - tests/sim/test_sitting_sections.py
  - tests/sim/test_sitting_debrief_protocol.py
  - logbook/2026-11-14-runbook-becomes-an-index.md
  - logbook/INDEX.md
subsystem:
  - mapkit
tags:
  - runbook
  - sitting-sheet
---

# The Runbook becomes an index (close-out C11, part a)

**What.** `docs/runbook.md` kept its own ordering (priority weight per minute, blocks by setup set, `first:` pins) next to the Sitting sheet's, which is what a Sitting actually flies. The owner uses only the sheet, so the two orders could disagree and the Runbook's was never the one flown. Decision: the sheet is the only ordering authority. `mapkit/sitting.py` loses `_score`/`PRIORITY_WEIGHT`, the block grouping, "Setup 0" and "Close-out". The page is now, per rig, a table `| Item | Minutes | Setup | Branch | On a sheet |` (rows by priority then ID), the header line "Fly from a Sitting sheet: /sitting-sheet robot|bench-leg|bench-hand", `## Needs a Sitting section` and the open-question list. `first:` is parsed and ignored.

**On a sheet.** The newest `map/sheets/*.yaml` whose steps `serve` the Item, while the Item has no `## Sitting result` dated on or after the sheet's date; derived from Items and sheet files only, never the app's `.results.yaml`, so the committed page stays reproducible from a commit.

**Compatibility.** `collect()` keeps `blocks` (one block per rig, every score 0) and `rigs`, so the deployed map app (which reads `blocks`) keeps working; part (b) in the app repo reads the same field. Dropping it is part (c).

**Not done here (frozen or later).** The two protocol files (`.claude/commands/night-shift.md`, `sitting-debrief.md` still say the Runbook's `## Close-out` promises the debrief), `docs/agents/map-writes.md:62,66`, `mapkit/sheet.py` docstring ("in Runbook order"), and the GLOSSARY entries are reworded in the final docs step.

## Verification

- 2026-11-14, `python -m pytest -q -n 2 tests/sim/test_sitting_*.py tests/sim/test_mapkit*.py tests/sim/test_night_shift_protocol.py tests/sim/test_night_shift_landed.py tests/sim/test_logbook_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py`: 439 passed in 34.12 s; after adding this entry and regenerating, the logbook and mapkit subset: 381 passed in 31.13 s.
