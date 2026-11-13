---
title: "Night Shift close-out recorded as a plan: steps C1 to C17 under PWAY-9, owner decisions of 2026-11-13"
type: refactor
date: 2026-11-13
status: complete
phase: night-shift
subsystem: night-shift
files_changed:
  - plans/active/night-shift-closeout.md
  - plans/active/night-shift.md
  - plans/active/INDEX.md
  - logbook/2026-11-13-night-shift-closeout-plan.md
  - logbook/INDEX.md
tags:
  - night-shift
  - plan
---

# Night Shift close-out plan

**What.** The close-out design (steps C1 to C17: gate bound to the merge, delta walk with NOTHING_NEW, skip reasons on the Item, runtime records under `temp/runtime`, notes-only Sitting sheets, a tested debrief merge script, an index Runbook) is now `plans/active/night-shift-closeout.md` (222 lines), under Pathway `night-shift`, all steps `not_started`. The design's Owner questions section is replaced by a Decisions section holding the owner's answers of 2026-11-13. `night-shift.md` changed by `last_updated` and one pointer line under R6 (status untouched).

**Why.** The design lived only in a scratchpad file; a plan holds decisions, interfaces and status where the next Session finds it. No code or behaviour changed.

**Verification.** Docs only; tier 3, named tests (`pytest tests/sim/test_plans_index.py tests/sim/test_mapkit_docs.py tests/sim/test_glossary_avoid_words.py tests/sim/test_docs_links.py tests/sim/test_mapkit_steps.py tests/sim/test_mapkit_plans.py tests/sim/test_logbook_front_matter.py tests/sim/test_night_shift_runner.py tests/sim/test_night_shift_candidate.py -q`, run 2026-11-13): 133 passed in 95.74 s.
