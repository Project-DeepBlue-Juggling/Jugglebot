---
title: "Night Shift 2026-11-10: 1 merged (BUG-87 log line), 4 held (FEAT-47, carried BUG-60, BUG-83, BUG-80), 5 questions (TKT-57 to TKT-61)"
type: investigation
date: 2026-11-10
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-10.md
  - logbook/2026-11-10-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-10

**Counts.** 1 merged, 4 held (1 new, 3 carried), 5 new question tickets (TKT-57 to TKT-61), 99 skipped. PWAY-10 (GUI Improvements) was created at the owner's request in IDEA-21. Report: `docs/night-shift/2026-11-10.md`.

**Nightly.** CLAIMED GREEN 9211/9313 passed, 0 failed, 0 errored, 0 xfailed, 102 skipped 2026-11-10T04:01:31+11:00.

**Cost.** 5 Sonnet units and 3 full gate runs, all `EXIT=0`. The runner adds the figure to the report.

**Repaired from 2026-11-09.** That night ended with a unit still running and filed no ticket for its two re-held app branches. Tonight filed them (TKT-57, TKT-58) and wrote BUG-83's missing hold note. Every unit tonight was waited on before the next step, and no background work is left running at the report.

**What a future night should copy.**

- A narrow first slice merges where a whole Feature holds. BUG-87's log line passed audit with no behaviour finding. FEAT-47's whole counter held on a path the brief never named: `magazine` writes a placeholder not-caught report.
- When carrying skip reasons forward by script, parse only the Skipped table. A 4-column regex also matched the old report's Sitting-results table, and four rows briefly read `pass` or `fail` as their reason before review caught them.
- The deny list matches words inside a `--note` too. A note naming the ROS build tool was refused; reword it, never work around it.

## Verification

- 2026-11-10, `tools/night_shift_nonet.sh pytest -q tests/sim/test_night_shift_report.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py`: 53 passed in 3.66 s.
