---
title: "Night Shift 2026-11-13: 1 merged (IDEA-21), 2 held (IDEA-22, carried BUG-80), 1 question (TKT-67)"
type: investigation
date: 2026-11-13
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-13.md
  - logbook/2026-11-13-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-13

**Counts.** 1 merged, 2 held (1 resumed, 1 carried), 1 new question ticket (TKT-67), 110 skipped. Both answered tickets from last night were acted on (TKT-65, TKT-66). Report: `docs/night-shift/2026-11-13.md`.

**Nightly.** CLAIMED GREEN 9412/9512 passed, 0 failed, 0 errored, 0 xfailed, 100 skipped 2026-11-13T04:00:31+11:00.

**Cost.** 2 builder units (1 Sonnet, 1 Opus), 2 Sonnet audits, 1 Sonnet fixer; 2 completed full gate runs, 1 stopped early. The runner adds the figure to the report.

**What a future night should copy.**

- Audit before the first full gate when the audit will likely touch the tree: IDEA-21's first gate was stopped and re-run after its audit fixes, and IDEA-22's single gate ran after its audit.
- A resumed branch that carries the Item's closure conflicts on the Item's frontmatter, `logbook/INDEX.md` and `docs/project-map.md` when `origin/main` is merged in. The resolution is mechanical: keep the branch's `status: complete` with `owners: []`, keep both INDEX rows, and regenerate with `python -m mapkit gen`.
- A changed Sitting section on a held branch is written identically on `main` in the hold write, so a later merge of the branch is clean.
- A resume answer can move a feasibility figure the owner relied on. TKT-65's 3 deg was chosen from the +y margins, and about +x it is refused. Re-run the margin the answer depends on before calling the answer applied.

## Verification

- 2026-11-13, `tools/night_shift_nonet.sh python -m pytest -q tests/sim/test_night_shift_report.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py`: 53 passed in 3.78 s.
