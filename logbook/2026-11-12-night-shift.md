---
title: "Night Shift 2026-11-12: 1 merged (BUG-88), 3 held (IDEA-22, IDEA-21, carried BUG-80), 2 questions (TKT-65, TKT-66)"
type: investigation
date: 2026-11-12
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-12.md
  - logbook/2026-11-12-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-12

**Counts.** 1 merged, 3 held (2 new, 1 carried), 2 new question tickets (TKT-65, TKT-66), 108 skipped. Two answered tickets were acted on (TKT-62, TKT-63). Report: `docs/night-shift/2026-11-12.md`.

**Nightly.** CLAIMED GREEN 9405/9506 passed, 0 failed, 0 errored, 0 xfailed, 101 skipped 2026-11-12T04:03:27+11:00.

**Cost.** 3 builder units (2 Sonnet, 1 Opus), 3 Sonnet audits, 1 Sonnet fixer; 4 completed full gate runs. The runner adds the figure to the report.

**What a future night should copy.**

- Two units built in parallel in `git worktree`s under `temp/reports/night-shift/wt/` while the main clone ran BUG-88's gate. The tests resolve the package from the tree they run in, so a worktree gate tests its own branch.
- A branch cut before an earlier Item merges fails the gate's base precondition. Merge `origin/main` into it (a `logbook/INDEX.md` top-row conflict is mechanical: keep both rows), regenerate with `python -m mapkit gen`, then gate once.
- Two hold tickets in a row trip `item new`'s similar-title screen against older hold tickets; `--allow-similar` is right for a new hold.
- A unit's `until pgrep -f "<its own pattern>"` wait loop never exits because it matches itself; tonight's was killed by hand.

## Verification

- 2026-11-12, `tools/night_shift_nonet.sh python -m pytest -q tests/sim/test_night_shift_report.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py`: 53 passed in 3.78 s.
