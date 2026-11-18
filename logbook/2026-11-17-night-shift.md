---
title: "Night Shift 2026-11-17: 0 merged, 12 held (BUG-94, BUG-96, BUG-104 built on TKT-75, TKT-77, TKT-78; BUG-108 fixed; BUG-95 noted on TKT-76; 7 carried), 4 questions (TKT-79 to TKT-82); BUG-109 filed (the ledger turns the legacy ID test red)"
type: investigation
date: 2026-11-17
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-17.md
  - logbook/2026-11-17-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-17

**Counts.** 5 Items in the delta (BUG-108, plus the answered TKT-75 to TKT-78): 0 merged, 12 held (4 built tonight, BUG-95 noted, 7 carried), 4 new question tickets (TKT-79 to TKT-82), 0 unfinished, 4 skipped (the answered tickets, acted on through their parents). BUG-109 filed. Report: `docs/night-shift/2026-11-17.md`.

**Nightly.** CLAIMED RED 9977/10092 passed, 1 failed, 0 errored, 1 xfailed, 113 skipped 2026-11-17T04:00:27+11:00. The one failure, `test_no_legacy_item_ids`, also fails on `main`: the runner's ledger row names four old-form branches whose prune failed, and will again every night (BUG-109). Every gate tonight would have held at (a) for it.

**Cost.** 4 Sonnet builders, 4 Sonnet audits, 4 full suites. The session started on the API and was resumed on the Claude subscription when the credit balance ran out; the runner adds the figure to the report.

**What a future night should copy.**

- The map pushes again landed after the branches had merged `main` (BUG-109's filing, its ticket and BUG-95's note), so every gate held at `[pre]` first. Last night's lesson stands: hold every `map:` write until after the gates.
- zsh does not word-split an unquoted `$var`, so `for f in $files` runs once over the whole list. A conflict loop written that way committed conflict markers on three branches; the commits were local, were reset, and the merges were redone. Iterate with Python, or a `${(f)...}` split.
- BUG-108's diagnosis got live support: the BUG-96 suite, which lacks the fix, failed `test_TI1` with `robot_state telemetry stale`.

## Verification

- 2026-11-17, `tools/night_shift_nonet.sh python -m pytest -q -p no:cacheprovider tests/sim/test_night_shift_report.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py tests/sim/test_no_legacy_item_ids.py`: 92 passed, 1 failed in 6.58 s; the one failure is `test_no_legacy_item_ids` on `docs/night-shift/history.jsonl` alone (26 against 22 pinned, BUG-109), as on `main` before this commit.
