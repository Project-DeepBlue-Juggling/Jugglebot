---
title: "Night Shift 2026-11-08: 6 merged (BUG-67, BUG-77, BUG-86, FEAT-28 slice, BUG-36, BUG-59), 2 held (BUG-60, BUG-83), 3 questioned (BUG-72, BUG-75, BUG-80)"
type: investigation
date: 2026-11-08
status: complete
phase: "night-shift (eighth live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-08.md
  - logbook/2026-11-08-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-08

**Counts.** 6 merged, 2 held, 3 questioned, 86 skipped; question tickets TKT-48 to TKT-51 and TKT-52. Report: `docs/night-shift/2026-11-08.md`.

**Nightly.** CLAIMED RED 9085/9199 passed, 1 failed, 0 errored, 0 xfailed, 113 skipped 2026-11-08T04:00:31+11:00. The failure was the race BUG-86 fixes.

**Cost.** The API credit ran out at launch and the night ran on the Claude subscription. 20 Sonnet units; 5 full gate runs and 4 app suites. The runner adds the figure to the report.

**Mistake, and how it was contained.** The first resume merge chained `mapkit gen`, `git add -A` and `git commit` with `;`. `gen` failed on a conflicted Item, and the commit went in with conflict markers in `logbook/INDEX.md` and BUG-77's Item. It was local only, found from the traceback within the same step, and amended before any gate or push. Later merges stopped on a non-zero exit and searched for markers before committing.

**What a future night should copy.**

- Read a skipped Item's reopen rule against new results. BUG-72 says a second separation reopens it, and BUG-79's fail result had said so since 2026-11-07 16:26. Last night's skip reason was about to be copied forward. Nothing in the preflight checks this, so a reopen rule only fires if someone reads it.
- Recount a unit's numbers from the log before they reach the owner. BUG-75's 25 refusals looked wrong beside last night's 15, and a recount by code showed both were right. BUG-72's peaks were recomputed from the install lines and matched.
- Check that a new condition can actually fire on the real path. BUG-67's ledger rule requires a row, so the path to `row=True` was traced, not taken from a hand-built test report.
- Treat an audit note as an order not finished when it is one. BUG-67's first test did not tilt anything, and the owner had ordered a tilted-tail test.
- Audit before the gate where nothing else moves `main`. It saved a second 10-minute run on BUG-67 and BUG-86. Merging one branch can conflict with the next: BUG-77 and BUG-67 both added a keyword to `end_line`, and the conflict was mechanical.

**Left for the owner.** Remote branches of Items already complete can be deleted, since the deny list stops the night doing it: `night-shift/2026-11-01-BUG-31`, `-11-05-BUG-64`, `-11-06-BUG-67`, `-11-06-BUG-68`, `-11-07-BUG-77`.

## Verification

- 2026-11-08, `tools/night_shift_nonet.sh pytest -q tests/sim/test_night_shift_report.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py`: 52 passed.
