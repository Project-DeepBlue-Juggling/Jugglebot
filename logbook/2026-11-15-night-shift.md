---
title: "Night Shift 2026-11-15 (Sunday full walk): 0 merged, 6 held (BUG-91, BUG-92, BUG-93, IDEA-23 built; IDEA-22, FEAT-50 carried), 2 questions (TKT-73, TKT-74); four runner tests fail every Sunday (BUG-107)"
type: investigation
date: 2026-11-15
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-15.md
  - logbook/2026-11-15-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-15

**Counts.** Sunday full walk, 88 Items in the delta: 0 merged, 6 held, 2 new question tickets (TKT-73, TKT-74), 7 unfinished, 75 skipped. 20 unrated Items rated, 70 skip reasons written (the first batch), BUG-107 filed. Report: `docs/night-shift/2026-11-15.md`.

**Nightly.** CLAIMED RED 9890/10018 passed, 4 failed, 0 errored, 1 xfailed, 123 skipped 2026-11-15T04:04:57+11:00. The four failures are `tests/sim/test_night_shift_runner.py` tests that call the runner without `NIGHT_SHIFT_WEEKDAY`, so on a Sunday the full walk ends `OK` where they expect `NOTHING_NEW` (BUG-107). The file is on the never-list, so every Sunday gate fails (a) until a day Session pins the weekday.

**Cost.** 12 units (2 Opus, 10 Sonnet), 4 full suites started, 2 completed. The API credit ran out at about 06:15 and cut two units off (BUG-91, finished by the orchestrator; BUG-90, left as unverified work in progress); the runner resumed the session on the Claude subscription. The runner adds the figure to the report.

**What a future night should copy.**

- Run units in parallel `git worktree`s under the reports directory and serialise the suites with the box-wide lock; but five concurrent units plus a sweep put the box at load 40, the first full suite took over an hour and the Bash 1-hour limit killed it. Run the suite with the 2-hour limit, and no more than three units at once.
- Merge `origin/main` into each branch just before its audit, and push no `map:` commit until that branch's gate has run: every map write moves the base, and three gates tonight stopped at `[pre]`.
- `test_skills_nominate.py::test_columns_at_p4_plans_every_segment_and_matches_the_p3a_probe_cell` fails under `-k` on `main`'s code and passes in a full suite: order-dependent, not a regression.

## Verification

- 2026-11-15, `tools/night_shift_nonet.sh python -m pytest -q -p no:cacheprovider tests/sim/test_night_shift_report.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py`: 71 passed in 5.27 s.
