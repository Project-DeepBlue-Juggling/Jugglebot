---
title: "Night Shift 2026-11-16: 2 merged (BUG-91 on TKT-74, BUG-97), 11 held (BUG-94, BUG-95, BUG-96, BUG-98, BUG-104 built; BUG-90 finished; IDEA-22 re-planned on TKT-73), 4 questions (TKT-75 to TKT-78); BUG-108 filed (a load-stale install test)"
type: investigation
date: 2026-11-16
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-16.md
  - logbook/2026-11-16-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-16

**Counts.** 9 Items in the delta (7 carried from last night, 2 answered tickets): 2 merged (BUG-91, BUG-97), 11 held (7 worked tonight, 4 carried), 4 new question tickets (TKT-75 to TKT-78), 0 unfinished, 2 skipped (the answered TKT-73 and TKT-74, acted on). BUG-108 filed. Report: `docs/night-shift/2026-11-16.md`.

**Nightly.** CLAIMED GREEN 9963/10082 passed, 0 failed, 0 errored, 1 xfailed, 118 skipped 2026-11-16T04:02:57+11:00.

**Cost.** 8 units (1 Opus for BUG-95, 7 Sonnet), 10 Sonnet audits, 9 full suites completed. The API session failed on its first turn (credit balance), so the whole night ran on the Claude subscription. The runner adds the figure to the report.

**What a future night should copy.**

- Run the avoid-words test (`tests/sim/test_glossary_avoid_words.py`) in each worktree before queueing its suite: two branches tonight failed or would have failed a 12-minute suite on the Feed's avoided word in their own logbook entries, and so did one ticket body at `item new`.
- A map push while a branch's suite is running costs that suite: the gate's base-ancestor check fails first and reports nothing else. Batch every `map:` write after the mergeable branches have merged. Held branches gated after that are held at `[pre]` alone, so the report lists their other reasons from the suite log and the audit.
- Last night's lesson about `test_skills_nominate.py::test_columns_at_p4_plans_every_segment_and_matches_the_p3a_probe_cell` was wrong. The test is not order-dependent: the worst rung is a numerical tie (to about 1e-11), and the BLAS thread count breaks it. `run_tests.sh` pins one thread, so scoped runs need `env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1` after the wrapper to agree with the gate.
- `test_install_segment.py::test_TI1_...` goes stale under extra box load and failed two gates, one on a JavaScript-only branch (BUG-108). Do not run extra tests beside a suite.
- A unit's backgrounded pytest can outlive the unit: one ran for 40 minutes at 60 % CPU beside three suites. After each unit returns, check `ps` for a pytest left in its worktree.

## Verification

- 2026-11-16, `tools/night_shift_nonet.sh python -m pytest -q -p no:cacheprovider tests/sim/test_night_shift_report.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py`: 71 passed in 4.98 s.
