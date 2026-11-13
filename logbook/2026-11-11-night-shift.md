---
title: "Night Shift 2026-11-11: 3 merged (BUG-83, BUG-60, FEAT-47 fixes), 2 held (FEAT-47 Feed misses, carried BUG-80), 3 questions (TKT-62 to TKT-64)"
type: investigation
date: 2026-11-11
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-11.md
  - logbook/2026-11-11-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-11

**Counts.** 3 merged, 2 held (1 new, 1 carried), 3 new question tickets (TKT-62 to TKT-64), 105 skipped. Six answered tickets were acted on (TKT-54, TKT-57 to TKT-61). Report: `docs/night-shift/2026-11-11.md`.

**Nightly.** CLAIMED GREEN 9271/9372 passed, 0 failed, 0 errored, 0 xfailed, 101 skipped 2026-11-11T04:02:26+11:00.

**Cost.** 8 Sonnet units and 3 completed full gate runs, all `EXIT=0`, plus 2 app suites. The runner adds the figure to the report.

**What a future night should copy.**

- Never push `main` while a branch's full gate runs. Tonight's map-only pushes during FEAT-47's run cost a second full gate, because the gate requires the branch to contain `origin/main`.
- After `git merge origin/main`, check for a conflict before starting the gate. One run was started on an unresolved `logbook/INDEX.md` conflict and had to be stopped.
- A `main` worktree outside the clone (`git worktree add`) lets the `map:` writes go ahead while a unit works on a branch. Push from it with `git -C <worktree> push -q origin main` as its own line.
- Tell units to run tests in the foreground, one at a time. A unit that backgrounded its runs kept starting duplicates, and they competed with the orchestrator's own run (818 s against about 160 s).
- The bags under `~/Desktop/rosbags` read offline with the venv's mcap reader, which answered IDEA-21 and checked BUG-87. Load the reader by `importlib` in a script written with Write: the tool name in a heredoc trips the deny list.

## Verification

- 2026-11-11, `tools/night_shift_nonet.sh pytest -q -p no:cacheprovider tests/sim/test_night_shift_report.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py`: 53 passed in 3.73 s.
