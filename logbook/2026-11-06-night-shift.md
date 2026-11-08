---
title: "Night Shift 2026-11-06: 3 merged (BUG-61, BUG-64, BUG-69), 2 held (BUG-68, BUG-67), 1 questioned (FEAT-40)"
type: investigation
date: 2026-11-06
status: complete
phase: "night-shift (sixth live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-06.md
  - logbook/2026-11-06-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-06

**Counts.** 3 merged, 2 held, 1 questioned, 75 skipped; question tickets TKT-40, TKT-41, TKT-42; new bugs BUG-70, BUG-71. Report: `docs/night-shift/2026-11-06.md`.

**Nightly.** CLAIMED GREEN 8192/8297 passed, 0 failed, 0 errored, 0 xfailed, 105 skipped 2026-11-06T04:03:31+11:00.

**Cost.** The API credit ran out at launch and the night ran on the Claude subscription. 11 Sonnet units; 8 full gate runs. The runner adds the figure to the report.

**What a future night should copy.**

- Review each unit's diff against the real thing before the commit. BUG-68's first version missed the guard latch the Item asked for. BUG-67's first predicate never fired on a real `compile_columns` schedule, because its hand-built test schedule had no closing REST. Both unit reports read as complete and their scoped tests passed.
- A question's options can be wrong, and an owner's "agreed" does not make them right. FEAT-40's option 2 assumed the STOP could reach the host within a tick; the heartbeat that carries it is 10 Hz.
- Do not run a bag read during a gate. `test_normal_interpolation_unchanged` failed one gate under that load (BUG-71).

**Protocol notes for the owner's Session.**

- `.claude/commands/night-shift.md` § 2g omits `PYTHONPATH=.`: the copied gate cannot import `mapkit` from `temp/`. Run that way, it exits 1 with a traceback. Tonight used `PYTHONPATH=.`, as earlier nights did.
- `mapkit item new` refuses hold tickets as similar to earlier hold tickets for other Items. TKT-42 needed `--allow-similar`, and the first refusal was silent behind a `tail`.
- The deny list refuses `git push --delete`, so the remote branches of completed Items stay: BUG-2, BUG-31, BUG-54 and now BUG-64.

**Verification.** Each Item's gate result is in its row of the report. Map writes ran `pytest -q tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py` before each push (2026-11-06, last result: 163 passed, 43 skipped). Nothing was flown.
