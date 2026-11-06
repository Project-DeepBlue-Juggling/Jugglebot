---
title: "Night Shift 2026-11-06: 3 merged (JB-159, JB-163, JB-176), 2 held (JB-175, JB-169), 1 questioned (JB-167)"
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

**Counts.** 3 merged, 2 held, 1 questioned, 75 skipped; question tickets JB-178, JB-179, JB-180; new bugs JB-181, JB-182. Report: `docs/night-shift/2026-11-06.md`.

**Nightly.** CLAIMED GREEN 8192/8297 passed, 0 failed, 0 errored, 0 xfailed, 105 skipped 2026-11-06T04:03:31+11:00.

**Cost.** The API credit ran out at launch and the night ran on the Claude subscription. 11 Sonnet units; 8 full gate runs. The runner adds the figure to the report.

**What a future night should copy.**

- Review each unit's diff against the real thing before the commit. JB-175's first version missed the guard latch the Item asked for. JB-169's first predicate never fired on a real `compile_columns` schedule, because its hand-built test schedule had no closing REST. Both unit reports read as complete and their scoped tests passed.
- A question's options can be wrong, and an owner's "agreed" does not make them right. JB-167's option 2 assumed the STOP could reach the host within a tick; the heartbeat that carries it is 10 Hz.
- Do not run a bag read during a gate. `test_normal_interpolation_unchanged` failed one gate under that load (JB-182).

**Protocol notes for the owner's Session.**

- `.claude/commands/night-shift.md` § 2g omits `PYTHONPATH=.`: the copied gate cannot import `mapkit` from `temp/`. Run that way, it exits 1 with a traceback. Tonight used `PYTHONPATH=.`, as earlier nights did.
- `mapkit item new` refuses hold tickets as similar to earlier hold tickets for other Items. JB-180 needed `--allow-similar`, and the first refusal was silent behind a `tail`.
- The deny list refuses `git push --delete`, so the remote branches of completed Items stay: JB-21, JB-89, JB-143 and now JB-163.

**Verification.** Each Item's gate result is in its row of the report. Map writes ran `pytest -q tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py` before each push (2026-11-06, last result: 163 passed, 43 skipped). Nothing was flown.
