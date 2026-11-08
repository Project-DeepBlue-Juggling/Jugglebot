---
title: "Night Shift 2026-11-07: 3 merged (BUG-68, BUG-76, BUG-71), 2 held (BUG-67, BUG-77), 3 questioned (FEAT-40, BUG-75, BUG-80)"
type: investigation
date: 2026-11-07
status: complete
phase: "night-shift (seventh live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-07.md
  - logbook/2026-11-07-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-07

**Counts.** 3 merged, 2 held, 3 questioned, 82 skipped; question tickets TKT-43 to TKT-47. BUG-70 has findings and no ticket. Report: `docs/night-shift/2026-11-07.md`.

**Nightly.** CLAIMED GREEN 8752/8862 passed, 0 failed, 0 errored, 0 xfailed, 110 skipped 2026-11-07T04:03:27+11:00.

**Cost.** The API credit ran out at launch and the night ran on the Claude subscription. 19 Sonnet units and 1 Opus; 9 full gate runs. The runner adds the figure to the report.

**What a future night should copy.**

- A diagnosis unit can withdraw the Item's own cause. BUG-80's brief called the Item's numbers "a claim to check, not a fact", and the unit found no over-strong flyback gain: the planner returns 1.00 mm per mm. A fix built from the Item's title would have traded x for y.
- Mutation-check what a unit leaves unconfirmed. BUG-67's unit stopped while waiting on its own test run with one test red; the production edits were sound, the test stub was wrong, and four reverts showed each new test fails without its change.
- An owner's answer can reach further than it was asked to. TKT-42's N/N count also adds a Catch with no evidence and one on an aborted goal. Only the audit saw that, so TKT-44 asks again.

**Protocol notes for the owner's Session.**

- Write briefs with the Write tool before launching their units. A shell heredoc whose text names forbidden tools is refused whole, and tonight three units launched against files that did not exist. They stopped cleanly and changed nothing.
- A unit could not grep `tools/probes/` from the shell, so BUG-77's brief check of line readers missed `feed_catch_bag_probe.py`; the audit caught it. A brief about log text should name that directory for a Read, or the orchestrator should grep it first.
- `.claude/commands/night-shift.md` § 2g still omits `PYTHONPATH=.`, and the hold-ticket command still needs `--allow-similar`. The `set-status` in § 2j needs `--owner night-shift` even when the Item is already held by night-shift.
- The remote branches of completed Items stay, since the deny list refuses `git push --delete`: BUG-2, BUG-31, BUG-54, BUG-64 and now BUG-68.

**Verification.** Each Item's gate result is in its row of the report. Map writes ran `pytest -q tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py` before each push (2026-11-07, last result: 179 passed, 45 skipped). Nothing was flown.
