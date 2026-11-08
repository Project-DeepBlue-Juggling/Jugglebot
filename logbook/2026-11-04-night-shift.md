---
title: "Night Shift 2026-11-04: 2 merged (BUG-61 chart callouts no longer overlap; BUG-65 the opening-REST tick load flake, found and fixed tonight), 0 held, 1 question (TKT-35 for BUG-64); last night's six answers were already acted on by the owner's Session"
type: investigation
date: 2026-11-04
status: complete
phase: "night-shift (fourth live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-04.md
  - logbook/2026-11-04-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
  - report
---

# Night Shift 2026-11-04 (fourth live night)

**Counts.** There were 70 candidates. Merged 2 (BUG-61, BUG-65), held 0, 1 question ticket (TKT-35), skipped 68. Report: `docs/night-shift/2026-11-04.md`.

**Nightly.** `CLAIMED GREEN 8111/8206 passed, 0 failed, 0 errored, 0 xfailed, 95 skipped 2026-11-04T04:02:27+11:00`.

**What the night found.** All six of last night's question tickets (TKT-29 to TKT-34) were answered, and the owner's Session had already carried out every answer on 2026-11-03. So there was nothing to resume. Three `night-shift/*` branches of complete Items are still on the remote and were left in place. Two new GUI bugs were in reach because the robot GUI is in this repo (`ros_ws/gui`); the map app is not. BUG-61 was fixed. BUG-64 turned out not to be a search bug: its options went to the owner. BUG-61's first full gate failed once on an unrelated skill-node test; a probe confirmed a fourth unfrozen fixture clock, filed and fixed as BUG-65. The gate script copy needs `PYTHONPATH=.` to import `mapkit` when run from the reports directory.

**Verification.** Full gates (`./run_tests.sh --full`, 2026-11-04): BUG-61 on 3f76aa56, 8154 passed + 8 serial, `EXIT=0`; BUG-65 on 217fa2bf, 8156 passed + 8 serial, `EXIT=0`. Both got `MERGE` from `origin/main`'s `tools/night_shift_gate.py`. Map-write check (`pytest -q tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py`) green before each `map:` push (147, then 148 passed).

**Cost.** API until the credit ran out at launch, then the Claude subscription (opus). Units: 3 Sonnet. Wall clock: 04:46 to 05:50 AEDT.
