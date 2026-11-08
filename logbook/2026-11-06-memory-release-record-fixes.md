---
title: "The Memory row records the release the Throw used, and a refused re-send no longer leaves the release record at the refused aim (BUG-81)"
type: bugfix
date: 2026-11-06
status: complete
phase: "BUG-81"
related_plan: learner-generalisation.md
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_bump.py
  - logbook/2026-11-06-memory-release-record-fixes.md
subsystem: [motion]
tags: [learner, correctness]
---

# Memory release record fixes (BUG-81)

**What.** Two fixes in `motion/skills/executor.py`, both found by the Carry creep probe
(`logbook/2026-11-06-arrival-pull-and-hand-dropout-probes.md`) and approved by the owner on 2026-11-06.

- F1: the pending Memory row of a carried Throw was registered once at dispatch and never refreshed, so on a Catch
  the tracker re-aimed (92 of 98 two-ball Throws at the P2 Sitting) the row's `rel_x`, `rel_y`, `seat_x`, `seat_y`
  held the dispatch aim, not the release that flew. The row is now refreshed after each accepted re-send or bump
  (`_refresh_pending_release`).
- F2: a refused re-send left `_carried_release_mm` (and, on a fold, `_carried_catch_xy_mm`) at the refused aim,
  because the record was written before the install and not restored on refusal (23 of 98 rows at the P2 Sitting
  had one). The record is now restored to the last accepted release (`_restore_release_record`). This also
  corrects a bump after a refused re-send, which added its offset to the refused aim.

**What does not change.** No command changes with F1: the Learner's command is cached per Catch, the fly-back
reads the release record (not the row), and the plan is installed before the refresh. The Learner reads Site xy,
release z, release velocity, Dwell, the command, the outcome, ball and release kind; release xy and the seat offset
are not inputs, and `rel_z` cannot move (the release z is the Site's Throw z). F2 changes the starting aim of the
next Catch of that ball by at most the refused re-aim; that Catch passes the same clamp and the same online
admissibility check, and the restored value is always a release that was installed. Every reader and writer runs
inside the executor's tick under the skill node's tick lock, so there is no new race. Neither fix shrinks the
two-ball arrival pull (BUG-79), which is almost all velocity.

**Old rows.** Rows written before 2026-11-06 hold the dispatch aim in `rel` and `seat` on a re-aimed Catch; tell
them apart by `t_abs_s`. `INVARIANTS.md` row I-LEARN-1 now says so. No production reader assumes `rel` equals the
Site; only `tools/probes/learner_replay*` read release xy, for analysis.

**Evidence.** Three new tests fail on the parent commit and pass with the fixes: the row carries the re-aimed
release (it read 0.000 against 0.015 before), the next Catch's prior after a refused re-send is unchanged (it read
-17.5 mm/s against 0), and the pending row follows a bump. The sim gate
(`sim/skills_gate.py --learn --pattern columns --seeds 0 1 2`) gave 75/75 caught and bit-identical rows before and
after; the sim makes no re-aims, so it shows only that nothing else changed and the unit tests carry the proof.
Not covered by a test: the hand-corrected re-send path (it calls the same helpers; on a vertical Throw the release
xy does not move there) and a fold where only the seat is refreshed.

## Verification

Full gate, run 2026-11-06 19:39 to 19:48 with the fixes in the tree (`./run_tests.sh --full`): **8797 passed, 52 skipped, 1 failed** in 468 s, plus the serial tail **10 passed**. The one failure was a document check, not code (`tests/sim/test_map_items_rated.py`: Item BUG-81 was closed without its `effort`, `owner` and `radius` ratings). It was rated, with no code edit after the gate, and the document tests were re-run: (2026-11-06 19:50, `python -m pytest tests/sim/test_map_items_rated.py tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_sitting_sections.py tests/sim/test_night_shift_gate.py -q`: 305 passed, 48 skipped; `python -m pytest tests/sim -k "mapkit or map_items" -q`: 311 passed, 48 skipped).

Scoped runs by the implementing agent, 2026-11-06 (`python -m pytest`, OPENBLAS and OMP threads 1): the 29 motion and ros test files that import `executor.py`, `-n 3 --dist loadfile -m "not serial"`: 1692 passed; `tests/sim/test_skills_gate.py tests/sim/test_skills_gate_beats.py tests/sim/test_skills_gate_online.py`: 72 passed.

One audit pass (2026-11-06): clean on behaviour. Noted and left: an installer that raises instead of refusing is not restored (the exception already propagates out of the tick; unchanged by this fix).
