---
title: "bb-calibration-one-attempt archived: banner auto-hide, rows 44-46 flown per the owner, acceptance 2 closed on the evidence, row 36 waived"
type: refactor
date: 2026-10-19
status: resolved
phase: "two-ball-skill-stack - post-R6 (Ball Butler calibration, archival)"
related_plan: bb-calibration-one-attempt.md
files_changed:
  - plans/archived/bb-calibration-one-attempt.md
  - plans/active/INDEX.md
  - plans/archived/INDEX.md
  - plans/active/two-ball-oval.md
  - logbook/INDEX.md
  - logbook/2026-10-19-robot-frame-move-observability.md
  - tests/hardware/session_skill_stack_rehearsals.md
  - ros_ws/gui/js/robot-frame.js
---

## What changed

- `ros_ws/gui/js/robot-frame.js` (owner, after the pm sitting): the move banner auto-hides after `ROBOT_FRAME_BANNER_MS` = 3000 ms, `x` closes it sooner, and the first `mocap/status` after a GUI refresh only seeds the move count, so a `reload` does not replay an old move. The "sticky" wording in the plan, the rehearsal row 44 and the move-observability entry was reconciled. NOT flown.
- Rehearsal rows 44-46 flown 2026-10-19 pm, PASSED per the owner ("working nicely"), no bag analysed.
- `plans/active/bb-calibration-one-attempt.md` archived by `git mv` (no rename) with `status: completed`, an Archival note, § 6 residue and a final § 5. Owner closures recorded: acceptance 2 (ten consecutive sittings) closed on the evidence, the tally not formally kept past 4 of 10; rehearsal row 36 waived, the path pinned by `tests/ros/test_mocap_node_robot_frame.py`, `tests/ros/test_skill_node_robot_frame.py`, `tests/motion/test_skills_executor.py`.
- 22 path-qualified references to the old plan path across 17 files re-pointed (logbook frontmatter and prose, three test docstrings, three probe docstrings, the readings ledger, a hardware runbook); 0 remain. Test edits are docstring-only.
- Seven calibration entries set to `resolved` (frontmatter and `logbook/INDEX.md` rows): phase0-characterise, phase1-gate-inversion, phase2-template-residual, power-cycle-ab, phase3-robot-frame, fit-yaw-only-hold, move-observability. The 2026-10-14 investigation entry is left as a record.

## Why

The plan's phases are all landed and flown, and the owner closed the two remaining gates; leaving it on the active board would advertise unschedulable work. No code behaviour changed in this unit.

## Verification

- Full gate 2026-10-19 15:50 AEDT, `./run_tests.sh --full` (log `temp/logs/gate_archive.log`): 6969 passed, 9 skipped in 314.59 s + serial 7 passed, exit 0. GUI: `node --check ros_ws/gui/js/robot-frame.js` 2026-10-19, OK (not browser-tested).
- Scoped: 2026-10-19, `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/ros/test_mocap_node_yaw_gate.py tests/ros/test_mocap_node_attempt_fields.py tests/ros/test_mocap_node_base_frame.py tests/ros/test_bb_calibration_replay.py -q -n 4` -> 168 passed in 18.14 s
