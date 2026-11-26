---
title: "FEAT-56 pins: one test file ties the hand-copied gravity, capture bound, flag bits, RELEASE_KINDS, Kt and REST budget copies together (tests only)"
type: feature
date: 2026-11-26
status: complete
phase: "FEAT-56"
related_plan: ""
related_issues: []
files_changed:
  - tests/firmware/test_hand_copied_constants.py
  - logbook/2026-11-26-feat56-copied-constant-pins.md
subsystem: [tests, config]
---

## Summary

`tests/firmware/test_hand_copied_constants.py` (20 tests, no production change) reads each copy from source text (AST, or a regex on the firmware header) and fails naming both files and both values. Families: gravity (5 production copies and `cup_cycle.GRAVITY` against the generated config), the capture bound `CAPTURE_MAX_OFFSET_M` against hypot of the `skill_node` authority defaults, the node's `_T2J_*`, `_T2J_BB_*`, J2T and `_DIAG_FLAG_*` bits (FW 53 LEG_HELD 0x10 and RIG_BENCH 0x20 included) against the generated enum, the protocol comments and the firmware headers, the REST return acceleration limit against its own budget plus the restated resume tolerances, `RELEASE_KINDS` across three modules, Kt against the generated config.

Pinned as found, not as correct (owner has not decided): `sim/skills_gate.py` 9810, `motion/skills/human_catch.py` 9810.0 (a copy the FEAT-56 table omits), probe literals 9810 and 9806.65. Already pinned elsewhere and cited in the file docstring: REBASE_TOL and the REST half-relation (`test_sched_c2_twin.py`), LEG_HELD/RIG_BENCH vs `leg_stow_gate.h` (`test_stow_gate_xref.py`). `tools/trapped_ball_finder.py` no longer holds 578.2 or 32.567 (it imports them via `tracking/trapped_ball.py`, which reads the config).

## Verification

- 2026-11-26, `pytest tests/firmware/test_hand_copied_constants.py -q`: 20 passed in 5.2 s. No copies disagree today.
- 2026-11-26, one mutation per family in the worktree, each restored: ballistics 9806 to 9810, cup_cycle hypot 0.035 to 0.040, node `_DIAG_FLAG_LANE_STALL` 0x8 to 0x10, `_T2J_BB_FLAG_HEARTBEAT_STALE` 0x4 to 0x8, memory.py RELEASE_KINDS extra entry, `KT_MEASURED` 0.0570 to 0.0624, REST acc limit 5.0 to 9.0, skills_gate 9810 to 9806, telemetry.cpp stale literal 0x1 to 0x40: each failed the intended test.
