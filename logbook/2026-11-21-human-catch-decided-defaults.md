---
title: "human_catch: owner-decided defaults (clear 0.3 s, fixed wait Site, D10 bar 35 mm) (IDEA-16)"
type: feature
date: 2026-11-21
status: complete
phase: "PWAY-11 tooling"
related_plan: "human-partner-juggling.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/human_catch.py
  - tools/probes/human_catch_replay.py
  - tests/motion/test_skills_human_catch_policy.py
  - tests/tracking/test_human_catch_replay.py
  - plans/active/human-partner-juggling.md
  - logbook/2026-11-21-human-catch-decided-defaults.md
  - logbook/INDEX.md
subsystem: [motion, tooling]
tags: [human_catch, defaults, owner-decision]
---

## What and why

Owner, 2026-11-21, after the replay sweep: "default to those numbers and we can tune more as we progress". `HumanCatchPolicy` defaults: `clear_s` 0.5 -> 0.3 (return 1.0 s unchanged); `wait_radius_mm` 150 -> 0, a FIXED wait Site (the mean of past Landings was inert in every sweep variant; pass a radius above 0 to turn it back on, `mean_n` and `wait_deadband_mm` are kept); plan window 0.62 s and first lead 0.55 s are now DECIDED; the replay probe's D10 bar 30 -> 35 mm xy (`D10_XY_MM`, the Cup radius `hand_radius_mm`), 20 ms unchanged. The ROS parameters read their defaults from the policy signature, so the node follows. D4, D5, D6, D9, the holding timeout and the pending-key rule stay PROPOSED.

Sweep evidence (clear 0.3 / return 1.0 vs 0.5 / 1.0): planned 21 vs 18, D10 good 11 vs 8 at 30 mm; D10 good of 23 in reach = 8 / 10 / 13 at 30 / 35 / 40 mm. Control implication: HOLDING ends 0.2 s sooner after a Catch, so the RETURNING REST (an ordinary rest-terminal Segment through the same planner and gate) starts 0.2 s sooner; the Catch move, its gate and every limit are untouched. A fixed Site makes every RETURNING REST go to the same point: the Site-change moves are removed, none added.

## Verification

- 2026-11-21, `pytest -q tests/motion/test_skills_human_catch*.py tests/ros/test_skill_node_human_catch*.py tests/tracking/test_human_catch_replay.py tests/motion/test_skills_nominate_human_catch.py tests/motion/test_skills_executor_human_catch.py`: **662 passed in 283.79 s** (the same files plus `tests/sim/test_logbook_front_matter.py`, `test_logbook_index.py`, `test_plans_index.py`, `test_docs_links.py`, `test_no_legacy_item_ids.py`, `tests/ros/test_skill_node.py`).
- 2026-11-21, replay of the cached 12:53:20-12:56:45 stream, hand frame, new defaults: planned 21, D10 good 13 of 23 in reach at 35 mm (matches the sweep's clear 0.3 / return 1.0 row, g35 = 13).
- Full gate: GATE_RESULT_PLACEHOLDER
