---
title: "Oval box cells at dwell 0.55 and 0.50 swept and merged into the committed admissible box"
type: feature
date: 2026-10-23
status: resolved
phase: "two-ball-oval - rung 2 prerequisite"
related_plan: two-ball-oval.md
files_changed:
  - config/generated/admissible_box.yaml
  - tests/motion/test_skills_admissible.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_gui_juggle_defaults.py
  - plans/active/two-ball-oval.md
  - tests/hardware/session_oval_rung1.md
  - logbook/INDEX.md
subsystem:
  - skills
tags:
  - admissible-box
  - oval
---

## Summary

Rung 2 flies the one-ball oval at dwell 0.55 then 0.50, and `select()` refuses a dwell the box was not swept at, so the two cells had to exist first. No planner or gated-file change: the swept stamp (swept_at 2026-10-23, gate_hash d6e60d5556b9, limits 350/5000/200000/3900) is unchanged. The GUI default stays 0.65.

## Sweep and merge

- Sweep (2026-10-23, `python tools/admissible_sweep.py --leg-vel 350 --leg-acc 5000 --leg-jerk 200000 --hand-acc 3900 --dwell-s <0.55|0.50> --site-pairs oval --oval-separation-mm 150 --oval-apex 0.95 --single-apex-halfwidth 0.025 --out temp/sweeps/box_oval<055|050>_<a|b>.yaml`): 225 s wall each, a/b `cmp` IDENTICAL at both dwells; one `oval_1ball` cell each, apex band [0.925, 0.975], landing +-20 mm.
- Merge (2026-10-23, `admissible.load` x3 then `admissible.dump`): result diffs against the committed box as append-only (0 removed lines, 44 added), header unchanged, 22 boxes with oval dwells [0.65, 0.55, 0.50], re-`load` passes the overlap/provenance checks, `gate_hash()` == d6e60d5556b9.

## Verification

- Tests (2026-10-23, `pytest tests/motion/test_skills_admissible.py tests/ros/test_skill_node.py tests/ros/test_gui_juggle_defaults.py tests/ros/test_skills_plan_bench.py tests/sim/test_skills_gate.py -q`): 551 passed in 187 s. Updated: oval cells 1 -> 3 and 20 -> 22 boxes; oval dwells subset of {0.65, 0.55, 0.50}; GUI oval dwell must be in the swept set.
- Sim learner gate (2026-10-23, `python sim/skills_gate.py --learn --pattern oval_1ball --separation-mm 150 --apex-m 0.95 --dwell-s <d> --seeds 0`): PASS at 0.65 (50 s), 0.55 (48 s), 0.50 (47 s); 25 makes, 3 attempts, 0 drops each. (A first 6-throw run FAILed only because the monotone window was too short; not a defect.)
- Not done here: the launch-side `skills/check` step (runbook R2-4) and a launch restart so `skill_node` loads the new box are the operator's.
