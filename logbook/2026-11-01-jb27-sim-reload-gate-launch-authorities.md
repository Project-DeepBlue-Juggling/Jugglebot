---
title: "JB-27: the sim --reload gate runs the launch's lateral authorities (20 mm learner, 10 / 35 mm catch); 2 makes per seed again, and the LIMIT_JERK ends no longer reproduce on main"
type: bugfix
date: 2026-11-01
status: complete
phase: "night-shift follow-up"
related_plan:
  - columns-empty-cup-two-feed.md
related_entries:
  - 2026-10-19-held-axis-reload-catch-refuses-tracker-reaim.md
files_changed:
  - sim/skills_gate.py
  - tests/sim/test_skills_gate.py
  - map/bug-sim-reload-gate-limit-jerk-regression.md
  - logbook/INDEX.md
---

# JB-27: the sim `--reload` gate runs the launch's lateral authorities

**Question.** JB-27: the sim `--reload` gate (`sim/skills_gate.py`) made 5/5 at 8364808c and,
after 08ec3fe6 (2026-09-30), 1 make with `LIMIT_JERK` in the continuation throws, while the robot
flew R5 cleanly. The owner chose JB-128 option 3: treat it as a gate-configuration gap first,
and only bisect the planner if the gate stays red.

**What was found.** On current `main` the symptom as filed does not reproduce:
`python sim/skills_gate.py --reload --seeds 0 1 2 3 4` passes 5/5 with no `LIMIT_JERK`, but makes
1 per seed where 8364808c made 2. Which commit removed the `LIMIT_JERK` ends was not bisected. The
shortfall has a configuration cause: `jugglebot_launch.py` passes none of skill_node's lateral
authorities, so the robot runs the declared defaults (`learner_lateral_authority_mm` 20,
`catch_outward_authority_mm` 10, `catch_inward_authority_mm` 35), while `run_reload_attempt`
built its `SkillExecutor` with all three unset. The second catch's 41-45 mm tracker re-aim then
went through unclamped where the robot clips it to 20 mm (the gap the 2026-10-19 held-axis entry
first named).

**Fix.** `sim/skills_gate.py` gains `LAUNCH_LATERAL_AUTHORITY_MM` and passes it to the `--reload`
executor. They are literals because skill_node imports rclpy; the new
`test_gate_lateral_authorities_match_the_launch` parses skill_node.py's `declare_parameter`
defaults with `ast`, fails if the table drifts, and fails if the launch starts passing these
parameters (then the launch, not the declaration, is the source of truth). The learner and the
other scenarios build their own executors and are unchanged.

**Result.** Same command, 2026-11-01: before PASS 5/5, makes 1,1,1,1,1; after PASS 5/5, makes
2,2,2,2,2, drops 0, installs 7/7 and 8/8. Option 2 (the planner bisect) is not needed: the
gate is green and matches the launch.

**Verification.** 2026-11-01: `./run_tests.sh --full` -> 7898 passed, 52 skipped in 409.65 s; serial phase 8 passed; exit 0. The scenario before/after above is the oracle; `test_gate_lateral_authorities_match_the_launch` fails if the gate's table or the launch drifts.
