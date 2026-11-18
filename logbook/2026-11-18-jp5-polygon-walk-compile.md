---
title: "JP-5 polygon walk built on the one-ball compile path; the plan's 250 mm edge is infeasible for a square and the BeatEvent stream cannot carry a one-ball walk"
type: feature
date: 2026-11-18
status: complete
phase: "pattern-progression P7 (offline half)"
related_plan: "pattern-progression.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/records.py
  - sim/skills_gate.py
  - tests/motion/test_skills_polygon_walk.py
  - tests/ros/test_skill_node_polygon.py
  - docs/records.md
  - plans/active/pattern-progression.md
subsystem: [skill-stack, learner]
tags: [pattern, polygon-walk, jp-5]
---

# JP-5 polygon walk: the offline half

## What

A new one-Ball Pattern, machine label `polygon_walk` (provisional until the owner confirms it): one Ball fed by Ball Butler and thrown Site to Site around a regular polygon of N Sites (`polygon_n`, default 4) with the edge given by `separation_mm`. It is wired into `/skills/nominate`, `skill_node`, its own row in `records.PATTERNS` and `sim/skills_gate.py`. The GUI Juggle panel is not wired.

## Discussion

- **Withdrawn: "on BeatEvent".** The plan (P7) and IDEA-9 put the walk on the `BeatEvent` stream. That stream is two-ball by construction (`_validate_stream` requires `ball_id == i % 2`, the opening needs an in-flight Ball, the Beat is half the cycle). A one-Ball walk has the full cycle as its period, which only the one-ball compiler builds. So the walk rides `OneBallPattern(roles='polygon')`, an additive case; Hop, self-throw and oval labels are unchanged and pinned.
- **Consequence accepted:** JP-5 no longer exercises the machinery JP-11 and JP-14 need. It still tests Site-by-Site admissibility and the Learner's first visit to a Site with no two-ball risk; JP-11 becomes the first real test of per-Beat Sites.
- **Withdrawn: "a square whose edge the Hop already flies" (250 mm).** A Hop reverses direction at every Throw; a square turns 90 degrees. At Apex 0.9 the nomination check gives: 250 mm at Dwell 0.27 infeasible; 200 mm refused (jerk margin -0.651); 150 mm refused (-0.229); 125 mm refused (-0.020); 100 mm accepted (+0.189); 125 mm at Dwell 0.45 accepted (+0.200); 150 mm at Dwell 0.45 accepted (+0.101). A hexagon at 100 mm passes at only +0.013. The P7 gate wording is corrected in the plan.
- **Memory on a first visit** (run on the 5264-row Memory): a never-visited Site has almost no support, and the command is the fitted plant model's inverse, not the identity: Apex command lowered by 69.5 mm, aim 4 to 10.6 mm off the commanded landing, inside the 20 mm clamp. With an empty Memory the command is exactly the commanded landing (pinned by a test).

## Verification

- (2026-11-18, the agent's 14-file scoped pytest run, command in the commit body of the feature commit): 967 passed, 2 failed; both failures were `tests/motion/test_records.py` on the committed ledger lacking the new empty table, fixed here by re-rendering `docs/records.md`.
- (2026-11-18, `python sim/skills_gate.py --learn --pattern polygon_walk --seeds 0 1`): PASS, 25 Catches and 0 drops per seed.
- The gate run on the integrated tree is cited in the commit that files this entry.
- Not flown. First flight: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'polygon_walk,apex_m=0.9,separation_mm=100,dwell_s=0.27,polygon_n=4'}"`, then the Juggle goal from the terminal.
