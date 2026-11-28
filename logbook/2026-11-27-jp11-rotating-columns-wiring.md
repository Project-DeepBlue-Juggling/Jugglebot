---
title: "JP-11 rotating Columns wired (U2): nominate step_deg and the full-turn span rule, node param columns_step_deg and its refusals by name, records row, skills_gate --step-deg; the full-turn nomination takes 6 min 48 s and the sim gate flies 38/38 Catches but fails its leg mirror band 1.20e-4 vs 1.0e-4 rev"
type: feature
date: 2026-11-27
status: in-progress
phase: "JP-11 rotating Columns U2"
related_plan: "pattern-progression.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/juggle_request.py
  - ros_ws/src/jugglebot/jugglebot/records.py
  - sim/skills_gate.py
  - tests/motion/test_skills_columns_rotating_wiring.py
  - tests/ros/test_skill_node_columns_rotating.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_nominate.py
  - docs/records.md
subsystem: [skill-stack]
tags: [pattern, rotating-columns, jp-11]
---

# JP-11 rotating Columns: the wiring (U2)

## What

U1 compiled the Pattern; U2 wires it the way `columns_turned` and `centre_orbit` are wired. No executor change, no schedule.py or sites.py change, nothing in the GUI.

- `nominate`: `NominationConfig.step_deg` (last field), `rotating_span(step)` = ceil(360/step)+2 (38 at 10 deg, 74 at 5, 20 at 20), `columns_rotating` in `NOMINATED_PATTERNS` and `COLUMNS_FED`, `build_pattern` branch: Columns' fed pair, Feed Site always P1 (`columns_feed_site` is not read), `step_deg` and `apex_m_ball_b` handed to the Pattern so it raises its own codes. A nomination plans min(operator count, span) Throws (the short pattern is one full turn when no count is given); the Attempt's own build (`n_throws` passed) is never capped, so the nomination's Pattern is a prefix of the Attempt's. Step 0 on this pattern is refused by name, never read as plain Columns.
- `skill_node`: `_PATTERNS`, cup rule (Columns' own row), param `columns_step_deg` (default 0.0; out of range refused at once, changed mid-goal refused), `_rotating_refusal` (pose planner only; `columns_angle_deg` non-zero -> `ROTATING_WITH_ANGLE`; unequal Apex and a bad step surface the Pattern's own `ROTATING_*` code), `_run_columns(step_deg=)`, `_nomination_config(step_deg=, n_throws=)`, `skills/nominate` key `step_deg=`, records param `step_deg`, goal-start line and the `juggle_start` event print the step.
- `juggle_request`: `step_deg` is a nominate-only key (the goal relay refuses it; a goal reads the node parameter). `records.PATTERNS`: `columns_rotating`. `sim/skills_gate.py`: `--step-deg` (plain gate flown from the hop entry with the feed landing prior, as the node compiles; `--nominate --pattern columns_rotating`).

## Verification

- Existing Schedules unmoved (2026-11-27, the U1 dump scripts `dump_sched.py` / `dump_orbit.py`): 337 configs sha256 f09c9519..., 60 configs 602ba28f..., both as before U2.
- Nomination wall time, this box, `nice -n 10`, loaded (2026-11-27, `skills_gate.py --nominate --pattern columns_rotating --step-deg 10 --n-throws 38 --no-search`): 6 min 47.9 s (1100 s user, 270 % CPU), viable, worst margin 0.327 (jerk, CATCH Ball 1 at R38@10, rung -20 %). Step 5 / 74 Throws not run (about twice that). The goal accept nominates the same config in the node and would block as long, unless a `skills/nominate` with the same fields and count ran first (the in-process cache is keyed on the whole config).
- Sim gate (2026-11-27, `nice -n 10 python sim/skills_gate.py --step-deg 10 --separation-mm 125 --apex-m 0.95 --n-throws 38 --seeds 0 1 2 3 4`): every seed 38/38 Catches, 0 drops, 40/40 installs (41/41 seed 2), pump clean, flags clean, final cup error 0.05 mm, **FAIL on the leg mirror band only**: 1.20e-04 rev against `MIRROR_TOL_LEG_REV` 1.0e-4 (hand 4.8e-7 passes). Plain Columns at the same Apex, separation and count passes at 1.09e-5. The worst residual is on the opening hop plan's pre-t0 hold (tau -0.2 s; every later plan is at or under 1.07e-5) and is nearly independent of the step (probe, n 6: step 20 1.152e-4, step 5 1.207e-4; step 10 1.204e-4), so it is a property of the hop-entry opening, not of the rotation; cause not found. No limit was loosened.
- `docs/records.md` regenerated with `records.render(records.parse(text))` (a new empty `Columns — rotating` section; `test_records.py` pins the file as a fixed point of render).
- Scoped tests (2026-11-27, `pytest tests/motion/test_skills_columns_rotating.py tests/motion/test_skills_columns_rotating_wiring.py tests/motion/test_records.py tests/ros/test_skill_node_centre_orbit.py -q`): 139 passed. Node subset (`pytest tests/ros/test_skill_node.py tests/ros/test_skill_node_nominate.py -q -n 2 -k "opens_on_a_rest or turned or angle or every or cup_cell or pattern or param or records"`): 157 passed, 2 failed in 18 min 37 s on the loaded box; the 2 (`test_the_node_and_the_nomination_build_the_identical_pattern[columns_rotating-*]`) were this change's own gap (no step parameter set; the run path's config lacked the count), both fixed, re-run `-k "identical"` plus `test_skill_node_columns_rotating.py` and `-k rotating` over `test_skill_node.py`: 21 and 11 passed. NOT run to completion: the whole `tests/ros/test_skill_node.py`, `tests/motion/test_skills_nominate.py` and `tests/motion/test_skills_centre_orbit.py` (a -n 2 run of the full scoped list reached 36 of 288 tests in 22 minutes under load 12 and was stopped); the orchestrator's gate covers them.
- New tests fail without the change: with `rotating_span`'s `+ 2` mutated to `+ 1`, `test_skills_columns_rotating_wiring.py` fails 7 of 16; with `_nomination_config`'s `step_deg` forced to 0, `test_the_node_and_the_nomination_build_the_identical_pattern` fails (both restored).

## Operator commands (Sitting section)

```
ros2 param set /skill_node planner_mode pose
ros2 param set /skill_node columns_angle_deg 0.0
ros2 param set /skill_node columns_step_deg 10.0
ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns_rotating,apex_m=0.95,separation_mm=125,dwell_s=0.27,num_cycles=38,step_deg=10'}"
ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns_rotating, apex_m: 0.95, separation_mm: 125.0, dwell_s: 0.27, num_cycles: 38}" --feedback
```

Run the nominate call first with the SAME fields and count: it takes minutes and fills the in-process cache, so the goal accept finds it instantly; without it the accept blocks for the whole nomination.
