---
title: "Bump re-derives the C-HAND-5 brake seed; install line shows the segment's own Hand peak; gap trackers reset on re-arm; MISSED-BALL evidence line"
type: bugfix
date: 2026-11-02
status: resolved
phase: "pattern progression"
related_plan: pattern-progression.md
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/motion/test_skills_bump.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_trajectory_node_console.py
  - tests/ros/test_teensy_bridge_node_bridge_diag.py
  - logbook/2026-11-02-bump-brake-seed-and-debrief-observability.md
  - logbook/INDEX.md
subsystem: [motion, ros]
tags: [safety, performance, dynamics, testing]
---

# Bump re-derives the C-HAND-5 brake seed, plus three debrief observability fixes

Four non-threshold fixes found by the 2026-11-02 Sitting debrief (`2026-11-02-sitting-debrief.md`, which holds the evidence and the reasoning). Related item: BUG-42. No limit, margin or guard value changed.

## Summary

- **(a) Bump brake seed (`bump.py`, `_bumped_brake_seed`).** A Bump moved the release mark's site and velocity but left the C-HAND-5 brake seed (`ReleaseMark.brake_accel_mm_s2`) at its pre-bump direction, so `a_R - g` no longer lay along the bumped take-off velocity. The Bump now re-derives the seed through `release_brake_accel_mm_s2`. It declines BUMP-NO-CARRY when there is no cup track to derive from.
  - Offline before/after (committed probe `tools/probes/bump_seed_replay.py`, Hand limit 4500): 4496 -> 3625, 4664 -> 3380, 4513 -> 3601 rev/s^2. Debrief replay of the Sitting's refused Catch at a 30.5 mm Bump: 4696 (refused) -> 3407.
  - Accepted tradeoff: the tilt is unmoved by a Bump, so up to about 0.75 m/s^2 of the brake sits across the cup axis, about 1 deg off the detach axis. LIMIT_VEL is not reached by the new seed (+-0.6 %).
- **(b) Install line (`trajectory_node.py`).** The Hand acc peak is now this segment's own (closed form per Hermite span from the seam); the old running maximum stays on the line, labelled `buffer max`. The 2026-11-01 suffix printed `unified_cycle._merged_peaks`, a running max over the plan buffer, which made a debrief unit read 61 % of Catches as above 90 % of the limit. Leg vel/acc remain buffer maxima: a per-segment leg figure needs a range `validate_cycle` does not expose (about 89 % of a solve).
- **(c) Gap trackers (`teensy_bridge_node.py`, `_start_setpoint_output`).** The forward and loop gap trackers and the forward latch are reset on re-arm. Before, the first record after a re-arm measured the whole disarmed interval as a gap and raised two false "starved" WARNs (17:00 Sitting launch.log line 4361; 18:13 line 4979).
- **(d) MISSED-BALL line (`executor.py`, `lost_ball_line`).** END MISSED_CATCH now prints a MISSED-BALL line: ball, Throw, scheduled release, and the Tracker's last fit or NO FIT. Four of 16 misses in the Sittings had no evidence at all.

## Verification

- Scoped run (2026-11-02): `python -m pytest tests/motion/test_skills_executor.py tests/motion/test_skills_report.py tests/ros/test_skill_node.py tests/ros/test_trajectory_node_console.py tests/ros/test_teensy_bridge_node_bridge_diag.py tests/ros/test_teensy_bridge_node_setpoint.py tests/teensy_link/test_gap_monitor.py -q` -> 752 passed.
- (2026-11-02) `pytest -q tests/motion/test_skills_bump.py tests/motion/test_release_step.py` -> 35 passed.
- Gate: `./run_tests.sh` (2026-11-02, in the debrief worktree on origin/main 4761e68e): 8004 passed, 54 skipped, 1 failed + 5 serial passed; the one failure was `tests/sim/test_glossary_avoid_words.py::test_no_avoid_words_in_live_docs` on an Avoid-word in the new BUG-58 body, fixed (doc-only) and re-run with the named doc tests below. Named doc tests after the fix (2026-11-02, `python -m pytest -q tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_plans_index.py tests/sim/test_mapkit*.py`): 186 passed in 49.37 s

## Outcome

Fix (a) removes the step-specific excess on bumped successors (the three HAND_LIMIT_ACC ends of the Sittings), not the pre-C-HAND-5 bump-seed effect; the structural backstop (Bump decision also plans Catch N+1 and shrinks or declines) is still open under BUG-42. The next Sitting should re-check the bumped-successor peaks with the segment-own figure from (b).
