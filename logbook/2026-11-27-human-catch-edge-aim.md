---
title: "human_catch edge aim: a Fit Landing up to 35 mm outside the settle box is caught at the box edge instead of refused SETTLE_SITE (PWAY-11 P3; 7 to 9 of 14 in-reach lobs inside 35 mm on the 2026-11-27 bag)"
type: feature
date: 2026-11-27
status: complete
phase: "PWAY-11 P3"
related_plan: "human-partner-juggling.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/human_catch.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tools/probes/human_catch_replay.py
  - tests/motion/test_skills_human_catch_policy.py
  - tests/ros/test_skill_node_human_catch.py
  - tests/tracking/test_human_catch_replay.py
  - logbook/2026-11-27-human-catch-edge-aim.md
subsystem: [skills, motion, tooling]
tags: [human_catch, PWAY-11, settle box, edge aim]
---

## What and why

Owner decision 2026-11-27 (plan `human-partner-juggling.md` step P3): in the `human_catch` Pattern, when the Fit's Landing at the plan instant is outside the ±150 mm settle box (`CupCycleConfig.workspace_xy_m`, gate `cup_cycle._gate_settle_site`) by at most 35 mm, aim the Catch at the nearest point of the box edge instead of refusing; farther out, refuse as before. On the 2026-11-27 lob bag three in-reach lobs were refused `SETTLE_SITE` with the Fit 1 to 11 mm outside while the Ball crossed inside it.

`HumanCatchPolicy` gains `edge_aim_mm` (DECIDED 35 mm, 0 turns it off; node parameter `human_catch_edge_aim_mm`) and `edge_aim(evt)`, the one place the rule lives: the distance is the Euclidean distance from the Fit's xy to the box (a corner counts both axes), the aim is `np.clip(xy, -150, 150)`, and z, arrival velocity and landing time stay the Fit's. A PLAN Decision carries the aimed event, the Fit (`fit_mm`), `outside_mm` and the line `HUMAN-CATCH-EDGE-AIM id .. seq .. rev ..: Fit Landing (x, y) mm is d mm outside the +-150 mm box; the Catch aims at the box edge (x, y) mm`. A re-aim's target goes through the same rule (`reaim_target`, called by `skill_node._on_human_throw` where it stores `ctx.latest`; `HUMAN-CATCH-EDGE-REAIM` logged once per Throw). The source of 35 mm is the Cup's capture radius, `hand_radius_mm` = `GEOM_HAND_RADIUS_MM` 35.0 (the same number as the replay's D10 35 mm bar; they agree). `workspace_xy_m`, the gate and every limit are untouched.

**Why the policy and not `compile_human_catch`.** (1) A re-aim never passes through the compile (the executor re-solves from `ctx.latest` through `_human_tracker`), so a compile-side clip would need a second copy in the re-aim path, and the two could drift. (2) The replay drives the real policy, so it scores the rule with no parallel copy. (3) The compile keeps its contract that the CATCH's Site is the Landing it is handed, bit for bit; the clip is an owner decision about where to aim, which is what the policy holds. (4) Nothing outside `human_catch` builds a `HumanCatchPolicy`, so no other Pattern can see the rule.

## One control cycle with the change

1. **Event.** `/human_throws` arrives; `_on_human_throw` puts the Landing in the schedule frame. Association (400 mm from W) and the running mean read the raw Fit, as before. A revision 0 with lead above 0.62 s is held (PENDING); later revisions replace it.
2. **Policy decides.** At the 40 Hz tick where the lead first drops to 0.62 s, `poll` claims the latest revision; `_claim` calls `edge_aim`. Inside the box: the Fit itself (unchanged). Outside by d ≤ 35 mm: the aim moves to the nearest edge point, at most 35 mm from the Fit. Farther out: the Fit itself.
3. **Plan solved and gated.** `_hc_plan` compiles the aimed event (Site = aim, landing prior = aim). The executor installs on the next tick through `install_segment`: `_gate_settle_site` passes at exactly 150.0 mm (`>`, probe below), then the solve and the command-time admissibility check at 100 % (validate at install), all unchanged. A Fit more than 35 mm out still reaches the gate unchanged and is refused `SETTLE_SITE` (a continue code; nothing moves). An edge aim the limits cannot reach is refused by its limit like any in-box target (probe: the (150, 150) corner at lead 0.55 is `LIMIT_VEL`).
4. **What the Platform does.** It flies one level CATCH from the REST at W to touch-down at the edge point at the Fit's time and rests there. Every knot stays inside the box (the QP's hard box rows, unchanged), so the move is one the planner already flew for any in-box Fit at the edge; no new motion class. Largest displacement from W (-50, 0): 200 mm in +x, the same as before for an in-box Fit at x = 150.
5. **Next event / re-aim.** A later revision is stored through `reaim_target`: up to 35 mm out it becomes the edge point (the executor then sees a target inside the box and, if it moved less than 10 mm, requests nothing); farther out it is passed unchanged, the executor's ±50 mm clamp about the commanded landing can leave it outside, the install refuses it, re-aiming ends for that Catch and the committed Segment flies on, exactly as for any Catch today. A revision back inside the box re-aims there as before.
6. **The Ball really lands outside the box.** The Cup waits at the edge point; the miss is the distance from the crossing to that point (inside the 35 mm Cup radius it is a Catch, beyond it a MISSED_CATCH and the Attempt continues). The worst case is not bounded by this change (it is the Fit error plus up to 35 mm), but the Cup is never commanded beyond the existing ±150 mm box: the aim is always the Fit or a point inside the box, and the gate and box rows are unchanged.

## Verification

- (2026-11-27, `python tools/probes/human_catch_reach.py`-context probe `check_reach` at W (-50, 0), scratch script) the edge is plannable: (150.0, 0) lead 0.62 OK margin 0.233 (lead 0.55 MARGIN -0.063); (150.0001, 0) `SETTLE_SITE` at both leads; (-50, ±150), (-150, 0) OK at 0.55 and 0.62; corners (150, 150) OK at 0.62, (150, -150) MARGIN at 0.62.
- (2026-11-27, `python -m pytest tests/motion/test_skills_human_catch_policy.py tests/ros/test_skill_node_human_catch.py -q -p no:cacheprovider`) 118 passed. With `human_catch.py` and `skill_node.py` reverted to `origin/main`: 15 failed (every new edge-aim test plus the parameter-list test), 103 passed. `test_the_clip_lives_in_the_policy_only_compile_and_gate_are_unchanged` and `test_a_fit_farther_out_is_handed_on_unchanged_for_the_gate_to_refuse` pin the absence of change and pass either way; the first fails against a mutant that clips inside `compile_human_catch` (1 failed).
- (2026-11-27, `python -m pytest tests/tracking/test_human_catch_replay.py -q -p no:cacheprovider`) 12 passed; with the replay reverted to `origin/main` 1 failed (the new edge-aim test).
- (2026-11-27, `flock /tmp/jugglebot-tests.lock python -m pytest tests/motion/test_skills_human_catch_policy.py tests/motion/test_skills_human_catch.py tests/motion/test_skills_executor_human_catch.py tests/motion/test_skills_nominate_human_catch.py tests/motion/test_skills_human_catch_bounded.py tests/tracking/test_human_catch_replay.py tests/ros/test_skill_node_human_catch.py tests/ros/test_skill_node_human_catch_book.py tests/ros/test_skill_node_nominate.py tests/ros/test_skill_node_online.py tests/ros/test_skill_node.py tests/firmware/test_hand_copied_constants.py -q -p no:cacheprovider -n 2`, the readers found by `grep -rln "human_catch\|HumanCatchPolicy" tests tools`) **689 passed in 1052 s** (loaded box, load average 10.7). The default gate was not run (the orchestrator runs it at integration).
- Replay, 2026-11-27 lob bag, Robot frame, 13:50:30 to 13:54:30 (`python -u tools/probes/human_catch_replay.py ~/Desktop/rosbags/2026-11-27_13-49-34 --t-start 13:50:30 --t-end 13:54:30 --frame robot --transport-ms {0,44}`, before on `origin/main`, after on this branch, same cached stream), all rc 0:

| transport | in reach, inside 35 mm / 20 ms (before → after) | planned | Cup to crossing p50 / p90 | `SETTLE_SITE` refusals | edge aims |
|---|---|---|---|---|---|
| 0 ms | 7 → 9 of 14 | 16 → 18 | 24 / 48 → 24 / 46 mm | 12 → 8 | 2 |
| 44 ms | 6 → 9 of 14 | 15 → 18 | 31 / 52 → 28 / 49 mm | 14 → 9 | 4 |

Edge-aimed Catches that scored (Cup at the edge to the Ball's crossing): 0 ms k1 1.5 mm out, 28.7 mm; k31 8.0 mm out, 15.7 mm. 44 ms: k2 21.3 mm out, 19.6 mm; k16 4.4 mm out, 24.0 mm; k31 17.0 mm out, 13.9 mm. At 0 ms the third refused lob (k2) is not edge-aimed but BUSY: catching k1 holds the Cup, so the next lob arrives during HOLDING (k32, `SETTLE_SITE` before, is BUSY after in both runs). At 44 ms the fourth edge aim (k1, near the (150, 150) corner) is refused `LIMIT_JERK` by the gate. "Inside 35 mm" is the replay's model, not a measured Catch.

Nothing changes on the robot until the live checkout is rebuilt (`colcon build`); after that a `human_catch` Attempt aims a Fit up to 35 mm outside the box at the box edge.
