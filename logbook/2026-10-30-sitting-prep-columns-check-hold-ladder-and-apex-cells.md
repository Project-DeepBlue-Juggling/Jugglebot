---
title: "Sitting prep: columns-under-pose check, the pre-release-hold ladder (hold 0.100 -> 0 predicted to need delay-aware composition), and oval cells at apex 1.00/1.05"
type: investigation
date: 2026-10-30
status: in-progress
phase: "two-ball-oval P4.0 - pre-release hold rung, Sitting prep"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-30-two-ball-oval-p3-regate-go-and-dwell-headroom.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
  - tests/motion/test_hold_rung_bias_probe.py
  - tests/ros/test_trajectory_node.py
  - config/generated/admissible_box.yaml
  - tests/motion/test_skills_admissible.py
  - tools/probes/p4_headroom_cells.py
  - tools/probes/hold_rung_bias.py
  - tools/probes/README.md
  - .gitignore
  - logbook/INDEX.md
  - temp/learn/r4-20260928/memory.csv
  - temp/learn/r4-20260929/memory.csv
  - temp/learn/r5-20260930/memory.csv
  - sim/skills_gate.py
  - tests/sim/test_skills_gate.py
  - tests/hardware/session_oval_rung1.md
  - plans/active/two-ball-oval.md
  - map/feature-pre-release-hold-rung-release-from-a-moving-platform-hold-0.md
---

## Summary

Prep for one hardware Sitting (runbook `tests/hardware/session_oval_rung1.md` § 11) that checks columns under the `pose` default, runs the pre-release-hold ladder (JB-102) with a pre-registered mean-bias criterion, and flies the oval at apex 1.00 / 1.05. Offline analysis found the 10-17 "0.30 at apex 1.0" number only reproduces at hold 0, and that the hold protects against plant tracking delay, not an un-composed velocity. Hold 0 is therefore gated on a measurement at hold 0.050.

## What landed (uncommitted, after ef6ff436)

- **A.** `trajectory_node.py`: `pre_release_hold_s` (live param, [0, 0.25]) is REFUSED while a solve is in flight or a cycle plan is streaming ("pre_release_hold_s cannot change while a goal is active (cycle plan still streaming); set it between goals"); a same-value set is a no-op; each accepted set logs one INFO `segment config: pre_release_hold_s=... (live set; applies from the next install)`. A goal never mixes holds.
- **F.** `.gitignore` whitelist: the three older per-Sitting memories `temp/learn/r4-20260928`, `r4-20260929`, `r5-20260930` (185 rows, 48 KB) became visible and are committed with this entry as history.
- **B.** Admissible box: 8 oval cells at apex 1.00 / 1.05 (bands 0.975-1.025, 1.025-1.075), dwell 0.50 / 0.45, both planners; 54 cells = 27 level + 27 pose; gate_hash unchanged `81a2244c1297`; every new cell non-empty (landing windows +-20 mm x/y; at dwell 0.45 the x upper edge is +10 mm, pose at 1.00 and both planners at 1.05). Pin test in `tests/motion/test_skills_admissible.py`.
- **C.** `tools/probes/p4_headroom_cells.py` promoted from the scratch probe; `tools/probes/hold_rung_bias.py` (new) reads the launch log + the single `memory.csv` residual (y - u), attributes goals and holds itself, and gives the C6 verdict and the NEXT action.
- **D.** `sim/skills_gate.py` gains `--pre-release-hold-s` (`--learn` only, range [0, 0.25], printed on the header line); `tests/sim/test_skills_gate.py` extended.
- **E. One learner memory; attribution by the probe (revised 2026-10-30, owner: "can't you figure out what was run from the recording?").** No per-rung `plant_id`: the bias read is the plant residual y - u, which does not move when the learner adapts, so one shared memory (`jugglebot`) is enough; the only cost of changing the hold back is a ~16-throw learner transient that affects catches, not the read. `hold_rung_bias.py` attributes throws to goals and holds from the launch log (goal start/end lines, `segment config: pre_release_hold_s=` lines) and the memory's `t_abs_s`, and prints the next action. The goal START line logs the pattern, throw count and (oval) apex but NOT dwell or separation (checked on the 2026-10-30 launch log), so dwell is attributed from the runbook order and printed as `0.45*`. (An earlier version of this entry recorded a per-rung `plant_id` switch via `ros2 param set /skill_node plant_id`; `plant_id` is still read live at every goal accept, but the runbook no longer uses it.)

- **G. Goal start line carries dwell and separation (second commit, owner feedback).** The owner asked whether per-rung `plant_id` switching was necessary and for a Runbook section they can follow at the robot. It was not: the bias read is the plant residual (y - u), which the learner does not move, so one memory serves every rung. `tools/probes/hold_rung_bias.py` was rewritten to attribute every memory row to its goal and hold from the launch log (`--latest`), print one row per goal, and end with a `NEXT:` line naming the hold to set and the goal to fly; the pre-registered rule lives in the probe. The goal start line logged apex but not dwell or separation, so skill_node now appends ` · dwell <d.dd> s · sep <N> mm` to every pattern's start line (one call site, `_juggle_goal`); the probe uses the logged dwell and falls back to goal-order inference (marked `*`) only on older logs. Runbook § 11 is a numbered list of goals in the GUI's own labels, no tables, no plant_id.

## Discussion

**C1: the 10-17 note reframed.** "0.30 at apex 1.0 plans at jerk 194k, 17.1 deg" reproduces ONLY at pre-release hold 0. At P4 600/8000/200k, hand 3900, apex 1.0, dwell 0.30: hold 0.100 INFEASIBLE (the hold rows, `cup_cycle.py` ~1643-1655, together with the pose shaper rows bind; removing the I-PLAN-14 skid rows or the C-CUP-4 rows alone leaves it infeasible; removing both solves, but the realised plan then fails HAND_LIMIT_ACC 4308 / seat 75 deg / skid 1141 mm/s); hold 0.050 LIMIT_ACC 8703 > 8000; hold 0 one ball OK (jerk 177.5k, hand 3665) and two balls accepted at jerk 186.9k, seat 18.7 deg, but over the 0.9 x 200k = 180k box margin. The 10-17 number came from the gitignored wayfinder prototype that had none of the hold rows, C-CUP-4 or I-PLAN-14, so it was effectively a hold-0 number. The two logbook door cells reproduce at hold 0 (1.05/0.30 OK, tau 0.313, hand 3843; 1.00/0.35 needs hand 3925).

**C2: JB-102 reframed; the hold's real purpose.** The planner already composes the whole cup velocity (Platform carrier + stroke) into the ball's launch velocity: the release triple pins cup position, cup velocity = take-off velocity, cup acceleration = g (`cup_cycle.py` ~1636-1641). The hold pins vel_xy = kappa * vel_z over the last N knots so the carrier is ~0 for the last 100 ms, protecting against PLANT TRACKING DELAY (`unified_cycle.py` ~288-307: 45 ms delay + 30 ms lag on the Platform, ball separating +10..+20 ms late; R4 saw +10..+40). So "release from a moving Platform composing its velocity" is already true; the open design item is DELAY-AWARE composition (zero the delay+lag-kernel-weighted carrier at t_rel + delta_sep, two rows replacing the N per-knot hold rows, linear/convex, kernel delta_eff from this Sitting). Described only, not implemented.

**C3/C4: what the hold costs in landing.** At 0.45/150/0.95 hold 0 plans OK today (leg vel 211, jerk 52k, hand 3486) with the planned carrier -399/-229/-145/-74/-24 mm/s at -150/-100/-75/-50/-25 ms before release and ~0 at release; under the 45+30 ms model the real carrier at separation is -116/-92 mm/s at +10/+20 ms. Predicted dx bias (short = negative) is in the runbook § 11 table (hold 0: -125/-103/-81/-47 mm at +0/+10/+20/+40 ms; hold 0.050: -44.5/-31.5/-22.3/-11.0). Two-ball door cells at hold 0 still land -265/-209 and -191/-151 mm short: removing the hold is not enough for the destination, it needs C2's composition. Caveat: if the Hand lags as much as the Platform the bias tends to 0 (the +40 column); the Sitting measures exactly this.

**C5: why hold 0 is gated on the 0.050 measurement.** Learner state x = (site xy, seat offset); hold and apex are not in x, so rungs share one memory neighbourhood (E: accepted; the residual read is invariant to it). k = 16 recent rows absorb a constant offset over ~16 throws; authorities are learner command +-20 mm and catch re-aim inward 35 / outward 10 mm, so at most ~55 mm constant inward bias is absorbable in steady state (35 mm before adaptation). 0.075 is trivially absorbed, 0.050 (-22..-32) is learned in ~16 throws, hold 0 (-81..-103) is not absorbable and drops. JB-102's gate as written (scatter + catch rate) cannot see a pure mean shift the learner and catch absorb, so a mean-bias criterion is added.

**C6: pre-registered criterion.** Primary read = plant residual (y - u) along the throw axis from the rung's `memory.csv` (does not move when the learner adapts); secondary = `oval_carry_bag_metrics.py` landed dx, first 5 throws per rung. sigma_x ~ 16 mm, n = 20 per rung gives SE 5.1 mm on a rung difference (2 sigma ~ 10 mm). "Delay effect confirmed": D(dx)(0.050 - 0.100) short by >= 10 mm AND beyond 2 SE of the rung difference, with |dy| < 10 and |dapex| < 10 mm (a -10..-12 mm shift at sd 16 mm, n = 20 reads inconclusive: fly 20 more at 0.050, else fall back to 0.075 and stop). "No effect": |D| < 10 mm, relative delay >= ~40 ms, hold can go freely. Hold-0 go/no-go on the predicted bias at the inferred delay: <= 35 mm fly; 35-55 warm up at 0.025 (>= 20 throws) first; > 55 do NOT fly hold 0. Fallback: the shortest hold with |D| <= 20 mm, >= 30/30, sd <= baseline + 3 mm (model: 0.075, or 0.050 if delta_eff >= ~25 ms). Fallback and decision are pre-registered so a pivot is planned, not a sunk-cost spiral.

**Correction 2026-10-31 (flagged by the peer session that re-swept the box).** § 11 goals 1 and 2 said Sep. 100 and Apex 0.90; every flown Columns goal (10-28 and 10-30 launch logs: separation_mm=125.0, apex_m=0.95) and every Columns cell in the box is at separation 125, and select() matches site xy to 1e-6 mm, so Sep. 100 would have been a BOX refusal at the Sitting. Both goals now read Sep. 125, Apex 0.95. The re-swept box (gate_hash c6c4d7adccf9, 74 cells) keeps the 54 cells § 11 relies on value-identical.

**Run order.** Columns first: it is the cheapest check of the pose default (narrower x window) and its pass is a precondition for trusting anything after it. The in-launch 0.100 control precedes 0.050 so the comparison shares a launch. Apex last: it is independent of the hold question, costs the most Hand heat and is hand-capped near 1.05.

## Outcome / Next

Fly runbook § 11 (rows 11-0..11-6). The delay-aware composition is a P4 design item (gated on the Sitting's delta_eff), not part of this Sitting. Status stays in-progress until the Sitting results are filled.

## Verification

- Second commit (probe rewrite, start line, section 11): ./run_tests.sh --full, run 2026-10-30 (second commit, temp/logs/sp_full5.log): parallel 7589 passed, 55 skipped, 4 warnings in 345.36s (0:05:45); serial 8 passed, 7644 deselected in 30.35s; exit 0.

- Trajectory node: 2026-10-30, `pytest tests/ros/test_trajectory_node*.py tests/ros/test_install_segment.py -q -p no:randomly -n 4 --dist loadfile -m "not nightly"`: 243 passed (`test_trajectory_node.py` alone collects 153).
- Box: `tests/motion/test_skills_admissible.py` 97 passed, 2026-10-30; sweep `temp/sweeps/sweep_apex_ext.sh`, 2 rounds byte-identical (4 cmp pairs, ~9 min/round), merged by `temp/sweeps/merge_apex_ext.py` asserting the original 46 cells value-identical; gate_hash `81a2244c1297` unchanged.
- Probe smoke, 2026-10-30, `python3 -I tools/probes/hold_rung_bias.py --file temp/learn/jugglebot/memory.csv --last 50`: n=50 caught=50/50, (y-u) x mean -0.3 mm sd 16.0, y +13.3 / 10.7, apex +62.5 / 5.3.
- Probe rewrite, 2026-10-30 (launch-log attribution, no `--plant-id`): `pytest tests/motion/test_hold_rung_bias_probe.py -q -p no:randomly` 9 passed; `python -I tools/probes/hold_rung_bias.py --launch-log <10-30 16:17 launch.log>` finds the 5 oval goals (50/50, 5/6 stopped, 6/6 `reload`, 50/50, 40/40).
- SIMGATE: 2026-10-30, `python sim/skills_gate.py --learn --target-throws 12` (pose default, BLAS 1 thread; logs `temp/logs/sp_*.log`): columns 100/0.90 seeds 0-4 PASS 5/5 (12 catches, 0 bumps, no BOX-EXPECTED-LANDING, release error max 17.8 ms); oval 150/0.95/0.45 seeds 0-2 at hold 0.100/0.075/0.050/0.0 PASS 3/3 each (release error max ~17 ms at every hold: the sim plant has no tracking delay, so the hold's effect is invisible in sim); apex 1.00 and 1.05 at dwell 0.50 PASS 3/3 each; dwell 0.40 at hold 0.050/0.0 FAIL 3/3 NO_ADMISSIBLE_COMMAND (no 0.40 box cell, not flown). `tests/sim/test_skills_gate.py`: 35 passed in 165 s, 2026-10-30.
- - ./run_tests.sh --full, run 2026-10-30 21:47-21:54 on the final tree (temp/logs/sp_full3.log): parallel 7576 passed / 55 skipped in 343.69 s; serial 8 passed in 30.62 s; exit 0.
