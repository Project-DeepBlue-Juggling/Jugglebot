---
title: "Two-ball oval P3 re-gate: GO for pose at dwell 0.50 (50/50 vs 50/50), the 0.45 goal ends on an 80 ms emitter gap, and the dwell limit is the 0.100 s pre-release hold, not the legs"
type: investigation
date: 2026-10-30
status: complete
phase: "two-ball-oval P3 - re-gate Sitting; P4 premise"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-24-two-ball-oval-p3-ab-sittings-skid-bound.md
  - 2026-10-28-two-ball-oval-p3-regate-prep.md
  - 2026-10-28-jb95-solver-worker-sched-fifo.md
files_changed:
  - tests/hardware/session_oval_rung1.md
  - plans/active/two-ball-oval.md
  - map/bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid.md
  - map/feature-trim-the-nice-0-chain-cpu-the-flame-graphs-point-at-skill.md
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - config/generated/admissible_box.yaml
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_segments.py
  - tests/motion/test_splice_tilt_rate_seam.py
  - tests/motion/test_validate_cycle.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node.py
  - tests/sim/test_skills_gate.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - logbook/2026-10-28-two-ball-oval-p3-regate-prep.md
  - temp/learn/jugglebot/memory.csv
subsystem:
  - motion
tags:
  - oval
  - p3
  - regate
  - dwell
  - headroom
---

# Two-ball oval P3 re-gate: GO, the 0.45 ending, and the dwell headroom

## Summary

The P3 re-gate Sitting flew 2026-10-30 16:17 in one launch of the default configuration (CFS solver worker, bridge FW 34; 0 SETPOINT_STALE, 0 CLAMP_DUTY, 0 mocap overflow). `pose` at dwell 0.50 matched the same-launch `level` column on catches (50/50 and 56/56 vs 50/50 plus 5), drew less leg current, lost no seat bit, and arrived at touch-down with the cup nearly still (v_x median -73 mm/s against a 175 bound; 10-24 read -507). Owner: GO. A `pose` 0.45 goal then caught 40/40 and ended at throw 41 on one 80 ms emitter wake gap, which is the environment (JB-98), not the planner. Offline the same day, the dwell limit under `pose` turned out to be the 0.100 s pre-release hold, which reframes P4.

## Evidence

Sources: launch log `~/.ros/log/2026-10-30-16-17-11-746307-jetson-190841/launch.log`; bag `~/Desktop/rosbags/2026-10-30_16-17-11`; `tools/probes/oval_carry_bag_metrics.py`.

| Goal | Planner | Dwell | Result |
|---|---|---|---|
| warm-up | level | 0.27 columns | 4 columns goals |
| O1 oval_1ball x50 | level | 0.50 | COMPLETED 50/50 |
| O2 | level | 0.50 | stopped by the operator at 5 caught, to set the param |
| O3 | pose | 0.50 | 6/6 |
| O4 oval_1ball x50 | pose | 0.50 | 50/50 |
| O5 oval_1ball x50 | pose | 0.45 | 40/40 caught, then LANE_REFUSED |

P3R-4, `level` vs `pose` at 0.50 (same launch):

| Measure | level | pose |
|---|---|---|
| catches | 50/50 (+5/6, stopped by the operator) | 56/56 (50/50 + 6/6 fed) |
| legs 1/2 peak current | 7.38 / 9.96 A | 6.59 / 9.45 A |
| ball_held unbroken | 49/49 | 55/55 (no flickers) |
| carry LIMIT_* | none | none |
| touch-down cup v_x median | -6 mm/s | -73 mm/s (max abs v_x 126; 0 carries over 175) |
| window jerk median (bag echo measure) | 63.0k | 25.8k |
| landing scatter sd | about 14-16 mm | about 14-16 mm (dx/dy -0.5 (15.9) / +0.4 (11.8)) |

Two `level` carries had seam-jerk spikes (337k, 689k); no `pose` carry did. 6 `SPLICE_TOO_LATE` re-aim refusals (3 level, 2 pose, 1 O2); none ended a goal.

P3R-5, `pose` 0.45: 40/40 caught; seat bit flickered on 4/39 carries (0.09-0.10 s, 0.28-0.40 s before release, ball still thrown); legs 1/2 peak median 6.56 / 8.81 A, leg 2 over 10.05 A (the old rung-2 reference; the same-launch `level` peak, now the P3R-4 reference, was 9.96 A) on 2 carries (10.70, 10.37); jerk median 38.9k; touch-down v_x -59 (max 125). Fault chain: at throw 41 a single 80 ms emitter wake gap during a 125.7 ms wall / 101.5 ms CPU CATCH solve (93 involuntary context switches) -> firmware scheduler STOP (+2) -> `LANE_REFUSED` -> hand `MAX_DEVIATION` +6.56 rev -> recovery failed ("a real fault wins"). Launch-wide: 444 CATCH solves, wall p50 104 / p90 136 / max 194 ms, CPU p50 81 ms, wall/CPU 1.29, 19 solves over 150 ms. Not read: bridge staircase counter, FET temperature, the clamp > 50 ms rule. The records ledger is unchanged by design (50/50 ties the 2026-10-28 record; it writes only on strictly more).

### Software tail (2026-10-30 evening)

- **Default flip.** `trajectory_op.planner_mode` `level` -> `pose` in `config/hardware_config.yaml` (the single source; `python config/generate_config.py` regenerated `config/generated/hardware_config.{py,h}`, the ROS copy and the three firmware headers). Every consumer derives from it (`unified_cycle.PLANNER_MODE_DEFAULT`, `SegmentConfig.planner_mode`, the `skill_node` / `trajectory_node` parameter defaults); `level` stays selectable per install. INVARIANTS.md I-PLAN-13 prose names the pose default.
- **Box re-sweep.** `hardware_config.py` is gated, so gate_hash moved f8a9e188b2cf -> 81a2244c1297 and every cell was re-swept: `temp/sweeps/sweep_pose_default.sh`, 14 invocations per round, every family under both planners, oval at 0.65/0.55/0.50/0.45, BLAS one thread, two rounds (30.4 + ~31 min wall) byte-identical by `cmp` on all 14 files. Merged box: 46 cells, 23 `level` + 23 `pose`, swept_at 2026-10-30, limits unchanged. All 22 previously committed `level` cells and the 4 `pose` oval cells are value-identical; new: `level` oval 0.45, `pose` single (5), hop (8), columns (6). No `pose` cell is empty where its `level` twin is not (columns 0.825-0.925 empty under both). Hop and the four ovals admit identical landing windows under both planners; `pose` reshapes self_toss (y +-30 -> +-20 mm in the 0.45-0.55 band; x [-40,30] -> [-30,30] above) and narrows columns 0.925-0.975 in x (P2 [-2,4] -> [-1,2], P1 [-4,2] -> [-2,1]) while moving y ([-8,20] -> [-20,10]). Observation: the `level` oval 0.45 cell is admissible and identical to `pose`'s, although the 2026-10-23 notes had `level` refusing 0.45; worth a look before relying on `level` at 0.45.
- **Tests re-pinned, no gated file edited after the sweep.** First scoped run 191 failed / 3582 passed. About 177 were fixture boxes stamped `level`, refused as a planner_mode mismatch (restamped `pose`: `test_skill_node.py` 8 sites, `test_skills_executor.py` 1; the deliberate `level` tests kept). 14 motion tests were triaged one by one: 10 asserted numbers measured on the `level` program (the 2026-10-19 cone / stroke-end figures in `test_validate_cycle.py` x7; the I-PLAN-11 rate-pin guard in `test_splice_tilt_rate_seam.py` x2, since a `pose` CATCH never calls `tilt_schedule`; the hold-rows message in `test_skills_segments.py`; the cone45 snapshot filter in `test_skills_admissible.py`) and now pin `planner_mode='level'` with a docstring line saying why, with `pose` twins added where `pose` refuses differently (`POSE_WINDOW: no knots left`); 3 were load flakes with no path to the default (QP/gate ratio under load; the budget test at 56 vs 50 ms under a parallel run, 2 passed serial; a worker THROW bit-mismatch under bare pytest without the BLAS cap); 1 was a finding, filed as JB-104: `test_a_hop_catch_releases_from_the_caught_position_too` lands +15 mm off-site at hop 250 / apex 0.9 / dwell 0.30, outside the box (the `pose` hop cell there admits +-1 mm) and on the feasibility edge for both planners (`level` OK at 15, both INFEASIBLE at 16); `pose` ends UNVERIFIED (equality residual 8.0e-03 after 66 iterations) where `level` just converges. The gate held (refused, not returned); the residue is the refusal's name. The test pins `level` at +15 and gains a `pose` twin at +10.
- **Sim gate under the `pose` default** (run 2026-10-30 19:47, `temp/logs/pd_simgate.log`, BLAS one thread): `python sim/skills_gate.py --learn --pattern oval_1ball --separation-mm 150 --apex-m 0.95 --dwell-s 0.50 --seeds 0 --target-throws 12` PASS (wall 23.3 s); same at `--dwell-s 0.45` PASS (23.0 s); `--learn --pattern columns --seeds 0 --target-throws 12` PASS (19.8 s); 0 bumps in all three.

## Discussion

**Why GO.** The 10-24 NO-GO had one named cause, the cup moving inward at the Catch (-507 mm/s) so balls landing more than 15 mm inward met the rim. The skid bound (I-PLAN-14, 175 mm/s) removed it: -73 median, no carry over the bound, and the catch count went from 54/66 to 56/56 (50/50 + 6/6 fed). Every other P3R-4 criterion holds with margin (current lower, no seat-bit flicker, no `LIMIT_*`, equal scatter). The reference for the current criterion is the same-launch `level` column (owner), because 7.49 / 10.05 A came from a different day's launch; the old numbers become history.

**Why the 0.45 ending is the environment, not the planner.** All 40 catches were clean and the touch-down velocity stayed inside the bound. The goal died on one 80 ms gap in the emitter's wake time, longer than the firmware's 70 ms STOP cover, while a CATCH solve was running at 101.5 ms CPU with 93 involuntary context switches. That is the 2026-10-28 22:26 signature (JB-95) occurring under CFS, so the stall is not specific to FIFO scheduling. It very likely explains JB-98's unexplained 21:02 trip as well. JB-98 is therefore reframed: the setpoint stream has a 70 ms firmware STOP cliff and the loaded box can stall past it. It stays open. The next CPU lever is the `skill_node` executor trim (JB-94); JB-95 stays backlog.

**The headroom finding and what it does to P4.** Offline analysis (scratch probe built on `tools/probes/oval_dwell_sweep.py`; to be promoted to `tools/probes/` before P4). The dwell limit under `pose` is the 0.100 s pre-release hold. At 0.40 the refusal is `HAND_LIMIT_ACC` 4308 vs 3900, flat across apex, and it vanishes when the hold shrinks: hold 0.050 makes 0.40 admissible at today's limits (hand -10 percent, leg velocity -12 percent); hold 0.025 makes 0.35. The P4 working point 600/8000/200k produces identical pose plans, because limits only gate, so it buys margin and no reach. Apex is capped near 1.05 m by hand acceleration (3489 / 3657 / 3843 / 3902 rev/s^2 at 0.95 / 1.00 / 1.05 / 1.06, fixed stroke). The 2026-10-24 premise had P4 laddering dwell by raising the working point; that is wrong for reach, so P4.0 (the hold rung) goes first. The hold exists because of the R4 finding (2026-09-27/28): the Platform still moving +70..+100 mm/s at separation added overshoot; 0.100 = 45 ms tracking delay + 30 ms lag + 20 ms. Removing it means releasing from a moving Platform, a design item rather than a parameter change.

**Two-ball arithmetic, with its caveats.** Two balls in one hand give T_f = dwell + 2 tau. A hand estimate put tau_min at about 0.30 s (about 0.050 post-hold + 0.075 capture + 0.175 transit at 150 mm), giving minimum apex 1.48 m at dwell 0.50, 1.35 at 0.45, 1.23 at 0.40, 1.11 at 0.35, 0.99 at 0.30 (+0.1 m of apex buys about 22 ms of tau). The two admissible cells are planner solves, not table entries: the two-ball fold at the P4 working point (600/8000/200k), pre-release hold 0, post-release hold 0.050, where tau is whatever the flight leaves, tau = (T_flight - dwell)/2. Apex 1.00 / dwell 0.35 solves at tau = 0.277 s (throw hand 3925) and apex 1.05 / dwell 0.30 at tau = 0.313 s (hand 3843). The 1.05 / 0.30 cell fits under the configured hand limit 3900; the 1.00 / 0.35 cell needs the limit raised from 3900 to the measured authority (3925.5), so 3925 is the acceleration it demands, not a limit it meets today. The planner accepting tau = 0.277 makes the hand estimate's 0.175 transit term about 25 ms conservative: at that tau the table's 1.11 m becomes 1.00 m. With the hand limit hypothetically 6000, 1.1 / 0.30 and 1.2 / 0.30-0.40 also solve (throw hand 4119 / 4629, beyond the measured authority), so apex is hand-capped, not planner-capped. Caveats: the two-ball fold was the analysis's own construction, since no production two-ball compiler exists; and it contradicts plan section 1.2's 10-17 note "0.30 plans at apex 1.0 under P4", which predates the skid bound and C-CUP-4 and was not re-run. Reproducing that result on today's code is an offline item ahead of P4's commit.

**Owner decisions 2026-10-30 and why the destination stands.** (a) GO; default flips to `pose`, every family re-swept under `pose`, sim gate, one columns check Sitting. (b) The leg-current reference is the same-launch `level` column. (c) A pre-release-hold rung is scheduled ahead of the P4 working-point commit; the ideal is no hold with the Platform moving continuously; ladder 0.100 -> 0.075 -> 0.050 -> 0 at dwell 0.45, judged on landing scatter and catch rate (map JB-102). (d) The two-ball destination is not changed (owner refused): the numbers permit a path (no hold, apex 1.0-1.05, dwell 0.30-0.35), so the destination is hard rather than closed. (e) The next CPU lever is JB-94.

## Outcome / Next

- Default flipped to `pose`, every family re-swept under both planners, sim gate PASS (all landed 2026-10-30, see Software tail); one columns check Sitting follows.
- P4.0 pre-release hold rung (JB-102) precedes the working-point commit; the offline reproduction of the 10-17 0.30@1.0 result and the probe promotion precede it.
- JB-94 (executor trim) is the next CPU lever; JB-98 stays open under its reframe.
- Docs changed: runbook section 9.1, plan P3/P4, map notes on JB-98 and JB-94, `config/generated/admissible_box.yaml`, `INVARIANTS.md` (I-PLAN-13), the 2026-10-28 re-gate prep entry; code changed: `config/hardware_config.yaml` and its generated copies (`config/generated/hardware_config.{py,h}`, the ROS copy, three firmware headers), re-pinned tests (`tests/motion/test_skills_admissible.py`, `test_skills_executor.py`, `test_skills_segments.py`, `test_splice_tilt_rate_seam.py`, `test_validate_cycle.py`, `tests/ros/test_install_segment.py`, `test_skill_node.py`, `tests/sim/test_skills_gate.py`), `temp/learn/jugglebot/memory.csv`.

## Verification

- ./run_tests.sh --full, run 2026-10-30 19:55-20:01 on the final tree (temp/logs/pd_full.log): parallel 7551 passed / 45 skipped in 343.94 s; serial 8 passed in 30.63 s; exit 0. Sim gate (temp/logs/pd_simgate.log, 2026-10-30 19:47): oval_1ball 150 mm / apex 0.95 at dwell 0.50 PASS, 0.45 PASS, columns PASS, 0 bumps.
