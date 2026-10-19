---
title: "C-CUP-4 cone refuses the columns self-toss catch on its stroke-braking knot: cone 30 -> 45 deg, judged from the hand stroke's end"
type: investigation
date: 2026-10-19
status: in-progress
phase: "two-ball-skill-stack - post-R6 (columns empty-cup two-feed, 15:23 sitting) / two-ball-oval P1 contract"
related_plan: two-ball-oval.md
related:
  - 2026-10-19-held-axis-reload-catch-refuses-tracker-reaim.md
  - 2026-10-18-two-ball-oval-p1-oval-pattern-dispatchable.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - sim/skills_gate.py
  - tools/admissible_sweep.py
  - tests/hardware/session_oval_rung1.md
  - tests/motion/test_validate_cycle.py
  - tests/motion/test_skills_admissible.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skills_plan_bench.py
  - tests/sim/test_skills_gate.py
  - tests/fixtures/cup_catch_plans/README.md
  - tests/hardware/session_skill_stack_rehearsals.md
  - plans/active/two-ball-oval.md
subsystem:
  - motion
  - sim
tags:
  - safety
  - dynamics
  - testing
  - docs
---

# C-CUP-4 cone refuses the columns self-toss catch on its stroke-braking knot

## Summary

The 2026-10-19 15:23 columns sitting flew 7 attempts and none succeeded. Four ended `CUP_SEATING_CONE`, two ended `ABORTED_NO_RELEASE` after cone-refused re-aims, and one ended `LIMIT_JERK`. Every refusal read 31.6-41.7 deg against the 30 deg cone, and exactly ONE interior knot was outside it each time. That knot is `catch_k + 1` of the columns self-toss CATCH at the far site, spliced after the hop-entry transit. At that knot the centred second difference straddles touch-down, where the hand's stroke starts braking while the platform sheds the transit's lateral velocity. The skill stack flew 32 consecutive columns catches through this window at R5 (2026-10-13); C-CUP-4 was written (2026-10-18) for the oval's loaded CARRY and was never meant to judge a brake-in. The leading hypothesis of the afternoon, that the morning's new capture move (a15bd458) had pushed the feed catch past the cone, was WITHDRAWN by an offline probe. The owner amended the contract: the judged span now starts where the hand's catch stroke ends, braking knots are read but never refused, and the cone is 45 deg (floor unchanged at 0.30 g). Status in-progress: code and tests landed uncommitted, NOT FLOWN; the admissible box is stale until it is re-swept.

## Symptoms

15:23 sitting (`~/.ros/log/2026-10-19-15-23-58-088700-jetson-3231325/launch.log`, bag `~/Desktop/rosbags/2026-10-19_15-23-58`; 33 C-CUP-4 lines in the launch.log and the same 33 decoded from `/rosout`). The launch ran the `Jugglebot-skills` install rebuilt at 14:53 from a9feb1c9, which carries C-CUP-4 at the 30 deg cone (the check's `feasibility.py` was installed at 09:21 from 40b38556, so the 11:04 sitting ran under it too). 7 attempts, 0 succeeded.

| # | accepted | mode | cone refusal(s) | where | outcome |
|---|---|---|---|---|---|
| A1 | 15:26:10 | columns (reload) | none | - | `LIMIT_JERK` (1/3 caught; skill 3 CATCH, leg jerk 229k) |
| A2 | 15:27:00 | columns (reload) -> feed | 41.7 deg @ knot 12, 1/11 | skill 2 CATCH, first solve | `CUP_SEATING_CONE` (1/2 caught) |
| A3 | 15:27:29 | columns hop entry | 34.8 deg @ 13, 1/10 | skill 2 CATCH, first solve | `CUP_SEATING_CONE` (1/2 caught) |
| A4 | 15:27:40 | columns hop entry | 40.9 deg @ 13, 1/10 (skill 2 re-aim, 31.6 mm); 33.4 deg @ 13, 1/12 (skill 4 re-aim, 12.5 mm) | re-aims after an accepted first solve | RESEND-REFUSED x2, then `ABORTED_NO_RELEASE` (2/3 caught) |
| A5 | 15:28:01 | columns (reload) -> feed | 31.6 deg @ 13, 1/10 (skill 2 re-aim, 11.1 mm); 33.8 deg @ 13, 1/12 (skill 4 re-aim, 10.7 mm) | re-aims | RESEND-REFUSED x2, then `ABORTED_NO_RELEASE` (2/3 caught) |
| A6 | 15:28:26 | columns (reload) -> feed | 37.4 deg @ 13, 1/10 | skill 2 CATCH, first solve | `CUP_SEATING_CONE` (1/2 caught) |
| A7 | 15:28:53 | columns (reload) -> feed | 35.7 deg @ 13, 1/10 | skill 2 CATCH, first solve | `CUP_SEATING_CONE` (1/2 caught) |

Reason text, A2 verbatim (head): `REJECTED_CYCLE_INFEASIBLE(CUP_SEATING_CONE: cup seating (C-CUP-4): realised specific force 41.7 deg off the prescribed cup axis at knot 12 > cone 30 deg - 1/11 interior loaded knots outside it; the ball would skid on the cup wall ...)`.

Shared features of all eight refusals: every value is just over the cone (1.6-11.7 deg), exactly one knot of 10-12 is outside it, and that knot is segment-local 12 or 13. Every refusal sits on a skill 2 or skill 4 CATCH (the self-toss catch at the +62 site, reached by a ~125 mm x transit from the -62.5 site). No BB feed catch (skill 1) and no reload catch was ever refused on the cone. The 2 `ABORTED_NO_RELEASE` ends each followed a MISSED Ball 2 (`throw 2/4 MISSED`), so throw 4/4 "never left the hand" probably because no ball was in it: the known end-reason attribution gap, not a new defect.

11:04 sitting (`~/.ros/log/2026-10-19-11-04-03-078330-jetson-3068207/launch.log`): `CUP_SEATING_CONE` as an END code was never seen. C-CUP-4 fired once, co-reported behind a `LIMIT_JERK` (313335 mm/s^3 > 200000) on a skill-4 cross-site re-aim: 33.4 deg at knot 10, 1/11. That sitting had ONE columns cross-site run (done 3/4) against seven at 15:23, so it is the same window type seen once, not a different regime. 15:23 passed 3 of 7 first solves.

## Diagnosis

**Which window.** The fed columns schedule has six skills: skill 0 THROW Ball 1 (the hop, P1 to the cross site), skill 1 CATCH Ball 2 (the BB feed catch at the feed site), skill 2 CATCH Ball 1 at the far site. In the hop-entry attempts skill 2 always splices at `k_s 40` = 1.000 s with Ball 1's touch-down at event tau 1.305-1.308 s, i.e. local knot ~12.2, so local knot 13 is touch-down + 1, the first interior loaded knot. The 11:04 case fits the same arithmetic (first solve tau 2.455, re-aim at `k_s 89` = local 9.2, worst knot 10). This is INFERRED from the knot arithmetic: the plan's own `catch_evidence.knot` is not logged. The offline replica of the hop-entry skill-2 plan (below) puts `catch_k` at 11 and the braking peak at knot 12, one knot earlier than the bag arithmetic; the logged "knot N" is segment-local, so a one-knot indexing offset between the two readings is not excluded.

**Physical reading.** At touch-down the hand's stroke is at its fastest (-48 rev/s along the catch axis in the replica) and starts braking the ball into the cup. The centred second difference at `catch_k + 1` straddles touch-down: it mixes the pre-touch-down cup motion with the braking, and the platform is simultaneously shedding the transit's lateral velocity. The realised specific force there is not the carried ball's seating force; it is the brake-in. R5 flew 32 consecutive columns catches through exactly this window with the ball seating each time, so the readings are not evidence of a ball skidding.

**What was ruled out.**

- *The capture move (a15bd458).* An offline probe (2026-10-19, read-only; real `compile_reload`, real `executor.install_segment`, live limits 600/8000->5000/200k) built the reload held-axis LANDING catch clean and captured, for replay balls 10, 17 and 18, and read `validate_cycle` on `rig.records[-1].plan`. Captured and clean plans read identical C-CUP-4 values to <= 0.01 deg / 0.0001 g: **10.0 deg** (10.03 / 10.02 / 10.14 at knot 30), `a_z + g` 0.987 g, in every case. A re-aim sweep of 0, 5, 10, 15, 20, 28.3, 35, 40 mm on x and y separately, both signs, with the authority opened to 50 mm, read 10.02 deg identical to 1e-8: no crossing of 30 deg anywhere in 0..40 mm. The mid-flight RESEND path read 14.54 deg / 0.682 g at `splice_k = 3` for all three fits (it depends on replan timing, not on re-aim magnitude or direction) and 10.02 deg / 0.987 g at `splice_k` 5 and 7. By construction the capture span ends at the contact-window opening, 5+ knots before the first judged knot. Separately, the capture is opt-in only for LANDING-kind held-axis catches (`unified_cycle._events_for`: `capture = kind == LANDING and axis is not None`), which is the reload catch; every reload capture installed and caught on the cone, and no capture window was ever cone-refused. The level-after-END REST is also out: no `DECAY-ON-END` fired, and the refusals are first solves.
- *The robot frame.* Before attempt 1 the frame was measured at origin (-23.2, 27.9, 0.8) mm / yaw -0.80 deg, then MOVED four times by the owner (the last at 15:25:47), then held. After that: origin (-58.4..-59.4, 58.5..59.6) mm, yaw -9.15 to -9.28 deg (yaw-only build, fit rms 0.80 mm), against 11:04's 0.1, 1.0 mm / 0.01 deg, i.e. ~83 mm and ~9.3 deg different. No `REJECTED_ROBOT_FRAME_*` and no SETTLING lines appeared and nothing moved after 15:25:47. The cone check runs on the robot-frame plan, so the frame reaches it only through the tracker aims, and the tracker re-aim offsets at 15:23 (y -22..-52 mm) look like 11:04's (y -24..-55 mm) with no rotation signature. NOT supported as a cause; a replay of a refused solve with its seed would be needed to settle it fully.
- *The aim command.* The same skill-2 first-solve command u = (+7.6, -4.9) mm was REFUSED in A2 and ACCEPTED in A4. Aim alone does not split pass from fail; the difference sits in the live spliced state (below).
- *A larger off-site feed start.* Partly seen: four feed catches at 15:23 were aimed from the tracker at dispatch (`source=tracker`), never at 11:04, and the BB feed throw sat on the 20 mm y clamp every time. But A2 and A3 were refused with the 11:04-style schedule-then-resend start, so it is at most a contributor.

**The offline-vs-hardware gap.** The offline replica of the hop-entry skill-2 plan (a real solve at the class window, landing offset +(7.6, -4.9) mm, R5 limits) reads **16.5-21.7 deg at `catch_k + 1`** (20.16 deg in the case tabulated below) for landing offsets up to 30 mm, and could not be steered past 45 deg; the hardware read **31.6-41.7 deg**. The remainder is the live spliced state (the feed-catch re-aim and tracker-at-dispatch state upstream of the splice), which the replica does not carry, and it is unexplained. It no longer decides the gate, because that knot is now exempt, but it is why the regression test is hand-built (Fix).

Hop-entry skill-2 plan, per knot (catch_k 11, contact 6-21; centred cone reading, hand speed along the stroke in rev/s):

| k | 11 (catch) | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 | 20 | 21 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| deg | 56.86 | **20.16** | 7.13 | 3.66 | 0.28 | 4.62 | 8.71 | 1.97 | **0.23** | 0.08 | 0.01 |
| rev/s | -48.07 | -48.88 (peak) | -40.26 | -27.91 | -18.32 | -11.47 | -7.37 | -4.31 | +6.94 (reversed: END) | +38.6 | +96.2 |

## Discussion

Three options existed for the cone-refusals on that knot.

*Keep 30 deg, judge from touch-down + 1 (status quo).* Refuses the columns window the stack has flown 32 times. Rejected: the contract's job is the oval's loaded carry, and at touch-down + 1 it measures the brake-in, not the carry.

*Judge from touch-down + 2 (skip one knot).* Rejected: the exempt span is 6-8 knots in the replica plans (hop-entry 12-18, reload 21-27, rung-1 36-41), not one. A fixed skip would be a number tuned to today's logs and would not follow the stroke if its length changes. Judging from the END of the stroke is the same fact stated physically: while the hand is still braking the ball into the cup there is no carry to protect.

*Estimate the stroke length from physics (`v_td / hand_acc_limit`).* Rejected on measurement: the planner brakes at about 440 rev/s^2, not the 3900 rev/s^2 limit, so that formula gives 13 ms, under one knot, against the measured 6-8 knots. The stroke end is therefore read off the REALISED hand velocity of the plan being checked, per catch record: with `v_td` the hand velocity at the interpolated touch-down and `s(k) = hand_vel_rps[k] * sign(v_td)`, `k_b` is the speed peak (first `k >= catch_k` with `s(k+1) < s(k)`), and `k_end` is the first later knot where `s(k) <= 0.02 * |v_td|` or `s(k) >= s(k-1)`.

*Why the `s(k) >= s(k-1)` reversal clause.* Without it, "stroke done" fires only at 98 % shed. The rung-1 carry at dwell 0.30 has a hand that dips into the throw's wind-up (7.79, 7.92, 11.8 rev/s at knots 40-42) before it ever stops, so it would be exempt through the wind-up and hide **65 deg at knot 41**. With the clause, judging starts at knot 41 and that window is refused. The same clause is what ends the hop-entry stroke at knot 19, where the hand reverses (+6.94 rev/s).

*The exemption is BRAKING only (owner direction, applied after the first audit).* The first cut let `k_b` run off the end of the window when the hand speed after touch-down never fell, which exempted the whole window - a hand at constant or rising speed is not braking anything. The rule now returns `catch_k + 1` when `|v_td| < 1e-3 rev/s` or when no speed peak followed by a fall exists at or before the window's last contact knot.

*The known hole (accepted; the only one left).* A carry that physically overlaps REAL braking is not judged on the cone on those knots. Example: rung-1 at dwell 0.30 has an exempt knot reading 50.0 deg, and is still refused only because knot 41, after the stroke ends, reads 65 deg. `max_cup_braking_deg` is where such a case would show: it is recorded on every report and must be read in the next sittings.

*Why 45 deg.* The cup's physical cone is 55 deg; 45 deg leaves 10 deg under it for the estimator (a centred second difference of a 25 ms plan) and for the live state. The floor `a_z + g >= 0.30 g` is unchanged, including its span: the owner's decision was about the CONE, so the floor still runs over braking AND judged knots (touch-down + 1 .. last contact knot). The amendment is about where the cone's span starts and how far off-axis a loaded knot may be, not about the vertical seating force. Minimum `a_z + g` over the BRAKING knots alone on the five real plans (2026-10-19, scratchpad probe over the plans below): reload LANDING clean / ball-17 capture 1.334 g, hop-entry skill 2 1.437 g, hop-entry feed catch 0.743 g, rung-1 dwell 0.50 1.168 g, columns catch-carry-throw 1.170 g - none near 0.30 g, so the floor costs those plans nothing.

*The tradeoff, accepted explicitly by the owner (2026-10-19).* Raising the cone 30 -> 45 deg weakens the protection of the oval's loaded carry, which is the thing C-CUP-4 was written for, alongside the stroke-end rule (the exemption is what removes the 15:23 knot from judgement; the cone value is a separate owner call, set 10 deg under the cup's physical 55 deg cone). The owner accepted the weaker carry protection. **30 deg is recorded as the fall-back: if a loaded carry is observed losing the ball, restore `CUP_SEATING_CONE_DEG = 30.0` (kept in the constant's comment) and re-sweep.** Consequence for the oval: the rung-1 carry at dwell 0.50 reads 32.1 deg (it was refused at 30 deg; it passes at 45 deg), and the minimum passing dwell of 0.51 recorded on 2026-10-18 was measured at 30 deg, so that floor moves and needs re-measuring (Outcome).

## Fix

**Rule** (`motion/trajectory/feasibility.py::_cup_seating_check` + `::_cup_stroke_end_knot`, called from `validate_cycle`):

- `k_b = min{k >= catch_k : s(k+1) < s(k)}`; `k_end = min{k > k_b : s(k) <= 0.02 * |v_td| or s(k) >= s(k-1)}`; `k_end = catch_k + 1` if `|v_td| < 1e-3 rev/s` OR if no speed peak followed by a fall exists at or before the window's last contact knot (constant / rising speed: nothing is braking, nothing is exempt); `k_end = n_knots` if braking starts but never ends on the plan.
- Judged knots: `k_end` through the last contact knot (at most `n-2`). Cone 45 deg on the judged knots. Braking knots `catch_k + 1 .. k_end - 1` are exempt from the CONE only and recorded. The floor `a_z + g >= 0.30 g` runs over judged AND braking knots (its original span; the braking-only minima are 0.743-1.437 g on the five real plans, see Discussion), and `min_cup_seating_az_g` is the minimum over that span. A window with no catch is not judged; a braking-only window has no cone judgement but its floor is still checked.
- Constants: `CUP_SEATING_CONE_DEG = 45.0` (was 30.0; the comment keeps 30.0 as the fall-back), new `CUP_STROKE_DONE_FRAC = 0.02` and `CUP_STROKE_MIN_SPEED_RPS = 1e-3`, `CUP_SEATING_FLOOR_G` unchanged.
- Report fields: new `max_cup_braking_deg` and `cup_stroke_end_k` (a tuple, one entry per catch record); `max_cup_seating_deg` / `min_cup_seating_az_g` keep their meaning, now taken over the judged span. Cone and floor reason text still contain `floor 0.30 g` and `knot N`; `detail=True` returns `{'braking_deg', 'stroke_end_k'}`.
- Contract text: the C-CUP-4 row in `ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md` is rewritten first (invariant, formula, measured values, the known hole, enforcement and report fields, test list) and its Fate says `AMENDED 2026-10-19 (owner)`.

**Measured on real plans** (`validate_cycle` report, 2026-10-19):

| plan | catch_k | stroke end | worst judged (deg) | min a_z+g (g) | worst exempt (deg) |
|---|---|---|---|---|---|
| reload held-axis LANDING, clean | 20 | 28 | 10.02 | 0.987 | 7.39 |
| same, ball-17 capture (-20, -20) | 20 | 28 | 10.02 | 0.987 | 7.39 |
| hop-entry FEED catch (skill 1, level pin) | 11 | 20 | 0.03 | 3.741 | 1.99 |
| hop-entry skill 2, landing +(7.6, -4.9), R5 limits | 11 | 19 | 0.23 | 3.748 | **20.16 (at catch_k + 1)** |
| rung-1 oval window, dwell 0.50 (spy on the production window) | 35 | 42 | 32.11 (k49) | 0.820 | 23.56 (k36) |
| columns catch-carry-throw window | 35 | 42 | 4.34 | 0.839 | 2.16 |

Rung-1 dwell sweep (window gate): 0.50 OK at 32.1 deg; 0.45 OK at 39.4; 0.40 REFUSED `CUP_SEATING_CONE` at 49.1 (k45); 0.35 `LIMIT_VEL` plus cone 56.3 (k43); 0.30 `LIMIT_VEL` plus cone 65.0 (k41, `k_end` 41); 0.27 plus cone 79.3 (k40). The full rung-1 SEGMENT at dwell 0.50 now fails `LIMIT_VEL` on the SETTLE tail (447.8 > 350 mm/s at the 350 test limits), not C-CUP-4.

**Tests** (`tests/motion/test_validate_cycle.py`):

- `test_rung1_through_the_production_segment_path_refuses_the_cone` re-pointed and split into `test_rung1_through_the_production_segment_path_passes_the_45_deg_cone` (margin assertion via a spy on the production window; 32.1 deg) and `test_a_faster_rung1_carry_is_refused_after_the_stroke_ends` (dwell 0.40, a real refusal).
- New helpers `_hop_entry_skill2_plan` (the class window of the 15:23 refusals, a real solve) and `_with_pose_x_spike`.
- New: `test_the_stroke_end_is_read_off_the_realised_hand`; `test_a_braking_knot_outside_the_cone_passes_a_post_stroke_one_refuses` (a hand-built one-knot pose-x spike on the real plan: 10 mm at k12 reads 62.46 deg, exempt, plan passes; 15 mm at k19 reads 52.74 deg, judged, plan refused); `test_a_window_that_is_all_braking_is_not_judged`.
- Added after the audit: `test_a_hand_that_never_brakes_is_judged_from_touch_down_plus_one` (constant-speed, rising-speed and near-zero-`v_td` hands, hand-built `CyclePlan`s) and `test_a_braking_knot_below_the_floor_is_still_refused_though_its_cone_is_exempt`; `test_a_window_that_is_all_braking_is_not_judged` now also asserts the floor is read on the braking knots.
- Two docstrings refreshed (holdless rung-1 and the p3 fixtures). The regression is hand-built because the hop sweep could not be steered past 45 deg at `catch_k + 1` (the offline-vs-hardware gap above).

**Comment-only edits** (no behaviour): `sim/skills_gate.py` (the rung-1 32.1 deg note), `tests/motion/test_skills_admissible.py`, `tests/ros/test_skill_node.py`, `tests/ros/test_skills_plan_bench.py`, `tests/sim/test_skills_gate.py`, `tests/fixtures/cup_catch_plans/README.md`, and the test-name reference in `plans/active/two-ball-oval.md`. Comments saying "refused at 30 deg" in the plan-bench, gate and admissible tiny_sweep tests were annotated, not re-measured: the dwell-0.25/0.30 columns refusals may now pass.

## Verification

- 2026-10-19, `pytest tests/motion/test_validate_cycle.py tests/motion/test_validate_cycle_vectorised.py tests/motion/test_skills_admissible.py tests/motion/test_cup_cycle.py -q -p no:cacheprovider`: **257 passed, 1 failed in 28.93 s.** The one failure is `tests/motion/test_skills_admissible.py::test_the_committed_box_is_swept_at_the_launch_defaults_and_the_live_gate` (`LimitsMismatch`, gate hash `4c4d65a39ae1` -> `a7f11db8c7e1`): `feasibility.py` is a gate-hashed file, so the committed admissible box is stale. Expected; resolved by the re-sweep below.
- 2026-10-19, `pytest tests/motion/test_skills_executor.py tests/motion/test_skills_oval.py tests/motion/test_skills_capture_replay.py -q -x`: **294 passed in 10.08 s** (the executor / oval / capture set).
- 2026-10-19 (post-audit fixes: braking-only exemption, floor span restored), `pytest tests/motion/test_validate_cycle.py tests/motion/test_validate_cycle_vectorised.py tests/motion/test_skills_admissible.py -q -p no:cacheprovider`: **169 passed, 1 failed in 27.15 s**; the one failure is the same stale gate-hash test (`4c4d65a39ae1` vs live `2e23d662ead8`), pending the re-sweep.
- Admissible-box re-sweep (2026-10-19, `bash temp/sweeps/sweep_P1.sh` - four families x2, BLAS one thread, no gated-file edit during the run; log `temp/logs/sweep_cone45_20261019.log`): all four `cmp` IDENTICAL; merged 20 cells, gate_hash 4c4d65a39ae1 -> 2e23d662ead8. **The box MOVED**: hop (8), columns (2) and oval (1) boxes value-identical; all five single-site `self_toss` boxes RETURNED, on every key, to the PRE-OVAL 2026-10-09 snapshot (`tests/fixtures/admissible_box_pre_oval.yaml`): `landing_xy_m` 0.45-0.55 [-.03,.03]/[-.03,.03], the four higher bands [-.04,.03]/[-.04,.03]. The 2026-10-18 re-sweep had shrunk them (to +/-20 mm and [-.03,.03]/[-.02,.02]) because the sweep's same-site offset cell plans a loaded-cup FLYBACK whose first post-touch-down knot read C-CUP-4 at ~1 deg per mm of offset against the 30 deg cone - exactly the catch_k+1 braking reading this entry exempts - while the executor never dispatches that flyback (`test_skills_admissible.py` pin test, `plans/active/box-sweep-same-site-fidelity.md`). The one band the 10-18 sweep had WIDENED (0.65-0.75, x upper 0.03 -> 0.04, one grid step) is back at 0.03: the sweep fits the largest-area rectangle over the passing cells (`tools/admissible_sweep.py:578`), and with y widened the area-winner moved. Only the single-site self-toss pattern reads those boxes.
- Full gate (2026-10-19, `./run_tests.sh --full` on the working tree after the box write and the pin-test rewrite, log `temp/logs/gate_cone45b_20261019.log`): **6961 passed, 9 skipped in 314.19 s + serial 7 passed; RESULT: PASS**. The run before the pin-test rewrite (`gate_cone45_20261019.log`) read 6960 passed, 1 failed: the old `test_oval_resweep_pins_...` pinned the 10-18 self_toss values, replaced by `test_cone45_resweep_restores_every_non_oval_cell_to_the_pre_oval_snapshot` (scoped: 85 passed in 7.13 s).
- NOT run: any hardware sitting; the 15:23 refusals were NOT reproduced offline (the replica reads 16.5-21.7 deg at that knot), so no offline check shows the amended rule would have passed the live plans, only that the exempt knot is exactly the one the live refusals were attributed to by knot arithmetic.

## Outcome

NOT FLOWN. Code and tests are landed uncommitted on `two-ball-oval`; the admissible box is stale until re-swept (`seating_deg` per cell is re-recorded over the judged span by the sweep; `tools/admissible_sweep.py` now also records `max_cup_braking_deg` per cell as `braking_deg`, a log/print value only - the YAML box schema is unchanged).

Next: the next columns sitting with hop entry (rehearsal row 43). Grep its launch.log for zero `CUP_SEATING_CONE` ends and record the `max_cup_braking_deg` readings; if any carry is seen losing the ball, restore 30 deg per the fall-back above.

Residue, not scheduled:

- The oval dwell floor (minimum passing dwell 0.51, measured 2026-10-18 at 30 deg with 1.0 deg of margin) was measured at the old cone and needs re-measuring at 45 deg. Indicative only: the rung-1 window gate reads dwell 0.45 OK at 39.4 deg and 0.40 refused at 49.1 deg. The committed cell stays at 0.65.
- Unexplained: the offline 16.5-21.7 deg vs hardware 31.6-41.7 deg at `catch_k + 1` (feed-catch re-aim / tracker-at-dispatch upstream state).
- `ABORTED_NO_RELEASE` attributed to a missed Ball 2 (A4, A5) is the already-known end-reason attribution gap.
- Open follow-up named by the bag read: replay one refused skill-2 solve (A3, `k_s 40`) with its seed to settle the robot-frame question; no longer gating.
