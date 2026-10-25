---
title: "Hop re-aims and stop RESTs refused mid-transit: the splice seam is C0 but not C1 in the tilt-borne channels (22:53 sitting, first on the ASA cup)"
type: investigation
date: 2026-10-22
status: resolved
phase: "two-ball-skill-stack - post-R6 (hop/columns, 2026-10-21 22:53 sitting, first sitting on the ASA cup)"
related_plan: two-ball-oval.md
related:
  - 2026-10-19-limit-jerk-at-the-seam-after-a-bump-stale-cup-track.md
  - 2026-10-21-hand-cup-pla-broke-throw-catch-census.md
  - 2026-10-21-juggle-goal-carries-dwell.md
  - 2026-10-21-gui-oval-animation-shows-the-loaded-carry.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - tests/motion/test_splice_tilt_rate_seam.py
  - config/generated/admissible_box.yaml
  - temp/learn/jugglebot/memory.csv
  - tests/hardware/session_skill_stack_rehearsals.md
subsystem:
  - motion
  - config
tags:
  - safety
  - dynamics
  - kinematics
  - testing
---

# Hop re-aims and stop RESTs refused mid-transit: a tilt-rate seam

## Summary

The 2026-10-21 22:53 sitting (`~/.ros/log/2026-10-21-22-53-41-347705-jetson-3884763/launch.log`, bag `~/Desktop/rosbags/2026-10-21_22-53-41`) was the first on the new ASA-printed Hand cup and the flight check of I-PLAN-10 (rehearsal row 47). 18 goals: 6 hop, 11 columns, 1 oval (refused at accept). The owner saw every BB feed and hop/columns throw caught less smoothly than before, the catch rate improve through the session, and the oval refused. Three questions: was the improvement the learner adapting to the new hand; should the learner be reset; why was the oval refused.

- **Row 47 PASSED on hardware.** The launch ran the 95d148cd build (installed `unified_cycle.py` / `bump.py` md5-identical to source, `INSTALL_SKEW: OK`). The late-catch BUMP installed 20 times; 0 was followed by `LIMIT_JERK` at t = 0.025 s (10-19: 5 of 6 columns attempts), and 0 `CHAIN_DISCONTINUITY` fired.
- **The sitting's 12 other `LIMIT_JERK` refusals are an OLDER class**, not the new cup and not I-PLAN-10: a hop splice lands mid-transit where the head banks at ~0.28 rad/s and the new segment's tilt schedule pins the tilt VALUE but not its RATE. I-PLAN-10's `k_s-2` range only relabelled the refused knot from t = 0.050 s to 0.075 s. Fixed in code as I-PLAN-11 (offline 0/101 -> 38/101, zero t = 0.075 refusals), NOT FLOWN. A second class (the segment's own gate, knots 7-13) is characterised and left open.
- **Owner's three questions.** (1) The improvement is not attributable to the learner: its commands moved 5-17 mm, landing error did not shrink, and the inflection tracks hop -> columns and the feed catch starting to work. (2) Do not reset (owner, 2026-10-22: "Don't reset"). (3) The oval was refused because the live node dwell was 0.27 s against a box swept at 0.65 s; runbook row 12a (`ros2 param set /skill_node dwell_s 0.65`) was not executed and the GUI showed dwell read-only; closed by `2026-10-21-juggle-goal-carries-dwell.md`.
- **Catch smoothness** is a real change, but on BB feeds it is the ball settling 252 ms in the cup (PLA: 119 ms) at an unchanged closing speed. The lever is unproven, so the decision is instrument first plus two checks, no catch learner yet.

## Symptoms

Build check first: the launch ran the right code (see Summary). Attempt timeline (wall = 22:53:41 + offset; "c/n" from the END line):

| # | wall | goal | result | END reason / what the log says |
|---|---|---|---|---|
| 1 | 22:55:14 | hop (`reload`), apex 0.9, sep 250 | 4/4 | every throw "re-aimed 0x, then refused" (C2, JERK) |
| 2 | 22:55:37 | hop | 4/4 | same |
| 3 | 22:55:57 | hop | 0/0 | MISSED_CATCH skill 4 cup EMPTY; stop REST refused `LIMIT_JERK` (t = 0.075, knot 125); throw-1 re-aim `HAND_LIMIT_C2` 4.18 |
| 4 | 22:56:10 | hop | 0/0 | REJECTED_NO_BALL (ball lost in #3) |
| 5 | 22:57:04 | hop | 0/0 | MISSED_CATCH; stop REST `LIMIT_JERK` (t = 0.075, knot 126, 444900); re-aim `HAND_LIMIT_C2` 4.32 |
| 6 | 22:57:18 | hop | 4/4 | all four re-aims refused |
| 7 | 22:57:36 | columns, hop entry, apex 0.95, sep 125 | 0/2 | MISSED_CATCH skill 2 cup EMPTY (BB-fed Ball 2 landed (-53,-26)) |
| 8 | 22:57:46 | columns (`reload`) | 0/0 | `LIMIT_JERK` at the REST, 219k, leg 0, t = 0.025 |
| 9-10 | 22:58:10, :31 | columns (`reload`) | 0/0 | MISSED_CATCH, feed missed (Ball 2 skill 1 then Ball 1 skill 2 EMPTY) |
| 11 | 22:58:56 | columns (`reload`) | 2/2 | MISSED_CATCH Ball 2 skill 1 EMPTY; one re-aim refused JERK 287k t = 0.100 |
| 12 | 22:59:30 | columns, hop entry | 2/3 | ABORTED_NO_RELEASE; one JERK 224k t = 0.250, one LIMIT_ACC |
| 13 | 22:59:44 | columns, 10 cycles | 9/10 | done; last throw landed on the held ball |
| 14-17 | 23:00:09-23:01:04 | columns 50 | 3/3, 6/6, 14/14, 3/3 | each ends with a cup-EMPTY MISSED_CATCH after 3-14 catches |
| 18 | 23:01:20 | oval_1ball, apex 0.95, sep 150 | refused at accept | dwell_s 0.27 vs box swept at 0.65 |

Hops: 12 caught of 14, but 0 of 14 tracker re-aims accepted. Columns accepted ~28 of ~35 re-aims after 22:59:30. The first five columns goals all failed on the BB-fed ball (CATCH-AIM skill 1 source = schedule, `columns_feed_bb_bias_mm = [0,0]`); from 22:59:44 the feed was never again the terminal miss.

**Refusal census** (13 refused splices; the 39 matching log lines are each refusal echoed 2-3 times):

| t on the gated clock | count | what |
|---|---|---|
| 0.025 s | 1 | the 218661 leg-0 REST at a "0 throws" columns `reload` (22:57:48), no BUMP in that goal |
| 0.075 s | 4 | hop RESENDs (437969, 398983) and the two hop stop RESTs (423599, 444900); legs 4/5, ~2x the 200k limit |
| 0.100 s | 1 | columns RESEND 287151 |
| 0.125 s | 3 | hop RESENDs 260852, 291485, 263263 |
| 0.250 s | 4 | RESENDs 212722, 211488, 224480, 209256 |

Plus 7 `HAND_LIMIT_C2` re-aim refusals (4.18-5.41 vs the 3.9 rev/s^2 tolerance = 1e-3 x hand acc limit 3900), all in hop goals (5 of 7 on the first throw's catch, skill 4), and 4 `CUP_CONTACT_ACC` stop-REST dives in the offline probe. The 10-19 log had zero C2 lines (that sitting had no hops).

## Diagnosis

**1. Row 47.** 20 BUMP installs (first at log line 2086, 130 s after the one t = 0.025 refusal), 0 followed by `LIMIT_JERK` at t = 0.025 s, 0 `CHAIN_DISCONTINUITY`. Later refusals after bump installs were other codes (JERK t = 0.250 209k, INFEASIBLE, SPLICE_TOO_LATE 200.7 ms, transit outran deadline). BUMP declines are not splice failures: BUMP-LAUNCH (lateral launch velocity 128-153 > 50 mm/s) x3, LIMIT_ACC on the bumped plan x4, ~60 WITHIN-TOLERANCE.

**2. The t = 0.075 s signature is not new.** Planner diagnosis (offline, `probe_hop_splice.py`, the production `compile_one_ball` hop -> `SkillExecutor` -> `install_segment` -> `splice_at`/`extend` -> gate; hop sep 250 / apex 0.9 / dwell 0.27, limits 350/5000/200k, hand 3900): the same refusals appear in older logs with the same knot and peak, on a clock one knot shorter.

| log | range start / k_s | peak | t on clock |
|---|---|---|---|
| 2026-10-14 21:27 stop REST | knot 326 | 444815, leg 4 | 0.050 |
| 2026-10-19 14:24 hop re-aim | 152 / 153 | 455409, leg 5 | 0.050 |
| 2026-10-19 14:24 hop re-aim | 243 / 244 | 429251, leg 5 | 0.050 |
| 2026-10-21 22:53 hop re-aim | 221 / 223 | 437969, leg 5 | 0.075 |

95d148cd (I-PLAN-10) starts the truncated-head re-gate at `k_s-2` instead of `k_s-1`, so the same knot, `k_s+1`, prints as 0.075. Offline, the pre-I-PLAN-10 tree (0d948f96) and HEAD give bit-identical peaks and codes on all 101 probe splices; only the range start and the reported time move. The 10-19 hop sitting (14:24) was already on the two-ball-oval P1 content (merged 40b38556, 2026-10-19 09:21), so nothing in that merge is new for tonight either.

**3. Mechanism: the tilt seam is C0 but not C1.** `cup_realize.tilt_schedule(start_tilt=...)` pins knot 0's value only; `unified_cycle` writes `tilts[0] = state.pose[3:5]` back exactly but nothing carries `state.pose_vel[3:5]`, and `_accel_bounded_schedule` restarts the banking slew from rest at the seam. A hop splice lands mid-transit (every hop stop lands in the next transit because dwell 0.27 < the 0.43 s verdict instant), where the head banks at ~0.28 rad/s. Seam geometry for three hop stop RESTs (all refused `LIMIT_JERK` at `k_s+1`):

| case | head ry rate | segment ry rate | re-derived seam | head x vel | segment x vel | pose-x acc jump at k_s-1 / k_s / k_s+1 (mm/s^2) | peak |
|---|---|---|---|---|---|---|---|
| catch0 -0.50 s | -0.284 rad/s | +0.015 | -0.139 | 475.3 mm/s | 404.5 | +2474 / -474 / **-3277** | 454k |
| catch0 +0.45 s | +0.269 | -0.017 | +0.137 | -451.3 | -390.9 | -2223 / +462 / **+3004** | 428k |
| catch1 -0.55 s | +0.273 | -0.012 | +0.140 | -478.3 | -413.2 | -2454 / +432 / **+3071** | 434k |

The 0.29 rad/s tilt-rate gap through the ~245 mm lever is a ~71 mm/s platform-x velocity gap. `_seam_velocity` averages the two sides, which splits the gap into knot-acceleration jumps of ~2.5k mm/s^2 at `k_s-1` and ~3.3k at `k_s+1`; ~2.4k mm/s^2 of leg delta-a over one 6.25 ms sub-step x 1.05 gives 400-470k against 200k. The cup acceleration matches exactly at the seam (QP seed rows): the step is in the tilt-borne channels only. Without the re-derivation the jump would sit at `k_s` at ~10-11k mm/s^2; the averaging moves the peak to `k_s+1`. Offline: hop stop REST 20 `LIMIT_JERK` at t = 0.075 (333-462k, legs 4/5) plus 4 `CUP_CONTACT_ACC` (the dive); re-aims 21 at t = 0.075 (404-470k). A probe-only C1 tilt seam removed every t = 0.075 refusal.

**4. Why columns passes.** On columns (sep 125, vertical throws) the tilt rate at the splice knot is ~0 and the seam delta-V is the known 1.7-2.2 mm/s re-derivation baseline; offline columns re-aims pass 44/88 with no seam-class refusal. The columns refusals are the segment's own gate on the larger x/time moves. That, not the learner, is why refusals nearly vanish when the pattern switches from hop to columns.

**5. The one t = 0.025 case (218661, leg 0).** A REST 1.4 s after the REST install at a fresh-origin `reload`, no BUMP. It is the `k_s-1` straddle that I-PLAN-10's widened range now measures and the old range never did, so that jerk flew unmeasured before. Exact mechanism not traced (candidate: the same tilt-borne re-derivation across a stop-then-slew kink at `k_s` = 65, pre-tilt REST hold 9.456 deg).

**6. Is the hand involved?** Seat time does not feed `k_s` or the REST lead: a re-aim `k_s` is the RESEND dispatch (0.55-0.63 s before landing) + `LEAD_S`; the stop-REST `k_s` is the MISSED_CATCH verdict instant (+0.39 s, fired ~+0.43 s) + `LEAD_S`. Both stops followed "cup EMPTY on all 112 valid samples", a real miss. The head's transit banking rate is flat at 0.27-0.30 rad/s across [-0.6, -0.5] and [+0.3, +0.5] s, so offline every transit offset refuses at 333-470k with an ideal hand.

**7. Learner: old hand vs new hand** (939-row `temp/learn/jugglebot/memory.csv`; 47 rows from 10-19 on the PLA hand, 46 from 10-21 on the ASA hand). Row error e = y - u (mm), by site:

| site x0 | day | n | ex mean (sd) | ey mean (sd) | apex err mean (sd) | caught |
|---|---|---|---|---|---|---|
| -0.125 | 10-19 | 6 | +6.9 (20.0) | +24.4 (19.6) | 67.9 (8.1) | 6/6 |
| -0.125 | 10-21 | 6 | -16.3 (14.7) | -1.7 (15.9) | 69.5 (16.4) | 6/6 |
| -0.0625 | 10-19 | 19 | -4.0 (20.8) | +20.1 (27.0) | 76.9 (10.1) | 14/19 |
| -0.0625 | 10-21 | 18 | +13.2 (31.0) | +8.5 (20.9) | 79.4 (15.7) | 16/18 |
| +0.0625 | 10-19 | 12 | -5.3 (23.7) | +1.9 (26.0) | 78.6 (5.3) | 8/12 |
| +0.0625 | 10-21 | 16 | +9.2 (16.1) | +19.7 (12.6) | 82.1 (21.7) | 16/16 |
| +0.125 | 10-19 | 6 | +9.8 (16.2) | +18.9 (26.6) | 71.0 (4.8) | 6/6 |
| +0.125 | 10-21 | 6 | -7.7 (27.4) | +10.2 (25.4) | 101.4 (16.7) | 6/6 |
| all | 10-19 | 47 | | | 75.2 (9.0) | 38/47 = 81 % |
| all | 10-21 | 46 | | | 81.9 (19.6) | 44/46 = 96 % |

Lateral bias flips sign at all four sites but with per-site Welch p of 0.05-0.57 at n = 6-19, so scatter, not a plant shift. The one clear statistical signature of the new hand is apex SCATTER: sd 9.0 -> 19.6 mm (Levene p = 0.0026), a release effect. Radial landing error is unchanged (10-14 28.4, 10-15 26.0, 10-19 30.8, 10-21 29.6 mm); first half 30.4 vs second half 28.8 (Mann-Whitney p = 0.58): no convergence across the sitting.

Did the corrections move? Commanded offset u (mm, x/y) over the sitting:

| site | start | end | note |
|---|---|---|---|
| -0.125 | (-10.7, -20.0) | (-4.6, -15.0) | 6 and 5 mm toward zero over 3 visits |
| +0.125 | (-1.1, -12.7) | (-1.6, -11.5) | flat |
| -0.0625 | (+1.1, -19.1), a miss | (-13.6, -10.6) | x moved 17 mm, y 8 mm, mostly in the last ~20 throws |
| +0.0625 | (+2.1, -4.7) | (-6.9, -15.7) | x 9, y 11 mm, in the last ~20 throws |

Start values equal the END of 10-19, so the old memory seeded tonight. Replay of `learner.command` on the memory prefix reproduces the logged u. Both row-misses (landed (-53,-26) and (+40,+6), radial 59 and 40 mm vs a 27 mm caught median) were at x0 = -0.0625 BEFORE the correction moved; after it moved, 17 of 17 caught but with no smaller landing error. At the attempt level (which the learner never sees) the inflection tracks the planner accepting re-aims and the feed catch starting to work.

**8. Catch quality** (probe over three bags, medians [IQR]; Mann-Whitney 10-19 vs 10-21):

| metric | src | 10-13 PLA (best) | 10-19 PLA | 10-21 ASA | p |
|---|---|---|---|---|---|
| contact - arrival (ms) | self | +3 [1,11] n62 | 0 [-4,9] n13 | +15 [5,20] n37 | 0.002 |
| contact - arrival (ms) | BB | - | +9 [7,11] n6 | +14 [12,15] n11 | 0.005 |
| seat - arrival (ms) | self | 154 [102,227] | 68 [29,143] | 110 [57,182] | 0.17 |
| contact -> seat settle (ms) | BB | (n3) 80 | 119 [95,127] | 252 [237,318] | 0.002 |
| contact -> seat settle (ms) | self | 140 [59,231] | 40 [21,110] | 81 [30,171] | 0.49 |
| cup speed at contact (m/s) | self | -1.43 | -1.49 | -1.51 | 0.72 |
| cup speed at contact (m/s) | BB | -1.57 | -1.56 | -1.52 | 0.22 |
| hand lag at arrival (mm) | self | 13.4 | 2.1 | 13.8 | <1e-4 |
| hand tracking delay on descents | all | 10.9 ms | 4.6 ms | 11.9 ms | - |

Logged seat time (against `t_land_scheduled`): mean +0.175 s / median +0.160 (n = 51) vs 10-19 mean +0.125 / median +0.100 (n = 19); CATCH-EVIDENCE-ABSTAIN 57 vs 20; no `CUP_SEATING` lines either night. About 40 ms of every self-throw "seat" is structural: `t_land_scheduled` uses the COMMANDED apex and the learner commands the apex ~80 mm low. Within each session, smoothness (seat - arrival) shows no correlation with landing error (pooled rho -0.07, p = 0.48) or catch-aim timing error (rho -0.10, p = 0.30); the cup speed at contact has pooled rho +0.28 (p = 0.002). Median cruise-upstroke iq rose 2.6 / 3.1 / 4.0 A (10-13 / 10-19 / 10-21), weak support for the stiffer-slide report.

**9. The oval refusal.** `oval_1ball` at 23:01:20: "the live dwell is 0.27 s but the box covering this request was swept at dwell_s=0.65 s". 0.27 is the node default; `tests/hardware/session_oval_rung1.md` row 12a told the operator to `ros2 param set /skill_node dwell_s 0.65` and it was not done; the GUI showed dwell read-only. Closed by `2026-10-21-juggle-goal-carries-dwell.md` (dwell is now a goal field, GUI-defaulted per pattern from the box).

## Discussion

**(a) Hypothesis withdrawn: "the new cup caused the refusals."** The brief opened here, and it was the natural reading: new hand, owner reports rougher catches, refusals suddenly in the log. It does not survive the evidence. The refused knot and peak are the same as on 2026-10-14 and 2026-10-19 14:24 (PLA cup), the offline reproduction needs no hand (an ideal hand refuses at 333-470k on every transit offset), and the quantity is set by the head's banking rate, which seat time cannot reach. The cup can still matter for catch quality (item 8); it is not in the refusal path. A second withdrawn framing: the three t = 0.075 refusals looked like a new defect in 95d148cd's `k_s-2` range. They are the old k_s+1 refusal on a longer clock; only the t = 0.025 REST is a newly MEASURED jerk, and keeping the `k_s-2` range is correct (do not narrow it to recover that one attempt).

**(b) Tradeoff accepted: the learner is NOT reset** (owner, 2026-10-22: "Don't reset"). Against: apex scatter doubled (sd 9.0 -> 19.6 mm, p = 0.003) and the old rows were taken on another cup. For: the lateral axes show no significant shift (per-site p 0.05-0.57); recency already replaces old rows, because the query takes the 16 most recent support rows, so after ~16 throws per site the old hand's rows have left the window; a cold memory is WORSE, since the identity prior needs 10-20 mm lateral corrections at exactly the site where the plant needs them (the lateral authority clip is 20 mm), costing ~16 throws per site of a known 20-30 mm worse landing; and landing error is unchanged, so no measured error exists for a reset to remove. The learner also cannot see catch-side effects by construction (`caught` is stored but never enters the query). Pre-registered revisit criterion: at the next sitting with >= 20 new-hand rows per site, if the new-hand y - u mean at any site differs from the old by > 15 mm with p < 0.01 (or the apex mean by > 30 mm), quarantine the old rows at that site (backup first, then move rows with `t_abs_s < 1792580000` to a `memory_quarantine_<date>.csv`, as 2026-10-13 did). A per-hardware-revision `plant_id` key is the contract-shaped option for a change that moves release (a different hand or friction), not for a tolerance-level cup swap.

**(c) Why I-PLAN-11 pins the rate inside the schedule's widen loop, not as a post-hoc blend.** The cheap probe was a post-hoc blend of the tilt series toward the head's rate (`FIXRATE`). It removed every t = 0.075 refusal, but the QP never sees the blended tilt-shift profile, so it relocated the step: the crude blend adds a `LIMIT_ACC` at t = 0.025. Putting `start_tilt_rate` into `_accel_bounded_schedule` makes the carried rate a boundary condition of the same smoother whose measure-and-widen loop already scores tilt accel / jerk / leg jerk, so the turn from the head's rate onto the segment's banking solution is bounded by the same guarantee as every other pin gap, and `validate_cycle` stays the authority. Control walk-through, one 40 Hz cycle at the seam: before, the Teensy interpolates Hermite segments between knots from (pose, pose_vel), so a knot-velocity inconsistency is an acceleration STEP (475 -> ~440 -> 404 mm/s over two knots against curves that do not bend that way, dA +2474 / -474 / -3277 mm/s^2). After, the knot-0 anchor of the banking blend is a ramp (v0 + r0 t) instead of a constant; the smoothstep has W' = W'' = 0 at its pin, so the output's first derivative at knot 0 is exactly r0; the remaining discontinuity is the tilt ACCELERATION step (~0.5 rad/s^2 x lever ~ 120 mm/s^2, ~20k mm/s^3), a C2 residue and not a relocated C1 step. The ramp target is projected onto the 12 deg cap disc, so the cap-by-convexity argument survives. Not carried: post-release seeds (the release tilt rate is the throw's own slew) and the zero-banking branches (held / rest_slew / legacy), which open from a static attitude.

**(d) The second class: diagnosed, hypothesis WITHDRAWN, fixed (2026-10-22, same day; this is a trigger-(a) withdrawal).** With the rate pinned, 30 re-aim `LIMIT_JERK` refusals (200-265k, t = 0.175-0.325 s, segment knots 7-13) and 21 `HAND_LIMIT_C2` (3.94-4.9 rev/s^2 vs 3.9) remained offline; none at the seam, all the new segment's OWN gate (the C2 count rose 1 -> 21 because the pin removed the `LIMIT_JERK` that `validate_cycle` checks first: unmasked, not created). The earlier hypothesis here was that the knot-0 blend floor (8 knots) was too short and the widen loop did not score hand C2. **Refuted**: every refused re-aim's knot-0 window ran at L = 10-11 knots, well above the floor, and 11 is `blend_max`, the narrowest anchor gap anywhere in the window: the catch-to-release gap (anchors [0, 16, 27] at lead 0.63). The widen loop does not "pass while the hand fails": it fails its own tilt-accel score (1.24-1.81) in every case, accepted ones included, so it is saturated and adding hand C2 to its score could not change its answer. The 0.56-passes / 0.63-fails split of the unchanged aim is a margin artefact (C2 3.83 vs 4.29 against 3.9), not a lead threshold. The binding constraint: `_accel_bounded_schedule` gave every anchor ONE blend width, so the knot-0 pin (13-16 knots before the catch pin) was clamped to 11 and its handover from the head's ramp was forced to finish at knots 10-11, on top of the banking's swing into the cap-saturated dive (raw -12 deg, knots 11-16). The re-solved cup track matched the head's to < 1 mm; the tilt `ry` overshot the receive pin (-4.68 deg, then back to -3.94; the head approached monotonically) with tilt jerk 122 rad/s^3 vs the head's 56. With central-difference knot velocities a Hermite knot-acceleration jump is about 2h x tilt jerk (2 x 0.025 x 117 = 5.9 predicted, 5.48 measured), which the gate reads as 135-265k leg jerk and, through the slider drop term, as 3.9-5.9 rev/s^2 hand C2: one defect, two codes.

Options weighed. (a) CHOSEN: the knot-0 pin carrying a non-zero rate gets its own blend width, the whole gap to its next pin. It is one gated function, leaves the seam by construction (the smoothstep's W0 = 1, W0' = W0'' = 0 at knot 0 holds for any width), keeps every rest seam and columns bit-identical, and measured 0 regressions over 161 hop rows. A variant (V2: also retarget the knot-0 anchor at the head's own tilt track) scored 68/80 vs 66/80 and was NOT taken: two more re-aims for a second construction change in a gated function. (b) A lead floor / earlier refusal: the data does not support one (the unchanged aim passed at 0.55-0.57 AND 0.7-0.9, failed at 0.60-0.63), so it would refuse feasible re-aims; reduced to a report label, "refused by the segment's own gate, not the seam", owed to the executor/report owner. (c) A BUMP-style overlay for the hop: closes the class by never re-solving, but the 50 mm/s launch bound is against the hop's own ~0.3 m/s release (every bump would be `BUMP_LAUNCH`) and the carry-velocity ramp adds an unmodelled 0.044-0.059 g lateral acceleration under the seated ball = 2.5-3.4 deg of the 4 deg bank. Not unsafe in principle, but design work the measured class no longer needs; revisit only if hardware shows re-aims of more than ~20 mm late in the transit are common. Residual after (a): 14 of 80 = 8 QP-INFEASIBLE 35 mm moves (the honest limit of re-planning the remaining transit) + 6 borderline hand C2 (3.91-5.0) on the 10 mm short / 40 ms late move.

**(e) Catch smoothness: instrument and two checks before any catch learner.** The measurement does not support a learner change. Landing error and catch-aim timing error (about +-10 ms) do not track smoothness in any of the three sessions (|rho| <= 0.23 within a session), so folding smoothness into the aim learner (option b) cannot buy it at today's 27 mm scatter, and a per-site catch-timing memory (option a) would learn noise. What changed is the BB-feed settle: 119 -> 252 ms (p = 0.002) at an unchanged cup speed and a timing only +5 ms later, which reads as cup mechanics (restitution, seat profile), not a gain a learner can learn. Monday is also an unusual baseline: its hand tracking delay was 4.6 ms vs 11-12 ms on 10-13 and 10-21 (cause unknown), so comparing against Monday overstates the cup. Decision (owner, 2026-10-22): put contact - arrival and contact -> seat settle on the throw line and as a passive memory column (not in the query), stop reading `seat` against `t_land_scheduled` (it carries the learner's ~40 ms apex bias), and run two checks: (1) the owner measures the new cup's seat and rim height against the PLA cup (a drop of 20 mm or more would explain the +15 ms contact as a geometry constant: 15 ms x 2.5 m/s closing ~ 35 mm); (2) a catch-time offset A/B on BB feeds, a debug delta-t applied at DISPATCH (never as a re-send, which tonight's `HAND_LIMIT_C2` refusals show is the risky regime) at -20 / 0 / +20 ms, 10 feeds per arm, pre-registered: a median settle move of more than 60 ms between any two arms means timing is a lever. The owner asked whether a second, CATCH learner should exist. Answer: yes as the end state (two learners, a throw learner keyed on release/landing and a catch learner keyed on contact timing, each with its own memory and its own safety envelope), but only after check 2 qualifies the lever; a catch learner whose lever has not been shown to move settle would learn noise and add a second regime of per-catch re-solves at the seam. If the offset does not move settle, the cause is mechanical and the fix is the cup, with the learner left on the throw.

## Fix

**I-PLAN-11, "a splice seed is C1 in every realised channel"** (written first as an `INVARIANTS.md` row after I-PLAN-10: the new window's realised `pose_vel` and `hand_vel` at knot 0 equal the head's at `k_s` within the `_seam_velocity` baseline, ~2 mm/s and ~0.014 rad/s; enforcement and tests named in the row; Fate NEW 2026-10-21, NOT FLOWN). Three code pieces, from the I-PLAN-11 handoff:

1. `cup_realize.py` (a gated file): `_accel_bounded_schedule(..., start_rate=None)` builds a per-anchor target series, with the knot-0 target a ramp `v0 + r0 t` projected on the cap disc, and the blend uses `targets[j]`; `start_rate` None or zero is bit-identical (tested). `tilt_schedule(..., start_tilt_rate=None)` validates (requires `start_tilt`, else `ValueError`) and passes the rate to the banking branch only; the zero-banking branches ignore it.
2. `unified_cycle.py` (a gated file): `_realize` takes `start_tilt_rate = state.pose_vel[3:5]` for every non-post-release seed with a `start_tilt`, mapped into the gravity frame by differencing `_tilt_to_gravity` over 1 ms, and passes it to `cr.tilt_schedule`. Rest seams carry rate 0 (bit-identical); any chain install spliced mid-motion now carries the head's rate.
3. `tests/motion/test_splice_tilt_rate_seam.py` (new, 11 tests): hop stop RESTs at three transit offsets accepted; an unchanged-aim mid-transit re-aim accepted with a baseline seam (knot-acc jump <= 1000 mm/s^2); the segment opens at the head's tilt rate (|dV tilt| <= 0.02 rad/s, |dV xy| <= 5 mm/s, asserting the head rate > 0.2 rad/s); a regression guard that a value-pin-only seam refuses `LIMIT_JERK` > 400k at t = 0.075; a zero start rate is bit-identical to the value pin. Seam bounds are set against measured 3.1 mm/s / 0.0136 rad/s / 465 mm/s^2.

Two gated files changed, so the admissible box was re-swept (`config/generated/admissible_box.yaml`, `gate_hash` '1be1b456acc7' -> 'c1278bac6251' at the working tree the handoff measured). The sitting's learner rows (`temp/learn/jugglebot/memory.csv`, +46 rows) are committed with this entry, with no reset (item (b)). The dwell fix landed separately (`2026-10-21-juggle-goal-carries-dwell.md`). Not done by choice: the catch instrumentation (item (e)); no hop-RESEND stopgap gate was added, since the pin makes the stop RESTs pass.

4. **Second-class fix (2026-10-22, `cup_realize.py::_accel_bounded_schedule`, a gated file, on top of piece 1):** a NON-ZERO `start_rate` gives the knot-0 anchor its own blend width, `L0_carried` = the gap to the next anchor (the blend loop uses `Lj = max(L, L0_carried)` for anchor 0), instead of the shared `blend_max` width capped by the narrowest gap anywhere in the window. A zero rate or None keeps the shared width, bit-identically. Five tests appended to `test_splice_tilt_rate_seam.py` (now 16): `test_a_mid_transit_reaim_passes_its_own_segment_gate` (4 cases) and `test_a_carried_rate_gives_the_knot0_window_the_whole_gap`. The admissible-box `gate_hash` changes AGAIN (cup_realize.py edited after the sweep the handoff measured), so the box re-sweep is owed on the final tree. 40 Hz walk at knots 7-8 of the c0 0.63 segment: velocity-feedforward slope change 1312 -> 941 mm/s^2 (x), 5.48 -> 3.92 rad/s^2 (ry), hand torque-feedforward step 4.29 -> 2.81 rev/s^2-equivalent; a refused re-aim leaves the old record streaming.

## Verification

Triples are (date, command, result).

| date | command | result |
|---|---|---|
| 2026-10-21 | `pytest tests/motion/test_splice_tilt_rate_seam.py -q -p no:cacheprovider` on a `git archive` of 7ffa7a22 + the new test file (BEFORE) | 9 failed, 2 passed (the 2 guards pass in both by construction) |
| 2026-10-21 | same command in the worktree (AFTER) | 11 passed in 2.76 s |
| 2026-10-21 | `pytest tests/motion/test_unified_cycle_splice.py tests/motion/test_skills_bump.py tests/motion/test_splice_tilt_rate_seam.py -q -p no:cacheprovider` | 46 passed in 16.95 s |
| 2026-10-21 | `pytest tests/motion -q -p no:cacheprovider -n 4 --deselect <box-hash test>` | 2547 passed, 2 skipped in 141 s; the box-hash test fails until the re-sweep (expected, two gated files changed) |
| 2026-10-21 | `probe_hop_splice.py` on HEAD (hop 250 / 0.9 / 0.27), BEFORE | 0/101: stop REST 0/21 (17 `LIMIT_JERK` at t = 0.075, 4 `CUP_CONTACT_ACC`), re-aims 0/80 (71 `LIMIT_JERK`, 8 `INFEASIBLE`, 1 `HAND_LIMIT_C2`) |
| 2026-10-21 | same probe with I-PLAN-11 | 38/101: stop REST 17/21 (the 4 `CUP_CONTACT_ACC` are the dive, out of scope), re-aims 21/80 (30 segment-own `LIMIT_JERK` t = 0.175-0.325, 21 `HAND_LIMIT_C2`, 8 `INFEASIBLE`); zero t = 0.075 refusals |
| 2026-10-21 | `TRK=1 PAT=columns SEP=125 APEX=0.95 DWELL=0.27 python probe_hop_splice.py` HEAD vs I-PLAN-11 | 44/88 both, identical row for row |
| 2026-10-21 | `python sim/skills_gate.py --learn --pattern hop --separation-mm 250 --apex-m 0.9 --dwell-s 0.27 --seeds 0 --target-throws 12` before and after | PASS 12/12, 0 bumps, both (the sim gate does not exercise re-aims or stops, which is why it never saw this class) |
| 2026-10-22 | `pytest tests/motion/test_splice_tilt_rate_seam.py tests/motion/test_unified_cycle_splice.py -q -p no:cacheprovider` (after the per-pin width + 5 tests) | 33 passed in 1.50 s |
| 2026-10-22 | same, in-process patch `L0_carried = None`, `-k 'own_segment_gate or whole_gap'` (revert check) | 5 failed |
| 2026-10-22 | `pytest tests/motion -q -p no:cacheprovider -n 4 --deselect <box-hash test>` | 2558 passed, 2 skipped, 1 failed: `test_motor_guard.py::test_ipc_latency` (nightly+serial wall clock, run under 3 concurrent probes); rerun alone: 1 passed |
| 2026-10-22 | `python sim/skills_gate.py --learn --pattern hop --separation-mm 250 --apex-m 0.9 --dwell-s 0.27 --seeds 0 --target-throws 12` | PASS 12/12, attempts 3, drops 0, 0 bumps |
| 2026-10-22 | `python scratchpad/hand/proto_a.py` on the landed tree (hop 250 / 0.9 / 0.27, 80 re-aims + 21 stop RESTs) | re-aims 66/80 (was 21), stop REST 17/21 (unchanged); `LEADS=0.9,0.8,0.7` 53/60 (was 51); 0 row-level regressions vs the I-PLAN-11 tree |
| 2026-10-22 | `PAT=columns SEP=125 APEX=0.95 DWELL=0.27 TRK=1 python scratchpad/hand/proto_a.py` | 40/80 + 4/8, identical row for row |
| 2026-10-22 | per-case segment gate, `probe_second_class.py` before vs after | peaks 135-265k -> 97-127k, hand C2 3.8-5.9 -> 2.5-2.95; ry overshoot -4.68 -> -4.32 deg |
| 2026-10-22 | box sweep: `bash temp/sweeps/sweep_P1.sh` (four families x 2 runs, `tools/admissible_sweep.py` at 350/5000/200000/3900, dwell 0.27 single/hop/columns + 0.65 oval, BLAS one thread, gated files frozen for the run; started once both gated edits were final) | single/hop/columns/oval all IDENTICAL across the two runs; merged box content-identical to the committed one (20 boxes, same limits); `gate_hash` 1be1b456acc7 -> 21e873c3bd22, `swept_at` 2026-10-22; `pytest tests/motion/test_skills_admissible.py -q` 85 passed |
| 2026-10-22 | `./run_tests.sh --full` (every tier, after the last edit; log `temp/logs/gate_iplan11_20261022.log`) | parallel phase 7017 passed, 9 skipped in 313.53 s; serial tail 7 passed in 27.74 s; exit 0 |
| 2026-10-22 | commit on `skill-stack` with the `Logbook-Entry` trailer (the canonical link; no SHA here by convention) | pushed to origin/skill-stack; both packages colcon-built |

Seam numbers with the pin (proto probe): delta-V (segment knot 0 - head) 1.4-3.1 mm/s, ry 0.006-0.0136 rad/s, hand 0.001-0.004 rev/s (without: 60-71 mm/s, 0.29 rad/s, 0.02-0.077 rev/s); installed pose-x knot-acc jumps at `k_s-1..k_s+1` <= 465 mm/s^2 (was 2.2k-3.3k); joined peak leg jerk 107-119k (was 428-470k at t = 0.075). Hardware: none yet.

## Outcome

FLOWN 2026-10-22, PASSED (rehearsal row 48; launch log `/home/jetson/.ros/log/2026-10-22-19-50-38-975040-jetson-50159/launch.log`). I-PLAN-11 flew on the 964bd409 build: 3 hop goals, 12/12 caught, zero `LIMIT_JERK` at `t=0.075 / 0.050 / 0.025` anywhere in the log, so the old mid-transit seam class is gone (2026-10-21 had 12 of 13 refusals there). Hop re-aims 11 accepted / 7 refused (5 `SPLICE_TOO_LATE`, 1 `HAND_LIMIT_C2`, 1 infeasible). The stop-REST half was not exercised (no hop ended early). Six `bump declined` `LIMIT_JERK` 217-355k (legs 4/5, `t=0.1-0.225`) were graceful declines; 0 `CHAIN_DISCONTINUITY`. One new single refusal: 377161 mm/s3 leg 3 at `t=0.425` on a late CATCH re-aim whose fit moved the landing 31.5 mm (L3368), cost one throw: residue, not the old class. Row 49 and the oval at dwell 0.65 flew in the same arc (see `2026-10-23-oval-rung1-flown-and-rows-48-49`).

Residue, not scheduled (carried forward from the pre-flight Outcome):

- The second-class residual (item (d), after the per-pin width): 14 of 80 offline re-aims = 8 QP-INFEASIBLE 35 mm moves (the honest limit) + 6 borderline hand C2 (3.91-5.0) on the 10 mm short / 40 ms late move. Accepted as is.
- Owed to the executor/report owner: a report label "re-aim refused by the segment's own gate, not the seam" (executor.py / report.py were off limits to the second-class unit).
- The V2 variant (also retarget the knot-0 anchor at the head's tilt track, 68/80 vs the landed 66/80) was not taken; the BUMP overlay for the hop (option (c)) was rejected for now (launch bound and unmodelled carry banking, see Discussion (d)).
- The box re-sweep is owed on the final tree (cup_realize.py changed after the sweep this entry's Fix measured).
- The 4 `CUP_CONTACT_ACC` stop-REST dives (one per catch at -0.4 s 'before'); out of scope here.
- The 218661 t = 0.025 `reload`-REST straddle: mechanism untraced.
- Monday's (10-19) unusually tight hand tracking (4.6 ms vs 11-12 ms) is unexplained.
- The new cup's seat and rim height measurement is RETIRED 2026-10-23 (owner: the ASA hand is geometrically identical to the PLA hand).
- The sim `--reload` gate LIMIT_JERK regression (bisected to 08ec3fe6, 2026-09-30) is unchanged and not this class.
