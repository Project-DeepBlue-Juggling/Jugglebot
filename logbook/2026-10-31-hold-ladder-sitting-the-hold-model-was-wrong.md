---
title: "Hold-ladder Sitting 2026-10-31: the pre-release hold model was wrong (shorter hold lands LONG, a Platform ring, not tracking delay); hold 0 becomes the default"
type: investigation
date: 2026-10-31
status: complete
phase: "two-ball-oval P4.0 - pre-release hold rung, Sitting results"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-30-sitting-prep-columns-check-hold-ladder-and-apex-cells.md
  - 2026-09-18-opening-rest-homes-the-hand.md
files_changed:
  - config/generate_config.py
  - config/generated/admissible_box.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - config/hardware_config.yaml
  - docs/project-map.md
  - docs/runbook.md
  - logbook/2026-10-30-sitting-prep-columns-check-hold-ladder-and-apex-cells.md
  - logbook/2026-10-31-pattern-progression-p4-high-low-columns.md
  - logbook/INDEX.md
  - map/bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid.md
  - map/feature-pre-release-hold-rung-release-from-a-moving-platform-hold-0.md
  - mapkit/sitting.py
  - plans/active/two-ball-oval.md
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - sim/juggle_tilt.py
  - sim/model/generate_mjcf.py
  - sim/model/jugglebot.xml
  - sim/plant/mujoco_plant.py
  - tests/hardware/session_oval_rung1.md
  - tests/hardware/session_skills_r4_diag.md
  - tests/motion/test_cup_realize.py
  - tests/motion/test_hold_rung_bias_probe.py
  - tests/motion/test_magazine_throw.py
  - tests/motion/test_pose_planner.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_skills_bump.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_schedule.py
  - tests/motion/test_skills_segments.py
  - tests/motion/test_unified_cycle.py
  - tests/motion/test_validate_cycle.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_trajectory_node.py
  - tests/sim/test_hand.py
  - tests/sim/test_mjcf_drift.py
  - tools/probes/hold_rung_bias.py
  - map/bug-admissible-box-gate-passes-a-cell-whose-solve-exceeds-hand.md
  - map/bug-executor-re-splices-a-rest-that-is-already-live-splice.md
  - map/bug-two-ball-columns-hop-start-throw-lands-25-56-mm-inboard-past.md
subsystem:
  - skill-stack
  - platform
  - hand
tags:
  - hardware
  - withdrawn-hypothesis
---

## Summary

Runbook § 11 flew on 2026-10-31 (second launch of the day; launch `~/.ros/log/2026-10-31-13-14-00-526910-jetson-1123157`, bag `~/Desktop/rosbags/2026-10-31_13-14-00`, bridge FW 34 / PROTOCOL 10, planner `pose`, Hand acc limit 3900 rev/s^2). The first launch of the day ended when a motor cable was bumped (`SPINOUT_DETECTED`); the encoders were re-calibrated with ROS2 down and the second launch ran clean from there.

The ladder's pre-registered prediction had the wrong sign. The prep model (C2-C4 of the 10-30 entry) said the hold protects against Platform tracking delay, so a shorter hold would land the ball SHORT (hold 0.050 about +44 mm short, hold 0 about +100 mm short, dropped balls). Measured: a shorter hold lands the ball LONG, monotone in the hold, and nothing dropped. Hold 0 flew 30/30 with the tightest landing spread of the Sitting (9.8 mm), the owner called it the smoothest pattern, and leg acceleration fell 44 percent. The owner set the `pre_release_hold_s` default to 0.000.

## Goals flown (dx_short POSITIVE = landed short of the commanded landing, NEGATIVE = long; dapex = y2 - u2 plant residual)

| # | start | pattern | hold | dwell | apex | caught | dx_short (mm) | dapex (mm) | status |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 13:14:36 | columns | 0.100 | 0.27 | 0.95 | 0/0 | - | - | ENDED MISSED_CATCH (CATCH of Ball 2, skill 3) |
| 2 | 13:14:50 | columns (fed) | 0.100 | 0.27 | 0.95 | 4/4 | -9.1 +-9.8 | +95.3 | ENDED MISSED_CATCH (Ball 1, skill 6) |
| 3 | 13:15:24 | columns | 0.100 | 0.27 | 0.95 | 0/0 | - | - | ENDED MISSED_CATCH (Ball 2, skill 3) |
| 4 | 13:15:36 | columns (fed) | 0.100 | 0.27 | 0.95 | 12/13 | -7.5 +-12.4 | +77.6 | ENDED HAND_LIMIT_ACC 4112.5 > 3900 at the CATCH |
| 5 | 13:16:38 | oval_1ball_fed | 0.100 | 0.45 | 0.95 | 2/2 | -4.2 +-8.6 | +88.4 | ENDED MISSED_CATCH (cup empty, ball never seen after release) |
| 6 | 13:16:55 | oval_1ball_fed | 0.100 | 0.45 | 0.95 | 6/6 | +13.7 +-13.1 | +86.7 | done |
| 7 | 13:17:40 | oval_1ball (CONTROL) | 0.100 | 0.45 | 0.95 | 30/30 | +9.9 +-14.1 | +89.8 | done |
| 8 | 13:19:04 | oval_1ball | 0.050 | 0.45 | 0.95 | 30/30 | -30.9 +-14.1 | +80.7 | done |
| 9 | 13:20:23 | oval_1ball | 0.050 | 0.45 | 0.95 | 30/30 | -28.2 +-15.6 | +91.2 | done |
| 10 | 13:21:40 | oval_1ball | 0.075 | 0.45 | 0.95 | 6/6 | -12.3 +-32.7 | +74.5 | ENDED LANE_REFUSED |
| 11 | 13:22:07 | oval_1ball | 0.075 | 0.45 | 0.95 | 30/30 | -6.7 +-21.3 | +75.5 | done |
| 12 | 13:23:47 | oval_1ball | 0.100 | 0.50 | 1.00 | 18/18 | +3.0 +-22.2 | +72.2 | ENDED LANE_REFUSED |
| 13 | 13:24:40 | oval_1ball (fed) | 0.100 | 0.50 | 1.00 | 0/0 | - | - | ENDED SPLICE_TOO_LATE |
| 14 | 13:24:52 | oval_1ball (fed) | 0.100 | 0.50 | 1.00 | 30/30 | +2.8 +-18.7 | +76.9 | done |
| 15 | 13:25:52 | oval_1ball | 0.050 | 0.50 | 1.05 | 30/30 | -17.8 +-13.8 | +72.6 | done |
| 16 | 13:26:52 | oval_1ball | 0.000 | 0.50 | 1.05 | 30/30 | -46.2 +-10.0 | +73.7 | done |

Probe verdict (`tools/probes/hold_rung_bias.py` output JSON `temp/probes/hold_ladder_2026-10-31.json`): control +9.9 mm; rung 0.075 Delta -17.3 mm (SE 4.6); rung 0.050 Delta -39.5 mm (SE 3.2); hold 0 against 0.050 at the same 1.05/0.50 cell, a further 28.4 mm LONG.

## Discussion

**Hypothesis WITHDRAWN: "the Platform tracks late (45 ms delay + 30 ms lag), so the ball separates late and lands short; the hold buys the carrier time to settle."** Each part fails against the bag (analysis `hold_sign`, bag `2026-10-31_13-14-00`):

- **Sign.** The pre-registered rule predicted SHORT for every shorter hold. Observed LONG, with delta_apex and delta_y inside the 10 mm bars, so it is a real along-throw shift, not a drifting cell.
- **Tracking delay is 15 ms, not 75.** A pure-delay fit of the Platform (mocap `Platform` body) to the FK of `/leg_cmd_executed` gives 15 ms in all seven analysed goals (xy rms 3.3-4.6 mm, tilt rms 0.10 deg). The executed command itself runs about 10-20 ms behind the plan clock, comparable to the ball's own separation lag.
- **Separation does not depend on the hold.** The cup-vz versus ball-vz crossing falls +21..+33 ms after the announced release in every rung, and the log's own `release` metric reads +4..+6 ms in every rung. So candidate (a) "early separation" is ruled out, and the ball is late, not early.
- **Composition sign is right.** The commanded cup lateral velocity at the commanded release equals the announced take-off velocity within 4-15 mm/s in every goal. Candidate (b), a backwards composition term, is ruled out; the hold-0.100 control lands short, so the hold was not hiding one.
- **The carrier is not driven to zero.** Under `pose` the commanded centroid velocity along the throw axis at release is about +40 mm/s at every hold (the commanded tilt, 1.3-2.1 deg, is shallower than the launch angle, about 2.5 deg), so "the hold drives the carrier to about 0" is not true in this planner mode.

**What the bag showed instead.** The Platform tracks the command, then rings. The carry deceleration (command -640 mm/s at -250 ms to 0 at -100 ms) excites an underdamped lateral mode at roughly 6 Hz (zeta about 0.15-0.2, decaying about 0.5-0.65 per half cycle). At hold 0.100 the measured Platform overshoots to +94 mm/s forward while the command is stationary, then swings back; its trough lands on the release, so the ball is SHORT (control +9.9 mm). At hold 0 the deceleration runs through the release and the Platform is still moving forward faster than commanded when the ball leaves (+20 mm/s centroid error at +27 ms), so the ball is LONG. The hold only moves the ring phase at separation: tau = t_sep - t_decel_end, about 125 ms (3/4 period, trough) at hold 0.100 and 0 to 30 ms (first positive lobe) at hold 0.

Measured ball launch-velocity deltas against the announced velocity along the throw axis: hold 0.100 -29 mm/s, 0.075 -8, 0.050 +26, hold 0 +37 (the 1.05 cell: 0.100 -18, 0.050 +5). With a flight of about 0.89 s, 0.050 against 0.100 at the 0.95 cell is +55 mm/s, about 49 mm long (probe residual 39.5 mm), and hold 0 against 0.050 at 1.05 is +32 mm/s, about 29 mm long (probe 28.4 mm).

**The unexplained half of the 0.050 rung.** At the 0.95 cell's 0.050 rung the centroid error at +27 ms is only -11..-14 mm/s, yet the ball error is +26 mm/s; about half of that rung's +55 mm/s shift is unattributed. Tilt error times v_z (+18..+26 mm/s at every rung, absorbed by the learner), the tilt-rate lever term (flips sign by 6-13 mm/s between rungs) and the Hand speed deficit at the 1.05 cell (600-750 mm/s below command, common to all three holds) may cover it, but the per-throw R^2 is poor (centroid alone 0.06; with tilt and tilt rate 0.42) and the two ends of the ladder are clean while the middle is only partial. Confidence: high that (a) and (b) are wrong and that this is a Platform ring, not timing; moderate on the exact decomposition. Not enough to implement a model-based fix.

**dapex +72..+95 mm is hold-independent.** The plant residual y2 - u2 along the apex is +72..+95 mm in every rung (7 percent to 9 percent of the throw), unchanged across holds 0.100 down to 0. It is a plant offset the learner absorbs, not a hold effect; it is why apex is under-delivered by about 7 to 9 cm and a separate item.

**Consequences for JB-102.** Delay-aware composition as specified would predict SHORT, add forward velocity and make throws longer. Do NOT build it. The remaining design options: (1) identify the roughly 6 Hz mode from this bag and compose through it; (2) soften (jerk-limit) the carry deceleration; (3) end the deceleration a whole number of ring periods before release; (4) damp the mode in the leg loop, which reopens the frozen leg gain hunt and is the owner's call.

**What I could not measure.** The planned trajectory is not in the bag, only the executed command; the plan-to-command clock offset is inferred. Cup velocity is reconstructed approximately; raw in-cup ball markers (`/mocap_data`) and measured leg positions were not decoded, so where the 6 Hz mode lives at leg level is unverified.

## The Columns findings (goals 1-4, analysis `failures`)

Columns "49/49, 57/59, 50/50" were runs that got past the cold start; the two-ball start (hop entry) is a separate failure surface.

- **Hop-start inboard landing, RECURRING (JB-121).** The hop throw has no learner row (constant P1 correction, apex 0.877, dy -20/-18 mm). Four hop starts landed 56, 40, 37 and 25 mm inboard in x against a 35 mm inward catch authority (outward 10 mm); goals 1 and 3 ended 0/0. On 10-30 hop starts already ended MISSED_CATCH at 3/3, 8/8 and 3/3; on 10-28 they landed outward (+13, +19 mm). The sign change coincides with `pose` and hold 0.100, which fits the hold analysis above (hold 0.100 lands short), so hold 0 may move it; no hop-start data at hold 0 exists. Goal 2 ended on the +62 site where Ball 1 y scatter was 40-80 mm against a 20 mm y authority.
- **TRACKER-IDENTITY +1.2 s: same ball, next flight, chronic and harmless.** The fit is handed the same ball's next flight (2.07 beats, 1.158 s at beat 0.579 s); the gate is working as designed (it keeps the other ball's fit out of the learner). Counts: 44 on 10-28, 62 on 10-30, 99 on 10-31, present on caught throws too, so it did not end goal 2. Cost: refused throws give the learner no row.
- **HAND_LIMIT_ACC 4112.5 vs the box "inside" (JB-120).** First ever on this cell (zero hits in the 10-28 and 10-30 logs). Skill 13 (the CATCH of Ball 2) was refused at install while `BOX-EXPECTED-LANDING` printed `inside box x[-2.0,+1.0] y[-20.0,+8.0]`; the learner had Ball 2 y pinned at -20.0 mm, the box edge, on every Ball 2 catch. Gate and solver disagree by about 5 percent there. Two later `RESEND-REFUSED` HAND_LIMIT_ACC lines (4049.5, 4134.2) in oval goals were benign re-aims.
- **LANE_REFUSED (goals 10 and 12) = JB-98 again.** Both are firmware STOP `+2` events after a heavy solve: 107-138 ms against a typical 30-60, about 100 involuntary context switches; goal 10 also had a 1198 ms leg-1 heartbeat gap; goal 12 logged `emitter wake gap 48 ms` (the firmware stops the lane past about 70 ms). The singleton STOPs (12 in this log) were absorbed; two inside one window refused the lane. Firmware STOPs per launch: 10-30 = 7, 10-31 = 17. No fix inside the skill layer.
- **SPLICE_TOO_LATE (goal 13), from the slow REST return (JB-122).** After goal 12's LANE_REFUSED the Hand was left held at 8.62 rev; the next goal's opening REST homed it over 6.5 s (peak capped at 2.5 rev/s), the REST solve cost 470 ms (a normal 1.75 s REST: 20-83 ms), was rebased by the fresh-origin rule, and then the Ball Butler announcement swap re-spliced the same REST: solve 323 ms against a fixed 158 ms budget, wire at knot 49, END `SPLICE_TOO_LATE`. The first attempt-ending SPLICE_TOO_LATE since the 10-16 closure; a sibling of the closed handoff class, not the same. Recovery after a pure limit end (goal 4) was already fast; only the post-LANE_REFUSED Hand return is slow.
- **Goal 5 (`oval_1ball_fed`, MISSED_CATCH 2/2):** the third ball went `status 1, tracking 0` and the cup read empty on 134 samples; the Hand released normally and no free marker was seen after release. Cause unknown (ball left the capture volume or rolled out); not a recovery abort.

## The Hand findings (analysis `hand_stroke`)

- **The non-monotonic catch-to-throw velocity is in the plan, not in tracking** (measured feedback follows the command within 10-20 ms; an offline replan with production `plan_segment` reproduces it). Three mechanisms: (1) at hold 0.100 the pause before the throw IS the hold: the rows pin cup v_xy to kappa * v_z (kappa 0.039, about zero) for the last four knots, so the 150 mm sideways carry must finish 100 ms before release and the QP brings the cup to rest at the first held knot (lateral acceleration rises from about 3-4 to 6.6 m/s^2); (2) the early stop at about 2.2 rev and the downward re-acceleration come from the `pose` planner's leg-jerk objective (`pose_leg_jerk_weight` 10.0, `cup_cycle.py` `_pose_augment` block 1), through cup-z times tilt-slew cross terms; (3) at hold 0 the plan descends to the cup-box floor (0.307 rev) and floats up about 9 mm (0.597 rev peak) before the throw.
- **Stroke use.** The bag used 0.27 to 9.78 rev; the top leaves 0.92 rev (30 mm) to the metal and the bottom 0.38 rev (12 mm). The TOP binds first (the plan rides the box top in the post-release deceleration); lowering the floor alone buys nothing. Rule of thumb: +1 mm of runway on the binding side is about -0.6 percent peak Hand acceleration. Apex ceiling at 3900 rev/s^2: 1.05 today, 1.10 with top +10 mm, 1.15 with top +20 mm. A top of +16 mm gives about -8 percent peak acceleration at 1.05 and an apex near 1.13.
- **`cup_realize` clamp model bug.** The slider clamp uses `GEOM_HAND_STROKE_MM` 344.75 mm (a GUI/MuJoCo key from the old 10.8 rev stop) in a frame where rev 0 is slider 20 mm, so it sits at 9.972 rev, 0.73 rev (24 mm) under the metal. Raising the box top past it saturates the slider and zeroes its velocity (`np.where(saturated, 0, ...)`), producing a fake `HAND_LIMIT_ACC` spike (5365-6219 rev/s^2). Masked today because the box top (9.652) and validate ceiling (9.959) are below it; it must move with any top extension. The firmware clip (10.501 rev, 0.2 rev under the metal) stays.

## Owner decisions (2026-10-31)

1. **Hold default 0.100 to 0.000.** Hold 0 flew 30/30 (goal 16), the tightest spread (9.8 mm), the smoothest pattern, leg acceleration down 44 percent, and the hold's premise is false. The learner absorbs the roughly 30 mm LONG bias (+37 mm/s); for two balls it stays systematic as long as every carry has the same deceleration profile.
2. **Hand stroke: top +16 mm** (`hand_catch_prime_rev` 9.9594 to 10.45 rev, box top 10.14 with the 10 mm margin kept, firmware clip 10.501), **bottom box margin 0** (floor 0.307 to 0.0 rev). Owner: "when the hand homes we set the actual end-stop as -0.1 revolutions, so pos=0 is still off the end-stop; the hand approaches the bottom slowly, overrun risk minimal". Shipped as 2 mm, not 0: the validate floor has no tolerance, so a box floor of exactly 0 sat on it and the planner refused HAND_STROKE -0.001..-0.002 rev at 0, 0.5 and 1 mm (`_CUP_Z_BOTTOM_INSET_M = 0.002`; box floor 0.0614 rev, cup z 0.6855 m, `SETTLE_CUP_Z_MM` 685.5, `REST_HAND_REV` 0.0614). Plus the `cup_realize` slider clamp fix.
3. **The REST return of the Hand after a LANE_REFUSED end is a smooth move, not homing.** The ODrive keeps the homed position (no slip), so the opening REST moves the Hand to the rest position at sensible limits instead of the 2.5 rev/s gentle-park cap. The one surviving constraint to check: `2026-09-18-opening-rest-homes-the-hand.md` (a resumed lane moving more than 0.05 rev in its first 100 ms tripped MAX_DEVIATION) against bridge FW 34's arm-edge grace and post-arm output hold.

## Probe fix

`tools/probes/hold_rung_bias.py` printed "adopt hold 0.050" at Delta -39.5 mm (long): its adoption rule used a signed `Delta <= 20`. It now uses `|Delta| <= 20`, scores a long overshoot as the new verdict `reversed (long)` (routed like the 0.075 fallback), and the ladder-done line names the direction (SHORT or LONG). Pinned by `test_ladder_done_refuses_a_long_rung_and_names_direction` in `tests/motion/test_hold_rung_bias_probe.py`.

## Code landing

Landed in the working tree after the Sitting (the Verification section is filled at commit).

- **(a) Hold default 0.** `unified_cycle.PRE_RELEASE_HOLD_S` and the `pre_release_hold_s` launch default 0.100 to 0.000 (launch description now says 0.000 = default since 2026-10-31, 0.1 re-flies the hold); the tests that pinned the 0.100 default were re-pinned (e.g. the bump release offset 16.2696 to 17.0802 mm, with a hold-pinned-0.1 variant restoring the old value).
- **(b) Hand stroke.** `hand_catch_prime_rev` 9.9594 to 10.45 rev (box top 10.143, firmware clip 10.501 kept); bottom inset 10 to 2 mm (owner asked for 0; 2 mm because the validate floor has no tolerance); new derived `HAND_PLANNER_TRAVEL_MM` = 361.986 (20 + 10.501 x 32.567) replaces `GEOM_HAND_STROKE_MM` 344.75 as the `cup_realize` slider clamp and the `mujoco_plant` / MJCF joint range (`sim/model/jugglebot.xml` regenerated). Cup z band [0.6835, 1.0238] m, box [0.6855, 1.0138] m (0.0614 .. 10.143 rev). Ripple: the tilt lever to the band top grew ~482.7 to ~498.6 mm, so the tilt acceleration cap falls ~5.18 to ~5.01 rad/s^2 (about 3%, the safe direction).
- **(c) Opening-REST smooth return.** The 2.5 rev/s park-rate velocity cap is retired (velocity cap 15 rev/s, `REST_RETURN_HAND_VEL_LIMIT_RPS`); the 5 rev/s^2 acceleration bound is kept because of the FW 34 first-frame-within-0.05-rev resume rule (`2026-09-18-opening-rest-homes-the-hand.md`). The goal-13 hand (+8.6174 rev) returns in 3.20 s (delta 8.556 rev), against 6.42 s under the old cap.
- **(d) REST splice lead scales with the window.** The re-splice lead for a REST is sized by the window length (fit: 1.5 s window 80 ms, 3.2 s 290-370 ms, 6.5 s 1360 ms; checked on a replay of goal 13), so the long return no longer outruns a fixed 158 ms budget (JB-122 candidate 2). Candidate 1, skipping the re-splice of a live REST, was rejected: the announcement swap installs a different terminal.
- **(e) Box re-sweep, 82 cells** (`config/generated/admissible_box.yaml`, gate_hash there). Added: oval 0.40 in both modes, 0.35 in pose, apex 1.10 in both; nothing removed; the high-low 1.05/0.95 LOW cells are now non-empty. Two value-identity pins in `tests/motion/test_skills_admissible.py` were relaxed to structure (counts and order) because the re-sweep moves values by design, and the pins' point is the tail's structure.
- **(f) Rung-1 seating threshold 10 to 5 deg at dwell 0.65** (`tests/motion/test_skills_admissible.py::test_oval_cell_records_the_carry_seating_not_the_standalone_catch`, `seating_deg >= 5.0`): the seating angle fell 17.8 to 8.05 deg at hold 0. To be checked against the cone limits at the next Sitting.
- **BallButler headers.** `config/generate_config.py` also rewrote `BallButler/ball_butler_main/{hardware_config,protocol_config}.h` (prime 10.45 plus earlier-lagging pose/protocol constants); they need review and a commit in the BallButler repo. No reflash is implied (nothing reads `HAND_CATCH_PRIME_REV`).

## Outcome / Next

Status stays in-progress until the wave-1 code is committed. The next Sitting reads hold 0 at the working point, the hop start at hold 0 (JB-121), and the Hand stroke at the new top.

## Verification

Full suite on the final tree (run 2026-10-31 ~21:50, `./run_tests.sh --full`, log `temp/logs/sp_full_h0c.log`): **7844 passed, 55 skipped in 373.17 s; serial tail 8 passed in 30.47 s**. Box: two sweeps on rsync copies of the edited tree (`sweep_h0.sh`, 28 families, two rounds each, BLAS one thread): 28/28 `cmp` IDENTICAL within each sweep, and the second sweep (after the audit comment fixes re-hashed the gated files) matched the first value-for-value in every family (only the gate_hash line differs). gate_hash 961808ee2d20 = `admissible.gate_hash()` on the committed tree. After this run only this Verification paragraph changed; the logbook/index tests were re-run scoped.
