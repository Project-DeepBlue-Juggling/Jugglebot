---
title: "Two-ball oval P3 A/B Sittings: pose NO-GO at dwell 0.50 (47/57), the touch-down skid becomes a QP bound (I-PLAN-14, 175 mm/s), the lead-clamp staircase becomes a prerequisite plan"
type: investigation
date: 2026-10-24
status: resolved
phase: "two-ball-oval — Phase 3 (hardware gate tail)"
related_plan:
  - two-ball-oval.md
  - leg-lead-clamp-age-aware.md
related_entries:
  - 2026-10-23-oval-rung2-flown-dwell-050.md
  - 2026-10-23-two-ball-oval-p3-pose-level-carry-planner.md
sessions: []
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tools/probes/oval_carry_bag_metrics.py
  - tests/motion/test_pose_planner.py
  - config/generated/admissible_box.yaml
  - tests/motion/test_skills_admissible.py
  - tests/hardware/session_oval_rung1.md
  - plans/active/two-ball-oval.md
  - plans/active/leg-lead-clamp-age-aware.md
  - plans/active/INDEX.md
  - docs/project-map.md
subsystem:
  - motion
  - config
  - tools
  - testing
tags:
  - oval
  - hardware
  - planner
  - safety
  - dynamics
  - performance
---

# Two-ball oval P3 A/B Sittings: the pose planner dropped what the level planner caught, because the cup was skidding

## Summary
- **NO-GO for `pose` as built at dwell 0.50.** Two Sittings on 2026-10-24 (bags `2026-10-24_10-44-21` and `2026-10-24_11-34-29`): `pose` caught 47/57 in Sitting 1 and 7/8 (plus a feed-site first-Throw miss and two solver-late or transit-margin self-ends) in Sitting 2, against `level`'s 32/32 at the same cell (rung 2 on 2026-10-23 flew `level` 38/38). The debounced seat bit broke on 9 of 77 `pose` carries.
- **Cause: touch-down skid.** Under `pose` the cup is already moving inward, toward T, when the ball lands: median v_x -507 mm/s at 0.50, -174 mm/s at 0.65, +20 mm/s under `level`. Every miss was a ball landing more than 15 mm inward meeting the advancing rim (9 of 16 missed under `pose` 0.50; 0 of 7 under `level` 0.65). Throw quality was identical in every column (landing sd 16-19 / 13-15 mm).
- **Decision 28b ("skid logged only") is withdrawn.** The skid is a bound: `I-PLAN-14`, `PoseConfig.touchdown_skid_mmps` = 175 mm/s (the 0.65 figure that caught 63/64), QP rows on the cup's lateral velocity at the catch time, `pose` mode only.
- **Single-leg ~10 Hz vibrations** (leg 1 twice, leg 2 once, two with the iq setpoint on the 15 A clamp) are the can-bridge leg lead clamp amplifying leg-bus encoder frame drops (`leg_interp.cpp:1066-1072`), under both planners. Not the planner. Re-opened as the prerequisite plan `plans/active/leg-lead-clamp-age-aware.md`.
- **Owner decisions (2026-10-24, `AskUserQuestion`, recommended options taken):** (1) lead clamp is a prerequisite plan, flown before the next powered oval Sitting and before P4's leg-current raise; (2) `pose` is fixed with the skid bound and re-gated (runbook § 9); (3) P4/P5 aim at the first plannable dwell (0.45 today), not 0.30.
- **Landed (code, not flown):** the knob, I-PLAN-14, the box re-swept (a 0.45 pose cell added, 26 cells), three new skid tests, the probe's missed-catch crash fixed and a touch-down cup-velocity column added. Default stays `level`.

## The Sittings as flown
Evidence base: the Opus bag review `temp/probes/oval_carry_2026-10-24_10-44-21/sitting_review_2026-10-24.md` (temp/ rots; this entry is its durable home). Method: launch logs parsed for dispatch, END, `throw k/N` and `install_segment ... planner X` lines (the mode IS logged on every install, so it needs no inference); carry metrics from `tools/probes/oval_carry_bag_metrics.py`; cup velocity at the scheduled landing from the mocap `Platform` body on `/rigid_body_poses` (300 Hz, 30 ms central difference) rotated by the robot-frame yaw (10.81 deg); landing offset from the last `/balls` fit before landing in the robot frame; vibration from `/robot_state`, `/leg_cmd_executed` (post-clamp target) and `/leg_setpoint_echo` (commanded).

**Sitting 1 (10:44).** The A/B was not flown as written: the `level` column ran at dwell 0.65 (G1, 32/32), not 0.50. Each goal opens with the feed catch, a REST and the first Throw under `level`; every carry-folded CATCH install says `pose` from G3 onward (the P3-3 expectation, confirmed). STL = SPLICE_TOO_LATE. Epochs are 1792xxxxxx.x.

| goal | t0 (epoch) | dwell | carry planner | result | miss: skill (= throw k+3) | miss landing off C (fit, mm) dx / dy | cup v_x at touch-down (mm/s) | re-aim refusals |
|---|---|---|---|---|---|---|---|---|
| G1 | 799140.6 | 0.65 | level | 32/32 COMPLETED | - | - | ~+20 (column median) | 28 STL |
| G2 | 799229.4 | 0.65 | - | REJECTED_BALL_IN_HAND | | | | |
| G3 | 799242.3 | 0.65 | pose | 6/6 COMPLETED | - | - | | 4 STL |
| G4 | 799641.5 | 0.65 | pose | 25/26 MISSED_CATCH | 29 (throw 26) | -29 / +7 | -195 | 19 STL, 1 INFEASIBLE |
| G5 | 799703.3 | 0.65 | pose | 32/32 COMPLETED | - | - | | 25 STL, 1 INFEASIBLE |
| G6/G7 | 799777 / 799787 | 0.45 | - | refused (in hand; box: no 0.45 cell) | | | | |
| G8 | 799812.8 | 0.50 | pose | 1/1 MISSED | 5 | -39 / -104 (log early fit: -22 y, clamped to 20) | -404 | 2 STL |
| G9 | 799826.5 | 0.50 | pose | 0/0 MISSED | 4 (first throw) | +5 / -40 (log: -52 y, clamped to 20) | -650 | 0 |
| G10 | 799838.8 | 0.50 | pose | 5/5 MISSED | 9 | -22 / -9 | -794 | 4 STL |
| G11 | 799863.2 | 0.50 | pose | 0/0 MISSED | 4 (first throw) | -26 / 0 | -590 | 1 STL, 1 CATCH_AXIS (feed) |
| G12 | 799876.7 | 0.50 | pose | 4/4 MISSED | 8 | -19 / +2 | -446 | 3 STL, 1 CATCH_AXIS (feed) |
| G13 | 799894.4 | 0.50 | pose | 4/4 MISSED | 8 | -22 / +1 | -439 | 2 STL |
| G14 | 799911.8 | 0.50 | pose | 3/3 MISSED | 7 | -76 / -47 | -688 | 2 STL |
| G15 | 799934.8 | 0.50 | pose | 1/1 MISSED | 5 | -51 / +23 (log: -37 x, clamped to -35) | -403 | 2 STL |
| G16 | 799952.7 | 0.50 | pose | 9/9 MISSED | 13 | -34 / +25 | -556 | 7 STL |
| G17 | 799977.6 | 0.50 | pose | 20/20 MISSED | 24 | -84 / -13 (log: -97.5 x, clamped to -35) | -912 | 11 STL, 1 INFEASIBLE |

Columns: **L0.65** 32/32. **P0.65** 63/64 (98 %). **P0.50** 47/57 (82 %). All P0.50 misses sit at the CATCH at C; two were the very first Throw after the feed (G9, G11). The "drops" are missed catches, not carry drops: on all 11 misses the raw seat bit `ball_held_raw` never went true in [-0.3, +0.6] s of the landing.

**Sitting 2 (11:34).** Epochs are 17928022xx.x.

| goal | dispatched | planner (`install_segment`) | result | miss / end | landing dx / dy mean (sd), mm |
|---|---|---|---|---|---|
| G1 | 149.4 | level (37 CATCH installs, all `level`) | **32/32 COMPLETED** | - | -1.7 (16.8) / -5.3 (21.8) |
| G2 | 217.5 | - | REJECTED_BALL_IN_HAND | | |
| G3 | 226.5 | pose (8 carry installs) | 7/8, **ENDED SPLICE_TOO_LATE** at skill 11 | no ball missed: the CATCH install at 242.89 got no solver reply by its deadline (`plan 386.3 ms vs deadline +385.1 ms`); earlier the skill-4 re-aim was refused the same way (250.8 vs 250.0). The carried Throw 8 ran after the END and was not counted | -1.0 (15.2) / +12.2 (11.0) |
| G4 | 247.8 | - (first REST only) | 0/0, **ENDED SPLICE_TOO_LATE at the REST** | skill-0 REST: `reply transit outran the wire deadline` (solve 158.1 ms, t_inst +142.4 ms, deadline +1000 ms; the first knot was no longer 3 ahead of the wire). The BB throw was cancelled. A transit-margin end, not solve time | - |
| G5 | 255.7 | pose | 0/0, **MISSED_CATCH** at skill 4 | the first Throw after the feed; seat bit never true | - |

`level` 0.50 column (S2 G1) against rung 2:

| item | G1 level 0.50 | criterion / rung 2 |
|---|---|---|
| catch rate | **32/32** | rung 2 38/38 |
| leg 1 iq peak, carry max | 11.31 A (the lead-clamp carry); **7.32 A** on every other carry (median 6.14) | 7.49 |
| leg 2 iq peak, carry max | **9.87 A** (median 9.12) | 10.05 |
| `ball_held` unbroken | 32/32 | |
| `ball_held_raw` dropouts | 81 % (26/32) | as-is reference 69 % |
| commanded leg jerk median (echo carry window) | **66.5k** (p90 69.9k) | the expected ~66k level figure |
| bank, commanded max | median 2.99, max 3.61 deg | +-2.3 |
| re-aim refusals | 24 STL, 1 INFEASIBLE, 2 LIMIT_ACC (re-aims only, 6674 and 15915 mm/s^2 > 5000; never installed) | no carry LIMIT_* installed |

`pose` 0.50 in Sitting 2 (G3 + G5, 11 carries): leg 1/2 iq max 4.97 / 6.68 A, `ball_held` unbroken 9/11 (two ~0.1 s flickers ~0.14 s before release, as in Sitting 1), raw dropouts 64 %, jerk median 15.5k. **Combined at 0.50:** `level` 32/32; `pose` 47/57 (S1) + 7/8 caught, 1 feed miss (S2), i.e. 54/66 of the attempts that reached a catch. The A/B is now a same-cell comparison and the NO-GO stands.

## Row 27 per column
Probe carry window = scheduled landing to next Jugglebot release; steady carries < 1.0 s. Sitting 1 columns; the `level` 0.50 column is in the table above.

| row-27 item | as-is `level` (flown at **0.65**, S1 G1) | `pose` 0.65 (G3-G5) | `pose` 0.50 (G8-G17) | criterion / reference |
|---|---|---|---|---|
| catch rate | 32/32 | 63/64 | **47/57 (82 %)** | >= as-is over 30. Rung 2 `level` 0.50: 38/38 |
| legs 1/2 iq peak, carry median / max (A) | 3.90 / 5.28 ; 6.38 / 7.11 | 4.01 / 5.89 ; 5.95 / 7.01 | 4.38 / **11.98** ; 6.65 / **11.88** | <= 7.49 / 10.05. Both maxima are the lead-clamp events (G13, G14); otherwise leg 1 <= 6.2, leg 2 <= 8.1 |
| `ball_held` unbroken | 32/32 | 66/66 | **68/77** | every carry. The 9 breaks are ~0.1 s flickers 0.14-0.29 s before release; the ball was still thrown |
| `ball_held_raw` dropouts | 53 % (17/32) | 64 % (42/66) | 62 % (48/77) | vs as-is 69 % at 0.50: no rise |
| commanded leg jerk median (echo, carry window) | 35.1k (p90 92.8k) | 13.1k (p90 64.1k) | 15.7k (p90 19.4k) | ~45k vs ~66k expected on the node's window measure; the echo measure reads lower, ratio in the expected direction |
| leg vel / acc median | 164 / 1331 | 167 / 923 | 239 / 1251 | |
| bank, commanded max: median / max (deg) | 2.97 / 3.68 | 3.01 / 3.41 | 3.00 / 3.45 | unchanged (+-2.3) |
| carry LIMIT_* | none | none | none | no LIMIT_ anywhere in the launch log |
| cup abs v_xy at touch-down, median (mocap) | 24 | 175 | **507-572** | offline skid: level 14-33, pose 225 / 494 |

Call: strictly the A/B was INCONCLUSIVE after Sitting 1 (no `level` 0.50 column); the `pose` 0.50 column failed on its own on catch rate against the rung-2 reference and on the debounced seat bit, and Sitting 2 supplied the `level` column. Per the runbook the ladder continues on `level`.

## The vibration events
Scans: high-passed tracking error (`/robot_state` position minus `/leg_cmd_executed`) on a 100 Hz grid ranked by single-leg dominance; executed target frozen while target_vel is above 0.5 rev/s (only four episodes in the whole Sitting 1 bag, all legs 1 and 2); encoder update interval and target-minus-last-fresh-fb against the lead.

| | A | B | minor | minor |
|---|---|---|---|---|
| time (epoch) | 1792799742.4-742.9 | 1792799921.7-922.6 | 1792799908.8-909.1 | 1792799710.3-710.4 |
| goal, mode | S1 G5, pose 0.65 | S1 G14, pose 0.50 | S1 G13, pose 0.50, during the END-MISSED stop | S1 G5, pose 0.65 |
| phase | +0.24 s after release, moving back to C | +0.21 s after release, same | stop REST after the miss | +0.16 s after release |
| leg | 1 | 2 | 1 | 2 |
| encoder update interval (fresh `/robot_state` pos) | median **94 ms** (normal ~20) | median **90 ms** | 20 ms median, 89 ms max | 20 ms median, 101 ms max |
| executed target pinned at last fb +- 0.10 rev (7.07 mm) | 54 % of samples | 55 % | 31 % | 24 % |
| dominant frequency | ~10 Hz (step period 90-94 ms) | ~10 Hz | ~11-12 Hz | ~11 Hz |
| amplitude | hp error RMS 2.3 mm; iq 6.9 A, iq_sp 9.8 A | hp error RMS 2.6 mm; error +-19 mm; iq 11.9 A, iq_sp **14.99 A** (1 sample on the clamp, 5 >= 10 A) | error to +18.8 mm; iq 12.0 A | iq 5.5 A |
| other legs' max iq | 4.5 A | 6.0 A | 7.2 A | 5.3 A |
| `/leg_setpoint_echo` | smooth (hp 0.3 mm) | smooth (hp 0.6 mm) | smooth | smooth |
| stroke position | 136-193 mm of 5-275 | 122-182 mm | 145-212 mm | 141-187 mm |

Sitting 2 added one more event, under `level`: leg 1, 1792802191.80-192.84 (one ~1.0 s event, two scan episodes), G1 `level` 0.50, mid-carry with the ball (+0.16 s after landing, 0.37 s before release). Encoder interval median 89 / 94 ms, target pinned at last fb 76 % / 34 %, peak iq 11.31 A with **iq_setpoint 14.99 A**, other legs' max 9.56 A, command-minus-measured 7.1 mm, echo smooth (second difference <= 1.2 mm). The bridge logged `leg 1 heartbeat gap 906 ms (0/91 encoder frames)` at 192.47. That is leg 1 twice (S1 A, S2), leg 2 once (S1 B). Heartbeat gaps at 220.65 (leg 4, 503 ms, 28/50 frames) and 248.05 (leg 0, 1097 ms, 24/110) produced no staircase because those legs were slow or idle.

**Mechanism, read off the raw samples** (event B, leg 2, `raw_922.txt` in the review's scratch):
- The measured position arrives only every ~100 ms: 126.60 -> 137.33 -> 149.71 -> 161.19 -> 169.97 mm.
- The executed target equals each of those plus 7.07 mm exactly: 144.40, 156.77, 168.25, 177.03. target_vel stays smooth throughout.
- So the bridge's leg lead clamp holds target = stale fb + `MAX_LEAD_REV` (`leg_interp.cpp:1066-1072`; no age extrapolation, unlike the hand lane's `fb_ex`), then jumps ~12 mm each time a fresh encoder frame lands. The ODrive is hit with a ~10 Hz staircase of 11-12 mm steps.
- Not the planner (the echo is smooth). Not mechanical (the measured response follows the executed staircase). Not a stroke end (all mid-stroke). Bus side: heartbeat-gap lines bracket three of the four S1 events (leg 2 gap ended 709.69 after 901 ms; leg 1 ended 908.49 after 400 ms with 0/40 encoder frames; leg 2 ended 922.40 after 1002 ms); event A had no logged gap.
- Not attributable to `pose`: the mechanism engages whenever leg speed x feedback age exceeds 7 mm, which both planners do, and S2 produced it under `level`. `pose` goals filled ~85 % of the S1 goal time.
- The runbook stop rule "leg on clamp > 50 ms" was not met by telemetry (one sample at 60 Hz), but that telemetry under-samples the ODrive.

This is the lead-clamp amplifier of the archived plan `plans/archived/leg-bus-frame-drops.md` (Workstream A, archived unimplemented 2026-10-14). New: `plans/active/leg-lead-clamp-age-aware.md` (P1 firmware: age-aware anchor plus a per-leg engagement counter on the wire; P2 bench validation against these bag episodes; P3 one `level` 0.50 Sitting with zero staircase episodes). Why it is now a safety item: a 15 A setpoint and +-19 mm error on one leg in one Sitting, and P4's `leg_curr_limit_a` 15 -> 30 would make that a 30 A staircase.

## Why `pose` missed (ranked)
1. **Touch-down skid removes the inward capture margin. Strong.**
   - Under `pose` the cup at landing is already leaving C toward T: v_x median -507 mm/s caught, -572 missed at 0.50; -174 at 0.65; +20 under `level`.
   - Misses at 0.50 lie inward: dx median -29.7 mm, 9 of 10 with dx < -15; caught balls dx median +5.6. Conditional miss rate at P0.50: dx < -15 is **9/16 missed**; dx >= -15 is **1/21**. With a stationary cup inward balls were caught: L0.65 0/7 missed, P0.65 1/11.
   - The advancing inward rim meets the ball at ~0.5 m/s relative; the seat bit never fires on any miss, which fits a ball deflected at entry.
   - The skid scales with shorter dwell, matching the offline 225 -> 494 mm/s, and so does the miss rate (1/64 -> 10/57). Robot-measured against planned: 0.65 planned 225 / measured -174; 0.50 planned 494 / measured -507.
2. **Wide throws beyond the aim authority. Present, not differential.** About 5 of the 10 P0.50 misses had fits beyond +-20 y or the -35/+10 x authority (G9, G14, G15, G17, G8). Landing scatter is equal in every column (dx sd 16-19, dy sd 13-15), and caught balls beyond authority exist in every column (L0.65 8, P0.65 25, P0.50 8): a stationary cup absorbs them, a skidding cup does not when they lie inward.
3. **Re-aims refused (SPLICE_TOO_LATE). Not differential.** Most catches fly on the first aim in every column (L0.65 28 refusals in 32 catches; P0.50 34 in 57); `level` caught 32/32 with the same handicap. It amplifies (1): the cup cannot chase an inward ball.
4. **`ball_held_raw` chatter then loss. Rejected.** Raw was never true on any of the 11 misses and the dropout rate did not rise (62 % vs 69 %). The 9 debounced breaks were on carries whose ball was then thrown successfully: a late-carry unseat 0.14-0.29 s before release, a separate finding (Open questions).
5. **Receive-pin tilt off the landing, or tracker occlusion. No evidence.** Caught-ball landing bias is small; tracker confirms are normal (7-11 steps, 14-20 mm from expected).
6. **"The throw is not angled enough." Refuted** (the operator's read: `pose` throws land near the midline). Landing offsets by planner from the launch logs, which are not angle-dependent: S1 level n=36 dx median -8 sd 18; S1 pose n=107 dx +2 sd 19; S2 level n=34 dx +1 sd 16; S2 pose n=6 dx -6 sd 14. The two planners put the ball in the same place; the misses are the rim deflection of the inward tail.

Side finding: on two misses (G9, G12) the tracker "confirmed" the next announced ball on a ~1.8-2.0 m/s marker about 90 mm off; that is the deflected missed ball, and it clamped a post-END aim. Harmless here, a matcher hazard (Open questions).

## Discussion
**The withdrawn hypothesis.** Decision 28b recorded the touch-down skid (225-494 mm/s planned at 0.65-0.50) as a logged diagnostic that is never refused, on the reasoning that a cup moving at 0.5 m/s in the plane of the catch is a velocity-matched catch of a ball whose flight is fixed, so the skid is the price of the lower leg jerk and the catch is robust to it. The hardware does not support that. A moving cup is velocity-matched only along the cup axis; the lateral component is a closing speed against the inward rim, and the inward tail of the landing scatter (sd 16-19 mm; 16 of the 37 labelled pose-0.50 landings were beyond 15 mm inward) meets that rim at ~0.5 m/s. 9 of 16 such balls were missed, against 0 of 7 under a stationary cup. The offline ladder already showed 494 mm/s; what was missing was a measure of how much of the landing scatter the cup is allowed to hit. The hypothesis "skid is a harmless diagnostic" does not survive the A/B; the bound replaces it.

**Why a bound and not a weight.** The pose objective is the LEG jerk through one Jacobian, and its minimiser passes through the catch site at speed, so a lateral catch weight (decision 2's recorded lever) competes with the objective and its balance drifts with every re-tune of the leg-jerk weight, the dwell or the limits. A row holds exactly where a weight only leans. Three failure modes the row prevents: the objective's optimum moving the skid back out at the next dwell; a weight tuned at 0.50 being wrong at 0.45; and a skid that is only visible after a powered Sitting. It is also the quantity `feasibility._catch_closing_check` already records as `catch_skid_mmps`, so planner and check now agree by construction.

**Why 175 mm/s.** It is the dwell-0.65 figure on the robot (-174 measured, 225 planned) at which `pose` caught 63/64, i.e. an envelope the catch has been demonstrated inside. Tighter costs jerk monotonically (offline ladder below: 50 mm/s gives 57.0k / 77.9k at 0.50 / 0.45 against 53.3k / 73.9k at 175) and buys nothing measured. Looser (300) leaves 277 mm/s planned at 0.50, between the demonstrated 174 and the failing 507. The planned skid on the 8-gon is 161.7 mm/s (= 175 cos(pi/8)) on every cell, below the 174 measured at 0.65 and the margin that matters. The robot read lower than plan at 0.65 (174 vs 225) and equal at 0.50 (507 vs 494), so the planned figure is a fair predictor.

**Ruled out.** The receive-pin hold (item 5; the pin is a tilt, the failure is a lateral velocity, and caught-ball bias is small). Tracker occlusion (confirms normal). `ball_held_raw` chatter (raw never true on a miss; dropout rate did not rise). Throw angle (landing table). A lead-clamp cause of the misses: the vibration events fell in the return stroke after a release, away from every landing.

**Tradeoff accepted.** The bound costs window jerk: at 0.50 it rises from 44.3k to 53.3k mm/s^3 (+20 %; `level` is 111.5k, so `pose` is still less than half), at 0.45 from 56.5k to 73.9k, and 0.40 changes from `LIMIT_VEL` (357 > 350) to `HAND_LIMIT_ACC`. A smoother planner that dropped more is worse than a rougher planner that catches: the catch rate is the gate, the window jerk is the means, and the bound spends the means to protect the gate. The seat residual is unchanged at 18.6 deg (11.95 -> 13.85 at 0.65).

**Why the loaded solve time is recorded, not fixed here.** CATCH install solve times on the loaded Jetson: S1 0.65 level 133 / 168 / 191 ms (median / p90 / max, n=43), pose 168 / 240 / 332 (n=74); S1 0.50 pose 156 / 232 / 253 (n=95); S2 0.50 level **207 / 278 / 338** (n=38, 14 over 250 ms), pose 236 / 305 / 318 (n=10, 4 over). `pose` self-ended once on solve time (S2 G3) and once on transit margin (S2 G4); only S2 G5 ended on a dropped ball. The solve-time end is not specific to `pose`: `level` at 0.50 solved slower in S2 than `pose` did in S1, the rise of 50-80 ms between Sittings on the same cell points at Jetson load, not formulation, and `level` simply did not draw an install whose reply missed its own deadline. Idle profile (scratch `profile_pose_vs_level.py`, `plan_steady` on the folded oval cell, 12 solves, box lightly loaded): dwell 0.50 level vars/eq/ineq 165/24/686, iters 44, solve 16.7 ms, qp stage 21.5, wall 42.0; pose 165/24/856, iters 45, solve 19.3, qp 32.8, wall 46.3 (pose skips the 7.3 ms tilt stage). Dwell 0.65: level 183/24/764, iters 49, solve 19.6, wall 48.4; pose 183/24/988, iters 62, solve 25.6, wall 55.2. The pose extra is assembly (~8-10 ms), not the solver (+2-6 ms). The deadline is stack-wide SPLICE_TOO_LATE residue; the idle budget test cannot see load inflation (UH-3's lesson). Fixing it here would change a stack-wide contract on the strength of one cell.

**Why the P4 destination moved.** Offline at P4's working point 600/8000/200k (same probe): `level` is still `CUP_SEATING_CONE`-refused at 0.40 (the cone is not a limits problem) and `LIMIT_ACC` at 0.35 / 0.30; `pose` (bound 175) clears 0.50 (53,341) and 0.45 (73,949), is `HAND_LIMIT_ACC`-refused at 0.40, and is QP INFEASIBLE at 0.35 and 0.30 (the 0.100 + 0.050 s release holds eat half the window; no report). Neither planner reaches 0.30 at 150 mm at this working point, so P4 laddering to 0.30 was a premise that no longer holds. P4/P5 therefore aim at the first plannable dwell (0.45), with two design items that may open the next rungs. P4 also presumes `pose` passes its re-gate; if not, the ladder ends at 0.45 on `level` and the destination is re-examined.

## What landed in code
- **Knob:** `trajectory_op.pose_touchdown_skid_mmps: 175.0` in `config/hardware_config.yaml`, regenerated into `hardware_config.py/.h` and the three delivered header copies (`CatchingCone_code`, `Teensy_code_canbridge`, `Teensy_code_platform`) plus `jugglebot/hardware_config.py`; read as `PoseConfig.touchdown_skid_mmps` (validated >= 0; 0 = rows not assembled).
- **Rows:** `cup_cycle._pose_augment` block 3 (the dz block becomes 4): the cup velocity at the catch time, interpolated between knots as the catch-position row is, projected on the two directions perpendicular to the PRESCRIBED cup axis there, inside an inscribed `cone_sides`-gon of radius `b`: `n_i . v <= b cos(pi/m)`. Level is untouched (`pose=None`).
- **Invariant:** `I-PLAN-14` in `ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md` (and I-PLAN-13's measured line re-stated with the bound).
- **Box:** `config/generated/admissible_box.yaml` re-swept under the bound; a 0.45 pose cell added (26 cells); `tests/motion/test_skills_admissible.py` follows.
- **Probe:** `tools/probes/oval_carry_bag_metrics.py` no longer crashes on a missed catch (the `seat_lag_ms=round(...)` of a NaN seat lag) and gains a touch-down cup-velocity column (mocap `Platform` body at the scheduled landing).
- **Tests:** `tests/motion/test_pose_planner.py` pins moved 44,329 -> 53,341 at 0.50, 0.45 pinned at 73,949, 0.40 `LIMIT_VEL` -> `HAND_LIMIT_ACC`, three new skid tests (`test_touchdown_skid_rows_bind_on_the_production_path`, `test_touchdown_skid_bound_scales_and_keeps_the_jerk_gain`, `test_touchdown_skid_negative_is_refused`).
- **Docs:** runbook `tests/hardware/session_oval_rung1.md` § 8.1 (results) and § 9 (re-gate Sitting, P3R-1..6); plan `two-ball-oval.md` § 1.3 amendments and P3 outcome; new plan `leg-lead-clamp-age-aware.md`; boards regenerated (`plans/active/INDEX.md`, `docs/project-map.md`).

Offline skid-bound ladder (production path, 150 mm / 0.95 / 350-5000-200k-3900, pose; scratch copy of `tools/probes/oval_dwell_sweep.py` with the bound injected via `CycleGoals.pose_cfg`):

| bound (mm/s) | window jerk at dwell 0.65 / 0.55 / 0.50 / 0.45 | planned skid (mm/s) | 0.40 |
|---|---|---|---|
| 0 (off) | 44,329 / 44,329 / 44,329 / 56,479 | 225 / 379 / 494 / 645 | REFUSED `LIMIT_VEL` |
| 300 | 44,329 / 44,329 / 49,718 / 70,370 | 225 / 277 / 277 / 277 | `HAND_LIMIT_ACC` |
| **175 (default)** | 44,329 / 44,329 / 53,341 / 73,949 | 161.7 on every cell | `HAND_LIMIT_ACC` |
| 100 | 44,329 / 44,329 / 55,549 / 76,135 | 92.4 | |
| 50 | 44,329 / 44,329 / 57,021 / 77,894 | 46.3 | `LIMIT_ACC` |

## P4 premise probe (working point 600/8000/200k, 150 mm, apex 0.95)
| dwell | level | pose, bound 175 |
|---|---|---|
| 0.50 | ok | ok, window jerk 53,341 |
| 0.45 | ok | ok, 73,949 |
| 0.40 | REFUSED `CUP_SEATING_CONE` | REFUSED `HAND_LIMIT_ACC` |
| 0.35 | REFUSED `LIMIT_ACC` | QP INFEASIBLE (no report) |
| 0.30 | REFUSED `LIMIT_ACC` | QP INFEASIBLE (no report) |

## Verification
- Planner and cycle tests: `pytest tests/motion/test_pose_planner.py tests/motion/test_cup_cycle.py -q` (2026-10-24, `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1`): **113 passed in 4.37 s**.
- Doc tests: `pytest tests/sim/test_plans_index.py tests/sim/test_glossary_avoid_words.py -q` (2026-10-24): **16 passed in 2.99 s**; `pytest tests/sim/test_docs_links.py -q` (2026-10-24): **6 passed**.
- Probe fix: `python tools/probes/oval_carry_bag_metrics.py` re-run on Sitting 1 (2026-10-24, `--out temp/probes/oval_carry_2026-10-24_10-44-21_probefix/`): **0 tracebacks across 17 goals**; cup v_x per column +19.7 / -174.3 / -533 mm/s (pooled; the review's -507 is over the same goals, caught-only differs); legs 1/2 iq and jerk unchanged from the review.
- Offline skid ladder and P4 premise probe: scratch copies of `tools/probes/oval_dwell_sweep.py` (2026-10-24, production path), tables above.
- Box re-sweep (2026-10-24 11:58–12:54, `temp/sweeps/sweep_P3b.sh`: ten `tools/admissible_sweep.py` invocations × two rounds, BLAS one thread, every family + the four pose oval cells 0.65 / 0.55 / 0.50 / 0.45): **all ten families IDENTICAL across rounds** (`cmp`); merged 26 cells, gate_hash `53609fd5cfab` = live gate, the 22 level cells value-identical to `tests/fixtures/admissible_box_pre_p3.yaml`, box md5 `66efe34829cf37c32c5e1dc2a64b5ccb`; the 0.45 pose cell's landing box is x [−20, +10] × y [−20, +20] mm (one column narrower than the 0.50 cell's).
- Full gate: `./run_tests.sh --full` on the final tree (2026-10-24 13:12–13:19, after the audit fixes; the box's gate_hash re-checked equal to the live `admissible.gate_hash()` 53609fd5cfab first): **parallel 7149 passed / 4 skipped in 322.37 s + serial 8 passed in 29.91 s, RESULT PASS** (P3's closing baseline was 7140 / 4 + 8). An earlier run of the same tree with a post-sweep COMMENT edit in `cup_cycle.py` failed 28 tests on the gate hash alone (every mocked goal refused); the comment was reverted rather than the box re-swept — the gated files are frozen from sweep start to commit.
- NOT FLOWN: nothing in this entry's code has run on the robot. The re-gate is runbook § 9, after the prerequisite plan's P3.

## Open questions

- Cosmetic, deferred to the next gated-file change (a re-sweep is due then anyway): `cup_cycle.py`'s block comments number the dz block `3.` while the `_pose_augment` docstring numbers it `4.` (the skid rows are block 3); a post-sweep comment fix re-hashes the box, so it was reverted.
- **Late-carry seat flickers under `pose`.** 9/77 (S1) and 2/11 (S2) carries show a ~0.1 s debounced `ball_held` break 0.14-0.29 s before release; the ball was still thrown. Not a miss, but the row-27 criterion ("every carry") fails softly. Lives in: Row 27 table. Does the skid bound change it? Read at the re-gate.
- **Tracker matching a deflected ball.** On G9 and G12 the next announced ball was confirmed on a ~1.8-2.0 m/s marker ~90 mm off (the deflected missed ball) and clamped a post-END aim. Lives in: Why `pose` missed, side finding.
- **Loaded-Jetson solve inflation and the 250 ms deadline, both planners.** `level` 0.50 median 207 ms (14 of 38 over 250 ms) vs 42 ms idle wall; the 50-80 ms rise between Sittings on the same cell is undiagnosed. Lives in: Discussion, loaded solve time. Stack-wide SPLICE_TOO_LATE residue, not P3's.
- **Hand-acceleration headroom at 0.40.** `hand_acc_rps2` 3900 is the `pose` refusal there (not a leg knob). Lives in: P4 premise probe; a P4 design item.
- **Release-hold budget at 0.35 / 0.30.** The 0.100 + 0.050 s release holds eat half the window and the QP is infeasible. Lives in: P4 premise probe; a P4 design item.

## Related
- Plan: `plans/active/two-ball-oval.md` (§ 1.3 Amendments P3 tail; P3 A/B outcome; P4 premise), `plans/active/leg-lead-clamp-age-aware.md` (prerequisite), archived `plans/archived/leg-bus-frame-drops.md` (Workstream A origin).
- Runbook: `tests/hardware/session_oval_rung1.md` § 8.1, § 9.
- Entries: `2026-10-23-oval-rung2-flown-dwell-050.md` (the `level` 0.50 reference, 38/38), `2026-10-23-two-ball-oval-p3-pose-level-carry-planner.md` (the planner and the ladder this entry amends).
- Invariants: `I-PLAN-13`, `I-PLAN-14` in `ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md`.
- Evidence: bags `/home/jetson/Desktop/rosbags/2026-10-24_10-44-21/` and `2026-10-24_11-34-29/`; launch logs `/home/jetson/.ros/log/2026-10-24-10-44-21-255138-jetson-913736/launch.log` and `/home/jetson/.ros/log/2026-10-24-11-34-29-354803-jetson-939205/launch.log`.
