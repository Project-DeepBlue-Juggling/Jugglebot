---
id: JB-102
kind: feature
title: 'Pre-release hold rung: release from a moving Platform (hold 0.100 → 0)'
status: in_progress
created: '2026-10-30'
updated: '2026-11-05T12:29:16Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: normative
area: skill-stack
blocked_by: []
links:
- plans/active/two-ball-oval.md
- logbook/2026-10-31-hold-ladder-sitting-the-hold-model-was-wrong.md
- logbook/2026-11-04-sitting-debrief.md
- plans/active/carry-ring-zvd.md
parent: two-ball-oval
filed_by: claude (P3 re-gate close-out 2026-10-30)
---

Owner decision 2026-10-30 (P3 re-gate GO): a pre-release-hold rung is scheduled ahead of the P4 working-point commit (plans/active/two-ball-oval.md P4.0).

## Why

The dwell limit under the pose planner is the 0.100 s PRE-RELEASE HOLD, not the legs. At dwell 0.40 the refusal is HAND_LIMIT_ACC 4308 vs 3900, flat across apex, and it vanishes when the hold shrinks: hold 0.050 makes 0.40 admissible at today's limits (hand -10 %, leg velocity -12 %); hold 0.025 makes 0.35 admissible.

The P4 working point 600/8000/200k produces identical pose plans (limits only gate), so it is margin, not reach. Two balls in one hand need dwell 0.30-0.35 at apex 1.0-1.05, which only a hold of 0 plus P4 permits (offline arithmetic, scratch probe, 2026-10-30).

## Why the hold exists

The R4 finding (2026-09-27/28), the Platform still moving +70..+100 mm/s at separation added overshoot; 0.100 = 45 ms tracking delay + 30 ms lag + 20 ms.

## Design item

The ideal result is NO hold, the Platform moving continuously through the pattern. Release from a moving Platform: compose the Platform's velocity into the release state and aim, and account for the 45 + 30 ms tracking delay using measured, not commanded, state.

## Ladder

Hold 0.100 -> 0.075 -> 0.050 -> 0, at dwell 0.45, judged on landing scatter and catch rate.

## Gate

Landing scatter not worse than the 0.100 baseline (sd about 14-16 mm in the 2026-10-30 re-gate) at dwell 0.45, and catch rate at least 30/30.

## Reframe (2026-10-31)

The Sitting flew the ladder and **withdrew the model** this Item was written on. Do not build delay-aware composition as specified under `## Design item`: it would predict short and add forward velocity, making throws longer.

- Result: a shorter hold lands the ball LONG (hold 0.050 is 39.5 mm long of the control, hold 0 a further 28.4 mm), monotone in the hold, nothing dropped.
- Cause: the Platform tracks the executed command within about 15 ms, not 45 + 30 ms. The carry deceleration excites a roughly 6 Hz ring; the hold only moves the ring phase at release.
- Decision (owner): `pre_release_hold_s` default 0.000. Hold 0 flew 30/30 with the tightest spread (9.8 mm) and leg acceleration down 44 percent.

## Design options that remain

1. Identify the roughly 6 Hz mode from the 2026-10-31 bag and compose through it.
2. Soften the carry deceleration (jerk-limit it).
3. End the deceleration a whole number of ring periods before release.
4. Damp the mode in the leg loop; this reopens the frozen gain hunt and is the owner's call.

## Progress note 2026-10-30 evening

- Reframe: the planner already composes the whole cup velocity (Platform carrier + stroke) into the ball's launch velocity. The hold protects against PLANT TRACKING DELAY (45 ms + 30 ms lag), so the open design item is delay-aware composition, not "compose the velocity".
- Hold 0 at 0.45/150/0.95 predicts a −81..−103 mm short landing, beyond the learner + catch authority (~55 mm); the 10-17 "0.30 at apex 1.0" number is a hold-0 number.
- Gate addition: a mean-bias criterion (plant residual y − u per rung, read by `tools/probes/hold_rung_bias.py --latest` from the launch log and the one shared memory; it attributes goals and holds itself and prints the next hold), since scatter + catch rate cannot see a shift the learner absorbs.
- Sitting Runbook section: `tests/hardware/session_oval_rung1.md` § 11. Logbook: `logbook/2026-10-30-sitting-prep-columns-check-hold-ladder-and-apex-cells.md`.

## Sitting

- setup: armed, bb-fed, bridge-cycled
- why: carry-ring-zvd P0 could not identify the Platform mode from the bags on disk (the encoder and mocap fits disagree, 5.4 vs 4.45 Hz), so the mode is identified on the robot first and the ZVD rows (provisional 4.9 Hz / 0.20 / 20 ms) are then flown A/B against the flag-off carry. Needs bridge FW 39 (Hand C1 grace 14 ms, BridgeIdentity receipt in the launch log); under FW 39 only Apex <= 1.10 flies (release-knot C1 stop vs the clip) until JB-174 lands; this row is Apex 1.00.
- do:
  - Before arming: confirm the colcon build is done (`cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot`, then re-source; InstallSegment.srv changed) and the launch log shows the new box line gate_hash e8d303ae93bc. The launch log shows bridge FW 39. Run `ros2 param get /skill_node carry_zvd` and confirm it reads false.
  - Identification: no step row (2026-11-05 showed 20 mm profiled steps excite no resolvable ring); the mode comes from `python tools/probes/platform_mode_fit.py <bag>` on the Sitting's own bag afterwards (forced-response fit).
  - A arm (flag off, the existing behaviour): `ros2 param set /skill_node plant_id zvd-oval-<date>` and read it back, then one Attempt of `{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.40, num_cycles: 32}` (cup empty, Ball Butler loaded).
  - Empty the Cup after every ended Attempt before re-sending a fed goal (two REJECTED_BALL_IN_HAND on 2026-11-05); after clear_errors re-enter TRAJECTORY before the next goal (one WRONG_MODE).
  - B arm: `ros2 param set /skill_node carry_zvd true` (refused while a goal runs) and read it back with `ros2 param get /skill_node carry_zvd`; the goal start line must show carry_zvd. Fly the same oval goal at dwell 0.40 on the same plant. Then dwell 0.35 (`dwell_s: 0.35`, same other keys) with the flag on, only if 0.40 holds.
  - Columns canary with the flag OFF (the swept box has no Columns cell with the rows on): before it run `ros2 param set /skill_node carry_zvd false` and read it back with `ros2 param get /skill_node carry_zvd`; two Attempts: GUI Columns, Sep. 125, Apex 0.95, Dwell 0.27, Throws 21.
  - Afterwards re-fit the mode on this bag: `python tools/probes/platform_mode_fit.py <bag>` (the step responses give the free-decay estimate P0 could not get).
- pass: at Dwell 0.40 with the flag on, the carried minus from-rest raw landing bias is within +/-10 mm, the carried per-Throw sd is under 17.5 mm, and 30 consecutive Catches land in one Attempt; no CUP_SEATING_CONE or leg LIMIT end; the Columns canary (flag off) is unchanged (each Attempt COMPLETED, no new refusal).
- record: raw x per Throw (landed minus u) split from-rest / carried, per arm; encoder and mocap excess at separation per arm; RING refusals (the RING refusal text if any oval install is refused); install-line leg peaks; the platform_mode_fit.py result on the identification bag (x and y); the bag and launch log paths.
- minutes: 45

## Sitting result (2026-11-05 22:55)

- verdict: fail
- notes: Attempted. Only tried up to step 3, as every attempt was ending prematurely due to LANE_REFUSED, MAX_DEVIATION, or SPINOUT_DETECTED

## Progress

- 2026-10-31: 2026-10-31 ladder flown: shorter hold lands LONG, model withdrawn, hold default now 0; do not build delay-aware composition as specified (detail in Reframe section; logbook 2026-10-31-hold-ladder-sitting-the-hold-model-was-wrong)
- 2026-11-04: 2026-11-03 Sitting (one-ball oval, dwell 0.40/0.35, legs 600/8000): the release-from-a-moving-Platform mechanism measured on the CARRY.
  - The carry decelerates until 25-30 ms before release; the Platform moves +40 (0.40) / +79 (0.35) mm/s faster than commanded at separation (encoder FK; mocap +75/+131), the ball's launch-velocity error matches ~1:1 and carried throws land +76 / +122 mm long (+48 from rest; +46 at dwell 0.50, hold 0, on 10-31).
  - Root cause (offline, 2026-11-04, logbook/2026-11-04-sitting-debrief.md): the STEADY QP's minimum-energy carry uses the whole Dwell and decelerates into the release knot; nothing models the ~6 Hz mode. A 6 Hz / damping 0.17 model gives a ring share of +24 / +51 / +79 mm at dwell 0.50 / 0.40 / 0.35.
  - Ranked fixes: ZVD rows on the carry in cup_cycle._pose_augment (gated, re-sweep, mode ID first); pre_release_hold_s 0.050 as phase compensation (about -60 mm at 0.40); leg accel FF; learner-side compensation; Dwell 0.45-0.50.
- 2026-11-04: 2026-11-04: planned as plans/active/carry-ring-zvd.md (owner 2026-11-04): P0 mode identification from the 2026-10-31 and 2026-11-03 bags, P1 ZVD equality rows on the modelled Platform residual at release in cup_cycle._pose_augment behind trajectory_op.carry_zvd (I-PLAN-16, soft-penalty fallback pre-registered), P2 box re-sweep with the rows on, P3 A/B oval Sitting.
- 2026-11-05: P2 box swept (gate_hash e8d303ae93bc, 195 cells): oval Dwell 0.40 / Apex 1.00 keeps 30 mm with the rows on; the Columns canary cell (Dwell 0.27) is empty with the rows on (RING), so the Columns canary flies flag off; fallback is an owner question (plan carry-ring-zvd.md section 8)
- 2026-11-05: Sitting 22:46: step 3 (A arm, flag off) only; 3 fed Attempts 9/9, 3/3, 0/0, each cut by a firmware STOP (2 Hand C1 stops + MAX_DEVIATION, 1 bridge GC pause); B arm never set, no A/B. Flag-off repeats 11-03 (raw x +59 mm carried). Identification steps excite no ring; whole-bag fit x 5.47 enc / 4.68 mocap Hz, y 4.1-4.2 Hz. Re-fly under FW 39 (C1 grace 14 ms).
