---
title: "Sitting 2026-11-03 (evening) debrief: P4 ramp not passed; the one-ball oval's carried Throws land 76-122 mm long because the carry decelerates into the release and the ~6 Hz Platform ring is still swinging forward; JB-161 refuted for the oval, JB-147 and JB-149 refuted for the firmware-STOP ends"
type: investigation
date: 2026-11-04
status: in-progress
phase: "two-ball oval P4"
related_plan: two-ball-oval.md
related_issues: []
files_changed:
  - logbook/2026-11-04-sitting-debrief.md
  - logbook/INDEX.md
  - map/feature-p4-ramp-sitting-columns-canary-at-600-8000-and-20-a-then-the.md
  - map/feature-pre-release-hold-rung-release-from-a-moving-platform-hold-0.md
  - map/bug-ball-2-s-first-throw-after-the-feed-catch-lands-about-180-mm.md
  - map/bug-ball-2-learner-command-pinned-at-the-20-20-mm-clamp-corner.md
  - map/bug-limit-refusal-jitter-margin.md
  - map/bug-hand-stall-stop-near-a-release-cannot-fit-the-stroke-the-20.md
  - map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md
  - map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
  - map/bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid.md
  - map/bug-max-deviation-latched-on-40-90-ms-stale-hand-telemetry.md
  - map/bug-firmware-scheduler-stop-fires-on-a-clean-host-stream-and.md
  - map/feature-install-the-host-hold-within-one-tick-of-a-firmware.md
  - map/idea-learner-context-cannot-tell-a-throw-from-rest-from-a-carried.md
  - docs/runbook.md
  - docs/project-map.md
subsystem: [motion, ros, can]
tags: [dynamics, safety, performance]
---

# Sitting 2026-11-03 (evening) debrief

Fourth run of `/sitting-debrief`, after `2026-11-03-sitting-debrief.md`. One Sitting, 22:29 to 22:42, the first flight at the P4 working point (legs 600 mm/s, 8000 mm/s^2, jerk 200000, 20 A; box gate_hash 2605fc1bd25e). Launch log `~/.ros/log/2026-11-03-22-29-00-754746-jetson-102791/launch.log`, bag `~/Desktop/rosbags/2026-11-03_22-29-00/2026-11-03_22-29-00_0.mcap`, Hand capture `temp/hand_capture/hand_capture_20261103_222904.csv`. Item: JB-162. Owner's typed result: `pending`. Owner observation: most misses landed too far in +x.

## Results

| Item | Verdict | Evidence |
|---|---|---|
| JB-162 P4 ramp | not passed (owner confirmed) | Columns canary 1 of 5 flown, COMPLETED 20/20 (L1008), no leg LIMIT end, peak leg iq 11.7 A (leg 2; 0 samples at or above 15 A). One-ball oval: 24 goals, best 16/16 at dwell 0.40 (L2926-3355), dwell 0.35 best 2/2, 0.30 not flown. No fail condition observed. |

Ends over the 24 oval goals: MISSED_CATCH 17, LANE_REFUSED 2, GUARD_LATCHED 1, REJECTED_NO_BALL 3 (Ball Butler throw 6-17 % slow, the catch did not seat), REJECTED_FRAME_OFFSET 1 (Platform still settling, 2.19 mm against 2.0 mm). Leg refusals: 0 LIMIT_VEL, 0 LIMIT_ACC; one LIMIT_JERK on a CATCH re-aim resend in the canary (510456 against 200000, L983-984); the first plan stood and the ball was caught. The launch log carries no BridgeIdentity, gate_hash or Sitting-limits line, so the pre-arm checks are unconfirmed; the no-ball rehearsal goal was not sent. `docs/records.md` unchanged.

Nothing merged and nothing held: JB-162 has no branch.

## Findings

**1. The oval's long Throws (17 of 24 ends).** Every oval Throw goes from the -75 Site to the +75 Site, so the owner's "+x" is "long". Raw landing (landed minus the learner command u):

| group | n | raw x mean +/- sd |
|---|---|---|
| Throw 1, from rest | 19 | +47.8 +/- 15.1 mm |
| Throws 2+, after a carry | 56 | +76.2 +/- 17.5 mm |
| carried, by dwell | | +73 (0.40), +122 (0.35) |
| misses with a tracker fit | 15 | +87 .. +152 mm |

From the bag (encoder FK and mocap at release, medians): the commanded Platform velocity along +x at release is +46 mm/s in every group, but the measured Platform runs faster by +20 (from rest), +39 (dwell 0.40) and +79 mm/s (0.35) on the encoders, +39 / +75 / +131 on mocap; the ball's launch-velocity error matches mocap almost 1:1 (51/39, 78/75, 131/131 mm/s; flight ~0.93 s). The carry back from +75 to -75 peaks at -707 mm/s (0.40) / -790 (0.35) and its commanded velocity crosses zero only 30 / 25 ms before release; the measured Platform velocity peaks forward +66 / +105 mm/s at +30 / +25 ms. That is the 2026-10-31 ~6 Hz lateral ring (`2026-10-31-hold-ladder-sitting-the-hold-model-was-wrong.md`), excited by a deceleration that runs into the release. At dwell 0.50 (hold 0) on 2026-10-31 Throws landed +46 +/- 10: the same shape with a gentler deceleration. The analysis units first read that entry's "carry ends ~100 ms before release" as the dwell-0.50 rung; it belongs to the hold-0.100 rungs (corrected by the root-cause investigation below).

The learner pushed the right way (u_x -18 -> -26 -> -30 mm) and was pinned at its 30 mm authority from goal 4 (`BOX-EXPECTED-LANDING-UNMET` on every later Throw, y_hat +32..+52 mm); the catch reaches 10 mm outward. About 40 mm of correction against a +76 mm mean: the misses are the tail. Most Attempts died at the second Throw, the first carried one.

**2. Firmware scheduler STOPs on a clean host stream (3 ends).** Goals 3 (GUARD_LATCHED, Hand MAX_DEVIATION -3.057 rev), 9 (LANE_REFUSED) and 15 (LANE_REFUSED, then Hand MAX_DEVIATION +7.660 rev). Forward gap 37-47 ms, loop gap at most 20 ms, emit max 34 ms, bridge frame age max 24.8 ms; the firmware's expired, superseded and stale counters did not move. The Hand braked on its profile at -23..-24 A (peak |iq_set| 54.1 A Sitting-wide against 70 A, no SPINOUT_DETECTED). FW 37's C1 stop fired twice (goals 9, 15) and stopped the Hand inside the stroke (rest 8.71 and 1.69 rev, clip 10.501). Goal 15's guard tripped 68 ms after the skill END; the host hold landed 214 ms after it.

## Discussion

- **Withdrawn: JB-161's feed-catch mechanism, for the oval.** The first Throw after the feed catch was the best Throw (+47.8 raw, caught 19 of 19), and the 7 clamped feed catches were each followed by a normal Throw 1. The killer is the first CARRIED Throw. JB-161's two-ball Columns case is not tested by this data; the carry-ring probe is its next check.
- **Withdrawn: JB-147 (braking torque lost) and JB-149 (host starvation) for the STOP ends**, the second Sitting running for JB-149. The cause of the cover expiry is firmware-side and unknown; filed as its own bug rather than kept under JB-149's refuted title.
- **Refuted: release timing (-7..+18 ms on caught and missed alike) and the learner pushing the wrong way.**
- **Not settled: did P4 make the oval worse?** Dwell and the leg point changed together; the install peaks (leg vel at most 49 %, acc at most 33 % of 600 / 8000) would fit inside 350 / 5000. One Attempt at dwell 0.50 at 600 / 8000 separates them; it is now the first oval step of JB-162.
- **Owner decisions 2026-11-04.**
  - (1) Next Sitting: dwell 0.50 first, then the same 0.40 / 0.35 ladder.
  - (2) Catch outward authority 10 -> 30 mm declined as a band-aid: it widens the net for a wrong Throw and lengthens the carry that rings.
  - (3) Learner authority 60 mm for the oval: on hold until the root-cause investigation reports. The scoping found that the Throw command already passes the box edge by the authority (the forward clip), so a re-sweep would not be needed. The single `learner_lateral_authority_mm` also bounds catch re-aims and the gated capture cap, so it would need a Throw-only parameter.
  - (4) The planner root cause (JB-102): an Opus investigation the same morning, offline only, no code changed. Its report is summarised under Root cause.

## Root cause (offline investigation, 2026-11-04)

- **Cause, not a defect.** `compile_one_ball` sets `t_release = t_catch + dwell_s`, so the 150 mm carry must fit in the Dwell. The window is one STEADY QP (`cup_cycle._assemble` minimises cup acceleration, `_pose_augment` adds leg jerk) with the release pinned. Its minimum-energy carry uses the whole Dwell and decelerates into the release knot: commanded v_x = +0.1 mm/s at release at every Dwell, Platform x deceleration 2.6 / 4.4 / 5.9 m/s^2 at 0.50 / 0.40 / 0.35. Nothing in the program sees the ~6 Hz mode, and the legs carry no acceleration feedforward.
- **Model.** A 6 Hz, damping 0.17, 15 ms-delay tracking model driven by the commanded acceleration gives a ring share of +24 / +51 / +79 mm at 0.50 / 0.40 / 0.35. It reproduces two of three 2026-10-31 hold-ladder differences within ~15 % and misses the 0.075 rung by ~20 mm/s. The remaining ~+22 mm common offset (also on Throws from rest) is not the ring. The commanded +46 mm/s at release is the same in every group and fits the composition term, not carry residue. The MuJoCo plant has no such mode, so a sim replay cannot show it.
- **Ranked fixes.**
  - (1) Root cause: zero-residual-vibration (ZVD) rows on the carry in `cup_cycle._pose_augment`, with the mode in `hardware_config.yaml`. Gated: re-hash and box re-sweep; needs the mode identified on hardware first. 1-D prototype: ring 0 +/- 12 mm/s over a +/-15 % frequency error. Cost at 0.40: acc x1.2, jerk x1.8, inside 600 / 8000 / 200000. At 0.35 leg jerk ~175k, near the limit.
  - (2) Flyable now, compensation by phase: `pre_release_hold_s` 0.050, predicted about -60 mm at 0.40. Phase-fragile; the box cells were swept at hold 0, though live validation gates every segment.
  - (3) Leg acceleration feedforward: reopens the superseded accel-FF arc.
  - (4) Fold the ring into the commanded landing (a carried flag in the learner context, or the 60 mm authority): removes the mean, not the scatter.
  - (5) Dwell 0.45-0.50: a workaround.
- **What a Sitting must measure.** Mode identification: dwell 0.40 at hold 0 and hold 0.050, mocap and encoders, fit frequency and damping from the free decay. The model predicts carried bias +73 -> about +13 +/- 25 mm with the hold.

## Follow-ups filed

- Bug: Firmware scheduler STOP fires on a clean host stream and ends the Attempt.
- Feature: Install the host hold within one tick of a firmware scheduler STOP (owner decision: next to the Teensy guard).
- Idea: Learner context cannot tell a Throw from rest from a carried Throw.

Progress notes: JB-162 (Sitting section rewritten for the re-fly), JB-102, JB-161, JB-139, JB-24, JB-145, JB-147, JB-149, JB-98, JB-160.

## Verification

- (2026-11-04, `grep -n 'ENDED (.*caught'` and `grep -n 'Juggle requested.*goal dispatched'` on the launch log, result: 25 goals, 24 ends plus the canary's COMPLETED 20/20 at L1008).
- (2026-11-04, mcap_ros2 over `/robot_state` `motor_states[0..5].iq_measured` in the bag, result: per-leg max 6.2 / 11.0 / 11.7 / 8.1 / 7.1 / 9.6 A, 0 samples at or above 15 A).
- (2026-11-04, per-throw parse of `throw k/N CAUGHT` lines against the `memory row appended` command before each, result: 75 caught Throws, raw x +47.8 +/- 15.1 from rest, +76.2 +/- 17.5 carried).
- Gate: see the commit message.
