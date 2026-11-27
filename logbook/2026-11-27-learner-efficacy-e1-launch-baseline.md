---
title: "Learner efficacy E1 (FEAT-68): a hand-placed, settled ball launches with the same 15 mm/s scatter as a running Throw at any Dwell, so the launch scatter arises at separation and seating is not its cause"
type: investigation
date: 2026-11-27
status: resolved
phase: "learner-efficacy E1 (PWAY-2)"
related_plan: learner-efficacy.md
related_entries:
  - 2026-11-25-learner-efficacy-e0-unsettled-ball-test.md
  - 2026-11-25-learner-efficacy-rescoped-to-the-hand.md
  - 2026-11-25-release-probe-blocks-report.md
files_changed:
  - tools/probes/release_settle_bag_probe.py
  - plans/active/learner-efficacy.md
  - map/feature-bench-hand-launch-baseline-launch-scatter-of-a-settled-a.md
subsystem:
  - tracking
  - hand
tags:
  - hardware
  - learner
---

# Learner efficacy E1: the launch baseline on the robot's Hand

## Summary

FEAT-68 was flown on 2026-11-27 (bag `2026-11-27_18-55-16`, plant `e1-2026-11-27-FEAT-68`, every Throw
caught). Its question: does the launch scatter come from the ball not seating during the Dwell (the owner's
account), or does it arise as the ball separates from the Cup, settled or not (E0's reading)?

The answer is separation on the point estimates, and seating is rejected outright. A ball placed by hand and rested for over a second leaves the Cup with the same
lateral scatter as a ball thrown 0.27 s after a Catch.

| Block | Throws | Launch sd x / y (mm/s) | Pooled (95 %) | Variance against S (95 %) | Landing sd x / y (mm) |
|---|---|---|---|---|---|
| S, hand-placed | 30 | 13.3 / 16.3 | 14.9 (12.2..17.2) | - | 14.0 / 15.6 |
| first of a 12-Throw Attempt | 17 | 18.4 / 13.6 | 16.2 (12.2..19.9) | 1.18 (0.61..2.07) | - |
| D27 | 66 | 13.0 / 17.6 | 15.5 (14.2..16.6) | 1.08 (0.78..1.62) | 16.3 / 17.4 |
| D50 | 66 | 14.0 / 15.4 | 14.7 (13.3..16.3) | 0.98 (0.69..1.52) | 15.4 / 16.1 |
| D100 | 55 | 16.2 / 13.7 | 15.0 (13.2..16.7) | 1.01 (0.68..1.59) | 16.4 / 15.1 |

Launch sd is the Platform-corrected figure of the pre-registered criterion. Landing sd is Memory's measured
landing about its block mean, which needs no correction and no regression.

## Against the pre-registered criterion

- **Seating** needed S at 6 mm/s or less on both axes and D27 at twice S's variance or more. S is 13.3 and
  16.3 mm/s, with a lower bound of 12.2 on the pooled figure, and D27 is 1.08 times S with an upper bound of
  1.62. Both parts fail, on the intervals as well as the point estimates.
- **Separation** needed S at 12 mm/s or more on both axes with every Dwell block within 1.5 times S's
  variance. Every point estimate meets it: the ratios are 1.08, 0.98 and 1.01.
- The `blocks` report prints UNDECIDED. Its rule is stricter than the criterion: it asks the whole 95 %
  interval of each ratio to sit below 1.5, and the upper bounds are 1.52 to 1.62. That rule is not met.

The result is recorded as Separation on the point estimates. Block S is not re-flown: that is a judgement,
not something the criterion forces. More settled Throws would only narrow an interval whose centre is 1.0,
and the seating account is already excluded. Pooling the three Dwell blocks (a reading made after the fact, not
pre-registered) gives 15.1 against 14.9 mm/s, a variance ratio of 1.02 (0.75..1.54).

## Discussion

**The settled block was settled.** In block S the ball's lateral velocity in the Cup over the last 35 ms
before release has an sd of 1.8 / 3.1 mm/s, which is the measurement floor E0 found for a ball at rest, and
its offset from the Hand axis varies by 0.3 / 0.6 mm between Throws. The running blocks read the same
(1.6 / 2.0 mm/s at Dwell 0.27). So in every block the ball is at rest in the Cup just before release, and it
still leaves with about 15 mm/s of scatter.

**The scatter is real and reaches the landing.** The launch residual times the flight time correlates with
Memory's measured landing at r 0.82 to 0.86 in block S and 0.57 to 0.74 in the Dwell blocks. The landing
scatter itself, read straight from Memory, is 14 to 17 mm per axis in every block, settled or running.

**What the correction does and does not do.** The raw launch sd of the running blocks is 26 to 33 mm/s
against 16 to 18 for block S. That difference is the Learner's own corrections: the commanded Cup velocity
varies by 14 to 17 mm/s between running Throws and by under 3 in block S. Removing the command alone brings
the running blocks to 15 to 22 mm/s; the measured Platform motion brings every block to the table's figures.
The landing column is the check that this is not an artefact of the fit.

**The owner's two observations.** The owner saw the Platform wiggle just before the Throw in block S and
asked whether the yaw rate the IMU shows could be the cause.

- The wiggle is real. At release the Platform's Mocap centroid moves at 17.5 mm/s in x where the command
  asks for 2.6. It is the same on every Throw: its sd in block S is 3.2 / 2.2 mm/s. A repeatable motion
  shifts the mean launch, which the Learner already corrects (the launch runs 16 / 14 mm/s off the command
  in every block), and it cannot make one Throw differ from the next by 15 mm/s.
- Yaw adds nothing. The Platform's yaw rate at release, from Mocap, has an sd of 0.014 rad/s in block S and
  0.03 in the running blocks. Added to the fit it leaves the residual unchanged (14.54 to 14.55 mm/s in x,
  15.68 in y). A yaw about the Hand axis also cannot move a centred ball sideways: at the 0.6 mm offsets
  measured, 0.03 rad/s is 0.02 mm/s.
- All the measured Platform motion together (velocity, tilt and tilt rate at release) explains about a fifth
  of the launch variance in block S (16.1 / 17.8 to 13.6 / 16.4 mm/s). The other four fifths are not in
  anything Mocap sees of the Platform.

**What was withdrawn.** The preview on the 2026-11-13 bag (settled 12.4 against running 17.2 mm/s, variance
ratio 1.93) suggested a seating share. It compared different Sites and Patterns; with one Site, one Apex and
mirrored blocks the ratio is 1.0. That lead is withdrawn. The owner's seating account is not supported
either: the owner also noted that with the new gravity offset the Catches were smoother with far less
movement in the Hand, and the scatter did not fall with it.

**The off-axis arm.** D27 binned by where the ball came in at the Catch gives 16.3, 17.2 and 12.5 mm/s for
the near, middle and far thirds. A ball that lands further off-axis does not launch with more scatter. The
entry offset was not validated in E0, so this is a lead only.

**What is not measured.** Mocap sees the Platform, not the Cup. Lateral motion of the Cup relative to the
Platform during the stroke (play or flex in the Hand's guide, which the roughened motor may add to) would
look exactly like this result, and so would the ball's contact with the Cup wall as it lifts off. This
Sitting cannot tell those two apart.

## What changed

- `tools/probes/release_settle_bag_probe.py`: the default Memory path now reads `temp/runtime/learn` (where
  the robot has written since 2026-11-26) as well as `temp/learn`, each row once. Without it the probe found
  no Memory row for this bag and extracted nothing. Two columns added, `myaw` and `mwz` (Platform yaw and
  yaw rate at release); they are not block regressors, and the `blocks` output is identical with and without
  them.
- Plan: E1 complete; E4 is aimed at the release, and its first step is to tell Cup motion from ball contact.
- FEAT-68 closed.

## Verification

- 2026-11-27, `python tools/probes/release_settle_bag_probe.py extract ~/Desktop/rosbags/2026-11-27_18-55-16`
  then `features`: 246 Throws, 246 with a launch fit, 246 with an in-Cup series.
- 2026-11-27, `python tools/probes/release_settle_bag_probe.py blocks temp/probes/release_settle/2026-11-27_18-55-16.csv --skip-attempts 1`:
  234 Throws in 47 Attempts, the table above, verdict line UNDECIDED on the interval rule. Output saved to
  `temp/logs/e1-blocks-2026-11-27.log`.
- 2026-11-27, `features` re-run after `myaw` and `mwz` were added, then `blocks` as above piped to
  `diff - temp/logs/e1-blocks-2026-11-27.log`: no difference.
- 2026-11-27, `python tools/probes/release_settle_bag_probe.py run ~/Desktop/rosbags/2026-11-27_18-55-16`
  before the Memory path fix: `{'cmd': 0, 'ht': 0, 'plat': 0, 'unl': 0, 'ann': 247, 'clk': 35}` and an
  IndexError in `features`. After it: cmd 38827, ht 36203, plat 464234, unl 860233.
- Block D100 holds 55 Throws in 5 Attempts, one Attempt short of the six planned; the pass bar was 50.
