---
id: BUG-160
kind: bug
title: Ball Butler's Feed of Ball 2 lands 25-29 mm short of its announced landing in Columns
status: in_progress
created: '2026-11-28'
updated: '2026-11-28T14:24:11Z'
owners: []
priority: high
effort: s
hardware: robot
owner: none
radius: subsystem
diagnosis: hypothesis
area: ball-butler
blocked_by: []
links:
- logbook/2026-11-28-first-flight-of-the-learner-merge-and-the-15-mm-catch-authority-trial.md
- bug-two-ball-columns-runs-end-on-a-missed-catch-of-ball-2-with
parent: learner-generalisation
filed_by: claude
---

Ball Butler's Feed of Ball 2 in Columns lands about 25 to 29 mm short of the point it announced, so it arrives outward of Ball 2's Site, past the 10 mm outward Catch authority. The owner confirmed this matches what he sees and decided to trim it in software.

## Findings

- Free-fall fit of the Mocap ball track to the Catch plane against the Platform body xy, 28 Ball 2 Feeds (bags 2026-11-28_12-21-52, 2026-11-28_12-31-28, 2026-11-27_19-13-16). Mean dx / sd (mm): -25.1 / 6.8, -16.6 / 5.7, -27.8 / 6.5. Mean dy: +4.9, +7.7, +6.5. +x is toward Ball 1's Site.
- Ball Butler lands 29 +/- 7 mm short of its own announced landing (`/throw_announcements`) on every Feed. The ball arrives with 6 to 7 % less horizontal speed than announced (drag or a slower exit, inferred, not fitted).
- Arrival time is within about 2 ms of the Schedule, so timing is ruled out.
- In the first and last bags every Feed with dx <= -26 mm was lost (11 of 11). Ball 2 Feed lost: 6 of 10, 7 of 10 and (with the 15 mm authority) 2 of 8.
- Ball 1's Feed has the same bias but 30 mm of Catch authority, so it is caught.
- `columns_feed_bb_bias_mm` was [0, 0] in all three Sittings.

## Sign

`columns_feed_bb_bias_mm` is `landing - request` in the schedule frame, and Ball Butler is asked to hit `aim_site - bias` (docstrings in `skill_node.py` and `executor.py`). With the bias at 0 the Feed lands at aim + (-25, +5), so the bias to set is the measured miss, [-25.0, +5.0]: Ball Butler is then asked for aim + (25, -5) and lands on the aim. Recipe for later trims: new bias = bias in force + measured dx, dy.

## Unknown

- Why the bias reads about -25 mm now when it read +4 mm on 2026-10-11. No Feed-aim code changed between 2026-11-27 and 2026-11-28, and Ball Butler ran FW 10 throughout.
- Why Attempt 7 of the 2026-11-28 12:31 bag had a clean Feed (landed only 6.8 mm short, same logged throw parameters).

## Sitting

- setup: level, armed, bb-fed
- why: the fixed Feed trim failed because Ball Butler's own static aim correction, not a steady bias, is the shortfall; refit that table against the freshly calibrated Mocap, then check the Feed with the trim at zero.
- do:
  - Before the refit: with the stack up (`ros2 launch jugglebot jugglebot_launch.py record:=false`), Ball Butler calibrated and the hopper loaded, follow `BallButler/zTesting/throw_testing/accuracy_testing/LOCAL_CALIBRATION.md`.
  - Plan: `python run_local_calibration.py plan --out local_calibration_plan.json`.
  - No-motion check: `python run_local_calibration.py run local_calibration_plan.json --schedule-to-mocap DX DY --check-only` (DX DY from the Sitting's Mocap frame check).
  - Throws: `python run_local_calibration.py run local_calibration_plan.json --schedule-to-mocap DX DY --expect-solver-sha <prefix printed by the check>`; the full plan is about 66 minutes with refill pauses, `--padding 300` is a shorter plan (owner's choice).
  - Procedure for the refit to be confirmed with the owner: fit with `python analyze_local_calibration.py <session>/session.json`, copy the validated matrix to `ros_ws/src/jugglebot/resources/throw_affine_correction.json` and rebuild.
  - The 2026-10-09 refit followed that path (Jugglebot logbook 2026-10-09-bb-positive-s-and-refitted-aim-correction); no one script fits and deploys, and no campaign has been run against the new Mocap frame.
  - Before arming: `ros2 param set /skill_node columns_feed_bb_bias_mm "[0.0, 0.0]"` (the default; confirm it reads zero).
  - Before arming: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns,apex_m=0.95,separation_mm=125,dwell_s=0.27'}"` and read the margins.
  - Fly 6 Columns Attempts (Sep. 125, Apex 0.95, Dwell 0.27, Throws 21) at the default Catch outward authority.
  - Quick alternative, one launch: `ros2 launch jugglebot jugglebot_launch.py apply_aim_correction:=false` and fly the same 6 Attempts, to see whether Ball Butler's uncorrected aim lands on the Site.
- pass: median |dx| of Ball 2's Feed at the Catch plane, measured from the bag against the schedule Site (not against the Cup, which follows the ball), is 10 mm or less, and no more than 1 of 6 Attempts is lost to Ball 2's Feed.
- record: the end code and caught of thrown per Attempt, the Ball 2 Feed dx and dy per Attempt, the new matrix and its fit residual, and the solver sha the campaign used.
- minutes: 90

## Sitting result (2026-11-28)

- verdict: fail
- notes (split into bullets):
  - Attempts:
  - 1 - Finished without issue. Some catches were pretty aggressive on the leg joints and the base seemed to move around a bit, with Leg 2 looking close to separating, but it held for the whole Attempt!
  - 2 - Ball2 feed collided with the Platform.
  - 3 - Finished, though Leg 1 was extremely close to being pulled from the Base on the last catch. Ball2 feed was very smooth.
  - 4 - Ball2 feed collided with the Platform
  - 5 - Mid-air collision right at the start
  - 6 - Ball2 feed collided with the Platform
  - (sheet 2026-11-28-robot, step 7)
  - derived: bag 2026-11-28_19-38-37, six Attempts of step 7: Ball 2 Feed dx against the Site +5.6, +14.1, +16.6, +13.3, +20.5, +24.2 mm (schedule frame), median |dx| 15.3 mm; Attempts 2, 4 and 6 ended MISSED_CATCH at skill 3 (launch log 2026-11-28-19-38-37), so 3 of 6 lost to the Feed.
  - derived (cont.): The trim moved BB's aim 25 mm inward as intended, but BB's own shortfall this evening was about 8 mm (Attempts 1-12) and about 35 mm (13-16), not the morning's steady 25-29 mm.

## Evening Sitting 2026-11-28: fail, the steady-bias hypothesis is withdrawn

- Trim `[-25.0, 5.0]` flown on 6 Attempts: Ball 2 Feed dx against the Site +5.6, +14.1, +16.6, +13.3, +20.5, +24.2 mm, median |dx| 15.3 mm (bar 10); Attempts 2, 4 and 6 lost to the Feed (bar: at most 1 of 6).
- The trim did what it asked: Ball Butler was told to land at the Site minus the bias, and the Catch prior stayed at the Site.
- Ball Butler's shortfall against its own announced landing was not steady: morning -29.2 mm and -24.6 mm, evening Attempts 1-12 -8.2 mm (sd 8.7), Attempts 13-16 -35.0 mm (sd 4.6). Nothing in the launch log changed between Attempts 12 and 13; the jump is unexplained.
- Lead: Ball Butler applies a static fitted aim correction (`ros_ws/src/jugglebot/resources/throw_affine_correction.json`, 116 pairs, last refit 2026-10-09) to its target just before the throw solve, and the announcement is built from the uncorrected target. At the Columns Feed target it amounts to about (-30, -3) mm in robot x, y.
- In the morning the ball landed at announced plus that correction to within 0.8 and 5.4 mm in x, so the correction is the shortfall and an uncorrected aim would have been about on target. Double application is ruled out (it would read about -60 mm).
- The Mocap was re-calibrated before the Sitting: in the Robot frame Ball Butler's base origin moved (+10.5, +14.4) mm and its heading +0.8 deg, about 10 mm of lateral aim at 750 mm range (inferred, not demonstrated).
- Owner decision 2026-11-29: refit the table against the new Mocap before flying Columns again; the software trim returns to [0, 0].

## Progress

- 2026-11-28: The BUG-161 re-aim fix is on main, so at the default 10 mm authority the Cup now moves to the clamp edge on a short Feed. Judge the trim on the Feed's dx against the Site, which the fix does not change; read the lost-Feed count with that in mind.
- 2026-11-29: Failed at the Sitting of 2026-11-28 (evening): trim [-25, 5] left Ball 2 Feed median |dx| 15.3 mm against the 10 mm bar, 3 of 6 Attempts lost to the Feed. Steady-bias hypothesis withdrawn; lead is BB's static aim correction table; refit it against the new Mocap (detail below)
