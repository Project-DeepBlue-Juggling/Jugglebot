---
id: BUG-160
kind: bug
title: Ball Butler's Feed of Ball 2 lands 25-29 mm short of its announced landing in Columns
status: inbox
created: '2026-11-28'
updated: '2026-11-28T07:58:28Z'
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
- why: Ball Butler's Feed of Ball 2 lands about 25 mm short and outward of the Site, beyond the Catch authority; trimming the aim in software should stop the Feed losses without moving the Catch.
- do:
  - Before arming: `ros2 param set /skill_node columns_feed_bb_bias_mm "[-25.0, 5.0]"` (read at use, no restart).
  - Before arming: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns,apex_m=0.95,separation_mm=125,dwell_s=0.27'}"` and read the margins.
  - Fly 6 Columns Attempts (Sep. 125, Apex 0.95, Dwell 0.27, Throws 21) at the default 10 mm Catch authority.
  - Afterwards `ros2 param set /skill_node columns_feed_bb_bias_mm "[0.0, 0.0]"` unless it passed.
- pass: median |dx| of Ball 2's Feed at the Catch plane, measured from the bag against the schedule Site (not against the Cup, which follows the ball), is 10 mm or less, and no more than 1 of 6 Attempts is lost to Ball 2's Feed.
- record: the end code and caught of thrown per Attempt, and the Ball 2 Feed dx and dy per Attempt.
- minutes: 20

## Progress

- 2026-11-28: The BUG-161 re-aim fix is on main, so at the default 10 mm authority the Cup now moves to the clamp edge on a short Feed. Judge the trim on the Feed's dx against the Site, which the fix does not change; read the lost-Feed count with that in mind.
