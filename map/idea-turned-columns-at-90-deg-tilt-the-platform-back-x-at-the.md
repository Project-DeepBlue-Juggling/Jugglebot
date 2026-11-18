---
id: IDEA-22
kind: idea
title: 'Turned Columns at 90 deg: tilt the Platform back (+x) at the Catch so the ball stops hitting the Platform top'
status: in_progress
created: '2026-11-10'
updated: '2026-11-18T02:09:10Z'
owners:
- night-shift
priority: medium
effort: m
hardware: robot
owner: hands
radius: subsystem
area: skill-stack
blocked_by: []
links:
- idea-jp-10-turned-columns-the-columns-pair-at-fixed-angles-about
- bug-ball-2-s-first-throw-after-the-feed-catch-lands-about-180-mm
- logbook/2026-11-13-sitting-debrief.md
- logbook/2026-11-18-sitting-debrief.md
parent: pattern-progression-columns-to-centre-and-orbit
filed_by: claude (sitting debrief 2026-11-10)
---

Owner, Sitting of 2026-11-10 (sheet step 8, IDEA-11): "45 deg passed perfectly, but 90 deg once again didn't succeed. I think we need to make the 90 deg version tilt the Platform slightly back (+ve x) when catching, to stop the ball hitting the top of the Platform so often."

Evidence the same night (launch log 2026-11-10-20-11-22, bag ~/Desktop/rosbags/2026-11-10_20-11-22): 4 Attempts at 90 deg, 8/8, 3/3, 3/3, 3/3, all MISSED_CATCH on an empty Cup; three ended at Ball 2's first Catch after the Feed with the Tracker fit 47, 26 and 14 mm outward along -y and 9-23 mm in -x. The 2026-11-03 and 2026-11-07 notes on IDEA-11 report the same Platform-top hits at 90 deg only.

Questions before software: is the Catch tilt a per-Site constant (a Catch plane tilted about the pair axis) or does it follow the arrival direction; how many degrees clear the Platform top at a 125 mm separation; does the pose planner's admissible box at 90 deg (Apex 0.875-0.975) survive the tilt at the Sitting limits (a nominate probe answers it offline).

## Options (2026-11-11)

What exists: every Catch is already banked toward the ball's arrival (`tilt_geometry.tilt_to_receive`, the Cup axis anti-parallel to the arrival, clamped at 12 deg). A Catch can also be pinned to a fixed attitude (`Skill.receive_tilt`); today only the Feed Catch is, pinned Level.

1. **A constant Banking offset for turned Columns:** a Pattern parameter (degrees about the pair axis, +x as you asked) added to the auto-banked Catch tilt on every Throw's Catch. The Feed Catch stays pinned Level. Offline first, a nominate probe at 90 deg shows whether the admissible box (Apex 0.875 to 0.975) survives the extra tilt at the Sitting limits. Then a Sitting row.
2. **A fixed Catch attitude per Site at 90 deg:** pin `receive_tilt` instead of auto-banking. Simpler to reason about, but it gives up the arrival-matched seat that keeps the ball from skidding in the Cup.
3. **Measure first, no code:** read the 2026-11-10 20:11 bag for the commanded Catch tilt at 90 and 45 deg, whether the 12 deg clamp binds at 90, and where each miss struck. The Night Shift can do this offline.
4. **Fix the landing first:** the three ends were Ball 2's first Catch after the Feed, 14 to 47 mm outward along -y, which is BUG-63's outward first Throw. If the ball lands where it should, it may stop hitting the Platform top without any tilt.

Recommendation: 3, then 1. The read is cheap, and it shows whether the misses are an attitude problem (1) or a landing problem (4).

## Sitting

- setup: level, armed, bb-fed
- why: Turned Columns at 90 deg failed on 2026-11-10 (4 Attempts, all MISSED_CATCH within 8 Throws, the ball striking the Platform top). The branch adds a rotation about +x to every auto-banked Catch (the Cup leans toward -y), 3 deg by default (TKT-65); the Feed Catch stays Level.
- do:
  - TKT-73 answered (2026-11-16, options 1 and 3): fly tilt-back 0 deg at Apex 0.975 first as the control, then 0.5 deg at Apex 0.975. Offline (Findings 2026-11-15) both clear the re-aim figure: 0 deg +0.350 / +0.131, 0.5 deg +0.350 / +0.146.
  - Mocap up on the robot base markers and Ball Butler's Base markers (columns_turned picks the fed Site from Ball Butler's position).
  - Rehearse: `ros2 param set /skill_node columns_angle_deg 90` and `ros2 param set /skill_node columns_turned_catch_tilt_back_deg 0.0` (a double: write 0.0, not 0), then `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns_turned,apex_m=0.975,separation_mm=125,dwell_s=0.27,columns_angle_deg=90'}"`. Nothing moves. Expect an accepted line with no WARNING: RE-AIM MARGIN NEGATIVE.
  - Fly 2 Attempts of columns_turned at 90 deg (Sep. 125, Dwell 0.27, Apex 0.975) at tilt-back 0.0. Watch where the ball meets the Cup, any Platform-top strike, and whether the Cup slides sideways (toward +y) at the Catch.
  - Between Attempts, `ros2 param set /skill_node columns_turned_catch_tilt_back_deg 0.5`, nominate again with the same string, then fly 2 Attempts at tilt-back 0.5.
  - If time allows, 1 Attempt at 45 deg as a control: it compiles exactly as before.
- pass: At 90 deg, at least one of 4 Attempts passes Ball 2's first Catch after the Feed and carries 10 or more Throws, with no Platform-top strike seen.
- record: per tilt-back (0.0, then 0.5) at Apex 0.975: the nomination margin and re-aim figure (or refusal text), Throws caught per Attempt and each end reason, any Platform-top strike, any sideways Cup slide at the Catch; the 45 deg control.
- minutes: 25
- branch: night-shift/2026-11-12-IDEA-22

## Sitting result (2026-11-13 13:42)

- verdict: fail
- notes: (sheet 2026-11-13-robot, step 33)

## Findings (2026-11-15)

Your TKT-68 answer: the offline sweep on the held branch (`columns_turned` 90 deg, Sep. 125, Dwell 0.27, hop entry, P4 limits 600/8000/200000/4500, `nominate(search=False)` with the re-aim figure at the skill_node default authorities). Run by the Night Shift on 2026-11-15; the three known cells reproduced within 0.011 (3 deg at Apex 0.90: +0.027 / -0.080). Each cell: nominal margin / margin with the Catch re-aim at its limit.

| tilt-back | 0.850 | 0.875 | 0.900 | 0.925 | 0.950 | 0.975 |
|---|---|---|---|---|---|---|
| 0 | +0.279 / -0.051 | +0.279 / -0.017 | +0.300 / +0.050 | +0.304 / +0.068 | +0.346 / +0.118 | +0.350 / +0.131 |
| 0.5 | +0.280 / -0.033 | +0.280 / -0.009 | +0.288 / +0.053 | +0.287 / +0.093 | +0.346 / +0.132 | +0.350 / +0.146 |
| 1 | +0.185 / -0.068 | +0.191 / -0.110 | +0.261 / +0.027 | +0.254 / +0.082 | +0.232 / +0.097 | +0.237 / +0.096 |
| 1.5 | +0.059 / -0.102 | +0.068 / -0.097 | +0.235 / +0.029 | +0.229 / +0.054 | +0.109 / -0.027 | +0.107 / -0.025 |
| 2 | refused (margin) | refused (margin) | +0.208 / +0.014 | +0.204 / +0.027 | refused (margin) | refused (margin) |
| 2.5 | refused (jerk) | refused (jerk) | +0.123 / -0.011 | +0.136 / +0.000 | refused (jerk) | refused (jerk) |
| 3 | refused (jerk) | refused (jerk) | +0.027 / -0.080 | +0.040 / -0.026 | refused (jerk) | refused (jerk) |

- Only 0.5 deg reaches the debrief's +0.10 at the re-aim limit: +0.132 at Apex 0.95, +0.146 at 0.975. Tilt 0 reads +0.118 and +0.131 there, so 0.5 deg is free in this model but small.
- 1.0 deg falls just short (+0.097); 1.5 deg and above go negative at Apex 0.95 and up; 3 deg has no cell above 0 at the re-aim limit.
- Apex below 0.90 is negative at the re-aim limit even at tilt 0. The refusals are LIMIT_JERK on Leg 1 at the second Catch (212k to 227k against 200k).
- Next: TKT-73 (which tilt and Apex to fly, or park 90 deg).

## Sitting result (2026-11-18 13:05)

- verdict: data
- notes (split into bullets):
  - No attempts succeeded through completion, and it was difficult to visually distinguish these attempts from those with tilt_back_deg 0.0, though there were a handful of throws that collided with the top of the Platform. Many of these were recovered from, however some led to the following throws being messy, which cascaded to failure.
  - Overall, this seemed less reliable than tilt_back_deg 0.0 (sheet 2026-11-18-robot, step 39) [derived: temp/logs/launch_2026-11-18_3.log 11:54-11:55 and launch_2026-11-18_3-1.log. Nomination margin +0.350, with re-aim +0.131 (tilt-back 0.0) and +0.146 (0.5). 0.0 deg: 6 Attempts caught 19, 6, 18, 7, 5, 20. 0.5 deg: 5 Attempts caught 7, 9, 11, 14, 9, all MISSED_CATCH. Ball 2's first Catch after the Feed passed at both settings.
  - The operator saw a handful of Platform-top strikes at 0.5; the sample cannot separate the two settings. The 3 deg default is refused at Apex 0.975.]

## Progress

- 2026-11-11: Night Shift 2026-11-11: options written; awaiting the owner's choice in the question ticket
- 2026-11-12: Night Shift 2026-11-12: held on night-shift/2026-11-12-IDEA-22: tilt-back built, 4 deg default refused at P4 offline (3 passes); unblocked by your call on the default and a Sitting
- 2026-11-13: Night Shift 2026-11-13: held on night-shift/2026-11-12-IDEA-22: tilt-back now about +x at 3 deg (TKT-65), refused at P4 offline (1.9 passes); unblocked by TKT-67 and a Sitting
- 2026-11-13: Failed at the Sitting of 2026-11-13: apex 0.90, tilt-back 3 deg (nominate +0.027, -0.080 at re-aim limit); 5 Attempts all ended at the first Catch after the Feed (LIMIT_JERK x3, NO_ADMISSIBLE_COMMAND x2), best 2. "3 deg tilt-back makes 90 deg flyable" refuted at today limits; Platform-top not reached. Branch origin/night-shift/2026-11-12-IDEA-22 stays held (BUG-91).
- 2026-11-15: Night Shift 2026-11-15: offline sweep done (TKT-68); only 0.5 deg clears +0.10 at the re-aim limit, Apex 0.95-0.975; branch stays held; unblocked by TKT-73 and a Sitting
- 2026-11-16: Night Shift 2026-11-16: TKT-73 answered (fly 0 deg then 0.5 deg at Apex 0.975); Sitting steps rewritten; held on night-shift/2026-11-12-IDEA-22; unblocked by a Sitting
- 2026-11-18: Flown at the Sitting of 2026-11-18, verdict data: at 90 deg, Apex 0.975, tilt-back 0.0 caught 19, 6, 18, 7, 5, 20 and 0.5 caught 7, 9, 11, 14, 9 with Platform-top strikes seen; 0.5 was not better. The branch default of 3 deg is REFUSED at Apex 0.975. Owner decision: the default tilt-back (0.0, or drop the feature). Branch stays held.
