---
id: IDEA-22
kind: idea
title: 'Turned Columns at 90 deg: tilt the Platform back (+x) at the Catch so the ball stops hitting the Platform top'
status: complete
created: '2026-11-10'
updated: '2026-11-11T19:00:02Z'
owners: []
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
- logbook/2026-11-12-turned-columns-catch-tilt-back.md
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
- why: Turned Columns at 90 deg failed on 2026-11-10 (4 Attempts, all MISSED_CATCH within 8 Throws, the ball striking the Platform top). The branch tilts every auto-banked Catch back toward +x; the Feed Catch stays Level.
- do:
  - Answer TKT-65 on the default first (4 deg as built, or 3 deg).
  - Mocap up on the robot base markers and Ball Butler's Base markers (columns_turned picks the fed Site from Ball Butler's position).
  - Rehearse: `ros2 param set /skill_node columns_angle_deg 90`, then `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns_turned,apex_m=0.95,separation_mm=125,dwell_s=0.27,columns_angle_deg=90'}"`. Nothing moves.
  - Offline at P4 limits 4 deg is refused (LIMIT_JERK) and 3 deg passes at margin 0.024. If refused, `ros2 param set /skill_node columns_turned_catch_tilt_back_deg 3.0` and nominate again; if 3 is refused too, use 2.0.
  - Fly 4 Attempts of columns_turned at 90 deg (Apex 0.95, Sep. 125, Dwell 0.27) at the highest tilt-back the nomination accepted. Watch where the ball meets the Cup and whether the Cup slides sideways at the Catch.
  - If time allows, 1 Attempt at 45 deg as a control: it compiles exactly as before.
- pass: At 90 deg, at least one of 4 Attempts passes Ball 2's first Catch after the Feed and carries 10 or more Throws, with no Platform-top strike seen.
- record: the tilt-back flown and its nomination margin (or refusal text), Throws caught per Attempt and each end reason, any Platform-top strike, any sideways Cup slide at the Catch, the 45 deg control.
- minutes: 25
- branch: night-shift/2026-11-12-IDEA-22

## Progress

- 2026-11-11: Night Shift 2026-11-11: options written; awaiting the owner's choice in the question ticket
- 2026-11-12: Night Shift 2026-11-12: held on night-shift/2026-11-12-IDEA-22: tilt-back built, 4 deg default refused at P4 offline (3 passes); unblocked by your call on the default and a Sitting
