---
id: IDEA-11
kind: idea
title: 'JP-10 Turned columns: the columns pair at fixed angles about +z, under pose'
status: in_progress
created: '2026-10-30'
updated: '2026-11-10T12:41:42Z'
owners:
- jugglebot-f5
priority: medium
effort: s
hardware: robot
owner: hands
radius: subsystem
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-10-sitting-debrief.md
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
parent: pattern-progression-columns-to-centre-and-orbit
---

Columns with the Site pair rotated to a fixed angle (30, 45, 60, 90 deg) about the Platform +z.

## Offline probe (2026-10-30)

- The level planner is direction-sensitive: LIMIT_JERK 205k-207k at 45 and 135 deg against 158k and 154k at 0 and 90 deg.
- The pose planner is flat at every angle (107k-112k), so this is pose-only (owner: pose-only is fine).

## Why file it

It proves the rotational symmetry on hardware that lets the admissible box key on the pair's RELATIVE geometry (separation, apex band, dwell, planner mode) instead of exact xy stamps, which is what JP-11 to JP-14 need. Owner 2026-10-30: pick the most efficient robust key, not the traditional one.

## Software

- Site pairs at the new angles (or a rotation parameter on `compile_columns`).
- An admissible-box cell per angle.
- A learner key that rotates into the pair frame.

## Sitting

- setup: level, armed, bb-fed
- why: P3 of plan pattern-progression.md: two-ball Columns off the x axis under the pose planner, one plant_id per angle (set explicitly in the steps below), which answers PWAY-2 P2(b) (does the lateral bias rotate with the pair).
- do:
  - Rehearse with mocap on the robot base and Ball Butler's Base markers (the fed Site needs them): `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns_turned,apex_m=0.95,separation_mm=125,dwell_s=0.27,columns_angle_deg=45'}"`, then the same at `columns_angle_deg=90`. Each reply: `success: True`, nothing moves; `position is unknown` means bring mocap up and retry. Never a Juggle goal.
  - Fly a straight `Columns` control first if this Sitting has not flown one (GUI Columns, Sep. 125, Apex 0.95, Dwell 0.27, Throws 21).
  - GUI: Juggle panel `Columns — turned`, Angle 45, Apex 0.95, Sep. 125, Dwell 0.27, Throws 21 (Start writes columns_angle_deg to the node first). Terminal fallback: `ros2 param set /skill_node columns_angle_deg 45` (refused while a goal runs), then `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns_turned, apex_m: 0.95, separation_mm: 125.0, dwell_s: 0.27, num_cycles: 21}" --feedback`. Only the 0.925 to 0.975 Apex band is admissible at 45 deg.
  - Angle 90 (or `ros2 param set /skill_node columns_angle_deg 90`), then the same goal at Apex 0.95 (0.90 is also admissible at 90 deg).
  - Set the plant per angle before each angle's first goal (never set on 2026-11-01 or 2026-11-03, so PWAY-2 P2(b) is unanswered): `ros2 param set /skill_node plant_id <name>` with a distinct name per angle, for example `turned45` and `turned90`, refused while a goal runs; read it back from the goal start line (`plant_id=...`) and write it in the record.
  - Afterwards `ros2 param set /skill_node columns_angle_deg 0`.
- pass: 20 consecutive Catches in one Attempt at each angle, flown with Throws 21 (the 21st throw is the Stop throw, so `COMPLETED (20/20 caught)`; with Throws 20 it reads 19/19); landing scatter, per ball and per axis, with the mean within 15 mm and the sd within 10 mm of the straight control (proposed 2026-11-03, owner to confirm; the old "within 3 mm" was below the control's own run-to-run sd of 18-31 mm); no HAND_LIMIT line.
- record: caught of thrown per angle, each angle's plant_id, landing scatter against the control, Ball 2's first-throw landing (x, y) mm per Attempt (BUG-63: 5 of 7 landed outward at 90 deg on 2026-11-03), the bag path; commit docs/records.md.
- minutes: 20

## Sitting 2026-11-01 findings

- 45 deg: three MISSED_CATCH on an empty cup, two after TRACKER-IDENTITY-REFUSED, one after a half-speed launch (2341 vs 4268 mm/s); one leg LIMIT_VEL 361 > 350 at 0/1.
- Hand iq 53.3 A at L3521 (over the 50 A watch). Tracker apex 1.01-1.04 m turned vs 0.93-0.96 straight.
- No per-angle `plant_id` was set, and no tracker landing on any miss, so PWAY-2 P2(b) is not answered.

## Sitting result (2026-11-03)

- verdict: fail
- notes: At 45 deg: first attempt completed all throws. Second and Third attempts ended with mid-air collisions between the balls. Final attempt ended successfully with all 20 catches At 90 deg: every ended with a ball hitting the top of the Platform and bouncing off in such a way as to be uncatchable

## Sitting result (2026-11-07 16:42)

- verdict: fail
- notes: plant_id: columns45 - first attempt ended due to a mid-air collision. Second passed with all throws caught. Third ended with a mis-caught ball. plant_id: columns90 - Forgot to change angle in GUI plant_id: columns90-2 - first attempt ended with mid-air collision. Most attempts ended with Ball2 hitting the top of the Platform. No attempt lasted more than about 5 throws.

## Sitting result (2026-11-10)

- verdict: fail
- notes: Flown from sheet step 8 on the default plant: 45 deg COMPLETED 20/20; 90 deg 8/8, 3/3, 3/3, 3/3, Ball 2's first Catch after the Feed missed outward (IDEA-22). Verdict from the logs at the debrief, confirmed by the owner 2026-11-11.

## Progress

- 2026-10-31: P2 of plan pattern-progression.md done offline: goal columns_turned, parameter columns_angle_deg, 12 pose box cells at 45 and 90 deg (45 deg admits only Apex 0.925-0.975, 90 deg 0.875-0.975). Next: the JP-10 Rung (P3) at a Sitting. Logbook 2026-10-31-pattern-progression-p2-turned-columns.md.
- 2026-11-01: Sitting: gate NOT met. 45 deg best 2/2, 90 deg best 16/16 (GUARD_LATCHED, BUG-50); hand iq 53.3 A on a 45 deg hop launch (BUG-51); PWAY-2 P2(b) NOT answered (detail below)
- 2026-11-03: Failed at the Sitting of 2026-11-03: 45 deg COMPLETED 19/19 twice of 5, 90 deg 0 of 7 (best 12/12), Ball 2 lands 37-50 mm outward at 90 deg vs 10 mm outward authority; plant_id not set per angle so PWAY-2 P2(b) unanswered; a 14th goal rejected at angle 135. 19 in 19 is a records row.
- 2026-11-10: Failed at the Sitting of 2026-11-10 (step 8, real Attempts, plant_id not set: 32 rows in the default Memory, separable by Site). 45 deg COMPLETED 20/20; 90 deg 8/8, 3/3, 3/3, 3/3, three at Ball 2's first Catch after the Feed (BUG-63). PWAY-2 P2(b) offline: the pull toward the partner rotates with the pair (BUG-79). Filed IDEA-22. Logbook 2026-11-10-sitting-debrief.md.
