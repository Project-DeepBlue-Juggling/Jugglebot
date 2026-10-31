---
id: JB-108
kind: idea
title: 'JP-10 Turned columns: the columns pair at fixed angles about +z, under pose'
status: in_progress
created: '2026-10-30'
updated: '2026-10-31T00:29:27Z'
owners:
- jugglebot-f5
priority: medium
effort: s
hardware: robot
owner: hands
radius: subsystem
area: skill-stack
blocked_by: []
links: []
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
- why: P3 of plan pattern-progression.md: two-ball Columns off the x axis under the pose planner, one plant_id per angle, which answers JB-38 P2(b) (does the lateral bias rotate with the pair).
- do:
  - Rehearse before arming: cup empty, Ball Butler unpowered, `ros2 param set /skill_node columns_angle_deg 45`, send the goal below once. The accept message may name only the cup and Ball Butler refusals (no box, planner or angle refusal) and nothing moves. Repeat at 90.
  - Fly a straight `Columns` control first if this Sitting has not flown one (GUI Columns, Sep. 125, Apex 0.95, Dwell 0.27, Throws 20).
  - `ros2 param set /skill_node columns_angle_deg 45` (refused while a goal runs), then `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns_turned, apex_m: 0.95, separation_mm: 125.0, dwell_s: 0.27, num_cycles: 20}" --feedback`. Only the 0.925 to 0.975 Apex band is admissible at 45 deg.
  - `ros2 param set /skill_node columns_angle_deg 90`, then the same goal at Apex 0.95 (0.90 is also admissible at 90 deg).
  - Afterwards `ros2 param set /skill_node columns_angle_deg 0`.
- pass: 20 consecutive Catches in one Attempt at each angle; landing scatter within 3 mm of the straight control; no HAND_LIMIT line.
- record: caught of thrown per angle, each angle's plant_id, landing scatter against the control, the bag path; commit docs/records.md.
- minutes: 20

## Progress

- 2026-10-31: P2 of plan pattern-progression.md done offline: goal columns_turned, parameter columns_angle_deg, 12 pose box cells at 45 and 90 deg (45 deg admits only Apex 0.925-0.975, 90 deg 0.875-0.975). Next: the JP-10 Rung (P3) at a Sitting. Logbook 2026-10-31-pattern-progression-p2-turned-columns.md.
