---
id: IDEA-9
kind: idea
title: 'JP-5 Polygon walk: one ball, N Sites, the cheapest way to land per-Beat Sites'
status: in_progress
created: '2026-10-30'
updated: '2026-11-18T05:13:48Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: hands
radius: subsystem
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-18-jp5-polygon-walk-compile.md
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
parent: pattern-progression-columns-to-centre-and-orbit
night_skip: low-priority
night_skip_on: '2026-11-15'
night_skip_note: Feasibility is in the body; robot, owner hands.
---

One ball hopping around N Sites (triangle, square, hexagon) instead of two. Physics trivial at any step the hop already flies (250 mm, apex 0.9).

## Why file it

It is the smallest change that makes a Skill's Site differ Beat by Beat, which every moving two-ball Pattern (JP-10 to JP-14) needs; the admissible box keys on exact Site xy stamps and the learner memory on release xy (30 mm support), so this walk exercises both with no two-ball risk.

## Software

A compile_one_ball over a Site list plus an admissible-box cell per edge, or the pattern-relative key proposed under JP-10.

## Sitting

- setup: level, armed, bb-fed
- why: the first Pattern whose Site changes every Throw, and the Learner's first visits to new Sites, have only run in sim.
- do:
  - Before arming: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'polygon_walk,apex_m=0.9,separation_mm=100,dwell_s=0.27,polygon_n=4'}"` and read the margins (offline: worst +0.189, Leg jerk).
  - `ros2 param set /skill_node plant_id polygon-<date>` so the walk does not write into the Columns Memory; `ros2 param set /skill_node polygon_n 4`.
  - Fly `polygon_walk` from the terminal (the GUI panel is not wired), Apex 0.9, Sep. 100, Dwell 0.27, 25 Throws; up to five Attempts.
  - Set `plant_id` back to `jugglebot`.
- pass: 24 consecutive Catches in one Attempt (six laps of the square), with no miss on a first visit to a Site.
- record: Throws caught per Attempt and each end reason, the Site of every miss and whether it was a first visit, the nomination margins, the bag path.
- minutes: 20

## Progress

- 2026-11-18: Offline half built 2026-11-18: Pattern polygon_walk (label provisional) on the one-ball path. The 250 mm Hop edge is infeasible for a square; the nomination accepts 100 mm at Dwell 0.27 (+0.189) and 125 mm at 0.45 (+0.200). Not on the BeatEvent stream. GUI not wired. Next: the Rung Sitting (plan P7).
