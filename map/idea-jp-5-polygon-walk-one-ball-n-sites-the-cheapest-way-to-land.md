---
id: IDEA-9
kind: idea
title: 'JP-5 Polygon walk: one ball, N Sites, the cheapest way to land per-Beat Sites'
status: complete
created: '2026-10-30'
updated: '2026-11-20T04:12:51Z'
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
- logbook/2026-11-20-sitting-debrief.md
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
parent: pattern-progression-columns-to-centre-and-orbit
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

## Sitting result (2026-11-20 15:11)

- verdict: pass
- notes (split into bullets):
  - Perfect! The pattern didn't seem super square-y though. I'd like to see some plots showing the pattern viewed from above, to see how precise and repeatable the throws were (and how square the pattern was). Second attempt ended with MAX_DEVIATION after 29 throws. (sheet 2026-11-20-robot, step 14) [derived:
  - temp/logs/launch_2026-11-20_1.log:7872 polygon_walk done 25/25 caught (start 13:38:15, plant polygon-2026-11-20 from 0 Memory rows, landing median 16 mm, worst 43 mm), and line 8201 29/29 caught in a 50-Throw Attempt ended by a Hand MAX_DEVIATION latch; no miss, so none on a first visit. The step-6 nomination margins are not in the log.]

## Progress

- 2026-11-18: Offline half built 2026-11-18: Pattern polygon_walk (label provisional) on the one-ball path. The 250 mm Hop edge is infeasible for a square; the nomination accepts 100 mm at Dwell 0.27 (+0.189) and 125 mm at 0.45 (+0.200). Not on the BeatEvent stream. GUI not wired. Next: the Rung Sitting (plan P7).
- 2026-11-18: Owner 2026-11-18: label polygon_walk stands; first flight is the square at edge 100 mm, Dwell 0.27, Apex 0.9.
- 2026-11-20: Passed at the Sitting of 2026-11-20: polygon_walk 25/25 from an empty Memory (the log's landed values: median 16 mm, worst 43 mm), then 29/29 of a 50-Throw Attempt ended by a Hand MAX_DEVIATION latch; no miss. Landed square: sides 92-108 mm for 100 commanded, per-axis scatter 12-26 mm; plots in temp/probes/polygon_2026-11-20/.
