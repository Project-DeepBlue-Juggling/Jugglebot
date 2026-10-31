---
id: JB-107
kind: idea
title: 'JP-9 High-low columns: Ball 1 at apex 1.05 m, Ball 2 at 0.95 m'
status: in_progress
created: '2026-10-30'
updated: '2026-10-31T00:29:27Z'
owners:
- jugglebot-f5
priority: medium
effort: m
hardware: robot
owner: hands
radius: subsystem
area: skill-stack
blocked_by: []
links: []
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
parent: pattern-progression-columns-to-centre-and-orbit
---

Owner wants to see it (2026-10-30). Two-ball columns with unequal apexes.

## Timing

- The 0.045 s flight difference is absorbed by the Dwells (the low ball holds longer) and shortens one of the two empty transits.
- The offline probe planned 1.05 over 0.95 at today's limits under pose.

## Binding constraint

Hand acceleration at the high ball (3843 of 3900 rev/s2 at 1.05).

## Software

- Pattern carries one apex_m for both Sites and the box keys on one apex band, so the compiler needs a per-Site apex and the admissible box a two-band cell.
- The learner's apex outcome is already per throw.

## Appearance

Visually subtle: the two apexes differ by 100 mm.

## Sitting

- setup: level, armed, bb-fed
- why: P5 of plan pattern-progression.md: two Apexes in one Pattern (JP-9). The only admissible pair today is 1.00 over 0.90 at separation 100; 1.05 over 0.95 waits for the Hand thread JB-114.
- do:
  - Rehearse before arming: cup empty, Ball Butler unpowered, `ros2 param set /skill_node apex_m_ball2 0.90`, send the goal below once. The accept message may name only the cup and Ball Butler refusals (no box, planner or apex2 refusal) and nothing moves.
  - No GUI entry yet: `ros2 param set /skill_node apex_m_ball2 0.90` (refused while a goal runs), then `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns_highlow, apex_m: 1.00, separation_mm: 100.0, dwell_s: 0.27, num_cycles: 20}" --feedback`.
  - Watch the hand current at every launch; cancel the goal if any hand iq sample reaches 50.0 A.
  - Afterwards `ros2 param set /skill_node apex_m_ball2 0.0`.
- pass: 20 consecutive Catches in one Attempt; Tracker mean Apex difference between the two balls within 20 mm of the commanded 100 mm; hand iq under 45 A at every launch.
- record: caught of thrown, each ball's mean Tracker Apex, peak hand iq per launch, the bag path; commit docs/records.md.
- minutes: 12

## Progress

- 2026-10-31: P4 of plan pattern-progression.md done offline: goal columns_highlow, parameter apex_m_ball2, per-ball Apex and dwell in the compile, 16 pose cells swept. Nothing fits at separation 125; at 100 the pair 1.00/0.90 fits at both Sites, 1.05/0.95 only the high ball. Next: the JP-9 Rung (P5) at 100 with 1.00/0.90. Logbook 2026-10-31-pattern-progression-p4-high-low-columns.md.
