---
id: JB-111
kind: idea
title: 'JP-14 Centre and orbit: Ball 1 in a column, Ball 2 on a circle of radius 100 mm'
status: inbox
created: '2026-10-30'
updated: '2026-10-30T11:31:18Z'
owners: []
priority: high
effort: l
hardware: robot
owner: hands
radius: cross-cutting
area: skill-stack
blocked_by: []
links: []
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
parent: pattern-progression-columns-to-centre-and-orbit
---

Ball 1 goes straight up and down at the centre; Ball 2 is thrown from one point of a circle to the next (N steps per revolution) while the Cup alternates centre, ring, centre, ring. Every transit is the radius, so the pattern is columns with a moving second Site.

## Feasibility

- Offline probe 2026-10-30: feasible today under pose at radius 95-105 mm with 6-12 steps per revolution (chord 100 / 77 mm at N 6 / 8, one lap in 7-14 s).
- The best show value reachable without P4.
- Owner's one-up-one-over (JP-15) is its two-step case.

## Software

- A two-ball compiler with a Site LIST for Ball 2 (Pattern takes exactly two Sites today).
- Aimed throws of a few degrees for Ball 2.
- Per-Beat Sites as under JP-11.
- Pose-only.
