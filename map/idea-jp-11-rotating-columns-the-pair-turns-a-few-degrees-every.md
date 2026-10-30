---
id: JB-109
kind: idea
title: 'JP-11 Rotating columns: the pair turns a few degrees every Beat'
status: inbox
created: '2026-10-30'
updated: '2026-10-30T09:07:31Z'
owners: []
priority: medium
effort: l
hardware: robot
owner: hands
radius: cross-cutting
area: skill-stack
blocked_by: []
links: []
filed_by: claude (jugglebot-f5, pattern exploration 2026-10-30)
---

Columns whose Site pair rotates about +z continuously. Pose-only.

## Numbers

- Per-Beat displacement 2r sin(d_theta): 11 / 22 / 43 mm per Beat at 5 / 10 / 20 deg with separation 125 mm, aim under 0.7 deg.
- A full turn takes 41 / 21 / 10 s.
- Offline probe: feasible under pose up to about 45 deg per Beat; ball clearance collides at 90 deg per Beat.
- Reads as rotating at 20 deg per Beat or less.

## Blocker and routes

Blocked on per-Beat Sites: the executor, admissible box and learner all assume a Pattern's two Sites are fixed for the Attempt. Two routes:

- The learner-generalisation plan P3 and P4 (online admissibility, per-Beat BeatEvent stream).
- The cheaper pattern-relative admissible-box key proved by JP-10.
