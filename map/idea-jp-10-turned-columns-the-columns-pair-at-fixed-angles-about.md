---
id: JB-108
kind: idea
title: 'JP-10 Turned columns: the columns pair at fixed angles about +z, under pose'
status: inbox
created: '2026-10-30'
updated: '2026-10-30T11:31:18Z'
owners: []
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
