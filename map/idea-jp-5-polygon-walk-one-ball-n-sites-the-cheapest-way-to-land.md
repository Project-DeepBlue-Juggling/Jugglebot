---
id: JB-106
kind: idea
title: 'JP-5 Polygon walk: one ball, N Sites, the cheapest way to land per-Beat Sites'
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

One ball hopping around N Sites (triangle, square, hexagon) instead of two. Physics trivial at any step the hop already flies (250 mm, apex 0.9).

## Why file it

It is the smallest change that makes a Skill's Site differ Beat by Beat, which every moving two-ball Pattern (JP-10 to JP-14) needs; the admissible box keys on exact Site xy stamps and the learner memory on release xy (30 mm support), so this walk exercises both with no two-ball risk.

## Software

A compile_one_ball over a Site list plus an admissible-box cell per edge, or the pattern-relative key proposed under JP-10.
