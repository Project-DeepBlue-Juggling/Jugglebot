---
id: JB-24
kind: bug
title: 10 % jitter-margin class of LIMIT_* refusals
status: inbox
created: '2026-10-24'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: decision
radius: subsystem
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
parent: two-ball-oval
filed_by: claude
---

Landing-time jitter of +-20-30 ms against the 10 % margin, with session acc/jerk sitting on the YAML ceilings, makes `LIMIT_*` refusals the likely dominant miss class. Source: the sitting-4 analysis in `plans/archived/two-ball-skill-stack.md` and the wave-3 tally. Trigger: `LIMIT_*` returns as the dominant miss; the owner's options (a)-(d) from sitting 4 are still undecided. More Dwell costs transit.
