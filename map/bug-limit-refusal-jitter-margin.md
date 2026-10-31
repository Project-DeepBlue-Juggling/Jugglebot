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

## Options (2026-11-01)

1. Widen the planning margin from 10 to about 15 percent on time-dependent rows only (the catch-time rows), leaving the Sitting limits where they are; measured cost: a few percent of the admissible box per cell, re-swept offline before any Sitting.
2. Absorb the jitter in the re-aim instead of the margin: let the re-send after a tracker fit move the catch time within the splice budget without a fresh LIMIT check against the full margin (the refusal class moves from install to re-send, where a refusal is harmless).
3. Raise the Sitting limits a notch (acceleration or jerk) with the box re-swept: buys margin everywhere at the price of the hardware headroom the 2026-10-31 Hand findings just spent.
4. Measure first: tally `LIMIT_*` refusals per goal from the last three Sittings' launch logs (hold 0 changed the plan shape) and decide on that number; no code until the class is shown dominant at hold 0.

Recommendation: 4 now (one log read, the Item's own trigger), then 1 if the tally confirms the class.

A question ticket carries this to the owner; the answer under its `## Answer` re-opens this Item.
