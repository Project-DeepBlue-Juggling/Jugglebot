---
id: JB-127
kind: ticket
title: 'LIMIT_* jitter margin: widen the time rows'' margin, absorb jitter in the re-send, raise a Sitting limit, or tally the class at hold 0 first?'
status: open
created: '2026-11-01'
updated: '2026-10-31T18:57:50Z'
owners: []
priority: medium
blocked_by: []
links: []
parent: bug-limit-refusal-jitter-margin
type: question
---

JB-24: landing-time jitter of plus or minus 20-30 ms against the 10 percent planning margin, with the Sitting limits for acceleration and jerk on their YAML ceilings, makes `LIMIT_*` refusals the likely dominant miss class (sitting-4 analysis in `plans/archived/two-ball-skill-stack.md`, the wave-3 tally); the owner's options (a)-(d) from sitting 4 are still undecided and a longer Dwell costs transit. Night Shift 2026-11-01 cannot choose a margin or a limit for the robot; the options below are drafted into the Item's `## Options (2026-11-01)` section.

## Options

1. Widen the planning margin from 10 to about 15 percent on time-dependent rows only (the catch-time rows), leaving the Sitting limits where they are; measured cost: a few percent of the admissible box per cell, re-swept offline before any Sitting.
2. Absorb the jitter in the re-aim instead of the margin: let the re-send after a tracker fit move the catch time within the splice budget without a fresh LIMIT check against the full margin (the refusal class moves from install to re-send, where a refusal is harmless).
3. Raise the Sitting limits a notch (acceleration or jerk) with the box re-swept: buys margin everywhere at the price of the hardware headroom the 2026-10-31 Hand findings just spent.
4. Measure first: tally `LIMIT_*` refusals per goal from the last three Sittings' launch logs (hold 0 changed the plan shape) and decide on that number; no code until the class is shown dominant at hold 0.

Recommendation: 4 now (one log read, the Item's own trigger), then 1 if the tally confirms the class.

## What each answer unblocks

- 1 or 3: an offline unit (margin or limit change plus the box re-sweep) and a robot Sitting section on this Item.
- 2: a design unit on the executor's re-send path for the owner's Session.
- 4: a Night Shift unit that reads the launch logs and writes `## Findings`.
