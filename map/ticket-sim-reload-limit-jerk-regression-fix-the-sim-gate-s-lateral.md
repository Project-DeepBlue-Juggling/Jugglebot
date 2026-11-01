---
id: JB-128
kind: ticket
title: 'Sim `--reload` LIMIT_JERK regression: fix the sim gate''s lateral authorities first, bisect the planner, both in order, or retire the scenario?'
status: complete
created: '2026-11-01'
updated: '2026-11-01T00:18:10Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: sim
blocked_by: []
links: []
parent: bug-sim-reload-gate-limit-jerk-regression
type: question
---

JB-27: the sim `--reload` gate (`sim/skills_gate.py`) was good at 8364808c (5/5 makes, dwell 0.30 s) and after 08ec3fe6 (2026-09-30, the R5 columns feed-catch commit) gets 1 make with `LIMIT_JERK` in the continuation throws; no config or limit changed in that commit and hardware flew R5 cleanly after it. The candidate fix in `plans/archived/columns-empty-cup-two-feed.md` section 6 is to set the lateral authorities in the sim gate.

The open decision is whether the regression is in the sim gate's configuration (fix the gate) or in the planner (a real regression the hardware happened to tolerate). Night Shift 2026-11-01 drafted the options into the Item's `## Options (2026-11-01)` section.

## Options

1. Treat it as a gate-configuration gap: set the sim gate's lateral authorities to the launch defaults (20 mm learner, 35 / 10 mm catch) and re-run the scenario; if it returns to 5/5 the Item closes with the gate fixed.
2. Treat it as a planner regression: bisect inside 08ec3fe6 (the feed-catch segment shape) with the sim scenario as the oracle and fix the planner; needs a design look, since hardware did not show it.
3. Both, in order: option 1 first (one evening, offline); if the scenario stays red, option 2.
4. Retire the `--reload` scenario from the sim gate as superseded by the Columns scenarios flown since.

Recommendation: 3. Option 1 is cheap and offline; its result decides whether 2 is needed.

## What each answer unblocks

- 1 or 3: a Night Shift unit (hardware none, sim) the gate can merge.
- 2: a design unit for the owner's Session.
- 4: the Item closes with the scenario removed.

## Answer

Agreed; Option 3. I expect that treating this as a gate-configuration gap should resolve the problem, as no issues have arisen on hardware, but it'd be good to check
