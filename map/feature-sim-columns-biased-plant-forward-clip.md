---
id: JB-32
kind: feature
title: Sim columns run with a biased plant to exercise the forward-clip rule
status: in_progress
created: '2026-10-24'
updated: '2026-11-01T18:20:27Z'
owners:
- night-shift
priority: medium
effort: m
hardware: none
owner: none
radius: local
area: sim
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
- logbook/2026-11-02-jb32-sim-columns-biased-plant-forward-clip.md
filed_by: claude
---

Run the sim columns pattern against a deliberately biased plant so the forward-clip rule is exercised. Not built (wave-3 open 2). Trigger: a change to the clip rule.

## Progress

- 2026-11-02: Night Shift 2026-11-02: plant_landing_bias_mm knob, release hook and executor_lines landed on the self-toss and hop learner runs, where the forward clip is live; the columns gate builds its executor with boxes=None, so wiring the swept columns boxes into run_columns_attempt is the remaining slice (detail in the logbook entry)
