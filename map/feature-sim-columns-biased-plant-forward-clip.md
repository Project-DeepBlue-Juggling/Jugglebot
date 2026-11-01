---
id: JB-32
kind: feature
title: Sim columns run with a biased plant to exercise the forward-clip rule
status: in_progress
created: '2026-10-24'
updated: '2026-11-01T18:47:30Z'
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
filed_by: claude
---

Run the sim columns pattern against a deliberately biased plant so the forward-clip rule is exercised. Not built (wave-3 open 2). Trigger: a change to the clip rule.

## Progress

- 2026-11-02: Night Shift 2026-11-02: held on night-shift/2026-11-02-JB-32: (b) three audit findings open on the unit's own tests and CLI (the pre-clip regex ignores UNMET lines, the biased-versus-unbiased comparison is not tied to a skill index, --plant-bias-xy is accepted silently for columns runs); gate green (7928 passed); unblocked by your call on those fixes and a re-gate; the knob is built on the self_toss and hop learner runs, the columns slice is a second unit
