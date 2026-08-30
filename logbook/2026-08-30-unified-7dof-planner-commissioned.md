---
title: "Unified whole-cycle 7-DoF planner commissioned — six owner resolutions baked into the plan, multi-catch-pose superseded"
type: plan
date: 2026-08-30
status: resolved
phase: "unified-7dof-planner — commissioning"
related_plan: unified-7dof-planner.md
files_changed:
  - plans/active/unified-7dof-planner.md
  - plans/active/toss-multi-catch-pose.md
  - plans/active/INDEX.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
tags:
  - planning
  - trajectory
  - toss
  - docs
---

# Unified whole-cycle 7-DoF planner commissioned

## Summary

The implementation plan for the Scope-B unified whole-cycle 7-DoF motion planner
(`plans/active/unified-7dof-planner.md`) was written and owner-commissioned
2026-08-29: platform + hand planned as **one cup trajectory per cycle** (throw +
carry + catch), ball-frame constraints hard, banking free during carry, a 7th
hand channel in the knot stream (Setpoint v6, PROTOCOL_VERSION 5→6), exclusive
Jetson→bridge hand mastery behind a firmware `hand_source` latch, and the z=170
centroid pin default-on. The plan file v1 rode yesterday's wave-2 commit; the
follow-up commit this entry accompanies bakes in the **six owner resolutions**
recorded 2026-08-29 via multiple-choice review: (1) catch runway as a hard QP
constraint + kinematic-capture sim authority (MuJoCo contact advisory — resolves
Rung 3's "slam is the seat's runway" by construction); (2) `toss-multi-catch-pose`
**HALTED at the pre-M2 boundary** — stop clean, no reverts, superseded outright
(archival follows in the next commit); (3) event-timeline planner API; (4) commit
+ bounded catch-side replans; (5) v6 carries exact v1 for all 7 channels behind
HAS_V1 (208 B); (6) Phase 6 stroke-engine retirement fires on UH-ladder (UH-7)
completion.

Why now: front-load the architectural decisions so a fresh implementing session
starts with zero open forks. The whole-cycle planner is the architecture the
3-ball endgame requires (the phase machine is single-ball by construction), and
it precedes the critical-point-ILC build ladder because Scope B changes the
launch mechanism ILC would otherwise learn on.

## Verification

- Mandated narrative audit `/audit --unstaged` run 2026-08-30: **1 BLOCKING +
  6 WARNING + 3 NOTE findings, all 10 applied** (stale 180 B diagram, v1 codegen
  scope, key counts, emitter HAS_V1 scoping + partial-set reject rule, M2 tense,
  T-H5 criterion re-anchored, MP board-row contradiction, six-vs-five count,
  UH rung labels).
- Plans-board consistency: (2026-08-30, `python -m pytest
  tests/sim/test_plans_index.py -q`, **74/74 pass in 0.19 s**) — the main session
  re-runs this after this entry lands and before the commit.
