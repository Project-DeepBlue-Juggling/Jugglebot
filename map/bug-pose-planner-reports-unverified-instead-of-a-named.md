---
id: JB-104
kind: bug
title: Pose planner reports UNVERIFIED instead of a named INFEASIBLE at the feasibility edge
status: inbox
created: '2026-10-30'
updated: '2026-10-30T08:52:13Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-10-30-two-ball-oval-p3-regate-go-and-dwell-headroom.md
filed_by: claude (P3 pose-default test triage)
---

Found 2026-10-30 while re-pinning tests after the default planner flipped to pose.

## What happens

A hop CATCH carrying its throw (hop 250 mm, apex 0.9, dwell 0.30, limits 350/5000/200k, hand 3900) with every tracker landing offset +15 mm in x sits on the window's feasibility edge for BOTH planners:

- Level solves at +15 and both refuse INFEASIBLE at +16.
- Pose ends REJECTED_CYCLE_INFEASIBLE(UNVERIFIED: QP solution failed verification (equality residual 8.0e-03, bar 1e-7) after 66 iterations, working set 14) where level just converges.

## Not a safety defect

The gate held (plan refused, not returned) and the geometry is OUTSIDE the swept box (the pose hop cell at apex 0.875-0.925 admits landing x of +-1 mm), so this is not a safety or in-box defect.

## The residue

**The refusal's NAME.** An operator reading UNVERIFIED should instead see a named INFEASIBLE with the binding row, the same plain-language class as the 2026-09-29 test_skills_segments fix.

## Repro

`tests/motion/test_skills_executor.py` `test_a_hop_catch_releases_from_the_caught_position_too` docstring and the probe recipe it records (pose at +15 mm).

## Fix shape

When the active-set solve exits unverified on a near-degenerate problem, re-classify as INFEASIBLE naming the tightest constraint, or run a tightened second pass before reporting.
