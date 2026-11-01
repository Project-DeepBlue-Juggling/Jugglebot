---
id: JB-136
kind: bug
title: 'HAND_STROKE refuses the hop-start first CATCH at -0.000..-0.004 rev: the cup QP bounds height only at knots, the gate samples 4 per knot with a strict floor'
status: inbox
created: '2026-11-01'
updated: '2026-11-01T12:19:17Z'
owners: []
priority: high
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- map/bug-two-ball-columns-hop-start-throw-lands-25-56-mm-inboard-past.md
- map/bug-admissible-box-gate-passes-a-cell-whose-solve-exceeds-hand.md
filed_by: claude
---

Found at the 2026-11-01 Sitting: 9 refusals and 7 goal ends on straight Columns hop starts.

## Findings

- The hand sits at -0.000 to -0.004 rev against a 0.000 floor at the first CATCH of a hop start (hand -0.004, -0.002, -0.001 rev).
- The cup QP bounds height only at knots; the gate samples 4 points per knot with a strict floor, so a dip between knots is refused.
- This, not the landing bias, is the hop-start failure mode at hold 0 (JB-121). Same gate-versus-solver mismatch class as JB-120.

## Wanted

- Mid-span QP rows so the continuum respects the floor.
- A 0.01 rev gate tolerance.
- A fix is in flight this Sitting.
