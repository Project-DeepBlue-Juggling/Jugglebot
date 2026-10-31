---
id: JB-126
kind: ticket
title: 'UNVERIFIED at the feasibility edge: keep the reason and name the worst row, add a tightened second pass, reclassify as INFEASIBLE, or leave it?'
status: open
created: '2026-11-01'
updated: '2026-10-31T18:57:46Z'
owners: []
priority: low
blocked_by: []
links: []
parent: bug-pose-planner-reports-unverified-instead-of-a-named
type: question
---

JB-104 asks that the pose planner report a named INFEASIBLE with the binding row instead of `REJECTED_CYCLE_INFEASIBLE(UNVERIFIED: QP solution failed verification ...)` at the feasibility edge (hop CATCH carrying its throw at +15 mm x, outside the swept box; the gate held).

Night Shift 2026-11-01 read the refusal path: `cup_cycle._verify` refuses on an equality residual above the bar and its docstring says a genuinely infeasible request and a numerically degenerate one are DELIBERATELY not distinguished there (2026-08-30, Phase 0: both mean no trustworthy plan; which refusal path fires is BLAS-dependent and no caller may branch on it). Renaming the verdict reverses that decision, so it is the owner's call, not a unit's.

Options drafted into the Item's `## Options (2026-11-01)` section.

## Options

1. Keep `UNVERIFIED` as the reason but append the worst-violated row's name and residual to the message (one line in `_verify`; no caller branches on the reason, tests asserting the reason unchanged). Operator readability without touching the classification.
2. Tightened second pass: on an unverified exit re-solve with a stricter tolerance / more iterations and report `INFEASIBLE` naming the tightest constraint only if the second pass also fails; `UNVERIFIED` otherwise. Costs one more solve on a refusal (25-200 ms), on the refusal path only.
3. Reclassify outright: `UNVERIFIED` becomes `INFEASIBLE` with the binding row (the Item's wording). Reverses the 2026-08-30 non-distinction; every test pinning `reason == 'UNVERIFIED'` moves.
4. Leave it: the gate held, the geometry is outside the box, and the name is a cosmetic residue.

Recommendation: 1 (the operator gets the row name; the contract and its tests stand), 2 only if the owner wants the degenerate class measured.

## What each answer unblocks

- 1: a Night Shift unit (hardware none, radius local) that the gate can merge.
- 2 or 3: a design unit for the owner's Session (planner math).
- 4: the Item is closed as won't-fix.
