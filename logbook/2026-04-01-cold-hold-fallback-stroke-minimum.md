---
title: Cold-hold fallback commanded stroke minimum instead of current position
type: investigation
date: 2026-04-01
status: resolved
phase: "4.1"
related_plan: hardware-bringup.md
related_issues:
  - COLD_HOLD_STROKE_MIN
sessions:
  - mpc_20260401_143845.csv
  - mpc_20260401_152101.csv
files_changed:
  - controller/mpc.py
  - sim/analysis/diagnose.py
  - sim/analysis/known_issues.yaml
  - tests/sim/test_mpc_static.py
  - tests/motion/test_motor_guard.py
commits:
  - 3a3381f
subsystem:
  - mpc
  - controller
tags:
  - safety
  - performance
---

# Cold-hold fallback commanded stroke minimum instead of current position

## Summary

During Phase 4.1 moderate-motion testing, the MPC solver timed out on its first
solve (cold-start, no warm-start data). The cold_hold fallback returned
`cmd_ext = stroke_margin_mm (5mm)` for all legs, but the platform was at Active
position (~154mm). This produced a ~150mm command discontinuity that was caught
by the CAN step-reject safety, aborting the session.

## Symptoms

- MPC solver timeout on first solve (cold-start, 25-35ms vs 24ms budget)
- Fallback commanded all legs to 5mm extension
- Platform was at ~154mm (Active position) — a 149mm discontinuity
- CAN step-reject safety fired (>0.2 rev single-step limit)
- Session faulted and aborted; no physical harm due to safety catch

## Diagnosis

**Verdict: FAIL** (19 flags on primary session mpc_20260401_152101.csv)

Key flags:
- **[error]** Command discontinuity >100mm detected (cold_hold pattern)
- **[error]** 4 consecutive solve budget violations
- **[warning]** Solve times 25-35ms exceed 24ms budget for off-centre targets

The cold-hold fallback was unconditionally returning `stroke_margin_mm` (5mm)
without checking the current actual position, creating a catastrophic step command
whenever the solver timed out with no prior solution to fall back on.

### Flagged Issues

- **COLD_HOLD_STROKE_MIN** (critical) — cold-hold fallback to stroke minimum
- **MPC_STALENESS** (high) — consecutive solve budget violations

## Discussion

The cold_hold logic assumed that `stroke_margin_mm` was a safe fallback position.
This is only true at startup when legs are near their minimum extension. At any
other position, it creates a dangerous discontinuity.

The correct fallback is to hold at the current actual position (`q_cur`), which
is always safe — it commands "stay where you are".

A secondary issue was also identified: IPOPT partial solutions from timed-out
solves were being discarded. Even a partial solution (solver returning with
`Maximum_Iterations_Exceeded`) is often usable and better than the fallback.

**Rejected alternatives:**
- Increasing solver timeout — would delay the MPC loop and cause staleness
- Pre-warming the solver before motion — adds complexity; the fallback should
  be correct regardless

## Fix

**`controller/mpc.py`** — Two changes:
1. Cold_hold fallback now returns `q_cur` (current actual extensions) when
   available, instead of `stroke_margin_mm`
2. IPOPT partial solutions are salvaged when the solver times out with
   `Maximum_Iterations_Exceeded`, reducing the frequency of fallback activation

**Commit:** `3a3381f` `fix: cold-hold fallback holds at current position instead of stroke minimum`
**Branch:** `refactor`

## Outcome

Fix-validation sessions after the change:
- Static hold test (mpc_20260401_143817.csv): **PASS** — zero discontinuities
- Motion test (mpc_20260401_152101.csv): cold-hold fallback now produces
  <0.1mm discontinuity (holds at current position)
- Solver timeout cascade still occurs for off-centre targets (solve times
  25-35ms), but partial solution salvaging reduces fallback frequency

## Open Questions

- MPC solve times for off-centre targets (25-35ms) still exceed the 24ms budget.
  This is a performance issue, not a safety issue — the fallback is now safe.
  May need solver tuning or problem reformulation for Phase 4+ trajectories.
