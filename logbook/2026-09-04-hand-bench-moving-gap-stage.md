---
title: "Hand-stream bench gains a moving-gap stage to observe FW 17's falling-edge decay"
type: feature
date: 2026-09-04
status: resolved
phase: "unified-7dof-planner — Phase 3 (sitting three prep)"
related_plan: unified-7dof-planner.md
files_changed:
  - tests/hardware/hand_stream_bench.py
  - tests/hardware/session_unified7_hand_bringup.md
  - tests/teensy_link/test_hand_stream_bench_gap.py
subsystem:
  - can
  - tools
tags:
  - testing
  - safety
---

# Hand-stream bench gains a moving-gap stage to observe FW 17's falling-edge decay

## Summary

New `moving_gap` stage in `tests/hardware/hand_stream_bench.py`: takes the
hand-lane knot gap mid-motion (rides row 14's slow triangle, 0.5 rev/s) rather
than at rest, so FW 17's falling-edge decay (Mode 1 Hermite → Mode 2 Taylor →
Mode 3 linear decay, wind-down 0.135 s) can finally be exercised — the
existing `gap` stage rides a Hold and cannot
([`2026-09-04-fw17-hand-ladder-sitting-two.md`](2026-09-04-fw17-hand-ladder-sitting-two.md)
§ Open Questions item 6). Built and unit-verified offline only; carried into
sitting three, not yet flown.

## Motivation

Phase 3 shipped the decay as the normative behaviour for a hand-less frame
([`2026-09-02-unified-7dof-planner-phase3-fw17-hand-lane.md`](2026-09-02-unified-7dof-planner-phase3-fw17-hand-lane.md)),
but no bench stage has ever seen it happen: the hand was at rest (max |vel|
0.073 rev/s) every time a gap has been taken, so there was never a nonzero
`v1` to decay. Phase 4's unified cycle fires this falling edge once per
throw-catch, so shipping it unobserved is a real gap in the safety case.

## Design

`moving_gap` withholds N knots (`--gap-knots`, default 9) starting at
`--gap-at` (derived = 2.0 s, mid the triangle's first ascending ramp — the
driver refuses if the withheld window straddles a vertex), then resumes from
the *current* planned position, bounded by `--reentry-max-rev` (0.25). The
driver computes the re-entry step before sending and refuses over-bar: with
observe-first, `MAX_DEVIATION_HAND_REV` only counts a `dev_over` tick after
`hand7 arm`, the lead clamp silently absorbs anything under 2.0 rev, and the
pump's `_prev_hand` clears on any accepted hand-absent frame
(`setpoint_pump.py:720`) so the re-entry knot has no prior — the driver's own
bound is the only layer that says no.

Closed-form prediction (accel[6] ≡ 0, `setpoint_pump.py:710`, so the
firmware's jerk EMA is identically 0 and every cubic term vanishes):

| Quantity | Formula | @0.5 rev/s |
|---|---|---|
| Decay-model coast | v·(SEG_T+MAX_EXTRAP_DT_S+DECAY_DT_S/2) = v·0.105 | +0.0525 rev = 1.66 mm |
| Mode-1-endpoint-hold (forbidden mode) | v·SEG_T = v·0.025 | +0.0125 rev |
| Never-winds-down model | v·(gap−SEG_T) = v·0.225 | +0.1125 rev |
| Re-entry step | v·(gap−0.105) | 0.0725 rev = 2.29 mm |

Pass criteria — an un-judgeable one returns SKIP with reason, never a silent
PASS:

| # | Criterion | Bar |
|---|---|---|
| G1 | coast target nearest & within tol of DECAY model | max(0.010, 0.02v) rev; nearest of the three (¼ smallest inter-model gap v·0.08, floor 8× row 13's dev_max 0.0013 rev) |
| G2 | target frozen past 0.135 s | echo span ≤0.005 rev (4× noise floor vs 0.045 rev a non-decaying lane would sweep) |
| G3 | encoder tracks decayed target | ≤0.10 rev = 3.16 mm (1.6× sitting two's worst hand tracking 1.92/1.85 mm; 5 % MAX_LEAD_HAND_REV) — derived at 0.5 rev/s, does **not** scale with speed |
| G4 | re-entry step | ≤0.25 rev (3.4× predicted; 12.5 % MAX_LEAD_HAND_REV, 10 % MAX_DEVIATION_HAND_REV, 5 % pump's 5.0 gate) |
| G5 | lead-clamp bit 6 | never sampled set |

## Implementation

New `pred_rev` CSV column appended (old indices preserved); `cmd_rev` reads
the literal `withheld` value on hand-less ticks, scoped to this stage.
`MAX_EXTRAP_DT_S`/`EXTRAP_DECAY_DT_S` mirrored from `canbridge_config.h` with
a pinning test. Runbook gets a new stage row marked CARRIED-IN for sitting
three, § Results row NOT RUN. New `tests/teensy_link/test_hand_stream_bench_gap.py`
(379 lines, 38 tests, offline: withheld-knot indices, resume position,
pass-criteria evaluation on synthetic traces, the constant pins) sits beside
the other Setpoint-source covers — `tests/hardware/` is pytest-ignored.

**Caveats carried, not resolved here:** G6 (echo-move re-entry proof, reusing
`GAP_ECHO_MOVE_REV` 0.05) is only sufficient alongside G2; the `[hand7]`
counter deltas the runbook also wants are console-only (the driver reports
the 40 Hz `lead_clamp_mask` bit-6 *sample*, not a duty count) and the runbook
requires bracketing them with row 11c's capture; G3's bar was derived at
0.5 rev/s only; no `uptime_ms` CSV column was added (sitting-two Open
Question 8, out of scope here).

## Verification

- (2026-09-04, `python -m pytest tests/teensy_link/test_hand_stream_bench_gap.py -q`,
  **38 passed in 0.27 s**).
- (2026-09-04, `python -m pytest tests/teensy_link/ -q`, **335 passed in 7.57 s**).
- (2026-09-04, `python -m pytest tests/teensy_link/ tests/firmware/ -q`,
  **742 passed in 31.35 s**).
- No hardware invoked; argparse refusal paths exercised by hand.
- Full gate: run by the orchestrator before the commit; result recorded in
  the commit message.

## Outcome

**Flown 2026-09-05, and it did what it was built to do: G1–G5 PASS, prediction
confirmed** — coast +0.05250 rev bit-equal to the closed-form `v·0.105`, target
frozen with 0.00000 rev of span, re-entry +0.0725 rev against +0.0725 predicted
([`2026-09-05-fw17-hand-ladder-sitting-three.md`](2026-09-05-fw17-hand-ladder-sitting-three.md)).
It was carried into sitting three alongside row 18's arming half and row 19(b)
([`2026-09-04-fw17-hand-ladder-sitting-two.md`](2026-09-04-fw17-hand-ladder-sitting-two.md)
§ Outcome), and it closed Open Questions item 6 there: the falling-edge decay
behaves exactly as the normative rule requires, and the lane keeps transmitting
its decayed target across the edge rather than going quiet.

**Both Open Questions this entry carried are resolved.** (1) The sitting
confirmed G1–G5 with no divergence from the closed-form DECAY model —
`|coast − DECAY| = 0.00000`, four times the tolerance from the forbidden
hold-at-endpoint mode and six from no-wind-down. (2) The `[hand7]` counter
deltas this stage depended on now exist: sitting three produced the project's
first bracketed row-11c console capture
(`temp/logs/hand7_console_20260905_135613.log`, 313 blocks at 1 Hz), and this
stage's deltas in it are `lead 0 dev_over 0 unseen 0 stale 0` with `sent`
advancing by exactly 500 in every full second including the gap's own.

The caveats above stay carried: G3's bar is still derived at 0.5 rev/s only,
and no `uptime_ms` CSV column exists.
