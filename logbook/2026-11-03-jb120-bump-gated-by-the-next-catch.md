---
title: "A Bump is gated by the next Catch: shrunk in logged halvings or declined BUMP-NEXT-CATCH when the seed it leaves cannot be planned (I-CATCH-7, BUG-42)"
type: investigation
date: 2026-11-03
status: in-progress
phase: "pattern progression"
related_plan: pattern-progression.md
related_issues:
  - BUG-42
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/motion/test_skills_bump.py
  - tests/motion/test_skills_bump_next_catch.py
  - logbook/2026-11-03-jb120-bump-gated-by-the-next-catch.md
  - logbook/INDEX.md
subsystem: [motion]
tags: [safety, dynamics, testing]
---

# A Bump is gated by the next Catch (I-CATCH-7, BUG-42)

## Summary

The structural step BUG-42 filed: at Bump time, ask whether the next Catch can be planned from the seed the Bump leaves, and shrink or decline the Bump when it cannot. Built as `bump.bump_with_next_catch_gate` (the one enforcement point, reached through `bump_record` by both live callers) with an injected next-catch gate; the executor logs requested vs applied and commits the APPLIED landing. New invariant I-CATCH-7. **No live caller supplies a gate yet**: the only faithful gate found is the next Catch's own solve, and it does not fit the Bump's 25-50 ms dispatch-to-swap allowance. That placement is the owner's decision (Open Questions); with no gate the live path is I-CATCH-5 bit-for-bit. Related: `2026-11-02-bump-brake-seed-and-debrief-observability.md`, `2026-11-02-sitting-debrief.md`.

## Symptoms

- 2026-11-01 and 2026-11-02 Sittings: every `HAND_LIMIT_ACC` end (7, then 3) was the first install of a Catch seeded from a bumped predecessor; 0 of 329 unbumped seeds refused (debrief).
- f81984b2 fixed one field of that seed (the stale C-HAND-5 brake direction; S2 L1489 4696 -> 3407 offline). The 2026-11-03 re-fly passed with no `HAND_LIMIT_ACC`.
- But nothing certifies the seed as a whole. `apply_catch_bump`'s own `validate_cycle` measures the bumped record, whose hand arrays a Bump never touches, so it is blind to the successor's solve by construction.

## Diagnosis

Offline sweep on main (2026-11-03, `/tmp`-class probe in the scratchpad, the `tools/probes/bump_seed_replay.py` construction: Columns sep 125, Apex 0.95, dwell 0.27, planner pose, hold 0, 2026-11-01 learner rows; next Catch solved by production `plan_segment` with the Hand limit lifted to read the peak):

| Bump of Ball 2's Catch (mm) | 0 | 2.5 x | 5 x | 7.5 x | 10 x | 15 x | 20 x | 25 x | 30 x | -10 y | -30 y |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Next Catch peak Hand acc (rev/s^2) | 2319 | 2225 | 2477 | 3024 | 3463 | 3688 | 3661 | 3567 | 3436 | 2509 | 3096 |

Ball 1's Catch: -20 / -10 / +5 / +10 mm x -> 3701 / 3474 / 2836 / 3443. So after the brake-seed fix a 10 mm Bump still adds about 1100 rev/s^2 to the successor; nothing on main reaches the live 4500 (max 3701, 82 %). The rise is monotone below about 15 mm, which is the range halving works in.

Cost, measured on this Jetson (OMP/OpenBLAS 1 thread): `apply_catch_bump` with its `validate_cycle` 8 ms idle, 17-24 ms under box load; the next Catch's `plan_segment` 33-36 ms idle, 66-93 ms under load.

## Discussion

**Why gate on the next Catch, not re-solve it at the unlearned aim.** The held branch (TKT-29, dropped) retried the next Catch at the unlearned aim y_d. Offline the aim moves the peak by at most 0.7 points and the seed by about 12, so the retry was inert: it changed an input the peak does not depend on. The gate acts on the input that matters, the Bump that writes the seed, before it is committed; the y_d fallback acts after, when the seed is already flying.

**Why no cheap surrogate gate.** The brief proposed a cheap check at Bump time (the vectorised `validate_cycle` runs under 10 ms). `validate_cycle` validates a plan, and the next window has no plan until its solve. The seed-level quantities do not predict the solve's peak in the sweep: the along-axis brake is -71.56 k mm/s^2 for every Bump, and the cross-axis part of `a_R` is non-monotone against the peak (x 379 mm/s^2 -> 3688; y 464 -> 2660; x 783 -> 3436). A threshold on them would be a fitted cap, the stopgap the debrief declined at 21.5 mm, and could be inert exactly as the y_d fallback was. Hypothesis withdrawn (below).

**Why the solve is not wired live.** (1) Timing: `k_b = k_wire(dispatch) + 2` gives the node 25-50 ms from dispatch to swap; one gated solve (33-93 ms) alone turns the swap into `SPLICE_TOO_LATE`, and the shrink ladder can need four. (2) CPU: it adds a CATCH-solve burst inside the Catch window, where BUG-58 already shows such bursts starving the 40 Hz setpoint stream past the firmware's 70 ms cover. Moving the solve to the executor before dispatch avoids (1) but not (2), and needs the executor to hold the record and build the next terminal early. So the enforcement point, the shrink rule, the logging and the invariant are built and tested with the real solve injected, and the live gate is an owner trade (Open Questions).

**Why halve rather than clip to a fitted bound.** Each step is a whole Bump of a smaller Δ: `b`, `ḃ`, `b̈` scale linearly, so every step is the same C2 shape, knots `0..k_b` stay bit-identical (asserted per step) and nothing streamed mixes two shapes. Halving needs no model of the peak, and it stops at a step the real gate accepted, not at a predicted bound. I-CATCH-5's "never scaled down" rule exists because a partial Bump is a SILENT clip; here the shrink is named on the line (`BUMP-SHRUNK step k: requested ... -> applied ...`), and the executor commits the applied landing and the realised `b(t_rel)`, so the learner row stays true (I-CATCH-5 (d)). The I-CATCH-5 row now names this one exception.

**Never edit the seed.** Re-deriving the seed is `_bumped_brake_seed`'s job (I-PLAN-10 / C-HAND-5). Changing the seed to make a gate pass would make the plan disagree with the ball's actual take-off.

## Control walk-through (one 40 Hz tick at Bump time, gate supplied)

1. Executor tick t: fences pass (NO-CONVERGED-FIT, WITHIN-TOLERANCE, CAP-SPENT, BUMP-FREEZE, BUMP-WINDOW), Δ requested, bumper -> `bump_record` -> `bump_with_next_catch_gate`.
2. `apply_catch_bump(Δ)`: refused by its own gate -> the I-CATCH-5 decline, unchanged. Accepted -> gate(Δ).
3. Gate passes -> Δ applied as today (one gate call added). Refuses -> Δ/2, Δ/4, Δ/8, each built by `apply_catch_bump` (own gate first) then gated; first pass returned, else `BUMP-NEXT-CATCH`, record untouched.
4. Wire: the node swaps whatever is returned under its existing apply-time check; a step that took too long is refused `SPLICE_TOO_LATE` (non-terminal, record untouched). No feed-forward step: the returned plan's `pose_vel` is the analytic derivative of the same offset, as before.
5. Worst case: 4 x (8-24 ms + gate). No gate: 0 ms added (today's live path).

## Fix

- `bump.py`: `bump_with_next_catch_gate` (I-CATCH-7), `NextCatchGate`, `BUMP_NEXT_CATCH`, `BUMP_SHRUNK`, `BUMP_SHRINK_STEPS` = 3, `BUMP_SHRINK_FACTOR` = 0.5; `BumpResult` gains `requested_delta_xy_mm`, `shrink_steps`, `carried_release_k` (the next Catch's seed knot); `bump_record` routes through it (a `next_catch_gate` kwarg, absent on both live callers).
- `executor.py`: `catch_bump(..., next_catch_gate=)`; `BUMP-NEXT-CATCH` passes through by name and joins the shrink-retry rule; a shrunk accept commits committed landing + applied Δ. Log lines: `BUMP skill N: ... — BUMP-SHRUNK step k: requested (dx, dy) -> applied (dx, dy) mm; next catch refused ...` and `BUMP-DECLINED BUMP-NEXT-CATCH skill N: ...`.
- `INVARIANTS.md`: new I-CATCH-7 row; I-CATCH-5 (b) names the exception.

## Outcome

- New tests on the pre-change tree (2026-11-03, HEAD f3d365fa exported by `git archive` into the scratchpad with the two new test files copied in, `python -m pytest -q -p no:cacheprovider tests/motion/test_skills_bump.py tests/motion/test_skills_bump_next_catch.py`): **6 failed, 22 passed** (the six new tests: no `bump_with_next_catch_gate`, no `carried_release_k` / `shrink_steps`, no `next_catch_gate` on `catch_bump`).
- Same command on this tree (2026-11-03): **28 passed**. The real-planner half shows the gap the old path had: `apply_catch_bump` accepts the 20 mm bump and its seed is refused `HAND_LIMIT_ACC` by the next Catch at a 3600 rev/s^2 gate; the gated path applies 10 mm (one halving), and declines `BUMP-NEXT-CATCH` at a 2000 gate after trying 20 / 10 / 5 / 2.5 mm. The gate limit sits below 4500 because nothing on main reaches 4500 (see Diagnosis).
- Scoped run (2026-11-03 13:40, `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 pytest -q -p no:cacheprovider tests/motion/ tests/sim/test_skills_gate.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py`): **2863 passed, 2 skipped in 577.66 s**. Also `tests/sim/test_glossary_avoid_words.py`: 4 passed.
- Not run: `./run_tests.sh` / `--full` (the CLAUDE.md tier-1 gate for `ros_ws/` changes); owed before merge to main. No box-gated file changed (`admissible._GATED_FILES`), so `gate_hash` is unchanged.
- Hardware: none. With no live gate supplied the flown path is I-CATCH-5 unchanged, so no Sitting is owed by this commit alone.

## Withdrawn claims

- "A cheap check at Bump time (validate_cycle-class, under 10 ms) can gate the next Catch." Withdrawn on the sweep above: no existing check evaluates the next window without solving it, and the seed metrics do not track the solve's peak.

## Open Questions

1. The live gate (owner): (A) the executor solves the next Catch in the solver worker before dispatch (1-4 solves of 33-93 ms in the Catch window, the BUG-58 CPU budget); (B) an offline-certified Bump envelope per box cell, a table lookup at Bump time (the box sweep gains a Δ axis); (C) leave it unwired: on main every swept bumped seed plans at or under 82 % of 4500.
2. The shrink floor: 3 halvings stop at Δ/8 (2.5 mm for a 20 mm request, near `resend_pos_tol_mm`).
3. Should a `BUMP-NEXT-CATCH` decline also suppress the learner row for that Catch (the Catch is then flown off the tracker's landing by the full Δ)?
