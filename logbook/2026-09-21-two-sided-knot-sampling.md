---
title: "Two-sided knot sampling in validate_cycle — the gate read only one side of a knot, under-measuring peak leg acc/jerk 10.8–24.3%"
type: investigation
date: 2026-09-21
status: resolved
phase: "two-ball-skill-stack — R3"
related_plan: cup-contact-contract.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - tests/motion/test_validate_cycle.py
  - tests/motion/test_validate_cycle_vectorised.py
  - tests/motion/test_unified_cycle.py
  - config/generated/admissible_box.yaml
  - plans/active/cup-contact-contract.md
  - logbook/2026-09-20-cup-contact-contract-implemented.md
  - logbook/INDEX.md
subsystem:
  - motion
tags:
  - safety
  - performance
  - testing
---

## Summary

`validate_cycle`'s peak-leg-acceleration/jerk gate sampled each knot from one side only,
which side decided by a floating-point ULP — under-measuring the plan's true supremum by
10.8–24.3% on measured plans (`logbook/2026-09-20-cup-contact-contract-implemented.md`
Diagnosis #5 Red 1, carried here). Pass 1 of `validate_cycle` now reads both one-sided
closed-form limits at every knot and the FD jerk takes both differences that straddle a
knot; the fix is exact (no new tolerance), costs +0.19 ms on a 40-knot gate, and changes
only what the gate *refuses* — no emit-path consumer reads these fields.

## Symptoms

`CyclePlan`'s pose channel is a cubic Hermite: C1 at a knot, not C2, so leg acceleration is
discontinuous there. `validate_cycle`'s grid samples each knot once; `CyclePlan._locate_batch`
decides which side by `k = (t/dt).astype(np.intp)` — a floor whose result flips on the last
bit of `t/dt`. Concretely: `4.05/0.025` falls one ULP short of `162` and reads the LEFT
limit (span 161's `s=1`); the identical physical instant read from a range view starting at
knot 135 is `27*0.025`, divides exactly, and reads the RIGHT limit (span 162's `s=0`) —
so the *same knot* reads differently depending on whether the plan is gated whole or as a
splice-seam range view.

## Diagnosis

### Magnitude — before → after, independent two-sided sup

| plan | before (mm/s²) | after (mm/s²) | Δ | % of limit |
|---|---:|---:|---:|---:|
| long_ring whole (361 knots) | 417.4568 | 468.2153 | +12.2% (was 10.84% under) | 9.4% of 5000 |
| long_ring merged `meta.report` | 404.4773 | 468.2153 | (was 13.6% under) | — |
| long_ring tail view (n−57) | 404.4773 | 468.2153 | — | — |
| THROW 0.9 m, +40 mm lateral | 563.04 | 744.015 | (was 24.32% under) | 14.9% of 5000 |
| CATCH-and-throw 0.9 m, +40 mm | 1619.87 | 1619.87 | 0% (sup already on the read side) | 32.4% of 5000 |
| on-axis THROW/CATCH-and-throw/REST, 0.6 & 0.9 m | 0 | 0 | pose identically constant — see below | — |

On-axis R3 skills (0.6/0.9 m) move **zero legs at all** — the whole throw/catch motion is
carried by the hand channel, so `pose`/`pose_vel` are bit-identical at every knot and the
defect is invisible at the canonical operating point, only appearing once an aim moves off
dead-centre (which `admissible_sweep.py`'s box sweep does routinely, ±0.5…40 mm).
(CATCH-and-throw 0.6 m on-axis is separately refused `HAND_LIMIT_ACC` 4243.3 > 3500 —
unrelated.)

Jerk (mm/s³, 1.05 margin): long_ring 24352.85 → 24352.85 (already two-sided-equivalent);
THROW+40 72705.5 → 72705.5; CATCH+40 72523.1 → 73743.2 (+1.65%, probe value, production
matches to 1e-9 via the twin parity test); synthetic test plan 3427657.7 → 6536370.0
(47.6% under-read). **The under-read is not bounded by ~13%** — 24.3% measured on a plan
this stack flies — because the knot's jump (528.63 on THROW+40, 212.95 on long_ring) is a
property of the trajectory, unrelated to the size of the peak.

### Determinacy of the fix

| check | result |
|---|---|
| long_ring merged vs whole, acc | `\|Δ\|/whole = 0.0` (bit-identical) |
| long_ring merged vs whole, jerk | `4.86e-11` |
| closed form vs picosecond-offset FD reference | `4.2e-11` rel |
| `_VALIDATE_STENCIL_KNOTS = 1` | still suffices — argued structurally in `probe_results.md`: a seam knot's missing side and its jerk straddle terms are always fully contained in one span on one side of the seam, which is exactly what reach-1 buys |

### Runtime

Min of 30 after 2 warm-ups, 40-knot segment, two interleaved rounds, BEFORE from a detached
worktree of HEAD `ce3fba5` (2026-09-21): **4.819/4.814 ms → 5.000/5.017 ms (+0.19 ms,
+3.9%)** — the plan's per-segment bar is < 10 ms. No extra Jacobians, no per-knot Python
loop, three `einsum`s over `n_knots` rows. Caveat: a Sonnet test agent was running during the
capture, box not strictly idle.

### Class enumeration (`enum_results.md`)

| site | two-sided? |
|---|---|
| leg jerk (`validate_cycle` FD pass) | **exposed** — fixed here |
| leg velocity | C1 at knots by construction — clean |
| `_hand_span_extrema` (hand acc) | closed-form both span ends — clean |
| hand C2 (pass 5) | two-sided by construction |
| `_cup_contact_floor_check` | min of both sides inside a range, inside-only at range endpoints — the conservative read for the contract as written |
| `validate()` / `validate_follow()` / `_validate_shaped_batched` | gate quintic plans, C2 at joins by construction (`quintic.py:14-16`) — not in this class, out of scope on the merits |
| `REFERENCE_LAYER_CONTRACT` K3 | dormant MPC-era bound, silent on sampling methodology — not applicable |

### Control-cycle walk

This changes what the gate **refuses**, never what is **emitted** — no emit-path consumer
reads a `CyclePlan`'s `peak_leg_acc`/`jerk` (`planner.py`'s `_stretch_factor` consumes only
quintic-plan reports; `unified_cycle.py:2091-2094` merges head/tail by `max` for reporting
only). A live re-aim inside the old under-read band is now refused `LIMIT_ACC` instead of
silently admitted, but not reachable at the launch limits (300/5000/150000): worst measured
true sup is 1620 of 5000 (32%), and `admissible.py`'s `MARGIN_FRAC = 0.9` already keeps
swept box plans ≤ 4500. Jerk is the limit that actually binds (48–50% of 150k) and moved
≤ 1.65% on the plan kinds this stack flies. Leg-path safety authority remains the firmware
`MAX_DEVIATION` guard, unaffected.

## Discussion

**1 — two-sided closed-form read vs an acceleration margin (like `_VALIDATE_JERK_MARGIN`).**
A margin would need ≥ 1.32× for today's THROW+40 alone, over-refuses every plan whose true
peak is not exactly on a knot (by up to 32%), and is still not a bound — it leaves the ULP
lottery (merged ≠ whole) in place, since it scales whatever the ULP happened to pick. The
two-sided read is exact: a plain componentwise cubic on all six pose columns, no `t`
anywhere so no ULP, and `J`/`J̇` at a knot are side-independent because pose and twist *are*
continuous there — so the fix costs zero extra Jacobians, only the pose-acceleration term.

**2 — jerk fixed in the same unit, same FD semantic.** The knot jump over `sub_dt` is still
counted exactly once, as before. Deferring jerk to a second unit would mean a second
`feasibility.py` edit, a second 29-minute admissible re-sweep, and a second `gate_hash()`
change for one root cause, while the same ULP would still decide `LIMIT_JERK`, the limit
that actually binds most often. Tradeoff accepted and stated plainly: the FD jerk's
jump/`sub_dt` term is a mesh-dependent quantity (it scales with `m`, samples-per-knot); that
semantic is pre-existing and deliberately **not** changed here — carried below, not fixed.

**3 — the scalar twin extended in lockstep, not kept verbatim with parity scoped away from
acc/jerk.** The new vectorised block is the newest code in the gate and exactly what
`test_validate_cycle_vectorised.py` exists to cross-examine; the twin's new block is an
independent spelling (scalar `_hermite` at explicit `(k, s)` on the plan's knot arrays +
`accel_to_leg_accels`, vs production's closed-form end formulas on grid rows). Cost accepted:
the twin is no longer purely a frozen historical reference (`two_sided=False` reproduces the
pre-fix `67445f3` body verbatim, so the old reading is still recoverable for comparison).

The main session made the ~40-line gate edit itself rather than spawning the Opus unit the
phase prompt allowed, since the function was already in context and an `Edit` costs less
than an agent (CLAUDE.md token-budget rule). Four Sonnet units ran this phase — probe
(44 calls), class enumeration (51), failing tests (35), scalar twin (23) — plus this one.

## Fix

- **`feasibility.py`** — new block under `peak_vel` in pass 1: `a_above`/`a_below` (closed-form
  Hermite acceleration limits at `s=0`/`s=1` on either side of every knot — algebraically
  identical to the formulas Pass 5's hand-channel C2FF check and `_cup_contact_floor_check`
  already use), `acc_above`/`acc_below` (leg-space via `J`, `J̇`), `peak_acc = max` of the grid
  reading and both closed-form sides. `span_acc` (shape `n_spans, m+1, 6`) holds intra-span
  differences plus the `into_knot`/`out_of_knot` straddle differences for the jerk FD;
  docstring and `_CYCLE_SAMPLES_PER_KNOT` comment updated with the mechanism and magnitudes.
- **`tests/motion/test_validate_cycle.py`** — three new tests (T-A/T-B/T-C, `t1_handoff.md`);
  red on HEAD `ce3fba5`: **3 failed, 50 passed**; **53 passed** after the gate edit.
- **`tests/motion/test_unified_cycle.py`** — the pre-existing acceptance test brought back to
  < 1e-9 on all nine peaks, plus a new assertion that the gate's acceleration reading equals
  the independent two-sided sup to 1e-6.
- **`tests/motion/test_validate_cycle_vectorised.py`** — the scalar twin's new two-sided
  block, `two_sided` kwarg, and a never-lower test: strictly greater on `steady-1.4`/
  `extend-chain` acc (ratio 1.1576) and `landing-1.0` jerk (ratio 1.0502); **9 passed,
  18.59 s**.

Golden numbers: the enumeration found ~91 test lines mentioning the two peaks and
essentially none pin a literal float against gate output — no re-pins needed, no number
moved down anywhere.

## Verification

All runs 2026-09-21, venv, `PYTHONPATH=ros_ws/src/jugglebot`.

| what | command | result |
|---|---|---|
| `test_validate_cycle.py`, HEAD `ce3fba5` + new tests | `pytest tests/motion/test_validate_cycle.py -q -p no:cacheprovider` | 3 failed, 50 passed |
| `test_validate_cycle.py`, after the gate edit | same command | **53 passed in 0.74 s** |
| motion + sim skill-stack scoped sweep | `pytest tests/motion/test_validate_cycle.py tests/motion/test_validate_cycle_budget.py tests/motion/test_unified_cycle.py tests/motion/test_cup_contact_contract.py tests/motion/test_skills_segments.py tests/ros/test_unified_cycle_integration.py tests/ros/test_skills_plan_bench.py -q -p no:cacheprovider -p no:randomly` | **335 passed in 13.23 s** (log `temp/logs/two_sided_scoped_20260921.log`) |
| scalar twin | `pytest tests/motion/test_validate_cycle_vectorised.py -q -p no:cacheprovider` | **9 passed in 18.59 s** |
| phase-end audit (one read-only pass, before the sweep so every `feasibility.py` byte was final) | 2026-09-21, `/audit --unstaged` over the 7-file diff + this entry | **CLEAN — 0 behaviour-affecting, 2 narrative, both applied**: the pass-1 comment said "four einsums" (it is three — `knot_bias` is reused), and the new test block still said the fix was "landing separately". The auditor re-derived the closed form against `_hermite`, and ran 3000 randomised old-vs-new trials (n_knots 3–164, m ∈ {1,2,3,4,5,7,11}): the two-sided read was never lower, 0 violations |
| admissible box re-sweep (launch limits 300/5000/150000), ONCE, after the audit's `feasibility.py` fix | 2026-09-21, `python tools/admissible_sweep.py --site-pairs both --single-apex 0.5 0.6 0.7 0.8 0.9`, log `temp/logs/admissible_sweep_20260921_twosided.log` | **1780.4 s; box content IDENTICAL to HEAD's** (`gate_hash`/`swept_at` lines excluded) and **all 14 731 row verdicts identical** to the 2026-09-21 run-3 log row for row (`wall_s` excluded) — 0 leg `LIMIT_ACC` refusals (the 3 `LIMIT_ACC` hits are the same three `THROW:HAND_LIMIT_ACC` rows as before). `gate_hash` `baecbc55a839` → `2fcdec288853`. `feasibility.py` sha1 `3f2aa14b…` at sweep start and at commit. Box table below |
| full-tier gate on the final tree (after the sweep; plan-phase closure, precedes the contract's first sitting) | 2026-09-21, `./run_tests.sh --full`, log `temp/logs/two_sided_full_20260921.log` | **PASS — parallel 6379 passed, 9 skipped, 1 xfailed in 287.57 s; serial 6 passed in 19.61 s** (baseline 2026-09-21 `cup_contact_full2`: 6373 + 6 — this unit's five new `test_validate_cycle.py` cases and the twin's never-lower test) |

**Box re-sweep, old (2026-09-21 run 3, `gate_hash baecbc55a839`) → new (`2fcdec288853`):** no box
moved — the stricter acc read refuses nothing the sweep visits, because the worst true leg-acc
sup on a box plan is ~32 % of the 5000 limit and `MARGIN_FRAC = 0.9` was never near.

| box | old xy mm | new xy mm | apex m |
|---|---|---|---|
| single 0.9 m | ±40 × ±40 | ±40 × ±40 (unchanged — still covers the 40 mm launch authority) | 0.576–0.900 |
| single 0.8 m | ±40 × ±40 | unchanged | 0.512–0.882 |
| single 0.7 m | ±40 × ±40 | unchanged | 0.448–0.847 |
| single 0.6 m | ±40 × ±30 | unchanged | 0.384–0.726 |
| single 0.5 m | ±20 × ±10 | unchanged | 0.320–0.605 |
| columns P1→P2 | ±20 × ±20 | unchanged | 0.900 only |
| columns P2→P1 | ±20 × ±20 | unchanged | 0.900 only |

## Outcome

The gate now reads a plan's true two-sided supremum exactly, with no new tolerance and a
measured +0.19 ms per 40-knot segment. The fix is scoped, unmasked no emit-path change, and
every one of the ~91 test lines that mention the two peaks needed no re-pin. No number moved
down. The phase-end audit came back CLEAN (two narrative fixes, applied before the sweep), the
one re-sweep left every box and all 14 731 row verdicts unchanged (only `gate_hash` moved), and
the full tier is green on the final tree — the three rows above. `colcon build` is owed before
any sitting (`feasibility.py` is in the ROS package).

## Carried

- **The FD jerk's jump/`sub_dt` term is mesh-dependent** (Discussion #2) — is a bounded
  leg-acceleration jump (a leg "C2 tolerance", as the hand channel already has) the better
  contract than a mesh-dependent FD read? Owner call, not built here.
- **Everything still open on the 2026-09-20 entry's Carried list stays there** (the seam
  tilt-rate pin, the QP/gate wall-clock ratio flake, the columns apex-band collapse, τ
  re-validation after a sitting) — pointer only, not copied.
- **`colcon build`** owed before any sitting (`feasibility.py` is in the ROS package).
