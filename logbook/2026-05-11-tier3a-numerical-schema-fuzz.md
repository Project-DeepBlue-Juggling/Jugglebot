---
title: MPC sad-path coverage — Phase 7 Tier 3a numerical + schema fuzz
type: feature
date: 2026-05-11
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 7"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-12-tier3a-fuzz-bugfix
  - 2026-05-11-tier2c-zmq-recv-resilience-bugfix
  - 2026-05-11-tier2c-zmq-corruption
  - 2026-05-11-tier1c-input-fuzz-bugfix
  - 2026-05-11-tier1c-input-fuzz
files_changed:
  - tests/sim/test_mpc_input_fuzz.py
  - tests/sim/test_diag_schema_fuzz.py
  - logbook/2026-05-11-tier3a-numerical-schema-fuzz.md
  - logbook/INDEX.md
  - plans/archived/2026-05-18 mpc-sadpath-coverage-tiers-1-3.md
commits:
  - 2105bb4
subsystem:
  - controller
  - mpc
tags:
  - testing
  - fuzz
  - hypothesis
  - schema
  - numerical
  - contract-surfaced
---

# MPC sad-path coverage — Phase 7 Tier 3a numerical + schema fuzz

## Summary

Plan 2 Phase 7 (Tier 3a) — two distinct fuzz surfaces:

* **Tier 3a-1 (numerical)**: extends
  [tests/sim/test_mpc_input_fuzz.py](../tests/sim/test_mpc_input_fuzz.py)
  with **7 new test classes** (108 parametrised cases for T-U-T3a-N1
  alone; 18 cases total across N2–N7) covering NaN/Inf inputs and
  edge cases on every public API surface the Phase 3 (Tier 1c) fuzz
  did NOT touch — `feasibility.segment_is_feasible`,
  `feasibility.quintic_peak_vel_per_axis`,
  `hermite.quintic_interp_with_accel`,
  `target.flat_target_to_events`, `target.make_feasible_events`,
  `mpc._numerical_ik`, and `runner.run_mpc_loop`.

* **Tier 3a-2 (schema)**: adds
  [tests/sim/test_diag_schema_fuzz.py](../tests/sim/test_diag_schema_fuzz.py)
  with **7 new test classes** covering the `diag` schema-completeness
  contract across every documented solve path (success, fallback
  walk-forward, hold_extrap, cold_hold, non_finite_solution,
  exception) plus a property test asserting the schema invariant
  across random sequences.  Drops the originally-planned T-U-T3a-S8
  ("extras namespace") because the surface doesn't exist as a real
  schema (see Discussion).

Coverage matrix:

| Surface | Adversarial input | Production code | New test coverage |
|---------|-------------------|-----------------|-------------------|
| `feasibility.segment_is_feasible` | NaN/Inf in any of 6 args | `feasibility.py:203-224` | T-U-T3a-N1 (108 cases) |
| `feasibility.quintic_peak_vel_per_axis` | T <= 0 | `feasibility.py:155-179` | T-U-T3a-N2 (3 cases, XFAIL pre-bugfix) |
| `hermite.quintic_interp_with_accel` | duration<=0; t-out-of-range; NaN/Inf | `hermite.py:88-158` | T-U-T3a-N3 (5 sub-cases) |
| `target.flat_target_to_events` | arrival<t_now+0.05 | `target.py:204-209` | T-U-T3a-N4 (3 sub-cases) |
| `target.make_feasible_events` | NaN in proposal pose | `target.py:317+` | T-U-T3a-N5 (2 sub-cases) |
| `mpc._numerical_ik` | angle<1e-10 | `mpc.py:1336-1338` | T-U-T3a-N6 (3 sub-cases) |
| `runner.run_mpc_loop` | duration=0 / duration<0 | `runner.py:300` | T-U-T3a-N7 (2 sub-cases) |
| `diag` schema — Solve_Succeeded | (primed MPC + feasible target) | `mpc.py:1207-1214` | T-U-T3a-S1 |
| `diag` schema — fallback walk-forward | NaN injection on platform_pos_mm | `mpc.py:1588-1594, 1663` | T-U-T3a-S2 (XFAIL pre-fix) |
| `diag` schema — hold_extrap escalation | repeated NaN → max_consecutive | `mpc.py:1683` | T-U-T3a-S3 (XFAIL pre-fix) |
| `diag` schema — cold_hold | fresh MPC + NaN first solve | `mpc.py:1694-1699` | T-U-T3a-S4 (XFAIL pre-fix) |
| `diag` schema — non_finite_solution | monkey-patch _solver → NaN sol['x'] | `mpc.py:1119-1123` | T-U-T3a-S5 |
| `diag` schema — exception path | monkey-patch _solver → raise | `mpc.py:1223-1238` | T-U-T3a-S6 |
| `diag` schema — invariant property | hypothesis stateful (rules: clean_solve, nan_solve) | union of all paths | T-U-T3a-S7 (XFAIL pre-fix) |

**Test additions only at this commit; four xfail-strict markers
gated on `_PHASE_7_BUGFIX_LANDED = False`.**  Empirical probing
on 2026-05-11 surfaced two production-code gaps that fail Phase 7's
stated criteria:

* **Bug C** — `feasibility.quintic_peak_vel_per_axis` has no
  `T <= 0` guard.  `T=0` causes division by zero
  inside `_evaluate_poly(...)/T` returning a NaN array; `T<0` is
  even more pathological (returns finite-but-nonsensical peak
  velocities because the per-axis math has no sign-handling).
  Surfaces as T-U-T3a-N2's `xfail(strict=True)` here; fixed in the
  companion bugfix commit.

* **Bug D** — `MPCController._handle_failure` does not populate
  `iter_count` on any failure-path diag (`mpc.py:1588-1594`).
  Schema is `{solve_time_ms, status, cost, constraint_violation,
  cmd_next_mm, cmd_next2_mm}` — six keys.  Success path has the
  same six plus `iter_count` (`mpc.py:1207-1214`).  Walk-forward
  fallback additionally adds `fallback_step` (`mpc.py:1663`).
  Schema is asymmetric, not unified.  Consumers (e.g.
  `runner.log_mpc_step` at `:207`) defend with `diag.get(...,
  default)` so the gap is silent-truncation, not crash — but real
  diagnostic data (per-failure iter count from the partial IPOPT
  inner loop) is lost on every failure tick.  Surfaces as T-U-T3a-S2,
  S3, S4, S7's `xfail(strict=True)` here; fixed in the companion
  bugfix commit alongside the new
  [`controller/DIAG_SCHEMA_CONTRACT.md`](../controller/DIAG_SCHEMA_CONTRACT.md)
  normative document.

Pattern matches Phase 3 → Phase 3 bugfix and Phase 6 → Phase 6
bugfix.  Both fixes (Bug C + Bug D contract + unify) land as a
**single follow-up commit** (user-confirmed combined-commit choice
matching Phase 6).

Pre-Phase-7: **1258 passed + 1 xfailed** (`pytest tests/ -q`, run
2026-05-11 against SHA `148dee1`).
Post-Phase-7 (this test commit, before the bugfix): **1384
passed + 8 xfailed in 348.95 s**
(`pytest tests/ -q`, run 2026-05-11; +126 new passing test
functions; +7 xfailed = T-U-T3a-N2's 3 parametrised cases plus
T-U-T3a-S2 + S3 + S4 + S7; T-U-T1a-4 permanent xfail remains as
the 8th).  Full triple repeated in Verification below.

## Motivation

Phases 1–6 covered the IPC, hardware-plant, and solver-input
sad-paths.  Phase 7 (Tier 3a) closes two remaining surfaces:

1. **Numerical edge cases on every non-`solve()` public API** —
   `feasibility`, `hermite`, `target`, `mpc._numerical_ik`,
   `runner.run_mpc_loop`.  Phase 3 (Tier 1c) covered the `solve()`
   entry-point input surface; Phase 7 extends that discipline to
   the helper / math / runner layer.  Adversarial input fuzz
   reveals where the math primitives silently propagate NaN /
   produce nonsense for invalid arguments, and where the runner /
   target / IK layers correctly guard at the boundary.

2. **`diag` schema completeness across every solve path** —
   `MPCController.solve()` returns a `diag` dict whose keys vary
   silently by execution path.  Success populates 7 keys
   (including `iter_count`); failure paths populate 6 (missing
   `iter_count`); walk-forward fallback adds `fallback_step`.
   Consumers defend against missing keys via `.get(default)`, but
   the data the consumer sees on a failure tick is partly
   sentinel.  Phase 7 makes the schema **explicit**: a normative
   document (`DIAG_SCHEMA_CONTRACT.md`), a canonical enforcement
   point in `_handle_failure`, and an invariant test
   (T-U-T3a-S7).  This is the Plan-1 K1–K6 pattern applied to a
   second invariant family — the schema contract Working Note #6
   anticipated.

Both surfaces add NO hardware exposure and no real-time risk —
Phase 7 is pure unit-test work + a contract document.

## Design

### Per-test empirical-probe table (run 2026-05-11)

Pre-implementation probes (`/tmp/probe_phase7.py` and
`/tmp/probe_phase7_schema.py`, neither committed) confirmed
each driver produces the expected behaviour against SHA `148dee1`:

| Test ID    | Driver                                                                 | Pre-fix outcome                                                       | Post-fix outcome (bugfix commit)                          |
|------------|------------------------------------------------------------------------|-----------------------------------------------------------------------|-----------------------------------------------------------|
| T-U-T3a-N1 | NaN/Inf in any of (p0, v0, a0, p1, v1, a1)[axis]                       | `LinAlgError` from `np.roots` (acceptable per plan)                   | unchanged                                                  |
| T-U-T3a-N2 | T=0 / T=-0.5 / T=-1e-12                                                | T=0 → NaN array; T<0 → finite-nonsense; no guard                      | `ValueError` from added guard                              |
| T-U-T3a-N3 | duration=0; duration<0; t>duration; NaN/Inf in inputs                  | Per-clause as documented                                              | unchanged                                                  |
| T-U-T3a-N4 | arrival=t_now+0.01 (<0.05) / +0.1 / =t_now                              | 1 hold event / 2 events / dist-based                                  | unchanged                                                  |
| T-U-T3a-N5 | NaN in proposal[0].pose / proposal[1].pose                              | `LinAlgError` downstream                                              | unchanged                                                  |
| T-U-T3a-N6 | rv=[0,0,0]; rv with norm 1e-11; rv with norm 0.1                         | identity / identity / Rodrigues — identical exts at norm < 1e-10      | unchanged                                                  |
| T-U-T3a-N7 | duration=0 / duration=-0.5                                              | `estop_exit=False`, 0 records                                         | unchanged                                                  |
| T-U-T3a-S1 | primed MPC + feasible target                                            | 7 keys: solve_time_ms, status, iter_count, cost, cv, cmd_next_mm, cmd_next2_mm | unchanged                                                  |
| T-U-T3a-S2 | primed MPC + NaN in platform_pos_mm[0]                                  | 7 keys: replaces iter_count with fallback_step                        | 8 keys (canonical 7 + fallback_step)                       |
| T-U-T3a-S3 | primed MPC + repeated NaN → hold_extrap                                | 6 keys: no iter_count, no fallback_step                                | 7 keys (canonical, fallback_step=-1 sentinel)              |
| T-U-T3a-S4 | unprimed MPC + NaN first solve → cold_hold                              | 6 keys                                                                | 7 keys (canonical)                                         |
| T-U-T3a-S5 | monkey-patch `_solver` to NaN sol['x']                                   | status = 'fallback(non_finite_solution)'                              | unchanged                                                  |
| T-U-T3a-S6 | monkey-patch `_solver` to raise `RuntimeError`                          | status contains 'exception:'; _prev_w cleared                          | unchanged                                                  |
| T-U-T3a-S7 | hypothesis stateful — clean_solve / nan_solve rules                     | Fails: at least one rule drives a failure path missing iter_count     | Holds: every solve leaves canonical schema intact          |

### `_PHASE_7_BUGFIX_LANDED` flag — single source of truth

`tests/sim/test_mpc_input_fuzz.py` defines a module-level
`_PHASE_7_BUGFIX_LANDED = False`.  `test_diag_schema_fuzz.py`
imports the flag for its own xfail gates.  This is the same pattern
Phase 6 used (`_BUGFIX_LANDED`) — one symbol, one flip in the
bugfix commit, lifts all four xfail-strict markers atomically.

The flag's xfail conditions:

* T-U-T3a-N2 (parametrised over T ∈ {0, -0.5, -1e-12}) — gated on
  the T<=0 guard fix.
* T-U-T3a-S2 — gated on the schema-unification fix.
* T-U-T3a-S3 — same.
* T-U-T3a-S4 — same.
* T-U-T3a-S7 — same.

= **5 xfail markers** total, accounting for **7 expected
failures** at this commit (3 parametrised cases × 1 marker for N2
plus 4 markers for S2/S3/S4/S7).

### Why drop T-U-T3a-S8

The plan's T-U-T3a-S8 reads: *"Property: `extras` namespace
populated identically across solve paths | hypothesis stateful |
`set(vars(extras))` is invariant across success/fallback/hold."*

There is no `extras` namespace returned by `mpc.solve()` or
populated on the `diag` dict.  The repo's `extras` references are
on a **different surface**: the `**extras` kwargs collected by
`log_mpc_step` from the optional `on_log_extras` hook (a runner
construct, not an mpc.solve() return).  The `vars(extras)`
construct in the plan suggests the author imagined an
`extras: SimpleNamespace` field on diag — which doesn't exist
in code.

Two paths from here:

(α) **Reframe S8 as "diag schema invariant" property** —
subsume into S7 (which already does exactly this).  Then S8 is
redundant.

(β) **Test the `on_log_extras` hook surface** — a *different*
contract: the hook's return shape (dict vs SimpleNamespace) and the
runner's handling at `runner.py:486-496`.  This is structurally
distinct from the solve()'s diag and belongs in a runner-hooks
test surface (Phase 8 territory — see T-U-T3b-H1..H4).

(γ) **Drop S8 explicitly** — acknowledge the plan-text framing
was incorrect; ship Phase 7 with 14 tests instead of 15.

The user (asked during Phase 7 design discussion) chose (γ) —
drop with explicit acknowledgement of the plan-text ambiguity.
S7 already covers the schema invariant; adding a synthetic "extras"
test on a non-existent surface would be test theatre, not coverage.

### Why monkey-patch `_solver` for S5 / S6 over a real driver

Phase 7's S5 (`non_finite_solution`) and S6 (`exception:`) both
require driving rare in-solve failure paths:

* **S5** — the `if not np.all(np.isfinite(w_opt))` guard at
  `mpc.py:1119-1123` only fires when IPOPT returns a solution
  containing NaN/Inf.  IPOPT's internal NaN detector almost always
  catches this earlier and reports `Invalid_Number_Detected` before
  the post-solve guard sees the bad solution.  Driving the
  post-solve-NaN path via parameters alone is not reliable.

* **S6** — the `except Exception` at `mpc.py:1223-1238` catches any
  exception raised inside `self._solver(...)` or the immediate
  post-call block.  Triggering this via parameters requires
  hitting an internal CasADi assert — not reproducible across
  versions.

The clean drive: monkey-patch `mpc._solver` at the **boundary**
(production code's solver-call interface).  Replace with a
callable object that:

* For S5 — calls the real solver, then NaN-pokes `sol['x']` before
  returning; reports `stats()` as `Solve_Succeeded` so the
  production code routes through the success path and hits the
  post-solve `isfinite()` guard.
* For S6 — raises `RuntimeError('synthetic solver failure')`
  unconditionally.

This is exactly the pattern Phase 1's T-U-T1a-3 used to drive
`Restoration_Failed` (a documented exit code not reachable via
plain `MPCParams`).  Per Plan 2 Working Note #1, the patch is at
the **production-code boundary** (the `_solver` attribute, not a
private internal), so the downstream handler under test is the
real production code, not a mocked surrogate.

Critical subtlety: `mpc._solver` must be replaced with a class
instance (not a plain `def`), because the production code at
`mpc.py` calls both `self._solver(...)` AND `self._solver.stats()`.
Plain functions can't carry arbitrary attributes; a class with
`__call__` + `stats` methods does.

### Schema contract — preview of the bugfix commit

The companion bugfix commit lands:

1. **[controller/DIAG_SCHEMA_CONTRACT.md](../controller/DIAG_SCHEMA_CONTRACT.md)** —
   normative document modelled on Plan 1's `REFERENCE_LAYER_CONTRACT.md`
   and `PLANT_INTERFACE_CONTRACT.md`.  Declares the canonical 7
   keys (`solve_time_ms, status, iter_count, cost,
   constraint_violation, cmd_next_mm, cmd_next2_mm`) plus one
   conditional key (`fallback_step` — populated on every path with
   sentinel `-1` when no fallback walk-forward is active).
   Documents the consumer contract (everything in `runner.py:203-207`
   that reads from `diag`).

2. **Canonical enforcement** — `_handle_failure` (mpc.py:1588-1594)
   gains `iter_count = 0` and `fallback_step = -1` (sentinel) in
   the base failure-diag dict.  The walk-forward branch at
   `mpc.py:1663` already overwrites `fallback_step` with the
   per-step counter; no change there.  The success path at
   `mpc.py:1207-1214` adds `fallback_step = -1` to the success
   diag (mirrors the failure-path sentinel for symmetry).

3. **Invariant enforcement** — T-U-T3a-S7's hypothesis property
   asserts the canonical 7 keys are present on every solve return.
   Same pattern as Plan 1's K1–K6 invariant test.

The contract is a 3-part landing (doc + enforcement + test) per the
Plan 1 K1–K6 + S1–S6 + P1–P4 template.

### T-U-T3a-N2 — T<=0 guard preview

Phase 7's other bug surface: `quintic_peak_vel_per_axis` has no
`T <= 0` guard.  Empirically:

* `T=0`: `_evaluate_poly(...) / T` produces `inf/nan`; max-abs of
  `[0, inf, ...]` is NaN; returns a NaN array.
* `T<0`: the math runs (no division-by-zero); peak velocities are
  divided by a negative T → wrong sign, then `np.abs` strips the
  sign → finite but **nonsensical**.

Production callers (e.g., `flat_target_to_events` at
`target.py:196-200`) already guard against `arrival_time <= t_now`
via the dist-based duration fallback.  So T<=0 doesn't reach
`quintic_peak_vel_per_axis` through the normal MPC pipeline.  But
the function is a public API; any future caller that doesn't
pre-validate gets nonsense.  Defense-in-depth: raise `ValueError`
explicitly when `T <= 0`.  ~5 LoC fix; lifts T-U-T3a-N2's xfail.

### Citation refresh against current SHA

* `controller/feasibility.py:155-179` (quintic_peak_vel_per_axis) —
  verified at current SHA.
* `controller/feasibility.py:203-224` (segment_is_feasible) — verified.
* `controller/hermite.py:88-158` (quintic_interp_with_accel) — verified.
* `controller/target.py:204-209` (degenerate arrival single-hold) —
  verified.
* `controller/mpc.py:1336-1338` (numerical_ik zero-rotation) — verified.
* `controller/mpc.py:1119-1123` (non_finite_solution guard) — verified.
* `controller/mpc.py:1207-1214` (success-path diag) — verified.
* `controller/mpc.py:1223-1238` (exception path) — verified.
* `controller/mpc.py:1588-1594` (failure-diag base dict) — verified.
* `controller/mpc.py:1663` (walk-forward fallback_step) — verified.
* `controller/mpc.py:1683, 1687, 1694` (hold_extrap / hold / cold_hold) —
  verified.
* `controller/runner.py:207` (`diag.get('iter_count', 0)` consumer) — verified.
* `controller/runner.py:300` (`n_steps = int(duration / control_dt)`) — verified.

All citations stable at SHA `148dee1`.

## Implementation

### tests/sim/test_mpc_input_fuzz.py — Phase 7 extension

| Class                                              | ID         | Tests (functions) | Strategy                                         |
|----------------------------------------------------|------------|---------------------|--------------------------------------------------|
| `TestT3aN1SegmentIsFeasibleNanInf`                  | T-U-T3a-N1 | 1 × 108 cases (parametrised)| (NaN/Inf) × (6 args) × (6 axes)            |
| `TestT3aN2PeakVelZeroDuration`                      | T-U-T3a-N2 | 3 (xfail pre-fix)   | parametrised T ∈ {0, -0.5, -1e-12}              |
| `TestT3aN3HermiteInterpBoundary`                    | T-U-T3a-N3 | 5                   | scenario tests for duration<=0, t out, NaN/Inf  |
| `TestT3aN4FlatTargetDegenerate`                     | T-U-T3a-N4 | 3                   | parametrised arrival_time relative to t_now      |
| `TestT3aN5MakeFeasibleEventsNan`                    | T-U-T3a-N5 | 2                   | NaN in start / target proposal pose             |
| `TestT3aN6NumericalIkZeroRotation`                  | T-U-T3a-N6 | 3                   | rv at zero, just-below-threshold, above         |
| `TestT3aN7RunMpcLoopZeroDuration`                   | T-U-T3a-N7 | 2                   | duration=0, duration<0                          |

The `_PHASE_7_BUGFIX_LANDED = False` flag is defined at the bottom
of the Phase 7 block.  Module-level docstring updated with the
empirical findings table.

### tests/sim/test_diag_schema_fuzz.py — new file

| Class                                              | ID         | Tests | Strategy                                                          |
|----------------------------------------------------|------------|-------|-------------------------------------------------------------------|
| `TestT3aS1SolveSucceededSchema`                     | T-U-T3a-S1 | 1     | primed MPC + 3 settle solves; assert all canonical keys present  |
| `TestT3aS2FallbackWalkForwardSchema`                | T-U-T3a-S2 | 1     | NaN injection on primed MPC; xfail-strict pre-fix                |
| `TestT3aS3HoldExtrapSchema`                         | T-U-T3a-S3 | 1     | 12× NaN spam; xfail-strict pre-fix                                |
| `TestT3aS4ColdHoldSchema`                           | T-U-T3a-S4 | 1     | unprimed MPC + NaN first solve; xfail-strict pre-fix              |
| `TestT3aS5NonFiniteSolutionSchema`                  | T-U-T3a-S5 | 1     | monkey-patch _solver to NaN-poke sol['x']                        |
| `TestT3aS6ExceptionSchema`                          | T-U-T3a-S6 | 1     | monkey-patch _solver to raise RuntimeError                       |
| `TestT3aS7DiagSchemaInvariantProperty`              | T-U-T3a-S7 | 1     | RuleBasedStateMachine over (clean_solve, nan_solve); xfail-strict pre-fix |

Module-level singletons (`_T3aS7_SINGLETONS`) follow Phase 3's
T1cWarmStartIntegrityMachine pattern — one IK precompute + one
solver prime amortised across all hypothesis examples.

### Xfail accounting — Phase 7 (this commit; bugfix commit below)

At this commit (test additions; bugfix follow-up pending in same
session), the suite carries **8 xfails total** (1 inherited
T-U-T1a-4 + 7 Phase-7-specific):

| Test ID                          | Reason                                                                       | Tracking                                                                                                                       | Target close                          |
|----------------------------------|------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|---------------------------------------|
| T-U-T1a-4                        | `Restoration_Failed` not drivable via `MPCParams` in CasADi 3.7.2             | [logbook 2026-05-11-tier1a-real-solver-failures.md](2026-05-11-tier1a-real-solver-failures.md) (Discussion → Xfail)             | Permanent (CasADi 3.7.2 limitation)   |
| T-U-T3a-N2[0.0]                  | `quintic_peak_vel_per_axis` has no T<=0 guard; bug C                          | [logbook 2026-05-12-tier3a-fuzz-bugfix.md](2026-05-12-tier3a-fuzz-bugfix.md) (pending)                                          | Same session — bugfix follow-up        |
| T-U-T3a-N2[-0.5]                 | same                                                                          | same                                                                                                                            | same                                  |
| T-U-T3a-N2[-1e-12]               | same                                                                          | same                                                                                                                            | same                                  |
| T-U-T3a-S2 (walk-forward)        | Schema gap: iter_count missing on failure paths; bug D                        | same                                                                                                                            | same                                  |
| T-U-T3a-S3 (hold_extrap)         | same                                                                          | same                                                                                                                            | same                                  |
| T-U-T3a-S4 (cold_hold)           | same                                                                          | same                                                                                                                            | same                                  |
| T-U-T3a-S7 (property)            | same                                                                          | same                                                                                                                            | same                                  |

T-U-T1a-4 has a permanent justification; all seven Phase 7 xfails
will be removed in the bugfix commit landing today.

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple per the workflow rule on test-count claims.

### Baseline (post Phase 6 bugfix backfill, SHA `148dee1`)

* `pytest tests/ -q`, run 2026-05-11 against SHA `148dee1`:
  **1258 passed + 1 xfailed.**

### Module-isolated runs

* `pytest tests/sim/test_mpc_input_fuzz.py -k "T3a" -v`, run
  2026-05-11: **123 passed + 3 xfailed in 2.56 s** (T3a numerical
  block; T-U-T3a-N2's 3 parametrised cases xfailed pre-bugfix).
* `pytest tests/sim/test_diag_schema_fuzz.py -v`, run 2026-05-11:
  **3 passed + 4 xfailed in 3.69 s** (T-U-T3a-S2/S3/S4/S7 xfailed
  pre-bugfix; S1/S5/S6 pass; T-U-T3a-S7 hypothesis stateful
  invariant fires the missing-iter_count assertion on every
  failure-path rule).

### Full-suite gate (post Phase 7, this commit)

* `pytest tests/ -q`, run 2026-05-11 with all Phase 7 test
  additions applied (no production-code change in this commit):
  **1384 passed + 8 xfailed in
  348.95 s.**  Net delta from baseline: +126
  passing test functions; +7 xfailed (T-U-T3a-N2 ×3 + S2 + S3 +
  S4 + S7); the inherited T-U-T1a-4 permanent xfail remains.

### Hot-loop allocation contract — post-additions regression

* The bugfix touches no hot-loop production code; the test
  additions exercise non-hot-loop surfaces (feasibility, hermite,
  target, runner) via direct calls (not via the 40 Hz loop).  No
  allocation regression expected.  Will be re-validated in the
  bugfix commit's verification.

## Discussion

### What Phase 7 reveals about the math + schema surfaces

Three structural observations:

1. **The math-primitive layer has very few input guards.**
   `quintic_peak_vel_per_axis`, `hermite.quintic_interp_with_accel`,
   `_numerical_ik` all silently propagate NaN/Inf or produce
   nonsense for invalid arguments.  This is **by design** at the
   math layer (input validation belongs to callers per a defensible
   layering convention), but the absence is invisible until tested
   adversarially.  Phase 7's coverage doesn't *change* the layering;
   it pins it as the documented behaviour.  Future contributors
   touching these helpers know the layering and can either
   preserve it or argue for a deliberate change.

2. **The diag schema asymmetry is the canonical example of an
   implicit contract.**  Six keys always; +1 on success; +1 on
   walk-forward.  No single source of truth, no consumer-side
   enforcement (consumers `.get()` with defaults).  Until Phase 7,
   nobody had written down what `diag` *must* contain.  This is
   the structural-problem class Plan 1's K1–K6 work addressed for
   reference-feasibility; Phase 7 applies the same template to
   the diagnostics layer.

3. **`extras` was a phantom surface.**  The plan author imagined a
   namespace that doesn't exist — likely conflating the `**extras`
   kwargs in `log_mpc_step` (a runner kwargs collector for the
   `on_log_extras` hook) with a diag-side schema.  Phase 7 drops
   S8 with that acknowledgement.  The `on_log_extras` hook is a
   separate contract surface (Phase 8 territory — runner hooks);
   if it warrants pinning, it lives in `test_mpc_runner.py` or
   the future Phase-8 hooks coverage.

### Why monkey-patch over plain-input drivers for S5 / S6

The plan suggests Phase 1's mechanism (parameter-tuning) for S6
("Force CasADi exception (Phase 1 mechanism)").  Empirical reality
on CasADi 3.7.2:

* IPOPT's internal NaN detection is aggressive — catches NaN
  inputs at problem setup time and reports `Invalid_Number_Detected`
  rather than failing through to the post-solve guard.  The
  `non_finite_solution` path (mpc.py:1119) is therefore
  structurally hard to reach from outside.
* CasADi internal exceptions (the `except Exception` at :1223) are
  similarly rare — most internal errors are caught and reported as
  IPOPT exit codes, not raised through.  The "exception:" status
  prefix at :1237 is reachable in theory but not via routine
  parameter tuning.

For both paths, the cleanest test driver is **boundary-level
monkey-patch on `mpc._solver`**: replace with a class instance that
behaves as the real solver would behave UNDER the failure mode.
The downstream handler under test is the real production code;
only the solver itself is replaced.  This is Phase 1's
T-U-T1a-3 precedent applied to two different exit conditions.

The key subtlety (caught during initial implementation): plain
`def` callables can't carry the `.stats` attribute the production
code queries.  S5's solver replacement must be a class with both
`__call__` and `stats` methods.

### The schema gap is silent-truncation, not a crash — and that's its danger

Consumers (`runner.log_mpc_step` at `:203-207`) defend with
`diag.get('iter_count', 0)`.  So a missing key produces a logged
`ipopt_iter=0` value rather than a `KeyError`.  Telemetry CSVs
under `temp/logs/` contain `ipopt_iter=0` on every fallback /
hold tick.  Operators reading the CSV would conclude IPOPT
converged in 0 iterations on those ticks — which is wrong.  IPOPT
ran some non-zero number of iterations before timing out / hitting
NaN; that data is just discarded.

The same silent-truncation applies to `fallback_step` on non-walk-
forward paths — `log_mpc_step` doesn't currently log it (the
field isn't in the canonical record schema), but any future
consumer reading `diag['fallback_step']` (e.g., a debugging plot)
would get `None` (pre-fix) or `-1` (post-fix sentinel).

Schema contracts protect against this class of silent data loss.
The bugfix landing the contract document makes the producer-side
obligations explicit; future refactors that add a new fallback
path (Phase 8?) inherit the obligation by reading the document.

### Test-additions only at this commit; bugfix follow-up

Per the plan's *"Production-code changes triggered by tests"*
subsection: bugs surfaced by Phase 7's tests land in their own
commit with their own logbook entry.  Phase 7's two bugs (C: T<=0
guard; D: schema unify + contract) ship together as a single
combined bugfix commit — matches Phase 6's user-confirmed
combined-commit choice (two related bugs, one commit) and
preserves rollback granularity (revert the bugfix → revert both
production changes; the test commit alone remains).

Rollback discipline:

* Reverting this test commit alone removes Phase 7's coverage but
  keeps any pre-existing schema asymmetry visible (the production
  code is unchanged at this commit).
* Reverting the bugfix commit alone restores both bugs (T-U-T3a-N2
  re-xfails; S2/S3/S4/S7 re-xfail); the test commit still asserts
  what the canonical schema *should* be.
* Reverting both removes the entire Phase 7 surface.

## Open Questions

* **Should the contract extend to `cmd_next_mm` / `cmd_next2_mm`
  shape semantics?**  Currently the canonical keys are pinned by
  *presence*, not by *value shape*.  A consumer that does
  `diag['cmd_next_mm'][0]` on a None value (walk-forward fallback,
  pre-W4 versions) would crash.  The contract document acknowledges
  this — `cmd_next_mm` is `None | (6,) np.ndarray`.  Whether to add
  a shape invariant to T-U-T3a-S7 is filed as a Phase 8 follow-up.

* **Should `quintic_peak_acc_per_axis` also gain a T<=0 guard?**
  The companion math primitive at `feasibility.py:182-200` has
  the same structural gap as `quintic_peak_vel_per_axis`.  The
  bugfix commit fixes both for symmetry; if scope concerns force
  a narrower fix, this Open Question becomes a Phase 8 follow-up.

* **Should the hypothesis stateful machine fuzz the target_pose
  axis too?**  Currently T-U-T3a-S7's `nan_solve` rule injects
  NaN into platform_pos_mm[0] only.  A richer machine would
  randomise WHICH adversarial input fires (NaN axis, Inf axis,
  extreme magnitude).  Filed as a future expansion if the
  pre-bugfix invariant fires on inputs the current rules don't
  reach.

## Related

* [plans/archived/2026-05-18 mpc-sadpath-coverage-tiers-1-3.md](../plans/archived/2026-05-18%20mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 7 specification.
* [logbook/2026-05-12-tier3a-fuzz-bugfix.md](2026-05-12-tier3a-fuzz-bugfix.md)
  — the follow-up bugfix commit covering Bug C (T<=0 guard) + Bug D
  (DIAG_SCHEMA_CONTRACT.md + diag unify).
* [logbook/2026-05-11-tier2c-zmq-corruption.md](2026-05-11-tier2c-zmq-corruption.md)
  & [logbook/2026-05-11-tier2c-zmq-recv-resilience-bugfix.md](2026-05-11-tier2c-zmq-recv-resilience-bugfix.md)
  — Phase 6 test + bugfix arc; same xfail-strict + combined-bugfix
  pattern Phase 7 mirrors.
* [logbook/2026-05-11-tier1c-input-fuzz.md](2026-05-11-tier1c-input-fuzz.md)
  & [logbook/2026-05-11-tier1c-input-fuzz-bugfix.md](2026-05-11-tier1c-input-fuzz-bugfix.md)
  — Phase 3 fuzz-then-fix arc; canonical precedent for the
  test-then-bugfix two-commit pattern.
* [controller/feasibility.py](../controller/feasibility.py) —
  N1, N2, N5 surface; T<=0 guard goes in
  `quintic_peak_vel_per_axis` at `:155-179`.
* [controller/hermite.py](../controller/hermite.py) — N3 surface.
* [controller/target.py](../controller/target.py) — N4
  (`:204-209`); N5 (`:317+`).
* [controller/mpc.py](../controller/mpc.py) — N6 (`:1336-1338`);
  S1 (`:1207-1214`); S2 (`:1663`); S3-S4 (`:1683, 1687, 1694`);
  S5 (`:1119-1123`); S6 (`:1223-1238`).
* [controller/runner.py](../controller/runner.py) — N7
  (`:300`); diag consumer at `:203-207`.
* [tests/sim/test_mpc_input_fuzz.py](../tests/sim/test_mpc_input_fuzz.py)
  — Phase 3 + Phase 7 numerical fuzz.
* [tests/sim/test_diag_schema_fuzz.py](../tests/sim/test_diag_schema_fuzz.py)
  — Phase 7 schema fuzz (new file).
