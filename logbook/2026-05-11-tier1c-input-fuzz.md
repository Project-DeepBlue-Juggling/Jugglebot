---
title: MPC sad-path coverage — Phase 3 Tier 1c NaN/Inf input fuzz on solve()
type: feature
date: 2026-05-11
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 3"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-11-tier1b-fallback-escalation-cascade
  - 2026-05-11-tier1a-real-solver-failures
  - 2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles
  - 2026-04-23-hot-loop-zero-allocation-contract
files_changed:
  - tests/sim/test_mpc_input_fuzz.py
  - tests/sim/test_solver_failures.py
  - tests/sim/test_hot_loop_allocation_contract.py
  - logbook/2026-05-11-tier1c-input-fuzz.md
  - logbook/INDEX.md
  - plans/archived/mpc-sadpath-coverage-tiers-1-3.md
commits:
  - 7582764
subsystem:
  - controller
  - mpc
tags:
  - testing
  - safety
  - solver
  - hypothesis
  - input-validation
  - fuzz
---

# MPC sad-path coverage — Phase 3 Tier 1c NaN/Inf input fuzz on solve()

## Summary

Plan 2 Phase 3 (Tier 1c) — adds new file
[tests/sim/test_mpc_input_fuzz.py](../tests/sim/test_mpc_input_fuzz.py)
with **15 new passing tests + 1 strict xfail** that hammer the
``MPCController.solve()`` entry-point input surface with NaN, ±Inf,
extreme magnitudes, and wrong shapes:

- **T-U-T1c-1, -2, -3, -4** — Hypothesis ``@given`` tests over the
  axis space for NaN/Inf in the four live input fields
  (``state.platform_pos_mm``, ``state.platform_rot``, ``target_pose``,
  ``ref_events[i].twist``).  Every adversarial input routes to
  ``fallback(Invalid_Number_Detected)`` with warm-start integrity
  preserved.
- **T-U-T1c-D1** — regression guard for the *empirical finding* that
  ``state.platform_twist`` is a DEAD field in ``solve()`` (NaN/Inf
  in it produces identical cmd to a clean field).  The plan
  originally targeted ``platform_twist``; T-U-T1c-2 was redirected to
  the closest live twist-shaped field (``platform_rot``) and the
  dead-field finding got its own dedicated test.
- **T-U-T1c-5** — parametrised scenarios for extreme magnitudes
  (±1e10 mm in ``target_pose`` and ``state.platform_pos_mm``).
  Empirical: IPOPT handles these cleanly; ``Solve_Succeeded``.
- **T-U-T1c-6** — parametrised scenarios for wrong-shape inputs.
  Empirical: numpy broadcast raises ``ValueError`` at the parameter-
  vector pack inside ``solve()`` (P3-level boundary).
- **T-U-T1c-7** — Hypothesis ``RuleBasedStateMachine`` over
  (succeed_solve, fuzz_solve, advance_time) with the load-bearing
  invariant ``_prev_w is None or np.all(np.isfinite(_prev_w))``.
  Holds at ci-deep (1000 examples, 363.68 s) over the SAFE strategy
  space (see ``leg_extensions_mm`` exclusion below).
- **T-U-T1c-7-bug (xfail-strict)** — deterministic demonstration of
  a real production bug T-U-T1c-7's stateful property surfaced:
  NaN in ``state.leg_extensions_mm`` combined with a stale W7
  snapshot routes through ``hold_extrap`` and propagates the NaN
  from ``q_cur`` into ``self._prev_u``.  Fix lands in a separate
  commit per Plan 2's "production-code changes triggered by tests"
  rule; this xfail is the regression artefact for the fix.

Plus two pieces of cross-test plumbing that landed in this same
phase:

- **Phase 1 keyword-matrix extension** in
  [tests/sim/test_solver_failures.py](../tests/sim/test_solver_failures.py):
  added ``Invalid_Number_Detected`` to ``_FALLBACK_KEYWORD_MATRIX``.
  Phase 3's fuzz surfaced this as a real IPOPT exit code that the
  Phase 1 enumeration didn't cover.  Test-only change; the
  classifier in ``mpc.py`` already routes it to fallback (literal-
  tuple "is success" check, so any non-success string falls through).
- **Working Note #5 mitigation** in
  [tests/sim/test_hot_loop_allocation_contract.py](../tests/sim/test_hot_loop_allocation_contract.py):
  ``gc.collect()`` at the start of both
  ``test_hot_loop_allocation_contract`` and
  ``test_hot_loop_allocation_contract_hardware`` to drop the
  baseline heap before snapshotting.  Pre-emptive — Phase 3's
  three Hypothesis property tests across two files materially
  increase the heap pressure on the test process; the gc.collect()
  is a cheap, hardware-safe protection against the flake the
  Working Note flagged.

Test additions only; **zero production-code changes**.  The xfail'd
T-U-T1c-7-bug is the regression artefact for the production fix
that lands separately.

Pre-Phase-3: 1216 passed + 1 xfailed.  Post-Phase-3: **1232 passed
+ 2 xfailed** (+16 passed, +1 xfail).  Hot-loop allocation contract
remains green at ci-deep; T-U-T1c-7 holds at ci-deep.

## Motivation

Phases 1 (real IPOPT exit codes) and 2 (escalation cascade) covered
the failure paths *downstream of* the solver call.  Phase 3 covers
the failure paths *upstream of* it — what happens when the inputs to
``solve()`` themselves are adversarial?

The 2026-05-08 audit identified two specific gaps:

1. **No NaN/Inf coverage on any ``solve()`` input.**  The success-
   path NaN check at the success branch of ``solve()`` in
   ``controller/mpc.py`` enforces that the *output* is finite, but
   nothing tests what happens when the *input* is NaN.
2. **Warm-start integrity property tested only across solver
   failures** (Phase 2 T-U-T1b-6) — not across input-fuzz failures.
   A NaN at the entry boundary takes a different code path
   (``Invalid_Number_Detected`` from IPOPT vs ``_StatsInjector``-
   driven failures); the warm-start integrity invariant must hold
   on both paths.

Plan 2's framing is "drive real failures, not synthetic ones."
Phase 3's compliance: every adversarial input flows through the
real solver call.  ``_StatsInjector`` is not used in this phase —
the fuzz drivers produce real failures by injecting real values
that IPOPT then rejects via real arithmetic.

## Design

### Per-test recipe — empirical-probe table

Pre-implementation probes (``/tmp/probe_input_fuzz.py``,
``/tmp/probe_more.py``; not committed) catalogued the production
behaviour at every input field surface:

| Adversarial input                                      | IPOPT exit                            | Production status                          | Warm-start intact? |
|--------------------------------------------------------|---------------------------------------|--------------------------------------------|-------------------|
| NaN in ``state.platform_pos_mm[i]``                    | ``Invalid_Number_Detected``           | ``fallback(Invalid_Number_Detected)``      | yes               |
| Inf in ``state.platform_rot[i]``                       | ``Invalid_Number_Detected``           | ``fallback(Invalid_Number_Detected)``      | yes               |
| NaN in ``target_pose[i]``                              | ``Invalid_Number_Detected``           | ``fallback(Invalid_Number_Detected)``      | yes               |
| Inf in ``target_pose[2]``                              | ``Invalid_Number_Detected``           | ``hold_extrap(Invalid_Number_Detected)`` * | yes               |
| NaN in ``ref_events[i].twist``                         | ``Invalid_Number_Detected``           | ``fallback(Invalid_Number_Detected)``      | yes               |
| ±1e10 mm in ``target_pose`` / ``platform_pos_mm``      | ``Solve_Succeeded``                    | ``Solve_Succeeded``                         | yes               |
| Wrong-shape ``target_pose`` / state arrays             | not reached                           | ``ValueError`` from numpy at param pack    | yes               |
| NaN in ``state.leg_velocities_mmps`` (success path)    | ``Solve_Succeeded`` (q_dot is failure-only) | ``Solve_Succeeded``                  | yes               |
| NaN in ``state.platform_twist``                        | dead field                            | ``Solve_Succeeded`` (identical to clean)   | yes               |
| **NaN in ``state.leg_extensions_mm`` + stale snapshot**| ``Invalid_Number_Detected``           | ``hold_extrap(Invalid_Number_Detected)``    | **NO — `_prev_u` corrupted** |

\\* Inf in ``target_pose[2]`` produces a finite-vs-infinite mid-node
ref delta that trips the W7 walk-forward-unsafe ref-shift branch
(Phase 2 logic).  The status is ``hold_extrap(...)`` rather than
``fallback(...)`` because the input crosses two contracts at once;
the warm-start integrity invariant still holds.

The last row is the **bug surfaced by Phase 3**.  See Discussion →
"Bug surfaced — _prev_u corruption via hold_extrap" below for the
full analysis, the xfail accounting, and the planned fix.

### T-U-T1c-D1 — the dead-field finding

The plan's text for T-U-T1c-2 named ``state.platform_twist`` as the
target field.  The probe found ``platform_twist`` is NOT consumed by
``solve()`` on the current code base — two solves with adversarial
vs clean ``platform_twist`` produce *identical* cmd.

Two response options were considered:

(a) Test ``platform_twist`` anyway, assert ``Solve_Succeeded``,
document the dead-field finding.  Acts as a regression guard, but
the test name is misleading (it's "fuzz on a dead field").

(b) Replace T-U-T1c-2 with ``platform_rot`` (live field) AND add a
dedicated dead-field documentation test as T-U-T1c-D1.

Phase 3 chose (b).  Rationale: T-U-T1c-2's PURPOSE was to verify
the warm-start integrity invariant on a twist-shaped input — that
purpose is best served by a live field (``platform_rot``).  The
dead-field finding is a separate property worth its own regression
guard (so a future refactor that wires ``platform_twist`` into
``solve()`` without input-fuzz coverage will fail loudly).

### T-U-T1c-7 — strategy restriction and the xfail-strict bug test

T-U-T1c-7's first run at ci-fast surfaced the
NaN-in-``leg_extensions_mm``-corrupts-``_prev_u`` bug.  Per Plan 2's
"production-code changes triggered by tests" rule:

> 1. Don't fix the bug in this plan's commits.
> 2. Add the test with ``xfail`` and a tracking reference.
> 3. The bug fix lands in its own commit ... un-xfailed in the
>    same commit as the fix.

Two implementation patterns considered:

(a) **Xfail the WHOLE T-U-T1c-7 stateful machine.**  Loses 99% of
the strategy-space coverage; only one branch of the strategy
exposes the bug.

(b) **Restrict T-U-T1c-7's fuzz_solve strategy to skip
``leg_extensions_mm``** AND add a separate
``TestT1cLegExtNanCorruptsPrevU`` test, marked ``xfail(strict=True)``,
that demonstrates the bug deterministically.

Phase 3 chose (b).  This mirrors Phase 1's
``TestRestorationFailedNotDrivable`` pattern — a single targeted
xfail captures the gap; the rest of the suite stays green.

The xfail comment carries the three-field accounting Plan 2's
Working Note #2 requires:

| Field             | Value                                                              |
|-------------------|--------------------------------------------------------------------|
| Test ID           | T-U-T1c-7-bug                                                      |
| Tracking          | This logbook entry's Discussion → "Bug surfaced — _prev_u corruption" |
| Target close      | Plan 2 archival (the bug fix is the gate)                          |

Restriction must be reverted in lockstep with the fix: the comment on
``_T1C_FAULT_FIELD`` in
[test_mpc_input_fuzz.py](../tests/sim/test_mpc_input_fuzz.py)
explicitly lists the two follow-up actions (remove xfail; widen
strategy) so the un-restriction can't be forgotten.

### Test infrastructure — alias-break pitfall

P1 in
[PLANT_INTERFACE_CONTRACT.md](../controller/PLANT_INTERFACE_CONTRACT.md)
specifies ``MuJoCoPlant.get_state()`` returns the SAME ``self._state``
instance every call, with array fields aliased to the plant's
internal buffers and rewritten in place at each call.  This makes
test patterns that REASSIGN state attributes hazardous on a shared
plant fixture:

```python
state = plant.get_state()
state.platform_pos_mm = np.zeros(2)   # breaks plant alias
# next test's plant.get_state() now writes to platform_pos_mm[2]
# of the (2,) array → IndexError, contaminating the rest of the suite.
```

The first run of T-U-T1c-6 hit this — the
``platform_pos_mm shape (2,)`` test corrupted the shared plant; the
following ``leg_extensions_mm shape (5,)`` test then crashed inside
``plant.get_state()`` instead of the intended ``ValueError`` at the
parameter-vector pack.

Two-pronged fix:

1. **In-place mutation everywhere possible.**
   ``state.platform_pos_mm[i] = nan`` writes through the alias and
   is overwritten by the next ``plant.get_state()`` — safe.
2. **Fresh ``MuJoCoPlant`` per test for the wrong-shape and recovery
   tests** (function-scoped ``fresh_plant`` fixture).  These tests
   MUST reassign attributes; they pay ~1-2 s of plant-build cost
   per test in exchange for isolation from the shared plant.

The fix is documented in the ``plant`` and ``fresh_plant`` fixture
docstrings so the next contributor reading the file sees the
discipline before they trip the same wire.

### Working Note #5 mitigation

Plan 2 Working Note #5 flagged that the hot-loop allocation
contract may flake more under hypothesis fuzz suites earlier in the
test run.  Phase 2 deferred the mitigation to Phase 3 by design;
Phase 3 introduces three new Hypothesis property tests across two
files (``test_solver_failures.py`` already had T-U-T1b-6;
``test_mpc_input_fuzz.py`` adds T-U-T1c-1 through T-U-T1c-7), so the
heap pressure rationale tips definitively toward applying the
mitigation.

Two forms were on the table:

(a) ``gc.collect()`` at the start of
``test_hot_loop_allocation_contract``.  Hardware-safe, one-line
change, no test reorganization.

(b) Mark the test ``slow`` and run it in a separate pytest
invocation.  Bigger blast radius (touches CI workflows, CLAUDE.md);
heavier protection.

Phase 3 chose (a).  Rationale:

- The contract passed at ci-deep through Phase 2 with no flake
  observed.  The pressure increase from Phase 3 is real but not
  catastrophic.
- ``gc.collect()`` is invisible to the contract under test (it
  drops the baseline before snapshotting; doesn't change the
  per-tick allocation count being measured).
- The slow-marker option is a one-way reorganization; if the gc
  approach proves insufficient later, the slow marker is still
  available as a follow-up.

Both ``test_hot_loop_allocation_contract`` and
``test_hot_loop_allocation_contract_hardware`` now call
``gc.collect()`` at entry.  Verified green at ci-deep
post-mitigation: 3 / 3 in 15.87 s.

### ``Invalid_Number_Detected`` — Phase 1 matrix extension

Phase 3's probes immediately surfaced ``Invalid_Number_Detected`` as
the IPOPT exit code for any NaN/Inf input.  Phase 1's
``_FALLBACK_KEYWORD_MATRIX`` did not enumerate it.  The classifier
in ``mpc.py`` is a literal-tuple "is success" check, so the new
string correctly routes to fallback regardless of whether it's
enumerated — but the matrix is the documentation/regression artefact
for the IPOPT exit-code surface.  A one-line addition (with an
inline comment naming this logbook entry as the discovery context)
keeps the matrix complete.

This is a test-only change to a Phase 1 file landed in a Phase 3
commit.  The cross-phase touch is justified because:

- The discovery is a Phase 3 byproduct.
- The matrix is the right home for the new string.
- The commit message + this logbook entry document the cross-phase
  touch transparently.

## Implementation

### tests/sim/test_mpc_input_fuzz.py — new file

| Class                                           | ID(s)                  | Tests           | Strategy                                       |
|-------------------------------------------------|------------------------|-----------------|------------------------------------------------|
| ``TestNanInPlatformPos``                        | T-U-T1c-1              | 1 (hyp)         | ``@given`` over xyz axis                       |
| ``TestInfInPlatformRot``                        | T-U-T1c-2              | 1 (hyp)         | ``@given`` over rotation axis × {NaN, ±Inf}    |
| ``TestPlatformTwistIsDead``                     | T-U-T1c-D1             | 1               | comparison: clean vs adversarial cmd identity  |
| ``TestNanInTargetPose``                         | T-U-T1c-3              | 1 (hyp)         | ``@given`` over pose axis (0–5)                 |
| ``TestNanInRefEventsTwist``                     | T-U-T1c-4              | 1 (hyp)         | ``@given`` over twist axis (0–5)                |
| ``TestExtremeMagnitudes``                       | T-U-T1c-5              | 4 (param)       | ±1e10 in target_pose / platform_pos_mm         |
| ``TestWrongShapeRaises``                        | T-U-T1c-6              | 4 (param)       | wrong-shape arrays at API boundary             |
| ``TestRecoveryAfterFuzzFault``                  | (recovery)             | 1               | NaN fault → clean recovery solve               |
| ``T1cWarmStartIntegrityMachine`` (TestCase)     | T-U-T1c-7              | 1 (stateful)    | RuleBasedStateMachine over 3 fields × 3 ops    |
| ``TestT1cLegExtNanCorruptsPrevU``               | T-U-T1c-7-bug          | 1 (xfail-strict)| deterministic bug reproducer                   |

Cross-cutting helpers: ``_create_mpc`` mirrors Phase 1's signature
(``use_aot_solver=False``, ``prime_solver=False``); ``_seed_mpc``
runs one successful solve to populate warm-start state;
``_assert_warm_start_integrity`` checks the load-bearing safety
property on every fuzz path; ``_assert_safe_outcome`` checks the
status routes into one of the five documented buckets and the cmd
is finite.

### tests/sim/test_solver_failures.py — Phase 1 matrix extension

One-line addition to ``_FALLBACK_KEYWORD_MATRIX``:

```python
('Invalid_Number_Detected',          False),
```

with an inline comment naming this logbook entry.  The matrix's
classifier test (``test_keyword_classifier``) auto-parametrises over
the new entry — no other code changes needed.

### tests/sim/test_hot_loop_allocation_contract.py — WN#5 mitigation

Added ``import gc`` and a single ``gc.collect()`` line at the top of
both ``test_hot_loop_allocation_contract`` and
``test_hot_loop_allocation_contract_hardware``.  Each call site has
an inline docstring paragraph explaining the rationale + linking
this logbook entry.

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple per the workflow rule on test-count claims.

### Baseline (post Phase 2)

- ``pytest tests/ -q``, run 2026-05-11 against SHA ``029495d``:
  **1216 passed + 1 xfailed in 316.96 s.**

### Module-isolated run

- ``pytest tests/sim/test_mpc_input_fuzz.py -q``, run 2026-05-11:
  **15 passed + 1 xfailed in 30.71 s.**  All Phase 3 tests in
  isolation including the strict xfail.

### Property test depth — ci-deep validation

- ``pytest tests/sim/test_mpc_input_fuzz.py::TestT1cWarmStartIntegrity
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q``, run
  2026-05-11: **1 passed in 363.68 s** (1000 examples).  The
  warm-start integrity invariant holds across the safe strategy
  space at nightly depth with deterministic seed.

### Hot-loop allocation contract — post-mitigation regression check

- ``pytest tests/sim/test_hot_loop_allocation_contract.py
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q``, run
  2026-05-11: **3 passed in 15.87 s.**  ``gc.collect()`` mitigation
  effective; contract still measures within budget.

### Full-suite gate (post Phase 3)

- ``pytest tests/ -q``, run 2026-05-11 with all Phase 3 changes
  applied: **1232 passed + 2 xfailed in 345.85 s.**  +16 passed
  matches the new tests (15 in test_mpc_input_fuzz.py + 1 added
  matrix entry parametrised in test_solver_failures.py); +1 xfailed
  matches T-U-T1c-7-bug.  Zero regressions on existing tests.

## Discussion

### Bug surfaced — _prev_u corruption via hold_extrap when q_cur is non-finite

T-U-T1c-7's first run at ci-fast surfaced a real production bug.
Hypothesis shrunk the failing example to:

```
T1cWarmStartIntegrityMachine.advance_time(t_advance=1.0)
T1cWarmStartIntegrityMachine.fuzz_solve(
    field_spec=('leg_extensions_mm', 6),
    axis_offset=0,
    value=nan,
)
# invariant: prev_u_finite_or_none → AssertionError
# _prev_u = array([nan,  5.,  5.,  5.,  5.,  5.])
```

Trace of the corruption path:

1. The stateful machine's ``__init__`` runs a successful seed solve.
   Post-seed: ``_prev_u`` is finite, ``_t_at_last_success`` is the
   wall-clock time of the seed solve, ``_ref_at_last_success_mid``
   is the seed ref.
2. ``advance_time(1.0)`` ages ``_t_at_last_success`` by 1 second
   (≫ 500 ms staleness threshold).
3. ``fuzz_solve(field='leg_extensions_mm', axis=0, value=nan)`` sets
   ``state.leg_extensions_mm[0] = nan`` and calls ``solve()``.
4. ``solve()`` reads ``q_cur = state.leg_extensions_mm.copy()`` —
   ``q_cur[0]`` is ``nan``.  IPOPT detects the NaN at the first
   gradient eval, returns ``Invalid_Number_Detected``.
5. ``solve()`` routes to ``_handle_failure(... q_dot=...)``.  The
   W7 walk-forward arm requires
   ``not walk_forward_unsafe AND consecutive_failures <= max_consecutive_failures``;
   the staleness from step 2 makes ``walk_forward_unsafe = True``,
   so the arm is gated out.
6. ``_handle_failure`` falls through to the ``hold_extrap`` arm:
   ``cmd = np.clip(q_cur + q_dot * dt0, margin, stroke - margin)``.
   ``q_cur[0]`` is ``nan``; ``cmd[0]`` is ``nan`` (NaN propagates
   through arithmetic and through ``np.clip``).
7. ``self._prev_u = cmd`` — corruption.  The next solve will then
   warm-start from a non-finite ``_prev_u``, and the cascade is on.

**Why is this load-bearing?** Production telemetry plausibly emits
NaN in ``leg_extensions_mm`` when an encoder packet is corrupted
mid-tick (rare but documented).  The hardware plant's freshness
watchdog catches this on a slower cadence (telemetry-stale ESTOP at
20× control_dt); a single-tick NaN that arrives between watchdog
checks would currently corrupt the controller.

**Fix sketch (NOT in this commit):** the simplest fix is to
sanitize ``q_cur`` at the boundary into ``_handle_failure`` —
either reject non-finite ``q_cur`` (route to ``cold_hold`` instead)
or replace the NaN axes with the previous finite ``_prev_u`` values.
The latter is surgically smaller; the former is more conservative.
The correct choice is a control-system question and belongs in the
fix's own discussion section.

**Xfail discipline.** Per Plan 2 Working Note #2, the xfail carries
three fields (test ID, tracking, target close).  The tracking field
points at this Discussion section.  The target-close field is "Plan
2 archival" — the bug fix is a gate on Plan 2's archival, not on
Phase 3's completion.

### Why test infrastructure required a deep fix

The first run of the new file produced 3 failures, of which 2 were
test infrastructure bugs (alias-break under shared plant fixture)
and 1 was the real production bug above.  The audit lesson: when a
fuzz test's first run produces multiple failures, classify each
failure separately before assuming they're related.  In this phase:

- Failure 1 (``leg_extensions_mm shape (5,)`` IndexError) — looked
  like a wrong-shape detection bug in production code.  Root cause:
  prior parametrize iteration ``platform_pos_mm shape (2,)``
  reassigned ``state.platform_pos_mm`` on the shared plant alias,
  contaminating the plant's internal state for all subsequent
  tests.  Fix: function-scoped fresh plant for tests that reassign.
- Failure 2 (``test_recovery_after_nan_in_platform_pos``) — same
  alias-break class; same fix.
- Failure 3 (``TestT1cWarmStartIntegrity`` invariant violation) —
  real production bug.  Fix: separate commit, xfail in this commit.

If the first failure had been treated as the same bug as the third,
Phase 3 would have either over-reported the production-code bug
surface OR under-reported the test-infrastructure discipline gap.
Both would erode the audit's signal value over time.

### What Phase 3 reveals about the input-validation contract

The MPC currently has NO input validation at the ``solve()`` API
boundary.  Three observations from this work:

1. **IPOPT is the de facto NaN watchdog.**  Every NaN in any field
   that flows into the optimization gets caught by IPOPT at the
   first gradient evaluation, exits with
   ``Invalid_Number_Detected``, and routes through ``_handle_failure``.
   The classifier (literal-tuple "is success") routes everything
   non-success to fallback automatically.  This is *correct
   behaviour by construction* — but it's also *implicit* and could
   regress silently if a future refactor swallows the IPOPT
   exception or short-circuits the classifier.
2. **Wrong-shape inputs raise from numpy at the parameter-vector
   pack** (``p_param[6:12] = q_cur`` etc.).  This works today but
   the error message names the broadcast shape, not the field
   name — useful enough for debugging but not as informative as a
   dedicated shape-check would be.  Filed as a follow-up note in
   T-U-T1c-6's docstring.
3. **The walk-forward fallback assumes ``q_cur`` is finite** —
   the bug above.  This is the only input-validation gap in the
   failure-path machinery itself.  Fix lands separately.

A future Tier-3 effort might formalize an input-validation contract
(``CONTROLLER_INPUT_CONTRACT.md``) modelled on the K1–K6 / S1–S6 /
P1–P4 patterns, with normative invariants on each field.  Phase 3's
empirical-probe table is the seed material for such a contract;
filed as a Phase 7 follow-up topic.

### Working Note #3 — symbol references in this phase's tests

Phase 3's test docstrings cite symbols
(``_handle_failure``, ``_prev_w``, ``_prev_u``, ``q_cur``,
``walk_forward_unsafe``, ``hold_extrap``,
``_FALLBACK_KEYWORD_MATRIX``, ``_T1C_FAULT_FIELD``) rather than
line numbers.  The xfail-strict bug test's docstring references
the W7 staleness branch by name, not line.  The hot-loop
mitigation comment references this logbook entry by file path,
not line.  Compliance with the working note is uniform across the
phase.

### Xfail accounting — Phase 3

Phase 3 added **1 new strict-xfail** (T-U-T1c-7-bug) plus the
inherited Phase 1 xfail.  Total xfails on the suite at end of
Phase 3: **2** (T-U-T1a-4 ``Restoration_Failed``, T-U-T1c-7-bug
``_prev_u`` corruption).

Three-field accounting per Plan 2 Working Note #2:

| Test ID         | Tracking                                                        | Target close                          |
|-----------------|-----------------------------------------------------------------|---------------------------------------|
| T-U-T1a-4       | logbook 2026-05-11-tier1a-real-solver-failures.md (Discussion → Xfail accounting) | Permanent (CasADi 3.7.2 limitation; structural matrix coverage) |
| T-U-T1c-7-bug   | this entry (Discussion → "Bug surfaced — _prev_u corruption")  | Plan 2 archival (the bug fix is the gate) |

Per Plan 2's archival-gate language: "zero unfixed xfails at
archival, OR each residual xfail has a documented justification
for why it's permanently acceptable."  T-U-T1a-4 has a permanent
justification (CasADi-internal options not exposed via MPCParams).
T-U-T1c-7-bug does NOT — it MUST be fixed before Plan 2 can
archive.  This is a hard gate.

## Open Questions

- **Should the hold_extrap arm sanitize ``q_cur`` even after the
  bug is fixed?**  The narrowest fix routes non-finite ``q_cur``
  through ``cold_hold`` instead of ``hold_extrap``.  But
  ``cold_hold`` itself uses ``q_cur`` (``cmd = np.clip(q_cur.copy(),
  margin, stroke - margin)``) — same NaN propagation risk!  The
  correct fix needs to consider both arms.  Filed as a fix-spec
  question for the follow-up commit.

- **Should T-U-T1c-7 also exercise ``ref_events`` mutation as a
  rule?**  Currently the rules cover (state, target, time);
  ``ref_events`` is fixed to None across the walk.  The
  T-U-T1c-4 scenario test exercises NaN in events but only
  one-shot.  A stateful walk that builds an event stream over
  time, occasionally injecting NaN, would strengthen the
  invariant coverage.  Cost: more rule complexity for marginal
  invariant strength.  Deferred — file as a Phase 7 follow-up
  if the schema-completeness work surfaces an events-related gap.

- **Should the dead-field ``platform_twist`` be removed from
  ``PlantState`` entirely?**  T-U-T1c-D1 documents the field is
  not consumed by ``solve()``.  Removing it would simplify the
  ``PlantState`` dataclass and would force any future use to
  re-add it deliberately.  Out of Phase 3's scope; filed as a
  candidate Phase 7 / Plan 3 cleanup.

- **Should ``Invalid_Number_Detected`` fall under a more
  specific status prefix than ``fallback(...)``?**  The
  classifier wraps it as ``fallback(Invalid_Number_Detected)``
  alongside, e.g., ``fallback(Maximum_CpuTime_Exceeded)``.
  Operationally these are very different events (input
  corruption vs solver overload), but downstream observability
  treats them the same.  A follow-up could extend the
  ``_handle_failure`` status formatter to differentiate; out of
  Phase 3's scope.

## Related

- [plans/archived/mpc-sadpath-coverage-tiers-1-3.md](../plans/archived/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 3 specification.
- [logbook/2026-05-11-tier1a-real-solver-failures.md](2026-05-11-tier1a-real-solver-failures.md)
  — Phase 1 (Tier 1a); Phase 3's matrix extension applies to that
  phase's keyword matrix.
- [logbook/2026-05-11-tier1b-fallback-escalation-cascade.md](2026-05-11-tier1b-fallback-escalation-cascade.md)
  — Phase 2 (Tier 1b); the W7 staleness branch the bug
  reproduction depends on.
- [logbook/2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles.md](2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles.md)
  — Working Note #5 origin.
- [logbook/2026-04-23-hot-loop-zero-allocation-contract.md](2026-04-23-hot-loop-zero-allocation-contract.md)
  — Hot-loop contract still green after Phase 3 + WN#5 mitigation.
- [controller/PLANT_INTERFACE_CONTRACT.md](../controller/PLANT_INTERFACE_CONTRACT.md)
  — P1 alias contract that the test infrastructure pitfall hinged on.
- [controller/mpc.py](../controller/mpc.py) — ``solve()``,
  ``_handle_failure`` (the corruption path).
- [tests/sim/test_mpc_input_fuzz.py](../tests/sim/test_mpc_input_fuzz.py)
  — this phase's new test file.
- [tests/sim/test_solver_failures.py](../tests/sim/test_solver_failures.py)
  — Phase 1 file with Phase 3's matrix extension.
- [tests/sim/test_hot_loop_allocation_contract.py](../tests/sim/test_hot_loop_allocation_contract.py)
  — WN#5 mitigation site.
