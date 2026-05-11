---
title: MPC sad-path coverage — Phase 2 Tier 1b fallback escalation cascade + cold-start IK budget
type: feature
date: 2026-05-11
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 2"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-11-tier1a-real-solver-failures
  - 2026-05-10-scheduler-cancel-next-during-transitioning
  - 2026-05-10-scheduler-begin-return-s3-overwrite
  - 2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles
  - 2026-04-23-hot-loop-zero-allocation-contract
files_changed:
  - tests/sim/test_solver_failures.py
  - logbook/2026-05-11-tier1b-fallback-escalation-cascade.md
  - logbook/INDEX.md
  - plans/active/mpc-sadpath-coverage-tiers-1-3.md
commits:
  - da57869
subsystem:
  - controller
  - mpc
tags:
  - testing
  - safety
  - solver
  - fallback
  - escalation
  - hypothesis
---

# MPC sad-path coverage — Phase 2 Tier 1b fallback escalation cascade + cold-start IK budget

## Summary

Plan 2 Phase 2 (Tier 1b) — extends
[tests/sim/test_solver_failures.py](../tests/sim/test_solver_failures.py)
with **6 new tests** covering the W7 walk-forward escalation logic
(`fallback(...)` → `hold_extrap(...)`) and the cold-start IK budget
guard:

- **T-U-T1b-1, -2** — pair test for the 20 mm ref-shift branch of
  `walk_forward_unsafe` in `_handle_failure`.  -1 confirms a 25 mm
  delta escalates to `hold_extrap`; -2 confirms a 5 mm delta stays
  in the walk-forward `fallback` arm.
- **T-U-T1b-3** — 600 ms wall-clock advance after the seed solve
  trips the staleness clause and routes the next failure to
  `hold_extrap`.
- **T-U-T1b-4** — strict-status escalation through
  `max_consecutive_failures`: with `max_consecutive_failures=2` and
  a steady reference, exactly the first 2 failures stay in
  `fallback(...)` (with monotonically incrementing `fallback_step`),
  failures 3, 4 escalate to `hold_extrap(...)`.  This test exists
  because the pre-existing
  [test_mpc_static.py::test_escalation_fallback_to_hold](../tests/sim/test_mpc_static.py)
  was discovered to drive the *ref-shift* branch instead of the
  *counter* branch — see Discussion → "Audit of the existing test".
- **T-U-T1b-5** — cold-start per-node IK budget exhaustion via
  `_numerical_ik` monkey-patch sleeping 0.42 s per call.  Asserts
  the `logger.info` budget-exceeded message is emitted, the
  cold-start completes, and the downstream solve returns a finite
  command.
- **T-U-T1b-6** — Hypothesis `RuleBasedStateMachine` over
  (succeed_solve, fail_solve, shift_snapshot_z, advance_time)
  rules with the load-bearing invariant
  `_prev_w is None or np.all(np.isfinite(_prev_w))`.  Across-call
  warm-start integrity holds at ci-deep (1000 examples, 441 s).

Test additions only; **zero production-code changes**.

Pre-Phase-2: 1210 passing + 1 xfailed.  Post-Phase-2: **1216
passing + 1 xfailed** (+6 net).  Hot-loop allocation contract
remains green at ci-deep (3 / 3 in 15.49 s) — the deferred Working
Note #5 mitigation is justified in Discussion below.

## Motivation

Phase 1 closed the entry-point classifier (real exit codes + literal-
tuple matrix).  The next layer down is the *escalation cascade* in
`_handle_failure` — the W7 walk-forward heuristic and its three
escalation triggers:

1. **20 mm xyz ref shift** at the mid-horizon node (`min(N, 5)`)
   between the snapshot taken at last success
   (`_ref_at_last_success_mid`) and the current
   `_last_ref_traj[mid_k]`.
2. **Twist direction flip** — any linear axis where the snapshot
   twist (`_twist_at_last_success`) and current
   (`_last_twist_traj[0]`) disagree on sign with snapshot magnitude
   > 10 mm/s.  (Not directly tested in Phase 2 — quiescent twist in
   the seeded scenarios; surfaces orthogonally in T-U-T1b-6.)
3. **>500 ms staleness** from `_t_at_last_success` by
   `_time.perf_counter()` reckoning.

Pre-Phase-2 coverage of these branches:

- **Ref-shift threshold** — never tested.
- **Staleness** — never tested.
- **`max_consecutive_failures`** — purportedly tested by
  `test_mpc_static.py::test_escalation_fallback_to_hold`; audit
  during the Phase 2 implementation found the test drives the
  *ref-shift* branch instead (see Discussion).
- **Cold-start IK budget guard** — never tested.

Plan 2 framing: "drive real failures, not synthetic ones."  Each
Phase 2 scenario test keeps the failure arm on a real solver call
(via `_StatsInjector` from Phase 1) and drives the escalation
trigger through the smallest internal-state mutation that isolates
one branch.

## Design

### Per-test recipe — empirical-probe table

Pre-implementation probes (`/tmp/probe_walk_forward.py`,
`/tmp/probe_cold_start_ik.py`; not committed) confirmed each
recipe before any test was written:

| ID         | Trigger                                | Driver                                                                           | Asserted status                              | Wall-clock |
|------------|----------------------------------------|----------------------------------------------------------------------------------|----------------------------------------------|-----------|
| T-U-T1b-1  | ref-shift > 20 mm                      | mutate `_ref_at_last_success_mid[2] -= 25`                                        | `hold_extrap(Maximum_CpuTime_Exceeded)`      | <1 s      |
| T-U-T1b-2  | ref-shift < 20 mm                      | mutate `_ref_at_last_success_mid[2] -= 5`                                         | `fallback(Maximum_CpuTime_Exceeded)` + `fallback_step=1` | <1 s |
| T-U-T1b-3  | staleness > 500 ms                     | `time.sleep(0.6)` between seed and failure                                       | `hold_extrap(Maximum_CpuTime_Exceeded)`      | ~1 s      |
| T-U-T1b-4  | counter > `max_consecutive_failures`   | `max_consecutive_failures=2`; inject 4 consecutive `_StatsInjector` failures      | `[fallback, fallback, hold_extrap, hold_extrap]` | <1 s |
| T-U-T1b-5  | per-node IK budget exhaustion          | monkey-patch `_numerical_ik` to `time.sleep(0.42)` per call; `max_cpu_time=2.0`   | `logger.info("_cold_start: per-node IK budget exceeded at node 2/10; ...")` | ~1 s |
| T-U-T1b-6  | (across-call warm-start integrity)     | `RuleBasedStateMachine` over 4 rules                                              | invariant: `_prev_w is None or np.all(np.isfinite(_prev_w))` | 24 s ci-fast / 441 s ci-deep |

The probe scripts also confirmed three subtleties the plan's text did
not explicitly call out:

- The mid-node index is `min(self._N, 5)` (mpc.py W7 block); on
  `N=10` this is index 5.  The snapshot is taken at success on this
  index, and the comparison reads the *current* `_last_ref_traj` at
  the same index.
- The walk-forward arm sets `diag['fallback_step']`; the
  `hold_extrap` arm does not.  Asserting on the presence/absence of
  this key is a stronger check than substring-matching the status
  string alone.
- `_handle_failure` is always called from `solve()` with
  `q_dot=state.leg_velocities_mmps`, so the escalation path is
  `hold_extrap(...)` not `hold(...)` even though the plan summary
  uses the shorter "hold".  T-U-T1b-4's assertions reflect the
  actual production path.

### T-U-T1b-1, -2: snapshot mutation rather than honest end-to-end

Two driver options were considered for the ref-shift threshold:

(a) **Honest end-to-end.**  Run a successful solve at one
target_pose; then on subsequent ticks, use a `target_pose` shifted
by 30 mm while injecting failures.  The threshold is crossed by
changing the input ref between calls.

(b) **Direct snapshot mutation.**  Seed via a successful solve, then
mutate `_ref_at_last_success_mid[2]` to introduce a delta vs the
current `_last_ref_traj` mid-node, then inject one failure.

Phase 2 chose (b).  Rationale:

- The mutation isolates the threshold cleanly: a 25 mm xyz delta is
  unambiguously above 20 mm without coupling to twist direction-flip,
  target-feasibility, scheduler-event timing, or reference-builder
  behaviour.  The W7 contract is independent of how the ref shifted;
  it depends only on the snapshot-vs-current delta.
- It mirrors Phase 1's `_StatsInjector` discipline — failure
  injection at the smallest internal-state surface that exposes the
  branch.  The honest end-to-end driver was probed and works (see
  the probe table) but adds 2-3 solves of latency per test and
  couples to reference-builder behaviour orthogonal to the W7
  contract.

The pair-test pattern (-1 above the threshold, -2 below) is the
canonical way to assert a threshold contract: each test pins one
branch of the boundary, and a regression that flips the comparison
operator (`>` → `>=` accidentally rounding 20.0 down) breaks one
test, not both.

### T-U-T1b-3: real wall-clock vs perf_counter monkey-patch

Two driver options for staleness:

(a) **Real wall-clock `time.sleep(0.6)`** between seed and failure.

(b) **Direct mutation `mpc._t_at_last_success -= 0.6`** to simulate
wall-clock advance.

Phase 2 chose (a).  Rationale:

- Phase 1's `test_max_cpu_time_exceeded` already accepts wall-clock
  budgets; staying in the same idiom (real wall-clock advance,
  `perf_counter()` consulted by the production code as-is) preserves
  consistency.
- The cost is 0.6 s per test run, acceptable in a suite that already
  runs ~290 s.
- The probe confirmed the direct-mutation path behaves identically
  (Probe D in `/tmp/probe_walk_forward.py`); the choice is
  consistency, not behaviour.

### T-U-T1b-4: audit of the existing escalation test

The plan called for "verify and extend with edge cases" against
[test_mpc_static.py::test_escalation_fallback_to_hold](../tests/sim/test_mpc_static.py).
The audit found the existing test does NOT exercise
`max_consecutive_failures` cleanly:

- It seeds at `[0, 0, 50, 0, 0, 0]` and drives failures against
  `_INFEASIBLE_REF = [500, 500, 500, 0, 0, 0]`.
- The 500 mm xyz shift is **25× above** the 20 mm walk-forward-unsafe
  threshold.  The first failure tick already trips the ref-shift
  branch and routes into `hold_extrap`.
- The loose assertions `fallback_count >= 1` / `hold_count >= 1`
  hide the surprise: `hold_count` is satisfied from the first tick
  (via ref shift), not from the counter exceeding the bound.

T-U-T1b-4 is the **strict-status, isolated-branch** test the existing
one only purported to be: same `REF_NORMAL` for both seed and failures
(no walk-forward trip from ref shift), `_StatsInjector` for the
failures (no walk-forward trip from staleness during the
fast-injected sequence), and per-tick exact-status assertions
(`[fallback, fallback, hold_extrap, hold_extrap]` with
`max_consecutive_failures=2`).

The existing `test_escalation_fallback_to_hold` is left in place
unchanged — it is structurally similar to T-U-T1b-1's
"`hold_extrap` from large ref shift" but with synthetic infeasibility,
so the historical test still validates the integration.  T-U-T1b-4
adds the strict-counter-branch coverage the existing test lacked.

This pattern (audit existing tests for "asserts the right thing for
the wrong reason") is the canonical way Phase 1 caught the
`Maximum_Iterations_Exceeded`-vs-`Infeasible_Problem_Detected`
mismatch — the [Phase 1 logbook](2026-05-11-tier1a-real-solver-failures.md)
documents the precedent.

### T-U-T1b-5: the IK budget guard has no diag sentinel

The plan's text hedged
`diag['cold_start_method'] == 'linear_interp'` "or whichever sentinel
the code surfaces".  Empirically, the code surfaces NO `diag` key for
this branch — the only observable is the `logger.info` message at the
budget-exceeded log statement in `_cold_start`.

The test asserts on the log record via a manual handler attached to
`logging.getLogger('controller.mpc')`.  Pytest's `caplog` fixture was
considered but the global level/filter chain it configures interacts
poorly with the per-call MPC log volume; a dedicated handler scoped
to the `controller.mpc` logger is cleaner and matches the
log-capture pattern used elsewhere in the suite.

A defensive `assert 'cold_start_method' not in diag` clause is
included in the test.  If a future refactor *does* add a diag key
(e.g., as part of a Tier-3 schema-completeness effort), this
assertion fails loudly and prompts a coordinated update across this
test, the plan, the logbook, and downstream consumers of `diag`.

The test also validates the post-budget-exhaustion property: the
cold-start STILL completes (linear-interp seed remains a valid
IPOPT initial guess), and the downstream solve returns a finite
`cmd`.  The budget guard is defense-in-depth, not a failure mode —
this assertion is the primary safety property and the log message is
the secondary observability signal.

### T-U-T1b-6: `RuleBasedStateMachine` shape and singleton management

The warm-start integrity invariant is across-call by definition: a
sequence of failures, recoveries, ref shifts, and staleness advances
must not leave `_prev_w` in a NaN/Inf state at any observable point.
A pure `@given` test would only sample one call at a time;
`RuleBasedStateMachine` is the right tool.  Mirrors the
[`SchedulerStateMachine` pattern](../tests/sim/test_scheduler_contract.py)
landed by Plan 1 Phase 3.

**Singleton-MPC pitfall.**  First implementation captured the
`_real_solver` per StateMachine instance:

```python
def __init__(self):
    if hasattr(self, '_real_solver'):  # never True — fresh instance
        self._mpc._solver = self._real_solver
    self._real_solver = self._mpc._solver  # captures whatever's there
```

With a module-scoped singleton MPC reused across walks, a prior walk
that ended with `_StatsInjector` active leaves the injector in
`mpc._solver`.  The next `__init__` captures the *injector* as
`_real_solver`, then `_do_succeed`'s assert
`diag['status'] == 'Solve_Succeeded'` fires because the injector
returns the previously-injected status.  Hypothesis attempts to
shrink the failing example, observes different behaviour on replay
(because the injector status depends on whichever rule fired last in
the prior walk), and raises `FlakyStrategyDefinition`.

Fix: capture the real solver **once at module init** (inside
`_w7_get_singletons`), and unconditionally restore it in `__init__`
before resetting state.  After this fix the property holds at ci-deep
(1000 examples) without flakiness.

The lesson generalises beyond this test: shared mutable state across
hypothesis StateMachine instances must be *fully* restored in
`__init__`, including any state mutated as a side effect of test
infrastructure (solver swaps, monkey-patches), not just the
under-test object's own state.  Any field captured per-instance is
captured *post-restoration* and inherits whatever the prior walk
left in place.

## Implementation

### tests/sim/test_solver_failures.py — Phase 2 extension

Adds four scenario classes and one stateful property class:

| Class                                       | ID(s)                | Tests | Strategy                                       |
|---------------------------------------------|----------------------|-------|------------------------------------------------|
| `TestWalkForwardRefShiftThreshold`          | T-U-T1b-1, T-U-T1b-2 | 2     | Pair test; snapshot mutation                   |
| `TestWalkForwardStaleness`                  | T-U-T1b-3            | 1     | Real wall-clock `time.sleep(0.6)`              |
| `TestMaxConsecutiveFailuresEscalation`      | T-U-T1b-4            | 1     | Strict-status sequence with `_StatsInjector`   |
| `TestColdStartIkBudget`                     | T-U-T1b-5            | 1     | `_numerical_ik` monkey-patch + manual log handler |
| `W7WarmStartIntegrityMachine` (TestCase)    | T-U-T1b-6            | 1     | `RuleBasedStateMachine`                        |

All tests reuse Phase 1's helpers (`_create_mpc`, `_seed_warm_start`,
`_StatsInjector`).  The `_RaisingSolver` and
`_pin_q_decision_bounds_infeasible` helpers are not used by Phase 2.

Module docstring updated to a Phases-1-and-2 framing with a
combined per-test recipe table; the Tier 1a table is preserved
verbatim and the Tier 1b table is appended.  The conftest /
hypothesis profiles in [tests/conftest_hypothesis.py](../tests/conftest_hypothesis.py)
are unchanged — T-U-T1b-6 inherits the existing `ci-fast` /
`ci-deep` defaults.

### Cross-cutting validations in each scenario test

- T-U-T1b-1: assert exact status `hold_extrap(Maximum_CpuTime_Exceeded)`,
  `'fallback_step' not in diag`, `consecutive_failures == 1`,
  `np.all(np.isfinite(cmd))`.
- T-U-T1b-2: assert exact status `fallback(Maximum_CpuTime_Exceeded)`,
  `diag['fallback_step'] == 1`, counter and finiteness same as -1.
- T-U-T1b-3: assert exact status, `'fallback_step' not in diag`,
  counter and finiteness same as -1.
- T-U-T1b-4: assert per-tick exact status sequence
  `[fallback, fallback, hold_extrap, hold_extrap]`,
  monotonically incrementing `fallback_step` for ticks 1–2,
  `'fallback_step' not in diag` for ticks 3–4,
  `consecutive_failures == 4`.
- T-U-T1b-5: assert exactly one `'budget exceeded'` log record
  with format prefix `_cold_start: per-node IK budget exceeded`
  and substring `linear-interpolating remainder`,
  `np.all(np.isfinite(cmd))`,
  `'cold_start_method' not in diag` (defensive contract assertion).

### W7WarmStartIntegrityMachine — invariants asserted after every rule

| Invariant                          | Predicate                                                                          |
|------------------------------------|------------------------------------------------------------------------------------|
| `warm_start_finite_or_none`        | `_prev_w is None or np.all(np.isfinite(_prev_w))` — load-bearing safety property   |
| `prev_u_finite_or_none`            | `_prev_u is None or np.all(np.isfinite(_prev_u))` — walk-forward rate-limit anchor |
| `snapshot_finite_or_none`          | `_ref_at_last_success_mid` and `_twist_at_last_success` both finite-or-None        |

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple per the workflow rule on test-count claims — bare counts are
unverifiable from the artefact alone.

### Baseline (post Phase 1, pre Phase 2)

- `pytest tests/ -q`, run 2026-05-11 against SHA `31a4dae`:
  **1210 passed + 1 xfailed in 287.69 s.**

### Full-suite gate (post Phase 2)

- `pytest tests/ -q`, run 2026-05-11 with the Phase 2 changes
  applied: **1216 passed + 1 xfailed in 321.35 s.**  +6 net delta
  matches the six new tests (2 + 1 + 1 + 1 + 1).  Zero regressions
  on existing tests.

### Module-isolated run

- `pytest tests/sim/test_solver_failures.py -q`, run 2026-05-11:
  **23 passed + 1 xfailed in 28.86 s.**  Phase 1's 17 + 1 xfailed
  plus Phase 2's 6 new tests.

### Property test depth — ci-deep validation

- `pytest tests/sim/test_solver_failures.py::TestW7WarmStartIntegrity
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q`, run 2026-05-11:
  **1 passed in 441.45 s** (1000 examples).  The W7 warm-start
  integrity invariant holds across the strategy space (succeed × fail
  × shift × time advance) at nightly depth with deterministic seed.

### Hot-loop allocation contract — regression check

- `pytest tests/sim/test_hot_loop_allocation_contract.py
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q`, run
  2026-05-11: **3 passed in 15.49 s.**  The deferred Working Note #5
  mitigation (see Discussion) is justified — Phase 2's single
  `RuleBasedStateMachine` does not generate the heap pressure that
  Phase 3's NaN/Inf fuzz is expected to.

## Discussion

### Audit of the existing test_escalation_fallback_to_hold

This is the second instance in Plan 2 of an existing test asserting
the right thing for the wrong reason.  Phase 1 caught:

> The "infeasibility-via-flip-bounds" trick in
> `test_mpc_static.py::_INFEASIBLE_REF` is a misnamed test.  It
> routes through the exception handler, not the IPOPT classifier.

Phase 2 catches:

> `test_escalation_fallback_to_hold` claims to validate
> `max_consecutive_failures` but actually validates the 20 mm
> ref-shift branch — the 500 mm `_INFEASIBLE_REF` shift trips the
> walk-forward-unsafe threshold on the very first failure tick.

Both pre-existing tests still PASS (their loose assertions are
satisfied by either branch), so the audit is not a regression
finding — it's a coverage gap.  T-U-T1b-1 (ref-shift) and T-U-T1b-4
(counter) split the over-broad existing test into two
single-branch tests with strict per-tick status sequences.

The structural lesson: when extending an existing failure-path test,
trace the actual execution path with a probe script BEFORE assuming
the test's name reflects what it exercises.  The name reflects the
author's intent; the implementation reflects the actual coverage.

### Xfail accounting

Phase 2 added **0 new xfails**.  The single xfail on the suite
(T-U-T1a-4 ``Restoration_Failed``) is unchanged from Phase 1; its
three-field accounting (test ID, tracking reference, target close)
lives in [Phase 1's logbook entry](2026-05-11-tier1a-real-solver-failures.md)'s
Discussion → "Xfail accounting" subsection.  Total xfails on the
suite remain 1 — consistent with the pre-Phase-2 baseline (1210
passed + 1 xfailed) and the post-Phase-2 result (1216 passed +
1 xfailed).

This subsection is preserved as the canonical archival-gate
artefact for Plan 2: a future archival pass scans for an "Xfail
accounting" heading in each phase's logbook entry, so making the
zero-additions statement explicit (rather than relying on the
arithmetic delta) protects the discipline.

### Working Note #5 mitigation — deferred to Phase 3 by design

Working Note #5 in the plan flagged that the hot-loop allocation
contract may flake more under Plan 2's hypothesis fuzz, with two
proposed mitigations:

(a) `gc.collect()` at the start of `test_hot_loop_allocation_contract`.

(b) Mark the test `slow` and run it in a separate pytest invocation.

The note's explicit guidance: *"Don't wait for the second flake —
bake a mitigation in before Phase 3 lands."*

Phase 2 introduces the FIRST hypothesis property test in Plan 2
(T-U-T1b-6).  The case for pre-emption is real but materially
weaker than Phase 3's:

- Phase 2 adds one stateful machine with 4 rules and 3 invariants.
- Phase 3 will add NaN/Inf fuzz across `solve()` inputs plus a
  separate stateful warm-start machine — the heap pressure scales
  with the number of strategies × the number of fuzz arenas.

The decision (surfaced to the user via `AskUserQuestion`): **defer
to Phase 3**.  Rationale —

- The hot-loop allocation contract passed at ci-deep
  (`--hypothesis-profile=ci-deep --hypothesis-seed=0`) AFTER Phase 2
  landed.  No flake observed; the structural pressure is below the
  threshold where the mitigation is load-bearing.
- Phase 3's fuzz surface is an order of magnitude larger than
  Phase 2's; baking the mitigation into the same commit as Phase 3
  keeps the cause and the protection adjacent in `git log` and the
  logbook.
- The Working Note's text reads "before Phase 3 lands" — Phase 3 is
  the next phase, and Phase 2's logbook entry is the appropriate
  place to record the deferral so a future maintainer reading either
  Phase 2 or Phase 3's commit history sees the explicit decision.

If Phase 3's first commit introduces flake, the mitigation lands in
Phase 3 itself.  If Phase 2's hypothesis property test starts
flaking ci-deep before Phase 3 ships, the mitigation lands as a
follow-up commit on Phase 2 with an amendment to this entry.

### Real-driver discipline in Phase 2

The plan's most important self-imposed constraint — "drive real
failures, not synthetic ones" — applies to Phase 2 with two
adjustments from Phase 1's pattern:

- **Failures arm (every test):** real solver execution via
  `_StatsInjector`.  The classifier sees the injected status string
  but the solver runs to completion against the real CasADi NLP.
  Identical pattern to Phase 1's `TestFallbackKeywordMatrix`.
- **Trigger arm (per-test):** the smallest internal-state mutation
  that isolates the targeted escalation branch — direct snapshot
  mutation for ref-shift (T-U-T1b-1, -2), real wall-clock sleep for
  staleness (T-U-T1b-3), monkey-patched `_numerical_ik` for the IK
  budget (T-U-T1b-5).  T-U-T1b-4 uses the most honest driver
  (`_StatsInjector` injection of consecutive failures with a steady
  ref) — no internal-state mutation needed for that branch.

The trigger-arm compromises are justified the same way Phase 1
justified `_StatsInjector` and `_RaisingSolver`: the production
contract under test is independent of *how* the trigger fires, so
isolating one branch with the minimum-surface mutation produces a
cleaner test than driving an end-to-end sequence that happens to
trip the branch as a side effect.

### What Phase 2 reveals about the W7 escalation contract

Two non-obvious behaviours surfaced by the Phase 2 implementation:

1. **The `hold_extrap` arm (not `hold`) is the live escalation
   target.**  `solve()` always passes `q_dot=state.leg_velocities_mmps`
   to `_handle_failure`; the `hold(...)` branch
   only fires when `q_dot` is None (a code path no production
   caller takes).  The plan summary used "hold" as shorthand;
   T-U-T1b-4 asserts the actual `hold_extrap(...)` path.  A future
   refactor that drops the `q_dot` argument or adds a code path
   without it would change the escalation status string from
   `hold_extrap(...)` to `hold(...)` — T-U-T1b-4 would fail loudly
   and surface the change for review.

2. **The walk-forward arm exposes its progress via
   `diag['fallback_step']`.**  This key is set on the `fallback(...)`
   path and absent on the `hold_extrap(...)` / `hold(...)` /
   `cold_hold(...)` paths.  Asserting on its presence/absence is a
   stronger structural check than substring-matching status strings.
   This has implications for the Tier 3a schema-fuzz work in Phase 7 —
   the schema completeness check should treat `fallback_step` as a
   path-conditional key, not a universal one.

3. **The cold-start IK budget guard is silent in `diag`.**  Only
   the `logger.info` message records the breach.  This is a Tier 3
   schema-completeness gap (the operational signal "cold-start
   degraded to linear-interp" is non-`diag`-observable); whether it
   should be promoted to a `diag` key is a Phase 7 question.  Phase
   2's defensive `assert 'cold_start_method' not in diag` will
   serve as the contract anchor if/when Phase 7 elects to add it.

### Convert line citations to symbol references — Phase 2 application

Working Note #3 recommends refreshing line-number citations to
symbol references.  Phase 2's tests cite symbols
(`_handle_failure`, `_cold_start`, `_numerical_ik`,
`_ref_at_last_success_mid`, `_t_at_last_success`,
`_COLD_START_BUDGET_FRACTION`) rather than line numbers in their
docstrings.  The exception is the W7 block reference in the
`TestWalkForwardRefShiftThreshold` class docstring, which uses the
phrase "the W7 block in `_handle_failure`" — a symbolic anchor
that survives any refactor that keeps the W7 logic in
`_handle_failure`.

If the W7 block is hoisted into its own method (e.g.,
`_check_walk_forward_unsafe`), the test docstrings should be
refreshed to name the new symbol; the assertion targets are
public/internal-state surfaces that don't change with that refactor.

## Open Questions

- **Should the ref-shift threshold be tested on x and y axes
  separately?**  T-U-T1b-1, -2 only mutate the z axis.  The W7
  comparison uses `np.max(np.abs(delta))` on all three xyz axes, so
  a regression that accidentally restricted the comparison to one
  axis would pass T-U-T1b-1 (z still triggers) but might escape
  detection if the bug happened to keep z working.  A future
  parametrisation over (x, y, z) would close this; the current
  single-axis tests are adequate for the threshold-value contract.

- **Should T-U-T1b-3 also test the boundary at 500 ms exactly?**  A
  test at ~499 ms (stays in fallback) and a test at ~501 ms (goes
  to hold_extrap) would pin the boundary the same way T-U-T1b-1, -2
  pin the 20 mm boundary.  Currently only the supra-threshold test
  exists.  Cost: each adds ~0.5 s of wall-clock.  Deferred — the
  threshold value is a normative invariant in the W7 block and a
  refactor that changes it would either be intentional (and
  refresh both test and docstring) or surface elsewhere.

- **Does the twist-direction-flip branch need its own dedicated
  test?**  Phase 2 asserts only the ref-shift and staleness branches
  of `walk_forward_unsafe`.  The twist-flip branch fires on linear
  axes where `abs(v_old) > 10 mm/s` and `v_new × v_old < 0`.
  Driving it cleanly requires a seed with non-zero twist
  (a moving-target reference, not the static REF_NORMAL used here).
  T-U-T1b-6's `RuleBasedStateMachine` does NOT exercise it because
  the rules don't shift `_twist_at_last_success`; coverage is
  structurally absent.  Filed as a Tier 3 follow-up rather than
  extending Phase 2.

- **Should T-U-T1b-6's rules include an `unswap_solver` rule?**  The
  current state machine swaps to `_StatsInjector` on `fail_solve`
  and unswaps inside `_do_succeed`.  An explicit rule that swaps
  back without running a solve would let hypothesis explore the
  "swap, swap, swap, swap, succeed" pathological sequence.  Cost:
  adds one rule and one strategy axis.  Benefit: marginal — the
  invariant is on `_prev_w`, which is independent of the solver
  reference.  Deferred.

## Related

- [plans/active/mpc-sadpath-coverage-tiers-1-3.md](../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 2 specification.
- [logbook/2026-05-11-tier1a-real-solver-failures.md](2026-05-11-tier1a-real-solver-failures.md)
  — Phase 1 (Tier 1a); helper patterns Phase 2 reuses.
- [logbook/2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md](2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md)
  — `SchedulerStateMachine` exemplar that T-U-T1b-6 mirrors.
- [logbook/2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles.md](2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles.md)
  — Working Note #5 origin (hot-loop flake under hypothesis fuzz).
- [logbook/2026-04-23-hot-loop-zero-allocation-contract.md](2026-04-23-hot-loop-zero-allocation-contract.md)
  — Hot-loop allocation contract verified green after Phase 2.
- [controller/mpc.py](../controller/mpc.py) — `_handle_failure`,
  `_cold_start`, W7 walk-forward heuristic under test.
- [tests/sim/test_solver_failures.py](../tests/sim/test_solver_failures.py)
  — this phase's test additions.
