---
title: MPC Sad-Path Coverage Rollup — Tiers 1–3
created: 2026-05-08
status: active
---

# MPC Sad-Path Coverage Rollup — Tiers 1–3

## Context

### Why this plan exists

The 2026-05-08 MPC-layer audit (conversation, no logbook entry) identified ~60% of documented failure modes as untested. The structural pattern: **where contracts exist, sad-path coverage is rigorous; where contracts don't, coverage is threadbare.** Plan 1 (`mpc-tier0-contracts.md`) closes the contract gap for the scheduler and plant adapter. This plan closes the coverage gap.

The work is grouped into three tiers by the consequence of the failure on the robot:

- **Tier 1 — Solver-fallback safety paths.** What happens when IPOPT actually fails (infeasibility, timeout, exception)? When the fallback escalation cascade fires? When inputs to `solve()` contain NaN? Today most "fallback tests" use synthetic infeasible references rather than driving real solver failures. The substring matching against `_FALLBACK_KEYWORDS` is untested against real CasADi exit strings.
- **Tier 2 — HardwarePlant edges.** FK divergence + watchdog at `hardware_plant.py:195`; singular Jacobian zero-twist at `:715`; frozen-motor detector at `:163`; telemetry staleness threshold matrix (warn/hard/estop) at `:67–71`; `set_pose()` torque-FF singular path at `:782`; cold-start zero-state at `:507`; ZMQ corruption + version skew. Several of these are documented but never directly exercised.
- **Tier 3 — Every conceivable failure mode.** Numerical edge case fuzz across every public API; schema-drift fuzz on the `diag`/`extras` namespaces (a fallback path that forgets to populate a key produces `KeyError` in `record_from_arrays`); time pathologies (clock skew, dropped tick, dt change mid-run); resource exhaustion (event-queue, telemetry pool); status-substring brittleness; hook contract failures; concurrent-reset races.

### What this plan achieves

1. **Drives real solver failure paths** — not synthetic ones — and verifies the fallback machinery responds correctly (Tier 1).
2. **Exercises every documented failure mode in the hardware adapter** (Tier 2).
3. **Establishes property-based coverage** for input-domain failure modes across the MPC API surface (Tier 3).
4. **Replaces `FakeIPC` with real-msgpack-bytes harness** in the tests where IPC corruption matters (Tier 2).
5. **Closes the schema-completeness gap** on `diag`/`extras` — every fallback path populates the documented schema or the test catches the omission.

### When to do this

**After** Plan 1 (`mpc-tier0-contracts.md`) lands. Tier 1 phases reference contract-defined invariants in places (e.g., S6 monotonic sim_time when fuzzing time pathologies in Tier 3).

**Concurrent with** the in-flight `refactor` branch. Test additions only (except Phase 0, which closes Plan 1's deferred bugs; see Notes). Production-code changes surfaced by later-phase tests are tracked separately per "Production-code changes triggered by tests" in Notes.

**Prerequisites:** Plan 1 phases 1–3 (Scheduler contract enforced) and 5–6 (PlantInterface P1, P2, P4 enforced). Phase 8 (CI hypothesis profiles) ideally landed; if not, tier tests use a temporary local profile registration.

### Related

- [mpc-tier0-contracts.md](../archived/2026-05-10%20mpc-tier0-contracts.md) — Plan 1, prerequisite
- [REFERENCE_LAYER_CONTRACT.md](../../controller/REFERENCE_LAYER_CONTRACT.md) — K1–K6
- [HOT_LOOP_CONTRACT.md](../../controller/HOT_LOOP_CONTRACT.md) — hot-loop budget (must remain green throughout)
- [SCHEDULER_CONTRACT.md](../../controller/SCHEDULER_CONTRACT.md) — landed by Plan 1
- [PLANT_INTERFACE_CONTRACT.md](../../controller/PLANT_INTERFACE_CONTRACT.md) — landed by Plan 1
- [logbook/2026-04-23-hot-loop-zero-allocation-contract.md](../../logbook/2026-04-23-hot-loop-zero-allocation-contract.md) — process model

## Architecture

### Current

```
[ MPC layer surface ]                  [ failure modes ]                      [ test coverage ]
─────────────────────                  ──────────────────                     ────────────────
mpc.solve()                       ◀──  Solver_Succeeded path                    test_mpc_static.py:: ~10 happy-path tests
                                       Fallback (synthetic infeasible ref)     test_mpc_static.py::test_fallback_on_failure
                                       Real IPOPT timeout                      <UNTESTED>
                                       Real IPOPT infeasible                   <UNTESTED>
                                       NaN/Inf in inputs                       <UNTESTED> (only finite-check on output)
                                       walk_forward_unsafe escalation          <UNTESTED> (logic exists at mpc.py:1574–1592)
                                       Cold-start IK budget exhaustion         <UNTESTED>

hardware_plant.get_state()        ◀──  FK convergence path                      test_hardware_plant_deadband.py
                                       FK divergence + watchdog                <UNTESTED>
                                       Singular Jacobian                        <UNTESTED>
                                       Frozen-motor detector                    <UNTESTED>
                                       Telemetry stale (warn/hard/estop)        <UNTESTED> (3 thresholds, no test fires any)
                                       Cold-start zero-state                    <UNTESTED>

hardware_plant.set_pose()         ◀──  Nominal torque-FF path                   test_plant.py
                                       Torque-FF singular Jacobian              <UNTESTED>

ZMQ telemetry / cmd path          ◀──  Nominal recv/send                        test_zmq_target.py (FakeIPC)
                                       Partial msgpack frame                    <UNTESTED>
                                       Version-skew (added/removed field)       <UNTESTED>
                                       Connection drop mid-tick                  <UNTESTED>

diag dict / extras namespace      ◀──  Solve_Succeeded path populates 7 keys   test_mpc_static.py asserts a few keys
                                       Fallback path populates schema?          <UNTESTED across all fallback statuses>
                                       hold_extrap path populates schema?       <UNTESTED>
                                       cold_hold path populates schema?         <UNTESTED>

Time / resource / hooks           ◀──  Steady-state                             test_mpc_dynamic.py
                                       Backward sim_time                        <UNTESTED> (Plan 1 S6 catches some)
                                       Dropped tick                             <UNTESTED>
                                       dt schedule change mid-run               <UNTESTED>
                                       Telemetry pool overflow                  <UNTESTED>
                                       Event-queue overflow                     <UNTESTED> (Plan 1 S3 catches some)
                                       on_target_override returning None        <UNTESTED>
                                       Concurrent reset race                    <UNTESTED>
```

### Proposed

```
[ MPC layer surface ]                  [ failure modes ]                      [ test coverage ]
─────────────────────                  ──────────────────                     ────────────────
mpc.solve()                       ◀──  Solver_Succeeded path                    test_mpc_static.py (unchanged)
                                       Real IPOPT timeout                      tests/sim/test_solver_failures.py (NEW)
                                       Real IPOPT infeasible                   tests/sim/test_solver_failures.py
                                       _FALLBACK_KEYWORDS exit-code matrix     tests/sim/test_solver_failures.py
                                       walk_forward_unsafe escalation          tests/sim/test_solver_failures.py
                                       Cold-start IK budget exhaustion         tests/sim/test_solver_failures.py
                                       NaN/Inf in inputs (hypothesis fuzz)     tests/sim/test_mpc_input_fuzz.py (NEW)

hardware_plant.get_state()        ◀──  FK divergence + watchdog                tests/sim/test_hardware_plant_failure_paths.py (NEW)
                                       Singular Jacobian
                                       Frozen-motor detector
                                       Telemetry stale matrix
                                       Cold-start zero-state

hardware_plant.set_pose()         ◀──  Torque-FF singular Jacobian             tests/sim/test_hardware_plant_failure_paths.py

ZMQ                               ◀──  Real-msgpack corruption harness         tests/sim/test_zmq_corruption.py (NEW)
                                       Version-skew (field add/remove)         (drops FakeIPC for these tests)
                                       Connection drop simulation

diag / extras schema              ◀──  Schema completeness over all statuses  tests/sim/test_diag_schema_fuzz.py (NEW)

Time / resource / hooks           ◀──  Time-pathology fuzz                     tests/sim/test_mpc_time_pathologies.py (NEW)
                                       Resource exhaustion
                                       Hook contract failures
                                       Concurrent reset races
```

### IPC / Message formats

**No new IPC channels.** This plan is test additions only. One change to existing test infrastructure:

- `tests/sim/_zmq_test_harness.py` (new) — a real-ZMQ harness that wraps `MpcTargetIPC` so tests can inject corrupted `msgpack` bytes and observe behaviour. Replaces `FakeIPC` for the corruption-specific tests; existing happy-path tests in `test_zmq_target.py` continue using `FakeIPC` for speed.

### Files to create

| Path | Purpose |
|------|---------|
| `tests/sim/test_solver_failures.py` | Real IPOPT failure scenarios + escalation cascade |
| `tests/sim/test_mpc_input_fuzz.py` | NaN/Inf hypothesis fuzz on `solve()` inputs |
| `tests/sim/test_hardware_plant_failure_paths.py` | FK divergence, singular Jacobian, frozen motor, telemetry staleness matrix, set_pose FF singular |
| `tests/sim/test_zmq_corruption.py` | Real-msgpack corruption + version-skew |
| `tests/sim/_zmq_test_harness.py` | Helper: real-ZMQ wrapper for corruption injection |
| `tests/sim/test_diag_schema_fuzz.py` | Schema completeness across every fallback status |
| `tests/sim/test_mpc_time_pathologies.py` | Clock skew, dropped tick, dt mid-run change, hook failures, concurrent reset |

### Files to modify

Test additions only by design (except Phase 0; see Notes). Production-code changes triggered by tests in Phases 1–8 are tracked separately (see "Production-code changes triggered by tests" in Notes for Collaborators) and land in their own commits with logbook entries.

## Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| 0 | Close Plan 1's deferred `@precondition` gates (cancel_next mid-TRANSITIONING + begin_return overwrite) | COMPLETE | 2026-05-11 | Med | Plan 2 starts on a clean foundation; the "no `@precondition` for known bugs" rule isn't immediately self-violating |
| 1 | Tier 1a — Real IPOPT infeasibility + timeout exit codes | COMPLETE | 2026-05-11 | Med | Solver-fallback latching on every documented exit code |
| 2 | Tier 1b — Fallback escalation cascade + cold-start IK budget | COMPLETE | 2026-05-11 | Med | walk_forward_unsafe → hold_extrap escalation; IK budget exhaustion path |
| 3 | Tier 1c — NaN/Inf input fuzz on `solve()` | COMPLETE | 2026-05-11 | Med | Adversarial inputs route through `_handle_failure`, never corrupt warm-start |
| 4 | Tier 2a — HardwarePlant FK degradation | COMPLETE | 2026-05-11 | High | FK divergence watchdog, singular Jacobian, frozen-motor detector |
| 5 | Tier 2b — HardwarePlant telemetry & FF | NOT STARTED | | High | Staleness threshold matrix; set_pose torque-FF singular; cold-start zero-state |
| 6 | Tier 2c — ZMQ corruption (real-msgpack harness) | NOT STARTED | | Med | Partial frame, version-skew, connection drop |
| 7 | Tier 3a — Numerical + schema fuzz | NOT STARTED | | Med | Every public API NaN-safe; `diag`/`extras` schema-complete |
| 8 | Tier 3b — Time pathologies, resources, hooks, races | NOT STARTED | | Med | Clock-skew, dt change, hook failures, concurrent-reset, resource exhaustion |

## Implementation Phases (detailed)

### Phase 0: Close Plan 1's deferred `@precondition` gates — COMPLETE (2026-05-11)

**Outcome.** Both gates removed.  Bug 1 (`cancel_next` mid-TRANSITIONING)
fixed in commit `167b9f8` —
[logbook entry](../../logbook/2026-05-10-scheduler-cancel-next-during-transitioning.md).
Bug 2 (`begin_return` slot-overflow) fixed in commit `239a35d` —
[logbook entry](../../logbook/2026-05-10-scheduler-begin-return-s3-overwrite.md).
Both fixes raise `ValueError` (`cancel_next` with its own diagnostic
about TRANSITIONING semantics; `begin_return` via the existing
`_validate_slot_capacity('begin_return')` helper for canonical S3
message symmetry with `submit_event`).
`SchedulerStateMachine.cancel_next` and `.begin_return` rules
un-gated; both now wrap the call in `try/except ValueError` and
assert both the canonical message substring AND the phase/slot-state
preconditions at raise time.
[SCHEDULER_CONTRACT.md](../../controller/SCHEDULER_CONTRACT.md)
gained a new "S3 — Phase preconditions on slot mutators" sub-section
(introduced in 167b9f8 with the `cancel_next` clause; extended in
239a35d with the `begin_return` clause).  Full-suite ci-deep
(`pytest tests/ -q --hypothesis-profile=ci-deep`, run 2026-05-11):
**1193 / 1193 pass in 560.98 s** — every contract and property test
green at nightly depth.  Phase 1 (Tier 1a real IPOPT failures)
cleared to start.

*Note: the Scope / Bugs to fix / New-modified files / Test cases /
Critical details / Exit criteria sub-sections below are preserved as
the as-planned record; line citations (e.g.
`test_scheduler_contract.py:999–1015`) and tense reflect pre-fix
state.  The Outcome paragraph above is authoritative for what
actually shipped; the two logbook entries are the canonical
post-mortems.*

**Scope.** Plan 1 Phase 3 landed two `@precondition` gates in
[tests/sim/test_scheduler_contract.py:999–1015](../../tests/sim/test_scheduler_contract.py#L999)
to suppress known scheduler bugs in the `SchedulerStateMachine` random walk.
The gates are correctly tagged as Plan 2 Tier-1 follow-ups in their inline
comments and in the [Phase 3 logbook](../../logbook/2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement.md)'s
Open Questions. Plan 2 starts by closing them out so the rest of the work
begins on a clean foundation — and so the rule "no `@precondition` for
known bugs" articulated in Plan 2's Working Notes (below) isn't
self-violating from day one.

**Bugs to fix.**

1. **`cancel_next` mid-TRANSITIONING** leaves the scheduler in an
   inconsistent state: the phase remains `TRANSITIONING` but
   `_next_event` is `None`. Subsequent ticks produce undefined behaviour.
   The fix is most likely either (a) `cancel_next` raises when called
   during `TRANSITIONING` (and the caller transitions to a different
   phase first), or (b) `cancel_next` transitions the scheduler to an
   appropriate phase (likely `HOLDING` or `RETURNING`) atomically.
2. **`begin_return` silently overwrites `_next_event`** when both
   slots are filled, bypassing the S3 capacity check. The fix is to
   raise per S3 (consistent with `submit_event`) — `begin_return` is
   not a documented bypass for S3.

Both bugs are state-machine semantic gaps orthogonal to S4/S5/S6 — Plan 1
intentionally scoped them out so the contract enforcement work could
ship.

**New/modified files.**
- `controller/scheduler.py` — production fix for both methods
- `tests/sim/test_scheduler_contract.py` — remove both `@precondition`
  gates; add direct scenario tests for the corrected behaviour
- `tests/sim/test_scheduler.py` — adjust any existing happy-path tests
  affected by the new fault behaviour

**Test cases.**

| ID | Test | How to drive | Pass criterion |
|----|------|--------------|----------------|
| T-U-T0-1 | `cancel_next` during `TRANSITIONING` | Submit → tick into TRANSITIONING → `cancel_next` | Documented behaviour (raise OR atomic transition); no inconsistent state |
| T-U-T0-2 | `cancel_next` during `APPROACHING` (regression) | Submit → tick into APPROACHING → `cancel_next` | Existing behaviour preserved; `_next_event = None`; phase unchanged at `APPROACHING` (next tick past arrival lands in `HOLDING`) |
| T-U-T0-3 | `begin_return` with both slots filled | Submit current + submit next → `begin_return` | Raises per S3; state unchanged |
| T-U-T0-4 | `begin_return` with only `_current_event` filled (regression) | Submit current → `begin_return` | Existing behaviour preserved |
| T-U-T0-5 | State machine: `cancel_next` rule un-gated | Remove `@precondition` at scheduler test line 999–1001 | Random walk passes at ci-deep |
| T-U-T0-6 | State machine: `begin_return` rule un-gated | Remove `@precondition` at scheduler test line 1011–1013 | Random walk passes at ci-deep |

**Critical details.**

- The two fixes are independent — land each in its own commit with its
  own logbook entry per the CLAUDE.md "Analyze control-system
  implications before changes" rule. Discuss the chosen behaviour
  (raise vs atomic transition) explicitly in each entry's Discussion
  section.
- The `@precondition` gates are removed in the *same* commit as the
  fix that makes them unnecessary — preserving the "fix → un-gate
  in one commit" symmetry that this plan's Production-code-changes
  rule requires.
- Run the full suite at ci-fast after each commit, and the full
  suite at ci-deep before declaring Phase 0 complete.

**Dependencies.** None. Plan 1 closure is the only prerequisite, and
that's done.

**Exit criteria.** Both `@precondition` gates removed from
`test_scheduler_contract.py`. `SchedulerStateMachine.TestCase` passes at
ci-deep with no rule gating. Two logbook entries (one per bug). Phase 1
of Plan 2 cleared to start.

---

### Phase 1: Tier 1a — Real IPOPT infeasibility + timeout exit codes — COMPLETE (2026-05-11)

**Outcome.** New file
[tests/sim/test_solver_failures.py](../../tests/sim/test_solver_failures.py)
adds 17 tests + 1 xfail covering the four IPOPT exit codes drivable
via `MPCParams` on the pinned CasADi 3.7.2 (`Maximum_CpuTime_Exceeded`,
`Maximum_Iterations_Exceeded`, `Infeasible_Problem_Detected`, plus
the CasADi-exception path), the success-keyword classifier matrix
over every documented IPOPT status string, and the warm-start /
counter-dynamics critical details from the plan.  T-U-T1a-4
(`Restoration_Failed`) is the documented permanent xfail — the
IPOPT-internal options that would force it (`start_with_resto`,
`expect_infeasible_problem`) are not exposed via `MPCParams` and
adding them would be a production-code change orthogonal to this
plan's discipline; structural coverage of the classifier for that
string is in `TestFallbackKeywordMatrix`.  See [logbook
entry](../../logbook/2026-05-11-tier1a-real-solver-failures.md) for
the per-exit-code recipe table, the design discussion (real-driver
vs synthetic-driver discipline; `_StatsInjector` rationale), and the
xfail accounting against Plan 2's archival-gate language.  Full
suite passes at ci-fast (1210/1210 + 1 xfailed in 287.81 s).
Phase 2 (Tier 1b — fallback escalation cascade + cold-start IK
budget) cleared to start.

*Note: the Scope / Test cases / Critical details / Exit criteria
sub-sections below are preserved as the as-planned record; the
Outcome paragraph above is authoritative for what actually shipped.*

**Scope.** Drive *real* CasADi exit codes (not synthetic infeasible references) and verify the fallback machinery responds correctly. Verify `_FALLBACK_KEYWORDS` substring matching against the actual strings IPOPT emits.

**New/modified files.**
- `tests/sim/test_solver_failures.py` (new)

**Test cases.**

| ID | Test | How to drive | Pass criterion |
|----|------|--------------|----------------|
| T-U-T1a-1 | Real `Maximum_CpuTime_Exceeded` | Construct an MPC with `max_cpu_time=1e-6` (effectively zero) so every solve exhausts budget | `diag['status']` matches `_FALLBACK_KEYWORDS`; `_handle_failure` runs; warm-start preserved |
| T-U-T1a-2 | Real `Infeasible_Problem_Detected` | Construct an MPC with explicitly contradictory bounds (e.g., `x_min > x_max` on one node) | `diag['status']` matches; fallback latches |
| T-U-T1a-3 | Real `Maximum_Iterations_Exceeded` | Construct an MPC with `max_iter=1` | `diag['status']` matches; fallback latches |
| T-U-T1a-4 | Real `Restoration_Failed` | Construct an MPC where IPOPT can't restore feasibility (combine 1a-2 with limited restoration iters) | `diag['status']` matches; fallback latches |
| T-U-T1a-5 | CasADi exception (programming error injection) | Patch the solver to raise on call | Caught in `mpc.py:1223–1238` exception handler; warm-start cleared (per code at `:1228–1234`); fallback runs |
| T-U-T1a-6 | `_FALLBACK_KEYWORDS` substring matrix | Parameterize over `['Solve_Succeeded', 'Solved_To_Acceptable_Level', 'Maximum_CpuTime_Exceeded', 'Maximum_Iterations_Exceeded', 'Infeasible_Problem_Detected', 'Restoration_Failed', 'Search_Direction_Becomes_Too_Small', 'Diverging_Iterates', 'User_Requested_Stop', 'Internal_Error']` | First two: fallback NOT latched. Others: fallback latched. |

**Critical details.**

- The `_FALLBACK_KEYWORDS` matrix is a categorical matrix, not generative — each exit code is a distinct branch. Use `pytest.mark.parametrize` not `hypothesis`.
- For the "real solver failure" tests, the MPC instance must be otherwise functional: same params, same warm-start, same plant. We're isolating the *solver behaviour* by manipulating CasADi's options, not the rest of the loop.
- After a real failure, the next solve with normal params must succeed — verify the warm-start clearing logic at `mpc.py:1228–1234` doesn't permanently kill the controller.
- Check `_consecutive_failures` counter increments and resets correctly across these scenarios.

**Dependencies.** Phase 0 (deferred `@precondition` cleanup) — required so the `SchedulerStateMachine` random walk runs un-gated when this phase's tests cross-reference scheduler behaviour. Plan 1 Phase 8 (CI hypothesis profiles) — useful but not strictly required.

**Exit criteria.** All 6 categories of solver exit confirmed. `_FALLBACK_KEYWORDS` matrix complete. Logbook entry `2026-XX-XX-tier1-real-solver-failures.md` documenting the IPOPT exit codes that ship with the pinned CasADi version (in case future upgrades add new codes).

---

### Phase 2: Tier 1b — Fallback escalation cascade + cold-start IK budget — COMPLETE (2026-05-11)

**Outcome.** Six new tests added to
[tests/sim/test_solver_failures.py](../../tests/sim/test_solver_failures.py)
covering all four W7 walk-forward escalation triggers and the
cold-start IK budget guard:

* **T-U-T1b-1, -2** — pair test on the 20 mm xyz ref-shift threshold
  via direct mutation of `_ref_at_last_success_mid` (25 mm
  Δ → `hold_extrap`; 5 mm Δ → `fallback` with `fallback_step=1`).
* **T-U-T1b-3** — 600 ms wall-clock advance via `time.sleep(0.6)`
  trips the staleness clause; status flips to `hold_extrap`.
* **T-U-T1b-4** — strict-status escalation through
  `max_consecutive_failures=2`: exact sequence
  `[fallback, fallback, hold_extrap, hold_extrap]` confirmed with
  per-tick `fallback_step` assertions.  Audit during implementation
  found the existing
  [test_mpc_static.py::test_escalation_fallback_to_hold](../../tests/sim/test_mpc_static.py)
  drives the *ref-shift* branch (500 mm Δ trips the 20 mm
  threshold), not the counter branch — T-U-T1b-4 is the strict
  counter-isolated test the existing one only purported to be.
* **T-U-T1b-5** — cold-start per-node IK budget exhaustion via
  `_numerical_ik` monkey-patch sleeping 0.42 s per call.  Asserts
  the `logger.info` budget-exceeded record (no `diag` sentinel
  exists — the plan's hedge "or whichever sentinel the code
  surfaces" was the right framing) AND that the cold-start still
  completes with a finite `cmd`.
* **T-U-T1b-6** — Hypothesis `RuleBasedStateMachine` over
  `(succeed_solve, fail_solve, shift_snapshot_z, advance_time)`
  rules with the load-bearing invariant `_prev_w is None or
  np.all(np.isfinite(_prev_w))`.  Holds at ci-deep (1000 examples,
  441.45 s).

Test additions only; **zero production-code changes**.  Pre-Phase-2
1210 + 1 xfailed → post-Phase-2 **1216 + 1 xfailed** in 321.35 s
ci-fast.  Hot-loop allocation contract still green at ci-deep
(3 / 3 in 15.49 s); per Working Note #5 the pre-emptive mitigation
is *deferred to Phase 3* — surfaced as an explicit decision via
`AskUserQuestion`, justified in the [Phase 2 logbook
entry](../../logbook/2026-05-11-tier1b-fallback-escalation-cascade.md)'s
Discussion.

See [logbook entry](../../logbook/2026-05-11-tier1b-fallback-escalation-cascade.md)
for the per-test recipe table, design discussion (snapshot-mutation
vs honest end-to-end driver; pair-test pattern; singleton-MPC
pitfall under hypothesis stateful), and the audit of the existing
escalation test.  Phase 3 (Tier 1c — NaN/Inf input fuzz on
`solve()`) cleared to start.

*Note: the Scope / Test cases / Critical details / Exit criteria
sub-sections below are preserved as the as-planned record; the
Outcome paragraph above is authoritative for what actually shipped.*

**Scope.** Drive the fallback escalation logic at `mpc.py:1574–1592` (walk_forward_unsafe → hold_extrap) and the cold-start IK budget exhaustion path at `mpc.py:1295–1308`.

**New/modified files.**
- `tests/sim/test_solver_failures.py` (extend)

**Test cases.**

| ID | Test | How to drive | Pass criterion |
|----|------|--------------|----------------|
| T-U-T1b-1 | walk_forward_unsafe with ref shift > 20 mm | Force consecutive failures via Phase 1's CPU-time mechanism + change `target_pose` by 30 mm between ticks | `diag['status']` switches from `fallback(...)` to `hold_extrap` once mid-horizon ref shift exceeds 20 mm |
| T-U-T1b-2 | walk_forward_unsafe NOT triggered for ref shift < 20 mm | Force failures + change `target_pose` by 5 mm | `diag['status']` stays `fallback(...)` |
| T-U-T1b-3 | Time-staleness escalation (>500 ms) | Force failures, advance simulated time by 600 ms | `diag['status']` switches to `hold_extrap` per `mpc.py:1590–1592` |
| T-U-T1b-4 | Escalation from `fallback` to `hold` after `max_consecutive_failures` | Force N+1 consecutive failures (already partially covered by `test_mpc_static.py::test_escalation_fallback_to_hold` — verify and extend with edge cases) | After `max_consecutive_failures` hit: `diag['status'] == 'hold'` |
| T-U-T1b-5 | Cold-start IK budget exhaustion | Construct MPC with `max_cpu_time` set such that per-node IK consumes >50% of budget on a cold start; verify linear-interp fallback fires | `diag['cold_start_method'] == 'linear_interp'` (or whichever sentinel the code surfaces); first solve still completes |
| T-U-T1b-6 | Property: walk-forward never produces a command discontinuity > `cmd_step_max_mmps × dt` | Hypothesis fuzz over (target_pose_jump, n_consecutive_failures, t_now_advance) | For all combinations, `‖cmd[t] − cmd[t−1]‖_∞ ≤ rate_limit × dt` |

**Critical details.**

- The walk_forward heuristic at `mpc.py:1574–1592` reads as: "compare current target_pose to the snapshotted mid-horizon ref from the last successful solve; if the L∞ diff exceeds 20 mm OR the snapshot is stale by >500 ms, switch fallback mode to hold_extrap." Tests must drive both branches separately.
- For T-U-T1b-1, the snapshot is taken at success (mpc.py:1146-onwards in the success branch). To exercise the threshold cleanly, run a successful solve first, then introduce failures, then change the target.
- T-U-T1b-5 is hard to drive cleanly because the IK budget guard depends on solver wall-clock. Approach: monkey-patch the IK call to sleep for 60% of `max_cpu_time` per node; verify the budget guard at `mpc.py:1295–1308` triggers before all 6 nodes are processed.

**Dependencies.** Phase 1.

**Exit criteria.** All escalation paths confirmed. Hot-loop allocation contract still green. Logbook entry.

---

### Phase 3: Tier 1c — NaN/Inf input fuzz on `solve()` — COMPLETE (2026-05-11)

**Outcome.** New file
[tests/sim/test_mpc_input_fuzz.py](../../tests/sim/test_mpc_input_fuzz.py)
adds **15 passing tests + 1 strict xfail** covering the entry-point
input surface of ``MPCController.solve()``.  All adversarial values
(NaN, ±Inf, extreme magnitudes, wrong shapes) route to one of five
documented buckets (``Solve_Succeeded`` / ``fallback(...)`` /
``hold(...)`` / ``hold_extrap(...)`` / ``cold_hold(...)``) with the
warm-start
integrity invariant ``_prev_w is None or np.all(np.isfinite(_prev_w))``
preserved on every path.  Hypothesis property test T-U-T1c-7 holds at
ci-deep (1000 examples, 363.68 s).

Three Phase-3 byproducts landed in the same commit:

* **Real production bug surfaced** — T-U-T1c-7 found that NaN in
  ``state.leg_extensions_mm`` combined with a stale W7 snapshot
  propagates through the ``hold_extrap`` arm into ``_prev_u``
  (warm-start corruption).  Per Plan 2's "production-code changes
  triggered by tests" rule the fix lands in its own commit; the
  bug is captured by ``TestT1cLegExtNanCorruptsPrevU`` (xfail
  strict, three-field accounting in this entry's logbook).  The
  T-U-T1c-7 fuzz strategy is restricted to skip ``leg_extensions_mm``
  pending the fix, with an inline comment listing the two
  un-restriction actions to take in the same commit as the fix.
* **``Invalid_Number_Detected`` matrix extension** — Phase 3's
  probes immediately surfaced this as a real IPOPT exit code that
  Phase 1's ``_FALLBACK_KEYWORD_MATRIX`` did not enumerate.  One-
  line addition to
  [tests/sim/test_solver_failures.py](../../tests/sim/test_solver_failures.py)
  with an inline comment naming the discovery context.
* **Working Note #5 mitigation landed** — ``gc.collect()`` at the
  start of both ``test_hot_loop_allocation_contract`` and
  ``test_hot_loop_allocation_contract_hardware``.  The deferral
  from Phase 2 was justified at the time (Phase 2 only added one
  property test); Phase 3 adds three more across two files, so the
  pressure rationale tipped definitively.  Hardware-safe one-line
  change; hot-loop contract still green at ci-deep post-mitigation
  (3 / 3 in 15.87 s).

Test additions only; **zero production-code changes**.  Pre-Phase-3
1216 + 1 xfailed → post-Phase-3 **1232 + 2 xfailed** in 345.85 s
ci-fast.

See [logbook entry](../../logbook/2026-05-11-tier1c-input-fuzz.md)
for the per-test recipe table, the ``platform_twist``-is-dead
finding, the alias-break test infrastructure pitfall, and the full
trace of the ``_prev_u`` corruption bug.  Phase 4 (Tier 2a —
HardwarePlant FK degradation) cleared to start.

**Plan 2 archival gate update.**  After the same-session bugfix
follow-up (see ``2026-05-11-tier1c-input-fuzz-bugfix``), one xfail
on the suite at end of Phase 3:

| Test ID         | Reason                                                          | Target close                          |
|-----------------|-----------------------------------------------------------------|---------------------------------------|
| T-U-T1a-4       | ``Restoration_Failed`` not drivable via ``MPCParams`` in CasADi 3.7.2 | Permanent (structural matrix coverage) |

The original Phase 3 commit (7582764) introduced T-U-T1c-7-bug as a
strict-xfail tracking the ``_prev_u`` corruption surfaced by the
hypothesis property test.  The fix landed in the same session
([logbook entry](../../logbook/2026-05-11-tier1c-input-fuzz-bugfix.md)):
``_handle_failure`` now sanitizes non-finite ``q_cur`` / ``q_dot``
axes at function entry (per-axis substitution from ``_prev_u`` for
``q_cur``, zero for ``q_dot``).  The xfail was removed and the
``T1cWarmStartIntegrityMachine``'s strategy was widened to fuzz the
full 5-field input surface.  Property holds at ci-deep with the
widened strategy.

Same-session bugfix discipline established as a workflow rule in
[CLAUDE.md](../../CLAUDE.md) — the deferral-by-default pattern that
landed T-U-T1c-7-bug as xfail in the first place is the kind of
"latest-acceptable-moment" scheduling the rule explicitly bans.

*Note: the Scope / Test cases / Critical details / Exit criteria
sub-sections below are preserved as the as-planned record; the
Outcome paragraph above is authoritative for what actually shipped.*

**Scope.** Hypothesis-based fuzz on every input to `mpc.solve()`. Verify that adversarial (NaN/Inf/extreme-magnitude) inputs route through `_handle_failure` without corrupting `_prev_w`, `_prev_lam_g`, `_prev_lam_x`.

**New/modified files.**
- `tests/sim/test_mpc_input_fuzz.py` (new)

**Test cases.**

| ID | Test | Strategy | Property |
|----|------|----------|----------|
| T-U-T1c-1 | NaN in `state.platform_pos_mm` | hypothesis: position arrays with `allow_nan=True` | `solve()` either raises or returns fallback; `_prev_w` is the same object as before (or None — never partially-corrupted) |
| T-U-T1c-2 | Inf in `state.platform_twist` | hypothesis: twist arrays with `allow_infinity=True` | Same as 1c-1 |
| T-U-T1c-3 | NaN in `target_pose` | hypothesis | Same |
| T-U-T1c-4 | NaN in `ref_events[i].twist` | hypothesis: build event lists with NaN-spiked twists | Same |
| T-U-T1c-5 | Extreme magnitudes (e.g., `1e10` mm) | hypothesis: position with `min_value=-1e10, max_value=1e10` | Same; warm-start integrity invariant holds |
| T-U-T1c-6 | Wrong-shape inputs | scenario: `np.zeros(5)` instead of `np.zeros(6)` | `solve()` raises (P3 trusted-callee — but at the *entry boundary*, shape is allowed to be checked) |
| T-U-T1c-7 | Property: warm-start integrity across N=100 random adversarial sequences | hypothesis-stateful with rule "fuzz solve()" + invariant "warm-start is well-formed" | After N calls, `_prev_w` is either None or `np.all(np.isfinite(_prev_w))` |

**Critical details.**

- The "warm-start integrity" property is the core safety invariant: a fuzz sequence must never leave the controller in a state where `_prev_w` contains NaN. The success-path NaN check at `mpc.py:1119` already enforces this; the fuzz suite verifies it across the entry-point NaN paths too.
- Hypothesis strategy for `PlantState`: a custom `@composite` strategy that produces a `PlantState` with mostly-valid fields and one field randomly NaN/Inf. This maximises the probability of triggering different downstream branches.
- The `assume()` clause should reject inputs that would be caught at the API boundary (e.g., wrong shape) — those are P3-level errors, not fuzz subjects.

**Dependencies.** Phase 1, Phase 2.

**Exit criteria.** Property tests pass at `max_examples=1000`. Any real bugs surfaced are filed and either fixed (in a separate code-change commit with logbook entry) or have a documented `xfail` with a tracking issue.

---

### Phase 4: Tier 2a — HardwarePlant FK degradation — COMPLETE (2026-05-11)

**Outcome.** New file
[tests/sim/test_hardware_plant_failure_paths.py](../../tests/sim/test_hardware_plant_failure_paths.py)
adds **10 passing tests + 0 new xfails** covering the three watchdog
cascades on ``HardwarePlant.get_state()``:

* **T-U-T2a-1, -2, -3 + boundary regression** — FK divergence cascade
  via ``inject_fk_failure`` (patches
  ``controller.hardware_plant.leg_lengths_to_pose`` at the boundary).
  Confirms ``_FK_FAIL_ESTOP_THRESHOLD = 5`` fires
  ``estop(reason='fk_convergence_failure')`` at exactly count=5
  (boundary regression: count=4 does NOT fire); cache-fallback at
  ``hardware_plant.py:614–617`` returns the last-good pose bit-equal;
  one clean tick resets the counter per ``:593``.
* **T-U-T2a-4, -5** — singular-Jacobian zero-twist via
  ``inject_singular_jacobian`` (patches ``compute_jacobian`` to return
  rank-deficient + forces ``fk_jacobian=None`` so the
  ``compute_jacobian`` branch at ``:721–724`` fires).
  ``np.linalg.solve`` raises ``LinAlgError`` deterministically;
  ``platform_twist`` is zero (unconditional fill at ``:715``); the
  ``Jacobian singular`` warning logs exactly once across 10
  consecutive singular ticks (``_jacobian_singular_warned`` flag at
  ``:740``).
* **T-U-T2a-6a, -6b** — frozen-motor detector via real msgpack frames
  through ``install_telemetry_pump``.  Empirical reading of
  ``hardware_plant.py:193–194`` revealed **two** thresholds, not one:
  ``_FROZEN_MOTOR_POS_WARN = 20`` (warn) and
  ``_FROZEN_MOTOR_POS_ESTOP = 40``
  (``estop(reason='telemetry_frozen')``).  Pair-test pattern (cf.
  Phase 2 T-U-T1b-1/-2) pins both edges.
* **T-U-T2a-7** — bit-different ``motor_pos`` (1 ULP per tick) does
  NOT fire the detector across 44 ticks.  Empirical reading of
  ``hardware_plant.py:657`` revealed the comparison is
  ``np.array_equal`` (bit-exact), **stricter** than the plan's
  "differ by < float epsilon" framing implied — any nonzero
  difference resets the counter, so no false-positive is possible
  from sub-LSB encoder noise.
* **T-U-T2a-8** — Hypothesis ``RuleBasedStateMachine`` over
  (``succeed_tick``, preconditioned ``fail_tick``) with invariant
  ``not _estop_requested``.  Per-tick rule design replaced an
  initial-draft ``fail_burst(n)`` rule that hypothesis correctly
  falsified by *legitimately* summing two bursts past threshold;
  the per-tick + ``@precondition`` pattern expresses the property's
  precondition (consecutive-run length < threshold) directly.
  Companion invariant ``fk_count_matches_shadow`` pins the
  counter-reset semantic.  Holds at ci-deep
  (``--hypothesis-profile=ci-deep --hypothesis-seed=0``, run
  2026-05-11): **1 passed in 616.27 s** (1000 examples).

[tests/sim/_hardware_plant_stub.py](../../tests/sim/_hardware_plant_stub.py)
gained three injection helpers (``inject_fk_failure``,
``inject_singular_jacobian``, ``install_telemetry_pump``) — each
patching at a PRODUCTION-CODE BOUNDARY per Plan 2 Working Note #1
("Drive real failures, not mocked ones"), never at the watchdog
counter.  ``_capture_hp_warnings`` context manager reuses Phase 1's
custom ``logging.Handler`` workaround for pytest ``caplog``
incompatibility on this logger.

Test additions only; **zero production-code changes**.  No xfails
introduced this phase.  Pre-Phase-4: **1233 passed + 1 xfailed**
(``pytest tests/ -q``, run 2026-05-11 against SHA ``e5427e4``,
341.45 s).  Post-Phase-4: **1243 passed + 1 xfailed**
(``pytest tests/ -q``, run 2026-05-11, 382.03 s).  Hot-loop
allocation contract still green at ci-deep
(``pytest tests/sim/test_hot_loop_allocation_contract.py
--hypothesis-profile=ci-deep --hypothesis-seed=0 -q``, run
2026-05-11): **3 passed in 16.59 s.**

**Plan 2 archival gate update.**  No change.  One xfail on the
suite at end of Phase 4 (the inherited Phase 1 T-U-T1a-4
``Restoration_Failed`` permanent xfail):

| Test ID         | Reason                                                          | Target close                          |
|-----------------|-----------------------------------------------------------------|---------------------------------------|
| T-U-T1a-4       | ``Restoration_Failed`` not drivable via ``MPCParams`` in CasADi 3.7.2 | Permanent (structural matrix coverage) |

**Hardware-test target date.**  T-H-T2a-1 (CAN unplug 1–2 s) is
gated on **T-H-T2b-1 PASS** (encoder-publisher kill in isolation —
a Phase 5 deliverable).  Per Plan 2 Working Note #7, the target
window is **within 2 weeks of Phase 5 commit** — concrete deadline,
not "before plan closes".  When Phase 5 commits, that phase's
Outcome paragraph fixes the absolute T-H-T2a-1 deadline.

*Note on Working Note #7 literalism.*  A strict reading of Note #7
("The phase isn't COMPLETE until the hardware test runs and is
logged") would forbid the COMPLETE marking above until T-H-T2a-1
runs.  T-H-T2a-1 cannot run before T-H-T2b-1 PASS, which is a
Phase 5 deliverable — so the literal reading inverts the gating.
Phase 4's unit-test deliverables ARE all delivered (the ten tests
above; zero new xfails); the hardware verification is correctly
deferred to a downstream phase's prerequisite.  This Outcome
paragraph carries the binding commitment to the 2-week window;
Phase 5's Outcome paragraph will fix the absolute T-H-T2a-1
deadline at its commit time.  The intent of Note #7 — concrete
windows, not "before plan closes" framing — is honoured; only the
phase-locality of the close-out claim is relaxed.

See [logbook entry](../../logbook/2026-05-11-tier2a-hardware-plant-fk-degradation.md)
for the per-test empirical-probe recipe table, the design discussion
(boundary-patch vs counter-poking; per-tick rules over per-burst
under hypothesis; bit-exact frozen-motor compare; caplog workaround;
e-stop spy pattern), and the line-citation refresh against
``hardware_plant.py``.  Phase 5 (Tier 2b — HardwarePlant telemetry
& FF) cleared to start.

*Note: the Scope / Test cases / Critical details / Exit criteria
sub-sections below are preserved as the as-planned record; the
Outcome paragraph above is authoritative for what actually shipped.
Plan-side line citations (e.g. ``hardware_plant.py:195``) all drifted
between plan-writing and Phase 4 implementation; the Outcome's
post-Phase-4 citations are ground truth.*

**Scope.** Drive the FK divergence watchdog, singular-Jacobian zero-twist branch, and frozen-motor detector. These are documented at:

- FK divergence + watchdog: `hardware_plant.py:195` (`FK_FAIL_ESTOP_THRESHOLD=5`), `:554–575` (cache fallback)
- Singular Jacobian: `hardware_plant.py:715–718`
- Frozen-motor: `hardware_plant.py:163–172`

**New/modified files.**
- `tests/sim/test_hardware_plant_failure_paths.py` (new)
- `tests/sim/_hardware_plant_stub.py` (extend to support FK injection)

**Test cases.**

| ID | Test | How to drive | Pass criterion |
|----|------|--------------|----------------|
| T-U-T2a-1 | FK convergence failure on a single tick | Patch FK to return non-converged result on one call | `get_state()` returns last-good pose; `_fk_fail_count == 1` |
| T-U-T2a-2 | FK divergence watchdog fires at threshold | Patch FK to return non-converged for `FK_FAIL_ESTOP_THRESHOLD` consecutive ticks | E-stop triggered (verify via mock or by spying on `estop()` call) |
| T-U-T2a-3 | FK recovers within threshold | Fail 4 consecutive times then succeed | E-stop NOT triggered; `_fk_fail_count` resets |
| T-U-T2a-4 | Singular Jacobian → zero twist | Construct a leg-extension state that produces a near-singular Jacobian (e.g., legs at the workspace boundary) | `state.platform_twist == 0`; warning logged once |
| T-U-T2a-5 | Singular Jacobian warning is once-only | Trigger the singularity for 10 consecutive ticks | Warning logged once; subsequent ticks silent |
| T-U-T2a-6 | Frozen-motor detector | Feed identical motor positions for `_FROZEN_MOTOR_THRESHOLD` ticks | Detector fires; documented action taken (warning, E-stop, etc. — verify against current code) |
| T-U-T2a-7 | Frozen-motor false-positive avoidance | Feed positions that differ by `< float epsilon` (legitimately stalled at a setpoint) | Detector does NOT fire (verify the threshold |
| T-U-T2a-8 | Property: any sequence of FK failures of length < threshold does not trigger E-stop | hypothesis stateful | Invariant holds |

**Critical details.**

- Use `_HardwarePlantStub` from Plan 1 Phase 5 as the base. Extend with:
  - `inject_fk_failure(n_ticks: int)` — next N FK calls return non-converged
  - `inject_singular_jacobian(n_ticks: int)` — next N Jacobian computations are near-singular
  - `inject_frozen_motors(n_ticks: int)` — next N motor positions are identical
- The "near-singular Jacobian" leg state must be constructed deliberately. Look for the workspace boundary states; these are documented in `motion/kinematics.py` somewhere. If not, find a pose where the Jacobian's smallest singular value drops below `1e-3 × largest`.
- T-U-T2a-2 and T-U-T2a-6 are particularly safety-critical — these are the paths that protect the platform from oblivious-MPC scenarios. Direct E-stop-trigger verification is a hardware test (T-H-T2a-1, see Testing Plan).

**Dependencies.** Plan 1 Phase 5 (HardwarePlantStub exists).

**Exit criteria.** All FK / Jacobian / frozen-motor failure paths exercised. Logbook entry. **No production code changes** unless a test surfaces a real bug — those changes land in their own commits.

---

### Phase 5: Tier 2b — HardwarePlant telemetry & FF — NOT STARTED

**Scope.** Drive the telemetry-staleness threshold matrix (warn / hard / estop), the `set_pose()` torque-FF singular path, and the cold-start zero-state path.

**New/modified files.**
- `tests/sim/test_hardware_plant_failure_paths.py` (extend)

**Test cases.**

| ID | Test | How to drive | Pass criterion |
|----|------|--------------|----------------|
| T-U-T2b-1 | Telemetry stale at WARN threshold (3× control_dt) | Patch ZMQ recv timestamp to be `3.5 × control_dt` old | Warning logged once (edge-triggered); velocities NOT zeroed; E-stop NOT fired |
| T-U-T2b-2 | Telemetry stale at HARD threshold (5× control_dt) | Patch timestamp to `5.5 × control_dt` old | Velocities zeroed; warning logged; E-stop NOT fired |
| T-U-T2b-3 | Telemetry stale at ESTOP threshold (20× control_dt) | Patch timestamp to `20.5 × control_dt` old | E-stop fired with reason `'telemetry_stale'`; velocities zeroed |
| T-U-T2b-4 | Stale-then-recovered | Sequence: stale at WARN → fresh again | Warning fired once; recovery does not re-warn until next stale event |
| T-U-T2b-5 | Property: thresholds scale linearly with control_dt (P4 from Plan 1) | hypothesis: `control_dt ∈ [0.01, 0.1]` | All three thresholds scale as documented multipliers |
| T-U-T2b-6 | `set_pose()` torque-FF singular Jacobian | Pass a pose at the singular workspace boundary | Per documented behaviour at `hardware_plant.py:782–797` — silent zero (verify) or raise (check current code) |
| T-U-T2b-7 | Cold-start zero-state | Construct fresh `HardwarePlant` and call `get_state()` before any telemetry has arrived | Returns the zero-initialised state at `:507–513`; `data_age_s` reflects the un-init state |
| T-U-T2b-8 | Cold-start with `enable()` then no recv for 1 s | Fresh plant, `enable()`, advance time 1 s without recv | E-stop fires (telemetry stale) |

**Critical details.**

- The threshold tests are categorical (3 thresholds × 2 sides each = 6 boundary cases). Use parameterized scenarios, not hypothesis.
- T-U-T2b-6 might surface a real semantic question: should `set_pose()` silently zero on singular FF, or should it raise? Document the current behaviour; if it's silent zero with a once-only warning, that's the contract; if it's silent without warning, that's a bug worth filing. Either way the test asserts the *current* documented behaviour.
- For T-U-T2b-5 to work, Plan 1 Phase 6 (P4) must be landed — `control_dt` must be a constructor parameter on `HardwarePlant`.

**Dependencies.** Phase 4. Plan 1 Phase 6 (P4 implemented).

**Exit criteria.** Telemetry threshold matrix complete. `set_pose()` FF behaviour documented. Cold-start path covered. Logbook entry.

---

### Phase 6: Tier 2c — ZMQ corruption (real-msgpack harness) — NOT STARTED

**Scope.** Replace `FakeIPC` with a real-ZMQ + real-msgpack harness for the corruption tests. Cover partial frames, version-skew (added/removed fields), and connection-drop simulation.

**New/modified files.**
- `tests/sim/_zmq_test_harness.py` (new) — wraps `MpcTargetIPC` with byte-level injection
- `tests/sim/test_zmq_corruption.py` (new)

**Test cases.**

| ID | Test | How to drive | Pass criterion |
|----|------|--------------|----------------|
| T-U-T2c-1 | Partial msgpack frame (truncated bytes) | Send first 50% of a valid frame's bytes | Recv side drops the frame; logs warning; subsequent frames recv cleanly |
| T-U-T2c-2 | Malformed msgpack (corrupt mid-frame) | Send a valid frame with one byte flipped | Recv side raises `msgpack.UnpackException` (or equivalent); handled gracefully |
| T-U-T2c-3 | Version-skew: extra field on send | Send a frame with one extra key the consumer doesn't expect | Consumer ignores the extra field (forward-compat) OR fails loudly (back-compat) — assert documented behaviour |
| T-U-T2c-4 | Version-skew: missing field | Send a frame without a key the consumer expects | Consumer raises with a clear error message naming the missing field |
| T-U-T2c-5 | Connection-drop mid-recv | Establish connection, send a frame, kill the publisher socket between frames | Recv side detects via timeout; data_age_s grows; eventually telemetry-stale watchdog fires |
| T-U-T2c-6 | Property: random byte corruption never produces silent field corruption | hypothesis: byte-level mutations | Either valid msgpack with field values matching the strategy, or unpack failure — never silent partial-deserialise |

**Critical details.**

- `_zmq_test_harness.py` exposes a `corrupt_send(frame_bytes, mutation_fn)` that takes a callable to mutate bytes before send. Standard mutations: truncate, flip, swap, prepend-garbage, append-garbage.
- The version-skew tests must use the `MpcTargetIPC` and `MotionIPC` actual schema definitions, not invented ones. If those schemas are not currently defined in code (e.g., implicit dict keys), this work surfaces a need to formalise them — file as a Tier 3 follow-up.
- The connection-drop test uses `socket.close()` on the publisher side; the recv side detects via heartbeat timeout. Verify the timeout value matches what production expects.

**Dependencies.** None within this plan; Phase 5's telemetry-stale infrastructure overlaps.

**Exit criteria.** All corruption modes covered. ZMQ infrastructure resilience confirmed. Logbook entry.

---

### Phase 7: Tier 3a — Numerical + schema fuzz — NOT STARTED

**Scope.** Two distinct fuzz surfaces: (a) numerical edge cases on every public API in the MPC layer, (b) schema-completeness on the `diag` and `extras` namespaces across every fallback branch.

**New/modified files.**
- `tests/sim/test_mpc_input_fuzz.py` (extend with non-`solve()` API surfaces)
- `tests/sim/test_diag_schema_fuzz.py` (new)

**Test cases — numerical fuzz (Tier 3a-1).**

| ID | Test | Surface | Property |
|----|------|---------|----------|
| T-U-T3a-N1 | `feasibility.segment_is_feasible` NaN/Inf inputs | `feasibility.py` | Returns `(False, inf, inf)` or raises; never returns `(True, ...)` for NaN inputs |
| T-U-T3a-N2 | `feasibility.quintic_peak_vel_per_axis` zero-duration | `feasibility.py` | Documented: today no `T<=0` guard. This test surfaces the gap; either add guard or assert the current behaviour. |
| T-U-T3a-N3 | `hermite.quintic_interp_with_accel` invalid accel boundary | `hermite.py` | Output finite for finite inputs; documented behaviour for infinite inputs |
| T-U-T3a-N4 | `target.flat_target_to_events` degenerate `arrival_time` (`< t_now + 50ms`) | `target.py` | Returns single hold event per `:204–209`; verified |
| T-U-T3a-N5 | `target.make_feasible_events` proposal with NaN pose | `target.py` | Either raises with clear error or routes through K1 anchor cleanly |
| T-U-T3a-N6 | `mpc._numerical_ik` zero-rotation edge case | `mpc.py:1337` | Returns identity rotation per documented fallback |
| T-U-T3a-N7 | `runner.run_mpc_loop` with `n_steps=0` | `runner.py` | Either no-op return or raises; documented |

**Test cases — schema fuzz (Tier 3a-2).**

| ID | Test | Surface | Property |
|----|------|---------|----------|
| T-U-T3a-S1 | `diag` schema completeness — Solve_Succeeded | `mpc.py:1207–1215` | All 7 keys populated with non-default values |
| T-U-T3a-S2 | `diag` schema completeness — fallback | After forced fallback (Phase 1 mechanism) | All 7 keys + `fallback_step` populated |
| T-U-T3a-S3 | `diag` schema completeness — hold_extrap | After forced escalation (Phase 2) | All 7 keys + escalation-specific keys populated |
| T-U-T3a-S4 | `diag` schema completeness — hold | After max-consecutive-failures (Phase 2) | All 7 keys populated |
| T-U-T3a-S5 | `diag` schema completeness — non_finite_solution | Force NaN solver output (Phase 3) | All keys populated; `status == 'non_finite_solution'` |
| T-U-T3a-S6 | `diag` schema completeness — exception path | Force CasADi exception (Phase 1 mechanism) | All keys populated; `status` starts with `'exception:'` |
| T-U-T3a-S7 | Property: every solve() call leaves `diag` populated with the canonical 7 keys | hypothesis stateful (rule: fuzz solve, invariant: diag has all keys) | Invariant holds across N random sequences |
| T-U-T3a-S8 | Property: `extras` namespace populated identically across solve paths | hypothesis stateful | `set(vars(extras))` is invariant across success/fallback/hold |

**Critical details.**

- Schema-completeness is the contract `record_from_arrays` and `log_mpc_step` rely on. A fallback path that forgets a key produces `KeyError` mid-tick. The schema is implicit today — Phase 7 makes it explicit by testing it.
- The schema-fuzz tests parameterize over the *status* axis: every documented status string in the codebase becomes a test case. Use `grep "diag\['status'\] = " controller/mpc.py` to enumerate.
- If a schema gap is found (e.g., `hold_extrap` doesn't populate `cmd_next_mm`), file it as a Tier-3 follow-up; the test asserts the *current* behaviour with a clear `xfail` marker if the gap is intentional, or is fixed in a follow-up commit.

**Dependencies.** Phases 1, 2, 3 (need their failure-driving mechanisms).

**Exit criteria.** Numerical fuzz passes at `max_examples=1000`. All schema branches confirmed. Logbook entry. Any schema gaps surfaced are filed as separate work items.

---

### Phase 8: Tier 3b — Time pathologies, resource exhaustion, hooks, races — NOT STARTED

**Scope.** Final cleanup of remaining Tier-3 categories.

**New/modified files.**
- `tests/sim/test_mpc_time_pathologies.py` (new)

**Test cases — time pathologies.**

| ID | Test | Surface | Pass criterion |
|----|------|---------|----------------|
| T-U-T3b-T1 | Backward `sim_time` to `solve()` | `mpc.solve(t_now=...)` | Documented behaviour: solver runs at given t (no validation in `solve()` itself); but scheduler-driven sources (Plan 1 S6) raise. Test confirms separation. |
| T-U-T3b-T2 | Dropped tick (sim_time advances by 2× control_dt instead of 1×) | `runner.run_mpc_loop` | Loop continues; warm-start shift accommodates the gap |
| T-U-T3b-T3 | dt schedule change mid-run | Construct MPC with `dt_schedule_a`, run 10 ticks, swap to `dt_schedule_b` | Warm-start invalidation logic kicks in; first tick after swap is cold-start |
| T-U-T3b-T4 | Property: random `(sim_time, dt)` sequences never produce `_prev_w` corruption | hypothesis stateful | Invariant holds |

**Test cases — resource exhaustion.**

| ID | Test | Surface | Pass criterion |
|----|------|---------|----------------|
| T-U-T3b-R1 | Telemetry pool overflow | Configure `TelemetryLogger` with `pool_size=10`, run 100 ticks | Pool grows or wraps per documented policy at `telemetry.py:260–270` |
| T-U-T3b-R2 | Event-queue overflow at S3 boundary | Submit 4 events back-to-back (Plan 1 Phase 2 catches; verify) | Plan 1 S3 raises |
| T-U-T3b-R3 | File I/O failure on `write_csv` | Patch `open()` to raise `PermissionError` | Documented behaviour: IOError propagates |

**Test cases — hooks & races.**

| ID | Test | Surface | Pass criterion |
|----|------|---------|----------------|
| T-U-T3b-H1 | `on_target_override` returning None | `runner.py:161` | Documented: silently uses None (verify and surface gap if any) |
| T-U-T3b-H2 | `on_target_override` raising | `runner.py` | Exception propagates; loop terminates cleanly with clear logging |
| T-U-T3b-H3 | `on_pre_command` raising | `runner.py` | Same |
| T-U-T3b-H4 | `on_post_solve` allocating in hot loop | `runner.py` | Hot-loop allocation contract still green (regression check) |
| T-U-T3b-R4 | `ZmqTargetSource.reset()` mid-APPROACHING | `zmq_target.py:380–390` | Documented behaviour; no warm-start corruption |
| T-U-T3b-R5 | Concurrent state read/write (if any threading) | thread-safety audit | No race-condition-shaped failures (likely no-op for this codebase since hot loop is single-threaded) |

**Critical details.**

- The hook tests use a real (not mocked) `runner.run_mpc_loop` configured with deliberately-broken hooks. The point is to verify the loop's resilience, not to test the hook itself.
- T-U-T3b-R4 may surface a real bug — if the source `reset()` clears cached events while the MPC is mid-tick, `_prev_w` could be invalidated incorrectly. Test for `np.all(np.isfinite(_prev_w))` after reset; if it fails, file as a follow-up.
- The codebase is largely single-threaded on the MPC hot loop; T-U-T3b-R5 may legitimately be a no-op. Document the audit even if no tests are added.

**Dependencies.** Plan 1 Phase 3 (S6). Phase 7.

**Exit criteria.** All Tier-3 categories covered or documented as out-of-scope. Logbook entry. Plan can be archived via `/archive-plan mpc-sadpath-coverage-tiers-1-3`.

## Testing Plan

### Unit tests (offline, no hardware)

All tests in this plan are unit tests by design (test additions, not production-code changes — except Phase 0; see Notes).

| Phase | New test file | Test count (approx) | Strategy |
|-------|---------------|---------------------|----------|
| 0 | `test_scheduler_contract.py` (modify) + `scheduler.py` production fix | ~6 | Scenario tests for both bugs + un-gate state machine |
| 1 | `test_solver_failures.py` | ~15 | Scenarios + parameterized `_FALLBACK_KEYWORDS` matrix |
| 2 | `test_solver_failures.py` (extend) | ~10 | Scenarios + 1 hypothesis property |
| 3 | `test_mpc_input_fuzz.py` | ~10 | Hypothesis (composite strategies + stateful) |
| 4 | `test_hardware_plant_failure_paths.py` | ~12 | Scenarios + 1 hypothesis stateful |
| 5 | `test_hardware_plant_failure_paths.py` (extend) | ~8 | Parameterized scenarios + 1 hypothesis property |
| 6 | `test_zmq_corruption.py` + `_zmq_test_harness.py` | ~8 | Scenarios + 1 hypothesis byte-corruption property |
| 7 | `test_mpc_input_fuzz.py` (extend) + `test_diag_schema_fuzz.py` | ~16 | Mix |
| 8 | `test_mpc_time_pathologies.py` | ~14 | Mix |

Total: ~99 new tests (Phase 0's six + Phases 1–8's ~93).

### Integration tests

Limited to "the test suite still works" — full `pytest tests/ -q` after each phase.

| ID | Test | Pass criterion |
|----|------|----------------|
| T-I-1 | Full `pytest tests/ -q` after each phase | 100% pass |
| T-I-2 | `tests/sim/test_hot_loop_allocation_contract.py` after each phase | `< THRESHOLD_BYTES = 256` (no regression from new test infrastructure) |
| T-I-3 | `pytest tests/ -q --hypothesis-profile=ci-deep` after Phase 8 (final) | 100% pass at `max_examples=1000` |

### Hardware tests

**The original test framing was scope-mismatched** — CAN unplug doesn't
drive FK divergence (a numerical convergence failure), it drives
telemetry-stale (no leg-length data arriving at all). The corrected
design separates the two concerns and re-orders them by safety.

| ID | Test | Validates | Pass criterion | Hazard profile |
|----|------|-----------|----------------|----------------|
| T-H-T2b-1 | After Phase 5 — encoder-publisher kill on the Jetson (`SIGSTOP` for ~1 s) | Jetson-side telemetry-stale watchdog fires E-stop while CAN bus stays live | E-stop fires within 500 ms ± 25 ms of publisher pause; ODrives stay armed throughout (CAN traffic uninterrupted from `motor_guard`); platform never freewheels | **Low.** ODrives keep receiving setpoints; platform stays held. The only thing affected is the Jetson-side observation of `/robot_state`. |
| T-H-T2a-1 | After Phase 4 + T-H-T2b-1 PASS — CAN unplug for 1–2 s (operator's minimum guaranteed duration) | The cascaded safety chain on CAN loss: ODrive CAN heartbeat watchdog → ODrive disarm → Jetson observes telemetry stale → motor_guard fires E-stop → clean exit | Each link in the chain measurable; total time from unplug to E-stop within documented bounds; platform freewheels under gravity from the moment ODrives disarm; lands within mechanical workspace | **Medium-high.** Platform freewheels during the watchdog window. Mitigated by holding at low workspace (small drop) + operator E-stop on physical button. Run only after T-H-T2b-1 has demonstrated the watchdog mechanism works. |
| T-H-T2a-2 | (DEFERRED — see Notes) — actual FK divergence on real hardware via deliberate near-singular pose or encoder noise injection | Numerical FK convergence failure path (distinct from CAN/telemetry loss) | FK divergence watchdog fires; E-stop reason includes `fk_convergence_failure` (not `telemetry_stale`) — see `controller/hardware_plant.py:613` | Mechanism design needed before this test can be specified. Phase 4's unit-test coverage with `HardwarePlantStub` injection is the primary verification; hardware verification of FK-divergence-specifically requires a separate driver. **Out of scope for Plan 2 unless designed.** |

These hardware tests are NOT part of the regular test suite — they are manual bringup tests run after the corresponding phase lands. Each gets a logbook investigation entry under `/investigate`.

**Pre-test setup checklist (apply to both T-H-T2b-1 and T-H-T2a-1):**

1. Platform commanded to a **low safe pose** — close to the lower workspace bound so any freewheeling drop is mechanically bounded. Not the standard `0,0,170` mid-workspace pose.
2. Hand detached. No ball. Platform-only mode.
3. Operator hand on the physical E-stop button throughout the test.
4. Second person present in the workshop.
5. Test instrumented to log the timing chain (fault-injection time, ODrive disarm time, telemetry-stale fire time, motor_guard E-stop fire time). The test's real value is the timing breakdown, not a pass/fail boolean.
6. **Run T-H-T2b-1 first.** If the publisher-kill test fails to fire the watchdog within the threshold, fix that before doing the more severe CAN-unplug test. T-H-T2b-1 validates the Jetson-side mechanism in isolation; T-H-T2a-1 validates the cascaded chain.
7. Hardware exit-criterion checklist date set when the corresponding phase commits — not "before Plan 2 closes". Plan 1 nearly forgot Phase 6's smoke; this plan has TWO of them.

### Regression tests

| ID | Test | Pass criterion |
|----|------|----------------|
| T-R-1 | All existing tests pass after every phase | 100% pass |
| T-R-2 | `test_hot_loop_allocation_contract.py` budget unchanged | `< 256 B/tick` |
| T-R-3 | `test_make_feasible_events.py` properties (Plan 1 Phase 7 retroactive expansion) still pass | 100% at `max_examples=1000` |

### Property-test depth schedule

Same as Plan 1: per-PR `ci-fast` (50 examples), nightly `ci-deep` (1000 examples). Property tests added in this plan respect those profiles.

## Notes for Collaborators

### Working notes — lessons from Plan 1

These notes capture lessons that apply *specifically to Plan 2*. Read
this section before starting any phase.  Cross-plan process patterns
(audit gate, SHA backfill, etc.) live in `~/.claude/.../memory/` and
are loaded automatically when relevant.

**1. Drive real failures, not mocked ones — honour the plan's discipline.**

The plan's framing — "Drives real solver failure paths, not synthetic
ones" — is its most important self-imposed constraint.  Two specific
places it will be tempting to cheat:

- **Phase 1's `max_cpu_time=1e-6`** may cause CasADi to raise an
  internal error before IPOPT even initialises, not return
  `Maximum_CpuTime_Exceeded`. Find the value where IPOPT *genuinely*
  times out (probably 1–2 ms for this MPC, not 1 µs). If a documented
  exit code can't be driven cleanly with parameter tuning, document
  why and fall back to a documented-second-best mechanism (e.g.,
  monkey-patch the solver's CPU clock) — but never paper over with
  "test passes if we mock the return value".
- **Phase 4's FK divergence tests** must drive through `get_state()`
  with simulated bad telemetry, not poke `_fk_fail_count` directly.
  A test that pokes a private counter doesn't validate the watchdog —
  it validates that the counter increments. The right surface is the
  public interface; mocks belong at the *boundary* (the FK function
  itself), not at the consumer (the watchdog).

**2. xfail discipline — every xfail needs three fields and an exit plan.**

The plan permits surfacing real bugs as xfail markers with a tracking
reference (see "Production-code changes triggered by tests" below).
Without discipline, xfails accumulate and become invisible. **Every
xfail must carry three fields, in the phase's logbook entry**:

(a) test ID, (b) tracking reference (issue or follow-up logbook entry),
(c) target close phase or date.

**Plan 2 archival gate**: zero unfixed xfails at archival, OR each
residual xfail has a documented justification for why it's permanently
acceptable. Plan 1 had no such gate (because it didn't use xfails);
Plan 2 needs one because the xfail mechanism is a first-class part of
its workflow.

**3. Convert line citations to symbol references during implementation.**

Plan 1 Phase 7's docstring refresh (commit `3c04bbe`) demonstrated the
value: `test_property_K1_K6_two_event_proposal` doesn't drift; line
167 does. Plan 2's phase descriptions cite many specific lines in
`mpc.py` (e.g., `:1228–1234`, `:1295–1308`, `:1574–1592`). When
implementing, refresh those to symbol references in test docstrings,
even if the original plan used line numbers. The line numbers will
drift the moment anyone touches `_handle_failure`.

**4. Real-ZMQ harness in Phase 6 — the highest test-infra risk.**

ZMQ tests are notoriously flaky: port collisions, lingering sockets
after pytest interruption, timing-sensitive recv timeouts.
`_zmq_test_harness.py` needs strict `setUp` / `tearDown` with explicit
`socket.close()` + `context.term()` even on test failure (use pytest
fixtures with `yield` and a cleanup block that runs on raise). Build
a minimal prototype of the harness pattern *before* writing any
corruption test. If the harness pattern looks fragile, mark this
phase higher-risk and consider running its tests in a separate
pytest invocation.

**5. Hot-loop allocation contract will flake more under Plan 2.**

Plan 2 adds substantial hypothesis fuzz (Phase 3 — Tier 1c NaN/Inf
fuzz with stateful warm-start; Phase 7 — Tier 3a schema fuzz).
[Phase 8's logbook](../../logbook/2026-05-10-mpc-tier0-phase-8-ci-hypothesis-profiles.md)
already documented the heap-state flake on
`test_hot_loop_allocation_contract` at ci-deep. Pre-empt: either add
`gc.collect()` at the start of that test, or mark it `slow` and run
it in a separate pytest invocation. **Don't wait for the second
flake** — bake a mitigation in before Phase 3 lands.

**6. Schema completeness as a contract, not just a test.**

Phase 7's `diag` / `extras` schema-fuzz work is structurally similar
to Plan 1's contract work — the schema is currently *implicit*.  If
the test surfaces a gap (e.g., `hold_extrap` doesn't populate
`cmd_next_mm`), the right fix is often to write a small
`DIAG_SCHEMA_CONTRACT.md` (matching Plan 1's K1–K6 / S1–S6 / P1–P4
pattern), not to add yet another keyword check. Resist leaving the
schema implicit; the explicit contract is the higher-leverage outcome.

**7. Hardware tests have target dates, not "before plan closes".**

Plan 1 nearly forgot Phase 6's hardware bringup smoke — the user
caught it post-archival, requiring a follow-up commit (`400418b`) to
backfill the result into the logbook. Plan 2 has TWO in-scope hardware tests
(T-H-T2b-1 and T-H-T2a-1; T-H-T2a-2 is deferred — see Testing Plan
above for hazard profiles).
**Each phase that depends on a hardware test sets the target date for
that test in the phase's exit criteria, when the phase is committed.**
Not "user responsibility before plan closes". The phase isn't
COMPLETE until the hardware test runs and is logged.

**8. Test count is not a quality metric.**

The plan estimates ~93 new tests across 8 phases. Some will be
genuinely high-leverage (driving real IPOPT exit codes; warm-start
integrity property). Some will be lower-value (re-asserting the same
invariant from a slightly different angle). Resist completionism. If a
test category is "I've covered it with N=3, do I need N=8?", the
answer is usually no. Quality > count, and the plan's exit criteria
are about coverage of the *failure modes*, not the *test count*.

**9. Hardware risk model is concentrated in two phases.**

Phases 1–3, 6, 7, 8 are pure software (unit tests with stubs / fakes /
real-but-in-process ZMQ). Zero hardware exposure. **Phases 4 and 5**
have hardware bringup tests with non-trivial fault injection — read
the Testing Plan's pre-test setup checklist before either lands. The
CAN-unplug test (T-H-T2a-1) freewheels the platform under gravity for
the duration of the watchdog window; it's strictly worse than the
publisher-kill test (T-H-T2b-1) and should only run after the latter
has demonstrated the watchdog works.

### Production-code changes triggered by tests

*Phase 0 is the documented exception to the rules below.* The two
scheduler bugs Phase 0 fixes are inherited from Plan 1's deferred work,
not surfaced by Plan 2's own tests. Removing their `@precondition`
gates *requires* fixing the underlying bugs in the same commit (the
gates can't be removed without the fix). Every later phase (1–8)
follows the file-separately-and-xfail rule below.

This plan is test additions only by design (with the Phase 0 carve-out
above). **However**, tests in Phases 1–8 may surface real bugs (a
documented failure path that doesn't actually behave as documented).
When that happens:

1. **Don't fix the bug in this plan's commits.** File it as a separate work item.
2. **Add the test with `xfail` and a tracking reference.** Example: `@pytest.mark.xfail(reason='Surfaces hardware_plant.py:797 silent zero on singular FF; tracked in <issue>', strict=True)`.
3. **The bug fix lands in its own commit** with its own logbook entry. The test is un-xfailed in the same commit as the fix.

This separation keeps test-coverage commits cleanly diff-able from behaviour-change commits, and preserves the rollback granularity established in Plan 1.

### Safety-critical invariants (preserve)

| Invariant | Location | Consequence of violation |
|-----------|----------|-------------------------|
| Warm-start integrity (`_prev_w` finite or None) | `controller/mpc.py:1119, 1228–1234` | Solver corruption cascade |
| Hot-loop allocation budget | `controller/hot_loop_contract.py` | GC pause → cmd discontinuity |
| K1–K6 reference feasibility | `controller/target.py:make_feasible_events` | Solver saturation |
| Scheduler S1–S6 (Plan 1) | `controller/scheduler.py` | Past-time events / duplicate IDs / state-machine violation |
| PlantInterface P1–P4 (Plan 1) | `controller/plant.py` | Aliasing breakage / silent reset / threshold-scaling drift |
| Telemetry-staleness watchdog | `controller/hardware_plant.py:67–71, 605–671` | Stale telemetry → MPC commands on stale pose |

### Architecture decisions (non-obvious)

1. **Why drop `FakeIPC` for corruption tests but keep it elsewhere.** `FakeIPC` is faster and cleaner for happy-path tests. For *corruption* tests, the value is in driving real bytes through real serializers; `FakeIPC` would only test the test-double's own corruption logic. The tradeoff is some test setup complexity for the corruption tests.
2. **Why the `_FALLBACK_KEYWORDS` matrix is parameterized, not generative.** IPOPT exit codes are categorical: there's a finite documented list. Generation gives no extra coverage and makes regressions harder to read.
3. **Why hypothesis-stateful for warm-start integrity.** The invariant is across-call (a sequence of fuzz calls must not corrupt state). Stateful is the right tool. Pure `@given` would only test one call at a time.
4. **Why this plan is test-only.** Mixing test additions with production-code changes in the same commits would make the work harder to review and partially revert. Bugs surfaced get tracked as separate work items per the "Production-code changes triggered by tests" rule above.

### Startup / shutdown ordering

No changes to startup ordering. This plan is test additions.

### Files affected

| Path | Change | Phase |
|------|--------|-------|
| `controller/scheduler.py` | Modified (cancel_next mid-TRANSITIONING fix; begin_return S3 fix) | 0 |
| `tests/sim/test_scheduler_contract.py` | Modified (remove `@precondition` gates; add T0 scenarios) | 0 |
| `tests/sim/test_scheduler.py` | Modified (adjust any tests affected by the new fault behaviour) | 0 |
| `tests/sim/test_solver_failures.py` | Created | 1, 2 |
| `tests/sim/test_mpc_input_fuzz.py` | Created | 3, 7 |
| `tests/sim/test_hardware_plant_failure_paths.py` | Created | 4, 5 |
| `tests/sim/test_zmq_corruption.py` | Created | 6 |
| `tests/sim/_zmq_test_harness.py` | Created | 6 |
| `tests/sim/_hardware_plant_stub.py` | Modified (extend with FK injection) | 4 |
| `tests/sim/test_diag_schema_fuzz.py` | Created | 7 |
| `tests/sim/test_mpc_time_pathologies.py` | Created | 8 |
| `logbook/2026-XX-XX-...` | Created (one entry per phase, two for Phase 0) | 0, 1, 2, 3, 4, 5, 6, 7, 8 |

### Rollback plan

Each phase is one or more commits on the `refactor` branch. Rollback per phase is `git revert <sha>`. Because the plan is test-only, rollback never affects production behaviour — it only removes the tests.

If a test surfaces a real bug AND the bug fix has been committed (per "Production-code changes triggered by tests"), the bug fix can be reverted independently of the test. The test will fail post-revert; that's the point — the failure is the signal that the regression has been re-introduced.

If the entire plan needs to revert (e.g., a fundamental design issue with the hypothesis profiles): revert the test files; the codebase returns to status quo ante.

### Sequencing across the two plans

```
Plan 1 (Tier 0)                                 Plan 2 (Tiers 1–3)
─────────────────                                ──────────────────
Phase 1: Scheduler contract draft
Phase 2: S1–S3 enforcement                ┐
Phase 3: S4–S6 enforcement                ├─── unblock Plan 2 Phase 1+
Phase 4: PlantInterface contract draft    │
Phase 5: P1–P2 enforcement                ┴─── unblock Plan 2 Phase 4+
Phase 6: P3–P4 enforcement                ──── unblock Plan 2 Phase 5
Phase 7: K1–K6 hypothesis expansion       ──── orthogonal (no Plan 2 dependency)
Phase 8: CI hypothesis profiles           ──── unblock Plan 2 nightly CI

                                                 Phase 0: Close Plan 1 @precondition gates
                                                 Phase 1: T1a real IPOPT failures
                                                 Phase 2: T1b escalation cascade
                                                 Phase 3: T1c NaN/Inf input fuzz
                                                 Phase 4: T2a HardwarePlant FK degradation
                                                 Phase 5: T2b telemetry & FF
                                                 Phase 6: T2c ZMQ corruption
                                                 Phase 7: T3a numerical + schema fuzz
                                                 Phase 8: T3b time / resources / hooks / races
```

Plan 2 begins with Phase 0 (Plan 1 cleanup of two deferred `@precondition` gates). Phase 1 starts after Phase 0 lands; all of Plan 1 Phase 3's prerequisites (scheduler contract enforcement) are already in place. Plan 2 Phases 4–5 require Plan 1 Phase 6 (P4 wired) — already landed. Plan 2 Phase 8 requires Plan 1 Phase 3 (S6 raises on backward time) — already landed. So Plan 2's external dependencies are all green; only Phase 0 (internal cleanup) gates Phase 1.
