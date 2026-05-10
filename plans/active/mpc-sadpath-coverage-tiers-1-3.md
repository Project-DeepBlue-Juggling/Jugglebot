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

**Concurrent with** the in-flight `refactor` branch. Test additions only (no production-code changes except where a test surfaces a real bug; see "Production-code changes triggered by tests" in Notes).

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

Test additions only by design. Production-code changes triggered by tests are tracked separately (see "Production-code changes triggered by tests" in Notes for Collaborators) and land in their own commits with logbook entries.

## Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| 1 | Tier 1a — Real IPOPT infeasibility + timeout exit codes | NOT STARTED | | Med | Solver-fallback latching on every documented exit code |
| 2 | Tier 1b — Fallback escalation cascade + cold-start IK budget | NOT STARTED | | Med | walk_forward_unsafe → hold_extrap escalation; IK budget exhaustion path |
| 3 | Tier 1c — NaN/Inf input fuzz on `solve()` | NOT STARTED | | Med | Adversarial inputs route through `_handle_failure`, never corrupt warm-start |
| 4 | Tier 2a — HardwarePlant FK degradation | NOT STARTED | | High | FK divergence watchdog, singular Jacobian, frozen-motor detector |
| 5 | Tier 2b — HardwarePlant telemetry & FF | NOT STARTED | | High | Staleness threshold matrix; set_pose torque-FF singular; cold-start zero-state |
| 6 | Tier 2c — ZMQ corruption (real-msgpack harness) | NOT STARTED | | Med | Partial frame, version-skew, connection drop |
| 7 | Tier 3a — Numerical + schema fuzz | NOT STARTED | | Med | Every public API NaN-safe; `diag`/`extras` schema-complete |
| 8 | Tier 3b — Time pathologies, resources, hooks, races | NOT STARTED | | Med | Clock-skew, dt change, hook failures, concurrent-reset, resource exhaustion |

## Implementation Phases (detailed)

### Phase 1: Tier 1a — Real IPOPT infeasibility + timeout exit codes — NOT STARTED

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

**Dependencies.** Plan 1 Phase 8 (CI hypothesis profiles) — useful but not strictly required.

**Exit criteria.** All 6 categories of solver exit confirmed. `_FALLBACK_KEYWORDS` matrix complete. Logbook entry `2026-XX-XX-tier1-real-solver-failures.md` documenting the IPOPT exit codes that ship with the pinned CasADi version (in case future upgrades add new codes).

---

### Phase 2: Tier 1b — Fallback escalation cascade + cold-start IK budget — NOT STARTED

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

### Phase 3: Tier 1c — NaN/Inf input fuzz on `solve()` — NOT STARTED

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

### Phase 4: Tier 2a — HardwarePlant FK degradation — NOT STARTED

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

All tests in this plan are unit tests by design (test additions, not production-code changes).

| Phase | New test file | Test count (approx) | Strategy |
|-------|---------------|---------------------|----------|
| 1 | `test_solver_failures.py` | ~15 | Scenarios + parameterized `_FALLBACK_KEYWORDS` matrix |
| 2 | `test_solver_failures.py` (extend) | ~10 | Scenarios + 1 hypothesis property |
| 3 | `test_mpc_input_fuzz.py` | ~10 | Hypothesis (composite strategies + stateful) |
| 4 | `test_hardware_plant_failure_paths.py` | ~12 | Scenarios + 1 hypothesis stateful |
| 5 | `test_hardware_plant_failure_paths.py` (extend) | ~8 | Parameterized scenarios + 1 hypothesis property |
| 6 | `test_zmq_corruption.py` + `_zmq_test_harness.py` | ~8 | Scenarios + 1 hypothesis byte-corruption property |
| 7 | `test_mpc_input_fuzz.py` (extend) + `test_diag_schema_fuzz.py` | ~16 | Mix |
| 8 | `test_mpc_time_pathologies.py` | ~14 | Mix |

Total: ~93 new tests.

### Integration tests

Limited to "the test suite still works" — full `pytest tests/ -q` after each phase.

| ID | Test | Pass criterion |
|----|------|----------------|
| T-I-1 | Full `pytest tests/ -q` after each phase | 100% pass |
| T-I-2 | `tests/sim/test_hot_loop_allocation_contract.py` after each phase | `< THRESHOLD_BYTES = 256` (no regression from new test infrastructure) |
| T-I-3 | `pytest tests/ -q --hypothesis-profile=ci-deep` after Phase 8 (final) | 100% pass at `max_examples=1000` |

### Hardware tests

| ID | Test | Validates | Pass criterion |
|----|------|-----------|----------------|
| T-H-T2a-1 | After Phase 4 — actual FK divergence (force a transient encoder dropout via CAN unplug for ~150 ms) | FK watchdog fires E-stop on real hardware | `motor_guard` reports E-stop with `reason='telemetry_stale'` or `'fk_divergence'` |
| T-H-T2b-1 | After Phase 5 — actual telemetry stall (kill the encoder publisher thread for 600 ms) | ESTOP threshold fires | E-stop fires within 500 ms ± 25 ms |

These hardware tests are NOT part of the regular test suite — they are manual bringup tests run after the corresponding phase lands. Each hardware test gets a logbook investigation entry under `/investigate`.

### Regression tests

| ID | Test | Pass criterion |
|----|------|----------------|
| T-R-1 | All existing tests pass after every phase | 100% pass |
| T-R-2 | `test_hot_loop_allocation_contract.py` budget unchanged | `< 256 B/tick` |
| T-R-3 | `test_make_feasible_events.py` properties (Plan 1 Phase 7 retroactive expansion) still pass | 100% at `max_examples=1000` |

### Property-test depth schedule

Same as Plan 1: per-PR `ci-fast` (50 examples), nightly `ci-deep` (1000 examples). Property tests added in this plan respect those profiles.

## Notes for Collaborators

### Production-code changes triggered by tests

This plan is test additions only by design. **However**, tests in this plan may surface real bugs (a documented failure path that doesn't actually behave as documented). When that happens:

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
| `tests/sim/test_solver_failures.py` | Created | 1, 2 |
| `tests/sim/test_mpc_input_fuzz.py` | Created | 3, 7 |
| `tests/sim/test_hardware_plant_failure_paths.py` | Created | 4, 5 |
| `tests/sim/test_zmq_corruption.py` | Created | 6 |
| `tests/sim/_zmq_test_harness.py` | Created | 6 |
| `tests/sim/_hardware_plant_stub.py` | Modified (extend with FK injection) | 4 |
| `tests/sim/test_diag_schema_fuzz.py` | Created | 7 |
| `tests/sim/test_mpc_time_pathologies.py` | Created | 8 |
| `logbook/2026-XX-XX-...` | Created (one entry per phase) | 1, 2, 3, 4, 5, 6, 7, 8 |

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

                                                 Phase 1: T1a real IPOPT failures
                                                 Phase 2: T1b escalation cascade
                                                 Phase 3: T1c NaN/Inf input fuzz
                                                 Phase 4: T2a HardwarePlant FK degradation
                                                 Phase 5: T2b telemetry & FF
                                                 Phase 6: T2c ZMQ corruption
                                                 Phase 7: T3a numerical + schema fuzz
                                                 Phase 8: T3b time / resources / hooks / races
```

Plan 2 Phase 1 can start as soon as Plan 1 Phase 3 lands (scheduler contract enforcement complete). Plan 2 Phases 4–5 require Plan 1 Phase 6 (P4 wired). Plan 2 Phase 8 requires Plan 1 Phase 3 (S6 raises on backward time) — without that, the time-pathology tests have no contract to verify.
