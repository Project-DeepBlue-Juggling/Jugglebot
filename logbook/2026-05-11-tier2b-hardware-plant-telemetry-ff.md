---
title: MPC sad-path coverage — Phase 5 Tier 2b HardwarePlant telemetry-staleness, set_pose FF, cold-start
type: feature
date: 2026-05-11
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 5"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-11-tier2a-hardware-plant-fk-degradation
  - 2026-05-11-tier1c-input-fuzz-bugfix
  - 2026-05-11-tier1c-input-fuzz
  - 2026-05-11-tier1b-fallback-escalation-cascade
  - 2026-05-11-tier1a-real-solver-failures
  - 2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement
files_changed:
  - tests/sim/test_hardware_plant_failure_paths.py
  - tests/sim/_hardware_plant_stub.py
  - logbook/2026-05-11-tier2b-hardware-plant-telemetry-ff.md
  - logbook/INDEX.md
  - plans/active/mpc-sadpath-coverage-tiers-1-3.md
commits:
  - 3e71ce5
subsystem:
  - controller
  - hardware-plant
  - safety
tags:
  - testing
  - safety
  - watchdog
  - hypothesis
  - hardware-plant
  - estop
  - telemetry
  - cold-start
  - bug-surfaced
---

# MPC sad-path coverage — Phase 5 Tier 2b HardwarePlant telemetry-staleness, set_pose FF, cold-start

## Summary

Plan 2 Phase 5 (Tier 2b) — extends
[tests/sim/test_hardware_plant_failure_paths.py](../tests/sim/test_hardware_plant_failure_paths.py)
with **9 new tests** (8 + 1 boundary-pair) exercising the
telemetry-staleness three-tier watchdog (WARN @ 3× control_dt,
HARD @ 5×, ESTOP @ 20×), the `set_pose()` torque-FF singular-Jacobian
path, and the cold-start zero-state path; plus an extension to
[tests/sim/_hardware_plant_stub.py](../tests/sim/_hardware_plant_stub.py)
with two telemetry-staleness driver helpers (`drain_recv_pump`,
`freeze_perf_counter_at`) and a module-level MuJoCo-IK cache that
makes per-example plant construction ~50× cheaper.

The surfaces covered + their newly-tested behaviours:

| Surface | Threshold / behaviour | Production code | New test coverage |
|---------|-----------------------|-----------------|-------------------|
| Telemetry-staleness WARN | `3× control_dt` → once-only 'Telemetry aging' WARN; flag set | `hardware_plant.py:695–700` | T-U-T2b-1 |
| Telemetry-staleness HARD | `5× control_dt` → 'zeroing velocities' WARN; `leg_velocities_mmps` zeroed | `:686–694` | T-U-T2b-2 |
| Telemetry-staleness ESTOP | `20× control_dt` → `estop('telemetry_stale')` | `:627–631` | T-U-T2b-3 + below-edge boundary regression |
| Edge-triggered re-arm | flag resets on recovery; new WARN on next stale event | `:701–702` | T-U-T2b-4 |
| Linear-scaling property (P4) | `_TELEM_STALE_*_MULT × control_dt` derives per-instance thresholds | `:73–75, :130–132` | T-U-T2b-5 (hypothesis, control_dt ∈ [0.01, 0.1]) |
| set_pose singular-FF (bug) | All-zero `torque_ff_Nm` on singular J — **silent** in current code | `dynamics.py:341–344` | T-U-T2b-6 (XFAIL pre-bugfix) |
| Cold-start zero-state | `_last_telem is None` → all-zero `PlantState`; `data_age_s=None` | `hardware_plant.py:529–535` | T-U-T2b-7 |
| Cold-start staleness ESTOP | Fresh plant + 1 s no recv → `estop('telemetry_stale')` | `:627–631` (cold path) | T-U-T2b-8 |

**Test-additions only at this commit; one xfail-strict marker.**
T-U-T2b-6 surfaced a real semantic bug: `dynamics.py:341–344` silently
catches `LinAlgError` in `np.linalg.solve(J.T, W_total)` and returns
`np.zeros(6)`.  `HardwarePlant.set_pose` propagates the all-zero
feedforward to `_ff_torque_buf` with **no warning**, despite the
parallel handler in `get_state()`'s twist-solve path at
`hardware_plant.py:737–740` emitting a once-only 'Jacobian singular'
warning.  Per CLAUDE.md's *"Fix surfaced bugs in the same session
when diagnosis is clear"* rule, the fix lands as a follow-up commit
in the same session — see
[`logbook/2026-05-11-tier2b-set-pose-singular-ff-bugfix.md`](2026-05-11-tier2b-set-pose-singular-ff-bugfix.md)
(populated when the bugfix commit lands).

Pre-Phase-5: **1243 passed + 1 xfailed** (`pytest tests/ -q`, run
2026-05-11 against SHA `2ea1a33`, 383.33 s).  Post-Phase-5
(this test commit): **1251 passed + 2 xfailed in 351.06 s**
(`pytest tests/ -q`, run 2026-05-11; +8 new passing tests
[T-U-T2b-1..5, -7, -8 = 7 IDs but T-U-T2b-3 is a pair → 8 functions];
+1 new xfail = T-U-T2b-6, pending the bugfix follow-up commit).
Full triple repeated in Verification below.

## Motivation

Phase 4 (Tier 2a) covered the three watchdog cascades on
`HardwarePlant.get_state()` (FK convergence, singular-Jacobian
twist-zero, frozen-motor detector).  Phase 5 (Tier 2b) covers the
remaining HardwarePlant safety surfaces:

1. **Telemetry-staleness three-tier watchdog.**  The most
   safety-critical of the watchdogs — protects against the case
   where ZMQ telemetry stops arriving (publisher death, network
   wedge) and the MPC keeps commanding the platform from stale
   feedback.  Three documented thresholds with linear scaling per
   `control_dt`:
   * WARN @ 3× `control_dt` — operator awareness, no behaviour change
   * HARD @ 5× `control_dt` — zero leg velocities (protects velocity
     feedforward consumers from cascading errors)
   * ESTOP @ 20× `control_dt` — telemetry definitely lost; abort
2. **`set_pose()` torque-FF singular-Jacobian path.**  The dynamics
   model uses `np.linalg.solve(J.T, W_total)` to decompose the
   gravity + inertia wrench into per-leg forces.  At workspace
   singularities the solve raises `LinAlgError`; the current
   `dynamics.py:341–344` silently catches and returns zeros — but
   nothing emits a warning, so operator gets no signal that the
   FF model has failed at this pose.  This is an asymmetry to
   `get_state()`'s singular-Jacobian handler at `:737–740` which
   emits a once-only warning.
3. **Cold-start zero-state.**  Before any telemetry has arrived,
   `_last_telem is None` triggers a zero-state return at `:529–535`.
   This path is exercised on every fresh process startup but had
   zero test coverage.

Phases 4 and 5 together complete the **HardwarePlant unit-test
surface** for Plan 2.  Hardware-bringup tests (T-H-T2b-1 the gate,
T-H-T2a-1 the cascade) are scheduled below.

## Design

### Per-test empirical-probe table (run 2026-05-11)

Pre-implementation probes (`/tmp/probe_telem_stale.py`,
`/tmp/probe_remaining.py`; not committed) confirmed each driver
produces the expected behaviour on the current `hardware_plant.py`
(commit `2ea1a33` baseline).  Recipes are deterministic on the
pinned dependency stack:

| Test       | Production surface                                        | Driver                                                                                 | Empirical confirmation                                                                          |
|------------|-----------------------------------------------------------|----------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------|
| T-U-T2b-1  | WARN cascade at `:695–700`                                | `drain_recv_pump` + `freeze_perf_counter_at(3.5 × control_dt)`                          | One 'Telemetry aging' record; `_telem_stale_warned=True`; no estop                              |
| T-U-T2b-2  | HARD cascade at `:686–694`                                | `drain_recv_pump` + `freeze_perf_counter_at(5.5 × control_dt)`                          | 'zeroing velocities' record; `leg_velocities_mmps = 0`; no estop                                |
| T-U-T2b-3a | ESTOP gate at `:627–631`                                  | `drain_recv_pump` + `freeze_perf_counter_at(20.5 × control_dt)`                         | `estop(reason='telemetry_stale')`; HARD ALSO fires (age > 5×)                                   |
| T-U-T2b-3b | (boundary regression) below ESTOP                         | `drain_recv_pump` + `freeze_perf_counter_at(19.5 × control_dt)`                         | No estop; HARD still fires                                                                       |
| T-U-T2b-4  | Edge-triggered re-arm via `:701–702`                      | Three episodes: stale → recovery → stale-again                                          | Records: 1 → 1 (no new) → 2; flag: True → False → True                                          |
| T-U-T2b-5  | Property — `_TELEM_STALE_*_MULT × control_dt` derivation  | `@given(control_dt=[0.01,0.1], factor=['warn','hard','estop'])`; per-example fresh plant | 1000 examples ci-deep, seed=0, 29.70 s; invariant holds                                          |
| T-U-T2b-6  | `dynamics.py:341–344` silent LinAlgError catch            | patch `motor_commands.compute_jacobian` → rank-5 matrix                                 | `_ff_torque_buf=[0,0,0,0,0,0]`; **zero warning records** (pre-bugfix) → XFAIL                    |
| T-U-T2b-7  | Cold-start at `:529–535`                                  | `drain_recv_pump` BEFORE any `get_state()` call                                          | `_last_telem=None`; `data_age_s=None`; all-zero state; no estop; `_fk_ever_succeeded=False`     |
| T-U-T2b-8  | Cold-start + 1 s no recv → ESTOP                          | One prime tick, then `drain_recv_pump` + `freeze_perf_counter_at(1.0)`                  | `estop(reason='telemetry_stale')`; `data_age_s=1.0 > 0.5`                                       |

### Telemetry-staleness driver — choice and tradeoffs

The plan's at-risk decision (per the Phase 5 prompt) was how to
drive the staleness watchdog at a controlled `telem_age`.  Three
options were enumerated:

(a) **Frame-mutator timestamp encoding** — encode a `timestamp`
field in the msgpack payload that the plant reads at recv-time.
*Rejected* because the production code uses
`time.perf_counter()` at the recv boundary (`hardware_plant.py:513`),
NOT a payload field; this would require also patching the recv
handler.  Doesn't apply.

(b) **`drain_recv_pump` + patch `time.perf_counter`** — replace the
pump with an always-Again callable (so `drain_count == 0` keeps
`_last_telem_recv_time` from being refreshed at `:512–513`) and
patch `controller.hardware_plant.time` so `perf_counter()` returns
a controlled instant.  Drives the **real**
`telem_age = now - self._last_telem_recv_time` arithmetic at
`:519–520` with the real recv-timestamp.  *Selected.*

(c) **Direct `_last_telem_recv_time` poke** — set
`plant._last_telem_recv_time = perf_counter() - age_s` directly.
*Rejected* because it violates Plan 2 Working Note #1 ("Drive real
failures, not mocked ones") — pokes private state rather than
patching at the boundary.

The selected approach respects the boundary discipline: the time
source is patched at the module-import surface
(`controller.hardware_plant.time`), and the recv side is patched at
the ZMQ socket surface (`plant._sub.recv_multipart`).  No
production state field is touched directly.

### Module-level IK cache — enabling cheap-plant property testing

T-U-T2b-5 is the first test in Plan 2 to construct many
`HardwarePlant` instances at varying `control_dt` (the property
test fuzzes `control_dt ∈ [0.01, 0.1]`).  Each construction was
paying a ~1 s MuJoCo IK precompute inside the stub fixture
(`MuJoCoPlant.pose_to_extensions(target_pose)` at the old
`_hardware_plant_stub.py:118–120`).  Without mitigation, ci-deep
(1000 examples) would have been ~20 min.

**Mitigation** (landed in
[tests/sim/_hardware_plant_stub.py](../tests/sim/_hardware_plant_stub.py)):
hoist the IK precompute behind a module-level dict cache
`_IK_PRECOMPUTE_CACHE` keyed by `tuple(target_pose)`.  Each unique
pose pays the cost once; subsequent stubs at the same pose hit a
~0.02 s cache lookup.

Smoke-measured result (probe `/tmp/probe_remaining.py`, run
2026-05-11):

* First `build_hardware_plant_stub`: **0.94 s** (IK precompute).
* Second `build_hardware_plant_stub`: **0.02 s** (cache hit).
* ~50× speedup per repeated build.

The cache is hidden behind `_get_cached_motor_rev` — a private
helper used by both `build_hardware_plant_stub` and
`install_telemetry_pump`.  As a side benefit Phase 4's tests
also got faster (5.5 s vs 41.5 s in the Phase 4 logbook's
Verification; ~7.5× speedup).

ci-deep T-U-T2b-5 result with the cache landed: **1 passed in
29.70 s** (1000 examples).  See Verification below.

### T-U-T2b-5 hypothesis property structure

The plan offered three structures:

(a) `@given` parametrized over `(control_dt, factor)` with
per-example fresh plant (simple, requires the IK cache).
(b) `RuleBasedStateMachine` with fixed control_dt buckets (faster
per-example, less coverage of the `control_dt` axis).
(c) Hybrid stateful + `@given` outer wrapper.

**(a) selected** because:

* The property is structurally one-shot per `(control_dt,
  threshold)` — each example fully exercises one threshold case at
  one `control_dt`.  Stateful walks would either replay redundant
  scenarios or require complex per-walk plant cleanup.
* The IK cache (above) bounds per-example cost to ~50 ms, so 1000
  examples × 3 thresholds-sampled-from = ~30 s, comfortable for
  ci-deep.
* Mirrors Plan 1's pattern of using `@given` for one-shot property
  checks and `RuleBasedStateMachine` for across-tick invariants
  (cf. Phase 4's `T2aFkBurstBelowThresholdMachine` over consecutive
  failure runs — there state matters; here it doesn't).

The strategy space is:

* `control_dt ∈ st.floats(min_value=0.01, max_value=0.1,
  allow_nan=False, allow_infinity=False)` — the documented bounds
  for the watchdog's linear-scaling promise.  Outside this range
  the multipliers may no longer be meaningful (e.g. 1 ms loop ×
  3× = 3 ms WARN, tighter than ZMQ recv jitter; 1 s loop × 20× = 20 s
  ESTOP, longer than any reasonable hardware fault recovery).
* `factor_choice ∈ st.sampled_from(['warn', 'hard', 'estop'])` —
  one of the three thresholds.  Sampling across the categorical
  axis catches threshold-specific bugs that `control_dt` alone
  wouldn't.

Two invariants per example:

1. **Per-instance derivation**:
   `plant._telem_stale_<factor>_s == _TELEM_STALE_<factor>_MULT *
   control_dt` (float compare to `abs=1e-12`).  Catches drift in the
   multiplier constants OR the `__init__` derivation.
2. **Cascade firing at the documented age**: `freeze_perf_counter_at(
   plant, factor_above × control_dt)` drives one tick; the
   per-factor assertion (record present / vels zeroed / estop
   fired) holds.

### T-U-T2b-6 — the surfaced bug

Plan text hedged: *"T-U-T2b-6 might surface a real semantic
question: should `set_pose()` silently zero on singular FF, or
should it raise? Document the current behaviour; if it's silent
zero with a once-only warning, that's the contract; if it's silent
without warning, that's a bug worth filing."*

Empirical probe (`/tmp/probe_remaining.py`) confirmed: **silent
without warning**.

Trace:
1. `HardwarePlant.set_pose` (`hardware_plant.py:770`) calls
   `cartesian_to_motor_commands` (`motor_commands.py:35`).
2. `cartesian_to_motor_commands` computes `J = compute_jacobian(...)`
   (line 82) then calls
   `compute_full_feedforward_torques(..., J=J, ...)` (line 94).
3. `compute_full_feedforward_torques` (`dynamics.py:287`) tries
   `f_legs = np.linalg.solve(J.T, W_total)` (line 342) — **catches
   `LinAlgError` silently and returns `np.zeros(6)`** (lines 343–344).
4. `set_pose` writes the (all-zero) `torque_ff_Nm` to
   `_ff_torque_buf` (line 826) — **no warning emitted**.

This is asymmetric to `get_state()`'s twist-solve singular-J handler
at `hardware_plant.py:737–740`, which emits a once-only warning:

```python
except np.linalg.LinAlgError:
    if not self._jacobian_singular_warned:
        logger.warning("Jacobian singular — platform twist zeroed")
        self._jacobian_singular_warned = True
```

The `set_pose` path silently propagates the all-zero FF.  Operator
gets no signal that the FF model failed at a workspace boundary —
PID has to handle gravity + inertia alone, and the operator never
knows.

**Per CLAUDE.md's "Fix surfaced bugs in the same session when
diagnosis is clear"**: the diagnosis is unambiguous (the LinAlgError
catch is visible at `dynamics.py:341–344`; the missing handler in
`set_pose` is the gap; the fix mirrors the existing
`get_state()` once-only pattern).  Same-session fix selected.

T-U-T2b-6 is marked `@pytest.mark.xfail(strict=True)` in **this**
commit.  The follow-up commit removes the xfail marker as part of
the bugfix.  Test commit → bugfix commit pattern mirrors
[`logbook/2026-05-11-tier1c-input-fuzz.md`](2026-05-11-tier1c-input-fuzz.md)
→ [`logbook/2026-05-11-tier1c-input-fuzz-bugfix.md`](2026-05-11-tier1c-input-fuzz-bugfix.md).

### Why detect-symptom over detect-cause in the fix

Two options for the fix:

(α) **Detect cause** — refactor `dynamics.py:341–344` to NOT catch
LinAlgError; let it propagate; `set_pose` catches and warns.  Clean
layering but changes the `dynamics.py` contract; other callers
(`compute_full_feedforward_torques` is exported) may depend on the
silent-zero behaviour.
(β) **Detect symptom** — check if `torque_ff_Nm` is all-zero in
`set_pose` after the call.  Once-only flag + reset-on-recovery
mirrors the `get_state()` pattern.  Localised, layering preserved.

The user selected (β) — the symptom-detection approach.  Rationale:
the singular-FF detection becomes a `set_pose`-specific concern,
the dynamics module stays pure math, and the false-positive
surface is small (the gravity wrench is non-zero in any pose with
feedforward enabled, so an all-zero `torque_ff_Nm` reliably
indicates the singular fallback fired).

The bugfix in the follow-up commit adds a `_singular_ff_warned`
flag (mirrors `_jacobian_singular_warned`) and a once-only warning
in `set_pose`.

### Cold-start path — boundary citation refresh

The plan cites the cold-start zero-state branch at
`hardware_plant.py:507–513`.  Empirical reading shows the actual
branch is at **`:529–535`**:

```python
if self._last_telem is None:
    state.leg_extensions_mm.fill(0.0)
    state.leg_velocities_mmps.fill(0.0)
    state.platform_pos_mm.fill(0.0)
    state.platform_rot.fill(0.0)
    state.platform_twist.fill(0.0)
    return state
```

The `_state` itself is initialised at **`:284–294`** (the plan's
"approximate :283–294" estimate matches ground truth).  The drift is
~22 lines forward — small but worth recording per Plan 2 Working
Note #3.  Phase 5 tests reference symbols (`_last_telem`,
`_state.platform_pos_mm`) rather than line numbers, so the drift
doesn't affect the tests; only the docstring's narrative citation
is refreshed.

Similarly the singular FF "documented behaviour at
`hardware_plant.py:782–797`" in the plan resolves to
`hardware_plant.py:770–827` (the full `set_pose` body) — the FF
catch is INSIDE `dynamics.py`, not in `hardware_plant.py` at all,
so the plan's narrative was misleading.  The Outcome paragraph
above refers to the actual location.

### Hardware-test target dates — T-H-T2b-1 and T-H-T2a-1

Per Plan 2 Working Note #7, this phase's commit fixes the absolute
deadlines for both hardware bringup tests.

| Test       | Hazard          | Target date    | Gating                                                 |
|------------|-----------------|----------------|--------------------------------------------------------|
| T-H-T2b-1  | Low             | **2026-05-18** | None — the gate test itself                            |
| T-H-T2a-1  | Medium-high     | **2026-05-25** | T-H-T2b-1 PASS confirms the publisher-kill mechanism   |

Both within 2 weeks of Phase 5's commit (today, 2026-05-11).
T-H-T2b-1 runs first because:

* It is hazardously **Low** (ODrives keep receiving setpoints from
  `motor_guard`; platform stays held; only the Jetson-side observation
  of `/robot_state` is affected).
* It is the normative gate for T-H-T2a-1 (Medium-high hazard;
  platform freewheels under gravity during the watchdog window).
* Per the Testing Plan's pre-test setup checklist, T-H-T2a-1 must
  not run before T-H-T2b-1 has demonstrated the watchdog
  mechanism works in isolation.

Both target dates carry the operator-confirmation pre-test setup
checklist (low pose, hand detached, operator on E-stop button,
second person present, instrumented logging of the timing chain).

### Xfail accounting — Phase 5 (this commit; bugfix commit below)

At this commit (test additions; bugfix follow-up pending in same
session), the suite carries **2 xfails**:

| Test ID       | Reason                                                          | Tracking                                                                                                              | Target close                          |
|---------------|-----------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------|---------------------------------------|
| T-U-T1a-4     | `Restoration_Failed` not drivable via `MPCParams` in CasADi 3.7.2 | [logbook 2026-05-11-tier1a-real-solver-failures.md](2026-05-11-tier1a-real-solver-failures.md) (Discussion → Xfail)   | Permanent (CasADi 3.7.2 limitation) |
| T-U-T2b-6     | `dynamics.py:341–344` silent LinAlgError catch; no warning in `set_pose` | [logbook 2026-05-11-tier2b-set-pose-singular-ff-bugfix.md](2026-05-11-tier2b-set-pose-singular-ff-bugfix.md) (pending) | **Same session — bugfix follow-up commit** |

T-U-T2b-6's target close is the immediate next commit in this
session.  Per Plan 2's archival-gate language: zero unfixed xfails
at archival, OR each residual xfail has a permanent justification.
T-U-T1a-4 has a permanent justification; T-U-T2b-6 will be removed
in the bugfix commit landing today.

## Implementation

### tests/sim/_hardware_plant_stub.py — Phase 5 extension

| Helper                                       | Boundary                                                              | Purpose                                                                |
|----------------------------------------------|-----------------------------------------------------------------------|------------------------------------------------------------------------|
| `drain_recv_pump(plant)`                     | `plant._sub.recv_multipart`                                            | Replace with always-`Again` so `drain_count=0` (no recv-time refresh)  |
| `freeze_perf_counter_at(plant, age_s)`       | `controller.hardware_plant.time` (module reference)                    | Patch `perf_counter()` to return `_last_telem_recv_time + age_s`        |
| `_IK_PRECOMPUTE_CACHE` + `_get_cached_motor_rev` | Internal — wraps `MuJoCoPlant.pose_to_extensions`                  | Module-level cache; ~50× speedup on repeat stubs                       |

`drain_recv_pump` writes nothing to `_last_telem_recv_time`
directly; it only replaces the recv pump.  `freeze_perf_counter_at`
patches the module-level `time` reference under
`unittest.mock.patch.object`, restored on `__exit__`.

### tests/sim/test_hardware_plant_failure_paths.py — Phase 5 extension

| Class / function                                       | ID(s)                | Tests | Strategy                                                              |
|--------------------------------------------------------|----------------------|-------|-----------------------------------------------------------------------|
| `TestTelemetryStalenessThresholds`                     | T-U-T2b-1, -2, -3, -4 | 5     | scenario (`drain_recv_pump` + `freeze_perf_counter_at`); +1 boundary regression |
| `test_t2b_5_threshold_scaling_property`                | T-U-T2b-5            | 1     | `@given(control_dt, factor_choice)` per-example fresh plant            |
| `TestSetPoseFfSingular`                                | T-U-T2b-6            | 1     | `xfail(strict=True)` on the asserted post-fix behaviour                |
| `TestColdStartZeroState`                               | T-U-T2b-7, -8        | 2     | scenario (cold-start via `drain_recv_pump` BEFORE prime)               |

Module-level helpers (`_TELEM_WARN_MULT`, `_TELEM_HARD_MULT`,
`_TELEM_ESTOP_MULT`) read the production constants at import time so
any drift surfaces as a test-import error rather than a silent test
bug.

`_telem_stale_records(records)` is a thin filter on
`_capture_hp_warnings()` output that selects telemetry-staleness
records (matches 'telemetry', 'velocities', 'aging' — the three
substring tokens the production code uses across the WARN, HARD,
ESTOP messages).

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple per the workflow rule on test-count claims.

### Baseline (post Phase 4 + SHA backfill)

* `pytest tests/ -q`, run 2026-05-11 against SHA `2ea1a33`:
  **1243 passed + 1 xfailed in 383.33 s.**

### Module-isolated run

* `pytest tests/sim/test_hardware_plant_failure_paths.py -q`, run
  2026-05-11: **18 passed + 1 xfailed in 8.10 s.**  Combined Phase 4
  + Phase 5 tests in isolation at ci-fast.  Notably faster than the
  Phase 4 logbook's 41.5 s run thanks to the IK cache (~7.5×
  speedup on the Phase 4 subset).

### Property test depth — ci-deep validation

* `pytest tests/sim/test_hardware_plant_failure_paths.py::test_t2b_5_threshold_scaling_property
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q`, run
  2026-05-11: **1 passed in 29.70 s** (1000 examples).
  Linear-scaling property holds across `control_dt ∈ [0.01, 0.1]`
  for all three thresholds at nightly depth with deterministic seed.

### Hot-loop allocation contract — post-additions regression check

* `pytest tests/sim/test_hot_loop_allocation_contract.py
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q`, run
  2026-05-11: **3 passed in 16.00 s.**  `gc.collect()` mitigation
  from Phase 3 holds; Phase 5's hypothesis additions did NOT
  introduce a regression.

### Full-suite gate (post Phase 5, this commit)

* `pytest tests/ -q`, run 2026-05-11 with all Phase 5 changes
  applied: **1251 passed + 2 xfailed in 351.06 s.**  +8 passed
  matches the new tests (T-U-T2b-1, -2, -3, -3-boundary, -4, -5,
  -7, -8); +1 xfailed is T-U-T2b-6, pending the bugfix follow-up
  commit landing in this session.  Zero regressions on existing
  tests; the inherited Phase 1 xfail (T-U-T1a-4 `Restoration_Failed`)
  remains.

## Discussion

### What Phase 5 reveals about the HardwarePlant safety surface

Three structural observations:

1. **The telemetry-staleness watchdog scales linearly with `control_dt`
   by design — and the test proves it.**  Plan 1 Phase 6's P4
   contract ("Period awareness — `control_dt` is a mandatory
   constructor kwarg.  All time-derived thresholds scale linearly with
   it") was previously documented but unverified.  T-U-T2b-5 is the
   first test in this codebase to exercise P4's linear-scaling
   promise across a range of `control_dt` values.  The 1000-example
   ci-deep run confirms the promise holds at every floating-point
   `control_dt` ∈ [0.01, 0.1].  A future operating-regime change
   (e.g. drop to 20 Hz for a heavier payload, or rise to 100 Hz for
   tighter tracking) is now safe to ship without re-validating the
   staleness thresholds by hand.

2. **The set_pose silent-FF bug is the first Phase-surfaced
   production bug in this plan.**  Phase 3 (Tier 1c) surfaced
   `T-U-T1c-7-bug` (NaN/Inf `q_cur`/`q_dot` propagation in
   `_handle_failure`); Phase 4 surfaced no production bugs.  Phase
   5's T-U-T2b-6 surfaces the second — a parallel-handler
   asymmetry where `get_state()` warns on singular Jacobian but
   `set_pose()` does not.  Pattern: **safety-critical paths come in
   pairs (get_state observes; set_pose commands), and a
   well-handled one path can mask the gap in the other.**  The
   Phase 4 logbook's *"three cascades are independent — and that's
   the point"* framing applies here too: a singular-J event at
   a workspace boundary should warn the operator REGARDLESS of
   which surface (read or write) hit it.  The bugfix restores
   parity.

3. **The cold-start zero-state path was uncovered before Phase 5.**
   Every fresh process startup runs through `:529–535` until the
   first telemetry arrives — i.e. every operator-initiated MPC
   session begins with this branch firing.  T-U-T2b-7 and T-U-T2b-8
   are not edge cases; they exercise the *normal startup path*.
   That this branch had **zero** test coverage before Phase 5
   is a structural gap, not a hardening overhead.

### Why per-example fresh plants for T-U-T2b-5 (over a singleton)

Phase 4's `T2aFkBurstBelowThresholdMachine` uses a module-scoped
singleton plant to amortise the IK precompute cost across
hypothesis examples.  T-U-T2b-5 could not reuse that pattern
because **the staleness thresholds are baked into the plant at
construction** (`self._telem_stale_warn_s = 3.0 × self._control_dt`
etc., at `:130–132`).  A singleton plant carries a single
`control_dt` value; the property test needs to vary `control_dt`
across examples.

Three options were considered:

(a) Per-example fresh plant (selected).
(b) One singleton per discrete `control_dt` bucket (e.g.
{0.01, 0.025, 0.05, 0.1}).  Faster but loses the float-fuzz
coverage of the property.
(c) Reconstruct the staleness thresholds dynamically (poke
`plant._control_dt = new_dt` then re-derive `_telem_stale_*_s`).
Rejected — pokes private state, violates the test-the-watchdog
discipline of WN#1.

(a) wins because the IK cache lands first.  Without the cache,
1000 examples × ~1 s/example = ~17 min — infeasible for ci-deep.
With the cache, 1000 × ~30 ms = 30 s.  The cache turned an
otherwise-infeasible structure into the cheapest one.

### Stub extension footprint — Phase 4 vs Phase 5

Phase 4's three injection helpers all patched at production-code
**function boundaries** (`leg_lengths_to_pose`, `compute_jacobian`,
`recv_multipart`).  Phase 5's two new helpers patch at **module
boundaries** — one (`drain_recv_pump`) replaces the same
`recv_multipart` Phase 4 already touched, but with an
always-`Again` callable rather than a programmable frame mutator;
the other (`freeze_perf_counter_at`) replaces the `time` module
reference inside `controller.hardware_plant`.

Both helpers honour the boundary-patch discipline (no private
state pokes).  Distinguishing feature of Phase 5: the time-module
patch is **module-scoped, not socket-scoped** — it affects every
`time.perf_counter()` call inside `hardware_plant.py` for the
duration of the context, not just the recv path.  This is a
deliberate choice: the production code computes `telem_age` against
`time.perf_counter()` calls in `get_state()` (at `:515, :519`); both
must agree.  A socket-scoped patch would leave the inner
`perf_counter` read uncontrolled.

### Plan-text drift — line-citation refresh

The plan's Phase 5 section cites several line ranges in
`hardware_plant.py`: `:73–75`, `:128–132`, `:507–540`, `:625–631`,
`:685–702`, `:284–294`, `:782–797`.  Empirical reading shows:

* `:73–75` (multipliers) — ground truth.
* `:128–132` (per-instance derivation) — ground truth at
  `:130–132` (off by ~2 lines).
* `:507–540` (telemetry recv) — ground truth at `:499–520`
  (drifted forward ~8 lines).
* `:625–631` (ESTOP threshold check) — ground truth at `:627–631`
  (off by ~2 lines).
* `:685–702` (HARD + WARN cascade) — ground truth at `:686–702`
  (off by ~1 line).
* `:284–294` (cold-start PlantState init) — ground truth at
  `:284–294` (exact).
* `:782–797` (set_pose torque-FF) — ground truth at `:770–827`
  (the FF singular-J catch is **not** in `hardware_plant.py` at
  all; it's in `dynamics.py:341–344`).

Phase 5 documents the actual locations both in the test file's
docstrings (with symbol references) and in this entry.  Same
pattern as Phase 4 — the line numbers drifted between plan-writing
and implementation; the symbol references in the test code don't.

### Working Note #1 compliance — the test-the-watchdog framing

Plan 2 Working Note #1 warned that Phase 4's FK divergence tests
must drive through `get_state()` with simulated bad telemetry, not
poke the watchdog counter.  Phase 5 inherits and extends:

* **Telemetry-stale tests** — patch `time.perf_counter` at the
  module boundary and the recv pump at the socket boundary.  No
  direct write to `_last_telem_recv_time`, `_telem_stale_warned`,
  or `_estop_requested`.
* **`set_pose` singular-FF test** — patch `compute_jacobian` at
  the `motor_commands` module boundary.  The real
  `np.linalg.solve(J.T, W_total)` raises `LinAlgError` at
  `dynamics.py:342`; the real silent-catch at `:343–344` returns
  zeros; the real `_ff_torque_buf` ends up all-zero.  The bug
  surface (no warning) is observed via real logging records.
* **Cold-start tests** — replace the recv pump BEFORE any
  `get_state()` call, so `_last_telem is None` remains true
  through the entire test.  No write to `_last_telem`.

The only setup-time write to production state is in
`_EstopSpy.__init__`, which replaces `plant.estop` with a
record-and-call-original wrapper (the same Phase 4 spy pattern).
That's setup, not verification; the spy observes the public call
boundary, not a private flag.

### Per-instance derivation invariant — value of pinning it

T-U-T2b-5's *first* invariant is:

```python
plant._telem_stale_<factor>_s == _TELEM_STALE_<factor>_MULT * control_dt
```

This is structurally redundant given the production code at
`:130–132` literally implements this assignment.  Why pin it?

* A future refactor might decide to derive `_telem_stale_estop_s`
  from `_telem_stale_hard_s` instead of from `control_dt` (e.g.
  "estop is 4× hard").  The semantic would shift even if the
  numerical values at `control_dt=0.025` remained identical.
  The invariant catches this.
* The MULTIPLIER constants at `:73–75` are module-level; a future
  refactor might move them to a config file or compute them
  dynamically.  Pinning the constants by name (rather than by
  hard-coded `3.0`) catches the rename; pinning the derivation
  catches the relocation.

Without the invariant, only the **firing** behaviour is verified
— and float-equal values could agree even when the derivation
shifted (e.g. `0.075 == 3.0 * 0.025 == 5.0 * 0.015` is
ambiguous).

### Test-additions only at this commit; bugfix follow-up

Per the plan's *"Production-code changes triggered by tests"*
subsection:

> If a test surfaces a real bug AND the bug fix has been committed
> (per "Production-code changes triggered by tests"), the bug fix
> can be reverted independently of the test. The test will fail
> post-revert; that's the point — the failure is the signal that
> the regression has been re-introduced.

Phase 5's test additions land first as a single commit; the
T-U-T2b-6 bugfix lands as the immediate next commit in the same
session.  Two commits, two logbook entries:

1. This commit: tests; T-U-T2b-6 xfail-strict; logbook (this entry).
2. Next commit: `HardwarePlant.set_pose` warning + `_singular_ff_warned`
   flag; T-U-T2b-6 xfail marker removed; logbook
   `2026-05-11-tier2b-set-pose-singular-ff-bugfix.md`.

Rollback discipline: reverting commit 2 alone reproduces the bug
(T-U-T2b-6 returns to passing as xfail).  Reverting both commits
removes the test surface entirely.  Neither operation requires
touching the other.

## Open Questions

* **Should the staleness thresholds be documented in a
  ``HARDWARE_PLANT_SAFETY_CONTRACT.md`` document?**  Phase 4's
  logbook posed the same question for the FK / singular-J /
  frozen-motor cascades.  Phase 5 adds three more thresholds to
  the list (WARN, HARD, ESTOP staleness).  The case for a
  K1–K6-style contract document grows stronger: six normative
  thresholds in two phases, all safety-critical, all currently
  documented only in code comments.  Filed (still) as a Plan 2
  Phase 7 follow-up topic.

* **Should `set_pose` also detect non-finite (NaN/Inf) torque_ff,
  not just all-zero?**  Phase 5's bugfix detects the all-zero
  symptom (the documented singular-J fallback in `dynamics.py`).
  If a future refactor in `dynamics.py` returns NaN instead of
  zeros (e.g. removing the LinAlgError catch entirely; letting
  NaN-spread propagate), the all-zero detector would miss that
  surface.  A NaN/Inf detector would be a strict superset.  Filed
  for the Phase 7 schema-completeness work — `extras` and `diag`
  schema validation may surface this naturally.

* **Is 1.0 s the right age to drive T-U-T2b-8's cold-start ESTOP?**
  Chose 1.0 s because it's safely above the 0.5 s ESTOP threshold
  at default `control_dt=0.025`.  Closer to the threshold (e.g.
  0.51 s) would be a tighter test but adds nothing in coverage —
  T-U-T2b-3 already pins the 20× boundary at 20.5× and 19.5×.
  T-U-T2b-8's job is the *cold-start* path (no prior FK success,
  but with one prime tick).  Documented; not a follow-up.

* **Does T-U-T2b-5 need to also check the WARN flag's edge-triggered
  semantic across `control_dt` values?**  Currently the property
  fires each threshold once per example.  Across examples, the
  flag state is reset (fresh plant), so the edge-triggered
  re-arm behaviour is exercised by T-U-T2b-4 (one-shot) and not
  by the property.  An extension would assert that the re-arm
  semantic also scales — but T-U-T2b-4 already confirms the flag
  resets when age drops below WARN, and the flag is the same
  field regardless of `control_dt`.  Not a follow-up.

## Related

* [plans/active/mpc-sadpath-coverage-tiers-1-3.md](../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 5 specification.
* [logbook/2026-05-11-tier2a-hardware-plant-fk-degradation.md](2026-05-11-tier2a-hardware-plant-fk-degradation.md)
  — Phase 4 (Tier 2a); injection-helper pattern, caplog workaround,
  e-stop spy pattern, hypothesis singleton + stub-extension all
  inherited.
* [logbook/2026-05-11-tier1c-input-fuzz.md](2026-05-11-tier1c-input-fuzz.md)
  & [logbook/2026-05-11-tier1c-input-fuzz-bugfix.md](2026-05-11-tier1c-input-fuzz-bugfix.md)
  — Phase 3 → bugfix arc; precedent for the same-session-fix
  pattern Phase 5 follows.
* [logbook/2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement.md](2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement.md)
  — Plan 1 Phase 6's P4 (`control_dt` constructor kwarg + linear
  threshold scaling).  T-U-T2b-5 is the first test to exercise the
  linear-scaling promise.
* [controller/PLANT_INTERFACE_CONTRACT.md](../controller/PLANT_INTERFACE_CONTRACT.md)
  — P4 normative text; T-U-T2b-5 validates compliance.
* [controller/HOT_LOOP_CONTRACT.md](../controller/HOT_LOOP_CONTRACT.md)
  — allocation budget; Phase 5's `gc.collect()` mitigation from
  Phase 3 holds post-additions (regression-checked at ci-deep).
* [controller/hardware_plant.py](../controller/hardware_plant.py)
  — `_TELEM_STALE_*_MULT` at `:73–75`; per-instance derivation at
  `:130–132`; cascade at `:627–702`; cold-start at `:529–535`.
* [ros_ws/src/jugglebot/jugglebot/motion/dynamics.py](../ros_ws/src/jugglebot/jugglebot/motion/dynamics.py)
  — `compute_full_feedforward_torques` singular-J catch at `:341–344`
  (the bug surface).
* [tests/sim/test_hardware_plant_failure_paths.py](../tests/sim/test_hardware_plant_failure_paths.py)
  — combined Phase 4 + Phase 5 tests.
* [tests/sim/_hardware_plant_stub.py](../tests/sim/_hardware_plant_stub.py)
  — Phase 5 extensions: `drain_recv_pump`, `freeze_perf_counter_at`,
  IK precompute cache.
