---
title: MPC Tier 0 — Scheduler & PlantInterface Contracts
created: 2026-05-08
status: completed
completed: 2026-05-10
archived: 2026-05-10
---

# MPC Tier 0 — Scheduler & PlantInterface Contracts

## Context

### Why this plan exists

A cross-cutting audit of the MPC layer (2026-05-08, conversation-only, no logbook entry) surfaced two structural patterns:

1. **Sad-path tests are strong where contracts are written, weak where they aren't.** [K1–K6](../../controller/REFERENCE_LAYER_CONTRACT.md) and the [hot-loop allocation contract](../../controller/HOT_LOOP_CONTRACT.md) have ratchet-protected enforcement and rich coverage (`tests/sim/test_make_feasible_events.py`, `tests/sim/test_hot_loop_allocation_contract.py`). The scheduler ([controller/scheduler.py](../../controller/scheduler.py)) and the plant abstraction ([controller/plant.py](../../controller/plant.py)) — equally safety-critical — have no normative contracts and the tests reflect that.
2. **`PlantInterface` under-specifies semantics that the hot-loop contract assumes.** PlantState aliasing (same instance returned every tick), `reset()` capability, and `control_dt` awareness are documented in HOT_LOOP_CONTRACT.md and various docstrings but not encoded in the abstract base class. A future `PlantInterface` implementation can pass every sim test and break on hardware.

This plan establishes two new normative contracts following the existing structural template, plus retroactively expands K1–K6 hypothesis-property coverage:

- **SCHEDULER_CONTRACT.md** — invariants **S1** (submission time), **S2** (unique IDs), **S3** (bounded in-flight slot set), **S4** (internal quintic feasibility — already partly enforced by `_verify_segment_feasibility`), **S5** (C0 continuity at every newly-built segment), **S6** (clock monotonicity).
- **PLANT_INTERFACE_CONTRACT.md** — invariants **P1** (PlantState aliasing), **P2** (`can_reset` capability), **P3** (input-validation contract), **P4** (`control_dt` awareness).
- **K1–K6 hypothesis expansion** — multi-event proposals (currently only two-event), K4 inter-event-span boundary as a property, K5 coincident-twist as a property, K6 clamp idempotence.

### What this plan achieves

1. **Closes the contract-vs-tests gap** for two safety-critical surfaces. Future failures of the same class are caught by enforcement tests, not in production.
2. **Reifies semantic invariants** that today live only in markdown. Future implementations of `PlantInterface` will fail the contract test rather than diverge silently from `MuJoCoPlant` / `HardwarePlant`.
3. **Establishes property-first testing** for state-machine and mathematical invariants. Hypothesis already ships in the venv (6.113.0) and is already used in `tests/sim/test_make_feasible_events.py:22`.
4. **Sets the foundation for Plan 2** — the MPC sad-path coverage rollup. Tier 1–3 testing leans on contract-defined invariants (e.g., "the scheduler refuses past-time events" is testable because S1 is enforced).

### When to do this

**Before** Plan 2 (`mpc-sadpath-coverage-tiers-1-3.md`). Tier 1–3 testing references contract-defined invariants in places.

**Concurrent with** the in-flight `refactor` branch — this is doc-and-test work plus narrow ABC additions. Conflicts should be minimal; phase commits will rebase cleanly.

**Prerequisites:** none. Hypothesis 6.113.0 already in `~/Desktop/PDJ_venv/venv/`.

### Related

- [REFERENCE_LAYER_CONTRACT.md](../../controller/REFERENCE_LAYER_CONTRACT.md) — K1–K6 contract (template for new contracts)
- [HOT_LOOP_CONTRACT.md](../../controller/HOT_LOOP_CONTRACT.md) — hot-loop allocation contract (template for new contracts; source of P1/P4 invariants in markdown form)
- [DOCUMENTATION_GUIDE.md § 2.4](../../DOCUMENTATION_GUIDE.md) — governs subsystem contract placement
- [tests/sim/test_make_feasible_events.py](../../tests/sim/test_make_feasible_events.py) — current K1–K6 hypothesis baseline
- [logbook/2026-04-20-k1-k6-reference-feasibility-resolution.md](../../logbook/2026-04-20-k1-k6-reference-feasibility-resolution.md) — K1–K6 W-phase rollout (process model for this plan)
- [logbook/2026-04-23-hot-loop-zero-allocation-contract.md](../../logbook/2026-04-23-hot-loop-zero-allocation-contract.md) — hot-loop contract rollout (second process model)

## Architecture

### Current

```
[ contracts ]                      [ enforcement point ]            [ tests ]
─────────────                      ──────────────────────           ─────────
REFERENCE_LAYER_CONTRACT.md  ◀───  controller/target.py             tests/sim/test_make_feasible_events.py
(K1–K6)                            :: make_feasible_events()        :: 2 hypothesis properties
                                                                    :: ~25 scenarios

HOT_LOOP_CONTRACT.md         ◀───  controller/hot_loop_contract.py  tests/sim/test_hot_loop_allocation_contract.py
                                   :: THRESHOLD_BYTES = 256         :: tracemalloc measurement gate

   <no contract>             ◀───  controller/scheduler.py          tests/sim/test_scheduler.py
                                   :: submit_event()                :: 24 happy-path tests / 7 classes
                                   :: replace_next_event()          :: no input-domain validation
                                   :: _verify_segment_feasibility() :: state-machine invariants implicit

   <no contract>             ◀───  controller/plant.py (ABC)        tests/sim/test_plant.py
                                   :: 4 abstract methods            :: 4 PlantState aliasing checks
                                                                    :: only against MuJoCoPlant
```

Gaps that this plan closes:

| Surface | Gap |
|---------|-----|
| `submit_event()` | No validation of `t_event >= t_now`; duplicate IDs silently replaced; queue unbounded |
| `replace_next_event()` mid-TRANSITIONING | Behaviour exists but not specified; no test asserts the splice is well-defined |
| `update(sim_time)` | No assertion that `sim_time` is monotone non-decreasing across calls |
| `_verify_segment_feasibility()` | Already enforces K2/K3 in strict mode; not yet covered as a contract invariant in markdown |
| `PlantInterface.get_state()` | Aliasing convention documented in HOT_LOOP_CONTRACT.md:434 only; no abstract guarantee, no test against `HardwarePlant` |
| `PlantInterface.reset()` | No-op on hardware (silent footgun); no `can_reset` capability flag |
| `PlantInterface.command()` | Input-validation contract undefined: trusted-callee or defensive boundary? Today neither |
| Period awareness | `hardware_plant.py:67–71` thresholds hard-coded for `control_dt=0.025`; no parameter wiring |

### Proposed

```
[ contracts ]                              [ enforcement point ]              [ tests ]
─────────────                              ──────────────────────             ─────────
REFERENCE_LAYER_CONTRACT.md          ◀───  controller/target.py               tests/sim/test_make_feasible_events.py
(K1–K6, +retroactive expansion)            :: make_feasible_events()          :: 6 hypothesis properties (was 2)
                                                                              :: ~25 scenarios + new K4/K6 boundary
                                                                              :: multi-event proposals

HOT_LOOP_CONTRACT.md                 ◀───  controller/hot_loop_contract.py    tests/sim/test_hot_loop_allocation_contract.py
                                           (unchanged)                         (unchanged)

SCHEDULER_CONTRACT.md (NEW)          ◀───  controller/scheduler.py            tests/sim/test_scheduler_contract.py (NEW)
:: S1  submission time                     :: submit_event() validates t      :: hypothesis RuleBasedStateMachine
:: S2  unique IDs                          :: ID-set membership check         :: 6 invariants as @invariant rules
:: S3  bounded queue                       :: queue-depth guard               :: scenario tests for documented branches
:: S4  internal quintic feasibility        :: _verify_segment_feasibility()   :: hypothesis property over segments
                                              (existing; promoted to contract)
:: S5  C0 continuity at segment splices    :: _build_segment_to_event reads   :: state-machine rule
                                              :: live motion state
:: S6  clock monotonicity                  :: update() rejects backward time  :: state-machine invariant

PLANT_INTERFACE_CONTRACT.md (NEW)    ◀───  controller/plant.py (ABC)          tests/sim/test_plant_interface_contract.py (NEW)
:: P1  PlantState aliasing                 :: docstring + abstract assertion  :: parameterized over MuJoCoPlant + HardwarePlantStub
:: P2  can_reset capability                :: new abstract property           :: assertion: reset() raises iff not can_reset
:: P3  input-validation boundary           :: documented (trusted-callee)     :: contract test pumps adversarial inputs
:: P4  control_dt awareness                :: new constructor kwarg           :: assertion: thresholds scale with dt
```

### IPC / Message formats

**No new IPC channels.** Plan 1 is doc + ABC + tests. The only Python-API change is:

```python
# controller/plant.py — PlantInterface gains:
class PlantInterface(ABC):
    @property
    @abstractmethod
    def can_reset(self) -> bool:
        """Whether reset() is supported. Hardware: False. MuJoCo: True."""

    @property
    @abstractmethod
    def control_dt(self) -> float:
        """Configured control period (s). Used for time-window thresholds."""
```

`HardwarePlant.__init__` gains:

```python
def __init__(self, *, control_dt: float = 0.025, **kw):
    self._control_dt = float(control_dt)
    # Stale-telemetry thresholds derived from control_dt, not hard-coded:
    self._telem_stale_warn_s  = 3.0  * self._control_dt   # was 0.075
    self._telem_stale_hard_s  = 5.0  * self._control_dt   # was 0.125
    self._telem_stale_estop_s = 20.0 * self._control_dt   # was 0.500
```

`MuJoCoPlant.__init__` gains the same `control_dt` parameter (default unchanged behaviour).

### Files to create

| Path | Purpose |
|------|---------|
| `controller/SCHEDULER_CONTRACT.md` | Normative S1–S6 specification |
| `controller/PLANT_INTERFACE_CONTRACT.md` | Normative P1–P4 specification |
| `tests/sim/test_scheduler_contract.py` | Hypothesis state-machine + scenario tests |
| `tests/sim/test_plant_interface_contract.py` | Parameterized contract test over implementations |
| `tests/conftest_hypothesis.py` | `register_profile('ci-fast', max_examples=50)` and `register_profile('ci-deep', max_examples=1000)` |

### Files to modify

| Path | Change |
|------|--------|
| `controller/scheduler.py` | Add S1/S2/S3 input validation; tighten S4 default to strict in tests; add S5 mid-TRANSITIONING splice spec; add S6 monotone `sim_time` check |
| `controller/plant.py` | Add `can_reset`, `control_dt` abstract properties; tighten docstrings |
| `controller/hardware_plant.py` | Implement `can_reset=False`, accept `control_dt`, scale staleness thresholds |
| `sim/plant/mujoco_plant.py` | Implement `can_reset=True`, accept `control_dt` |
| `controller/runner.py` | Pass `control_dt` to plant constructor (call site); already accepts `control_dt` parameter |
| `tests/sim/test_make_feasible_events.py` | +K4 boundary property, +K5 coincident-twist property, +K6 idempotence property, +multi-event proposal property |
| `tests/conftest.py` | Import `conftest_hypothesis` (sibling helper) to register hypothesis profiles; ci-fast loaded by default at collection time |
| `pyproject.toml` (existing `[tool.pytest.ini_options]`) | Add `markers = [slow, nightly, hypothesis_deep]` |

## Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| 1 | SCHEDULER_CONTRACT.md draft + scheduler audit | COMPLETE | 2026-05-09 | Low | Document captures every invariant the code currently relies on; no behaviour change |
| 2 | Scheduler S1, S2, S3 enforcement (input-domain) | COMPLETE | 2026-05-09 | Med | Past-time events rejected; duplicate IDs handled deterministically; queue bounded |
| 3 | Scheduler S4 tightening + S5, S6 enforcement (state-machine) | COMPLETE | 2026-05-09 | Med | Mid-TRANSITIONING replace specified; backward `sim_time` rejected; S4 strict-by-default in tests |
| 4 | PLANT_INTERFACE_CONTRACT.md draft + interface audit | COMPLETE | 2026-05-09 | Low | Document captures aliasing/reset/dt/validation conventions; no behaviour change |
| 5 | PlantInterface P1 + P2 (aliasing + can_reset) | COMPLETE | 2026-05-09 | Low | ABC additions; both `MuJoCoPlant` and `HardwarePlant` updated; contract test parameterized |
| 6 | PlantInterface P3 + P4 (input-validation + control_dt) | COMPLETE | 2026-05-10 | Med | Stale-telemetry thresholds derived from `control_dt`; `command()` validation contract surfaced |
| 7 | K1–K6 hypothesis retroactive expansion | COMPLETE | 2026-05-10 | Low | 4 new properties added to `test_make_feasible_events.py`; existing tests still pass |
| 8 | CI hypothesis profiles wired | COMPLETE | 2026-05-10 | Low | `ci-fast` (50 ex) for per-PR; `ci-deep` (1000 ex) for nightly; documented in CLAUDE.md |

## Implementation Phases (detailed)

### Phase 1: SCHEDULER_CONTRACT.md draft + scheduler audit — COMPLETE (2026-05-09)

**Scope.** Audit the current `controller/scheduler.py` to enumerate every invariant the code actually relies on (explicit or implicit). Draft `controller/SCHEDULER_CONTRACT.md` following the existing template. **No code changes.** This phase is a discovery + design pass; the markdown document is the deliverable.

**New/modified files.**
- `controller/SCHEDULER_CONTRACT.md` (new, ~400 lines following the K1–K6 template)

**Critical details.**

- Document must contain: Background → Invariants (S1–S6) → Enforcement (canonical point) → Implementing-a-new-source template → Diagnosis → Related.
- Each invariant follows the K1–K6 voice: a single normative MUST/MUST NOT statement, a "Why:" paragraph grounded in a specific failure mode, an enforcement pointer.
- **S1 (submission time)** initial draft: every submitted `ScheduledEvent` MUST satisfy `event.time >= t_now - τ_grace` where `τ_grace ≤ 1×control_dt` (one-tick grace for clock skew); past-time events MUST raise or be rejected with a documented sentinel.
- **S2 (unique IDs)** initial draft: `event.event_id` MUST be unique across `_current_event` and `_next_event`; replacement is the documented behaviour (already implemented at `scheduler.py:302–336 — current locations after Phase 2`); attempting to submit an event whose ID matches an in-flight `_current_event` MUST update via `update_current_event` instead of stacking.
- **S3 (bounded in-flight slot set)** initial draft: the in-flight slot set is structurally bounded to 2 `ScheduledEvent` slots (`_current_event`, `_next_event`) + 1 `_QuinticSegment` return slot (`_seg_return`). Today the code happens to enforce this structurally (only 3 slots) — codify it. Future extensions to a deeper queue are explicit contract changes.
- **S4 (internal quintic feasibility)** initial draft: every `_QuinticSegment` constructed by the scheduler MUST pass `_verify_segment_feasibility` (at `scheduler.py:901` (was `:667–717` pre-refactor)). In test contexts (`strict_feasibility=True`) the verifier raises; in production it warns + degrades. Promote the existing toggle to a contract invariant.
- **S5 (C0 continuity at every newly-built segment)** initial draft: every `_QuinticSegment` constructed by the scheduler MUST start at the live motion state (`_last_pose / _last_twist / _last_accel`) at build time. This codifies an existing implementation invariant — guaranteeing C0 (and operationally C1) at every segment splice including IDLE→APPROACHING, HOLDING→TRANSITIONING, the `replace_next_event` mid-TRANSITIONING path, and HOLDING→RETURNING.
- **S6 (clock monotonicity)** initial draft: `update(sim_time)` MUST satisfy `sim_time >= last_sim_time` across calls. Backward jumps are caller bugs (sim restart without `clear()`, timestamp wrap). Reject with `ValueError`; provide `clear()` as the documented reset.

**Dependencies.** None. This phase is doc-only.

**Exit criteria.** Document committed to `refactor` branch. Logbook entry `2026-XX-XX-scheduler-contract-s-phase-1-audit.md` summarises the audit and links to the draft. User signoff at gate.

---

### Phase 2: Scheduler S1, S2, S3 enforcement — COMPLETE (2026-05-09)

**Scope.** Implement the input-domain invariants — those checked at `submit_event()`. These are easy to enforce because the validation point is a single function entry.

**New/modified files.**
- `controller/scheduler.py` — modify `submit_event`, `update_current_event`, `replace_next_event`
- `tests/sim/test_scheduler_contract.py` (new) — initial scenarios for S1, S2, S3
- `tests/sim/test_scheduler.py` — adjust any existing tests that submit past-time events without expecting rejection

**Scope of behaviour change.**

| Invariant | Before | After |
|-----------|--------|-------|
| S1 | `submit_event(event)` accepts any `event.time` | Raises `ValueError` if `event.time < self._last_sim_time - τ_grace` (default `τ_grace = control_dt`) |
| S2 | Duplicate `event_id` matching `_current_event.event_id`: silently replaces `_current_event` reference | Raises `ValueError`; caller must use `update_current_event` for refinements |
| S3 | No slot-occupancy check | Asserts that, after submission, `(_current_event ≠ None) + (_next_event ≠ None) + (_seg_return ≠ None) ≤ 3` |

**Critical details.**

- S1's `τ_grace` should be exposed as a constructor parameter on `EventScheduler` with a documented default, NOT hard-coded. Tests will exercise both paths.
- S2 enforcement must distinguish the legitimate "replace next event" path from the illegitimate "stack two currents" path. Today the code happens to handle this correctly (`scheduler.py:302–336 — current locations after Phase 2` branches on `_current_event is None`); the contract is just verifying it.
- S3 is structural today (only 3 slots) — the assertion is defense-in-depth against future deepening of the queue without an explicit contract update.

**Tests to add (initial; full suite in Phase 4).**
- `T-U-S1-1` past-time event raises `ValueError`
- `T-U-S1-2` event at exactly `t_now - τ_grace` accepted
- `T-U-S2-1` duplicate `event_id` matching current raises `ValueError`
- `T-U-S2-2` event with new `event_id` replaces `_next_event` cleanly
- `T-U-S3-1` submitting a 4th event during a state where 3 slots are filled raises (or merges per documented policy)

**Dependencies.** Phase 1 must be complete (contract document exists; tests reference it).

**Exit criteria.** All new tests pass. Existing scheduler tests pass (with adjustments for any that were submitting past-time events). Full `pytest tests/ -q` green. Logbook entry `2026-XX-XX-scheduler-contract-s1-s3-enforcement.md`. Commit trailer: `Logbook-Entry: 2026-XX-XX-scheduler-contract-s1-s3-enforcement`.

---

### Phase 3: Scheduler S4 tightening + S5, S6 enforcement — COMPLETE (2026-05-09)

**Scope.** State-machine invariants — those checked across `update()` calls or during phase transitions.

**New/modified files.**
- `controller/scheduler.py` — tighten `_verify_segment_feasibility` default; document `replace_next_event` splice; add `sim_time` monotonicity guard at the top of `update`
- `tests/sim/test_scheduler_contract.py` — extend with S4/S5/S6 scenarios

**Behaviour change details.**

| Invariant | Implementation |
|-----------|----------------|
| S4 | Constructor parameter `strict_feasibility` defaults to `True` in test contexts (via `pytest` fixture or env-detection in `__init__`). Production callers in `runner.py` / `zmq_target.py` still pass `False` to preserve graceful degradation. The contract requires strict-by-default in tests, not in production. |
| S5 | Document the existing splice. Add an explicit `_replan_segment_from_live_state(now, current_motion_state, new_event)` private method that captures what `replace_next_event` does today; tests assert C0 continuity at the splice point (no position discontinuity). |
| S6 | At top of `update(sim_time)`: if `sim_time < self._last_sim_time - 1e-6`, raise `ValueError("scheduler sim_time went backward; call clear() before reusing")`. Update `self._last_sim_time = sim_time` after the check. |

**Critical details.**

- S5 splice: when `replace_next_event` is called during `TRANSITIONING`, the existing segment was being followed; the live motion state at the moment of replacement (position, velocity, acceleration) becomes the new segment's `(p0, v0, a0)`. The new segment's terminal `(p1, v1, a1)` is the new event. Duration is `event.time - sim_time`. K2/K3 verification still applies via `_verify_segment_feasibility`.
- S6 must not break sim-restart workflows. The contract is: a restart without `clear()` is a contract violation. Tests must `clear()` between scenarios.

**Tests to add.**
- `T-U-S4-1` segment that violates K2 raises in test context (strict default)
- `T-U-S4-2` same scheduler in production-like context (strict=False) warns + accepts
- `T-U-S5-1` mid-TRANSITIONING replace produces C0-continuous segment at splice point (`|p_segment(t_splice⁻) − p_segment(t_splice⁺)| < 1e-9`)
- `T-U-S5-2` mid-APPROACHING replace defers segment build to next `update()`
- `T-U-S6-1` `update(sim_time)` then `update(sim_time - 0.001)` raises
- `T-U-S6-2` `update(t)` then `clear()` then `update(t')` for `t' < t` accepted

**Dependencies.** Phase 2.

**Exit criteria.** All scheduler-contract tests green. Full suite green. Logbook entry. User signoff.

---

### Phase 4: PLANT_INTERFACE_CONTRACT.md draft + interface audit — COMPLETE (2026-05-09)

**Scope.** Audit `controller/plant.py`, `controller/hardware_plant.py`, and `sim/plant/mujoco_plant.py` for the four invariants. Draft `controller/PLANT_INTERFACE_CONTRACT.md` following the same template. **No code changes.**

**New/modified files.**
- `controller/PLANT_INTERFACE_CONTRACT.md` (new, ~300 lines)

**Critical details.**

- **P1 (PlantState aliasing)** initial draft: `get_state()` MUST return the same `PlantState` instance on every call. Consumers MUST NOT retain references across ticks. Already documented in HOT_LOOP_CONTRACT.md:434–442 — this contract pulls it into the canonical interface document.
- **P2 (can_reset capability)** initial draft: implementations MUST expose `can_reset: bool`. `reset()` is called only when `can_reset is True`; calling it when False MUST raise `NotImplementedError`. Removes the silent no-op at `hardware_plant.py:748` (pre-Phase-5: warn-and-return).
- **P3 (input-validation contract)** initial draft: `command()` is a *trusted-callee* boundary — callers (hot loop) MUST guarantee finite, correctly-shaped inputs. `command()` MUST NOT silently coerce, clip, or default malformed inputs. Validation belongs at the upstream boundary (MPC `solve()` exit, motor guard input). The trusted-callee designation is a deliberate hot-loop choice — defensive validation in `command()` would burn ~100 ns/tick that we don't have.
- **P4 (control_dt awareness)** initial draft: implementations MUST accept `control_dt: float` at construction. Internal time-window thresholds (telemetry staleness, watchdog deadlines) MUST derive from `control_dt`, not hard-code. Default `control_dt=0.025` (40 Hz) preserves current behaviour.

**Dependencies.** None.

**Exit criteria.** Document committed. Logbook entry. User signoff.

---

### Phase 5: PlantInterface P1 + P2 (aliasing + can_reset) — COMPLETE (2026-05-09)

**Scope.** Add the two abstract properties; update `MuJoCoPlant` and `HardwarePlant` to declare them; add a parameterized contract test that exercises both implementations against the invariants.

**New/modified files.**
- `controller/plant.py` — add `can_reset` abstract property
- `controller/hardware_plant.py` — implement `can_reset = False`; raise from `reset()` (was logging a warning and returning)
- `sim/plant/mujoco_plant.py` — implement `can_reset = True`
- `tests/sim/test_plant_interface_contract.py` (new) — parameterized over `[MuJoCoPlant, HardwarePlantStub]`

**Critical details.**

- P1 enforcement: contract test asserts `id(plant.get_state()) == id(plant.get_state())` across N ≥ 100 calls AND that all numeric ndarray fields are the *same* numpy array (not just equal-valued). The aliasing convention is identity, not value.
- P2 behaviour change: `HardwarePlant.reset()` previously logged a warning and returned None. After this phase it raises `NotImplementedError("HardwarePlant: reset is not supported; use the orchestrator for homing")`. **This is a behaviour change** — any caller that calls `reset()` on hardware will now fail loudly. Audit callers: `runner.py:run_mpc_loop()` calls `plant.reset(pose)` only when `pose_6dof is not None`; check whether hardware code ever passes `pose_6dof`. If yes, the call must be gated on `plant.can_reset`.
- `HardwarePlantStub`: a thin in-memory `HardwarePlant` that mocks the ZMQ subsystem (returns synthetic telemetry on demand). Used for the contract test so we don't need a live ZMQ topology in CI. Lives in `tests/sim/_hardware_plant_stub.py`.

**Tests to add.**
- `T-U-P1-1` `id(plant.get_state())` invariant over 100 calls (parameterized)
- `T-U-P1-2` ndarray field identity (`platform_pos_mm` is the same array object) over 100 calls (parameterized)
- `T-U-P2-1` `plant.can_reset` matches expected for each implementation
- `T-U-P2-2` `plant.reset(...)` raises `NotImplementedError` iff `not plant.can_reset`

**Dependencies.** Phase 4.

**Exit criteria.** Both implementations pass the contract test. Existing tests using `MuJoCoPlant` and `HardwarePlant` pass (no regressions). Logbook entry.

---

### Phase 6: PlantInterface P3 + P4 (input-validation + control_dt) — COMPLETE (2026-05-10)

**Scope.** Document P3; implement P4 (`control_dt` parameter and threshold derivation).

**New/modified files.**
- `controller/plant.py` — add `control_dt` abstract property
- `controller/hardware_plant.py` — accept `control_dt` constructor kwarg; derive `_telem_stale_warn_s`, `_telem_stale_hard_s`, `_telem_stale_estop_s` from it
- `sim/plant/mujoco_plant.py` — accept `control_dt` constructor kwarg (default 0.025)
- `controller/runner.py` — pass `control_dt` from `run_mpc_loop` parameter to plant constructor (audit call sites)
- `run_mpc.py` — pass `control_dt` to `HardwarePlant.__init__`
- `sim/main.py` — pass `control_dt` to `MuJoCoPlant.__init__`
- `tests/sim/test_plant_interface_contract.py` — extend with P3/P4 tests

**Behaviour change details.**

- `HardwarePlant`'s magic numbers at `hardware_plant.py:67–71` (`_TELEM_STALE_WARN_S = 0.075`, `_HARD_S = 0.125`, `_ESTOP_S = 0.500`) are replaced with the multiplier expressions derived from `control_dt`. **Same values for default `control_dt=0.025`.** Behaviour change only manifests if a caller passes a different `control_dt`.
- P3 is doc + a defensive contract test that confirms `command()` does *not* silently coerce malformed inputs in the implementations we ship — i.e., a test that calls `plant.command(np.array([np.nan]*6))` and asserts the implementation either raises or passes through (the contract permits either; what's prohibited is silent coercion).

**Tests to add.**
- `T-U-P3-1` `plant.command(np.full(6, np.nan))` does not silently coerce (NaN reaches the ODrive layer or raises)
- `T-U-P4-1` `HardwarePlant(control_dt=0.05)` has staleness thresholds 2× the `control_dt=0.025` values
- `T-U-P4-2` `plant.control_dt` matches the constructor argument

**Dependencies.** Phase 5.

**Exit criteria.** All P-contract tests green. Hardware-side staleness watchdog still fires correctly at the same wall-clock time for default `control_dt`. Logbook entry. Hardware bringup smoke (run `python run_mpc.py --pose 0,0,170,0,0,0 --duration 5` on the Jetson) — confirms P4 wiring.

---

### Phase 7: K1–K6 hypothesis retroactive expansion — COMPLETE (2026-05-10)

**Scope.** Add four new hypothesis properties to the existing K1–K6 surface, mirroring the style at `tests/sim/test_make_feasible_events.py:163` (canonical two-event property) and `:190` (no-stretch rejection property).

**New/modified files.**
- `tests/sim/test_make_feasible_events.py` — extend

**Properties to add.**

| Property | Statement | Strategy |
|----------|-----------|----------|
| `test_property_K4_min_span` | For all proposals where two events have `|t_i - t_j| < 50ms`, the output drops or merges per K4 | `events` strategy that includes near-duplicate times |
| `test_property_K5_coincident_twist` | For all proposals where two events have identical times but different twists, `make_feasible_events` raises `ValueError` | Two events at same time with mismatched twists |
| `test_property_K6_idempotence` | For all valid `(twist, clamp)` pairs, `clamp(clamp(twist)) == clamp(twist)` element-wise | `twist_strategy` × `clamp_strategy` |
| `test_property_K1_K6_multi_event` | For all 3..6 event proposals (today only 2-event tested), the output satisfies K1–K6 | Variable-length event lists |

**Critical details.**

- Use the existing `_pose_strat`, `_twist_strat`, `_T_strat`, `_vmax_strat`, `_tau_strat` from `test_make_feasible_events.py:129-137` as building blocks.
- For multi-event, the strategy must produce strictly increasing event times. Use `st.lists(min_size=3, max_size=6)` then sort + add `_K4_MIN_SPAN_S`.
- Each new property uses `@settings(max_examples=ci_default(), deadline=None)` where `ci_default()` returns 50 in CI-fast and 1000 in CI-deep. This wires into Phase 8.

**Dependencies.** None functional; can run in parallel with Phases 5/6 if desired.

**Exit criteria.** New properties pass at `max_examples=1000`. No new failures discovered (or if discovered, they're either real bugs filed for follow-up or strategy bounds tightened with documented rationale).

---

### Phase 8: CI hypothesis profiles wired — COMPLETE (2026-05-10)

**Scope.** Two hypothesis profiles, integrated with pytest, defaulting to `ci-fast` and selectable via `--hypothesis-profile=ci-deep` for nightly.

**New/modified files.**
- `tests/conftest_hypothesis.py` (new)
- `tests/conftest.py` — import + register profiles
- `pyproject.toml` (existing `[tool.pytest.ini_options]`) — add markers (this repo's pytest config lives in `pyproject.toml`, not `pytest.ini`; the implementation sketch below predates that discovery)
- `CLAUDE.md` — one-line addition under "Tests" describing how to run nightly profile

**Implementation sketch.**

```python
# tests/conftest_hypothesis.py
from hypothesis import settings, HealthCheck

settings.register_profile(
    "ci-fast",
    max_examples=50,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow],
)
settings.register_profile(
    "ci-deep",
    max_examples=1000,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow],
)
settings.register_profile(
    "dev",
    max_examples=200,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow],
)

# Default is ci-fast unless --hypothesis-profile is passed
settings.load_profile("ci-fast")
```

```ini
# pytest.ini (create if missing)
[pytest]
markers =
    slow: tests that take > 30s
    nightly: tests run only in nightly CI
    hypothesis_deep: hypothesis tests benefiting from --hypothesis-profile=ci-deep
```

**CLAUDE.md addition.**

```
### Nightly hypothesis run
pytest tests/ -v --hypothesis-profile=ci-deep   # max_examples=1000
```

**Dependencies.** Phase 7 (uses the profiles).

**Exit criteria.** `pytest tests/ -q` runs with default `ci-fast` (full suite < 5 min). `pytest tests/ -q --hypothesis-profile=ci-deep` completes (no time bound; expected ~30 min). Logbook entry. CLAUDE.md committed.

## Testing Plan

### Unit tests (offline, no hardware)

#### Scheduler contract — `tests/sim/test_scheduler_contract.py`

| ID | Test | Validates | Pass criterion |
|----|------|-----------|----------------|
| T-U-S1-1 | `submit_event(event)` with `event.time < last_sim_time - τ_grace` | S1 | Raises `ValueError` |
| T-U-S1-2 | `submit_event(event)` with `event.time = last_sim_time - τ_grace` | S1 (boundary) | Accepted |
| T-U-S1-3 | `τ_grace` configurable per scheduler instance | S1 | Custom `τ_grace=0.1` allows `event.time = last_sim_time - 0.05` |
| T-U-S2-1 | Submit event with `event_id == _current_event.event_id` | S2 | Raises `ValueError` |
| T-U-S2-2 | `update_current_event(event)` with matching ID is the legitimate path | S2 | Accepted; current refined |
| T-U-S2-3 | Property: hypothesis-generated submission sequences never produce duplicate IDs in `(current, next)` | S2 (state-machine) | Invariant holds |
| T-U-S3-1 | Submit a 4th event when 3 slots filled | S3 | Raises `ValueError` |
| T-U-S3-2 | Property: hypothesis-generated submission/cancel sequences never have `> 3` events in flight | S3 (state-machine) | Invariant holds |
| T-U-S4-1 | Inject K2-violating segment in strict mode | S4 | `_verify_segment_feasibility` raises |
| T-U-S4-2 | Same in non-strict mode | S4 (production) | Logs warning, returns |
| T-U-S4-3 | Property: hypothesis-generated event sequences in strict mode either succeed or raise | S4 | No silent K2/K3 violation |
| T-U-S5-1 | Mid-TRANSITIONING `replace_next_event` produces C0 continuity at splice | S5 | `‖p(t_splice−) − p(t_splice+)‖ < 1e-9` |
| T-U-S5-2 | Mid-APPROACHING replace defers segment build | S5 | `_seg_next` is None until next `update()` |
| T-U-S5-3 | Property: hypothesis state-machine over submit/replace/transition never violates C0 continuity | S5 | Invariant holds |
| T-U-S6-1 | Backward `sim_time` raises | S6 | Raises `ValueError` |
| T-U-S6-2 | After `clear()`, backward `sim_time` accepted | S6 | Accepted |
| T-U-S6-3 | Property: hypothesis-generated `(sim_time, op)` sequences (with `clear()` interleavings) never silently regress time | S6 (state-machine) | Invariant holds |
| | **Implementation note:** ``T-U-S6-3`` shipped as scenario tests in `TestS6ClockMonotonicity` rather than as a state-machine `@invariant`. The state machine's `tick` rule advances `sim_time` strictly forward by a positive `dt`, so backward-time scenarios are unreachable from a state-machine random walk. Targeted backward-jump scenarios live in the explicit class. Rationale documented at `tests/sim/test_scheduler_contract.py:898–903`. | | |

The state-machine tests use `hypothesis.stateful.RuleBasedStateMachine`:

```python
class SchedulerStateMachine(RuleBasedStateMachine):
    @rule(event=event_strategy())
    def submit(self, event): self.scheduler.submit_event(event)
    @rule(event=event_strategy())
    def replace(self, event): self.scheduler.replace_next_event(event)
    @rule(dt=st.floats(0.001, 0.1))
    def tick(self, dt): self.scheduler.update(self._sim_time + dt); self._sim_time += dt
    @invariant()
    def s2_unique_ids(self): assert <ids in current/next are distinct>
    @invariant()
    def s3_bounded(self): assert <≤ 3 events in flight>
    @invariant()
    def s6_monotonic(self): assert self._sim_time >= self._last_sim_time
```

#### PlantInterface contract — `tests/sim/test_plant_interface_contract.py`

Parameterized over `[MuJoCoPlant, HardwarePlantStub]` via `pytest.mark.parametrize`:

| ID | Test | Validates | Pass criterion |
|----|------|-----------|----------------|
| T-U-P1-1 | `id(plant.get_state())` invariant over 100 calls | P1 (instance aliasing) | All IDs equal |
| T-U-P1-2 | ndarray field identity (`platform_pos_mm`, `leg_extensions_mm`, `platform_twist`, etc.) over 100 calls | P1 (field aliasing) | Each field is the same array object every call |
| T-U-P2-1 | `plant.can_reset` returns the expected boolean per implementation | P2 | Hardware: False; MuJoCo: True |
| T-U-P2-2 | `plant.reset(...)` raises `NotImplementedError` iff `not plant.can_reset` | P2 | Behaviour matches capability flag |
| T-U-P3-1 | `plant.command(np.full(6, np.nan))` does not silently coerce | P3 | Either raises or NaN reaches downstream — not silently zeroed |
| T-U-P4-1 | `Plant(control_dt=0.05).control_dt == 0.05` | P4 | Property echoes constructor |
| T-U-P4-2 | `HardwarePlant(control_dt=0.05)._telem_stale_estop_s == 1.0` | P4 | Threshold scaled by 2× |
| T-U-P4-3 | `HardwarePlant(control_dt=0.025)._telem_stale_estop_s == 0.5` | P4 (regression) | Default behaviour preserved |

#### K1–K6 expansion — `tests/sim/test_make_feasible_events.py`

| ID | Test | Validates | Pass criterion |
|----|------|-----------|----------------|
| T-U-K4-1 | Property: events with `|t_i − t_j| < 50ms` are dropped/merged | K4 boundary | Output respects `_K4_MIN_SPAN_S` |
| T-U-K5-1 | Property: coincident events with mismatched twist raise | K5 | `ValueError` with documented message |
| T-U-K6-1 | Property: `clamp(clamp(twist)) == clamp(twist)` | K6 idempotence | Element-wise equality |
| T-U-Kall-1 | Property: 3..6-event proposals satisfy K1–K6 | Multi-event | All invariants hold on output |

### Integration tests (real components, sim only)

| ID | Test | Validates | Pass criterion |
|----|------|-----------|----------------|
| T-I-1 | Full `run_mpc_loop` with new `control_dt` parameter wired through `run_mpc.py` (sim, no hardware) | P4 wiring end-to-end | 5 s sim hold completes without errors |
| T-I-2 | Sim run with scheduler driving 3 events in sequence (APPROACH → HOLD → TRANSITION) | S4 + S5 in production | All segments K1–K6 compliant; no warnings |

### Hardware tests (Jetson, E-stop ready)

| ID | Test | Validates | Pass criterion |
|----|------|-----------|----------------|
| T-H-1 | `python run_mpc.py --pose 0,0,170,0,0,0 --duration 5` after Phase 6 | P4 wiring on hardware | Telemetry-staleness watchdog fires at the same wall-clock time as before (regression check) |
| T-H-2 | Same with `--control-dt 0.05` (if exposed; else verified by HardwarePlant unit test) | P4 derived thresholds | Watchdog fires at 2× the default time |

### Regression tests

| ID | Test | Validates | Pass criterion |
|----|------|-----------|----------------|
| T-R-1 | Full `pytest tests/ -q` after each phase | No collateral breakage | 100% pass |
| T-R-2 | `tests/sim/test_hot_loop_allocation_contract.py` after Phase 5/6 (PlantInterface changes) | Hot-loop budget preserved | `< THRESHOLD_BYTES = 256` |
| T-R-3 | `tests/sim/test_make_feasible_events.py` after Phase 7 expansion | Existing K1–K6 properties preserved | Both old properties still pass at `max_examples=1000` |

### Property-test depth schedule

- Per-PR: `pytest tests/ -q` runs with hypothesis profile `ci-fast` (`max_examples=50`). Total suite target < 5 minutes.
- Nightly (manual or scheduled): `pytest tests/ -q --hypothesis-profile=ci-deep` (`max_examples=1000`). Total suite target < 30 minutes. Failures in this profile that don't reproduce in `ci-fast` are filed as flaky-property issues with the seed pinned.

## Notes for Collaborators

### Safety-critical invariants preserved

| Invariant | Location | Consequence of violation |
|-----------|----------|-------------------------|
| K1–K6 enforcement at `make_feasible_events()` | `controller/target.py:317` | Reference saturation → solver stall → motor jerk |
| Hot-loop allocation budget `256 B/tick` | `controller/hot_loop_contract.py` | GC pause → cmd discontinuity |
| `PlantState` instance aliasing | `controller/hardware_plant.py:488` (`get_state()` returns `self._state`, initialised at `:284`); after Phase 5, formalised as P1 | Cross-tick reference retention by a consumer reads stale data on the next tick |
| Telemetry-staleness watchdog thresholds | `hardware_plant.py:67–71` (after Phase 6: derived from `control_dt`) | Stale telemetry below threshold → MPC commands on stale pose |

### Architecture decisions (non-obvious)

1. **Why P3 documents `command()` as trusted-callee, not defensive.** `command()` is on the hot loop (called every 25 ms). NaN-checking 6 floats per tick costs ~100 ns; 6 shape-checks + 6 finiteness checks costs ~600 ns. At 40 Hz over 1 hour, that's 86 ms of overhead — most of one tick. Validation belongs at the boundaries (MPC `solve()` exit, motor guard input). The contract makes this explicit so a future contributor doesn't add "defensive" checks that silently degrade hot-loop budget.
2. **Why S6 is a hard raise, not a warning.** A scheduler with backward `sim_time` produces negative-duration quintic segments — `_QuinticSegment.duration < 0` makes `_verify_segment_feasibility` undefined. Silent degradation here means silent infeasibility downstream. Better to crash and force the caller to use `clear()`.
3. **Why S2 distinguishes `submit_event` (raise on duplicate ID) from `update_current_event` (replace on matching ID).** The two operations have different semantics: `submit` queues a new event; `update_current_event` refines an in-flight event (live catch tracking). Conflating them would let a tracking glitch silently overwrite an event mid-approach.
4. **Why `_verify_segment_feasibility` strict mode defaults to True in tests, False in production.** In production, a K2/K3 violation should not crash a juggling demo — graceful degradation via W7 fallback is the established policy. In tests, the same violation is a contract bug that must be loud. The fixture-vs-runtime distinction is the right axis.

### Startup / shutdown ordering

No changes to startup ordering. Phase 6 adds a `control_dt` constructor argument; existing call sites that don't pass it use the default (0.025 s) — current behaviour preserved.

### Files affected

| Path | Change | Phase |
|------|--------|-------|
| `controller/SCHEDULER_CONTRACT.md` | Created | 1 |
| `controller/PLANT_INTERFACE_CONTRACT.md` | Created | 4 |
| `controller/scheduler.py` | Modified (S1–S6 enforcement) | 2, 3 |
| `controller/plant.py` | Modified (P2, P4 abstract properties) | 5, 6 |
| `controller/hardware_plant.py` | Modified (can_reset, control_dt, threshold derivation) | 5, 6 |
| `sim/plant/mujoco_plant.py` | Modified (can_reset, control_dt) | 5, 6 |
| `controller/runner.py` | Modified (pass control_dt) | 6 |
| `run_mpc.py` | Modified (pass control_dt) | 6 |
| `sim/main.py` | Modified (pass control_dt) | 6 |
| `tests/sim/test_scheduler_contract.py` | Created | 2, 3 |
| `tests/sim/test_plant_interface_contract.py` | Created | 5, 6 |
| `tests/sim/_hardware_plant_stub.py` | Created (test helper) | 5 |
| `tests/sim/test_make_feasible_events.py` | Modified (4 new properties) | 7 |
| `tests/conftest_hypothesis.py` | Created | 8 |
| `tests/conftest.py` | Modified (load profiles) | 8 |
| `pyproject.toml` | Modified (markers added under `[tool.pytest.ini_options]`) | 8 |
| `.gitignore` | Modified (`.hypothesis/` added next to `.pytest_cache/`) | 8 |
| `CLAUDE.md` | Modified (1 line: nightly profile invocation) | 8 |
| `logbook/2026-XX-XX-...` | Created (one entry per phase) | 1, 2, 3, 4, 5, 6, 7, 8 |

### Rollback plan

Each phase is a single commit (or small commit cluster) on the `refactor` branch. Rollback per phase is `git revert <sha>`. Because each phase is gated by tests and a logbook entry, rollback is well-defined.

The phases are non-cumulative in their behaviour change: rolling back Phase 6 (control_dt wiring) does not require rolling back Phase 5 (can_reset), etc. The contract documents stay landed even if enforcement is reverted — the tests fail loudly without the enforcement, signalling the regression rather than masking it.

If the entire plan needs to revert: `git revert` the contract documents, the test files, and the ABC additions. The behaviour reverts to status quo ante.
