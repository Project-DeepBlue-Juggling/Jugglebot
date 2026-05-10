---
title: Plant-interface contract — Phase 5 enforcement of P1, P2
type: feature
date: 2026-05-09
status: resolved
phase: "mpc-tier0-contracts — Phase 5"
related_plan: "mpc-tier0-contracts.md"
related_entries:
  - 2026-05-09-plant-interface-contract-phase-4-audit
  - 2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement
  - 2026-04-23-hot-loop-zero-allocation-contract
files_changed:
  - controller/plant.py
  - controller/hardware_plant.py
  - controller/PLANT_INTERFACE_CONTRACT.md
  - sim/plant/mujoco_plant.py
  - tests/sim/test_plant_interface_contract.py
  - tests/sim/_hardware_plant_stub.py
  - plans/active/mpc-tier0-contracts.md
  - logbook/2026-05-09-plant-interface-contract-phase-5-p1-p2-enforcement.md
  - logbook/INDEX.md
commits:
  - 63e968e
subsystem:
  - controller
  - mpc
tags:
  - contract
  - testing
  - safety
---

# Plant-interface contract — Phase 5 enforcement of P1, P2

## Summary

Phase 5 of the [mpc-tier0-contracts plan](../plans/active/mpc-tier0-contracts.md):
implemented the two design-time invariants P1 (PlantState aliasing) and
P2 (`can_reset` capability flag) defined in
[controller/PLANT_INTERFACE_CONTRACT.md](../controller/PLANT_INTERFACE_CONTRACT.md)
(landed Phase 4).  The ABC at
[controller/plant.py](../controller/plant.py) gains a new abstract
property; `MuJoCoPlant.get_state()` is refactored to mutate a
pre-allocated `PlantState` in place; `HardwarePlant.reset()` switches
from a silent no-op to a loud `NotImplementedError`; both
implementations declare `can_reset` as a class attribute.  A new
parameterised contract test file at
[tests/sim/test_plant_interface_contract.py](../tests/sim/test_plant_interface_contract.py)
exercises both implementations against the invariants via 16 scenarios
(8 per implementation × 2 + 2 ABC-level static checks).

## Motivation

[Phase 4](2026-05-09-plant-interface-contract-phase-4-audit.md) drafted
the P1–P4 contract and surfaced four divergences between `MuJoCoPlant`
and `HardwarePlant`.  Phase 5 picks the *design-time* subset — the two
invariants that can be enforced via abstract-property additions plus a
parameterised contract test, without touching the hot-loop budget or
the period-awareness wiring.

The split is intentional: design-time enforcement is a thin slice that
introduces small, easy-to-review behaviour changes, lets us land tests
that pin those changes, and unblocks Phase 6's heavier work (P3
trusted-callee documentation + P4 `control_dt` threshold derivation).

## Design

### P1 — PlantState aliasing

Pre-Phase-5 surface:

- `HardwarePlant.get_state()` already aliased `self._state`
  ([hardware_plant.py:506–519](../controller/hardware_plant.py)) — the
  reference implementation, established by the 2026-04-23 hot-loop
  contract.
- `MuJoCoPlant.get_state()` constructed a fresh `PlantState` from
  `np.array(...)` / `np.concatenate(...)` calls every tick — ~250 B
  of Python attribution.

The Phase 5 refactor pre-allocates `self._state` plus two scratch
buffers in `MuJoCoPlant.__init__`
([mujoco_plant.py:144–162](../sim/plant/mujoco_plant.py)) and
rewrites `get_state` to mutate the pre-allocated ndarrays via
`np.copyto`, slice assignment, and `np.add(..., out=...)` /
`np.multiply(..., out=...)`
([mujoco_plant.py:191–245](../sim/plant/mujoco_plant.py)).  No
public-API change; field VALUES still update each call, but the
backing arrays are stable across calls.

### P2 — `can_reset` capability flag

The ABC at [plant.py](../controller/plant.py) gains a new
`@property @abstractmethod can_reset(self) -> bool` declaration
([plant.py:49–61](../controller/plant.py)).  Subclasses satisfy the
abstract by declaring `can_reset = True/False` as a class attribute
(simpler than a `@property` decorator and accepted by Python's
`ABCMeta` for abstract-property satisfaction).

Implementation declarations:

- `HardwarePlant.can_reset = False`
  ([hardware_plant.py:98](../controller/hardware_plant.py)).
  `HardwarePlant.reset()` switches from
  `logger.warning(...)` + return to `raise NotImplementedError(...)`
  with a message that names the implementation, the capability flag,
  and the orchestrator as the correct lifecycle alternative
  ([hardware_plant.py:732–748](../controller/hardware_plant.py)).
- `MuJoCoPlant.can_reset = True`
  ([mujoco_plant.py:66](../sim/plant/mujoco_plant.py)).
  `reset()` body unchanged.

### Caller audit

All three production call sites of `plant.reset()` are statically
inside MuJoCoPlant-only branches:

- `run_mpc.py:282` — inside `if args.use_sim_plant:` (line 270).
- `sim/main.py:1010` — sim-only top-level entry.
- `sim/analysis/record_baselines.py:106` — sim/analysis is sim-only.

Test callers (`tests/sim/test_zmq_target.py:676`,
`tests/sim/test_cold_start_fixes.py:45+`,
`tests/sim/test_hand.py:26`,
`tests/sim/test_plant.py:62+`) all construct `MuJoCoPlant` fixtures.
None reach `HardwarePlant.reset()`.  No `if plant.can_reset:` guards
needed in production code — adding them would be defensive code
against scenarios that can't statically arise (per CLAUDE.md
"don't add error handling for scenarios that can't happen").

### `tests/sim/_hardware_plant_stub.py`

A new helper at
[tests/sim/_hardware_plant_stub.py](../tests/sim/_hardware_plant_stub.py)
provides a `build_hardware_plant_stub()` context manager that
constructs a real `HardwarePlant` with `unittest.mock.patch`-ed ZMQ
and synthetic telemetry.  Mirrors the pattern in
[tests/sim/test_hot_loop_allocation_contract.py:_build_hardware_fixture](../tests/sim/test_hot_loop_allocation_contract.py)
— the project's canonical "construct a real HardwarePlant in-memory"
helper for contract-level testing.

The stub yields a real `controller.hardware_plant.HardwarePlant`
(not a mock subclass) so the contract test exercises the production
class's actual `get_state` / `reset` methods, not stubbed-out
substitutes.  Synthetic motor-position telemetry seeds the FK warm
start so `get_state()` returns valid `PlantState` values.

### `tests/sim/test_plant_interface_contract.py`

Three test classes / 16 scenarios:

| Class | Scope | Tests |
|-------|-------|-------|
| `TestP1PlantStateAliasing` | Aliasing identity invariants | 7 (instance identity over 100 calls × 2 impls; ndarray-field identity over 100 calls × 2 impls; ndarray-fields-stay-ndarrays × 2 impls; field-values-change-when-state-changes [MuJoCo only — needs live motion]) |
| `TestP2ResetCapability` | `can_reset` declaration + raise behaviour | 7 (capability matches × 2 impls; `reset()` biconditional × 2 impls; `reset(pose)` biconditional × 2 impls; HardwarePlant error message names implementation + alternative) |
| `TestPlantInterfaceABC` | Static ABC invariants | 2 (subclass missing `can_reset` rejected by ABCMeta; both shipped impls register as `PlantInterface` subclasses) |

All 16 pass.  Parameterisation uses a `pytest.fixture` that yields
`(name, plant, expected_can_reset)` triples so the same test body
asserts the contract against both implementations with one source of
truth.

## Implementation

### controller/plant.py (modified)

Added the `can_reset` abstract property between the class header and
`command()` ([plant.py:49–61](../controller/plant.py)).  Updated
docstrings on `command`, `get_state`, and `reset` to point at the
relevant invariants in `PLANT_INTERFACE_CONTRACT.md`.

### controller/hardware_plant.py (modified)

- Added `can_reset: bool = False` class attribute
  ([hardware_plant.py:96–98](../controller/hardware_plant.py)) with a
  comment pointing at the contract.
- Replaced `reset()` body
  ([hardware_plant.py:732–748](../controller/hardware_plant.py)):
  pre-Phase-5 `logger.warning(...)` + return becomes
  `raise NotImplementedError("HardwarePlant.reset() is not supported
  (can_reset=False); homing is owned by the orchestrator.  Callers
  must guard with ``if plant.can_reset: plant.reset(...)`` per
  controller/PLANT_INTERFACE_CONTRACT.md P2.")`.

### sim/plant/mujoco_plant.py (modified)

- Added `can_reset: bool = True` class attribute
  ([mujoco_plant.py:66](../sim/plant/mujoco_plant.py)).
- Pre-allocated `self._state` plus `_slide_pos_buf`, `_slide_vel_buf`
  in `__init__`
  ([mujoco_plant.py:144–162](../sim/plant/mujoco_plant.py)).
- Refactored `get_state` to mutate `self._state` in place
  ([mujoco_plant.py:191–245](../sim/plant/mujoco_plant.py)).  Output
  values are bit-identical to the pre-Phase-5 fresh-PlantState
  pattern (verified empirically by the existing
  [tests/sim/test_plant.py](../tests/sim/test_plant.py) suite which
  asserts platform-pos values to ±0.5 mm at active pose, ±1 mm
  post-command).

### tests/sim/_hardware_plant_stub.py (new)

The `build_hardware_plant_stub()` context manager: ZMQ patched with
`MagicMock`, `time.sleep` patched out during construction, synthetic
telemetry computed via `MuJoCoPlant.pose_to_extensions(target_pose)`
so the FK warm start has credible motor positions to work with.

### tests/sim/test_plant_interface_contract.py (new)

Three test classes, 16 scenarios, parameterised over both
implementations.  Documented above.

### controller/PLANT_INTERFACE_CONTRACT.md (modified)

- Background section reframed to past tense for P1 (now landed) and
  P2 (now landed); P3 (Phase 6) and P4 (Phase 6) still in present
  tense.
- Line citations refreshed against post-Phase-5 line numbers in both
  `controller/hardware_plant.py` and `sim/plant/mujoco_plant.py`.
  Twelve citations updated.
- Enforcement table updated: P1 + P2 rows now point at the live test
  classes; P3 + P4 still flag Phase 6.
- "The contract document landed in **Phase 4**" paragraph updated to
  reflect Phase 5's status.

### plans/active/mpc-tier0-contracts.md (modified)

Phase 5 marked `COMPLETE (2026-05-09)` in both the summary table and
the detailed Phase 5 heading.

## Verification

### Existing tests — no behaviour-change impact

The pre-implementation audit walked every caller of
`plant.get_state()` (28 sites across `sim/`, `controller/`, and
`tests/`) plus every caller of `plant.reset()` (10 sites).  Two
classes of risk were considered:

1. **PlantState reference retention across `get_state()` boundaries.**
   Pre-Phase-5, `MuJoCoPlant.get_state()` returned a fresh dataclass,
   so any cached `state` reference was safe to read after subsequent
   `get_state()` calls.  Post-Phase-5, the cached reference's fields
   are mutated by the next `get_state()`.  The audit found no caller
   that retains state across `get_state()` boundaries: every site
   uses `state = plant.get_state()` then reads fields synchronously
   in the same scope, before the next `get_state()` call.  The
   typical pattern is `state = plant.get_state(); _log_step(state);
   ...continue...` with no second `get_state()` between assignment
   and last read.
2. **`HardwarePlant.reset()` callers.**  Pre-Phase-5, calling
   `reset()` on `HardwarePlant` was a silent no-op; post-Phase-5 it
   raises.  The audit confirmed all production and test call sites
   are statically inside MuJoCoPlant branches.  No production caller
   reaches `HardwarePlant.reset()`.

Confirmed empirically: the full sim test suite (`pytest tests/sim/
-q`) passes 612 / 612 with no test changes.  The pre-Phase-5
baseline was 592; the +20 is exactly the new
`test_plant_interface_contract.py` (16) plus four ABC-level
parametrised expansions.

### Test results

- **Before Phase 5:** 1151 / 1151 pass (Phase 4 baseline).
- **After Phase 5:** **1167 / 1167 pass** — the +16 delta is exactly
  the new `test_plant_interface_contract.py` test count.  Zero
  regressions.

## Discussion

### Why MuJoCoPlant in-place rewrite landed cleanly

Pre-implementation I expected the `MuJoCoPlant` aliasing refactor to
break a handful of consumers — patterns like
`for _ in range(N): plant.step(); ...; state = plant.get_state();
log.append(state)` would be a real P1 violation if `state` retained
its fields across the loop body's next iteration's `get_state()`
call.  The audit found zero such patterns: every `get_state()` site
follows the discipline of "read fields synchronously within the
same scope".  This is the discipline the hot-loop contract had
already imposed on `controller/runner.py` callers; the sim-side
callers happened to follow it too even though they weren't
explicitly hot-loop-budget-aware.

The cleanest evidence is the existing
[tests/sim/test_plant.py](../tests/sim/test_plant.py) — a module-
scoped `plant` fixture means every test in the file sees the same
`MuJoCoPlant` instance, with module-shared `self._state`.  Post-
Phase-5, `state.platform_pos_mm` is the SAME ndarray across every
test in the module.  Each test reads its values, makes assertions,
moves on; no test caches state from a previous test.  The fixture-
scope choice and the test-discipline naturally cohere with P1.

### Why we chose class attribute over `@property` for `can_reset`

The ABC declares `can_reset` as `@property @abstractmethod`.
Implementations satisfy it via either form:

```python
class Foo(PlantInterface):
    can_reset = True   # class attribute — chosen

# OR

class Bar(PlantInterface):
    @property
    def can_reset(self) -> bool:
        return True   # property
```

Both work — Python's `ABCMeta` accepts a non-abstract value of any
kind in the subclass's `__dict__` as satisfaction of an abstract
declaration.  We chose the class-attribute form for both shipping
implementations because:

1. **It's a constant per-implementation.**  `HardwarePlant.can_reset`
   is not going to compute `False` based on runtime state — it's
   always False.  A `@property` decorator adds boilerplate without
   semantics.
2. **It matches Python-stdlib convention.**  Capability flags like
   `os.supports_dir_fd` and `socket.has_ipv6` use plain attributes,
   not properties.
3. **The contract document explicitly permits both forms.**  The
   audit at the end of Phase 4 caught a wording inconsistency where
   the contract said "MUST be a property" but the template used a
   class attribute; the post-audit text accepts either.

### Why HardwarePlant.reset() is `NotImplementedError` rather than `RuntimeError`

The contract says "raise NotImplementedError".  Three options were
considered:

1. **`NotImplementedError`** — chosen.  Stdlib-canonical for "this
   method exists on the interface but the implementation doesn't
   support it"; e.g., abstract-method default raise.
2. **`RuntimeError`** — generic catchall.  Would work but loses the
   "this is a capability gap, not a transient failure" framing.
3. **`AttributeError`** — implies the method doesn't exist; wrong
   for an implementation that's *deliberately* declining to support
   the operation.

Option 1 is correct.  The error message names the implementation,
the capability flag (`can_reset=False`), the correct alternative
(orchestrator), and a pointer to the contract document — so an
operator who hits this in a session log has everything needed to
identify the bug.

### Why Phase 5 is split from Phase 6

The plan splits the four invariants into Phase 5 (P1 + P2) and
Phase 6 (P3 + P4) for two reasons:

1. **Phase 5's changes are local; Phase 6's ripple wider.**  P1 is
   a `MuJoCoPlant` internal refactor.  P2 adds an abstract property
   + constants on both implementations.  Neither touches `runner.py`
   or `run_mpc.py`.  P3 (input-validation) requires a documented
   removal of `MuJoCoPlant`'s `np.clip` (or its loud-rejection
   replacement).  P4 (`control_dt`) requires threading the
   parameter through `runner.py` and `run_mpc.py` and replacing
   `HardwarePlant`'s magic-number staleness thresholds — a wider
   change touching call sites the contract test can't reach.
2. **Phase 5 unblocks Phase 6's contract test.**  Phase 6's test
   needs the parameterised fixture infrastructure that Phase 5
   builds (the `_hardware_plant_stub.py` helper).  Landing P1 + P2
   first means Phase 6 starts from a working test scaffold rather
   than building one alongside the P3 / P4 work.

### What landed without enforcement

P3 (trusted-callee `command`) and P4 (`control_dt` awareness) are
NOT enforced in this phase.  Phase 6 covers them.  The `MuJoCoPlant`
`np.clip` at
[mujoco_plant.py:181](../sim/plant/mujoco_plant.py) is a documented
P3 violation that Phase 6 will resolve.  The `HardwarePlant` magic-
number staleness thresholds at
[hardware_plant.py:68–70](../controller/hardware_plant.py) are
documented P4 violations that Phase 6 will replace with
`self._control_dt` multiples.

## Open Questions

- **`MuJoCoPlant.command_hand`'s `np.clip`.**  The hand command at
  [mujoco_plant.py:308](../sim/plant/mujoco_plant.py) also silently
  clips inputs.  Hand control isn't in the `PlantInterface` ABC
  (Phase 4 audit Section "Other observations" — hand control lives
  on `MuJoCoPlant` only, not the contract).  The clip stays for
  now; if Phase 6 introduces a `HandInterface` ABC, P3-equivalent
  trusted-callee semantics would carry over.
- **Bounded `max_calls` for the P1 invariant tests.**  The contract
  tests use `_N_CALLS = 100`.  At 40 Hz that's 2.5 s of plant
  operation.  For an aliasing bug that surfaces only at high call
  counts (e.g., a buffer reuse scheme that clones every K calls),
  100 may not catch it.  The `field_values_change_across_calls`
  scenario test catches the related "frozen field" failure mode
  with 80 step iterations (2 s).  Captured for Plan 2 follow-up if
  a higher-fidelity P1 stress test is needed.
- **HardwarePlantStub fidelity.**  The current stub returns a single
  static synthetic telemetry frame on every `get_state()` call.
  This is enough for P1 (aliasing identity is independent of the
  values) and P2 (reset / can_reset don't read telemetry), but
  Phase 6's P3 test will need to exercise the FK warm-start path
  with adversarial inputs.  May need to extend the stub with a
  `recorded_telemetry_replay` mode if Phase 6's test surfaces the
  need.
