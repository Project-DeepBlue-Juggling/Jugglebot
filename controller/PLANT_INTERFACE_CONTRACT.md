# Plant-Interface Contract (P1–P4)

This document is the **normative specification** of the invariants that
every implementation of [PlantInterface (controller/plant.py)](plant.py)
— and every consumer of one — must satisfy.  It is the structural sibling
of the K1–K6 [REFERENCE_LAYER_CONTRACT.md](REFERENCE_LAYER_CONTRACT.md),
the S1–S6 [SCHEDULER_CONTRACT.md](SCHEDULER_CONTRACT.md), and the
zero-allocation [HOT_LOOP_CONTRACT.md](HOT_LOOP_CONTRACT.md), and uses
the same RFC 2119 normative language (MUST, MUST NOT, MAY).

It exists so that future ``PlantInterface`` implementations (a digital-
twin twin running alongside hardware, a remote-machine plant for
distributed simulation, a mocked plant for hermetic testing) can be
implemented without reintroducing the failure modes that the Phase 4
audit of the [mpc-tier0-contracts plan](../plans/archived/2026-05-10%20mpc-tier0-contracts.md)
identified.

## Background

The MPC consumes plant state via ``plant.get_state()`` and emits
commands via ``plant.command()``.  The plant is the boundary between
the controller's pure-Python world and the physical (or simulated)
robot — every safety-critical signal that the MPC sees, and every
command that drives a real motor, crosses this boundary each tick.

Pre-contract, the ABC at
[plant.py (the four abstract methods ``command`` / ``get_state`` /
``step`` / ``reset``)](plant.py) specified the surface but not the
semantics.  Phase 5 added ``can_reset`` as a fifth abstract property
([plant.py:49–61](plant.py)).  Two concrete implementations exist —
[MuJoCoPlant (sim/plant/mujoco_plant.py)](../sim/plant/mujoco_plant.py)
and
[HardwarePlant (controller/hardware_plant.py)](hardware_plant.py).
Pre-contract they diverged silently on every one of the four
invariants below; Phases 5 and 6 close the divergences:

- **PlantState aliasing** — ``HardwarePlant.get_state()`` has always
  returned the same pre-allocated ``self._state`` instance every call
  (mutated in place; see
  [hardware_plant.py:284–294, :521–535](hardware_plant.py)).
  Pre-Phase-5, ``MuJoCoPlant.get_state()`` constructed a fresh
  ``PlantState`` from fresh ndarrays every call.  Phase 5 brings it
  into compliance — see the post-Phase-5 in-place pattern at
  [mujoco_plant.py:211–265](../sim/plant/mujoco_plant.py) (matches the
  HardwarePlant reference).  Pre-Phase-5, consumers relying on
  aliasing for hot-loop budget compliance
  (see [HOT_LOOP_CONTRACT.md:434–442](HOT_LOOP_CONTRACT.md)) were
  silently broken under sim; consumers retaining references across
  ticks were silently broken under hardware.  The aliasing convention
  lives in HOT_LOOP_CONTRACT.md but wasn't enforced at the ABC level —
  this contract pulls it into the canonical interface document.
- **Reset capability** — pre-Phase-5, ``HardwarePlant.reset()`` was a
  silent no-op with a ``logger.warning``.  Phase 5 lands the loud
  raise (see [hardware_plant.py:748–764](hardware_plant.py)).  A
  caller passing ``pose_6dof`` expecting the platform to move there
  would have been silently ignored pre-Phase-5; post-Phase-5 the
  caller crashes with ``NotImplementedError`` and a pointer to the
  orchestrator.  ``MuJoCoPlant.reset()`` (declared
  [can_reset = True at mujoco_plant.py:75](../sim/plant/mujoco_plant.py))
  honours the call as before.
- **Input-validation** — pre-Phase-6, ``MuJoCoPlant.command()``
  silently clamped out-of-range extensions to
  ``[margin, stroke - margin]`` with a ``logger.warning``.
  ``HardwarePlant.command()`` did no input validation.  The same call
  with the same inputs produced different observable outputs on the
  two implementations — a future contributor who wrote "the plant
  accepts ext_mm and clamps" against MuJoCoPlant was wrong on
  hardware.  Phase 6 resolves: the post-Phase-6
  [MuJoCoPlant.command body at mujoco_plant.py:191–209](../sim/plant/mujoco_plant.py)
  is a pure trusted-callee — no clip, no warn.  Both implementations
  now forward inputs without coercion.
- **Period awareness** — pre-Phase-6, ``HardwarePlant.__init__``
  accepted ``control_dt: float = 0.025`` but used module-level
  hard-coded constants for staleness thresholds that assumed 40 Hz.
  ``MuJoCoPlant.__init__`` did not accept ``control_dt`` at all.  A
  20 Hz / 100 Hz operating regime would have broken the watchdog
  thresholds.  Phase 6 resolves: ``HardwarePlant`` derives staleness
  thresholds from ``self._control_dt`` (see
  [hardware_plant.py:73–75](hardware_plant.py) for the multipliers
  and [:128–132](hardware_plant.py) for the per-instance
  derivation); ``MuJoCoPlant`` now accepts ``control_dt`` (see
  [mujoco_plant.py:117](../sim/plant/mujoco_plant.py)); both
  expose ``control_dt`` as a property.

The P1–P4 invariants below close that whole class of divergence.

## The Invariants

Every ``PlantInterface`` implementation MUST satisfy:

### P1 — PlantState aliasing

``get_state()`` MUST return the same ``PlantState`` instance on every
call.  Each ndarray field on that instance MUST be the same array
object across calls — fields are mutated in place, not rebound.  A
bare ``return PlantState(leg_extensions_mm=np.array(...), ...)`` (the
``MuJoCoPlant`` pattern as of Phase 4) is a contract violation.

Consumers MUST NOT retain ``PlantState`` references across ticks: the
next ``get_state()`` call mutates the fields they hold.  Consumers
needing a snapshot MUST explicitly copy (``state.platform_pos_mm.copy()``
or ``np.copyto(local_buf, state.platform_pos_mm)``).

The aliasing caveat in
[HOT_LOOP_CONTRACT.md:434–442](HOT_LOOP_CONTRACT.md) elaborates the
rationale: fresh PlantState dataclasses per tick produce ~250 B of
tracemalloc attribution per call, which over 40 Hz drives Gen-2 GC
pressure and 40–80 ms pauses.

**Why.** The 2026-04-23 hot-loop audit (see
[logbook/2026-04-23-hot-loop-zero-allocation-contract.md](../logbook/2026-04-23-hot-loop-zero-allocation-contract.md))
measured ~19 KB of Python allocation per pre-contract tick, of which
the per-tick PlantState dataclass plus its six ndarray fields was a
named contributor.  Aliasing eliminates that source.

The budget enforcement test
([tests/sim/test_hot_loop_allocation_contract.py](../tests/sim/test_hot_loop_allocation_contract.py))
already runs against both implementations — the default test against
``MuJoCoPlant`` plus a sibling at
[line 450](../tests/sim/test_hot_loop_allocation_contract.py) that
patches ZMQ for ``HardwarePlant`` — and ``MuJoCoPlant``'s per-tick
``PlantState`` allocations are not filtered out (the filter excludes
only ``sim/analysis/``, ``sim/viz/``, and ``matplotlib/``).  The
budget test fits within the 256 B/tick threshold by margin, not by
exclusion.  What's missing is a *design-time* contract: a third
``PlantInterface`` implementation gets no warning until its first
budget-test run, and the aliasing convention lives in
HOT_LOOP_CONTRACT.md rather than in the interface itself.  Lifting it
into P1 means future implementations honour the convention from
construction time, by reading the contract, not by archaeology
through HOT_LOOP_CONTRACT.md.

### P2 — Reset capability

Every implementation MUST expose ``can_reset: bool`` as a class
attribute or property.  ``can_reset`` is False when ``reset()``
cannot meaningfully execute
(e.g., hardware whose homing is owned by the orchestrator), True when
``reset()`` can return the plant to a known state.

When ``can_reset is True``, ``reset()`` MUST execute the documented
reset (return to home, or to a specified ``pose_6dof``).

When ``can_reset is False``, ``reset()`` MUST raise
``NotImplementedError`` with a message naming the implementation and
pointing at the correct lifecycle API.  Silent no-ops with a
``logger.warning`` (the pre-Phase-5 ``HardwarePlant.reset()`` body)
are a contract violation; the post-Phase-5 form at
[hardware_plant.py:748–764](hardware_plant.py) raises with an
operator-actionable message.

Callers that may run against either implementation MUST guard with
``if plant.can_reset: plant.reset(...)``.  Code that unconditionally
calls ``reset()`` MUST document its plant-implementation assumption.

**Why.** Pre-contract, ``HardwarePlant.reset(pose)`` logged a warning
and returned None.  A test or migration script that called
``plant.reset(pose)`` expected the platform to move; on hardware it
silently didn't.  The warning is in the log, but the caller's control
flow continues as if the reset succeeded — and the next ``command()``
runs on whatever pose the platform was actually at, not the one the
caller asked for.  A loud ``NotImplementedError`` is the correct
behaviour: callers either honour ``can_reset`` and handle the
no-reset case (the hardware-aware path uses the orchestrator for
homing) or they crash and surface the assumption.  Either way the
silent divergence between sim and hardware is gone.

### P3 — Trusted-callee command boundary

``command()`` is a *trusted-callee* boundary.  Callers (the MPC hot
loop) MUST guarantee finite, correctly-shaped inputs:
- ``leg_extensions_mm`` MUST be an ``ndarray`` of shape ``(6,)`` with
  finite floats.
- Optional ``vel_mm_s``, ``cmd_next_mm``, ``cmd_next2_mm`` MUST be
  None or shape ``(6,)`` finite floats.
- All values MUST be inside the implementation's mechanical envelope
  (``[margin, stroke - margin]`` for the current Stewart hardware).

Implementations MUST NOT silently coerce, clip, default, or sanitise
malformed inputs.  Specifically:
- An out-of-range extension MUST NOT be silently clamped.  The
  pre-Phase-6 ``MuJoCoPlant`` ``np.clip`` at the head of
  ``command()`` was the canonical example; Phase 6 removed it.
  The post-Phase-6 ``command`` body at
  [mujoco_plant.py:191–209](../sim/plant/mujoco_plant.py) is a pure
  trusted-callee path matching ``HardwarePlant``.
- A NaN or inf MUST NOT be silently zeroed or replaced.
- A wrong-shape array MUST NOT be silently broadcast or reshaped.

Implementations MAY raise ``ValueError`` on out-of-range inputs (the
defensive option), or MAY pass the bad values through to the layer
below (the trusting option that ``HardwarePlant`` takes — bad values
flow to motor_guard, which has its own workspace-and-overspeed
guards).  Both are P3-compliant.  What's prohibited is silent
correction, because the correction hides the upstream caller bug
that produced the malformed input.

Validation belongs at the upstream boundary: MPC ``solve()``'s exit
clamps the solver output to the actuator envelope; the motor guard's
500 Hz safety pipeline re-clamps before driving the ODrives.  By the
time inputs reach ``plant.command()``, they have already been
validated twice.

**Why.** ``command()`` is on the 40 Hz hot loop (called every 25 ms).
Defensive validation on 6 floats per tick costs ~100 ns; full
shape-and-finiteness checks on every optional arg cost ~600 ns.  At
40 Hz over 1 hour that's 86 ms — almost a full tick, burned on
re-validating inputs that the MPC and motor guard have already
validated.

The contract makes the trusted-callee designation explicit so a future
contributor who notices "this method doesn't check its inputs" doesn't
add "defensive" validation that silently degrades the hot-loop budget.
And the prohibition on silent correction means the divergence between
``MuJoCoPlant`` (clamps) and ``HardwarePlant`` (passes through) is
eliminated: a future fix lands by removing the clamp, not by adding
one.

### P4 — control_dt awareness

Every implementation MUST accept ``control_dt: float`` as a
constructor keyword argument.  The default MUST be ``0.025`` (40 Hz)
to preserve the current operating point as the unchanged-default.

Internal time-window thresholds (telemetry-staleness watchdogs,
deadlock deadlines, dead-band re-arm intervals) MUST be derived from
``control_dt`` rather than hard-coded.  The current
``HardwarePlant`` constants
([hardware_plant.py:73–75](hardware_plant.py)) —

    _TELEM_STALE_WARN_S  = 0.075   # 3x MPC period — log warning
    _TELEM_STALE_HARD_S  = 0.125   # 5x MPC period — zero velocities
    _TELEM_STALE_ESTOP_S = 0.5     # 20x MPC period — telemetry definitely lost

— are correct for ``control_dt=0.025`` only.  The post-Phase-6
shape derives them from ``self._control_dt``:

    self._telem_stale_warn_s  =  3.0 * self._control_dt
    self._telem_stale_hard_s  =  5.0 * self._control_dt
    self._telem_stale_estop_s = 20.0 * self._control_dt

Implementations MUST expose ``control_dt`` as a read-only property so
external consumers (the runner, the scheduler's S1 ``τ_grace`` default
which today derives from
[scheduler.py:238–244](scheduler.py)) can read the canonical period
from one source.

**Why.** Pre-contract, the staleness thresholds in
``HardwarePlant`` were hard-coded magic numbers with comments naming
them as multiples of the (then-implicit) MPC period.  A future
operating regime — slower control loop for catch-only sequences,
faster loop for ball-tracking telemetry, dynamic regime switching —
would silently break the watchdog: the 75 ms warn threshold would
fire on every tick at 10 Hz (100 ms period) and never fire at 100 Hz
(10 ms period), in both cases without anyone noticing the magic
number drifted from the operating point.

P4 also unifies the source of truth for ``control_dt``.  Today the
scheduler derives ``τ_grace`` from
``cumulative_times[1] - cumulative_times[0]``, which is the MPC's
horizon discretisation, not its actuation period.  These happen to
match in the current configuration but need not — and the divergence
is one rename away from a silent contract violation.  Making
``plant.control_dt`` the single source means every consumer that
needs the period reads it from the plant (the boundary that owns the
real wall-clock relationship), not from inferred metadata elsewhere.

## Enforcement

All four invariants are enforced at the canonical implementation:
[controller/plant.py](plant.py) (the ABC) plus
[controller/hardware_plant.py](hardware_plant.py) and
[sim/plant/mujoco_plant.py](../sim/plant/mujoco_plant.py) (the two
implementations).  The enforcement points are:

| Invariant | ABC surface | Enforcement test |
|-----------|-------------|------------------|
| P1 | ``get_state() -> PlantState`` (existing); contract test asserts ``id(plant.get_state())`` invariant + ndarray-field identity over 100 calls | ``tests/sim/test_plant_interface_contract.py::TestP1PlantStateAliasing`` (Phase 5, landed) — parameterised over ``[MuJoCoPlant, HardwarePlant via _hardware_plant_stub]`` |
| P2 | New abstract property ``can_reset: bool`` (Phase 5, landed at [plant.py:49–61](plant.py)) | ``TestP2ResetCapability`` asserts ``plant.reset() raises NotImplementedError iff not plant.can_reset``, plus ``TestPlantInterfaceABC::test_can_reset_is_abstract_on_the_abc`` enforces declaration |
| P3 | Documented on ``command()`` (Phase 4 / 6, landed at [plant.py:81–105](plant.py)).  ``MuJoCoPlant.command`` ``np.clip`` removed at Phase 6 — both implementations now trusted-callee. | ``TestP3TrustedCallee`` feeds NaN / out-of-range / negative inputs and asserts no silent coercion (parameterised over both impls) |
| P4 | New abstract property ``control_dt: float`` (Phase 6, landed at [plant.py:64–79](plant.py)); constructor kwarg on both impls | ``TestP4ControlDtAwareness`` asserts ``plant.control_dt`` echoes constructor; ``HardwarePlant`` staleness thresholds scale linearly with ``control_dt``; default-period thresholds reproduce the pre-Phase-6 magic numbers |

The contract document landed in **Phase 4** is the discovery + design
pass.  **Phase 5** lands the P1 + P2 enforcement (abstract additions
+ parameterised contract test).  **Phase 6** (this commit) lands P3
+ P4: ``np.clip`` removed from ``MuJoCoPlant.command``;
``control_dt`` plumbed through to staleness thresholds; ABC gains
``control_dt`` abstract property; contract tests extended.

## Implementing a new ``PlantInterface``

A new ``PlantInterface`` implementation is any subclass of
[plant.py:PlantInterface](plant.py) that exposes the four required
abstract methods plus the post-Phase-5/6 ``can_reset`` and
``control_dt`` properties.  Examples:

- A ``RemotePlant`` that proxies a Jetson plant over an SSH tunnel.
- A ``DigitalTwinPlant`` that runs a MuJoCo simulation alongside the
  real hardware for state estimation.
- A ``MockPlant`` for hermetic CI testing without MuJoCo at all.

Implementations MUST:

1. **Honour P1.**  Pre-allocate a single ``PlantState`` instance plus
   its ndarray fields in ``__init__``.  ``get_state()`` mutates
   fields in place via ``np.copyto`` / ``np.divide(..., out=...)`` /
   slice assignment.  Document the aliasing in the docstring so
   consumers know not to retain references.

2. **Honour P2.**  Declare ``can_reset`` as a class attribute or
   property at construction.  If False, ``reset()`` raises
   ``NotImplementedError`` with a message naming the implementation
   and pointing at the correct alternative API.

3. **Honour P3.**  Treat ``command()`` inputs as trusted.  If any
   defensive validation is needed (e.g., a development-mode plant
   that prefers to crash on bad inputs rather than corrupt state),
   raise ``ValueError`` rather than silently correcting.  Do not
   ``np.clip``, do not zero NaNs, do not reshape.

4. **Honour P4.**  Accept ``control_dt: float = 0.025`` in
   ``__init__``.  Store as ``self._control_dt`` and expose as a
   read-only ``control_dt`` property.  Derive any time-window
   thresholds from ``self._control_dt``, not from hard-coded
   constants.

Template:

```python
import numpy as np
from controller.plant import PlantInterface, PlantState

class MyPlant(PlantInterface):
    can_reset = True   # or False if reset() is unsupported

    def __init__(self, *, control_dt: float = 0.025, **kw):
        self._control_dt = float(control_dt)
        # Pre-allocate the per-tick PlantState (P1 aliasing).
        self._state = PlantState(
            leg_extensions_mm=np.zeros(6),
            leg_velocities_mmps=np.zeros(6),
            platform_pos_mm=np.zeros(3),
            platform_rot=np.zeros(3),
            platform_twist=np.zeros(6),
            time=0.0,
        )
        # Derive time windows from control_dt (P4).
        self._stale_warn_s = 3.0 * self._control_dt

    @property
    def control_dt(self) -> float:
        return self._control_dt

    def get_state(self) -> PlantState:
        # P1: mutate self._state in place; do NOT return a fresh dataclass.
        np.copyto(self._state.leg_extensions_mm, ...)
        self._state.time = ...
        return self._state

    def command(self, leg_extensions_mm, vel_mm_s=None,
                cmd_next_mm=None, cmd_next2_mm=None) -> None:
        # P3: trust the inputs; do NOT clip, validate, or coerce.
        ...

    def step(self, dt: float) -> None:
        ...

    def reset(self, pose_6dof: np.ndarray | None = None) -> None:
        if not self.can_reset:
            raise NotImplementedError(
                f"{type(self).__name__}: reset is not supported; "
                "use <implementation-specific lifecycle API>."
            )
        ...
```

## Diagnosis

If a ``PlantInterface``-related symptom surfaces in a session log:

1. **Hot-loop allocation regression**
   ([tests/sim/test_hot_loop_allocation_contract.py](../tests/sim/test_hot_loop_allocation_contract.py)
   fails with > 256 B/tick) — first suspect a P1 violation.  Grep the
   plant under test for ``return PlantState(...)`` (fresh dataclass)
   or ``np.array(...)`` inside ``get_state()`` (fresh ndarrays).
   ``HardwarePlant.get_state()``'s pattern at
   [hardware_plant.py:521–535](hardware_plant.py) is the reference.

2. **A consumer reads stale plant state across ticks** — a cached
   ``state = plant.get_state()`` reference that's read on a later
   tick.  Per P1, fields have been overwritten by the next call.
   Audit for ``self._cached_state = plant.get_state()`` patterns and
   replace with explicit field copies (``np.copyto(self._buf,
   state.platform_pos_mm)``).

3. **A test passes against MuJoCoPlant but fails on hardware** —
   one of:
   - P1: the test retains a ``PlantState`` reference and reads stale
     fields after a ``get_state()`` call (passes on sim because sim
     returns fresh state every call; fails on hardware because the
     same instance is reused).
   - P3: the test feeds out-of-range extensions expecting them to
     come out clamped (passes on sim because MuJoCoPlant clips,
     fails on hardware because HardwarePlant doesn't).
   Both are real divergences caught by the parameterised contract
   test at
   [tests/sim/test_plant_interface_contract.py](../tests/sim/test_plant_interface_contract.py)
   (Phase 5/6).

4. **Watchdog fires too early or too late at a non-default
   control_dt** — P4 violation.  Grep the plant for hard-coded
   second-magnitudes.  In ``HardwarePlant``, the post-Phase-6 form
   at [hardware_plant.py:128–132](hardware_plant.py) derives
   ``self._telem_stale_*_s`` from
   [the multipliers at hardware_plant.py:73–75](hardware_plant.py)
   times ``self._control_dt`` — any new threshold should follow
   the same pattern.

5. **``plant.reset()`` returned but the platform didn't move** —
   P2 violation.  Pre-Phase-5, ``HardwarePlant.reset()`` was a
   silent no-op (the symptom).  Post-Phase-5 it raises
   ``NotImplementedError`` instead, so this symptom should not
   recur for current code; it remains a useful diagnosis for any
   future implementation that regresses to a silent no-op.  Audit
   for unconditional ``plant.reset()`` calls; gate them with
   ``if plant.can_reset:``.

## Related

- [REFERENCE_LAYER_CONTRACT.md](REFERENCE_LAYER_CONTRACT.md) — K1–K6
  reference-feasibility contract.  Orthogonal to this contract:
  K1–K6 governs *what* trajectory the MPC tracks; P1–P4 governs the
  *interface* through which the MPC reads plant state and writes
  commands.
- [SCHEDULER_CONTRACT.md](SCHEDULER_CONTRACT.md) — S1–S6 scheduler
  contract.  P4 will be the source of truth for the period that
  S1's ``τ_grace`` default derives from (currently derived from the
  MPC's horizon discretisation; Phase 6 unifies them).
- [HOT_LOOP_CONTRACT.md](HOT_LOOP_CONTRACT.md) — hot-loop
  zero-allocation contract.  P1 lifts the aliasing requirement from
  HOT_LOOP_CONTRACT.md:434–442 into the canonical interface
  document.  P3's trusted-callee designation is the
  hot-loop-budget-aware choice.
- [controller/plant.py](plant.py) — the abstract base class.
- [controller/hardware_plant.py](hardware_plant.py) — the hardware
  implementation; the P1 / P3 reference and the P2 / P4 violator.
- [sim/plant/mujoco_plant.py](../sim/plant/mujoco_plant.py) — the
  simulation implementation; the P2 reference and the P1 / P3 / P4
  violator.
- [tests/sim/test_plant_interface_contract.py](../tests/sim/test_plant_interface_contract.py)
  — enforcement tests (Phases 5 / 6 of the
  [mpc-tier0-contracts plan](../plans/archived/2026-05-10%20mpc-tier0-contracts.md)).
- [plans/archived/2026-05-10 mpc-tier0-contracts.md](../plans/archived/2026-05-10%20mpc-tier0-contracts.md)
  — phased implementation plan; this document is the Phase 4
  deliverable.
- [logbook/2026-05-09-plant-interface-contract-phase-4-audit.md](../logbook/2026-05-09-plant-interface-contract-phase-4-audit.md)
  — Phase 4 audit and contract draft.
