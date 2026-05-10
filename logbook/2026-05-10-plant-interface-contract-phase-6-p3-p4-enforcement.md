---
title: Plant-interface contract — Phase 6 enforcement of P3, P4
type: feature
date: 2026-05-10
status: resolved
phase: "mpc-tier0-contracts — Phase 6"
related_plan: "mpc-tier0-contracts.md"
related_entries:
  - 2026-05-09-plant-interface-contract-phase-5-p1-p2-enforcement
  - 2026-05-09-plant-interface-contract-phase-4-audit
  - 2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement
  - 2026-04-23-hot-loop-zero-allocation-contract
files_changed:
  - controller/plant.py
  - controller/hardware_plant.py
  - controller/PLANT_INTERFACE_CONTRACT.md
  - sim/plant/mujoco_plant.py
  - run_mpc.py
  - sim/main.py
  - tests/sim/test_plant_interface_contract.py
  - plans/archived/2026-05-10 mpc-tier0-contracts.md
  - logbook/2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement.md
  - logbook/INDEX.md
commits:
  - e296a82
subsystem:
  - controller
  - mpc
tags:
  - contract
  - testing
  - safety
---

# Plant-interface contract — Phase 6 enforcement of P3, P4

## Summary

Phase 6 of the [mpc-tier0-contracts plan](../plans/archived/2026-05-10%20mpc-tier0-contracts.md):
implemented the two remaining invariants P3 (trusted-callee
``command()``) and P4 (``control_dt`` awareness) defined in
[controller/PLANT_INTERFACE_CONTRACT.md](../controller/PLANT_INTERFACE_CONTRACT.md)
(landed Phase 4).  Three concrete changes:

1. ``MuJoCoPlant.command()`` no longer silently clips out-of-range
   extensions to ``[margin, stroke - margin]``.  The ``np.clip`` +
   ``logger.warning`` block is gone; ``command()`` now forwards
   inputs to MuJoCo without coercion, matching ``HardwarePlant``'s
   trusted-callee pattern.
2. ``HardwarePlant``'s telemetry-staleness watchdog thresholds are
   no longer module-level magic numbers.  They derive from
   ``self._control_dt`` via per-instance multipliers, so the
   watchdog stays correct at any operating regime.  At the default
   ``control_dt=0.025`` the values reproduce the pre-Phase-6
   numbers (0.075 / 0.125 / 0.5 s) exactly.
3. ``PlantInterface`` ABC gains a new abstract property
   ``control_dt: float``.  Both implementations declare it; the
   runner / ``run_mpc.py`` / ``sim/main.py`` thread the parameter
   to plant constructors.

A new 13 contract scenarios extend
[tests/sim/test_plant_interface_contract.py](../tests/sim/test_plant_interface_contract.py)
to exercise both implementations against P3 and P4: NaN / out-of-
range / negative inputs (P3, parameterised over both impls) and
default + explicit ``control_dt`` + threshold-scaling (P4).

## Motivation

[Phase 4](2026-05-09-plant-interface-contract-phase-4-audit.md) drafted
the P1–P4 contract.  [Phase 5](2026-05-09-plant-interface-contract-phase-5-p1-p2-enforcement.md)
landed P1 (PlantState aliasing) + P2 (``can_reset`` capability).
Phase 6 closes the remaining two invariants.

The phase-5/6 split is intentional: P1+P2 are design-time invariants
that landed via thin abstract additions and one in-place refactor
in ``MuJoCoPlant.get_state()``.  P3+P4 ripple wider — P3 changes
``MuJoCoPlant.command()``'s observable behaviour (out-of-range values
no longer come back clamped), and P4 adds a new constructor kwarg
that flows through ``run_mpc.py`` / ``sim/main.py`` to both plant
constructors and replaces module-level constants in ``HardwarePlant``
with per-instance attributes.  Splitting them lets each phase ship
under its own contract test gate without compounding behaviour
changes.

## Design

### P3 — trusted-callee ``command()``

The pre-Phase-6 ``MuJoCoPlant.command()`` body silently clipped
extensions to ``[margin, stroke - margin]`` and logged a warning
when the clip fired:

```python
ext = np.asarray(leg_extensions_mm, dtype=float)
stroke = self._geom.leg_stroke_mm
lo = self._cmd_margin_mm
hi = stroke - self._cmd_margin_mm
ext_clamped = np.clip(ext, lo, hi)
if not np.allclose(ext, ext_clamped, atol=0.1):
    logger.warning("command() clamped extensions: ...")
slide_m = self._extensions_to_slide(ext_clamped)
self._data.ctrl[:6] = slide_m
```

The post-Phase-6 form is a pure trusted-callee path — no clip, no
warn, no defensive validation:

```python
ext = np.asarray(leg_extensions_mm, dtype=float)
slide_m = self._extensions_to_slide(ext)
self._data.ctrl[:6] = slide_m
```

The MuJoCo model's slide-joint range provides the physics-layer
envelope at the actuator level; an out-of-range extension produces
saturated joint targets that MuJoCo handles correctly without Python
intervention.

The ``cmd_margin_mm`` constructor kwarg is retained as inert state
(not removed) so existing callers that pass ``MuJoCoPlant(cmd_margin_mm=…)``
don't break; the field is no longer read by ``command()``.  A
follow-up cleanup can remove the kwarg once all callers stop passing
it.

### P4 — ``control_dt`` awareness

Three sub-changes:

**4a. ABC abstract property** — ``PlantInterface`` gains a new
``@property @abstractmethod control_dt(self) -> float`` declaration
([plant.py:64–79](../controller/plant.py)).  Subclasses satisfy the
abstract via a normal ``@property`` decorator on a method that
returns ``self._control_dt``.

**4b. ``HardwarePlant`` per-instance staleness thresholds** — the
pre-Phase-6 module-level constants

```python
_TELEM_STALE_WARN_S = 0.075   # 3x MPC period
_TELEM_STALE_HARD_S = 0.125   # 5x MPC period
_TELEM_STALE_ESTOP_S = 0.5    # 20x MPC period
```

become per-instance attributes derived from ``self._control_dt``:

```python
self._telem_stale_warn_s  =  3.0 * self._control_dt   # multiplier at module
self._telem_stale_hard_s  =  5.0 * self._control_dt
self._telem_stale_estop_s = 20.0 * self._control_dt
```

The multipliers themselves stay at module level
([hardware_plant.py:73–75](../controller/hardware_plant.py)) — they
are dimensionless physical constants ("3 ticks of clock-skew
tolerance"), not operating-regime magic numbers.  The instance
attributes are computed once at construction
([hardware_plant.py:128–132](../controller/hardware_plant.py)) and
read in ``get_state()`` at three sites
([:627, :686, :695](../controller/hardware_plant.py)) — same call
sites as the pre-Phase-6 module-constant reads.

**4c. ``MuJoCoPlant`` ``control_dt`` kwarg** — ``MuJoCoPlant.__init__``
gains ``control_dt: float = 0.025`` as a keyword argument
([mujoco_plant.py:82](../sim/plant/mujoco_plant.py)).  ``MuJoCoPlant``
itself doesn't have control-period-derived thresholds today (the
sim doesn't run a telemetry-staleness watchdog), but external
consumers — the runner, the scheduler's S1 ``τ_grace`` — read
``plant.control_dt`` for the canonical period.  Storing the value
satisfies the abstract and makes the consumer-side wiring possible.

### Caller threading

Two production call sites construct ``MuJoCoPlant`` and run a
control loop against it:

- [run_mpc.py:275](../run_mpc.py) — sim dry-run path.
- [sim/main.py:1306](../sim/main.py) — main MPC sim entry point.

Both updated to pass ``control_dt=CONTROL_DT`` explicitly.  The
other ``MuJoCoPlant()`` construction sites (``generate_solver.py``,
``demo_mpc.py``, ``record_baselines.py``) don't run a control loop
— they construct plants for one-shot IK conversion or kinematics
analysis — so the default ``control_dt=0.025`` is correct without
explicit passing.

### Cross-contract refactor (deferred)

The Phase 4 audit identified a cross-contract opportunity: the
scheduler's S1 ``τ_grace`` default currently derives from
``cumulative_times[1] - cumulative_times[0]`` (the MPC horizon
discretisation, see
[scheduler.py:238–244](../controller/scheduler.py)).  These happen
to match ``control_dt=0.025`` in the current configuration but need
not — the divergence is one rename away from a silent contract
violation.

Phase 6 lands the property-side of the unification: every
``PlantInterface`` implementation now exposes ``control_dt`` as a
read-only property.  Test fixtures and future callers that
construct an ``EventScheduler`` can pass
``tau_grace_s=plant.control_dt`` to thread the canonical value.
The full unification — scheduler-side automatic derivation from a
plant reference — is deferred to Plan 2 (``mpc-sadpath-coverage``)
because it requires either coupling the scheduler to a plant
instance or introducing a third "control config" surface; that
design choice is out of P1–P4's scope.

## Implementation

### controller/plant.py (modified)

Added the ``control_dt`` abstract property at
[plant.py:64–79](../controller/plant.py).  Updated the module
docstring at the top to mention the new abstract.

### controller/hardware_plant.py (modified)

- Replaced the three module-level ``_TELEM_STALE_*_S`` constants
  with three ``_TELEM_STALE_*_MULT`` multipliers
  ([hardware_plant.py:73–75](../controller/hardware_plant.py)).
- Added per-instance ``self._telem_stale_*_s`` attributes derived
  from ``self._control_dt`` × multiplier
  ([hardware_plant.py:128–132](../controller/hardware_plant.py)).
- Replaced the three module-constant reads in ``get_state()``
  ([:627, :630, :686, :693, :695, :699](../controller/hardware_plant.py))
  with ``self._telem_stale_*_s`` reads.
- Added ``control_dt`` property
  ([hardware_plant.py:963–967](../controller/hardware_plant.py))
  satisfying the abstract.
- ``self._control_dt = float(control_dt)`` (was unboxed; now
  explicit cast for clarity and to match HardwarePlant Phase-3
  pattern).

### sim/plant/mujoco_plant.py (modified)

- Added ``control_dt: float = 0.025`` to ``__init__`` signature
  ([mujoco_plant.py:82](../sim/plant/mujoco_plant.py)).
- Added ``self._control_dt = float(control_dt)`` storage
  ([mujoco_plant.py:117](../sim/plant/mujoco_plant.py)) with a
  docstring comment naming P4.
- Removed the ``np.clip`` block from ``command()``
  ([mujoco_plant.py:191–209](../sim/plant/mujoco_plant.py));
  signature unchanged, body simplified to the trusted-callee form.
- Added ``control_dt`` property
  ([mujoco_plant.py:393–397](../sim/plant/mujoco_plant.py))
  satisfying the abstract.
- Updated the class docstring to document the new ``control_dt``
  kwarg and to note that ``cmd_margin_mm`` is now inert.

### run_mpc.py (modified)

[run_mpc.py:275](../run_mpc.py) now passes ``control_dt=CONTROL_DT``
to the ``MuJoCoPlant`` constructor in the sim-dry-run branch.

### sim/main.py (modified)

[sim/main.py:1306](../sim/main.py) now passes
``control_dt=CONTROL_DT`` to the ``MuJoCoPlant`` constructor in the
non-hardware branch.

### tests/sim/test_plant_interface_contract.py (modified)

Added 13 new scenarios across two new test classes plus one
extension to ``TestPlantInterfaceABC``:

| Class | Scope | Tests |
|-------|-------|-------|
| ``TestP3TrustedCallee`` | ``command()`` does not silently coerce | 6 (NaN × 2 impls; out-of-range × 2 impls; negative × 2 impls) |
| ``TestP4ControlDtAwareness`` | ``control_dt`` parameterisation | 6 (default 0.025 × 2 impls; explicit MuJoCoPlant; explicit HardwarePlant; staleness scaling 2× at 20 Hz; default thresholds match pre-Phase-6 magic numbers) |
| ``TestPlantInterfaceABC`` | +1 (control_dt is abstract on the ABC) | 1 |

The P3 helper ``_read_forwarded_extensions(name, plant)`` reads the
"what got forwarded" snapshot from each implementation's distinct
surface — ``self._data.ctrl[:6]`` (MuJoCo) or ``self._cmd_ext_buf``
(Hardware) — so the same test body asserts the no-coercion property
on both.

### controller/PLANT_INTERFACE_CONTRACT.md (modified)

- Background section reframed past-tense for P3 and P4 (now landed).
- Twelve line citations refreshed against post-Phase-6 line numbers.
- Enforcement table: P3 + P4 rows now point at the live test
  classes; closing paragraph updated to reflect Phase 6's status.
- "An out-of-range extension MUST NOT be silently clamped" P3
  bullet rewritten to past-tense ("the pre-Phase-6 ``MuJoCoPlant``
  ``np.clip`` was the canonical example; Phase 6 removed it").
- Diagnosis #4 (watchdog at non-default control_dt) updated to
  point at the per-instance derivation site.

### plans/archived/2026-05-10 mpc-tier0-contracts.md (modified)

Phase 6 marked ``COMPLETE (2026-05-10)`` in both the summary table
and the detailed Phase 6 heading.

## Verification

### Existing tests — no behaviour-change impact (pre-audit prediction)

Two classes of risk:

1. **MuJoCoPlant tests that pass out-of-range extensions and rely
   on the silent clip.** Pre-implementation grep:
   ``tests/sim/test_plant.py`` and the rest of ``tests/sim/``.
   None do — every test passes valid extensions from
   ``pose_to_extensions(target)``.  The clamp warning was only
   exercised by adversarial inputs which no test sends.
2. **MuJoCoPlant constructions that omit ``control_dt`` AND rely
   on a non-default rate.** Predicted: none, because the only
   non-default control rates would come from ``run_mpc.py`` /
   ``sim/main.py``, and those now pass ``control_dt`` explicitly.
   ``generate_solver.py`` / ``demo_mpc.py`` / ``record_baselines.py``
   construct ``MuJoCoPlant()`` for IK / kinematics work, not a
   40 Hz loop, so the default is correct.

Confirmed empirically: full sim test suite passes (612/612 → 641/641
with the new contract tests) and full pytest suite passes (1167
→ 1180 with the +13 new tests).

### Test results

- **Before Phase 6:** 1167 / 1167 pass (Phase 5 baseline).
- **After Phase 6:** **1180 / 1180 pass** — the +13 delta is exactly
  the new ``test_plant_interface_contract.py`` scenario count.  Zero
  regressions.

### Hardware bringup smoke (recommended)

The plan's Phase 6 exit criterion includes a hardware bringup smoke:

```bash
python run_mpc.py --pose 0,0,170,0,0,0 --duration 5
```

Confirms the P4 wiring on real hardware — the staleness watchdog
should fire at the same wall-clock time as before for the default
``control_dt=0.025``.  Not run as part of this phase commit
(hardware test); user responsibility before Plan 1 closes.

## Discussion

### Why P3 took the pass-through route, not the loud-rejection route

The contract permits both: an implementation MAY raise on
out-of-range inputs (defensive), or MAY pass them through (trusting).
Both are P3-compliant; what's prohibited is silent correction.

We took the pass-through route for ``MuJoCoPlant`` because it
matches ``HardwarePlant``'s pattern and minimises behaviour
change.  Loud rejection would have:

- Required a documented error type (``ValueError`` or a custom
  ``CommandRangeError``).
- Forced every caller that previously relied on the silent clip
  to either pre-validate or catch the exception.  No such callers
  exist today, but the loud-rejection path is more invasive in
  general.
- Made the contract test brittle — the test would have to
  encode the expected error type.

Pass-through is the simpler option.  The MuJoCo model's slide-joint
``range`` attribute provides the physics-layer envelope at the
actuator level: an out-of-range ``data.ctrl`` value produces a
saturated joint target.  No segfault, no NaN, no Python exception.
The MPC's solver-exit clamp and the K6 reference twist clamp
already validate inputs at the contract boundary upstream;
``command()`` doesn't need to re-validate.

### Why module-level multipliers, instance-level thresholds

The Phase 6 design moves the staleness *thresholds* to per-instance
attributes (because they depend on ``self._control_dt``) but keeps
the *multipliers* at module level (because they're dimensionless
physical constants).  Three options were considered:

1. **Everything per-instance**: ``self._telem_stale_warn_mult = 3.0``
   etc.  Cleaner symmetry but adds three attributes per instance
   that never vary.
2. **Everything module-level**: keep the thresholds as constants
   parameterised by a module-level ``control_dt`` global.  Wrong —
   one global ``control_dt`` shared by every ``HardwarePlant``
   instance precludes mixed-rate operation (digital twin alongside
   production).
3. **Multipliers module, thresholds instance**: chosen.  The
   multipliers are physical constants ("3 control-ticks of clock-
   skew tolerance is enough for this protocol") that don't depend
   on operating regime; the thresholds are
   regime-dependent and live per-instance.

Option 3 mirrors the K1–K6 design (the ``β`` margin lives at module
level; the velocity / acceleration thresholds derive from
``v_max_mmps`` per-instance).  Same structural pattern, same
rationale.

### Why we left the τ_grace–plant.control_dt unification for Plan 2

The Phase 4 audit identified that the scheduler's S1 ``τ_grace``
default derives from
``cumulative_times[1] - cumulative_times[0]`` — the MPC horizon
discretisation, not the control period.  These happen to coincide
in the current configuration, but the contract should be tied to
``plant.control_dt`` (the canonical period) to prevent silent
divergence.

Three approaches considered for Phase 6:

1. **Tighten the scheduler to require a plant reference.** Add
   ``plant: PlantInterface`` to ``EventScheduler.__init__``; derive
   ``tau_grace_s`` from ``plant.control_dt``.  Tightest coupling
   but adds a dependency the scheduler currently doesn't have.
2. **Add a ``control_dt`` kwarg to the scheduler.** Optional.
   Callers pass ``plant.control_dt`` explicitly.  Documentation
   clarifies that ``control_dt`` overrides the
   ``cumulative_times``-derived default.
3. **Document only.** Update both contracts to note the unification
   intent; thread the value via callers' explicit
   ``tau_grace_s=plant.control_dt`` pass-through.  Defer the
   structural refactor.

Phase 6 took option 3 — minimal coupling, defers the structural
choice between options 1 and 2 to Plan 2.  The current
``cumulative_times`` derivation is *correct* (just inverted-source);
the Phase 6 scope was to land the ``plant.control_dt`` *property*
that consumers can read.  Plan 2's "Tier 1 state-machine
completeness" follow-up will choose the kwarg vs. plant-reference
direction.

### What landed without enforcement

P3's contract permits ``command()`` to either raise OR pass through
on bad inputs.  The contract test asserts no silent coercion —
*either* path is P3-compliant.  Both shipped implementations took
the pass-through path; a future implementation that opts for loud
rejection (e.g., a development-mode plant) is still compliant.
The test's per-implementation ``except (ValueError, RuntimeError):
return`` early-exits on raise, so adding a defensive plant
implementation later won't require test changes.

The hardware bringup smoke (``python run_mpc.py --pose 0,0,170,0,0,0
--duration 5``) is a recommended exit criterion that requires
hardware access; it's deferred to the user's pre-Plan-1-close
checklist.

## Open Questions

- **``cmd_margin_mm`` cleanup.** The constructor kwarg is still
  accepted but no longer read by ``command()``.  A follow-up
  cleanup could remove it once all callers stop passing it.
  Captured here; not in scope for Phase 6.
- **τ_grace–plant.control_dt unification (deferred to Plan 2).**
  See the Discussion section above.  The contract document
  flags this as a Phase 6 follow-through; Plan 2 will choose
  between the kwarg-on-scheduler and plant-reference-on-scheduler
  paths.
- **MuJoCoPlant ``cmd_margin_mm`` interaction with hardware
  workspace limits.** The pre-Phase-6 clip protected MuJoCo from
  out-of-range commands; the post-Phase-6 sim now relies on
  MuJoCo's slide-joint ``range`` attribute for the saturation
  envelope.  An adversarial caller that sends ``-1e6`` could
  surface MuJoCo edge cases (NaN propagation through quaternion
  normalisation, etc.) that the clip previously hid.  The contract
  test exercises ``±10 000`` and ``-500`` — both within ranges
  MuJoCo handles cleanly.  If a future test surfaces a MuJoCo
  edge case at extreme out-of-range values, Plan 2 may want to
  add a per-implementation upper-bound documentation requirement.
- **Hand-control ``np.clip`` at ``mujoco_plant.py:328``.** The
  ``command_hand`` method still silently clips the hand position
  to ``[0, hand_stroke_mm]``.  Hand control isn't in the
  ``PlantInterface`` ABC (Phase 4 audit "Other observations" —
  hand control lives on ``MuJoCoPlant`` only).  Not in P3's
  scope.  Captured in the SCHEDULER_CONTRACT.md S5 follow-up
  for the eventual ``HandInterface`` ABC.
