---
title: Plant-interface contract — Phase 4 audit and P1–P4 draft
type: feature
date: 2026-05-09
status: resolved
phase: "mpc-tier0-contracts — Phase 4"
related_plan: "mpc-tier0-contracts.md"
related_entries:
  - 2026-05-09-scheduler-contract-phase-3-s4-s6-enforcement
  - 2026-05-09-scheduler-contract-phase-2-s1-s3-enforcement
  - 2026-05-09-scheduler-contract-phase-1-audit
  - 2026-04-23-hot-loop-zero-allocation-contract
  - 2026-04-20-k1-k6-reference-feasibility-resolution
files_changed:
  - controller/PLANT_INTERFACE_CONTRACT.md
  - plans/archived/2026-05-10 mpc-tier0-contracts.md
  - logbook/2026-05-09-plant-interface-contract-phase-4-audit.md
  - logbook/INDEX.md
commits:
  - a5da552
subsystem:
  - controller
  - mpc
tags:
  - contract
  - docs
  - safety
---

# Plant-interface contract — Phase 4 audit and P1–P4 draft

## Summary

Phase 4 of the [mpc-tier0-contracts plan](../plans/archived/2026-05-10%20mpc-tier0-contracts.md):
audited [controller/plant.py](../controller/plant.py),
[controller/hardware_plant.py](../controller/hardware_plant.py), and
[sim/plant/mujoco_plant.py](../sim/plant/mujoco_plant.py) for invariants
the code currently relies on (explicit and implicit), then drafted
[controller/PLANT_INTERFACE_CONTRACT.md](../controller/PLANT_INTERFACE_CONTRACT.md)
following the structural template of
[REFERENCE_LAYER_CONTRACT.md](../controller/REFERENCE_LAYER_CONTRACT.md),
[SCHEDULER_CONTRACT.md](../controller/SCHEDULER_CONTRACT.md), and
[HOT_LOOP_CONTRACT.md](../controller/HOT_LOOP_CONTRACT.md).  No code
changes — Phases 5–6 land enforcement.

## Motivation

The Plan 1 ratchet — K1–K6 first, then S1–S6, now P1–P4 — addresses the
same structural insight at progressively wider boundaries.  K1–K6 codified
the *reference* contract between target sources and the MPC.  S1–S6
codified the *temporal* contract between event sources and the scheduler.
P1–P4 codifies the *plant* contract between the MPC and the
physical/simulated robot — the boundary every safety-critical signal
crosses each tick.

Pre-contract, the
[PlantInterface ABC at controller/plant.py:37–73](../controller/plant.py)
specified four abstract methods (``command``, ``get_state``, ``step``,
``reset``) without normative guarantees on their semantics.  Two
concrete implementations exist —
[MuJoCoPlant](../sim/plant/mujoco_plant.py) and
[HardwarePlant](../controller/hardware_plant.py) — and they diverge
silently on every one of the four invariants below.  The hot-loop
allocation contract (HOT_LOOP_CONTRACT.md:434–442) documents the
``PlantState`` aliasing convention but enforces it only against
``HardwarePlant``; the ABC accepts both aliasing and non-aliasing
implementations.  A future ``PlantInterface`` implementation can pass
every sim test against ``MuJoCoPlant`` and break on hardware.

Phase 4 deliberately produces no code change; it is the discovery +
design pass that surfaces what the two implementations currently
guarantee and where they diverge.  Phases 5 and 6 land enforcement
against this specification.

## Design

The contract follows the existing Plan 1 / project template:
Background → Invariants → Enforcement → Implementing-a-new-plant
template → Diagnosis → Related.  Four normative invariants:

| Invariant | Statement (one line) |
|-----------|----------------------|
| P1 | ``get_state()`` MUST return the same ``PlantState`` instance every call; ndarray fields mutate in place |
| P2 | ``can_reset: bool`` is mandatory; ``reset()`` raises ``NotImplementedError`` iff not ``can_reset`` |
| P3 | ``command()`` is a trusted-callee boundary; implementations MUST NOT silently coerce, clip, or sanitise inputs |
| P4 | ``control_dt: float`` is a mandatory constructor kwarg with default 0.025; time-window thresholds derive from it |

P1 *promotes* an existing implementation feature (``HardwarePlant``
already aliases) to contract status, against a sibling implementation
(``MuJoCoPlant``) that violates it.  P2 *introduces* a new
capability flag the ABC does not yet expose.  P3 *codifies* the
implicit "trust your inputs" pattern the hot loop already relies on,
against a sibling that silently corrects.  P4 *parameterises* a
hard-coded operating-point assumption.

## Audit findings

The audit walked the ABC plus both implementations, comparing
documented behaviour against actual code.  Four concrete divergences
surfaced; each became an invariant.

### Gap 1 — PlantState aliasing diverges between the two implementations

[hardware_plant.py:262–272](../controller/hardware_plant.py)
pre-allocates a single ``PlantState`` instance in ``__init__``;
[hardware_plant.py:500–513](../controller/hardware_plant.py)
mutates its fields in place each ``get_state()`` call via
``np.copyto`` / ``np.divide(..., out=...)`` / direct slice
assignment, then returns ``self._state``.  Field-level identity is
preserved across calls.

[mujoco_plant.py:198–207](../sim/plant/mujoco_plant.py)
constructs a fresh ``PlantState`` from fresh ndarrays
(``np.array(...)``, ``np.concatenate(...)``) every call.  Each
``get_state()`` allocates ~250 B of Python objects.

The hot-loop allocation contract test
([tests/sim/test_hot_loop_allocation_contract.py](../tests/sim/test_hot_loop_allocation_contract.py))
already runs against both implementations: the default fixture
exercises ``MuJoCoPlant`` and a sibling test
([test_hot_loop_allocation_contract_hardware at line 450](../tests/sim/test_hot_loop_allocation_contract.py))
patches ZMQ and runs the same tracemalloc bracketing against
``HardwarePlant``.  The tracemalloc filter at
[lines 140–147](../tests/sim/test_hot_loop_allocation_contract.py)
excludes ``sim/analysis/``, ``sim/viz/``, and ``matplotlib/`` only —
``sim/plant/`` is *not* filtered, so MuJoCoPlant's per-tick
``PlantState`` allocations DO count toward the 256 B/tick budget,
and the test fits within the threshold today by margin rather than
by exclusion.  What's missing is a *contract*: the budget test is a
runtime gate, not a design-time invariant, so a future
``PlantInterface`` implementation gets no warning until its first
budget-test run.  Worse, a sim test that cached a ``PlantState``
reference across ticks would silently work on the (currently fresh-
state-each-call) ``MuJoCoPlant`` and fail on the in-place-aliasing
``HardwarePlant``.  P1 lifts the aliasing requirement into the ABC
itself so every implementation honours it from construction time.

Phase 5 lifts the aliasing requirement into P1 and adds a
parameterised contract test that exercises both implementations.
``MuJoCoPlant`` will need a refactor to pre-allocate its state.

### Gap 2 — reset capability is silently absent on hardware

[hardware_plant.py:726–734](../controller/hardware_plant.py):

    def reset(self, pose_6dof: np.ndarray | None = None) -> None:
        """Not supported in hardware mode."""
        logger.warning("HardwarePlant.reset() is a no-op; ...")

A caller that calls ``plant.reset(pose)`` expecting the platform to
move there gets a warning in the log and silent acceptance from the
control flow — the next ``command()`` runs against whatever pose
the platform was actually at.  ``MuJoCoPlant.reset()``
([mujoco_plant.py:227–254](../sim/plant/mujoco_plant.py)) actually
resets.  The ABC has no surface for callers to ask "does this plant
honour reset?"

The result is latent rather than active today: ``run_mpc.py``'s
settling sequence at lines 282–286 (``plant.reset(pose); plant.command(ext);
plant.step(2.0)``) lives inside the ``if args.use_sim_plant:`` branch
(line 270), so it never runs against ``HardwarePlant``.  But the
loop-restart pattern at
[sim/main.py:1009–1011](../sim/main.py) — ``source.reset(); plant.reset();
mpc.reset()`` — is exactly the kind of cross-plant helper that
plausibly gets lifted into a shared utility once a runner-level
"start over" path is needed against either implementation.  At that
moment the silent no-op on hardware becomes a real bug.  P2 closes
the gap *before* the lift, so the silent divergence cannot survive
the refactor.

Phase 5 adds the ``can_reset: bool`` abstract property and makes
``HardwarePlant.reset()`` raise ``NotImplementedError`` when called.
Callers (``runner.py``, ``run_mpc.py``) that may be invoked against
either plant get the explicit ``if plant.can_reset:`` guard.

### Gap 3 — input-validation diverges between the two implementations

[mujoco_plant.py:154–159](../sim/plant/mujoco_plant.py):

    ext_clamped = np.clip(ext, lo, hi)
    if not np.allclose(ext, ext_clamped, atol=0.1):
        logger.warning("command() clamped extensions: ...")

``MuJoCoPlant.command()`` silently clips out-of-range extensions to
``[margin, stroke - margin]`` with a warning.

[hardware_plant.py:315–464](../controller/hardware_plant.py):
``HardwarePlant.command()`` does no range validation at all — bad
values flow through to the motor guard's safety pipeline (workspace,
overspeed, max-deviation) which has its own guards.  ``HardwarePlant``
is a *trusted-callee* boundary; ``MuJoCoPlant`` is a defensive
boundary.

The result: a unit test that feeds ``np.array([1000.0]*6)`` (out of
range) to ``MuJoCoPlant.command()`` works (clamped, warning logged);
the same test against ``HardwarePlant.command()`` would forward the
out-of-range values to the motor guard, which would either reject
them (best case) or saturate at the workspace limit (acceptable case)
or trip MAX_DEVIATION (degraded case).  Different observable outputs
for the same input.

The contract makes the trusted-callee pattern explicit (P3) and
prohibits silent correction in either implementation.  Phase 6 lands
the test that verifies neither implementation silently coerces NaN /
out-of-range inputs.  ``MuJoCoPlant``'s clip+warn pattern will need
to either go away or be promoted to a raise.

### Gap 4 — control_dt awareness is partial on hardware, absent on sim

[hardware_plant.py:99](../controller/hardware_plant.py)
takes ``control_dt: float = 0.025`` as a constructor kwarg but uses
it only as a small-dt floor in the velocity / acceleration
feedforward fallbacks
([hardware_plant.py:379, :381, :401](../controller/hardware_plant.py)).
The
telemetry-staleness watchdog thresholds are hard-coded module-level
constants
([hardware_plant.py:67–70](../controller/hardware_plant.py)):

    _TELEM_STALE_WARN_S  = 0.075   # 3x MPC period — log warning
    _TELEM_STALE_HARD_S  = 0.125   # 5x MPC period — zero velocities
    _TELEM_STALE_ESTOP_S = 0.5     # 20x MPC period — telemetry definitely lost

The comments name them as multiples of "the MPC period" but the
multiplier is implicit in the magic number, not the code.  At any
``control_dt`` other than 0.025 the watchdog is wrong.

[mujoco_plant.py:64–69](../sim/plant/mujoco_plant.py)
takes no ``control_dt`` parameter at all.  Sim tests cannot exercise
non-default operating points.

Phase 6 adds ``control_dt`` to ``MuJoCoPlant.__init__``, exposes
``control_dt`` as a read-only abstract property on ``PlantInterface``,
and replaces the ``HardwarePlant`` magic numbers with
``self._control_dt`` multiples.  S1's ``τ_grace`` default in
[scheduler.py:238–244](../controller/scheduler.py) — currently
derived from the MPC horizon discretisation — moves to
``plant.control_dt`` so the operating period has one source of truth.

### Other observations (not promoted to invariants)

The audit also surfaced patterns that are intentional design choices,
not gaps:

- **Hand control surface lives on MuJoCoPlant only.**
  ``MuJoCoPlant`` exposes ``command_hand``, ``hand_to_home``,
  ``hand_to_prime`` as concrete methods; ``HardwarePlant`` does not.
  Hand control on hardware goes through a separate orchestrator
  channel.  This is by design; the contract scopes itself to the
  Stewart platform interface (the four ABC methods) and treats hand
  control as an implementation extension.  A future ``HandInterface``
  ABC could parallel ``PlantInterface`` if more hand implementations
  appear.
- **Ball management lives on MuJoCoPlant only.** Same story —
  ``has_ball``, ``ball_manager``, ``spawn_ball``,
  ``check_and_capture``, ``get_ball_state``, ``release_ball`` are
  sim-only (the hardware doesn't have a "ball" concept; the catch
  coordinator interfaces with a real ball via tracking).
- **Diagnostic accessors on HardwarePlant.**
  ``last_fk_iterations``, ``last_ff_torque_max_Nm``,
  ``last_drain_count``, ``cmd_deadband_hit_count``,
  ``estop_requested`` are HardwarePlant-only diagnostics surfaced for
  the runner's per-tick telemetry breakdown.  These are
  implementation extensions, not contract requirements.
- **set_pose() / enable() / disable() / estop() / close() are
  hardware-specific lifecycle methods.** Not on the ABC; called
  conditionally by ``run_mpc.py`` and the runner via ``hasattr`` /
  ``getattr`` guards.  Promoting them to the ABC would force
  ``MuJoCoPlant`` to implement them as no-ops — the current pattern
  is cleaner.

## Discussion

### Why P1 lifts aliasing from HOT_LOOP_CONTRACT.md into the interface

The hot-loop contract documents PlantState aliasing as a hot-loop
budget property: tracemalloc attributes ~250 B/tick to a fresh
PlantState dataclass plus its ndarray fields, which over 40 Hz drives
Gen-2 GC pressure.  Eliminating that pressure means aliasing.

But the contract's enforcement test runs against ``MuJoCoPlant``
(which violates the convention), with a default tracemalloc filter
that excludes ``sim/plant/`` from the budget — so the test passes
*because* the violation is filtered out, not because the violation is
absent.  Hardware doesn't run the test at all.

The result: a contributor who reads HOT_LOOP_CONTRACT.md and writes
a new ``PlantInterface`` implementation against the
``MuJoCoPlant`` model (fresh PlantState every call) has done the
"wrong" thing per HOT_LOOP_CONTRACT.md but the "right" thing per the
ABC + the existing test.  The contract conflict is the gap P1
closes.

P1 makes aliasing a property of the *interface*, not just the
*hot-loop budget*.  Every ``PlantInterface`` implementation, even ones
that won't ever run in production hot-loops (e.g., a development-
mode ``MockPlant`` for hermetic CI), MUST alias.  Two reasons:

1. **A consumer can be reused across plant implementations.** Code
   that caches ``state = plant.get_state()`` and reads it across
   ticks works on a non-aliasing plant and silently breaks on an
   aliasing one.  Forcing aliasing universally means the consumer
   pattern is the same regardless of plant.
2. **Mixed-mode testing is real.** A digital-twin run pairs a
   ``HardwarePlant`` (the production target) with a ``MuJoCoPlant``
   (the twin running alongside for state estimation).  Both feed
   the same MPC solve.  If their aliasing semantics diverge, the
   MPC reads stale data from one and fresh from the other.

The cost is a one-time refactor of ``MuJoCoPlant.get_state()`` to
pre-allocate.  Phase 5 lands that.

### Why P2 introduces ``can_reset`` rather than overloading ``reset()``

Three options were considered for the reset-capability gap:

1. **Make ``reset()`` raise ``NotImplementedError`` unconditionally
   on HardwarePlant.**  Cleanest at the implementation level; breaks
   every caller that today assumes ``reset()`` is callable on any
   plant.
2. **Add a default ``can_reset = True`` class attribute to the ABC,
   override in HardwarePlant.**  Requires a `hasattr` / `getattr`
   guard convention, which is fragile (silently True if the attribute
   is absent).
3. **Make ``can_reset`` an abstract property.**  Forces every
   implementation to declare the capability explicitly.  Chosen.

Option 3 is consistent with the "explicit boundary" pattern
established by K1–K6 (every TargetSource declares its
``feasibility_layer``), the hot-loop contract (every implementation
declares its allocation profile via the test fixture), and S1–S6
(every event source honours the documented invariants).  Implicit
defaults are how silent divergences land.

The cost of Option 3 is one new abstract property — a documentation
and test addition, not a behavioural one.  Phase 5 lands it.

### Why P3 codifies trusted-callee rather than defensive-clamp

This is the design choice that's most counterintuitive to a
contributor reading the codebase fresh.  The hot-loop budget is the
critical constraint: at 40 Hz over 1 hour, defensive validation on
six floats per tick burns ~86 ms of CPU — almost a full tick — for
no observable benefit, because the MPC `solve()` exit and the motor
guard's 500 Hz safety pipeline have already validated the values.

But "no defensive validation" is a dangerous precedent if read
without context.  The audit identified one clear violation
(``MuJoCoPlant``'s ``np.clip`` + warning), but a future contributor
reading the contract could conclude that ANY input check is
prohibited.  The contract is more nuanced than that:

- **Silent correction is prohibited.**  Clipping NaN to a default,
  truncating an out-of-range value to the envelope, reshaping a
  malformed array — all hide the upstream bug that produced the
  malformed input.
- **Loud rejection is permitted.**  An implementation MAY raise
  ``ValueError`` on a bad input; that's a defensive option that
  surfaces the upstream bug.  The current ``HardwarePlant`` neither
  validates nor raises — it forwards.  Both are P3-compliant.
- **The trusted-callee designation is the *default*, not the
  *requirement*.**  The hot-loop budget makes silent forwarding the
  pragmatic choice, and the upstream pipeline (MPC solve →
  motor_guard) already validates.  But a development-mode plant
  that prefers to crash early on bad inputs is welcome to.

The contract document spells this out so a future contributor
adding "defensive" ``np.clip`` calls knows the implication.

### Why P4 parameterises control_dt rather than leaving it implicit

The audit found ``control_dt`` taking three different forms in the
codebase:

1. **HardwarePlant constructor kwarg** with default 0.025
   (used for velocity-feedforward backward-diff only).
2. **Hard-coded multipliers** of 3 / 5 / 20 in the watchdog
   thresholds (correct for control_dt=0.025 only).
3. **MuJoCoPlant: absent** (the period is implicit in the runner's
   ``CONTROL_DT`` constant, which the plant never sees).

A future operating regime — slower control loop for catch-only
sequences, faster loop for ball-tracking telemetry, dynamic regime
switching — would silently break the watchdog: at 10 Hz the 75 ms
warn threshold fires every tick; at 100 Hz it never fires.  Either
case lands without any test surfacing the drift.

P4 makes ``control_dt`` a first-class plant property.  The
``HardwarePlant`` watchdog thresholds become
``self._control_dt * <multiplier>``; ``MuJoCoPlant`` accepts the
parameter (default unchanged); the runner passes it explicitly to
both.  The scheduler's S1 ``τ_grace`` default — currently derived
from MPC horizon discretisation in
[scheduler.py:238–244](../controller/scheduler.py) — moves to
``plant.control_dt`` so the operating period has one source of
truth.

This is a Phase 6 change.  The ABC addition (``control_dt`` abstract
property) and the ``MuJoCoPlant`` constructor change are part of
Phase 5/6 enforcement; Phase 4 only commits the documented
intention.

## Open Questions

- **Hand and ball interfaces.**  ``MuJoCoPlant`` exposes hand
  control (``command_hand``, ``hand_to_home``, ``hand_to_prime``) and
  ball management (``has_ball``, ``ball_manager``, …) directly on the
  plant.  ``HardwarePlant`` exposes hand control through a separate
  orchestrator channel.  The contract treats these as implementation
  extensions, but if more implementations land (a digital twin with
  its own hand model, a real-hardware variant for a different hand
  design) it would be worth a parallel ``HandInterface`` /
  ``BallInterface`` ABC to formalise the surface.  Captured for
  Plan 2 (``mpc-sadpath-coverage``) Tier 2 work.
- **set_pose / enable / disable / estop on the ABC.**  These are
  hardware-specific lifecycle methods on ``HardwarePlant``, called
  conditionally by ``run_mpc.py`` and ``runner.py`` via
  ``hasattr`` / ``getattr`` guards.  Promoting them to the ABC would
  require ``MuJoCoPlant`` no-ops — adding boilerplate without
  surfacing a real divergence.  Left out of P1–P4.
- **HardwarePlantStub for the contract test.**  The Phase 5 test
  parameterises over ``[MuJoCoPlant, HardwarePlantStub]`` — the
  stub mocks ZMQ telemetry so the test doesn't need a live
  socket topology.  Its design is a Phase 5 deliverable; this audit
  identifies it as a needed component but doesn't specify the
  fidelity (synthetic encoder traces?  recorded telemetry replay?).
- **Cross-contract refactor: scheduler's τ_grace from
  plant.control_dt.**  The scheduler's S1 ``τ_grace`` derives from
  the MPC horizon today.  P4 lands the unification.  Captured for
  Phase 6 follow-through (the SCHEDULER_CONTRACT.md S1 docstring
  already flags this — "the τ_grace default will derive from
  plant.control_dt once Phase 4 lands").  Phase 4 has now landed;
  Phase 6 owns the wiring.
