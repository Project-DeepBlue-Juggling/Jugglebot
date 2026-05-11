---
title: MPC sad-path coverage — Phase 4 Tier 2a HardwarePlant FK degradation, singular Jacobian, frozen motor
type: feature
date: 2026-05-11
status: resolved
phase: "mpc-sadpath-coverage-tiers-1-3 — Phase 4"
related_plan: "mpc-sadpath-coverage-tiers-1-3.md"
related_entries:
  - 2026-05-11-tier1c-input-fuzz
  - 2026-05-11-tier1c-input-fuzz-bugfix
  - 2026-05-11-tier1b-fallback-escalation-cascade
  - 2026-05-11-tier1a-real-solver-failures
  - 2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement
  - 2026-05-09-plant-interface-contract-phase-5-p1-p2-enforcement
files_changed:
  - tests/sim/test_hardware_plant_failure_paths.py
  - tests/sim/_hardware_plant_stub.py
  - logbook/2026-05-11-tier2a-hardware-plant-fk-degradation.md
  - logbook/INDEX.md
  - plans/active/mpc-sadpath-coverage-tiers-1-3.md
commits:
  - <pending>
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
---

# MPC sad-path coverage — Phase 4 Tier 2a HardwarePlant FK degradation, singular Jacobian, frozen motor

## Summary

Plan 2 Phase 4 (Tier 2a) — adds new file
[tests/sim/test_hardware_plant_failure_paths.py](../tests/sim/test_hardware_plant_failure_paths.py)
with **10 passing tests** exercising the three watchdog cascades on
``HardwarePlant.get_state()`` that protect the platform from
oblivious-MPC scenarios under degraded motor telemetry, plus an
extension to [tests/sim/_hardware_plant_stub.py](../tests/sim/_hardware_plant_stub.py)
with three injection helpers (FK divergence, singular Jacobian,
programmable telemetry pump).

The three cascades + their newly-covered surfaces:

| Cascade | Threshold(s) | Production code | New test coverage |
|---------|--------------|-----------------|--------------------|
| FK convergence failure | ``_FK_FAIL_ESTOP_THRESHOLD = 5`` (``hardware_plant.py:217``) → ``estop('fk_convergence_failure')`` at ``:613`` | RuntimeError handler at ``:602–617`` | T-U-T2a-1, -2, -3 + boundary regression |
| Singular Jacobian → zero twist | Once-only warning via ``_jacobian_singular_warned`` flag (``:740``) | ``np.linalg.LinAlgError`` branch at ``:737–740`` | T-U-T2a-4, -5 |
| Frozen motor (publisher-alive-but-dead) | ``_FROZEN_MOTOR_POS_WARN = 20`` warn; ``_FROZEN_MOTOR_POS_ESTOP = 40`` → ``estop('telemetry_frozen')`` | ``np.array_equal`` bit-exact compare at ``:657`` + dual-threshold cascade at ``:648–683`` | T-U-T2a-6a, -6b, -7 |

Plus one Hypothesis stateful property:

* **T-U-T2a-8** — ``RuleBasedStateMachine`` over (``succeed_tick``,
  preconditioned ``fail_tick``) with invariant
  ``not _estop_requested``.  Per-tick rule design with the precondition
  ``shadow_consec < THRESHOLD - 1`` proves directly that any
  consecutive-failure run strictly below ``_FK_FAIL_ESTOP_THRESHOLD``
  cannot fire e-stop.  Holds at ci-deep (1000 examples,
  ``--hypothesis-seed=0``, 616.27 s).

Test additions only; **zero production-code changes**.  No xfails
introduced this phase.

Pre-Phase-4: 1233 passed + 1 xfailed.  Post-Phase-4: **1243 passed
+ 1 xfailed** (+10 new tests, no new xfails).  Hot-loop allocation
contract remains green at ci-deep post-additions.

## Motivation

Phases 1–3 (Tiers 1a, 1b, 1c) drove the MPC solver's sad-path
surface end-to-end: real IPOPT failures, fallback escalation,
NaN/Inf input fuzz.  Phase 4 shifts to the **HardwarePlant adapter**
— the boundary between the controller's pure-Python world and the
physical robot.  Three watchdog cascades documented in the
2026-05-08 audit had **zero test coverage** before this phase:

1. **FK convergence failure cascade.**  Newton-Raphson FK at
   ``leg_lengths_to_pose`` raises ``RuntimeError`` when the iteration
   budget (``max_iter=10``) exhausts.  The handler at
   ``hardware_plant.py:602–613`` increments a counter, returns the
   last-good pose, and on the 5th consecutive failure fires
   ``estop(reason='fk_convergence_failure')``.  Pre-Phase-4 nothing
   exercised this path.
2. **Singular-Jacobian zero-twist.**  When the platform Jacobian
   becomes rank-deficient (e.g. at a workspace boundary),
   ``np.linalg.solve`` raises ``LinAlgError``; the handler at ``:737–740``
   logs a ONCE-only warning and leaves ``platform_twist`` zero
   (zeroed unconditionally at ``:715`` before the try/except).
   Pre-Phase-4 nothing verified the once-only semantic.
3. **Frozen-motor detector.**  When the motor-guard publisher
   freezes its output frame (e.g. CAN drops, encoder stops
   reporting), telemetry timestamps stay fresh but the payload
   stops moving.  The detector at ``:648–683`` catches this via
   bit-exact ``np.array_equal`` and escalates: warn at 20
   identical ticks, e-stop at 40.  Pre-Phase-4 nothing exercised
   either threshold or the bit-exact compare semantic.

All three cascades are safety-critical: each protects against a
class of "MPC continues commanding the platform with stale or wrong
feedback" failure modes.  Each is documented in code but, before
this phase, structurally unverified.

## Design

### Per-test empirical-probe table (run 2026-05-11)

Pre-implementation probes (``/tmp/probe_fk_divergence.py``,
``/tmp/probe_singular_and_frozen.py``, ``/tmp/probe_helpers_smoke.py``;
not committed) confirmed each driver produces the expected behaviour
on the current ``hardware_plant.py`` (commit ``e5427e4`` baseline).
The recipes are deterministic on the pinned dependency stack:

| Test       | Production surface                                          | Driver / boundary                                                          | Empirical confirmation                                              |
|------------|-------------------------------------------------------------|----------------------------------------------------------------------------|---------------------------------------------------------------------|
| T-U-T2a-1  | RuntimeError handler at ``:602–617``                        | ``inject_fk_failure(n_ticks=1)``                                            | count=1 after one fail; cache-fallback pose bit-equal to last-good  |
| T-U-T2a-2  | ESTOP gate at ``:608–613``                                  | ``inject_fk_failure(n_ticks=5)``                                            | Fires at exactly count=5; reason='fk_convergence_failure'           |
| T-U-T2a-2  | (boundary regression) below-threshold                       | ``inject_fk_failure(n_ticks=4)``                                            | count=4; no estop                                                   |
| T-U-T2a-3  | Counter reset at ``:593``                                   | ``inject_fk_failure(n_ticks=4)`` then 1 clean tick                          | count→0 on success                                                  |
| T-U-T2a-4  | LinAlgError branch at ``:737–740``                          | ``inject_singular_jacobian()`` (compute_jacobian + fk_jacobian=None)        | ``platform_twist=[0,0,0,0,0,0]``; ``_jacobian_singular_warned=True``; 1 warn record |
| T-U-T2a-5  | Once-only flag at ``:740``                                  | 10 consecutive singular ticks                                              | Exactly 1 'Jacobian singular' record across 10 ticks                |
| T-U-T2a-6a | WARN at ``:660–667`` (``_FROZEN_MOTOR_POS_WARN=20``)        | ``install_telemetry_pump`` with identical frames                            | Warn record at tick 20; flag set; no estop                          |
| T-U-T2a-6b | ESTOP at ``:668–676`` (``_FROZEN_MOTOR_POS_ESTOP=40``)      | ``install_telemetry_pump`` × 40 identical frames                            | ``estop(reason='telemetry_frozen')`` at exactly tick 40             |
| T-U-T2a-7  | Bit-exact ``np.array_equal`` at ``:657``                    | ``install_telemetry_pump`` with 1-ULP per-tick variation                    | count stays 0 across 44 ticks; no warn, no estop                    |
| T-U-T2a-8  | Property — bursts < THRESHOLD never E-stop                  | ``RuleBasedStateMachine`` (per-tick rules + ``@precondition``)              | 1000 examples ci-deep, seed=0, 616.27 s; invariant holds            |

### Why drive failures through the boundary, not the counter

Plan 2 Working Note #1 ("Drive real failures, not mocked ones")
explicitly flags Phase 4 as an at-risk spot: *"FK divergence tests
must drive through ``get_state()`` with simulated bad telemetry, not
poke ``_fk_fail_count`` directly. A test that pokes a private counter
doesn't validate the watchdog — it validates that the counter
increments."*

The three injection helpers honour this:

* **``inject_fk_failure(n_ticks)``** patches
  ``controller.hardware_plant.leg_lengths_to_pose`` (the module-level
  binding the production code uses at ``hardware_plant.py:580``).
  The patch raises ``RuntimeError`` — the same exception shape real
  Newton-Raphson non-convergence produces — so the production code's
  ``except RuntimeError`` at ``:602`` is the surface under test, not
  the boundary patch.  The watchdog counter is observed via
  ``plant._fk_fail_count`` only for *verification*, never poked for
  *setup*.
* **``inject_singular_jacobian()``** patches both
  ``compute_jacobian`` (to return a rank-deficient matrix) AND
  ``leg_lengths_to_pose`` (to return ``fk_jacobian=None``, forcing
  the production code's ``else`` branch at ``:721–724``).  Drives
  the genuine ``np.linalg.LinAlgError`` path.
* **``install_telemetry_pump``** replaces the plant's
  ``_sub.recv_multipart`` with a programmable pump.  The detector's
  bit-exact ``np.array_equal`` runs on real msgpack-decoded payloads
  through the production code path at ``:648–683`` — no mock between
  the pump and the detector.

The setup-vs-verification line is honoured throughout: Phase 4's
tests touch private state only in two narrow places — the hypothesis
stateful machine's ``__init__`` (resetting watchdog state to a known
baseline before each walk, mirroring Phase 3's
``T1cWarmStartIntegrityMachine`` discipline) and the boundary
constant reads (``plant._FK_FAIL_ESTOP_THRESHOLD`` etc., used to
make the threshold ground truth explicit and catch upstream drift).

### Singular-Jacobian construction: synthetic vs workspace-boundary

The plan offered two paths: (a) workspace-boundary pose (honest
end-to-end driver, requires finding a near-singular pose in
``motion/kinematics.py``) or (b) synthetic SVD construction
(monkey-patch ``compute_jacobian``).

Phase 4 chose (b).  Rationale:

* The Jacobian production code (``hardware_plant.py:706–736``) uses
  a *normalized* Jacobian (rotational columns scaled by
  ``plat_radius_mm``) for the solve.  The condition-number-based
  workspace-boundary heuristic in ``motion/kinematics.py`` operates
  on the raw Jacobian — translating one to the other reliably to
  produce a strictly-singular normalized J would be a non-trivial
  numerical exercise.
* The production surface under test is ``except LinAlgError`` —
  the test's purpose is exercising the handler, not validating the
  workspace-boundary heuristic.
* Patching at the ``compute_jacobian`` boundary AND nulling out the
  FK-supplied Jacobian path (which would otherwise short-circuit
  ``compute_jacobian``) gives deterministic, reproducible singularity
  with minimal coupling to numerical details.  The matrix shape is
  ``np.eye(6)`` with column 0 zeroed → rank 5 → ``LinAlgError`` on
  every solve.

The plan's preference for an end-to-end driver is honoured in
spirit (the LinAlgError is REAL — produced by real
``np.linalg.solve`` arithmetic on a real singular matrix); only the
*source* of the singularity is synthetic.

### Frozen-motor: two thresholds, not one

The plan text named "the frozen-motor detector" with a single
threshold (``_FROZEN_MOTOR_THRESHOLD``).  Empirical reading of
``hardware_plant.py`` revealed **two** thresholds and the constant
is named differently:

* ``self._FROZEN_MOTOR_POS_WARN = 20`` (``hardware_plant.py:193``)
  — one-time ``logger.warning`` + flag set.
* ``self._FROZEN_MOTOR_POS_ESTOP = 40`` (``:194``) —
  ``estop(reason='telemetry_frozen')``, gated on
  ``_fk_ever_succeeded`` so de-energised start-up frames don't
  trip.

Phase 4 splits T-U-T2a-6 into a pair-test (T-U-T2a-6a + T-U-T2a-6b)
following the Phase 2 pattern (cf. ``TestWalkForwardRefShiftThreshold``
in ``test_solver_failures.py`` — pair on the 20 mm xyz shift
threshold).  Each test pins one edge of the dual threshold.

### T-U-T2a-7: the plan's epsilon framing vs the actual bit-exact compare

The plan wrote *"Feed positions that differ by `< float epsilon`
(legitimately stalled at a setpoint) | Detector does NOT fire (verify
the threshold)"*.  Empirical read of ``hardware_plant.py:657`` shows
the detector uses ``np.array_equal``, which is **bit-exact** —
not epsilon-based.

The actual semantic is therefore *stronger* than the plan's framing
implied: **any** nonzero per-leg difference, down to a 1-ULP
increment, resets the counter and prevents the warning from firing.
No false-positive is possible from sub-LSB encoder noise because
any noise is nonzero.  T-U-T2a-7 documents this explicitly in its
docstring; the test varies leg 0 by ``1e-15 * tick_idx`` per tick
(physically negligible, bit-distinguishable) and asserts the
detector stays at count=0 across 44 ticks.

The plan's hedge "verify against current code" was the right
framing — Phase 4's job is to capture the actual production
semantic, not the plan author's hypothesis.  Filed as a methodology
echo of Phase 2's T-U-T1b-5 hedge ("`cold_start_method == 'linear_interp'`
or whichever sentinel the code surfaces") and Phase 3's discovery
that ``platform_twist`` is a dead field in ``solve()``.

### T-U-T2a-8 design — per-tick rules over per-burst

The first design used a ``fail_burst(n)`` rule with ``n in [0, 4]``.
Hypothesis correctly produced a sequence
``fail_burst(3) → fail_burst(2)`` that *legitimately* fires
``estop`` — five consecutive failures in total, count=5, threshold
reached.  This wasn't a watchdog bug; it was a flaw in the test's
expression of the property's precondition.

The property is *"any sequence of FK failures of length < threshold
does not trigger E-stop"* — where **"length"** means the maximum
consecutive-failure run, not the per-burst count.  Per-burst rules
can't enforce this precondition because the strategy doesn't see
across-rule state.

The revised design uses per-tick rules with a ``@precondition`` that
reads the machine's shadow counter:

```python
@precondition(lambda self:
              self._shadow_consec
              < self._plant._FK_FAIL_ESTOP_THRESHOLD - 1)
@rule()
def fail_tick(self):
    with inject_fk_failure(n_ticks=1):
        self._plant.get_state()
    self._shadow_consec += 1
```

The precondition guarantees that *before* a ``fail_tick`` fires,
``shadow_consec < THRESHOLD - 1``, so *after* it fires
``shadow_consec ≤ THRESHOLD - 1`` (never reaches THRESHOLD).
Hypothesis can never propose a path that legitimately fires the
e-stop; any observed e-stop is a violation of the watchdog's
documented threshold semantic.

The second invariant (``fk_count_matches_shadow``) couples shadow
to production counter, catching counter corruption from a future
production refactor.  Together the two invariants pin both
**(a)** the cascade's safety property (no E-stop below threshold)
and **(b)** the counter-reset semantic (succeed_tick → 0).

### Singleton plant + frozen-motor pump interaction

Phase 3's logbook documented the
``FlakyStrategyDefinition`` pitfall: a hypothesis ``RuleBasedStateMachine``
that monkey-patches singleton state must unconditionally restore
that state in ``__init__``, or a prior walk's monkey-patch can bleed
into the next walk's strategy generation.

Phase 4 inherits this discipline (the
``T1cWarmStartIntegrityMachine`` resets ``mpc._solver = real_solver``
in its ``__init__``; Phase 4's ``T2aFkBurstBelowThresholdMachine``
resets ``_fk_fail_count``, ``_estop_requested``, the
frozen-motor counter, and the singular-J warned flag).

Phase 4 surfaced a NEW pitfall not present in Phase 3:

**The default stub pump publishes byte-identical telemetry frames.**
At 40 ticks the frozen-motor detector itself fires e-stop with
``reason='telemetry_frozen'``.  A hypothesis walk of more than 40
rules on the same singleton would trigger this orthogonal cascade
before the per-tick FK property under test had a chance to falsify.

**Mitigation** (landed in
[test_hardware_plant_failure_paths.py](../tests/sim/test_hardware_plant_failure_paths.py)):
the stateful machine's ``__init__`` installs a per-tick varying
telemetry pump via ``install_telemetry_pump(plant, frame_mutator=...)``.
The mutator adds ``1e-15 * tick_idx`` to leg 0 of ``motor_pos`` —
physically negligible, bit-distinguishable, keeps
``np.array_equal`` from incrementing the frozen counter.

This is in turn the same logic the T-U-T2a-7 false-positive test
verifies — the property test reuses the false-positive test's
mechanism to keep the orthogonal cascade silent.  No new test
infrastructure was needed.

### Caplog incompatibility — direct-handler workaround

Initial drafts used pytest's ``caplog`` fixture to verify the
once-only ``Jacobian singular`` / ``motor_pos frozen`` warnings.
``caplog`` did not capture records — empirically the
``controller.hardware_plant`` logger's emission level chain (set by
``caplog.set_level`` for the test's duration) didn't propagate to
the warning records.

Phase 1's ``test_per_node_ik_budget_exhaustion_falls_back_to_linear_interp``
test in ``test_solver_failures.py`` already worked around the same
issue with a custom ``logging.Handler`` subclass.  Phase 4 reuses
that pattern as a ``_ListHandler`` + ``_capture_hp_warnings``
context manager in
[test_hardware_plant_failure_paths.py](../tests/sim/test_hardware_plant_failure_paths.py).
The context manager attaches a handler at WARNING level for the
duration of the ``with`` block; on exit, removes it and restores
the prior level.

### E-stop verification — spy pattern

The plan offered three options for verifying ``estop()`` fired:
mock-only (couples to method name), state-based (poking private
flag, the plan explicitly warns against), or spy-based
(record-and-call-original).  Phase 4 chose **spy** via the
``_EstopSpy`` helper.  Rationale:

* Captures the call shape (``reason=`` argument) AND the state
  side-effect (production ``_estop_requested = True``) in one
  observation.
* Doesn't couple to ``estop``'s internal implementation — if the
  method's signature changes to accept additional kwargs, the spy
  still works (it forwards via ``self._real(reason=reason)``).
* Avoids the "private state poking is forbidden" objection — the
  spy observes the public call boundary, not the private flag
  directly.  (The state-based assertion ``plant._estop_requested``
  is also used, but only as a *verification* read alongside the
  spy's call-count assertion — both checks must pass.)

## Implementation

### tests/sim/test_hardware_plant_failure_paths.py — new file

| Class                                    | ID(s)              | Tests     | Strategy                                          |
|------------------------------------------|--------------------|-----------|---------------------------------------------------|
| ``TestFkDivergenceCascade``              | T-U-T2a-1, -2, -3  | 4 (incl. boundary regression) | scenario tests with ``inject_fk_failure`` |
| ``TestSingularJacobianZeroTwist``        | T-U-T2a-4, -5      | 2         | scenario + ``inject_singular_jacobian``           |
| ``TestFrozenMotorDetector``              | T-U-T2a-6a, -6b, -7| 3         | scenario + ``install_telemetry_pump``             |
| ``T2aFkBurstBelowThresholdMachine``      | T-U-T2a-8          | 1 (stateful) | ``RuleBasedStateMachine`` per-tick + precondition |

Cross-cutting helpers in the test module:

* ``_prime_plant(plant)`` — one successful ``get_state()`` so
  ``_fk_ever_succeeded`` flips True (the cascade ESTOP gates at
  ``hardware_plant.py:608, 668`` require this).
* ``_EstopSpy(plant)`` — record-and-call-original wrapper around
  ``plant.estop``; exposes ``call_count`` and ``reasons``.
* ``_capture_hp_warnings()`` — context manager attaching a
  ``logging.Handler`` to ``controller.hardware_plant`` at WARNING
  level; returns the records list.

### tests/sim/_hardware_plant_stub.py — extension

| Helper                                  | Boundary                                                       | Purpose                                                 |
|-----------------------------------------|----------------------------------------------------------------|----------------------------------------------------------|
| ``inject_fk_failure(n_ticks)``          | ``controller.hardware_plant.leg_lengths_to_pose``               | Force next N FK calls to raise ``RuntimeError``          |
| ``inject_singular_jacobian()``          | ``compute_jacobian`` + ``leg_lengths_to_pose`` (drop ``fk_jacobian``) | Force ``np.linalg.solve`` to raise ``LinAlgError``       |
| ``install_telemetry_pump(plant, ...)``  | ``plant._sub.recv_multipart``                                   | Programmable per-tick telemetry frame (mutator hook)     |

Each helper is documented in the file with its production-code
boundary cited by symbol (per Plan 2 Working Note #3) and a
docstring linking to the production code line ranges.

## Verification

Each cited count carries the (date, exact pytest invocation, result)
triple per the workflow rule on test-count claims.

### Baseline (post Phase 3 + bugfix)

* ``pytest tests/ -q``, run 2026-05-11 against SHA ``e5427e4``:
  **1233 passed + 1 xfailed in 341.45 s.**

### Module-isolated run

* ``pytest tests/sim/test_hardware_plant_failure_paths.py -q``, run
  2026-05-11: **10 passed in 41.46 s.**  All Phase 4 tests in
  isolation at ci-fast.

### Property test depth — ci-deep validation

* ``pytest tests/sim/test_hardware_plant_failure_paths.py::TestT2aFkBurstBelowThreshold
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q``, run
  2026-05-11: **1 passed in 616.27 s** (1000 examples).
  The FK-burst-below-threshold invariant holds at nightly depth
  with deterministic seed.

### Hot-loop allocation contract — post-additions regression check

* ``pytest tests/sim/test_hot_loop_allocation_contract.py
  --hypothesis-profile=ci-deep --hypothesis-seed=0 -q``, run
  2026-05-11: **3 passed in 16.59 s.**  ``gc.collect()`` mitigation
  from Phase 3 holds; no regression from Phase 4's added Hypothesis
  test or stub-extension allocations.

### Full-suite gate (post Phase 4)

* ``pytest tests/ -q``, run 2026-05-11 with all Phase 4 changes
  applied: **1243 passed + 1 xfailed in 382.03 s.**  +10 passed
  matches the new tests in Phase 4; no new xfails introduced.
  Zero regressions on existing tests; the inherited Phase 1 xfail
  (T-U-T1a-4 ``Restoration_Failed``) remains.

## Discussion

### What Phase 4 reveals about the HardwarePlant safety surface

Three structural observations from this work:

1. **The three cascades are independent — and that's the point.**
   FK convergence, Jacobian conditioning, and motor-pos freshness
   are three orthogonal failure modes that each surface different
   classes of hardware degradation.  The cascade IS the contract
   between "telemetry is good enough" and "platform is no longer
   safely controllable by the MPC".  Each cascade's threshold is a
   design decision about how much *single-axis* degradation is
   tolerable before falling back to an e-stop.  Phase 4 makes the
   thresholds testable; it does not (yet) make them documentable.
   A successor effort — perhaps a ``HARDWARE_PLANT_SAFETY_CONTRACT.md``
   in the K1–K6 / S1–S6 / P1–P4 family — could pull the thresholds
   and their rationales into normative text.  Filed as a Plan 2
   follow-up topic for the Tier-3 schema work in Phase 7.

2. **The bit-exact frozen-motor compare is stricter than the plan
   anticipated.**  The plan's "differ by `< float epsilon`"
   framing in T-U-T2a-7 reflected a guess about the implementation;
   the actual implementation (``np.array_equal``) is bit-exact and
   therefore strictly safer.  No false-positive from encoder LSB
   noise is structurally possible — any noise is nonzero, and the
   detector resets.  This is a property worth pinning explicitly
   (T-U-T2a-7 does), because any future refactor that swaps
   ``np.array_equal`` for ``np.allclose`` or a tolerance-based
   compare would silently introduce a false-positive surface.

3. **The stub extension's injection helpers are reusable across
   future phases.**  Phase 5 (Tier 2b — telemetry & FF) will need
   ``install_telemetry_pump`` for the staleness-threshold matrix
   (with timestamp manipulation).  Phase 6 (Tier 2c — ZMQ
   corruption) may reuse the pump pattern for byte-level injection.
   The boundary-patch pattern (``inject_fk_failure``,
   ``inject_singular_jacobian``) generalizes to any production code
   that catches a specific exception class — Phase 6 may pattern
   on it for ``msgpack.UnpackException`` injection.

### Per-tick rules vs per-burst — why the precondition pattern won

The first-draft T-U-T2a-8 used a ``fail_burst(n)`` rule with
``n in [0, THRESHOLD-1]``.  Hypothesis falsified it with
``fail_burst(3) → fail_burst(2)`` — five consecutive failures
total, threshold reached, e-stop fires.  This is *correct cascade
behaviour*, not a bug; the test design failed to express the
property's precondition (consecutive-run length, not per-burst
length).

The fix — per-tick rules with a ``@precondition`` that reads a
shadow counter — has two virtues:

(a) It expresses the property's precondition directly, with no
"strategy might exceed the bound" loophole.

(b) It composes naturally with the production code's per-tick
state machine — each ``get_state()`` call IS one tick of the
cascade.  A rule that does N ticks at once obscures this; a rule
that does one tick mirrors the production cadence.

The two invariants (``estop_never_fires``,
``fk_count_matches_shadow``) split the property into the safety
claim (no false E-stop) and the bookkeeping claim (counter
semantics match the shadow model).  Either invariant firing
independently is informative: an ``estop_never_fires`` failure
indicates a threshold drift; a ``fk_count_matches_shadow``
failure indicates a counter-reset semantic change.

### Working Note #1 compliance — the test-the-watchdog framing

Plan 2 Working Note #1 specifically warns that *"FK divergence
tests must drive through ``get_state()`` with simulated bad
telemetry, not poke ``_fk_fail_count`` directly. A test that pokes
a private counter doesn't validate the watchdog — it validates
that the counter increments."*  Every Phase 4 test honours this:

* FK injection patches the FK function (boundary), not the counter.
* Singular-J injection patches the Jacobian function + nulls
  ``fk_jacobian`` (boundary), not the warning flag.
* Frozen-motor injection feeds real msgpack frames through real
  ``recv_multipart``, not a poke of ``_frozen_motor_pos_count``.

The only counter reads are *verifications* (assert
``_fk_fail_count == n`` after driving n failures); the only counter
writes are in the hypothesis state machine's ``__init__``, which
is SETUP (resetting to a known baseline before each walk).  Setup
≠ verification; setup is a precondition on the walk, not a poke of
the surface under test.

### Plan-text drift — line-citation refresh

The plan's Phase 4 section cites four line ranges in
``hardware_plant.py``: ``:195``, ``:554–575``, ``:715–718``,
``:163–172``.  Empirical reading shows all four have drifted (the
constants moved; the cache-fallback is at ``:614–617``; the
singular-J handler is at ``:737–740``; the frozen-motor detector is
at ``:648–683``).  The drift is harmless (Phase 4's test file uses
SYMBOL references — ``_FK_FAIL_ESTOP_THRESHOLD``, ``np.array_equal``,
``_jacobian_singular_warned`` — per Plan 2 Working Note #3) but
worth recording so a future reader doesn't trust the plan-side
line citations.  The Phase 4 entry in the plan's Outcome paragraph
authoritatively cites the post-Phase-4 line numbers.

### Hardware test target date — T-H-T2a-1

Per Plan 2 Working Note #7 ("Hardware tests have target dates, not
'before plan closes'"), Phase 4's exit criteria must set a target
date for the deferred hardware test T-H-T2a-1 (CAN unplug 1–2s).
Per the Testing Plan in the plan document, T-H-T2a-1 is gated on
**T-H-T2b-1 PASS** (encoder-publisher kill in isolation — a Phase
5 deliverable).

**Target window**: T-H-T2a-1 runs within **2 weeks of Phase 5
commit** (the date T-H-T2b-1 lands).  This honours the working
note's discipline (concrete window, not "before plan closes") while
respecting the normative gate (T-H-T2b-1 must validate the
publisher-kill mechanism in isolation before the cascaded CAN-loss
test runs).  If Phase 5 commits 2026-05-12, T-H-T2a-1 runs by
2026-05-26 at the latest; if 2026-05-15, by 2026-05-29; etc.

This is recorded in the Phase 4 Outcome paragraph in the plan, in
the Phase 5 exit criteria (when Phase 5 commits), and here in the
Phase 4 logbook entry.  No xfail discipline applies because the
hardware test is not an automated test — it's an investigation
entry under ``/investigate``.

### Xfail accounting — Phase 4

Phase 4 added **0 new xfails**.  Total xfails on the suite at end
of Phase 4 remains **1** (T-U-T1a-4 ``Restoration_Failed``,
inherited from Phase 1):

| Test ID     | Tracking                                                        | Target close                          |
|-------------|-----------------------------------------------------------------|---------------------------------------|
| T-U-T1a-4   | logbook 2026-05-11-tier1a-real-solver-failures.md (Discussion → Xfail accounting) | Permanent (CasADi 3.7.2 limitation; structural matrix coverage) |

Per Plan 2's archival-gate language: zero unfixed xfails at
archival, OR each residual xfail has a documented justification.
T-U-T1a-4 has a permanent justification.  Phase 4 surfaced no new
production-code bugs (the frozen-motor compare semantic is
stronger-than-documented, not buggy; the singular-J once-only
warning works as designed; the FK threshold fires at exactly 5).
Phase 3's T-U-T1c-7-bug was fixed in the same session, so no Phase
3 xfail remains either.

## Open Questions

* **Should the cascade thresholds be documented in a
  ``HARDWARE_PLANT_SAFETY_CONTRACT.md`` document?**  Phase 4 makes
  them testable; it does not normalize them.  A K1–K6-style
  contract document would pin (a) the threshold values, (b) the
  E-stop reasons each cascade uses, and (c) the gating conditions
  (``_fk_ever_succeeded`` for the FK + frozen-motor ESTOP gates).
  Out of Phase 4's scope.  Filed as a Plan 2 Phase 7 (Tier 3a
  schema fuzz) follow-up topic — the schema-completeness work is
  the natural home for the cascade contract.

* **Should the once-only warning flag be reset on success-tick or
  only on exit from the singular streak?**  Currently
  ``_jacobian_singular_warned = False`` resets on the *next*
  successful twist-solve (``hardware_plant.py:736``), which means a
  successful-then-singular-again sequence emits a second warning.
  If the platform repeatedly traverses a workspace boundary (e.g.
  near a juggle apex), the log may fill with warnings.  Phase 4
  doesn't change this; the test T-U-T2a-5 verifies the current
  semantic (once per streak).  If a future operator log-noise
  audit surfaces this as a problem, the fix is straightforward
  (replace the per-streak reset with a per-session reset).  Filed
  as an open question for the next hardware bringup.

* **Should the frozen-motor compare tolerate quantized encoder
  motion?**  Currently bit-exact ``np.array_equal``.  A motor
  truly held at a setpoint with active position control will show
  encoder dither at the LSB level — *unless* the encoder reports
  the same quantized value across two consecutive 25 ms windows,
  which IS plausible for a static hold.  If this ever produces a
  false-positive in production, the fix would be a small-window
  tolerance (e.g. "max-abs-diff across last N ticks > 0").  Phase
  4 documents the current strictness via T-U-T2a-7; surfacing a
  real false-positive in production would be the trigger to
  reconsider.

* **Does T-U-T2a-8's ``shadow_consec`` need to also model
  ``_fk_ever_succeeded`` transitions?**  Currently the machine
  pre-seeds ``_fk_ever_succeeded = True`` and never resets it.
  The production code only gates ESTOP on this flag (not WARN
  records), so the property holds vacuously for cold-start
  scenarios.  A future widening (e.g. ``cold_start_tick`` rule
  that resets ``_fk_ever_succeeded`` to ``False``) could close
  this gap; Phase 4 deferred it because the cold-start surface is
  better covered by a future Phase 5 test (cold-start zero-state
  is a T-U-T2b-7 deliverable).

## Related

* [plans/active/mpc-sadpath-coverage-tiers-1-3.md](../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
  — Plan 2 Phase 4 specification.
* [logbook/2026-05-11-tier1c-input-fuzz.md](2026-05-11-tier1c-input-fuzz.md)
  — Phase 3 (Tier 1c); singleton + ``FlakyStrategyDefinition``
  discipline inherited from there.
* [logbook/2026-05-11-tier1c-input-fuzz-bugfix.md](2026-05-11-tier1c-input-fuzz-bugfix.md)
  — Phase 3 follow-up; same-session-fix workflow rule precedent.
* [logbook/2026-05-11-tier1b-fallback-escalation-cascade.md](2026-05-11-tier1b-fallback-escalation-cascade.md)
  — Phase 2; pair-test pattern (T-U-T1b-1 + -2) precedent.
* [logbook/2026-05-11-tier1a-real-solver-failures.md](2026-05-11-tier1a-real-solver-failures.md)
  — Phase 1; ``_ListHandler`` custom-handler workaround for
  ``caplog`` originated here.
* [logbook/2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement.md](2026-05-10-plant-interface-contract-phase-6-p3-p4-enforcement.md)
  — P4 ``control_dt`` derivation; Phase 5 will test the staleness
  matrix at multiple ``control_dt`` regimes.
* [logbook/2026-05-09-plant-interface-contract-phase-5-p1-p2-enforcement.md](2026-05-09-plant-interface-contract-phase-5-p1-p2-enforcement.md)
  — P1 ``PlantState`` aliasing discipline.  Phase 4's tests honour
  the alias-break rules established there: per-call ``copy()``
  snapshots before driving a fault (e.g.
  ``last_good_pos = plant._state.platform_pos_mm.copy()`` in
  T-U-T2a-1) rather than retaining a ``PlantState`` reference
  across ``get_state()`` calls.
* [controller/PLANT_INTERFACE_CONTRACT.md](../controller/PLANT_INTERFACE_CONTRACT.md)
  — P1–P4; the alias-break discipline that Phase 4's tests honour.
* [controller/hardware_plant.py](../controller/hardware_plant.py) —
  cascade implementations: FK divergence at ``:602–617``, singular
  Jacobian at ``:737–740``, frozen motor at ``:648–683``.
* [tests/sim/test_hardware_plant_failure_paths.py](../tests/sim/test_hardware_plant_failure_paths.py)
  — this phase's new test file.
* [tests/sim/_hardware_plant_stub.py](../tests/sim/_hardware_plant_stub.py)
  — extended with three injection helpers.
