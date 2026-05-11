"""Plan 2 Phase 4 (Tier 2a) — HardwarePlant FK degradation, singular
Jacobian, and frozen-motor detector.

This module exercises the three watchdog cascades on
``HardwarePlant.get_state()`` that protect the platform from oblivious-
MPC scenarios when motor telemetry degrades:

1. **FK divergence cascade.**  ``leg_lengths_to_pose`` raises
   ``RuntimeError`` when Newton-Raphson fails to converge within
   ``max_iter=10`` (``hardware_plant.py:580–584``).  The handler at
   ``:602–613`` returns the last-good pose, increments
   ``_fk_fail_count``, and fires ``estop(reason='fk_convergence_failure')``
   on the **5th** consecutive failure (``self._FK_FAIL_ESTOP_THRESHOLD``,
   instance attr at ``:217``).
2. **Singular-Jacobian zero-twist.**  When the platform Jacobian becomes
   rank-deficient at a workspace boundary, ``np.linalg.solve`` raises
   ``LinAlgError`` (``hardware_plant.py:737``); the handler logs a
   ONCE-only warning (``_jacobian_singular_warned`` flag at ``:740``)
   and leaves ``state.platform_twist`` zero (zeroed unconditionally at
   ``:715`` before the try/except).
3. **Frozen-motor detector.**  Bit-identical ``motor_pos`` payloads
   across consecutive *fresh* ticks (``drain_count > 0``) increment
   ``_frozen_motor_pos_count`` (``:659``).  Two thresholds fire:
   ``_FROZEN_MOTOR_POS_WARN = 20`` triggers a once-only warning
   (``:660–667``); ``_FROZEN_MOTOR_POS_ESTOP = 40`` triggers
   ``estop(reason='telemetry_frozen')`` (``:668–676``), gated on
   ``_fk_ever_succeeded`` so de-energised start-up frames cannot trip.

Test IDs from
[plans/active/mpc-sadpath-coverage-tiers-1-3.md](../../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
Phase 4:

| ID         | Surface                                            | Driver                                                              |
|------------|----------------------------------------------------|---------------------------------------------------------------------|
| T-U-T2a-1  | Single non-converged FK call                       | ``inject_fk_failure(n_ticks=1)``; assert cache-fallback             |
| T-U-T2a-2  | FK watchdog fires at threshold                     | ``inject_fk_failure(n_ticks=5)``; spy ``estop()``                   |
| T-U-T2a-3  | Counter resets within threshold                    | ``inject_fk_failure(n_ticks=4)`` + 1 clean tick                     |
| T-U-T2a-4  | Singular Jacobian → zero twist + once-only warn    | ``inject_singular_jacobian()``                                      |
| T-U-T2a-5  | Singular-J warning is once-only over 10 ticks      | ``inject_singular_jacobian()`` + ``_capture_hp_warnings``           |
| T-U-T2a-6a | Frozen-motor WARN at 20 consecutive ticks          | ``install_telemetry_pump`` with identical frames                    |
| T-U-T2a-6b | Frozen-motor ESTOP at 40 consecutive ticks         | ``install_telemetry_pump`` with identical frames                    |
| T-U-T2a-7  | Bit-different ``motor_pos`` (1 ULP) doesn't fire   | ``install_telemetry_pump`` with per-tick 1-ULP variation            |
| T-U-T2a-8  | Property: any FK-failure burst < threshold leaves  | ``RuleBasedStateMachine`` (succeed_tick, preconditioned fail_tick)  |
|            | ``_estop_requested == False``                      |                                                                     |

**Empirical-probe table (run 2026-05-11; recipes confirmed deterministic
on this commit):**

| Production symbol                  | Location                         | Driver / observation                       |
|------------------------------------|----------------------------------|--------------------------------------------|
| ``_FK_FAIL_ESTOP_THRESHOLD = 5``   | ``hardware_plant.py:217``         | Fires at exactly count=5 (not 4, not 6)    |
| ``_fk_fail_count`` reset on success| ``:593``                          | Single clean FK after 4 fails → count=0   |
| Cache-fallback pose                | ``:614–617``                      | bit-equal to last-good ``platform_pos_mm`` |
| ``_jacobian_singular_warned`` flag | ``:218`` (init), ``:740`` (set)   | Set on first LinAlgError; only one record  |
| ``_FROZEN_MOTOR_POS_WARN = 20``    | ``:193``                          | Warning + flag at exactly tick 20          |
| ``_FROZEN_MOTOR_POS_ESTOP = 40``   | ``:194``                          | ``estop('telemetry_frozen')`` at tick 40   |
| Frozen detector comparison         | ``:657`` (``np.array_equal``)     | BIT-EXACT; 1-ULP variation does not fire   |

Per Plan 2 Working Note #1 ("Drive real failures, not mocked ones"):

* FK injection patches ``controller.hardware_plant.leg_lengths_to_pose``
  (the BOUNDARY of the FK contract).  The production code's ``except
  RuntimeError`` at ``:602`` is the surface under test; the watchdog
  counter is observed, never poked.
* Singular-Jacobian injection patches ``compute_jacobian`` AND nulls the
  FK-supplied Jacobian path.  The production code's ``except
  np.linalg.LinAlgError`` at ``:737`` is the surface under test.
* Frozen-motor injection feeds identical ZMQ frames through the real
  ``recv_multipart`` plumbing.  The detector's bit-exact
  ``np.array_equal`` at ``:657`` is the surface under test.

See [logbook/2026-05-11-tier2a-hardware-plant-fk-degradation.md](../../logbook/2026-05-11-tier2a-hardware-plant-fk-degradation.md).
"""
from __future__ import annotations

import contextlib
import logging

import numpy as np
import pytest
from hypothesis import HealthCheck, settings, strategies as st
from hypothesis.stateful import (
    RuleBasedStateMachine, invariant, precondition, rule,
)

import controller.hardware_plant as _hp_mod
from tests.sim._hardware_plant_stub import (
    build_hardware_plant_stub,
    inject_fk_failure,
    inject_singular_jacobian,
    install_telemetry_pump,
)


# NOTE on logger noise: the FK / Jacobian / frozen-motor tests
# intentionally trigger many WARNINGs and ERRORs.  We do NOT silence
# ``controller.hardware_plant`` at module level because that would
# block pytest's ``caplog`` fixture from observing the once-only
# warnings the tests assert on (caplog uses the logger's effective
# level to gate emission, not just its own handler).  pytest captures
# log output and only surfaces it for failed tests, so the noise is
# invisible on the happy path.


# Hypothesis settings tuned to the stub fixture cost.  The T-U-T2a-8
# stateful machine reuses a module-scoped plant singleton (same pattern
# as Phase 3's T1cWarmStartIntegrityMachine) so per-example cost is one
# get_state(), not one plant construction.  HealthCheck.too_slow is
# suppressed because the per-example time is dominated by the real FK
# Newton-Raphson loop the production code runs on every succeed_tick.
_PHASE_4_SETTINGS = settings(
    suppress_health_check=(
        HealthCheck.too_slow,
        HealthCheck.data_too_large,
        HealthCheck.filter_too_much,
    ),
)


# =====================================================================
# Helpers
# =====================================================================

def _prime_plant(plant):
    """Run one successful ``get_state()`` so ``_fk_ever_succeeded`` flips
    True and ``_last_measured_pose`` is populated.  Both are gating
    preconditions for the watchdog cascade (see ``hardware_plant.py:608``
    for the FK-fail ESTOP gate, ``:668`` for the frozen-motor ESTOP gate).
    """
    plant.get_state()
    assert plant._fk_ever_succeeded, (
        "stub primed unsuccessfully — _fk_ever_succeeded should be True"
    )


class _ListHandler(logging.Handler):
    """Capture log records into a list — mirrors the helper Phase 2 uses
    in ``test_solver_failures.py::test_per_node_ik_budget_exhaustion`` to
    work around pytest's ``caplog`` not propagating to the
    ``controller.hardware_plant`` logger reliably across all environments.
    """

    def __init__(self):
        super().__init__()
        self.records: list[logging.LogRecord] = []

    def emit(self, record):
        self.records.append(record)


@contextlib.contextmanager
def _capture_hp_warnings():
    """Attach a ``_ListHandler`` to ``controller.hardware_plant`` for the
    duration of the ``with`` block.  Returns the records list.
    """
    logger = logging.getLogger('controller.hardware_plant')
    handler = _ListHandler()
    prior_level = logger.level
    logger.setLevel(logging.WARNING)
    logger.addHandler(handler)
    try:
        yield handler.records
    finally:
        logger.removeHandler(handler)
        logger.setLevel(prior_level)


class _EstopSpy:
    """Record-and-call-original wrapper around ``HardwarePlant.estop``.

    Lets the test verify (a) ``estop()`` was called exactly N times,
    (b) with the documented ``reason=`` argument, AND (c) the production
    code's state-mutation side-effect (``_estop_requested = True``)
    happened — without coupling to private flag-poking.  Compare to a
    pure ``MagicMock`` which would lose property (c).
    """

    def __init__(self, plant):
        self._plant = plant
        self._real = plant.estop
        self.calls: list[dict] = []
        plant.estop = self._spy

    def _spy(self, reason='hardware_plant'):
        self.calls.append({'reason': reason})
        return self._real(reason=reason)

    @property
    def call_count(self):
        return len(self.calls)

    @property
    def reasons(self):
        return [c['reason'] for c in self.calls]


# =====================================================================
# T-U-T2a-1, -2, -3: FK divergence cascade
# =====================================================================

class TestFkDivergenceCascade:
    """T-U-T2a-1, -2, -3 — FK convergence watchdog at
    ``hardware_plant.py:602–613``.

    The watchdog protects against the case where Newton-Raphson FK
    repeatedly fails to converge on the leg-extension feedback (e.g.
    near a workspace boundary, or under transient sensor noise).
    ``_FK_FAIL_ESTOP_THRESHOLD = 5`` consecutive failures fire an
    e-stop with ``reason='fk_convergence_failure'``.
    """

    def test_t2a_1_single_failure_returns_last_good_pose(self):
        """T-U-T2a-1 — one non-converged FK call.

        After a successful prime, inject one ``RuntimeError`` from the
        FK boundary.  The handler at ``hardware_plant.py:614–617``
        copies ``_last_measured_pose`` into the state buffers.
        ``_fk_fail_count`` is exactly 1; no e-stop.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            # Snapshot the last-good pose BEFORE injection — per P1
            # aliasing the state arrays are mutated on the next call,
            # so we must copy.
            last_good_pos = plant._state.platform_pos_mm.copy()
            last_good_rot = plant._state.platform_rot.copy()

            with inject_fk_failure(n_ticks=1):
                state = plant.get_state()

            assert plant._fk_fail_count == 1, (
                f"single FK failure should set count=1; got "
                f"{plant._fk_fail_count}"
            )
            assert not plant._estop_requested, (
                "single FK failure must NOT trigger e-stop"
            )
            np.testing.assert_array_equal(
                state.platform_pos_mm, last_good_pos,
                err_msg="FK failure should fall back to last-good pose",
            )
            np.testing.assert_array_equal(
                state.platform_rot, last_good_rot,
                err_msg="FK failure should fall back to last-good rot",
            )

    def test_t2a_2_threshold_fires_estop(self):
        """T-U-T2a-2 — exactly ``_FK_FAIL_ESTOP_THRESHOLD`` consecutive
        failures fire e-stop.

        Drives 5 real ``RuntimeError`` paths through ``get_state()``.
        Spies on ``estop()`` to capture both the call shape (reason=)
        and the state-mutation side-effect (_estop_requested).
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            spy = _EstopSpy(plant)
            threshold = plant._FK_FAIL_ESTOP_THRESHOLD
            assert threshold == 5, (
                f"production constant drifted: _FK_FAIL_ESTOP_THRESHOLD="
                f"{threshold}, expected 5 (see hardware_plant.py:217)"
            )

            with inject_fk_failure(n_ticks=threshold):
                for n in range(1, threshold + 1):
                    plant.get_state()
                    if n < threshold:
                        assert not plant._estop_requested, (
                            f"e-stop fired prematurely at tick {n} "
                            f"(threshold {threshold})"
                        )
                        assert plant._fk_fail_count == n, (
                            f"after {n} failures, count={plant._fk_fail_count}"
                        )

            assert plant._estop_requested, (
                f"after {threshold} consecutive FK failures, "
                f"_estop_requested must be True"
            )
            assert spy.call_count == 1, (
                f"estop() called {spy.call_count} times; expected exactly 1"
            )
            assert spy.reasons == ['fk_convergence_failure'], (
                f"estop reason wrong: {spy.reasons}"
            )

    def test_t2a_2_one_below_threshold_does_not_estop(self):
        """T-U-T2a-2 (boundary regression) — exactly
        ``_FK_FAIL_ESTOP_THRESHOLD - 1`` consecutive failures must NOT
        trigger e-stop.  Pair with the test above to pin the off-by-one
        boundary on both sides.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            spy = _EstopSpy(plant)
            threshold = plant._FK_FAIL_ESTOP_THRESHOLD

            with inject_fk_failure(n_ticks=threshold - 1):
                for _ in range(threshold - 1):
                    plant.get_state()

            assert plant._fk_fail_count == threshold - 1
            assert not plant._estop_requested, (
                f"after {threshold - 1} failures (one below threshold), "
                f"_estop_requested must remain False"
            )
            assert spy.call_count == 0, (
                f"estop() called {spy.call_count} times; expected 0"
            )

    def test_t2a_3_recovery_within_threshold_resets_counter(self):
        """T-U-T2a-3 — 4 consecutive failures followed by one success
        resets ``_fk_fail_count`` to zero per ``hardware_plant.py:593``.
        E-stop is never triggered.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            spy = _EstopSpy(plant)
            threshold = plant._FK_FAIL_ESTOP_THRESHOLD

            with inject_fk_failure(n_ticks=threshold - 1):
                for n in range(1, threshold):
                    plant.get_state()
                    assert plant._fk_fail_count == n

            # The context exits — FK reverts to real.  One clean tick.
            plant.get_state()

            assert plant._fk_fail_count == 0, (
                f"after recovery, count should reset to 0; got "
                f"{plant._fk_fail_count}"
            )
            assert not plant._estop_requested
            assert spy.call_count == 0


# =====================================================================
# T-U-T2a-4, -5: singular Jacobian → zero twist + once-only warning
# =====================================================================

class TestSingularJacobianZeroTwist:
    """T-U-T2a-4, -5 — singular-Jacobian path at
    ``hardware_plant.py:737–740``.

    The unconditional ``state.platform_twist.fill(0.0)`` at ``:715``
    means a singular Jacobian leaves twist at zero structurally; the
    test value is observing the ONCE-only ``logger.warning`` (gated by
    ``_jacobian_singular_warned`` at ``:740``).
    """

    def test_t2a_4_singular_jacobian_zeroes_twist_and_warns_once(self):
        """T-U-T2a-4 — a singular Jacobian zeroes ``platform_twist``
        and logs the warning exactly once per singular-streak.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            # Telemetry must publish non-zero motor_vel for the
            # twist-solve branch (hardware_plant.py:716) to fire.
            install_telemetry_pump(plant, motor_vel=[0.1] * 6)
            plant.get_state()  # warm the pump frame
            assert not plant._jacobian_singular_warned, (
                "pre-condition: _jacobian_singular_warned should be False"
            )

            with _capture_hp_warnings() as records, \
                    inject_singular_jacobian():
                state = plant.get_state()

            assert np.array_equal(state.platform_twist, np.zeros(6)), (
                f"singular J should zero platform_twist; got "
                f"{state.platform_twist}"
            )
            assert plant._jacobian_singular_warned, (
                "first singular tick must set _jacobian_singular_warned=True"
            )
            singular_records = [
                r for r in records
                if 'Jacobian singular' in r.getMessage()
            ]
            assert len(singular_records) == 1, (
                f"expected exactly 1 'Jacobian singular' record; got "
                f"{len(singular_records)}"
            )

    def test_t2a_5_singular_warning_is_once_only_over_10_ticks(self):
        """T-U-T2a-5 — 10 consecutive singular-Jacobian ticks emit the
        warning exactly once (the second-through-tenth tick are silent).
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            install_telemetry_pump(plant, motor_vel=[0.1] * 6)
            plant.get_state()

            with _capture_hp_warnings() as records, \
                    inject_singular_jacobian():
                for _ in range(10):
                    plant.get_state()

            singular_records = [
                r for r in records
                if 'Jacobian singular' in r.getMessage()
            ]
            assert len(singular_records) == 1, (
                f"expected exactly 1 'Jacobian singular' record across "
                f"10 ticks; got {len(singular_records)}"
            )


# =====================================================================
# T-U-T2a-6a, -6b, -7: frozen-motor detector
# =====================================================================

class TestFrozenMotorDetector:
    """T-U-T2a-6a, -6b, -7 — frozen-motor detector at
    ``hardware_plant.py:648–683``.

    Two thresholds: WARN at ``_FROZEN_MOTOR_POS_WARN = 20``
    (``hardware_plant.py:193``); ESTOP at ``_FROZEN_MOTOR_POS_ESTOP = 40``
    (``:194``).  Comparison uses ``np.array_equal`` (``:657``) — strict
    bit-equality; any nonzero per-leg difference resets the counter.
    """

    def test_t2a_6a_warn_at_twenty_consecutive_identical_ticks(self):
        """T-U-T2a-6a — exactly ``_FROZEN_MOTOR_POS_WARN`` identical
        ticks log the warning once and set the warned flag.  E-stop NOT
        triggered.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            warn_n = plant._FROZEN_MOTOR_POS_WARN
            estop_n = plant._FROZEN_MOTOR_POS_ESTOP
            assert warn_n == 20 and estop_n == 40, (
                f"frozen-motor constants drifted: WARN={warn_n}, "
                f"ESTOP={estop_n}; expected 20 / 40 (hardware_plant.py:193–194)"
            )

            install_telemetry_pump(plant)  # default: identical zero-vel frames

            with _capture_hp_warnings() as records:
                for _ in range(warn_n):
                    plant.get_state()

            assert plant._frozen_motor_pos_count == warn_n
            assert plant._frozen_motor_pos_warned, (
                "_frozen_motor_pos_warned should be True at the WARN edge"
            )
            assert not plant._estop_requested, (
                "WARN threshold must NOT trigger e-stop"
            )
            frozen_records = [
                r for r in records
                if 'motor_pos frozen' in r.getMessage()
            ]
            # Exactly one WARN-level record (the ESTOP record fires only
            # at the estop threshold).
            assert len(frozen_records) == 1, (
                f"expected exactly 1 'motor_pos frozen' WARN record at "
                f"the WARN edge; got {len(frozen_records)}"
            )

    def test_t2a_6b_estop_at_forty_consecutive_identical_ticks(self):
        """T-U-T2a-6b — exactly ``_FROZEN_MOTOR_POS_ESTOP`` identical
        ticks fire ``estop(reason='telemetry_frozen')``.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            spy = _EstopSpy(plant)
            estop_n = plant._FROZEN_MOTOR_POS_ESTOP

            install_telemetry_pump(plant)

            for _ in range(estop_n):
                plant.get_state()

            assert plant._frozen_motor_pos_count == estop_n
            assert plant._estop_requested
            assert spy.call_count == 1
            assert spy.reasons == ['telemetry_frozen'], (
                f"estop reason wrong: {spy.reasons}"
            )

    def test_t2a_7_bit_different_motor_pos_does_not_fire_detector(self):
        """T-U-T2a-7 — 44 ticks of ``motor_pos`` varying by 1 ULP per
        tick leave the detector at count=0 and the warning unfired.

        The detector uses ``np.array_equal`` at ``hardware_plant.py:657``
        which is bit-exact — ANY nonzero difference (down to a 1-ULP
        increment) resets the counter.  Note: this is a STRONGER
        false-positive avoidance than the plan's "differ by < float
        epsilon" phrasing implied; the actual production semantic is
        bit-exact, not epsilon-based.  No false-positive is possible
        from sub-LSB encoder noise because any noise is nonzero.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)

            def vary_one_ulp(tick_idx, motor_pos, motor_vel):
                # Add a vanishingly small per-tick delta to leg 0.  The
                # absolute magnitude is irrelevant — only that it is
                # nonzero.
                motor_pos[0] = motor_pos[0] + 1e-15 * tick_idx

            install_telemetry_pump(plant, frame_mutator=vary_one_ulp)

            with _capture_hp_warnings() as records:
                for _ in range(plant._FROZEN_MOTOR_POS_ESTOP + 4):
                    plant.get_state()

            assert plant._frozen_motor_pos_count == 0, (
                f"1-ULP-varying motor_pos must keep count=0; got "
                f"{plant._frozen_motor_pos_count}"
            )
            assert not plant._frozen_motor_pos_warned
            assert not plant._estop_requested
            frozen_records = [
                r for r in records
                if 'motor_pos frozen' in r.getMessage()
            ]
            assert len(frozen_records) == 0, (
                f"no 'motor_pos frozen' records should appear; got "
                f"{len(frozen_records)}"
            )


# =====================================================================
# T-U-T2a-8: Hypothesis stateful — FK failures < threshold never E-stop
# =====================================================================
#
# Module-scoped plant singleton + ExitStack — same pattern as Phase 3's
# T1cWarmStartIntegrityMachine.  The stub's MuJoCo IK pre-computation
# is a ~1-2s one-shot cost; reusing the plant across hypothesis
# examples keeps the property test bounded at ci-deep.
#
# UNCONDITIONAL state restoration in __init__ (per Phase 3's documented
# FlakyStrategyDefinition discipline) — a prior walk's _fk_fail_count
# / _estop_requested / _frozen_motor_pos_count cannot bleed into data
# generation.

_T2A_PROP_PLANT = None
_T2A_PROP_STACK = None


def _t2a_get_plant():
    global _T2A_PROP_PLANT, _T2A_PROP_STACK
    if _T2A_PROP_PLANT is None:
        _T2A_PROP_STACK = contextlib.ExitStack()
        _T2A_PROP_PLANT = _T2A_PROP_STACK.enter_context(
            build_hardware_plant_stub()
        )
        _T2A_PROP_PLANT.get_state()  # prime _fk_ever_succeeded
    return _T2A_PROP_PLANT


class T2aFkBurstBelowThresholdMachine(RuleBasedStateMachine):
    """T-U-T2a-8 — invariant: any sequence of FK failures whose
    longest **consecutive** run is strictly shorter than
    ``_FK_FAIL_ESTOP_THRESHOLD`` never fires the e-stop.

    Per-tick rules with a ``precondition`` that gates the
    failure-injection rule when the next failure would push the
    consecutive-failure run to THRESHOLD.  This implements the property
    directly: the strategy can never propose a path that legitimately
    fires the e-stop, so any observed e-stop is a violation.

    **Rules:**

    * ``succeed_tick`` — one real ``get_state()`` with FK ground-truth.
      Per ``hardware_plant.py:593`` this resets the production counter
      AND our shadow ``_shadow_consec`` to 0.
    * ``fail_tick`` (preconditioned) — drive ONE real ``RuntimeError``
      path through ``get_state()``.  Gated on
      ``self._shadow_consec < THRESHOLD - 1`` so the strategy never
      proposes the THRESHOLD-th consecutive failure (which would
      legitimately fire the e-stop).

    **Invariants:**

    * ``estop_never_fires`` — ``_estop_requested`` is False after every
      rule (any True is a regression of the cascade's safety property).
    * ``fk_count_matches_shadow`` — production ``_fk_fail_count`` tracks
      the machine's shadow counter exactly.  Catches counter corruption
      from a future production refactor.

    **Why per-tick over per-burst:** the original design used a
    ``fail_burst(n)`` rule with ``n in [0, 4]``.  Hypothesis correctly
    found that *back-to-back* bursts can sum past threshold
    (``burst(3) → burst(2) → count=5 → e-stop``) — a legitimate
    cascade firing, not a bug.  The per-tick + precondition pattern
    expresses the property's actual precondition (consecutive-run
    length < threshold) rather than the per-burst surface that
    permitted accumulation.

    **Why ``RuleBasedStateMachine`` over ``@composite + @given``:** the
    invariant is across-call — a single hypothesis input couldn't
    distinguish "5 failures spread over 3 walks" (safe) from
    "5 failures consecutive" (unsafe).  Phase 3's logbook documents the
    same choice for T-U-T1c-7.

    **State-restoration discipline:** the plant singleton's watchdog
    fields are reset UNCONDITIONALLY in ``__init__`` (per Phase 3's
    FlakyStrategyDefinition discipline) so a prior walk's state cannot
    bleed into the next walk's strategy generation.
    """

    def __init__(self):
        super().__init__()
        self._plant = _t2a_get_plant()
        # Unconditional state reset — SETUP, not verification.
        self._plant._fk_fail_count = 0
        self._plant._estop_requested = False
        self._plant._frozen_motor_pos_count = 0
        self._plant._frozen_motor_pos_warned = False
        self._plant._jacobian_singular_warned = False
        # Install a per-tick varying telemetry pump so the
        # frozen-motor detector (hardware_plant.py:648–683) does NOT
        # trip during long walks.  The default stub pump returns
        # byte-identical frames every tick, which at the 40-tick mark
        # would fire ``estop(reason='telemetry_frozen')`` — an
        # orthogonal cascade that would mask whatever the FK property
        # is doing.  A 1-ULP per-tick variation keeps the frozen
        # detector at count=0 (the np.array_equal comparison at
        # :657 is bit-exact; see T-U-T2a-7).
        install_telemetry_pump(
            self._plant,
            frame_mutator=lambda tick, mp, mv: (
                mp.__setitem__(0, mp[0] + 1e-15 * tick)
            ),
        )
        # Shadow consecutive-failure counter — kept in lockstep with
        # the production ``_fk_fail_count`` by the rules below.  The
        # precondition reads this rather than the production counter
        # to avoid hypothesis observing private production state when
        # picking rule applicability.
        self._shadow_consec = 0
        # One clean tick to re-seed ``_last_measured_pose`` against
        # the (now per-tick varying) pump frame.
        self._plant.get_state()

    @rule()
    def succeed_tick(self):
        """One clean ``get_state()`` — resets _fk_fail_count per :593."""
        self._plant.get_state()
        self._shadow_consec = 0

    @precondition(lambda self:
                  self._shadow_consec
                  < self._plant._FK_FAIL_ESTOP_THRESHOLD - 1)
    @rule()
    def fail_tick(self):
        """One real ``RuntimeError`` FK path, gated below threshold.

        Precondition keeps the consecutive-failure run strictly below
        THRESHOLD-1 *before* the tick; after the tick the count is at
        most THRESHOLD-1, never THRESHOLD.  E-stop fires only at
        count == THRESHOLD, so the precondition makes the cascade
        unreachable by construction — any observed e-stop is a
        production-code violation of the watchdog's documented
        threshold semantics.
        """
        with inject_fk_failure(n_ticks=1):
            self._plant.get_state()
        self._shadow_consec += 1

    @invariant()
    def estop_never_fires(self):
        assert not self._plant._estop_requested, (
            "T-U-T2a-8 invariant violation: e-stop fired despite all "
            "consecutive-failure runs being strictly below threshold "
            f"(_FK_FAIL_ESTOP_THRESHOLD="
            f"{self._plant._FK_FAIL_ESTOP_THRESHOLD}, shadow_consec="
            f"{self._shadow_consec})"
        )

    @invariant()
    def fk_count_matches_shadow(self):
        c = self._plant._fk_fail_count
        assert c == self._shadow_consec, (
            f"_fk_fail_count={c} drifted from shadow="
            f"{self._shadow_consec} — counter / reset semantics broken"
        )


T2aFkBurstBelowThresholdMachine.TestCase.settings = _PHASE_4_SETTINGS

TestT2aFkBurstBelowThreshold = T2aFkBurstBelowThresholdMachine.TestCase
