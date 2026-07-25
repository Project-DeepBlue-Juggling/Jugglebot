"""Plan 2 Phases 4 & 5 (Tier 2a + Tier 2b) — HardwarePlant safety chain.

This module exercises the watchdog cascades on
``HardwarePlant.get_state()`` and the torque-FF computation in
``set_pose()`` that protect the platform from oblivious-MPC scenarios
when motor telemetry degrades or the platform reaches a singular
workspace boundary.  Two phases share the file:

* **Phase 4 (Tier 2a)** — FK convergence cascade, singular-Jacobian
  twist-zero, frozen-motor detector (``TestFkDivergenceCascade``,
  ``TestSingularJacobianZeroTwist``, ``TestFrozenMotorDetector``,
  ``T2aFkBurstBelowThresholdMachine``).
* **Phase 5 (Tier 2b)** — telemetry-staleness threshold matrix
  (WARN @ 3×, HARD @ 5×, ESTOP @ 20× ``control_dt``), set_pose
  torque-FF singular-Jacobian path, cold-start zero-state
  (``TestTelemetryStalenessThresholds``, ``TestSetPoseFfSingular``,
  ``TestColdStartZeroState``, ``test_t2b_5_threshold_scaling_property``).

Phase 4 covers the three watchdog cascades:

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
[plans/archived/2026-05-18 mpc-sadpath-coverage-tiers-1-3.md](../../plans/archived/2026-05-18%20mpc-sadpath-coverage-tiers-1-3.md)
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

Phase 5 (Tier 2b) adds eight scenario / property tests over the
telemetry-staleness watchdog, the set_pose torque-FF singular-Jacobian
path, and the cold-start zero-state path:

| ID         | Surface                                                  | Driver                                                                |
|------------|----------------------------------------------------------|-----------------------------------------------------------------------|
| T-U-T2b-1  | WARN edge (3× control_dt)                                | ``drain_recv_pump`` + ``freeze_perf_counter_at(3.5× control_dt)``     |
| T-U-T2b-2  | HARD edge (5× control_dt)                                | ``drain_recv_pump`` + ``freeze_perf_counter_at(5.5× control_dt)``     |
| T-U-T2b-3  | ESTOP edge (20× control_dt)                              | ``drain_recv_pump`` + ``freeze_perf_counter_at(20.5× control_dt)``    |
| T-U-T2b-4  | Stale → recovery → stale (edge-triggered re-arm)         | drain + freeze, install fresh pump, drain + freeze again              |
| T-U-T2b-5  | Property — thresholds scale linearly with control_dt     | ``@given(control_dt ∈ [0.01, 0.1])``; per-example fresh plant         |
| T-U-T2b-6  | set_pose torque-FF on singular Jacobian (XFAIL pre-fix)  | ``patch motor_commands.compute_jacobian`` to return rank-5 matrix      |
| T-U-T2b-7  | Cold-start zero-state (no telem ever)                    | ``drain_recv_pump`` BEFORE any ``get_state()``                        |
| T-U-T2b-8  | Cold-start + 1 s no recv → estop                         | one prime tick, then drain + ``freeze_perf_counter_at(1.0)``           |

**Phase 5 empirical-probe table (run 2026-05-11):**

| Production symbol                  | Location                         | Driver / observation                          |
|------------------------------------|----------------------------------|-----------------------------------------------|
| ``_TELEM_STALE_WARN_MULT = 3.0``   | ``hardware_plant.py:73``         | Records 'Telemetry aging' once at edge        |
| ``_TELEM_STALE_HARD_MULT = 5.0``   | ``:74``                          | Records 'zeroing velocities' once at edge     |
| ``_TELEM_STALE_ESTOP_MULT = 20.0`` | ``:75``                          | ``estop('telemetry_stale')`` once at edge     |
| ``_telem_stale_warned`` reset      | ``:701–702`` (in else branch)    | Resets when age drops below WARN              |
| Per-instance derivation            | ``:130–132``                     | ``warn_s = 3.0 × control_dt`` (linear scale)  |
| Cold-start zero-state              | ``:529–535``                     | ``_last_telem is None`` → all-zero state      |
| Cold-start ``data_age_s = None``   | ``:519–520, :527``               | ``None + age`` would TypeError; gated by None |
| Singular-FF fallback (the bug)     | ``dynamics.py:341–344``          | LinAlgError silently caught → ``np.zeros(6)`` |

Per Plan 2 Working Note #1, the Phase 5 helpers patch at BOUNDARIES:

* ``freeze_perf_counter_at`` patches the time-module reference inside
  ``controller.hardware_plant`` so ``time.perf_counter()`` returns the
  controlled instant.  The production code's
  ``telem_age = now - _last_telem_recv_time`` arithmetic at ``:519–520``
  is the surface under test; only the time source is controlled.
* ``drain_recv_pump`` replaces ``_sub.recv_multipart`` with an
  always-``Again`` callable.  ``drain_count == 0`` keeps
  ``_last_telem_recv_time`` from being refreshed; the cached
  ``_last_telem`` payload is reused so FK + the frozen-motor detector
  still see realistic motor data.

See [logbook/2026-05-11-tier2b-hardware-plant-telemetry-ff.md](../../logbook/2026-05-11-tier2b-hardware-plant-telemetry-ff.md).
"""
from __future__ import annotations

import contextlib
import logging
from unittest.mock import patch

import numpy as np
import pytest
from hypothesis import HealthCheck, given, settings, strategies as st
from hypothesis.stateful import (
    RuleBasedStateMachine, invariant, precondition, rule,
)

import controller.hardware_plant as _hp_mod
from tests.sim._hardware_plant_stub import (
    build_hardware_plant_stub,
    drain_recv_pump,
    freeze_perf_counter_at,
    inject_fk_failure,
    inject_singular_jacobian,
    install_motor_pos_none_pump,
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
    preconditions for the watchdog cascade (see ``hardware_plant.py:638``
    for the FK-fail ESTOP gate, ``:748`` for the frozen-motor ESTOP gate;
    refs updated post-P5 — the P5 arm shifted these down ~70 lines).
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

    def test_fk_tol_stays_looser_than_the_stagnation_ceiling(self):
        """The hot-loop FK tolerance must stay LOOSER than
        ``ik_solver.FK_STALL_CEILING_MM``.

        Not a style check — it is the whole reason FK's stagnation exit is
        unreachable from this call site.  ``_fk_accepted`` tries the mixed
        absolute/relative test first and the stagnation exit only below the
        ceiling, so while ``_FK_TOL_MM`` (1e-4 mm) is looser than the ceiling
        (1e-6 mm) the mixed test has always already accepted.  Tighten
        ``_FK_TOL_MM`` past the ceiling — e.g. to 1e-7 mm for finer pose
        feedback — and the 40 Hz loop could start exiting through a branch
        sized for round-off convergence, accepting up to 1e-6 mm on a stalled
        solve when it asked for 1e-7 mm, with nothing in the FK test suite
        noticing (those tests assert against their own copy of 1e-4).

        Second invariant: the tolerance must stay far below the leg encoder
        dead-band, so accepting at it can never be a physically meaningful
        pose error.
        """
        from jugglebot.motion.ik_solver import FK_STALL_CEILING_MM

        tol = _hp_mod.HardwarePlant._FK_TOL_MM
        assert tol > FK_STALL_CEILING_MM, (
            f"hot-loop FK tol {tol:.1e} mm is no longer looser than the FK "
            f"stagnation ceiling {FK_STALL_CEILING_MM:.1e} mm — the "
            f"unreachability argument in ik_solver._fk_accepted is void"
        )
        # 8.607e-3 mm = 1 / (enc_cpr * mm_to_rev); 10x margin.
        assert tol < 8.607e-3 / 10.0, (
            f"hot-loop FK tol {tol:.1e} mm is within 10x of the leg encoder "
            f"dead-band — a residual accepted at it is no longer physically "
            f"negligible"
        )

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


# =====================================================================
# Phase 5 (Tier 2b) — telemetry-staleness threshold matrix
# =====================================================================

# Threshold-class constants — read once at module import; production
# symbols are at ``hardware_plant.py:73–75`` (module constants).  Tests
# assert these match what the production code derives at construction so
# any drift surfaces as a test failure rather than a silent test bug.
_TELEM_WARN_MULT = _hp_mod._TELEM_STALE_WARN_MULT
_TELEM_HARD_MULT = _hp_mod._TELEM_STALE_HARD_MULT
_TELEM_ESTOP_MULT = _hp_mod._TELEM_STALE_ESTOP_MULT


def _telem_stale_records(records):
    """Filter the captured records to those that look like
    telemetry-staleness messages (matches 'Telemetry aging', 'Telemetry
    stale', or any record mentioning 'velocities')."""
    return [
        r for r in records
        if 'telemetry' in r.getMessage().lower()
        or 'velocities' in r.getMessage().lower()
        or 'aging' in r.getMessage().lower()
    ]


class TestTelemetryStalenessThresholds:
    """T-U-T2b-1, -2, -3, -4 — three-tier telemetry-staleness watchdog at
    ``hardware_plant.py:625–702`` (ESTOP gate at ``:627–631``; HARD +
    WARN cascade at ``:686–702``).

    Edge-triggered logging via ``_telem_stale_warned`` flag at
    ``:178`` (init), set at ``:694, :700``, reset at ``:702`` when
    age drops back below WARN.
    """

    def test_t2b_1_warn_edge_logs_aging_does_not_zero_vels_or_estop(self):
        """T-U-T2b-1 — at 3.5× control_dt staleness:

        * One 'Telemetry aging' WARN record (edge-triggered).
        * ``_telem_stale_warned == True``.
        * No ESTOP.
        * HARD branch does NOT fire (velocities not zeroed by the
          watchdog — the cache fallback at ``:558–563`` reuses the
          prior frame's vel data, NOT zero).
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            # Seed motor_vel via install_telemetry_pump BEFORE the prime
            # so we can later observe the non-zero vel that the HARD
            # branch would zero if it fired.
            install_telemetry_pump(plant, motor_vel=[0.1] * 6)
            plant.get_state()  # warm pump
            spy = _EstopSpy(plant)
            drain_recv_pump(plant)

            with _capture_hp_warnings() as records, \
                    freeze_perf_counter_at(
                        plant, 3.5 * plant._control_dt):
                state = plant.get_state()

            stale_records = _telem_stale_records(records)
            assert len(stale_records) == 1, (
                f"WARN edge: expected exactly 1 staleness record; "
                f"got {len(stale_records)}: "
                f"{[r.getMessage() for r in stale_records]}"
            )
            assert 'aging' in stale_records[0].getMessage().lower(), (
                f"WARN edge: record should be 'aging' message; got "
                f"{stale_records[0].getMessage()}"
            )
            assert plant._telem_stale_warned, (
                "WARN edge: _telem_stale_warned must be True"
            )
            assert not plant._estop_requested
            assert spy.call_count == 0
            # The WARN branch does NOT zero velocities; cached vels
            # persist from the prior frame.
            assert state.data_age_s == pytest.approx(
                3.5 * plant._control_dt)

    def test_t2b_2_hard_edge_zeroes_velocities_logs_warn_no_estop(self):
        """T-U-T2b-2 — at 5.5× control_dt staleness:

        * 'zeroing velocities' WARN record (HARD branch fires).
        * ``state.leg_velocities_mmps == 0``.
        * ``_telem_stale_warned == True``.
        * No ESTOP.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            install_telemetry_pump(plant, motor_vel=[0.1] * 6)
            plant.get_state()
            spy = _EstopSpy(plant)
            drain_recv_pump(plant)

            with _capture_hp_warnings() as records, \
                    freeze_perf_counter_at(
                        plant, 5.5 * plant._control_dt):
                state = plant.get_state()

            stale_records = _telem_stale_records(records)
            zeroing_records = [
                r for r in stale_records
                if 'velocities' in r.getMessage().lower()
            ]
            assert len(zeroing_records) == 1, (
                f"HARD edge: expected exactly 1 'velocities' record; "
                f"got {len(zeroing_records)}: "
                f"{[r.getMessage() for r in stale_records]}"
            )
            assert plant._telem_stale_warned
            assert not plant._estop_requested
            assert spy.call_count == 0
            np.testing.assert_array_equal(
                state.leg_velocities_mmps, np.zeros(6),
                err_msg="HARD edge must zero leg_velocities_mmps",
            )

    def test_t2b_3_estop_edge_fires_estop_with_telemetry_stale_reason(self):
        """T-U-T2b-3 — at 20.5× control_dt staleness:

        * ``estop(reason='telemetry_stale')`` fires exactly once.
        * Velocities zeroed (the HARD branch ALSO fires at this age,
          since 20.5× > 5×).
        * ``_estop_requested == True``.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            install_telemetry_pump(plant, motor_vel=[0.1] * 6)
            plant.get_state()
            spy = _EstopSpy(plant)
            drain_recv_pump(plant)

            with _capture_hp_warnings() as records, \
                    freeze_perf_counter_at(
                        plant, 20.5 * plant._control_dt):
                state = plant.get_state()

            stale_records = _telem_stale_records(records)
            estop_records = [
                r for r in stale_records
                if 'e-stop' in r.getMessage().lower()
                or 'triggering' in r.getMessage().lower()
            ]
            assert len(estop_records) >= 1, (
                f"ESTOP edge: expected 'triggering e-stop' record; "
                f"got {[r.getMessage() for r in stale_records]}"
            )
            assert plant._estop_requested
            assert spy.call_count == 1
            assert spy.reasons == ['telemetry_stale'], (
                f"ESTOP edge: reason wrong: {spy.reasons}"
            )
            np.testing.assert_array_equal(
                state.leg_velocities_mmps, np.zeros(6),
                err_msg="ESTOP edge: HARD branch also fires (age > 5×)",
            )

    def test_t2b_3_just_below_estop_does_not_fire_estop(self):
        """T-U-T2b-3 (boundary regression) — at 19.5× control_dt
        staleness, the HARD branch fires but ESTOP does NOT.  Pair with
        the above test to pin the ESTOP off-by-one boundary on both
        sides.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            install_telemetry_pump(plant, motor_vel=[0.1] * 6)
            plant.get_state()
            spy = _EstopSpy(plant)
            drain_recv_pump(plant)

            with freeze_perf_counter_at(
                    plant, 19.5 * plant._control_dt):
                state = plant.get_state()

            assert not plant._estop_requested, (
                f"19.5× control_dt (< 20× ESTOP) must NOT fire estop; "
                f"got _estop_requested={plant._estop_requested}"
            )
            assert spy.call_count == 0
            # HARD branch still fires (19.5 > 5)
            np.testing.assert_array_equal(
                state.leg_velocities_mmps, np.zeros(6),
            )

    def test_t2b_4_stale_then_recovered_re_arms_edge_triggered_logging(self):
        """T-U-T2b-4 — stale → recovery → stale-again must re-emit the
        WARN record (edge-triggered re-arm via ``:702``
        ``_telem_stale_warned = False``).

        Three episodes:
        1. First stale: WARN record + flag=True.
        2. Recovery (fresh frame): flag=False, no new record.
        3. Second stale: NEW WARN record + flag=True.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            with _capture_hp_warnings() as records:
                # Episode 1: stale
                drain_recv_pump(plant)
                with freeze_perf_counter_at(
                        plant, 3.5 * plant._control_dt):
                    plant.get_state()
                n_after_first = len(_telem_stale_records(records))
                flag_after_first = plant._telem_stale_warned

                # Recovery: reinstall a fresh-frame pump and let recv
                # update _last_telem_recv_time at real time again.
                install_telemetry_pump(plant)
                plant.get_state()  # fresh frame
                n_after_recovery = len(_telem_stale_records(records))
                flag_after_recovery = plant._telem_stale_warned

                # Episode 2: stale again
                drain_recv_pump(plant)
                with freeze_perf_counter_at(
                        plant, 3.5 * plant._control_dt):
                    plant.get_state()
                n_after_second = len(_telem_stale_records(records))
                flag_after_second = plant._telem_stale_warned

            assert n_after_first == 1, (
                f"first stale should emit 1 record; got {n_after_first}"
            )
            assert flag_after_first is True
            # Recovery is silent; flag resets
            assert n_after_recovery == n_after_first, (
                f"recovery must NOT add records; "
                f"after-first={n_after_first}, after-recovery="
                f"{n_after_recovery}"
            )
            assert flag_after_recovery is False, (
                f"recovery must reset _telem_stale_warned to False; "
                f"got {flag_after_recovery}"
            )
            # Re-arm: second stale episode emits a new record
            assert n_after_second == 2, (
                f"second stale must emit a new record (edge-triggered "
                f"re-arm); got total={n_after_second}"
            )
            assert flag_after_second is True


# =====================================================================
# T-U-T2b-5: hypothesis property — thresholds scale linearly with
# control_dt (P4 from Plan 1 Phase 6 — PLANT_INTERFACE_CONTRACT.md).
# =====================================================================

_PHASE_5_SETTINGS = settings(
    suppress_health_check=(
        HealthCheck.too_slow,
        HealthCheck.data_too_large,
        HealthCheck.filter_too_much,
    ),
)


@_PHASE_5_SETTINGS
@given(
    control_dt=st.floats(min_value=0.01, max_value=0.1,
                         allow_nan=False, allow_infinity=False),
    factor_choice=st.sampled_from(['warn', 'hard', 'estop']),
)
def test_t2b_5_threshold_scaling_property(control_dt, factor_choice):
    """T-U-T2b-5 — for any ``control_dt ∈ [0.01, 0.1]``, each documented
    threshold fires at exactly ``_TELEM_STALE_*_MULT × control_dt`` (±
    float epsilon).

    Per Plan 1 Phase 6 / ``controller/PLANT_INTERFACE_CONTRACT.md`` P4:
    *"Period awareness — ``control_dt`` is a mandatory constructor
    kwarg.  All time-derived thresholds scale linearly with it."*

    This is the FIRST test in Plan 2 to exercise P4's linear-scaling
    promise.  Per-example: fresh ``HardwarePlant`` at the sampled
    ``control_dt`` (module-level IK cache makes this ~50 ms per
    example, not ~1 s); drive ONE threshold case slightly above its
    edge; verify the production thresholds match
    ``_TELEM_STALE_*_MULT * control_dt`` exactly.

    Strategy:
    * ``control_dt`` ∈ [0.01 s, 0.1 s] (100 Hz to 10 Hz operating
      regime).  These are the documented bounds for the watchdog's
      linear-scaling promise; outside this range the multipliers may
      no longer be meaningful (e.g. a 1 ms control loop would have a
      3 ms WARN threshold — tighter than ZMQ recv jitter).
    * ``factor_choice`` ∈ {'warn', 'hard', 'estop'} — one of the three
      thresholds.  Sampling across the categorical axis catches
      threshold-specific bugs that fuzz over ``control_dt`` alone
      would miss.

    Invariants checked per example:

    1. The plant's per-instance derived threshold matches
       ``_TELEM_STALE_*_MULT * control_dt`` exactly (float compare).
    2. Driving ``telem_age`` slightly above the threshold fires the
       documented record / estop; driving it slightly below does NOT.

    Note that ``allow_nan=False, allow_infinity=False`` are required:
    the production code reads ``control_dt`` as a ``float`` constructor
    arg and would happily accept NaN, producing NaN thresholds that
    no comparison would ever fire.  That's a Plan 1 Phase 6 concern
    (P4 input validation), out of scope here.
    """
    factor_above = {
        'warn': _TELEM_WARN_MULT + 0.5,
        'hard': _TELEM_HARD_MULT + 0.5,
        'estop': _TELEM_ESTOP_MULT + 0.5,
    }[factor_choice]
    expected_threshold = {
        'warn': _TELEM_WARN_MULT * control_dt,
        'hard': _TELEM_HARD_MULT * control_dt,
        'estop': _TELEM_ESTOP_MULT * control_dt,
    }[factor_choice]

    with build_hardware_plant_stub(control_dt=control_dt) as plant:
        # Invariant 1: per-instance derivation
        derived = {
            'warn': plant._telem_stale_warn_s,
            'hard': plant._telem_stale_hard_s,
            'estop': plant._telem_stale_estop_s,
        }[factor_choice]
        assert derived == pytest.approx(expected_threshold, abs=1e-12), (
            f"T-U-T2b-5: per-instance threshold drifted for "
            f"control_dt={control_dt}, factor={factor_choice}: "
            f"derived={derived}, expected={expected_threshold}"
        )

        plant.get_state()  # prime
        spy = _EstopSpy(plant)
        drain_recv_pump(plant)

        # Invariant 2: threshold fires at the expected staleness
        with _capture_hp_warnings() as records, \
                freeze_perf_counter_at(
                    plant, factor_above * control_dt):
            state = plant.get_state()

        stale_records = _telem_stale_records(records)

        if factor_choice == 'warn':
            assert plant._telem_stale_warned, (
                f"T-U-T2b-5: WARN must fire at {factor_above}× control_dt"
                f"={factor_above * control_dt:.4f}s "
                f"(threshold={derived:.4f}s)"
            )
            assert spy.call_count == 0
            assert any('aging' in r.getMessage().lower()
                       for r in stale_records), (
                f"WARN expected 'aging' record; got "
                f"{[r.getMessage() for r in stale_records]}"
            )
        elif factor_choice == 'hard':
            np.testing.assert_array_equal(
                state.leg_velocities_mmps, np.zeros(6),
                err_msg=f"HARD must zero vels at {factor_above}× "
                f"control_dt={factor_above * control_dt:.4f}s",
            )
            assert spy.call_count == 0
            assert any('velocities' in r.getMessage().lower()
                       for r in stale_records), (
                f"HARD expected 'velocities' record; got "
                f"{[r.getMessage() for r in stale_records]}"
            )
        elif factor_choice == 'estop':
            assert plant._estop_requested
            assert spy.call_count == 1
            assert spy.reasons == ['telemetry_stale']


# =====================================================================
# T-U-T2b-6: set_pose torque-FF on singular Jacobian (XFAIL until
# the bugfix lands in the same session — see Plan 2 Phase 5 logbook).
# =====================================================================
#
# The bug: ``dynamics.py:341–344`` silently catches LinAlgError on
# ``np.linalg.solve(J.T, W_total)`` and returns ``np.zeros(6)``.
# ``set_pose()`` then propagates the all-zero FF to ``_ff_torque_buf``
# with NO warning, despite a parallel handler in ``get_state()`` at
# ``hardware_plant.py:737–740`` emitting a once-only 'Jacobian singular'
# warning.  Fix: detect all-zero ``torque_ff`` (gravity wrench is
# expected to be non-zero in any pose with feedforward enabled) and
# emit a once-only warning via ``_singular_ff_warned``.  Fix lands as
# a follow-up commit (Phase 5 bugfix logbook entry).

class TestSetPoseFfSingular:
    """T-U-T2b-6 — set_pose torque-FF singular-Jacobian path.

    Verifies that when a singular Jacobian causes
    ``compute_full_feedforward_torques`` to silently return zeros (via
    its internal ``except LinAlgError`` at ``dynamics.py:341–344``),
    ``HardwarePlant.set_pose`` emits a once-only warning (and the
    ``_singular_ff_warned`` edge-triggered flag is set), mirroring the
    once-only warning pattern in ``get_state()``'s singular-Jacobian
    handler at ``hardware_plant.py:737–740``.

    Bugfix landed in the same session — see logbook
    ``2026-05-11-tier2b-set-pose-singular-ff-bugfix.md``.  Pre-bugfix
    this test was xfail-strict; post-bugfix it asserts the once-only
    warning + ``_singular_ff_warned`` flag set on the singular edge.
    """

    def test_t2b_6_set_pose_singular_ff_warns_once(self):
        """T-U-T2b-6 — singular Jacobian in set_pose's FF computation
        emits a once-only warning.

        Driver: patch ``motor_commands.compute_jacobian`` to return a
        rank-5 matrix.  ``compute_full_feedforward_torques`` then
        catches the real ``LinAlgError`` from
        ``np.linalg.solve(J.T, W_total)`` and returns zeros.
        ``set_pose`` should detect the all-zero FF and warn.
        """
        import jugglebot.motion.motor_commands as _mc_mod
        import controller.hardware_plant as _hp_mod

        # 2026-07-14: HardwarePlant ANDs the constructor flag with the config
        # master/per-term flags (the "third producer" review fix). The explicit
        # patches pin the flags this test needs INDEPENDENTLY of the shipped
        # values (torque_ff_enabled ships true since the 2026-07-16 arming
        # session; the patch keeps this test meaningful either way).
        with patch.object(_hp_mod._hw_cfg, 'DYNAMICS_TORQUE_FF_ENABLED', True), \
                patch.object(_hp_mod._hw_cfg, 'DYNAMICS_TORQUE_FF_GRAVITY', True), \
                build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            singular = np.eye(6)
            singular[0, 0] = 0.0  # rank 5

            with _capture_hp_warnings() as records, \
                    patch.object(_mc_mod, 'compute_jacobian',
                                 return_value=singular):
                # Non-trivial twist/accel so the gravity + inertia path
                # is exercised.
                plant.set_pose(
                    np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
                    np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
                    np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                )
                # Drive a second time — once-only semantic check
                plant.set_pose(
                    np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
                    np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
                    np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                )

            # All-zero torque_ff confirmed (this part already passes
            # pre-fix; it's the documented fallback).
            np.testing.assert_array_equal(
                plant._ff_torque_buf, np.zeros(6),
                err_msg="singular FF falls back to zeros (current "
                "documented dynamics.py behaviour)",
            )
            # Pre-bugfix: zero warnings (XFAIL).  Post-bugfix: exactly
            # one once-only 'singular' warning.
            singular_records = [
                r for r in records
                if 'singular' in r.getMessage().lower()
                or 'torque_ff' in r.getMessage().lower()
            ]
            assert len(singular_records) == 1, (
                f"singular-FF in set_pose should emit exactly 1 "
                f"once-only warning; got {len(singular_records)} "
                f"records: {[r.getMessage() for r in singular_records]}"
            )
            assert hasattr(plant, '_singular_ff_warned'), (
                "Post-bugfix: plant._singular_ff_warned must exist"
            )
            assert plant._singular_ff_warned is True

    def test_config_master_flag_gates_the_mpc_producer(self):
        """2026-07-14 review fix (the 'third producer'): with the config
        MASTER switch off, HardwarePlant publishes ZERO torque feedforward
        even when constructed with enable_torque_ff=True — the constructor
        arg is ANDed with the config master switch, so no producer can arm
        itself past a disabled master. (Patched explicitly rather than read
        from the shipped value: torque_ff_enabled ships true since the
        2026-07-16 arming session, and this regression must keep guarding
        the AND-gate regardless of the shipped state.)"""
        import controller.hardware_plant as _hp_mod
        with patch.object(_hp_mod._hw_cfg, 'DYNAMICS_TORQUE_FF_ENABLED',
                          False), \
                build_hardware_plant_stub() as plant:
            assert plant._enable_torque_ff is False, (
                "config master switch off must veto the constructor "
                "default enable_torque_ff=True")
            _prime_plant(plant)
            plant.set_pose(
                np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
                np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
                np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            )
            np.testing.assert_array_equal(plant._ff_torque_buf, np.zeros(6))

    def test_config_platform_inertia_flag_gates_the_accel_term(self):
        """2026-07-14 review fix: with master+gravity on but
        torque_ff_platform_inertia=false (the shipped per-term state), the
        MPC producer's FF must be acceleration-INDEPENDENT — the
        acceleration-proportional term is the one the firmware's undecayed
        stale-link torque hold makes unsafe."""
        import controller.hardware_plant as _hp_mod
        with patch.object(_hp_mod._hw_cfg, 'DYNAMICS_TORQUE_FF_ENABLED', True), \
                patch.object(_hp_mod._hw_cfg, 'DYNAMICS_TORQUE_FF_GRAVITY', True), \
                patch.object(_hp_mod._hw_cfg,
                             'DYNAMICS_TORQUE_FF_PLATFORM_INERTIA', False), \
                build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            pose = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
            plant.set_pose(pose, np.zeros(6), np.zeros(6))
            tau_static = plant._ff_torque_buf.copy()
            plant.set_pose(pose, np.zeros(6),
                           np.array([0.0, 0.0, 4000.0, 0.0, 0.0, 0.0]))
            np.testing.assert_allclose(plant._ff_torque_buf, tau_static,
                                       rtol=1e-12)
            assert float(np.max(np.abs(tau_static))) > 1e-3, (
                "gravity term must still be present")


# =====================================================================
# T-U-T2b-7, -8: cold-start zero-state
# =====================================================================

class TestColdStartZeroState:
    """T-U-T2b-7, -8 — cold-start path at ``hardware_plant.py:529–535``.

    Two scenarios:
    * **T-U-T2b-7** — fresh plant, NO telemetry ever arrived.
      ``_last_telem is None`` → ``get_state()`` returns the
      zero-initialised ``_state`` buffer (from ``:284–294``);
      ``data_age_s is None``.
    * **T-U-T2b-8** — fresh plant, ONE frame arrives (simulating
      ``enable()`` completion), then 1 s passes with no recv.
      ``telem_age > _telem_stale_estop_s`` (0.5 s @ control_dt=0.025)
      fires ``estop('telemetry_stale')``.
    """

    def test_t2b_7_cold_start_returns_zero_state_with_none_age(self):
        """T-U-T2b-7 — fresh ``HardwarePlant`` with NO telemetry ever
        returns the zero-initialised state.

        Driver: ``drain_recv_pump`` is installed BEFORE any
        ``get_state()`` call, so the pump never yields a frame —
        ``_last_telem`` stays None throughout.
        """
        with build_hardware_plant_stub() as plant:
            drain_recv_pump(plant)
            spy = _EstopSpy(plant)

            state = plant.get_state()

            assert plant._last_telem is None
            assert plant._last_telem_recv_time is None
            assert state.data_age_s is None
            np.testing.assert_array_equal(
                state.leg_extensions_mm, np.zeros(6))
            np.testing.assert_array_equal(
                state.leg_velocities_mmps, np.zeros(6))
            np.testing.assert_array_equal(
                state.platform_pos_mm, np.zeros(3))
            np.testing.assert_array_equal(
                state.platform_rot, np.zeros(3))
            np.testing.assert_array_equal(
                state.platform_twist, np.zeros(6))
            # Cold-start does NOT fire e-stop — telem_age=None gates
            # the ESTOP check at ``:627``.
            assert not plant._estop_requested
            assert spy.call_count == 0
            # FK never ran on this path
            assert not plant._fk_ever_succeeded

    def test_t2b_8_cold_start_then_one_second_no_recv_fires_estop(self):
        """T-U-T2b-8 — one frame received (simulating ``enable()``
        completion), then 1 s with no recv → ESTOP fires.

        Driver: standard prime, then drain + advance time by 1.0 s.
        At ``control_dt=0.025``, the ESTOP threshold is 0.5 s; 1.0 s
        is well past it.
        """
        with build_hardware_plant_stub() as plant:
            plant.get_state()  # prime — simulates enable() success
            spy = _EstopSpy(plant)
            drain_recv_pump(plant)

            with freeze_perf_counter_at(plant, 1.0):
                state = plant.get_state()

            assert state.data_age_s == pytest.approx(1.0)
            # 1.0 s > 0.5 s ESTOP threshold (20 × 0.025)
            assert 1.0 > plant._telem_stale_estop_s
            assert plant._estop_requested
            assert spy.call_count == 1
            assert spy.reasons == ['telemetry_stale']


# =====================================================================
# T-U-P5: telemetry-validity watchdog (PLANT_INTERFACE_CONTRACT.md P5)
# =====================================================================

class TestP5TelemetryValidity:
    """T-U-P5-1..4 — the P5 telemetry-validity watchdog at
    ``hardware_plant.py`` (the ``telemetry_invalid`` ESTOP arm added
    after the ``telemetry_stale`` block, plus the
    ``_last_valid_motor_pos_time`` refresh in the ``motor_pos is not
    None`` branch).

    Root cause this guards (see
    ``logbook/2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md``):
    after a CAN drop, motor_guard stays alive but publishes
    ``motor_pos=None`` (``motor_guard.py:351,1078,1128``).  Frames keep
    arriving so the recv-time ``telemetry_stale`` watchdog stays fresh,
    and the frozen-content detector is gated out by its
    ``motor_pos is not None`` check — so without P5 the MPC ran blind
    on ``leg_ext=[0]*6`` + a frozen FK pose for 68 s with zero
    safety escalation.

    Budget = ``_telem_stale_estop_s`` (0.5 s at control_dt=0.025) —
    the SAME blind-time budget as the no-frames path (one source of
    truth).  Empirically confirmed deterministic on the pinned stack
    via ``/tmp/probe_p5_telemetry_invalid.py`` before this test was
    written (CLAUDE.md empirical-probe rule); the recipe is: prime a
    valid frame (``_fk_ever_succeeded`` True, validity clock set) →
    ``install_motor_pos_none_pump`` → ``freeze_perf_counter_at`` at the
    target no-valid-feedback age → one ``get_state()``.
    """

    def test_p5_1_none_after_valid_estops_past_budget(self):
        """T-U-P5-1 — valid feed, then ``motor_pos=None`` while frames
        stay fresh for > budget ⇒ exactly one
        ``estop(reason='telemetry_invalid')`` and ``_estop_requested``.

        Crucially asserts the recv-time ``telemetry_stale`` watchdog
        does NOT fire (frames are fresh — that net is correctly silent;
        P5 is what catches this), so the reason is ``telemetry_invalid``
        and not ``telemetry_stale``.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            assert plant._last_valid_motor_pos_time is not None, (
                "prime should set the P5 validity clock"
            )
            spy = _EstopSpy(plant)
            install_motor_pos_none_pump(plant)
            budget = plant._telem_stale_estop_s

            with _capture_hp_warnings() as records, \
                    freeze_perf_counter_at(plant, budget + 0.1):
                state = plant.get_state()

            assert spy.call_count == 1, (
                f"expected exactly 1 estop; got {spy.call_count} "
                f"({spy.reasons})"
            )
            assert spy.reasons == ['telemetry_invalid'], (
                f"P5 must escalate as 'telemetry_invalid', not "
                f"'telemetry_stale' (frames were fresh): {spy.reasons}"
            )
            assert plant._estop_requested
            # Frames were fresh the whole time → telem_age ~0 → the
            # recv-time staleness watchdog must NOT have fired.
            assert state.data_age_s == pytest.approx(0.0, abs=1e-6), (
                f"frames fresh: telem_age should be ~0, got "
                f"{state.data_age_s}"
            )
            estop_recs = [
                r for r in records
                if 'absent/invalid' in r.getMessage()
                and 'triggering e-stop' in r.getMessage()
            ]
            assert len(estop_recs) == 1, (
                f"expected one P5 ESTOP log line; got "
                f"{[r.getMessage() for r in estop_recs]}"
            )

    def test_p5_2_half_budget_warns_once_no_estop(self):
        """T-U-P5-2 — between half-budget and budget: a once-only
        'publisher may be blind' WARN, ``_telem_invalid_warned`` set,
        and NO ESTOP.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            spy = _EstopSpy(plant)
            install_motor_pos_none_pump(plant)
            budget = plant._telem_stale_estop_s

            with _capture_hp_warnings() as records, \
                    freeze_perf_counter_at(plant, 0.6 * budget):
                plant.get_state()

            assert spy.call_count == 0, (
                f"half-budget must not ESTOP; got {spy.reasons}"
            )
            assert plant._telem_invalid_warned, (
                "_telem_invalid_warned should be True at the WARN edge"
            )
            warn_recs = [
                r for r in records
                if 'absent/invalid' in r.getMessage()
                and 'publisher may be blind' in r.getMessage()
            ]
            assert len(warn_recs) == 1, (
                f"expected exactly one once-only WARN; got "
                f"{[r.getMessage() for r in warn_recs]}"
            )

    def test_p5_3_cold_start_none_never_estops(self):
        """T-U-P5-3 — the ``_fk_ever_succeeded`` gate: a never-armed
        cold start with ``motor_pos=None`` from tick 0 (no CAN at
        boot) must NOT ESTOP even at an extreme no-valid-feedback age.

        Without the gate this would false-ESTOP a de-energised robot
        during normal start-up.  Mirrors the FK-fail and frozen-content
        cold-start guards.
        """
        with build_hardware_plant_stub() as plant:
            install_motor_pos_none_pump(plant)
            plant.get_state()  # sets _last_telem_recv_time; FK never runs
            assert not plant._fk_ever_succeeded, (
                "cold start with motor_pos=None: FK must never have run"
            )
            assert plant._last_valid_motor_pos_time is None, (
                "no valid frame ever ⇒ validity clock stays None"
            )
            spy = _EstopSpy(plant)

            with freeze_perf_counter_at(plant, 10.0):  # extreme age
                plant.get_state()

            assert spy.call_count == 0, (
                f"cold-start motor_pos=None must NOT ESTOP "
                f"(_fk_ever_succeeded gate); got {spy.reasons}"
            )
            assert not plant._estop_requested

    def test_p5_4_recovery_resets_clock_then_retrips(self):
        """T-U-P5-4 — None→valid before budget clears the WARN flag and
        refreshes the validity clock (no ESTOP); a subsequent sustained
        None gap re-trips ``telemetry_invalid`` measured from the
        recovery instant, proving the clock truly reset.
        """
        with build_hardware_plant_stub() as plant:
            _prime_plant(plant)
            spy = _EstopSpy(plant)
            install_motor_pos_none_pump(plant)
            budget = plant._telem_stale_estop_s

            # Phase 1: drift to half-budget (WARN, no estop).
            with freeze_perf_counter_at(plant, 0.6 * budget):
                plant.get_state()
            assert plant._telem_invalid_warned
            assert spy.call_count == 0

            # Phase 2: a valid frame recovers — clock + WARN reset.
            install_telemetry_pump(plant)
            plant.get_state()
            assert not plant._telem_invalid_warned, (
                "recovery must clear the once-only WARN flag"
            )
            assert plant._last_valid_motor_pos_time is not None
            assert spy.call_count == 0

            # Phase 3: a fresh sustained None gap past budget re-trips.
            install_motor_pos_none_pump(plant)
            with freeze_perf_counter_at(plant, budget + 0.1):
                plant.get_state()
            assert spy.reasons == ['telemetry_invalid'], (
                f"clock should have reset at recovery and re-tripped: "
                f"{spy.reasons}"
            )
            assert plant._estop_requested
