"""Plan 2 Phase 7 (Tier 3a-2) — ``diag`` schema-completeness fuzz.

Pins the contract that every ``MPCController.solve()`` call returns a
``diag`` dict carrying the canonical schema keys, regardless of which
solve path executed (success / fallback walk-forward / hold_extrap /
hold / cold_hold / non_finite_solution / exception).

The schema is codified in
[controller/DIAG_SCHEMA_CONTRACT.md](../../controller/DIAG_SCHEMA_CONTRACT.md)
(landed by the Phase 7 bugfix commit — see logbook
[2026-05-12-tier3a-fuzz-bugfix.md](../../logbook/2026-05-12-tier3a-fuzz-bugfix.md)):
eight canonical keys uniformly present on every solve path
(``solve_time_ms, status, iter_count, cost, constraint_violation,
cmd_next_mm, cmd_next2_mm, fallback_step``), with ``iter_count=0``
and ``fallback_step=-1`` sentinels where the value isn't meaningful.

**Pre-fix history** (preserved here because it shaped the test
design).  Phase 7's empirical probe (2026-05-11,
``/tmp/probe_phase7_schema.py``, not committed) found two schema
asymmetries that prompted the contract document:

* ``iter_count`` was populated ONLY on ``Solve_Succeeded`` and
  missing from every failure path.
* ``fallback_step`` was populated ONLY on the walk-forward fallback
  path and missing from success, hold_extrap, hold, cold_hold.

Consumers (``runner.log_mpc_step`` at ``controller/runner.py:207``)
defended with ``diag.get('iter_count', 0)`` so the missing keys
produced silent default-zero values rather than a ``KeyError`` — but
the data lost on failure ticks (iteration counts during fallback /
fallback_step counter on non-walk-forward paths) was real diagnostic
signal.

This test commit (`2105bb4`) pinned the gap as failing assertions
under ``xfail(strict=True)``; the companion bugfix commit lifted
the markers atomically by flipping ``_PHASE_7_BUGFIX_LANDED = True``
in [tests/sim/test_mpc_input_fuzz.py](test_mpc_input_fuzz.py) AND
landing the producer-side enforcement (``__init__`` declares 8
keys; success path populates ``fallback_step=-1``;
``_handle_failure`` populates ``iter_count=0`` + ``fallback_step=-1``).

Test cases (per the plan; S8 dropped — the "extras namespace" was
not a real surface; see logbook Discussion):

| ID         | Surface                                          | Strategy                                                |
|------------|--------------------------------------------------|---------------------------------------------------------|
| T-U-T3a-S1 | Solve_Succeeded                                  | Primed MPC + feasible target; assert all 8 keys present |
| T-U-T3a-S2 | fallback walk-forward                            | NaN injection (Phase 3 mechanism); assert schema        |
| T-U-T3a-S3 | hold_extrap                                      | Repeated NaN to escalate; assert schema (XFAIL pre-fix) |
| T-U-T3a-S4 | cold_hold                                        | Fresh MPC + NaN on first solve; assert schema (XFAIL)   |
| T-U-T3a-S5 | non_finite_solution                              | Monkey-patch ``_solver`` to return NaN in sol['x']      |
| T-U-T3a-S6 | exception path                                   | Monkey-patch ``_solver`` to raise; status='exception:'  |
| T-U-T3a-S7 | Property: every solve() leaves diag schema-complete | hypothesis stateful (XFAIL pre-fix)                 |

See logbook 2026-05-11-tier3a-numerical-schema-fuzz.md (test commit)
and 2026-05-12-tier3a-fuzz-bugfix.md (companion bugfix commit
landing the contract document).
"""
from __future__ import annotations

import logging

import numpy as np
import pytest
from hypothesis import HealthCheck, given, settings, strategies as st
from hypothesis.stateful import RuleBasedStateMachine, invariant, rule

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.plant import PlantState
from controller.target import flat_target_to_events

# Re-use the _PHASE_7_BUGFIX_LANDED flag from the numerical fuzz module so
# the two test files lift xfails together via a single source of truth.
from tests.sim.test_mpc_input_fuzz import _PHASE_7_BUGFIX_LANDED


# Suppress MPC log noise (these tests intentionally drive failures).
logging.getLogger('controller.mpc').setLevel(logging.CRITICAL)


CONTROL_DT = 0.025
TARGET = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])


# Documented canonical keys (always present, every path).
# See controller/DIAG_SCHEMA_CONTRACT.md D1–D8.  Post-Phase-7-bugfix
# the schema is uniformly 8 keys (with sentinels where the value
# isn't meaningful — `iter_count=0` on failure, `fallback_step=-1`
# on non-walk-forward paths).
_CANONICAL_KEYS = frozenset({
    'solve_time_ms',
    'status',
    'iter_count',
    'cost',
    'constraint_violation',
    'cmd_next_mm',
    'cmd_next2_mm',
    'fallback_step',
})

# Convenience reference for the conditional-value key.  Walk-forward
# fallback populates this with the per-step counter (>=0); every other
# path populates it with the sentinel value -1.
_FALLBACK_STEP_KEY = 'fallback_step'


# =====================================================================
# Fixtures + helpers
# =====================================================================

@pytest.fixture(scope='module')
def plant():
    """Module-scoped MuJoCoPlant — IK precompute is expensive."""
    p = MuJoCoPlant(control_dt=CONTROL_DT)
    p.reset()
    p.step(CONTROL_DT)
    yield p


def _state_at(plant, time_s=None):
    """Get a clean PlantState from a primed plant."""
    s = plant.get_state()
    if time_s is not None:
        s.time = time_s
    return s


def _state_with_nan_pos(state):
    """Return a copy of state with NaN injected into platform_pos_mm[0]."""
    return PlantState(
        leg_extensions_mm=state.leg_extensions_mm.copy(),
        leg_velocities_mmps=state.leg_velocities_mmps.copy(),
        platform_pos_mm=np.array([np.nan, 0.0, 50.0]),
        platform_rot=state.platform_rot.copy(),
        platform_twist=state.platform_twist.copy(),
        time=state.time,
    )


def _build_ref(state):
    """Build a feasible 2-event ref centred on the plant pose."""
    cur = np.concatenate([state.platform_pos_mm, state.platform_rot])
    return flat_target_to_events(
        cur, state.platform_twist, TARGET, t_now=state.time,
        v_max_mmps=140.0, tau_s=0.04,
    )


def _solve(mpc, state, target=TARGET, ref_events=None, warm_start_valid=True):
    """Single-line solve wrapper using the canonical kwargs."""
    if ref_events is None:
        ref_events = _build_ref(state)
    return mpc.solve(
        state, target_pose=target, ref_events=ref_events,
        t_now=state.time, warm_start_valid=warm_start_valid,
    )


def _build_primed_mpc(plant):
    """Construct a primed MPC instance ready to produce Solve_Succeeded.

    Uses a generous ``max_cpu_time=0.2`` (200 ms) because the schema
    tests are testing schema-completeness, not solve-time performance.
    Under concurrent CPU load (parallel test runs, hypothesis ci-deep)
    the default 18 ms budget makes solves hit
    ``Maximum_CpuTime_Exceeded`` and the prime sequence fails to
    reach ``Solve_Succeeded`` — irrelevant to the schema invariant
    under test.
    """
    params = MPCParams(prime_solver=True, max_leg_vel_mmps=140.0,
                       max_cpu_time=0.2)
    return MPCController.from_plant(params, plant)


def _build_unprimed_mpc(plant):
    """Construct an MPC without priming (for cold_hold path).

    Same generous CPU budget as the primed variant — see
    ``_build_primed_mpc`` docstring for the load-sensitivity rationale.
    """
    return MPCController.from_plant(
        MPCParams(prime_solver=False, max_cpu_time=0.2), plant)


def _assert_canonical_keys(diag, path_name):
    """Assert every canonical schema key is present.

    The pre-fix code is missing ``iter_count`` on all failure paths.
    Tests that depend on this contract are xfail-strict pre-fix.
    """
    missing = _CANONICAL_KEYS - set(diag.keys())
    assert not missing, (
        f'{path_name}: diag is missing canonical key(s): {sorted(missing)}. '
        f'diag.keys() = {sorted(diag.keys())}. '
        f'Schema-completeness contract violated — see '
        f'controller/DIAG_SCHEMA_CONTRACT.md.'
    )


# =====================================================================
# T-U-T3a-S1 — Solve_Succeeded path
# =====================================================================

class TestT3aS1SolveSucceededSchema:
    """T-U-T3a-S1 — every key in the canonical schema is populated
    on the success path.

    The success path (``mpc.py:1214-1228``) explicitly assigns all 8
    canonical keys.  This test pins that contract — if any one is
    dropped in a refactor, the test fails.
    """

    def test_t3a_s1_solve_succeeded_has_all_canonical_keys(self, plant):
        mpc = _build_primed_mpc(plant)
        state = _state_at(plant)
        # Drive a few solves to ensure we're firmly on the success
        # path (the first post-prime solve may converge in 1 iter but
        # we want robust steady-state success).
        for _ in range(3):
            cmd, cv, diag = _solve(mpc, state)
        assert diag['status'] == 'Solve_Succeeded', (
            f'expected Solve_Succeeded, got {diag["status"]!r}'
        )
        _assert_canonical_keys(diag, 'Solve_Succeeded')
        # Each canonical value must be non-None (success path populates
        # real values, not sentinels).
        for k in _CANONICAL_KEYS:
            assert diag[k] is not None, (
                f'Solve_Succeeded: diag[{k!r}] is None; success path '
                f'should populate real values, not sentinels'
            )


# =====================================================================
# T-U-T3a-S2 — fallback walk-forward path
# =====================================================================

class TestT3aS2FallbackExtrapSchema:
    """T-U-T3a-S2 — fallback (cmd-stream extrapolation) populates the
    canonical schema with ``fallback_step = -1`` sentinel.

    Drive: NaN in ``state.platform_pos_mm`` → IPOPT
    ``Invalid_Number_Detected`` → ``_handle_failure`` → linear
    cmd-stream extrapolation arm (Tier-1 rewrite 2026-05-20; replaced
    the prior walk-forward arm) → diag['status'] starts with
    ``fallback_extrap(``.  fallback_step is now the ``-1`` sentinel
    on every path (walk-forward arm removed; see
    DIAG_SCHEMA_CONTRACT.md "2026-05-20" History entry).
    """

    @pytest.mark.xfail(
        condition=not _PHASE_7_BUGFIX_LANDED,
        strict=True,
        reason=(
            'Pre-fix schema gap: _handle_failure does not populate '
            'iter_count. Bugfix commit adds iter_count=0 to failure '
            'diag dict.'
        ),
    )
    def test_t3a_s2_fallback_extrap_has_canonical_keys(
        self, plant,
    ):
        mpc = _build_primed_mpc(plant)
        state = _state_at(plant)
        # Build the reference ONCE from the clean state — passing the
        # NaN-state to flat_target_to_events propagates NaN through the
        # feasibility math (np.linalg.eigvals) and raises LinAlgError
        # BEFORE solve() is reached.  We want solve() to see the NaN,
        # not the ref builder.
        ref = _build_ref(state)
        # Prime with two successful solves so _prev_u AND _prev_prev_u
        # are populated (required for the fallback_extrap arm; with
        # only one success, the bootstrap fallback_hold arm fires
        # instead — also schema-compliant, but a different path).
        for _ in range(2):
            cmd, cv, diag = _solve(mpc, state, ref_events=ref)
        assert diag['status'] == 'Solve_Succeeded', (
            f'prime failed: {diag["status"]!r}'
        )
        # Now drive a NaN — should land on the linear cmd-stream
        # extrapolation arm (Tier-1 rewrite; replaces walk-forward).
        state_nan = _state_with_nan_pos(state)
        cmd, cv, diag = _solve(mpc, state_nan, ref_events=ref,
                               warm_start_valid=True)

        # Accept any fallback-class status: fallback_extrap (primary
        # path), fallback_hold (bootstrap edge), fallback(...) (legacy
        # status string used by the non_finite_solution and exception
        # call sites that pass status_str through unmodified), or
        # cold_hold (cold-start path).  The schema check is the
        # invariant we care about.
        assert any(diag['status'].startswith(p)
                   for p in ('fallback_extrap(', 'fallback_hold(',
                             'fallback(', 'cold_hold(')), (
            f'expected fallback-class status, got {diag["status"]!r}'
        )
        _assert_canonical_keys(diag, diag['status'])
        assert _FALLBACK_STEP_KEY in diag, (
            f'every fallback path MUST include fallback_step; '
            f'keys: {sorted(diag.keys())}'
        )
        # Post-Tier-1 rewrite (2026-05-20): walk-forward arm removed
        # → fallback_step is always the -1 sentinel.
        assert diag[_FALLBACK_STEP_KEY] == -1, (
            f'post-Tier-1 rewrite, fallback_step is always -1 '
            f'(no walk-forward arm exists); got '
            f'{diag[_FALLBACK_STEP_KEY]!r}'
        )


# =====================================================================
# T-U-T3a-S3 — hold_extrap (escalation) path
# =====================================================================

class TestT3aS3EscalationSchema:
    """T-U-T3a-S3 — sustained-cascade escalation populates the
    canonical schema.

    Post-Tier-1-rewrite (2026-05-20): escalation is purely time-based
    (500 ms since last success → cold_hold(q_cur)).  Pre-rewrite this
    was a heterogeneous ladder (walk-forward → hold_extrap →
    cold_hold) gated on the failure counter; both the counter-based
    escalation and the hold_extrap arm were removed.  This test
    drives a sustained cascade and asserts the resulting escalation
    status carries the canonical 8-key schema — accepting any
    fallback-class family the design happens to land on.

    Drive: prime MPC; NaN-spam until escalation fires.  The exact
    landing arm depends on whether 500 ms wall-clock elapses during
    the NaN-spam loop (it usually does on CI given solve-time
    variability and the 12-tick spam below).
    """

    @pytest.mark.xfail(
        condition=not _PHASE_7_BUGFIX_LANDED,
        strict=True,
        reason=(
            'Pre-fix schema gap: _handle_failure populates iter_count '
            'on no failure path.  Bugfix commit unifies the schema.'
        ),
    )
    def test_t3a_s3_escalation_has_canonical_keys(self, plant):
        mpc = _build_primed_mpc(plant)
        state = _state_at(plant)
        ref = _build_ref(state)  # see T-U-T3a-S2 for ref-from-clean rationale
        # Prime
        for _ in range(2):
            _solve(mpc, state, ref_events=ref)

        # NaN-spam to drive a sustained cascade.  Under the Tier-1
        # rewrite the loop emits fallback_extrap until the 500 ms
        # time bound trips, then cold_hold.  Either family is
        # schema-compliant.
        state_nan = _state_with_nan_pos(state)
        diag = None
        for _ in range(12):
            _, _, diag = _solve(mpc, state_nan, ref_events=ref,
                                warm_start_valid=True)

        # Accept any fallback-class status family from the post-rewrite
        # design.  The schema check is the invariant we care about.
        assert any(diag['status'].startswith(p)
                   for p in ('fallback_extrap(', 'fallback_hold(',
                             'fallback(', 'cold_hold(')), (
            f'expected a fallback-class status; got {diag["status"]!r}'
        )
        _assert_canonical_keys(diag, diag['status'])


# =====================================================================
# T-U-T3a-S4 — cold_hold path
# =====================================================================

class TestT3aS4ColdHoldSchema:
    """T-U-T3a-S4 — cold_hold fires when _prev_u is None (no prior
    success) AND the solver fails.  Schema must be complete here too.

    Drive: fresh MPC (no priming); NaN on first solve → cold_hold.
    """

    @pytest.mark.xfail(
        condition=not _PHASE_7_BUGFIX_LANDED,
        strict=True,
        reason=(
            'Pre-fix schema gap: _handle_failure missing iter_count '
            'on cold_hold path.  Bugfix commit unifies the schema.'
        ),
    )
    def test_t3a_s4_cold_hold_has_canonical_keys(self, plant):
        mpc = _build_unprimed_mpc(plant)
        state = _state_at(plant)
        ref = _build_ref(state)  # see T-U-T3a-S2 for ref-from-clean rationale
        state_nan = _state_with_nan_pos(state)
        # First solve, no prior success → cold_hold arm fires
        _, _, diag = _solve(mpc, state_nan, ref_events=ref,
                            warm_start_valid=False)
        assert diag['status'].startswith(('cold_hold(', 'hold_extrap(',
                                          'hold(', 'fallback(')), (
            f'expected a non-success status; got {diag["status"]!r}'
        )
        _assert_canonical_keys(diag, 'cold_hold')


# =====================================================================
# T-U-T3a-S5 — non_finite_solution path (solver returns NaN sol['x'])
# =====================================================================

class TestT3aS5NonFiniteSolutionSchema:
    """T-U-T3a-S5 — when the solver returns a solution containing
    NaN/Inf (``sol['x']``), the production guard at
    ``mpc.py:1119-1123`` routes through ``_handle_failure`` with
    status ``'non_finite_solution'``.

    The path is structurally hard to drive via plant inputs: IPOPT
    usually catches NaN internally and reports
    ``Invalid_Number_Detected`` before the post-solve guard sees it.
    To drive THIS specific path, monkey-patch the solver at its
    boundary — replace ``mpc._solver`` with a callable that returns
    a CasADi-shaped result dict containing NaN in ``'x'``.
    Mirrors Phase 1's T-U-T1a-3 pattern (solver-boundary monkey-patch
    for documented exit codes not drivable via MPCParams).

    Asserts BOTH that the routing fires (status contains
    'non_finite_solution') AND that the resulting diag carries the
    canonical schema (post-bugfix; xfail-strict pre-fix).
    """

    def test_t3a_s5_non_finite_solver_output_routes_to_handle_failure(
        self, plant,
    ):
        # Use an UNPRIMED MPC: priming + settle solves would store the
        # successful solution's ``_prev_w`` (which IS
        # ``_prev_w_buf`` by reference).  The next ``np.copyto`` into
        # ``_prev_w_buf`` mutates ``_prev_w`` in place, so a NaN-poked
        # solve would corrupt ``_prev_w`` BEFORE the isfinite-check
        # fires, leaving ``_handle_failure``'s walk-forward arm
        # reading NaN.  An unprimed MPC has ``_prev_w is None`` so the
        # cold_hold arm fires instead — which uses ``q_cur`` (clean
        # plant state) for the command.  Schema-completeness on the
        # non_finite_solution + cold_hold combination is what this
        # test exercises.
        mpc = _build_unprimed_mpc(plant)
        state = _state_at(plant)
        ref_events = _build_ref(state)

        # Capture the real solver, replace with a callable OBJECT that
        # exposes both __call__ (mirrors the CasADi nlpsol function call
        # surface) AND .stats() (which the production code at
        # ``mpc.py`` queries for ``return_status`` + ``iter_count``
        # after the solve).  Plain Python functions can't carry
        # arbitrary attributes; a class-based wrapper does.
        import casadi as _ca
        real_solver = mpc._solver

        class _NaNSolver:
            """Calls the real solver, then NaNs out sol['x'].

            Reports stats() = Solve_Succeeded so the production code's
            success-path post-solve isfinite() check at
            ``controller/mpc.py:1119`` is the surface under test.
            """
            def __call__(self, *args, **kwargs):
                sol = real_solver(*args, **kwargs)
                n_x = sol['x'].numel()
                sol['x'] = _ca.DM(np.full(n_x, np.nan))
                return sol

            def stats(self):
                return {'return_status': 'Solve_Succeeded',
                        'iter_count': 1}

        mpc._solver = _NaNSolver()
        try:
            cmd, cv, diag = _solve(mpc, state, ref_events=ref_events)
        finally:
            mpc._solver = real_solver

        # status should reflect the non_finite_solution routing
        assert 'non_finite_solution' in diag['status'], (
            f'expected non_finite_solution status; got {diag["status"]!r}'
        )
        # cmd MUST be finite (the handle_failure path's job)
        assert np.all(np.isfinite(cmd)), (
            f'cmd contains non-finite values after non_finite_solution '
            f'routing: {cmd!r}'
        )
        # Schema-completeness on this path: xfail-gated like S2-S4
        # (the non_finite_solution path routes through _handle_failure
        # whose diag dict pre-fix is missing iter_count).
        if _PHASE_7_BUGFIX_LANDED:
            _assert_canonical_keys(diag, 'non_finite_solution')
        else:
            missing = _CANONICAL_KEYS - set(diag.keys())
            assert missing == {'iter_count'}, (
                f'non_finite_solution path: expected only iter_count '
                f'missing pre-fix; got missing={sorted(missing)}.  '
                f'If iter_count is no longer the only missing key, '
                f'the schema contract has drifted.'
            )


# =====================================================================
# T-U-T3a-S6 — exception path
# =====================================================================

class TestT3aS6ExceptionSchema:
    """T-U-T3a-S6 — when the in-solve try block raises an arbitrary
    exception (CasADi internal error, memory error, etc.), the
    production code at ``mpc.py:1223-1238`` clears the warm-start
    and routes through ``_handle_failure`` with status
    ``'exception: <repr>'``.

    Drive: monkey-patch ``mpc._solver`` to raise.  This is the
    boundary that all in-solve exception paths flow through.

    Asserts BOTH the routing (status contains 'exception',
    _prev_w cleared, cmd finite) AND the canonical schema (post-fix;
    pre-fix the path is missing iter_count via _handle_failure).
    """

    def test_t3a_s6_solver_exception_routes_to_handle_failure(self, plant):
        mpc = _build_primed_mpc(plant)
        state = _state_at(plant)
        for _ in range(2):
            _solve(mpc, state)

        # Replace the solver with one that raises.  The production
        # handler at :1223-1238 clears _prev_w / _prev_lam_g /
        # _prev_lam_x and calls _handle_failure with the exception
        # repr in the status string.
        real_solver = mpc._solver
        class _FakeSolverError(RuntimeError):
            pass

        def raising_solver(*args, **kwargs):
            raise _FakeSolverError('synthetic solver failure')

        mpc._solver = raising_solver
        try:
            cmd, cv, diag = _solve(mpc, state)
        finally:
            mpc._solver = real_solver

        # The status should start with 'exception:' (or 'fallback(exception:'
        # etc. depending on which failure-handler arm fires).
        assert 'exception' in diag['status'].lower(), (
            f'expected exception in status; got {diag["status"]!r}'
        )
        # cmd MUST be finite
        assert np.all(np.isfinite(cmd))
        # Warm start MUST be cleared on the exception path
        # (see :1229-1234).
        assert mpc._prev_w is None, (
            f'_prev_w should be cleared after solver exception; '
            f'got {mpc._prev_w!r}'
        )
        # Schema-completeness on this path: same xfail-style gate as S5.
        if _PHASE_7_BUGFIX_LANDED:
            _assert_canonical_keys(diag, 'exception')
        else:
            missing = _CANONICAL_KEYS - set(diag.keys())
            assert missing == {'iter_count'}, (
                f'exception path: expected only iter_count missing '
                f'pre-fix; got missing={sorted(missing)}.'
            )


# =====================================================================
# T-U-T3a-S7 — property: every solve() leaves diag schema-complete
# =====================================================================

# Hypothesis settings for the stateful machine.  Per-example cost is
# dominated by the real CasADi solve (~5–15 ms steady state).
_PHASE_7_SETTINGS = settings(
    suppress_health_check=(
        HealthCheck.too_slow,
        HealthCheck.data_too_large,
        HealthCheck.filter_too_much,
    ),
)


def _build_machine_singletons():
    """Build a module-scoped MPC + plant pair for the RuleBasedStateMachine.

    Per-example construction would cost ~1 s (IK precompute + solver
    prime); a singleton amortises the cost across all examples.  Same
    pattern as Phase 3's T1cWarmStartIntegrityMachine.
    """
    plant_ = MuJoCoPlant(control_dt=CONTROL_DT)
    plant_.reset()
    plant_.step(CONTROL_DT)
    state_ = plant_.get_state()
    params_ = MPCParams(prime_solver=True, max_leg_vel_mmps=140.0,
                        max_cpu_time=0.2)
    mpc_ = MPCController.from_plant(params_, plant_)
    # Settle into success
    ref_ = flat_target_to_events(
        np.concatenate([state_.platform_pos_mm, state_.platform_rot]),
        state_.platform_twist, TARGET, t_now=state_.time,
        v_max_mmps=140.0, tau_s=0.04,
    )
    for _ in range(3):
        mpc_.solve(state_, target_pose=TARGET, ref_events=ref_,
                   t_now=state_.time, warm_start_valid=True)
    return plant_, mpc_, ref_


_T3aS7_SINGLETONS = None


def _t3a_s7_get_singletons():
    global _T3aS7_SINGLETONS
    if _T3aS7_SINGLETONS is None:
        _T3aS7_SINGLETONS = _build_machine_singletons()
    return _T3aS7_SINGLETONS


class T3aS7DiagSchemaInvariantMachine(RuleBasedStateMachine):
    """T-U-T3a-S7 — schema-completeness invariant across random
    sequences of clean / adversarial solves.

    Rules:
      * ``clean_solve`` — solve with the primed reference; produces
        Solve_Succeeded most of the time.
      * ``nan_solve`` — solve with a NaN-injected state; produces
        a fallback class status.

    Invariant (checked after every solve):
      * ``diag.keys()`` is a superset of ``_CANONICAL_KEYS``.

    Pre-fix the invariant fails on every failure-path solve
    (iter_count is missing).  Post-fix it holds across every sampled
    path.
    """

    def __init__(self):
        super().__init__()
        self._plant, self._mpc, self._ref = _t3a_s7_get_singletons()
        self._clean_state = self._plant.get_state()
        self._nan_state = _state_with_nan_pos(self._clean_state)

    @rule()
    def clean_solve(self):
        """Drive the success path."""
        _, _, diag = self._mpc.solve(
            self._clean_state, target_pose=TARGET, ref_events=self._ref,
            t_now=self._clean_state.time, warm_start_valid=True,
        )
        # Stash for the invariant.
        self._last_diag = diag

    @rule()
    def nan_solve(self):
        """Drive a fallback class path."""
        _, _, diag = self._mpc.solve(
            self._nan_state, target_pose=TARGET, ref_events=self._ref,
            t_now=self._nan_state.time, warm_start_valid=True,
        )
        self._last_diag = diag

    @invariant()
    def schema_complete(self):
        """Every solve() return leaves diag containing the canonical schema."""
        if not hasattr(self, '_last_diag'):
            return
        missing = _CANONICAL_KEYS - set(self._last_diag.keys())
        assert not missing, (
            f'Schema invariant violated: missing keys '
            f'{sorted(missing)} on status={self._last_diag.get("status")!r}. '
            f'See controller/DIAG_SCHEMA_CONTRACT.md.'
        )


# Wrap the state-machine in the pytest entry point AND in the xfail
# gate so the test is skipped pre-fix.  TestCase-style attribute
# avoids one-instance-per-example construction (which would invalidate
# the singleton).
T3aS7DiagSchemaInvariantMachine.TestCase.settings = _PHASE_7_SETTINGS


class TestT3aS7DiagSchemaInvariantProperty:
    """Wrap T3aS7DiagSchemaInvariantMachine in a pytest class so the
    xfail marker can be applied class-wide (the machine itself is a
    hypothesis construct).
    """

    @pytest.mark.xfail(
        condition=not _PHASE_7_BUGFIX_LANDED,
        strict=True,
        reason=(
            'Pre-fix schema gap: iter_count missing on every failure '
            'path.  The stateful machine WILL drive a failure path '
            '(nan_solve rule) and the invariant WILL fail.'
        ),
    )
    def test_t3a_s7_schema_invariant_across_random_sequences(self):
        """Run the stateful machine via the standard hypothesis driver."""
        from hypothesis.stateful import run_state_machine_as_test
        run_state_machine_as_test(T3aS7DiagSchemaInvariantMachine)
