"""Plan 2 Phase 1 (Tier 1a) — real IPOPT exit codes + fallback-keyword matrix.

This module exercises the ``MPCController`` failure-handling machinery
against the **actual exit codes** IPOPT emits, plus the canonical-string
classifier at the success-vs-fallback boundary
(``mpc.py`` ``solve()`` ``ret in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')``).

The pre-Plan-2 suite drives fallback only via a synthetic "infeasible
reference" pattern (see [test_mpc_static.py::TestFallback](test_mpc_static.py)),
which routes through ``cold_hold(Maximum_Iterations_Exceeded)`` — a
specific code path that doesn't exercise the keyword classifier on the
full set of strings IPOPT may emit, or on the strings the classifier
*depends on* but our problem rarely produces (``Solved_To_Acceptable_Level``).

Test IDs from
[plans/active/mpc-sadpath-coverage-tiers-1-3.md](../../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
Phase 1:

| ID         | Real driver                                          | Exit code                       |
|------------|------------------------------------------------------|---------------------------------|
| T-U-T1a-1  | ``max_cpu_time=1e-3``                                | ``Maximum_CpuTime_Exceeded``    |
| T-U-T1a-2  | pin ``q``-decision bounds (singleton infeasible)     | ``Infeasible_Problem_Detected`` |
| T-U-T1a-3  | ``max_iter=1``                                       | ``Maximum_Iterations_Exceeded`` |
| T-U-T1a-4  | (xfail) — not drivable via ``MPCParams`` in 3.7.2    | ``Restoration_Failed``          |
| T-U-T1a-5  | monkey-patch ``_solver`` to raise                    | ``exception: …``                |
| T-U-T1a-6  | parametrize all status strings; stats() injection    | (all)                           |

Pinned CasADi: **3.7.2**.  IPOPT's status-string set is stable across
patch releases; a CasADi major-version upgrade should re-run this matrix
to confirm no string was renamed.

See [logbook/2026-05-11-tier1a-real-solver-failures.md](../../logbook/2026-05-11-tier1a-real-solver-failures.md).
"""

from __future__ import annotations

import numpy as np
import pytest

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams


CONTROL_DT = 0.025  # 40 Hz, matches the rest of the suite

REF_NORMAL = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])


# ---------------------------------------------------------------------
# Fixtures + helpers
# ---------------------------------------------------------------------

@pytest.fixture(scope='module')
def plant():
    """Shared MuJoCo plant for all tests in this module."""
    return MuJoCoPlant()


def _create_mpc(plant, **overrides):
    """Build an MPC for failure-path tests.

    ``use_aot_solver=False`` is mandatory here — the AOT-compiled solver
    has IPOPT options baked into its ``.so`` and was generated against
    the production ``max_cpu_time`` / ``max_iter`` defaults.  Failure-
    path tests need the runtime ``opts`` dict to be honoured at solver
    construction time, which only happens when ``use_aot_solver=False``
    forces the in-process ``cs.nlpsol`` build at
    [mpc.py:582–583](../../controller/mpc.py).

    ``prime_solver=False`` skips the from_plant warm-up; tests build
    the warm-start they need explicitly so the failure scenario starts
    from a known state.
    """
    defaults = dict(
        max_cpu_time=2.0, max_iter=500, max_leg_vel_mmps=280.0,
        prime_solver=False, use_aot_solver=False,
    )
    defaults.update(overrides)
    return MPCController.from_plant(MPCParams(**defaults), plant)


def _seed_warm_start(plant, mpc, ref=REF_NORMAL):
    """Run one successful solve so the failing-MPC fixtures have a
    warm-start to fall back on.  Returns the seeding diagnostics."""
    plant.reset()
    state = plant.get_state()
    _, _, diag = mpc.solve(state, ref)
    assert diag['status'] == 'Solve_Succeeded', (
        f"seed solve must succeed; got {diag['status']!r}"
    )
    return diag


def _pin_q_decision_bounds_infeasible(mpc, value=1000.0):
    """Pin the ``q[k]`` (leg-extension) decision variables to a single
    out-of-workspace value (~1000 mm vs the ~275 mm stroke).  IPOPT
    enters restoration, can't reconcile against the dynamics
    constraints, and returns ``Infeasible_Problem_Detected`` cleanly.

    Layout reminder (from [mpc.py:646–651](../../controller/mpc.py)):
    ``W = [u[0..N-1] | q[1..N] | p[1..N]]``; ``q`` occupies indices
    ``6N..12N``.

    Mutates ``mpc._lbw`` / ``mpc._ubw`` in place.  Callers MUST stash
    and restore the originals if the MPC will be reused.
    """
    N = mpc._params.N
    mpc._lbw[6 * N: 12 * N] = value
    mpc._ubw[6 * N: 12 * N] = value + 1e-9


class _RaisingSolver:
    """Drop-in replacement for ``mpc._solver`` that raises on every call.

    Used by T-U-T1a-5 to drive the exception handler at
    [mpc.py:1223–1238](../../controller/mpc.py) directly.  CasADi
    ``Function`` objects are SWIG-wrapped and don't allow attribute
    assignment, so wrapping is the only way to inject this behaviour.
    """

    def __init__(self, message: str = "injected solver crash"):
        self._message = message

    def __call__(self, **kw):
        raise RuntimeError(self._message)

    def stats(self):
        # solve()'s exception handler never reads stats(), but provide
        # a benign fallback in case future refactors add a stats() read
        # in the exception arm.
        return {}


class _StatsInjector:
    """Wrap a CasADi solver to inject a custom ``return_status`` string.

    The real solver runs (so ``sol['x']`` / ``sol['lam_g']`` etc. are
    valid), and ``stats()`` returns the caller-supplied status.  Used
    by T-U-T1a-6 to drive the classifier at
    [mpc.py:1082](../../controller/mpc.py) over every documented IPOPT
    status string, regardless of whether IPOPT actually emits each
    string on our problem.
    """

    def __init__(self, inner, return_status: str, iter_count: int = 5):
        self._inner = inner
        self._return_status = return_status
        self._iter_count = iter_count

    def __call__(self, **kw):
        return self._inner(**kw)

    def stats(self):
        return {
            'return_status': self._return_status,
            'iter_count': self._iter_count,
        }


# ---------------------------------------------------------------------
# T-U-T1a-1, -2, -3, -5: real IPOPT exit codes
# ---------------------------------------------------------------------

class TestRealIpoptExitCodes:
    """Drive the four IPOPT exit codes our pinned CasADi 3.7.2 cleanly
    produces, plus the CasADi-exception path.  ``Restoration_Failed``
    has its own xfail class below."""

    def test_max_cpu_time_exceeded(self, plant):
        """T-U-T1a-1: ``max_cpu_time=1e-3`` produces
        ``Maximum_CpuTime_Exceeded`` reliably on the Jetson + CasADi
        3.7.2.  Working Note #1 in the plan flagged a concern that
        very-small ``max_cpu_time`` might cause an internal init error
        instead — empirically (see logbook), this does not occur on
        our pinned stack."""
        plant.reset()
        mpc = _create_mpc(plant, max_cpu_time=1e-3)
        state = plant.get_state()
        cmd, _vel, diag = mpc.solve(state, REF_NORMAL)
        assert 'Maximum_CpuTime_Exceeded' in diag['status'], (
            f"expected Maximum_CpuTime_Exceeded in status, "
            f"got {diag['status']!r}"
        )
        # Fallback latched; counter incremented.
        assert mpc.consecutive_failures == 1
        # Cmd well-formed (cold_hold path emits margin-clamped pose).
        assert np.all(np.isfinite(cmd))

    def test_max_iterations_exceeded(self, plant):
        """T-U-T1a-3: ``max_iter=1`` produces ``Maximum_Iterations_Exceeded``."""
        plant.reset()
        mpc = _create_mpc(plant, max_iter=1)
        state = plant.get_state()
        cmd, _vel, diag = mpc.solve(state, REF_NORMAL)
        assert 'Maximum_Iterations_Exceeded' in diag['status'], (
            f"expected Maximum_Iterations_Exceeded in status, "
            f"got {diag['status']!r}"
        )
        assert mpc.consecutive_failures == 1
        assert np.all(np.isfinite(cmd))

    def test_infeasible_problem_detected(self, plant):
        """T-U-T1a-2: pin ``q[k]`` decision bounds to a singleton far
        outside the workspace.  IPOPT enters restoration, fails to find
        a feasible point against the dynamics + IK constraints, and
        returns ``Infeasible_Problem_Detected``.

        Bounds are constructed to pass CasADi's lb≤ub pre-validation
        (the simple "ubg<lbg" trick would assert at problem
        construction, routing through the exception handler instead).
        """
        plant.reset()
        mpc = _create_mpc(plant)
        _pin_q_decision_bounds_infeasible(mpc, value=1000.0)
        state = plant.get_state()
        cmd, _vel, diag = mpc.solve(state, REF_NORMAL)
        assert 'Infeasible_Problem_Detected' in diag['status'], (
            f"expected Infeasible_Problem_Detected in status, "
            f"got {diag['status']!r}"
        )
        assert mpc.consecutive_failures == 1
        assert np.all(np.isfinite(cmd))

    def test_casadi_exception_routes_through_handler(self, plant):
        """T-U-T1a-5: a ``RuntimeError`` thrown from inside the solver
        call routes through the exception handler at
        [mpc.py:1223–1238](../../controller/mpc.py).  The handler must
        (a) tag the status as ``exception: …``, (b) clear the
        warm-start state to prevent corruption cascade, and (c) still
        produce a finite ``cmd`` via ``_handle_failure``.
        """
        plant.reset()
        mpc = _create_mpc(plant)
        # Seed a valid warm-start so we can verify it gets cleared.
        _seed_warm_start(plant, mpc)
        assert mpc._prev_w is not None
        assert mpc._prev_lam_g is not None
        assert mpc._prev_lam_x is not None

        mpc._solver = _RaisingSolver("injected solver crash for T-U-T1a-5")
        plant.reset()
        state = plant.get_state()
        cmd, _vel, diag = mpc.solve(state, REF_NORMAL)

        assert diag['status'].startswith('hold') or diag['status'].startswith('cold_hold'), (
            f"exception path should route through hold/cold_hold fallback, "
            f"got {diag['status']!r}"
        )
        assert 'exception' in diag['status'], (
            f"exception status string missing 'exception' prefix marker, "
            f"got {diag['status']!r}"
        )
        assert 'injected solver crash' in diag['status'], (
            f"exception message not propagated, got {diag['status']!r}"
        )
        # Warm-start state cleared per mpc.py:1229–1234.
        assert mpc._prev_w is None
        assert mpc._prev_lam_g is None
        assert mpc._prev_lam_x is None
        assert mpc._timeout_hint is None
        assert mpc._timeout_lam_g is None
        assert mpc._timeout_lam_x is None
        # Fallback step counter reset (also per mpc.py:1235).
        assert mpc._fallback_step == 0
        # cmd still safe.
        assert np.all(np.isfinite(cmd))
        # Counter still incremented through _handle_failure.
        assert mpc.consecutive_failures == 1


# ---------------------------------------------------------------------
# T-U-T1a-4: Restoration_Failed — not drivable via MPCParams
# ---------------------------------------------------------------------

@pytest.mark.xfail(
    reason=(
        "Restoration_Failed is not drivable through the MPCParams "
        "surface in CasADi 3.7.2.  Driving it requires ipopt "
        "internals — start_with_resto=yes and "
        "expect_infeasible_problem=yes plus a very tight restoration "
        "iter cap — none of which are exposed via MPCParams.  Adding "
        "those would be a production-code change orthogonal to Plan "
        "2's 'test additions only' discipline.  Structural coverage "
        "of the keyword classifier for this status string is in "
        "TestFallbackKeywordMatrix below.  See "
        "logbook/2026-05-11-tier1a-real-solver-failures.md Discussion "
        "for the full gap analysis.  Permanently acceptable per "
        "Plan 2's archival-gate language."
    ),
    strict=True,
)
class TestRestorationFailedNotDrivable:
    """T-U-T1a-4 placeholder.  See xfail marker above."""

    def test_restoration_failed_drivable_via_mpcparams(self, plant):
        plant.reset()
        # Best-effort attempt: infeasibility + tight iter cap should
        # *sometimes* surface Restoration_Failed before hitting
        # max_iter.  Empirically (probe runs documented in the
        # logbook) this never converges to Restoration_Failed on
        # CasADi 3.7.2 — IPOPT either hits Maximum_Iterations_Exceeded
        # first or completes restoration into Infeasible_Problem_Detected.
        mpc = _create_mpc(plant, max_iter=50)
        _pin_q_decision_bounds_infeasible(mpc, value=1000.0)
        state = plant.get_state()
        _, _, diag = mpc.solve(state, REF_NORMAL)
        assert 'Restoration_Failed' in diag['status'], (
            f"expected Restoration_Failed; got {diag['status']!r} "
            f"(empirically: IPOPT does not surface this status on our "
            f"problem shape + parameter range; see logbook)"
        )


# ---------------------------------------------------------------------
# T-U-T1a-6: success-keyword classifier matrix
# ---------------------------------------------------------------------

# (status_string, expect_classified_as_success)
#
# This is the union of (a) the two strings ``mpc.py:1082`` treats as
# success, and (b) every other documented IPOPT status string the
# pinned CasADi 3.7.2 may emit.  The two success strings live at
# mpc.py:1082; the failure strings come from IPOPT's documented exit-
# code enumeration.  Future CasADi upgrades that add a new status
# string (e.g., the ``Maximum_WallTime_Exceeded`` introduced in newer
# IPOPT releases) should add a row here.
_FALLBACK_KEYWORD_MATRIX = [
    # IPOPT success strings — classifier MUST treat these as success.
    ('Solve_Succeeded',                  True),
    ('Solved_To_Acceptable_Level',       True),
    # Every other documented IPOPT status string — classifier MUST
    # route these to _handle_failure.
    ('Maximum_CpuTime_Exceeded',         False),
    ('Maximum_Iterations_Exceeded',      False),
    ('Infeasible_Problem_Detected',      False),
    ('Restoration_Failed',               False),
    ('Search_Direction_Becomes_Too_Small', False),
    ('Diverging_Iterates',               False),
    ('User_Requested_Stop',              False),
    ('Internal_Error',                   False),
]


class TestFallbackKeywordMatrix:
    """T-U-T1a-6 — classifier matrix.

    Verifies the literal-tuple match at
    [mpc.py:1082](../../controller/mpc.py)
    (``ret in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')``)
    behaves correctly across the entire documented IPOPT status-string
    set, regardless of whether the underlying solver actually emits
    each string on our problem shape.

    This test guards three regression classes the real-driver tests
    above don't catch (see Phase 1 logbook Discussion for the full
    rationale):

    1. ``Solved_To_Acceptable_Level`` → success.  IPOPT essentially
       never emits this on our well-conditioned problem, so the
       real-driver tests don't exercise it — but the production code
       *depends on* the classifier accepting it as success.
    2. Tuple-structure regressions (typo, dropped entry, Python
       implicit string-concat ``'a' 'b'`` collapsing two entries
       into one).
    3. Future-CasADi string-rename surface: when the suite is run
       against a new CasADi version, this matrix is the diff anchor
       for any renamed / added status string.

    Strategy: monkey-patch ``mpc._solver`` with ``_StatsInjector`` so
    the real solver runs (producing valid ``sol['x']`` etc.) but the
    ``return_status`` is the caller-supplied string.  The classifier
    at line 1082 then sees the injected string and routes accordingly.
    """

    @pytest.mark.parametrize('status,expect_success', _FALLBACK_KEYWORD_MATRIX)
    def test_keyword_classifier(self, plant, status, expect_success):
        plant.reset()
        mpc = _create_mpc(plant)
        # Inject the status string for one solve.
        mpc._solver = _StatsInjector(mpc._solver, status)
        state = plant.get_state()
        cmd, _vel, diag = mpc.solve(state, REF_NORMAL)

        if expect_success:
            assert diag['status'] == status, (
                f"success classifier should report the raw IPOPT status "
                f"unchanged; got {diag['status']!r} for injected {status!r}"
            )
            assert mpc.consecutive_failures == 0, (
                f"success-classified status must NOT increment the "
                f"failure counter; got {mpc.consecutive_failures} for "
                f"{status!r}"
            )
        else:
            assert status in diag['status'], (
                f"fallback-classified status should wrap the raw status "
                f"as 'fallback(<status>)' / 'hold(<status>)' / "
                f"'cold_hold(<status>)'; got {diag['status']!r} for "
                f"injected {status!r}"
            )
            assert any(prefix in diag['status']
                       for prefix in ('fallback(', 'hold(', 'cold_hold(',
                                      'hold_extrap(')), (
                f"fallback-classified status should carry one of the "
                f"_handle_failure prefixes; got {diag['status']!r}"
            )
            assert mpc.consecutive_failures == 1, (
                f"fallback-classified status must increment the failure "
                f"counter; got {mpc.consecutive_failures} for {status!r}"
            )
        assert np.all(np.isfinite(cmd))


# ---------------------------------------------------------------------
# Critical-detail assertions: warm-start integrity + counter dynamics
# ---------------------------------------------------------------------

class TestFailureBookkeeping:
    """Critical-detail assertions from the Phase 1 plan:

      * "After a real failure, the next solve with normal params must
        succeed — verify the warm-start clearing logic at
        [mpc.py:1228–1234](../../controller/mpc.py) doesn't permanently
        kill the controller."
      * "Check ``_consecutive_failures`` counter increments and resets
        correctly across these scenarios."

    These tests are scoped to the real-IPOPT-failure paths (CpuTime /
    Iter / Infeasible).  Warm-start clearing on the exception path is
    asserted in ``test_casadi_exception_routes_through_handler``
    above.
    """

    def test_warm_start_intact_after_solver_failure(self, plant):
        """A failed IPOPT solve (non-exception) MUST leave ``_prev_w``
        intact when one exists — only the exception path clears it.

        The seed-then-fail pattern: a successful solve populates
        ``_prev_w``; a subsequent failing solve (with the warm-start
        still in place) must NOT zero ``_prev_w`` — the
        ``_handle_failure`` walk-forward path reads it.
        """
        plant.reset()
        mpc = _create_mpc(plant)
        _seed_warm_start(plant, mpc)
        prev_w_id_before = id(mpc._prev_w)
        prev_w_snapshot = mpc._prev_w.copy()

        # Inject a Maximum_CpuTime_Exceeded via _StatsInjector — runs
        # the real solver to completion, then re-tags as failure.
        # (Using stats injection rather than max_cpu_time so the
        # warm-start *survives* the IPOPT call into _handle_failure;
        # the real max_cpu_time=1e-3 path may unsettle prev_w via the
        # timeout_hint capture even before _handle_failure runs.)
        mpc._solver = _StatsInjector(mpc._solver, 'Maximum_CpuTime_Exceeded')
        state = plant.get_state()
        _, _, diag = mpc.solve(state, REF_NORMAL)
        assert 'Maximum_CpuTime_Exceeded' in diag['status']

        # _prev_w identity preserved (same buffer — W4a pre-allocation
        # discipline).  Contents unchanged because _handle_failure
        # doesn't write to _prev_w on the walk-forward path.
        assert mpc._prev_w is not None
        assert id(mpc._prev_w) == prev_w_id_before
        np.testing.assert_array_equal(mpc._prev_w, prev_w_snapshot)

    def test_consecutive_failures_resets_on_recovery(self, plant):
        """``_consecutive_failures`` MUST reset to 0 the moment a
        success arrives (per [mpc.py:1105](../../controller/mpc.py)),
        regardless of how many failures preceded it.  Regression guard
        for the failure-counter-saturation class."""
        plant.reset()
        # Construct an MPC that fails, fails, then succeeds:
        # use max_cpu_time=1e-3 for the failing arm, then swap in a
        # fresh MPC with normal params for the success arm.  We swap
        # via re-creating the solver (not the controller) so the
        # counter persists across the swap.
        mpc = _create_mpc(plant, max_cpu_time=1e-3)
        # Two failing solves.
        for _ in range(2):
            plant.reset()
            state = plant.get_state()
            mpc.solve(state, REF_NORMAL)
        assert mpc.consecutive_failures == 2

        # Swap in a generous-budget solver from a sibling MPC built
        # with the same plant / NLP shape.  The controller's warm-start
        # state and ``_consecutive_failures`` counter persist on
        # ``mpc``; only the solver's runtime IPOPT options change.
        # This is closer to the real production recovery (CPU
        # contention lifts → next solve has full budget) than
        # synthesising the success via ``_StatsInjector`` would be.
        plant.reset()
        relaxed = _create_mpc(plant, max_cpu_time=2.0, max_iter=500)
        mpc._solver = relaxed._solver
        state = plant.get_state()
        _, _, diag = mpc.solve(state, REF_NORMAL)
        assert diag['status'] == 'Solve_Succeeded', (
            f"recovery solve should succeed under relaxed budget; "
            f"got {diag['status']!r}"
        )
        assert mpc.consecutive_failures == 0, (
            f"_consecutive_failures must reset on success; got "
            f"{mpc.consecutive_failures}"
        )

    def test_recovery_after_real_infeasibility(self, plant):
        """After a real ``Infeasible_Problem_Detected``, the next solve
        with restored bounds MUST succeed.  Verifies the failure path
        doesn't poison solver state in a way that permanently breaks
        the controller — a critical property for production where
        infeasibility may be transient (e.g., a momentary
        K1–K6-violating reference)."""
        plant.reset()
        mpc = _create_mpc(plant)
        lbw_original = mpc._lbw.copy()
        ubw_original = mpc._ubw.copy()

        # Force infeasibility.
        _pin_q_decision_bounds_infeasible(mpc, value=1000.0)
        state = plant.get_state()
        _, _, diag_fail = mpc.solve(state, REF_NORMAL)
        assert 'Infeasible_Problem_Detected' in diag_fail['status']
        assert mpc.consecutive_failures == 1

        # Restore bounds; next solve must succeed.
        mpc._lbw = lbw_original
        mpc._ubw = ubw_original
        plant.reset()
        state = plant.get_state()
        _, _, diag_ok = mpc.solve(state, REF_NORMAL)
        assert diag_ok['status'] == 'Solve_Succeeded', (
            f"recovery after Infeasible_Problem_Detected should succeed; "
            f"got {diag_ok['status']!r}"
        )
        assert mpc.consecutive_failures == 0
