"""Plan 2 Phases 1 & 2 — real IPOPT exit codes + fallback-escalation cascade.

**Phase 1 (Tier 1a)** — exercise the ``MPCController`` failure-handling
machinery against the **actual exit codes** IPOPT emits, plus the
canonical-string classifier at the success-vs-fallback boundary
(``mpc.py`` ``solve()`` ``ret in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')``).

The pre-Plan-2 suite drives fallback only via a synthetic "infeasible
reference" pattern (see [test_mpc_static.py::TestFallback](test_mpc_static.py)),
which routes through ``cold_hold(Maximum_Iterations_Exceeded)`` — a
specific code path that doesn't exercise the keyword classifier on the
full set of strings IPOPT may emit, or on the strings the classifier
*depends on* but our problem rarely produces (``Solved_To_Acceptable_Level``).

Test IDs from
[plans/archived/2026-05-18 mpc-sadpath-coverage-tiers-1-3.md](../../plans/archived/2026-05-18%20mpc-sadpath-coverage-tiers-1-3.md)
Phase 1:

| ID         | Real driver                                          | Exit code                       |
|------------|------------------------------------------------------|---------------------------------|
| T-U-T1a-1  | ``max_cpu_time=1e-3``                                | ``Maximum_CpuTime_Exceeded``    |
| T-U-T1a-2  | pin ``q``-decision bounds (singleton infeasible)     | ``Infeasible_Problem_Detected`` |
| T-U-T1a-3  | ``max_iter=1``                                       | ``Maximum_Iterations_Exceeded`` |
| T-U-T1a-4  | (xfail) — not drivable via ``MPCParams`` in 3.7.2    | ``Restoration_Failed``          |
| T-U-T1a-5  | monkey-patch ``_solver`` to raise                    | ``exception: …``                |
| T-U-T1a-6  | parametrize all status strings; stats() injection    | (all)                           |

**Phase 2 (Tier 1b)** — drive the fallback escalation cascade and the
cold-start IK budget exhaustion path.  Each Tier-1b test is a *targeted*
probe of one escalation branch — the real-driver discipline is preserved
end-to-end on the *failure* arm (every failure is produced by injecting
an IPOPT status into a real solver call), and the escalation triggers
themselves are driven through the smallest internal-state mutation that
isolates one branch:

| ID         | Branch                                                            | Driver                                                                                  |
|------------|-------------------------------------------------------------------|-----------------------------------------------------------------------------------------|
| T-U-T1b-3  | Staleness escalation → ``cold_hold`` via >500 ms                  | ``time.sleep(0.6)`` between seed and failure                                            |
| T-U-T1b-4  | Homogeneous cascade emission (all ``fallback_extrap``, no escalation) | small ref, inject N+2 consecutive failures via ``_StatsInjector``                       |
| T-U-T1b-5  | cold-start IK budget exhaustion                                    | monkey-patch ``_numerical_ik`` to ``time.sleep(0.42)`` per call                          |
| T-U-T1b-6  | warm-start integrity property (Hypothesis stateful)                | ``RuleBasedStateMachine`` over (succeed_solve, fail_solve, advance_time)                |

Tier-1b history: the original Phase 2 (commits prior to 2026-05-20) also
included T-U-T1b-1 and T-U-T1b-2, which exercised the W7 "20 mm ref
shift" walk-forward-unsafe guard.  The Tier-1 fallback rewrite of
2026-05-20 (logbook
``2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md``) replaced
the walk-forward + ``hold_extrap`` ladder with a single linear
cmd-stream extrapolation arm + 500 ms ``cold_hold`` time bound; the
20 mm threshold no longer exists, so T-U-T1b-1 and T-U-T1b-2 were
deleted and T-U-T1b-3 / T-U-T1b-4 were rewritten with the semantics
shown above.  The remaining 500 ms staleness number is the one
surviving normative threshold; the tests use *current symbol names*
(``_t_at_last_success``, ``_numerical_ik``, etc.) rather than line
numbers — line citations in this file's docstrings are indicative
anchors for code-review, not assertion targets.

Pinned CasADi: **3.7.2**.  IPOPT's status-string set is stable across
patch releases; a CasADi major-version upgrade should re-run this matrix
to confirm no string was renamed.

See logbook entries:
* [2026-05-11-tier1a-real-solver-failures.md](../../logbook/2026-05-11-tier1a-real-solver-failures.md)
* [2026-05-11-tier1b-fallback-escalation-cascade.md](../../logbook/2026-05-11-tier1b-fallback-escalation-cascade.md)
"""

from __future__ import annotations

import logging
import time as _time

import numpy as np
import pytest
from hypothesis import strategies as st
from hypothesis.stateful import RuleBasedStateMachine, invariant, rule

from sim.plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams



# NIGHTLY TIER — the MPC is operationally dormant (plans/active/refactor-2026-07.md
# Phase 3: jugglebot_launch.py no longer starts motor_guard/motion_bridge_node; the
# leg path is trajectory_node -> teensy_bridge_node -> the Teensy MAX_DEVIATION
# guard). The code is parked, not deleted, so this battery is parked with it: it
# runs nightly via tools/nightly_suite.sh and on `./run_tests.sh --full`, which is
# mandatory before any hardware sitting. Promotion back to per-commit is step 4 of
# the MPC revival.
pytestmark = pytest.mark.nightly


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

        # Post-Tier-1-rewrite (2026-05-20): the exception handler at
        # mpc.py:1268-1283 clears the warm-start vectors (_prev_w,
        # _prev_lam_g, _prev_lam_x, _timeout_hint) but does NOT clear
        # _prev_u / _prev_prev_u (the cmd stream history is preserved
        # for the new fallback_extrap arm).  So the exception path now
        # routes through fallback_extrap rather than the deleted
        # hold_extrap arm.  Pre-rewrite this asserted hold/cold_hold;
        # post-rewrite accept any fallback-class family.
        assert any(diag['status'].startswith(p)
                   for p in ('fallback_extrap(', 'fallback_hold(',
                             'cold_hold(', 'hold(', 'hold_extrap(')), (
            f"exception path should route through a fallback-class "
            f"family, got {diag['status']!r}"
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
    # Discovered by Phase 3's NaN/Inf input fuzz on solve(): when an
    # adversarial input flows into the optimization, IPOPT detects the
    # NaN at the first gradient evaluation and exits with this status
    # rather than ``Internal_Error`` or ``Infeasible_Problem_Detected``.
    # See logbook 2026-05-11-tier1c-input-fuzz.md.
    ('Invalid_Number_Detected',          False),
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
                       for prefix in ('fallback(', 'fallback_extrap(',
                                      'fallback_hold(', 'cold_hold(',
                                      # Legacy (pre-2026-05-20 hold_extrap
                                      # / hold arms; no longer produced
                                      # but accepted for forward compat).
                                      'hold(', 'hold_extrap(')), (
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


# =====================================================================
# Phase 2 (Tier 1b) — fallback escalation cascade + cold-start IK budget
# =====================================================================
#
# All Phase 2 tests share the helpers above (``_create_mpc``,
# ``_seed_warm_start``, ``_StatsInjector``).  The *failure* arm of every
# test stays on the real-driver path: ``_StatsInjector`` runs the real
# CasADi solver to completion, then re-tags ``return_status`` so the
# classifier at the success-vs-fallback boundary routes the call into
# ``_handle_failure``.  The *escalation trigger* itself is driven by
# the smallest mutation that isolates one branch of the cascade.
#
# See the module docstring's per-test recipe table for the full list.


# ---------------------------------------------------------------------
# T-U-T1b-1, -2: walk_forward_unsafe via ref-shift threshold (pair test)
# ---------------------------------------------------------------------
#
# DELETED 2026-05-20 — the Tier-1 fallback rewrite removed the
# walk-forward arm and its W7 escalation logic entirely.  The 20 mm
# ref-shift threshold no longer exists as a code branch; cmd-stream
# extrapolation does not read the reference at all during fallback,
# so a ref shift cannot perturb the emission.  The underlying safety
# property (cmd stays bounded when the reference shifts mid-cascade)
# is now structural rather than gated on a threshold — see
# tests/sim/test_mpc_static.py::test_fallback_no_positive_feedback_growth
# and logbook/2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md.


# ---------------------------------------------------------------------
# T-U-T1b-3: walk_forward_unsafe via 500 ms staleness
# ---------------------------------------------------------------------

class TestStalenessEscalation:
    """The 500 ms staleness escalation survived the Tier-1 fallback
    rewrite (2026-05-20).  Post-rewrite, when the last-success
    snapshot is older than 500 ms by ``_time.perf_counter()``
    reckoning, the fallback arm escalates to ``cold_hold(q_cur)``
    (was ``hold_extrap(...)`` pre-rewrite).

    Driver: real wall-clock ``time.sleep(0.6)`` between the seed solve
    and the failure-injected solve.  Mirrors Phase 1's
    ``test_max_cpu_time_exceeded`` wall-clock-budget idiom.  Cost:
    0.6s per test run.
    """

    def test_staleness_above_500ms_escalates(self, plant):
        """T-U-T1b-3 (updated 2026-05-20 for Tier-1 fallback rewrite):
        600 ms wall-clock advance after the seed solve MUST flip the
        next fallback to ``cold_hold(...)`` (was ``hold_extrap(...)``
        pre-rewrite).

        The escalation now bypasses the cmd-stream extrapolation arm
        directly to ``cold_hold(q_cur)`` — a measured-position hold
        with ``cmd_vel = 0`` — because at >500 ms of cascade we no
        longer trust the cmd stream's recent history as a forward
        predictor (the original W7 staleness rationale, preserved).
        """
        plant.reset()
        mpc = _create_mpc(plant)
        _seed_warm_start(plant, mpc)

        # Real-time advance of 600 ms.
        _time.sleep(0.6)

        mpc._solver = _StatsInjector(mpc._solver, 'Maximum_CpuTime_Exceeded')
        state = plant.get_state()
        cmd, cmd_vel, diag = mpc.solve(state, REF_NORMAL)

        assert diag['status'] == 'cold_hold(Maximum_CpuTime_Exceeded)', (
            f">500 ms staleness must escalate to cold_hold; got "
            f"{diag['status']!r}"
        )
        # Plan 2 Phase 7 bugfix: fallback_step is uniformly -1
        # sentinel on every path.  See DIAG_SCHEMA_CONTRACT.md.
        assert diag.get('fallback_step') == -1
        assert mpc.consecutive_failures == 1
        assert np.all(np.isfinite(cmd))
        # cmd_vel must be zero on the cold_hold escalation arm
        # (no extrapolation; measured-position hold).
        np.testing.assert_array_equal(cmd_vel, np.zeros(6))


# ---------------------------------------------------------------------
# T-U-T1b-4: escalation after max_consecutive_failures
# ---------------------------------------------------------------------

class TestHomogeneousCascadeEmission:
    """The Tier-1 fallback rewrite (2026-05-20) replaced the
    counter-gated walk-forward → hold_extrap ladder with a single
    homogeneous arm: linear cmd-stream extrapolation
    (``fallback_extrap(...)``) for the full duration of a cascade,
    until the 500 ms time bound fires and escalates to
    ``cold_hold(...)``.

    Why this test replaces the prior ``TestMaxConsecutiveFailuresEscalation``:
    the original test asserted a *counter-based escalation* pattern
    (statuses == fallback, fallback, hold_extrap, hold_extrap with
    max_consecutive_failures=2).  That branching no longer exists —
    counter-based escalation was removed because the empirical
    evidence from the 2026-05-20 chaotic-motion event showed
    plant-state-driven escalation (``hold_extrap``) introduced a
    positive-feedback bug class that the cmd-stream primitive avoids
    structurally.  This test guards the new homogeneous behaviour:
    every fallback in a short cascade (well under 500 ms) emits
    ``fallback_extrap(...)``, never ``hold_extrap(...)``.
    """

    def test_short_cascade_stays_in_fallback_extrap(self, plant):
        """T-U-T1b-4 (rewritten 2026-05-20): 4 consecutive failures
        well under the 500 ms time bound all emit
        ``fallback_extrap(Maximum_CpuTime_Exceeded)`` — no escalation
        to ``hold_extrap`` (deleted arm) or ``cold_hold`` (only fires
        at the 500 ms time bound).
        """
        plant.reset()
        mpc = _create_mpc(plant)
        _seed_warm_start(plant, mpc)

        # Inject 4 consecutive failures.  Wall-clock duration of the
        # loop is well under the 500 ms time bound.
        mpc._solver = _StatsInjector(mpc._solver, 'Maximum_CpuTime_Exceeded')
        state = plant.get_state()
        diags = []
        for _ in range(4):
            _, _, diag = mpc.solve(state, REF_NORMAL)
            diags.append(diag.copy())

        statuses = [d['status'] for d in diags]
        for s in statuses:
            assert s == 'fallback_extrap(Maximum_CpuTime_Exceeded)', (
                f"Tier-1 rewrite (2026-05-20) made every short-cascade "
                f"fallback emit fallback_extrap.  Got statuses={statuses}. "
                f"If a hold_extrap status appears, the deleted arm has "
                f"been reintroduced — see logbook "
                f"2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md."
            )
        # fallback_step is the -1 sentinel on every path (walk-forward
        # arm removed; see DIAG_SCHEMA_CONTRACT.md History entry).
        for i, d in enumerate(diags):
            assert d['fallback_step'] == -1, (
                f"fallback_step on diag[{i}] should be -1 sentinel; "
                f"got {d['fallback_step']!r}")
        # Counter increments through every failure.
        assert mpc.consecutive_failures == 4


# ---------------------------------------------------------------------
# T-U-T1b-5: cold-start IK budget exhaustion
# ---------------------------------------------------------------------

class TestColdStartIkBudget:
    """The cold-start IK loop in
    [``_cold_start``](../../controller/mpc.py) is guarded by a budget
    of ``max_cpu_time × _COLD_START_BUDGET_FRACTION`` (= 30%).  When
    per-node IK exceeds the budget mid-loop, the remaining nodes fall
    back to linear interpolation; an ``INFO`` log is emitted naming the
    node index where the budget was breached.

    Sentinel discipline: the plan hedged
    ``diag['cold_start_method'] == 'linear_interp'`` "or whichever
    sentinel the code surfaces".  Empirically (see logbook probe), the
    code surfaces NO ``diag`` key for this branch — the only observable
    is the ``logger.info`` message.  This test asserts on the log
    record via the standard library's logging infrastructure (since the
    suite captures via pytest's ``caplog`` fixture only when the test
    declares it).  We use a manual handler attached to
    ``logging.getLogger('controller.mpc')`` to avoid the global level/
    filter chain that ``caplog`` configures.

    Driver discipline: monkey-patch ``_numerical_ik`` on the instance
    to ``time.sleep(0.42s)`` per call.  With ``max_cpu_time=2.0`` →
    budget ≈ 0.6s.  ``t0 = perf_counter()`` is captured BEFORE the
    pre-loop ``q_final_approx = self._numerical_ik(ref_traj[N])`` call,
    so that 0.42s elapses inside the budget window.  The loop's first
    check at ``k=1`` sees ≈0.42s (under budget) and runs another IK
    (≈0.84s elapsed); the ``k=2`` check sees ≈0.84s (over the 0.6s
    budget) → break → linear-interp the remainder.  Empirically this
    triggers the log at "node 2/10".

    Critical post-fallback property: the cold-start STILL completes
    successfully (linear-interp seed remains a valid IPOPT initial
    guess), and the downstream solve returns a finite ``cmd``.  The
    budget guard is defense-in-depth, not a failure mode.
    """

    @staticmethod
    def _make_slow_ik(real_ik, sleep_s: float):
        """Wrap ``_numerical_ik`` to sleep before delegating."""
        def slow_ik(pose):
            _time.sleep(sleep_s)
            return real_ik(pose)
        return slow_ik

    def test_per_node_ik_budget_exhaustion_falls_back_to_linear_interp(
            self, plant):
        """T-U-T1b-5: with each ``_numerical_ik`` call sleeping 0.42s
        and ``max_cpu_time=2.0`` (→ budget=0.6s), the cold-start
        budget guard MUST fire before all N+1 nodes process; the log
        record must name the breach node and the solve must still
        return a finite ``cmd``."""
        plant.reset()
        mpc = _create_mpc(plant, max_cpu_time=2.0)
        # Verify cold-start path is the one we'll hit: prev_w / hint
        # both None at construction (prime_solver=False is mandatory in
        # _create_mpc, so no warm-start has been seeded).
        assert mpc._prev_w is None
        assert mpc._timeout_hint is None

        real_ik = mpc._numerical_ik
        mpc._numerical_ik = self._make_slow_ik(real_ik, sleep_s=0.42)

        # Capture INFO records from controller.mpc.
        records = []

        class _Capture(logging.Handler):
            def emit(self, record):
                records.append(record)

        mpc_logger = logging.getLogger('controller.mpc')
        prior_level = mpc_logger.level
        mpc_logger.setLevel(logging.INFO)
        handler = _Capture()
        mpc_logger.addHandler(handler)
        try:
            state = plant.get_state()
            cmd, _, diag = mpc.solve(state, REF_NORMAL)
        finally:
            mpc_logger.removeHandler(handler)
            mpc_logger.setLevel(prior_level)
            # Restore _numerical_ik so any module-scoped reuse of the
            # plant fixture isn't poisoned by the slow patch (the
            # plant fixture is shared, but the MPC is per-test; this is
            # belt-and-braces).
            mpc._numerical_ik = real_ik

        budget_msgs = [
            r.getMessage() for r in records
            if 'budget exceeded' in r.getMessage()
        ]
        assert len(budget_msgs) == 1, (
            f"expected exactly one 'per-node IK budget exceeded' INFO log "
            f"during cold-start with slow IK injection; got "
            f"{len(budget_msgs)} messages.  All captured records: "
            f"{[r.getMessage() for r in records]}"
        )
        msg = budget_msgs[0]
        assert msg.startswith('_cold_start: per-node IK budget exceeded'), (
            f"log message format drifted; got {msg!r}.  If the log was "
            f"reformatted, refresh this assertion."
        )
        assert 'linear-interpolating remainder' in msg, (
            f"log must name the linear-interp fallback action; got "
            f"{msg!r}"
        )
        # Cold-start completes; downstream solve returns finite cmd.
        # (The solve may succeed or hit the existing CPU-time budget
        # depending on Jetson load — both are acceptable here.  What
        # matters for T-U-T1b-5 is the budget guard fired and the
        # tick produced a safe command.)
        assert np.all(np.isfinite(cmd)), (
            f"cmd must be finite even when cold-start IK budget guard "
            f"fires; got cmd={cmd!r}, status={diag['status']!r}"
        )
        # No diag sentinel is emitted by the budget guard (see class
        # docstring).  Asserting this explicitly so a future refactor
        # that DOES add a diag key trips this test and prompts a
        # docstring update.
        assert 'cold_start_method' not in diag, (
            f"a diag['cold_start_method'] key now exists — Plan 2 Phase 2 "
            f"asserted this branch had no diag sentinel; refresh the "
            f"plan, the logbook, and this test if the contract changed."
        )


# ---------------------------------------------------------------------
# T-U-T1b-6: warm-start integrity property — Hypothesis stateful
# ---------------------------------------------------------------------

# Module-scoped plant + MPC singletons reused across StateMachine
# instances.  Building a fresh MuJoCoPlant + MPCController per test
# case is too slow at ci-deep (1000 examples × ~3s each).  The
# StateMachine ``__init__`` is responsible for fully restoring the
# singletons to a clean baseline before each walk; the *real* solver
# captured at module init is the source of truth — prior walks may
# have swapped it for a ``_StatsInjector``, so the restore is
# unconditional, not conditioned on a per-instance flag (the prior
# bug was: per-instance ``_real_solver`` capture re-captured the
# injector left over from the previous walk → ``__init__``'s seed
# solve ran against the injector → flaky generation under shrinking).
_W7_PROP_PLANT: MuJoCoPlant | None = None
_W7_PROP_MPC: MPCController | None = None
_W7_PROP_REAL_SOLVER = None  # captured ONCE, at module init


def _w7_get_singletons():
    global _W7_PROP_PLANT, _W7_PROP_MPC, _W7_PROP_REAL_SOLVER
    if _W7_PROP_PLANT is None:
        _W7_PROP_PLANT = MuJoCoPlant()
        _W7_PROP_MPC = _create_mpc(_W7_PROP_PLANT)
        _W7_PROP_REAL_SOLVER = _W7_PROP_MPC._solver
    return _W7_PROP_PLANT, _W7_PROP_MPC, _W7_PROP_REAL_SOLVER


class WarmStartIntegrityMachine(RuleBasedStateMachine):
    """T-U-T1b-6 — warm-start integrity invariant under arbitrary
    sequences of (succeed_solve, fail_solve, advance_time).

    **Invariant (the load-bearing property):** at every observable
    point in the random walk, ``mpc._prev_w`` is either ``None`` or
    contains only finite values.  No combination of failures or
    staleness advances may leave the warm-start in a NaN/Inf state —
    that would corrupt the next solve and cascade.

    This invariant is the warm-start integrity contract documented at
    the success branch of [``solve()``](../../controller/mpc.py): the
    success path checks ``np.all(np.isfinite(w_opt))`` and routes to
    ``_handle_failure`` (with status ``non_finite_solution``) on
    violation; the exception path clears warm-start state to ``None``.
    The failure paths exercised in Phase 1 already validated that
    single-shot failures preserve the invariant.  T-U-T1b-6 verifies it
    across-call under arbitrary sequences of failure / success /
    time-advance — the post-Tier-1-fallback-rewrite cascade triggers
    (logbook
    ``2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md``).

    History: prior to the Tier-1 fallback rewrite this class also
    included a ``shift_snapshot_z`` rule (mutating
    ``_ref_at_last_success_mid`` to cross the W7 20 mm ref-shift
    threshold) and a ``snapshot_finite_or_none`` invariant on the W7
    snapshot fields.  Both were removed when the walk-forward +
    ``hold_extrap`` ladder was deleted; the surviving rules and
    invariants are the load-bearing slice that survives the rewrite.

    Plan choice: ``RuleBasedStateMachine`` rather than ``@composite + @given``
    because the invariant is across-call — only a stateful walk
    exposes the kind of (failure, succeed, fail, advance, fail) sequence
    that could plausibly leave ``_prev_w`` in an inconsistent state.
    Mirrors [``SchedulerStateMachine`` in
    test_scheduler_contract.py](test_scheduler_contract.py) as the
    in-repo exemplar for stateful property tests on safety-critical
    invariants.

    Plant + MPC are module-scoped singletons (see ``_w7_get_singletons``)
    rather than per-instance to keep ci-deep wall-clock under control.
    The ``__init__`` resets both before each StateMachine walk; the
    rules drive only the public + internal-state surfaces relevant to
    the warm-start integrity property.
    """

    def __init__(self):
        super().__init__()
        self._plant, self._mpc, real_solver = _w7_get_singletons()
        # UNCONDITIONALLY restore the real solver before resetting state
        # — the prior walk may have left a _StatsInjector in place.
        self._mpc._solver = real_solver
        self._real_solver = real_solver
        self._plant.reset()
        self._mpc.reset()
        # Track current failure-injection state.
        self._injector_active = False
        # Seed one successful solve so prev_w / prev_u / snapshot are
        # populated (otherwise every fail rule routes through
        # cold_hold and never touches the W7 escalation paths).
        self._do_succeed()

    # ---- Internal helpers ----

    def _do_succeed(self):
        """Run one real solve.  No-op if solver is currently swapped
        for a _StatsInjector (the test author should call
        _restore_real_solver first; we do so defensively here)."""
        self._restore_real_solver()
        state = self._plant.get_state()
        _, _, diag = self._mpc.solve(state, REF_NORMAL)
        # The seeded solve must succeed for the rules to stay on a
        # well-defined branch of the W7 cascade.
        assert diag['status'] == 'Solve_Succeeded', (
            f"seeded succeed_solve must converge under default params; "
            f"got {diag['status']!r}"
        )

    def _restore_real_solver(self):
        if self._injector_active:
            self._mpc._solver = self._real_solver
            self._injector_active = False

    def _activate_injector(self, status: str):
        if not self._injector_active:
            self._mpc._solver = _StatsInjector(self._real_solver, status)
            self._injector_active = True

    # ---- Rules: state mutations ----

    @rule(status=st.sampled_from([
        'Maximum_CpuTime_Exceeded',
        'Maximum_Iterations_Exceeded',
        'Infeasible_Problem_Detected',
    ]))
    def fail_solve(self, status):
        """Inject one failure with the chosen IPOPT status string."""
        self._activate_injector(status)
        state = self._plant.get_state()
        self._mpc.solve(state, REF_NORMAL)

    @rule()
    def succeed_solve(self):
        """Run one real solve — restores _prev_w to a fresh post-success
        snapshot.  Resets the W7 timer + ref snapshot as a side
        effect."""
        self._do_succeed()

    @rule(t_advance=st.floats(min_value=0.0, max_value=1.5,
                               allow_nan=False, allow_infinity=False))
    def advance_time(self, t_advance):
        """Simulate wall-clock advance by retroactively aging the
        snapshot timestamp.  Crosses the 500 ms staleness threshold
        when t_advance > 0.5."""
        if self._mpc._t_at_last_success is None:
            return
        self._mpc._t_at_last_success -= t_advance

    # ---- Invariants: structural predicates ----

    @invariant()
    def warm_start_finite_or_none(self):
        """The load-bearing property.  ``_prev_w`` must be either
        ``None`` (cleared on exception path or initial state) or
        contain only finite values (success path's finite-check at
        the success branch of ``solve()``)."""
        w = self._mpc._prev_w
        assert w is None or np.all(np.isfinite(w)), (
            f"W7 invariant violation: _prev_w contains non-finite "
            f"values: {w!r}"
        )

    @invariant()
    def prev_u_finite_or_none(self):
        """Companion invariant: ``_prev_u`` (the last commanded
        leg-extension vector) must be finite or None.  Used by the
        Tier-1 fallback (``fallback_extrap`` arm) as both the
        rate-limit anchor and the extrapolation base
        (``cmd = 2·_prev_u − _prev_prev_u``)."""
        u = self._mpc._prev_u
        assert u is None or np.all(np.isfinite(u)), (
            f"_prev_u contains non-finite values: {u!r}"
        )


TestWarmStartIntegrity = WarmStartIntegrityMachine.TestCase
