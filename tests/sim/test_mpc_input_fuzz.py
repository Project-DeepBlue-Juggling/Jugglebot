"""Plan 2 Phase 3 (Tier 1c) — NaN/Inf input fuzz on ``MPCController.solve()``.

This module hammers the entry-point input surface of ``solve()`` with
adversarial values (NaN, ±Inf, extreme magnitudes, wrong shapes) and
asserts two safety properties hold for every adversarial input:

1. **Warm-start integrity** — ``_prev_w`` is either ``None`` (cleared by
   the exception handler) or contains only finite values.  This is the
   load-bearing safety property: a corrupted warm-start would cascade
   into the next solve and command non-finite extensions to the
   actuators.
2. **Status-routing finiteness** — every status string falls into one
   of the documented buckets (``Solve_Succeeded`` /
   ``fallback(...)`` / ``fallback_extrap(...)`` /
   ``fallback_hold(...)`` / ``cold_hold(...)``, plus the legacy buckets
   ``hold(...)`` and ``hold_extrap(...)`` retained for replay of
   pre-2026-05-20 logs — see logbook
   ``2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md``); the
   returned ``cmd`` is always finite.

Test IDs from
[plans/archived/mpc-sadpath-coverage-tiers-1-3.md](../../plans/archived/mpc-sadpath-coverage-tiers-1-3.md)
Phase 3:

| ID         | Surface                                                              | Driver                                                                   |
|------------|----------------------------------------------------------------------|--------------------------------------------------------------------------|
| T-U-T1c-1  | NaN in ``state.platform_pos_mm``                                     | ``@composite`` strategy: PlantState with one xyz axis NaN                |
| T-U-T1c-2  | Inf in ``state.platform_rot``                                        | ``@composite`` strategy: PlantState with one rotation-vector axis ±Inf   |
| T-U-T1c-3  | NaN in ``target_pose``                                               | ``@composite`` strategy: target with one axis NaN                        |
| T-U-T1c-4  | NaN in ``ref_events[i].twist``                                       | ``@composite`` strategy: 2-event list with NaN-spiked twist on event[1]  |
| T-U-T1c-5  | Extreme magnitudes (±1e10 mm) anywhere in ``state``/``target``       | parametrised scenarios over the four live xyz/rot fields                 |
| T-U-T1c-6  | Wrong-shape inputs at the API boundary                               | parametrised scenarios over ``target_pose`` shape and ``state`` fields   |
| T-U-T1c-7  | Property: warm-start integrity across N adversarial sequences        | ``RuleBasedStateMachine`` over (succeed_solve, fuzz_solve, advance_time) |

Plus one structural test that documents the dead-field finding:

| ID         | Surface                                                | Asserts                                                            |
|------------|--------------------------------------------------------|--------------------------------------------------------------------|
| T-U-T1c-D1 | ``state.platform_twist`` is dead in ``solve()``        | NaN/Inf injection produces identical ``cmd`` to a clean field      |

**Empirical findings (probe table, 2026-05-11 on CasADi 3.7.2 + Jetson):**

| Adversarial input                          | IPOPT exit                            | Production status                              |
|--------------------------------------------|---------------------------------------|------------------------------------------------|
| NaN in ``state.platform_pos_mm[i]``        | ``Invalid_Number_Detected``           | ``fallback(Invalid_Number_Detected)``          |
| Inf in ``state.platform_rot[i]``           | ``Invalid_Number_Detected``           | ``fallback(Invalid_Number_Detected)``          |
| NaN in ``target_pose[i]``                  | ``Invalid_Number_Detected``           | ``fallback(Invalid_Number_Detected)``          |
| Inf in ``target_pose[2]``                  | ``Invalid_Number_Detected``           | ``cold_hold(Invalid_Number_Detected)`` *       |
| NaN in ``ref_events[i].twist``             | ``Invalid_Number_Detected``           | ``fallback(Invalid_Number_Detected)``          |
| ±1e10 mm in ``target_pose[i]``             | ``Solve_Succeeded``                    | ``Solve_Succeeded``                            |
| ±1e10 mm in ``state.platform_pos_mm[i]``   | ``Solve_Succeeded``                    | ``Solve_Succeeded``                            |
| Wrong-shape ``target_pose`` (n!=6)         | not reached                           | ``ValueError`` from numpy broadcast at boundary |
| NaN in ``state.leg_velocities_mmps``       | ``Solve_Succeeded`` (q_dot is failure-only) | ``Solve_Succeeded``                       |
| NaN in ``state.platform_twist``            | dead field — solve() never reads it    | ``Solve_Succeeded`` (identical cmd to clean)   |

\\* Inf in ``target_pose[2]`` produces a non-finite reference that
results in repeated solver failures; under the Tier-1 fallback rewrite
(logbook ``2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md``)
the resulting cascade trips the 500 ms staleness bound and escalates to
``cold_hold(...)``.  The pre-rewrite path routed through the W7
walk-forward-unsafe ref-shift branch and produced ``hold_extrap(...)``;
the rewrite replaced both with the homogeneous ``fallback_extrap`` arm
+ ``cold_hold`` escalation.  This is correct cascade behaviour but
worth flagging because the surface (target_pose) and the symptom (Inf)
are both Phase 3 inputs, while the path is Phase 2 escalation.

The NaN-routing path produces a NEW IPOPT exit code
``Invalid_Number_Detected`` that Phase 1's ``TestFallbackKeywordMatrix``
did not enumerate.  Phase 3 extends the matrix in
[test_solver_failures.py](test_solver_failures.py).

See [logbook/2026-05-11-tier1c-input-fuzz.md](../../logbook/2026-05-11-tier1c-input-fuzz.md).
"""
from __future__ import annotations

import logging

import numpy as np
import pytest
from hypothesis import HealthCheck, assume, given, settings, strategies as st
from hypothesis.stateful import RuleBasedStateMachine, invariant, rule

from sim.plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.target import ReferenceEvent



# NIGHTLY TIER — the MPC is operationally dormant (plans/parked/refactor-2026-07.md
# Phase 3: jugglebot_launch.py no longer starts motor_guard/motion_bridge_node; the
# leg path is trajectory_node -> teensy_bridge_node -> the Teensy MAX_DEVIATION
# guard). The code is parked, not deleted, so this battery is parked with it: it
# runs nightly via tools/nightly_suite.sh and on `./run_tests.sh --full`, which is
# mandatory before any hardware sitting. Promotion back to per-commit is step 4 of
# the MPC revival.
pytestmark = pytest.mark.nightly


# Suppress the per-fault MPC log noise from solve() failures — the
# fuzz tests intentionally generate many failures and the logger output
# is otherwise overwhelming.  Tests that need to inspect log records
# attach their own handler.
logging.getLogger('controller.mpc').setLevel(logging.CRITICAL)


CONTROL_DT = 0.025
REF_NORMAL = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])

# IPOPT detects NaN in any input that flows into the optimization on
# CasADi 3.7.2.  Used by every status assertion in this file.
_NAN_FAULT_KEYWORD = 'Invalid_Number_Detected'

# The four documented status prefixes a non-success solve may produce.
# Used by the warm-start-integrity invariant to assert "status is
# either Solve_Succeeded or one of the fallback prefixes".
_FALLBACK_PREFIXES = (
    'fallback(',           # IPOPT non-success status passed through to _handle_failure
    'fallback_extrap(',    # primary cmd-stream-extrapolation arm (Tier-1 rewrite 2026-05-20)
    'fallback_hold(',      # bootstrap arm when _prev_prev_u is None
    'hold_extrap(',        # legacy (pre-2026-05-20 hold_extrap arm; no longer produced)
    'hold(',               # legacy (pre-2026-05-20 cold-freeze arm; no longer produced)
    'cold_hold(',          # absolute-fallback arm (cold-start OR 500 ms time-bound escalation)
)


# ---------------------------------------------------------------------
# Fixtures + helpers
# ---------------------------------------------------------------------

@pytest.fixture(scope='module')
def plant():
    """Shared MuJoCoPlant for parametrised scenario tests that DO NOT
    reassign ``PlantState`` array attributes.  Per
    [PLANT_INTERFACE_CONTRACT.md](../../controller/PLANT_INTERFACE_CONTRACT.md)
    P1, ``MuJoCoPlant.get_state()`` returns the SAME ``self._state``
    instance every call, with array fields aliased to the plant's
    internal buffers and rewritten in place at each call.  Tests that
    do ``state.platform_pos_mm = np.zeros(2)`` reassign the attribute
    on the shared instance and break the alias — the next call to
    ``plant.get_state()`` then writes through the orphaned reference
    and crashes (or pollutes results).  In-place writes through the
    alias (``state.platform_pos_mm[i] = nan``) are safe — the plant
    overwrites them on the next ``get_state()``.

    Tests that MUST reassign attributes (wrong-shape tests, recovery
    test) take the function-scoped ``fresh_plant`` fixture below
    instead."""
    return MuJoCoPlant()


@pytest.fixture
def fresh_plant():
    """Function-scoped fresh ``MuJoCoPlant`` for tests that reassign
    ``PlantState`` array attributes (wrong-shape tests, recovery
    test).  See ``plant`` fixture docstring for the alias-break
    rationale.  Costs ~1-2 s per test; acceptable for the small
    number of tests that need it."""
    return MuJoCoPlant()


def _create_mpc(plant, **overrides):
    """Build an MPC for fuzz tests.  ``use_aot_solver=False`` is
    mandatory (see Phase 1's ``test_solver_failures.py::_create_mpc``
    for rationale) and ``prime_solver=False`` keeps the cold-start
    explicit so seed sequences are deterministic."""
    defaults = dict(
        max_cpu_time=2.0, max_iter=500, max_leg_vel_mmps=280.0,
        prime_solver=False, use_aot_solver=False,
    )
    defaults.update(overrides)
    return MPCController.from_plant(MPCParams(**defaults), plant)


def _seed_mpc(plant, mpc, ref=REF_NORMAL):
    """Run one successful solve so the failing-MPC scenarios have
    populated warm-start state.  Returns the seed state for callers
    that need it."""
    plant.reset()
    state = plant.get_state()
    cmd, _, diag = mpc.solve(state, ref)
    assert diag['status'] == 'Solve_Succeeded', (
        f"seed solve must succeed; got {diag['status']!r}"
    )
    return state


def _assert_warm_start_integrity(mpc):
    """The load-bearing safety invariant: ``_prev_w`` is either ``None``
    (cleared on the exception path) or contains only finite values
    (preserved by every other failure path).  Companion fields
    ``_prev_lam_g``, ``_prev_lam_x``, ``_prev_u`` follow the same
    contract."""
    for name in ('_prev_w', '_prev_lam_g', '_prev_lam_x', '_prev_u',
                 '_prev_prev_u', '_ref_at_last_success_mid',
                 '_twist_at_last_success'):
        v = getattr(mpc, name)
        assert v is None or np.all(np.isfinite(v)), (
            f"warm-start integrity violation: {name} contains "
            f"non-finite values: {v!r}"
        )


def _assert_safe_outcome(diag, cmd):
    """Generic safety assertion: the status routes into one of the
    documented buckets, the cmd is finite.  Any non-bucketed status
    indicates either a bug or a future status string the suite needs to
    learn about."""
    status = diag['status']
    is_success = status == 'Solve_Succeeded'
    is_fallback = any(status.startswith(p) for p in _FALLBACK_PREFIXES)
    assert is_success or is_fallback, (
        f"status {status!r} is neither Solve_Succeeded nor one of "
        f"{_FALLBACK_PREFIXES} — refresh _FALLBACK_PREFIXES if a new "
        f"status family was added to ``_handle_failure``"
    )
    assert np.all(np.isfinite(cmd)), (
        f"cmd MUST be finite even on adversarial input; got {cmd!r} "
        f"with status {status!r}"
    )


# ---------------------------------------------------------------------
# Hypothesis composite strategies — one PlantState mostly-valid, one
# field adversarial.
# ---------------------------------------------------------------------

# Index strategy for "which xyz axis to spike" (3 axes).
_XYZ_AXIS = st.integers(min_value=0, max_value=2)
# Index strategy for "which rotation-vector axis to spike" (3 axes).
_ROT_AXIS = st.integers(min_value=0, max_value=2)
# Adversarial scalar values: NaN, +Inf, -Inf.  Extreme magnitudes are a
# parametrised scenario rather than a fuzz axis because they're
# expected to succeed (not fault) and don't change the warm-start
# integrity behaviour.
_NAN_OR_INF = st.sampled_from([np.nan, np.inf, -np.inf])

# Hypothesis settings for module-level @given tests: ci-fast scales
# automatically via the conftest profile.  No deadline — solve() is
# inherently O(20 ms) on a cold start.
_GIVEN_SETTINGS = settings(
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow,
                           HealthCheck.function_scoped_fixture],
)


# ---------------------------------------------------------------------
# T-U-T1c-1: NaN in state.platform_pos_mm
# ---------------------------------------------------------------------

class TestNanInPlatformPos:
    """T-U-T1c-1 — adversarial NaN in any xyz axis of
    ``state.platform_pos_mm`` MUST route to fallback with
    ``Invalid_Number_Detected`` and preserve warm-start integrity."""

    @given(axis=_XYZ_AXIS)
    @_GIVEN_SETTINGS
    def test_nan_routes_to_fallback(self, plant, axis):
        mpc = _create_mpc(plant)
        _seed_mpc(plant, mpc)
        state = plant.get_state()
        # In-place write through the plant's alias.  The next
        # plant.get_state() call (in the next test or seed) overwrites
        # state.platform_pos_mm[i] with the correct sensor reading at
        # mujoco_plant.py's get_state body — so this mutation does NOT
        # persist across tests.
        state.platform_pos_mm[axis] = np.nan

        cmd, _, diag = mpc.solve(state, REF_NORMAL)

        assert _NAN_FAULT_KEYWORD in diag['status'], (
            f"NaN in platform_pos_mm[{axis}] should produce "
            f"{_NAN_FAULT_KEYWORD!r} in status; got {diag['status']!r}"
        )
        # Post-Tier-1-rewrite (2026-05-20): the IPOPT
        # Invalid_Number_Detected return code routes through
        # _handle_failure's primary linear cmd-stream extrapolation
        # arm (status prefix ``fallback_extrap(``).  Pre-rewrite this
        # routed through the walk-forward arm with prefix
        # ``fallback(``.  Accept either to keep the test stable across
        # the design change while ensuring the safety-routing intent
        # is preserved.
        assert (diag['status'].startswith('fallback_extrap(')
                or diag['status'].startswith('fallback(')), (
            f"NaN in platform_pos_mm should route through the "
            f"linear-extrap fallback arm (or legacy fallback(...)); "
            f"got {diag['status']!r}"
        )
        _assert_warm_start_integrity(mpc)
        _assert_safe_outcome(diag, cmd)
        assert mpc.consecutive_failures == 1


# ---------------------------------------------------------------------
# T-U-T1c-2: ±Inf in state.platform_rot
# ---------------------------------------------------------------------

class TestInfInPlatformRot:
    """T-U-T1c-2 — adversarial ±Inf in any axis of
    ``state.platform_rot`` (rotation vector, rad) MUST route to
    fallback with ``Invalid_Number_Detected``.

    Plan called for ``state.platform_twist`` originally; the empirical
    probe found ``platform_twist`` is a dead field in ``solve()`` (see
    ``test_platform_twist_is_dead`` below for the regression guard).
    ``platform_rot`` is the closest live twist-shaped field that flows
    into the optimization."""

    @given(axis=_ROT_AXIS, value=_NAN_OR_INF)
    @_GIVEN_SETTINGS
    def test_inf_or_nan_routes_to_fallback(self, plant, axis, value):
        mpc = _create_mpc(plant)
        _seed_mpc(plant, mpc)
        state = plant.get_state()
        # In-place write through the plant's alias — see TestNanInPlatformPos
        # for rationale.
        state.platform_rot[axis] = value

        cmd, _, diag = mpc.solve(state, REF_NORMAL)

        assert _NAN_FAULT_KEYWORD in diag['status'], (
            f"non-finite in platform_rot[{axis}] (value={value}) should "
            f"produce {_NAN_FAULT_KEYWORD!r} in status; got "
            f"{diag['status']!r}"
        )
        _assert_warm_start_integrity(mpc)
        _assert_safe_outcome(diag, cmd)


# ---------------------------------------------------------------------
# T-U-T1c-D1: state.platform_twist is dead under solve()
# ---------------------------------------------------------------------

class TestPlatformTwistIsDead:
    """T-U-T1c-D1 — documents the empirical finding that
    ``state.platform_twist`` is currently NOT consumed by ``solve()``.
    Acts as a regression guard: if a future refactor wires
    ``platform_twist`` into the optimization, this test will fail
    (loudly), prompting the maintainer to review the input-fuzz
    coverage of that new path.

    Why this is a separate test class: T-U-T1c-2 was originally
    targeted at ``platform_twist``; the dead-field finding came out of
    the Phase 3 empirical probe.  Splitting into the two classes
    preserves the planned T-U-T1c-2 coverage (now on ``platform_rot``)
    and adds the dead-field regression guard as T-U-T1c-D1."""

    def test_platform_twist_does_not_affect_solve(self, plant):
        """Two solves with identical state but adversarial vs. clean
        ``platform_twist`` MUST produce identical cmd + status."""
        # Build solve A: clean state.
        mpc_a = _create_mpc(plant)
        _seed_mpc(plant, mpc_a)
        state_a = plant.get_state()
        cmd_a, _, diag_a = mpc_a.solve(state_a, REF_NORMAL)

        # Build solve B: adversarial twist on otherwise-identical state.
        # Use a fresh plant so MuJoCo state is identical to A's seed.
        plant_b = MuJoCoPlant()
        mpc_b = _create_mpc(plant_b)
        _seed_mpc(plant_b, mpc_b)
        state_b = plant_b.get_state()
        state_b.platform_twist = np.array(
            [np.nan, np.inf, -np.inf, np.nan, np.inf, -np.inf]
        )
        cmd_b, _, diag_b = mpc_b.solve(state_b, REF_NORMAL)

        assert diag_a['status'] == diag_b['status'], (
            f"platform_twist is supposed to be dead in solve(); status "
            f"differs: clean={diag_a['status']!r}, "
            f"adversarial={diag_b['status']!r}.  If solve() now reads "
            f"platform_twist, refresh the dead-field finding in "
            f"logbook 2026-05-11-tier1c-input-fuzz.md and add a "
            f"NaN/Inf-in-twist test that exercises the new path."
        )
        np.testing.assert_allclose(cmd_a, cmd_b, atol=1e-12, err_msg=(
            "platform_twist is supposed to be dead in solve(); cmds "
            "differ between clean and adversarial twist — see this "
            "test's docstring for the follow-up checklist."
        ))


# ---------------------------------------------------------------------
# T-U-T1c-3: NaN in target_pose
# ---------------------------------------------------------------------

class TestNanInTargetPose:
    """T-U-T1c-3 — adversarial NaN in any pose axis of ``target_pose``
    MUST route to fallback with ``Invalid_Number_Detected``.  Note the
    pose axis range is 0–5 (xyz + rotation vector)."""

    @given(axis=st.integers(min_value=0, max_value=5))
    @_GIVEN_SETTINGS
    def test_nan_routes_to_fallback(self, plant, axis):
        mpc = _create_mpc(plant)
        _seed_mpc(plant, mpc)
        state = plant.get_state()
        bad_target = REF_NORMAL.copy()
        bad_target[axis] = np.nan

        cmd, _, diag = mpc.solve(state, bad_target)

        assert _NAN_FAULT_KEYWORD in diag['status']
        _assert_warm_start_integrity(mpc)
        _assert_safe_outcome(diag, cmd)


# ---------------------------------------------------------------------
# T-U-T1c-4: NaN in ref_events[i].twist
# ---------------------------------------------------------------------

class TestNanInRefEventsTwist:
    """T-U-T1c-4 — adversarial NaN in the ``twist`` field of any
    ``ReferenceEvent`` in the events list MUST route to fallback with
    ``Invalid_Number_Detected`` and preserve warm-start integrity.

    Driver: a 2-event list with a clean event[0] and an adversarial
    event[1] (NaN-spiked twist on one axis, chosen by hypothesis)."""

    @given(axis=st.integers(min_value=0, max_value=5))
    @_GIVEN_SETTINGS
    def test_nan_in_event_twist_routes_to_fallback(self, plant, axis):
        mpc = _create_mpc(plant)
        state = _seed_mpc(plant, mpc)
        bad_twist = np.zeros(6)
        bad_twist[axis] = np.nan
        events = [
            ReferenceEvent(time=state.time + 0.0,
                           pose=REF_NORMAL,
                           twist=np.zeros(6)),
            ReferenceEvent(time=state.time + 0.5,
                           pose=REF_NORMAL,
                           twist=bad_twist),
        ]

        cmd, _, diag = mpc.solve(state, REF_NORMAL, ref_events=events)

        assert _NAN_FAULT_KEYWORD in diag['status']
        _assert_warm_start_integrity(mpc)
        _assert_safe_outcome(diag, cmd)


# ---------------------------------------------------------------------
# T-U-T1c-5: extreme magnitudes (±1e10)
# ---------------------------------------------------------------------

# Parametrised scenarios — the four LIVE input fields that might
# plausibly carry extreme magnitudes.  ``state.platform_twist`` is
# excluded (dead field; the dead-field test covers it).  Tuple format:
# (label, mutator).  Each mutator takes (state, target) and returns
# (state, target) ready for ``solve()``.  Mutators perform in-place
# writes through the plant's alias for state fields (see TestNan*
# rationale); ``target`` is a fresh array so a copy-and-set is safe.
_EXTREME_MAGNITUDE_SCENARIOS = [
    ('target_pose[2]=+1e10',     lambda s, t: (s, _set_target(t, 2, +1e10))),
    ('target_pose[2]=-1e10',     lambda s, t: (s, _set_target(t, 2, -1e10))),
    ('platform_pos_mm[2]=+1e10', lambda s, t: (_inplace(s, 'platform_pos_mm', 2, +1e10), t)),
    ('platform_pos_mm[2]=-1e10', lambda s, t: (_inplace(s, 'platform_pos_mm', 2, -1e10), t)),
]


def _set_target(arr, idx, value):
    """Copy-and-set helper for ``target_pose`` (a caller-supplied
    array, not aliased to the plant — copy is safe and avoids
    polluting the test's REF_NORMAL constant)."""
    out = arr.copy()
    out[idx] = value
    return out


def _inplace(state, field, idx, value):
    """In-place write through the plant's alias on a state field.
    Overwritten on the next plant.get_state() — does NOT persist
    across tests."""
    getattr(state, field)[idx] = value
    return state


class TestExtremeMagnitudes:
    """T-U-T1c-5 — extreme magnitudes (±1e10 mm) at any live xyz field.

    Empirical finding: IPOPT handles these cleanly — the per-leg IK
    constraints clamp the working set well before ±1e10 propagates into
    the gradient.  Result: ``Solve_Succeeded`` and finite cmd.

    This test asserts the surprising-but-correct behaviour as a
    regression guard: a future refactor that introduces an unguarded
    ``np.exp`` / ``np.log`` / division on raw position values would
    produce overflow + NaN at ±1e10, regressing the property."""

    @pytest.mark.parametrize('label,mutator', _EXTREME_MAGNITUDE_SCENARIOS,
                              ids=[s[0] for s in _EXTREME_MAGNITUDE_SCENARIOS])
    def test_extreme_magnitude_succeeds(self, plant, label, mutator):
        mpc = _create_mpc(plant)
        _seed_mpc(plant, mpc)
        state = plant.get_state()
        target = REF_NORMAL.copy()
        state, target = mutator(state, target)

        cmd, _, diag = mpc.solve(state, target)

        # The empirical contract is "extreme magnitudes succeed"; if a
        # future change makes them fault, we want the failure to be
        # diagnosable rather than silently corrupting state.
        _assert_warm_start_integrity(mpc)
        _assert_safe_outcome(diag, cmd)
        # On the production stack at probe time, this status was
        # exactly Solve_Succeeded.  Asserting that explicitly so a
        # regression to fallback (e.g., new bound check that rejects
        # 1e10) surfaces immediately.
        assert diag['status'] == 'Solve_Succeeded', (
            f"extreme magnitude {label!r} should produce Solve_Succeeded "
            f"on the pinned stack; got {diag['status']!r}.  If a new "
            f"bound check legitimately rejects 1e10, refresh the probe "
            f"table in logbook 2026-05-11-tier1c-input-fuzz.md."
        )


# ---------------------------------------------------------------------
# T-U-T1c-6: wrong-shape inputs
# ---------------------------------------------------------------------

# Shape-error scenarios — the API boundary should raise (P3-level
# trusted-callee), not silently degrade.  Tuple format:
# (label, mutator, expected_substring_in_error).  ``expected_substring``
# is matched in the str(exception); use the most specific stable
# substring that survives numpy version bumps.  Mutators that REASSIGN
# state attributes (the (5,)/(2,) cases below) corrupt the plant's
# alias if the plant fixture is shared — this test class uses
# ``fresh_plant`` (function scope) for that reason.
_WRONG_SHAPE_SCENARIOS = [
    ('target_pose shape (5,)',
     lambda s, t: (s, np.zeros(5)),
     'shape (5,)'),
    ('target_pose shape (7,)',
     lambda s, t: (s, np.zeros(7)),
     'shape (7,)'),
    ('platform_pos_mm shape (2,)',
     lambda s, t: (_set_state_attr(s, 'platform_pos_mm', np.zeros(2)), t),
     'shape (2,)'),
    ('leg_extensions_mm shape (5,)',
     lambda s, t: (_set_state_attr(s, 'leg_extensions_mm', np.zeros(5)), t),
     'shape (5,)'),
]


def _set_state_attr(state, field, value):
    """REASSIGN a state attribute.  Breaks the plant alias — must use
    ``fresh_plant`` fixture (not the shared ``plant`` fixture)."""
    setattr(state, field, value)
    return state


class TestWrongShapeRaises:
    """T-U-T1c-6 — wrong-shape inputs at the API boundary MUST raise.

    Per the plan's critical-detail note: shape errors are P3-level
    "trusted-callee" boundary errors; ``solve()`` is allowed to detect
    them at the entry boundary rather than route them through the
    fallback machinery.  Currently the detection happens inside
    ``solve()``'s parameter-vector pack via ``numpy``'s broadcast
    error — this is *adequate* (the error is raised loudly with a
    clear shape-mismatch message), but if a future refactor moves
    parameter packing later in the call, the error might surface
    less cleanly.  This test pins the current detection surface.

    Uses the ``fresh_plant`` (function-scoped) fixture because two of
    the scenarios reassign ``state.platform_pos_mm`` /
    ``state.leg_extensions_mm`` to wrong-shape arrays — that breaks
    the P1 alias on a shared plant and corrupts subsequent
    ``plant.get_state()`` calls."""

    @pytest.mark.parametrize('label,mutator,expected_substring',
                              _WRONG_SHAPE_SCENARIOS,
                              ids=[s[0] for s in _WRONG_SHAPE_SCENARIOS])
    def test_wrong_shape_raises_value_error(self, fresh_plant, label, mutator,
                                              expected_substring):
        mpc = _create_mpc(fresh_plant)
        _seed_mpc(fresh_plant, mpc)
        state = fresh_plant.get_state()
        target = REF_NORMAL.copy()
        state, target = mutator(state, target)

        with pytest.raises(ValueError) as exc_info:
            mpc.solve(state, target)
        assert expected_substring in str(exc_info.value), (
            f"shape-error message should mention {expected_substring!r}; "
            f"got: {exc_info.value!s}"
        )


# ---------------------------------------------------------------------
# Recovery: after each documented fault, a clean solve must succeed.
# ---------------------------------------------------------------------

class TestRecoveryAfterFuzzFault:
    """Companion to the fuzz tests: every fault path MUST be transient.
    A NaN-induced fallback this tick MUST NOT prevent the next clean
    tick from converging.  Critical safety property: production faults
    are typically momentary (a single corrupted telemetry frame), so
    the controller must shrug off the fault and continue.

    Uses ``fresh_plant`` so the in-place NaN write below cannot leak
    into other tests via the module-shared plant alias."""

    def test_recovery_after_nan_in_platform_pos(self, fresh_plant):
        mpc = _create_mpc(fresh_plant)
        _seed_mpc(fresh_plant, mpc)

        # Fault tick — in-place write through the alias.
        state = fresh_plant.get_state()
        state.platform_pos_mm[1] = np.nan
        _, _, diag_fault = mpc.solve(state, REF_NORMAL)
        assert _NAN_FAULT_KEYWORD in diag_fault['status']
        assert mpc.consecutive_failures == 1

        # Recovery tick — plant.get_state() overwrites the NaN with
        # the live sensor reading, so clean_state.platform_pos_mm[1]
        # is finite again.
        fresh_plant.reset()
        clean_state = fresh_plant.get_state()
        assert np.all(np.isfinite(clean_state.platform_pos_mm))
        _, _, diag_recover = mpc.solve(clean_state, REF_NORMAL)
        assert diag_recover['status'] == 'Solve_Succeeded', (
            f"recovery solve after NaN fault must succeed; got "
            f"{diag_recover['status']!r}"
        )
        assert mpc.consecutive_failures == 0


# ---------------------------------------------------------------------
# T-U-T1c-7: warm-start integrity property — Hypothesis stateful
# ---------------------------------------------------------------------

# Module-scoped plant + MPC singletons reused across StateMachine
# instances.  Same singleton-management pattern as
# [test_solver_failures.py::WarmStartIntegrityMachine](test_solver_failures.py):
# capture the real solver ONCE at module init and unconditionally
# restore it in __init__ so a prior walk's state cannot bleed into
# data generation (would otherwise produce FlakyStrategyDefinition).
_T1C_PROP_PLANT: MuJoCoPlant | None = None
_T1C_PROP_MPC: MPCController | None = None
_T1C_PROP_REAL_SOLVER = None


def _t1c_get_singletons():
    global _T1C_PROP_PLANT, _T1C_PROP_MPC, _T1C_PROP_REAL_SOLVER
    if _T1C_PROP_PLANT is None:
        _T1C_PROP_PLANT = MuJoCoPlant()
        _T1C_PROP_MPC = _create_mpc(_T1C_PROP_PLANT)
        _T1C_PROP_REAL_SOLVER = _T1C_PROP_MPC._solver
    return _T1C_PROP_PLANT, _T1C_PROP_MPC, _T1C_PROP_REAL_SOLVER


# Strategies for the fuzz_solve rule.
#
# All four input fields are exercised after the
# 2026-05-11-tier1c-input-fuzz-bugfix landed the per-axis q_cur/q_dot
# sanitization at the top of ``_handle_failure``.  Pre-bugfix,
# ``leg_extensions_mm`` and ``leg_velocities_mmps`` were excluded
# because non-finite values in either propagated through the
# ``hold_extrap`` arm into ``_prev_u`` (the corruption cascade
# T-U-T1c-7 surfaced).  Post-bugfix, both are safe to fuzz: the
# sanitization substitutes per-axis from ``_prev_u`` (q_cur) or
# zero (q_dot), so the warm-start integrity invariant holds across
# the full input surface.
_T1C_FAULT_FIELD = st.sampled_from([
    ('platform_pos_mm', 3),       # 3 axes
    ('platform_rot', 3),          # 3 axes
    ('leg_extensions_mm', 6),     # 6 axes — sanitized at _handle_failure entry
    ('leg_velocities_mmps', 6),   # 6 axes — sanitized at _handle_failure entry
    ('target_pose', 6),           # 6 axes
])
_T1C_FAULT_VALUE = _NAN_OR_INF


class T1cWarmStartIntegrityMachine(RuleBasedStateMachine):
    """T-U-T1c-7 — warm-start integrity invariant under arbitrary
    sequences of (succeed, fuzz, advance_time).

    **Invariants checked after every rule:**

    * ``warm_start_finite_or_none`` — ``_prev_w`` is None or all-finite
      (the load-bearing safety property).
    * ``prev_u_finite_or_none`` — companion check for the rate-limit
      anchor.
    * ``snapshot_finite_or_none`` — W7 snapshot fields stay clean.
    * ``no_state_corruption`` — ``mpc._consecutive_failures`` is
      bounded above by the number of fuzz operations the walk has
      performed (catches counter overflow / negative resets).

    Plan choice rationale (see [Phase 2 logbook
    Discussion](../../logbook/2026-05-11-tier1b-fallback-escalation-cascade.md)):
    ``RuleBasedStateMachine`` over ``@composite`` because the
    invariant is across-call.  Pure ``@given`` would only test one
    adversarial input at a time; the warm-start corruption mode the
    invariant guards against requires a *sequence* (succeed → fuzz →
    succeed → fuzz → advance_time → fuzz → succeed) to expose."""

    def __init__(self):
        super().__init__()
        self._plant, self._mpc, real_solver = _t1c_get_singletons()
        # UNCONDITIONALLY restore the real solver before resetting (a
        # prior walk may have monkey-patched it; this protects against
        # the exact FlakyStrategyDefinition pitfall the Phase 2
        # logbook documents).  In Phase 3 the rules don't swap the
        # solver, so this is belt-and-braces, but the discipline keeps
        # future rules safe.
        self._mpc._solver = real_solver
        self._plant.reset()
        self._mpc.reset()
        self._n_fuzz_ops = 0
        # Seed one successful solve so prev_w / snapshot are populated.
        state = self._plant.get_state()
        _, _, diag = self._mpc.solve(state, REF_NORMAL)
        assert diag['status'] == 'Solve_Succeeded'

    # ---- Rules ----

    @rule()
    def succeed_solve(self):
        """Run one clean solve at REF_NORMAL.  Restores _prev_w to a
        fresh post-success snapshot."""
        self._plant.reset()
        state = self._plant.get_state()
        _, _, diag = self._mpc.solve(state, REF_NORMAL)
        # The seed succeeded so this should too — but if it doesn't
        # (e.g., a prior fuzz left the plant in an unexpected state),
        # we still want the invariants to hold.  Don't assert on
        # status here.

    @rule(field_spec=_T1C_FAULT_FIELD,
          axis_offset=st.integers(min_value=0, max_value=5),
          value=_T1C_FAULT_VALUE)
    def fuzz_solve(self, field_spec, axis_offset, value):
        """Inject one adversarial value into the chosen field/axis,
        then call solve().  ``axis_offset`` is modulo'd against the
        field's actual axis count (3 for xyz/rot, 6 for leg/pose), so
        the strategy is dense across all valid axes regardless of
        which field is selected."""
        field, n_axes = field_spec
        axis = axis_offset % n_axes
        self._plant.reset()
        state = self._plant.get_state()
        target = REF_NORMAL.copy()
        if field == 'target_pose':
            target = target.copy()
            target[axis] = value
        else:
            # In-place write through the plant alias — matches the
            # alias-break discipline documented in the ``plant`` /
            # ``fresh_plant`` fixture docstrings and in the Phase 3
            # logbook (Test infrastructure — alias-break pitfall).
            # The next plant.get_state() call (in succeed_solve,
            # fuzz_solve, or __init__) overwrites state.<field>[axis]
            # with the live sensor reading, so the NaN/Inf does not
            # persist across rules.
            getattr(state, field)[axis] = value
        try:
            self._mpc.solve(state, target)
        except ValueError:
            # Wrong-shape paths can also surface here if a future
            # field-spec extension adds shape-mutation axes; tolerate
            # them — the invariants still apply.
            pass
        self._n_fuzz_ops += 1

    @rule(t_advance=st.floats(min_value=0.0, max_value=1.5,
                               allow_nan=False, allow_infinity=False))
    def advance_time(self, t_advance):
        """Simulate wall-clock advance (W7 staleness probe).  Reuses
        the Phase 2 stateful machine's pattern; combined with fuzz
        rules this exercises the (fault → stale snapshot → next fault)
        sequence that could plausibly leave _prev_w in an
        inconsistent state if the snapshot-update logic and the
        fault-handling logic disagree."""
        if self._mpc._t_at_last_success is None:
            return
        self._mpc._t_at_last_success -= t_advance

    # ---- Invariants ----

    @invariant()
    def warm_start_finite_or_none(self):
        w = self._mpc._prev_w
        assert w is None or np.all(np.isfinite(w)), (
            f"T-U-T1c-7 invariant violation: _prev_w contains "
            f"non-finite values: {w!r}"
        )

    @invariant()
    def prev_u_finite_or_none(self):
        u = self._mpc._prev_u
        assert u is None or np.all(np.isfinite(u)), (
            f"_prev_u non-finite: {u!r}"
        )

    @invariant()
    def snapshot_finite_or_none(self):
        for name in ('_ref_at_last_success_mid', '_twist_at_last_success'):
            v = getattr(self._mpc, name)
            assert v is None or np.all(np.isfinite(v)), (
                f"W7 snapshot {name} non-finite: {v!r}"
            )

    @invariant()
    def consecutive_failures_bounded(self):
        # Counter cannot exceed the number of fuzz operations the walk
        # has performed (succeed_solve is *expected* to reset the
        # counter; fuzz_solve may increment; advance_time only mutates
        # the staleness clock).  Negative would indicate corruption.
        #
        # Strictly, this bound assumes every succeed_solve call
        # actually converges — if a succeed_solve returned a non-
        # success status, _consecutive_failures would increment
        # without _n_fuzz_ops incrementing, and two such failures
        # back-to-back would trip this assertion for reasons unrelated
        # to the warm-start property under test.  Empirically the
        # current strategy space never produces that case (verified at
        # ci-deep, 1000 examples, 363.68 s).  If a future rule
        # extension widens succeed_solve's surface (e.g.
        # parameterising the seed ref) and this invariant starts
        # firing spuriously, the right fix is either to widen the
        # bound or to count succeed_solve invocations explicitly —
        # NOT to weaken the assertion.
        c = self._mpc.consecutive_failures
        assert 0 <= c <= self._n_fuzz_ops + 1, (
            f"_consecutive_failures={c} out of bounds [0, "
            f"{self._n_fuzz_ops + 1}]"
        )


TestT1cWarmStartIntegrity = T1cWarmStartIntegrityMachine.TestCase


# ---------------------------------------------------------------------
# T-U-T1c-7-bug: regression pin for the NaN-in-leg_extensions_mm +
# stale-snapshot → _prev_u corruption path.  Bug fixed in
# 2026-05-11-tier1c-input-fuzz-bugfix; this test asserts the fix holds.
# ---------------------------------------------------------------------

class TestT1cLegExtNanCorruptsPrevU:
    """T-U-T1c-7-bug — regression-pin for the
    NaN-in-leg_extensions_mm + stale-snapshot corruption path that
    T-U-T1c-7's hypothesis stateful machine surfaced.

    History.  Phase 3 (commit 7582764) added this test marked
    ``xfail(strict=True)`` because the corruption was a real
    production bug:  q_cur (= ``state.leg_extensions_mm.copy()``)
    flowed through the (then-extant) ``hold_extrap`` arm of
    ``_handle_failure`` via ``cmd = np.clip(q_cur + q_dot * dt, ...)``,
    propagating NaN into ``self._prev_u`` and thereby corrupting the
    warm-start.

    Fix (separate commit, see logbook
    2026-05-11-tier1c-input-fuzz-bugfix.md): ``_handle_failure`` now
    sanitizes non-finite ``q_cur`` / ``q_dot`` axes at function entry
    — per-axis substitution from ``_prev_u`` (or stroke margin if
    ``_prev_u`` is None) for ``q_cur``; per-axis substitution with
    ``0.0`` for ``q_dot``.  The Tier-1 fallback rewrite of 2026-05-20
    (logbook
    ``2026-05-20-hold-extrap-positive-feedback-chaotic-motion.md``)
    further removed ``q_dot`` from the cmd RHS entirely, so the
    propagation path that originally created this bug no longer exists
    — but the entry-point sanitization (and this regression pin) are
    retained because q_cur still flows through the cold_hold arm.

    Reproducer recipe:
      1. Seed a successful solve (populates ``_prev_u``, ``_prev_w``,
         success-time snapshot fields).
      2. Force the staleness branch (now: cascade_too_long →
         cold_hold) by aging ``_t_at_last_success`` past 500 ms.
      3. Set ``state.leg_extensions_mm[0] = NaN``.
      4. Solve.
      5. Assert ``_prev_u`` is finite (sanitization substituted
         ``_prev_u[0]`` for the NaN axis before the cold_hold arm
         constructed cmd).
    """

    def test_nan_leg_ext_plus_stale_snapshot_keeps_prev_u_finite(self,
                                                                   fresh_plant):
        mpc = _create_mpc(fresh_plant)
        _seed_mpc(fresh_plant, mpc)
        prev_u_pre = mpc._prev_u.copy()
        # Force walk_forward_unsafe via the staleness branch.
        mpc._t_at_last_success -= 0.6
        # Inject NaN through the alias.
        state = fresh_plant.get_state()
        state.leg_extensions_mm[0] = np.nan

        cmd, _, diag = mpc.solve(state, REF_NORMAL)

        assert mpc._prev_u is not None and np.all(np.isfinite(mpc._prev_u)), (
            f"REGRESSION: _prev_u contains non-finite values after a "
            f"NaN-in-leg_extensions_mm fault on the cold_hold path: "
            f"{mpc._prev_u!r}.  The _handle_failure entry sanitization "
            f"that was added in 2026-05-11-tier1c-input-fuzz-bugfix has "
            f"regressed."
        )
        assert np.all(np.isfinite(cmd)), (
            f"cmd MUST be finite after the sanitization fix; got {cmd!r}"
        )
        # The sanitization should substitute prev_u[0] for the NaN axis,
        # so cmd[0] should be near prev_u[0] + q_dot[0]*dt (q_dot is the
        # plant's clean leg_velocities, ≈ 0 in steady state) — i.e.
        # close to prev_u[0].
        assert abs(cmd[0] - prev_u_pre[0]) < 1.0, (
            f"After sanitization, cmd[0] should be ≈ prev_u[0] (pre-fault "
            f"= {prev_u_pre[0]:.3f}); got cmd[0]={cmd[0]:.3f}.  This may "
            f"indicate the substitution policy changed away from "
            f"per-axis _prev_u."
        )


# =====================================================================
# Plan 2 Phase 7 (Tier 3a-1) — Numerical fuzz on non-`solve()` API surfaces
# =====================================================================
#
# Coverage of the public API surfaces that Phase 3 (T-U-T1c-*) did NOT
# touch.  Each test drives the production function with adversarial
# NaN / Inf / zero-duration inputs and asserts the documented
# behaviour — "returns sentinel value" OR "raises" OR "propagates
# the non-finite input".  No test attempts to fix downstream behaviour
# here; assertions pin what the code DOES today (with one xfail for
# T-U-T3a-N2 pending a same-session guard-fix).
#
# Empirical findings (probe table, 2026-05-11 on this Jetson stack):
#
# | ID         | Surface                                       | Adversarial input         | Observed behaviour                                        |
# |------------|-----------------------------------------------|---------------------------|-----------------------------------------------------------|
# | T-U-T3a-N1 | feasibility.segment_is_feasible              | NaN/Inf in any of 6 args  | Raises LinAlgError from np.roots in quintic peak-vel       |
# | T-U-T3a-N2 | feasibility.quintic_peak_vel_per_axis        | T=0 / T<0                 | Pre-fix: NaN array (T=0) or nonsense (T<0)                 |
# | T-U-T3a-N3 | hermite.quintic_interp_with_accel            | duration<=0; NaN/Inf inputs| duration<=0 → returns p1/v1/a1; NaN/Inf propagate          |
# | T-U-T3a-N4 | target.flat_target_to_events                 | arrival_time < t_now+0.05 | Returns single hold event per :204–209                     |
# | T-U-T3a-N5 | target.make_feasible_events                  | NaN in proposal pose      | Raises LinAlgError (same downstream as N1)                 |
# | T-U-T3a-N6 | mpc._numerical_ik                            | angle < 1e-10             | Returns identity rotation per :1336–1338                   |
# | T-U-T3a-N7 | runner.run_mpc_loop                          | duration=0 → n_steps=0    | Returns False, 0 records logged                            |
#
# See logbook 2026-05-11-tier3a-numerical-schema-fuzz.md.
# =====================================================================


# Flag — flipped to True by the companion bugfix commit (logbook
# 2026-05-12-tier3a-fuzz-bugfix.md) which adds the T<=0 guard to
# quintic_peak_vel_per_axis AND lifts the iter_count/fallback_step
# schema unification (controller/DIAG_SCHEMA_CONTRACT.md).  Used by
# T-U-T3a-N2 here AND by several tests in test_diag_schema_fuzz.py.
_PHASE_7_BUGFIX_LANDED = True


# ---------------------------------------------------------------------
# T-U-T3a-N1 — feasibility.segment_is_feasible NaN/Inf inputs
# ---------------------------------------------------------------------

class TestT3aN1SegmentIsFeasibleNanInf:
    """T-U-T3a-N1 — ``feasibility.segment_is_feasible`` with NaN/Inf
    in any of the six (p0, v0, a0, p1, v1, a1) inputs.

    Production surface: ``segment_is_feasible`` delegates to
    ``quintic_peak_vel_per_axis`` → ``_roots_in_unit_interval`` →
    ``np.roots`` → ``np.linalg.eigvals``, which calls
    ``_assert_finite`` on input and raises ``LinAlgError`` if any
    coefficient is non-finite.

    Pass criterion (per plan): "Returns ``(False, inf, inf)`` or
    raises; never returns ``(True, ...)`` for NaN inputs".  Empirical:
    raises ``LinAlgError`` for every non-finite injection.
    """

    @pytest.mark.parametrize('arg_idx', list(range(6)))
    @pytest.mark.parametrize('axis', list(range(6)))
    @pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf')])
    def test_t3a_n1_nan_inf_raises_or_returns_infeasible(
        self, arg_idx, axis, value,
    ):
        """Every (arg, axis, value) combination either raises or
        returns ``(False, ...)``.
        """
        from controller.feasibility import segment_is_feasible

        args = [np.zeros(6, dtype=float) for _ in range(6)]
        # Make p1 distinct so the baseline is non-degenerate
        args[3] = np.array([0, 0, 50.0, 0, 0, 0])
        args[arg_idx] = args[arg_idx].copy()
        args[arg_idx][axis] = value

        try:
            feasible, vr, ar = segment_is_feasible(
                *args, T=0.5, v_max_mmps=140.0, a_max_mmps2=300.0,
            )
            assert not feasible, (
                f'arg{arg_idx}[{axis}]={value}: returned feasible=True '
                f'(vr={vr}, ar={ar}) — must never return True for non-finite '
                f'inputs'
            )
        except (np.linalg.LinAlgError, ValueError) as exc:
            # Acceptable — math layer correctly refused the non-finite input.
            _ = exc


# ---------------------------------------------------------------------
# T-U-T3a-N2 — feasibility.quintic_peak_vel_per_axis T<=0
# ---------------------------------------------------------------------

class TestT3aN2PeakVelZeroDuration:
    """T-U-T3a-N2 — ``feasibility.quintic_peak_vel_per_axis(..., T)``
    with ``T <= 0``.

    Pre-fix (this commit): no ``T<=0`` guard.  ``T=0`` produces a
    NaN array (division by zero inside ``_evaluate_poly(...) / T``);
    ``T<0`` produces a finite-but-nonsense array (the per-axis math
    treats T as a denominator without sign-handling).

    Post-fix: ``ValueError`` raised when ``T <= 0`` (the math is
    undefined; callers MUST pre-validate).  See companion bugfix
    commit (logbook 2026-05-12-tier3a-fuzz-bugfix.md).
    """

    @pytest.mark.xfail(
        condition=not _PHASE_7_BUGFIX_LANDED,
        strict=True,
        reason=(
            'Pre-fix: quintic_peak_vel_per_axis has no T<=0 guard. '
            'Test asserts the post-fix ValueError; will pass when '
            'the bugfix commit adds the guard.'
        ),
    )
    @pytest.mark.parametrize('T', [0.0, -0.5, -1e-12])
    def test_t3a_n2_t_le_zero_raises(self, T):
        """``T <= 0`` raises ``ValueError`` naming the parameter."""
        from controller.feasibility import quintic_peak_vel_per_axis

        p0 = np.zeros(6); v0 = np.zeros(6); a0 = np.zeros(6)
        p1 = np.array([0, 0, 50.0, 0, 0, 0]); v1 = np.zeros(6); a1 = np.zeros(6)

        with pytest.raises(ValueError, match=r'\bT\b'):
            quintic_peak_vel_per_axis(p0, v0, a0, p1, v1, a1, T)


# ---------------------------------------------------------------------
# T-U-T3a-N3 — hermite.quintic_interp_with_accel
# ---------------------------------------------------------------------

class TestT3aN3HermiteInterpBoundary:
    """T-U-T3a-N3 — ``hermite.quintic_interp_with_accel`` with
    ``duration<=0``, ``t`` out of range, and non-finite inputs.

    Documented behaviour (production code):

    * ``duration <= 0`` (guard at :109) → returns ``(p1.copy(),
      v1.copy(), a1.copy())``.
    * ``t > duration`` (clamp at :112) → returns the endpoint values
      at ``s=1``.
    * ``t < 0`` (clamp at :112) → returns the start values at ``s=0``.
    * NaN/Inf in any input → propagates through arithmetic (no guard).
      This is by-design: the function is a math primitive; input
      validation is the caller's responsibility.
    """

    def test_t3a_n3a_duration_zero_returns_endpoint(self):
        """``duration=0`` returns ``(p1, v1, a1)`` exactly."""
        from controller.hermite import quintic_interp_with_accel

        p0 = np.zeros(6); v0 = np.zeros(6); a0 = np.zeros(6)
        p1 = np.array([0, 0, 50.0, 0, 0, 0])
        v1 = np.array([1, 2, 3.0, 0, 0, 0])
        a1 = np.array([4, 5, 6.0, 0, 0, 0])

        pose, twist, accel = quintic_interp_with_accel(
            p0, v0, a0, p1, v1, a1, duration=0.0, t=0.0,
        )
        np.testing.assert_array_equal(pose, p1)
        np.testing.assert_array_equal(twist, v1)
        np.testing.assert_array_equal(accel, a1)

    def test_t3a_n3b_duration_negative_returns_endpoint(self):
        """``duration<0`` also enters the ``<=0`` guard."""
        from controller.hermite import quintic_interp_with_accel

        p0 = np.zeros(6); v0 = np.zeros(6); a0 = np.zeros(6)
        p1 = np.array([0, 0, 50.0, 0, 0, 0])
        v1 = np.zeros(6); a1 = np.zeros(6)

        pose, twist, accel = quintic_interp_with_accel(
            p0, v0, a0, p1, v1, a1, duration=-0.1, t=0.0,
        )
        np.testing.assert_array_equal(pose, p1)

    def test_t3a_n3c_t_clamped_above_duration(self):
        """``t > duration`` clamps to ``s=1`` (terminal pose)."""
        from controller.hermite import quintic_interp_with_accel

        p0 = np.zeros(6); v0 = np.zeros(6); a0 = np.zeros(6)
        p1 = np.array([0, 0, 50.0, 0, 0, 0])
        v1 = np.zeros(6); a1 = np.zeros(6)

        pose, _, _ = quintic_interp_with_accel(
            p0, v0, a0, p1, v1, a1, duration=0.5, t=1.0,  # 2× duration
        )
        np.testing.assert_allclose(pose, p1, atol=1e-9)

    def test_t3a_n3d_finite_input_finite_output(self):
        """Finite inputs produce finite output across the interior."""
        from controller.hermite import quintic_interp_with_accel

        p0 = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        v0 = np.array([10.0, -5.0, 0.0, 0.01, 0.0, -0.02])
        a0 = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        p1 = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        v1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        a1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        for t in (0.0, 0.1, 0.25, 0.4, 0.5):
            pose, twist, accel = quintic_interp_with_accel(
                p0, v0, a0, p1, v1, a1, duration=0.5, t=t,
            )
            assert np.all(np.isfinite(pose)), f't={t} pose: {pose}'
            assert np.all(np.isfinite(twist)), f't={t} twist: {twist}'
            assert np.all(np.isfinite(accel)), f't={t} accel: {accel}'

    def test_t3a_n3e_nan_input_propagates(self):
        """NaN inputs propagate (no implicit guard at this layer)."""
        from controller.hermite import quintic_interp_with_accel

        p0 = np.zeros(6); p0[2] = np.nan
        v0 = np.zeros(6); a0 = np.zeros(6)
        p1 = np.zeros(6); v1 = np.zeros(6); a1 = np.zeros(6)

        pose, _, _ = quintic_interp_with_accel(
            p0, v0, a0, p1, v1, a1, duration=0.5, t=0.25,
        )
        # Documented: NaN propagates.  Caller is responsible for guarding.
        assert np.isnan(pose[2])


# ---------------------------------------------------------------------
# T-U-T3a-N4 — target.flat_target_to_events degenerate arrival_time
# ---------------------------------------------------------------------

class TestT3aN4FlatTargetDegenerate:
    """T-U-T3a-N4 — ``target.flat_target_to_events`` with
    ``arrival_time`` so close to ``t_now`` that the quintic would be
    degenerate.

    Production guard at ``controller/target.py:204-209``:
    when ``t_arrival - t_now < 0.05``, returns a single ``ReferenceEvent``
    at the terminal pose (the "hold at target" policy).

    The threshold is 0.05 s (50 ms).  Below threshold → single hold
    event; AT or above threshold → two events (start + arrival).
    Note: the threshold is checked AFTER the ``arrival_time > t_now``
    test in the dist-based fallback at :196-200, so passing
    ``arrival_time == t_now`` does NOT enter the single-hold branch —
    it falls through to the dist-based duration.
    """

    def test_t3a_n4a_arrival_below_threshold_single_event(self):
        """``arrival_time = t_now + 0.01 s`` (< 0.05 s) → single hold event."""
        from controller.target import flat_target_to_events

        cur = np.array([0, 0, 50, 0, 0, 0], dtype=float)
        cur_tw = np.zeros(6)
        tgt = np.array([0, 0, 100, 0, 0, 0], dtype=float)

        events = flat_target_to_events(
            cur, cur_tw, tgt, t_now=10.0, arrival_time=10.01,
            v_max_mmps=140.0, tau_s=0.04,
        )
        assert len(events) == 1, f'expected 1 event, got {len(events)}'
        np.testing.assert_array_equal(events[0].pose, tgt)
        assert events[0].time == 10.0

    def test_t3a_n4b_arrival_above_threshold_two_events(self):
        """``arrival_time = t_now + 0.1 s`` (> 0.05 s) → two events."""
        from controller.target import flat_target_to_events

        cur = np.array([0, 0, 50, 0, 0, 0], dtype=float)
        cur_tw = np.zeros(6)
        tgt = np.array([0, 0, 100, 0, 0, 0], dtype=float)

        events = flat_target_to_events(
            cur, cur_tw, tgt, t_now=10.0, arrival_time=10.1,
            v_max_mmps=140.0, tau_s=0.04,
        )
        assert len(events) == 2, f'expected 2 events, got {len(events)}'
        # Event 0: start
        np.testing.assert_array_equal(events[0].pose, cur)
        # Event 1: arrival
        np.testing.assert_array_equal(events[1].pose, tgt)

    def test_t3a_n4c_arrival_equal_t_now_falls_to_dist_based(self):
        """``arrival_time == t_now`` falls through to dist-based duration.

        The production guard at :196 (``arrival_time > t_now``) requires
        strict greater-than.  An arrival exactly at t_now does NOT take
        the single-hold branch; instead, the dist-based duration kicks
        in (max(response_time, dist/cruise_speed)).
        """
        from controller.target import flat_target_to_events

        cur = np.array([0, 0, 50, 0, 0, 0], dtype=float)
        cur_tw = np.zeros(6)
        tgt = np.array([0, 0, 100, 0, 0, 0], dtype=float)

        events = flat_target_to_events(
            cur, cur_tw, tgt, t_now=10.0, arrival_time=10.0,
            v_max_mmps=140.0, tau_s=0.04,
        )
        # 2 events expected (dist-based duration ≈ 0.625s for 50 mm at 80 mm/s)
        assert len(events) == 2


# ---------------------------------------------------------------------
# T-U-T3a-N5 — target.make_feasible_events NaN pose
# ---------------------------------------------------------------------

class TestT3aN5MakeFeasibleEventsNan:
    """T-U-T3a-N5 — ``target.make_feasible_events`` with NaN in
    proposal pose.

    Production surface: ``make_feasible_events`` invokes
    ``segment_is_feasible`` per segment which delegates to
    ``np.roots`` → ``np.linalg.eigvals``.  ``LinAlgError`` raises on
    any NaN/Inf input.

    Pass criterion (per plan): "Either raises with clear error or
    routes through K1 anchor cleanly".  Empirical: raises
    ``LinAlgError`` (same as N1).
    """

    def test_t3a_n5a_nan_start_pose_raises(self):
        """NaN in proposal[0].pose raises ``LinAlgError``."""
        from controller.target import make_feasible_events, ReferenceEvent

        cur_nan = np.array([0, 0, np.nan, 0, 0, 0], dtype=float)
        tgt = np.array([0, 0, 100, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=10.0, pose=cur_nan.copy(),
                           twist=np.zeros(6), accel=np.zeros(6)),
            ReferenceEvent(time=10.5, pose=tgt.copy(),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        with pytest.raises((np.linalg.LinAlgError, ValueError)):
            make_feasible_events(
                cur_nan, np.zeros(6), proposal, t_now=10.0,
                v_max_mmps=140.0, tau_s=0.04,
            )

    def test_t3a_n5b_nan_target_pose_raises(self):
        """NaN in proposal[1].pose also raises."""
        from controller.target import make_feasible_events, ReferenceEvent

        cur = np.array([0, 0, 50, 0, 0, 0], dtype=float)
        tgt_nan = np.array([0, 0, np.nan, 0, 0, 0], dtype=float)
        proposal = [
            ReferenceEvent(time=10.0, pose=cur.copy(),
                           twist=np.zeros(6), accel=np.zeros(6)),
            ReferenceEvent(time=10.5, pose=tgt_nan.copy(),
                           twist=np.zeros(6), accel=np.zeros(6)),
        ]
        with pytest.raises((np.linalg.LinAlgError, ValueError)):
            make_feasible_events(
                cur, np.zeros(6), proposal, t_now=10.0,
                v_max_mmps=140.0, tau_s=0.04,
            )


# ---------------------------------------------------------------------
# T-U-T3a-N6 — mpc._numerical_ik zero-rotation
# ---------------------------------------------------------------------

class TestT3aN6NumericalIkZeroRotation:
    """T-U-T3a-N6 — ``MPCController._numerical_ik`` at the
    zero-rotation edge case.

    Production code at ``controller/mpc.py:1336-1338``:

        angle = np.linalg.norm(rv)
        if angle < 1e-10:
            R = np.eye(3)

    The guard prevents division-by-zero in the Rodrigues formula
    (``sin(angle)/angle``, ``(1 - cos(angle))/(angle*angle)``).  Below
    threshold, R defaults to identity — equivalent to "rotation is
    too small to matter".

    Pass criterion (per plan): "Returns identity rotation per
    documented fallback".  Verifies the threshold and the identity-R
    behaviour via the extension output.
    """

    def test_t3a_n6a_exact_zero_rotation(self, plant):
        """Pose with rv=[0,0,0]: identity rotation; extensions finite."""
        mpc = _create_mpc(plant)
        pose = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        exts = mpc._numerical_ik(pose)
        assert np.all(np.isfinite(exts))

    def test_t3a_n6b_below_threshold_matches_zero(self, plant):
        """rv with norm < 1e-10 produces identical extensions to rv=[0,0,0]."""
        mpc = _create_mpc(plant)
        pose_zero = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        pose_tiny = np.array([0.0, 0.0, 50.0, 1e-11, 0.0, 0.0])

        exts_zero = mpc._numerical_ik(pose_zero)
        exts_tiny = mpc._numerical_ik(pose_tiny)
        # Below-threshold rv enters the angle<1e-10 branch → R = I.
        # Above-threshold rv (e.g. 1e-9) enters the Rodrigues branch.
        # At norm=1e-11, the difference SHOULD be at floating-point noise.
        np.testing.assert_array_equal(exts_zero, exts_tiny)

    def test_t3a_n6c_above_threshold_differs(self, plant):
        """rv with norm >> 1e-10 produces different extensions."""
        mpc = _create_mpc(plant)
        pose_zero = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        pose_finite = np.array([0.0, 0.0, 50.0, 0.1, 0.0, 0.0])
        exts_zero = mpc._numerical_ik(pose_zero)
        exts_finite = mpc._numerical_ik(pose_finite)
        # A 0.1-rad rotation should produce a measurable extension change.
        assert not np.allclose(exts_zero, exts_finite, atol=1e-6)


# ---------------------------------------------------------------------
# T-U-T3a-N7 — runner.run_mpc_loop with duration=0
# ---------------------------------------------------------------------

class TestT3aN7RunMpcLoopZeroDuration:
    """T-U-T3a-N7 — ``runner.run_mpc_loop`` with ``duration=0`` →
    ``n_steps = int(0/control_dt) = 0`` → empty loop.

    Production code at ``controller/runner.py:300``:
    ``n_steps = int(duration / control_dt)``.  For ``duration=0`` the
    main ``for _step_idx in range(n_steps):`` loop body never
    executes.  The function returns ``estop_exit`` (default False)
    and the telemetry logger contains zero records.

    Pass criterion (per plan): "Either no-op return or raises;
    documented".  Empirical: no-op return; ``estop_exit=False``;
    zero records logged.
    """

    def test_t3a_n7a_duration_zero_returns_no_op(self, plant):
        """``duration=0`` → returns False; zero log records."""
        from controller.runner import run_mpc_loop
        from controller.target import StaticTargetSource
        from controller.telemetry import TelemetryLogger

        mpc = _create_mpc(plant)
        source = StaticTargetSource(
            [(0.0, REF_NORMAL.copy())],
            v_max_mmps=140.0, tau_s=0.04,
        )
        tlog = TelemetryLogger(pool_size=10)

        plant.reset()
        plant.step(0.025)
        estop_exit = run_mpc_loop(plant, mpc, source,
                                  duration=0.0, logger=tlog)
        assert estop_exit is False, f'estop_exit was {estop_exit!r}'
        assert len(tlog.records) == 0, (
            f'expected 0 records, got {len(tlog.records)}'
        )

    def test_t3a_n7b_duration_negative_also_no_op(self, plant):
        """``duration<0`` → ``int(neg/0.025) = 0`` → same no-op path."""
        from controller.runner import run_mpc_loop
        from controller.target import StaticTargetSource
        from controller.telemetry import TelemetryLogger

        mpc = _create_mpc(plant)
        source = StaticTargetSource(
            [(0.0, REF_NORMAL.copy())],
            v_max_mmps=140.0, tau_s=0.04,
        )
        tlog = TelemetryLogger(pool_size=10)

        plant.reset()
        plant.step(0.025)
        estop_exit = run_mpc_loop(plant, mpc, source,
                                  duration=-0.5, logger=tlog)
        assert estop_exit is False
        assert len(tlog.records) == 0
