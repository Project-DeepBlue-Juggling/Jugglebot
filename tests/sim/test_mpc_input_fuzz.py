"""Plan 2 Phase 3 (Tier 1c) — NaN/Inf input fuzz on ``MPCController.solve()``.

This module hammers the entry-point input surface of ``solve()`` with
adversarial values (NaN, ±Inf, extreme magnitudes, wrong shapes) and
asserts two safety properties hold for every adversarial input:

1. **Warm-start integrity** — ``_prev_w`` is either ``None`` (cleared by
   the exception handler) or contains only finite values.  This is the
   load-bearing safety property: a corrupted warm-start would cascade
   into the next solve and command non-finite extensions to the
   actuators.
2. **Status-routing finiteness** — every status string falls into one of
   five documented buckets (``Solve_Succeeded`` / ``fallback(...)`` /
   ``hold(...)`` / ``hold_extrap(...)`` / ``cold_hold(...)``); the
   returned ``cmd`` is always finite.

Test IDs from
[plans/active/mpc-sadpath-coverage-tiers-1-3.md](../../plans/active/mpc-sadpath-coverage-tiers-1-3.md)
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
| Inf in ``target_pose[2]``                  | ``Invalid_Number_Detected``           | ``hold_extrap(Invalid_Number_Detected)`` *     |
| NaN in ``ref_events[i].twist``             | ``Invalid_Number_Detected``           | ``fallback(Invalid_Number_Detected)``          |
| ±1e10 mm in ``target_pose[i]``             | ``Solve_Succeeded``                    | ``Solve_Succeeded``                            |
| ±1e10 mm in ``state.platform_pos_mm[i]``   | ``Solve_Succeeded``                    | ``Solve_Succeeded``                            |
| Wrong-shape ``target_pose`` (n!=6)         | not reached                           | ``ValueError`` from numpy broadcast at boundary |
| NaN in ``state.leg_velocities_mmps``       | ``Solve_Succeeded`` (q_dot is failure-only) | ``Solve_Succeeded``                       |
| NaN in ``state.platform_twist``            | dead field — solve() never reads it    | ``Solve_Succeeded`` (identical cmd to clean)   |

\\* Inf in ``target_pose[2]`` produces a finite-vs-infinite mid-node ref
delta that trips the W7 walk-forward-unsafe ref-shift branch covered by
Phase 2 — the resulting status is ``hold_extrap(...)``, not
``fallback(...)``.  This is correct cascade behaviour but worth flagging
because the surface (target_pose) and the symptom (Inf) are both Phase 3
inputs, while the path is Phase 2 escalation.

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

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from controller.target import ReferenceEvent


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
_FALLBACK_PREFIXES = ('fallback(', 'hold_extrap(', 'hold(', 'cold_hold(')


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
        assert diag['status'].startswith('fallback('), (
            f"NaN in platform_pos_mm should route through walk-forward "
            f"fallback (the snapshot is fresh from the seed solve); "
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
# [test_solver_failures.py::W7WarmStartIntegrityMachine](test_solver_failures.py):
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
# ``leg_extensions_mm`` is DELIBERATELY excluded — it surfaces a real
# production bug (see ``TestT1cLegExtNanCorruptsPrevU`` below): when
# combined with a stale W7 snapshot, NaN in ``state.leg_extensions_mm``
# propagates through the ``hold_extrap`` arm into ``_prev_u``, leaving
# the warm-start in a non-finite state.  The bug is demonstrated by an
# xfail-strict scenario test below; once it's fixed in a follow-up
# commit, this strategy MUST be widened to re-include
# ``leg_extensions_mm`` and the xfail marker MUST be removed in the
# same commit as the fix.
#
# ``state.leg_velocities_mmps`` is ALSO excluded for a related reason:
# NaN in it doesn't cause a solver failure on its own (q_dot is read
# only on the failure path), but combined with a forced fallback +
# stale snapshot it would produce the same hold_extrap corruption via
# ``cmd = q_cur + q_dot * dt``.  Same bug class; same xfail covers it.
_T1C_FAULT_FIELD = st.sampled_from([
    ('platform_pos_mm', 3),  # 3 axes
    ('platform_rot', 3),
    ('target_pose', 6),
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
# T-U-T1c-7-bug: NaN in leg_extensions_mm + stale snapshot → _prev_u
# corruption.  Real production bug surfaced by Phase 3's stateful
# property; xfail-strict pending the fix.
# ---------------------------------------------------------------------

@pytest.mark.xfail(
    strict=True,
    reason=(
        "Real production bug surfaced by T-U-T1c-7's stateful "
        "property.  NaN in state.leg_extensions_mm[i], combined with "
        "walk_forward_unsafe=True (here forced via stale W7 snapshot), "
        "routes through the hold_extrap arm of _handle_failure where "
        "``cmd = np.clip(q_cur + q_dot * dt, ...)`` propagates the NaN "
        "from q_cur (= state.leg_extensions_mm) into cmd, then into "
        "self._prev_u.  Subsequent solves then warm-start from a "
        "non-finite _prev_u — the corruption cascade the warm-start "
        "integrity invariant exists to prevent.\n\n"
        "Plan 2 Phase 3 surfaces the bug; per the plan's "
        "'Production-code changes triggered by tests' rule the fix "
        "lands in its own commit with its own logbook entry.  When "
        "the fix is committed, this xfail marker MUST be removed in "
        "the same commit, AND _T1C_FAULT_FIELD in this file MUST be "
        "widened to re-include ('leg_extensions_mm', 6).\n\n"
        "Three-field xfail accounting (Plan 2 WN#2):\n"
        "  test ID:  T-U-T1c-7-bug\n"
        "  tracking: logbook/2026-05-11-tier1c-input-fuzz.md "
        "(Discussion → 'Bug surfaced — _prev_u corruption via "
        "hold_extrap when q_cur is non-finite')\n"
        "  target:   Plan 2 archival (the bug fix is the gate)"
    ),
)
class TestT1cLegExtNanCorruptsPrevU:
    """T-U-T1c-7-bug — deterministic demonstration of the
    NaN-in-leg_extensions_mm + stale-snapshot corruption path that
    T-U-T1c-7's hypothesis stateful machine surfaced.

    Reproducer recipe:
      1. Seed a successful solve (populates ``_prev_u``, ``_prev_w``,
         W7 snapshot fields).
      2. Force the W7 staleness branch by aging
         ``_t_at_last_success`` past 500 ms.
      3. Set ``state.leg_extensions_mm[0] = NaN``.
      4. Solve.
      5. Assert ``_prev_u`` is finite.

    Expected (current production behaviour, the bug): assertion fires.
    Once fixed: assertion holds; remove xfail marker AND widen
    ``_T1C_FAULT_FIELD`` in this file."""

    def test_nan_leg_ext_plus_stale_snapshot_keeps_prev_u_finite(self,
                                                                   fresh_plant):
        mpc = _create_mpc(fresh_plant)
        _seed_mpc(fresh_plant, mpc)
        # Force walk_forward_unsafe via the staleness branch.
        mpc._t_at_last_success -= 0.6
        # Inject NaN through the alias.
        state = fresh_plant.get_state()
        state.leg_extensions_mm[0] = np.nan

        mpc.solve(state, REF_NORMAL)

        assert mpc._prev_u is None or np.all(np.isfinite(mpc._prev_u)), (
            f"BUG: _prev_u contains non-finite values after a NaN-in-"
            f"leg_extensions_mm fault on the hold_extrap path: "
            f"{mpc._prev_u!r}.  This is the corruption cascade the "
            f"warm-start integrity invariant exists to prevent."
        )
