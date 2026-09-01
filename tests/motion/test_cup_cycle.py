"""``motion/trajectory/cup_cycle`` — the numpy QP port of the sim cup planner.

WHAT THESE TESTS DEFEND
-----------------------
``cup_cycle.py`` replaces a CasADi/IPOPT nonlinear program with a hand-rolled
dense QP and a hand-rolled Goldfarb–Idnani dual active set. Two things can go
wrong with that, and only one of them is loud:

* **T-U1 (physics)** — the trajectory violates an invariant the whole juggle
  rests on: the cup is not in free-fall at release, the ball gets a sideways
  shove at detach, the cup is not where the ball lands, a box is breached, or
  the catch is planned with no slider stroke left to decelerate into.
* **T-U2 (parity)** — the QP quietly disagrees with the reference formulation.
  Phase 0 measured the naive active-set loop returning trajectories **124 m**
  from the reference *without raising anything*, and a 0.40 s cycle found while
  building T-U1 terminated with every inequality satisfied and an equality
  residual of **121**. A parity gate is the only instrument that sees this class.

The reference is FROZEN, not recomputed: CasADi is not importable in this test
environment (``motion/trajectory/`` is numpy-only Python 3.8 by architectural
rule, and CasADi lives only in the project venv). The fixtures are committed at
``tools/probes/data/cup_cycle_qp_refs.npz``.

REGENERATE THE FIXTURES (venv interpreter only — CasADi 3.7.2 lives there)::

    /home/jetson/Desktop/PDJ_venv/venv/bin/python \
        tools/probes/capture_cup_cycle_refs.py --emit-fixture

Unmarked and parallel-safe: this is production planner code on the unified
7-DoF path, not research characterisation, so it belongs in the per-commit gate.
No timing assertions live here (the Phase 0 logbook entry owns the runtime
numbers) and nothing touches the filesystem outside ``tmp_path``.

Plan: ``plans/active/unified-7dof-planner.md`` § 4 Phase 1.
Phase 0 record: ``logbook/2026-08-30-unified-7dof-planner-phase0-probes.md``.
"""

from __future__ import annotations

import dataclasses
import json
import os

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.trajectory import cup_cycle as cc

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
FIXTURE_PATH = os.path.join(_REPO_ROOT, 'tools', 'probes', 'data',
                            'cup_cycle_qp_refs.npz')

#: The plan's parity bars (§ 4 Phase 0 probe 1), in mm and mm/s.
PARITY_POS_MM = 1.0
PARITY_VEL_MM_S = 10.0


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def reference():
    """``(cases, arrays)`` from the committed CasADi capture.

    Module-scoped and read-only — the npz is loaded once per worker and never
    written, so this stays safe under ``--dist loadfile``.
    """
    if not os.path.exists(FIXTURE_PATH):
        pytest.fail(
            "missing reference fixture %s — regenerate with: "
            "/home/jetson/Desktop/PDJ_venv/venv/bin/python "
            "tools/probes/capture_cup_cycle_refs.py --emit-fixture"
            % FIXTURE_PATH)
    data = np.load(FIXTURE_PATH, allow_pickle=False)
    return json.loads(str(data['params_json'])), data


def _plan_reference_case(case, *, runway=False, warm_start=None):
    """Run the port on one captured case, legacy constraint set by default."""
    cfg = cc.CupCycleConfig(z_min_m=case['z_min_m'], z_max_m=case['z_max_m'],
                            catch_runway_enabled=runway)
    return cc.plan_cup_cycle(
        case['pos0'], case['vel0'], case['acc0'],
        case['throw_pos'], case['throw_target'], case['flight_s'],
        case['catch_time_s'], case['catch_pos'], case['catch_vel'],
        case['period_s'], cfg=cfg, detach_axis=case['detach_axis'],
        warm_start=warm_start)


# ---------------------------------------------------------------------------
# The T-U1 grid
# ---------------------------------------------------------------------------

#: Lateral amplitude (m) per cycle period. It SHRINKS with the period on
#: purpose and the shrink is physics, not tuning: the lateral channel is the
#: band-limited platform at ``max_jerk_xy = 300 m/s³``, and a jerk-limited
#: out-and-back of duration T covers roughly ``j·T³/32``. At 0.4 s that is
#: ~55 mm for the return leg alone, so a 0.15 m swing at 0.4 s is genuinely
#: unreachable — measured: amplitudes ≤ 0.03 m plan at 0.4 s and ≥ 0.05 m do
#: not. ``test_short_cycle_overreach_is_refused`` pins that boundary rather
#: than letting this table quietly hide it.
_GRID_AMPLITUDE_M = {0.4: 0.02, 0.6: 0.05, 0.8: 0.08, 1.0: 0.10, 1.2: 0.10}
_GRID_FLIGHTS_S = (0.6, 0.8, 1.0)
#: One catch fraction per flight, so the catch knot lands at a different phase
#: of the cycle (and a different interpolation offset ``d`` inside its knot) in
#: each column of the grid.
_GRID_CATCH_FRAC = (0.45, 0.55, 0.65)


def _grid_case(period_s, flight_s, tilted):
    """A physically-consistent cycle: throw at +A, catch displaced at −A."""
    amp = _GRID_AMPLITUDE_M[period_s]
    throw_pos = np.array([amp, 0.4 * amp, 0.80])
    throw_target = np.array([-amp, -0.4 * amp, 0.80])
    v_take = cc.takeoff_velocity(throw_pos, throw_target, flight_s)
    frac = _GRID_CATCH_FRAC[_GRID_FLIGHTS_S.index(flight_s)]
    return dict(
        period_s=period_s, flight_s=flight_s,
        throw_pos=throw_pos, throw_target=throw_target,
        # The cycle starts JUST AFTER a throw: the achieved state is the
        # release state, and the detach axis is that release's cup axis.
        pos0=throw_pos, vel0=v_take, acc0=cc.GRAVITY,
        detach_axis=(v_take / np.linalg.norm(v_take)) if tilted else None,
        catch_time_s=period_s * frac,
        catch_pos=np.array([-0.9 * amp, -0.5 * amp, 0.78]),
        catch_vel=np.array([0.10, -0.05, -4.50]))


GRID = [(p, f, t) for p in sorted(_GRID_AMPLITUDE_M)
        for f in _GRID_FLIGHTS_S for t in (False, True)]
GRID_IDS = ['p%03d_f%03d_%s' % (p * 1000, f * 1000, 'tilt' if t else 'level')
            for p, f, t in GRID]


@pytest.mark.parametrize('period_s,flight_s,tilted', GRID, ids=GRID_IDS)
def test_grid_cycle_invariants(period_s, flight_s, tilted):
    """T-U1 — every planned cycle satisfies the physics it is built on.

    Release free-fall, detach collinearity, catch placement, both box families
    and the catch runway, over periods 0.4–1.2 s × flights 0.6–1.0 s × level and
    tilted detach axes, with the catch site displaced from the throw site.

    Every grid point must PLAN. A point that raises here is a real regression,
    not a licence to trim the grid: the whole set was solved when it was pinned
    (2026-08-30), so the constraint set has moved if one stops.
    """
    case = _grid_case(period_s, flight_s, tilted)
    cfg = cc.CupCycleConfig()
    plan = cc.plan_cup_cycle(
        case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
        case['throw_target'], case['flight_s'], case['catch_time_s'],
        case['catch_pos'], case['catch_vel'], case['period_s'],
        cfg=cfg, detach_axis=case['detach_axis'])

    n = int(round(period_s / cfg.dt))
    assert plan.pos.shape == (n + 1, 3)
    assert plan.vel.shape == (n + 1, 3)
    assert plan.acc.shape == (n + 1, 3)
    assert plan.jerk.shape == (n, 3)
    assert plan.t.shape == (n + 1,)

    # Release: the cup is in FREE FALL at the throw, so the ball detaches with
    # no contact force at all. This is Kai's `cacc == g`.
    assert np.linalg.norm(plan.acc[-1] - cc.GRAVITY) < 1e-6

    # Detach: over the first n_detach knots the net applied acceleration is
    # purely AXIAL, so the just-released ball gets no sideways shove. Level and
    # tilted are the SAME invariant with a different axis.
    axis = (np.array([0.0, 0.0, 1.0]) if case['detach_axis'] is None
            else np.asarray(case['detach_axis'], dtype=float))
    for i in range(1, cfg.n_detach + 1):
        cross = np.cross(plan.acc[i] - cc.GRAVITY, axis)
        assert np.linalg.norm(cross) < 1e-6, 'detach knot %d' % i

    # Catch: the cup is where the ball lands, at the INTERPOLATED touch-down
    # time — sampled through the plan's own cubic, which is what the emitter
    # will do.
    catch_err_mm = np.linalg.norm(
        plan.sample(case['catch_time_s'])[0] - case['catch_pos']) * 1e3
    assert catch_err_mm < 0.1

    # Boxes: per-axis jerk and the workspace prism.
    assert np.abs(plan.jerk[:, :2]).max() <= cfg.max_jerk_xy + 1e-6
    assert np.abs(plan.jerk[:, 2]).max() <= cfg.max_jerk_z + 1e-6
    assert np.abs(plan.pos[:, :2]).max() <= cfg.workspace_xy_m + 1e-9
    assert plan.pos[:, 2].min() >= cfg.z_min_m - 1e-9
    assert plan.pos[:, 2].max() <= cfg.z_max_m + 1e-9

    # Catch runway: stroke left below the catch knot for the deceleration the
    # target catch velocity implies.
    floor_m, need_m = cc.catch_runway_requirement(case['catch_vel'], cfg)
    assert plan.pos[plan.catch_k, 2] - floor_m >= need_m - 1e-9

    # catch_k indexes the knot at/just before the touch-down.
    assert (plan.catch_k * cfg.dt <= case['catch_time_s'] + 1e-12
            < (plan.catch_k + 1) * cfg.dt)


def test_short_cycle_overreach_is_refused():
    """A cycle that is genuinely infeasible RAISES — it does not sneak through.

    A 0.4 s period with the 0.8 s grid's 0.10 m lateral amplitude asks the
    band-limited platform for an out-and-back it cannot make at
    ``max_jerk_xy``. The CasADi/IPOPT reference refuses this exact request too
    (checked 2026-08-30 under the venv, ``Solver failed``), so the refusal is
    the correct answer and not our solver being weaker. The point of the test is
    that it refuses rather than returning the kind of constraint-violating
    trajectory Phase 0 caught the naive loop returning.

    The refusal PATH is deliberately not asserted — it can be the unbounded
    dual step or the ``_verify`` gate depending on BLAS
    (see :func:`test_verify_refuses_a_point_that_misses_the_equalities`).
    """
    case = _grid_case(0.8, 0.8, False)          # borrows the 0.10 m amplitude
    with pytest.raises(cc.CupCycleInfeasible):
        cc.plan_cup_cycle(
            case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
            case['throw_target'], case['flight_s'], 0.4 * 0.55,
            case['catch_pos'], case['catch_vel'], 0.4,
            cfg=cc.CupCycleConfig(), detach_axis=None)


#: The probe recipe behind ``test_catch_inside_the_detach_block_is_refused``.
#: A short throw-to-throw window whose catch is dragged into the opening knots.
def _early_catch_window(t_catch, runway):
    """``(events, state0, cfg, period_s)`` for a catch ``t_catch`` s into the cycle."""
    throw_pos = np.array([0.02, 0.0, 0.86])
    target = np.array([-0.02, 0.0, 0.86])
    v_take = cc.takeoff_velocity(throw_pos, target, 0.6)
    state0 = cc.CupState(pos=throw_pos, vel=v_take, acc=cc.GRAVITY)
    cfg = cc.CupCycleConfig(z_min_m=0.69, z_max_m=0.985,
                            catch_runway_z_floor_m=0.69,
                            catch_runway_enabled=runway)
    events = [
        cc.CatchEvent(0, t_catch, np.array([0.019, 0.0, 0.83]),
                      np.array([0.0, 0.0, -2.5])),
        cc.ThrowEvent(1, 1.4, throw_pos, target, 0.6),
    ]
    return events, state0, cfg, 1.4


@pytest.mark.parametrize('runway', [True, False], ids=['runway_on', 'runway_off'])
@pytest.mark.parametrize('t_catch', [0.010, 0.026])
def test_catch_inside_the_detach_block_is_refused(t_catch, runway):
    """A catch landing in the detach block is RANK-DEFICIENT and must refuse.

    The detach equalities pin the cup's acceleration at knots ``1..n_detach``,
    and their jerk support is the leading variables ``0..n_detach-1``. The
    interpolated catch-position equality reaches only ``0..k_td``. With
    ``k_td <= n_detach`` those rows live on a support too small to carry them:
    the KKT matrix is singular and every number downstream of it is noise. This
    is not the same failure as an over-reaching cycle — the program is
    ill-posed before the solver ever runs — so it refuses in ``_assemble`` with
    its own reason rather than surfacing as a mystery working set 200 iterations
    deep.

    CONFIRMED RECIPE (empirical probe, venv interpreter, 2026-09-01, run twice
    with identical output): throw ``[0.02, 0, 0.86]`` → target
    ``[-0.02, 0, 0.86]`` at flight 0.6 s over a 1.4 s window, catch site
    ``[0.019, 0, 0.83]`` at ``[0, 0, -2.5]`` m/s, ``z_min 0.69 / z_max 0.985``,
    runway floor 0.69. At the default ``dt = 0.025`` and ``n_detach = 2``,
    ``t_catch = 0.010`` gives ``k_td = 0`` and ``0.026`` gives ``k_td = 1``;
    both refuse with ``CATCH_TOO_EARLY``, and so does ``0.024``. Asserted under
    BOTH runway settings because the two take different assembly paths to the
    same gate, and the gate must precede either. The refusal is BLAS-independent
    by construction — it is raised before any linear algebra — unlike the
    ``_verify`` path (see
    :func:`test_verify_refuses_a_point_that_misses_the_equalities`).

    A mid-cycle catch on the same window plans normally (measured
    ``catch_k = 30`` at ``t_catch = 0.77``), which is what proves the refusal is
    the detach gate and not the window being infeasible in general.
    """
    events, state0, cfg, period_s = _early_catch_window(t_catch, runway)
    with pytest.raises(cc.CupCycleInfeasible) as excinfo:
        cc.plan_window(events, state0, cfg, period_s=period_s)
    assert excinfo.value.reason == 'CATCH_TOO_EARLY'
    assert 'detach block' in str(excinfo.value)

    # Same window, catch moved clear of the detach block: plans fine.
    ok_events, state0, cfg, period_s = _early_catch_window(0.77, runway)
    plan = cc.plan_window(ok_events, state0, cfg, period_s=period_s)
    assert plan.catch_k > cfg.n_detach


def test_verify_refuses_a_point_that_misses_the_equalities():
    """The ``_verify`` gate, tested directly on a known-good program.

    WHY IT EXISTS, and why it is tested as a unit rather than through a real
    infeasible cycle: a 0.40 s grid point found on 2026-08-30 drove the working
    set to 38 of 48 rows, made ``NᵀH⁻¹N`` numerically singular, and terminated
    with **every inequality satisfied** and an equality residual of **121** — a
    cup trajectory 121 units off the constraint manifold, returned without a
    word. Inequality feasibility alone does not see it; the equality residual is
    the only witness.

    That case is NOT the driver here. It sits on a numerical knife-edge: on the
    same Python 3.8.10 / numpy 1.24.4 it refuses through the unbounded-dual path
    under the venv and through ``_verify`` under the system interpreter,
    differing only by BLAS. Pinning it by refusal path would pin the BLAS. So
    the gate is driven directly: perturb a verified solution off the equality
    manifold and require the refusal.
    """
    case = _grid_case(0.8, 0.8, False)
    state0 = cc.CupState(case['pos0'], case['vel0'], case['acc0'], None)
    throw = cc.ThrowEvent(1, case['period_s'], case['throw_pos'],
                          case['throw_target'], case['flight_s'])
    catch = cc.CatchEvent(0, case['catch_time_s'], case['catch_pos'],
                          case['catch_vel'])
    prog = cc._assemble(state0, throw, catch, case['period_s'],
                        cc.CupCycleConfig())
    good, active, iters = cc._solve_qp(prog, None, 300, 1e-9, 1e-7)
    cc._verify(prog, good, 1e-7, active, iters)          # must not raise

    # Off the equality manifold by ~1 unit, every inequality still slack.
    null_breaking = np.zeros_like(good)
    null_breaking[0] = 1.0
    with pytest.raises(cc.CupCycleInfeasible) as excinfo:
        cc._verify(prog, good + null_breaking, 1e-7, active, iters)
    assert excinfo.value.reason == 'UNVERIFIED'
    with pytest.raises(cc.CupCycleInfeasible):
        cc._verify(prog, good * np.nan, 1e-7, active, iters)


def test_solve_is_deterministic():
    """Two solves of the same cycle agree bit for bit.

    Phase 0 pinned determinism (its parity probe ran twice and compared), and it
    is a real property to defend: a planner re-solved every cycle whose answer
    wandered between identical inputs would make every downstream regression
    unreproducible.
    """
    case = _grid_case(1.0, 0.8, True)
    args = (case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
            case['throw_target'], case['flight_s'], case['catch_time_s'],
            case['catch_pos'], case['catch_vel'], case['period_s'])
    first = cc.plan_cup_cycle(*args, detach_axis=case['detach_axis'])
    second = cc.plan_cup_cycle(*args, detach_axis=case['detach_axis'])
    assert np.array_equal(first.jerk, second.jerk)
    assert np.array_equal(first.pos, second.pos)


# ---------------------------------------------------------------------------
# T-U2 — parity against the frozen CasADi reference
# ---------------------------------------------------------------------------

def test_reference_fixture_is_the_expected_capture(reference):
    """The committed fixture is the six-case capture the parity gate assumes.

    Regenerate with:
      ``/home/jetson/Desktop/PDJ_venv/venv/bin/python
      tools/probes/capture_cup_cycle_refs.py --emit-fixture``
    """
    cases, data = reference
    assert [c['name'] for c in cases] == [
        'fixture_level', 'fixture_tilted', 'grid_p050_lat', 'grid_p075_diag',
        'grid_p100_tall', 'grid_p065_tilted']
    assert any(c['detach_axis'] is not None for c in cases), 'no tilted case'
    for case in cases:
        for field in ('pos', 'vel', 'acc', 'jerk', 't', 'catch_k',
                      'takeoff_vel'):
            assert case['name'] + '/' + field in data.files


def test_qp_matches_casadi_reference(reference):
    """T-U2 — < 1 mm / < 10 mm/s against IPOPT on every captured cycle.

    The runway constraint is OFF here: the reference program does not have it,
    so this is the exact-legacy-parity assertion. ``test_runway_is_inactive_on
    _the_reference_cycles`` covers the constraint-ON case separately.

    Regenerate the reference with:
      ``/home/jetson/Desktop/PDJ_venv/venv/bin/python
      tools/probes/capture_cup_cycle_refs.py --emit-fixture``
    """
    cases, data = reference
    for case in cases:
        plan = _plan_reference_case(case)
        name = case['name']
        d_pos_mm = np.abs(plan.pos - data[name + '/pos']).max() * 1e3
        d_vel_mm_s = np.abs(plan.vel - data[name + '/vel']).max() * 1e3
        assert d_pos_mm < PARITY_POS_MM, '%s: %.4g mm' % (name, d_pos_mm)
        assert d_vel_mm_s < PARITY_VEL_MM_S, '%s: %.4g mm/s' % (name, d_vel_mm_s)
        assert plan.catch_k == int(data[name + '/catch_k'])
        assert np.allclose(plan.takeoff_vel, data[name + '/takeoff_vel'],
                           atol=1e-12)
        assert plan.jerk.shape == data[name + '/jerk'].shape
        assert np.allclose(plan.t, data[name + '/t'], atol=1e-12)


def test_plan_window_matches_plan_cup_cycle_on_one_throw_one_catch(reference):
    """T-U2's event-timeline twin (owner resolution 3, 2026-08-29).

    v1 callers pass exactly one throw and one catch, and on that timeline
    :func:`plan_window` must be the SAME planner as the ``plan_cup_cycle``
    wrapper — bit-identical, not merely close, because the wrapper is a pure
    re-spelling of the arguments. Parity against the CasADi reference then
    transfers to the event API for free, which is the point of asserting it
    here rather than only on the wrapper.
    """
    cases, data = reference
    for case in cases:
        cfg = cc.CupCycleConfig(z_min_m=case['z_min_m'],
                                z_max_m=case['z_max_m'],
                                catch_runway_enabled=False)
        events = [
            cc.CatchEvent(ball_id=7, t_s=case['catch_time_s'],
                          site=np.array(case['catch_pos'], dtype=float),
                          vel=np.array(case['catch_vel'], dtype=float)),
            cc.ThrowEvent(ball_id=8, t_s=case['period_s'],
                          site=np.array(case['throw_pos'], dtype=float),
                          target=np.array(case['throw_target'], dtype=float),
                          flight_s=case['flight_s']),
        ]
        state0 = cc.CupState(
            pos=np.array(case['pos0'], dtype=float),
            vel=np.array(case['vel0'], dtype=float),
            acc=np.array(case['acc0'], dtype=float),
            detach_axis=(None if case['detach_axis'] is None
                         else np.array(case['detach_axis'], dtype=float)))
        via_window = cc.plan_window(events, state0, cfg)
        via_wrapper = _plan_reference_case(case)
        assert np.array_equal(via_window.pos, via_wrapper.pos)
        assert np.array_equal(via_window.vel, via_wrapper.vel)
        assert np.array_equal(via_window.jerk, via_wrapper.jerk)
        assert via_window.catch_k == via_wrapper.catch_k
        # ...and therefore inside the parity bar against IPOPT.
        d_pos_mm = np.abs(via_window.pos - data[case['name'] + '/pos']).max() * 1e3
        assert d_pos_mm < PARITY_POS_MM


def test_runway_is_inactive_on_the_reference_cycles(reference):
    """Turning the runway ON changes nothing on the six captured cycles.

    Worth pinning: it means the added constraint is *slack* on realistic
    cycles, so the parity result above is not an artefact of having disabled
    it, and enabling it by default does not silently reshape known-good plans.
    """
    cases, _ = reference
    for case in cases:
        off = _plan_reference_case(case, runway=False)
        on = _plan_reference_case(case, runway=True)
        assert np.array_equal(off.pos, on.pos), case['name']
        assert np.array_equal(off.jerk, on.jerk), case['name']


# ---------------------------------------------------------------------------
# The catch-runway constraint
# ---------------------------------------------------------------------------

def test_runway_requirement_is_the_stopping_distance():
    """``v²/(2a) + margin`` on the TARGET catch speed, and nothing else."""
    cfg = cc.CupCycleConfig(z_min_m=0.50, catch_runway_margin_m=0.02,
                            catch_slider_vel_ratio=0.7,
                            catch_runway_decel_mps2=110.0)
    floor_m, need_m = cc.catch_runway_requirement([0.0, 0.0, -5.0], cfg)
    assert floor_m == pytest.approx(0.50)          # defaults to z_min_m
    v_target = 0.7 * 5.0
    assert need_m == pytest.approx(v_target ** 2 / (2 * 110.0) + 0.02)
    # An explicit floor overrides z_min_m — the slider bottom is geometry the
    # caller owns, not something this module infers.
    cfg.catch_runway_z_floor_m = 0.66
    assert cc.catch_runway_requirement([0.0, 0.0, -5.0], cfg)[0] == pytest.approx(0.66)


def test_runway_default_decel_is_the_signed_off_hand_limit():
    """3500 rev/s² ÷ the package's slider gain, and the gain's two sources agree.

    3500 rev/s² is the owner-signed hand acceleration limit (Phase 0 decision 4,
    2026-08-30), under the C-HAND-2 authority bound of 3925.5. The gain is taken
    from ``hand_stroke``, which DERIVES it from the firmware spool geometry;
    ``TEENSY_LINEAR_GAIN`` is the generated constant the firmware itself uses.
    They must be the same number — the runway distance is sized against one and
    the hardware obeys the other.
    """
    assert cc.LINEAR_GAIN_REV_PER_M == hw.TEENSY_LINEAR_GAIN
    assert cc.HAND_ACC_LIMIT_RPS2 == 3500.0
    assert cc.HAND_ACC_LIMIT_RPS2 == hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2
    assert cc.HAND_MAX_DECEL_MPS2 == pytest.approx(3500.0 / hw.TEENSY_LINEAR_GAIN)
    assert cc.CupCycleConfig().catch_runway_decel_mps2 == cc.HAND_MAX_DECEL_MPS2


def test_catch_with_no_runway_is_refused_and_names_itself():
    """A catch too close to the slider floor is REFUSED before the solve.

    This is the constraint doing the job it was added for (owner resolution 1,
    2026-08-29): deceleration room is *planned*, not obtained by overshoot. The
    same cycle plans with the constraint disabled, which is what proves the
    refusal comes from the runway and not from some unrelated infeasibility.
    """
    case = _grid_case(0.8, 0.8, False)
    # Floor raised to 25 mm below the catch: nowhere near the ~0.08 m the
    # 0.7 × 4.5 m/s target catch speed needs at the default authority.
    kwargs = dict(z_min_m=0.45, catch_runway_z_floor_m=0.755)
    with pytest.raises(cc.CupCycleInfeasible) as excinfo:
        cc.plan_cup_cycle(
            case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
            case['throw_target'], case['flight_s'], case['catch_time_s'],
            case['catch_pos'], case['catch_vel'], case['period_s'],
            cfg=cc.CupCycleConfig(catch_runway_enabled=True, **kwargs))
    assert excinfo.value.reason == 'CATCH_RUNWAY'
    assert 'runway' in str(excinfo.value)
    # Same cycle, constraint off: plans fine. The refusal is the runway's.
    plan = cc.plan_cup_cycle(
        case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
        case['throw_target'], case['flight_s'], case['catch_time_s'],
        case['catch_pos'], case['catch_vel'], case['period_s'],
        cfg=cc.CupCycleConfig(catch_runway_enabled=False, **kwargs))
    assert plan.pos.shape[0] == int(round(0.8 / 0.025)) + 1


def test_runway_adds_a_real_row_to_the_program():
    """The knot bound is a constraint in the QP, not a docstring.

    Enabling the runway must add exactly one inequality column. Pinned because
    the QP-row half of the constraint is, on a DESCENDING catch (every real
    one), implied by the analytic touch-down gate — so nothing else in this
    file would notice if it silently vanished.
    """
    case = _grid_case(0.8, 0.8, False)
    state0 = cc.CupState(case['pos0'], case['vel0'], case['acc0'], None)
    throw = cc.ThrowEvent(1, case['period_s'], case['throw_pos'],
                          case['throw_target'], case['flight_s'])
    catch = cc.CatchEvent(0, case['catch_time_s'], case['catch_pos'],
                          case['catch_vel'])
    off = cc._assemble(state0, throw, catch, case['period_s'],
                       cc.CupCycleConfig(catch_runway_enabled=False))
    on = cc._assemble(state0, throw, catch, case['period_s'],
                      cc.CupCycleConfig(catch_runway_enabled=True))
    assert on.C.shape[1] == off.C.shape[1] + 1


# ---------------------------------------------------------------------------
# The cup acceleration boxes (WP4)
# ---------------------------------------------------------------------------

def _assembled(case, cfg):
    """``_assemble`` on a grid case, level detach."""
    state0 = cc.CupState(case['pos0'], case['vel0'], case['acc0'], None)
    throw = cc.ThrowEvent(1, case['period_s'], case['throw_pos'],
                          case['throw_target'], case['flight_s'])
    catch = cc.CatchEvent(0, case['catch_time_s'], case['catch_pos'],
                          case['catch_vel'])
    return cc._assemble(state0, throw, catch, case['period_s'], cfg)


def test_acc_box_appends_its_own_column_block_after_every_existing_one():
    """The z box adds ``2n`` columns and MOVES NOTHING already there.

    Column order is the load-bearing property, not just the count. A working
    set is a list of constraint INDICES, so a box inserted mid-array would
    silently re-point every warm start at a different constraint, and the
    T-U1/T-U2 parity fixtures would stop being bit-comparable. Asserting the
    prefix is byte-identical is what pins "appended, not interleaved" — a count
    check alone would pass an implementation that inserted the block at the
    front.
    """
    case = _grid_case(0.8, 0.8, False)
    n = int(round(case['period_s'] / 0.025))
    off = _assembled(case, cc.CupCycleConfig(catch_runway_enabled=False))
    on = _assembled(case, cc.CupCycleConfig(catch_runway_enabled=False,
                                            max_acc_z=200.0))
    assert on.C.shape[1] == off.C.shape[1] + 2 * n
    assert np.array_equal(on.C[:, :off.C.shape[1]], off.C)


def test_acc_box_below_gravity_is_refused_by_name():
    """``max_acc_z < |g|`` is infeasible BY CONSTRUCTION, and says so.

    The release equality pins the cup's acceleration to ``g`` exactly, so any z
    box under 9.806 m/s² contradicts a hard equality. That is a caller error in
    the config, not a hard cycle — it must arrive as ``ACC_BOX`` before the
    solve, not as an unbounded dual step whose message names a constraint index.
    """
    case = _grid_case(0.8, 0.8, False)
    args = (case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
            case['throw_target'], case['flight_s'], case['catch_time_s'],
            case['catch_pos'], case['catch_vel'], case['period_s'])
    with pytest.raises(cc.CupCycleInfeasible) as excinfo:
        cc.plan_cup_cycle(*args, cfg=cc.CupCycleConfig(max_acc_z=5.0))
    assert excinfo.value.reason == 'ACC_BOX'
    assert 'max_acc_z' in str(excinfo.value)


def test_binding_acc_box_shapes_the_plan_and_holds():
    """A box that BINDS is honoured, and it really did change the trajectory.

    Both halves matter. "Under the bound" alone is satisfied by any box the
    plan was already inside, which would pass even if the rows were never
    assembled — ``test_runway_adds_a_real_row_to_the_program`` exists for
    exactly that failure mode on the runway. Requiring the plan to DIFFER from
    the unboxed one is what proves the box is shaping rather than decorating.

    CONFIRMED RECIPE (empirical probe, venv interpreter, 2026-09-01, run twice
    bit-identically): the 1.2 s / 0.6 s flight grid case peaks at 19.908 m/s²
    unboxed, so a 17.0 m/s² box binds. The feasible edge is between 15 and 16
    (15.0 refuses, 16.0 and 17.0 plan), so 17.0 carries real margin on both
    sides rather than sitting on a knife-edge; the same box also plans on the
    1.0 s period. Boxes below ~15 refuse because the release still has to reach
    ``acc == g`` from whatever the catch left behind.
    """
    case = _grid_case(1.2, 0.6, False)
    args = (case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
            case['throw_target'], case['flight_s'], case['catch_time_s'],
            case['catch_pos'], case['catch_vel'], case['period_s'])
    box = 17.0
    base = cc.plan_cup_cycle(*args, cfg=cc.CupCycleConfig())
    assert np.abs(base.acc[:, 2]).max() > box, 'box must BIND to test anything'
    boxed = cc.plan_cup_cycle(*args, cfg=cc.CupCycleConfig(max_acc_z=box))
    assert np.abs(boxed.acc[:, 2]).max() <= box + 1e-6
    assert not np.array_equal(base.pos, boxed.pos)
    # Still a legal trajectory in every other respect.
    assert np.linalg.norm(boxed.acc[-1] - cc.GRAVITY) < 1e-6


def test_acc_boxes_none_is_bit_identical_to_the_default_program():
    """Passing the boxes explicitly OFF is the default program, bit for bit.

    ``None`` is the default, so this looks tautological — it is not. The
    assembly branches on ``acc_z is not None or acc_xy is not None``, and a
    transcription that treated an explicit ``None`` as "box present, limit
    unset" would append a degenerate block, shift every column index, and break
    the parity fixtures. This is the cheapest possible guard on that branch.
    """
    case = _grid_case(0.8, 0.8, False)
    args = (case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
            case['throw_target'], case['flight_s'], case['catch_time_s'],
            case['catch_pos'], case['catch_vel'], case['period_s'])
    explicit = cc.plan_cup_cycle(
        *args, cfg=cc.CupCycleConfig(max_acc_z=None, max_acc_xy=None))
    default = cc.plan_cup_cycle(*args, cfg=cc.CupCycleConfig())
    assert np.array_equal(explicit.pos, default.pos)
    assert np.array_equal(explicit.jerk, default.jerk)


# ---------------------------------------------------------------------------
# API surface, warm start, duck-typing
# ---------------------------------------------------------------------------

def test_cup_cycle_plan_mirrors_the_sim_dataclass_fields():
    """The plan object duck-types with ``sim.juggle_planner.CupCyclePlan``.

    Pinned as a literal list rather than compared against the sim class,
    because importing ``sim.juggle_planner`` pulls in CasADi, which does not
    exist in this environment — the very constraint that made this port
    necessary. ``warm_start`` is this port's only addition and is deliberately
    LAST with a default, so positional construction written against the sim
    dataclass still works.
    """
    names = [f.name for f in dataclasses.fields(cc.CupCyclePlan)]
    assert names == ['pos', 'vel', 'acc', 'jerk', 't', 'dt', 'catch_k',
                     'takeoff_vel', 'warm_start']
    assert dataclasses.fields(cc.CupCyclePlan)[-1].default is None
    assert callable(cc.CupCyclePlan.sample)


def test_sample_is_the_plans_own_cubic():
    """``sample`` agrees with the knots it interpolates, and clamps outside."""
    case = _grid_case(0.8, 0.8, True)
    plan = cc.plan_cup_cycle(
        case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
        case['throw_target'], case['flight_s'], case['catch_time_s'],
        case['catch_pos'], case['catch_vel'], case['period_s'],
        detach_axis=case['detach_axis'])
    for k in range(plan.jerk.shape[0]):
        p, v, a = plan.sample(k * plan.dt)
        assert np.allclose(p, plan.pos[k], atol=1e-12)
        assert np.allclose(v, plan.vel[k], atol=1e-12)
        assert np.allclose(a, plan.acc[k], atol=1e-12)
    end = plan.jerk.shape[0] * plan.dt
    assert np.allclose(plan.sample(-5.0)[0], plan.pos[0], atol=1e-12)
    assert np.allclose(plan.sample(end + 5.0)[0], plan.sample(end)[0],
                       atol=1e-12)


def test_warm_start_reproduces_the_same_solution(reference):
    """A warm start buys iterations, never a different answer.

    The warm start biases only WHICH violated constraint is admitted next; any
    violated constraint is a legal choice for Goldfarb–Idnani, so the optimum
    and the final working set must be unchanged. That is the contract worth
    pinning — an implementation that let the warm start move the answer would
    be a correctness bug wearing a performance costume.
    """
    cases, _ = reference
    for case in cases:
        cold = _plan_reference_case(case)
        warm = _plan_reference_case(case, warm_start=cold.warm_start)
        assert np.allclose(warm.pos, cold.pos, atol=1e-12), case['name']
        assert np.allclose(warm.jerk, cold.jerk, atol=1e-12), case['name']
        assert set(warm.warm_start.active) == set(cold.warm_start.active)


def test_mismatched_warm_start_is_ignored_not_misapplied(reference):
    """A warm start from a differently-shaped problem is dropped silently.

    Constraint INDICES are the working-set representation, so a warm start
    whose problem had a different knot count means nothing on the new one.
    Ignoring it must not corrupt the answer.
    """
    cases, _ = reference
    short = _plan_reference_case(cases[0])                   # n = 25
    tall = _plan_reference_case(cases[4])                    # n = 40
    assert short.warm_start.key != tall.warm_start.key
    reused = _plan_reference_case(cases[4], warm_start=short.warm_start)
    assert np.allclose(reused.pos, tall.pos, atol=1e-12)


def test_plan_window_rejects_timelines_v1_cannot_plan():
    """Refuse what v1 cannot do, rather than approximating it.

    A multi-ball window silently planned as a single-ball one would be a wrong
    trajectory delivered on the ball's clock, so the refusal is the safe
    behaviour and the error type says which limitation was hit.
    """
    case = _grid_case(0.8, 0.8, False)
    state0 = cc.CupState(case['pos0'], case['vel0'], case['acc0'], None)
    throw = cc.ThrowEvent(1, case['period_s'], case['throw_pos'],
                          case['throw_target'], case['flight_s'])
    catch = cc.CatchEvent(0, case['catch_time_s'], case['catch_pos'],
                          case['catch_vel'])
    with pytest.raises(NotImplementedError):
        cc.plan_window([catch, catch, throw], state0)
    with pytest.raises(NotImplementedError):
        cc.plan_window([throw], state0)
    with pytest.raises(TypeError):
        cc.plan_window([catch, throw, 'not-an-event'], state0)
    with pytest.raises(ValueError):        # out of time order
        cc.plan_window([throw, catch], state0, period_s=case['period_s'])
    late = cc.CatchEvent(0, case['period_s'] + 0.1, case['catch_pos'],
                         case['catch_vel'])
    with pytest.raises(ValueError):        # catch outside the window
        cc.plan_window([throw, late], state0, period_s=case['period_s'])


def test_level_axis_takes_the_level_path():
    """``detach_axis = [0,0,1]`` is the flat cup, bit-for-bit with ``None``.

    The sim planner guarantees this ("the tilt = 0 plan is byte-for-byte
    unchanged"), and it matters: the tilted branch writes two general rows
    where the level branch writes ``acc_xy == 0``, so a drifted level test
    would put every zero-tilt cycle on the wrong code path.
    """
    case = _grid_case(0.8, 0.8, False)
    args = (case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
            case['throw_target'], case['flight_s'], case['catch_time_s'],
            case['catch_pos'], case['catch_vel'], case['period_s'])
    a = cc.plan_cup_cycle(*args, detach_axis=None)
    b = cc.plan_cup_cycle(*args, detach_axis=np.array([0.0, 0.0, 1.0]))
    assert np.array_equal(a.pos, b.pos)
    assert np.array_equal(a.jerk, b.jerk)


def test_config_without_runway_fields_runs_the_legacy_constraint_set():
    """Handed a sim ``PlannerConfig``, the port plans the legacy program.

    The WP4 sim-parity harness drives both planners through one call site with
    one config object, so a config lacking this port's extra fields must be
    accepted — and must read as runway-OFF, since the constraint it describes
    does not exist in the program that config was written for. Modelled here
    with a stand-in carrying exactly the sim dataclass's fields, since
    importing the real one needs CasADi.
    """
    @dataclasses.dataclass
    class LegacyPlannerConfig:
        dt: float = 0.025
        max_jerk_z: float = 6000.0
        max_jerk_xy: float = 300.0
        lateral_accel_weight: float = 3.0
        n_detach: int = 2
        throw_coast_knots: int = 0
        throw_coast_weight: float = 0.0
        catch_vel_ratio: float = 0.7
        catch_vel_weight: float = 200.0
        catch_slider_vel_ratio: float = 0.7
        catch_slider_vel_weight: float = 600.0
        catch_dwell_pre: int = 0
        catch_dwell_post: int = 0
        catch_dwell_weight: float = 0.0
        workspace_xy_m: float = 0.15
        z_min_m: float = 0.45
        z_max_m: float = 1.10
        solver_print: bool = False

    case = _grid_case(0.8, 0.8, False)
    args = (case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
            case['throw_target'], case['flight_s'], case['catch_time_s'],
            case['catch_pos'], case['catch_vel'], case['period_s'])
    legacy = cc.plan_cup_cycle(*args, cfg=LegacyPlannerConfig())
    ours = cc.plan_cup_cycle(
        *args, cfg=cc.CupCycleConfig(catch_runway_enabled=False))
    assert np.array_equal(legacy.pos, ours.pos)


def test_optional_cost_terms_change_the_plan_without_breaking_it():
    """The dwell and coast knobs are wired, not decorative.

    Both are disabled by default in BOTH planners, so nothing else in this file
    exercises their assembly branches; a transcription error in either would
    otherwise sit unnoticed until the first person turned one on.
    """
    case = _grid_case(1.0, 0.8, False)
    args = (case['pos0'], case['vel0'], case['acc0'], case['throw_pos'],
            case['throw_target'], case['flight_s'], case['catch_time_s'],
            case['catch_pos'], case['catch_vel'], case['period_s'])
    base = cc.plan_cup_cycle(*args, cfg=cc.CupCycleConfig())
    dwell = cc.plan_cup_cycle(*args, cfg=cc.CupCycleConfig(
        catch_dwell_pre=2, catch_dwell_post=2, catch_dwell_weight=1500.0))
    coast = cc.plan_cup_cycle(*args, cfg=cc.CupCycleConfig(
        throw_coast_knots=3, throw_coast_weight=50.0))
    for other in (dwell, coast):
        assert not np.array_equal(base.pos, other.pos)
        # still a legal trajectory
        assert np.linalg.norm(other.acc[-1] - cc.GRAVITY) < 1e-6
        assert np.abs(other.jerk[:, :2]).max() <= 300.0 + 1e-6
    # The dwell does what it says: less lateral travel across the catch window.
    k = base.catch_k
    span = slice(max(0, k - 2), k + 3)
    assert (np.ptp(dwell.pos[span, :2], axis=0).sum()
            < np.ptp(base.pos[span, :2], axis=0).sum())


def test_integrator_maps_are_cached_and_bounded(tmp_path):
    """The coefficient cache is keyed on ``(n_steps, dt)`` and cannot grow forever.

    ``tmp_path`` is requested only to keep this test's footprint explicit; the
    module writes nothing to disk.
    """
    assert not list(tmp_path.iterdir())
    first = cc._integrator_maps(12, 0.025)
    assert cc._integrator_maps(12, 0.025) is first
    assert cc._integrator_maps(13, 0.025) is not first
    for n in range(100, 100 + cc._MAPS_CAP + 5):
        cc._integrator_maps(n, 0.025)
    assert len(cc._MAPS) <= cc._MAPS_CAP


def test_module_stays_inside_the_motion_boundary():
    """No ROS2, no repo-root, no ``controller`` imports — numpy and stdlib only.

    ``motion/trajectory/`` is pure-Python by architectural rule and this module
    is the one most tempted to break it (its source formulation lives in
    ``sim/`` and imports CasADi). Reading the source is the honest check: an
    import-based one would pass simply because the offending package happens to
    be installed in whatever environment the suite runs in.
    """
    path = os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot', 'jugglebot',
                        'motion', 'trajectory', 'cup_cycle.py')
    with open(path, encoding='utf-8') as handle:
        source = handle.read()
    for banned in ('import rclpy', 'import casadi', 'from controller',
                   'import controller', 'from sim', 'import sim.',
                   'import scipy'):
        assert banned not in source, banned
    assert 'from __future__ import annotations' in source
