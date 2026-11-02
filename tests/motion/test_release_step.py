"""C-HAND-5, the release step: the hand's acceleration may STEP at a release knot.

The cup QP used to pin ``acc == g`` at the release (the CasADi transcription),
which on the 25 ms knot grid makes the launch ramp from +A to -g over its last
knot and the brake ramp from -g to -B over its first: both peaks ran 1.5-1.85x
the constant-acceleration ideal and capped the Apex near 1.1 m at the old 3900
rev/s^2 hand limit, 1.3 m at 4500 (R0, logbook ``2026-11-02-hand-release-step``).  With the step the launch keeps its
acceleration to the release, the next window starts from the brake seed the
release mark carries, and ``validate_cycle`` admits the step at that knot only.

Every plan here is a real QP + gate (~0.2-0.5 s); fixtures are module-scoped.
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as fz
from jugglebot.motion.trajectory.cycle_plan import (
    CyclePlan, shift_release_steps)
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si

HAND_ACC = 4500.0
REST_MM = np.array([0.0, 0.0, 750.0])
THROW_SITE_MM = np.array([0.0, 0.0, si.RELEASE_CUP_Z_MM])
G = ballistics_bc.GRAVITY_MMS2 / 1000.0


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=HAND_ACC)


def _rest_state(cup_mm):
    rcfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - rcfg.cup_z_base_mm
    rev = ((slider_mm - rcfg.slider_rev_zero_mm) / 1000.0) * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, rcfg)


def _throw(apex_m, limits, geom, target_mm=None, step=True):
    """A self-throw THROW segment from REST (launch 0.4 s + SETTLE tail)."""
    target = THROW_SITE_MM if target_mm is None else np.asarray(target_mm, float)
    term = sg.ThrowTerminal(site_mm=THROW_SITE_MM, target_mm=target,
                            flight_s=sc.flight_s(apex_m), t_release_s=0.4)
    old = uc.RELEASE_ACC_STEP
    uc.RELEASE_ACC_STEP = step
    try:
        return sg.plan_segment(sg.THROW, _rest_state(REST_MM), term,
                               sg.SegmentConfig(), limits, geom)
    finally:
        uc.RELEASE_ACC_STEP = old


def _hermite_acc(plan):
    """``(a_right, a_left)`` of the hand at each knot (right span s=0, left s=1)."""
    p, v, h = plan.hand_rev, plan.hand_vel_rps, plan.dt
    a_right = (-6 * p[:-1] + 6 * p[1:]) / h ** 2 + (-4 * v[:-1] - 2 * v[1:]) / h
    a_left = (6 * p[:-1] - 6 * p[1:]) / h ** 2 + (2 * v[:-1] + 4 * v[1:]) / h
    return a_right, a_left


def _kappas(seg):
    """Launch and brake peaks over their constant-acceleration ideals."""
    plan = seg.plan
    k = int(round(float(seg.event_t_s) / plan.dt))
    ar, al = _hermite_acc(plan)
    p, v = plan.hand_rev, plan.hand_vel_rps
    launch_pk = float(np.max(np.r_[ar[:k], al[:k]]))
    launch_ideal = v[k] ** 2 / (2.0 * (p[k] - p[:k].min()))
    k_stop = k + int(np.argmax(v[k:] <= 0.5))
    brake_pk = float(-np.min(np.r_[ar[k:k_stop], al[k:k_stop]]))
    brake_ideal = v[k] ** 2 / (2.0 * (p[k_stop] - p[k]))
    return launch_pk / launch_ideal, brake_pk / brake_ideal, k


@pytest.fixture(scope='module')
def stepped(limits, geom):
    return _throw(1.05, limits, geom, step=True)


@pytest.fixture(scope='module')
def continuous(limits, geom):
    return _throw(1.05, limits, geom, step=False)


def test_the_continuous_release_is_unchanged_with_the_step_off(continuous):
    """Off: no brake on the mark, no declared knot, the seed is ``g``."""
    mark = continuous.meta.releases[0]
    assert mark.brake_accel_mm_s2 is None
    assert continuous.plan.release_step_knots is None
    assert np.array_equal(continuous.release_state.cup_accel_mm_s2,
                          uc._G_MM_S2)


def test_a_stepped_release_lowers_both_peaks_toward_the_ideal(stepped,
                                                              continuous):
    """MEASURED 2026-11-02 on a THROW from REST (launch + SETTLE tail; Apex
    1.05, release 860, hand 4500): continuous launch / brake kappa 1.81 / 1.56,
    stepped 1.31 / 1.10; whole-plan hand peak 3270 -> 2366 rev/s^2.  (The
    logbook's 3484 -> 2362 is the folded STEADY window.)  Bounds leave room for
    solver drift."""
    kl, kb, _ = _kappas(stepped)
    assert kl <= 1.40, kl
    assert kb <= 1.15, kb
    kl0, kb0, _ = _kappas(continuous)
    assert kl0 >= 1.70 and kb0 >= 1.45, (kl0, kb0)     # not vacuous
    assert (stepped.meta.report.peak_hand_acc_rps2
            <= 0.8 * continuous.meta.report.peak_hand_acc_rps2)


def test_the_stepped_release_declares_its_knot_and_seeds_the_brake(stepped, limits,
                                                                  geom):
    mark = stepped.meta.releases[0]
    k = int(round(float(mark.t_s) / stepped.plan.dt))
    assert stepped.plan.release_step_knots == (k,)
    assert mark.brake_accel_mm_s2 is not None
    assert np.array_equal(stepped.release_state.cup_accel_mm_s2,
                          mark.brake_accel_mm_s2)
    ar, al = _hermite_acc(stepped.plan)
    # the ball is pressed in up to the release, then the cup drops away
    assert al[k - 1] > 0.0
    assert ar[k] < -2.0 * G * cr.HAND_REV_PER_M
    assert fz.validate_cycle(stepped.plan, limits, geom).ok


def stepped_limits():
    return TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=HAND_ACC)


def test_the_ceiling_apex_plans_inside_the_hand_limit(limits, geom):
    """Apex 1.42 (the ceiling target, 1.45 above the catch plane), THROW from
    REST: refused HAND_LIMIT_ACC continuous (4859 rev/s^2 measured), planned
    stepped (3137)."""
    seg = _throw(1.42, limits, geom, step=True)
    assert seg.meta.report.ok
    assert seg.meta.report.peak_hand_acc_rps2 <= 3400.0
    with pytest.raises(uc.CycleInfeasible) as exc:
        _throw(1.42, limits, geom, step=False)
    assert exc.value.code == fz.HAND_LIMIT_ACC


def test_a_tilted_stepped_release_steps_along_the_takeoff_axis(limits, geom):
    """The step is a SLIDER step: ``a_R - a_L`` and ``a_R - g`` are both along
    the take-off direction, so the platform's acceleration is continuous (a
    vertical-only step on a tilted cup cost 200 k mm/s^3 of leg jerk) and the
    detach cone already holds at knot 0."""
    seg = _throw(0.95, limits, geom, target_mm=THROW_SITE_MM + [60.0, 0.0, 0.0])
    mark = seg.meta.releases[0]
    u = np.asarray(mark.vel_mm_s, float)
    u = u / np.linalg.norm(u)
    assert abs(u[0]) > 0.01                            # genuinely tilted
    k = int(round(float(mark.t_s) / seg.plan.dt))
    a_l = np.asarray(seg.meta.cup_plan.acc[k], float) * 1000.0   # launch end
    a_r = np.asarray(mark.brake_accel_mm_s2, float)
    for d in (a_r - a_l, a_r - uc._G_MM_S2, a_l - uc._G_MM_S2):
        assert np.linalg.norm(np.cross(d, u)) <= 1e-6 * np.linalg.norm(d)


def _synthetic(step_sign, declared):
    """A 9-knot hand-only plan whose acceleration steps by ``step_sign·400`` at
    knot 4, velocity-continuous (the platform holds still at the rest pose)."""
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    rcfg = cr.RealizeConfig()
    pose = np.tile([0.0, 0.0, rcfg.active_z_mm, 0.0, 0.0, 0.0], (9, 1))
    a = np.where(np.arange(9) < 4, -200.0 * step_sign, 200.0 * step_sign)
    t = np.arange(9) * dt
    p = np.empty(9)
    v = np.empty(9)
    p0, v0 = 5.0, 0.0
    for i in range(9):
        if i <= 4:
            v[i] = v0 + a[0] * t[i]
            p[i] = p0 + v0 * t[i] + 0.5 * a[0] * t[i] ** 2
        else:
            tt = t[i] - t[4]
            v[i] = v[4] + a[8] * tt
            p[i] = p[4] + v[4] * tt + 0.5 * a[8] * tt ** 2
    return CyclePlan(pose, np.zeros((9, 6)), p, v, dt,
                     release_step_knots=declared)


@pytest.mark.parametrize('declared', [None, (3,), (5,)])
def test_an_undeclared_step_is_refused_hand_limit_c2(declared):
    rep = fz.validate_cycle(_synthetic(-1, declared), stepped_limits(),
                            StewartGeometry())
    assert rep.code == fz.HAND_LIMIT_C2, rep.reasons


def test_a_downward_step_at_a_declared_release_passes():
    rep = fz.validate_cycle(_synthetic(-1, (4,)), stepped_limits(),
                            StewartGeometry())
    assert rep.ok, rep.reasons


def test_an_upward_step_at_a_declared_release_is_refused():
    rep = fz.validate_cycle(_synthetic(+1, (4,)), stepped_limits(),
                            StewartGeometry())
    assert rep.code == fz.HAND_LIMIT_C2
    assert 'upward' in rep.reasons[0]


def test_release_step_knots_shift_and_clip_to_interior_knots():
    assert shift_release_steps(None, 3, 10) is None
    assert shift_release_steps((4, 2, 4), 0, 10) == (2, 4)
    assert shift_release_steps((4,), -4, 10) is None      # now knot 0: an end
    assert shift_release_steps((4,), -2, 7) == (2,)
    assert shift_release_steps((8,), 0, 9) is None        # the last knot


def test_the_brake_seed_is_clamped_to_the_stream_stop_and_to_two_g(limits):
    """Vertical take-off: the seed is ``(0, 0, -B)``.  A fast release clamps at
    the smaller of the hand limit and the firmware STOP deceleration (a stall
    mid-brake hands the hand to that stop); a slow one at 2 g (below 1 g the
    cup would carry the ball on)."""
    up = np.array([0.0, 0.0, 1.0])
    a_l = np.array([0.0, 0.0, 30000.0])
    fast = uc.release_brake_accel_mm_s2(50000.0 * up, a_l, 860.0, 984.0, limits)
    cap = min(HAND_ACC, float(hw.JB_TRAJ_STREAM_STOP_HAND_ACCEL_RPS2))
    assert np.allclose(fast, [0.0, 0.0, -cap / cr.HAND_REV_PER_M * 1000.0])
    slow = uc.release_brake_accel_mm_s2(100.0 * up, a_l, 860.0, 984.0, limits)
    assert np.allclose(slow, [0.0, 0.0, -uc.RELEASE_BRAKE_MIN_G * G * 1000.0])
    # Tilted: the cap is the SLIDER's (the step runs along u), so the vertical
    # brake is capped at A·u_z, i.e. a slider deceleration of A.
    tilt = np.array([np.sin(0.2), 0.0, np.cos(0.2)])
    a_lt = uc._G_MM_S2 + 30000.0 * tilt                  # (a_L - g) along u
    fast_t = uc.release_brake_accel_mm_s2(50000.0 * tilt, a_lt, 860.0, 984.0, limits)
    assert np.isclose(-fast_t[2], cap / cr.HAND_REV_PER_M * 1000.0 * tilt[2])
    assert np.linalg.norm(np.cross(fast_t - uc._G_MM_S2, tilt)) < 1e-6 * 1e5
    v = 4500.0
    mid = uc.release_brake_accel_mm_s2(v * up, a_l, 860.0, 984.0, limits)
    assert np.allclose(mid, [0.0, 0.0, -uc.RELEASE_BRAKE_FACTOR * v * v
                             / (2.0 * 124.0)])


def test_a_stepped_chain_declares_every_seam_and_seeds_each_window(limits, geom):
    """LAUNCH -> STEADY -> STEADY through ``extend``: each release seam is a
    declared step knot, each window after a release starts from that release's
    brake seed (its cup track's knot-0 acceleration), and the joined plan
    passes the gate (the ring tests in ``test_unified_cycle.py`` pin the
    continuous shape)."""
    throw = THROW_SITE_MM
    catch = np.array([20.0, 0.0, 830.0])

    def goals(period):
        return uc.CycleGoals(period_s=period, throw_site_mm=throw,
                             throw_target_mm=throw, flight_s=0.6,
                             catch_site_mm=catch,
                             catch_vel_mm_s=np.array([100.0, -50.0, -2500.0]),
                             catch_frac=0.55, settle_site_mm=REST_MM)
    p0, m0 = uc.plan_launch(goals(0.6), _rest_state(REST_MM), limits, geom)
    seed1 = uc.release_state_from_meta(m0, p0)
    p1, m1 = uc.plan_steady(goals(1.4), seed1, limits, geom)
    j1, jm1 = uc.extend(p0, m0, p1, m1, limits, geom)
    seed2 = uc.release_state_from_meta(m1, p1)
    p2, m2 = uc.plan_steady(goals(1.4), seed2, limits, geom)
    j2, jm2 = uc.extend(j1, jm1, p2, m2, limits, geom)
    k1 = int(p0.n_knots) - 1
    k2 = k1 + int(p1.n_knots) - 1
    assert j1.release_step_knots == (k1,)
    assert j2.release_step_knots == (k1, k2)
    for mark, meta in ((m0.releases[-1], m1), (m1.releases[-1], m2)):
        assert mark.brake_accel_mm_s2 is not None
        assert np.allclose(np.asarray(meta.cup_plan.acc[0]) * 1000.0,
                           mark.brake_accel_mm_s2, atol=1e-6)
    assert jm2.report.ok
    assert fz.validate_cycle(j2, limits, geom).ok
    ar, al = _hermite_acc(j2)
    for k in (k1, k2):
        assert ar[k] < al[k - 1] - 100.0        # a real, downward step
