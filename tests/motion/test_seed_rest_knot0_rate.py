"""I-PLAN-17: a plan seeded from rest opens at the seed's own (zero) rate (BUG-152).

WHAT THIS DEFENDS
-----------------
``cup_realize.decompose`` finite-differences the tilt series, the cup axis it
drives and the z-float with ``_knot_derivative``, whose knot-0 value is a
second-order ONE-SIDED stencil over the first three knots.  A window seeded from
rest leaves it C1 (every tilt branch opens at the seed's rate, I-PLAN-11), so its
series starts like ``c·t³`` and the stencil answers ``-2·c·dt²``: a rate of the
WRONG sign that the seed does not have.  ``CyclePlan.state_at(t <= 0)`` streams
knot 0's pose AND rate, so before the start time the emitter sent a held position
with that rate and the firmware's cubic Hermite rippled every Leg.

MEASURED (2026-11-27, BUG-152; ``scratchpad out/jp11diag/probe_d.py``): the fed
Columns hop entry's opening THROW at Sep. 125 / Apex 0.95 carried knot-0
``x -1.1305 mm/s`` and ``ry -0.01803 rad/s``; the sim gate's Leg mirror read
1.20e-4 rev against its 1.0e-4 band.  Reproduced below with a cross-site LAUNCH
from rest (``out/bug152/probe_min.py``, 2026-11-28, origin/main 47f85f31):
knot-0 ``ry`` +0.0087..+0.0214 rad/s and ``x`` -0.016..+0.039 mm/s against a
tilt series that starts 0, -0.00024, -0.00156 rad (the stencil's sign is
opposite to the motion), and exactly zero with ``seed_at_rest``.

Without the change, ``test_a_rest_seeded_plan_opens_at_zero_rate`` fails on the
three cross-site cases and ``test_the_pre_start_hold_streams_zero_rate`` fails on
the twist; the decompose-level tests fail on the unknown keyword.
"""
from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory.limits import TrajectoryLimits

REST_MM = np.array([0.0, 0.0, 750.0])


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0)


def _rest_state(cup_mm=REST_MM) -> uc.CycleState:
    cfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - cfg.cup_z_base_mm
    rev = (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def _launch(dx_mm, target_x_mm, period_s, limits, geom):
    throw = np.array([dx_mm, 0.0, 860.0])
    target = np.array([target_x_mm, 0.0, 860.0])
    goals = uc.CycleGoals(period_s=period_s, throw_site_mm=throw,
                          throw_target_mm=target, flight_s=0.8,
                          settle_site_mm=REST_MM)
    return uc.plan_launch(goals, _rest_state(), limits, geom)


# (throw-site x, target x, period): the on-site case was already zero; the
# three cross-site cases carried the stencil's rate before I-PLAN-17.
LAUNCHES = [(0.0, 0.0, 0.55), (0.0, 125.0, 0.55), (60.0, 125.0, 0.70),
            (-60.0, 60.0, 0.70)]


@pytest.mark.parametrize('dx,tgt,period', LAUNCHES)
def test_a_rest_seeded_plan_opens_at_zero_rate(dx, tgt, period, limits, geom):
    plan, meta = _launch(dx, tgt, period, limits, geom)
    assert np.max(np.abs(np.asarray(plan.pose_vel[0]))) <= 1e-9, plan.pose_vel[0]
    assert abs(float(plan.hand_vel_rps[0])) <= 1e-9
    # the knots after it still move: the fix is a boundary condition, not a hold
    if tgt != dx:
        assert abs(float(np.asarray(meta.tilts)[2, 1])) > 1e-4


def test_the_pre_start_hold_streams_zero_rate(limits, geom):
    """``state_at(t <= 0)`` is what the emitter streams before t0 (contract
    unchanged); with knot 0 at the seed's rate it is an exact hold."""
    plan, _ = _launch(60.0, 125.0, 0.70, limits, geom)
    pose, twist, _ = plan.state_at(-0.1)
    assert np.array_equal(np.asarray(pose), np.asarray(plan.pose[0]))
    assert np.max(np.abs(np.asarray(twist))) <= 1e-9, twist


def _flat_cup(n, dt, vz0=0.0):
    pos = np.zeros((n, 3)); pos[:, 2] = 0.80
    vel = np.zeros((n, 3)); vel[0, 2] = vz0
    t = np.arange(n) * dt
    return type('Cup', (), dict(pos=pos, vel=vel, acc=np.zeros((n, 3)),
                                t=t, dt=dt, catch_k=-1))()


def test_decompose_seed_at_rest_changes_knot_zero_only():
    n, dt = 8, 0.025
    tilts = np.zeros((n, 2))
    tilts[:, 1] = 0.02 * (np.arange(n) * dt) ** 3 / dt ** 3 * 1e-2   # c·t³ start
    base = cr.decompose(_flat_cup(n, dt), tilts)
    rest = cr.decompose(_flat_cup(n, dt), tilts, seed_at_rest=True)
    assert abs(base.pose_vel[0, 4]) > 0.0            # the stencil's artefact
    assert np.array_equal(rest.pose_vel[0], np.zeros(6))
    assert rest.slider_vel_rev_s[0] == 0.0
    assert np.array_equal(rest.pose_vel[1:], base.pose_vel[1:])
    assert np.array_equal(rest.slider_vel_rev_s[1:], base.slider_vel_rev_s[1:])
    assert np.array_equal(rest.pose, base.pose)


def test_seed_is_at_rest_is_exact():
    s = _rest_state()
    assert uc._seed_is_at_rest(s)
    moving = uc.CycleState(pose=s.pose, pose_vel=np.array([0, 0, 0, 0, 1e-12, 0.]),
                           pose_accel=np.zeros(6), hand_rev=s.hand_rev,
                           hand_vel_rps=0.0)
    assert not uc._seed_is_at_rest(moving)
    hand = uc.CycleState(pose=s.pose, pose_vel=np.zeros(6),
                         pose_accel=np.zeros(6), hand_rev=s.hand_rev,
                         hand_vel_rps=0.01)
    assert not uc._seed_is_at_rest(hand)
    post = uc.CycleState(pose=s.pose, pose_vel=np.zeros(6),
                         pose_accel=np.zeros(6), hand_rev=s.hand_rev,
                         hand_vel_rps=0.0, post_release=True)
    assert not uc._seed_is_at_rest(post)
