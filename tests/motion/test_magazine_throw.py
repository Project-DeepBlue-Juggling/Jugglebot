"""The MAGAZINE throw (``motion/skills/magazine_throw.py``) and its install hook.

The throw is formulaic — no cup QP — so these pin the properties that make it
safe to stream: it is gated by ``validate_cycle`` at the passed limits, it
starts and ends at rest, the hand is C2 by construction, the release the meta
announces reproduces the ballistic, the heave is used only when the hand cannot
reach, and the refusals carry their named codes through
``executor.install_segment``.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import levelling
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import magazine_throw as mt
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.trajectory import TrajectoryLimits
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import tilt_geometry as tg

G = float(ballistics_bc.GRAVITY_MMS2)
SITE = np.array([0.0, 0.0, 860.0])
REST = np.array([0.0, 0.0, uc.SETTLE_CUP_Z_MM])
#: A hand-only target: ~4 m/s, small lateral (6 deg aim).
EASY_TARGET = np.array([300.0, 100.0, 1100.0])
EASY_FLIGHT_S = 0.75
#: The real magazine (plan frame, mm) — needs the heave.
MAG_TARGET = np.array([-1256.0, -620.0, 2413.0])


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    """Today's session limits: legs 350/5000/200000, hand 3900."""
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3900.0)


def _rest_state(cup_mm, correction=None) -> uc.CycleState:
    """A stopped machine with the cup at ``cup_mm`` (level in GRAVITY)."""
    rc = cr.RealizeConfig()
    tilt = uc._tilts_to_plan(np.zeros((1, 2)), correction)[0]
    pose = np.array([cup_mm[0], cup_mm[1], rc.active_z_mm, tilt[0], tilt[1], 0.0])
    rev = uc.hand_rev_for_cup_z(float(cup_mm[2]))
    return uc.CycleState.at_rest(pose, rev, rc, levelling_correction=correction)


def _terminal(target=EASY_TARGET, flight_s=EASY_FLIGHT_S, t_release_s=1.3):
    return mt.MagazineTerminal(target_mm=target, flight_s=flight_s,
                               t_release_s=t_release_s, release_site_mm=SITE,
                               rest_site_mm=REST)


def _knot_acc_jumps(p, v, h):
    """|a_left - a_right| at every interior knot of a cubic-Hermite series."""
    p0, p1, v0, v1 = p[:-1], p[1:], v[:-1], v[1:]
    a_start = (6.0 * (p1 - p0) / h ** 2 - (4.0 * v0 + 2.0 * v1) / h)
    a_end = (-6.0 * (p1 - p0) / h ** 2 + (2.0 * v0 + 4.0 * v1) / h)
    return np.abs(a_end[:-1] - a_start[1:])


@pytest.mark.parametrize('correction', [None, 'offset'])
def test_reachable_throw_passes_the_gate_rest_to_rest(limits, geom, correction):
    corr = (levelling.correction_from_offset(0.0117, -0.004)
            if correction else None)
    seed = _rest_state(REST, corr)
    seg = mt.plan_magazine(seed, _terminal(), sg.SegmentConfig(), limits, geom)
    plan, meta = seg.plan, seg.meta
    assert seg.kind == mt.MAGAZINE
    assert meta.report.ok
    # Starts exactly at the seed, at rest.
    np.testing.assert_array_equal(plan.pose[0], seed.pose)
    assert plan.hand_rev[0] == seed.hand_rev
    assert np.max(np.abs(plan.pose_vel[0])) == 0.0 and plan.hand_vel_rps[0] == 0.0
    # Ends at rest: zero velocity, level in gravity, hand parked at the rest site.
    assert np.max(np.abs(plan.pose_vel[-1])) < 1e-9
    assert abs(plan.hand_vel_rps[-1]) < 1e-9
    end_tilt_g = uc._tilt_to_gravity(plan.pose[-1, 3:5], corr)
    assert np.max(np.abs(end_tilt_g)) < 1e-9
    np.testing.assert_allclose(
        mt.rigid_cup_state(plan.pose[-1], plan.hand_rev[-1], [0.0, 0.0, 1.0]),
        REST, atol=1e-6)
    # C2 hand (and pose) by construction: no acceleration jump at any knot.
    h = float(plan.dt)
    assert _knot_acc_jumps(plan.hand_rev, plan.hand_vel_rps, h).max() < 1e-6
    for i in range(3):
        assert _knot_acc_jumps(plan.pose[:, i], plan.pose_vel[:, i], h).max() < 1e-6
    # The release tilt is the gravity-referenced aim, re-expressed into the plan
    # frame the way unified_cycle._realize does it (``_tilts_to_plan``).  The
    # round trip is exact only to the dropped second-order ``rz`` term
    # (``_tilts_to_plan`` docstring: worst axis change 1.276e-4 rad over the
    # whole regime), so that is the tolerance, not float precision.
    k = int(round(seg.event_t_s / h))
    np.testing.assert_allclose(uc._tilt_to_gravity(plan.pose[k, 3:5], corr),
                               meta.throw_tilt, atol=1.3e-4)
    if corr is not None:   # ... and it is NOT the uncorrected tilt (0.67 deg off)
        assert np.max(np.abs(plan.pose[k, 3:5] - meta.throw_tilt)) > 3e-3


def _landing(site, vel, T):
    return site + vel * T + 0.5 * np.array([0.0, 0.0, -G]) * T * T


def test_release_in_meta_reproduces_the_ballistic(limits, geom):
    """The announced release is the RIGID-body cup's (the IK rotates about the
    platform centre, which rides with the heave) and it lands on the target."""
    seg = mt.plan_magazine(_rest_state(REST), _terminal(), sg.SegmentConfig(),
                           limits, geom)
    rel = seg.meta.releases[-1]
    assert seg.event_t_s == rel.t_s == seg.meta.t_release_s
    np.testing.assert_array_equal(seg.takeoff_vel_mm_s, rel.vel_mm_s)
    assert np.linalg.norm(_landing(rel.site_mm, rel.vel_mm_s, rel.flight_s)
                          - EASY_TARGET) < 1.0
    # ... and the release really is ON the plan, through the rigid cup.
    axis = tg.cup_axis(*rel.tilt)
    pose, twist, _ = seg.plan.state_at(rel.t_s)
    rev, rps = seg.plan.hand_at(rel.t_s)
    np.testing.assert_allclose(mt.rigid_cup_state(pose, rev, axis),
                               rel.site_mm, atol=1e-6)
    np.testing.assert_allclose(mt.rigid_cup_velocity(twist, rps, axis),
                               rel.vel_mm_s, atol=1e-6)
    np.testing.assert_allclose(rel.site_mm, SITE, atol=1e-6)
    # The requested release instant is honoured to within half a knot.
    assert abs(rel.t_s - 1.3) <= 0.5 * float(seg.plan.dt) + 1e-9


@pytest.mark.parametrize('target, flight_s', [
    (np.array([200.0, 0.0, 2400.0]), 0.9),     # heave along a 1.6 deg axis
    (MAG_TARGET, 0.9),                          # heave + the 12 deg cap's lean
])
def test_heave_assisted_rigid_release_hits_the_target(limits, target, flight_s):
    """At nonzero heave the rigid release still lands within 1 mm, and the
    repo map's (fixed tilt centre) disagreement is reported, not designed in."""
    d = mt.design_magazine(_rest_state(REST), _terminal(target, flight_s), limits)
    assert d.heave_mm_s > 100.0
    assert np.linalg.norm(_landing(d.release_site_mm, d.release_vel_mm_s,
                                   flight_s) - target) < 1.0
    np.testing.assert_allclose(d.release_vel_mm_s, d.design_vel_mm_s, atol=1e-6)
    gap = d.repo_minus_rigid_vel_mm_s
    assert gap.shape == (3,) and np.all(np.isfinite(gap))
    assert math.isfinite(d.repo_minus_rigid_landing_mm)
    assert d.repo_minus_rigid_landing_mm == pytest.approx(np.linalg.norm(
        _landing(d.release_site_mm + d.repo_minus_rigid_site_mm,
                 d.release_vel_mm_s + gap, flight_s) - target))
    if d.tilt_deg > 5.0:
        # Heave at a real tilt: the fixed-centre map reads it as axial slider
        # travel, ~a_xy * u_z of lateral velocity off the rigid truth.
        assert np.linalg.norm(gap[:2]) > 50.0


def test_heave_only_when_the_hand_cannot_reach(limits):
    easy = mt.design_magazine(_rest_state(REST), _terminal(), limits)
    assert easy.hand_rel_rps < easy.hand_cap_rps
    assert easy.heave_mm_s == pytest.approx(0.0, abs=1e-9)
    hard = mt.design_magazine(_rest_state(REST), _terminal(MAG_TARGET, 0.9),
                              limits)
    assert hard.hand_rel_rps == pytest.approx(hard.hand_cap_rps, rel=1e-9)
    assert hard.heave_mm_s > 100.0
    assert hard.tilt_deg <= 12.0 + 1e-9


def test_real_magazine_target_refuses_at_session_limits(limits, geom):
    with pytest.raises(uc.CycleInfeasible) as err:
        mt.plan_magazine(_rest_state(REST), _terminal(MAG_TARGET, 0.9),
                         sg.SegmentConfig(), limits, geom)
    code = err.value.code
    assert code in ('WORKSPACE', 'LIMIT_VEL', 'LIMIT_ACC', 'LIMIT_JERK',
                    'STEP_BOUND', 'TOO_FAST'), code
    assert err.value.outcome().startswith(uc.OUTCOME_CODE + '(' + code)


def test_aim_past_the_tilt_ceiling_refuses(limits, geom):
    # 2 m sideways in 0.5 s: ~4 m/s of lateral against ~2.6 m/s of vertical.
    far = np.array([2000.0, 0.0, 900.0])
    with pytest.raises(uc.CycleInfeasible) as err:
        mt.plan_magazine(_rest_state(REST), _terminal(far, 0.5),
                         sg.SegmentConfig(), limits, geom)
    assert err.value.code == mt.MAGAZINE_TILT


def test_moving_seed_and_early_release_refuse(limits, geom):
    seed = _rest_state(REST)
    moving = uc.CycleState(pose=seed.pose, pose_vel=np.array([0, 0, 10.0, 0, 0, 0]),
                           pose_accel=np.zeros(6), hand_rev=seed.hand_rev,
                           hand_vel_rps=0.0)
    with pytest.raises(uc.CycleInfeasible) as err:
        mt.plan_magazine(moving, _terminal(), sg.SegmentConfig(), limits, geom)
    assert err.value.code == mt.MAGAZINE_NOT_AT_REST
    with pytest.raises(uc.CycleInfeasible) as err:
        mt.plan_magazine(seed, _terminal(t_release_s=0.3), sg.SegmentConfig(),
                         limits, geom)
    assert err.value.code == mt.MAGAZINE_WINDOW


def test_install_segment_fresh_origin_only(limits, geom):
    t_now = 50.0
    term = _terminal(t_release_s=t_now + 1.3)          # ABSOLUTE, like THROW
    rec, res, seg = ex.install_segment(None, _rest_state(REST), mt.MAGAZINE,
                                       term, t_now, limits=limits, geom=geom)
    assert res.accepted, res.message
    assert seg.kind == mt.MAGAZINE and res.splice_k == 0
    assert res.event_t_s == pytest.approx(seg.meta.t_release_s)
    assert abs(rec.t0_s + res.event_t_s - (t_now + 1.3)) <= 0.0125 + 1e-9
    # The same request while that plan is still moving is refused, untouched.
    rec2, res2, seg2 = ex.install_segment(rec, None, mt.MAGAZINE,
                                          _terminal(t_release_s=t_now + 2.5),
                                          t_now + 0.5, limits=limits, geom=geom)
    assert not res2.accepted and res2.code == mt.MAGAZINE_NEEDS_REST
    assert rec2 is rec and seg2 is None
    # ... and NOTHING splices onto a running MAGAZINE record (its meta has no
    # cup_plan for splice_at): a REST inside it refuses the same way.
    rec3, res3r, seg3 = ex.install_segment(
        rec, None, sg.REST, sg.RestTerminal(rest_site_mm=REST,
                                            t_rest_s=t_now + 2.5),
        t_now + 0.5, limits=limits, geom=geom)
    assert not res3r.accepted and res3r.code == mt.MAGAZINE_NEEDS_REST
    assert rec3 is rec and seg3 is None
    assert rec.meta.kind == mt.MAGAZINE
    # Once it has ended, a fresh-origin install from its terminal rest is fine.
    _, res4, _ = ex.install_segment(
        rec, None, sg.REST, sg.RestTerminal(rest_site_mm=REST,
                                            t_rest_s=rec.end_s + 1.0),
        rec.end_s + 0.1, limits=limits, geom=geom)
    assert res4.accepted, res4.message
    # A planner refusal surfaces as a refusing InstallResult, never an exception.
    _, res3, _ = ex.install_segment(None, _rest_state(REST), mt.MAGAZINE,
                                    _terminal(MAG_TARGET, 0.9, t_now + 1.3),
                                    t_now, limits=limits, geom=geom)
    assert not res3.accepted and res3.code != 'OK'
    assert math.isfinite(res3.plan_wall_s)
