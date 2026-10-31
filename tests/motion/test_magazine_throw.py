"""The MAGAZINE throw (``motion/skills/magazine_throw.py``) and its install hook.

The throw is formulaic — no cup QP — so these pin the properties that make it
safe to stream: it is gated by ``validate_cycle`` at the passed limits, it
starts and ends at rest, the platform is STATIONARY through the stroke (no
heave — owner decision 2026-10-13), the hand is C2 by construction, the
release the meta announces reproduces the ballistic, and the refusals carry
their named codes (with the operator advice) through
``executor.install_segment``.
"""
from __future__ import annotations

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
#: Near the hand cap, at the tilt ceiling (probe 2026-10-13: 5.16 m/s, 11.9 deg).
HARD_TARGET = np.array([-754.0, -372.0, 1792.0])
HARD_FLIGHT_S = 0.79
#: The real magazine (plan frame, mm): out of the hand's reach at any flight.
MAG_TARGET = np.array([-1256.0, -620.0, 2413.0])


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    """Today's sitting limits: legs 350/5000/200000, hand 3900."""
    return TrajectoryLimits.from_config(hw).with_sitting_limits(
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


def _landing(site, vel, T):
    return site + vel * T + np.array([0.0, 0.0, -0.5 * G * T * T])


@pytest.mark.parametrize('correction', [None, 'offset'])
def test_reachable_throw_passes_the_gate_rest_to_rest(limits, geom, correction):
    corr = (levelling.correction_from_offset(0.0117, -0.004)
            if correction else None)
    seed = _rest_state(REST, corr)
    seg = mt.plan_magazine(seed, _terminal(), sg.SegmentConfig(), limits, geom)
    assert seg.kind == mt.MAGAZINE and seg.meta.kind == mt.MAGAZINE
    assert seg.meta.report.ok
    plan = seg.plan
    # Knot 0 is the seed's own float; the end is a stopped, level rest.
    np.testing.assert_allclose(plan.pose[0], seed.pose)
    assert plan.hand_rev[0] == pytest.approx(seed.hand_rev)
    assert np.all(np.abs(plan.pose_vel[-1]) < 1e-9)
    assert abs(plan.hand_vel_rps[-1]) < 1e-9
    assert np.max(np.abs(uc._tilt_to_gravity(plan.pose[-1, 3:5], corr))) < 1e-9
    # The release tilt is the gravity-referenced aim in the plan frame
    # (`_tilts_to_plan`; exact to the dropped second-order rz term).
    k = int(round(seg.event_t_s / float(plan.dt)))
    np.testing.assert_allclose(uc._tilt_to_gravity(plan.pose[k, 3:5], corr),
                               seg.meta.throw_tilt, atol=1.3e-4)
    if corr is not None:
        assert np.max(np.abs(plan.pose[k, 3:5] - seg.meta.throw_tilt)) > 3e-3
    np.testing.assert_allclose(plan.pose[-1, :2], REST[:2], atol=1e-6)
    assert plan.pose[-1, 2] == pytest.approx(cr.RealizeConfig().active_z_mm)
    assert plan.hand_rev[-1] == pytest.approx(uc.hand_rev_for_cup_z(REST[2]), abs=1e-6)
    # C2 by construction, on every channel.
    h = float(plan.dt)
    for ch in range(6):
        assert np.max(_knot_acc_jumps(plan.pose[:, ch], plan.pose_vel[:, ch], h)) < 1e-6
    assert np.max(_knot_acc_jumps(plan.hand_rev, plan.hand_vel_rps, h)) < 1e-6
    # The hand stroke stays inside its clip, with the top margin.
    assert np.max(plan.hand_rev) < float(mt.fz.HAND_STROKE_MAX_REV)
    # The release instant is the segment's event, segment-relative.
    assert seg.event_t_s == pytest.approx(seg.meta.releases[-1].t_s)
    assert seg.event_t_s == pytest.approx(1.3, abs=0.03)


def test_the_platform_is_stationary_through_the_stroke(limits):
    """No heave (owner, 2026-10-13): every pose channel is at rest from the
    stroke's start to the post-release hold; the hand alone makes the speed."""
    d = mt.design_magazine(_rest_state(REST), _terminal(HARD_TARGET, HARD_FLIGHT_S),
                           limits)
    plan = d.plan
    h = float(plan.dt)
    k_t = d.k_release
    # Find the stroke start: the last knot before the release where the hand is still.
    k_hs = max(k for k in range(k_t) if abs(plan.hand_vel_rps[k]) < 1e-12)
    k_end = k_t + 1 + uc.post_release_hold_knots()
    assert np.all(np.abs(plan.pose_vel[k_hs:k_end + 1]) < 1e-9)
    assert np.all(np.abs(np.diff(plan.pose[k_hs:k_end + 1], axis=0)) < 1e-9)
    assert d.hand_rel_rps * mt.MM_PER_REV == pytest.approx(
        np.linalg.norm(d.design_vel_mm_s), rel=1e-9)
    assert d.hand_rel_rps <= d.hand_cap_rps * (1.0 + 1e-9)
    assert d.tilt_deg <= 12.0 + 1e-9
    assert d.t_release_s == pytest.approx(k_t * h, abs=h)


def test_release_in_meta_reproduces_the_ballistic(limits, geom):
    seg = mt.plan_magazine(_rest_state(REST), _terminal(), sg.SegmentConfig(),
                           limits, geom)
    rel = seg.meta.releases[-1]
    land = _landing(rel.site_mm, rel.vel_mm_s, rel.flight_s)
    assert np.linalg.norm(land - EASY_TARGET) < 1.0
    np.testing.assert_allclose(rel.target_mm, EASY_TARGET)
    np.testing.assert_allclose(seg.takeoff_vel_mm_s, rel.vel_mm_s)
    # The release site is the requested one (the platform sits at the active
    # z with the hand bottom chosen for it), and the release is mid-span at
    # the instant the cup's axial acceleration reaches -g a_z.
    assert np.linalg.norm(rel.site_mm - SITE) < 1.0
    _, rps = seg.plan.hand_at(rel.t_s)
    assert rps * mt.MM_PER_REV == pytest.approx(np.linalg.norm(rel.vel_mm_s), rel=1e-9)


@pytest.mark.parametrize('target, flight_s', [
    (EASY_TARGET, EASY_FLIGHT_S), (HARD_TARGET, HARD_FLIGHT_S),
])
def test_rigid_release_agrees_with_the_repo_cup_map_to_its_model_gap(limits, target,
                                                                     flight_s):
    """The meta carries the RIGID cup (the IK's truth); the repo map's
    ``cup_velocity_from_platform`` is a different model (its lateral term is
    ``a_xy / (2 - a_z)``, ~2 % under at 12 deg). With the platform stationary
    the two agree to that model gap — nothing larger hides in the knots."""
    d = mt.design_magazine(_rest_state(REST), _terminal(target, flight_s), limits)
    pose_r, twist_r, _ = d.plan.state_at(d.t_release_s)
    rev_r, rps_r = d.plan.hand_at(d.t_release_s)
    assert np.all(np.abs(twist_r) < 1e-9)
    v_repo = uc.cup_velocity_from_platform(pose_r, twist_r, rev_r, rps_r)
    s_repo = uc.cup_state_from_platform(pose_r, rev_r)
    speed = np.linalg.norm(d.release_vel_mm_s)
    assert np.linalg.norm(v_repo - d.release_vel_mm_s) < 0.006 * speed
    assert np.linalg.norm(s_repo - d.release_site_mm) < 1.0


def test_unreachable_target_refuses_with_the_speed_and_least_speed_flight(limits, geom):
    with pytest.raises(uc.CycleInfeasible) as err:
        mt.plan_magazine(_rest_state(REST), _terminal(MAG_TARGET, 1.1),
                         sg.SegmentConfig(), limits, geom)
    assert err.value.code == mt.MAGAZINE_UNREACHABLE
    text = err.value.reasons[0]
    assert 'hand cap is 5.3' in text and 'least-speed flight' in text
    assert err.value.outcome().startswith(uc.OUTCOME_CODE + '(' + mt.MAGAZINE_UNREACHABLE)


def test_aim_past_the_tilt_ceiling_refuses_naming_a_flight_that_fits(limits, geom):
    with pytest.raises(uc.CycleInfeasible) as err:
        mt.plan_magazine(_rest_state(REST), _terminal(MAG_TARGET, 0.9),
                         sg.SegmentConfig(), limits, geom)
    assert err.value.code == mt.MAGAZINE_TILT
    assert 'a flight of 1.02 s aims inside it' in err.value.reasons[0]
    # 2 m sideways in 0.5 s: no flight time brings it inside 12 deg from here.
    best = mt.least_speed_flight(SITE, np.array([2000.0, 0.0, 900.0]))
    assert best is not None and best[0] > 1.3   # only a long, slow lob fits


def test_least_speed_flight_is_the_slowest_take_off_inside_the_ceiling():
    T, speed = mt.least_speed_flight(SITE, EASY_TARGET)
    v = mt.takeoff_velocity(SITE, EASY_TARGET, T)
    assert speed == pytest.approx(np.linalg.norm(v))
    assert mt.aim_deg(v) <= 12.0
    for T2 in (T - 0.1, T + 0.1):
        v2 = mt.takeoff_velocity(SITE, EASY_TARGET, T2)
        if mt.aim_deg(v2) <= 12.0:
            assert np.linalg.norm(v2) >= speed - 1e-9


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


def test_gate_refusals_carry_their_own_code(limits, geom):
    """A shape the hand can make but the legs cannot follow at the sitting
    limits is refused by ``validate_cycle``'s own code, never flown."""
    tight = TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=60.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3900.0)
    with pytest.raises(uc.CycleInfeasible) as err:
        mt.plan_magazine(_rest_state(REST), _terminal(HARD_TARGET, HARD_FLIGHT_S),
                         sg.SegmentConfig(), tight, geom)
    assert err.value.code in ('LIMIT_VEL', 'LIMIT_ACC', 'LIMIT_JERK', 'STEP_BOUND',
                              'WORKSPACE', 'TOO_FAST'), err.value.code


def test_install_segment_fresh_origin_only(limits, geom):
    t_now = 50.0
    term = _terminal(t_release_s=t_now + 1.3)          # ABSOLUTE, like THROW
    rec, res, seg = ex.install_segment(None, _rest_state(REST), mt.MAGAZINE,
                                       term, t_now, limits=limits, geom=geom)
    assert res.accepted, res.message
    assert seg.kind == mt.MAGAZINE and res.splice_k == 0
    assert res.event_t_s == pytest.approx(seg.meta.t_release_s)
    # Any splice onto a MAGAZINE record is refused: its meta has no cup plan.
    rec2, res2, _ = ex.install_segment(rec, None, sg.REST,
                                       sg.RestTerminal(rest_site_mm=REST, t_rest_s=t_now + 3.0),
                                       t_now + 0.5, limits=limits, geom=geom)
    assert not res2.accepted and res2.code == mt.MAGAZINE_NEEDS_REST
