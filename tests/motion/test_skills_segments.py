"""``motion/skills/segments`` — one skill as a rest-terminal ``CyclePlan``
(plan § 2.2).

Every ``plan_segment`` call here solves a real QP + gate, so fixtures are
module-scoped (plan § 0's "keep planning to a handful of solves" instruction)
and each is ~0.1-0.4 s. Unmarked and parallel-safe: no filesystem, no shared
clock, no wall-clock assertion (that shape is ``test_unified_cycle_budget.py``,
which is ``serial`` for that reason).

Plan: ``plans/active/two-ball-skill-stack.md`` § 2.2.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import ballistics_bc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si

#: The owner's R2 operating point (plan § 0, 2026-09-12): leg 300/5000/200000
#: mm(/s, /s^2, /s^3), hand acc 3500 rev/s^2 -- the same limits the rest_tail_s
#: probe (see segments.REST_TAIL_S) used.
LEG_VEL = 300.0
LEG_ACC = 5000.0
LEG_JERK = 200000.0
HAND_ACC = 3500.0

#: 0.9 m apex -> the owner's t_f = 0.857 s (plan § 0), full precision.
T_F = 2.0 * (2.0 * 0.9 / (ballistics_bc.GRAVITY_MMS2 / 1000.0)) ** 0.5
TRANSIT_S = (T_F - 0.30) / 2.0

REST_MM = np.array([0.0, 0.0, 750.0])
THROW_SITE_MM = np.array([0.0, 0.0, 860.0])
CATCH_SITE_MM = np.array([0.0, 0.0, 830.0])
#: The fixed 40 Hz knot grid — a segment's events land on it, so a scheduled
#: instant and the realised one differ by up to a knot.
DT = float(hw.JB_TRAJ_KNOT_DT_S)


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=LEG_VEL, leg_acc_mmps2=LEG_ACC, leg_jerk_mmps3=LEG_JERK,
        hand_acc_rps2=HAND_ACC)


@pytest.fixture(scope='module')
def cfg():
    return sg.SegmentConfig()


def _rest_state(cup_mm):
    rcfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - rcfg.cup_z_base_mm
    rev = ((slider_mm - rcfg.slider_rev_zero_mm) / 1000.0) * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, rcfg)


@pytest.fixture(scope='module')
def throw_segment(limits, geom, cfg):
    """A THROW from rest: LAUNCH (0.4 s) then the SETTLE tail."""
    seed = _rest_state(REST_MM)
    terminal = sg.ThrowTerminal(site_mm=THROW_SITE_MM, target_mm=THROW_SITE_MM,
                                flight_s=T_F, t_release_s=0.4)
    return sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)


@pytest.fixture(scope='module')
def catch_seed(limits, geom):
    """The post-release state a CATCH plans from: a real LAUNCH's release."""
    seed = _rest_state(REST_MM)
    goals = uc.CycleGoals(period_s=0.4, throw_site_mm=THROW_SITE_MM,
                          throw_target_mm=THROW_SITE_MM, flight_s=T_F)
    plan_a, meta_a = uc.plan_launch(goals, seed, limits, geom)
    return uc.release_state_from_meta(meta_a, plan_a)


@pytest.fixture(scope='module')
def catch_terminal():
    v_arrival = np.array([0.0, 0.0, -ballistics_bc.GRAVITY_MMS2 * (T_F / 2.0)])
    return sg.CatchTerminal(landing_mm=CATCH_SITE_MM, landing_vel_mm_s=v_arrival,
                            t_land_s=TRANSIT_S,
                            rest_site_mm=np.array([0.0, 0.0, uc.SETTLE_CUP_Z_MM]))


@pytest.fixture(scope='module')
def catch_segment(catch_seed, catch_terminal, cfg, limits, geom):
    return sg.plan_segment(sg.CATCH, catch_seed, catch_terminal, cfg, limits,
                           geom)


@pytest.fixture(scope='module')
def catch_throw_terminal(catch_terminal):
    """The same catch, carrying the next same-site throw one dwell later."""
    return dataclasses.replace(
        catch_terminal,
        then_throw=sg.ThrowAfterCatch(t_release_s=TRANSIT_S + 0.30,
                                      site_mm=THROW_SITE_MM,
                                      target_mm=CATCH_SITE_MM, flight_s=T_F))


@pytest.fixture(scope='module')
def catch_throw_segment(catch_seed, catch_throw_terminal, cfg, limits, geom):
    return sg.plan_segment(sg.CATCH, catch_seed, catch_throw_terminal, cfg,
                           limits, geom)


@pytest.fixture(scope='module')
def rest_segment(catch_seed, cfg, limits, geom):
    terminal = sg.RestTerminal(rest_site_mm=np.array([0.0, 0.0, 700.0]),
                               t_rest_s=0.5)
    return sg.plan_segment(sg.REST, catch_seed, terminal, cfg, limits, geom)


# ---------------------------------------------------------------------------
# The rest-terminal post-condition, all three kinds (INVARIANTS gap 6 / I-PLAN-7)
# ---------------------------------------------------------------------------

_FEAS_TOL_MM = 1e-7 * 1000.0  # cup_cycle.CupCycleConfig.feas_tol, in mm


@pytest.mark.parametrize('fixture_name', ['throw_segment', 'catch_segment',
                                          'catch_throw_segment',
                                          'rest_segment'])
def test_every_segment_kind_ends_rest_terminal(fixture_name, request):
    segment = request.getfixturevalue(fixture_name)
    plan = segment.plan
    np.testing.assert_allclose(plan.pose_vel[-1], np.zeros(6), atol=1e-9)
    assert abs(float(plan.hand_vel_rps[-1])) < 1e-9
    cup_last = uc.cup_state_from_platform(plan.pose[-1], plan.hand_rev[-1])
    np.testing.assert_allclose(cup_last, segment.rest_site_mm,
                               atol=_FEAS_TOL_MM)
    assert uc.is_release_terminal(segment.meta) is False


def test_throw_segment_carries_the_release_event_and_takeoff_velocity(
        throw_segment):
    assert throw_segment.kind == sg.THROW
    assert throw_segment.event_t_s == pytest.approx(0.4)
    assert throw_segment.takeoff_vel_mm_s is not None
    # A throw straight up: no lateral takeoff component.
    assert abs(float(throw_segment.takeoff_vel_mm_s[0])) < 1e-6
    assert abs(float(throw_segment.takeoff_vel_mm_s[1])) < 1e-6
    assert float(throw_segment.takeoff_vel_mm_s[2]) > 0.0


def test_catch_segment_carries_the_catch_event_and_no_takeoff(catch_segment):
    assert catch_segment.kind == sg.CATCH
    assert catch_segment.event_t_s == pytest.approx(TRANSIT_S)
    assert catch_segment.takeoff_vel_mm_s is None


def test_rest_segment_carries_no_event(rest_segment):
    assert rest_segment.kind == sg.REST
    assert rest_segment.event_t_s is None
    assert rest_segment.takeoff_vel_mm_s is None


def test_a_catch_with_a_throw_carries_both_events_and_the_takeoff(
        catch_throw_segment, catch_throw_terminal):
    """One window, two events.  ``event_t_s`` stays the CATCH instant (that is
    what the executor schedules and re-sends against); ``release_t_s`` names
    the release the same segment also flies, and the take-off velocity is the
    one the QP pinned at it.  A standalone catch has neither."""
    seg = catch_throw_segment
    assert seg.kind == sg.CATCH
    assert seg.event_t_s == pytest.approx(TRANSIT_S, abs=1.5 * DT)
    assert seg.release_t_s == pytest.approx(
        catch_throw_terminal.then_throw.t_release_s, abs=1.5 * DT)
    assert seg.release_t_s > seg.event_t_s
    assert seg.takeoff_vel_mm_s is not None
    # A vertical self-toss of T_F: the take-off is up at g*T_F/2.
    assert seg.takeoff_vel_mm_s[2] == pytest.approx(
        ballistics_bc.GRAVITY_MMS2 * T_F / 2.0, rel=0.05)


def test_a_standalone_catch_carries_no_release(catch_segment):
    assert catch_segment.release_t_s is None
    assert catch_segment.takeoff_vel_mm_s is None


def test_a_carried_release_must_come_after_the_touch_down():
    """The ball has to be in the cup before it can be thrown."""
    with pytest.raises(ValueError, match='after the touch-down'):
        sg.CatchTerminal(
            landing_mm=CATCH_SITE_MM, landing_vel_mm_s=np.zeros(3),
            t_land_s=0.30, rest_site_mm=CATCH_SITE_MM,
            then_throw=sg.ThrowAfterCatch(t_release_s=0.30,
                                          site_mm=THROW_SITE_MM,
                                          target_mm=CATCH_SITE_MM,
                                          flight_s=T_F))


def test_plan_segment_rejects_an_unknown_kind(catch_seed, cfg, limits, geom):
    terminal = sg.RestTerminal(rest_site_mm=REST_MM, t_rest_s=0.3)
    with pytest.raises(ValueError, match='kind'):
        sg.plan_segment('NONSENSE', catch_seed, terminal, cfg, limits, geom)


# ---------------------------------------------------------------------------
# warm_start round trip
# ---------------------------------------------------------------------------

def test_warm_start_round_trips_to_a_faithful_plan_with_fewer_iterations(
        catch_seed, catch_terminal, cfg, limits, geom, catch_segment):
    """Feeding a segment's own ``warm_start`` into the next SAME-SHAPE plan is
    accepted (the key matches, so the active set actually seeds the solve --
    checked by iteration count dropping, not just by not erroring) and lands
    the SAME plan, to float round-off."""
    reseeded = sg.plan_segment(sg.CATCH, catch_seed, catch_terminal, cfg,
                               limits, geom, warm_start=catch_segment.warm_start)
    assert reseeded.warm_start.key == catch_segment.warm_start.key
    assert reseeded.warm_start.iters <= catch_segment.warm_start.iters
    np.testing.assert_allclose(reseeded.plan.pose, catch_segment.plan.pose,
                               atol=1e-9)
    np.testing.assert_allclose(reseeded.plan.hand_rev, catch_segment.plan.hand_rev,
                               atol=1e-9)


# ---------------------------------------------------------------------------
# Refusal propagation
# ---------------------------------------------------------------------------

def test_a_refusal_propagates_the_inner_code_unchanged(catch_seed, cfg, limits,
                                                        geom):
    """An infeasible terminal (a release window far too short to reach the
    site) raises ``unified_cycle.CycleInfeasible`` with the chain's own code --
    ``plan_segment`` neither wraps it nor invents a second one."""
    seed = _rest_state(REST_MM)
    terminal = sg.ThrowTerminal(site_mm=THROW_SITE_MM, target_mm=THROW_SITE_MM,
                                flight_s=T_F, t_release_s=0.05)
    with pytest.raises(uc.CycleInfeasible):
        sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)


def test_launch_leg_jerk_scales_with_the_aim_offset(geom, cfg):
    """A single-site launch from rest aimed 10 mm off its site must cost clearly
    less leg jerk than one aimed 20 mm off, and both must fit the 0.9 margin.

    Pins ``cup_realize._TILT_BLEND_MIN_KNOTS``: without the floor a small pin gap
    got a 2-knot tilt blend and every nonzero aim cost ~125k mm/s³ whatever its
    size (MEASURED 2026-09-13 at 300/5000/150000: 10 mm 123 165, 20 mm 124 686),
    which collapsed the R3 single-site admissible box to a zero aim."""
    limits = TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=LEG_VEL, leg_acc_mmps2=LEG_ACC, leg_jerk_mmps3=150000.0,
        hand_acc_rps2=HAND_ACC)
    site = np.array([-50.0, 0.0, 860.0])
    seed = _rest_state(np.array([-50.0, 0.0, uc.SETTLE_CUP_Z_MM]))

    def peak_jerk(dx_mm):
        terminal = sg.ThrowTerminal(site_mm=site,
                                    target_mm=site + np.array([dx_mm, 0.0, 0.0]),
                                    flight_s=T_F, t_release_s=0.4)
        seg = sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)
        return float(seg.meta.report.peak_leg_jerk_mmps3)

    j10, j20 = peak_jerk(10.0), peak_jerk(20.0)
    assert j10 < 0.6 * j20
    assert j20 < 0.9 * 150000.0


def test_terminal_validation_rejects_bad_shapes_and_non_positive_times():
    with pytest.raises(ValueError, match='site_mm'):
        sg.ThrowTerminal(site_mm=np.zeros(2), target_mm=THROW_SITE_MM,
                         flight_s=0.5, t_release_s=0.4)
    with pytest.raises(ValueError, match='flight_s'):
        sg.ThrowTerminal(site_mm=THROW_SITE_MM, target_mm=THROW_SITE_MM,
                         flight_s=0.0, t_release_s=0.4)
    with pytest.raises(ValueError, match='t_land_s'):
        sg.CatchTerminal(landing_mm=CATCH_SITE_MM,
                         landing_vel_mm_s=np.zeros(3), t_land_s=0.0,
                         rest_site_mm=REST_MM)
    with pytest.raises(ValueError, match='t_rest_s'):
        sg.RestTerminal(rest_site_mm=REST_MM, t_rest_s=-1.0)


# ---------------------------------------------------------------------------
# The opening REST homes the hand inside the firmware's own envelope
# (2026-09-18; `schedule.floor_lift_s`, `sites.REST_HAND_REV`)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('seed_rev', [0.0, 2.3071, 9.6227])
def test_the_opening_rest_homes_the_hand_inside_the_firmware_envelope(
        seed_rev, limits, geom, cfg):
    """SAMPLE THE PLANNED HAND LANE and assert its peaks — the test the
    sizing's shape factors (`schedule._SHAPE_KV` / `_SHAPE_KA`) are pinned by.

    A homing REST sized by ``schedule.floor_lift_s`` must keep the hand lane
    inside BOTH firmware numbers for the resume-and-follow regime:
    ``JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS`` (2.5 rev/s, the profiled park's own
    rate) and ``RECOVER_SLEW_ACCEL_RPS2`` (5 rev/s², the slew onset ramp). The
    three seeds cover the three branches of the ``max``: 0.0 rev (the ACTIVATE
    park, Δ = 0.307 ⇒ the 1.5 s floor), 2.3071 rev (Δ = 2.0 ⇒ the ACCELERATION
    bound binds, 1.549 s) and 9.6227 rev (the 2026-09-18 sitting's own hand,
    Δ = 9.316 ⇒ the VELOCITY bound binds, 6.987 s).

    Derivatives by finite difference on the 40 Hz knot grid, which is what the
    can-bridge itself interpolates — so this measures the lane the firmware
    will see, not a continuous-time ideal.
    """
    period = sc.floor_lift_s(seed_rev)
    seed = uc.CycleState.at_rest(
        np.array([0.0, 0.0, cr.RealizeConfig().active_z_mm, 0.0, 0.0, 0.0]),
        seed_rev, cr.RealizeConfig())
    terminal = sg.RestTerminal(
        rest_site_mm=np.array([-50.0, 0.0, uc.SETTLE_CUP_Z_MM]),
        t_rest_s=period)
    seg = sg.plan_segment(sg.REST, seed, terminal, cfg, limits, geom)

    hand = np.asarray(seg.plan.hand_rev, dtype=float)
    dt = float(seg.plan.dt)
    vel = np.gradient(hand, dt)
    acc = np.gradient(vel, dt)
    assert np.max(np.abs(vel)) <= sc.HOME_HAND_VEL_LIMIT_RPS
    assert np.max(np.abs(acc)) <= sc.HOME_HAND_ACC_LIMIT_RPS2
    # ... and it actually ARRIVES home, at rest.
    assert hand[-1] == pytest.approx(sc.REST_HAND_REV, abs=1e-3)
    # At rest at the terminal knot — the PLAN's own velocity channel, not the
    # one-sided edge difference `np.gradient` leaves at the last sample.
    assert float(seg.plan.hand_vel_rps[-1]) == pytest.approx(0.0, abs=1e-3)
    # The bounds the node LOGS are honest upper bounds on what was planned.
    bound_v, bound_a = sc.home_hand_bounds(seed_rev, period)
    assert np.max(np.abs(vel)) <= bound_v + 1e-9
    assert np.max(np.abs(acc)) <= bound_a + 1e-9


# ---------------------------------------------------------------------------
# The held-attitude CATCH and the attitude-bearing REST (R4 U2)
# ---------------------------------------------------------------------------
#
# THE RECIPE (probe ``probe_r4_bb_catch4.py``, venv, 2026-09-23, these limits):
# a BB arrival 18-40 deg off vertical at 4.5-5.5 m/s clamps to the 12 deg receive
# tilt; a PRE-TILT REST carries the platform from its level rest onto the axis
# line at that attitude (1.0 s: leg vel 70.8 mm/s, acc 221.7, jerk 10450;
# 1.5 s: 47.1 / 97.5 / 3611; 2.0 s: 35.3 / 54.6 / 1706); the held-attitude CATCH
# then plans at peak leg vel 3.1 mm/s, acc 61.6-64.3, jerk 2377-2557, hand acc
# 1029-1076 rev/s^2, with the cup opening at the touch-down knot exactly on the
# landing point; the DECAY REST mirrors the pre-tilt; a 0.9 m self-toss THROW
# from the level rest it leaves passes at hand acc 3152 rev/s^2.

_BB_ARRIVAL = np.array([1390.5, 0.0, -4279.6])       # 18 deg at 4.5 m/s, mm/s


def test_receive_hold_tilt_is_the_clamped_receive_tilt():
    """One derivation of the reload's attitude, and it SATURATES.

    A BB arrival is 18-40 deg off vertical but the platform can only track 12,
    so past the clamp the arrival is partly nulled and the residual is in-cup
    skid — which the FSM accepted and caught through. Asserted here because the
    schedule, the segment and the QP must all read the same 12 deg.
    """
    from jugglebot.motion.trajectory import tilt_geometry as tg
    for deg in (18.0, 25.0, 40.0):
        a = np.radians(deg)
        v = 4800.0 * np.array([np.sin(a), 0.0, -np.cos(a)])
        tilt = sg.receive_hold_tilt(v)
        assert tilt == tuple(tg.tilt_to_receive(v))
        assert abs(np.degrees(np.hypot(*tilt)) - tg.MAX_TILT_DEG) < 1e-9
    # ...and BELOW the clamp it is the exact collinear answer: the cup axis lands
    # anti-parallel to the arrival, which is what "no skid" means.
    a = np.radians(8.0)
    v = 4800.0 * np.array([np.sin(a), 0.0, -np.cos(a)])
    axis = tg.cup_axis(*sg.receive_hold_tilt(v))
    assert np.allclose(axis, -v / np.linalg.norm(v), atol=1e-12)


def test_hold_axis_site_is_on_the_axis_through_the_landing_point():
    """Every site a held-attitude catch names is a point on ONE line.

    The cup moves along the held axis and nowhere else, so its rest is up the
    axis from the landing point, not under it. At the 12 deg ceiling that is
    0.2126 mm of lateral per mm of height — 29.8 mm over the 140.4 mm from the
    catch height to the rest height, which is why the pre-tilt REST also
    translates.
    """
    from jugglebot.motion.trajectory import tilt_geometry as tg
    tilt = sg.receive_hold_tilt(_BB_ARRIVAL)
    axis = tg.cup_axis(*tilt)
    land = CATCH_SITE_MM
    for z in (750.0, 830.0, 900.0):
        site = sg.hold_axis_site(land, tilt, z)
        assert site[2] == z
        d = site - land
        if abs(d[2]) > 0:
            assert np.allclose(d / np.linalg.norm(d),
                               np.sign(d[2]) * axis, atol=1e-12)
    assert np.allclose(sg.hold_axis_site(land, tilt, land[2]), land, atol=1e-12)


def test_catch_terminal_carries_receive_tilt():
    """The touch-down pin threads through the dataclass unchanged — the
    segment-layer half of ``unified_cycle.CycleGoals.receive_tilt``'s
    contract."""
    v = np.array([0.0, 0.0, -1000.0])
    ct = sg.CatchTerminal(landing_mm=CATCH_SITE_MM, landing_vel_mm_s=v,
                          t_land_s=0.3,
                          rest_site_mm=np.array([0.0, 0.0, uc.SETTLE_CUP_Z_MM]),
                          receive_tilt=(0.0, 0.0))
    assert ct.receive_tilt == (0.0, 0.0)


# ---------------------------------------------------------------------------
# R5: the BB-fed columns feed catch, level-pinned (``receive_tilt``) — the
# acceptance case for ``level_pinned_feed_report.md`` / ``probe_
# level_pinned_feed.py`` (scratchpad, 2026-09-30), reusing ``probe_
# bbfed_columns.py``'s S1/S2 construction exactly: apex 0.90 m, dwell 0.30 s,
# separation 100 mm, session limits 300/5000/200000 mm-space, hand_acc 3500
# rev/s^2, seeded from ball A's own THROW-from-rest release at P1.
# ---------------------------------------------------------------------------

_BBFED_LEG_VEL = 300.0
_BBFED_LEG_ACC = 5000.0
_BBFED_LEG_JERK = 200000.0
_BBFED_HAND_ACC = 3500.0
_BBFED_APEX_M = 0.90
_BBFED_DWELL_S = 0.30
_BBFED_SEP_MM = 100.0
#: BB's real arrival at P2 (bbfed_columns_probe.md's figure): 5.628 m/s,
#: 11.892 deg off vertical.
_BBFED_ARRIVAL_MM_S = np.array([1058.0, 475.0, -5507.0])


@pytest.fixture(scope='module')
def bbfed_limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=_BBFED_LEG_VEL, leg_acc_mmps2=_BBFED_LEG_ACC,
        leg_jerk_mmps3=_BBFED_LEG_JERK, hand_acc_rps2=_BBFED_HAND_ACC)


@pytest.fixture(scope='module')
def bbfed_seed(bbfed_limits, geom, cfg):
    """S1: THROW A from rest at P1 — the seed every S2 case below shares."""
    p1, _p2 = si.columns_sites(_BBFED_SEP_MM)
    seed = _rest_state(p1.rest_site_mm())
    terminal = sg.ThrowTerminal(site_mm=p1.throw_site_mm(),
                                target_mm=p1.throw_site_mm(),
                                flight_s=sc.flight_s(_BBFED_APEX_M),
                                t_release_s=0.4)
    seg1 = sg.plan_segment(sg.THROW, seed, terminal, cfg, bbfed_limits, geom)
    return seg1.release_state


def _bbfed_catch_terminal(receive_tilt):
    p1, p2 = si.columns_sites(_BBFED_SEP_MM)
    t_f = sc.flight_s(_BBFED_APEX_M)
    tau = sc.transit_s(t_f, _BBFED_DWELL_S)
    then_throw = sg.ThrowAfterCatch(t_release_s=tau + _BBFED_DWELL_S,
                                    site_mm=p2.throw_site_mm(),
                                    target_mm=p2.throw_site_mm(), flight_s=t_f)
    return sg.CatchTerminal(landing_mm=p2.catch_site_mm(),
                            landing_vel_mm_s=_BBFED_ARRIVAL_MM_S, t_land_s=tau,
                            rest_site_mm=p2.rest_site_mm(), then_throw=then_throw,
                            receive_tilt=receive_tilt)


def test_a_bb_fed_feed_catch_refuses_unpinned_but_fits_level_pinned(
        bbfed_limits, geom, cfg, bbfed_seed):
    """The acceptance case (probe_bbfed_columns.py S2 / probe_level_pinned_
    feed.py ratio=0.7, the default ``catch_vel_ratio``): BB's real arrival
    refuses the ordinary auto-banked receive tilt (measured LIMIT_VEL, 122%
    of the leg-velocity cap — bbfed_columns_probe.md sec 2) and fits
    comfortably, hand-bound at ~95% like the pattern's own vertical catch,
    once the touch-down attitude is pinned level."""
    with pytest.raises(uc.CycleInfeasible):
        sg.plan_segment(sg.CATCH, bbfed_seed, _bbfed_catch_terminal(None),
                        cfg, bbfed_limits, geom)
    seg = sg.plan_segment(sg.CATCH, bbfed_seed, _bbfed_catch_terminal((0.0, 0.0)),
                          cfg, bbfed_limits, geom)
    hand_frac = (seg.meta.report.peak_hand_acc_rps2
                 / bbfed_limits.hand_acc_limit_rps2)
    assert hand_frac <= 0.96, hand_frac


def test_a_held_attitude_catch_carries_its_throw_and_releases_level(
        bb_chain, limits, geom):
    """PRE-TILT REST → ONE held catch-with-throw segment, rest-terminal.

    The R4 reload's held CATCH + DECAY REST + THROW folded into one STEADY: the
    receive attitude is held through the touch-down, re-levelled after it, and
    the ball leaves a LEVEL cup going straight up; the SETTLE tail then brings
    the machine to rest like every other segment (plan § 0: every segment
    ends at rest at its site).  The release knot's velocity is the slew's true
    zero, so ``extend``'s seam re-gate sees no step there.

    MEASURED (scratchpad ``probe_fb_segment.py``, 2026-09-30, this module's
    limits 300/5000/200k/3500, the 12° clamp of an 18° 4.5 m/s arrival):
    (t_land 0.20, release 0.65) OK v187 a1970 j146k h3064; release 0.60
    refuses LIMIT_JERK 211k.  With the slew landing ON the release knot the
    same segment read 315k at the seam (``cup_realize.
    HELD_SLEW_LANDS_BEFORE_RELEASE_KNOTS``).  FAIL-BEFORE: ``CatchTerminal``
    refused ``then_throw`` + ``hold_tilt`` at construction.
    """
    tilt, pre, _, _ = bb_chain
    cfg = sg.SegmentConfig()
    seed = uc.state_at_knot(pre.plan, pre.meta, pre.plan.pose.shape[0] - 1)
    level = np.array([CATCH_SITE_MM[0], CATCH_SITE_MM[1], uc.SETTLE_CUP_Z_MM])
    tt = sg.ThrowAfterCatch(t_release_s=0.65, site_mm=THROW_SITE_MM,
                            target_mm=THROW_SITE_MM, flight_s=T_F)
    seg = sg.plan_segment(sg.CATCH, seed, sg.CatchTerminal(
        landing_mm=CATCH_SITE_MM, landing_vel_mm_s=_BB_ARRIVAL, t_land_s=0.20,
        rest_site_mm=level, then_throw=tt, hold_tilt=tilt), cfg, limits, geom)
    assert seg.kind == sg.CATCH
    k_rel = int(round(seg.release_t_s / DT))
    k_td = int(np.floor(seg.event_t_s / DT + 1e-9))
    tilts = seg.plan.pose[:, 3:5]
    assert np.max(np.abs(tilts[:k_td + 2] - np.asarray(tilt))) < 1e-6
    assert np.max(np.abs(tilts[k_rel - 2:k_rel + 1])) < 1e-9     # level release
    assert np.allclose(seg.takeoff_vel_mm_s[:2], 0.0, atol=1e-6)
    assert seg.takeoff_vel_mm_s[2] > 0.0
    # rest-terminal: the machine stops at the rest site
    assert np.max(np.abs(seg.plan.pose_vel[-1])) < 1e-6
    assert abs(float(seg.plan.hand_vel_rps[-1])) < 1e-6
    assert np.allclose(seg.rest_site_mm, level)
    rep = seg.meta.report
    assert rep.peak_leg_jerk_mmps3 <= limits.leg_jerk_mmps3


@pytest.fixture(scope='module')
def bb_chain(limits, geom):
    """PRE-TILT REST -> held-attitude CATCH -> DECAY REST, chained as the
    schedule will chain them (each seeded from the last one's terminal knot)."""
    cfg = sg.SegmentConfig()
    tilt = sg.receive_hold_tilt(_BB_ARRIVAL)
    land = CATCH_SITE_MM
    pre_site = sg.hold_axis_site(land, tilt, uc.SETTLE_CUP_Z_MM)
    level = np.array([land[0], land[1], uc.SETTLE_CUP_Z_MM])
    seed = _rest_state(level)
    pre = sg.plan_segment(sg.REST, seed, sg.RestTerminal(
        rest_site_mm=pre_site, t_rest_s=1.5, tilt=tilt), cfg, limits, geom)
    seed1 = uc.state_at_knot(pre.plan, pre.meta, pre.plan.pose.shape[0] - 1)
    cat = sg.plan_segment(sg.CATCH, seed1, sg.CatchTerminal(
        landing_mm=land, landing_vel_mm_s=_BB_ARRIVAL, t_land_s=1.0,
        rest_site_mm=pre_site, hold_tilt=tilt), cfg, limits, geom)
    seed2 = uc.state_at_knot(cat.plan, cat.meta, cat.plan.pose.shape[0] - 1)
    dec = sg.plan_segment(sg.REST, seed2, sg.RestTerminal(
        rest_site_mm=level, t_rest_s=1.5, tilt=(0.0, 0.0)), cfg, limits, geom)
    return tilt, pre, cat, dec


def test_the_pre_tilt_rest_leaves_the_machine_at_the_receive_attitude(bb_chain):
    """The PRE-TILT's whole job: end AT the attitude, on the axis line.

    The held-attitude catch refuses (``TILT_PIN``) a seed that is not already
    there, so this is the segment that makes the catch plannable at all.
    """
    tilt, pre, _, _ = bb_chain
    assert np.allclose(pre.plan.pose[-1, 3:5], np.asarray(tilt), atol=1e-12)
    assert np.allclose(pre.plan.pose[0, 3:5], 0.0, atol=1e-12)
    assert pre.meta.report.peak_leg_vel_mmps < 100.0


def test_the_held_attitude_catch_holds_it_and_puts_the_cup_on_the_ball(
        bb_chain, limits):
    """One attitude for the whole window, the cup on the landing point, and the
    legs essentially still — 3.1 mm/s against the 300 mm/s session limit.

    That last number is the point of the unit. The same catch WITHOUT the held
    attitude asks the legs to cancel the stroke's own lateral projection
    (v_z sin 12 deg = 624 mm/s) and refuses LIMIT_VEL (probes 1-3, 2026-09-23).
    """
    tilt, _, cat, _ = bb_chain
    tilts = cat.plan.pose[:, 3:5]
    assert np.max(np.abs(tilts - np.asarray(tilt))) < 1e-12
    k = int(round(cat.event_t_s / cat.plan.dt))
    cup = uc.cup_state_from_platform(cat.plan.pose[k], cat.plan.hand_rev[k],
                                    uc.build_realize_config(limits))
    assert np.allclose(np.asarray(cup)[:2], CATCH_SITE_MM[:2], atol=0.05)
    assert cat.meta.report.peak_leg_vel_mmps < 20.0
    assert cat.meta.report.peak_hand_acc_rps2 < HAND_ACC


def test_the_decay_rest_returns_the_machine_to_level(bb_chain):
    """The FSM let the tilt decay after the seat; the DECAY REST is that, and it
    is what leaves a level rest for the next THROW to launch from."""
    _, _, _, dec = bb_chain
    assert np.allclose(dec.plan.pose[-1, 3:5], 0.0, atol=1e-12)
    assert dec.meta.report.peak_leg_vel_mmps < 100.0


# ---------------------------------------------------------------------------
# The post-release platform hold (SegmentConfig.post_release_hold_s, 2026-09-28)
# ---------------------------------------------------------------------------

#: A 250 mm hop: the shape whose release is TILTED, and the only one where the
#: hold has anything to do — a vertical self-toss releases level, the detach cone
#: reduces to ``acc_xy == 0`` there, and the centroid is already still.
#: The production 250 mm column pair, exactly as the runsheet's hop uses it.
HOP_P1, HOP_P2 = si.columns_sites(250.0)
#: Knots the hold covers, read from the production constant (never re-derived).
HOLD_K = uc.post_release_hold_knots()


@pytest.fixture(scope='module')
def hop_throw_segment(limits, geom, cfg):
    """A THROW from rest aimed 250 mm away — release tilt 4.006 deg."""
    seed = _rest_state(HOP_P1.rest_site_mm())
    terminal = sg.ThrowTerminal(site_mm=HOP_P1.throw_site_mm(),
                                target_mm=HOP_P2.catch_site_mm(),
                                flight_s=T_F, t_release_s=0.4)
    return sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)


@pytest.fixture(scope='module')
def hop_release_seed(hop_throw_segment):
    """The state a SPLICE seeds the next skill with when it cuts AT the release
    knot — which ``splice_at`` is allowed to do (``k_s == k_rel`` is
    :func:`unified_cycle.extend`'s own chain), and which is where the 250 mm
    hop's post-release knots actually come from in the field."""
    k_rel = int(round(float(hop_throw_segment.event_t_s) / DT))
    return uc.release_state_at_knot(hop_throw_segment.plan,
                                    hop_throw_segment.meta, k_rel)


def _hold_verdict(plan, k_rel, n=HOLD_K):
    """(worst centroid speed mm/s, worst centroid displacement mm) over knots
    ``k_rel+1 .. k_rel+n`` — the quantity ``tools/probes/planned_release_motion.py``
    prints as its HOLD verdict."""
    pose = np.asarray(plan.pose)
    vel = np.asarray(plan.pose_vel)
    ks = [k for k in range(k_rel + 1, k_rel + 1 + n) if k < len(pose)]
    assert len(ks) == n
    speed = max(float(np.hypot(vel[k][0], vel[k][1])) for k in ks)
    disp = max(float(np.linalg.norm(pose[k][:3] - pose[k_rel][:3])) for k in ks)
    return speed, disp


def _cup_vel_at(plan, k):
    pose = np.asarray(plan.pose)
    return np.asarray(uc.cup_velocity_from_platform(
        pose[k], np.asarray(plan.pose_vel)[k], np.asarray(plan.hand_rev)[k],
        np.asarray(plan.hand_vel_rps)[k])).ravel()[:3]


def _off_stroke_line_mm_s(plan, k_rel, n=HOLD_K):
    """Worst |cup velocity off the held line| (mm/s) over the held knots.

    THE invariant the hold's QP rows state: ``v_xy == kappa · v_z`` with
    ``kappa = v_xy / v_z`` of the cup at the RELEASE knot — the launch line the
    ball leaves on (2026-09-28 sitting-2 fix; the first version held the
    platform-tilt axis, which a levelling correction rotates off the launch
    line, see ``unified_cycle._realize``).  With no correction the two agree to
    the sin/tan difference of the 4 deg release tilt (0.48 mm/s here).
    """
    v_rel = _cup_vel_at(plan, k_rel)
    kappa = v_rel[:2] / v_rel[2]
    worst = 0.0
    for k in range(k_rel + 1, k_rel + 1 + n):
        v = _cup_vel_at(plan, k)
        worst = max(worst, float(np.max(np.abs(v[:2] - kappa * v[2]))))
    return worst


def test_a_tilted_throws_tail_holds_the_cup_on_the_frozen_stroke_line(
        hop_throw_segment):
    """The SETTLE tail of a 250 mm hop keeps the cup on the held line.

    MEASURED 2026-09-28 (``tools/probes/planned_release_motion.py``, and the
    fail-before run of this test, at the FIRST 3-knot/75 ms sizing -- the shipped hold is 2 knots, `HOLD_K` below): off-line cup velocity over the three held
    knots was **17.1 / 34.3 / 51.4 mm/s** before the hold (0.0041 mm/s after) — the detach cone's
    ``acc == g + lambda·axis``, whose lateral part is ``9810·sin 4.006 deg =
    686 mm/s^2`` of centroid acceleration — and is **< 1e-6 mm/s** with it.
    The centroid velocity over those knots falls 28.7 / 70.3 / 107.6 -> 11.1 /
    35.9 / 57.3 mm/s; the residual is entirely the attitude slew through the
    measured 224-261 mm cup lever, which this hold deliberately does NOT pin
    (see ``unified_cycle._realize``'s "attitude half" block).
    """
    k_rel = int(round(float(hop_throw_segment.event_t_s) / DT))
    # The residual is the reconstruction, not the pin —
    # cup_velocity_from_platform rebuilds the cup velocity from the REALISED pose
    # with finite-differenced tilt rates, while the QP pinned the analytic one.
    assert _off_stroke_line_mm_s(hop_throw_segment.plan, k_rel) < 0.05
    speed, disp = _hold_verdict(hop_throw_segment.plan, k_rel)
    assert speed < 60.0
    assert disp < 2.0


def test_a_hop_throw_under_a_cross_hop_level_correction_plans_inside_the_hand_limit(
        limits, geom, cfg):
    """A levelling correction across the hop must not break the post-release hold.

    MEASURED 2026-09-28 21:41 sitting: all three 250 mm hop attempts refused
    ``HAND_LIMIT_ACC: peak hand acceleration 4969.7 rev/s^2 > 3500.0`` at the
    THROW install, under the sitting's level offset (-3.857, +5.809) mrad.  The
    hold held the platform-TILT axis, which the correction rotates 5.8 mrad off
    the launch velocity across the hop; one knot after a release with
    ``v_y == 0`` the rows demanded ``v_y == 0.0058·v_z``, and the QP collapsed
    ``v_z`` — the hand.  Fail-before (this test, 2026-09-28, the committed planner):
    ``CycleInfeasible`` HAND_LIMIT_ACC 5235.7 rev/s^2 from the y half alone on
    this test's own seed (the executor-driven probe, a spliced seed, read 5229.2).
    Pass-after: 2995.9 rev/s^2, the no-correction plan's 2980.2 within 1 %.
    """
    from jugglebot.motion import levelling
    base = _rest_state(HOP_P1.rest_site_mm())
    corr = levelling.correction_from_offset(0.0, 0.005809100317737187)
    seed = uc.CycleState.at_rest(base.pose, base.hand_rev,
                                 levelling_correction=corr)
    terminal = sg.ThrowTerminal(site_mm=HOP_P1.throw_site_mm(),
                                target_mm=HOP_P2.catch_site_mm(),
                                flight_s=T_F, t_release_s=0.4)
    seg = sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)
    assert seg.meta.report.peak_hand_acc_rps2 <= limits.hand_acc_limit_rps2
    k_rel = int(round(float(seg.event_t_s) / DT))
    assert _off_stroke_line_mm_s(seg.plan, k_rel) < 0.05


def test_a_post_release_landing_holds_the_platform_after_the_release(
        hop_release_seed, cfg, limits, geom):
    """The window the 250 mm hop's post-release knots really come from.

    The CATCH at the far site is installed BEFORE the release and spliced in at
    the release knot, so the hold has to live on the LANDING too — not only on
    the THROW's tail.  MEASURED 2026-09-28 through the real install chain
    (``tools/probes/planned_release_motion.py``): worst centroid speed over the
    three held knots **68.10 -> 13.36 mm/s** (measured at the first 3-knot sizing; 2.75 mm/s at the shipped 2 knots), worst centroid displacement
    **2.018 -> 0.342 mm**; the ball rode the 68 mm/s out of the cup as the
    +85..+110 mm/s of unplanned lateral velocity that overshot the far site by
    87-104 mm on 3/3 hops on 2026-09-27.
    """
    v_arrival = np.array([0.0, 0.0, -ballistics_bc.GRAVITY_MMS2 * (T_F / 2.0)])
    landing_mm = np.asarray(HOP_P2.catch_site_mm(), dtype=float)
    terminal = sg.CatchTerminal(
        landing_mm=landing_mm, landing_vel_mm_s=v_arrival,
        t_land_s=T_F,
        rest_site_mm=np.array([landing_mm[0], landing_mm[1],
                               uc.SETTLE_CUP_Z_MM]))
    seg = sg.plan_segment(sg.CATCH, hop_release_seed, terminal, cfg, limits,
                          geom)
    assert _off_stroke_line_mm_s(seg.plan, 0) < 0.05
    speed, disp = _hold_verdict(seg.plan, 0)
    assert speed < 20.0
    assert disp < 0.6
    # still rest-terminal, and the gate passed (plan_segment raises otherwise)
    assert float(np.max(np.abs(np.asarray(seg.plan.pose_vel)[-1]))) < 1e-6


def test_the_hold_is_off_and_bit_identical_off_a_seed_that_followed_no_release(
        limits, geom):
    """A REST from a machine at rest is bit-for-bit what it was before the hold.

    ``_hold_knots`` answers 0 off a seed with ``post_release=False``: there is no
    ball leaving, and the rows would only forbid the cup from accelerating out of
    rest at all (the same reason the detach cone is not assembled there).
    """
    seed = _rest_state(REST_MM)
    assert sg._hold_knots(sg.SegmentConfig(), seed) == 0
    terminal = sg.RestTerminal(rest_site_mm=np.array([0.0, 0.0, 700.0]),
                               t_rest_s=0.5)
    held = sg.plan_segment(sg.REST, seed, terminal, sg.SegmentConfig(), limits,
                           geom)
    off = sg.plan_segment(sg.REST, seed,
                          terminal, sg.SegmentConfig(post_release_hold_s=0.0),
                          limits, geom)
    assert np.array_equal(np.asarray(held.plan.pose), np.asarray(off.plan.pose))
    assert np.array_equal(np.asarray(held.plan.hand_rev),
                          np.asarray(off.plan.hand_rev))


# ---------------------------------------------------------------------------
# The PRE-release platform hold (SegmentConfig.pre_release_hold_s, 2026-09-29)
# ---------------------------------------------------------------------------

#: Knots the pre-release hold covers, read from the production constant.
PRE_K = uc.pre_release_hold_knots()
#: The last 100 ms before a release, in knots — FIXED, not read from the
#: constant, so forcing the constant to 0 makes the tests below fail rather
#: than check an empty range.
LAST_100MS_K = int(round(0.100 / DT))
#: R4's launch limits (the runsheet's 300/5000/150000 + hand 3500) — tighter
#: in jerk than this module's ``limits`` fixture, which predates them.
R4_LIMITS = TrajectoryLimits.from_config(hw).with_session_limits(
    leg_vel_mmps=300.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=150000.0,
    hand_acc_rps2=3500.0)


def _flight_s(apex_m):
    return 2.0 * (2.0 * apex_m / (ballistics_bc.GRAVITY_MMS2 / 1000.0)) ** 0.5


def _hop_throw(apex_m, cfg, geom, limits=R4_LIMITS):
    seed = _rest_state(HOP_P1.rest_site_mm())
    terminal = sg.ThrowTerminal(site_mm=HOP_P1.throw_site_mm(),
                                target_mm=HOP_P2.catch_site_mm(),
                                flight_s=_flight_s(apex_m), t_release_s=0.4)
    return sg.plan_segment(sg.THROW, seed, terminal, cfg, limits, geom)


def _carrier_xy_mm_s(plan, k):
    """The platform's rigid-body velocity at the cup (hand velocity zeroed) —
    what the ball rides laterally while it is still in the cup."""
    pose = np.asarray(plan.pose)
    v = np.asarray(uc.cup_velocity_from_platform(
        pose[k], np.asarray(plan.pose_vel)[k], np.asarray(plan.hand_rev)[k],
        0.0)).ravel()
    return float(np.hypot(v[0], v[1]))


def _off_launch_line_mm_s(plan, k_rel, n):
    """Worst |cup velocity off the LAUNCH line| (mm/s) over knots
    ``k_rel−n .. k_rel−1`` — the invariant the pre-hold rows state, with kappa
    read off the cup at the release knot."""
    v_rel = _cup_vel_at(plan, k_rel)
    kappa = v_rel[:2] / v_rel[2]
    return max(float(np.max(np.abs(_cup_vel_at(plan, k)[:2]
                                    - kappa * _cup_vel_at(plan, k)[2])))
               for k in range(k_rel - n, k_rel))


@pytest.mark.parametrize('apex_m', [0.85, 0.9, 0.95])
def test_a_hop_throw_holds_the_platform_still_for_the_last_100_ms(apex_m, geom):
    """The 250 mm hop's platform no longer swings laterally into the release.

    MEASURED 2026-09-29 (probe ``probe_prehold.py`` and this test, R4 limits):
    without the hold the planned centroid vx at −75 / −50 ms was +105.7/+91.6
    (0.85 m), +106.1/+95.2 (0.9 m), +105.4/+99.1 (0.95 m) mm/s — the QP ramping
    the cup's lateral velocity to ``v_takeoff_xy`` independently of the stroke,
    which the plant tracks 45 ms late and the ball carries out as +x overshoot.
    With it the cup rides the launch line (off-line < 1e-6 mm/s in the QP) and
    the platform carrier at the cup is ≤ 1.9 mm/s over the last 100 ms (the
    residual is the reconstruction's finite-differenced tilt rate; it is
    +0.6..+0.8 at the release knot with or without the hold).

    FAIL-BEFORE (2026-09-29, ``PRE_RELEASE_HOLD_S`` forced to 0.0): the
    carrier assertion fails at 98.8 / 99.5 / 99.1 mm/s (0.85 / 0.9 / 0.95 m).
    """
    seg = _hop_throw(apex_m, sg.SegmentConfig(), geom)
    k_rel = int(round(float(seg.event_t_s) / DT))
    worst = max(_carrier_xy_mm_s(seg.plan, k)
                for k in range(k_rel - LAST_100MS_K, k_rel + 1))
    assert worst <= 2.5, worst
    assert _off_launch_line_mm_s(seg.plan, k_rel, LAST_100MS_K) < 1.0
    assert PRE_K == LAST_100MS_K == 4
    assert seg.meta.releases[0].pre_hold_knots == PRE_K

    # ...and the swing it replaces, so the test cannot pass vacuously.
    off = _hop_throw(apex_m, sg.SegmentConfig(pre_release_hold_s=0.0), geom)
    vx = np.asarray(off.plan.pose_vel)[:, 0]
    assert 85.0 <= vx[k_rel - 3] <= 115.0      # −75 ms
    assert 85.0 <= vx[k_rel - 2] <= 115.0      # −50 ms


@pytest.mark.parametrize('apex_m', [0.85, 0.9, 0.95])
def test_a_held_hop_throw_is_inside_the_r4_limits(apex_m, geom):
    """Peaks at R4's limits, and they FALL with the hold (the −97/+106 mm/s
    reversal becomes a −45 mm/s backswing): 0.9 m leg jerk 129k → 107k,
    acc 1735 → 1489 mm/s² (2026-09-29). plan_segment raises on any breach;
    the asserts pin the margin."""
    held = _hop_throw(apex_m, sg.SegmentConfig(), geom).meta.report
    off = _hop_throw(apex_m, sg.SegmentConfig(pre_release_hold_s=0.0),
                     geom).meta.report
    assert held.peak_leg_vel_mmps <= 300.0
    assert held.peak_leg_acc_mmps2 <= 5000.0
    assert held.peak_leg_jerk_mmps3 <= 150000.0
    assert held.peak_hand_acc_rps2 <= 3500.0
    assert held.peak_leg_jerk_mmps3 <= off.peak_leg_jerk_mmps3
    assert held.peak_leg_acc_mmps2 <= off.peak_leg_acc_mmps2


def test_a_held_self_toss_and_its_catch_and_throw_are_inside_the_r4_limits(
        geom):
    """0.9 m self-toss: the THROW and the CATCH-and-throw (one STEADY window,
    touch-down 0.30 s before the release, well before the hold) both plan at
    R4's limits with the hold on, and the legs do not move for the THROW."""
    tf = _flight_s(0.9)
    cfg = sg.SegmentConfig()
    throw = sg.plan_segment(
        sg.THROW, _rest_state(REST_MM),
        sg.ThrowTerminal(site_mm=THROW_SITE_MM, target_mm=THROW_SITE_MM,
                         flight_s=tf, t_release_s=0.4), cfg, R4_LIMITS, geom)
    assert throw.meta.releases[0].pre_hold_knots == PRE_K
    rel_k = int(round(float(throw.event_t_s) / DT))
    seed = uc.release_state_at_knot(throw.plan, throw.meta, rel_k)
    v_arr = np.array([0.0, 0.0, -ballistics_bc.GRAVITY_MMS2 * (tf / 2.0)])
    ct = sg.plan_segment(sg.CATCH, seed, sg.CatchTerminal(
        landing_mm=THROW_SITE_MM, landing_vel_mm_s=v_arr, t_land_s=tf,
        rest_site_mm=REST_MM,
        then_throw=sg.ThrowAfterCatch(t_release_s=tf + 0.30,
                                      site_mm=THROW_SITE_MM,
                                      target_mm=THROW_SITE_MM, flight_s=tf)),
        cfg, R4_LIMITS, geom)
    for rep in (throw.meta.report, ct.meta.report):
        assert rep.peak_leg_vel_mmps <= 300.0
        assert rep.peak_leg_acc_mmps2 <= 5000.0
        assert rep.peak_leg_jerk_mmps3 <= 150000.0
        assert rep.peak_hand_acc_rps2 <= 3500.0
    assert throw.meta.report.peak_leg_vel_mmps < 1.0
    assert ct.meta.releases[-1].pre_hold_knots == PRE_K


def test_the_pre_release_hold_at_zero_is_bit_identical_to_no_hold(geom, cfg):
    """``pre_release_hold_s=0`` (the sitting's A/B arm) is the plan as it was
    before the hold existed: the same LAUNCH + SETTLE chain built by hand from
    goals that never mention the field."""
    seg = _hop_throw(0.9, sg.SegmentConfig(pre_release_hold_s=0.0), geom)
    seed = _rest_state(HOP_P1.rest_site_mm())
    rest_mm = np.array([HOP_P1.throw_site_mm()[0], HOP_P1.throw_site_mm()[1],
                        uc.SETTLE_CUP_Z_MM])
    goals_a = uc.CycleGoals(period_s=0.4,
                            throw_site_mm=HOP_P1.throw_site_mm(),
                            throw_target_mm=HOP_P2.catch_site_mm(),
                            flight_s=_flight_s(0.9), settle_site_mm=rest_mm)
    plan_a, meta_a = uc.plan_launch(goals_a, seed, R4_LIMITS, geom)
    seed_b = uc.release_state_from_meta(meta_a, plan_a)
    goals_b = uc.CycleGoals(period_s=cfg.rest_tail_s, settle_site_mm=rest_mm,
                            hold_platform_knots=sg._hold_knots(cfg, seed_b))
    plan_b, meta_b = uc.plan_settle(goals_b, seed_b, R4_LIMITS, geom)
    ref, ref_meta = uc.extend(plan_a, meta_a, plan_b, meta_b, R4_LIMITS, geom)
    for name in ('pose', 'pose_vel', 'hand_rev', 'hand_vel_rps'):
        assert np.array_equal(np.asarray(getattr(seg.plan, name)),
                              np.asarray(getattr(ref, name))), name
    assert seg.meta.releases[0].pre_hold_knots == 0


def test_a_splice_inside_the_pre_release_hold_is_refused_and_keeps_the_plan(
        geom):
    """A re-plan landing in the last 100 ms before a release is REFUSED (the
    installed plan, which already holds, keeps streaming): the shorter window
    could not carry the hold (rank) and without it would re-open the lateral
    ramp.  Before the hold, and on a plan built with the hold off, the guard
    is silent — so the A/B arm splices exactly as before.

    FAIL-BEFORE (2026-09-29, constant forced to 0.0): the splice at k_rel−4 is
    not refused by the guard and reaches the seam check (CHAIN_DISCONTINUITY).
    """
    seg = _hop_throw(0.9, sg.SegmentConfig(), geom)
    k_rel = int(round(float(seg.event_t_s) / DT))
    n = int(seg.plan.n_knots)
    for k_s in range(k_rel - LAST_100MS_K, k_rel):
        with pytest.raises(uc.CycleInfeasible) as ei:
            uc.splice_at(seg.plan, seg.meta, k_s, seg.plan, seg.meta,
                         R4_LIMITS, geom)
        assert ei.value.code == uc.REPLAN_WINDOW
        assert 'too late to change this plan' in ei.value.outcome()
    uc._refuse_splice_into_a_detach_cone(seg.meta, k_rel - LAST_100MS_K - 1,
                                         DT, n)
    off = _hop_throw(0.9, sg.SegmentConfig(pre_release_hold_s=0.0), geom)
    uc._refuse_splice_into_a_detach_cone(off.meta, k_rel - 1, DT, n)


def test_a_catch_inside_the_pre_release_hold_is_refused_in_plain_language(
        catch_seed, catch_terminal, limits, geom):
    """A touch-down in the last 100 ms before the carried throw cannot be
    planned: once the hold rows own a knot there is no lateral freedom left
    to steer the cup to the touch-down site.

    FAIL-BEFORE (2026-09-29, constant forced to 0.0): the same terminal was
    refused as ``UNVERIFIED: QP solution failed verification (equality residual
    5.943e+01 ...)`` — infeasible either way, but not in words an operator can
    act on.
    """
    term = dataclasses.replace(
        catch_terminal,
        then_throw=sg.ThrowAfterCatch(
            t_release_s=catch_terminal.t_land_s + 0.05,
            site_mm=THROW_SITE_MM, target_mm=CATCH_SITE_MM, flight_s=T_F))
    with pytest.raises(uc.CycleInfeasible) as ei:
        sg.plan_segment(sg.CATCH, catch_seed, term, sg.SegmentConfig(),
                        limits, geom)
    assert 'catch comes too close to the next throw' in ei.value.outcome()
