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


def test_a_held_attitude_catch_cannot_carry_its_own_throw():
    """The refusal is at construction: a release needs its own take-off tilt."""
    tilt = sg.receive_hold_tilt(_BB_ARRIVAL)
    tt = sg.ThrowAfterCatch(t_release_s=1.3, site_mm=THROW_SITE_MM,
                            target_mm=THROW_SITE_MM, flight_s=T_F)
    with pytest.raises(ValueError, match='own take-off tilt'):
        sg.CatchTerminal(landing_mm=CATCH_SITE_MM,
                         landing_vel_mm_s=_BB_ARRIVAL, t_land_s=1.0,
                         rest_site_mm=REST_MM, then_throw=tt, hold_tilt=tilt)


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
