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
