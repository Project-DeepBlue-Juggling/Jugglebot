"""I-PLAN-12 — a REBASED fresh REST starts where the machine is at its new origin.

``executor.install_segment``'s fresh-origin REBASE (2026-09-28) slides a REST's
origin past the wire when its solve outran ``WIRE_READ_KNOTS`` knots, on the
premise "a REST from rest is the same motion later".  The premise is false for
the post-Feed-catch settle REST: a fresh REST is dispatched while the catch
stroke still streams, so its seed is the Hand IN MOTION (~1.0 rev at
-18 rev/s).  On 2026-10-23 (17:30 launch, oval_1ball_fed) two of eight Feed
catches had an 82-87 ms solve, were REBASED +0.16 s with the stale in-motion
plan intact, and knot 0 put the Hand back at the seed: a +0.52 / +0.67 rev
commanded step 0.1 s after seating, inside the node's 1.0 rev install-
continuity bound (``logbook/2026-10-23-rest-rebase-stale-seed-hand-bump.md``).

The fix (``executor._rebase_fresh_rest``): an in-motion seed is PROPAGATED over
the slide by dropping the plan's first ``k = ceil(slide/dt)`` knots (``t0 =
t_now + k*dt``); an at-rest seed keeps the old rule bit for bit.  Either way
knot 0 is checked against the propagated state and a mismatch is refused
``REBASE_DISCONTINUOUS``.

Driven the way ``scratchpad/probe_rest_rebase.py`` drives ``install_segment``
(fresh branch, ``seed_rest``, ``t_install_s`` injecting the solve time).
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (pose_to_leg_lengths,
                                        rotvec_to_rot_matrix)
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from tests.motion.test_skills_install_origin import (DT, T0_ABS, WIRE_MARGIN_S,
                                                     _REST_MM, _catch_terminal,
                                                     _rest_state,
                                                     _rest_terminal)

#: The bag's settle REST length and its two solve times either side of the
#: 75 ms wire-read margin (85 ms ~ catches #2/#5's 84/87 ms; 60 ms clean).
REST_S = 1.25
SLOW_SOLVE_S = 0.085
FAST_SOLVE_S = 0.060


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=300.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3500.0)


def _seed(hand_rev, hand_vel, pose_off=None, pose_vel=None):
    s = _rest_state(_REST_MM)
    pose = s.pose.copy() if pose_off is None else s.pose + pose_off
    pv = np.zeros(6) if pose_vel is None else np.asarray(pose_vel, float)
    return dataclasses.replace(s, pose=pose, pose_vel=pv, hand_rev=hand_rev,
                               hand_vel_rps=hand_vel, cup_pos_mm=None,
                               cup_vel_mm_s=None, cup_accel_mm_s2=None)


#: The Feed-catch seed: the Hand riding down the catch stroke.
_MOVING = dict(hand_rev=1.0, hand_vel=-18.0)
#: The same with the platform moving too (the leg-lane exposure).
_LEGS_MOVING = dict(hand_rev=1.0, hand_vel=-18.0,
                    pose_off=np.array([3.0, 0.0, 0.0, 0.0, 0.01, 0.0]),
                    pose_vel=[20.0, 0.0, -60.0, 0.0, -0.05, 0.0])


def _install(limits, geom, seed, solve_s):
    return ex.install_segment(
        None, seed, sg.REST, _rest_terminal(T0_ABS + REST_S), T0_ABS,
        limits=limits, geom=geom, t_install_s=lambda: T0_ABS + solve_s)


def _motor_rev(pose, geom):
    pose = np.asarray(pose, float)
    return (pose_to_leg_lengths(pose[:3], rotvec_to_rot_matrix(pose[3:6]), geom)
            * np.asarray(geom.mm_to_rev, float))


def _assert_same_plan(a, b):
    for f in ('pose', 'pose_vel', 'hand_rev', 'hand_vel_rps'):
        np.testing.assert_array_equal(getattr(a, f), getattr(b, f), err_msg=f)
    assert a.dt == b.dt and a.n_knots == b.n_knots


def test_an_in_motion_seed_is_propagated_over_the_slide(limits, geom):
    """(a) The bag's case: seed 1.0 rev at -18 rev/s, 85 ms solve.  The origin
    lands on the solve's own grid at the first knot past the wire
    (``ceil((0.085 + 0.075)/dt) = 7`` -> +0.175 s), the plan is the un-delayed
    solve with its first 7 knots dropped (bit for bit), and knot 0 is the
    un-delayed plan's state at the new origin: zero Hand step (was +0.65 rev)."""
    seed = _seed(**_MOVING)
    _r, res0, seg0 = _install(limits, geom, seed, 0.0)       # same solve, no slide
    rec, res, seg = _install(limits, geom, seed, SLOW_SOLVE_S)
    assert res.accepted, res.message
    assert 'REBASED' in res.message and '7 knot(s) dropped' in res.message
    k = int(math.ceil((SLOW_SOLVE_S + WIRE_MARGIN_S) / DT - 1e-9))
    assert k == 7
    assert res.t0_s == pytest.approx(T0_ABS + k * DT, abs=1e-12)
    assert res.t0_s >= T0_ABS + SLOW_SOLVE_S + WIRE_MARGIN_S - 1e-12
    assert rec.t0_s == res.t0_s and rec.plan is seg.plan
    full = seg0.plan
    for f in ('pose', 'pose_vel', 'hand_rev', 'hand_vel_rps'):
        np.testing.assert_array_equal(getattr(seg.plan, f),
                                      getattr(full, f)[k:], err_msg=f)
    # The REST still ends on its schedule (the cut, unlike the old slide,
    # does not push the terminal later by the solve overrun).
    assert rec.end_s == pytest.approx(T0_ABS + full.total_duration, abs=1e-9)
    # Meta re-indexed with the plan.
    assert seg.meta.n_knots == seg.plan.n_knots == full.n_knots - k
    assert len(seg.meta.tilts) == seg.plan.n_knots
    assert seg.meta.cup_plan.pos.shape[0] == seg.plan.n_knots
    np.testing.assert_array_equal(seg.meta.cup_plan.pos, seg0.meta.cup_plan.pos[k:])
    # The knot-0 step against the propagated state, both lanes.
    tau = res.t0_s - T0_ABS
    h_ref, hv_ref = full.hand_at(tau)
    assert abs(seg.plan.hand_rev[0] - h_ref) < 0.05
    assert abs(seg.plan.hand_rev[0] - h_ref) < 1e-9
    assert abs(seg.plan.hand_vel_rps[0] - hv_ref) < 1e-9
    # What the pre-I-PLAN-12 slide would have put on the wire: knot 0 = seed.
    assert full.hand_rev[0] - h_ref > 0.5


def test_a_solve_inside_the_wire_margin_is_unchanged(limits, geom):
    """(b) 60 ms: no rebase, ``t0 == t_now``, the plan bit-identical to the
    zero-latency solve — the no-slide path is untouched by I-PLAN-12."""
    seed = _seed(**_MOVING)
    _r, _res0, seg0 = _install(limits, geom, seed, 0.0)
    rec, res, seg = _install(limits, geom, seed, FAST_SOLVE_S)
    assert res.accepted and 'REBASED' not in res.message, res.message
    assert res.t0_s == T0_ABS and rec.t0_s == T0_ABS
    _assert_same_plan(seg.plan, seg0.plan)


def test_an_at_rest_seed_keeps_the_whole_plan_and_the_old_origin(limits, geom):
    """(c) Seed at rest (0.293 rev, zero rates), 85 ms: the machine HOLDS the
    seed over the slide, so the REBASE is the pre-I-PLAN-12 rule bit for bit —
    NOT cut: ``t0 = t_inst + WIRE_READ_KNOTS*dt`` (off the solve grid, as
    before), the whole solved plan, knot 0 = the held seed.  (Cutting here would
    itself be a step: the plan walks the Hand 0.293 -> 0.342 rev over the
    slide while the real Hand stands still.)"""
    seed = _seed(0.293, 0.0)
    _r, _res0, seg0 = _install(limits, geom, seed, 0.0)
    rec, res, seg = _install(limits, geom, seed, SLOW_SOLVE_S)
    assert res.accepted and 'REBASED' in res.message, res.message
    assert 'dropped' not in res.message
    assert res.message.startswith('fresh origin REBASED +0.160 s')
    assert res.t0_s == pytest.approx(T0_ABS + SLOW_SOLVE_S + WIRE_MARGIN_S,
                                     abs=1e-9)
    _assert_same_plan(seg.plan, seg0.plan)
    assert seg.plan.hand_rev[0] == pytest.approx(0.293, abs=1e-9)


def test_a_moving_platform_seed_puts_no_step_on_the_legs(limits, geom):
    """(d) Legs AND Hand moving at the seed (20 mm/s x, -60 mm/s z, -0.05 rad/s):
    the old slide stepped the leg lanes too (probe: 1.7 mm / 8 mrad of pose);
    the propagated knot 0 matches the un-delayed plan in motor rev on every
    leg, far inside the firmware's 0.005 rev resume tolerance."""
    seed = _seed(**_LEGS_MOVING)
    _r, _res0, seg0 = _install(limits, geom, seed, 0.0)
    rec, res, seg = _install(limits, geom, seed, SLOW_SOLVE_S)
    assert res.accepted and 'dropped' in res.message, res.message
    tau = res.t0_s - T0_ABS
    pose_ref, _tw, _a = seg0.plan.state_at(tau)
    step = np.max(np.abs(_motor_rev(seg.plan.pose[0], geom)
                         - _motor_rev(pose_ref, geom)))
    assert step < ex.REBASE_TOL_LEG_REV
    stale = np.max(np.abs(_motor_rev(seg0.plan.pose[0], geom)
                          - _motor_rev(pose_ref, geom)))
    assert stale > ex.REBASE_TOL_LEG_REV      # what the old slide shipped


def test_a_rebase_that_does_not_propagate_the_seed_is_refused(
        limits, geom, monkeypatch):
    """(e) The enforcement: a REBASED plan whose knot 0 is not the propagated
    state is REFUSED ``REBASE_DISCONTINUOUS`` (nothing installed, record
    unchanged) — here by reinstating the old behaviour (slide the origin, keep
    the stale in-motion head) through ``tail_view``."""
    monkeypatch.setattr(uc, 'tail_view', lambda plan, meta, k: (plan, meta))
    rec, res, seg = _install(limits, geom, _seed(**_MOVING), SLOW_SOLVE_S)
    assert res.accepted is False
    assert res.code == ex.REBASE_DISCONTINUOUS, res.message
    assert 'hand position step' in res.message
    assert rec is None and seg is None


def test_the_rebase_check_names_a_leg_step(limits, geom):
    """(e') ``rebase_discontinuity`` measures the legs in motor rev: a 1 mm z
    offset on knot 0 (0.014 rev) is over the 0.005 rev bar and is named."""
    seed = _seed(0.293, 0.0)
    _r, _res, seg = _install(limits, geom, seed, 0.0)
    p = seg.plan
    assert ex.rebase_discontinuity(p, p.pose[0], p.pose_vel[0],
                                   p.hand_rev[0], p.hand_vel_rps[0],
                                   geom=geom) is None
    shifted = p.pose[0] + np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    msg = ex.rebase_discontinuity(p, shifted, p.pose_vel[0], p.hand_rev[0],
                                  p.hand_vel_rps[0], geom=geom)
    assert msg is not None and 'leg position step' in msg
    assert 'hand' not in msg


def test_tail_view_refuses_an_event_bearing_plan(limits, geom):
    """``unified_cycle.tail_view`` is for an event-free window only: dropping
    knots of a CATCH would move its touch-down."""
    seed = _rest_state(_REST_MM)
    _r, res, seg = ex.install_segment(
        None, seed, sg.CATCH, _catch_terminal(T0_ABS + 0.6), T0_ABS,
        limits=limits, geom=geom)
    assert res.accepted, res.message
    with pytest.raises(ValueError, match='event-free'):
        uc.tail_view(seg.plan, seg.meta, 2)


@pytest.mark.parametrize('hand_rev', [0.293, 0.0001, 0.5639])
def test_an_at_rest_rebase_with_a_levelling_frame_still_installs(
        limits, geom, hand_rev):
    """Audit (2026-10-23): the held branch is now CHECKED (knot 0 vs the seed at
    the firmware resume bars), where HEAD never refused an at-rest REBASE.  A
    real at-rest seed carries a non-None ``levelling_correction`` (5.8 mrad,
    the hop's measured offset) and full ``cup_*`` fields from
    ``CycleState.at_rest``, and its Hand may sit off the settle clamp (0.307
    rev): at the cup floor 0.293, at the ACTIVATE park 0.0001, at the
    2026-09-16 stale hold 0.5639.  The REBASE must still install, with the
    whole plan bit-identical to the zero-latency solve and HEAD's origin."""
    from jugglebot.motion import levelling
    base = _rest_state(_REST_MM)
    corr = levelling.correction_from_offset(0.0, 0.005809100317737187)
    seed = uc.CycleState.at_rest(base.pose, hand_rev,
                                 levelling_correction=corr)
    _r, res0, seg0 = _install(limits, geom, seed, 0.0)
    assert res0.accepted, res0.message
    rec, res, seg = _install(limits, geom, seed, SLOW_SOLVE_S)
    assert res.accepted, res.message
    assert 'REBASED' in res.message and 'dropped' not in res.message
    assert res.t0_s == pytest.approx(T0_ABS + SLOW_SOLVE_S + WIRE_MARGIN_S,
                                     abs=1e-9)
    _assert_same_plan(seg.plan, seg0.plan)
