"""``trajectory/install_segment`` — the skill stack's ONE install path (R2, Unit D2).

Drives the service directly against a real ``TrajectoryNode`` (ROS mocked by
``tests/ros/conftest.py``). The planning/splice-vs-fresh-origin logic itself is
``jugglebot.motion.skills.executor.install_segment`` (pure Python) and is
covered by ``tests/motion/test_skills_executor.py`` (Unit B) — what this file
pins is the ROS SHELL: guard ladder, epoch/hold interaction, the ROS-clock
crossing, and that a refusal never touches the active plan.

T-I1 (the real :5557 wire seam), the hold-reentrancy wiring pin and the
node-boot MultiThreadedExecutor pin below were ported from the deleted
``test_unified_cycle_integration.py`` / ``test_trajectory_hold_preempts_solve.py``
at the FSM deletion (R4, 2026-09-24, unit U6b cluster B) — re-driven through
``trajectory/install_segment`` instead of the retired ``trajectory/plan_cycle``.
The fixtures below (``_cycle_node``, ``_frozen_perf``, ``_refresh``,
``_robot_state``) were moved here from ``test_unified_cycle_integration.py`` for
the same reason: this file is now their only consumer.
"""

from __future__ import annotations

import contextlib
import dataclasses
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from std_srvs.srv import Trigger

from jugglebot_interfaces.msg import MotorStateSingle, RobotState
from jugglebot_interfaces.srv import InstallSegment
import jugglebot.hardware_config as hw
from jugglebot import skill_node as sn
from jugglebot import trajectory_node as tn
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.skills import executor as sk_exec
from jugglebot.motion.skills import segments as sk_seg
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory.cycle_plan import CyclePlan

from tests.ros.test_trajectory_node import _HandHoldPlan, _link_status


# ── Geometry the LAUNCH is planned at ─────────────────────────────────────────
# `sim/cycle_gate.py`'s Phase-1 point, in the frame `CycleGoals` uses. The rest
# cup sits at 750 mm — INSIDE the QP's slider-reachable box (0.6896…0.9846 m),
# which the hand's homed 0.0 rev is NOT (679.6 mm, 10 mm below the floor). So the
# fixtures below seed a real hand position rather than the default zero; a test
# that forgot to would refuse with a cup-box error and read as a planner fault.
_REST_CUP_Z_MM = 750.0


def _hand_rev_for_cup_z(cup_z_mm: float) -> float:
    """The slider rev whose LEVEL realisation puts the cup opening at ``cup_z_mm``.

    Built from the level relation ``cup_z = CUP_Z_BASE_MM + slider_mm`` rather
    than by inverting the forward map, so the fixture does not depend on the map
    the tests exercise.
    """
    cfg = cr.RealizeConfig()
    slider_mm = float(cup_z_mm) - cfg.cup_z_base_mm
    return (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M


_REST_HAND_REV = _hand_rev_for_cup_z(_REST_CUP_Z_MM)

# Sitting limits a unified sitting raises to at start (plan Phase 1, owner
# decision 1) — the ones `sim/cycle_gate.py` and `tests/motion/test_unified_cycle`
# both run at.
_SESSION_VEL = 250.0
_SESSION_ACC = 3000.0
_SESSION_JERK = 150000.0


def _robot_state(hand_rev=_REST_HAND_REV, is_homed=True):
    """A SEVEN-axis robot_state: six legs at the ACTIVE pose plus the hand."""
    from tests.ros.test_trajectory_node import _ACTIVATE_REV
    rs = RobotState()
    rs.motor_states = [MotorStateSingle(pos_estimate=float(_ACTIVATE_REV[i]))
                       for i in range(6)]
    rs.motor_states.append(MotorStateSingle(pos_estimate=float(hand_rev)))
    rs.is_homed = bool(is_homed)
    return rs


def _cycle_node(**kw):
    """A seeded, TRAJECTORY-mode node at sitting limits, emitter NOT started."""
    from jugglebot_interfaces.srv import SetTrajectoryLimits
    node = tn.TrajectoryNode(start_emitter=False, **kw)
    node._on_robot_state(_robot_state())
    from std_msgs.msg import String
    node._on_control_mode(String(data='TRAJECTORY'))
    req = SetTrajectoryLimits.Request()
    req.leg_vel_limit_mmps = _SESSION_VEL
    req.leg_acc_limit_mmps2 = _SESSION_ACC
    req.leg_jerk_limit_mmps3 = _SESSION_JERK
    node._svc_set_limits(req, SetTrajectoryLimits.Response())
    return node


def _refresh(node):
    """Re-stamp the robot_state freshness window.

    NOT a fudge: on the real graph ``robot_state`` arrives at 100 Hz, so it is
    never more than 10 ms old at a service entry. Here a single solve+gate is
    enough to walk past the node's 0.5 s staleness bound, and back-to-back
    planning calls in one test would then refuse STALE_STATE for a reason that
    exists only in the harness. Re-stamping between calls reproduces the live
    graph rather than relaxing the guard.
    """
    node._on_robot_state(_robot_state())
    return node


@contextlib.contextmanager
def _frozen_perf():
    """Freeze ``time.perf_counter()`` at entry — for the NODE and for this module.

    Not a convenience. An assertion here samples a plan at a ``tau`` captured on
    one line and compares it against ``_current_state()``, which captures its
    own ``perf_counter()`` on the next (any preemption between them reads as
    drift). Freezing makes both exact instead of approximately true.

    ``trajectory_node`` does ``import time`` and calls ``time.perf_counter()``
    through the module, and this file imported the same module object, so setting
    the attribute reaches both. That is process-wide for the duration, which is
    safe here and nowhere near safe in general: these nodes are built with
    ``start_emitter=False`` so no 40 Hz thread is reading the clock, ROS is
    mocked so no timer is either, and pytest runs one test at a time per xdist
    worker. Restore is in a ``finally``.
    """
    real = time.perf_counter
    at = real()
    time.perf_counter = lambda: at
    try:
        yield at
    finally:
        time.perf_counter = real


def _throw_req(t_event_s, site_z=860.0, flight_s=0.6, ball_id=0):
    req = InstallSegment.Request()
    req.kind = req.KIND_THROW
    req.ball_id = ball_id
    req.t_event_s = float(t_event_s)
    req.site_mm = [0.0, 0.0, float(site_z)]
    req.target_mm = [0.0, 0.0, float(site_z)]
    req.flight_s = float(flight_s)
    return req


def _catch_req(t_event_s, site_z=830.0, vel_z=-2500.0, rest_z=750.0, ball_id=0):
    req = InstallSegment.Request()
    req.kind = req.KIND_CATCH
    req.ball_id = ball_id
    req.t_event_s = float(t_event_s)
    req.site_mm = [0.0, 0.0, float(site_z)]
    req.landing_vel_mm_s = [0.0, 0.0, float(vel_z)]
    req.rest_site_mm = [0.0, 0.0, float(rest_z)]
    return req


def _catch_throw_req(t_event_s, t_release_s, site_z=830.0, vel_z=-2500.0,
                     rest_z=750.0, release_site_z=860.0, target_z=860.0,
                     flight_s=0.5, ball_id=0):
    """A CATCH-with-throw request: the touch-down at ``t_event_s``, the
    same-site release it carries at ``t_release_s`` (both wall clock)."""
    req = InstallSegment.Request()
    req.kind = req.KIND_CATCH
    req.ball_id = ball_id
    req.t_event_s = float(t_event_s)
    req.site_mm = [0.0, 0.0, float(site_z)]
    req.landing_vel_mm_s = [0.0, 0.0, float(vel_z)]
    req.rest_site_mm = [0.0, 0.0, float(rest_z)]
    req.t_release_s = float(t_release_s)
    req.release_site_mm = [0.0, 0.0, float(release_site_z)]
    req.target_mm = [0.0, 0.0, float(target_z)]
    req.flight_s = float(flight_s)
    return req


def _rest_req(t_event_s, rest_z=750.0, ball_id=0):
    req = InstallSegment.Request()
    req.kind = req.KIND_REST
    req.ball_id = ball_id
    req.t_event_s = float(t_event_s)
    req.rest_site_mm = [0.0, 0.0, float(rest_z)]
    return req


def _perf_node(**kw):
    """A ``_cycle_node`` with the ROS<->perf offset zeroed.

    ``t_event_s`` is nominally a ROS-clock instant; zeroing the offset makes it
    directly comparable to ``time.perf_counter()`` reads in the test, without
    exercising ``clock_offset``'s own estimator (covered elsewhere).
    """
    node = _cycle_node(**kw)
    node._ros_to_perf_offset = 0.0
    return node


# ═════════════════════════════════════════════════════════════════════════════
# Fresh origin
# ═════════════════════════════════════════════════════════════════════════════

def test_throw_from_rest_accepts_at_a_fresh_origin():
    """The live path reserves the dispatch lead for an event-bearing fresh
    origin (``install_segment(..., reserve_fresh_lead=True)``, 2026-10-02):
    ``t0`` is the grid-aligned request instant PLUS the THROW's reserved
    lead (``FRESH_THROW_LEAD_S``, 0.325 s since 2026-10-16), not the
    request instant — pinned with an ABSOLUTE one-knot tolerance because a
    bare ``approx(at)`` on a large ``perf_counter`` value has a relative
    tolerance wider than the 0.325 s it must tell apart."""
    node = _perf_node()
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is True, resp.message
    assert resp.code == feas.OK
    assert resp.splice_k == 0
    # t0 = the node's grid-aligned request instant + the THROW's reserved
    # lead (FRESH_THROW_LEAD_S, 13 knots since 2026-10-16 -- the lead the
    # schedule dispatches a THROW by): within one knot of `at + lead` (the
    # grid alignment is the node's, not this test's).
    assert sk_exec.FRESH_THROW_LEAD_S == pytest.approx(0.325)
    assert resp.t0_mono - sk_exec.FRESH_THROW_LEAD_S == pytest.approx(
        at, abs=float(hw.JB_TRAJ_KNOT_DT_S) + 1e-9)
    # The release is still the requested absolute instant, to the knot the
    # planner places it on.
    assert resp.t_event_mono == pytest.approx(
        at + 0.6, abs=float(hw.JB_TRAJ_KNOT_DT_S) + 1e-9)
    assert isinstance(node._active_plan, CyclePlan)
    assert node._cycle is not None
    assert node._cycle[0] is node._active_plan


def test_a_rest_segment_accepts_at_a_fresh_origin():
    node = _perf_node()
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_rest_req(at + 0.5),
                                         InstallSegment.Response())
    assert resp.accepted is True, resp.message
    assert resp.splice_k == 0


# ═════════════════════════════════════════════════════════════════════════════
# A fresh THROW/CATCH dispatched while the previous REST is STILL STREAMING
# (2026-10-02, unit U2, `scratchpad/design_fresh_origin_budget.md`)
#
# A REST keeps moving right up to its own last knot (2.4-33 mm/s 25 ms before
# the end on the schedule's own park/pretilt moves — probe
# `scratchpad/probe_rest_tail.py/.out`), and the reload CATCH dispatches
# exactly at the pre-tilt REST's NOMINAL end (`schedule.py`'s
# `pretilt_end_rel`, also the CATCH's own `dispatch_s`). A REBASED pre-tilt
# REST (R5 sitting 2: +0.16-0.18 s) ends AFTER that nominal instant, so the
# record is still on the wire when the next install lands. `_cycle_start_state`'s
# live rest bound (0.3 mm/s platform, 0.2 rev/s hand) used to read that as
# "the machine is moving" and refuse every such install fail-closed; the fix
# seeds the fresh origin from the STILL-STREAMING record's own terminal rest
# instead, because `fresh`'s own test (`t_now + lead_s >= record.end_s`) is
# exactly the claim `t0 >= record.end_s` the reservation needs.
# ═════════════════════════════════════════════════════════════════════════════

def test_a_fresh_throw_installs_while_the_previous_rest_is_still_streaming():
    """A REST to a different cup height (hand moving, platform untouched) is
    installed over 1.0 s; a THROW is then dispatched 0.2 s into it — inside
    the THROW's 0.325 s reserved lead (FRESH_THROW_LEAD_S since 2026-10-16;
    0.225 s before) of the REST's own end (`fresh` fires), and the
    REST's own plan is still short of ITS end (0.2 s < 1.0 s duration) when
    this handler runs, so the record is provably still streaming. The
    install must ACCEPT: the seed is the REST's TERMINAL rest (exact zero
    velocity), not the live, still-moving hand — the live hand is checked
    below to be well over the 0.2 rev/s bound that used to gate this, so the
    old live-rest test would have refused it."""
    node = _perf_node()
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_rest_req(at + 1.0, rest_z=850.0),
                                         InstallSegment.Response())
        assert resp.accepted is True, resp.message
    rest_end = resp.t0_mono + resp.duration_s

    real_perf = time.perf_counter
    t2 = rest_end - 0.2
    time.perf_counter = lambda: t2
    try:
        _refresh(node)
        _, hand_vel = node._commanded_hand_state()
        assert abs(hand_vel) > 1.0, (
            'fixture must still be moving at t2 for this test to mean '
            'anything: hand_vel=%.4f' % hand_vel)
        resp2 = node._svc_install_segment(_throw_req(t2 + 0.6),
                                          InstallSegment.Response())
    finally:
        time.perf_counter = real_perf
    assert resp2.accepted is True, resp2.message
    assert resp2.code == feas.OK
    assert resp2.splice_k == 0


def test_a_fresh_throw_refuses_when_nothing_is_streaming_and_the_hand_moves():
    """The mirror case: with NO record streaming (the lane held — `node._cycle`
    stays `None`, only `_active_plan` set) the live at-rest test is exactly
    as it always was. A moving hand with no cup track to read refuses
    `_IN_MOTION` before any seed is built — the same refusal
    `test_a_moving_machine_is_never_reconciled` pins directly on
    `_cycle_start_state`, driven here through the full service."""
    node = _reconcile_node(commanded_hand_rev=0.5639, measured_hand_rev=0.0001)
    with node._plan_lock:
        node._active_plan = _HandHoldPlan(node._last_pose.copy(),
                                          lambda tau: (0.5639, 5.0))
        node._plan_t0 = time.perf_counter()
    assert node._cycle is None
    resp = node._svc_install_segment(_throw_req(time.perf_counter() + 0.6),
                                     InstallSegment.Response())
    assert resp.accepted is False
    assert resp.code == tn._IN_MOTION
    assert 'hand 5.0000 rev/s' in resp.message


def test_a_fresh_rest_while_the_previous_rest_still_streams_seeds_from_the_live_state():
    """A fresh REST is NOT lead-reserved (`install_segment`: `reserved = ...
    and kind != REST`, its origin is the dispatch instant itself), so the
    seed-skip above must not apply to it (audit, 2026-10-02): a second REST
    dispatched 0.2 s before the first one ends starts NOW, from the live,
    still-moving hand — `_cycle_start_state` keeps building that seed — and
    its knot 0 therefore sits on the commanded hand of the moment, not on the
    first REST's terminal rest (0.2 s of tail motion away, inside the 1.0 rev
    drift bound `_install_continuity_ok` would let through)."""
    node = _perf_node()
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_rest_req(at + 1.0, rest_z=850.0),
                                         InstallSegment.Response())
        assert resp.accepted is True, resp.message
    rest_end = resp.t0_mono + resp.duration_s

    real_perf = time.perf_counter
    t2 = rest_end - 0.2
    time.perf_counter = lambda: t2
    try:
        _refresh(node)
        hand_before, hand_vel = node._commanded_hand_state()
        assert abs(hand_vel) > 1.0, (
            'fixture must still be moving at t2: hand_vel=%.4f' % hand_vel)
        resp2 = node._svc_install_segment(_rest_req(t2 + 1.0, rest_z=800.0),
                                          InstallSegment.Response())
        assert resp2.accepted is True, resp2.message
        assert resp2.splice_k == 0
        # Unreserved: the origin is the dispatch instant (one knot of grid
        # snapping), not `t2 + LEAD_S`.
        assert resp2.t0_mono == pytest.approx(t2, abs=float(hw.JB_TRAJ_KNOT_DT_S) + 1e-9)
        _refresh(node)
        hand_after, _ = node._commanded_hand_state()
    finally:
        time.perf_counter = real_perf
    # Knot 0 of the new plan IS the live hand of the dispatch instant.
    assert hand_after == pytest.approx(hand_before, abs=0.05), (
        'the fresh REST was seeded %.3f rev away from the live hand'
        % (hand_after - hand_before))


def _fast_rest_then_fresh_throw(during_solve=None):
    """A 900 mm REST (fast enough to be ~150 mm / several rev/s from its own
    terminal with 0.225 s left), then a fresh THROW dispatched at ``rest_end -
    0.225`` -- inside the THROW's 0.325 s lead, so ``fresh`` fires and the
    THROW's origin lands 0.1 s AFTER the REST ends. ``during_solve(node)``
    runs inside the solve (the executor seam the in-process solver double
    looks up at call time). Returns ``(node, resp2, rest_plan, rest_t0, t2)``."""
    node = _perf_node()
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_rest_req(at + 0.5, rest_z=900.0),
                                         InstallSegment.Response())
        assert resp.accepted is True, resp.message
    rest_plan, _meta, rest_t0 = node._cycle
    rest_end = resp.t0_mono + resp.duration_s
    real_perf, real_install = time.perf_counter, tn.sk_exec.install_segment

    def hooked(*args, **kwargs):
        if during_solve is not None:
            during_solve(node)
        return real_install(*args, **kwargs)

    t2 = rest_end - 0.225
    time.perf_counter = lambda: t2
    tn.sk_exec.install_segment = hooked
    try:
        _refresh(node)
        resp2 = node._svc_install_segment(_throw_req(t2 + 0.6),
                                          InstallSegment.Response())
    finally:
        time.perf_counter = real_perf
        tn.sk_exec.install_segment = real_install
    return node, resp2, rest_plan, rest_t0, t2


def _legs(node, pose):
    return node._pose_to_motor_rev(pose)


def test_a_fresh_throw_on_a_fast_rest_tail_streams_the_tail_to_its_origin():
    """I-INSTALL-1. Until 2026-10-16 this case REFUSED ``STALE_STATE``: the
    install swapped the wire onto the THROW's knot 0 (the REST's terminal
    rest), a step of > 1.0 rev of hand from the still-moving tail, and only
    ``_install_continuity_ok`` stood between it and the pump. With the
    handoff the REST plays on to the THROW's origin, so nothing steps and
    the install is ACCEPTED (the step was the HAND's, the 900 mm REST's legs
    barely move): the continuity guard now asks whether the
    running REST ARRIVES on knot 0 by ``t0`` (it does -- rest-terminal by
    construction, plan § 0). The planning head is the THROW from install;
    the stream view (``_current_state``) is the REST until ``t0``."""
    node, resp2, rest_plan, rest_t0, t2 = _fast_rest_then_fresh_throw()
    assert resp2.accepted is True, resp2.message
    t0 = resp2.t0_mono
    assert t0 > rest_t0 + rest_plan.total_duration        # holds the rest between
    view, _ = node._stream_plan_locked()
    assert isinstance(view, tn.StreamHandoff)
    assert view.head is node._active_plan is node._cycle[0]
    # at the install instant the wire is still on the moving REST tail ...
    np.testing.assert_allclose(
        _legs(node, node._current_state(at_mono=t2)[0]),
        _legs(node, rest_plan.state_at(t2 - rest_t0)[0]), atol=1e-12)
    hand_tail = rest_plan.hand_at(t2 - rest_t0)[0]
    assert node._commanded_hand_state(at_mono=t2)[0] == pytest.approx(
        hand_tail, abs=1e-12)
    knot0_hand = node._active_plan.hand_at(0.0)[0]
    # the hand step the pre-fix install would have put on the wire
    assert abs(hand_tail - knot0_hand) > 1.0
    # ... and just before t0 the stream is exactly the THROW's knot 0
    np.testing.assert_allclose(
        _legs(node, node._current_state(at_mono=t0 - 1e-9)[0]),
        _legs(node, node._active_plan.state_at(0.0)[0]), atol=1e-9)
    assert node._commanded_hand_state(at_mono=t0 - 1e-9)[0] == pytest.approx(
        knot0_hand, abs=1e-9)


def test_a_stop_landing_during_a_fresh_throw_solve_still_refuses_stale_state():
    """The guard half of I-INSTALL-1: the continuity question moves to ``t0``
    but still fires when the running record will NOT arrive on knot 0. A
    graceful stop installed while the THROW solves (seeded off the moving
    REST tail, decelerating IN PLACE) leaves the wire ~150 mm short of the
    REST terminal the THROW was seeded from -> ``STALE_STATE``."""
    node, resp2, _rp, _rt0, _t2 = _fast_rest_then_fresh_throw(
        during_solve=lambda n: n._install_graceful_stop('test'))
    assert resp2.accepted is False
    assert resp2.code == feas.STALE_STATE
    assert 'commanded state moved during planning' in resp2.message


def test_a_direct_active_plan_assignment_retires_the_handoff_by_identity():
    """I-INSTALL-1: the handoff is bound to its head plan by identity, so the
    emitter's backstop freeze (a bare ``_active_plan`` / ``_plan_t0``
    assignment, copied from ``_emit_once``) drops it with no extra write."""
    node, resp2, _rp, _rt0, t2 = _fast_rest_then_fresh_throw()
    assert resp2.accepted is True, resp2.message
    with node._plan_lock:
        assert node._handoff is not None
        assert node._stream_plan_locked()[0] is node._handoff
        pose = node._last_pose
        node._active_plan = tn.HoldPlan(pose)
        node._plan_t0 = t2
        assert node._stream_plan_locked()[0] is node._active_plan


def test_a_fresh_origin_before_the_running_records_end_refuses_stale_state(
        monkeypatch):
    """I-INSTALL-1 fence (``_svc_install_segment``): a solver reply whose
    origin precedes the running record's end would truncate it at the switch."""
    node = _perf_node()
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_rest_req(at + 0.5, rest_z=900.0),
                                         InstallSegment.Response())
        assert resp.accepted is True, resp.message
    record = node._cycle[0], node._cycle[1], node._cycle[2]
    rest_end = resp.t0_mono + resp.duration_s
    real_solve = node._segment_solver.solve

    def early(req):
        reply = real_solve(req)
        nr = dataclasses.replace(reply.new_record,
                                 t0_s=rest_end - 0.05)
        return dataclasses.replace(reply, new_record=nr)

    monkeypatch.setattr(node._segment_solver, 'solve', early)
    real_perf = time.perf_counter
    time.perf_counter = lambda: rest_end - 0.225
    try:
        _refresh(node)
        resp2 = node._svc_install_segment(_throw_req(rest_end + 0.375),
                                          InstallSegment.Response())
    finally:
        time.perf_counter = real_perf
    assert record is not None
    assert resp2.accepted is False
    assert resp2.code == feas.STALE_STATE
    assert "precedes the running record's end" in resp2.message


def test_a_graceful_stop_between_install_and_origin_wins_from_the_live_tail():
    """A graceful stop requested after the THROW installed but before its
    origin RETIRES the handoff and takes the wire from where it really is --
    the REST tail (the stream view), not the THROW's knot 0 -- so it neither
    steps nor lets the THROW start."""
    node, resp2, rest_plan, rest_t0, t2 = _fast_rest_then_fresh_throw()
    assert resp2.accepted is True, resp2.message
    t3 = t2 + 0.05
    assert t3 < resp2.t0_mono
    real_perf = time.perf_counter
    time.perf_counter = lambda: t3
    try:
        node._install_graceful_stop('test')
        stop = node._active_plan
        assert not isinstance(stop, CyclePlan) and node._cycle is None
        view, _ = node._stream_plan_locked()
        assert view is stop                       # handoff retired
        np.testing.assert_allclose(
            _legs(node, stop.state_at(0.0)[0]),
            _legs(node, rest_plan.state_at(t3 - rest_t0)[0]), atol=1e-6)
    finally:
        time.perf_counter = real_perf


def test_a_catch_dispatched_before_the_throw_origin_splices_into_the_throw():
    """The planning head is the new record FROM INSTALL: the columns' first
    CATCH dispatches 0.300 s after THROW 0, before THROW 0's origin, and must
    splice into the THROW record (same origin), while the wire keeps playing
    the REST until that origin (the splice re-wraps the pending handoff)."""
    from jugglebot.motion.skills import schedule as sc
    from jugglebot.motion.skills import sites as st
    site = st.columns_sites(125.0)[0]
    sched = sc.compile_reload_wait(site, sc.FLOOR_LIFT_S, 0.0, holds_ball=True)
    node = _perf_node()
    resp2, tail = _schedule_rest_then_throw_at(
        node, sched.skills[0], site, sc.FRESH_THROW_LEAD_S, 0.0)
    assert resp2.accepted is True, resp2.message
    rest_plan, rest_t0 = tail['rest']
    throw_t0 = resp2.t0_mono
    t3 = throw_t0 - 0.025
    t_release = resp2.t_event_mono
    real_perf = time.perf_counter
    time.perf_counter = lambda: t3
    try:
        _refresh(node)
        req = _catch_req(t_release + 0.857)
        req.site_mm = [float(v) for v in site.catch_site_mm()]
        req.rest_site_mm = [float(v) for v in site.rest_site_mm()]
        resp3 = node._svc_install_segment(req, InstallSegment.Response())
        assert resp3.accepted is True, resp3.message
        assert resp3.t0_mono == throw_t0          # spliced: the THROW's origin
        assert resp3.splice_k > 0
        view, _ = node._stream_plan_locked()
        assert isinstance(view, tn.StreamHandoff)
        assert view.head is node._active_plan is node._cycle[0]
        np.testing.assert_allclose(
            _legs(node, node._current_state()[0]),
            _legs(node, rest_plan.state_at(t3 - rest_t0)[0]), atol=1e-12)
    finally:
        time.perf_counter = real_perf


def _schedule_rest_then_throw_at(node, rest_skill, site, lead_s,
                                 hand_start_rev):
    """Install ``rest_skill`` (a REST the schedule compilers build) fresh,
    then dispatch a launch THROW at ``rest_end - lead_s + 1e-3`` -- one ms
    after the instant the THROW's ``fresh`` test first fires, i.e. the most
    REST tail the THROW can ever take over (the frozen clock makes the solve
    free, so the install lands at dispatch). Returns ``(resp2, tail)``:
    ``tail`` is the step the install puts on the wire, measured the way
    ``_install_continuity_ok`` measures it -- the streaming REST's commanded
    state at the install instant against its terminal rest (= the THROW's
    knot 0, which the emitter plays from install to ``t0``)."""
    node._on_robot_state(_robot_state(hand_rev=hand_start_rev))
    with _frozen_perf() as at:
        req = _rest_req(at + rest_skill.window_s)
        req.rest_site_mm = [float(v) for v in rest_skill.site.rest_site_mm()]
        resp = node._svc_install_segment(req, InstallSegment.Response())
        assert resp.accepted is True, resp.message
    rest_end = resp.t0_mono + resp.duration_s
    rest_plan, _meta, rest_t0 = node._cycle
    t2 = rest_end - lead_s + 1e-3
    pose_now, twist_now, _ = rest_plan.state_at(t2 - rest_t0)
    pose_end, _tw, _ = rest_plan.state_at(rest_end - rest_t0)
    hand_now, hand_vel_now = rest_plan.hand_at(t2 - rest_t0)
    hand_end, _hv = rest_plan.hand_at(rest_end - rest_t0)
    tail = {
        'leg_rev': float(np.max(np.abs(node._pose_to_motor_rev(pose_now)
                                       - node._pose_to_motor_rev(pose_end)))),
        'leg_rev_s': float(np.max(np.abs(
            node._pose_twist_to_motor_rev_s(pose_now, twist_now)))),
        'hand_rev': abs(float(hand_now) - float(hand_end)),
        'hand_rev_s': abs(float(hand_vel_now)),
    }
    real_perf = time.perf_counter
    time.perf_counter = lambda: t2
    try:
        node._on_robot_state(_robot_state(hand_rev=float(hand_end)))
        req2 = _throw_req(t2 + lead_s + 0.4)
        req2.site_mm = [float(v) for v in site.throw_site_mm()]
        req2.target_mm = [float(v) for v in site.catch_site_mm()]
        req2.flight_s = 0.857
        resp2 = node._svc_install_segment(req2, InstallSegment.Response())
    finally:
        time.perf_counter = real_perf
    tail['wire'] = (_wire_trace(node, t2, resp2.t0_mono + 4 * float(
        hw.JB_TRAJ_KNOT_DT_S)) if resp2.accepted else [])
    tail['rest'] = (rest_plan, rest_t0)
    return resp2, tail


def _wire_trace(node, t_from, t_to):
    """Drive the REAL emitter (``_emit_once``) tick by tick, clock frozen at
    each grid tick, for every lead knot ``T`` in ``[t_from, t_to]``; returns
    ``[(T, motor_rev, hand_rev)]`` as shipped. ``T = t_k + L*dt`` (the C2FF
    emit lead) -- the instant each frame's u0 plays at."""
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    lead = float(hw.JB_TRAJ_EMIT_LEAD_KNOTS) * dt
    cap = SimpleNamespace(frames=[])
    cap.send = cap.frames.append
    cap.close = lambda: None
    node._pub = cap
    real_perf = time.perf_counter
    out = []
    k = int(math.ceil((t_from - lead) / dt))
    try:
        while k * dt + lead <= t_to + 1e-9:
            tick = k * dt
            time.perf_counter = lambda: tick
            n0 = len(cap.frames)
            node._emit_once(tick)
            if len(cap.frames) > n0:
                f = cap.frames[-1]
                out.append((tick + lead, np.asarray(f['motor_rev'], float),
                            f.get('hand_rev')))
            k += 1
    finally:
        time.perf_counter = real_perf
    return out


def test_a_launch_throw_at_the_fresh_throw_lead_takes_over_the_opening_lift_tail():
    """I-INSTALL-1 (2026-10-16): THROW 0 dispatches ``FRESH_THROW_LEAD_S``
    (13 knots) before the REST it follows ends, so its install lands while
    that REST is still streaming its last 0.325 s (less the solve). The wire
    must play that REST tail on to the THROW's origin ``t0`` and switch
    THERE -- never step onto the REST's terminal rest at install.

    Pinned on the REAL shape THROW 0 follows in ``compile_rest_columns`` /
    the fed-columns bridge: ``compile_reload_wait``'s opening REST from the
    ACTIVATE park (hand 0.0 rev) onto columns site P1 over ``FLOOR_LIFT_S``,
    worst case (zero solve: dispatch at ``rest_end - lead + 1 ms``, so the
    whole lead of tail is still to play). Driver: the REAL handler, then the
    REAL emitter ticked on the grid with the clock frozen per tick
    (``_wire_trace``) from the install to ``t0 + 4`` knots.

    MEASURED 2026-10-16 (``-s``) -- ``tail`` is the step the pre-fix emitter
    put on the wire at install (the REST's commanded state there vs its
    terminal rest = the THROW's knot 0): 0.0403 / 0.0512 / 0.0550 rev at
    9 / 12 / 13 knots (67 / 85 / 92 % of the 0.06 rev guard). With the
    handoff the largest per-knot leg change anywhere in the trace is the
    REST's own tail motion, and the change across the switch knot is 0.
    A DECAY REST (tilt decay after a reload catch) is NOT covered here."""
    from jugglebot.motion.skills import schedule as sc
    from jugglebot.motion.skills import sites as st
    site = st.columns_sites(125.0)[0]
    sched = sc.compile_reload_wait(site, sc.FLOOR_LIFT_S, 0.0, holds_ball=True)
    hand_start = 0.0
    rest_skill = sched.skills[0]
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    for knots in (sc.LEAD_KNOTS, 12, sc.FRESH_THROW_LEAD_KNOTS):
        lead = knots * dt
        node = _perf_node()
        resp2, tail = _schedule_rest_then_throw_at(
            node, rest_skill, site, lead, hand_start)
        assert resp2.accepted is True, resp2.message
        rest_plan, rest_t0 = tail['rest']
        wire = tail['wire']
        t0 = resp2.t0_mono
        pre = [w for w in wire if w[0] < t0 - 1e-9]
        post = [w for w in wire if w[0] >= t0 - 1e-9]
        assert pre and post
        # (a) before t0 the wire is the REST, exactly (no knot-0 pre-emption)
        for T, rev, hand in pre:
            want = node._pose_to_motor_rev(rest_plan.state_at(T - rest_t0)[0])
            np.testing.assert_allclose(rev, want, atol=1e-9)
            assert hand == pytest.approx(rest_plan.hand_at(T - rest_t0)[0],
                                         abs=1e-9)
        # (b) the switch knot carries no step: last REST frame -> knot 0
        switch_leg = float(np.max(np.abs(post[0][1] - pre[-1][1])))
        switch_hand = abs(float(post[0][2]) - float(pre[-1][2]))
        steps = [float(np.max(np.abs(b[1] - a[1])))
                 for a, b in zip(wire, wire[1:])]
        print('lead %d knots: old install step leg %.4f rev, hand %.4f rev | '
              'switch leg %.2e rev, hand %.2e rev; max per-knot leg %.4f rev '
              '(%d tail frames, t0-first %.3f, t0-T_last_pre %.3f)' % (
                  knots, tail['leg_rev'], tail['hand_rev'], switch_leg,
                  switch_hand, max(steps), len(pre), t0 - wire[0][0], t0 - pre[-1][0]))
        assert switch_leg < 1e-3
        assert switch_hand < 1e-3
        # the ONLY leg motion before t0 is the REST's own, knot to knot
        assert max(steps[:len(pre)]) < tail['leg_rev']
        # the head is the THROW from install (planning readers see it)
        assert node._cycle[0] is node._active_plan


# ═════════════════════════════════════════════════════════════════════════════
# R5 sitting 6 (2026-10-05): the levelling REST after a HIGH catch, replayed
# ═════════════════════════════════════════════════════════════════════════════
# The one reload attempt of that sitting caught Ball Butler's ball and then
# ended `STALE_STATE` at the REST: "hand position drift 1.2600 rev > 1.0000",
# seed "IN MOTION ... hand -34.6888 rev/s". A REST is dispatched a lead before
# its scheduled start and is given no lead by `install_segment`, so after a
# CATCH it takes the machine over mid-runway from the moving hand; the guard
# measures the hand's travel across the solve. At the 930 mm catch plane the
# runway is ~100 mm longer in the same time, so the hand is ~10 rev/s faster
# there than at 830 (15-25 rev/s in sittings 4-5) and a 36 ms solve crosses
# the bound. The guard is untouched; `SkillExecutor` retries the REST each
# tick until its scheduled start. This test is that whole chain offline: the
# REAL executor, the REAL handler, planner and guard, a clock that charges
# each install the solve time the robot measured.
#
# PINNED at 930 mm deliberately (2026-10-06 ripple, catch plane reverted to
# 830 -- plan `two-ball-skill-stack.md`): rewriting the harness to follow the
# live ``sites.CATCH_CUP_Z_MM`` (now 830) with the matching landing speed
# (sqrt(2 g (860 + 950 - 830)/1000) ~= 4.38 m/s, vs 930's ~5.52 m/s) was
# tried first and makes the refusal NEVER reproduce, even at the slowest
# solve ever measured on the robot (73 ms) -- the shorter 830 mm runway
# doesn't move the hand far enough to cross the 1.0 rev deviation bound.
# That is itself the useful finding (reverting catch plane also removes the
# STALE_STATE/REST-RETRY risk catch-high introduced), but it leaves nothing
# for these three tests to characterise. Freezing the scenario at the
# historical 930 mm / -5520 mm/s sitting-6 numbers (same pattern as
# ``test_skills_admissible.py``'s ``_TINY_SWEEP_CATCH_Z_MM`` and
# ``test_skills_plan_bench.py``'s ``r3_catch_plane``) keeps this harness
# testing the REST-RETRY mechanism itself rather than testing that nothing
# happens at today's default.

def _sitting6_rest_after_a_high_catch(solve_s):
    from jugglebot_interfaces.srv import SetTrajectoryLimits
    from jugglebot.motion.skills import schedule as sk_sched
    from jugglebot.motion.skills import sites as sk_sites
    site = sk_sites.columns_sites(125.0)[1]
    rest_mm = np.array([0.0, 0.0, float(site.rest_site_mm()[2])])
    node = _perf_node()
    req = SetTrajectoryLimits.Request()          # the R5 launch default
    req.leg_vel_limit_mmps = 350.0
    req.leg_acc_limit_mmps2 = 5000.0
    req.leg_jerk_limit_mmps3 = 200000.0
    node._svc_set_limits(req, SetTrajectoryLimits.Response())
    home = _robot_state(hand_rev=_hand_rev_for_cup_z(rest_mm[2]))
    node._on_robot_state(home)

    real_perf, real_install = time.perf_counter, tn.sk_exec.install_segment
    # The clock starts 0.1 ms before a knot-grid instant, so the handler's
    # grid snap of the CATCH's origin is the same on every run: without it the
    # catch record's end moves by up to a knot with the wall clock's phase, and
    # with it whether the REST installs fresh or as a splice.
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    grid0 = (int(real_perf() / dt) + 2) * dt
    clock = {'t': grid0 - 1e-4, 'solve_s': 0.0}

    def slow_install(*args, **kwargs):
        # the seed is already sampled (`_cycle_start_state`); the guard reads
        # the clock after this returns -- exactly the solve's place in time
        clock['t'] += clock['solve_s']
        return real_install(*args, **kwargs)

    log = SimpleNamespace(tries=[], lines=[])

    def installer(kind, terminal, t_now_s, ball_id=None):
        node._on_robot_state(home)               # 100 Hz on the real graph
        _rev, hand_vel = node._commanded_hand_state()
        resp = node._svc_install_segment(
            _rest_req(terminal.t_rest_s, rest_z=float(terminal.rest_site_mm[2])),
            InstallSegment.Response())
        log.tries.append((clock['t'], float(hand_vel), bool(resp.accepted),
                          str(resp.code), str(resp.message), node._cycle))
        return sk_exec.InstallResult(bool(resp.accepted), str(resp.code),
                                     str(resp.message), 0.0, splice_k=0)

    time.perf_counter = lambda: clock['t']
    tn.sk_exec.install_segment = slow_install
    try:
        t_land = grid0 + sk_sched.LEAD_S + 0.6
        # Frozen at the historical 930 mm sitting-6 catch plane and its
        # landing speed -- see the "PINNED at 930 mm" note above the
        # function docstring for why this does not follow the live
        # ``sites.CATCH_CUP_Z_MM`` (830 since 2026-10-06).
        resp = node._svc_install_segment(
            _catch_req(t_land, site_z=930.0,
                       vel_z=-5520.0, rest_z=float(rest_mm[2])),
            InstallSegment.Response())
        assert resp.accepted is True, resp.message
        catch_cycle = node._cycle
        # `compile_reload`'s own placement of the DECAY REST
        start = t_land + sk_seg.REST_TAIL_S + sk_sched.REST_FRESH_MARGIN_S
        rest = sk_sched.Skill(kind=sk_seg.REST, ball_id=0, site=site,
                              t_abs_s=start + sk_sched.DECAY_S,
                              window_s=sk_sched.DECAY_S, rest_site_mm=rest_mm,
                              lead_s=sk_sched.LEAD_S)
        sched = sk_sched.Schedule(
            pattern='self_toss', skills=(rest,), flight_s=0.0,
            beat_s=sk_sched.DECAY_S, transit_s=0.0, dwell_s=0.0,
            t0_abs_s=start)
        x = sk_exec.SkillExecutor(sched, installer,
                                  dispatch_lookahead_s=sn._DISPATCH_LOOKAHEAD_S)
        clock['solve_s'] = float(solve_s)
        # The first tick lands 10 ms into the executor's dispatch tick (on the
        # robot, anywhere in its 25 ms): `t_now + lead` is then 10 ms past the
        # catch record's end, so the install is FRESH -- the branch the robot
        # took -- rather than sitting on the fresh/splice boundary.
        t = rest.dispatch_s() - sn._DISPATCH_LOOKAHEAD_S + 0.010
        while t < start + 0.2 and not x.attempt_ended and 0 not in x.dispatched:
            clock['t'] = max(clock['t'], t)
            log.lines.extend(x.tick(clock['t']))
            t = max(t + 1.0 / sn._TICK_HZ, clock['t'])
    finally:
        time.perf_counter = real_perf
        tn.sk_exec.install_segment = real_install
    return x, log, catch_cycle, start


@pytest.mark.parametrize('solve_s', [0.036, 0.073])
def test_the_rest_after_a_high_catch_is_refused_then_installs_on_a_retry(solve_s):
    """36 ms is the solve the robot measured on the refused attempt; 73 ms is
    the slowest REST solve in the sitting-4/5 logs. Probe (2026-10-06, this
    harness): at 36 ms the first try refuses at 1.2002 rev with the hand at
    -34.1 rev/s (the robot: 1.2600 rev at -34.69), the second at 1.1035, the
    third installs at -28.7 rev/s, 0.157 s before the scheduled start; at
    73 ms 2.3324 and 1.6503 are refused and the third installs at -15.4
    rev/s, 0.046 s before it; at 21 ms the first try installs."""
    x, log, catch_cycle, start = _sitting6_rest_after_a_high_catch(solve_s)
    assert not x.attempt_ended, (x.end_code, x.end_message)
    assert 0 in x.dispatched
    refused, accepted = log.tries[:-1], log.tries[-1]
    assert refused, 'the fixture must reproduce the refusal to mean anything'
    for _t, hand_vel, ok, code, message, cycle in refused:
        assert not ok and code == feas.STALE_STATE
        assert 'hand position drift' in message
        assert abs(hand_vel) > 25.0             # past anything sitting 5 flew
        assert cycle is catch_cycle             # a refusal leaves the CATCH streaming
    assert accepted[2] is True, accepted[4]
    assert accepted[5] is not catch_cycle       # the REST now owns the wire
    # every try, the accepted one included, is inside the REST's lead window
    assert accepted[0] < start
    assert sum('REST-RETRY skill 0' in ln for ln in log.lines) == len(refused)


def test_the_rest_after_a_high_catch_installs_at_once_when_the_solve_is_quick():
    """The control: a 21 ms solve (the fastest in the logs) keeps the hand's
    travel under the bound even at 930 mm, so nothing is retried and the REST
    starts when it always did."""
    x, log, _catch_cycle, _start = _sitting6_rest_after_a_high_catch(0.021)
    assert not x.attempt_ended
    assert [tr[2] for tr in log.tries] == [True]
    assert not any('REST-RETRY' in ln for ln in log.lines)


# ═════════════════════════════════════════════════════════════════════════════
# Splice
# ═════════════════════════════════════════════════════════════════════════════

def test_a_catch_requested_while_a_throw_streams_splices_head_bit_identical():
    node = _perf_node()
    with _frozen_perf() as at:
        resp1 = node._svc_install_segment(_throw_req(at + 0.6),
                                          InstallSegment.Response())
        assert resp1.accepted is True, resp1.message
        plan1, meta1, t0_1 = node._cycle

        # Re-anchor the origin 0.2 s into the past, as `_running_carry` does for
        # PlanCycle — the plan is a pure function of tau, so there is nothing
        # to be gained by sleeping for it.
        origin = at - 0.2
        node._cycle = (plan1, meta1, origin)
        node._plan_t0 = origin
        _refresh(node)

        # `at + 0.7` (not `+0.5`) — the head's release (0.6 s, relative) now
        # PINS the splice for any raw candidate at or before it
        # (`executor._snap_to_release`: "the rule is stated on the RELEASE,
        # not on the dispatch"), so the window this test measures is from the
        # RELEASE knot to the catch event, not from the raw splice candidate.
        resp2 = node._svc_install_segment(_catch_req(at + 0.7),
                                          InstallSegment.Response())
    assert resp2.accepted is True, resp2.message
    assert resp2.splice_k > 0
    k_s = resp2.splice_k
    plan2 = node._active_plan
    assert plan2 is not plan1
    np.testing.assert_array_equal(plan2.pose[:k_s + 1], plan1.pose[:k_s + 1])
    np.testing.assert_array_equal(plan2.hand_rev[:k_s + 1],
                                  plan1.hand_rev[:k_s + 1])


def test_a_catch_with_throw_requested_at_the_handoff_lead_snaps_to_the_release():
    """A CATCH-with-throw dispatched so its splice base lands ON the previous
    release's own knot snaps to it and seeds POST-RELEASE
    (``executor._snap_to_release``) — the ring's own handoff, per
    ``schedule.compile_columns``'s docstring on why a catch-with-throw's
    dispatch instant lands on the previous release.

    The head THROW releases at knot ``k_rel = round(1.0 / dt)``; the origin is
    re-anchored (same test-fiction as the plain-splice test above) so that
    ``tau + lead_s`` lands EXACTLY on ``k_rel * dt`` — the earliest legal
    splice knot is then the release knot itself, snapped rather than refused.

    ``k_rel`` is read off the head plan's own release, not assumed to be
    ``round(1.0 / dt)``: since C2FF (2026-09-14) a fresh install snaps its t0
    UP onto the emitter's knot grid, so the release (an absolute instant) sits
    up to one knot earlier on the plan's clock.
    """
    node = _perf_node()
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    release_rel = 1.0
    lead_s = node._segment_lead_s
    with _frozen_perf() as at:
        resp1 = node._svc_install_segment(_throw_req(at + release_rel),
                                          InstallSegment.Response())
        assert resp1.accepted is True, resp1.message
        plan1, meta1, t0_1 = node._cycle

        k_rel = int(round(float(meta1.releases[0].t_s) / dt))
        tau = k_rel * dt - lead_s
        origin = at - tau
        node._cycle = (plan1, meta1, origin)
        node._plan_t0 = origin
        _refresh(node)

        t_origin = origin + k_rel * dt  # == at, by construction
        t_land = t_origin + 0.4
        t_release = t_origin + 0.9
        resp2 = node._svc_install_segment(
            _catch_throw_req(t_land, t_release), InstallSegment.Response())
    assert resp2.accepted is True, resp2.message
    assert resp2.code == feas.OK
    assert resp2.splice_k == k_rel
    assert resp2.seeded_post_release is True
    assert resp2.t_event_mono == pytest.approx(t_land)
    assert resp2.t_release_mono == pytest.approx(t_release)


def test_a_plain_catch_still_splices_without_carrying_a_release():
    """``t_release_s`` left at the wire's 0.0 sentinel builds a standalone
    CATCH — ``seeded_post_release`` may be true or false depending on where
    the splice lands, but ``t_release_mono`` stays 0.0 (no release to carry)."""
    node = _perf_node()
    with _frozen_perf() as at:
        resp1 = node._svc_install_segment(_throw_req(at + 0.6),
                                          InstallSegment.Response())
        assert resp1.accepted is True, resp1.message
        plan1, meta1, t0_1 = node._cycle
        origin = at - 0.2
        node._cycle = (plan1, meta1, origin)
        node._plan_t0 = origin
        _refresh(node)
        # `at + 0.7`, not `+0.5` — see the head-pinning comment on the splice
        # test above.
        resp2 = node._svc_install_segment(_catch_req(at + 0.7),
                                          InstallSegment.Response())
    assert resp2.accepted is True, resp2.message
    assert resp2.t_release_mono == 0.0


def test_splice_too_late_when_the_solve_runs_past_the_wire():
    """A slow solve must be caught AGAINST A FRESH CLOCK READ, not against
    the instant the handler was entered — see ``executor.install_segment``'s
    ``t_install_s`` docstring."""
    node = _perf_node()
    with _frozen_perf() as at:
        resp1 = node._svc_install_segment(_throw_req(at + 0.6),
                                          InstallSegment.Response())
        assert resp1.accepted is True, resp1.message
        plan1, meta1, t0_1 = node._cycle
        origin = at - 0.2
        node._cycle = (plan1, meta1, origin)
        node._plan_t0 = origin
        _refresh(node)

        # The FIRST two `perf_counter()` reads inside the handler (t_wall/t_now_s,
        # then the one `_robot_state_fresh` takes) see the frozen instant, so the
        # guard ladder passes exactly as it does above; every read after that
        # (the `t_install_s` read, and any wall-time bookkeeping past it) sees a
        # clock 10 s further on — simulating a solve that ran far longer than it
        # actually did, without needing the QP itself to be slow.
        calls = {'n': 0}
        real_at = at

        def _fake_perf_counter():
            calls['n'] += 1
            return real_at if calls['n'] <= 2 else real_at + 10.0

        # Set the attribute DIRECTLY, never via `monkeypatch` inside
        # `_frozen_perf`: monkeypatch snapshots the CURRENT value (the frozen
        # lambda) as the original and restores it at teardown — AFTER the
        # context manager has already put the real clock back — so the frozen
        # clock leaked for the rest of the xdist worker's life and every later
        # wall-time read in that process returned a constant
        # (`test_unified_cycle_splice.py::test_splice_at_the_terminal_knot…`
        # failed `0.0 < 0.0` on the 2026-09-14 full gate). `_frozen_perf`'s own
        # exit restores the real clock.
        time.perf_counter = _fake_perf_counter
        # `at + 0.7`, not `+0.5` — see the head-pinning comment on the splice
        # test above; a too-tight window would refuse WINDOW_TOO_SHORT before
        # the late clock read is ever taken, which is not what this test means
        # to exercise.
        resp2 = node._svc_install_segment(_catch_req(at + 0.7),
                                          InstallSegment.Response())
    assert resp2.accepted is False
    assert resp2.code == sk_exec.SPLICE_TOO_LATE, resp2.message
    assert node._active_plan is plan1
    assert node._cycle[0] is plan1
    assert node._cycle[1] is meta1
    assert node._cycle[2] == origin


# ═════════════════════════════════════════════════════════════════════════════
# Guard ladder
# ═════════════════════════════════════════════════════════════════════════════

def test_guard_latched_refuses_before_planning():
    node = _perf_node()
    node._guard_frozen = True
    active_before = node._active_plan
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is False
    assert resp.code == tn._GUARD_LATCHED
    assert node._active_plan is active_before


def test_wrong_mode_refuses_before_planning():
    from std_msgs.msg import String
    node = _perf_node()
    node._on_control_mode(String(data='STANDBY'))
    active_before = node._active_plan
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is False
    assert resp.code == feas.WRONG_MODE
    assert node._active_plan is active_before


def test_not_seeded_refuses_before_planning():
    node = _perf_node()
    node._seeded = False
    active_before = node._active_plan
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is False
    assert resp.code == feas.STALE_STATE
    assert node._active_plan is active_before


def test_stale_telemetry_refuses_before_planning():
    node = _perf_node()
    node._robot_state_mono = time.perf_counter() - 10.0
    active_before = node._active_plan
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is False
    assert resp.code == feas.STALE_STATE
    assert node._active_plan is active_before


# ═════════════════════════════════════════════════════════════════════════════
# Hold interaction
# ═════════════════════════════════════════════════════════════════════════════

def test_a_hold_landed_during_the_solve_supersedes_the_install(monkeypatch):
    """Mirrors ``test_trajectory_hold_preempts_solve.py``'s pattern for
    ``plan_cycle``: a hold landing WHILE the solve runs (here, between the
    epoch capture and ``sk_exec.install_segment`` returning) must refuse the
    install even though it produced an otherwise-accepted plan."""
    node = _perf_node()
    real_install_segment = tn.sk_exec.install_segment

    def _hold_then_install(*a, **kw):
        assert node._svc_hold(Trigger.Request(), Trigger.Response()).success
        return real_install_segment(*a, **kw)

    monkeypatch.setattr(tn.sk_exec, 'install_segment', _hold_then_install)
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is False
    assert resp.code == tn._SUPERSEDED_BY_HOLD, resp.message
    assert not isinstance(node._active_plan, CyclePlan)


# ═════════════════════════════════════════════════════════════════════════════
# Status
# ═════════════════════════════════════════════════════════════════════════════

def test_status_publishes_cycle_plan_wall_ms_from_the_install():
    node = _perf_node()
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is True, resp.message
    assert node._cycle_plan_wall_ms == pytest.approx(resp.plan_wall_ms)
    node._publish_status()
    msg = node.status_pub.published[-1]
    assert msg.cycle_plan_wall_ms == pytest.approx(resp.plan_wall_ms)


# ═════════════════════════════════════════════════════════════════════════════
# A5 — an accept on a disarmed wire is loud
# ═════════════════════════════════════════════════════════════════════════════

def test_an_accept_on_a_disarmed_wire_carries_the_disarmed_marker():
    """ARMING_CONTRACT A5: every accepted motion command on this node appends the
    wire state when ``mpc_active=0`` (``_wire_state_suffix``), so a harness prints
    that the setpoints are not reaching the legs. ``install_segment`` did not until
    2026-09-13 — found writing the R2 hardware-gate runsheet
    (``tests/hardware/session_skills_r2_plan_gate.md``), which runs the whole gate
    on a DISARMED wire and reads this marker as its per-install proof that
    nothing moved."""
    node = _perf_node()
    node._on_link_status(_link_status(mpc_active='0'))
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is True, resp.message
    assert 'wire DISARMED' in resp.message


def test_an_accept_on_an_armed_wire_carries_no_disarmed_marker():
    node = _perf_node()
    node._on_link_status(_link_status(mpc_active='1'))
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is True, resp.message
    assert 'DISARMED' not in resp.message


# ═════════════════════════════════════════════════════════════════════════════
# The fresh-origin HAND SEED is reconciled against the encoder (2026-09-16)
#
# This is where `REJECTED_HAND_NOT_PARKED` went. The refusal used to sit in
# `motion/skills/executor.py`'s ladder and asked "is the hand at park?"; it is
# retired (owner decision, 2026-09-16) and the invariant it stood proxy for
# — knot 0 of a fresh-origin window IS the hand — is enforced here, by
# CORRECTING the seed rather than by refusing the skill.
# ═════════════════════════════════════════════════════════════════════════════

def _reconcile_node(*, commanded_hand_rev, measured_hand_rev):
    """A rest-terminal node whose COMMANDED hand seed and MEASURED hand
    DISAGREE — the 2026-09-16 shape.

    The commanded side is set through `_last_hand_rev` (source 2 of
    `_commanded_hand_state`: "the last rev the emitter actually put on the
    wire"), which is exactly what a finished plan's terminal hold leaves
    behind. The measured side is a real `robot_state` message, so the node's
    own freshness gate is satisfied the way the live graph satisfies it.
    """
    node = _perf_node()
    node._on_robot_state(_robot_state(hand_rev=float(measured_hand_rev)))
    node._last_hand_rev = float(commanded_hand_rev)
    return node


def test_a_fresh_rest_seed_is_reconciled_to_the_measured_hand():
    """THE 2026-09-16 NUMBERS (bag `2026-09-16_14-16-38`): an attempt ended
    `SPLICE_TOO_LATE` and installed a hold at +0.5639 rev; the hand was then
    re-parked and read +0.0001 rev. Nine later schedules were refused at
    skill 0.

    With the refusal retired, the next fresh-origin REST must PLAN — and it
    must plan from the ENCODER, not from the stale hold, because a window
    seeded 0.56 rev above the hand is the L2 failure class in miniature.
    """
    node = _reconcile_node(commanded_hand_rev=0.5639, measured_hand_rev=0.0001)
    state, code, err = node._cycle_start_state(uc.SETTLE)
    assert state is not None, (code, err)
    assert state.hand_rev == pytest.approx(0.0001, abs=1e-9)
    assert state.hand_rev != pytest.approx(0.5639)


def test_a_disagreement_inside_the_firmware_tolerance_keeps_the_commanded_seed():
    """Below `_SEED_HAND_RECONCILE_TOL_REV` the two readings describe the same
    machine by the FIRMWARE's own definition (`SCHED_RESUME_TOL_POS_HAND_REV`
    = 0.05 rev is what `sched_apply` allows between a promoted frame's u0[6]
    and the hand it already holds), so the COMMANDED value is kept — folding
    ordinary tracking error into knot 0 is the defect `_commanded_hand_state`
    exists to avoid."""
    node = _reconcile_node(commanded_hand_rev=0.3400, measured_hand_rev=0.3071)
    state, code, err = node._cycle_start_state(uc.SETTLE)
    assert state is not None, (code, err)
    assert state.hand_rev == pytest.approx(0.3400, abs=1e-9)


def test_the_reconciliation_is_reported_on_the_console():
    """A silent seed correction is a seed correction nobody can audit against
    a bag. ONE warning, naming BOTH values (Workflow Rules: a bench reading
    needs its preconditions recorded next to it)."""
    node = _reconcile_node(commanded_hand_rev=0.5639, measured_hand_rev=0.0001)
    lines = []
    node.get_logger().warning = lambda m: lines.append(str(m))
    node._cycle_start_state(uc.SETTLE)
    assert len(lines) == 1, lines
    assert 'HAND SEED RECONCILED' in lines[0]
    assert '0.5639' in lines[0] and '0.0001' in lines[0]


def test_a_moving_machine_is_never_reconciled():
    """Reconciliation is gated on `at_rest`. With the hand MOVING, source (1)
    of `_commanded_hand_state` is exact at every instant and the measurement
    is the channel that LAGS (by the whole launch), so reconciling there would
    fold the tracking error into knot 0 — the defect that method exists to
    avoid.

    With a moving hand and no live cup track a SETTLE is refused `_IN_MOTION`
    before any seed is built, and the refusal names the hand rate: proof the
    reconciliation never got the chance to quietly rewrite the seed. The WARN
    is asserted absent for the same reason.

    The motion is seeded through an ACTIVE PLAN (source 1), not
    ``_latest_hand_vel_rps`` (source 2/3's telemetry rate) — 2026-09-28 (unit
    C-exec, R4 sitting-1 fact 6): sources (2)/(3) now report velocity 0.0
    UNCONDITIONALLY (``_commanded_hand_state``'s docstring), because with no
    hand-bearing plan installed the streamed lane — the one hand master — is
    not moving the hand, so a nonzero ``vel_estimate`` there is encoder noise
    on a stationary axis, not evidence of motion (a noisy -0.0435 rev/s seeded
    a fresh REST that dove below the parked start and refused
    ``HAND_STROKE``). Source (1) is unaffected by that fix and is this test's
    OWN stated reasoning ("source (1) is exact at every instant"), so this is
    the mechanism that actually needs pinning here.
    """
    node = _reconcile_node(commanded_hand_rev=0.5639, measured_hand_rev=0.0001)
    with node._plan_lock:
        # source (1): really moving — legs held still, hand rate 5.0 rev/s.
        node._active_plan = _HandHoldPlan(node._last_pose.copy(),
                                          lambda tau: (0.5639, 5.0))
        node._plan_t0 = time.perf_counter()
    lines = []
    node.get_logger().warning = lambda m: lines.append(str(m))
    state, code, err = node._cycle_start_state(uc.SETTLE)
    assert state is None
    assert code == tn._IN_MOTION
    assert 'hand 5.0000 rev/s' in err
    assert not [m for m in lines if 'HAND SEED RECONCILED' in m]


def test_the_opening_rest_carries_a_far_hand_home_inside_the_floor_lift():
    """THE REPLACEMENT FOR THE REFUSAL, end to end: the schedule's opening
    REST (`schedule.FLOOR_LIFT_S`, 1.5 s) installs from a hand 8 rev off the
    park — the 2026-09-13 L2 magnitude — and settles it at the SETTLE clamp.

    8.0 rev used to be the clearest `REJECTED_HAND_NOT_PARKED`; it is now a
    1.5 s carry. The terminal is `SETTLE_CUP_Z_MM`'s rev (0.3071), NOT 0.0:
    the QP's cup box is inset 10 mm above the homed zero, so a window asked
    to settle at the literal park is refused `SETTLE_SITE` — see
    `unified_cycle.SETTLE_CUP_Z_MM`'s own note. 0.3071 rev is inside the
    0.5 rev park band with 39 % to spare.
    """
    from jugglebot.motion.skills import schedule as sch
    node = _reconcile_node(commanded_hand_rev=8.7610, measured_hand_rev=8.0)
    with _frozen_perf() as at:
        resp = node._svc_install_segment(
            _rest_req(at + sch.FLOOR_LIFT_S, rest_z=uc.SETTLE_CUP_Z_MM),
            InstallSegment.Response())
    assert resp.accepted is True, resp.message
    plan = node._active_plan
    assert float(plan.hand_rev[0]) == pytest.approx(8.0, abs=1e-6)
    settle_rev = float(plan.hand_rev[-1])
    assert settle_rev == pytest.approx(0.3071, abs=2e-3)
    assert abs(settle_rev) <= float(hw.HOMING_HAND_PARK_BAND_REV)
    # And it is a GENTLE carry, not a lunge: the whole 7.7 rev inside 1.5 s
    # peaks three orders under the 200 rev/s session ceiling.
    assert float(np.max(np.abs(plan.hand_vel_rps))) < 12.0


# ═════════════════════════════════════════════════════════════════════════════
# hold_tilt_rad / rest_tilt_rad (R4 reload, Unit U3) — the wire sentinel
# ═════════════════════════════════════════════════════════════════════════════

def test_the_nan_sentinel_decodes_to_no_tilt():
    """A request that never touches the new fields (every pre-R4 caller, and
    the mock's own default — see conftest.InstallSegment.Request) must decode
    to `hold_tilt=None` / `tilt=None`, exactly the pre-R4 behaviour."""
    node = _perf_node()
    catch_terminal = node._segment_terminal_from_request(
        'CATCH', _catch_req(10.0), 10.0)
    rest_terminal = node._segment_terminal_from_request(
        'REST', _rest_req(10.0), 10.0)
    assert catch_terminal.hold_tilt is None
    assert rest_terminal.tilt is None


def test_an_explicit_zero_tilt_round_trips_as_itself_not_as_none():
    """THE reason the sentinel is NaN and not zero (`trajectory_node.
    _wire_tilt`'s docstring): the DECAY REST's own target is the REAL value
    (0.0, 0.0) rad, level, and must decode as that value — not collapse onto
    `tilt=None`, which routes `unified_cycle._realize_tilted` to the
    zero-banking branch U2 measured refusing `LIMIT_VEL` for this exact
    transition (handoff_U2.md)."""
    node = _perf_node()
    req = _rest_req(10.0)
    req.rest_tilt_set = True
    req.rest_tilt_rad = [0.0, 0.0]
    terminal = node._segment_terminal_from_request('REST', req, 10.0)
    assert terminal.tilt == (0.0, 0.0)
    assert terminal.tilt is not None


def test_a_nonzero_hold_tilt_rad_round_trips_onto_the_catch_terminal():
    node = _perf_node()
    req = _catch_req(10.0)
    req.hold_tilt_set = True
    req.hold_tilt_rad = [0.01, -0.20943951023931956]
    terminal = node._segment_terminal_from_request('CATCH', req, 10.0)
    assert terminal.hold_tilt == pytest.approx((0.01, -0.20943951023931956))


# ═════════════════════════════════════════════════════════════════════════════
# receive_tilt_rad (R5, 2026-10-02) -- the level touch-down pin the BB-fed
# columns feed catch needs was NOT carried over InstallSegment: skill_node's
# `_installer` encoded only `terminal.hold_tilt`, and trajectory_node's
# `_segment_terminal_from_request` decoded only `request.hold_tilt_*`, so
# every live feed catch planned with `receive_tilt=None` (the banked
# `tilt_to_receive` pin) and refused LIMIT_VEL on the robot -- while the
# offline pre-throw check (`executor.plan_columns_first_cycle`) and the sim
# gate both plan IN-PROCESS with the executor's own terminal (pin present)
# and so both passed. Four of seven fed columns attempts refused the feed
# catch at install on the 2026-10-02 sitting.
# ═════════════════════════════════════════════════════════════════════════════

def test_receive_tilt_round_trips_through_both_wire_halves():
    """Drives BOTH wire halves with the real functions: `skill_node.
    _wire_tilt_out` (what `_installer`'s CATCH branch calls to encode
    `terminal.receive_tilt`) builds the request fields, and trajectory_node's
    `_segment_terminal_from_request` decodes them back. `(0.0, 0.0)` is a
    REAL value (the set flag carries it), not "unset" -- same discipline as
    `hold_tilt_rad` / `rest_tilt_rad` above."""
    node = _perf_node()
    req = _catch_req(10.0)
    req.receive_tilt_set, req.receive_tilt_rad = sn._wire_tilt_out((0.0, 0.0))
    terminal = node._segment_terminal_from_request('CATCH', req, 10.0)
    assert terminal.receive_tilt == (0.0, 0.0)
    assert terminal.receive_tilt is not None
    assert terminal.hold_tilt is None          # untouched by the new field


def test_receive_tilt_none_round_trips_to_none():
    node = _perf_node()
    req = _catch_req(10.0)
    req.receive_tilt_set, req.receive_tilt_rad = sn._wire_tilt_out(None)
    terminal = node._segment_terminal_from_request('CATCH', req, 10.0)
    assert terminal.receive_tilt is None


def test_a_nonzero_receive_tilt_rad_round_trips_onto_the_catch_terminal():
    node = _perf_node()
    req = _catch_req(10.0)
    req.receive_tilt_set = True
    req.receive_tilt_rad = [0.01, -0.20943951023931956]
    terminal = node._segment_terminal_from_request('CATCH', req, 10.0)
    assert terminal.receive_tilt == pytest.approx((0.01, -0.20943951023931956))
    assert terminal.hold_tilt is None          # the two are mutually exclusive


# ═════════════════════════════════════════════════════════════════════════════
# Wire-map contract (R5, 2026-10-02) -- THE class fix. Every dataclass field
# of ThrowTerminal / CatchTerminal / RestTerminal must appear below either in
# an explicit WIRE MAP (the InstallSegment.Request field(s) it travels as) or
# in a commented EXCLUSION list (fields that are derived or never cross the
# wire, each with the reason). `receive_tilt` landed on `CatchTerminal`
# 2026-09-30 with no entry in `skill_node._installer` or `trajectory_node.
# _segment_terminal_from_request` -- a gap neither the mocked-ROS test suite
# (which never round-trips a terminal through the service) nor the offline
# pre-throw check / sim gate (which plan in-process with the executor's own
# terminal) could see. A field added to one of these three dataclasses
# without a matching entry here now fails THIS test, by name, instead of
# surfacing only as a refusal on the real robot.
# ═════════════════════════════════════════════════════════════════════════════

_THROW_WIRE_MAP = {
    'site_mm': ('site_mm',),
    'target_mm': ('target_mm',),
    'flight_s': ('flight_s',),
    't_release_s': ('t_event_s',),
}

_CATCH_WIRE_MAP = {
    'landing_mm': ('site_mm',),
    'landing_vel_mm_s': ('landing_vel_mm_s',),
    't_land_s': ('t_event_s',),
    'rest_site_mm': ('rest_site_mm',),
    # The carried release -- ThrowAfterCatch's own fields, reused onto the
    # THROW wire fields (InstallSegment.srv's own comment: "one throw, one
    # pair of fields, whichever kind carries it"). `then_throw is None`
    # leaves `t_release_s` at its wire sentinel (<= 0.0); the other three
    # stay at their rosidl defaults and are not read in that case.
    'then_throw': ('t_release_s', 'release_site_mm', 'target_mm', 'flight_s'),
    'hold_tilt': ('hold_tilt_set', 'hold_tilt_rad'),
    'receive_tilt': ('receive_tilt_set', 'receive_tilt_rad'),   # R5, 2026-10-02
}

_REST_WIRE_MAP = {
    'rest_site_mm': ('rest_site_mm',),
    't_rest_s': ('t_event_s',),
    'tilt': ('rest_tilt_set', 'rest_tilt_rad'),
}

_REST_WIRE_EXCLUDED = {
    'holds_ball': (
        'NOT currently wire-plumbed -- InstallSegment.srv has no holds_ball '
        "field, so trajectory_node's rebuild always defaults RestTerminal."
        'holds_ball to True regardless of what the executor set '
        '(executor._rest_terminal passes skill.holds_ball through on the '
        'live dispatch path). Same CLASS of gap as receive_tilt above; '
        'pre-existing, out of scope for the 2026-10-02 receive_tilt fix, '
        'and flagged here by name rather than silently excused.'
    ),
}


def _assert_wire_map_complete(cls, wire_map, excluded):
    """Every field of ``cls`` is in ``wire_map`` or ``excluded``; every
    ``InstallSegment.Request`` field name ``wire_map`` cites actually
    exists."""
    fields = {f.name for f in dataclasses.fields(cls)}
    accounted = set(wire_map) | set(excluded)
    missing = fields - accounted
    assert not missing, (
        '%s field(s) %r are neither wire-mapped nor excluded -- a new '
        'terminal field needs an entry in this wire-map contract '
        '(tests/ros/test_install_segment.py) or it can silently never '
        'reach the live service' % (cls.__name__, sorted(missing)))
    stale = accounted - fields
    assert not stale, (
        '%s wire map/exclusion names field(s) %r that no longer exist on '
        'the dataclass -- stale entry' % (cls.__name__, sorted(stale)))
    req = InstallSegment.Request()
    for terminal_field, req_fields in wire_map.items():
        for req_field in req_fields:
            assert hasattr(req, req_field), (
                '%s.%s is wire-mapped to InstallSegment.Request.%s, which '
                'does not exist' % (cls.__name__, terminal_field, req_field))


def test_every_throw_terminal_field_is_wire_mapped_or_excluded():
    _assert_wire_map_complete(sk_seg.ThrowTerminal, _THROW_WIRE_MAP, {})


def test_every_catch_terminal_field_is_wire_mapped_or_excluded():
    _assert_wire_map_complete(sk_seg.CatchTerminal, _CATCH_WIRE_MAP, {})


def test_every_rest_terminal_field_is_wire_mapped_or_excluded():
    _assert_wire_map_complete(sk_seg.RestTerminal, _REST_WIRE_MAP,
                              _REST_WIRE_EXCLUDED)


# ═════════════════════════════════════════════════════════════════════════════
# T-I1 — the real :5557 seam, end to end (ported from the deleted
# test_unified_cycle_integration.py at the FSM deletion, R4 unit U6b cluster B,
# 2026-09-24 — re-driven through install_segment instead of the retired
# trajectory/plan_cycle; same fixtures, same asserts)
# ═════════════════════════════════════════════════════════════════════════════

def test_TI1_seven_channel_frames_reach_the_wire_and_the_flags_fall():
    """T-I1: trajectory_node -> real :5557 -> teensy_bridge_node -> loopback UDP.

    Every hop is production code: the real ``SetpointCommandPub`` (ephemeral port),
    the real ``_SetpointCommandSource`` decoder, the real ``SetpointPump``,
    the real v6 ``Setpoint`` encoding, and a real UDP socket. What is pinned:

    * while a ``CyclePlan`` is installed (here, via ``install_segment``), every
      frame carries ``HAS_HAND`` AND ``HAS_V1`` — the seven-channel path is
      genuinely live, not merely encodable;
    * the hand lane (index 6) carries the PLAN's commanded rev, not a zero;
    * and the flags CLEAR on the falling edge when a legacy (non-cycle) plan
      supersedes it via a hold. That edge is what FW 17's hand-lane decay is
      written against, and a path that never produced it would leave the decay
      untested from this side.
    """
    import zmq
    from teensy_link import MsgType
    from teensy_link.protocol import Setpoint
    from jugglebot.motion.ipc import SetpointCommandPub
    from jugglebot.teensy_bridge_node import _SetpointCommandSource
    from teensy_link.setpoint_pump import (
        FLAG_HAS_HAND, FLAG_HAS_SCHED, FLAG_HAS_V1)
    from tests.ros._bridge_harness import _build_paired_node, _teardown

    pub = SetpointCommandPub(addr='tcp://127.0.0.1:0')
    addr = pub._pub.getsockopt_string(zmq.LAST_ENDPOINT)
    traj = _perf_node(command_pub_factory=lambda: pub)
    traj._pub = pub                       # emitter thread not started; wire by hand
    teensy, client, bridge = _build_paired_node()
    src = _SetpointCommandSource(addr=addr)
    try:
        # Frozen for the install call ONLY (restored before the real ZMQ/
        # thread dance below needs a live clock): 2026-09-28 (unit C-exec)
        # gave `install_segment`'s fresh branch a genuine wall-clock lateness
        # check (`ORIGIN_TOO_LATE` / REBASE — see its docstring), so an
        # UNFROZEN `time.perf_counter()` here measures this shared box's
        # actual CPU contention across the solve, not the solve itself —
        # MEASURED flaky (2026-09-28: failed with "the solve took 1.430 s"
        # under a parallel session's load, passed clean moments later at
        # lower load). Every other planning call in this file already
        # freezes the clock for exactly this reason; this one predates the
        # lateness check and never needed to.
        with _frozen_perf() as at:
            # at + 0.7: the 0.375 s window this test was sized on, from the
            # THROW's 13-knot reserved origin (0.6 s on the 9-knot LEAD_S
            # until 2026-10-16).
            resp = traj._svc_install_segment(_throw_req(at + 0.7),
                                             InstallSegment.Response())
        assert resp.accepted is True, resp.message
        plan, _meta, t0 = traj._cycle
        bridge._start_setpoint_output(src)
        time.sleep(0.15)                  # PUB/SUB slow-joiner

        # Drive the emitter across the whole 0.6 s window on the PLAN's clock.
        n_knots = 24
        for k in range(n_knots):
            traj._emit_once(t0 + k * hw.JB_TRAJ_KNOT_DT_S)
            time.sleep(0.01)
        got = teensy.wait_for(int(MsgType.SETPOINT), count=8, timeout=3.0)
        assert got, 'no v6 Setpoint frames reached the loopback sink'
        cycle_frames = [Setpoint.unpack(m.payload) for m in got]
        for sp in cycle_frames:
            assert sp.flags & FLAG_HAS_HAND, 'HAS_HAND clear while a cycle streams'
            assert sp.flags & FLAG_HAS_V1, 'HAS_V1 clear while a cycle streams'
        # The hand lane carries the plan, not a placeholder: it MOVES across the
        # throw, and it moves in the direction the throw strokes.
        hand_lane = [sp.u0[6] for sp in cycle_frames]
        assert max(hand_lane) - min(hand_lane) > 0.5
        assert hand_lane[-1] > hand_lane[0]

        # ── The falling edge ──
        n_before = len(teensy.received(int(MsgType.SETPOINT)))
        traj._svc_hold(Trigger.Request(), Trigger.Response())
        assert not isinstance(traj._active_plan, CyclePlan)
        for k in range(8):
            traj._emit_once(time.perf_counter())
            time.sleep(0.01)
        legacy = teensy.wait_for(int(MsgType.SETPOINT), count=n_before + 3,
                                 timeout=3.0)
        assert legacy and len(legacy) > n_before
        for m in legacy[n_before:]:
            sp = Setpoint.unpack(m.payload)
            assert not (sp.flags & FLAG_HAS_HAND), 'HAS_HAND stuck after the cycle'
            assert sp.u0[6] == pytest.approx(0.0)
            # C2FF decision 2 (2026-09-14): the LEGS stay on the stamped clock
            # across the falling edge — the hold still carries exact v1/v2 and
            # the knot stamp, so a hand-off never flips them to arrival phase.
            assert sp.flags & FLAG_HAS_V1, 'legs lost exact v1 at the hand-off'
            assert sp.flags & FLAG_HAS_SCHED, 'legs fell back to arrival phase'
    finally:
        try:
            src.close()
        except Exception:      # noqa: BLE001 — teardown must not mask a failure
            pass
        _teardown(teensy, client, bridge)
        traj.on_shutdown()
        pub.close()


# ═════════════════════════════════════════════════════════════════════════════
# Hold/executor wiring pins (ported from the deleted
# test_trajectory_hold_preempts_solve.py at the FSM deletion, R4 U6b cluster B,
# 2026-09-24 — the behavioural hold-preempts-solve claim was already covered
# above by test_a_hold_landed_during_the_solve_supersedes_the_install; these two
# pin the WIRING that claim depends on (only the hold service is reentrant, and
# trajectory_node.main() actually runs a MultiThreadedExecutor) — general node
# facts, not specific to the retired trajectory/plan_cycle service.
# ═════════════════════════════════════════════════════════════════════════════

def test_only_the_hold_service_is_reentrant():
    """The hold is on its OWN reentrant group; everything else keeps its group.

    The narrowness is the safety argument. Moving every service to a reentrant
    group would let ``install_segment`` interleave with the timers and
    subscriptions it shares state with, and nothing here is written for that.
    Only the hold needs to run during a solve, so only the hold moves — and
    this test fails if a later refactor widens it.
    """
    from rclpy.callback_groups import ReentrantCallbackGroup
    node = _cycle_node()
    services = node._services
    hold = services['trajectory/hold']
    assert isinstance(hold.callback_group, ReentrantCallbackGroup)
    # Everything else stays on the node default (recorded as None by the mock),
    # i.e. mutually exclusive with the timers and subscriptions.
    for name, svc in services.items():
        if name == 'trajectory/hold':
            continue
        assert svc.callback_group is None, name
    # Named explicitly, because this is the pair whose serialisation the shared
    # state depends on: install_segment must NOT be reentrant.
    assert services['trajectory/install_segment'].callback_group is None


def test_main_runs_a_multi_threaded_executor_not_plain_spin(monkeypatch):
    """A reentrant group is inert under ``rclpy.spin`` — the executor must match.

    This is the half a refactor is most likely to undo, because ``spin`` looks
    like the simpler call and the callback group keeps compiling. Under plain
    ``spin`` the hold queues behind the solve exactly as before and the
    supersede cliff is back, silently.
    """
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    built = {}

    class _Spy(MultiThreadedExecutor):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            built['threads'] = kw.get('num_threads')

        def spin(self):
            built['spun'] = True

    monkeypatch.setattr('jugglebot.trajectory_node.MultiThreadedExecutor', _Spy)
    monkeypatch.setattr(rclpy, 'init', lambda *a, **k: None)
    monkeypatch.setattr(rclpy, 'shutdown', lambda *a, **k: None)
    monkeypatch.setattr(rclpy, 'spin',
                        lambda *a, **k: pytest.fail('plain spin is back'),
                        raising=False)

    tn.main()
    assert built.get('spun') is True
    # At least two: one thread to run the serialized group, one free for the hold.
    assert built.get('threads', 0) >= 2


def test_an_untouched_request_decodes_no_tilt_even_though_rosidl_zero_fills_the_array():
    """R4 phase-end audit (2026-09-24): rosidl initialises an unset
    ``float64[2]`` to ZEROS, not NaN, so a request built by anything other
    than ``skill_node._installer`` (a hand-issued ``ros2 service call``, a
    pre-R4 caller) carries ``[0.0, 0.0]`` — which is ALSO the DECAY REST's
    real level target. The ``*_tilt_set`` flags (rosidl default false) are
    what say "no tilt given"; the mock mirrors the generated type's defaults,
    so this test drives the exact shape a hand-built request has."""
    node = _perf_node()
    req = _rest_req(10.0)
    assert req.rest_tilt_set is False and list(req.rest_tilt_rad) == [0.0, 0.0]
    assert node._segment_terminal_from_request('REST', req, 10.0).tilt is None
    req2 = _catch_req(10.0)
    assert req2.hold_tilt_set is False
    assert node._segment_terminal_from_request('CATCH', req2, 10.0).hold_tilt is None
    # and a set flag with a NaN payload is malformed -> also no tilt
    req3 = _rest_req(10.0)
    req3.rest_tilt_set = True
    req3.rest_tilt_rad = [float('nan'), 0.0]
    assert node._segment_terminal_from_request('REST', req3, 10.0).tilt is None


# ═════════════════════════════════════════════════════════════════════════════
# MAGAZINE (KIND_MAGAZINE = 3): the wire decodes to a MagazineTerminal
# ═════════════════════════════════════════════════════════════════════════════

def _magazine_req(t_event_s, site=(10.0, 20.0, 780.0), target=(900.0, -300.0, 750.0),
                  flight_s=0.9, rest=(0.0, 0.0, 750.0)):
    req = InstallSegment.Request()
    req.kind = req.KIND_MAGAZINE
    req.t_event_s = float(t_event_s)
    req.site_mm = list(site)
    req.target_mm = list(target)
    req.flight_s = float(flight_s)
    req.rest_site_mm = list(rest)
    return req


def test_magazine_request_decodes_to_a_magazine_terminal():
    from jugglebot.motion.skills import magazine_throw as mt
    assert InstallSegment.Request.KIND_MAGAZINE == 3
    assert tn.TrajectoryNode._SEGMENT_KINDS[3] == mt.MAGAZINE
    node = _perf_node()
    terminal = node._segment_terminal_from_request(
        mt.MAGAZINE, _magazine_req(12.0), 12.0)
    assert isinstance(terminal, mt.MagazineTerminal)
    assert terminal.t_release_s == 12.0
    assert terminal.flight_s == pytest.approx(0.9)
    assert list(terminal.release_site_mm) == [10.0, 20.0, 780.0]
    assert list(terminal.target_mm) == [900.0, -300.0, 750.0]
    assert list(terminal.rest_site_mm) == [0.0, 0.0, 750.0]


def test_magazine_request_reaches_executor_with_kind_magazine(monkeypatch):
    from jugglebot.motion.skills import magazine_throw as mt
    seen = {}

    def _fake_install(record, seed, kind, terminal, t_now, **kw):
        seen['kind'] = kind
        seen['terminal'] = terminal
        return None, SimpleNamespace(accepted=False, code='MAGAZINE_UNREACHABLE',
                                     message='stub'), None

    monkeypatch.setattr(tn.sk_exec, 'install_segment', _fake_install)
    node = _perf_node()
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_magazine_req(at + 1.5),
                                         InstallSegment.Response())
    assert resp.accepted is False
    assert resp.code == 'MAGAZINE_UNREACHABLE'
    assert seen['kind'] == mt.MAGAZINE
    assert isinstance(seen['terminal'], mt.MagazineTerminal)
    # the ROS->perf crossing is the same one THROW gets (offset is zero here)
    assert seen['terminal'].t_release_s == pytest.approx(at + 1.5)


# ═════════════════════════════════════════════════════════════════════════════
# The segment SOLVER seam (wave 3, unit I2: the solve runs in a worker process)
#
# `TrajectoryNode(segment_solver=...)` injects a FAKE here -- no process is ever
# spawned by these tests.  What they pin is the node's side of the contract:
# refusal mapping, the deadline it ships, the reply-transit guard, lifecycle
# hooks, and that the shipped default is the worker (never the test double).
# ═════════════════════════════════════════════════════════════════════════════
from jugglebot.motion.skills import solver_worker as sk_solver  # noqa: E402


class _FakeSolver:
    """Records every request; answers via ``reply_fn(req) -> SegmentSolveReply``
    (default: the real in-process solve, so a chain can be built)."""

    pid = None

    def __init__(self, reply_fn=None):
        self.requests = []
        self.health_polls = 0
        self.closed = 0
        self._inner = sk_solver.InProcessSegmentSolver()
        self._reply_fn = reply_fn

    def solve(self, req):
        self.requests.append(req)
        if self._reply_fn is not None:
            return self._reply_fn(req, self._inner)
        return self._inner.solve(req)

    def poll_health(self):
        self.health_polls += 1

    def close(self, timeout_s=1.0):
        self.closed += 1


def _solver_node(solver):
    return _perf_node(segment_solver=solver)


def test_the_shipped_default_segment_solver_is_the_worker_not_the_double(
        monkeypatch):
    """The autouse fixture swaps the factory for the in-process double in this
    directory; production must still build the WORKER, or the emitter shares an
    interpreter with the 60-206 ms solve again (sitting 7: 45-57 ms stalls)."""
    monkeypatch.undo()          # drop `in_process_segment_solver`
    assert tn._default_segment_solver_factory is sk_solver.WorkerSegmentSolver
    assert tn.SOLVER_UNAVAILABLE == sk_solver.SOLVER_UNAVAILABLE


def test_the_test_fixture_really_replaces_the_factory_in_this_directory():
    node = _cycle_node()
    assert isinstance(node._segment_solver, sk_solver.InProcessSegmentSolver)


def test_solver_unavailable_surfaces_by_name_and_is_not_aim_dependent():
    solver = _FakeSolver(lambda req, _inner: sk_solver._refusal(
        req.seq, sk_solver.SOLVER_UNAVAILABLE, 'worker starting'))
    node = _solver_node(solver)
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is False
    assert resp.code == 'SOLVER_UNAVAILABLE'
    assert 'worker starting' in resp.message
    # The client executor must NOT re-dispatch a learned-prior fallback on it.
    assert sk_exec._aim_dependent_refusal(resp.code) is False
    assert node._cycle is None


def _slow_return(clock_jump_s):
    """A solver whose reply 'arrives' ``clock_jump_s`` after the executor's own
    lateness check passed: the real solve runs at the frozen clock, then the
    clock the node reads jumps (``_frozen_perf``'s exit restores the real one)."""
    def _fn(req, inner):
        rep = inner.solve(req)
        now = time.perf_counter()
        time.perf_counter = lambda: now + clock_jump_s
        return rep
    return _fn


def test_a_reply_that_outran_the_wire_in_transit_is_refused_not_installed():
    """A fresh origin: the executor accepted at the frozen clock, then the
    reply took 10 s to arrive (SIGSTOP-class).  The node re-checks the same
    arithmetic at its own clock: ORIGIN_TOO_LATE, nothing installed."""
    node = _solver_node(_FakeSolver(_slow_return(10.0)))
    before = node._active_plan
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is False
    assert resp.code == sk_exec.ORIGIN_TOO_LATE
    assert 'reply transit' in resp.message
    assert node._active_plan is before and node._cycle is None


def test_a_stalled_node_refuses_a_fresh_rest_whose_origin_was_rebased_upstream():
    """A fresh REST's origin is set at the WORKER's clock; a node stalled after
    the reply (SIGSTOP / GC) must not install it."""
    def _fn(req, inner):
        rep = inner.solve(req)
        now = time.perf_counter()
        rep = dataclasses.replace(
            rep, t_inst_s=now + sk_solver.REPLY_TRANSIT_ALLOW_S)
        time.perf_counter = lambda: now + 0.5
        return rep
    node = _solver_node(_FakeSolver(_fn))
    before = node._active_plan
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_rest_req(at + 0.5),
                                         InstallSegment.Response())
    assert resp.accepted is False
    assert resp.code == sk_exec.ORIGIN_TOO_LATE
    assert 'reply transit' in resp.message
    assert node._active_plan is before and node._cycle is None


def test_a_normally_timed_fresh_rest_still_installs_under_the_stall_fence():
    def _fn(req, inner):
        rep = inner.solve(req)
        now = time.perf_counter()
        rep = dataclasses.replace(
            rep, t_inst_s=now + sk_solver.REPLY_TRANSIT_ALLOW_S)
        time.perf_counter = lambda: now + 0.009
        return rep
    node = _solver_node(_FakeSolver(_fn))
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_rest_req(at + 0.5),
                                         InstallSegment.Response())
    assert resp.accepted is True, resp.message


def test_a_normal_transit_delay_inside_the_wire_margin_still_installs():
    """The guard must not refuse the real thing: a few ms of transit + splice
    work (measured 6-9 ms) is far inside the 3-knot wire read margin."""
    node = _solver_node(_FakeSolver(_slow_return(0.010)))
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is True, resp.message


def test_a_late_reply_on_a_splice_is_refused_splice_too_late_and_the_record_stays():
    node = _solver_node(_FakeSolver())
    with _frozen_perf() as at:
        r1 = node._svc_install_segment(_throw_req(at + 0.6),
                                       InstallSegment.Response())
        assert r1.accepted is True, r1.message
        plan1, meta1, _t0 = node._cycle
        origin = at - 0.2
        node._cycle = (plan1, meta1, origin)
        node._plan_t0 = origin
        _refresh(node)
        node._segment_solver = _FakeSolver(_slow_return(0.4))
        # The telemetry-continuity check (a harness clock jump reads as drift
        # against the unmoved mock robot_state) runs BEFORE the transit guard;
        # neutralise it so THIS test reaches the guard it pins.
        node._install_continuity_ok = lambda *a, **k: True
        r2 = node._svc_install_segment(_catch_req(at + 0.7),
                                       InstallSegment.Response())
    assert r2.accepted is False
    assert r2.code == sk_exec.SPLICE_TOO_LATE, r2.message
    assert 'reply transit' in r2.message
    assert node._active_plan is plan1
    assert node._cycle[0] is plan1 and node._cycle[1] is meta1
    assert node._cycle[2] == origin


def test_a_missed_wire_deadline_maps_to_the_timing_code_the_fresh_predicate_implies():
    late = lambda req, _inner: sk_solver._refusal(  # noqa: E731
        req.seq, sk_solver.SKIPPED_LATE, 'deadline passed')
    node = _solver_node(_FakeSolver(late))
    with _frozen_perf() as at:
        fresh_resp = node._svc_install_segment(_throw_req(at + 0.6),
                                               InstallSegment.Response())
        assert fresh_resp.accepted is False
        assert fresh_resp.code == sk_exec.ORIGIN_TOO_LATE
        assert 'wire deadline' in fresh_resp.message
        rest_resp = node._svc_install_segment(_rest_req(at + 0.5),
                                              InstallSegment.Response())
        assert rest_resp.code == 'SOLVER_UNAVAILABLE'

        # A SPLICE: install a real THROW first, then miss the CATCH's deadline.
        node2 = _solver_node(_FakeSolver())
        r1 = node2._svc_install_segment(_throw_req(at + 0.6),
                                        InstallSegment.Response())
        assert r1.accepted is True, r1.message
        plan1, meta1, _ = node2._cycle
        node2._cycle = (plan1, meta1, at - 0.2)
        node2._plan_t0 = at - 0.2
        _refresh(node2)
        node2._segment_solver = _FakeSolver(late)
        splice = node2._svc_install_segment(_catch_req(at + 0.7),
                                            InstallSegment.Response())
    assert splice.code == sk_exec.SPLICE_TOO_LATE
    assert 'wire deadline' in splice.message
    assert node2._active_plan is plan1


def test_the_request_carries_every_input_and_the_per_kind_deadline():
    solver = _FakeSolver()
    node = _solver_node(solver)
    with _frozen_perf() as at:
        node._svc_install_segment(_throw_req(at + 0.6), InstallSegment.Response())
        node._svc_install_segment(_rest_req(at + 0.5), InstallSegment.Response())
    throw_req, rest_req = solver.requests[0], solver.requests[1]
    assert throw_req.seq < rest_req.seq
    # The THROW's deadline and lead are the THROW lead (2026-10-16): the node
    # derives it per kind from `schedule.install_lead_s`, the SAME function
    # the schedule dispatched the skill by, so the origin lands on the
    # skill's splice base; the REST keeps the general lead.
    assert throw_req.deadline_s == pytest.approx(
        at + sk_exec.FRESH_THROW_LEAD_S + float(hw.JB_TRAJ_KNOT_DT_S))
    assert rest_req.deadline_s == pytest.approx(
        at + sk_solver.SOLVER_REST_DEADLINE_S)
    assert throw_req.kind == sk_seg.THROW and rest_req.kind == sk_seg.REST
    # Live-mutable inputs travel per call, by the node's CURRENT binding.
    assert throw_req.limits is node._limits and throw_req.geom is node._geom
    assert throw_req.cfg is node._segment_cfg
    assert throw_req.lead_s == sk_exec.FRESH_THROW_LEAD_S
    assert rest_req.lead_s == sk_exec.LEAD_S == node._segment_lead_s
    assert throw_req.reserve_fresh_lead is True


def test_the_node_carries_the_replys_warm_start_and_release():
    sentinel = object()
    solver = _FakeSolver(lambda req, inner: dataclasses.replace(
        inner.solve(req), warm_start=sentinel))
    node = _solver_node(solver)
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is True, resp.message
    assert node._segment_warm_start is sentinel


def test_status_timer_polls_solver_health_and_shutdown_closes_it():
    solver = _FakeSolver()
    node = _solver_node(solver)
    node._publish_status()
    node._publish_status()
    assert solver.health_polls == 2
    node.on_shutdown()
    assert solver.closed == 1


def test_the_planner_warm_up_also_warms_the_worker_through_the_seam(monkeypatch):
    """A solver with a pid is a SEPARATE process: warming only this interpreter
    would claim a warm planner while the first live install pays the cold
    penalty.  The REST goes through `solve`, is dropped, installs nothing."""
    solver = _FakeSolver()
    solver.pid = 4242
    solver.wait_ready = lambda timeout_s=10.0: True
    node = _solver_node(solver)
    monkeypatch.setattr(tn.uc, 'plan_cycle', lambda *a, **kw: (None, None))
    node._warm_planner_once()
    assert [r.kind for r in solver.requests] == [sk_seg.REST]
    assert node._cycle is None and node._active_plan is not None  # seed hold only


def test_a_fresh_throw_installs_through_the_real_worker_process(monkeypatch):
    """End to end on the SHIPPED default: the node builds a real
    ``WorkerSegmentSolver`` (a spawned planner process), and one THROW goes
    ship -> solve in the worker -> reply -> transit guard -> ``_install``.
    Everything the in-process double cannot see: spawn/exec, env and sys.path
    inheritance, pickling of the real request/reply, the cross-process clock."""
    monkeypatch.undo()          # drop `in_process_segment_solver`: default factory
    node = _perf_node()
    try:
        assert isinstance(node._segment_solver, sk_solver.WorkerSegmentSolver)
        assert node._segment_solver.wait_ready(20.0), 'worker never became READY'
        # A real wall-clock solve can be legitimately late on a loaded box (the
        # gate runs 4 workers; a 175 ms fresh-origin budget is real): retry ONLY
        # the timing refusal, a few times -- any other refusal fails at once.
        for _attempt in range(4):
            _refresh(node)
            at = time.perf_counter()
            resp = node._svc_install_segment(_throw_req(at + 0.8),
                                             InstallSegment.Response())
            if resp.code != sk_exec.ORIGIN_TOO_LATE:
                break
        assert resp.accepted is True, resp.message
        assert resp.code == feas.OK
        assert isinstance(node._active_plan, CyclePlan)
        assert node._cycle is not None and node._cycle[0] is node._active_plan
    finally:
        node.on_shutdown()
        assert node._segment_solver.pid is None or node._segment_solver.state != 'READY'


# ═════════════════════════════════════════════════════════════════════════════
# The node's wait follows a PINNED splice's cliff (2026-10-14, SPLICE_TOO_LATE)
#
# A CATCH-with-throw dispatched HANDOFF_LEAD_S before the release the head
# carries splices AT that release knot (`executor._snap_to_release`), so its
# result stays useful until `t0 + (k_rel - WIRE_READ_KNOTS)*dt`, which can lie
# past the default `t_wall + LEAD_S + dt` (250 ms).  The fake below honours
# `req.deadline_s` the way `WorkerSegmentSolver._await` does, so the deadline
# the node ships is what decides.
# ═════════════════════════════════════════════════════════════════════════════

def _deadline_honouring(solve_s):
    """The real in-process solve at the frozen clock, then the clock jumps to
    ``t_wall + solve_s``; past ``req.deadline_s`` the answer is the client's
    own SKIPPED_LATE (``WorkerSegmentSolver._late``)."""
    def _fn(req, inner):
        t_wall = time.perf_counter()
        rep = inner.solve(req)
        time.perf_counter = lambda: t_wall + solve_s
        if t_wall + solve_s > req.deadline_s:
            return sk_solver._refusal(
                req.seq, sk_solver.SKIPPED_LATE,
                'solver did not answer by the wire deadline (no reply in time)')
        return rep
    return _fn


def _pinned_handoff(solve_s, early_s=0.010):
    """Install a THROW, re-anchor its origin so the CATCH-with-throw's dispatch
    sits ``HANDOFF_LEAD_S + early_s`` before the release knot (``early_s``
    models the up-to-one-knot quantisation between the release instant and the
    knot grid, i.e. a cliff past 250 ms), then request the catch through a
    solver that takes ``solve_s``.  Returns ``(node, resp, k_rel, at, origin,
    solver, debug_lines)``."""
    node = _solver_node(_FakeSolver())
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    with _frozen_perf() as at:
        r1 = node._svc_install_segment(_throw_req(at + 1.0),
                                       InstallSegment.Response())
        assert r1.accepted is True, r1.message
        plan1, meta1, _t0 = node._cycle
        k_rel = int(round(float(meta1.releases[0].t_s) / dt))
        origin = at - (k_rel * dt - (sk_exec.HANDOFF_LEAD_S + early_s))
        node._cycle = (plan1, meta1, origin)
        node._plan_t0 = origin
        _refresh(node)
        solver = _FakeSolver(_deadline_honouring(solve_s))
        node._segment_solver = solver
        # The harness clock jump reads as drift against the unmoved mock
        # robot_state; neutralise it so the timing path is what is tested.
        node._install_continuity_ok = lambda *a, **k: True
        lines = []
        node._logger.debug = lambda msg, **kw: lines.append(msg)
        t_rel = origin + k_rel * dt
        resp = node._svc_install_segment(
            _catch_throw_req(t_rel + 0.4, t_rel + 0.9),
            InstallSegment.Response())
    return node, resp, k_rel, at, origin, solver, lines


def test_a_pinned_handoff_catch_slower_than_the_default_wait_still_installs():
    """Solve 355 ms: past the default `t_wall + LEAD_S + dt` (250 ms) the node
    used to stop waiting at, inside the pinned cliff.  Before 2026-10-14 this
    was 'solver did not answer by the wire deadline'.

    Re-pinned 2026-10-16 for 8 extra knots: cliff - dispatch =
    (HANDOFF_LEAD_KNOTS 17 - WIRE_READ_KNOTS 3)*25 ms + 10 ms quantisation =
    360 ms; the node waits to cliff - 3 ms = 357 ms, so 355 ms installs."""
    dt = float(hw.JB_TRAJ_KNOT_DT_S)
    node, resp, k_rel, at, origin, solver, _ = _pinned_handoff(0.355)
    cliff = origin + (k_rel - sk_exec.WIRE_READ_KNOTS) * dt
    assert cliff - at == pytest.approx(0.360)
    req = solver.requests[-1]
    assert req.deadline_s == pytest.approx(
        cliff - sk_solver.REPLY_TRANSIT_ALLOW_S)
    assert req.deadline_s > at + sk_exec.LEAD_S + dt
    assert resp.accepted is True, resp.message
    assert resp.splice_k == k_rel and resp.seeded_post_release is True


def test_a_pinned_handoff_catch_past_its_cliff_is_refused_with_its_timing_logged():
    """Solve 362 ms (8-knot cliff 360 ms, deadline 357 ms; was 262 ms at 4
    knots): past the pinned cliff less the transit allowance.  Same
    refusal as before 2026-10-14 (SPLICE_TOO_LATE, nothing installed), and the
    DEBUG refusal line now carries the timing that was invisible in the
    sitting logs: which check refused, the deadline, k_s and k_wire."""
    node, resp, k_rel, at, origin, _solver, lines = _pinned_handoff(0.362)
    assert resp.accepted is False
    assert resp.code == sk_exec.SPLICE_TOO_LATE, resp.message
    assert 'wire deadline' in resp.message
    assert node._cycle[2] == origin           # the head is untouched
    assert resp.plan_wall_ms == pytest.approx(362.0)
    refused = [m for m in lines if m.startswith('install_segment refused')]
    assert len(refused) == 1
    line = refused[0]
    assert 'check=node-wait' in line
    assert 'deadline +357.0 ms' in line
    assert 'k_s %d' % (k_rel,) in line
    assert 'cliff +360.0 ms' in line
    assert 'k_wire@node' in line


def test_an_unpinned_splice_keeps_the_default_wait():
    """A plain CATCH (no release before its event to pin it) splices at the
    first knot LEAD_S ahead, whose cliff is inside `t_wall + LEAD_S + dt`:
    the shipped deadline is the default, unchanged."""
    solver = _FakeSolver()
    node = _solver_node(solver)
    with _frozen_perf() as at:
        r1 = node._svc_install_segment(_throw_req(at + 0.6),
                                       InstallSegment.Response())
        assert r1.accepted is True, r1.message
        plan1, meta1, _ = node._cycle
        node._cycle = (plan1, meta1, at - 0.2)
        node._plan_t0 = at - 0.2
        _refresh(node)
        node._svc_install_segment(_catch_req(at + 0.7),
                                  InstallSegment.Response())
    assert solver.requests[-1].deadline_s == pytest.approx(
        at + sk_exec.LEAD_S + float(hw.JB_TRAJ_KNOT_DT_S))


# ═════════════════════════════════════════════════════════════════════════════
# KIND_BUMP — the late catch re-aim bump on the active record (I-CATCH-5, G2)
# ═════════════════════════════════════════════════════════════════════════════

def _bump_node_and_record(at):
    """A node streaming a THROW -> CATCH-with-throw record (the G1 fixture's
    columns geometry) whose catch is 0.45 s after ``at`` (perf clock)."""
    from jugglebot.motion.skills import sites as si
    from jugglebot.motion.trajectory.limits import TrajectoryLimits
    from tests.motion import test_skills_bump as tb
    node = _perf_node()
    node._limits = TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3900.0)
    p1, p2 = si.columns_sites(100.0)
    t_land = at + 0.45
    t_rel0 = t_land - tb.FLIGHT_S
    rec, res, _ = sk_exec.install_segment(
        None, tb._rest_state(p1.rest_site_mm()), sk_seg.THROW,
        sk_seg.ThrowTerminal(site_mm=p1.throw_site_mm(), target_mm=p1.catch_site_mm(),
                             flight_s=tb.FLIGHT_S, t_release_s=t_rel0),
        t_rel0 - tb.LAUNCH_S, limits=node._limits, geom=node._geom)
    assert res.accepted, res.message
    term = sk_seg.CatchTerminal(
        landing_mm=p2.catch_site_mm(), landing_vel_mm_s=tb.LAND_VEL,
        t_land_s=t_land, rest_site_mm=p2.rest_site_mm(),
        then_throw=sk_seg.ThrowAfterCatch(t_release_s=t_land + tb.DWELL_S,
                                          site_mm=p2.throw_site_mm(),
                                          target_mm=p2.catch_site_mm(),
                                          flight_s=tb.FLIGHT_S))
    rec, res, _ = sk_exec.install_segment(rec, None, sk_seg.CATCH, term,
                                          t_rel0 - sk_exec.LEAD_S,
                                          limits=node._limits, geom=node._geom)
    assert res.accepted, res.message
    with node._plan_lock:
        node._active_plan = rec.plan
        node._plan_t0 = rec.t0_s
        node._cycle = (rec.plan, rec.meta, rec.t0_s)
        node._handoff = None
    return node, rec, t_land


def _bump_req(t_land, t_dispatch, delta=(20.0, 0.0)):
    req = InstallSegment.Request()
    req.kind = req.KIND_BUMP
    req.t_event_s = float(t_land)
    req.t_dispatch_s = float(t_dispatch)
    req.bump_delta_xy_mm = [float(delta[0]), float(delta[1])]
    return req


def test_a_bump_installs_on_the_active_record_with_the_same_t0():
    from jugglebot.motion.skills import bump as sk_bump
    with _frozen_perf() as at:
        node, rec, t_land = _bump_node_and_record(at)
        _refresh(node)
        resp = node._svc_install_segment(_bump_req(t_land, at),
                                         InstallSegment.Response())
    assert resp.accepted, (resp.code, resp.message)
    assert resp.terminal is False and resp.code == 'OK'
    new = node._active_plan
    assert new is not rec.plan and node._cycle[0] is new
    assert node._plan_t0 == rec.t0_s and node._cycle[2] == rec.t0_s
    assert node._cycle_owner == tn._OWNER_SEGMENT and node._handoff is None
    k_b = sk_bump.k_wire_at(rec.t0_s, at, rec.plan.dt) + sk_bump.BUMP_EXTRA_KNOTS
    assert resp.splice_k == k_b
    # Every knot the wire may have read, and k_b, bit-identical; k_b+1 moved.
    assert np.array_equal(new.pose[:k_b + 1], rec.plan.pose[:k_b + 1])
    assert np.array_equal(new.pose_vel[:k_b + 1], rec.plan.pose_vel[:k_b + 1])
    assert not np.array_equal(new.pose[k_b + 1:], rec.plan.pose[k_b + 1:])
    # The hand lane is untouched.
    assert np.array_equal(new.hand_rev, rec.plan.hand_rev)
    assert np.array_equal(new.hand_vel_rps, rec.plan.hand_vel_rps)
    b_rel = np.asarray(resp.release_offset_xy_mm)
    assert 0.0 < float(np.hypot(*b_rel)) < 20.0
    assert float(np.hypot(*resp.launch_bias_xy_mm_s)) <= sk_bump.BUMP_MAX_LAUNCH_LATERAL_MM_S
    assert resp.t_release_mono > t_land
    # `_install` clears the predicted peaks; the bump writes the bumped plan's
    # report AFTER it (so /trajectory/status and /diagnose are not 0.0).
    assert node._last_peak_vel_mmps > 0.0
    assert node._last_peak_acc_mmps2 > 0.0
    assert node._last_peak_jerk_mmps3 > 0.0


def _record_unchanged(node, rec):
    assert node._active_plan is rec.plan
    assert node._cycle[0] is rec.plan and node._cycle[2] == rec.t0_s
    assert node._plan_t0 == rec.t0_s


def test_a_clock_that_passes_k_b_during_the_compute_is_splice_too_late_at_the_swap():
    """The swap-time k_wire re-check (under `_plan_lock`): the entry check
    passed, then the clock moves past k_b before `_install` -> non-terminal
    SPLICE_TOO_LATE, record untouched."""
    from jugglebot.motion.skills import bump as sk_bump
    real = sk_bump.bump_record
    with _frozen_perf() as at:
        node, rec, t_land = _bump_node_and_record(at)
        _refresh(node)

        def slow(*a, **k):
            out = real(*a, **k)
            time.perf_counter = lambda: at + 0.5   # restored by _frozen_perf
            return out
        sk_bump.bump_record = slow
        try:
            resp = node._svc_install_segment(_bump_req(t_land, at),
                                             InstallSegment.Response())
        finally:
            sk_bump.bump_record = real
        time.perf_counter = lambda: at
    assert not resp.accepted and resp.code == sk_exec.SPLICE_TOO_LATE
    assert resp.terminal is False
    _record_unchanged(node, rec)


def test_a_record_superseded_during_the_compute_is_a_terminal_no_record():
    from jugglebot.motion.skills import bump as sk_bump
    real = sk_bump.bump_record
    with _frozen_perf() as at:
        node, rec, t_land = _bump_node_and_record(at)
        _refresh(node)
        other = object()

        def swap(*a, **k):
            out = real(*a, **k)
            node._active_plan = other      # a hold/stop landed meanwhile
            return out
        sk_bump.bump_record = swap
        try:
            resp = node._svc_install_segment(_bump_req(t_land, at),
                                             InstallSegment.Response())
        finally:
            sk_bump.bump_record = real
    assert not resp.accepted and resp.code == 'BUMP-NO-RECORD'
    assert resp.terminal is True
    assert node._active_plan is other and node._cycle[0] is rec.plan
    assert node._plan_t0 == rec.t0_s


def test_install_with_a_false_precondition_leaves_the_record_untouched():
    with _frozen_perf() as at:
        node, rec, _t_land = _bump_node_and_record(at)
        cycle = node._cycle
        ok = node._install(rec.plan, t0=rec.t0_s + 1.0, cycle=cycle,
                           cycle_owner=tn._OWNER_SEGMENT,
                           precondition=lambda: False)
    assert ok is False
    assert node._cycle is cycle
    _record_unchanged(node, rec)


def test_a_bump_is_refused_while_a_stream_handoff_is_pending():
    from jugglebot.motion.trajectory.stream_handoff import StreamHandoff
    with _frozen_perf() as at:
        node, rec, t_land = _bump_node_and_record(at)
        _refresh(node)
        with node._plan_lock:
            node._handoff = StreamHandoff(rec.plan, at - 1.0, rec.plan,
                                          rec.t0_s)
        resp = node._svc_install_segment(_bump_req(t_land, at),
                                         InstallSegment.Response())
    assert not resp.accepted and resp.code == 'BUMP_HANDOFF'
    assert resp.terminal is False
    assert node._active_plan is rec.plan and node._handoff is not None


def test_a_late_bump_apply_is_a_non_terminal_splice_too_late():
    """Dispatched 4 knots before the service saw it: k_b = k_wire(dispatch) +
    2 <= k_wire(now) — refused SPLICE_TOO_LATE, non-terminal, record kept."""
    with _frozen_perf() as at:
        node, rec, t_land = _bump_node_and_record(at)
        _refresh(node)
        resp = node._svc_install_segment(_bump_req(t_land, at - 0.100),
                                         InstallSegment.Response())
    assert not resp.accepted and resp.code == sk_exec.SPLICE_TOO_LATE
    assert resp.terminal is False
    assert node._active_plan is rec.plan and node._cycle[0] is rec.plan


def test_a_bump_with_no_matching_catch_in_the_record_is_terminal():
    with _frozen_perf() as at:
        node, rec, t_land = _bump_node_and_record(at)
        _refresh(node)
        resp = node._svc_install_segment(_bump_req(t_land + 0.2, at),
                                         InstallSegment.Response())
    assert not resp.accepted and resp.code == 'BUMP-NO-RECORD'
    assert resp.terminal is True and node._active_plan is rec.plan


# ═════════════════════════════════════════════════════════════════════════════
# planner_mode on the wire (two-ball-oval P3, 2026-10-23)
# ═════════════════════════════════════════════════════════════════════════════
# ONE live switch: skill_node's `planner_mode` parameter rides on every
# InstallSegment request and trajectory_node plans THAT segment with it; the
# node's own SegmentConfig.planner_mode only answers a request that carries
# none (''), so the box selected and the plan flown cannot disagree.

def _fresh_catch_throw(node, at, mode):
    req = _catch_throw_req(at + 0.5, at + 1.0)
    if mode is not None:
        req.planner_mode = mode
    return node._svc_install_segment(req, InstallSegment.Response())


def test_a_level_request_overrides_the_pose_default_per_install():
    solver = _FakeSolver()
    node = _solver_node(solver)
    with _frozen_perf() as at:
        resp = _fresh_catch_throw(node, at, 'level')
    assert resp.accepted is True, resp.message
    assert resp.planner_mode == 'level'
    assert solver.requests[0].cfg.planner_mode == 'level'
    # per-install only: the node's own default is untouched
    assert node._segment_cfg.planner_mode == 'pose'


def test_an_empty_planner_mode_uses_the_nodes_default():
    solver = _FakeSolver()
    node = _solver_node(solver)
    with _frozen_perf() as at:
        resp = _fresh_catch_throw(node, at, '')
    assert resp.accepted is True, resp.message
    assert resp.planner_mode == 'pose'
    assert solver.requests[0].cfg is node._segment_cfg


def test_an_unknown_planner_mode_is_refused_by_name_before_any_solve():
    solver = _FakeSolver()
    node = _solver_node(solver)
    active_before, cycle_before = node._active_plan, node._cycle
    with _frozen_perf() as at:
        resp = _fresh_catch_throw(node, at, 'banana')
    assert resp.accepted is False
    assert resp.code == 'PLANNER_MODE'
    assert 'banana' in resp.message and 'pose' in resp.message
    assert solver.requests == []
    assert node._active_plan is active_before and node._cycle is cycle_before


def test_the_planner_mode_parameter_is_the_default_for_an_empty_request():
    node = _perf_node()
    assert node._segment_cfg.planner_mode == 'pose'


def test_the_solve_cpu_suffix_rides_inside_the_solve_phrase():
    """JB-76: a worker reply's CPU time and context switches are logged as a
    suffix after ``solve X ms`` (the phrase the launch-log greps key on); a
    reply without them (refusal, in-process double) adds nothing."""
    rep = SimpleNamespace(cpu_s=0.0652, ivcsw=12, vcsw=3)
    assert tn._solve_cpu_suffix(rep) == ' (cpu 65.2 ms, ivcsw 12, vcsw 3)'
    assert tn._solve_cpu_suffix(SimpleNamespace(cpu_s=math.nan)) == ''
    assert tn._solve_cpu_suffix(SimpleNamespace()) == ''
