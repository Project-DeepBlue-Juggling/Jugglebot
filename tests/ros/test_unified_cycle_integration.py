"""Wave B integration + unit tests for the unified 7-DoF cycle (plan Phase 4).

Four things are pinned here that nothing else in the suite can see:

1. **T-I1 — the :5557 seam, end to end.** A real ``TrajectoryNode`` plans and
   installs a real ``CyclePlan`` through the real ``trajectory/plan_cycle``
   handler, its emitter publishes on a real ``MpcCommandPub`` bound to an
   EPHEMERAL port, a real ``TeensyBridgeNode`` consumes that stream through the
   real ``_MpcCommandSetpointSource`` + ``SetpointPump``, and a loopback UDP sink
   receives the actual v6 ``Setpoint`` bytes. The claim is that the seven-channel
   path is live: ``HAS_HAND`` and ``HAS_V1`` set while the cycle plan streams, and
   CLEAR again on the falling edge when a legacy plan supersedes it — the exact
   edge FW 17's hand-lane decay is written against.

2. **T-I3 — the interlock choreography.** Under unified mode the reactive hand
   arm is provably never dispatched, the legacy kind-0 throw RPC is never issued,
   the deferred A→B reach is never published, the pipeline is forced off, and the
   can-bridge hand-mastery latch is VERIFIED at session start and left exactly
   where the operator put it at the terminal. Under legacy the same spies fire
   exactly as they do today.

3. **The trajectory_node service itself** — NEW / EXTEND / REPLAN happy paths and
   their refusals, the hand continuity term, the replan bound, and the
   ``catch/dynamic_target`` routing.

4. **The wire defaults** — the ``unified_cycle`` goal field ships FALSE in the
   ``.action`` IDL and in the conftest mock, and the session reads it ONCE.

ROS 2 is mocked by ``tests/ros/conftest.py``; the UDP and ZMQ transports are
REAL, on ephemeral ports, so the whole file is xdist-parallel-safe.
"""

from __future__ import annotations

import dataclasses
import time
import types
from pathlib import Path

import numpy as np
import pytest

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Quaternion, Vector3

from jugglebot_interfaces.msg import DynamicTargetCommand, MotorStateSingle, RobotState
from jugglebot_interfaces.srv import PlanCycle

import jugglebot.hardware_config as hw
from jugglebot import reload_coordinator_node as rcn
from jugglebot.catch_coordinator_node import CatchCoordinatorNode
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory.cycle_plan import CyclePlan
from jugglebot.toss_sequencer import (
    ACTION_ANNOUNCE,
    ACTION_REACH_CATCH,
    TossDecision,
    TossResult,
    TossSequencer,
)
from jugglebot import trajectory_node as tn
from jugglebot.trajectory_node import TrajectoryNode

from tests.ros.test_toss_continuous_node import (
    _Clock,
    _ContGoalHandle,
    _ready_node,
    _stamp,
    _stub_cycles,
)


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
    return (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.LINEAR_GAIN_REV_PER_M


_REST_HAND_REV = _hand_rev_for_cup_z(_REST_CUP_Z_MM)

# Session limits a unified sitting raises to at start (plan Phase 1, owner
# decision 1) — the ones `sim/cycle_gate.py` and `tests/motion/test_unified_cycle`
# both run at. The SHIPPED leg jerk reads LIMIT_JERK on every gate cycle for a
# structural reason recorded there, so planning against it would test that known
# fact rather than this wiring.
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
    """A seeded, TRAJECTORY-mode node at session limits, emitter NOT started."""
    from jugglebot_interfaces.srv import SetTrajectoryLimits
    node = TrajectoryNode(start_emitter=False, **kw)
    node._on_robot_state(_robot_state())
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
    never more than 10 ms old at a service entry. Here a single ~250 ms plan+gate
    is enough to walk past the node's 0.5 s staleness bound, and back-to-back
    planning calls in one test would then refuse STALE_STATE for a reason that
    exists only in the harness. Re-stamping between calls reproduces the live
    graph rather than relaxing the guard — the guard itself is exercised
    explicitly by ``test_plan_cycle_guards_mirror_the_timed_target_ladder``.
    """
    node._on_robot_state(_robot_state())
    return node


def _hold_head(node):
    """Re-anchor the installed cycle's origin at NOW.

    An EXTEND is only meaningful while the plan it chains onto is STILL PLAYING —
    ``unified_cycle.extend``'s bit-identical-head guarantee is worth nothing once
    the emitter has run off the end (the coordinator's ``_UNIFIED_EXTEND_LEAD_S``
    is exactly that rule, on the production side). In this harness the LAUNCH's
    0.6 s window can expire during a solve that is competing with the rest of the
    suite for the Jetson's cores, which would make these tests fail for a load
    reason rather than a wiring one. Re-anchoring reproduces the production
    precondition; the case where the head HAS run out is a real refusal and is
    left to the guard.
    """
    plan, meta, _t0 = node._cycle
    now = time.perf_counter()
    node._cycle = (plan, meta, now)
    node._plan_t0 = now
    return node


def _extend_alive(node, req, attempts=4):
    """Run an EXTEND with the head GUARANTEED still playing. ``(resp, origin)``.

    `_hold_head` reproduces the production precondition by re-anchoring the
    origin at NOW, but the solve then runs for a few hundred ms — and on a busy
    box (four xdist workers, say) it can run for longer than the 0.6 s window it
    re-anchored, at which point the continuity check samples PAST the head and
    the install is refused STALE_STATE. That refusal is CORRECT — the head really
    did run out — so the fix is to reproduce the precondition again rather than
    to relax the guard or to lengthen every window. Retrying keeps the check
    meaningful (each attempt still compares a live mid-window sample) while
    removing a failure that is purely about how long the box took.

    Returns the origin the accepted attempt was anchored at, because a caller
    asserting "the EXTEND kept the origin" has to know which one.
    """
    resp = None
    for _ in range(attempts):
        _hold_head(_refresh(node))
        origin = node._cycle[2]
        resp = node._svc_plan_cycle(req, PlanCycle.Response())
        if resp.accepted or resp.code != feas.STALE_STATE:
            return resp, origin
    return resp, node._cycle[2]


def _launch_req(period_s=0.6, flight_s=0.6, throw_z=860.0):
    req = PlanCycle.Request()
    req.mode = req.MODE_NEW
    req.kind = req.KIND_LAUNCH
    req.period_s = period_s
    req.throw_site_mm = [0.0, 0.0, throw_z]
    req.throw_target_mm = [0.0, 0.0, throw_z]
    req.flight_s = flight_s
    req.catch_site_mm = [0.0, 0.0, 830.0]
    req.catch_vel_mm_s = [0.0, 0.0, -2500.0]
    req.catch_frac = 0.0
    req.settle_site_mm = [0.0, 0.0, _REST_CUP_Z_MM]
    req.banking_enabled = True
    req.lead_s = 0.0
    return req


def _landing_req(period_s=1.0, catch_t_s=0.6):
    req = _launch_req()
    req.mode = req.MODE_EXTEND
    req.kind = req.KIND_LANDING
    req.period_s = period_s
    req.catch_frac = catch_t_s / period_s
    return req


# ═════════════════════════════════════════════════════════════════════════════
# The service: NEW / EXTEND / REPLAN
# ═════════════════════════════════════════════════════════════════════════════

def test_plan_cycle_new_launch_installs_a_seven_channel_plan():
    """MODE_NEW plans a LAUNCH from the LIVE commanded state and installs it.

    The response's `t_release_mono` is the plan origin plus the window, which is
    what the announcement's throw_time is built from — so it is asserted against
    the installed plan rather than against the request.
    """
    node = _cycle_node()
    resp = node._svc_plan_cycle(_launch_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    assert resp.code == feas.OK
    assert isinstance(node._active_plan, CyclePlan)
    assert node._cycle is not None
    plan, meta, t0 = node._cycle
    assert plan is node._active_plan
    assert resp.t0_mono == pytest.approx(t0)
    assert resp.t_release_mono == pytest.approx(t0 + plan.total_duration)
    assert resp.duration_s == pytest.approx(0.6, abs=1e-9)
    # The release velocity is the QP's pinned take-off, up (a vertical self-toss).
    assert resp.release_vel_mm_s[2] > 1000.0
    assert resp.hand_peak_vel_rps > 0.0
    assert resp.plan_wall_ms > 0.0
    assert resp.replans_used == 0


def test_plan_cycle_new_launch_measures_its_own_wall_time():
    """`plan_wall_ms` is the WHOLE callback, and it lands on trajectory/status.

    The budget the owner confirmed is split (core <= 50 ms, total <= 250 ms) and
    nothing can be judged against it unless the node publishes what it measured.
    This pins the plumbing, NOT the number — a threshold here would be a
    machine-speed assertion in a functional test.
    """
    node = _cycle_node()
    resp = node._svc_plan_cycle(_launch_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    assert node._cycle_plan_wall_ms == pytest.approx(resp.plan_wall_ms)
    node._publish_status()
    status = node._publishers['trajectory/status'].published[-1]
    assert status.cycle_active is True
    assert status.cycle_plan_wall_ms == pytest.approx(resp.plan_wall_ms)
    assert status.cycle_hand_peak_rev > 0.0


def _settle_req(period_s=1.4, dx_mm=60.0, dy_mm=0.0):
    """MODE_NEW + KIND_SETTLE — the UH-3 banked-carry rung's request.

    A pure lateral re-pose of the CUP at a fixed cup z, planned from rest. The
    throw/catch fields are inert for this kind and are left at the zeros the
    service defaults to, exactly as `unified_cycle_bench.py` builds it.

    The cup z is `unified_cycle.SETTLE_CUP_Z_MM` (689.6 mm), NOT this module's
    750 mm launch rest: that is the height the bench driver actually asks for and
    the height every cycle settles at, and the free-fall defect below is an order
    of magnitude larger there (MEASURED 2026-09-05, `/tmp/probe_a6_freefall.py`:
    2.3845 rev of slider and a 75.42 mm cup-z arc at 689.6 mm, against 0.8012 rev
    and 25.34 mm at 750 mm). A regression test wants the configuration the rung
    flies at, and the one where the fault is loudest.
    """
    req = PlanCycle.Request()
    req.mode = req.MODE_NEW
    req.kind = req.KIND_SETTLE
    req.period_s = period_s
    req.throw_site_mm = [0.0, 0.0, 0.0]
    req.throw_target_mm = [0.0, 0.0, 0.0]
    req.flight_s = 0.0
    req.catch_site_mm = [0.0, 0.0, 0.0]
    req.catch_vel_mm_s = [0.0, 0.0, 0.0]
    req.catch_frac = 0.0
    req.settle_site_mm = [dx_mm, dy_mm, float(uc.SETTLE_CUP_Z_MM)]
    req.banking_enabled = True
    req.lead_s = 0.0
    return req


def _settle_node():
    """A node whose hand sits at the SETTLE height the `_settle_req` asks for.

    `_cycle_node`'s seed is the 750 mm launch rest; a SETTLE planned from there
    to 689.6 mm would be a 60 mm CARRY plus a 60 mm DESCENT, and the descent is
    slider travel the assertions below are trying to measure the absence of.
    """
    node = _cycle_node()
    node._on_robot_state(_robot_state(
        hand_rev=_hand_rev_for_cup_z(float(uc.SETTLE_CUP_Z_MM))))
    return node


def test_a_post_release_kind_planned_from_REST_is_not_given_free_fall():
    """A `KIND_SETTLE` at MODE_NEW off a terminal hold plans FLAT, not falling.

    `to_cup_state` defaults a post-release window's cup acceleration to `g` —
    correct for the case it was written for (chain from a state measured just
    after a release, ball in the air, cup following it) and simply false for a
    post-release KIND planned at MODE_NEW from a stationary machine, where there
    is no ball in the air.

    It is not cosmetic, which is why this is a test and not a comment. MEASURED
    (2026-09-05, `/tmp/probe_a6_freefall.py`, this same 60 mm lateral carry at
    1.4 s, banking on, session limits 250/3000/150000), **at the request's own
    cup height** `SETTLE_CUP_Z_MM` = 689.6 mm: with the `g` fallback knot 0
    carried cup a_z = -9.806 m/s^2 — apparent gravity in the cup EXACTLY ZERO
    for one 25 ms knot, i.e. the seated ball goes weightless — knot 1 reversed
    to +19.612 m/s^2, and the cup arced 75.42 mm UPWARD across a move that is
    purely lateral (hand 0.3161 -> 2.7007 rev, 2.3845 rev of travel = 9.5x the
    bar below). The same carry at this module's 750 mm launch rest costs only
    0.8012 rev and a 25.34 mm arc (3.2x the bar), which is why the request is
    built at 689.6: it is both the height the bench rung flies at and the one
    where the fault is loudest. `validate_cycle` accepts BOTH shapes, so nothing
    downstream refuses the falling one; the guard is `_cycle_start_state` or
    nowhere. UH-3's whole pass criterion is "no visible ball disturbance".

    Asserted on the HAND channel because that is where a cup-z excursion lands
    under the z = 170 centroid pin, and it is the channel the bench driver and
    the firmware guards both watch.
    """
    node = _settle_node()
    resp = node._svc_plan_cycle(_settle_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    plan, _meta, _t0 = node._cycle
    start = float(plan.hand_rev[0])
    # A flat carry moves the slider only by what the tilt schedule's lever-arm
    # compensation asks for (sub-millimetre at these accelerations). The free-
    # fall start cost 2.3845 rev; 0.25 rev (8 mm of slider) separates the two by
    # an order of magnitude in both directions.
    assert float(np.max(np.abs(plan.hand_rev - start))) < 0.25, (
        'the carry bowed the cup: hand travelled %.3f rev from %.3f — the '
        'free-fall boundary condition is back'
        % (float(np.max(np.abs(plan.hand_rev - start))), start))
    assert resp.hand_peak_rev == pytest.approx(start, abs=0.25)


def test_a_post_release_kind_planned_over_a_LIVE_plan_keeps_free_fall():
    """The `g` fallback survives where it is TRUE — a genuinely moving machine.

    The fix above is scoped by the SAME rest predicate the LAUNCH branch already
    trusts to declare exact zero acceleration, so it cannot leak into the case it
    was written for. Here the machine is moving, `at_rest` is false, and the
    state must carry no cup acceleration at all — leaving `to_cup_state` to
    supply `g`, which is what a chain from a real release means.
    """
    node = _cycle_node()
    # Ten times the node's own linear rest bound, read from the node rather than
    # hardcoded — the bound is a thousandth of the LIVE session velocity limit,
    # and a literal here would silently stop meaning "moving" the day the session
    # raises or lowers that limit.
    rest_bound = 1e-3 * node._limits.leg_vel_mmps
    twist = np.zeros(6)
    twist[0] = 10.0 * rest_bound
    monkey = (np.zeros(6), twist, np.zeros(6))
    node._current_state = lambda: monkey  # noqa: E731
    state, err = node._cycle_start_state(uc.SETTLE)
    assert err == '' and state is not None
    assert state.post_release is True
    assert state.cup_accel_mm_s2 is None
    # And the LAUNCH branch still refuses outright over a live plan.
    state, err = node._cycle_start_state(uc.LAUNCH)
    assert state is None and 'starts from REST' in err


def test_the_rest_predicate_is_scaled_off_the_LIVE_session_LIMITS():
    """The stopped-detector's bound, on all three channels the state carries.

    Until 2026-09-05 the platform bound was `max_step_rev / mm_to_rev / knot_dt`
    — the largest per-knot |Δu0| the firmware will accept, read as a speed. That
    is a PUMP SAFETY CEILING, not a stopped-detector: MEASURED (2026-09-05,
    `/tmp/probe_a1_a3.py`) it is **841.04 mm/s**, against the **250 mm/s** a
    unified sitting commands, so `at_rest` was true at every platform speed the
    machine can reach and the predicate could not fire. A window planned during a
    live move was then handed EXACT zero cup acceleration alongside a non-zero
    forward-mapped velocity — a boundary condition describing no machine at all.

    The bound is now a thousandth of the LIVE commanded velocity limit on each
    channel: **0.25 mm/s** linear, **0.00114 rad/s** angular (that thousandth
    carried to the platform rim through `GEOM_PLAT_RADIUS_MM` = 219.075 mm) and
    **0.20 rev/s** on the hand. The angular term is not decoration: a banked
    carry is largely rotation, and without it a platform tilting at any rate at
    all read as stopped.
    """
    def seeded(twist=None, hand_vel=None):
        node = _cycle_node()
        tw = np.zeros(6) if twist is None else np.asarray(twist, dtype=float)
        node._current_state = lambda: (np.zeros(6), tw, np.zeros(6))  # noqa: E731
        if hand_vel is not None:
            node._commanded_hand_state = (                            # noqa: E731
                lambda: (_REST_HAND_REV, float(hand_vel)))
        return node

    # A true rest seeds the EXACT zeros — and, since no ball left this cup, a
    # state that does not claim one did.
    at_rest, err = seeded()._cycle_start_state(uc.SETTLE)
    assert err == '' and at_rest is not None
    assert at_rest.post_release is False
    assert at_rest.cup_accel_mm_s2 is not None
    assert float(np.max(np.abs(at_rest.cup_accel_mm_s2))) == 0.0
    assert seeded()._cycle_start_state(uc.LAUNCH)[0] is not None

    # And these three are NOT at rest. Each is a channel the old predicate was
    # blind to: 5 mm/s is 20x the linear bound and 1/168th of the OLD one;
    # 0.02 rad/s carries no linear component at all; 0.5 rev/s is the hand.
    for label, twist, hand_vel in (
            ('5 mm/s linear', [5.0, 0.0, 0.0, 0.0, 0.0, 0.0], None),
            ('0.02 rad/s angular', [0.0, 0.0, 0.0, 0.02, 0.0, 0.0], None),
            ('0.5 rev/s hand', None, 0.5)):
        node = seeded(twist, hand_vel)
        state, err = node._cycle_start_state(uc.SETTLE)
        assert err == '' and state is not None, label
        assert state.post_release is True, label
        assert state.cup_accel_mm_s2 is None, label
        state, err = node._cycle_start_state(uc.LAUNCH)
        assert state is None and 'starts from REST' in err, label


def test_plan_cycle_extend_keeps_the_origin_and_the_head():
    """EXTEND re-installs at the SAME origin with a bit-identical head.

    That property is what makes a chain safe: the emitter has already sent knots
    out of the head, and a joined plan whose head moved would step six legs and
    the slider inside one 25 ms knot.
    """
    node = _cycle_node()
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    head_plan, _meta, t0 = node._cycle
    head_pose = head_plan.pose.copy()
    head_hand = head_plan.hand_rev.copy()
    n_head = head_plan.n_knots

    resp, head_t0 = _extend_alive(node, _landing_req())
    assert resp.accepted is True, resp.message
    joined, meta, t0_after = node._cycle
    t0 = head_t0
    assert t0_after == pytest.approx(t0)          # SAME origin
    assert resp.t0_mono == pytest.approx(t0)
    assert joined.n_knots > n_head
    # Bit-identical head, all four channels.
    assert np.array_equal(joined.pose[:n_head], head_pose)
    assert np.array_equal(joined.hand_rev[:n_head], head_hand)
    # The LANDING contributes the catch, so the response now carries one.
    assert resp.t_catch_mono > resp.t0_mono
    assert resp.arm_lead_s > 0.0


def test_the_reported_release_is_the_one_still_AHEAD_not_a_spent_one():
    """`t_release_mono` after an EXTEND is the NEXT throw, not the last one.

    An EXTEND joins a new window onto the active plan and re-installs at the SAME
    origin, so the joined meta carries every release the plan has ever had —
    including the one the ball already left on. Reporting `releases[0]` there
    hands the coordinator a SPENT instant, and two consumers break on it in ways
    that read as physics rather than as bookkeeping: the chained branch compares
    it against the cycle's schedule and refuses `CHAIN_SKEW` on every cycle, and
    `_expected_landing_perf` cuts the hand ball sensor's arrival window around a
    throw that already happened, so a real catch reads as a MISS.

    Driven on `_accept_cycle` directly against a two-release meta — the shape an
    EXTEND produces — because the alternative is a second multi-second solve to
    manufacture one.
    """
    node = _cycle_node()
    resp = node._svc_plan_cycle(_launch_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    plan, meta, _t0 = node._cycle
    first = meta.releases[0]
    second = dataclasses.replace(first, t_s=float(first.t_s) + 0.9)
    joined = dataclasses.replace(meta, releases=(first, second))
    now = time.perf_counter()

    def _released_at(t0):
        r = node._accept_cycle(PlanCycle.Response(), plan, joined, t0, now)
        return r.t_release_mono - t0

    # Nothing has passed ⇒ the FIRST release, exactly as before.
    assert _released_at(now) == pytest.approx(float(first.t_s), abs=1e-3)
    # The first release is behind us ⇒ the second one is reported.
    assert _released_at(now - float(first.t_s) - 0.1) == pytest.approx(
        float(second.t_s), abs=1e-3)
    # Every release has passed ⇒ the LAST, never 0.0. A rest-terminal plan
    # sampled after its final throw still threw; "never" would be a lie the
    # coordinator's `> now` guard would silently swallow.
    assert _released_at(now - float(second.t_s) - 0.1) == pytest.approx(
        float(second.t_s), abs=1e-3)


def test_plan_cycle_extend_without_an_active_cycle_is_refused():
    node = _cycle_node()
    resp = node._svc_plan_cycle(_landing_req(), PlanCycle.Response())
    assert resp.accepted is False
    assert resp.code == 'NO_CYCLE'
    assert node._active_plan is None or not isinstance(node._active_plan,
                                                       CyclePlan)


#: The shipped cycle's flight time, and the numbers below are quoted at it.
_SHIPPED_FLIGHT_S = 0.8
#: `reload_coordinator_node._UNIFIED_LAUNCH_WINDOW_S`.
_SHIPPED_LAUNCH_S = 0.6
#: Where on the installed cycle's clock a landing update is driven from.
#:
#: It has to clear TWO bounds and it sits between them. Below, the splice would
#: land on or before the launch release (knot 24 of 81) or inside its detach cone
#: — `replan_tail` refuses that, because the tail is re-solved as a LANDING and
#: would erase the throw and its cone rows. Above, the splice would reach the
#: catch (knot 56) and there would be nothing left to re-aim. 0.70 s + the 0.30 s
#: lead puts `k_s` at 40, comfortably inside (MEASURED 2026-09-05,
#: /tmp/probe_ros_shipped_replan.py, run twice with identical output).
_SHIPPED_REPLAN_TAU_S = 0.70


def _shipped_req():
    """The coordinator's ONE install: LAUNCH chained to its LANDING.

    Byte-for-byte the request `reload_coordinator_node._toss_unified_start_cycle`
    builds — including `settle_site_mm` at `unified_cycle.SETTLE_CUP_Z_MM`, which
    is the cup box's own floor and therefore the boundary case the planner's
    settle pin is chosen for. The resulting plan is REST-terminal, which is the
    point of the chain: a release-terminal plan streamed to its end commands a
    hard stop at the throw.
    """
    chain_period = _SHIPPED_FLIGHT_S + _SHIPPED_LAUNCH_S
    req = _launch_req(period_s=_SHIPPED_LAUNCH_S, flight_s=_SHIPPED_FLIGHT_S)
    req.settle_site_mm = [0.0, 0.0, float(rcn.uc.SETTLE_CUP_Z_MM)]
    req.catch_vel_mm_s = [0.0, 0.0, -0.5 * 9806.0 * _SHIPPED_FLIGHT_S]
    req.chain = True
    req.chain_kind = req.KIND_LANDING
    req.chain_period_s = chain_period
    req.chain_catch_frac = _SHIPPED_FLIGHT_S / chain_period
    return req


def _shipped_cycle():
    """A node holding the SHIPPED install — chained, rest-terminal, with a catch."""
    node = _cycle_node()
    node._catch_armed = False                     # the unified state
    resp = node._svc_plan_cycle(_shipped_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    assert resp.release_terminal is False
    assert resp.t_catch_mono > resp.t0_mono
    return node


def _anchor_mid_flight(node, tau=_SHIPPED_REPLAN_TAU_S):
    """Put the installed cycle's clock at ``tau`` as of NOW.

    The production precondition, reproduced: a tracker landing update revises
    where a ball ALREADY IN THE AIR will come down, so it arrives after the
    release and before the catch. `_hold_head` (tau = 0) is the wrong anchor for
    this shape — a splice there lands on the launch release itself and is
    refused, correctly.
    """
    plan, meta, _t0 = node._cycle
    t0 = time.perf_counter() - float(tau)
    node._cycle = (plan, meta, t0)
    node._plan_t0 = t0
    return node


def _landing_update(node, x_mm, y_mm=0.0):
    """One REAL tracker landing update, through `catch/dynamic_target`.

    Driven at the SUBSCRIBER, not at the service, because the wiring under test
    is the routing as much as the splice: `_on_dynamic_target` is what decides a
    cycle update is a replan rather than a `build_catch` reach.
    """
    _anchor_mid_flight(_refresh(node))
    node._on_dynamic_target(_dyn_msg(x=x_mm, y=y_mm))
    return node._publishers['trajectory/target_feedback'].published[-1]


def test_landing_updates_replan_the_shipped_cycle_and_are_bounded_at_two():
    """F3, on the shape the machine actually flies: two replans, then the bound.

    Until 2026-09-05 this whole path was dead on the shipped install.
    `replan_tail` refused a REST-terminal plan — *"the plan has no terminal throw
    to hold fixed"* — and the coordinator's LAUNCH + chained LANDING is
    rest-terminal by design, because that is the fix for the release-terminal
    cliff. So every tracker landing update of every cycle was refused before it
    solved, and the owner's replan policy existed only for a shape (LAUNCH →
    STEADY) that nothing installs yet.

    Two, not unbounded: an unbounded catch-side replan is a receding horizon by
    another name, which the owner ruled out (2026-08-29). Driven with REAL solves
    through the real subscriber rather than by writing the counter, because the
    thing that broke was the counter's own bookkeeping — a test that sets it is
    blind to exactly that.

    The third update must change nothing at all: same plan object, same origin.
    """
    node = _shipped_cycle()
    first_plan = node._cycle[0]

    fb = _landing_update(node, 10.0)
    assert fb.accepted is True, fb.reason
    assert fb.source == 'cycle'
    assert node._cycle_replans == 1
    second_plan = node._cycle[0]
    assert second_plan is not first_plan

    fb = _landing_update(node, 16.0)
    assert fb.accepted is True, fb.reason
    assert node._cycle_replans == 2
    third_plan = node._cycle[0]
    assert third_plan is not second_plan

    # Snapshot AFTER the harness re-anchor, not before it: `_anchor_mid_flight`
    # moves the origin itself (that is its whole job), so a snapshot taken across
    # it would measure the harness rather than the refusal.
    _anchor_mid_flight(_refresh(node))
    before_plan, _bm, before_t0 = node._cycle
    node._on_dynamic_target(_dyn_msg(x=22.0))
    fb = node._publishers['trajectory/target_feedback'].published[-1]
    assert fb.accepted is False
    assert fb.code == 'REPLAN_BUDGET'
    assert node._cycle_replans == tn._MAX_CYCLE_REPLANS
    assert node._cycle[0] is before_plan        # the last good plan still stands
    assert node._cycle[2] == pytest.approx(before_t0)


def test_a_replan_of_the_shipped_cycle_keeps_it_rest_terminal():
    """The splice must not hand back the cliff it was installed to remove.

    A tail re-solved as a STEADY window would end at a release, and the plan the
    emitter is streaming would once again command a hard stop at the throw
    (`latest_supersede_time_s`: the u1 sample at `duration - dt` reads the
    terminal HOLD, so the release stroke ships `v1 = 0`). So the shape is
    asserted on the far side of a real replan, through the response fields the
    coordinator's supersede alarm actually reads.
    """
    node = _shipped_cycle()
    fb = _landing_update(node, 10.0)
    assert fb.accepted is True, fb.reason
    _plan, meta, _t0 = node._cycle
    assert rcn.uc.is_release_terminal(meta) is False
    assert node._cycle_supersede_deadline is None
    # The spent launch release is CARRIED, not erased — a cycle that threw still
    # threw, and `_expected_landing_perf` reads that instant.
    assert [float(r.t_s) for r in meta.releases] == [
        pytest.approx(_SHIPPED_LAUNCH_S)]


def test_a_landing_update_on_a_bare_launch_is_refused_without_a_solve(
        monkeypatch):
    """No catch to re-aim ⇒ NO_CYCLE at the routing layer, before any planner.

    A bare LAUNCH carries no catch mark, so there is nothing a landing revision
    could move. Refusing it in `_replan_cycle_from_target` rather than letting
    `replan_tail` say so keeps a ~230 ms solve off trajectory_node's
    single-threaded executor to answer a question that was already answered, and
    keeps the code the coordinator's blacklist sees a SERVICE code rather than
    the planner's vocabulary.
    """
    node = _cycle_node()
    node._catch_armed = False
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    plan, meta, _t0 = node._cycle
    assert not meta.catches
    monkeypatch.setattr(
        node, '_svc_plan_cycle',
        lambda *a, **k: pytest.fail('a launch-only cycle paid for a solve'))
    node._on_dynamic_target(_dyn_msg(x=10.0))
    fb = node._publishers['trajectory/target_feedback'].published[-1]
    assert fb.accepted is False
    assert fb.code == 'NO_CYCLE'
    assert fb.source == 'cycle'
    assert node._cycle[0] is plan               # untouched
    assert node._cycle_replans == 0


def test_the_replan_budget_is_restored_by_AN_INSTALL_and_never_by_the_clock():
    """The bound is per INSTALLED WINDOW, and time alone must not lift it.

    The regression this pins: the budget used to be handed back whenever `now`
    was past the plan's release instant. That is EVERY landing update — a
    landing update revises where a ball ALREADY IN THE AIR will come down, so it
    arrives after the release by construction — so the counter reset on every
    call and the bound never bound at all.
    """
    node = _cycle_node()
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    plan, meta, _t0 = node._cycle
    node._cycle_replans = tn._MAX_CYCLE_REPLANS
    # Put the whole installed window in the PAST — the state every landing
    # update is read in. The budget must stay spent.
    past = time.perf_counter() - float(meta.duration_s) - 1.0
    node._cycle = (plan, meta, past)
    node._plan_t0 = past
    assert node._cycle_replan_budget_left() == 0
    # An INSTALL is the only thing that hands it back.
    resp, _origin = _extend_alive(node, _landing_req())
    assert resp.accepted is True, resp.message
    assert node._cycle_replans == 0
    assert node._cycle_replan_budget_left() == tn._MAX_CYCLE_REPLANS


def test_plan_cycle_refuses_a_launch_over_a_moving_machine():
    """`CycleState.at_rest` DECLARES rest; the node VERIFIES it.

    The install-continuity guard compares POSITIONS, and a position can match to
    the micron across a velocity STEP — so without this check a LAUNCH planned
    while the machine is mid-carry would install a first knot carrying zero
    velocity against a live one, and nothing downstream would notice.
    """
    node = _cycle_node()
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    plan, _meta, _t0 = node._cycle
    # Re-anchor the origin 0.3 s BEFORE NOW (not before the install) so
    # `_current_state()` samples the plan mid-launch. Anchoring off the install
    # instant would not do it: the plan+gate itself costs a few hundred ms, so the
    # sample would land past the 0.6 s window and read the terminal HOLD — zero
    # velocity, which is exactly the state the guard is supposed to admit.
    node._plan_t0 = time.perf_counter() - 0.3
    resp = _refresh(node)._svc_plan_cycle(_launch_req(), PlanCycle.Response())
    assert resp.accepted is False
    assert resp.code == feas.STALE_STATE
    assert 'REST' in resp.message
    assert node._active_plan is plan            # untouched


@pytest.mark.parametrize('mutate,code', [
    (lambda n: setattr(n, '_current_mode', 'STANDBY'), feas.WRONG_MODE),
    (lambda n: setattr(n, '_seeded', False), feas.STALE_STATE),
    (lambda n: setattr(n, '_guard_frozen', True), 'GUARD_LATCHED'),
])
def test_plan_cycle_guards_mirror_the_timed_target_ladder(mutate, code):
    """Same guards, same order as `_svc_timed_target` — a cycle is not a special
    case of the acceptance state, only of the planner."""
    node = _cycle_node()
    mutate(node)
    resp = node._svc_plan_cycle(_launch_req(), PlanCycle.Response())
    assert resp.accepted is False
    assert resp.code == code
    assert node._cycle is None


def test_plan_cycle_infeasible_carries_the_layer_that_refused():
    """A refusal names WHICH LAYER said no, through `CycleInfeasible.outcome()`.

    An unreachable throw height is refused by the cup QP's own box, and the
    outcome string round-trips `outcome_detail`'s (code, subcode) split so a
    guard can match on the bare code once the refusal starts carrying numbers.
    """
    from jugglebot.outcome_detail import base_outcome, outcome_subcode
    node = _cycle_node()
    resp = node._svc_plan_cycle(_launch_req(throw_z=1400.0),
                                PlanCycle.Response())
    assert resp.accepted is False
    assert resp.code == rcn.uc.OUTCOME_CODE
    assert base_outcome(resp.message) == 'REJECTED_CYCLE_INFEASIBLE'
    assert outcome_subcode(resp.message)          # a non-empty leading CODE
    assert node._cycle is None


# ═════════════════════════════════════════════════════════════════════════════
# The hand continuity term
# ═════════════════════════════════════════════════════════════════════════════

def test_install_continuity_rejects_a_hand_step_no_pose_check_can_see():
    """The 7th-channel half of the install guard, and the SIZE of its bound.

    A CyclePlan swap steps the SLIDER as well as the legs, and a hand
    discontinuity is invisible in every leg coordinate — the platform can be
    perfectly continuous while the hand jumps. This drives exactly that case: the
    same plan, with the commanded-hand reference moved a long way off.

    The bound is a QUARTER of the margin-discounted pump gate, the same fraction
    the leg term takes of its own step gate. It used to be the FULL pump gate
    (5.0 rev), which admitted an install discontinuity 2x the firmware's
    `MAX_DEVIATION_HAND_REV` (2.5) and 2.5x `MAX_LEAD_HAND_REV` (2.0) — a step
    the guard would E-STOP on and this gate would wave through. Both halves are
    asserted with real numbers rather than against the expression, so a change to
    either scaling has to face the guard bands it is judged against.
    """
    node = _cycle_node()
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    plan, _meta, _t0 = node._cycle
    pump_gate = node._limits.hand_vel_limit_rps * node._limits.knot_dt_s
    bound = 0.25 * feas.STEP_BOUND_MARGIN * pump_gate
    assert pump_gate == pytest.approx(5.0)      # 200 rev/s x 25 ms
    assert bound == pytest.approx(1.0)          # 0.25 x 0.80 x 5.0
    # Under BOTH firmware hand guards, which is the property that matters. The
    # two numbers are `canbridge_config.h`'s (MAX_LEAD_HAND_REV 2.0,
    # MAX_DEVIATION_HAND_REV 2.5) and are written as literals because they are
    # FIRMWARE constants that reach no generated Python header — the bench
    # runbook and `sim/unified_gate.py` quote them the same way.
    assert bound < 2.0                          # MAX_LEAD_HAND_REV
    assert bound < 2.5                          # MAX_DEVIATION_HAND_REV
    # A NEW install from rest is an EXACT match, so the bound is nowhere near
    # binding on the case the machine actually flies (measured 4.7e-10 rev).
    assert abs(float(plan.hand_at(0.0)[0])
               - float(node._commanded_hand_state()[0])) < 1e-6
    assert node._install_continuity_ok(plan, 0.0) is True
    # Supersede with a LEGACY hold: the platform is left exactly where the cycle
    # plan's first knot has it (the launch is slider-only under the z = 170 pin,
    # so the POSE term is satisfied by construction) and the hand reference falls
    # to the last shipped rev — which is what this test then moves.
    node._svc_hold(Trigger.Request(), Trigger.Response())
    node._last_hand_rev = float(plan.hand_at(0.0)[0])
    assert node._install_continuity_ok(plan, 0.0) is True
    # 1.5 rev of stale hand: REFUSED now, ADMITTED under the old 5.0 rev gate.
    node._last_hand_rev = float(plan.hand_at(0.0)[0]) + 1.5
    assert node._install_continuity_ok(plan, 0.0) is False
    # Just inside the new bound still installs — it is a bound, not a ban.
    node._last_hand_rev = float(plan.hand_at(0.0)[0]) + bound * 0.9
    assert node._install_continuity_ok(plan, 0.0) is True
    # One bound + epsilon away ⇒ refused, with the POSE unchanged. No leg
    # coordinate moved between these calls: a hand discontinuity is invisible
    # in every one of them, which is the whole reason the term exists.
    node._last_hand_rev = float(plan.hand_at(0.0)[0]) + bound * 1.01
    assert node._install_continuity_ok(plan, 0.0) is False


def test_the_commanded_hand_reference_has_three_sources_in_one_order():
    """Active plan → last shipped rev → measured encoder, and the ORDER is the
    content.

    With a cycle plan streaming, both of the other two LAG it — the encoder by the
    whole launch — so a continuity check against either would refuse a plan whose
    head is bit-identical to what is on the wire. With no hand track anywhere the
    firmware is holding the hand where it physically is, and the encoder is the
    only truth there is.
    """
    node = _cycle_node()
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    plan, _meta, t0 = node._cycle
    # (1) A streaming cycle plan wins over both stale copies.
    node._last_hand_rev = 99.0
    node._latest_hand_rev = -99.0
    node._plan_t0 = time.perf_counter() - 0.30
    rev, vel = node._commanded_hand_state()
    assert rev == pytest.approx(float(plan.hand_at(0.30)[0]), abs=0.05)
    assert abs(vel) > 1.0                       # mid-launch: the slider is moving
    # (2) A LEGACY plan carries no hand track ⇒ the last SHIPPED rev.
    node._svc_hold(Trigger.Request(), Trigger.Response())
    node._last_hand_rev = 4.25
    assert node._commanded_hand_state()[0] == pytest.approx(4.25)
    # (3) Nothing shipped ⇒ the measured encoder.
    node._last_hand_rev = None
    node._latest_hand_rev = 2.5
    assert node._commanded_hand_state()[0] == pytest.approx(2.5)
    # (4) Nothing at all ⇒ unknown, and a hand-carrying plan is REFUSED rather
    # than installed blind — that is the step MAX_DEVIATION_HAND E-STOPs on.
    node._latest_hand_rev = None
    assert node._commanded_hand_state()[0] is None
    assert node._install_continuity_ok(plan, 0.0) is False


def test_legacy_plans_skip_the_hand_term_entirely():
    """A plan with no hand track is judged exactly as it was before Phase 4."""
    node = _cycle_node()
    node._svc_hold(Trigger.Request(), Trigger.Response())
    plan = node._active_plan
    assert not hasattr(plan, 'hand_at')
    node._last_hand_rev = None
    node._latest_hand_rev = None
    assert node._install_continuity_ok(plan, 0.0) is True


class _VelocityFiction:
    """A plan wrapper whose knot-0 VELOCITY is a lie and whose POSITION is not.

    Exactly the class the position-only guard could not see: `state_at` returns
    the real pose (so the leg-position term matches to the micron) with an added
    twist. Only the two attributes the guard reads are forwarded.
    """

    def __init__(self, plan, dtwist):
        self._plan = plan
        self._dtwist = np.asarray(dtwist, dtype=float)

    def state_at(self, tau):
        pose, twist, accel = self._plan.state_at(tau)
        return pose, np.asarray(twist, dtype=float) + self._dtwist, accel

    def hand_at(self, tau):
        return self._plan.hand_at(tau)


def test_the_install_guard_charges_VELOCITY_as_well_as_position(monkeypatch):
    """A plan whose knot-0 velocity is a fiction is REFUSED, not installed.

    Until 2026-09-05 this guard compared POSITIONS only — and a position matches
    to the micron across a velocity step, which is precisely what the emitter's
    v0/v1 channels then ship to the pump. That is a CLASS, not a case: the
    settle-from-rest defect (`_cycle_start_state`, same day) was one instance of
    it, where the QP was handed an exact zero acceleration alongside a non-zero
    forward-mapped velocity. So the guard grows the rate twin of each term it
    already had, at the same 0.25 x STEP_BOUND_MARGIN fraction of the per-knot
    bound the pump implies.

    MEASURED (2026-09-05, `/tmp/probe_a3_replan.py`, session limits
    250/3000/150000): the leg bound is **2.4 rev/s** and the hand bound
    **40 rev/s**; a NEW install from rest and an EXTEND at its live plan time
    both measure **0.000000 rev/s** of leg drift, and the two REPLANs of the
    shipped install measure 0.000000 rev/s on the legs and 7.55 / 7.47 rev/s on
    the HAND — 19 % of that term's bound, and it is the solve's own duration
    against the hand's acceleration, not a discontinuity. Every path that
    installs today installs with the whole bound as margin.
    """
    node = _cycle_node()
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    plan, _meta, _t0 = node._cycle
    vel_bound = (0.25 * feas.STEP_BOUND_MARGIN
                 * hw.JB_OP_MAX_POSITION_STEP_REV / node._limits.knot_dt_s)
    assert vel_bound == pytest.approx(2.4)      # 0.25 x 0.80 x 0.3 / 0.025

    # The honest install: exact, on both rate channels.
    pose0, twist0, _ = plan.state_at(0.0)
    live_pose, live_twist, _ = node._current_state()
    assert float(np.max(np.abs(
        node._pose_twist_to_motor_rev_s(pose0, twist0)
        - node._pose_twist_to_motor_rev_s(live_pose, live_twist)))) < 1e-9
    assert node._install_continuity_ok(plan, 0.0) is True

    # Two bounds of leg velocity, with the POSE left exactly where it was.
    unit = np.zeros(6)
    unit[2] = 1.0
    per_unit = float(np.max(np.abs(
        node._pose_twist_to_motor_rev_s(pose0, unit))))
    fiction = _VelocityFiction(plan, unit * (2.0 * vel_bound / per_unit))
    assert node._install_continuity_ok(fiction, 0.0) is False
    assert 'leg velocity drift' in node._continuity_detail
    # The POSITION term is untouched by the fiction, which is the whole point.
    assert float(np.max(np.abs(
        node._pose_to_motor_rev(fiction.state_at(0.0)[0])
        - node._pose_to_motor_rev(node._current_state()[0])))) < 1e-9

    # And the SERVICE refusal an operator reads NAMES the velocity term.
    real_plan_cycle = uc.plan_cycle

    def _fictional(*a, **kw):
        p, m = real_plan_cycle(*a, **kw)
        return _VelocityFiction(p, unit * (2.0 * vel_bound / per_unit)), m

    monkeypatch.setattr(tn.uc, 'plan_cycle', _fictional)
    resp = _refresh(node)._svc_plan_cycle(_launch_req(), PlanCycle.Response())
    assert resp.accepted is False
    assert resp.code == feas.STALE_STATE
    assert 'leg velocity drift' in resp.message, resp.message


def test_the_velocity_term_passes_the_same_origin_re_installs(monkeypatch):
    """EXTEND and REPLAN re-install a bit-identical head, so Δv is zero.

    The bound is only safe if the paths that must pass do pass with margin, and
    the two same-origin chains are the ones with the most to lose: a refusal
    there holds the last good plan and the cycle stalls. Both are driven through
    real solves rather than asserted on the expression.
    """
    node = _cycle_node()
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    resp, origin = _extend_alive(node, _landing_req())
    assert resp.accepted is True, resp.message
    joined = node._cycle[0]
    tau = time.perf_counter() - origin
    pose, twist, _ = joined.state_at(tau)
    live_pose, live_twist, _ = node._current_state()
    assert float(np.max(np.abs(
        node._pose_twist_to_motor_rev_s(pose, twist)
        - node._pose_twist_to_motor_rev_s(live_pose, live_twist)))) < 1e-6

    # The REPLAN path, on the shape the machine actually flies.
    shipped = _shipped_cycle()
    fb = _landing_update(shipped, 10.0)
    assert fb.accepted is True, fb.reason
    assert shipped._continuity_detail == ''


def test_robot_state_reads_the_hand_and_survives_a_six_axis_message():
    node = _cycle_node()
    assert node._latest_hand_rev == pytest.approx(_REST_HAND_REV)
    short = _robot_state()
    short.motor_states = short.motor_states[:6]   # an older six-axis publisher
    node._latest_hand_rev = 4.0
    node._on_robot_state(short)
    assert node._latest_hand_rev == 4.0           # UNKNOWN is left alone, not zeroed


# ═════════════════════════════════════════════════════════════════════════════
# catch/dynamic_target routing
# ═════════════════════════════════════════════════════════════════════════════

def test_dynamic_target_routes_to_the_cycle_replan_not_build_catch(monkeypatch):
    """A landing update while a cycle streams is the REPLAN trigger the plan names.

    Routing it through `build_catch` instead would install a 6-channel reach over
    a 7-channel plan — dropping the hand track mid-flight with a ball in the air.
    Deliberately NOT gated on `_catch_armed`: under unified nothing arms a
    reactive catch, so the latch is down by design.
    """
    node = _cycle_node()
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    resp, _origin = _extend_alive(node, _landing_req())
    assert resp.accepted is True, resp.message
    monkeypatch.setattr(
        node, '_plan_and_install_catch',
        lambda *a, **k: pytest.fail('build_catch ran over a CyclePlan'))
    seen = []
    monkeypatch.setattr(node, '_svc_plan_cycle',
                        lambda req, resp: seen.append(req) or _refuse(resp))
    node._catch_armed = False                     # the unified state
    _refresh(node)
    msg = DynamicTargetCommand()
    msg.target_pos = Point(x=12.0, y=-4.0, z=170.0)
    msg.target_quat = Quaternion()
    msg.target_vel = Vector3()
    msg.arrival_time = time.perf_counter() + 0.5
    node._on_dynamic_target(msg)
    assert len(seen) == 1
    req = seen[0]
    assert req.mode == PlanCycle.Request.MODE_REPLAN
    # xy from the wire, z and arrival velocity CARRIED from the committed catch:
    # the message cannot say either (a cup-opening height is not derivable from a
    # platform centroid z, and the field that looks like an arrival velocity is
    # always zero on this wire).
    assert req.catch_site_mm[0] == pytest.approx(12.0)
    assert req.catch_site_mm[1] == pytest.approx(-4.0)
    assert req.catch_site_mm[2] == pytest.approx(830.0)
    assert req.catch_vel_mm_s[2] == pytest.approx(-2500.0)
    assert req.lead_s == pytest.approx(0.30)
    # The kind and the banking flag are STATED, not left at the IDL defaults
    # (KIND_LAUNCH / banking off). `replan_tail` reads neither — it inherits both
    # from the plan it is splicing — but a request that says "an unbanked launch"
    # while asking for a banked landing's tail is one the next reader has to know
    # to disbelieve, and the banking-off warning fired on every landing update
    # because of it.
    assert req.kind == PlanCycle.Request.KIND_LANDING
    assert req.banking_enabled is True
    # The coordinator's accept/reject correlation still gets an answer.
    fb = node._publishers['trajectory/target_feedback'].published[-1]
    assert fb.source == 'cycle'
    assert fb.accepted is False


def _refuse(resp):
    resp.accepted = False
    resp.code = 'NO_CYCLE'
    resp.message = 'stubbed'
    return resp


def _dyn_msg(x=0.0, y=0.0, z=170.0, lead_s=0.5):
    msg = DynamicTargetCommand()
    msg.target_pos = Point(x=float(x), y=float(y), z=float(z))
    msg.target_quat = Quaternion()
    msg.target_vel = Vector3()
    msg.arrival_time = time.perf_counter() + float(lead_s)
    return msg


def _cycle_installed_sentinel(node):
    """Mark the node as holding a cycle WITHOUT paying for a solve.

    `_on_dynamic_target` branches on `self._cycle is not None` and nothing else,
    and the two gates under test refuse strictly before the tuple is unpacked —
    so a sentinel is exactly the state being tested. Anything that DID unpack it
    would fail loudly on the strings rather than pass by accident.
    """
    node._cycle = ('plan', 'meta', 0.0)
    return node


@pytest.mark.parametrize('gate,code', [('freeze', tn._FROZEN),
                                       ('envelope', feas.WORKSPACE)])
def test_a_gated_landing_update_costs_no_solve_under_unified(monkeypatch,
                                                             gate, code):
    """The reach-freeze and the envelope are checked BEFORE `replan_tail` runs.

    Both gates used to sit under the unified early return, so a target that was
    always going to be refused still bought a full `replan_tail` — ~180 ms of
    solve on trajectory_node's single-threaded executor, which stalls ingest for
    a fifth of a second to say no. Worse, the refusal then came back wearing the
    PLANNER's vocabulary instead of `WORKSPACE` / `_FROZEN`, and
    catch_coordinator's feasibility blacklist keys off exactly those two.
    """
    node = _cycle_installed_sentinel(_cycle_node())
    monkeypatch.setattr(
        node, '_svc_plan_cycle',
        lambda *a, **k: pytest.fail('a gated target paid for a solve'))
    monkeypatch.setattr(
        node, '_replan_cycle_from_target',
        lambda *a, **k: pytest.fail('a gated target reached replan_tail'))
    node._catch_armed = False                     # the unified state
    if gate == 'freeze':
        node._catch_arrival_perf = time.perf_counter() + 0.01
    else:
        node._catch_envelope_center = np.zeros(3)
        node._catch_reach_envelope_mm = 10.0
    node._on_dynamic_target(_dyn_msg(x=500.0))
    fb = node._publishers['trajectory/target_feedback'].published[-1]
    assert fb.accepted is False
    assert fb.code == code
    assert fb.source == 'cycle'


def test_a_replan_does_not_warn_that_banking_is_off(monkeypatch):
    """The banking warning is about a DECISION, and a replan makes none.

    `replan_tail` re-solves the catch-side tail of the plan already installed and
    inherits its banking, so `banking_enabled` is inert for MODE_REPLAN. Warning
    on it fired on every tracker landing update of every cycle, which is how an
    operator learns to ignore the line that matters.
    """
    node = _cycle_node()
    warnings = []
    monkeypatch.setattr(node.get_logger(), 'warning',
                        lambda m, **kw: warnings.append(str(m)))
    req = _launch_req()
    req.mode = req.MODE_REPLAN
    req.banking_enabled = False
    # No active cycle ⇒ NO_CYCLE, but the warning is emitted BEFORE the mode
    # dispatch, so reaching the refusal is enough to have seen it.
    resp = node._svc_plan_cycle(req, PlanCycle.Response())
    assert resp.code == 'NO_CYCLE'
    assert not any('banking' in w for w in warnings), warnings
    # A NEW install with banking off still warns — the line is not deleted, only
    # kept off the path that carries no decision.
    req2 = _launch_req()
    req2.banking_enabled = False
    node._svc_plan_cycle(req2, PlanCycle.Response())
    assert any('banking' in w for w in warnings), warnings


def test_dynamic_target_uses_the_legacy_catch_path_when_no_cycle_is_active():
    """Byte-identical legacy behaviour: no cycle ⇒ the reactive reach as today."""
    from tests.ros.test_trajectory_node import _arm_catch, _dyn_target
    node = _cycle_node()
    _arm_catch(node, True)
    node._on_dynamic_target(_dyn_target(node, z=190.0, lead_s=3.0))
    assert node._active_plan is not None
    assert not isinstance(node._active_plan, CyclePlan)


def test_a_non_cycle_install_clears_the_cycle_record():
    """A hold / stop / guard descent SUPERSEDES the cycle, so the record dies.

    Without this an EXTEND would chain a window onto a release that is never
    going to happen, because the emitter stopped playing that plan minutes ago.
    """
    node = _cycle_node()
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    assert node._cycle is not None
    node._svc_hold(Trigger.Request(), Trigger.Response())
    assert node._cycle is None
    node._publish_status()
    assert node._publishers['trajectory/status'].published[-1].cycle_active is False


# ═════════════════════════════════════════════════════════════════════════════
# T-I1 — the :5557 seam, end to end
# ═════════════════════════════════════════════════════════════════════════════

def test_TI1_seven_channel_frames_reach_the_wire_and_the_flags_fall(monkeypatch):
    """T-I1: trajectory_node → real :5557 → teensy_bridge_node → loopback UDP.

    Every hop is production code: the real ``MpcCommandPub`` (ephemeral port),
    the real ``_MpcCommandSetpointSource`` decoder, the real ``SetpointPump``,
    the real v6 ``Setpoint`` encoding, and a real UDP socket. What is pinned:

    * while a ``CyclePlan`` is installed, every frame carries ``HAS_HAND`` AND
      ``HAS_V1`` — the seven-channel path is genuinely live, not merely encodable;
    * the hand lane (index 6) carries the PLAN's commanded rev, not a zero;
    * and the flags CLEAR on the falling edge when a legacy plan supersedes the
      cycle. That edge is what FW 17's hand-lane decay is written against, and a
      path that never produced it would leave the decay untested from this side.
    """
    import zmq
    from teensy_link import MsgType
    from teensy_link.protocol import Setpoint
    from jugglebot.motion.ipc import MpcCommandPub
    from jugglebot.teensy_bridge_node import _MpcCommandSetpointSource
    from teensy_link.setpoint_pump import FLAG_HAS_HAND, FLAG_HAS_V1
    from tests.ros._bridge_harness import _build_paired_node, _teardown

    pub = MpcCommandPub(addr='tcp://127.0.0.1:0')
    addr = pub._pub.getsockopt_string(zmq.LAST_ENDPOINT)
    traj = _cycle_node(command_pub_factory=lambda: pub)
    traj._pub = pub                       # emitter thread not started; wire by hand
    teensy, client, bridge = _build_paired_node()
    src = _MpcCommandSetpointSource(addr=addr)
    try:
        resp = traj._svc_plan_cycle(_launch_req(), PlanCycle.Response())
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
        # launch, and it moves in the direction the launch strokes.
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
            assert not (sp.flags & FLAG_HAS_V1), 'HAS_V1 stuck after the cycle'
            assert sp.u0[6] == pytest.approx(0.0)
    finally:
        try:
            src.close()
        except Exception:      # noqa: BLE001 — teardown must not mask a failure
            pass
        _teardown(teensy, client, bridge)
        traj.on_shutdown()
        pub.close()


# ═════════════════════════════════════════════════════════════════════════════
# T-I3 — the interlock choreography
# ═════════════════════════════════════════════════════════════════════════════

def _unified_goal(**kw):
    gh = _ContGoalHandle(**kw)
    gh.request.unified_cycle = True
    return gh


def _unified_session_node(monkeypatch, clock, *, enabled=True):
    monkeypatch.setattr(rcn.hw, 'JB_OP_UNIFIED_CYCLE_ENABLED', enabled,
                        raising=False)
    node = _ready_node(clock)
    return node


def test_TI3_the_session_verifies_the_hand_latch_and_leaves_it_alone(monkeypatch):
    """T-I3 (a): hand mastery is VERIFIED once and never switched, in either
    direction.

    The firmware refuses a `hand_source` transition whenever the setpoint output
    is armed (`hand_source.cpp:60`, `mpc_active`) and the wire is armed for the
    whole ACTIVE state (`ARMING_CONTRACT.md` § A2), so the latch is an
    OPERATOR-owned precondition and the single STREAMED call is the idempotent
    re-assert of it. Exactly ONE call, and the order against the declaration is
    still load-bearing: the assertion precedes `catch/unified_mode` going up (the
    declaration must not precede the firmware refusal it stands in for) and the
    declaration comes down at the terminal on its own.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _unified_session_node(monkeypatch, clock)
    events = []
    monkeypatch.setattr(
        node, '_set_hand_source',
        lambda streamed: (events.append(('hand_source', bool(streamed))),
                          (True, 'ok'))[1])
    pub = node._publishers['catch/unified_mode']
    monkeypatch.setattr(
        pub, 'publish',
        lambda msg: events.append(('unified_mode', bool(msg.data))))
    _stub_cycles(node, monkeypatch, clock, [TossResult(True, 'CAUGHT', 2.0, .8)])
    result = node._execute_toss_continuous(_unified_goal(num_throws=1))
    assert events == [('hand_source', True), ('unified_mode', True),
                      ('unified_mode', False)], events
    assert node._toss_unified_live is False
    assert node._toss_hand_source_streamed is False   # session-scoped, cleared
    assert result.outcome == 'COMPLETED'


def test_TI3_a_refused_hand_latch_stops_the_session_before_anything_runs(
        monkeypatch):
    """FAIL CLOSED, with its own outcome.

    With the latch still LEGACY the firmware DISCARDS every Setpoint hand channel
    — counted, but invisible to the plan — so the platform would fly a whole cycle
    with a dead hand and a seated ball. The distinct outcome matters because the
    operator's next action (settle the hand, disarm, retry) is nothing like the
    one an infeasible cycle calls for.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _unified_session_node(monkeypatch, clock)
    monkeypatch.setattr(node, '_set_hand_source',
                        lambda streamed: (False, 'hand not settled at rest'))
    monkeypatch.setattr(
        node, '_build_toss_cycle',
        lambda *a, **k: pytest.fail('a cycle was built after a refused latch'))
    result = node._execute_toss_continuous(_unified_goal(num_throws=2))
    assert result.success is False
    assert result.outcome.startswith('REJECTED_HAND_SOURCE(')
    assert 'hand not settled' in result.outcome
    assert node._goal_claimed is False


def test_TI3_the_latch_is_asserted_before_a_cycle_and_the_drain_still_runs(
        monkeypatch):
    """T-I3 (a′): WHERE the one `_set_hand_source` call sits, as an order.

    **The firmware refuses a hand_source TRANSITION whenever the setpoint output
    is armed** (`hand_source.cpp:60`, `mpc_active`), and the wire is armed for the
    whole ACTIVE state — the orchestrator arms it on ACTIVE entry and is its sole
    caller (`ARMING_CONTRACT.md` § A2). So this node cannot switch the latch from
    inside a session at all: the STREAMED call can only ever be the idempotent
    re-assert of a latch the operator set before ACTIVATE, and it is placed where
    it is so that a session which finds the latch LEGACY is refused BEFORE
    anything is armed rather than after a cycle has flown with a dead hand.

    So there is exactly ONE placement left to defend, and it is asserted as an
    order: the assertion precedes the FIRST CYCLE — nothing the session arms (the
    `arm_catch` raise, the PREPARE bundle, the plan install) happens before it.

    The terminal DRAIN is asserted here too, because deleting the hand-back is
    what made this test's second half look optional: the drain is what puts the
    staged slot back and lowers the catch latch, and it must still run on every
    way out of a unified session.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _unified_session_node(monkeypatch, clock)
    events = []
    monkeypatch.setattr(
        node, '_set_hand_source',
        lambda streamed: (events.append(('hand_source', bool(streamed))),
                          (True, 'ok'))[1])
    monkeypatch.setattr(node, '_drain_pipeline_and_disarm',
                        lambda: events.append('drain'))

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
        events.append('cycle')
        clock.t = seq.t_release + float(seq.flight_time_s) + 0.3
        _stamp(node, clock.t)
        return TossResult(True, 'CAUGHT', 2.0, 0.8), 'fsm'

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    node._execute_toss_continuous(_unified_goal(num_throws=1))
    assert events.index(('hand_source', True)) < events.index('cycle')
    assert events.index('cycle') < events.index('drain')
    assert events.count(('hand_source', True)) == 1
    assert ('hand_source', False) not in events


def test_TI3_the_teardown_never_attempts_a_hand_back(monkeypatch):
    """The session leaves the latch where the OPERATOR put it — no RPC, no verdict.

    `hand_source.cpp:60` refuses any real transition while `mpc_active` is set,
    and the setpoint output is armed for the whole ACTIVE state (its sole caller
    is the ACTIVE-state orchestrator, `ARMING_CONTRACT.md` § A2), so a session
    cannot switch the latch in either direction. A teardown hand-back could
    therefore only ever be refused — and the previous behaviour turned that
    EXPECTED refusal into `REJECTED_HAND_SOURCE(STUCK_STREAMED: …)` with
    `success=False`, i.e. every clean session ended as a rejection. An alarm that
    fires on the happy path is an alarm an operator learns to read past, and
    nothing was stuck: the latch was exactly where it was set.

    So: no call at all, and the session's own verdict is untouched.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _unified_session_node(monkeypatch, clock)
    calls = []

    def _spy(streamed):
        calls.append(bool(streamed))
        # Answer a hand-back the way the firmware would, so a re-introduced call
        # fails HERE rather than passing on a lenient stub.
        return (True, 'ok') if streamed else (False, 'ERR_REJECTED')

    monkeypatch.setattr(node, '_set_hand_source', _spy)
    infos = []
    monkeypatch.setattr(
        node, 'get_logger',
        lambda: types.SimpleNamespace(
            info=lambda msg, **kw: infos.append(str(msg)),
            warning=lambda msg, **kw: None, warn=lambda msg, **kw: None,
            error=lambda msg, **kw: None, debug=lambda msg, **kw: None))
    _stub_cycles(node, monkeypatch, clock, [TossResult(True, 'CAUGHT', 2.0, .8)])
    result = node._execute_toss_continuous(_unified_goal(num_throws=1))
    assert calls == [True], calls          # the start verification, and nothing else
    assert result.outcome == 'COMPLETED'
    assert result.success is True
    # The declaration still comes down; only the latch is left alone.
    assert node._publishers['catch/unified_mode'].published[-1].data is False
    # The latch state reaches the operator through the session's own OUTCOME
    # line — appended to the stats, never folded into the outcome code.
    outcome_lines = [ln for ln in infos if ln.startswith('TossContinuous ')]
    assert len(outcome_lines) == 1, infos
    assert 'COMPLETED' in outcome_lines[0]
    assert 'hand_source STREAMED' in outcome_lines[0]
    # ...and the terminal names the two operator routes back to LEGACY.
    disengaged = [ln for ln in infos if 'DISENGAGED' in ln]
    assert len(disengaged) == 1, infos
    assert 'REMAINS STREAMED' in disengaged[0]
    assert '/set_hand_source false' in disengaged[0]
    assert 'reboot' in disengaged[0]


def test_TI3_the_pipeline_is_forced_off_under_unified(monkeypatch):
    """Two owners of one interval is a class, not a cadence choice.

    The pipeline stages cycle N+1's whole preamble INSIDE cycle N's flight; a
    unified CyclePlan already owns that interval end to end. Forced at the single
    read so a session cannot be admitted under a pipelined dwell floor and then
    run serially.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    monkeypatch.setattr(rcn.hw, 'JB_OP_TOSS_PIPELINE_ENABLED', True,
                        raising=False)
    node = _unified_session_node(monkeypatch, clock)
    monkeypatch.setattr(node, '_set_hand_source', lambda s: (True, 'ok'))
    seen = {}
    real = rcn.TossSessionSequencer

    def _spy(*a, **kw):
        seen['pipelined'] = kw.get('pipelined')
        return real(*a, **kw)

    monkeypatch.setattr(rcn, 'TossSessionSequencer', _spy)
    _stub_cycles(node, monkeypatch, clock, [TossResult(True, 'CAUGHT', 2.0, .8)])
    node._execute_toss_continuous(_unified_goal(num_throws=1))
    assert seen['pipelined'] is False


def test_TI3_legacy_sessions_never_touch_the_hand_latch(monkeypatch):
    """T-R3's half of T-I3: with the goal field off, nothing above happens.

    The pipeline flag is stated rather than inherited (the discipline
    `test_toss_continuous_node`'s autouse fixture enforces in its own file): the
    shipped default is TRUE, and `_stub_cycles` patches the SERIAL runner, so an
    inherited default would take the pipelined branch and fail for a harness
    reason. The unified tests above do not need it — unified forces the pipeline
    off, which is itself part of what they assert.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    monkeypatch.setattr(rcn.hw, 'JB_OP_TOSS_PIPELINE_ENABLED', False,
                        raising=False)
    node = _unified_session_node(monkeypatch, clock)
    monkeypatch.setattr(
        node, '_set_hand_source',
        lambda s: pytest.fail('a legacy session touched the hand_source latch'))
    pub = node._publishers['catch/unified_mode']
    monkeypatch.setattr(
        pub, 'publish',
        lambda msg: pytest.fail('a legacy session declared unified_mode'))
    _stub_cycles(node, monkeypatch, clock, [TossResult(True, 'CAUGHT', 2.0, .8)])
    result = node._execute_toss_continuous(_ContGoalHandle(num_throws=1))
    assert result.outcome == 'COMPLETED'
    assert node._toss_unified_live is False


def test_TI3_the_goal_field_alone_does_not_engage_unified(monkeypatch):
    """TWO keys, and the build-time one is read fail-closed.

    A goal that asks for unified on a build that has not enabled it runs the
    LEGACY path — silently and safely — rather than half-engaging.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _unified_session_node(monkeypatch, clock, enabled=False)
    monkeypatch.setattr(
        node, '_set_hand_source',
        lambda s: pytest.fail('unified engaged with the build flag off'))
    _stub_cycles(node, monkeypatch, clock, [TossResult(True, 'CAUGHT', 2.0, .8)])
    node._execute_toss_continuous(_unified_goal(num_throws=1))
    assert node._toss_unified_live is False


class _CountingHw:
    """A ``hw`` stand-in that COUNTS reads of the unified key, forwards the rest.

    The single-read claim is about how many times the machine ASKS, which no
    amount of reading the source proves — a `getattr` inside a helper called
    twice is one occurrence of the text and two reads. So the module is swapped
    for a proxy and the reads are counted.
    """

    def __init__(self, real, value):
        self.__dict__['_real'] = real
        self.__dict__['_value'] = value
        self.__dict__['reads'] = 0

    def __getattr__(self, name):
        if name == 'JB_OP_UNIFIED_CYCLE_ENABLED':
            self.__dict__['reads'] += 1
            return self.__dict__['_value']
        return getattr(self.__dict__['_real'], name)


def test_the_session_asks_the_unified_key_exactly_once_per_goal(monkeypatch):
    """ONE read per goal, and the answer is then handed down.

    Re-resolving at a branch would let a mid-session config change put the legacy
    stroke engine and the streamed plan on one axis — the dual-mastery class the
    firmware's `hand_source` latch exists to make structurally impossible. A
    second read is also a second DECISION, free to disagree with the first about
    a session that is already flying.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)               # built before the proxy: __init__
    counting = _CountingHw(rcn.hw, True)    # reads plenty of other keys
    monkeypatch.setattr(rcn, 'hw', counting)
    monkeypatch.setattr(node, '_set_hand_source', lambda s: (True, 'ok'))
    _stub_cycles(node, monkeypatch, clock,
                 [TossResult(True, 'CAUGHT', 2.0, .8)] * 2)
    result = node._execute_toss_continuous(_unified_goal(num_throws=2))
    assert result.outcome == 'COMPLETED'
    assert counting.reads == 1, (
        'the session asked %d times — every extra read is a decision that can '
        'disagree with the one the session is already flying' % counting.reads)


def test_a_single_toss_never_asks_the_unified_key_at_all(monkeypatch):
    """The single `Toss` has no session to scope a hand-mastery latch to.

    A per-goal mastery flip is not something this action offers, so the key must
    not reach it — a read here would be a `Toss` that quietly took the streamed
    branch with nothing to hand the latch back.
    """
    from tests.ros.test_toss_coordinator import _TossGoalHandle
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _ready_node(clock)
    counting = _CountingHw(rcn.hw, True)
    monkeypatch.setattr(rcn, 'hw', counting)
    monkeypatch.setattr(
        node, '_run_toss_cycle',
        lambda seq, **kw: (TossResult(True, 'CAUGHT', 2.0, 0.8), 'fsm'))
    node._execute_toss(_TossGoalHandle())
    assert counting.reads == 0


# ── the three action seams ────────────────────────────────────────────────────

def _seq_state(node, unified):
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=0.8, throw_delay_s=5.0, unified=unified)
    seq.start(time.perf_counter())
    seq._prepare_dispatched = True
    state = node._toss_committed
    node._toss_unified_live = unified
    return seq, state


def _drive(node, seq, state, action):
    """Run ONE `_step_toss_sequence` tick whose decision carries ``action``."""
    now = time.perf_counter()
    decision = TossDecision('PREPARING', action, False, None)
    seq.step = lambda _now, _obs, _d=decision: _d
    node._step_toss_sequence(seq, now, state=state,
                             obs=node._build_toss_observations(now, state))
    return decision


@pytest.mark.parametrize('unified', [False, True])
def test_the_dispatch_seam_never_issues_a_legacy_hand_rpc_under_unified(
        monkeypatch, unified):
    """`_dispatch_toss_throw` is the ONE kind-0 write and it must not run.

    The firmware would refuse it anyway (ERR_HAND_SOURCE while STREAMED), which
    is exactly why it must not be attempted: the ack path would log an arm, the
    FSM would advance on a dispatch that never reached the motor, and the failure
    would surface as a ball that never left.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified)
    calls = []
    monkeypatch.setattr(node, '_dispatch_toss_throw',
                        lambda s, st=None: calls.append(1) or ('ok', 'legacy'))
    outcome, message = node._dispatch_toss(seq, state, unified)
    assert outcome == 'ok'
    if unified:
        assert calls == []
        assert 'no hand RPC issued' in message
        assert state.throw_dispatched is True   # release evidence still armed
    else:
        assert calls == [1]


@pytest.mark.parametrize('unified', [False, True])
def test_the_reach_seam_publishes_nothing_under_unified(monkeypatch, unified):
    """The deferred A→B reach is a 6-channel target; the plan already has both.

    Publishing it would install a reach over the running CyclePlan and drop the
    hand track with the ball in the air.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified)
    calls = []
    monkeypatch.setattr(node, '_publish_toss_reach',
                        lambda st=None: calls.append(1))
    monkeypatch.setattr(node, '_tick_unified_extend', lambda *a, **k: None)
    _drive(node, seq, state, ACTION_REACH_CATCH)
    assert calls == ([] if unified else [1])


def test_the_announce_seam_defers_under_unified(monkeypatch):
    """The announcement waits for the plan that carries the release.

    Under unified the throw is not committed until a plan containing it is
    installed, and the announcement's `throw_time` must be the instant the ball
    ACTUALLY leaves — which only the installed plan knows. So the FSM's ANNOUNCE
    tick arms a deferral instead of publishing, and the FSM stays in PREPARING
    (its own release-window guard still live) until the plan lands.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    monkeypatch.setattr(
        node, '_announce_toss',
        lambda *a, **k: pytest.fail('the legacy announcement fired under unified'))
    monkeypatch.setattr(node, '_tick_unified_launch', lambda *a, **k: None)
    _drive(node, seq, state, ACTION_ANNOUNCE)
    assert state.unified_launch_pending is True
    assert node._publishers['throw_announcements'].published == []


def test_the_launch_tick_waits_for_the_release_lead(monkeypatch):
    """Planning early would put the release seconds before the FSM's schedule.

    The FSM's landing instant is what the ball sensor's arrival window is cut
    from, so a release that lands in the wrong second makes every catch read as a
    miss. The trigger is therefore `t_release - now <= window + budget + a loop
    period`, and nothing is planned before it.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    state.unified_launch_pending = True
    calls = []
    monkeypatch.setattr(node, '_call_plan_cycle',
                        lambda req: calls.append(req) or None)
    # 5 s of lead: far too early.
    node._tick_unified_launch(seq, state, seq.t_release - 5.0)
    assert calls == []
    assert state.unified_launch_pending is True
    # Inside the lead: it plans (and here, fails to reach the service).
    node._tick_unified_launch(seq, state,
                              seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert len(calls) == 1
    assert calls[0].mode == PlanCycle.Request.MODE_NEW
    assert calls[0].kind == PlanCycle.Request.KIND_LAUNCH
    assert calls[0].period_s == pytest.approx(rcn._UNIFIED_LAUNCH_WINDOW_S)
    assert calls[0].banking_enabled is True
    assert calls[0].throw_site_mm[2] == pytest.approx(rcn._UNIFIED_THROW_CUP_Z_MM)
    assert calls[0].catch_site_mm[2] == pytest.approx(rcn._UNIFIED_CATCH_CUP_Z_MM)
    # A service failure is a NAMED refusal, not a silent one.
    assert state.unified_reject.startswith('REJECTED_PLAN_SERVICE(')


def test_a_refused_plan_relabels_the_cycle_outcome(monkeypatch):
    """The FSM's ladder is right; only the NAME is wrong.

    A plan refused at the launch point leaves the FSM to terminalise through its
    own release-window guard — nothing armed, nothing flew, the safing ran — but
    ABORTED_CANT_MAKE_RELEASE says "ran out of lead" when what happened is "the
    planner refused, and here is which layer".
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _unified_session_node(monkeypatch, clock)
    monkeypatch.setattr(node, '_set_hand_source', lambda s: (True, 'ok'))
    refusal = 'REJECTED_CYCLE_INFEASIBLE(CATCH_RUNWAY: no runway below 0.83 m)'

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
        state.unified_reject = refusal
        clock.t = seq.t_release + float(seq.flight_time_s) + 0.3
        _stamp(node, clock.t)
        return TossResult(False, 'ABORTED_CANT_MAKE_RELEASE'), 'fsm'

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    result = node._execute_toss_continuous(_unified_goal(num_throws=2))
    assert list(result.per_cycle_outcomes) == [refusal]


def test_the_plan_outcome_families_are_kept_apart():
    """A planner refusal and a service refusal are different findings.

    `REJECTED_CYCLE_INFEASIBLE(...)` passes through verbatim (re-wrapping would
    nest the parentheses and break `outcome_subcode`); the node's own acceptance
    codes get `REJECTED_CYCLE_PLAN(<code>: ...)`, because calling a wrong-mode
    node "infeasible" sends an operator looking for a cycle the machine cannot
    fly instead of a node in the wrong mode.
    """
    from jugglebot.outcome_detail import base_outcome, outcome_subcode
    composed = 'REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: 186215 > 150000 mm/s^3)'
    assert rcn.ReloadCoordinatorNode._unified_plan_outcome(
        'REJECTED_CYCLE_INFEASIBLE', composed) == composed
    node_side = rcn.ReloadCoordinatorNode._unified_plan_outcome(
        'WRONG_MODE', 'plan_cycle requires TRAJECTORY mode')
    assert base_outcome(node_side) == 'REJECTED_CYCLE_PLAN'
    assert outcome_subcode(node_side) == 'WRONG_MODE'


# ── the survived MISS ─────────────────────────────────────────────────────────

def test_the_cycle_comes_to_rest_INSIDE_the_hand_park_band():
    """The settle site is the PARK, and the next cycle's CHECKING gate says so.

    `toss_sequencer` refuses a cycle REJECTED_HAND_NOT_PARKED unless the hand is
    inside `HAND_PARK_BAND_REV` (0.5 rev) of retract. Settling where the cup
    CAUGHT — the LANDING's IDL default, and what the coordinator asked for until
    2026-09-05 — leaves it at 4.755 rev, 9.5x the band, so every cycle from the
    second on was refused before it planned. Asserted on the PLAN rather than on
    the request, because the request is only a wish until the QP agrees.

    Not inside the FIRMWARE's ±0.10 rev `hand_source` settle band, and that is
    structural rather than a shortfall: the true park (0.0 rev, cup 679.6 mm)
    sits 10 mm BELOW this planner's own cup box and is refused SETTLE_SITE before
    it plans (MEASURED 2026-09-05). Nothing depends on reaching it — the latch
    switch is refused while the wire is armed regardless, so it is never
    attempted from inside a session.
    """
    from jugglebot.motion.trajectory import hand_stroke
    node = _cycle_node()
    req = _launch_req()
    req.settle_site_mm = [0.0, 0.0, rcn.uc.SETTLE_CUP_Z_MM]
    req.chain = True
    req.chain_kind = PlanCycle.Request.KIND_LANDING
    req.chain_period_s = 0.6 + 0.6
    req.chain_catch_frac = 0.5
    resp = node._svc_plan_cycle(req, PlanCycle.Response())
    assert resp.accepted is True, resp.message
    plan, _meta, _t0 = node._cycle
    rest_rev = float(plan.hand_rev[-1])
    assert abs(rest_rev) <= hand_stroke.HAND_PARK_BAND_REV, (
        'the cycle rests at %.3f rev — outside the %.2f rev park band, so the '
        'NEXT cycle is refused REJECTED_HAND_NOT_PARKED'
        % (rest_rev, hand_stroke.HAND_PARK_BAND_REV))
    # And it is genuinely low: the catch height would have left it 9.5x out.
    assert rest_rev == pytest.approx(
        rcn.uc.hand_rev_for_cup_z(rcn.uc.SETTLE_CUP_Z_MM), abs=1e-3)
    assert _hand_rev_for_cup_z(rcn._UNIFIED_CATCH_CUP_Z_MM) > (
        9.0 * hand_stroke.HAND_PARK_BAND_REV)


def test_the_coordinators_request_settles_at_the_park_over_the_catch_xy():
    """The rule, at the ONE seam that builds a `PlanCycle.Request`.

    xy from the catch (the cup stops over the seat the ball is already in) and z
    from `unified_cycle.SETTLE_CUP_Z_MM` (the park, clamped into the planner's
    box) — never a literal here, so the two cannot drift apart.
    """
    node = _ready_node(_Clock())
    req = node._unified_cycle_request(
        PlanCycle.Request.MODE_NEW, PlanCycle.Request.KIND_LAUNCH,
        period_s=0.6, throw_xy_mm=(10.0, -20.0), catch_xy_mm=(30.0, 40.0),
        flight_s=0.8, catch_frac=0.0, catch_vel_mm_s=(0.0, 0.0, -3900.0))
    assert req.settle_site_mm[0] == pytest.approx(30.0)
    assert req.settle_site_mm[1] == pytest.approx(40.0)
    assert req.settle_site_mm[2] == pytest.approx(rcn.uc.SETTLE_CUP_Z_MM)
    # It is NOT the catch height — the bug this closes.
    assert req.settle_site_mm[2] != pytest.approx(req.catch_site_mm[2])


def test_a_survived_miss_holds_the_pose_and_never_goes_home(monkeypatch):
    """Owner directive (2026-08-28), at its single enforcement point.

    Three of `_safe_abort`'s four rungs survive and one does not. The RETRACT is
    dropped too, and for a reason worth stating: it is a kind-3 smooth move, and
    under a STREAMED hand_source the firmware refuses every legacy hand command —
    and the hand is not parked at the top of a stroke anyway, because the plan
    brought it wherever the window ended. The plan IS the retract.
    """
    node = _ready_node(_Clock())
    node._toss_unified_live = True
    calls = []
    for name in ('_safe_abort', '_go_home', '_retract_hand_with_retries'):
        monkeypatch.setattr(node, name,
                            (lambda _n: lambda *a, **k: calls.append(_n))(name))
    monkeypatch.setattr(node, '_arm_catch', lambda a: calls.append('arm_catch'))
    monkeypatch.setattr(node, '_drain_pipeline_and_disarm', lambda: None)
    monkeypatch.setattr(rcn, '_UNIFIED_MISS_SETTLE_S', 0.02)
    node._toss_safe_abort()
    assert calls == ['arm_catch']
    assert '_go_home' not in calls
    assert '_safe_abort' not in calls
    assert '_retract_hand_with_retries' not in calls


def test_the_session_warms_the_planner_before_its_first_cycle(monkeypatch):
    """The cold solve is paid at session start, where nothing is armed.

    Measured 2026-09-04: 3267 ms for the first LAUNCH+LANDING install in a
    process against a 424 ms warm median. The launch trigger fires
    `window + budget` before the FSM's release, so a cold solve THERE lands the
    release ~2.1 s late — past the 0.5 s grace — and cycle 1 of every session
    aborts ABORTED_NO_RELEASE with the ball in the air.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _unified_session_node(monkeypatch, clock)
    order = []
    monkeypatch.setattr(node, '_set_hand_source',
                        lambda s: order.append('hand_source') or (True, 'ok'))
    monkeypatch.setattr(node, '_unified_warm_planner',
                        lambda: order.append('warm') or 0.4)
    _stub_cycles(node, monkeypatch, clock, [TossResult(True, 'CAUGHT', 2.0, .8)])
    node._execute_toss_continuous(_unified_goal(num_throws=1))
    # Warmed once, after the hand latch and before anything else.
    assert order[:2] == ['hand_source', 'warm']
    assert order.count('warm') == 1
    # And it really does SOLVE — a warm-up that returned without planning would
    # leave the cold cost exactly where it was, silently. Spied on the planner
    # itself rather than timed, because this module's clock is a fake here.
    real = rcn.ReloadCoordinatorNode()
    solved = []
    orig = rcn.uc.plan_launch
    monkeypatch.setattr(rcn.uc, 'plan_launch',
                        lambda *a, **k: solved.append(1) or orig(*a, **k))
    real._unified_warm_planner()
    assert solved == [1]


def test_a_warm_up_failure_never_costs_the_session(monkeypatch):
    """A warm-up that cannot run must degrade, not abort.

    The cost of a failed warm-up is that the first cycle pays the cold solve —
    which is exactly the pre-warm-up behaviour — so it is a WARN and the session
    continues.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _unified_session_node(monkeypatch, clock)
    monkeypatch.setattr(node, '_set_hand_source', lambda s: (True, 'ok'))
    monkeypatch.setattr(rcn.uc, 'plan_launch',
                        lambda *a, **k: (_ for _ in ()).throw(RuntimeError('boom')))
    _stub_cycles(node, monkeypatch, clock, [TossResult(True, 'CAUGHT', 2.0, .8)])
    result = node._execute_toss_continuous(_unified_goal(num_throws=1))
    assert result.outcome == 'COMPLETED'


def test_the_unified_miss_settle_is_derived_not_chosen():
    """Both terms, and neither is a magic number.

    Machine term: the legacy cleanup MINUS the go_home profile — the recentre is
    exactly the part that does not happen. Ball term: free fall from the catch cup
    height plus a geometric bounce series at e = 0.5. Legacy's own constant is
    UNCHANGED, so a legacy session's cadence accounting is untouched.
    """
    from jugglebot.toss_session import (DEFAULT_SESSION_MISS_CLEANUP_S,
                                        GO_HOME_DURATION_S)
    assert rcn._UNIFIED_MISS_SETTLE_S == pytest.approx(
        DEFAULT_SESSION_MISS_CLEANUP_S - GO_HOME_DURATION_S
        + rcn._UNIFIED_BALL_SETTLE_S)
    # The ball term IS the physics: t = sqrt(2h/g) . (1 + 2e/(1-e)) at e = 0.5.
    h_m = rcn._UNIFIED_CATCH_CUP_Z_MM / 1000.0
    t_fall = (2.0 * h_m / (hw.GRAVITY_MMPS2 / 1000.0)) ** 0.5
    assert rcn._UNIFIED_BALL_SETTLE_S == pytest.approx(3.0 * t_fall, abs=0.02)
    # And legacy is untouched.
    assert DEFAULT_SESSION_MISS_CLEANUP_S == pytest.approx(
        rcn.DEFAULT_SESSION_MISS_CLEANUP_S)


@pytest.mark.parametrize('settle,expect_wait', [(True, True), (False, False)])
def test_the_ball_settle_wait_is_charged_only_where_a_cycle_can_spend_it(
        monkeypatch, settle, expect_wait):
    """2.03 s of blocking buys the NEXT cycle a quiet floor — or buys nothing.

    `_toss_safe_abort` serves both the FSM's own SAFE_ABORT terminal (the
    survived MISS, after which the session runs another cycle over whatever the
    dropped ball is doing) and `_safe_toss_on_early_exit` (an honoured cancel, the
    session timeout, rclpy shutting down). On the second family the wait is 2.03 s
    of a thread already being torn down — and on the cancel path specifically it
    is 2.03 s the operator waits after asking the machine to STOP, because the
    goal cannot be reported cancelled until this returns.

    The retained SAFING rungs are unconditional in both cases: they are what makes
    the machine safe, and only the ball wait is about the ball.
    """
    node = _ready_node(_Clock())
    node._toss_unified_live = True
    calls = []
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda v: calls.append(('catch_armed', v)))
    monkeypatch.setattr(node, '_arm_catch',
                        lambda v: calls.append(('arm_catch', v)) or True)
    slept = []
    # The wait itself is shortened so the test does not spend 2.03 s proving it
    # exists — the QUESTION is which path charges it, not how long it is (that is
    # `test_the_unified_miss_settle_is_derived_not_chosen`'s).
    monkeypatch.setattr(rcn, '_UNIFIED_MISS_SETTLE_S', 0.02)
    monkeypatch.setattr(rcn.time, 'sleep', lambda s: slept.append(s))
    node._unified_hold_after_abort(settle)
    # The safing runs either way, in its order.
    assert calls == [('catch_armed', False), ('arm_catch', False)]
    assert bool(slept) is expect_wait


def test_an_early_exit_never_charges_the_settle_wait(monkeypatch):
    """The wiring half: which call site passes which value.

    Asserted through `_toss_safe_abort` rather than on the flag, because the
    thing that regressed is which LADDER charges the wait, and the ladders are
    what the six teardown paths reach.
    """
    node = _ready_node(_Clock())
    node._toss_unified_live = True
    seen = []
    monkeypatch.setattr(node, '_drain_pipeline_and_disarm', lambda: None)
    monkeypatch.setattr(node, '_release_toss_holds', lambda st: None)
    monkeypatch.setattr(node, '_unified_hold_after_abort',
                        lambda settle: seen.append(settle))
    # The FSM's own SAFE_ABORT terminal: another cycle may follow, so it waits.
    node._toss_safe_abort(node._toss_committed)
    # cancel / timeout / shutdown: the session is ending, so it does not.
    seq = types.SimpleNamespace(prepared=True)
    node._safe_toss_on_early_exit(seq, node._toss_committed)
    assert seen == [True, False]


def test_the_unified_mode_declaration_is_LATCHED_on_both_ends():
    """`catch/unified_mode` goes out exactly TWICE per session, at its edges.

    A catch_coordinator that starts or restarts between those two publishes — a
    crash-restart, a bench `ros2 run`, a late composition — never sees a VOLATILE
    True and goes on arming legacy hand strokes into a STREAMED `hand_source` for
    the rest of the session, which the firmware refuses with the ball already in
    the air. TRANSIENT_LOCAL hands a late subscriber the standing declaration on
    connect.

    Both ends, because durability must MATCH: a volatile subscription receives
    nothing from a transient-local publisher's history, which would leave exactly
    the gap this closes.
    """
    from rclpy.qos import DurabilityPolicy
    node = _ready_node(_Clock())
    pub = node._publishers['catch/unified_mode']
    assert pub.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert pub.qos.depth == 1
    ccn = CatchCoordinatorNode()
    sub = ccn._subscriptions['catch/unified_mode']
    assert sub.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert sub.qos.depth == 1


def test_a_legacy_miss_still_runs_the_full_safe_abort_ladder(monkeypatch):
    node = _ready_node(_Clock())
    node._toss_unified_live = False
    calls = []
    monkeypatch.setattr(node, '_safe_abort', lambda: calls.append('safe_abort'))
    monkeypatch.setattr(node, '_drain_pipeline_and_disarm', lambda: None)
    node._toss_safe_abort()
    assert calls == ['safe_abort']


# ═════════════════════════════════════════════════════════════════════════════
# catch_coordinator_node — the reactive arm is off
# ═════════════════════════════════════════════════════════════════════════════

def test_unified_mode_withholds_the_reactive_hand_arm(monkeypatch):
    """Belt AND braces: the call site skips it and the dispatch refuses it.

    The dispatch-side gate is the enforcement point a future third caller cannot
    bypass; the call-site gate keeps the per-ball bookkeeping (the one-shot latch,
    the dispatch counter) from running for an arm that will never be attempted.
    """
    from tests.ros.test_catch_coordinator_node import _balls_msg, _catchable_cmd
    ccn = CatchCoordinatorNode()
    ccn._catch_armed = True
    dispatched = []
    monkeypatch.setattr(ccn._hand_traj_client, 'call_async',
                        lambda req: dispatched.append(req) or _DoneFuture())
    monkeypatch.setattr(ccn._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None:
                        _catchable_cmd())
    ccn._on_unified_mode(Bool(data=True))
    assert ccn._unified_mode is True
    ccn._on_balls(_balls_msg())
    assert dispatched == []
    # Directly, too — the enforcement point, not just the call site.
    assert ccn._arm_hand_catch(0.5, 2.0) is False
    assert dispatched == []
    # A DEFERRAL, never a drop: the one-shot latch is left open, so the next
    # balls tick after the session ends arms normally.
    assert ccn._hand_traj_armed_for_ball is None
    ccn._on_unified_mode(Bool(data=False))
    ccn._on_balls(_balls_msg())
    assert len(dispatched) == 1


def test_unified_mode_leaves_the_stroke_busy_window_inert(monkeypatch):
    """There is no firmware stroke to be busy with.

    `_latch_throw_stroke_window` sizes a suppression from `hand_stroke`'s model of
    the legacy stroke engine's deceleration. Under unified that engine is not
    running on this axis, and the plan's own twins are facts about the trajectory
    instead. Leaving the window None is the same INERT state a BallButler
    announcement produces.
    """
    ccn = CatchCoordinatorNode()
    ccn._on_unified_mode(Bool(data=True))
    msg = types.SimpleNamespace(
        thrower_name=ccn._coordinator.robot_name,
        throw_time=types.SimpleNamespace(sec=100, nanosec=0),
        initial_velocity=types.SimpleNamespace(x=0.0, y=0.0, z=3900.0))
    ccn._latch_throw_stroke_window(msg)
    assert ccn._throw_stroke_clear_ros is None
    # Legacy: the window latches exactly as today.
    ccn._on_unified_mode(Bool(data=False))
    ccn._latch_throw_stroke_window(msg)
    assert ccn._throw_stroke_clear_ros is not None


def test_unified_mode_still_consumes_announcements():
    """Only the ARM is withheld — the announcement itself is consumed in full.

    It drives the tracker's correlation, the possession latch and the open-loop
    pre-tilt, none of which command the hand; dropping it would break the catch
    this mode exists to make. Asserted on the SIDE EFFECTS rather than on the
    handler's source text, and against the legacy run as the reference, so the
    claim is "identical, except the stroke window" rather than "the string does
    not appear".
    """
    from tests.ros.test_catch_coordinator_node import _announcement

    def _consume(unified):
        ccn = CatchCoordinatorNode()
        ccn._on_catch_armed(Bool(data=True))
        ccn._on_unified_mode(Bool(data=unified))
        n0 = len(ccn._dyn_target_pub.published)
        ccn._on_throw_announcement(_announcement())
        return {
            'seen': ccn._announcement_seen,
            'landing': ccn._announced_landing_time,
            'pretilt': ccn._pretilt_cmd is not None,
            'targets': len(ccn._dyn_target_pub.published) - n0,
            'stroke_window': ccn._throw_stroke_clear_ros,
        }

    legacy, unified = _consume(False), _consume(True)
    assert unified['seen'] is True
    assert unified['targets'] == 1
    for key in ('seen', 'landing', 'pretilt', 'targets'):
        assert unified[key] == legacy[key], key
    # The ONE deliberate difference, pinned so the equality above cannot quietly
    # start covering it: there is no firmware stroke to be busy with.
    assert unified['stroke_window'] is None


class _DoneFuture:
    def add_done_callback(self, cb):
        pass


# ═════════════════════════════════════════════════════════════════════════════
# Wire defaults + the envelope carve-out
# ═════════════════════════════════════════════════════════════════════════════

def test_unified_cycle_wire_default_is_false():
    """The IDL default is LOAD-BEARING and the mock must not diverge from it.

    An omitted field must never put the hand on the 40 Hz stream: under unified
    the can-bridge latch is flipped to STREAMED for the whole session, and a goal
    that took that branch by accident would find its reactive catch arm silently
    refused mid-flight with a ball in the air.
    """
    from jugglebot_interfaces.action import TossContinuous
    action = (Path(rcn.__file__).parents[2] / 'jugglebot_interfaces'
              / 'action' / 'TossContinuous.action')
    if action.exists():
        goal_block = action.read_text().split('\n---\n')[0]
        assert 'bool unified_cycle false' in goal_block
    assert TossContinuous.Goal().unified_cycle is False


def test_the_arm_window_bound_is_the_only_envelope_carve_out():
    """Under unified only C-HAND-3's ARM_WINDOW bound is dropped.

    It is the one bound of the seven that models the REACTIVE catch — whether a
    kind-1 stroke can still be dispatched after the kind-0 throw stroke has
    decelerated. The other six describe the hand's METAL and its motor, which the
    unified path drives just as hard, so they still gate.
    """
    from jugglebot.motion.trajectory import throw_envelope as te
    # A flight short enough that the arm window closes but the metal is fine.
    t = te.MIN_FLIGHT_TIME_S - 0.05
    v = te.vertical_release_speed_mps(t)
    assert te.evaluate(t, v).ok is False
    assert te.evaluate(t, v).bound == 'ARM_WINDOW'
    assert te.evaluate(t, v, arm_window=False).ok is True
    # A speed that breaks metal is still refused with the carve-out applied.
    fast = te.evaluate(te.MAX_FLIGHT_TIME_S, 9.0, arm_window=False)
    assert fast.ok is False
    assert fast.bound != 'ARM_WINDOW'


def test_the_arm_window_carve_out_reaches_ALL_THREE_of_its_call_sites():
    """Bound 7 is dropped everywhere under unified, or layer 3 goes inert.

    `throw_envelope.evaluate(..., arm_window=)` is consulted in three places and
    they have to agree, because they are three views of ONE question — *may this
    release speed be commanded?*:

    1. `toss_sequencer`'s CHECKING gate — is this goal flyable at all;
    2. `toss_session.floor_event_vel_mps` — the SLOWEST release the ILC speed
       trim could ask for, which every cadence floor is computed against;
    3. `reload_coordinator._ilc_vel_trim_refusal` — the APPLY seam that admits or
       drops the trim.

    Bound 7 refuses the whole SHORT half of the flight band, and it refuses the
    NEGATIVE side of the trim first: at the band floor T = 0.4949 s the
    admissible negative headroom is exactly +0.000 m/s. So charging it at (2) and
    (3) under unified — where there is no stroke engine on that axis to arm —
    does not make layer 3 conservative, it makes it INERT across exactly the half
    of the band the carve-out exists to unlock.

    Legacy keeps the bound at all three, and that half is pinned too.
    """
    from jugglebot.motion.trajectory import throw_envelope as te
    from jugglebot.toss_session import ILC_SPEED_AUTHORITY, TossSessionSequencer
    # A flight where bound 7 binds and nothing else does — the carve-out's own
    # territory, and where a disagreement between the three is visible.
    flight = te.MIN_FLIGHT_TIME_S + 0.005
    nominal = te.vertical_release_speed_mps(flight)
    slow = nominal * (1.0 - ILC_SPEED_AUTHORITY)
    assert te.evaluate(flight, slow).bound == 'ARM_WINDOW'
    assert te.evaluate(flight, slow, arm_window=False).ok is True

    # (2) the session's floor. Legacy cannot reach the slow end; unified can.
    def _floor(unified):
        return TossSessionSequencer(
            num_throws=1, flight_time_s=flight,
            ilc_speed_trim_possible=True, unified=unified).floor_event_vel_mps

    assert _floor(False) > slow * 1.001, (
        'legacy already reached the trim floor, so this flight does not '
        'exercise the carve-out')
    assert _floor(True) == pytest.approx(slow)

    # (3) the apply seam. Same trim, opposite verdicts.
    refusal = rcn.ReloadCoordinatorNode._ilc_vel_trim_refusal
    assert 'ARM_WINDOW' in refusal(nominal, slow, flight)
    assert refusal(nominal, slow, flight, arm_window=False) == ''
    # A speed that breaks METAL is still refused with the carve-out applied —
    # the other six bounds describe the hand's motor, which unified drives just
    # as hard.
    assert refusal(nominal, 9.0, te.MAX_FLIGHT_TIME_S, arm_window=False) != ''


def test_the_unified_flag_reaches_the_session_FSM_as_well_as_the_cycle_FSM(
        monkeypatch):
    """One resolution, handed to BOTH sequencers — never re-read at either.

    The session's floors and the cycle's CHECKING gate ask the same envelope
    question; if only one of them were told, a session would admit a cadence its
    own cycles refuse (or the reverse) for a reason no log line names.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _unified_session_node(monkeypatch, clock)
    monkeypatch.setattr(node, '_set_hand_source', lambda s: (True, 'ok'))
    seen = {}
    real = rcn.TossSessionSequencer
    monkeypatch.setattr(rcn, 'TossSessionSequencer',
                        lambda *a, **kw: (seen.update(kw), real(*a, **kw))[1])
    _stub_cycles(node, monkeypatch, clock, [TossResult(True, 'CAUGHT', 2.0, .8)])
    node._execute_toss_continuous(_unified_goal(num_throws=1))
    assert seen['unified'] is True
    # The cycle FSM's half of the same resolution is pinned by
    # `test_the_sequencer_hands_the_carve_out_down_rather_than_re_reading_it`.


@pytest.mark.parametrize('unified', [False, True])
def test_the_sequencer_is_TOLD_the_carve_out_and_never_re_reads_it(monkeypatch,
                                                                   unified):
    """`unified` reaches the cycle FSM as a FIELD, carrying the session's value.

    Spied on the CONSTRUCTOR rather than read out of the source: what matters is
    the value the FSM is built with, and a kwarg that is present in the text but
    passes the wrong thing looks identical to a reader.

    The FSM must also never ask the config key itself. A second read there would
    be a second decision, free to disagree with the session's — and it would
    disagree exactly when a config reload lands mid-session, which is the
    dual-mastery case the `hand_source` latch exists to make impossible.
    """
    import jugglebot.toss_sequencer as ts
    node = _ready_node(_Clock())
    node._toss_unified_live = unified
    seen = {}
    real = rcn.TossSequencer
    monkeypatch.setattr(rcn, 'TossSequencer',
                        lambda *a, **kw: (seen.update(kw), real(*a, **kw))[1])
    seq, _state = node._build_toss_cycle(np.array([0.0, 0.0, 170.0]), 0.8, 5.0,
                                         0.0)
    assert seen['unified'] is unified
    assert seq.unified is unified
    # The FSM cannot re-read the key: its module holds no handle on the config at
    # all. An OBJECT-level fact rather than a source-text one — importing `hw`
    # into `toss_sequencer` is what a re-read would have to start with, and this
    # fails the moment someone does.
    assert not hasattr(ts, 'hw'), (
        'toss_sequencer imported the config module — the next step is a second '
        'read of JB_OP_UNIFIED_CYCLE_ENABLED, free to disagree with the session')


# ── the post-release EXTEND ───────────────────────────────────────────────────

def test_the_extend_fires_once_BEFORE_the_plans_release_instant(monkeypatch):
    """TIME-triggered a lead AHEAD of `t_release_mono`, and exactly once.

    Not evidence-triggered, for the reason `_reach_action_if_due` gives: release
    evidence can lag by up to the 0.5 s grace, which here would eat the window the
    ~250 ms solve has to fit in.

    And BEFORE the release, not after — `extend`'s whole guarantee is that the head
    survives BIT FOR BIT, which is worth something only while the head is still
    PLAYING. Installed after the head has run out, the joined plan's `tau` already
    sits inside the NEW window and the swap jumps the machine forward by the solve
    time. The continuity guard catches that, but a refusal on every chain is a
    session that cannot run.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    calls = []
    monkeypatch.setattr(node, '_extend_unified_cycle',
                        lambda s, st, last_cycle: calls.append(last_cycle))
    lead = rcn._UNIFIED_EXTEND_LEAD_S
    assert lead > 0.0
    # A REST-terminal plan is cliff-safe and owes nothing — the shipped shape.
    state.unified_plan = types.SimpleNamespace(
        t_release_mono=1000.0, release_terminal=False,
        supersede_deadline_mono=0.0)
    node._tick_unified_extend(seq, state, 1001.0)
    assert calls == []
    assert state.unified_extended is False
    # A RELEASE-terminal one owes an extend before the deadline the NODE
    # published (not before the release, and not on a locally re-derived number).
    state.unified_plan = types.SimpleNamespace(
        t_release_mono=1000.0, release_terminal=True,
        supersede_deadline_mono=1000.0)
    node._tick_unified_extend(seq, state, 1000.0 - lead - 0.01)   # too early
    assert calls == []
    node._tick_unified_extend(seq, state, 1000.0 - lead + 0.01)   # in the window
    assert len(calls) == 1
    node._tick_unified_extend(seq, state, 1000.5)                 # never again
    assert len(calls) == 1


@pytest.mark.parametrize('remaining,kind', [
    (2, PlanCycle.Request.KIND_STEADY),
    (0, PlanCycle.Request.KIND_LANDING),
])
def test_the_extend_kind_follows_how_many_cycles_are_left(monkeypatch,
                                                          remaining, kind):
    """STEADY while cycles remain, LANDING for the last — and the STEADY period
    is the SESSION's beat.

    `flight + dwell` is exactly what `TossSessionSequencer.next_release_at`
    computes (landing + dwell, and landing = release + flight), so the plan's
    terminal release names the same instant the session's next cycle does rather
    than a nearby one. Getting that wrong is not a cadence blemish: the FSM's
    landing schedule is what the hand ball sensor's arrival window is cut from.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    node._toss_session_ref = types.SimpleNamespace(
        num_throws=3, cycle_index=3 - remaining, dwell_time_s=5.2)
    sent = []
    monkeypatch.setattr(node, '_call_plan_cycle',
                        lambda req: sent.append(req) or None)
    state.unified_plan = types.SimpleNamespace(
        t_release_mono=1.0, release_terminal=True, supersede_deadline_mono=1.0)
    node._tick_unified_extend(seq, state, 1.0)
    assert len(sent) == 1
    assert sent[0].mode == PlanCycle.Request.MODE_EXTEND
    assert sent[0].kind == kind
    if kind == PlanCycle.Request.KIND_STEADY:
        assert sent[0].period_s == pytest.approx(0.8 + 5.2)
        assert sent[0].catch_frac == pytest.approx(0.8 / (0.8 + 5.2))
    else:
        assert sent[0].period_s == pytest.approx(
            0.8 + rcn._UNIFIED_LAUNCH_WINDOW_S)


def test_a_steady_extend_hands_its_release_to_the_next_cycle(monkeypatch):
    """The chained release is ANNOUNCED, never re-planned.

    Planning a NEW LAUNCH there would be actively wrong: a LAUNCH is built through
    `CycleState.at_rest`, which DECLARES zero velocity, while the machine is
    mid-carry on the standing plan. trajectory_node refuses that — so the failure
    would be loud rather than dangerous, but a loud refusal on every chained cycle
    is still a session that cannot run.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    state.unified_launch_pending = True
    chained = types.SimpleNamespace(
        t_release_mono=seq.t_release, release_vel_mm_s=[0.0, 0.0, 3900.0],
        plan_wall_ms=180.0, duration_s=6.0, t_catch_mono=0.0,
        release_terminal=True, supersede_deadline_mono=seq.t_release + 6.0)
    node._toss_unified_chain = chained
    monkeypatch.setattr(
        node, '_call_plan_cycle',
        lambda req: pytest.fail('a chained release was re-planned'))
    node._tick_unified_launch(seq, state,
                              seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert state.unified_plan is chained
    assert node._toss_unified_chain is None
    ann = node._publishers['throw_announcements'].published[-1]
    assert ann.thrower_name == ann.target_id == node._robot_name
    assert ann.initial_velocity.z == pytest.approx(3900.0)
    assert ann.predicted_tof_sec == pytest.approx(0.8)


def test_a_skewed_chain_is_refused_rather_than_announced(monkeypatch):
    """The chain and the FSM must agree about WHEN the ball leaves.

    The FSM's `_t_release` is what its landing schedule — and therefore the hand
    ball sensor's arrival window — is cut from, so announcing a release the FSM
    does not expect would put every catch of that cycle in the wrong window and
    read as a MISS with the ball in the cup.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    state.unified_launch_pending = True
    node._toss_unified_chain = types.SimpleNamespace(
        t_release_mono=seq.t_release + 4.0,      # a whole beat out
        release_vel_mm_s=[0.0, 0.0, 3900.0], plan_wall_ms=1.0,
        duration_s=6.0, t_catch_mono=0.0, release_terminal=True,
        supersede_deadline_mono=seq.t_release + 10.0)
    monkeypatch.setattr(
        node, '_call_plan_cycle',
        lambda req: pytest.fail('a skewed chain fell through to a NEW launch'))
    node._tick_unified_launch(seq, state,
                              seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert node._publishers['throw_announcements'].published == []
    assert 'CHAIN_SKEW' in state.unified_reject
    assert node._toss_unified_chain is None


def test_the_announcement_is_built_from_the_plan_not_from_the_goal(monkeypatch):
    """The physics come off the TRAJECTORY THAT WILL BE EXECUTED.

    Same six fields, same units, same frame and the same thrower/target identity
    as `_announce_toss`, so every downstream consumer is unchanged — but the
    release velocity is the QP's pinned take-off rather than a value re-derived
    from the goal, and the landing is that release's ballistics under the SAME
    `ballistics_bc` gravity the planner pinned it against.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    vel = [12.0, -5.0, 3920.0]
    resp = types.SimpleNamespace(t_release_mono=time.perf_counter() + 1.0,
                                 release_vel_mm_s=vel, plan_wall_ms=175.0)
    node._announce_unified(seq, state, resp)
    ann = node._publishers['throw_announcements'].published[-1]
    site = np.array([0.0, 0.0, rcn._UNIFIED_THROW_CUP_Z_MM])
    lp = rcn.uc.ballistics_bc.position_at(site, np.array(vel), 0.8)
    assert ann.initial_position.z == pytest.approx(rcn._UNIFIED_THROW_CUP_Z_MM)
    assert ann.initial_velocity.x == pytest.approx(12.0)
    assert ann.landing_position.z == pytest.approx(float(lp[2]))
    assert ann.predicted_tof_sec == pytest.approx(0.8)
    # ONE announcement per throw, and the FSM is told about it exactly once.
    assert seq._announced is True
    assert len(node._publishers['throw_announcements'].published) == 1


# ── the clock seams the solve time creates ───────────────────────────────────

def test_the_cycle_origin_is_the_install_instant_not_the_seed():
    """The whole window is EXECUTED — the solve time is not skipped.

    `_plan_and_install_timed` anchors at the seed because its plan encodes an
    ABSOLUTE arrival. A cycle window encodes none: its release is "period_s after
    the window starts". Anchoring at the seed would make the emitter's first
    sample land at tau = solve_cost — a third of a 0.6 s launch skipped, as a step
    on seven channels — and the continuity guard would not see it, because it
    checks tau = 0 while the emitter samples somewhere else.
    """
    node = _cycle_node()
    before = time.perf_counter()
    resp = node._svc_plan_cycle(_launch_req(), PlanCycle.Response())
    after = time.perf_counter()
    assert resp.accepted is True, resp.message
    # The origin is at the END of the callback, not the start: the solve is a
    # measurable fraction of the window, so the two are far apart.
    solve_s = resp.plan_wall_ms / 1e3
    assert resp.t0_mono >= before + 0.5 * solve_s
    assert resp.t0_mono <= after + 1e-6
    # And the emitter's very first sample is the plan's own first knot.
    plan, _meta, t0 = node._cycle
    pose0, _tw, _ac = plan.state_at(0.0)
    live = node._current_state()[0]
    assert np.allclose(pose0[:3], live[:3], atol=0.5)


def test_the_launch_lead_makes_the_release_land_LATE_not_early(monkeypatch):
    """The residual release skew must never run past the FSM's release grace.

    The two sides are not symmetric. EARLY is free — `_step_throwing` explicitly
    tolerates release evidence that beats `t_release`, the settle deadline only
    gets more generous, and the possession window is cut from the PLAN's release
    under unified. LATE is bounded hard by `TOSS_RELEASE_GRACE_S` = 0.5 s: past it
    the cycle mints ABORTED_NO_RELEASE with the ball in the air. So the lead is
    derived from an UPPER bound on the solve cost, and this test is what says the
    bound is genuinely above it.

    ⚠ This assertion has already earned its keep: an earlier draft derived the
    lead from a LOWER bound (argued from the arrival band, before
    `_expected_landing_perf` was made plan-following), and under concurrent load
    the release landed 0.708 s past the FSM's schedule — straight through the
    grace. Measured against the SHIPPED shape (LAUNCH + chained LANDING), because
    the coordinator never asks for the cheap unchained one.
    """
    from jugglebot.toss_sequencer import TOSS_RELEASE_GRACE_S
    assert rcn._UNIFIED_LAUNCH_LEAD_S == pytest.approx(
        rcn._UNIFIED_LAUNCH_WINDOW_S + rcn._UNIFIED_PLAN_BUDGET_S)
    # The bound is genuinely BELOW the measured solve cost, which is what makes
    # the sign argument hold rather than merely stating it.
    node = _cycle_node()

    def _chained():
        r = _launch_req()
        r.chain = True
        r.chain_kind = PlanCycle.Request.KIND_LANDING
        r.chain_period_s = 0.6 + 0.6
        r.chain_catch_frac = 0.5
        return r

    # WARM FIRST — the ceiling bounds the cost of a call made INSIDE a session,
    # and the coordinator pays the cold one at session start on purpose
    # (`_unified_warm_planner`). Measuring the cold call here would pin the
    # ceiling against a cost the choreography has already moved off the critical
    # path: 3267 ms cold against a 424 ms warm median, measured 2026-09-04.
    warm = node._svc_plan_cycle(_chained(), PlanCycle.Response())
    assert warm.accepted is True, warm.message
    # Measure on a FRESH node in the same PROCESS: the planner is warm (that is
    # process state) while the machine is back at rest (that is node state), which
    # is exactly the situation a session's cycle 1 is in after the session-start
    # warm-up. Re-planning over the still-streaming first plan would instead be
    # refused STALE_STATE — correctly, and for an unrelated reason.
    resp = _cycle_node()._svc_plan_cycle(_chained(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    cost_s = resp.plan_wall_ms / 1e3
    assert cost_s < rcn._UNIFIED_PLAN_BUDGET_S, (
        'the plan solved in %.0f ms, SLOWER than the %.0f ms the launch lead '
        'assumes as a ceiling — the release would land %.3f s LATE, and anything '
        'past the %.2f s grace mints ABORTED_NO_RELEASE with the ball in the '
        'air. Re-measure and RAISE _UNIFIED_PLAN_BUDGET_S rather than deleting '
        'this assertion.'
        % (resp.plan_wall_ms, rcn._UNIFIED_PLAN_BUDGET_S * 1e3,
           cost_s - rcn._UNIFIED_PLAN_BUDGET_S, TOSS_RELEASE_GRACE_S))
    # The skew that results is EARLY, which costs nothing, and is bounded by the
    # ceiling itself rather than by anything downstream. SIGNED: the trigger
    # fires `window + budget` before the FSM's release and the plan then puts the
    # release `window + cost` after the install, so the release lands at
    # `cost − budget` relative to the schedule. Negative is early.
    # (This assertion read `BUDGET − cost < BUDGET` until 2026-09-05, which is
    # true for every cost > 0 and therefore said nothing at all.)
    skew = cost_s + rcn._UNIFIED_LAUNCH_WINDOW_S - rcn._UNIFIED_LAUNCH_LEAD_S
    assert skew < 0.0, (
        'the release lands %.3f s LATE against the FSM schedule' % (skew,))
    assert abs(skew) < rcn._UNIFIED_PLAN_BUDGET_S


def test_the_possession_window_follows_the_plan_under_unified():
    """The cup is watched around the instant the ball ACTUALLY leaves.

    The FSM's `landing_perf` is its own scheduled release plus the flight, and
    under unified the release is not the FSM's to schedule — it is
    `install + window` on the plan the emitter is streaming. Looking for the
    arrival edge around the FSM's number instead would mint a MISS on a real
    catch for a reason that is purely a clock.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    with node._lock:
        node._active_seq = seq
    # Legacy reading: the FSM's own schedule.
    state.unified_plan = None
    node._toss_unified_live = False
    assert node._expected_landing_perf() == pytest.approx(
        seq.t_release + seq.flight_time_s)
    # Unified: the PLAN's release + the flight, even when the two disagree.
    node._toss_unified_live = True
    state.unified_plan = types.SimpleNamespace(
        t_release_mono=seq.t_release + 0.37)
    assert node._expected_landing_perf() == pytest.approx(
        seq.t_release + 0.37 + seq.flight_time_s)
    # A plan with no release (a LANDING window) falls back rather than answering 0.
    state.unified_plan = types.SimpleNamespace(t_release_mono=0.0)
    assert node._expected_landing_perf() == pytest.approx(
        seq.t_release + seq.flight_time_s)


def test_the_session_schedules_off_the_plans_release_under_unified(monkeypatch):
    """`note_cycle_result` is fed PLAN-derived instants.

    The session's dwell is measured landing → next release, so scheduling off the
    FSM's own release under unified would build the solve time into every beat.
    Legacy is untouched: with no `unified_plan` on the cycle, the FSM's number is
    used exactly as before.
    """
    clock = _Clock()
    monkeypatch.setattr(rcn, 'time', clock)
    node = _unified_session_node(monkeypatch, clock)
    monkeypatch.setattr(node, '_set_hand_source', lambda s: (True, 'ok'))
    seen = []
    real = rcn.TossSessionSequencer.note_cycle_result
    monkeypatch.setattr(
        rcn.TossSessionSequencer, 'note_cycle_result',
        lambda self, res, t_rel, t_land, **kw: (
            seen.append((t_rel, t_land)), real(self, res, t_rel, t_land, **kw))[1])

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
        # The plan released 0.21 s after the FSM's scheduled instant (the solve).
        state.unified_plan = types.SimpleNamespace(
            t_release_mono=seq.t_release + 0.21)
        clock.t = seq.t_release + float(seq.flight_time_s) + 0.3
        _stamp(node, clock.t)
        return TossResult(True, 'CAUGHT', 2.0, 0.8), 'fsm'

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    gh = _unified_goal(num_throws=1)
    node._execute_toss_continuous(gh)
    assert len(seen) == 1
    t_rel, t_land = seen[0]
    assert t_land - t_rel == pytest.approx(0.8)
    # The PLAN's release, not the FSM's.
    assert t_rel > 0.0


# ═════════════════════════════════════════════════════════════════════════════
# The release cliff (HIGH review finding, 2026-09-04)
# ═════════════════════════════════════════════════════════════════════════════
#
# A plan that ENDS AT A RELEASE commands a hard STOP at the throw if it is
# streamed to its end: `CyclePlan.state_at/hand_at(t >= duration)` return the
# terminal hold with ZERO twist, and `KnotEmitter.frame` samples tau + dt for the
# u1/v1 knot — so the frame at `duration - dt` carries `hand_next_vel_rps = 0` and
# `vel_next_mm_s = 0`, and that v1 is what the firmware's Hermite uses for the
# FINAL 25 ms segment, i.e. the release stroke itself. The resulting error is well
# inside MAX_LEAD_HAND_REV and MAX_DEVIATION_HAND_REV, so NO guard fires.

def test_the_release_cliff_is_real_and_the_emitted_v1_goes_to_zero():
    """THE MECHANISM, measured rather than asserted from the docstring.

    This is the test that would have caught the finding: it compares the hand
    velocity the plan actually carries at its terminal knot against the v1 the
    emitter puts on the wire for the final segment.
    """
    from jugglebot.motion.trajectory.emitter import KnotEmitter
    node = _cycle_node()
    req = _launch_req()
    req.chain = False                       # deliberately release-terminal
    resp = node._svc_plan_cycle(req, PlanCycle.Response())
    assert resp.accepted is True, resp.message
    assert resp.release_terminal is True
    plan, meta, t0 = node._cycle
    dt = float(plan.dt)
    # The plan's own terminal hand velocity: a full-speed launch stroke.
    true_terminal = float(plan.hand_vel_rps[-1])
    assert abs(true_terminal) > 50.0, true_terminal
    # What the emitter would ship for the LAST segment.
    emitter = KnotEmitter(node._geom, knot_dt_s=dt)
    frame = emitter.frame(plan, plan.total_duration - dt, 0)
    assert frame['hand_next_vel_rps'] == pytest.approx(0.0, abs=1e-9)
    assert np.allclose(frame['vel_next_mm_s'], 0.0, atol=1e-9)
    # One segment before it, the same call carries the real velocity — so the
    # cliff is the LAST segment specifically, not a property of the emitter.
    ok = emitter.frame(plan, plan.total_duration - 2.0 * dt, 0)
    assert abs(float(ok['hand_next_vel_rps'])) > 50.0


def test_a_release_terminal_plan_left_to_expire_is_detected_and_logged(
        monkeypatch):
    """The node ALARMS when the deadline passes with the plan still installed.

    The node does not fix it — the orchestrator owns the chain, and a rescue plan
    invented here would be a second motion authority on the wire. What the node
    owns is the alarm, and it is the whole point: nothing else in the machine
    notices. No guard trips, nothing latches, and the only symptom is a throw that
    went somewhere else.
    """
    node = _cycle_node()
    req = _launch_req()
    req.chain = False
    resp = node._svc_plan_cycle(req, PlanCycle.Response())
    assert resp.accepted is True, resp.message
    assert resp.release_terminal is True
    assert resp.supersede_deadline_mono == pytest.approx(
        resp.t0_mono + resp.duration_s - float(node._cycle[0].dt))
    errors = []
    monkeypatch.setattr(node.get_logger(), 'error',
                        lambda msg, **kw: errors.append(str(msg)))
    # Before the deadline: a positive remaining time, no alarm.
    node._publish_status()
    st = node._publishers['trajectory/status'].published[-1]
    assert st.cycle_supersede_deadline_s > 0.0
    assert errors == []
    # Past it, with nothing installed after: NEGATIVE on the wire and ONE loud
    # line (once per install — this runs on the 5 Hz timer).
    node._cycle_supersede_deadline = time.perf_counter() - 0.20
    node._publish_status()
    st = node._publishers['trajectory/status'].published[-1]
    assert st.cycle_supersede_deadline_s < 0.0
    assert len(errors) == 1
    assert 'CLIFF' in errors[0]
    node._publish_status()
    assert len(errors) == 1                 # not re-logged every tick


def test_the_shipped_launch_request_installs_a_REST_terminal_plan():
    """The fix, end to end: LAUNCH + LANDING as ONE install.

    The first installed plan is rest-terminal, so the cliff class is REMOVED
    rather than raced — there is no window in which a correctly planned launch is
    one late service call away from a stopped throw. And the announced release is
    the FIRST one (the throw), not the plan's terminal instant.
    """
    node = _cycle_node()
    req = _launch_req()
    req.chain = True
    req.chain_kind = PlanCycle.Request.KIND_LANDING
    req.chain_period_s = 0.6 + 0.6
    req.chain_catch_frac = 0.6 / (0.6 + 0.6)
    resp = node._svc_plan_cycle(req, PlanCycle.Response())
    assert resp.accepted is True, resp.message
    assert resp.release_terminal is False
    assert resp.supersede_deadline_mono == 0.0
    # The announced release is the LAUNCH's, a window in — not the plan's end.
    assert resp.t_release_mono == pytest.approx(resp.t0_mono + 0.6, abs=1e-6)
    assert resp.duration_s > 1.0
    assert resp.t_catch_mono > resp.t_release_mono
    # The joined plan's terminal knot is at REST on both channels: that is what
    # makes streaming it to the end harmless.
    plan, _meta, _t0 = node._cycle
    assert abs(float(plan.hand_vel_rps[-1])) < 1e-6
    assert np.allclose(plan.pose_vel[-1], 0.0, atol=1e-6)
    # And the node arms no alarm for it.
    node._publish_status()
    assert node._publishers['trajectory/status'].published[-1] \
        .cycle_supersede_deadline_s == 0.0


def test_a_chained_launch_is_what_the_coordinator_actually_asks_for():
    """The coordinator's LAUNCH request carries the chain, with the seam times.

    The chained window's catch instant is the FLIGHT time on its own clock,
    because the seam it starts at IS the release and the ball touches down a
    flight later. Asserted on the request rather than on the plan so the
    derivation is pinned even when the planner is stubbed.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    state.unified_launch_pending = True
    sent = []
    node._call_plan_cycle = lambda req: sent.append(req) or None
    node._tick_unified_launch(seq, state,
                              seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert len(sent) == 1
    req = sent[0]
    flight = float(seq.flight_time_s)
    assert req.chain is True
    assert req.chain_kind == PlanCycle.Request.KIND_LANDING
    assert req.chain_period_s == pytest.approx(
        flight + rcn._UNIFIED_LAUNCH_WINDOW_S)
    assert req.chain_catch_frac * req.chain_period_s == pytest.approx(flight)


def test_the_extend_safety_net_lands_before_the_published_deadline(monkeypatch):
    """A release-terminal plan IS extended, and with the whole solve cost to spare.

    The clock is injected so the 250 ms budget is exercised rather than the box's
    real speed: the trigger must fire at least one full plan cost before the
    deadline the node published, or the extend installs after the cliff and the
    guarantee is worthless.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    node._toss_session_ref = types.SimpleNamespace(
        num_throws=3, cycle_index=1, dwell_time_s=5.2)
    deadline = 5000.0
    state.unified_plan = types.SimpleNamespace(
        t_release_mono=deadline - 1.0, release_terminal=True,
        supersede_deadline_mono=deadline)
    fired_at = []
    monkeypatch.setattr(
        node, '_call_plan_cycle',
        lambda req: fired_at.append(req) or None)
    # Step the injected clock across the trigger in 40 ms coordinator ticks.
    t = deadline - 2.0
    while t < deadline and not fired_at:
        node._tick_unified_extend(seq, state, t)
        t += 0.040
    assert fired_at, 'the safety net never fired before the deadline'
    trigger_t = t - 0.040
    slack = deadline - trigger_t
    # Against the EXTEND's own measured cost, not the launch's: an extend is ONE
    # window plus the join's revalidate (measured max 387 ms on this box, idle),
    # while `_UNIFIED_PLAN_BUDGET_S` bounds the doubled LAUNCH+LANDING install.
    assert slack >= rcn._UNIFIED_EXTEND_LEAD_S - 0.05, (
        'the extend fired %.3f s before the deadline, short of its own %.2f s '
        'lead' % (slack, rcn._UNIFIED_EXTEND_LEAD_S))
    # The MEASURED worst EXTEND (one window + the join's revalidate over the
    # whole result) is 387 ms on this box, idle. The lead must cover that plus the
    # round trip, or the extend installs AFTER the cliff it was asked for.
    assert slack >= 0.40, (
        'the extend fired %.3f s before the deadline — under the measured 387 ms '
        'worst-case MODE_EXTEND cost, so a nominal solve would install AFTER the '
        'cliff' % slack)
