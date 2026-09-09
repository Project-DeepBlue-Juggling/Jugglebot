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

import contextlib
import dataclasses
import math
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


@contextlib.contextmanager
def _frozen_perf():
    """Freeze ``time.perf_counter()`` at entry — for the NODE and for this module.

    Not a convenience. Two kinds of assertion in this file are otherwise about
    the Jetson's scheduler rather than about the code: one samples a plan at a
    ``tau`` captured on one line and compares it against ``_current_state()``,
    which captures its own ``perf_counter()`` on the next (any preemption between
    them reads as drift — that is how
    ``test_the_velocity_term_passes_the_same_origin_re_installs`` failed once in a
    full ``tests/ros/`` run on 2026-09-06 and passed in isolation); the other has
    to know the exact ``tau`` the node seeded a window at, to compare the cup
    acceleration it sampled against the running plan's own. Freezing makes both
    exact instead of approximately true.

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


#: How far into a running carry the "planned over a LIVE cycle" seeds sit. Far
#: enough that the platform is unambiguously moving (42.8 mm/s measured, 171x the
#: 0.25 mm/s rest bound) and the cup acceleration is a real number rather than a
#: turning point, and short enough to leave most of the 1.4 s window ahead.
_LIVE_TAU_S = 0.30


def _running_carry(node, tau=_LIVE_TAU_S):
    """Install a SETTLE and re-anchor its origin ``tau`` in the PAST. ``origin``.

    The node is then in the state a second ``MODE_NEW`` arrives in on the bench:
    a cycle plan streaming, ``tau`` seconds in, the platform mid-carry. Both
    clocks are moved together — ``_plan_t0`` (what ``_current_state`` samples on)
    and the cycle record's own origin — because a seed that read them apart is
    the very defect these tests pin.

    Re-anchoring rather than sleeping: a real 0.30 s sleep costs 0.30 s per test
    and still lands wherever the scheduler puts it, and the plan is a pure
    function of ``tau``, so there is nothing to be gained by waiting.
    """
    resp = node._svc_plan_cycle(_settle_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    plan, meta, _t0 = node._cycle
    origin = time.perf_counter() - float(tau)
    node._plan_t0 = origin
    node._cycle = (plan, meta, origin)
    _refresh(node)
    return origin


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


def test_the_accept_line_attributes_its_own_solve():
    """The accept message carries the per-stage split AND the box's load.

    ADDED 2026-09-06. The hardware's five slow solves (1655.1 / 2021.2 / 2158.9
    / 1461.5 / 1444.7 ms against ~200 ms nominal) left exactly one number behind
    — ``plan %.1f ms`` — so every hypothesis had to be re-tested offline, and two
    were withdrawn on measurements taken below the knee. Two things fix that at
    the source, and both have to be ON THE LINE ITSELF rather than in a topic a
    session may or may not have bagged:

    * the stage split, which says WHERE the time went (measured: ``val`` is
      ~89 % of a healthy solve, ``qp`` ~6 %); and
    * ``load1``, because **the bag carries no host-CPU channel at all** — after
      the fact there is no way to ask whether the box was busy, which is the one
      condition that reproduces a multi-second solve.
    """
    node = _cycle_node()
    resp = node._svc_plan_cycle(_launch_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    for key in ('qp=', 'tilt=', 'dec=', 'val=', 'cont='):
        assert key in resp.message, resp.message
    assert 'load1=' in resp.message, resp.message
    # The split is in ms and sums to the reported plan wall time, so a reader can
    # check the line against itself.
    import re as _re
    parts = {k: float(v) for k, v in
             _re.findall(r'(qp|tilt|dec|val|cont)=([0-9.]+)', resp.message)}
    assert sum(parts.values()) == pytest.approx(resp.plan_wall_ms, rel=0.05), (
        parts, resp.plan_wall_ms)


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


def test_a_NEW_carry_from_a_level_rest_INSTALLS_continuously():
    """The UH-3 rung, end to end: planned from a held pose, installed onto it.

    `_install_continuity_ok` is the last gate before a `CyclePlan` replaces what
    the emitter is streaming, and it asks one question — does the plan's knot 0
    match the live COMMANDED state? Until 2026-09-06 a NEW window's knot 0
    carried whatever tilt the banking schedule's smoother left there (knot 0 is
    not one of its anchors), and `cup_realize.decompose` turns tilt into centroid
    through the 744.3 mm `CUP_TILT_CENTER_Z_MM` lever, so the plan opened at a
    pose the machine was not at and this guard refused the install:

        STALE_STATE: leg position drift 0.1248 rev > 0.0600   (bench, 2026-09-06)

    The rung could not be flown at all. That is the shape of the whole finding:
    the plan was never *wrong* by any downstream measure — `validate_cycle`
    passes it — it just did not start where the robot was.

    Both halves are asserted here because only the pair is a regression test:
    the plan is accepted with the seed pin, and REFUSED without it. Removing the
    pin is what reproduces the defect; asserting the pass alone would keep
    passing if the pin were deleted and the request happened to stay under the
    bound.
    """
    # The shipped rung: 60 mm in +x, 1.4 s. Knot 0 lands exactly on the held
    # pose, so the drift the guard measures is zero rather than merely small.
    #
    # The reference pose is read BEFORE the call: a successful install REPLACES
    # the active plan, so `_current_state()` afterwards samples the plan that was
    # just installed, a few ms in, and would measure the machine MOVING rather
    # than the seam this test is about.
    node = _settle_node()
    live_pose = np.asarray(node._current_state()[0], dtype=float).copy()
    resp = node._svc_plan_cycle(_settle_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    plan, _meta, _t0 = node._cycle
    drift = float(np.max(np.abs(node._pose_to_motor_rev(plan.pose[0])
                                - node._pose_to_motor_rev(live_pose))))
    assert drift == pytest.approx(0.0, abs=1e-9), drift
    # The tilt channel is the one that was lying, so name it.
    assert np.array_equal(plan.pose[0][3:5], live_pose[3:5])


def test_without_the_seed_pin_the_carry_is_REFUSED_at_the_install(monkeypatch):
    """The same service call, with the pin removed: `STALE_STATE`, at the guard.

    A HARDER carry than the shipped rung — 100 mm in +x over 0.8 s — because the
    knot-0 tilt the smoother leaves scales with the cup acceleration, and this
    node's seed sits at `SETTLE_CUP_Z_MM` where the shipped 60 mm / 1.4 s carry
    leaves only 0.398° (0.0193 rev, UNDER the 0.06 bound). MEASURED (2026-09-06,
    `/tmp/probe_knot0_tilt.py`, session limits 250/3000/150000): 100 mm / 0.8 s
    leaves **1.7703°** on knot 0 and **0.0861 rev** of drift, against the guard's
    0.06 — the same mechanism as the bench's 2.53° / 0.1248 rev at its own seed,
    which the two share to 0.0494 rev per degree.

    (The bench seed itself is a PARKED hand at -0.038 rev, whose cup sits 11.2 mm
    BELOW the QP's cup-box floor. That start is outside the box the solver is
    allowed to move in, and the plan it produces shoots the cup to the box
    CEILING — 9.65 rev of slider on a carry that should be flat. That is a
    separate, pre-existing defect, unrelated to the tilt pin and present with or
    without it, so this test does not build on it.)
    """
    node = _settle_node()
    monkeypatch.setattr(uc, '_start_tilt_for', lambda state: None)
    resp = node._svc_plan_cycle(_settle_req(period_s=0.8, dx_mm=100.0),
                                PlanCycle.Response())
    assert resp.accepted is False
    assert resp.code == feas.STALE_STATE
    assert 'leg position drift' in resp.message, resp.message

    # ...and with the pin back, the identical request installs.
    monkeypatch.undo()
    node = _settle_node()
    live_pose = np.asarray(node._current_state()[0], dtype=float).copy()
    resp = node._svc_plan_cycle(_settle_req(period_s=0.8, dx_mm=100.0),
                                PlanCycle.Response())
    assert resp.accepted is True, resp.message
    plan, _meta, _t0 = node._cycle
    assert float(np.max(np.abs(
        node._pose_to_motor_rev(plan.pose[0])
        - node._pose_to_motor_rev(live_pose)))) < 1e-9


def test_a_NEW_window_over_a_LIVE_cycle_SAMPLES_the_running_cup_acceleration():
    """A second SETTLE seeded mid-carry opens on the acceleration the cup HAS.

    Until 2026-09-06 the moving branch of `_cycle_start_state` handed the QP
    `post_release=True` with no cup acceleration, so `to_cup_state` supplied the
    free-fall `g` as the EXACT start-of-window equality — the claim "a ball just
    left this cup", asserted about a machine mid-lateral-carry with a seated ball
    and nothing in the air. MEASURED (2026-09-06, `/tmp/probe_rep2e.py`, these
    same fixtures, `tau` = 0.3001 s into the 1.4 s carry, banking on, session
    limits 250/3000/150000): the running plan's own cup acceleration there is
    **[110.80, 0.00, -0.00] mm/s^2** against the fabricated **[0, 0, -9806]** —
    88x larger and pointing the other way. The twin test below is what that
    costs; this one pins the source.

    The equality against `meta.cup_plan.sample(tau)` is the whole claim: the
    boundary condition is SAMPLED from the running cycle's own cup track (the
    same object `unified_cycle.replan_tail` reads its splice state from), not
    defaulted, not differentiated off a pose, and not zeroed. The clock is frozen
    so `tau` here IS the `tau` the node seeded at — an approximate comparison
    would pass with a *different* plan's acceleration in it.
    """
    node = _settle_node()
    origin = _running_carry(node)
    _plan, meta, _t0 = node._cycle
    with _frozen_perf() as now:
        tau = now - origin
        state, code, err = node._cycle_start_state(uc.SETTLE)
        assert (code, err) == ('', ''), (code, err)
        assert state is not None
        # It IS moving — the branch under test is the one this reaches.
        assert float(np.max(np.abs(state.pose_vel[:3]))) > 10.0

        expected = np.asarray(meta.cup_plan.sample(tau)[2], dtype=float) * 1000.0
        assert state.cup_accel_mm_s2 is not None
        assert state.cup_accel_mm_s2 == pytest.approx(expected, abs=1e-12)
        # ...and that number is neither zero nor free fall.
        assert float(np.max(np.abs(expected))) > 1.0
        assert float(np.max(np.abs(expected))) < 0.05 * abs(uc._G_MM_S2[2])
        # No ball left this cup mid-carry, so the detach cone is not asserted —
        # the same claim `replan_tail` makes for its own splice state. That also
        # restores the knot-0 TILT pin, which `_start_tilt_for` withholds from a
        # post-release state carrying no detach axis.
        assert state.post_release is False
        assert state.detach_axis is None

        # End to end, on the same frozen instant: the window installs.
        resp = node._svc_plan_cycle(_settle_req(), PlanCycle.Response())
        assert resp.accepted is True, resp.message
        plan2, _meta2, _t2 = node._cycle

    # The HEAD is monotone — no dip below knot 0. Sampled through `hand_at`,
    # because the dip the fabrication produces lives BETWEEN knots 0 and 1 (the
    # firmware's Hermite is what executes it) and a knot-array check cannot see
    # it at all.
    start = float(plan2.hand_rev[0])
    dip = min(float(plan2.hand_at(t / 4000.0)[0]) for t in range(200)) - start
    assert dip > -1e-4, 'the head dipped %.6f rev below knot 0' % dip
    assert float(np.max(np.abs(plan2.hand_rev - start))) < 0.25


def test_the_fabricated_free_fall_bows_the_SAME_seed_and_nothing_refuses_it():
    """The regression twin: one seed, two boundary conditions, two plans.

    Identical to the test above in every respect except the two fields the old
    moving branch set — `post_release=True` and `cup_accel_mm_s2=None`, which is
    exactly what `to_cup_state` turns into `g`. MEASURED (2026-09-06,
    `/tmp/probe_rep2e.py`, the same seed):

    * knot 1 - knot 0 = **-1.42e-07 rev** while the knot velocities run
      **-0.0000 -> +3.8755 rev/s** — a head whose position does not move while
      its velocity takes off, so the firmware's Hermite dips **-0.014353 rev**
      (0.454 mm of slider) BELOW knot 0 before it climbs;
    * the plan then bows **2.3845 rev** of slider (75 mm of cup z) across a move
      that is purely lateral, and leaves **0.008014 rad** on knot 0's tilt
      against the live pose (0.022293 rev of leg drift at the install gate)
      because `_start_tilt_for` returns None for a post-release state with no
      detach axis;
    * the honest seed measures **-0.000000 rev** of dip, **0.0001 rev** of
      travel and **0.000000** on both drift terms.

    **And `validate_cycle` ACCEPTS BOTH.** That is why this is a test at the seed
    and not a gate assertion: nothing downstream refuses the fabricated plan on
    these fixtures. (It does at the bench's parked-hand seed, where the same dip
    crosses the stroke floor — MEASURED 2026-09-06, `/tmp/probe_rep2d.py`:
    `HAND_STROKE: hand position -0.002 rev outside [0.000, 9.959]`. A defect that
    is a hard refusal at one seed and a silent 75 mm bow at another is worse than
    one that is always loud, not better.)
    """
    node = _settle_node()
    origin = _running_carry(node)
    with _frozen_perf():
        honest, code, err = node._cycle_start_state(uc.SETTLE)
        assert (code, err) == ('', ''), (code, err)
        goals = node._cycle_goals_from_request(_settle_req())
        fabricated = dataclasses.replace(honest, post_release=True,
                                         cup_accel_mm_s2=None)
        plans = {}
        for name, state in (('honest', honest), ('fabricated', fabricated)):
            plan, _meta = uc.plan_cycle(uc.SETTLE, goals, state,
                                        node._limits, node._geom)
            plans[name] = plan

    def _dip(plan):
        start = float(plan.hand_rev[0])
        return min(float(plan.hand_at(t / 4000.0)[0]) for t in range(200)) - start

    def _travel(plan):
        return float(np.max(np.abs(plan.hand_rev - plan.hand_rev[0])))

    # The fabrication: a dip below knot 0 and a bow the carry never asked for.
    assert _dip(plans['fabricated']) < -1e-3
    assert _travel(plans['fabricated']) > 1.0
    # The sampled boundary condition: neither.
    assert _dip(plans['honest']) > -1e-4
    assert _travel(plans['honest']) < 0.25
    # The tilt pin follows `post_release`, so the fabrication loses it too.
    live_tilt = np.asarray(honest.pose, dtype=float)[3:5]
    assert np.array_equal(np.asarray(plans['honest'].pose[0])[3:5], live_tilt)
    assert not np.array_equal(np.asarray(plans['fabricated'].pose[0])[3:5],
                              live_tilt)


def test_a_moving_machine_with_NO_cycle_to_sample_is_refused_IN_MOTION():
    """No cup track, no boundary condition — and the node says so rather than
    inventing one.

    A legacy plan moving the machine (a `go_to_pose`, a hold's decel-to-rest, a
    graceful stop) leaves this node with a pose, a twist and NO cup trajectory:
    `CycleState.pose_accel` is carried but `to_cup_state` never reads it, and
    differentiating a pose to recover a cup acceleration is precisely the
    measurement noise the detach-cone rows must not be fed. So there is nothing
    honest to put on knot 0 and the request is refused `IN_MOTION`, naming the
    speeds and the remedy. The alternative is the defect the two tests above
    measure — `g`, 88x the real number, in the one constraint a ball's flight
    depends on.

    `IN_MOTION` rather than `STALE_STATE` because the caller's mistake is a
    different one: nothing here is stale, the state is perfectly fresh and
    perfectly moving.
    """
    # (1) The unit shape: a moving seed with no cycle record at all.
    node = _cycle_node()
    # Ten times the node's own linear rest bound, read from the node rather than
    # hardcoded — the bound is a thousandth of the LIVE session velocity limit,
    # and a literal here would silently stop meaning "moving" the day the session
    # raises or lowers that limit.
    rest_bound = 1e-3 * node._limits.leg_vel_mmps
    twist = np.zeros(6)
    twist[0] = 10.0 * rest_bound
    monkey = (np.zeros(6), twist, np.zeros(6))
    node._current_state = lambda **kw: monkey  # noqa: E731
    assert node._cycle is None
    state, code, err = node._cycle_start_state(uc.SETTLE)
    assert state is None
    assert code == tn._IN_MOTION
    assert 'no unified cycle is active' in err, err
    assert '%.4f mm/s' % (10.0 * rest_bound) in err, err
    # And the LAUNCH branch still refuses outright over a live plan, as before.
    state, code, err = node._cycle_start_state(uc.LAUNCH)
    assert state is None and code == feas.STALE_STATE
    assert 'starts from REST' in err

    # (2) The real shape, through the service: a running carry, then a HOLD —
    # which is a profiled decel-to-rest, i.e. a LEGACY plan that is still moving
    # the machine — and `_install` drops the cycle record with it. The clock is
    # frozen from before the hold so the seed lands at tau = 0 on that decel,
    # where it is unambiguously moving; unfrozen the stop finishes in ~14 ms at
    # the session's 3000 mm/s^2 and the test would race it.
    node = _settle_node()
    _running_carry(node)
    with _frozen_perf():
        node._svc_hold(Trigger.Request(), Trigger.Response())
        assert node._cycle is None
        assert float(np.max(np.abs(node._current_state()[1][:3]))) > 10.0
        resp = node._svc_plan_cycle(_settle_req(), PlanCycle.Response())
    assert resp.accepted is False
    assert resp.code == tn._IN_MOTION, resp.message
    assert 'no unified cycle is active' in resp.message, resp.message


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
        node._current_state = (                                       # noqa: E731
            lambda **kw: (np.zeros(6), tw, np.zeros(6)))
        if hand_vel is not None:
            node._commanded_hand_state = (                            # noqa: E731
                lambda **kw: (_REST_HAND_REV, float(hand_vel)))
        return node

    # A true rest seeds the EXACT zeros — and, since no ball left this cup, a
    # state that does not claim one did.
    at_rest, code, err = seeded()._cycle_start_state(uc.SETTLE)
    assert (code, err) == ('', '') and at_rest is not None
    assert at_rest.post_release is False
    assert at_rest.cup_accel_mm_s2 is not None
    assert float(np.max(np.abs(at_rest.cup_accel_mm_s2))) == 0.0
    assert seeded()._cycle_start_state(uc.LAUNCH)[0] is not None

    # And these three are NOT at rest. Each is a channel the old predicate was
    # blind to: 5 mm/s is 20x the linear bound and 1/168th of the OLD one;
    # 0.02 rad/s carries no linear component at all; 0.5 rev/s is the hand.
    #
    # These nodes carry no cycle record, so the post-release kind is REFUSED
    # rather than seeded — there is no cup track to sample the boundary
    # condition from, and the refusal is what proves the predicate FIRED on that
    # channel. (Where a cycle IS running, the same predicate routes the seed to
    # the sampled acceleration instead; that is the pair of tests above.)
    for label, twist, hand_vel in (
            ('5 mm/s linear', [5.0, 0.0, 0.0, 0.0, 0.0, 0.0], None),
            ('0.02 rad/s angular', [0.0, 0.0, 0.0, 0.02, 0.0, 0.0], None),
            ('0.5 rev/s hand', None, 0.5)):
        node = seeded(twist, hand_vel)
        state, code, err = node._cycle_start_state(uc.SETTLE)
        assert state is None, label
        assert code == tn._IN_MOTION, label
        assert 'no unified cycle is active' in err, label
        state, code, err = node._cycle_start_state(uc.LAUNCH)
        assert state is None and code == feas.STALE_STATE, label
        assert 'starts from REST' in err, label


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


# ═════════════════════════════════════════════════════════════════════════════
# The UH-7 STEADY ring, through the REAL service
# ═════════════════════════════════════════════════════════════════════════════
#
# Nothing else in the suite drives a `KIND_STEADY` solve through
# `trajectory/plan_cycle`: the coordinator-side tests stub `_call_plan_cycle` and
# inspect the request, and `tests/motion/` exercises `plan_steady` in-process with
# no origin, no install-continuity guard and no supersede alarm. So the chain the
# constant beat is made of — a plan that STAYS release-terminal beat after beat,
# with its streaming deadline moving forward rather than being retired — has
# never been executed end to end. These are the tests that execute it.

#: The ring's beat (s). The sim gate's `CYCLE_PERIOD_S`, and it is a MEASURED
#: floor rather than a round number: `sim/unified_gate.py:240-244` records that a
#: 1.0 s window cannot absorb a displaced catch (LIMIT_JERK 155-198 k against the
#: 150 k session cap) while 1.4 s carries the whole ring at 81-90 k.
_RING_BEAT_S = 1.4
#: Time of flight the ring throws for, and `_launch_req`'s own default.
_RING_FLIGHT_S = 0.6


def _steady_chain_req():
    """`MODE_NEW` LAUNCH with a STEADY chained in — the ring's FIRST install.

    The shipped install chains a LANDING and is rest-terminal by design; this one
    chains a STEADY and is therefore release-terminal from cycle 1, which is the
    whole shape change UH-7 makes and the reason the supersede deadline arms for
    the first time.
    """
    req = _launch_req(flight_s=_RING_FLIGHT_S)
    req.chain = True
    req.chain_kind = req.KIND_STEADY
    req.chain_period_s = _RING_BEAT_S
    req.chain_catch_frac = _RING_FLIGHT_S / _RING_BEAT_S
    return req


def _steady_extend_req():
    """`MODE_EXTEND` KIND_STEADY — one more beat onto the terminal release."""
    req = _launch_req(flight_s=_RING_FLIGHT_S)
    req.mode = req.MODE_EXTEND
    req.kind = req.KIND_STEADY
    req.period_s = _RING_BEAT_S
    req.catch_frac = _RING_FLIGHT_S / _RING_BEAT_S
    return req


def _landing_extend_req():
    """`MODE_EXTEND` KIND_LANDING — the last cycle, which brings the ring to rest."""
    req = _steady_extend_req()
    req.kind = req.KIND_LANDING
    return req


@pytest.fixture(scope='module')
def ring_stages():
    """The three real service calls that fly a two-cycle ring, recorded.

    ``[(plan, meta, response), …]`` after MODE_NEW(LAUNCH+STEADY),
    MODE_EXTEND(STEADY) and MODE_EXTEND(LANDING) respectively.

    Module-scoped because the three solves cost ~3.5 s together (MEASURED
    2026-09-07 on this Jetson: 1935 / 904 / 659 ms, the first paying the cold QP).
    The plans and metas are immutable products, so the tests below install them
    into their OWN fresh nodes rather than sharing this one — the node is the part
    that gets poked.
    """
    node = _cycle_node()
    node._catch_armed = False                      # the unified state
    out = []
    resp = node._svc_plan_cycle(_steady_chain_req(), PlanCycle.Response())
    assert resp.accepted is True, resp.message
    out.append(node._cycle[:2] + (resp,))
    for req in (_steady_extend_req(), _landing_extend_req()):
        resp, _origin = _extend_alive(node, req)
        assert resp.accepted is True, resp.message
        out.append(node._cycle[:2] + (resp,))
    return out


def _install_ring(node, stage, tau):
    """Put ``stage``'s plan on ``node`` with the plan clock reading ``tau``.

    The same poke `_anchor_mid_flight` performs, against a pre-solved plan: both
    clocks are moved together — `_plan_t0` (what `_current_state` samples on) and
    the cycle record's own origin — because a seed that read them apart is the
    defect those tests pin. Re-installing a finished plan is exactly what the
    service would have left behind, without paying for the solve again.
    """
    plan, meta = stage[0], stage[1]
    t0 = time.perf_counter() - float(tau)
    node._install(plan, t0=t0)
    node._cycle = (plan, meta, t0)
    node._plan_t0 = t0
    _refresh(node)
    return t0


def test_a_real_KIND_STEADY_ring_installs_and_keeps_its_deadline_armed(
        ring_stages):
    """LAUNCH+STEADY, +STEADY, +LANDING — release-terminal until the last one.

    The claim UH-7 rests on, executed rather than asserted about a stub: chaining
    a STEADY keeps the installed plan RELEASE-terminal, so
    `latest_supersede_time_s` returns `duration - dt` instead of `inf` and the
    node's supersede alarm ARMS — for the first time in this codebase's life, the
    shipped LAUNCH+LANDING install being rest-terminal by design. Each extend
    moves that deadline forward by exactly one beat; the terminal LANDING retires
    it. Anything else is the release-terminal cliff left un-managed: past the
    deadline the emitter's u1 sample reads the plan's terminal HOLD, so the
    release stroke ships `v1 = 0` against a true 93 rev/s — inside every firmware
    guard, so the only symptom is a throw that went somewhere else.

    MEASURED (2026-09-07, /tmp/probe_uh7_ros.py, run twice with identical
    output): durations 2.0 / 3.4 / 4.8 s; releases 0.6, 2.0, 3.4; catches 1.2,
    2.6, 4.0; supersede at t0 + 1.975 then t0 + 3.375 then retired.
    """
    (p1, m1, r1), (p2, m2, r2), (p3, m3, r3) = ring_stages
    dt = float(p1.dt)

    # Cycle 1: LAUNCH + STEADY, release-terminal, deadline armed at duration - dt.
    assert r1.duration_s == pytest.approx(0.6 + _RING_BEAT_S, abs=1e-9)
    assert r1.release_terminal is True
    assert (r1.supersede_deadline_mono - r1.t0_mono) == pytest.approx(
        float(p1.total_duration) - dt, abs=1e-9)
    assert [float(m.t_s) for m in m1.releases] == [pytest.approx(0.6),
                                                   pytest.approx(2.0)]
    assert [float(m.t_s) for m in m1.catches] == [pytest.approx(1.2)]

    # Cycle 2: one more beat. STILL release-terminal; the deadline MOVED, and by
    # exactly the window — not retired, and not left where it was.
    assert r2.duration_s == pytest.approx(float(r1.duration_s) + _RING_BEAT_S,
                                          abs=1e-9)
    assert r2.release_terminal is True
    assert ((r2.supersede_deadline_mono - r2.t0_mono)
            - (r1.supersede_deadline_mono - r1.t0_mono)) == pytest.approx(
                _RING_BEAT_S, abs=1e-9)
    assert [float(m.t_s) for m in m2.releases] == [
        pytest.approx(0.6), pytest.approx(2.0), pytest.approx(3.4)]
    assert [float(m.t_s) for m in m2.catches] == [pytest.approx(1.2),
                                                  pytest.approx(2.6)]
    assert np.array_equal(p2.pose[:p1.n_knots], p1.pose)   # head bit-identical

    # The last cycle brings it to rest and RETIRES the deadline in the same call.
    assert r3.release_terminal is False
    assert r3.supersede_deadline_mono == 0.0
    assert uc.is_release_terminal(m3) is False
    assert uc.latest_supersede_time_s(m3) == math.inf
    assert [float(m.t_s) for m in m3.catches] == [
        pytest.approx(1.2), pytest.approx(2.6), pytest.approx(4.0)]
    assert np.array_equal(p3.pose[:p2.n_knots], p2.pose)


def test_the_reported_CATCH_is_the_one_still_AHEAD_not_a_spent_one(ring_stages):
    """`t_catch_mono` / `arm_lead_s` after an EXTEND name the NEXT touch-down.

    The mirror of `test_the_reported_release_is_the_one_still_AHEAD_not_a_spent_
    one`, and the same defect one field over. `meta.t_catch_s` and
    `meta.arm_lead_s` read `catches[0]`, and `extend` pins that to the FIRST catch
    on the joined clock permanently — so on a ring both fields reported a
    touch-down a beat in the past. `arm_lead_s` is not cosmetic: under unified
    `catch_coordinator` takes its stroke-busy window from the plan's own
    `stroke_clear_s`/`arm_lead_s` rather than the legacy firmware model, so a
    spent instant sizes that window around a catch that has already happened.

    Driven on `_accept_cycle` against the REAL three-catch ring meta at three
    plan times, with the clock frozen so the boundary cases are exact rather than
    approximately true.

    MEASURED (2026-09-07, /tmp/probe_uh7_ros.py, run twice with identical
    output): at tau 0.10 → catch 1.2, at 2.20 → 2.6, at 3.50 → 4.0.
    """
    plan, meta, _r = ring_stages[2]
    node = _cycle_node()
    with _frozen_perf() as at:
        def _reported(tau):
            t0 = at - float(tau)
            r = node._accept_cycle(PlanCycle.Response(), plan, meta, t0, at)
            return r.t_catch_mono - t0, r.arm_lead_s

        assert _reported(0.10)[0] == pytest.approx(1.2, abs=1e-9)
        assert _reported(2.20)[0] == pytest.approx(2.6, abs=1e-9)
        # Past every catch the LAST is reported, never 0.0 — a plan that caught
        # still caught, and "never" is a lie the consumer's `> now` test would
        # swallow silently.
        assert _reported(4.50)[0] == pytest.approx(4.0, abs=1e-9)
        # `arm_lead_s` follows the SAME mark rather than `catches[0]`.
        for tau, mark in ((0.10, meta.catches[0]), (2.20, meta.catches[1]),
                          (3.50, meta.catches[2])):
            assert _reported(tau)[1] == pytest.approx(float(mark.arm_lead_s))


def test_a_landing_update_on_a_RING_re_aims_the_SECOND_windows_catch(
        ring_stages):
    """The replan follows the LIVE catch, and the first window is left alone.

    On a LAUNCH + STEADY join `catches[0]` happens to BE the live catch, so the
    defect is invisible; it appears the moment a second STEADY is chained and the
    plan carries two. Before this, `_replan_cycle_from_target` read
    `meta.catch_site_mm` (i.e. `catches[0]`) and left `catch_frac = 0.0`, so
    `replan_tail` bounded the splice against the SPENT catch's knot and refused
    every landing update from the second chained window onward with *"the catch
    is inside the committed head"* — the replan policy dead code on the ring
    shape exactly as it was dead code on the rest-terminal shape before
    2026-09-05.

    Three claims: the SECOND catch moves, the FIRST window is bit-identical
    through the splice (which is what protects the launch release at knot 24 and
    its detach cone at 25-26 — a splice into that cone delivers 1.126 m/s² of
    off-axis force to a ball already in the air), and the plan stays
    release-terminal with its beat intact.

    MEASURED (2026-09-07, /tmp/probe_ring_knots.py, run twice with identical
    output) on this exact ring: 137 knots at dt 0.025, releases at knots 24, 80
    and 136, catches at 47 and 103; `tau = 2.10` with the node's own 0.30 s lead
    splices at knot 96 — clear of the second release's 80 + 2 detach cone and
    short of the live catch at 103.
    """
    node = _cycle_node()
    node._catch_armed = False
    plan, meta, _r = ring_stages[1]
    live = meta.catches[1]
    k_s = uc.splice_knot(meta, 2.10, tn._CYCLE_REPLAN_LEAD_S)
    assert meta.catches[0].knot < k_s < live.knot

    # The clock is frozen for the same reason the other same-origin re-install
    # tests freeze it: `_plan_cycle_replan` captures `tau` at handler ENTRY while
    # `_commanded_hand_state` samples the live plan on its own `perf_counter()`
    # call several hundred ms later, so an unfrozen run measures the SOLVE
    # (1.9 rev of slider travel through the carry, measured) as continuity drift
    # and refuses for a reason that is about the Jetson's clock, not the splice.
    with _frozen_perf():
        t0 = _install_ring(node, ring_stages[1], tau=2.10)
        msg = _dyn_msg(x=10.0)
        msg.arrival_time = t0 + float(live.t_s)
        node._on_dynamic_target(msg)
    fb = node._publishers['trajectory/target_feedback'].published[-1]
    assert fb.accepted is True, fb.reason
    assert fb.source == 'cycle'
    assert node._cycle_replans == 1

    spliced, meta2, t0_after = node._cycle
    assert t0_after == pytest.approx(t0)
    assert spliced is not plan
    # The whole first window — release, detach cone, catch — is carried bit for
    # bit, on all four channels.
    assert np.array_equal(spliced.pose[:k_s], plan.pose[:k_s])
    assert np.array_equal(spliced.pose_vel[:k_s], plan.pose_vel[:k_s])
    assert np.array_equal(spliced.hand_rev[:k_s], plan.hand_rev[:k_s])
    assert np.array_equal(spliced.hand_vel_rps[:k_s], plan.hand_vel_rps[:k_s])
    # The SECOND catch moved; the first is carried, and so are all three
    # releases — the beat the next EXTEND chains onto is untouched.
    assert len(meta2.catches) == 2
    assert meta2.catches[0].t_s == pytest.approx(float(meta.catches[0].t_s))
    assert np.allclose(meta2.catches[0].site_mm, meta.catches[0].site_mm)
    assert meta2.catches[1].t_s == pytest.approx(float(live.t_s))
    assert meta2.catches[1].site_mm[0] == pytest.approx(10.0, abs=0.1)
    assert [float(r.t_s) for r in meta2.releases] == [
        pytest.approx(0.6), pytest.approx(2.0), pytest.approx(3.4)]
    assert uc.is_release_terminal(meta2) is True


@pytest.mark.parametrize('arrival_tau,needle', [
    (1.2, 'inside the committed head'),      # the SPENT catch of window 1
    (2.0, 'some other ball'),                # a landing no catch of this plan makes
])
def test_a_landing_update_that_names_no_LIVE_catch_is_refused(
        ring_stages, arrival_tau, needle):
    """Being ahead of now is not enough to name a catch: the update must MATCH one.

    Two failures this closes, both specific to a ring and neither visible on a
    single-window plan:

    * **the spent ball.** After cycle k's touch-down the tracker keeps seeing
      ball k — now sitting in the cup and moving WITH it, so it clears
      `catch_coordinator`'s 5 mm movement gate and keeps publishing. Selecting
      "the first catch still ahead of now" would re-aim cycle k+1's catch onto
      wherever the LAST ball ended up, and the next ball would be caught at the
      previous ball's landing point. Selecting by TIME refuses it instead.
    * **a foreign or corrupt track.** Nothing else filters one under unified:
      `catch_coordinator`'s own announced-landing guard gates the HAND ARM, which
      unified never dispatches. An update whose predicted landing matches no
      catch this plan makes is not this cycle's ball.

    Refused at the routing layer, so neither costs the ~230 ms seven-channel
    solve on the single-threaded executor to say no, and the last good plan keeps
    streaming untouched.
    """
    node = _cycle_node()
    node._catch_armed = False
    plan, _meta, _r = ring_stages[1]
    node._svc_plan_cycle = lambda *a, **k: pytest.fail(
        'a landing update that names no live catch paid for a solve')

    # Frozen so `tau_now` is exactly 2.10 and the two boundary cases below are
    # decided by the rule rather than by how long the harness took to get here.
    with _frozen_perf():
        t0 = _install_ring(node, ring_stages[1], tau=2.10)
        msg = _dyn_msg(x=10.0)
        msg.arrival_time = t0 + float(arrival_tau)
        node._on_dynamic_target(msg)
    fb = node._publishers['trajectory/target_feedback'].published[-1]
    assert fb.accepted is False
    assert fb.code == tn._NO_LIVE_CATCH
    assert needle in fb.reason
    assert fb.source == 'cycle'
    assert node._cycle[0] is plan               # the last good plan still stands
    assert node._cycle_replans == 0


def test_the_match_band_must_stay_INSIDE_the_commit_lead():
    """The stale-ball refusal works BECAUSE the match band is under the lead.

    The coupling is easy to break by tuning either constant alone, and what
    breaks is silent and physical, so it is pinned rather than left as a comment.

    The stale ball: after cycle k's touch-down the tracker keeps seeing ball k,
    now sitting in the cup and moving WITH it, so it clears `catch_coordinator`'s
    5 mm movement gate and keeps publishing. Its arrival instant is close to
    catch k, so the match selects catch k — which is exactly what we want, because
    the SPENT test then refuses it. And that test refuses it precisely because
    `_CYCLE_CATCH_MATCH_S` (0.25) < `_CYCLE_REPLAN_LEAD_S` (0.30): a mark within
    the band of NOW is inside the commit lead by construction, so any catch a
    stale update can select is already spent.

    Raise the band above the lead and a stale ball's update selects a catch the
    spent test still calls live — and the cup for the NEXT ball gets dragged to
    wherever the LAST one ended up, which reads as a physics fault and is
    bookkeeping.
    """
    assert tn._CYCLE_CATCH_MATCH_S < tn._CYCLE_REPLAN_LEAD_S
    # ...and the consequence, driven rather than asserted about the constants: a
    # catch that has JUST passed cannot be selected, at any arrival within the
    # band of it.
    node = _cycle_node()
    with _frozen_perf() as at:
        mark = uc.CatchMark(t_s=0.50, knot=20,
                            site_mm=np.array([0.0, 0.0, 830.0]),
                            vel_mm_s=np.array([0.0, 0.0, -2500.0]))
        meta = types.SimpleNamespace(catches=(mark,), releases=('r',))
        # tau_now sits just past the catch: the ball is in the cup.
        t0 = at - (float(mark.t_s) + 0.01)
        for slack in (0.0, 0.1, -0.1, tn._CYCLE_CATCH_MATCH_S * 0.99):
            got, why = node._live_catch_mark(
                meta, t0, t0 + float(mark.t_s) + slack)
            assert got is None, slack
            assert 'inside the committed head' in why


def test_two_catches_inside_the_match_band_are_refused_not_guessed():
    """A tie is refused by name, because nearest-wins would be a coin flip.

    The band is sized to stay under half the shortest beat the planner can fly,
    so on every shape it can build today there is at most one candidate and this
    never fires. "Never fires on today's shapes" is not "cannot fire": the ring's
    whole purpose is to SHORTEN the beat, and a beat under twice the band walks
    straight into it. What it would become is a tie broken by float noise that
    re-aims one of two real catches — and the wrong one drags the cup away from a
    ball already in flight, with nothing in the log to say a choice was made.

    So both candidates are named and the update is refused. `AMBIGUOUS` leads the
    message so an operator greps one word to tell it from the two ordinary
    refusals.
    """
    node = _cycle_node()
    with _frozen_perf() as at:
        def _mark(t_s, knot):
            return uc.CatchMark(t_s=t_s, knot=knot,
                                site_mm=np.array([0.0, 0.0, 830.0]),
                                vel_mm_s=np.array([0.0, 0.0, -2500.0]))
        # Two catches 0.30 s apart — a beat well under 2 x the 0.25 s band.
        meta = types.SimpleNamespace(catches=(_mark(1.00, 40), _mark(1.30, 52)),
                                     releases=('r',))
        t0 = at                                  # tau_now = 0, both still live
        got, why = node._live_catch_mark(meta, t0, t0 + 1.15)
        assert got is None
        assert why.startswith('AMBIGUOUS')
        assert '1.000 s' in why and '1.300 s' in why
        # ...and it is not blanket paranoia: an arrival that matches exactly one
        # of them still selects, so the ordinary case is untouched.
        # 1.45 s is 0.15 from the second mark and 0.45 from the first.
        got, why = node._live_catch_mark(meta, t0, t0 + 1.45)
        assert got is not None and float(got.t_s) == 1.30, why


def test_the_extend_hands_back_the_replan_budget_ONCE_PER_WINDOW(ring_stages):
    """A ring extends every beat, so the budget is two replans PER BEAT.

    Deliberate, and the only reading that keeps the policy meaning what it says:
    the bound exists so ONE ball's landing is not chased without limit, and every
    extend brings a new window carrying a new catch for a NEW ball. A
    session-wide budget would spend itself on the first two balls and fly every
    remaining catch blind to the tracker; a TIME-based reset was tried and
    removed, because every landing update arrives after the release by
    construction, so the counter reset on every call and the bound never bound
    (`test_the_replan_budget_is_restored_by_AN_INSTALL_and_never_by_the_clock`).

    The reset is safe because an EXTEND cannot move a committed knot: the joined
    head is bit-identical and `_install_continuity_ok` re-checks it at the LIVE
    plan time, so the two replans the fresh budget buys can only reach the window
    this call just added.
    """
    node = _cycle_node()
    node._catch_armed = False
    _install_ring(node, ring_stages[0], tau=0.0)
    head_plan = node._cycle[0]
    n_head = int(head_plan.n_knots)
    node._cycle_replans = tn._MAX_CYCLE_REPLANS
    assert node._cycle_replan_budget_left() == 0

    resp, _origin = _extend_alive(node, _steady_extend_req())
    assert resp.accepted is True, resp.message
    assert resp.replans_used == 0
    assert node._cycle_replans == 0
    assert node._cycle_replan_budget_left() == tn._MAX_CYCLE_REPLANS
    # ...and everything the spent budget had already committed is still there.
    assert np.array_equal(node._cycle[0].pose[:n_head], head_plan.pose)
    assert np.array_equal(node._cycle[0].hand_rev[:n_head], head_plan.hand_rev)


def test_a_LATE_extend_is_refused_STALE_STATE_and_the_last_plan_keeps_streaming():
    """The failure a ring meets every beat if the extend lead is wrong.

    `_UNIFIED_EXTEND_LEAD_S` exists to ask for the next window BEFORE the release,
    not after. When it is asked for too late the emitter has already run off the
    end of the head: the live plan is clamped to its terminal HOLD (zero twist,
    zero hand rate) while the joined plan at the same `tau` is well inside the NEW
    window at full speed, so installing it would jump the machine forward by
    however long the solve took — on seven channels, as a step. The continuity
    guard refuses it and the last good plan keeps streaming.

    Until now the suite RETRIED past this: `_extend_alive` re-anchors and tries
    again (up to four times), and its docstring says the refusal is correct — but
    nothing asserted it, and nothing asserted the last good plan survives it. So
    this drives `tau` past the seam DELIBERATELY, through the real service, with
    no retry.

    MEASURED (2026-09-07, /tmp/probe_uh7_ros.py, run twice with identical output)
    on the 0.6 s LAUNCH at three taus past its end: refused STALE_STATE every
    time, on the HAND position term (3.20-4.24 rev of drift against the 1.00 rev
    bound) — the legs alone would not have caught it, which is the whole reason
    that term exists.
    """
    node = _cycle_node()
    node._catch_armed = False
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    plan, meta, _t0 = node._cycle
    assert float(plan.total_duration) == pytest.approx(0.6)

    for tau in (0.65, 0.80, 1.20):
        _refresh(node)
        past = time.perf_counter() - tau
        node._cycle = (plan, meta, past)
        node._plan_t0 = past
        resp = node._svc_plan_cycle(_landing_req(), PlanCycle.Response())
        assert resp.accepted is False, resp.message
        assert resp.code == feas.STALE_STATE
        assert 'not continuous at the live plan time' in resp.message
        assert 'holding the last good plan' in resp.message
        # The LAST GOOD PLAN keeps streaming: same object on the emitter, same
        # cycle record, same origin. A refusal that quietly installed anything
        # would be worse than the cliff it was refusing.
        assert node._active_plan is plan
        assert node._cycle[0] is plan
        assert node._cycle[1] is meta
        assert node._cycle[2] == pytest.approx(past)


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

    `arrival_time` is read off the installed plan (`_live_catch_perf`), which is
    what the coordinator publishes: the ball's own predicted landing, i.e. the
    instant the plan committed to catch it. That is the field the node now uses
    to decide WHICH catch of a chained plan is being re-aimed.
    """
    _anchor_mid_flight(_refresh(node))
    msg = _dyn_msg(x=x_mm, y=y_mm)
    msg.arrival_time = _live_catch_perf(node)
    node._on_dynamic_target(msg)
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

    **The clock is frozen, and that is the difference between testing this code
    and testing the Jetson's scheduler.** The comparison below reads the plan at
    a `tau` captured on one line and `_current_state()` — which captures its own
    `perf_counter()` — on the next, so any preemption between the two shows up as
    velocity drift: this test failed once inside a full `tests/ros/` run on
    2026-09-06 and passed in isolation, which is the signature. Frozen, both
    sides are sampled at one instant and the claim reverts to the one that was
    always intended: the EXTEND kept the ORIGIN, so a re-install measures no
    drift. (If it re-anchored, `_plan_t0` would differ from `origin` and the
    difference would be the whole elapsed window, frozen clock or not.)
    """
    node = _cycle_node()
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    resp, origin = _extend_alive(node, _landing_req())
    assert resp.accepted is True, resp.message
    joined = node._cycle[0]
    with _frozen_perf() as now:
        tau = now - origin
        pose, twist, _ = joined.state_at(tau)
        live_pose, live_twist, _ = node._current_state()
        assert float(np.max(np.abs(
            node._pose_twist_to_motor_rev_s(pose, twist)
            - node._pose_twist_to_motor_rev_s(live_pose, live_twist)))) < 1e-6
        # ...and through the guard itself, which is what the service calls.
        assert node._install_continuity_ok(joined, tau) is True
        assert node._continuity_detail == ''

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
    # The tracker's predicted landing for THIS ball, which is what selects the
    # catch mark being re-aimed. Read off the installed plan rather than invented
    # as `now + 0.5`: the coordinator publishes the ball's own predicted landing,
    # and a made-up instant more than `_CYCLE_CATCH_MATCH_S` from any catch the
    # plan makes is refused NO_LIVE_CATCH — correctly, and for a reason that has
    # nothing to do with the routing under test.
    msg.arrival_time = _live_catch_perf(node)
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


def _live_catch_perf(node):
    """The perf instant of the first catch on the installed plan still AHEAD.

    What the tracker's `arrival_time` carries on the real graph: its own
    predicted landing for the ball in flight, which the plan committed to catch.
    `_replan_cycle_from_target` selects the catch mark NEAREST that instant, so a
    harness that invents one unrelated to the plan gets `NO_LIVE_CATCH` — the
    correct answer to a made-up question, and never the wiring under test.
    """
    _plan, meta, t0 = node._cycle
    tau_now = time.perf_counter() - float(t0)
    for mark in meta.catches:
        if float(mark.t_s) > tau_now:
            return float(t0) + float(mark.t_s)
    raise AssertionError('the installed cycle has no catch left ahead of now')


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


def _tilted_dyn_msg(rx=0.0, ry=0.0, x=0.0, y=0.0, z=170.0, lead_s=0.5):
    """A ``catch/dynamic_target`` carrying a real receive tilt.

    The quaternion is built from the rotation vector the same way
    ``catch_coordinator.compute_catch_orientation`` builds it, so the axis the
    replan recovers is the axis the coordinator compensated with.
    """
    from jugglebot.motion.ik_solver import rot_matrix_to_quat, rotvec_to_rot_matrix
    q = rot_matrix_to_quat(
        rotvec_to_rot_matrix(np.array([float(rx), float(ry), 0.0])))
    msg = DynamicTargetCommand()
    msg.target_pos = Point(x=float(x), y=float(y), z=float(z))
    msg.target_quat = Quaternion(w=float(q[0]), x=float(q[1]), y=float(q[2]),
                                 z=float(q[3]))
    msg.target_vel = Vector3()
    msg.arrival_time = time.perf_counter() + float(lead_s)
    return msg


def test_the_replan_converts_the_wire_CENTROID_into_the_planners_CUP_site(
        monkeypatch):
    """`catch_site_mm` is a CUP site; `target_pos` is the platform CENTROID.

    `catch_coordinator._compute_catch_command` publishes
    ``landing − HAND_CATCH_OFFSET_MM · platform_z``, and `build_catch` consumes
    that centroid verbatim — that is what pins the frame. Copying `target_pos`
    straight into `catch_site_mm` therefore aimed the cycle's catch at a point
    the offset SHORT of the ball, leaning the way the receive tilt leans: a
    systematic ``64.78·sin θ`` mm bias, 13.47 mm at the 12° ceiling and largest
    exactly when the ball is arriving fastest sideways.

    Pinned at the ceiling on both axes (the sign convention is load-bearing:
    ``+ry`` swings the cup toward ``+x``, ``+rx`` toward ``−y``) and at level,
    where the conversion must be an exact no-op.
    """
    sent = []

    def _capture(node):
        monkeypatch.setattr(
            node, '_svc_plan_cycle',
            lambda req, resp: sent.append(req) or _accepted(resp))

    def _accepted(resp):
        resp.accepted, resp.code, resp.message = True, 'OK', ''
        return resp

    # A sentinel cycle: `_replan_cycle_from_target` reads only the meta's catch
    # MARKS and the plan's duration, and `_svc_plan_cycle` is captured below, so
    # nothing here needs a real ~250 ms solve to pin a frame conversion. The clock
    # is frozen so the mark's liveness (`t_s > tau_now + lead`) and the message's
    # arrival instant are exact rather than approximately true — the selection is
    # by TIME now, and a scheduler stall between the origin and the call would
    # otherwise read as a spent catch.
    node = _cycle_node()
    theta = math.radians(12.0)
    _T_CATCH = 0.77
    with _frozen_perf() as at:
        node._cycle = (types.SimpleNamespace(total_duration=1.4),
                       types.SimpleNamespace(
                           catches=(uc.CatchMark(
                               t_s=_T_CATCH, knot=30,
                               site_mm=np.array([0.0, 0.0, 830.0]),
                               vel_mm_s=np.array([0.0, 0.0, -2500.0])),),
                           releases=('r',)),
                       at)
        _capture(node)

        node._replan_cycle_from_target(
            _tilted_dyn_msg(ry=theta, x=100.0, y=-50.0, lead_s=_T_CATCH), node._cycle)
        assert sent[-1].catch_site_mm[0] == pytest.approx(100.0 + 13.4685,
                                                          abs=1e-3)
        assert sent[-1].catch_site_mm[1] == pytest.approx(-50.0, abs=1e-9)
        # z is NEVER touched: a centroid says nothing about the cup height, which
        # the slider the plan is about to choose owns.
        assert sent[-1].catch_site_mm[2] == pytest.approx(830.0)

        node._replan_cycle_from_target(
            _tilted_dyn_msg(rx=theta, x=100.0, y=-50.0, lead_s=_T_CATCH), node._cycle)
        assert sent[-1].catch_site_mm[0] == pytest.approx(100.0, abs=1e-9)
        assert sent[-1].catch_site_mm[1] == pytest.approx(-50.0 - 13.4685,
                                                          abs=1e-3)

        # LEVEL: an exact no-op, so a vertical self-toss's replan is
        # bit-identical to what it was before this conversion existed.
        node._replan_cycle_from_target(
            _tilted_dyn_msg(x=100.0, y=-50.0, lead_s=_T_CATCH), node._cycle)
        assert sent[-1].catch_site_mm[0] == 100.0
        assert sent[-1].catch_site_mm[1] == -50.0
        # The catch INSTANT is nominated, unchanged, as a fraction of the whole
        # plan's duration — that is how the planner is told WHICH catch this is.
        assert sent[-1].catch_frac * sent[-1].period_s == pytest.approx(_T_CATCH)


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
    """A started sequencer plus the node's committed state, unified or not.

    The hand seed is part of the fixture and not an incidental default. Since
    2026-09-07 a unified LAUNCH is refused outright when the seed hand sits
    below the planner's cup floor (``HAND_BELOW_FLOOR``) or when
    ``/hand_telemetry`` is stale enough that the seed is UNKNOWN — the two
    states in which a LAUNCH planned from that seed slams the cup into the box
    on its first knot. ``_ready_node`` leaves the hand at 0.0 rev, the homed
    zero, which is genuinely ~10 mm BELOW the floor, and stamps freshness on
    the FAKE clock, which ``_unified_hand_seed_rev`` (a real ``perf_counter``
    reader) cannot see. So a unified fixture that skipped this would exercise
    the refusal on every one of these seam tests instead of the seam, and each
    would be red for a reason that has nothing to do with what it asserts.
    Seating the seed AT the floor and stamping it on the real clock is what a
    machine parked ready actually looks like; the refusal itself is covered by
    ``tests/ros/test_unified_launch_floor.py``.
    """
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=0.8, throw_delay_s=5.0, unified=unified)
    seq.start(time.perf_counter())
    seq._prepare_dispatched = True
    state = node._toss_committed
    node._toss_unified_live = unified
    if unified:
        with node._lock:
            node._hand_pos_meas = uc.hand_rev_for_cup_z(uc.SETTLE_CUP_Z_MM)
            node._hand_telemetry_mono = time.perf_counter()
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
    # `_call_plan_cycle` returns (response, dispatched) since 2026-09-06 — the
    # second half is what separates "never sent" from "sent and unacked", and the
    # UNAVAILABLE outcome this test asserts is the FALSE branch.
    monkeypatch.setattr(node, '_call_plan_cycle',
                        lambda req, **kw: calls.append(req) or (None, False))
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


_SITTING_VEL_TRIM = -0.1076        # the ILC's independent fit, 2026-09-06


def _launch_request_with_trim(monkeypatch, trim):
    """The ``PlanCycle.Request`` a unified LAUNCH builds at ``trim``."""
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    state.unified_launch_pending = True
    state.aim = {'ilc_vel_trim': float(trim)}
    sent = []
    monkeypatch.setattr(node, '_call_plan_cycle',
                        lambda req, **kw: sent.append(req) or (None, False))
    node._tick_unified_launch(seq, state,
                              seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert len(sent) == 1
    return sent[0]


def test_the_unified_launch_carries_the_SESSION_SPEED_TRIM(monkeypatch):
    """The unified launch scales its take-off by the trim the legacy path carries.

    Owner, 2026-09-06. Every unified throw of that sitting left 11-15 % too fast
    (achieved release +5…+17 % of plan) while the record declared
    ``speed_bias_applied 1.0`` and ``ilc_vel_trim 0.0`` — every learned
    correction inactive, because the trim was applied to ``event_vel_mps``, which
    the unified path does not command.

    It is a MODEL correction: the machine throws faster than it is told to, so
    the COMMANDED release is scaled by ``1 + trim`` to get the physical one that
    was asked for. ``flight_s`` and the throw SITE are untouched — the ball is
    still meant to fly the requested flight and land where it left — and the
    scale reaches the QP through the only field that carries the release
    velocity: ``throw_target_mm``, from which
    ``cup_cycle.takeoff_velocity`` derives it.
    """
    from jugglebot.motion.trajectory import ballistics_bc as bb
    nominal = _launch_request_with_trim(monkeypatch, 0.0)
    trimmed = _launch_request_with_trim(monkeypatch, _SITTING_VEL_TRIM)

    # Untouched: the flight and the site the ball leaves from.
    assert trimmed.flight_s == nominal.flight_s
    assert list(trimmed.throw_site_mm) == list(nominal.throw_site_mm)
    assert list(trimmed.catch_site_mm) == list(nominal.catch_site_mm)
    assert list(trimmed.catch_vel_mm_s) == list(nominal.catch_vel_mm_s)

    v_nom = bb.launch_velocity(nominal.throw_site_mm, nominal.throw_target_mm,
                               nominal.flight_s)
    v_cmd = bb.launch_velocity(trimmed.throw_site_mm, trimmed.throw_target_mm,
                               trimmed.flight_s)
    assert v_cmd[2] / v_nom[2] == pytest.approx(1.0 + _SITTING_VEL_TRIM)
    assert v_cmd[2] / v_nom[2] == pytest.approx(0.8924)
    # A pure MAGNITUDE knob: scaling v scales both its components, so the take-off
    # direction — and the throw tilt the planner derives from it — is unchanged.
    assert np.linalg.norm(np.cross(v_cmd, v_nom)) == pytest.approx(0.0, abs=1e-9)


def test_trim_zero_leaves_the_unified_request_BIT_IDENTICAL(monkeypatch):
    """An un-tuned session is the machine it was, to the last float.

    The trim's round trip (`launch_velocity` then `position_at`) is exact at
    ``1 + 0.0``, but it is guarded on ``!= 0.0`` anyway so the untrimmed request
    is not merely equal but built by the same expression as before.
    """
    nominal = _launch_request_with_trim(monkeypatch, 0.0)
    assert list(nominal.throw_target_mm) == list(nominal.throw_site_mm)


def test_the_announcement_is_INVARIANT_under_the_speed_trim(monkeypatch):
    """The announcement describes the BALL, so it un-trims the commanded release.

    The plan's `release_vel_mm_s` is the COMMANDED velocity, which under a trim
    is deliberately `1 + trim` times the nominal. The trim's premise is that the
    machine then achieves the nominal — and the announcement feeds the tracker's
    correlation, the possession plausibility reference and the landing schedule,
    all of which are about the ball. Announcing the commanded value would put the
    predicted landing ~190 mm low at a 0.6 s flight.
    """
    def _announced(trim):
        node = _ready_node(_Clock())
        seq, state = _seq_state(node, unified=True)
        state.aim = {'ilc_vel_trim': float(trim)}
        commanded = 3900.0 * (1.0 + float(trim))
        resp = types.SimpleNamespace(
            t_release_mono=time.perf_counter() + 1.0,
            release_vel_mm_s=[0.0, 0.0, commanded], plan_wall_ms=175.0)
        node._announce_unified(seq, state, resp)
        return node._publishers['throw_announcements'].published[-1]

    plain, trimmed = _announced(0.0), _announced(_SITTING_VEL_TRIM)
    assert trimmed.initial_velocity.z == pytest.approx(
        plain.initial_velocity.z, rel=1e-12)
    assert trimmed.landing_position.z == pytest.approx(
        plain.landing_position.z, rel=1e-12)


def test_a_REFUSED_trim_is_not_flown_by_the_unified_launch(monkeypatch):
    """Layer 3 is a refinement, never a gate — on this path too.

    `_ilc_vel_trim_refusal` zeroes the aim block's `ilc_vel_trim` when the
    trimmed speed would break the throw envelope, and the record then declares
    0.0 because that is what was commanded. Reading the block (rather than
    re-deriving the trim) is what keeps the unified launch on the same side of
    that gate as the record.
    """
    req = _launch_request_with_trim(monkeypatch, 0.0)
    assert list(req.throw_target_mm) == list(req.throw_site_mm)
    node = _ready_node(_Clock())
    assert node._unified_vel_trim(types.SimpleNamespace(aim=None)) == 0.0
    assert node._unified_vel_trim(None) == 0.0
    assert node._unified_vel_trim(
        types.SimpleNamespace(aim={'ilc_vel_trim': -0.05})) == -0.05


def test_the_record_names_the_UNIFIED_catch_knobs(monkeypatch):
    """A corpus that pools two catch tunings is a corpus of two machines.

    Under unified no `HAND_TRAJ_CMD` is dispatched at all, so the Teensy
    catcher's `catch_vel_ratio` / `catch_vel_hold_pct` and the operator's
    `catch_vel_scale` command NOTHING — the 2026-09-06 rows declared a 0.6 ratio
    and a 0.9 scale no stroke ever used. The knob that DID shape every catch is
    the planner's `catch_slider_vel_ratio`, and it appeared nowhere.
    """
    from jugglebot.motion.trajectory import cup_cycle as cc
    node = _ready_node(_Clock())
    legacy = node._toss_record_catch_knobs(0.9, unified=False)
    unified = node._toss_record_catch_knobs(0.9, unified=True)
    assert legacy['catch_vel_scale'] == 0.9
    assert legacy['catch_vel_ratio'] == pytest.approx(
        float(hw.TEENSY_TRAJ_CATCH_VEL_RATIO))
    assert legacy['catch_slider_vel_ratio'] is None
    for dead in ('catch_vel_scale', 'catch_vel_ratio', 'catch_vel_hold_pct'):
        assert unified[dead] is None, dead
    assert unified['catch_slider_vel_ratio'] == pytest.approx(
        float(cc.CupCycleConfig().catch_slider_vel_ratio))
    assert unified['catch_slider_vel_ratio'] == pytest.approx(0.7)
    # The knobs that DO still bind under unified are unchanged.
    for shared in ('catch_reach_freeze_s', 'catch_reach_envelope_mm',
                   'hand_pos_gain'):
        assert unified[shared] == legacy[shared], shared


class _StubHoldClient:
    """A `trajectory/hold` client that records the calls and answers at once."""

    def __init__(self, ready=True, success=True, message='held'):
        self.ready, self.success, self.message = ready, success, message
        self.calls = []

    def wait_for_service(self, timeout_sec=None):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        return _ImmediateFuture(
            types.SimpleNamespace(success=self.success, message=self.message))


class _ImmediateFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        cb(self)


def test_an_UNACKED_plan_call_HOLDS_the_machine_and_says_a_plan_may_be_running(
        monkeypatch):
    """A slow solve throws a ball the coordinator believes it never commanded.

    `trajectory/plan_cycle` INSTALLS on accept and starts streaming BEFORE it
    replies, and a solve on a loaded box was measured at 2.02-2.16 s on
    2026-09-06 — past this node's 2.0 s client wait. Until then both halves of a
    service failure returned a bare None and the LAUNCH logged *"no plan
    installed, nothing was commanded"*, which is exactly false in the half that
    matters: the plan is installed, it is streaming, and it will throw.

    So a DISPATCHED-and-unacked call holds the machine FIRST — the same
    `trajectory/hold` decel-to-rest the bench driver aborts with — and mints an
    outcome that carries the hold's verdict rather than a denial.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    state.unified_launch_pending = True
    hold = _StubHoldClient()
    node._traj_hold_cli = hold
    monkeypatch.setattr(node, '_call_plan_cycle', lambda req, **kw: (None, True))
    node._tick_unified_launch(seq, state,
                              seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert len(hold.calls) == 1, 'the machine was not held'
    assert state.unified_reject.startswith('REJECTED_PLAN_SERVICE(UNACKED:')
    assert 'MAY' in state.unified_reject
    assert 'held' in state.unified_reject
    # ...and the OTHER half is unchanged: nothing dispatched ⇒ nothing to hold.
    node2 = _ready_node(_Clock())
    seq2, state2 = _seq_state(node2, unified=True)
    state2.unified_launch_pending = True
    hold2 = _StubHoldClient()
    node2._traj_hold_cli = hold2
    monkeypatch.setattr(node2, '_call_plan_cycle', lambda req, **kw: (None, False))
    node2._tick_unified_launch(seq2, state2,
                               seq2.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert hold2.calls == []
    assert state2.unified_reject.startswith('REJECTED_PLAN_SERVICE(UNAVAILABLE:')


def test_a_hold_that_did_not_land_is_REPORTED_not_swallowed(monkeypatch):
    """"I could not stop it" is the one fact the operator must not have to hunt.

    A refused or unavailable hold leaves a plan streaming toward a release, and
    an outcome that says only "unacked" sends the operator to the console to
    discover that. The verdict rides in the outcome string itself.
    """
    node = _ready_node(_Clock())
    seq, state = _seq_state(node, unified=True)
    state.unified_launch_pending = True
    node._traj_hold_cli = _StubHoldClient(ready=False)
    monkeypatch.setattr(node, '_call_plan_cycle', lambda req, **kw: (None, True))
    node._tick_unified_launch(seq, state,
                              seq.t_release - rcn._UNIFIED_LAUNCH_LEAD_S + 0.01)
    assert 'STILL STREAMING' in state.unified_reject


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


def _unified_ball_node(monkeypatch, cmd_fn):
    """A catch_coordinator in the state a unified session actually puts it in.

    ALL THREE flags raised, which is the thing the pre-2026-09-06 tests never
    did: `catch/armed` (the session's one raise), `catch/pretilt_hold` (raised
    for the whole session at `_arm_session_declare`) and `catch/unified_mode`.
    With only the first and third raised the open-loop branch is not reached at
    all, so a test written that way passes whatever the branch does — which is
    why the shipped gate carried zero `catch/dynamic_target` messages through a
    whole sitting and every test stayed green.
    """
    ccn = CatchCoordinatorNode()
    ccn._on_catch_armed(Bool(data=True))
    ccn._on_pretilt_hold(Bool(data=True))
    ccn._on_unified_mode(Bool(data=True))
    ccn._announcement_seen = True                # the announcement has landed
    monkeypatch.setattr(ccn._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None: cmd_fn())
    return ccn


def test_a_unified_ball_tick_PUBLISHES_a_dynamic_target(monkeypatch):
    """The unified catch re-aims off the tracker; the open-loop freeze is legacy.

    Under unified BOTH open-loop triggers stand — `pretilt_hold` for the whole
    session and `JB_OP_RELOAD_PLATFORM_OPEN_LOOP` by config — so every ball tick
    took the open-loop branch, where `_pretilt_cmd` is None and
    `_republish_pretilt` no-ops. The 2026-09-06 bag carried ZERO
    `/catch/dynamic_target` messages, and the toss coordinator's own
    `_publish_toss_reach` (which covers this on the legacy path) is deliberately
    OFF under unified — so nothing at all was re-aiming the catch.

    trajectory_node's consumer was already correct: it bypasses `_catch_armed`
    under unified and routes to `_replan_cycle_from_target`, which re-solves only
    the catch-side tail of the 7-channel plan.
    """
    from tests.ros.test_catch_coordinator_node import _balls_msg, _catchable_cmd
    ccn = _unified_ball_node(monkeypatch, _catchable_cmd)
    n0 = len(ccn._dyn_target_pub.published)
    ccn._on_balls(_balls_msg())
    assert len(ccn._dyn_target_pub.published) == n0 + 1

    # LEGACY, same three flags minus unified_mode: the open-loop branch, and
    # `_pretilt_cmd` is None under pretilt_hold, so NOTHING is published. Pinned
    # as the reference so the change above cannot silently widen to legacy.
    legacy = CatchCoordinatorNode()
    legacy._on_catch_armed(Bool(data=True))
    legacy._on_pretilt_hold(Bool(data=True))
    legacy._announcement_seen = True
    monkeypatch.setattr(legacy._coordinator, 'update',
                        lambda balls, current_time, exclude_ids=None:
                        _catchable_cmd())
    m0 = len(legacy._dyn_target_pub.published)
    legacy._on_balls(_balls_msg())
    assert len(legacy._dyn_target_pub.published) == m0


def test_the_unified_replan_is_rate_limited_by_LANDING_MOVEMENT(monkeypatch):
    """Every mocap tick must not buy a ~230 ms seven-channel re-solve.

    The balls topic ticks at mocap rate; each accepted target costs
    trajectory_node a `replan_tail` on its single-threaded executor. Unfiltered,
    that is the shape that gapped the emitter past the Teensy's 250 ms setpoint
    watchdog on 2026-09-06. The gate is on MOVEMENT, not on a timer, so a track
    that keeps re-predicting the same point costs nothing while one that is
    genuinely revising gets every revision through.
    """
    from tests.ros.test_catch_coordinator_node import _balls_msg, _catchable_cmd
    pos = {'v': np.array([0.0, 0.0, 809.08])}

    def _cmd():
        cmd = _catchable_cmd()
        cmd.target_pos = pos['v'].copy()
        return cmd

    ccn = _unified_ball_node(monkeypatch, _cmd)
    ccn._on_balls(_balls_msg())                       # the FIRST is never filtered
    assert len(ccn._dyn_target_pub.published) == 1
    # 4 mm: under the bar, no solve.
    pos['v'] = pos['v'] + np.array([4.0, 0.0, 0.0])
    ccn._on_balls(_balls_msg())
    assert len(ccn._dyn_target_pub.published) == 1
    # ...and the reference is what was PUBLISHED, not what was last seen: another
    # 4 mm in the same direction is 8 mm from the published target and DOES go.
    pos['v'] = pos['v'] + np.array([4.0, 0.0, 0.0])
    ccn._on_balls(_balls_msg())
    assert len(ccn._dyn_target_pub.published) == 2
    # A NEW ball is never filtered by the previous ball's last target.
    ccn._on_balls(_balls_msg())
    assert len(ccn._dyn_target_pub.published) == 2
    same_pos = pos['v'].copy()
    monkeypatch.setattr(
        ccn._coordinator, 'update',
        lambda balls, current_time, exclude_ids=None:
        types.SimpleNamespace(**dict(vars(_catchable_cmd(ball_id=6)),
                                     target_pos=same_pos)))
    ccn._on_balls(_balls_msg())
    assert len(ccn._dyn_target_pub.published) == 3


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
    # The extend is NON-BLOCKING (UH-7a): it dispatches through the client and
    # polls the future on later ticks, so "what did it ask for" is a question
    # about `call_async`, not about `_call_plan_cycle`. A future that never
    # resolves is what a dispatched-and-unanswered request looks like, and it
    # keeps this test on the DISPATCH, which is all it asserts.
    sent = []
    monkeypatch.setattr(node, '_plan_cycle_cli', types.SimpleNamespace(
        service_is_ready=lambda: True,
        wait_for_service=lambda timeout_sec=None: True,
        call_async=lambda req: (sent.append(req),
                                types.SimpleNamespace(done=lambda: False))[1]))
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
        lambda req, **kw: pytest.fail('a chained release was re-planned'))
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
        lambda req, **kw: pytest.fail('a skewed chain fell through to a NEW launch'))
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
    node._call_plan_cycle = lambda req, **kw: sent.append(req) or (None, False)
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
    # UH-7a: the ask is a `call_async` on the client, not a blocking
    # `_call_plan_cycle`. What this test measures — WHEN the request goes out
    # relative to the published deadline — is unchanged by that, and the future
    # deliberately never resolves so the trigger is all that is exercised.
    fired_at = []
    monkeypatch.setattr(node, '_plan_cycle_cli', types.SimpleNamespace(
        service_is_ready=lambda: True,
        wait_for_service=lambda timeout_sec=None: True,
        call_async=lambda req: (fired_at.append(req),
                                types.SimpleNamespace(done=lambda: False))[1]))
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
