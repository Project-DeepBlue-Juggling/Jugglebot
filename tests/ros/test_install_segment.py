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
import time

import numpy as np
import pytest

from std_srvs.srv import Trigger

from jugglebot_interfaces.msg import MotorStateSingle, RobotState
from jugglebot_interfaces.srv import InstallSegment
import jugglebot.hardware_config as hw
from jugglebot import trajectory_node as tn
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.skills import executor as sk_exec
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory.cycle_plan import CyclePlan

from tests.ros.test_trajectory_node import _link_status


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

# Session limits a unified sitting raises to at start (plan Phase 1, owner
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
    """A seeded, TRAJECTORY-mode node at session limits, emitter NOT started."""
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
    node = _perf_node()
    with _frozen_perf() as at:
        resp = node._svc_install_segment(_throw_req(at + 0.6),
                                         InstallSegment.Response())
    assert resp.accepted is True, resp.message
    assert resp.code == feas.OK
    assert resp.splice_k == 0
    assert resp.t0_mono == pytest.approx(at)
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
    is asserted absent for the same reason."""
    node = _reconcile_node(commanded_hand_rev=0.5639, measured_hand_rev=0.0001)
    node._latest_hand_vel_rps = 5.0        # source (2)/(3)'s rate
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
    req.rest_tilt_rad = [0.0, 0.0]
    terminal = node._segment_terminal_from_request('REST', req, 10.0)
    assert terminal.tilt == (0.0, 0.0)
    assert terminal.tilt is not None


def test_a_nonzero_hold_tilt_rad_round_trips_onto_the_catch_terminal():
    node = _perf_node()
    req = _catch_req(10.0)
    req.hold_tilt_rad = [0.01, -0.20943951023931956]
    terminal = node._segment_terminal_from_request('CATCH', req, 10.0)
    assert terminal.hold_tilt == pytest.approx((0.01, -0.20943951023931956))


# ═════════════════════════════════════════════════════════════════════════════
# T-I1 — the real :5557 seam, end to end (ported from the deleted
# test_unified_cycle_integration.py at the FSM deletion, R4 unit U6b cluster B,
# 2026-09-24 — re-driven through install_segment instead of the retired
# trajectory/plan_cycle; same fixtures, same asserts)
# ═════════════════════════════════════════════════════════════════════════════

def test_TI1_seven_channel_frames_reach_the_wire_and_the_flags_fall():
    """T-I1: trajectory_node -> real :5557 -> teensy_bridge_node -> loopback UDP.

    Every hop is production code: the real ``MpcCommandPub`` (ephemeral port),
    the real ``_MpcCommandSetpointSource`` decoder, the real ``SetpointPump``,
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
    from jugglebot.motion.ipc import MpcCommandPub
    from jugglebot.teensy_bridge_node import _MpcCommandSetpointSource
    from teensy_link.setpoint_pump import (
        FLAG_HAS_HAND, FLAG_HAS_SCHED, FLAG_HAS_V1)
    from tests.ros._bridge_harness import _build_paired_node, _teardown

    pub = MpcCommandPub(addr='tcp://127.0.0.1:0')
    addr = pub._pub.getsockopt_string(zmq.LAST_ENDPOINT)
    traj = _perf_node(command_pub_factory=lambda: pub)
    traj._pub = pub                       # emitter thread not started; wire by hand
    teensy, client, bridge = _build_paired_node()
    src = _MpcCommandSetpointSource(addr=addr)
    try:
        at = time.perf_counter()
        resp = traj._svc_install_segment(_throw_req(at + 0.6),
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
