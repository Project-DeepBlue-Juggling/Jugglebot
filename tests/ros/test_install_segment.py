"""``trajectory/install_segment`` — the skill stack's ONE install path (R2, Unit D2).

Mirrors ``test_unified_cycle_integration.py``'s service tests in shape (a real
``TrajectoryNode``, ROS mocked by ``tests/ros/conftest.py``) but drives the new
service directly. The planning/splice-vs-fresh-origin logic itself is
``jugglebot.motion.skills.executor.install_segment`` (pure Python) and is
covered by ``tests/motion/test_skills_executor.py`` (Unit B) — what this file
pins is the ROS SHELL: guard ladder, epoch/hold interaction, the ROS-clock
crossing, and that a refusal never touches the active plan.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from std_srvs.srv import Trigger

from jugglebot_interfaces.srv import InstallSegment
import jugglebot.hardware_config as hw
from jugglebot import trajectory_node as tn
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.skills import executor as sk_exec
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory.cycle_plan import CyclePlan

from tests.ros.test_unified_cycle_integration import _cycle_node, _frozen_perf, _refresh
from tests.ros.test_trajectory_node import _link_status


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

        tau = release_rel - lead_s
        origin = at - tau
        node._cycle = (plan1, meta1, origin)
        node._plan_t0 = origin
        _refresh(node)

        t_origin = origin + release_rel  # == at, by construction
        t_land = t_origin + 0.4
        t_release = t_origin + 0.9
        resp2 = node._svc_install_segment(
            _catch_throw_req(t_land, t_release), InstallSegment.Response())
    assert resp2.accepted is True, resp2.message
    assert resp2.code == feas.OK
    k_rel = int(round(release_rel / dt))
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


def test_splice_too_late_when_the_solve_runs_past_the_wire(monkeypatch):
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

        monkeypatch.setattr(time, 'perf_counter', _fake_perf_counter)
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
