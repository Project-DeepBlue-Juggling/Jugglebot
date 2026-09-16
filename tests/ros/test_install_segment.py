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
    from tests.ros.test_unified_cycle_integration import _robot_state
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
