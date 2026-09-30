"""skill_node — the mocked-ROS shell smoke test (R2, Unit D2).

The pure-Python schedule/segment/executor policy is pinned by
``tests/motion/test_skills_*.py``; ``trajectory/install_segment`` itself is
pinned by ``tests/ros/test_install_segment.py``. What this file exercises is
the SHELL: a `columns` Juggle goal compiles a schedule, the 40 Hz tick
dispatches the first due skill through the (mocked) install_segment client
with the right request fields, a refusal ends the attempt, and a ``/balls``
message feeds the executor's tracker callable.
"""

from __future__ import annotations

import math
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from tests.ros.conftest import MockTime, _MockParameter

import rclpy
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as _MockNodeClass

from std_msgs.msg import String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Vector3
from jugglebot_interfaces.msg import (BallButlerHeartbeat, BallStateArray,
                                      HandTelemetryMessage, RigidBodyPose,
                                      RigidBodyPoses, ThrowAnnouncement,
                                      TrajectoryStatus)
from jugglebot_interfaces.action import Juggle
from jugglebot_interfaces.srv import BallButlerThrow, InstallSegment

from jugglebot import ball_possession
from jugglebot import skill_node as sn
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills.executor import CAUGHT_WINDOW_S
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills.memory import Memory, memory_path
from jugglebot.motion.trajectory import tilt_geometry as tg

from tests.ros.conftest import MockFuture


class _Vec:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _Time:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


def _ball(id, status=0, destination='', tracking=0,
         x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0,
         sec=0, nanosec=0, landing_from_fit=True):
    """A duck-typed ``/balls`` element — every field `_on_balls` /
    `latch_announced_ball` reads (id/status/destination/tracking/landing
    fields), never the real ``BallState`` message type (see the module
    docstring: `_on_balls` just reads attributes)."""
    ball = MagicMock()
    ball.id = id
    ball.status = status
    ball.destination = destination
    ball.tracking = tracking
    ball.landing_position = _Vec(x, y, z)
    ball.landing_velocity = _Vec(vx, vy, vz)
    ball.time_at_land = _Time(sec, nanosec)
    # Set explicitly, not left to the MagicMock's truthiness: since 2026-09-18
    # this flag decides whether a landing may become a learner row.
    ball.landing_from_fit = landing_from_fit
    return ball


def _pose_at(x, y):
    """Just enough of a `RigidBodyPose.pose` (a `PoseStamped`) for
    `_on_mocap`'s `body.pose.pose.position.x/y` access — duck-typed like
    `_ball()` above, not the real message type."""
    pose = MagicMock()
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose


def _status(**overrides):
    """A `trajectory/status` message at the admissible box's swept limits
    (`config/generated/admissible_box.yaml`: 300/5000/200000 since the R5
    re-sweep, 2026-09-30 — the session leg jerk ramped from 150000 to the YAML
    ceiling; the launch default stays 150000 until the ramp is logged, which
    is why the runsheet sets the limits before `skills/check`) — the LIVE
    limits `_run_one_ball` reads."""
    st = TrajectoryStatus()
    st.leg_vel_limit_mmps = 300.0
    st.leg_acc_limit_mmps2 = 5000.0
    st.leg_jerk_limit_mmps3 = 200000.0
    for k, v in overrides.items():
        setattr(st, k, v)
    return st


class _RecordingClient:
    """Stand-in for the trajectory/install_segment service client."""

    def __init__(self, response=None, ready=True):
        self.calls = []
        self._response = response
        self._ready = ready

    def wait_for_service(self, timeout_sec=None):
        return self._ready

    def call_async(self, request):
        self.calls.append(request)
        future = MockFuture()
        if self._response is not None:
            future.set_result(self._response)
        return future


def _response(**overrides):
    resp = InstallSegment.Response()
    resp.accepted = True
    resp.code = 'OK'
    resp.message = 'ok'
    resp.plan_wall_ms = 12.0
    resp.splice_k = 0
    resp.t0_mono = 100.0
    resp.t_event_mono = 100.4
    resp.duration_s = 0.7
    for k, v in overrides.items():
        setattr(resp, k, v)
    return resp


def _hand_at(node, rev=sn.REST_HAND_REV, ball_held=True):
    """Give `node` a FRESH `/hand_telemetry` sample with the hand at ``rev``.

    EVERY start service now reads the measured hand to SIZE its opening REST
    (`_opening_rest_period`, 2026-09-18) and refuses when that read is stale or
    absent, so without this a start would refuse on freshness alone. Called
    from `_node_with_client` with the hand AT HOME, so a test opts INTO a
    displaced or unread hand rather than out of a healthy one.
    """
    node._on_hand_telemetry(HandTelemetryMessage(
        pos_meas=float(rev), pos_cmd=float(rev),
        ball_held=ball_held, ball_held_raw=ball_held, ball_held_valid=True))


def _node_with_client(response=None, ready=True, hand=True, frame=True):
    """``frame=True`` (the default since 2026-09-21, the unpinning of
    `learner_lateral_authority_mm` 0 -> 40): give the node a ready,
    in-tolerance session-start frame via `_frame_ready(node)` (Platform and
    commanded position both at the origin, at rest, fresh) so a
    default-parameter start path is not refused `REJECTED_FRAME_OFFSET` by a
    test that isn't ABOUT the frame check (plan `cup-contact-contract.md`
    § 1: with authority > 0 a start with no mocap sample is refused — that
    fail-closed behaviour is now live at the launch default, not just under
    an explicit `:=40`).

    Pass ``frame=False`` for a test that IS about the frame check itself —
    one that populates `_platform_mocap_xy` / `_commanded_xy_hist` its own
    way (directly or via its own `_frame_ready(...)` call with a deliberate
    offset), asserts on the raw buffer contents, or wants the
    cannot-evaluate ("no Platform sample yet") branch. Injecting the ready
    default AND a test's own frame data would blend two sets of samples into
    one averaged offset — never what either party intended."""
    node = sn.SkillNode()
    client = _RecordingClient(response=response, ready=ready)
    node._install_cli = client
    # The live session limits the box was swept at (`_status()`: 300/5000/
    # 200000 since the R5 ramp, 2026-09-30). A node that has heard no
    # `trajectory/status` falls back to the YAML launch default (150000) and
    # refuses every 200 k box by name -- exactly what the robot does until the
    # runsheet's `set_limits` row runs, and not what a test about anything
    # else is asking. A test that IS about the mismatch overrides the status.
    node._on_traj_status(_status())
    if hand:
        _hand_at(node)
    if frame:
        _frame_ready(node)
    return node, client


def _go_to_pose_response(**overrides):
    resp = MagicMock()
    resp.accepted = True
    resp.code = 'OK'
    resp.message = 'ok'
    resp.planned_duration_s = 0.0
    for k, v in overrides.items():
        setattr(resp, k, v)
    return resp


def _prelevel_ready(node, *, x=-50.0, y=0.0, z=830.0, **resp_overrides):
    """Wire `node` so `_prelevel` (item 7) succeeds instantly: a ready
    `trajectory/go_to_pose` client and a fresh `trajectory/commanded_position`
    — everything a test that only cares about what happens AFTER the
    self-toss's pre-level move needs, without a real (or ROS-clock-driven)
    wait."""
    node._go_to_pose_cli = _RecordingClient(
        response=_go_to_pose_response(**resp_overrides), ready=True)
    node._commanded_pos_mm = (x, y, z)
    node._commanded_pos_mono = time.perf_counter()
    return node._go_to_pose_cli


def _frame_ready(node, *, plat_x=0.0, plat_y=0.0, cmd_x=0.0, cmd_y=0.0,
                 n_plat=None, n_cmd=5):
    """Populate the session-start frame check's two buffers directly (plan
    `cup-contact-contract.md` § 1) — the Platform mocap body at
    ``(plat_x, plat_y)`` mm and the commanded position at ``(cmd_x, cmd_y)``
    mm, both fresh and, with repeated identical samples, at rest. Bypasses
    `_on_mocap` / `_on_commanded_position` (like `_prelevel_ready` bypasses
    `_on_commanded_position` for `_commanded_pos_mm`) — the node methods are
    exercised directly by `test_on_mocap_...` style tests instead."""
    if n_plat is None:
        n_plat = sn._FRAME_CHECK_MIN_SAMPLES + 5
    now = time.perf_counter()
    for _ in range(n_plat):
        node._platform_mocap_xy.append((now, plat_x, plat_y))
    for _ in range(n_cmd):
        node._commanded_xy_hist.append((now, cmd_x, cmd_y))


def _goal(pattern, **overrides):
    """A `Juggle.Goal` for `node._start_pattern`/`node._juggle_goal` — the
    conftest mock's field defaults (0/False/'') mean "use the node's own
    parameter", exactly like an un-set field on the real action goal."""
    g = Juggle.Goal()
    g.pattern = pattern
    for k, v in overrides.items():
        setattr(g, k, v)
    return g


def _columns_goal(**overrides):
    return _goal('columns', **overrides)


def _self_toss_goal(**overrides):
    return _goal('self_toss', **overrides)


def _hop_goal(**overrides):
    return _goal('hop', **overrides)


# ═════════════════════════════════════════════════════════════════════════════
# columns / stop
# ═════════════════════════════════════════════════════════════════════════════

def test_node_starts_idle():
    node, _client = _node_with_client(hand=False)
    assert node._executor is None
    assert node._possession_evidence == ball_possession.EVIDENCE_UNKNOWN


def _columns_fed(node, tmp_path, *, plant_id):
    """Start a columns attempt (R5, ``reload=False``) and resolve the FEED
    WAIT with a converged tracker landing comfortably inside the lead
    bound, so ``node._executor`` ends up holding the REAL
    ``compile_columns`` schedule rather than the two-phase start's opening
    bridge — the steady state a test that asserts on the compiled
    schedule's own shape needs. Mirrors
    `test_columns_tracker_feed_resolves_an_unannounced_landing_near_p2`
    exactly (brief_S2.md item 2's own recipe), just factored out for reuse
    by tests that came from BEFORE the two-phase start existed and only
    care about the compiled schedule, not the feed machinery itself."""
    node._params['plant_id'] = plant_id
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    pattern = sn.Pattern(sites=sn.columns_sites(100.0), apex_m=0.9,
                         dwell_s=0.30, n_throws=3)
    tau = sn.transit_s(sn.flight_s(pattern.apex_m), pattern.dwell_s)
    lead_needed = tau + pattern.launch_s + sn.LEAD_S + (1.0 / sn._TICK_HZ)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(reload=False,
                                                  separation_mm=100.0))
    assert resp.success is True, resp.message
    node._on_balls(BallStateArray(balls=[
        _ball(7, status=1, tracking=1, x=51.0, y=-2.0, z=800.0,
             vz=-3000.0, sec=0, nanosec=int((lead_needed + 1.0) * 1e9),
             landing_from_fit=True)]))
    assert node._reload_ctx is None
    assert node._executor is not None
    return resp


def test_columns_compiles_a_schedule_at_the_owner_operating_point(tmp_path):
    """R5: `_start_pattern` alone only arms the opening bridge + FEED WAIT
    (item 1) — the compiled ``compile_columns`` schedule this test is
    actually about only exists once a feed resolves it (`_columns_fed`)."""
    node, _client = _node_with_client()
    _columns_fed(node, tmp_path, plant_id='test_columns_compile_op')
    schedule = node._executor.schedule
    assert len(schedule.skills) > 0
    assert schedule.skills[0].kind == 'THROW'


def test_a_second_start_while_running_is_refused(tmp_path):
    node, _client = _node_with_client()
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert node._start_pattern(
            _columns_goal(separation_mm=100.0)).success is True
        resp2 = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp2.success is False
    assert 'already running' in resp2.message


def test_stop_ends_the_attempt(tmp_path):
    node, _client = _node_with_client()
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        node._start_pattern(_columns_goal(separation_mm=100.0))
    assert node._executor is not None
    resp = node._stop_attempt()
    assert resp.success is True
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'STOPPED'
    # Not retired by `_stop_attempt` itself — `.done` is a next-tick question
    # (main-session addition: a stopped attempt keeps ticking until any
    # pending outcome finalises, see the mid-flight test below). Here there
    # is nothing pending (`on_experience is None` for columns), so the very
    # next tick retires it.
    assert node._executor is not None
    node._on_tick()
    assert node._executor is None


def test_stop_keeps_ticking_until_a_pending_flight_finalises_then_appends_its_row(
        tmp_path):
    """cancelling the goal must not discard an observable flight already in
    progress: it ends the attempt (`attempt_ended=True`, `end_code='STOPPED'`)
    but leaves the executor live so `_on_tick` keeps calling `.tick()` until
    `.done` — the stopped attempt's flight still produces its memory row
    (main-session addition). A new start is refused while that finalisation
    is still pending.

    ``_REPO_ROOT`` is patched to `tmp_path` (a fresh, isolated memory file)
    rather than writing into the repo's own ``temp/learn/`` — a fixed shared
    path would accumulate rows across repeated runs (CLAUDE.md: "write files
    via tmp_path, never a fixed shared path")."""
    node, _client = _node_with_client(response=_response(t_release_mono=50.0))
    node._on_traj_status(_status())
    _prelevel_ready(node)
    plant_id = 'test_stop_mid_flight'
    node._params['plant_id'] = plant_id

    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert node._start_pattern(_self_toss_goal()).success is True
        # Freshen the R3 ladder AFTER start (item 6/7 wiring is per-tick, not
        # a start-time check) so the dispatch below is not itself refused by
        # `precondition_refusals` — this test is about the outcome-capture /
        # stop interaction, not the ladder.
        _freshen(node)

        first_throw = next(s for s in node._executor.schedule.skills
                           if s.kind == 'THROW')
        node._executor.tick(first_throw.dispatch_s())    # dispatches the throw
        assert len(node._executor._pending_outcomes) == 1
        pend = node._executor._pending_outcomes[0]

        t_land = pend.t_land_scheduled_s
        node._on_balls(BallStateArray(balls=[
            _ball(7, status=1, destination='jugglebot', tracking=1,
                 x=-50.0, y=0.0, z=830.0, vz=-2500.0,
                 sec=int(t_land), nanosec=int((t_land % 1.0) * 1e9))]))

        # The outcome observation FREEZES at the crossing (2026-09-16), so the
        # tracker has to be sampled while the ball is still in the air — which
        # the live 40 Hz tick loop does dozens of times per flight. This
        # harness's only remaining tick is AFTER the landing, and an extra
        # in-flight tick here would dispatch this ball's CATCH (re-announcing
        # it, which clears the tracker correlation, and registering a second
        # pending row) — neither of which this test is about. So feed the one
        # in-flight sample the tick loop would have taken.
        node._executor._consider_landing(pend, node._tracker(pend.ball_id),
                                         pend.t_release_s + 0.01)
        assert pend.best_landing is not None

        resp = node._stop_attempt()
        assert resp.success is True
        assert node._executor is not None          # still finalising
        assert node._executor.attempt_ended is True
        assert node._executor.end_code == 'STOPPED'

        resp2 = node._start_pattern(_self_toss_goal())
        assert resp2.success is False
        assert 'already running' in resp2.message

        finalise_at = t_land + CAUGHT_WINDOW_S
        node._executor.tick(finalise_at)
        assert node._executor._pending_outcomes == []
        node._on_tick()
        assert node._executor is None

    memory = Memory(memory_path(str(tmp_path), plant_id))
    assert len(memory) == 1


def test_stop_with_no_attempt_is_a_harmless_success():
    node, _client = _node_with_client()
    resp = node._stop_attempt()
    assert resp.success is True
    assert 'no attempt' in resp.message


# ═════════════════════════════════════════════════════════════════════════════
# self_toss (R3 items 4/5) — the box/limits gate and the compiled schedule
# ═════════════════════════════════════════════════════════════════════════════

def test_self_toss_is_refused_when_the_box_file_is_missing(tmp_path):
    node, _client = _node_with_client()
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(tmp_path / 'nope.yaml')):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'admissible box refused' in resp.message
    assert node._executor is None


def test_self_toss_is_refused_on_a_limits_mismatch(tmp_path):
    box_path = tmp_path / 'admissible_box.yaml'
    box = adm.AdmissibleBox(
        site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), apex_m=(0.6, 1.2),
        pattern='self_toss', release_site_xy_mm=(-50.0, 0.0),
        target_site_xy_mm=(-50.0, 0.0),
        limits={'leg_vel_mmps': 1.0, 'leg_acc_mmps2': 1.0,
               'leg_jerk_mmps3': 1.0, 'hand_acc_rps2': 1.0},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13')
    adm.dump(str(box_path), [box])
    node, _client = _node_with_client()
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(box_path)):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'admissible box refused' in resp.message
    assert 'leg_vel_mmps' in resp.message
    assert node._executor is None


def test_self_toss_is_refused_on_a_dwell_mismatch(tmp_path):
    """R5 (D4, 2026-09-30): a box swept at a different dwell than the LIVE
    session dwell (``skill_node``'s ``dwell_s`` parameter) refuses by name --
    the columns cell's admitted set is dwell-shaped (a shorter dwell widens
    the transit and loosens the catch-with-throw gate), so a stale-dwell box
    must not be silently selected."""
    box_path = tmp_path / 'admissible_box.yaml'
    box = adm.AdmissibleBox(
        site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), apex_m=(0.6, 1.2),
        pattern='self_toss', release_site_xy_mm=(-50.0, 0.0),
        target_site_xy_mm=(-50.0, 0.0),
        limits={'leg_vel_mmps': 300.0, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': 3500.0},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13',
        dwell_s=0.20)                            # != the node's default 0.30
    adm.dump(str(box_path), [box])
    node, _client = _node_with_client()
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(box_path)):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'admissible box refused' in resp.message
    assert 'dwell_s' in resp.message
    assert node._executor is None


def test_self_toss_is_refused_with_no_status_received_yet():
    """The live-limits fallback (`_live_limits`) reads the YAML module
    defaults when `trajectory/status` has never arrived (its own documented
    "0.0 = field absent" sentinel) — which do not match the real box's swept
    session limits, so the start refuses rather than silently gating against
    the wrong machine."""
    node, _client = _node_with_client()
    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    # Since 2026-09-16 the YAML launch defaults ARE the swept session limits
    # (300/5000/200000 since 2026-09-30, owner decision), so the box no longer refuses here;
    # the start is still refused, by the next rung of the ladder (no live
    # commanded position yet). Either refusal is the contract: no motion on
    # a machine state nobody has confirmed.
    assert ('admissible box refused' in resp.message
            or 'stale' in resp.message)


def test_a_new_schedule_drops_the_previous_attempts_flight_latches(tmp_path):
    """A new schedule reuses ball id 0: a latch left by an earlier attempt must
    not answer the new schedule's first catch (audit, 2026-09-16)."""
    node, _client = _node_with_client()
    with node._correlation_lock:
        node._correlation[0] = ('stale-latch',)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        node._start_pattern(_columns_goal(separation_mm=100.0))
    assert 0 not in node._correlation


def test_the_learner_lateral_authority_parameter_defaults_to_twenty():
    """Owner 2026-09-16 pinned this at 0.0 (the learner corrects flight only)
    until the planner's small-lateral-offset banking defect was fixed; owner
    2026-09-21 lifted the pin to 40 mm once the cup-contact contract landed
    amplitude-aware banking (`plans/archived/cup-contact-contract.md` § 6);
    owner 2026-09-23 set 20 mm after the first unpinned sitting — 40 mm
    dropped balls on the 25-throw chains at 0.9 m, 20 mm was stable at both
    apexes. The box still admits +/-40 mm; this is the executor's clamp."""
    node, _client = _node_with_client()
    assert node.get_parameter('learner_lateral_authority_mm').value == 20.0


def test_the_hold_tilt_max_deg_parameter_defaults_to_the_planner_ceiling():
    """R5 owner experiment (2026-09-30): unset, the reload's held-catch tilt
    cap is the planner's own ceiling (`tilt_geometry.MAX_TILT_DEG`, 12 deg) —
    an un-set launch flies exactly as before this knob existed."""
    node, _client = _node_with_client()
    assert node.get_parameter('hold_tilt_max_deg').value == pytest.approx(
        tg.MAX_TILT_DEG)
    assert node._hold_tilt_max_deg() == pytest.approx(tg.MAX_TILT_DEG)


def test_hold_tilt_max_deg_reads_the_runsheet_override():
    """`ros2 param set .../skill_node hold_tilt_max_deg 4.0` (the owner's
    experiment) reaches `compile_reload` through `_hold_tilt_max_deg`."""
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter(4.0, name='hold_tilt_max_deg')])
    assert node._hold_tilt_max_deg() == pytest.approx(4.0)


def test_hold_tilt_max_deg_clamps_outside_the_ceiling():
    """A bound on a physical attitude, not a policy (mirrors
    `_catch_resend_max`): out-of-range is clamped into ``(0, 12]``, never
    refused, so a runsheet typo cannot silently fly with an un-capped tilt
    or a zero one."""
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter(30.0, name='hold_tilt_max_deg')])
    assert node._hold_tilt_max_deg() == pytest.approx(tg.MAX_TILT_DEG)
    node.set_parameters([_MockParameter(0.0, name='hold_tilt_max_deg')])
    assert 0.0 < node._hold_tilt_max_deg() <= tg.MAX_TILT_DEG
    node.set_parameters([_MockParameter(-5.0, name='hold_tilt_max_deg')])
    assert 0.0 < node._hold_tilt_max_deg() <= tg.MAX_TILT_DEG


def test_self_toss_refuses_an_uncovered_apex_before_any_motion(tmp_path):
    """THE LATENT DEFECT this closes (found 2026-09-14): the only swept box
    covers apex 0.85-0.95 m; requesting 0.5 m (uncovered) must refuse BEFORE
    `_prelevel` ever calls `trajectory/go_to_pose` -- no platform motion for
    an apex nothing was swept for."""
    box_path = tmp_path / 'admissible_box.yaml'
    box = adm.AdmissibleBox(
        site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), apex_m=(0.6, 1.2),
        pattern='self_toss', release_site_xy_mm=(-50.0, 0.0),
        target_site_xy_mm=(-50.0, 0.0),
        limits={'leg_vel_mmps': 300.0, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': 3500.0},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13',
        dwell_s=sn._DEFAULT_DWELL_S)
    adm.dump(str(box_path), [box])
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    prelevel_client = _prelevel_ready(node)
    node._params['apex_m'] = 0.5
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(box_path)):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'no admissible box covers' in resp.message
    assert '0.500' in resp.message
    assert '0.850' in resp.message and '0.950' in resp.message
    assert node._executor is None
    assert prelevel_client.calls == []


def test_self_toss_compiles_a_schedule_at_the_owner_operating_point():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    prelevel_client = _prelevel_ready(node)
    node._params['plant_id'] = 'test_self_toss_compiles'
    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is True, resp.message
    assert node._executor is not None
    schedule = node._executor.schedule
    assert schedule.skills[0].kind == 'REST'   # the opening floor lift
    np.testing.assert_allclose(schedule.skills[0].site.cup_mm[:2], [-50.0, 0.0])
    assert any(s.kind == 'THROW' for s in schedule.skills)
    assert any(b.site_pair == ('P1', 'P1') for b in node._executor.boxes)
    assert node._executor.learner is not None
    assert callable(node._executor.learner.command)
    assert callable(node._executor.on_experience)
    assert node._executor.observer == node._ball_evidence
    assert node._executor.observations == node._observations
    assert 'memory rows=0' in resp.message
    # The pre-level move itself: identity orientation, at the LIVE commanded
    # xy/z (item 7) — a pure attitude move, nothing about position restated.
    assert len(prelevel_client.calls) == 1
    req = prelevel_client.calls[0]
    assert (req.pose.position.x, req.pose.position.y, req.pose.position.z) \
        == (-50.0, 0.0, 830.0)


def test_self_toss_while_running_is_refused():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    node._params['plant_id'] = 'test_self_toss_while_running'
    assert node._start_pattern(_self_toss_goal()).success is True
    resp2 = node._start_pattern(_self_toss_goal())
    assert resp2.success is False
    assert 'already running' in resp2.message


def test_a_site_off_the_swept_box_is_refused_before_the_platform_moves():
    """Finding 5, R3 audit (2026-09-13): the admissible box carries no site
    xy, so a `site_x_mm`/`site_y_mm` off the swept default (-50, 0) mm must
    be refused before `_prelevel` ever moves the platform."""
    node, _c = _node_with_client()
    node._on_traj_status(_status())
    pre = _prelevel_ready(node)
    node._params['plant_id'] = 'sketch_site_off_box'
    node._params['site_x_mm'] = 0.0
    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'site' in resp.message and 'swept default' in resp.message
    assert pre.calls == []
    assert node._executor is None


def test_a_bad_plant_id_is_refused_before_the_platform_moves(tmp_path):
    """Finding 11, R3 audit (2026-09-13): `memory_path` raises `ValueError`
    on a bad `plant_id` — must be caught before `_prelevel`, not left to
    kill the node in a service callback after the platform has moved."""
    node, _c = _node_with_client()
    node._on_traj_status(_status())
    pre = _prelevel_ready(node)
    node._params['plant_id'] = '../escape'
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'memory refused' in resp.message
    assert pre.calls == []
    assert node._executor is None


def test_an_unreadable_memory_file_is_refused_not_raised(tmp_path):
    """Finding 11, R3 audit (2026-09-13): `Memory(...)` can raise `OSError`
    (here, the memory path is a directory, not a file) — refused, not an
    uncaught exception that kills the node."""
    node, _c = _node_with_client()
    node._on_traj_status(_status())
    pre = _prelevel_ready(node)
    node._params['plant_id'] = 'dir_not_file'
    (tmp_path / 'temp' / 'learn' / 'dir_not_file' / 'memory.csv').mkdir(
        parents=True)
    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'memory refused' in resp.message
    assert pre.calls == []
    assert node._executor is None


# ═════════════════════════════════════════════════════════════════════════════
# self_toss (R3-e2 item 7) — the pre-level move
# ═════════════════════════════════════════════════════════════════════════════

def test_self_toss_is_refused_when_go_to_pose_is_unavailable():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    node._go_to_pose_cli = _RecordingClient(ready=False)
    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'self_toss refused' in resp.message
    assert 'go_to_pose unavailable' in resp.message
    assert node._executor is None


def test_self_toss_is_refused_when_commanded_position_is_stale():
    """No `trajectory/commanded_position` has ever arrived — `_prelevel` has
    no xy/z to hold and must refuse rather than guess one."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    node._go_to_pose_cli = _RecordingClient(response=_go_to_pose_response())
    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'commanded_position is stale' in resp.message
    assert node._executor is None


def test_self_toss_is_refused_when_the_prelevel_move_is_refused():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node, accepted=False, code='WORKSPACE', message='nope')
    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'pre-level move was refused' in resp.message
    assert 'WORKSPACE' in resp.message
    assert node._executor is None


# ═════════════════════════════════════════════════════════════════════════════
# the tick
# ═════════════════════════════════════════════════════════════════════════════

def test_the_tick_dispatches_the_opening_rest_bridge_through_the_mocked_client():
    """Since the R5 feed-triggered start (2026-09-30) a columns goal installs
    its bridge first — ONE opening REST that homes the hand and lifts the cup
    onto P1 with ball A held — and waits for the feed; the THROW/CATCH
    schedule is compiled only when a feed landing arrives (pinned by the
    feed-wait tests below).  What this test pins is the seam: the first tick
    hands the bridge's one skill to the mocked `install_segment` client as a
    REST with the schedule's own instant.  Before R5 the first skill was the
    launch THROW and the test asserted its flight time; a REST has none."""
    node, client = _node_with_client(response=_response())
    _freshen(node)
    assert node._start_pattern(_columns_goal(separation_mm=100.0)).success is True
    bridge = node._executor.schedule.skills[0]
    assert bridge.kind == 'REST'
    assert bridge.holds_ball is True
    t_dispatch = bridge.dispatch_s()

    node._executor.tick(t_dispatch)

    assert len(client.calls) == 1
    req = client.calls[0]
    assert req.kind == InstallSegment.Request.KIND_REST
    assert req.ball_id == bridge.ball_id
    assert req.t_event_s == pytest.approx(bridge.t_abs_s)


def test_a_refusal_ends_the_attempt(tmp_path):
    """The columns bridge executor is wired with `observations=` (unlike the
    schedule-shape assumed before R5's two-phase start), so `tick` must see
    `in_trajectory_mode=True` or it ends `ABORTED_MODE_CHANGED` before ever
    reaching the install response under test — `_freshen` (mode='TRAJECTORY')
    keeps this test about the INSTALL refusal, not the mode gate."""
    node, _client = _node_with_client(
        response=_response(accepted=False, code='WRONG_MODE',
                          message='not in TRAJECTORY mode'))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        node._start_pattern(_columns_goal(separation_mm=100.0))
    first = node._executor.schedule.skills[0]
    node._executor.tick(first.dispatch_s())
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'WRONG_MODE'
    # `_on_tick` retires the executor once the attempt has ended.
    node._on_tick()
    assert node._executor is None


# ═════════════════════════════════════════════════════════════════════════════
# tracking
# ═════════════════════════════════════════════════════════════════════════════

def test_a_balls_message_feeds_the_landing_cache():
    """`_on_balls` still caches every reported ball's landing by its OWN
    (tracker) id, independent of correlation (item 3's `_balls` cache, read
    by `_tracker` only once a schedule ball id is correlated to one)."""
    node, _client = _node_with_client()
    node._on_balls(BallStateArray(
        balls=[_ball(3, x=1.0, y=2.0, z=3.0, vz=-2000.0, sec=10,
                     nanosec=500_000_000)]))
    landing = node._balls.get(3)
    assert landing is not None
    np.testing.assert_array_equal(landing.pos_mm, [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(landing.vel_mm_s, [0.0, 0.0, -2000.0])
    assert landing.t_land_abs_s == pytest.approx(10.5)


def test_the_tracker_returns_none_before_any_release_is_announced():
    """`_tracker` is keyed by SCHEDULE ball id and needs a correlation entry
    (`_maybe_announce`) before it can return anything — a `/balls` id alone,
    with no throw ever dispatched for that schedule ball id, is not enough."""
    node, _client = _node_with_client()
    node._on_balls(BallStateArray(balls=[_ball(3, status=1,
                                               destination='jugglebot',
                                               tracking=1)]))
    assert node._tracker(0) is None


def test_a_throw_install_announces_the_release_and_starts_correlation():
    """Wire-faithful response: `trajectory_node` answers a plain THROW with
    `t_release_mono == 0.0` (that field is a CATCH-with-throw's SECOND event;
    `segments._plan_throw` never sets `release_t_s`) and the release on
    `t_event_mono`. Until 2026-09-13 this test overrode `t_release_mono` to a
    value the node never produces, and the node read only that field — so
    every standalone self-toss on the first R3 sitting announced nothing,
    never correlated its ball, and ended NO_LANDING against a CONFIRMED
    track (bag 2026-09-13_22-57-18, five of five)."""
    from jugglebot.motion.skills.segments import ThrowTerminal
    from jugglebot.motion.trajectory import ballistics_bc

    node, _client = _node_with_client(
        response=_response(t_event_mono=50.0, t_release_mono=0.0))
    terminal = ThrowTerminal(site_mm=[0.0, 0.0, 860.0],
                             target_mm=[0.0, 0.0, 830.0], flight_s=0.6,
                             t_release_s=100.0)
    result = node._installer('THROW', terminal, 100.0, ball_id=0)
    assert result.accepted is True
    assert len(node._announce_pub.published) == 1
    ann = node._announce_pub.published[0]
    assert ann.thrower_name == node._robot_name
    assert ann.target_id == node._robot_name
    assert ann.predicted_tof_sec == pytest.approx(0.6)
    expected_vel = ballistics_bc.launch_velocity([0.0, 0.0, 860.0],
                                                 [0.0, 0.0, 830.0], 0.6)
    assert ann.initial_velocity.z == pytest.approx(expected_vel[2])
    assert ann.landing_position.z == pytest.approx(830.0)
    assert 0 in node._correlation
    # ONE latch per announced release (2026-09-16), unlatched until a
    # `/balls` snapshot carries a candidate.
    assert len(node._correlation[0]) == 1
    assert node._correlation[0][0].announced_id is None
    assert node._correlation[0][0].preexisting == ()


def test_correlation_latches_the_new_ball_and_excludes_a_preexisting_one():
    from jugglebot.motion.skills.segments import ThrowTerminal

    node, _client = _node_with_client(response=_response(t_event_mono=50.0, t_release_mono=0.0))
    # A phantom track already IN_FLIGHT before the throw.
    node._on_balls(BallStateArray(
        balls=[_ball(99, status=1, destination='jugglebot', tracking=1)]))
    terminal = ThrowTerminal(site_mm=[0.0, 0.0, 860.0],
                             target_mm=[0.0, 0.0, 830.0], flight_s=0.6,
                             t_release_s=100.0)
    node._installer('THROW', terminal, 100.0, ball_id=0)
    assert node._correlation[0][0].preexisting == (99,)

    # The phantom is still there; the NEW ball (id 7) also appears.
    node._on_balls(BallStateArray(balls=[
        _ball(99, status=1, destination='jugglebot', tracking=1),
        _ball(7, status=1, destination='jugglebot', tracking=1,
             x=1.0, y=2.0, z=3.0),
    ]))
    assert node._correlation[0][0].announced_id == 7
    landing = node._tracker(0)
    assert landing is not None
    np.testing.assert_array_equal(landing.pos_mm, [1.0, 2.0, 3.0])


def test_the_tracker_withholds_a_landing_until_tracking_is_confirmed():
    from jugglebot.motion.skills.segments import ThrowTerminal

    node, _client = _node_with_client(response=_response(t_event_mono=50.0, t_release_mono=0.0))
    terminal = ThrowTerminal(site_mm=[0.0, 0.0, 860.0],
                             target_mm=[0.0, 0.0, 830.0], flight_s=0.6,
                             t_release_s=100.0)
    node._installer('THROW', terminal, 100.0, ball_id=0)
    node._on_balls(BallStateArray(
        balls=[_ball(7, status=1, destination='jugglebot', tracking=0)]))
    assert node._correlation[0][0].announced_id == 7
    assert node._tracker(0) is None    # IN_FLIGHT alone proves nothing
    node._on_balls(BallStateArray(
        balls=[_ball(7, status=1, destination='jugglebot', tracking=1)]))
    assert node._tracker(0) is not None


def test_a_catch_resend_does_not_re_announce_or_reset_correlation():
    from jugglebot.motion.skills.segments import CatchTerminal, ThrowAfterCatch

    node, _client = _node_with_client(response=_response(t_release_mono=50.0))
    then_throw = ThrowAfterCatch(t_release_s=101.0, site_mm=[0.0, 0.0, 860.0],
                                 target_mm=[10.0, -5.0, 830.0], flight_s=0.6)
    terminal = CatchTerminal(landing_mm=[0.0, 0.0, 830.0],
                             landing_vel_mm_s=[0.0, 0.0, -2500.0],
                             t_land_s=100.0, rest_site_mm=[0.0, 0.0, 750.0],
                             then_throw=then_throw)
    node._installer('CATCH', terminal, 99.0, ball_id=0)
    assert len(node._announce_pub.published) == 1
    node._on_balls(BallStateArray(
        balls=[_ball(7, status=1, destination='jugglebot', tracking=1)]))
    assert node._correlation[0][0].announced_id == 7

    # A re-send carrying the IDENTICAL release (same then_throw content).
    node._installer('CATCH', terminal, 99.5, ball_id=0)
    assert len(node._announce_pub.published) == 1          # not re-announced
    assert len(node._correlation[0]) == 1                   # no second latch
    assert node._correlation[0][0].announced_id == 7        # not re-latched


def _correlate(node, releases, *, now_s):
    """Seed `_correlation[0]` with one unlatched latch per release instant and
    pin the node's clock at ``now_s`` — the two inputs `flight_in_progress`
    reads. Built directly rather than through two `_installer` calls because
    the mocked ROS clock is frozen at 0 and both announcements would then land
    at (almost) the same instant; the announce path's own latch-queueing is
    covered by the tests above."""
    node._correlation[0] = tuple(
        ball_possession.FlightLatch(t_release_s=float(t)) for t in releases)
    node._now_s = lambda: float(now_s)


def test_the_correlation_follows_the_flight_in_progress_not_the_next_release():
    """THE 2026-09-16 regression (bag `2026-09-16_16-22-22`, armB-090 attempt
    1). A chained CATCH+THROW announces its carried release at DISPATCH, ~1.25 s
    ahead, while the PREVIOUS flight of the same schedule ball is still in the
    air; the tracker mints a new id per announcement (48 airborne, 49
    announced) and flips 49 to IN_FLIGHT at ITS OWN release. The correlation
    must stay on 48 for the whole of 48's flight — the node used to reset the
    latch at each announcement and re-latch onto 49 the instant it went
    IN_FLIGHT, which handed the flight-48 outcome row flight 49's landing
    (observed 2.2317 s for an 0.8569 s command)."""
    node, _client = _node_with_client()
    # Releases as measured: 417.361 (flying) and 418.511 (still 1.1 s away).
    _correlate(node, (417.361, 418.511), now_s=417.9)
    node._on_balls(BallStateArray(balls=[
        _ball(48, status=1, destination='jugglebot', tracking=1,
             sec=418, nanosec=218_000_000),
        # Adversarial: the tracker ALSO reports 49 as IN_FLIGHT+CONFIRMED with
        # the next flight's pre-computed landing. Its release has not happened,
        # so it cannot be the flight in progress whatever /balls says.
        _ball(49, status=1, destination='jugglebot', tracking=1,
             sec=419, nanosec=368_000_000),
    ]))
    assert node._correlation[0][0].announced_id == 48
    assert node._correlation[0][1].announced_id is None     # not yet released
    landing = node._tracker(0)
    assert landing is not None
    assert landing.t_land_abs_s == pytest.approx(418.218)    # flight 48's


def test_the_correlation_moves_to_the_next_flight_once_its_release_passes():
    """...and then it DOES move: once 48 is CAUGHT and 49's release has
    passed, 49 is the flight in progress and its landing is the one returned.
    Same two latches, one clock read later (measured instants: 48 CAUGHT at
    418.714, 49 released at 418.511)."""
    node, _client = _node_with_client()
    _correlate(node, (417.361, 418.511), now_s=417.9)
    node._on_balls(BallStateArray(balls=[
        _ball(48, status=1, destination='jugglebot', tracking=1,
             sec=418, nanosec=218_000_000)]))
    assert node._correlation[0][0].announced_id == 48

    node._now_s = lambda: 418.9
    node._on_balls(BallStateArray(balls=[
        _ball(48, status=2, destination='jugglebot', tracking=1,   # CAUGHT
             sec=418, nanosec=510_000_000),
        _ball(49, status=1, destination='jugglebot', tracking=1,
             sec=419, nanosec=368_000_000),
    ]))
    # 49 is latched to the SECOND release, not the first: 48 is excluded both
    # as a sibling's claim and because it is no longer IN_FLIGHT.
    assert node._correlation[0][0].announced_id == 48
    assert node._correlation[0][1].announced_id == 49
    landing = node._tracker(0)
    assert landing is not None
    assert landing.t_land_abs_s == pytest.approx(419.368)    # flight 49's


def test_the_tracker_withholds_a_landing_when_the_flight_in_progress_ended():
    """A CAUGHT flight yields NO landing rather than the previous flight's or
    the next one's: `flight_in_progress` never falls back to an earlier latch
    (that flight is over) and never reads an unreleased one."""
    node, _client = _node_with_client()
    _correlate(node, (417.361, 418.511), now_s=418.9)
    node._on_balls(BallStateArray(balls=[
        _ball(48, status=2, destination='jugglebot', tracking=1,
             sec=418, nanosec=510_000_000),
    ]))
    assert node._tracker(0) is None


def test_a_second_announcement_does_not_reset_the_flight_in_progress():
    """The announce path itself (not a hand-built latch): a second ACCEPTED
    install for the SAME schedule ball appends a latch and leaves the first
    one's id alone. `preexisting` then keeps the new release off the airborne
    id, which is what made the reset so damaging — it excluded exactly the
    ball that was flying."""
    from jugglebot.motion.skills.segments import ThrowTerminal

    node, _client = _node_with_client(
        response=_response(t_event_mono=50.0, t_release_mono=0.0))
    first = ThrowTerminal(site_mm=[0.0, 0.0, 860.0], target_mm=[0.0, 0.0, 830.0],
                          flight_s=0.6, t_release_s=100.0)
    node._installer('THROW', first, 100.0, ball_id=0)
    node._on_balls(BallStateArray(balls=[
        _ball(48, status=1, destination='jugglebot', tracking=1,
             sec=100, nanosec=600_000_000)]))
    assert node._correlation[0][0].announced_id == 48

    node._install_cli._response = _response(t_event_mono=51.15,
                                            t_release_mono=0.0)
    second = ThrowTerminal(site_mm=[0.0, 0.0, 860.0], target_mm=[0.0, 0.0, 830.0],
                           flight_s=0.6, t_release_s=101.15)
    node._installer('THROW', second, 101.15, ball_id=0)
    assert len(node._announce_pub.published) == 2
    assert len(node._correlation[0]) == 2
    assert node._correlation[0][0].announced_id == 48        # NOT reset
    assert node._correlation[0][1].announced_id is None
    assert node._correlation[0][1].preexisting == (48,)      # the airborne id


def test_hand_telemetry_updates_possession_evidence():
    node, _client = _node_with_client()
    node._on_hand_telemetry(HandTelemetryMessage(ball_held_raw=True,
                                                 ball_held_valid=True))
    assert node._possession_evidence == ball_possession.EVIDENCE_SEATED
    node._on_hand_telemetry(HandTelemetryMessage(ball_held_raw=False,
                                                 ball_held_valid=True))
    assert node._possession_evidence == ball_possession.EVIDENCE_EMPTY
    node._on_hand_telemetry(HandTelemetryMessage(ball_held_raw=False,
                                                 ball_held_valid=False))
    assert node._possession_evidence == ball_possession.EVIDENCE_UNKNOWN


def _freshen(node, *, mocap=True, levelled=True, in_traj=True, hand_fresh=True,
            pos_meas=0.0, pos_cmd=0.0, seated=True):
    """Land every message `_observations` (item 6) reads, fresh, so a single
    test can flip exactly one axis stale/off-band and check only that field
    moved.

    `pos_meas`/`pos_cmd` no longer feed any PREDICATE (the hand-position
    ladder row was retired 2026-09-16) — they are kept as parameters because
    `skills/check` still REPORTS both, and the sitting that retired the row
    was one where they disagreed."""
    if mocap:
        node._on_mocap(RigidBodyPoses())
    node._on_traj_status(_status(
        gravity_correction_loaded=levelled,
        mode='TRAJECTORY' if in_traj else 'STANDBY'))
    if hand_fresh:
        node._on_hand_telemetry(HandTelemetryMessage(
            pos_meas=pos_meas, pos_cmd=pos_cmd,
            ball_held=seated, ball_held_raw=seated, ball_held_valid=True))


# ═════════════════════════════════════════════════════════════════════════════
# the R3 precondition ladder's observations (item 6)
# ═════════════════════════════════════════════════════════════════════════════

def test_observations_are_all_false_before_anything_has_arrived():
    node, _client = _node_with_client(hand=False)
    obs = node._observations(0.0)
    assert obs.mocap_fresh is False
    assert obs.hand_fresh is False
    assert obs.levelled is False
    assert obs.in_trajectory_mode is False
    assert obs.ball_evidence == ball_possession.EVIDENCE_UNKNOWN


def test_the_ladder_reads_the_debounced_bit_and_the_observer_the_raw_one():
    """2026-09-16: two of the sitting's three REJECTED_NO_BALL refusals were a
    single raw-sample carry-flicker during the post-hold park motion while the
    debounced bit stayed True. The ladder's SEATED precondition is a STATE
    question (debounced); release/catch edges stay on the raw bit (the debounce
    lags a departing ball by ~240 ms)."""
    node, _client = _node_with_client()
    _freshen(node)
    node._on_hand_telemetry(HandTelemetryMessage(
        ball_held=True, ball_held_raw=False, ball_held_valid=True))
    obs = node._observations(0.0)
    assert obs.ball_evidence == ball_possession.EVIDENCE_SEATED
    assert node._ball_evidence(0, 0.0) == ball_possession.EVIDENCE_EMPTY
    node._on_hand_telemetry(HandTelemetryMessage(
        ball_held=False, ball_held_raw=True, ball_held_valid=True))
    assert node._observations(0.0).ball_evidence == ball_possession.EVIDENCE_EMPTY
    node._on_hand_telemetry(HandTelemetryMessage(
        ball_held=True, ball_held_raw=True, ball_held_valid=False))
    assert node._observations(0.0).ball_evidence == ball_possession.EVIDENCE_UNKNOWN


def test_observations_report_true_once_everything_is_fresh_and_levelled():
    node, _client = _node_with_client()
    _freshen(node)
    obs = node._observations(0.0)
    assert obs.mocap_fresh is True
    assert obs.hand_fresh is True
    assert obs.levelled is True
    assert obs.in_trajectory_mode is True
    assert obs.ball_evidence == ball_possession.EVIDENCE_SEATED


def test_observations_go_stale_after_the_freshness_window(monkeypatch):
    node, _client = _node_with_client()
    _freshen(node)
    # Rewind every arrival stamp past its own window — a stale MESSAGE, not
    # an absent one, is the case a live sitting actually hits.
    node._mocap_mono -= sn._MOCAP_STALE_S + 0.1
    node._traj_status_mono -= sn._TRAJ_STATUS_STALE_S + 0.1
    node._hand_telemetry_mono -= sn._HAND_STATE_STALE_S + 0.1
    obs = node._observations(0.0)
    assert obs.mocap_fresh is False
    assert obs.levelled is False
    assert obs.in_trajectory_mode is False
    assert obs.hand_fresh is False


def test_observations_carry_no_hand_position_predicate_at_all():
    """RETIRED 2026-09-16 (owner decision). `Observations` used to carry
    `hand_at_seed` (|pos_meas − pos_cmd|) and `hand_at_park`, and on
    2026-09-16 they refused nine schedules at skill 0 on a hand measured at
    +0.0001 rev whose bridge echo had gone stale at +0.5639.

    The strongest available assertion is a NEGATIVE one on the built object:
    a wildly disagreeing pair produces no predicate to be false, so the
    refusal cannot come back by accident."""
    node, _client = _node_with_client()
    _freshen(node, pos_meas=0.0001, pos_cmd=0.5639)
    obs = node._observations(0.0)
    assert not hasattr(obs, 'hand_at_seed')
    assert not hasattr(obs, 'hand_at_park')
    assert obs.hand_fresh is True        # the row that IS kept


def test_ball_evidence_observer_delegates_to_observations():
    """The executor's `observer` callable (item 6) reads the SAME evidence
    `_observations` builds — one build, no second copy."""
    node, _client = _node_with_client()
    _freshen(node, seated=False)
    assert node._ball_evidence(0, 0.0) == ball_possession.EVIDENCE_EMPTY
    assert node._ball_evidence(0, 0.0) == node._observations(0.0).ball_evidence


# ═════════════════════════════════════════════════════════════════════════════
# skills/check (item 8)
# ═════════════════════════════════════════════════════════════════════════════

def test_check_reports_every_ladder_refusal_and_the_box_status_at_once():
    node, _client = _node_with_client(hand=False)
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'ladder REFUSED' in resp.message
    for code in ('REJECTED_MOCAP_STALE', 'REJECTED_NOT_LEVELLED',
                'REJECTED_HAND_STALE', 'REJECTED_BALL_UNKNOWN'):
        assert code in resp.message
    assert 'REJECTED_HAND_NOT_PARKED' not in resp.message   # retired 2026-09-16
    assert 'box' in resp.message


def test_check_reports_the_hand_position_without_gating_on_it():
    """`skills/check` REPORTS pos_meas and the bridge's pos_cmd echo side by
    side (2026-09-16): the hand's position is no longer a refusal, but the
    operator still needs to see where the hand is before a sitting — and the
    echo is the channel that went stale and produced nine false refusals, so
    a disagreement has to be visible rather than inferred."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _freshen(node, pos_meas=0.0001, pos_cmd=0.5639)
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert 'ladder OK' in resp.message
    assert '0.0001' in resp.message and '0.5639' in resp.message
    assert 'not gated' in resp.message
    # ... and, since 2026-09-18, the ONE number a displaced hand changes about
    # the schedule: how long its opening REST would take to home it.
    assert '%.2f s to home it' % (sn.floor_lift_s(0.0001),) in resp.message


def test_check_reports_ok_when_everything_is_fresh_and_the_box_is_valid():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _freshen(node)
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert resp.success is True
    assert 'ladder OK' in resp.message
    assert 'box OK' in resp.message


def test_check_reports_box_refused_when_the_apex_param_is_uncovered(tmp_path):
    """`skills/check` must judge the SAME (site pair, apex band) predicate a
    live throw is judged by -- not merely "a box file exists"."""
    box_path = tmp_path / 'admissible_box.yaml'
    box = adm.AdmissibleBox(
        site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), apex_m=(0.6, 1.2),
        pattern='self_toss', release_site_xy_mm=(-50.0, 0.0),
        target_site_xy_mm=(-50.0, 0.0),
        limits={'leg_vel_mmps': 300.0, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': 3500.0},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13',
        dwell_s=sn._DEFAULT_DWELL_S)
    adm.dump(str(box_path), [box])
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _freshen(node)
    node._params['apex_m'] = 0.5
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(box_path)):
        resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'ladder OK' in resp.message
    assert 'box REFUSED' in resp.message
    assert '0.500' in resp.message


def test_check_reports_box_refused_when_the_file_is_missing(tmp_path):
    node, _client = _node_with_client()
    _freshen(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(tmp_path / 'nope.yaml')):
        resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'ladder OK' in resp.message
    assert 'box REFUSED' in resp.message


# ═════════════════════════════════════════════════════════════════════════════
# skills/check — the session-start frame check (handoff_u5.md open question 1:
# a refusal the operator would meet at `start_*` must be reported by the
# dry-run path too, UH-3 2026-09-06)
# ═════════════════════════════════════════════════════════════════════════════

def test_check_reports_the_frame_offset_informationally_at_zero_authority():
    """`learner_lateral_authority_mm` explicitly pinned to 0.0 (the launch
    default was 0 through 2026-09-16..20; it is 40 since 2026-09-21, so this
    test states its own pin rather than relying on the default): the offset
    is still measured and reported, but as an INFORMATIONAL line, not a
    refusal — same "always log/report the offset" discipline as the start
    paths' logging, now visible in the `skills/check` response itself."""
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(0.0, name='learner_lateral_authority_mm')])
    _frame_ready(node, plat_x=2.1, plat_y=31.1, cmd_x=0.0, cmd_y=0.0)
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert 'REJECTED_FRAME_OFFSET' not in resp.message
    assert 'frame check:' in resp.message
    assert '+31.2' in resp.message
    assert 'x +2.1' in resp.message and 'y +31.1' in resp.message
    assert 'informational' in resp.message
    assert 'learner_lateral_authority_mm=0' in resp.message


def test_check_reports_the_frame_offset_informationally_when_unevaluable():
    """No `Platform` mocap sample yet, and `learner_lateral_authority_mm`
    explicitly pinned to 0.0 (no longer the launch default since
    2026-09-21): still informational at zero authority, naming what is
    missing rather than a bare 'cannot evaluate'."""
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(0.0, name='learner_lateral_authority_mm')])
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert 'REJECTED_FRAME_OFFSET' not in resp.message
    assert 'frame check: cannot evaluate' in resp.message
    assert 'Platform' in resp.message
    assert 'informational' in resp.message


def test_check_lists_the_frame_offset_as_a_refusal_over_limit_with_authority():
    """`learner_lateral_authority_mm > 0` and an offset over the 5 mm limit:
    `skills/check` must list it as a REFUSAL, the identical
    `REJECTED_FRAME_OFFSET: ...` string `_run_columns` /
    `_run_one_ball` would refuse with — the dress-rehearsal gate is
    fail-closed the same way the powered path is."""
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(40.0, name='learner_lateral_authority_mm')])
    _frame_ready(node, plat_x=2.1, plat_y=31.1, cmd_x=0.0, cmd_y=0.0)
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'REJECTED_FRAME_OFFSET' in resp.message
    assert '+31.2' in resp.message
    assert 'x +2.1' in resp.message and 'y +31.1' in resp.message
    assert node._frame_check_error() in resp.message   # the SAME string


def test_check_lists_the_frame_offset_as_a_refusal_when_unevaluable_with_authority():
    """Same fail-closed rule, the cannot-evaluate branch: with authority > 0
    and no `Platform` sample yet, `skills/check` must refuse rather than stay
    silent — a cannot-evaluate result is never a silent pass."""
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(40.0, name='learner_lateral_authority_mm')])
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'REJECTED_FRAME_OFFSET' in resp.message
    assert 'Platform' in resp.message


def test_check_reports_frame_check_ok_when_within_limit_with_authority():
    """Authority > 0 and the offset is within the 5 mm limit: reported as a
    clean line, same shape as the ladder/box/site 'OK' lines beside it."""
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(40.0, name='learner_lateral_authority_mm')])
    _frame_ready(node, plat_x=3.0, plat_y=0.0, cmd_x=0.0, cmd_y=0.0)
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert 'REJECTED_FRAME_OFFSET' not in resp.message
    assert 'frame check OK' in resp.message


def test_default_parameters_refuse_every_start_path_with_no_frame_data(tmp_path):
    """`learner_lateral_authority_mm` defaults to 40.0 since 2026-09-21
    (`plans/archived/cup-contact-contract.md` § 6, the unpinning) — a session
    that never saw a mocap `Platform` sample must now be refused
    REJECTED_FRAME_OFFSET on EVERY default-parameter start path, not just
    under an explicit `:=40`. This is the production fail-closed face of the
    new default (plan § 1: cannot-evaluate refuses exactly like over-limit).
    `skills/check` must list the identical refusal so the dress rehearsal
    catches it ahead of a powered `start_*` call (UH-3, 2026-09-06)."""
    node, _client = _node_with_client(frame=False)
    node._on_traj_status(_status())
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp_toss = node._start_pattern(_self_toss_goal())
    assert resp_toss.success is False
    assert 'REJECTED_FRAME_OFFSET' in resp_toss.message

    node2, _client2 = _node_with_client(frame=False)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp_columns = node2._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp_columns.success is False
    assert 'REJECTED_FRAME_OFFSET' in resp_columns.message

    node3, _client3 = _node_with_client(frame=False)
    resp_check = node3._svc_check(Trigger.Request(), Trigger.Response())
    assert resp_check.success is False
    assert 'REJECTED_FRAME_OFFSET' in resp_check.message


# ═════════════════════════════════════════════════════════════════════════════
# installer plumbing
# ═════════════════════════════════════════════════════════════════════════════

def test_installer_reports_service_unavailable():
    """The columns bridge executor is wired with `observations=` since R5's
    two-phase start, so the machine must READ as streaming (`_freshen`) or
    the tick ends `ABORTED_MODE_CHANGED` before the installer is ever asked —
    the end code this test is about is the installer's own."""
    node, _client = _node_with_client(ready=False)
    _freshen(node)
    node._start_pattern(_columns_goal(separation_mm=100.0))
    first = node._executor.schedule.skills[0]
    node._executor.tick(first.dispatch_s())
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'SERVICE_UNAVAILABLE'


def test_a_catch_with_throw_terminal_copies_the_carried_release_onto_the_request():
    """A CATCH whose terminal carries ``then_throw`` (Unit G's fold) must reach
    the wire with ``t_release_s`` / ``release_site_mm`` set from it — the SAME
    generic ``target_mm`` / ``flight_s`` fields a plain THROW uses, reused
    here for the carried release (``InstallSegment.srv``'s one pair of fields,
    not a second copy per kind)."""
    from jugglebot.motion.skills.segments import CatchTerminal, ThrowAfterCatch

    node, client = _node_with_client(response=_response())
    then_throw = ThrowAfterCatch(t_release_s=101.0, site_mm=[0.0, 0.0, 860.0],
                                 target_mm=[10.0, -5.0, 860.0], flight_s=0.6)
    terminal = CatchTerminal(landing_mm=[0.0, 0.0, 830.0],
                             landing_vel_mm_s=[0.0, 0.0, -2500.0],
                             t_land_s=100.0, rest_site_mm=[0.0, 0.0, 750.0],
                             then_throw=then_throw)
    node._installer('CATCH', terminal, 100.0, ball_id=1)
    assert len(client.calls) == 1
    req = client.calls[0]
    assert req.t_event_s == pytest.approx(100.0)
    assert req.t_release_s == pytest.approx(101.0)
    assert list(req.release_site_mm) == pytest.approx([0.0, 0.0, 860.0])
    assert list(req.target_mm) == pytest.approx([10.0, -5.0, 860.0])
    assert req.flight_s == pytest.approx(0.6)


def test_a_plain_catch_terminal_leaves_the_release_fields_at_the_sentinel():
    node, client = _node_with_client(response=_response())
    from jugglebot.motion.skills.segments import CatchTerminal

    terminal = CatchTerminal(landing_mm=[0.0, 0.0, 830.0],
                             landing_vel_mm_s=[0.0, 0.0, -2500.0],
                             t_land_s=100.0, rest_site_mm=[0.0, 0.0, 750.0])
    node._installer('CATCH', terminal, 100.0, ball_id=1)
    req = client.calls[0]
    assert req.t_release_s == 0.0


def test_the_installer_propagates_seeded_post_release_from_the_response():
    node, client = _node_with_client(
        response=_response(seeded_post_release=True))
    from jugglebot.motion.skills.segments import RestTerminal

    terminal = RestTerminal(rest_site_mm=[0.0, 0.0, 750.0], t_rest_s=100.0)
    result = node._installer('REST', terminal, 100.0, ball_id=0)
    assert result.seeded_post_release is True


# ═════════════════════════════════════════════════════════════════════════════
# the executor threading fix (item 1) — the single-threaded deadlock this
# replaces, and the re-entry guard the fix's own reentrancy needs
# ═════════════════════════════════════════════════════════════════════════════

def test_install_client_and_tick_timer_share_a_reentrant_group():
    """The wiring pinned so a refactor cannot silently return to a plain,
    non-reentrant group — see `trajectory_node`'s identical pin
    (`test_only_the_hold_service_is_reentrant`)."""
    captured = {}
    orig_client = _MockNodeClass.create_client
    orig_timer = _MockNodeClass.create_timer

    def spy_client(self, *a, **kw):
        captured['client_cbgroup'] = kw.get('callback_group')
        return orig_client(self, *a, **kw)

    def spy_timer(self, *a, **kw):
        captured['timer_cbgroup'] = kw.get('callback_group')
        return orig_timer(self, *a, **kw)

    with patch.object(_MockNodeClass, 'create_client', spy_client), \
         patch.object(_MockNodeClass, 'create_timer', spy_timer):
        sn.SkillNode()

    assert isinstance(captured.get('client_cbgroup'), ReentrantCallbackGroup)
    assert isinstance(captured.get('timer_cbgroup'), ReentrantCallbackGroup)
    assert captured['client_cbgroup'] is captured['timer_cbgroup']


def test_main_runs_a_multi_threaded_executor_not_plain_spin(monkeypatch):
    """A reentrant group is inert under `rclpy.spin` — the executor must
    match, or the single-threaded deadlock item 1 fixes is back (mirrors
    `trajectory_node`'s `test_main_runs_a_multi_threaded_executor_not_plain_spin`)."""
    built = {}

    class _Spy(MultiThreadedExecutor):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            built['threads'] = kw.get('num_threads')

        def spin(self):
            built['spun'] = True

    monkeypatch.setattr(sn, 'MultiThreadedExecutor', _Spy)
    monkeypatch.setattr(rclpy, 'init', lambda *a, **k: None)
    monkeypatch.setattr(rclpy, 'shutdown', lambda *a, **k: None)
    monkeypatch.setattr(rclpy, 'spin',
                        lambda *a, **k: pytest.fail('plain spin is back'),
                        raising=False)

    sn.main()

    assert built.get('spun') is True
    # At least three (finding 4, R3 audit, 2026-09-13): one thread for the
    # default group, one free for the reentrant install-client + tick-timer
    # pair, and one free for the subscriptions' own `MutuallyExclusiveCallback
    # Group` -- with only two, a blocked service call (`_prelevel`) can starve
    # every subscription callback for its duration.
    assert built.get('threads', 0) >= 3


def test_every_subscription_has_its_own_non_default_callback_group():
    """Finding 4, R3 audit (2026-09-13): every `create_subscription` must be
    handed a non-default `MutuallyExclusiveCallbackGroup`, or a blocked
    `_prelevel` service call can starve them all for its duration (rclpy Foxy
    yields ready timers before subscriptions), and the first `_on_tick` can
    then read a stale `trajectory/status` sample and abort
    `ABORTED_MODE_CHANGED` before the opening REST even dispatches."""
    groups = []
    orig = _MockNodeClass.create_subscription

    def spy(self, *a, **kw):
        groups.append(kw.get('callback_group'))
        return orig(self, *a, **kw)

    with patch.object(_MockNodeClass, 'create_subscription', spy):
        sn.SkillNode()
    assert groups and all(isinstance(g, MutuallyExclusiveCallbackGroup)
                          for g in groups)


class _SignallingLock:
    """Wraps a real `threading.Lock`, setting `acquire_started` the instant
    `.acquire()` is entered -- so a test can wait (deterministically, via
    `Event.wait`, never a sleep) for the exact moment `_stop_attempt`'s blocking
    acquire begins, instead of guessing at a delay."""

    def __init__(self, real_lock):
        self._lock = real_lock
        self.acquire_started = threading.Event()

    def acquire(self, blocking=True, timeout=-1):
        self.acquire_started.set()
        return self._lock.acquire(blocking, timeout)

    def release(self):
        self._lock.release()


def test_stop_waits_for_an_in_progress_tick(tmp_path):
    """Finding 12, R3 audit (2026-09-13): `_stop_attempt` must take `_tick_lock`
    before writing `attempt_ended` / `end_code`, bounded by
    `_STOP_LOCK_WAIT_S` — otherwise it can race a tick mid-`_dispatch` on the
    timer thread."""
    node, _c = _node_with_client()
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        node._start_pattern(_columns_goal(separation_mm=100.0))
    real_lock = node._tick_lock
    assert real_lock.acquire(blocking=False)   # simulate a tick mid-install
    wrapped = _SignallingLock(real_lock)
    node._tick_lock = wrapped

    result = {}

    def run():
        result['resp'] = node._stop_attempt()

    th = threading.Thread(target=run)
    th.start()
    try:
        assert wrapped.acquire_started.wait(timeout=5.0), (
            '_stop_attempt never attempted to acquire _tick_lock')
        # The blocking acquire is now queued behind the "in-progress tick" --
        # nothing has been written yet.
        assert node._executor.attempt_ended is False
    finally:
        real_lock.release()
    th.join(timeout=5.0)
    assert not th.is_alive()
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'STOPPED'
    assert result['resp'].success is True


# ═════════════════════════════════════════════════════════════════════════════
# Unit A — an ended attempt must not leave a throwing plan streaming (L1)
# ═════════════════════════════════════════════════════════════════════════════

def _hold_client(success=True, message='holding at current pose'):
    resp = Trigger.Response()
    resp.success = success
    resp.message = message
    return _RecordingClient(response=resp)


def test_an_aborted_attempt_with_a_future_release_pending_holds_once():
    """`ABORTED_NO_RELEASE` (executor-internal, no install call) leaves the
    last ACCEPTED install's release pending — `_on_tick` must install one
    `trajectory/hold` and clear the pending event."""
    node, _client = _node_with_client()
    hold_client = _hold_client()
    node._hold_cli = hold_client
    # On the WIRE's clock (perf_counter), not the ROS tick clock.
    node._pending_event_mono = time.perf_counter() + 5.0
    node._executor = MagicMock()
    node._executor.done = False
    node._executor.attempt_ended = False
    node._executor.end_code = ''

    def _fake_tick(now):
        node._executor.attempt_ended = True
        node._executor.end_code = 'ABORTED_NO_RELEASE'
        return []
    node._executor.tick.side_effect = _fake_tick

    node._on_tick()

    assert len(hold_client.calls) == 1
    assert node._pending_event_mono <= 0.0
    # A second tick (the executor keeps ticking for outcome finalisation)
    # must not install a second hold.
    node._executor.tick.side_effect = lambda now: []
    node._on_tick()
    assert len(hold_client.calls) == 1


def test_a_naturally_completed_attempt_does_not_hold():
    """`attempt_ended` only ever flips on a refusal (executor.py's `done`
    docstring) — a fully successful run never sets it, so a stray pending
    event must not trigger a hold just because `done` went True."""
    node, _client = _node_with_client()
    hold_client = _hold_client()
    node._hold_cli = hold_client
    node._pending_event_mono = time.perf_counter() + 5.0
    node._executor = MagicMock()
    node._executor.done = True
    node._executor.attempt_ended = False
    node._executor.end_code = ''
    node._executor.tick.return_value = []

    node._on_tick()

    assert hold_client.calls == []
    assert node._executor is None


# ═════════════════════════════════════════════════════════════════════════════
# R5 D2/D3 (owner decisions, 2026-09-30) — the end line and the Juggle result
# must carry a columns attempt's clean shadow-landing finish (D2) and a
# dropped-survivor finish (D3), neither of which flips `attempt_ended`.
# ═════════════════════════════════════════════════════════════════════════════

def test_a_clean_columns_finish_logs_the_d2_stop_line(tmp_path):
    """D2: a columns attempt that finishes with nothing refused (`end_code`
    empty, `attempt_ended` never flips — executor.py's `done` docstring)
    leaves the platform holding BOTH balls, not at an ordinary rest — the
    end line must say so, not the generic 'done' tally
    (`report.end_line`'s `shadow_landing_stop`). R5: the shadow-landing
    throw lives in the REAL compiled schedule, only reachable once the
    two-phase start's feed resolves it (`_columns_fed`)."""
    node, _client = _node_with_client()
    _columns_fed(node, tmp_path, plant_id='test_columns_d2_finish')
    executor = node._executor
    # Sanity: the owner-operating-point columns schedule (n_throws >= 2)
    # DOES carry the Stop's cross-site last throw (E1').
    assert sc.schedule_has_shadow_landing(executor.schedule)
    executor.attempt_ended = False
    executor.end_code = ''
    node._logger = MagicMock()
    node.get_logger = lambda: node._logger

    node._log_attempt_end(executor)

    assert node._logger.info.call_count == 1
    assert node._logger.error.call_count == 0
    text = node._logger.info.call_args[0][0]
    assert 'one ball held' in text
    assert 'rests on the platform' in text


def test_a_dropped_survivor_that_finishes_carries_the_drop_code_through():
    """D3 fix (2026-09-30): `DROPPED_SURVIVOR_STOPPED` deliberately leaves
    `attempt_ended` False the whole time (so `tick` keeps dispatching the
    survivor's own tail — `SkillExecutor._advance_release_evidence`), which
    used to make BOTH `_goal_end_code` (`_on_tick`) and the end line
    (`_log_attempt_end`) silently read it back as '' — a plain 'done' INFO
    line and a Juggle result with no end code at all, even though a ball was
    dropped. Both must now carry `DROPPED_SURVIVOR_STOPPED` once the
    survivor's tail itself finishes (`done` True, `attempt_ended` STILL
    False — the D3 contract)."""
    node, _client = _node_with_client()
    node._executor = MagicMock()
    node._executor.done = False
    node._executor.attempt_ended = False
    node._executor.end_code = ''
    node._executor.end_message = ''
    node._executor.end_kind = ''
    node._executor.schedule = MagicMock()
    node._executor.schedule.pattern = 'columns'
    node._executor.stop_terminals.return_value = []
    node._logger = MagicMock()
    node.get_logger = lambda: node._logger

    def _drop_tick(now):
        node._executor.end_code = 'DROPPED_SURVIVOR_STOPPED'
        node._executor.end_message = ('ball 1/3 never left the hand -- ball '
                                      '2 caught, no further throws')
        return []
    node._executor.tick.side_effect = _drop_tick

    node._on_tick()
    # The drop tick alone does not retire the executor (`done` is still
    # False -- the survivor's tail is still streaming), so `_goal_end_code`
    # is not captured yet (`_on_tick` only captures it inside `if
    # self._executor.done:`) and nothing has logged.
    assert node._goal_end_code == ''
    assert node._logger.error.call_count == 0     # not retired yet

    # A later tick: the survivor's own (shortened) tail finishes.
    node._executor.done = True
    node._executor.tick.side_effect = lambda now: []
    node._on_tick()

    assert node._executor is None
    assert node._goal_end_code == 'DROPPED_SURVIVOR_STOPPED'
    assert node._logger.error.call_count == 1
    text = node._logger.error.call_args[0][0]
    assert 'DROPPED_SURVIVOR_STOPPED' in text


def test_stop_with_a_pending_event_holds():
    node, _client = _node_with_client()
    hold_client = _hold_client()
    node._hold_cli = hold_client
    node._pending_event_mono = time.perf_counter() + 5.0
    node._executor = MagicMock()
    node._executor.attempt_ended = False
    node._executor.end_code = ''

    resp = node._stop_attempt()

    assert resp.success is True
    assert len(hold_client.calls) == 1
    assert node._pending_event_mono <= 0.0
    assert node._executor.end_code == 'STOPPED'


def _ending_executor(node, stops, end_code='ABORTED_NO_RELEASE'):
    """A MagicMock executor whose next tick ends the attempt `end_code`, and
    whose `stop_terminals` offers `stops`."""
    node._executor = MagicMock()
    node._executor.done = False
    node._executor.attempt_ended = False
    node._executor.end_code = ''
    node._executor.stop_terminals.return_value = list(stops)

    def _fake_tick(now):
        node._executor.attempt_ended = True
        node._executor.end_code = end_code
        return []
    node._executor.tick.side_effect = _fake_tick


def test_a_pending_release_is_stopped_with_a_rest_not_the_legacy_hold():
    """2026-09-29 (owner): the stop after an empty throw is a REST spliced
    into the running plan -- platform AND hand to rest, the hand at home --
    not the legacy `trajectory/hold` (a platform-only return to the stop
    pose in 0.2 s, refused LIMIT_JERK on all three drops of the R4 gate
    sitting, after which the NEXT empty throw ran anyway)."""
    from jugglebot.motion.skills.segments import RestTerminal

    node, client = _node_with_client(
        response=_response(t_event_mono=0.0, t_release_mono=0.0))
    hold_client = _hold_client()
    node._hold_cli = hold_client
    node._pending_event_mono = time.perf_counter() + 5.0
    before = RestTerminal(rest_site_mm=[125.0, 0.0, 750.0], t_rest_s=1234.5,
                          holds_ball=True)
    _ending_executor(node, [('before', before)])

    node._on_tick()

    assert len(client.calls) == 1
    req = client.calls[0]
    assert req.kind == InstallSegment.Request.KIND_REST
    assert req.t_event_s == pytest.approx(1234.5)
    assert list(req.rest_site_mm) == pytest.approx([125.0, 0.0, 750.0])
    assert hold_client.calls == []
    assert node._pending_event_mono <= 0.0


def test_the_stop_after_an_ended_tick_runs_outside_the_tick_lock():
    """Phase-end audit, 2026-09-29: the stop can now be up to THREE blocking
    round trips (REST before, REST after, legacy hold), and `_on_tick` used to
    run it holding `_tick_lock` -- past the `_STOP_LOCK_WAIT_S` budget a
    concurrent operator Stop waits for. `_on_tick` now stops the machine AFTER
    releasing the lock, as `_stop_attempt` and `_end_attempt` always did."""
    from jugglebot.motion.skills.segments import RestTerminal

    node, client = _node_with_client(
        response=_response(t_event_mono=0.0, t_release_mono=0.0))
    node._hold_cli = _hold_client()
    node._pending_event_mono = time.perf_counter() + 5.0
    _ending_executor(node, [('before', RestTerminal(
        rest_site_mm=[125.0, 0.0, 750.0], t_rest_s=10.0))])
    lock_held = []
    real_call_async = client.call_async

    def recording_call_async(request):
        lock_held.append(node._tick_lock.locked())
        return real_call_async(request)
    client.call_async = recording_call_async

    node._on_tick()

    assert lock_held == [False]


def test_a_refused_stop_rest_tries_the_next_then_the_legacy_hold():
    from jugglebot.motion.skills.segments import RestTerminal

    node, client = _node_with_client(
        response=_response(accepted=False, code='LIMIT_JERK',
                           message='REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK)'))
    hold_client = _hold_client()
    node._hold_cli = hold_client
    node._pending_event_mono = time.perf_counter() + 5.0
    site = [125.0, 0.0, 750.0]
    _ending_executor(node, [
        ('before', RestTerminal(rest_site_mm=site, t_rest_s=10.0)),
        ('after', RestTerminal(rest_site_mm=site, t_rest_s=11.0,
                               holds_ball=False))])

    node._on_tick()

    assert [r.t_event_s for r in client.calls] == [
        pytest.approx(10.0), pytest.approx(11.0)]
    assert len(hold_client.calls) == 1


def test_a_refused_hand_lane_holds_without_trying_a_rest():
    """The forced hold (HAND_LANE_REFUSED) stays the HAND-LESS legacy hold:
    the firmware is holding a refused hand lane, and a REST carries a hand
    track that would keep walking the deviation the guard measures."""
    from jugglebot.motion.skills.segments import RestTerminal

    node, client = _node_with_client()
    hold_client = _hold_client()
    node._hold_cli = hold_client
    node._force_hold = True
    _ending_executor(node, [('before', RestTerminal(
        rest_site_mm=[0.0, 0.0, 750.0], t_rest_s=10.0))],
        end_code='HAND_LANE_REFUSED')
    node._hold_forced_code = 'HAND_LANE_REFUSED'

    node._on_tick()

    assert client.calls == []
    assert len(hold_client.calls) == 1


def test_a_throw_install_updates_the_pending_event_and_a_rest_clears_it():
    """`_installer` (Unit A) tracks the SAME release instant `_maybe_announce`
    already computes — a THROW's `t_event_mono`; a subsequent accepted REST
    (no release) clears it."""
    from jugglebot.motion.skills.segments import RestTerminal, ThrowTerminal

    node, _client = _node_with_client(
        response=_response(t_event_mono=50.0, t_release_mono=0.0))
    throw_terminal = ThrowTerminal(site_mm=[0.0, 0.0, 860.0],
                                   target_mm=[0.0, 0.0, 830.0], flight_s=0.6,
                                   t_release_s=100.0)
    node._installer('THROW', throw_terminal, 100.0, ball_id=0)
    assert node._pending_event_mono == pytest.approx(50.0)

    node._install_cli._response = _response(t_event_mono=0.0, t_release_mono=0.0)
    rest_terminal = RestTerminal(rest_site_mm=[0.0, 0.0, 750.0], t_rest_s=101.0)
    node._installer('REST', rest_terminal, 101.0, ball_id=0)
    assert node._pending_event_mono == 0.0


def test_a_tick_already_in_progress_does_not_start_a_second_dispatch():
    """`_on_tick`'s re-entry guard: a `ReentrantCallbackGroup` lets the
    executor run this callback again on another thread while the first
    invocation is still blocked in `_installer._wait_future` — this must be a
    no-op, not a second dispatch on top of one already in flight."""
    node, _client = _node_with_client()
    node._executor = MagicMock()
    node._executor.done = False
    assert node._tick_lock.acquire(blocking=False)   # simulate an in-progress tick
    try:
        node._on_tick()
    finally:
        node._tick_lock.release()
    node._executor.tick.assert_not_called()


# ═════════════════════════════════════════════════════════════════════════════
# catch_aim_source — the tracker-aimed catch (owner decision 2026-09-18)
# ═════════════════════════════════════════════════════════════════════════════


def test_the_live_catch_aim_default_is_the_trackers_converged_fit():
    """THE LIVE DEFAULT (owner 2026-09-18).  The catch follows the tracker's
    converged ballistic fit, with the schedule's commanded landing as the
    prior at dispatch: the release lags its knot by 0.019-0.137 s throw to
    throw, which nothing can learn away, and the tracker has confirmed every
    flight since `ce6d603` (22/22, 2026-09-17).  The 2026-09-15 `NO_LANDING`
    sitting is not repeated because no step of that order WAITS — that is
    `executor._catch_aim`'s job, pinned in
    `tests/motion/test_skills_executor.py`; what this test pins is that the
    node ships the tracker aim rather than the open-loop A/B arm."""
    node, _client = _node_with_client()
    assert node.get_parameter('catch_aim_source').value == 'tracker'
    node._start_pattern(_columns_goal(separation_mm=100.0))
    assert node._executor.catch_aim_source == 'tracker'
    assert node._executor.launch_ratio is not None


@pytest.mark.parametrize('value', ['schedule', 'schedule_hand'])
def test_the_aim_source_parameter_reaches_the_executor(value):
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter(value, name='catch_aim_source')])
    node._start_pattern(_columns_goal(separation_mm=100.0))
    assert node._executor.catch_aim_source == value


def test_an_unknown_aim_source_falls_back_to_the_live_default():
    """A typo in a launch override must not leave the operator with no catch
    at all, and must not silently fly the OTHER arm of an A/B: the fallback
    is the live default, which is the only value a session can fly without
    having chosen it."""
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter('mocap', name='catch_aim_source')])
    node._start_pattern(_columns_goal(separation_mm=100.0))
    assert node._executor.catch_aim_source == 'tracker'


def test_hand_telemetry_feeds_the_launch_ratio_the_executor_asks_for():
    """`/hand_telemetry` is the ONLY source of the measured correction (never
    QTM): the node's `launch_ratio` callable must answer from the samples that
    topic delivered, and answer `None` before any stroke has been seen."""
    node, _client = _node_with_client()
    t_release = 1000.0
    assert node._launch_ratio(0, t_release) is None      # nothing recorded yet

    # One throw stroke delivered through the REAL callback, sample by sample.
    # Each sample is stamped with the node's wall clock at arrival (the clock
    # the schedule's release instants live on), so the mock clock is stepped
    # rather than left at 0 — an unstamped sample is dropped by design, since
    # it cannot be compared with a release instant.
    n, dt = 20, 0.01
    for i in range(n):
        t = t_release - (n - 1 - i) * dt
        node._clock = MagicMock()
        node._clock.now.return_value = MockTime(int(round(t * 1e9)))
        cmd = 120.0 * (i + 1) / n
        node._on_hand_telemetry(HandTelemetryMessage(
            vel_ff_cmd=cmd, vel_meas=cmd * 1.086, ball_held_valid=False))
    assert len(node._hand_launch) == n
    assert node._launch_ratio(0, t_release) == pytest.approx(1.086, abs=1e-6)


def test_a_refused_memory_row_is_logged_and_never_raises():
    """`Memory.append` refuses a row outside `APEX_RATIO_BAND` (2026-09-16,
    in apex since 2026-09-18).
    The executor drops such a row first, so this is the belt-and-braces path —
    and it runs inside the tick loop, where an exception would surface as an
    executor fault and end an attempt over bookkeeping."""
    import numpy as np
    from jugglebot.motion.skills.memory import Experience

    node, _client = _node_with_client()
    errors = []
    node.get_logger().error = errors.append

    class _RefusingMemory:
        def append(self, _exp):
            raise ValueError('refusing a row whose observed apex 6.1079 m '
                             'is outside [0.2250, 2.3040] of the commanded '
                             '0.9000 m')

    on_experience = node._bind_on_experience(_RefusingMemory())
    exp = Experience(x=np.zeros(4), u=np.array([0.0, 0.0, 0.8569]),
                     y=np.array([0.0, 0.0, 2.2317]), t_abs_s=1.0, ball_id=0,
                     caught=True)
    on_experience(exp)                       # must not raise
    assert len(errors) == 1
    assert 'memory row REFUSED (not appended)' in errors[0]
    assert '2.2317' in errors[0]


# ═════════════════════════════════════════════════════════════════════════════
# The opening REST homes the hand (2026-09-18 evening)
# ═════════════════════════════════════════════════════════════════════════════
# THE DEFECT, twice over. (1) bag `2026-09-18_13-25-15`, three MAX_DEVIATION
# latches: an attempt ended `ABORTED_NO_RELEASE` with the hand at 9.63 rev — the
# top of the stroke — and the next schedule's opening REST, seeded correctly
# from that encoder, planned it home over its fixed 1.5 s window (~5.4 rev/s)
# against a firmware hand group that was HOLDING; the refused command ran
# 3.28 rev from the encoder and the guard E-STOPPED 0.61 s in. (2) that day's
# first fix — a blocking `/park_hand` plus a park-band precondition — refused
# every attempt after the first, because the band was measured against the
# ACTIVATE park (0.0 rev) while a schedule's REST correctly leaves the hand at
# `sites.REST_HAND_REV` (0.3071 rev). ONE home, and the REST's PERIOD is sized
# to the displacement (`schedule.floor_lift_s`).

def test_the_opening_rest_is_sized_to_home_a_displaced_hand():
    """The 2026-09-18 hand itself: 9.6227 rev accepts, and the schedule's own
    first window is the sized period, not the 1.5 s floor."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    _hand_at(node, rev=9.6227)
    node._params['plant_id'] = 'test_opening_rest_sized'

    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is True, resp.message
    rest0 = node._executor.schedule.skills[0]
    assert rest0.kind == sn.REST
    want = sn.floor_lift_s(9.6227)
    assert want == pytest.approx(6.9867, abs=1e-3)
    assert rest0.window_s == pytest.approx(want)
    # ... and the whole schedule follows it: THROW 0 releases `launch_s` after
    # the sized REST, not after the floor.
    assert (node._executor.schedule.skills[1].t_abs_s
            - rest0.t_abs_s) == pytest.approx(0.4)


def test_a_hand_already_at_home_gets_the_default_opening_rest():
    """Where the previous schedule's REST left it (0.3071 rev) ⇒ nothing
    changes: the floor period, and no refusal anywhere."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    _hand_at(node, rev=sn.REST_HAND_REV)
    node._params['plant_id'] = 'test_opening_rest_default'

    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is True, resp.message
    assert node._executor.schedule.skills[0].window_s == pytest.approx(
        sc.FLOOR_LIFT_S)


def test_the_activate_park_is_not_a_special_case():
    """A hand at the bridge's ACTIVATE park (0.0 rev) is simply 0.307 rev from
    home: accepted, sized, and NOT refused — the regression the one-day-old
    park-band precondition produced in reverse."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    _hand_at(node, rev=0.0)
    node._params['plant_id'] = 'test_opening_rest_from_park'

    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is True, resp.message
    assert node._executor.schedule.skills[0].window_s == pytest.approx(
        sc.FLOOR_LIFT_S)


def test_the_homing_rest_is_refused_when_the_hand_position_is_unread():
    """The ONE refusal left, and it is pre-motion: a window sized from a stale
    encoder is the same defect as a seed reconciled against one."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    prelevel = _prelevel_ready(node)
    node._hand_telemetry_mono -= sn._HAND_STATE_STALE_S + 0.1

    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'hand_telemetry is stale or absent' in resp.message
    assert node._executor is None
    assert prelevel.calls == [], 'moved the platform on an unread hand'


def test_the_sized_rest_is_logged_with_its_own_peaks():
    """One INFO line the operator can read the sizing off."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    _hand_at(node, rev=9.6227)
    node._params['plant_id'] = 'test_opening_rest_log'
    lines = []
    node.get_logger().debug = lambda m: lines.append(m)   # detail: DEBUG since console phase 2

    node._start_pattern(_self_toss_goal())
    homing = [ln for ln in lines if 'opening REST homes the hand' in ln]
    assert len(homing) == 1, lines
    assert '+9.6227' in homing[0] and '+0.3071' in homing[0]
    assert '6.99 s' in homing[0]


def test_columns_homes_a_displaced_hand_via_the_opening_rest_bridge(tmp_path):
    """R5 (owner decision D1, 2026-09-30): columns' first skill is now the
    opening REST BRIDGE (`compile_reload_wait(..., holds_ball=True)`,
    `_run_columns`'s own two-phase start) — the SAME `_opening_rest_period`
    sizing self_toss/hop already use, not the pre-R5 pre-motion refusal
    (`_hand_home_error`, deleted, "columns has no opening REST"). Mirrors
    `test_the_opening_rest_is_sized_to_home_a_displaced_hand`."""
    node, _client = _node_with_client()
    _hand_at(node, rev=9.6227)
    node._params['plant_id'] = 'test_columns_opening_rest_sized'

    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    rest0 = node._executor.schedule.skills[0]
    assert rest0.kind == sn.REST
    want = sn.floor_lift_s(9.6227)
    assert want == pytest.approx(6.9867, abs=1e-3)
    assert rest0.window_s == pytest.approx(want)


def test_columns_starts_with_the_hand_at_home(tmp_path):
    node, _client = _node_with_client()

    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    assert node._executor is not None


# ═════════════════════════════════════════════════════════════════════════════
# Fail closed on a REFUSED hand lane (2026-09-18 evening)
# ═════════════════════════════════════════════════════════════════════════════

def _link_status(sched_refused):
    return DiagnosticStatus(values=[
        KeyValue(key='fault_state', value='NONE'),
        KeyValue(key='sched_refused', value=str(int(sched_refused)))])


def _running_self_toss(node, tmp_path):
    """A started self-toss attempt with the WHOLE ladder fresh, so the only
    thing that can end it is the one fact under test. Returns the executor
    object itself: `_on_tick` retires a finished one off the node."""
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _prelevel_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is True, resp.message
    return node._executor


def test_a_sched_refused_bump_ends_the_attempt_and_installs_the_hold(tmp_path):
    """The firmware refused the streamed lane and is HOLDING the hand: the rest
    tail is not a safe end (every knot the plan walks on widens the deviation
    the guard measures), so the attempt ENDS and a HAND-LESS hold goes in."""
    node, _client = _node_with_client(response=_response())
    node._on_link_status(_link_status(4))
    node._params['plant_id'] = 'test_lane_refused'
    execu = _running_self_toss(node, tmp_path)
    hold = _hold_client()
    node._hold_cli = hold

    node._on_link_status(_link_status(5))
    node._on_tick()

    assert execu.attempt_ended is True
    assert execu.end_code == ex.HAND_LANE_REFUSED
    assert len(hold.calls) == 1
    # Once per attempt, not once per tick.
    node._on_tick()
    assert len(hold.calls) == 1


def test_a_counter_already_high_at_the_start_is_history(tmp_path):
    """A refusal a PREVIOUS attempt provoked must not end this one — the
    baseline is latched when the schedule is compiled."""
    node, _client = _node_with_client(response=_response())
    node._on_link_status(_link_status(7))
    node._params['plant_id'] = 'test_lane_refused_baseline'
    execu = _running_self_toss(node, tmp_path)
    hold = _hold_client()
    node._hold_cli = hold

    node._on_link_status(_link_status(7))
    node._on_tick()

    assert execu.attempt_ended is False
    assert hold.calls == []


def test_a_bridge_that_never_publishes_the_counter_changes_nothing(tmp_path):
    """No `sched_refused` key (an older bridge) ⇒ unobserved ⇒ no refusal: the
    sizing is what keeps the lane inside the envelope, and this is the
    defence behind it."""
    node, _client = _node_with_client(response=_response())
    node._on_link_status(DiagnosticStatus(values=[
        KeyValue(key='fault_state', value='NONE')]))
    node._params['plant_id'] = 'test_lane_refused_absent'
    execu = _running_self_toss(node, tmp_path)

    node._on_tick()
    assert execu.attempt_ended is False
    assert node._observations(0.0).hand_lane_refused is False


# ═════════════════════════════════════════════════════════════════════════════
# session-start mocap-vs-commanded frame check (plan cup-contact-contract.md § 1)
# ═════════════════════════════════════════════════════════════════════════════

def test_frame_offset_check_hand_computed_case():
    """The plan's own worked example (`cup-contact-contract.md` § 1): a
    mean Platform sample of (2.1, 31.1) mm against a commanded position at
    the origin -> offset +31.2 mm, over the 5 mm limit. The mean is a real
    average, not a constant sample: ten samples at (2.0, 31.0) and ten at
    (2.2, 31.2)."""
    now = 100.0
    platform = ([(now - 0.5, 2.0, 31.0)] * 10
               + [(now - 0.1, 2.2, 31.2)] * 10)
    commanded = [(now - 0.2, 0.0, 0.0)] * 5
    result = sn._frame_offset_check(platform, commanded, now)
    assert result.evaluable is True
    assert result.within_limit is False
    assert result.dx_mm == pytest.approx(2.1)
    assert result.dy_mm == pytest.approx(31.1)
    assert result.offset_mm == pytest.approx(31.17, abs=0.01)
    assert '+31.2 mm' in result.detail
    assert 'x +2.1' in result.detail and 'y +31.1' in result.detail


def test_frame_offset_check_within_limit():
    now = 100.0
    platform = [(now - 0.1, 3.0, 0.0)] * (sn._FRAME_CHECK_MIN_SAMPLES + 5)
    commanded = [(now - 0.1, 0.0, 0.0)] * 5
    result = sn._frame_offset_check(platform, commanded, now)
    assert result.evaluable is True
    assert result.within_limit is True
    assert result.offset_mm == pytest.approx(3.0)


def test_frame_offset_check_refuses_with_no_platform_body():
    result = sn._frame_offset_check([], [(100.0, 0.0, 0.0)], 100.0)
    assert result.evaluable is False
    assert 'Platform' in result.detail


def test_frame_offset_check_refuses_when_the_platform_sample_is_stale():
    now = 100.0
    platform = [(now - (sn._FRAME_CHECK_WINDOW_S + 4.0), 0.0, 0.0)] * (
        sn._FRAME_CHECK_MIN_SAMPLES + 5)
    commanded = [(now - 0.1, 0.0, 0.0)] * 5
    result = sn._frame_offset_check(platform, commanded, now)
    assert result.evaluable is False
    assert 'stale' in result.detail


def test_frame_offset_check_refuses_with_too_few_platform_samples():
    now = 100.0
    platform = [(now - 0.1, 0.0, 0.0)] * (sn._FRAME_CHECK_MIN_SAMPLES - 1)
    commanded = [(now - 0.1, 0.0, 0.0)] * 5
    result = sn._frame_offset_check(platform, commanded, now)
    assert result.evaluable is False
    assert 'samples' in result.detail


def test_frame_offset_check_refuses_when_commanded_position_is_stale():
    now = 100.0
    platform = [(now - 0.1, 0.0, 0.0)] * (sn._FRAME_CHECK_MIN_SAMPLES + 5)
    commanded = [(now - (sn._FRAME_CHECK_WINDOW_S + 4.0), 0.0, 0.0)]
    result = sn._frame_offset_check(platform, commanded, now)
    assert result.evaluable is False
    assert 'commanded_position' in result.detail


def test_frame_offset_check_refuses_when_commanded_position_is_not_at_rest():
    now = 100.0
    platform = [(now - 0.1, 0.0, 0.0)] * (sn._FRAME_CHECK_MIN_SAMPLES + 5)
    commanded = [(now - 0.5, 0.0, 0.0), (now - 0.1, 2.0, 0.0)]  # 2 mm spread
    result = sn._frame_offset_check(platform, commanded, now)
    assert result.evaluable is False
    assert 'not at rest' in result.detail


def test_frame_check_within_limit_starts_columns_with_authority(tmp_path):
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(40.0, name='learner_lateral_authority_mm')])
    _frame_ready(node, plat_x=3.0, plat_y=0.0, cmd_x=0.0, cmd_y=0.0)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is True


def test_frame_check_over_limit_refuses_columns_with_the_number(tmp_path):
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(40.0, name='learner_lateral_authority_mm')])
    _frame_ready(node, plat_x=2.1, plat_y=31.1, cmd_x=0.0, cmd_y=0.0)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is False
    assert 'REJECTED_FRAME_OFFSET' in resp.message
    assert '+31.2' in resp.message
    assert 'x +2.1' in resp.message and 'y +31.1' in resp.message


def test_frame_check_with_zero_authority_starts_and_logs_the_offset(tmp_path):
    """31 mm, but `learner_lateral_authority_mm` explicitly pinned to 0.0
    (no longer the launch default since 2026-09-21) — the schedule still
    starts, and every sitting still gets the offset line (plan § 1: "always
    log the offset")."""
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(0.0, name='learner_lateral_authority_mm')])
    _frame_ready(node, plat_x=2.1, plat_y=31.1, cmd_x=0.0, cmd_y=0.0)
    lines = []
    node.get_logger().debug = lambda m: lines.append(m)   # detail: DEBUG since console phase 2
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is True
    frame_lines = [ln for ln in lines if ln.startswith('frame check:')]
    assert len(frame_lines) == 1, lines
    assert '+31.2' in frame_lines[0]


def test_frame_check_cannot_evaluate_refuses_naming_the_missing_input(tmp_path):
    """No `Platform` body has ever been seen (`node._platform_mocap_xy`
    stays empty) — fail-closed: cannot-evaluate refuses exactly like
    over-limit, and names what is missing."""
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(40.0, name='learner_lateral_authority_mm')])
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is False
    assert 'REJECTED_FRAME_OFFSET' in resp.message
    assert 'Platform' in resp.message


def test_frame_check_refuses_when_the_commanded_position_is_moving(tmp_path):
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(40.0, name='learner_lateral_authority_mm')])
    now = time.perf_counter()
    for _ in range(sn._FRAME_CHECK_MIN_SAMPLES + 5):
        node._platform_mocap_xy.append((now, 0.0, 0.0))
    node._commanded_xy_hist.append((now - 0.5, 0.0, 0.0))
    node._commanded_xy_hist.append((now, 2.0, 0.0))  # 2 mm > 1 mm tolerance
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is False
    assert 'REJECTED_FRAME_OFFSET' in resp.message
    assert 'not at rest' in resp.message


def test_on_mocap_buffers_the_platform_body_xy():
    """`_on_mocap` (extended, plan § 1) appends the Platform body's xy to
    the frame check's buffer and ignores every other body."""
    node, _client = _node_with_client(hand=False, frame=False)
    node._on_mocap(RigidBodyPoses(bodies=[
        RigidBodyPose(name='Base', pose=_pose_at(10.0, 20.0)),
        RigidBodyPose(name='Platform', pose=_pose_at(3.0, 4.0)),
    ]))
    assert len(node._platform_mocap_xy) == 1
    _t, x, y = node._platform_mocap_xy[-1]
    assert (x, y) == (3.0, 4.0)


def test_on_commanded_position_buffers_xy_history():
    node, _client = _node_with_client(hand=False, frame=False)
    node._on_commanded_position(Point(x=5.0, y=6.0, z=830.0))
    assert len(node._commanded_xy_hist) == 1
    _t, x, y = node._commanded_xy_hist[-1]
    assert (x, y) == (5.0, 6.0)


# ═════════════════════════════════════════════════════════════════════════════
# The frame offset is SUBTRACTED from tracker landings (2026-09-23)
# ─────────────────────────────────────────────────────────────────────────────
# 2026-09-22 sitting: with QTM re-aligned to the base the Platform body sat
# (-1.56, -8.53) mm from the command, sd 0.03 mm over 77 s, invariant under
# relocating the base — a lever arm (574.3 mm x the levelling pose offset
# (0.015, 0.002) rad = (8.6, 1.1) mm), not alignment noise. The old 5 mm limit
# refused Block B on it. A learner fed raw mocap landings would have "corrected"
# it into an 8.5 mm real miss, so the measured offset is now subtracted at the
# one point tracker landings enter the node (`_on_balls`), the limit is a
# 25 mm sanity bound, and a moving Platform body refuses to evaluate.
# ═════════════════════════════════════════════════════════════════════════════

def test_frame_offset_check_refuses_a_moving_platform_body():
    """A Platform body whose samples spread 5 mm inside the window is not a
    session-start number — refuse to evaluate, naming the spread."""
    now = 100.0
    n = sn._FRAME_CHECK_MIN_SAMPLES + 5
    platform = ([(now - 0.5, 0.0, 0.0)] * n + [(now - 0.1, 0.0, 5.0)] * n)
    commanded = [(now - 0.1, 0.0, 0.0)] * 5
    result = sn._frame_offset_check(platform, commanded, now)
    assert result.evaluable is False
    assert 'Platform body moved 5.00 mm' in result.detail
    assert result.plat_spread_mm == pytest.approx(5.0)


def test_frame_offset_check_reports_the_body_spread():
    now = 100.0
    platform = ([(now - 0.5, -1.5, -8.5)] * 15 + [(now - 0.1, -1.6, -8.6)] * 15)
    commanded = [(now - 0.1, 0.0, 0.0)] * 5
    result = sn._frame_offset_check(platform, commanded, now)
    assert result.evaluable is True
    assert result.within_limit is True            # 8.7 mm < the 25 mm sanity bound
    assert result.plat_spread_mm == pytest.approx(math.hypot(0.1, 0.1))
    assert 'body spread 0.14 mm' in result.detail


def test_the_2026_09_22_lever_arm_offset_is_adopted_not_refused():
    """The sitting's own number: (-1.56, -8.53) mm with authority 40 starts
    (it is under the 25 mm bound) AND becomes the correction."""
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(40.0, name='learner_lateral_authority_mm')])
    _frame_ready(node, plat_x=-1.56, plat_y=-8.53, cmd_x=0.0, cmd_y=0.0)
    lines = []
    node.get_logger().debug = lambda m: lines.append(m)   # detail: DEBUG since console phase 2
    assert node._frame_check_error() == ''
    assert node._mocap_to_schedule_mm == pytest.approx((-1.56, -8.53))
    corr = [ln for ln in lines if ln.startswith('tracker landings are now corrected')]
    assert len(corr) == 1, lines
    assert 'x +1.6' in corr[0] and 'y +8.5' in corr[0]


def test_on_balls_subtracts_the_adopted_offset_from_landing_xy_only():
    """A ball the tracker says landed at (-51.56, -8.53) — i.e. exactly on the
    cup, which physically sits at command + offset — reads as the commanded
    site (-50, 0) once the offset is subtracted; z and velocity untouched."""
    node, _client = _node_with_client(frame=False)
    _frame_ready(node, plat_x=-1.56, plat_y=-8.53, cmd_x=0.0, cmd_y=0.0)
    node._frame_check_error()
    node._on_balls(BallStateArray(balls=[
        _ball(7, status=1, x=-51.56, y=-8.53, z=830.0, vx=1.0, vy=2.0,
              vz=-2500.0, sec=10)]))
    landing = node._balls[7]
    assert landing.pos_mm[0] == pytest.approx(-50.0)
    assert landing.pos_mm[1] == pytest.approx(0.0)
    assert landing.pos_mm[2] == pytest.approx(830.0)
    assert landing.vel_mm_s.tolist() == pytest.approx([1.0, 2.0, -2500.0])


def test_on_balls_passes_landings_through_before_any_frame_measurement():
    node, _client = _node_with_client(frame=False)
    assert node._mocap_to_schedule_mm is None
    node._on_balls(BallStateArray(balls=[
        _ball(7, status=1, x=-51.56, y=-8.53, z=830.0, sec=10)]))
    assert node._balls[7].pos_mm.tolist() == pytest.approx([-51.56, -8.53, 830.0])


def test_an_over_limit_or_unevaluable_check_keeps_the_earlier_correction():
    """A stale good number beats none: a later over-limit (wrong alignment)
    or cannot-evaluate result leaves the adopted correction in place and
    warns, rather than zeroing it under the learner."""
    node, _client = _node_with_client(frame=False)
    node.set_parameters(
        [_MockParameter(0.0, name='learner_lateral_authority_mm')])
    _frame_ready(node, plat_x=-1.56, plat_y=-8.53, cmd_x=0.0, cmd_y=0.0)
    node._frame_check_error()
    assert node._mocap_to_schedule_mm == pytest.approx((-1.56, -8.53))
    warnings = []
    node.get_logger().warning = lambda m: warnings.append(m)
    node._platform_mocap_xy.clear()
    _frame_ready(node, plat_x=2.1, plat_y=31.1, cmd_x=0.0, cmd_y=0.0)
    node._frame_check_error()                       # over the 25 mm bound
    assert node._mocap_to_schedule_mm == pytest.approx((-1.56, -8.53))
    node._platform_mocap_xy.clear()                 # cannot evaluate
    node._frame_check_error()
    assert node._mocap_to_schedule_mm == pytest.approx((-1.56, -8.53))
    assert len(warnings) == 2 and all('NOT updated' in w for w in warnings)
    assert 'keeping the earlier (x +1.6, y +8.5) mm' in warnings[0]


# ═════════════════════════════════════════════════════════════════════════════
# R4 reload (brief step 1: skill_node's BB orchestration)
# ═════════════════════════════════════════════════════════════════════════════

def _good_box_path(tmp_path, *, apex_band_m=(0.85, 0.95), hop=False,
                   columns=False):
    """A P1/P1 self_toss box at the LIVE gate hash and the session's
    operating limits (300/5000/200000/3500 since the R5 200 k ramp, 2026-09-30) — mirrors
    `test_self_toss_refuses_an_uncovered_apex_before_any_motion`'s box
    exactly, so a reload test's shared refusal chain (identical to the
    self-toss path up to `_start_reload`) passes without depending on
    `config/generated/admissible_box.yaml`, which a concurrent sweep may be
    mid-rewrite of (brief_common.md: "a background sweep is writing
    config/generated/admissible_box.yaml (do not touch it or tools/)")."""
    box_path = tmp_path / 'admissible_box.yaml'
    # `dwell_s=sn._DEFAULT_DWELL_S` (0.30): `check_limits` now enforces the live
    # session dwell against the box's own stamp too (R5, D4, 2026-09-30) --
    # every box this file hands the node must carry the SAME dwell the node
    # itself compiles with, or every caller of this helper refuses
    # `LimitsMismatch: ... dwell_s ...` before it reaches whatever the test
    # actually means to exercise (measured: unstamped boxes here broke 50
    # tests across this file the moment the node started passing `dwell_s`).
    box = adm.AdmissibleBox(
        site_pair=('P1', 'P1'), apex_band_m=apex_band_m,
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), apex_m=(0.6, 1.2),
        pattern='self_toss', release_site_xy_mm=(-50.0, 0.0),
        target_site_xy_mm=(-50.0, 0.0),
        limits={'leg_vel_mmps': 300.0, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': 3500.0},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13',
        dwell_s=sn._DEFAULT_DWELL_S)
    boxes = [box]
    if hop:
        # The 250 mm hop's two boxes as the 2026-09-23 sweep produced them
        # (release/target xy at +-125 mm), so a check/start at the node's
        # separation default finds them without the committed yaml.
        for rel, tgt, names in (((-125.0, 0.0), (125.0, 0.0), ('P1', 'P2')),
                                ((125.0, 0.0), (-125.0, 0.0), ('P2', 'P1'))):
            boxes.append(adm.AdmissibleBox(
                site_pair=names, apex_band_m=apex_band_m,
                landing_xy_m=((-0.02, 0.02), (-0.02, 0.02)), apex_m=(0.85, 0.90),
                pattern='hop', release_site_xy_mm=rel, target_site_xy_mm=tgt,
                limits={'leg_vel_mmps': 300.0, 'leg_acc_mmps2': 5000.0,
                       'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': 3500.0},
                gate_hash=adm.gate_hash(), swept_at='2026-09-13',
                dwell_s=sn._DEFAULT_DWELL_S))
    if columns:
        # R5 (owner decision D1, 2026-09-30): `compile_columns` folds each
        # ball into its own SAME-site chain, so it needs (P1,P1)/(P2,P2) --
        # `columns_sites(100.0)`'s own xy (+-50 mm), not the 250 mm hop
        # separation above.
        for name, xy in (('P1', (-50.0, 0.0)), ('P2', (50.0, 0.0))):
            boxes.append(adm.AdmissibleBox(
                site_pair=(name, name), apex_band_m=apex_band_m,
                landing_xy_m=((-0.02, 0.02), (-0.02, 0.02)), apex_m=(0.85, 0.95),
                pattern='columns', release_site_xy_mm=xy, target_site_xy_mm=xy,
                limits={'leg_vel_mmps': 300.0, 'leg_acc_mmps2': 5000.0,
                       'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': 3500.0},
                gate_hash=adm.gate_hash(), swept_at='2026-09-13',
                dwell_s=sn._DEFAULT_DWELL_S))
    adm.dump(str(box_path), boxes)
    return str(box_path)


def _bb_throw_response(*, success=True, message='ok', predicted_tof_s=0.7,
                       throw_delay_s=3.0):
    resp = BallButlerThrow.Response()
    resp.success = success
    resp.message = message
    resp.predicted_tof_s = predicted_tof_s
    resp.throw_delay_s = throw_delay_s
    return resp


def _reload_ready(node, *, reload_ok=True, reload_message='Reload command sent.',
                  throw_ok=True, throw_message='ok', predicted_tof_s=0.7,
                  throw_delay_s=3.0):
    """Wire `node`'s two BB clients to succeed (the default) so
    `_start_reload` reaches `_reload_ctx` -- mirrors `_prelevel_ready`'s
    role for the pre-level move. Returns ``(reload_client, throw_client)``
    so a test can inspect the requests each received."""
    reload_resp = Trigger.Response()
    reload_resp.success = reload_ok
    reload_resp.message = reload_message
    reload_client = _RecordingClient(response=reload_resp, ready=True)
    throw_client = _RecordingClient(
        response=_bb_throw_response(success=throw_ok, message=throw_message,
                                    predicted_tof_s=predicted_tof_s,
                                    throw_delay_s=throw_delay_s),
        ready=True)
    node._reload_cli = reload_client
    node._bb_throw_cli = throw_client
    return reload_client, throw_client


def _bb_announcement(*, target_id, landing_mm, landing_vel_mm_s,
                     throw_time_s, landing_time_s, thrower_name='ball_butler'):
    """A real `ThrowAnnouncement` -- `_on_announcement` reads `msg.throw_time
    .sec/.nanosec` etc, which only a real (or equally strict) sub-message
    supports; mutating the auto-constructed nested `Time` fields in place
    (rather than assigning a duck-typed stand-in) is what a real rosidl
    message accepts."""
    ann = ThrowAnnouncement()
    ann.thrower_name = thrower_name
    ann.target_id = target_id
    ann.landing_position = Point(x=float(landing_mm[0]), y=float(landing_mm[1]),
                                 z=float(landing_mm[2]))
    ann.landing_velocity = Vector3(x=float(landing_vel_mm_s[0]),
                                   y=float(landing_vel_mm_s[1]),
                                   z=float(landing_vel_mm_s[2]))
    ann.throw_time = _Time(sec=int(throw_time_s),
                          nanosec=int(round((throw_time_s - int(throw_time_s))
                                            * 1e9)))
    ann.landing_time = _Time(sec=int(landing_time_s),
                            nanosec=int(round((landing_time_s
                                              - int(landing_time_s)) * 1e9)))
    return ann


def _bb_ready(node, *, state=None, ball_in_hand=True, connected=True,
              after_check=True):
    """Drive `node`'s cached Ball Butler state to "ready" (C1, 2026-09-28):
    `state=None` defaults to `BallButlerStates.IDLE` -- the state
    `_maybe_fire_reload_throw` fires on. Goes through `_on_bb_heartbeat`
    itself (not the raw fields) so a test exercises the same subscription
    path the real `bb/heartbeat` topic drives.

    ``after_check`` (default) sends the CHECKING_BALL heartbeat Ball Butler
    really publishes between a `bb/reload` and its return to IDLE (bag
    2026-09-28_21-41-10: 1.0 s of CHECKING_BALL after every reload) before
    the IDLE one -- a reload that sent `bb/reload` fires only on an IDLE
    that FOLLOWS a non-IDLE state (the 2026-09-28 21:41 race fix)."""
    if state is None:
        state = sn.proto.BallButlerStates.IDLE
    if after_check and int(state) == int(sn.proto.BallButlerStates.IDLE):
        node._on_bb_heartbeat(BallButlerHeartbeat(
            state=int(sn.proto.BallButlerStates.CHECKING_BALL),
            ball_in_hand=bool(ball_in_hand), connected=bool(connected)))
    node._on_bb_heartbeat(BallButlerHeartbeat(
        state=int(state), ball_in_hand=bool(ball_in_hand),
        connected=bool(connected)))


def _start_reload(node, tmp_path, **reload_kw):
    """Drive `node` through the full R4 reload start path (a `reload=True`
    Juggle goal, the shared refusal chain wired ready, both BB clients
    succeeding by default) and return the goal-accept result.

    C1 (2026-09-28): `bb/throw_at_target` no longer fires inline inside the
    goal accept -- `_start_pattern` only gets as far as arming
    `await_bb_idle`. Every EXISTING caller of this helper was written
    against the pre-C1 shape (both BB clients already called by the time it
    returns), so once the goal is accepted into that phase this drives Ball
    Butler to ready and lets `_maybe_fire_reload_throw` fire the throw
    request itself, reaching the SAME post-condition
    (`bb/throw_at_target` asked, `_reload_ctx` in `await_announcement` on
    success) the old synchronous code produced. A test that is ABOUT the
    `await_bb_idle` wait itself (C1's own tests) calls `_start_pattern`
    directly instead, so it can inspect the phase before this helper would
    advance it."""
    node._params['plant_id'] = 'test_reload_' + str(id(node))
    # `_run_one_ball` wires `observer=`/`observations=` on the
    # executor it builds (self-toss AND reload alike) -- unlike columns,
    # which doesn't -- so a tick must see `in_trajectory_mode=True` or
    # `SkillExecutor.tick`'s very first check ends the attempt
    # `ABORTED_MODE_CHANGED` before the bridge REST ever dispatches.
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _prelevel_ready(node)
    _reload_ready(node, **reload_kw)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal(reload=True))
    if (resp.success and node._reload_ctx is not None
            and node._reload_ctx.phase == 'await_bb_idle'):
        _bb_ready(node)
        node._maybe_fire_reload_throw()
    return resp


def _finish_bridge(node):
    """Dispatch the reload bridge's one REST at its own `dispatch_s()` (the
    mock clock always reads 0.0 -- ticking at the skill's own instant is
    this file's established way around that, see `test_the_tick_dispatches_
    the_first_throw_through_the_mocked_client`) and let `_on_tick` retire
    the finished executor -- a REST releases nothing, so `done` is true the
    instant it dispatches."""
    rest = node._executor.schedule.skills[0]
    node._executor.tick(rest.dispatch_s())
    node._on_tick()


def test_reload_installs_the_bridge_rest_then_calls_bb_reload_and_throw(tmp_path):
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_reload_bridge'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    prelevel_client = _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal(reload=True))
    assert resp.success is True, resp.message
    # C1 (2026-09-28): bb/reload is asked BEFORE bb/throw_at_target, but the
    # throw is NOT asked inline any more -- `_start_pattern` only arms
    # `await_bb_idle` (the 2026-09-27 sitting measured BB's own
    # CHECKING_BALL cycle racing an inline throw request on every attempt).
    assert len(reload_client.calls) == 1
    assert throw_client.calls == []
    assert node._reload_ctx is not None
    assert node._reload_ctx.phase == 'await_bb_idle'
    # The bridge REST is a fresh, single-skill schedule -- installed through
    # the SAME `trajectory/install_segment` client every other schedule uses,
    # once ticked at its own dispatch instant (the mock clock always reads
    # 0.0, exactly like `test_the_tick_dispatches_the_first_throw_through_
    # the_mocked_client` -- ticking at the skill's own `dispatch_s()`
    # directly is this file's established way around that).
    bridge_rest = node._executor.schedule.skills[0]
    node._executor.tick(bridge_rest.dispatch_s())
    assert len(_client.calls) == 1
    req = _client.calls[0]
    assert req.kind == InstallSegment.Request.KIND_REST
    # Ball Butler reports ready (IDLE, ball in hand, connected) -- the tick
    # fires the throw request.
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    assert len(throw_client.calls) == 1
    assert node._reload_ctx.phase == 'await_announcement'
    throw_req = throw_client.calls[0]
    assert throw_req.use_target_point is True
    assert throw_req.target_name == node._robot_name
    # The aim point is P1's CATCH point (-50, 0) mm at CATCH_CUP_Z_MM, in the
    # MOCAP frame -- with no frame check ever run here `_mocap_to_schedule_mm`
    # is still `None`, so the mocap-frame point equals the schedule-frame one.
    assert throw_req.target_point_global_mm.x == pytest.approx(-50.0)
    assert throw_req.target_point_global_mm.y == pytest.approx(0.0)
    assert throw_req.target_point_global_mm.z == pytest.approx(sn.CATCH_CUP_Z_MM)
    assert throw_req.throw_delay_s >= sn._BB_THROW_DELAY_FLOOR_S
    assert node._reload_ctx is not None
    assert prelevel_client.calls == [] or len(prelevel_client.calls) == 1


def test_reload_with_a_ball_already_in_hand_skips_bb_reload(tmp_path):
    """Owner, 2026-09-28 21:41 sitting: Ball Butler's RELOAD_CMD runs its ~1 s
    ball check, which a ball already in hand does not need. A FRESH heartbeat
    reporting IDLE + ball in hand + connected at goal time means `bb/reload`
    is never sent and the throw fires on the next tick (no wait for a check
    cycle that will not happen). Fail-before (2026-09-28, committed
    skill_node): `len(reload_client.calls) == 1`."""
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_reload_ball_in_hand'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    _bb_ready(node, after_check=False)          # BB idle, ball in hand, fresh
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal(reload=True))
    assert resp.success is True, resp.message
    assert reload_client.calls == []
    assert 'bb/reload skipped' in resp.message
    assert node._reload_ctx.reload_sent is False
    node._maybe_fire_reload_throw()
    assert len(throw_client.calls) == 1
    assert node._reload_ctx.phase == 'await_announcement'


def test_reload_does_not_fire_on_the_idle_from_before_the_ball_check(tmp_path):
    """The 2026-09-28 21:41 race (bag 2026-09-28_21-41-10): after `bb/reload`
    answered, the next heartbeat still reported the IDLE from BEFORE the check
    began (CHECKING_BALL arrived 73-121 ms later), and the tick fired the throw
    ~20 ms after the reload. After a `bb/reload`, only an IDLE that follows a
    non-IDLE state counts. Fail-before (2026-09-28, committed skill_node):
    the fetch-deadline assertion (3.0 s, no empty-hand wait existed); the
    committed ready gate had no left-IDLE term, so it fired on the first IDLE."""
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_reload_race'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    _bb_ready(node, ball_in_hand=False, after_check=False)   # empty hand
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal(reload=True))
    assert resp.success is True, resp.message
    assert len(reload_client.calls) == 1
    assert node._reload_ctx.deadline_mono - time.perf_counter() > 5.0
    _bb_ready(node, after_check=False)           # the stale pre-check IDLE
    node._maybe_fire_reload_throw()
    assert throw_client.calls == []
    _bb_ready(node, state=sn.proto.BallButlerStates.CHECKING_BALL)
    node._maybe_fire_reload_throw()
    assert throw_client.calls == []
    _bb_ready(node, after_check=False)           # IDLE after the check
    node._maybe_fire_reload_throw()
    assert len(throw_client.calls) == 1


def test_a_stale_bb_heartbeat_is_not_evidence_of_a_ball(tmp_path):
    """A heartbeat older than `_BB_HEARTBEAT_FRESH_S` neither skips `bb/reload`
    nor fires the throw: the bridge publishes it at 10 Hz, so silence past
    0.5 s means the state is unknown. Fail-before (2026-09-28, committed
    skill_node): `AttributeError` -- no heartbeat age was kept at all."""
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_reload_stale_hb'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    _bb_ready(node, after_check=False)
    node._bb_heartbeat_mono -= sn._BB_HEARTBEAT_FRESH_S + 0.1
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal(reload=True))
    assert resp.success is True, resp.message
    assert len(reload_client.calls) == 1
    node._reload_ctx.bb_left_idle = True
    node._maybe_fire_reload_throw()
    assert throw_client.calls == []


def test_reload_aim_point_carries_the_mocap_frame_offset(tmp_path):
    """`_mocap_aim_point_mm` is the INVERSE of `_on_balls`'s subtraction: an
    adopted offset must be ADDED to the schedule-frame site to reach the
    mocap frame BB's own aim resolves in."""
    node, _client = _node_with_client(frame=False)
    _frame_ready(node, plat_x=5.0, plat_y=-3.0, cmd_x=0.0, cmd_y=0.0)
    node._frame_check_error()      # adopts (dx, dy) = (5.0, -3.0)
    assert node._mocap_to_schedule_mm == pytest.approx((5.0, -3.0))
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    throw_req = node._bb_throw_cli.calls[0]
    assert throw_req.target_point_global_mm.x == pytest.approx(-50.0 + 5.0)
    assert throw_req.target_point_global_mm.y == pytest.approx(0.0 - 3.0)


def test_reload_while_running_is_refused_even_after_the_bridge_finishes(tmp_path):
    """The bridge REST's own `SkillExecutor` is `done` (releases nothing)
    within a couple of ticks -- `_reload_ctx`, not `_executor`, is what must
    keep a second start refused while BB's announcement is still awaited."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)
    assert node._executor is None            # the bridge finished
    assert node._reload_ctx is not None       # still awaiting the announcement
    resp2 = node._start_pattern(_self_toss_goal())
    assert resp2.success is False
    assert 'already running' in resp2.message


def test_a_bb_reload_refusal_surfaces_verbatim(tmp_path):
    """INVARIANTS.md `REJECTED_BB(<message>)`: an external thrower's OWN
    refusal surfaces verbatim, not laundered into a generic abort.

    C4 (2026-09-28, the 2026-09-27 field bug): by the time `bb/reload`
    answers, the bridge REST `SkillExecutor` already exists and is already
    ticking toward `trajectory/install_segment` on the 40 Hz timer -- a
    REJECT here would leave that live and untracked (the orchestrator's
    "an attempt is already in progress?" while the REST streamed on). The
    goal is ACCEPTED instead and the attempt ends through `_goal_end_code`,
    the same path every other in-flight reload refusal takes."""
    node, _client = _node_with_client()
    resp = _start_reload(node, tmp_path, reload_ok=False,
                         reload_message='Reload failed: ERR_BUS_DOWN')
    assert resp.success is True, resp.message
    assert 'REJECTED_BB(Reload failed: ERR_BUS_DOWN)' in resp.message
    assert node._bb_throw_cli.calls == []     # never asked to throw
    assert node._reload_ctx is None
    # `_end_attempt` marks the (still undispatched) bridge executor rather
    # than setting `_goal_end_code` directly -- `_on_tick`'s own capture
    # line carries it through, exactly like an operator STOP does.
    assert node._executor is not None
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'REJECTED_BB(Reload failed: ERR_BUS_DOWN)'
    node._on_tick()
    assert node._executor is None
    assert node._goal_end_code == 'REJECTED_BB(Reload failed: ERR_BUS_DOWN)'


def test_a_bb_throw_at_target_refusal_surfaces_verbatim(tmp_path):
    """C1/C4: the throw request now fires from the tick once BB reports
    ready, well after the goal is accepted (`_start_reload` returns success
    once `bb/reload` is OK) -- its refusal ends the attempt through
    `_goal_end_code`, not the goal-accept response."""
    node, _client = _node_with_client()
    resp = _start_reload(node, tmp_path, throw_ok=False,
                         throw_message='no solution within BB limits')
    assert resp.success is True, resp.message
    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.end_code == 'REJECTED_BB(no solution within BB limits)'
    node._on_tick()
    assert node._executor is None
    assert node._goal_end_code == 'REJECTED_BB(no solution within BB limits)'


def test_reload_carries_hold_tilt_on_the_catch_after_the_announcement(tmp_path):
    """`_on_announcement` compiles the real `compile_reload` schedule and
    swaps the executor; the CATCH it carries is aimed by the announced
    landing (`landing_prior`) and holds the receive tilt the arrival
    derives."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)
    assert node._executor is None
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0),
        throw_time_s=2.0, landing_time_s=2.7)
    node._on_announcement(ann)
    assert node._reload_ctx is None
    assert node._executor is not None
    schedule = node._executor.schedule
    kinds = [s.kind for s in schedule.skills]
    assert kinds[:3] == ['REST', 'CATCH', 'REST']
    catch = schedule.skills[1]
    assert catch.hold_tilt is not None
    assert catch.landing_prior is not None
    np.testing.assert_allclose(catch.landing_prior.pos_mm,
                               [-50.0, 0.0, sn.CATCH_CUP_Z_MM])
    assert any(s.kind == 'THROW' for s in schedule.skills)
    assert node._executor.learner is not None
    assert node._executor.boxes is not None
    # The FlightLatch for the reload ball (schedule id 0) is queued from the
    # announcement's OWN throw_time, exactly as `_maybe_announce` does for
    # our own releases.
    assert 0 in node._correlation
    assert len(node._correlation[0]) == 1
    assert node._correlation[0][0].t_release_s == pytest.approx(2.0)


def test_an_announcement_not_from_ball_butler_is_ignored(tmp_path):
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)
    ctx_before = node._reload_ctx
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0), throw_time_s=2.0,
        landing_time_s=2.7, thrower_name=node._robot_name)   # OUR OWN throw
    node._on_announcement(ann)
    assert node._reload_ctx is ctx_before    # untouched
    assert node._executor is None


def _announce_with_a_concurrent_step(node, step):
    """Deliver a Ball Butler announcement to `node` and run ``step(node)``
    INSIDE `_on_announcement`'s compile, the window between the handler
    claiming the reload and installing the compiled schedule. On the robot
    that window is a few ms of `compile_reload` on the subscription thread,
    during which the 40 Hz tick (and the Juggle action's thread) keep
    running on other threads. Returns what ``step`` returned."""
    real_compile = sn.compile_reload
    seen = {}

    def compile_with_a_concurrent_step(*args, **kwargs):
        seen['step'] = step(node)
        return real_compile(*args, **kwargs)

    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0),
        throw_time_s=2.0, landing_time_s=2.7)
    with patch.object(sn, 'compile_reload', compile_with_a_concurrent_step):
        node._on_announcement(ann)
    return seen['step']


def test_the_juggle_goal_is_not_done_while_the_announced_reload_compiles(
        tmp_path):
    """R4 sitting 4 (2026-09-29 19:11): all three reload goals reported
    "COMPLETED (0/0 caught)" to the GUI about 20 ms after the button, while
    the reload went on to catch and throw four times each. `_on_announcement`
    cleared `_reload_ctx` BEFORE compiling, so a tick in the compile window
    saw no executor and no reload context and fired `_goal_done_event`."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)
    assert node._executor is None
    node._goal_done_event.clear()

    def tick_mid_compile(n):
        n._maybe_signal_goal_done()
        return n._goal_done_event.is_set()

    done_mid_compile = _announce_with_a_concurrent_step(node, tick_mid_compile)
    assert done_mid_compile is False, (
        'the goal was reported done while the reload schedule was compiling')
    assert node._executor is not None
    assert node._reload_ctx is None
    node._maybe_signal_goal_done()
    assert not node._goal_done_event.is_set()


def test_a_stop_during_the_reload_compile_is_not_overwritten_by_the_swap(
        tmp_path):
    """The same window, the operator's side: a Stop that lands while the
    announced schedule compiles must win. Before the fix the stop saw no
    executor and no reload context ("no attempt was running"), and the swap
    then installed the reload schedule anyway -- the robot went on to catch
    and throw after the operator had pressed Stop."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)

    stop = _announce_with_a_concurrent_step(node, lambda n: n._stop_attempt())
    assert 'reload wait cancelled' in stop.message
    assert node._executor is None, (
        'the reload schedule was installed after the operator stopped')
    assert node._reload_ctx is None


def test_a_second_announcement_during_the_compile_is_ignored(tmp_path):
    """Regression guard for the claim that replaced the pop-first shape: an
    announcement delivered while the first one is still compiling must not
    compile or install a second schedule."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)
    calls = []
    real_compile = sn.compile_reload

    def counting_compile(*args, **kwargs):
        calls.append(1)
        return real_compile(*args, **kwargs)

    def second_announcement(n):
        with patch.object(sn, 'compile_reload', counting_compile):
            n._on_announcement(_bb_announcement(
                target_id=n._robot_name,
                landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
                landing_vel_mm_s=(1200.0, 0.0, -2500.0),
                throw_time_s=2.0, landing_time_s=2.7))
        return n._executor

    executor_mid_compile = _announce_with_a_concurrent_step(
        node, second_announcement)
    assert calls == []
    assert executor_mid_compile is None
    assert node._executor is not None
    assert node._reload_ctx is None


def test_an_unschedulable_announcement_ends_the_attempt_with_its_own_code(
        tmp_path):
    """The claim holds `_reload_ctx` through the compile, so a landing
    `compile_reload` refuses must end the attempt itself -- not sit claimed
    until the wait times out as ABORTED_NO_ANNOUNCEMENT (the announcement
    did arrive), and not leave the Juggle goal to report COMPLETED."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)

    def refuse(*_args, **_kwargs):
        raise ValueError('the reload CATCH window is too short')

    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0),
        throw_time_s=2.0, landing_time_s=2.7)
    with patch.object(sn, 'compile_reload', refuse):
        node._on_announcement(ann)
    assert node._executor is None
    assert node._reload_ctx is None
    assert node._goal_end_code == 'REJECTED_RELOAD_UNSCHEDULABLE'


def test_the_reload_timeout_leaves_a_claimed_announcement_to_its_compile(
        tmp_path):
    """Phase-end audit, 2026-09-29: the claim keeps `_reload_ctx` set through
    the compile, so `_check_reload_timeout` must not time it out there -- the
    announcement DID arrive (ABORTED_NO_ANNOUNCEMENT would name the wrong
    fact) and the compiled schedule would be dropped. Mirrors the `'firing'`
    exclusion: `_on_announcement` alone resolves a claimed context."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)

    def deadline_passes_mid_compile(n):
        n._reload_ctx.deadline_mono = time.perf_counter() - 1.0
        n._check_reload_timeout()
        return n._reload_ctx

    ctx_after_timeout_check = _announce_with_a_concurrent_step(
        node, deadline_passes_mid_compile)
    assert ctx_after_timeout_check is not None
    assert node._executor is not None, (
        'the timeout dropped a schedule compiled for an announced throw')
    assert node._goal_end_code == ''


def test_an_unexpected_error_in_the_compile_ends_the_attempt_by_name(tmp_path):
    """The claim's backstop: with the timeout no longer able to clear a
    claimed context, ANY error after the claim must end the attempt itself,
    naming the error, or the attempt would sit claimed until a Stop."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)

    def broken(*_args, **_kwargs):
        raise RuntimeError('boom')

    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0),
        throw_time_s=2.0, landing_time_s=2.7)
    with patch.object(sn, 'compile_reload', broken):
        node._on_announcement(ann)
    assert node._executor is None
    assert node._reload_ctx is None
    assert node._goal_end_code == 'ABORTED_RELOAD_ERROR(RuntimeError)'


def test_the_reload_wait_times_out_aborted_no_announcement(tmp_path):
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    node._reload_ctx.deadline_mono = time.perf_counter() - 1.0
    node._on_tick()
    assert node._reload_ctx is None
    # A late announcement after the timeout is ignored, not acted on --
    # the executor (whatever `_on_tick` left it as) is UNCHANGED, never
    # swapped for a reload schedule this announcement did not earn.
    executor_before = node._executor
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0), throw_time_s=2.0,
        landing_time_s=2.7)
    node._on_announcement(ann)
    assert node._executor is executor_before


def test_stop_during_a_reload_wait_clears_the_context():
    node, _client = _node_with_client()
    node._reload_ctx = SimpleNamespace(
        pattern=None, boxes=None, memory=None,
        deadline_mono=time.perf_counter() + 10.0)
    resp = node._stop_attempt()
    assert resp.success is True
    assert node._reload_ctx is None
    assert 'cannot be aborted' in resp.message


# ═════════════════════════════════════════════════════════════════════════════
# R4 reload C1 (2026-09-28): sequencing on Ball Butler's OWN heartbeat state
# ═════════════════════════════════════════════════════════════════════════════

def _reload_armed_await_bb_idle(node, tmp_path):
    """Drive `node` to the `await_bb_idle` phase WITHOUT advancing past it
    (unlike the `_start_reload` helper, which auto-drives to
    `await_announcement` for every OTHER test's sake) -- for the C1 tests
    that are about this phase itself."""
    node._params['plant_id'] = 'test_c1_' + str(id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal(reload=True))
    assert resp.success is True, resp.message
    assert node._reload_ctx is not None
    assert node._reload_ctx.phase == 'await_bb_idle'
    return throw_client


def test_c1_bb_stuck_in_checking_ball_never_fires_the_throw(tmp_path):
    """The 2026-09-27 22:37 sitting's own failure mode, held open: BB's
    `requestCheckBall()` cycle (its OWN side effect of `bb/reload`) has BB
    in CHECKING_BALL, not IDLE -- no throw call, `_reload_ctx` stays
    `await_bb_idle`."""
    node, _client = _node_with_client(response=_response())
    throw_client = _reload_armed_await_bb_idle(node, tmp_path)
    _bb_ready(node, state=sn.proto.BallButlerStates.CHECKING_BALL)
    node._on_tick()
    assert throw_client.calls == []
    assert node._reload_ctx is not None
    assert node._reload_ctx.phase == 'await_bb_idle'


def test_c1_bb_ready_fires_exactly_one_throw_with_the_derived_delay(tmp_path):
    """IDLE + ball in hand + connected -> exactly one `bb/throw_at_target`
    call, `delay_s` derived from the bridge's own settle instant (not
    `pattern.floor_lift_s`, the pre-C1 proxy)."""
    node, _client = _node_with_client(response=_response())
    throw_client = _reload_armed_await_bb_idle(node, tmp_path)
    bridge_end_abs = node._reload_ctx.bridge.skills[-1].t_abs_s
    _bb_ready(node)
    node._on_tick()
    assert len(throw_client.calls) == 1
    assert node._reload_ctx.phase == 'await_announcement'
    req = throw_client.calls[0]
    now = node.get_clock().now().nanoseconds / 1e9
    expected = max(bridge_end_abs - now + sn.PRETILT_S + sn.LEAD_S,
                   sn._BB_THROW_DELAY_FLOOR_S)
    assert req.throw_delay_s == pytest.approx(expected)
    # A second tick must not fire a second throw.
    node._on_tick()
    assert len(throw_client.calls) == 1


def test_c1_deadline_ends_aborted_bb_not_ready_naming_the_state(tmp_path):
    """A deadline with no ready heartbeat ends the attempt
    `ABORTED_BB_NOT_READY(<state>, ball_in_hand=<bool>)` -- one physical
    fact, not a generic abort."""
    node, _client = _node_with_client(response=_response())
    _reload_armed_await_bb_idle(node, tmp_path)
    _bb_ready(node, state=sn.proto.BallButlerStates.CHECKING_BALL,
             ball_in_hand=False)
    node._reload_ctx.deadline_mono = time.perf_counter() - 1.0
    # `_check_reload_timeout` (called first, inside THIS `_on_tick`) marks
    # the still-live bridge executor ended, and the SAME call's own tick_lock
    # block retires it (unlike ending an attempt from `_start_reload`,
    # which runs OUTSIDE any `_on_tick` and needs a SEPARATE later tick to
    # propagate) -- one call does both here.
    node._on_tick()
    assert node._reload_ctx is None
    assert node._executor is None
    assert node._goal_end_code == (
        'ABORTED_BB_NOT_READY(CHECKING_BALL, ball_in_hand=False)')


# ═════════════════════════════════════════════════════════════════════════════
# R4 reload C2 (2026-09-28): armed BEFORE the bb/throw_at_target call
# ═════════════════════════════════════════════════════════════════════════════

class _RacingThrowClient(_RecordingClient):
    """A `bb/throw_at_target` client whose call is overtaken: while the
    (blocking) call is in flight, the firmware's `sched_refused` moves and the
    concurrent tick's `_check_reload_lane_refused` ends the attempt -- the
    field's own ordering (2026-09-27 22:46: the REST refused on the wire while
    skill_node was still inside a BB service call)."""

    def __init__(self, node, **kw):
        super().__init__(**kw)
        self._node = node

    def call_async(self, req):
        fut = super().call_async(req)
        self._node._on_link_status(_link_status(9))
        self._node._check_reload_lane_refused()
        return fut


def test_c1_a_throw_call_overtaken_by_a_lane_refusal_does_not_overwrite_the_end_code(tmp_path):
    """`_fire_reload_throw` blocks in `wait_for_service`/`_wait_future` without
    `_reload_lock`; `_check_reload_lane_refused` runs on every tick and can end
    the attempt `HAND_LANE_REFUSED` (force_hold) in that window. When the stale
    call then returns a failure it must NOT call `_end_attempt` again: with the
    bridge executor already retired, that second call overwrites
    `_goal_end_code` and the Juggle result loses the E-stop-preventing reason
    (2026-09-28 audit finding). The hold stays armed either way; this pins
    the diagnosability."""
    node, _client = _node_with_client(response=_response())
    node._on_link_status(_link_status(4))
    _reload_armed_await_bb_idle(node, tmp_path)
    # The field precondition (C5b): the one-skill bridge executor dispatches
    # and RETIRES before BB is ready -- so nothing is left to re-capture
    # `_goal_end_code` from an executor afterwards.
    _finish_bridge(node)
    assert node._executor is None
    assert node._reload_ctx is not None and node._reload_ctx.phase == 'await_bb_idle'
    node._bb_throw_cli = _RacingThrowClient(
        node, response=_bb_throw_response(success=False, message='BAD_STATE',
                                          predicted_tof_s=0.7, throw_delay_s=3.0),
        ready=True)
    hold = _hold_client()
    node._hold_cli = hold
    _bb_ready(node)
    node._on_tick()
    assert node._reload_ctx is None
    assert node._goal_end_code == ex.HAND_LANE_REFUSED, node._goal_end_code
    assert len(hold.calls) == 1


def test_c2_announcement_delivered_while_the_throw_call_is_in_flight(tmp_path):
    """An announcement delivered from INSIDE the mocked service's own
    `call_async` (simulating the pre-C2b field bug, where ball_butler_node
    published synchronously inside its OWN service handler, before its
    response to US even returned) is compiled, not dropped as "no reload
    awaiting one"."""
    node, _client = _node_with_client(response=_response())
    throw_client = _reload_armed_await_bb_idle(node, tmp_path)
    ann_holder = {}
    orig_call_async = throw_client.call_async

    def _call_async_with_announcement(request):
        node._on_announcement(ann_holder['ann'])
        return orig_call_async(request)
    throw_client.call_async = _call_async_with_announcement
    ann_holder['ann'] = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0), throw_time_s=2.0,
        landing_time_s=2.7)

    _bb_ready(node)
    node._maybe_fire_reload_throw()

    assert node._reload_ctx is None
    assert node._executor is not None
    kinds = [s.kind for s in node._executor.schedule.skills]
    assert kinds[:2] == ['REST', 'CATCH']


# ═════════════════════════════════════════════════════════════════════════════
# R4 reload C3 (2026-09-28): bb/throw_outcome ends a rejected throw same-tick
# ═════════════════════════════════════════════════════════════════════════════

def test_c3_throw_rejected_bad_state_ends_the_attempt_same_tick(tmp_path):
    """A `THROW_REJECTED_BAD_STATE` outcome arriving during `await_announcement`
    ends the attempt immediately with `REJECTED_BB(THROW_REJECTED_BAD_STATE)`
    -- no waiting out `RELOAD_ANNOUNCE_TIMEOUT_S` for an announcement a
    rejected throw will never produce (the 2026-09-27 sitting waited the
    full 4.6 s for exactly this)."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    assert node._reload_ctx is not None
    assert node._reload_ctx.phase == 'await_announcement'

    node._on_bb_throw_outcome(
        String(data='THROW_REJECTED_BAD_STATE (axis=n/a, detail1=0)'))

    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.end_code == 'REJECTED_BB(THROW_REJECTED_BAD_STATE)'
    node._on_tick()
    assert node._goal_end_code == 'REJECTED_BB(THROW_REJECTED_BAD_STATE)'


def test_c3_an_ok_outcome_is_ignored():
    node, _client = _node_with_client()
    node._reload_ctx = SimpleNamespace(
        phase='await_announcement', pattern=None, boxes=None, memory=None,
        deadline_mono=time.perf_counter() + 10.0)
    node._on_bb_throw_outcome(String(data='OK (axis=n/a, detail1=0)'))
    assert node._reload_ctx is not None


def test_c3_an_outcome_outside_a_reload_wait_is_ignored():
    """`bb/throw_outcome` is a node-wide topic (calibration throws share
    it) -- a rejection with no reload `await_announcement` in progress must
    not touch anything."""
    node, _client = _node_with_client()
    node._on_bb_throw_outcome(
        String(data='THROW_REJECTED_BAD_STATE (axis=n/a, detail1=0)'))
    assert node._reload_ctx is None
    assert node._goal_end_code == ''

def test_c3_a_rejection_after_the_announcement_compiled_the_schedule_ends_it(tmp_path):
    """Ball Butler announces at DISPATCH (its Teensy's single CMD_RESULT is
    terminal -- `OK` only once the ball has left, so an announcement gated on
    it would come after the flight began), so a reload schedule can already be
    compiled and waiting at the receive attitude when the firmware refuses or
    aborts the throw (2026-09-27: every rejected throw was announced first and
    `THROW_REJECTED_BAD_STATE` 20 ms later). The outcome relay must end THAT
    attempt too, not only an `await_announcement` wait -- through the rest
    tail, since the ball is not coming."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0),
        throw_time_s=2.0, landing_time_s=2.7)
    node._on_announcement(ann)
    assert node._reload_ctx is None
    assert node._executor is not None and not node._executor.attempt_ended
    assert node._reload_throw_inflight is True

    node._on_bb_throw_outcome(String(data='OK (axis=n/a, detail1=0)'))
    assert not node._executor.attempt_ended

    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_NOT_SETTLED (axis=1, detail1=120)'))
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'REJECTED_BB(THROW_ABORTED_NOT_SETTLED)'
    assert node._reload_throw_inflight is False
    # And nothing lingers for the NEXT attempt: a later outcome is ignored.
    node._on_bb_throw_outcome(
        String(data='THROW_REJECTED_BAD_STATE (axis=n/a, detail1=0)'))
    assert node._executor.end_code == 'REJECTED_BB(THROW_ABORTED_NOT_SETTLED)'



# ═════════════════════════════════════════════════════════════════════════════
# R4 reload C4 (2026-09-28): a refusal after the bridge executor exists ends
# the attempt via _goal_end_code, and never blocks the NEXT goal
# ═════════════════════════════════════════════════════════════════════════════

def test_c4_after_a_bb_refusal_ends_the_next_goal_is_not_refused(tmp_path):
    """C4's whole point: the 2026-09-27 field bug left the orchestrator
    believing "an attempt is already in progress?" while the REST streamed
    on with nothing tracking it. Once `_on_tick` retires the ended
    executor, a fresh goal must start cleanly."""
    node, _client = _node_with_client()
    resp = _start_reload(node, tmp_path, reload_ok=False,
                         reload_message='BB calibration not yet received.')
    assert resp.success is True, resp.message
    node._on_tick()          # retires the ended bridge executor
    assert node._executor is None
    assert node._reload_ctx is None
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp2 = node._start_pattern(_self_toss_goal())
    assert resp2.success is True, resp2.message
    assert 'already running' not in resp2.message


# ═════════════════════════════════════════════════════════════════════════════
# R4 reload C5b (2026-09-28): a refused hand lane during a reload wait, after
# the one-skill bridge executor has already retired itself
# ═════════════════════════════════════════════════════════════════════════════

def test_c5b_lane_refused_during_a_reload_wait_ends_and_holds_even_after_the_bridge_retires(tmp_path):
    """Root cause of the 2026-09-27 22:37 sitting's guard latch: the bridge
    (`compile_reload_wait`) is a ONE-skill schedule, so `SkillExecutor.done`
    is true (and `_on_tick` nulls `self._executor`) the SAME tick it
    dispatches -- long before the REST has physically settled (measured:
    6.42 s of hand-homing in the field). With no executor left,
    `hand_lane_refused` was never read again for the rest of that move, and
    the field guard latched with NO `HAND_LANE_REFUSED` END ever logged.
    `_check_reload_lane_refused` is the fix: it watches `_reload_ctx`
    directly, independent of `self._executor`."""
    node, _client = _node_with_client(response=_response())
    node._on_link_status(_link_status(0))
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    assert node._reload_ctx is not None
    # Dispatch the bridge REST and let `_on_tick` retire it -- the SAME
    # gap the field hit (`_finish_bridge`'s own established idiom).
    _finish_bridge(node)
    assert node._executor is None
    assert node._reload_ctx is not None      # still awaiting the announcement
    hold = _hold_client()
    node._hold_cli = hold

    node._on_link_status(_link_status(2))
    node._on_tick()

    assert node._reload_ctx is None
    assert len(hold.calls) == 1
    assert node._executor is None
    assert node._goal_end_code == ex.HAND_LANE_REFUSED


def test_c5b_lane_refused_before_the_bridge_ever_dispatches_holds_only_once(tmp_path):
    """A refusal caught while the bridge executor is STILL alive (never
    ticked yet) exercises a SECOND path to the same hold:
    `_check_reload_lane_refused` (via `_end_attempt`, `force_hold=True`)
    marks the executor `attempt_ended` and fires the hold, then `_on_tick`'s
    OWN inline `HAND_LANE_REFUSED` branch (later in the SAME tick, once it
    finds `self._executor.attempt_ended` already true) must not fire a
    SECOND one -- `_end_attempt` sets `_hold_forced_code` for exactly this
    reason."""
    node, _client = _node_with_client(response=_response())
    node._on_link_status(_link_status(0))
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    assert node._executor is not None        # never ticked -- still alive
    hold = _hold_client()
    node._hold_cli = hold

    node._on_link_status(_link_status(2))
    node._on_tick()

    assert len(hold.calls) == 1
    assert node._executor is None
    assert node._goal_end_code == ex.HAND_LANE_REFUSED


# ═════════════════════════════════════════════════════════════════════════════
# the Juggle action (R4 U5): goal accept/reject, cancel, execute/feedback


class TestJuggleAction:
    """`jugglebot/juggle` replaces the deleted columns-start / self-toss-start /
    stop Trigger services (owner decision D3): goal accept
    runs `_start_pattern` (`_juggle_goal`), cancel calls `_stop_attempt`
    (`_juggle_cancel` + `_juggle_execute`'s wait loop), execute waits on
    `_goal_done_event` and returns the `Juggle.Result`."""

    def test_goal_callback_accepts_a_valid_pattern(self, tmp_path):
        node, _client = _node_with_client()
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                          _good_box_path(tmp_path, columns=True)), \
             patch.object(sn, '_REPO_ROOT', str(tmp_path)):
            assert (node._juggle_goal(_columns_goal(separation_mm=100.0))
                   == sn.GoalResponse.ACCEPT)
        assert node._executor is not None

    def test_goal_callback_rejects_an_unknown_pattern(self):
        node, _client = _node_with_client()
        assert node._juggle_goal(_goal('not_a_pattern')) == sn.GoalResponse.REJECT
        assert node._executor is None

    def test_goal_callback_rejects_a_second_goal_while_the_first_runs(self, tmp_path):
        node, _client = _node_with_client()
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                          _good_box_path(tmp_path, columns=True)), \
             patch.object(sn, '_REPO_ROOT', str(tmp_path)):
            assert (node._juggle_goal(_columns_goal(separation_mm=100.0))
                   == sn.GoalResponse.ACCEPT)
            assert (node._juggle_goal(_columns_goal(separation_mm=100.0))
                   == sn.GoalResponse.REJECT)

    def test_columns_with_reload_is_lifted_at_r5(self, tmp_path):
        """R5 (owner decision D3, 2026-09-30): `reload=True` on a columns
        goal is no longer refused -- it ACCEPTS and arms the BB
        choreography (`await_bb_idle`), same as a reload='self_toss'
        goal."""
        node, _client = _node_with_client()
        node._params['plant_id'] = 'test_columns_reload_lift'
        _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
        _reload_ready(node)
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                          _good_box_path(tmp_path, columns=True)), \
             patch.object(sn, '_REPO_ROOT', str(tmp_path)):
            assert (node._juggle_goal(_columns_goal(reload=True, separation_mm=100.0))
                   == sn.GoalResponse.ACCEPT)
        assert node._executor is not None
        assert node._reload_ctx is not None
        assert node._reload_ctx.kind == 'columns'
        assert node._reload_ctx.phase == 'await_bb_idle'
        assert node._reload_ctx.aim_site.name == 'P2'
        assert node._reload_ctx.site.name == 'P1'

    def test_cancel_callback_always_accepts(self):
        node, _client = _node_with_client()
        assert node._juggle_cancel(MagicMock()) == sn.CancelResponse.ACCEPT

    def test_execute_returns_the_end_code_once_on_tick_retires_the_executor(self, tmp_path):
        """`_goal_done_event` fires from `_on_tick` (`_maybe_signal_goal_done`),
        never merely because `_executor` went `None` mid-reload-wait (see that
        method's docstring) -- here a plain `_stop_attempt` + one `_on_tick`
        is the shortest path to that state, mirroring `test_stop_ends_the_
        attempt`.

        Uses self_toss, not columns (R5): `_maybe_signal_goal_done` requires
        BOTH `_executor is None` AND `_reload_ctx is None`, but a
        `reload=False` columns start leaves `_reload_ctx` armed for the FEED
        WAIT, and `_stop_attempt` only clears `_reload_ctx` on its
        "no executor yet" branch -- the bridge executor is still live here,
        so it never does. `_goal_done_event` then never fires and
        `_juggle_execute` (called synchronously below, no thread) blocks
        forever. Confirmed empirically (2026-09-30) -- not this unit's fix
        (no production-code edits), flagged in the handoff. self_toss with
        `reload=False` (the default) never touches `_reload_ctx`."""
        node, _client = _node_with_client()
        _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
        _prelevel_ready(node)
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)):
            assert (node._juggle_goal(_self_toss_goal())
                   == sn.GoalResponse.ACCEPT)
        node._stop_attempt()
        node._on_tick()
        assert node._executor is None
        gh = MagicMock(is_cancel_requested=False)
        result = node._juggle_execute(gh)
        assert result.outcome == 'STOPPED'
        assert result.success is False
        assert result.throws == 0
        assert result.caught == 0
        assert result.per_throw == []
        gh.canceled.assert_not_called()
        gh.abort.assert_called_once()

    def test_execute_publishes_feedback_and_honours_a_live_cancel(self, tmp_path):
        """Uses self_toss, not columns (R5): a `reload=False` columns start
        leaves `_reload_ctx` armed for the FEED WAIT even once the opening
        bridge's own executor ends/retires (`_stop_attempt` only clears
        `_reload_ctx` on the "no executor yet" branch — the bridge executor
        is still live here), so `_maybe_signal_goal_done` (which requires
        BOTH `_executor is None` AND `_reload_ctx is None`) never fires and
        this test's cancel-and-wait hangs forever. Confirmed empirically
        (2026-09-30): with `_columns_goal()` the background thread never
        returns even after `attempt_ended` goes True and a `_on_tick()`
        retires the executor. Not this unit's fix (no production-code
        edits) — flagged in the handoff for the main session. self_toss
        with `reload=False` (the default) never touches `_reload_ctx` at
        all, so it exercises the exact same `_juggle_execute` cancel path
        this test is actually about."""
        node, _client = _node_with_client()
        _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
        _prelevel_ready(node)
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)):
            assert (node._juggle_goal(_self_toss_goal())
                   == sn.GoalResponse.ACCEPT)
        gh = MagicMock(is_cancel_requested=False)
        holder = {}

        def run():
            holder['result'] = node._juggle_execute(gh)

        th = threading.Thread(target=run)
        th.start()
        try:
            deadline = time.time() + 2.0
            while not gh.publish_feedback.called and time.time() < deadline:
                time.sleep(0.01)
            assert gh.publish_feedback.called, 'no feedback published'
            fb = gh.publish_feedback.call_args[0][0]
            assert fb.phase == 'RUNNING'
            gh.is_cancel_requested = True
            deadline = time.time() + 2.0
            while (node._executor is not None
                   and not node._executor.attempt_ended
                   and time.time() < deadline):
                time.sleep(0.01)
            assert node._executor is not None and node._executor.attempt_ended, (
                '_juggle_execute never observed the cancel and called '
                '_stop_attempt')
            node._on_tick()
        finally:
            th.join(timeout=5.0)
        assert not th.is_alive(), '_juggle_execute never returned after the hold'
        result = holder['result']
        assert result.outcome == 'STOPPED'
        gh.canceled.assert_called_once()

    def test_hop_pattern_compiles_a_two_site_schedule(self, tmp_path):
        node, _client = _node_with_client()
        node._on_traj_status(_status())
        _prelevel_ready(node)
        node._params['plant_id'] = 'test_hop_' + str(id(node))
        box_path = tmp_path / 'admissible_box.yaml'
        boxes = [
            adm.AdmissibleBox(
                site_pair=pair, apex_band_m=(0.85, 0.95),
                landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), apex_m=(0.6, 1.2),
                pattern='hop', release_site_xy_mm=rel, target_site_xy_mm=tgt,
                limits={'leg_vel_mmps': 300.0, 'leg_acc_mmps2': 5000.0,
                       'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': 3500.0},
                gate_hash=adm.gate_hash(), swept_at='2026-09-13',
                dwell_s=sn._DEFAULT_DWELL_S)
            for pair, rel, tgt in [(('P1', 'P2'), (-50.0, 0.0), (50.0, 0.0)),
                                   (('P2', 'P1'), (50.0, 0.0), (-50.0, 0.0))]]
        adm.dump(str(box_path), boxes)
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(box_path)), \
             patch.object(sn, '_REPO_ROOT', str(tmp_path)):
            resp = node._start_pattern(_hop_goal(separation_mm=100.0))
        assert resp.success is True, resp.message
        assert node._executor.schedule.pattern == 'hop'
        sites = {sk.site.name for sk in node._executor.schedule.skills}
        assert sites == {'P1', 'P2'}

    def test_hop_refusal_names_the_separation_mismatch_not_just_the_apex(
            self, tmp_path):
        """R4 sitting 3 (owner-reported): a 0.95 m hop refused with "no
        admissible box covers site pair(s) ... at apex 0.950 m" even though a
        swept box DID cover that apex -- it was swept at the node's default
        separation_mm=250 (release/target sites at +-125 mm) while the goal
        asked for separation_mm=100 (+-50 mm). A bands-only message hides that
        the apex band WAS satisfied and points the operator at the wrong
        knob; the refusal must name the site-geometry mismatch instead
        (`.scratch/r4-throw-precision/issues/04-plain-language-refusal-messages.md`)."""
        node, _client = _node_with_client()
        node._on_traj_status(_status())
        _prelevel_ready(node)
        node._params['plant_id'] = 'test_hop_sep_mismatch_' + str(id(node))
        box_path = _good_box_path(tmp_path, hop=True)
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH', box_path), \
             patch.object(sn, '_REPO_ROOT', str(tmp_path)):
            resp = node._start_pattern(
                _hop_goal(separation_mm=100.0, apex_m=0.95))
        assert resp.success is False
        assert 'no admissible box covers' in resp.message
        # the swept sites (+-125 mm) and the live sites (+-50 mm) must both
        # be named, along with the swept (250 mm) and requested (100 mm)
        # separations -- an operator changing separation_mm must be able to
        # see that THAT is the mismatch, not the apex.
        assert '125.0' in resp.message and '-125.0' in resp.message
        assert '50.0' in resp.message and '-50.0' in resp.message
        assert '250.0' in resp.message
        assert '100.0' in resp.message
        assert 'separation_mm' in resp.message
        assert node._executor is None



def test_the_separation_default_is_the_swept_hop_separation():
    """The GUI relay sends 0 for every numeric goal field ("the node's
    default"), so the node's `separation_mm` default IS the separation a
    GUI-started hop flies. It must be the one the hop boxes were swept at
    (250 mm, owner decision D2 2026-09-23) — at 100 mm there is no hop box
    and the goal is refused before anything moves (R4 runsheet finding,
    2026-09-24)."""
    node, _client = _node_with_client()
    assert float(node.get_parameter('separation_mm').value) == 250.0
    assert sn._DEFAULT_SEPARATION_MM == 250.0


def test_check_reports_the_hop_boxes_at_the_node_separation(tmp_path):
    """`skills/check` validates the HOP's boxes (both directions, at the
    node's separation) — not only the self-toss box — so a missing hop box
    surfaces at the dry-run check rather than as a goal rejection (R4
    runsheet finding, 2026-09-24)."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _freshen(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path, hop=True)):
        resp = node._svc_check(Trigger.Request(), Trigger.Response())
    hop_lines = [l for l in resp.message.split('; ') if 'hop box' in l]
    assert len(hop_lines) == 2, resp.message
    assert all('hop box OK' in l for l in hop_lines), resp.message
    assert 'P1->P2 at 250.0 mm' in resp.message and 'P2->P1 at 250.0 mm' in resp.message
    # ... and a box file WITHOUT hop boxes is a refusal at the check, not a
    # goal rejection later.
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)):
        resp2 = node._svc_check(Trigger.Request(), Trigger.Response())
    assert resp2.success is False
    assert 'hop box REFUSED' in resp2.message



# ── the operator's lines (console phase 2, `motion/skills/report.py`) ─────
#
# One start line, one line per throw, one end line; every detail line at
# DEBUG (recorded, not shown); the Juggle result counted from ThrowReports.

def _throw_report(n, caught):
    from jugglebot.motion.skills.report import ThrowReport
    return ThrowReport(throw_no=n, n_throws=2, ball_id=0, caught=caught,
                       row=True, no_row_reason='', apex_m=0.9,
                       landing_err_mm=(1.0, -2.0), release_err_s=0.01,
                       arrival_err_s=None, seat_s=0.1)


class TestOperatorLines:

    def test_the_node_records_debug(self):
        node, _client = _node_with_client()
        assert node.get_logger().level == sn.LoggingSeverity.DEBUG

    def test_an_accepted_goal_is_one_start_line(self, tmp_path):
        node, _client = _node_with_client()
        info = []
        node.get_logger().info = info.append
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                          _good_box_path(tmp_path, columns=True)), \
             patch.object(sn, '_REPO_ROOT', str(tmp_path)):
            assert (node._juggle_goal(_columns_goal(separation_mm=100.0))
                   == sn.GoalResponse.ACCEPT)
        assert len(info) == 1 and info[0].startswith('columns started: '), info

    def test_a_refused_goal_is_one_error_line(self):
        node, _client = _node_with_client(frame=False)
        node.set_parameters(
            [_MockParameter(40.0, name='learner_lateral_authority_mm')])
        errors = []
        node.get_logger().error = errors.append
        assert node._juggle_goal(_columns_goal()) == sn.GoalResponse.REJECT
        assert len(errors) == 1 and 'REJECTED' in errors[0], errors

    def test_each_throw_is_one_line_and_the_result_counts_reports(self, tmp_path):
        """Uses self_toss, not columns (R5): with `reload=False` (the
        default) columns' `_reload_ctx` stays armed for the FEED WAIT past
        `_stop_attempt` (the bridge executor is still live, so
        `_stop_attempt` takes the branch that never clears `_reload_ctx`),
        which makes `_log_attempt_end` return early (its own docstring:
        skip the end line "while the announcement is still awaited") --
        the 'columns stopped ... 1/2 caught' line this test is about would
        never be logged, and the later synchronous `_juggle_execute` call
        would block forever on `_goal_done_event`
        (`_maybe_signal_goal_done` needs `_reload_ctx is None` too).
        Confirmed empirically (2026-09-30) -- not this unit's fix (no
        production-code edits), flagged in the handoff. self_toss with
        `reload=False` never touches `_reload_ctx`."""
        from jugglebot.motion.skills import report as rp
        node, _client = _node_with_client()
        _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
        _prelevel_ready(node)
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)):
            assert (node._juggle_goal(_self_toss_goal())
                   == sn.GoalResponse.ACCEPT)
        info, warn = [], []
        node.get_logger().info = info.append
        node.get_logger().warning = warn.append
        caught, missed = _throw_report(1, True), _throw_report(2, False)
        node._executor.reports.extend([caught, missed])
        node._stop_attempt()
        node._on_tick()
        assert node._executor is None
        assert rp.throw_line(caught)[1] in info
        assert warn == [rp.throw_line(missed)[1]]
        ends = [ln for ln in info if ln.startswith('self_toss stopped by the '
                                                   'operator: 1/2 caught')]
        assert len(ends) == 1, info
        result = node._juggle_execute(MagicMock(is_cancel_requested=False))
        assert (result.throws, result.caught) == (2, 1)
        assert result.per_throw == [rp.throw_line(caught)[1],
                                    rp.throw_line(missed)[1]]

    def test_ending_a_live_attempt_leaves_its_one_line_to_retirement(self, tmp_path):
        node, _client = _node_with_client()
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                          _good_box_path(tmp_path, columns=True)), \
             patch.object(sn, '_REPO_ROOT', str(tmp_path)):
            assert (node._juggle_goal(_columns_goal(separation_mm=100.0))
                   == sn.GoalResponse.ACCEPT)
        errors = []
        node.get_logger().error = errors.append
        node._end_attempt('ABORTED_X', 'something broke')
        assert errors == []
        node._on_tick()
        assert errors == ['columns ENDED (ABORTED_X): something broke · '
                          '0/0 caught']

    def test_ending_with_no_executor_logs_the_end_line_itself(self):
        node, _client = _node_with_client()
        node._attempt_label = 'self_toss (reload)'
        errors = []
        node.get_logger().error = errors.append
        node._end_attempt('ABORTED_BB_NOT_READY',
                          'Ball Butler never became ready')
        assert errors == ['self_toss (reload) ENDED (ABORTED_BB_NOT_READY): '
                          'Ball Butler never became ready · 0/0 caught']

    def test_a_code_only_end_is_not_said_twice(self):
        node, _client = _node_with_client()
        node._attempt_label = 'hop (reload)'
        errors = []
        node.get_logger().error = errors.append
        code = 'REJECTED_BB(BB calibration not yet received)'
        node._end_attempt(code, code)
        assert errors == ['hop (reload) ENDED (%s) · 0/0 caught' % (code,)]

    def test_the_reload_bridge_rest_retiring_is_not_the_end(self):
        node, _client = _node_with_client()
        node._reload_ctx = SimpleNamespace(announced=False)
        lines = []
        node.get_logger().info = lines.append
        node.get_logger().error = lines.append
        node._log_attempt_end(SimpleNamespace(
            attempt_ended=False, end_code='', end_message='', end_kind='',
            schedule=SimpleNamespace(pattern='self_toss')))
        assert lines == []


def test_the_mock_logger_enforces_rclpy_one_severity_per_call_site():
    """The guard itself: one call site logging INFO then WARN raises, exactly
    as Foxy's RcutilsLogger does -- the ValueError that killed skill_node on
    2026-09-29."""
    from tests.ros.conftest import MockLogger
    logger = MockLogger()

    def one_site(method):
        method('x')

    one_site(logger.info)
    with pytest.raises(ValueError, match='severity cannot be changed'):
        one_site(logger.warning)
    logger.info('distinct call sites may differ')
    logger.warning('distinct call sites may differ')


def test_a_caught_then_a_missed_throw_then_a_failed_end_do_not_crash():
    """THE 2026-09-29 CRASH, through the ENFORCING mock logger (no captured
    lambdas): INFO, WARN, INFO throw lines, then INFO and ERROR end lines."""
    node, _client = _node_with_client()
    node._drain_reports(SimpleNamespace(reports=[
        _throw_report(1, True), _throw_report(2, False),
        _throw_report(3, True)]))
    node._log_at('INFO', 'self_toss done: 2/3 caught')
    node._log_at('ERROR', 'self_toss ENDED (X) · 2/3 caught')
    node._log_at('WARN', 'w')
    assert len(node._goal_reports) == 3


# ── phase 3: startup line, skills/check severity, quiet shutdown ────────────

def _rec_logger(node):
    from tests.ros.conftest import MockLogger
    rec = MockLogger()
    rec.lines = []
    for name, sev in (('info', 'INFO'), ('warning', 'WARN'),
                      ('error', 'ERROR'), ('debug', 'DEBUG')):
        def make(name=name, sev=sev):
            def f(msg, **kw):
                rec.lines.append((sev, msg))
            return f
        setattr(rec, name, make())
    node._logger = rec   # records only; site enforcement runs via the plain mock
    return rec


def test_skills_check_reports_ok_at_info_and_a_refusal_at_warning():
    node, _ = _node_with_client()
    node._svc_check(None, SimpleNamespace())     # through the enforcing mock
    rec = _rec_logger(node)
    resp = SimpleNamespace()
    node._svc_check(None, resp)
    (sev, line), = [(s, m) for s, m in rec.lines if m.startswith('skills/check:')]
    assert sev == ('INFO' if resp.success else 'WARN')


def test_startup_ready_line_carries_the_blas_count(monkeypatch):
    from jugglebot.motion import blas_threads as bt
    seen = []
    monkeypatch.setattr(bt, 'read_blas_threads', lambda: (1, 'test'))
    from tests.ros import conftest as cf
    orig = cf.MockLogger.info
    monkeypatch.setattr(cf.MockLogger, 'info',
                        lambda self, m, **kw: (seen.append(m),
                                               orig(self, m, **kw))[1])
    sn.SkillNode()
    assert 'skill_node ready · BLAS 1 thread' in seen
    assert not any(m.startswith('blas threads:') for m in seen)


# ═════════════════════════════════════════════════════════════════════════════
# R5 (owner decision D1, 2026-09-30): the feed-triggered columns start
# ═════════════════════════════════════════════════════════════════════════════

def _columns_bb_started(node, tmp_path):
    """Drive `node` to `_reload_ctx.phase == 'await_announcement'` for a
    columns goal (`reload=True`): bridge installed, BB ready, throw fired --
    mirrors `_start_reload`'s own helper shape for the one-ball reload."""
    node._params['plant_id'] = 'test_columns_feed_' + str(id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(reload=True, separation_mm=100.0))
    assert resp.success is True, resp.message
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    return resp


def test_columns_bb_announcement_installs_the_fed_schedule(tmp_path):
    """The announcement path (brief_S2.md item 2): BB's `ThrowAnnouncement`
    for us, naming a landing at P2, compiles `compile_columns(pattern,
    feed=...)` and swaps the executor -- the SAME claim-then-swap
    `_install_announced_reload` uses for the one-ball reload."""
    node, _client = _node_with_client()
    _columns_bb_started(node, tmp_path)
    assert node._reload_ctx.phase == 'await_announcement'
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(50.0, 0.0, 830.0),
        landing_vel_mm_s=(0.0, 0.0, -3000.0), throw_time_s=0.5,
        landing_time_s=5.0)
    node._on_announcement(ann)
    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.schedule.pattern == 'columns'
    catches = [s for s in node._executor.schedule.skills if s.kind == sn.CATCH]
    first = min(catches, key=lambda s: s.t_abs_s)
    assert first.landing_prior is not None
    assert first.landing_prior.t_land_abs_s == pytest.approx(5.0)
    assert first.ball_id == 1


def test_columns_tracker_feed_resolves_an_unannounced_landing_near_p2(tmp_path):
    """The tracker path (brief_S2.md item 2, ``reload=False`` -- no Ball
    Butler round trip at all): an un-announced, converged, descending
    landing within the feed bound of P2 resolves the FEED WAIT the same
    way, through `_on_balls`."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_tracker_feed'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    pattern = sn.Pattern(sites=sn.columns_sites(100.0), apex_m=0.9,
                         dwell_s=0.30, n_throws=3)
    tau = sn.transit_s(sn.flight_s(pattern.apex_m), pattern.dwell_s)
    lead_needed = tau + pattern.launch_s + sn.LEAD_S + (1.0 / sn._TICK_HZ)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(reload=False, separation_mm=100.0))
    assert resp.success is True, resp.message
    assert node._reload_ctx.kind == 'columns'
    assert node._reload_ctx.phase == 'await_feed'
    node._on_balls(BallStateArray(balls=[
        _ball(7, status=1, tracking=1, x=51.0, y=-2.0, z=800.0,
             vz=-3000.0, sec=0, nanosec=int((lead_needed + 1.0) * 1e9),
             landing_from_fit=True)]))
    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.schedule.pattern == 'columns'
    assert node._correlation.get(1) == (
        sn.FlightLatch(t_release_s=0.0, announced_id=7),)


def test_columns_tracker_feed_ignores_a_landing_outside_the_bound(tmp_path):
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_tracker_feed_bound'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(reload=False, separation_mm=100.0))
    assert resp.success is True, resp.message
    node._on_balls(BallStateArray(balls=[
        _ball(7, status=1, tracking=1, x=51.0 + sn.COLUMNS_FEED_BOUND_X_MM + 5.0,
             y=0.0, z=800.0, vz=-3000.0, sec=100, landing_from_fit=True)]))
    assert node._reload_ctx is not None
    assert node._reload_ctx.phase == 'await_feed'
    assert node._executor.schedule.pattern != 'columns'


def test_columns_tracker_feed_lead_deficit_is_logged_once(tmp_path):
    """A landing that fails the lead is logged ONCE with the deficit and the
    wait continues (brief_S2.md item 2)."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_lead_deficit'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(reload=False, separation_mm=100.0))
    assert resp.success is True, resp.message
    rec = _rec_logger(node)
    too_soon = _ball(7, status=1, tracking=1, x=50.0, y=0.0, z=800.0,
                     vz=-3000.0, sec=0, nanosec=1, landing_from_fit=True)
    node._on_balls(BallStateArray(balls=[too_soon]))
    node._on_balls(BallStateArray(balls=[too_soon]))
    deficit_lines = [m for s, m in rec.lines
                     if s == 'INFO' and 'short of' in m]
    assert len(deficit_lines) == 1
    assert node._reload_ctx is not None
    assert node._reload_ctx.phase == 'await_feed'


def test_columns_feed_wait_times_out_aborted_no_columns_feed(tmp_path):
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_feed_timeout'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(reload=False, separation_mm=100.0))
    assert resp.success is True, resp.message
    node._reload_ctx.deadline_mono = time.perf_counter() - 1.0
    node._on_tick()
    assert node._reload_ctx is None
    assert node._goal_end_code == 'ABORTED_NO_COLUMNS_FEED'


def test_a_stop_during_the_columns_bb_feed_compile_wins(tmp_path):
    """The 2026-09-29 race fix must survive for columns too: a Stop that
    lands while `_install_columns_schedule` is compiling must win -- the
    compiled schedule is dropped, not installed over the stop."""
    node, _client = _node_with_client()
    _columns_bb_started(node, tmp_path)
    _finish_bridge(node)  # the bridge REST retires -- `_executor` is None,
                          # the ordinary case `_stop_attempt`'s own docstring
                          # describes ("done by now in every ordinary case")
    ctx = node._reload_ctx
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(50.0, 0.0, 830.0),
        landing_vel_mm_s=(0.0, 0.0, -3000.0), throw_time_s=0.5,
        landing_time_s=5.0)
    # Claim the context exactly as `_on_announcement` would, then simulate a
    # Stop landing before the (still-claimed) install runs.
    with node._reload_lock:
        claimed = node._reload_ctx is ctx and not ctx.announced
        ctx.announced = claimed
    assert claimed
    node._stop_attempt()
    with node._reload_lock:
        assert node._reload_ctx is None
    bridge_executor = node._executor
    node._install_announced_reload(ctx, ann)
    assert node._executor is bridge_executor



def test_a_stop_during_the_columns_feed_wait_clears_the_reload_ctx_too(tmp_path):
    """Production gap flagged by unit T (handoff_T.md, R5, 2026-09-30): a
    `reload=False` columns start arms the opening REST BRIDGE (`_executor`,
    the ``phase='await_feed'`` bridge from `_run_columns`) AND `_reload_ctx`
    together. Before the fix, `_stop_attempt`'s executor-live branch only
    ended the bridge's attempt -- it never touched `_reload_ctx` -- so
    `_maybe_signal_goal_done` (`executor is None AND reload_ctx is None`)
    could never fire even once the bridge retired on its next tick, and
    `_juggle_execute`'s wait hung until the feed wait's own 30 s deadline
    (`COLUMNS_TRACKER_FEED_DEADLINE_S`) ended it with the wrong code
    (`ABORTED_NO_COLUMNS_FEED`) instead of `STOPPED`.

    Calls `_stop_attempt()` immediately after the start -- `node._executor`
    is still the live, undispatched bridge (mirrors `test_stop_ends_the_
    attempt`'s own "not retired by `_stop_attempt` itself" step, not
    `_columns_fed`, which ticks the feed to resolution first) -- then
    retires it with exactly one `_on_tick()` (the ordinary next-cycle path,
    same as `test_stop_ends_the_attempt`), which is what actually calls
    `_maybe_signal_goal_done` in production."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_stop_during_columns_feed_wait'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(reload=False,
                                                  separation_mm=100.0))
    assert resp.success is True, resp.message
    assert node._executor is not None
    assert node._reload_ctx is not None
    assert node._reload_ctx.phase == 'await_feed'

    stop_resp = node._stop_attempt()
    assert stop_resp.success is True
    assert node._reload_ctx is None
    assert 'the feed/reload wait is cancelled too' in stop_resp.message
    # Not discarded by `_stop_attempt` itself (its own docstring) -- still
    # the same bridge object, just ended.
    assert node._executor is not None
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'STOPPED'
    assert node._goal_done_event.is_set() is False

    node._on_tick()    # the ordinary next-cycle retirement + goal-done signal
    assert node._executor is None
    assert node._goal_done_event.is_set() is True
