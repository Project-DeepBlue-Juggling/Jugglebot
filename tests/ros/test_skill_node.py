"""skill_node — the mocked-ROS shell smoke test (R2, Unit D2).

The pure-Python schedule/segment/executor policy is pinned by
``tests/motion/test_skills_*.py``; ``trajectory/install_segment`` itself is
pinned by ``tests/ros/test_install_segment.py``. What this file exercises is
the SHELL: ``skills/start_columns`` compiles a schedule, the 40 Hz tick
dispatches the first due skill through the (mocked) install_segment client
with the right request fields, a refusal ends the attempt, and a ``/balls``
message feeds the executor's tracker callable.
"""

from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from tests.ros.conftest import MockTime, _MockParameter

import rclpy
from rclpy.callback_groups import (MutuallyExclusiveCallbackGroup,
                                   ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node as _MockNodeClass

from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import (BallStateArray, HandTelemetryMessage,
                                      RigidBodyPoses, TrajectoryStatus)
from jugglebot_interfaces.srv import InstallSegment

from jugglebot import ball_possession
from jugglebot import skill_node as sn
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills.executor import CAUGHT_WINDOW_S
from jugglebot.motion.skills.memory import Memory, memory_path

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
         sec=0, nanosec=0):
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
    return ball


def _status(**overrides):
    """A `trajectory/status` message at the admissible box's swept limits
    (`config/generated/admissible_box.yaml`: 300/5000/150000) — the LIVE
    limits `_svc_start_self_toss` reads."""
    st = TrajectoryStatus()
    st.leg_vel_limit_mmps = 300.0
    st.leg_acc_limit_mmps2 = 5000.0
    st.leg_jerk_limit_mmps3 = 150000.0
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


def _node_with_client(response=None, ready=True):
    node = sn.SkillNode()
    client = _RecordingClient(response=response, ready=ready)
    node._install_cli = client
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


# ═════════════════════════════════════════════════════════════════════════════
# start_columns / stop
# ═════════════════════════════════════════════════════════════════════════════

def test_node_starts_idle():
    node, _client = _node_with_client()
    assert node._executor is None
    assert node._possession_evidence == ball_possession.EVIDENCE_UNKNOWN


def test_start_columns_compiles_a_schedule_at_the_owner_operating_point():
    node, _client = _node_with_client()
    resp = node._svc_start_columns(Trigger.Request(), Trigger.Response())
    assert resp.success is True, resp.message
    assert node._executor is not None
    schedule = node._executor.schedule
    assert len(schedule.skills) > 0
    assert schedule.skills[0].kind == 'THROW'


def test_a_second_start_while_running_is_refused():
    node, _client = _node_with_client()
    assert node._svc_start_columns(Trigger.Request(),
                                   Trigger.Response()).success is True
    resp2 = node._svc_start_columns(Trigger.Request(), Trigger.Response())
    assert resp2.success is False
    assert 'already running' in resp2.message


def test_stop_ends_the_attempt():
    node, _client = _node_with_client()
    node._svc_start_columns(Trigger.Request(), Trigger.Response())
    assert node._executor is not None
    resp = node._svc_stop(Trigger.Request(), Trigger.Response())
    assert resp.success is True
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'STOPPED'
    # Not retired by `_svc_stop` itself — `.done` is a next-tick question
    # (main-session addition: a stopped attempt keeps ticking until any
    # pending outcome finalises, see the mid-flight test below). Here there
    # is nothing pending (`on_experience is None` for columns), so the very
    # next tick retires it.
    assert node._executor is not None
    node._on_tick()
    assert node._executor is None


def test_stop_keeps_ticking_until_a_pending_flight_finalises_then_appends_its_row(
        tmp_path):
    """`skills/stop` must not discard an observable flight already in
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

    with patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert node._svc_start_self_toss(Trigger.Request(),
                                         Trigger.Response()).success is True
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

        resp = node._svc_stop(Trigger.Request(), Trigger.Response())
        assert resp.success is True
        assert node._executor is not None          # still finalising
        assert node._executor.attempt_ended is True
        assert node._executor.end_code == 'STOPPED'

        resp2 = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
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
    resp = node._svc_stop(Trigger.Request(), Trigger.Response())
    assert resp.success is True
    assert 'no attempt' in resp.message


# ═════════════════════════════════════════════════════════════════════════════
# start_self_toss (R3 items 4/5) — the box/limits gate and the compiled schedule
# ═════════════════════════════════════════════════════════════════════════════

def test_start_self_toss_is_refused_when_the_box_file_is_missing(tmp_path):
    node, _client = _node_with_client()
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(tmp_path / 'nope.yaml')):
        resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'admissible box refused' in resp.message
    assert node._executor is None


def test_start_self_toss_is_refused_on_a_limits_mismatch(tmp_path):
    box_path = tmp_path / 'admissible_box.yaml'
    box = adm.AdmissibleBox(
        site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), flight_s=(0.7, 0.9),
        limits={'leg_vel_mmps': 1.0, 'leg_acc_mmps2': 1.0,
               'leg_jerk_mmps3': 1.0, 'hand_acc_rps2': 1.0},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13')
    adm.dump(str(box_path), [box])
    node, _client = _node_with_client()
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(box_path)):
        resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'admissible box refused' in resp.message
    assert 'leg_vel_mmps' in resp.message
    assert node._executor is None


def test_start_self_toss_is_refused_with_no_status_received_yet():
    """The live-limits fallback (`_live_limits`) reads the YAML module
    defaults when `trajectory/status` has never arrived (its own documented
    "0.0 = field absent" sentinel) — which do not match the real box's swept
    session limits, so the start refuses rather than silently gating against
    the wrong machine."""
    node, _client = _node_with_client()
    resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'admissible box refused' in resp.message


def test_start_self_toss_refuses_an_uncovered_apex_before_any_motion(tmp_path):
    """THE LATENT DEFECT this closes (found 2026-09-14): the only swept box
    covers apex 0.85-0.95 m; requesting 0.5 m (uncovered) must refuse BEFORE
    `_prelevel` ever calls `trajectory/go_to_pose` -- no platform motion for
    an apex nothing was swept for."""
    box_path = tmp_path / 'admissible_box.yaml'
    box = adm.AdmissibleBox(
        site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), flight_s=(0.7, 0.9),
        limits={'leg_vel_mmps': 300.0, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 150000.0, 'hand_acc_rps2': 3500.0},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13')
    adm.dump(str(box_path), [box])
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    prelevel_client = _prelevel_ready(node)
    node._params['apex_m'] = 0.5
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(box_path)):
        resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'no admissible box covers' in resp.message
    assert '0.500' in resp.message
    assert '0.850' in resp.message and '0.950' in resp.message
    assert node._executor is None
    assert prelevel_client.calls == []


def test_start_self_toss_compiles_a_schedule_at_the_owner_operating_point():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    prelevel_client = _prelevel_ready(node)
    node._params['plant_id'] = 'test_start_self_toss_compiles'
    resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
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


def test_start_self_toss_while_running_is_refused():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    node._params['plant_id'] = 'test_start_self_toss_while_running'
    assert node._svc_start_self_toss(Trigger.Request(),
                                     Trigger.Response()).success is True
    resp2 = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
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
    resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
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
        resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
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
        resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'memory refused' in resp.message
    assert pre.calls == []
    assert node._executor is None


# ═════════════════════════════════════════════════════════════════════════════
# start_self_toss (R3-e2 item 7) — the pre-level move
# ═════════════════════════════════════════════════════════════════════════════

def test_start_self_toss_is_refused_when_go_to_pose_is_unavailable():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    node._go_to_pose_cli = _RecordingClient(ready=False)
    resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'self-toss refused' in resp.message
    assert 'go_to_pose unavailable' in resp.message
    assert node._executor is None


def test_start_self_toss_is_refused_when_commanded_position_is_stale():
    """No `trajectory/commanded_position` has ever arrived — `_prelevel` has
    no xy/z to hold and must refuse rather than guess one."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    node._go_to_pose_cli = _RecordingClient(response=_go_to_pose_response())
    resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'commanded_position is stale' in resp.message
    assert node._executor is None


def test_start_self_toss_is_refused_when_the_prelevel_move_is_refused():
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node, accepted=False, code='WORKSPACE', message='nope')
    resp = node._svc_start_self_toss(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'pre-level move was refused' in resp.message
    assert 'WORKSPACE' in resp.message
    assert node._executor is None


# ═════════════════════════════════════════════════════════════════════════════
# the tick
# ═════════════════════════════════════════════════════════════════════════════

def test_the_tick_dispatches_the_first_throw_through_the_mocked_client():
    node, client = _node_with_client(response=_response())
    assert node._svc_start_columns(Trigger.Request(),
                                   Trigger.Response()).success is True
    first_throw = node._executor.schedule.skills[0]
    assert first_throw.kind == 'THROW'
    t_dispatch = first_throw.dispatch_s()

    node._executor.tick(t_dispatch)

    assert len(client.calls) == 1
    req = client.calls[0]
    assert req.kind == InstallSegment.Request.KIND_THROW
    assert req.ball_id == first_throw.ball_id
    assert req.t_event_s == pytest.approx(first_throw.t_abs_s)
    assert req.flight_s == pytest.approx(first_throw.y_d[1])


def test_a_refusal_ends_the_attempt():
    node, _client = _node_with_client(
        response=_response(accepted=False, code='WRONG_MODE',
                          message='not in TRAJECTORY mode'))
    node._svc_start_columns(Trigger.Request(), Trigger.Response())
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
    assert node._correlation[0]['announced_id'] is None
    assert node._correlation[0]['preexisting'] == set()


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
    assert node._correlation[0]['preexisting'] == {99}

    # The phantom is still there; the NEW ball (id 7) also appears.
    node._on_balls(BallStateArray(balls=[
        _ball(99, status=1, destination='jugglebot', tracking=1),
        _ball(7, status=1, destination='jugglebot', tracking=1,
             x=1.0, y=2.0, z=3.0),
    ]))
    assert node._correlation[0]['announced_id'] == 7
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
    assert node._correlation[0]['announced_id'] == 7
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
    assert node._correlation[0]['announced_id'] == 7

    # A re-send carrying the IDENTICAL release (same then_throw content).
    node._installer('CATCH', terminal, 99.5, ball_id=0)
    assert len(node._announce_pub.published) == 1          # not re-announced
    assert node._correlation[0]['announced_id'] == 7        # not reset


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
            hand_at_seed=True, seated=True):
    """Land every message `_observations` (item 6) reads, fresh, so a single
    test can flip exactly one axis stale/off-band and check only that field
    moved."""
    if mocap:
        node._on_mocap(RigidBodyPoses())
    node._on_traj_status(_status(
        gravity_correction_loaded=levelled,
        mode='TRAJECTORY' if in_traj else 'STANDBY'))
    if hand_fresh:
        node._on_hand_telemetry(HandTelemetryMessage(
            pos_meas=0.0, pos_cmd=(0.0 if hand_at_seed else 999.0),
            ball_held_raw=seated, ball_held_valid=True))


# ═════════════════════════════════════════════════════════════════════════════
# the R3 precondition ladder's observations (item 6)
# ═════════════════════════════════════════════════════════════════════════════

def test_observations_are_all_false_before_anything_has_arrived():
    node, _client = _node_with_client()
    obs = node._observations(0.0)
    assert obs.mocap_fresh is False
    assert obs.hand_fresh is False
    assert obs.hand_at_seed is False
    assert obs.levelled is False
    assert obs.in_trajectory_mode is False
    assert obs.ball_evidence == ball_possession.EVIDENCE_UNKNOWN


def test_observations_report_true_once_everything_is_fresh_and_levelled():
    node, _client = _node_with_client()
    _freshen(node)
    obs = node._observations(0.0)
    assert obs.mocap_fresh is True
    assert obs.hand_fresh is True
    assert obs.hand_at_seed is True
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
    assert obs.hand_at_seed is False    # gated on hand_fresh


def test_observations_hand_not_at_seed_when_measured_and_commanded_disagree():
    node, _client = _node_with_client()
    _freshen(node, hand_at_seed=False)
    assert node._observations(0.0).hand_at_seed is False


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
    node, _client = _node_with_client()
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'ladder REFUSED' in resp.message
    for code in ('REJECTED_MOCAP_STALE', 'REJECTED_NOT_LEVELLED',
                'REJECTED_HAND_STALE', 'REJECTED_HAND_NOT_PARKED',
                'REJECTED_BALL_UNKNOWN'):
        assert code in resp.message
    assert 'box' in resp.message


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
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), flight_s=(0.7, 0.9),
        limits={'leg_vel_mmps': 300.0, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 150000.0, 'hand_acc_rps2': 3500.0},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13')
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
# installer plumbing
# ═════════════════════════════════════════════════════════════════════════════

def test_installer_reports_service_unavailable():
    node, _client = _node_with_client(ready=False)
    node._svc_start_columns(Trigger.Request(), Trigger.Response())
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
    `Event.wait`, never a sleep) for the exact moment `_svc_stop`'s blocking
    acquire begins, instead of guessing at a delay."""

    def __init__(self, real_lock):
        self._lock = real_lock
        self.acquire_started = threading.Event()

    def acquire(self, blocking=True, timeout=-1):
        self.acquire_started.set()
        return self._lock.acquire(blocking, timeout)

    def release(self):
        self._lock.release()


def test_stop_waits_for_an_in_progress_tick():
    """Finding 12, R3 audit (2026-09-13): `_svc_stop` must take `_tick_lock`
    before writing `attempt_ended` / `end_code`, bounded by
    `_STOP_LOCK_WAIT_S` — otherwise it can race a tick mid-`_dispatch` on the
    timer thread."""
    node, _c = _node_with_client()
    node._svc_start_columns(Trigger.Request(), Trigger.Response())
    real_lock = node._tick_lock
    assert real_lock.acquire(blocking=False)   # simulate a tick mid-install
    wrapped = _SignallingLock(real_lock)
    node._tick_lock = wrapped

    result = {}

    def run():
        result['resp'] = node._svc_stop(Trigger.Request(), Trigger.Response())

    th = threading.Thread(target=run)
    th.start()
    try:
        assert wrapped.acquire_started.wait(timeout=5.0), (
            '_svc_stop never attempted to acquire _tick_lock')
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


def test_stop_with_a_pending_event_holds():
    node, _client = _node_with_client()
    hold_client = _hold_client()
    node._hold_cli = hold_client
    node._pending_event_mono = time.perf_counter() + 5.0
    node._executor = MagicMock()
    node._executor.attempt_ended = False
    node._executor.end_code = ''

    resp = node._svc_stop(Trigger.Request(), Trigger.Response())

    assert resp.success is True
    assert len(hold_client.calls) == 1
    assert node._pending_event_mono <= 0.0
    assert node._executor.end_code == 'STOPPED'


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
# catch_aim_source — the open-loop catch aim (owner decision 2026-09-15)
# ═════════════════════════════════════════════════════════════════════════════


def test_the_live_catch_aim_default_is_the_schedules_own_throw_state():
    """THE LIVE DEFAULT.  At the 2026-09-15 sitting all 13 self-tosses ended
    `NO_LANDING` — mocap never produced a marker for the flying ball — so the
    catch is aimed open loop from the throw state the schedule COMMANDED.
    The executor's own constructor default stays `tracker` (the sim gate's
    refine path), which is exactly why this node passes the value
    EXPLICITLY: this test is what pins the two apart."""
    node, _client = _node_with_client()
    assert node.get_parameter('catch_aim_source').value == 'schedule'
    node._svc_start_columns(Trigger.Request(), Trigger.Response())
    assert node._executor.catch_aim_source == 'schedule'
    assert node._executor.launch_ratio is not None


@pytest.mark.parametrize('value', ['tracker', 'schedule_hand'])
def test_the_aim_source_parameter_reaches_the_executor(value):
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter(value, name='catch_aim_source')])
    node._svc_start_columns(Trigger.Request(), Trigger.Response())
    assert node._executor.catch_aim_source == value


def test_an_unknown_aim_source_falls_back_to_the_open_loop_default():
    """A typo in a launch override must not leave the operator with no catch
    at all — and must not select the tracker, which is the mode that failed
    on 2026-09-15."""
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter('mocap', name='catch_aim_source')])
    node._svc_start_columns(Trigger.Request(), Trigger.Response())
    assert node._executor.catch_aim_source == 'schedule'


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
