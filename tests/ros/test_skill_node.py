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

import dataclasses
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
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from jugglebot_interfaces.msg import (BallButlerHeartbeat, BallStateArray,
                                      HandTelemetryMessage, RigidBodyPose,
                                      RigidBodyPoses, ThrowAnnouncement,
                                      TrajectoryStatus)
from jugglebot_interfaces.action import Juggle
from jugglebot_interfaces.srv import BallButlerThrow, InstallSegment

from jugglebot import ball_possession
from jugglebot import skill_node as sn
import jugglebot.hardware_config as hw

#: The session leg velocity every box fixture and the status fixture below
#: carry -- 350 since 2026-10-04 (R5 sitting 3: the roomier columns geometry
#: is swept and flown at 350 mm/s; `config/generated/admissible_box.yaml`
#: says so, and `check_limits` refuses a live limit the box was not swept
#: at, so a fixture pinning the old 300 fails every test that loads the
#: real box). One name, so the next raise is one edit.
_SESSION_LEG_VEL_MMPS = 350.0
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
         sec=0, nanosec=0, landing_from_fit=True, source='',
         throw_time_s=None):
    """A duck-typed ``/balls`` element — every field `_on_balls` /
    `latch_announced_ball` / `match_announced_track` reads (id/status/
    destination/tracking/source/throw_time/landing fields), never the real
    ``BallState`` message type (see the module docstring: `_on_balls` just
    reads attributes). ``throw_time_s=None`` is an un-announced track (zero
    stamp, no identity)."""
    ball = MagicMock()
    ball.id = id
    ball.status = status
    ball.destination = destination
    ball.tracking = tracking
    ball.source = source
    if throw_time_s is None:
        ball.throw_time = _Time(0, 0)
    else:
        sec_ = int(math.floor(float(throw_time_s)))
        ball.throw_time = _Time(sec_, int(round((float(throw_time_s) - sec_) * 1e9)))
    ball.landing_position = _Vec(x, y, z)
    ball.landing_velocity = _Vec(vx, vy, vz)
    ball.time_at_land = _Time(sec, nanosec)
    # Set explicitly, not left to the MagicMock's truthiness: since 2026-09-18
    # this flag decides whether a landing may become a learner row.
    ball.landing_from_fit = landing_from_fit
    return ball


def _identity(node, ball_id=0, k=-1):
    """The ``(source, throw_time)`` the tracker stamps on the track it mints
    for schedule ball ``ball_id``'s ``k``-th announced release — read off the
    node's own identity latch, exactly the key `match_announced_track`
    matches (`ball_possession`, the two-ball association contract)."""
    latch = node._correlation[ball_id][k]
    return {'source': latch.thrower, 'throw_time_s': latch.t_release_s}


def _pose_at(x, y):
    """Just enough of a `RigidBodyPose.pose` (a `PoseStamped`) for
    `_on_mocap`'s `body.pose.pose.position.x/y` access — duck-typed like
    `_ball()` above, not the real message type."""
    pose = MagicMock()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    return pose


def _status(**overrides):
    """A `trajectory/status` message at the admissible box's swept limits
    (`config/generated/admissible_box.yaml`: 350/5000/200000 since the sitting-3
    re-sweep, 2026-10-04; 300 from 2026-09-30 — the session leg jerk ramped from 150000 to the YAML
    ceiling; the launch default stays 150000 until the ramp is logged, which
    is why the runsheet sets the limits before `skills/check`) — the LIVE
    limits `_run_one_ball` reads."""
    st = TrajectoryStatus()
    st.leg_vel_limit_mmps = _SESSION_LEG_VEL_MMPS
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


_HAND_CLOCK_FROZEN_S = 1.0e6
#: `_node_with_client(frame=True)` freezes the node's frame clock here, the
#: frame-check twin of `_HAND_CLOCK_FROZEN_S` (JB-64, 2026-10-28).
_FRAME_CLOCK_FROZEN_S = 1.0e6


def _robot_frame_msg(t=(0.0, 0.0, 0.0), quat_xyzw=(0.0, 0.0, 0.0, 1.0)):
    """A `robot_frame` PoseStamped as mocap_node publishes it: position =
    robot origin in world mm, orientation = R robot -> world. The default is
    the EXACT identity (QTM aligned to the robot) -- every transform then
    returns its input bit for bit."""
    msg = PoseStamped()
    msg.pose.position = Point(x=float(t[0]), y=float(t[1]), z=float(t[2]))
    msg.pose.orientation = Quaternion(x=float(quat_xyzw[0]),
                                      y=float(quat_xyzw[1]),
                                      z=float(quat_xyzw[2]),
                                      w=float(quat_xyzw[3]))
    return msg


def _node_with_client(response=None, ready=True, hand=True, frame=True,
                      robot_frame=True):
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
    one that populates `_platform_mocap_xyz` / `_commanded_xy_hist` its own
    way (directly or via its own `_frame_ready(...)` call with a deliberate
    offset), asserts on the raw buffer contents, or wants the
    cannot-evaluate ("no Platform sample yet") branch. Injecting the ready
    default AND a test's own frame data would blend two sets of samples into
    one averaged offset — never what either party intended.

    ``robot_frame=True`` (default, 2026-10-19): the node has received an
    IDENTITY `robot_frame` (QTM aligned to the robot), so no start path is
    refused `REJECTED_ROBOT_FRAME_UNKNOWN` by a test not about the robot
    frame; ``False`` for one that is (`test_skill_node_robot_frame.py`)."""
    node = sn.SkillNode()
    # frozen hand clock: a loaded run cannot age the fixture's sample (2026-10-15)
    node._hand_clock = lambda: _HAND_CLOCK_FROZEN_S
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
        # frozen frame clock: a loaded run cannot age the fixture's frame
        # samples past `_FRAME_CHECK_WINDOW_S` (JB-64, 2026-10-28). A
        # ``frame=False`` test keeps the real clock and stamps its own.
        node._frame_clock = lambda: _FRAME_CLOCK_FROZEN_S
        _frame_ready(node)
    if robot_frame:
        node._on_robot_frame(_robot_frame_msg())
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
                 n_plat=None, n_cmd=5, plat_z_world=None):
    """Populate the session-start frame check's two buffers directly (plan
    `cup-contact-contract.md` § 1) — the Platform mocap body at
    ``(plat_x, plat_y)`` mm and the commanded position at ``(cmd_x, cmd_y)``
    mm, both fresh and, with repeated identical samples, at rest. Bypasses
    `_on_mocap` / `_on_commanded_position` (like `_prelevel_ready` bypasses
    `_on_commanded_position` for `_commanded_pos_mm`) — the node methods are
    exercised directly by `test_on_mocap_...` style tests instead.

    The Platform buffer holds WORLD points since 2026-10-19: ``plat_z_world``
    defaults to `hw.GEOM_INITIAL_HEIGHT_MM` (the body at the stow height,
    published z 0 plus `mocap_interface`'s shift added back)."""
    if n_plat is None:
        n_plat = sn._FRAME_CHECK_MIN_SAMPLES + 5
    if plat_z_world is None:
        plat_z_world = float(hw.GEOM_INITIAL_HEIGHT_MM)
    now = node._frame_clock()   # the check's own clock (frozen by the fixture)
    for _ in range(n_plat):
        node._platform_mocap_xyz.append((now, plat_x, plat_y, plat_z_world))
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


#: The columns point `config/generated/admissible_box.yaml` is swept at --
#: separation 125 / apex 0.95 since 2026-10-04 (R5 sitting 3). Tests that
#: start a columns goal against the REAL box (no `_good_box_path` fixture)
#: use this; the fixture-backed tests keep their own 100 mm / 0.9 m boxes.
_FLOWN_COLUMNS = dict(separation_mm=125.0, apex_m=0.95)


def _flown_columns_goal(**overrides):
    kw = dict(_FLOWN_COLUMNS); kw.update(overrides)
    return _goal('columns', **kw)


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
    """Start a columns attempt the way the live path does (cup SEATED -> ball
    B is fed by Ball Butler, no flag; the human-lob / tracker feed was
    retired 2026-10-17, P1 of `columns-empty-cup-two-feed.md`) and resolve
    the BB wait with a fake `ThrowAnnouncement` naming a landing at the aim
    point, so ``node._executor`` ends up holding the REAL ``compile_columns``
    schedule rather than the opening bridge -- the steady state a test that
    asserts on the compiled schedule's own shape needs. Built on
    `_columns_bb_started` + `_on_announcement`, exactly like
    `test_columns_bb_announcement_installs_the_fed_schedule`."""
    resp = _columns_bb_started(node, tmp_path, plant_id=plant_id)
    aim = node._reload_ctx.aim_site.cup_mm
    node._on_announcement(_bb_announcement(
        target_id=node._robot_name,
        landing_mm=tuple(float(v) for v in aim),
        landing_vel_mm_s=(0.0, 0.0, -3000.0), throw_time_s=0.5,
        landing_time_s=5.0))
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
    _reload_ready(node)
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
                 sec=int(t_land), nanosec=int((t_land % 1.0) * 1e9),
                 **_identity(node))]))

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


def test_the_committed_box_is_swept_at_the_node_default_dwell():
    """The dwell half of the launch/box contract (2026-10-05; the limits and
    gate half is tests/motion/test_skills_admissible.py): every committed box
    carries ``skill_node``'s default ``dwell_s``, so a launch with the default
    parameters is not refused ``LimitsMismatch: ... dwell_s ...`` at its first
    goal. Changing the default dwell means a re-sweep in the same commit."""
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                        'config', 'generated', 'admissible_box.yaml')
    boxes = adm.load(path)
    assert boxes
    # The oval cells are the one exception by design (two-ball-oval P1,
    # 2026-10-18): ``dwell_s`` joined the ``select()`` key so a cell swept at
    # the rung's dwell (0.65, decision 16 amended) coexists with the 0.27
    # cells and is REFUSED BY NAME at the default dwell -- the runsheet sets
    # ``dwell_s`` for an oval sitting (row 12a). Every other family still
    # carries the node default so the default launch is not refused.
    oval = [box for box in boxes if box.pattern in ('oval_1ball', 'oval')]
    assert oval, 'the committed box has no oval cell'
    # rung 2 (2026-10-23) swept the oval at 0.55 / 0.50 as well as 0.65;
    # P3 tail (2026-10-24) added the 0.45 cell under the pose planner.
    assert any(box.dwell_s == pytest.approx(0.65) for box in oval)
    for box in oval:
        assert any(box.dwell_s == pytest.approx(d) for d in (0.65, 0.55, 0.50, 0.45)), (
            box.site_pair, box.pattern, box.dwell_s)
    for box in boxes:
        if box.pattern in ('oval_1ball', 'oval'):
            continue
        assert box.dwell_s == pytest.approx(sn._DEFAULT_DWELL_S), (
            box.site_pair, box.pattern, box.dwell_s)


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
        limits={'leg_vel_mmps': _SESSION_LEG_VEL_MMPS, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 200000.0,
               # The node's live hand limit IS this constant (`_live_limits`;
               # set_limits has no hand field) — 3900 since 2026-10-02.
               'hand_acc_rps2': float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13',
        dwell_s=0.20, planner_mode='level')                            # != the node's default 0.30
    adm.dump(str(box_path), [box])
    node, _client = _node_with_client()
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(box_path)):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    # Since the dwell joined the select() key (plan two-ball-oval § 2.6) the
    # mismatch is named by the box-coverage refusal, not check_limits.
    assert 'no admissible box covers' in resp.message
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


def test_the_catch_outward_authority_parameter_defaults_to_ten_and_plumbs():
    """2026-10-12: outward tracker re-aims are capped at 10 mm (replay: passes
    with >= 2 % margin to 10 mm, refuses at the symmetric 20 mm); the helper
    returns metres and floors a negative value at 0."""
    node, _client = _node_with_client()
    assert node.get_parameter('catch_outward_authority_mm').value == 10.0
    assert node._catch_outward_authority_m() == pytest.approx(0.010)
    node._params['catch_outward_authority_mm'] = -5.0
    assert node._catch_outward_authority_m() == 0.0


def test_the_catch_inward_authority_parameter_defaults_to_35_and_plumbs():
    """2026-10-13 (owner decision 3): inward catch aims -- the learner-
    predicted prior and tracker re-aims -- may move 35 mm toward the other
    site (sitting 7's P1 bias was +27 mm); the learner's throw command keeps
    `learner_lateral_authority_mm`. Metres, floored at 0."""
    node, _client = _node_with_client()
    assert node.get_parameter('catch_inward_authority_mm').value == 35.0
    assert node._catch_inward_authority_m() == pytest.approx(0.035)
    node._params['catch_inward_authority_mm'] = -5.0
    assert node._catch_inward_authority_m() == 0.0


def test_the_capture_cap_is_the_widest_re_aim_the_clamp_can_emit():
    """The authority pin (I-CATCH-6, 2026-10-19): ``cup_cycle.
    CAPTURE_MAX_OFFSET_M`` -- the planner's backstop on the capture move's
    sideways slide -- is the 2-norm of the widest re-aim the executor's clamp
    can emit, i.e. the two-site inward authority along one axis with the
    learner's lateral authority across it: ``hypot(catch_inward_authority_mm,
    learner_lateral_authority_mm)``.  (Pre-norm design: 20 mm per axis, which
    the clamp's own 28.3 mm diagonal already exceeds.)  If either launch
    default moves without the cap, a re-aim the clamp is entitled to emit would
    be refused ``CATCH_AXIS`` at the planner (cap too small) or the backstop
    would stop being one (cap too large)."""
    import math
    from jugglebot.motion.trajectory import cup_cycle as cc
    node, _client = _node_with_client()
    inward = float(node.get_parameter('catch_inward_authority_mm').value)
    lateral = float(node.get_parameter('learner_lateral_authority_mm').value)
    assert cc.CAPTURE_MAX_OFFSET_M * 1000.0 == pytest.approx(
        math.hypot(inward, lateral))


def test_the_executor_gets_the_inward_authority_and_a_forward_model():
    """The started executor carries `catch_inward_authority_m` and a learner
    exposing `predict` (the catch prior's forward model, 2026-10-13)."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    node._params['plant_id'] = 'test_inward_authority_plumbs'
    resp = node._start_pattern(_self_toss_goal())
    assert resp.success is True, resp.message
    assert node._executor.catch_inward_authority_m == pytest.approx(0.035)
    assert node._executor.lateral_authority_m == pytest.approx(0.020)
    y_hat = node._executor.learner.predict(
        np.zeros(4), np.array([0.001, 0.0, 0.9]), np.array([0.0, 0.0, 0.9]))
    np.testing.assert_allclose(y_hat, [0.001, 0.0, 0.9])   # cold: identity


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


def test_columns_feed_site_defaults_to_p1():
    """R5 sitting 2 (owner decision, 2026-10-02): the NEW default feeds P1
    (A at P2) -- the site NEAREST Ball Butler along its bearing, so B's
    descent never crosses A's own column."""
    node, _client = _node_with_client()
    assert node.get_parameter('columns_feed_site').value == 'P1'
    assert node._columns_feed_site_name() == 'P1'


def test_columns_feed_site_p2_restores_the_flown_layout(tmp_path):
    """`columns_feed_site='P2'` restores the OLD (2026-09-30..10-02) layout
    -- A at P1, feed at P2 -- for an A/B comparison. Hop entry (the only
    entry): the bridge parks at the FEED site P2 and the aim is the
    un-walked feed site (+50, 0)."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_feed_site_p2'
    node.set_parameters([_MockParameter('P2', name='columns_feed_site')])
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert (node._juggle_goal(_columns_goal(separation_mm=100.0))
               == sn.GoalResponse.ACCEPT)
    assert node._reload_ctx.site.name == 'P2'
    assert node._reload_ctx.aim_site.name == 'P2'
    assert node._reload_ctx.aim_site.cup_mm[:2] == pytest.approx([50.0, 0.0])


def test_columns_feed_site_rejects_an_unknown_value():
    """Same shape as `_catch_aim_source`: a typo in a launch override must
    not leave the operator with no columns start at all -- falls back to
    the LIVE default ('P1') with an error logged."""
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter('P3', name='columns_feed_site')])
    assert node._columns_feed_site_name() == 'P1'


def test_columns_feed_aim_toward_a_mm_defaults_to_20():
    """2026-10-02 sitting 2 margins: 20 mm toward A clears the swapped
    layout's undisplaced LIMIT_ACC refusal at 80/83/73 % vel/acc/jerk."""
    node, _client = _node_with_client()
    assert node.get_parameter('columns_feed_aim_toward_a_mm').value == 10.0
    assert node._columns_feed_aim_toward_a_mm() == pytest.approx(10.0)


def test_columns_feed_aim_toward_a_mm_clamps_outside_0_40():
    """A bound on a planner cost, not a policy (mirrors `_catch_resend_
    max`/`_hold_tilt_max_deg`): out-of-range is clamped into ``[0, 40]``,
    never refused, with a WARN logged."""
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter(100.0, name='columns_feed_aim_toward_a_mm')])
    assert node._columns_feed_aim_toward_a_mm() == pytest.approx(40.0)
    node.set_parameters([_MockParameter(-5.0, name='columns_feed_aim_toward_a_mm')])
    assert node._columns_feed_aim_toward_a_mm() == pytest.approx(0.0)


def test_columns_feed_bb_bias_mm_defaults_to_0_0():
    """B4 (R5 sitting 4): ``[0.0, 0.0]`` -- unset, a FED catch plans and
    requests exactly as before this parameter existed."""
    node, _client = _node_with_client()
    assert list(node.get_parameter('columns_feed_bb_bias_mm').value) == [0.0, 0.0]
    assert node._columns_feed_bb_bias_mm() == (0.0, 0.0)


def test_columns_feed_bb_bias_mm_clamps_outside_60():
    """Same WARN-and-clamp shape as `_columns_feed_aim_toward_a_mm`: a
    bound on a planner input, not a policy -- out-of-range per axis is
    clamped into ``[-60, 60]``, never refused."""
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter([100.0, -90.0], name='columns_feed_bb_bias_mm')])
    assert node._columns_feed_bb_bias_mm() == pytest.approx((60.0, -60.0))
    node.set_parameters([_MockParameter([29.0, 25.0], name='columns_feed_bb_bias_mm')])
    assert node._columns_feed_bb_bias_mm() == pytest.approx((29.0, 25.0))


def test_columns_feed_bb_bias_mm_rejects_a_malformed_value():
    """A malformed value (wrong length, or non-finite) is REFUSED (ERROR)
    and ``(0.0, 0.0)`` is used -- never a silently-carried-forward prior
    value, same rule `trajectory_node._apply_level_trim_param` applies to
    its own ``[x, y]`` parameter."""
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter([5.0], name='columns_feed_bb_bias_mm')])
    assert node._columns_feed_bb_bias_mm() == (0.0, 0.0)
    node.set_parameters([_MockParameter([float('nan'), 5.0],
                                        name='columns_feed_bb_bias_mm')])
    assert node._columns_feed_bb_bias_mm() == (0.0, 0.0)
    node.set_parameters([_MockParameter([float('inf'), 5.0],
                                        name='columns_feed_bb_bias_mm')])
    assert node._columns_feed_bb_bias_mm() == (0.0, 0.0)


# ---------------------------------------------------------------------------
# R5 sitting 6 prep (owner decision, 2026-10-05): the HOP ENTRY -- the only
# fed-columns entry. The pre-R5 'transit' entry and its `columns_entry`
# parameter were retired 2026-10-14.

def test_columns_entry_hop_bridge_rest_holds_at_the_feed_site(tmp_path):
    """Hop entry, the SEATED-cup (Ball Butler feeds B) branch, default layout
    (`columns_feed_site='P1'`: feed P1, A P2): the bridge REST parks at the
    FEED site, not ball A's own site, and Ball Butler's aim is the
    UN-walked feed site (the 10 mm walk toward A exists only to counter the
    TRANSIT entry's catch reversal, which the hop entry never makes)."""
    node, _client = _node_with_client()
    _reload_ready(node)
    node._params['plant_id'] = 'test_columns_entry_hop_bridge'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    assert node._reload_ctx.site.name == 'P1'
    assert node._reload_ctx.aim_site.name == 'P1'
    assert node._reload_ctx.aim_site.cup_mm[:2] == pytest.approx([-50.0, 0.0])


def test_columns_entry_hop_bb_request_aims_at_the_unwalked_feed_site_plus_bias(
        tmp_path):
    """The cup-EMPTY Ball Butler path: under the hop entry the request
    point is the UN-walked feed site (not walked toward A), with
    `columns_feed_bb_bias_mm` still applied on top exactly as under
    transit (`_fire_reload_throw` reads `ctx.aim_site` generically, with no
    `entry` branch of its own)."""
    node, _client = _node_with_client()
    node.set_parameters([
        _MockParameter([29.0, 25.0], name='columns_feed_bb_bias_mm')])
    _columns_bb_started(node, tmp_path)
    assert node._reload_ctx.site.name == 'P1'
    aim = node._reload_ctx.aim_site.cup_mm
    assert aim[:2] == pytest.approx([-50.0, 0.0])
    throw_req = node._bb_throw_cli.calls[0]
    assert throw_req.target_point_global_mm.x == pytest.approx(float(aim[0]) - 29.0)
    assert throw_req.target_point_global_mm.y == pytest.approx(float(aim[1]) - 25.0)


def test_the_installed_fed_columns_schedule_marks_throw_0_as_the_hop_entry(
        tmp_path):
    """`_install_columns_schedule` must pass ``hop_entry=True`` to
    `compile_columns` (its committed default is the parked, non-hop entry),
    or THROW 0 of the live fed schedule silently loses ``entry_hop``.
    2026-10-15: the transit retirement dropped the argument and the committed
    default is the parked entry; this pins the live hop entry at the node,
    where the schedule-level tests cannot see it."""
    node, _client = _node_with_client()
    _columns_fed(node, tmp_path, plant_id='test_columns_installed_hop_flag')
    skills = node._executor.schedule.skills
    throws = [s for s in skills if s.kind == sc.THROW]
    assert throws, 'the installed schedule has no THROW at all'
    assert skills[0].kind == sc.THROW and skills[0].entry_hop is True
    assert [i for i, s in enumerate(skills) if s.entry_hop] == [0]


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
        limits={'leg_vel_mmps': _SESSION_LEG_VEL_MMPS, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13',
        dwell_s=sn._DEFAULT_DWELL_S, planner_mode='level')
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
    _reload_ready(node)
    _freshen(node)
    assert node._start_pattern(_flown_columns_goal()).success is True
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
    _reload_ready(node)
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
             x=1.0, y=2.0, z=3.0, **_identity(node)),
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
        balls=[_ball(7, status=1, destination='jugglebot', tracking=0,
                     **_identity(node))]))
    assert node._correlation[0][0].announced_id == 7
    assert node._tracker(0) is None    # IN_FLIGHT alone proves nothing
    node._on_balls(BallStateArray(
        balls=[_ball(7, status=1, destination='jugglebot', tracking=1,
                     **_identity(node))]))
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
        balls=[_ball(7, status=1, destination='jugglebot', tracking=1,
                     **_identity(node))]))
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
             sec=100, nanosec=600_000_000, **_identity(node))]))
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


def test_columns_releases_each_correlate_to_their_own_track():
    """The two-ball association contract through the node's own announce and
    ``/balls`` paths (sitting 3, 2026-10-02). Ball 0 (A) is released, then
    ball 1 (B) one beat later. 50 ms before B's release A's track is
    IN_FLIGHT+CONFIRMED and B's own is still TO_BE_THROWN -- the instant
    HEAD's exclusion rule handed B's latch A's id, and skill 3 then aimed B's
    catch at A's landing (WINDOW_TOO_SHORT). The identity latch takes B's
    own track, and the tracker serves B's landing for ball 1."""
    from jugglebot.motion.skills.segments import (CatchTerminal, ThrowAfterCatch,
                                                  ThrowTerminal)

    node, _client = _node_with_client(
        response=_response(t_event_mono=50.0, t_release_mono=0.0))
    a = ThrowTerminal(site_mm=[50.0, 0.0, 860.0], target_mm=[50.0, 0.0, 830.0],
                      flight_s=0.857, t_release_s=100.0)
    node._installer('THROW', a, 100.0, ball_id=0)
    node._install_cli._response = _response(t_release_mono=50.579)
    b = CatchTerminal(
        landing_mm=[-50.0, 0.0, 830.0], landing_vel_mm_s=[0.0, 0.0, -4200.0],
        t_land_s=100.5, rest_site_mm=[-50.0, 0.0, 750.0],
        then_throw=ThrowAfterCatch(t_release_s=100.579,
                                   site_mm=[-50.0, 0.0, 860.0],
                                   target_mm=[-50.0, 0.0, 830.0],
                                   flight_s=0.857))
    node._installer('CATCH', b, 100.0, ball_id=1)
    id_a, id_b = _identity(node, 0), _identity(node, 1)
    assert id_a['source'] == id_b['source'] == node._robot_name
    # One beat apart (less the wall time this test spent between the two
    # installs: each release is `t_release_mono - perf_counter()` from now).
    assert id_b['throw_time_s'] - id_a['throw_time_s'] == pytest.approx(
        0.579, abs=0.05)

    def snapshot(b_status, b_tracking):
        return BallStateArray(balls=[
            _ball(47, status=1, destination='jugglebot', tracking=1,
                  x=50.0, z=830.0, vz=-4200.0, sec=10, **id_a),
            _ball(48, status=b_status, destination='jugglebot',
                  tracking=b_tracking, x=-50.0, z=830.0, vz=-4200.0, sec=11,
                  **id_b)])

    t_b = id_b['throw_time_s']
    node._now_s = lambda: t_b - 0.05
    node._on_balls(snapshot(0, 0))
    assert [l.announced_id for l in node._correlation[0]] == [47]
    assert [l.announced_id for l in node._correlation[1]] == [48]
    node._now_s = lambda: t_b + 0.1
    node._on_balls(snapshot(1, 1))
    assert node._tracker(1).pos_mm[0] == pytest.approx(-50.0)   # B's, not A's
    assert node._tracker(0).pos_mm[0] == pytest.approx(50.0)
    # The node refuses to start against a stale BallState (grep-proof that
    # it calls the shared check rather than a lookalike).
    assert sn.require_identity_fields is ball_possession.require_identity_fields


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
        limits={'leg_vel_mmps': _SESSION_LEG_VEL_MMPS, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13',
        dwell_s=sn._DEFAULT_DWELL_S, planner_mode='level')
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
    _reload_ready(node)
    _freshen(node)
    node._start_pattern(_flown_columns_goal())
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

def test_tick_timer_is_reentrant_and_clients_are_off_the_pool_node():
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

    # 2026-10-11: the clients moved to the dedicated helper node (own executor
    # thread, default group); the tick timer stays reentrant.
    assert captured.get('client_cbgroup') is None
    assert isinstance(captured.get('timer_cbgroup'), ReentrantCallbackGroup)


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
    _reload_ready(node)
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
    """The forced hold (LANE_REFUSED) stays the HAND-LESS legacy hold:
    the firmware is holding a refused hand lane, and a REST carries a hand
    track that would keep walking the deviation the guard measures."""
    from jugglebot.motion.skills.segments import RestTerminal

    node, client = _node_with_client()
    hold_client = _hold_client()
    node._hold_cli = hold_client
    node._force_hold = True
    _ending_executor(node, [('before', RestTerminal(
        rest_site_mm=[0.0, 0.0, 750.0], t_rest_s=10.0))],
        end_code='LANE_REFUSED')
    node._hold_forced_code = 'LANE_REFUSED'

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
    node._start_pattern(_flown_columns_goal())
    assert node._executor.catch_aim_source == 'tracker'
    assert node._executor.launch_ratio is not None


@pytest.mark.parametrize('value', ['schedule', 'schedule_hand'])
def test_the_aim_source_parameter_reaches_the_executor(value):
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter(value, name='catch_aim_source')])
    node._start_pattern(_flown_columns_goal())
    assert node._executor.catch_aim_source == value


def test_an_unknown_aim_source_falls_back_to_the_live_default():
    """A typo in a launch override must not leave the operator with no catch
    at all, and must not silently fly the OTHER arm of an A/B: the fallback
    is the live default, which is the only value a session can fly without
    having chosen it."""
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter('mocap', name='catch_aim_source')])
    node._start_pattern(_flown_columns_goal())
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
    # Since 2026-10-14 the goal-ACCEPT possession check meets a stale hand
    # first (an unread hand is an unread CUP too); the sizing guard itself
    # is still pinned directly below.
    assert 'REJECTED_BALL_UNKNOWN' in resp.message and 'stale' in resp.message
    assert node._executor is None
    assert prelevel.calls == [], 'moved the platform on an unread hand'
    _period, why_not = node._opening_rest_period()
    assert 'hand_telemetry is stale or absent' in why_not


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
    assert execu.end_code == ex.LANE_REFUSED
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


# ── R5 sitting 7 (2026-10-13): no new goal while a lane is LATCHED ──────────
# A stream stall latched the LEG lane mid-throw; the latch clears only on a
# disarm/arm edge, so the next goal dispatched Ball Butler and ended 0.5 s
# later on the same latch. `_start_pattern` now refuses up front.

def _latched_link_status(sched_refused, latched):
    return DiagnosticStatus(values=[
        KeyValue(key='fault_state', value='NONE'),
        KeyValue(key='sched_refused', value=str(int(sched_refused))),
        KeyValue(key='sched_hold_latched', value=str(int(latched)))])


def _try_self_toss_start(node, tmp_path):
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _prelevel_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)):
        return node._start_pattern(_self_toss_goal())


def test_a_latched_lane_refuses_the_next_goal(tmp_path):
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_lane_latched_bit'
    node._on_link_status(_latched_link_status(1435, latched=1))
    resp = _try_self_toss_start(node, tmp_path)
    assert resp.success is False
    assert 'LATCHED' in resp.message and 'auto' in resp.message
    assert 'IDLE' not in resp.message
    assert node._executor is None


def test_a_still_rising_counter_refuses_without_the_bit(tmp_path):
    """An older bridge node (no `sched_hold_latched` key): a counter that rose
    on the last publish is the latch refusing every frame."""
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_lane_latched_rising'
    node._on_link_status(_link_status(788))
    node._on_link_status(_link_status(805))
    resp = _try_self_toss_start(node, tmp_path)
    assert resp.success is False
    assert 'still rising' in resp.message
    assert node._executor is None


def test_an_old_rise_with_the_bit_clear_is_history(tmp_path):
    """The latch cleared at the disarm/arm edge (bit 0) and the counter froze:
    a rise older than the window must not block the next goal."""
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_lane_latched_history'
    node._on_link_status(_latched_link_status(1438, latched=1))
    node._on_link_status(_latched_link_status(1439, latched=0))
    node._sched_refused_rise_mono -= sn._LANE_LATCH_RISE_WINDOW_S + 0.1
    node._on_link_status(_latched_link_status(1439, latched=0))
    resp = _try_self_toss_start(node, tmp_path)
    assert resp.success is True, resp.message


def test_a_stale_link_status_does_not_refuse_the_goal(tmp_path):
    """Unobserved is not a refusal (`_hand_lane_refused`'s own choice)."""
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_lane_latched_stale'
    node._on_link_status(_latched_link_status(1435, latched=1))
    node._sched_refused_mono -= sn._TRAJ_STATUS_STALE_S + 0.1
    resp = _try_self_toss_start(node, tmp_path)
    assert resp.success is True, resp.message


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
    """No `Platform` body has ever been seen (`node._platform_mocap_xyz`
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
        node._platform_mocap_xyz.append((now, 0.0, 0.0,
                                         float(hw.GEOM_INITIAL_HEIGHT_MM)))
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
    """`_on_mocap` (extended, plan § 1) appends the Platform body's position
    to the frame check's buffer and ignores every other body -- as a WORLD
    point since 2026-10-19: x/y as published, z plus the platform shift
    `mocap_interface` subtracts from non-base bodies."""
    node, _client = _node_with_client(hand=False, frame=False)
    node._on_mocap(RigidBodyPoses(bodies=[
        RigidBodyPose(name='Base', pose=_pose_at(10.0, 20.0)),
        RigidBodyPose(name='Platform', pose=_pose_at(3.0, 4.0)),
    ]))
    assert len(node._platform_mocap_xyz) == 1
    _t, x, y, z = node._platform_mocap_xyz[-1]
    assert (x, y) == (3.0, 4.0)
    assert z == 0.0 + float(hw.GEOM_INITIAL_HEIGHT_MM)


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
    node._platform_mocap_xyz.clear()
    _frame_ready(node, plat_x=2.1, plat_y=31.1, cmd_x=0.0, cmd_y=0.0)
    node._frame_check_error()                       # over the 25 mm bound
    assert node._mocap_to_schedule_mm == pytest.approx((-1.56, -8.53))
    node._platform_mocap_xyz.clear()                 # cannot evaluate
    node._frame_check_error()
    assert node._mocap_to_schedule_mm == pytest.approx((-1.56, -8.53))
    assert len(warnings) == 2 and all('NOT updated' in w for w in warnings)
    assert 'keeping the earlier (x +1.6, y +8.5) mm' in warnings[0]


# ═════════════════════════════════════════════════════════════════════════════
# R4 reload (brief step 1: skill_node's BB orchestration)
# ═════════════════════════════════════════════════════════════════════════════

def _good_box_path(tmp_path, *, apex_band_m=(0.85, 0.95), hop=False,
                   columns=False, oval=False, oval_half_m=0.02,
                   oval_dwell_s=None):
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
        limits={'leg_vel_mmps': _SESSION_LEG_VEL_MMPS, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)},
        gate_hash=adm.gate_hash(), swept_at='2026-09-13',
        dwell_s=sn._DEFAULT_DWELL_S, planner_mode='level')
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
                limits={'leg_vel_mmps': _SESSION_LEG_VEL_MMPS, 'leg_acc_mmps2': 5000.0,
                       'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)},
                gate_hash=adm.gate_hash(), swept_at='2026-09-13',
                dwell_s=sn._DEFAULT_DWELL_S, planner_mode='level'))
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
                limits={'leg_vel_mmps': _SESSION_LEG_VEL_MMPS, 'leg_acc_mmps2': 5000.0,
                       'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)},
                gate_hash=adm.gate_hash(), swept_at='2026-09-13',
                dwell_s=sn._DEFAULT_DWELL_S, planner_mode='level'))
    if oval:
        # The one-ball oval (plan two-ball-oval): ONE box, T -> C at
        # separation 150 mm (columns_sites(150) = P1 (-75, 0), P2 (+75, 0)),
        # box label 'oval_1ball' (the fed variant flies the same cell).
        # `oval_half_m` is the landing half-extent (workspace-rule tests
        # vary it); `oval_dwell_s` the stamp (dwell-mismatch tests vary it).
        boxes.append(adm.AdmissibleBox(
            site_pair=('P1', 'P2'), apex_band_m=apex_band_m,
            landing_xy_m=((-oval_half_m, oval_half_m),
                          (-oval_half_m, oval_half_m)), apex_m=(0.85, 0.95),
            pattern='oval_1ball', release_site_xy_mm=(-75.0, 0.0),
            target_site_xy_mm=(75.0, 0.0),
            limits={'leg_vel_mmps': _SESSION_LEG_VEL_MMPS, 'leg_acc_mmps2': 5000.0,
                   'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)},
            gate_hash=adm.gate_hash(), swept_at='2026-09-13',
            dwell_s=sn._DEFAULT_DWELL_S if oval_dwell_s is None
            else oval_dwell_s,
            planner_mode='level'))
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
    """Drive `node` through the full R4 reload start path (an EMPTY-cup
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
    # `seated=False`: a reload's first skill is the Ball Butler catch, so the
    # goal-ACCEPT possession check (2026-10-14) needs an EMPTY cup.
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)
    _prelevel_ready(node)
    _reload_ready(node, **reload_kw)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
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
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    prelevel_client = _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
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
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    _bb_ready(node, after_check=False)          # BB idle, ball in hand, fresh
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
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
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    _bb_ready(node, ball_in_hand=False, after_check=False)   # empty hand
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
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
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    _bb_ready(node, after_check=False)
    node._bb_heartbeat_mono -= sn._BB_HEARTBEAT_FRESH_S + 0.1
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
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
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    _prelevel_ready(node)
    reload_client, throw_client = _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
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
    the attempt `LANE_REFUSED` (force_hold) in that window. When the stale
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
    assert node._goal_end_code == ex.LANE_REFUSED, node._goal_end_code
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
# R5 (2026-09-30, `not_settled_report.md` Q4 option (d)): a bounded, once-
# per-attempt Jetson-side retry on the literal token
# THROW_ABORTED_NOT_SETTLED -- the firmware sends no frame on this code
# alone (HandTrajectoryStreamer.h:117-122 (FW 5): ball retained, hand at rest).
# ═════════════════════════════════════════════════════════════════════════════

def test_r5_not_settled_retries_once_and_refires_the_throw(tmp_path):
    """The FIRST `THROW_ABORTED_NOT_SETTLED` on an attempt re-fires
    `bb/throw_at_target` instead of ending the attempt: the ctx survives,
    re-arms `await_announcement`, and is marked so a second NOT_SETTLED
    cannot retry again."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    assert node._reload_ctx is not None
    assert node._reload_ctx.phase == 'await_announcement'
    assert node._reload_ctx.not_settled_retried is False
    assert len(node._bb_throw_cli.calls) == 1

    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_NOT_SETTLED (axis=1, detail1=120)'))

    assert node._reload_ctx is not None
    assert node._reload_ctx.phase == 'await_announcement'
    assert node._reload_ctx.not_settled_retried is True
    assert len(node._bb_throw_cli.calls) == 2
    assert node._goal_end_code == ''
    assert node._executor is not None and not node._executor.attempt_ended


def test_r5_a_second_not_settled_on_the_same_attempt_ends_it(tmp_path):
    """The retry is bounded to ONCE per attempt: a second
    `THROW_ABORTED_NOT_SETTLED` on the SAME ctx ends it exactly like every
    other refusal always has -- no third `bb/throw_at_target` call."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_NOT_SETTLED (axis=1, detail1=120)'))
    assert node._reload_ctx is not None
    assert len(node._bb_throw_cli.calls) == 2

    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_NOT_SETTLED (axis=1, detail1=95)'))

    assert node._reload_ctx is None
    assert len(node._bb_throw_cli.calls) == 2
    assert node._executor is not None
    assert node._executor.end_code == 'REJECTED_BB(THROW_ABORTED_NOT_SETTLED)'
    node._on_tick()
    assert node._goal_end_code == 'REJECTED_BB(THROW_ABORTED_NOT_SETTLED)'


def test_r5_a_different_refusal_code_does_not_retry(tmp_path):
    """Only the literal token `THROW_ABORTED_NOT_SETTLED` retries -- every
    other refusal ends the attempt on the FIRST occurrence, unchanged, since
    the firmware's no-frame-sent guarantee is specific to that one code."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    assert len(node._bb_throw_cli.calls) == 1

    node._on_bb_throw_outcome(
        String(data='THROW_REJECTED_BAD_STATE (axis=n/a, detail1=0)'))

    assert len(node._bb_throw_cli.calls) == 1
    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.end_code == 'REJECTED_BB(THROW_REJECTED_BAD_STATE)'


def test_r5_not_settled_retry_extends_the_deadline(tmp_path):
    """The retry must not be killed by `_check_reload_timeout` on the
    ORIGINAL (now stale) deadline -- `_fire_reload_throw` recomputes and
    extends `ctx.deadline_mono` exactly as it does for the first fire."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    # An about-to-expire deadline (same idiom this file already uses
    # elsewhere, e.g. `_reload_ctx.deadline_mono = time.perf_counter() -
    # 1.0`).
    node._reload_ctx.deadline_mono = time.perf_counter() + 0.001

    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_NOT_SETTLED (axis=1, detail1=120)'))

    assert node._reload_ctx is not None
    # Floored throw delay (2.5 s) + the ToF ceiling (1.5 s) + the announce
    # timeout (1.0 s) = 5.0 s -- comfortably clear of the stale ~0 s deadline.
    assert node._reload_ctx.deadline_mono - time.perf_counter() > 3.0
    node._check_reload_timeout()
    assert node._reload_ctx is not None


def test_r5_a_stop_during_the_not_settled_retry_cancels_cleanly(tmp_path):
    """A Stop landing while the retry's own `bb/throw_at_target` round trip
    is in flight must not resurrect the attempt or double-end it -- the same
    `_still_current(ctx)` guard `_fire_reload_throw` already relies on for
    the FIRST fire
    (test_c1_a_throw_call_overtaken_by_a_lane_refusal_does_not_overwrite_the_end_code)."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message

    class _StopOnCallThrowClient(_RecordingClient):
        def __init__(self, node, **kw):
            super().__init__(**kw)
            self._node = node

        def call_async(self, req):
            fut = super().call_async(req)
            self._node._stop_attempt()
            return fut

    node._bb_throw_cli = _StopOnCallThrowClient(
        node, response=_bb_throw_response(success=True, message='ok',
                                          predicted_tof_s=0.7,
                                          throw_delay_s=3.0),
        ready=True)

    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_NOT_SETTLED (axis=1, detail1=120)'))

    assert len(node._bb_throw_cli.calls) == 1
    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.end_code == 'STOPPED'


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
    _cup(node, seated=True)   # a self-toss throws a held ball (accept check)
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
    the field guard latched with NO `LANE_REFUSED` END ever logged.
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
    assert node._goal_end_code == ex.LANE_REFUSED


def test_c5b_lane_refused_before_the_bridge_ever_dispatches_holds_only_once(tmp_path):
    """A refusal caught while the bridge executor is STILL alive (never
    ticked yet) exercises a SECOND path to the same hold:
    `_check_reload_lane_refused` (via `_end_attempt`, `force_hold=True`)
    marks the executor `attempt_ended` and fires the hold, then `_on_tick`'s
    OWN inline `LANE_REFUSED` branch (later in the SAME tick, once it
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
    assert node._goal_end_code == ex.LANE_REFUSED


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
        """R5 (owner decision D3, 2026-09-30): an EMPTY cup on a columns
        goal is no longer refused -- it ACCEPTS and arms the BB
        choreography (`await_bb_idle`), same as a reload='self_toss'
        goal.

        R5 sitting 2 (2026-10-02): the default `columns_feed_site` ('P1')
        holds ball A at P2 and feeds P1 -- the site NEAREST Ball Butler
        along its bearing, so B's descent never crosses A's own column
        (the OLD default, P1-holds/P2-feeds, let the two balls merge in
        flight on 4/4 attempts). `aim_site` keeps the feed site's NAME
        (P1) but is walked `columns_feed_aim_toward_a_mm` (20 mm default)
        toward A's site -- the swapped layout's undisplaced catch refuses
        LIMIT_ACC at 101 %."""
        node, _client = _node_with_client()
        node._params['plant_id'] = 'test_columns_reload_lift'
        _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
        _reload_ready(node)
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                          _good_box_path(tmp_path, columns=True)), \
             patch.object(sn, '_REPO_ROOT', str(tmp_path)):
            assert (node._juggle_goal(_columns_goal(separation_mm=100.0))
                   == sn.GoalResponse.ACCEPT)
        assert node._executor is not None
        assert node._reload_ctx is not None
        assert node._reload_ctx.kind == 'columns'
        assert node._reload_ctx.phase == 'await_bb_idle'
        assert node._reload_ctx.aim_site.name == 'P1'
        # Hop entry: un-walked feed site, and the bridge parks AT it.
        assert node._reload_ctx.aim_site.cup_mm[:2] == pytest.approx(
            [-50.0, 0.0])
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
        SEATED-cup columns start leaves `_reload_ctx` armed for the FEED
        WAIT, and `_stop_attempt` only clears `_reload_ctx` on its
        "no executor yet" branch -- the bridge executor is still live here,
        so it never does. `_goal_done_event` then never fires and
        `_juggle_execute` (called synchronously below, no thread) blocks
        forever. Confirmed empirically (2026-09-30) -- not this unit's fix
        (no production-code edits), flagged in the handoff. self_toss with
        a SEATED cup (throw first) never touches `_reload_ctx`."""
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

    @pytest.mark.parametrize('pattern', ['self_toss', 'columns'])
    def test_a_tick_during_the_start_compile_does_not_end_the_goal(
            self, tmp_path, pattern):
        """2026-10-02: every Juggle goal reported COMPLETED (0/0) the moment it
        started, while the attempt went on to run -- and with the goal already
        terminal, the GUI Stop (a goal cancel) had nothing to cancel.
        `_start_pattern` cleared `_goal_done_event`, then spent the frame
        check, `_prelevel` round trip and compile with no executor installed;
        the 40 Hz tick runs on its own thread and fired the event in that
        window. Here the tick runs from inside the frame check, which both
        start paths call before installing."""
        node, _client = _node_with_client()
        _reload_ready(node)
        _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
        _prelevel_ready(node)
        real_frame_check = node._frame_check_error

        def frame_check_with_a_concurrent_tick():
            node._on_tick()
            return real_frame_check()

        node._frame_check_error = frame_check_with_a_concurrent_tick
        node._goal_done_event.set()          # the previous goal's, as live
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                          _good_box_path(tmp_path, columns=True)), \
             patch.object(sn, '_REPO_ROOT', str(tmp_path)):
            assert (node._juggle_goal(_goal(pattern, separation_mm=100.0))
                   == sn.GoalResponse.ACCEPT)
        assert not node._goal_done_event.is_set(), (
            'the goal was reported done before its attempt was installed')
        node._on_tick()
        assert not node._goal_done_event.is_set()
        # ...and the attempt's real end still ends the goal.
        node._stop_attempt()
        with node._reload_lock:
            node._reload_ctx = None          # columns' feed wait (see above)
        node._on_tick()
        assert node._goal_done_event.is_set()

    # ── skills/attempt: the GUI's view of every goal, whatever sent it ──

    @staticmethod
    def _attempt_events(node):
        return [(m.name, m.message, {kv.key: kv.value for kv in m.values})
                for m in node._attempt_pub.published]

    @pytest.mark.parametrize('num_cycles, expected', [
        (0, str(sn._DEFAULT_N_THROWS)),   # 0 = the node's n_throws param
        (7, '7'),                          # a terminal goal's own count wins
    ])
    def test_an_accepted_goal_announces_its_resolved_request(
            self, tmp_path, num_cycles, expected):
        node, _client = _node_with_client()
        _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
        _prelevel_ready(node)
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)):
            assert (node._juggle_goal(_self_toss_goal(num_cycles=num_cycles))
                   == sn.GoalResponse.ACCEPT)
        [(name, _msg, values)] = self._attempt_events(node)
        assert name == sn.ATTEMPT_START
        assert values['pattern'] == 'self_toss'
        assert values['n_throws'] == expected
        assert values['reload'] == '0'
        assert set(values) == {'pattern', 'n_throws', 'reload', 'apex_m',
                               'separation_mm', 'dwell_s'}

    def test_a_refused_goal_announces_the_reason(self):
        node, _client = _node_with_client()
        assert node._juggle_goal(_goal('not_a_pattern')) == sn.GoalResponse.REJECT
        [(name, message, values)] = self._attempt_events(node)
        assert name == sn.ATTEMPT_REFUSED
        assert values == {'pattern': 'not_a_pattern'}
        assert 'not_a_pattern' in message      # the refusal's own sentence

    def test_execute_announces_the_attempt_end(self, tmp_path):
        """Same shortest-path-to-idle as the end-code test above."""
        node, _client = _node_with_client()
        _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
        _prelevel_ready(node)
        with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)):
            assert (node._juggle_goal(_self_toss_goal())
                   == sn.GoalResponse.ACCEPT)
        node._stop_attempt()
        node._on_tick()
        gh = MagicMock(is_cancel_requested=False)
        gh.request.pattern = 'self_toss'
        node._juggle_execute(gh)
        name, message, values = self._attempt_events(node)[-1]
        assert name == sn.ATTEMPT_END
        assert message == 'STOPPED'
        assert values == {'pattern': 'self_toss', 'outcome': 'STOPPED',
                          'throws': '0', 'caught': '0'}

    def test_execute_publishes_feedback_and_honours_a_live_cancel(self, tmp_path):
        """Uses self_toss, not columns (R5): a SEATED-cup columns start
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
        with a SEATED cup (throw first) never touches `_reload_ctx` at
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
                limits={'leg_vel_mmps': _SESSION_LEG_VEL_MMPS, 'leg_acc_mmps2': 5000.0,
                       'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)},
                gate_hash=adm.gate_hash(), swept_at='2026-09-13',
                dwell_s=sn._DEFAULT_DWELL_S, planner_mode='level')
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
        _reload_ready(node)
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
        """Uses self_toss, not columns (R5): with a SEATED cup (throw
        first) columns' `_reload_ctx` stays armed for the FEED WAIT past
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
        a SEATED cup never touches `_reload_ctx`."""
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
        _reload_ready(node)
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

def _columns_bb_started(node, tmp_path, plant_id=None):
    """Drive `node` to `_reload_ctx.phase == 'await_announcement'` for a
    columns goal (cup SEATED -> ball B is BB-fed; the goal's `reload` token
    is ignored since 2026-10-17): bridge installed, BB ready, throw fired --
    mirrors `_start_reload`'s own helper shape for the one-ball reload."""
    node._params['plant_id'] = plant_id or ('test_columns_feed_' + str(id(node)))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    return resp


def test_columns_bb_announcement_installs_the_fed_schedule(tmp_path):
    """The announcement path (brief_S2.md item 2): BB's `ThrowAnnouncement`
    for us, naming a landing near the feed AIM point (R5 sitting 2,
    2026-10-02: the default FED layout feeds P1, aim walked 20 mm toward
    A's site P2, `ctx.aim_site` -- not P2 itself, the OLD default's feed
    site), compiles `compile_columns(pattern, feed=...)` and swaps the
    executor -- the SAME claim-then-swap `_install_announced_reload` uses
    for the one-ball reload."""
    node, _client = _node_with_client()
    _columns_bb_started(node, tmp_path)
    assert node._reload_ctx.phase == 'await_announcement'
    aim = node._reload_ctx.aim_site.cup_mm
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=tuple(float(v) for v in aim),
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


def _announce_displaced_feed(node, dx_mm):
    """Announce a Ball Butler landing `dx_mm` in x from the feed site."""
    feed_site = node._reload_ctx.site     # hop: the bridge parks AT the feed site
    ann = _bb_announcement(
        target_id=node._robot_name,
        landing_mm=(float(feed_site.cup_mm[0]) + dx_mm, 0.0, 830.0),
        landing_vel_mm_s=(1058.0, 475.0, -5507.0), throw_time_s=0.5,
        landing_time_s=5.0)
    node._on_announcement(ann)


def test_columns_bb_announcement_refuses_an_uncatchable_feed(tmp_path):
    """The claim-then-swap tail's refusal CONTRACT
    (`_install_columns_schedule`): when the columns compile refuses the
    announced feed, the attempt ends cleanly -- the bridge schedule stays
    (no executor swap), `_reload_ctx` is released, no THROW (or anything)
    reaches the installer, and the compile's own reason text reaches the
    end message. Driven DETERMINISTICALLY by making `compile_columns`
    raise, not by a geometry pin.

    Why not geometry: the old -80 mm pin ("refuses LIMIT_VEL") was measured
    2026-10-14 while the live call was silently compiling with the parked
    entry (`hop_entry=False`). Probe 2026-10-15 under the real hop entry
    (`hop_entry=True`; scratch pytest driving `_on_announcement` with
    `landing_vel=(1058, 475, -5507)`, landing x offset from the feed site):
    -80, +60, +80, +100 mm SWAP (accepted); -100, -110, -120, -150, -200,
    -250, -300, -350, -400 and +200, +250, +300, +400 mm refuse
    `REJECTED_CYCLE_INFEASIBLE(INFEASIBLE: QP infeasible: unbounded ...)`;
    +120 mm refuses LIMIT_ACC; +150/+180 mm refuse LIMIT_VEL (386/496 mm/s).
    `compile_columns` itself accepts every displacement in -400..+400 mm;
    all those refusals are the later `plan_columns_first_cycle` check
    (`REJECTED_COLUMNS_FEED_UNCATCHABLE`), pinned geometrically in
    `test_columns_bb_announcement_refuses_a_feed_landing_far_outside_reach`.
    """
    node, _client = _node_with_client()
    _columns_bb_started(node, tmp_path)
    bridge_schedule = node._executor.schedule
    with patch.object(sn, 'compile_columns',
                      side_effect=ValueError('uncatchable feed (probe)')):
        _announce_displaced_feed(node, 0.0)
    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.schedule is bridge_schedule   # no swap happened
    assert node._executor.schedule.pattern != 'columns'
    assert _client.calls == []            # no THROW (or anything) installed
    # The bridge REST is still live (never ticked) at the refusal, so
    # `_end_attempt` marks IT ended (`_end_attempt`'s own docstring) rather
    # than `_goal_end_code` directly.
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'REJECTED_COLUMNS_UNSCHEDULABLE'
    assert 'uncatchable feed (probe)' in node._executor.end_message


def test_columns_bb_announcement_refuses_a_feed_landing_far_outside_reach(
        tmp_path):
    """Geometry pin for the pre-throw feasibility check
    (`plan_columns_first_cycle` -> `REJECTED_COLUMNS_FEED_UNCATCHABLE`):
    a landing 250 mm outward (-x, away from A's site) of the feed site.
    Pinned in the MIDDLE of the refusing region, not at a cliff: probe
    2026-10-15 (hop entry, sweep in the docstring of `..._refuses_an_
    uncatchable_feed`) -- every offset from -100 to -400 mm refuses
    `REJECTED_CYCLE_INFEASIBLE(INFEASIBLE: QP infeasible: unbounded)`,
    while -80 mm still plans."""
    node, _client = _node_with_client()
    _columns_bb_started(node, tmp_path)
    bridge_schedule = node._executor.schedule
    _announce_displaced_feed(node, -250.0)
    assert node._reload_ctx is None
    assert node._executor.schedule is bridge_schedule
    assert _client.calls == []
    assert node._executor.attempt_ended is True
    assert node._executor.end_code == 'REJECTED_COLUMNS_FEED_UNCATCHABLE'
    assert 'REJECTED_CYCLE_INFEASIBLE' in node._executor.end_message
    assert 'QP infeasible' in node._executor.end_message


def test_columns_bb_announcement_at_the_aim_point_still_installs(tmp_path):
    """The pre-throw feasibility check is a no-op at the real operating
    point -- the feed AIM point (R5 sitting 2, 2026-10-02: the default
    FED layout's undisplaced feed SITE refuses LIMIT_ACC at 101 %,
    `_run_columns`'s own docstring, so the aim is walked
    `columns_feed_aim_toward_a_mm` toward A's site before this check ever
    runs). Re-asserted here, alongside `test_columns_bb_announcement_
    installs_the_fed_schedule`'s own undisplaced-aim case, so a future
    change to the check's margin has a same-file regression pinned
    alongside the refusal case above, with Ball Butler's own real
    arrival velocity this time (the other test's is a plain vertical
    drop)."""
    node, _client = _node_with_client()
    _columns_bb_started(node, tmp_path)
    aim = node._reload_ctx.aim_site.cup_mm
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=tuple(float(v) for v in aim),
        landing_vel_mm_s=(1058.0, 475.0, -5507.0), throw_time_s=0.5,
        landing_time_s=5.0)
    node._on_announcement(ann)
    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.schedule.pattern == 'columns'


def test_columns_feed_bb_bias_mm_shifts_the_request_and_the_feed_prior(tmp_path):
    """B4 (R5 sitting 4): on the fed-columns path (`kind == 'columns'`) the
    point SENT to Ball Butler is walked BACK by `columns_feed_bb_bias_mm`
    (so Ball Butler's own measured landing error cancels), and the
    announced landing used for the feed `LandingPrior` is walked FORWARD
    by it (the announcement reports Ball Butler's belief -- the request
    point it actually received -- not where the ball truly lands).
    `_run_columns`'s own docstring on the parameter."""
    node, _client = _node_with_client()
    node.set_parameters([_MockParameter([29.0, 25.0], name='columns_feed_bb_bias_mm')])
    _columns_bb_started(node, tmp_path)
    aim = node._reload_ctx.aim_site.cup_mm
    throw_req = node._bb_throw_cli.calls[0]
    assert throw_req.target_point_global_mm.x == pytest.approx(float(aim[0]) - 29.0)
    assert throw_req.target_point_global_mm.y == pytest.approx(float(aim[1]) - 25.0)

    # Ball Butler announces its OWN belief -- the request it actually
    # received -- same shape as `test_columns_bb_announcement_installs_
    # the_fed_schedule`.
    ann = _bb_announcement(
        target_id=node._robot_name,
        landing_mm=(throw_req.target_point_global_mm.x,
                   throw_req.target_point_global_mm.y, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(0.0, 0.0, -3000.0), throw_time_s=0.5,
        landing_time_s=5.0)
    node._on_announcement(ann)
    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.schedule.pattern == 'columns'
    catches = [s for s in node._executor.schedule.skills if s.kind == sn.CATCH]
    first = min(catches, key=lambda s: s.t_abs_s)
    # Walked FORWARD by the bias from the announced (request) landing --
    # back to the TRUE aim point, since announced == aim - bias here.
    assert first.landing_prior.pos_mm[0] == pytest.approx(float(aim[0]), abs=1e-6)
    assert first.landing_prior.pos_mm[1] == pytest.approx(float(aim[1]), abs=1e-6)


def test_columns_feed_bb_bias_mm_leaves_self_toss_reload_untouched(tmp_path):
    """B4: the bias applies ONLY to `kind == 'columns'` -- a plain
    self-toss/hop reload request (`kind == 'reload'`) is unchanged by a
    non-zero `columns_feed_bb_bias_mm` (same request point
    `test_reload_installs_the_bridge_rest_then_calls_bb_reload_and_throw`
    asserts with the parameter at its default)."""
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_reload_bridge_bias'
    node.set_parameters([_MockParameter([29.0, 25.0], name='columns_feed_bb_bias_mm')])
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    _prelevel_ready(node)
    _reload_client, throw_client = _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is True, resp.message
    bridge_rest = node._executor.schedule.skills[0]
    node._executor.tick(bridge_rest.dispatch_s())
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    throw_req = throw_client.calls[0]
    assert throw_req.target_point_global_mm.x == pytest.approx(-50.0)
    assert throw_req.target_point_global_mm.y == pytest.approx(0.0)


def test_columns_feed_bb_bias_mm_leaves_columns_1ball_reload_untouched(tmp_path):
    """B4: `kind == 'columns_1ball'` (ball A's own reload, the OTHER half
    of `columns_1ball_fed`) is unaffected too -- only `kind == 'columns'`
    is biased, same request point
    `test_columns_1ball_reload_true_starts_the_plain_reload_choreography`
    asserts with the parameter at its default."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_1ball_reload_bias_' + str(id(node))
    node.set_parameters([_MockParameter([29.0, 25.0], name='columns_feed_bb_bias_mm')])
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    _reload_client, throw_client = _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_1ball_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    a_site = node._reload_ctx.site
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    throw_req = throw_client.calls[0]
    assert throw_req.target_point_global_mm.x == pytest.approx(
        float(a_site.catch_site_mm()[0]))
    assert throw_req.target_point_global_mm.y == pytest.approx(
        float(a_site.catch_site_mm()[1]))


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



def _columns_1ball_goal(**overrides):
    return _goal('columns_1ball', **overrides)


def test_columns_1ball_is_one_of_the_patterns_the_juggle_goal_accepts():
    assert 'columns_1ball' in sn.SkillNode._PATTERNS


def test_columns_1ball_reload_false_compiles_a_phantom_schedule_immediately(
        tmp_path):
    """Unlike plain `columns` (which arms a FEED WAIT), a SEATED cup here
    compiles the whole schedule straight away (`compile_rest_columns`: an
    opening REST, then `compile_columns`) -- A is assumed already held,
    `_run_one_ball`'s own non-reload shape -- and ball A ends up real at P2,
    the phantom at P1 (the default `_columns_feed_site_name`, owner's own
    framing: "A at P2, B phantom at P1")."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_1ball_' + str(id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_1ball_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    assert node._reload_ctx is None       # no feed wait armed -- no real B
    assert node._executor is not None
    schedule = node._executor.schedule
    assert schedule.pattern == 'columns'       # the box-lookup contract
    assert schedule.phantom_balls == (1,)
    assert schedule.is_phantom(1) is True
    assert schedule.is_phantom(0) is False
    ball0_site = next(s.site.name for s in schedule.skills if s.ball_id == 0)
    ball1_site = next(s.site.name for s in schedule.skills if s.ball_id == 1)
    assert ball0_site == 'P2'       # A, real
    assert ball1_site == 'P1'       # B, phantom


def test_columns_1ball_reload_false_opens_on_a_rest_from_the_activate_park(
        tmp_path):
    """R5 sitting 6 (2026-10-05): the first Juggle after ACTIVATE, ball
    hand-seated, hand parked at 0.0 rev -- 7 attempts ended `INFEASIBLE` at
    THROW 0 because the node logged "opening REST homes the hand" and never
    scheduled it. The schedule now OPENS on that REST: ball A's site, ball
    held, over the period `_opening_rest_period` sized from the parked hand;
    THROW 0's launch window starts where the REST ends."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_1ball_park_' + str(id(node))
    _freshen(node, pos_meas=0.0, pos_cmd=0.0)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_1ball_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    schedule = node._executor.schedule
    rest0 = schedule.skills[0]
    assert rest0.kind == 'REST'
    assert rest0.site.name == 'P2' and rest0.holds_ball is True
    assert rest0.window_s == pytest.approx(sn.floor_lift_s(0.0))
    throw0 = next(s for s in schedule.skills if s.kind == 'THROW')
    assert throw0.ball_id == 0
    assert throw0.t_abs_s - throw0.window_s == pytest.approx(rest0.t_abs_s)
    # the first dispatch is still ahead of "now": nothing is already late
    now = node.get_clock().now().nanoseconds / 1e9
    assert rest0.dispatch_s() >= now


#: Every ACCEPTED (pattern, cup) cell of the cup-keyed rule table
#: (`_CUP_RULE`, 2026-10-17). `magazine` has no schedule (own tests:
#: tests/ros/test_skill_node_magazine.py).
_ACCEPTED_CUP_CELLS = [
    (pattern, ev) for pattern, row in sn._CUP_RULE.items()
    if pattern != 'magazine'
    for ev, decision in row.items()
    if decision in (sn.RELOAD_FIRST, sn.THROW_FIRST)]


@pytest.mark.parametrize('pattern,evidence', _ACCEPTED_CUP_CELLS)
def test_every_pattern_entry_opens_on_a_rest(tmp_path, pattern, evidence):
    """THE START-STATE CONTRACT (R5 sitting 6, 2026-10-05): whatever a Juggle
    goal starts, the first skill it puts on the machine is a REST -- the one
    skill that may be planned from wherever the machine is (parked hand,
    off-site, tilted) and that leaves it level, homed and at the site, which
    is the only start every gate (the sim gate, the box sweep) certifies a
    THROW or CATCH from. `columns_1ball` with a SEATED cup was the one
    entry that broke it, and it cost a sitting: 9 attempts, 0 throws.

    Parametrised over every `(pattern, cup evidence)` cell the node accepts
    (the operator's `reload` flag is gone: the cup decides); the last
    assert fails when `_PATTERNS` grows without this list growing with it."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_opens_on_rest_%s_%s' % (pattern, id(node))
    # the ACTIVATE park hand, cup as the accept possession check needs it
    _freshen(node, pos_meas=0.0, pos_cmd=0.0,
             seated=(evidence == ball_possession.EVIDENCE_SEATED))
    _prelevel_ready(node)       # self_toss/hop pre-level; unused by columns
    _reload_ready(node)         # a SEATED columns / an EMPTY cup uses BB
    # the fixture's hop boxes are the 250 mm sweep's; the others sit at 100 mm
    separation_mm = (250.0 if pattern == 'hop'
                     else 150.0 if pattern.startswith('oval_1ball') else 100.0)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, hop=True, columns=True,
                                     oval=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_goal(pattern, separation_mm=separation_mm))
    assert resp.success is True, resp.message
    first = node._executor.schedule.skills[0]
    assert first.kind == 'REST', (
        '%s with a %s cup puts a %s on the machine first' % (pattern, evidence,
                                                             first.kind))
    covered = {'self_toss', 'hop', 'columns', 'columns_1ball',
               'columns_1ball_fed', 'oval_1ball', 'oval_1ball_fed', 'reload'}
    # 'magazine' has no schedule (one MAGAZINE segment from rest; its own
    # tests are tests/ros/test_skill_node_magazine.py).
    assert set(sn.SkillNode._PATTERNS) - {'magazine'} == covered


def test_columns_1ball_reload_true_starts_the_plain_reload_choreography(
        tmp_path):
    """An EMPTY cup reuses the self-toss reload verbatim for ball A ALONE
    -- `kind='columns_1ball'` on the ctx, not `kind='columns'` (that one
    feeds ball B) -- and asks Ball Butler to throw at A's OWN site, not a
    feed-aim-walked point (there is nothing to walk toward; B never
    arrives)."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_1ball_reload_' + str(id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    reload_client, throw_client = _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_1ball_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    assert node._reload_ctx is not None
    assert node._reload_ctx.kind == 'columns_1ball'
    assert node._reload_ctx.phase == 'await_bb_idle'
    a_site = node._reload_ctx.site
    assert a_site.name == 'P2'
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    assert node._reload_ctx.phase == 'await_announcement'
    throw_req = throw_client.calls[0]
    assert throw_req.target_point_global_mm.x == pytest.approx(
        float(a_site.catch_site_mm()[0]))
    assert throw_req.target_point_global_mm.y == pytest.approx(
        float(a_site.catch_site_mm()[1]))


def test_columns_1ball_announcement_installs_the_merged_schedule(tmp_path):
    """Once Ball Butler's announcement for ball A lands, the executor swaps
    to `compile_reload_columns`'s merged schedule: the reload lead-in
    (REST/CATCH/REST) then the columns tail, phantom B included."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_1ball_announce_' + str(id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_1ball_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    a_site = node._reload_ctx.site
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    ann = _bb_announcement(
        target_id=node._robot_name,
        landing_mm=tuple(float(v) for v in a_site.catch_site_mm()),
        landing_vel_mm_s=(0.0, 0.0, -3000.0), throw_time_s=0.5,
        landing_time_s=5.0)
    node._on_announcement(ann)
    assert node._reload_ctx is None
    assert node._executor is not None
    schedule = node._executor.schedule
    assert schedule.pattern == 'columns'
    assert schedule.phantom_balls == (1,)
    kinds = [s.kind for s in schedule.skills[:4]]
    assert kinds == ['REST', 'CATCH', 'REST', 'THROW']
    assert schedule.skills[1].landing_prior is not None   # BB's own catch
    assert schedule.skills[3].ball_id == 0
    # The columns release sits one launch window AFTER the decay REST's end
    # (`compile_reload_columns`, 2026-10-04: anchored AT the decay end, the
    # launch window fell inside the decay slew and refused LIMIT_JERK).
    assert schedule.skills[3].t_abs_s == pytest.approx(
        schedule.skills[2].t_abs_s + schedule.skills[3].window_s)


def test_columns_1balls_phantom_first_visit_never_refuses_no_ball_or_no_release(
        tmp_path):
    """The whole point of B2's three enforcement points, exercised through
    the node's own install path: dispatching the phantom's THROW and CATCH
    produces ordinary ACCEPTED installs and never ends the attempt
    `REJECTED_NO_BALL` (the hand-possession ladder) or `ABORTED_NO_RELEASE`
    (the release-evidence timeout) -- neither of which has anything to read
    for a ball that was never really thrown.

    Ball A's OWN release evidence (a real ball, unaffected by B2) is
    confirmed through a TRACKER fit rather than the hand sensor -- the
    sensor-latch path is sensitive to exactly which instant each discrete
    tick lands on (ticking at successive skills' own ``dispatch_s()``, as
    this file's established idiom does, visits instants BEFORE ball A's
    release as often as after it), where the tracker path is not: it simply
    does nothing before the release and confirms once at or after it,
    which is what this test needs from a ball it is not actually about.
    """
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_columns_1ball_dispatch_' + str(id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_1ball_goal(separation_mm=100.0,
                                                       num_cycles=2))
    assert resp.success is True, resp.message
    executor = node._executor
    ball0_catch_idx, ball0_catch = next(
        (i, s) for i, s in enumerate(executor.schedule.skills)
        if s.kind == 'CATCH' and s.ball_id == 0)
    fit = executor._predicted_landing(ball0_catch_idx, ball0_catch)
    # Patched on the EXECUTOR, not `node._tracker` -- the executor already
    # captured `self._tracker` (the bound method) by value at construction
    # (`_run_columns_1ball`), so patching the node's own attribute after the
    # fact would not reach it.
    executor.tracker = lambda ball_id: (
        dataclasses.replace(fit, from_fit=True) if ball_id == 0 else None)
    forbidden = {'REJECTED_NO_BALL', 'ABORTED_NO_RELEASE'}
    for sk in executor.schedule.skills:
        executor.tick(sk.dispatch_s())
        assert executor.end_code not in forbidden, (
            executor.end_code, executor.end_message)
    phantom_calls = [c for c in _client.calls
                    if getattr(c, 'ball_id', None) == 1]
    assert phantom_calls, 'the phantom never dispatched -- nothing to prove'


# ---------------------------------------------------------------------------
# B2 (R5 sitting 4): `columns_1ball_fed` -- the OTHER half of `columns_1ball`
# above: ball A (schedule id 0) phantom, ball B (schedule id 1) real and fed
# by Ball Butler, through `_run_columns(..., phantom_a=True)` (the SAME fed
# path the plain `columns` pattern uses, not `_run_columns_1ball`).
# ---------------------------------------------------------------------------

def _columns_1ball_fed_goal(**overrides):
    return _goal('columns_1ball_fed', **overrides)


def test_columns_1ball_fed_is_one_of_the_patterns_the_juggle_goal_accepts():
    assert 'columns_1ball_fed' in sn.SkillNode._PATTERNS


def test_columns_1ball_fed_refuses_reload_false():
    """Unlike plain `columns`, this pattern has no ball A to fall back on
    for a human-lob feed -- Ball Butler is the ONLY feed for ball B, so
    A SEATED cup is refused outright rather than silently arming a
    tracker-only wait that can never be satisfied."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_1ball_fed_refuse_' + str(id(node))
    resp = node._start_pattern(_columns_1ball_fed_goal(separation_mm=100.0))
    assert resp.success is False
    assert 'reload' in resp.message
    assert node._executor is None
    assert node._reload_ctx is None


def test_columns_1ball_fed_reload_true_arms_the_fed_choreography_with_empty_cup_bridge(
        tmp_path):
    """An EMPTY cup routes through the SAME `kind='columns'` BB-feed
    choreography the plain `columns` pattern uses (`_run_columns`'s own
    `phantom_a` paragraph) -- not `columns_1ball`'s own `kind='columns_
    1ball'` reload (that one reloads ball A itself; here ball A never
    arrives at all). The opening bridge REST carries `holds_ball=False`
    (the cup is empty), and the compiled `columns_pattern` names ball 0 as
    the phantom."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_1ball_fed_reload_' + str(id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_1ball_fed_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    assert node._reload_ctx is not None
    assert node._reload_ctx.kind == 'columns'
    assert node._reload_ctx.phase == 'await_bb_idle'
    assert node._reload_ctx.columns_pattern.phantom_balls == (0,)
    assert node._reload_ctx.bridge.skills[0].holds_ball is False


# ═════════════════════════════════════════════════════════════════════════════
# 2026-10-11 response starvation (sitting 7 re-fly): every waited client lives
# on a dedicated helper node with its own executor thread; first end wins; the
# columns feed swap declines after an END; the BB no-response text is neutral.
# ═════════════════════════════════════════════════════════════════════════════

def test_every_waited_client_is_created_on_the_dedicated_client_node():
    """All five clients `_wait_future` blocks on must sit on the helper node
    (spun by its own executor), never on the 4-thread pool's SkillNode: with
    the pool's four threads all parked in blocking waits nobody could dispatch
    the service responses (install + bb/throw_at_target both timed out at
    2.0 s on 2026-10-11)."""
    created = []
    orig = _MockNodeClass.create_client

    def spy(self, srv_type, name, **kw):
        created.append((self, name))
        return orig(self, srv_type, name, **kw)

    with patch.object(_MockNodeClass, 'create_client', spy):
        node = sn.SkillNode()
    names = {n for _owner, n in created}
    # + bb/cancel_throw (2026-10-13): never waited on, but on the helper node
    # so its done-callback (the one cancel log line) runs on that executor.
    assert names == {'trajectory/install_segment', 'trajectory/go_to_pose',
                     'trajectory/hold', 'bb/reload', 'bb/throw_at_target',
                     'bb/cancel_throw', 'recover_lane'}
    assert all(owner is node._client_node for owner, _n in created)
    assert node._client_node is not node


def test_main_spins_the_client_node_on_its_own_executor_thread(monkeypatch):
    spun = []

    class _Rec:
        def __init__(self, *a, **kw):
            self.nodes = []
            self.kw = kw

        def add_node(self, n):
            self.nodes.append(n)

        def spin(self):
            spun.append((threading.current_thread(), self))

        def shutdown(self):
            pass

    monkeypatch.setattr(sn, 'MultiThreadedExecutor', _Rec)
    monkeypatch.setattr(sn, 'SingleThreadedExecutor', _Rec)
    monkeypatch.setattr(rclpy, 'init', lambda *a, **k: None)
    monkeypatch.setattr(rclpy, 'shutdown', lambda *a, **k: None)
    sn.main()
    deadline = time.time() + 2.0
    while len(spun) < 2 and time.time() < deadline:
        time.sleep(0.01)
    assert len(spun) == 2
    by_exec = {ex_: th for th, ex_ in spun}
    main_ex = next(e for e in by_exec if 'num_threads' in e.kw)
    client_ex = next(e for e in by_exec if 'num_threads' not in e.kw)
    assert main_ex.kw['num_threads'] == 4
    assert len(client_ex.nodes) == 1 and client_ex.nodes[0]._name == 'skill_node_clients'
    assert main_ex.nodes[0] is not client_ex.nodes[0]
    # its own thread, not the main one
    assert by_exec[client_ex] is not threading.main_thread()
    assert by_exec[client_ex] is not by_exec[main_ex]


def test_columns_feed_swap_declines_after_the_goal_ended(tmp_path):
    """A feed compile that wins `_tick_lock` AFTER an END must never arm a
    fresh executor: the ctx is still current but `_goal_end_code` is set."""
    node, _client = _node_with_client(response=_response())
    _columns_bb_started(node, tmp_path)
    _finish_bridge(node)
    ctx = node._reload_ctx
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(50.0, 0.0, 830.0),
        landing_vel_mm_s=(0.0, 0.0, -3000.0), throw_time_s=0.5,
        landing_time_s=5.0)
    with node._reload_lock:
        ctx.announced = True
    assert node._goal_end_code == ''              # the bridge REST answered
    node._goal_end_code = 'SERVICE_TIMEOUT'      # ended, ctx NOT cleared
    bridge_executor = node._executor
    node._install_announced_reload(ctx, ann)
    assert node._executor is bridge_executor
    assert node._goal_end_code == 'SERVICE_TIMEOUT'


def test_columns_feed_swap_declines_over_a_refused_live_executor(tmp_path):
    """The other form of an END (`_ended_code_for_swap`): the live executor
    was refused (`attempt_ended`) but has not retired yet, so
    `_goal_end_code` is still ''. Swapping over it would lose its end code
    and arm a fresh executor behind it."""
    node, _client = _node_with_client(response=_response())
    _columns_bb_started(node, tmp_path)
    _finish_bridge(node)
    ctx = node._reload_ctx
    # Land AT the feed site (the aim BB is asked for), not 100 mm off it: the
    # old (50, 0) landing is a 100 mm displacement the realised-plan
    # CUP_SEATING_CONE check (30 deg, 2026-10-18; 45 deg judged from the
    # stroke end since 2026-10-19) refused (31.3 deg, 1/11 loaded knots)
    # with REJECTED_COLUMNS_FEED_UNCATCHABLE, which ended the attempt before
    # the swap path under test.  At the aim the same
    # (0, 0, -3000) mm/s arrival plans clean (measured 2026-10-18: seating
    # 1.6 deg, closing ratio 0.738, skid 7.6 mm/s) so the swap-decline is
    # what is actually exercised.
    ann = _bb_announcement(
        target_id=node._robot_name,
        landing_mm=tuple(float(v) for v in ctx.aim_site.cup_mm),
        landing_vel_mm_s=(0.0, 0.0, -3000.0), throw_time_s=0.5,
        landing_time_s=5.0)
    with node._reload_lock:
        ctx.announced = True
    # The bridge executor has retired by now; stand in a refused, not yet
    # retired one (the swap reads only these two fields).
    import types
    bridge_executor = types.SimpleNamespace(
        attempt_ended=True, end_code='SERVICE_TIMEOUT', done=False)
    node._executor = bridge_executor
    assert node._goal_end_code == ''
    node._install_announced_reload(ctx, ann)
    assert node._executor is bridge_executor
    assert node._ended_code_for_swap() == 'SERVICE_TIMEOUT'


class _NoAnswerClient(_RecordingClient):
    def call_async(self, request):
        fut = super().call_async(request)
        fut._is_bb = True
        return fut


def test_bb_no_response_text_is_neutral_and_first_end_wins(tmp_path):
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_bb_noresp'
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)  # a reload opens with the catch: EMPTY cup
    _prelevel_ready(node)
    _reload_ready(node)
    node._bb_throw_cli = _NoAnswerClient(response=None, ready=True)
    orig_wait = node._wait_future
    node._wait_future = (lambda fut, timeout_s=0.0:
                         None if getattr(fut, '_is_bb', False)
                         else orig_wait(fut))
    seen = []
    orig_end = node._end_attempt
    node._end_attempt = lambda code, msg, **kw: (seen.append((code, msg)),
                                                 orig_end(code, msg, **kw))
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is True, resp.message
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    assert seen and seen[0][0] == 'ABORTED_BB_THROW_TIMEOUT'
    msg = seen[0][1]
    assert 'answered nothing' not in msg and 'silent' not in msg
    assert 'may still have been executed' in msg

    # first end wins: an earlier code is not overwritten (no live executor)
    node._executor = None
    node._goal_end_code = 'SERVICE_TIMEOUT'
    orig_end('ABORTED_BB_THROW_TIMEOUT', 'later')
    assert node._goal_end_code == 'SERVICE_TIMEOUT'


def test_a_magazine_install_encodes_the_wire_and_reads_the_release_off_t_event():
    from jugglebot.motion.skills import magazine_throw as mt
    node, client = _node_with_client(
        response=_response(t_event_mono=50.0, t_release_mono=0.0))
    terminal = mt.MagazineTerminal(
        target_mm=np.array([900.0, -300.0, 750.0]), flight_s=0.9,
        t_release_s=100.0, release_site_mm=np.array([10.0, 20.0, 780.0]),
        rest_site_mm=np.array([0.0, 0.0, 750.0]))
    result = node._installer(mt.MAGAZINE, terminal, 100.0, ball_id=0)
    assert result.accepted is True
    req = client.calls[0]
    assert req.kind == InstallSegment.Request.KIND_MAGAZINE
    assert req.t_event_s == 100.0
    assert list(req.site_mm) == [10.0, 20.0, 780.0]
    assert list(req.target_mm) == [900.0, -300.0, 750.0]
    assert req.flight_s == pytest.approx(0.9)
    assert list(req.rest_site_mm) == [0.0, 0.0, 750.0]
    # a MAGAZINE's release IS its event, like a THROW's
    assert node._pending_event_mono == 50.0


# ═════════════════════════════════════════════════════════════════════════════
# R5 sitting 7 decision 5 (2026-10-13): an attempt that ENDS with a Ball
# Butler throw still counting down cancels it (bb/cancel_throw), once.
# ═════════════════════════════════════════════════════════════════════════════

class _CancelClient(_RecordingClient):
    """bb/cancel_throw stand-in: `_maybe_cancel_bb_throw` checks
    `service_is_ready()` (never `wait_for_service` -- it must not block an
    end path, the 40 Hz tick included)."""

    def service_is_ready(self):
        return self._ready


def _cancel_client(node, *, ready=True, success=True,
                   message='cancel sent (bb/cancel)'):
    resp = Trigger.Response()
    resp.success = success
    resp.message = message
    cli = _CancelClient(response=resp, ready=ready)
    node._bb_cancel_cli = cli
    return cli


def test_bb_cancel_end_with_a_pending_bb_throw_cancels_it_once(tmp_path):
    """The sitting-7 case: the attempt ends while Ball Butler's accepted
    throw (3 s lead) is still counting down -> exactly ONE bb/cancel_throw,
    however many end paths run afterwards."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    assert node._bb_pending_release_mono is not None
    cancel = _cancel_client(node)
    node._end_attempt('ABORTED_TEST', 'ABORTED_TEST')
    assert len(cancel.calls) == 1
    assert node._bb_pending_release_mono is None
    node._maybe_cancel_bb_throw('AGAIN')
    node._stop_attempt()
    assert len(cancel.calls) == 1


def test_bb_cancel_stop_during_the_reload_wait_cancels_the_pending_throw(tmp_path):
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    cancel = _cancel_client(node)
    node._stop_attempt()
    assert len(cancel.calls) == 1


def test_bb_cancel_a_release_already_past_is_not_cancelled():
    node, _client = _node_with_client(response=_response())
    cancel = _cancel_client(node)
    node._bb_pending_release_mono = time.perf_counter() - 0.05
    node._maybe_cancel_bb_throw('MISSED_CATCH')
    assert cancel.calls == []
    assert node._bb_pending_release_mono is None


def test_bb_cancel_no_pending_throw_sends_nothing():
    node, _client = _node_with_client(response=_response())
    cancel = _cancel_client(node)
    node._end_attempt('ABORTED_TEST', 'ABORTED_TEST')
    assert cancel.calls == []


def test_bb_cancel_unavailable_service_neither_blocks_nor_raises(tmp_path):
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    cancel = _cancel_client(node, ready=False)
    node._end_attempt('ABORTED_TEST', 'ABORTED_TEST')
    assert cancel.calls == []
    assert node._bb_pending_release_mono is None


def test_bb_cancel_a_throw_call_overtaken_by_an_end_is_cancelled(tmp_path):
    """The end ran (lane refusal) while `bb/throw_at_target` was in flight,
    so its cancel found no record; the throw the call then reports ACCEPTED
    is cancelled by `_fire_reload_throw` itself."""
    node, _client = _node_with_client(response=_response())
    node._on_link_status(_link_status(4))
    _reload_armed_await_bb_idle(node, tmp_path)
    _finish_bridge(node)
    node._bb_throw_cli = _RacingThrowClient(
        node, response=_bb_throw_response(success=True, throw_delay_s=6.8),
        ready=True)
    node._hold_cli = _hold_client()
    cancel = _cancel_client(node)
    _bb_ready(node)
    node._on_tick()
    assert node._goal_end_code == ex.LANE_REFUSED, node._goal_end_code
    assert len(cancel.calls) == 1


def _throw_call_delivers(node, throw_client, on_call):
    """Wrap ``throw_client.call_async`` so ``on_call()`` runs INSIDE the
    blocking `bb/throw_at_target` round trip -- ball_butler_node publishes
    the ThrowAnnouncement synchronously inside that service handler, so
    the announcement (and the executor swap it drives) lands BEFORE the
    reply on the robot (2026-10-13 22:29 launch, every attempt)."""
    orig_call_async = throw_client.call_async

    def _call_async(request):
        on_call()
        return orig_call_async(request)
    throw_client.call_async = _call_async


def test_bb_cancel_announcement_swap_during_the_throw_call_does_not_cancel_it(tmp_path):
    """THE 2026-10-13 22:29 bug: the announcement's swap retires `_reload_ctx`
    during the throw call -- the NORMAL success ordering -- and the reply's
    overtaken branch read "ctx no longer current" as "the attempt ended" and
    cancelled the throw it had just dispatched (`END ENDED_DURING_BB_DISPATCH`,
    BB `THROW_ABORTED_BY_HOST`), then JB caught a phantom. The swap case must
    send NO cancel and leave the pending release recorded for a real end."""
    node, _client = _node_with_client(response=_response())
    throw_client = _reload_armed_await_bb_idle(node, tmp_path)
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0), throw_time_s=2.0,
        landing_time_s=2.7)
    _throw_call_delivers(node, throw_client, lambda: node._on_announcement(ann))
    cancel = _cancel_client(node)
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    assert len(throw_client.calls) == 1
    assert node._reload_ctx is None
    assert node._executor is not None and not node._executor.attempt_ended
    assert node._reload_throw_inflight is True
    assert cancel.calls == []
    assert node._bb_pending_release_mono is not None
    assert node._goal_end_code == ''
    # A REAL end afterwards still cancels the (still pending) throw, once.
    node._stop_attempt()
    assert len(cancel.calls) == 1


def test_bb_cancel_columns_feed_swap_during_the_throw_call_does_not_cancel_it(tmp_path):
    """The columns sibling: ball B's Ball Butler feed goes through the SAME
    `_fire_reload_throw`, and `_install_columns_schedule`'s swap clears the
    ctx the same way -- two-ball columns with BB feeds must not cancel its
    own feed either."""
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_columns_race_' + str(id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _reload_client, throw_client = _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    ctx = node._reload_ctx
    aim = ctx.aim_site.cup_mm
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=tuple(float(v) for v in aim),
        landing_vel_mm_s=(0.0, 0.0, -3000.0), throw_time_s=0.5,
        landing_time_s=5.0)
    _throw_call_delivers(node, throw_client, lambda: node._on_announcement(ann))
    cancel = _cancel_client(node)
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    assert len(throw_client.calls) == 1
    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.schedule.pattern == 'columns'
    assert not node._executor.attempt_ended
    assert cancel.calls == []
    assert node._goal_end_code == ''


def test_bb_cancel_a_stop_during_the_throw_round_trip_cancels_exactly_once(tmp_path):
    """The true overtaken case the wave-2 cancel was written for: a Stop
    lands while `bb/throw_at_target` is in flight (no announcement yet), its
    own cancel finds nothing recorded, so the reply's branch cancels -- once."""
    node, _client = _node_with_client(response=_response())
    throw_client = _reload_armed_await_bb_idle(node, tmp_path)
    _finish_bridge(node)
    assert node._executor is None
    _throw_call_delivers(node, throw_client, node._stop_attempt)
    cancel = _cancel_client(node)
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    assert node._reload_ctx is None
    assert len(cancel.calls) == 1
    node._stop_attempt()
    node._on_tick()
    assert len(cancel.calls) == 1


def test_bb_cancel_an_end_that_wins_the_compile_cancels_exactly_once(tmp_path):
    """Case (iii): the announcement CLAIMS the ctx, a Stop lands while the
    schedule compiles, the swap declines -- all still inside the throw call.
    The Stop's cancel ran before the release was recorded, so the reply's
    overtaken branch is the ONE path that cancels."""
    node, _client = _node_with_client(response=_response())
    throw_client = _reload_armed_await_bb_idle(node, tmp_path)
    _finish_bridge(node)
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0), throw_time_s=2.0,
        landing_time_s=2.7)
    real_compile = sn.compile_reload

    def _compile_then_stop(*a, **kw):
        sched = real_compile(*a, **kw)
        node._stop_attempt()
        return sched
    _throw_call_delivers(node, throw_client, lambda: node._on_announcement(ann))
    cancel = _cancel_client(node)
    _bb_ready(node)
    with patch.object(sn, 'compile_reload', _compile_then_stop):
        node._maybe_fire_reload_throw()
    assert node._executor is None          # the swap declined
    assert node._reload_ctx is None
    assert len(cancel.calls) == 1
    node._on_tick()
    assert len(cancel.calls) == 1


def _announced_reload_live(node, tmp_path):
    """A reload whose announcement has swapped the real schedule in (bridge
    retired first), with a recording cancel client attached."""
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    _finish_bridge(node)
    ann = _bb_announcement(
        target_id=node._robot_name, landing_mm=(-50.0, 0.0, sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0), throw_time_s=2.0,
        landing_time_s=2.7)
    node._on_announcement(ann)
    assert node._reload_ctx is None
    assert node._executor is not None and not node._executor.attempt_ended
    return _cancel_client(node)


def test_bb_throw_aborted_by_host_ends_a_live_announced_attempt(tmp_path):
    """Phantom-catch guard: a NOT-thrown verdict -- THROW_ABORTED_BY_HOST
    included -- for the throw a LIVE announced executor is waiting on ends
    the attempt, so JB never dispatches a CATCH for a ball that will not
    come (2026-10-13 22:29: REST -> CATCH -> REST against an empty sky).
    The throw is already dead, so no cancel is sent for it."""
    node, client = _node_with_client(response=_response())
    cancel = _announced_reload_live(node, tmp_path)
    executor = node._executor
    catch = next(s for s in executor.schedule.skills if s.kind == sn.CATCH)
    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_BY_HOST (axis=n/a, detail1=2940)'))
    assert executor.attempt_ended is True
    assert executor.end_code == 'REJECTED_BB(THROW_ABORTED_BY_HOST)'
    assert node._reload_throw_inflight is False
    assert cancel.calls == []
    n_before = len(client.calls)
    executor.tick(catch.dispatch_s())
    assert [r for r in client.calls[n_before:]
            if r.kind == InstallSegment.Request.KIND_CATCH] == []


def test_bb_throw_aborted_by_host_ends_a_live_pre_announcement_wait(tmp_path):
    """The same verdict before any announcement (a deferred cancel answered
    CANCELLED_BEFORE_DISPATCH, or an operator cancel): nothing will be
    announced, so waiting out ABORTED_NO_ANNOUNCEMENT is waiting for nothing."""
    node, _client = _node_with_client(response=_response())
    resp = _start_reload(node, tmp_path)
    assert resp.success is True, resp.message
    assert node._reload_ctx.phase == 'await_announcement'
    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_BY_HOST (axis=n/a, detail1=4200)'))
    assert node._reload_ctx is None
    node._on_tick()
    assert node._goal_end_code == 'REJECTED_BB(THROW_ABORTED_BY_HOST)'


def test_bb_throw_aborted_by_host_on_an_idle_node_is_ignored():
    """(a) Nothing running: a stray BY_HOST (a manual cancel, a calibration
    throw) touches nothing."""
    node, _client = _node_with_client(response=_response())
    cancel = _cancel_client(node)
    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_BY_HOST (axis=n/a, detail1=2940)'))
    assert node._goal_end_code == ''
    assert node._reload_ctx is None and node._executor is None
    assert cancel.calls == []


def test_bb_throw_aborted_by_host_before_this_attempt_threw_is_ignored(tmp_path):
    """(b) A reload still in `await_bb_idle` has asked Ball Butler for
    nothing yet -- a BY_HOST cannot be about its ball."""
    node, _client = _node_with_client(response=_response())
    _reload_armed_await_bb_idle(node, tmp_path)
    ctx = node._reload_ctx
    executor = node._executor
    cancel = _cancel_client(node)
    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_BY_HOST (axis=n/a, detail1=2940)'))
    assert node._reload_ctx is ctx and ctx.phase == 'await_bb_idle'
    assert node._executor is executor
    assert node._goal_end_code == ''
    assert cancel.calls == []


def test_bb_cancel_our_own_throw_aborted_by_host_after_the_end_is_ignored(tmp_path):
    """THROW_ABORTED_BY_HOST after the attempt ALREADY ended is the echo of
    OUR cancel: never a second end, never a relabel."""
    node, _client = _node_with_client(response=_response())
    cancel = _announced_reload_live(node, tmp_path)
    executor = node._executor
    node._stop_attempt()
    assert len(cancel.calls) == 1
    node._on_bb_throw_outcome(
        String(data='THROW_ABORTED_BY_HOST (axis=n/a, detail1=2940)'))
    assert executor.end_code == 'STOPPED'
    # Pre-announcement flavour: the end path cleared the ctx.
    node2, _c2 = _node_with_client(response=_response())
    resp = _start_reload(node2, tmp_path)
    assert resp.success is True, resp.message
    node2._end_attempt('ABORTED_TEST', 'ABORTED_TEST')
    node2._on_tick()
    node2._on_bb_throw_outcome(
        String(data='THROW_ABORTED_BY_HOST (axis=n/a, detail1=4200)'))
    assert node2._goal_end_code == 'ABORTED_TEST'


# ═════════════════════════════════════════════════════════════════════════════
# Unit C2 (2026-10-13): automatic lane-latch recovery triggers (bridge
# `recover_lane`). T1 = after the lane-refused END's hold; T2 = no executor and
# the latch bit up for 3 fresh samples. ONE call per latch episode.
# ═════════════════════════════════════════════════════════════════════════════

def _recover_client(node, *, ready=True, success=True,
                    message='lane recovery DONE'):
    resp = Trigger.Response()
    resp.success = success
    resp.message = message
    cli = _CancelClient(response=resp, ready=ready)
    node._lane_recovery_cli = cli
    return cli


def _lane_status(latched, recovery='idle', stage='', sched_refused=100):
    return DiagnosticStatus(values=[
        KeyValue(key='fault_state', value='NONE'),
        KeyValue(key='sched_refused', value=str(int(sched_refused))),
        KeyValue(key='sched_hold_latched', value=str(int(latched))),
        KeyValue(key='lane_recovery', value=recovery),
        KeyValue(key='lane_recovery_stage', value=stage)])


def test_t1_the_lane_refused_hold_then_requests_recovery_once():
    node, _client = _node_with_client()
    node._hold_cli = _hold_client()
    cli = _recover_client(node)
    node._force_hold = True
    node._maybe_hold_pending_event('LANE_REFUSED')
    assert len(node._hold_cli.calls) == 1
    assert len(cli.calls) == 1
    # a second forced end in the same latch episode does not call again
    node._force_hold = True
    node._maybe_hold_pending_event('LANE_REFUSED')
    assert len(cli.calls) == 1


def test_t1_without_the_service_falls_back_to_hold_only():
    node, _client = _node_with_client()
    node._hold_cli = _hold_client()
    cli = _recover_client(node, ready=False)
    node._force_hold = True
    node._maybe_hold_pending_event('LANE_REFUSED')
    assert len(node._hold_cli.calls) == 1                # today's behaviour
    assert cli.calls == []
    assert node._lane_cli_unavail_logged is True


def test_t1_a_plain_pending_event_hold_does_not_request_recovery():
    node, _client = _node_with_client()
    node._hold_cli = _hold_client()
    cli = _recover_client(node)
    node._pending_event_mono = time.perf_counter() + 5.0
    node._maybe_hold_pending_event('ABORTED_NO_RELEASE')
    assert cli.calls == []


def test_t2_fires_once_after_three_latched_samples_with_no_executor():
    node, _client = _node_with_client()
    cli = _recover_client(node)
    node._on_link_status(_lane_status(1))
    node._on_link_status(_lane_status(1))
    assert cli.calls == []                               # only 2 samples
    node._on_link_status(_lane_status(1))
    assert len(cli.calls) == 1                           # the third
    for _ in range(5):
        node._on_link_status(_lane_status(1))
    assert len(cli.calls) == 1                           # once per episode


def test_t2_does_not_fire_while_an_executor_is_running():
    node, _client = _node_with_client()
    cli = _recover_client(node)
    node._executor = MagicMock()
    for _ in range(5):
        node._on_link_status(_lane_status(1))
    assert cli.calls == []


def test_t2_does_not_refire_after_failed_until_the_bit_rises_again():
    node, _client = _node_with_client()
    cli = _recover_client(node, success=False, message='failed at stage arm')
    for _ in range(3):
        node._on_link_status(_lane_status(1))
    assert len(cli.calls) == 1
    for _ in range(10):                                  # still latched, failed
        node._on_link_status(_lane_status(1, recovery='failed'))
    assert len(cli.calls) == 1
    node._on_link_status(_lane_status(0, recovery='failed'))   # bit falls
    for _ in range(3):
        node._on_link_status(_lane_status(1))            # next rise
    assert len(cli.calls) == 2


def test_t2_a_done_callback_logs_one_line_per_outcome():
    node, _client = _node_with_client()
    _recover_client(node, success=False, message='failed at stage arm')
    errs = []
    node._logger.error = errs.append
    for _ in range(3):
        node._on_link_status(_lane_status(1))
    assert any('lane recovery' in m and 'FAILED' in m and 'DEACTIVATE' in m
               for m in errs)


def test_a_goal_is_refused_while_lane_recovery_is_in_progress(tmp_path):
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_lane_recovery_busy'
    node._on_link_status(_lane_status(1, recovery='in_progress', stage='rest'))
    resp = _try_self_toss_start(node, tmp_path)
    assert resp.success is False
    assert 'lane recovery in progress (stage rest)' in resp.message
    assert node._executor is None


def test_a_failed_recovery_names_the_deactivate_fallback(tmp_path):
    node, _client = _node_with_client(response=_response())
    node._params['plant_id'] = 'test_lane_recovery_failed'
    node._on_link_status(_lane_status(1, recovery='failed'))
    resp = _try_self_toss_start(node, tmp_path)
    assert resp.success is False
    assert 'LATCHED' in resp.message and 'DEACTIVATE' in resp.message


def test_the_hold_end_line_echoes_a_non_idle_lane_recovery():
    node, _client = _node_with_client()
    node._hold_cli = _hold_client()
    _recover_client(node)
    node._on_link_status(_lane_status(1, recovery='in_progress', stage='rest'))
    node._force_hold = True
    infos = []
    node._logger.info = infos.append
    node._maybe_hold_pending_event('LANE_REFUSED')
    assert any('hold installed' in m and 'lane_recovery=in_progress' in m
               for m in infos)


# ═════════════════════════════════════════════════════════════════════════════
# cup-possession precondition at goal ACCEPT (owner ask 2026-10-14; sitting 8
# attempt 4: a columns reload goal accepted with an EMPTY cup dispatched Ball
# Butler, and BB threw at an unattended platform 5.4 s later)
# ═════════════════════════════════════════════════════════════════════════════

def _cup(node, *, seated=True, valid=True):
    """A FRESH `/hand_telemetry` sample with the cup's DEBOUNCED (and raw)
    possession bit set -- the accept check reads the debounced one."""
    node._on_hand_telemetry(HandTelemetryMessage(
        pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
        ball_held=seated, ball_held_raw=seated, ball_held_valid=valid))


def _assert_no_bb_and_no_motion(node, reload_client, throw_client):
    assert reload_client.calls == []
    assert throw_client.calls == []
    assert node._reload_ctx is None
    assert node._executor is None


def test_accept_possession_reload_with_a_seated_cup_is_refused_ball_in_hand():
    """The owner's ask, verbatim case: a plain RELOAD with a ball already in
    the cup is refused at accept -- no `bb/reload`, no `bb/throw_at_target`,
    nothing installed."""
    node, _client = _node_with_client()
    reload_client, throw_client = _reload_ready(node)
    _cup(node, seated=True)
    resp = node._start_pattern(_goal('reload'))
    assert resp.success is False
    assert ex.REJECTED_BALL_IN_HAND in resp.message
    assert 'ball already in hand' in resp.message
    assert 'reload' in resp.message
    _assert_no_bb_and_no_motion(node, reload_client, throw_client)


def test_accept_possession_self_toss_reload_with_an_empty_cup_is_accepted(
        tmp_path):
    node, _client = _node_with_client()
    resp = _start_reload(node, tmp_path)        # the helper empties the cup
    assert resp.success is True, resp.message
    assert len(node._bb_throw_cli.calls) == 1


def test_accept_possession_columns_1ball_fed_with_a_seated_cup_is_refused():
    node, _client = _node_with_client()
    reload_client, throw_client = _reload_ready(node)
    _cup(node, seated=True)
    resp = node._start_pattern(_columns_1ball_fed_goal(separation_mm=100.0))
    assert resp.success is False
    assert ex.REJECTED_BALL_IN_HAND in resp.message
    _assert_no_bb_and_no_motion(node, reload_client, throw_client)


def test_accept_possession_self_toss_with_a_seated_cup_throws_first_whatever_the_flag(
        tmp_path):
    """Cup-keyed (2026-10-17): a SEATED cup makes self_toss throw-first --
    accepted, no Ball Butler dispatch, reload='0'. (Formerly an operator `reload` flag + SEATED
    was REJECTED_BALL_IN_HAND; the cup sensor decides now.)"""
    node, _client = _node_with_client()
    reload_client, throw_client = _reload_ready(node)
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=True)
    _prelevel_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', _good_box_path(tmp_path)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert (node._juggle_goal(_self_toss_goal())
                == sn.GoalResponse.ACCEPT)
    assert node._reload_ctx is None
    assert reload_client.calls == [] and throw_client.calls == []
    [(name, _msg, values)] = TestJuggleAction._attempt_events(node)
    assert name == sn.ATTEMPT_START and values['reload'] == '0'


@pytest.mark.parametrize('pattern', ['self_toss', 'hop', 'columns_1ball'])
def test_accept_possession_an_empty_cup_makes_a_throwing_pattern_reload_first(
        tmp_path, pattern):
    """Cup-keyed (2026-10-17): an EMPTY cup on self_toss / hop / columns_1ball
    is no longer refused -- the attempt opens with the Ball Butler reload
    catch: label ends ' (reload)', the start event says reload='1', and the
    BB path is armed."""
    node, _client = _node_with_client()
    reload_client, throw_client = _reload_ready(node)
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)
    _prelevel_ready(node)
    node._params['plant_id'] = 'test_empty_reload_first_%s_%s' % (pattern,
                                                                  id(node))
    separation_mm = 250.0 if pattern == 'hop' else 100.0
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, hop=True, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert (node._juggle_goal(_goal(pattern, separation_mm=separation_mm))
                == sn.GoalResponse.ACCEPT)
    assert node._attempt_label.endswith(' (reload)'), node._attempt_label
    [(name, _msg, values)] = TestJuggleAction._attempt_events(node)
    assert name == sn.ATTEMPT_START and values['reload'] == '1'
    assert node._reload_ctx is not None            # the BB path is armed


def test_accept_possession_magazine_with_an_empty_cup_is_refused():
    node, _client = _node_with_client()
    reload_client, throw_client = _reload_ready(node)
    _cup(node, seated=False)
    resp = node._start_pattern(_goal('magazine'))
    assert resp.success is False
    assert ex.REJECTED_NO_BALL in resp.message
    _assert_no_bb_and_no_motion(node, reload_client, throw_client)


def test_accept_columns_seated_feeds_b_from_bb_without_a_flag(tmp_path):
    """A SEATED cup is exactly what a fed columns attempt needs (A held, B fed
    after A's throw): accepted with the goal's reload False, Ball Butler
    dispatched once."""
    node, _client = _node_with_client()
    _cup(node, seated=True)
    resp = _columns_bb_started(node, tmp_path)
    assert resp.success is True
    assert len(node._bb_throw_cli.calls) == 1
    assert node._reload_ctx.kind == 'columns'


def test_accept_columns_empty_cup_opens_with_the_reload_of_a(tmp_path):
    """P3 (2026-10-17): columns + EMPTY is accepted -- the attempt opens with
    the Ball Butler reload of ball A at the columns FEED site (the shared
    tail's bridge site, not the legacy (-50, 0) reload site): label ends
    ' (reload)', the start event says reload='1', pre-level was reached, and
    the BB reload path is armed with kind 'reload' (the chain follows)."""
    node, _client = _node_with_client(response=_response())
    reload_client, throw_client = _reload_ready(node)
    prelevel = _prelevel_ready(node)
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)
    node._params['plant_id'] = 'test_columns_empty_a_%s' % id(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert (node._juggle_goal(_columns_goal(separation_mm=100.0))
                == sn.GoalResponse.ACCEPT)
    assert node._attempt_label.endswith(' (reload)'), node._attempt_label
    [(name, _msg, values)] = TestJuggleAction._attempt_events(node)
    assert name == sn.ATTEMPT_START and values['reload'] == '1'
    assert len(prelevel.calls) == 1                     # _prelevel reached
    ctx = node._reload_ctx
    assert ctx is not None and ctx.kind == 'reload'
    assert ctx.then_columns is not None
    feed_site = ctx.then_columns.site
    np.testing.assert_allclose(ctx.site.cup_mm, feed_site.cup_mm)
    # (the fixture's 100 mm layout puts the feed site P1 at x = -50, which
    # coincides with the legacy reload site; the identity above is the pin)
    assert len(reload_client.calls) == 1                # bb/reload asked
    assert node._executor is not None
    assert node._executor.schedule.skills[0].kind == 'REST'


_EVIDENCES = (ball_possession.EVIDENCE_EMPTY, ball_possession.EVIDENCE_SEATED,
              ball_possession.EVIDENCE_UNKNOWN)


def _expected_cup_decision(pattern, evidence):
    if evidence == ball_possession.EVIDENCE_UNKNOWN:
        return ex.REJECTED_BALL_UNKNOWN, False
    decision = sn._CUP_RULE[pattern][evidence]
    if decision == sn.RELOAD_FIRST:
        return None, True
    if decision == sn.THROW_FIRST:
        return None, False
    return decision, False


@pytest.mark.parametrize('evidence', _EVIDENCES)
@pytest.mark.parametrize('pattern', sorted(sn._CUP_RULE))
def test_accept_cup_decision_table(pattern, evidence):
    """`(pattern, cup evidence) -> (refusal code | None, reload)` for all 7
    patterns x {EMPTY, SEATED, UNKNOWN}, per plan `columns-empty-cup-two-feed`
    § 2 (columns EMPTY is reload-first since P3)."""
    assert sn.accept_cup_decision(pattern, evidence) == _expected_cup_decision(
        pattern, evidence)


def test_accept_cup_decision_table_pins_the_plan_cells():
    E, S = ball_possession.EVIDENCE_EMPTY, ball_possession.EVIDENCE_SEATED
    d = sn.accept_cup_decision
    for p in ('self_toss', 'hop', 'columns_1ball'):
        assert d(p, E) == (None, True) and d(p, S) == (None, False)
    assert d('columns', E) == (None, True)     # P3: reload A, then feed B
    assert d('columns', S) == (None, False)
    assert d('columns_1ball_fed', E) == (None, True)
    assert d('columns_1ball_fed', S) == (ex.REJECTED_BALL_IN_HAND, False)
    # the one-ball oval: hop's row and columns_1ball_fed's row
    assert d('oval_1ball', E) == (None, True)
    assert d('oval_1ball', S) == (None, False)
    assert d('oval_1ball_fed', E) == (None, True)
    assert d('oval_1ball_fed', S) == (ex.REJECTED_BALL_IN_HAND, False)
    assert d('reload', E) == (None, True)
    assert d('reload', S) == (ex.REJECTED_BALL_IN_HAND, False)
    assert d('magazine', E) == (ex.REJECTED_NO_BALL, False)
    assert d('magazine', S) == (None, False)


def test_accept_cup_decision_table_covers_every_pattern():
    assert set(sn._CUP_RULE) == set(sn.SkillNode._PATTERNS)
    for pattern in sn.SkillNode._PATTERNS:
        for evidence in _EVIDENCES:
            code, reload = sn.accept_cup_decision(pattern, evidence)
            assert code is None or code.startswith('REJECTED_')
            assert reload in (True, False)


@pytest.mark.parametrize('pattern,reload', [
    ('reload', False), ('columns', True), ('magazine', False)])
def test_accept_possession_a_blind_sensor_is_refused_ball_unknown(pattern,
                                                                   reload):
    node, _client = _node_with_client()
    reload_client, throw_client = _reload_ready(node)
    _cup(node, seated=True, valid=False)
    resp = node._start_pattern(_goal(pattern))
    assert resp.success is False
    assert ex.REJECTED_BALL_UNKNOWN in resp.message
    assert 'sensor' in resp.message
    _assert_no_bb_and_no_motion(node, reload_client, throw_client)


def test_a_slow_test_run_cannot_age_the_fixture_sample_past_the_accept_check():
    """The accept-time cup check ages the last /hand_telemetry sample on the
    hand clock against `_HAND_STATE_STALE_S`; on a loaded box the real clock
    aged the fixture's seated sample stale before `_start_pattern`, refusing
    REJECTED_BALL_UNKNOWN. probe 2026-10-15: failed before the clock
    injection, passes after.

    The frame check is the second sample family the fixture seeds (JB-64,
    2026-10-28): its Platform/commanded buffers were stamped on the real
    `time.perf_counter()` and judged against `_FRAME_CHECK_WINDOW_S` (1.0 s),
    so this test's own 0.6 s sleep left 0.4 s of margin and a loaded gate
    refused REJECTED_FRAME_OFFSET ('stale'). Reproduced 6/6 under eight
    busy-loop loaders before `_frame_clock` was injected."""
    node, _client = _node_with_client()
    time.sleep(max(sn._HAND_STATE_STALE_S, sn._FRAME_CHECK_WINDOW_S) + 0.1)
    resp = node._start_pattern(_flown_columns_goal())
    assert ex.REJECTED_BALL_UNKNOWN not in resp.message
    assert 'REJECTED_FRAME_OFFSET' not in resp.message
    assert node._executor is not None, resp.message


def test_accept_possession_stale_hand_telemetry_is_refused_ball_unknown():
    node, _client = _node_with_client()
    reload_client, throw_client = _reload_ready(node)
    _cup(node, seated=False)
    node._hand_telemetry_mono -= sn._HAND_STATE_STALE_S + 0.2
    resp = node._start_pattern(_goal('reload'))
    assert resp.success is False
    assert ex.REJECTED_BALL_UNKNOWN in resp.message
    assert 'stale' in resp.message and 'ms' in resp.message
    _assert_no_bb_and_no_motion(node, reload_client, throw_client)


def test_accept_possession_no_hand_telemetry_yet_is_refused_ball_unknown():
    node, _client = _node_with_client(hand=False)
    reload_client, throw_client = _reload_ready(node)
    resp = node._start_pattern(_goal('reload'))
    assert resp.success is False
    assert ex.REJECTED_BALL_UNKNOWN in resp.message
    assert 'no /hand_telemetry' in resp.message
    _assert_no_bb_and_no_motion(node, reload_client, throw_client)


def test_accept_possession_a_running_attempt_keeps_its_own_reason(tmp_path):
    """The check sits AFTER `_refuse_if_running`: a goal sent while an attempt
    runs is refused for THAT, not for the cup the running attempt holds."""
    node, _client = _node_with_client()
    _start_reload(node, tmp_path)
    _cup(node, seated=True)
    resp = node._start_pattern(_goal('reload'))
    assert resp.success is False
    assert ex.REJECTED_BALL_IN_HAND not in resp.message


def test_accept_possession_refusal_leaves_the_previous_result_untouched():
    """The check runs BEFORE the per-goal bookkeeping reset, so a goal
    refused for the cup never clears a previous goal's still-unread
    result."""
    node, _client = _node_with_client()
    _reload_ready(node)
    node._goal_outcome_lines = ['previous OUTCOME line']
    node._goal_reports = ['previous report']
    node._goal_end_code = 'PREVIOUS_END'
    node._attempt_label = 'previous'
    _cup(node, seated=True)
    resp = node._start_pattern(_goal('reload'))
    assert resp.success is False
    assert node._goal_outcome_lines == ['previous OUTCOME line']
    assert node._goal_reports == ['previous report']
    assert node._goal_end_code == 'PREVIOUS_END'
    assert node._attempt_label == 'previous'


@pytest.mark.parametrize('seated,valid,word', [
    (True, True, 'SEATED'), (False, True, 'EMPTY'), (True, False, 'UNKNOWN')])
def test_accept_possession_check_reports_the_cup_line(seated, valid, word):
    node, _client = _node_with_client()
    _cup(node, seated=seated, valid=valid)
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert ('cup: %s (age ' % (word,)) in resp.message


def test_accept_possession_check_reports_unknown_with_no_hand_telemetry():
    node, _client = _node_with_client(hand=False)
    resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert 'cup: UNKNOWN (no /hand_telemetry yet)' in resp.message


# ── The late catch re-aim BUMP: per-pattern switch + the service bumper (G2) ──

def test_the_catch_bump_is_on_for_a_columns_goal(tmp_path):
    from jugglebot.motion.skills import bump as sk_bump
    node, _client = _node_with_client()
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert node._start_pattern(
            _columns_goal(separation_mm=100.0)).success is True
    assert node._executor.catch_bump_enabled is True
    assert node._executor.bumper == node._bumper
    assert sk_bump.bump_enabled_for('columns_1ball_fed') is True


@pytest.mark.parametrize('goal_name', ['self_toss', 'hop', 'reload'])
def test_the_catch_bump_is_off_for_self_toss_hop_and_reload(tmp_path, goal_name):
    if goal_name == 'reload':
        node, _c = _node_with_client(response=_response(t_release_mono=50.0))
        node._on_traj_status(_status())
        _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
                 seated=False)
        _prelevel_ready(node)
        _reload_ready(node)
        goal = _self_toss_goal()
        box = _good_box_path(tmp_path)
    else:
        node, _c = _node_with_client(response=_response(t_release_mono=50.0))
        node._on_traj_status(_status())
        _prelevel_ready(node)
        goal = _self_toss_goal() if goal_name == 'self_toss' else _hop_goal()
        box = _good_box_path(tmp_path, hop=(goal_name == 'hop'))
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', box), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(goal)
    assert resp.success is True, resp.message
    assert node._executor.catch_bump_enabled is False


def test_the_bumper_sends_kind_bump_and_maps_the_reply():
    node, client = _node_with_client(response=_response(
        accepted=False, code='LIMIT_ACC', terminal=False, splice_k=42,
        release_offset_xy_mm=[1.5, -2.0], launch_bias_xy_mm_s=[3.0, 4.0]))
    res = node._bumper(1, 0, (20.0, -5.0), 101.25, 100.8)
    req = client.calls[-1]
    req = req[0] if isinstance(req, tuple) else req
    assert req.kind == InstallSegment.Request.KIND_BUMP
    assert req.t_event_s == 101.25 and req.t_dispatch_s == 100.8
    assert list(req.bump_delta_xy_mm) == [20.0, -5.0]
    assert (res.accepted, res.code, res.terminal, res.k_b) == (
        False, 'LIMIT_ACC', False, 42)
    assert np.allclose(res.release_offset_xy_mm, (1.5, -2.0))
    assert np.allclose(res.launch_bias_xy_mm_s, (3.0, 4.0))


# ═════════════════════════════════════════════════════════════════════════════
# The one-ball oval (plan two-ball-oval P1): accept path, workspace rule,
# dwell mismatch, skills/check, Stop mid-carry log
# ═════════════════════════════════════════════════════════════════════════════

def _oval_node(plant_tag):
    node, _client = _node_with_client()
    node._params['plant_id'] = 'test_oval_%s_%s' % (plant_tag, id(node))
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=True)
    _prelevel_ready(node)
    return node


def test_oval_1ball_seated_cup_accepts_and_compiles_fixed_roles(tmp_path):
    """SEATED -> throw first; the schedule is the fixed-role oval (box label
    oval_1ball, every THROW at P1 = T, every catch at P2 = C)."""
    node = _oval_node('accept')
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, oval=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_goal('oval_1ball', separation_mm=150.0))
    assert resp.success is True, resp.message
    sched = node._executor.schedule
    assert sched.pattern == 'oval_1ball'
    assert {sk.site.name for sk in sched.skills if sk.kind == 'THROW'} == {'P1'}
    assert {sk.site.name for sk in sched.skills if sk.kind == 'CATCH'} == {'P2'}


def test_oval_1ball_fed_empty_cup_reloads_at_T_and_seated_is_refused(tmp_path):
    node, _client = _node_with_client()
    reload_client, throw_client = _reload_ready(node)
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)
    _prelevel_ready(node)
    node._params['plant_id'] = 'test_oval_fed_%s' % id(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, oval=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_goal('oval_1ball_fed', separation_mm=150.0))
    assert resp.success is True, resp.message
    assert node._reload_ctx is not None
    assert node._reload_ctx.site.name == 'P1'          # BB feeds T = sites[0]
    node2, _c = _node_with_client()
    reload2, throw2 = _reload_ready(node2)
    _cup(node2, seated=True)
    resp = node2._start_pattern(_goal('oval_1ball_fed', separation_mm=150.0))
    assert resp.success is False and ex.REJECTED_BALL_IN_HAND in resp.message
    _assert_no_bb_and_no_motion(node2, reload2, throw2)


def test_oval_site_outside_workspace_is_refused_by_name_through_accept(tmp_path):
    """|site| 75 + half-extent 80 = 155 > 150 mm (no margin since the
    2026-10-18 amendment of decision 38): refused by the
    REAL accept path with the code and the numbers. (Fails before the check
    existed: the same goal was ACCEPTED -- recorded in report_node.md.)"""
    node = _oval_node('ws')
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, oval=True, oval_half_m=0.08)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_goal('oval_1ball', separation_mm=150.0))
    assert resp.success is False
    assert ex.REJECTED_SITE_OUTSIDE_WORKSPACE in resp.message
    assert '155.0' in resp.message and '150.0' in resp.message
    assert node._executor is None


def test_oval_at_exact_workspace_equality_is_accepted(tmp_path):
    node = _oval_node('ws_eq')
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, oval=True, oval_half_m=0.075)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_goal('oval_1ball', separation_mm=150.0))
    assert resp.success is True, resp.message


def test_accept_reports_a_missing_box_and_an_outside_site_together(tmp_path):
    """Every refusal at once: hop at 250 mm with the (P1,P2) box wide enough
    (30 mm half-extent) to leave the workspace and NO (P2,P1) box -> one message naming both."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    node._params['plant_id'] = 'test_both_%s' % id(node)
    box_path = _good_box_path(tmp_path, hop=True)
    import dataclasses
    # (P1,P2) widened to 30 mm half-extent: 125 + 30 = 155 > 150 (no margin
    # since 2026-10-18; the 20 mm fixture now sits inside the bound).
    boxes = [dataclasses.replace(b, landing_xy_m=((-0.03, 0.03), (-0.03, 0.03)))
             if b.site_pair == ('P1', 'P2') and b.pattern == 'hop' else b
             for b in adm.load(box_path) if b.site_pair != ('P2', 'P1')]
    adm.dump(box_path, boxes)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', box_path), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_hop_goal(separation_mm=250.0, apex_m=0.9))
    assert resp.success is False
    assert 'no admissible box covers' in resp.message
    assert ex.REJECTED_SITE_OUTSIDE_WORKSPACE in resp.message


def test_oval_refuses_a_box_swept_at_another_dwell_naming_dwell_s(tmp_path):
    node = _oval_node('dwell')
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, oval=True, oval_dwell_s=0.27)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        node._params['dwell_s'] = 0.65
        resp = node._start_pattern(_goal('oval_1ball', separation_mm=150.0))
    assert resp.success is False
    assert 'dwell_s' in resp.message
    assert '0.27' in resp.message and '0.65' in resp.message
    assert node._executor is None


def test_goal_dwell_wins_over_the_node_param_for_the_oval_selection(tmp_path):
    """2026-10-21 (owner): dwell_s travels on the goal. The node param is the
    columns 0.27, the oval box is stamped 0.65: a goal sending 0.65 selects it
    (the 22:53 sitting's refusal), and the resolved dwell is on the start event."""
    node = _oval_node('goal_dwell')
    assert node._params['dwell_s'] == pytest.approx(0.27)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, oval=True, oval_dwell_s=0.65)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(
            _goal('oval_1ball', separation_mm=150.0, dwell_s=0.65))
    assert resp.success is True, resp.message
    assert resp.attempt['dwell_s'] == pytest.approx(0.65)


def test_goal_dwell_zero_falls_back_to_the_node_param(tmp_path):
    node = _oval_node('goal_dwell0')
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, oval=True, oval_dwell_s=0.65)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        node._params['dwell_s'] = 0.27
        refused = node._start_pattern(_goal('oval_1ball', separation_mm=150.0))
        node._params['dwell_s'] = 0.65
        ok = node._start_pattern(_goal('oval_1ball', separation_mm=150.0,
                                       dwell_s=0.0))
    assert refused.success is False and 'dwell_s' in refused.message
    assert '0.27' in refused.message and '0.65' in refused.message
    assert 'on the goal' in refused.message      # points at the goal, not the param
    assert ok.success is True, ok.message
    assert ok.attempt['dwell_s'] == pytest.approx(0.65)


def test_check_prints_the_oval_box_line_and_the_workspace_result(tmp_path):
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _freshen(node)
    node._params['separation_mm'] = 150.0
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, hop=True, oval=True)):
        resp = node._svc_check(Trigger.Request(), Trigger.Response())
    [line] = [l for l in resp.message.split('; ') if l.startswith('oval box')]
    assert 'oval_1ball P1->P2 at 150.0 mm' in line
    assert 'apex 0.85-0.95 m' in line and 'dwell' in line
    assert 'oval workspace OK' in resp.message
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, hop=True, oval=True,
                                     oval_half_m=0.08)):
        resp = node._svc_check(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'oval box REFUSED: ' + ex.REJECTED_SITE_OUTSIDE_WORKSPACE in resp.message


def test_check_names_dwell_s_when_the_loaded_cell_dwell_differs(tmp_path):
    """Runsheet row 12a: the node dwell (0.65) differs from the box stamp
    (0.27) -> skills/check prints the oval line and box line from the NODE's
    dwell_s and carries describe_miss's dwell_s text."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _freshen(node)
    node._params['separation_mm'] = 150.0
    node._params['dwell_s'] = 0.65
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, hop=True, oval=True,
                                     oval_dwell_s=0.27)):
        resp = node._svc_check(Trigger.Request(), Trigger.Response())
    [line] = [l for l in resp.message.split('; ') if l.startswith('oval box')]
    assert 'dwell 0.650 s' in line
    assert 'dwell_s' in line and '0.27' in line and '0.65' in line


def test_hop_at_the_125_plus_20_operating_point_passes_the_workspace_rule(tmp_path):
    """Decision 38 amended 2026-10-18: 125 mm sites + 20 mm half-extent =
    145 <= 150 is accepted (the 20 mm margin refused it)."""
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    node._params['plant_id'] = 'test_hop125_%s' % id(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, hop=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_hop_goal(separation_mm=250.0, apex_m=0.9))
    assert ex.REJECTED_SITE_OUTSIDE_WORKSPACE not in resp.message, resp.message


def test_hop_at_135_plus_20_is_refused_by_the_workspace_rule(tmp_path):
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    _prelevel_ready(node)
    node._params['plant_id'] = 'test_hop135_%s' % id(node)
    # boxes swept at +-135 mm (the 270 mm separation), 20 mm half-extent
    box_path = _good_box_path(tmp_path, hop=True)
    import dataclasses
    moved = []
    for b in adm.load(box_path):
        if b.pattern == 'hop':
            sgn = 1.0 if b.release_site_xy_mm[0] > 0 else -1.0
            b = dataclasses.replace(b, release_site_xy_mm=(135.0 * sgn, 0.0),
                                    target_site_xy_mm=(-135.0 * sgn, 0.0))
        moved.append(b)
    adm.dump(box_path, moved)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', box_path), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_hop_goal(separation_mm=270.0, apex_m=0.9))
    assert resp.success is False
    assert ex.REJECTED_SITE_OUTSIDE_WORKSPACE in resp.message
    assert '155.0' in resp.message and '150.0' in resp.message


def test_stop_mid_carry_logs_why_only_the_after_rest_was_offered():
    node, _client = _node_with_client()
    rec = _rec_logger(node)
    fake = SimpleNamespace(
        stop_before_declined=ex.STOP_BEFORE_CARRIED,
        stop_terminals=lambda now: [],)
    assert node._stop_with_rest('STOPPED', fake) is False
    infos = [m for sev, m in rec.lines if sev == 'INFO']
    assert any(ex.STOP_BEFORE_CARRIED in m for m in infos)
    quiet = SimpleNamespace(stop_before_declined=None,
                            stop_terminals=lambda now: [])
    rec.lines.clear()
    node._stop_with_rest('STOPPED', quiet)
    assert not any(ex.STOP_BEFORE_CARRIED in m for _s, m in rec.lines)


# ═════════════════════════════════════════════════════════════════════════════
# P3 of plans/archived/columns-empty-cup-two-feed.md (2026-10-17): the columns
# two-feed chain. EMPTY cup -> Ball Butler reloads ball A at the feed site,
# then (gate: DECAY REST ended, cup SEATED, BB IDLE with a ball after its own
# refill) the SEATED columns tail feeds ball B. Mocked-ROS only: these pin
# the phases and the Stop/timeout races, blind to choreography (plan § 4 P3).
# ═════════════════════════════════════════════════════════════════════════════

def _chain_a_started(node, tmp_path, plan_capture=None):
    """EMPTY-cup columns goal accepted into the A reload and BB's throw for A
    fired: `_reload_ctx` is the kind-'reload' ctx in `await_announcement`."""
    node._params['plant_id'] = 'test_columns_chain_%s' % id(node)
    _freshen(node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV,
             seated=False)
    _prelevel_ready(node)
    _reload_ready(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    assert node._reload_ctx.kind == 'reload'
    assert node._reload_ctx.phase == 'await_bb_idle'
    _bb_ready(node)
    node._maybe_fire_reload_throw()
    assert node._reload_ctx.phase == 'await_announcement'
    return resp


def _announce_at(node, site, *, throw_time_s=2.0, landing_time_s=2.7):
    node._on_announcement(_bb_announcement(
        target_id=node._robot_name,
        landing_mm=(float(site.cup_mm[0]), float(site.cup_mm[1]),
                    sn.CATCH_CUP_Z_MM),
        landing_vel_mm_s=(1200.0, 0.0, -2500.0),
        throw_time_s=throw_time_s, landing_time_s=landing_time_s))


def _chain_a_swapped(node, tmp_path):
    """..., then A's bridge REST dispatched/retired and A's announcement
    swapped the reload schedule in: the chain ctx is armed."""
    _chain_a_started(node, tmp_path)
    _finish_bridge(node)
    assert node._executor is None
    _announce_at(node, node._reload_ctx.site)
    return node._reload_ctx


def _retire_a(node, end_code=''):
    """Retire A's live reload executor on the next tick with ``end_code``
    ('' = every skill dispatched cleanly). `done` is a property of the real
    executor; patched on the class for the one tick so the REAL executor
    object (the ctx's `a_executor` identity) is what retires."""
    from unittest.mock import PropertyMock
    execu = node._executor
    assert execu is not None
    execu.end_code = end_code
    execu.attempt_ended = bool(end_code)
    with patch.object(type(execu), 'done', new_callable=PropertyMock,
                      return_value=True):
        node._on_tick()
    assert node._executor is None


def _chain_at_gate(node, tmp_path):
    ctx = _chain_a_swapped(node, tmp_path)
    _retire_a(node)
    assert node._reload_ctx is ctx and ctx.phase == 'await_chain_gate'
    return ctx


def _open_chain_gate(node, ctx):
    _cup(node, seated=True)
    ctx.decay_end_abs = -1.0          # the mock ROS clock reads 0.0
    _bb_ready(node)                   # IDLE + ball_in_hand + connected


def test_columns_empty_cup_a_swap_arms_the_chain_ctx_not_none(tmp_path):
    node, _client = _node_with_client(response=_response())
    _chain_a_started(node, tmp_path)
    _finish_bridge(node)
    plan = node._reload_ctx.then_columns
    _announce_at(node, node._reload_ctx.site)
    ctx = node._reload_ctx
    assert ctx is not None
    assert ctx.kind == 'columns_chain' and ctx.phase == 'await_reload_a'
    assert ctx.then_columns is plan
    live = node._executor
    assert live is not None and ctx.a_executor is live
    last_rest = live.schedule.skills[-1]
    assert last_rest.kind == 'REST'
    assert ctx.decay_end_abs == pytest.approx(float(last_rest.t_abs_s))
    assert [s.kind for s in live.schedule.skills][:3] == ['REST', 'CATCH', 'REST']


def test_columns_chain_waits_for_decay_end_cup_seated_and_bb_idle_with_ball(
        tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    reload_client = node._reload_cli
    calls = []
    real = node._start_columns_feed
    node._start_columns_feed = lambda *a, **k: calls.append(1) or real(*a, **k)

    def shut(label):
        node._on_tick()
        assert node._reload_ctx is ctx and ctx.phase == 'await_chain_gate', label
        assert calls == [], label

    # everything else true, but the DECAY REST has not ended (now 0 < end)
    assert ctx.decay_end_abs > 0.0
    _cup(node, seated=True)
    _bb_ready(node)
    shut('decay not ended')
    ctx.decay_end_abs = -1.0
    _cup(node, seated=False)
    shut('cup EMPTY')
    _cup(node, seated=True)
    _bb_ready(node, state=sn.proto.BallButlerStates.CHECKING_BALL,
              ball_in_hand=True)
    shut('BB not IDLE')
    _bb_ready(node, ball_in_hand=False)
    shut('BB IDLE without a ball')
    _bb_ready(node, ball_in_hand=True, connected=False)
    shut('BB not connected')
    _bb_ready(node)
    node._on_tick()
    assert len(calls) == 1                       # all hold -> opens
    assert node._reload_ctx is not ctx           # B's await ctx replaced it
    assert node._reload_ctx.kind == 'columns'


def _assert_plans_equal(a, b):
    assert set(vars(a)) == set(vars(b))
    for k in vars(a):
        va, vb = getattr(a, k), getattr(b, k)
        if k == 'memory':
            assert type(va) is type(vb)          # a fresh learner memory each
            continue
        if isinstance(va, np.ndarray) or hasattr(va, 'cup_mm'):
            if hasattr(va, 'cup_mm'):
                np.testing.assert_allclose(va.cup_mm, vb.cup_mm, err_msg=k)
                assert va.name == vb.name, k
            else:
                np.testing.assert_allclose(va, vb, err_msg=k)
            continue
        try:
            same = bool(va == vb)
        except ValueError:      # dataclasses holding ndarrays (Site, patterns)
            same = repr(va) == repr(vb)
        assert same, k


def test_columns_chain_runs_the_seated_columns_tail(tmp_path):
    """The chain calls the SAME `_start_columns_feed` with a plan equal field
    by field to the one the SEATED path builds for the same goal."""
    seated_node, _c = _node_with_client(response=_response())
    seen_seated = []
    real = seated_node._start_columns_feed
    seated_node._start_columns_feed = (
        lambda plan, **k: seen_seated.append(plan) or real(plan, **k))
    seated_node._params['plant_id'] = 'test_columns_tail_seated'
    _freshen(seated_node, pos_meas=sn.REST_HAND_REV, pos_cmd=sn.REST_HAND_REV)
    _reload_ready(seated_node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        resp = seated_node._start_pattern(_columns_goal(separation_mm=100.0))
    assert resp.success is True, resp.message
    assert len(seen_seated) == 1

    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    seen = []
    real = node._start_columns_feed
    node._start_columns_feed = (
        lambda plan, **k: seen.append((plan, k)) or real(plan, **k))
    _open_chain_gate(node, ctx)
    node._on_tick()
    assert len(seen) == 1
    plan, kw = seen[0]
    assert plan is ctx.then_columns and kw == {'chain_ctx': ctx}
    assert plan.holds_ball is True
    _assert_plans_equal(plan, seen_seated[0])


def _chain_b_announced(node, tmp_path):
    ctx = _chain_at_gate(node, tmp_path)
    _open_chain_gate(node, ctx)
    node._on_tick()
    b_ctx = node._reload_ctx
    assert b_ctx.kind == 'columns' and b_ctx.phase == 'await_bb_idle'
    return ctx, b_ctx


def test_columns_chain_second_announcement_latches_ball_1_and_installs_compile_columns(
        tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx, b_ctx = _chain_b_announced(node, tmp_path)
    assert len(node._reload_cli.calls) == 1      # bb/reload only for A (B's
    #                                              hand already holds a ball)
    node._correlation.pop(0, None)               # A's flight is long landed
    node._maybe_fire_reload_throw()              # BB already IDLE + ball
    assert node._reload_ctx.phase == 'await_announcement'
    assert len(node._bb_throw_cli.calls) == 2    # A's throw + B's
    aim = b_ctx.aim_site.cup_mm
    node._on_announcement(_bb_announcement(
        target_id=node._robot_name,
        landing_mm=tuple(float(v) for v in aim),
        landing_vel_mm_s=(0.0, 0.0, -3000.0), throw_time_s=0.5,
        landing_time_s=5.0))
    assert node._reload_ctx is None
    assert node._executor is not None
    assert node._executor.schedule.pattern == 'columns'
    catches = [s for s in node._executor.schedule.skills if s.kind == sn.CATCH]
    first = min(catches, key=lambda s: s.t_abs_s)
    assert first.ball_id == 1
    assert first.landing_prior.t_land_abs_s == pytest.approx(5.0)
    assert 1 in node._correlation and len(node._correlation[1]) == 1
    assert 0 not in node._correlation


def test_columns_chain_a_missed_catch_ends_without_asking_bb_again(tmp_path):
    node, _client = _node_with_client(response=_response())
    _chain_a_swapped(node, tmp_path)
    _retire_a(node, end_code=ex.MISSED_CATCH)
    assert node._reload_ctx is None
    assert node._goal_end_code == ex.MISSED_CATCH
    # nothing further can start B: cup seated + BB ready changes nothing
    _cup(node, seated=True)
    _bb_ready(node)
    node._on_tick()
    node._maybe_advance_chain()
    assert node._executor is None and node._reload_ctx is None
    assert len(node._bb_throw_cli.calls) == 1
    assert len(node._reload_cli.calls) == 1


def test_columns_chain_an_empty_cup_at_the_gate_is_rejected_no_ball(tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    _cup(node, seated=False)
    ctx.deadline_mono = time.perf_counter() - 1.0
    node._on_tick()
    assert node._reload_ctx is None
    assert node._goal_end_code == ex.REJECTED_NO_BALL
    assert len(node._bb_throw_cli.calls) == 1


def test_columns_chain_gate_timeout_names_bb_state(tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    _cup(node, seated=True)
    _bb_ready(node, state=sn.proto.BallButlerStates.CHECKING_BALL,
              ball_in_hand=False)
    # the deadline armed at A's retire is COLUMNS_CHAIN_BB_REFILL_TIMEOUT_S out
    assert (ctx.deadline_mono - time.perf_counter()
            == pytest.approx(sn.COLUMNS_CHAIN_BB_REFILL_TIMEOUT_S, abs=2.0))
    node._on_tick()
    assert node._reload_ctx is ctx               # not yet
    ctx.deadline_mono = time.perf_counter() - 1.0
    node._on_tick()
    assert node._reload_ctx is None
    assert node._goal_end_code == (
        'ABORTED_BB_NOT_READY(CHECKING_BALL, ball_in_hand=False)')
    assert len(node._bb_throw_cli.calls) == 1


def test_columns_chain_gate_timeout_with_unknown_cup_is_ball_unknown(tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    _cup(node, seated=True, valid=False)
    ctx.deadline_mono = time.perf_counter() - 1.0
    node._on_tick()
    assert node._reload_ctx is None
    assert node._goal_end_code == ex.REJECTED_BALL_UNKNOWN


def test_columns_chain_stop_during_chaining_does_not_resurrect(tmp_path):
    """A Stop that lands between the gate marking the ctx 'chaining' (lock
    released) and `_start_columns_feed` returning: the bridge executor and
    B's ctx are DROPPED by `_chain_guarded`, never installed over the Stop."""
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    _open_chain_gate(node, ctx)
    real = node._start_columns_feed

    ran = []

    def stop_then_feed(plan, **kw):
        ran.append(1)
        assert ctx.phase == 'chaining'
        node._stop_attempt()
        return real(plan, **kw)

    node._start_columns_feed = stop_then_feed
    node._on_tick()
    assert ran == [1]
    assert node._executor is None
    assert node._reload_ctx is None
    assert len(node._bb_throw_cli.calls) == 1        # A's only
    node._on_tick()
    node._maybe_signal_goal_done()
    assert node._goal_done_event.is_set()
    assert node._executor is None and node._reload_ctx is None
    assert len(node._bb_throw_cli.calls) == 1


def test_columns_chain_stop_at_the_gate_never_asks_bb_for_b(tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    node._stop_attempt()
    assert node._reload_ctx is None
    _open_chain_gate(node, ctx)
    node._on_tick()
    node._maybe_advance_chain()
    assert node._executor is None and node._reload_ctx is None
    assert len(node._bb_throw_cli.calls) == 1
    assert len(node._reload_cli.calls) == 1
    node._maybe_signal_goal_done()
    assert node._goal_done_event.is_set()


def test_columns_chain_late_a_announcement_during_b_await_bb_idle_is_not_claimed(
        tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx, b_ctx = _chain_b_announced(node, tmp_path)
    assert b_ctx.phase == 'await_bb_idle'
    executor = node._executor
    _announce_at(node, b_ctx.site)                   # a late announcement
    assert node._reload_ctx is b_ctx
    assert b_ctx.phase == 'await_bb_idle'
    assert getattr(b_ctx, 'announced', False) is False
    assert node._executor is executor


def test_columns_chain_by_host_between_feeds_is_ignored(tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    for token in ('THROW_ABORTED_BY_HOST', 'THROW_ABORTED_NOT_SETTLED (axis=1)'):
        node._on_bb_throw_outcome(String(data=token))
    assert node._reload_ctx is ctx and ctx.phase == 'await_chain_gate'
    assert node._executor is None
    assert node._goal_end_code in ('', None)
    assert len(node._bb_throw_cli.calls) == 1


def test_columns_chain_a_foreign_executor_retiring_does_not_open_the_gate(
        tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_a_swapped(node, tmp_path)
    assert ctx.phase == 'await_reload_a'
    ctx.a_executor = object()
    _retire_a(node)
    assert node._reload_ctx is ctx
    assert ctx.phase == 'await_reload_a'


def test_columns_chain_await_reload_a_never_times_out(tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_a_swapped(node, tmp_path)
    assert ctx.phase == 'await_reload_a'
    far = time.perf_counter() + 10.0 * sn.COLUMNS_CHAIN_BB_REFILL_TIMEOUT_S
    with patch.object(sn.time, 'perf_counter', return_value=far):
        node._check_reload_timeout()
    assert node._reload_ctx is ctx and ctx.phase == 'await_reload_a'
    assert node._goal_end_code in (None, '')


def test_columns_chain_chaining_phase_is_skipped_by_the_timeout_check(
        tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    ctx.phase = 'chaining'
    ctx.deadline_mono = time.perf_counter() - 100.0
    node._check_reload_timeout()
    assert node._reload_ctx is ctx and ctx.phase == 'chaining'
    assert node._goal_end_code in (None, '')


def test_columns_chain_stop_between_the_two_guarded_steps_drops_the_arming(
        tmp_path):
    """Stop lands after the bridge executor is installed (guarded step 1) but
    before B's Ball Butler wait is armed (guarded step 2)."""
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    _open_chain_gate(node, ctx)
    real = node._bb_heartbeat_fresh
    fired = []

    def stop_once_bridge_installed(*a, **kw):
        if node._executor is not None and not fired:
            fired.append(1)
            node._stop_attempt()
        return real(*a, **kw)

    node._bb_heartbeat_fresh = stop_once_bridge_installed
    node._on_tick()
    assert fired == [1]
    assert node._reload_ctx is None
    execu = node._executor
    assert execu is None or getattr(execu, 'stopped', True) or execu.done
    assert len(node._bb_throw_cli.calls) == 1


def test_columns_chain_bridge_compile_refusal_ends_rejected_columns_unschedulable(
        tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    _open_chain_gate(node, ctx)

    def boom(*a, **kw):
        raise ValueError('forced compile refusal')

    with patch.object(sn, 'compile_reload_wait', boom):
        node._on_tick()
    assert node._reload_ctx is None
    assert node._goal_end_code == 'REJECTED_COLUMNS_UNSCHEDULABLE'


def test_columns_chain_feed_step_exception_ends_the_attempt_instead_of_hanging(
        tmp_path):
    node, _client = _node_with_client(response=_response())
    ctx = _chain_at_gate(node, tmp_path)
    _open_chain_gate(node, ctx)

    def boom(*a, **kw):
        raise RuntimeError('forced feed-step failure')

    node._start_columns_feed = boom
    node._on_tick()
    assert node._goal_end_code == 'ABORTED_RELOAD_ERROR(RuntimeError)'
    assert node._reload_ctx is None
    assert node._executor is None


# ═════════════════════════════════════════════════════════════════════════════
# D2 (2026-10-19): level the platform after an END that left it tilted. The
# executor owns the choreography (tests/motion/test_skills_executor.py); the
# shell owns the debug hook that rehearses it, the notices, the cancel
# receipt and the go_home supersede signal.
# ═════════════════════════════════════════════════════════════════════════════

def _catch_terminal_for_tests():
    from jugglebot.motion.skills.segments import CatchTerminal
    return CatchTerminal(landing_mm=[0.0, 0.0, 830.0],
                         landing_vel_mm_s=[0.0, 0.0, -2500.0],
                         t_land_s=100.0, rest_site_mm=[0.0, 0.0, 750.0])


def _set_debug_param(node, value):
    return node.set_parameters(
        [_MockParameter(value, name=sn._DEBUG_FORCE_CATCH_REFUSAL_PARAM)])[0]


def _logged(node):
    node._logger = MagicMock()
    node.get_logger = lambda: node._logger
    return node._logger


def test_the_debug_force_catch_refusal_defaults_off():
    node, client = _node_with_client(response=_response())
    assert node._params[sn._DEBUG_FORCE_CATCH_REFUSAL_PARAM] is False
    res = node._installer('CATCH', _catch_terminal_for_tests(), 99.0,
                          ball_id=0)
    assert res.accepted is True and len(client.calls) == 1


def test_the_debug_force_catch_refusal_fires_once_warns_and_resets():
    node, client = _node_with_client(response=_response())
    log = _logged(node)
    result = _set_debug_param(node, True)
    assert result.successful is True
    # A REST is not a CATCH: it goes to the wire and leaves the one-shot armed.
    from jugglebot.motion.skills.segments import RestTerminal
    rest = RestTerminal(rest_site_mm=[0.0, 0.0, 750.0], t_rest_s=101.0)
    assert node._installer('REST', rest, 99.0, ball_id=0).accepted is True
    assert len(client.calls) == 1
    # The first CATCH is refused on purpose, never reaching the wire.
    res = node._installer('CATCH', _catch_terminal_for_tests(), 99.0,
                          ball_id=0)
    assert res.accepted is False and res.code == 'CATCH_AXIS'
    assert 'DEBUG' in res.message and len(client.calls) == 1
    warn = [c[0][0] for c in log.warning.call_args_list]
    assert any('debug_force_catch_refusal_once FIRED' in w for w in warn)
    # Reset to off: the parameter reads False and the next CATCH is real.
    assert node._params[sn._DEBUG_FORCE_CATCH_REFUSAL_PARAM] is False
    res2 = node._installer('CATCH', _catch_terminal_for_tests(), 99.5,
                           ball_id=0)
    assert res2.accepted is True and len(client.calls) == 2
    assert sum('FIRED' in c[0][0] for c in log.warning.call_args_list) == 1


def test_the_debug_force_catch_refusal_is_refused_while_a_goal_is_active(
        tmp_path):
    node, _client = _node_with_client(response=_response())
    log = _logged(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert node._start_pattern(
            _columns_goal(separation_mm=100.0)).success is True
    assert node._executor is not None
    result = _set_debug_param(node, True)
    assert result.successful is False
    assert 'goal is active' in result.reason
    assert node._params[sn._DEBUG_FORCE_CATCH_REFUSAL_PARAM] is False
    assert node._debug_force_catch_refusal_armed is False
    assert any('REFUSED' in c[0][0] for c in log.warning.call_args_list)
    # The reload/feed wait counts as an active goal too.
    node._executor = None
    node._reload_ctx = object()
    assert _set_debug_param(node, True).successful is False
    # Disarming is always allowed.
    assert _set_debug_param(node, False).successful is True


def test_the_debug_force_catch_refusal_launch_value_arms_with_a_warning():
    """A value given at LAUNCH bypasses the set-callback (it only fires on a
    later `ros2 param set`), so the constructor arms it itself and warns."""
    orig = _MockNodeClass.declare_parameter

    def _launch_override(self, name, default_value):
        if name == sn._DEBUG_FORCE_CATCH_REFUSAL_PARAM:
            default_value = True
        return orig(self, name, default_value)

    with patch.object(_MockNodeClass, 'declare_parameter', _launch_override):
        node = sn.SkillNode()
    assert node._debug_force_catch_refusal_armed is True
    assert node._consume_debug_force_catch_refusal() is True
    assert node._params[sn._DEBUG_FORCE_CATCH_REFUSAL_PARAM] is False
    assert node._consume_debug_force_catch_refusal() is False


def test_executor_notices_are_logged_at_their_severity_once():
    node, _client = _node_with_client()
    log = _logged(node)
    executor = SimpleNamespace(notices=[('INFO', 'a'), ('WARN', 'b'),
                                        ('ERROR', 'c')])
    node._drain_notices(executor)
    assert executor.notices == []
    assert [c[0][0] for c in log.info.call_args_list] == ['a']
    assert [c[0][0] for c in log.warning.call_args_list] == ['b']
    assert [c[0][0] for c in log.error.call_args_list] == ['c']
    node._drain_notices(MagicMock())             # not a list: ignored


def test_a_cancel_receipt_with_the_release_outside_the_windup_levels_the_decay():
    node, _client = _node_with_client()
    _logged(node)
    node._executor = MagicMock()
    ok = SimpleNamespace(result=lambda: SimpleNamespace(
        success=True, message='cancel sent'))
    node._on_bb_cancel_done(ok, 'STOPPED', 2.0)
    node._executor.note_ball_cancelled.assert_called_once_with()
    # A release inside BB's wind-up may answer CANCEL_TOO_LATE: not trusted.
    node._executor = MagicMock()
    node._on_bb_cancel_done(ok, 'STOPPED', 0.3)
    node._executor.note_ball_cancelled.assert_not_called()
    # A failed cancel never levels.
    node._executor = MagicMock()
    bad = SimpleNamespace(result=lambda: SimpleNamespace(
        success=False, message='unavailable'))
    node._on_bb_cancel_done(bad, 'STOPPED', 2.0)
    node._executor.note_ball_cancelled.assert_not_called()


def test_the_cancel_trust_boundary_is_0_9_s():
    """The receipt is trusted only when the release was > the 0.6 s wind-up
    PLUS 0.3 s of latency away at the request (`_BB_CANCEL_TRUST_S`)."""
    node, _client = _node_with_client()
    _logged(node)
    ok = SimpleNamespace(result=lambda: SimpleNamespace(
        success=True, message='cancel sent'))
    node._executor = MagicMock()
    node._on_bb_cancel_done(ok, 'STOPPED', 0.65)
    node._executor.note_ball_cancelled.assert_not_called()
    node._executor = MagicMock()
    node._on_bb_cancel_done(ok, 'STOPPED', 0.95)
    node._executor.note_ball_cancelled.assert_called_once_with()


def test_a_stop_during_the_closing_wait_keeps_the_decay_and_says_so():
    node, _client = _node_with_client()
    _logged(node)
    node._executor = MagicMock()
    node._executor.attempt_ended = True
    node._executor.end_code = 'CATCH_AXIS'
    node._executor.closing_due_s.return_value = 1.5     # node clock reads 0.0
    resp = node._stop_attempt()
    assert resp.success is True
    assert node._executor.end_code == 'CATCH_AXIS'      # untouched
    assert 'decay to level is still due in 1.50 s' in resp.message
    assert 'a Stop keeps it' in resp.message


def _status_with_cycle(active):
    st = _status()
    st.cycle_active = bool(active)
    return st


def test_plan_superseded_reads_a_non_cycle_plan_only_after_the_install_settles():
    """go_home / go_to_pose / hold install a NON-cycle plan and leave the mode
    at TRAJECTORY; `cycle_active` False is the signal -- but only on a status
    received well AFTER this node's last accepted install."""
    node, _client = _node_with_client(response=_response())
    assert node._observations(0.0).plan_superseded is False
    node._on_traj_status(_status_with_cycle(False))
    assert node._observations(0.0).plan_superseded is False      # no install yet
    from jugglebot.motion.skills.segments import RestTerminal
    rest = RestTerminal(rest_site_mm=[0.0, 0.0, 750.0], t_rest_s=101.0)
    assert node._installer('REST', rest, 99.0, ball_id=0).accepted is True
    node._on_traj_status(_status_with_cycle(False))              # too soon
    assert node._observations(0.0).plan_superseded is False
    node._last_install_mono -= sn._SUPERSEDE_STATUS_SETTLE_S + 0.1
    assert node._observations(0.0).plan_superseded is True
    node._on_traj_status(_status_with_cycle(True))               # still ours
    assert node._observations(0.0).plan_superseded is False


# ═════════════════════════════════════════════════════════════════════════════
# DEBUG catch-time offset + the online cup-contact probe (2026-10-22)
# ═════════════════════════════════════════════════════════════════════════════

def test_the_catch_time_offset_defaults_to_zero_and_reads_in_range_values():
    node, _client = _node_with_client(response=_response())
    assert node._params[sn._CATCH_TIME_OFFSET_PARAM] == 0.0
    assert node._catch_time_offset_s == 0.0
    for ms in (20.0, -20.0, 40.0, -40.0):
        node._params[sn._CATCH_TIME_OFFSET_PARAM] = ms
        assert node._catch_time_offset_s == pytest.approx(ms / 1000.0)


@pytest.mark.parametrize('ms', [41.0, -40.5, 200.0, float('nan')])
def test_an_out_of_range_catch_time_offset_is_ignored_with_a_log(ms):
    node, _client = _node_with_client(response=_response())
    log = _logged(node)
    node._params[sn._CATCH_TIME_OFFSET_PARAM] = ms
    assert node._catch_time_offset_s == 0.0
    assert log.error.called and 'IGNORED' in log.error.call_args[0][0]


def test_the_contact_probe_finds_the_hand_velocity_kick_in_its_window():
    node, _client = _node_with_client(response=_response())
    # (t, valid, raw, held, vel_ff_cmd, vel_meas): descending cup (vff -1.5),
    # a kick (meas 10 rev/s below ff) from t = 100.03.
    node._hand_telemetry_hist.clear()
    for i in range(30):
        t = 100.0 + 0.01 * i
        vmeas = -1.5 - (12.0 if t >= 100.03 - 1e-9 else 0.5)
        node._hand_telemetry_hist.append((t, True, False, False, -1.5, vmeas))
    assert node._contact_time(100.0, 100.2) == pytest.approx(100.03)
    # before the kick: none; a window the history does not cover: None.
    assert node._contact_time(100.0, 100.02) is None
    assert node._contact_time(100.0, 105.0) is None


# ── planner_mode on the install wire (two-ball-oval P3, 2026-10-23) ────────
# ONE live switch (this node's `planner_mode` parameter) selects the box AND
# rides on every InstallSegment request; trajectory_node plans with it.

def _p3_terminals():
    from jugglebot.motion.skills.segments import (
        CatchTerminal, RestTerminal, ThrowTerminal)
    return [
        ('THROW', ThrowTerminal(site_mm=[0.0, 0.0, 860.0],
                                target_mm=[0.0, 0.0, 830.0], flight_s=0.6,
                                t_release_s=100.0)),
        ('CATCH', CatchTerminal(landing_mm=[0.0, 0.0, 830.0],
                                landing_vel_mm_s=[0.0, 0.0, -2500.0],
                                t_land_s=100.0, rest_site_mm=[0.0, 0.0, 750.0])),
        ('REST', RestTerminal(rest_site_mm=[0.0, 0.0, 750.0], t_rest_s=100.0)),
    ]


def test_every_install_request_carries_the_default_planner_mode():
    node, client = _node_with_client(response=_response())
    for kind, terminal in _p3_terminals():
        node._installer(kind, terminal, 100.0, ball_id=0)
    assert [r.planner_mode for r in client.calls] == ['level'] * 3


def test_every_install_request_carries_the_planner_mode_parameter():
    node, client = _node_with_client(response=_response())
    node._params['planner_mode'] = 'pose'
    for kind, terminal in _p3_terminals():
        node._installer(kind, terminal, 100.0, ball_id=0)
    assert [r.planner_mode for r in client.calls] == ['pose'] * 3


def test_planner_mode_cannot_change_while_a_goal_is_active(tmp_path):
    """Audit finding 2026-10-24: the executor's box selection is frozen at goal
    accept while every InstallSegment request re-reads the live parameter, so a
    mid-goal `ros2 param set /skill_node planner_mode pose` would solve the
    remaining installs under a planner the box was not selected under. Refused
    by name while a goal is active (the same guard as the debug param); the
    same value re-set is a no-op and allowed; between attempts it applies."""
    node, _client = _node_with_client(response=_response())
    log = _logged(node)
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH',
                      _good_box_path(tmp_path, columns=True)), \
         patch.object(sn, '_REPO_ROOT', str(tmp_path)):
        assert node._start_pattern(
            _columns_goal(separation_mm=100.0)).success is True
    assert node._executor is not None
    result = node.set_parameters([_MockParameter('pose', name='planner_mode')])[0]
    assert result.successful is False
    assert 'goal is active' in result.reason
    assert node._planner_mode() == 'level'
    assert any('REFUSED planner_mode' in c[0][0] for c in log.warning.call_args_list)
    # Re-setting the value already in force is not a change.
    assert node.set_parameters(
        [_MockParameter('level', name='planner_mode')])[0].successful is True
    # Between attempts the switch applies.
    node._executor = None
    node._reload_ctx = None
    assert node.set_parameters(
        [_MockParameter('pose', name='planner_mode')])[0].successful is True
    # ...and a typo is refused by name whatever the state.
    assert node.set_parameters(
        [_MockParameter('banana', name='planner_mode')])[0].successful is False


def _self_toss_box(planner_mode):
    return adm.AdmissibleBox(
        site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), apex_m=(0.6, 1.2),
        pattern='self_toss', release_site_xy_mm=(-50.0, 0.0),
        target_site_xy_mm=(-50.0, 0.0),
        limits={'leg_vel_mmps': _SESSION_LEG_VEL_MMPS, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 200000.0,
               'hand_acc_rps2': float(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)},
        gate_hash=adm.gate_hash(), swept_at='2026-10-23',
        dwell_s=sn._DEFAULT_DWELL_S, planner_mode=planner_mode)


def test_a_level_box_is_refused_by_name_under_the_pose_planner(tmp_path):
    box_path = tmp_path / 'admissible_box.yaml'
    adm.dump(str(box_path), [_self_toss_box('level')])
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    node._params['planner_mode'] = 'pose'
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(box_path)):
        resp = node._start_pattern(_self_toss_goal())
    assert resp.success is False
    assert 'planner_mode' in resp.message
    assert node._executor is None


def test_a_pose_box_passes_the_box_select_under_the_pose_planner(tmp_path):
    box_path = tmp_path / 'admissible_box.yaml'
    adm.dump(str(box_path), [_self_toss_box('pose')])
    node, _client = _node_with_client()
    node._on_traj_status(_status())
    node._params['planner_mode'] = 'pose'
    with patch.object(sn, '_ADMISSIBLE_BOX_PATH', str(box_path)):
        resp = node._start_pattern(_self_toss_goal())
    # Refused later for an unrelated reason (no live prelevel/commanded state
    # in this fixture) -- but NOT by the box select.
    assert 'no admissible box covers' not in resp.message
    assert 'planner_mode' not in resp.message
