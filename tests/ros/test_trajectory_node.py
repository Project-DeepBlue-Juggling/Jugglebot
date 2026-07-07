"""trajectory_node tests (Phase 1) — seeding, mode gating, services, cadence.

ROS 2 is mocked by ``tests/ros/conftest.py``. These tests drive the node's
callbacks and the emitter directly (``start_emitter=False``) for logic, and spin a
real emitter thread with an injected capturing PUB (no ZMQ bind) for the cadence
assertion. The emitter's frames are fed to a REAL ``SetpointPump`` to re-assert the
production-in-the-loop invariant at the node boundary.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, Pose, Quaternion
from jugglebot_interfaces.msg import RobotState, MotorStateSingle, TrajectoryStatus
from jugglebot_interfaces.srv import GoToPose, SetTrajectoryLimits

from jugglebot.trajectory_node import TrajectoryNode
from jugglebot.motion.trajectory import feasibility as feas
from controller.teensy_link.setpoint_pump import SetpointPump

_ACTIVATE_REV = list(hw.JB_OP_ACTIVATE_POSITION_REVS)


def _go_to_pose_req(x=0.0, y=0.0, z=170.0, duration_s=0.0):
    req = GoToPose.Request()
    req.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion())
    req.duration_s = duration_s
    return req


class _CapturePub:
    """Fake MpcCommandPub: records sent frames + send timestamps (no ZMQ)."""

    def __init__(self):
        self.frames = []
        self.times = []
        self.closed = False

    def send(self, msg):
        self.frames.append(msg)
        self.times.append(time.perf_counter())

    def close(self):
        self.closed = True


def _robot_state(pos_rev=None):
    pos_rev = pos_rev if pos_rev is not None else _ACTIVATE_REV
    rs = RobotState()
    rs.motor_states = [MotorStateSingle(pos_estimate=float(pos_rev[i]))
                       for i in range(6)] + [MotorStateSingle()]
    return rs


def _node(**kw):
    return TrajectoryNode(start_emitter=False, **kw)


# ── Construction defaults ─────────────────────────────────────

def test_starts_not_streaming_not_seeded():
    node = _node()
    assert node._streaming is False
    assert node._seeded is False
    assert node._active_plan is None


# ── Seeding + mode gating ─────────────────────────────────────

def test_mode_then_telemetry_seeds_hold_at_measured_pose():
    node = _node()
    node._on_control_mode(String(data='STANDBY'))   # streaming on, no telem yet
    assert node._streaming is True
    assert node._seeded is False                     # can't seed without telemetry
    node._on_robot_state(_robot_state())             # first telem → seed
    assert node._seeded is True
    assert node._active_plan is not None and node._active_plan.kind == 'hold'
    # Seeded pose ≈ neutral active pose (measured == activate revs).
    pose, _, _ = node._active_plan.state_at(0.0)
    assert np.allclose(pose[:3], [0.0, 0.0, 170.0], atol=1e-2)


def test_telemetry_before_mode_seeds_on_mode_entry():
    node = _node()
    node._on_robot_state(_robot_state())             # telem cached, not streaming
    assert node._seeded is False
    node._on_control_mode(String(data='STANDBY'))    # entry seeds from cached telem
    assert node._seeded is True and node._streaming is True


def test_leaving_stream_mode_stops_and_requires_reseed():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='STANDBY'))
    assert node._seeded and node._streaming
    node._on_control_mode(String(data='DISABLED'))   # non-streaming mode
    assert node._streaming is False
    assert node._seeded is False                      # forces re-seed on re-entry


def test_stale_telemetry_defers_seed_until_fresh_robot_state():
    """Mode entry with STALE telemetry must NOT seed/stream — a stale measured pose
    could place the first u0 outside the pump/firmware gates. The seed is deferred
    until the next (inherently fresh) robot_state arrives."""
    node = _node()
    # Cache a pose but stamp it older than robot_state_stale_s (stale).
    node._latest_pos_rev = _ACTIVATE_REV
    node._robot_state_mono = time.perf_counter() - (node._robot_state_stale_s + 5.0)
    node._on_control_mode(String(data='STANDBY'))
    assert node._streaming is True
    assert node._seeded is False               # stale ⇒ deferred, not seeded
    assert node._active_plan is None
    # A fresh robot_state arrives → seeds now.
    node._on_robot_state(_robot_state())
    assert node._seeded is True
    assert node._active_plan is not None and node._active_plan.kind == 'hold'


# ── Emitted frames are pump-acceptable at the node boundary ───

def test_emit_once_produces_pump_accepted_frames():
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    # Wire the pub manually (emitter thread not started).
    node._pub = pub
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='STANDBY'))
    pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
    t = time.perf_counter()
    for _ in range(50):
        node._emit_once(t)
        t += 0.025
    assert len(pub.frames) == 50
    for i, frame in enumerate(pub.frames):
        sp, reason = pump.build(frame, t_origin_us=i * 25000)
        assert reason is None, f"emitted frame {i} pump-rejected: {reason}"


def test_emit_once_noop_when_not_streaming():
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._emit_once(time.perf_counter())        # not seeded/streaming
    assert pub.frames == []


# ── Services (Trigger) ────────────────────────────────────────

def test_hold_rejected_when_not_seeded():
    node = _node()
    resp = node._svc_hold(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'seed' in resp.message.lower()


def test_go_home_rejected_when_not_seeded():
    node = _node()
    resp = node._svc_go_home(Trigger.Request(), Trigger.Response())
    assert resp.success is False


def test_go_home_from_neutral_is_accepted_noop():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='STANDBY'))
    resp = node._svc_go_home(Trigger.Request(), Trigger.Response())
    assert resp.success is True
    # Installed a return-to-neutral plan; ends at rest at neutral.
    pose, twist, _ = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(pose, [0, 0, 170, 0, 0, 0], atol=1e-2)
    assert np.allclose(twist, 0.0, atol=1e-9)


def test_hold_installs_holdplan_when_at_rest():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='STANDBY'))
    resp = node._svc_hold(Trigger.Request(), Trigger.Response())
    assert resp.success is True
    assert node._active_plan.kind == 'hold'


# ── go_to_pose (Phase 2) ──────────────────────────────────────

def _traj_mode_node():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='TRAJECTORY'))
    return node


def test_go_to_pose_accepts_feasible_move_in_trajectory_mode():
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is True
    assert resp.code == feas.OK
    assert resp.planned_duration_s == pytest.approx(2.0)
    # A move plan was installed (not a hold).
    assert node._active_plan.kind == 'move'
    pose, twist, _ = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(pose[:3], [0, 0, 190], atol=1e-2)
    assert np.allclose(twist, 0.0, atol=1e-9)


def test_go_to_pose_minimal_feasible_when_duration_zero():
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=0.0),
                                GoToPose.Response())
    assert resp.accepted is True
    assert resp.planned_duration_s > 0.0


def test_go_to_pose_rejected_wrong_mode():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='STANDBY'))   # streaming, but not TRAJECTORY
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is False
    assert resp.code == feas.WRONG_MODE
    assert 'TRAJECTORY' in resp.message
    # No move installed — the seeded hold stays put.
    assert node._active_plan.kind == 'hold'


def test_go_to_pose_rejected_when_not_seeded():
    node = _node()
    node._current_mode = 'TRAJECTORY'   # mode set but never seeded
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is False
    assert resp.code == feas.STALE_STATE


def test_go_to_pose_too_fast_rejected_with_min_duration():
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(_go_to_pose_req(x=20.0, y=20.0, z=185.0,
                                                duration_s=0.05),
                                GoToPose.Response())
    assert resp.accepted is False
    assert resp.code == feas.TOO_FAST
    assert resp.min_duration_s > 0.05
    assert node._active_plan.kind == 'hold'   # unchanged


def test_go_to_pose_out_of_workspace_rejected():
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(_go_to_pose_req(z=500.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is False
    assert resp.code == feas.WORKSPACE


# ── set_limits (Phase 2) ──────────────────────────────────────

def test_set_limits_updates_session_limits():
    node = _node()
    req = SetTrajectoryLimits.Request()
    req.leg_vel_limit_mmps = 150.0
    req.leg_acc_limit_mmps2 = 0.0        # keep current
    req.leg_jerk_limit_mmps3 = 0.0
    resp = node._svc_set_limits(req, SetTrajectoryLimits.Response())
    assert resp.success is True
    assert node._limits.leg_vel_mmps == pytest.approx(150.0)
    assert resp.applied_vel_limit_mmps == pytest.approx(150.0)
    # acc/jerk unchanged (0 => keep).
    assert node._limits.leg_acc_mmps2 == pytest.approx(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2)


def test_set_limits_clamps_to_ceiling():
    node = _node()
    req = SetTrajectoryLimits.Request()
    req.leg_vel_limit_mmps = 10_000.0    # absurd — must clamp to the YAML ceiling
    req.leg_acc_limit_mmps2 = 0.0
    req.leg_jerk_limit_mmps3 = 0.0
    resp = node._svc_set_limits(req, SetTrajectoryLimits.Response())
    assert resp.applied_vel_limit_mmps == pytest.approx(hw.JB_TRAJ_LEG_VEL_CEILING_MMPS)
    assert node._limits.leg_vel_mmps == pytest.approx(hw.JB_TRAJ_LEG_VEL_CEILING_MMPS)


def test_set_limits_then_move_uses_new_limit():
    node = _traj_mode_node()
    # Tighten the jerk ceiling hard, then a move must stretch longer to comply.
    req = SetTrajectoryLimits.Request()
    req.leg_vel_limit_mmps = 0.0
    req.leg_acc_limit_mmps2 = 0.0
    req.leg_jerk_limit_mmps3 = 2000.0
    node._svc_set_limits(req, SetTrajectoryLimits.Response())
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=0.0),
                                GoToPose.Response())
    assert resp.accepted is True
    rep = feas.validate(node._active_plan, node._limits, node._geom)
    assert rep.peak_leg_jerk_mmps3 <= 2000.0


# ── Status is the typed TrajectoryStatus ──────────────────────

def test_status_publishes_typed_trajectory_status():
    node = _traj_mode_node()
    # The publisher is a mock recording published messages.
    node._publish_status()
    published = node.status_pub.published[-1]
    assert isinstance(published, TrajectoryStatus)
    assert published.streaming is True
    assert published.mode == 'TRAJECTORY'
    assert published.plan_kind == 'hold'


# ── Emitter cadence (real thread, injected fake PUB) ──────────

def test_emitter_cadence_40hz():
    pub = _CapturePub()
    node = TrajectoryNode(start_emitter=True, command_pub_factory=lambda: pub)
    try:
        node._on_robot_state(_robot_state())
        node._on_control_mode(String(data='STANDBY'))
        time.sleep(1.0)
    finally:
        node.on_shutdown()
    n = len(pub.frames)
    # 40 Hz over ~1 s → allow generous scheduler slack on the non-RT Jetson.
    assert 34 <= n <= 46, f"expected ~40 frames, got {n}"
    # Max inter-frame gap well under the 250 ms staleness E-STOP (10× margin).
    gaps = np.diff(pub.times)
    if len(gaps):
        assert np.max(gaps) < 0.1, f"max emitter gap {np.max(gaps)*1e3:.1f} ms"
    assert pub.closed is True    # PUB closed on shutdown
