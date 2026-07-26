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
import types

from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from jugglebot_interfaces.msg import (
    DynamicTargetCommand,
    MotorStateSingle,
    PlatformPoseCommand,
    RobotState,
    TrajectoryStatus,
)
from jugglebot_interfaces.srv import GoToPose, SetTrajectoryLimits, TimedTarget

from jugglebot.trajectory_node import TrajectoryNode
from jugglebot.motion.trajectory import feasibility as feas
from jugglebot.motion.trajectory import planner
from jugglebot.motion.trajectory import HoldPlan
from jugglebot.motion.trajectory import TrajectoryInfeasible
from jugglebot.motion.trajectory.plan import TrajectoryPlan
from jugglebot.motion.trajectory.segment import QuinticSegment
from controller.teensy_link.setpoint_pump import SetpointPump

_ACTIVATE_REV = list(hw.JB_OP_ACTIVATE_POSITION_REVS)


def _go_to_pose_req(x=0.0, y=0.0, z=170.0, duration_s=0.0, lean_gain=0.0):
    req = GoToPose.Request()
    req.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion())
    req.duration_s = duration_s
    req.lean_gain = lean_gain
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


def _robot_state(pos_rev=None, is_homed=True):
    # is_homed=True by default: these tests model a homed, ACTIVE robot. The
    # seed gate (ARMING_CONTRACT A5) refuses to seed while is_homed is False —
    # covered explicitly by the seed-gate tests below.
    pos_rev = pos_rev if pos_rev is not None else _ACTIVATE_REV
    rs = RobotState()
    rs.motor_states = [MotorStateSingle(pos_estimate=float(pos_rev[i]))
                       for i in range(6)] + [MotorStateSingle()]
    rs.is_homed = bool(is_homed)
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


# ── go_to_pose audit fixes (2026-07-07) ───────────────────────

def test_go_to_pose_rejected_when_telemetry_stale():
    """Move planning needs a fresh seed: if robot_state has gone stale after
    seeding, reject STALE_STATE rather than plan on a possibly-mismatched pose."""
    node = _traj_mode_node()
    assert node._seeded and node._current_mode == 'TRAJECTORY'
    # Age the telemetry past the staleness window (keeps _seeded True).
    node._robot_state_mono = time.perf_counter() - (node._robot_state_stale_s + 5.0)
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is False
    assert resp.code == feas.STALE_STATE
    assert 'stale' in resp.message.lower()
    assert node._active_plan.kind == 'hold'          # seeded hold untouched


def test_go_to_pose_rejected_when_move_in_flight_busy():
    """Phase-2 BUSY restriction: a go_to_pose during an in-flight move is rejected
    (moves accepted only from a hold), and the active move is left untouched."""
    node = _traj_mode_node()
    first = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                 GoToPose.Response())
    assert first.accepted and node._active_plan.kind == 'move'
    # Pin the move as in-flight (well within its 2 s duration).
    node._plan_t0 = time.perf_counter() - 0.5
    in_flight_plan = node._active_plan
    resp = node._svc_go_to_pose(_go_to_pose_req(z=185.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is False
    assert resp.code == 'BUSY'
    assert 'in flight' in resp.message.lower()
    assert node._active_plan is in_flight_plan       # active move untouched


def test_go_to_pose_rejected_non_finite_duration():
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=float('nan')),
                                GoToPose.Response())
    assert resp.accepted is False
    assert resp.code == feas.TOO_FAST
    assert 'non-finite' in resp.message.lower()
    assert node._active_plan.kind == 'hold'


def test_go_to_pose_rejected_when_commanded_state_moved_during_planning(monkeypatch):
    """Install-continuity guard: build_move seeds at service entry but the gate
    takes ~1.5 s while the OLD plan streams. Simulate the commanded state drifting
    during planning (monkeypatch build_move to install a moved hold before
    returning) → STALE_STATE reject, and the drifted plan is NOT overwritten."""
    node = _traj_mode_node()
    real_build_move = planner.build_move

    def latency_build_move(state0, target, dur, limits, geom, *, shaper=None,
                           **kwargs):
        plan, report = real_build_move(state0, target, dur, limits, geom,
                                       shaper=shaper, **kwargs)
        # Simulate ~1.5 s of streaming: the commanded pose drifts 10 mm in z
        # (≈0.13 rev/leg, well past the ~0.06 rev continuity bound).
        moved = np.asarray(node._current_state()[0], dtype=float).copy()
        moved[2] += 10.0
        node._install(HoldPlan(moved))
        return plan, report

    monkeypatch.setattr(planner, 'build_move', latency_build_move)
    seeded_hold = node._active_plan                  # hold at the ~170 mm seed
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is False
    assert resp.code == feas.STALE_STATE
    assert 'moved during planning' in resp.message.lower()
    # The move (target z=190) was NOT installed — the drifted hold (seed z≈170 + 10)
    # the latency stub set is what remains active.
    assert node._active_plan is not seeded_hold
    assert node._active_plan.kind == 'hold'
    assert node._active_plan.final_pose[2] == pytest.approx(180.0, abs=1e-2)


def test_standby_exit_mid_move_installs_profiled_stop():
    """Leaving TRAJECTORY for STANDBY while a move is in flight installs a profiled
    C2 decel-to-rest (the move is silenced), and streaming continues."""
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted and node._active_plan.kind == 'move'
    move_target_z = float(node._active_plan.final_pose[2])
    assert move_target_z == pytest.approx(190.0, abs=1e-2)
    # Place us in the DECEL half of the move (nonzero leg velocity, but a stop from
    # here satisfies the gate — verified empirically 2026-07-07).
    node._plan_t0 = time.perf_counter() - 1.5
    _, mid_twist, _ = node._current_state()
    assert not np.allclose(mid_twist, 0.0)           # genuinely moving
    node._on_control_mode(String(data='STANDBY'))
    # Streaming continues, mode switched, and a profiled stop replaced the move.
    assert node._streaming is True and node._seeded is True
    assert node._current_mode == 'STANDBY'
    stop = node._active_plan
    p_end, v_end, a_end = stop.state_at(stop.total_duration)
    assert np.allclose(v_end, 0.0, atol=1e-9)        # decel-to-REST
    assert np.allclose(a_end, 0.0, atol=1e-9)
    assert p_end[2] < move_target_z - 1.0            # stopped short of the target


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


# ── Lean shaping (Phase 4) ────────────────────────────────────

def _diag_kv(node):
    """Publish status and return the trajectory/diagnostics KeyValues as a dict."""
    node._publish_status()
    diag = node.diagnostics_pub.published[-1]
    return {kv.key: kv.value for kv in diag.values}


def test_go_to_pose_explicit_zero_lean_gain_forces_lean_off():
    """An EXPLICIT lean_gain=0.0 forces lean OFF — an unshaped move installs.
    (Since 2026-07-17 the srv FIELD default is -1.0 = defer-to-config, so 0.0
    is the deliberate A/B-baseline arm, no longer the default-constructed value —
    the _go_to_pose_req helper passes 0.0 explicitly.)"""
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(_go_to_pose_req(x=25.0, z=185.0, duration_s=0.0),
                                GoToPose.Response())
    assert resp.accepted is True
    assert node._lean_gain_active == 0.0
    # Plain (unshaped) TrajectoryPlan, not a _ShapedPlan.
    from jugglebot.motion.trajectory.shaping import _ShapedPlan
    assert not isinstance(node._active_plan, _ShapedPlan)


def test_default_constructed_request_defers_to_shipped_lean_gain():
    """A default-constructed request (srv field default -1.0, mirrored by the
    conftest mock) defers to the SHIPPED config gain — lean is ON by default
    since 2026-07-17. This is the choreography-level tripwire: it fails if the
    conftest mock's default drifts from -1.0 OR the shipped config gain is
    turned off; the sibling shipped-defaults test regex-pins the srv file
    itself, so the pair jointly closes the drift loop in both directions."""
    node = _traj_mode_node()
    req = GoToPose.Request()
    req.pose = _go_to_pose_req(x=25.0, z=185.0).pose   # pose only; lean_gain untouched
    assert req.lean_gain == pytest.approx(-1.0)        # the mock mirrors the srv default
    resp = node._svc_go_to_pose(req, GoToPose.Response())
    assert resp.accepted is True
    assert node._lean_gain_active == pytest.approx(hw.JB_TRAJ_LEAN_GAIN)
    from jugglebot.motion.trajectory.shaping import _ShapedPlan
    assert isinstance(node._active_plan, _ShapedPlan)


def test_shipped_trajectory_defaults_are_the_s4_working_point():
    """Tripwire: the S4 working point persisted 2026-07-17 SHIPS — session limits
    (1000, 5000, 30000) and lean_gain 0.6 (bags 2026-07-16_21-58-59 / _22-06-30;
    see logbook 2026-07-17 S4-closure entry). Deliberately changing the working
    point means updating this test — that is the logged act. Also pins the
    GoToPose.srv FIELD default at -1.0 (defer-to-config): without it the YAML
    gain is unreachable by default-constructed requests (the 2026-07-17 lesson)."""
    assert hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS == pytest.approx(1000.0)
    assert hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2 == pytest.approx(5000.0)
    assert hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3 == pytest.approx(30000.0)
    assert hw.JB_TRAJ_LEAN_GAIN == pytest.approx(0.6)
    import re
    from pathlib import Path
    srv = (Path(__file__).resolve().parents[2]
           / 'ros_ws/src/jugglebot_interfaces/srv/GoToPose.srv').read_text()
    assert re.search(r'^float64\s+lean_gain\s+-1\.0\b', srv, re.M), (
        'GoToPose.srv lean_gain field default must be -1.0 (defer-to-config) — '
        'an undefaulted float64 reads 0.0 and silently forces lean OFF')


def test_shipped_retime_model_is_off_until_tracking_envelope_closed():
    """Tripwire: JB_TRAJ_RETIME_MODEL SHIPS FALSE (2026-07-17 evening, bag
    2026-07-17_19-32-03). The model's honest durations on lean traverses exceed
    the velocity loop's TRACKING envelope (x-traverse deviation 0.45 -> 0.73 rev,
    above the ~0.6 in-move ABORT line) — the legacy loop's overshoot was
    accidentally load-bearing there. Re-enable ONLY with accel FF landed (see
    plans/active/accel-ff-inertia.md Verification) or a tracking-aware duration
    floor, and re-run the 19-32-03 A/B as the acceptance test — see
    logbook/2026-07-17-retime-model-tracking-envelope.md. Deliberately flipping
    the default means updating this test; that is the logged act."""
    assert hw.JB_TRAJ_RETIME_MODEL is False


def test_go_to_pose_per_call_lean_override_installs_shaped_plan():
    """lean_gain=0.3 overrides the config default and installs a shaped move."""
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(
        _go_to_pose_req(x=25.0, y=-15.0, z=185.0, duration_s=0.0, lean_gain=0.3),
        GoToPose.Response())
    assert resp.accepted is True
    assert node._lean_gain_active == pytest.approx(0.3)
    from jugglebot.motion.trajectory.shaping import _ShapedPlan
    assert isinstance(node._active_plan, _ShapedPlan)
    assert 'lean_gain=0.30' in resp.message


def test_go_to_pose_negative_lean_gain_uses_config_default():
    """A negative override defers to JB_TRAJ_LEAN_GAIN (ships 0.6 since 2026-07-17;
    this test pins the defer MECHANISM with a simulated config value)."""
    node = _traj_mode_node()
    node._config_lean_gain = 0.25            # simulate a ramped config default
    resp = node._svc_go_to_pose(
        _go_to_pose_req(x=25.0, z=185.0, duration_s=0.0, lean_gain=-1.0),
        GoToPose.Response())
    assert resp.accepted is True
    assert node._lean_gain_active == pytest.approx(0.25)


def test_lean_gain_over_one_is_clamped():
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(
        _go_to_pose_req(x=10.0, z=180.0, duration_s=0.0, lean_gain=5.0),
        GoToPose.Response())
    assert resp.accepted is True
    assert node._lean_gain_active == pytest.approx(1.0)


def test_diagnostics_publishes_realized_peaks_limits_and_lean_gain():
    node = _traj_mode_node()
    node._pub = _CapturePub()
    node._svc_go_to_pose(_go_to_pose_req(x=20.0, z=185.0, duration_s=1.0),
                         GoToPose.Response())
    # Emit a few frames so realized peaks accumulate off the active move.
    t0 = node._plan_t0
    for k in range(6):
        node._emit_once(t0 + k * 0.025)
    kv = _diag_kv(node)
    for key in ('peak_leg_vel_mmps', 'realized_peak_leg_vel_mmps',
                'realized_peak_leg_acc_mmps2', 'realized_peak_leg_jerk_mmps3',
                'limit_leg_vel_mmps', 'limit_leg_acc_mmps2', 'limit_leg_jerk_mmps3',
                'lean_gain', 'move_seq'):
        assert key in kv, f"missing diagnostics key {key}"
    # Limits reflect the node's current session limits.
    assert float(kv['limit_leg_vel_mmps']) == pytest.approx(node._limits.leg_vel_mmps)
    # Realized vel peak is nonzero mid-move (the platform is translating).
    assert float(kv['realized_peak_leg_vel_mmps']) > 0.0


def test_realized_peaks_reset_on_new_install():
    node = _traj_mode_node()
    node._pub = _CapturePub()
    node._svc_go_to_pose(_go_to_pose_req(x=20.0, z=185.0, duration_s=1.0),
                         GoToPose.Response())
    for k in range(6):
        node._emit_once(node._plan_t0 + k * 0.025)
    assert node._run_peak_vel_mmps > 0.0
    # A fresh install (e.g. a hold) resets the realized window.
    node._install(node._active_plan)     # re-install clears the peaks
    assert node._run_peak_vel_mmps == 0.0
    assert node._prev_frame_leg_acc is None


def test_move_seq_increments_per_accepted_move_for_diagnose_segmentation():
    """Each accepted go_to_pose bumps move_seq (published on diagnostics) so the
    /diagnose summariser can split back-to-back moves — plan_kind stays 'move'
    across a completed move's terminal hold, so move_seq is the only boundary."""
    node = _traj_mode_node()
    node._pub = _CapturePub()
    assert node._move_seq == 0
    node._svc_go_to_pose(_go_to_pose_req(x=20.0, z=185.0, duration_s=1.0),
                         GoToPose.Response())
    assert node._move_seq == 1
    assert _diag_kv(node)['move_seq'] == '1'
    # Let the first move finish so the BUSY guard admits the next one (the battery
    # fires moves back-to-back from the settled hold, never issuing trajectory/hold).
    node._plan_t0 = time.perf_counter() - 5.0
    # A second move (no intervening hold) increments again — the battery case.
    resp2 = node._svc_go_to_pose(_go_to_pose_req(x=-20.0, z=185.0, duration_s=1.0),
                                 GoToPose.Response())
    assert resp2.accepted is True
    assert node._move_seq == 2
    assert _diag_kv(node)['move_seq'] == '2'


# ── Audit fixes (2026-07-08): move_seq per non-follower install ────────────────

def test_go_home_bumps_move_seq_into_its_own_diagnose_row():
    """A go_home installs a kind=='move' plan that RESETS the realized peaks under
    the same plan_kind — so without a move_seq bump the /diagnose summariser's
    last-sample-wins would report go_home's near-zero peaks as the preceding move's.
    The bump makes go_home its own row, preserving the move's true realized peaks."""
    node = _traj_mode_node()
    node._pub = _CapturePub()
    node._svc_go_to_pose(_go_to_pose_req(x=20.0, z=185.0, duration_s=1.0),
                         GoToPose.Response())
    move_seq = node._move_seq
    for k in range(8):
        node._emit_once(node._plan_t0 + k * 0.025)
    move_realized_vel = node._run_peak_vel_mmps
    assert move_realized_vel > 0.0
    # Finish the move, then go_home (a non-follower install).
    node._plan_t0 = time.perf_counter() - 5.0
    assert node._svc_go_home(Trigger.Request(), Trigger.Response()).success is True
    assert node._move_seq == move_seq + 1        # go_home is its OWN summariser row
    assert node._run_peak_vel_mmps == 0.0        # its install reset the realized window


def test_follower_installs_do_not_bump_move_seq():
    """A SpaceMouse stream's per-tick follow installs must NOT bump move_seq — the
    whole stream is ONE /diagnose window. Only non-follower installs start a new row.
    """
    node = _follower_node()
    seq0 = node._move_seq                         # seed hold does not bump
    t = time.perf_counter()
    for k in range(5):
        node._on_platform_pose(_platform_pose(x=5.0 + k, z=176.0,
                                              publisher='SPACEMOUSE'))
        tgt, _ = node._follower_target
        node._follower_target = (tgt, t)          # fresh on the synthetic clock
        assert node._follower_tick(t) is True
        t += 0.025
    assert node._move_seq == seq0                 # no bump across the follow stream


def test_track_realized_peaks_skips_frame_from_superseded_plan():
    """The emit thread can sample a frame from the OLD plan, then a service-thread
    install resets the accumulators before _track_realized_peaks runs. The stale
    frame must NOT contaminate the new move's realized window — the tracker skips
    accumulation when its sampled plan is no longer the active plan."""
    node = _traj_mode_node()
    node._pub = _CapturePub()
    node._svc_go_to_pose(_go_to_pose_req(x=20.0, z=185.0, duration_s=1.0),
                         GoToPose.Response())
    old_plan = node._active_plan
    node._emit_once(node._plan_t0 + 0.1)          # a real mid-move frame accumulates
    frame_old = node._pub.frames[-1]
    assert node._run_peak_vel_mmps > 0.0
    # An install swaps the active plan and resets the realized window.
    node._install(HoldPlan(old_plan.state_at(0.1)[0]))
    assert node._run_peak_vel_mmps == 0.0
    # The late OLD-plan frame is ignored (active_plan changed since it was sampled).
    node._track_realized_peaks(frame_old, old_plan)
    assert node._run_peak_vel_mmps == 0.0
    assert node._prev_frame_leg_acc is None
    # A frame tagged with the CURRENT active plan IS accumulated (the identity gate
    # admits it).
    node._track_realized_peaks(frame_old, node._active_plan)
    assert node._run_peak_vel_mmps > 0.0


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


# ── C-LEVEL-1 observability: gravity_correction_loaded ────────
# The correction is per-process, in-memory, delivered by a VOLATILE topic with
# one latched push per orchestrator boot and no re-request path — so a relaunch
# empties it while the Teensy-persisted RobotState.levelling_complete still reads
# True. This flag is the only observation of the node that APPLIES it, and the
# toss's REJECTED_NOT_LEVELLED gate consumes it.

def test_status_reports_no_correction_until_an_offset_arrives():
    node = _traj_mode_node()
    node._publish_status()
    assert node.status_pub.published[-1].gravity_correction_loaded is False
    node._on_gravity_offset(Float64MultiArray(data=[0.013592347421588673,
                                                   0.001207157476773584]))
    node._publish_status()
    assert node.status_pub.published[-1].gravity_correction_loaded is True


def test_zero_offset_counts_as_loaded_even_though_the_correction_is_identity():
    """A genuinely level machine measures a ZERO tilt, whose correction IS the
    identity. Reading "loaded" off `R != I` would refuse that machine forever —
    the flag records that a measurement was taken and pushed, not that it was
    large. This is the case a naive non-identity check gets wrong."""
    node = _traj_mode_node()
    node._on_gravity_offset(Float64MultiArray(data=[0.0, 0.0]))
    node._publish_status()
    assert node.status_pub.published[-1].gravity_correction_loaded is True
    assert np.allclose(node._gravity_correction, np.eye(3))


@pytest.mark.parametrize('data', [
    [0.05],                              # short — cannot be decoded at all
    [float('nan'), 0.0],                 # NaN tilt_x
    [0.0, float('nan')],                 # NaN tilt_y
    [float('inf'), 0.0],                 # inf tilt_x
])
def test_malformed_offset_does_not_count_as_loaded(data):
    """A message the node could not decode did not tell it where level is; the
    stored correction is untouched, so the flag must stay False rather than
    affirm a frame that was never set.

    The NON-FINITE rows matter as much as the short one and are easy to miss:
    ``levelling.correction_from_offset`` does no finiteness validation, so a
    NaN offset would be negated straight into the stored rotation. The flag
    would then read True, the toss would pass CHECKING, and the NaN would only
    surface downstream as a POSITIONING feasibility rejection — AFTER the goal
    has claimed the platform. The contract wants that failure loud and early,
    as REJECTED_NOT_LEVELLED."""
    node = _traj_mode_node()
    node._on_gravity_offset(Float64MultiArray(data=data))
    node._publish_status()
    assert node.status_pub.published[-1].gravity_correction_loaded is False
    assert np.all(np.isfinite(node._gravity_correction))
    assert np.allclose(node._gravity_correction, np.eye(3))


def test_a_restarted_node_reports_no_correction_although_the_teensy_flag_persists():
    """THE RESTART CASE. Node A holds a correction; the `colcon build` +
    relaunch this contract's own deployment mandates replaces it with node B.
    Nothing republishes (/gravity_offset is VOLATILE, its startup push is
    latched per orchestrator boot) and RobotState.levelling_complete is
    unaffected — it lives on the Teensy. B must therefore report False: the gate
    that consumes this has to refuse the state, not inherit A's answer."""
    node_a = _traj_mode_node()
    node_a._on_gravity_offset(Float64MultiArray(data=[0.05, -0.03]))
    node_a._publish_status()
    assert node_a.status_pub.published[-1].gravity_correction_loaded is True

    node_b = _traj_mode_node()                     # the post-relaunch process
    node_b._on_robot_state(_robot_state())         # levelling_complete lives here,
    node_b._publish_status()                       #   and says nothing about B
    assert node_b.status_pub.published[-1].gravity_correction_loaded is False
    assert np.allclose(node_b._gravity_correction, np.eye(3))


# ── leg_torques_diagnostic (gravity FF observability) ─────────
# trajectory_node republishes each emitted frame's torque_Nm on the SAME topic
# (leg_torques_diagnostic, Float64MultiArray, 6 leg values, TRUE Nm, extension-
# positive) that motion_bridge_node uses on the MPC path — so the rosbag record
# list / GUI / consumers see identical data regardless of producer. The 40 Hz
# emitter only STASHES the vector; _publish_status (5 Hz) does the ROS publish.

def test_publish_status_publishes_leg_torques_diagnostic():
    """After a seed + at least one emitted frame, _publish_status publishes the
    latest frame's torque_Nm on leg_torques_diagnostic. With the shipped gravity
    FF enabled the six values are nonzero and physically small at the active pose."""
    node = _traj_mode_node()
    node._pub = _CapturePub()
    node._emit_once(node._plan_t0)                    # stashes the frame's torque_Nm
    assert node._last_torque_Nm is not None
    node._publish_status()
    pub = node.leg_torques_pub
    assert len(pub.published) == 1
    msg = pub.published[-1]
    assert isinstance(msg, Float64MultiArray)
    assert len(msg.data) == 6
    # Same topic AND same data as the last emitted frame (produced-agnostic parity).
    assert np.allclose(msg.data, node._last_torque_Nm)
    assert np.allclose(msg.data, node._pub.frames[-1]['torque_Nm'])
    # Shipped gravity FF: nonzero, physically small at the active pose. NOTE the
    # empirically-measured band at [0,0,170] is 0.01253-0.03941 Nm (asymmetric —
    # the six legs do NOT share the vertical gravity load equally at home); the
    # bounds bracket that with margin. (Slightly wider low end than the MPC-path
    # band documented in tests/motion/test_leg_torque_ff.py (0.013-0.041) — leg 3's
    # analytic value at [0,0,170] is 0.01253, just under 0.013.)
    assert all(v != 0.0 for v in msg.data)
    assert np.all(np.abs(msg.data) >= 0.012)
    assert np.all(np.abs(msg.data) <= 0.045)


def test_publish_status_no_torques_when_not_streaming():
    """Never seeded / not streaming ⇒ _publish_status publishes nothing on
    leg_torques_diagnostic (there is no stream to sample)."""
    node = _node()
    assert node._streaming is False and node._seeded is False
    node._publish_status()
    assert node.leg_torques_pub.published == []


def test_publish_status_no_torques_before_first_frame():
    """Seeded + streaming but no emitter frame yet ⇒ nothing published: the stash
    is None until frame 1, so no torque leaks before the wire carries one."""
    node = _traj_mode_node()                          # seeded + streaming
    assert node._last_torque_Nm is None               # no _emit_once yet
    node._publish_status()
    assert node.leg_torques_pub.published == []


def test_publish_status_stops_torques_after_leaving_stream():
    """Once streaming stops, nothing republishes: the streaming gate closes AND
    the stash itself is dropped (audit 2026-07-16 — a surviving stash could leak
    the previous stream's torque in the re-entry window, see the next test)."""
    node = _traj_mode_node()
    node._pub = _CapturePub()
    node._emit_once(node._plan_t0)
    node._publish_status()
    assert len(node.leg_torques_pub.published) == 1
    node._on_control_mode(String(data='DISABLED'))    # leave the streaming set
    assert node._last_torque_Nm is None               # stash dropped with the stream
    node._publish_status()
    assert len(node.leg_torques_pub.published) == 1    # no new torque message


def test_no_stale_torque_republish_in_stream_reentry_window():
    """Audit 2026-07-16: leave the stream, re-enter and RESEED, but no new frame
    has shipped yet — a 5 Hz status tick in that window must NOT republish the
    previous stream's last torque vector."""
    node = _traj_mode_node()
    node._pub = _CapturePub()
    node._emit_once(node._plan_t0)
    node._publish_status()
    assert len(node.leg_torques_pub.published) == 1
    node._on_control_mode(String(data='DISABLED'))    # leave the streaming set
    node._on_robot_state(_robot_state())              # fresh telemetry
    node._on_control_mode(String(data='TRAJECTORY'))  # re-enter: reseeds
    assert node._streaming is True and node._seeded is True
    node._publish_status()                            # tick BEFORE the first new frame
    assert len(node.leg_torques_pub.published) == 1    # nothing stale republished
    node._emit_once(node._plan_t0)                    # first frame of the new stream
    node._publish_status()
    assert len(node.leg_torques_pub.published) == 2    # now it flows again


def test_emitter_hot_path_only_stashes_does_not_publish_torques():
    """The 40 Hz emitter must add no publisher call to the hot path — it only
    STASHES the torque vector; the ROS publish is the 5 Hz _publish_status's job."""
    node = _traj_mode_node()
    node._pub = _CapturePub()
    for k in range(5):
        node._emit_once(node._plan_t0 + k * 0.025)
    assert node._last_torque_Nm is not None            # stash updated by the emitter
    assert node.leg_torques_pub.published == []         # but emitter published nothing


# ── SpaceMouse follower (Phase 3) ─────────────────────────────

def _platform_pose(x=0.0, y=0.0, z=170.0, publisher='SPACEMOUSE'):
    msg = PlatformPoseCommand()
    msg.pose_stamped = PoseStamped()
    msg.pose_stamped.pose = Pose(position=Point(x=x, y=y, z=z),
                                 orientation=Quaternion())
    msg.publisher = publisher
    return msg


def _follower_node():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='SPACEMOUSE'))
    return node


def test_platform_pose_ignored_outside_follower_mode():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='STANDBY'))    # not a follower mode
    node._on_platform_pose(_platform_pose(x=10.0, publisher='SPACEMOUSE'))
    assert node._follower_target is None             # target not stored


def test_platform_pose_ignored_when_publisher_mismatches_mode():
    node = _follower_node()                           # SPACEMOUSE mode
    node._on_platform_pose(_platform_pose(x=10.0, publisher='GUI'))
    assert node._follower_target is None              # publisher != active mode


def test_platform_pose_stored_in_follower_mode():
    node = _follower_node()
    node._on_platform_pose(_platform_pose(x=12.0, y=5.0, z=178.0,
                                          publisher='SPACEMOUSE'))
    assert node._follower_target is not None
    target, _mono = node._follower_target
    assert np.allclose(target[:3], [12.0, 5.0, 178.0], atol=1e-6)


def test_follower_tick_installs_move_toward_target():
    node = _follower_node()
    assert node._active_plan.kind == 'hold'
    node._on_platform_pose(_platform_pose(x=15.0, y=0.0, z=178.0,
                                          publisher='SPACEMOUSE'))
    installed = node._follower_tick(time.perf_counter())
    assert installed is True
    assert node._active_plan.kind == 'move'


def test_follower_tick_deadbands_repeat_target():
    node = _follower_node()
    node._on_platform_pose(_platform_pose(x=15.0, z=178.0, publisher='SPACEMOUSE'))
    assert node._follower_tick(time.perf_counter()) is True
    # Same target again (within deadband) → no reinstall.
    node._on_platform_pose(_platform_pose(x=15.0, z=178.0, publisher='SPACEMOUSE'))
    assert node._follower_tick(time.perf_counter()) is False


def test_follower_input_loss_installs_graceful_stop():
    node = _follower_node()
    # A target arrives, we plan a move, then the stream goes silent.
    node._on_platform_pose(_platform_pose(x=15.0, z=178.0, publisher='SPACEMOUSE'))
    node._follower_tick(time.perf_counter())
    # Age the target past the input-loss window and pin the move as in-flight.
    target, _ = node._follower_target
    node._follower_target = (target,
                             time.perf_counter() - (node._follower_input_loss_s + 1.0))
    node._plan_t0 = time.perf_counter() - 0.05     # mid-move (moving seed)
    installed = node._follower_tick(time.perf_counter())
    assert installed is True
    assert node._follower_input_lost is True
    # The installed stop ends at rest (a graceful decel).
    stop = node._active_plan
    _, v_end, a_end = stop.state_at(stop.total_duration)
    assert np.allclose(v_end, 0.0, atol=1e-9)
    assert np.allclose(a_end, 0.0, atol=1e-9)
    # A second tick under continued loss does NOT reinstall (stop fires once).
    assert node._follower_tick(time.perf_counter()) is False


def test_follower_emits_pump_accepted_frames_over_a_stream():
    """End-to-end at the node boundary: a moving SPACEMOUSE target stream produces
    only pump-accepted frames (the production-in-the-loop invariant)."""
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='SPACEMOUSE'))
    pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
    t = time.perf_counter()
    for k in range(60):
        # Gentle moving target within the workspace.
        node._on_platform_pose(_platform_pose(
            x=10.0 * np.sin(k * 0.1), y=6.0 * np.cos(k * 0.1), z=176.0,
            publisher='SPACEMOUSE'))
        # Restamp the just-stored target on the SYNTHETIC clock: _on_platform_pose
        # stamps with real perf_counter, but this loop advances `t` synthetically, so
        # without this the target reads as stale within ~16 ticks (t outruns the real
        # stamp by > follower_input_loss_s) and the stream would exercise the
        # input-loss STOP path instead of the follow path under test.
        tgt, _ = node._follower_target
        node._follower_target = (tgt, t)
        node._emit_once(t)
        # Pin the fix: the stream stays on the FOLLOW path (a move), never a stop.
        assert node._active_plan.kind == 'move', \
            f"tick {k}: expected a follow move, got {node._active_plan.kind}"
        t += 0.025
    assert len(pub.frames) == 60
    for i, frame in enumerate(pub.frames):
        sp, reason = pump.build(frame, t_origin_us=i * 25000)
        assert reason is None, f"emitted frame {i} pump-rejected: {reason}"


def test_gravity_offset_composed_into_follower_target():
    node = _follower_node()
    # Apply a levelling correction, then a zero-tilt target must come out tilted.
    node._on_gravity_offset(Float64MultiArray(data=[0.05, -0.03]))
    node._on_platform_pose(_platform_pose(x=0.0, z=170.0, publisher='SPACEMOUSE'))
    target, _ = node._follower_target
    # Correction is -[tilt_x, tilt_y] → target rotvec ≈ [-0.05, 0.03, 0].
    assert np.allclose(target[3:6], [-0.05, 0.03, 0.0], atol=1e-3)


def test_follower_mode_entry_resets_stale_target():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='STANDBY'))
    # Enter SPACEMOUSE → follower target cleared (nothing stale carried in).
    node._on_control_mode(String(data='SPACEMOUSE'))
    assert node._follower_target is None
    assert node._follower_input_lost is False


# ── SpaceMouse follower audit fixes (2026-07-08) ──────────────

def test_no_input_loss_within_grace_after_follower_entry():
    """Input loss must NOT be declared instantly on follower-mode entry: a None
    target within the grace window (since entry) is the pre-first-frame period, not a
    dead SpaceMouse node. Early ticks with no target yet → no stop, no latch."""
    node = _follower_node()                    # enters SPACEMOUSE, records entry mono
    assert node._follower_target is None
    plan_before = node._active_plan
    installed = node._follower_tick(time.perf_counter())   # immediately after entry
    assert installed is False
    assert node._follower_input_lost is False
    assert node._pending_stop is False
    assert node._active_plan is plan_before    # seeded hold untouched
    # Past the grace window with STILL no target → input loss finally fires.
    node._follower_entry_mono = (
        time.perf_counter() - (node._follower_input_loss_s + 1.0))
    assert node._follower_tick(time.perf_counter()) is True
    assert node._follower_input_lost is True


def test_follower_install_dropped_when_mode_left_follower_set():
    """TOCTOU: a follow plan built during a tick's ~4-7 ms gate must NOT clobber a
    stop a concurrent mode-exit installed. _install(require_follower_mode=True)
    re-checks the mode under _plan_lock and drops the plan once the mode has left the
    follower set."""
    node = _follower_node()
    follow_plan, _ = planner.build_follow(
        node._current_state(), np.array([15.0, 0.0, 178.0, 0.0, 0.0, 0.0]),
        node._limits, node._geom, hw.JB_TRAJ_SPACEMOUSE_HORIZON_S)
    node._on_control_mode(String(data='STANDBY'))   # mode leaves the follower set
    surviving = node._active_plan
    assert node._install(follow_plan, require_follower_mode=True) is False
    assert node._active_plan is surviving            # not clobbered by the stale plan
    # Without the guard the same plan WOULD install (control).
    assert node._install(follow_plan) is True
    assert node._active_plan is follow_plan


def test_install_with_explicit_t0_sets_plan_time_origin():
    """Per-replan t0: the follower installs with t0 = its seed-sample time so the
    plan's origin matches the state it was seeded from (no v·Δt rewind). _install with
    an explicit t0 uses it verbatim; state_at(0) is exactly the seed pose (C2 origin)."""
    node = _follower_node()
    seed = node._current_state()
    plan, _ = planner.build_follow(
        seed, np.array([12.0, 0.0, 176.0, 0.0, 0.0, 0.0]),
        node._limits, node._geom, hw.JB_TRAJ_SPACEMOUSE_HORIZON_S)
    t0 = time.perf_counter() - 0.01
    assert node._install(plan, t0=t0) is True
    assert node._plan_t0 == t0
    pose0, _, _ = plan.state_at(0.0)
    assert np.allclose(pose0, seed[0], atol=1e-9)


def _boundary_plan(x0):
    """A plan whose t=0 state is at (x0, 0, 170) moving OUTWARD at 40 mm/s — the live
    commanded state a graceful stop is seeded from. At x0≈245 (2 mm inside max +x ≈
    247) build_graceful_stop raises WORKSPACE; well inside it converges."""
    p0 = np.array([x0, 0.0, 170.0, 0.0, 0.0, 0.0])
    v0 = np.array([40.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    seg = QuinticSegment(p0=p0, v0=v0, a0=np.zeros(6),
                         p1=p0, v1=np.zeros(6), a1=np.zeros(6), duration=1.0)
    return TrajectoryPlan(segments=(seg,), final_pose=p0)


def test_pending_stop_on_infeasible_stop_then_retry_converges():
    """Near-boundary outward seed: the input-loss stop build RAISES, so the node sets
    _pending_stop (move keeps running, no failure latch) rather than dropping the stop;
    _emit_once retries from the decaying live state and converges once the state is
    safely stoppable, installing the stop and clearing the flag."""
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='SPACEMOUSE'))
    # Pin the GENTLE limits this scenario's boundary geometry was designed around:
    # at the 2026-07-17 shipped working point (1000/5000/30000) a stop from 2 mm
    # inside the edge is FEASIBLE (the hotter jerk/acc halt within the margin), so
    # the infeasible-raise path under test never engages. The test exercises the
    # pending-stop RETRY mechanism, not the shipped limits.
    node._limits = node._limits.with_session_limits(
        leg_vel_mmps=100.0, leg_acc_mmps2=400.0, leg_jerk_mmps3=8000.0)
    node._install(_boundary_plan(245.0))       # ~2 mm inside +x edge, moving outward
    # Drive the input-loss path (stale target): the stop is wanted but build raises.
    node._follower_target = (
        node._neutral_pose.copy(),
        time.perf_counter() - (node._follower_input_loss_s + 1.0))
    assert node._follower_tick(time.perf_counter()) is False
    assert node._pending_stop is True
    assert node._active_plan.kind == 'move'    # move keeps running (gated), not dropped
    assert node._follower_input_lost is False  # NOT latched on the failed build
    # Velocity "decays" into the safe region → the _emit_once retry converges.
    node._install(_boundary_plan(100.0))       # well inside; a stop is feasible here
    node._emit_once(time.perf_counter())
    assert node._pending_stop is False         # retry converged, flag cleared
    stop = node._active_plan
    _, v_end, a_end = stop.state_at(stop.total_duration)
    assert np.allclose(v_end, 0.0, atol=1e-9)  # a real decel-to-rest
    assert np.allclose(a_end, 0.0, atol=1e-9)


# ── Publish-first emitter + chase-follower escalation (chase-clamp rewrite) ────

def _corrupt_pose_beyond_bound(geom, delta_mm):
    """A level +x pose whose MAX leg extension is ``delta_mm`` past the hard bound
    (delta > 0 = corrupt/out-of-stroke; delta < 0 = in-band). Bisected on x."""
    from jugglebot.motion.trajectory.feasibility import _workspace_limits
    from jugglebot.motion.ik_solver import pose_to_leg_lengths
    wl = _workspace_limits(geom)
    target = wl.leg_hard_max_mm + delta_mm

    def ext_max(x):
        return float(np.max(pose_to_leg_lengths(
            np.array([x, 0.0, 170.0]), np.eye(3), geom)))

    lo, hi = 200.0, 300.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if ext_max(mid) < target:
            lo = mid
        else:
            hi = mid
    return np.array([hi, 0.0, 170.0, 0.0, 0.0, 0.0])


def test_publish_precedes_planning_within_a_tick():
    """C3 publish-first: within one _emit_once the knot is PUBLISHED before the
    follower replan runs. Proven by ordering the pub.send / follower.follow calls —
    a slow or raising replan can never delay this tick's knot."""
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='SPACEMOUSE'))
    node._on_platform_pose(_platform_pose(x=15.0, z=178.0, publisher='SPACEMOUSE'))
    events = []
    real_send, real_follow = pub.send, node._follower.follow
    pub.send = lambda m: (events.append('publish'), real_send(m))[1]
    node._follower.follow = lambda *a, **k: (events.append('follow'),
                                             real_follow(*a, **k))[1]
    t = time.perf_counter()
    node._follower_target = (node._follower_target[0], t)
    node._emit_once(t)
    assert events == ['publish', 'follow'], events


def test_publish_first_emits_frame_even_when_follower_raises():
    """C3: a follower tick that RAISES must not block this tick's knot — publish
    happens first, and the raise is contained (never kills the stream)."""
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='SPACEMOUSE'))
    node._on_platform_pose(_platform_pose(x=15.0, z=178.0, publisher='SPACEMOUSE'))

    def boom(*a, **kw):
        raise RuntimeError('follower boom')

    node._follower.follow = boom
    t = time.perf_counter()
    node._follower_target = (node._follower_target[0], t)
    node._emit_once(t)                       # must not raise
    assert len(pub.frames) == 1              # the knot went out despite the raise


def test_frame_sampled_from_prior_tick_plan_install():
    """C3: the follower install lands AFTER this tick's publish, so it is the NEXT
    tick that samples it. Tick 0 publishes the pre-existing hold and installs the
    move; the move is active only from tick 1 on."""
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='SPACEMOUSE'))
    assert node._active_plan.kind == 'hold'
    node._on_platform_pose(_platform_pose(x=20.0, z=185.0, publisher='SPACEMOUSE'))
    t = time.perf_counter()
    node._follower_target = (node._follower_target[0], t)
    node._emit_once(t)                       # publishes the HOLD, then installs a move
    assert node._active_plan.kind == 'move'  # install happened post-publish
    assert len(pub.frames) == 1


def test_pending_stop_retry_passes_bounded_max_iters():
    """A4: the node's post-publish graceful-stop retry passes a bounded max_iters so
    the follow block stays far under the ~117 ms decay→sprint threshold."""
    from jugglebot.trajectory_node import _STOP_RETRY_ITERS
    node = _follower_node()
    node._pub = _CapturePub()
    node._install(_boundary_plan(245.0))       # near-boundary outward: stop build raises
    node._pending_stop = True
    captured = {}
    real = planner.build_graceful_stop

    def spy(*a, **kw):
        captured['max_iters'] = kw.get('max_iters')
        return real(*a, **kw)

    planner.build_graceful_stop = spy
    try:
        node._emit_once(time.perf_counter())   # post-publish retry runs
    finally:
        planner.build_graceful_stop = real
    assert captured.get('max_iters') == _STOP_RETRY_ITERS


def test_escalation_installs_graceful_stop_under_live_stream():
    """A3: sustained follower rejects (escalate=True) under a LIVE 40 Hz fresh-target
    stream latch a graceful stop that actually INSTALLS (not merely the flag firing).
    A stoppable moving seed → the pending-stop retry installs a decel-to-rest."""
    from jugglebot.motion.trajectory.follower import FollowResult
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='SPACEMOUSE'))
    node._install(_boundary_plan(100.0))       # moving, well inside → stoppable
    node._follower.follow = lambda *a, **kw: FollowResult(
        plan=None, rejection='forced sustained reject', alpha=0.0, escalate=True)
    real_install = node._install
    recorded = []

    def spy_install(plan, **kw):
        ok = real_install(plan, **kw)
        if ok:
            recorded.append(plan)
        return ok

    node._install = spy_install
    t = time.perf_counter()
    for k in range(5):
        node._follower_target = (node._neutral_pose.copy(), t + k * 0.025)
        node._emit_once(t + k * 0.025)
    assert node._escalation_stop is True            # latched under the stream
    assert len(recorded) >= 1                        # a stop actually installed
    stop = node._active_plan
    _, v_end, a_end = stop.state_at(stop.total_duration)
    assert np.allclose(v_end, 0.0, atol=1e-9)        # a real decel-to-rest
    assert np.allclose(a_end, 0.0, atol=1e-9)


def test_corrupt_seed_backstop_holds_under_live_stream():
    """A3 backstop: a seed 0.3 mm BEYOND the hard bound can never be stopped in place
    (build_graceful_stop raises WORKSPACE every tick). Under a live fresh-target
    stream the follower escalates, the retry keeps failing, and after
    _ESCALATE_HOLD_TICKS the node installs a HoldPlan at the last emitted pose
    DIRECTLY — a loud terminal action, never a silent keep-last forever."""
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='SPACEMOUSE'))
    corrupt = _corrupt_pose_beyond_bound(node._geom, 0.3)
    node._install(HoldPlan(corrupt))
    original = node._active_plan
    t = time.perf_counter()
    for k in range(40):                              # > escalate (12) + backstop (12)
        node._follower_target = (node._neutral_pose.copy(), t + k * 0.025)
        node._emit_once(t + k * 0.025)
    assert node._escalation_stop is True             # latched
    assert node._active_plan.kind == 'hold'          # the backstop hold
    assert node._active_plan is not original         # a NEW plan installed (backstop)


def test_escalation_recovery_clears_pending_stop():
    """F1: on the escalation-RECOVERY tick, a fresh follow plan installs and clears
    BOTH the escalation latch AND the pending stop. During an escalation episode the
    fresh-target clear inside _follower_tick is gated off _escalation_stop, so it is
    SKIPPED on the recovery tick; without also clearing _pending_stop when the fresh
    plan installs, it leaks True and the NEXT tick's post-publish retry installs a
    spurious graceful stop over the just-recovered tracking plan — a stop-stutter in
    exactly the near-boundary accept-then-reject regime the rewrite targets."""
    from jugglebot.motion.trajectory.follower import FollowResult
    node = _follower_node()
    node._install(_boundary_plan(100.0))             # a moving, stoppable/trackable seed
    # A real, installable follow plan stands in for the recovered tracking move; the
    # follow() returns it while the escalation latch is (still) set.
    recovered, report = planner.build_follow(
        node._current_state(), np.array([100.0, 0.0, 176.0, 0.0, 0.0, 0.0]),
        node._limits, node._geom, hw.JB_TRAJ_SPACEMOUSE_HORIZON_S)
    node._follower.follow = lambda *a, **kw: FollowResult(
        plan=recovered, report=report, alpha=0.5)
    # Simulate a latched escalation episode still pending (12 prior rejects, stop
    # requested but not yet converged).
    node._escalation_stop = True
    node._pending_stop = True
    t = time.perf_counter()
    node._follower_target = (np.array([100.0, 0.0, 176.0, 0.0, 0.0, 0.0]), t)
    assert node._follower_tick(t) is True
    assert node._active_plan is recovered             # the fresh plan installed
    assert node._escalation_stop is False             # recovery cleared the latch
    assert node._pending_stop is False                # ... and the leaked pending stop


def test_escalation_does_not_latch_after_mode_left_follower():
    """F2: an in-flight follower escalation completing AFTER a concurrent mode-exit
    must NOT re-latch _escalation_stop into the new (non-follower) mode. _on_control_
    mode clears the latches atomically with its _current_mode write under _plan_lock;
    _enter_escalation_stop re-checks the mode under the same lock and refuses once we
    have left. Without it the stale follow plan is dropped by the TOCTOU install guard
    (so _clear_escalation_stop never runs), and the leaked latch reads escalation_stop
    =1 in STANDBY with no active escalation, driving a redundant stop + 'move silenced'."""
    node = _follower_node()                           # SPACEMOUSE
    node._on_control_mode(String(data='STANDBY'))     # leave the follower set
    assert node._current_mode == 'STANDBY'
    assert node._escalation_stop is False and node._pending_stop is False
    # The stale in-flight escalation's follow() returns escalate=True and calls
    # _enter_escalation_stop AFTER the mode exit — it must refuse to latch.
    node._enter_escalation_stop('stale in-flight escalation after mode exit')
    assert node._escalation_stop is False             # refused (mode re-check under lock)
    assert node._pending_stop is False


def test_diagnostics_publish_chase_and_escalation_fields():
    """The chase-follower diagnostics ride the existing DiagnosticStatus KeyValue
    pattern (no new .msg): last chase alpha, consecutive rejects, escalation latch,
    and the post-publish follow-block cost."""
    node = _follower_node()
    node._pub = _CapturePub()
    node._on_platform_pose(_platform_pose(x=15.0, z=178.0, publisher='SPACEMOUSE'))
    tgt, _ = node._follower_target
    node._follower_target = (tgt, time.perf_counter())
    node._emit_once(time.perf_counter())
    kv = _diag_kv(node)
    for key in ('chase_alpha', 'consecutive_rejects', 'escalation_stop',
                'follow_block_max_ms'):
        assert key in kv, f"missing diagnostics key {key}"
    assert kv['escalation_stop'] in ('0', '1')
    assert 0.0 <= float(kv['chase_alpha']) <= 1.0


def test_hold_rejected_in_follower_mode():
    """A hold in a follower mode is a mode confusion (the input stream supersedes it
    within one tick) — reject loudly and tell the operator to exit the mode."""
    node = _follower_node()
    resp = node._svc_hold(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'supersede' in resp.message.lower()


def test_go_home_rejected_in_follower_mode():
    node = _follower_node()
    resp = node._svc_go_home(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'supersede' in resp.message.lower()


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


# ══════════════════════════════════════════════════════════════
# Phase 5 — timed targets + catch path
# ══════════════════════════════════════════════════════════════


def _mk_ros_time(sec_float):
    """builtin_interfaces/Time-like (sec/nanosec) for a float second value."""
    s = int(sec_float)
    return types.SimpleNamespace(sec=s, nanosec=int(round((sec_float - s) * 1e9)))


def _timed_req(node, x=0.0, y=0.0, z=170.0, lead_s=3.0,
               vx=0.0, vy=0.0, vz=0.0, hold_after=True):
    """A TimedTarget request arriving lead_s seconds after service receipt.

    lead_time_s is RELATIVE (the node anchors the absolute arrival at handler
    entry on perf_counter), so no ROS↔perf offset pinning is needed; ``node``
    is kept in the signature for call-site uniformity with the other builders."""
    req = TimedTarget.Request()
    req.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion())
    req.velocity_mm_s = Vector3(x=vx, y=vy, z=vz)
    req.lead_time_s = float(lead_s)
    req.hold_after = hold_after
    return req


def _dyn_target(node, x=0.0, y=0.0, z=190.0, lead_s=3.0, vx=0.0, vy=0.0, vz=0.0):
    """A catch/dynamic_target with a perf-domain arrival ~lead_s ahead of now.
    The pose is STOW-relative (the wire frame, hardware-verified 2026-07-23:
    0 = stow, ~170 = active) and maps through to the commanded pose VERBATIM —
    the former active-relative reading was the double-add that drove every
    hardware catch reach out of stroke."""
    msg = DynamicTargetCommand()
    msg.target_pos = Point(x=x, y=y, z=z)
    msg.target_quat = Quaternion()          # identity
    msg.target_vel = Vector3(x=vx, y=vy, z=vz)
    msg.arrival_time = time.perf_counter() + lead_s
    return msg


def _dyn_target_tilted(node, x=0.0, y=0.0, z=190.0, lead_s=3.0, tilt_rad=0.15):
    """A catch/dynamic_target with a receive tilt about +x (the coordinator supplies
    the tilt in target_quat via compute_catch_orientation). q = rot(tilt_rad about x)."""
    msg = DynamicTargetCommand()
    msg.target_pos = Point(x=x, y=y, z=z)
    half = 0.5 * tilt_rad
    msg.target_quat = Quaternion(w=float(np.cos(half)), x=float(np.sin(half)),
                                 y=0.0, z=0.0)
    msg.target_vel = Vector3()              # stationary catch (coordinator sends zeros)
    msg.arrival_time = time.perf_counter() + lead_s
    return msg


def _arm_catch(node, data):
    """Call the trajectory/arm_catch SetBool handler directly."""
    req = SetBool.Request()
    req.data = bool(data)
    return node._svc_arm_catch(req, SetBool.Response())


def _traj_armed_node():
    """TRAJECTORY mode + the catch-armed latch raised (the reload-action trigger):
    the catch runs in the live stream, gated only by the latch."""
    node = _traj_mode_node()
    _arm_catch(node, True)
    return node


# ── timed_target service ──────────────────────────────────────

def test_timed_target_accepts_generous_lead():
    node = _traj_mode_node()
    resp = node._svc_timed_target(_timed_req(node, z=185.0, lead_s=3.0),
                                  TimedTarget.Response())
    assert resp.accepted is True
    assert resp.code == feas.OK
    assert resp.planned_duration_s == pytest.approx(3.0, abs=0.05)
    assert node._active_plan.kind == 'move'
    # Arrival pose is hit at the reach end.
    p_arr = node._active_plan.state_at(node._active_plan.total_duration)[0]
    assert np.allclose(p_arr[:3], [0, 0, 185], atol=1e-2)


def test_timed_target_wrong_mode_rejected():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='STANDBY'))    # streaming, not TRAJECTORY
    resp = node._svc_timed_target(_timed_req(node, z=185.0), TimedTarget.Response())
    assert resp.accepted is False
    assert resp.code == feas.WRONG_MODE
    assert node._active_plan.kind == 'hold'


def test_timed_target_not_seeded_rejected():
    node = _node()
    node._current_mode = 'TRAJECTORY'    # mode set, never seeded
    resp = node._svc_timed_target(_timed_req(node, z=185.0), TimedTarget.Response())
    assert resp.accepted is False
    assert resp.code == feas.STALE_STATE


def test_timed_target_too_tight_lead_rejected_with_min_duration():
    node = _traj_mode_node()
    resp = node._svc_timed_target(
        _timed_req(node, x=20.0, y=20.0, z=185.0, lead_s=0.05),
        TimedTarget.Response())
    assert resp.accepted is False
    assert resp.code == feas.TOO_FAST
    assert resp.min_duration_s > 0.05
    assert node._active_plan.kind == 'hold'      # unchanged


def test_timed_target_non_finite_lead_rejected():
    """NaN/±inf lead_time_s is the one value class the planner's lead floor/ceiling
    comparisons cannot catch (NaN compares False against both bounds) — the handler
    rejects it loudly UNREACHABLE before any arrival is computed."""
    node = _traj_mode_node()
    for bad in (float('nan'), float('inf'), float('-inf')):
        resp = node._svc_timed_target(_timed_req(node, z=185.0, lead_s=bad),
                                      TimedTarget.Response())
        assert resp.accepted is False, f"lead {bad} must be rejected"
        assert resp.code == feas.UNREACHABLE
    assert node._active_plan.kind == 'hold'      # unchanged


def test_timed_target_non_positive_lead_rejected_too_fast():
    """A zero/negative relative lead flows into the single feasibility gate
    (build_timed's lead floor) and is loudly rejected TOO_FAST with the achievable
    min_duration_s — never planned, never silently late."""
    node = _traj_mode_node()
    for bad in (0.0, -1.0):
        resp = node._svc_timed_target(_timed_req(node, z=185.0, lead_s=bad),
                                      TimedTarget.Response())
        assert resp.accepted is False, f"lead {bad} must be rejected"
        assert resp.code == feas.TOO_FAST
        assert resp.min_duration_s > 0.0
    assert node._active_plan.kind == 'hold'      # unchanged


def test_timed_target_nonzero_velocity_ends_at_rest():
    node = _traj_mode_node()
    resp = node._svc_timed_target(
        _timed_req(node, x=12.0, z=178.0, lead_s=2.5, vx=15.0),
        TimedTarget.Response())
    assert resp.accepted is True
    _, v_end, a_end = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(v_end, 0.0, atol=1e-9)
    assert np.allclose(a_end, 0.0, atol=1e-9)


def test_timed_target_hold_after_false_returns_to_neutral():
    node = _traj_mode_node()
    resp = node._svc_timed_target(
        _timed_req(node, x=12.0, z=185.0, lead_s=3.0, hold_after=False),
        TimedTarget.Response())
    assert resp.accepted is True
    p_end = node._active_plan.state_at(node._active_plan.total_duration)[0]
    assert np.allclose(p_end, node._neutral_pose, atol=1e-2)


# ── Mid-plan supersede (no BUSY for timed targets) ────────────

def test_timed_target_supersedes_in_flight_plan_c2():
    """A timed target is accepted while a move is in flight (NO BUSY restriction) and
    the replan is C2 — the new plan's t=0 state equals the live commanded state."""
    node = _traj_mode_node()
    # First timed target — a move now in flight.
    assert node._svc_timed_target(_timed_req(node, x=15.0, z=185.0, lead_s=3.0),
                                  TimedTarget.Response()).accepted is True
    assert node._active_move_in_flight() is True
    first_plan = node._active_plan
    live_before = node._current_state()
    # Second timed target mid-flight — must be accepted (supersede), not BUSY.
    resp = node._svc_timed_target(_timed_req(node, x=-10.0, z=178.0, lead_s=3.0),
                                  TimedTarget.Response())
    assert resp.accepted is True
    assert node._active_plan is not first_plan
    # C2: the new plan starts at (≈) the state the platform was commanded to.
    p0 = node._active_plan.state_at(0.0)[0]
    assert np.allclose(p0, live_before[0], atol=0.5)


def test_go_to_pose_still_busy_mid_move():
    """go_to_pose keeps its Phase-2 BUSY guard (it uses the ~377 ms analytic gate for
    shaped plans); only the fast-gated timed path lifts BUSY. Documents the
    deliberate asymmetry."""
    node = _traj_mode_node()
    node._svc_timed_target(_timed_req(node, x=15.0, z=185.0, lead_s=3.0),
                           TimedTarget.Response())
    assert node._active_move_in_flight() is True
    resp = node._svc_go_to_pose(_go_to_pose_req(z=180.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is False
    assert resp.code == 'BUSY'


# ── target_feedback publication ───────────────────────────────

def test_timed_target_publishes_accept_feedback():
    node = _traj_mode_node()
    node._svc_timed_target(_timed_req(node, z=185.0, lead_s=3.0),
                           TimedTarget.Response())
    fbs = node.target_feedback_pub.published
    assert len(fbs) >= 1
    assert fbs[-1].accepted is True
    assert fbs[-1].source == 'timed'


def test_timed_target_publishes_reject_feedback():
    node = _traj_mode_node()
    node._svc_timed_target(_timed_req(node, x=20.0, y=20.0, z=185.0, lead_s=0.05),
                           TimedTarget.Response())
    fbs = node.target_feedback_pub.published
    assert fbs[-1].accepted is False
    assert fbs[-1].code == feas.TOO_FAST


# ── catch/dynamic_target consumption (gated by the catch-armed latch) ──

def test_dynamic_target_tilted_installs_tilt_through_seat_plan():
    """Phase 7: a tilted catch routes through ``build_catch`` (not the Phase-5 reach-only
    ``build_timed``), so the installed plan carries the receive tilt through the seat —
    the reach arrives at the tilted pose with a small residual tilt rate that decays to
    rest, then a quiescent hold. Structural evidence: MORE than the level catch's
    reach+hold (the tilt-through decay segment is present), and the plan ends at rest."""
    node = _traj_armed_node()
    node._on_dynamic_target(_dyn_target_tilted(node, x=10.0, z=190.0, lead_s=3.0,
                                               tilt_rad=0.15))
    plan = node._active_plan
    assert plan.kind == 'move'
    assert node.target_feedback_pub.published[-1].accepted is True
    # Tilt-through: reach + decay + quiescent hold ≥ 3 segments (a level catch is 2).
    assert len(plan.segments) >= 3
    # The reach arrival (t = lead) carries the receive tilt with a residual tilt rate.
    _, v_arr, _ = plan.state_at(3.0)
    assert abs(v_arr[3]) > 1e-3          # nonzero rx-rate through the seat
    # Rest-terminating: the settle/hold ends at zero twist and zero accel (a nonzero
    # terminal velocity would be an unbounded-jerk snap at the terminal hold).
    _, v_end, a_end = plan.state_at(plan.total_duration)
    assert np.allclose(v_end, 0.0, atol=1e-6)
    assert np.allclose(a_end, 0.0, atol=1e-6)


def test_dynamic_target_level_catch_has_no_tilt_through():
    """A LEVEL catch (identity orientation) has no tilt direction to carry through the
    seat, so ``build_catch`` degenerates to reach + quiescent hold (2 segments) with no
    residual tilt rate — the tilt-through path is opt-in on a real receive tilt."""
    node = _traj_armed_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=190.0, lead_s=3.0))
    plan = node._active_plan
    assert len(plan.segments) == 2       # reach + quiescent hold, no decay segment
    _, v_arr, _ = plan.state_at(3.0)
    assert np.allclose(v_arr, 0.0, atol=1e-6)


def test_dynamic_target_catch_knots_pump_accepted():
    """Production-in-the-loop at the node boundary: every 40 Hz knot the emitter ships
    for an installed (tilted) ``build_catch`` plan is accepted by a REAL SetpointPump —
    the load-bearing emitter/pump invariant, re-asserted for the catch path."""
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='TRAJECTORY'))
    _arm_catch(node, True)
    node._on_dynamic_target(_dyn_target_tilted(node, x=15.0, y=8.0, z=190.0,
                                               lead_s=2.5, tilt_rad=0.18))
    assert node._active_plan.kind == 'move'
    pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
    t = time.perf_counter()
    pub.frames.clear()
    for _ in range(120):                 # 3 s of catch at 40 Hz — through the seat + hold
        node._emit_once(t)
        t += 0.025
    assert len(pub.frames) == 120
    for i, frame in enumerate(pub.frames):
        _sp, reason = pump.build(frame, t_origin_us=i * 25000)
        assert reason is None, f"catch frame {i} pump-rejected: {reason}"


def test_dynamic_target_reach_freeze_ignores_late_updates():
    """Inside the reach-freeze window a late target update is ignored (the committed
    reach is held) AND reported FROZEN — a distinct service-level code that
    distinguishes the freeze branch from a TOO_FAST feasibility reject."""
    node = _traj_armed_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=190.0, lead_s=3.0))
    committed = node._active_plan
    # Pin the committed arrival DETERMINISTICALLY inside the freeze window: now is past
    # (arrival − reach_freeze) and before (arrival + settle_hold).
    node._catch_arrival_perf = time.perf_counter() + 0.5 * node._catch_reach_freeze_s
    n_fb = len(node.target_feedback_pub.published)
    # A generous-lead update that WOULD supersede if not frozen — proves the freeze
    # (not a feasibility reject) is what blocks it.
    node._on_dynamic_target(_dyn_target(node, x=40.0, z=190.0, lead_s=3.0))
    assert node._active_plan is committed
    fb = node.target_feedback_pub.published[-1]
    assert fb.accepted is False
    assert fb.code == 'FROZEN'
    assert len(node.target_feedback_pub.published) == n_fb + 1


def test_dynamic_target_post_settle_supersedes_as_new_reach():
    """Once the committed arrival + settle-hold has fully passed, the freeze releases
    and a later target supersedes as a fresh reach (repeated catches — Phases 8/9),
    rather than being silently dropped forever behind a latched freeze."""
    node = _traj_armed_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=190.0, lead_s=3.0))
    first = node._active_plan
    # Pretend the committed arrival + settle-hold is now fully in the past.
    node._catch_arrival_perf = (time.perf_counter()
                                - node._catch_settle_hold_s - 0.1)
    node._on_dynamic_target(_dyn_target(node, x=25.0, z=190.0, lead_s=3.0))
    assert node._active_plan is not first
    assert node.target_feedback_pub.published[-1].accepted is True
    p_arr = node._active_plan.state_at(node._active_plan.total_duration)[0]
    assert np.allclose(p_arr[:3], [25.0, 0.0, 190.0], atol=1e-2)


def test_dynamic_target_past_arrival_rejected_too_fast():
    """A catch target whose arrival is already in the PAST → negative lead → loud
    TOO_FAST (never a crash), plan unchanged."""
    node = _traj_armed_node()
    committed = node._active_plan            # the seeded hold
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=190.0, lead_s=-1.0))
    assert node._active_plan is committed
    fb = node.target_feedback_pub.published[-1]
    assert fb.accepted is False
    assert fb.code == feas.TOO_FAST


def test_dynamic_target_supersedes_before_freeze():
    """Outside the freeze window a new catch target supersedes the prior (C2)."""
    node = _traj_armed_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=190.0, lead_s=3.0))
    first = node._active_plan
    node._on_dynamic_target(_dyn_target(node, x=25.0, z=190.0, lead_s=3.0))
    assert node._active_plan is not first
    p_arr = node._active_plan.state_at(node._active_plan.total_duration)[0]
    assert np.allclose(p_arr[:3], [25.0, 0.0, 190.0], atol=1e-2)


def test_dynamic_target_z_wire_is_stow_relative_no_lift():
    """REGRESSION (2026-07-23 Phase-7 hardware session): the wire z is STOW-relative
    and must map to the commanded pose VERBATIM. The node previously added the active
    z (170) on the false premise that the wire was active-relative, commanding
    z ≈ 341 mm — past the 275 mm leg-stroke hard max — so EVERY hardware catch reach
    was gate-rejected WORKSPACE and the platform never tilted. A wire z of ~171 (the
    coordinator's output for a cup-plane ball at the 12° tilt clamp) must land the
    commanded catch a few mm above the ACTIVE pose, in-stroke, accepted."""
    node = _traj_armed_node()
    node._on_dynamic_target(_dyn_target_tilted(node, x=-13.5, z=171.4, lead_s=3.0,
                                               tilt_rad=0.209))
    assert node.target_feedback_pub.published[-1].accepted is True
    plan = node._active_plan
    assert plan.kind == 'move'
    # The reach arrival (t = lead) is at the WIRE z — no lift applied.
    p_arr = plan.state_at(3.0)[0]
    assert p_arr[2] == pytest.approx(171.4, abs=1e-2)


def test_catch_reach_envelope_rejects_far_target():
    """A catch target farther than JB_TRAJ_CATCH_REACH_ENVELOPE_MM (3D) from the pose
    held at arm-latch raise is rejected WORKSPACE before planning — the enforcement
    point for the envelope ``build_catch`` documents as the caller's responsibility.
    WORKSPACE (not a transient code) is deliberate: the coordinator's feasibility
    blacklist counts it, so a drifting ball-tracker landing estimate (measured
    435-605 mm/s on BB throws, 2026-07-23) blacklists out instead of dragging the
    platform sideways through the flight window."""
    node = _traj_armed_node()
    committed = node._active_plan            # the seeded hold
    node._on_dynamic_target(_dyn_target(node, x=100.0, z=190.0, lead_s=3.0))
    assert node._active_plan is committed
    fb = node.target_feedback_pub.published[-1]
    assert fb.accepted is False
    assert fb.code == feas.WORKSPACE
    assert 'envelope' in fb.reason
    # A z-frame regression (e.g. the double-add reappearing) is caught HERE, loudly:
    # a level target at z=360 (= 190 + a stray active-z lift) is ~190 mm out.
    node._on_dynamic_target(_dyn_target(node, x=0.0, z=360.0, lead_s=3.0))
    fb = node.target_feedback_pub.published[-1]
    assert fb.accepted is False and 'envelope' in fb.reason


def test_catch_reach_envelope_center_lifecycle():
    """The envelope center is captured at the arm-latch RAISE (the held pose the catch
    session starts from) and cleared at the lower edge — a stale center must never
    bound the NEXT session's targets from an old pose."""
    node = _traj_mode_node()
    assert node._catch_envelope_center is None
    _arm_catch(node, True)
    assert node._catch_envelope_center is not None
    assert np.allclose(node._catch_envelope_center,
                       node._current_state()[0][:3], atol=1e-9)
    _arm_catch(node, False)
    assert node._catch_envelope_center is None


# ── catch-armed latch (reload-action-catch-latch Phase 1) ─────

def test_arm_catch_starts_disarmed():
    node = _node()
    assert node._catch_armed is False


def test_dynamic_target_ignored_when_disarmed_in_trajectory():
    """Latch down → a dynamic_target is ignored (the gate holds when the catch-armed
    latch is not raised)."""
    node = _traj_mode_node()
    assert node._catch_armed is False
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=190.0, lead_s=3.0))
    assert node._active_plan.kind == 'hold'
    assert node.target_feedback_pub.published == []


def test_arm_catch_latch_lets_dynamic_target_install_catch():
    """Arming the latch while in TRAJECTORY lets a catch/dynamic_target install a
    build_catch reach — the reload-action trigger, in the live stream."""
    node = _traj_armed_node()
    assert node._current_mode == 'TRAJECTORY'      # the latch is the trigger, not a mode
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=190.0, lead_s=3.0))
    assert node._active_plan.kind == 'move'
    p_arr = node._active_plan.state_at(node._active_plan.total_duration)[0]
    assert np.allclose(p_arr[:3], [10.0, 0.0, 190.0], atol=1e-2)     # wire z verbatim (stow-relative)
    assert node._catch_arrival_perf is not None
    assert node.target_feedback_pub.published[-1].accepted is True
    assert node.target_feedback_pub.published[-1].source == 'catch'


def test_arm_catch_idempotent_arm_is_noop():
    """Arming when already armed (or disarming when already disarmed) is a no-op — no
    edge, so no freeze reset / stop (mirrors _on_control_mode's same-mode return)."""
    node = _traj_armed_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=190.0, lead_s=3.0))
    committed = node._active_plan
    node._catch_arrival_perf = time.perf_counter() + 3.0
    resp = _arm_catch(node, True)                  # already armed
    assert resp.success is True
    assert node._catch_arrival_perf is not None    # NOT reset (no edge)
    assert node._active_plan is committed


def test_disarm_catch_resets_freeze_no_discontinuity():
    """Disarming mid-reach (the documented abort) installs a graceful stop — the catch
    reach is silenced, not run on to the target — and clears the freeze, with the stop
    seeded C2 off the live state (ends at rest: no command discontinuity at the seam)."""
    node = _traj_armed_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=190.0, lead_s=3.0))
    catch_plan = node._active_plan
    assert catch_plan.kind == 'move'
    assert node._catch_arrival_perf is not None
    node._plan_t0 -= 1.0                            # advance into the reach (real velocity)
    assert node._active_move_in_flight() is True
    _arm_catch(node, False)                         # disarm mid-reach = abort
    assert node._catch_armed is False
    assert node._active_plan is not catch_plan      # a graceful stop was installed
    assert node._catch_arrival_perf is None         # freeze released
    end = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(end[1], 0.0, atol=1e-6)                        # ends at rest
    assert not np.allclose(end[0][:3], [10.0, 0.0, 190.0], atol=1.0)  # not the target
    # The catch path is gated off again after disarm.
    n_fb = len(node.target_feedback_pub.published)
    node._on_dynamic_target(_dyn_target(node, x=25.0, z=190.0, lead_s=3.0))
    assert len(node.target_feedback_pub.published) == n_fb            # ignored


def test_arm_catch_mid_move_installs_stop():
    """Arming while a TRAJECTORY move is in flight installs a graceful stop (so the
    catch begins from a clean, non-jittering state) and resets the freeze."""
    node = _traj_mode_node()
    node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=3.0), GoToPose.Response())
    move_plan = node._active_plan
    assert move_plan.kind == 'move'
    node._plan_t0 -= 1.0
    assert node._active_move_in_flight() is True
    node._catch_arrival_perf = time.perf_counter() + 3.0
    _arm_catch(node, True)
    assert node._catch_armed is True
    assert node._active_plan is not move_plan
    assert node._catch_arrival_perf is None
    end = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(end[1], 0.0, atol=1e-6)                       # ends at rest


def test_arm_catch_from_hold_no_stop():
    """Arming from a settled hold (no move in flight) does not install a stop — it just
    raises the latch and clears the freeze (the common reload pre-position case)."""
    node = _traj_mode_node()
    hold = node._active_plan
    assert hold.kind == 'hold'
    _arm_catch(node, True)
    assert node._catch_armed is True
    assert node._active_plan is hold                # untouched — no graceful stop
    assert node._catch_arrival_perf is None


def test_arm_catch_force_disarmed_on_mode_change():
    """SAFETY: the catch-armed latch is force-disarmed on ANY control-mode change. The
    reload runs only in TRAJECTORY; leaving it (an E-STOP → STANDBY, an operator
    abort/deactivate, or the action dying without lowering the latch) must clear the
    latch so a stray catch/dynamic_target can't install a build_catch with no reload in
    progress. Restores the safety the retired CATCH mode gave for free (its gate was
    `_current_mode != CATCH`, so leaving CATCH inherently disarmed the catch)."""
    node = _traj_armed_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=190.0, lead_s=3.0))
    assert node._catch_armed is True
    assert node._catch_envelope_center is not None
    assert node._catch_arrival_perf is not None
    # A mode change away from TRAJECTORY (e.g. an E-STOP → STANDBY) force-disarms.
    node._on_control_mode(String(data='STANDBY'))
    assert node._catch_armed is False
    assert node._catch_arrival_perf is None          # freeze released
    assert node._catch_envelope_center is None       # envelope center cleared
    # Re-entering TRAJECTORY does NOT re-arm — the reload must explicitly re-arm.
    node._on_control_mode(String(data='TRAJECTORY'))
    assert node._catch_armed is False
    n_fb = len(node.target_feedback_pub.published)
    node._on_dynamic_target(_dyn_target(node, x=25.0, z=190.0, lead_s=3.0))
    assert len(node.target_feedback_pub.published) == n_fb   # ignored (latch down)


def test_timed_target_excessive_lead_rejected_no_crash():
    """A lead well over max_timed_lead_s (e.g. an absolute timestamp mistakenly
    pasted into the relative lead_time_s) is loudly rejected via the service —
    never a crash / MemoryError — and the plan is unchanged."""
    node = _traj_mode_node()
    committed = node._active_plan
    resp = node._svc_timed_target(_timed_req(node, z=185.0, lead_s=61.0),
                                  TimedTarget.Response())
    assert resp.accepted is False
    assert resp.code == feas.TOO_FAST
    assert node._active_plan is committed


def test_go_to_pose_records_predicted_peaks_after_the_install_clears_them():
    """`_install` clears `_last_peak_*`, so the write must come AFTER it.

    The ordering is load-bearing in both directions and each half fails silently
    on its own: written before the install, the accepted move's peaks are erased
    and `/trajectory/diagnostics` reports 0.0 for a real move; never cleared, a
    report-LESS install (hold / go_home / graceful stop / guard descent) leaves
    the superseded plan's peaks published under an already-advanced `move_seq`,
    which is the bench observation behind `tests/hardware/mvp_bench_runbook.md`
    open item 7.
    """
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is True
    # Non-zero proves the write survived `_install`'s clear; matching the
    # re-gated plan proves it is THIS plan's report and not a leftover. The
    # tolerance is loose because `go_to_pose` is lean-SHAPED: `build_move`
    # returns the retiming path's report, whose sampling grid differs from a
    # fresh `validate_follow` by ~3e-4 relative. (`timed_target` below has no
    # shaper, so it matches exactly — hence the two different tolerances.)
    rep = feas.validate_follow(node._active_plan, node._limits, node._geom)
    assert node._last_peak_vel_mmps > 0.0
    assert node._last_peak_vel_mmps == pytest.approx(rep.peak_leg_vel_mmps,
                                                     rel=1e-3)
    assert node._last_peak_acc_mmps2 == pytest.approx(rep.peak_leg_acc_mmps2,
                                                      rel=1e-3)
    assert node._last_peak_jerk_mmps3 == pytest.approx(rep.peak_leg_jerk_mmps3,
                                                       rel=1e-2)


def test_a_report_less_install_clears_the_predicted_peaks():
    """A hold carries no `FeasibilityReport`, so it must publish 0.0, not the
    previous plan's numbers under a NEW `move_seq` — a diagnostic that silently
    attributes one plan's peaks to another sent the 2026-07-25 investigation
    chasing a "stale or cached" field that was neither."""
    node = _traj_mode_node()
    assert node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                GoToPose.Response()).accepted is True
    assert node._last_peak_vel_mmps > 0.0
    seq_before = node._move_seq

    assert node._svc_hold(Trigger.Request(), Trigger.Response()).success is True
    assert node._move_seq > seq_before, 'the hold must bump move_seq'
    assert node._last_peak_vel_mmps == 0.0
    assert node._last_peak_acc_mmps2 == 0.0
    assert node._last_peak_jerk_mmps3 == 0.0


def test_timed_target_records_predicted_peaks():
    """An accepted timed install records the accepting report's PREDICTED leg peaks into
    _last_peak_* (as go_to_pose does), so /diagnose per-move rows carry the right
    predicted peaks — the report is no longer discarded."""
    node = _traj_mode_node()
    resp = node._svc_timed_target(_timed_req(node, x=12.0, z=185.0, lead_s=3.0),
                                  TimedTarget.Response())
    assert resp.accepted is True
    rep = feas.validate_follow(node._active_plan, node._limits, node._geom)
    assert node._last_peak_vel_mmps == pytest.approx(rep.peak_leg_vel_mmps)
    assert node._last_peak_acc_mmps2 == pytest.approx(rep.peak_leg_acc_mmps2)
    assert node._last_peak_jerk_mmps3 == pytest.approx(rep.peak_leg_jerk_mmps3)
    assert node._last_peak_vel_mmps > 0.0


# ── clock conversion (retained plumbing — no production consumer since the
# timed_target service moved to a relative lead_time_s; kept as the single
# crossing point for any future ROS-clock-timed surface) ──────

def test_ros_time_to_perf_uses_offset():
    node = _node()
    node._ros_to_perf_offset = 100.0
    perf = node._ros_time_to_perf(_mk_ros_time(5.25))
    assert perf == pytest.approx(105.25)


# ── Production-in-the-loop at the node: timed frames pump-accepted ──

def test_timed_target_emitted_frames_pump_accepted():
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='TRAJECTORY'))
    node._svc_timed_target(_timed_req(node, x=12.0, z=182.0, lead_s=2.5, vx=15.0),
                           TimedTarget.Response())
    pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
    t0 = node._plan_t0
    plan = node._active_plan
    n = int(plan.total_duration / 0.025) + 8
    for i in range(n):
        frame = node._emitter.frame(plan, i * 0.025, i)
        sp, reason = pump.build(frame, t_origin_us=i * 25000)
        assert reason is None, f"pump rejected timed node frame {i}: {reason}"
    assert pump.frames_rejected == 0


# ══════════════════════════════════════════════════════════════════
# FIX 1 — Teensy guard-latch freeze (subscribe /link_status; on a latched guard,
# walk u0 down to measured via a gate-validated PROFILED DESCENT + freeze target
# advancement; on clear, reseed + resume; a latch outside streaming is a no-op).
# See the 2026-07-10 runaway: an INSTANT reseed to measured was a >0.3 rev u0 jump
# the pump's step gate rejected, so the collapse never reached the Teensy.
# ══════════════════════════════════════════════════════════════════

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def _link_status(fault_state='NONE', mpc_active='1'):
    """Build a /link_status DiagnosticStatus with the fields FIX 1 reads."""
    msg = DiagnosticStatus()
    msg.values = [KeyValue(key='fault_state', value=fault_state),
                  KeyValue(key='mpc_active', value=mpc_active)]
    return msg


def _install_far_command(node, dz_mm=40.0):
    """Simulate the runaway: install a commanded hold ``dz_mm`` above the measured
    encoder (z+40 mm ≈ 0.53 rev/leg > the 0.3 rev pump step gate), so an instant
    reseed to measured would be a jump the pump REJECTS but a profiled descent walks
    down through the gate. Returns the far commanded pose."""
    far = node._active_plan.state_at(0.0)[0].copy()
    far[2] += dz_mm
    node._install(planner.build_hold((far, np.zeros(6), np.zeros(6)),
                                     node._limits, node._geom))
    return far


def test_guard_latch_installs_profiled_descent_to_measured():
    node = _follower_node()                       # SPACEMOUSE, seeded at activate
    # The measured pose moves, THEN the guard latches → the descent walks the command
    # down to the NEW measured pose (not the stale pre-latch command). A profiled
    # descent (a MOVE ending at rest), NOT the old instant hold.
    moved = [r + 0.15 for r in _ACTIVATE_REV]
    node._on_robot_state(_robot_state(moved))
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    assert node._active_plan.kind == 'move'       # a profiled descent, not an instant hold
    fp, ftw, _ = node._active_plan.state_at(node._active_plan.total_duration)
    fp_rev = node._pose_to_motor_rev(fp)
    assert np.allclose(fp_rev, moved, atol=1e-2)  # descent ends AT measured
    assert np.allclose(ftw, 0.0, atol=1e-9)       # at rest


def test_guard_latch_descent_knots_all_pump_accepted_then_holds_at_measured():
    """Production-in-the-loop: on a runaway-sized divergence the latch installs a
    profiled descent whose EVERY emitted knot passes a REAL SetpointPump (primed with
    the far command), it is never superseded while frozen, and it terminally holds AT
    the measured encoder, at rest."""
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._on_robot_state(_robot_state())              # measured = activate (z≈170)
    node._on_control_mode(String(data='SPACEMOUSE'))   # follower, seeded hold
    _install_far_command(node, dz_mm=40.0)             # command ran to z≈210
    pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)
    # Prime the pump with the pre-latch (runaway) command so _prev_pos == far: an
    # instant reseed to measured now would be a ≈0.53 rev jump the pump REJECTS.
    t_far = node._plan_t0
    node._emit_once(t_far)
    _sp, reason = pump.build(pub.frames[-1], t_origin_us=0)
    assert reason is None
    far_prev = list(pump._prev_pos)
    assert max(abs(far_prev[i] - _ACTIVATE_REV[i]) for i in range(6)) > 0.3

    # Latch: the guard installs a PROFILED DESCENT, not an instant hold.
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    assert node._active_plan.kind == 'move'
    descent_plan = node._active_plan
    t0 = node._plan_t0
    n = int(descent_plan.total_duration / 0.025) + 40
    for k in range(1, n + 1):
        # A live SpaceMouse target it must IGNORE while frozen (no supersede).
        node._follower_target = (np.array([50.0, 0.0, 210.0, 0.0, 0.0, 0.0]),
                                 time.perf_counter())
        node._emit_once(t0 + k * 0.025)
        _sp, reason = pump.build(pub.frames[-1], t_origin_us=k * 25000)
        assert reason is None, f"descent knot {k} pump-rejected: {reason}"
    assert node._active_plan is descent_plan          # never superseded while frozen
    # Converged: the last accepted u0 sits on the measured encoder, at rest.
    assert np.allclose(pump._prev_pos, _ACTIVATE_REV, atol=2e-2)
    fp, ftw, _ = descent_plan.state_at(descent_plan.total_duration)
    assert np.allclose(fp[:3], [0.0, 0.0, 170.0], atol=0.5)
    assert np.allclose(ftw, 0.0, atol=1e-9)


def test_guard_latch_descent_falls_back_to_stop_then_descent(monkeypatch):
    """Edge (a): a near-boundary outward-moving commanded seed makes the DIRECT
    descent overshoot the workspace, so build_follow raises. The handler decels in
    place (build_graceful_stop) THEN descends from rest, concatenated into one C2
    plan that still ends at measured, at rest."""
    node = _follower_node()
    far = _install_far_command(node, dz_mm=40.0)
    # Put the command MID-MOVE (a moving seed) so the in-place decel yields real
    # segments and the concatenation is non-trivial (stop segs + descent segs).
    mv, _rep = planner.build_move((far, np.zeros(6), np.zeros(6)),
                                  node._neutral_pose.copy(), 2.0,
                                  node._limits, node._geom)
    node._install(mv, t0=time.perf_counter() - 0.4)    # 0.4 s into a 2 s move → moving
    real_build_follow = planner.build_follow
    calls = {'n': 0}

    def fake_build_follow(*args, **kw):
        calls['n'] += 1
        if calls['n'] == 1:                            # the direct (moving-seed) descent
            raise TrajectoryInfeasible(feas.WORKSPACE, ['overshoot leaves workspace'])
        return real_build_follow(*args, **kw)          # the from-rest descent

    monkeypatch.setattr(planner, 'build_follow', fake_build_follow)
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    assert node._active_plan.kind == 'move'
    assert len(node._active_plan.segments) >= 2         # stop segs + descent segs
    assert calls['n'] == 2                              # direct rejected → rest-descent built
    fp, ftw, _ = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(node._pose_to_motor_rev(fp), _ACTIVATE_REV, atol=1e-2)
    assert np.allclose(ftw, 0.0, atol=1e-9)


def test_guard_latch_freezes_in_place_when_descent_gate_rejected(monkeypatch):
    """Edge (a) final fallback: the descent is gate-rejected even after an in-place
    decel → freeze u0 in place (hold at the last emitted pose) so it stops running
    away. u0 stays diverged, so /recover cannot converge (the manual dance is then
    required) — the deliberately-safe non-recovery."""
    node = _follower_node()
    far = _install_far_command(node, dz_mm=40.0)
    node._last_pose = far.copy()                        # the last emitted (known-good) pose

    def _raise(*a, **k):
        raise TrajectoryInfeasible(feas.WORKSPACE, ['rejected'])

    monkeypatch.setattr(planner, 'build_follow', _raise)
    monkeypatch.setattr(planner, 'build_graceful_stop', _raise)
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    assert node._active_plan.kind == 'hold'             # froze in place (never advanced)
    assert np.allclose(node._active_plan.state_at(0.0)[0], far, atol=1e-9)


def test_guard_latch_drops_fresh_spacemouse_target():
    node = _follower_node()
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    node._on_platform_pose(_platform_pose(x=30.0, z=190.0, publisher='SPACEMOUSE'))
    assert node._follower_target is None          # dropped (never stored) while frozen


def test_guard_latch_emit_streams_descent_without_supersede():
    """While frozen the emitter keeps PUBLISHING (no MPC_STALE) and plays out the
    installed descent — u0 ADVANCES down toward measured — but a fresh follower target
    must NOT supersede the descent (which would re-diverge u0 on the next clear)."""
    pub = _CapturePub()
    node = _node(command_pub_factory=lambda: pub)
    node._pub = pub
    node._on_robot_state(_robot_state())               # measured = activate
    node._on_control_mode(String(data='SPACEMOUSE'))
    _install_far_command(node, dz_mm=40.0)             # command ran to z≈210
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    descent_plan = node._active_plan
    assert descent_plan.kind == 'move'                 # a descent, playing out
    t0 = node._plan_t0
    u0_first = None
    for k in range(5):
        # Inject a fresh follower target directly (bypassing the frozen intake): it
        # must NOT be chased while frozen.
        node._follower_target = (np.array([50.0, 0.0, 210.0, 0.0, 0.0, 0.0]),
                                 time.perf_counter())
        node._emit_once(t0 + k * 0.025)
        if u0_first is None:
            u0_first = np.asarray(pub.frames[-1]['motor_rev'], dtype=float)
    assert node._active_plan is descent_plan           # follower never superseded it
    assert len(pub.frames) == 5                         # stream never stopped
    # u0 is DESCENDING (advancing along the plan), not frozen in place.
    u0_last = np.asarray(pub.frames[-1]['motor_rev'], dtype=float)
    assert not np.allclose(u0_last, u0_first, atol=1e-4)


def test_guard_clear_reseeds_and_resumes_targets():
    node = _follower_node()                            # command ≈ measured (converged)
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    # Guard clears (fault_state back to NONE) while still streaming. The descent has
    # converged (command within the pump gate of measured) → clean instant hold reseed.
    node._on_link_status(_link_status('NONE'))
    assert node._guard_frozen is False
    assert node._active_plan.kind == 'hold'            # zero-step reseed at measured
    # Target acceptance resumes: a fresh target installs a follower move again.
    node._on_platform_pose(_platform_pose(x=15.0, z=178.0, publisher='SPACEMOUSE'))
    assert node._follower_target is not None
    assert node._follower_tick(time.perf_counter()) is True
    assert node._active_plan.kind == 'move'


def test_guard_clear_mid_descent_does_not_jump_u0():
    """A bare /clear_errors MID-descent (u0 still > the pump gate from measured) must
    NOT instant-reseed — that would jump u0 across the gate. The profiled descent is
    left in flight to finish (it terminally holds at measured)."""
    node = _follower_node()
    _install_far_command(node, dz_mm=40.0)             # command ≈0.53 rev from measured
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    descent_plan = node._active_plan
    assert descent_plan.kind == 'move'
    # Clear arrives immediately — the descent has NOT converged yet. No instant reseed.
    node._on_link_status(_link_status('NONE'))
    assert node._guard_frozen is False
    assert node._active_plan is descent_plan           # NOT replaced by an instant hold
    assert node._active_plan.kind == 'move'


def test_guard_latch_outside_streaming_is_noop():
    node = _node()
    node._on_robot_state(_robot_state())           # telemetry cached, NOT streaming
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is False             # no-op: nothing to freeze
    assert node._active_plan is None               # nothing seeded/installed


def test_leaving_stream_mode_clears_guard_freeze():
    node = _follower_node()
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    node._on_control_mode(String(data='DISABLED'))  # leave the streaming set
    assert node._guard_frozen is False
    assert node._streaming is False


def test_go_to_pose_rejected_while_guard_latched():
    node = _traj_mode_node()                        # TRAJECTORY, seeded
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    resp = node._svc_go_to_pose(_go_to_pose_req(x=10.0, z=180.0),
                                GoToPose.Response())
    assert resp.accepted is False
    assert resp.code == 'GUARD_LATCHED'
    assert 'guard' in resp.message.lower()


def test_go_home_rejected_while_guard_latched():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='STANDBY'))
    node._on_link_status(_link_status('MAX_DEVIATION'))
    resp = node._svc_go_home(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert 'guard' in resp.message.lower()


def test_reseed_from_measured_installs_profiled_descent():
    """/recover step 1: reseed_from_measured installs the gate-validated PROFILED
    descent onto measured (NOT an instant hold — that would jump u0 across the pump
    gate mid-descent). The descent ends at the measured encoder, at rest."""
    node = _follower_node()
    _install_far_command(node, dz_mm=40.0)             # command far from the encoder
    resp = node._svc_reseed_from_measured(Trigger.Request(), Trigger.Response())
    assert resp.success is True
    assert 'descent' in resp.message.lower()
    assert node._active_plan.kind == 'move'            # a descent, not an instant hold
    fp, ftw, _ = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(node._pose_to_motor_rev(fp), _ACTIVATE_REV, atol=1e-2)
    assert np.allclose(ftw, 0.0, atol=1e-9)


def test_reseed_from_measured_rejected_when_not_streaming():
    node = _node()
    node._on_robot_state(_robot_state())            # telemetry, but not streaming
    resp = node._svc_reseed_from_measured(Trigger.Request(), Trigger.Response())
    assert resp.success is False
    assert node._active_plan is None


def test_reseed_re_samples_live_encoder_each_install():
    """Drift-resample (2026-07-11 clear-errors jolt): each reseed_from_measured targets
    the CURRENT encoder, not a cached install-time snapshot. The leg drifts ~0.1 rev
    during guard suppression (the ODrive closing its frozen lead offset — the Event-1
    −0.102 rev plateau), so a re-install (the bridge's bounded re-descend) must chase
    the new measured pose, or u0 lands off the live encoder and the clear still jolts."""
    node = _follower_node()                            # seeded at _ACTIVATE_REV
    _install_far_command(node, dz_mm=40.0)             # command far above the encoder
    # First reseed → descent onto the current (activate) encoder.
    node._svc_reseed_from_measured(Trigger.Request(), Trigger.Response())
    fp, _tw, _ = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(node._pose_to_motor_rev(fp), _ACTIVATE_REV, atol=1e-2)
    # The leg drifts +0.1 rev during suppression → fresh telemetry reports the new pose.
    drifted = [r + 0.1 for r in _ACTIVATE_REV]
    node._on_robot_state(_robot_state(drifted))
    # Re-install → the descent now targets the DRIFTED encoder (re-sampled, not cached).
    node._svc_reseed_from_measured(Trigger.Request(), Trigger.Response())
    fp2, tw2, _ = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(node._pose_to_motor_rev(fp2), drifted, atol=1e-2)
    assert np.allclose(tw2, 0.0, atol=1e-9)


def test_guard_clear_edge_reseed_re_samples_drifted_encoder():
    """The guard-CLEAR-edge instant reseed (the /recover happy path) must also snap the
    hold to the LIVE (drifted) encoder, not the install-time pose. After the descent
    converged onto the encoder, a small further drift within the pump step gate must be
    picked up so the resumed hold sits exactly where the leg rests (zero re-enable step)."""
    node = _follower_node()
    node._on_link_status(_link_status('MAX_DEVIATION'))   # latch → descent installed
    # The descent converged; the leg then settles a hair further, still within the gate.
    settled = [r + 0.02 for r in _ACTIVATE_REV]
    node._on_robot_state(_robot_state(settled))
    node._on_link_status(_link_status('NONE'))            # clear edge → instant reseed
    assert node._guard_frozen is False
    assert node._active_plan.kind == 'hold'               # zero-step reseed at measured
    fp, _tw, _ = node._active_plan.state_at(0.0)
    assert np.allclose(node._pose_to_motor_rev(fp), settled, atol=1e-2)  # the DRIFTED pose


def test_guard_latch_freezes_at_last_pose_when_telemetry_stale():
    """If measured telemetry is stale when the guard latches, the reseed-to-measured
    can't run — but u0 must still STOP advancing: fall back to holding the last
    emitted pose (never let a diverged move run on)."""
    node = _follower_node()
    node._robot_state_mono = time.perf_counter() - (node._robot_state_stale_s + 5.0)
    node._on_link_status(_link_status('MAX_DEVIATION'))
    assert node._guard_frozen is True
    assert node._active_plan.kind == 'hold'      # froze at last pose (did not advance)


# ── ARMING_CONTRACT A5: seed gating + the loud disarmed wire ───

def test_seed_deferred_until_homed():
    """No seeding before homing completes: mid-homing the legs sit at arbitrary
    index-search extensions below the workspace hard margin, so every attempt
    failed the gate at the 100 Hz telemetry rate (4091 ERROR lines in 41 s on
    2026-07-15, burying the real fault under spam)."""
    node = _node()
    node._on_robot_state(_robot_state(is_homed=False))
    node._on_control_mode(String(data='STANDBY'))
    assert node._streaming is True
    assert node._seeded is False                 # gate held
    node._on_robot_state(_robot_state(is_homed=False))
    assert node._seeded is False                 # still held, no spam-y attempts
    node._on_robot_state(_robot_state(is_homed=True))
    assert node._seeded is True                  # seeds on the first homed frame


def test_accept_while_disarmed_is_loud():
    """An accepted move on a disarmed wire executes into a dropped stream (the
    2026-07-15 silent no-op battery: two moves 'accepted', realized peaks on
    /trajectory/diagnostics, zero motion, setpoints_sent frozen at 0). The
    accept response must carry the wire state so a harness prints it."""
    node = _traj_mode_node()
    node._on_link_status(_link_status(mpc_active='0'))
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is True
    assert 'wire DISARMED' in resp.message


def test_accept_while_armed_has_no_disarmed_marker():
    node = _traj_mode_node()
    node._on_link_status(_link_status(mpc_active='1'))
    resp = node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=2.0),
                                GoToPose.Response())
    assert resp.accepted is True
    assert 'DISARMED' not in resp.message
