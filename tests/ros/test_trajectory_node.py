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
from std_srvs.srv import Trigger
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

    def latency_build_move(state0, target, dur, limits, geom, *, shaper=None):
        plan, report = real_build_move(state0, target, dur, limits, geom,
                                       shaper=shaper)
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


def test_go_to_pose_default_request_is_lean_off():
    """A bare request (lean_gain=0.0) forces lean OFF — an unshaped move installs."""
    node = _traj_mode_node()
    resp = node._svc_go_to_pose(_go_to_pose_req(x=25.0, z=185.0, duration_s=0.0),
                                GoToPose.Response())
    assert resp.accepted is True
    assert node._lean_gain_active == 0.0
    # Plain (unshaped) TrajectoryPlan, not a _ShapedPlan.
    from jugglebot.motion.trajectory.shaping import _ShapedPlan
    assert not isinstance(node._active_plan, _ShapedPlan)


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
    """A negative override defers to JB_TRAJ_LEAN_GAIN (0.0 by default ⇒ OFF)."""
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
    """A TimedTarget request whose ROS arrival maps to ~lead_s ahead of now.

    Pins the node's ROS↔perf offset to perf_counter() at build time so
    arrival_perf = arrival_sec + offset ≈ now + lead_s (deterministic, independent
    of the mock clock returning 0)."""
    node._ros_to_perf_offset = time.perf_counter()
    req = TimedTarget.Request()
    req.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion())
    req.velocity_mm_s = Vector3(x=vx, y=vy, z=vz)
    req.arrival_time = _mk_ros_time(lead_s)
    req.hold_after = hold_after
    return req


def _dyn_target(node, x=0.0, y=0.0, z=20.0, lead_s=3.0, vx=0.0, vy=0.0, vz=0.0):
    """A catch/dynamic_target with a perf-domain arrival ~lead_s ahead of now.
    z is in the MPC offset convention (0 = active)."""
    msg = DynamicTargetCommand()
    msg.target_pos = Point(x=x, y=y, z=z)
    msg.target_quat = Quaternion()          # identity
    msg.target_vel = Vector3(x=vx, y=vy, z=vz)
    msg.arrival_time = time.perf_counter() + lead_s
    return msg


def _catch_mode_node():
    node = _node()
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data='CATCH'))
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


# ── catch/dynamic_target consumption ──────────────────────────

def test_dynamic_target_ignored_outside_catch_mode():
    node = _traj_mode_node()          # TRAJECTORY, not CATCH
    node._on_dynamic_target(_dyn_target(node, x=10.0))
    # No catch plan installed; still the seeded hold.
    assert node._active_plan.kind == 'hold'
    assert node.target_feedback_pub.published == []


def test_dynamic_target_installs_catch_plan_with_z_offset():
    """A catch target's z is an offset from active (0 = active); the installed plan's
    target z is lifted by the active-z (170) into the trajectory convention."""
    node = _catch_mode_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=20.0, lead_s=3.0))
    assert node._active_plan.kind == 'move'
    p_arr = node._active_plan.state_at(node._active_plan.total_duration)[0]
    assert np.allclose(p_arr[:3], [10.0, 0.0, 190.0], atol=1e-2)  # z = 20 + 170
    assert node._catch_arrival_perf is not None
    assert node.target_feedback_pub.published[-1].accepted is True
    assert node.target_feedback_pub.published[-1].source == 'catch'


def test_dynamic_target_reach_freeze_ignores_late_updates():
    """Inside the reach-freeze window a late target update is ignored (the committed
    reach is held) AND reported FROZEN — a distinct service-level code that
    distinguishes the freeze branch from a TOO_FAST feasibility reject."""
    node = _catch_mode_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=20.0, lead_s=3.0))
    committed = node._active_plan
    # Pin the committed arrival DETERMINISTICALLY inside the freeze window: now is past
    # (arrival − reach_freeze) and before (arrival + settle_hold).
    node._catch_arrival_perf = time.perf_counter() + 0.5 * node._catch_reach_freeze_s
    n_fb = len(node.target_feedback_pub.published)
    # A generous-lead update that WOULD supersede if not frozen — proves the freeze
    # (not a feasibility reject) is what blocks it.
    node._on_dynamic_target(_dyn_target(node, x=40.0, z=20.0, lead_s=3.0))
    assert node._active_plan is committed
    fb = node.target_feedback_pub.published[-1]
    assert fb.accepted is False
    assert fb.code == 'FROZEN'
    assert len(node.target_feedback_pub.published) == n_fb + 1


def test_dynamic_target_post_settle_supersedes_as_new_reach():
    """Once the committed arrival + settle-hold has fully passed, the freeze releases
    and a later target supersedes as a fresh reach (repeated catches — Phases 8/9),
    rather than being silently dropped forever behind a latched freeze."""
    node = _catch_mode_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=20.0, lead_s=3.0))
    first = node._active_plan
    # Pretend the committed arrival + settle-hold is now fully in the past.
    node._catch_arrival_perf = (time.perf_counter()
                                - node._catch_settle_hold_s - 0.1)
    node._on_dynamic_target(_dyn_target(node, x=25.0, z=20.0, lead_s=3.0))
    assert node._active_plan is not first
    assert node.target_feedback_pub.published[-1].accepted is True
    p_arr = node._active_plan.state_at(node._active_plan.total_duration)[0]
    assert np.allclose(p_arr[:3], [25.0, 0.0, 190.0], atol=1e-2)


def test_dynamic_target_past_arrival_rejected_too_fast():
    """A catch target whose arrival is already in the PAST → negative lead → loud
    TOO_FAST (never a crash), plan unchanged."""
    node = _catch_mode_node()
    committed = node._active_plan            # the seeded hold
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=20.0, lead_s=-1.0))
    assert node._active_plan is committed
    fb = node.target_feedback_pub.published[-1]
    assert fb.accepted is False
    assert fb.code == feas.TOO_FAST


def test_dynamic_target_supersedes_before_freeze():
    """Outside the freeze window a new catch target supersedes the prior (C2)."""
    node = _catch_mode_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=20.0, lead_s=3.0))
    first = node._active_plan
    node._on_dynamic_target(_dyn_target(node, x=25.0, z=20.0, lead_s=3.0))
    assert node._active_plan is not first
    p_arr = node._active_plan.state_at(node._active_plan.total_duration)[0]
    assert np.allclose(p_arr[:3], [25.0, 0.0, 190.0], atol=1e-2)


def test_leaving_catch_clears_freeze():
    node = _catch_mode_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=20.0, lead_s=3.0))
    assert node._catch_arrival_perf is not None
    node._on_control_mode(String(data='STANDBY'))
    assert node._catch_arrival_perf is None


@pytest.mark.parametrize('exit_mode', ['STANDBY', 'TRAJECTORY'])
def test_leaving_catch_mid_reach_installs_stop(exit_mode):
    """Leaving CATCH mid-reach (the documented abort = a switch away from CATCH)
    installs a graceful stop — the catch reach is silenced, not run on to the catch
    target — and clears the freeze. Both exit classes: CATCH→non-motion (STANDBY,
    via CATCH ∈ _MOTION_MODES) and CATCH→motion (TRAJECTORY, via the unconditional
    leaving-CATCH-mid-move branch — otherwise the reach would run on with go_to_pose
    BUSY-blocked behind it)."""
    node = _catch_mode_node()
    node._on_dynamic_target(_dyn_target(node, x=10.0, z=20.0, lead_s=3.0))
    catch_plan = node._active_plan
    assert catch_plan.kind == 'move'
    node._plan_t0 -= 1.0                       # advance into the reach (real velocity)
    assert node._active_move_in_flight() is True
    node._on_control_mode(String(data=exit_mode))
    assert node._active_plan is not catch_plan
    assert node._catch_arrival_perf is None
    end = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(end[1], 0.0, atol=1e-6)                       # ends at rest
    assert not np.allclose(end[0][:3], [10.0, 0.0, 190.0], atol=1.0)  # not the target


def test_entering_catch_mid_move_installs_stop():
    """Entering CATCH while a TRAJECTORY move is in flight installs a graceful stop
    (motion→motion doesn't trip leaving_motion_move) and resets the catch freeze, so
    the catch begins from a clean, non-jittering state."""
    node = _traj_mode_node()
    node._svc_go_to_pose(_go_to_pose_req(z=190.0, duration_s=3.0), GoToPose.Response())
    move_plan = node._active_plan
    assert move_plan.kind == 'move'
    node._plan_t0 -= 1.0
    assert node._active_move_in_flight() is True
    node._on_control_mode(String(data='CATCH'))
    assert node._active_plan is not move_plan
    assert node._catch_arrival_perf is None
    end = node._active_plan.state_at(node._active_plan.total_duration)
    assert np.allclose(end[1], 0.0, atol=1e-6)                       # ends at rest


def test_timed_target_excessive_lead_rejected_no_crash():
    """A clock-domain-confused lead (well over max_timed_lead_s) is loudly rejected via
    the service — never a crash / MemoryError — and the plan is unchanged."""
    node = _traj_mode_node()
    committed = node._active_plan
    resp = node._svc_timed_target(_timed_req(node, z=185.0, lead_s=61.0),
                                  TimedTarget.Response())
    assert resp.accepted is False
    assert resp.code == feas.TOO_FAST
    assert node._active_plan is committed


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


# ── clock conversion (single point) ───────────────────────────

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
