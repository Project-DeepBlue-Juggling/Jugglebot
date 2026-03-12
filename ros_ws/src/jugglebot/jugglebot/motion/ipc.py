"""ZeroMQ-based IPC layer for the motion control system.

Provides communication between the standalone control process and the
ROS2 bridge node.  Messages are serialised with msgpack for speed.

Architecture::

    ROS2 Bridge                          Control Process
    ──────────                          ───────────────
    BridgeIPC                           ControlProcessIPC
      PUB  ─── tcp://localhost:5555 ──►  SUB   (targets, commands)
      SUB  ◄── tcp://localhost:5556 ───  PUB   (telemetry)

Message types:
  - TargetState:   desired platform pose/twist/accel from the planner
  - ModeCommand:   enable, disable, estop, set_gains, etc.
  - Telemetry:     control loop output (leg states, timing, torques)
  - MotorFeedback: encoder positions/velocities/currents from CAN node

No ROS2 dependency.  Requires: pyzmq, msgpack.
"""

from __future__ import annotations

import logging
import time
from typing import Any

import msgpack
import zmq

logger = logging.getLogger(__name__)

# Default IPC addresses
COMMAND_ADDR = 'tcp://127.0.0.1:5555'   # bridge PUB → control SUB
TELEMETRY_ADDR = 'tcp://127.0.0.1:5556'  # control PUB → bridge SUB

# Topic prefixes for ZMQ PUB/SUB filtering
TOPIC_TARGET = b'target'
TOPIC_MODE = b'mode'
TOPIC_TELEMETRY = b'telem'
TOPIC_MOTOR_FB = b'motorfb'
TOPIC_TRAJECTORY = b'traj'
TOPIC_DYN_TARGET = b'dyntgt'


# ---------------------------------------------------------------------------
# Message constructors
# ---------------------------------------------------------------------------

def make_target_state(pos: list | tuple,
                      rot_quat: list | tuple,
                      twist: list | tuple | None = None,
                      accel: list | tuple | None = None) -> dict:
    """Create a TargetState message.

    Parameters
    ----------
    pos : [x, y, z] platform offset from home (mm)
    rot_quat : [w, x, y, z] quaternion (platform → base)
    twist : [vx, vy, vz, ωx, ωy, ωz] or None (mm/s, rad/s)
    accel : [ax, ay, az, αx, αy, αz] or None (mm/s², rad/s²)
    """
    return {
        'type': 'target',
        'pos': list(pos),
        'rot': list(rot_quat),
        'twist': list(twist) if twist is not None else [0.0] * 6,
        'accel': list(accel) if accel is not None else [0.0] * 6,
        'ts': time.time(),
    }


def make_mode_command(command: str, **params) -> dict:
    """Create a ModeCommand message.

    Parameters
    ----------
    command : 'enable', 'disable', 'estop', 'set_gains', etc.
    **params : command-specific parameters (e.g. kp=10.0)
    """
    return {
        'type': 'mode',
        'cmd': command,
        'params': params,
        'ts': time.time(),
    }


def make_telemetry(leg_positions: list | tuple,
                   leg_velocities: list | tuple,
                   commanded_torques: list | tuple,
                   loop_dt_s: float,
                   ff_torques: list | tuple | None = None,
                   pd_torques: list | tuple | None = None,
                   traj_state: str | None = None,
                   traj_progress: float | None = None,
                   cond_number: float | None = None,
                   workspace_status: str | None = None,
                   workspace_speed_scale: float | None = None,
                   tracking_error_mm: list | tuple | None = None,
                   fault_state: str | None = None,
                   motor_pos: list | tuple | None = None,
                   motor_vel: list | tuple | None = None,
                   motor_cur: list | tuple | None = None,
                   slew_limited: bool = False) -> dict:
    """Create a Telemetry message from the control loop.

    Parameters
    ----------
    ff_torques : per-motor gravity feedforward torques (Nm), or None
    pd_torques : per-motor PD feedback torques (Nm), or None
    traj_state : trajectory state string ('idle', 'executing', 'complete'), or None
    traj_progress : trajectory progress 0.0-1.0, or None
    cond_number : Jacobian condition number at current pose, or None
    workspace_status : 'ok', 'soft', or 'hard', or None
    workspace_speed_scale : workspace-imposed speed scale 0.0-1.0, or None
    tracking_error_mm : per-leg position tracking error (mm), or None
    fault_state : fault description string, or None
    motor_pos : 6 actual motor positions (rev) from encoder feedback, or None
    motor_vel : 6 actual motor velocities (rev/s) from encoder feedback, or None
    motor_cur : 6 actual motor currents (A) from encoder feedback, or None
    slew_limited : True if the slew limiter is actively clamping this cycle
    """
    msg = {
        'type': 'telemetry',
        'leg_pos': list(leg_positions),
        'leg_vel': list(leg_velocities),
        'cmd_torques': list(commanded_torques),
        'dt': loop_dt_s,
        'ts': time.time(),
    }
    if ff_torques is not None:
        msg['ff_torques'] = list(ff_torques)
    if pd_torques is not None:
        msg['pd_torques'] = list(pd_torques)
    if traj_state is not None:
        msg['traj_state'] = traj_state
    if traj_progress is not None:
        msg['traj_progress'] = traj_progress
    if cond_number is not None:
        msg['cond_number'] = cond_number
    if workspace_status is not None:
        msg['workspace_status'] = workspace_status
    if workspace_speed_scale is not None:
        msg['workspace_speed_scale'] = workspace_speed_scale
    if tracking_error_mm is not None:
        msg['tracking_error_mm'] = list(tracking_error_mm)
    if fault_state is not None:
        msg['fault_state'] = fault_state
    if motor_pos is not None:
        msg['motor_pos'] = list(motor_pos)
    if motor_vel is not None:
        msg['motor_vel'] = list(motor_vel)
    if motor_cur is not None:
        msg['motor_cur'] = list(motor_cur)
    if slew_limited:
        msg['slew_limited'] = True
    return msg


def make_trajectory_command(
        start_pose: list | tuple,
        start_twist: list | tuple,
        start_accel: list | tuple,
        end_pose: list | tuple,
        end_twist: list | tuple,
        end_accel: list | tuple,
        duration: float,
        speed_scale: float = 1.0) -> dict:
    """Create a trajectory command message.

    Parameters
    ----------
    start_pose : [x, y, z, rx, ry, rz] in mm, rad
    start_twist : [vx, vy, vz, wx, wy, wz] in mm/s, rad/s
    start_accel : [ax, ay, az, alphax, alphay, alphaz] in mm/s^2, rad/s^2
    end_pose, end_twist, end_accel : same as start
    duration : trajectory duration in seconds (before speed scaling)
    speed_scale : 0.0-1.0, uniformly scales velocities/accelerations
    """
    return {
        'type': 'trajectory',
        'start_pose': list(start_pose),
        'start_twist': list(start_twist),
        'start_accel': list(start_accel),
        'end_pose': list(end_pose),
        'end_twist': list(end_twist),
        'end_accel': list(end_accel),
        'duration': duration,
        'speed_scale': speed_scale,
        'ts': time.time(),
    }


def make_dynamic_target_command(
        target_pos: list | tuple,
        target_quat: list | tuple,
        target_vel: list | tuple,
        arrival_time: float,
        speed_scale: float = 1.0) -> dict:
    """Create a dynamic target command message.

    The caller specifies only the desired end state.  The control process
    automatically samples the current platform state as the start and plans
    a quintic trajectory with feasibility checking.

    Parameters
    ----------
    target_pos : [x, y, z] platform offset from home (mm)
    target_quat : [w, x, y, z] quaternion orientation
    target_vel : [vx, vy, vz] linear velocity at target (mm/s).
        Angular velocity is always zero.
    arrival_time : absolute arrival time (perf_counter timestamp)
    speed_scale : 0.0-1.0, uniformly scales velocities/accelerations
    """
    return {
        'type': 'dynamic_target',
        'target_pos': list(target_pos),
        'target_quat': list(target_quat),
        'target_vel': list(target_vel),
        'arrival_time': arrival_time,
        'speed_scale': speed_scale,
        'ts': time.time(),
    }


def make_motor_feedback(positions: list | tuple,
                        velocities: list | tuple,
                        currents: list | tuple) -> dict:
    """Create a MotorFeedback message from the bridge (CAN node data)."""
    return {
        'type': 'motor_feedback',
        'pos': list(positions),
        'vel': list(velocities),
        'cur': list(currents),
        'ts': time.time(),
    }


# ---------------------------------------------------------------------------
# Serialisation
# ---------------------------------------------------------------------------

def _pack(topic: bytes, msg: dict) -> list[bytes]:
    """Serialise a message for ZMQ multipart send."""
    return [topic, msgpack.packb(msg, use_bin_type=True)]


def _unpack(frames: list[bytes]) -> tuple[bytes, dict]:
    """Deserialise a ZMQ multipart message."""
    topic = frames[0]
    msg = msgpack.unpackb(frames[1], raw=False)
    return topic, msg


# ---------------------------------------------------------------------------
# Control Process side
# ---------------------------------------------------------------------------

class ControlProcessIPC:
    """IPC endpoints for the standalone control process.

    Uses three SUB sockets to isolate message types:
      - ``_sub_mode``: mode/trajectory/dynamic-target commands (no CONFLATE)
      - ``_sub_target``: platform pose targets (CONFLATE=1)
      - ``_sub_motor_fb``: motor feedback from CAN node (CONFLATE=1)

    Targets and motor feedback each get their own CONFLATE socket so that
    high-frequency motor feedback (100 Hz) cannot overwrite target messages
    and vice-versa.  Without this separation, ZMQ CONFLATE keeps only the
    single latest message *regardless of topic prefix*, silently dropping
    whichever message arrived first.
    """

    def __init__(self,
                 command_addr: str = COMMAND_ADDR,
                 telemetry_addr: str = TELEMETRY_ADDR):
        self._ctx = zmq.Context()

        # SUB socket for pose targets — CONFLATE keeps only latest
        self._sub_target = self._ctx.socket(zmq.SUB)
        self._sub_target.connect(command_addr)
        self._sub_target.setsockopt(zmq.SUBSCRIBE, TOPIC_TARGET)
        self._sub_target.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_target.setsockopt(zmq.CONFLATE, 1)   # keep only latest

        # SUB socket for motor feedback — CONFLATE keeps only latest
        self._sub_motor_fb = self._ctx.socket(zmq.SUB)
        self._sub_motor_fb.connect(command_addr)
        self._sub_motor_fb.setsockopt(zmq.SUBSCRIBE, TOPIC_MOTOR_FB)
        self._sub_motor_fb.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_motor_fb.setsockopt(zmq.CONFLATE, 1)   # keep only latest

        # SUB socket for mode/trajectory/dynamic-target commands — no CONFLATE,
        # every command delivered
        self._sub_mode = self._ctx.socket(zmq.SUB)
        self._sub_mode.connect(command_addr)
        self._sub_mode.setsockopt(zmq.SUBSCRIBE, TOPIC_MODE)
        self._sub_mode.setsockopt(zmq.SUBSCRIBE, TOPIC_TRAJECTORY)
        self._sub_mode.setsockopt(zmq.SUBSCRIBE, TOPIC_DYN_TARGET)
        self._sub_mode.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_mode.setsockopt(zmq.RCVHWM, 64)   # bound queue size

        # PUB socket — publishes telemetry to bridge
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(telemetry_addr)

        self._last_recv_time = time.monotonic()

    def recv_all(self) -> list[tuple[bytes, dict]]:
        """Non-blocking: receive all pending messages.

        Drains mode commands first (critical), then targets, then motor
        feedback.  Returns list of (topic, message_dict) tuples.  Empty
        list if nothing available.
        """
        messages = []
        # Drain mode commands first — these are rare but critical
        for sub in (self._sub_mode, self._sub_target, self._sub_motor_fb):
            while True:
                try:
                    frames = sub.recv_multipart(flags=zmq.NOBLOCK)
                    topic, msg = _unpack(frames)
                    messages.append((topic, msg))
                    self._last_recv_time = time.monotonic()
                except zmq.Again:
                    break
        return messages

    def send_telemetry(self, msg: dict) -> None:
        """Publish telemetry to the bridge."""
        self._pub.send_multipart(_pack(TOPIC_TELEMETRY, msg), flags=zmq.NOBLOCK)

    @property
    def seconds_since_last_recv(self) -> float:
        """Seconds since any message was last received from the bridge."""
        return time.monotonic() - self._last_recv_time

    def close(self) -> None:
        self._sub_target.close()
        self._sub_motor_fb.close()
        self._sub_mode.close()
        self._pub.close()
        self._ctx.term()


# ---------------------------------------------------------------------------
# Bridge (ROS2) side
# ---------------------------------------------------------------------------

class BridgeIPC:
    """IPC endpoints for the ROS2 bridge node."""

    def __init__(self,
                 command_addr: str = COMMAND_ADDR,
                 telemetry_addr: str = TELEMETRY_ADDR):
        self._ctx = zmq.Context()

        # PUB socket — sends targets and mode commands to control process
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(command_addr)

        # SUB socket — receives telemetry from control process
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.connect(telemetry_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, TOPIC_TELEMETRY)
        self._sub.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub.setsockopt(zmq.CONFLATE, 1)

    def send_target(self, msg: dict) -> None:
        """Send a TargetState message to the control process."""
        self._pub.send_multipart(_pack(TOPIC_TARGET, msg), flags=zmq.NOBLOCK)

    def send_mode_command(self, msg: dict) -> None:
        """Send a ModeCommand message to the control process."""
        self._pub.send_multipart(_pack(TOPIC_MODE, msg), flags=zmq.NOBLOCK)

    def send_trajectory_command(self, msg: dict) -> None:
        """Send a trajectory command to the control process."""
        self._pub.send_multipart(_pack(TOPIC_TRAJECTORY, msg), flags=zmq.NOBLOCK)

    def send_dynamic_target(self, msg: dict) -> None:
        """Send a dynamic target command to the control process."""
        self._pub.send_multipart(_pack(TOPIC_DYN_TARGET, msg), flags=zmq.NOBLOCK)

    def send_motor_feedback(self, msg: dict) -> None:
        """Send MotorFeedback to the control process."""
        self._pub.send_multipart(_pack(TOPIC_MOTOR_FB, msg), flags=zmq.NOBLOCK)

    def recv_telemetry(self) -> dict | None:
        """Non-blocking: receive the latest telemetry message, or None."""
        try:
            frames = self._sub.recv_multipart(flags=zmq.NOBLOCK)
            _, msg = _unpack(frames)
            return msg
        except zmq.Again:
            return None

    def close(self) -> None:
        self._pub.close()
        self._sub.close()
        self._ctx.term()
