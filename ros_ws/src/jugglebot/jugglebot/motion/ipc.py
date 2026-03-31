"""ZeroMQ-based IPC layer for the motion control system.

Provides communication between the motor guard process, the ROS2 bridge
nodes, and the MPC process.  Messages are serialised with msgpack for speed.

Architecture::

    ROS2 Motion Bridge                   Motor Guard (500 Hz)
    ──────────────────                  ────────────────────
    BridgeIPC                           MotorGuardIPC
      PUB  ─── tcp://localhost:5555 ──►  SUB   (mode cmds, motor feedback)
      SUB  ◄── tcp://localhost:5556 ───  PUB   (telemetry)

    HardwarePlant (MPC process)
      PUB  ─── tcp://localhost:5557 ──►  SUB   (mpc commands + fallback enable)
      SUB  ◄── tcp://localhost:5556 ───        (telemetry, shared)

    ROS2 MPC Bridge                      MPC Process
    ───────────────                     ───────────
    MpcBridgeIPC                        MpcTargetIPC
      PUB  ─── tcp://localhost:5558 ──►  SUB   (target poses, mode)

    MPC Process                          Catch Coordinator Node
    ───────────                         ──────────────────────
    TargetFeedbackPub                   TargetFeedbackSub
      PUB  ─── tcp://localhost:5559 ──►  SUB   (target accept/reject)

Message types:
  - ModeCommand:   enable, disable, estop, fault, etc.
  - Telemetry:     motor guard output (leg states, timing, torques)
  - MotorFeedback: encoder positions/velocities/currents from CAN node
  - MPCCommand:    pre-computed motor commands from external MPC controller
  - MPCTarget:     target pose for the MPC to track (from any input source)
  - MPCMode:       active input mode forwarded to the MPC process
  - TargetFeedback: accept/reject decision for catch targets (MPC → coordinator)

Lifecycle authority:
  - The ROS2 motion bridge on :5555 is the primary authority for motor guard
    enable/disable/estop.  It sends disable+enable on every mode transition.
  - HardwarePlant on :5557 sends a fallback enable for direct hardware mode
    (no ROS2).  When the bridge is running, this resolves to a no-op at the
    motor guard (enable-when-already-ENABLED is idempotent).

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
COMMAND_ADDR = 'tcp://127.0.0.1:5555'       # bridge PUB → motor guard SUB
TELEMETRY_ADDR = 'tcp://127.0.0.1:5556'     # motor guard PUB → bridge SUB
MPC_COMMAND_ADDR = 'tcp://127.0.0.1:5557'   # HardwarePlant PUB → motor guard SUB
MPC_TARGET_ADDR = 'tcp://127.0.0.1:5558'    # mpc_bridge PUB → MPC process SUB
MPC_FEEDBACK_ADDR = 'tcp://127.0.0.1:5559'  # MPC process PUB → catch coordinator SUB

# Topic prefixes for ZMQ PUB/SUB filtering
TOPIC_MODE = b'mode'
TOPIC_TELEMETRY = b'telem'
TOPIC_MOTOR_FB = b'motorfb'
TOPIC_MPC_CMD = b'mpccmd'       # pre-computed motor commands from MPC
TOPIC_MPC_TARGET = b'mpctgt'    # target pose for MPC to track
TOPIC_MPC_MODE = b'mpcmode'     # active input mode for MPC process
TOPIC_TARGET_FB = b'tgtfb'      # target accept/reject feedback from MPC


# ---------------------------------------------------------------------------
# Message constructors
# ---------------------------------------------------------------------------

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
    }


def make_telemetry(leg_positions: list | tuple,
                   leg_velocities: list | tuple,
                   leg_torques: list | tuple,
                   loop_dt_s: float,
                   cond_number: float | None = None,
                   workspace_status: str | None = None,
                   workspace_speed_scale: float | None = None,
                   tracking_error_mm: list | tuple | None = None,
                   fault_state: str | None = None,
                   motor_pos: list | tuple | None = None,
                   motor_vel: list | tuple | None = None,
                   motor_cur: list | tuple | None = None,
                   timestamp: float | None = None) -> dict:
    """Create a Telemetry message from the motor guard.

    Parameters
    ----------
    leg_torques : per-motor feedforward torques (Nm)
    cond_number : Jacobian condition number at current pose, or None
    workspace_status : 'ok', 'soft', or 'hard', or None
    workspace_speed_scale : workspace-imposed speed scale 0.0-1.0, or None
    tracking_error_mm : per-leg position tracking error (mm), or None
    fault_state : fault description string, or None
    motor_pos : 6 actual motor positions (rev) from encoder feedback, or None
    motor_vel : 6 actual motor velocities (rev/s) from encoder feedback, or None
    motor_cur : 6 actual motor currents (A) from encoder feedback, or None
    timestamp : time.perf_counter() at message creation, or None
    """
    msg = {
        'type': 'telemetry',
        'leg_pos': list(leg_positions),
        'leg_vel': list(leg_velocities),
        'leg_torques': list(leg_torques),
        'dt': loop_dt_s,
    }
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
    msg['timestamp'] = timestamp
    return msg


def make_motor_feedback(positions: list | tuple,
                        velocities: list | tuple,
                        currents: list | tuple) -> dict:
    """Create a MotorFeedback message from the bridge (CAN node data)."""
    return {
        'type': 'motor_feedback',
        'pos': list(positions),
        'vel': list(velocities),
        'cur': list(currents),
    }


def make_mpc_command(ext_mm: list | tuple,
                     pose_6dof: list | tuple,
                     motor_rev: list | tuple | None = None,
                     vel_mm_s: list | tuple | None = None,
                     acc_mm_s2: list | tuple | None = None,
                     torque_Nm: list | tuple | None = None,
                     seq: int = 0) -> dict:
    """Create an MPC command message.

    Sent by the external MPC controller (HardwarePlant) directly to the
    motor guard.  The motor guard uses motor_rev directly as commanded
    positions (matching ODrive encoder convention where 0 = STOW), and
    applies the full safety pipeline.

    Parameters
    ----------
    ext_mm : 6 IK-convention leg extensions (mm) -- for workspace checks
    pose_6dof : [x, y, z, rx, ry, rz] in mm, rad -- Cartesian pose for
        condition-number checks (from MPC predicted_poses[0])
    motor_rev : 6 absolute motor positions (rev) in ODrive convention
        (0 = STOW).  Includes the stow offset.  If None, the motor guard
        falls back to ``extensions_mm_to_revs(ext_mm)`` (no stow offset --
        only correct for unit tests / sim).
    vel_mm_s : 6 leg velocities (mm/s), or None for zeros
    acc_mm_s2 : 6 leg accelerations (mm/s²), or None for zeros.
        Used by the motor guard for cubic interpolation between 40 Hz
        MPC commands (eliminates position step at command boundaries).
    torque_Nm : 6 motor torques (Nm), or None for zeros
    seq : monotonic sequence number (for debugging / drop detection)
    """
    msg = {
        'type': 'mpc_cmd',
        'ext_mm': list(ext_mm),
        'pose_6dof': list(pose_6dof),
        'seq': seq,
    }
    if motor_rev is not None:
        msg['motor_rev'] = list(motor_rev)
    if vel_mm_s is not None:
        msg['vel_mm_s'] = list(vel_mm_s)
    if acc_mm_s2 is not None:
        msg['acc_mm_s2'] = list(acc_mm_s2)
    if torque_Nm is not None:
        msg['torque_Nm'] = list(torque_Nm)
    return msg


def make_mpc_target(target_pose: list | tuple,
                    arrival_time: float | None = None,
                    target_twist: list | tuple | None = None,
                    source: str = '') -> dict:
    """Create an MPC target message.

    Sent by the MPC bridge node to the MPC process.  Contains the target
    pose that the MPC should track, optionally with an arrival deadline
    and a desired twist at arrival.

    Parameters
    ----------
    target_pose : [x, y, z, rx, ry, rz] in mm / rad (rotation vector)
    arrival_time : absolute time (perf_counter) to arrive, or None = ASAP
    target_twist : [vx, vy, vz, wx, wy, wz] in mm/s / rad/s, or None = hold
    source : input source identifier ('spacemouse', 'gui', 'shell', 'catch')
    """
    msg = {
        'type': 'mpc_target',
        'target_pose': list(target_pose),
        'source': source,
    }
    if arrival_time is not None:
        msg['arrival_time'] = arrival_time
    if target_twist is not None:
        msg['target_twist'] = list(target_twist)
    return msg


def make_mpc_mode(mode: str) -> dict:
    """Create an MPC mode message.

    Sent by the MPC bridge node to inform the MPC process of the current
    active input mode.

    Parameters
    ----------
    mode : 'spacemouse', 'gui', 'shell', 'catch', or 'disabled'
    """
    return {'type': 'mpc_mode', 'mode': mode}


def make_target_feedback(arrival_time: float,
                         accepted: bool,
                         source: str = '',
                         violations: list | None = None) -> dict:
    """Create a target feedback message (accept/reject).

    Published by the MPC process after evaluating a catch target.  Consumed
    by the catch coordinator node to maintain its blacklist.

    Parameters
    ----------
    arrival_time : float
        The ``arrival_time`` from the original target request (used to
        correlate feedback with the submitted target).
    accepted : bool
        True if the MPC solver converged and the predicted trajectory
        reaches the target by ``arrival_time``.  False if the solver
        failed or the target is unreachable in time.
    source : str
        Input source identifier (e.g. 'catch').
    violations : list of str or None
        Human-readable reasons for rejection (e.g. ['solver_failed',
        'unreachable_in_time']).  Empty/None when accepted.
    """
    msg = {
        'type': 'target_feedback',
        'arrival_time': arrival_time,
        'accepted': accepted,
        'source': source,
    }
    if violations:
        msg['violations'] = violations
    return msg


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
# Motor Guard (standalone process) side
# ---------------------------------------------------------------------------

class MotorGuardIPC:
    """IPC endpoints for the motor guard process.

    Uses four SUB sockets to isolate message types:
      - ``_sub_mode``: mode commands from the bridge (no CONFLATE)
      - ``_sub_motor_fb``: motor feedback from CAN node (CONFLATE=1)
      - ``_sub_mpc_cmd``: MPC commands (CONFLATE=1, separate port)
      - ``_sub_mpc_mode``: mode commands from HardwarePlant (no CONFLATE,
        separate port)

    Motor feedback gets its own CONFLATE socket so that high-frequency
    feedback (100 Hz) cannot overwrite mode messages and vice-versa.
    Without this separation, ZMQ CONFLATE keeps only the single latest
    message *regardless of topic prefix*, silently dropping whichever
    message arrived first.

    Mode commands from HardwarePlant (enable/disable/estop) get their own
    non-CONFLATE socket on the MPC port for the same reason — mixing them
    with MPC commands on a single CONFLATE socket would silently drop
    whichever message arrived first in the same poll window.

    The MPC command socket listens on a separate port (MPC_COMMAND_ADDR)
    because the bridge already ``bind()``s PUB on COMMAND_ADDR -- ZMQ does
    not support two PUB ``bind()``s on the same address.
    """

    def __init__(self,
                 command_addr: str = COMMAND_ADDR,
                 telemetry_addr: str = TELEMETRY_ADDR,
                 mpc_command_addr: str = MPC_COMMAND_ADDR):
        self._ctx = zmq.Context()

        # SUB socket for motor feedback -- CONFLATE keeps only latest
        self._sub_motor_fb = self._ctx.socket(zmq.SUB)
        self._sub_motor_fb.connect(command_addr)
        self._sub_motor_fb.setsockopt(zmq.SUBSCRIBE, TOPIC_MOTOR_FB)
        self._sub_motor_fb.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_motor_fb.setsockopt(zmq.CONFLATE, 1)   # keep only latest

        # SUB socket for mode commands -- no CONFLATE, every command delivered
        self._sub_mode = self._ctx.socket(zmq.SUB)
        self._sub_mode.connect(command_addr)
        self._sub_mode.setsockopt(zmq.SUBSCRIBE, TOPIC_MODE)
        self._sub_mode.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_mode.setsockopt(zmq.RCVHWM, 64)   # bound queue size

        # SUB socket for MPC commands -- CONFLATE, separate port.
        self._sub_mpc_cmd = self._ctx.socket(zmq.SUB)
        self._sub_mpc_cmd.connect(mpc_command_addr)
        self._sub_mpc_cmd.setsockopt(zmq.SUBSCRIBE, TOPIC_MPC_CMD)
        self._sub_mpc_cmd.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_mpc_cmd.setsockopt(zmq.CONFLATE, 1)   # keep only latest

        # SUB socket for mode commands from HardwarePlant -- no CONFLATE,
        # every enable/disable/estop is delivered.  Separate from _sub_mpc_cmd
        # because CONFLATE keeps only the single latest message regardless of
        # topic prefix — mixing mode + MPC on one CONFLATE socket silently
        # drops whichever arrives first in the same poll window.
        self._sub_mpc_mode = self._ctx.socket(zmq.SUB)
        self._sub_mpc_mode.connect(mpc_command_addr)
        self._sub_mpc_mode.setsockopt(zmq.SUBSCRIBE, TOPIC_MODE)
        self._sub_mpc_mode.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub_mpc_mode.setsockopt(zmq.RCVHWM, 64)   # bound queue size

        # PUB socket -- publishes telemetry to bridge
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(telemetry_addr)

        self._last_recv_time = time.monotonic()

    def recv_all(self) -> list[tuple[bytes, dict]]:
        """Non-blocking: receive all pending messages.

        Drains mode commands first (critical), then MPC commands, then
        motor feedback.  Returns list of (topic, message_dict) tuples.
        Empty list if nothing available.
        """
        messages = []
        # Drain mode commands first -- these are rare but critical.
        # Both bridge mode (_sub_mode) and HardwarePlant mode (_sub_mpc_mode)
        # are drained before MPC commands.
        for sub in (self._sub_mode, self._sub_mpc_mode, self._sub_mpc_cmd,
                    self._sub_motor_fb):
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
        self._sub_motor_fb.close()
        self._sub_mode.close()
        self._sub_mpc_cmd.close()
        self._sub_mpc_mode.close()
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

        # PUB socket -- sends mode commands and motor feedback to motor guard
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(command_addr)

        # SUB socket -- receives telemetry from motor guard (CONFLATE: latest only)
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.connect(telemetry_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, TOPIC_TELEMETRY)
        self._sub.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub.setsockopt(zmq.CONFLATE, 1)

    def send_mode_command(self, msg: dict) -> None:
        """Send a ModeCommand message to the motor guard."""
        self._pub.send_multipart(_pack(TOPIC_MODE, msg), flags=zmq.NOBLOCK)

    def send_motor_feedback(self, msg: dict) -> None:
        """Send MotorFeedback to the motor guard."""
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


# ---------------------------------------------------------------------------
# MPC Bridge (ROS2 node) side — publishes targets to MPC process
# ---------------------------------------------------------------------------

class MpcBridgeIPC:
    """IPC endpoint for the MPC bridge node (PUB side).

    Publishes target poses and mode changes to the MPC process on
    MPC_TARGET_ADDR (:5558).
    """

    def __init__(self, target_addr: str = MPC_TARGET_ADDR):
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(target_addr)

    def send_target(self, msg: dict) -> None:
        """Publish an MPC target command."""
        self._pub.send_multipart(
            _pack(TOPIC_MPC_TARGET, msg), flags=zmq.NOBLOCK)

    def send_mode(self, msg: dict) -> None:
        """Publish an MPC mode change."""
        self._pub.send_multipart(
            _pack(TOPIC_MPC_MODE, msg), flags=zmq.NOBLOCK)

    def close(self) -> None:
        self._pub.close()
        self._ctx.term()


# ---------------------------------------------------------------------------
# MPC Process (standalone) side — receives targets from bridge
# ---------------------------------------------------------------------------

class MpcTargetIPC:
    """IPC endpoint for the MPC process (SUB side).

    Uses two SUB sockets on MPC_TARGET_ADDR (:5558):
      - ``_sub_target``: target poses (CONFLATE=1, latest wins)
      - ``_sub_mode``: mode changes (no CONFLATE, every message delivered)

    Separate sockets prevent CONFLATE from silently dropping mode messages
    when a target arrives in the same inter-poll window (the same pattern
    used by MotorGuardIPC).
    """

    def __init__(self, target_addr: str = MPC_TARGET_ADDR):
        self._ctx = zmq.Context()

        # CONFLATE SUB for targets — only the latest matters
        self._sub_target = self._ctx.socket(zmq.SUB)
        self._sub_target.connect(target_addr)
        self._sub_target.setsockopt(zmq.SUBSCRIBE, TOPIC_MPC_TARGET)
        self._sub_target.setsockopt(zmq.RCVTIMEO, 0)   # non-blocking
        self._sub_target.setsockopt(zmq.CONFLATE, 1)    # keep only latest

        # Non-CONFLATE SUB for mode — every transition matters
        self._sub_mode = self._ctx.socket(zmq.SUB)
        self._sub_mode.connect(target_addr)
        self._sub_mode.setsockopt(zmq.SUBSCRIBE, TOPIC_MPC_MODE)
        self._sub_mode.setsockopt(zmq.RCVTIMEO, 0)     # non-blocking
        self._sub_mode.setsockopt(zmq.RCVHWM, 64)      # bound queue size

    def recv_all(self) -> list[tuple[bytes, dict]]:
        """Non-blocking: receive all pending messages.

        Drains mode messages first (critical), then the latest target.
        Returns list of (topic, message_dict) tuples.
        """
        messages = []
        for sub in (self._sub_mode, self._sub_target):
            while True:
                try:
                    frames = sub.recv_multipart(flags=zmq.NOBLOCK)
                    topic, msg = _unpack(frames)
                    messages.append((topic, msg))
                except zmq.Again:
                    break
        return messages

    def close(self) -> None:
        self._sub_target.close()
        self._sub_mode.close()
        self._ctx.term()


# ---------------------------------------------------------------------------
# Target Feedback (MPC process → catch coordinator)
# ---------------------------------------------------------------------------

class TargetFeedbackPub:
    """PUB endpoint for target accept/reject feedback.

    Used by the MPC process to inform the catch coordinator whether a
    catch target was accepted (solver converged, target reachable by
    arrival_time) or rejected (solver failed / unreachable).

    Binds on MPC_FEEDBACK_ADDR (:5559).
    """

    def __init__(self, feedback_addr: str = MPC_FEEDBACK_ADDR):
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(feedback_addr)

    def send(self, msg: dict) -> None:
        """Publish a target feedback message."""
        self._pub.send_multipart(
            _pack(TOPIC_TARGET_FB, msg), flags=zmq.NOBLOCK)

    def close(self) -> None:
        self._pub.close()
        self._ctx.term()


class TargetFeedbackSub:
    """SUB endpoint for target accept/reject feedback.

    Used by the catch coordinator node to receive accept/reject decisions
    from the MPC process.  Connects to MPC_FEEDBACK_ADDR (:5559).
    """

    def __init__(self, feedback_addr: str = MPC_FEEDBACK_ADDR):
        self._ctx = zmq.Context()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.connect(feedback_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, TOPIC_TARGET_FB)
        self._sub.setsockopt(zmq.RCVTIMEO, 0)   # non-blocking
        self._sub.setsockopt(zmq.RCVHWM, 64)

    def recv(self) -> dict | None:
        """Non-blocking: receive the next feedback message, or None."""
        try:
            frames = self._sub.recv_multipart(flags=zmq.NOBLOCK)
            _, msg = _unpack(frames)
            return msg
        except zmq.Again:
            return None

    def close(self) -> None:
        self._sub.close()
        self._ctx.term()
