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
COMMAND_ADDR = 'tcp://localhost:5555'   # bridge PUB → control SUB
TELEMETRY_ADDR = 'tcp://localhost:5556'  # control PUB → bridge SUB

# Topic prefixes for ZMQ PUB/SUB filtering
TOPIC_TARGET = b'target'
TOPIC_MODE = b'mode'
TOPIC_TELEMETRY = b'telem'
TOPIC_MOTOR_FB = b'motorfb'


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
                   loop_dt_s: float) -> dict:
    """Create a Telemetry message from the control loop."""
    return {
        'type': 'telemetry',
        'leg_pos': list(leg_positions),
        'leg_vel': list(leg_velocities),
        'cmd_torques': list(commanded_torques),
        'dt': loop_dt_s,
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
    """IPC endpoints for the standalone control process."""

    def __init__(self,
                 command_addr: str = COMMAND_ADDR,
                 telemetry_addr: str = TELEMETRY_ADDR):
        self._ctx = zmq.Context()

        # SUB socket — receives targets and mode commands from bridge
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.connect(command_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, TOPIC_TARGET)
        self._sub.setsockopt(zmq.SUBSCRIBE, TOPIC_MODE)
        self._sub.setsockopt(zmq.SUBSCRIBE, TOPIC_MOTOR_FB)
        self._sub.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub.setsockopt(zmq.CONFLATE, 1)   # keep only latest per topic

        # PUB socket — publishes telemetry to bridge
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(telemetry_addr)

        self._last_recv_time = time.monotonic()

    def recv_all(self) -> list[tuple[bytes, dict]]:
        """Non-blocking: receive all pending messages.

        Returns list of (topic, message_dict) tuples.  Empty list if nothing
        available.
        """
        messages = []
        while True:
            try:
                frames = self._sub.recv_multipart(flags=zmq.NOBLOCK)
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
        self._sub.close()
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
