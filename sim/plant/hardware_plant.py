"""Real hardware plant — sends MPC commands to the control loop via ZeroMQ.

Implements the PlantInterface so the MPC controller can swap between
MuJoCoPlant (simulation) and HardwarePlant (real robot) transparently.

Architecture:
  - PUB socket on MPC_COMMAND_ADDR (:5557): sends mpc_cmd and mode messages
  - SUB socket on TELEMETRY_ADDR (:5556): receives telemetry from control loop

The control loop's safety pipeline (slew limiter, workspace checks, overspeed,
tracking error, staleness watchdog) remains active — this plant only converts
and transmits, never bypasses safety.
"""

from __future__ import annotations

import logging
import sys
import time

import numpy as np

# Import IPC from the production motion package.
# Callers must ensure the ros_ws package is on sys.path.
from jugglebot.motion.ipc import (
    MPC_COMMAND_ADDR,
    TELEMETRY_ADDR,
    TOPIC_MPC_CMD,
    TOPIC_MODE,
    TOPIC_TELEMETRY,
    make_mpc_command,
    make_mode_command,
    _pack,
    _unpack,
)
from jugglebot.motion.conversions import revs_to_extensions_mm
from jugglebot.motion.geometry import StewartGeometry

from .interface import PlantInterface, PlantState

try:
    import msgpack
    import zmq
except ImportError as e:
    raise ImportError(
        f"HardwarePlant requires pyzmq and msgpack: {e}. "
        "These are Jetson-only dependencies (not needed on Windows for sim)."
    ) from e

logger = logging.getLogger(__name__)

# Time to wait for ZMQ PUB socket to connect before sending first message
_ZMQ_CONNECT_SETTLE_S = 0.1


class HardwarePlant(PlantInterface):
    """Stewart platform plant that communicates with real hardware via IPC.

    Sends pre-computed leg extensions to the control loop's MPC pass-through
    mode.  Reads motor feedback from control loop telemetry.

    Parameters
    ----------
    geom : StewartGeometry
        Robot geometry (for unit conversions in get_state).
    mpc_command_addr : str
        ZeroMQ address to bind PUB socket for MPC commands.
    telemetry_addr : str
        ZeroMQ address to connect SUB socket for telemetry.
    """

    def __init__(
        self,
        geom: StewartGeometry | None = None,
        mpc_command_addr: str = MPC_COMMAND_ADDR,
        telemetry_addr: str = TELEMETRY_ADDR,
    ):
        self._geom = geom or StewartGeometry()
        self._seq = 0
        self._start_time = time.perf_counter()

        # Pose to include in MPC commands (for workspace checks).
        # Set by caller via set_pose() before command().
        self._last_pose_6dof = np.zeros(6)

        # Latest telemetry from control loop
        self._last_telem: dict | None = None

        # ZeroMQ context and sockets
        self._ctx = zmq.Context()

        # PUB socket — sends MPC commands + mode commands to control loop
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(mpc_command_addr)

        # SUB socket — receives telemetry from control loop (CONFLATE: latest)
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.connect(telemetry_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, TOPIC_TELEMETRY)
        self._sub.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking
        self._sub.setsockopt(zmq.CONFLATE, 1)

        # Let ZMQ sockets settle (PUB bind needs time before first send)
        time.sleep(_ZMQ_CONNECT_SETTLE_S)

        logger.info(
            f"HardwarePlant: PUB bound on {mpc_command_addr}, "
            f"SUB connected to {telemetry_addr}")

    # ------------------------------------------------------------------
    # PlantInterface implementation
    # ------------------------------------------------------------------

    def command(self, leg_extensions_mm: np.ndarray) -> None:
        """Send leg extension commands to the control loop.

        The control loop converts these to motor revolutions and applies
        the full safety pipeline (slew limiter, workspace checks, etc.).

        Parameters
        ----------
        leg_extensions_mm : (6,) ndarray — home-relative extensions in mm
        """
        msg = make_mpc_command(
            ext_mm=leg_extensions_mm.tolist(),
            pose_6dof=self._last_pose_6dof.tolist(),
            seq=self._seq,
        )
        self._pub.send_multipart(
            _pack(TOPIC_MPC_CMD, msg), flags=zmq.NOBLOCK)
        self._seq += 1

    def get_state(self) -> PlantState:
        """Read the latest telemetry from the control loop.

        Returns a PlantState constructed from motor feedback fields in
        the telemetry message.  If no telemetry has been received yet,
        returns a zero-state at the current elapsed time.
        """
        # Drain to get the latest (CONFLATE socket, but drain anyway)
        while True:
            try:
                frames = self._sub.recv_multipart(flags=zmq.NOBLOCK)
                _, msg = _unpack(frames)
                self._last_telem = msg
            except zmq.Again:
                break

        t = time.perf_counter() - self._start_time

        if self._last_telem is None:
            return PlantState(
                leg_extensions_mm=np.zeros(6),
                leg_velocities_mmps=np.zeros(6),
                platform_pos_mm=np.zeros(3),
                platform_rot=np.zeros(3),
                platform_twist=np.zeros(6),
                time=t,
            )

        telem = self._last_telem

        # Motor feedback (rev) → leg extensions (mm)
        motor_pos = telem.get('motor_pos')
        motor_vel = telem.get('motor_vel')
        if motor_pos is not None:
            pos_rev = np.array(motor_pos, dtype=float)
            ext_mm = revs_to_extensions_mm(pos_rev, self._geom)
        else:
            ext_mm = np.zeros(6)

        if motor_vel is not None:
            vel_rps = np.array(motor_vel, dtype=float)
            vel_mmps = vel_rps / self._geom.mm_to_rev
        else:
            vel_mmps = np.zeros(6)

        # We don't have direct Cartesian feedback from the control loop
        # telemetry (it sends motor-space data).  Use the last commanded
        # pose as a reasonable approximation — the MPC already knows the
        # actual state from its own model.
        return PlantState(
            leg_extensions_mm=ext_mm,
            leg_velocities_mmps=vel_mmps,
            platform_pos_mm=self._last_pose_6dof[:3].copy(),
            platform_rot=self._last_pose_6dof[3:6].copy(),
            platform_twist=np.zeros(6),
            time=t,
        )

    def step(self, dt: float) -> None:
        """No-op — hardware runs in real time."""
        pass

    def reset(self, pose_6dof: np.ndarray | None = None) -> None:
        """Not supported in hardware mode.

        The control loop handles homing via the orchestrator.  Calling
        reset() on hardware would require commanding a trajectory to home,
        which is out of scope for the plant interface.
        """
        logger.warning("HardwarePlant.reset() is a no-op; use the "
                        "orchestrator for homing")

    # ------------------------------------------------------------------
    # MPC-specific methods
    # ------------------------------------------------------------------

    def set_pose(self, pose_6dof: np.ndarray) -> None:
        """Set the Cartesian pose to include in the next MPC command.

        The control loop uses this for workspace/condition-number checks.
        Call this with ``mpc.predicted_poses[0]`` before ``command()``.
        """
        self._last_pose_6dof = np.asarray(pose_6dof, dtype=float).copy()

    def enable(self) -> None:
        """Send enable command with source='MPC' to activate pass-through mode."""
        msg = make_mode_command('enable', source='MPC')
        self._pub.send_multipart(
            _pack(TOPIC_MODE, msg), flags=zmq.NOBLOCK)
        logger.info("HardwarePlant: sent enable (source=MPC)")

    def disable(self) -> None:
        """Send disable command to the control loop."""
        msg = make_mode_command('disable')
        self._pub.send_multipart(
            _pack(TOPIC_MODE, msg), flags=zmq.NOBLOCK)
        logger.info("HardwarePlant: sent disable")

    def estop(self, reason: str = 'hardware_plant') -> None:
        """Send E-stop command to the control loop."""
        msg = make_mode_command('estop', reason=reason)
        self._pub.send_multipart(
            _pack(TOPIC_MODE, msg), flags=zmq.NOBLOCK)
        logger.warning(f"HardwarePlant: sent E-STOP ({reason})")

    @property
    def last_telemetry(self) -> dict | None:
        """The most recent telemetry message, or None."""
        return self._last_telem

    @property
    def geom(self) -> StewartGeometry:
        return self._geom

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Close ZeroMQ sockets and context."""
        self._pub.close()
        self._sub.close()
        self._ctx.term()
        logger.info("HardwarePlant: ZMQ sockets closed")

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
