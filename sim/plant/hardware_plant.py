"""Real hardware plant — sends MPC commands to the control loop via ZeroMQ.

Implements the PlantInterface so the MPC controller can swap between
MuJoCoPlant (simulation) and HardwarePlant (real robot) transparently.

Architecture:
  - PUB socket on MPC_COMMAND_ADDR (:5557): sends mpc_cmd and mode messages
  - SUB socket on TELEMETRY_ADDR (:5556): receives telemetry from control loop

Coordinate convention (post STOW-zero refactor):
  - Extensions are STOW-relative: q=0 at STOW, q≈154.5mm at Active.
  - Motor conversion is direct: motor_rev = q × mm_to_rev (no offsets).

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
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import leg_lengths_to_pose, rot_matrix_to_rotvec, rotvec_to_rot_matrix
from jugglebot.motion.motor_commands import cartesian_to_motor_commands
from jugglebot.motion.dynamics import DynamicsParams

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

        # Dynamics parameters for feedforward computation
        self._dynamics_params = DynamicsParams.from_config()

        # Pose to include in MPC commands (for workspace checks).
        # Set by caller via set_pose() before command().
        self._last_pose_6dof = np.zeros(6)

        # Feedforward: computed from the MPC's predicted pose via the
        # existing dynamics model (gravity + inertia).  Set by set_pose().
        self._ff_vel_mm_s: np.ndarray | None = None
        self._ff_torque_Nm: np.ndarray | None = None

        # Latest telemetry from control loop
        self._last_telem: dict | None = None

        # FK warm-start: cache last FK result as initial guess for next call.
        # Newton-Raphson converges in 1-2 iterations when warm-started from
        # a nearby pose (motor feedback at 50-100 Hz moves very little).
        self._fk_last_guess: tuple[np.ndarray, np.ndarray] | None = None

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

        Converts STOW-relative extensions to motor revolutions directly:
        motor_rev = ext_mm × mm_to_rev (no offsets).

        The control loop applies the full safety pipeline (slew limiter,
        workspace checks via the included pose_6dof, overspeed, etc.).

        Parameters
        ----------
        leg_extensions_mm : (6,) ndarray — STOW-relative extensions in mm
            (0 = STOW, ~154.5 = Active position)
        """
        ext_mm = np.asarray(leg_extensions_mm, dtype=float)
        motor_rev = ext_mm * self._geom.mm_to_rev
        msg = make_mpc_command(
            ext_mm=ext_mm.tolist(),
            motor_rev=motor_rev.tolist(),
            pose_6dof=self._last_pose_6dof.tolist(),
            vel_mm_s=(self._ff_vel_mm_s.tolist()
                      if self._ff_vel_mm_s is not None else None),
            torque_Nm=(self._ff_torque_Nm.tolist()
                       if self._ff_torque_Nm is not None else None),
            seq=self._seq,
        )
        self._pub.send_multipart(
            _pack(TOPIC_MPC_CMD, msg), flags=zmq.NOBLOCK)
        self._seq += 1

    def get_state(self) -> PlantState:
        """Read the latest telemetry from the control loop.

        Returns a PlantState constructed from motor feedback fields in
        the telemetry message.  When motor positions are available,
        forward kinematics is used to compute the actual Cartesian pose
        so the MPC sees the real platform state (not its own prediction).

        If no telemetry has been received yet, returns a zero-state at
        the current elapsed time.
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

        # Motor feedback (rev, ODrive convention: 0 = STOW) → extensions (mm)
        motor_pos = telem.get('motor_pos')
        motor_vel = telem.get('motor_vel')
        if motor_pos is not None:
            pos_rev = np.array(motor_pos, dtype=float)
            ext_mm = pos_rev / self._geom.mm_to_rev  # direct: motor_rev / mm_to_rev
            ik_ext_mm = ext_mm  # same thing (q = IK ext after refactor)
        else:
            ik_ext_mm = None
            ext_mm = np.zeros(6)

        if motor_vel is not None:
            vel_rps = np.array(motor_vel, dtype=float)
            vel_mmps = vel_rps / self._geom.mm_to_rev
        else:
            vel_mmps = np.zeros(6)

        # Compute Cartesian pose from actual motor positions via FK.
        # This closes the feedback loop: the MPC sees where the platform
        # actually is, not where it predicted it would be.
        if ik_ext_mm is not None:
            try:
                pos_offset, rot_matrix = leg_lengths_to_pose(
                    ik_ext_mm, self._geom,
                    initial_guess=self._fk_last_guess)
                rot_vec = rot_matrix_to_rotvec(rot_matrix)
                # Cache for warm-starting next FK call
                self._fk_last_guess = (pos_offset.copy(), rot_matrix.copy())
                platform_pos_mm = pos_offset
                platform_rot = rot_vec
            except RuntimeError:
                # FK failed to converge — fall back to last commanded pose
                logger.warning("FK did not converge; using last commanded pose")
                platform_pos_mm = self._last_pose_6dof[:3].copy()
                platform_rot = self._last_pose_6dof[3:6].copy()
        else:
            platform_pos_mm = self._last_pose_6dof[:3].copy()
            platform_rot = self._last_pose_6dof[3:6].copy()

        return PlantState(
            leg_extensions_mm=ext_mm,
            leg_velocities_mmps=vel_mmps,
            platform_pos_mm=platform_pos_mm,
            platform_rot=platform_rot,
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

    def set_pose(self, pose_6dof: np.ndarray,
                 twist_6dof: np.ndarray | None = None) -> None:
        """Set the Cartesian pose and compute feedforward for the next command.

        Computes velocity and torque feedforward from the pose using the
        existing dynamics model (gravity + inertia).  These are included
        in the next ``command()`` call so the ODrive PID doesn't have to
        fight gravity alone.

        Parameters
        ----------
        pose_6dof : [x, y, z, rx, ry, rz] in mm, rad
            From ``mpc.predicted_poses[0]``.
        twist_6dof : [vx, vy, vz, wx, wy, wz] in mm/s, rad/s, or None
            Platform twist (velocity).  If None, zeros are used (static).
        """
        self._last_pose_6dof = np.asarray(pose_6dof, dtype=float).copy()

        if twist_6dof is None:
            twist_6dof = np.zeros(6)
        accel_6dof = np.zeros(6)

        # Compute feedforward using the production dynamics model
        _, vel_ff_rps, torque_ff_Nm, _ = cartesian_to_motor_commands(
            pose_6dof, twist_6dof, accel_6dof,
            self._geom, self._dynamics_params,
            feedforward_enabled=True)

        # Convert vel_ff from rev/s back to mm/s for the IPC message
        self._ff_vel_mm_s = vel_ff_rps / self._geom.mm_to_rev
        self._ff_torque_Nm = torque_ff_Nm

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
