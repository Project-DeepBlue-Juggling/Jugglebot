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

        # Home extensions: same computation as MuJoCoPlant.
        # Geometric home leg lengths (mm) minus IK init_leg_lengths_mm.
        base_m = self._geom.base_nodes / 1000.0
        plat_m = self._geom.plat_nodes / 1000.0
        height_m = self._geom.init_height_mm / 1000.0
        plat_world_m = plat_m + np.array([0.0, 0.0, height_m])
        geom_home_lengths_mm = np.linalg.norm(plat_world_m - base_m, axis=1) * 1000.0
        self._home_extensions_mm = geom_home_lengths_mm - self._geom.init_leg_lengths_mm

        # Stow offset: the motor revolutions at IK extension = 0.
        # The IK model's init_leg_lengths_mm represents a physical leg length
        # that requires ~154mm of cable extension from STOW (motor = 0 rev).
        # The activate position (from hardware_config) is the motor position
        # at IK home.  The stow offset is activate_rev - home_ik_rev.
        import jugglebot.hardware_config as hw
        self._activate_rev = np.array(hw.JB_OP_ACTIVATE_POSITION_REVS)
        home_ik_rev = self._home_extensions_mm * self._geom.mm_to_rev
        self._stow_offset_rev = self._activate_rev - home_ik_rev

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

        Converts MPC home-relative extensions to absolute motor revolutions
        (matching the ODrive encoder convention where 0 = STOW).

        The control loop applies the full safety pipeline (slew limiter,
        workspace checks via the included pose_6dof, overspeed, etc.).

        Parameters
        ----------
        leg_extensions_mm : (6,) ndarray — home-relative extensions in mm
            (MPC convention: 0 = home position)
        """
        # Convert: MPC home-relative (mm) → IK convention (mm) → motor rev
        # The stow offset accounts for the ~154mm of cable at STOW that
        # the IK model's init_leg_lengths_mm already includes.
        ik_extensions_mm = np.asarray(leg_extensions_mm) + self._home_extensions_mm
        motor_rev = ik_extensions_mm * self._geom.mm_to_rev + self._stow_offset_rev
        msg = make_mpc_command(
            ext_mm=ik_extensions_mm.tolist(),
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

        # Motor feedback (rev, absolute ODrive convention) → home-relative mm
        motor_pos = telem.get('motor_pos')
        motor_vel = telem.get('motor_vel')
        if motor_pos is not None:
            pos_rev = np.array(motor_pos, dtype=float)
            # Subtract stow offset to get IK-convention revolutions,
            # then convert to IK extensions (mm), then to home-relative.
            ik_rev = pos_rev - self._stow_offset_rev
            ik_ext_mm = ik_rev / self._geom.mm_to_rev
            ext_mm = ik_ext_mm - self._home_extensions_mm
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
    def home_extensions_mm(self) -> np.ndarray:
        """IK extensions at home (the offset used by the MPC for warm-starting)."""
        return self._home_extensions_mm.copy()

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
