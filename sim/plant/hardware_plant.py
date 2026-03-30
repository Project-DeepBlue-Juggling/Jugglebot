"""Real hardware plant — sends MPC commands to the motor guard via ZeroMQ.

Implements the PlantInterface so the MPC controller can swap between
MuJoCoPlant (simulation) and HardwarePlant (real robot) transparently.

Architecture:
  - PUB socket on MPC_COMMAND_ADDR (:5557): sends mpc_cmd and mode messages
  - SUB socket on TELEMETRY_ADDR (:5556): receives telemetry from motor guard

Coordinate convention (post STOW-zero refactor):
  - Extensions are STOW-relative: q=0 at STOW, q≈154.5mm at Active.
  - Motor conversion is direct: motor_rev = q × mm_to_rev (no offsets).

The motor guard's safety pipeline (workspace checks, overspeed, max deviation,
staleness watchdog) remains active — this plant only converts and transmits,
never bypasses safety.
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
from jugglebot.motion.ik_solver import (
    compute_jacobian,
    leg_lengths_to_pose,
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
)
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

# Telemetry staleness thresholds (MPC runs at 50 Hz = 20 ms period)
_TELEM_STALE_WARN_S = 0.060   # 3x MPC period — log warning
_TELEM_STALE_HARD_S = 0.100   # 5x MPC period — zero velocities


class HardwarePlant(PlantInterface):
    """Stewart platform plant that communicates with real hardware via IPC.

    Sends pre-computed leg extensions to the motor guard.  Reads motor
    feedback from motor guard telemetry.

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
        control_dt: float = 0.02,
    ):
        self._geom = geom or StewartGeometry()
        self._seq = 0
        self._start_time = time.perf_counter()
        self._control_dt = control_dt

        # Dynamics parameters for feedforward computation
        self._dynamics_params = DynamicsParams.from_config()

        # Pose to include in MPC commands (for workspace checks).
        # Set by caller via set_pose() before command().
        self._last_pose_6dof = np.zeros(6)

        # Feedforward: torque_ff from dynamics model (set_pose).
        # vel_ff computed here from (cmd - q_init) / control_dt and sent via IPC.
        self._ff_torque_Nm: np.ndarray | None = None

        # Cached actual leg extensions from the last get_state() call.
        # Used as q_init for velocity feedforward: vel = (cmd - q_init) / dt.
        self._last_state_ext_mm: np.ndarray | None = None

        # Extension command history for acceleration feedforward.
        # Tracks the two most recent commanded extensions so command() can
        # compute acc = (u_curr - 2·u_prev + u_prev_prev) / dt².
        self._prev_cmd_ext_mm: np.ndarray | None = None
        self._prev_prev_cmd_ext_mm: np.ndarray | None = None

        # Latest telemetry from motor guard
        self._last_telem: dict | None = None
        self._last_telem_recv_time: float | None = None
        self._telem_stale_warned = False  # edge-triggered logging

        # FK warm-start: cache last FK result as initial guess for next call.
        # Newton-Raphson converges in 1-2 iterations when warm-started from
        # a nearby pose (motor feedback at 50-100 Hz moves very little).
        self._fk_last_guess: tuple[np.ndarray, np.ndarray] | None = None

        # Last successfully measured pose from FK (not the MPC's prediction).
        # Used as fallback when FK fails, so the MPC never sees its own
        # output masquerading as sensor feedback.
        self._last_measured_pose = np.zeros(6)

        # Consecutive FK failure counter.  If FK fails for too many cycles
        # in a row, something is seriously wrong — trigger e-stop.
        self._fk_fail_count = 0
        self._FK_FAIL_ESTOP_THRESHOLD = 5  # 5 × 20ms = 100ms at 50 Hz
        self._jacobian_singular_warned = False

        # ZeroMQ context and sockets
        self._ctx = zmq.Context()

        # PUB socket — sends MPC commands + mode commands to motor guard
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(mpc_command_addr)

        # SUB socket — receives telemetry from motor guard (CONFLATE: latest)
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

    def command(self, leg_extensions_mm: np.ndarray,
                vel_mm_s: np.ndarray | None = None) -> None:
        """Send leg extension commands to the motor guard.

        Converts STOW-relative extensions to motor revolutions directly:
        motor_rev = ext_mm × mm_to_rev (no offsets).

        Parameters
        ----------
        leg_extensions_mm : (6,) ndarray — STOW-relative extensions in mm
            (0 = STOW, ~154.5 = Active position)
        vel_mm_s : (6,) ndarray or None — forward-looking command velocity
            (mm/s) from the MPC solver.  When provided, this is preferred
            over the backward-difference fallback because it uses the MPC's
            own ``q_cur`` (no tracking error noise) and is forward-looking.
            Falls back to ``(cmd - last_measured) / control_dt`` when None.
        """
        ext_mm = np.asarray(leg_extensions_mm, dtype=float)
        motor_rev = ext_mm * self._geom.mm_to_rev

        # Velocity feedforward: prefer MPC-provided forward-looking velocity.
        # Fall back to backward-diff from last measured state when not provided.
        # Default to zero on first command (platform is stationary at boot).
        if vel_mm_s is not None:
            vel_mm_s = np.asarray(vel_mm_s, dtype=float)
        elif self._last_state_ext_mm is not None:
            vel_mm_s = (ext_mm - self._last_state_ext_mm) / self._control_dt
        else:
            vel_mm_s = np.zeros(6)

        # Acceleration feedforward from three-point second difference of
        # consecutive commanded extensions.  Requires two prior commands.
        acc_mm_s2 = None
        if (self._prev_cmd_ext_mm is not None
                and self._prev_prev_cmd_ext_mm is not None):
            dt2 = self._control_dt * self._control_dt
            acc_mm_s2 = (
                ext_mm - 2.0 * self._prev_cmd_ext_mm
                + self._prev_prev_cmd_ext_mm) / dt2

        # Shift command history
        self._prev_prev_cmd_ext_mm = (
            self._prev_cmd_ext_mm.copy()
            if self._prev_cmd_ext_mm is not None else None)
        self._prev_cmd_ext_mm = ext_mm.copy()

        msg = make_mpc_command(
            ext_mm=ext_mm.tolist(),
            motor_rev=motor_rev.tolist(),
            pose_6dof=self._last_pose_6dof.tolist(),
            vel_mm_s=vel_mm_s.tolist() if vel_mm_s is not None else None,
            acc_mm_s2=acc_mm_s2.tolist() if acc_mm_s2 is not None else None,
            torque_Nm=(self._ff_torque_Nm.tolist()
                       if self._ff_torque_Nm is not None else None),
            seq=self._seq,
        )
        self._pub.send_multipart(
            _pack(TOPIC_MPC_CMD, msg), flags=zmq.NOBLOCK)
        self._seq += 1

    def get_state(self) -> PlantState:
        """Read the latest telemetry from the motor guard.

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
                self._last_telem_recv_time = time.perf_counter()
            except zmq.Again:
                break

        now = time.perf_counter()
        t = now - self._start_time

        # Compute telemetry data age (None if no telemetry ever received)
        telem_age = (now - self._last_telem_recv_time
                     if self._last_telem_recv_time is not None else None)

        if self._last_telem is None:
            return PlantState(
                leg_extensions_mm=np.zeros(6),
                leg_velocities_mmps=np.zeros(6),
                platform_pos_mm=np.zeros(3),
                platform_rot=np.zeros(3),
                platform_twist=np.zeros(6),
                time=t,
                data_age_s=telem_age,
            )

        telem = self._last_telem

        # Motor feedback (rev, ODrive convention: 0 = STOW) → extensions (mm)
        motor_pos = telem.get('motor_pos')
        motor_vel = telem.get('motor_vel')
        if motor_pos is not None:
            pos_rev = np.array(motor_pos, dtype=float)
            ext_mm = pos_rev / self._geom.mm_to_rev  # direct: motor_rev / mm_to_rev
            ik_ext_mm = ext_mm  # same thing (q = IK ext after refactor)
            self._last_state_ext_mm = ext_mm.copy()
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
        rot_matrix = None
        if ik_ext_mm is not None:
            try:
                pos_offset, rot_matrix = leg_lengths_to_pose(
                    ik_ext_mm, self._geom,
                    initial_guess=self._fk_last_guess)
                rot_vec = rot_matrix_to_rotvec(rot_matrix)
                # Cache for warm-starting next FK call
                self._fk_last_guess = (pos_offset.copy(), rot_matrix.copy())
                # Cache measured pose for honest fallback
                self._last_measured_pose[:3] = pos_offset
                self._last_measured_pose[3:6] = rot_vec
                self._fk_fail_count = 0
                platform_pos_mm = pos_offset
                platform_rot = rot_vec
            except RuntimeError:
                self._fk_fail_count += 1
                logger.warning(
                    "FK did not converge; using last measured pose "
                    f"(consecutive failures: {self._fk_fail_count})")
                if self._fk_fail_count >= self._FK_FAIL_ESTOP_THRESHOLD:
                    logger.error(
                        f"FK failed {self._fk_fail_count} consecutive times "
                        "— triggering e-stop")
                    self.estop(reason='fk_convergence_failure')
                platform_pos_mm = self._last_measured_pose[:3].copy()
                platform_rot = self._last_measured_pose[3:6].copy()
        else:
            # No motor position data — use last measured pose (not MPC prediction)
            platform_pos_mm = self._last_measured_pose[:3].copy()
            platform_rot = self._last_measured_pose[3:6].copy()

        # Degrade velocity data when telemetry is stale
        if telem_age is not None and telem_age > _TELEM_STALE_HARD_S:
            vel_mmps = np.zeros(6)
            if not self._telem_stale_warned:
                logger.warning(
                    f"Telemetry stale ({telem_age:.3f}s > "
                    f"{_TELEM_STALE_HARD_S}s) — zeroing velocities")
                self._telem_stale_warned = True
        elif telem_age is not None and telem_age > _TELEM_STALE_WARN_S:
            if not self._telem_stale_warned:
                logger.warning(
                    f"Telemetry aging ({telem_age:.3f}s > "
                    f"{_TELEM_STALE_WARN_S}s)")
                self._telem_stale_warned = True
        else:
            self._telem_stale_warned = False

        # Compute platform twist from real motor velocities via J^{-1}.
        # twist = solve(J, vel_mmps) where J maps platform twist → leg rates.
        #
        # The raw Jacobian has mixed units (dimensionless translational cols,
        # mm/rad rotational cols) giving cond ~450.  A bare solve amplifies
        # encoder noise asymmetrically into the angular twist components.
        # We scale the rotational columns by the platform circumradius (the
        # characteristic length), solve the better-conditioned system, then
        # un-scale to recover physical units.  Mathematically identical in
        # exact arithmetic, but distributes floating-point error evenly.
        platform_twist = np.zeros(6)
        if motor_vel is not None and np.any(vel_mmps != 0.0):
            try:
                if rot_matrix is None:
                    rot_matrix = rotvec_to_rot_matrix(platform_rot)
                J = compute_jacobian(platform_pos_mm, rot_matrix, self._geom)
                L_c = self._geom.plat_radius_mm
                J_norm = J.copy()
                J_norm[:, 3:] /= L_c
                twist_scaled = np.linalg.solve(J_norm, vel_mmps)
                twist_scaled[3:] /= L_c  # un-scale angular components
                platform_twist = twist_scaled
                self._jacobian_singular_warned = False
            except np.linalg.LinAlgError:
                if not self._jacobian_singular_warned:
                    logger.warning("Jacobian singular — platform twist zeroed")
                    self._jacobian_singular_warned = True

        return PlantState(
            leg_extensions_mm=ext_mm,
            leg_velocities_mmps=vel_mmps,
            platform_pos_mm=platform_pos_mm,
            platform_rot=platform_rot,
            platform_twist=platform_twist,
            time=t,
            data_age_s=telem_age,
        )

    def step(self, dt: float) -> None:
        """No-op — hardware runs in real time."""
        pass

    def reset(self, pose_6dof: np.ndarray | None = None) -> None:
        """Not supported in hardware mode.

        The motor guard handles homing via the orchestrator.  Calling
        reset() on hardware would require commanding a trajectory to home,
        which is out of scope for the plant interface.
        """
        logger.warning("HardwarePlant.reset() is a no-op; use the "
                        "orchestrator for homing")

    # ------------------------------------------------------------------
    # MPC-specific methods
    # ------------------------------------------------------------------

    def set_pose(self, pose_6dof: np.ndarray,
                 twist_6dof: np.ndarray | None = None,
                 accel_6dof: np.ndarray | None = None) -> None:
        """Set the Cartesian pose and compute torque feedforward.

        Computes torque feedforward (gravity + platform inertia + reflected
        motor inertia) from the pose, twist, and acceleration using the
        full Newton-Euler dynamics model.  Velocity feedforward is computed
        by ``command()`` as ``(cmd - q_init) / control_dt``.

        Parameters
        ----------
        pose_6dof : [x, y, z, rx, ry, rz] in mm, rad
            From ``mpc.predicted_poses[0]``.
        twist_6dof : [vx, vy, vz, wx, wy, wz] in mm/s, rad/s, or None
            Platform twist (velocity).  If None, zeros are used (static).
        accel_6dof : [ax, ay, az, alphax, alphay, alphaz] in mm/s², rad/s²
            Platform acceleration.  If None, zeros are used (gravity-only
            feedforward).  Providing real acceleration enables inertia
            feedforward (platform + reflected motor), reducing PID effort
            during dynamic motion.
        """
        self._last_pose_6dof = np.asarray(pose_6dof, dtype=float).copy()

        if twist_6dof is None:
            twist_6dof = np.zeros(6)
        if accel_6dof is None:
            accel_6dof = np.zeros(6)

        # Compute torque feedforward using the production dynamics model.
        # vel_ff is computed by the motor guard from consecutive extensions.
        _, _, torque_ff_Nm, _ = cartesian_to_motor_commands(
            pose_6dof, twist_6dof, accel_6dof,
            self._geom, self._dynamics_params,
            feedforward_enabled=True)

        self._ff_torque_Nm = torque_ff_Nm

    def enable(self, timeout_s: float = 2.0) -> None:
        """Send enable command and wait for motor feedback telemetry.

        Blocks until the motor guard publishes telemetry containing valid
        motor positions, so the MPC's first ``get_state()`` call returns
        the actual platform pose (not zeros).  Without this, the MPC
        would plan a trajectory from STOW to Active and trigger
        MAX_DEVIATION on the first command.

        Parameters
        ----------
        timeout_s : float
            Maximum time to wait for valid telemetry (default 2 s).

        Raises
        ------
        RuntimeError
            If no telemetry with motor positions arrives within timeout.
        """
        msg = make_mode_command('enable', source='MPC')
        self._pub.send_multipart(
            _pack(TOPIC_MODE, msg), flags=zmq.NOBLOCK)
        logger.info("HardwarePlant: sent enable (source=MPC)")

        # Wait for the motor guard to publish telemetry with valid motor
        # positions.  The guard publishes feedback-only telemetry while
        # waiting for the first MPC command, which gives us the actual
        # motor state to seed the MPC.
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            self.get_state()  # drains telemetry into _last_telem
            if (self._last_telem is not None
                    and self._last_telem.get('motor_pos') is not None):
                logger.info(
                    "HardwarePlant: received motor feedback from guard")
                return
            time.sleep(0.01)

        raise RuntimeError(
            f"HardwarePlant: no motor feedback telemetry within {timeout_s}s "
            "— is the motor guard running and receiving CAN feedback?")

    def disable(self) -> None:
        """Send disable command to the motor guard."""
        msg = make_mode_command('disable')
        self._pub.send_multipart(
            _pack(TOPIC_MODE, msg), flags=zmq.NOBLOCK)
        logger.info("HardwarePlant: sent disable")

    def estop(self, reason: str = 'hardware_plant') -> None:
        """Send E-stop command to the motor guard."""
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
