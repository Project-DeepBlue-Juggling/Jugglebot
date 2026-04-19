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

from .plant import PlantInterface, PlantState

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

# Telemetry staleness thresholds (MPC runs at 40 Hz = 25 ms period)
_TELEM_STALE_WARN_S = 0.075   # 3x MPC period — log warning
_TELEM_STALE_HARD_S = 0.125   # 5x MPC period — zero velocities
_TELEM_STALE_ESTOP_S = 0.5    # 20x MPC period — telemetry definitely lost

# Leg motor incremental encoder resolution — used to derive a per-leg command
# dead-band.  Any cmd change below one encoder count is un-actionable by the
# motor (actual position cannot move sub-LSB), so forwarding such changes
# only injects jitter into the motor-guard Hermite interp and the ODrive
# pos_setpoint.  8192 CPR is the ODrive Pro leg encoder spec.
_LEG_ENC_CPR = 8192


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
        control_dt: float = 0.025,
        enable_torque_ff: bool = True,
        enable_vel_ff: bool = True,
        enable_acc_ff: bool = True,
    ):
        self._geom = geom or StewartGeometry()
        self._seq = 0
        self._start_time = time.perf_counter()
        self._control_dt = control_dt
        self._enable_torque_ff = enable_torque_ff
        self._enable_vel_ff = enable_vel_ff
        self._enable_acc_ff = enable_acc_ff

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
        self._prev_cmd_time: float | None = None
        self._prev_prev_cmd_time: float | None = None

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
        # Only counted after at least one successful FK (cold-start failures
        # are expected when the initial guess is None).
        self._fk_fail_count = 0
        self._fk_ever_succeeded = False
        self._FK_FAIL_ESTOP_THRESHOLD = 5  # 5 × 25ms = 125ms at 40 Hz
        self._jacobian_singular_warned = False

        # Lightweight diagnostics (read by main loop for telemetry)
        self._last_fk_iterations = 0
        self._last_ff_torque_max_Nm = 0.0
        # Number of telemetry frames drained from the SUB on the most recent
        # get_state() call.  Exposed so the MPC loop can correlate overhead
        # spikes with ZMQ queue depth.
        self._last_drain_count = 0

        # Per-leg command dead-band: one encoder count in mm (varies ~1%
        # across legs due to slightly different mm_to_rev).  When the next
        # cmd is within this of the last sent cmd on every leg, we re-send
        # the last value to avoid passing sub-LSB jitter into the motor
        # guard's Hermite interpolation.
        self._cmd_deadband_mm = 1.0 / (_LEG_ENC_CPR * self._geom.mm_to_rev)
        self._last_sent_ext_mm: np.ndarray | None = None
        self._cmd_deadband_hit_count = 0  # diagnostic: how often dead-band fires

        # ZeroMQ context and sockets
        self._ctx = zmq.Context()

        # PUB socket — sends MPC commands + mode commands to motor guard
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.bind(mpc_command_addr)

        # SUB socket — receives telemetry from motor guard.
        # Use SUBSCRIBE=b'' (accept all) instead of a topic filter: ZMQ 4.3.5
        # has a bug where CONFLATE + topic filter on a late-connecting SUB
        # permanently drops all messages.  The motor guard PUB on this port
        # only publishes telemetry, so accepting all is safe and equivalent.
        # Cap RECONNECT_IVL: without this, ZMQ exponential backoff
        # (100ms → 30s) can leave telemetry frozen for seconds after any
        # transient disconnect on :5556.
        # RCVHWM=2 + drain-in-loop in get_state() is used in lieu of CONFLATE:
        # CONFLATE must be set *before* connect() to take effect, and even
        # when correctly ordered it silently ignores multi-part messages on
        # some libzmq versions.  Explicit draining is robust.
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.RECONNECT_IVL, 100)      # 100ms base
        self._sub.setsockopt(zmq.RECONNECT_IVL_MAX, 200)   # 200ms cap
        self._sub.setsockopt(zmq.RCVHWM, 2)                # bounded queue
        self._sub.connect(telemetry_addr)
        self._sub.setsockopt(zmq.SUBSCRIBE, b'')
        self._sub.setsockopt(zmq.RCVTIMEO, 0)  # non-blocking

        # Let ZMQ sockets settle (PUB bind needs time before first send)
        time.sleep(_ZMQ_CONNECT_SETTLE_S)

        logger.info(
            f"HardwarePlant: PUB bound on {mpc_command_addr}, "
            f"SUB connected to {telemetry_addr}")

    # ------------------------------------------------------------------
    # PlantInterface implementation
    # ------------------------------------------------------------------

    def command(self, leg_extensions_mm: np.ndarray,
                vel_mm_s: np.ndarray | None = None,
                cmd_next_mm: np.ndarray | None = None,
                cmd_next2_mm: np.ndarray | None = None) -> None:
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
        cmd_next_mm : (6,) ndarray or None — MPC's predicted next-step
            command (u[1]).  Sent to the motor guard for Hermite
            interpolation between the current and next command.
        cmd_next2_mm : (6,) ndarray or None — MPC's predicted step-after-
            next command (u[2]).  Used with cmd_next_mm for C1-continuous
            Hermite interpolation across segment boundaries.
        """
        ext_mm = np.asarray(leg_extensions_mm, dtype=float)

        # Per-leg dead-band: if every leg's cmd change is below one encoder
        # count, hold at the last sent value.  Sub-LSB changes cannot move
        # the motor; forwarding them only creates 40 Hz jitter at the
        # motor-guard interpolation boundaries.  During real motion the
        # deltas are far above LSB so this never fires.  Compared against
        # the last SENT value (not the MPC's internal prev_u) so the
        # dead-band tracks the actual command history downstream.
        if self._last_sent_ext_mm is not None:
            delta = np.abs(ext_mm - self._last_sent_ext_mm)
            if np.all(delta < self._cmd_deadband_mm):
                ext_mm = self._last_sent_ext_mm
                vel_mm_s = np.zeros(6)
                cmd_next_mm = ext_mm
                cmd_next2_mm = ext_mm
                self._cmd_deadband_hit_count += 1

        motor_rev = ext_mm * self._geom.mm_to_rev

        # Velocity feedforward: prefer MPC-provided forward-looking velocity.
        # Fall back to backward-diff from last measured state when not provided.
        # Default to zero on first command (platform is stationary at boot).
        if vel_mm_s is not None:
            vel_mm_s = np.asarray(vel_mm_s, dtype=float)
        elif self._last_state_ext_mm is not None:
            # Use actual elapsed time for backward-diff (not fixed control_dt)
            if self._prev_cmd_time is not None:
                bd_dt = max(time.perf_counter() - self._prev_cmd_time,
                            self._control_dt * 0.5)
            else:
                bd_dt = self._control_dt
            vel_mm_s = (ext_mm - self._last_state_ext_mm) / bd_dt
        else:
            vel_mm_s = np.zeros(6)

        # Acceleration feedforward from three-point second difference of
        # consecutive commanded extensions.  Uses actual wall-clock dt
        # (not fixed control_dt) so acceleration is correct regardless of
        # MPC loop jitter.
        acc_mm_s2 = None
        if (self._prev_cmd_ext_mm is not None
                and self._prev_prev_cmd_ext_mm is not None
                and self._prev_cmd_time is not None
                and self._prev_prev_cmd_time is not None):
            t_now = time.perf_counter()
            dt1 = t_now - self._prev_cmd_time
            dt2 = self._prev_cmd_time - self._prev_prev_cmd_time
            avg_dt = max(0.5 * (dt1 + dt2), self._control_dt * 0.5)
            acc_mm_s2 = (
                ext_mm - 2.0 * self._prev_cmd_ext_mm
                + self._prev_prev_cmd_ext_mm) / (avg_dt * avg_dt)

        # Shift command history (extensions and timestamps).
        # prev→prev_prev is a reference swap (no copy needed — we never
        # mutate stored arrays).  Only the new entry needs a copy.
        self._prev_prev_cmd_ext_mm = self._prev_cmd_ext_mm
        self._prev_cmd_ext_mm = ext_mm.copy()
        self._prev_prev_cmd_time = self._prev_cmd_time
        self._prev_cmd_time = time.perf_counter()

        if not self._enable_vel_ff:
            vel_mm_s = np.zeros(6)
        if not self._enable_acc_ff:
            acc_mm_s2 = np.zeros(6)

        msg = make_mpc_command(
            ext_mm=ext_mm,
            motor_rev=motor_rev,
            pose_6dof=self._last_pose_6dof,
            vel_mm_s=vel_mm_s,
            acc_mm_s2=acc_mm_s2,
            torque_Nm=self._ff_torque_Nm,
            seq=self._seq,
            cmd_next_mm=cmd_next_mm,
            cmd_next2_mm=cmd_next2_mm,
        )
        self._pub.send_multipart(
            _pack(TOPIC_MPC_CMD, msg), flags=zmq.NOBLOCK)
        self._seq += 1
        # Remember what we actually sent so the next dead-band comparison
        # tracks the live command history, not the MPC's internal prev_u.
        self._last_sent_ext_mm = ext_mm.copy()

    def get_state(self) -> PlantState:
        """Read the latest telemetry from the motor guard.

        Returns a PlantState constructed from motor feedback fields in
        the telemetry message.  When motor positions are available,
        forward kinematics is used to compute the actual Cartesian pose
        so the MPC sees the real platform state (not its own prediction).

        If no telemetry has been received yet, returns a zero-state at
        the current elapsed time.
        """
        # Drain all pending messages, keep only the latest.  Explicit drain
        # is more robust than ZMQ_CONFLATE (which only works when set before
        # connect).  At 40–100 Hz polling vs 500 Hz publishing, a single
        # recv per poll would fall behind and read stale telemetry.
        drain_count = 0
        while True:
            try:
                frames = self._sub.recv_multipart(flags=zmq.NOBLOCK)
                _, self._last_telem = _unpack(frames)
                drain_count += 1
            except zmq.Again:
                break
        self._last_drain_count = drain_count
        if drain_count > 0:
            self._last_telem_recv_time = time.perf_counter()

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
        fk_jacobian = None  # Reuse Jacobian from FK for twist solve
        if ik_ext_mm is not None:
            try:
                # Bound FK to a real-time-friendly iteration budget.  Default
                # max_iter=50 and tol=1e-10 mm let Newton chase floating-point
                # noise for ~30 ms before throwing on hard starting guesses
                # (the 30 ms get_state spikes observed on motion onset).
                # tol=1e-4 mm is 700× below encoder LSB and still converges
                # in 3 iters on the common path; max_iter=10 caps the
                # divergence case at ~6 ms before falling back to the last
                # measured pose (same fallback semantics as before).
                pos_offset, rot_matrix, fk_jacobian = leg_lengths_to_pose(
                    ik_ext_mm, self._geom,
                    initial_guess=self._fk_last_guess,
                    max_iter=10,
                    tol=1e-4)
                rot_vec = rot_matrix_to_rotvec(rot_matrix)
                # Cache for warm-starting next FK call.
                # FK returns fresh arrays, so we can store references directly.
                # The cache is only read (never mutated) by the next FK call.
                self._fk_last_guess = (pos_offset, rot_matrix)
                # Cache measured pose for honest fallback
                self._last_measured_pose[:3] = pos_offset
                self._last_measured_pose[3:6] = rot_vec
                self._fk_fail_count = 0
                self._fk_ever_succeeded = True
                self._last_fk_iterations = leg_lengths_to_pose.last_iterations
                platform_pos_mm = pos_offset
                platform_rot = rot_vec
            except RuntimeError:
                self._fk_fail_count += 1
                logger.warning(
                    "FK did not converge; using last measured pose "
                    f"(consecutive failures: {self._fk_fail_count}, "
                    f"ever_succeeded: {self._fk_ever_succeeded})")
                if (self._fk_ever_succeeded
                        and self._fk_fail_count >= self._FK_FAIL_ESTOP_THRESHOLD):
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

        # E-stop when telemetry is completely lost — prevents runaway commands
        # when MPC keeps seeing stale position feedback and ramping commands.
        if telem_age is not None and telem_age > _TELEM_STALE_ESTOP_S:
            logger.error(
                f"Telemetry stale ({telem_age:.3f}s > "
                f"{_TELEM_STALE_ESTOP_S}s) — triggering e-stop")
            self.estop(reason='telemetry_stale')

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
                # Reuse Jacobian from FK when available (avoids recomputation)
                if fk_jacobian is not None:
                    J = fk_jacobian
                else:
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

        Computes torque feedforward (gravity + platform inertia) from the
        pose, twist, and acceleration using the Newton-Euler dynamics model.
        Reflected motor inertia is skipped to avoid the expensive numerical
        J_dot computation (2 extra Jacobian evaluations); its contribution
        is marginal (~2% PID effort reduction at current speeds).

        Parameters
        ----------
        pose_6dof : [x, y, z, rx, ry, rz] in mm, rad
            From ``mpc.predicted_poses[0]``.
        twist_6dof : [vx, vy, vz, wx, wy, wz] in mm/s, rad/s, or None
            Platform twist (velocity).  If None, zeros are used (static).
        accel_6dof : [ax, ay, az, alphax, alphay, alphaz] in mm/s², rad/s²
            Platform acceleration.  If None, zeros are used (gravity-only
            feedforward).  Providing real acceleration enables platform
            inertia feedforward, reducing PID effort during dynamic motion.
        """
        self._last_pose_6dof = np.asarray(pose_6dof, dtype=float).copy()

        if not self._enable_torque_ff:
            self._ff_torque_Nm = np.zeros(6)
            self._last_ff_torque_max_Nm = 0.0
            return

        if twist_6dof is None:
            twist_6dof = np.zeros(6)
        if accel_6dof is None:
            accel_6dof = np.zeros(6)

        # Compute torque feedforward using the production dynamics model.
        # Reflected motor inertia is skipped (skip_reflected_inertia=True)
        # to avoid 2 extra Jacobian evaluations per tick (~0.5 ms saved).
        _, _, torque_ff_Nm, _ = cartesian_to_motor_commands(
            pose_6dof, twist_6dof, accel_6dof,
            self._geom, self._dynamics_params,
            feedforward_enabled=True,
            skip_reflected_inertia=True)

        self._ff_torque_Nm = torque_ff_Nm
        self._last_ff_torque_max_Nm = float(np.max(np.abs(torque_ff_Nm)))

    def enable(self, timeout_s: float = 2.0) -> None:
        """Send disable+enable and wait for motor feedback telemetry.

        A leading ``disable`` is required because motor_guard's mode handler
        ignores ``enable`` received while in ESTOP (only ``DISABLED → ENABLED``
        is valid).  Sending ``disable`` first forces a clean DISABLED state,
        clearing any lingering ESTOP from a previous session or external fault.

        Then blocks until the motor guard publishes telemetry containing valid
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
        self._pub.send_multipart(
            _pack(TOPIC_MODE, make_mode_command('disable')),
            flags=zmq.NOBLOCK)
        msg = make_mode_command('enable', source='MPC')
        self._pub.send_multipart(
            _pack(TOPIC_MODE, msg), flags=zmq.NOBLOCK)
        logger.info("HardwarePlant: sent disable+enable (source=MPC)")

        # Phase 1: Wait for the motor guard to publish telemetry with valid
        # motor positions.  The guard publishes feedback-only telemetry while
        # waiting for the first MPC command, which gives us the actual
        # motor state to seed the MPC.
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            self.get_state()  # drains telemetry into _last_telem
            if (self._last_telem is not None
                    and self._last_telem.get('motor_pos') is not None):
                logger.info(
                    "HardwarePlant: received motor feedback from guard")
                break
            time.sleep(0.01)
        else:
            raise RuntimeError(
                f"HardwarePlant: no motor feedback telemetry within "
                f"{timeout_s}s — is the motor guard running and receiving "
                f"CAN feedback?")

        # Phase 2: Verify the command channel (PUB :5557 → motor guard SUB)
        # is connected by sending a hold-at-current probe command and waiting
        # for the motor guard to acknowledge it (telemetry with leg_pos set).
        # Without this, ZMQ reconnect backoff on the motor guard's SUB can
        # delay command delivery by seconds, causing the MPC to fly blind.
        motor_pos = np.array(self._last_telem['motor_pos'], dtype=float)
        ext_mm = motor_pos / self._geom.mm_to_rev
        probe_msg = make_mpc_command(
            ext_mm=ext_mm,
            motor_rev=motor_pos,
            pose_6dof=self._last_measured_pose.copy(),
            vel_mm_s=np.zeros(6),
            torque_Nm=np.zeros(6),
            seq=-1,
        )

        probe_deadline = time.perf_counter() + timeout_s
        probe_interval = 0.05  # resend every 50ms
        next_send = 0.0
        while time.perf_counter() < probe_deadline:
            now = time.perf_counter()
            if now >= next_send:
                self._pub.send_multipart(
                    _pack(TOPIC_MPC_CMD, probe_msg), flags=zmq.NOBLOCK)
                next_send = now + probe_interval

            self.get_state()
            if (self._last_telem is not None
                    and self._last_telem.get('leg_pos') is not None):
                logger.info(
                    "HardwarePlant: command channel verified "
                    "(motor guard received probe)")
                return
            time.sleep(0.01)

        raise RuntimeError(
            "HardwarePlant: motor guard did not acknowledge probe command "
            f"within {timeout_s}s — ZMQ command channel (:5557) may not be "
            "connected")

    def disable(self) -> None:
        """Send disable command to the motor guard."""
        msg = make_mode_command('disable')
        self._pub.send_multipart(
            _pack(TOPIC_MODE, msg), flags=zmq.NOBLOCK)
        # Reset dead-band history — the platform may be moved externally while
        # disabled, so the first command after re-enable must pass through
        # regardless of proximity to the last sent value.
        self._last_sent_ext_mm = None
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

    @property
    def last_fk_iterations(self) -> int:
        """FK Newton-Raphson iteration count from last get_state()."""
        return self._last_fk_iterations

    @property
    def last_ff_torque_max_Nm(self) -> float:
        """Max absolute feedforward torque (Nm) from last set_pose()."""
        return self._last_ff_torque_max_Nm

    @property
    def last_drain_count(self) -> int:
        """Number of ZMQ telemetry frames drained in the most recent get_state()."""
        return self._last_drain_count

    @property
    def cmd_deadband_hit_count(self) -> int:
        """Total number of commands held by the dead-band over the run."""
        return self._cmd_deadband_hit_count

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
