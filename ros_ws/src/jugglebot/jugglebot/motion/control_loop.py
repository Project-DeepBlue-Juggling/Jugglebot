"""Standalone fixed-rate control process for the Stewart platform.

Runs independently from ROS2 for timing predictability.  Communicates
with the ROS2 world via the IPC layer (ZeroMQ).

Two modes of operation:
  1. **Trajectory mode** (Phase 4): execute quintic trajectories that
     produce time-parameterized (pos, vel_ff, torque_ff) per cycle.
  2. **Direct-target mode** (Phase 3): use a fixed target pose for
     IK + dynamics (spacemouse, shell commands).

Phase 6 additions:
  - Workspace limit enforcement (soft/hard limits on leg extensions
    and Jacobian condition number)
  - Runtime condition number monitoring with speed degradation
  - Motor feedback handling for tracking error computation
  - Fault detection (ODrive fault forwarding via IPC)
  - Extended telemetry: condition number, workspace status, tracking
    error, trajectory progress, fault state

Common infrastructure:
  - Fixed-rate main loop with nanosecond timing instrumentation
  - IPC message dispatch (targets, trajectories, mode commands, motor feedback)
  - Position IK: pose -> leg extensions -> motor revolutions (input_pos)
  - Velocity IK: twist -> leg velocities -> motor velocities (vel_ff)
  - Full feedforward: dynamics model -> motor torques (torque_ff)
  - E-stop on IPC heartbeat loss or hard workspace limit violation

The ODrive handles all feedback at 8 kHz via its cascaded PID.
This process computes IK + dynamics and outputs (pos, vel_ff, torque_ff).

Run:  python -m jugglebot.motion.control_loop [--rate HZ]
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from dataclasses import dataclass, field

import numpy as np

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    compute_jacobian,
    pose_to_leg_lengths,
    quat_to_rot_matrix,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
)
from jugglebot.motion.conversions import (
    extensions_mm_to_revs,
    revs_to_extensions_mm,
    leg_velocities_to_motor_velocities,
)
from jugglebot.motion.dynamics import (
    DynamicsParams,
    compute_full_feedforward_torques,
    gravity_to_motor_torques,
)
from jugglebot.motion.ipc import (
    TOPIC_DYN_TARGET,
    TOPIC_MODE,
    TOPIC_MOTOR_FB,
    TOPIC_TARGET,
    TOPIC_TRAJECTORY,
    ControlProcessIPC,
    make_telemetry,
)
from jugglebot.motion.trajectory import (
    TrajectoryManager,
    TrajectoryState,
    create_trajectory,
)
from jugglebot.motion.workspace import (
    WorkspaceLimits,
    WorkspaceStatus,
    check_workspace_limits,
    compute_condition_number,
)

logger = logging.getLogger(__name__)

# Maximum time without a message from the bridge before triggering E-stop
IPC_HEARTBEAT_TIMEOUT_S = 0.5

# Default control loop rate
DEFAULT_RATE_HZ = 500


# ---------------------------------------------------------------------------
# Timing statistics
# ---------------------------------------------------------------------------

@dataclass
class LoopStats:
    """Accumulates per-cycle timing data for jitter analysis."""
    target_dt_s: float
    cycle_times: list[float] = field(default_factory=list)
    _window_size: int = 10000  # keep last N cycles

    def record(self, dt_s: float) -> None:
        self.cycle_times.append(dt_s)
        if len(self.cycle_times) > self._window_size:
            self.cycle_times.pop(0)

    @property
    def count(self) -> int:
        return len(self.cycle_times)

    @property
    def mean(self) -> float:
        return np.mean(self.cycle_times) if self.cycle_times else 0.0

    @property
    def std(self) -> float:
        return np.std(self.cycle_times) if self.cycle_times else 0.0

    @property
    def percentile_99(self) -> float:
        return float(np.percentile(self.cycle_times, 99)) if self.cycle_times else 0.0

    @property
    def max(self) -> float:
        return float(np.max(self.cycle_times)) if self.cycle_times else 0.0

    @property
    def jitter_99(self) -> float:
        """99th percentile jitter = |dt - target_dt|."""
        if not self.cycle_times:
            return 0.0
        jitters = np.abs(np.array(self.cycle_times) - self.target_dt_s)
        return float(np.percentile(jitters, 99))

    def summary(self) -> str:
        if not self.cycle_times:
            return "no data"
        return (f"mean={self.mean*1000:.2f}ms  "
                f"std={self.std*1000:.2f}ms  "
                f"p99={self.percentile_99*1000:.2f}ms  "
                f"max={self.max*1000:.2f}ms  "
                f"jitter_p99={self.jitter_99*1000:.2f}ms  "
                f"n={self.count}")


# ---------------------------------------------------------------------------
# Control modes
# ---------------------------------------------------------------------------

class ControlMode:
    DISABLED = 'disabled'
    ENABLED = 'enabled'
    ESTOP = 'estop'


# ---------------------------------------------------------------------------
# Control loop
# ---------------------------------------------------------------------------

class ControlLoop:
    """Fixed-rate control process for the Stewart platform.

    Parameters
    ----------
    target_rate_hz : float -- desired loop rate
    geom : StewartGeometry
    ipc : ControlProcessIPC -- IPC transport (injected for testability)
    """

    def __init__(self,
                 target_rate_hz: float = DEFAULT_RATE_HZ,
                 geom: StewartGeometry | None = None,
                 ipc: ControlProcessIPC | None = None):
        self.rate_hz = target_rate_hz
        self.dt_target = 1.0 / target_rate_hz
        self.geom = geom or StewartGeometry()
        self.ipc = ipc or ControlProcessIPC()

        self.mode = ControlMode.DISABLED
        self.stats = LoopStats(target_dt_s=self.dt_target)
        self._running = False

        # Latest state from IPC
        self._target_pos = np.zeros(3)
        self._target_rot = np.eye(3)
        self._target_twist = np.zeros(6)
        self._target_accel = np.zeros(6)
        self._has_target = False

        # Control outputs (sent to bridge -> CAN node as set_input_pos fields)
        self._commanded_pos_rev = np.zeros(6)      # input_pos (rev)
        self._commanded_vel_ff_rps = np.zeros(6)    # vel_ff (rev/s)
        self._commanded_torque_ff_Nm = np.zeros(6)  # torque_ff (Nm)

        # Dynamics parameters (for gravity feedforward)
        self._dynamics_params = DynamicsParams.from_config()

        # Feedforward enable flag (can be toggled via IPC for A/B testing)
        self._feedforward_enabled = True

        # Trajectory manager (Phase 4)
        self._traj_manager = TrajectoryManager(self.geom, self._dynamics_params)

        # Workspace limits (Phase 6) — precomputed from geometry
        self._workspace_limits = WorkspaceLimits.from_geometry(self.geom)
        self._workspace_status = WorkspaceStatus.OK
        self._workspace_speed_scale = 1.0
        self._cond_number = self._workspace_limits.cond_home

        # Motor feedback from CAN node (Phase 6)
        self._motor_fb_pos_rev = np.zeros(6)   # actual positions (rev)
        self._motor_fb_vel_rps = np.zeros(6)   # actual velocities (rev/s)
        self._motor_fb_cur_A = np.zeros(6)     # actual currents (A)
        self._has_motor_fb = False
        self._tracking_error_mm = np.zeros(6)

        # Gravity correction (from levelling)
        self._gravity_correction: np.ndarray | None = None

        # Fault state (Phase 6)
        self._fault_state: str | None = None

        # Logging interval
        self._last_log_time = 0.0
        self._log_interval_s = 5.0

    def run(self) -> None:
        """Main loop.  Blocks until stop() is called or signal received."""
        self._running = True
        logger.info(f"Control loop starting at {self.rate_hz} Hz "
                    f"(dt={self.dt_target*1000:.2f} ms)")
        logger.info(f"Workspace limits: leg soft=[{self._workspace_limits.leg_soft_min_mm:.1f}, "
                    f"{self._workspace_limits.leg_soft_max_mm:.1f}] mm, "
                    f"leg hard=[{self._workspace_limits.leg_hard_min_mm:.1f}, "
                    f"{self._workspace_limits.leg_hard_max_mm:.1f}] mm, "
                    f"cond soft={self._workspace_limits.cond_soft:.1f}, "
                    f"cond hard={self._workspace_limits.cond_hard:.1f}")

        t_prev = time.perf_counter()

        while self._running:
            t_start = time.perf_counter()
            dt_actual = t_start - t_prev
            t_prev = t_start

            self.stats.record(dt_actual)

            # 1. Read all pending IPC messages
            self._process_ipc()

            # 2. Poll async feasibility results
            self._poll_async_result()

            # 3. Check IPC heartbeat
            self._check_heartbeat()

            # 4. Compute control output + workspace checks
            self._compute()

            # 5. Publish telemetry
            self._publish_telemetry(dt_actual)

            # 6. Periodic logging
            self._periodic_log(t_start)

            # 7. Sleep for remainder of cycle
            elapsed = time.perf_counter() - t_start
            sleep_time = self.dt_target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self) -> None:
        """Signal the loop to stop."""
        self._running = False

    # ------------------------------------------------------------------
    # IPC dispatch
    # ------------------------------------------------------------------

    def _process_ipc(self) -> None:
        """Read and dispatch all pending IPC messages."""
        for topic, msg in self.ipc.recv_all():
            if topic == TOPIC_TARGET:
                self._on_target(msg)
            elif topic == TOPIC_MODE:
                self._on_mode_command(msg)
            elif topic == TOPIC_TRAJECTORY:
                self._on_trajectory(msg)
            elif topic == TOPIC_DYN_TARGET:
                self._on_dynamic_target(msg)
            elif topic == TOPIC_MOTOR_FB:
                self._on_motor_feedback(msg)

    def _on_target(self, msg: dict) -> None:
        """Handle an incoming target state."""
        pos = np.array(msg['pos'])
        quat = msg['rot']  # [w, x, y, z]
        rot = quat_to_rot_matrix(*quat)
        twist = np.array(msg['twist'])
        accel = np.array(msg['accel'])

        self._target_pos = pos
        self._target_rot = rot
        self._target_twist = twist
        self._target_accel = accel
        self._has_target = True

    def _on_mode_command(self, msg: dict) -> None:
        """Handle an incoming mode command."""
        cmd = msg['cmd']
        params = msg.get('params', {})
        if cmd == 'enable':
            if self.mode == ControlMode.DISABLED:
                self.mode = ControlMode.ENABLED
                self._fault_state = None
                # Initialise outputs to the home/activate pose so the first
                # telemetry cycle sends the current platform position rather
                # than stale zeros.
                self._seed_home_pose()
                logger.info("Control loop ENABLED")
        elif cmd == 'disable':
            self.mode = ControlMode.DISABLED
            self._zero_outputs()
            self._traj_manager.cancel()
            self._fault_state = None
            logger.info("Control loop DISABLED")
        elif cmd == 'estop':
            self.mode = ControlMode.ESTOP
            self._zero_outputs()
            self._traj_manager.cancel()
            self._fault_state = msg.get('params', {}).get('reason', 'external')
            logger.warning(f"E-STOP triggered via command: {self._fault_state}")
        elif cmd == 'set_feedforward':
            enabled = bool(params.get('enabled', True))
            self._feedforward_enabled = enabled
            self._traj_manager.set_feedforward_enabled(enabled)
            logger.info(f"Feedforward {'ENABLED' if enabled else 'DISABLED'}")
        elif cmd == 'set_gravity_offset':
            tx = params.get('tilt_x', 0.0)
            ty = params.get('tilt_y', 0.0)
            rotvec = np.array([-tx, -ty, 0.0])
            self._gravity_correction = rotvec_to_rot_matrix(rotvec)
            self._traj_manager.set_gravity_correction(self._gravity_correction)
            logger.info(f"Gravity correction set: tilt=[{tx:.4f}, {ty:.4f}] rad")
        elif cmd == 'fault':
            # ODrive fault forwarded from CAN node via bridge
            fault_desc = params.get('description', 'unknown fault')
            self.mode = ControlMode.ESTOP
            self._zero_outputs()
            self._traj_manager.cancel()
            self._fault_state = fault_desc
            logger.error(f"ODrive FAULT received: {fault_desc} -- E-STOP")
        else:
            logger.warning(f"Unknown mode command: {cmd}")

    def _on_trajectory(self, msg: dict) -> None:
        """Handle an incoming trajectory command."""
        try:
            traj = create_trajectory(
                start_pose=np.array(msg['start_pose']),
                start_twist=np.array(msg['start_twist']),
                start_accel=np.array(msg['start_accel']),
                end_pose=np.array(msg['end_pose']),
                end_twist=np.array(msg['end_twist']),
                end_accel=np.array(msg['end_accel']),
                duration=msg['duration'],
                t_start=time.perf_counter(),
                speed_scale=msg.get('speed_scale', 1.0),
            )
            self._traj_manager.submit(traj)
        except (ValueError, RuntimeError) as e:
            logger.error(f"Trajectory command rejected: {e}")

    def _on_dynamic_target(self, msg: dict) -> None:
        """Handle a dynamic target command.

        Queues the target for background feasibility checking via the
        async pipeline.  The control loop continues sending motor commands
        for the current trajectory while the check runs.  Results are
        picked up on the next cycle via ``_poll_async_result()``.
        """
        target_pos = np.array(msg['target_pos'])
        target_quat = np.array(msg['target_quat'])
        target_vel = np.array(msg['target_vel'])
        arrival_time = msg['arrival_time']

        t_now = time.perf_counter()
        self._traj_manager.request_dynamic_target(
            target_pos=target_pos,
            target_quat=target_quat,
            target_vel=target_vel,
            arrival_time=arrival_time,
            t_now=t_now,
        )
        logger.debug(
            f"Dynamic target queued for async check: pos={target_pos.tolist()}, "
            f"vel_norm={np.linalg.norm(target_vel):.1f} mm/s")

    def _on_motor_feedback(self, msg: dict) -> None:
        """Handle motor feedback from the CAN node (via bridge)."""
        self._motor_fb_pos_rev = np.array(msg['pos'])
        self._motor_fb_vel_rps = np.array(msg['vel'])
        self._motor_fb_cur_A = np.array(msg['cur'])
        self._has_motor_fb = True

    # ------------------------------------------------------------------
    # Heartbeat watchdog
    # ------------------------------------------------------------------

    def _check_heartbeat(self) -> None:
        """E-stop if no IPC messages received within timeout."""
        if self.mode == ControlMode.ENABLED:
            if self.ipc.seconds_since_last_recv > IPC_HEARTBEAT_TIMEOUT_S:
                self.mode = ControlMode.ESTOP
                self._zero_outputs()
                self._traj_manager.cancel()
                self._fault_state = 'ipc_heartbeat_lost'
                logger.warning(
                    f"IPC heartbeat lost ({self.ipc.seconds_since_last_recv:.1f}s "
                    f"since last message) -- E-STOP")

    # ------------------------------------------------------------------
    # Async feasibility result polling
    # ------------------------------------------------------------------

    def _poll_async_result(self) -> None:
        """Check for completed async feasibility results and commit them.

        Non-blocking.  Called once per control cycle.
        """
        if self.mode != ControlMode.ENABLED:
            return

        result = self._traj_manager.poll_pending_result()
        if result is None:
            return

        if result['accepted']:
            accepted = self._traj_manager.commit_async_trajectory(result)
            if accepted:
                logger.info("Async dynamic target committed")
            else:
                logger.debug("Async dynamic target commit failed (stale/expired)")
        else:
            logger.debug(
                f"Async dynamic target rejected: "
                f"{'; '.join(result.get('violations', []))}")

    # ------------------------------------------------------------------
    # Control computation
    # ------------------------------------------------------------------

    def _compute(self) -> None:
        """Compute control output for this cycle.

        Two modes of operation:
        1. **Trajectory mode** (Phase 4): when a trajectory is executing,
           evaluate the trajectory at the current time to get motor commands.
        2. **Direct-target mode** (Phase 3): use the latest IPC target pose
           for IK + dynamics.  Used by spacemouse, shell commands, etc.

        Phase 6 additions:
        - After computing motor commands, check workspace limits.
        - If hard limit violated, abort trajectory and E-stop.
        - Compute tracking error from motor feedback.
        - Compute condition number for telemetry.
        """
        if self.mode != ControlMode.ENABLED:
            return

        # Trajectory mode: evaluate trajectory at current time
        if self._traj_manager.state in (
                TrajectoryState.EXECUTING, TrajectoryState.RETURNING):
            t_now = time.perf_counter()
            pos, vel, torque = self._traj_manager.evaluate(t_now)
            self._commanded_pos_rev = pos
            self._commanded_vel_ff_rps = vel
            self._commanded_torque_ff_Nm = torque
        elif self._has_target:
            # Direct-target mode: use latest IPC target pose
            # Apply gravity correction if set
            effective_rot = self._target_rot
            if self._gravity_correction is not None:
                effective_rot = self._gravity_correction @ self._target_rot

            # Position IK: pose -> desired leg extensions (mm) -> motor revolutions
            desired_extensions_mm = pose_to_leg_lengths(
                self._target_pos, effective_rot, self.geom)
            self._commanded_pos_rev = extensions_mm_to_revs(
                desired_extensions_mm, self.geom)

            # Velocity IK: twist -> desired leg velocities (mm/s) -> motor rev/s
            desired_vel_mm_s = twist_to_leg_velocities(
                self._target_twist, self._target_pos, effective_rot, self.geom)
            self._commanded_vel_ff_rps = leg_velocities_to_motor_velocities(
                desired_vel_mm_s, self.geom)

            # Feedforward: gravity + inertia -> motor torques (Nm)
            if self._feedforward_enabled:
                self._commanded_torque_ff_Nm = compute_full_feedforward_torques(
                    self._target_pos, effective_rot,
                    self._target_twist, self._target_accel,
                    self.geom, self._dynamics_params)
            else:
                self._commanded_torque_ff_Nm = np.zeros(6)
        else:
            return

        # --- Phase 6: workspace limit check ---
        # Get current pose for condition number computation.
        # Use the trajectory manager's current_pose_6dof which is set
        # during evaluate() above, or construct from target for direct mode.
        if self._traj_manager.state in (
                TrajectoryState.EXECUTING, TrajectoryState.RETURNING):
            pose_6dof = self._traj_manager.current_pose_6dof
        elif self._has_target:
            # For direct-target mode, we don't have a rotvec handy,
            # but we have pos and rot matrix. Use commanded extensions.
            pose_6dof = None  # skip cond check for direct mode (no rotvec)
        else:
            pose_6dof = None

        # Compute leg extensions from commanded positions
        commanded_extensions_mm = revs_to_extensions_mm(
            self._commanded_pos_rev, self.geom)

        # Condition number (only computed during trajectory mode for performance)
        if pose_6dof is not None:
            pos_cart = pose_6dof[:3]
            rot_mat = rotvec_to_rot_matrix(pose_6dof[3:6])
            self._cond_number = compute_condition_number(pos_cart, rot_mat, self.geom)
        else:
            # For direct-target mode, use target pos/rot (with gravity correction)
            if self._has_target:
                cond_rot = self._target_rot
                if self._gravity_correction is not None:
                    cond_rot = self._gravity_correction @ self._target_rot
                self._cond_number = compute_condition_number(
                    self._target_pos, cond_rot, self.geom)

        # Check workspace limits
        ws_check = check_workspace_limits(
            commanded_extensions_mm,
            self._cond_number,
            self._workspace_limits,
        )
        self._workspace_status = ws_check.status
        self._workspace_speed_scale = ws_check.speed_scale

        if ws_check.status == WorkspaceStatus.HARD_LIMIT:
            # Hard limit violated: abort trajectory, E-stop
            violation_str = '; '.join(ws_check.violations)
            self._traj_manager.cancel()
            self.mode = ControlMode.ESTOP
            self._zero_outputs()
            self._fault_state = f'workspace_hard_limit: {violation_str}'
            logger.error(f"WORKSPACE HARD LIMIT -- E-STOP: {violation_str}")
        elif ws_check.status == WorkspaceStatus.SOFT_LIMIT:
            # Soft limit: log warning (speed degradation is informational
            # for now -- Phase 7 will use it for mid-trajectory replanning)
            logger.debug(
                f"Workspace soft limit: speed_scale={ws_check.speed_scale:.2f}, "
                f"{'; '.join(ws_check.violations)}")

        # --- Phase 6: tracking error from motor feedback ---
        if self._has_motor_fb:
            # Convert commanded and actual positions to mm for tracking error
            actual_extensions_mm = revs_to_extensions_mm(
                self._motor_fb_pos_rev, self.geom)
            self._tracking_error_mm = np.abs(
                commanded_extensions_mm - actual_extensions_mm)

    def _zero_outputs(self) -> None:
        """Zero all control outputs (safe state)."""
        self._commanded_pos_rev = np.zeros(6)
        self._commanded_vel_ff_rps = np.zeros(6)
        self._commanded_torque_ff_Nm = np.zeros(6)

    def _seed_home_pose(self) -> None:
        """Initialise control outputs to the home/activate pose.

        Called on enable so the first telemetry cycle holds the platform
        at its current position instead of sending zeros.
        """
        home = self._traj_manager.home_pose  # [x,y,z,rx,ry,rz]
        rot = rotvec_to_rot_matrix(home[3:6])
        if self._gravity_correction is not None:
            rot = self._gravity_correction @ rot

        extensions_mm = pose_to_leg_lengths(home[:3], rot, self.geom)
        self._commanded_pos_rev = extensions_mm_to_revs(extensions_mm, self.geom)
        self._commanded_vel_ff_rps = np.zeros(6)

        if self._feedforward_enabled:
            self._commanded_torque_ff_Nm = gravity_to_motor_torques(
                home[:3], rot, self.geom, self._dynamics_params)
        else:
            self._commanded_torque_ff_Nm = np.zeros(6)

        self._traj_manager.set_hold_pose(home)
        self._target_pos = home[:3].copy()
        self._target_rot = rot
        self._target_twist = np.zeros(6)
        self._target_accel = np.zeros(6)
        self._has_target = True

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def _publish_telemetry(self, dt_actual: float) -> None:
        """Send telemetry to the bridge."""
        msg = make_telemetry(
            leg_positions=self._commanded_pos_rev.tolist(),
            leg_velocities=self._commanded_vel_ff_rps.tolist(),
            commanded_torques=self._commanded_torque_ff_Nm.tolist(),
            loop_dt_s=dt_actual,
            ff_torques=self._commanded_torque_ff_Nm.tolist(),
            traj_state=self._traj_manager.state.value,
            traj_progress=self._traj_manager.progress,
            cond_number=self._cond_number,
            workspace_status=self._workspace_status.value,
            workspace_speed_scale=self._workspace_speed_scale,
            tracking_error_mm=(self._tracking_error_mm.tolist()
                               if self._has_motor_fb else None),
            fault_state=self._fault_state,
            motor_pos=(self._motor_fb_pos_rev.tolist()
                       if self._has_motor_fb else None),
            motor_vel=(self._motor_fb_vel_rps.tolist()
                       if self._has_motor_fb else None),
            motor_cur=(self._motor_fb_cur_A.tolist()
                       if self._has_motor_fb else None),
        )
        self.ipc.send_telemetry(msg)

    def _periodic_log(self, t_now: float) -> None:
        """Log timing statistics periodically."""
        if t_now - self._last_log_time >= self._log_interval_s:
            self._last_log_time = t_now
            if self.stats.count > 0:
                logger.info(f"Loop timing: {self.stats.summary()}")
                if self.mode == ControlMode.ENABLED:
                    logger.info(
                        f"  cond#={self._cond_number:.1f}  "
                        f"ws={self._workspace_status.value}  "
                        f"ws_scale={self._workspace_speed_scale:.2f}  "
                        f"traj={self._traj_manager.state.value}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Jugglebot motion control process")
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ,
                        help=f"Control loop rate in Hz (default: {DEFAULT_RATE_HZ})")
    parser.add_argument('--log-level', default='INFO',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'])
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )

    geom = StewartGeometry()
    loop = ControlLoop(target_rate_hz=args.rate, geom=geom)

    # Graceful shutdown on SIGINT/SIGTERM
    def signal_handler(sig, frame):
        logger.info("Shutdown signal received")
        loop.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        loop.run()
    finally:
        loop._traj_manager.shutdown()
        loop.ipc.close()
        logger.info(f"Final loop timing: {loop.stats.summary()}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
