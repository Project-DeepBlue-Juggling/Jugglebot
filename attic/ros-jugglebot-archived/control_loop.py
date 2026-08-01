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

Safety hardening:
  - Slew limiter: per-cycle rate limit on commanded positions vs actual
    motor feedback.  Prevents step discontinuities regardless of source.
  - Motor feedback staleness: suppresses all commands if feedback is
    older than JB_OP_MOTOR_FB_STALENESS_TIMEOUT_S.
  - Motor overspeed: ESTOP if any motor exceeds velocity limit.
  - Tracking error: logged if any leg deviates > MAX_TRACKING_ERROR_MM.

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
from collections import deque
from dataclasses import dataclass, field

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    compute_jacobian,
    pose_to_leg_lengths,
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
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
    TOPIC_MPC_CMD,
    TOPIC_MOTOR_FB,
    TOPIC_TARGET,
    TOPIC_TRAJECTORY,
    ControlProcessIPC,
    make_telemetry,
)
from jugglebot.motion.stream_smoother import StreamSmoother
from jugglebot.motion.motor_commands import cartesian_to_motor_commands
from jugglebot.motion.quintic import create_trajectory
from jugglebot.motion.trajectory_manager import TrajectoryManager, TrajectoryState
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

# Slew limiter: maximum motor velocity (rev/s) enforced on commanded positions.
# Conservative limit: ~667 mm/s Cartesian ≈ 200 mm in 0.3 s.
MAX_SLEW_RATE_REV_PER_S = 9.5

# MPC period used for slew budget in MPC pass-through mode.
# The MPC runs at 50 Hz; commands arrive as 20 ms position steps.
# Using the MPC period (instead of the loop dt) allows the full step
# to pass through unclamped as long as velocity is within limits.
MPC_SLEW_DT_S = 0.02

# Trigger FAULT if slew limiter clamps for this many consecutive seconds.
SLEW_FAULT_DURATION_S = 0.5

# Motor overspeed threshold — ESTOP if any motor exceeds this (rev/s).
# 10 % margin above hardware limit for measurement noise.
MAX_MOTOR_VEL_RPS = hw.ODRIVE_TRAP_VEL_LIMIT_RPS * 1.1

# Tracking error threshold (mm) — ESTOP if any leg exceeds this.
MAX_TRACKING_ERROR_MM = 10.0

# Motor feedback staleness timeout (seconds).  If no fresh motor-feedback
# message has arrived within this window, the control loop suppresses ALL
# motor commands because it cannot verify that its outputs are safe.
# 150 ms gives ~50% headroom over the 100 Hz feedback rate (10ms period),
# tolerating a single dropped packet plus typical IPC latency (~1ms).
MOTOR_FB_STALENESS_S = 0.15

# MPC command staleness timeout (seconds).  If no fresh MPC command has
# arrived within this window while in MPC pass-through mode, the control
# loop triggers E-STOP.  The MPC runs at 50 Hz (20 ms period), so 200 ms
# is 10× the expected period — generous for solve time jitter while still
# catching a crashed MPC within ~200 ms.
MPC_CMD_STALENESS_S = 0.2


# ---------------------------------------------------------------------------
# Timing statistics
# ---------------------------------------------------------------------------

@dataclass
class LoopStats:
    """Accumulates per-cycle timing data for jitter analysis."""
    target_dt_s: float
    cycle_times: deque = field(default=None)
    _window_size: int = 10000  # keep last N cycles

    def __post_init__(self):
        if self.cycle_times is None:
            self.cycle_times = deque(maxlen=self._window_size)

    def record(self, dt_s: float) -> None:
        self.cycle_times.append(dt_s)

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

        # Stream smoother — universal C2-continuous gate for all direct targets
        # Initialised with spacemouse limits (most common); updated on enable
        self._smoother = StreamSmoother(
            self.geom,
            vel_limit_rps=hw.SPACEMOUSE_SMOOTHER_VEL_LIMIT_RPS,
            accel_limit_rps2=hw.SPACEMOUSE_SMOOTHER_ACCEL_LIMIT_RPS2)
        # Base smoother limits (before workspace speed scaling)
        self._base_smoother_vel = hw.SPACEMOUSE_SMOOTHER_VEL_LIMIT_RPS
        self._base_smoother_accel = hw.SPACEMOUSE_SMOOTHER_ACCEL_LIMIT_RPS2

        # Track trajectory state transitions (for smoother reset on completion)
        self._prev_traj_state = TrajectoryState.IDLE

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
        self._motor_fb_timestamp = 0.0         # perf_counter time of last feedback
        self._tracking_error_mm = np.zeros(6)

        # Slew limiter state
        self._slew_fault_cycles = int(SLEW_FAULT_DURATION_S * target_rate_hz)
        self._slew_limit_count = 0
        self._slew_limited = False

        # MPC pass-through state (Phase 6 hardware bridge)
        self._mpc_passthrough_active = False
        self._mpc_cmd_ext_mm = np.zeros(6)
        self._mpc_cmd_vel_mm_s = np.zeros(6)
        self._mpc_cmd_torque_Nm = np.zeros(6)
        self._mpc_cmd_pose_6dof = np.zeros(6)
        self._has_mpc_cmd = False
        self._mpc_cmd_timestamp = 0.0
        self._mpc_cmd_seq = -1
        self._mpc_cmd_new = False  # True when an unforwarded command is waiting
        self._mpc_cmd_motor_rev: np.ndarray | None = None  # pre-computed motor revolutions
        self._mpc_prev_ext_mm: np.ndarray | None = None  # for vel_ff finite-diff
        self._mpc_prev_timestamp = 0.0
        self._has_new_output = False  # gates telemetry publishing in MPC mode
        # Linear interpolation state: base position + velocity from last MPC update
        self._mpc_base_pos_rev = np.zeros(6)
        self._mpc_base_vel_rps = np.zeros(6)
        self._mpc_base_timestamp = 0.0

        # Gravity correction (from levelling)
        self._gravity_correction: np.ndarray | None = None

        # Fault state (Phase 6)
        self._fault_state: str | None = None

        # Workspace clamping for direct-target mode (spacemouse etc.)
        # When workspace violation occurs, hold this pose instead of E-STOPing.
        self._last_safe_pose_6dof: np.ndarray | None = None
        self._workspace_clamped = False

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

            # 5. Slew limiter — final safety gate on motor commands
            ok_to_publish = self._slew_limit(dt_actual)

            # 6. Publish telemetry.  Motor commands are only included when
            #    ENABLED and the safety gate approves.  In ESTOP/DISABLED,
            #    a reduced telemetry message (with fault_state) is sent so
            #    the bridge has visibility into the control loop state.
            if self.mode == ControlMode.ENABLED and ok_to_publish:
                # Publish when we have a computed output this cycle.
                # In MPC mode, the interpolator produces output every cycle.
                if self._has_new_output:
                    self._publish_telemetry(dt_actual)
            elif self.mode == ControlMode.ESTOP:
                self._publish_fault_telemetry(dt_actual)

            # 7. Periodic logging
            self._periodic_log(t_start)

            # 8. Sleep for remainder of cycle
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
            elif topic == TOPIC_MPC_CMD:
                self._on_mpc_command(msg)

    def _on_target(self, msg: dict) -> None:
        """Handle an incoming target state.

        Converts the quaternion target to a 6-DoF pose (rotvec) and feeds
        it into the stream smoother for C2-continuous profiling.
        """
        if self._mpc_passthrough_active:
            return  # MPC is sole command source; ignore Cartesian targets
        try:
            pos = np.array(msg['pos'], dtype=float)
            quat = msg['rot']  # [w, x, y, z]
            if pos.shape != (3,):
                logger.warning(f"Malformed target: pos shape {pos.shape}, expected (3,)")
                return
            if len(quat) != 4:
                logger.warning(f"Malformed target: rot length {len(quat)}, expected 4")
                return
            rot = quat_to_rot_matrix(*quat)
        except (KeyError, TypeError, ValueError) as e:
            logger.warning(f"Malformed target message, ignoring: {e}")
            return

        # Convert to 6-DoF pose for the smoother: [x, y, z, rx, ry, rz]
        rotvec = rot_matrix_to_rotvec(rot)
        pose_6dof = np.concatenate([pos, rotvec])

        # Feed the smoother (computes duration from motor-space displacement)
        self._smoother.set_target(pose_6dof, time.perf_counter())

        # Keep raw target storage for backward compat / telemetry
        self._target_pos = pos
        self._target_rot = rot
        self._has_target = True

    def _on_mode_command(self, msg: dict) -> None:
        """Handle an incoming mode command."""
        try:
            cmd = msg['cmd']
        except (KeyError, TypeError) as e:
            logger.warning(f"Malformed mode command, ignoring: {e}")
            return
        params = msg.get('params', {})
        if cmd == 'enable':
            source = params.get('source', '')
            if self.mode == ControlMode.DISABLED:
                self.mode = ControlMode.ENABLED
                self._fault_state = None
                self._mpc_passthrough_active = (source == 'MPC')
                if self._mpc_passthrough_active:
                    self._has_mpc_cmd = False
                    self._mpc_cmd_new = False
                    self._mpc_prev_ext_mm = None
                    self._mpc_cmd_timestamp = time.perf_counter()
                else:
                    self._apply_smoother_limits(source)
                # Initialise outputs to the home/activate pose so the first
                # telemetry cycle sends the current platform position rather
                # than stale zeros.
                self._seed_home_pose()
                logger.info(
                    f"Control loop ENABLED (source={source}"
                    f"{', mpc_passthrough=True' if self._mpc_passthrough_active else ''})")
            elif self.mode == ControlMode.ENABLED and source == 'MPC':
                # Already enabled — switch to MPC pass-through mode.
                # This allows the orchestrator to activate the robot first
                # (putting ODrives in CLOSED_LOOP), then the HardwarePlant
                # takes over as the command source.  The slew limiter
                # ensures smooth transition regardless of any position
                # difference between the smoother's current output and the
                # first MPC command.
                if not self._mpc_passthrough_active:
                    self._mpc_passthrough_active = True
                    self._has_mpc_cmd = False
                    self._mpc_cmd_new = False
                    self._mpc_prev_ext_mm = None
                    self._mpc_cmd_timestamp = time.perf_counter()
                    logger.info("Switched to MPC pass-through mode (was already ENABLED)")
        elif cmd == 'disable':
            self.mode = ControlMode.DISABLED
            self._zero_outputs()
            self._traj_manager.cancel()
            self._smoother.reset(self._traj_manager.home_pose)
            self._mpc_passthrough_active = False
            self._has_mpc_cmd = False
            self._mpc_cmd_new = False
            self._mpc_prev_ext_mm = None
            self._fault_state = None
            logger.info("Control loop DISABLED")
        elif cmd == 'estop':
            if self.mode != ControlMode.ESTOP:
                self.mode = ControlMode.ESTOP
                self._zero_outputs()
                self._traj_manager.cancel()
                self._smoother.reset(self._traj_manager.home_pose)
                self._mpc_passthrough_active = False
                self._has_mpc_cmd = False
                self._mpc_cmd_new = False
                self._mpc_prev_ext_mm = None
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
        elif cmd == 'set_smoother_limits':
            vel = params.get('vel_limit_rps')
            accel = params.get('accel_limit_rps2')
            if vel is not None and accel is not None:
                self._base_smoother_vel = float(vel)
                self._base_smoother_accel = float(accel)
                self._smoother.set_limits(float(vel), float(accel))
                logger.info(f"Smoother limits updated: vel={vel} rps, accel={accel} rps²")
        elif cmd == 'fault':
            # ODrive fault forwarded from CAN node via bridge
            fault_desc = params.get('description', 'unknown fault')
            self.mode = ControlMode.ESTOP
            self._zero_outputs()
            self._traj_manager.cancel()
            self._smoother.reset(self._traj_manager.home_pose)
            self._mpc_passthrough_active = False
            self._has_mpc_cmd = False
            self._mpc_cmd_new = False
            self._mpc_prev_ext_mm = None
            self._fault_state = fault_desc
            logger.error(f"ODrive FAULT received: {fault_desc} -- E-STOP")
        else:
            logger.warning(f"Unknown mode command: {cmd}")

    def _on_trajectory(self, msg: dict) -> None:
        """Handle an incoming trajectory command."""
        if self._mpc_passthrough_active:
            return  # MPC is sole command source; ignore trajectory commands
        try:
            arrays = {}
            for key in ('start_pose', 'start_twist', 'start_accel',
                        'end_pose', 'end_twist', 'end_accel'):
                arr = np.array(msg[key], dtype=float)
                if arr.shape != (6,):
                    logger.error(f"Trajectory rejected: {key} shape {arr.shape}, expected (6,)")
                    return
                arrays[key] = arr
            traj = create_trajectory(
                start_pose=arrays['start_pose'],
                start_twist=arrays['start_twist'],
                start_accel=arrays['start_accel'],
                end_pose=arrays['end_pose'],
                end_twist=arrays['end_twist'],
                end_accel=arrays['end_accel'],
                duration=msg['duration'],
                t_start=time.perf_counter(),
                speed_scale=msg.get('speed_scale', 1.0),
            )
            self._traj_manager.submit(traj)
        except (KeyError, TypeError, ValueError, RuntimeError) as e:
            logger.error(f"Trajectory command rejected: {e}")

    def _on_dynamic_target(self, msg: dict) -> None:
        """Handle a dynamic target command.

        Queues the target for background feasibility checking via the
        async pipeline.  The control loop continues sending motor commands
        for the current trajectory while the check runs.  Results are
        picked up on the next cycle via ``_poll_async_result()``.
        """
        if self._mpc_passthrough_active:
            return  # MPC is sole command source; ignore dynamic targets
        try:
            target_pos = np.array(msg['target_pos'], dtype=float)
            target_quat = np.array(msg['target_quat'], dtype=float)
            target_vel = np.array(msg['target_vel'], dtype=float)
            arrival_time = msg['arrival_time']
        except (KeyError, TypeError, ValueError) as e:
            logger.warning(f"Malformed dynamic target message, ignoring: {e}")
            return
        if target_pos.shape != (3,):
            logger.warning(f"Malformed dynamic target: target_pos shape {target_pos.shape}, expected (3,)")
            return
        if target_quat.shape != (4,):
            logger.warning(f"Malformed dynamic target: target_quat shape {target_quat.shape}, expected (4,)")
            return
        if target_vel.shape != (3,):
            logger.warning(f"Malformed dynamic target: target_vel shape {target_vel.shape}, expected (3,)")
            return

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
        try:
            pos = np.array(msg['pos'], dtype=float)
            vel = np.array(msg['vel'], dtype=float)
            cur = np.array(msg['cur'], dtype=float)
        except (KeyError, TypeError, ValueError) as e:
            logger.warning(f"Malformed motor feedback message, ignoring: {e}")
            return
        if pos.shape != (6,) or vel.shape != (6,) or cur.shape != (6,):
            logger.warning(
                f"Malformed motor feedback: shapes pos={pos.shape} vel={vel.shape} "
                f"cur={cur.shape}, expected (6,)")
            return
        self._motor_fb_pos_rev = pos
        self._motor_fb_vel_rps = vel
        self._motor_fb_cur_A = cur
        self._has_motor_fb = True
        self._motor_fb_timestamp = time.perf_counter()

    def _on_mpc_command(self, msg: dict) -> None:
        """Handle pre-computed motor commands from an external MPC controller.

        In MPC pass-through mode, the control loop receives leg extensions
        (mm) directly — no IK or stream smoother needed.  The Cartesian
        pose is included for workspace/condition-number checks.
        """
        if not self._mpc_passthrough_active:
            return  # Silently ignore if not in MPC mode
        try:
            ext = np.array(msg['ext_mm'], dtype=float)
            pose = np.array(msg['pose_6dof'], dtype=float)
            if ext.shape != (6,):
                logger.warning(f"Malformed mpc_cmd: ext_mm shape {ext.shape}")
                return
            if pose.shape != (6,):
                logger.warning(f"Malformed mpc_cmd: pose_6dof shape {pose.shape}")
                return
        except (KeyError, TypeError, ValueError) as e:
            logger.warning(f"Malformed mpc_cmd message, ignoring: {e}")
            return

        self._mpc_cmd_ext_mm = ext
        self._mpc_cmd_pose_6dof = pose

        # Motor revolutions (absolute, ODrive convention: 0 = STOW).
        # After STOW-zero refactor, motor_rev = ext_mm × mm_to_rev directly.
        # When present, used as-is to avoid redundant multiplication.
        motor_rev = msg.get('motor_rev')
        if motor_rev is not None:
            self._mpc_cmd_motor_rev = np.array(motor_rev, dtype=float)
        else:
            self._mpc_cmd_motor_rev = None

        torque = msg.get('torque_Nm')
        if torque is not None:
            self._mpc_cmd_torque_Nm = np.array(torque, dtype=float)
        else:
            self._mpc_cmd_torque_Nm = np.zeros(6)

        self._mpc_cmd_seq = msg.get('seq', -1)
        self._mpc_cmd_timestamp = time.perf_counter()
        self._has_mpc_cmd = True

        # Compute vel_ff via finite-difference of consecutive extensions.
        # This replaces the IPC-carried vel_mm_s (which may be zero if the
        # sender doesn't provide twist information).
        if self._mpc_prev_ext_mm is not None:
            dt_mpc = self._mpc_cmd_timestamp - self._mpc_prev_timestamp
            if dt_mpc > 1e-6:
                self._mpc_cmd_vel_mm_s = (ext - self._mpc_prev_ext_mm) / dt_mpc
            else:
                self._mpc_cmd_vel_mm_s = np.zeros(6)
        else:
            # First command — no previous, vel_ff = 0
            self._mpc_cmd_vel_mm_s = np.zeros(6)

        self._mpc_prev_ext_mm = ext.copy()
        self._mpc_prev_timestamp = self._mpc_cmd_timestamp
        self._mpc_cmd_new = True

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
    # Slew limiter + safety checks
    # ------------------------------------------------------------------

    def _slew_limit(self, dt: float) -> bool:
        """Final safety gate: limit rate of position change vs actual motor
        position.  Returns True if commands should be published, False to
        suppress.

        When motor feedback is available: enforces slew rate, overspeed,
        tracking error, and staleness checks.  When feedback is NOT
        available: all commands are suppressed until feedback arrives.
        """
        if self.mode != ControlMode.ENABLED:
            return False  # Only check safety when actively commanding motors

        if not self._has_motor_fb:
            return False  # No feedback yet — cannot verify safety

        fb_age = time.perf_counter() - self._motor_fb_timestamp
        if fb_age > MOTOR_FB_STALENESS_S:
            logger.warning(
                f"Motor feedback stale ({fb_age:.3f}s) — suppressing commands")
            # Reset tracking error so stale values don't trigger false warnings
            # when feedback resumes.
            self._tracking_error_mm = np.zeros(6)
            return False

        # --- Motor overspeed check ---
        if np.any(np.abs(self._motor_fb_vel_rps) > MAX_MOTOR_VEL_RPS):
            worst = np.max(np.abs(self._motor_fb_vel_rps))
            self.mode = ControlMode.ESTOP
            self._zero_outputs()
            self._traj_manager.cancel()
            self._fault_state = f'motor_overspeed: {worst:.2f} rev/s'
            logger.error(
                f"MOTOR OVERSPEED — E-STOP: worst={worst:.2f} rev/s "
                f"(limit {MAX_MOTOR_VEL_RPS:.1f})")
            return False

        # --- Tracking error check ---
        # Large tracking errors are logged but do NOT trigger E-STOP.
        # The slew limiter already rate-limits motor commands, so the motors
        # will naturally catch up.  Freezing the platform (E-STOP) is worse
        # than allowing recovery, especially during manual spacemouse control.
        if np.any(self._tracking_error_mm > MAX_TRACKING_ERROR_MM):
            worst = np.max(self._tracking_error_mm)
            worst_leg = int(np.argmax(self._tracking_error_mm))
            logger.warning(
                f"Large tracking error: leg {worst_leg} = {worst:.2f} mm "
                f"(threshold {MAX_TRACKING_ERROR_MM:.1f})")

        # --- Slew rate limit ---
        # With the MPC interpolator upsampling to 500 Hz, each cycle's
        # position step is a small increment (not a 20 ms MPC jump), so
        # the standard per-cycle budget applies uniformly to all modes.
        actual = self._motor_fb_pos_rev
        delta = self._commanded_pos_rev - actual
        max_delta = MAX_SLEW_RATE_REV_PER_S * dt

        if np.any(np.abs(delta) > max_delta):
            clamped_delta = np.clip(delta, -max_delta, max_delta)
            self._commanded_pos_rev = actual + clamped_delta
            # Scale vel_ff proportionally to how much each leg was clamped.
            # This keeps the feedforward direction correct while reducing
            # magnitude.  torque_ff (gravity comp) is left unchanged —
            # zeroing it removed gravity compensation and caused oscillation.
            abs_delta = np.abs(delta)
            scale = np.where(abs_delta > 1e-9,
                             np.abs(clamped_delta) / abs_delta,
                             1.0)
            self._commanded_vel_ff_rps *= scale
            self._slew_limited = True
            self._slew_limit_count += 1
            # Only log on first activation to avoid spamming at 500 Hz
            if self._slew_limit_count == 1:
                logger.warning(
                    f"Slew limiter active: worst_delta="
                    f"{np.max(np.abs(delta)):.4f} rev, clamped to "
                    f"{max_delta:.4f} rev")

            if self._slew_limit_count == self._slew_fault_cycles:
                logger.warning(
                    f"Slew limiter sustained for {self._slew_limit_count} "
                    f"cycles — motors are chasing target")
        else:
            self._slew_limit_count = 0
            self._slew_limited = False

        return True

    # ------------------------------------------------------------------
    # Async feasibility result polling
    # ------------------------------------------------------------------

    def _poll_async_result(self) -> None:
        """Check for completed async feasibility results and commit them.

        Non-blocking.  Called once per control cycle.  If the feasibility
        worker process has crashed, attempts a restart up to the
        configured limit before triggering an E-STOP.
        """
        if self.mode != ControlMode.ENABLED:
            return

        from jugglebot.motion.feasibility_worker import WorkerCrashed
        try:
            result = self._traj_manager.poll_pending_result()
        except WorkerCrashed:
            logger.error("Feasibility worker crashed — attempting restart")
            if self._traj_manager.try_restart_worker():
                logger.info("Feasibility worker restarted successfully")
            else:
                self.mode = ControlMode.ESTOP
                self._zero_outputs()
                self._traj_manager.cancel()
                self._fault_state = 'feasibility_worker_crash'
                logger.error(
                    "Feasibility worker restart limit exceeded — E-STOP")
            return

        if result is None:
            return

        arrival_time = result.get('arrival_time', 0.0)

        if result['accepted']:
            accepted = self._traj_manager.commit_async_trajectory(result)
            if accepted:
                logger.info("Async dynamic target committed")
                self._send_dynamic_feedback(True, arrival_time)
            else:
                logger.debug("Async dynamic target commit failed (stale/expired)")
                self._send_dynamic_feedback(False, arrival_time, ['stale_or_expired'])
        else:
            violations = result.get('violations', [])
            logger.debug(
                f"Async dynamic target rejected: "
                f"{'; '.join(violations)}")
            self._send_dynamic_feedback(False, arrival_time, violations)

    def _send_dynamic_feedback(
        self, accepted: bool, arrival_time: float,
        violations: list[str] | None = None,
    ) -> None:
        """Send accept/reject feedback for a dynamic target via IPC."""
        from jugglebot.motion.ipc import make_dynamic_target_feedback
        msg = make_dynamic_target_feedback(
            accepted=accepted,
            arrival_time=arrival_time,
            violations=violations,
        )
        self.ipc.send_dynamic_feedback(msg)

    # ------------------------------------------------------------------
    # Control computation
    # ------------------------------------------------------------------

    def _compute(self) -> None:
        """Compute control output for this cycle.

        Three modes of operation:
        1. **Trajectory mode** (Phase 4): when a trajectory is executing,
           evaluate the trajectory at the current time to get motor commands.
        2. **MPC pass-through mode** (Phase 6): receive pre-computed leg
           extensions from an external MPC controller.  No IK or stream
           smoother — just unit conversion + safety pipeline.
        3. **Direct-target mode** (Phase 3): use the latest IPC target pose
           for IK + dynamics.  Used by spacemouse, shell commands, etc.

        Phase 6 additions:
        - After computing motor commands, check workspace limits.
        - If hard limit violated in trajectory/MPC mode, abort and E-stop.
        - If hard limit violated in direct-target mode, hold last safe
          pose (no E-stop) so the spacemouse can recover.
        - Compute tracking error from motor feedback.
        - Compute condition number for telemetry.
        """
        if self.mode != ControlMode.ENABLED:
            return

        self._has_new_output = False

        # Track trajectory state transitions for smoother reset
        traj_state = self._traj_manager.state
        if (traj_state == TrajectoryState.IDLE
                and self._prev_traj_state != TrajectoryState.IDLE):
            # Trajectory just completed — reset smoother to end pose
            end_pose = self._traj_manager.current_pose_6dof
            if end_pose is not None:
                self._smoother.reset(end_pose)
        self._prev_traj_state = traj_state

        # Trajectory mode: evaluate trajectory at current time
        if traj_state in (
                TrajectoryState.EXECUTING, TrajectoryState.RETURNING):
            t_now = time.perf_counter()
            pos, vel, torque = self._traj_manager.evaluate(t_now)
            self._commanded_pos_rev = pos
            self._commanded_vel_ff_rps = vel
            self._commanded_torque_ff_Nm = torque
            self._has_new_output = True
            pose_6dof = self._traj_manager.current_pose_6dof
            cached_J = self._traj_manager.last_jacobian
        elif self._mpc_passthrough_active and self._has_mpc_cmd:
            # MPC pass-through with linear interpolation.
            #
            # The MPC sends commands at 50 Hz but the ODrive needs frequent
            # position updates in PASSTHROUGH mode.  Rather than forwarding
            # once and letting vel_ff persist (which causes overshoot), we
            # linearly interpolate between MPC updates:
            #
            #   pos(t) = mpc_pos + vel_ff × (t - t_mpc_cmd)
            #
            # This upsamples the 50 Hz MPC trajectory to the control loop
            # rate (500 Hz), giving the ODrive smooth position ramps with
            # matching vel_ff and torque_ff.  No IK or dynamics needed —
            # just one multiply-add per cycle.
            t_now = time.perf_counter()

            # Staleness check: runs every cycle regardless of new-command flag
            mpc_age = t_now - self._mpc_cmd_timestamp
            if mpc_age > MPC_CMD_STALENESS_S:
                self.mode = ControlMode.ESTOP
                self._zero_outputs()
                self._mpc_passthrough_active = False
                self._has_mpc_cmd = False
                self._mpc_cmd_new = False
                self._mpc_prev_ext_mm = None
                self._fault_state = (
                    f'mpc_cmd_stale: {mpc_age:.3f}s since last command')
                logger.error(
                    f"MPC command stale ({mpc_age:.3f}s > "
                    f"{MPC_CMD_STALENESS_S}s) — E-STOP")
                return

            # Latch the base position and vel_ff when a new MPC command arrives.
            if self._mpc_cmd_new:
                self._mpc_cmd_new = False
                if self._mpc_cmd_motor_rev is not None:
                    self._mpc_base_pos_rev = self._mpc_cmd_motor_rev.copy()
                else:
                    self._mpc_base_pos_rev = self._mpc_cmd_ext_mm * self.geom.mm_to_rev
                self._mpc_base_vel_rps = leg_velocities_to_motor_velocities(
                    self._mpc_cmd_vel_mm_s, self.geom)
                self._mpc_base_timestamp = self._mpc_cmd_timestamp

            # Linearly extrapolate from the last MPC command.
            dt_since_cmd = t_now - self._mpc_base_timestamp
            self._commanded_pos_rev = (
                self._mpc_base_pos_rev + self._mpc_base_vel_rps * dt_since_cmd)
            self._commanded_vel_ff_rps = self._mpc_base_vel_rps.copy()
            # Torques (gravity comp) are static — pass through unchanged.
            self._commanded_torque_ff_Nm = self._mpc_cmd_torque_Nm.copy()

            self._has_new_output = True

            # Compute Jacobian from MPC-provided pose for condition number
            pose_6dof = self._mpc_cmd_pose_6dof
            pos_cart = pose_6dof[:3]
            rot_mat = rotvec_to_rot_matrix(pose_6dof[3:6])
            cached_J = compute_jacobian(pos_cart, rot_mat, self.geom)
        elif self._has_target:
            # Direct-target mode: evaluate the C2-continuous smoother
            t_now = time.perf_counter()
            pose, twist, accel = self._smoother.evaluate(t_now)

            pos_rev, vel_ff, torque_ff, cached_J = cartesian_to_motor_commands(
                pose, twist, accel,
                self.geom, self._dynamics_params,
                self._feedforward_enabled, self._gravity_correction)

            self._commanded_pos_rev = pos_rev
            self._commanded_vel_ff_rps = vel_ff
            self._commanded_torque_ff_Nm = torque_ff
            self._has_new_output = True
            pose_6dof = pose
        else:
            return

        # --- Phase 6: workspace limit check ---
        # Compute leg extensions from commanded positions
        commanded_extensions_mm = revs_to_extensions_mm(
            self._commanded_pos_rev, self.geom)

        # Condition number from 6-DoF pose (available in both modes now)
        if pose_6dof is not None:
            pos_cart = pose_6dof[:3]
            rot_mat = rotvec_to_rot_matrix(pose_6dof[3:6])
            self._cond_number = compute_condition_number(
                pos_cart, rot_mat, self.geom, J=cached_J)

        # Check workspace limits
        ws_check = check_workspace_limits(
            commanded_extensions_mm,
            self._cond_number,
            self._workspace_limits,
        )
        self._workspace_status = ws_check.status
        self._workspace_speed_scale = ws_check.speed_scale

        # Determine whether we're in direct-target mode (spacemouse, shell, etc.)
        # MPC pass-through is NOT direct mode — it should E-stop on hard limits,
        # not hold-pose, because MPC is autonomous (not human-in-the-loop).
        in_direct_mode = (traj_state == TrajectoryState.IDLE
                          and self._has_target
                          and not self._mpc_passthrough_active)

        if ws_check.status == WorkspaceStatus.HARD_LIMIT:
            if in_direct_mode and self._last_safe_pose_6dof is not None:
                # Direct-target mode: hold last safe pose instead of E-STOPing.
                # The smoother keeps tracking the spacemouse input, so when the
                # user moves back to a valid region, we resume seamlessly.
                held = self._last_safe_pose_6dof
                pos_rev, vel_ff, torque_ff, _ = cartesian_to_motor_commands(
                    held, np.zeros(6), np.zeros(6),
                    self.geom, self._dynamics_params,
                    self._feedforward_enabled, self._gravity_correction)
                self._commanded_pos_rev = pos_rev
                self._commanded_vel_ff_rps = vel_ff
                self._commanded_torque_ff_Nm = torque_ff
                if not self._workspace_clamped:
                    logger.warning(
                        f"Workspace hard limit in direct mode — holding position: "
                        f"{'; '.join(ws_check.violations)}")
                self._workspace_clamped = True
            else:
                # Trajectory mode or no safe fallback: E-stop
                violation_str = '; '.join(ws_check.violations)
                self._traj_manager.cancel()
                self.mode = ControlMode.ESTOP
                self._zero_outputs()
                self._fault_state = f'workspace_hard_limit: {violation_str}'
                logger.error(f"WORKSPACE HARD LIMIT -- E-STOP: {violation_str}")
        elif ws_check.status == WorkspaceStatus.SOFT_LIMIT:
            logger.debug(
                f"Workspace soft limit: speed_scale={ws_check.speed_scale:.2f}, "
                f"{'; '.join(ws_check.violations)}")
            # Soft limit is informational for direct-target mode.
            # The hard-limit hold-pose and slew limiter already provide
            # safety.  Scaling smoother limits here causes the platform to
            # crawl near workspace boundaries, degrading responsiveness.
            if pose_6dof is not None:
                self._last_safe_pose_6dof = pose_6dof.copy()
            self._workspace_clamped = False
        else:
            # OK — update last safe pose
            if pose_6dof is not None:
                self._last_safe_pose_6dof = pose_6dof.copy()
            self._workspace_clamped = False

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

    def _apply_smoother_limits(self, source: str) -> None:
        """Set stream smoother velocity/acceleration limits based on control source."""
        if source == 'SPACEMOUSE':
            vel = hw.SPACEMOUSE_SMOOTHER_VEL_LIMIT_RPS
            accel = hw.SPACEMOUSE_SMOOTHER_ACCEL_LIMIT_RPS2
        elif source == 'GUI':
            vel = hw.GUI_SMOOTHER_VEL_LIMIT_RPS
            accel = hw.GUI_SMOOTHER_ACCEL_LIMIT_RPS2
        else:
            vel = hw.ODRIVE_TRAP_VEL_LIMIT_RPS
            accel = hw.ODRIVE_TRAP_ACC_LIMIT_RPS2
        self._base_smoother_vel = vel
        self._base_smoother_accel = accel
        self._smoother.set_limits(vel, accel)
        logger.info(f"Smoother limits: vel={vel} rps, accel={accel} rps²")

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
        self._smoother.reset(home)
        self._last_safe_pose_6dof = home.copy()
        self._workspace_clamped = False
        self._target_pos = home[:3].copy()
        self._target_rot = rot
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
            workspace_clamped=self._workspace_clamped,
            tracking_error_mm=(self._tracking_error_mm.tolist()
                               if self._has_motor_fb else None),
            fault_state=self._fault_state,
            motor_pos=(self._motor_fb_pos_rev.tolist()
                       if self._has_motor_fb else None),
            motor_vel=(self._motor_fb_vel_rps.tolist()
                       if self._has_motor_fb else None),
            motor_cur=(self._motor_fb_cur_A.tolist()
                       if self._has_motor_fb else None),
            slew_limited=self._slew_limited,
        )
        self.ipc.send_telemetry(msg)

    def _publish_fault_telemetry(self, dt_actual: float) -> None:
        """Send reduced telemetry during ESTOP so the bridge can see fault state.

        Motor command fields (leg_pos, leg_vel, cmd_torques) are set to None
        so the bridge's existing ``positions is not None`` gate suppresses
        any motor command publishing.  This prevents the bridge from ever
        forwarding zero-position commands during internally-triggered faults
        (e.g. workspace hard limit, motor overspeed) where the bridge hasn't
        yet received an ERROR mode from the orchestrator.
        """
        msg = {
            'type': 'telemetry',
            'leg_pos': None,
            'leg_vel': None,
            'cmd_torques': None,
            'dt': dt_actual,
            'fault_state': self._fault_state,
            'workspace_status': self._workspace_status.value,
            'slew_limited': False,
        }
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
    # Set multiprocessing start method before any Process is created.
    # 'forkserver' is safer than 'fork' (avoids copying parent thread
    # state) and avoids issues with CUDA or C libraries in the future.
    # Falls back to 'spawn' on Windows (dev machine).
    import multiprocessing
    import platform
    try:
        if platform.system() == 'Windows':
            multiprocessing.set_start_method('spawn')
        else:
            multiprocessing.set_start_method('forkserver')
    except RuntimeError:
        pass  # already set (e.g. in tests)

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
