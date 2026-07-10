"""500 Hz interpolator + safety monitor between MPC and motor hardware.

Receives pre-computed motor commands from the MPC (via HardwarePlant) at
40 Hz, cubically interpolates to 500 Hz for smooth PASSTHROUGH-mode
operation, validates against motor feedback, and forwards approved
commands to the ROS2 bridge via ZeroMQ.

Does NOT compute IK, dynamics, or trajectories -- the MPC handles all
motion planning.

Safety checks (every cycle):
  - Motor feedback staleness  (suppress commands if feedback stale)
  - MPC command staleness     (E-stop if no command within timeout)
  - Motor overspeed           (E-stop if any motor > limit)
  - Max deviation             (E-stop if commanded pos too far from actual)
  - Workspace limits          (E-stop on hard limit, log on soft)
  - IPC heartbeat             (E-stop if bridge silent > timeout)
  - NaN/Inf rejection         (reject any non-finite command or feedback)

Run:  python -m jugglebot.motion.motor_guard [--rate HZ]
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass, field, replace as _dc_replace

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion.conversions import (
    leg_velocities_to_motor_velocities,
    revs_to_extensions_mm,
)
from jugglebot.motion.friction_ff_params import (
    FrictionFFParams,
    load_params as _load_friction_ff_params,
)
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ipc import (
    TOPIC_MODE,
    TOPIC_MPC_CMD,
    TOPIC_MOTOR_FB,
    MotorGuardIPC,
)
from jugglebot.motion.workspace import (
    WorkspaceLimits,
    WorkspaceStatus,
    check_workspace_limits,
    compute_condition_number,
)
from jugglebot.motion.ik_solver import rotvec_to_rot_matrix

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Default loop rate (Hz).  500 Hz is required for smooth cubic
# interpolation of 40 Hz MPC commands.
DEFAULT_RATE_HZ = 500

# Maximum time without any IPC message before triggering E-stop.
IPC_HEARTBEAT_TIMEOUT_S = 0.5

# Motor overspeed threshold -- E-stop if any motor exceeds this (rev/s).
# 10% margin above hardware limit for measurement noise.
MAX_MOTOR_VEL_RPS = hw.ODRIVE_TRAP_VEL_LIMIT_RPS * 1.1

# Motor feedback staleness timeout (seconds).
# 150 ms gives ~50% headroom over the 100 Hz feedback rate (10 ms period).
MOTOR_FB_STALENESS_S = 0.15

# MPC command staleness timeout (seconds).
# MPC runs at 40 Hz (25 ms period); 250 ms = 10x the expected period.
MPC_CMD_STALENESS_S = 0.25

# Max deviation threshold (rev).  E-stop if commanded position diverges
# from actual motor position by more than this.
# At ~71.5 mm/rev, 0.5 rev ~ 36 mm -- generous but catches runaway.
MAX_DEVIATION_REV = 0.5

# Maximum command lead relative to actual encoder position (rev).
# Caps how far ahead the interpolated command can run before the motors
# catch up.  Set to half the CAN step-limit (0.3 rev) so that commands
# reaching the CAN node are always accepted.  At ~70.5 mm/rev this is
# ~10.6 mm -- well above the largest per-step increment at max catch
# speed (700 mm/s → 0.020 rev/step at 500 Hz).
MAX_LEAD_REV = 0.15

# Maximum extrapolation time before velocity decay begins (seconds).
# 2x MPC period (40 Hz = 25 ms) covers normal timing jitter.
MAX_EXTRAP_DT_S = 0.05

# Duration over which extrapolation velocity decays linearly to zero
# after MAX_EXTRAP_DT_S is exceeded (seconds).  Provides smooth
# deceleration rather than an abrupt position freeze.
EXTRAP_DECAY_DT_S = 0.06

# Exponential moving average factor for jerk estimation.
# 0.3 = responsive but filters solver non-determinism noise.
JERK_EMA_ALPHA = 0.3

# Worst-value label per recurring-warning kind, for the aggregated summary
# (see MotorGuard._warn_agg / _flush_warning_summary). Module constant so the
# summary path allocates no dict; the hot loop never touches it.
_WARN_WORST_LABEL = {
    'motor_fb_stale': 'age_s',      # worst = oldest feedback age observed (s)
    'stroke_clamp': 'excursion_rev',  # worst = largest clamp overshoot (rev)
}


# ---------------------------------------------------------------------------
# Timing statistics (ported from control_loop.py)
# ---------------------------------------------------------------------------

@dataclass
class LoopStats:
    """Accumulates per-cycle timing data for jitter analysis."""
    target_dt_s: float
    cycle_times: deque = field(default=None)
    _window_size: int = 10000

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
# Guard modes
# ---------------------------------------------------------------------------

class GuardMode:
    DISABLED = 'disabled'
    ENABLED = 'enabled'
    ESTOP = 'estop'


# ---------------------------------------------------------------------------
# Motor Guard
# ---------------------------------------------------------------------------

class MotorGuard:
    """Interpolator + safety monitor between MPC and motor hardware.

    Receives pre-computed motor commands from the MPC (via HardwarePlant),
    cubically interpolates between 40 Hz MPC updates at 500 Hz, validates
    against motor feedback, and forwards approved commands to the bridge.

    Does NOT compute IK, dynamics, or trajectories -- the MPC handles all
    motion planning.

    Parameters
    ----------
    rate_hz : float
        Target loop rate.  500 Hz for smooth interpolation of 40 Hz MPC.
    geom : StewartGeometry
        Platform geometry (for unit conversions and workspace checks).
    ipc : MotorGuardIPC
        IPC transport (injected for testability).
    """

    def __init__(self,
                 rate_hz: float = DEFAULT_RATE_HZ,
                 geom: StewartGeometry | None = None,
                 ipc: MotorGuardIPC | None = None,
                 friction_ff_enable_override: bool | None = None):
        self.rate_hz = rate_hz
        self.dt_target = 1.0 / rate_hz
        self.geom = geom or StewartGeometry()
        self.ipc = ipc or MotorGuardIPC()

        self.mode = GuardMode.DISABLED
        self.stats = LoopStats(target_dt_s=self.dt_target)
        self._running = False

        # --- MPC command state ---
        # Latched on each new MPC command for interpolation
        self._mpc_base_pos_rev = np.zeros(6)     # base position for interpolation
        self._mpc_base_vel_rps = np.zeros(6)     # velocity for interpolation slope
        self._mpc_base_accel_rps2 = np.zeros(6)  # acceleration for quadratic curvature
        self._mpc_base_torque_Nm = np.zeros(6)   # torque_ff (static, not interpolated)
        self._mpc_base_timestamp = 0.0            # time of last MPC command
        self._mpc_prev_accel_rps2: np.ndarray | None = None  # previous accel for jerk estimation
        self._mpc_base_jerk_rps3 = np.zeros(6)               # estimated jerk (EMA-filtered)

        # Forward-looking waypoint from MPC u[1] — enables Hermite
        # interpolation between u[0] and u[1] instead of Taylor extrapolation.
        # None when absent (solver failure, old MPC) → fallback to extrapolation.
        self._mpc_next_pos_rev: np.ndarray | None = None
        # MPC u[2] — used to compute endpoint velocity at u[1] as
        # (u[2] - u[1]) / T, matching the next segment's v0 so Hermite
        # interpolation is C1-continuous across segment boundaries (no
        # velocity step at 40 Hz tick transitions).  When absent, falls
        # back to the chord velocity v1 = (u[1] - u[0]) / T.
        self._mpc_next2_pos_rev: np.ndarray | None = None
        self._mpc_segment_T: float = 0.025  # nominal MPC fine step (s)

        # Raw MPC command fields (for workspace/condition checks)
        self._mpc_cmd_ext_mm = np.zeros(6)
        self._mpc_cmd_pose_6dof = np.zeros(6)
        self._mpc_cmd_vel_mm_s = np.zeros(6)
        self._mpc_cmd_seq = -1
        self._has_mpc_cmd = False
        self._mpc_cmd_timestamp = 0.0

        # Finite-difference state for vel_ff computation
        self._mpc_prev_ext_mm: np.ndarray | None = None
        self._mpc_prev_timestamp = 0.0

        # --- Motor feedback state ---
        self._motor_fb_pos_rev = np.zeros(6)
        self._motor_fb_vel_rps = np.zeros(6)
        self._motor_fb_cur_A = np.zeros(6)
        self._has_motor_fb = False
        self._motor_fb_timestamp = 0.0

        # --- Control outputs ---
        self._commanded_pos_rev = np.zeros(6)
        self._commanded_vel_ff_rps = np.zeros(6)
        self._commanded_torque_ff_Nm = np.zeros(6)

        # --- Friction feedforward ---
        # Loaded lazily here (not at module import) so the test environment
        # can construct a guard with a custom params instance via
        # ``self._friction_ff_params = ...`` before any compute.
        self._friction_ff_params: FrictionFFParams = _load_friction_ff_params()
        # Per-launch enable override — wired to the ``--friction-ff`` CLI
        # flag and the ``friction_ff_enable`` ROS2 launch argument.  When
        # set, supersedes the YAML default for this session only (no
        # YAML edit / no rebuild).  Used for on-platform A/B comparison
        # of baseline-off vs friction-on (see PR 3a in
        # plans/archived/2026-05-08 friction-ff-motor-guard-integration.md §9).
        # ``FrictionFFParams`` is a frozen dataclass, so we ``replace``
        # to produce a new instance — this also correctly invalidates
        # the ``_compute_friction_ff_Nm`` derived-params cache on the
        # next call (cache uses ``is`` identity).
        if (friction_ff_enable_override is not None
                and friction_ff_enable_override
                != self._friction_ff_params.enabled):
            self._friction_ff_params = _dc_replace(
                self._friction_ff_params,
                enabled=bool(friction_ff_enable_override))
            logger.warning(
                "friction_ff.enabled OVERRIDDEN via CLI to %s "
                "(YAML default would have been %s).  This applies for "
                "this launch session only.",
                self._friction_ff_params.enabled,
                not self._friction_ff_params.enabled)
        # Pre-allocated friction-FF buffers.  All length-6, all reused
        # across calls — ``_compute_friction_ff_Nm`` makes zero per-call
        # ndarray allocations beyond these buffers (zero-allocation hot
        # path at 500 Hz; verified by
        # ``test_friction_ff_scratch_buffer_identity`` and
        # ``test_friction_ff_no_steady_state_alloc``).  Buffer
        # identities are stable across enable/disable/estop cycles.
        #
        # PR 2.1 (2026-05-08): the boost-band buffers from PR 2 are gone.
        # The boost band's hard 0 → τ_s step at v=±1e-4 bootstrapped a
        # platform 5 Hz limit cycle; the smooth gate runs entirely
        # through ``_friction_stribeck`` × ``_friction_gate``.  See
        # logbook/2026-05-08-friction-ff-platform-limit-cycle.md.
        self._friction_ff_buf = np.zeros(6)            # result (returned)
        self._friction_av = np.zeros(6)                # |v|
        self._friction_sgn = np.zeros(6)               # sign(v)
        self._friction_tmp = np.zeros(6)               # general scratch (Stribeck exp arg, gate exp arg)
        self._friction_stribeck = np.zeros(6)          # Stribeck taper magnitude
        self._friction_gate = np.zeros(6)              # smooth low-v gate ∈ [0, 1)
        self._friction_iq = np.zeros(6)                # gated friction magnitude (Stribeck × gate)
        self._friction_iq_total = np.zeros(6)          # iq_total = -sgn·iq + load_offset
        # Cached derived params, recomputed when the params instance
        # changes (FrictionFFParams is frozen, so ``is`` identity tracks
        # content).  First compute call after __init__ or after a test
        # swap performs the cache fill in-place — still zero-alloc.
        self._friction_params_cached: FrictionFFParams | None = None
        self._friction_tau_diff = np.zeros(6)          # tau_s − tau_c (Stribeck taper amplitude)
        self._friction_kt_signed = np.zeros(6)         # ff_sign × motor_kt_nm_per_a (final scale)

        # --- Workspace monitoring ---
        self._workspace_limits = WorkspaceLimits.from_geometry(self.geom)
        self._workspace_status = WorkspaceStatus.OK
        self._workspace_speed_scale = 1.0
        self._cond_number = self._workspace_limits.cond_reference

        # --- Per-cycle stroke clamp (rev) ---
        self._stroke_min_rev = (
            self._workspace_limits.leg_hard_min_mm * self.geom.mm_to_rev)
        self._stroke_max_rev = (
            self._workspace_limits.leg_hard_max_mm * self.geom.mm_to_rev)

        # --- Tracking error (informational) ---
        self._tracking_error_mm = np.zeros(6)

        # --- Fault state ---
        self._fault_state: str | None = None

        # --- Rate-limited warning timestamps ---
        self._last_fb_nan_warn_t = 0.0

        # --- Pre-allocated telemetry dict (updated in-place each cycle) ---
        self._telem_msg: dict = {
            'type': 'telemetry',
            'leg_pos': None,
            'leg_vel': None,
            'leg_torques': None,
            'dt': 0.0,
            'cond_number': None,
            'workspace_status': None,
            'workspace_speed_scale': None,
            'tracking_error_mm': None,
            'fault_state': None,
            'motor_pos': None,
            'motor_vel': None,
            'motor_cur': None,
            'timestamp': None,
        }

        # --- Logging ---
        self._last_log_time = 0.0
        self._log_interval_s = 5.0

        # --- Recurring-warning aggregation (logging-only) ---
        # The 500 Hz interpolate path can re-hit the same condition (stale
        # feedback, stroke clamp) every cycle. Logging each occurrence spammed
        # the console AND built a fresh f-string per cycle in the hot loop.
        # Instead: log the FIRST occurrence of each new episode immediately
        # (full detail), count the rest, and flush ONE aggregated summary
        # (count + worst value per type) every _warn_summary_interval_s. This
        # is strictly logging behaviour — the control path (suppress on stale,
        # clamp + zero-ff on stroke limit) is byte-identical. Per-cycle updates
        # touch only these pre-existing scalars/records (int increment + float
        # compare), so the change REMOVES per-cycle allocation rather than
        # adding it.
        self._warn_summary_interval_s = 10.0
        self._last_warn_summary_t = 0.0
        # kind -> [count_since_flush, worst_value, episode_active]. Fixed keys,
        # allocated once here; the hot loop never inserts a key (no dict resize).
        self._warn_agg = {
            'motor_fb_stale': [0, 0.0, False],
            'stroke_clamp': [0, 0.0, False],
        }

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Main loop.  Blocks until stop() is called or signal received."""
        self._running = True
        logger.info(f"Motor guard starting at {self.rate_hz} Hz "
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

            # 2. Safety checks (heartbeat, staleness)
            self._check_safety()

            # 3. Interpolate and send if enabled
            if self.mode == GuardMode.ENABLED and self._has_mpc_cmd:
                self._interpolate_and_send(dt_actual)
            elif self.mode == GuardMode.ENABLED and not self._has_mpc_cmd:
                # Publish feedback-only telemetry so the MPC plant can
                # read the actual motor state before sending its first
                # command.  Without this, get_state() returns zeros and
                # the MPC's first command triggers MAX_DEVIATION.
                self._publish_feedback_telemetry(dt_actual)
            elif self.mode == GuardMode.ESTOP:
                self._publish_fault_telemetry(dt_actual)

            # 4. Periodic logging
            self._periodic_log(t_start)

            # 5. Sleep for remainder of cycle
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
            if topic == TOPIC_MPC_CMD:
                self._on_mpc_command(msg)
            elif topic == TOPIC_MODE:
                self._on_mode_command(msg)
            elif topic == TOPIC_MOTOR_FB:
                self._on_motor_feedback(msg)

    def _on_mpc_command(self, msg: dict) -> None:
        """Handle pre-computed motor commands from the MPC (via HardwarePlant).

        Validates the command, determines vel_ff (preferring sender-provided
        velocity over wall-clock finite-difference), and latches as the new
        interpolation base.
        """
        if self.mode != GuardMode.ENABLED:
            return

        # --- Parse and validate ---
        try:
            ext = np.array(msg['ext_mm'], dtype=float)
            pose = np.array(msg['pose_6dof'], dtype=float)
            if ext.shape != (6,) or pose.shape != (6,):
                logger.warning(f"Malformed mpc_cmd: ext shape {ext.shape}, "
                               f"pose shape {pose.shape}")
                return
        except (KeyError, TypeError, ValueError) as e:
            logger.warning(f"Malformed mpc_cmd message, ignoring: {e}")
            return

        # --- NaN/Inf check ---
        if not np.all(np.isfinite(ext)) or not np.all(np.isfinite(pose)):
            logger.warning("MPC command contains NaN/Inf -- rejected")
            return

        motor_rev_raw = msg.get('motor_rev')
        torque_raw = msg.get('torque_Nm')
        vel_raw = msg.get('vel_mm_s')
        acc_raw = msg.get('acc_mm_s2')

        if motor_rev_raw is not None:
            motor_rev = np.array(motor_rev_raw, dtype=float)
            if not np.all(np.isfinite(motor_rev)):
                logger.warning("MPC command motor_rev contains NaN/Inf -- rejected")
                return
        else:
            motor_rev = None

        if torque_raw is not None:
            torque_Nm = np.array(torque_raw, dtype=float)
            if not np.all(np.isfinite(torque_Nm)):
                logger.warning("MPC command torque_Nm contains NaN/Inf -- rejected")
                return
        else:
            torque_Nm = np.zeros(6)

        if acc_raw is not None:
            acc_mm_s2 = np.array(acc_raw, dtype=float)
            if not np.all(np.isfinite(acc_mm_s2)):
                logger.warning("MPC command acc_mm_s2 contains NaN/Inf -- rejected")
                return
        else:
            acc_mm_s2 = np.zeros(6)

        # --- Store raw command state ---
        self._mpc_cmd_ext_mm = ext
        self._mpc_cmd_pose_6dof = pose
        self._mpc_cmd_seq = msg.get('seq', -1)
        t_now = time.perf_counter()
        self._mpc_cmd_timestamp = t_now
        self._has_mpc_cmd = True

        # --- Compute vel_ff ---
        # Prefer sender-provided velocity (computed with deterministic MPC dt,
        # immune to ZMQ delivery jitter).  Fall back to wall-clock finite-
        # difference only when the sender does not provide velocity.
        if vel_raw is not None:
            vel_arr = np.array(vel_raw, dtype=float)
            if np.all(np.isfinite(vel_arr)):
                self._mpc_cmd_vel_mm_s = vel_arr
            else:
                self._mpc_cmd_vel_mm_s = self._finite_diff_velocity(
                    ext, t_now)
        else:
            self._mpc_cmd_vel_mm_s = self._finite_diff_velocity(ext, t_now)

        prev_ts = self._mpc_prev_timestamp  # capture before overwrite (for jerk)
        self._mpc_prev_ext_mm = ext.copy()
        self._mpc_prev_timestamp = t_now

        # --- Workspace check on incoming command ---
        # Runs at MPC rate (40 Hz), not interpolation rate (500 Hz).
        # This is safe because:
        #   - Leg extension hard limits are enforced every cycle via stroke clamp
        #   - Condition number is geometrically smooth (negligible change in 25ms)
        #   - speed_scale is informational only (not consumed by any controller)
        pos_cart = pose[:3]
        rot_mat = rotvec_to_rot_matrix(pose[3:6])
        cond = compute_condition_number(pos_cart, rot_mat, self.geom)
        self._cond_number = cond

        ws_check = check_workspace_limits(
            ext, cond, self._workspace_limits)
        self._workspace_status = ws_check.status
        self._workspace_speed_scale = ws_check.speed_scale

        if ws_check.status == WorkspaceStatus.HARD_LIMIT:
            violation_str = '; '.join(ws_check.violations)
            self._trigger_estop(f'workspace_hard_limit: {violation_str}')
            logger.error(f"WORKSPACE HARD LIMIT -- E-STOP: {violation_str}")
            return
        elif ws_check.status == WorkspaceStatus.SOFT_LIMIT:
            logger.debug(
                f"Workspace soft limit: speed_scale={ws_check.speed_scale:.2f}, "
                f"{'; '.join(ws_check.violations)}")

        # --- Max deviation check ---
        if self._has_motor_fb:
            pos_rev = motor_rev if motor_rev is not None else ext * self.geom.mm_to_rev
            deviation = np.abs(pos_rev - self._motor_fb_pos_rev)
            if np.any(deviation > MAX_DEVIATION_REV):
                worst = float(np.max(deviation))
                worst_leg = int(np.argmax(deviation))
                self._trigger_estop(
                    f'max_deviation: leg {worst_leg} = {worst:.4f} rev '
                    f'(limit {MAX_DEVIATION_REV})')
                logger.error(
                    f"MAX DEVIATION -- E-STOP: leg {worst_leg} = {worst:.4f} rev "
                    f"(limit {MAX_DEVIATION_REV})")
                return

        # --- Latch interpolation base ---
        if motor_rev is not None:
            self._mpc_base_pos_rev = motor_rev.copy()
        else:
            self._mpc_base_pos_rev = ext * self.geom.mm_to_rev

        self._mpc_base_vel_rps = leg_velocities_to_motor_velocities(
            self._mpc_cmd_vel_mm_s, self.geom)
        # Acceleration: same linear conversion as velocity (mm_to_rev is constant)
        self._mpc_base_accel_rps2 = acc_mm_s2 * self.geom.mm_to_rev
        self._mpc_base_torque_Nm = torque_Nm.copy()
        self._mpc_base_timestamp = t_now

        # --- Jerk estimation (for cubic extrapolation fallback) ---
        new_accel = self._mpc_base_accel_rps2
        if self._mpc_prev_accel_rps2 is not None:
            dt_mpc = t_now - prev_ts
            if dt_mpc > 1e-6:
                raw_jerk = (new_accel - self._mpc_prev_accel_rps2) / dt_mpc
                self._mpc_base_jerk_rps3 = (
                    JERK_EMA_ALPHA * raw_jerk
                    + (1.0 - JERK_EMA_ALPHA) * self._mpc_base_jerk_rps3)
        else:
            self._mpc_base_jerk_rps3 = np.zeros(6)
        self._mpc_prev_accel_rps2 = new_accel.copy()

        # --- Forward-looking waypoints (u[1], u[2]) for Hermite interpolation ---
        cmd_next_raw = msg.get('cmd_next_mm')
        if cmd_next_raw is not None:
            cmd_next_arr = np.array(cmd_next_raw, dtype=float)
            if (np.all(np.isfinite(cmd_next_arr))
                    and cmd_next_arr.shape == (6,)):
                self._mpc_next_pos_rev = cmd_next_arr * self.geom.mm_to_rev
            else:
                self._mpc_next_pos_rev = None
        else:
            self._mpc_next_pos_rev = None

        # u[2]: used to compute C1-continuous v1 = (u[2] - u[1]) / T.
        # Only meaningful when u[1] is also present.
        cmd_next2_raw = msg.get('cmd_next2_mm')
        if cmd_next2_raw is not None and self._mpc_next_pos_rev is not None:
            cmd_next2_arr = np.array(cmd_next2_raw, dtype=float)
            if (np.all(np.isfinite(cmd_next2_arr))
                    and cmd_next2_arr.shape == (6,)):
                self._mpc_next2_pos_rev = cmd_next2_arr * self.geom.mm_to_rev
            else:
                self._mpc_next2_pos_rev = None
        else:
            self._mpc_next2_pos_rev = None

    def _on_mode_command(self, msg: dict) -> None:
        """Handle mode commands (enable, disable, estop, fault)."""
        try:
            cmd = msg['cmd']
        except (KeyError, TypeError) as e:
            logger.warning(f"Malformed mode command, ignoring: {e}")
            return

        params = msg.get('params', {})

        if cmd == 'enable':
            source = params.get('source', '')
            if self.mode == GuardMode.DISABLED:
                self.mode = GuardMode.ENABLED
                self._fault_state = None
                self._has_mpc_cmd = False
                self._mpc_prev_ext_mm = None
                self._mpc_cmd_timestamp = time.perf_counter()
                logger.info(f"Motor guard ENABLED (source={source})")
            elif self.mode == GuardMode.ENABLED:
                # Already enabled — no-op.  The bridge always sends
                # disable+enable on mode transitions, which clears MPC state
                # via _reset_mpc_state().  A second enable (from HardwarePlant
                # or a repeated bridge message) must NOT reset state, or it
                # creates a race where vel_ff is lost.
                logger.debug("Motor guard already ENABLED, ignoring enable "
                             f"(source={source})")

        elif cmd == 'disable':
            self.mode = GuardMode.DISABLED
            self._zero_outputs()
            self._reset_mpc_state()
            self._fault_state = None
            logger.info("Motor guard DISABLED")

        elif cmd == 'estop':
            if self.mode != GuardMode.ESTOP:
                reason = params.get('reason', 'external')
                self._trigger_estop(reason)
                logger.warning(f"E-STOP triggered via command: {reason}")

        elif cmd == 'fault':
            fault_desc = params.get('description', 'unknown fault')
            self._trigger_estop(fault_desc)
            logger.error(f"ODrive FAULT received: {fault_desc} -- E-STOP")

        else:
            logger.warning(f"Unknown mode command: {cmd}")

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
        if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(vel))
                and np.all(np.isfinite(cur))):
            now = time.perf_counter()
            if now - self._last_fb_nan_warn_t >= 2.0:
                logger.warning("Motor feedback contains NaN/Inf -- rejected")
                self._last_fb_nan_warn_t = now
            return
        self._motor_fb_pos_rev = pos
        self._motor_fb_vel_rps = vel
        self._motor_fb_cur_A = cur
        self._has_motor_fb = True
        self._motor_fb_timestamp = time.perf_counter()

    # ------------------------------------------------------------------
    # Friction feedforward
    # ------------------------------------------------------------------

    def _compute_friction_ff_Nm(self) -> np.ndarray:
        """Per-leg Stribeck-with-smooth-gate friction torque feedforward, in Nm.

        Returns a (6,) array — the additive friction-FF contribution to
        ``_commanded_torque_ff_Nm``.  Returns zeros when the global flag
        ``friction_ff.enabled`` is False, so the disabled path costs one
        branch + a memset.

        Magnitude (mirrors bench fit):
            stribeck(v) = τ_c + (τ_s − τ_c)·exp(−(|v|/ω_s)²) + b·|v|
        Smooth low-v gate (replaces bench-era boost band):
            gate(v) = 1 − exp(−(|v|/v_gate)²)
        Output:
            iq_friction(v) = stribeck(v) · gate(v)
            torque_ff = (−sign(v) · iq_friction(v) + load_offset) · ff_sign · Kt

        The smooth gate is the load-bearing change from PR 2 → PR 2.1:
        the bench's hard "stiction-boost band" applied full τ_s for any
        |v| in [1e-4, 0.20) and zero below.  That 0 → τ_s step at
        v=±1e-4 produced a 0 → 0.122 Nm jump in torque_ff within a
        single 500 Hz tick.  On platform — where MPC's hold-phase
        commanded vel naturally crosses ±1e-4 (~5 % of hold time per
        the BASELINE rosbag) — that step kicks the leg into a self-
        sustaining 5 Hz limit cycle with ~1.5 mm peak-to-peak hold
        oscillation.  The smooth gate scales the FF magnitude
        continuously with |v|: hold-phase commanded-vel corrections
        produce <1 mNm of torque (well within the position loop's
        absorption capacity), while motion-onset ramps engage near-
        full FF within ~v=0.10 rev/s.

        The bench couldn't see the failure because its Python-generated
        trapezoid trajectories produced ``vel_ff = 0.0`` exactly during
        hold — never crossing the dead-zone edge.  See
        logbook/2026-05-08-friction-ff-platform-limit-cycle.md for the
        full analysis (rosbag-validated mechanism + sizing rationale
        for v_gate).

        At |v| ≪ v_gate (hold-phase noise): gate ≈ (|v|/v_gate)², so
        torque scales as |v|², heavily damped — no kick, no chatter.
        At |v| = v_gate: gate = 1 − e⁻¹ ≈ 0.63, torque ≈ 63 % of full.
        At |v| = 2·v_gate: gate ≈ 0.98, torque ≈ full.  Effective
        engagement velocity is ~2·v_gate.

        Sign convention (verified against can_node 2026-05-08):
            ``can_node._send_position_target`` negates ``torque_ff``
            for leg axes via ``_leg_sign(axis_id, torque_ff)``
            (can_node.py:713).  The bench bypassed can_node entirely
            and validated ``ff_sign = -1`` against direct ODrive sends.
            On platform, the chain is motor_guard's negation × can_node's
            negation × ODrive's wiring inversion — two negations vs the
            bench's one, so platform ``ff_sign`` must be flipped to +1.
            YAML default reflects this.

        The function is the single canonical enforcement point of the
        friction-FF contract — no other code path writes friction-derived
        torque to ``_commanded_torque_ff_Nm`` (see §2 of
        plans/archived/2026-05-08 friction-ff-motor-guard-integration.md).

        Zero-allocation contract: this function makes no per-call
        ndarray allocations.  All temporaries are pre-allocated in
        ``__init__`` (``_friction_av`` etc.) and reused across calls
        via numpy ``out=`` ufunc kwargs.  Verified by
        ``test_friction_ff_scratch_buffer_identity`` and
        ``test_friction_ff_no_steady_state_alloc``.

        Vectorised port of ``friction_ff_nm`` in
        tests/hardware/friction_ff_demo.py:167-205, with the boost band
        replaced by the smooth gate as documented above.
        """
        p = self._friction_ff_params
        buf = self._friction_ff_buf
        if not p.enabled:
            buf.fill(0.0)
            return buf

        # Refresh derived-params cache when the params instance changes.
        # FrictionFFParams is frozen, so ``is`` identity tracks content;
        # tests that swap ``self._friction_ff_params`` get a new instance
        # and trigger an in-place recompute on the next call.
        if self._friction_params_cached is not p:
            np.subtract(p.tau_s_A, p.tau_c_A, out=self._friction_tau_diff)
            np.multiply(p.ff_sign, p.motor_kt_nm_per_a,
                        out=self._friction_kt_signed)
            self._friction_params_cached = p

        v = self._commanded_vel_ff_rps
        av = self._friction_av
        sgn = self._friction_sgn
        tmp = self._friction_tmp
        stribeck = self._friction_stribeck
        gate = self._friction_gate
        iq = self._friction_iq
        iq_total = self._friction_iq_total

        np.abs(v, out=av)
        np.sign(v, out=sgn)

        # Stribeck taper:
        #   stribeck = τ_c + (τ_s − τ_c) · exp(−(av/ω_s)²) + b·av
        # Build via in-place ops on tmp + stribeck.
        np.divide(av, p.omega_s_rps, out=tmp)            # tmp = av/ω_s
        np.square(tmp, out=tmp)                          # tmp = (av/ω_s)²
        np.negative(tmp, out=tmp)                        # tmp = −(av/ω_s)²
        np.exp(tmp, out=tmp)                             # tmp = exp(...)
        np.multiply(self._friction_tau_diff, tmp, out=stribeck)  # (τ_s−τ_c)·exp
        np.add(stribeck, p.tau_c_A, out=stribeck)        # + τ_c
        np.multiply(p.b_A_per_rps, av, out=tmp)          # tmp = b·av
        np.add(stribeck, tmp, out=stribeck)              # + b·av  → full Stribeck taper

        # Smooth low-v gate:
        #   gate = 1 − exp(−(av/v_gate)²)
        # Same Gaussian form as the Stribeck τ_s−τ_c term (which decays
        # FROM 1 to 0 as v rises) but flipped (rises FROM 0 to 1).  This
        # parallel is intentional — both shapes are tuned to the leg's
        # natural transition between hold-phase noise (av ≈ 0) and real
        # commanded motion (av ≫ v_gate).
        np.divide(av, p.v_gate_rps, out=tmp)             # tmp = av/v_gate
        np.square(tmp, out=tmp)                          # tmp = (av/v_gate)²
        np.negative(tmp, out=tmp)                        # tmp = −(av/v_gate)²
        np.exp(tmp, out=gate)                            # gate = exp(...)  ∈ (0, 1]
        np.subtract(1.0, gate, out=gate)                 # gate = 1 − exp  ∈ [0, 1)

        # iq_friction = stribeck · gate.  At |v| ≪ v_gate this kills the
        # friction torque cleanly; at |v| ≫ v_gate gate≈1 and we get the
        # full Stribeck taper.  No discontinuity, no boost-band step.
        np.multiply(stribeck, gate, out=iq)

        # iq_total = −sgn·iq + load_offset.  ``np.sign(0)`` is exactly 0
        # in numpy, so the v=0 case naturally yields iq_total = load_offset
        # (only the constant load is applied at hold) without an explicit
        # dead-zone clamp.  This is the structural replacement for PR 2's
        # ``np.where(av < 1e-4, 0, ...)`` — at v=0 the gate is 0, iq is 0,
        # and −sgn·iq is 0 regardless of how sgn is computed.
        np.multiply(sgn, iq, out=iq_total)
        np.negative(iq_total, out=iq_total)
        np.add(iq_total, p.load_offset_A, out=iq_total)

        # buf = iq_total · (ff_sign · Kt)  [pre-cached scaling]
        np.multiply(iq_total, self._friction_kt_signed, out=buf)
        return buf

    # ------------------------------------------------------------------
    # Safety checks
    # ------------------------------------------------------------------

    def _check_safety(self) -> None:
        """Per-cycle safety checks: heartbeat, staleness, overspeed."""
        if self.mode != GuardMode.ENABLED:
            return

        # --- IPC heartbeat ---
        if self.ipc.seconds_since_last_recv > IPC_HEARTBEAT_TIMEOUT_S:
            self._trigger_estop('ipc_heartbeat_lost')
            logger.warning(
                f"IPC heartbeat lost ({self.ipc.seconds_since_last_recv:.1f}s "
                f"since last message) -- E-STOP")
            return

        # --- MPC command staleness ---
        if self._has_mpc_cmd:
            mpc_age = time.perf_counter() - self._mpc_cmd_timestamp
            if mpc_age > MPC_CMD_STALENESS_S:
                self._trigger_estop(
                    f'mpc_cmd_stale: {mpc_age:.3f}s since last command')
                logger.error(
                    f"MPC command stale ({mpc_age:.3f}s > "
                    f"{MPC_CMD_STALENESS_S}s) -- E-STOP")
                return

        # --- Motor feedback staleness ---
        # Don't E-stop; just suppress commands (checked in _interpolate_and_send)

        # --- Motor overspeed ---
        if self._has_motor_fb:
            if np.any(np.abs(self._motor_fb_vel_rps) > MAX_MOTOR_VEL_RPS):
                worst = float(np.max(np.abs(self._motor_fb_vel_rps)))
                self._trigger_estop(f'motor_overspeed: {worst:.2f} rev/s')
                logger.error(
                    f"MOTOR OVERSPEED -- E-STOP: worst={worst:.2f} rev/s "
                    f"(limit {MAX_MOTOR_VEL_RPS:.1f})")

    # ------------------------------------------------------------------
    # Interpolation + output
    # ------------------------------------------------------------------

    def _interpolate_and_send(self, dt_actual: float) -> None:
        """Cubically interpolate from the last MPC command and send to bridge.

        Each 500 Hz cycle extrapolates:
            pos(t) = base_pos + vel·dt + ½·acc·dt² + ⅙·jerk·dt³
            vel_ff(t) = vel + acc·dt + ½·jerk·dt²

        Jerk is estimated via EMA-filtered finite-difference of consecutive
        MPC accelerations.  The cubic term reduces interpolation error from
        O(jerk·dt³) to O(snap·dt⁴), improving tracking at catch/throw speeds.

        torque_ff passes through unchanged (static per MPC step).
        Output is suppressed if motor feedback is stale.
        """
        # Suppress output if no motor feedback or feedback is stale
        if not self._has_motor_fb:
            return

        fb_age = time.perf_counter() - self._motor_fb_timestamp
        if fb_age > MOTOR_FB_STALENESS_S:
            # Logging-only aggregation: log the first occurrence of a stale
            # episode immediately, count the rest (summary every
            # _warn_summary_interval_s). The control effect — suppress output
            # by returning — is unchanged.
            rec = self._warn_agg['motor_fb_stale']
            if not rec[2]:
                rec[2] = True
                logger.warning(
                    f"Motor feedback stale ({fb_age:.3f}s) -- suppressing "
                    f"commands (further occurrences aggregated every "
                    f"{self._warn_summary_interval_s:.0f}s)")
            rec[0] += 1
            if fb_age > rec[1]:
                rec[1] = fb_age
            return
        # Feedback is fresh — close any open stale episode so the next onset
        # logs its first occurrence immediately again.
        self._warn_agg['motor_fb_stale'][2] = False

        # Interpolate from last MPC command.  Three modes:
        #
        # 1. Hermite interpolation (cmd_next present): cubic Hermite between
        #    u[0] and u[1], clamped at s=1 when MPC is late.  Bounded — never
        #    extrapolates past the known next waypoint.
        #
        # 2. Taylor extrapolation (cmd_next absent, dt <= MAX_EXTRAP_DT_S):
        #    fallback cubic polynomial from base position.  Used when the MPC
        #    solver failed or is an older version without cmd_next.
        #
        # 3. Velocity decay (cmd_next absent, dt > MAX_EXTRAP_DT_S):
        #    velocity ramps linearly to zero to prevent unbounded drift.
        t_now = time.perf_counter()
        dt_since_cmd = t_now - self._mpc_base_timestamp

        if self._mpc_next_pos_rev is not None:
            # --- Hermite interpolation between u[0] and u[1] ---
            T = self._mpc_segment_T
            s = min(dt_since_cmd / T, 1.0)  # clamp: hold at u[1] if late
            s2 = s * s
            s3 = s2 * s

            p0 = self._mpc_base_pos_rev
            p1 = self._mpc_next_pos_rev
            v0 = self._mpc_base_vel_rps       # forward-looking from MPC
            # Endpoint velocity at u[1]:
            #   If u[2] is available, use (u[2] - u[1]) / T — this matches
            #   the next segment's v0 = (new_u[1] - new_u[0]) / T when the
            #   next MPC tick arrives (since new_u[0] = old_u[1] and
            #   new_u[1] = old_u[2]).  C1-continuous across boundaries.
            #   Fallback: chord velocity (u[1] - u[0]) / T.
            if self._mpc_next2_pos_rev is not None:
                v1 = (self._mpc_next2_pos_rev - p1) / T
            else:
                v1 = (p1 - p0) / T                # chord velocity at endpoint

            # Cubic Hermite basis functions
            h00 = 2.0 * s3 - 3.0 * s2 + 1.0
            h10 = s3 - 2.0 * s2 + s
            h01 = -2.0 * s3 + 3.0 * s2
            h11 = s3 - s2

            self._commanded_pos_rev = (
                h00 * p0 + h10 * (T * v0) + h01 * p1 + h11 * (T * v1))

            # Velocity: dp/dt = (1/T) · dp/ds
            inv_T = 1.0 / T
            dh00 = (6.0 * s2 - 6.0 * s) * inv_T
            dh10 = 3.0 * s2 - 4.0 * s + 1.0
            dh01 = (-6.0 * s2 + 6.0 * s) * inv_T
            dh11 = 3.0 * s2 - 2.0 * s

            self._commanded_vel_ff_rps = (
                dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1)

        elif dt_since_cmd <= MAX_EXTRAP_DT_S:
            # --- Fallback: cubic Taylor extrapolation (unchanged) ---
            dt2 = dt_since_cmd * dt_since_cmd
            self._commanded_pos_rev = (
                self._mpc_base_pos_rev
                + self._mpc_base_vel_rps * dt_since_cmd
                + 0.5 * self._mpc_base_accel_rps2 * dt2
                + (1.0 / 6.0) * self._mpc_base_jerk_rps3 * (dt2 * dt_since_cmd))
            self._commanded_vel_ff_rps = (
                self._mpc_base_vel_rps
                + self._mpc_base_accel_rps2 * dt_since_cmd
                + 0.5 * self._mpc_base_jerk_rps3 * dt2)
        else:
            # MPC late — ramp velocity to zero over EXTRAP_DECAY_DT_S.
            # Velocity at the boundary of normal interpolation (cubic):
            dt_b2 = MAX_EXTRAP_DT_S * MAX_EXTRAP_DT_S
            vel_at_boundary = (
                self._mpc_base_vel_rps
                + self._mpc_base_accel_rps2 * MAX_EXTRAP_DT_S
                + 0.5 * self._mpc_base_jerk_rps3 * dt_b2)
            # Position at the boundary (cubic up to MAX_EXTRAP_DT_S):
            pos_at_boundary = (
                self._mpc_base_pos_rev
                + self._mpc_base_vel_rps * MAX_EXTRAP_DT_S
                + 0.5 * self._mpc_base_accel_rps2 * dt_b2
                + (1.0 / 6.0) * self._mpc_base_jerk_rps3 * (dt_b2 * MAX_EXTRAP_DT_S))

            dt_over = dt_since_cmd - MAX_EXTRAP_DT_S
            decay_frac = max(0.0, 1.0 - dt_over / EXTRAP_DECAY_DT_S)

            # Coast-down: linearly decay boundary velocity to zero
            if dt_over >= EXTRAP_DECAY_DT_S:
                # Fully decayed — hold at coast-down endpoint
                extra = vel_at_boundary * (EXTRAP_DECAY_DT_S * 0.5)
            else:
                extra = vel_at_boundary * dt_over * (
                    1.0 - dt_over / (2.0 * EXTRAP_DECAY_DT_S))

            self._commanded_pos_rev = pos_at_boundary + extra
            self._commanded_vel_ff_rps = vel_at_boundary * decay_frac

        # torque_ff = MPC-supplied rigid-body dynamics torque (gravity +
        # platform inertia projected through the Jacobian, populated by
        # controller/hardware_plant.py:_populate_ff_torque) + per-leg
        # Stribeck friction FF.  The two stack additively — they model
        # orthogonal physics (bulk rigid-body vs. transmission friction).
        # Single canonical enforcement point per
        # plans/archived/2026-05-08 friction-ff-motor-guard-integration.md §3.3.
        np.add(self._mpc_base_torque_Nm,
               self._compute_friction_ff_Nm(),
               out=self._commanded_torque_ff_Nm)

        # Tracking clamp: never let the commanded position run more than
        # MAX_LEAD_REV ahead of actual encoder position.  This guarantees
        # CAN commands are always within the step-limit (0.3 rev) and
        # prevents velocity accumulation from feedforward errors.
        if self._has_motor_fb:
            pre_clamp_pos = self._commanded_pos_rev.copy()
            deviation = self._commanded_pos_rev - self._motor_fb_pos_rev
            np.clip(deviation, -MAX_LEAD_REV, MAX_LEAD_REV, out=deviation)
            self._commanded_pos_rev = self._motor_fb_pos_rev + deviation
            # Zero vel_ff on legs that were clamped to prevent ODrive
            # from driving past the clamp boundary.
            lead_clamped = pre_clamp_pos != self._commanded_pos_rev
            if lead_clamped.any():
                self._commanded_vel_ff_rps[lead_clamped] = 0.0

        # Clamp to stroke hard limits — backstop against extrapolation
        # overshooting physical boundaries.
        pre_clamp = self._commanded_pos_rev.copy()
        np.clip(self._commanded_pos_rev,
                self._stroke_min_rev, self._stroke_max_rev,
                out=self._commanded_pos_rev)
        clamped_mask = pre_clamp != self._commanded_pos_rev
        if clamped_mask.any():
            # Zero feedforward for clamped legs — position is held at the
            # limit so vel_ff and torque_ff are physically meaningless and
            # would fight the position loop, causing current spikes.
            self._commanded_vel_ff_rps[clamped_mask] = 0.0
            self._commanded_torque_ff_Nm[clamped_mask] = 0.0
            # Promote to workspace hard limit so telemetry/GUI reflect
            # that the platform is being held at a physical boundary.
            self._workspace_status = WorkspaceStatus.HARD_LIMIT
            self._workspace_speed_scale = 0.0
            # Logging-only aggregation: first occurrence of a clamp episode
            # logs immediately (with the per-leg detail — np.where/.round only
            # run here, not every clamped cycle), the rest are counted for the
            # summary. worst = largest clamp overshoot this window.
            rec = self._warn_agg['stroke_clamp']
            excursion = float(np.abs(pre_clamp - self._commanded_pos_rev).max())
            if not rec[2]:
                rec[2] = True
                clamped_legs = np.where(clamped_mask)[0]
                logger.warning(
                    f"Stroke clamp active on legs {clamped_legs.tolist()}: "
                    f"pre-clamp={pre_clamp[clamped_legs].round(4)}, "
                    f"clamped={self._commanded_pos_rev[clamped_legs].round(4)} "
                    f"(further occurrences aggregated every "
                    f"{self._warn_summary_interval_s:.0f}s)")
            rec[0] += 1
            if excursion > rec[1]:
                rec[1] = excursion
        else:
            # No clamp this cycle — close any open clamp episode.
            self._warn_agg['stroke_clamp'][2] = False

        # Compute tracking error (informational)
        commanded_ext_mm = revs_to_extensions_mm(
            self._commanded_pos_rev, self.geom)
        actual_ext_mm = revs_to_extensions_mm(
            self._motor_fb_pos_rev, self.geom)
        self._tracking_error_mm = np.abs(commanded_ext_mm - actual_ext_mm)

        # Publish telemetry with motor commands
        self._publish_telemetry(dt_actual)

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def _publish_telemetry(self, dt_actual: float) -> None:
        """Send telemetry (with motor commands) to the bridge.

        Updates the pre-allocated ``_telem_msg`` dict in-place to avoid
        per-cycle dict construction.  ``.tolist()`` is still used for
        numpy→list conversion (required by msgpack), but the dict itself
        and all its keys are reused across cycles.
        """
        msg = self._telem_msg
        msg['leg_pos'] = self._commanded_pos_rev.tolist()
        msg['leg_vel'] = self._commanded_vel_ff_rps.tolist()
        msg['leg_torques'] = self._commanded_torque_ff_Nm.tolist()
        msg['dt'] = dt_actual
        msg['cond_number'] = self._cond_number
        msg['workspace_status'] = self._workspace_status.value
        msg['workspace_speed_scale'] = self._workspace_speed_scale
        msg['fault_state'] = self._fault_state
        if self._has_motor_fb:
            msg['tracking_error_mm'] = self._tracking_error_mm.tolist()
            msg['motor_pos'] = self._motor_fb_pos_rev.tolist()
            msg['motor_vel'] = self._motor_fb_vel_rps.tolist()
            msg['motor_cur'] = self._motor_fb_cur_A.tolist()
        else:
            msg['tracking_error_mm'] = None
            msg['motor_pos'] = None
            msg['motor_vel'] = None
            msg['motor_cur'] = None
        msg['timestamp'] = time.perf_counter()
        self.ipc.send_telemetry(msg)

    def _publish_fault_telemetry(self, dt_actual: float) -> None:
        """Send reduced telemetry during ESTOP so the bridge can see fault state.

        Motor command fields are set to None so the bridge's existing
        ``positions is not None`` gate suppresses any motor command publishing.
        """
        msg = {
            'type': 'telemetry',
            'leg_pos': None,
            'leg_vel': None,
            'leg_torques': None,
            'dt': dt_actual,
            'fault_state': self._fault_state,
            'workspace_status': self._workspace_status.value,
            'timestamp': time.perf_counter(),
        }
        self.ipc.send_telemetry(msg)

    def _publish_feedback_telemetry(self, dt_actual: float) -> None:
        """Publish motor feedback while waiting for the first MPC command.

        Sends motor_pos/motor_vel/motor_cur so the HardwarePlant can read
        the actual platform state before the MPC computes its first command.
        Command fields (leg_pos, leg_vel, leg_torques) are None so the
        bridge will not send motor commands.
        """
        msg = {
            'type': 'telemetry',
            'leg_pos': None,
            'leg_vel': None,
            'leg_torques': None,
            'dt': dt_actual,
            'fault_state': None,
            'cond_number': self._cond_number,
            'workspace_status': self._workspace_status.value,
            'workspace_speed_scale': self._workspace_speed_scale,
            'tracking_error_mm': None,
            'timestamp': time.perf_counter(),
        }
        if self._has_motor_fb:
            msg['motor_pos'] = self._motor_fb_pos_rev.tolist()
            msg['motor_vel'] = self._motor_fb_vel_rps.tolist()
            msg['motor_cur'] = self._motor_fb_cur_A.tolist()
        else:
            msg['motor_pos'] = None
            msg['motor_vel'] = None
            msg['motor_cur'] = None
        self.ipc.send_telemetry(msg)

    # ------------------------------------------------------------------
    # State management helpers
    # ------------------------------------------------------------------

    def _trigger_estop(self, reason: str) -> None:
        """Transition to E-STOP state."""
        self.mode = GuardMode.ESTOP
        self._zero_outputs()
        self._reset_mpc_state()
        self._fault_state = reason

    def _zero_outputs(self) -> None:
        """Zero all control outputs (safe state).

        In-place fill (not rebind) keeps buffer identity stable across
        estop / disable / re-enable cycles.  The torque buffer in
        particular is the ``out=`` target for ``np.add`` in
        ``_interpolate_and_send`` — preserving its identity matches
        the identity-stability contract held by the sibling
        ``_friction_ff_buf`` and prevents a silent buffer swap on
        E-stop from biting future callers that cache ``id(...)``.
        """
        self._commanded_pos_rev.fill(0.0)
        self._commanded_vel_ff_rps.fill(0.0)
        self._commanded_torque_ff_Nm.fill(0.0)

    def _reset_mpc_state(self) -> None:
        """Reset all MPC pass-through state."""
        self._has_mpc_cmd = False
        self._mpc_prev_ext_mm = None
        self._mpc_prev_accel_rps2 = None
        self._mpc_base_jerk_rps3 = np.zeros(6)
        self._mpc_next_pos_rev = None
        self._mpc_next2_pos_rev = None

    def _finite_diff_velocity(self, ext: np.ndarray, t_now: float
                              ) -> np.ndarray:
        """Compute velocity via finite-difference of consecutive extensions.

        Fallback when the sender does not provide velocity.
        Uses wall-clock timing (subject to IPC jitter).
        """
        if self._mpc_prev_ext_mm is not None:
            dt_mpc = t_now - self._mpc_prev_timestamp
            if dt_mpc > 1e-6:
                return (ext - self._mpc_prev_ext_mm) / dt_mpc
        return np.zeros(6)

    # ------------------------------------------------------------------
    # Periodic logging
    # ------------------------------------------------------------------

    def _flush_warning_summary(self, t_now: float) -> None:
        """Emit one aggregated WARNING per active recurring-warning kind.

        Runs every cycle but does real work only once per
        _warn_summary_interval_s (a float compare gates it — no allocation on
        the common path). For each kind that fired since the last flush, logs a
        single ``<kind>: N occurrences in last 10s (worst <label>=<value>)``
        line and resets its count + worst. The ``episode_active`` flag is left
        untouched: it is cleared only when the underlying condition clears, so a
        fault spanning the flush boundary does not re-log its "first occurrence"
        line.
        """
        if t_now - self._last_warn_summary_t < self._warn_summary_interval_s:
            return
        self._last_warn_summary_t = t_now
        for kind, rec in self._warn_agg.items():
            count, worst = rec[0], rec[1]
            if count > 0:
                logger.warning(
                    "%s: %d occurrences in last %.0fs (worst %s=%.4f)",
                    kind, count, self._warn_summary_interval_s,
                    _WARN_WORST_LABEL[kind], worst)
                rec[0] = 0
                rec[1] = 0.0

    def _periodic_log(self, t_now: float) -> None:
        """Log timing statistics periodically."""
        self._flush_warning_summary(t_now)
        if t_now - self._last_log_time >= self._log_interval_s:
            self._last_log_time = t_now
            if self.stats.count > 0:
                logger.info(f"Loop timing: {self.stats.summary()}")
                if self.mode == GuardMode.ENABLED:
                    logger.info(
                        f"  cond#={self._cond_number:.1f}  "
                        f"ws={self._workspace_status.value}  "
                        f"ws_scale={self._workspace_speed_scale:.2f}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Jugglebot motor guard process (MPC interpolator + safety)")
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE_HZ,
                        help=f"Loop rate in Hz (default: {DEFAULT_RATE_HZ})")
    parser.add_argument('--log-level', default='INFO',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'])
    parser.add_argument('--friction-ff', default='yaml',
                        choices=['yaml', 'true', 'false'],
                        help="Override hardware_config.yaml friction_ff.enabled "
                             "for this launch only (no YAML edit / no rebuild). "
                             "'yaml' (default): use the YAML setting.  'true': "
                             "force enable.  'false': force disable.  Wired to "
                             "the ros2 launch argument 'friction_ff_enable' so "
                             "the platform A/B for PR 3a (see plans/active/"
                             "friction-ff-motor-guard-integration.md §9) can "
                             "be done by relaunching only.")
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )

    if args.friction_ff == 'yaml':
        friction_ff_override: bool | None = None
    else:
        friction_ff_override = (args.friction_ff == 'true')

    geom = StewartGeometry()
    guard = MotorGuard(rate_hz=args.rate, geom=geom,
                       friction_ff_enable_override=friction_ff_override)

    # Graceful shutdown on SIGINT/SIGTERM
    def signal_handler(sig, frame):
        logger.info("Shutdown signal received")
        guard.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        guard.run()
    finally:
        guard.ipc.close()
        logger.info(f"Final loop timing: {guard.stats.summary()}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
