"""Standalone fixed-rate control process for the Stewart platform.

Runs independently from ROS2 for timing predictability.  Communicates
with the ROS2 world via the IPC layer (ZeroMQ).

Phase 3 scope:
  - Fixed-rate main loop with nanosecond timing instrumentation
  - IPC message dispatch (targets, mode commands, motor feedback)
  - Position IK passthrough (pose → leg extensions → motor revolutions)
  - PD feedback in leg extension space (mm → N → Nm)
  - Gravity feedforward via Jacobian inverse-transpose
  - E-stop on IPC heartbeat loss

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
    twist_to_leg_velocities,
)
from jugglebot.motion.conversions import (
    extensions_mm_to_revs,
    leg_forces_to_motor_torques,
    revs_to_extensions_mm,
    motor_velocities_to_leg_velocities,
)
from jugglebot.motion.dynamics import DynamicsParams, gravity_to_motor_torques
from jugglebot.motion.ipc import (
    TOPIC_MODE,
    TOPIC_MOTOR_FB,
    TOPIC_TARGET,
    ControlProcessIPC,
    make_telemetry,
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
    target_rate_hz : float — desired loop rate
    geom : StewartGeometry
    ipc : ControlProcessIPC — IPC transport (injected for testability)
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

        # Motor feedback from bridge (in motor units: rev, rev/s, A)
        self._motor_positions = np.zeros(6)
        self._motor_velocities = np.zeros(6)
        self._motor_currents = np.zeros(6)

        # Control outputs
        self._commanded_leg_positions = np.zeros(6)
        self._commanded_leg_velocities = np.zeros(6)
        self._commanded_torques = np.zeros(6)

        # Dynamics parameters (for gravity feedforward)
        self._dynamics_params = DynamicsParams.from_config()

        # PD gains — per-leg arrays, in leg-extension space
        # Kp: N/mm  (force per position error)
        # Kd: N·s/mm  (force per velocity error)
        self._kp = np.full(6, 1.0)    # conservative default
        self._kd = np.full(6, 0.01)   # conservative default

        # Feedforward enable flag (can be toggled via IPC for A/B testing)
        self._feedforward_enabled = True

        # Diagnostic torque split (for telemetry)
        self._ff_torques = np.zeros(6)
        self._pd_torques = np.zeros(6)

        # Logging interval
        self._last_log_time = 0.0
        self._log_interval_s = 5.0

    def run(self) -> None:
        """Main loop.  Blocks until stop() is called or signal received."""
        self._running = True
        logger.info(f"Control loop starting at {self.rate_hz} Hz "
                    f"(dt={self.dt_target*1000:.2f} ms)")

        t_prev = time.perf_counter()

        while self._running:
            t_start = time.perf_counter()
            dt_actual = t_start - t_prev
            t_prev = t_start

            self.stats.record(dt_actual)

            # 1. Read all pending IPC messages
            self._process_ipc()

            # 2. Check IPC heartbeat
            self._check_heartbeat()

            # 3. Compute control output
            self._compute()

            # 4. Publish telemetry
            self._publish_telemetry(dt_actual)

            # 5. Periodic logging
            self._periodic_log(t_start)

            # 6. Sleep for remainder of cycle
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
                logger.info("Control loop ENABLED")
        elif cmd == 'disable':
            self.mode = ControlMode.DISABLED
            self._zero_outputs()
            logger.info("Control loop DISABLED")
        elif cmd == 'estop':
            self.mode = ControlMode.ESTOP
            self._zero_outputs()
            logger.warning("E-STOP triggered via command")
        elif cmd == 'set_gains':
            self._apply_gains(params)
        elif cmd == 'set_feedforward':
            enabled = bool(params.get('enabled', True))
            self._feedforward_enabled = enabled
            logger.info(f"Feedforward {'ENABLED' if enabled else 'DISABLED'}")
        else:
            logger.warning(f"Unknown mode command: {cmd}")

    def _on_motor_feedback(self, msg: dict) -> None:
        """Handle motor feedback from the bridge."""
        self._motor_positions = np.array(msg['pos'])
        self._motor_velocities = np.array(msg['vel'])
        self._motor_currents = np.array(msg['cur'])

    # ------------------------------------------------------------------
    # Heartbeat watchdog
    # ------------------------------------------------------------------

    def _check_heartbeat(self) -> None:
        """E-stop if no IPC messages received within timeout."""
        if self.mode == ControlMode.ENABLED:
            if self.ipc.seconds_since_last_recv > IPC_HEARTBEAT_TIMEOUT_S:
                self.mode = ControlMode.ESTOP
                self._zero_outputs()
                logger.warning(
                    f"IPC heartbeat lost ({self.ipc.seconds_since_last_recv:.1f}s "
                    f"since last message) — E-STOP")

    # ------------------------------------------------------------------
    # Control computation
    # ------------------------------------------------------------------

    def _compute(self) -> None:
        """Compute control output for this cycle.

        1. Position IK: pose → desired leg extensions (mm) → motor revolutions
        2. Velocity IK: twist → desired leg velocities (mm/s)
        3. PD feedback: error in leg extension space → force → motor torque
        4. Gravity feedforward: dynamics model → motor torque
        5. Sum: τ_total = τ_PD + τ_FF
        """
        if self.mode != ControlMode.ENABLED or not self._has_target:
            return

        # Position IK: pose → desired leg extensions (mm)
        desired_extensions_mm = pose_to_leg_lengths(
            self._target_pos, self._target_rot, self.geom)

        # Convert to motor revolutions (for position command path)
        self._commanded_leg_positions = extensions_mm_to_revs(
            desired_extensions_mm, self.geom)

        # Velocity IK: twist → desired leg velocities (mm/s)
        desired_vel_mm_s = twist_to_leg_velocities(
            self._target_twist, self._target_pos, self._target_rot, self.geom)
        self._commanded_leg_velocities = desired_vel_mm_s

        # --- PD feedback in leg extension space ---
        # Convert motor feedback to leg space
        actual_extensions_mm = revs_to_extensions_mm(
            self._motor_positions, self.geom)
        actual_vel_mm_s = motor_velocities_to_leg_velocities(
            self._motor_velocities, self.geom)

        # Position and velocity error
        e_pos = desired_extensions_mm - actual_extensions_mm  # mm
        e_vel = desired_vel_mm_s - actual_vel_mm_s            # mm/s

        # PD force per leg (N), then convert to motor torque (Nm)
        f_pd = self._kp * e_pos + self._kd * e_vel
        self._pd_torques = leg_forces_to_motor_torques(f_pd, self.geom)

        # --- Gravity feedforward ---
        if self._feedforward_enabled:
            self._ff_torques = gravity_to_motor_torques(
                self._target_pos, self._target_rot,
                self.geom, self._dynamics_params)
        else:
            self._ff_torques = np.zeros(6)

        # Total commanded torque
        self._commanded_torques = self._pd_torques + self._ff_torques

    def _apply_gains(self, params: dict) -> None:
        """Apply PD gain parameters from an IPC command.

        Accepts scalar values (applied to all legs) or per-leg lists.
        """
        if 'kp' in params:
            kp = params['kp']
            self._kp = np.full(6, kp) if np.isscalar(kp) else np.asarray(kp)
            logger.info(f"Kp set to {self._kp}")
        if 'kd' in params:
            kd = params['kd']
            self._kd = np.full(6, kd) if np.isscalar(kd) else np.asarray(kd)
            logger.info(f"Kd set to {self._kd}")

    def _zero_outputs(self) -> None:
        """Zero all control outputs (safe state)."""
        self._commanded_leg_positions = np.zeros(6)
        self._commanded_leg_velocities = np.zeros(6)
        self._commanded_torques = np.zeros(6)
        self._ff_torques = np.zeros(6)
        self._pd_torques = np.zeros(6)

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def _publish_telemetry(self, dt_actual: float) -> None:
        """Send telemetry to the bridge."""
        msg = make_telemetry(
            leg_positions=self._commanded_leg_positions.tolist(),
            leg_velocities=self._commanded_leg_velocities.tolist(),
            commanded_torques=self._commanded_torques.tolist(),
            loop_dt_s=dt_actual,
            ff_torques=self._ff_torques.tolist(),
            pd_torques=self._pd_torques.tolist(),
        )
        self.ipc.send_telemetry(msg)

    def _periodic_log(self, t_now: float) -> None:
        """Log timing statistics periodically."""
        if t_now - self._last_log_time >= self._log_interval_s:
            self._last_log_time = t_now
            if self.stats.count > 0:
                logger.info(f"Loop timing: {self.stats.summary()}")


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
        loop.ipc.close()
        logger.info(f"Final loop timing: {loop.stats.summary()}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
