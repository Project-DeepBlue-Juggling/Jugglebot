"""Hardware MPC entry point — run the MPC controller on real hardware.

Usage examples:
    python run_mpc.py                              # ZMQ targets from mpc_bridge_node
    python run_mpc.py --pose 0,0,170,0,0,0         # hold at active pose
    python run_mpc.py --pose 0,0,190,0,0,0 --duration 10
    python run_mpc.py --dashboard                   # with live telemetry dashboard

This is the production entry point for hardware. For simulation, use sim/main.py.

Operational notes:
  - The orchestrator must be in ACTIVE (STANDBY or any sub-mode) for motor
    feedback telemetry to flow.  Launching this script in IDLE will fail
    at the `HardwarePlant.enable()` telemetry wait.
  - This process owns the motor guard's enable state.  Startup sends
    `disable+enable` (the leading `disable` clears any lingering ESTOP);
    shutdown (Ctrl-C) sends `disable`.
  - DO NOT launch two `run_mpc.py` processes simultaneously.  Both publish
    commands on :5557 and the motor guard will see interleaved commands
    from two planners, causing jerky or unsafe motion.  No safeguard
    currently prevents this — enforce manually.
  - Ctrl-C this process before clicking Deactivate in the GUI.  Deactivating
    while MPC is running leaves the motor guard in a self-ESTOP state
    (harmless but noisy in telemetry).
"""

from __future__ import annotations

import argparse
import datetime
import os
import sys
import time

import numpy as np

# Ensure repo root is on sys.path for controller/ imports
_repo_root = os.path.dirname(os.path.abspath(__file__))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

# Ensure ros_ws package is importable (for jugglebot.motion.*)
_ros_pkg = os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot')
if _ros_pkg not in sys.path:
    sys.path.insert(0, _ros_pkg)

from controller.mpc import MPCController
from controller.params import MPCParams
from controller.target import TargetCommand, StaticTargetSource
from controller.plant import PlantState
from controller.telemetry import TelemetryLogger
from controller.runner import run_mpc_loop, MpcLoopHooks

# MPC control rate (Hz) — must match motor guard expectations
CONTROL_RATE_HZ = 40
CONTROL_DT = 1.0 / CONTROL_RATE_HZ

# Telemetry data age threshold for MPC deceleration (seconds).
# Must be less than motor guard's MPC_CMD_STALENESS_S (0.25 s).
_MPC_STALE_DECEL_S = 0.125


def parse_args():
    p = argparse.ArgumentParser(
        description='Run MPC controller on real hardware')
    p.add_argument('--pose', type=str, default=None,
                   help='Target pose: x,y,z,rx,ry,rz (mm, rad). '
                        'Z is STOW-relative; active position ≈ 170mm.')
    p.add_argument('--sequence', type=str, default=None,
                   help='Timed pose sequence: "pose@time pose@time ..."')
    p.add_argument('--duration', type=float, default=None,
                   help='Run duration in seconds (default: 24h for ZMQ, '
                        '10s for --pose)')
    p.add_argument('--log-dir', type=str, default='temp/logs',
                   dest='log_dir',
                   help='Directory for telemetry CSV logs')
    p.add_argument('--dashboard', action='store_true',
                   help='Start live telemetry dashboard (web browser)')
    p.add_argument('--dashboard-port', type=int, default=8082,
                   help='Dashboard server port (default: 8082)')
    p.add_argument('--no-torque-ff', dest='enable_torque_ff',
                   action='store_false',
                   help='Disable torque feedforward (send zeros to motor_guard). '
                        'For ablation testing.')
    p.add_argument('--no-vel-ff', dest='enable_vel_ff',
                   action='store_false',
                   help='Disable velocity feedforward (send zeros to motor_guard).')
    p.add_argument('--no-acc-ff', dest='enable_acc_ff',
                   action='store_false',
                   help='Disable acceleration feedforward (zeros cubic interp '
                        'acc+jerk terms in motor_guard).')
    return p.parse_args()


def _parse_pose(pose_str: str) -> np.ndarray:
    vals = [float(v) for v in pose_str.split(',')]
    if len(vals) != 6:
        raise ValueError(f"Pose must have 6 values, got {len(vals)}")
    return np.array(vals)


def _parse_sequence(seq_str: str) -> list[tuple[float, np.ndarray]]:
    entries = seq_str.strip().split()
    schedule: list[tuple[float, np.ndarray]] = []
    for entry in entries:
        if '@' not in entry:
            raise ValueError(f"Sequence entry must be 'pose@time', got: {entry}")
        pose_str, time_str = entry.rsplit('@', 1)
        schedule.append((float(time_str), _parse_pose(pose_str)))
    schedule.sort(key=lambda x: x[0])
    return schedule


class _StdoutTee:
    """Duplicate writes to multiple streams; used to tee stdout/stderr to a log file."""
    def __init__(self, *streams):
        self._streams = streams
    def write(self, data):
        for s in self._streams:
            s.write(data)
            s.flush()
        return len(data)
    def flush(self):
        for s in self._streams:
            s.flush()
    def isatty(self):
        return False


def _start_stdout_tee(log_path: str):
    """Tee stdout/stderr to a companion .log file next to the CSV log_path."""
    stdout_log_path = os.path.splitext(log_path)[0] + '.log'
    os.makedirs(os.path.dirname(stdout_log_path), exist_ok=True)
    fh = open(stdout_log_path, 'w', buffering=1)  # line-buffered
    sys.stdout = _StdoutTee(sys.__stdout__, fh)
    sys.stderr = _StdoutTee(sys.__stderr__, fh)
    return stdout_log_path


def main():
    args = parse_args()

    # --- Target source selection ---
    if args.sequence:
        schedule = _parse_sequence(args.sequence)
        source_label = f"Sequence: {len(schedule)} poses"
        default_duration = schedule[-1][0] + 2.0
    elif args.pose:
        target = _parse_pose(args.pose)
        schedule = [(0.0, target)]
        source_label = f"Static pose: {target}"
        default_duration = 10.0
    else:
        schedule = None
        source_label = "ZMQ targets from mpc_bridge_node"
        default_duration = 86400.0  # 24h — runs until Ctrl-C

    duration = args.duration if args.duration is not None else default_duration
    print(f"Hardware MPC [{source_label}]")
    print(f"Duration: {duration:.1f}s, Control rate: {CONTROL_RATE_HZ} Hz")

    # --- Create HardwarePlant ---
    from controller.hardware_plant import HardwarePlant
    plant = HardwarePlant(
        control_dt=CONTROL_DT,
        enable_torque_ff=args.enable_torque_ff,
        enable_vel_ff=args.enable_vel_ff,
        enable_acc_ff=args.enable_acc_ff,
    )
    print("HardwarePlant: connected to motor_guard via IPC")
    print(
        f"Feedforward: torque={'ON' if args.enable_torque_ff else 'OFF'} "
        f"vel={'ON' if args.enable_vel_ff else 'OFF'} "
        f"acc={'ON' if args.enable_acc_ff else 'OFF'}"
    )

    # --- Target feedback for catch coordinator (accept/reject on :5559) ---
    from jugglebot.motion.ipc import TargetFeedbackPub
    feedback_pub = TargetFeedbackPub()
    print("TargetFeedbackPub: catch feedback on :5559")

    # --- Telemetry logging ---
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(args.log_dir, f'mpc_{timestamp}.csv')
    stdout_log_path = _start_stdout_tee(log_path)
    logger = TelemetryLogger(log_path)
    print(f"Logging to: {log_path}")
    print(f"Stdout log: {stdout_log_path}")

    # --- Session metadata for ROS2 log correlation ---
    from jugglebot.motion.ipc import SessionMetadataPush
    session_push = SessionMetadataPush()
    time.sleep(0.1)  # ZMQ connection establishment
    session_push.send_session_start(os.path.basename(log_path))
    session_push.close()
    print("Session metadata sent to mpc_bridge_node")

    # --- Optional live dashboard ---
    dashboard = None
    if args.dashboard:
        # Import from sim/viz since dashboard is visualization-specific
        _sim_dir = os.path.join(_repo_root, 'sim')
        if _sim_dir not in sys.path:
            sys.path.insert(0, _sim_dir)
        from viz.dashboard import DashboardServer
        dashboard = DashboardServer(port=args.dashboard_port)
        dashboard.start()

    # --- Build MPC controller ---
    # Hardware real-time budget: 18 ms IPOPT (matches params.py default and the
    # AOT-compiled solver in controller/generated/) + ~5 ms overhead = 23 ms,
    # comfortably inside the 25 ms MPC period.  The earlier 22 ms override
    # drove p50 solves over budget on off-Active multi-axis moves and kept
    # the fallback path firing constantly (see logbook
    # 2026-04-17-mpc-fallback-cmd-sawtooth-stutter.md).
    params = MPCParams(max_cpu_time=0.018, max_iter=100)
    mpc = MPCController.from_plant(params, plant)
    assert abs(CONTROL_DT - mpc.params.dt_fine) < 1e-6, (
        f"CONTROL_DT ({CONTROL_DT}) must match MPC dt_fine "
        f"({mpc.params.dt_fine}) for correct warm-start shifting"
    )
    print(f"MPC controller initialised "
          f"(N={mpc.params.N}, tau={mpc.params.tau*1000:.0f} ms, "
          f"v_max={mpc.params.max_leg_vel_mmps:.0f} mm/s)")

    # --- Build target source ---
    if schedule is not None:
        source = StaticTargetSource(schedule)
    else:
        from controller.zmq_target import ZmqTargetSource
        default_z = (plant.geom.active_extensions_mm[0]
                     if hasattr(plant.geom, 'active_extensions_mm')
                     else 170.0)
        source = ZmqTargetSource(default_z_mm=default_z)
        print("ZmqTargetSource: waiting for targets from mpc_bridge_node on :5558")

    # --- Hardware hooks ---
    _fb_last_arrival = [None]

    def _on_target_override(state: PlantState, tc: TargetCommand) -> TargetCommand:
        """Hold-in-place when telemetry is stale."""
        if (state.data_age_s is not None
                and state.data_age_s > _MPC_STALE_DECEL_S):
            return TargetCommand(
                target_pose=np.concatenate(
                    [state.platform_pos_mm, state.platform_rot]),
            )
        return tc

    def _on_pre_command(plant_, mpc_, tc, cmd, cmd_vel, diag):
        """Set feedforward torques from MPC's predicted trajectory."""
        poses = mpc_.predicted_poses_view
        times = mpc_.predicted_times_view
        if poses is not None:
            dt0 = times[1] - times[0]
            dt1 = times[2] - times[1]
            twist = (poses[1] - poses[0]) / dt0
            twist_next = (poses[2] - poses[1]) / dt1
            accel = (twist_next - twist) / (0.5 * (dt0 + dt1))
            plant_.set_pose(poses[0], twist_6dof=twist, accel_6dof=accel)

    def _on_post_solve(tc, diag):
        """Publish accept/reject feedback for catch targets."""
        from jugglebot.motion.ipc import make_target_feedback
        source_id = getattr(source, 'source', '')
        if source_id != 'catch' or tc.arrival_time is None:
            return
        if (_fb_last_arrival[0] is not None
                and abs(tc.arrival_time - _fb_last_arrival[0]) < 0.05):
            return
        accepted = diag.get('status', '') in (
            'Solve_Succeeded', 'Solved_To_Acceptable_Level')
        violations = None if accepted else [diag.get('status', 'unknown')]
        feedback_pub.send(make_target_feedback(
            tc.arrival_time, accepted, 'catch', violations))
        _fb_last_arrival[0] = tc.arrival_time

    def _on_log_extras(plant_):
        return {
            'fk_iterations': getattr(plant_, 'last_fk_iterations', 0),
            'ff_torque_max_Nm': getattr(plant_, 'last_ff_torque_max_Nm', 0.0),
        }

    hooks = MpcLoopHooks(
        on_target_override=_on_target_override,
        on_pre_command=_on_pre_command,
        on_post_solve=_on_post_solve,
        on_log_extras=_on_log_extras,
    )

    # --- Run ---
    try:
        # Enable pass-through mode (skip for ZmqTargetSource — the loop
        # manages enable/disable transitions based on mode messages).
        if not hasattr(source, 'enabled'):
            plant.enable()

        run_mpc_loop(
            plant, mpc, source, duration, logger,
            control_dt=CONTROL_DT, dashboard=dashboard, hooks=hooks,
        )
    except KeyboardInterrupt:
        print("\nInterrupted — flushing telemetry...")
        logger.flush()
    finally:
        plant.disable()
        time.sleep(0.05)
        plant.close()
        feedback_pub.close()
        if hasattr(source, 'close'):
            source.close()
        if dashboard is not None:
            dashboard.stop()

    print(f"Logged {logger.total_count} records to {log_path}")


if __name__ == '__main__':
    main()
