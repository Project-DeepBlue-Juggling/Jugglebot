"""Simulation entry point — run the MuJoCo Stewart platform simulation.

Usage examples:
    python sim/main.py                              # viewer, hold at active pose
    python sim/main.py --pose 0,0,50,0,0,0          # command z+50mm (direct)
    python sim/main.py --no-viewer --duration 5      # headless 5-second run

    # Sequence: pose@time  pose@time  ...  (space-separated)
    python sim/main.py --sequence "0,0,50,0,0,0@0.5  0,0,0,0,0,0@2.0"

    # Live telemetry dashboard (open http://localhost:8082 in browser)
    python sim/main.py --dashboard --pose 0,0,50,0,0,0

History: the MPC control modes (``--mpc``, ``--hardware``, and the
catch/toss/juggle/keyboard/spacemouse/trajectory modes that implied them)
were removed 2026-09-01 — dormant since 2026-08-01 and superseded by the
unified 7-DoF planner (``plans/active/unified-7dof-planner.md``) as the
per-cycle replanner.  The final MPC implementation lives at git tag
``mpc-final``; see ``logbook/2026-09-01-mpc-chain-removed.md``.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
import datetime

import numpy as np

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

from sim.plant.mujoco_plant import MuJoCoPlant
from sim.plant.interface import PlantState
from sim.viz.telemetry import TelemetryLogger, record_from_arrays


# Simulation control rate (Hz).
CONTROL_RATE_HZ = 40
CONTROL_DT = 1.0 / CONTROL_RATE_HZ


def parse_args():
    p = argparse.ArgumentParser(description='Jugglebot MuJoCo simulation')
    p.add_argument('--no-viewer', action='store_true',
                   help='Run headless (no MuJoCo viewer)')
    p.add_argument('--pose', type=str, default=None,
                   help='Target pose: x,y,z,rx,ry,rz (mm, rad). '
                        'Example: 0,0,50,0,0,0')
    p.add_argument('--sequence', type=str, default=None,
                   help='Pose sequence: "x,y,z,rx,ry,rz@t ..." '
                        'Each pose is commanded at time t (seconds). '
                        'Example: "0,0,50,0,0,0@0.5 0,0,0,0,0,0@2.0"')
    p.add_argument('--duration', type=float, default=None,
                   help='Simulation duration in seconds '
                        '(default: 10, or last sequence time + 2)')
    p.add_argument('--log-dir', type=str, default=os.path.join(_repo_root, 'temp', 'logs'),
                   help='Directory for telemetry CSV output')
    p.add_argument('--dashboard', action='store_true',
                   help='Start live telemetry dashboard (web browser)')
    p.add_argument('--dashboard-port', type=int, default=8082,
                   help='Dashboard server port (default: 8082)')
    return p.parse_args()


def _parse_pose(pose_str: str) -> np.ndarray:
    """Parse comma-separated pose string into (6,) array."""
    vals = [float(v) for v in pose_str.split(',')]
    if len(vals) != 6:
        raise ValueError(f"Pose must have 6 values, got {len(vals)}: {pose_str}")
    return np.array(vals)


def _parse_sequence(seq_str: str) -> list[tuple[float, np.ndarray]]:
    """Parse 'pose@time pose@time ...' into sorted [(time, pose), ...]."""
    entries = seq_str.strip().split()
    schedule: list[tuple[float, np.ndarray]] = []

    for entry in entries:
        if '@' not in entry:
            raise ValueError(f"Sequence entry must be 'pose@time', got: {entry}")
        pose_str, time_str = entry.rsplit('@', 1)
        schedule.append((float(time_str), _parse_pose(pose_str)))

    schedule.sort(key=lambda x: x[0])
    return schedule


def pose_6dof_from_state(state: PlantState) -> np.ndarray:
    """Extract [x,y,z,rx,ry,rz] from a PlantState."""
    return np.concatenate([state.platform_pos_mm, state.platform_rot])


# ---------------------------------------------------------------------------
# Direct-command mode (original Phase 1 path)
# ---------------------------------------------------------------------------

class PoseScheduler:
    """Tracks which pose to command based on simulation time.

    Given a sorted list of (time, pose) pairs, returns the current target
    pose and commanded extensions for any given time.
    """

    def __init__(self, schedule: list[tuple[float, np.ndarray]], plant: MuJoCoPlant):
        self._schedule = schedule
        self._plant = plant
        self._current_idx = -1
        self._target_pose = np.zeros(6)
        self._cmd_extensions = plant.pose_to_extensions(self._target_pose)

    def update(self, sim_time: float) -> bool:
        """Advance to the correct pose for the given time.

        Returns True if the pose changed this call.
        """
        new_idx = self._current_idx
        for i, (t, _) in enumerate(self._schedule):
            if sim_time >= t:
                new_idx = i
            else:
                break

        if new_idx != self._current_idx:
            self._current_idx = new_idx
            self._target_pose = self._schedule[new_idx][1]
            self._cmd_extensions = self._plant.pose_to_extensions(self._target_pose)
            self._plant.command(self._cmd_extensions)
            return True
        return False

    @property
    def target_pose(self) -> np.ndarray:
        return self._target_pose

    @property
    def cmd_extensions(self) -> np.ndarray:
        return self._cmd_extensions


def _log_step(logger: TelemetryLogger, state: PlantState,
              scheduler: PoseScheduler, dashboard=None) -> None:
    """Record one telemetry step (direct-command mode)."""
    record = record_from_arrays(
        time=state.time,
        ref_pose=scheduler.target_pose,
        ref_twist=np.zeros(6),
        actual_pose=pose_6dof_from_state(state),
        actual_twist=state.platform_twist,
        cmd_extensions=scheduler.cmd_extensions,
        actual_extensions=state.leg_extensions_mm,
        leg_velocities=state.leg_velocities_mmps,
    )
    logger.append(record)
    if dashboard is not None:
        dashboard.broadcast(record)


# ---------------------------------------------------------------------------
# Direct-command loops
# ---------------------------------------------------------------------------

def run_headless(plant: MuJoCoPlant, schedule: list[tuple[float, np.ndarray]],
                 duration: float, logger: TelemetryLogger,
                 dashboard=None) -> None:
    """Run the simulation loop without a viewer (direct-command mode)."""
    scheduler = PoseScheduler(schedule, plant)
    n_steps = int(duration / CONTROL_DT)

    for _ in range(n_steps):
        plant.step(CONTROL_DT)
        state = plant.get_state()
        scheduler.update(state.time)
        _log_step(logger, state, scheduler, dashboard)

    logger.flush()
    final = plant.get_state()
    print(f"Final pose: pos={final.platform_pos_mm} mm, "
          f"rot={np.degrees(final.platform_rot)} deg")
    if logger.records:
        print(f"Tracking error: {logger.records[-1].tracking_error_mm:.3f} mm, "
              f"{logger.records[-1].tracking_error_deg:.4f} deg")


def run_with_viewer(plant: MuJoCoPlant, schedule: list[tuple[float, np.ndarray]],
                    duration: float, logger: TelemetryLogger,
                    dashboard=None) -> None:
    """Run the simulation loop with the MuJoCo passive viewer (direct-command)."""
    import mujoco.viewer

    scheduler = PoseScheduler(schedule, plant)

    with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
        start_wall = time.monotonic()
        sim_time_target = 0.0

        while viewer.is_running() and sim_time_target < duration:
            plant.step(CONTROL_DT)
            sim_time_target += CONTROL_DT

            state = plant.get_state()
            scheduler.update(state.time)
            _log_step(logger, state, scheduler, dashboard)

            viewer.sync()

            # Real-time pacing
            elapsed_wall = time.monotonic() - start_wall
            sleep_time = sim_time_target - elapsed_wall
            if sleep_time > 0:
                time.sleep(sleep_time)

    logger.flush()
    if logger.records:
        final_rec = logger.records[-1]
        print(f"Final tracking error: {final_rec.tracking_error_mm:.3f} mm, "
              f"{final_rec.tracking_error_deg:.4f} deg")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    # ---- Mode selection → pose schedule + label + default_duration ----

    schedule = None          # pose schedule (direct-command)

    if args.sequence:
        schedule = _parse_sequence(args.sequence)
        label = f"Sequence: {len(schedule)} poses"
        default_duration = schedule[-1][0] + 2.0

    elif args.pose:
        target = _parse_pose(args.pose)
        schedule = [(0.0, target)]
        label = f"Static pose: {target}"
        default_duration = 10.0

    else:
        schedule = [(0.0, np.zeros(6))]
        label = "Active pose (hold)"
        default_duration = 10.0

    duration = args.duration if args.duration is not None else default_duration

    print(f"{label} [Direct, sim]")
    if schedule is not None:
        for t, pose in schedule:
            print(f"  t={t:.1f}s -> [{', '.join(f'{v:.2f}' for v in pose)}]")
    print(f"Duration: {duration:.1f}s, Control rate: {CONTROL_RATE_HZ} Hz")

    # Create plant
    plant = MuJoCoPlant(control_dt=CONTROL_DT)

    # Set up telemetry logging
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(args.log_dir, f'sim_{timestamp}.csv')
    logger = TelemetryLogger(log_path)
    print(f"Logging to: {log_path}")

    # Optional live dashboard
    dashboard = None
    if args.dashboard:
        from sim.viz.dashboard import DashboardServer
        dashboard = DashboardServer(port=args.dashboard_port)
        dashboard.start()

    # ---- Run ----

    try:
        if args.no_viewer:
            run_headless(plant, schedule, duration, logger, dashboard)
        else:
            run_with_viewer(plant, schedule, duration, logger, dashboard)
    except KeyboardInterrupt:
        print("\nInterrupted — flushing telemetry...")
        logger.flush()
    finally:
        if dashboard is not None:
            dashboard.stop()

    print(f"Logged {logger.total_count} records to {log_path}")


if __name__ == '__main__':
    main()
