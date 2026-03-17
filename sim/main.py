"""Simulation entry point — run the MuJoCo Stewart platform simulation.

Usage examples:
    python sim/main.py                              # viewer, hold at home
    python sim/main.py --pose 0,0,50,0,0,0          # command z+50mm (direct)
    python sim/main.py --no-viewer --duration 5      # headless 5-second run

    # Sequence: pose@time  pose@time  ...  (space-separated)
    python sim/main.py --sequence "0,0,50,0,0,0@0.5  0,0,0,0,0,0@2.0"

    # MPC mode: use the NMPC controller instead of direct IK commands
    python sim/main.py --mpc --pose 0,0,50,0,0,0
    python sim/main.py --mpc --sequence "0,0,50,0,0,0@0.5 0,0,0,0,0,0@2.0"

    # Interactive input (Phase 4): MPC plans smooth motion to live target
    python sim/main.py --spacemouse                  # SpaceMouse (Linux only)
    python sim/main.py --keyboard                    # Keyboard (cross-platform)

    # Live telemetry dashboard (open http://localhost:8082 in browser)
    python sim/main.py --dashboard --mpc --pose 0,0,50,0,0,0
"""

import argparse
import os
import sys
import time
import datetime

import numpy as np

# Ensure sim/ is importable
_sim_dir = os.path.dirname(os.path.abspath(__file__))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

from plant.mujoco_plant import MuJoCoPlant
from plant.interface import PlantState
from viz.telemetry import TelemetryLogger, StepRecord, record_from_arrays


# MPC control rate (Hz).
CONTROL_RATE_HZ = 50
CONTROL_DT = 1.0 / CONTROL_RATE_HZ


def parse_args():
    p = argparse.ArgumentParser(description='Jugglebot MuJoCo simulation')
    p.add_argument('--no-viewer', action='store_true',
                   help='Run headless (no MuJoCo viewer)')
    p.add_argument('--mpc', action='store_true',
                   help='Use MPC controller instead of direct IK commands')
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
    p.add_argument('--log-dir', type=str, default=os.path.join(_sim_dir, 'logs'),
                   help='Directory for telemetry CSV output')
    p.add_argument('--trajectory', type=str, default=None,
                   help='Scripted trajectory name (T1, T2, T3, T4). '
                        'Implies --mpc. Overrides --pose/--sequence.')
    p.add_argument('--spacemouse', action='store_true',
                   help='SpaceMouse interactive input (Linux only, implies --mpc)')
    p.add_argument('--keyboard', action='store_true',
                   help='Keyboard interactive input (cross-platform, implies --mpc)')
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


def _pose_6dof_from_state(state: PlantState) -> np.ndarray:
    """Extract [x,y,z,rx,ry,rz] from a PlantState."""
    return np.concatenate([state.platform_pos_mm, state.platform_rot])


# ---------------------------------------------------------------------------
# Reference scheduler (for MPC — does NOT command the plant directly)
# ---------------------------------------------------------------------------

class ReferenceScheduler:
    """Returns the target pose based on a time schedule.

    Unlike PoseScheduler, this does not call plant.command() — the MPC
    controller is responsible for generating and applying commands.
    """

    def __init__(self, schedule: list[tuple[float, np.ndarray]]):
        self._schedule = schedule
        self._target_pose = np.zeros(6)

    def update(self, sim_time: float) -> np.ndarray:
        """Return the target pose for the given simulation time."""
        for t, pose in self._schedule:
            if sim_time >= t:
                self._target_pose = pose
            else:
                break
        return self._target_pose

    @property
    def target_pose(self) -> np.ndarray:
        return self._target_pose


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
        actual_pose=_pose_6dof_from_state(state),
        actual_twist=state.platform_twist,
        cmd_extensions=scheduler.cmd_extensions,
        actual_extensions=state.leg_extensions_mm,
        leg_velocities=state.leg_velocities_mmps,
    )
    logger.append(record)
    if dashboard is not None:
        dashboard.broadcast(record)


def _log_mpc_step(logger: TelemetryLogger, state: PlantState,
                  ref_pose: np.ndarray, cmd_ext: np.ndarray,
                  diag: dict, ref_twist: np.ndarray | None = None,
                  dashboard=None) -> None:
    """Record one telemetry step (MPC mode)."""
    record = record_from_arrays(
        time=state.time,
        ref_pose=ref_pose,
        ref_twist=ref_twist if ref_twist is not None else np.zeros(6),
        actual_pose=_pose_6dof_from_state(state),
        actual_twist=state.platform_twist,
        cmd_extensions=cmd_ext,
        actual_extensions=state.leg_extensions_mm,
        leg_velocities=state.leg_velocities_mmps,
        solve_time_ms=diag.get('solve_time_ms', 0.0),
        solve_status=diag.get('status', 'n/a'),
        cost=diag.get('cost', 0.0),
        constraint_violation=diag.get('constraint_violation', 0.0),
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
# MPC loops
# ---------------------------------------------------------------------------

def run_mpc_headless(plant: MuJoCoPlant, mpc, schedule, duration: float,
                     logger: TelemetryLogger, dashboard=None) -> None:
    """Run the MPC simulation loop without a viewer."""
    ref = ReferenceScheduler(schedule)
    n_steps = int(duration / CONTROL_DT)

    for step_idx in range(n_steps):
        state = plant.get_state()
        target = ref.update(state.time)
        cmd, diag = mpc.solve(state, target)
        plant.command(cmd)
        plant.step(CONTROL_DT)
        _log_mpc_step(logger, state, target, cmd, diag, dashboard=dashboard)

    logger.flush()
    _print_mpc_summary(logger)


def run_mpc_with_viewer(plant: MuJoCoPlant, mpc, schedule, duration: float,
                        logger: TelemetryLogger, dashboard=None) -> None:
    """Run the MPC simulation loop with the MuJoCo passive viewer."""
    import mujoco.viewer
    from viz.horizon import HorizonRenderer

    ref = ReferenceScheduler(schedule)
    horizon = HorizonRenderer(plant.geom.init_height_mm)

    with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
        start_wall = time.monotonic()
        sim_time_target = 0.0

        while viewer.is_running() and sim_time_target < duration:
            state = plant.get_state()
            target = ref.update(state.time)
            cmd, diag = mpc.solve(state, target)
            plant.command(cmd)
            plant.step(CONTROL_DT)
            sim_time_target += CONTROL_DT

            _log_mpc_step(logger, state, target, cmd, diag, dashboard=dashboard)

            # Horizon visualisation
            horizon.update(mpc.predicted_poses)
            horizon.render(viewer)

            viewer.sync()

            # Real-time pacing
            elapsed = time.monotonic() - start_wall
            sleep_time = sim_time_target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    logger.flush()
    _print_mpc_summary(logger)


# ---------------------------------------------------------------------------
# Trajectory-aware MPC loops (Phase 3)
# ---------------------------------------------------------------------------

def run_trajectory_headless(plant: MuJoCoPlant, mpc, ref_gen, duration: float,
                            logger: TelemetryLogger, dashboard=None) -> None:
    """Run MPC with a ReferenceGenerator (trajectory mode), headless."""
    n_steps = int(duration / CONTROL_DT)

    for step_idx in range(n_steps):
        state = plant.get_state()
        poses, twists = ref_gen.evaluate(state.time, CONTROL_DT, mpc.params.N)
        cmd, diag = mpc.solve(state, poses, ref_twist=twists)
        plant.command(cmd)
        plant.step(CONTROL_DT)
        _log_mpc_step(logger, state, poses[0], cmd, diag,
                      ref_twist=twists[0], dashboard=dashboard)

    logger.flush()
    _print_mpc_summary(logger)


def run_trajectory_with_viewer(plant: MuJoCoPlant, mpc, ref_gen, duration: float,
                               logger: TelemetryLogger, dashboard=None) -> None:
    """Run MPC with a ReferenceGenerator (trajectory mode), with viewer."""
    import mujoco.viewer
    from viz.horizon import HorizonRenderer

    horizon = HorizonRenderer(plant.geom.init_height_mm)

    with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
        start_wall = time.monotonic()
        sim_time_target = 0.0

        while viewer.is_running() and sim_time_target < duration:
            state = plant.get_state()
            poses, twists = ref_gen.evaluate(state.time, CONTROL_DT, mpc.params.N)
            cmd, diag = mpc.solve(state, poses, ref_twist=twists)
            plant.command(cmd)
            plant.step(CONTROL_DT)
            sim_time_target += CONTROL_DT

            _log_mpc_step(logger, state, poses[0], cmd, diag,
                          ref_twist=twists[0], dashboard=dashboard)

            horizon.update(mpc.predicted_poses)
            horizon.render(viewer)
            viewer.sync()

            elapsed = time.monotonic() - start_wall
            sleep_time = sim_time_target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    logger.flush()
    _print_mpc_summary(logger)


# ---------------------------------------------------------------------------
# Interactive input MPC loops (Phase 4)
# ---------------------------------------------------------------------------

def run_interactive_with_viewer(plant: MuJoCoPlant, mpc, input_source,
                                duration: float, logger: TelemetryLogger,
                                dashboard=None) -> None:
    """Run MPC with live interactive input (spacemouse or keyboard), with viewer.

    Parameters
    ----------
    input_source : object
        Must have a ``read() -> np.ndarray`` method returning (6,) target pose,
        and optionally a ``key_callback`` attribute for keyboard input.
    """
    import mujoco.viewer
    from viz.horizon import HorizonRenderer

    horizon = HorizonRenderer(plant.geom.init_height_mm)

    # key_callback must be passed at construction — cannot be set later
    kc = getattr(input_source, 'key_callback', None)
    with mujoco.viewer.launch_passive(
        plant.model, plant.data, key_callback=kc
    ) as viewer:
        start_wall = time.monotonic()
        sim_time_target = 0.0

        while viewer.is_running() and sim_time_target < duration:
            state = plant.get_state()

            # Integrate held-key motion (keyboard); no-op for spacemouse
            if hasattr(input_source, 'apply'):
                input_source.apply(CONTROL_DT)

            # Read the latest target from the input device
            target_pose = input_source.read()

            # Build a static reference for the MPC horizon
            # (MPC plans smooth optimal path to reach the target)
            ref_poses = np.tile(target_pose, (mpc.params.N + 1, 1))
            ref_twists = np.zeros_like(ref_poses)

            cmd, diag = mpc.solve(state, ref_poses, ref_twist=ref_twists)
            plant.command(cmd)
            plant.step(CONTROL_DT)
            sim_time_target += CONTROL_DT

            _log_mpc_step(logger, state, target_pose, cmd, diag,
                          ref_twist=np.zeros(6), dashboard=dashboard)

            horizon.update(mpc.predicted_poses)
            horizon.render(viewer)
            viewer.sync()

            # Real-time pacing
            elapsed = time.monotonic() - start_wall
            sleep_time = sim_time_target - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    logger.flush()
    _print_mpc_summary(logger)


def _print_mpc_summary(logger: TelemetryLogger) -> None:
    """Print summary statistics from an MPC run."""
    if not logger.records:
        return
    final = logger.records[-1]
    solve_times = [r.solve_time_ms for r in logger.records if r.solve_time_ms > 0]
    print(f"Final tracking error: {final.tracking_error_mm:.3f} mm, "
          f"{final.tracking_error_deg:.4f} deg")
    if solve_times:
        print(f"Solve time: mean={np.mean(solve_times):.1f} ms, "
              f"max={np.max(solve_times):.1f} ms, "
              f"p95={np.percentile(solve_times, 95):.1f} ms")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    # Interactive input mode (Phase 4) — overrides everything else
    input_source = None
    if args.spacemouse:
        args.mpc = True
        from input.spacemouse import SpaceMouseInput
        input_source = SpaceMouseInput()
        if not input_source.connected:
            print("ERROR: SpaceMouse not available. Use --keyboard instead.")
            sys.exit(1)
        label = "SpaceMouse interactive"
        default_duration = 300.0  # 5 minutes, effectively unlimited
    elif args.keyboard:
        args.mpc = True
        from input.keyboard import KeyboardInput
        input_source = KeyboardInput()
        label = "Keyboard interactive"
        default_duration = 300.0

    # Trajectory mode (Phase 3) — overrides --pose/--sequence
    ref_gen = None
    if args.trajectory and input_source is None:
        args.mpc = True  # trajectories always use MPC
        from input.scripted import get_trajectory
        ref_gen, default_duration = get_trajectory(args.trajectory)
        label = f"Trajectory: {args.trajectory}"
        schedule = None  # not used in trajectory mode
    elif input_source is not None:
        schedule = None  # not used in interactive mode
    elif args.sequence:
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
        label = "Home"
        default_duration = 10.0

    duration = args.duration if args.duration is not None else default_duration

    mode = "MPC" if args.mpc else "Direct"
    print(f"{label} [{mode}]")
    if schedule is not None:
        for t, pose in schedule:
            print(f"  t={t:.1f}s -> [{', '.join(f'{v:.2f}' for v in pose)}]")
    print(f"Duration: {duration:.1f}s, Control rate: {CONTROL_RATE_HZ} Hz")

    # Create plant
    plant = MuJoCoPlant()

    # Set up telemetry logging
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    prefix = 'mpc' if args.mpc else 'sim'
    log_path = os.path.join(args.log_dir, f'{prefix}_{timestamp}.csv')
    logger = TelemetryLogger(log_path)
    print(f"Logging to: {log_path}")

    # Optional live dashboard
    dashboard = None
    if args.dashboard:
        from viz.dashboard import DashboardServer
        dashboard = DashboardServer(port=args.dashboard_port)
        dashboard.start()

    # Optional MPC controller
    mpc = None
    if args.mpc:
        from controller import MPCController, MPCParams
        # Use generous solver budget for interactive use (cold starts need headroom).
        # The tight 18ms budget is for production (Phase 6) where the control
        # period is a hard deadline. In simulation, we want correct solutions.
        param_overrides = dict(max_cpu_time=2.0, max_iter=500)
        if ref_gen is not None or input_source is not None:
            param_overrides['max_leg_vel_mmps'] = 1000.0
        mpc = MPCController.from_plant(MPCParams(**param_overrides), plant)
        print("MPC controller initialised "
              f"(N={mpc.params.N}, tau={mpc.params.tau*1000:.0f} ms, "
              f"v_max={mpc.params.max_leg_vel_mmps:.0f} mm/s)")

    try:
        if input_source is not None:
            # Interactive input mode (Phase 4) — viewer required
            if args.no_viewer:
                print("ERROR: --spacemouse/--keyboard requires the viewer "
                      "(remove --no-viewer)")
                sys.exit(1)
            run_interactive_with_viewer(plant, mpc, input_source, duration,
                                       logger, dashboard)
        elif ref_gen is not None:
            # Trajectory mode (Phase 3)
            if args.no_viewer:
                run_trajectory_headless(plant, mpc, ref_gen, duration, logger, dashboard)
            else:
                run_trajectory_with_viewer(plant, mpc, ref_gen, duration, logger, dashboard)
        elif args.mpc:
            if args.no_viewer:
                run_mpc_headless(plant, mpc, schedule, duration, logger, dashboard)
            else:
                run_mpc_with_viewer(plant, mpc, schedule, duration, logger, dashboard)
        else:
            if args.no_viewer:
                run_headless(plant, schedule, duration, logger, dashboard)
            else:
                run_with_viewer(plant, schedule, duration, logger, dashboard)
    except KeyboardInterrupt:
        print("\nInterrupted — flushing telemetry...")
        logger.flush()
    finally:
        if input_source is not None and hasattr(input_source, 'close'):
            input_source.close()
        if dashboard is not None:
            dashboard.stop()

    print(f"Logged {len(logger.records)} records to {log_path}")


if __name__ == '__main__':
    main()
