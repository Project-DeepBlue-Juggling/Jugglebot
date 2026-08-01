"""Record sim baselines for all standard trajectories (T1-T6).

Runs each trajectory through the MPC controller in headless sim mode and
saves the resulting StepRecord CSV to ``sim/analysis/baselines/``.

These baselines serve as the reference for sim-to-real comparison: when
the same trajectory is run on real hardware, the hardware CSV can be
compared against the baseline using ``compare.py``.

Usage:
    python -m analysis.record_baselines              # all trajectories
    python -m analysis.record_baselines T1 T3        # specific ones
    python -m analysis.record_baselines --force       # overwrite existing
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()

from sim.plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from sim.input.scripted import TRAJECTORIES, get_trajectory
from sim.viz.telemetry import TelemetryLogger, record_from_arrays

CONTROL_DT = 0.02  # 50 Hz
BASELINES_DIR = os.path.join(os.path.dirname(__file__), 'baselines')


def run_trajectory(
    name: str,
    plant: MuJoCoPlant,
    mpc: MPCController,
) -> list:
    """Run a single trajectory and return the TelemetryLogger's records."""
    waypoints, duration = get_trajectory(name)
    n_steps = int(duration / CONTROL_DT)

    logger = TelemetryLogger()
    idx = 0

    for _ in range(n_steps):
        state = plant.get_state()

        # Advance past reached waypoints
        while idx + 1 < len(waypoints) and state.time >= waypoints[idx][1]:
            idx += 1
        target_pose, arrival_time = waypoints[idx]

        from controller.target import flat_target_to_events
        current_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        ref_events = flat_target_to_events(
            current_pose, state.platform_twist, target_pose, state.time,
            arrival_time=arrival_time,
            v_max_mmps=mpc.params.max_leg_vel_mmps, tau_s=mpc.params.tau)
        cmd, _cmd_vel, diag = mpc.solve(state, target_pose, ref_events=ref_events)
        plant.command(cmd)
        plant.step(CONTROL_DT)

        actual_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        record = record_from_arrays(
            time=state.time,
            ref_pose=target_pose,
            ref_twist=np.zeros(6),
            actual_pose=actual_pose,
            actual_twist=state.platform_twist,
            cmd_extensions=cmd,
            actual_extensions=state.leg_extensions_mm,
            leg_velocities=state.leg_velocities_mmps,
            solve_time_ms=diag.get('solve_time_ms', 0.0),
            solve_status=diag.get('status', 'n/a'),
            cost=diag.get('cost', 0.0),
            constraint_violation=diag.get('constraint_violation', 0.0),
        )
        logger.append(record)

    logger.flush()
    return logger.load()


def record_baseline(
    name: str,
    plant: MuJoCoPlant,
    mpc: MPCController,
    force: bool = False,
) -> str:
    """Record one trajectory baseline. Returns the output CSV path."""
    out_path = os.path.join(BASELINES_DIR, f'{name}_baseline.csv')

    if os.path.exists(out_path) and not force:
        print(f"  {name}: SKIP (exists, use --force to overwrite)")
        return out_path

    plant.reset()

    t0 = time.monotonic()
    records = run_trajectory(name, plant, mpc)
    elapsed = time.monotonic() - t0

    # Write CSV
    logger = TelemetryLogger(out_path)
    for r in records:
        logger.append(r)
    logger.flush()

    # Summary stats
    errors_mm = [r.tracking_error_mm for r in records]
    solve_ms = [r.solve_time_ms for r in records]
    print(f"  {name}: {len(records)} steps, "
          f"RMS err {np.sqrt(np.mean(np.array(errors_mm)**2)):.3f} mm, "
          f"peak err {max(errors_mm):.3f} mm, "
          f"solve p50 {np.median(solve_ms):.1f} ms, "
          f"wall {elapsed:.1f}s "
          f"-> {out_path}")

    return out_path


def main():
    parser = argparse.ArgumentParser(
        description="Record sim baselines for standard trajectories")
    parser.add_argument('trajectories', nargs='*', default=list(TRAJECTORIES.keys()),
                        help=f"Trajectories to record (default: all). "
                             f"Available: {', '.join(TRAJECTORIES.keys())}")
    parser.add_argument('--force', action='store_true',
                        help="Overwrite existing baseline files")
    args = parser.parse_args()

    # Validate trajectory names
    for name in args.trajectories:
        if name not in TRAJECTORIES:
            print(f"Unknown trajectory: {name}. "
                  f"Available: {', '.join(TRAJECTORIES.keys())}")
            sys.exit(1)

    os.makedirs(BASELINES_DIR, exist_ok=True)

    print("=" * 60)
    print("  RECORDING SIM BASELINES")
    print("=" * 60)
    print(f"  Trajectories: {', '.join(args.trajectories)}")
    print(f"  Output dir: {BASELINES_DIR}")
    print(f"  Control rate: {1/CONTROL_DT:.0f} Hz")
    print()

    # Shared plant and MPC (re-used across trajectories for speed)
    plant = MuJoCoPlant()
    params = MPCParams(max_cpu_time=2.0, max_iter=500, max_leg_vel_mmps=1000.0)
    mpc = MPCController.from_plant(params, plant)

    paths = []
    for name in args.trajectories:
        path = record_baseline(name, plant, mpc, force=args.force)
        paths.append(path)

    print()
    print(f"  Done. {len(paths)} baselines in {BASELINES_DIR}/")
    print()
    print("  To compare against a hardware run:")
    print(f"    python -m analysis.compare baselines/T1_baseline.csv <hw_csv>")
    print()


if __name__ == '__main__':
    main()
