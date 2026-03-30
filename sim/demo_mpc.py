"""Phase 2 MPC demo — shows the controller tracking a sequence of static poses.

Run with viewer (recommended):
    python sim/demo_mpc.py

Run with viewer + live dashboard (open http://localhost:8082):
    python sim/demo_mpc.py --dashboard

Run headless (prints summary only):
    python sim/demo_mpc.py --no-viewer

The demo commands a sequence of reachable poses, pausing at each one to
demonstrate settling behaviour.  At the end it prints tracking statistics
and MPC solve-time percentiles.
"""

import argparse
import os
import sys
import time
import datetime

import numpy as np

_sim_dir = os.path.dirname(os.path.abspath(__file__))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)
_repo_root = os.path.dirname(_sim_dir)
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

from plant.mujoco_plant import MuJoCoPlant
from controller.mpc import MPCController
from controller.params import MPCParams
from viz.telemetry import TelemetryLogger, record_from_arrays


CONTROL_DT = 0.02  # 50 Hz

# Demo pose sequence: (time_s, [x, y, z, rx, ry, rz])
# All poses are reachable (all extensions > 0)
DEMO_SEQUENCE = [
    (0.0,  [0.0,   0.0,    0.0,  0.0,              0.0,              0.0]),        # active pose
    (0.5,  [0.0,   0.0,   50.0,  0.0,              0.0,              0.0]),        # z +50 mm
    (2.0,  [20.0, -15.0,  60.0,  0.0,              0.0,              0.0]),        # lateral offset
    (3.5,  [0.0,   0.0,   80.0,  np.radians(5.0),  np.radians(-3.0), 0.0]),       # tilt at z+80
    (5.0,  [30.0, -20.0,  50.0,  np.radians(3.0),  np.radians(-2.0), 0.0]),       # combined
    (6.5,  [0.0,   0.0,   50.0,  0.0,              0.0,              0.0]),        # back to z+50
    (8.0,  [0.0,   0.0,    0.0,  0.0,              0.0,              0.0]),        # return to active pose
]


def parse_args():
    p = argparse.ArgumentParser(description="Phase 2 MPC demo")
    p.add_argument("--no-viewer", action="store_true", help="Run headless")
    p.add_argument("--dashboard", action="store_true",
                   help="Start live dashboard on http://localhost:8082")
    p.add_argument("--dashboard-port", type=int, default=8082)
    return p.parse_args()


def get_target(sim_time, sequence):
    """Return the current target pose for the given time."""
    target = np.zeros(6)
    for t, pose in sequence:
        if sim_time >= t:
            target = np.array(pose, dtype=float)
        else:
            break
    return target


def run(viewer_mode, dashboard=None):
    plant = MuJoCoPlant()
    params = MPCParams(max_cpu_time=2.0, max_iter=500)
    mpc = MPCController.from_plant(params, plant)

    duration = DEMO_SEQUENCE[-1][0] + 2.0  # 2s after final pose
    n_steps = int(duration / CONTROL_DT)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join(_sim_dir, "logs")
    log_path = os.path.join(log_dir, f"mpc_demo_{timestamp}.csv")
    logger = TelemetryLogger(log_path)

    print("=== Phase 2 MPC Demo ===")
    print(f"Poses: {len(DEMO_SEQUENCE)}, Duration: {duration:.1f}s, Rate: 50 Hz")
    for t, pose in DEMO_SEQUENCE:
        print(f"  t={t:5.1f}s  [{', '.join(f'{v:7.2f}' for v in pose)}]")
    print(f"Logging to: {log_path}")
    print()

    solve_times = []
    statuses = []

    def step_callback(state, target, cmd, diag):
        actual_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        record = record_from_arrays(
            time=state.time,
            ref_pose=target,
            ref_twist=np.zeros(6),
            actual_pose=actual_pose,
            actual_twist=state.platform_twist,
            cmd_extensions=cmd,
            actual_extensions=state.leg_extensions_mm,
            leg_velocities=state.leg_velocities_mmps,
            solve_time_ms=diag.get("solve_time_ms", 0.0),
            solve_status=diag.get("status", "n/a"),
            cost=diag.get("cost", 0.0),
            constraint_violation=diag.get("constraint_violation", 0.0),
        )
        logger.append(record)
        if dashboard is not None:
            dashboard.broadcast(record)
        solve_times.append(diag.get("solve_time_ms", 0.0))
        statuses.append(diag.get("status", "n/a"))

    if viewer_mode:
        import mujoco.viewer
        from viz.horizon import HorizonRenderer

        horizon = HorizonRenderer(plant.geom.init_height_mm)

        with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
            wall_start = time.monotonic()
            sim_target = 0.0

            while viewer.is_running() and sim_target < duration:
                state = plant.get_state()
                target = get_target(state.time, DEMO_SEQUENCE)
                cmd, _cmd_vel, diag = mpc.solve(state, target)
                plant.command(cmd)
                plant.step(CONTROL_DT)
                sim_target += CONTROL_DT

                step_callback(state, target, cmd, diag)

                horizon.update(mpc.predicted_poses, mpc.predicted_times)
                horizon.render(viewer)
                viewer.sync()

                elapsed = time.monotonic() - wall_start
                sleep = sim_target - elapsed
                if sleep > 0:
                    time.sleep(sleep)
    else:
        for _ in range(n_steps):
            state = plant.get_state()
            target = get_target(state.time, DEMO_SEQUENCE)
            cmd, _cmd_vel, diag = mpc.solve(state, target)
            plant.command(cmd)
            plant.step(CONTROL_DT)
            step_callback(state, target, cmd, diag)

    logger.flush()

    # Summary
    print()
    print("=== Results ===")
    st = np.array(solve_times)
    n_ok = sum(1 for s in statuses if "Solve_Succeeded" in s or "Acceptable" in s)
    n_fail = len(statuses) - n_ok
    print(f"Steps: {len(statuses)},  Converged: {n_ok},  Failed: {n_fail}")
    print(f"Solve time — mean: {np.mean(st):.1f} ms,  "
          f"p50: {np.median(st):.1f} ms,  "
          f"p95: {np.percentile(st, 95):.1f} ms,  "
          f"max: {np.max(st):.1f} ms")

    if logger.records:
        errors_mm = [r.tracking_error_mm for r in logger.records]
        errors_deg = [r.tracking_error_deg for r in logger.records]
        # Steady-state errors: last 10 records before each pose change
        print(f"Tracking error — final: {errors_mm[-1]:.3f} mm / {errors_deg[-1]:.4f}°,  "
              f"max: {max(errors_mm):.1f} mm")

    print(f"\nTelemetry saved to: {log_path}")
    print("Tip: run with --dashboard and open http://localhost:8082 for live charts")


def main():
    args = parse_args()

    dashboard = None
    if args.dashboard:
        from viz.dashboard import DashboardServer
        dashboard = DashboardServer(port=args.dashboard_port)
        dashboard.start()
        print(f"Dashboard: http://localhost:{args.dashboard_port}")

    try:
        run(viewer_mode=not args.no_viewer, dashboard=dashboard)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if dashboard is not None:
            dashboard.stop()


if __name__ == "__main__":
    main()
