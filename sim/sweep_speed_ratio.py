"""Sweep --platform-event-speed-ratio and plot tracking error, solver cost, and solver times.

Usage:
    cd sim
    python sweep_speed_ratio.py

Runs headless simulations with --cycle-time 1.2 --lateral-spacing 120
and varies --platform-event-speed-ratio from 0.0 to 5.0 in 0.1 increments.
Parses the telemetry CSV from each run and plots summary metrics.
"""

from __future__ import annotations

import csv
import os
import sys
import subprocess
import shutil
import tempfile
import numpy as np

# Range of speed ratios to test
SPEED_RATIOS = np.round(np.arange(0.0, 5.01, 0.1), 2)

# Simulation parameters
CYCLE_TIME = 1.2
HOLD_RATIO = 0.4
LATERAL_SPACING = 120.0
DURATION = 15.0  # seconds per run (enough for several cycles)

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
MAIN_PY = os.path.join(SIM_DIR, "main.py")
PYTHON = sys.executable


def run_sim(speed_ratio: float, log_dir: str) -> str | None:
    """Run one headless simulation and return the CSV path, or None on failure."""
    cmd = [
        PYTHON, MAIN_PY,
        "--no-viewer",
        "--cycle-time", str(CYCLE_TIME),
        "--hold-ratio", str(HOLD_RATIO),
        "--lateral-spacing", str(LATERAL_SPACING),
        "--platform-event-speed-ratio", str(speed_ratio),
        "--duration", str(DURATION),
        "--log-dir", log_dir,
    ]
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=120, cwd=SIM_DIR, env=env
        )
        if result.returncode != 0:
            print(f"  FAILED (rc={result.returncode}): {result.stderr[:200]}")
            return None
    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT")
        return None

    # Find the CSV written to log_dir
    csvs = sorted(
        [f for f in os.listdir(log_dir) if f.endswith(".csv")],
        key=lambda f: os.path.getmtime(os.path.join(log_dir, f)),
    )
    if not csvs:
        print(f"  No CSV found")
        return None
    return os.path.join(log_dir, csvs[-1])


def parse_csv(csv_path: str) -> dict:
    """Extract summary metrics from a telemetry CSV."""
    tracking_errors = []
    solve_times = []
    costs = []

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            tracking_errors.append(float(row["tracking_error_mm"]))
            st = float(row["solve_time_ms"])
            if st > 0:
                solve_times.append(st)
            c = float(row["cost"])
            if c > 0:
                costs.append(c)

    if not tracking_errors:
        return {}

    return {
        "mean_tracking_error_mm": np.mean(tracking_errors),
        "max_tracking_error_mm": np.max(tracking_errors),
        "p95_tracking_error_mm": np.percentile(tracking_errors, 95),
        "mean_solve_time_ms": np.mean(solve_times) if solve_times else 0.0,
        "max_solve_time_ms": np.max(solve_times) if solve_times else 0.0,
        "p95_solve_time_ms": np.percentile(solve_times, 95) if solve_times else 0.0,
        "mean_cost": np.mean(costs) if costs else 0.0,
        "max_cost": np.max(costs) if costs else 0.0,
    }


def main():
    results = {}
    tmp_base = tempfile.mkdtemp(prefix="sweep_sr_")
    print(f"Temp dir: {tmp_base}")
    print(f"Sweeping speed_ratio from {SPEED_RATIOS[0]} to {SPEED_RATIOS[-1]} "
          f"({len(SPEED_RATIOS)} points)")
    print(f"Sim params: cycle_time={CYCLE_TIME}, hold_ratio={HOLD_RATIO}, "
          f"lateral_spacing={LATERAL_SPACING}, duration={DURATION}s\n")

    for i, sr in enumerate(SPEED_RATIOS):
        print(f"[{i+1}/{len(SPEED_RATIOS)}] speed_ratio={sr:.2f} ... ", end="", flush=True)
        log_dir = os.path.join(tmp_base, f"sr_{sr:.2f}")
        os.makedirs(log_dir, exist_ok=True)

        csv_path = run_sim(sr, log_dir)
        if csv_path is None:
            print("SKIPPED")
            continue

        metrics = parse_csv(csv_path)
        if not metrics:
            print("EMPTY CSV")
            continue

        results[sr] = metrics
        print(f"track_err={metrics['mean_tracking_error_mm']:.2f}mm  "
              f"cost={metrics['mean_cost']:.1f}  "
              f"solve={metrics['mean_solve_time_ms']:.1f}ms")

    if not results:
        print("\nNo successful runs — nothing to plot.")
        return

    # ---- Plot ----
    import matplotlib.pyplot as plt

    srs = sorted(results.keys())
    mean_err = [results[s]["mean_tracking_error_mm"] for s in srs]
    max_err = [results[s]["max_tracking_error_mm"] for s in srs]
    p95_err = [results[s]["p95_tracking_error_mm"] for s in srs]

    mean_cost = [results[s]["mean_cost"] for s in srs]
    max_cost = [results[s]["max_cost"] for s in srs]

    mean_solve = [results[s]["mean_solve_time_ms"] for s in srs]
    max_solve = [results[s]["max_solve_time_ms"] for s in srs]
    p95_solve = [results[s]["p95_solve_time_ms"] for s in srs]

    fig, axes = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    fig.suptitle(
        f"Effect of platform-event-speed-ratio on MPC performance\n"
        f"(cycle={CYCLE_TIME}s, hold={HOLD_RATIO}, spacing={LATERAL_SPACING}mm, "
        f"duration={DURATION}s)",
        fontsize=13,
    )

    # ---- Tracking error ----
    ax = axes[0]
    ax.plot(srs, mean_err, "o-", label="Mean", markersize=3)
    ax.plot(srs, p95_err, "s-", label="P95", markersize=3)
    ax.plot(srs, max_err, "^-", label="Max", markersize=3, alpha=0.6)
    ax.set_ylabel("Tracking Error (mm)")
    ax.set_title("Tracking Error vs Speed Ratio")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ---- Solver cost ----
    ax = axes[1]
    ax.plot(srs, mean_cost, "o-", label="Mean", markersize=3)
    ax.plot(srs, max_cost, "s-", label="Max", markersize=3, alpha=0.6)
    ax.set_ylabel("Solver Cost")
    ax.set_title("Solver Cost vs Speed Ratio")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ---- Solve time ----
    ax = axes[2]
    ax.plot(srs, mean_solve, "o-", label="Mean", markersize=3)
    ax.plot(srs, p95_solve, "s-", label="P95", markersize=3)
    ax.plot(srs, max_solve, "^-", label="Max", markersize=3, alpha=0.6)
    ax.set_ylabel("Solve Time (ms)")
    ax.set_xlabel("Platform Event Speed Ratio")
    ax.set_title("Solver Time vs Speed Ratio")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()

    out_path = os.path.join(SIM_DIR, "sweep_speed_ratio_results.png")
    fig.savefig(out_path, dpi=150)
    print(f"\nPlot saved to: {out_path}")
    plt.show()

    # Cleanup
    shutil.rmtree(tmp_base, ignore_errors=True)


if __name__ == "__main__":
    main()
