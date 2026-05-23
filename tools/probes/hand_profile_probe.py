#!/usr/bin/env python3
"""hand_profile_probe.py — characterise the current hand throw/catch generator.

What this probe characterises
-----------------------------
The platform hand axis generates throw and catch motion profiles on the
Teensy (`ros_ws/src/jugglebot/Teensy_code/Trajectory.h`, `HandTrajGenerator`).
``sim/hand/trajectory.py`` is the Python port of that firmware. This probe
drives the *real port classes* (``HandThrowTrajectory``, ``HandCatchTrajectory``,
``HandSmoothMove``) — it is NOT a re-implementation — and quantifies the two
limitations Phase 1 of the overhaul plan needs measured:

1. **Unbounded jerk.** ``calcThrow``/``calcCatch`` build three-segment
   profiles with piecewise-CONSTANT acceleration. Acceleration *steps* at
   every segment boundary, so jerk is a Dirac spike there. This probe reports
   the exact analytic acceleration-step magnitude at each boundary AND the
   finite-difference jerk a 500 Hz controller actually sees (sample-rate
   dependent — itself proof the underlying jerk is unbounded).
2. **Backwards velocity scaling.** With stroke fixed at the effective
   ``0.315 m``, peak acceleration scales as ``~v^2`` and event duration as
   ``~1/v``. This probe fits both power laws over ``v in [0.3, 7.0] m/s``.

``HandSmoothMove`` (the already-quintic, jerk-bounded point-to-point move) is
characterised alongside as the design reference point for Phase 2.

catch_vel_ratio — port reconciled to firmware (2026-05-23)
----------------------------------------------------------
``config/hardware_config.yaml`` → ``config/generated/hardware_config.h`` sets
``CATCH_VEL_RATIO = 0.6f`` — what the Teensy firmware compiles and runs.
Phase 1 found the Python port had drifted to a hardcoded ``0.9`` (introduced
2026-03-21 in commit 6859a9c); the port was reconciled to ``0.6`` on
2026-05-23 so port and firmware now agree. The probe still reads the port's
exported value and records both side-by-side in the JSON for traceability;
the WARNING fires only if a future drift re-opens the divergence.

Outputs (all written to temp/probes/, gitignored)
--------------------------------------------------
- ``hand_profile_metrics.csv``   — per-velocity throw/catch metrics sweep
- ``hand_profile_summary.json``  — headline numbers + power-law fit coefficients
- ``hand_throw_profiles.png``    — pos/vel/accel/jerk vs time, 5 velocities
- ``hand_catch_profiles.png``    — pos/vel/accel/jerk vs time, 5 velocities
- ``hand_smooth_move_reference.png`` — the jerk-bounded reference profile

Run:  python tools/probes/hand_profile_probe.py

Plan:    plans/active/hand-trajectory-generator-overhaul.md  (Phase 1)
Logbook: logbook/2026-05-22-hand-generator-phase1-characterisation.md
Tests:   tests/sim/test_hand_profile_probe.py  (asserts this probe's numbers)
"""

from __future__ import annotations

import csv
import json
import os
import sys

# --- self-contained path setup (probe convention: runnable from repo root) --
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
for _p in (_REPO_ROOT, os.path.join(_REPO_ROOT, "sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from hand.trajectory import (  # noqa: E402  (after sys.path setup)
    CATCH_VEL_RATIO,
    HandCatchTrajectory,
    HandSmoothMove,
    HandThrowTrajectory,
    MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2,
    STROKE_MARGIN_MM,
    _LINEAR_GAIN,
    _TOTAL_STROKE_MM,
)

# ---------------------------------------------------------------------------
# Pinned probe parameters — changing these changes the recorded measurements.
# ---------------------------------------------------------------------------
SAMPLE_RATE_HZ = 500.0                  # matches the Teensy 500 Hz sample rate
DT = 1.0 / SAMPLE_RATE_HZ

SWEEP_V_MIN = 0.3                       # m/s — plan's event-velocity range
SWEEP_V_MAX = 7.0
SWEEP_V_STEP = 0.1                      # → 68 sweep points
PLOT_VELOCITIES = (0.5, 1.0, 3.0, 5.0, 7.0)   # m/s — full time-series plots

# The firmware-authoritative value (config/hardware_config.yaml → .h).
FIRMWARE_CATCH_VEL_RATIO = 0.6

OUTPUT_DIR = os.path.join(_REPO_ROOT, "temp", "probes")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def sweep_velocities() -> list[float]:
    """Deterministic velocity sweep grid (rounded to avoid float drift)."""
    n = int(round((SWEEP_V_MAX - SWEEP_V_MIN) / SWEEP_V_STEP)) + 1
    return [round(SWEEP_V_MIN + i * SWEEP_V_STEP, 4) for i in range(n)]


def _finite_diff(values: list[float], dt: float) -> list[float]:
    """Forward finite difference: returns len(values) - 1 samples."""
    return [(values[i + 1] - values[i]) / dt for i in range(len(values) - 1)]


def _sample_positions(traj, t_start: float, t_end: float) -> tuple[list[float], list[float]]:
    """Sample traj.sample() on the pinned 500 Hz grid → (times_s, pos_mm).

    Drives the real port object — traj.sample() is the production code path.
    """
    n = int(round((t_end - t_start) / DT))
    times = [t_start + i * DT for i in range(n + 1)]
    pos_mm = [traj.sample(t) for t in times]
    return times, pos_mm


def _profile_derivatives(pos_mm: list[float]) -> dict:
    """Finite-difference pos→vel→accel→jerk and return peak magnitudes (SI).

    Positions are in mm; results are converted to m, m/s, m/s^2, m/s^3.
    """
    pos_m = [p / 1000.0 for p in pos_mm]
    vel = _finite_diff(pos_m, DT)
    acc = _finite_diff(vel, DT)
    jrk = _finite_diff(acc, DT)
    return {
        "vel_mps": vel,
        "acc_mps2": acc,
        "jerk_mps3": jrk,
        "fd_peak_vel_mps": max((abs(x) for x in vel), default=0.0),
        "fd_peak_accel_mps2": max((abs(x) for x in acc), default=0.0),
        "fd_peak_jerk_mps3": max((abs(x) for x in jrk), default=0.0),
    }


def _accel_steps(seg_accels: list[float]) -> list[dict]:
    """Signed acceleration steps for a piecewise-constant-accel profile.

    seg_accels is the per-segment constant acceleration; the profile starts
    and ends at rest (accel 0). The boundaries are: rest→seg0, seg(i)→seg(i+1),
    seg(last)→rest.
    """
    levels = [0.0] + list(seg_accels) + [0.0]
    labels = ["rest→accel"] + [
        f"seg{i}→seg{i + 1}" for i in range(len(seg_accels) - 1)
    ] + ["decel→rest"]
    return [
        {"boundary": labels[i], "step_mps2": levels[i + 1] - levels[i]}
        for i in range(len(levels) - 1)
    ]


# ---------------------------------------------------------------------------
# Characterisation — pure functions (unit tests call these directly)
# ---------------------------------------------------------------------------
def characterise_throw(v: float) -> dict:
    """Characterise one throw profile at event velocity *v* (m/s)."""
    traj = HandThrowTrajectory(v)
    t_start, t_end = traj.start_time, traj.end_time
    _, pos_mm = _sample_positions(traj, t_start, t_end)
    deriv = _profile_derivatives(pos_mm)

    # Analytic per-segment accelerations (read from the real port object).
    seg_accels = [traj._throwA, 0.0, traj._throwD]
    steps = _accel_steps(seg_accels)
    peak_accel = max(abs(traj._throwA), abs(traj._throwD))

    return {
        "v_mps": v,
        "duration_s": t_end - t_start,
        # Motion duration = accel + vel-hold + decel (excludes any end hold).
        # This is the quantity that obeys the pure 1/v law.
        "motion_duration_s": traj._t_acc + traj._t_vel + traj._t_dec,
        "t_acc_s": traj._t_acc,
        "t_vel_s": traj._t_vel,
        "t_dec_s": traj._t_dec,
        "throwA_mps2": traj._throwA,
        "throwD_mps2": traj._throwD,
        "analytic_peak_accel_mps2": peak_accel,
        "analytic_peak_accel_revps2": peak_accel * _LINEAR_GAIN,
        "accel_steps": steps,
        "max_accel_step_mps2": max(abs(s["step_mps2"]) for s in steps),
        "fd_peak_accel_mps2": deriv["fd_peak_accel_mps2"],
        "fd_peak_jerk_mps3": deriv["fd_peak_jerk_mps3"],
        "release_speed_mps": traj.release_speed_mps,
        "release_pos_mm": traj.release_pos_mm,
    }


def characterise_catch(v: float) -> dict:
    """Characterise one catch profile at event velocity *v* (m/s)."""
    traj = HandCatchTrajectory(v)
    t_start, t_end = traj.start_time, traj.end_time
    _, pos_mm = _sample_positions(traj, t_start, t_end)
    deriv = _profile_derivatives(pos_mm)

    seg_accels = [traj._catchA, 0.0, traj._catchD]
    steps = _accel_steps(seg_accels)
    peak_accel = max(abs(traj._catchA), abs(traj._catchD))

    return {
        "v_mps": v,
        # Catch total duration carries a fixed END_PROFILE_HOLD tail, so it
        # is 1/v + const; motion_duration_s is the pure-1/v stroke part.
        "duration_s": t_end - t_start,
        "motion_duration_s": traj._t_acc + traj._t_vel + traj._t_dec,
        "t_acc_s": traj._t_acc,
        "t_vel_s": traj._t_vel,
        "t_dec_s": traj._t_dec,
        "catchA_mps2": traj._catchA,
        "catchD_mps2": traj._catchD,
        "catch_velocity_mps": traj.catch_velocity_mps,
        "analytic_peak_accel_mps2": peak_accel,
        "analytic_peak_accel_revps2": peak_accel * _LINEAR_GAIN,
        "accel_steps": steps,
        "max_accel_step_mps2": max(abs(s["step_mps2"]) for s in steps),
        "fd_peak_accel_mps2": deriv["fd_peak_accel_mps2"],
        "fd_peak_jerk_mps3": deriv["fd_peak_jerk_mps3"],
    }


def characterise_smooth_move() -> dict:
    """Characterise HandSmoothMove over the full effective stroke (reference).

    The quintic S-curve is jerk-bounded by construction; this is the design
    target Phase 2's jerk-limited throw/catch family is measured against.
    """
    start = STROKE_MARGIN_MM
    end = STROKE_MARGIN_MM + _TOTAL_STROKE_MM
    move = HandSmoothMove(start, end)
    _, pos_mm = _sample_positions(move, 0.0, move.duration)
    deriv = _profile_derivatives(pos_mm)
    return {
        "start_pos_mm": start,
        "end_pos_mm": end,
        "duration_s": move.duration,
        "fd_peak_accel_mps2": deriv["fd_peak_accel_mps2"],
        "fd_peak_accel_revps2": deriv["fd_peak_accel_mps2"] * _LINEAR_GAIN,
        "fd_peak_jerk_mps3": deriv["fd_peak_jerk_mps3"],
        "accel_limit_revps2": MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2,
    }


def fit_power_law(xs: list[float], ys: list[float], exponent: float) -> dict:
    """Fit y = k * x**exponent by least-squares on the coefficient k.

    For an exact power law the residual is ~0; a non-zero max relative
    residual would indicate the law does not hold.
    """
    preds = [x ** exponent for x in xs]
    k = sum(p * y for p, y in zip(preds, ys)) / sum(p * p for p in preds)
    residuals = [abs(k * p - y) / abs(y) for p, y in zip(preds, ys) if y != 0.0]
    return {"coefficient": k, "exponent": exponent,
            "max_rel_residual": max(residuals, default=0.0)}


# ---------------------------------------------------------------------------
# Probe assembly
# ---------------------------------------------------------------------------
def run_probe() -> dict:
    """Run the full characterisation sweep and return the summary dict.

    Pure computation — no file IO. main() handles writing outputs.
    """
    velocities = sweep_velocities()
    throws = [characterise_throw(v) for v in velocities]
    catches = [characterise_catch(v) for v in velocities]
    smooth = characterise_smooth_move()

    # Power-law fits use motion_duration_s (pure 1/v); total catch duration
    # carries a fixed END_PROFILE_HOLD tail and would not fit a pure law.
    t_peak = fit_power_law(velocities, [t["analytic_peak_accel_mps2"] for t in throws], 2.0)
    t_dur = fit_power_law(velocities, [t["motion_duration_s"] for t in throws], -1.0)
    c_peak = fit_power_law(velocities, [c["analytic_peak_accel_mps2"] for c in catches], 2.0)
    c_dur = fit_power_law(velocities, [c["motion_duration_s"] for c in catches], -1.0)

    throw_vmax = throws[-1]
    catch_vmax = catches[-1]

    summary = {
        "probe": "hand_profile_probe",
        "subject": "sim/hand/trajectory.py (port of Teensy Trajectory.h)",
        "sample_rate_hz": SAMPLE_RATE_HZ,
        "catch_vel_ratio": {
            "port_value": CATCH_VEL_RATIO,
            "firmware_value": FIRMWARE_CATCH_VEL_RATIO,
            "divergent": CATCH_VEL_RATIO != FIRMWARE_CATCH_VEL_RATIO,
            "note": (
                "Port matches firmware (config/hardware_config.h)."
                if CATCH_VEL_RATIO == FIRMWARE_CATCH_VEL_RATIO else
                "Port DIVERGES from firmware — firmware is authoritative. "
                "Catch analytic accel scales with ratio^2."
            ),
        },
        "velocity_sweep": {
            "min_mps": SWEEP_V_MIN, "max_mps": SWEEP_V_MAX,
            "step_mps": SWEEP_V_STEP, "n_points": len(velocities),
        },
        "throw": {
            "peak_accel_v2_fit": t_peak,
            "motion_duration_inv_v_fit":t_dur,
            "accel_steps_at_v_min": throws[0]["accel_steps"],
            "accel_steps_at_v_max": throw_vmax["accel_steps"],
            "max_accel_step_mps2_at_v_max": throw_vmax["max_accel_step_mps2"],
            "peak_accel_revps2_at_v_max": throw_vmax["analytic_peak_accel_revps2"],
            "fd_peak_jerk_mps3_at_v_max": throw_vmax["fd_peak_jerk_mps3"],
            "peak_accel_over_smooth_limit_at_v_max": (
                throw_vmax["analytic_peak_accel_revps2"]
                / MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2),
        },
        "catch": {
            "peak_accel_v2_fit": c_peak,
            "motion_duration_inv_v_fit":c_dur,
            "accel_steps_at_v_max": catch_vmax["accel_steps"],
            "max_accel_step_mps2_at_v_max": catch_vmax["max_accel_step_mps2"],
            "peak_accel_revps2_at_v_max": catch_vmax["analytic_peak_accel_revps2"],
            "fd_peak_jerk_mps3_at_v_max": catch_vmax["fd_peak_jerk_mps3"],
        },
        "smooth_move_reference": smooth,
        "_per_velocity_throw": throws,
        "_per_velocity_catch": catches,
    }
    return summary


def _write_csv(path: str, throws: list[dict], catches: list[dict]) -> None:
    cols = ["v_mps", "throw_motion_duration_s", "throw_peak_accel_mps2",
            "throw_peak_accel_revps2", "throw_max_accel_step_mps2",
            "throw_fd_peak_jerk_mps3", "catch_motion_duration_s",
            "catch_total_duration_s", "catch_peak_accel_mps2",
            "catch_peak_accel_revps2", "catch_max_accel_step_mps2",
            "catch_fd_peak_jerk_mps3"]
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for t, c in zip(throws, catches):
            w.writerow([
                t["v_mps"], t["motion_duration_s"], t["analytic_peak_accel_mps2"],
                t["analytic_peak_accel_revps2"], t["max_accel_step_mps2"],
                t["fd_peak_jerk_mps3"], c["motion_duration_s"],
                c["duration_s"], c["analytic_peak_accel_mps2"],
                c["analytic_peak_accel_revps2"], c["max_accel_step_mps2"],
                c["fd_peak_jerk_mps3"],
            ])


def _plot_profiles(path: str, traj_factory, title: str) -> None:
    """Plot pos/vel/accel/jerk vs time for the PLOT_VELOCITIES set."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 1, figsize=(9, 11), sharex=False)
    for v in PLOT_VELOCITIES:
        traj = traj_factory(v)
        times, pos_mm = _sample_positions(traj, traj.start_time, traj.end_time)
        pos_m = [p / 1000.0 for p in pos_mm]
        vel = _finite_diff(pos_m, DT)
        acc = _finite_diff(vel, DT)
        jrk = _finite_diff(acc, DT)
        label = f"v={v:.1f} m/s"
        axes[0].plot(times, pos_mm, label=label)
        axes[1].plot(times[:-1], vel, label=label)
        axes[2].plot(times[:-2], acc, label=label)
        axes[3].plot(times[:-3], jrk, label=label)
    for ax, ylabel in zip(axes, ["position [mm]", "velocity [m/s]",
                                 "acceleration [m/s²]", "jerk [m/s³]"]):
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    axes[3].set_xlabel("time [s]")
    axes[0].set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110, metadata={"Software": "hand_profile_probe"})
    plt.close(fig)


def _plot_smooth_move(path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    move = HandSmoothMove(STROKE_MARGIN_MM, STROKE_MARGIN_MM + _TOTAL_STROKE_MM)
    times, pos_mm = _sample_positions(move, 0.0, move.duration)
    pos_m = [p / 1000.0 for p in pos_mm]
    vel = _finite_diff(pos_m, DT)
    acc = _finite_diff(vel, DT)
    jrk = _finite_diff(acc, DT)
    fig, axes = plt.subplots(4, 1, figsize=(9, 11))
    axes[0].plot(times, pos_mm)
    axes[1].plot(times[:-1], vel)
    axes[2].plot(times[:-2], acc)
    axes[3].plot(times[:-3], jrk)
    for ax, ylabel in zip(axes, ["position [mm]", "velocity [m/s]",
                                 "acceleration [m/s²]", "jerk [m/s³]"]):
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
    axes[3].set_xlabel("time [s]")
    axes[0].set_title("HandSmoothMove — jerk-bounded reference (full stroke)")
    fig.tight_layout()
    fig.savefig(path, dpi=110, metadata={"Software": "hand_profile_probe"})
    plt.close(fig)


def main() -> int:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    summary = run_probe()
    throws = summary.pop("_per_velocity_throw")
    catches = summary.pop("_per_velocity_catch")

    csv_path = os.path.join(OUTPUT_DIR, "hand_profile_metrics.csv")
    json_path = os.path.join(OUTPUT_DIR, "hand_profile_summary.json")
    _write_csv(csv_path, throws, catches)
    with open(json_path, "w") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
        fh.write("\n")
    _plot_profiles(os.path.join(OUTPUT_DIR, "hand_throw_profiles.png"),
                   HandThrowTrajectory, "Current throw generator — buildThrow()")
    _plot_profiles(os.path.join(OUTPUT_DIR, "hand_catch_profiles.png"),
                   HandCatchTrajectory, "Current catch generator — buildCatch()")
    _plot_smooth_move(os.path.join(OUTPUT_DIR, "hand_smooth_move_reference.png"))

    cvr = summary["catch_vel_ratio"]
    th, ca, sm = summary["throw"], summary["catch"], summary["smooth_move_reference"]
    print("=" * 70)
    print("hand_profile_probe — current hand generator characterisation")
    print("=" * 70)
    if cvr["divergent"]:
        print(f"  WARNING: catch_vel_ratio divergence — port={cvr['port_value']} "
              f"vs firmware={cvr['firmware_value']} (firmware is authoritative).")
        print("           Catch accel/jerk below are PORT values; firmware "
              f"values scale by ({cvr['firmware_value']}/{cvr['port_value']})^2.")
    print(f"  Throw  peak accel = {th['peak_accel_v2_fit']['coefficient']:.4f} * v^2 "
          f"m/s² (max rel residual {th['peak_accel_v2_fit']['max_rel_residual']:.2e})")
    print(f"  Throw  duration   = {th['motion_duration_inv_v_fit']['coefficient']:.4f} / v "
          f"s   (max rel residual {th['motion_duration_inv_v_fit']['max_rel_residual']:.2e})")
    print(f"  Throw  @v=7: peak accel {th['peak_accel_revps2_at_v_max']:.0f} rev/s² "
          f"= {th['peak_accel_over_smooth_limit_at_v_max']:.0f}x the smooth-move "
          f"limit; FD jerk {th['fd_peak_jerk_mps3_at_v_max']:.0f} m/s³")
    print(f"  Catch  peak accel = {ca['peak_accel_v2_fit']['coefficient']:.4f} * v^2 "
          f"m/s² (max rel residual {ca['peak_accel_v2_fit']['max_rel_residual']:.2e})")
    print(f"  Catch  motion dur = {ca['motion_duration_inv_v_fit']['coefficient']:.4f} / v s "
          f"(+ fixed END_PROFILE_HOLD tail)")
    print(f"  SmoothMove ref: peak accel {sm['fd_peak_accel_revps2']:.1f} rev/s² "
          f"(limit {sm['accel_limit_revps2']:.0f}), FD jerk {sm['fd_peak_jerk_mps3']:.1f} m/s³")
    print(f"  Outputs → {OUTPUT_DIR}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
