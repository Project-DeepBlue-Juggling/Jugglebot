#!/usr/bin/env python3
"""Standalone trajectory test harness for Jugglebot Phase 4.

Evaluates quintic trajectories in a tight loop (~500 Hz) and sends
set_input_pos + vel_ff + torque_ff directly to all 6 ODrive leg axes
via python-can.  Bypasses ROS2 entirely.

Phase 4 hardware tests (free platform, <=25% speed):
  T1. Small move from home (10mm Z, 1.0s, 25% speed)
  T2. Graduated distance/tilt (20mm, 5 deg, combined, 40mm)
  T3. Multi-pose sequence across workspace
  T4. Speed scale verification (50% vs 25%)

Prerequisites:
  - All Phase 3 Stage C tests must PASS
  - Robot fully assembled, platform free-standing
  - Legs must be homed (use --home flag)

Safety:
  - Mandatory check_feasibility() before every trajectory execution
  - Conservative current limit (50% of rated = 10A)
  - All tests send IDLE to all 6 axes on completion, error, or Ctrl-C
  - Heartbeat + error monitoring every cycle inside execution loop
  - POSITION/PASSTHROUGH mode (quintic provides the smoothing)
  - Interactive confirmation before each test

Usage:
  python tools/trajectory_test.py --home --test all
  python tools/trajectory_test.py --test small_move
  python tools/trajectory_test.py --dry-run --preview
  python tools/trajectory_test.py --test all --preview

Requirements:
  pip install python-can numpy
  (optional for --preview: matplotlib)
"""

from __future__ import annotations

import argparse
import datetime
import json
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_SCRIPT_DIR)
_CONFIG_DIR = os.path.join(_PROJECT_ROOT, 'config', 'generated')
_ROS_PKG_DIR = os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'jugglebot')

sys.path.insert(0, _CONFIG_DIR)
sys.path.insert(0, _ROS_PKG_DIR)
sys.path.insert(0, _SCRIPT_DIR)

import hardware_config as hw  # noqa: E402

# Import CAN harness and helpers from the existing free_platform_test
from free_platform_test import (  # noqa: E402
    PlatformTestHarness,
    LEG_AXES,
    NUM_LEGS,
    MM_TO_REV,
    LEG_VEL_FF_SCALE,
    LEG_TOR_FF_SCALE,
    BASELINE_POS_GAIN,
    BASELINE_VEL_GAIN,
    BASELINE_VEL_INT_GAIN,
    SAFE_CURRENT_LIMIT_A,
    encode_set_input_pos,
    pose_to_raw_positions,
    interactive_pause,
    error_names,
)

# Motion modules
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.ik_solver import rotvec_to_rot_matrix
from jugglebot.motion.trajectory import (
    create_trajectory,
    evaluate,
    check_feasibility,
    cartesian_to_motor_commands,
    FeasibilityResult,
)
from jugglebot.motion.conversions import extensions_mm_to_revs

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TARGET_LOOP_HZ = 500
TARGET_DT_S = 1.0 / TARGET_LOOP_HZ
HOLD_AFTER_S = 2.0         # hold at end pose after trajectory completes
SETTLE_CHECK_S = 1.0       # monitor stability during final second of hold

# Pass criteria
MAX_TRACKING_ERROR_MM = 1.5     # during trajectory motion
MAX_HOLD_DEVIATION_MM = 0.5     # during post-trajectory hold
MAX_SPEED_RATIO_ERROR = 0.15    # T4: duration ratio tolerance

# Default speed scale for all tests
DEFAULT_SPEED_SCALE = 0.25


# ---------------------------------------------------------------------------
# Data logging
# ---------------------------------------------------------------------------

@dataclass
class TrajectoryLog:
    """Records per-cycle data and run metadata during trajectory execution."""
    timestamps: list = field(default_factory=list)
    commanded_pos_rev: list = field(default_factory=list)
    commanded_vel_rps: list = field(default_factory=list)
    commanded_torque_Nm: list = field(default_factory=list)
    actual_pos_rev: list = field(default_factory=list)
    actual_vel_rps: list = field(default_factory=list)
    poses: list = field(default_factory=list)
    loop_dt_s: list = field(default_factory=list)
    metadata: dict = field(default_factory=dict)


def build_run_metadata(
    test_name: str,
    speed_scale: float,
    geom: StewartGeometry,
    params: DynamicsParams,
    *,
    tracking_threshold_mm: float | None = None,
    move_duration_s: float | None = None,
    extra: dict | None = None,
) -> dict:
    """Build a metadata dict capturing the full test configuration.

    Call this before execution and assign to ``log.metadata``.

    Parameters
    ----------
    test_name : str — e.g. 'JT1', 'DT2', 'T1'
    speed_scale : float
    geom : StewartGeometry
    params : DynamicsParams
    tracking_threshold_mm : float or None
    move_duration_s : float or None — base move duration before scaling
    extra : dict or None — test-specific parameters to merge in
    """
    from jugglebot.motion.workspace import (
        LEG_HARD_MARGIN_MM,
        LEG_SOFT_MARGIN_MM,
        COND_SOFT_FACTOR,
        COND_HARD_FACTOR,
        compute_condition_number,
    )

    # Feasibility defaults (same as check_feasibility)
    cond_home = compute_condition_number(np.zeros(3), np.eye(3), geom)

    # Git revision
    git_rev = _get_git_revision()

    meta = {
        # Environment
        'timestamp': datetime.datetime.now().isoformat(),
        'git_revision': git_rev,

        # Test parameters
        'test_name': test_name,
        'speed_scale': speed_scale,
        'tracking_threshold_mm': tracking_threshold_mm,
        'move_duration_s': move_duration_s,
        'target_loop_hz': TARGET_LOOP_HZ,

        # ODrive gains and limits
        'odrive': {
            'pos_gain': BASELINE_POS_GAIN,
            'vel_gain': BASELINE_VEL_GAIN,
            'vel_int_gain': BASELINE_VEL_INT_GAIN,
            'current_limit_A': float(SAFE_CURRENT_LIMIT_A),
            'vel_limit_rps': float(hw.ODRIVE_LEG_VEL_LIMIT_RPS),
        },

        # Kinematic limits (feasibility checker defaults)
        'kinematic_limits': {
            'vel_limit_rps': float(hw.ODRIVE_TRAP_VEL_LIMIT_RPS),
            'accel_limit_rps2': float(hw.ODRIVE_TRAP_ACC_LIMIT_RPS2),
            'jerk_trans_limit': 50_000.0,
            'jerk_rot_limit': 400.0,
            'stroke_min_mm': float(LEG_HARD_MARGIN_MM),
            'stroke_max_mm': float(geom.leg_stroke_mm - LEG_HARD_MARGIN_MM),
            'condition_limit': 2.0 * cond_home,
            'condition_home': cond_home,
        },

        # Workspace margins
        'workspace': {
            'leg_stroke_mm': float(geom.leg_stroke_mm),
            'hard_margin_mm': float(LEG_HARD_MARGIN_MM),
            'soft_margin_mm': float(LEG_SOFT_MARGIN_MM),
            'cond_soft_factor': COND_SOFT_FACTOR,
            'cond_hard_factor': COND_HARD_FACTOR,
        },

        # Geometry snapshot
        'geometry': {
            'home_z_mm': float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM),
            'mm_to_rev': [float(v) for v in hw.GEOM_MM_TO_REV],
        },

        # Dynamics
        'dynamics': {
            'platform_mass_kg': float(params.mass_kg),
            'com_offset_mm': [float(v) for v in params.com_offset_mm],
            'gravity_mps2': float(params.gravity_mps2),
        },
    }

    if extra:
        meta['test_params'] = extra

    return meta


def _get_git_revision() -> str:
    """Return the current git short hash + dirty flag, or 'unknown'."""
    try:
        rev = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            stderr=subprocess.DEVNULL, timeout=5,
        ).decode().strip()
        # Check for uncommitted changes
        status = subprocess.check_output(
            ['git', 'status', '--porcelain'],
            stderr=subprocess.DEVNULL, timeout=5,
        ).decode().strip()
        if status:
            rev += '-dirty'
        return rev
    except Exception:
        return 'unknown'


def save_log(log: TrajectoryLog, filepath: str | Path) -> None:
    """Save a TrajectoryLog (metadata + sample data) to a JSON file.

    Sample arrays are stored as nested lists for portability.
    """
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)

    data = {
        'metadata': log.metadata,
        'samples': {
            'timestamps': log.timestamps,
            'commanded_pos_rev': [a.tolist() if hasattr(a, 'tolist') else a
                                  for a in log.commanded_pos_rev],
            'commanded_vel_rps': [a.tolist() if hasattr(a, 'tolist') else a
                                  for a in log.commanded_vel_rps],
            'commanded_torque_Nm': [a.tolist() if hasattr(a, 'tolist') else a
                                    for a in log.commanded_torque_Nm],
            'actual_pos_rev': [a.tolist() if hasattr(a, 'tolist') else a
                               for a in log.actual_pos_rev],
            'actual_vel_rps': [a.tolist() if hasattr(a, 'tolist') else a
                               for a in log.actual_vel_rps],
            'poses': [a.tolist() if hasattr(a, 'tolist') else a
                      for a in log.poses],
            'loop_dt_s': log.loop_dt_s,
        },
    }

    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"  Log saved: {filepath} ({len(log.timestamps)} samples)")


def analyze_log(log: TrajectoryLog) -> dict:
    """Compute tracking metrics from a trajectory execution log.

    Returns dict with:
      max_error_mm     : worst-case per-leg tracking error (mm)
      rms_error_mm     : RMS tracking error across all legs and samples
      per_leg_max_mm   : (6,) array of per-leg max errors
      loop_mean_ms     : mean loop dt in ms
      loop_p99_ms      : 99th percentile loop dt in ms
      loop_max_ms      : max loop dt in ms
      n_samples        : number of logged samples
      worst_sample_idx : index of the sample with worst tracking error
      worst_sample_t   : timestamp (s) of the worst tracking sample
      worst_sample_leg : leg axis with worst tracking error at that sample
      worst_dt_idx     : index of the sample with worst loop dt
      worst_dt_t       : timestamp (s) of the worst loop-dt sample
      worst_dt_ms      : the worst loop dt value (ms)
    """
    if not log.timestamps:
        return {'max_error_mm': float('inf'), 'rms_error_mm': float('inf'),
                'per_leg_max_mm': np.full(6, float('inf')),
                'loop_mean_ms': 0, 'loop_p99_ms': 0, 'loop_max_ms': 0,
                'n_samples': 0,
                'worst_sample_idx': -1, 'worst_sample_t': 0,
                'worst_sample_leg': -1,
                'worst_dt_idx': -1, 'worst_dt_t': 0, 'worst_dt_ms': 0}

    cmd = np.array(log.commanded_pos_rev)   # (N, 6)
    act = np.array(log.actual_pos_rev)      # (N, 6)
    timestamps = np.array(log.timestamps)

    # Tracking error in mm: convert rev error to mm per leg
    err_rev = np.abs(cmd - act)
    err_mm = err_rev / MM_TO_REV[np.newaxis, :]  # broadcast (N, 6)

    per_leg_max = np.max(err_mm, axis=0)          # (6,)
    max_error = float(np.max(per_leg_max))
    rms_error = float(np.sqrt(np.mean(err_mm ** 2)))

    # Identify worst tracking sample
    worst_flat = int(np.argmax(err_mm))
    worst_sample_idx = worst_flat // 6
    worst_sample_leg = worst_flat % 6
    worst_sample_t = float(timestamps[worst_sample_idx])

    dts = np.array(log.loop_dt_s) * 1000  # ms
    worst_dt_idx = int(np.argmax(dts)) if len(dts) > 0 else -1
    worst_dt_t = float(timestamps[worst_dt_idx]) if worst_dt_idx >= 0 else 0
    worst_dt_ms = float(dts[worst_dt_idx]) if worst_dt_idx >= 0 else 0

    return {
        'max_error_mm': max_error,
        'rms_error_mm': rms_error,
        'per_leg_max_mm': per_leg_max,
        'loop_mean_ms': float(np.mean(dts)) if len(dts) > 0 else 0,
        'loop_p99_ms': float(np.percentile(dts, 99)) if len(dts) > 0 else 0,
        'loop_max_ms': float(np.max(dts)) if len(dts) > 0 else 0,
        'n_samples': len(log.timestamps),
        'worst_sample_idx': worst_sample_idx,
        'worst_sample_t': worst_sample_t,
        'worst_sample_leg': worst_sample_leg,
        'worst_dt_idx': worst_dt_idx,
        'worst_dt_t': worst_dt_t,
        'worst_dt_ms': worst_dt_ms,
    }


def print_analysis(label: str, analysis: dict):
    """Pretty-print a trajectory analysis summary."""
    print(f"\n  --- {label} ---")
    print(f"    Samples:        {analysis['n_samples']}")
    print(f"    Max error:      {analysis['max_error_mm']:.3f} mm")
    print(f"    RMS error:      {analysis['rms_error_mm']:.3f} mm")
    print(f"    Per-leg max:    [{', '.join(f'{e:.3f}' for e in analysis['per_leg_max_mm'])}] mm")
    print(f"    Loop timing:    mean={analysis['loop_mean_ms']:.2f} ms  "
          f"p99={analysis['loop_p99_ms']:.2f} ms  "
          f"max={analysis['loop_max_ms']:.2f} ms")
    # Diagnostic: where the worst tracking error and worst loop spike occurred
    w_idx = analysis.get('worst_sample_idx', -1)
    if w_idx >= 0:
        print(f"    Worst error:    sample {w_idx} at t={analysis['worst_sample_t']:.3f}s, "
              f"leg {analysis['worst_sample_leg']}")
    dt_idx = analysis.get('worst_dt_idx', -1)
    if dt_idx >= 0:
        print(f"    Worst dt:       sample {dt_idx} at t={analysis['worst_dt_t']:.3f}s, "
              f"dt={analysis['worst_dt_ms']:.2f} ms")


# ---------------------------------------------------------------------------
# Feasibility display
# ---------------------------------------------------------------------------

def print_feasibility(label: str, result: FeasibilityResult):
    """Print a feasibility check summary."""
    status = 'PASS' if result.feasible else 'FAIL'
    print(f"  Pre-flight [{status}]: {label}")
    print(f"    Peak vel:    {np.max(result.peak_leg_vel_rps):.3f} rev/s  "
          f"(limit: {hw.ODRIVE_TRAP_VEL_LIMIT_RPS})")
    print(f"    Peak accel:  {np.max(result.peak_leg_accel_rps2):.3f} rev/s^2  "
          f"(limit: {hw.ODRIVE_TRAP_ACC_LIMIT_RPS2})")
    print(f"    Peak cond:   {result.peak_condition_number:.0f}")
    print(f"    Stroke:      [{np.min(result.min_extension_mm):.1f}, "
          f"{np.max(result.max_extension_mm):.1f}] mm  "
          f"(range: 0-{hw.GEOM_LEG_STROKE_MM})")
    if not result.feasible:
        for v in result.violations:
            print(f"    VIOLATION: {v}")


# ---------------------------------------------------------------------------
# Trajectory helpers
# ---------------------------------------------------------------------------

def make_rest_to_rest(end_pose, duration, speed_scale=DEFAULT_SPEED_SCALE,
                      start_pose=None):
    """Create a rest-to-rest quintic trajectory.

    Parameters
    ----------
    end_pose : (6,) array — [x, y, z, rx, ry, rz] in mm, rad
    duration : float — base duration in seconds (before speed scaling)
    speed_scale : float — 0 < scale <= 1
    start_pose : (6,) array or None — defaults to home (zeros)
    """
    zeros6 = np.zeros(6)
    sp = np.asarray(start_pose, dtype=float) if start_pose is not None \
        else zeros6.copy()
    ep = np.asarray(end_pose, dtype=float)
    return create_trajectory(
        start_pose=sp, start_twist=zeros6, start_accel=zeros6,
        end_pose=ep, end_twist=zeros6, end_accel=zeros6,
        duration=duration, t_start=0.0, speed_scale=speed_scale)


# ---------------------------------------------------------------------------
# Core: trajectory execution loop
# ---------------------------------------------------------------------------

def execute_trajectory(harness: PlatformTestHarness,
                       traj,
                       geom: StewartGeometry,
                       params: DynamicsParams,
                       hold_s: float = HOLD_AFTER_S) -> TrajectoryLog:
    """Execute a trajectory by evaluating it in a tight loop and sending
    set_input_pos commands directly to all 6 ODrive axes via CAN.

    The loop targets ~500 Hz.  Each cycle:
      1. Evaluate trajectory at current time -> (pose, twist, accel)
      2. Convert to motor commands -> (pos_rev, vel_ff_rps, torque_ff_Nm)
      3. Send set_input_pos to all 6 axes (with vel_ff and torque_ff as int16)
      4. Poll CAN bus for encoder feedback
      5. Log commanded vs actual positions
      6. Sleep for remainder of cycle

    Parameters
    ----------
    harness : PlatformTestHarness — connected CAN interface
    traj : QuinticTrajectory — trajectory to execute (t_start=0.0)
    geom : StewartGeometry
    params : DynamicsParams
    hold_s : float — seconds to hold at end pose after trajectory completes

    Returns
    -------
    TrajectoryLog — per-cycle recording of commanded/actual state
    """
    log = TrajectoryLog()
    total_duration = traj.duration + hold_s

    if sys.platform == 'win32':
        print("  WARNING: Windows timer limits loop to ~60 Hz. "
              "Run on Jetson for 500 Hz.")

    t_loop_start = time.perf_counter()
    t_prev = t_loop_start

    while True:
        t_now = time.perf_counter()
        t_elapsed = t_now - t_loop_start
        dt = t_now - t_prev
        t_prev = t_now

        if t_elapsed > total_duration:
            break

        # 1. Evaluate trajectory (clamps beyond duration to end pose)
        pose, twist, accel = evaluate(traj, t_elapsed)

        # 2. Convert to motor commands
        pos_rev, vel_ff_rps, torque_ff_Nm = cartesian_to_motor_commands(
            pose, twist, accel, geom, params, feedforward_enabled=True)

        # 3. Send to all 6 axes
        for i, axis_id in enumerate(LEG_AXES):
            # Sign conventions for direct CAN (matching can_node.py):
            # All three fields are negated for legs — ODrive negative = extension
            raw_pos = -pos_rev[i]
            vel_int = int(round(-vel_ff_rps[i] * LEG_VEL_FF_SCALE))
            tor_int = int(round(-torque_ff_Nm[i] * LEG_TOR_FF_SCALE))
            vel_int = max(-32767, min(32767, vel_int))
            tor_int = max(-32767, min(32767, tor_int))
            harness.send_no_delay(encode_set_input_pos(
                axis_id, raw_pos, vel_int, tor_int))

        # 4. Poll for encoder feedback (non-blocking)
        harness._poll(timeout=0)

        # 5. Log data
        actual_pos = np.array([harness.states[a].pos_rev for a in LEG_AXES])
        actual_vel = np.array([harness.states[a].vel_rps for a in LEG_AXES])

        log.timestamps.append(t_elapsed)
        log.commanded_pos_rev.append(pos_rev.copy())
        log.commanded_vel_rps.append(vel_ff_rps.copy())
        log.commanded_torque_Nm.append(torque_ff_Nm.copy())
        log.actual_pos_rev.append(actual_pos)
        log.actual_vel_rps.append(actual_vel)
        log.poses.append(pose.copy())
        log.loop_dt_s.append(dt)

        # 6. Safety checks
        for axis_id in LEG_AXES:
            if harness.states[axis_id].active_errors != 0:
                names = error_names(harness.states[axis_id].active_errors)
                raise RuntimeError(
                    f"Axis {axis_id} error during trajectory: {names}")

        if not harness.all_heartbeats_fresh():
            raise RuntimeError("Heartbeat timeout during trajectory execution")

        # 7. Sleep for remainder of cycle
        elapsed = time.perf_counter() - t_now
        sleep_time = TARGET_DT_S - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    return log


def hold_and_monitor(harness: PlatformTestHarness,
                     pose_6dof: np.ndarray,
                     geom: StewartGeometry,
                     params: DynamicsParams,
                     duration_s: float = 2.0) -> dict:
    """Hold at a pose with feedforward, monitor position stability.

    Returns dict with max_deviation_mm and per_leg_max_mm.
    """
    pos = pose_6dof[:3]
    rot = rotvec_to_rot_matrix(pose_6dof[3:6])

    raw_positions = pose_to_raw_positions(pos, rot, geom)
    from jugglebot.motion.dynamics import gravity_to_motor_torques
    torques_Nm = gravity_to_motor_torques(pos, rot, geom, params)

    # Record starting positions
    harness._poll()
    start_rev = np.array([harness.states[a].pos_rev for a in LEG_AXES])

    max_dev_rev = np.zeros(6)
    n_samples = int(duration_s / 0.05)

    for _ in range(n_samples):
        # Send hold command with feedforward (negate torque for ODrive leg convention)
        for i, axis_id in enumerate(LEG_AXES):
            tor_int = int(round(-torques_Nm[i] * LEG_TOR_FF_SCALE))
            tor_int = max(-32767, min(32767, tor_int))
            harness.send_no_delay(encode_set_input_pos(
                axis_id, raw_positions[i], vel_ff=0, torque_ff=tor_int))

        harness.poll_for(0.05)

        for i, axis_id in enumerate(LEG_AXES):
            dev = abs(harness.states[axis_id].pos_rev - start_rev[i])
            if dev > max_dev_rev[i]:
                max_dev_rev[i] = dev

            if harness.states[axis_id].active_errors != 0:
                names = error_names(harness.states[axis_id].active_errors)
                raise RuntimeError(
                    f"Axis {axis_id} error during hold: {names}")

    max_dev_mm = max_dev_rev / MM_TO_REV
    return {
        'max_deviation_mm': float(np.max(max_dev_mm)),
        'per_leg_max_mm': max_dev_mm,
    }


def prepare_harness(harness: PlatformTestHarness):
    """Common harness setup: clear errors, set limits/gains, enter position mode."""
    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.set_gains_all(BASELINE_POS_GAIN, BASELINE_VEL_GAIN,
                          BASELINE_VEL_INT_GAIN)
    harness.require_no_errors_all()
    harness.enter_position_mode_all()


def move_to_home(harness: PlatformTestHarness, geom: StewartGeometry,
                 params: DynamicsParams):
    """Move to home pose using TRAP_TRAJ (safe ramped transition)."""
    home_raw = pose_to_raw_positions(np.zeros(3), np.eye(3), geom)
    from jugglebot.motion.dynamics import gravity_to_motor_torques
    home_torques = gravity_to_motor_torques(
        np.zeros(3), np.eye(3), geom, params)

    harness.enter_trap_traj_mode_all(vel_limit=1.5, acc_limit=5.0,
                                     dec_limit=5.0)
    for i, axis_id in enumerate(LEG_AXES):
        tor_int = int(round(-home_torques[i] * LEG_TOR_FF_SCALE))
        tor_int = max(-32767, min(32767, tor_int))
        harness.send(encode_set_input_pos(
            axis_id, home_raw[i], vel_ff=0, torque_ff=tor_int))

    harness.wait_for_all_trajectories_done(timeout_s=15.0)
    harness.poll_for(1.0)
    print("  At home pose.")


def switch_to_passthrough(harness: PlatformTestHarness):
    """Switch all axes from TRAP_TRAJ to PASSTHROUGH (for quintic control)."""
    from free_platform_test import encode_set_controller_mode
    for axis_id in LEG_AXES:
        harness.send(encode_set_controller_mode(
            axis_id, 'POSITION', 'PASSTHROUGH'))
        time.sleep(0.05)
    print("  Switched to POSITION/PASSTHROUGH mode.")


# ---------------------------------------------------------------------------
# Test T1: Small move from home
# ---------------------------------------------------------------------------

def test_small_move(harness: PlatformTestHarness,
                    speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """T1: Home -> +10mm Z, 1.0s duration, rest-to-rest.

    Verifies the complete trajectory execution pipeline:
      - Trajectory evaluation in tight loop
      - set_input_pos with vel_ff and torque_ff
      - Encoder tracking
      - Post-trajectory hold stability

    Pass criteria:
      - Max tracking error < 1.5 mm during motion
      - Hold deviation < 0.5 mm after settling
    """
    print("\n" + "=" * 60)
    print("T1: Small move from home (+10mm Z)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    end_pose = np.array([0, 0, 10, 0, 0, 0], dtype=float)

    # Forward trajectory
    traj_fwd = make_rest_to_rest(end_pose, duration=1.0,
                                 speed_scale=speed_scale)
    result_fwd = check_feasibility(traj_fwd, geom, params)
    print_feasibility('Home -> +10mm Z', result_fwd)
    if not result_fwd.feasible:
        print("  FAIL: Forward trajectory infeasible")
        return False

    # Return trajectory
    traj_ret = make_rest_to_rest(np.zeros(6), duration=1.0,
                                 speed_scale=speed_scale,
                                 start_pose=end_pose)
    result_ret = check_feasibility(traj_ret, geom, params)
    print_feasibility('+10mm Z -> Home', result_ret)
    if not result_ret.feasible:
        print("  FAIL: Return trajectory infeasible")
        return False

    interactive_pause("Press Enter to execute T1...")

    # Setup
    prepare_harness(harness)
    move_to_home(harness, geom, params)
    switch_to_passthrough(harness)

    # Execute forward
    print(f"\n  Executing: Home -> +10mm Z "
          f"(duration {traj_fwd.duration:.2f}s + {HOLD_AFTER_S}s hold)...")
    log_fwd = execute_trajectory(harness, traj_fwd, geom, params)
    analysis_fwd = analyze_log(log_fwd)
    print_analysis('Forward trajectory', analysis_fwd)

    # Hold check
    print(f"\n  Checking hold stability at +10mm Z...")
    hold_result = hold_and_monitor(harness, end_pose, geom, params,
                                   duration_s=HOLD_AFTER_S)
    print(f"    Hold max deviation: {hold_result['max_deviation_mm']:.3f} mm")
    print(f"    Per-leg: [{', '.join(f'{d:.3f}' for d in hold_result['per_leg_max_mm'])}] mm")

    # Execute return
    print(f"\n  Executing: +10mm Z -> Home "
          f"(duration {traj_ret.duration:.2f}s + {HOLD_AFTER_S}s hold)...")
    log_ret = execute_trajectory(harness, traj_ret, geom, params)
    analysis_ret = analyze_log(log_ret)
    print_analysis('Return trajectory', analysis_ret)

    harness.idle_all()

    # Verdict
    pass_fwd = analysis_fwd['max_error_mm'] < MAX_TRACKING_ERROR_MM
    pass_hold = hold_result['max_deviation_mm'] < MAX_HOLD_DEVIATION_MM
    pass_ret = analysis_ret['max_error_mm'] < MAX_TRACKING_ERROR_MM

    print(f"\n  Results:")
    print(f"    Forward tracking:  {analysis_fwd['max_error_mm']:.3f} mm  "
          f"[{'PASS' if pass_fwd else 'FAIL'}]")
    print(f"    Hold stability:    {hold_result['max_deviation_mm']:.3f} mm  "
          f"[{'PASS' if pass_hold else 'FAIL'}]")
    print(f"    Return tracking:   {analysis_ret['max_error_mm']:.3f} mm  "
          f"[{'PASS' if pass_ret else 'FAIL'}]")

    all_pass = pass_fwd and pass_hold and pass_ret
    print(f"\n  {'PASS' if all_pass else 'FAIL'}: T1 Small move from home")
    return all_pass


# ---------------------------------------------------------------------------
# Test T2: Graduated distance/tilt
# ---------------------------------------------------------------------------

def test_graduated(harness: PlatformTestHarness,
                   speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """T2: Graduated increases in distance and tilt.

    Moves (each from home, return to home):
      a) +20mm Z
      b) 5 deg tilt about X
      c) +20mm Z + 5 deg tilt X (combined)
      d) +40mm Z

    Pass criteria: tracking error < 1.5 mm for each move.
    """
    print("\n" + "=" * 60)
    print("T2: Graduated distance/tilt")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    zeros6 = np.zeros(6)

    rx5 = np.deg2rad(5)
    moves = [
        ('T2a: +20mm Z',           np.array([0, 0, 20, 0, 0, 0], dtype=float), 1.0),
        ('T2b: 5 deg tilt X',      np.array([0, 0, 0, rx5, 0, 0], dtype=float), 1.0),
        ('T2c: +20mm Z + 5deg X',  np.array([0, 0, 20, rx5, 0, 0], dtype=float), 1.5),
        ('T2d: +40mm Z',           np.array([0, 0, 40, 0, 0, 0], dtype=float), 1.5),
    ]

    # Pre-flight all trajectories
    print("\n  Pre-flight checks:")
    all_feasible = True
    move_trajs = []
    for label, end_pose, dur in moves:
        traj_fwd = make_rest_to_rest(end_pose, dur, speed_scale)
        traj_ret = make_rest_to_rest(zeros6, dur, speed_scale,
                                     start_pose=end_pose)
        res_fwd = check_feasibility(traj_fwd, geom, params)
        res_ret = check_feasibility(traj_ret, geom, params)
        print_feasibility(f'{label} (fwd)', res_fwd)
        if not res_fwd.feasible or not res_ret.feasible:
            all_feasible = False
        move_trajs.append((label, end_pose, traj_fwd, traj_ret))

    if not all_feasible:
        print("  FAIL: Some trajectories infeasible")
        return False

    interactive_pause("Press Enter to execute T2...")

    prepare_harness(harness)
    all_pass = True

    for label, end_pose, traj_fwd, traj_ret in move_trajs:
        print(f"\n  --- {label} ---")

        # Move to home first
        move_to_home(harness, geom, params)
        switch_to_passthrough(harness)

        # Execute forward
        print(f"  Executing forward ({traj_fwd.duration:.2f}s)...")
        log_fwd = execute_trajectory(harness, traj_fwd, geom, params)
        analysis = analyze_log(log_fwd)
        print_analysis(f'{label} forward', analysis)

        # Execute return
        print(f"  Executing return ({traj_ret.duration:.2f}s)...")
        log_ret = execute_trajectory(harness, traj_ret, geom, params)
        analysis_ret = analyze_log(log_ret)
        print_analysis(f'{label} return', analysis_ret)

        worst = max(analysis['max_error_mm'], analysis_ret['max_error_mm'])
        ok = worst < MAX_TRACKING_ERROR_MM
        print(f"  {label}: worst tracking = {worst:.3f} mm "
              f"[{'PASS' if ok else 'FAIL'}]")
        if not ok:
            all_pass = False

    harness.idle_all()
    print(f"\n  {'PASS' if all_pass else 'FAIL'}: T2 Graduated distance/tilt")
    return all_pass


# ---------------------------------------------------------------------------
# Test T3: Multi-pose sequence
# ---------------------------------------------------------------------------

def test_multi_pose(harness: PlatformTestHarness,
                    speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """T3: Multi-pose sequence across the workspace.

    Chains 5 poses in sequence (start from home, return to home):
      1. (0, 0, 30) mm
      2. (20, -10, 20) mm
      3. (-15, 15, 10) mm + 3 deg X tilt
      4. (0, 0, 50) mm + 3 deg X - 2 deg Y
      5. Home

    Pass criteria: each segment tracking error < 1.5 mm.
    """
    print("\n" + "=" * 60)
    print("T3: Multi-pose sequence")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    rx3 = np.deg2rad(3)
    ry2 = np.deg2rad(-2)
    sequence = [
        ('30mm Z',                np.array([0, 0, 30, 0, 0, 0], dtype=float)),
        ('20,-10,20mm',           np.array([20, -10, 20, 0, 0, 0], dtype=float)),
        ('-15,15,10mm + 3deg X',  np.array([-15, 15, 10, rx3, 0, 0], dtype=float)),
        ('0,0,50mm + 3X -2Y',    np.array([0, 0, 50, rx3, ry2, 0], dtype=float)),
        ('Home',                  np.zeros(6)),
    ]

    # Build chained trajectories
    segment_trajs = []
    prev_pose = np.zeros(6)
    print("\n  Pre-flight checks:")
    all_feasible = True

    for label, end_pose in sequence:
        traj = make_rest_to_rest(end_pose, duration=1.5,
                                 speed_scale=speed_scale,
                                 start_pose=prev_pose)
        res = check_feasibility(traj, geom, params)
        print_feasibility(f'-> {label}', res)
        if not res.feasible:
            all_feasible = False
        segment_trajs.append((label, traj))
        prev_pose = end_pose.copy()

    if not all_feasible:
        print("  FAIL: Some segments infeasible")
        return False

    interactive_pause("Press Enter to execute T3...")

    prepare_harness(harness)
    move_to_home(harness, geom, params)
    switch_to_passthrough(harness)

    all_pass = True
    for label, traj in segment_trajs:
        print(f"\n  Executing: -> {label} ({traj.duration:.2f}s)...")
        log = execute_trajectory(harness, traj, geom, params, hold_s=1.0)
        analysis = analyze_log(log)
        print_analysis(f'-> {label}', analysis)

        ok = analysis['max_error_mm'] < MAX_TRACKING_ERROR_MM
        print(f"  -> {label}: max tracking = {analysis['max_error_mm']:.3f} mm "
              f"[{'PASS' if ok else 'FAIL'}]")
        if not ok:
            all_pass = False

    harness.idle_all()
    print(f"\n  {'PASS' if all_pass else 'FAIL'}: T3 Multi-pose sequence")
    return all_pass


# ---------------------------------------------------------------------------
# Test T4: Speed scale verification
# ---------------------------------------------------------------------------

def test_speed_scaling(harness: PlatformTestHarness,
                       speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """T4: Speed scale verification.

    Execute the same 30mm Z move at two speed scales:
      - Primary: 2 * speed_scale (default 50%)
      - Comparison: speed_scale (default 25%)

    Both speeds are doubled vs the original 25%/12.5% to avoid
    cogging-torque effects that distort velocity tracking at very
    low speeds.

    Verify:
      - Duration ratio ~2x (within 15%)
      - 95th-percentile encoder velocity ratio ~2x (within 15%)
      - Both reach the same target

    Pass criteria: ratio errors < MAX_SPEED_RATIO_ERROR.
    """
    print("\n" + "=" * 60)
    print("T4: Speed scale verification")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    end_pose = np.array([0, 0, 30, 0, 0, 0], dtype=float)
    scale_a = speed_scale * 2.0
    scale_b = speed_scale

    # Create both trajectories
    traj_a = make_rest_to_rest(end_pose, duration=1.0, speed_scale=scale_a)
    traj_b = make_rest_to_rest(end_pose, duration=1.0, speed_scale=scale_b)
    traj_ret_a = make_rest_to_rest(np.zeros(6), duration=1.0,
                                    speed_scale=scale_a,
                                    start_pose=end_pose)
    traj_ret_b = make_rest_to_rest(np.zeros(6), duration=1.0,
                                    speed_scale=scale_b,
                                    start_pose=end_pose)

    print(f"\n  Trajectory A: scale={scale_a}, duration={traj_a.duration:.2f}s")
    print(f"  Trajectory B: scale={scale_b}, duration={traj_b.duration:.2f}s")
    print(f"  Expected duration ratio: {traj_b.duration / traj_a.duration:.2f}x")

    # Pre-flight
    for label, traj in [('A fwd', traj_a), ('A ret', traj_ret_a),
                         ('B fwd', traj_b), ('B ret', traj_ret_b)]:
        res = check_feasibility(traj, geom, params)
        print_feasibility(label, res)
        if not res.feasible:
            print(f"  FAIL: {label} infeasible")
            return False

    interactive_pause("Press Enter to execute T4...")

    prepare_harness(harness)

    # Execute trajectory A (faster)
    print(f"\n  Executing trajectory A (scale={scale_a})...")
    move_to_home(harness, geom, params)
    switch_to_passthrough(harness)
    log_a = execute_trajectory(harness, traj_a, geom, params, hold_s=1.0)
    analysis_a = analyze_log(log_a)
    print_analysis('Trajectory A', analysis_a)

    # Return home
    log_ret_a = execute_trajectory(harness, traj_ret_a, geom, params,
                                    hold_s=1.0)

    # Execute trajectory B (slower)
    print(f"\n  Executing trajectory B (scale={scale_b})...")
    log_b = execute_trajectory(harness, traj_b, geom, params, hold_s=1.0)
    analysis_b = analyze_log(log_b)
    print_analysis('Trajectory B', analysis_b)

    # Return home
    log_ret_b = execute_trajectory(harness, traj_ret_b, geom, params,
                                    hold_s=1.0)

    harness.idle_all()

    # Compute encoder velocities — use 95th percentile of absolute velocity
    # across all legs instead of raw max, which is dominated by encoder noise.
    vel_a = np.array(log_a.actual_vel_rps)       # (N, 6)
    vel_b = np.array(log_b.actual_vel_rps)       # (N, 6)
    abs_vel_a = np.abs(vel_a)
    abs_vel_b = np.abs(vel_b)
    peak_vel_a_raw = float(np.max(abs_vel_a))
    peak_vel_b_raw = float(np.max(abs_vel_b))
    p95_vel_a = float(np.percentile(abs_vel_a, 95))
    p95_vel_b = float(np.percentile(abs_vel_b, 95))

    # Compute duration ratio (trajectory portion only, exclude hold)
    dur_a = traj_a.duration
    dur_b = traj_b.duration
    dur_ratio = dur_b / dur_a
    expected_dur_ratio = 2.0
    dur_error = abs(dur_ratio - expected_dur_ratio) / expected_dur_ratio

    # Compute velocity ratio from robust (p95) estimate
    vel_ratio = p95_vel_a / p95_vel_b if p95_vel_b > 1e-6 else float('inf')
    expected_vel_ratio = 2.0
    vel_error = abs(vel_ratio - expected_vel_ratio) / expected_vel_ratio

    print(f"\n  --- Speed Scale Comparison ---")
    print(f"    Duration A: {dur_a:.2f}s,  Duration B: {dur_b:.2f}s")
    print(f"    Duration ratio: {dur_ratio:.3f}x  "
          f"(expected {expected_dur_ratio:.1f}x, error {dur_error*100:.1f}%)")
    print(f"    Peak vel A: {peak_vel_a_raw:.4f} rev/s (raw max),  "
          f"{p95_vel_a:.4f} rev/s (p95)")
    print(f"    Peak vel B: {peak_vel_b_raw:.4f} rev/s (raw max),  "
          f"{p95_vel_b:.4f} rev/s (p95)")
    print(f"    Velocity ratio (p95): {vel_ratio:.3f}x  "
          f"(expected {expected_vel_ratio:.1f}x, error {vel_error*100:.1f}%)")

    pass_dur = dur_error < MAX_SPEED_RATIO_ERROR
    pass_vel = vel_error < MAX_SPEED_RATIO_ERROR
    pass_track_a = analysis_a['max_error_mm'] < MAX_TRACKING_ERROR_MM
    pass_track_b = analysis_b['max_error_mm'] < MAX_TRACKING_ERROR_MM

    print(f"\n  Results:")
    print(f"    Duration ratio:    {dur_error*100:.1f}% error  "
          f"[{'PASS' if pass_dur else 'FAIL'}]")
    print(f"    Velocity ratio:    {vel_error*100:.1f}% error  "
          f"[{'PASS' if pass_vel else 'FAIL'}]")
    print(f"    Tracking A:        {analysis_a['max_error_mm']:.3f} mm  "
          f"[{'PASS' if pass_track_a else 'FAIL'}]")
    print(f"    Tracking B:        {analysis_b['max_error_mm']:.3f} mm  "
          f"[{'PASS' if pass_track_b else 'FAIL'}]")

    all_pass = pass_dur and pass_vel and pass_track_a and pass_track_b
    print(f"\n  {'PASS' if all_pass else 'FAIL'}: T4 Speed scale verification")
    return all_pass


# ---------------------------------------------------------------------------
# Test registry and CLI
# ---------------------------------------------------------------------------

TESTS = {
    'small_move':  ('T1: Small move from home (10mm Z)',
                    test_small_move),
    'graduated':   ('T2: Graduated distance/tilt',
                    test_graduated),
    'multi_pose':  ('T3: Multi-pose sequence',
                    test_multi_pose),
    'speed_scale': ('T4: Speed scale verification',
                    test_speed_scaling),
}

TEST_GROUPS = {
    'all': ['small_move', 'graduated', 'multi_pose', 'speed_scale'],
}


def run_dry_run(speed_scale: float, show_preview: bool):
    """Run feasibility checks on all test trajectories without CAN."""
    from trajectory_viewer import build_test_trajectories, preview_test_sequence

    print("\n" + "=" * 60)
    print("DRY RUN: Feasibility checks (no hardware)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    trajs = build_test_trajectories(geom, speed_scale=speed_scale)

    print(f"\n  Speed scale: {speed_scale}")
    print(f"  Trajectories: {len(trajs)}")
    print()

    all_ok = True
    for label, traj in trajs:
        result = check_feasibility(traj, geom, params)
        status = 'OK' if result.feasible else 'FAIL'
        peak_vel = np.max(result.peak_leg_vel_rps)
        peak_cond = result.peak_condition_number
        print(f"  [{status}] {label}: "
              f"dur={traj.duration:.2f}s  "
              f"peak_vel={peak_vel:.3f} rev/s  "
              f"peak_cond={peak_cond:.0f}")
        if not result.feasible:
            all_ok = False
            for v in result.violations:
                print(f"        {v}")

    print(f"\n  {'ALL FEASIBLE' if all_ok else 'SOME INFEASIBLE'}")

    if show_preview:
        print("\n  Launching 3D preview (close window to continue)...")
        preview_test_sequence(trajs, geom, params)


def main():
    parser = argparse.ArgumentParser(
        description='Trajectory test harness for Jugglebot Phase 4',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests available:
  small_move    T1: Small move from home (10mm Z, 25%% speed)
  graduated     T2: Graduated distance/tilt (4 moves)
  multi_pose    T3: Multi-pose sequence (5 chained poses)
  speed_scale   T4: Speed scale verification (50%% vs 25%%)

Test groups:
  all           Run T1-T4 in order [DEFAULT]

Modes:
  --dry-run     Feasibility checks only, no CAN connection
  --preview     Show 3D viewer before hardware execution
  --dry-run --preview    Feasibility + 3D viewer, no hardware

Prerequisites:
  - All Phase 3 Stage C tests must PASS first
  - Platform free-standing, legs homed (use --home flag)
  - BE READY: watch for unexpected motion

Safety:
  Current limit: {curr}A (50%% of {rated}A).
  All tests send IDLE to all 6 axes on Ctrl-C.
  Mandatory feasibility check before every trajectory execution.
        """.format(curr=SAFE_CURRENT_LIMIT_A,
                   rated=hw.ODRIVE_LEG_CURR_LIMIT_A))

    parser.add_argument('--interface', default='socketcan',
                        help='python-can interface (default: socketcan)')
    parser.add_argument('--channel', default='can0',
                        help='CAN channel name (default: can0)')
    parser.add_argument('--test', default='all',
                        choices=list(TESTS.keys()) + list(TEST_GROUPS.keys()),
                        help='Which test or group to run (default: all)')
    parser.add_argument('--home', action='store_true',
                        help='Home all 6 axes before running tests')
    parser.add_argument('--preview', action='store_true',
                        help='Show 3D viewer preview before hardware execution')
    parser.add_argument('--dry-run', action='store_true',
                        help='Feasibility checks only, no CAN connection')
    parser.add_argument('--speed-scale', type=float, default=DEFAULT_SPEED_SCALE,
                        help=f'Speed scale factor (default: {DEFAULT_SPEED_SCALE})')

    args = parser.parse_args()

    # Dry-run mode: no CAN, just feasibility + optional preview
    if args.dry_run:
        run_dry_run(args.speed_scale, args.preview)
        return 0

    # Determine which tests to run
    if args.test in TEST_GROUPS:
        tests_to_run = TEST_GROUPS[args.test]
    else:
        tests_to_run = [args.test]

    # Optional 3D preview before hardware execution
    if args.preview:
        try:
            from trajectory_viewer import (build_test_trajectories,
                                           preview_test_sequence)
            geom = StewartGeometry()
            params = DynamicsParams.from_config()
            trajs = build_test_trajectories(geom, speed_scale=args.speed_scale)
            print("\n  Launching 3D preview (close window to continue)...")
            preview_test_sequence(trajs, geom, params)
        except ImportError:
            print("  WARNING: matplotlib not available, skipping preview")

    # Install Ctrl-C handler for clean shutdown
    harness_ref = [None]
    original_handler = signal.getsignal(signal.SIGINT)

    def sigint_handler(sig, frame):
        print("\n\n  *** Ctrl-C: Emergency IDLE (all axes) ***")
        if harness_ref[0] is not None:
            try:
                harness_ref[0].idle_all()
            except Exception:
                pass
        signal.signal(signal.SIGINT, original_handler)
        sys.exit(1)

    signal.signal(signal.SIGINT, sigint_handler)

    # Run tests
    harness = PlatformTestHarness(
        interface=args.interface,
        channel=args.channel)
    harness_ref[0] = harness

    print(f"\nJugglebot Trajectory Test Harness (Phase 4)")
    print(f"  Interface:     {args.interface}")
    print(f"  Channel:       {args.channel}")
    print(f"  Speed scale:   {args.speed_scale}")
    print(f"  Current limit: {SAFE_CURRENT_LIMIT_A} A "
          f"(50% of {hw.ODRIVE_LEG_CURR_LIMIT_A} A)")
    print(f"\n  *** Platform must be free-standing ***")
    print(f"  *** Mandatory feasibility check before every trajectory ***")

    results = {}
    try:
        with harness:
            if args.home:
                harness.home_all()

            for test_name in tests_to_run:
                label, func = TESTS[test_name]
                try:
                    result = func(harness, speed_scale=args.speed_scale)
                    results[test_name] = result
                except Exception as e:
                    print(f"\n  ERROR in {label}: {e}")
                    results[test_name] = False
                    try:
                        harness.idle_all()
                    except Exception:
                        pass
                    time.sleep(1.0)

    except TimeoutError as e:
        print(f"\nFATAL: {e}")
        return 1
    except Exception as e:
        print(f"\nFATAL: {e}")
        return 1

    # Summary
    print("\n" + "=" * 60)
    print("RESULTS SUMMARY")
    print("=" * 60)
    for test_name in tests_to_run:
        label, _ = TESTS[test_name]
        result = results.get(test_name)
        if result is True:
            status = "PASS"
        elif result is False:
            status = "FAIL"
        else:
            status = "SKIP"
        print(f"  {status:6s}  {label}")

    n_pass = sum(1 for r in results.values() if r is True)
    n_total = len(tests_to_run)
    print(f"\n  {n_pass}/{n_total} tests passed")

    return 0 if n_pass == n_total else 1


if __name__ == '__main__':
    sys.exit(main())
