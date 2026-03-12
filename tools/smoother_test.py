#!/usr/bin/env python3
"""Standalone hardware test for the StreamSmoother (universal C2 target gate).

Feeds pose targets through the StreamSmoother in a tight ~500 Hz loop and
sends (pos, vel_ff, torque_ff) directly to ODrives via python-can.
Bypasses ROS2 entirely.

Tests (graduated risk):
  S1. Static hold — reset smoother to home, hold for 2s.  Platform should
      not move.  Baseline for tracking error.
  S2. Small step — single 20mm Z step through the smoother.
      Verifies profiled motion with nonzero vel_ff and torque_ff.
  S3. Simulated spacemouse — slow sinusoidal stream at 100Hz, +-20mm XY.
      Verifies continuous smooth motion with vel_ff tracking.
  S4. Large step — single 150mm Z step.  Verifies duration computation
      produces a properly profiled multi-hundred-ms move.
  S5. Multi-axis step — combined 80mm X, 50mm Z, 5deg roll.
      Verifies coupled motion.

Prerequisites:
  - All Phase 6 hardening tests must PASS
  - Robot fully assembled, platform free-standing
  - Legs must be homed (use --home flag)

Safety:
  - Mandatory feasibility pre-check via IK on smoother endpoints
  - Conservative current limit (50% of rated)
  - POSITION/PASSTHROUGH mode (smoother provides smoothing)
  - Interactive confirmation before each test
  - All axes sent IDLE on completion, error, or Ctrl-C

Usage:
  python tools/smoother_test.py --home --test all
  python tools/smoother_test.py --test static_hold
  python tools/smoother_test.py --preview
  python tools/smoother_test.py --dry-run --preview

Requirements:
  pip install python-can numpy
  (optional for --preview: matplotlib)
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
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

import hardware_config as hw
import protocol_config as proto

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams, gravity_to_motor_torques
from jugglebot.motion.ik_solver import (
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.conversions import extensions_mm_to_revs
from jugglebot.motion.trajectory import cartesian_to_motor_commands
from jugglebot.motion.stream_smoother import StreamSmoother
from jugglebot.motion.workspace import WorkspaceLimits, check_workspace_limits

from free_platform_test import (
    PlatformTestHarness,
    LEG_AXES,
    encode_set_input_pos,
    encode_set_controller_mode,
    pose_to_raw_positions,
    error_names,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TARGET_LOOP_HZ = 500
TARGET_DT_S = 1.0 / TARGET_LOOP_HZ
HOLD_AFTER_S = 1.0                    # hold time after each test

LEG_VEL_FF_SCALE = proto.INPUT_SCALE_LEG_VEL     # 1000
LEG_TOR_FF_SCALE = proto.INPUT_SCALE_LEG_TOR     # 10000
MM_TO_REV = np.array(hw.GEOM_MM_TO_REV, dtype=np.float64)

BASELINE_POS_GAIN = 40.0
BASELINE_VEL_GAIN = 0.2
BASELINE_VEL_INT_GAIN = 0.32
SAFE_CURRENT_LIMIT_A = hw.ODRIVE_LEG_CURR_LIMIT_A * 0.5

HOME_POSE = np.array([0, 0, hw.JB_OP_DEFAULT_ACTIVE_Z_MM, 0, 0, 0],
                      dtype=float)

# Tracking thresholds
STATIC_HOLD_THRESHOLD_MM = 0.5
STEP_TRACKING_THRESHOLD_MM = 2.0
STREAM_TRACKING_THRESHOLD_MM = 2.5
LARGE_STEP_TRACKING_THRESHOLD_MM = 3.0


# ---------------------------------------------------------------------------
# Data logging (reuses trajectory_test pattern)
# ---------------------------------------------------------------------------

@dataclass
class SmootherLog:
    """Records per-cycle data during smoother execution."""
    timestamps: list = field(default_factory=list)
    commanded_pos_rev: list = field(default_factory=list)
    commanded_vel_rps: list = field(default_factory=list)
    commanded_torque_Nm: list = field(default_factory=list)
    actual_pos_rev: list = field(default_factory=list)
    actual_vel_rps: list = field(default_factory=list)
    poses: list = field(default_factory=list)
    twists: list = field(default_factory=list)
    smoother_durations: list = field(default_factory=list)
    loop_dt_s: list = field(default_factory=list)
    metadata: dict = field(default_factory=dict)


def analyze_log(log: SmootherLog) -> dict:
    """Compute tracking metrics from a smoother execution log."""
    if not log.timestamps:
        return {'max_error_mm': float('inf'), 'rms_error_mm': float('inf'),
                'per_leg_max_mm': [float('inf')] * 6,
                'loop_mean_ms': 0, 'loop_p99_ms': 0, 'loop_max_ms': 0,
                'n_samples': 0, 'peak_vel_ff_rps': 0, 'peak_torque_ff_Nm': 0}

    cmd = np.array(log.commanded_pos_rev)
    act = np.array(log.actual_pos_rev)
    vel = np.array(log.commanded_vel_rps)
    torq = np.array(log.commanded_torque_Nm)

    err_rev = np.abs(cmd - act)
    err_mm = err_rev / MM_TO_REV[np.newaxis, :]

    per_leg_max = np.max(err_mm, axis=0)
    max_error = float(np.max(per_leg_max))
    rms_error = float(np.sqrt(np.mean(err_mm ** 2)))

    dts = np.array(log.loop_dt_s) * 1000

    return {
        'max_error_mm': max_error,
        'rms_error_mm': rms_error,
        'per_leg_max_mm': per_leg_max.tolist(),
        'loop_mean_ms': float(np.mean(dts)) if len(dts) > 0 else 0,
        'loop_p99_ms': float(np.percentile(dts, 99)) if len(dts) > 0 else 0,
        'loop_max_ms': float(np.max(dts)) if len(dts) > 0 else 0,
        'n_samples': len(log.timestamps),
        'peak_vel_ff_rps': float(np.max(np.abs(vel))),
        'peak_torque_ff_Nm': float(np.max(np.abs(torq))),
    }


def print_analysis(label: str, analysis: dict):
    """Pretty-print a smoother analysis summary."""
    print(f"\n  --- {label} ---")
    print(f"    Samples:        {analysis['n_samples']}")
    print(f"    Max error:      {analysis['max_error_mm']:.3f} mm")
    print(f"    RMS error:      {analysis['rms_error_mm']:.3f} mm")
    print(f"    Per-leg max:    [{', '.join(f'{e:.3f}' for e in analysis['per_leg_max_mm'])}] mm")
    print(f"    Peak vel_ff:    {analysis['peak_vel_ff_rps']:.3f} rev/s")
    print(f"    Peak torque_ff: {analysis['peak_torque_ff_Nm']:.4f} Nm")
    print(f"    Loop timing:    mean={analysis['loop_mean_ms']:.2f} ms  "
          f"p99={analysis['loop_p99_ms']:.2f} ms  "
          f"max={analysis['loop_max_ms']:.2f} ms")


def save_log(log: SmootherLog, filepath: str | Path) -> None:
    """Save a SmootherLog to JSON file."""
    filepath = Path(filepath)
    filepath.parent.mkdir(parents=True, exist_ok=True)

    def to_list(a):
        return a.tolist() if hasattr(a, 'tolist') else a

    data = {
        'metadata': log.metadata,
        'samples': {
            'timestamps': log.timestamps,
            'commanded_pos_rev': [to_list(a) for a in log.commanded_pos_rev],
            'commanded_vel_rps': [to_list(a) for a in log.commanded_vel_rps],
            'commanded_torque_Nm': [to_list(a) for a in log.commanded_torque_Nm],
            'actual_pos_rev': [to_list(a) for a in log.actual_pos_rev],
            'actual_vel_rps': [to_list(a) for a in log.actual_vel_rps],
            'poses': [to_list(a) for a in log.poses],
            'twists': [to_list(a) for a in log.twists],
            'smoother_durations': log.smoother_durations,
            'loop_dt_s': log.loop_dt_s,
        },
    }

    with open(filepath, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"  Log saved: {filepath} ({len(log.timestamps)} samples)")


# ---------------------------------------------------------------------------
# Harness setup helpers (matching trajectory_test.py pattern)
# ---------------------------------------------------------------------------

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
    """Move to active home using TRAP_TRAJ (safe ramped transition)."""
    home_pos = HOME_POSE[:3]
    home_rot = rotvec_to_rot_matrix(HOME_POSE[3:6])
    home_raw = pose_to_raw_positions(home_pos, home_rot, geom)
    home_torques = gravity_to_motor_torques(home_pos, home_rot, geom, params)

    harness.enter_trap_traj_mode_all(vel_limit=1.5, acc_limit=5.0,
                                     dec_limit=5.0)
    for i, axis_id in enumerate(LEG_AXES):
        tor_int = int(round(-home_torques[i] * LEG_TOR_FF_SCALE))
        tor_int = max(-32767, min(32767, tor_int))
        harness.send(encode_set_input_pos(
            axis_id, home_raw[i], vel_ff=0, torque_ff=tor_int))

    harness.wait_for_all_trajectories_done(timeout_s=15.0)
    harness.poll_for(1.0)
    print("  At active home pose.")


def switch_to_passthrough(harness: PlatformTestHarness):
    """Switch all axes from TRAP_TRAJ to PASSTHROUGH."""
    for axis_id in LEG_AXES:
        harness.send(encode_set_controller_mode(
            axis_id, 'POSITION', 'PASSTHROUGH'))
        time.sleep(0.05)
    print("  Switched to POSITION/PASSTHROUGH mode.")


def interactive_pause(prompt: str) -> str:
    """Pause for operator input."""
    return input(f"\n  >>> {prompt} ").strip()


# ---------------------------------------------------------------------------
# Smoother execution loop
# ---------------------------------------------------------------------------

def execute_smoother_sequence(
        harness: PlatformTestHarness,
        smoother: StreamSmoother,
        targets: list[tuple[float, np.ndarray]],
        geom: StewartGeometry,
        params: DynamicsParams,
        total_duration_s: float,
        hold_s: float = HOLD_AFTER_S) -> SmootherLog:
    """Execute a sequence of timed targets through the StreamSmoother.

    Parameters
    ----------
    harness : PlatformTestHarness — connected CAN interface
    smoother : StreamSmoother — already reset to initial pose
    targets : list of (time_s, pose_6dof) — targets to feed at the given times
    geom : StewartGeometry
    params : DynamicsParams
    total_duration_s : float — total run time (including motion + hold)
    hold_s : float — extra hold time after total_duration_s

    Returns
    -------
    SmootherLog — per-cycle recording
    """
    log = SmootherLog()
    full_duration = total_duration_s + hold_s

    if sys.platform == 'win32':
        print("  WARNING: Windows timer limits loop to ~60 Hz. "
              "Run on Jetson for 500 Hz.")

    # Sort targets by time
    targets_sorted = sorted(targets, key=lambda x: x[0])
    target_idx = 0

    t_loop_start = time.perf_counter()
    t_prev = t_loop_start

    while True:
        t_now = time.perf_counter()
        t_elapsed = t_now - t_loop_start
        dt = t_now - t_prev
        t_prev = t_now

        if t_elapsed > full_duration:
            break

        # Feed any targets whose time has arrived
        while target_idx < len(targets_sorted):
            t_target, pose_target = targets_sorted[target_idx]
            if t_elapsed >= t_target:
                smoother.set_target(pose_target, t_now)
                target_idx += 1
            else:
                break

        # Evaluate smoother
        pose, twist, accel = smoother.evaluate(t_now)

        # Convert to motor commands
        pos_rev, vel_ff_rps, torque_ff_Nm = cartesian_to_motor_commands(
            pose, twist, accel, geom, params, feedforward_enabled=True)

        # Send to all 6 axes (negate for ODrive convention)
        for i, axis_id in enumerate(LEG_AXES):
            raw_pos = -pos_rev[i]
            vel_int = int(round(-vel_ff_rps[i] * LEG_VEL_FF_SCALE))
            tor_int = int(round(-torque_ff_Nm[i] * LEG_TOR_FF_SCALE))
            vel_int = max(-32767, min(32767, vel_int))
            tor_int = max(-32767, min(32767, tor_int))
            harness.send_no_delay(encode_set_input_pos(
                axis_id, raw_pos, vel_int, tor_int))

        # Poll for encoder feedback
        harness._poll(timeout=0)

        # Log
        actual_pos = np.array([harness.states[a].pos_rev for a in LEG_AXES])
        actual_vel = np.array([harness.states[a].vel_rps for a in LEG_AXES])

        log.timestamps.append(t_elapsed)
        log.commanded_pos_rev.append(pos_rev.copy())
        log.commanded_vel_rps.append(vel_ff_rps.copy())
        log.commanded_torque_Nm.append(torque_ff_Nm.copy())
        log.actual_pos_rev.append(actual_pos)
        log.actual_vel_rps.append(actual_vel)
        log.poses.append(pose.copy())
        log.twists.append(twist.copy())
        log.smoother_durations.append(smoother._duration)
        log.loop_dt_s.append(dt)

        # Safety checks
        for axis_id in LEG_AXES:
            if harness.states[axis_id].active_errors != 0:
                names = error_names(harness.states[axis_id].active_errors)
                raise RuntimeError(
                    f"Axis {axis_id} error during execution: {names}")

        if not harness.all_heartbeats_fresh():
            raise RuntimeError("Heartbeat timeout during execution")

        # Sleep for remainder of cycle
        elapsed = time.perf_counter() - t_now
        sleep_time = TARGET_DT_S - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    return log


# ---------------------------------------------------------------------------
# Preview support
# ---------------------------------------------------------------------------

def preview_smoother_test(test_name: str, targets: list[tuple[float, np.ndarray]],
                          total_duration_s: float):
    """Preview a smoother test by evaluating offline and plotting.

    Compatible with --preview flag.  Uses the same StreamSmoother logic
    but evaluates with synthetic timestamps instead of hardware.
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("  WARNING: matplotlib not available, skipping preview")
        return

    from trajectory_viewer import (
        LEG_COLORS,
        CARTESIAN_LABELS,
        CARTESIAN_COLORS,
    )

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    smoother = StreamSmoother(
        geom,
        vel_limit_rps=hw.ODRIVE_TRAP_VEL_LIMIT_RPS,
        accel_limit_rps2=hw.ODRIVE_TRAP_ACC_LIMIT_RPS2)
    smoother.reset(HOME_POSE)

    # Simulate at 500 Hz
    n_samples = int(total_duration_s * TARGET_LOOP_HZ)
    dt = 1.0 / TARGET_LOOP_HZ

    targets_sorted = sorted(targets, key=lambda x: x[0])
    target_idx = 0

    ts = np.empty(n_samples)
    poses = np.empty((n_samples, 6))
    twists = np.empty((n_samples, 6))
    accels = np.empty((n_samples, 6))
    extensions = np.empty((n_samples, 6))
    leg_vels = np.empty((n_samples, 6))
    torques = np.empty((n_samples, 6))
    durations = np.empty(n_samples)

    for i in range(n_samples):
        t = i * dt
        ts[i] = t

        # Feed targets
        while target_idx < len(targets_sorted):
            t_target, pose_target = targets_sorted[target_idx]
            if t >= t_target:
                smoother.set_target(pose_target, t)
                target_idx += 1
            else:
                break

        pose, twist, accel = smoother.evaluate(t)
        poses[i] = pose
        twists[i] = twist
        accels[i] = accel
        durations[i] = smoother._duration

        pos_rev, vel_ff, torque_ff = cartesian_to_motor_commands(
            pose, twist, accel, geom, params, feedforward_enabled=True)
        rot = rotvec_to_rot_matrix(pose[3:6])
        extensions[i] = pose_to_leg_lengths(pose[:3], rot, geom)
        leg_vels[i] = vel_ff / MM_TO_REV  # approximate mm/s
        torques[i] = torque_ff

    # Plot 5 rows: pose, twist, extensions, leg velocities, torques
    fig, axes = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    fig.subplots_adjust(hspace=0.4, left=0.10, right=0.95,
                        top=0.94, bottom=0.06)

    # Row 1: Cartesian pose
    for j in range(6):
        axes[0].plot(ts, poses[:, j], color=CARTESIAN_COLORS[j],
                     label=CARTESIAN_LABELS[j], linewidth=0.8)
    axes[0].set_ylabel('mm / rad')
    axes[0].set_title('Cartesian Pose')
    axes[0].legend(fontsize=7, ncol=6, loc='upper right')

    # Row 2: Cartesian twist
    for j in range(6):
        axes[1].plot(ts, twists[:, j], color=CARTESIAN_COLORS[j],
                     label=CARTESIAN_LABELS[j], linewidth=0.8)
    axes[1].set_ylabel('mm/s / rad/s')
    axes[1].set_title('Cartesian Twist (from smoother)')

    # Row 3: Leg extensions
    for j in range(6):
        axes[2].plot(ts, extensions[:, j], color=LEG_COLORS[j],
                     label=f'Leg {j}', linewidth=0.8)
    axes[2].set_ylabel('mm')
    axes[2].set_title('Leg Extensions')
    axes[2].legend(fontsize=7, ncol=6, loc='upper right')

    # Row 4: Motor vel_ff
    for j in range(6):
        axes[3].plot(ts, leg_vels[:, j], color=LEG_COLORS[j],
                     linewidth=0.8)
    axes[3].set_ylabel('mm/s (approx)')
    axes[3].set_title('Motor Velocity Feedforward')

    # Row 5: Torque feedforward
    for j in range(6):
        axes[4].plot(ts, torques[:, j], color=LEG_COLORS[j],
                     linewidth=0.8)
    axes[4].set_ylabel('Nm')
    axes[4].set_title('Torque Feedforward')
    axes[4].set_xlabel('Time (s)')

    # Mark target arrival times
    for ax in axes:
        for t_target, _ in targets_sorted:
            ax.axvline(t_target, color='gray', linestyle='--',
                       linewidth=0.5, alpha=0.5)

    fig.suptitle(f'StreamSmoother Preview: {test_name}', fontsize=12)
    plt.show()


# ---------------------------------------------------------------------------
# Test definitions
# ---------------------------------------------------------------------------

def build_test_targets(test_name: str) -> tuple[
        list[tuple[float, np.ndarray]], float, float, str]:
    """Build (targets, total_duration, threshold, description) for a test.

    Returns
    -------
    targets : list of (time_s, pose_6dof)
    total_duration_s : float — run time including motion
    threshold_mm : float — max acceptable tracking error
    description : str
    """
    if test_name == 'static_hold':
        # S1: Just hold at home
        targets = []
        return targets, 2.0, STATIC_HOLD_THRESHOLD_MM, \
            "S1: Static hold at home (2s)"

    elif test_name == 'small_step':
        # S2: Step +20mm Z at t=0.5
        target = HOME_POSE.copy()
        target[2] += 20.0
        targets = [(0.5, target)]
        return targets, 3.0, STEP_TRACKING_THRESHOLD_MM, \
            "S2: Small step (+20mm Z)"

    elif test_name == 'stream':
        # S3: Sinusoidal XY at 100Hz for 5s
        targets = []
        freq = 0.5  # Hz — slow sine
        amplitude_mm = 20.0
        stream_hz = 100
        stream_dt = 1.0 / stream_hz
        for i in range(int(5.0 * stream_hz)):
            t = 0.5 + i * stream_dt
            angle = 2.0 * math.pi * freq * (t - 0.5)
            target = HOME_POSE.copy()
            target[0] = amplitude_mm * math.sin(angle)
            target[1] = amplitude_mm * math.cos(angle)
            targets.append((t, target))
        return targets, 6.0, STREAM_TRACKING_THRESHOLD_MM, \
            "S3: Simulated spacemouse (20mm circular, 0.5Hz)"

    elif test_name == 'large_step':
        # S4: Step +150mm Z at t=0.5
        target = HOME_POSE.copy()
        target[2] += 150.0
        targets = [(0.5, target)]
        return targets, 3.0, LARGE_STEP_TRACKING_THRESHOLD_MM, \
            "S4: Large step (+150mm Z)"

    elif test_name == 'multi_axis':
        # S5: Combined 80mm X + 50mm Z + 5deg roll at t=0.5
        target = HOME_POSE.copy()
        target[0] += 80.0
        target[2] += 50.0
        target[3] = math.radians(5.0)
        targets = [(0.5, target)]
        return targets, 3.0, LARGE_STEP_TRACKING_THRESHOLD_MM, \
            "S5: Multi-axis step (80mm X + 50mm Z + 5deg roll)"

    else:
        raise ValueError(f"Unknown test: {test_name}")


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------

def run_test(harness: PlatformTestHarness, test_name: str) -> bool:
    """Run a single smoother test on hardware."""
    targets, duration, threshold, description = build_test_targets(test_name)

    print(f"\n{'=' * 60}")
    print(f"  {description}")
    print(f"{'=' * 60}")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    # Verify target poses are within workspace
    ws_limits = WorkspaceLimits.from_geometry(geom)
    for t_time, pose in targets:
        rot = rotvec_to_rot_matrix(pose[3:6])
        ext = pose_to_leg_lengths(pose[:3], rot, geom)
        from jugglebot.motion.workspace import compute_condition_number
        cond = compute_condition_number(pose[:3], rot, geom)
        ws_check = check_workspace_limits(ext, cond, ws_limits)
        if ws_check.status.value == 'hard':
            print(f"  ABORT: Target at t={t_time:.2f}s violates hard workspace limit")
            print(f"    Extensions: {ext.round(1)} mm")
            print(f"    Violations: {ws_check.violations}")
            return False
        print(f"  Target at t={t_time:.2f}s: workspace OK "
              f"(cond={cond:.0f}, ext=[{ext.min():.1f}, {ext.max():.1f}] mm)")

    # Create smoother
    smoother = StreamSmoother(
        geom,
        vel_limit_rps=hw.ODRIVE_TRAP_VEL_LIMIT_RPS,
        accel_limit_rps2=hw.ODRIVE_TRAP_ACC_LIMIT_RPS2)
    smoother.reset(HOME_POSE)

    # Print expected smoother durations
    for t_time, pose in targets:
        smoother_copy = StreamSmoother(
            geom,
            vel_limit_rps=hw.ODRIVE_TRAP_VEL_LIMIT_RPS,
            accel_limit_rps2=hw.ODRIVE_TRAP_ACC_LIMIT_RPS2)
        smoother_copy.reset(HOME_POSE)
        smoother_copy.set_target(pose, 0.0)
        print(f"  Expected smoother duration (from home): "
              f"{smoother_copy._duration * 1000:.1f} ms")
        break  # only show first target

    # Confirm with operator
    interactive_pause("Platform is at home. Press Enter to execute...")

    # Execute
    try:
        log = execute_smoother_sequence(
            harness, smoother, targets, geom, params, duration)
    except RuntimeError as e:
        print(f"  ABORTED: {e}")
        return False

    # Analyze
    analysis = analyze_log(log)
    print_analysis(description, analysis)

    # Save log
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = Path(_PROJECT_ROOT) / f'S_{test_name}_{timestamp}.json'
    save_log(log, log_path)

    # Pass/fail
    passed = analysis['max_error_mm'] < threshold
    status = 'PASS' if passed else 'FAIL'
    print(f"\n  [{status}] Max error: {analysis['max_error_mm']:.3f} mm "
          f"(threshold: {threshold:.1f} mm)")

    if test_name != 'static_hold':
        if analysis['peak_vel_ff_rps'] < 0.001:
            print("  WARNING: vel_ff is near-zero — smoother may not be working")
        else:
            print(f"  vel_ff active: peak {analysis['peak_vel_ff_rps']:.3f} rev/s")

    return passed


def run_dry_run(show_preview: bool):
    """Dry-run: workspace checks + optional preview, no CAN."""
    print("\nDry-run mode: workspace checks only (no CAN connection)")
    print("=" * 60)

    test_names = ['static_hold', 'small_step', 'stream',
                  'large_step', 'multi_axis']

    geom = StewartGeometry()
    ws_limits = WorkspaceLimits.from_geometry(geom)

    for test_name in test_names:
        targets, duration, threshold, description = build_test_targets(test_name)
        print(f"\n  {description}")

        if not targets:
            print("    No targets (hold test)")
            continue

        for t_time, pose in targets[:3]:  # show first 3 targets max
            rot = rotvec_to_rot_matrix(pose[3:6])
            ext = pose_to_leg_lengths(pose[:3], rot, geom)
            from jugglebot.motion.workspace import compute_condition_number
            cond = compute_condition_number(pose[:3], rot, geom)
            ws_check = check_workspace_limits(ext, cond, ws_limits)
            status = 'OK' if ws_check.status.value != 'hard' else 'FAIL'
            print(f"    t={t_time:.2f}s: [{status}] "
                  f"ext=[{ext.min():.1f}, {ext.max():.1f}] mm, "
                  f"cond={cond:.0f}")

        if len(targets) > 3:
            print(f"    ... and {len(targets) - 3} more targets")

        # Show smoother duration for first target
        smoother = StreamSmoother(
            geom,
            vel_limit_rps=hw.ODRIVE_TRAP_VEL_LIMIT_RPS,
            accel_limit_rps2=hw.ODRIVE_TRAP_ACC_LIMIT_RPS2)
        smoother.reset(HOME_POSE)
        if targets:
            smoother.set_target(targets[0][1], 0.0)
            print(f"    Smoother duration (from home): "
                  f"{smoother._duration * 1000:.1f} ms")

    if show_preview:
        print("\n  Launching preview for each test...")
        for test_name in test_names:
            targets, duration, _, description = build_test_targets(test_name)
            preview_smoother_test(description, targets, duration)


# ---------------------------------------------------------------------------
# Test registry
# ---------------------------------------------------------------------------

TESTS = {
    'static_hold': 'S1: Static hold',
    'small_step':  'S2: Small step',
    'stream':      'S3: Simulated spacemouse',
    'large_step':  'S4: Large step',
    'multi_axis':  'S5: Multi-axis step',
}

TEST_GROUPS = {
    'all': list(TESTS.keys()),
    'steps': ['small_step', 'large_step', 'multi_axis'],
}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description='StreamSmoother hardware test',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests:
  static_hold   S1: Hold at home (2s, no targets)
  small_step    S2: Single +20mm Z step
  stream        S3: Simulated 100Hz spacemouse (circular +-20mm)
  large_step    S4: Single +150mm Z step
  multi_axis    S5: Combined 80mm X + 50mm Z + 5deg roll

Test groups:
  all           Run S1-S5 in order [DEFAULT]
  steps         Run S2, S4, S5

Modes:
  --preview     Show plots before hardware execution
  --dry-run     Workspace checks only, no CAN connection

Prerequisites:
  - All Phase 6 hardening tests must PASS first
  - Platform free-standing, legs homed (use --home flag)
        """)

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
                        help='Show plots before hardware execution')
    parser.add_argument('--dry-run', action='store_true',
                        help='Workspace checks only, no CAN connection')

    args = parser.parse_args()

    # Dry-run mode
    if args.dry_run:
        run_dry_run(args.preview)
        return 0

    # Determine which tests to run
    if args.test in TEST_GROUPS:
        tests_to_run = TEST_GROUPS[args.test]
    else:
        tests_to_run = [args.test]

    # Optional preview before hardware execution
    if args.preview:
        for test_name in tests_to_run:
            targets, duration, _, description = build_test_targets(test_name)
            preview_smoother_test(description, targets, duration)

    # Install Ctrl-C handler for clean shutdown
    harness_ref = [None]
    original_handler = signal.getsignal(signal.SIGINT)

    def sigint_handler(sig, frame):
        print("\n\n  *** Ctrl-C: Emergency IDLE (all axes) ***")
        h = harness_ref[0]
        if h is not None:
            try:
                h.idle_all()
            except Exception:
                pass
        signal.signal(signal.SIGINT, original_handler)
        sys.exit(1)

    signal.signal(signal.SIGINT, sigint_handler)

    # Connect and run
    harness = PlatformTestHarness(
        interface=args.interface, channel=args.channel)
    harness_ref[0] = harness

    passed = 0
    failed = 0

    try:
        with harness:
            if args.home:
                print("\n  Homing all axes...")
                harness.home_all()

            prepare_harness(harness)
            geom = StewartGeometry()
            params = DynamicsParams.from_config()
            move_to_home(harness, geom, params)
            switch_to_passthrough(harness)

            for test_name in tests_to_run:
                # Return to home between tests
                if test_name != tests_to_run[0]:
                    print("\n  Returning to home...")
                    # Use trap traj for safe return
                    move_to_home(harness, geom, params)
                    switch_to_passthrough(harness)

                if run_test(harness, test_name):
                    passed += 1
                else:
                    failed += 1

    except Exception as e:
        print(f"\n  FATAL: {e}")
        failed += 1
    finally:
        print(f"\n{'=' * 60}")
        print(f"Results: {passed} passed, {failed} failed "
              f"out of {passed + failed}")
        print(f"{'=' * 60}")

    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
