#!/usr/bin/env python3
"""Standalone hardware test harness for Jugglebot Phase 5 (Inertia Feedforward).

Validates full feedforward (gravity + platform inertia + reflected motor inertia)
at graduated speed levels.  Reuses the Phase 4 trajectory infrastructure and
adds Phase 5-specific comparisons and diagnostics.

Phase 5 hardware tests:
  T5. Feedforward comparison: gravity-only vs full feedforward (same trajectory)
  T6. Trajectory replay at speed (Phase 4 moves at 50%/75%/100%)
  T7. Feedforward prediction (differential iq: gravity-only vs full FF)

Graduated speed ramp approach:
  Execute at 25% first (baseline), then 50% → 75% → 100%.
  Each speed level must pass before proceeding to the next.

Prerequisites:
  - All Phase 4 tests must PASS
  - Robot fully assembled, platform free-standing
  - Legs must be homed (use --home flag)

Safety:
  - Mandatory check_feasibility() before every trajectory execution
  - Conservative current limit (50% of rated = 10A)
  - All tests send IDLE to all 6 axes on completion, error, or Ctrl-C
  - Interactive confirmation before each test
  - POSITION/PASSTHROUGH mode

Usage:
  python tools/inertia_test.py --home --test all --speed-scale 0.5
  python tools/inertia_test.py --test ff_comparison --speed-scale 0.25
  python tools/inertia_test.py --dry-run --preview
  python tools/inertia_test.py --test replay --speed-scale 0.75
  python tools/inertia_test.py --test all --preview --speed-scale 0.5

Requirements:
  pip install python-can numpy
  (optional for --preview: matplotlib)
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

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

# Import CAN harness and helpers
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

# Import trajectory test infrastructure (reuse Phase 4 helpers)
from trajectory_test import (  # noqa: E402
    TrajectoryLog,
    analyze_log,
    print_analysis,
    print_feasibility,
    prepare_harness,
    move_to_home,
    switch_to_passthrough,
    make_rest_to_rest,
    execute_trajectory,
    TARGET_DT_S,
    TARGET_LOOP_HZ,
    HOLD_AFTER_S,
    MAX_TRACKING_ERROR_MM,
    DEFAULT_SPEED_SCALE,
)

# Motion modules
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import (
    DynamicsParams,
    gravity_to_motor_torques,
    compute_full_feedforward_torques,
)
from jugglebot.motion.ik_solver import rotvec_to_rot_matrix
from jugglebot.motion.trajectory import (
    create_trajectory,
    evaluate,
    check_feasibility,
    cartesian_to_motor_commands,
    FeasibilityResult,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Speed-dependent tracking thresholds (mm) — from the plan document
TRACKING_THRESHOLDS_MM = {
    0.25: 1.5,   # Phase 4 baseline
    0.50: 2.0,
    0.75: 2.5,
    1.00: 3.0,
}

# Feedforward prediction validation threshold:
# PID correction current must not exceed this fraction of feedforward current
FF_PREDICTION_THRESHOLD = 0.15

# Motor torque constant (Nm/A) — from hardware measurements (Phase 3 Stage A)
MOTOR_KT = 0.0624


# ---------------------------------------------------------------------------
# Extended trajectory execution with iq logging
# ---------------------------------------------------------------------------

def execute_trajectory_with_iq(harness: PlatformTestHarness,
                               traj,
                               geom: StewartGeometry,
                               params: DynamicsParams,
                               hold_s: float = HOLD_AFTER_S) -> TrajectoryLog:
    """Execute a trajectory like execute_trajectory() but also log iq data.

    Extends the base TrajectoryLog with iq_setpoint and iq_measured lists.
    """
    log = TrajectoryLog()
    # Add iq logging lists
    log.iq_setpoint = []
    log.iq_measured = []

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

        # 1. Evaluate trajectory
        pose, twist, accel = evaluate(traj, t_elapsed)

        # 2. Convert to motor commands
        pos_rev, vel_ff_rps, torque_ff_Nm = cartesian_to_motor_commands(
            pose, twist, accel, geom, params, feedforward_enabled=True)

        # 3. Send to all 6 axes
        for i, axis_id in enumerate(LEG_AXES):
            raw_pos = -pos_rev[i]
            vel_int = int(round(-vel_ff_rps[i] * LEG_VEL_FF_SCALE))
            tor_int = int(round(-torque_ff_Nm[i] * LEG_TOR_FF_SCALE))
            vel_int = max(-32767, min(32767, vel_int))
            tor_int = max(-32767, min(32767, tor_int))
            harness.send_no_delay(encode_set_input_pos(
                axis_id, raw_pos, vel_int, tor_int))

        # 4. Poll for encoder + iq feedback
        harness._poll(timeout=0)

        # 5. Log data (including iq)
        actual_pos = np.array([harness.states[a].pos_rev for a in LEG_AXES])
        actual_vel = np.array([harness.states[a].vel_rps for a in LEG_AXES])
        iq_sp = np.array([harness.states[a].iq_setpoint for a in LEG_AXES])
        iq_meas = np.array([harness.states[a].iq_measured for a in LEG_AXES])

        log.timestamps.append(t_elapsed)
        log.commanded_pos_rev.append(pos_rev.copy())
        log.commanded_vel_rps.append(vel_ff_rps.copy())
        log.commanded_torque_Nm.append(torque_ff_Nm.copy())
        log.actual_pos_rev.append(actual_pos)
        log.actual_vel_rps.append(actual_vel)
        log.poses.append(pose.copy())
        log.loop_dt_s.append(dt)
        log.iq_setpoint.append(iq_sp)
        log.iq_measured.append(iq_meas)

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


def execute_trajectory_with_iq_gravity_only(
        harness: PlatformTestHarness,
        traj,
        geom: StewartGeometry,
        params: DynamicsParams,
        hold_s: float = HOLD_AFTER_S) -> TrajectoryLog:
    """Execute a trajectory with gravity-only feedforward, logging iq data.

    Same as execute_trajectory_gravity_only() but also records iq_setpoint
    and iq_measured per cycle for differential comparison in T7.
    """
    from jugglebot.motion.ik_solver import (
        pose_to_leg_lengths,
        twist_to_leg_velocities,
    )
    from jugglebot.motion.conversions import (
        extensions_mm_to_revs,
        leg_velocities_to_motor_velocities,
    )

    log = TrajectoryLog()
    log.iq_setpoint = []
    log.iq_measured = []

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

        # 1. Evaluate trajectory
        pose, twist, accel = evaluate(traj, t_elapsed)

        # 2. Convert to motor commands — gravity only
        pos = pose[:3]
        rot = rotvec_to_rot_matrix(pose[3:6])
        extensions_mm = pose_to_leg_lengths(pos, rot, geom)
        pos_rev = extensions_mm_to_revs(extensions_mm, geom)
        vel_mm_s = twist_to_leg_velocities(twist, pos, rot, geom)
        vel_ff_rps = leg_velocities_to_motor_velocities(vel_mm_s, geom)
        torque_ff_Nm = gravity_to_motor_torques(pos, rot, geom, params)

        # 3. Send to all 6 axes
        for i, axis_id in enumerate(LEG_AXES):
            raw_pos = -pos_rev[i]
            vel_int = int(round(-vel_ff_rps[i] * LEG_VEL_FF_SCALE))
            tor_int = int(round(-torque_ff_Nm[i] * LEG_TOR_FF_SCALE))
            vel_int = max(-32767, min(32767, vel_int))
            tor_int = max(-32767, min(32767, tor_int))
            harness.send_no_delay(encode_set_input_pos(
                axis_id, raw_pos, vel_int, tor_int))

        # 4. Poll for encoder + iq feedback
        harness._poll(timeout=0)

        # 5. Log data (including iq)
        actual_pos = np.array([harness.states[a].pos_rev for a in LEG_AXES])
        actual_vel = np.array([harness.states[a].vel_rps for a in LEG_AXES])
        iq_sp = np.array([harness.states[a].iq_setpoint for a in LEG_AXES])
        iq_meas = np.array([harness.states[a].iq_measured for a in LEG_AXES])

        log.timestamps.append(t_elapsed)
        log.commanded_pos_rev.append(pos_rev.copy())
        log.commanded_vel_rps.append(vel_ff_rps.copy())
        log.commanded_torque_Nm.append(torque_ff_Nm.copy())
        log.actual_pos_rev.append(actual_pos)
        log.actual_vel_rps.append(actual_vel)
        log.poses.append(pose.copy())
        log.loop_dt_s.append(dt)
        log.iq_setpoint.append(iq_sp)
        log.iq_measured.append(iq_meas)

        # 6. Safety checks
        for axis_id in LEG_AXES:
            if harness.states[axis_id].active_errors != 0:
                names = error_names(harness.states[axis_id].active_errors)
                raise RuntimeError(
                    f"Axis {axis_id} error during trajectory: {names}")

        if not harness.all_heartbeats_fresh():
            raise RuntimeError("Heartbeat timeout during trajectory execution")

        # 7. Sleep
        elapsed = time.perf_counter() - t_now
        sleep_time = TARGET_DT_S - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    return log


def execute_trajectory_gravity_only(harness: PlatformTestHarness,
                                     traj,
                                     geom: StewartGeometry,
                                     params: DynamicsParams,
                                     hold_s: float = HOLD_AFTER_S) -> TrajectoryLog:
    """Execute a trajectory with gravity-only feedforward (no inertia terms).

    Used for A/B comparison in the feedforward comparison test.
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

        # 1. Evaluate trajectory
        pose, twist, accel = evaluate(traj, t_elapsed)

        # 2. Convert to motor commands — gravity only
        pos = pose[:3]
        rot = rotvec_to_rot_matrix(pose[3:6])
        from jugglebot.motion.ik_solver import (
            pose_to_leg_lengths,
            twist_to_leg_velocities,
        )
        from jugglebot.motion.conversions import (
            extensions_mm_to_revs,
            leg_velocities_to_motor_velocities,
        )

        extensions_mm = pose_to_leg_lengths(pos, rot, geom)
        pos_rev = extensions_mm_to_revs(extensions_mm, geom)
        vel_mm_s = twist_to_leg_velocities(twist, pos, rot, geom)
        vel_ff_rps = leg_velocities_to_motor_velocities(vel_mm_s, geom)
        torque_ff_Nm = gravity_to_motor_torques(pos, rot, geom, params)

        # 3. Send to all 6 axes
        for i, axis_id in enumerate(LEG_AXES):
            raw_pos = -pos_rev[i]
            vel_int = int(round(-vel_ff_rps[i] * LEG_VEL_FF_SCALE))
            tor_int = int(round(-torque_ff_Nm[i] * LEG_TOR_FF_SCALE))
            vel_int = max(-32767, min(32767, vel_int))
            tor_int = max(-32767, min(32767, tor_int))
            harness.send_no_delay(encode_set_input_pos(
                axis_id, raw_pos, vel_int, tor_int))

        # 4. Poll for encoder feedback
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

        # 7. Sleep
        elapsed = time.perf_counter() - t_now
        sleep_time = TARGET_DT_S - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    return log


# ---------------------------------------------------------------------------
# Test T5: Feedforward comparison (gravity-only vs full)
# ---------------------------------------------------------------------------

def test_ff_comparison(harness: PlatformTestHarness,
                       speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """T5: Compare gravity-only vs full feedforward on identical trajectories.

    Executes the same move twice at the current speed:
      A) Gravity-only torque_ff
      B) Full feedforward (gravity + inertia + reflected motor)

    Pass criteria:
      - Full feedforward should reduce peak tracking error vs gravity-only.
      - Both runs must complete without errors.
    """
    print("\n" + "=" * 60)
    print(f"T5: Feedforward comparison (scale={speed_scale})")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    # Use a combined move that exercises both translation and rotation
    rx3 = np.deg2rad(3)
    end_pose = np.array([0, 0, 30, rx3, 0, 0], dtype=float)

    traj_fwd = make_rest_to_rest(end_pose, duration=1.5,
                                  speed_scale=speed_scale)
    traj_ret = make_rest_to_rest(np.zeros(6), duration=1.5,
                                  speed_scale=speed_scale,
                                  start_pose=end_pose)

    result_fwd = check_feasibility(traj_fwd, geom, params)
    result_ret = check_feasibility(traj_ret, geom, params)
    print_feasibility('Forward', result_fwd)
    print_feasibility('Return', result_ret)
    if not result_fwd.feasible or not result_ret.feasible:
        print("  FAIL: Trajectory infeasible")
        return False

    interactive_pause("Press Enter to execute T5...")

    # --- Run A: Gravity-only ---
    print("\n  --- Run A: Gravity-only feedforward ---")
    prepare_harness(harness)
    move_to_home(harness, geom, params)
    switch_to_passthrough(harness)

    print(f"  Executing forward (gravity-only, {traj_fwd.duration:.2f}s)...")
    log_a = execute_trajectory_gravity_only(harness, traj_fwd, geom, params)
    analysis_a = analyze_log(log_a)
    print_analysis('A: Forward (gravity-only)', analysis_a)

    print(f"  Executing return (gravity-only, {traj_ret.duration:.2f}s)...")
    log_a_ret = execute_trajectory_gravity_only(harness, traj_ret, geom, params)
    analysis_a_ret = analyze_log(log_a_ret)
    print_analysis('A: Return (gravity-only)', analysis_a_ret)

    # --- Run B: Full feedforward ---
    print("\n  --- Run B: Full feedforward ---")
    move_to_home(harness, geom, params)
    switch_to_passthrough(harness)

    print(f"  Executing forward (full FF, {traj_fwd.duration:.2f}s)...")
    log_b = execute_trajectory(harness, traj_fwd, geom, params)
    analysis_b = analyze_log(log_b)
    print_analysis('B: Forward (full FF)', analysis_b)

    print(f"  Executing return (full FF, {traj_ret.duration:.2f}s)...")
    log_b_ret = execute_trajectory(harness, traj_ret, geom, params)
    analysis_b_ret = analyze_log(log_b_ret)
    print_analysis('B: Return (full FF)', analysis_b_ret)

    harness.idle_all()

    # Compare
    worst_a = max(analysis_a['max_error_mm'], analysis_a_ret['max_error_mm'])
    worst_b = max(analysis_b['max_error_mm'], analysis_b_ret['max_error_mm'])
    improvement = (worst_a - worst_b) / worst_a * 100 if worst_a > 0 else 0

    print(f"\n  --- Comparison ---")
    print(f"    Gravity-only worst error: {worst_a:.3f} mm")
    print(f"    Full FF worst error:      {worst_b:.3f} mm")
    print(f"    Improvement:              {improvement:+.1f}%")

    # The full FF should not be worse than gravity-only
    pass_improvement = worst_b <= worst_a * 1.05  # 5% tolerance for noise
    threshold = TRACKING_THRESHOLDS_MM.get(speed_scale, 3.0)
    pass_tracking = worst_b < threshold

    print(f"\n  Results:")
    print(f"    FF not worse:    {worst_b:.3f} <= {worst_a*1.05:.3f} mm  "
          f"[{'PASS' if pass_improvement else 'FAIL'}]")
    print(f"    Tracking < {threshold} mm:  {worst_b:.3f} mm  "
          f"[{'PASS' if pass_tracking else 'FAIL'}]")

    all_pass = pass_improvement and pass_tracking
    print(f"\n  {'PASS' if all_pass else 'FAIL'}: T5 Feedforward comparison")
    return all_pass


# ---------------------------------------------------------------------------
# Test T6: Trajectory replay at speed
# ---------------------------------------------------------------------------

def test_replay(harness: PlatformTestHarness,
                speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """T6: Re-run Phase 4 trajectory moves at the current speed level.

    Executes T1-T3 equivalent moves with full feedforward at the given speed.

    Pass criteria (speed-dependent):
      50%:  <=2.0 mm peak tracking error
      75%:  <=2.5 mm peak tracking error
      100%: <=3.0 mm peak tracking error
    """
    print("\n" + "=" * 60)
    print(f"T6: Trajectory replay (scale={speed_scale})")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    zeros6 = np.zeros(6)

    threshold = TRACKING_THRESHOLDS_MM.get(speed_scale, 3.0)
    print(f"  Tracking threshold: {threshold} mm (for scale={speed_scale})")

    rx5 = np.deg2rad(5)
    rx3 = np.deg2rad(3)
    ry2 = np.deg2rad(-2)

    # Define moves (subset of Phase 4 T1-T3)
    moves = [
        ('10mm Z',           np.array([0, 0, 10, 0, 0, 0], dtype=float), 1.0),
        ('20mm Z',           np.array([0, 0, 20, 0, 0, 0], dtype=float), 1.0),
        ('5 deg tilt X',     np.array([0, 0, 0, rx5, 0, 0], dtype=float), 1.0),
        ('20mm Z + 5deg X',  np.array([0, 0, 20, rx5, 0, 0], dtype=float), 1.5),
        ('40mm Z',           np.array([0, 0, 40, 0, 0, 0], dtype=float), 1.5),
        ('30mm Z',           np.array([0, 0, 30, 0, 0, 0], dtype=float), 1.5),
        ('20,-10,20mm',      np.array([20, -10, 20, 0, 0, 0], dtype=float), 1.5),
        ('-15,15,10 + 3X',   np.array([-15, 15, 10, rx3, 0, 0], dtype=float), 1.5),
        ('50mm + 3X -2Y',    np.array([0, 0, 50, rx3, ry2, 0], dtype=float), 1.5),
    ]

    # Pre-flight all
    print("\n  Pre-flight checks:")
    all_feasible = True
    move_trajs = []
    for label, end_pose, dur in moves:
        traj_fwd = make_rest_to_rest(end_pose, dur, speed_scale)
        traj_ret = make_rest_to_rest(zeros6, dur, speed_scale,
                                      start_pose=end_pose)
        res_fwd = check_feasibility(traj_fwd, geom, params)
        res_ret = check_feasibility(traj_ret, geom, params)
        fwd_ok = res_fwd.feasible
        ret_ok = res_ret.feasible
        peak_vel = np.max(res_fwd.peak_leg_vel_rps)
        print(f"  [{'OK' if fwd_ok and ret_ok else 'FAIL'}] {label}: "
              f"dur={traj_fwd.duration:.2f}s  peak_vel={peak_vel:.3f} rev/s")
        if not fwd_ok or not ret_ok:
            all_feasible = False
            if not fwd_ok:
                for v in res_fwd.violations:
                    print(f"        {v}")
        else:
            move_trajs.append((label, end_pose, traj_fwd, traj_ret))

    if not all_feasible:
        # Only warn — skip infeasible moves but continue with feasible ones
        print(f"  WARNING: Some trajectories infeasible at scale={speed_scale}")
        print(f"  Continuing with {len(move_trajs)}/{len(moves)} feasible moves")

    if not move_trajs:
        print("  FAIL: No feasible trajectories at this speed")
        return False

    interactive_pause("Press Enter to execute T6...")

    prepare_harness(harness)
    all_pass = True
    worst_overall = 0.0

    for label, end_pose, traj_fwd, traj_ret in move_trajs:
        print(f"\n  --- {label} ---")

        move_to_home(harness, geom, params)
        switch_to_passthrough(harness)

        # Forward
        print(f"  Executing forward ({traj_fwd.duration:.2f}s)...")
        log_fwd = execute_trajectory(harness, traj_fwd, geom, params,
                                     hold_s=1.0)
        analysis_fwd = analyze_log(log_fwd)
        print_analysis(f'{label} forward', analysis_fwd)

        # Return
        print(f"  Executing return ({traj_ret.duration:.2f}s)...")
        log_ret = execute_trajectory(harness, traj_ret, geom, params,
                                     hold_s=1.0)
        analysis_ret = analyze_log(log_ret)
        print_analysis(f'{label} return', analysis_ret)

        worst = max(analysis_fwd['max_error_mm'], analysis_ret['max_error_mm'])
        worst_overall = max(worst_overall, worst)
        ok = worst < threshold
        print(f"  {label}: worst = {worst:.3f} mm "
              f"[{'PASS' if ok else 'FAIL'}]")
        if not ok:
            all_pass = False

    harness.idle_all()

    print(f"\n  --- Summary ---")
    print(f"    Worst tracking: {worst_overall:.3f} mm  "
          f"(threshold: {threshold} mm)")
    print(f"    Moves tested:   {len(move_trajs)}/{len(moves)}")
    print(f"\n  {'PASS' if all_pass else 'FAIL'}: T6 Trajectory replay "
          f"(scale={speed_scale})")
    return all_pass


# ---------------------------------------------------------------------------
# Test T7: Feedforward prediction validation
# ---------------------------------------------------------------------------

def test_ff_prediction(harness: PlatformTestHarness,
                       speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """T7: Validate feedforward prediction via differential iq comparison.

    Runs the same trajectory twice — once with gravity-only feedforward,
    once with full feedforward — both logging iq_measured.  Compares the
    RMS iq between runs.

    Rationale: Directly comparing iq_measured vs torque_ff/Kt is dominated
    by stiction/cogging torque that is not (and should not be) modelled in
    the dynamics.  The differential approach cancels stiction out, isolating
    the effect of the inertia feedforward on PID effort.

    Pass criteria:
      - Full FF must not *increase* RMS iq vs gravity-only (per leg).
        A small tolerance (10%) accounts for run-to-run variation.
      - Both runs must track within the speed-dependent threshold.
      - Diagnostic: report the iq reduction from the inertia terms.
    """
    print("\n" + "=" * 60)
    print(f"T7: Feedforward prediction validation (scale={speed_scale})")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    # Use a move that exercises all DoFs
    rx3 = np.deg2rad(3)
    end_pose = np.array([15, -10, 30, rx3, 0, 0], dtype=float)

    traj_fwd = make_rest_to_rest(end_pose, duration=1.5,
                                  speed_scale=speed_scale)
    result = check_feasibility(traj_fwd, geom, params)
    print_feasibility('Forward', result)
    if not result.feasible:
        print("  FAIL: Trajectory infeasible")
        return False

    interactive_pause("Press Enter to execute T7...")

    # --- Run A: Gravity-only with iq logging ---
    print("\n  --- Run A: Gravity-only feedforward (with iq logging) ---")
    prepare_harness(harness)
    move_to_home(harness, geom, params)
    switch_to_passthrough(harness)

    print(f"  Executing (gravity-only, {traj_fwd.duration:.2f}s)...")
    log_a = execute_trajectory_with_iq_gravity_only(
        harness, traj_fwd, geom, params)
    analysis_a = analyze_log(log_a)
    print_analysis('A: Gravity-only', analysis_a)

    # --- Run B: Full feedforward with iq logging ---
    print("\n  --- Run B: Full feedforward (with iq logging) ---")
    move_to_home(harness, geom, params)
    switch_to_passthrough(harness)

    print(f"  Executing (full FF, {traj_fwd.duration:.2f}s)...")
    log_b = execute_trajectory_with_iq(harness, traj_fwd, geom, params)
    analysis_b = analyze_log(log_b)
    print_analysis('B: Full FF', analysis_b)

    harness.idle_all()

    # --- Differential iq analysis ---
    # Align to the motion portion only (skip first 50ms and hold phase)
    t_start = 0.05
    t_end = traj_fwd.duration

    ts_a = np.array(log_a.timestamps)
    ts_b = np.array(log_b.timestamps)
    mask_a = (ts_a >= t_start) & (ts_a <= t_end)
    mask_b = (ts_b >= t_start) & (ts_b <= t_end)

    if np.sum(mask_a) < 10 or np.sum(mask_b) < 10:
        print("  WARNING: Too few samples in motion period")
        print("  FAIL: T7 insufficient data")
        return False

    iq_a = np.array(log_a.iq_measured)[mask_a]   # (N_a, 6)
    iq_b = np.array(log_b.iq_measured)[mask_b]   # (N_b, 6)
    ff_a = np.array(log_a.commanded_torque_Nm)[mask_a]  # gravity-only torques
    ff_b = np.array(log_b.commanded_torque_Nm)[mask_b]  # full FF torques

    # RMS iq per leg for each run
    rms_iq_a = np.sqrt(np.mean(iq_a ** 2, axis=0))  # (6,)
    rms_iq_b = np.sqrt(np.mean(iq_b ** 2, axis=0))  # (6,)

    # RMS feedforward current for context
    rms_ff_a = np.sqrt(np.mean((ff_a / MOTOR_KT) ** 2, axis=0))
    rms_ff_b = np.sqrt(np.mean((ff_b / MOTOR_KT) ** 2, axis=0))

    # Delta: negative = full FF reduced PID effort (good)
    iq_delta = rms_iq_b - rms_iq_a  # (6,)
    iq_delta_pct = np.zeros(6)
    for i in range(6):
        if rms_iq_a[i] > 0.05:
            iq_delta_pct[i] = iq_delta[i] / rms_iq_a[i] * 100

    print(f"\n  --- Differential iq Analysis ---")
    print(f"    Motor Kt:        {MOTOR_KT:.4f} Nm/A")
    print(f"    Analysis window: {t_start:.3f}s to {t_end:.3f}s")
    print(f"    Samples:         A={np.sum(mask_a)}, B={np.sum(mask_b)}")
    print()
    print(f"    {'Leg':>4s}  {'RMS iq A':>9s}  {'RMS iq B':>9s}  "
          f"{'Delta':>8s}  {'Delta%':>8s}  "
          f"{'FF A (A)':>9s}  {'FF B (A)':>9s}")
    print(f"    {'----':>4s}  {'---------':>9s}  {'---------':>9s}  "
          f"{'--------':>8s}  {'------':>8s}  "
          f"{'---------':>9s}  {'---------':>9s}")
    for i in range(6):
        print(f"    {i:4d}  {rms_iq_a[i]:9.4f}  {rms_iq_b[i]:9.4f}  "
              f"{iq_delta[i]:+8.4f}  {iq_delta_pct[i]:+7.1f}%  "
              f"{rms_ff_a[i]:9.4f}  {rms_ff_b[i]:9.4f}")

    # Aggregate metrics
    mean_delta_pct = float(np.mean(iq_delta_pct))
    worst_increase_pct = float(np.max(iq_delta_pct))

    print()
    print(f"    Mean iq change:     {mean_delta_pct:+.1f}%  "
          f"(negative = FF reduced PID effort)")
    print(f"    Worst leg increase: {worst_increase_pct:+.1f}%")

    # Pass criteria
    threshold = TRACKING_THRESHOLDS_MM.get(speed_scale, 3.0)
    pass_tracking_a = analysis_a['max_error_mm'] < threshold
    pass_tracking_b = analysis_b['max_error_mm'] < threshold

    # Full FF must not increase RMS iq by more than 10% on any leg
    # (accounts for run-to-run stiction variation)
    iq_increase_tolerance = 10.0  # percent
    pass_iq = worst_increase_pct < iq_increase_tolerance

    print(f"\n  Results:")
    print(f"    Tracking A:      {analysis_a['max_error_mm']:.3f} mm  "
          f"(threshold: {threshold} mm)  "
          f"[{'PASS' if pass_tracking_a else 'FAIL'}]")
    print(f"    Tracking B:      {analysis_b['max_error_mm']:.3f} mm  "
          f"(threshold: {threshold} mm)  "
          f"[{'PASS' if pass_tracking_b else 'FAIL'}]")
    print(f"    iq not worse:    {worst_increase_pct:+.1f}%  "
          f"(tolerance: +{iq_increase_tolerance:.0f}%)  "
          f"[{'PASS' if pass_iq else 'FAIL'}]")

    if not pass_iq:
        print(f"\n  NOTE: Full FF increased PID effort. Check:")
        print(f"    - Sign of torque_ff (should oppose gravity)")
        print(f"    - Inertia wrench sign convention")
        print(f"    - Motor Kt ({MOTOR_KT} Nm/A) vs ODrive torque_constant")

    all_pass = pass_tracking_a and pass_tracking_b and pass_iq
    print(f"\n  {'PASS' if all_pass else 'FAIL'}: T7 Feedforward prediction "
          f"(scale={speed_scale})")
    return all_pass


# ---------------------------------------------------------------------------
# Test registry and CLI
# ---------------------------------------------------------------------------

TESTS = {
    'ff_comparison':  ('T5: Feedforward comparison (gravity vs full)',
                       test_ff_comparison),
    'replay':         ('T6: Trajectory replay at speed',
                       test_replay),
    'ff_prediction':  ('T7: Feedforward prediction (differential iq)',
                       test_ff_prediction),
}

TEST_GROUPS = {
    'all': ['ff_comparison', 'replay', 'ff_prediction'],
}


def run_dry_run(speed_scale: float, show_preview: bool):
    """Run feasibility checks on Phase 5 test trajectories without CAN."""
    print("\n" + "=" * 60)
    print("DRY RUN: Phase 5 Feasibility checks (no hardware)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    threshold = TRACKING_THRESHOLDS_MM.get(speed_scale, 3.0)
    print(f"\n  Speed scale: {speed_scale}")
    print(f"  Tracking threshold: {threshold} mm")
    print()

    trajs = build_phase5_test_trajectories(geom, speed_scale=speed_scale)

    all_ok = True
    for label, traj in trajs:
        result = check_feasibility(traj, geom, params)
        status = 'OK' if result.feasible else 'FAIL'
        peak_vel = np.max(result.peak_leg_vel_rps)
        peak_torque = np.max(np.abs(result.peak_torque_Nm)) if hasattr(
            result, 'peak_torque_Nm') else 0.0
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
        try:
            from trajectory_viewer import preview_test_sequence
            print("\n  Launching 3D preview (close window to continue)...")
            preview_test_sequence(trajs, geom, params)
        except ImportError:
            print("  WARNING: matplotlib not available, skipping preview")


def build_phase5_test_trajectories(geom: StewartGeometry = None,
                                    speed_scale: float = 0.5):
    """Build the Phase 5 hardware test trajectories for preview/dry-run.

    Returns a list of (label, QuinticTrajectory) tuples.
    """
    geom = geom or StewartGeometry()
    zeros6 = np.zeros(6)

    def rest_to_rest(end_pose, duration, start_pose=None):
        sp = np.asarray(start_pose, dtype=float) if start_pose is not None \
            else zeros6.copy()
        ep = np.asarray(end_pose, dtype=float)
        return create_trajectory(
            start_pose=sp, start_twist=zeros6, start_accel=zeros6,
            end_pose=ep, end_twist=zeros6, end_accel=zeros6,
            duration=duration, t_start=0.0, speed_scale=speed_scale)

    trajs = []

    rx3 = np.deg2rad(3)
    rx5 = np.deg2rad(5)
    ry2 = np.deg2rad(-2)

    # --- T5: Feedforward comparison move ---
    t5_end = np.array([0, 0, 30, rx3, 0, 0], dtype=float)
    trajs.append(('T5: Home -> 30mm Z + 3deg X',
                  rest_to_rest(t5_end, 1.5)))
    trajs.append(('T5: 30mm Z + 3deg X -> Home',
                  rest_to_rest(zeros6, 1.5, start_pose=t5_end)))

    # --- T6: Trajectory replay moves ---
    replay_moves = [
        ('T6: 10mm Z',      np.array([0, 0, 10, 0, 0, 0], dtype=float), 1.0),
        ('T6: 20mm Z',      np.array([0, 0, 20, 0, 0, 0], dtype=float), 1.0),
        ('T6: 5deg X',      np.array([0, 0, 0, rx5, 0, 0], dtype=float), 1.0),
        ('T6: 20mm+5deg',   np.array([0, 0, 20, rx5, 0, 0], dtype=float), 1.5),
        ('T6: 40mm Z',      np.array([0, 0, 40, 0, 0, 0], dtype=float), 1.5),
        ('T6: 30mm Z',      np.array([0, 0, 30, 0, 0, 0], dtype=float), 1.5),
        ('T6: 20,-10,20',   np.array([20, -10, 20, 0, 0, 0], dtype=float), 1.5),
        ('T6: -15,15,10+3X', np.array([-15, 15, 10, rx3, 0, 0], dtype=float), 1.5),
        ('T6: 50mm+3X-2Y',  np.array([0, 0, 50, rx3, ry2, 0], dtype=float), 1.5),
    ]
    for label, end_pose, dur in replay_moves:
        trajs.append((f'{label} fwd', rest_to_rest(end_pose, dur)))
        trajs.append((f'{label} ret', rest_to_rest(zeros6, dur,
                                                     start_pose=end_pose)))

    # --- T7: Feedforward prediction move ---
    t7_end = np.array([15, -10, 30, rx3, 0, 0], dtype=float)
    trajs.append(('T7: Home -> 15,-10,30mm + 3deg X',
                  rest_to_rest(t7_end, 1.5)))
    trajs.append(('T7: 15,-10,30mm + 3deg X -> Home',
                  rest_to_rest(zeros6, 1.5, start_pose=t7_end)))

    return trajs


def main():
    parser = argparse.ArgumentParser(
        description='Inertia feedforward test harness for Jugglebot Phase 5',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests available:
  ff_comparison   T5: Gravity-only vs full feedforward comparison
  replay          T6: Trajectory replay at speed (Phase 4 moves)
  ff_prediction   T7: Differential iq comparison (gravity vs full FF)

Test groups:
  all             Run T5-T7 in order [DEFAULT]

Graduated speed ramp:
  Run at 25%% first (baseline), then increase:
    --speed-scale 0.25    (Phase 4 baseline)
    --speed-scale 0.50    (Phase 5 first target)
    --speed-scale 0.75    (Phase 5 second target)
    --speed-scale 1.00    (full speed — final validation)

  Each speed level must pass before proceeding to the next.

Modes:
  --dry-run         Feasibility checks only, no CAN connection
  --preview         Show 3D viewer before hardware execution
  --dry-run --preview    Feasibility + 3D viewer, no hardware

Prerequisites:
  - All Phase 4 tests must PASS first
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
    parser.add_argument('--speed-scale', type=float, default=0.5,
                        help='Speed scale factor (default: 0.5)')

    args = parser.parse_args()

    # Dry-run mode
    if args.dry_run:
        run_dry_run(args.speed_scale, args.preview)
        return 0

    # Determine which tests to run
    if args.test in TEST_GROUPS:
        tests_to_run = TEST_GROUPS[args.test]
    else:
        tests_to_run = [args.test]

    # Optional 3D preview
    if args.preview:
        try:
            from trajectory_viewer import preview_test_sequence
            geom = StewartGeometry()
            params = DynamicsParams.from_config()
            trajs = build_phase5_test_trajectories(geom,
                                                    speed_scale=args.speed_scale)
            print("\n  Launching 3D preview (close window to continue)...")
            preview_test_sequence(trajs, geom, params)
        except ImportError:
            print("  WARNING: matplotlib not available, skipping preview")

    # Install Ctrl-C handler
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

    print(f"\nJugglebot Inertia Feedforward Test Harness (Phase 5)")
    print(f"  Interface:     {args.interface}")
    print(f"  Channel:       {args.channel}")
    print(f"  Speed scale:   {args.speed_scale}")
    print(f"  Current limit: {SAFE_CURRENT_LIMIT_A} A "
          f"(50% of {hw.ODRIVE_LEG_CURR_LIMIT_A} A)")
    print(f"\n  *** Platform must be free-standing ***")
    print(f"  *** All Phase 4 tests must PASS first ***")

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
    print(f"  Speed scale: {args.speed_scale}")
    threshold = TRACKING_THRESHOLDS_MM.get(args.speed_scale, 3.0)
    print(f"  Tracking threshold: {threshold} mm")
    print()

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

    if n_pass == n_total:
        next_speed = {0.25: 0.5, 0.5: 0.75, 0.75: 1.0}
        ns = next_speed.get(args.speed_scale)
        if ns is not None:
            print(f"\n  All passed! Next: run at --speed-scale {ns}")
        else:
            print(f"\n  All passed at full speed! Phase 5 COMPLETE.")
    else:
        print(f"\n  Fix failures before increasing speed scale.")

    return 0 if n_pass == n_total else 1


if __name__ == '__main__':
    sys.exit(main())
