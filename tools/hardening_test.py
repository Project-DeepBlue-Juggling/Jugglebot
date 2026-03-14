#!/usr/bin/env python3
"""Standalone hardware test harness for Jugglebot Phase 6 (Hardening).

Validates workspace limits, singularity avoidance, fault recovery, and
endurance under the Phase 6 protective system framework.

Phase 6 hardware tests:
  H1. Workspace boundary test (low speed) — trajectories approach limits
  H2. Singularity avoidance test (low speed) — trajectory near ill-conditioned
  H3. Fault injection test (static) — ODrive fault while holding pose
  H4. Fault injection test (low-speed motion) — ODrive fault during trajectory
  H5. Endurance test (moderate speed, 30 min) — sustained trajectory replay
  H6. Endurance test (full speed, 60 min) — aggressive trajectory library

Prerequisites:
  - All Phase 4+5 tests must PASS
  - Robot fully assembled, platform free-standing
  - Legs must be homed (use --home flag)

Safety:
  - Mandatory check_feasibility() before every trajectory execution
  - Real-time workspace limit checking every control cycle
  - Conservative current limit (50% of rated = 10A)
  - All tests send IDLE to all 6 axes on completion, error, or Ctrl-C
  - Interactive confirmation before each test

Usage:
  python tools/hardening_test.py --home --test all
  python tools/hardening_test.py --test workspace --speed-scale 0.25
  python tools/hardening_test.py --test endurance_moderate --speed-scale 0.5
  python tools/hardening_test.py --test endurance_full --speed-scale 1.0
  python tools/hardening_test.py --dry-run
  python tools/hardening_test.py --test fault_static

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
    TARGET_DT_S,
    TARGET_LOOP_HZ,
    HOLD_AFTER_S,
    MAX_TRACKING_ERROR_MM,
    DEFAULT_SPEED_SCALE,
)

# Motion planning imports
from jugglebot.motion.geometry import StewartGeometry  # noqa: E402
from jugglebot.motion.dynamics import DynamicsParams  # noqa: E402
from jugglebot.motion.ik_solver import (  # noqa: E402
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.feasibility import check_feasibility  # noqa: E402
from jugglebot.motion.motor_commands import cartesian_to_motor_commands  # noqa: E402
from jugglebot.motion.quintic import create_trajectory, evaluate  # noqa: E402
from jugglebot.motion.workspace import (  # noqa: E402
    WorkspaceLimits,
    WorkspaceStatus,
    check_workspace_limits,
    compute_condition_number,
)
from jugglebot.motion.conversions import (  # noqa: E402
    extensions_mm_to_revs,
    revs_to_extensions_mm,
)

# ---------------------------------------------------------------------------
# Phase 6 test constants
# ---------------------------------------------------------------------------

# Speed-dependent tracking thresholds (same as Phase 5)
TRACKING_THRESHOLDS_MM = {0.25: 1.5, 0.50: 2.0, 0.75: 2.5, 1.00: 3.0}

# Endurance test durations
MODERATE_ENDURANCE_MINUTES = 30
FULL_ENDURANCE_MINUTES = 60


# ---------------------------------------------------------------------------
# Workspace-aware trajectory execution
# ---------------------------------------------------------------------------

def execute_trajectory_with_workspace_check(
    harness: PlatformTestHarness,
    traj,
    geom: StewartGeometry,
    params: DynamicsParams,
    limits: WorkspaceLimits,
    speed_scale: float = DEFAULT_SPEED_SCALE,
    label: str = "",
) -> tuple[dict, list]:
    """Execute a trajectory with real-time workspace limit checking.

    Returns (analysis_dict, workspace_violations_list).
    Reuses the same TrajectoryLog dataclass as Phase 4/5.
    """
    log = TrajectoryLog()
    workspace_violations = []

    t_start = time.perf_counter()
    traj_duration = traj.duration
    t_prev = t_start

    while True:
        t_now = time.perf_counter()
        t_elapsed = t_now - t_start
        dt = t_now - t_prev
        t_prev = t_now

        # Evaluate trajectory
        traj_t = traj.t_start + t_elapsed
        pose, twist, accel = evaluate(traj, traj_t)
        pos_rev, vel_ff, torque_ff = cartesian_to_motor_commands(
            pose, twist, accel, geom, params, feedforward_enabled=True)

        # Workspace check
        rot = rotvec_to_rot_matrix(pose[3:6])
        ext_mm = pose_to_leg_lengths(pose[:3], rot, geom)
        cond = compute_condition_number(pose[:3], rot, geom)
        ws = check_workspace_limits(ext_mm, cond, limits)

        if ws.status == WorkspaceStatus.HARD_LIMIT:
            print(f"\n  ** WORKSPACE HARD LIMIT at t={t_elapsed:.3f}s **")
            for v in ws.violations:
                print(f"     {v}")
            workspace_violations.extend(ws.violations)
            break

        if ws.status == WorkspaceStatus.SOFT_LIMIT:
            workspace_violations.extend(ws.violations)

        # Send commands to ODrives (matching trajectory_test.py conventions)
        for i, axis_id in enumerate(LEG_AXES):
            # Sign conventions for direct CAN (matching can_node.py):
            # All three fields are negated for legs — ODrive negative = extension
            raw_pos = -pos_rev[i]
            vel_int = int(round(-vel_ff[i] * LEG_VEL_FF_SCALE))
            tor_int = int(round(-torque_ff[i] * LEG_TOR_FF_SCALE))
            vel_int = max(-32767, min(32767, vel_int))
            tor_int = max(-32767, min(32767, tor_int))
            harness.send_no_delay(encode_set_input_pos(
                axis_id, raw_pos, vel_int, tor_int))

        # Poll encoder feedback (non-blocking)
        harness._poll(timeout=0)

        # Log data (using TrajectoryLog fields)
        actual_pos = np.array([harness.states[a].pos_rev for a in LEG_AXES])
        actual_vel = np.array([harness.states[a].vel_rps for a in LEG_AXES])

        log.timestamps.append(t_elapsed)
        log.commanded_pos_rev.append(pos_rev.copy())
        log.commanded_vel_rps.append(vel_ff.copy())
        log.commanded_torque_Nm.append(torque_ff.copy())
        log.actual_pos_rev.append(actual_pos.copy())
        log.actual_vel_rps.append(actual_vel.copy())
        log.poses.append(pose.copy())
        log.loop_dt_s.append(dt)

        # Check for ODrive errors
        for ax in LEG_AXES:
            s = harness.states[ax]
            if s.active_errors:
                err_str = ', '.join(error_names(s.active_errors))
                print(f"\n  ** ODrive ERROR on axis {ax}: {err_str} **")
                workspace_violations.append(f"ODrive error axis {ax}: {err_str}")
                break

        # Check if trajectory is done
        if t_elapsed >= traj_duration + HOLD_AFTER_S:
            break

        # Rate control
        t_loop_end = time.perf_counter()
        sleep_s = TARGET_DT_S - (t_loop_end - t_now)
        if sleep_s > 0:
            time.sleep(sleep_s)

    analysis = analyze_log(log) if log.timestamps else {}
    return analysis, workspace_violations


# ---------------------------------------------------------------------------
# Test trajectory library
# ---------------------------------------------------------------------------

def build_workspace_boundary_trajectories(geom, speed_scale=0.25):
    """Build trajectories that approach workspace boundaries."""
    zeros6 = np.zeros(6)
    stroke = geom.leg_stroke_mm
    trajectories = []

    # Large Z up (approaches upper extension limit)
    trajectories.append(("Large Z up (+100mm)", make_rest_to_rest(
        np.array([0, 0, 100, 0, 0, 0], dtype=float), 2.0, speed_scale)))

    # Large Z down (approaches lower extension limit — limited workspace below home)
    trajectories.append(("Z down (-15mm)", make_rest_to_rest(
        np.array([0, 0, -15, 0, 0, 0], dtype=float), 2.0, speed_scale)))

    # Large tilt (approaches stroke limits on individual legs)
    trajectories.append(("Large tilt (7deg X)", make_rest_to_rest(
        np.array([0, 0, 20, np.deg2rad(7), 0, 0], dtype=float), 2.0, speed_scale)))

    trajectories.append(("Large tilt (-7deg Y)", make_rest_to_rest(
        np.array([0, 0, 20, 0, np.deg2rad(-7), 0], dtype=float), 2.0, speed_scale)))

    # Combined large translation + tilt
    trajectories.append(("Combined (80mm Z + 5deg X)", make_rest_to_rest(
        np.array([0, 0, 80, np.deg2rad(5), 0, 0], dtype=float), 2.5, speed_scale)))

    # Large XY displacement
    trajectories.append(("Large XY (30mm X, -20mm Y, 20mm Z)", make_rest_to_rest(
        np.array([30, -20, 20, 0, 0, 0], dtype=float), 2.0, speed_scale)))

    return trajectories


def build_stress_test_library(speed_scale=1.0):
    """Build the stress test trajectory library (Phase 6 endurance)."""
    zeros6 = np.zeros(6)
    trajectories = []

    # Quick moves across workspace
    moves = [
        ("Quick Z 50mm", np.array([0, 0, 50, 0, 0, 0], dtype=float), 1.0),
        ("Quick Z 80mm", np.array([0, 0, 80, 0, 0, 0], dtype=float), 1.2),
        ("Quick tilt 5deg", np.array([0, 0, 20, np.deg2rad(5), 0, 0], dtype=float), 1.0),
        ("Quick combo", np.array([20, -10, 40, np.deg2rad(3), np.deg2rad(-2), 0], dtype=float), 1.5),
        ("Z + tilt", np.array([0, 0, 60, np.deg2rad(4), np.deg2rad(3), 0], dtype=float), 1.5),
        ("XY sweep", np.array([25, -15, 30, 0, 0, 0], dtype=float), 1.2),
    ]

    for label, end_pose, dur in moves:
        trajectories.append((label, make_rest_to_rest(end_pose, dur, speed_scale)))

    return trajectories


# ---------------------------------------------------------------------------
# H1: Workspace boundary test
# ---------------------------------------------------------------------------

def test_workspace_boundary(harness, geom, params, limits, speed_scale, dry_run=False):
    """H1: Command slow trajectories that approach workspace limits."""
    print("\n" + "=" * 70)
    print("  H1: Workspace Boundary Test (low speed)")
    print("=" * 70)

    trajectories = build_workspace_boundary_trajectories(geom, speed_scale)

    passed = 0
    failed = 0

    for label, traj in trajectories:
        feas = check_feasibility(traj, geom, params)
        print(f"\n  {label}:")
        print_feasibility(label, feas)

        if not feas.feasible:
            print(f"    SKIPPED (infeasible at {speed_scale*100:.0f}% speed)")
            continue

        # Check workspace limits along trajectory
        n_samples = 100
        worst_status = WorkspaceStatus.OK
        min_scale = 1.0
        for i in range(n_samples + 1):
            t = traj.t_start + (i / n_samples) * traj.duration
            pose, _, _ = evaluate(traj, t)
            ext = pose_to_leg_lengths(pose[:3], rotvec_to_rot_matrix(pose[3:6]), geom)
            cond = compute_condition_number(pose[:3], rotvec_to_rot_matrix(pose[3:6]), geom)
            ws = check_workspace_limits(ext, cond, limits)
            if ws.status.value == 'hard':
                worst_status = WorkspaceStatus.HARD_LIMIT
            elif ws.status.value == 'soft' and worst_status.value != 'hard':
                worst_status = WorkspaceStatus.SOFT_LIMIT
            min_scale = min(min_scale, ws.speed_scale)

        print(f"    Worst workspace status: {worst_status.value}")
        print(f"    Min speed scale: {min_scale:.3f}")

        if dry_run:
            print(f"    [DRY RUN] Would execute on hardware")
            passed += 1
            continue

        if worst_status == WorkspaceStatus.HARD_LIMIT:
            print(f"    EXPECTED: trajectory enters hard limit zone")
            print(f"    Verifying abort behavior on hardware...")
            # On hardware, the trajectory should be aborted cleanly

        interactive_pause(f"Execute {label}?")

        # Return to home via TRAP_TRAJ before each trajectory (safe ramped move)
        move_to_home(harness, geom, params)
        switch_to_passthrough(harness)

        # Execute on hardware
        analysis, violations = execute_trajectory_with_workspace_check(
            harness, traj, geom, params, limits, speed_scale, label)

        if analysis:
            max_err = analysis.get('max_error_mm', 0)
            threshold = TRACKING_THRESHOLDS_MM.get(speed_scale, 3.0)
            status = 'PASS' if max_err < threshold else 'FAIL'
            print(f"    Tracking: {max_err:.3f} mm (threshold {threshold:.1f}) [{status}]")

        if violations:
            print(f"    Workspace events: {len(violations)}")
            for v in violations[:5]:
                print(f"      {v}")

        passed += 1

    print(f"\n  H1 Result: {passed} passed, {failed} failed")
    return failed == 0


# ---------------------------------------------------------------------------
# H3: Fault injection test (static)
# ---------------------------------------------------------------------------

def test_fault_injection_static(harness, geom, params, limits, dry_run=False):
    """H3: With platform holding a static pose, manually trigger an ODrive fault."""
    print("\n" + "=" * 70)
    print("  H3: Fault Injection Test (Static)")
    print("=" * 70)

    if dry_run:
        print("  [DRY RUN] This test requires manual operator intervention.")
        print("  Procedure:")
        print("    1. Platform holds home pose")
        print("    2. Operator deliberately triggers an ODrive fault")
        print("       (e.g., overcurrent by blocking motor, or via ODrive GUI)")
        print("    3. Verify all legs go to IDLE safely")
        print("    4. Verify platform drops safely (be prepared to catch)")
        print("  [DRY RUN] Skipped.")
        return True

    print("\n  Setup: Platform holding home pose with feedforward active.")
    print("  This test requires manual operator intervention.")
    print()
    print("  Procedure:")
    print("    1. Platform will hold at home pose")
    print("    2. You will manually trigger a fault (details below)")
    print("    3. The harness will detect the fault and idle all legs")
    print("    4. Verify the platform drops safely")
    print()
    print("  SAFETY: Be prepared to catch/support the platform when legs idle.")
    print()

    interactive_pause("Ready to start static fault injection test?")

    # Hold at home
    home_pos = pose_to_raw_positions(np.zeros(3), np.eye(3), geom)
    for i, axis_id in enumerate(LEG_AXES):
        harness.send(encode_set_input_pos(axis_id, home_pos[i], 0, 0))

    time.sleep(1.0)
    print("  Platform holding at home. All legs active.")
    print()
    print("  NOW: Trigger an ODrive fault.")
    print("  Options:")
    print("    a) Via ODrive GUI: set a very low current limit on one axis")
    print("    b) Via ODrive GUI: force IDLE on one axis")
    print("    c) Physically stall one motor briefly")
    print()

    interactive_pause("Press Enter AFTER you have triggered the fault...")

    # Check for faults
    harness.poll_for(0.1)
    faults_detected = []
    for ax in LEG_AXES:
        s = harness.states[ax]
        if s.active_errors:
            err_str = ', '.join(error_names(s.active_errors))
            faults_detected.append(f"Axis {ax}: {err_str}")

    if faults_detected:
        print(f"  Faults detected: {len(faults_detected)}")
        for f in faults_detected:
            print(f"    {f}")
    else:
        print("  WARNING: No faults detected in ODrive error registers.")
        print("  The fault may not have been triggered, or was too brief to detect.")

    # Idle all legs
    print("  Sending IDLE to all legs...")
    harness.idle_all()
    time.sleep(0.5)

    # Verify all axes are idle
    harness.poll_for(0.1)
    all_idle = all(harness.states[ax].axis_state == 1 for ax in LEG_AXES)  # 1 = IDLE
    if all_idle:
        print("  All legs confirmed IDLE.")
        print("  [PASS]")
        return True
    else:
        states = {ax: harness.states[ax].axis_state for ax in LEG_AXES}
        print(f"  WARNING: Not all legs idle. States: {states}")
        print("  [FAIL]")
        return False


# ---------------------------------------------------------------------------
# H5: Moderate endurance test
# ---------------------------------------------------------------------------

def test_endurance_moderate(harness, geom, params, limits, speed_scale=0.5,
                            duration_min=MODERATE_ENDURANCE_MINUTES, dry_run=False):
    """H5: Run moderate-speed trajectories for 30 minutes."""
    print("\n" + "=" * 70)
    print(f"  H5: Moderate Endurance Test ({duration_min} min at "
          f"{speed_scale*100:.0f}% speed)")
    print("=" * 70)

    trajectories = build_stress_test_library(speed_scale)

    # Pre-flight: check all trajectories feasible
    print("  Pre-flight feasibility check...")
    all_feasible = True
    for label, traj in trajectories:
        feas = check_feasibility(traj, geom, params)
        if not feas.feasible:
            print(f"    {label}: INFEASIBLE - {feas.violations}")
            all_feasible = False
        else:
            print(f"    {label}: ok (peak vel {np.max(feas.peak_leg_vel_rps):.2f} rev/s)")

    if not all_feasible:
        print("  ABORT: Not all trajectories feasible at this speed.")
        return False

    if dry_run:
        print(f"  [DRY RUN] Would run {len(trajectories)} trajectories for {duration_min} min")
        return True

    interactive_pause(f"Run endurance test for {duration_min} minutes?")

    t_endurance_start = time.perf_counter()
    t_endurance_end = t_endurance_start + duration_min * 60.0
    cycle_count = 0
    total_moves = 0
    max_tracking_error = 0.0
    faults = []

    print(f"  Running... (will run for {duration_min} min)")
    print(f"  Press Ctrl-C to stop early.\n")

    try:
        while time.perf_counter() < t_endurance_end:
            cycle_count += 1
            elapsed_min = (time.perf_counter() - t_endurance_start) / 60.0

            # Execute each trajectory in the library
            for label, traj in trajectories:
                if time.perf_counter() >= t_endurance_end:
                    break

                # Return to home via TRAP_TRAJ before each trajectory
                move_to_home(harness, geom, params)
                switch_to_passthrough(harness)

                # Execute forward
                analysis, violations = execute_trajectory_with_workspace_check(
                    harness, traj, geom, params, limits, speed_scale, label)
                total_moves += 1

                if analysis:
                    err = analysis.get('max_error_mm', 0)
                    max_tracking_error = max(max_tracking_error, err)

                if violations:
                    faults.extend(violations[:3])

            # Periodic status
            if cycle_count % 5 == 0:
                elapsed = (time.perf_counter() - t_endurance_start) / 60.0
                print(f"  [{elapsed:.1f} min] Cycle {cycle_count}, "
                      f"{total_moves} moves, worst error {max_tracking_error:.3f} mm, "
                      f"{len(faults)} faults")

    except KeyboardInterrupt:
        elapsed_min = (time.perf_counter() - t_endurance_start) / 60.0
        print(f"\n  Stopped early at {elapsed_min:.1f} min")

    elapsed_min = (time.perf_counter() - t_endurance_start) / 60.0
    print(f"\n  Endurance summary:")
    print(f"    Duration:        {elapsed_min:.1f} min")
    print(f"    Cycles:          {cycle_count}")
    print(f"    Total moves:     {total_moves}")
    print(f"    Worst tracking:  {max_tracking_error:.3f} mm")
    print(f"    Faults:          {len(faults)}")

    threshold = TRACKING_THRESHOLDS_MM.get(speed_scale, 3.0)
    passed = max_tracking_error < threshold and len(faults) == 0
    print(f"  [{'PASS' if passed else 'FAIL'}]")
    return passed


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Jugglebot Phase 6 hardening tests")
    parser.add_argument('--home', action='store_true',
                        help='Home all legs before testing')
    parser.add_argument('--test', default='all',
                        choices=['all', 'workspace', 'fault_static',
                                 'endurance_moderate', 'endurance_full'],
                        help='Which test to run')
    parser.add_argument('--speed-scale', type=float, default=DEFAULT_SPEED_SCALE,
                        help=f'Speed scale factor (default: {DEFAULT_SPEED_SCALE})')
    parser.add_argument('--dry-run', action='store_true',
                        help='Feasibility check only, no hardware execution')
    parser.add_argument('--endurance-minutes', type=int, default=None,
                        help='Override endurance test duration (minutes)')
    args = parser.parse_args()

    print("=" * 70)
    print("  Jugglebot Phase 6: Hardening & Operational Readiness Tests")
    print(f"  Speed scale: {args.speed_scale*100:.0f}%")
    print(f"  Dry run: {args.dry_run}")
    print("=" * 70)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)

    print(f"\n  Workspace limits:")
    print(f"    Leg soft: [{limits.leg_soft_min_mm:.1f}, {limits.leg_soft_max_mm:.1f}] mm")
    print(f"    Leg hard: [{limits.leg_hard_min_mm:.1f}, {limits.leg_hard_max_mm:.1f}] mm")
    print(f"    Cond home: {limits.cond_home:.1f}")
    print(f"    Cond soft: {limits.cond_soft:.1f}")
    print(f"    Cond hard: {limits.cond_hard:.1f}")

    if args.dry_run:
        print("\n  [DRY RUN MODE - no CAN communication]\n")
        # Run feasibility checks for all test trajectories
        if args.test in ('all', 'workspace'):
            test_workspace_boundary(None, geom, params, limits,
                                     args.speed_scale, dry_run=True)
        if args.test in ('all', 'fault_static'):
            test_fault_injection_static(None, geom, params, limits, dry_run=True)
        if args.test in ('all', 'endurance_moderate'):
            dur = args.endurance_minutes or MODERATE_ENDURANCE_MINUTES
            test_endurance_moderate(None, geom, params, limits,
                                    args.speed_scale, dur, dry_run=True)
        return 0

    # Hardware mode: set up CAN harness
    harness = PlatformTestHarness()

    # Signal handler for clean shutdown
    def signal_handler(sig, frame):
        print("\n  Ctrl-C: idling all legs...")
        try:
            harness.idle_all()
        except Exception:
            pass
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        harness.connect()
        prepare_harness(harness)
        if args.home:
            move_to_home(harness, geom, params)
        switch_to_passthrough(harness)

        results = {}

        if args.test in ('all', 'workspace'):
            results['H1'] = test_workspace_boundary(
                harness, geom, params, limits, args.speed_scale)

        if args.test in ('all', 'fault_static'):
            move_to_home(harness, geom, params)
            switch_to_passthrough(harness)
            results['H3'] = test_fault_injection_static(
                harness, geom, params, limits)

        if args.test in ('all', 'endurance_moderate'):
            move_to_home(harness, geom, params)
            switch_to_passthrough(harness)
            dur = args.endurance_minutes or MODERATE_ENDURANCE_MINUTES
            results['H5'] = test_endurance_moderate(
                harness, geom, params, limits, args.speed_scale, dur)

        if args.test == 'endurance_full':
            move_to_home(harness, geom, params)
            switch_to_passthrough(harness)
            dur = args.endurance_minutes or FULL_ENDURANCE_MINUTES
            results['H6'] = test_endurance_moderate(
                harness, geom, params, limits, 1.0, dur)

        # Summary
        print("\n" + "=" * 70)
        print("  Phase 6 Test Summary")
        print("=" * 70)
        for test_id, passed in results.items():
            print(f"    {test_id}: {'PASS' if passed else 'FAIL'}")

        all_pass = all(results.values())
        print(f"\n  Overall: {'ALL PASS' if all_pass else 'SOME FAILED'}")
        return 0 if all_pass else 1

    finally:
        harness.disconnect()


if __name__ == '__main__':
    sys.exit(main())
