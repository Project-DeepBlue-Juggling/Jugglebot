#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone hardware test harness for Jugglebot juggling-like movements.

Validates semi-aggressive, juggling-like Stewart platform movements at
large lateral excursions (~200mm XY radius) with coordinated tilts.

Juggling tests:
  JT1. Lateral shuttle -- +/-175mm X with +/-5 deg Y tilt, 4 full cycles
  JT2. Star pattern -- 5-point star at 175mm radius with random tilt

Prerequisites:
  - All Phase 7 tests must PASS
  - Robot fully assembled, platform free-standing
  - Legs must be homed (use --home flag)

Safety:
  - Mandatory check_feasibility() for all planned trajectories
  - Real-time workspace limit checking every control cycle
  - Conservative current limit (50% of rated = 10A)
  - All tests send IDLE to all 6 axes on completion, error, or Ctrl-C
  - Interactive confirmation before each test

Usage:
    python3 tools/juggling_test.py --home --test all
    python3 tools/juggling_test.py --test shuttle --speed-scale 0.25
    python3 tools/juggling_test.py --test star --move-duration 4.0
    python3 tools/juggling_test.py --dry-run
    python3 tools/juggling_test.py --dry-run --preview

Requirements:
  pip install python-can numpy
"""

from __future__ import annotations

import argparse
import datetime
import signal
import sys
import traceback
import time
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

# Import CAN harness and helpers
from free_platform_test import (  # noqa: E402
    PlatformTestHarness,
    LEG_AXES,
    interactive_pause,
)

# Import trajectory test infrastructure (reuse Phase 4 helpers)
from trajectory_test import (  # noqa: E402
    analyze_log,
    print_analysis,
    print_feasibility,
    prepare_harness,
    switch_to_passthrough,
    build_run_metadata,
    save_log,
)

# Import dynamic target infrastructure (reuse Phase 7 helpers)
from dynamic_target_test import (  # noqa: E402
    execute_with_dynamic_targets,
    move_to_active_home,
    safe_idle_all,
    filter_startup_samples,
    get_tracking_threshold,
    check_sequence_continuity,
    _insert_transitions,
)

# Motion planning imports
from jugglebot.motion.geometry import StewartGeometry  # noqa: E402
from jugglebot.motion.dynamics import DynamicsParams  # noqa: E402
from jugglebot.motion.ik_solver import (  # noqa: E402
    rotvec_to_rot_matrix,
    rot_matrix_to_quat,
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
)
from jugglebot.motion.trajectory import (  # noqa: E402
    TrajectoryManager,
    TrajectoryState,
    create_trajectory,
    evaluate,
    check_feasibility,
)
from jugglebot.motion.workspace import (  # noqa: E402
    WorkspaceLimits,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Default move duration (before speed scaling)
DEFAULT_MOVE_DURATION_S = 3.0

# Pause at each endpoint
ENDPOINT_PAUSE_S = 0.5

# JT1 parameters
SHUTTLE_RADIUS_MM = 175.0
SHUTTLE_TILT_DEG = 5.0
SHUTTLE_REPETITIONS = 4    # 4 full left-right cycles = 8 moves

# JT2 parameters
STAR_RADIUS_MM = 175.0
STAR_MAX_TILT_DEG = 7.0
STAR_RNG_SEED = 42

# Home Z from config
HOME_Z = float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)  # 170.0

# Default speed scale (full speed -- operator can use --speed-scale 0.25
# for conservative first runs)
DEFAULT_SPEED_SCALE = 1.0


# ---------------------------------------------------------------------------
# Rotation helper
# ---------------------------------------------------------------------------

def rotvec_to_quat(rx: float, ry: float, rz: float = 0.0) -> list:
    """Convert rotation vector [rx, ry, rz] (radians) to quaternion [w, x, y, z].

    Uses the codebase's Rodrigues -> Shepperd pipeline.
    """
    R = rotvec_to_rot_matrix(np.array([rx, ry, rz]))
    w, x, y, z = rot_matrix_to_quat(R)
    return [float(w), float(x), float(y), float(z)]


# ---------------------------------------------------------------------------
# Target sequence builders
# ---------------------------------------------------------------------------

def build_shuttle_targets(
    move_duration: float = DEFAULT_MOVE_DURATION_S,
    speed_scale: float = DEFAULT_SPEED_SCALE,
    n_cycles: int = SHUTTLE_REPETITIONS,
) -> list[dict]:
    """Build target sequence for JT1 lateral shuttle test.

    Alternates between (-200, 0, 170) with +5 deg Y tilt and
    (+200, 0, 170) with -5 deg Y tilt.  Each endpoint is held for
    ENDPOINT_PAUSE_S before the next move.

    Parameters
    ----------
    move_duration : float -- base duration per move in seconds
    speed_scale : float -- speed scaling (duration = move_duration / speed_scale)
    n_cycles : int -- number of full left-right cycles (each = 2 moves)

    Returns
    -------
    targets : list of dicts for execute_with_dynamic_targets()
    """
    actual_duration = move_duration / speed_scale
    tilt_rad = np.deg2rad(SHUTTLE_TILT_DEG)

    # Define the two endpoints
    # At x=-200: tilt ry=+5deg (tilted right toward centre)
    # At x=+200: tilt ry=-5deg (tilted left toward centre)
    left_pos = [-SHUTTLE_RADIUS_MM, 0.0, HOME_Z]
    left_quat = rotvec_to_quat(0.0, +tilt_rad)

    right_pos = [+SHUTTLE_RADIUS_MM, 0.0, HOME_Z]
    right_quat = rotvec_to_quat(0.0, -tilt_rad)

    targets = []
    t_cumulative = 0.0

    for _cycle in range(n_cycles):
        # Move left
        targets.append({
            'pos': left_pos,
            'quat': left_quat,
            'vel': [0, 0, 0],
            'delay_s': t_cumulative,
            'duration_s': actual_duration,
        })
        t_cumulative += actual_duration + ENDPOINT_PAUSE_S

        # Move right
        targets.append({
            'pos': right_pos,
            'quat': right_quat,
            'vel': [0, 0, 0],
            'delay_s': t_cumulative,
            'duration_s': actual_duration,
        })
        t_cumulative += actual_duration + ENDPOINT_PAUSE_S

    # Return to HOME
    targets.append({
        'pos': [0.0, 0.0, HOME_Z],
        'quat': [1, 0, 0, 0],
        'vel': [0, 0, 0],
        'delay_s': t_cumulative,
        'duration_s': actual_duration,
    })

    return targets


def build_star_targets(
    move_duration: float = DEFAULT_MOVE_DURATION_S,
    speed_scale: float = DEFAULT_SPEED_SCALE,
    seed: int = STAR_RNG_SEED,
) -> tuple[list[dict], int]:
    """Build target sequence for JT2 star pattern test.

    Generates 5 points at 200mm radius arranged in a 5-pointed star
    order (0, 144, 288, 72, 216 degrees).  Each point gets a random
    orientation of up to +/-7 degrees about X and Y.

    Random tilts are checked for IK reachability and resampled (up to
    10 attempts per point) if the combined position + tilt exceeds leg
    stroke limits.

    Parameters
    ----------
    move_duration : float -- base duration per move in seconds
    speed_scale : float -- speed scaling
    seed : int -- RNG seed for reproducibility

    Returns
    -------
    targets : list of dicts for execute_with_dynamic_targets()
    seed : int -- the seed used (for logging)
    """
    from jugglebot.motion.workspace import check_reachability

    geom = StewartGeometry()
    rng = np.random.default_rng(seed)
    actual_duration = move_duration / speed_scale
    max_tilt_rad = np.deg2rad(STAR_MAX_TILT_DEG)

    # 5-pointed star: visit vertices in star-drawing order
    # Each step is 144 degrees = 2 * 360 / 5
    star_angles_deg = [0, 144, 288, 72, 216]

    targets = []
    t_cumulative = 0.0

    for angle_deg in star_angles_deg:
        angle_rad = np.deg2rad(angle_deg)
        x = STAR_RADIUS_MM * np.cos(angle_rad)
        y = STAR_RADIUS_MM * np.sin(angle_rad)
        pos = np.array([x, y, HOME_Z])

        # Random orientation with reachability check
        for attempt in range(10):
            rx = float(rng.uniform(-max_tilt_rad, max_tilt_rad))
            ry = float(rng.uniform(-max_tilt_rad, max_tilt_rad))
            rot = rotvec_to_rot_matrix(np.array([rx, ry, 0.0]))
            if check_reachability(pos, rot, geom):
                break
        else:
            # All attempts failed -- fall back to zero tilt
            rx, ry = 0.0, 0.0

        quat = rotvec_to_quat(rx, ry)

        targets.append({
            'pos': [float(x), float(y), HOME_Z],
            'quat': quat,
            'vel': [0, 0, 0],
            'delay_s': t_cumulative,
            'duration_s': actual_duration,
        })
        t_cumulative += actual_duration + ENDPOINT_PAUSE_S

    # Return to HOME
    targets.append({
        'pos': [0.0, 0.0, HOME_Z],
        'quat': [1, 0, 0, 0],
        'vel': [0, 0, 0],
        'delay_s': t_cumulative,
        'duration_s': actual_duration,
    })

    return targets, seed


# ---------------------------------------------------------------------------
# Auto-save helper
# ---------------------------------------------------------------------------

# Log output directory: tools/logs/ (gitignored)
_LOGS_DIR = Path(__file__).parent / 'logs'


def _auto_save_log(log, test_name: str, speed_scale: float) -> None:
    """Save a run log to tools/logs/ with a timestamped filename."""
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    fname = f"{test_name}_{speed_scale:.1f}x_{ts}.json"
    save_log(log, _LOGS_DIR / fname)


# ---------------------------------------------------------------------------
# JT1: Lateral shuttle
# ---------------------------------------------------------------------------

def test_shuttle(
    harness: PlatformTestHarness,
    speed_scale: float = DEFAULT_SPEED_SCALE,
    move_duration: float = DEFAULT_MOVE_DURATION_S,
) -> bool:
    """JT1: Lateral shuttle -- +/-200mm X with +/-5 deg Y tilt, 4 cycles.

    Pass criteria: max tracking error < threshold for speed scale,
    all targets accepted, no ODrive faults.
    """
    print("\n" + "=" * 60)
    print(f"JT1: Lateral shuttle (+/-{SHUTTLE_RADIUS_MM:.0f}mm X, "
          f"+/-{SHUTTLE_TILT_DEG:.0f} deg Y tilt)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(mgr.home_pose)

    threshold = get_tracking_threshold(speed_scale)
    targets = build_shuttle_targets(move_duration, speed_scale)

    n_moves = len(targets)
    actual_dur = move_duration / speed_scale
    total_est = n_moves * actual_dur + (n_moves - 1) * ENDPOINT_PAUSE_S

    print(f"  Speed scale:     {speed_scale}")
    print(f"  Move duration:   {actual_dur:.1f}s (base {move_duration:.1f}s)")
    print(f"  Moves:           {n_moves} ({SHUTTLE_REPETITIONS} cycles + "
          f"return home)")
    print(f"  Estimated total: {total_est:.1f}s")
    print(f"  Threshold:       {threshold:.1f} mm")

    meta = build_run_metadata(
        'JT1', speed_scale, geom, params,
        tracking_threshold_mm=threshold,
        move_duration_s=move_duration,
        extra={
            'shuttle_radius_mm': SHUTTLE_RADIUS_MM,
            'shuttle_tilt_deg': SHUTTLE_TILT_DEG,
            'shuttle_repetitions': SHUTTLE_REPETITIONS,
            'endpoint_pause_s': ENDPOINT_PAUSE_S,
            'n_targets': n_moves,
        },
    )

    prepare_harness(harness)
    move_to_active_home(harness, geom, params)
    switch_to_passthrough(harness)
    interactive_pause("Press Enter to execute JT1...")

    print(f"\n  Executing JT1...")
    max_dur = total_est + 15.0  # generous timeout
    events, log = execute_with_dynamic_targets(
        harness, mgr, geom, params, limits, targets,
        max_duration_s=max_dur, label="JT1", metadata=meta)

    # Analyze
    filtered = filter_startup_samples(log)
    n_filtered = len(log.timestamps) - len(filtered.timestamps)
    if n_filtered:
        print(f"  ({n_filtered} startup samples filtered)")

    analysis = analyze_log(filtered) if filtered.timestamps else {}
    if analysis:
        print_analysis('JT1 Lateral shuttle', analysis)

        n_accepted = sum(1 for e in events if e['accepted'])
        n_faults = sum(1 for ax in LEG_AXES
                       if harness.states[ax].active_errors)

        print(f"\n  Targets accepted: {n_accepted}/{len(targets)}")
        print(f"  ODrive faults:    {n_faults}")
        print(f"  Max tracking:     {analysis['max_error_mm']:.3f} mm "
              f"(threshold: {threshold:.1f} mm)")

        passed = (analysis['max_error_mm'] < threshold
                  and n_faults == 0
                  and n_accepted == len(targets))

        # Save log with analysis results
        log.metadata['analysis'] = analysis
        log.metadata['result'] = 'PASS' if passed else 'FAIL'
        log.metadata['events'] = events
        _auto_save_log(log, 'JT1', speed_scale)

        print(f"  {'PASS' if passed else 'FAIL'}: JT1 Lateral shuttle")
        return passed
    else:
        print("  FAIL: No data recorded")
        return False


# ---------------------------------------------------------------------------
# JT2: Star pattern
# ---------------------------------------------------------------------------

def test_star(
    harness: PlatformTestHarness,
    speed_scale: float = DEFAULT_SPEED_SCALE,
    move_duration: float = DEFAULT_MOVE_DURATION_S,
) -> bool:
    """JT2: Star pattern -- 5 points at 200mm radius with random tilt.

    Pass criteria: max tracking error < threshold for speed scale,
    all targets accepted, no ODrive faults.
    """
    print("\n" + "=" * 60)
    print(f"JT2: Star pattern ({STAR_RADIUS_MM:.0f}mm radius, "
          f"random +/-{STAR_MAX_TILT_DEG:.0f} deg tilt)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(mgr.home_pose)

    threshold = get_tracking_threshold(speed_scale)
    targets, seed = build_star_targets(move_duration, speed_scale)

    n_moves = len(targets)
    actual_dur = move_duration / speed_scale
    total_est = n_moves * actual_dur + (n_moves - 1) * ENDPOINT_PAUSE_S

    print(f"  Speed scale:     {speed_scale}")
    print(f"  Move duration:   {actual_dur:.1f}s (base {move_duration:.1f}s)")
    print(f"  Points:          {n_moves - 1} (5-pointed star + return home)")
    print(f"  RNG seed:        {seed}")
    print(f"  Estimated total: {total_est:.1f}s")
    print(f"  Threshold:       {threshold:.1f} mm")

    # Print the star pattern for operator verification
    print(f"\n  Star pattern (seed={seed}):")
    for i, tgt in enumerate(targets):
        pos = tgt['pos']
        R = quat_to_rot_matrix(*tgt['quat'])
        rv = rot_matrix_to_rotvec(R)
        rx_deg = np.rad2deg(rv[0])
        ry_deg = np.rad2deg(rv[1])
        if i < len(targets) - 1:
            angle_deg = [0, 144, 288, 72, 216][i]
            print(f"    Point {i + 1} ({angle_deg:3d} deg): "
                  f"({pos[0]:7.1f}, {pos[1]:7.1f}, {pos[2]:5.1f}) mm  "
                  f"tilt=({rx_deg:+.1f}, {ry_deg:+.1f}) deg")
        else:
            print(f"    HOME:           "
                  f"({pos[0]:7.1f}, {pos[1]:7.1f}, {pos[2]:5.1f}) mm")

    # Build star point descriptions for metadata
    star_points = []
    for i, tgt in enumerate(targets[:-1]):
        R = quat_to_rot_matrix(*tgt['quat'])
        rv = rot_matrix_to_rotvec(R)
        star_points.append({
            'pos': tgt['pos'],
            'tilt_deg': [float(np.rad2deg(rv[0])),
                         float(np.rad2deg(rv[1]))],
        })

    meta = build_run_metadata(
        'JT2', speed_scale, geom, params,
        tracking_threshold_mm=threshold,
        move_duration_s=move_duration,
        extra={
            'star_radius_mm': STAR_RADIUS_MM,
            'star_max_tilt_deg': STAR_MAX_TILT_DEG,
            'star_rng_seed': seed,
            'endpoint_pause_s': ENDPOINT_PAUSE_S,
            'n_targets': n_moves,
            'star_points': star_points,
        },
    )

    prepare_harness(harness)
    move_to_active_home(harness, geom, params)
    switch_to_passthrough(harness)
    interactive_pause("Press Enter to execute JT2...")

    print(f"\n  Executing JT2...")
    max_dur = total_est + 15.0
    events, log = execute_with_dynamic_targets(
        harness, mgr, geom, params, limits, targets,
        max_duration_s=max_dur, label="JT2", metadata=meta)

    # Analyze
    filtered = filter_startup_samples(log)
    n_filtered = len(log.timestamps) - len(filtered.timestamps)
    if n_filtered:
        print(f"  ({n_filtered} startup samples filtered)")

    analysis = analyze_log(filtered) if filtered.timestamps else {}
    if analysis:
        print_analysis('JT2 Star pattern', analysis)

        n_accepted = sum(1 for e in events if e['accepted'])
        n_faults = sum(1 for ax in LEG_AXES
                       if harness.states[ax].active_errors)

        print(f"\n  Targets accepted: {n_accepted}/{len(targets)}")
        print(f"  ODrive faults:    {n_faults}")
        print(f"  Max tracking:     {analysis['max_error_mm']:.3f} mm "
              f"(threshold: {threshold:.1f} mm)")

        passed = (analysis['max_error_mm'] < threshold
                  and n_faults == 0
                  and n_accepted == len(targets))

        log.metadata['analysis'] = analysis
        log.metadata['result'] = 'PASS' if passed else 'FAIL'
        log.metadata['events'] = events
        _auto_save_log(log, 'JT2', speed_scale)

        print(f"  {'PASS' if passed else 'FAIL'}: JT2 Star pattern")
        return passed
    else:
        print("  FAIL: No data recorded")
        return False


# ---------------------------------------------------------------------------
# Dry-run / preview trajectory builder
# ---------------------------------------------------------------------------

def build_juggling_test_trajectories(
    geom: StewartGeometry = None,
    speed_scale: float = DEFAULT_SPEED_SCALE,
    move_duration: float = DEFAULT_MOVE_DURATION_S,
) -> list[tuple[str, object]]:
    """Build offline trajectories for dry-run/preview.

    Simulates the TrajectoryManager's submit_dynamic_target() offline
    for both JT1 and JT2, returning (label, QuinticTrajectory) tuples.
    """
    geom = geom or StewartGeometry()
    params = DynamicsParams.from_config()
    home_pose = np.array([0.0, 0.0, HOME_Z, 0.0, 0.0, 0.0])
    zeros6 = np.zeros(6)

    trajs = []

    def _simulate_targets(test_prefix, targets):
        """Simulate submit_dynamic_target for a target list."""
        mgr = TrajectoryManager(geom, params)
        mgr.set_hold_pose(home_pose)
        t_sim = 100.0  # arbitrary reference time

        for i, tgt in enumerate(targets):
            t_submit = t_sim + tgt['delay_s']

            # Advance manager to current time
            if mgr.state in (TrajectoryState.EXECUTING,
                              TrajectoryState.RETURNING):
                mgr.evaluate(t_submit)

            accepted = mgr.submit_dynamic_target(
                target_pos=np.array(tgt['pos']),
                target_quat=np.array(tgt['quat']),
                target_vel=np.array(tgt['vel']),
                arrival_time=t_submit + tgt['duration_s'],
                t_now=t_submit,
            )

            if accepted and mgr._active_traj is not None:
                traj_i = mgr._active_traj
                sp, st, sa = evaluate(traj_i, traj_i.t_start)
                ep, _, _ = evaluate(
                    traj_i, traj_i.t_start + traj_i.duration)
                pos = tgt['pos']
                trajs.append((
                    f'{test_prefix}: -> '
                    f'({pos[0]:.0f},{pos[1]:.0f},{pos[2]:.0f}) '
                    f'[point {i + 1}/{len(targets)}]',
                    create_trajectory(
                        start_pose=sp, start_twist=st, start_accel=sa,
                        end_pose=ep, end_twist=zeros6, end_accel=zeros6,
                        duration=traj_i.duration, t_start=0.0,
                    )
                ))
            elif not accepted:
                pos = tgt['pos']
                print(f"  WARNING: {test_prefix} target {i + 1} "
                      f"({pos[0]:.0f},{pos[1]:.0f},{pos[2]:.0f}) "
                      f"rejected by feasibility checker")

    # Build JT1 trajectories
    jt1_targets = build_shuttle_targets(move_duration, speed_scale)
    _simulate_targets('JT1', jt1_targets)

    # Build JT2 trajectories
    jt2_targets, _ = build_star_targets(move_duration, speed_scale)
    _simulate_targets('JT2', jt2_targets)

    # Insert transitions at any discontinuities
    trajs = _insert_transitions(trajs, geom, params)

    return trajs


def _filter_trajs_by_prefix(
    trajs: list[tuple],
    test_prefixes: set[str],
) -> list[tuple]:
    """Filter trajectory list to requested tests, keeping transitions."""
    keep = set()
    for i, (label, _) in enumerate(trajs):
        if any(label.startswith(p) for p in test_prefixes):
            keep.add(i)

    # Keep transitions that bridge two kept trajectories
    for i, (label, _) in enumerate(trajs):
        if label.startswith('~transition~'):
            if (i - 1) in keep and (i + 1) in keep:
                keep.add(i)

    return [trajs[i] for i in sorted(keep)]


def run_dry_run(speed_scale: float, move_duration: float,
                show_preview: bool, tests_to_run: list[str]):
    """Run feasibility checks on juggling trajectories without CAN."""
    print("\n" + "=" * 60)
    print("DRY RUN: Juggling test feasibility (no hardware)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    threshold = get_tracking_threshold(speed_scale)

    print(f"\n  Speed scale:      {speed_scale}")
    print(f"  Move duration:    {move_duration:.1f}s")
    print(f"  Tracking thresh:  {threshold:.1f} mm")
    print(f"  Tests:            {', '.join(tests_to_run)}")
    print()

    trajs = build_juggling_test_trajectories(
        geom, speed_scale, move_duration)

    # Filter to requested tests
    test_prefixes = set()
    if 'shuttle' in tests_to_run:
        test_prefixes.add('JT1')
    if 'star' in tests_to_run:
        test_prefixes.add('JT2')

    filtered = _filter_trajs_by_prefix(trajs, test_prefixes)

    if not filtered:
        print("  No trajectories to check.")
        return

    all_ok = True
    for label, traj in filtered:
        result = check_feasibility(traj, geom, params)
        status = 'OK' if result.feasible else 'FAIL'
        peak_vel = np.max(result.peak_leg_vel_rps)
        peak_cond = result.peak_condition_number
        print(f"  [{status}] {label}:  "
              f"dur={traj.duration:.2f}s  "
              f"peak_vel={peak_vel:.3f} rev/s  "
              f"peak_cond={peak_cond:.0f}")
        if not result.feasible:
            all_ok = False
            for v in result.violations:
                print(f"        {v}")

    print(f"\n  {'ALL FEASIBLE' if all_ok else 'SOME INFEASIBLE'}")

    # Continuity check
    print("\n  Sequence continuity check:")
    cont_ok, cont_violations = check_sequence_continuity(filtered)
    if cont_ok:
        print("  [OK] All trajectory junctions are C2-continuous")
    else:
        print("  [FAIL] Trajectory discontinuities:")
        for v in cont_violations:
            print(f"        {v}")

    if show_preview:
        try:
            from trajectory_viewer import preview_test_sequence
            print("\n  Launching 3D preview (close window to continue)...")
            preview_test_sequence(filtered, geom, params)
        except ImportError:
            print("  WARNING: matplotlib not available, skipping preview")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

TEST_MAP = {
    'shuttle': test_shuttle,
    'star': test_star,
}

ALL_TESTS = ['shuttle', 'star']


def main():
    parser = argparse.ArgumentParser(
        description='Juggling-like movement hardware tests',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests available:
  shuttle   JT1: Lateral shuttle (+/-175mm X, +/-5 deg Y tilt, 4 cycles)
  star      JT2: Star pattern (175mm radius, random +/-7 deg tilt)

Modes:
  --dry-run             Feasibility checks only, no CAN
  --preview             3D viewer before hardware execution
  --dry-run --preview   Feasibility + 3D viewer, no hardware
        """)
    parser.add_argument('--home', action='store_true',
                        help='Home all axes before testing')
    parser.add_argument('--test', nargs='+', default=['all'],
                        choices=['all'] + ALL_TESTS,
                        help='Tests to run (default: all)')
    parser.add_argument('--speed-scale', type=float,
                        default=DEFAULT_SPEED_SCALE,
                        help=f'Speed scaling factor '
                             f'(default: {DEFAULT_SPEED_SCALE})')
    parser.add_argument('--move-duration', type=float,
                        default=DEFAULT_MOVE_DURATION_S,
                        help=f'Base move duration in seconds '
                             f'(default: {DEFAULT_MOVE_DURATION_S})')
    parser.add_argument('--dry-run', action='store_true',
                        help='Feasibility checks only, no CAN')
    parser.add_argument('--preview', action='store_true',
                        help='Show 3D viewer before execution')
    parser.add_argument('--can-channel', default='can0',
                        help='CAN interface name (default: can0)')
    args = parser.parse_args()

    tests_to_run = ALL_TESTS if 'all' in args.test else args.test

    print("=" * 60)
    print("Juggling Movement Hardware Tests")
    print("=" * 60)
    print(f"  Speed scale:    {args.speed_scale}")
    print(f"  Move duration:  {args.move_duration:.1f}s")
    print(f"  Tests:          {', '.join(tests_to_run)}")
    print(f"  CAN channel:    {args.can_channel}")

    # Dry-run mode: feasibility checks + optional preview, no CAN
    if args.dry_run:
        run_dry_run(args.speed_scale, args.move_duration,
                    args.preview, tests_to_run)
        return

    # Pre-flight feasibility check (mandatory before hardware)
    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    trajs = build_juggling_test_trajectories(
        geom, args.speed_scale, args.move_duration)

    test_prefixes = set()
    if 'shuttle' in tests_to_run:
        test_prefixes.add('JT1')
    if 'star' in tests_to_run:
        test_prefixes.add('JT2')
    filtered = _filter_trajs_by_prefix(trajs, test_prefixes)

    print("\n  Pre-flight feasibility check:")
    all_feasible = True
    for label, traj in filtered:
        result = check_feasibility(traj, geom, params)
        if not result.feasible:
            all_feasible = False
            print_feasibility(label, result)
    if all_feasible:
        print("  [OK] All trajectories feasible")
    else:
        print("\n  ABORTING: Some trajectories infeasible")
        sys.exit(1)

    print("\n  Pre-flight continuity check:")
    cont_ok, cont_violations = check_sequence_continuity(filtered)
    if cont_ok:
        print("  [OK] All trajectory junctions are C2-continuous")
    else:
        print("  [FAIL] Trajectory discontinuities detected -- ABORTING:")
        for v in cont_violations:
            print(f"        {v}")
        sys.exit(1)

    # Optional 3D preview before hardware execution
    if args.preview:
        try:
            from trajectory_viewer import preview_test_sequence
            print("\n  Launching 3D preview (close window to continue)...")
            preview_test_sequence(filtered, geom, params)
        except ImportError:
            print("  WARNING: matplotlib not available, skipping preview")

    # Create harness
    harness = PlatformTestHarness(channel=args.can_channel)

    # Signal handler for clean shutdown
    def signal_handler(sig, frame):
        print("\n\n  Caught Ctrl-C -- stowing and idling...")
        safe_idle_all(harness)
        sys.exit(1)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        harness.connect()

        if args.home:
            harness.home_all()

        results = {}
        for name in tests_to_run:
            test_fn = TEST_MAP[name]
            try:
                passed = test_fn(harness,
                                 speed_scale=args.speed_scale,
                                 move_duration=args.move_duration)
                results[name] = passed
            except Exception as e:
                print(f"\n  EXCEPTION in {name}: {e}")
                traceback.print_exc()
                results[name] = False
                safe_idle_all(harness)

        # Summary
        print("\n" + "=" * 60)
        print("Juggling Test Summary")
        print("=" * 60)
        for name in tests_to_run:
            jt_num = ALL_TESTS.index(name) + 1
            status = 'PASS' if results.get(name) else 'FAIL'
            print(f"  JT{jt_num} ({name}): {status}")

        n_pass = sum(1 for v in results.values() if v)
        n_total = len(results)
        print(f"\n  {n_pass}/{n_total} tests passed")

        if n_pass < n_total:
            sys.exit(1)

    finally:
        safe_idle_all(harness)
        harness.disconnect()


if __name__ == '__main__':
    main()
