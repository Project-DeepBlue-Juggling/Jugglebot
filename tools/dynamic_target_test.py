#!/usr/bin/env python3
"""Standalone hardware test harness for Jugglebot Phase 7 (Dynamic Targets).

Validates dynamic target commanding, mid-motion replanning, and automatic
return-to-home on the physical platform.

Phase 7 hardware tests:
  DT1. Static target from home — simple zero-twist move
  DT2. Nonzero velocity + auto-return — platform moves through target and returns
  DT3. Mid-motion replan — second target during active trajectory
  DT4. Rapid target updates — stream of targets at 2 Hz
  DT5. Infeasible target ignored — current trajectory continues undisturbed

Prerequisites:
  - All Phase 4/5/6 tests must PASS
  - Robot fully assembled, platform free-standing
  - Legs must be homed (use --home flag)

Safety:
  - Mandatory check_feasibility() for all planned trajectories
  - Real-time workspace limit checking every control cycle
  - Conservative current limit (50% of rated = 10A)
  - All tests send IDLE to all 6 axes on completion, error, or Ctrl-C
  - Interactive confirmation before each test

Usage:
  python tools/dynamic_target_test.py --home --test all
  python tools/dynamic_target_test.py --test static --speed-scale 0.25
  python tools/dynamic_target_test.py --test replan --speed-scale 0.50
  python tools/dynamic_target_test.py --dry-run

Requirements:
  pip install python-can numpy
"""

from __future__ import annotations

import argparse
import signal
import sys
import traceback
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
    switch_to_passthrough,
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
from jugglebot.motion.trajectory import (  # noqa: E402
    TrajectoryManager,
    TrajectoryState,
    check_feasibility,
    evaluate,
    cartesian_to_motor_commands,
)
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
# Phase 7 test constants
# ---------------------------------------------------------------------------

# Speed-dependent tracking thresholds (same as Phase 5/6)
TRACKING_THRESHOLDS_MM = {0.25: 1.5, 0.50: 2.0, 0.75: 2.5, 1.00: 3.0}


def get_tracking_threshold(speed_scale: float) -> float:
    """Get the tracking error threshold for a given speed scale."""
    for ss in sorted(TRACKING_THRESHOLDS_MM.keys()):
        if speed_scale <= ss:
            return TRACKING_THRESHOLDS_MM[ss]
    return TRACKING_THRESHOLDS_MM[1.0]


# ---------------------------------------------------------------------------
# Platform positioning helpers
# ---------------------------------------------------------------------------

def _move_to_pose(harness: PlatformTestHarness,
                  pos: np.ndarray,
                  geom: StewartGeometry,
                  params: DynamicsParams,
                  label: str = ""):
    """Move the physical platform to a pose using TRAP_TRAJ (safe ramp).

    Switches to TRAP_TRAJ mode, commands the target, waits for completion,
    then returns (still in TRAP_TRAJ mode — caller must switch to
    PASSTHROUGH if needed for quintic control).
    """
    from jugglebot.motion.dynamics import gravity_to_motor_torques

    rot = np.eye(3)
    raw = pose_to_raw_positions(pos, rot, geom)
    torques = gravity_to_motor_torques(pos, rot, geom, params)

    harness.enter_trap_traj_mode_all(vel_limit=1.5, acc_limit=5.0,
                                     dec_limit=5.0)

    # Clear stale trajectory_done flags before sending the real target
    for axis_id in LEG_AXES:
        harness.states[axis_id].trajectory_done = False

    for i, axis_id in enumerate(LEG_AXES):
        tor_int = int(round(-torques[i] * LEG_TOR_FF_SCALE))
        tor_int = max(-32767, min(32767, tor_int))
        harness.send(encode_set_input_pos(
            axis_id, raw[i], vel_ff=0, torque_ff=tor_int))

    harness.wait_for_all_trajectories_done(timeout_s=15.0)
    harness.poll_for(1.0)
    if label:
        print(f"  {label}")


def move_to_active_home(harness: PlatformTestHarness,
                        geom: StewartGeometry,
                        params: DynamicsParams):
    """Move the physical platform to the active home pose (Z=170mm).

    Dynamic target tests need the platform at the TrajectoryManager's
    home pose [0, 0, 170, 0, 0, 0] so that the manager's internal state
    matches the physical platform position.
    """
    home_pos = np.array([0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)])
    _move_to_pose(harness, home_pos, geom, params,
                  f"At active home pose (Z={hw.JB_OP_DEFAULT_ACTIVE_Z_MM}mm).")


def stow_platform(harness: PlatformTestHarness):
    """Move all legs to 0 rev (homed position) using TRAP_TRAJ.

    MUST be called before idling axes.  Commands each axis to raw
    position 0.0 rev, which is the homed/stowed position.  This is
    different from IK pose [0,0,0] which produces ~25 mm leg extension.
    """
    harness.enter_trap_traj_mode_all(vel_limit=1.5, acc_limit=5.0,
                                     dec_limit=5.0)

    # Clear stale trajectory_done flags — enter_trap_traj_mode_all
    # commands each axis to hold at its current position, which
    # completes instantly and sets trajectory_done=True.  We must
    # clear these before sending the real move-to-0 commands,
    # otherwise wait_for_all_trajectories_done returns immediately.
    for axis_id in LEG_AXES:
        harness.states[axis_id].trajectory_done = False

    for axis_id in LEG_AXES:
        harness.send(encode_set_input_pos(
            axis_id, 0.0, vel_ff=0, torque_ff=0))

    harness.wait_for_all_trajectories_done(timeout_s=15.0)
    harness.poll_for(1.0)
    print("  Platform stowed (all legs at 0 rev).")


def safe_idle_all(harness: PlatformTestHarness):
    """Stow the platform (legs to 0 rev) and then idle all axes.

    Never idles axes unless the platform is in the stowed position.
    """
    try:
        stow_platform(harness)
    except Exception as e:
        print(f"  WARNING: Failed to stow platform: {e}")
        print("  Idling axes anyway (emergency).")
    harness.idle_all()
    print("  All axes -> IDLE.")


# ---------------------------------------------------------------------------
# Dynamic target execution engine
# ---------------------------------------------------------------------------

def execute_with_dynamic_targets(
    harness: PlatformTestHarness,
    mgr: TrajectoryManager,
    geom: StewartGeometry,
    params: DynamicsParams,
    limits: WorkspaceLimits,
    targets: list[dict],
    max_duration_s: float = 30.0,
    label: str = "",
) -> tuple[list[dict], TrajectoryLog]:
    """Execute a sequence of dynamic targets with real-time control.

    Parameters
    ----------
    harness : PlatformTestHarness
    mgr : TrajectoryManager (should be in IDLE at home)
    geom : StewartGeometry
    params : DynamicsParams
    limits : WorkspaceLimits
    targets : list of dicts with keys:
        - 'pos': [x, y, z] in mm
        - 'quat': [w, x, y, z] quaternion
        - 'vel': [vx, vy, vz] in mm/s
        - 'delay_s': seconds after test start to send this target
        - 'duration_s': trajectory duration in seconds
    max_duration_s : float — max test duration
    label : str — test label for logging

    Returns
    -------
    events : list of dicts with acceptance/rejection info
    log : TrajectoryLog with all recorded data
    """
    log = TrajectoryLog()
    events = []
    target_idx = 0

    t_test_start = time.perf_counter()
    t_prev = t_test_start
    # Set the manager's internal clock reference
    mgr_t_offset = t_test_start  # manager uses perf_counter directly

    while True:
        t_now = time.perf_counter()
        t_elapsed = t_now - t_test_start
        dt = t_now - t_prev
        t_prev = t_now

        # Check if we should send the next target
        while target_idx < len(targets):
            tgt = targets[target_idx]
            if t_elapsed >= tgt['delay_s']:
                arrival_time = t_now + tgt.get('duration_s', 1.0)
                accepted = mgr.submit_dynamic_target(
                    target_pos=np.array(tgt['pos']),
                    target_quat=np.array(tgt['quat']),
                    target_vel=np.array(tgt['vel']),
                    arrival_time=arrival_time,
                    t_now=t_now,
                )
                events.append({
                    'idx': target_idx,
                    't': t_elapsed,
                    'accepted': accepted,
                    'pos': tgt['pos'],
                    'vel': tgt['vel'],
                })
                status = 'ACCEPTED' if accepted else 'REJECTED'
                print(f"    [{t_elapsed:.3f}s] Target {target_idx}: {status} "
                      f"pos={tgt['pos']}")
                target_idx += 1
            else:
                break

        # Evaluate trajectory manager
        pos_rev, vel_ff, torque_ff = mgr.evaluate(t_now)

        # Workspace check
        pose_6dof = mgr.current_pose_6dof
        rot = rotvec_to_rot_matrix(pose_6dof[3:6])
        ext_mm = pose_to_leg_lengths(pose_6dof[:3], rot, geom)
        cond = compute_condition_number(pose_6dof[:3], rot, geom)
        ws = check_workspace_limits(ext_mm, cond, limits)

        if ws.status == WorkspaceStatus.HARD_LIMIT:
            print(f"\n  ** WORKSPACE HARD LIMIT at t={t_elapsed:.3f}s **")
            for v in ws.violations:
                print(f"     {v}")
            mgr.cancel()
            break

        # Send commands to ODrives
        for i, axis_id in enumerate(LEG_AXES):
            raw_pos = -pos_rev[i]
            vel_int = int(round(-vel_ff[i] * LEG_VEL_FF_SCALE))
            tor_int = int(round(-torque_ff[i] * LEG_TOR_FF_SCALE))
            vel_int = max(-32767, min(32767, vel_int))
            tor_int = max(-32767, min(32767, tor_int))
            harness.send_no_delay(encode_set_input_pos(
                axis_id, raw_pos, vel_int, tor_int))

        # Poll encoder feedback
        harness._poll(timeout=0)

        # Log data
        actual_pos = np.array([harness.states[a].pos_rev for a in LEG_AXES])
        actual_vel = np.array([harness.states[a].vel_rps for a in LEG_AXES])

        log.timestamps.append(t_elapsed)
        log.commanded_pos_rev.append(pos_rev.copy())
        log.commanded_vel_rps.append(vel_ff.copy())
        log.commanded_torque_Nm.append(torque_ff.copy())
        log.actual_pos_rev.append(actual_pos.copy())
        log.actual_vel_rps.append(actual_vel.copy())
        log.poses.append(pose_6dof.copy())
        log.loop_dt_s.append(dt)

        # Check for ODrive errors
        for ax in LEG_AXES:
            s = harness.states[ax]
            if s.active_errors:
                err_str = ', '.join(error_names(s.active_errors))
                print(f"\n  ** ODrive ERROR on axis {ax}: {err_str} **")
                mgr.cancel()
                break

        # Check termination: all targets sent AND manager is IDLE or COMPLETE
        all_sent = target_idx >= len(targets)
        settled = mgr.state in (TrajectoryState.IDLE, TrajectoryState.COMPLETE)
        if all_sent and settled and t_elapsed > targets[-1]['delay_s'] + 1.0:
            # Wait a bit after the last target settles
            break

        if t_elapsed > max_duration_s:
            print(f"  Max duration ({max_duration_s}s) reached")
            break

        # Rate control
        t_loop_end = time.perf_counter()
        sleep_s = TARGET_DT_S - (t_loop_end - t_now)
        if sleep_s > 0:
            time.sleep(sleep_s)

    return events, log


# ---------------------------------------------------------------------------
# DT1: Static target from home
# ---------------------------------------------------------------------------

def test_static_target(
    harness: PlatformTestHarness,
    speed_scale: float = DEFAULT_SPEED_SCALE,
) -> bool:
    """DT1: Zero-twist target from home -> +30mm Z.

    Pass criteria: tracking error < threshold for speed scale.
    """
    print("\n" + "=" * 60)
    print("DT1: Static target from home (+30mm Z)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(mgr.home_pose)

    threshold = get_tracking_threshold(speed_scale)

    # Duration scales inversely with speed_scale (slower = longer)
    duration = 1.0 / speed_scale

    targets = [{
        'pos': [0, 0, 200],  # home Z + 30mm
        'quat': [1, 0, 0, 0],
        'vel': [0, 0, 0],
        'delay_s': 0.0,
        'duration_s': duration,
    }]

    prepare_harness(harness)
    move_to_active_home(harness, geom, params)
    switch_to_passthrough(harness)
    interactive_pause("Press Enter to execute DT1...")

    print(f"\n  Executing DT1 (speed_scale={speed_scale}, "
          f"duration={duration:.1f}s)...")
    events, log = execute_with_dynamic_targets(
        harness, mgr, geom, params, limits, targets,
        max_duration_s=duration + 5.0, label="DT1")

    safe_idle_all(harness)

    # Analyze
    analysis = analyze_log(log) if log.timestamps else {}
    if analysis:
        print_analysis('DT1 Static target', analysis)
        passed = analysis['max_error_mm'] < threshold
        print(f"\n  Max tracking error: {analysis['max_error_mm']:.3f} mm "
              f"(threshold: {threshold} mm)")
        print(f"  {'PASS' if passed else 'FAIL'}: DT1 Static target")
        return passed
    else:
        print("  FAIL: No data recorded")
        return False


# ---------------------------------------------------------------------------
# DT2: Nonzero velocity + auto-return
# ---------------------------------------------------------------------------

def test_auto_return(
    harness: PlatformTestHarness,
    speed_scale: float = DEFAULT_SPEED_SCALE,
) -> bool:
    """DT2: Target with Z velocity, verify auto-return to home.

    Pass criteria: tracking error < threshold, final pose near home.
    """
    print("\n" + "=" * 60)
    print("DT2: Nonzero velocity + auto-return")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(mgr.home_pose)

    threshold = get_tracking_threshold(speed_scale)

    # Target at Z=190 with upward velocity of 30 mm/s (scaled)
    duration = 1.0 / speed_scale

    targets = [{
        'pos': [0, 0, 190],
        'quat': [1, 0, 0, 0],
        'vel': [0, 0, 30 * speed_scale],  # scale velocity too
        'delay_s': 0.0,
        'duration_s': duration,
    }]

    prepare_harness(harness)
    move_to_active_home(harness, geom, params)
    switch_to_passthrough(harness)
    interactive_pause("Press Enter to execute DT2...")

    print(f"\n  Executing DT2 (speed_scale={speed_scale}, "
          f"duration={duration:.1f}s)...")
    events, log = execute_with_dynamic_targets(
        harness, mgr, geom, params, limits, targets,
        max_duration_s=duration * 3 + 5.0, label="DT2")

    safe_idle_all(harness)

    # Analyze
    analysis = analyze_log(log) if log.timestamps else {}
    if analysis:
        print_analysis('DT2 Auto-return', analysis)

        # Check final pose near home
        final_pose = mgr.current_pose_6dof
        home_err = np.linalg.norm(final_pose - mgr.home_pose)
        final_state = mgr.state

        print(f"\n  Final state: {final_state.value}")
        print(f"  Final pose: {final_pose.tolist()}")
        print(f"  Error from home: {home_err:.3f} mm")
        print(f"  Max tracking error: {analysis['max_error_mm']:.3f} mm")

        passed = (analysis['max_error_mm'] < threshold and
                  home_err < 1.0 and
                  final_state == TrajectoryState.IDLE)
        print(f"  {'PASS' if passed else 'FAIL'}: DT2 Auto-return")
        return passed
    else:
        print("  FAIL: No data recorded")
        return False


# ---------------------------------------------------------------------------
# DT3: Mid-motion replan
# ---------------------------------------------------------------------------

def test_replan(
    harness: PlatformTestHarness,
    speed_scale: float = DEFAULT_SPEED_SCALE,
) -> bool:
    """DT3: Send first target, then second target after 0.5s.

    Pass criteria: smooth transition, tracking < threshold.
    """
    print("\n" + "=" * 60)
    print("DT3: Mid-motion replan")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(mgr.home_pose)

    threshold = get_tracking_threshold(speed_scale)
    base_dur = 1.5 / speed_scale

    targets = [
        {  # First target: +30mm Z
            'pos': [0, 0, 200],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': 0.0,
            'duration_s': base_dur,
        },
        {  # Second target: +15mm X, +25mm Z (sent 0.7s after first)
            'pos': [15, 0, 195],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': 0.7,  # 0.7s after first target
            'duration_s': base_dur,
        },
    ]

    prepare_harness(harness)
    move_to_active_home(harness, geom, params)
    switch_to_passthrough(harness)
    interactive_pause("Press Enter to execute DT3...")

    print(f"\n  Executing DT3 (speed_scale={speed_scale}, "
          f"duration={base_dur:.1f}s)...")
    events, log = execute_with_dynamic_targets(
        harness, mgr, geom, params, limits, targets,
        max_duration_s=base_dur * 2 + 5.0, label="DT3")

    safe_idle_all(harness)

    # Analyze
    analysis = analyze_log(log) if log.timestamps else {}
    if analysis:
        print_analysis('DT3 Replan', analysis)

        # Both targets should be accepted
        n_accepted = sum(1 for e in events if e['accepted'])
        print(f"\n  Targets accepted: {n_accepted}/{len(targets)}")
        print(f"  Max tracking error: {analysis['max_error_mm']:.3f} mm")

        passed = (analysis['max_error_mm'] < threshold and
                  n_accepted == len(targets))
        print(f"  {'PASS' if passed else 'FAIL'}: DT3 Replan")
        return passed
    else:
        print("  FAIL: No data recorded")
        return False


# ---------------------------------------------------------------------------
# DT4: Rapid target updates
# ---------------------------------------------------------------------------

def test_rapid_targets(
    harness: PlatformTestHarness,
    speed_scale: float = DEFAULT_SPEED_SCALE,
) -> bool:
    """DT4: Stream targets at 2 Hz for 5 seconds. Verify no faults.

    Pass criteria: no ODrive faults, at least some targets accepted.
    """
    print("\n" + "=" * 60)
    print("DT4: Rapid target updates (2 Hz)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(mgr.home_pose)

    # Generate a series of small targets at 2 Hz
    duration = 0.8 / speed_scale
    targets = []
    z_values = [180, 195, 175, 190, 185, 200, 170, 195, 180, 190]
    for i, z in enumerate(z_values):
        targets.append({
            'pos': [0, 0, z],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': i * 0.5,  # 2 Hz, first target immediately
            'duration_s': duration,
        })

    prepare_harness(harness)
    move_to_active_home(harness, geom, params)
    switch_to_passthrough(harness)
    interactive_pause("Press Enter to execute DT4...")

    print(f"\n  Executing DT4 (speed_scale={speed_scale}, "
          f"{len(targets)} targets, duration={duration:.1f}s)...")
    events, log = execute_with_dynamic_targets(
        harness, mgr, geom, params, limits, targets,
        max_duration_s=duration + 10.0, label="DT4")

    safe_idle_all(harness)

    # Analyze
    n_accepted = sum(1 for e in events if e['accepted'])
    n_rejected = sum(1 for e in events if not e['accepted'])
    n_faults = 0
    for ax in LEG_AXES:
        if harness.states[ax].active_errors:
            n_faults += 1

    print(f"\n  Targets sent: {len(targets)}")
    print(f"  Accepted: {n_accepted}, Rejected: {n_rejected}")
    print(f"  ODrive faults: {n_faults}")

    passed = n_faults == 0 and n_accepted > 0
    print(f"  {'PASS' if passed else 'FAIL'}: DT4 Rapid targets")
    return passed


# ---------------------------------------------------------------------------
# DT5: Infeasible target ignored
# ---------------------------------------------------------------------------

def test_infeasible_ignored(
    harness: PlatformTestHarness,
    speed_scale: float = DEFAULT_SPEED_SCALE,
) -> bool:
    """DT5: During active trajectory, send infeasible target.

    Verify current trajectory continues undisturbed.
    Pass criteria: infeasible target rejected, current trajectory completes.
    """
    print("\n" + "=" * 60)
    print("DT5: Infeasible target ignored during execution")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(mgr.home_pose)

    threshold = get_tracking_threshold(speed_scale)
    base_dur = 2.0 / speed_scale

    targets = [
        {  # First target: gentle +20mm Z
            'pos': [0, 0, 190],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': 0.0,
            'duration_s': base_dur,
        },
        {  # Infeasible target: way too far, way too fast
            'pos': [0, 0, 500],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': 0.5,  # 0.5s after first starts
            'duration_s': 0.01,  # impossibly fast
        },
    ]

    prepare_harness(harness)
    move_to_active_home(harness, geom, params)
    switch_to_passthrough(harness)
    interactive_pause("Press Enter to execute DT5...")

    print(f"\n  Executing DT5 (speed_scale={speed_scale}, "
          f"duration={base_dur:.1f}s)...")
    events, log = execute_with_dynamic_targets(
        harness, mgr, geom, params, limits, targets,
        max_duration_s=base_dur + 5.0, label="DT5")

    safe_idle_all(harness)

    # Analyze
    analysis = analyze_log(log) if log.timestamps else {}
    first_accepted = events[0]['accepted'] if events else False
    second_rejected = not events[1]['accepted'] if len(events) > 1 else False

    if analysis:
        print_analysis('DT5 Infeasible ignored', analysis)
        print(f"\n  First target accepted: {first_accepted}")
        print(f"  Second target (infeasible) rejected: {second_rejected}")
        print(f"  Max tracking error: {analysis['max_error_mm']:.3f} mm")

        passed = (first_accepted and second_rejected and
                  analysis['max_error_mm'] < threshold)
        print(f"  {'PASS' if passed else 'FAIL'}: DT5 Infeasible ignored")
        return passed
    else:
        print("  FAIL: No data recorded")
        return False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

TEST_MAP = {
    'static': test_static_target,
    'autoreturn': test_auto_return,
    'replan': test_replan,
    'rapid': test_rapid_targets,
    'infeasible': test_infeasible_ignored,
}

ALL_TESTS = ['static', 'autoreturn', 'replan', 'rapid', 'infeasible']


def main():
    parser = argparse.ArgumentParser(
        description='Phase 7 Dynamic Target Hardware Tests')
    parser.add_argument('--home', action='store_true',
                        help='Home the robot before testing')
    parser.add_argument('--test', nargs='+', default=['all'],
                        choices=['all'] + ALL_TESTS,
                        help='Tests to run')
    parser.add_argument('--speed-scale', type=float, default=DEFAULT_SPEED_SCALE,
                        help=f'Speed scaling factor (default: {DEFAULT_SPEED_SCALE})')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print test plan without executing')
    parser.add_argument('--can-channel', default='can0',
                        help='CAN interface name (default: can0)')
    args = parser.parse_args()

    tests_to_run = ALL_TESTS if 'all' in args.test else args.test

    print("=" * 60)
    print("Phase 7: Dynamic Target Hardware Tests")
    print("=" * 60)
    print(f"  Speed scale:  {args.speed_scale}")
    print(f"  Tests:        {', '.join(tests_to_run)}")
    print(f"  CAN channel:  {args.can_channel}")

    if args.dry_run:
        print("\n  [DRY RUN] Would execute:")
        for name in tests_to_run:
            print(f"    - DT{ALL_TESTS.index(name)+1}: {name}")
        return

    # Create harness
    harness = PlatformTestHarness(channel=args.can_channel)

    # Signal handler for clean shutdown
    def signal_handler(sig, frame):
        print("\n\n  Caught Ctrl-C -- stowing platform and idling...")
        safe_idle_all(harness)
        sys.exit(1)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        harness.connect()

        if args.home:
            from trajectory_test import home_robot  # noqa
            home_robot(harness)

        results = {}
        for name in tests_to_run:
            test_fn = TEST_MAP[name]
            try:
                passed = test_fn(harness, speed_scale=args.speed_scale)
                results[name] = passed
            except Exception as e:
                print(f"\n  EXCEPTION in {name}: {e}")
                traceback.print_exc()
                results[name] = False
                safe_idle_all(harness)

        # Summary
        print("\n" + "=" * 60)
        print("Phase 7 Test Summary")
        print("=" * 60)
        for name in tests_to_run:
            status = 'PASS' if results.get(name) else 'FAIL'
            dt_num = ALL_TESTS.index(name) + 1
            print(f"  DT{dt_num} ({name}): {status}")

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
