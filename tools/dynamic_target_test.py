#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
    python3 tools/dynamic_target_test.py --home --test all
    python3 tools/dynamic_target_test.py --test static --speed-scale 0.25
    python3 tools/dynamic_target_test.py --test replan --speed-scale 0.50
    python3 tools/dynamic_target_test.py --dry-run

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
    QuinticTrajectory,
    TrajectoryManager,
    TrajectoryState,
    check_feasibility,
    create_trajectory,
    evaluate,
    find_min_feasible_duration,
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

# Home Z for trajectory building (same as TrajectoryManager)
_HOME_Z = float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)  # 170.0


def get_tracking_threshold(speed_scale: float) -> float:
    """Get the tracking error threshold for a given speed scale."""
    for ss in sorted(TRACKING_THRESHOLDS_MM.keys()):
        if speed_scale <= ss:
            return TRACKING_THRESHOLDS_MM[ss]
    return TRACKING_THRESHOLDS_MM[1.0]


# ---------------------------------------------------------------------------
# Startup-stall filter
# ---------------------------------------------------------------------------

# submit_dynamic_target() runs a 50-sample feasibility check that takes
# ~250ms on Jetson.  When this happens inside the first loop iteration,
# the trajectory evaluator advances ~269ms while no command reaches the
# ODrive, producing a tracking-error spike that is a test-harness artefact,
# not a real control issue.  Filter these samples before analysis.
_STARTUP_DT_THRESHOLD_S = 0.050  # 50 ms


def filter_startup_samples(
    log: TrajectoryLog,
    dt_threshold_s: float = _STARTUP_DT_THRESHOLD_S,
) -> TrajectoryLog:
    """Return a new log with samples whose dt exceeds *dt_threshold_s* removed."""
    filtered = TrajectoryLog()
    for i in range(len(log.timestamps)):
        if log.loop_dt_s[i] <= dt_threshold_s:
            filtered.timestamps.append(log.timestamps[i])
            filtered.commanded_pos_rev.append(log.commanded_pos_rev[i])
            filtered.commanded_vel_rps.append(log.commanded_vel_rps[i])
            filtered.commanded_torque_Nm.append(log.commanded_torque_Nm[i])
            filtered.actual_pos_rev.append(log.actual_pos_rev[i])
            filtered.actual_vel_rps.append(log.actual_vel_rps[i])
            filtered.poses.append(log.poses[i])
            filtered.loop_dt_s.append(log.loop_dt_s[i])
    return filtered


# ---------------------------------------------------------------------------
# Sequence continuity check
# ---------------------------------------------------------------------------

# Tolerances for inter-trajectory continuity.  These are generous enough to
# tolerate floating-point round-trip through quintic solve → evaluate, but
# tight enough to catch a real position/velocity jump.
_CONT_POS_TOL_MM   = 0.5     # mm  — positional jump
_CONT_POS_TOL_RAD  = 0.005   # rad — rotational jump (~0.3 deg)
_CONT_VEL_TOL_MM   = 5.0     # mm/s  — velocity discontinuity
_CONT_VEL_TOL_RAD  = 0.05    # rad/s — rotational velocity
_CONT_ACCEL_TOL_MM  = 200.0  # mm/s² — acceleration discontinuity
_CONT_ACCEL_TOL_RAD = 2.0    # rad/s²


def check_sequence_continuity(
    trajs: list[tuple[str, 'QuinticTrajectory']],
) -> tuple[bool, list[str]]:
    """Verify C2 continuity between every consecutive trajectory pair.

    Checks position, velocity, and acceleration continuity at each
    junction in the full sequence — both within a single test (e.g.
    DT1 outbound → DT1 return) and between tests (e.g. DT1 return →
    DT2 outbound), since the preview chains them all back-to-back.

    A jump at a junction means the platform would be commanded to
    instantaneously change position (infinite acceleration) — exactly
    the behaviour we want to prevent.

    Parameters
    ----------
    trajs : list of (label, QuinticTrajectory) tuples

    Returns
    -------
    ok : bool — True if all junctions are smooth
    violations : list of human-readable violation strings
    """
    violations = []

    for i in range(len(trajs) - 1):
        label_a, traj_a = trajs[i]
        label_b, traj_b = trajs[i + 1]

        # Use stored boundary conditions rather than evaluate(), because
        # evaluate() clamps twist/accel to zero past the trajectory end
        # (hold behaviour), which would mask the actual end-state velocity
        # that the next trajectory expects to start from.
        sa = traj_a.end_state    # (18,) [pose, twist, accel]
        sb = traj_b.start_state  # (18,)
        pose_a,  twist_a,  accel_a  = sa[:6], sa[6:12], sa[12:18]
        pose_b,  twist_b,  accel_b  = sb[:6], sb[6:12], sb[12:18]

        # Check position continuity (translation + rotation separately)
        dp_trans = np.abs(pose_b[:3] - pose_a[:3])
        dp_rot   = np.abs(pose_b[3:] - pose_a[3:])

        for j in range(3):
            if dp_trans[j] > _CONT_POS_TOL_MM:
                axis = 'xyz'[j]
                violations.append(
                    f"junction {i+1}->{i+2}: "
                    f"position jump {axis}={dp_trans[j]:.3f} mm "
                    f"(tol {_CONT_POS_TOL_MM} mm)  "
                    f"[{label_a}] -> [{label_b}]")

        for j in range(3):
            if dp_rot[j] > _CONT_POS_TOL_RAD:
                axis = 'rxryrz'[j*2:j*2+2]
                violations.append(
                    f"junction {i+1}->{i+2}: "
                    f"position jump {axis}={dp_rot[j]:.4f} rad "
                    f"(tol {_CONT_POS_TOL_RAD} rad)  "
                    f"[{label_a}] -> [{label_b}]")

        # Check velocity continuity
        dv_trans = np.abs(twist_b[:3] - twist_a[:3])
        dv_rot   = np.abs(twist_b[3:] - twist_a[3:])

        for j in range(3):
            if dv_trans[j] > _CONT_VEL_TOL_MM:
                axis = 'xyz'[j]
                violations.append(
                    f"junction {i+1}->{i+2}: "
                    f"velocity jump {axis}={dv_trans[j]:.2f} mm/s "
                    f"(tol {_CONT_VEL_TOL_MM} mm/s)  "
                    f"[{label_a}] -> [{label_b}]")
        for j in range(3):
            if dv_rot[j] > _CONT_VEL_TOL_RAD:
                axis = 'rxryrz'[j*2:j*2+2]
                violations.append(
                    f"junction {i+1}->{i+2}: "
                    f"velocity jump {axis}={dv_rot[j]:.4f} rad/s "
                    f"(tol {_CONT_VEL_TOL_RAD} rad/s)  "
                    f"[{label_a}] -> [{label_b}]")

        # Check acceleration continuity
        da_trans = np.abs(accel_b[:3] - accel_a[:3])
        da_rot   = np.abs(accel_b[3:] - accel_a[3:])

        for j in range(3):
            if da_trans[j] > _CONT_ACCEL_TOL_MM:
                axis = 'xyz'[j]
                violations.append(
                    f"junction {i+1}->{i+2}: "
                    f"accel jump {axis}={da_trans[j]:.1f} mm/s² "
                    f"(tol {_CONT_ACCEL_TOL_MM} mm/s²)  "
                    f"[{label_a}] -> [{label_b}]")
        for j in range(3):
            if da_rot[j] > _CONT_ACCEL_TOL_RAD:
                axis = 'rxryrz'[j*2:j*2+2]
                violations.append(
                    f"junction {i+1}->{i+2}: "
                    f"accel jump {axis}={da_rot[j]:.3f} rad/s² "
                    f"(tol {_CONT_ACCEL_TOL_RAD} rad/s²)  "
                    f"[{label_a}] -> [{label_b}]")

    return len(violations) == 0, violations


# ---------------------------------------------------------------------------
# Preview / dry-run trajectory builder
# ---------------------------------------------------------------------------

def build_phase7_test_trajectories(
    geom: StewartGeometry = None,
    speed_scale: float = DEFAULT_SPEED_SCALE,
):
    """Build Phase 7 test trajectories for preview/dry-run.

    Simulates the TrajectoryManager's submit_dynamic_target() logic
    offline to produce the actual trajectories that each test would
    execute on hardware.

    Returns a list of (label, QuinticTrajectory) tuples.
    """
    geom = geom or StewartGeometry()
    params = DynamicsParams.from_config()
    home_pose = np.array([0.0, 0.0, _HOME_Z, 0.0, 0.0, 0.0])
    zeros6 = np.zeros(6)

    trajs = []

    # Helper: simulate submit_dynamic_target from a known start state
    def _make_traj(start_pose, target_pos, target_vel, duration,
                   start_twist=None, start_accel=None):
        end_pose = np.array([
            target_pos[0], target_pos[1], target_pos[2],
            0.0, 0.0, 0.0])
        end_twist = np.array([
            target_vel[0], target_vel[1], target_vel[2],
            0.0, 0.0, 0.0])
        return create_trajectory(
            start_pose=np.asarray(start_pose, dtype=float),
            start_twist=start_twist if start_twist is not None else zeros6,
            start_accel=start_accel if start_accel is not None else zeros6,
            end_pose=end_pose,
            end_twist=end_twist,
            end_accel=zeros6,
            duration=duration,
            t_start=0.0,
        )

    # --- DT1: Static target from home ---
    dt1_dur = 1.0 / speed_scale
    trajs.append(('DT1: Home -> Z=200 (static)',
                  _make_traj(home_pose, [0, 0, 200], [0, 0, 0], dt1_dur)))
    trajs.append(('DT1: Z=200 -> Home (return)',
                  _make_traj([0, 0, 200, 0, 0, 0], [0, 0, _HOME_Z],
                             [0, 0, 0], dt1_dur)))

    # --- DT2: Nonzero velocity + auto-return ---
    dt2_dur = 1.0 / speed_scale
    dt2_vel = 30 * speed_scale
    dt2_outbound = _make_traj(home_pose, [0, 0, 190],
                              [0, 0, dt2_vel], dt2_dur)
    trajs.append(('DT2: Home -> Z=190 (vel=+Z)', dt2_outbound))
    # Return trajectory: starts from the outbound's exact end state
    # so the junction is seamless.  Use TrajectoryManager to find
    # the return duration, but build from the outbound's boundary.
    mgr_dt2 = TrajectoryManager(geom, params)
    mgr_dt2.set_hold_pose(home_pose)
    t_sim = 100.0  # arbitrary reference time
    mgr_dt2.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 190.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.array([0.0, 0.0, dt2_vel]),
        arrival_time=t_sim + dt2_dur,
        t_now=t_sim,
    )
    # Evaluate at trajectory end to trigger return planning
    t_end_dt2 = t_sim + dt2_dur
    mgr_dt2.evaluate(t_end_dt2 + 0.001)
    if mgr_dt2._active_traj is not None:
        ret_traj = mgr_dt2._active_traj
        # Use the outbound's end_state directly so the junction is exact
        out_end = dt2_outbound.end_state
        trajs.append(('DT2: Z=190 -> Home (auto-return)',
                      create_trajectory(
                          start_pose=out_end[:6],
                          start_twist=out_end[6:12],
                          start_accel=out_end[12:18],
                          end_pose=home_pose,
                          end_twist=zeros6,
                          end_accel=zeros6,
                          duration=ret_traj.duration,
                          t_start=0.0,
                      )))

    # --- DT3: Mid-motion replan ---
    # First target: home -> Z=200
    dt3_dur = 1.5 / speed_scale
    trajs.append(('DT3: Home -> Z=200 (1st target)',
                  _make_traj(home_pose, [0, 0, 200], [0, 0, 0], dt3_dur)))
    # Second target: splice at t=0.7s into first, redirect to [15,0,195]
    # Simulate with TrajectoryManager
    mgr_dt3 = TrajectoryManager(geom, params)
    mgr_dt3.set_hold_pose(home_pose)
    t_sim3 = 100.0
    mgr_dt3.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 200.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_sim3 + dt3_dur,
        t_now=t_sim3,
    )
    # Submit second target 0.7s later
    t_splice = t_sim3 + 0.7
    mgr_dt3.submit_dynamic_target(
        target_pos=np.array([15.0, 0.0, 195.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_splice + dt3_dur,
        t_now=t_splice,
    )
    if mgr_dt3._active_traj is not None:
        splice_traj = mgr_dt3._active_traj
        sp, st, sa = evaluate(splice_traj, splice_traj.t_start)
        ep, _, _ = evaluate(splice_traj,
                                 splice_traj.t_start + splice_traj.duration)
        trajs.append(('DT3: Splice -> [15,0,195] (2nd target)',
                      create_trajectory(
                          start_pose=sp, start_twist=st, start_accel=sa,
                          end_pose=ep, end_twist=zeros6, end_accel=zeros6,
                          duration=splice_traj.duration,
                          t_start=0.0,
                      )))

    # --- DT4: Rapid target updates (show a representative subset) ---
    dt4_dur = 0.8 / speed_scale
    z_values = [180, 195, 175, 190, 185, 200, 170, 195, 180, 190]
    # Simulate the full sequence
    mgr_dt4 = TrajectoryManager(geom, params)
    mgr_dt4.set_hold_pose(home_pose)
    t_sim4 = 100.0
    for i, z in enumerate(z_values):
        t_submit = t_sim4 + i * 0.5
        # Evaluate manager up to this point so state is current
        if mgr_dt4.state in (TrajectoryState.EXECUTING,
                              TrajectoryState.RETURNING):
            mgr_dt4.evaluate(t_submit)
        mgr_dt4.submit_dynamic_target(
            target_pos=np.array([0.0, 0.0, float(z)]),
            target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
            target_vel=np.zeros(3),
            arrival_time=t_submit + dt4_dur,
            t_now=t_submit,
        )
        if mgr_dt4._active_traj is not None:
            traj_i = mgr_dt4._active_traj
            sp_i, st_i, sa_i = evaluate(traj_i, traj_i.t_start)
            ep_i, _, _ = evaluate(
                traj_i, traj_i.t_start + traj_i.duration)
            trajs.append((f'DT4: -> Z={z} (target {i+1}/10)',
                          create_trajectory(
                              start_pose=sp_i, start_twist=st_i,
                              start_accel=sa_i,
                              end_pose=ep_i, end_twist=zeros6,
                              end_accel=zeros6,
                              duration=traj_i.duration,
                              t_start=0.0,
                          )))

    # --- DT5: Infeasible target ignored (only the feasible move) ---
    dt5_dur = 2.0 / speed_scale
    trajs.append(('DT5: Home -> Z=190 (feasible)',
                  _make_traj(home_pose, [0, 0, 190], [0, 0, 0], dt5_dur)))
    trajs.append(('DT5: Z=190 -> Home (return)',
                  _make_traj([0, 0, 190, 0, 0, 0], [0, 0, _HOME_Z],
                             [0, 0, 0], dt5_dur)))

    # --- Post-process: insert transition trajectories at discontinuities ---
    trajs = _insert_transitions(trajs, geom, params)

    return trajs


# Transition speed: 1.5x the TRAP_TRAJ limits used for stow/activate
# (hardware uses vel=1.5 rev/s, acc=5.0 rev/s²)
_TRANSITION_SPEED_FACTOR = 1.5


def _insert_transitions(
    trajs: list[tuple[str, QuinticTrajectory]],
    geom: StewartGeometry,
    params: DynamicsParams,
) -> list[tuple[str, QuinticTrajectory]]:
    """Insert rest-to-rest transition trajectories at discontinuous junctions.

    Scans consecutive trajectory pairs.  Where the end pose of trajectory A
    doesn't match the start pose of trajectory B (beyond tolerance), a
    smooth quintic transition is spliced in between.

    The transition duration is found via find_min_feasible_duration() with
    a 1.5x safety margin, so it moves briskly but within feasibility limits.
    """
    if len(trajs) < 2:
        return trajs

    result = [trajs[0]]

    for i in range(1, len(trajs)):
        label_prev, traj_prev = result[-1]
        label_next, traj_next = trajs[i]

        # Use stored boundary conditions (not evaluate(), which clamps
        # twist/accel to zero past the trajectory end).
        se = traj_prev.end_state    # (18,) [pose, twist, accel]
        ss = traj_next.start_state  # (18,)
        pose_end,  twist_end  = se[:6], se[6:12]
        pose_start, twist_start = ss[:6], ss[6:12]

        dp = np.linalg.norm(pose_end[:3] - pose_start[:3])
        dr = np.linalg.norm(pose_end[3:] - pose_start[3:])
        dv = np.linalg.norm(twist_end[:3] - twist_start[:3])

        needs_transition = (dp > _CONT_POS_TOL_MM
                            or dr > _CONT_POS_TOL_RAD
                            or dv > _CONT_VEL_TOL_MM)

        if needs_transition:
            # Build a transition from end of prev to start of next.
            # Start boundary: traj_prev's end state (evaluate returns zero
            # twist/accel after duration).
            # End boundary: traj_next's actual start state, which may have
            # nonzero twist if it was spliced mid-motion.
            zeros6 = np.zeros(6)
            duration = find_min_feasible_duration(
                start_pose=pose_end,
                start_twist=twist_end,
                start_accel=zeros6,
                end_pose=pose_start,
                end_twist=twist_start,
                end_accel=zeros6,
                geom=geom,
                dynamics_params=params,
            )
            if duration is None:
                # Shouldn't happen for modest transitions — warn and skip
                print(f"  WARNING: transition {label_prev} -> {label_next} "
                      f"infeasible, skipping")
                result.append(trajs[i])
                continue

            # Apply 1.5x safety margin (same philosophy as return-to-home)
            duration *= _TRANSITION_SPEED_FACTOR

            transition = create_trajectory(
                start_pose=pose_end,
                start_twist=twist_end,
                start_accel=zeros6,
                end_pose=pose_start,
                end_twist=twist_start,
                end_accel=zeros6,
                duration=duration,
                t_start=0.0,
            )
            result.append((f'~transition~ ({label_prev} -> {label_next})',
                           transition))

        result.append(trajs[i])

    return result


def _filter_trajs(
    trajs: list[tuple[str, QuinticTrajectory]],
    test_prefixes: set[str],
) -> list[tuple[str, QuinticTrajectory]]:
    """Filter trajectory list to requested tests, keeping transitions.

    Keeps any trajectory whose label starts with a requested prefix (e.g.
    "DT1", "DT4") AND any transition trajectory whose neighbours are both
    in the kept set.  A transition between two kept tests must be kept,
    otherwise the continuity check would see a gap.
    """
    # First pass: mark which indices are test trajectories we want
    keep = set()
    for i, (label, _) in enumerate(trajs):
        if any(label.startswith(p) for p in test_prefixes):
            keep.add(i)

    # Second pass: keep transitions that bridge two kept trajectories
    for i, (label, _) in enumerate(trajs):
        if label.startswith('~transition~'):
            # Keep if the trajectory before and after are both kept
            prev_kept = (i - 1) in keep
            next_kept = (i + 1) in keep
            if prev_kept and next_kept:
                keep.add(i)

    # Preserve order
    return [trajs[i] for i in sorted(keep)]


def run_dry_run(speed_scale: float, show_preview: bool,
                tests_to_run: list[str]):
    """Run feasibility checks on Phase 7 test trajectories without CAN."""
    from jugglebot.motion.trajectory import check_feasibility

    print("\n" + "=" * 60)
    print("DRY RUN: Phase 7 Feasibility checks (no hardware)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    threshold = get_tracking_threshold(speed_scale)
    print(f"\n  Speed scale: {speed_scale}")
    print(f"  Tracking threshold: {threshold} mm")
    print(f"  Tests: {', '.join(tests_to_run)}")
    print()

    trajs = build_phase7_test_trajectories(geom, speed_scale=speed_scale)

    # Filter to requested tests
    test_prefixes = set()
    for name in tests_to_run:
        dt_num = ALL_TESTS.index(name) + 1
        test_prefixes.add(f'DT{dt_num}')

    filtered = _filter_trajs(trajs, test_prefixes)

    all_ok = True
    for label, traj in filtered:
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

    # Sequence continuity check
    print("\n  Sequence continuity check:")
    cont_ok, cont_violations = check_sequence_continuity(filtered)
    if cont_ok:
        print("  [OK] All trajectory junctions are C2-continuous")
    else:
        all_ok = False
        print("  [FAIL] Trajectory discontinuities detected:")
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
    mgr.realtime_restamp = True
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

    # Analyze (stowing handled by main() finally block)
    filtered = filter_startup_samples(log)
    n_filtered = len(log.timestamps) - len(filtered.timestamps)
    if n_filtered:
        print(f"  ({n_filtered} startup samples filtered, dt > "
              f"{_STARTUP_DT_THRESHOLD_S*1e3:.0f}ms)")
    analysis = analyze_log(filtered) if filtered.timestamps else {}
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
    mgr.realtime_restamp = True
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

    # Analyze (stowing handled by main() finally block)
    filtered = filter_startup_samples(log)
    n_filtered = len(log.timestamps) - len(filtered.timestamps)
    if n_filtered:
        print(f"  ({n_filtered} startup samples filtered, dt > "
              f"{_STARTUP_DT_THRESHOLD_S*1e3:.0f}ms)")
    analysis = analyze_log(filtered) if filtered.timestamps else {}
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
    mgr.realtime_restamp = True
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

    # Analyze (stowing handled by main() finally block)
    filtered = filter_startup_samples(log)
    n_filtered = len(log.timestamps) - len(filtered.timestamps)
    if n_filtered:
        print(f"  ({n_filtered} startup samples filtered, dt > "
              f"{_STARTUP_DT_THRESHOLD_S*1e3:.0f}ms)")
    analysis = analyze_log(filtered) if filtered.timestamps else {}
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
    mgr.realtime_restamp = True
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

    # Analyze (stowing handled by main() finally block)
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
    mgr.realtime_restamp = True
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

    # Analyze (stowing handled by main() finally block)
    filtered = filter_startup_samples(log)
    n_filtered = len(log.timestamps) - len(filtered.timestamps)
    if n_filtered:
        print(f"  ({n_filtered} startup samples filtered, dt > "
              f"{_STARTUP_DT_THRESHOLD_S*1e3:.0f}ms)")
    analysis = analyze_log(filtered) if filtered.timestamps else {}
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
        description='Phase 7 Dynamic Target Hardware Tests',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests available:
  static      DT1: Static target from home (+30mm Z)
  autoreturn  DT2: Nonzero velocity + auto-return
  replan      DT3: Mid-motion replan (second target during active)
  rapid       DT4: Rapid target updates (2 Hz stream)
  infeasible  DT5: Infeasible target ignored

Modes:
  --dry-run             Feasibility checks only, no CAN connection
  --preview             Show 3D viewer before hardware execution
  --dry-run --preview   Feasibility + 3D viewer, no hardware
        """)
    parser.add_argument('--home', action='store_true',
                        help='Home the robot before testing')
    parser.add_argument('--test', nargs='+', default=['all'],
                        choices=['all'] + ALL_TESTS,
                        help='Tests to run')
    parser.add_argument('--speed-scale', type=float, default=DEFAULT_SPEED_SCALE,
                        help=f'Speed scaling factor (default: {DEFAULT_SPEED_SCALE})')
    parser.add_argument('--dry-run', action='store_true',
                        help='Feasibility checks only, no CAN connection')
    parser.add_argument('--preview', action='store_true',
                        help='Show 3D viewer preview before hardware execution')
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

    # Dry-run mode: feasibility checks + optional preview, no CAN
    if args.dry_run:
        run_dry_run(args.speed_scale, args.preview, tests_to_run)
        return

    # Pre-flight continuity check (mandatory before hardware)
    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    trajs = build_phase7_test_trajectories(
        geom, speed_scale=args.speed_scale)
    test_prefixes = set()
    for name in tests_to_run:
        dt_num = ALL_TESTS.index(name) + 1
        test_prefixes.add(f'DT{dt_num}')
    filtered = _filter_trajs(trajs, test_prefixes)

    print("\n  Pre-flight continuity check:")
    cont_ok, cont_violations = check_sequence_continuity(filtered)
    if cont_ok:
        print("  [OK] All trajectory junctions are C2-continuous")
    else:
        print("  [FAIL] Trajectory discontinuities detected — ABORTING:")
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
