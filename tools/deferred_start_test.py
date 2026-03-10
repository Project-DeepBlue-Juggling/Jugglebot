#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone hardware test harness for Deferred Start validation.

Validates that deferred start for dynamic targets works correctly
without position discontinuities ("small jumps" glitch).

Test phases:
  Phase A — Regression (existing functionality still works)
    A1. Short-duration target (no deferral) — same as DT1
    A2. Auto-return from velocity target — same as DT2
    A3. Mid-motion splice — same as DT3

  Phase B — Deferred Start Validation
    B1. Basic deferred start from IDLE (10s arrival, ~2s motion)
    B2. Different move sizes (small, medium, large Z offsets)
    B3. Deferred start + auto-return (nonzero velocity at target)
    B4. Repeated deferred targets (3 consecutive far-future targets)

  Phase C — Edge Cases
    C1. Borderline duration (just barely triggers deferral)
    C2. Deferred start during active trajectory (splice with deferral)

Primary glitch detector: inter-sample position jump metric.
Any jump > 0.5 mm between consecutive commanded positions (at 500 Hz)
indicates a position discontinuity — the exact bug we're testing for.

Prerequisites:
  - All Phase 4/5/6/7 tests must PASS
  - Robot fully assembled, platform free-standing
  - Legs must be homed (use --home flag)

Safety:
  - Mandatory check_feasibility() for all planned trajectories
  - Real-time workspace limit checking every control cycle
  - Conservative current limit (50% of rated = 10A)
  - All tests send IDLE to all 6 axes on completion, error, or Ctrl-C
  - Interactive confirmation before each test

Usage:
    python3 tools/deferred_start_test.py --home --test all
    python3 tools/deferred_start_test.py --test A1 --speed-scale 0.50
    python3 tools/deferred_start_test.py --phase A
    python3 tools/deferred_start_test.py --phase B --preview
    python3 tools/deferred_start_test.py --dry-run --preview

Requirements:
  pip install python-can numpy
  (optional for --preview: matplotlib)
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
    NUM_LEGS,
    MM_TO_REV,
    LEG_VEL_FF_SCALE,
    LEG_TOR_FF_SCALE,
    encode_set_input_pos,
    pose_to_raw_positions,
    interactive_pause,
    error_names,
)

# Import trajectory test infrastructure
from trajectory_test import (  # noqa: E402
    TrajectoryLog,
    analyze_log,
    print_analysis,
    print_feasibility,
    prepare_harness,
    switch_to_passthrough,
    build_run_metadata,
    save_log,
    TARGET_DT_S,
    TARGET_LOOP_HZ,
    HOLD_AFTER_S,
    DEFAULT_SPEED_SCALE,
)

# Reuse Phase 7 helpers
from dynamic_target_test import (  # noqa: E402
    move_to_active_home,
    safe_idle_all,
    stow_platform,
    execute_with_dynamic_targets,
    filter_startup_samples,
    get_tracking_threshold,
    check_sequence_continuity,
    _STARTUP_DT_THRESHOLD_S,
)

# Motion planning imports
from jugglebot.motion.geometry import StewartGeometry  # noqa: E402
from jugglebot.motion.dynamics import DynamicsParams  # noqa: E402
from jugglebot.motion.ik_solver import (  # noqa: E402
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.trajectory import (  # noqa: E402
    DEFERRED_START_BUFFER_S,
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
# Constants
# ---------------------------------------------------------------------------

_HOME_Z = float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)  # 170.0

# Speed-dependent tracking thresholds (same as Phase 7)
TRACKING_THRESHOLDS_MM = {
    0.25: 1.5, 0.50: 2.0, 0.75: 2.5, 1.00: 3.0,
    2.00: 5.0, 3.00: 7.0, 4.00: 10.0,
}

# Inter-sample jump thresholds — the PRIMARY glitch detector.
# At 500 Hz, max legitimate position change between samples is bounded
# by max velocity (15 rev/s) * dt (2ms) = 0.03 rev ≈ 2.1 mm.
# A discontinuity ("small jump") would produce much larger inter-sample
# changes.  We use a conservative threshold that catches real jumps but
# doesn't false-positive on normal motion.
MAX_INTERSAMPLE_JUMP_MM = 0.5   # mm per sample at 500 Hz


# ---------------------------------------------------------------------------
# Logs directory
# ---------------------------------------------------------------------------

_LOGS_DIR = Path(__file__).parent / 'logs'


def _auto_save_log(log: TrajectoryLog, test_name: str,
                   speed_scale: float) -> None:
    """Save a run log to tools/logs/ with a timestamped filename."""
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    fname = f"{test_name}_{speed_scale:.1f}x_{ts}.json"
    save_log(log, _LOGS_DIR / fname)


# ---------------------------------------------------------------------------
# Inter-sample jump analysis
# ---------------------------------------------------------------------------

def check_intersample_jumps(
    log: TrajectoryLog,
    threshold_mm: float = MAX_INTERSAMPLE_JUMP_MM,
) -> tuple[bool, float, list[dict]]:
    """Scan commanded positions for inter-sample jumps.

    This is the primary glitch detector.  At 500 Hz, a legitimate
    trajectory produces smooth position changes between samples.
    A position discontinuity ("small jump") produces a spike in the
    inter-sample difference that exceeds the threshold.

    Parameters
    ----------
    log : TrajectoryLog
    threshold_mm : float — max allowed position change per sample (mm)

    Returns
    -------
    ok : bool — True if no jumps exceed threshold
    max_jump_mm : float — largest inter-sample jump observed
    violations : list of dicts with details of each jump
    """
    if len(log.commanded_pos_rev) < 2:
        return True, 0.0, []

    violations = []
    max_jump = 0.0

    for i in range(1, len(log.commanded_pos_rev)):
        pos_prev = np.array(log.commanded_pos_rev[i - 1])
        pos_curr = np.array(log.commanded_pos_rev[i])
        # Convert position difference from rev to mm
        delta_rev = np.abs(pos_curr - pos_prev)
        delta_mm = delta_rev / MM_TO_REV  # MM_TO_REV is rev/mm
        max_leg_jump = np.max(delta_mm)

        if max_leg_jump > max_jump:
            max_jump = max_leg_jump

        if max_leg_jump > threshold_mm:
            worst_leg = int(np.argmax(delta_mm))
            violations.append({
                'sample': i,
                't': log.timestamps[i] if i < len(log.timestamps) else 0,
                'dt': log.loop_dt_s[i] if i < len(log.loop_dt_s) else 0,
                'max_jump_mm': max_leg_jump,
                'worst_leg': worst_leg,
                'delta_mm': delta_mm.tolist(),
            })

    return len(violations) == 0, max_jump, violations


def print_jump_analysis(ok: bool, max_jump: float,
                        violations: list[dict], label: str) -> None:
    """Print inter-sample jump analysis results."""
    print(f"\n  Inter-sample jump analysis ({label}):")
    print(f"    Max jump: {max_jump:.4f} mm "
          f"(threshold: {MAX_INTERSAMPLE_JUMP_MM} mm)")
    if ok:
        print(f"    [OK] No position discontinuities detected")
    else:
        print(f"    [FAIL] {len(violations)} jump(s) exceed threshold:")
        for v in violations[:5]:  # Show first 5
            print(f"      t={v['t']:.4f}s  dt={v['dt']*1e3:.1f}ms  "
                  f"jump={v['max_jump_mm']:.3f}mm  leg={v['worst_leg']}")
        if len(violations) > 5:
            print(f"      ... and {len(violations) - 5} more")


# ---------------------------------------------------------------------------
# Preview / dry-run trajectory builder
# ---------------------------------------------------------------------------

def build_deferred_start_test_trajectories(
    geom: StewartGeometry = None,
    speed_scale: float = DEFAULT_SPEED_SCALE,
) -> list[tuple[str, QuinticTrajectory]]:
    """Build deferred start test trajectories for preview/dry-run.

    Simulates the TrajectoryManager's deferred start logic offline
    to produce the actual trajectories each test would execute.

    Returns a list of (label, QuinticTrajectory) tuples.
    """
    geom = geom or StewartGeometry()
    params = DynamicsParams.from_config()
    home_pose = np.array([0.0, 0.0, _HOME_Z, 0.0, 0.0, 0.0])
    zeros6 = np.zeros(6)

    trajs = []

    def _make_rest_to_rest(start_pose, end_pose, duration):
        return create_trajectory(
            start_pose=np.asarray(start_pose, dtype=float),
            start_twist=zeros6, start_accel=zeros6,
            end_pose=np.asarray(end_pose, dtype=float),
            end_twist=zeros6, end_accel=zeros6,
            duration=duration, t_start=0.0,
        )

    # --- A1: Short-duration target (no deferral) ---
    a1_dur = 1.0 / speed_scale
    trajs.append(('A1: Home -> Z=200 (short, no defer)',
                  _make_rest_to_rest(home_pose,
                                     [0, 0, 200, 0, 0, 0], a1_dur)))
    trajs.append(('A1: Z=200 -> Home (return)',
                  _make_rest_to_rest([0, 0, 200, 0, 0, 0],
                                     home_pose, a1_dur)))

    # --- A2: Auto-return with velocity ---
    a2_dur = 1.0 / speed_scale
    a2_vel_z = 30.0 * speed_scale
    a2_end_twist = np.array([0, 0, a2_vel_z, 0, 0, 0])
    a2_out = create_trajectory(
        start_pose=home_pose, start_twist=zeros6, start_accel=zeros6,
        end_pose=np.array([0, 0, 190, 0, 0, 0]),
        end_twist=a2_end_twist, end_accel=zeros6,
        duration=a2_dur, t_start=0.0,
    )
    trajs.append(('A2: Home -> Z=190 (vel=+Z)', a2_out))
    # Return from end state
    es = a2_out.end_state
    ret_dur = find_min_feasible_duration(
        start_pose=es[:6], start_twist=es[6:12], start_accel=es[12:18],
        end_pose=home_pose, end_twist=zeros6, end_accel=zeros6,
        geom=geom, dynamics_params=params,
    )
    if ret_dur is not None:
        trajs.append(('A2: Z=190 -> Home (auto-return)',
                      create_trajectory(
                          start_pose=es[:6], start_twist=es[6:12],
                          start_accel=es[12:18],
                          end_pose=home_pose, end_twist=zeros6,
                          end_accel=zeros6,
                          duration=ret_dur * 1.5, t_start=0.0,
                      )))

    # --- A3: Mid-motion splice ---
    a3_dur = 1.5 / speed_scale
    a3_first = _make_rest_to_rest(home_pose, [0, 0, 200, 0, 0, 0], a3_dur)
    trajs.append(('A3: Home -> Z=200 (1st target)', a3_first))
    # Splice at t=0.7s
    sp, st, sa = evaluate(a3_first, 0.7)
    a3_splice = create_trajectory(
        start_pose=sp, start_twist=st, start_accel=sa,
        end_pose=np.array([15, 0, 195, 0, 0, 0]),
        end_twist=zeros6, end_accel=zeros6,
        duration=a3_dur, t_start=0.0,
    )
    trajs.append(('A3: Splice -> [15,0,195] (2nd target)', a3_splice))

    # --- B1: Basic deferred start from IDLE ---
    # 10s arrival, motion should be ~min_feasible + 2s ≈ 3s
    b1_target_pose = np.array([0, 0, 200, 0, 0, 0])
    b1_min = find_min_feasible_duration(
        start_pose=home_pose, start_twist=zeros6, start_accel=zeros6,
        end_pose=b1_target_pose, end_twist=zeros6, end_accel=zeros6,
        geom=geom, dynamics_params=params,
    )
    if b1_min is not None:
        b1_motion = b1_min + DEFERRED_START_BUFFER_S
        trajs.append(('B1: Home -> Z=200 (deferred, 10s arrival)',
                      _make_rest_to_rest(home_pose, b1_target_pose,
                                         b1_motion / speed_scale)))
    trajs.append(('B1: Z=200 -> Home (return)',
                  _make_rest_to_rest([0, 0, 200, 0, 0, 0], home_pose,
                                     1.5 / speed_scale)))

    # --- B2: Different move sizes ---
    for label_z, target_z in [('small', 180), ('medium', 200), ('large', 220)]:
        b2_pose = np.array([0, 0, float(target_z), 0, 0, 0])
        b2_min = find_min_feasible_duration(
            start_pose=home_pose, start_twist=zeros6, start_accel=zeros6,
            end_pose=b2_pose, end_twist=zeros6, end_accel=zeros6,
            geom=geom, dynamics_params=params,
        )
        if b2_min is not None:
            b2_motion = b2_min + DEFERRED_START_BUFFER_S
            trajs.append((f'B2: Home -> Z={target_z} ({label_z}, deferred)',
                          _make_rest_to_rest(home_pose, b2_pose,
                                             b2_motion / speed_scale)))
            trajs.append((f'B2: Z={target_z} -> Home ({label_z} return)',
                          _make_rest_to_rest(b2_pose, home_pose,
                                             1.5 / speed_scale)))

    # --- B3: Deferred + auto-return ---
    b3_target_pose = np.array([0, 0, 195, 0, 0, 0])
    b3_vel = np.array([0, 0, 20 * speed_scale, 0, 0, 0])
    b3_min = find_min_feasible_duration(
        start_pose=home_pose, start_twist=zeros6, start_accel=zeros6,
        end_pose=b3_target_pose, end_twist=b3_vel, end_accel=zeros6,
        geom=geom, dynamics_params=params,
    )
    if b3_min is not None:
        b3_motion = b3_min + DEFERRED_START_BUFFER_S
        b3_out = create_trajectory(
            start_pose=home_pose, start_twist=zeros6, start_accel=zeros6,
            end_pose=b3_target_pose, end_twist=b3_vel, end_accel=zeros6,
            duration=b3_motion / speed_scale, t_start=0.0,
        )
        trajs.append(('B3: Home -> Z=195 (vel, deferred)', b3_out))
        es3 = b3_out.end_state
        ret3 = find_min_feasible_duration(
            start_pose=es3[:6], start_twist=es3[6:12],
            start_accel=es3[12:18],
            end_pose=home_pose, end_twist=zeros6, end_accel=zeros6,
            geom=geom, dynamics_params=params,
        )
        if ret3 is not None:
            trajs.append(('B3: Z=195 -> Home (auto-return)',
                          create_trajectory(
                              start_pose=es3[:6], start_twist=es3[6:12],
                              start_accel=es3[12:18],
                              end_pose=home_pose, end_twist=zeros6,
                              end_accel=zeros6,
                              duration=ret3 * 1.5, t_start=0.0,
                          )))

    # --- B4: Repeated deferred targets ---
    for idx, target_z in enumerate([190, 200, 185]):
        b4_pose = np.array([0, 0, float(target_z), 0, 0, 0])
        b4_from = home_pose if idx == 0 else np.array(
            [0, 0, float([190, 200, 185][idx - 1]), 0, 0, 0])
        b4_min = find_min_feasible_duration(
            start_pose=b4_from, start_twist=zeros6, start_accel=zeros6,
            end_pose=b4_pose, end_twist=zeros6, end_accel=zeros6,
            geom=geom, dynamics_params=params,
        )
        if b4_min is not None:
            b4_motion = b4_min + DEFERRED_START_BUFFER_S
            trajs.append((f'B4: -> Z={target_z} (deferred #{idx+1})',
                          _make_rest_to_rest(b4_from, b4_pose,
                                             b4_motion / speed_scale)))
    trajs.append(('B4: -> Home (final return)',
                  _make_rest_to_rest([0, 0, 185, 0, 0, 0], home_pose,
                                     1.5 / speed_scale)))

    # --- C1: Borderline duration ---
    c1_pose = np.array([0, 0, 200, 0, 0, 0])
    c1_min = find_min_feasible_duration(
        start_pose=home_pose, start_twist=zeros6, start_accel=zeros6,
        end_pose=c1_pose, end_twist=zeros6, end_accel=zeros6,
        geom=geom, dynamics_params=params,
    )
    if c1_min is not None:
        # Duration just barely exceeds min + buffer (deferral triggers)
        c1_dur = (c1_min + DEFERRED_START_BUFFER_S + 0.1) / speed_scale
        trajs.append(('C1: Borderline deferred (just triggers)',
                      _make_rest_to_rest(home_pose, c1_pose, c1_dur)))
        # Duration just under threshold (no deferral)
        c1_dur_short = (c1_min + DEFERRED_START_BUFFER_S - 0.1) / speed_scale
        trajs.append(('C1: Borderline no-defer (just under)',
                      _make_rest_to_rest(c1_pose, home_pose, c1_dur_short)))

    # --- C2: Deferred during active trajectory ---
    c2_dur1 = 1.0 / speed_scale
    c2_first = _make_rest_to_rest(home_pose, [0, 0, 190, 0, 0, 0], c2_dur1)
    trajs.append(('C2: Home -> Z=190 (active traj)', c2_first))
    # Deferred target sent mid-motion
    sp2, st2, sa2 = evaluate(c2_first, 0.5 * c2_dur1)
    c2_min = find_min_feasible_duration(
        start_pose=sp2, start_twist=st2, start_accel=sa2,
        end_pose=np.array([0, 0, 210, 0, 0, 0]),
        end_twist=zeros6, end_accel=zeros6,
        geom=geom, dynamics_params=params,
    )
    if c2_min is not None:
        c2_motion = c2_min + DEFERRED_START_BUFFER_S
        trajs.append(('C2: Splice -> Z=210 (deferred mid-motion)',
                      create_trajectory(
                          start_pose=sp2, start_twist=st2, start_accel=sa2,
                          end_pose=np.array([0, 0, 210, 0, 0, 0]),
                          end_twist=zeros6, end_accel=zeros6,
                          duration=c2_motion / speed_scale, t_start=0.0,
                      )))
    trajs.append(('C2: Z=210 -> Home (return)',
                  _make_rest_to_rest([0, 0, 210, 0, 0, 0], home_pose,
                                     1.5 / speed_scale)))

    # Post-process: insert smooth transition trajectories at any
    # junctions where consecutive trajectories don't connect (e.g.
    # splice tests where the preview chains the pre-splice trajectory
    # end with the post-splice trajectory start).
    trajs = _insert_transitions(trajs, geom, params)

    return trajs


# ---------------------------------------------------------------------------
# Dry-run / preview
# ---------------------------------------------------------------------------

def run_dry_run(speed_scale: float, show_preview: bool,
                tests_to_run: list[str]) -> None:
    """Run feasibility checks on deferred start trajectories without CAN."""
    print("\n" + "=" * 60)
    print("DRY RUN: Deferred Start Feasibility Checks (no hardware)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    threshold = get_tracking_threshold(speed_scale)
    print(f"\n  Speed scale: {speed_scale}")
    print(f"  Tracking threshold: {threshold} mm")
    print(f"  Tests: {', '.join(tests_to_run)}")
    print()

    trajs = build_deferred_start_test_trajectories(geom, speed_scale)

    # Filter to requested tests
    test_prefixes = set()
    for name in tests_to_run:
        test_prefixes.add(name)
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

    # Sequence continuity check — verify no jumps between trajectories
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


# Tolerances for inter-trajectory continuity (same as dynamic_target_test).
_CONT_POS_TOL_MM   = 0.5     # mm
_CONT_POS_TOL_RAD  = 0.005   # rad
_CONT_VEL_TOL_MM   = 5.0     # mm/s

# Transition safety margin (same as dynamic_target_test).
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
    """
    if len(trajs) < 2:
        return trajs

    result = [trajs[0]]

    for i in range(1, len(trajs)):
        _, traj_prev = result[-1]
        label_next, traj_next = trajs[i]

        se = traj_prev.end_state
        ss = traj_next.start_state
        pose_end, twist_end = se[:6], se[6:12]
        pose_start, twist_start = ss[:6], ss[6:12]

        dp = np.linalg.norm(pose_end[:3] - pose_start[:3])
        dr = np.linalg.norm(pose_end[3:] - pose_start[3:])
        dv = np.linalg.norm(twist_end[:3] - twist_start[:3])

        needs_transition = (dp > _CONT_POS_TOL_MM
                            or dr > _CONT_POS_TOL_RAD
                            or dv > _CONT_VEL_TOL_MM)

        if needs_transition:
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
                label_prev = result[-1][0]
                print(f"  WARNING: transition {label_prev} -> {label_next} "
                      f"infeasible, skipping")
                result.append(trajs[i])
                continue

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
            label_prev = result[-1][0]
            result.append((f'~transition~ ({label_prev} -> {label_next})',
                           transition))

        result.append(trajs[i])

    return result


def _filter_trajs(
    trajs: list[tuple[str, QuinticTrajectory]],
    test_prefixes: set[str],
) -> list[tuple[str, QuinticTrajectory]]:
    """Filter trajectory list to requested tests, keeping transitions.

    Keeps any trajectory whose label starts with a requested prefix
    AND any transition trajectory whose neighbours are both in the
    kept set.
    """
    # First pass: mark which indices are test trajectories we want
    keep = set()
    for i, (label, _) in enumerate(trajs):
        if any(label.startswith(p) for p in test_prefixes):
            keep.add(i)

    # Second pass: keep transitions that bridge two kept trajectories
    for i, (label, _) in enumerate(trajs):
        if label.startswith('~transition~'):
            prev_kept = (i - 1) in keep
            next_kept = (i + 1) in keep
            if prev_kept and next_kept:
                keep.add(i)

    return [trajs[i] for i in sorted(keep)]


# ---------------------------------------------------------------------------
# Test helper: run a dynamic-target test with jump analysis
# ---------------------------------------------------------------------------

def _return_to_active_home(harness: PlatformTestHarness,
                           geom: StewartGeometry,
                           params: DynamicsParams) -> None:
    """Smoothly return the platform to active home after a test.

    Switches to TRAP_TRAJ mode (which initialises from the current
    encoder position) and ramps to the active home pose.  This MUST
    be called after every execution loop before starting the next test
    or idling — it guarantees the commanded position never jumps.
    """
    print("  Returning to active home (TRAP_TRAJ ramp)...")
    move_to_active_home(harness, geom, params)


def _run_test_with_jump_check(
    harness: PlatformTestHarness,
    targets: list[dict],
    test_name: str,
    speed_scale: float,
    extra_meta: dict | None = None,
    max_duration_s: float = 30.0,
    check_final_home: bool = False,
    check_accepted: int | None = None,
    check_rejected: int | None = None,
) -> bool:
    """Run a dynamic target test and apply both tracking and jump analysis.

    The platform is moved to active home (TRAP_TRAJ ramp) both before
    and after the test execution, ensuring no position jumps between
    tests.

    Returns True if all checks pass.
    """
    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(mgr.home_pose)

    threshold = get_tracking_threshold(speed_scale)

    meta = build_run_metadata(
        test_name, speed_scale, geom, params,
        tracking_threshold_mm=threshold,
        extra=extra_meta or {},
    )

    prepare_harness(harness)
    move_to_active_home(harness, geom, params)
    switch_to_passthrough(harness)
    interactive_pause(f"Press Enter to execute {test_name}...")

    print(f"\n  Executing {test_name} (speed_scale={speed_scale})...")
    try:
        events, log = execute_with_dynamic_targets(
            harness, mgr, geom, params, limits, targets,
            max_duration_s=max_duration_s, label=test_name, metadata=meta)
    finally:
        # ALWAYS return to active home smoothly, even on exception.
        # The platform may be at any pose after the execution loop —
        # this TRAP_TRAJ ramp ensures no position discontinuity.
        _return_to_active_home(harness, geom, params)

    # Filter startup samples
    filtered = filter_startup_samples(log)
    n_filtered = len(log.timestamps) - len(filtered.timestamps)
    if n_filtered:
        print(f"  ({n_filtered} startup samples filtered, dt > "
              f"{_STARTUP_DT_THRESHOLD_S * 1e3:.0f}ms)")

    # Tracking error analysis
    analysis = analyze_log(filtered) if filtered.timestamps else {}
    passed = True

    if analysis:
        print_analysis(test_name, analysis)

        # Tracking error check
        if analysis['max_error_mm'] >= threshold:
            print(f"  [FAIL] Tracking error {analysis['max_error_mm']:.3f} mm "
                  f">= {threshold} mm")
            passed = False
        else:
            print(f"  [OK] Tracking error {analysis['max_error_mm']:.3f} mm "
                  f"< {threshold} mm")
    else:
        print(f"  [FAIL] No data recorded")
        passed = False

    # Inter-sample jump check (PRIMARY glitch detector)
    jump_ok, max_jump, jump_violations = check_intersample_jumps(filtered)
    print_jump_analysis(jump_ok, max_jump, jump_violations, test_name)
    if not jump_ok:
        passed = False

    # Optional: check final pose near home
    if check_final_home and analysis:
        final_pose = mgr.current_pose_6dof
        home_err = np.linalg.norm(final_pose - mgr.home_pose)
        final_state = mgr.state
        print(f"\n  Final state: {final_state.value}")
        print(f"  Error from home: {home_err:.3f} mm")
        if home_err >= 1.0:
            print(f"  [FAIL] Not at home (err={home_err:.3f} mm >= 1.0 mm)")
            passed = False
        if final_state != TrajectoryState.IDLE:
            print(f"  [FAIL] Final state {final_state.value} != IDLE")
            passed = False

    # Optional: check event acceptance/rejection counts
    if check_accepted is not None:
        n_accepted = sum(1 for e in events if e['accepted'])
        if n_accepted != check_accepted:
            print(f"  [FAIL] Expected {check_accepted} accepted, "
                  f"got {n_accepted}")
            passed = False

    if check_rejected is not None:
        n_rejected = sum(1 for e in events if not e['accepted'])
        if n_rejected != check_rejected:
            print(f"  [FAIL] Expected {check_rejected} rejected, "
                  f"got {n_rejected}")
            passed = False

    # Save log
    if analysis:
        log.metadata['analysis'] = analysis
    log.metadata['jump_analysis'] = {
        'ok': jump_ok,
        'max_jump_mm': max_jump,
        'n_violations': len(jump_violations),
    }
    log.metadata['events'] = events
    log.metadata['result'] = 'PASS' if passed else 'FAIL'
    _auto_save_log(log, test_name, speed_scale)

    status = 'PASS' if passed else 'FAIL'
    print(f"\n  {status}: {test_name}")
    return passed


# ---------------------------------------------------------------------------
# Phase A: Regression tests
# ---------------------------------------------------------------------------

def test_A1(harness: PlatformTestHarness,
            speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """A1: Short-duration target — no deferral expected.

    Regression test: identical to DT1. Verifies that short-duration
    targets still work correctly after deferred start changes.
    """
    print("\n" + "=" * 60)
    print("A1: Short-duration target (regression, no deferral)")
    print("=" * 60)

    duration = 1.0 / speed_scale
    targets = [{
        'pos': [0, 0, 200],
        'quat': [1, 0, 0, 0],
        'vel': [0, 0, 0],
        'delay_s': 0.0,
        'duration_s': duration,
    }]

    return _run_test_with_jump_check(
        harness, targets, 'A1', speed_scale,
        extra_meta={'duration_s': duration, 'target_pos': [0, 0, 200]},
        max_duration_s=duration + 5.0,
    )


def test_A2(harness: PlatformTestHarness,
            speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """A2: Auto-return with nonzero velocity — regression test."""
    print("\n" + "=" * 60)
    print("A2: Auto-return with velocity (regression)")
    print("=" * 60)

    duration = 1.0 / speed_scale
    vel_z = 30 * speed_scale
    targets = [{
        'pos': [0, 0, 190],
        'quat': [1, 0, 0, 0],
        'vel': [0, 0, vel_z],
        'delay_s': 0.0,
        'duration_s': duration,
    }]

    return _run_test_with_jump_check(
        harness, targets, 'A2', speed_scale,
        extra_meta={'duration_s': duration, 'vel_z': vel_z},
        max_duration_s=duration * 3 + 5.0,
        check_final_home=True,
    )


def test_A3(harness: PlatformTestHarness,
            speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """A3: Mid-motion splice — regression test."""
    print("\n" + "=" * 60)
    print("A3: Mid-motion splice (regression)")
    print("=" * 60)

    base_dur = 1.5 / speed_scale
    targets = [
        {
            'pos': [0, 0, 200],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': 0.0,
            'duration_s': base_dur,
        },
        {
            'pos': [15, 0, 195],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': 0.7,
            'duration_s': base_dur,
        },
    ]

    return _run_test_with_jump_check(
        harness, targets, 'A3', speed_scale,
        extra_meta={'base_dur_s': base_dur, 'replan_delay_s': 0.7},
        max_duration_s=base_dur * 2 + 5.0,
        check_accepted=2,
    )


# ---------------------------------------------------------------------------
# Phase B: Deferred start validation
# ---------------------------------------------------------------------------

def test_B1(harness: PlatformTestHarness,
            speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """B1: Basic deferred start from IDLE.

    Send a target with 10s arrival time. The min_feasible duration
    is ~1s, so the trajectory should defer start by ~7s, hold at home,
    then move for ~3s (min_feasible + 2s buffer).

    Key check: NO position jump at the hold-to-move transition.
    """
    print("\n" + "=" * 60)
    print("B1: Basic deferred start from IDLE")
    print("=" * 60)

    arrival_duration = 10.0 / speed_scale
    targets = [{
        'pos': [0, 0, 200],
        'quat': [1, 0, 0, 0],
        'vel': [0, 0, 0],
        'delay_s': 0.0,
        'duration_s': arrival_duration,
    }]

    return _run_test_with_jump_check(
        harness, targets, 'B1', speed_scale,
        extra_meta={'arrival_duration_s': arrival_duration,
                    'target_pos': [0, 0, 200],
                    'expected_deferral': True},
        max_duration_s=arrival_duration + 5.0,
    )


def test_B2(harness: PlatformTestHarness,
            speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """B2: Deferred start with different move sizes.

    Three consecutive deferred-start targets with different Z offsets
    (small=10mm, medium=30mm, large=50mm). Each uses 8s arrival from
    IDLE, ensuring deferral triggers for all.

    Validates that deferred start works for different trajectory shapes.
    """
    print("\n" + "=" * 60)
    print("B2: Deferred start — different move sizes")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)

    results = []
    for label, target_z in [('small', 180), ('medium', 200), ('large', 220)]:
        print(f"\n  --- B2-{label}: Home -> Z={target_z} ---")

        mgr = TrajectoryManager(geom, params)
        mgr.set_hold_pose(mgr.home_pose)

        threshold = get_tracking_threshold(speed_scale)
        arrival_dur = 8.0 / speed_scale

        targets = [{
            'pos': [0, 0, target_z],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': 0.0,
            'duration_s': arrival_dur,
        }]

        meta = build_run_metadata(
            f'B2-{label}', speed_scale, geom, params,
            tracking_threshold_mm=threshold,
            extra={'target_z': target_z, 'arrival_dur': arrival_dur},
        )

        prepare_harness(harness)
        move_to_active_home(harness, geom, params)
        switch_to_passthrough(harness)
        interactive_pause(f"Press Enter to execute B2-{label}...")

        print(f"  Executing B2-{label}...")
        try:
            events, log = execute_with_dynamic_targets(
                harness, mgr, geom, params, limits, targets,
                max_duration_s=arrival_dur + 5.0,
                label=f'B2-{label}', metadata=meta)
        finally:
            # ALWAYS return to home smoothly before next sub-test
            _return_to_active_home(harness, geom, params)

        filtered = filter_startup_samples(log)
        analysis = analyze_log(filtered) if filtered.timestamps else {}

        sub_passed = True
        if analysis:
            print_analysis(f'B2-{label}', analysis)
            if analysis['max_error_mm'] >= threshold:
                sub_passed = False
        else:
            sub_passed = False

        jump_ok, max_jump, violations = check_intersample_jumps(filtered)
        print_jump_analysis(jump_ok, max_jump, violations, f'B2-{label}')
        if not jump_ok:
            sub_passed = False

        log.metadata['analysis'] = analysis
        log.metadata['result'] = 'PASS' if sub_passed else 'FAIL'
        _auto_save_log(log, f'B2_{label}', speed_scale)

        status = 'PASS' if sub_passed else 'FAIL'
        print(f"  {status}: B2-{label}")
        results.append(sub_passed)

    passed = all(results)
    print(f"\n  {'PASS' if passed else 'FAIL'}: B2 (all sizes)")
    return passed


def test_B3(harness: PlatformTestHarness,
            speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """B3: Deferred start + auto-return.

    Deferred target with nonzero velocity at target. After arrival,
    auto-return to home should trigger smoothly.
    """
    print("\n" + "=" * 60)
    print("B3: Deferred start + auto-return")
    print("=" * 60)

    arrival_dur = 8.0 / speed_scale
    vel_z = 20 * speed_scale
    targets = [{
        'pos': [0, 0, 195],
        'quat': [1, 0, 0, 0],
        'vel': [0, 0, vel_z],
        'delay_s': 0.0,
        'duration_s': arrival_dur,
    }]

    return _run_test_with_jump_check(
        harness, targets, 'B3', speed_scale,
        extra_meta={'arrival_dur_s': arrival_dur, 'vel_z': vel_z,
                    'expected_deferral': True},
        max_duration_s=arrival_dur * 2 + 5.0,
        check_final_home=True,
    )


def test_B4(harness: PlatformTestHarness,
            speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """B4: Repeated deferred targets.

    Send 3 consecutive far-future targets. Each one should defer,
    hold, then move smoothly. Tests that deferral logic handles
    the IDLE → EXECUTING → COMPLETE → IDLE cycle repeatedly.
    """
    print("\n" + "=" * 60)
    print("B4: Repeated deferred targets (3 consecutive)")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(mgr.home_pose)

    threshold = get_tracking_threshold(speed_scale)

    # 3 targets, each 10s from submission time, staggered
    arrival_dur = 10.0 / speed_scale
    z_targets = [190, 200, 185]
    targets = []
    cumulative_delay = 0.0
    for z in z_targets:
        targets.append({
            'pos': [0, 0, z],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': cumulative_delay,
            'duration_s': arrival_dur,
        })
        # Next target sent after this one arrives + 2s settle
        cumulative_delay += arrival_dur + 2.0

    meta = build_run_metadata(
        'B4', speed_scale, geom, params,
        tracking_threshold_mm=threshold,
        extra={'z_targets': z_targets, 'arrival_dur_s': arrival_dur},
    )

    prepare_harness(harness)
    move_to_active_home(harness, geom, params)
    switch_to_passthrough(harness)
    interactive_pause("Press Enter to execute B4...")

    print(f"  Executing B4 ({len(targets)} targets, "
          f"speed_scale={speed_scale})...")
    try:
        events, log = execute_with_dynamic_targets(
            harness, mgr, geom, params, limits, targets,
            max_duration_s=cumulative_delay + arrival_dur + 5.0,
            label='B4', metadata=meta)
    finally:
        # ALWAYS return to home smoothly after execution
        _return_to_active_home(harness, geom, params)

    # Analyze
    filtered = filter_startup_samples(log)
    analysis = analyze_log(filtered) if filtered.timestamps else {}
    passed = True

    if analysis:
        print_analysis('B4', analysis)
        if analysis['max_error_mm'] >= threshold:
            passed = False
    else:
        passed = False

    jump_ok, max_jump, violations = check_intersample_jumps(filtered)
    print_jump_analysis(jump_ok, max_jump, violations, 'B4')
    if not jump_ok:
        passed = False

    n_accepted = sum(1 for e in events if e['accepted'])
    print(f"\n  Targets accepted: {n_accepted}/{len(targets)}")

    log.metadata['analysis'] = analysis
    log.metadata['events'] = events
    log.metadata['result'] = 'PASS' if passed else 'FAIL'
    _auto_save_log(log, 'B4', speed_scale)

    status = 'PASS' if passed else 'FAIL'
    print(f"\n  {status}: B4 Repeated deferred targets")
    return passed


# ---------------------------------------------------------------------------
# Phase C: Edge cases
# ---------------------------------------------------------------------------

def test_C1(harness: PlatformTestHarness,
            speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """C1: Borderline duration — just barely triggers deferral.

    The arrival time is set to min_feasible + DEFERRED_START_BUFFER_S + 0.1s.
    This should just barely trigger deferred start, with a very short
    hold phase before motion begins.
    """
    print("\n" + "=" * 60)
    print("C1: Borderline deferred start duration")
    print("=" * 60)

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    home_pose = np.array([0.0, 0.0, _HOME_Z, 0.0, 0.0, 0.0])
    target_pose = np.array([0, 0, 200, 0, 0, 0])
    zeros6 = np.zeros(6)

    # Find minimum feasible duration
    min_dur = find_min_feasible_duration(
        start_pose=home_pose, start_twist=zeros6, start_accel=zeros6,
        end_pose=target_pose, end_twist=zeros6, end_accel=zeros6,
        geom=geom, dynamics_params=params,
    )
    if min_dur is None:
        print("  FAIL: Cannot find min feasible duration")
        return False

    # Just barely triggers deferral
    borderline_dur = (min_dur + DEFERRED_START_BUFFER_S + 0.1) / speed_scale
    print(f"  min_feasible: {min_dur:.3f}s")
    print(f"  buffer: {DEFERRED_START_BUFFER_S}s")
    print(f"  arrival duration: {borderline_dur:.3f}s (just triggers deferral)")

    targets = [{
        'pos': [0, 0, 200],
        'quat': [1, 0, 0, 0],
        'vel': [0, 0, 0],
        'delay_s': 0.0,
        'duration_s': borderline_dur,
    }]

    return _run_test_with_jump_check(
        harness, targets, 'C1', speed_scale,
        extra_meta={'min_feasible_s': min_dur,
                    'borderline_dur_s': borderline_dur,
                    'expected_deferral': True},
        max_duration_s=borderline_dur + 5.0,
    )


def test_C2(harness: PlatformTestHarness,
            speed_scale: float = DEFAULT_SPEED_SCALE) -> bool:
    """C2: Deferred start during active trajectory.

    First target starts immediately (short duration). While it's
    executing, a second target with a far-future arrival is sent.
    The second target should defer start, sampling state from the
    active trajectory at the future t_start.

    This tests the most complex deferral path: EXECUTING state
    with active trajectory interpolation at future t_start.
    """
    print("\n" + "=" * 60)
    print("C2: Deferred start during active trajectory")
    print("=" * 60)

    short_dur = 1.0 / speed_scale
    deferred_arrival = 10.0 / speed_scale
    targets = [
        {  # First target: immediate short move
            'pos': [0, 0, 190],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': 0.0,
            'duration_s': short_dur,
        },
        {  # Second target: far future, should defer
            'pos': [0, 0, 210],
            'quat': [1, 0, 0, 0],
            'vel': [0, 0, 0],
            'delay_s': 0.3,  # Sent 0.3s after first (during execution)
            'duration_s': deferred_arrival,
        },
    ]

    return _run_test_with_jump_check(
        harness, targets, 'C2', speed_scale,
        extra_meta={'short_dur_s': short_dur,
                    'deferred_arrival_s': deferred_arrival,
                    'expected_deferral': True},
        max_duration_s=deferred_arrival + 5.0,
        check_accepted=2,
    )


# ---------------------------------------------------------------------------
# Test registry
# ---------------------------------------------------------------------------

TEST_MAP = {
    'A1': test_A1,
    'A2': test_A2,
    'A3': test_A3,
    'B1': test_B1,
    'B2': test_B2,
    'B3': test_B3,
    'B4': test_B4,
    'C1': test_C1,
    'C2': test_C2,
}

ALL_TESTS = list(TEST_MAP.keys())

TEST_GROUPS = {
    'A': ['A1', 'A2', 'A3'],
    'B': ['B1', 'B2', 'B3', 'B4'],
    'C': ['C1', 'C2'],
}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Deferred Start Hardware Tests',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests available:
  Phase A — Regression (existing functionality still works)
    A1   Short-duration target (no deferral) — same as DT1
    A2   Auto-return from velocity target — same as DT2
    A3   Mid-motion splice — same as DT3

  Phase B — Deferred Start Validation
    B1   Basic deferred start from IDLE (10s arrival)
    B2   Different move sizes (small/medium/large Z offsets)
    B3   Deferred start + auto-return (nonzero velocity at target)
    B4   Repeated deferred targets (3 consecutive far-future targets)

  Phase C — Edge Cases
    C1   Borderline duration (just barely triggers deferral)
    C2   Deferred start during active trajectory

Modes:
  --dry-run             Feasibility checks only, no CAN connection
  --preview             Show 3D viewer before hardware execution
  --dry-run --preview   Feasibility + 3D viewer, no hardware
        """)
    parser.add_argument('--home', action='store_true',
                        help='Home the robot before testing')
    parser.add_argument('--test', nargs='+', default=None,
                        choices=['all'] + ALL_TESTS,
                        help='Individual tests to run (e.g. A1 B1 C2)')
    parser.add_argument('--phase', nargs='+', default=None,
                        choices=list(TEST_GROUPS.keys()),
                        help='Run all tests in phase(s) (A, B, C)')
    parser.add_argument('--speed-scale', type=float,
                        default=DEFAULT_SPEED_SCALE,
                        help=f'Speed scaling factor '
                             f'(default: {DEFAULT_SPEED_SCALE})')
    parser.add_argument('--dry-run', action='store_true',
                        help='Feasibility checks only, no CAN connection')
    parser.add_argument('--preview', action='store_true',
                        help='Show 3D viewer preview before hardware execution')
    parser.add_argument('--can-channel', default='can0',
                        help='CAN interface name (default: can0)')
    args = parser.parse_args()

    # Determine which tests to run
    if args.test is not None:
        if 'all' in args.test:
            tests_to_run = ALL_TESTS
        else:
            tests_to_run = args.test
    elif args.phase is not None:
        tests_to_run = []
        for phase in args.phase:
            tests_to_run.extend(TEST_GROUPS[phase])
    else:
        tests_to_run = ALL_TESTS

    print("=" * 60)
    print("Deferred Start Hardware Tests")
    print("=" * 60)
    print(f"  Speed scale:  {args.speed_scale}")
    print(f"  Tests:        {', '.join(tests_to_run)}")
    print(f"  CAN channel:  {args.can_channel}")

    # Dry-run mode
    if args.dry_run:
        run_dry_run(args.speed_scale, args.preview, tests_to_run)
        return

    # Pre-flight continuity check (mandatory before hardware).
    # Builds all trajectories offline and verifies no position jumps
    # at any junction in the full sequence.
    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    trajs = build_deferred_start_test_trajectories(
        geom, speed_scale=args.speed_scale)
    test_prefixes = set(tests_to_run)
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
        if filtered:
            try:
                from trajectory_viewer import preview_test_sequence
                print("\n  Launching 3D preview "
                      "(close window to continue)...")
                preview_test_sequence(filtered, geom, params)
            except ImportError:
                print("  WARNING: matplotlib not available, "
                      "skipping preview")

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

        # geom and params already created for pre-flight check above

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
                # On exception, smoothly return to home before
                # attempting the next test (or stow/idle).
                try:
                    _return_to_active_home(harness, geom, params)
                except Exception:
                    # If return-to-home also fails, fall back to
                    # safe_idle_all which stows via TRAP_TRAJ.
                    safe_idle_all(harness)

        # Summary
        print("\n" + "=" * 60)
        print("Deferred Start Test Summary")
        print("=" * 60)

        for group_name, group_tests in TEST_GROUPS.items():
            group_ran = [t for t in group_tests if t in results]
            if group_ran:
                print(f"\n  Phase {group_name}:")
                for name in group_ran:
                    status = 'PASS' if results.get(name) else 'FAIL'
                    print(f"    {name}: {status}")

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
