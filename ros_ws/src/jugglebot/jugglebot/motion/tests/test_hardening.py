"""Phase 6 verification tests for hardening & operational readiness.

Tests:
  1. Workspace limits construction — verify soft/hard margins from geometry
  2. Workspace check: OK status — normal pose returns OK with scale 1.0
  3. Workspace check: soft limit (leg extension) — returns speed degradation
  4. Workspace check: hard limit (leg extension) — returns scale 0.0
  5. Workspace check: soft limit (condition number) — returns speed degradation
  6. Workspace check: hard limit (condition number) — returns scale 0.0
  7. Workspace check: combined soft limits — min of individual scales
  8. Trajectory progress tracking — verify progress/time_remaining update
  9. Trajectory progress on cancel — verify reset to 0.0
  10. Telemetry message: Phase 6 fields — verify new fields are present
  11. Feasibility vs workspace limits consistency — feasible trajectories
      stay within workspace soft limits
  12. Workspace limits: boundary exactness — verify exact boundary transitions

Run:  python -m jugglebot.motion.tests.test_hardening
"""

from __future__ import annotations

import sys
import time

import numpy as np
from numpy.linalg import norm

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.ik_solver import (
    pose_to_leg_lengths,
    rotvec_to_rot_matrix,
    compute_jacobian,
)
from jugglebot.motion.conversions import extensions_mm_to_revs
from jugglebot.motion.workspace import (
    LEG_SOFT_MARGIN_MM,
    LEG_HARD_MARGIN_MM,
    COND_SOFT_FACTOR,
    COND_HARD_FACTOR,
    WorkspaceLimits,
    WorkspaceStatus,
    WorkspaceCheck,
    check_workspace_limits,
    compute_condition_number,
)
from jugglebot.motion.feasibility import check_feasibility
from jugglebot.motion.quintic import create_trajectory, evaluate
from jugglebot.motion.trajectory_manager import TrajectoryManager, TrajectoryState
try:
    from jugglebot.motion.ipc import make_telemetry
    _HAS_IPC = True
except ImportError:
    _HAS_IPC = False


def _header(name: str):
    print(f"\n{'='*70}")
    print(f"  {name}")
    print(f"{'='*70}")


# ---------------------------------------------------------------------------
# Test 1: Workspace limits construction
# ---------------------------------------------------------------------------

def test_workspace_limits_construction():
    """Verify WorkspaceLimits are computed correctly from geometry."""
    _header("Test 1: Workspace limits construction")
    geom = StewartGeometry()
    limits = WorkspaceLimits.from_geometry(geom)

    stroke = geom.leg_stroke_mm
    assert limits.leg_stroke_mm == stroke
    assert limits.leg_soft_min_mm == LEG_SOFT_MARGIN_MM
    assert limits.leg_soft_max_mm == stroke - LEG_SOFT_MARGIN_MM
    assert limits.leg_hard_min_mm == LEG_HARD_MARGIN_MM
    assert limits.leg_hard_max_mm == stroke - LEG_HARD_MARGIN_MM

    cond_home = compute_condition_number(np.zeros(3), np.eye(3), geom)
    assert abs(limits.cond_home - cond_home) < 1e-6
    assert abs(limits.cond_soft - COND_SOFT_FACTOR * cond_home) < 1e-6
    assert abs(limits.cond_hard - COND_HARD_FACTOR * cond_home) < 1e-6

    print(f"  Stroke:    {stroke:.1f} mm")
    print(f"  Soft:      [{limits.leg_soft_min_mm:.1f}, {limits.leg_soft_max_mm:.1f}] mm")
    print(f"  Hard:      [{limits.leg_hard_min_mm:.1f}, {limits.leg_hard_max_mm:.1f}] mm")
    print(f"  Cond home: {limits.cond_home:.1f}")
    print(f"  Cond soft: {limits.cond_soft:.1f}")
    print(f"  Cond hard: {limits.cond_hard:.1f}")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 2: Workspace check: OK status
# ---------------------------------------------------------------------------

def test_workspace_check_ok():
    """Verify normal pose returns OK with speed_scale 1.0."""
    _header("Test 2: Workspace check: OK status")
    geom = StewartGeometry()
    limits = WorkspaceLimits.from_geometry(geom)

    # Home pose: extensions are ~26-29 mm, well within limits
    pos = np.zeros(3)
    rot = np.eye(3)
    ext = pose_to_leg_lengths(pos, rot, geom)
    cond = compute_condition_number(pos, rot, geom)

    result = check_workspace_limits(ext, cond, limits)
    assert result.status == WorkspaceStatus.OK, f"Expected OK, got {result.status}"
    assert result.speed_scale == 1.0, f"Expected scale 1.0, got {result.speed_scale}"
    assert len(result.violations) == 0, f"Expected no violations, got {result.violations}"

    print(f"  Extensions: {np.array2string(ext, precision=1)} mm")
    print(f"  Cond:       {cond:.1f}")
    print(f"  Status:     {result.status.value}")
    print(f"  Scale:      {result.speed_scale}")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 3: Workspace check: soft limit (leg extension)
# ---------------------------------------------------------------------------

def test_workspace_check_soft_leg():
    """Verify soft limit on leg extension triggers speed degradation."""
    _header("Test 3: Workspace check: soft limit (leg extension)")
    geom = StewartGeometry()
    limits = WorkspaceLimits.from_geometry(geom)

    # Put leg 0 at 10 mm (inside soft zone [5, 15] but outside hard [5])
    ext = np.array([10.0, 50.0, 50.0, 50.0, 50.0, 50.0])
    cond = limits.cond_home  # normal cond

    result = check_workspace_limits(ext, cond, limits)
    assert result.status == WorkspaceStatus.SOFT_LIMIT, \
        f"Expected SOFT_LIMIT, got {result.status}"
    assert 0.0 < result.speed_scale < 1.0, \
        f"Expected degraded scale, got {result.speed_scale}"
    assert len(result.violations) > 0

    # Also test overextension soft limit
    ext_high = np.array([50.0, 50.0, 50.0, 50.0, 50.0,
                          limits.leg_soft_max_mm + 2.0])
    result_high = check_workspace_limits(ext_high, cond, limits)
    assert result_high.status == WorkspaceStatus.SOFT_LIMIT

    print(f"  Underextended leg 0 at 10.0 mm:")
    print(f"    Status: {result.status.value}, scale: {result.speed_scale:.3f}")
    print(f"    Violations: {result.violations}")
    print(f"  Overextended leg 5 at {limits.leg_soft_max_mm + 2.0:.1f} mm:")
    print(f"    Status: {result_high.status.value}, scale: {result_high.speed_scale:.3f}")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 4: Workspace check: hard limit (leg extension)
# ---------------------------------------------------------------------------

def test_workspace_check_hard_leg():
    """Verify hard limit on leg extension returns scale 0.0."""
    _header("Test 4: Workspace check: hard limit (leg extension)")
    geom = StewartGeometry()
    limits = WorkspaceLimits.from_geometry(geom)

    # Put leg 0 at 3 mm (below hard min of 5 mm)
    ext = np.array([3.0, 50.0, 50.0, 50.0, 50.0, 50.0])
    cond = limits.cond_home

    result = check_workspace_limits(ext, cond, limits)
    assert result.status == WorkspaceStatus.HARD_LIMIT, \
        f"Expected HARD_LIMIT, got {result.status}"
    assert result.speed_scale == 0.0, f"Expected scale 0.0, got {result.speed_scale}"

    # Also test overextension hard limit
    ext_high = np.array([50.0, 50.0, 50.0, 50.0, 50.0,
                          limits.leg_hard_max_mm + 2.0])
    result_high = check_workspace_limits(ext_high, cond, limits)
    assert result_high.status == WorkspaceStatus.HARD_LIMIT
    assert result_high.speed_scale == 0.0

    print(f"  Leg 0 at 3.0 mm (below hard min {limits.leg_hard_min_mm:.1f}):")
    print(f"    Status: {result.status.value}, scale: {result.speed_scale}")
    print(f"  Leg 5 at {limits.leg_hard_max_mm + 2.0:.1f} mm "
          f"(above hard max {limits.leg_hard_max_mm:.1f}):")
    print(f"    Status: {result_high.status.value}, scale: {result_high.speed_scale}")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 5: Workspace check: soft limit (condition number)
# ---------------------------------------------------------------------------

def test_workspace_check_soft_cond():
    """Verify soft limit on condition number triggers speed degradation."""
    _header("Test 5: Workspace check: soft limit (condition number)")
    geom = StewartGeometry()
    limits = WorkspaceLimits.from_geometry(geom)

    # Normal extensions, but elevated cond number
    ext = np.array([50.0] * 6)
    cond_mid = (limits.cond_soft + limits.cond_hard) / 2.0  # midpoint of soft-hard

    result = check_workspace_limits(ext, cond_mid, limits)
    assert result.status == WorkspaceStatus.SOFT_LIMIT, \
        f"Expected SOFT_LIMIT, got {result.status}"
    assert 0.0 < result.speed_scale < 1.0
    expected_scale = 0.5  # midpoint of ramp
    assert abs(result.speed_scale - expected_scale) < 0.01, \
        f"Expected scale ~{expected_scale}, got {result.speed_scale}"

    print(f"  Cond soft: {limits.cond_soft:.1f}")
    print(f"  Cond hard: {limits.cond_hard:.1f}")
    print(f"  Test cond: {cond_mid:.1f}")
    print(f"  Status:    {result.status.value}")
    print(f"  Scale:     {result.speed_scale:.3f} (expected ~{expected_scale})")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 6: Workspace check: hard limit (condition number)
# ---------------------------------------------------------------------------

def test_workspace_check_hard_cond():
    """Verify hard limit on condition number returns scale 0.0."""
    _header("Test 6: Workspace check: hard limit (condition number)")
    geom = StewartGeometry()
    limits = WorkspaceLimits.from_geometry(geom)

    ext = np.array([50.0] * 6)
    cond_over = limits.cond_hard + 10.0

    result = check_workspace_limits(ext, cond_over, limits)
    assert result.status == WorkspaceStatus.HARD_LIMIT
    assert result.speed_scale == 0.0

    print(f"  Cond hard: {limits.cond_hard:.1f}")
    print(f"  Test cond: {cond_over:.1f}")
    print(f"  Status:    {result.status.value}")
    print(f"  Scale:     {result.speed_scale}")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 7: Workspace check: combined soft limits
# ---------------------------------------------------------------------------

def test_workspace_check_combined_soft():
    """Verify combined soft limits take minimum of individual scales."""
    _header("Test 7: Workspace check: combined soft limits")
    geom = StewartGeometry()
    limits = WorkspaceLimits.from_geometry(geom)

    # Leg soft and cond soft simultaneously
    ext = np.array([10.0, 50.0, 50.0, 50.0, 50.0, 50.0])  # leg 0 soft
    cond_mid = (limits.cond_soft + limits.cond_hard) / 2.0  # cond soft

    result = check_workspace_limits(ext, cond_mid, limits)
    assert result.status == WorkspaceStatus.SOFT_LIMIT

    # Get individual scales
    result_leg_only = check_workspace_limits(ext, limits.cond_home, limits)
    result_cond_only = check_workspace_limits(
        np.array([50.0] * 6), cond_mid, limits)

    expected_combined = min(result_leg_only.speed_scale,
                            result_cond_only.speed_scale)
    assert abs(result.speed_scale - expected_combined) < 1e-6, \
        f"Combined scale {result.speed_scale} != min({result_leg_only.speed_scale}, " \
        f"{result_cond_only.speed_scale})"

    print(f"  Leg-only scale:  {result_leg_only.speed_scale:.3f}")
    print(f"  Cond-only scale: {result_cond_only.speed_scale:.3f}")
    print(f"  Combined scale:  {result.speed_scale:.3f}")
    print(f"  Expected (min):  {expected_combined:.3f}")
    print(f"  Violations: {result.violations}")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 8: Trajectory progress tracking
# ---------------------------------------------------------------------------

def test_trajectory_progress():
    """Verify TrajectoryManager.progress and time_remaining update correctly."""
    _header("Test 8: Trajectory progress tracking")
    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    zeros6 = np.zeros(6)

    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(zeros6)

    # IDLE: progress should be 0
    assert mgr.progress == 0.0
    assert mgr.time_remaining == 0.0

    # Submit trajectory
    t_start = 100.0
    duration = 2.0
    end_pose = np.array([0, 0, 20, 0, 0, 0], dtype=float)
    traj = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=duration, t_start=t_start)
    mgr.submit(traj)

    # At 25% through
    mgr.evaluate(t_start + 0.5)
    assert abs(mgr.progress - 0.25) < 0.01, \
        f"Expected progress ~0.25, got {mgr.progress}"
    assert abs(mgr.time_remaining - 1.5) < 0.01, \
        f"Expected time_remaining ~1.5, got {mgr.time_remaining}"

    # At 75% through
    mgr.evaluate(t_start + 1.5)
    assert abs(mgr.progress - 0.75) < 0.01
    assert abs(mgr.time_remaining - 0.5) < 0.01

    # After completion
    mgr.evaluate(t_start + 2.5)
    assert mgr.progress == 1.0
    assert mgr.time_remaining == 0.0
    assert mgr.state == TrajectoryState.COMPLETE

    print(f"  Progress at t+0.5: {0.25:.2f}")
    print(f"  Progress at t+1.5: {0.75:.2f}")
    print(f"  Progress at completion: {1.0:.2f}")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 9: Trajectory progress on cancel
# ---------------------------------------------------------------------------

def test_trajectory_progress_cancel():
    """Verify progress resets to 0 on cancel."""
    _header("Test 9: Trajectory progress on cancel")
    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    zeros6 = np.zeros(6)

    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(zeros6)

    traj = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=np.array([0, 0, 20, 0, 0, 0], dtype=float),
        end_twist=zeros6, end_accel=zeros6,
        duration=2.0, t_start=100.0)
    mgr.submit(traj)

    # Evaluate mid-way
    mgr.evaluate(101.0)
    assert mgr.progress > 0.0

    # Cancel
    mgr.cancel()
    assert mgr.progress == 0.0, f"Expected 0 after cancel, got {mgr.progress}"
    assert mgr.time_remaining == 0.0

    print("  Progress after mid-evaluate: >0")
    print("  Progress after cancel: 0.0")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 10: Telemetry message: Phase 6 fields
# ---------------------------------------------------------------------------

def test_telemetry_phase6_fields():
    """Verify make_telemetry includes all Phase 6 fields when provided."""
    _header("Test 10: Telemetry message: Phase 6 fields")

    if not _HAS_IPC:
        print("  [SKIP] missing dependency: msgpack/zmq")
        return

    msg = make_telemetry(
        leg_positions=[0.0] * 6,
        leg_velocities=[0.0] * 6,
        commanded_torques=[0.0] * 6,
        loop_dt_s=0.002,
        ff_torques=[0.01] * 6,
        traj_state='executing',
        traj_progress=0.5,
        cond_number=450.0,
        workspace_status='ok',
        workspace_speed_scale=1.0,
        tracking_error_mm=[0.1, 0.2, 0.1, 0.15, 0.3, 0.1],
        fault_state=None,
    )

    assert msg['type'] == 'telemetry'
    assert 'cond_number' in msg
    assert msg['cond_number'] == 450.0
    assert 'workspace_status' in msg
    assert msg['workspace_status'] == 'ok'
    assert 'workspace_speed_scale' in msg
    assert msg['workspace_speed_scale'] == 1.0
    assert 'tracking_error_mm' in msg
    assert len(msg['tracking_error_mm']) == 6
    assert 'traj_progress' in msg
    assert msg['traj_progress'] == 0.5
    assert 'fault_state' not in msg  # None values should be omitted

    # Verify fault_state is included when set
    msg2 = make_telemetry(
        leg_positions=[0.0] * 6,
        leg_velocities=[0.0] * 6,
        commanded_torques=[0.0] * 6,
        loop_dt_s=0.002,
        fault_state='overcurrent_leg_2',
    )
    assert 'fault_state' in msg2
    assert msg2['fault_state'] == 'overcurrent_leg_2'

    print("  All Phase 6 telemetry fields present and correct")
    print("  fault_state=None correctly omitted")
    print("  fault_state='overcurrent_leg_2' correctly included")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 11: Feasibility vs workspace limits consistency
# ---------------------------------------------------------------------------

def test_feasibility_workspace_consistency():
    """Verify feasible trajectories stay within workspace soft limits."""
    _header("Test 11: Feasibility vs workspace limits consistency")
    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    limits = WorkspaceLimits.from_geometry(geom)
    zeros6 = np.zeros(6)

    trajectories = [
        ("10mm Z, 1s", np.array([0, 0, 10, 0, 0, 0], dtype=float), 1.0),
        ("50mm Z, 2s", np.array([0, 0, 50, 0, 0, 0], dtype=float), 2.0),
        ("20mm XY + 3deg, 1.5s",
         np.array([20, -10, 20, np.deg2rad(3), 0, 0], dtype=float), 1.5),
    ]

    for label, end_pose, dur in trajectories:
        traj = create_trajectory(
            start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
            end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
            duration=dur, speed_scale=1.0)

        feas = check_feasibility(traj, geom, params)
        assert feas.feasible, f"{label}: not feasible: {feas.violations}"

        # Check all sampled points against workspace limits
        n_samples = 100
        worst_status = WorkspaceStatus.OK
        for i in range(n_samples + 1):
            t = traj.t_start + (i / n_samples) * traj.duration
            pose, _, _ = evaluate(traj, t)
            pos = pose[:3]
            rot = rotvec_to_rot_matrix(pose[3:6])
            ext = pose_to_leg_lengths(pos, rot, geom)
            cond = compute_condition_number(pos, rot, geom)
            ws = check_workspace_limits(ext, cond, limits)
            if ws.status == WorkspaceStatus.HARD_LIMIT:
                worst_status = WorkspaceStatus.HARD_LIMIT
                break
            elif ws.status == WorkspaceStatus.SOFT_LIMIT:
                worst_status = WorkspaceStatus.SOFT_LIMIT

        assert worst_status != WorkspaceStatus.HARD_LIMIT, \
            f"{label}: feasible trajectory hits workspace hard limit!"

        print(f"  {label}: feasible, worst workspace status = {worst_status.value}")

    print("  No feasible trajectory hits workspace hard limits")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 12: Workspace limits: boundary exactness
# ---------------------------------------------------------------------------

def test_workspace_boundary_exactness():
    """Verify exact boundary transitions between OK, soft, and hard."""
    _header("Test 12: Workspace limits: boundary exactness")
    geom = StewartGeometry()
    limits = WorkspaceLimits.from_geometry(geom)

    mid_ext = np.array([50.0] * 6)

    # Just inside soft limit (OK)
    ext_ok = mid_ext.copy()
    ext_ok[0] = limits.leg_soft_min_mm + 0.1
    r_ok = check_workspace_limits(ext_ok, limits.cond_home, limits)
    assert r_ok.status == WorkspaceStatus.OK

    # Just outside soft limit (SOFT)
    ext_soft = mid_ext.copy()
    ext_soft[0] = limits.leg_soft_min_mm - 0.1
    r_soft = check_workspace_limits(ext_soft, limits.cond_home, limits)
    assert r_soft.status == WorkspaceStatus.SOFT_LIMIT

    # Just inside hard limit (SOFT, not HARD)
    ext_near_hard = mid_ext.copy()
    ext_near_hard[0] = limits.leg_hard_min_mm + 0.1
    r_near_hard = check_workspace_limits(ext_near_hard, limits.cond_home, limits)
    assert r_near_hard.status == WorkspaceStatus.SOFT_LIMIT

    # Just outside hard limit (HARD)
    ext_hard = mid_ext.copy()
    ext_hard[0] = limits.leg_hard_min_mm - 0.1
    r_hard = check_workspace_limits(ext_hard, limits.cond_home, limits)
    assert r_hard.status == WorkspaceStatus.HARD_LIMIT

    # Condition number boundaries
    r_cond_ok = check_workspace_limits(
        mid_ext, limits.cond_soft - 1.0, limits)
    assert r_cond_ok.status == WorkspaceStatus.OK

    r_cond_soft = check_workspace_limits(
        mid_ext, limits.cond_soft + 1.0, limits)
    assert r_cond_soft.status == WorkspaceStatus.SOFT_LIMIT

    r_cond_hard = check_workspace_limits(
        mid_ext, limits.cond_hard + 1.0, limits)
    assert r_cond_hard.status == WorkspaceStatus.HARD_LIMIT

    print(f"  Leg boundaries:")
    print(f"    soft_min+0.1={limits.leg_soft_min_mm + 0.1:.1f} mm -> OK")
    print(f"    soft_min-0.1={limits.leg_soft_min_mm - 0.1:.1f} mm -> SOFT")
    print(f"    hard_min+0.1={limits.leg_hard_min_mm + 0.1:.1f} mm -> SOFT")
    print(f"    hard_min-0.1={limits.leg_hard_min_mm - 0.1:.1f} mm -> HARD")
    print(f"  Cond boundaries:")
    print(f"    soft-1={limits.cond_soft - 1:.1f} -> OK")
    print(f"    soft+1={limits.cond_soft + 1:.1f} -> SOFT")
    print(f"    hard+1={limits.cond_hard + 1:.1f} -> HARD")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("Phase 6 Hardening & Operational Readiness Verification Tests")
    print("=" * 70)

    t0 = time.time()
    passed = 0
    failed = 0

    tests = [
        test_workspace_limits_construction,
        test_workspace_check_ok,
        test_workspace_check_soft_leg,
        test_workspace_check_hard_leg,
        test_workspace_check_soft_cond,
        test_workspace_check_hard_cond,
        test_workspace_check_combined_soft,
        test_trajectory_progress,
        test_trajectory_progress_cancel,
        test_telemetry_phase6_fields,
        test_feasibility_workspace_consistency,
        test_workspace_boundary_exactness,
    ]

    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"  [FAIL] {e}")
            import traceback
            traceback.print_exc()
            failed += 1

    elapsed = time.time() - t0
    print(f"\n{'='*70}")
    print(f"Results: {passed} passed, {failed} failed ({elapsed:.2f}s)")
    print(f"{'='*70}")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
