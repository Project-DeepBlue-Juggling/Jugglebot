"""Phase 4 verification tests for the quintic trajectory generator.

Tests:
  1. Boundary conditions — quintic evaluates to exact BCs at t=0, t=T
  2. Rest-to-rest special case — coefficients match 10t^3-15t^4+6t^5
  3. Limit checking (rejects bad) — velocity, stroke, unreachable violations
  4. Limit checking (accepts good) — small move from home passes
  5. Speed scaling — duration 2x, peak vel 0.5x, peak accel 0.25x
  6. Feedforward torque preview — hardware trajectories within limits
  7. TrajectoryManager lifecycle — state machine transitions

Run:  python -m jugglebot.motion.tests.test_trajectory
"""

from __future__ import annotations

import sys
import time

import numpy as np
from numpy.linalg import norm

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.ik_solver import rotvec_to_rot_matrix
from jugglebot.motion.trajectory import (
    solve_quintic_1d,
    QuinticTrajectory,
    create_trajectory,
    evaluate,
    rescale_trajectory,
    cartesian_to_motor_commands,
    check_feasibility,
    FeasibilityResult,
    TrajectoryManager,
    TrajectoryState,
)


def _header(name: str):
    print(f"\n{'='*70}")
    print(f"  {name}")
    print(f"{'='*70}")


# ---------------------------------------------------------------------------
# Test 1: Boundary conditions
# ---------------------------------------------------------------------------

def test_boundary_conditions():
    """Verify quintic polynomial exactly satisfies boundary conditions."""
    _header("Test 1: Boundary conditions")

    test_cases = [
        # (label, x0, v0, a0, xf, vf, af, T)
        ("Rest-to-rest", 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 1.0),
        ("Moving start", 10.0, 5.0, 1.0, 50.0, 0.0, 0.0, 2.0),
        ("Moving end", 0.0, 0.0, 0.0, 80.0, -3.0, 2.0, 1.5),
        ("Both moving", -20.0, 10.0, -5.0, 30.0, -8.0, 3.0, 0.8),
        ("Short duration", 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.1),
    ]

    tol = 1e-10

    for label, x0, v0, a0, xf, vf, af, T in test_cases:
        c = solve_quintic_1d(x0, v0, a0, xf, vf, af, T)

        # Evaluate at tau=0
        p0 = c[0]
        dp0 = c[1] / T
        ddp0 = 2 * c[2] / (T * T)

        # Evaluate at tau=1
        p1 = np.sum(c)
        dp1 = (c[1] + 2*c[2] + 3*c[3] + 4*c[4] + 5*c[5]) / T
        ddp1 = (2*c[2] + 6*c[3] + 12*c[4] + 20*c[5]) / (T * T)

        assert abs(p0 - x0) < tol, f"{label}: p(0) = {p0}, expected {x0}"
        assert abs(dp0 - v0) < tol, f"{label}: p'(0) = {dp0}, expected {v0}"
        assert abs(ddp0 - a0) < tol, f"{label}: p''(0) = {ddp0}, expected {a0}"
        assert abs(p1 - xf) < tol, f"{label}: p(1) = {p1}, expected {xf}"
        assert abs(dp1 - vf) < tol, f"{label}: p'(1) = {dp1}, expected {vf}"
        assert abs(ddp1 - af) < tol, f"{label}: p''(1) = {ddp1}, expected {af}"

        print(f"  {label}: max BC error = "
              f"{max(abs(p0-x0), abs(dp0-v0), abs(ddp0-a0), abs(p1-xf), abs(dp1-vf), abs(ddp1-af)):.2e}")

    # Also test 6-DoF create_trajectory + evaluate
    print("  Testing 6-DoF create_trajectory + evaluate...")
    start_pose = np.array([10.0, -5.0, 20.0, 0.01, -0.02, 0.005])
    start_twist = np.array([5.0, -2.0, 10.0, 0.1, -0.05, 0.02])
    start_accel = np.array([1.0, -0.5, 3.0, 0.01, -0.01, 0.005])
    end_pose = np.array([50.0, 10.0, 80.0, 0.05, 0.03, -0.01])
    end_twist = np.array([-3.0, 1.0, -5.0, -0.02, 0.01, -0.01])
    end_accel = np.array([0.5, -1.0, 2.0, 0.005, -0.005, 0.002])
    T = 1.5

    traj = create_trajectory(start_pose, start_twist, start_accel,
                             end_pose, end_twist, end_accel,
                             duration=T, t_start=0.0)

    p0, v0, a0 = evaluate(traj, 0.0)
    pf, vf, af = evaluate(traj, T)

    assert norm(p0 - start_pose) < tol, f"6-DoF: pose(0) error = {norm(p0 - start_pose):.2e}"
    assert norm(v0 - start_twist) < tol, f"6-DoF: twist(0) error = {norm(v0 - start_twist):.2e}"
    assert norm(a0 - start_accel) < tol, f"6-DoF: accel(0) error = {norm(a0 - start_accel):.2e}"
    # After t_end, twist and accel are zero (hold at end)
    assert norm(pf - end_pose) < tol, f"6-DoF: pose(T) error = {norm(pf - end_pose):.2e}"

    # Evaluate just before T to check end BCs
    eps = 1e-12
    pf2, vf2, af2 = evaluate(traj, T - eps)
    assert norm(pf2 - end_pose) < 1e-6, f"6-DoF: pose(T-eps) error = {norm(pf2 - end_pose):.2e}"
    assert norm(vf2 - end_twist) < 1e-6, f"6-DoF: twist(T-eps) error = {norm(vf2 - end_twist):.2e}"
    assert norm(af2 - end_accel) < 1e-6, f"6-DoF: accel(T-eps) error = {norm(af2 - end_accel):.2e}"

    print(f"  6-DoF: max BC error at t=0: {max(norm(p0-start_pose), norm(v0-start_twist), norm(a0-start_accel)):.2e}")
    print(f"  6-DoF: max BC error at t=T-eps: {max(norm(pf2-end_pose), norm(vf2-end_twist), norm(af2-end_accel)):.2e}")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 2: Rest-to-rest special case
# ---------------------------------------------------------------------------

def test_rest_to_rest():
    """Verify rest-to-rest quintic matches the known closed form."""
    _header("Test 2: Rest-to-rest special case")

    # The rest-to-rest quintic s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
    # with unit displacement and unit duration
    c = solve_quintic_1d(x0=0.0, v0=0.0, a0=0.0, xf=1.0, vf=0.0, af=0.0, duration=1.0)

    assert abs(c[0]) < 1e-15, f"c0 should be 0, got {c[0]}"
    assert abs(c[1]) < 1e-15, f"c1 should be 0, got {c[1]}"
    assert abs(c[2]) < 1e-15, f"c2 should be 0, got {c[2]}"
    assert abs(c[3] - 10.0) < 1e-12, f"c3 should be 10, got {c[3]}"
    assert abs(c[4] - (-15.0)) < 1e-12, f"c4 should be -15, got {c[4]}"
    assert abs(c[5] - 6.0) < 1e-12, f"c5 should be 6, got {c[5]}"
    print(f"  Coefficients: {c}")
    print(f"  Expected:     [0, 0, 0, 10, -15, 6]")

    # Verify peak acceleration matches QUINTIC_S2_MAX = 5.7735027
    # s''(tau) = 60*tau - 180*tau^2 + 120*tau^3
    # For unit displacement and unit duration, max |s''| = max |p''(tau)/T^2|
    # = max |p''(tau)| since T=1
    # Peak is at tau where d/dtau[s''] = 0: 60 - 360*tau + 360*tau^2 = 0
    # tau = (360 +/- sqrt(360^2 - 4*360*60)) / (2*360)
    # = (360 +/- sqrt(129600 - 86400)) / 720
    # = (360 +/- sqrt(43200)) / 720
    # = (360 +/- 207.846) / 720
    # tau1 = 0.2113, tau2 = 0.7887
    taus = np.linspace(0, 1, 10000)
    s2 = 2*c[2] + 6*c[3]*taus + 12*c[4]*taus**2 + 20*c[5]*taus**3
    peak_s2 = np.max(np.abs(s2))

    QUINTIC_S2_MAX = 5.7735027
    rel_err = abs(peak_s2 - QUINTIC_S2_MAX) / QUINTIC_S2_MAX
    assert rel_err < 1e-4, f"Peak |s''| = {peak_s2:.7f}, expected {QUINTIC_S2_MAX}, error {rel_err:.2e}"
    print(f"  Peak |s''(tau)| = {peak_s2:.7f} (expected {QUINTIC_S2_MAX}, error {rel_err:.2e})")

    # Verify s(0.5) = 0.5 (symmetric rest-to-rest)
    tau_mid = 0.5
    s_mid = c[0] + c[1]*tau_mid + c[2]*tau_mid**2 + c[3]*tau_mid**3 + c[4]*tau_mid**4 + c[5]*tau_mid**5
    assert abs(s_mid - 0.5) < 1e-12, f"s(0.5) should be 0.5, got {s_mid}"
    print(f"  s(0.5) = {s_mid:.15f} (expected 0.5)")

    # Verify peak velocity: s'(0.5)/T = 1.875 for unit displacement, unit duration
    ds_mid = (c[1] + 2*c[2]*tau_mid + 3*c[3]*tau_mid**2 + 4*c[4]*tau_mid**3 + 5*c[5]*tau_mid**4)
    assert abs(ds_mid - 1.875) < 1e-12, f"s'(0.5) should be 1.875, got {ds_mid}"
    print(f"  s'(0.5) = {ds_mid:.15f} (expected 1.875)")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 3: Limit checking (rejects bad trajectories)
# ---------------------------------------------------------------------------

def test_limit_checking_rejects():
    """Verify feasibility checker rejects known-bad trajectories."""
    _header("Test 3: Limit checking (rejects bad)")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    zeros6 = np.zeros(6)

    # 3a: Velocity violation — 200mm in 0.05s (peak vel ~7500 mm/s ~ 106 rev/s >> 15 limit)
    print("  3a: Velocity violation (200mm in 0.05s)...")
    traj_fast = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=np.array([0, 0, 200, 0, 0, 0], dtype=float),
        end_twist=zeros6, end_accel=zeros6,
        duration=0.05)
    result = check_feasibility(traj_fast, geom, params)
    assert not result.feasible, "Fast trajectory should be rejected"
    has_vel_violation = any("Velocity" in v or "velocity" in v for v in result.violations)
    assert has_vel_violation, f"Should have velocity violation, got: {result.violations}"
    print(f"    Peak vel: {np.max(result.peak_leg_vel_rps):.1f} rev/s -- rejected")

    # 3b: Stroke violation — move to z = -50mm (known unreachable from Phase 3 C5)
    print("  3b: Stroke violation (z=-50mm, unreachable)...")
    traj_deep = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=np.array([0, 0, -50, 0, 0, 0], dtype=float),
        end_twist=zeros6, end_accel=zeros6,
        duration=2.0)
    result = check_feasibility(traj_deep, geom, params)
    assert not result.feasible, "Deep trajectory should be rejected"
    has_ext_violation = any("extended" in v.lower() for v in result.violations)
    assert has_ext_violation, f"Should have extension violation, got: {result.violations}"
    print(f"    Min extension: {np.min(result.min_extension_mm):.1f} mm -- rejected")

    # 3c: Acceleration violation — very short, large move
    print("  3c: Acceleration violation (100mm in 0.1s)...")
    traj_accel = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=np.array([0, 0, 100, 0, 0, 0], dtype=float),
        end_twist=zeros6, end_accel=zeros6,
        duration=0.1)
    result = check_feasibility(traj_accel, geom, params)
    assert not result.feasible, "High-accel trajectory should be rejected"
    print(f"    Peak accel: {np.max(result.peak_leg_accel_rps2):.1f} rev/s^2 -- rejected")

    print("  All bad trajectories correctly rejected")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 4: Limit checking (accepts good trajectories)
# ---------------------------------------------------------------------------

def test_limit_checking_accepts():
    """Verify feasibility checker accepts known-good trajectories."""
    _header("Test 4: Limit checking (accepts good)")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    zeros6 = np.zeros(6)

    test_cases = [
        ("10mm Z, 1.0s", np.array([0, 0, 10, 0, 0, 0], dtype=float), 1.0),
        ("50mm Z, 2.0s", np.array([0, 0, 50, 0, 0, 0], dtype=float), 2.0),
        ("20mm XY, 1.5s", np.array([20, -10, 0, 0, 0, 0], dtype=float), 1.5),
        ("30mm Z + 3deg tilt, 1.5s",
         np.array([0, 0, 30, np.deg2rad(3), 0, 0], dtype=float), 1.5),
    ]

    for label, end_pose, T in test_cases:
        traj = create_trajectory(
            start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
            end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
            duration=T)
        result = check_feasibility(traj, geom, params)
        assert result.feasible, f"{label}: should be feasible, violations: {result.violations}"
        print(f"  {label}: FEASIBLE")
        print(f"    Peak vel: {np.max(result.peak_leg_vel_rps):.3f} rev/s "
              f"(limit: 15.0)")
        print(f"    Peak accel: {np.max(result.peak_leg_accel_rps2):.3f} rev/s^2 "
              f"(limit: 30.0)")
        print(f"    Cond#: {result.peak_condition_number:.1f}")
        print(f"    Extensions: [{np.min(result.min_extension_mm):.1f}, "
              f"{np.max(result.max_extension_mm):.1f}] mm")

    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 5: Speed scaling
# ---------------------------------------------------------------------------

def test_speed_scaling():
    """Verify speed scaling preserves shape and scales velocities."""
    _header("Test 5: Speed scaling")

    zeros6 = np.zeros(6)
    end_pose = np.array([0, 0, 50, 0, 0, 0], dtype=float)
    T = 1.0

    # Full speed trajectory
    traj_full = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=T, speed_scale=1.0)

    # Half speed trajectory
    traj_half = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=T, speed_scale=0.5)

    # Quarter speed trajectory
    traj_quarter = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=T, speed_scale=0.25)

    # Check durations
    assert abs(traj_full.duration - 1.0) < 1e-12, f"Full: duration = {traj_full.duration}"
    assert abs(traj_half.duration - 2.0) < 1e-12, f"Half: duration = {traj_half.duration}"
    assert abs(traj_quarter.duration - 4.0) < 1e-12, f"Quarter: duration = {traj_quarter.duration}"
    print(f"  Durations: full={traj_full.duration}s, half={traj_half.duration}s, quarter={traj_quarter.duration}s")

    # Check spatial path: sample at 100 normalized time points
    n = 100
    full_poses = []
    half_poses = []
    for i in range(n + 1):
        tau = i / n
        p_full, _, _ = evaluate(traj_full, tau * traj_full.duration)
        p_half, _, _ = evaluate(traj_half, tau * traj_half.duration)
        full_poses.append(p_full)
        half_poses.append(p_half)

    full_poses = np.array(full_poses)
    half_poses = np.array(half_poses)
    path_err = norm(full_poses - half_poses, axis=1)
    max_path_err = np.max(path_err)
    assert max_path_err < 1e-10, f"Path shape differs: max error = {max_path_err:.2e}"
    print(f"  Path shape preserved: max error = {max_path_err:.2e}")

    # Check peak velocities
    def peak_vel(traj):
        """Sample peak velocity magnitude."""
        n_samp = 500
        max_v = 0.0
        for i in range(n_samp + 1):
            t = traj.t_start + (i / n_samp) * traj.duration
            _, twist, _ = evaluate(traj, t)
            max_v = max(max_v, norm(twist[:3]))
        return max_v

    def peak_accel(traj):
        """Sample peak acceleration magnitude."""
        n_samp = 500
        max_a = 0.0
        for i in range(n_samp + 1):
            t = traj.t_start + (i / n_samp) * traj.duration
            _, _, accel = evaluate(traj, t)
            max_a = max(max_a, norm(accel[:3]))
        return max_a

    v_full = peak_vel(traj_full)
    v_half = peak_vel(traj_half)
    v_quarter = peak_vel(traj_quarter)

    a_full = peak_accel(traj_full)
    a_half = peak_accel(traj_half)
    a_quarter = peak_accel(traj_quarter)

    # Half speed: vel should be ~0.5x, accel ~0.25x
    vel_ratio_half = v_half / v_full
    accel_ratio_half = a_half / a_full
    assert abs(vel_ratio_half - 0.5) < 0.01, f"Half: vel ratio = {vel_ratio_half:.4f}, expected 0.5"
    assert abs(accel_ratio_half - 0.25) < 0.01, f"Half: accel ratio = {accel_ratio_half:.4f}, expected 0.25"
    print(f"  scale=0.5: vel ratio = {vel_ratio_half:.4f} (expected 0.5), "
          f"accel ratio = {accel_ratio_half:.4f} (expected 0.25)")

    # Quarter speed: vel should be ~0.25x, accel ~0.0625x
    vel_ratio_quarter = v_quarter / v_full
    accel_ratio_quarter = a_quarter / a_full
    assert abs(vel_ratio_quarter - 0.25) < 0.01, f"Quarter: vel ratio = {vel_ratio_quarter:.4f}, expected 0.25"
    assert abs(accel_ratio_quarter - 0.0625) < 0.01, f"Quarter: accel ratio = {accel_ratio_quarter:.4f}, expected 0.0625"
    print(f"  scale=0.25: vel ratio = {vel_ratio_quarter:.4f} (expected 0.25), "
          f"accel ratio = {accel_ratio_quarter:.4f} (expected 0.0625)")

    # Test rescale_trajectory
    traj_rescaled = rescale_trajectory(traj_full, new_speed_scale=0.5)
    assert abs(traj_rescaled.duration - 2.0) < 1e-12, f"Rescaled: duration = {traj_rescaled.duration}"
    v_rescaled = peak_vel(traj_rescaled)
    assert abs(v_rescaled / v_full - 0.5) < 0.01, f"Rescaled: vel ratio = {v_rescaled/v_full:.4f}"
    print(f"  rescale_trajectory: duration={traj_rescaled.duration}s, vel ratio = {v_rescaled/v_full:.4f}")

    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 6: Feedforward torque preview
# ---------------------------------------------------------------------------

def test_feedforward_preview():
    """Verify hardware trajectories at 25% speed are within limits."""
    _header("Test 6: Feedforward torque preview (hardware pre-flight)")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    zeros6 = np.zeros(6)

    import jugglebot.hardware_config as hw
    vel_limit = float(hw.ODRIVE_TRAP_VEL_LIMIT_RPS)
    # Conservative torque limit: 20A current limit * 0.0637 Nm/A = 1.27 Nm
    torque_limit = 1.0  # Nm, conservative

    hardware_trajectories = [
        ("Small Z (10mm, 1.0s, 25%)",
         np.array([0, 0, 10, 0, 0, 0], dtype=float), 1.0),
        ("Medium Z (50mm, 1.5s, 25%)",
         np.array([0, 0, 50, 0, 0, 0], dtype=float), 1.5),
        ("Combined (20,-10,30mm + 3deg pitch, 1.5s, 25%)",
         np.array([20, -10, 30, np.deg2rad(3), 0, 0], dtype=float), 1.5),
        ("Large Z (80mm, 2.0s, 25%)",
         np.array([0, 0, 80, 0, 0, 0], dtype=float), 2.0),
    ]

    for label, end_pose, T in hardware_trajectories:
        traj = create_trajectory(
            start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
            end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
            duration=T, speed_scale=0.25)

        result = check_feasibility(traj, geom, params, torque_limit_Nm=torque_limit)
        assert result.feasible, (
            f"{label}: INFEASIBLE at 25% speed! violations: {result.violations}")

        # Verify peak vel is within 25% of limit
        assert np.all(result.peak_leg_vel_rps < 0.25 * vel_limit * 1.1), (
            f"{label}: peak vel {np.max(result.peak_leg_vel_rps):.3f} exceeds "
            f"25% of limit ({0.25*vel_limit:.1f})")

        print(f"  {label}: FEASIBLE")
        print(f"    Duration (scaled): {traj.duration:.2f}s")
        print(f"    Peak vel: {np.max(result.peak_leg_vel_rps):.4f} rev/s")
        print(f"    Peak accel: {np.max(result.peak_leg_accel_rps2):.4f} rev/s^2")
        print(f"    Peak torque_ff: {np.max(result.peak_torque_ff_Nm):.4f} Nm")
        print(f"    Extension range: [{np.min(result.min_extension_mm):.1f}, "
              f"{np.max(result.max_extension_mm):.1f}] mm")

    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 7: TrajectoryManager lifecycle
# ---------------------------------------------------------------------------

def test_trajectory_manager():
    """Verify TrajectoryManager state transitions and evaluate output."""
    _header("Test 7: TrajectoryManager lifecycle")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    zeros6 = np.zeros(6)

    mgr = TrajectoryManager(geom, params)

    # Initial state
    assert mgr.state == TrajectoryState.IDLE, f"Initial state: {mgr.state}"
    print("  Initial state: IDLE")

    # Set hold pose to home
    mgr.set_hold_pose(zeros6)
    pos, vel, torque = mgr.evaluate(0.0)
    assert norm(vel) < 1e-12, f"IDLE vel should be zero, got {norm(vel)}"
    print(f"  IDLE evaluate: pos_rev norm = {norm(pos):.6f}, vel norm = {norm(vel):.2e}")

    # Submit a trajectory: home -> 10mm Z, 0.5s
    end_pose = np.array([0, 0, 10, 0, 0, 0], dtype=float)
    t_now = 100.0  # arbitrary start time
    traj = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=0.5, t_start=t_now)

    mgr.submit(traj)
    assert mgr.state == TrajectoryState.EXECUTING, f"After submit: {mgr.state}"
    print("  After submit: EXECUTING")

    # Evaluate at midpoint (t = t_now + 0.25)
    pos_mid, vel_mid, torque_mid = mgr.evaluate(t_now + 0.25)
    assert norm(vel_mid) > 0, "Mid-trajectory vel should be nonzero"
    print(f"  Mid-trajectory (t=0.25): pos_rev[0:3] = {pos_mid[:3]}, "
          f"vel norm = {norm(vel_mid):.4f}")

    # Evaluate after completion (t = t_now + 0.6)
    pos_end, vel_end, torque_end = mgr.evaluate(t_now + 0.6)
    assert mgr.state == TrajectoryState.COMPLETE, f"After completion: {mgr.state}"
    assert norm(vel_end) < 1e-12, f"COMPLETE vel should be zero, got {norm(vel_end)}"
    print(f"  After completion: COMPLETE, vel norm = {norm(vel_end):.2e}")

    # Verify end position matches IK of end_pose
    from jugglebot.motion.ik_solver import pose_to_leg_lengths
    from jugglebot.motion.conversions import extensions_mm_to_revs
    expected_ext = pose_to_leg_lengths(end_pose[:3], rotvec_to_rot_matrix(end_pose[3:6]), geom)
    expected_rev = extensions_mm_to_revs(expected_ext, geom)
    pos_err = norm(pos_end - expected_rev)
    assert pos_err < 1e-10, f"End pos error: {pos_err:.2e}"
    print(f"  End position error: {pos_err:.2e} rev")

    # Submit a new trajectory from COMPLETE state
    end_pose2 = np.array([0, 0, 30, 0, 0, 0], dtype=float)
    traj2 = create_trajectory(
        start_pose=end_pose, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose2, end_twist=zeros6, end_accel=zeros6,
        duration=0.5, t_start=t_now + 1.0)
    mgr.submit(traj2)
    assert mgr.state == TrajectoryState.EXECUTING, f"Second submit: {mgr.state}"
    print("  Second submit from COMPLETE: EXECUTING")

    # Cancel
    mgr.cancel()
    assert mgr.state == TrajectoryState.IDLE, f"After cancel: {mgr.state}"
    print("  After cancel: IDLE")

    # Verify submit during EXECUTING raises
    traj3 = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=0.5, t_start=t_now + 2.0)
    mgr.submit(traj3)  # IDLE -> EXECUTING
    try:
        mgr.submit(traj3)  # Should raise
        assert False, "Should have raised RuntimeError"
    except RuntimeError:
        print("  Submit during EXECUTING correctly raises RuntimeError")

    mgr.cancel()
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("Phase 4 Trajectory Generator Verification Tests")
    print("=" * 70)

    t0 = time.time()
    passed = 0
    failed = 0

    tests = [
        test_boundary_conditions,
        test_rest_to_rest,
        test_limit_checking_rejects,
        test_limit_checking_accepts,
        test_speed_scaling,
        test_feedforward_preview,
        test_trajectory_manager,
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
