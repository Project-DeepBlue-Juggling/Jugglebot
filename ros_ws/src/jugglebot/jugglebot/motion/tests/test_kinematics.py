"""Phase 1 verification tests for the kinematic model.

Tests:
  1. Regression vs sp_ik.py — position IK matches the legacy implementation
  2. Numerical Jacobian check — analytical J matches finite-difference J
  3. Round-trip test — twist → leg velocities → integrate → verify pose
  4. Bias term test — Jdot*twist matches numerically differentiated J*twist
  5. FK round-trip — IK → FK → IK gives consistent results
  6. Singularity map — workspace sweep (informational, no pass/fail)

Run:  python -m jugglebot.motion.tests.test_kinematics
"""

from __future__ import annotations

import sys
import time

import numpy as np
from numpy.linalg import norm

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    accel_to_leg_accels,
    compute_jacobian,
    compute_jacobian_dot,
    compute_leg_vectors,
    leg_lengths_to_pose,
    pose_to_leg_lengths,
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
    twist_to_leg_velocities,
)
from jugglebot.motion.workspace import (
    check_leg_extensions,
    check_reachability,
    map_singularities,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _random_reachable_pose(geom: StewartGeometry,
                           rng: np.random.Generator,
                           max_xy: float = 80.0,
                           z_range: tuple[float, float] = (30.0, 170.0),
                           max_tilt_deg: float = 10.0
                           ) -> tuple[np.ndarray, np.ndarray]:
    """Generate a random pose that is likely reachable."""
    for _ in range(100):
        pos = np.array([
            rng.uniform(-max_xy, max_xy),
            rng.uniform(-max_xy, max_xy),
            rng.uniform(*z_range),
        ])
        tilt_rad = np.deg2rad(max_tilt_deg)
        rotvec = np.array([
            rng.uniform(-tilt_rad, tilt_rad),
            rng.uniform(-tilt_rad, tilt_rad),
            rng.uniform(-tilt_rad * 0.3, tilt_rad * 0.3),  # less yaw
        ])
        rot = rotvec_to_rot_matrix(rotvec)
        if check_reachability(pos, rot, geom):
            return pos, rot
    raise RuntimeError("Could not generate a reachable pose")


def _random_small_twist(rng: np.random.Generator,
                        v_scale: float = 50.0,
                        w_scale: float = 0.3) -> np.ndarray:
    """Random small twist [vx, vy, vz, ωx, ωy, ωz]."""
    twist = np.empty(6)
    twist[:3] = rng.uniform(-v_scale, v_scale, size=3)
    twist[3:] = rng.uniform(-w_scale, w_scale, size=3)
    return twist


# ---------------------------------------------------------------------------
# Test 1: Regression vs sp_ik.py
# ---------------------------------------------------------------------------

def test_regression_vs_sp_ik():
    """Verify new position IK matches the old sp_ik.py implementation."""
    import jugglebot.hardware_config as hw

    geom = StewartGeometry()

    # Replicate sp_ik.py logic verbatim
    start_pos = np.array([0.0, 0.0, hw.GEOM_INITIAL_HEIGHT_MM])
    base_nodes = np.array(hw.GEOM_BASE_NODES_MM)
    init_plat_nodes = np.array(hw.GEOM_INIT_PLAT_NODES_MM)
    init_leg_lengths = np.array(hw.GEOM_INIT_LEG_LENGTHS_MM)

    test_poses = [
        # (pos_offset, quaternion_wxyz)
        (np.array([0.0, 0.0, 0.0]), (1.0, 0.0, 0.0, 0.0)),       # home
        (np.array([0.0, 0.0, 170.0]), (1.0, 0.0, 0.0, 0.0)),     # raised
        (np.array([50.0, -30.0, 100.0]), (1.0, 0.0, 0.0, 0.0)),  # translated
        (np.array([0.0, 0.0, 100.0]),                              # tilted
         (0.9961947, 0.0871557, 0.0, 0.0)),  # ~10° about X
        (np.array([30.0, 20.0, 150.0]),                            # combined
         (0.9914449, 0.0436194, 0.0871557, 0.0697565)),
    ]

    max_err = 0.0
    for pos_offset, qwxyz in test_poses:
        w, x, y, z = qwxyz
        rot = quat_to_rot_matrix(w, x, y, z)

        # --- New code ---
        new_ext = pose_to_leg_lengths(pos_offset, rot, geom)

        # --- Old sp_ik.py logic ---
        new_position = pos_offset.reshape(3, 1) + start_pos.reshape(3, 1)
        old_plat_world = (new_position + rot @ init_plat_nodes.T).T
        old_ext = norm(old_plat_world - base_nodes, axis=1) - init_leg_lengths

        err = np.max(np.abs(new_ext - old_ext))
        max_err = max(max_err, err)

    passed = max_err < 1e-10
    print(f"  [{'PASS' if passed else 'FAIL'}] regression vs sp_ik.py  "
          f"(max error: {max_err:.2e} mm)")
    return passed


# ---------------------------------------------------------------------------
# Test 2: Numerical Jacobian check
# ---------------------------------------------------------------------------

def test_numerical_jacobian():
    """Verify analytical Jacobian matches finite-difference Jacobian.

    The Jacobian maps a platform twist [v, omega] to leg extension rates.
    For numerical validation:
      - Translation columns (0-2): perturb pos by delta * e_i
      - Rotation columns (3-5): apply a small world-frame rotation
        exp(skew(delta * e_i)) @ R  (NOT rotvec + delta * e_i)
    """
    from jugglebot.motion.ik_solver import skew

    geom = StewartGeometry()
    rng = np.random.default_rng(42)

    delta = 1e-7
    max_err = 0.0
    n_poses = 30

    for _ in range(n_poses):
        pos, rot = _random_reachable_pose(geom, rng)

        J_analytical = compute_jacobian(pos, rot, geom)
        J_numerical = np.empty((6, 6))

        for j in range(6):
            if j < 3:
                # Translation perturbation: shift pos
                e = np.zeros(3)
                e[j] = 1.0

                pos_p = pos + delta * e
                ext_plus = pose_to_leg_lengths(pos_p, rot, geom)

                pos_m = pos - delta * e
                ext_minus = pose_to_leg_lengths(pos_m, rot, geom)
            else:
                # Rotation perturbation: apply small world-frame rotation
                e = np.zeros(3)
                e[j - 3] = 1.0

                dR_plus = rotvec_to_rot_matrix(delta * e)
                rot_p = dR_plus @ rot
                ext_plus = pose_to_leg_lengths(pos, rot_p, geom)

                dR_minus = rotvec_to_rot_matrix(-delta * e)
                rot_m = dR_minus @ rot
                ext_minus = pose_to_leg_lengths(pos, rot_m, geom)

            J_numerical[:, j] = (ext_plus - ext_minus) / (2 * delta)

        err = np.max(np.abs(J_analytical - J_numerical))
        max_err = max(max_err, err)

    passed = max_err < 1e-4
    print(f"  [{'PASS' if passed else 'FAIL'}] numerical Jacobian check  "
          f"(max error: {max_err:.2e}, {n_poses} poses)")
    return passed


# ---------------------------------------------------------------------------
# Test 3: Round-trip (twist → integrate → verify)
# ---------------------------------------------------------------------------

def test_round_trip():
    """Verify J by integrating twist and comparing leg length evolution."""
    geom = StewartGeometry()
    rng = np.random.default_rng(123)

    max_err = 0.0
    n_tests = 10

    for _ in range(n_tests):
        pos0, rot0 = _random_reachable_pose(geom, rng)
        twist = _random_small_twist(rng, v_scale=20.0, w_scale=0.1)

        # Compute leg velocities at initial pose
        q_dot = twist_to_leg_velocities(twist, pos0, rot0, geom)

        # Advance the pose by a very small dt
        dt = 1e-8
        from jugglebot.motion.ik_solver import _advance_pose
        pos1, rot1 = _advance_pose(pos0, rot0, twist, dt)

        # Leg lengths at both poses
        ext0 = pose_to_leg_lengths(pos0, rot0, geom)
        ext1 = pose_to_leg_lengths(pos1, rot1, geom)

        # Compare (ext1 - ext0) / dt with q_dot
        numerical_q_dot = (ext1 - ext0) / dt
        err = np.max(np.abs(q_dot - numerical_q_dot))
        max_err = max(max_err, err)

    passed = max_err < 1e-4  # mm/s tolerance given dt=1e-8
    print(f"  [{'PASS' if passed else 'FAIL'}] round-trip (twist integration)  "
          f"(max error: {max_err:.2e} mm/s, {n_tests} tests)")
    return passed


# ---------------------------------------------------------------------------
# Test 4: Bias term test
# ---------------------------------------------------------------------------

def test_bias_term():
    """Verify Jdot*twist matches numerically differentiated (J*twist)."""
    geom = StewartGeometry()
    rng = np.random.default_rng(456)

    max_err = 0.0
    n_tests = 15

    for _ in range(n_tests):
        pos, rot = _random_reachable_pose(geom, rng)
        twist = _random_small_twist(rng, v_scale=30.0, w_scale=0.2)

        # J̇ẋ from our function
        J_dot = compute_jacobian_dot(pos, rot, twist, geom, dt=1e-7)
        bias_analytical = J_dot @ twist

        # Numerical: d/dt(J·ẋ) ≈ (J(t+h)·ẋ − J(t)·ẋ) / h
        # where ẋ is held constant and pose evolves by twist
        h = 1e-8
        from jugglebot.motion.ik_solver import _advance_pose
        J0 = compute_jacobian(pos, rot, geom)
        pos1, rot1 = _advance_pose(pos, rot, twist, h)
        J1 = compute_jacobian(pos1, rot1, geom)

        # The bias term should be: d(q̇)/dt - J·ẍ  at constant ẋ (ẍ=0)
        # So: d(q̇)/dt = J̇·ẋ = (J(t+h)·ẋ - J(t)·ẋ) / h
        bias_numerical = (J1 @ twist - J0 @ twist) / h

        err = np.max(np.abs(bias_analytical - bias_numerical))
        max_err = max(max_err, err)

    passed = max_err < 1e-2  # mm/s^2 -- numerical differentiation introduces more error here
    print(f"  [{'PASS' if passed else 'FAIL'}] bias term (Jdot * twist)  "
          f"(max error: {max_err:.2e} mm/s^2, {n_tests} tests)")
    return passed


# ---------------------------------------------------------------------------
# Test 5: FK round-trip
# ---------------------------------------------------------------------------

def test_fk_round_trip():
    """Verify IK → FK → IK gives consistent results."""
    geom = StewartGeometry()
    rng = np.random.default_rng(789)

    max_pos_err = 0.0
    max_rot_err = 0.0
    n_tests = 20

    for _ in range(n_tests):
        pos_orig, rot_orig = _random_reachable_pose(geom, rng)

        # IK: pose → leg lengths
        extensions = pose_to_leg_lengths(pos_orig, rot_orig, geom)

        # FK: leg lengths → pose
        pos_recovered, rot_recovered = leg_lengths_to_pose(
            extensions, geom, initial_guess=(pos_orig, rot_orig))

        # Verify: pose matches
        pos_err = norm(pos_recovered - pos_orig)
        rot_err = norm(rot_matrix_to_rotvec(rot_recovered @ rot_orig.T))

        max_pos_err = max(max_pos_err, pos_err)
        max_rot_err = max(max_rot_err, rot_err)

    # Also test FK from home without a close initial guess
    ext_home = pose_to_leg_lengths(np.zeros(3), np.eye(3), geom)
    pos_home, rot_home = leg_lengths_to_pose(ext_home, geom)
    home_pos_err = norm(pos_home)
    home_rot_err = norm(rot_matrix_to_rotvec(rot_home))
    max_pos_err = max(max_pos_err, home_pos_err)
    max_rot_err = max(max_rot_err, home_rot_err)

    passed = max_pos_err < 1e-8 and max_rot_err < 1e-8
    print(f"  [{'PASS' if passed else 'FAIL'}] FK round-trip  "
          f"(max pos err: {max_pos_err:.2e} mm, "
          f"max rot err: {max_rot_err:.2e} rad, {n_tests} poses)")
    return passed


# ---------------------------------------------------------------------------
# Test 6: Singularity map (informational)
# ---------------------------------------------------------------------------

def test_singularity_map():
    """Sweep workspace and report condition number statistics.

    This test is informational — it always passes but prints a summary
    of the workspace singularity landscape.
    """
    geom = StewartGeometry()

    print("  [INFO] singularity map — running workspace sweep...")
    t0 = time.time()
    result = map_singularities(geom, resolution=6)
    elapsed = time.time() - t0

    s = result['summary']
    print(f"         {s['total_poses']} poses swept in {elapsed:.1f}s")
    print(f"         Reachable: {s['reachable']}, "
          f"Unreachable: {s['unreachable']}")
    if s['reachable'] > 0:
        print(f"         Condition number — "
              f"min: {s['cond_min']:.1f}, "
              f"mean: {s['cond_mean']:.1f}, "
              f"median: {s['cond_median']:.1f}, "
              f"max: {s['cond_max']:.1f}")
        print(f"         Ill-conditioned (cond > 100): "
              f"{s['ill_conditioned']}")
    return True  # always passes — informational only


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 60)
    print("Phase 1 Kinematic Verification Tests")
    print("=" * 60)
    print()

    tests = [
        ("1. Regression vs sp_ik.py", test_regression_vs_sp_ik),
        ("2. Numerical Jacobian", test_numerical_jacobian),
        ("3. Round-trip (twist integration)", test_round_trip),
        ("4. Bias term (Jdot * twist)", test_bias_term),
        ("5. FK round-trip", test_fk_round_trip),
        ("6. Singularity map", test_singularity_map),
    ]

    results = []
    for name, test_fn in tests:
        print(f"\n{name}")
        try:
            passed = test_fn()
        except Exception as e:
            print(f"  [FAIL] {e}")
            passed = False
        results.append(passed)

    print("\n" + "=" * 60)
    n_passed = sum(results)
    n_total = len(results)
    all_passed = all(results)

    if all_passed:
        print(f"ALL {n_total} TESTS PASSED")
    else:
        print(f"{n_passed}/{n_total} tests passed")
        for (name, _), passed in zip(tests, results):
            if not passed:
                print(f"  FAILED: {name}")

    print("=" * 60)
    return 0 if all_passed else 1


if __name__ == '__main__':
    sys.exit(main())
