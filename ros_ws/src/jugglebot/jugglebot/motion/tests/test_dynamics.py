"""Phase 3 verification tests for the dynamics model.

Tests:
  1. Gravity wrench at home — verify force magnitude and CoM moment direction
  2. Leg force vertical sum — total vertical component equals platform weight
  3. Round-trip — leg forces via J^{-T} reproduce the input support wrench via J^T
  4. Centred CoM — zero moment when CoM is at geometric centre
  5. Tilted pose — gravity wrench rotates correctly with platform orientation
  6. Force conversion — motor torques match expected order of magnitude

Run:  python -m jugglebot.motion.tests.test_dynamics
"""

from __future__ import annotations

import sys
import time

import numpy as np
from numpy.linalg import norm

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import compute_jacobian, rotvec_to_rot_matrix
from jugglebot.motion.conversions import leg_forces_to_motor_torques
from jugglebot.motion.dynamics import (
    DynamicsParams,
    compute_gravity_wrench,
    compute_reflected_inertia,
    gravity_to_leg_forces,
    gravity_to_motor_torques,
)


def _header(name: str):
    print(f"\n{'='*70}")
    print(f"  {name}")
    print(f"{'='*70}")


def test_gravity_wrench_at_home():
    """Verify gravity wrench at home pose with known CoM offset."""
    _header("Test 1: Gravity wrench at home pose")
    params = DynamicsParams.from_config()
    rot = np.eye(3)

    W = compute_gravity_wrench(rot, params)

    # Force should be [0, 0, -mg]
    mg = params.mass_kg * params.gravity_mps2
    assert abs(W[0]) < 1e-12, f"Fx should be zero, got {W[0]}"
    assert abs(W[1]) < 1e-12, f"Fy should be zero, got {W[1]}"
    assert abs(W[2] - (-mg)) < 1e-10, f"Fz should be {-mg}, got {W[2]}"

    # Torque from CoM offset: tau = r_com x F_gravity
    # r_com = [-14.5, -67.0, 54.0] mm, F = [0, 0, -mg] N
    # tau_x = ry*Fz - rz*Fy = -67.0*(-mg) - 54.0*0 = 67.0*mg
    # tau_y = rz*Fx - rx*Fz = 54.0*0 - (-14.5)*(-mg) = -14.5*mg
    # tau_z = rx*Fy - ry*Fx = 0
    r = params.com_offset_mm
    expected_tau_x = r[1] * (-mg)  # -67.0 * (-mg) = 67.0*mg
    expected_tau_y = -r[0] * (-mg)  # -(-14.5)*(-mg) = -14.5*mg
    # Correct formula: tau = r x F
    # [ry*Fz - rz*Fy, rz*Fx - rx*Fz, rx*Fy - ry*Fx]
    expected_tau = np.cross(r, np.array([0.0, 0.0, -mg]))

    tau_err = norm(W[3:] - expected_tau)
    assert tau_err < 1e-10, f"Torque mismatch: error={tau_err}"

    print(f"  Platform mass: {params.mass_kg} kg")
    print(f"  CoM offset:    {params.com_offset_mm} mm")
    print(f"  Gravity force: [{W[0]:.4f}, {W[1]:.4f}, {W[2]:.4f}] N")
    print(f"  Gravity torque: [{W[3]:.2f}, {W[4]:.2f}, {W[5]:.2f}] N*mm")
    print(f"  Torque error:  {tau_err:.2e}")
    print("  [PASS]")


def test_leg_force_vertical_sum():
    """Verify that total vertical force component from all legs equals mg."""
    _header("Test 2: Leg force vertical sum equals platform weight")
    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    pos = np.zeros(3)
    rot = np.eye(3)

    f_legs = gravity_to_leg_forces(pos, rot, geom, params)

    # Verify via J^T: the wrench produced by these forces should be W_support
    J = compute_jacobian(pos, rot, geom)
    W_produced = J.T @ f_legs
    W_support = -compute_gravity_wrench(rot, params)

    wrench_err = norm(W_produced - W_support)
    mg = params.mass_kg * params.gravity_mps2

    # The vertical force component: sum of f_i * l_i_z
    # This is exactly W_produced[2], which should equal mg
    vertical_force = W_produced[2]
    vertical_err = abs(vertical_force - mg)

    print(f"  Per-leg forces (N): {np.array2string(f_legs, precision=4)}")
    print(f"  Total vertical force:  {vertical_force:.4f} N")
    print(f"  Expected (mg):         {mg:.4f} N")
    print(f"  Vertical error:        {vertical_err:.2e} N")
    print(f"  Full wrench error:     {wrench_err:.2e}")

    assert vertical_err < 1e-10, f"Vertical force mismatch: {vertical_err}"
    assert wrench_err < 1e-8, f"Wrench round-trip error: {wrench_err}"
    print("  [PASS]")


def test_round_trip():
    """Verify f_legs -> J^T -> W reproduces the input support wrench."""
    _header("Test 3: Round-trip f_legs -> J^T*f -> W_support")
    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    # Test at several poses
    test_poses = [
        (np.zeros(3), np.eye(3), "home"),
        (np.array([30.0, -20.0, 50.0]), np.eye(3), "translated"),
        (np.zeros(3), rotvec_to_rot_matrix(np.array([0.05, 0.03, 0.0])),
         "tilted 3deg"),
        (np.array([10.0, 15.0, 80.0]),
         rotvec_to_rot_matrix(np.array([0.08, -0.06, 0.02])),
         "translated+tilted"),
    ]

    max_err = 0.0
    for pos, rot, label in test_poses:
        f_legs = gravity_to_leg_forces(pos, rot, geom, params)
        J = compute_jacobian(pos, rot, geom)
        W_produced = J.T @ f_legs
        W_support = -compute_gravity_wrench(rot, params)

        err = norm(W_produced - W_support)
        max_err = max(max_err, err)
        status = "ok" if err < 1e-8 else "FAIL"
        print(f"  {label:25s} err={err:.2e}  [{status}]")

    assert max_err < 1e-8, f"Max round-trip error: {max_err}"
    print(f"  Max error across poses: {max_err:.2e}")
    print("  [PASS]")


def test_centred_com():
    """Verify zero gravity moment when CoM is at geometric centre."""
    _header("Test 4: Centred CoM produces zero moment")
    params_centred = DynamicsParams(
        mass_kg=0.96,
        com_offset_mm=np.array([0.0, 0.0, 0.0]),
        gravity_mps2=9.806,
    )
    rot = np.eye(3)

    W = compute_gravity_wrench(rot, params_centred)

    moment_mag = norm(W[3:])
    assert moment_mag < 1e-12, f"Moment should be zero, got {moment_mag}"

    # Also check that leg forces are roughly equal (symmetric loading)
    geom = StewartGeometry()
    pos = np.zeros(3)
    f_legs = gravity_to_leg_forces(pos, rot, geom, params_centred)

    f_std = np.std(f_legs)
    f_mean = np.mean(f_legs)
    cv = f_std / abs(f_mean) if abs(f_mean) > 1e-12 else 0.0

    print(f"  Moment magnitude: {moment_mag:.2e} N*mm")
    print(f"  Leg forces (N):   {np.array2string(f_legs, precision=4)}")
    print(f"  Force mean: {f_mean:.4f} N, std: {f_std:.4f} N, CV: {cv:.4f}")
    # Forces won't be perfectly equal due to non-symmetric geometry,
    # but they should be within a reasonable range
    assert cv < 0.5, f"Force variation too high: CV={cv:.4f}"
    print("  [PASS]")


def test_tilted_pose():
    """Verify gravity wrench rotates correctly with platform orientation."""
    _header("Test 5: Tilted pose — gravity wrench rotation")
    params = DynamicsParams.from_config()
    geom = StewartGeometry()

    # Tilt the platform 10 degrees about x-axis
    angle_rad = np.radians(10.0)
    rot = rotvec_to_rot_matrix(np.array([angle_rad, 0.0, 0.0]))

    W_home = compute_gravity_wrench(np.eye(3), params)
    W_tilted = compute_gravity_wrench(rot, params)

    # Gravity force is always [0, 0, -mg] regardless of platform orientation
    mg = params.mass_kg * params.gravity_mps2
    assert abs(W_tilted[2] - (-mg)) < 1e-10, "Fz should still be -mg"

    # But the moment changes because the CoM offset rotates with the platform
    r_com_tilted = rot @ params.com_offset_mm
    r_com_home = params.com_offset_mm
    assert not np.allclose(r_com_tilted, r_com_home), \
        "Rotated CoM should differ from home CoM"

    # Verify the torque matches r_com_tilted x F_gravity
    expected_tau = np.cross(r_com_tilted, np.array([0.0, 0.0, -mg]))
    tau_err = norm(W_tilted[3:] - expected_tau)
    assert tau_err < 1e-10, f"Tilted torque mismatch: {tau_err}"

    # Compute leg forces at tilted pose
    pos = np.zeros(3)
    f_legs = gravity_to_leg_forces(pos, rot, geom, params)
    f_legs_home = gravity_to_leg_forces(pos, np.eye(3), geom, params)

    print(f"  Tilt angle: {np.degrees(angle_rad):.1f} deg about x-axis")
    print(f"  CoM (home):   {np.array2string(r_com_home, precision=2)} mm")
    print(f"  CoM (tilted): {np.array2string(r_com_tilted, precision=2)} mm")
    print(f"  Moment (home):   [{W_home[3]:.2f}, {W_home[4]:.2f}, "
          f"{W_home[5]:.2f}] N*mm")
    print(f"  Moment (tilted): [{W_tilted[3]:.2f}, {W_tilted[4]:.2f}, "
          f"{W_tilted[5]:.2f}] N*mm")
    print(f"  Torque error:    {tau_err:.2e}")
    print(f"  Forces (home):   {np.array2string(f_legs_home, precision=4)} N")
    print(f"  Forces (tilted): {np.array2string(f_legs, precision=4)} N")
    print("  [PASS]")


def test_motor_torque_magnitude():
    """Verify motor torques are in the expected order of magnitude."""
    _header("Test 6: Motor torque magnitude check")
    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    pos = np.zeros(3)
    rot = np.eye(3)

    torques = gravity_to_motor_torques(pos, rot, geom, params)

    # Expected order of magnitude:
    # mg ~ 9.4 N, shared across 6 legs ~ 1.6 N each
    # spool_radius ~ 11 mm = 0.011 m
    # torque per motor ~ 1.6 * 0.011 ~ 0.018 Nm
    mg = params.mass_kg * params.gravity_mps2
    expected_mean = mg / 6.0 * np.mean(geom.spool_radius_mm) / 1000.0

    # Current per motor: I = tau / Kt, Kt ~ 0.0624 Nm/A
    Kt = 0.0624  # measured from Phase 2 bench test
    currents = torques / Kt

    print(f"  Motor torques (Nm): {np.array2string(torques, precision=5)}")
    print(f"  Expected mean:      {expected_mean:.5f} Nm")
    print(f"  Actual mean:        {np.mean(torques):.5f} Nm")
    print(f"  Estimated currents: {np.array2string(currents, precision=3)} A")
    print(f"  Max current:        {np.max(np.abs(currents)):.3f} A")

    # All torques should be positive (extension = push platform up)
    assert np.all(torques > 0), \
        f"All gravity FF torques should be positive, got {torques}"

    # All torques should be < 1 Nm (sanity — platform is only ~1 kg)
    assert np.all(np.abs(torques) < 1.0), \
        f"Torques unreasonably large: {torques}"

    # Mean should be within 3x of expected (geometry asymmetry + CoM offset)
    ratio = np.mean(torques) / expected_mean
    assert 0.3 < ratio < 3.0, \
        f"Mean torque ratio {ratio:.2f} outside expected range"

    print(f"  Mean/expected ratio: {ratio:.2f}")
    print("  [PASS]")


def test_reflected_inertia():
    """Verify reflected inertia computation (documented values)."""
    _header("Test 7: Reflected motor inertia (documentation)")
    geom = StewartGeometry()

    # Use a typical small BLDC motor rotor inertia: ~50e-6 kg*m^2
    # (placeholder — actual value TBD from motor datasheet)
    test_inertia = 50e-6  # kg*m^2

    reflected = compute_reflected_inertia(geom, test_inertia)

    spool_r_m = geom.spool_radius_mm / 1000.0
    expected = test_inertia / (spool_r_m ** 2)

    err = norm(reflected - expected)
    assert err < 1e-12, f"Reflected inertia mismatch: {err}"

    print(f"  Rotor inertia:     {test_inertia*1e6:.1f} x10^-6 kg*m^2")
    spool_str = np.array2string(geom.spool_radius_mm, precision=2)
    refl_str = np.array2string(reflected, precision=3)
    print(f"  Spool radii (mm):  {spool_str}")
    print(f"  Reflected (kg):    {refl_str}")
    print("  (These values represent the effective mass that motor inertia")
    print("   adds to each leg's dynamics. Used in Phase 5 feedforward.)")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("Phase 3 Dynamics Verification Tests")
    print("=" * 70)

    t0 = time.time()
    passed = 0
    failed = 0

    tests = [
        test_gravity_wrench_at_home,
        test_leg_force_vertical_sum,
        test_round_trip,
        test_centred_com,
        test_tilted_pose,
        test_motor_torque_magnitude,
        test_reflected_inertia,
    ]

    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"  [FAIL] {e}")
            failed += 1

    elapsed = time.time() - t0
    print(f"\n{'='*70}")
    print(f"Results: {passed} passed, {failed} failed ({elapsed:.2f}s)")
    print(f"{'='*70}")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
