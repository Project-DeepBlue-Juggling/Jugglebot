"""Phase 3 + Phase 5 verification tests for the dynamics model.

Phase 3 tests (1-7):
  1. Gravity wrench at active pose — verify force magnitude and CoM moment direction
  2. Leg force vertical sum — total vertical component equals platform weight
  3. Round-trip — leg forces via J^{-T} reproduce the input support wrench via J^T
  4. Centred CoM — zero moment when CoM is at geometric centre
  5. Tilted pose — gravity wrench rotates correctly with platform orientation
  6. Force conversion — motor torques match expected order of magnitude
  7. Reflected inertia — formula verification and magnitude documentation

Phase 5 tests (8-13):
  8. Inertia wrench at rest — zero wrench when twist and accel are both zero
  9. Pure translation accel — F = ma, zero torque for centred CoM
  10. Pure rotation accel — torque = I*alpha for small angles
  11. Gyroscopic term — omega x (I*omega) produces cross-axis torque
  12. Full feedforward at rest — equals gravity-only feedforward
  13. Full feedforward decomposition — gravity + inertia + reflected sum correctly

Removed in MPC-native refactor (Phase 4A):
  - Torque profile preview (used archived quintic/feasibility)

Run:  python -m jugglebot.motion.tests.test_dynamics
"""

from __future__ import annotations

import sys
import time

import numpy as np
from numpy.linalg import norm

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    compute_jacobian,
    rotvec_to_rot_matrix,
    accel_to_leg_accels,
)
from jugglebot.motion.conversions import leg_forces_to_motor_torques
from jugglebot.motion.dynamics import (
    DynamicsParams,
    compute_gravity_wrench,
    compute_inertia_wrench,
    compute_reflected_inertia,
    compute_full_feedforward_torques,
    gravity_to_leg_forces,
    gravity_to_motor_torques,
)


def _header(name: str):
    print(f"\n{'='*70}")
    print(f"  {name}")
    print(f"{'='*70}")


def test_gravity_wrench_at_active():
    """Verify gravity wrench at active pose with known CoM offset."""
    _header("Test 1: Gravity wrench at active pose")
    params = DynamicsParams.from_config()
    rot = np.eye(3)

    W = compute_gravity_wrench(rot, params)

    # Force should be [0, 0, -mg]
    mg = params.mass_kg * params.gravity_mps2
    assert abs(W[0]) < 1e-12, f"Fx should be zero, got {W[0]}"
    assert abs(W[1]) < 1e-12, f"Fy should be zero, got {W[1]}"
    assert abs(W[2] - (-mg)) < 1e-10, f"Fz should be {-mg}, got {W[2]}"

    # Torque from CoM offset: tau = r_com x F_gravity
    r = params.com_offset_mm
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

    test_poses = [
        (np.zeros(3), np.eye(3), "active"),
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

    geom = StewartGeometry()
    pos = np.zeros(3)
    f_legs = gravity_to_leg_forces(pos, rot, geom, params_centred)

    f_std = np.std(f_legs)
    f_mean = np.mean(f_legs)
    cv = f_std / abs(f_mean) if abs(f_mean) > 1e-12 else 0.0

    print(f"  Moment magnitude: {moment_mag:.2e} N*mm")
    print(f"  Leg forces (N):   {np.array2string(f_legs, precision=4)}")
    print(f"  Force mean: {f_mean:.4f} N, std: {f_std:.4f} N, CV: {cv:.4f}")
    assert cv < 0.5, f"Force variation too high: CV={cv:.4f}"
    print("  [PASS]")


def test_tilted_pose():
    """Verify gravity wrench rotates correctly with platform orientation."""
    _header("Test 5: Tilted pose — gravity wrench rotation")
    params = DynamicsParams.from_config()
    geom = StewartGeometry()

    angle_rad = np.radians(10.0)
    rot = rotvec_to_rot_matrix(np.array([angle_rad, 0.0, 0.0]))

    W_active = compute_gravity_wrench(np.eye(3), params)
    W_tilted = compute_gravity_wrench(rot, params)

    mg = params.mass_kg * params.gravity_mps2
    assert abs(W_tilted[2] - (-mg)) < 1e-10, "Fz should still be -mg"

    r_com_tilted = rot @ params.com_offset_mm
    r_com_active = params.com_offset_mm
    assert not np.allclose(r_com_tilted, r_com_active), \
        "Rotated CoM should differ from active CoM"

    expected_tau = np.cross(r_com_tilted, np.array([0.0, 0.0, -mg]))
    tau_err = norm(W_tilted[3:] - expected_tau)
    assert tau_err < 1e-10, f"Tilted torque mismatch: {tau_err}"

    pos = np.zeros(3)
    f_legs = gravity_to_leg_forces(pos, rot, geom, params)
    f_legs_active = gravity_to_leg_forces(pos, np.eye(3), geom, params)

    print(f"  Tilt angle: {np.degrees(angle_rad):.1f} deg about x-axis")
    print(f"  CoM (active):  {np.array2string(r_com_active, precision=2)} mm")
    print(f"  CoM (tilted):  {np.array2string(r_com_tilted, precision=2)} mm")
    print(f"  Moment (active):  [{W_active[3]:.2f}, {W_active[4]:.2f}, "
          f"{W_active[5]:.2f}] N*mm")
    print(f"  Moment (tilted):  [{W_tilted[3]:.2f}, {W_tilted[4]:.2f}, "
          f"{W_tilted[5]:.2f}] N*mm")
    print(f"  Torque error:     {tau_err:.2e}")
    print(f"  Forces (active):  {np.array2string(f_legs_active, precision=4)} N")
    print(f"  Forces (tilted):  {np.array2string(f_legs, precision=4)} N")
    print("  [PASS]")


def test_motor_torque_magnitude():
    """Verify motor torques are in the expected order of magnitude."""
    _header("Test 6: Motor torque magnitude check")
    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    pos = np.zeros(3)
    rot = np.eye(3)

    torques = gravity_to_motor_torques(pos, rot, geom, params)

    mg = params.mass_kg * params.gravity_mps2
    expected_mean = mg / 6.0 * np.mean(geom.spool_radius_mm) / 1000.0

    Kt = 0.0570  # measured 2026-07-15 (friction-cancelling traverse, pooled;
                 # canonical source: hardware_config.yaml motor_kt_nm_per_a —
                 # pinned by tests/sim/test_motor_kt_canonical_source.py)
    currents = torques / Kt

    print(f"  Motor torques (Nm): {np.array2string(torques, precision=5)}")
    print(f"  Expected mean:      {expected_mean:.5f} Nm")
    print(f"  Actual mean:        {np.mean(torques):.5f} Nm")
    print(f"  Estimated currents: {np.array2string(currents, precision=3)} A")
    print(f"  Max current:        {np.max(np.abs(currents)):.3f} A")

    assert np.all(torques > 0), \
        f"All gravity FF torques should be positive, got {torques}"
    assert np.all(np.abs(torques) < 1.0), \
        f"Torques unreasonably large: {torques}"

    ratio = np.mean(torques) / expected_mean
    assert 0.3 < ratio < 3.0, \
        f"Mean torque ratio {ratio:.2f} outside expected range"

    print(f"  Mean/expected ratio: {ratio:.2f}")
    print("  [PASS]")


def test_reflected_inertia():
    """Verify reflected inertia computation (documented values)."""
    _header("Test 7: Reflected motor inertia (documentation)")
    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    J_rotor = params.motor_rotor_inertia_kgm2

    reflected = compute_reflected_inertia(geom, J_rotor)

    spool_r_m = geom.spool_radius_mm / 1000.0
    expected = J_rotor / (spool_r_m ** 2)

    err = norm(reflected - expected)
    assert err < 1e-12, f"Reflected inertia mismatch: {err}"

    platform_mass_per_leg = params.mass_kg / 6.0

    print(f"  Rotor inertia:     {J_rotor*1e6:.1f} x10^-6 kg*m^2")
    spool_str = np.array2string(geom.spool_radius_mm, precision=2)
    refl_str = np.array2string(reflected, precision=3)
    print(f"  Spool radii (mm):  {spool_str}")
    print(f"  Reflected (kg):    {refl_str}")
    print(f"  Platform mass per leg: {platform_mass_per_leg:.3f} kg")
    print(f"  Reflected/platform ratio: {np.mean(reflected)/platform_mass_per_leg:.1f}x")
    print("  (Reflected motor inertia dominates platform mass per leg —")
    print("   Phase 5 feedforward is critical for high-speed tracking.)")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Phase 5 tests
# ---------------------------------------------------------------------------

def test_inertia_wrench_at_rest():
    """Verify inertia wrench is zero when twist and accel are both zero."""
    _header("Test 8: Inertia wrench at rest")
    params = DynamicsParams.from_config()
    rot = np.eye(3)
    twist = np.zeros(6)
    accel = np.zeros(6)

    W = compute_inertia_wrench(rot, twist, accel, params)

    mag = norm(W)
    assert mag < 1e-15, f"Inertia wrench should be zero at rest, got {mag}"

    # Also check at a tilted pose
    rot_tilted = rotvec_to_rot_matrix(np.array([0.1, -0.05, 0.02]))
    W_tilted = compute_inertia_wrench(rot_tilted, twist, accel, params)
    mag_tilted = norm(W_tilted)
    assert mag_tilted < 1e-15, \
        f"Inertia wrench should be zero at rest (tilted), got {mag_tilted}"

    print(f"  Wrench magnitude (active): {mag:.2e}")
    print(f"  Wrench magnitude (tilted): {mag_tilted:.2e}")
    print("  [PASS]")


def test_pure_translation_accel():
    """Verify F = m*a for pure translational acceleration, zero CoM offset."""
    _header("Test 9: Pure translational acceleration (centred CoM)")
    # Use centred CoM to decouple translation from rotation
    params = DynamicsParams(
        mass_kg=0.96,
        com_offset_mm=np.array([0.0, 0.0, 0.0]),
        gravity_mps2=9.806,
        inertia_tensor_kgmm2=np.diag([10000.0, 10000.0, 5000.0]),
    )
    rot = np.eye(3)
    twist = np.zeros(6)

    # Apply 1000 mm/s² acceleration in Z (= 1 m/s²)
    accel = np.array([0.0, 0.0, 1000.0, 0.0, 0.0, 0.0])
    W = compute_inertia_wrench(rot, twist, accel, params)

    # Expected: F = m * a / 1000 = 0.96 * 1000 / 1000 = 0.96 N in Z
    expected_Fz = params.mass_kg * 1000.0 / 1000.0  # = 0.96 N
    assert abs(W[0]) < 1e-12, f"Fx should be zero, got {W[0]}"
    assert abs(W[1]) < 1e-12, f"Fy should be zero, got {W[1]}"
    assert abs(W[2] - expected_Fz) < 1e-10, \
        f"Fz should be {expected_Fz}, got {W[2]}"

    # Torque should be zero for centred CoM with no angular accel
    tau_mag = norm(W[3:])
    assert tau_mag < 1e-10, f"Torque should be zero, got {tau_mag}"

    print(f"  Applied accel: [0, 0, 1000] mm/s^2 = [0, 0, 1] m/s^2")
    print(f"  Expected force: [0, 0, {expected_Fz:.4f}] N")
    print(f"  Computed force: [{W[0]:.6f}, {W[1]:.6f}, {W[2]:.6f}] N")
    print(f"  Torque magnitude: {tau_mag:.2e} N*mm")
    print("  [PASS]")


def test_pure_rotation_accel():
    """Verify tau = I*alpha for pure angular acceleration."""
    _header("Test 10: Pure angular acceleration")
    Ixx = 12000.0  # kg*mm^2
    Iyy = 11000.0
    Izz = 7000.0
    params = DynamicsParams(
        mass_kg=0.96,
        com_offset_mm=np.array([0.0, 0.0, 0.0]),
        gravity_mps2=9.806,
        inertia_tensor_kgmm2=np.diag([Ixx, Iyy, Izz]),
    )
    rot = np.eye(3)
    twist = np.zeros(6)

    # Apply 1 rad/s^2 about X-axis
    alpha_x = 1.0  # rad/s^2
    accel = np.array([0.0, 0.0, 0.0, alpha_x, 0.0, 0.0])
    W = compute_inertia_wrench(rot, twist, accel, params)

    # Expected: tau_x = Ixx * alpha_x / 1000 = 12000 * 1 / 1000 = 12 N*mm
    expected_tau_x = Ixx * alpha_x / 1000.0
    assert abs(W[3] - expected_tau_x) < 1e-8, \
        f"tau_x should be {expected_tau_x}, got {W[3]}"
    assert abs(W[4]) < 1e-10, f"tau_y should be zero, got {W[4]}"
    assert abs(W[5]) < 1e-10, f"tau_z should be zero, got {W[5]}"

    # Force should be zero (no CoM offset, no translational accel)
    assert norm(W[:3]) < 1e-12, f"Force should be zero, got {norm(W[:3])}"

    # Also test Y and Z axes
    for axis, I_val, label in [(1, Iyy, 'Y'), (2, Izz, 'Z')]:
        accel_test = np.zeros(6)
        accel_test[3 + axis] = 1.0
        W_test = compute_inertia_wrench(rot, twist, accel_test, params)
        expected = I_val / 1000.0
        err = abs(W_test[3 + axis] - expected)
        assert err < 1e-8, f"tau_{label} error: {err}"

    print(f"  Inertia tensor diag: [{Ixx}, {Iyy}, {Izz}] kg*mm^2")
    print(f"  alpha_x = 1 rad/s^2: tau_x = {W[3]:.4f} N*mm "
          f"(expected {expected_tau_x:.4f})")
    print(f"  All three axes verified")
    print("  [PASS]")


def test_gyroscopic_term():
    """Verify omega x (I*omega) produces cross-axis torque."""
    _header("Test 11: Gyroscopic cross-coupling term")
    Ixx = 12000.0  # kg*mm^2
    Iyy = 8000.0
    Izz = 7000.0
    params = DynamicsParams(
        mass_kg=0.96,
        com_offset_mm=np.array([0.0, 0.0, 0.0]),
        gravity_mps2=9.806,
        inertia_tensor_kgmm2=np.diag([Ixx, Iyy, Izz]),
    )
    rot = np.eye(3)

    # Spin about X and Y with angular velocity, zero angular acceleration
    omega_x = 2.0  # rad/s
    omega_y = 1.0  # rad/s
    twist = np.array([0.0, 0.0, 0.0, omega_x, omega_y, 0.0])
    accel = np.zeros(6)

    W = compute_inertia_wrench(rot, twist, accel, params)

    # omega x (I*omega) with omega = [wx, wy, 0], I diagonal:
    # I*omega = [Ixx*wx, Iyy*wy, 0]
    # omega x (I*omega) = [wy*0 - 0*Iyy*wy, 0*Ixx*wx - wx*0, wx*Iyy*wy - wy*Ixx*wx]
    #                   = [0, 0, (Iyy - Ixx)*wx*wy]
    I_omega = np.array([Ixx * omega_x, Iyy * omega_y, 0.0])
    omega_vec = np.array([omega_x, omega_y, 0.0])
    expected_gyro = np.cross(omega_vec, I_omega) / 1000.0  # N*mm

    tau_err = norm(W[3:] - expected_gyro)
    assert tau_err < 1e-8, f"Gyroscopic torque error: {tau_err}"

    expected_tz = (Iyy - Ixx) * omega_x * omega_y / 1000.0
    assert abs(W[5] - expected_tz) < 1e-8, \
        f"tau_z should be {expected_tz:.4f}, got {W[5]:.4f}"

    print(f"  omega = [{omega_x}, {omega_y}, 0] rad/s")
    print(f"  Gyroscopic torque: [{W[3]:.4f}, {W[4]:.4f}, {W[5]:.4f}] N*mm")
    print(f"  Expected:          [{expected_gyro[0]:.4f}, {expected_gyro[1]:.4f}, "
          f"{expected_gyro[2]:.4f}] N*mm")
    print(f"  tau_z = (Iyy-Ixx)*wx*wy/1000 = {expected_tz:.4f} N*mm")
    print(f"  Error: {tau_err:.2e}")
    print("  [PASS]")


def test_full_feedforward_at_rest():
    """Verify full feedforward equals gravity-only at rest."""
    _header("Test 12: Full feedforward at rest equals gravity-only")
    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    test_poses = [
        (np.zeros(3), np.eye(3), "active"),
        (np.array([20.0, -10.0, 30.0]),
         rotvec_to_rot_matrix(np.array([0.05, -0.03, 0.01])),
         "tilted"),
    ]

    max_err = 0.0
    for pos, rot, label in test_poses:
        grav_only = gravity_to_motor_torques(pos, rot, geom, params)
        full_ff = compute_full_feedforward_torques(
            pos, rot, np.zeros(6), np.zeros(6), geom, params)

        err = norm(full_ff - grav_only)
        max_err = max(max_err, err)

        print(f"  {label:15s}: gravity = {np.array2string(grav_only, precision=5)}")
        print(f"  {' '*15}  full_ff = {np.array2string(full_ff, precision=5)}")
        print(f"  {' '*15}  error   = {err:.2e}")

    assert max_err < 1e-10, f"Full FF should equal gravity at rest, error: {max_err}"
    print("  [PASS]")


def test_full_feedforward_decomposition():
    """Verify full feedforward = gravity_torques + inertia_torques + reflected_torques."""
    _header("Test 13: Full feedforward decomposition")
    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    pos = np.array([10.0, -5.0, 40.0])
    rot = rotvec_to_rot_matrix(np.array([0.05, -0.03, 0.01]))
    twist = np.array([50.0, -30.0, 100.0, 0.5, -0.3, 0.1])
    accel = np.array([500.0, -200.0, 1000.0, 2.0, -1.5, 0.5])

    # Full feedforward
    full_ff = compute_full_feedforward_torques(
        pos, rot, twist, accel, geom, params)

    # Manual decomposition:
    J = compute_jacobian(pos, rot, geom)
    W_gravity = compute_gravity_wrench(rot, params)
    W_support = -W_gravity
    W_inertia = compute_inertia_wrench(rot, twist, accel, params)

    W_total = W_support + W_inertia
    f_legs = np.linalg.solve(J.T, W_total)
    torque_platform = leg_forces_to_motor_torques(f_legs, geom)

    # Reflected motor inertia
    q_ddot = accel_to_leg_accels(accel, twist, pos, rot, geom)
    q_ddot_rps2 = q_ddot * geom.mm_to_rev
    q_ddot_rad_s2 = q_ddot_rps2 * (2.0 * np.pi)
    torque_reflected = params.motor_rotor_inertia_kgm2 * q_ddot_rad_s2

    manual_total = torque_platform + torque_reflected

    err = norm(full_ff - manual_total)
    assert err < 1e-10, f"Decomposition mismatch: {err}"

    # Report the magnitudes of each component
    grav_torque = gravity_to_motor_torques(pos, rot, geom, params)

    print(f"  Gravity FF:    {np.array2string(grav_torque, precision=5)} Nm")
    print(f"  Inertia terms: {np.array2string(torque_platform - grav_torque, precision=5)} Nm")
    print(f"  Reflected:     {np.array2string(torque_reflected, precision=5)} Nm")
    print(f"  Total:         {np.array2string(full_ff, precision=5)} Nm")
    print(f"  Decomposition error: {err:.2e}")

    grav_mag = norm(grav_torque)
    inertia_mag = norm(torque_platform - grav_torque)
    refl_mag = norm(torque_reflected)
    total_mag = norm(full_ff)
    print(f"\n  Component magnitudes (norm):")
    print(f"    Gravity:   {grav_mag:.5f} Nm ({grav_mag/total_mag*100:.1f}%)")
    print(f"    Inertia:   {inertia_mag:.5f} Nm ({inertia_mag/total_mag*100:.1f}%)")
    print(f"    Reflected: {refl_mag:.5f} Nm ({refl_mag/total_mag*100:.1f}%)")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("Phase 3 + Phase 5 Dynamics Verification Tests")
    print("=" * 70)

    t0 = time.time()
    passed = 0
    failed = 0

    tests = [
        # Phase 3 tests (1-7)
        test_gravity_wrench_at_active,
        test_leg_force_vertical_sum,
        test_round_trip,
        test_centred_com,
        test_tilted_pose,
        test_motor_torque_magnitude,
        test_reflected_inertia,
        # Phase 5 tests (8-14)
        test_inertia_wrench_at_rest,
        test_pure_translation_accel,
        test_pure_rotation_accel,
        test_gyroscopic_term,
        test_full_feedforward_at_rest,
        test_full_feedforward_decomposition,
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
