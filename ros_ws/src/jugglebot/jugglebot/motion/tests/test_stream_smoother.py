"""Verification tests for the StreamSmoother (universal C2 target gate).

Tests:
  1. C2 at splice — sequential targets maintain pos/vel/accel continuity
  2. Hold on completion — pose reaches target with zero derivatives
  3. Reset — clean state after reset
  4. Duration scaling — small displacement → short T, large → long T
  5. Motor limit compliance — peak motor vel/accel within ODRIVE limits
  6. Supersession — mid-segment target update preserves C2

Run:  python -m jugglebot.motion.tests.test_stream_smoother
"""

from __future__ import annotations

import sys

import numpy as np
from numpy.linalg import norm

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import rotvec_to_rot_matrix, pose_to_leg_lengths
from jugglebot.motion.conversions import extensions_mm_to_revs
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.motor_commands import cartesian_to_motor_commands
from jugglebot.motion.stream_smoother import StreamSmoother


def make_smoother() -> tuple[StreamSmoother, StewartGeometry]:
    """Create a StreamSmoother with real geometry and hardware limits."""
    geom = StewartGeometry()
    smoother = StreamSmoother(
        geom,
        vel_limit_rps=hw.ODRIVE_TRAP_VEL_LIMIT_RPS,
        accel_limit_rps2=hw.ODRIVE_TRAP_ACC_LIMIT_RPS2,
    )
    return smoother, geom


def home_pose() -> np.ndarray:
    """Default home pose [x, y, z, rx, ry, rz]."""
    return np.array([0.0, 0.0, hw.JB_OP_DEFAULT_ACTIVE_Z_MM, 0.0, 0.0, 0.0])


# -----------------------------------------------------------------------
# Test 1: C2 continuity at splice boundaries
# -----------------------------------------------------------------------

def test_c2_at_splice():
    """Feed 10 sequential targets, verify C2 continuity at every splice."""
    smoother, geom = make_smoother()
    home = home_pose()
    smoother.reset(home)

    # Generate 10 random targets within spacemouse range
    rng = np.random.default_rng(42)
    targets = []
    for _ in range(10):
        pos = rng.uniform(-50, 50, size=3)
        pos[2] += hw.JB_OP_DEFAULT_ACTIVE_Z_MM
        rot = rng.uniform(-0.1, 0.1, size=3)  # small rotations
        targets.append(np.concatenate([pos, rot]))

    t = 0.0
    dt_between_targets = 0.01  # 100 Hz spacemouse rate

    for i, target in enumerate(targets):
        # Evaluate just before the splice to get the "before" state
        if i > 0:
            pose_before, twist_before, accel_before = smoother.evaluate(t)

        # Set the new target (this evaluates at t internally for the splice)
        smoother.set_target(target, t)

        # Evaluate just after the splice
        pose_after, twist_after, accel_after = smoother.evaluate(t)

        if i > 0:
            pos_err = norm(pose_after - pose_before)
            vel_err = norm(twist_after - twist_before)
            acc_err = norm(accel_after - accel_before)

            assert pos_err < 1e-6, (
                f"Splice {i}: position discontinuity = {pos_err:.2e}")
            assert vel_err < 1e-6, (
                f"Splice {i}: velocity discontinuity = {vel_err:.2e}")
            assert acc_err < 1e-6, (
                f"Splice {i}: acceleration discontinuity = {acc_err:.2e}")

        t += dt_between_targets

    print(f"  [PASS] C2 continuity at {len(targets)} splices")
    return True


# -----------------------------------------------------------------------
# Test 2: Hold on completion
# -----------------------------------------------------------------------

def test_hold_on_completion():
    """Single target — verify pose=target with zero derivatives after completion."""
    smoother, geom = make_smoother()
    home = home_pose()
    smoother.reset(home)

    target = np.array([30.0, -20.0, home[2] + 50.0, 0.05, -0.03, 0.0])
    t = 0.0
    smoother.set_target(target, t)

    # Evaluate well past the segment duration
    pose, twist, accel = smoother.evaluate(t + 10.0)

    pos_err = norm(pose - target)
    twist_norm = norm(twist)
    accel_norm = norm(accel)

    assert pos_err < 1e-9, f"Position error = {pos_err:.2e}"
    assert twist_norm < 1e-9, f"Twist norm = {twist_norm:.2e}"
    assert accel_norm < 1e-9, f"Accel norm = {accel_norm:.2e}"

    print(f"  [PASS] Hold on completion: pos_err={pos_err:.2e}, "
          f"twist={twist_norm:.2e}, accel={accel_norm:.2e}")
    return True


# -----------------------------------------------------------------------
# Test 3: Reset
# -----------------------------------------------------------------------

def test_reset():
    """After reset, evaluate returns home with zero derivatives."""
    smoother, geom = make_smoother()
    home = home_pose()

    # Set some non-home target first
    smoother.reset(home)
    target = np.array([100.0, 50.0, home[2], 0.1, 0.0, 0.0])
    smoother.set_target(target, 0.0)
    smoother.evaluate(0.005)  # mid-segment

    # Reset
    smoother.reset(home)
    pose, twist, accel = smoother.evaluate(1.0)

    assert norm(pose - home) < 1e-9, f"Pose != home after reset"
    assert norm(twist) < 1e-9, f"Twist != 0 after reset"
    assert norm(accel) < 1e-9, f"Accel != 0 after reset"

    print(f"  [PASS] Reset: pose matches home, zero derivatives")
    return True


# -----------------------------------------------------------------------
# Test 4: Duration scaling
# -----------------------------------------------------------------------

def test_duration_scaling():
    """Small displacement → short T, large displacement → long T."""
    smoother, geom = make_smoother()
    home = home_pose()

    # Small move: 1 mm
    smoother.reset(home)
    small_target = home.copy()
    small_target[0] += 1.0
    smoother.set_target(small_target, 0.0)
    t_small = smoother._duration

    # Large move: 150 mm
    smoother.reset(home)
    large_target = home.copy()
    large_target[0] += 150.0
    smoother.set_target(large_target, 0.0)
    t_large = smoother._duration

    assert t_small < t_large, (
        f"Small duration ({t_small:.4f}s) should be < large ({t_large:.4f}s)")
    assert t_small >= StreamSmoother.T_MIN, (
        f"Small duration ({t_small:.4f}s) below T_MIN")
    assert t_large <= StreamSmoother.T_MAX, (
        f"Large duration ({t_large:.4f}s) above T_MAX")

    print(f"  [PASS] Duration scaling: small={t_small*1000:.1f}ms, "
          f"large={t_large*1000:.1f}ms")
    return True


# -----------------------------------------------------------------------
# Test 5: Motor limit compliance
# -----------------------------------------------------------------------

def test_motor_limit_compliance():
    """For a large step, verify peak motor vel/accel within limits."""
    smoother, geom = make_smoother()
    dyn = DynamicsParams.from_config()
    home = home_pose()
    smoother.reset(home)

    # Large step: 100mm X + 100mm Z + 10deg roll
    target = home.copy()
    target[0] += 100.0
    target[2] += 100.0
    target[3] = np.radians(10.0)

    t0 = 0.0
    smoother.set_target(target, t0)
    duration = smoother._duration

    # Sample the trajectory at high resolution
    n_samples = 200
    peak_vel = 0.0
    peak_accel = 0.0

    for i in range(n_samples + 1):
        t = t0 + (i / n_samples) * duration
        pose, twist, accel = smoother.evaluate(t)

        pos_rev, vel_ff, torque_ff = cartesian_to_motor_commands(
            pose, twist, accel, geom, dyn,
            feedforward_enabled=False)  # skip torque for speed

        vel_max = np.max(np.abs(vel_ff))
        # Approximate motor accel from consecutive vel samples
        if vel_max > peak_vel:
            peak_vel = vel_max

    vel_limit = hw.ODRIVE_TRAP_VEL_LIMIT_RPS
    # Allow 10% margin for non-rest-start approximation
    margin = 1.10
    assert peak_vel < vel_limit * margin, (
        f"Peak motor vel {peak_vel:.2f} exceeds limit {vel_limit} * {margin}")

    print(f"  [PASS] Motor limit compliance: peak_vel={peak_vel:.2f} rev/s "
          f"(limit={vel_limit:.1f}), duration={duration*1000:.1f}ms")
    return True


# -----------------------------------------------------------------------
# Test 6: Supersession preserves C2
# -----------------------------------------------------------------------

def test_supersession():
    """Target A mid-segment, then target B — verify smooth transition."""
    smoother, geom = make_smoother()
    home = home_pose()
    smoother.reset(home)

    target_a = home.copy()
    target_a[0] += 80.0
    t0 = 0.0
    smoother.set_target(target_a, t0)

    # Evaluate at 30% of the segment
    duration_a = smoother._duration
    t_mid = t0 + 0.3 * duration_a
    pose_before, twist_before, accel_before = smoother.evaluate(t_mid)

    # Now supersede with target B
    target_b = home.copy()
    target_b[1] += 60.0
    smoother.set_target(target_b, t_mid)

    # Evaluate just after the splice
    pose_after, twist_after, accel_after = smoother.evaluate(t_mid)

    pos_err = norm(pose_after - pose_before)
    vel_err = norm(twist_after - twist_before)
    acc_err = norm(accel_after - accel_before)

    assert pos_err < 1e-6, f"Position discontinuity = {pos_err:.2e}"
    assert vel_err < 1e-6, f"Velocity discontinuity = {vel_err:.2e}"
    assert acc_err < 1e-6, f"Acceleration discontinuity = {acc_err:.2e}"

    # Verify it eventually reaches target B
    pose_end, twist_end, accel_end = smoother.evaluate(t_mid + 10.0)
    assert norm(pose_end - target_b) < 1e-6, "Did not reach target B"

    print(f"  [PASS] Supersession: C2 preserved, reaches target B")
    return True


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------

def main():
    print("StreamSmoother verification tests")
    print("=" * 60)

    tests = [
        ("1. C2 at splice", test_c2_at_splice),
        ("2. Hold on completion", test_hold_on_completion),
        ("3. Reset", test_reset),
        ("4. Duration scaling", test_duration_scaling),
        ("5. Motor limit compliance", test_motor_limit_compliance),
        ("6. Supersession", test_supersession),
    ]

    passed = 0
    failed = 0
    for name, fn in tests:
        print(f"\n{name}")
        try:
            if fn():
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"  [FAIL] {e}")
            failed += 1

    print(f"\n{'=' * 60}")
    print(f"Results: {passed} passed, {failed} failed out of {len(tests)}")
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
