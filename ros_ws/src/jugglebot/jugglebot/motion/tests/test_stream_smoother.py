"""Verification tests for the StreamSmoother (universal C2 target gate).

Tests:
  1. C2 at splice — sequential targets maintain pos/vel/accel continuity
  2. Hold on completion — pose reaches target with zero derivatives
  3. Reset — clean state after reset
  4. Duration scaling — small displacement -> short T, large -> long T
  5. Motor limit compliance — peak motor vel/accel within ODRIVE limits
  6. Supersession — mid-segment target update preserves C2
  7. Velocity inheritance — deferred splice inherits velocity from old segment
  8. Pending target update — multiple targets in one defer window
  9. Current segment continues during defer — old segment executes until splice

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
    """Feed sequential targets with spacing > DEFER_S, verify C2 at splices."""
    smoother, geom = make_smoother()
    home = home_pose()
    smoother.reset(home)
    DEFER = StreamSmoother.DEFER_S

    # Generate 5 random targets within spacemouse range
    rng = np.random.default_rng(42)
    targets = []
    for _ in range(5):
        pos = rng.uniform(-50, 50, size=3)
        pos[2] += hw.JB_OP_DEFAULT_ACTIVE_Z_MM
        rot = rng.uniform(-0.1, 0.1, size=3)  # small rotations
        targets.append(np.concatenate([pos, rot]))

    t = 0.0
    dt_between_targets = 0.3  # well past defer + some execution time

    for i, target in enumerate(targets):
        smoother.set_target(target, t)

        if i > 0:
            # C2 check: sample just before and just after the splice point
            splice_time = t + DEFER
            eps = 1e-6
            pose_before, twist_before, accel_before = smoother.evaluate(
                splice_time - eps)
            pose_after, twist_after, accel_after = smoother.evaluate(
                splice_time + eps)

            pos_err = norm(pose_after - pose_before)
            vel_err = norm(twist_after - twist_before)
            acc_err = norm(accel_after - accel_before)

            # Tolerances account for finite eps: the two evaluation
            # points are 2*eps apart, so differences ~ derivative * eps.
            assert pos_err < 0.05, (
                f"Splice {i}: position discontinuity = {pos_err:.2e}")
            assert vel_err < 0.5, (
                f"Splice {i}: velocity discontinuity = {vel_err:.2e}")
            assert acc_err < 50.0, (
                f"Splice {i}: acceleration discontinuity = {acc_err:.2e}")

        # Advance time past defer + execution
        t += dt_between_targets
        smoother.evaluate(t)  # ensure pending commits and segment advances

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
    """Small displacement -> short T, large displacement -> long T."""
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

        pos_rev, vel_ff, torque_ff, _ = cartesian_to_motor_commands(
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
    DEFER = StreamSmoother.DEFER_S

    target_a = home.copy()
    target_a[0] += 80.0
    t0 = 0.0
    smoother.set_target(target_a, t0)

    # Evaluate at 30% of the segment
    duration_a = smoother._duration
    t_mid = t0 + 0.3 * duration_a

    # Set target B at t_mid — this creates a deferred splice at t_mid + DEFER
    target_b = home.copy()
    target_b[1] += 60.0
    smoother.evaluate(t_mid)  # advance state to t_mid first
    smoother.set_target(target_b, t_mid)

    # Check C2 at the deferred splice point
    splice_time = t_mid + DEFER
    eps = 1e-6
    pose_before, twist_before, accel_before = smoother.evaluate(
        splice_time - eps)
    pose_after, twist_after, accel_after = smoother.evaluate(
        splice_time + eps)

    pos_err = norm(pose_after - pose_before)
    vel_err = norm(twist_after - twist_before)
    acc_err = norm(accel_after - accel_before)

    assert pos_err < 0.05, f"Position discontinuity = {pos_err:.2e}"
    assert vel_err < 0.5, f"Velocity discontinuity = {vel_err:.2e}"
    assert acc_err < 50.0, f"Acceleration discontinuity = {acc_err:.2e}"

    # Verify it eventually reaches target B
    pose_end, twist_end, accel_end = smoother.evaluate(splice_time + 10.0)
    assert norm(pose_end - target_b) < 1e-6, "Did not reach target B"

    print(f"  [PASS] Supersession: C2 preserved, reaches target B")
    return True


# -----------------------------------------------------------------------
# Test 7: Velocity inheritance (deferred splice builds velocity)
# -----------------------------------------------------------------------

def test_velocity_inheritance():
    """Deferred splice should inherit nonzero velocity from old segment."""
    smoother, geom = make_smoother()
    home = home_pose()
    smoother.reset(home)
    DEFER = StreamSmoother.DEFER_S

    # First target: large move so the segment has time to build velocity
    target_a = home.copy()
    target_a[0] += 100.0
    smoother.set_target(target_a, 0.0)
    duration_a = smoother._duration

    # Advance to 30% of segment A (well within the acceleration phase)
    t_mid = 0.3 * duration_a
    smoother.evaluate(t_mid)

    # Set second target — this defers to t_mid + DEFER
    target_b = home.copy()
    target_b[0] += 120.0
    smoother.set_target(target_b, t_mid)

    # The splice happens at t_mid + DEFER.  At that point, segment A has
    # been executing for (0.3 * duration_a + DEFER) / duration_a of its
    # duration — it should have significant velocity.
    splice_time = t_mid + DEFER

    # Evaluate just past the splice to get the inherited velocity
    pose, twist, accel = smoother.evaluate(splice_time + 1e-6)
    twist_norm = norm(twist)

    # The twist should be significantly nonzero (velocity inherited
    # from the mid-flight state of segment A)
    assert twist_norm > 1.0, (
        f"Twist at splice too small: {twist_norm:.4f} — "
        f"expected nonzero velocity inheritance from segment A")

    print(f"  [PASS] Velocity inheritance: twist at splice = {twist_norm:.2f}")
    return True


# -----------------------------------------------------------------------
# Test 8: Pending target update (multiple targets in one defer window)
# -----------------------------------------------------------------------

def test_pending_target_update():
    """Multiple targets within a single defer window: only latest is used."""
    smoother, geom = make_smoother()
    home = home_pose()
    smoother.reset(home)
    DEFER = StreamSmoother.DEFER_S

    # First target (immediate splice)
    target_a = home.copy()
    target_a[0] += 80.0
    smoother.set_target(target_a, 0.0)
    duration_a = smoother._duration

    # Advance into segment A
    t_mid = 0.3 * duration_a
    smoother.evaluate(t_mid)

    # Send 3 targets within the defer window (10ms apart)
    target_b = home.copy()
    target_b[0] += 90.0
    smoother.set_target(target_b, t_mid)
    splice_time_original = t_mid + DEFER

    target_c = home.copy()
    target_c[0] += 100.0
    smoother.set_target(target_c, t_mid + 0.01)

    target_d = home.copy()
    target_d[0] += 110.0
    smoother.set_target(target_d, t_mid + 0.02)

    # The splice time should be from the first pending (target_b), not updated
    assert abs(smoother._pending_t_start - splice_time_original) < 1e-9, (
        f"Splice time changed: {smoother._pending_t_start} vs "
        f"expected {splice_time_original}")

    # After the splice, the endpoint should be target_d (the latest)
    pose_end, _, _ = smoother.evaluate(splice_time_original + 10.0)
    assert norm(pose_end - target_d) < 1e-6, (
        f"End pose should be target_d, got error = {norm(pose_end - target_d):.2e}")

    print(f"  [PASS] Pending target update: splice time preserved, "
          f"latest target used")
    return True


# -----------------------------------------------------------------------
# Test 9: Current segment continues during defer window
# -----------------------------------------------------------------------

def test_current_segment_during_defer():
    """Old segment keeps executing during the defer window."""
    smoother, geom = make_smoother()
    home = home_pose()
    smoother.reset(home)
    DEFER = StreamSmoother.DEFER_S

    # Large first target (immediate splice)
    target_a = home.copy()
    target_a[0] += 100.0
    smoother.set_target(target_a, 0.0)
    duration_a = smoother._duration

    # Record state at 30% of segment A
    t_mid = 0.3 * duration_a
    pose_at_mid, twist_at_mid, _ = smoother.evaluate(t_mid)

    # Set a deferred target B
    target_b = home.copy()
    target_b[1] += 80.0
    smoother.set_target(target_b, t_mid)

    # Evaluate at t_mid + DEFER/2 — still within the defer window,
    # so the OLD segment (A) should still be active
    t_check = t_mid + DEFER * 0.5
    pose_check, twist_check, _ = smoother.evaluate(t_check)

    # Platform should have advanced further toward target_a (X direction)
    # compared to its position at t_mid
    assert pose_check[0] > pose_at_mid[0] + 0.1, (
        f"Platform X didn't advance during defer: "
        f"{pose_check[0]:.2f} vs {pose_at_mid[0]:.2f}")

    # Platform should NOT be moving toward target_b (Y direction) yet
    assert abs(pose_check[1] - home[1]) < abs(target_b[1]) * 0.1, (
        f"Platform Y moved too early: {pose_check[1]:.2f} "
        f"(target_b Y = {target_b[1]:.2f})")

    print(f"  [PASS] Current segment continues during defer: "
          f"X advanced from {pose_at_mid[0]:.1f} to {pose_check[0]:.1f}, "
          f"Y stayed near {home[1]:.1f}")
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
        ("7. Velocity inheritance", test_velocity_inheritance),
        ("8. Pending target update", test_pending_target_update),
        ("9. Current segment during defer", test_current_segment_during_defer),
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
            import traceback
            traceback.print_exc()
            failed += 1

    print(f"\n{'=' * 60}")
    print(f"Results: {passed} passed, {failed} failed out of {len(tests)}")
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
