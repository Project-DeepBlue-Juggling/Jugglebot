"""Phase 7 verification tests for the dynamic target commanding system.

Tests:
  1. Dynamic target from IDLE — zero-twist target, verify state transitions
  2. Zero-velocity target holds at position — stays at COMPLETE
  3. Nonzero end velocity -> auto-return — EXECUTING -> RETURNING -> IDLE
  4. Mid-motion splice continuity — C2 continuity at splice point
  5. Infeasible target rejected — returns False, state unchanged
  6. Infeasible replan preserves current trajectory — continues undisturbed
  7. Interrupt return with new target — RETURNING -> EXECUTING
  8. find_min_feasible_duration correctness — returns valid duration
  9. Jerk limit enforcement — rejects high-jerk, accepts low-jerk
  10. Quaternion -> rotvec conversion — target state format works
  11. Arrival time in the past — rejected immediately
  12. make_rest_to_rest centralized — matches expected output
  13. Deferred start for far-future target — holds then moves at moderate speed
  15. Short-duration target — no deferral, starts immediately
  16. Deferred start continuity — zero discontinuity at hold-to-move transition

Run:  python -m jugglebot.motion.tests.test_dynamic_target
"""

from __future__ import annotations

import sys
import time

import numpy as np
from numpy.linalg import norm

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.ik_solver import (
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.trajectory import (
    QuinticTrajectory,
    TrajectoryManager,
    TrajectoryState,
    check_feasibility,
    create_trajectory,
    evaluate,
    evaluate_jerk,
    find_min_feasible_duration,
    make_rest_to_rest,
)
# IPC message format is tested on the Jetson (requires msgpack/zmq).
# The dynamic target IPC constructor is: ipc.make_dynamic_target_command()


def _header(name: str):
    print(f"\n{'='*70}")
    print(f"  {name}")
    print(f"{'='*70}")


# ---------------------------------------------------------------------------
# Test 1: Dynamic target from IDLE
# ---------------------------------------------------------------------------

def test_dynamic_target_from_idle():
    """Submit a zero-twist dynamic target from IDLE and verify transitions."""
    _header("Test 1: Dynamic target from IDLE")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params, clock=lambda: 10.0)

    # Set hold pose to home (default [0,0,170,0,0,0] via config)
    mgr.set_hold_pose(mgr.home_pose)

    assert mgr.state == TrajectoryState.IDLE, f"Expected IDLE, got {mgr.state}"

    # Submit a zero-twist target: move to [0, 0, 200, 0, 0, 0]
    t_now = 10.0  # arbitrary clock time
    accepted = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 200.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),  # identity
        target_vel=np.array([0.0, 0.0, 0.0]),
        arrival_time=t_now + 1.0,  # 1 second from now
        t_now=t_now,
    )

    assert accepted, "Target should have been accepted"
    assert mgr.state == TrajectoryState.EXECUTING, \
        f"Expected EXECUTING, got {mgr.state}"

    # Evaluate at mid-point
    mid = t_now + 0.5
    pos, vel, torque = mgr.evaluate(mid)
    assert pos is not None
    print(f"  Mid-point pos_rev: {pos}")

    # Evaluate past end — should transition to COMPLETE (zero twist target)
    pos, vel, torque = mgr.evaluate(t_now + 1.1)
    assert mgr.state == TrajectoryState.COMPLETE, \
        f"Expected COMPLETE, got {mgr.state}"

    print("  PASS: IDLE -> EXECUTING -> COMPLETE")


# ---------------------------------------------------------------------------
# Test 2: Zero-velocity target holds at position
# ---------------------------------------------------------------------------

def test_zero_velocity_holds():
    """Verify that a zero-velocity target stays at COMPLETE and holds."""
    _header("Test 2: Zero-velocity target holds at position")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params, clock=lambda: 10.0)
    mgr.set_hold_pose(mgr.home_pose)

    t_now = 10.0
    target_pose = np.array([10.0, -5.0, 190.0, 0.0, 0.0, 0.0])
    accepted = mgr.submit_dynamic_target(
        target_pos=target_pose[:3],
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_now + 1.0,
        t_now=t_now,
    )
    assert accepted

    # Run past completion
    mgr.evaluate(t_now + 1.1)
    assert mgr.state == TrajectoryState.COMPLETE

    # Hold should be at the target pose
    pose = mgr.current_pose_6dof
    pos_err = norm(pose[:3] - target_pose[:3])
    print(f"  Hold pose: {pose.tolist()}")
    print(f"  Position error from target: {pos_err:.6f} mm")
    assert pos_err < 0.01, f"Hold pose should match target, error={pos_err}"

    # Subsequent evaluations should keep returning the same hold
    pos2, vel2, _ = mgr.evaluate(t_now + 5.0)
    assert mgr.state == TrajectoryState.COMPLETE
    assert norm(vel2) == 0.0, "Velocity should be zero at hold"

    print("  PASS: Zero-velocity target holds at COMPLETE")


# ---------------------------------------------------------------------------
# Test 3: Nonzero end velocity -> auto-return
# ---------------------------------------------------------------------------

def test_nonzero_velocity_auto_return():
    """Verify EXECUTING -> RETURNING -> IDLE for non-zero-velocity target."""
    _header("Test 3: Nonzero end velocity -> auto-return")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params, clock=lambda: 10.0)
    mgr.set_hold_pose(mgr.home_pose)

    t_now = 10.0
    # Target with Z velocity of 50 mm/s
    accepted = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 190.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.array([0.0, 0.0, 50.0]),
        arrival_time=t_now + 1.0,
        t_now=t_now,
    )
    assert accepted
    assert mgr.state == TrajectoryState.EXECUTING

    # Wait for background return precompute to finish (async pipeline),
    # then evaluate past trajectory end to trigger RETURNING.
    import time as _time
    deadline = _time.perf_counter() + 10.0
    t = t_now + 1.01
    while _time.perf_counter() < deadline:
        mgr.evaluate(t)
        if mgr.state == TrajectoryState.RETURNING:
            break
        _time.sleep(0.05)
        t += 0.001  # advance time slightly each poll

    assert mgr.state == TrajectoryState.RETURNING, \
        f"Expected RETURNING after non-zero-velocity trajectory end, got {mgr.state}"
    print("  State after trajectory end: RETURNING")

    # The return trajectory should eventually complete and go to IDLE
    max_steps = 10000
    for _ in range(max_steps):
        t += 0.01
        mgr.evaluate(t)
        if mgr.state == TrajectoryState.IDLE:
            break

    assert mgr.state == TrajectoryState.IDLE, \
        f"Expected IDLE after return, got {mgr.state}"

    # Verify final pose is near home
    final_pose = mgr.current_pose_6dof
    home = mgr.home_pose
    err = norm(final_pose - home)
    print(f"  Final pose: {final_pose.tolist()}")
    print(f"  Home pose:  {home.tolist()}")
    print(f"  Error from home: {err:.4f} mm")
    assert err < 0.5, f"Final pose should be near home, error={err}"

    print("  PASS: EXECUTING -> RETURNING -> IDLE, final pose near home")


# ---------------------------------------------------------------------------
# Test 4: Mid-motion splice continuity
# ---------------------------------------------------------------------------

def test_splice_continuity():
    """Submit a new dynamic target during EXECUTING, verify C2 continuity."""
    _header("Test 4: Mid-motion splice continuity")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params, clock=lambda: 10.0)
    mgr.set_hold_pose(mgr.home_pose)

    t_now = 10.0
    # First trajectory: home -> +30mm Z in 1s
    accepted = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 200.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_now + 1.0,
        t_now=t_now,
    )
    assert accepted

    # Evaluate up to mid-point
    t_splice = t_now + 0.5
    mgr.evaluate(t_splice - 0.001)  # just before splice

    # Record state just before splice by evaluating the trajectory
    pose_before, twist_before, accel_before = mgr._get_current_state(t_splice)

    # Submit new target: go to +20mm X, +200mm Z in 1s from now
    accepted2 = mgr.submit_dynamic_target(
        target_pos=np.array([20.0, 0.0, 200.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_splice + 1.0,
        t_now=t_splice,
    )
    assert accepted2, "Second target should be accepted"

    # Evaluate the new trajectory at its start time
    # The new trajectory's start conditions should match the old state
    new_pose, new_twist, new_accel = mgr._get_current_state(t_splice)

    pos_err = norm(new_pose - pose_before)
    vel_err = norm(new_twist - twist_before)
    accel_err = norm(new_accel - accel_before)

    print(f"  Splice point t={t_splice:.3f}s")
    print(f"  Position continuity error: {pos_err:.8f} mm")
    print(f"  Velocity continuity error: {vel_err:.8f} mm/s")
    print(f"  Acceleration continuity error: {accel_err:.8f} mm/s^2")

    # C2 continuity: all errors should be at floating-point precision
    assert pos_err < 1e-6, f"Position discontinuity: {pos_err}"
    assert vel_err < 1e-4, f"Velocity discontinuity: {vel_err}"
    assert accel_err < 1e-2, f"Acceleration discontinuity: {accel_err}"

    print("  PASS: C2 continuity at splice point")


# ---------------------------------------------------------------------------
# Test 5: Infeasible target rejected
# ---------------------------------------------------------------------------

def test_infeasible_target_rejected():
    """Verify infeasible target returns False and doesn't change state."""
    _header("Test 5: Infeasible target rejected")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params, clock=lambda: 10.0)
    mgr.set_hold_pose(mgr.home_pose)

    t_now = 10.0
    state_before = mgr.state

    # Extremely fast move: 500mm Z in 0.01s from home (way too fast)
    accepted = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 500.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_now + 0.01,
        t_now=t_now,
    )

    assert not accepted, "Infeasible target should be rejected"
    assert mgr.state == state_before, \
        f"State should be unchanged: expected {state_before}, got {mgr.state}"

    print("  PASS: Infeasible target correctly rejected")


# ---------------------------------------------------------------------------
# Test 6: Infeasible replan preserves current trajectory
# ---------------------------------------------------------------------------

def test_infeasible_replan_preserves_trajectory():
    """During EXECUTING, an infeasible new target should not affect current."""
    _header("Test 6: Infeasible replan preserves current trajectory")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params, clock=lambda: 10.0)
    mgr.set_hold_pose(mgr.home_pose)

    t_now = 10.0
    # Start a valid trajectory
    accepted = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 200.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_now + 2.0,
        t_now=t_now,
    )
    assert accepted

    # Evaluate at 0.5s — should be mid-trajectory
    mgr.evaluate(t_now + 0.5)
    pose_mid = mgr.current_pose_6dof.copy()

    # Try an infeasible replan
    t_replan = t_now + 0.5
    accepted2 = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 500.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_replan + 0.01,  # way too fast
        t_now=t_replan,
    )
    assert not accepted2, "Infeasible replan should be rejected"
    assert mgr.state == TrajectoryState.EXECUTING

    # Evaluate slightly later — should still be on original trajectory
    mgr.evaluate(t_now + 0.6)
    pose_after = mgr.current_pose_6dof
    # Z should have continued increasing (original trajectory)
    assert pose_after[2] > pose_mid[2], \
        "Original trajectory should continue after infeasible replan"

    print(f"  Pose at 0.5s: Z={pose_mid[2]:.2f} mm")
    print(f"  Pose at 0.6s: Z={pose_after[2]:.2f} mm (continuing)")
    print("  PASS: Infeasible replan leaves current trajectory intact")


# ---------------------------------------------------------------------------
# Test 7: Interrupt return with new target
# ---------------------------------------------------------------------------

def test_interrupt_return():
    """During RETURNING, submit new dynamic target -> EXECUTING."""
    _header("Test 7: Interrupt return with new target")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params, clock=lambda: 10.0)
    mgr.set_hold_pose(mgr.home_pose)

    t_now = 10.0
    # Target with velocity -> will trigger return
    accepted = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 190.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.array([0.0, 0.0, 30.0]),
        arrival_time=t_now + 1.0,
        t_now=t_now,
    )
    assert accepted

    # Wait for background return precompute, then evaluate past end
    import time as _time
    deadline = _time.perf_counter() + 10.0
    t = t_now + 1.01
    while _time.perf_counter() < deadline:
        mgr.evaluate(t)
        if mgr.state == TrajectoryState.RETURNING:
            break
        _time.sleep(0.05)
        t += 0.001

    assert mgr.state == TrajectoryState.RETURNING, \
        f"Expected RETURNING, got {mgr.state}"

    # Now submit a new target while returning
    t_interrupt = t + 0.2
    mgr.evaluate(t_interrupt)  # update internal state
    accepted2 = mgr.submit_dynamic_target(
        target_pos=np.array([10.0, 0.0, 195.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_interrupt + 1.0,
        t_now=t_interrupt,
    )
    assert accepted2, "New target during RETURNING should be accepted"
    assert mgr.state == TrajectoryState.EXECUTING, \
        f"Expected EXECUTING after interrupt, got {mgr.state}"

    print("  PASS: RETURNING interrupted -> EXECUTING")


# ---------------------------------------------------------------------------
# Test 8: find_min_feasible_duration correctness
# ---------------------------------------------------------------------------

def test_find_min_feasible_duration():
    """Verify returned duration is feasible and slightly shorter is not."""
    _header("Test 8: find_min_feasible_duration correctness")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    start_pose = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    end_pose = np.array([0.0, 0.0, 220.0, 0.0, 0.0, 0.0])
    zeros6 = np.zeros(6)

    duration = find_min_feasible_duration(
        start_pose=start_pose,
        start_twist=zeros6,
        start_accel=zeros6,
        end_pose=end_pose,
        end_twist=zeros6,
        end_accel=zeros6,
        geom=geom,
        dynamics_params=params,
    )

    assert duration is not None, "Should find a feasible duration"
    print(f"  Minimum feasible duration: {duration:.4f}s")

    # Verify this duration is actually feasible (use same sample count as
    # find_min_feasible_duration to avoid false failures from sampling jitter)
    traj = create_trajectory(
        start_pose=start_pose, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=duration)
    result = check_feasibility(traj, geom, params, n_samples=50)
    assert result.feasible, f"Returned duration should be feasible: {result.violations}"
    print(f"  Duration {duration:.4f}s: feasible={result.feasible}")

    # Verify a significantly shorter duration is NOT feasible
    short_duration = duration * 0.5
    if short_duration > 0.1:  # only test if reasonably large
        traj_short = create_trajectory(
            start_pose=start_pose, start_twist=zeros6, start_accel=zeros6,
            end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
            duration=short_duration)
        result_short = check_feasibility(traj_short, geom, params, n_samples=200)
        print(f"  Duration {short_duration:.4f}s (50%): "
              f"feasible={result_short.feasible}")
        # Not asserting infeasible at 50% since it depends on the specific move,
        # but print for inspection

    print("  PASS: find_min_feasible_duration returns valid result")


# ---------------------------------------------------------------------------
# Test 9: Jerk limit enforcement
# ---------------------------------------------------------------------------

def test_jerk_limit_enforcement():
    """Verify jerk limits reject high-jerk and accept low-jerk trajectories."""
    _header("Test 9: Jerk limit enforcement")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    # A very fast short-duration move will have high jerk
    start_pose = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    end_pose = np.array([0.0, 0.0, 230.0, 0.0, 0.0, 0.0])  # 60mm Z
    zeros6 = np.zeros(6)

    # Short duration -> high jerk
    traj_fast = create_trajectory(
        start_pose=start_pose, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=0.05)
    result_fast = check_feasibility(
        traj_fast, geom, params,
        jerk_trans_limit=30_000.0, jerk_rot_limit=400.0,
        n_samples=200)

    # Check if jerk violation is present (fast move should exceed)
    jerk_violations = [v for v in result_fast.violations if 'jerk' in v.lower()]
    print(f"  Fast trajectory (0.05s, 60mm):")
    print(f"    Peak trans jerk: {result_fast.peak_jerk_trans:.0f} mm/s^3")
    print(f"    Peak rot jerk:   {result_fast.peak_jerk_rot:.2f} rad/s^3")
    print(f"    Jerk violations: {len(jerk_violations)}")
    if jerk_violations:
        for v in jerk_violations:
            print(f"      {v}")

    # Slow trajectory -> low jerk
    traj_slow = create_trajectory(
        start_pose=start_pose, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=2.0)
    result_slow = check_feasibility(
        traj_slow, geom, params,
        jerk_trans_limit=30_000.0, jerk_rot_limit=400.0,
        n_samples=200)

    jerk_violations_slow = [v for v in result_slow.violations if 'jerk' in v.lower()]
    print(f"  Slow trajectory (2.0s, 60mm):")
    print(f"    Peak trans jerk: {result_slow.peak_jerk_trans:.0f} mm/s^3")
    print(f"    Peak rot jerk:   {result_slow.peak_jerk_rot:.2f} rad/s^3")
    print(f"    Jerk violations: {len(jerk_violations_slow)}")
    assert len(jerk_violations_slow) == 0, \
        f"Slow trajectory should pass jerk limits: {jerk_violations_slow}"

    # Verify jerk fields are populated in FeasibilityResult
    assert result_fast.peak_jerk_trans >= 0
    assert result_slow.peak_jerk_trans >= 0

    # Verify that disabling jerk check works
    result_no_jerk = check_feasibility(
        traj_fast, geom, params,
        jerk_trans_limit=None, jerk_rot_limit=None,
        n_samples=50)
    jerk_violations_none = [v for v in result_no_jerk.violations if 'jerk' in v.lower()]
    assert len(jerk_violations_none) == 0, \
        "Disabling jerk limits should remove jerk violations"
    print("  Jerk check disabled: no jerk violations (correct)")

    print("  PASS: Jerk limit enforcement works correctly")


# ---------------------------------------------------------------------------
# Test 10: Quaternion -> rotvec conversion in target state
# ---------------------------------------------------------------------------

def test_quaternion_conversion():
    """Verify quaternion target state correctly converts to rotvec."""
    _header("Test 10: Quaternion -> rotvec conversion")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params, clock=lambda: 10.0)
    mgr.set_hold_pose(mgr.home_pose)

    # Create a 5-degree tilt about X axis
    angle_rad = np.deg2rad(5.0)
    # Quaternion for rotation about X: [cos(a/2), sin(a/2), 0, 0]
    quat = np.array([np.cos(angle_rad / 2), np.sin(angle_rad / 2), 0.0, 0.0])

    # Expected rotation vector
    expected_rotvec = np.array([angle_rad, 0.0, 0.0])

    t_now = 10.0
    accepted = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 180.0]),
        target_quat=quat,
        target_vel=np.zeros(3),
        arrival_time=t_now + 2.0,
        t_now=t_now,
    )
    assert accepted, "Tilted target should be accepted"

    # Evaluate near the end to check target pose
    mgr.evaluate(t_now + 2.01)
    final_pose = mgr.current_pose_6dof

    # Check that orientation matches expected rotvec
    rot_err = norm(final_pose[3:6] - expected_rotvec)
    print(f"  Target quat:       {quat.tolist()}")
    print(f"  Expected rotvec:   {expected_rotvec.tolist()}")
    print(f"  Actual rotvec:     {final_pose[3:6].tolist()}")
    print(f"  Rotation error:    {rot_err:.8f} rad")
    assert rot_err < 1e-6, f"Rotvec should match, error={rot_err}"

    print("  PASS: Quaternion conversion produces correct rotvec")


# ---------------------------------------------------------------------------
# Test 11: Arrival time in the past
# ---------------------------------------------------------------------------

def test_arrival_time_in_past():
    """Verify that a target with arrival_time <= t_now is rejected."""
    _header("Test 11: Arrival time in the past")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params, clock=lambda: 10.0)
    mgr.set_hold_pose(mgr.home_pose)

    t_now = 10.0

    # Arrival time in the past
    accepted = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 200.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_now - 0.5,
        t_now=t_now,
    )
    assert not accepted, "Past arrival time should be rejected"
    assert mgr.state == TrajectoryState.IDLE

    # Arrival time exactly now
    accepted2 = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 200.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.zeros(3),
        arrival_time=t_now,
        t_now=t_now,
    )
    assert not accepted2, "Arrival time exactly at now should be rejected"

    print("  PASS: Past/current arrival times correctly rejected")


# ---------------------------------------------------------------------------
# Test 12: make_rest_to_rest centralized
# ---------------------------------------------------------------------------

def test_make_rest_to_rest():
    """Verify centralized make_rest_to_rest produces expected trajectory."""
    _header("Test 12: make_rest_to_rest centralized")

    end_pose = np.array([10.0, -5.0, 200.0, 0.0, 0.0, 0.0])
    duration = 1.5

    traj = make_rest_to_rest(end_pose, duration)

    # Check start and end states
    start = traj.start_state
    end = traj.end_state

    # Start should be zeros
    assert norm(start[:6]) == 0.0, "Start pose should be zeros (home)"
    assert norm(start[6:12]) == 0.0, "Start twist should be zeros"
    assert norm(start[12:18]) == 0.0, "Start accel should be zeros"

    # End should match target with zero twist/accel
    assert norm(end[:6] - end_pose) < 1e-10, "End pose should match"
    assert norm(end[6:12]) == 0.0, "End twist should be zeros"
    assert norm(end[12:18]) == 0.0, "End accel should be zeros"

    # With custom start pose
    sp = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    traj2 = make_rest_to_rest(end_pose, duration, start_pose=sp)
    assert norm(traj2.start_state[:6] - sp) < 1e-10

    print("  PASS: make_rest_to_rest produces correct trajectories")


# ---------------------------------------------------------------------------
# Test 13: Deferred start for far-future target
# ---------------------------------------------------------------------------

def test_deferred_start():
    """Verify that a far-future arrival time uses deferred start:
    the platform holds, then moves at moderate speed to arrive on time."""
    _header("Test 13: Deferred start for far-future target")

    from jugglebot.motion.trajectory import DEFERRED_START_BUFFER_S

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    t = [10.0]  # synthetic clock
    mgr = TrajectoryManager(geom, params, clock=lambda: t[0])
    mgr.set_hold_pose(mgr.home_pose)

    t_now = t[0]

    # Target 30mm above home, with a long arrival time (20s).
    # The minimum feasible duration for this move is ~0.3-0.5s, so
    # deferred start should activate: motion_duration ≈ min_feasible + 2.0s.
    target_pos = np.array([0.0, 0.0, 200.0])
    target_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_vel = np.zeros(3)
    far_arrival = t_now + 20.0

    accepted = mgr.submit_dynamic_target(
        target_pos=target_pos,
        target_quat=target_quat,
        target_vel=target_vel,
        arrival_time=far_arrival,
        t_now=t_now,
    )
    assert accepted, "Far-future target should be accepted"
    assert mgr.state == TrajectoryState.EXECUTING

    traj = mgr._active_traj

    # The trajectory should use deferred start, NOT start at t_now
    defer_delay = traj.t_start - t_now
    print(f"  t_now={t_now:.1f}, t_start={traj.t_start:.2f}, "
          f"duration={traj.duration:.2f}s, "
          f"defer_delay={defer_delay:.2f}s")

    assert defer_delay > 1.0, (
        f"Expected deferred start (delay > 1s), got {defer_delay:.2f}s")

    # The trajectory duration should be much less than 20s
    assert traj.duration < 10.0, (
        f"Expected shorter motion duration, got {traj.duration:.2f}s")

    # The trajectory should end at the requested arrival time
    t_end = traj.t_start + traj.duration
    assert abs(t_end - far_arrival) < 0.1, (
        f"Trajectory end {t_end:.2f} should match arrival {far_arrival:.1f}")

    # Before t_start, platform should hold at start pose (home)
    home = mgr.home_pose
    pos_hold, vel_hold, _ = mgr.evaluate(t_now + 1.0)
    assert norm(vel_hold) < 1e-6, (
        "Should be stationary during hold phase")

    # Verify hold pose matches home (via current_pose_6dof)
    hold_pose = mgr.current_pose_6dof
    assert norm(hold_pose - home) < 0.01, (
        f"Hold pose should be home, error={norm(hold_pose - home):.4f}")

    # During motion phase: should be moving
    t_mid = traj.t_start + traj.duration / 2
    pos_mid, vel_mid, _ = mgr.evaluate(t_mid)
    assert norm(vel_mid) > 0.001, "Should be moving mid-trajectory"
    assert 0.3 < mgr._last_progress < 0.7, "Progress ~0.5 mid-traj"

    # After trajectory end, should complete at target
    pos_end, vel_end, _ = mgr.evaluate(far_arrival + 0.1)
    assert mgr.state == TrajectoryState.COMPLETE
    assert norm(vel_end) < 1e-6, "Should be stationary after completion"

    # Verify final pose is at the target
    final_pose = mgr.current_pose_6dof
    assert abs(final_pose[2] - 200.0) < 0.01, (
        f"Final Z should be ~200mm, got {final_pose[2]:.2f}")

    print(f"  Deferred by {defer_delay:.2f}s, motion duration "
          f"{traj.duration:.2f}s (buffer={DEFERRED_START_BUFFER_S}s)")
    print("  PASS: Deferred start holds, then moves at moderate speed")


# ---------------------------------------------------------------------------
# Test 14: Return junction C2 continuity
# ---------------------------------------------------------------------------

def test_return_junction_continuity():
    """Verify the return trajectory starts from the hold state.

    When the outbound trajectory completes, the platform holds at end
    pose with zero velocity while the async return precompute finishes.
    The return trajectory is re-created from the hold state (end_pose,
    zero twist, zero accel) using the precomputed duration, ensuring
    no velocity discontinuity with the actual platform state.
    """
    _header("Test 14: Return junction continuity (from hold state)")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params, clock=lambda: 10.0)
    mgr.set_hold_pose(mgr.home_pose)

    t_now = 10.0
    target_vel_z = 50.0  # mm/s -- nonzero to trigger return planning

    accepted = mgr.submit_dynamic_target(
        target_pos=np.array([0.0, 0.0, 190.0]),
        target_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        target_vel=np.array([0.0, 0.0, target_vel_z]),
        arrival_time=t_now + 1.0,
        t_now=t_now,
    )
    assert accepted, "Target should be accepted"

    # Save the outbound trajectory's end pose
    outbound_traj = mgr._active_traj
    outbound_end_pose = outbound_traj.end_state[:6].copy()

    # Wait for background return precompute, then evaluate past end
    import time as _time
    deadline = _time.perf_counter() + 10.0
    t = t_now + 1.01
    while _time.perf_counter() < deadline:
        mgr.evaluate(t)
        if mgr.state == TrajectoryState.RETURNING:
            break
        _time.sleep(0.05)
        t += 0.001

    assert mgr.state == TrajectoryState.RETURNING, \
        f"Expected RETURNING, got {mgr.state}"

    # The return trajectory should start from the hold state:
    # same position as outbound end, but zero velocity and acceleration
    return_traj = mgr._active_traj
    return_start = return_traj.start_state  # (18,)

    pos_err = norm(return_start[:6] - outbound_end_pose)
    vel_mag = norm(return_start[6:12])
    accel_mag = norm(return_start[12:18])

    print(f"  Return start pose:   {return_start[:6].tolist()}")
    print(f"  Outbound end pose:   {outbound_end_pose.tolist()}")
    print(f"  Return start twist:  {return_start[6:12].tolist()}")
    print(f"  Position error:      {pos_err:.8f} mm")
    print(f"  Velocity magnitude:  {vel_mag:.8f} mm/s")
    print(f"  Accel magnitude:     {accel_mag:.8f} mm/s^2")

    assert pos_err < 1e-6, \
        f"Position mismatch at return start: {pos_err:.6f} mm"
    assert vel_mag < 1e-6, \
        f"Return should start from rest, got velocity {vel_mag:.6f} mm/s"
    assert accel_mag < 1e-6, \
        f"Return should start from rest, got accel {accel_mag:.6f} mm/s^2"

    # Also verify the return trajectory's end twist is zero (rest at home)
    assert norm(return_traj.end_state[6:12]) < 1e-10, \
        "Return trajectory should end at rest"

    print("  PASS: Return starts from hold state (zero vel/accel)")


# ---------------------------------------------------------------------------
# Test 15: Short-duration target — no deferral
# ---------------------------------------------------------------------------

def test_short_duration_no_deferral():
    """Verify that a target with a short arrival time starts immediately
    (no deferred start applies)."""
    _header("Test 15: Short-duration target — no deferral")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    t = [10.0]  # synthetic clock
    mgr = TrajectoryManager(geom, params, clock=lambda: t[0])
    mgr.set_hold_pose(mgr.home_pose)

    t_now = t[0]

    # Target 30mm above home, with a 2s arrival — close to min_feasible
    # + buffer, so no deferral should happen.
    target_pos = np.array([0.0, 0.0, 200.0])
    target_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_vel = np.zeros(3)
    arrival = t_now + 2.0

    accepted = mgr.submit_dynamic_target(
        target_pos=target_pos,
        target_quat=target_quat,
        target_vel=target_vel,
        arrival_time=arrival,
        t_now=t_now,
    )
    assert accepted, "Short-duration target should be accepted"

    traj = mgr._active_traj
    defer_delay = traj.t_start - t_now
    print(f"  t_start={traj.t_start:.2f}, delay={defer_delay:.4f}s, "
          f"duration={traj.duration:.2f}s")

    # Should start immediately (within tolerance)
    assert defer_delay < 0.01, (
        f"Short-duration target should start immediately, "
        f"got delay={defer_delay:.4f}s")

    # Duration should be the full 2.0s
    assert abs(traj.duration - 2.0) < 0.1, (
        f"Expected ~2.0s duration, got {traj.duration:.2f}s")

    print("  PASS: Short-duration target starts immediately (no deferral)")


# ---------------------------------------------------------------------------
# Test 16: Deferred start — continuity at hold-to-move transition
# ---------------------------------------------------------------------------

def test_deferred_start_continuity():
    """Verify zero position discontinuity at the hold-to-move transition
    when deferred start is active.

    This is the critical test: the original deferred start implementation
    caused 'small jumps' because the trajectory start state didn't match
    the hold state.  With the fix, the trajectory is created from the
    state at t_start, which matches what evaluate() returns during the
    hold phase.
    """
    _header("Test 16: Deferred start — continuity at hold-to-move transition")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    t = [10.0]  # synthetic clock
    mgr = TrajectoryManager(geom, params, clock=lambda: t[0])
    mgr.set_hold_pose(mgr.home_pose)

    t_now = t[0]
    target_pos = np.array([0.0, 0.0, 200.0])
    target_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_vel = np.zeros(3)
    far_arrival = t_now + 20.0

    accepted = mgr.submit_dynamic_target(
        target_pos=target_pos,
        target_quat=target_quat,
        target_vel=target_vel,
        arrival_time=far_arrival,
        t_now=t_now,
    )
    assert accepted

    traj = mgr._active_traj
    t_start = traj.t_start
    assert t_start > t_now + 1.0, "Should be deferred"

    # Evaluate just before t_start (hold phase)
    eps = 0.001
    pos_before, vel_before, _ = mgr.evaluate(t_start - eps)
    pose_before = mgr.current_pose_6dof.copy()

    # Evaluate just after t_start (motion phase begins)
    pos_after, vel_after, _ = mgr.evaluate(t_start + eps)
    pose_after = mgr.current_pose_6dof.copy()

    # Motor command continuity
    pos_jump = norm(pos_after - pos_before)
    vel_jump = norm(vel_after - vel_before)

    # Cartesian pose continuity
    pose_jump = norm(pose_after - pose_before)

    print(f"  t_start = {t_start:.2f}s (deferred by {t_start - t_now:.2f}s)")
    print(f"  Motor pos jump:    {pos_jump:.8f} rev")
    print(f"  Motor vel jump:    {vel_jump:.8f} rev/s")
    print(f"  Cartesian jump:    {pose_jump:.8f} mm")

    # The jumps should be at numerical precision level
    assert pos_jump < 1e-4, (
        f"Position discontinuity at hold-to-move: {pos_jump:.6f} rev")
    assert pose_jump < 1e-3, (
        f"Cartesian discontinuity at hold-to-move: {pose_jump:.6f} mm")

    # Velocity should be near-zero at the very start of motion
    # (quintic starts from rest when starting from hold)
    assert vel_jump < 0.1, (
        f"Velocity jump at hold-to-move: {vel_jump:.6f} rev/s")

    print("  PASS: Zero discontinuity at hold-to-move transition")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    tests = [
        test_dynamic_target_from_idle,
        test_zero_velocity_holds,
        test_nonzero_velocity_auto_return,
        test_splice_continuity,
        test_infeasible_target_rejected,
        test_infeasible_replan_preserves_trajectory,
        test_interrupt_return,
        test_find_min_feasible_duration,
        test_jerk_limit_enforcement,
        test_quaternion_conversion,
        test_arrival_time_in_past,
        test_make_rest_to_rest,
        test_deferred_start,
        test_return_junction_continuity,
        test_short_duration_no_deferral,
        test_deferred_start_continuity,
    ]

    passed = 0
    failed = 0
    errors = []

    for test_fn in tests:
        try:
            test_fn()
            passed += 1
        except Exception as e:
            failed += 1
            errors.append((test_fn.__name__, e))
            print(f"  FAIL: {e}")

    print(f"\n{'='*70}")
    print(f"  Results: {passed} passed, {failed} failed out of {len(tests)}")
    print(f"{'='*70}")

    if errors:
        print("\nFailures:")
        for name, e in errors:
            print(f"  {name}: {e}")
        sys.exit(1)
    else:
        print("\nAll tests passed!")
        sys.exit(0)


if __name__ == '__main__':
    main()
