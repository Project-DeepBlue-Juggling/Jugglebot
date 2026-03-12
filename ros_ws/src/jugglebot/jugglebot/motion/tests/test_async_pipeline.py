"""Async feasibility pipeline verification tests.

Tests:
  1. Non-blocking submit — request_dynamic_target returns immediately
  2. Async acceptance — background check completes and result is accepted
  3. Async rejection — infeasible target rejected without stalling
  4. Supersession — newer target discards stale result
  5. Deceleration pre-computation — precomputed decel available on completion
  6. Cancel discards pending — cancel() clears async state
  7. Early-exit speedup — early_exit flag reduces sample count
  8. Torque skip speedup — no torque limit skips torque computation

Run:  python -m jugglebot.motion.tests.test_async_pipeline
"""

from __future__ import annotations

import sys
import time

import numpy as np
from numpy.linalg import norm

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.trajectory import (
    create_trajectory,
    evaluate,
    check_feasibility,
    TrajectoryManager,
    TrajectoryState,
)


def _header(name: str):
    print(f"\n{'='*70}")
    print(f"  {name}")
    print(f"{'='*70}")


# ---------------------------------------------------------------------------
# Test 1: Non-blocking submit
# ---------------------------------------------------------------------------

def test_nonblocking_submit():
    """Verify request_dynamic_target() returns immediately (< 5ms)."""
    _header("Test 1: Non-blocking submit")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(np.zeros(6))

    t_now = time.perf_counter()
    target_pos = np.array([0.0, 0.0, 10.0])
    target_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_vel = np.zeros(3)
    arrival_time = t_now + 1.0

    t_before = time.perf_counter()
    mgr.request_dynamic_target(
        target_pos, target_quat, target_vel, arrival_time, t_now)
    elapsed_ms = (time.perf_counter() - t_before) * 1000

    assert elapsed_ms < 5.0, (
        f"request_dynamic_target took {elapsed_ms:.1f}ms, expected < 5ms")
    print(f"  request_dynamic_target elapsed: {elapsed_ms:.2f}ms")
    print("  [PASS]")

    mgr.shutdown()


# ---------------------------------------------------------------------------
# Test 2: Async acceptance
# ---------------------------------------------------------------------------

def test_async_acceptance():
    """Verify async pipeline accepts a feasible target."""
    _header("Test 2: Async acceptance")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(np.zeros(6))

    t_now = time.perf_counter()
    target_pos = np.array([0.0, 0.0, 10.0])
    target_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_vel = np.zeros(3)
    arrival_time = t_now + 2.0

    mgr.request_dynamic_target(
        target_pos, target_quat, target_vel, arrival_time, t_now)

    # Wait for background check to complete (with timeout)
    deadline = time.perf_counter() + 5.0
    result = None
    while time.perf_counter() < deadline:
        result = mgr.poll_pending_result()
        if result is not None:
            break
        time.sleep(0.01)

    assert result is not None, "Async result never arrived (5s timeout)"
    assert result['accepted'], f"Expected accepted, got rejected: {result}"

    latency_ms = (time.perf_counter() - t_now) * 1000
    print(f"  Async result arrived in {latency_ms:.0f}ms")
    print(f"  accepted={result['accepted']}")

    # Commit
    accepted = mgr.commit_async_trajectory(result)
    assert accepted, "commit_async_trajectory returned False"
    assert mgr.state == TrajectoryState.EXECUTING, (
        f"Expected EXECUTING, got {mgr.state}")
    print(f"  Committed, state={mgr.state.value}")
    print("  [PASS]")

    mgr.shutdown()


# ---------------------------------------------------------------------------
# Test 3: Async rejection
# ---------------------------------------------------------------------------

def test_async_rejection():
    """Verify infeasible target rejected without stalling."""
    _header("Test 3: Async rejection")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(np.zeros(6))

    t_now = time.perf_counter()
    # Extreme target: 500mm Z displacement in 0.01s — clearly infeasible
    target_pos = np.array([0.0, 0.0, 500.0])
    target_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_vel = np.zeros(3)
    arrival_time = t_now + 0.01

    t_before = time.perf_counter()
    mgr.request_dynamic_target(
        target_pos, target_quat, target_vel, arrival_time, t_now)
    submit_ms = (time.perf_counter() - t_before) * 1000

    # Wait for result
    deadline = time.perf_counter() + 5.0
    result = None
    while time.perf_counter() < deadline:
        result = mgr.poll_pending_result()
        if result is not None:
            break
        time.sleep(0.01)

    # Duration was <= 0, so it should have been rejected at the request
    # stage (before queuing). Try a feasible-duration but workspace-infeasible:
    t_now2 = time.perf_counter()
    target_pos2 = np.array([200.0, 200.0, 300.0])  # way outside workspace
    arrival_time2 = t_now2 + 0.5  # must exceed MIN_LEAD_TIME_S (0.3s)

    mgr.request_dynamic_target(
        target_pos2, target_quat, target_vel, arrival_time2, t_now2)

    deadline = time.perf_counter() + 5.0
    result = None
    while time.perf_counter() < deadline:
        result = mgr.poll_pending_result()
        if result is not None:
            break
        time.sleep(0.01)

    assert result is not None, "Async result never arrived for infeasible target"
    assert not result['accepted'], f"Expected rejected, got accepted"

    total_ms = (time.perf_counter() - t_now2) * 1000
    print(f"  Infeasible target rejected in {total_ms:.0f}ms")
    assert mgr.state == TrajectoryState.IDLE, (
        f"State should remain IDLE, got {mgr.state}")
    print("  State remains IDLE")
    print("  [PASS]")

    mgr.shutdown()


# ---------------------------------------------------------------------------
# Test 4: Supersession
# ---------------------------------------------------------------------------

def test_supersession():
    """Verify newer request supersedes stale result."""
    _header("Test 4: Supersession (stale result discarded)")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(np.zeros(6))

    t_now = time.perf_counter()
    target_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_vel = np.zeros(3)

    # Submit first target
    mgr.request_dynamic_target(
        np.array([0.0, 0.0, 10.0]), target_quat, target_vel,
        t_now + 2.0, t_now)

    # Immediately submit second target (supersedes first)
    t_now2 = time.perf_counter()
    mgr.request_dynamic_target(
        np.array([0.0, 0.0, 20.0]), target_quat, target_vel,
        t_now2 + 2.0, t_now2)

    # Wait for result
    deadline = time.perf_counter() + 5.0
    result = None
    while time.perf_counter() < deadline:
        result = mgr.poll_pending_result()
        if result is not None:
            break
        time.sleep(0.01)

    assert result is not None, "No async result arrived"
    # The result should be for the second (latest) target
    assert result['accepted'], "Second target should be accepted"
    print(f"  Got result for generation {result['generation']}")

    # Commit should succeed (it's the latest generation)
    accepted = mgr.commit_async_trajectory(result)
    assert accepted, "commit should succeed for latest generation"
    print("  Latest result committed successfully")
    print("  [PASS]")

    mgr.shutdown()


# ---------------------------------------------------------------------------
# Test 5: Deceleration pre-computation
# ---------------------------------------------------------------------------

def test_decel_precompute():
    """Verify precomputed deceleration is available when outbound completes."""
    _header("Test 5: Deceleration pre-computation")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(np.zeros(6))

    # Submit an outbound trajectory with nonzero end velocity
    t_now = time.perf_counter()
    target_pos = np.array([0.0, 0.0, 20.0])
    target_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_vel = np.array([0.0, 0.0, 50.0])  # nonzero → triggers decel
    arrival_time = t_now + 3.0

    mgr.request_dynamic_target(
        target_pos, target_quat, target_vel, arrival_time, t_now)

    # Wait for feasibility result
    deadline = time.perf_counter() + 5.0
    result = None
    while time.perf_counter() < deadline:
        result = mgr.poll_pending_result()
        if result is not None:
            break
        time.sleep(0.01)

    assert result is not None and result['accepted'], "Target should be accepted"
    assert result['needs_decel'], "Target with nonzero velocity needs decel"

    # Commit (this triggers background deceleration precomputation)
    accepted = mgr.commit_async_trajectory(result)
    assert accepted, "Commit should succeed"
    assert mgr.state == TrajectoryState.EXECUTING

    # Wait for precomputed deceleration to become available
    deadline = time.perf_counter() + 10.0
    precomputed = None
    while time.perf_counter() < deadline:
        precomputed = mgr.poll_precomputed_decel()
        if precomputed is not None:
            break
        time.sleep(0.05)

    assert precomputed is not None, (
        "Pre-computed deceleration trajectory not available within 10s")
    assert precomputed.duration > 0, "Decel trajectory should have positive duration"
    # Verify end state is at rest
    end_twist = precomputed.end_state[6:12]
    assert np.allclose(end_twist, 0, atol=1e-6), (
        f"Decel trajectory should end at rest, got twist={end_twist}")
    print(f"  Pre-computed decel: duration={precomputed.duration:.3f}s")
    print("  [PASS]")

    mgr.shutdown()


# ---------------------------------------------------------------------------
# Test 6: Cancel discards pending
# ---------------------------------------------------------------------------

def test_cancel_discards():
    """Verify cancel() clears async state."""
    _header("Test 6: Cancel discards pending results")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(np.zeros(6))

    t_now = time.perf_counter()
    mgr.request_dynamic_target(
        np.array([0.0, 0.0, 10.0]),
        np.array([1.0, 0.0, 0.0, 0.0]),
        np.zeros(3),
        t_now + 2.0, t_now)

    # Cancel before the background check completes
    mgr.cancel()

    # Wait briefly and check no result arrives
    time.sleep(0.5)
    result = mgr.poll_pending_result()

    # Even if a result arrives, commit should fail (stale generation)
    if result is not None:
        accepted = mgr.commit_async_trajectory(result)
        assert not accepted, "Commit should fail after cancel (stale generation)"
        print("  Result arrived but commit rejected (stale generation)")
    else:
        print("  No result arrived after cancel")

    assert mgr.state == TrajectoryState.IDLE, f"State should be IDLE, got {mgr.state}"
    print("  State remains IDLE")
    print("  [PASS]")

    mgr.shutdown()


# ---------------------------------------------------------------------------
# Test 7: Early-exit speedup
# ---------------------------------------------------------------------------

def test_early_exit_speedup():
    """Verify early_exit=True reduces evaluation time on infeasible trajectories."""
    _header("Test 7: Early-exit speedup")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    # Create an infeasible trajectory (extreme move in very short duration)
    zeros6 = np.zeros(6)
    end_pose = np.array([100.0, 100.0, 100.0, 0.1, 0.1, 0.0])
    traj = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=0.05, t_start=0.0)

    # Full check (no early exit)
    t0 = time.perf_counter()
    result_full = check_feasibility(traj, geom, params, n_samples=50)
    dt_full = time.perf_counter() - t0

    # Early exit check
    t1 = time.perf_counter()
    result_early = check_feasibility(
        traj, geom, params, n_samples=50, early_exit=True)
    dt_early = time.perf_counter() - t1

    assert not result_full.feasible, "Trajectory should be infeasible"
    assert not result_early.feasible, "Early-exit should also reject"

    print(f"  Full check: {dt_full*1000:.1f}ms ({len(result_full.violations)} violations)")
    print(f"  Early exit: {dt_early*1000:.1f}ms ({len(result_early.violations)} violations)")

    speedup = dt_full / dt_early if dt_early > 0 else float('inf')
    print(f"  Speedup: {speedup:.1f}x")

    # Early exit should be at least somewhat faster for an infeasible trajectory
    # (violation occurs early in the sample loop)
    assert dt_early <= dt_full * 1.1, (
        f"Early exit ({dt_early*1000:.1f}ms) should not be slower than "
        f"full check ({dt_full*1000:.1f}ms)")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 8: Torque skip speedup
# ---------------------------------------------------------------------------

def test_torque_skip_speedup():
    """Verify skipping torque computation when no limit is set speeds up check."""
    _header("Test 8: Torque skip speedup")

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    zeros6 = np.zeros(6)
    end_pose = np.array([0.0, 0.0, 30.0, 0.0, 0.0, 0.0])
    traj = create_trajectory(
        start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
        end_pose=end_pose, end_twist=zeros6, end_accel=zeros6,
        duration=1.0, t_start=0.0)

    # With torque limit (forces torque computation)
    t0 = time.perf_counter()
    result_with = check_feasibility(
        traj, geom, params, n_samples=50, torque_limit_Nm=10.0)
    dt_with = time.perf_counter() - t0

    # Without torque limit (skips torque computation)
    t1 = time.perf_counter()
    result_without = check_feasibility(
        traj, geom, params, n_samples=50, torque_limit_Nm=None)
    dt_without = time.perf_counter() - t1

    assert result_with.feasible, "Should be feasible with generous torque limit"
    assert result_without.feasible, "Should be feasible without torque limit"

    print(f"  With torque check:    {dt_with*1000:.1f}ms")
    print(f"  Without torque check: {dt_without*1000:.1f}ms")

    speedup = dt_with / dt_without if dt_without > 0 else float('inf')
    print(f"  Speedup: {speedup:.1f}x")

    # Without torque should be faster (or at least not slower)
    assert dt_without <= dt_with * 1.1, (
        f"No-torque ({dt_without*1000:.1f}ms) should not be slower than "
        f"with-torque ({dt_with*1000:.1f}ms)")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("Async Feasibility Pipeline Verification Tests")
    print("=" * 70)

    t0 = time.time()
    passed = 0
    failed = 0

    tests = [
        test_nonblocking_submit,
        test_async_acceptance,
        test_async_rejection,
        test_supersession,
        test_decel_precompute,
        test_cancel_discards,
        test_early_exit_speedup,
        test_torque_skip_speedup,
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
