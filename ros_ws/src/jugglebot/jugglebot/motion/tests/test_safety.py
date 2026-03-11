"""Safety hardening verification tests.

Tests:
  1. Slew clamp — large step clamped to max_delta per cycle
  2. Slew transparency — normal trajectory doesn't trigger limiter
  3. Sustained slew fault — >SLEW_FAULT_CYCLES → ESTOP
  4. No feedback suppresses commands — _has_motor_fb=False → no publish
  5. Stale feedback suppresses commands — old timestamp → no publish
  6. Motor overspeed fault — feedback velocity > limit → ESTOP
  7. Tracking error fault — error > threshold → ESTOP
  8. Lead-time gate — dynamic target with < MIN_LEAD_TIME_S → rejected

Run:  python -m jugglebot.motion.tests.test_safety

Tests 1-7 require pyzmq/msgpack (Jetson-only); they are SKIPPED on
Windows if those dependencies are missing.  Test 8 uses trajectory.py
which has no IPC dependency and always runs.
"""

from __future__ import annotations

import sys
import time

import numpy as np


def _header(name: str):
    print(f"\n{'='*70}")
    print(f"  {name}")
    print(f"{'='*70}")


# ---------------------------------------------------------------------------
# Lazy import helpers (control_loop requires pyzmq/msgpack via ipc.py)
# ---------------------------------------------------------------------------

def _import_control_loop():
    """Import control loop classes/constants.  Returns None on ImportError."""
    try:
        from jugglebot.motion.control_loop import (
            ControlLoop,
            ControlMode,
            MAX_SLEW_RATE_REV_PER_S,
            SLEW_FAULT_CYCLES,
            MAX_MOTOR_VEL_RPS,
            MAX_TRACKING_ERROR_MM,
        )
        return (ControlLoop, ControlMode, MAX_SLEW_RATE_REV_PER_S,
                SLEW_FAULT_CYCLES, MAX_MOTOR_VEL_RPS, MAX_TRACKING_ERROR_MM)
    except ImportError as e:
        return None


def _make_loop():
    """Create a ControlLoop with mock IPC for testing."""
    from jugglebot.motion.control_loop import ControlLoop
    from jugglebot.motion.geometry import StewartGeometry

    class MockIPC:
        def recv_all(self):
            return []
        def send_telemetry(self, msg):
            self._last_telem = msg
        @property
        def seconds_since_last_recv(self):
            return 0.0
        def close(self):
            pass

    geom = StewartGeometry()
    ipc = MockIPC()
    return ControlLoop(target_rate_hz=500, geom=geom, ipc=ipc)


def _inject_motor_feedback(loop, pos_rev, vel_rps=None, cur_A=None):
    """Inject motor feedback into the control loop."""
    if vel_rps is None:
        vel_rps = np.zeros(6)
    if cur_A is None:
        cur_A = np.zeros(6)
    loop._on_motor_feedback({
        'pos': pos_rev.tolist(),
        'vel': vel_rps.tolist(),
        'cur': cur_A.tolist(),
    })


_SKIP_MSG = "  [SKIP] missing dependency (pyzmq/msgpack)"


# ---------------------------------------------------------------------------
# Test 1: Slew clamp
# ---------------------------------------------------------------------------

def test_slew_clamp():
    """Verify a large position step is clamped to max_delta."""
    _header("Test 1: Slew clamp")

    imports = _import_control_loop()
    if imports is None:
        print(_SKIP_MSG)
        return True
    _, _, MAX_SLEW_RATE_REV_PER_S, _, _, _ = imports

    loop = _make_loop()
    dt = loop.dt_target
    max_delta = MAX_SLEW_RATE_REV_PER_S * dt

    actual_pos = np.full(6, 2.0)
    _inject_motor_feedback(loop, actual_pos)

    # Command a large step to 0.0 (2 rev jump)
    loop._commanded_pos_rev = np.zeros(6)
    ok = loop._slew_limit(dt)

    assert ok, "Slew limiter should allow (clamped) commands to be sent"
    assert loop._slew_limited, "Slew limiter should flag as limited"

    expected = actual_pos - max_delta
    err = np.max(np.abs(loop._commanded_pos_rev - expected))
    assert err < 1e-10, f"Clamped position error: {err:.2e}"

    expected_vel = -np.full(6, MAX_SLEW_RATE_REV_PER_S)
    vel_err = np.max(np.abs(loop._commanded_vel_ff_rps - expected_vel))
    assert vel_err < 1e-6, f"Vel_ff error: {vel_err:.2e}"

    assert np.all(loop._commanded_torque_ff_Nm == 0.0), "Torque_ff should be zero"

    print(f"  max_delta per cycle: {max_delta:.4f} rev")
    print("  step clamped correctly")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 2: Slew transparency
# ---------------------------------------------------------------------------

def test_slew_transparency():
    """Verify the limiter is transparent for small (normal) position changes."""
    _header("Test 2: Slew transparency")

    imports = _import_control_loop()
    if imports is None:
        print(_SKIP_MSG)
        return True

    loop = _make_loop()
    dt = loop.dt_target

    actual_pos = np.full(6, 2.0)
    _inject_motor_feedback(loop, actual_pos)

    small_step = 0.001
    loop._commanded_pos_rev = actual_pos + small_step
    original_vel = np.full(6, 0.5)
    original_torque = np.full(6, 0.1)
    loop._commanded_vel_ff_rps = original_vel.copy()
    loop._commanded_torque_ff_Nm = original_torque.copy()

    ok = loop._slew_limit(dt)

    assert ok, "Should allow commands"
    assert not loop._slew_limited, "Should NOT be flagged as limited"
    assert loop._slew_limit_count == 0, "Count should be zero"
    assert np.allclose(loop._commanded_vel_ff_rps, original_vel), \
        "Vel_ff should be unchanged"
    assert np.allclose(loop._commanded_torque_ff_Nm, original_torque), \
        "Torque_ff should be unchanged"

    print("  Small step passed through unclamped")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 3: Sustained slew fault
# ---------------------------------------------------------------------------

def test_sustained_slew_fault():
    """Verify ESTOP triggers after SLEW_FAULT_CYCLES consecutive clamps."""
    _header("Test 3: Sustained slew fault")

    imports = _import_control_loop()
    if imports is None:
        print(_SKIP_MSG)
        return True
    _, ControlMode, _, SLEW_FAULT_CYCLES, _, _ = imports

    loop = _make_loop()
    loop.mode = ControlMode.ENABLED
    dt = loop.dt_target

    actual_pos = np.full(6, 2.0)
    _inject_motor_feedback(loop, actual_pos)

    for i in range(SLEW_FAULT_CYCLES - 1):
        loop._commanded_pos_rev = np.zeros(6)
        ok = loop._slew_limit(dt)
        assert ok, f"Should still allow at cycle {i}"
        assert loop.mode == ControlMode.ENABLED, f"Should be ENABLED at cycle {i}"

    loop._commanded_pos_rev = np.zeros(6)
    ok = loop._slew_limit(dt)

    assert not ok, "Should suppress commands on fault"
    assert loop.mode == ControlMode.ESTOP, "Should be ESTOP"
    assert loop._fault_state == 'slew_limit_sustained', \
        f"Wrong fault state: {loop._fault_state}"

    print(f"  ESTOP triggered after {SLEW_FAULT_CYCLES} cycles")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 4: No feedback suppresses commands
# ---------------------------------------------------------------------------

def test_no_feedback_suppresses():
    """Verify commands are suppressed when no motor feedback is available."""
    _header("Test 4: No feedback suppresses commands")

    imports = _import_control_loop()
    if imports is None:
        print(_SKIP_MSG)
        return True

    loop = _make_loop()
    loop._commanded_pos_rev = np.full(6, 2.0)
    ok = loop._slew_limit(loop.dt_target)

    assert not ok, "Should suppress when no feedback"

    print("  Commands suppressed with no feedback")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 5: Stale feedback suppresses commands
# ---------------------------------------------------------------------------

def test_stale_feedback_suppresses():
    """Verify commands are suppressed when motor feedback is too old."""
    _header("Test 5: Stale feedback suppresses commands")

    imports = _import_control_loop()
    if imports is None:
        print(_SKIP_MSG)
        return True

    loop = _make_loop()
    _inject_motor_feedback(loop, np.full(6, 2.0))
    loop._motor_fb_timestamp = time.perf_counter() - 0.2  # 200ms old

    loop._commanded_pos_rev = np.full(6, 2.0)
    ok = loop._slew_limit(loop.dt_target)

    assert not ok, "Should suppress when feedback is stale"

    print("  Commands suppressed with stale feedback (200ms)")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 6: Motor overspeed fault
# ---------------------------------------------------------------------------

def test_motor_overspeed():
    """Verify ESTOP on motor overspeed."""
    _header("Test 6: Motor overspeed fault")

    imports = _import_control_loop()
    if imports is None:
        print(_SKIP_MSG)
        return True
    _, ControlMode, _, _, MAX_MOTOR_VEL_RPS, _ = imports

    loop = _make_loop()
    loop.mode = ControlMode.ENABLED

    pos = np.full(6, 2.0)
    vel = np.zeros(6)
    vel[3] = MAX_MOTOR_VEL_RPS + 1.0
    _inject_motor_feedback(loop, pos, vel)

    loop._commanded_pos_rev = pos.copy()
    ok = loop._slew_limit(loop.dt_target)

    assert not ok, "Should suppress on overspeed"
    assert loop.mode == ControlMode.ESTOP, "Should be ESTOP"
    assert 'motor_overspeed' in loop._fault_state, \
        f"Wrong fault: {loop._fault_state}"

    print(f"  ESTOP triggered at {vel[3]:.1f} rev/s (limit {MAX_MOTOR_VEL_RPS:.1f})")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 7: Tracking error fault
# ---------------------------------------------------------------------------

def test_tracking_error():
    """Verify ESTOP on excessive tracking error."""
    _header("Test 7: Tracking error fault")

    imports = _import_control_loop()
    if imports is None:
        print(_SKIP_MSG)
        return True
    _, ControlMode, _, _, _, MAX_TRACKING_ERROR_MM = imports

    loop = _make_loop()
    loop.mode = ControlMode.ENABLED

    pos = np.full(6, 2.0)
    _inject_motor_feedback(loop, pos)

    loop._tracking_error_mm = np.zeros(6)
    loop._tracking_error_mm[1] = MAX_TRACKING_ERROR_MM + 1.0
    loop._commanded_pos_rev = pos.copy()

    ok = loop._slew_limit(loop.dt_target)

    assert not ok, "Should suppress on tracking error"
    assert loop.mode == ControlMode.ESTOP, "Should be ESTOP"
    assert 'tracking_error' in loop._fault_state, \
        f"Wrong fault: {loop._fault_state}"

    print(f"  ESTOP triggered at {loop._tracking_error_mm[1]:.1f}mm "
          f"(limit {MAX_TRACKING_ERROR_MM:.1f}mm)")
    print("  [PASS]")


# ---------------------------------------------------------------------------
# Test 8: Lead-time gate
# ---------------------------------------------------------------------------

def test_lead_time_gate():
    """Verify dynamic targets with < MIN_LEAD_TIME_S are rejected."""
    _header("Test 8: Lead-time gate")

    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.dynamics import DynamicsParams
    from jugglebot.motion.trajectory import (
        TrajectoryManager,
        MIN_LEAD_TIME_S,
    )

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    mgr = TrajectoryManager(geom, params)
    mgr.set_hold_pose(np.zeros(6))

    t_now = time.perf_counter()
    target_pos = np.array([0.0, 0.0, 10.0])
    target_quat = np.array([1.0, 0.0, 0.0, 0.0])
    target_vel = np.zeros(3)

    # Sync path — too soon
    arrival_too_soon = t_now + MIN_LEAD_TIME_S * 0.5
    accepted = mgr.submit_dynamic_target(
        target_pos, target_quat, target_vel, arrival_too_soon, t_now)
    assert not accepted, "Should reject target with insufficient lead time (sync)"

    # Async path — too soon
    t_now2 = time.perf_counter()
    arrival_too_soon2 = t_now2 + MIN_LEAD_TIME_S * 0.5
    queued = mgr.request_dynamic_target(
        target_pos, target_quat, target_vel, arrival_too_soon2, t_now2)
    assert not queued, "Should reject target with insufficient lead time (async)"

    print(f"  MIN_LEAD_TIME_S = {MIN_LEAD_TIME_S}s")
    print("  Too-soon target rejected (sync and async)")
    print("  [PASS]")

    mgr.shutdown()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    tests = [
        ("Slew clamp", test_slew_clamp),
        ("Slew transparency", test_slew_transparency),
        ("Sustained slew fault", test_sustained_slew_fault),
        ("No feedback suppresses", test_no_feedback_suppresses),
        ("Stale feedback suppresses", test_stale_feedback_suppresses),
        ("Motor overspeed", test_motor_overspeed),
        ("Tracking error", test_tracking_error),
        ("Lead-time gate", test_lead_time_gate),
    ]

    passed = 0
    failed = 0
    skipped = 0
    for name, fn in tests:
        try:
            result = fn()
            if result is True:
                # Explicit True return = skip (convention from test_control_loop)
                skipped += 1
            else:
                passed += 1
        except Exception as e:
            print(f"\n  [FAIL] {name}: {e}")
            failed += 1

    print(f"\n{'='*70}")
    print(f"  Results: {passed} passed, {failed} failed, "
          f"{skipped} skipped, {len(tests)} total")
    print(f"{'='*70}")

    return failed == 0


if __name__ == '__main__':
    ok = main()
    sys.exit(0 if ok else 1)
