"""Motor guard verification tests.

Tests the MotorGuard class: interpolation, safety checks, IPC, and
unit conversions.  Ports and replaces relevant tests from:
  - test_control_loop.py (loop timing, IPC latency, force conversion)
  - test_mpc_passthrough.py (command flow, staleness, workspace, disable)
  - test_safety.py (overspeed, feedback staleness, max deviation)

Run:  python -m jugglebot.motion.tests.test_motor_guard
"""

from __future__ import annotations

import sys
import threading
import time

import numpy as np

from jugglebot.motion.conversions import (
    extensions_mm_to_revs,
    leg_forces_to_motor_torques,
    leg_velocities_to_motor_velocities,
    motor_torques_to_leg_forces,
    revs_to_extensions_mm,
)
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ipc import (
    TOPIC_MODE,
    TOPIC_MPC_CMD,
    TOPIC_MOTOR_FB,
    make_mode_command,
    make_mpc_command,
)
from jugglebot.motion.motor_guard import (
    EXTRAP_DECAY_DT_S,
    GuardMode,
    JERK_EMA_ALPHA,
    MAX_DEVIATION_REV,
    MAX_EXTRAP_DT_S,
    MAX_MOTOR_VEL_RPS,
    MPC_CMD_STALENESS_S,
    MotorGuard,
)
from jugglebot.motion.workspace import WorkspaceStatus


# ---------------------------------------------------------------------------
# Mock IPC: injects messages and captures telemetry
# ---------------------------------------------------------------------------

class MockIPC:
    """IPC mock that feeds queued messages and captures published telemetry."""

    def __init__(self):
        self._queue: list[tuple[bytes, dict]] = []
        self.sent_telemetry: list[dict] = []
        self._last_recv = time.monotonic()

    def inject(self, topic: bytes, msg: dict) -> None:
        """Queue a message to be returned by recv_all()."""
        self._queue.append((topic, msg))

    def recv_all(self) -> list[tuple[bytes, dict]]:
        msgs = list(self._queue)
        self._queue.clear()
        if msgs:
            self._last_recv = time.monotonic()
        return msgs

    def send_telemetry(self, msg: dict) -> None:
        self.sent_telemetry.append(msg)

    @property
    def seconds_since_last_recv(self) -> float:
        return time.monotonic() - self._last_recv

    def close(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

def _make_guard() -> tuple[MotorGuard, MockIPC]:
    """Create a MotorGuard with a MockIPC for testing."""
    ipc = MockIPC()
    geom = StewartGeometry()
    guard = MotorGuard(rate_hz=500, geom=geom, ipc=ipc)
    return guard, ipc


def _enable(guard: MotorGuard, ipc: MockIPC, source: str = 'MPC') -> None:
    """Enable the motor guard."""
    ipc.inject(TOPIC_MODE, make_mode_command('enable', source=source))
    guard._process_ipc()


def _inject_mpc_cmd(ipc: MockIPC, ext_mm=None, pose_6dof=None,
                     vel_mm_s=None, acc_mm_s2=None, torque_Nm=None,
                     seq=0, motor_rev=None) -> None:
    """Inject an MPC command message."""
    if ext_mm is None:
        ext_mm = [140.0] * 6  # mid-stroke (STOW-relative)
    if pose_6dof is None:
        pose_6dof = [0.0, 0.0, 50.0, 0.0, 0.0, 0.0]
    msg = make_mpc_command(
        ext_mm=ext_mm, pose_6dof=pose_6dof,
        motor_rev=motor_rev, vel_mm_s=vel_mm_s,
        acc_mm_s2=acc_mm_s2,
        torque_Nm=torque_Nm, seq=seq)
    ipc.inject(TOPIC_MPC_CMD, msg)


def _inject_motor_feedback(ipc: MockIPC, pos_rev, vel_rps=None,
                            cur_A=None) -> None:
    """Inject motor feedback."""
    if vel_rps is None:
        vel_rps = [0.0] * 6
    if cur_A is None:
        cur_A = [0.0] * 6
    ipc.inject(TOPIC_MOTOR_FB, {
        'type': 'motor_feedback',
        'pos': list(pos_rev),
        'vel': list(vel_rps),
        'cur': list(cur_A),
    })


def _provide_matching_feedback(guard: MotorGuard, ipc: MockIPC,
                                ext_mm=None) -> None:
    """Inject motor feedback that matches a given extension (or default)."""
    if ext_mm is None:
        ext_mm = [140.0] * 6
    pos_rev = extensions_mm_to_revs(np.array(ext_mm), guard.geom).tolist()
    _inject_motor_feedback(ipc, pos_rev)


# ---------------------------------------------------------------------------
# Test 1: Loop timing
# ---------------------------------------------------------------------------

def test_loop_timing():
    """Run the motor guard for a short duration and check jitter.

    Exit gate: 99th-percentile jitter < 2x nominal period.
    """
    ipc = MockIPC()
    geom = StewartGeometry()
    guard = MotorGuard(rate_hz=500, geom=geom, ipc=ipc)

    rate_hz = 500
    duration_s = 10.0
    dt_nominal = 1.0 / rate_hz

    def run_loop():
        guard.run()

    t = threading.Thread(target=run_loop, daemon=True)
    t.start()

    time.sleep(duration_s)
    guard.stop()
    t.join(timeout=2.0)

    jitter_p99 = guard.stats.jitter_99
    mean_dt = guard.stats.mean
    n_cycles = guard.stats.count

    gate_threshold = 2.0 * dt_nominal
    passed = jitter_p99 < gate_threshold

    print(f"  [{'PASS' if passed else 'FAIL'}] loop timing at {rate_hz} Hz "
          f"({n_cycles} cycles in {duration_s:.0f}s)")
    print(f"         mean dt:     {mean_dt*1000:.3f} ms  "
          f"(nominal: {dt_nominal*1000:.3f} ms)")
    print(f"         p99 jitter:  {jitter_p99*1000:.3f} ms  "
          f"(gate: < {gate_threshold*1000:.3f} ms)")
    print(f"         max dt:      {guard.stats.max*1000:.3f} ms")
    print(f"         std:         {guard.stats.std*1000:.3f} ms")

    return passed


# ---------------------------------------------------------------------------
# Test 2: IPC latency
# ---------------------------------------------------------------------------

def test_ipc_latency():
    """Measure ZeroMQ round-trip latency.

    Sends a motor feedback message from BridgeIPC → MotorGuardIPC → back.
    """
    try:
        import zmq
        import msgpack
    except ImportError as e:
        print(f"  [SKIP] IPC latency — missing dependency: {e}")
        return True

    from jugglebot.motion.ipc import (
        BridgeIPC,
        MotorGuardIPC,
        make_motor_feedback,
    )

    bridge = BridgeIPC(
        command_addr='tcp://127.0.0.1:15555',
        telemetry_addr='tcp://127.0.0.1:15556',
    )
    guard_ipc = MotorGuardIPC(
        command_addr='tcp://127.0.0.1:15555',
        telemetry_addr='tcp://127.0.0.1:15556',
    )

    time.sleep(0.2)

    latencies = []
    n_messages = 100

    for _ in range(n_messages):
        t_send = time.perf_counter()
        fb = make_motor_feedback(
            positions=[1.0] * 6,
            velocities=[0.0] * 6,
            currents=[0.0] * 6,
        )
        bridge.send_motor_feedback(fb)

        time.sleep(0.0005)

        msgs = guard_ipc.recv_all()
        t_recv = time.perf_counter()

        if msgs:
            latencies.append(t_recv - t_send)

    guard_ipc.close()
    bridge.close()

    if not latencies:
        print("  [FAIL] IPC latency — no messages received")
        return False

    lat = np.array(latencies) * 1000  # ms
    passed = np.median(lat) < 2.0

    print(f"  [{'PASS' if passed else 'FAIL'}] IPC latency  "
          f"({len(latencies)}/{n_messages} messages received)")
    print(f"         median: {np.median(lat):.3f} ms  "
          f"mean: {np.mean(lat):.3f} ms  "
          f"p99: {np.percentile(lat, 99):.3f} ms")

    return passed


# ---------------------------------------------------------------------------
# Test 3: Force conversion (pure math, no IPC)
# ---------------------------------------------------------------------------

def test_force_conversion():
    """Verify force↔torque and extension↔revs conversions are self-consistent."""
    geom = StewartGeometry()

    # Round-trip: forces → torques → forces
    test_forces = np.array([100.0, 150.0, 120.0, 130.0, 110.0, 140.0])
    torques = leg_forces_to_motor_torques(test_forces, geom)
    recovered_forces = motor_torques_to_leg_forces(torques, geom)
    roundtrip_err = np.max(np.abs(test_forces - recovered_forces))

    # Round-trip: extensions → revs → extensions
    test_ext = np.array([50.0, 100.0, 150.0, 200.0, 250.0, 280.0])
    revs = extensions_mm_to_revs(test_ext, geom)
    recovered_ext = revs_to_extensions_mm(revs, geom)
    ext_roundtrip_err = np.max(np.abs(test_ext - recovered_ext))

    # Sanity: spool radii should be 10-15 mm range
    spool_radii = geom.spool_radius_mm
    radii_ok = np.all(spool_radii > 5.0) and np.all(spool_radii < 20.0)

    # Sanity: 100N leg force → ~1 Nm motor torque (given ~11mm spool)
    f100 = np.full(6, 100.0)
    t100 = leg_forces_to_motor_torques(f100, geom)
    torque_magnitude_ok = np.all(t100 > 0.5) and np.all(t100 < 2.0)

    passed = (roundtrip_err < 1e-10 and ext_roundtrip_err < 1e-10
              and radii_ok and torque_magnitude_ok)

    print(f"  [{'PASS' if passed else 'FAIL'}] force conversion")
    print(f"         force roundtrip error: {roundtrip_err:.2e} N")
    print(f"         extension roundtrip error: {ext_roundtrip_err:.2e} mm")
    print(f"         spool radii: {spool_radii.round(2)} mm  "
          f"(range ok: {radii_ok})")
    print(f"         100N -> torque: {t100.round(3)} Nm  "
          f"(magnitude ok: {torque_magnitude_ok})")

    return passed


# ---------------------------------------------------------------------------
# Test 4: MPC enable
# ---------------------------------------------------------------------------

def test_mpc_enable():
    """Enabling with source='MPC' activates the motor guard."""
    guard, ipc = _make_guard()
    assert guard.mode == GuardMode.DISABLED

    _enable(guard, ipc)

    assert guard.mode == GuardMode.ENABLED
    assert not guard._has_mpc_cmd
    print("  [PASS] test_mpc_enable")
    return True


# ---------------------------------------------------------------------------
# Test 5: MPC command flow
# ---------------------------------------------------------------------------

def test_mpc_command_flow():
    """MPC commands are converted to motor positions correctly."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [100.0, 110.0, 120.0, 130.0, 140.0, 150.0]
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)
    guard._process_ipc()

    # Run one interpolation cycle
    guard._interpolate_and_send(guard.dt_target)

    expected_rev = extensions_mm_to_revs(np.array(ext_mm), guard.geom)
    # Interpolated position = base_pos + vel * dt_since_cmd
    # For the first command, vel_ff is zero, so pos ≈ base_pos
    np.testing.assert_allclose(
        guard._mpc_base_pos_rev, expected_rev, atol=1e-6,
        err_msg="MPC extensions not correctly converted to motor revs")
    print("  [PASS] test_mpc_command_flow")
    return True


# ---------------------------------------------------------------------------
# Test 6: MPC staleness E-stop
# ---------------------------------------------------------------------------

def test_mpc_staleness_estop():
    """MPC command staleness triggers E-stop."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    _provide_matching_feedback(guard, ipc)
    _inject_mpc_cmd(ipc)
    guard._process_ipc()
    guard._interpolate_and_send(guard.dt_target)

    assert guard.mode == GuardMode.ENABLED

    # Backdate timestamp to simulate staleness
    guard._mpc_cmd_timestamp = time.perf_counter() - MPC_CMD_STALENESS_S - 0.01

    guard._check_safety()

    assert guard.mode == GuardMode.ESTOP
    assert 'mpc_cmd_stale' in (guard._fault_state or '')
    print("  [PASS] test_mpc_staleness_estop")
    return True


# ---------------------------------------------------------------------------
# Test 7: Disable clears state
# ---------------------------------------------------------------------------

def test_disable_clears_state():
    """Disable clears MPC state and returns to DISABLED."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)
    _provide_matching_feedback(guard, ipc)
    _inject_mpc_cmd(ipc)
    guard._process_ipc()

    assert guard.mode == GuardMode.ENABLED
    assert guard._has_mpc_cmd

    ipc.inject(TOPIC_MODE, make_mode_command('disable'))
    guard._process_ipc()

    assert guard.mode == GuardMode.DISABLED
    assert not guard._has_mpc_cmd
    print("  [PASS] test_disable_clears_state")
    return True


# ---------------------------------------------------------------------------
# Test 8: Workspace hard limit E-stop
# ---------------------------------------------------------------------------

def test_workspace_hard_limit_estops():
    """Workspace hard limit triggers E-stop."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    # Command extensions well beyond stroke (hard limit is ~275mm)
    ext_mm = [290.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, pose_6dof=[0, 0, 200, 0, 0, 0])
    guard._process_ipc()

    assert guard.mode == GuardMode.ESTOP
    assert 'workspace_hard_limit' in (guard._fault_state or '')
    print("  [PASS] test_workspace_hard_limit_estops")
    return True


# ---------------------------------------------------------------------------
# Test 9: Torque passthrough
# ---------------------------------------------------------------------------

def test_torque_passthrough():
    """torque_Nm is passed through to commanded torques correctly."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    torque_Nm = [0.1, -0.1, 0.05, -0.05, 0.02, -0.02]

    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, torque_Nm=torque_Nm)
    guard._process_ipc()

    np.testing.assert_allclose(
        guard._mpc_base_torque_Nm, torque_Nm, atol=1e-10)
    print("  [PASS] test_torque_passthrough")
    return True


# ---------------------------------------------------------------------------
# Test 10: Zeros when no feedforward
# ---------------------------------------------------------------------------

def test_zeros_when_no_feedforward():
    """When torque_Nm is omitted, zeros are used."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)  # no torque
    guard._process_ipc()

    np.testing.assert_allclose(guard._mpc_base_torque_Nm, 0.0, atol=1e-10)
    print("  [PASS] test_zeros_when_no_feedforward")
    return True


# ---------------------------------------------------------------------------
# Test 11: No feedback suppresses commands
# ---------------------------------------------------------------------------

def test_no_feedback_suppresses():
    """Commands are suppressed when no motor feedback is available."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    _inject_mpc_cmd(ipc)
    guard._process_ipc()

    ipc.sent_telemetry.clear()
    guard._interpolate_and_send(guard.dt_target)

    # No telemetry should be sent (feedback gate)
    assert len(ipc.sent_telemetry) == 0
    print("  [PASS] test_no_feedback_suppresses")
    return True


# ---------------------------------------------------------------------------
# Test 12: Stale feedback suppresses commands
# ---------------------------------------------------------------------------

def test_stale_feedback_suppresses():
    """Commands are suppressed when motor feedback is too old."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    _provide_matching_feedback(guard, ipc)
    _inject_mpc_cmd(ipc)
    guard._process_ipc()

    # Backdate feedback timestamp to simulate staleness (200ms old)
    guard._motor_fb_timestamp = time.perf_counter() - 0.2

    ipc.sent_telemetry.clear()
    guard._interpolate_and_send(guard.dt_target)

    # No telemetry should be sent (stale feedback gate)
    assert len(ipc.sent_telemetry) == 0
    print("  [PASS] test_stale_feedback_suppresses")
    return True


# ---------------------------------------------------------------------------
# Test 13: Motor overspeed E-stop
# ---------------------------------------------------------------------------

def test_motor_overspeed():
    """Motor overspeed triggers E-stop."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    # Provide feedback with one motor exceeding velocity limit
    pos_rev = [2.0] * 6
    vel_rps = [0.0] * 6
    vel_rps[3] = MAX_MOTOR_VEL_RPS + 1.0
    _inject_motor_feedback(ipc, pos_rev, vel_rps)
    _inject_mpc_cmd(ipc)
    guard._process_ipc()

    guard._check_safety()

    assert guard.mode == GuardMode.ESTOP
    assert 'motor_overspeed' in (guard._fault_state or '')
    print(f"  [PASS] test_motor_overspeed  "
          f"(triggered at {vel_rps[3]:.1f} rev/s, limit {MAX_MOTOR_VEL_RPS:.1f})")
    return True


# ---------------------------------------------------------------------------
# Test 14: Max deviation E-stop
# ---------------------------------------------------------------------------

def test_max_deviation_estop():
    """MPC command far from motor feedback triggers E-stop."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    # Motor feedback at 2.0 rev
    _inject_motor_feedback(ipc, pos_rev=[2.0] * 6)
    guard._process_ipc()

    # MPC command at position that's MAX_DEVIATION_REV + 0.1 away
    ext_target = revs_to_extensions_mm(
        np.full(6, 2.0 + MAX_DEVIATION_REV + 0.1), guard.geom)
    motor_rev_target = (2.0 + MAX_DEVIATION_REV + 0.1)
    _inject_mpc_cmd(
        ipc, ext_mm=ext_target.tolist(),
        motor_rev=[motor_rev_target] * 6,
        pose_6dof=[0, 0, 50, 0, 0, 0])
    guard._process_ipc()

    assert guard.mode == GuardMode.ESTOP
    assert 'max_deviation' in (guard._fault_state or '')
    print(f"  [PASS] test_max_deviation_estop  "
          f"(limit {MAX_DEVIATION_REV} rev)")
    return True


# ---------------------------------------------------------------------------
# Test 15: NaN rejection
# ---------------------------------------------------------------------------

def test_nan_rejection():
    """MPC command with NaN is rejected (not E-stop, just dropped)."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    _provide_matching_feedback(guard, ipc)
    guard._process_ipc()

    ext_nan = [140.0, float('nan'), 140.0, 140.0, 140.0, 140.0]
    _inject_mpc_cmd(ipc, ext_mm=ext_nan)
    guard._process_ipc()

    # Command should be rejected — _has_mpc_cmd remains False
    assert not guard._has_mpc_cmd
    assert guard.mode == GuardMode.ENABLED  # no E-stop, just rejection
    print("  [PASS] test_nan_rejection")
    return True


# ---------------------------------------------------------------------------
# Test 16: NaN feedback rejection
# ---------------------------------------------------------------------------

def test_nan_feedback_rejection():
    """Motor feedback with NaN/Inf is rejected, preserving safety checks.

    Without validation, NaN in feedback would bypass overspeed and max-
    deviation checks because numpy comparisons with NaN return False.
    """
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    # Provide valid feedback first, then corrupt it
    _provide_matching_feedback(guard, ipc)
    _inject_mpc_cmd(ipc)
    guard._process_ipc()
    assert guard._has_motor_fb

    # NaN in position — should be rejected, feedback unchanged
    old_pos = guard._motor_fb_pos_rev.copy()
    _inject_motor_feedback(ipc, pos_rev=[1.0, float('nan'), 1.0, 1.0, 1.0, 1.0])
    guard._process_ipc()
    np.testing.assert_array_equal(guard._motor_fb_pos_rev, old_pos,
                                  err_msg="NaN pos feedback was accepted")

    # Inf in velocity — should be rejected
    _inject_motor_feedback(ipc, pos_rev=[1.0]*6,
                           vel_rps=[0.0, 0.0, float('inf'), 0.0, 0.0, 0.0])
    guard._process_ipc()
    np.testing.assert_array_equal(guard._motor_fb_pos_rev, old_pos,
                                  err_msg="Inf vel feedback was accepted")

    # NaN in current — should be rejected
    _inject_motor_feedback(ipc, pos_rev=[1.0]*6, vel_rps=[0.0]*6,
                           cur_A=[0.0, 0.0, 0.0, float('nan'), 0.0, 0.0])
    guard._process_ipc()
    np.testing.assert_array_equal(guard._motor_fb_pos_rev, old_pos,
                                  err_msg="NaN cur feedback was accepted")

    # Guard should still be enabled (not E-stopped from corrupt data)
    assert guard.mode == GuardMode.ENABLED

    # Valid feedback should still be accepted
    _inject_motor_feedback(ipc, pos_rev=[2.0]*6, vel_rps=[0.1]*6, cur_A=[0.5]*6)
    guard._process_ipc()
    np.testing.assert_allclose(guard._motor_fb_pos_rev, [2.0]*6,
                               err_msg="Valid feedback rejected after NaN")

    print("  [PASS] test_nan_feedback_rejection")
    return True


# ---------------------------------------------------------------------------
# Test 17: Vel_ff finite-difference
# ---------------------------------------------------------------------------

def test_vel_ff_finite_difference():
    """Motor guard computes vel_ff from consecutive MPC extension commands."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm_1 = [140.0] * 6
    ext_mm_2 = [145.0] * 6  # +5mm on all legs

    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm_1)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm_1)
    guard._process_ipc()

    # Small delay to get a measurable dt
    time.sleep(0.02)

    _inject_mpc_cmd(ipc, ext_mm=ext_mm_2, seq=1)
    guard._process_ipc()

    # vel_ff should be positive (extensions increasing)
    assert np.all(guard._mpc_cmd_vel_mm_s > 0), \
        f"Expected positive vel, got {guard._mpc_cmd_vel_mm_s}"

    # vel_ff magnitude: ~5mm / 0.02s = ~250 mm/s (rough)
    expected_vel = 5.0 / 0.02  # approximate
    np.testing.assert_allclose(
        guard._mpc_cmd_vel_mm_s, expected_vel, rtol=0.5,
        err_msg="vel_ff finite-difference is too far off")

    # Verify it's converted to motor velocity (rps)
    expected_rps = leg_velocities_to_motor_velocities(
        guard._mpc_cmd_vel_mm_s, guard.geom)
    np.testing.assert_allclose(
        guard._mpc_base_vel_rps, expected_rps, atol=1e-6)

    print("  [PASS] test_vel_ff_finite_difference")
    return True


# ---------------------------------------------------------------------------
# Test 16b: Sender-provided velocity preferred over finite-difference
# ---------------------------------------------------------------------------

def test_vel_ff_sender_preferred():
    """Motor guard uses sender-provided vel_mm_s instead of finite-difference."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm_1 = [140.0] * 6
    ext_mm_2 = [145.0] * 6  # +5mm

    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm_1)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm_1)
    guard._process_ipc()

    time.sleep(0.02)

    # Sender provides an explicit velocity that differs from what
    # finite-difference would compute (~250 mm/s).
    sender_vel = [100.0] * 6
    _inject_mpc_cmd(ipc, ext_mm=ext_mm_2, vel_mm_s=sender_vel, seq=1)
    guard._process_ipc()

    # Guard should use the sender value, not the finite-difference
    np.testing.assert_allclose(
        guard._mpc_cmd_vel_mm_s, sender_vel, atol=1e-9,
        err_msg="Guard should prefer sender-provided velocity")

    print("  [PASS] test_vel_ff_sender_preferred")
    return True


# ---------------------------------------------------------------------------
# Test 16c: NaN sender velocity falls back to finite-difference
# ---------------------------------------------------------------------------

def test_vel_ff_nan_sender_fallback():
    """NaN in sender-provided vel_mm_s triggers finite-difference fallback."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm_1 = [140.0] * 6
    ext_mm_2 = [145.0] * 6

    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm_1)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm_1)
    guard._process_ipc()

    time.sleep(0.02)

    # Sender provides NaN velocity — guard should fall back to finite-diff
    nan_vel = [float('nan')] * 6
    _inject_mpc_cmd(ipc, ext_mm=ext_mm_2, vel_mm_s=nan_vel, seq=1)
    guard._process_ipc()

    # Should NOT be NaN — finite-difference fallback should produce ~250 mm/s
    assert np.all(np.isfinite(guard._mpc_cmd_vel_mm_s)), \
        f"Expected finite vel, got {guard._mpc_cmd_vel_mm_s}"
    assert np.all(guard._mpc_cmd_vel_mm_s > 0), \
        f"Expected positive vel from finite-diff fallback, got {guard._mpc_cmd_vel_mm_s}"

    print("  [PASS] test_vel_ff_nan_sender_fallback")
    return True


# ---------------------------------------------------------------------------
# Test 17: IPC heartbeat E-stop
# ---------------------------------------------------------------------------

def test_ipc_heartbeat_estop():
    """IPC heartbeat loss triggers E-stop."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    _provide_matching_feedback(guard, ipc)
    _inject_mpc_cmd(ipc)
    guard._process_ipc()

    # Simulate heartbeat loss by backdating the IPC timestamp
    ipc._last_recv = time.monotonic() - 1.0  # 1 second ago

    guard._check_safety()

    assert guard.mode == GuardMode.ESTOP
    assert 'ipc_heartbeat' in (guard._fault_state or '')
    print("  [PASS] test_ipc_heartbeat_estop")
    return True


# ---------------------------------------------------------------------------
# Test 18: Interpolation output
# ---------------------------------------------------------------------------

def test_interpolation_output():
    """Telemetry output contains interpolated position and vel_ff."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    torque_Nm = [0.05] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, torque_Nm=torque_Nm)
    guard._process_ipc()

    ipc.sent_telemetry.clear()
    guard._interpolate_and_send(guard.dt_target)

    assert len(ipc.sent_telemetry) == 1
    telem = ipc.sent_telemetry[0]

    # Telemetry should have leg_pos, leg_vel, leg_torques
    assert telem.get('leg_pos') is not None
    assert telem.get('leg_vel') is not None
    assert telem.get('leg_torques') is not None

    # Torques should match what was commanded
    np.testing.assert_allclose(telem['leg_torques'], torque_Nm, atol=1e-10)

    print("  [PASS] test_interpolation_output")
    return True


# ---------------------------------------------------------------------------
# Test 20: Normal interpolation unchanged within MAX_EXTRAP_DT_S
# ---------------------------------------------------------------------------

def test_normal_interpolation_unchanged():
    """Within MAX_EXTRAP_DT_S, interpolation is plain linear extrapolation."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    vel_mm_s = [500.0] * 6  # 500 mm/s on all legs
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)

    # Two commands to establish velocity via finite-difference
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)
    guard._process_ipc()
    time.sleep(0.02)
    ext_mm_2 = [150.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm_2)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm_2, seq=1)
    guard._process_ipc()

    # Backdate by 20ms (well within 40ms normal window)
    dt_test = 0.020
    guard._mpc_base_timestamp = time.perf_counter() - dt_test

    guard._interpolate_and_send(guard.dt_target)

    # Expected: base_pos + vel * dt (plain linear)
    expected = guard._mpc_base_pos_rev + guard._mpc_base_vel_rps * dt_test
    np.testing.assert_allclose(
        guard._commanded_pos_rev, expected, atol=1e-4,
        err_msg="Normal interpolation should be plain linear within window")

    # vel_ff should equal base velocity (no decay)
    np.testing.assert_allclose(
        guard._commanded_vel_ff_rps, guard._mpc_base_vel_rps, atol=1e-10,
        err_msg="vel_ff should be unmodified within normal window")

    print("  [PASS] test_normal_interpolation_unchanged")
    return True


# ---------------------------------------------------------------------------
# Test 21: Velocity decays during decay window
# ---------------------------------------------------------------------------

def test_extrapolation_vel_decays():
    """After MAX_EXTRAP_DT_S, vel_ff decays linearly toward zero."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)
    guard._process_ipc()

    # Set a known velocity
    test_vel = np.full(6, 5.0)  # 5 rev/s
    guard._mpc_base_vel_rps = test_vel.copy()

    # Backdate to midpoint of decay window: MAX_EXTRAP_DT_S + 0.5 * EXTRAP_DECAY_DT_S
    dt_test = MAX_EXTRAP_DT_S + 0.5 * EXTRAP_DECAY_DT_S
    guard._mpc_base_timestamp = time.perf_counter() - dt_test

    # Simulate motors having tracked during the backdated interval so the
    # tracking clamp doesn't interfere with the extrapolation math under test.
    guard._motor_fb_pos_rev = guard._mpc_base_pos_rev + test_vel * dt_test

    guard._interpolate_and_send(guard.dt_target)

    # At midpoint, decay_frac = 0.5 → vel_ff should be ~50% of original
    expected_vel = test_vel * 0.5
    np.testing.assert_allclose(
        guard._commanded_vel_ff_rps, expected_vel, atol=0.05,
        err_msg="vel_ff should be 50% at midpoint of decay window")

    print("  [PASS] test_extrapolation_vel_decays")
    return True


# ---------------------------------------------------------------------------
# Test 22: Position holds after full decay
# ---------------------------------------------------------------------------

def test_extrapolation_hold_after_decay():
    """After full decay, position is held and vel_ff is zero."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)
    guard._process_ipc()

    base_pos = guard._mpc_base_pos_rev.copy()
    test_vel = np.full(6, 5.0)  # 5 rev/s
    guard._mpc_base_vel_rps = test_vel.copy()

    # Backdate well past full decay: 150ms (> 40 + 60 = 100ms)
    dt_test = 0.150
    guard._mpc_base_timestamp = time.perf_counter() - dt_test

    # Simulate motors having tracked to the expected coast-down endpoint so the
    # tracking clamp doesn't interfere with the extrapolation math under test.
    coast_endpoint = (base_pos + test_vel * MAX_EXTRAP_DT_S
                      + test_vel * EXTRAP_DECAY_DT_S * 0.5)
    guard._motor_fb_pos_rev = coast_endpoint.copy()

    guard._interpolate_and_send(guard.dt_target)

    # vel_ff should be zero
    np.testing.assert_allclose(
        guard._commanded_vel_ff_rps, 0.0, atol=1e-10,
        err_msg="vel_ff should be zero after full decay")

    # Position should be base + vel*MAX_EXTRAP + vel*DECAY/2 (coast-down endpoint)
    expected_pos = (base_pos
                    + test_vel * MAX_EXTRAP_DT_S
                    + test_vel * EXTRAP_DECAY_DT_S * 0.5)
    np.testing.assert_allclose(
        guard._commanded_pos_rev, expected_pos, atol=1e-6,
        err_msg="Position should be held at coast-down endpoint")

    # A second call 10ms later should give the same position
    guard._mpc_base_timestamp = time.perf_counter() - 0.160
    guard._motor_fb_pos_rev = coast_endpoint.copy()
    guard._interpolate_and_send(guard.dt_target)
    np.testing.assert_allclose(
        guard._commanded_pos_rev, expected_pos, atol=1e-6,
        err_msg="Position should remain constant after full decay")

    print("  [PASS] test_extrapolation_hold_after_decay")
    return True


# ---------------------------------------------------------------------------
# Test 23: Extrapolation is bounded vs unbounded linear
# ---------------------------------------------------------------------------

def test_extrapolation_bounded():
    """After decay, position advances less than unbounded linear would."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)
    guard._process_ipc()

    base_pos = guard._mpc_base_pos_rev.copy()
    test_vel = np.full(6, 9.5)  # max velocity
    guard._mpc_base_vel_rps = test_vel.copy()

    # Backdate by 150ms (just before staleness E-stop at 200ms)
    dt_test = 0.150
    guard._mpc_base_timestamp = time.perf_counter() - dt_test

    # Simulate motors having tracked to the expected coast-down endpoint so the
    # tracking clamp doesn't interfere with the extrapolation math under test.
    coast_endpoint = (base_pos + test_vel * MAX_EXTRAP_DT_S
                      + test_vel * EXTRAP_DECAY_DT_S * 0.5)
    guard._motor_fb_pos_rev = coast_endpoint.copy()

    guard._interpolate_and_send(guard.dt_target)

    # Unbounded would be: base + 9.5 * 0.150 = base + 1.425 rev
    unbounded_pos = base_pos + test_vel * dt_test
    # Actual should be: base + 9.5*0.04 + 9.5*0.06*0.5 = base + 0.665 rev
    actual_advance = guard._commanded_pos_rev - base_pos
    unbounded_advance = unbounded_pos - base_pos

    # Bounded advance should be significantly less than unbounded
    assert np.all(actual_advance < unbounded_advance * 0.6), \
        (f"Bounded advance {actual_advance[0]:.4f} should be much less than "
         f"unbounded {unbounded_advance[0]:.4f}")

    # Verify the exact expected value
    expected_advance = test_vel * MAX_EXTRAP_DT_S + test_vel * EXTRAP_DECAY_DT_S * 0.5
    np.testing.assert_allclose(
        actual_advance, expected_advance, atol=1e-6,
        err_msg="Bounded extrapolation distance is wrong")

    print(f"  [PASS] test_extrapolation_bounded  "
          f"(bounded={actual_advance[0]:.3f} vs unbounded={unbounded_advance[0]:.3f} rev)")
    return True


# ---------------------------------------------------------------------------
# Test 24: Stroke clamp prevents out-of-bounds
# ---------------------------------------------------------------------------

def test_stroke_clamp():
    """Pathological velocity + dt is clamped to stroke hard limits."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)
    guard._process_ipc()

    # Set absurdly high velocity to force position past stroke limits
    guard._mpc_base_vel_rps = np.full(6, 100.0)  # way beyond physical limits

    # Backdate just within the normal window so full extrapolation applies
    guard._mpc_base_timestamp = time.perf_counter() - MAX_EXTRAP_DT_S

    guard._interpolate_and_send(guard.dt_target)

    # Position should be clamped to stroke max
    assert np.all(guard._commanded_pos_rev <= guard._stroke_max_rev + 1e-10), \
        (f"Position {guard._commanded_pos_rev.max():.4f} exceeds stroke max "
         f"{guard._stroke_max_rev:.4f}")
    assert np.all(guard._commanded_pos_rev >= guard._stroke_min_rev - 1e-10), \
        (f"Position {guard._commanded_pos_rev.min():.4f} below stroke min "
         f"{guard._stroke_min_rev:.4f}")

    # Feedforwards must be zeroed for clamped legs
    assert np.all(guard._commanded_vel_ff_rps == 0.0), \
        (f"vel_ff not zeroed on clamped legs: "
         f"{guard._commanded_vel_ff_rps}")
    assert np.all(guard._commanded_torque_ff_Nm == 0.0), \
        (f"torque_ff not zeroed on clamped legs: "
         f"{guard._commanded_torque_ff_Nm}")

    # Also test negative velocity (toward underextension)
    guard._mpc_base_vel_rps = np.full(6, -100.0)
    guard._mpc_base_timestamp = time.perf_counter() - MAX_EXTRAP_DT_S

    guard._interpolate_and_send(guard.dt_target)

    assert np.all(guard._commanded_pos_rev >= guard._stroke_min_rev - 1e-10), \
        (f"Position {guard._commanded_pos_rev.min():.4f} below stroke min "
         f"{guard._stroke_min_rev:.4f}")

    # Feedforwards zeroed for negative clamp too
    assert np.all(guard._commanded_vel_ff_rps == 0.0), \
        (f"vel_ff not zeroed on negative-clamped legs: "
         f"{guard._commanded_vel_ff_rps}")
    assert np.all(guard._commanded_torque_ff_Nm == 0.0), \
        (f"torque_ff not zeroed on negative-clamped legs: "
         f"{guard._commanded_torque_ff_Nm}")

    print("  [PASS] test_stroke_clamp")
    return True


# ---------------------------------------------------------------------------
# Test 25: Enable-when-already-enabled is a no-op
# ---------------------------------------------------------------------------

def test_enable_idempotent():
    """A second enable must NOT reset MPC state (race-condition fix)."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    # Inject an MPC command so the guard has live MPC state
    ext_mm = [100.0, 110.0, 120.0, 130.0, 140.0, 150.0]
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)
    guard._process_ipc()

    assert guard._has_mpc_cmd
    prev_ext = guard._mpc_prev_ext_mm.copy()

    # Send another enable — must be a no-op
    ipc.inject(TOPIC_MODE, make_mode_command('enable', source='MPC'))
    guard._process_ipc()

    assert guard.mode == GuardMode.ENABLED
    assert guard._has_mpc_cmd, "enable-when-ENABLED must not clear _has_mpc_cmd"
    assert guard._mpc_prev_ext_mm is not None, \
        "enable-when-ENABLED must not clear _mpc_prev_ext_mm"
    np.testing.assert_array_equal(guard._mpc_prev_ext_mm, prev_ext)

    print("  [PASS] test_enable_idempotent")
    return True


# ---------------------------------------------------------------------------
# Test 26: Bridge disable+enable clears MPC state
# ---------------------------------------------------------------------------

def test_bridge_disable_enable_clears_state():
    """The bridge's disable+enable sequence must clear MPC state."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    # Inject an MPC command so the guard has live MPC state
    ext_mm = [100.0, 110.0, 120.0, 130.0, 140.0, 150.0]
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)
    guard._process_ipc()

    assert guard._has_mpc_cmd

    # Bridge sequence: disable then enable
    ipc.inject(TOPIC_MODE, make_mode_command('disable'))
    ipc.inject(TOPIC_MODE, make_mode_command('enable', source='SPACEMOUSE'))
    guard._process_ipc()

    assert guard.mode == GuardMode.ENABLED
    assert not guard._has_mpc_cmd, \
        "disable+enable must clear _has_mpc_cmd"
    assert guard._mpc_prev_ext_mm is None, \
        "disable+enable must clear _mpc_prev_ext_mm"

    print("  [PASS] test_bridge_disable_enable_clears_state")
    return True


# ---------------------------------------------------------------------------
# Test 29: Stroke clamp promotes workspace status
# ---------------------------------------------------------------------------

def test_stroke_clamp_workspace_status():
    """Stroke clamp at 500 Hz must update workspace status to HARD_LIMIT.

    Ensures telemetry/GUI reflect that the platform is being held at a
    physical boundary, even though the full workspace check only runs at
    50 Hz on MPC command arrival.
    """
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)
    guard._process_ipc()

    # Confirm workspace status is OK before clamping
    assert guard._workspace_status == WorkspaceStatus.OK, \
        f"Expected OK before clamp, got {guard._workspace_status}"
    assert guard._workspace_speed_scale == 1.0, \
        f"Expected speed_scale=1.0 before clamp, got {guard._workspace_speed_scale}"

    # Force position past stroke limits via absurd velocity.
    # Set motor feedback near stroke max so the tracking clamp allows the
    # extrapolation to reach the stroke boundary.
    guard._mpc_base_vel_rps = np.full(6, 100.0)
    guard._mpc_base_timestamp = time.perf_counter() - MAX_EXTRAP_DT_S
    guard._motor_fb_pos_rev = guard._stroke_max_rev.copy()

    guard._interpolate_and_send(guard.dt_target)

    # Workspace status must now reflect the clamp
    assert guard._workspace_status == WorkspaceStatus.HARD_LIMIT, \
        f"Expected HARD_LIMIT after clamp, got {guard._workspace_status}"
    assert guard._workspace_speed_scale == 0.0, \
        f"Expected speed_scale=0.0 after clamp, got {guard._workspace_speed_scale}"

    # Telemetry should carry the updated status
    assert len(ipc.sent_telemetry) > 0, "No telemetry sent"
    last_telem = ipc.sent_telemetry[-1]
    assert last_telem['workspace_status'] == 'hard', \
        f"Telemetry workspace_status should be 'hard', got {last_telem['workspace_status']}"
    assert last_telem['workspace_speed_scale'] == 0.0, \
        f"Telemetry speed_scale should be 0.0, got {last_telem['workspace_speed_scale']}"

    print("  [PASS] test_stroke_clamp_workspace_status")
    return True


# ---------------------------------------------------------------------------
# Test 30: Cubic interpolation with jerk
# ---------------------------------------------------------------------------

def test_cubic_interpolation_with_jerk():
    """Cubic interpolation with known constant jerk is more accurate than quadratic.

    Sends 3 MPC commands with linearly increasing acceleration (constant jerk),
    then verifies the interpolated position includes the cubic term.
    """
    guard, ipc = _make_guard()
    _enable(guard, ipc)
    mm2rev = guard.geom.mm_to_rev

    ext_mm = [140.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)

    # Command 1: zero velocity, zero acceleration
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, vel_mm_s=[0.0]*6, acc_mm_s2=[0.0]*6, seq=0)
    guard._process_ipc()

    # Command 2: same position, zero velocity, acceleration = 1000 mm/s^2
    time.sleep(0.02)
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, vel_mm_s=[0.0]*6,
                     acc_mm_s2=[1000.0]*6, seq=1)
    guard._process_ipc()

    # Command 3: same position, zero velocity, acceleration = 2000 mm/s^2
    # This gives jerk ≈ (2000-1000)/0.02 = 50000 mm/s^3 (raw)
    time.sleep(0.02)
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, vel_mm_s=[0.0]*6,
                     acc_mm_s2=[2000.0]*6, seq=2)
    guard._process_ipc()

    # Jerk should be non-zero after 3 commands
    assert np.any(np.abs(guard._mpc_base_jerk_rps3) > 0), \
        "Jerk should be non-zero after 3 commands with changing acceleration"

    # Interpolate at a known dt and verify cubic term is present
    dt_test = 0.010  # 10 ms
    guard._mpc_base_timestamp = time.perf_counter() - dt_test
    ipc.sent_telemetry.clear()
    guard._interpolate_and_send(guard.dt_target)

    # Compute expected positions with and without jerk
    dt2 = dt_test * dt_test
    pos_quadratic = (guard._mpc_base_pos_rev
                     + guard._mpc_base_vel_rps * dt_test
                     + 0.5 * guard._mpc_base_accel_rps2 * dt2)
    pos_cubic = (pos_quadratic
                 + (1.0/6.0) * guard._mpc_base_jerk_rps3 * (dt2 * dt_test))

    # Actual should match cubic, not quadratic.
    # Tolerance is 1e-5 rev to absorb wall-clock jitter between
    # timestamp-set and the perf_counter() inside _interpolate_and_send.
    np.testing.assert_allclose(
        guard._commanded_pos_rev, pos_cubic, atol=1e-5,
        err_msg="Interpolation should match cubic formula")

    # Cubic and quadratic should differ (jerk is non-zero)
    diff = np.max(np.abs(pos_cubic - pos_quadratic))
    assert diff > 1e-10, \
        f"Cubic and quadratic should differ, but diff={diff}"

    print("  [PASS] test_cubic_interpolation_with_jerk")
    return True


# ---------------------------------------------------------------------------
# Test 31: Jerk zero on first command
# ---------------------------------------------------------------------------

def test_jerk_zero_on_first_command():
    """Jerk should be zero after the first MPC command (no prior accel)."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, acc_mm_s2=[500.0]*6, seq=0)
    guard._process_ipc()

    np.testing.assert_array_equal(
        guard._mpc_base_jerk_rps3, np.zeros(6),
        err_msg="Jerk should be zero after first command")

    print("  [PASS] test_jerk_zero_on_first_command")
    return True


# ---------------------------------------------------------------------------
# Test 32: Jerk reset on disable
# ---------------------------------------------------------------------------

def test_jerk_reset_on_disable():
    """Jerk state should clear on disable→enable cycle."""
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)

    # Build up jerk state with 3 commands
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, acc_mm_s2=[0.0]*6, seq=0)
    guard._process_ipc()
    time.sleep(0.02)
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, acc_mm_s2=[1000.0]*6, seq=1)
    guard._process_ipc()
    time.sleep(0.02)
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, acc_mm_s2=[2000.0]*6, seq=2)
    guard._process_ipc()

    # Verify jerk is non-zero
    assert np.any(np.abs(guard._mpc_base_jerk_rps3) > 0), \
        "Jerk should be non-zero before disable"
    assert guard._mpc_prev_accel_rps2 is not None, \
        "Prev accel should be set before disable"

    # Disable and re-enable
    ipc.inject(TOPIC_MODE, make_mode_command('disable'))
    guard._process_ipc()
    _enable(guard, ipc)

    # Jerk state should be cleared
    np.testing.assert_array_equal(
        guard._mpc_base_jerk_rps3, np.zeros(6),
        err_msg="Jerk should be zero after disable+enable")
    assert guard._mpc_prev_accel_rps2 is None, \
        "Prev accel should be None after disable+enable"

    print("  [PASS] test_jerk_reset_on_disable")
    return True


# ---------------------------------------------------------------------------
# Test 33: Decay boundary continuity with jerk
# ---------------------------------------------------------------------------

def test_decay_boundary_continuity():
    """Position and velocity are continuous at the MAX_EXTRAP_DT_S boundary.

    With non-zero jerk, the cubic normal path and the decay path must agree
    at dt = MAX_EXTRAP_DT_S.  A discontinuity here would produce a position
    glitch when the MPC delivers a command slightly late during high-jerk motion.
    """
    guard, ipc = _make_guard()
    _enable(guard, ipc)

    ext_mm = [140.0] * 6
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)

    # Build up jerk with 3 commands of increasing acceleration
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, vel_mm_s=[100.0]*6,
                     acc_mm_s2=[0.0]*6, seq=0)
    guard._process_ipc()
    time.sleep(0.02)
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, vel_mm_s=[100.0]*6,
                     acc_mm_s2=[1000.0]*6, seq=1)
    guard._process_ipc()
    time.sleep(0.02)
    _provide_matching_feedback(guard, ipc, ext_mm=ext_mm)
    _inject_mpc_cmd(ipc, ext_mm=ext_mm, vel_mm_s=[100.0]*6,
                     acc_mm_s2=[2000.0]*6, seq=2)
    guard._process_ipc()

    assert np.any(np.abs(guard._mpc_base_jerk_rps3) > 0), \
        "Jerk should be non-zero for this test"

    # Sample just inside the normal window (epsilon before boundary)
    eps = 1e-6
    guard._mpc_base_timestamp = time.perf_counter() - (MAX_EXTRAP_DT_S - eps)
    ipc.sent_telemetry.clear()
    guard._interpolate_and_send(guard.dt_target)
    pos_before = guard._commanded_pos_rev.copy()
    vel_before = guard._commanded_vel_ff_rps.copy()

    # Sample just outside the normal window (epsilon after boundary)
    guard._mpc_base_timestamp = time.perf_counter() - (MAX_EXTRAP_DT_S + eps)
    guard._interpolate_and_send(guard.dt_target)
    pos_after = guard._commanded_pos_rev.copy()
    vel_after = guard._commanded_vel_ff_rps.copy()

    # Position and velocity should be nearly identical across the boundary
    np.testing.assert_allclose(
        pos_before, pos_after, atol=1e-4,
        err_msg="Position discontinuity at decay boundary")
    np.testing.assert_allclose(
        vel_before, vel_after, atol=1e-4,
        err_msg="Velocity discontinuity at decay boundary")

    print("  [PASS] test_decay_boundary_continuity")
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 60)
    print("Motor Guard Verification Tests")
    print("=" * 60)
    print()

    tests = [
        ("1.  Loop timing", test_loop_timing),
        ("2.  IPC latency", test_ipc_latency),
        ("3.  Force conversion", test_force_conversion),
        ("4.  MPC enable", test_mpc_enable),
        ("5.  MPC command flow", test_mpc_command_flow),
        ("6.  MPC staleness E-stop", test_mpc_staleness_estop),
        ("7.  Disable clears state", test_disable_clears_state),
        ("8.  Workspace hard limit", test_workspace_hard_limit_estops),
        ("9.  Torque passthrough", test_torque_passthrough),
        ("10. Zeros when no feedforward", test_zeros_when_no_feedforward),
        ("11. No feedback suppresses", test_no_feedback_suppresses),
        ("12. Stale feedback suppresses", test_stale_feedback_suppresses),
        ("13. Motor overspeed E-stop", test_motor_overspeed),
        ("14. Max deviation E-stop", test_max_deviation_estop),
        ("15. NaN rejection (MPC)", test_nan_rejection),
        ("16. NaN rejection (feedback)", test_nan_feedback_rejection),
        ("17. Vel_ff finite-difference", test_vel_ff_finite_difference),
        ("18. IPC heartbeat E-stop", test_ipc_heartbeat_estop),
        ("19. Interpolation output", test_interpolation_output),
        ("20. Normal interpolation unchanged", test_normal_interpolation_unchanged),
        ("21. Extrapolation vel decays", test_extrapolation_vel_decays),
        ("22. Hold after full decay", test_extrapolation_hold_after_decay),
        ("23. Extrapolation bounded", test_extrapolation_bounded),
        ("24. Stroke clamp", test_stroke_clamp),
        ("25. Enable idempotent", test_enable_idempotent),
        ("26. Bridge disable+enable clears state",
         test_bridge_disable_enable_clears_state),
        ("27. Sender vel_ff preferred", test_vel_ff_sender_preferred),
        ("28. NaN sender vel_ff fallback", test_vel_ff_nan_sender_fallback),
        ("29. Stroke clamp promotes workspace status",
         test_stroke_clamp_workspace_status),
        ("30. Cubic interpolation with jerk",
         test_cubic_interpolation_with_jerk),
        ("31. Jerk zero on first command",
         test_jerk_zero_on_first_command),
        ("32. Jerk reset on disable",
         test_jerk_reset_on_disable),
        ("33. Decay boundary continuity",
         test_decay_boundary_continuity),
    ]

    results = []
    for name, test_fn in tests:
        print(f"\n{name}")
        try:
            passed = test_fn()
        except Exception as e:
            print(f"  [FAIL] {e}")
            import traceback
            traceback.print_exc()
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
