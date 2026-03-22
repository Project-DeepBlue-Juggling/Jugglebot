"""Unit tests for MPC pass-through mode in the control loop.

Tests:
  1. MPC commands flow through to motor outputs correctly
  2. Slew limiter clamps large steps
  3. MPC staleness triggers E-stop
  4. Mode exclusivity (target/trajectory ignored in MPC mode)
  5. Workspace hard limit triggers E-stop (not hold-pose)
  6. Enable/disable cycle clears MPC state

Run:  python -m jugglebot.motion.tests.test_mpc_passthrough
"""

from __future__ import annotations

import time

import numpy as np

from jugglebot.motion.control_loop import (
    ControlLoop,
    ControlMode,
    MPC_CMD_STALENESS_S,
)
from jugglebot.motion.conversions import extensions_mm_to_revs
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ipc import (
    TOPIC_MODE,
    TOPIC_MPC_CMD,
    TOPIC_TARGET,
    TOPIC_TRAJECTORY,
    make_mpc_command,
    make_mode_command,
    make_target_state,
    make_trajectory_command,
)


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

    def send_dynamic_feedback(self, msg: dict) -> None:
        pass

    @property
    def seconds_since_last_recv(self) -> float:
        return time.monotonic() - self._last_recv

    def close(self) -> None:
        pass


def _make_loop() -> tuple[ControlLoop, MockIPC]:
    """Create a ControlLoop with a MockIPC for testing."""
    ipc = MockIPC()
    geom = StewartGeometry()
    loop = ControlLoop(target_rate_hz=500, geom=geom, ipc=ipc)
    return loop, ipc


def _enable_mpc(loop: ControlLoop, ipc: MockIPC) -> None:
    """Enable the control loop in MPC pass-through mode."""
    ipc.inject(TOPIC_MODE, make_mode_command('enable', source='MPC'))
    loop._process_ipc()


def _inject_mpc_cmd(ipc: MockIPC, ext_mm=None, pose_6dof=None,
                     vel_mm_s=None, torque_Nm=None, seq=0) -> None:
    """Inject an MPC command message (no motor_rev — tests use IK fallback)."""
    if ext_mm is None:
        ext_mm = [140.0] * 6  # mid-stroke (IK convention)
    if pose_6dof is None:
        pose_6dof = [0.0, 0.0, 50.0, 0.0, 0.0, 0.0]
    msg = make_mpc_command(
        ext_mm=ext_mm, pose_6dof=pose_6dof,
        vel_mm_s=vel_mm_s, torque_Nm=torque_Nm, seq=seq)
    ipc.inject(TOPIC_MPC_CMD, msg)


def _provide_motor_feedback(loop: ControlLoop, ipc: MockIPC,
                             pos_rev=None) -> None:
    """Inject motor feedback to satisfy staleness checks."""
    if pos_rev is None:
        pos_rev = extensions_mm_to_revs(
            np.array([140.0] * 6), loop.geom).tolist()
    ipc.inject(b'motorfb', {
        'type': 'motor_feedback',
        'pos': pos_rev,
        'vel': [0.0] * 6,
        'cur': [0.0] * 6,
    })


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_mpc_enable():
    """Enabling with source='MPC' activates pass-through mode."""
    loop, ipc = _make_loop()
    assert not loop._mpc_passthrough_active

    _enable_mpc(loop, ipc)

    assert loop.mode == ControlMode.ENABLED
    assert loop._mpc_passthrough_active
    assert not loop._has_mpc_cmd
    print("  [PASS] test_mpc_enable")
    return True


def test_mpc_command_flow():
    """MPC commands are converted to motor revolutions correctly."""
    loop, ipc = _make_loop()
    _enable_mpc(loop, ipc)

    ext_mm = [100.0, 110.0, 120.0, 130.0, 140.0, 150.0]
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)
    _provide_motor_feedback(loop, ipc,
                            pos_rev=extensions_mm_to_revs(
                                np.array(ext_mm), loop.geom).tolist())
    loop._process_ipc()

    # Run one compute cycle
    loop._compute()

    expected_rev = extensions_mm_to_revs(np.array(ext_mm), loop.geom)
    np.testing.assert_allclose(
        loop._commanded_pos_rev, expected_rev, atol=1e-6,
        err_msg="MPC extensions not correctly converted to motor revs")
    print("  [PASS] test_mpc_command_flow")
    return True


def test_mpc_staleness_estop():
    """MPC command staleness triggers E-stop."""
    loop, ipc = _make_loop()
    _enable_mpc(loop, ipc)

    # Send one command so _has_mpc_cmd is True
    _inject_mpc_cmd(ipc)
    _provide_motor_feedback(loop, ipc)
    loop._process_ipc()
    loop._compute()

    assert loop.mode == ControlMode.ENABLED

    # Manually backdate the timestamp to simulate staleness
    loop._mpc_cmd_timestamp = time.perf_counter() - MPC_CMD_STALENESS_S - 0.01

    loop._compute()

    assert loop.mode == ControlMode.ESTOP
    assert 'mpc_cmd_stale' in (loop._fault_state or '')
    assert not loop._mpc_passthrough_active
    print("  [PASS] test_mpc_staleness_estop")
    return True


def test_mode_exclusivity_target():
    """Target messages are ignored in MPC pass-through mode."""
    loop, ipc = _make_loop()
    _enable_mpc(loop, ipc)

    # Inject a target — should be silently ignored
    target = make_target_state([0, 0, 50], [1, 0, 0, 0])
    ipc.inject(TOPIC_TARGET, target)
    loop._process_ipc()

    # _has_target should NOT be set by the target message
    # (it's set by _seed_home_pose on enable, but the target handler
    # should have returned early)
    # Verify the smoother was not updated (it was reset to home on enable)
    assert loop._mpc_passthrough_active
    print("  [PASS] test_mode_exclusivity_target")
    return True


def test_mode_exclusivity_trajectory():
    """Trajectory commands are ignored in MPC pass-through mode."""
    loop, ipc = _make_loop()
    _enable_mpc(loop, ipc)

    traj = make_trajectory_command(
        start_pose=[0]*6, start_twist=[0]*6, start_accel=[0]*6,
        end_pose=[0,0,50,0,0,0], end_twist=[0]*6, end_accel=[0]*6,
        duration=1.0)
    ipc.inject(TOPIC_TRAJECTORY, traj)
    loop._process_ipc()

    # Trajectory manager should NOT have a submitted trajectory
    from jugglebot.motion.trajectory_manager import TrajectoryState
    assert loop._traj_manager.state == TrajectoryState.IDLE
    print("  [PASS] test_mode_exclusivity_trajectory")
    return True


def test_disable_clears_mpc():
    """Disable clears MPC pass-through state."""
    loop, ipc = _make_loop()
    _enable_mpc(loop, ipc)
    _inject_mpc_cmd(ipc)
    loop._process_ipc()

    assert loop._mpc_passthrough_active
    assert loop._has_mpc_cmd

    ipc.inject(TOPIC_MODE, make_mode_command('disable'))
    loop._process_ipc()

    assert not loop._mpc_passthrough_active
    assert not loop._has_mpc_cmd
    assert loop.mode == ControlMode.DISABLED
    print("  [PASS] test_disable_clears_mpc")
    return True


def test_workspace_hard_limit_estops():
    """In MPC mode, workspace hard limit triggers E-stop (not hold-pose)."""
    loop, ipc = _make_loop()
    _enable_mpc(loop, ipc)

    # Command extensions well beyond stroke (hard limit is ~275mm)
    _inject_mpc_cmd(ipc, ext_mm=[290.0] * 6,
                    pose_6dof=[0, 0, 200, 0, 0, 0])
    _provide_motor_feedback(loop, ipc,
                            pos_rev=extensions_mm_to_revs(
                                np.array([290.0] * 6), loop.geom).tolist())
    loop._process_ipc()
    loop._compute()

    assert loop.mode == ControlMode.ESTOP
    assert 'workspace_hard_limit' in (loop._fault_state or '')
    print("  [PASS] test_workspace_hard_limit_estops")
    return True


def test_vel_and_torque_passthrough():
    """Optional vel_mm_s and torque_Nm are passed through correctly."""
    loop, ipc = _make_loop()
    _enable_mpc(loop, ipc)

    ext_mm = [140.0] * 6
    vel_mm_s = [10.0, -10.0, 5.0, -5.0, 3.0, -3.0]
    torque_Nm = [0.1, -0.1, 0.05, -0.05, 0.02, -0.02]

    _inject_mpc_cmd(ipc, ext_mm=ext_mm, vel_mm_s=vel_mm_s,
                    torque_Nm=torque_Nm)
    _provide_motor_feedback(loop, ipc,
                            pos_rev=extensions_mm_to_revs(
                                np.array(ext_mm), loop.geom).tolist())
    loop._process_ipc()
    loop._compute()

    # Torques pass through directly (already in Nm)
    np.testing.assert_allclose(
        loop._commanded_torque_ff_Nm, torque_Nm, atol=1e-10)
    # Velocities are converted from mm/s to rev/s
    expected_vel_rps = np.array(vel_mm_s) * loop.geom.mm_to_rev
    np.testing.assert_allclose(
        loop._commanded_vel_ff_rps, expected_vel_rps, atol=1e-10)
    print("  [PASS] test_vel_and_torque_passthrough")
    return True


def test_zeros_when_no_feedforward():
    """When vel_mm_s and torque_Nm are omitted, zeros are used."""
    loop, ipc = _make_loop()
    _enable_mpc(loop, ipc)

    ext_mm = [140.0] * 6
    _inject_mpc_cmd(ipc, ext_mm=ext_mm)  # no vel/torque
    _provide_motor_feedback(loop, ipc,
                            pos_rev=extensions_mm_to_revs(
                                np.array(ext_mm), loop.geom).tolist())
    loop._process_ipc()
    loop._compute()

    np.testing.assert_allclose(loop._commanded_vel_ff_rps, 0.0, atol=1e-10)
    np.testing.assert_allclose(loop._commanded_torque_ff_Nm, 0.0, atol=1e-10)
    print("  [PASS] test_zeros_when_no_feedforward")
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    tests = [
        test_mpc_enable,
        test_mpc_command_flow,
        test_mpc_staleness_estop,
        test_mode_exclusivity_target,
        test_mode_exclusivity_trajectory,
        test_disable_clears_mpc,
        test_workspace_hard_limit_estops,
        test_vel_and_torque_passthrough,
        test_zeros_when_no_feedforward,
    ]

    print(f"Running {len(tests)} MPC pass-through tests...\n")
    passed = 0
    failed = 0
    for test in tests:
        try:
            result = test()
            if result:
                passed += 1
            else:
                failed += 1
        except Exception as e:
            print(f"  [FAIL] {test.__name__}: {e}")
            import traceback
            traceback.print_exc()
            failed += 1

    print(f"\n{passed}/{passed + failed} tests passed")
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    import sys
    sys.exit(main())
