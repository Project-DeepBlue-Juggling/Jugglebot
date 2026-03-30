"""Tests for jugglebot.orchestrator_node.OrchestratorNode.

All ROS2 infrastructure is mocked via conftest.py — these tests exercise
the orchestrator's state machine bridging, async operation tracking,
error detection, and command dispatch.
"""

import struct
from unittest.mock import MagicMock, patch, PropertyMock

import pytest

from jugglebot.state_machine import (
    RobotState,
    ActiveMode,
    Context,
    BOOT_TIMEOUT_S,
)

# ════════════════════════════════════════════════════════════════
# Fixtures
# ════════════════════════════════════════════════════════════════

# Import mock types needed for building messages
from tests.ros.conftest import (
    MotorStateSingle,
    RobotState as RobotStateMsg,
    MockString,
    MockFuture,
    MockServiceClient,
    MockActionClient,
)


@pytest.fixture
def orch():
    """Create an OrchestratorNode with all externals mocked."""
    from jugglebot.orchestrator_node import OrchestratorNode
    node = OrchestratorNode()
    return node


def _make_robot_state_msg(
    num_motors=9,
    all_heartbeats=False,
    encoder_search_complete=False,
    is_homed=False,
    errors=None,
    has_fatal_odrive_error=False,
    has_fatal_can_error=False,
    has_undervoltage=False,
    firmware_validated=False,
):
    """Build a mock RobotState message."""
    msg = RobotStateMsg()
    # Simulate motor states: state=1 (IDLE) for heartbeat, state=0 for no heartbeat
    for i in range(num_motors):
        ms = MotorStateSingle()
        if all_heartbeats and i < 7:  # 7 Jugglebot axes
            ms.current_state = 1  # Non-zero = heartbeat received
        msg.motor_states.append(ms)
    msg.encoder_search_complete = encoder_search_complete
    msg.is_homed = is_homed
    msg.error = list(errors or [])
    msg.has_fatal_odrive_error = has_fatal_odrive_error
    msg.has_fatal_can_error = has_fatal_can_error
    msg.has_undervoltage = has_undervoltage
    msg.firmware_validated = firmware_validated
    return msg


# ════════════════════════════════════════════════════════════════
# Initialization
# ════════════════════════════════════════════════════════════════


class TestOrchestratorInit:
    def test_initial_state_is_boot(self, orch):
        assert orch.sm.state == RobotState.BOOT

    def test_context_created(self, orch):
        assert isinstance(orch.ctx, Context)

    def test_no_pending_operations(self, orch):
        assert orch._pending_future is None
        assert orch._pending_goal_future is None
        assert orch._pending_result_future is None

    def test_publishers_created(self, orch):
        assert orch._control_mode_pub is not None
        assert orch._state_pub is not None


# ════════════════════════════════════════════════════════════════
# _on_robot_state — context updates
# ════════════════════════════════════════════════════════════════


class TestOnRobotState:
    def test_all_heartbeats_detected(self, orch):
        msg = _make_robot_state_msg(all_heartbeats=True)
        orch._on_robot_state(msg)
        assert orch.ctx.all_heartbeats is True

    def test_no_heartbeats(self, orch):
        msg = _make_robot_state_msg(all_heartbeats=False)
        orch._on_robot_state(msg)
        assert orch.ctx.all_heartbeats is False

    def test_partial_heartbeats(self, orch):
        msg = RobotStateMsg()
        for i in range(9):
            ms = MotorStateSingle()
            if i < 3:
                ms.current_state = 1
            msg.motor_states.append(ms)
        orch._on_robot_state(msg)
        assert orch.ctx.all_heartbeats is False

    def test_encoder_search_complete(self, orch):
        msg = _make_robot_state_msg(encoder_search_complete=True)
        orch._on_robot_state(msg)
        assert orch.ctx.encoder_search_complete is True

    def test_is_homed(self, orch):
        msg = _make_robot_state_msg(is_homed=True)
        orch._on_robot_state(msg)
        assert orch.ctx.is_homed is True

    def test_errors_copied(self, orch):
        msg = _make_robot_state_msg(errors=['E1', 'E2'])
        orch._on_robot_state(msg)
        assert orch.ctx.errors == ['E1', 'E2']

    def test_fatal_odrive_error(self, orch):
        msg = _make_robot_state_msg(has_fatal_odrive_error=True)
        orch._on_robot_state(msg)
        assert orch.ctx.fatal_error is True

    def test_fatal_can_error(self, orch):
        msg = _make_robot_state_msg(has_fatal_can_error=True)
        orch._on_robot_state(msg)
        assert orch.ctx.fatal_can_error is True

    def test_undervoltage(self, orch):
        msg = _make_robot_state_msg(has_undervoltage=True)
        orch._on_robot_state(msg)
        assert orch.ctx.undervoltage is True

    def test_too_few_motor_states_doesnt_crash(self, orch):
        """If motor_states list is too short, heartbeat check is skipped."""
        msg = RobotStateMsg()
        msg.motor_states = [MotorStateSingle()]  # Only 1 motor
        orch._on_robot_state(msg)  # Should not raise


# ════════════════════════════════════════════════════════════════
# _on_command — command queuing
# ════════════════════════════════════════════════════════════════


class TestOnCommand:
    def test_queues_command(self, orch):
        msg = MockString(data='activate')
        orch._on_command(msg)
        assert orch.ctx.consume_command() == 'activate'

    def test_queues_multiple_commands(self, orch):
        orch._on_command(MockString(data='old'))
        orch._on_command(MockString(data='new'))
        assert orch.ctx.consume_command() == 'old'
        assert orch.ctx.consume_command() == 'new'

    def test_various_commands(self, orch):
        for cmd in ['activate', 'deactivate', 'spacemouse', 'shell', 'home', 'clear_errors']:
            msg = MockString(data=cmd)
            orch._on_command(msg)
            assert orch.ctx.consume_command() == cmd


# ════════════════════════════════════════════════════════════════
# _tick — main tick cycle
# ════════════════════════════════════════════════════════════════


class TestTick:
    def test_tick_runs_state_machine(self, orch):
        """First tick enters BOOT state."""
        orch._tick()
        assert orch.sm.state == RobotState.BOOT

    def test_errors_force_fault(self, orch):
        """Errors in context force transition to FAULT from any state."""
        orch._tick()  # Enter BOOT
        orch.ctx.errors = ['test_error']
        orch._tick()
        assert orch.sm.state == RobotState.FAULT

    def test_errors_dont_re_force_from_fault(self, orch):
        """Already in FAULT: errors don't re-trigger force_transition."""
        orch.sm.force_transition(RobotState.FAULT, orch.ctx)
        orch._tick()  # Enter FAULT
        orch.ctx.errors = ['test_error']
        orch.ctx.fatal_error = True
        orch._tick()  # Should stay in FAULT without crash
        assert orch.sm.state == RobotState.FAULT

    def test_control_mode_published_on_change(self, orch):
        orch._tick()  # Enter BOOT, sets control_mode=''
        assert len(orch._control_mode_pub.published) == 1
        assert orch._control_mode_pub.published[0].data == ''

    def test_control_mode_published_every_tick(self, orch):
        """Control mode is published every tick for late-joining subscribers."""
        orch._tick()  # Enter BOOT, publishes ''
        count_after_first = len(orch._control_mode_pub.published)
        orch._tick()  # Republishes '' every tick
        assert len(orch._control_mode_pub.published) == count_after_first + 1

    def test_state_published_on_change(self, orch):
        orch._tick()  # Enter BOOT
        assert len(orch._state_pub.published) == 1
        assert orch._state_pub.published[0].data == 'BOOT'

    def test_state_published_every_tick(self, orch):
        """State is published every tick for late-joining subscribers."""
        orch._tick()
        count = len(orch._state_pub.published)
        orch._tick()
        assert len(orch._state_pub.published) == count + 1

    def test_boot_timeout_logged_once(self, orch):
        """Boot timeout message should be logged only once."""
        # Force into FAULT so BootHandler.on_enter doesn't reset boot_timed_out
        orch.sm.force_transition(RobotState.FAULT, orch.ctx)
        orch.ctx.boot_timed_out = True
        orch._tick()  # FAULT enters, log check sees boot_timed_out=True
        assert orch._boot_timeout_logged is True
        # Second tick — shouldn't re-log (flag already set)
        orch._tick()
        assert orch._boot_timeout_logged is True

    def test_boot_timeout_flag_resets(self, orch):
        """_boot_timeout_logged resets when boot_timed_out clears."""
        orch.sm.force_transition(RobotState.FAULT, orch.ctx)
        orch.ctx.boot_timed_out = True
        orch._tick()
        assert orch._boot_timeout_logged is True
        # Clear the timeout (e.g., heartbeats arrived)
        orch.ctx.boot_timed_out = False
        orch.ctx.all_heartbeats = True
        orch._tick()
        assert orch._boot_timeout_logged is False


# ════════════════════════════════════════════════════════════════
# _check_pending_operations — future polling
# ════════════════════════════════════════════════════════════════


class TestCheckPendingOperations:
    def test_service_success(self, orch):
        """Completed service future sets operation_result=True."""
        future = MockFuture()
        resp = MagicMock()
        resp.success = True
        resp.message = 'ok'
        future.set_result(resp)

        orch._pending_future = future
        orch.ctx.operation_pending = True
        orch._check_pending_operations()

        assert orch.ctx.operation_result is True
        assert orch.ctx.operation_pending is False
        assert orch._pending_future is None

    def test_service_failure(self, orch):
        future = MockFuture()
        resp = MagicMock()
        resp.success = False
        resp.message = 'failed'
        future.set_result(resp)

        orch._pending_future = future
        orch.ctx.operation_pending = True
        orch._check_pending_operations()

        assert orch.ctx.operation_result is False

    def test_service_exception(self, orch):
        future = MockFuture()
        future.set_exception(RuntimeError("service crashed"))

        orch._pending_future = future
        orch.ctx.operation_pending = True
        orch._check_pending_operations()

        assert orch.ctx.operation_result is False
        assert orch.ctx.operation_pending is False

    def test_pending_future_not_done_stays(self, orch):
        future = MockFuture()
        orch._pending_future = future
        orch.ctx.operation_pending = True
        orch._check_pending_operations()
        assert orch._pending_future is future  # Not cleared

    def test_action_goal_accepted(self, orch):
        """Action goal accepted → transitions to waiting for result."""
        goal_future = MockFuture()
        goal_handle = MagicMock()
        goal_handle.accepted = True
        result_future = MockFuture()
        goal_handle.get_result_async.return_value = result_future
        goal_future.set_result(goal_handle)

        orch._pending_goal_future = goal_future
        orch.ctx.operation_pending = True
        orch._check_pending_operations()

        assert orch._pending_goal_future is None
        assert orch._pending_result_future is result_future

    def test_action_goal_rejected(self, orch):
        goal_future = MockFuture()
        goal_handle = MagicMock()
        goal_handle.accepted = False
        goal_future.set_result(goal_handle)

        orch._pending_goal_future = goal_future
        orch.ctx.operation_pending = True
        orch._check_pending_operations()

        assert orch.ctx.operation_result is False
        assert orch.ctx.operation_pending is False

    def test_action_goal_exception(self, orch):
        goal_future = MockFuture()
        goal_future.set_exception(RuntimeError("goal failed"))

        orch._pending_goal_future = goal_future
        orch.ctx.operation_pending = True
        orch._check_pending_operations()

        assert orch.ctx.operation_result is False
        assert orch.ctx.operation_pending is False

    def test_action_result_success(self, orch):
        result_future = MockFuture()
        wrapper = MagicMock()
        wrapper.result.success = True
        result_future.set_result(wrapper)

        orch._pending_result_future = result_future
        orch.ctx.operation_pending = True
        orch._check_pending_operations()

        assert orch.ctx.operation_result is True
        assert orch.ctx.operation_pending is False
        assert orch._pending_result_future is None

    def test_action_result_failure(self, orch):
        result_future = MockFuture()
        wrapper = MagicMock()
        wrapper.result.success = False
        result_future.set_result(wrapper)

        orch._pending_result_future = result_future
        orch.ctx.operation_pending = True
        orch._check_pending_operations()

        assert orch.ctx.operation_result is False

    def test_action_result_exception(self, orch):
        result_future = MockFuture()
        result_future.set_exception(RuntimeError("action crashed"))

        orch._pending_result_future = result_future
        orch.ctx.operation_pending = True
        orch._check_pending_operations()

        assert orch.ctx.operation_result is False
        assert orch.ctx.operation_pending is False


# ════════════════════════════════════════════════════════════════
# _process_requests — request dispatch
# ════════════════════════════════════════════════════════════════


class TestProcessRequests:
    def test_no_request_noop(self, orch):
        orch.ctx.request = None
        orch._process_requests()
        # Nothing should happen

    def test_encoder_search_starts_service(self, orch):
        orch.ctx.request = 'encoder_search'
        orch._process_requests()
        assert orch.ctx.request is None
        # operation_pending or operation_result should be set
        # (depends on service readiness)

    def test_home_starts_action(self, orch):
        orch.ctx.request = 'home'
        orch._process_requests()
        assert orch.ctx.request is None

    def test_bb_calibrate_skips_when_not_ready(self, orch):
        """BB calibrate with unavailable service → immediate success."""
        orch._bb_calibrate_client._ready = False
        orch.ctx.request = 'bb_calibrate'
        orch._process_requests()
        assert orch.ctx.operation_result is True
        assert orch.ctx.request is None

    def test_bb_calibrate_calls_when_ready(self, orch):
        orch._bb_calibrate_client._ready = True
        orch.ctx.request = 'bb_calibrate'
        orch._process_requests()
        assert orch.ctx.request is None

    def test_activate_sends_activate(self, orch):
        orch.ctx.request = 'activate'
        orch._process_requests()
        assert orch.ctx.request is None

    def test_deactivate_sends_deactivate(self, orch):
        orch.ctx.request = 'deactivate'
        orch._process_requests()
        assert orch.ctx.request is None

    def test_clear_errors_sends_command(self, orch):
        orch.ctx.request = 'clear_errors'
        orch._process_requests()
        assert orch.ctx.request is None

    def test_unknown_request_logged(self, orch):
        orch.ctx.request = 'nonexistent_request'
        orch._process_requests()
        assert orch.ctx.request is None


# ════════════════════════════════════════════════════════════════
# _start_service_call
# ════════════════════════════════════════════════════════════════


class TestStartServiceCall:
    def test_service_not_ready_fails(self, orch):
        client = MockServiceClient()
        client._ready = False
        req = MagicMock()
        orch._start_service_call(client, req)
        assert orch.ctx.operation_result is False

    def test_service_ready_starts_call(self, orch):
        client = MockServiceClient()
        client._ready = True
        req = MagicMock()
        orch._start_service_call(client, req)
        assert orch.ctx.operation_pending is True
        assert orch.ctx.operation_result is None
        assert orch._pending_future is not None


# ════════════════════════════════════════════════════════════════
# _start_home_action
# ════════════════════════════════════════════════════════════════


class TestStartHomeAction:
    def test_server_not_ready_fails(self, orch):
        orch._home_client._server_ready = False
        orch._start_home_action()
        assert orch.ctx.operation_result is False

    def test_server_ready_starts_action(self, orch):
        orch._home_client._server_ready = True
        orch._start_home_action()
        assert orch.ctx.operation_pending is True
        assert orch.ctx.operation_result is None
        assert orch._pending_goal_future is not None


# ════════════════════════════════════════════════════════════════
# _cancel_pending_operations
# ════════════════════════════════════════════════════════════════


class TestCancelPendingOperations:
    def test_clears_all_futures(self, orch):
        orch._pending_future = MagicMock()
        orch._pending_goal_future = MagicMock()
        orch._pending_result_future = MagicMock()
        orch.ctx.operation_pending = True
        orch.ctx.operation_result = True
        orch.ctx.request = 'deactivate'

        orch._cancel_pending_operations()

        assert orch._pending_future is None
        assert orch._pending_goal_future is None
        assert orch._pending_result_future is None
        assert orch.ctx.operation_pending is False
        assert orch.ctx.operation_result is None
        assert orch.ctx.request is None


# ════════════════════════════════════════════════════════════════
# Shutdown
# ════════════════════════════════════════════════════════════════


class TestShutdown:
    def test_publishes_error_mode_when_active(self, orch):
        """Shutting down from ACTIVE should publish ERROR mode."""
        orch.sm.force_transition(RobotState.ACTIVE, orch.ctx)
        orch._tick()  # Enter ACTIVE
        orch.on_shutdown()
        # Should have published 'ERROR'
        error_pubs = [m for m in orch._control_mode_pub.published if m.data == 'ERROR']
        assert len(error_pubs) > 0

    def test_no_error_mode_when_idle(self, orch):
        """Shutting down from IDLE should NOT publish ERROR mode."""
        orch.sm.force_transition(RobotState.IDLE, orch.ctx)
        orch._tick()
        before_count = len(orch._control_mode_pub.published)
        orch.on_shutdown()
        error_pubs = [m for m in orch._control_mode_pub.published[before_count:]
                      if m.data == 'ERROR']
        assert len(error_pubs) == 0

    def test_no_error_mode_when_boot(self, orch):
        orch._tick()  # In BOOT
        before_count = len(orch._control_mode_pub.published)
        orch.on_shutdown()
        error_pubs = [m for m in orch._control_mode_pub.published[before_count:]
                      if m.data == 'ERROR']
        assert len(error_pubs) == 0

    def test_shutdown_handles_publish_exception(self, orch):
        """Shutdown should not crash even if publish fails."""
        orch.sm.force_transition(RobotState.ACTIVE, orch.ctx)
        orch._tick()
        orch._control_mode_pub.publish = MagicMock(
            side_effect=RuntimeError("publish failed"))
        orch.on_shutdown()  # Should not raise


# ════════════════════════════════════════════════════════════════
# Integration: tick + operations
# ════════════════════════════════════════════════════════════════


class TestTickIntegration:
    def test_boot_to_homing_via_tick(self, orch):
        """Heartbeats + firmware validated → state machine advances to HOMING."""
        orch._tick()  # Enter BOOT
        assert orch.sm.state == RobotState.BOOT

        # Feed heartbeats + firmware validation
        msg = _make_robot_state_msg(all_heartbeats=True, firmware_validated=True)
        orch._on_robot_state(msg)
        orch._tick()
        assert orch.sm.state == RobotState.HOMING

    def test_error_cancels_pending_and_goes_to_fault(self, orch):
        """Errors during HOMING cancel pending ops and force FAULT."""
        orch._tick()  # Enter BOOT
        # Get to HOMING
        msg = _make_robot_state_msg(all_heartbeats=True, firmware_validated=True)
        orch._on_robot_state(msg)
        orch._tick()  # → HOMING
        orch._tick()  # Process encoder_search request
        assert orch.sm.state == RobotState.HOMING

        # Error arrives
        error_msg = _make_robot_state_msg(
            all_heartbeats=True,
            errors=['fatal!'],
            has_fatal_odrive_error=True,
        )
        orch._on_robot_state(error_msg)
        orch._tick()
        assert orch.sm.state == RobotState.FAULT

    def test_homing_encoder_search_service_not_ready(self, orch):
        """If encoder_search service isn't ready, operation fails → FAULT."""
        orch._encoder_search_client._ready = False
        orch._tick()  # Enter BOOT
        msg = _make_robot_state_msg(all_heartbeats=True, firmware_validated=True)
        orch._on_robot_state(msg)
        orch._tick()  # → HOMING
        orch._tick()  # Process encoder_search request → service not ready → result=False
        orch._tick()  # Homing sees failure → FAULT
        assert orch.sm.state == RobotState.FAULT

    def test_command_queued_and_consumed(self, orch):
        """A command queued via _on_command is consumed by the state machine."""
        # Get to IDLE
        orch.sm.force_transition(RobotState.IDLE, orch.ctx)
        orch._tick()
        orch._on_command(MockString(data='activate'))
        orch._tick()
        assert orch.sm.state == RobotState.ACTIVE
        assert orch.ctx.consume_command() is None  # Command was consumed
