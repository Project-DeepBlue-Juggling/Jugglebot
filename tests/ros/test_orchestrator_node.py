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
from std_srvs.srv import Trigger


@pytest.fixture
def orch():
    """Create an OrchestratorNode with all externals mocked."""
    from jugglebot.orchestrator_node import OrchestratorNode
    node = OrchestratorNode()
    return node


def _feed_mocap_status(orch, *, receiving='1', visible='5', marker3='1',
                       age_s=0.0):
    """Deliver one ``mocap/status`` sample to the orchestrator (F4/Q3).

    Healthy by default. ``age_s`` back-dates the cached arrival stamp, which is
    how the staleness branch is reached without sleeping.
    """
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from jugglebot import mocap_status as mocap_st

    msg = DiagnosticStatus()
    msg.values = [
        KeyValue(key=mocap_st.KEY_QTM_RECEIVING, value=receiving),
        KeyValue(key=mocap_st.KEY_BB_MARKERS_VISIBLE, value=visible),
        KeyValue(key=mocap_st.KEY_MARKER3_VISIBLE, value=marker3),
        KeyValue(key=mocap_st.KEY_ALIGNED, value='1'),
        KeyValue(key=mocap_st.KEY_QTM_SYNCED, value='1'),
    ]
    orch._on_mocap_status(msg)
    if age_s:
        orch._mocap_status_mono -= age_s
    return msg


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
        for cmd in ['activate', 'deactivate', 'spacemouse', 'home', 'clear_errors']:
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
        """Needs a healthy mocap/status too (F4/Q3) — without one the QTM gate
        below skips instead of calling."""
        orch._bb_calibrate_client._ready = True
        _feed_mocap_status(orch)
        orch.ctx.request = 'bb_calibrate'
        orch._process_requests()
        assert orch.ctx.request is None
        assert orch._pending_future is not None

    # ── F4/Q3: QTM gate on the HOMING bb_calibrate step ──────────────
    #
    # The orchestrator reads mocap/status itself rather than calling the
    # bridge and interpreting the refusal. That is not a style choice: the
    # bridge refuses with success=False, and HomingHandler.execute turns
    # `operation_result is False` straight into FAULT (state_machine.py). So
    # "the mocap PC is off" would become a faulted robot the operator has to
    # clear before the platform can home — for a subsystem (BB) that is
    # optional. Reading the topic lets HOMING SKIP, exactly as it already does
    # when the BB service itself is absent.

    def test_bb_calibrate_skips_when_qtm_never_reported(self, orch):
        """Service ready, QTM silent → SKIP, and specifically NOT a failure."""
        orch._bb_calibrate_client._ready = True
        orch.ctx.request = 'bb_calibrate'
        orch._process_requests()
        assert orch.ctx.bb_calibration_skipped is True
        assert orch.ctx.operation_result is True
        assert orch.ctx.request is None
        assert orch._pending_future is None      # the service was NOT called

    def test_bb_calibrate_skips_when_qtm_not_receiving(self, orch):
        orch._bb_calibrate_client._ready = True
        _feed_mocap_status(orch, receiving='0')
        orch.ctx.request = 'bb_calibrate'
        orch._process_requests()
        assert orch.ctx.bb_calibration_skipped is True
        assert orch.ctx.operation_result is True
        assert orch._pending_future is None

    def test_bb_calibrate_skips_when_bb_markers_not_visible(self, orch):
        orch._bb_calibrate_client._ready = True
        _feed_mocap_status(orch, visible='1')
        orch.ctx.request = 'bb_calibrate'
        orch._process_requests()
        assert orch.ctx.bb_calibration_skipped is True
        assert orch.ctx.operation_result is True
        assert orch._pending_future is None

    def test_bb_calibrate_skips_on_stale_mocap_status(self, orch):
        """A status that stopped arriving is the same as no status: mocap_node
        may have died, in which case nothing would ever say 'not receiving'."""
        orch._bb_calibrate_client._ready = True
        _feed_mocap_status(orch, age_s=5.0)
        orch.ctx.request = 'bb_calibrate'
        orch._process_requests()
        assert orch.ctx.bb_calibration_skipped is True
        assert orch.ctx.operation_result is True
        assert orch._pending_future is None

    def test_qtm_skip_never_faults_homing(self, orch):
        """THE point of Q3, asserted against the state machine rather than the
        dispatcher: run the skip and then tick HOMING, and the machine must not
        land in FAULT. ``operation_result False`` is the edge that would do it.
        """
        from jugglebot.state_machine import RobotState

        orch._bb_calibrate_client._ready = True
        orch.sm.force_transition(RobotState.HOMING, orch.ctx)
        orch.ctx.request = 'bb_calibrate'
        orch._process_requests()

        assert orch.ctx.operation_result is not False
        orch.sm.tick(orch.ctx)
        assert orch.sm.state != RobotState.FAULT

    def test_qtm_gate_uses_the_shared_predicate(self, orch):
        """Same predicate as the bridge's service gate (jugglebot.mocap_status),
        so the two views of 'ready' cannot drift into disagreeing about whether
        HOMING skips while the GUI button dispatches."""
        from jugglebot import mocap_status as mocap_st
        _feed_mocap_status(orch, visible='2')
        ready, code, _ = orch._qtm_ready()
        assert ready is False
        assert code == mocap_st.CODE_BB_MARKERS_NOT_VISIBLE

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
    # Task 3.3: the orchestrator's on_shutdown must NOT command any mode change.
    # Publishing control_mode='ERROR' on Ctrl-C is a dead can_node-era path that is
    # now actively harmful — ERROR leaves trajectory_node's streaming set, stopping
    # the 40 Hz emitter; if the bridge is still armed, the stream silence latches an
    # MPC_STALE E-STOP within 250 ms, which is exactly what makes the bridge's own
    # profiled DEACTIVATE impossible. The profiled stow is owned by
    # teensy_bridge_node.on_shutdown, which disarms first.

    def test_no_error_mode_when_active(self, orch):
        """Shutting down from ACTIVE must NOT publish ERROR mode (would kill the
        emitter and latch the guard that blocks the bridge's stow)."""
        orch.sm.force_transition(RobotState.ACTIVE, orch.ctx)
        orch._tick()  # Enter ACTIVE
        before_count = len(orch._control_mode_pub.published)
        orch.on_shutdown()
        error_pubs = [m for m in orch._control_mode_pub.published[before_count:]
                      if m.data == 'ERROR']
        assert len(error_pubs) == 0

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

    def test_shutdown_never_raises(self, orch):
        """Shutdown must be a safe no-op that never raises, from any state."""
        orch.sm.force_transition(RobotState.ACTIVE, orch.ctx)
        orch._tick()
        orch.on_shutdown()  # Should not raise


# ════════════════════════════════════════════════════════════════
# Integration: tick + operations
# ════════════════════════════════════════════════════════════════


class TestTickIntegration:
    def test_boot_to_homing_via_tick(self, orch):
        """Heartbeats + firmware validated + first /link_status → HOMING."""
        orch._tick()  # Enter BOOT
        assert orch.sm.state == RobotState.BOOT

        # Feed heartbeats + firmware validation + a clean /link_status
        # (A5: BOOT waits for the bridge's first report before exiting).
        msg = _make_robot_state_msg(all_heartbeats=True, firmware_validated=True)
        orch._on_robot_state(msg)
        orch._on_link_status(_link_status('NONE'))
        orch._tick()
        assert orch.sm.state == RobotState.HOMING

    def test_error_cancels_pending_and_goes_to_fault(self, orch):
        """Errors during HOMING cancel pending ops and force FAULT."""
        orch._tick()  # Enter BOOT
        # Get to HOMING
        msg = _make_robot_state_msg(all_heartbeats=True, firmware_validated=True)
        orch._on_robot_state(msg)
        orch._on_link_status(_link_status('NONE'))
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
        orch._on_link_status(_link_status('NONE'))
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


# ════════════════════════════════════════════════════════════════
# FIX 2 — Teensy guard awareness: a latched guard forces FAULT from ACTIVE and
# routes the existing clear_errors recovery, WITHOUT wedging the benign
# prior-session MPC_STALE latch at BOOT. See the 2026-07-10 blind-orchestrator
# incident (the guard suppresses leg output without disarming, so it never
# reaches /robot_state.error).
# ════════════════════════════════════════════════════════════════

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def _link_status(fault_state='NONE'):
    msg = DiagnosticStatus()
    msg.values = [KeyValue(key='fault_state', value=fault_state),
                  KeyValue(key='mpc_active', value='1')]
    return msg


class TestGuardLatch:
    def test_on_link_status_sets_and_clears_guard_latched(self, orch):
        orch._on_link_status(_link_status('MAX_DEVIATION'))
        assert orch.ctx.guard_latched is True
        orch._on_link_status(_link_status('NONE'))
        assert orch.ctx.guard_latched is False

    def test_unknown_fault_state_is_not_latched(self, orch):
        # 'UNKNOWN' (no heartbeat yet) must not be read as a latch.
        orch._on_link_status(_link_status('UNKNOWN'))
        assert orch.ctx.guard_latched is False

    def test_guard_latch_forces_fault_from_active(self, orch):
        orch.sm.force_transition(RobotState.ACTIVE, orch.ctx)
        orch._tick()  # settle into ACTIVE
        assert orch.sm.state == RobotState.ACTIVE
        orch._on_link_status(_link_status('MAX_DEVIATION'))
        orch._tick()
        assert orch.sm.state == RobotState.FAULT

    def test_guard_latch_does_not_fault_from_boot(self, orch):
        """Benign case: a prior-session MPC_STALE latch at BOOT must NOT FAULT the
        machine (that latch is caught by the ACTIVATE arming pre-check)."""
        orch._tick()  # BOOT
        orch._on_link_status(_link_status('MPC_STALE'))
        orch._tick()
        assert orch.sm.state == RobotState.BOOT

    def test_guard_latch_at_boot_is_cleared_by_preflight_then_proceeds(self, orch):
        """ARMING_CONTRACT A5 — a stale prior-session latch at BOOT is CLEARED
        (disarmed, one shot) instead of being carried into HOMING, where the
        firmware's guard-gated HOME verb would refuse it with ERR_BUS_DOWN (the
        2026-07-15 wedge, twice). BOOT holds until the latch drops, then proceeds."""
        spy = MagicMock()
        spy.service_is_ready.return_value = True
        spy.call_async.return_value = MockFuture()
        orch._odrive_cmd_client = spy

        orch._tick()  # BOOT
        orch._on_link_status(_link_status('MPC_STALE'))
        msg = _make_robot_state_msg(all_heartbeats=True, firmware_validated=True)
        orch._on_robot_state(msg)
        orch._tick()
        # Still BOOT — the pre-flight dispatched clear_errors instead of exiting.
        assert orch.sm.state == RobotState.BOOT
        spy.call_async.assert_called_once()
        assert spy.call_async.call_args[0][0].command == 'clear_errors'
        # The bridge clears the latch; /link_status reflects it → BOOT proceeds.
        orch._on_link_status(_link_status('NONE'))
        orch._tick()
        assert orch.sm.state == RobotState.HOMING

    def test_guard_fault_recovery_routes_clear_errors_then_exits(self, orch):
        """From an ACTIVE guard-forced FAULT, a clear_errors command routes
        odrive_command(clear_errors); once the guard clears, FAULT exits."""
        # Spy on the odrive_command client to assert the clear_errors routing.
        spy = MagicMock()
        spy.service_is_ready.return_value = True
        spy.call_async.return_value = MockFuture()
        orch._odrive_cmd_client = spy

        orch.sm.force_transition(RobotState.ACTIVE, orch.ctx)
        orch._tick()
        orch._on_link_status(_link_status('MAX_DEVIATION'))
        orch._tick()
        assert orch.sm.state == RobotState.FAULT
        # Persists while latched.
        orch._tick()
        assert orch.sm.state == RobotState.FAULT

        # Operator (or GUI) sends clear_errors → routed to odrive_command.
        orch._on_command(MockString(data='clear_errors'))
        orch._tick()
        spy.call_async.assert_called_once()
        assert spy.call_async.call_args[0][0].command == 'clear_errors'

        # Teensy latch released → /link_status back to NONE → FAULT exits.
        orch._on_link_status(_link_status('NONE'))
        orch._tick()
        assert orch.sm.state != RobotState.FAULT
        # F1 — a guard-only fault exits back to ACTIVE (the legs never disarmed), not
        # to BOOT. The transition fires this tick; ActiveHandler.on_enter runs next.
        assert orch.sm.state == RobotState.ACTIVE

    def test_guard_fault_exit_resumes_active_without_rearming(self, orch):
        """F1 — after a guard-only fault clears, re-entering ACTIVE must RESUME
        already-armed (no 'activate' request): the legs never disarmed, and re-running
        the ACTIVATE move would fight the live stream. Drive the full cycle and assert
        the resume path (no activate service call on the ACTIVE re-entry)."""
        spy_activate = MagicMock()
        spy_activate.service_is_ready.return_value = True
        spy_activate.call_async.return_value = MockFuture()
        orch._activate_client = spy_activate

        orch.sm.force_transition(RobotState.ACTIVE, orch.ctx)
        orch._tick()
        orch._on_link_status(_link_status('MAX_DEVIATION'))
        orch._tick()
        assert orch.sm.state == RobotState.FAULT

        orch._on_link_status(_link_status('NONE'))
        orch._tick()                       # FAULT.execute → return ACTIVE (resume armed)
        assert orch.sm.state == RobotState.ACTIVE
        calls_before = spy_activate.call_async.call_count
        orch._tick()                       # ActiveHandler.on_enter (resume) runs here
        orch._tick()
        assert orch.sm.state == RobotState.ACTIVE
        # No ACTIVATE service call was issued on the resume — the legs stayed armed.
        assert spy_activate.call_async.call_count == calls_before

    def test_guard_forced_fault_keeps_trajectory_streaming_and_frozen(self, orch):
        """(iv) Cross-node choreography, as close as the mocked-ROS harness allows:
        instantiate BOTH the orchestrator and a trajectory_node, hand-deliver messages
        between them, and assert that a guard-forced FAULT never publishes a mode that
        would silence trajectory_node's emitter — _streaming AND _guard_frozen both stay
        True THROUGH the fault. This is the exact 2026-07-10 deadlock the F1 fix closes:
        the old 'ERROR' publish dropped trajectory_node out of the streaming set."""
        import jugglebot.hardware_config as hw
        from std_msgs.msg import String
        from jugglebot.trajectory_node import TrajectoryNode

        activate_rev = list(hw.JB_OP_ACTIVATE_POSITION_REVS)

        def _traj_robot_state():
            msg = RobotStateMsg()
            msg.motor_states = [MotorStateSingle(pos_estimate=float(activate_rev[i]))
                                for i in range(6)] + [MotorStateSingle()]
            msg.is_homed = True   # A5 seed gate: no seeding before homing
            return msg

        traj = TrajectoryNode(start_emitter=False)
        # Bring trajectory_node to streaming + seeded in a streaming mode (STANDBY).
        traj._on_robot_state(_traj_robot_state())
        traj._on_control_mode(String(data='STANDBY'))
        assert traj._streaming is True and traj._seeded is True

        # Orchestrator into ACTIVE (STANDBY armed hold — matches trajectory_node's mode).
        orch.sm.force_transition(RobotState.ACTIVE, orch.ctx)
        orch._tick()
        assert orch.sm.state == RobotState.ACTIVE

        # The Teensy guard latches — deliver /link_status to BOTH nodes.
        orch._on_link_status(_link_status('MAX_DEVIATION'))
        traj._on_link_status(_link_status('MAX_DEVIATION'))
        assert traj._guard_frozen is True

        # Orchestrator tick forces the guard-only FAULT and publishes control_mode.
        before = len(orch._control_mode_pub.published)
        orch._tick()
        assert orch.sm.state == RobotState.FAULT
        faulted_modes = [m.data for m in orch._control_mode_pub.published[before:]]
        assert faulted_modes, 'orchestrator published no control_mode during the fault'

        # Every mode published during the guard-forced FAULT must be a STREAMING mode
        # (never 'ERROR'). Hand-deliver each to trajectory_node and confirm it stays
        # streaming AND guard-frozen — the emitter never goes silent, the descent lives.
        for mode in faulted_modes:
            assert mode != 'ERROR'
            assert mode in traj._stream_modes, (
                f'orchestrator published {mode!r} — outside trajectory_node\'s '
                f'streaming set, which would silence the emitter')
            traj._on_control_mode(String(data=mode))
            assert traj._streaming is True
            assert traj._guard_frozen is True


class TestReloadRelay:
    """The GUI reaches the jugglebot/reload ACTION only through this Trigger
    relay (rosbridge on Foxy has no action transport). The relay dispatches ONE
    Reload goal fire-and-forget with fixed values; the reload coordinator owns
    preconditions + the outcome. See orchestrator_node._svc_reload_request."""

    def test_reload_request_service_registered_as_trigger(self, orch):
        assert 'jugglebot/reload_request' in orch._services
        assert orch._services['jugglebot/reload_request'].srv_type is Trigger

    def test_reload_action_client_created(self, orch):
        # Relays to the ACTION named jugglebot/reload (NOT the request service).
        assert 'jugglebot/reload' in orch._action_clients

    def test_dispatch_acks_when_server_ready(self, orch):
        orch._reload_client._server_ready = True
        res = orch._svc_reload_request(Trigger.Request(), Trigger.Response())
        assert res.success is True
        assert 'dispatch' in res.message.lower()

    def test_dispatch_fails_when_server_not_ready(self, orch):
        orch._reload_client._server_ready = False
        res = orch._svc_reload_request(Trigger.Request(), Trigger.Response())
        assert res.success is False
        assert 'unavailable' in res.message.lower()

    def test_goal_carries_fixed_params(self, orch):
        """throw_delay_s 3.0, catch_vel_scale 0.0 (0 => coordinator default; the
        numeric default is NEVER encoded here or in the GUI)."""
        captured = {}

        def _spy_send(goal):
            captured['goal'] = goal
            return MockFuture()

        orch._reload_client._server_ready = True
        orch._reload_client.send_goal_async = _spy_send
        orch._svc_reload_request(Trigger.Request(), Trigger.Response())
        goal = captured['goal']
        assert goal.throw_delay_s == 3.0
        assert goal.catch_vel_scale == 0.0

    def test_not_ready_dispatches_nothing(self, orch):
        """A not-ready server must not send a goal (no half-fired reload)."""
        sent = []
        orch._reload_client._server_ready = False
        orch._reload_client.send_goal_async = lambda goal: sent.append(goal)
        orch._svc_reload_request(Trigger.Request(), Trigger.Response())
        assert sent == []

    def test_goal_response_rejected_is_safe(self, orch):
        goal_future = MockFuture()
        goal_handle = MagicMock()
        goal_handle.accepted = False
        goal_future.set_result(goal_handle)
        orch._on_reload_goal_response(goal_future)   # must not raise
        goal_handle.get_result_async.assert_not_called()

    def test_goal_response_accepted_chains_result(self, orch):
        goal_future = MockFuture()
        goal_handle = MagicMock()
        goal_handle.accepted = True
        goal_handle.get_result_async.return_value = MockFuture()
        goal_future.set_result(goal_handle)
        orch._on_reload_goal_response(goal_future)
        goal_handle.get_result_async.assert_called_once()

    def test_goal_response_exception_is_safe(self, orch):
        goal_future = MockFuture()
        goal_future.set_exception(RuntimeError('goal send blew up'))
        orch._on_reload_goal_response(goal_future)    # must not raise

    def test_result_callback_logs_outcome_safely(self, orch):
        result_future = MockFuture()
        wrapper = MagicMock()
        wrapper.result.success = True
        wrapper.result.outcome = 'CAUGHT'
        wrapper.result.catch_error_mm = 12.0
        result_future.set_result(wrapper)
        orch._on_reload_result(result_future)         # must not raise

    def test_result_callback_exception_is_safe(self, orch):
        result_future = MockFuture()
        result_future.set_exception(RuntimeError('result blew up'))
        orch._on_reload_result(result_future)         # must not raise

    def test_reload_request_never_touches_state_machine(self, orch):
        """The relay is a side-channel: it must NOT enqueue a command into the SM
        queue (that path is for lifecycle transitions, not the reload)."""
        orch._reload_client._server_ready = True
        orch._svc_reload_request(Trigger.Request(), Trigger.Response())
        assert orch.ctx.consume_command() is None


class TestFireForgetDisarm:
    def test_disarm_does_not_clobber_pending_operation(self, orch):
        """Review 2026-07-15: the natural ACTIVE→FAULT path dispatches the
        multi-second 'deactivate' one tick before FaultHandler requests the
        disarm. The disarm must ride OUTSIDE the single-slot operation tracking
        — overwriting _pending_future would open IdleHandler's
        wait-for-deactivate gate while the platform is still descending, and a
        failed descent's result would be silently dropped."""
        deactivate_future = MockFuture()
        spy_activate = MagicMock()
        spy_activate.service_is_ready.return_value = True
        spy_activate.call_async.return_value = deactivate_future
        orch._activate_client = spy_activate

        orch._dispatch_request('deactivate')
        assert orch._pending_future is deactivate_future
        assert orch.ctx.operation_pending is True

        orch._dispatch_request('disarm_setpoints')
        # The tracked slot still holds the deactivate; pending still gates IDLE.
        assert orch._pending_future is deactivate_future
        assert orch.ctx.operation_pending is True

    def test_arm_uses_tracked_slot(self, orch):
        """The ARM stays tracked — ActiveHandler consumes its result."""
        orch._dispatch_request('arm_setpoints')
        assert orch._pending_future is not None
        assert orch.ctx.operation_pending is True


# ════════════════════════════════════════════════════════════════
# F3/C4 — say WHY on entry to FAULT
#
# '[SM] Entering FAULT' told an operator nothing about the cause. The strings
# that FORCED the transition live in ctx.errors (copied wholesale from
# robot_state.error[] on every 100 Hz publish) and were visible only to whoever
# thought to echo the topic — which, on this Foxy box, is unreliable for a
# high-rate RELIABLE topic anyway. With F3 those strings now carry the decoded
# per-axis ODrive names, so logging them once per FAULT visit puts the actual
# cause in the launch shell beside the transition.
# ════════════════════════════════════════════════════════════════


class TestFaultEntryCauseLogging:
    def _warnings(self, orch):
        return [str(c[0][0]) for c in orch._logger.warning.call_args_list]

    def test_fault_entry_logs_the_error_strings(self, orch):
        orch._tick()  # BOOT
        orch._on_robot_state(_make_robot_state_msg(
            all_heartbeats=True,
            errors=['Fatal ODrive issue (Teensy fault_state=ODRIVE_FATAL).',
                    'ODrive leg 0: active=[] disarm=[SPINOUT_DETECTED] 0x0/0x4000000'],
            has_fatal_odrive_error=True))
        orch._logger.warning = MagicMock()
        orch._tick()
        assert orch.sm.state == RobotState.FAULT
        causes = [m for m in self._warnings(orch) if 'FAULT cause' in m]
        assert len(causes) == 1, self._warnings(orch)
        # The decoded per-axis name — the whole point of F3 — reaches the shell.
        assert 'SPINOUT_DETECTED' in causes[0]
        assert 'ODRIVE_FATAL' in causes[0]

    def test_fault_cause_logged_once_per_visit_not_every_tick(self, orch):
        """FAULT is held, and _tick runs at 10 Hz — an unguarded log would spam
        the shell until the operator cleared it, burying the one line that
        matters."""
        orch._tick()
        orch._on_robot_state(_make_robot_state_msg(
            all_heartbeats=True, errors=['boom'], has_fatal_odrive_error=True))
        orch._logger.warning = MagicMock()
        orch._tick()
        orch._tick()
        orch._tick()
        assert orch.sm.state == RobotState.FAULT
        assert len([m for m in self._warnings(orch) if 'FAULT cause' in m]) == 1

    def test_guard_only_fault_logs_an_honest_no_strings_line(self, orch):
        """A Teensy guard latch suppresses leg output WITHOUT disarming, so it
        never reaches robot_state.error[]. The line must say so rather than
        printing an empty list that reads as 'no cause found'."""
        orch.sm.force_transition(RobotState.ACTIVE, orch.ctx)
        orch.ctx.active_mode = ActiveMode.TRAJECTORY
        orch._tick()
        orch._on_link_status(_link_status('MAX_DEVIATION'))
        orch._logger.warning = MagicMock()
        orch._tick()
        assert orch.sm.state == RobotState.FAULT
        causes = [m for m in self._warnings(orch) if 'FAULT cause' in m]
        assert len(causes) == 1, self._warnings(orch)
        assert 'no robot_state.error[] strings' in causes[0]
        assert 'guard_latched=True' in causes[0]
