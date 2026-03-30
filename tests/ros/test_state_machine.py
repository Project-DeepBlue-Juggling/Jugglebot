"""Tests for jugglebot.state_machine — pure Python state machine.

No ROS2 dependency: all tests exercise the state machine directly via tick().
"""

import time
from unittest.mock import MagicMock, patch

import pytest

from jugglebot.state_machine import (
    ActiveMode,
    BOOT_TIMEOUT_S,
    BootHandler,
    Context,
    ErrorSeverity,
    FaultHandler,
    HomingHandler,
    IdleHandler,
    ActiveHandler,
    RobotState,
    StateHandler,
    StateMachine,
    build_default_machine,
)


# ════════════════════════════════════════════════════════════════
# RobotState enum
# ════════════════════════════════════════════════════════════════


class TestRobotStateEnum:
    def test_all_states_present(self):
        names = {s.name for s in RobotState}
        assert names >= {'BOOT', 'HOMING', 'IDLE', 'ACTIVE', 'FAULT'}

    def test_values_unique(self):
        values = [s.value for s in RobotState]
        assert len(values) == len(set(values))


# ════════════════════════════════════════════════════════════════
# ActiveMode enum
# ════════════════════════════════════════════════════════════════


class TestActiveModeEnum:
    def test_spacemouse_value(self):
        assert ActiveMode.SPACEMOUSE.value == 'SPACEMOUSE'

    def test_shell_value(self):
        assert ActiveMode.SHELL.value == 'SHELL'

    def test_from_uppercase_string(self):
        assert ActiveMode('SPACEMOUSE') == ActiveMode.SPACEMOUSE
        assert ActiveMode('SHELL') == ActiveMode.SHELL


# ════════════════════════════════════════════════════════════════
# Context
# ════════════════════════════════════════════════════════════════


class TestContext:
    def test_initial_values(self):
        ctx = Context()
        assert ctx.all_heartbeats is False
        assert ctx.encoder_search_complete is False
        assert ctx.is_homed is False
        assert ctx.errors == []
        assert ctx.fatal_error is False
        assert ctx.fatal_can_error is False
        assert ctx.undervoltage is False
        assert ctx.operation_pending is False
        assert ctx.operation_result is None
        assert ctx.consume_command() is None
        assert ctx.request is None
        assert ctx.control_mode is None
        assert ctx.active_mode == ActiveMode.SPACEMOUSE
        assert ctx.error_severity is None
        assert ctx.boot_timed_out is False

    def test_has_fatal_error_both_false(self):
        ctx = Context()
        assert ctx.has_fatal_error is False

    def test_has_fatal_error_odrive_true(self):
        ctx = Context()
        ctx.fatal_error = True
        assert ctx.has_fatal_error is True

    def test_has_fatal_error_can_true(self):
        ctx = Context()
        ctx.fatal_can_error = True
        assert ctx.has_fatal_error is True

    def test_has_fatal_error_both_true(self):
        ctx = Context()
        ctx.fatal_error = True
        ctx.fatal_can_error = True
        assert ctx.has_fatal_error is True

    def test_enqueue_and_consume_command(self):
        ctx = Context()
        ctx.enqueue_command('activate')
        assert ctx.consume_command() == 'activate'
        assert ctx.consume_command() is None

    def test_consume_command_none_when_empty(self):
        ctx = Context()
        assert ctx.consume_command() is None

    def test_consume_command_idempotent(self):
        ctx = Context()
        ctx.enqueue_command('home')
        ctx.consume_command()
        assert ctx.consume_command() is None

    def test_command_queue_fifo(self):
        ctx = Context()
        ctx.enqueue_command('activate')
        ctx.enqueue_command('deactivate')
        assert ctx.consume_command() == 'activate'
        assert ctx.consume_command() == 'deactivate'

    def test_clear_commands(self):
        ctx = Context()
        ctx.enqueue_command('activate')
        ctx.clear_commands()
        assert ctx.consume_command() is None

    def test_request_queue(self):
        ctx = Context()
        ctx.request = 'encoder_search'
        assert ctx.request == 'encoder_search'
        items = ctx.drain_requests()
        assert items == ['encoder_search']
        assert ctx.request is None


# ════════════════════════════════════════════════════════════════
# StateHandler base class
# ════════════════════════════════════════════════════════════════


class TestStateHandler:
    def test_default_on_enter_noop(self):
        handler = StateHandler()
        handler.on_enter(Context())  # Should not raise

    def test_default_execute_returns_none(self):
        handler = StateHandler()
        assert handler.execute(Context()) is None

    def test_default_on_exit_noop(self):
        handler = StateHandler()
        handler.on_exit(Context())  # Should not raise


# ════════════════════════════════════════════════════════════════
# StateMachine core
# ════════════════════════════════════════════════════════════════


class TestStateMachine:
    def test_initial_state(self):
        sm = StateMachine(RobotState.BOOT)
        assert sm.state == RobotState.BOOT

    def test_tick_calls_on_enter_once(self):
        handler = MagicMock(spec=StateHandler)
        handler.execute.return_value = None
        sm = StateMachine(RobotState.BOOT)
        sm.register(RobotState.BOOT, handler)
        ctx = Context()
        sm.tick(ctx)
        sm.tick(ctx)
        handler.on_enter.assert_called_once()

    def test_tick_calls_execute_every_tick(self):
        handler = MagicMock(spec=StateHandler)
        handler.execute.return_value = None
        sm = StateMachine(RobotState.BOOT)
        sm.register(RobotState.BOOT, handler)
        ctx = Context()
        sm.tick(ctx)
        sm.tick(ctx)
        sm.tick(ctx)
        assert handler.execute.call_count == 3

    def test_transition_calls_exit_and_enter(self):
        boot_handler = MagicMock(spec=StateHandler)
        boot_handler.execute.return_value = RobotState.HOMING
        homing_handler = MagicMock(spec=StateHandler)
        homing_handler.execute.return_value = None

        sm = StateMachine(RobotState.BOOT)
        sm.register(RobotState.BOOT, boot_handler)
        sm.register(RobotState.HOMING, homing_handler)
        ctx = Context()

        sm.tick(ctx)  # Enter BOOT, execute → transition to HOMING
        assert sm.state == RobotState.HOMING
        boot_handler.on_enter.assert_called_once()
        boot_handler.on_exit.assert_called_once()

        sm.tick(ctx)  # Enter HOMING, execute → stay
        homing_handler.on_enter.assert_called_once()

    def test_no_transition_on_same_state(self):
        handler = MagicMock(spec=StateHandler)
        handler.execute.return_value = RobotState.BOOT  # Return same state
        sm = StateMachine(RobotState.BOOT)
        sm.register(RobotState.BOOT, handler)
        ctx = Context()

        sm.tick(ctx)
        sm.tick(ctx)
        # on_exit should NOT be called (no transition)
        handler.on_exit.assert_not_called()

    def test_no_handler_logs_warning(self):
        log = MagicMock()
        sm = StateMachine(RobotState.BOOT, log_fn=log)
        ctx = Context()
        sm.tick(ctx)
        log.assert_called()
        assert 'No handler' in log.call_args[0][0]

    def test_force_transition_calls_on_exit(self):
        handler = MagicMock(spec=StateHandler)
        handler.execute.return_value = None
        sm = StateMachine(RobotState.BOOT)
        sm.register(RobotState.BOOT, handler)
        sm.register(RobotState.FAULT, MagicMock(spec=StateHandler))
        ctx = Context()

        sm.tick(ctx)  # Enter BOOT
        sm.force_transition(RobotState.FAULT, ctx)
        assert sm.state == RobotState.FAULT
        handler.on_exit.assert_called_once()

    def test_force_transition_same_state_noop(self):
        handler = MagicMock(spec=StateHandler)
        handler.execute.return_value = None
        sm = StateMachine(RobotState.BOOT)
        sm.register(RobotState.BOOT, handler)
        ctx = Context()

        sm.tick(ctx)  # Enter BOOT
        sm.force_transition(RobotState.BOOT, ctx)
        handler.on_exit.assert_not_called()

    def test_force_transition_before_enter_no_exit_call(self):
        """Force transition before first tick — on_exit not called since never entered."""
        handler = MagicMock(spec=StateHandler)
        sm = StateMachine(RobotState.BOOT)
        sm.register(RobotState.BOOT, handler)
        sm.register(RobotState.FAULT, MagicMock(spec=StateHandler))
        ctx = Context()

        sm.force_transition(RobotState.FAULT, ctx)
        handler.on_exit.assert_not_called()

    def test_default_log_fn(self):
        """Default log function is print-based (no crash)."""
        sm = StateMachine(RobotState.BOOT)
        sm.register(RobotState.BOOT, MagicMock(spec=StateHandler))
        ctx = Context()
        sm.tick(ctx)  # Should not raise


# ════════════════════════════════════════════════════════════════
# BootHandler
# ════════════════════════════════════════════════════════════════


class TestBootHandler:
    def test_enters_and_stays_without_heartbeats(self):
        handler = BootHandler()
        ctx = Context()
        handler.on_enter(ctx)
        assert ctx.control_mode == ''
        assert ctx.boot_timed_out is False
        assert handler.execute(ctx) is None

    def test_transitions_to_homing_on_heartbeats(self):
        handler = BootHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        assert handler.execute(ctx) == RobotState.HOMING

    def test_transitions_to_idle_if_already_homed(self):
        handler = BootHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.is_homed = True
        assert handler.execute(ctx) == RobotState.IDLE

    def test_stays_while_firmware_not_validated(self):
        """Heartbeats arrived but firmware not yet validated — stay in BOOT."""
        handler = BootHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.firmware_validated = False
        assert handler.execute(ctx) is None

    @patch('jugglebot.state_machine.time')
    def test_timeout_transitions_to_fault(self, mock_time):
        handler = BootHandler()
        ctx = Context()
        mock_time.time.return_value = 100.0
        handler.on_enter(ctx)
        # Advance past timeout
        mock_time.time.return_value = 100.0 + BOOT_TIMEOUT_S + 1.0
        result = handler.execute(ctx)
        assert result == RobotState.FAULT
        assert ctx.boot_timed_out is True

    @patch('jugglebot.state_machine.time')
    def test_no_timeout_within_limit(self, mock_time):
        handler = BootHandler()
        ctx = Context()
        mock_time.time.return_value = 100.0
        handler.on_enter(ctx)
        mock_time.time.return_value = 100.0 + BOOT_TIMEOUT_S - 1.0
        result = handler.execute(ctx)
        assert result is None
        assert ctx.boot_timed_out is False

    def test_consumes_commands_during_boot(self):
        handler = BootHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.enqueue_command('activate')
        handler.execute(ctx)
        assert ctx.consume_command() is None  # Command was cleared

    def test_resets_boot_timed_out_on_enter(self):
        handler = BootHandler()
        ctx = Context()
        ctx.boot_timed_out = True
        handler.on_enter(ctx)
        assert ctx.boot_timed_out is False


# ════════════════════════════════════════════════════════════════
# HomingHandler
# ════════════════════════════════════════════════════════════════


class TestHomingHandler:
    def test_starts_with_encoder_search(self):
        handler = HomingHandler()
        ctx = Context()
        handler.on_enter(ctx)
        assert handler._phase == 'encoder_search'
        assert ctx.request == 'encoder_search'

    def test_skips_encoder_search_if_complete(self):
        handler = HomingHandler()
        ctx = Context()
        ctx.encoder_search_complete = True
        handler.on_enter(ctx)
        assert handler._phase == 'home'
        assert ctx.request == 'home'

    def test_stays_while_operation_pending(self):
        handler = HomingHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.operation_result = None
        assert handler.execute(ctx) is None

    def test_transitions_to_fault_on_failure(self):
        handler = HomingHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.operation_result = False
        assert handler.execute(ctx) == RobotState.FAULT

    def test_advances_encoder_to_home(self):
        handler = HomingHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()  # Simulate orchestrator processing 'encoder_search'
        # Encoder search succeeds
        ctx.operation_result = True
        assert handler.execute(ctx) is None
        assert handler._phase == 'home'
        assert ctx.request == 'home'

    def test_advances_home_to_bb_calibrate(self):
        handler = HomingHandler()
        ctx = Context()
        ctx.encoder_search_complete = True  # Skip encoder search
        handler.on_enter(ctx)
        ctx.drain_requests()  # Simulate orchestrator processing 'home'
        # Home succeeds
        ctx.operation_result = True
        assert handler.execute(ctx) is None
        assert handler._phase == 'bb_calibrate'
        assert ctx.request == 'bb_calibrate'

    def test_advances_bb_calibrate_to_idle(self):
        handler = HomingHandler()
        ctx = Context()
        ctx.encoder_search_complete = True
        handler.on_enter(ctx)
        ctx.drain_requests()
        # Home succeeds
        ctx.operation_result = True
        handler.execute(ctx)
        ctx.drain_requests()
        # BB calibrate succeeds
        ctx.operation_result = True
        result = handler.execute(ctx)
        assert result == RobotState.IDLE

    def test_full_three_phase_sequence(self):
        """Run all three phases: encoder_search → home → bb_calibrate → IDLE."""
        handler = HomingHandler()
        ctx = Context()
        handler.on_enter(ctx)
        assert ctx.request == 'encoder_search'
        ctx.drain_requests()

        # Encoder search completes
        ctx.operation_result = True
        assert handler.execute(ctx) is None
        assert ctx.request == 'home'
        ctx.drain_requests()

        # Home completes
        ctx.operation_result = True
        assert handler.execute(ctx) is None
        assert ctx.request == 'bb_calibrate'
        ctx.drain_requests()

        # BB calibrate completes
        ctx.operation_result = True
        assert handler.execute(ctx) == RobotState.IDLE

    def test_operation_result_reset_between_phases(self):
        handler = HomingHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()
        ctx.operation_result = True
        handler.execute(ctx)  # → home
        assert ctx.operation_result is None

    def test_consumes_commands_during_homing(self):
        handler = HomingHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.enqueue_command('activate')
        handler.execute(ctx)
        assert ctx.consume_command() is None  # Command was cleared

    def test_on_exit_clears_request(self):
        handler = HomingHandler()
        ctx = Context()
        ctx.request = 'home'
        handler.on_exit(ctx)
        assert ctx.request is None

    def test_failure_at_home_phase(self):
        """Failure during home phase → FAULT."""
        handler = HomingHandler()
        ctx = Context()
        ctx.encoder_search_complete = True
        handler.on_enter(ctx)
        ctx.operation_result = False
        assert handler.execute(ctx) == RobotState.FAULT


# ════════════════════════════════════════════════════════════════
# IdleHandler
# ════════════════════════════════════════════════════════════════


class TestIdleHandler:
    def test_sets_empty_control_mode_on_enter(self):
        handler = IdleHandler()
        ctx = Context()
        handler.on_enter(ctx)
        assert ctx.control_mode == ''

    def test_activate_command_transitions_to_active(self):
        handler = IdleHandler()
        ctx = Context()
        ctx.enqueue_command('activate')
        assert handler.execute(ctx) == RobotState.ACTIVE

    def test_home_command_transitions_to_homing(self):
        handler = IdleHandler()
        ctx = Context()
        ctx.enqueue_command('home')
        assert handler.execute(ctx) == RobotState.HOMING

    def test_no_command_stays(self):
        handler = IdleHandler()
        ctx = Context()
        assert handler.execute(ctx) is None

    def test_unknown_command_stays(self):
        handler = IdleHandler()
        ctx = Context()
        ctx.enqueue_command('unknown_cmd')
        assert handler.execute(ctx) is None
        assert ctx.consume_command() is None  # Consumed by execute

    def test_operation_pending_blocks_commands(self):
        handler = IdleHandler()
        ctx = Context()
        ctx.operation_pending = True
        ctx.enqueue_command('activate')
        assert handler.execute(ctx) is None
        # Command was NOT consumed because pending op blocks
        assert ctx.consume_command() == 'activate'

    def test_deactivate_in_idle_ignored(self):
        handler = IdleHandler()
        ctx = Context()
        ctx.enqueue_command('deactivate')
        assert handler.execute(ctx) is None


# ════════════════════════════════════════════════════════════════
# ActiveHandler
# ════════════════════════════════════════════════════════════════


class TestActiveHandler:
    def test_requests_activate_on_enter(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        assert ctx.request == 'activate'
        assert ctx.operation_result is None
        assert handler._activated is False

    def test_waits_for_activation_result(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.operation_result = None
        assert handler.execute(ctx) is None

    def test_activation_failure_goes_to_fault(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.operation_result = False
        assert handler.execute(ctx) == RobotState.FAULT

    def test_activation_success_sets_control_mode(self):
        handler = ActiveHandler()
        ctx = Context()
        ctx.active_mode = ActiveMode.SPACEMOUSE
        handler.on_enter(ctx)
        ctx.operation_result = True
        assert handler.execute(ctx) is None
        assert handler._activated is True
        assert ctx.control_mode == 'SPACEMOUSE'

    def test_deactivate_command_returns_idle(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.operation_result = True
        handler.execute(ctx)  # Complete activation
        ctx.enqueue_command('deactivate')
        assert handler.execute(ctx) == RobotState.IDLE

    def test_spacemouse_command_switches_mode(self):
        handler = ActiveHandler()
        ctx = Context()
        ctx.active_mode = ActiveMode.SHELL
        handler.on_enter(ctx)
        ctx.operation_result = True
        handler.execute(ctx)  # Complete activation
        ctx.enqueue_command('spacemouse')
        assert handler.execute(ctx) is None
        assert ctx.active_mode == ActiveMode.SPACEMOUSE
        assert ctx.control_mode == 'SPACEMOUSE'

    def test_shell_command_switches_mode(self):
        handler = ActiveHandler()
        ctx = Context()
        ctx.active_mode = ActiveMode.SPACEMOUSE
        handler.on_enter(ctx)
        ctx.operation_result = True
        handler.execute(ctx)  # Complete activation
        ctx.enqueue_command('shell')
        assert handler.execute(ctx) is None
        assert ctx.active_mode == ActiveMode.SHELL
        assert ctx.control_mode == 'SHELL'

    def test_unknown_command_after_activation_stays(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.operation_result = True
        handler.execute(ctx)
        ctx.enqueue_command('unknown')
        assert handler.execute(ctx) is None

    def test_on_exit_requests_deactivate_and_clears_mode(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_exit(ctx)
        assert ctx.request == 'deactivate'
        assert ctx.control_mode == ''

    def test_mode_switch_uses_uppercase(self):
        """'spacemouse' → ActiveMode.SPACEMOUSE (uppercase conversion)."""
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.operation_result = True
        handler.execute(ctx)
        ctx.enqueue_command('spacemouse')
        handler.execute(ctx)
        assert ctx.active_mode == ActiveMode.SPACEMOUSE


# ════════════════════════════════════════════════════════════════
# FaultHandler
# ════════════════════════════════════════════════════════════════


class TestFaultHandler:
    def test_sets_error_control_mode_on_enter(self):
        handler = FaultHandler()
        ctx = Context()
        handler.on_enter(ctx)
        assert ctx.control_mode == 'ERROR'

    def test_boot_timeout_severity_fatal(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.boot_timed_out = True
        handler.on_enter(ctx)
        assert ctx.error_severity == ErrorSeverity.FATAL

    def test_fatal_odrive_error_severity(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.fatal_error = True
        handler.on_enter(ctx)
        assert ctx.error_severity == ErrorSeverity.FATAL

    def test_fatal_can_error_severity(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.fatal_can_error = True
        handler.on_enter(ctx)
        assert ctx.error_severity == ErrorSeverity.FATAL

    def test_undervoltage_severity_transient(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.undervoltage = True
        handler.on_enter(ctx)
        assert ctx.error_severity == ErrorSeverity.TRANSIENT

    def test_fatal_overrides_undervoltage(self):
        """Fatal + undervoltage → FATAL (fatal takes precedence)."""
        handler = FaultHandler()
        ctx = Context()
        ctx.fatal_error = True
        ctx.undervoltage = True
        handler.on_enter(ctx)
        assert ctx.error_severity == ErrorSeverity.FATAL

    def test_unknown_error_defaults_to_fatal(self):
        """No known error condition but still in FAULT → FATAL."""
        handler = FaultHandler()
        ctx = Context()
        handler.on_enter(ctx)
        assert ctx.error_severity == ErrorSeverity.FATAL

    def test_auto_recovery_when_errors_clear(self):
        """Recovery goes through BOOT to re-validate heartbeats."""
        handler = FaultHandler()
        ctx = Context()
        ctx.fatal_error = True
        handler.on_enter(ctx)
        # Errors clear
        ctx.fatal_error = False
        ctx.errors = []
        assert handler.execute(ctx) == RobotState.BOOT

    def test_stays_in_fault_with_active_errors(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.fatal_error = True
        ctx.errors = ['SOME_ERROR']
        handler.on_enter(ctx)
        assert handler.execute(ctx) is None

    def test_stays_in_fault_with_fatal_flag(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.fatal_error = True
        handler.on_enter(ctx)
        assert handler.execute(ctx) is None

    def test_stays_with_can_error(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.fatal_can_error = True
        handler.on_enter(ctx)
        assert handler.execute(ctx) is None

    def test_clear_errors_command_sets_request(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.fatal_error = True
        handler.on_enter(ctx)
        ctx.enqueue_command('clear_errors')
        handler.execute(ctx)
        assert ctx.request == 'clear_errors'

    def test_clear_errors_resets_boot_timeout(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.boot_timed_out = True
        handler.on_enter(ctx)
        ctx.enqueue_command('clear_errors')
        handler.execute(ctx)
        assert ctx.boot_timed_out is False

    def test_boot_timeout_auto_recovery_on_heartbeats(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.boot_timed_out = True
        handler.on_enter(ctx)
        # Heartbeats arrive
        ctx.all_heartbeats = True
        handler.execute(ctx)
        assert ctx.boot_timed_out is False

    def test_boot_timeout_no_recovery_without_heartbeats(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.boot_timed_out = True
        handler.on_enter(ctx)
        handler.execute(ctx)
        assert ctx.boot_timed_out is True

    def test_boot_timeout_recovery_then_exit(self):
        """Boot timeout clears when heartbeats arrive, then exits to BOOT."""
        handler = FaultHandler()
        ctx = Context()
        ctx.boot_timed_out = True
        handler.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.errors = []
        ctx.fatal_error = False
        assert handler.execute(ctx) == RobotState.BOOT

    def test_on_exit_clears_severity(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.error_severity = ErrorSeverity.FATAL
        handler.on_exit(ctx)
        assert ctx.error_severity is None

    def test_unknown_commands_discarded(self):
        handler = FaultHandler()
        ctx = Context()
        ctx.fatal_error = True
        handler.on_enter(ctx)
        ctx.enqueue_command('activate')
        handler.execute(ctx)
        assert ctx.consume_command() is None  # Consumed by execute
        assert ctx.request is None  # Not set because 'activate' is not 'clear_errors'

    def test_stays_with_only_error_strings(self):
        """Non-empty errors list but no fatal flags still stays in FAULT."""
        handler = FaultHandler()
        ctx = Context()
        ctx.errors = ['some transient error']
        handler.on_enter(ctx)
        assert handler.execute(ctx) is None


# ════════════════════════════════════════════════════════════════
# build_default_machine()
# ════════════════════════════════════════════════════════════════


class TestBuildDefaultMachine:
    def test_returns_state_machine(self):
        sm = build_default_machine()
        assert isinstance(sm, StateMachine)

    def test_initial_state_is_boot(self):
        sm = build_default_machine()
        assert sm.state == RobotState.BOOT

    def test_all_states_registered(self):
        sm = build_default_machine()
        for state in RobotState:
            assert state in sm._handlers

    def test_custom_log_fn(self):
        log = MagicMock()
        sm = build_default_machine(log_fn=log)
        ctx = Context()
        sm.tick(ctx)
        log.assert_called()

    def test_boot_handler_type(self):
        sm = build_default_machine()
        assert isinstance(sm._handlers[RobotState.BOOT], BootHandler)

    def test_homing_handler_type(self):
        sm = build_default_machine()
        assert isinstance(sm._handlers[RobotState.HOMING], HomingHandler)

    def test_idle_handler_type(self):
        sm = build_default_machine()
        assert isinstance(sm._handlers[RobotState.IDLE], IdleHandler)

    def test_active_handler_type(self):
        sm = build_default_machine()
        assert isinstance(sm._handlers[RobotState.ACTIVE], ActiveHandler)

    def test_fault_handler_type(self):
        sm = build_default_machine()
        assert isinstance(sm._handlers[RobotState.FAULT], FaultHandler)


# ════════════════════════════════════════════════════════════════
# Integration: full state flow
# ════════════════════════════════════════════════════════════════


class TestStateFlowIntegration:
    def test_boot_to_homing_to_idle(self):
        """BOOT → HOMING → IDLE with successful operations."""
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()

        # BOOT: waiting for heartbeats
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT

        # Heartbeats + firmware validated → transitions to HOMING
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        sm.tick(ctx)
        assert sm.state == RobotState.HOMING

        # Enter HOMING (on_enter sets request='encoder_search')
        sm.tick(ctx)
        assert ctx.request == 'encoder_search'
        ctx.drain_requests()

        # Encoder search completes
        ctx.operation_result = True
        sm.tick(ctx)
        assert ctx.request == 'home'
        ctx.drain_requests()

        # Home completes
        ctx.operation_result = True
        sm.tick(ctx)
        assert ctx.request == 'bb_calibrate'
        ctx.drain_requests()

        # BB calibrate completes
        ctx.operation_result = True
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE

    def test_idle_activate_deactivate_cycle(self):
        """IDLE → ACTIVE → IDLE."""
        sm = build_default_machine(log_fn=lambda msg: None)
        sm.force_transition(RobotState.IDLE, Context())

        ctx = Context()
        sm.tick(ctx)  # Enter IDLE

        # Activate
        ctx.enqueue_command('activate')
        sm.tick(ctx)
        assert sm.state == RobotState.ACTIVE

        # Activation completes
        sm.tick(ctx)  # Enter ACTIVE (requests activation)
        ctx.operation_result = True
        sm.tick(ctx)  # Activation succeeds

        # Deactivate
        ctx.enqueue_command('deactivate')
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE

    def test_error_during_active_goes_to_fault(self):
        """Force FAULT from ACTIVE when errors detected."""
        sm = build_default_machine(log_fn=lambda msg: None)
        sm.force_transition(RobotState.ACTIVE, Context())
        ctx = Context()
        sm.tick(ctx)  # Enter ACTIVE

        # Simulate error forcing FAULT
        sm.force_transition(RobotState.FAULT, ctx)
        assert sm.state == RobotState.FAULT

    def test_fault_recovery_to_boot(self):
        """FAULT → BOOT when errors clear (re-validates heartbeats)."""
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        ctx.fatal_error = True
        sm.force_transition(RobotState.FAULT, ctx)
        sm.tick(ctx)  # Enter FAULT

        # Errors clear
        ctx.fatal_error = False
        ctx.errors = []
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT

    def test_homing_failure_goes_to_fault(self):
        """HOMING → FAULT on operation failure."""
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        sm.tick(ctx)  # BOOT
        sm.tick(ctx)  # → HOMING
        assert sm.state == RobotState.HOMING

        # Enter HOMING, then encoder search fails
        sm.tick(ctx)
        ctx.operation_result = False
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT

    def test_idle_home_command_returns_to_homing(self):
        """From IDLE, 'home' command → HOMING."""
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        sm.force_transition(RobotState.IDLE, ctx)
        sm.tick(ctx)
        ctx.enqueue_command('home')
        sm.tick(ctx)
        assert sm.state == RobotState.HOMING


# ════════════════════════════════════════════════════════════════
# Custom handler registration
# ════════════════════════════════════════════════════════════════


class TestCustomHandler:
    def test_register_custom_state(self):
        """Verify the registry pattern works for extensions."""
        sm = build_default_machine(log_fn=lambda msg: None)

        class CountingHandler(StateHandler):
            def __init__(self):
                self.enter_count = 0

            def on_enter(self, ctx):
                self.enter_count += 1

        # Override the IDLE handler
        custom = CountingHandler()
        sm.register(RobotState.IDLE, custom)

        ctx = Context()
        sm.force_transition(RobotState.IDLE, ctx)
        sm.tick(ctx)
        assert custom.enter_count == 1
