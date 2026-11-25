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
    TRAJ_STATUS_STALE_S,
    activate_levelling_refusal,
    build_default_machine,
)

# BUG-130: six Levelled Active pose Leg targets (any six numbers do here).
_LEVELLED_TARGETS = [2.19, 2.257, 2.212, 2.221, 2.133, 2.098]


def _levelled(ctx):
    """A fresh trajectory/status with a loaded correction (C-LEVEL-1.O)."""
    ctx.gravity_correction_loaded = True
    ctx.traj_status_age_s = 0.1
    return ctx


def _activate_from_idle(handler, ctx, targets=_LEVELLED_TARGETS):
    """Drive IdleHandler through 'activate': refusal check, the target fetch
    (resolved as the orchestrator would), then the verdict."""
    ctx.enqueue_command('activate')
    assert handler.execute(ctx) is None
    assert ctx.drain_requests() == ['fetch_activate_targets']
    ctx.activate_leg_targets_rev = targets
    ctx.operation_result = targets is not None
    return handler.execute(ctx)


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
    def test_standby_value(self):
        assert ActiveMode.STANDBY.value == 'STANDBY'

    def test_spacemouse_value(self):
        assert ActiveMode.SPACEMOUSE.value == 'SPACEMOUSE'

    def test_trajectory_value(self):
        assert ActiveMode.TRAJECTORY.value == 'TRAJECTORY'

    def test_from_uppercase_string(self):
        assert ActiveMode('STANDBY') == ActiveMode.STANDBY
        assert ActiveMode('SPACEMOUSE') == ActiveMode.SPACEMOUSE
        assert ActiveMode('TRAJECTORY') == ActiveMode.TRAJECTORY


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
        assert ctx.active_mode == ActiveMode.STANDBY
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
        _legs(ctx, [_IDLE] * 6)   # A6: the Legs are down, Homing may start

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
        ctx.link_status_seen = True
        assert handler.execute(ctx) == RobotState.HOMING

    def test_transitions_to_idle_if_already_homed(self):
        handler = BootHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
        ctx.is_homed = True
        _legs(ctx, [_IDLE] * 6)                      # A7: at Stow, Legs Idle
        assert handler.execute(ctx) == RobotState.IDLE

    # ── ARMING_CONTRACT A5: BOOT stale-latch pre-flight ─────────

    def test_waits_for_first_link_status_before_exit(self):
        """BOOT must not conclude "no latch" from the default guard_latched=False
        before the bridge has reported at least once."""
        handler = BootHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        assert handler.execute(ctx) is None          # link_status not seen yet
        ctx.link_status_seen = True
        assert handler.execute(ctx) == RobotState.HOMING

    def test_stale_latch_cleared_then_proceeds(self):
        """A prior-session guard latch (Jetson-5V Teensy survives relaunches) is
        cleared ONCE, disarmed, at BOOT — instead of wedging HOMING at the first
        guard-gated verb (the 2026-07-15 failure, twice)."""
        handler = BootHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
        ctx.guard_latched = True
        assert handler.execute(ctx) is None
        assert ctx.drain_requests() == ['clear_errors']
        # Clear succeeds; latch still shows on /link_status for a tick or two.
        ctx.operation_result = True
        assert handler.execute(ctx) is None
        # /link_status reflects the released latch → BOOT proceeds.
        ctx.guard_latched = False
        assert handler.execute(ctx) == RobotState.HOMING

    def test_stale_latch_clear_only_requested_once(self):
        handler = BootHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
        ctx.guard_latched = True
        handler.execute(ctx)
        ctx.drain_requests()
        handler.execute(ctx)
        handler.execute(ctx)
        assert ctx.drain_requests() == []            # no re-request

    def test_stale_latch_clear_failure_goes_to_fault(self):
        handler = BootHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
        ctx.guard_latched = True
        handler.execute(ctx)
        ctx.operation_result = False
        assert handler.execute(ctx) == RobotState.FAULT

    @patch('jugglebot.state_machine.time')
    def test_stale_latch_never_drops_times_out_to_fault(self, mock_time):
        """A latch that survives a successful clear is a LIVE fault, not stale."""
        handler = BootHandler()
        ctx = Context()
        mock_time.time.return_value = 100.0
        handler.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
        ctx.guard_latched = True
        handler.execute(ctx)
        ctx.operation_result = True                  # clear "succeeded"
        mock_time.time.return_value = (
            100.0 + BootHandler.LATCH_CLEAR_TIMEOUT_S + 1.0)
        assert handler.execute(ctx) == RobotState.FAULT  # but latch never dropped

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
    def test_blanks_control_mode_only_after_pending_operation(self):
        """A4 (ARMING_CONTRACT): the streaming mode must survive until the
        deactivate RESOLVES — blanking it at IDLE entry stopped the emitter
        while the bridge could still be armed (the 250 ms SETPOINT_STALE race)."""
        handler = IdleHandler()
        ctx = Context()
        ctx.control_mode = 'STANDBY'          # left published by ActiveHandler
        ctx.operation_pending = True          # deactivate in flight
        assert handler.execute(ctx) is None
        assert ctx.control_mode == 'STANDBY'  # NOT blanked yet
        ctx.operation_pending = False
        handler.execute(ctx)
        assert ctx.control_mode == ''         # blanked after the op resolved

    def test_activate_command_transitions_to_active(self):
        handler = IdleHandler()
        ctx = _levelled(Context())
        assert _activate_from_idle(handler, ctx) == RobotState.ACTIVE
        assert ctx.activate_leg_targets_rev == _LEVELLED_TARGETS

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


class TestIdleLevelledActivate:
    """BUG-130, C-LEVEL-1.A (levelling_frame.md): the operator's activate is
    decided in IDLE. No fresh loaded correction, or a refused target fetch,
    stays IDLE with a WARN naming `level`; never ACTIVE then FAULT."""

    @pytest.mark.parametrize('loaded,age,why', [
        (False, 0.1, 'gravity_correction_loaded is false'),
        (True, None, 'no trajectory/status received'),
        (True, TRAJ_STATUS_STALE_S, 'stale'),
        (True, 5.0, 'stale'),
    ])
    def test_refused_without_a_fresh_loaded_correction(self, loaded, age, why):
        handler = IdleHandler()
        ctx = Context()
        ctx.gravity_correction_loaded = loaded
        ctx.traj_status_age_s = age
        ctx.enqueue_command('activate')
        assert handler.execute(ctx) is None
        assert ctx.drain_requests() == []            # nothing fetched, nothing moved
        assert len(ctx.operator_notices) == 1
        notice = ctx.operator_notices[0]
        assert notice.startswith('activate refused') and why in notice
        assert "'level'" in notice and 'stays IDLE' in notice
        assert handler.execute(ctx) is None          # still IDLE next tick

    def test_fresh_just_under_the_limit_proceeds(self):
        ctx = Context()
        ctx.gravity_correction_loaded = True
        ctx.traj_status_age_s = TRAJ_STATUS_STALE_S - 1e-3
        assert activate_levelling_refusal(ctx) is None

    def test_levelling_complete_is_not_the_answer(self):
        """C-LEVEL-1.O: the Platform Teensy's flag survives a trajectory_node
        restart, so it must never stand in for the applier's own answer."""
        ctx = Context()
        ctx.levelling_complete = True
        ctx.traj_status_age_s = 0.1
        assert activate_levelling_refusal(ctx) is not None

    def test_fetch_refusal_stays_idle_naming_the_reason(self):
        handler = IdleHandler()
        ctx = _levelled(Context())
        ctx.enqueue_command('activate')
        handler.execute(ctx)
        assert ctx.drain_requests() == ['fetch_activate_targets']
        ctx.operation_result = False
        ctx.operation_message = 'Leg 3 +0.1100 rev (+7.81 mm)'
        assert handler.execute(ctx) is None
        assert ctx.activate_leg_targets_rev is None
        assert 'Leg 3 +0.1100 rev' in ctx.operator_notices[0]
        assert "'level'" in ctx.operator_notices[0]
        ctx.operator_notices.clear()
        assert handler.execute(ctx) is None and ctx.drain_requests() == []

    def test_waits_for_the_fetch_in_idle(self):
        handler = IdleHandler()
        ctx = _levelled(Context())
        ctx.enqueue_command('activate')
        handler.execute(ctx)
        ctx.drain_requests()
        ctx.operation_pending = True                 # fetch in flight
        assert handler.execute(ctx) is None
        ctx.operation_pending = False
        ctx.activate_leg_targets_rev = list(_LEVELLED_TARGETS)
        ctx.operation_result = True
        assert handler.execute(ctx) == RobotState.ACTIVE

    def test_a_short_target_set_is_refused(self):
        handler = IdleHandler()
        ctx = _levelled(Context())
        assert _activate_from_idle(handler, ctx, targets=[2.2] * 5) is None
        assert 'activate refused' in ctx.operator_notices[0]

    def test_level_is_never_gated(self):
        """`level` must work with no correction loaded: it is how one is made."""
        handler = IdleHandler()
        ctx = Context()                              # nothing loaded, no status
        ctx.enqueue_command('level')
        assert handler.execute(ctx) == RobotState.LEVELLING

    def test_idle_entry_drops_stale_targets(self):
        handler = IdleHandler()
        ctx = Context()
        ctx.activate_leg_targets_rev = list(_LEVELLED_TARGETS)
        handler.on_enter(ctx)
        assert ctx.activate_leg_targets_rev is None


# ════════════════════════════════════════════════════════════════
# ActiveHandler
# ════════════════════════════════════════════════════════════════


def _activate_and_arm(handler, ctx):
    """Drive ActiveHandler through BOTH phases: activation, then the A2 arm."""
    ctx.operation_result = True
    handler.execute(ctx)                 # activation done → arm requested
    assert ctx.drain_requests() == ['arm_setpoints']
    ctx.operation_result = True
    handler.execute(ctx)                 # armed


class TestActiveHandler:
    def test_requests_activate_on_enter(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        assert ctx.request == 'activate'
        assert ctx.operation_result is None
        assert handler._activated is False

    # ── ARMING_CONTRACT A2: the arm phase ───────────────────────

    def test_arm_requested_after_activation(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()             # 'activate'
        ctx.operation_result = True
        assert handler.execute(ctx) is None
        assert ctx.control_mode == 'STANDBY'   # mode published BEFORE the arm
        assert ctx.request == 'arm_setpoints'
        assert ctx.operation_result is None

    def test_commands_wait_for_arm_phase(self):
        """Commands queued during the arm phase are consumed only once armed —
        a mode switch must never race the arm's stream-then-arm pre-check."""
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()
        ctx.operation_result = True
        handler.execute(ctx)             # arm requested
        ctx.enqueue_command('trajectory')
        ctx.operation_result = None
        assert handler.execute(ctx) is None
        assert ctx.active_mode == ActiveMode.STANDBY   # not consumed yet
        ctx.operation_result = True                    # arm succeeds
        handler.execute(ctx)
        handler.execute(ctx)                           # consumes 'trajectory'
        assert ctx.active_mode == ActiveMode.TRAJECTORY

    def test_arm_failure_retries(self):
        """The producer needs ~100-300 ms to seed after the mode publish, so the
        first arm attempt(s) may legitimately be refused — retry, don't FAULT."""
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()
        ctx.operation_result = True
        handler.execute(ctx)             # arm attempt 1 requested
        ctx.drain_requests()
        ctx.operation_result = False     # refused (e.g. no fresh frame yet)
        assert handler.execute(ctx) is None
        assert ctx.request == 'arm_setpoints'   # retried
        ctx.drain_requests()
        ctx.operation_result = True      # second attempt succeeds
        assert handler.execute(ctx) is None
        assert handler._armed is True

    def test_persistent_arm_refusal_goes_to_fault(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()
        ctx.operation_result = True
        handler.execute(ctx)             # arm attempt 1 requested
        result = None
        for _ in range(ActiveHandler.ARM_MAX_ATTEMPTS):
            ctx.drain_requests()
            ctx.operation_result = False
            result = handler.execute(ctx)
            if result is not None:
                break
        assert result == RobotState.FAULT

    def test_auto_arm_false_skips_arm_phase(self):
        """Manual-arm sessions (auto_arm_setpoint_output:=false): the operator
        arms via /set_setpoint_output; the handler must not request it."""
        handler = ActiveHandler()
        ctx = Context()
        ctx.auto_arm = False
        handler.on_enter(ctx)
        ctx.drain_requests()             # 'activate'
        ctx.operation_result = True
        assert handler.execute(ctx) is None
        assert ctx.drain_requests() == []          # NO arm request
        ctx.enqueue_command('trajectory')
        handler.execute(ctx)                       # commands work immediately
        assert ctx.active_mode == ActiveMode.TRAJECTORY

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
        handler.on_enter(ctx)
        ctx.operation_result = True
        assert handler.execute(ctx) is None
        assert handler._activated is True
        # Default sub-mode on activation is STANDBY (ROS2 input sources
        # silenced).
        assert ctx.active_mode == ActiveMode.STANDBY
        assert ctx.control_mode == 'STANDBY'

    def test_on_enter_resets_prior_sub_mode_to_standby(self):
        """Re-activation after FAULT or DEACTIVATE must always default to
        STANDBY — never inherit a prior session's sub-mode."""
        handler = ActiveHandler()
        ctx = Context()
        ctx.active_mode = ActiveMode.SPACEMOUSE  # stale state from prior run
        handler.on_enter(ctx)
        assert ctx.active_mode == ActiveMode.STANDBY

    def test_guard_clear_resume_skips_reactivation(self):
        """F1 — the guard-clear resume path (resume_active_no_rearm) must NOT request
        the ACTIVATE move: the legs never disarmed. It enters ACTIVE in the STANDBY
        hold, consumes the one-shot, and RE-VERIFIES the arm (A2 — idempotent: the
        bridge answers 'already enabled' if the wire stayed armed, and re-runs the
        full stream-then-arm pre-check if something disarmed it mid-fault)."""
        handler = ActiveHandler()
        ctx = Context()
        ctx.resume_active_no_rearm = True
        handler.on_enter(ctx)
        assert handler._activated is True           # NO 'activate' request
        assert ctx.active_mode == ActiveMode.STANDBY
        assert ctx.control_mode == 'STANDBY'
        assert ctx.resume_active_no_rearm is False  # one-shot consumed
        assert ctx.drain_requests() == ['arm_setpoints']  # arm re-verified
        ctx.operation_result = True                 # (idempotent) arm confirms
        handler.execute(ctx)
        ctx.enqueue_command('deactivate')
        assert handler.execute(ctx) is None         # stays ACTIVE until confirmed
        assert ctx.drain_requests() == ['deactivate']
        ctx.operation_result = True                 # bridge: stowed + IDLE
        assert handler.execute(ctx) == RobotState.IDLE

    def test_normal_activation_still_requests_activate(self):
        """Regression: a NORMAL entry (resume flag not set) still requests the
        ACTIVATE move — the resume path must not leak into ordinary activation."""
        handler = ActiveHandler()
        ctx = Context()
        assert ctx.resume_active_no_rearm is False
        handler.on_enter(ctx)
        assert ctx.request == 'activate'
        assert handler._activated is False

    def test_deactivate_command_returns_idle(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()
        _activate_and_arm(handler, ctx)
        ctx.enqueue_command('deactivate')
        assert handler.execute(ctx) is None
        assert ctx.drain_requests() == ['deactivate']
        ctx.operation_result = True
        assert handler.execute(ctx) == RobotState.IDLE

    def test_spacemouse_command_switches_mode(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()
        _activate_and_arm(handler, ctx)
        ctx.enqueue_command('spacemouse')
        assert handler.execute(ctx) is None
        assert ctx.active_mode == ActiveMode.SPACEMOUSE
        assert ctx.control_mode == 'SPACEMOUSE'

    def test_trajectory_command_switches_mode(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()
        _activate_and_arm(handler, ctx)
        ctx.enqueue_command('trajectory')
        assert handler.execute(ctx) is None
        assert ctx.active_mode == ActiveMode.TRAJECTORY
        assert ctx.control_mode == 'TRAJECTORY'

    def test_standby_command_returns_to_standby(self):
        """From any sub-mode, 'standby' returns to STANDBY (ROS2 inputs
        silenced, motors held at last MPC-commanded pose)."""
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()
        _activate_and_arm(handler, ctx)
        ctx.enqueue_command('spacemouse')
        handler.execute(ctx)
        assert ctx.active_mode == ActiveMode.SPACEMOUSE
        ctx.enqueue_command('standby')
        assert handler.execute(ctx) is None
        assert ctx.active_mode == ActiveMode.STANDBY
        assert ctx.control_mode == 'STANDBY'

    def test_unknown_command_after_activation_stays(self):
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()
        _activate_and_arm(handler, ctx)
        ctx.enqueue_command('unknown')
        assert handler.execute(ctx) is None

    def test_on_exit_requests_deactivate_and_preserves_mode(self):
        """A3/A4: on_exit requests the deactivate but must NOT blank the mode —
        the bridge disarms in-process inside _run_deactivate, and the stream
        must keep flowing until that operation resolves (IdleHandler blanks)."""
        handler = ActiveHandler()
        ctx = Context()
        ctx.control_mode = 'TRAJECTORY'
        handler.on_exit(ctx)
        assert ctx.request == 'deactivate'
        assert ctx.control_mode == 'TRAJECTORY'   # NOT blanked (A4)

    def test_mode_switch_uses_uppercase(self):
        """'spacemouse' → ActiveMode.SPACEMOUSE (uppercase conversion)."""
        handler = ActiveHandler()
        ctx = Context()
        handler.on_enter(ctx)
        ctx.drain_requests()
        _activate_and_arm(handler, ctx)
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
        ctx.drain_requests()   # A2: real-fault entry requests 'disarm_setpoints'
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
        ctx.drain_requests()   # A2: real-fault entry requests 'disarm_setpoints'
        ctx.enqueue_command('activate')
        handler.execute(ctx)
        assert ctx.consume_command() is None  # Consumed by execute
        assert ctx.request is None  # Not set because 'activate' is not 'clear_errors'

    # ── ARMING_CONTRACT A2: fault-entry disarm ──────────────────

    def test_real_fault_requests_disarm(self):
        """A REAL fault disarms the wire on entry: output is unusable and
        control_mode goes 'ERROR' (stopping the producer), so staying armed
        would only race the staleness watchdog."""
        handler = FaultHandler()
        ctx = Context()
        ctx.fatal_error = True
        handler.on_enter(ctx)
        assert 'disarm_setpoints' in ctx.drain_requests()

    def test_guard_only_fault_does_not_disarm(self):
        """A guard-only fault stays armed: the guard already suppresses output,
        and the resume path depends on the stream (and the arm) surviving."""
        handler = FaultHandler()
        ctx = Context()
        ctx.guard_latched = True
        ctx.prev_state = RobotState.ACTIVE
        ctx.wire_armed = True
        handler.on_enter(ctx)
        assert 'disarm_setpoints' not in ctx.drain_requests()

    def test_guard_latch_from_homing_is_not_guard_only(self):
        """Origin gate: a guard latch surfacing in a fault entered from HOMING is
        a stale prior-session latch, NOT a guard-only (ACTIVE) fault. Before the
        origin check, clearing it exited "back" to ACTIVE on a robot that never
        activated (the 2026-07-15 misroute — the comments said "from ACTIVE" but
        the code never enforced it)."""
        handler = FaultHandler()
        ctx = Context()
        ctx.guard_latched = True
        ctx.prev_state = RobotState.HOMING
        handler.on_enter(ctx)
        assert ctx.control_mode == 'ERROR'         # real-fault treatment
        ctx.guard_latched = False                   # operator cleared it
        assert handler.execute(ctx) == RobotState.BOOT   # NOT ACTIVE
        assert ctx.resume_active_no_rearm is False

    def test_stays_with_only_error_strings(self):
        """Non-empty errors list but no fatal flags still stays in FAULT."""
        handler = FaultHandler()
        ctx = Context()
        ctx.errors = ['some transient error']
        handler.on_enter(ctx)
        assert handler.execute(ctx) is None

    # ── FIX 2: a latched Teensy guard holds FAULT until it clears ──

    def test_stays_in_fault_while_guard_latched(self):
        """A latched guard (no ODrive error) holds FAULT: the guard suppresses leg
        output and only CLEAR_ERRORS releases it, so exiting FAULT here would return
        to a silently-crippled robot."""
        handler = FaultHandler()
        ctx = Context()
        ctx.guard_latched = True
        handler.on_enter(ctx)
        assert handler.execute(ctx) is None

    def test_exits_fault_when_guard_clears(self):
        """F1 — a GUARD-ONLY fault, once the guard clears, exits STRAIGHT BACK TO
        ACTIVE (not BOOT): the ODrives never disarmed, so re-running BOOT/HOMING would
        be incoherent. The one-shot resume flag tells ActiveHandler to skip re-arming."""
        handler = FaultHandler()
        ctx = Context()
        ctx.guard_latched = True
        ctx.prev_state = RobotState.ACTIVE   # guard-only requires ACTIVE origin
        ctx.wire_armed = True                # ... AND an armed wire
        handler.on_enter(ctx)
        assert handler.execute(ctx) is None            # held while latched
        ctx.guard_latched = False
        assert handler.execute(ctx) == RobotState.ACTIVE
        assert ctx.resume_active_no_rearm is True

    def test_guard_latch_default_is_false(self):
        assert Context().guard_latched is False

    # ── F1: a GUARD-ONLY fault must NOT publish a mode outside the streaming set ──

    # trajectory_node's streaming set (jugglebot.trajectory_node._DEFAULT_STREAM_MODES);
    # a control_mode outside this silences the 40 Hz emitter and abandons the descent.
    _STREAM_MODES = {'STANDBY', 'TRAJECTORY', 'SPACEMOUSE', 'GUI'}

    def test_guard_only_fault_preserves_streaming_mode_not_error(self):
        """(i) A guard-only fault PRESERVES the streaming mode — it must never publish
        'ERROR' (which is outside trajectory_node's streaming set and would kill the
        emitter mid-descent). Asserts the actual control_mode value the orchestrator
        will publish."""
        handler = FaultHandler()
        ctx = Context()
        ctx.active_mode = ActiveMode.TRAJECTORY  # the sub-mode active when the guard hit
        ctx.control_mode = ''
        ctx.guard_latched = True
        ctx.prev_state = RobotState.ACTIVE
        ctx.wire_armed = True
        handler.on_enter(ctx)
        assert ctx.control_mode == 'TRAJECTORY'
        assert ctx.control_mode != 'ERROR'
        assert ctx.control_mode in self._STREAM_MODES

    def test_guard_only_fault_preserves_standby_mode(self):
        """The armed-hold STANDBY case (the actual Phase-1 bench scenario): a guard
        latch in STANDBY keeps control_mode='STANDBY', never 'ERROR'."""
        handler = FaultHandler()
        ctx = Context()
        ctx.active_mode = ActiveMode.STANDBY
        ctx.control_mode = ''
        ctx.guard_latched = True
        ctx.prev_state = RobotState.ACTIVE
        ctx.wire_armed = True
        handler.on_enter(ctx)
        assert ctx.control_mode == 'STANDBY'
        assert ctx.control_mode in self._STREAM_MODES

    def test_real_fault_alongside_guard_is_treated_as_real(self):
        """(iii) A guard latch riding ALONGSIDE a real ODrive error is a REAL fault:
        the ODrive error dominates → 'ERROR' publish, and on full clear it exits to
        BOOT (unchanged), NOT to ACTIVE."""
        handler = FaultHandler()
        ctx = Context()
        ctx.active_mode = ActiveMode.TRAJECTORY
        ctx.fatal_error = True
        ctx.guard_latched = True
        handler.on_enter(ctx)
        assert ctx.control_mode == 'ERROR'
        # Both clear → real-fault BOOT exit, resume flag NOT set.
        ctx.fatal_error = False
        ctx.guard_latched = False
        assert handler.execute(ctx) == RobotState.BOOT
        assert ctx.resume_active_no_rearm is False

    def test_guard_only_fault_promoted_when_real_error_appears(self):
        """A guard-only fault that GAINS a real ODrive error mid-fault is promoted to
        the real-fault path: control_mode flips to 'ERROR' and, once everything clears,
        it exits to BOOT — never masking a genuine fault behind the streaming mode."""
        handler = FaultHandler()
        ctx = Context()
        ctx.active_mode = ActiveMode.TRAJECTORY
        ctx.control_mode = ''
        ctx.guard_latched = True
        ctx.prev_state = RobotState.ACTIVE
        ctx.wire_armed = True
        handler.on_enter(ctx)
        assert ctx.control_mode == 'TRAJECTORY'   # started guard-only
        # A real error now appears while still in FAULT.
        ctx.fatal_error = True
        assert handler.execute(ctx) is None        # held (real error present)
        assert ctx.control_mode == 'ERROR'         # promoted
        # Even if the guard clears first, a real error present must NOT exit to ACTIVE.
        ctx.guard_latched = False
        assert handler.execute(ctx) is None
        assert ctx.resume_active_no_rearm is False
        # Now the real error clears → real-fault BOOT exit.
        ctx.fatal_error = False
        assert handler.execute(ctx) == RobotState.BOOT

    def test_non_guard_fault_control_mode_and_exit_unchanged(self):
        """(iii) With NO guard latch, on_enter publishes 'ERROR' and execute exits to
        BOOT on clear — behaviourally identical to before F1."""
        handler = FaultHandler()
        ctx = Context()
        ctx.errors = ['SOME_ERROR']
        handler.on_enter(ctx)
        assert ctx.control_mode == 'ERROR'
        ctx.errors = []
        assert handler.execute(ctx) == RobotState.BOOT
        assert ctx.resume_active_no_rearm is False


# ════════════════════════════════════════════════════════════════
# Real-fault exit stow (ARMING_CONTRACT choreography step 6, 2026-10-06)
# ════════════════════════════════════════════════════════════════


# Requests the orchestrator tracks in its single operation slot (the ones these
# tests drive; 'arm_setpoints' is drained by hand where it occurs).
_TRACKED_REQUESTS = ('activate', 'deactivate', 'clear_errors',
                     'level_activate', 'level_get_tilt', 'level_deactivate')


def _dispatch(ctx):
    """Stand in for the orchestrator's _process_requests: drain the requests and
    start the tracked ones the way _start_service_call / _start_tilt_service_call
    do (pending, no result). 'disarm_setpoints' is fire-and-forget and never
    touches the slot."""
    reqs = ctx.drain_requests()
    if any(r in _TRACKED_REQUESTS for r in reqs):
        ctx.operation_pending = True
        ctx.operation_result = None
    return reqs


def _resolve(ctx, ok):
    """The tracked operation completes (_check_pending_operations)."""
    ctx.operation_pending = False
    ctx.operation_result = ok


def _force_fault(sm, ctx):
    """The orchestrator's forced FAULT entry: force_transition, then
    _cancel_pending_operations (which drops ActiveHandler.on_exit's deactivate)."""
    sm.force_transition(RobotState.FAULT, ctx)
    ctx.operation_pending = False
    ctx.operation_result = None
    ctx.clear_requests()
    ctx.clear_commands()


class TestFaultExitStow:
    """A real fault that began in ACTIVE stows the legs before BOOT when all six
    are closed-loop and homed, and holds FAULT until the stow succeeds. Replays
    2026-10-06 20:25: MOTOR_FB_STALE on axis 6 forced a guard-only FAULT, the hand
    ODrive's reboot promoted it to real, and the clear went FAULT -> BOOT -> IDLE
    with no deactivate, leaving the legs holding an ACTIVE pose 62 mm off centre."""

    @staticmethod
    def _armed_active(**ctx_kw):
        """A machine in ACTIVE, activated and armed, legs holding the pose."""
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        ctx.is_homed = True
        ctx.legs_closed_loop = True
        ctx.wire_armed = True
        for k, v in ctx_kw.items():
            setattr(ctx, k, v)
        sm.force_transition(RobotState.ACTIVE, ctx)
        sm.tick(ctx)                       # enter: requests 'activate'
        _dispatch(ctx)
        _resolve(ctx, True)
        sm.tick(ctx)                       # activated: requests 'arm_setpoints'
        ctx.drain_requests()
        ctx.operation_result = True
        sm.tick(ctx)                       # armed
        assert sm.state == RobotState.ACTIVE
        return sm, ctx

    def _guard_fault_promoted_then_cleared(self, **ctx_kw):
        """ACTIVE -> forced FAULT (guard-only) -> promoted real by a hand ODrive
        error -> guard and errors clear. Returns (sm, ctx, requests of the
        clearing tick)."""
        sm, ctx = self._armed_active(**ctx_kw)
        ctx.guard_latched = True                     # 718.60 MOTOR_FB_STALE
        _force_fault(sm, ctx)
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT
        assert ctx.control_mode == 'STANDBY'         # guard-only on entry
        assert _dispatch(ctx) == []
        ctx.errors = ['ODrive error on hand']        # 720.41 hand INITIALIZING
        ctx.fatal_error = True
        sm.tick(ctx)
        assert ctx.control_mode == 'ERROR'           # promoted to real
        assert _dispatch(ctx) == ['disarm_setpoints']
        ctx.errors = []                              # 721.00 hand back, guard clears
        ctx.fatal_error = False
        ctx.guard_latched = False
        sm.tick(ctx)
        return sm, ctx, _dispatch(ctx)

    def test_replay_2026_10_06_stows_before_boot(self):
        sm, ctx, reqs = self._guard_fault_promoted_then_cleared()
        assert reqs == ['deactivate']
        assert sm.state == RobotState.FAULT          # held while the stow runs
        for _ in range(5):
            sm.tick(ctx)
            assert sm.state == RobotState.FAULT
            assert _dispatch(ctx) == []              # no second request
        _resolve(ctx, True)
        ctx.legs_closed_loop = False                 # legs IDLE at STOW
        _legs(ctx, [_IDLE] * 6)                      # A7: BOOT may claim IDLE
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        assert _dispatch(ctx) == []
        # BOOT then skips to IDLE (already homed), as before.
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE

    def test_legs_not_closed_loop_exits_to_boot_at_once(self):
        sm, ctx, reqs = self._guard_fault_promoted_then_cleared(
            legs_closed_loop=False)
        assert 'deactivate' not in reqs
        assert sm.state == RobotState.BOOT

    def test_not_homed_with_posed_legs_still_stows(self):
        """BUG-122: the bridge clears is_homed on a Hand-only reference loss
        (MISSING_ESTIMATE or a Hand drive reboot) while the legs still hold an
        active pose in CLOSED_LOOP. The exit must still stow them first; going
        straight to BOOT would start HOMING with the legs at that pose."""
        handler = FaultHandler()
        ctx = Context()
        ctx.prev_state = RobotState.ACTIVE
        ctx.legs_closed_loop = True
        ctx.is_homed = False
        ctx.fatal_error = True
        handler.on_enter(ctx)
        ctx.drain_requests()
        ctx.fatal_error = False
        assert handler.execute(ctx) is None
        assert ctx.drain_requests() == ['deactivate']
        # Once stowed (legs IDLE), BOOT; un-homed, so BOOT goes to HOMING.
        ctx.operation_pending = False
        ctx.operation_result = True
        ctx.legs_closed_loop = False
        assert handler.execute(ctx) == RobotState.BOOT
        boot = BootHandler()
        boot.on_enter(ctx)
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
        assert boot.execute(ctx) == RobotState.HOMING

    @pytest.mark.parametrize('origin', [RobotState.HOMING, RobotState.IDLE,
                                        RobotState.BOOT])
    def test_real_fault_from_any_state_with_six_closed_loop_stows(self, origin):
        """FW 53 (2026-11-26): the exit stow is decided on drive state alone. An
        aborted ladder can leave the six HELD Closed-loop mid-air after any
        visit, so a visit that began outside ACTIVE/LEVELLING stows too.
        Mutation: restoring the prev_state-in-(ACTIVE, LEVELLING) condition
        returns BOOT here with the Legs up."""
        handler = FaultHandler()
        ctx = Context()
        ctx.prev_state = origin
        ctx.is_homed = True
        ctx.legs_closed_loop = True
        ctx.errors = ['SOME_ERROR']
        handler.on_enter(ctx)
        ctx.drain_requests()
        ctx.errors = []
        assert handler.execute(ctx) is None
        assert ctx.drain_requests() == ['deactivate']

    @pytest.mark.parametrize('origin', [RobotState.HOMING, RobotState.IDLE,
                                        RobotState.ACTIVE, RobotState.LEVELLING])
    def test_real_fault_with_legs_not_all_closed_loop_exits_to_boot(self, origin):
        """All six Idle (an FW 53 rise returned to Stow) or a mixed set: BOOT at
        once, no stow; BOOT's A7/A6 predicate decides from there."""
        handler = FaultHandler()
        ctx = Context()
        ctx.prev_state = origin
        ctx.is_homed = True
        ctx.legs_closed_loop = False
        ctx.errors = ['SOME_ERROR']
        handler.on_enter(ctx)
        ctx.drain_requests()
        ctx.errors = []
        assert handler.execute(ctx) == RobotState.BOOT
        assert ctx.drain_requests() == []

    def test_failed_stow_holds_fault_and_clear_errors_retries_once(self):
        sm, ctx, reqs = self._guard_fault_promoted_then_cleared()
        assert reqs == ['deactivate']
        _resolve(ctx, False)                         # the stow failed
        for _ in range(5):                           # no spin, no BOOT
            sm.tick(ctx)
            assert sm.state == RobotState.FAULT
            assert _dispatch(ctx) == []
        ctx.enqueue_command('clear_errors')
        sm.tick(ctx)
        assert _dispatch(ctx) == ['clear_errors']    # pending, as dispatched
        sm.tick(ctx)
        assert _dispatch(ctx) == []                  # waits for clear_errors
        _resolve(ctx, True)
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT
        assert _dispatch(ctx) == ['deactivate']      # exactly one retry
        sm.tick(ctx)
        assert _dispatch(ctx) == []
        _resolve(ctx, True)
        ctx.legs_closed_loop = False
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT

    def test_clear_errors_during_the_stow_waits_for_it(self):
        """A clear_errors sent while the stow is in flight is NOT dispatched (it
        would take the single tracked slot and its result would read as the
        stow's); it stays queued and is consumed after the stow resolves."""
        sm, ctx, reqs = self._guard_fault_promoted_then_cleared()
        assert reqs == ['deactivate']
        ctx.enqueue_command('clear_errors')
        sm.tick(ctx)
        assert _dispatch(ctx) == []
        assert sm.state == RobotState.FAULT
        _resolve(ctx, False)
        sm.tick(ctx)                                 # failure + queued command
        assert _dispatch(ctx) == ['clear_errors']
        assert sm.state == RobotState.FAULT

    def test_in_flight_tracked_op_delays_the_stow_request(self):
        """A handler-returned ACTIVE -> FAULT keeps ActiveHandler.on_exit's
        deactivate in flight (only a FORCED entry cancels it): the stow is not
        requested over it, and once it lands with the legs stowed, BOOT."""
        handler = FaultHandler()
        ctx = Context()
        ctx.prev_state = RobotState.ACTIVE
        ctx.is_homed = True
        ctx.legs_closed_loop = True
        ctx.operation_pending = True                 # on_exit's deactivate
        handler.on_enter(ctx)
        ctx.drain_requests()
        assert handler.execute(ctx) is None
        assert ctx.drain_requests() == []
        _resolve(ctx, True)
        ctx.legs_closed_loop = False
        assert handler.execute(ctx) == RobotState.BOOT
        assert ctx.drain_requests() == []

    def test_guard_only_clear_resumes_active_without_stow(self):
        """Pins the UNCHANGED guard-only path: legs closed-loop and homed, the
        guard clears with no ODrive error -> straight back to ACTIVE with the
        no-re-arm resume, and no deactivate."""
        sm, ctx = self._armed_active()
        ctx.guard_latched = True
        _force_fault(sm, ctx)
        sm.tick(ctx)
        assert _dispatch(ctx) == []
        ctx.guard_latched = False
        sm.tick(ctx)
        assert sm.state == RobotState.ACTIVE
        assert ctx.resume_active_no_rearm is True
        assert 'deactivate' not in _dispatch(ctx)


class TestFaultExitStowFromLevelling:
    """A real fault whose FAULT visit began in LEVELLING stows the legs the same
    way (ARMING_CONTRACT choreography step 6, extended 2026-10-07):
    level_activate runs the profiled ACTIVATE and the pose is held through
    settle / tilt / correction / persist / mocap-check, and
    LevellingHandler.on_exit only clears requests, so without the stow the
    real-fault exit reached BOOT -> IDLE with the legs at the level pose."""

    @staticmethod
    def _levelling_mid_settle(**ctx_kw):
        """A machine in LEVELLING, level_activate succeeded, mid-settle."""
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        ctx.is_homed = True
        ctx.legs_closed_loop = True
        for k, v in ctx_kw.items():
            setattr(ctx, k, v)
        sm.force_transition(RobotState.LEVELLING, ctx)
        sm.tick(ctx)                       # enter: requests 'level_activate'
        assert _dispatch(ctx) == ['level_activate']
        _resolve(ctx, True)
        sm.tick(ctx)                       # activated: settle starts
        assert sm.state == RobotState.LEVELLING
        assert _dispatch(ctx) == []
        return sm, ctx

    @staticmethod
    def _boot_to_idle(sm, ctx):
        _legs(ctx, [_IDLE] * 6)        # the stow left all six Legs Idle (A7)
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE

    def _forced_fault_then_cleared(self, **ctx_kw):
        """An ODrive error forces FAULT mid-settle (orchestrator _tick step 2),
        then clears. Returns (sm, ctx, requests of the clearing tick)."""
        sm, ctx = self._levelling_mid_settle(**ctx_kw)
        ctx.errors = ['ODrive error on leg 2']
        ctx.fatal_error = True
        _force_fault(sm, ctx)
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT
        assert ctx.control_mode == 'ERROR'
        assert _dispatch(ctx) == ['disarm_setpoints']
        ctx.errors = []
        ctx.fatal_error = False
        sm.tick(ctx)
        return sm, ctx, _dispatch(ctx)

    def test_forced_fault_mid_settle_stows_before_boot(self):
        sm, ctx, reqs = self._forced_fault_then_cleared()
        assert reqs == ['deactivate']
        for _ in range(5):
            sm.tick(ctx)
            assert sm.state == RobotState.FAULT      # held while the stow runs
            assert _dispatch(ctx) == []              # no second request
        _resolve(ctx, True)
        ctx.legs_closed_loop = False                 # legs IDLE at STOW
        _legs(ctx, [_IDLE] * 6)                      # A7: BOOT may claim IDLE
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        assert _dispatch(ctx) == []
        self._boot_to_idle(sm, ctx)

    def test_handler_returned_fault_on_failed_tilt_stows_before_boot(self):
        """level_get_tilt resolves False -> LevellingHandler returns FAULT. The
        failed op has RESOLVED (nothing in flight), no error string is set, so
        the stow is requested on the first FAULT tick, alongside the A2
        disarm."""
        sm, ctx = self._levelling_mid_settle()
        sm._handlers[RobotState.LEVELLING]._settle_start -= 60.0   # settled
        sm.tick(ctx)
        assert _dispatch(ctx) == ['level_get_tilt']
        _resolve(ctx, False)                         # NaN tilt / service fail
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT          # handler-returned
        sm.tick(ctx)
        assert ctx.control_mode == 'ERROR'
        assert _dispatch(ctx) == ['disarm_setpoints', 'deactivate']
        for _ in range(5):
            sm.tick(ctx)
            assert sm.state == RobotState.FAULT
            assert _dispatch(ctx) == []
        _resolve(ctx, True)
        ctx.legs_closed_loop = False
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        self._boot_to_idle(sm, ctx)

    def test_legs_not_closed_loop_exits_to_boot_at_once(self):
        """A leg out of CLOSED_LOOP at the clear (e.g. an ODrive reboot): no
        pose to lower, BOOT at once, as before — re-arming is HOMING's job."""
        sm, ctx, reqs = self._forced_fault_then_cleared(
            legs_closed_loop=False)
        assert 'deactivate' not in reqs
        assert sm.state == RobotState.BOOT

    def test_guard_latch_during_levelling_is_a_real_fault(self):
        """LEVELLING never arms the wire, so a guard latch carried into a
        LEVELLING fault is NOT guard-only even with a stale armed mirror: the
        mode goes ERROR, the wire is disarmed, and on the clear the exit is the
        stow + BOOT, never a resume into ACTIVE."""
        sm, ctx = self._levelling_mid_settle(wire_armed=True)
        sm._handlers[RobotState.LEVELLING]._settle_start -= 60.0
        sm.tick(ctx)
        assert _dispatch(ctx) == ['level_get_tilt']
        ctx.guard_latched = True                     # latch, no ODrive error
        _resolve(ctx, False)
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT
        sm.tick(ctx)
        assert ctx.control_mode == 'ERROR'           # not the guard-only hold
        assert _dispatch(ctx) == ['disarm_setpoints']  # held: guard latched
        ctx.guard_latched = False
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT
        assert _dispatch(ctx) == ['deactivate']
        assert ctx.resume_active_no_rearm is False
        _resolve(ctx, True)
        ctx.legs_closed_loop = False
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT


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
        _legs(ctx, [_IDLE] * 6)   # A6: the Legs are down, Homing may start

        # BOOT: waiting for heartbeats
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT

        # Heartbeats + firmware validated + first /link_status → HOMING
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
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

        # Activate (BUG-130: decided in IDLE, then the Levelled Active pose)
        _levelled(ctx)
        ctx.enqueue_command('activate')
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        assert ctx.drain_requests() == ['fetch_activate_targets']
        ctx.activate_leg_targets_rev = list(_LEVELLED_TARGETS)
        ctx.operation_result = True
        sm.tick(ctx)
        assert sm.state == RobotState.ACTIVE

        # Activation completes, then the A2 arm phase completes
        sm.tick(ctx)  # Enter ACTIVE (requests activation)
        ctx.operation_result = True
        sm.tick(ctx)  # Activation succeeds → arm requested
        assert 'arm_setpoints' in ctx.drain_requests()
        ctx.operation_result = True
        sm.tick(ctx)  # Armed

        # Deactivate — the machine stays ACTIVE (streaming mode published, A4)
        # until the bridge confirms the stow; only then IDLE, which blanks it.
        ctx.enqueue_command('deactivate')
        sm.tick(ctx)
        assert sm.state == RobotState.ACTIVE
        assert ctx.control_mode == 'STANDBY'
        assert ctx.drain_requests() == ['deactivate']
        ctx.operation_result = True             # deactivate confirmed
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        assert ctx.drain_requests() == []       # no second deactivate on exit
        assert ctx.control_mode == 'STANDBY'
        sm.tick(ctx)                            # IDLE blanks after the op
        assert ctx.control_mode == ''

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
        _legs(ctx, [_IDLE] * 6)   # A6: the Legs are down, Homing may start
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
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
        _legs(ctx, [_IDLE] * 6)   # A6: the Legs are down, Homing may start
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


# ════════════════════════════════════════════════════════════════
# ARMING_CONTRACT review fixes (2026-07-15 adversarial review round)
# ════════════════════════════════════════════════════════════════


class TestArmingContractReviewFixes:
    def test_guard_only_requires_wire_armed(self):
        """Guard-only = 'latched WHILE ARMED'. prev_state==ACTIVE alone
        misclassifies a latch-caused ACTIVATE refusal (FAULT raised BY
        ActiveHandler on an unarmed robot) as guard-only — which would 'resume'
        a robot that never activated. wire_armed=False routes to the
        heavier-but-safe real-fault path."""
        handler = FaultHandler()
        ctx = Context()
        ctx.guard_latched = True
        ctx.prev_state = RobotState.ACTIVE
        ctx.wire_armed = False              # NOT armed → not guard-only
        handler.on_enter(ctx)
        assert ctx.control_mode == 'ERROR'  # real-fault treatment
        ctx.guard_latched = False
        assert handler.execute(ctx) == RobotState.BOOT   # NOT ACTIVE
        assert ctx.resume_active_no_rearm is False

    def test_promotion_to_real_fault_requests_disarm(self):
        """The guard-only→real mid-FAULT promotion publishes 'ERROR' (stopping
        the producer) — leaving the wire armed would only race the staleness
        watchdog, and on_enter cannot re-run to issue the disarm."""
        handler = FaultHandler()
        ctx = Context()
        ctx.guard_latched = True
        ctx.prev_state = RobotState.ACTIVE
        ctx.wire_armed = True
        handler.on_enter(ctx)
        assert ctx.drain_requests() == []   # guard-only: no disarm on entry
        ctx.fatal_error = True              # real error arrives mid-FAULT
        handler.execute(ctx)
        assert 'disarm_setpoints' in ctx.drain_requests()
        assert ctx.control_mode == 'ERROR'


# ════════════════════════════════════════════════════════════════
# ARMING_CONTRACT A6: Homing starts only from a Platform that is down
# ════════════════════════════════════════════════════════════════

_IDLE = 1
_CL = 8


def _legs(ctx, states, idle_for_s=10.0):
    """Set the six Leg axis states the way the orchestrator derives them: the
    tuple, its arrival time (fresh), and legs_idle_since only while all six
    read IDLE."""
    ctx.leg_states = tuple(states)
    ctx.leg_states_time = time.monotonic()
    if all(s == _IDLE for s in states):
        ctx.legs_idle_since = time.monotonic() - idle_for_s
    else:
        ctx.legs_idle_since = None


def _boot_ready(ctx):
    ctx.all_heartbeats = True
    ctx.firmware_validated = True
    ctx.link_status_seen = True


def _requests_moving_legs(reqs):
    return [r for r in reqs if r in ('encoder_search', 'home')]


class TestHomingGate:
    """Every path into HOMING passes homing_refusal (StateMachine guard).
    Each case is a row of the 2026-11-25 table in
    logbook/2026-11-25-homing-only-from-a-lowered-platform.md."""

    def test_odrive_state_literals_match_the_protocol(self):
        from jugglebot.can import odrive
        from jugglebot import state_machine as smod
        assert smod.ODRIVE_AXIS_IDLE == odrive.AXIS_STATES['IDLE']
        assert smod.ODRIVE_AXIS_CLOSED_LOOP == odrive.AXIS_STATES['CLOSED_LOOP']
        assert smod.NUM_LEGS == len(odrive.LEG_AXES)

    # ── the predicate ──────────────────────────────────────────────
    def test_predicate_rows(self):
        from jugglebot.state_machine import (
            homing_refusal, HOMING_LEGS_IDLE_SETTLE_S, HOMING_SETTLING_PREFIX)
        settled = HOMING_LEGS_IDLE_SETTLE_S + 0.1
        assert homing_refusal([_IDLE] * 6, settled, 0.0) is None
        assert 'not known' in homing_refusal(None, None, 0.0)
        assert 'not known' in homing_refusal([_IDLE] * 5, settled, 0.0)
        assert homing_refusal([_IDLE] * 6, None, 0.0).startswith(HOMING_SETTLING_PREFIX)
        assert homing_refusal([_IDLE] * 6, 0.5, 0.0).startswith(HOMING_SETTLING_PREFIX)
        all_cl = homing_refusal([_CL] * 6, None, 0.0)
        assert 'all six Legs are Closed-loop' in all_cl and '/deactivate' in all_cl
        mixed = homing_refusal([_IDLE] + [_CL] * 5, None, 0.0)
        assert 'Leg(s) 1, 2, 3, 4, 5 are Closed-loop' in mixed
        assert 'Leg(s) 0 are not' in mixed
        assert 'E-stop button' in mixed and '/deactivate' not in mixed
        other = homing_refusal([0] + [_IDLE] * 5, None, 0.0)
        assert 'Leg 0 state 0' in other

    def test_predicate_stale_states_are_unknown(self):
        """A6 freshness: Leg states older than LEG_STATES_MAX_AGE_S (or never
        timestamped) are unknown, even when they read all-Idle and settled."""
        from jugglebot.state_machine import (
            homing_refusal, LEG_STATES_MAX_AGE_S, HOMING_STALE_PREFIX)
        assert homing_refusal([_IDLE] * 6, 10.0, LEG_STATES_MAX_AGE_S) is None
        stale = homing_refusal([_IDLE] * 6, 10.0, LEG_STATES_MAX_AGE_S + 0.1)
        assert stale.startswith(HOMING_STALE_PREFIX)
        assert 'not known' in stale and 'stale' in stale and '1.1 s old' in stale
        assert homing_refusal([_IDLE] * 6, 10.0, None).startswith(HOMING_STALE_PREFIX)

    def test_stale_states_hold_boot_and_notify_once(self):
        """The /robot_state topic stops: the 2 s settle must not keep running
        against the old all-Idle snapshot, even though it reads settled."""
        notes = []
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        ctx.notify = notes.append
        _boot_ready(ctx)
        ctx.is_homed = False
        _legs(ctx, [_IDLE] * 6)
        ctx.leg_states_time = time.monotonic() - 5.0      # the topic went quiet
        for _ in range(20):
            sm.tick(ctx)
            assert sm.state == RobotState.BOOT
            assert _requests_moving_legs(ctx.drain_requests()) == []
        assert len(notes) == 1 and 'stale' in notes[0]    # once, age varies
        _legs(ctx, [_IDLE] * 6)                           # topic resumed, fresh
        sm.tick(ctx)
        assert sm.state == RobotState.HOMING

    # ── (f) relaunch under a posed Platform ───────────────────────
    def test_relaunch_not_homed_under_closed_loop_legs_holds_boot(self):
        notes = []
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        ctx.notify = notes.append
        _boot_ready(ctx)
        ctx.is_homed = False
        _legs(ctx, [_CL] * 6)
        for _ in range(20):
            sm.tick(ctx)
            assert sm.state == RobotState.BOOT
            assert _requests_moving_legs(ctx.drain_requests()) == []
        assert len(notes) == 1 and '/deactivate' in notes[0]
        _legs(ctx, [_IDLE] * 6)                     # operator ran /deactivate
        sm.tick(ctx)
        assert sm.state == RobotState.HOMING

    def test_idle_home_command_with_closed_loop_legs_is_refused(self):
        notes = []
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        ctx.notify = notes.append
        _boot_ready(ctx)
        ctx.is_homed = True                         # relaunch, flag survived
        _legs(ctx, [_IDLE] * 6)                     # A7: BOOT claims IDLE at Stow
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE          # unchanged (not homing)
        _legs(ctx, [_CL] * 6)                       # a manual /activate in IDLE
        for n in (1, 2):
            ctx.enqueue_command('home')
            sm.tick(ctx)
            assert sm.state == RobotState.IDLE
            assert _requests_moving_legs(ctx.drain_requests()) == []
            sm.tick(ctx)
            assert len(notes) == n                  # one notice per command

    def test_forced_transition_into_homing_is_gated(self):
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        _legs(ctx, [_CL] * 6)
        sm.force_transition(RobotState.HOMING, ctx)
        assert sm.state == RobotState.BOOT
        _legs(ctx, [_IDLE] * 6)
        sm.force_transition(RobotState.HOMING, ctx)
        assert sm.state == RobotState.HOMING

    # ── (a)/(d) one Leg lost (or rebooted) during ACTIVE ──────────
    def test_one_leg_lost_in_active_never_homes_under_five_holding(self):
        notes = []
        sm, ctx = TestFaultExitStow._armed_active()
        _boot_ready(ctx)
        ctx.notify = notes.append
        _legs(ctx, [_CL] * 6)
        # Leg 0 MISSING_ESTIMATE: its drive disarms (IDLE); the firmware's Soft
        # E-stop holds Legs 1-5 at their setpoints. BUG-122 clears is_homed.
        _legs(ctx, [_IDLE] + [_CL] * 5)
        ctx.legs_closed_loop = False
        ctx.errors = ['ODrive error on leg 0: MISSING_ESTIMATE']
        ctx.fatal_error = True
        ctx.is_homed = False
        _force_fault(sm, ctx)
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT
        ctx.drain_requests()
        ctx.errors = []                             # operator clear_errors
        ctx.fatal_error = False
        for _ in range(30):
            sm.tick(ctx)
            assert sm.state in (RobotState.FAULT, RobotState.BOOT)
            reqs = ctx.drain_requests()
            assert 'deactivate' not in reqs         # no stow with a Leg lost
            assert _requests_moving_legs(reqs) == []
        assert sm.state == RobotState.BOOT
        refusals = [n for n in notes if n.startswith('Homing refused')]
        assert len(refusals) == 1 and 'E-stop button' in refusals[0]
        # Operator supports the Platform and presses the E-stop button: every
        # drive powers down, the Platform comes down, power returns Idle.
        _legs(ctx, [_IDLE] * 6, idle_for_s=0.0)
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT          # settling
        _legs(ctx, [_IDLE] * 6)
        sm.tick(ctx)
        assert sm.state == RobotState.HOMING

    # ── (b) Hand-only loss during ACTIVE: stow first, then home ───
    def test_hand_only_loss_stows_then_homes_after_the_settle(self):
        sm, ctx = TestFaultExitStow._armed_active()
        _boot_ready(ctx)
        _legs(ctx, [_CL] * 6)
        ctx.errors = ['ODrive error on hand: MISSING_ESTIMATE']
        ctx.fatal_error = True
        ctx.is_homed = False
        _force_fault(sm, ctx)
        sm.tick(ctx)
        ctx.drain_requests()
        ctx.errors = []
        ctx.fatal_error = False
        sm.tick(ctx)
        assert _dispatch(ctx) == ['deactivate']     # the BUG-122 exit stow
        _resolve(ctx, True)
        _legs(ctx, [_IDLE] * 6, idle_for_s=0.0)     # stowed, Legs IDLE on arrival
        ctx.legs_closed_loop = False
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT          # settle not yet elapsed
        _legs(ctx, [_IDLE] * 6)
        sm.tick(ctx)
        assert sm.state == RobotState.HOMING

    # ── (c) E-stop button from ACTIVE ─────────────────────────────
    def test_estop_button_recovery_homes_once_settled(self):
        sm, ctx = TestFaultExitStow._armed_active()
        _boot_ready(ctx)
        _legs(ctx, [0] * 6)                         # drives unpowered
        ctx.legs_closed_loop = False
        ctx.all_heartbeats = False
        ctx.errors = ['undervoltage']
        ctx.undervoltage = True
        ctx.is_homed = False
        _force_fault(sm, ctx)
        sm.tick(ctx)
        ctx.drain_requests()
        _legs(ctx, [_IDLE] * 6, idle_for_s=0.0)     # power back: Idle
        ctx.all_heartbeats = True
        ctx.errors = []
        ctx.undervoltage = False
        for _ in range(3):
            sm.tick(ctx)
            assert sm.state in (RobotState.FAULT, RobotState.BOOT)
            assert _requests_moving_legs(ctx.drain_requests()) == []
        _legs(ctx, [_IDLE] * 6)
        sm.tick(ctx)
        assert sm.state == RobotState.HOMING

    # ── (e) Fault from IDLE: the Legs are down, Homing proceeds ───
    def test_fault_from_idle_with_legs_down_homes(self):
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        _boot_ready(ctx)
        _legs(ctx, [_IDLE] * 6)
        sm.force_transition(RobotState.IDLE, ctx)
        sm.tick(ctx)
        ctx.errors = ['ODrive error on leg 2: MISSING_ESTIMATE']
        _force_fault(sm, ctx)
        sm.tick(ctx)
        ctx.errors = []
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        sm.tick(ctx)
        assert sm.state == RobotState.HOMING

    # ── (g) the exit stow fails part-way ──────────────────────────
    def test_failed_stow_leaving_legs_mixed_does_not_home(self):
        sm, ctx = TestFaultExitStow._armed_active()
        _boot_ready(ctx)
        _legs(ctx, [_CL] * 6)
        ctx.errors = ['ODrive error on hand']
        ctx.fatal_error = True
        ctx.is_homed = False
        _force_fault(sm, ctx)
        sm.tick(ctx)
        ctx.drain_requests()
        ctx.errors = []
        ctx.fatal_error = False
        sm.tick(ctx)
        assert _dispatch(ctx) == ['deactivate']
        _resolve(ctx, False)                         # stow failed
        _legs(ctx, [_IDLE, _IDLE] + [_CL] * 4)       # some Legs dropped out
        ctx.legs_closed_loop = False
        for _ in range(5):
            sm.tick(ctx)
            assert sm.state == RobotState.FAULT      # held until clear_errors
        ctx.enqueue_command('clear_errors')
        for _ in range(10):
            sm.tick(ctx)
            assert sm.state in (RobotState.FAULT, RobotState.BOOT)
            ctx.operation_pending = False
            assert _requests_moving_legs(ctx.drain_requests()) == []


class TestHomingGateAndLevelledActivateCompose:
    """The two IDLE gates on ONE machine instance (fw-integrate, 2026-11-25):
    ARMING_CONTRACT A6 guards every transition into HOMING (StateMachine
    `_homing_entry_refused`), C-LEVEL-1.A guards the operator's `activate`
    (IdleHandler, choreography step 0). Neither opens or closes the other:
    a loaded correction never lets `home` past Closed-loop Legs, and six
    settled Idle Legs never let `activate` past a missing correction."""

    def test_each_gate_refuses_its_own_command_and_neither_weakens_the_other(self):
        notes = []
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        ctx.notify = notes.append
        _boot_ready(ctx)
        ctx.is_homed = True                          # BOOT -> IDLE with no Homing
        _legs(ctx, [_IDLE] * 6)                      # A7: BOOT claims IDLE at Stow
        ctx.gravity_correction_loaded = False        # a fresh status, nothing loaded
        ctx.traj_status_age_s = 0.1
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        _legs(ctx, [_CL] * 6)                        # a manual /activate in IDLE

        # (a) Legs Closed-loop, no correction: `home` refused by A6 alone ...
        ctx.enqueue_command('home')
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        assert ctx.drain_requests() == []
        assert len(notes) == 1 and 'Closed-loop' in notes[0]
        assert not notes[0].startswith('activate refused')
        # ... and `activate` refused by C-LEVEL-1.A alone (no fetch, nothing moved).
        ctx.enqueue_command('activate')
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        assert ctx.drain_requests() == []
        assert len(notes) == 2 and notes[1].startswith('activate refused')
        assert 'gravity_correction_loaded is false' in notes[1]

        # (b) A loaded correction does not open A6.
        _levelled(ctx)
        _legs(ctx, [_CL] * 6)
        ctx.enqueue_command('home')
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        assert ctx.drain_requests() == []
        assert len(notes) == 3 and 'Closed-loop' in notes[2]

        # (c) Six settled Idle Legs do not open C-LEVEL-1.A.
        ctx.gravity_correction_loaded = False
        _legs(ctx, [_IDLE] * 6)
        ctx.enqueue_command('activate')
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        assert ctx.drain_requests() == []
        assert len(notes) == 4 and notes[3].startswith('activate refused')

        # (d) A6 passes for `home` with the correction still missing.
        ctx.enqueue_command('home')
        sm.tick(ctx)
        assert sm.state == RobotState.HOMING
        assert len(notes) == 4                       # no notice on a pass

        # (e) Back in IDLE (forced; HOMING's own flow is covered elsewhere), a
        # fresh loaded correction lets `activate` fetch and leave for ACTIVE.
        sm.force_transition(RobotState.IDLE, ctx)
        sm.tick(ctx)                                 # IDLE on_enter
        ctx.clear_requests()
        _levelled(ctx)
        ctx.enqueue_command('activate')
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        assert ctx.drain_requests() == ['fetch_activate_targets']
        ctx.activate_leg_targets_rev = list(_LEVELLED_TARGETS)
        ctx.operation_result = True
        sm.tick(ctx)
        assert sm.state == RobotState.ACTIVE
        assert len(notes) == 4


class TestBootIdleGate:
    """ARMING_CONTRACT A7 (owner rule 2026-11-25): IDLE means "Platform at
    Stow, Legs Idle", so BOOT enters it only when all six Legs are known,
    fresh and Idle. Each test fails if BootHandler's `if ctx.is_homed:` branch
    returns IDLE unconditionally (the pre-A7 code)."""

    @staticmethod
    def _homed_boot(states):
        notes = []
        sm = build_default_machine(log_fn=lambda msg: None)
        ctx = Context()
        ctx.notify = notes.append
        _boot_ready(ctx)
        ctx.is_homed = True                       # the flag survived
        if states is not None:
            _legs(ctx, states)
        return sm, ctx, notes

    def test_relaunch_under_a_posed_platform_holds_boot_and_names_deactivate(self):
        sm, ctx, notes = self._homed_boot([_CL] * 6)
        for _ in range(4):
            sm.tick(ctx)
            assert sm.state == RobotState.BOOT
            assert _requests_moving_legs(ctx.drain_requests()) == []
        assert len(notes) == 1                    # once per episode
        assert notes[0].startswith('IDLE refused: all six Legs are Closed-loop')
        assert '/deactivate' in notes[0] and 'A7' in notes[0]
        _legs(ctx, [_IDLE] * 6)                   # the operator's profiled Stow
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        assert len(notes) == 1

    def test_one_leg_dropped_five_holding_holds_boot_with_the_fallback(self):
        sm, ctx, notes = self._homed_boot([_IDLE] + [_CL] * 5)
        sm.tick(ctx)
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        assert len(notes) == 1
        assert 'Leg(s) 1, 2, 3, 4, 5 are Closed-loop' in notes[0]
        assert 'E-stop button' in notes[0] and 'reference is proven kept' in notes[0]

    @pytest.mark.parametrize('states,age', [
        (None, 0.0),                                   # never reported
        ([_IDLE] * 5, 0.0),                            # too short
        ([_IDLE] * 6, 5.0),                            # stale /robot_state
        ([0] + [_IDLE] * 5, 0.0),                      # a Leg UNDEFINED
    ])
    def test_unknown_or_not_idle_legs_hold_boot(self, states, age):
        sm, ctx, notes = self._homed_boot(None)
        if states is not None:
            ctx.leg_states = tuple(states)
            ctx.leg_states_time = time.monotonic() - age
        sm.tick(ctx)
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        assert len(notes) == 1 and notes[0].startswith('IDLE refused')

    def test_relaunch_at_stow_enters_idle_with_no_notice(self):
        sm, ctx, notes = self._homed_boot([_IDLE] * 6)
        ctx.legs_idle_since = time.monotonic()   # no settle needed for IDLE
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        assert notes == []

    def test_fault_exit_with_posed_legs_from_idle_stows_then_claims_idle(self):
        """A FAULT visit that began outside ACTIVE/LEVELLING with all six Legs
        Closed-loop (a manual /activate in IDLE) stows them first since FW 53,
        and BOOT claims IDLE only once all six read Idle (A7)."""
        sm, ctx, notes = self._homed_boot([_IDLE] * 6)
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE
        _legs(ctx, [_CL] * 6)
        ctx.legs_closed_loop = True
        sm.force_transition(RobotState.FAULT, ctx)
        ctx.errors = []
        reqs = []
        for _ in range(3):
            sm.tick(ctx)
            reqs += _dispatch(ctx)
        assert sm.state == RobotState.FAULT          # held while the stow runs
        assert reqs.count('deactivate') == 1
        _resolve(ctx, True)
        ctx.legs_closed_loop = False
        _legs(ctx, [_IDLE] * 6)
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE

    def test_predicate_rows(self):
        from jugglebot.state_machine import (
            idle_refusal, leg_idle_blockers, IDLE_STALE_PREFIX)
        assert idle_refusal([_IDLE] * 6, 0.0) is None
        assert idle_refusal([_IDLE] * 6, 5.0).startswith(IDLE_STALE_PREFIX)
        assert idle_refusal([_IDLE] * 6, None).startswith(IDLE_STALE_PREFIX)
        assert 'not known' in idle_refusal(None, 0.0)
        # the shared classifier: a mapping of present Legs, None = unknown
        assert leg_idle_blockers({0: _IDLE, 3: _IDLE}) == []
        assert leg_idle_blockers({0: _CL, 3: _IDLE}) == [(0, _CL)]
        assert leg_idle_blockers({0: _CL, 3: _IDLE}, exempt={0}) == []
        assert leg_idle_blockers({2: None}) == [(2, None)]
        assert leg_idle_blockers([_IDLE, _CL]) == [(1, _CL)]

    def test_homing_wording_is_unchanged_by_the_shared_predicate(self):
        from jugglebot.state_machine import homing_refusal
        assert homing_refusal([_CL] * 6, None, 0.0) == (
            'Homing refused: all six Legs are Closed-loop, so the Platform '
            'may be held up at a pose and Homing could collapse it '
            '(ARMING_CONTRACT A6). Stow it first with the profiled '
            'deactivate (ros2 service call /deactivate '
            'std_srvs/srv/Trigger); Homing starts by itself from BOOT once '
            'every Leg is Idle.')
        assert homing_refusal([_IDLE] + [_CL] * 5, None, 0.0).endswith(
            'clear_errors; Homing starts by itself once all six Legs have '
            'been Idle for 2 s.')


class TestMixedCaseRecovery:
    """FW 53 U6b, recovery R3 (owner 2026-11-25): one Leg dropped out by itself
    while five held the Platform up. After the operator's clear_errors, the FAULT
    exit re-engages it and stows all six (the profiled deactivate) only when its
    position reference is proven kept (jugglebot/leg_reference.py); otherwise BOOT
    holds with the E-stop-button remedy and one notice says why. Once per visit."""

    _MIXED = (_CL, _CL, _IDLE, _CL, _CL, _CL)

    @staticmethod
    def _feed(ctx, states, disarm2=0, pos2=2.10, active2=0):
        ctx.leg_drops.observe(states, [0, 0, active2, 0, 0, 0],
                              [0, 0, disarm2, 0, 0, 0],
                              [2.19, 2.19, pos2, 2.19, 2.19, 2.19], False)
        _legs(ctx, states)

    def _dropped_in_fault(self, bits=4096):
        """ACTIVE, Leg 2 disarms with ``bits``, forced FAULT, operator clear_errors
        dispatched and resolved, errors and the guard cleared. Returns (sm, ctx)
        at the tick before the exit decides."""
        sm, ctx = TestFaultExitStow._armed_active()
        self._feed(ctx, (_CL,) * 6, pos2=2.19)
        self._feed(ctx, self._MIXED, disarm2=bits)
        ctx.legs_closed_loop = False
        ctx.errors = ['Leg 2 disarmed']
        ctx.fatal_error = True
        ctx.guard_latched = True
        _force_fault(sm, ctx)
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT
        assert _dispatch(ctx) == ['disarm_setpoints']
        ctx.operator_notices.clear()
        ctx.enqueue_command('clear_errors')
        sm.tick(ctx)
        assert _dispatch(ctx) == ['clear_errors']
        _resolve(ctx, True)
        # The clear landed: drive errors and the disarm reason gone, guard released.
        self._feed(ctx, self._MIXED, disarm2=0)
        ctx.errors = []
        ctx.fatal_error = False
        ctx.guard_latched = False
        return sm, ctx

    def test_proven_drop_stows_all_six_then_boot_idle(self):
        """Mutation caught: the mixed branch removed (BOOT, no deactivate); the
        seal at clear_errors removed (the cleared reason reads as a lapse, refused)."""
        sm, ctx = self._dropped_in_fault()
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT
        assert _dispatch(ctx) == ['deactivate']
        assert any('Mixed-case recovery: Leg(s) 2' in n for n in ctx.operator_notices)
        sm.tick(ctx)
        assert _dispatch(ctx) == []                  # held, no second request
        _resolve(ctx, True)
        self._feed(ctx, (_IDLE,) * 6, pos2=0.0)
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
        sm.tick(ctx)
        assert sm.state == RobotState.IDLE

    @pytest.mark.parametrize('bits,why', [(8, 'MISSING_ESTIMATE'),
                                          (67108864, 'SPINOUT_DETECTED')])
    def test_unproven_drop_holds_and_names_the_reason(self, bits, why):
        """Mutation caught: the refusal ignored (a MISSING_ESTIMATE Leg re-energised)."""
        sm, ctx = self._dropped_in_fault(bits=bits)
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        assert _dispatch(ctx) == []
        said = [n for n in ctx.operator_notices if 'not offered' in n]
        assert len(said) == 1 and 'Leg 2: ' in said[0] and why in said[0]
        ctx.all_heartbeats = True
        ctx.firmware_validated = True
        ctx.link_status_seen = True
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT           # A7 holds the mixed set
        assert any('E-stop button' in n for n in ctx.operator_notices)

    def test_unhomed_drop_is_not_offered(self):
        sm, ctx = self._dropped_in_fault()
        ctx.is_homed = False
        sm.tick(ctx)
        assert _dispatch(ctx) == []
        assert any('homed flag is clear' in n for n in ctx.operator_notices)

    def test_failed_recovery_is_not_retried_in_the_same_visit(self):
        """The Leg re-faulted on re-engage (deactivate failed, set still mixed): the
        next clear_errors goes to BOOT with a notice, no second deactivate.
        Mutation caught: the _mixed_tried check removed."""
        sm, ctx = self._dropped_in_fault()
        sm.tick(ctx)
        assert _dispatch(ctx) == ['deactivate']
        _resolve(ctx, False)
        sm.tick(ctx)
        assert sm.state == RobotState.FAULT          # stow failed: held
        assert _dispatch(ctx) == []
        ctx.operator_notices.clear()
        ctx.enqueue_command('clear_errors')
        sm.tick(ctx)
        reqs = _dispatch(ctx)
        assert 'deactivate' not in reqs
        if 'clear_errors' in reqs:
            _resolve(ctx, True)
            sm.tick(ctx)
            assert _dispatch(ctx) == []
        assert sm.state == RobotState.BOOT
        assert any('not offered again' in n for n in ctx.operator_notices)

    def test_failed_recovery_leaving_six_closed_loop_uses_the_all_six_stow(self):
        """A descent timeout freezes all six Closed-loop: the next clear_errors
        runs the existing all-six exit stow."""
        sm, ctx = self._dropped_in_fault()
        sm.tick(ctx)
        assert _dispatch(ctx) == ['deactivate']
        _resolve(ctx, False)
        self._feed(ctx, (_CL,) * 6, pos2=1.0)
        ctx.legs_closed_loop = True
        sm.tick(ctx)
        ctx.enqueue_command('clear_errors')
        sm.tick(ctx)
        reqs = _dispatch(ctx)
        if reqs == ['clear_errors']:
            _resolve(ctx, True)
            sm.tick(ctx)
            reqs = _dispatch(ctx)
        assert reqs == ['deactivate']
        assert sm.state == RobotState.FAULT

    def test_reason_cleared_outside_the_orchestrator_is_not_proven(self):
        """The disarm reason went to 0 with no clear_errors through FAULT (a drive
        reboot, or a clear sent around the orchestrator): not proven."""
        sm, ctx = TestFaultExitStow._armed_active()
        self._feed(ctx, (_CL,) * 6, pos2=2.19)
        self._feed(ctx, self._MIXED, disarm2=4096)
        ctx.legs_closed_loop = False
        ctx.errors = ['Leg 2 disarmed']
        ctx.fatal_error = True
        _force_fault(sm, ctx)
        sm.tick(ctx)
        _dispatch(ctx)
        self._feed(ctx, self._MIXED, disarm2=0)    # no clear_errors through FAULT
        ctx.errors = []
        ctx.fatal_error = False
        sm.tick(ctx)
        assert sm.state == RobotState.BOOT
        assert _dispatch(ctx) == []
        assert any('cleared before' in n for n in ctx.operator_notices)
