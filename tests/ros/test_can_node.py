"""Tests for jugglebot.can_node.CanInterfaceNode — CAN message dispatch and logic."""

import math
import struct
import time
from unittest.mock import MagicMock, patch, call

import can
import pytest

import jugglebot.protocol_config as proto
import jugglebot.hardware_config as hw


# ════════════════════════════════════════════════════════════════
# Node fixture — construct CanInterfaceNode with all externals mocked
# ════════════════════════════════════════════════════════════════


@pytest.fixture
def node():
    """Create a CanInterfaceNode with mocked CAN bus.

    The constructor calls bus.setup() which tries to create a real CAN bus.
    We patch can.Bus so it succeeds without hardware.  After construction
    we replace node.bus with a fresh MagicMock so tests can track sends.
    """
    with patch('jugglebot.can.bus.can.Bus') as mock_bus_cls:
        mock_bus_cls.return_value.recv.return_value = None  # flush() needs this
        from jugglebot.can_node import CanInterfaceNode
        n = CanInterfaceNode()
    n.bus = MagicMock()
    return n


def _odrive_msg(axis_id, command_name, data):
    """Build a CAN message with ODrive-style arbitration ID."""
    from jugglebot.can import odrive
    aid = odrive.arb_id(axis_id, command_name)
    return can.Message(arbitration_id=aid, data=data, is_extended_id=False)


# ════════════════════════════════════════════════════════════════
# Message dispatch (_handle_message)
# ════════════════════════════════════════════════════════════════


class TestHandleMessage:
    def test_routes_traffic_report(self, node):
        data = struct.pack('<HH4x', 100, 500)
        msg = can.Message(arbitration_id=node._traffic_report_id, data=data)
        node._handle_message(msg)
        # Check the traffic pub received a message
        assert len(node.can_traffic_pub.published) == 1

    def test_routes_tilt_reading(self, node):
        data = struct.pack('<ff', 0.01, -0.02)
        msg = can.Message(arbitration_id=node._tilt_reading_id, data=data)
        node._handle_message(msg)
        assert node._tilt_reading == pytest.approx((0.01, -0.02), abs=1e-6)

    def test_tilt_reading_ignores_request_echo(self, node):
        """The request byte (0x01) is filtered out."""
        msg = can.Message(arbitration_id=node._tilt_reading_id, data=b'\x01')
        node._handle_message(msg)
        assert node._tilt_reading is None

    def test_routes_state_update(self, node):
        flags = 0x03  # is_homed=True, levelling_complete=True
        tx, ty = 10, -20
        data = struct.pack('<Bhh3x', flags, tx, ty)
        msg = can.Message(arbitration_id=node._state_update_id, data=data)
        node._handle_message(msg)
        assert node.last_known_state['is_homed'] is True
        assert node.last_known_state['levelling_complete'] is True

    def test_routes_hand_input_pos(self, node):
        pos, vel_ff, tor_ff = 1.5, 200, 50
        data = struct.pack('<fhh', pos, vel_ff, tor_ff)
        msg = can.Message(arbitration_id=node._hand_input_pos_id, data=data)
        node._handle_message(msg)
        assert node._last_hand_cmd['pos'] == pytest.approx(1.5)

    def test_routes_bb_heartbeat(self, node):
        from jugglebot.can import ball_butler
        state_byte = (1 << 1) | 0x01  # IDLE, ball_in_hand=True
        data = struct.pack('<BBHHH', state_byte, 0, 100, 50, 20)
        msg = can.Message(arbitration_id=ball_butler.HEARTBEAT_ID, data=data)
        node._handle_message(msg)
        assert node.last_bb_heartbeat.ball_in_hand is True

    def test_routes_odrive_heartbeat(self, node):
        data = struct.pack('<IBBBB', 0, 8, 0, 0x01, 0)  # CLOSED_LOOP, traj_done
        msg = _odrive_msg(0, 'heartbeat_message', data)
        node._handle_message(msg)
        assert node.motors.get_field(0, 'current_state') == 8

    def test_filters_irrelevant_bb_motor_cmd(self, node):
        """BB motor axes (7, 8) should ignore commands not in BB_RELEVANT_CMD_IDS."""
        from jugglebot.can import odrive
        # set_input_pos (0x0C) is NOT in BB_RELEVANT_CMD_IDS
        aid = (7 << 5) | proto.ODRIVE_COMMANDS['set_input_pos']
        msg = can.Message(arbitration_id=aid, data=bytes(8))
        # Should not raise, and should not update motor state
        node._handle_message(msg)

    def test_passes_relevant_bb_motor_cmd(self, node):
        """BB motor heartbeat (relevant cmd) SHOULD be processed."""
        data = struct.pack('<IBBBB', 0, 1, 0, 0x00, 0)  # IDLE
        msg = _odrive_msg(7, 'heartbeat_message', data)
        node._handle_message(msg)
        assert node.motors.get_field(7, 'current_state') == 1


# ════════════════════════════════════════════════════════════════
# _handle_heartbeat
# ════════════════════════════════════════════════════════════════


class TestHandleHeartbeat:
    def test_updates_motor_state(self, node):
        data = struct.pack('<IBBBB', 0, 8, 0, 0x01, 0)
        node._handle_heartbeat(0, data)
        assert node.motors.get_field(0, 'current_state') == 8
        assert node.motors.get_field(0, 'trajectory_done') is True

    def test_sets_received_heartbeat_flag(self, node):
        data = struct.pack('<IBBBB', 0, 1, 0, 0x00, 0)
        node._handle_heartbeat(0, data)
        assert node.motors.received_heartbeats[0] is True

    def test_records_heartbeat_timing(self, node):
        data = struct.pack('<IBBBB', 0, 1, 0, 0x00, 0)
        node._handle_heartbeat(0, data)
        assert node.motors.first_heartbeat_received is True


# ════════════════════════════════════════════════════════════════
# _handle_error — complex error classification
# ════════════════════════════════════════════════════════════════


class TestHandleError:
    def test_no_errors_clears_flags(self, node):
        """active_errors=0, disarm_reason=0 → all error flags cleared."""
        node.motors.fatal_error = True
        node.motors.undervoltage_error = True
        data = struct.pack('<II', 0, 0)
        node._handle_error(0, data)
        assert node.motors.fatal_error is False
        assert node.motors.undervoltage_error is False

    def test_active_error_sets_fatal(self, node):
        """Any non-zero active_errors → fatal_error=True."""
        data = struct.pack('<II', 1024, 0)  # DC_BUS_OVER_CURRENT
        node._handle_error(0, data)
        assert node.motors.fatal_error is True

    def test_undervoltage_active_sets_flag(self, node):
        """DC_BUS_UNDER_VOLTAGE in active_errors → undervoltage_error=True."""
        data = struct.pack('<II', 512, 0)
        node._handle_error(0, data)
        assert node.motors.undervoltage_error is True

    def test_disarm_no_closed_loop_soft_reset(self, node):
        """Disarmed but no CLOSED_LOOP → try clearing errors (soft reset)."""
        from jugglebot.can import odrive
        # All axes IDLE (state=1), axis 0 has disarm_reason
        for axis_id in odrive.JUGGLEBOT_AXES:
            node.motors.update(axis_id, current_state=odrive.AXIS_STATES['IDLE'])
        node.motors.soft_reset_attempts = 0
        data = struct.pack('<II', 0, 1024)  # disarm_reason only
        node._handle_error(0, data)
        # Should have called _clear_errors via bus.send
        assert node.bus.send.call_count > 0

    def test_disarm_no_closed_loop_exceeds_attempts(self, node):
        """Soft reset exhausted → fatal_error=True."""
        from jugglebot.can import odrive
        for axis_id in odrive.JUGGLEBOT_AXES:
            node.motors.update(axis_id, current_state=odrive.AXIS_STATES['IDLE'])
        node.motors.soft_reset_attempts = 1  # Already at max (max=1)
        data = struct.pack('<II', 0, 1024)
        node._handle_error(0, data)
        assert node.motors.fatal_error is True

    def test_disarm_with_closed_loop_fatal(self, node):
        """Disarmed AND at least one axis in CLOSED_LOOP → fatal."""
        from jugglebot.can import odrive
        # Put axis 0 in CLOSED_LOOP
        node.motors.update(0, current_state=odrive.AXIS_STATES['CLOSED_LOOP'])
        # Use a non-undervoltage disarm reason (1024 = DC_BUS_OVER_CURRENT)
        # to avoid the undervoltage auto-clear path
        data = struct.pack('<II', 0, 1024)
        node._handle_error(0, data)
        assert node.motors.fatal_error is True

    def test_undervoltage_disarm_only_clears(self, node):
        """Undervoltage in disarm_reason but NO active_errors → auto-clear."""
        from jugglebot.can import odrive
        for axis_id in odrive.JUGGLEBOT_AXES:
            node.motors.update(axis_id, current_state=odrive.AXIS_STATES['IDLE'],
                               active_errors=0)
        data = struct.pack('<II', 0, 512)  # disarm=undervoltage, active=0
        node._handle_error(0, data)
        assert node.motors.undervoltage_error is False
        assert node.motors.fatal_error is False

    def test_error_names_appended_to_state(self, node):
        """Known error codes should be appended to last_known_state['error']."""
        data = struct.pack('<II', 512, 0)  # DC_BUS_UNDER_VOLTAGE
        node._handle_error(0, data)
        assert "DC_BUS_UNDER_VOLTAGE" in node.last_known_state['error']


# ════════════════════════════════════════════════════════════════
# _handle_encoder — leg inversion
# ════════════════════════════════════════════════════════════════


class TestHandleEncoder:
    def test_leg_inversion(self, node):
        """Leg axes (0-5): pos and vel are negated."""
        data = struct.pack('<ff', 1.5, 0.25)
        node._handle_encoder(0, data)
        assert node.motors.get_field(0, 'pos_estimate') == pytest.approx(-1.5)
        assert node.motors.get_field(0, 'vel_estimate') == pytest.approx(-0.25)

    def test_hand_no_inversion(self, node):
        """Hand axis (6): pos and vel are NOT negated."""
        data = struct.pack('<ff', 1.5, 0.25)
        node._handle_encoder(6, data)
        assert node.motors.get_field(6, 'pos_estimate') == pytest.approx(1.5)
        assert node.motors.get_field(6, 'vel_estimate') == pytest.approx(0.25)

    def test_all_leg_axes_inverted(self, node):
        """Every leg axis (0-5) should have inverted encoder values."""
        from jugglebot.can import odrive
        for axis_id in odrive.LEG_AXES:
            data = struct.pack('<ff', 2.0, 1.0)
            node._handle_encoder(axis_id, data)
            assert node.motors.get_field(axis_id, 'pos_estimate') == pytest.approx(-2.0)
            assert node.motors.get_field(axis_id, 'vel_estimate') == pytest.approx(-1.0)

    def test_encoder_data_received_tracking(self, node):
        """_encoder_data_received[i] should be set True after receiving leg data."""
        assert node._encoder_data_received[0] is False
        data = struct.pack('<ff', 1.0, 0.0)
        node._handle_encoder(0, data)
        assert node._encoder_data_received[0] is True


# ════════════════════════════════════════════════════════════════
# Other ODrive handlers
# ════════════════════════════════════════════════════════════════


class TestHandleIq:
    def test_updates_motor_state(self, node):
        data = struct.pack('<ff', 5.0, 4.8)
        node._handle_iq(0, data)
        assert node.motors.get_field(0, 'iq_setpoint') == pytest.approx(5.0)
        assert node.motors.get_field(0, 'iq_measured') == pytest.approx(4.8)


class TestHandleTemps:
    def test_updates_motor_state(self, node):
        data = struct.pack('<ff', 45.0, 38.0)
        node._handle_temps(0, data)
        assert node.motors.get_field(0, 'fet_temp') == pytest.approx(45.0)
        assert node.motors.get_field(0, 'motor_temp') == pytest.approx(38.0)


class TestHandleBusVc:
    def test_updates_motor_state(self, node):
        data = struct.pack('<ff', 48.0, 2.5)
        node._handle_bus_vc(0, data)
        assert node.motors.get_field(0, 'bus_voltage') == pytest.approx(48.0)
        assert node.motors.get_field(0, 'bus_current') == pytest.approx(2.5)


class TestHandleSdoResponse:
    def test_nan_sets_false(self, node):
        """NaN value → encoder_search_feedback[axis_id] = False (not found)."""
        ep_id = proto.ENDPOINT_COMMUTATION_MAPPER_POS_ABS
        data = struct.pack('<BHBf', 0, ep_id, 0, float('nan'))
        node._handle_sdo_response(0, data)
        assert node.motors.encoder_search_feedback[0] is False

    def test_valid_value_sets_true(self, node):
        """Non-NaN value → encoder_search_feedback[axis_id] = True (found)."""
        ep_id = proto.ENDPOINT_COMMUTATION_MAPPER_POS_ABS
        data = struct.pack('<BHBf', 0, ep_id, 0, 1.23)
        node._handle_sdo_response(0, data)
        assert node.motors.encoder_search_feedback[0] is True

    def test_ignores_other_endpoints(self, node):
        """SDO responses for other endpoints should not affect encoder feedback."""
        node.motors.encoder_search_feedback[0] = None
        data = struct.pack('<BHBf', 0, proto.ENDPOINT_GPIO_STATES, 0, 1.0)
        node._handle_sdo_response(0, data)
        assert node.motors.encoder_search_feedback[0] is None  # Unchanged


# ════════════════════════════════════════════════════════════════
# _sub_control_mode — control mode switching
# ════════════════════════════════════════════════════════════════


class TestSubControlMode:
    def _make_mode_msg(self, mode_str):
        from tests.ros.conftest import MockString
        msg = MockString(data=mode_str)
        return msg

    def test_error_mode_calls_gentle_move(self, node):
        """ERROR mode should trigger stowing (gentle move to 0)."""
        with patch.object(node, '_gently_move_to_setpoint') as mock_move:
            node._sub_control_mode(self._make_mode_msg('ERROR'))
            mock_move.assert_called_once_with(0.0, deactivating=True)
            assert node.stowed_due_to_error is True

    def test_spacemouse_mode_legs_closed_loop(self, node):
        """SPACEMOUSE puts legs in CLOSED_LOOP, hand in IDLE."""
        from jugglebot.can import odrive
        # Start with all IDLE
        for axis_id in odrive.JUGGLEBOT_AXES:
            node.motors.update(axis_id, current_state=odrive.AXIS_STATES['IDLE'])
        node.motors.get_states()  # Refresh last_states

        node._sub_control_mode(self._make_mode_msg('SPACEMOUSE'))
        send_calls = node.bus.send.call_args_list
        # Should have sent CLOSED_LOOP for legs (6 commands)
        assert len(send_calls) >= 6

    def test_standby_mode_legs_closed_loop(self, node):
        """STANDBY (default sub-mode on activate) puts legs in CLOSED_LOOP.

        Without this, activating would trigger the 'Unknown control mode'
        branch and stow the platform.
        """
        from jugglebot.can import odrive
        for axis_id in odrive.JUGGLEBOT_AXES:
            node.motors.update(axis_id, current_state=odrive.AXIS_STATES['IDLE'])
        node.motors.get_states()

        with patch.object(node, '_gently_move_to_setpoint') as mock_move:
            node._sub_control_mode(self._make_mode_msg('STANDBY'))
            mock_move.assert_not_called()

        send_calls = node.bus.send.call_args_list
        assert len(send_calls) >= 6

    def test_shell_mode_legs_closed_loop(self, node):
        """SHELL puts legs in CLOSED_LOOP, hand in IDLE."""
        from jugglebot.can import odrive
        for axis_id in odrive.JUGGLEBOT_AXES:
            node.motors.update(axis_id, current_state=odrive.AXIS_STATES['IDLE'])
        node.motors.get_states()

        node._sub_control_mode(self._make_mode_msg('SHELL'))
        send_calls = node.bus.send.call_args_list
        assert len(send_calls) >= 6

    def test_spacemouse_hand_idle_if_closed_loop(self, node):
        """SPACEMOUSE: if hand is in CLOSED_LOOP, send it to IDLE."""
        from jugglebot.can import odrive
        for axis_id in odrive.LEG_AXES:
            node.motors.update(axis_id, current_state=odrive.AXIS_STATES['CLOSED_LOOP'])
        node.motors.update(odrive.HAND_AXIS, current_state=odrive.AXIS_STATES['CLOSED_LOOP'])
        node.motors.get_states()

        node._sub_control_mode(self._make_mode_msg('SPACEMOUSE'))
        # Should send IDLE to hand (1 command, legs already closed loop)
        send_calls = node.bus.send.call_args_list
        assert len(send_calls) >= 1

    def test_spacemouse_noop_if_already_correct(self, node):
        """SPACEMOUSE with legs already CLOSED_LOOP and hand IDLE → only hand IDLE skipped."""
        from jugglebot.can import odrive
        for axis_id in odrive.LEG_AXES:
            node.motors.update(axis_id, current_state=odrive.AXIS_STATES['CLOSED_LOOP'])
        node.motors.update(odrive.HAND_AXIS, current_state=odrive.AXIS_STATES['IDLE'])
        node.motors.get_states()

        node._sub_control_mode(self._make_mode_msg('SPACEMOUSE'))
        # Legs already CLOSED_LOOP, hand already IDLE: no commands needed
        assert node.bus.send.call_count == 0

    def test_legacy_mode_treated_as_unknown(self, node):
        """Legacy modes (STANDBY_ACTIVE, CATCH_DROPPED_BALL_NODE) now stow."""
        with patch.object(node, '_gently_move_to_setpoint') as mock_move:
            node._sub_control_mode(self._make_mode_msg('STANDBY_ACTIVE'))
            mock_move.assert_called_once_with(0.0, deactivating=True)

        with patch.object(node, '_gently_move_to_setpoint') as mock_move:
            node._sub_control_mode(self._make_mode_msg('CATCH_DROPPED_BALL_NODE'))
            mock_move.assert_called_once_with(0.0, deactivating=True)

    def test_empty_string_noop(self, node):
        """Empty string should do nothing."""
        node._sub_control_mode(self._make_mode_msg(''))
        node.bus.send.assert_not_called()

    def test_unknown_mode_stows(self, node):
        """Unknown mode should trigger stowing."""
        with patch.object(node, '_gently_move_to_setpoint') as mock_move:
            node._sub_control_mode(self._make_mode_msg('UNKNOWN_MODE'))
            mock_move.assert_called_once_with(0.0, deactivating=True)


# ════════════════════════════════════════════════════════════════
# _send_position_target — clipping + leg inversion
# ════════════════════════════════════════════════════════════════


class TestSendPositionTarget:
    def test_leg_clips_and_inverts(self, node):
        """Leg position is clipped then negated before sending."""
        node._send_position_target(0, 2.0)
        sent_msg = node.bus.send.call_args[0][0]
        pos = struct.unpack_from('<f', bytes(sent_msg.data))[0]
        assert pos == pytest.approx(-2.0)  # Negated

    def test_leg_clips_above_max(self, node):
        """Position above max is clipped to max, then negated."""
        node._send_position_target(0, 100.0)
        sent_msg = node.bus.send.call_args[0][0]
        pos = struct.unpack_from('<f', bytes(sent_msg.data))[0]
        assert pos == pytest.approx(-hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS)

    def test_hand_no_inversion(self, node):
        """Hand position is clipped but NOT negated."""
        node._send_position_target(6, 5.0)
        sent_msg = node.bus.send.call_args[0][0]
        pos = struct.unpack_from('<f', bytes(sent_msg.data))[0]
        assert pos == pytest.approx(5.0)  # NOT negated


# ════════════════════════════════════════════════════════════════
# _check_target_reached
# ════════════════════════════════════════════════════════════════


class TestCheckTargetReached:
    def test_none_in_targets_early_return(self, node):
        """Should do nothing if any target position is None."""
        node.legs_target_position = [None] * 6
        node._check_target_reached()
        # No publish should happen
        assert len(node.target_reached_pub.published) == 0

    def test_no_encoder_data_early_return(self, node):
        """Should do nothing if encoder data hasn't been received yet."""
        node.legs_target_position = [1.0] * 6
        node._encoder_data_received = [False] * 6
        node._check_target_reached()
        assert len(node.target_reached_pub.published) == 0

    def test_at_target(self, node):
        """When position and velocity are within tolerance → arrived."""
        from jugglebot.can import odrive
        target = 2.0
        node.legs_target_position = [target] * 6
        node._encoder_data_received = [True] * 6
        for axis_id in odrive.LEG_AXES:
            node.motors.update(axis_id, pos_estimate=target, vel_estimate=0.0)
        node.motors.get_states()

        node._check_target_reached()
        assert len(node.target_reached_pub.published) == 1
        msg = node.target_reached_pub.published[0]
        assert msg.leg0_has_arrived is True
        assert msg.leg5_has_arrived is True

    def test_not_at_target(self, node):
        """When position is far from target → not arrived."""
        from jugglebot.can import odrive
        node.legs_target_position = [2.0] * 6
        node._encoder_data_received = [True] * 6
        for axis_id in odrive.LEG_AXES:
            node.motors.update(axis_id, pos_estimate=0.0, vel_estimate=0.0)
        node.motors.get_states()

        node._check_target_reached()
        msg = node.target_reached_pub.published[0]
        assert msg.leg0_has_arrived is False

    def test_moving_too_fast(self, node):
        """Position ok but velocity above tolerance → not arrived."""
        from jugglebot.can import odrive
        target = 2.0
        node.legs_target_position = [target] * 6
        node._encoder_data_received = [True] * 6
        for axis_id in odrive.LEG_AXES:
            node.motors.update(axis_id, pos_estimate=target, vel_estimate=1.0)
        node.motors.get_states()

        node._check_target_reached()
        msg = node.target_reached_pub.published[0]
        assert msg.leg0_has_arrived is False


# ════════════════════════════════════════════════════════════════
# Teensy message parsing
# ════════════════════════════════════════════════════════════════


class TestDecodeTeensyState:
    def test_parses_homed_flag(self, node):
        flags = 0x01  # is_homed=True
        data = struct.pack('<Bhh3x', flags, 0, 0)
        msg = can.Message(arbitration_id=node._state_update_id, data=data)
        node._decode_teensy_state(msg)
        assert node.last_known_state['is_homed'] is True
        assert node.last_known_state['levelling_complete'] is False

    def test_parses_levelling_flag(self, node):
        flags = 0x02  # levelling_complete=True
        data = struct.pack('<Bhh3x', flags, 0, 0)
        msg = can.Message(arbitration_id=node._state_update_id, data=data)
        node._decode_teensy_state(msg)
        assert node.last_known_state['is_homed'] is False
        assert node.last_known_state['levelling_complete'] is True

    def test_parses_tilt_offsets(self, node):
        tx_mrad, ty_mrad = 10, -20
        data = struct.pack('<Bhh3x', 0, tx_mrad, ty_mrad)
        msg = can.Message(arbitration_id=node._state_update_id, data=data)
        node._decode_teensy_state(msg)
        assert node.last_known_state['pose_offset_rad'] == pytest.approx(
            (0.01, -0.02), abs=1e-6
        )


class TestHandleHandInputPos:
    def test_correct_unpacking(self, node):
        pos, vel_ff, tor_ff = 1.5, 200, 50
        data = struct.pack('<fhh', pos, vel_ff, tor_ff)
        msg = can.Message(arbitration_id=node._hand_input_pos_id, data=data)
        node._handle_hand_input_pos(msg)
        assert node._last_hand_cmd['pos'] == pytest.approx(1.5)

    def test_scaling(self, node):
        vel_ff_raw = 200
        tor_ff_raw = 50
        data = struct.pack('<fhh', 0.0, vel_ff_raw, tor_ff_raw)
        msg = can.Message(arbitration_id=node._hand_input_pos_id, data=data)
        node._handle_hand_input_pos(msg)
        assert node._last_hand_cmd['vel'] == pytest.approx(
            vel_ff_raw / proto.INPUT_SCALE_HAND_VEL
        )
        assert node._last_hand_cmd['tor'] == pytest.approx(
            tor_ff_raw / proto.INPUT_SCALE_HAND_TOR
        )


# ════════════════════════════════════════════════════════════════
# Generator state machines
# ════════════════════════════════════════════════════════════════


class TestRebootOdrivesSteps:
    def test_sends_reboot_commands(self, node):
        """_reboot_odrives_steps should send reboot to all Jugglebot axes."""
        from jugglebot.can import odrive
        gen = node._reboot_odrives_steps()
        delay = next(gen)
        # After first yield, reboot commands should have been sent
        assert node.bus.send.call_count == len(odrive.JUGGLEBOT_AXES)
        # Should yield a long delay (10s)
        assert isinstance(delay, (int, float))
        assert delay >= 5.0

    def test_resets_state_flags(self, node):
        """After the reboot delay, state flags should be cleared."""
        node.last_known_state['is_homed'] = True
        node.last_known_state['levelling_complete'] = True
        gen = node._reboot_odrives_steps()
        try:
            next(gen)
            # Drive to completion
            next(gen)
        except StopIteration as e:
            result = e.value
        assert node.last_known_state['is_homed'] is False
        assert node.last_known_state['levelling_complete'] is False


class TestGentleMoveSteps:
    def test_already_stowed_early_return(self, node):
        """If deactivating and legs idle and near zero → early return True."""
        from jugglebot.can import odrive
        # Put all legs in IDLE with pos near 0
        for axis_id in odrive.LEG_AXES:
            node.motors.update(axis_id,
                               current_state=odrive.AXIS_STATES['IDLE'],
                               pos_estimate=0.05)
        node.motors.get_states()

        gen = node._gentle_move_steps(0.0, deactivating=True)
        try:
            while True:
                next(gen)
        except StopIteration as e:
            result = e.value
        assert result is True


# ════════════════════════════════════════════════════════════════
# _clear_errors
# ════════════════════════════════════════════════════════════════


class TestClearErrors:
    def test_sends_clear_to_all_axes(self, node):
        """Should send clear_errors to all 7 Jugglebot axes."""
        from jugglebot.can import odrive
        node._clear_errors()
        assert node.bus.send.call_count == len(odrive.JUGGLEBOT_AXES)

    def test_clears_motor_flags(self, node):
        node.motors.fatal_error = True
        node.motors.undervoltage_error = True
        node._clear_errors()
        assert node.motors.fatal_error is False
        assert node.motors.undervoltage_error is False

    def test_clears_error_state(self, node):
        node.last_known_state['error'] = ['SOME_ERROR']
        node._clear_errors()
        assert node.last_known_state['error'] == []


# ════════════════════════════════════════════════════════════════
# _sub_leg_lengths
# ════════════════════════════════════════════════════════════════


class TestSubLegLengths:
    def test_sends_6_position_commands(self, node):
        from tests.ros.conftest import Float64MultiArray
        msg = Float64MultiArray(data=[1.0, 1.1, 1.2, 1.3, 1.4, 1.5])
        node._sub_leg_lengths(msg)
        # Should send one command per leg
        assert node.bus.send.call_count == 6

    def test_updates_target_position(self, node):
        from tests.ros.conftest import Float64MultiArray
        targets = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
        msg = Float64MultiArray(data=targets)
        node._sub_leg_lengths(msg)
        assert node.legs_target_position == targets


# ════════════════════════════════════════════════════════════════
# Publisher callbacks
# ════════════════════════════════════════════════════════════════


class TestPublishRobotState:
    def test_publishes_state(self, node):
        node._publish_robot_state()
        assert len(node.robot_state_pub.published) == 1

    def test_includes_undervoltage_error(self, node):
        node.motors.undervoltage_error = True
        node._publish_robot_state()
        msg = node.robot_state_pub.published[0]
        assert any("Undervoltage" in e for e in msg.error)

    def test_includes_fatal_error(self, node):
        node.motors.fatal_error = True
        node.last_known_state['error'] = ['TEST_ERROR']
        node._publish_robot_state()
        msg = node.robot_state_pub.published[0]
        assert any("Fatal" in e for e in msg.error)


class TestPublishBbHeartbeat:
    def test_publishes_heartbeat(self, node):
        node._publish_bb_heartbeat()
        assert len(node.bb_heartbeat_pub.published) == 1

    def test_reflects_last_heartbeat(self, node):
        from jugglebot.can.ball_butler import BallButlerHeartbeat, BallButlerStates
        node.last_bb_heartbeat = BallButlerHeartbeat(
            ball_in_hand=True,
            state=BallButlerStates.IDLE,
            yaw_deg=45.0,
        )
        node._publish_bb_heartbeat()
        msg = node.bb_heartbeat_pub.published[0]
        assert msg.ball_in_hand is True
        assert msg.yaw_deg == pytest.approx(45.0)


# ════════════════════════════════════════════════════════════════
# _set_leg_gains — per-leg PID gain application over CAN
# ════════════════════════════════════════════════════════════════


class TestSetLegGains:
    def test_emits_12_frames_in_order(self, node):
        """6 legs × 2 frames (pos_gain, vel_gains) in leg-0..leg-5 order."""
        from jugglebot.can import odrive

        node.bus.send = MagicMock(return_value=True)
        node._set_leg_gains()

        calls = node.bus.send.call_args_list
        assert len(calls) == 12, f"expected 12 frames, got {len(calls)}"

        for i, axis_id in enumerate(odrive.LEG_AXES):
            pos_msg = calls[2 * i].args[0]
            vel_msg = calls[2 * i + 1].args[0]
            assert pos_msg.arbitration_id == odrive.arb_id(axis_id, 'set_pos_gain')
            assert vel_msg.arbitration_id == odrive.arb_id(axis_id, 'set_vel_gains')

            pos_gain = struct.unpack_from('<f', pos_msg.data, 0)[0]
            vg, vig = struct.unpack_from('<ff', vel_msg.data, 0)
            assert pos_gain == pytest.approx(node.leg_gains[i]['pos_gain'])
            assert vg == pytest.approx(node.leg_gains[i]['vel_gain'])
            assert vig == pytest.approx(node.leg_gains[i]['vel_int_gain'])

    def test_uses_config_gains(self, node):
        """Gains sent must match hardware_config arrays."""
        node.bus.send = MagicMock(return_value=True)
        node._set_leg_gains()
        calls = node.bus.send.call_args_list
        for i in range(6):
            pos_msg = calls[2 * i].args[0]
            vel_msg = calls[2 * i + 1].args[0]
            pos_gain = struct.unpack_from('<f', pos_msg.data, 0)[0]
            vg, vig = struct.unpack_from('<ff', vel_msg.data, 0)
            assert pos_gain == pytest.approx(hw.ODRIVE_LEG_POS_GAINS[i])
            assert vg == pytest.approx(hw.ODRIVE_LEG_VEL_GAINS[i])
            assert vig == pytest.approx(hw.ODRIVE_LEG_VEL_INT_GAINS[i])

    def test_raises_on_can_error(self, node):
        """If any send() returns False, _set_leg_gains raises RuntimeError
        naming the failed leg so the caller can surface it via the service."""
        # Fail only the 6th frame (leg 2's vel_gains — index 5 in the 12-frame sequence)
        results = [True] * 12
        results[5] = False
        node.bus.send = MagicMock(side_effect=results)

        with pytest.raises(RuntimeError, match="leg2.vel_gains"):
            node._set_leg_gains()

    def test_raises_lists_all_failures(self, node):
        """Failed writes from multiple legs are reported together."""
        results = [True] * 12
        results[0] = False   # leg0.pos_gain
        results[3] = False   # leg1.vel_gains
        results[10] = False  # leg5.pos_gain
        node.bus.send = MagicMock(side_effect=results)

        with pytest.raises(RuntimeError) as excinfo:
            node._set_leg_gains()
        msg = str(excinfo.value)
        assert "leg0.pos_gain" in msg
        assert "leg1.vel_gains" in msg
        assert "leg5.pos_gain" in msg


class TestSetHandGains:
    def test_emits_two_frames(self, node):
        from jugglebot.can import odrive
        node.bus.send = MagicMock(return_value=True)
        node._set_hand_gains()
        calls = node.bus.send.call_args_list
        assert len(calls) == 2
        assert calls[0].args[0].arbitration_id == odrive.arb_id(
            odrive.HAND_AXIS, 'set_pos_gain')
        assert calls[1].args[0].arbitration_id == odrive.arb_id(
            odrive.HAND_AXIS, 'set_vel_gains')

    def test_raises_on_can_error(self, node):
        node.bus.send = MagicMock(side_effect=[False, True])
        with pytest.raises(RuntimeError, match="hand.pos_gain"):
            node._set_hand_gains()
