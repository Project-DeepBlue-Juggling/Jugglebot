"""Tests for jugglebot.can.odrive — ODrive CAN protocol encode/decode."""

import math
import struct

import pytest

from jugglebot.can import odrive
import jugglebot.protocol_config as proto
import jugglebot.hardware_config as hw


# ════════════════════════════════════════════════════════════════
# arb_id()
# ════════════════════════════════════════════════════════════════


class TestArbId:
    def test_axis0_heartbeat(self):
        assert odrive.arb_id(0, 'heartbeat_message') == 0x01

    def test_axis5_set_input_pos(self):
        expected = (5 << 5) | proto.ODRIVE_COMMANDS['set_input_pos']
        assert odrive.arb_id(5, 'set_input_pos') == expected

    def test_hand_axis_set_state(self):
        expected = (6 << 5) | proto.ODRIVE_COMMANDS['set_requested_state']
        assert odrive.arb_id(6, 'set_requested_state') == expected

    def test_bb_pitch_heartbeat(self):
        expected = (7 << 5) | proto.ODRIVE_COMMANDS['heartbeat_message']
        assert odrive.arb_id(7, 'heartbeat_message') == expected


# ════════════════════════════════════════════════════════════════
# Encode functions
# ════════════════════════════════════════════════════════════════


class TestEncodeSetState:
    def test_idle(self):
        msg = odrive.encode_set_state(0, 'IDLE')
        assert msg.arbitration_id == odrive.arb_id(0, 'set_requested_state')
        assert msg.dlc == 8
        state_val = struct.unpack_from('<I', bytes(msg.data))[0]
        assert state_val == proto.ODRIVE_STATES['IDLE']

    def test_closed_loop(self):
        msg = odrive.encode_set_state(3, 'CLOSED_LOOP')
        state_val = struct.unpack_from('<I', bytes(msg.data))[0]
        assert state_val == proto.ODRIVE_STATES['CLOSED_LOOP']

    def test_encoder_index_search(self):
        msg = odrive.encode_set_state(0, 'ENCODER_INDEX_SEARCH')
        state_val = struct.unpack_from('<I', bytes(msg.data))[0]
        assert state_val == proto.ODRIVE_STATES['ENCODER_INDEX_SEARCH']

    def test_not_extended_id(self):
        msg = odrive.encode_set_state(0, 'IDLE')
        assert msg.is_extended_id is False


class TestEncodeSetControllerMode:
    def test_position_trap_traj(self):
        msg = odrive.encode_set_controller_mode(0, 'POSITION', 'TRAP_TRAJ')
        ctrl, inp = struct.unpack_from('<II', bytes(msg.data))
        assert ctrl == proto.ODRIVE_CONTROL_MODES['POSITION']
        assert inp == proto.ODRIVE_INPUT_MODES['TRAP_TRAJ']

    def test_velocity_vel_ramp(self):
        msg = odrive.encode_set_controller_mode(2, 'VELOCITY', 'VEL_RAMP')
        ctrl, inp = struct.unpack_from('<II', bytes(msg.data))
        assert ctrl == proto.ODRIVE_CONTROL_MODES['VELOCITY']
        assert inp == proto.ODRIVE_INPUT_MODES['VEL_RAMP']


class TestEncodeSetInputPos:
    def test_basic(self):
        msg = odrive.encode_set_input_pos(0, 2.5)
        pos, vel_ff, tor_ff = struct.unpack_from('<fhh', bytes(msg.data))
        assert pos == pytest.approx(2.5)
        assert vel_ff == 0
        assert tor_ff == 0

    def test_with_feedforward(self):
        msg = odrive.encode_set_input_pos(0, 1.0, vel_ff=100, torque_ff=50)
        pos, vel_ff, tor_ff = struct.unpack_from('<fhh', bytes(msg.data))
        assert pos == pytest.approx(1.0)
        assert vel_ff == 100
        assert tor_ff == 50

    def test_zero_position(self):
        msg = odrive.encode_set_input_pos(0, 0.0)
        pos = struct.unpack_from('<f', bytes(msg.data))[0]
        assert pos == 0.0

    def test_arb_id(self):
        msg = odrive.encode_set_input_pos(3, 1.0)
        assert msg.arbitration_id == odrive.arb_id(3, 'set_input_pos')


class TestEncodeSetInputVel:
    def test_basic(self):
        msg = odrive.encode_set_input_vel(0, 5.0)
        vel, tor = struct.unpack_from('<ff', bytes(msg.data))
        assert vel == pytest.approx(5.0)
        assert tor == pytest.approx(0.0)

    def test_with_torque_ff(self):
        msg = odrive.encode_set_input_vel(0, 3.0, torque_ff=1.5)
        vel, tor = struct.unpack_from('<ff', bytes(msg.data))
        assert vel == pytest.approx(3.0)
        assert tor == pytest.approx(1.5)


class TestEncodeSetVelCurrLimits:
    def test_basic(self):
        msg = odrive.encode_set_vel_curr_limits(0, 50.0, 20.0)
        v, c = struct.unpack_from('<ff', bytes(msg.data))
        assert v == pytest.approx(50.0)
        assert c == pytest.approx(20.0)
        assert msg.arbitration_id == odrive.arb_id(0, 'set_vel_curr_limits')


class TestEncodeSetTrajVelLimit:
    def test_basic(self):
        msg = odrive.encode_set_traj_vel_limit(0, 15.0)
        val = struct.unpack_from('<f', bytes(msg.data))[0]
        assert val == pytest.approx(15.0)

    def test_padding_bytes_zero(self):
        msg = odrive.encode_set_traj_vel_limit(0, 15.0)
        assert bytes(msg.data)[4:8] == bytes(4)


class TestEncodeSetTrajAccLimits:
    def test_basic(self):
        msg = odrive.encode_set_traj_acc_limits(0, 30.0, 25.0)
        acc, dec = struct.unpack_from('<ff', bytes(msg.data))
        assert acc == pytest.approx(30.0)
        assert dec == pytest.approx(25.0)


class TestEncodeSetAbsolutePosition:
    def test_basic(self):
        msg = odrive.encode_set_absolute_position(0, 0.1)
        val = struct.unpack_from('<f', bytes(msg.data))[0]
        assert val == pytest.approx(0.1)
        assert msg.arbitration_id == odrive.arb_id(0, 'set_absolute_position')


class TestEncodeSetPosGain:
    def test_basic(self):
        msg = odrive.encode_set_pos_gain(6, 35.0)
        val = struct.unpack_from('<f', bytes(msg.data))[0]
        assert val == pytest.approx(35.0)


class TestEncodeSetVelGains:
    def test_basic(self):
        msg = odrive.encode_set_vel_gains(6, 0.007, 0.07)
        vg, vig = struct.unpack_from('<ff', bytes(msg.data))
        assert vg == pytest.approx(0.007, abs=1e-6)
        assert vig == pytest.approx(0.07, abs=1e-6)


class TestEncodeClearErrors:
    def test_correct_arb_id(self):
        msg = odrive.encode_clear_errors(0)
        assert msg.arbitration_id == odrive.arb_id(0, 'clear_errors')

    def test_zero_data(self):
        msg = odrive.encode_clear_errors(0)
        assert bytes(msg.data) == bytes(8)

    def test_dlc_8(self):
        msg = odrive.encode_clear_errors(0)
        assert msg.dlc == 8


class TestEncodeReboot:
    def test_correct_arb_id(self):
        msg = odrive.encode_reboot(0)
        assert msg.arbitration_id == odrive.arb_id(0, 'reboot_odrives')

    def test_zero_data(self):
        msg = odrive.encode_reboot(0)
        assert bytes(msg.data) == bytes(8)


class TestEncodeSdoRead:
    def test_commutation_mapper(self):
        msg = odrive.encode_sdo_read(0, 'commutation_mapper.pos_abs')
        opcode, ep_id, _, val = struct.unpack_from('<BHBf', bytes(msg.data))
        assert opcode == proto.OPCODE_READ
        assert ep_id == proto.ENDPOINT_COMMUTATION_MAPPER_POS_ABS
        assert val == pytest.approx(0.0)

    def test_arb_id(self):
        msg = odrive.encode_sdo_read(0, 'commutation_mapper.pos_abs')
        assert msg.arbitration_id == odrive.arb_id(0, 'RxSdo')


class TestEncodeSdoWrite:
    def test_commutation_mapper(self):
        msg = odrive.encode_sdo_write(0, 'commutation_mapper.pos_abs', 1.23)
        opcode, ep_id, _, val = struct.unpack_from('<BHBf', bytes(msg.data))
        assert opcode == proto.OPCODE_WRITE
        assert ep_id == proto.ENDPOINT_COMMUTATION_MAPPER_POS_ABS
        assert val == pytest.approx(1.23, abs=1e-5)


# ════════════════════════════════════════════════════════════════
# Decode functions
# ════════════════════════════════════════════════════════════════


class TestDecodeHeartbeat:
    def test_valid_data(self):
        data = struct.pack('<IBBBB', 0, 8, 0, 0x01, 0)
        state, proc, traj = odrive.decode_heartbeat(data)
        assert state == 8
        assert proc == 0
        assert traj is True

    def test_trajectory_not_done(self):
        data = struct.pack('<IBBBB', 0, 1, 0, 0x00, 0)
        _, _, traj = odrive.decode_heartbeat(data)
        assert traj is False

    def test_procedure_result(self):
        data = struct.pack('<IBBBB', 0, 8, 2, 0x01, 0)
        _, proc, _ = odrive.decode_heartbeat(data)
        assert proc == 2

    def test_short_data_raises(self):
        with pytest.raises(ValueError, match="expected 7 bytes"):
            odrive.decode_heartbeat(bytes(6))


class TestDecodeError:
    def test_valid_data(self):
        data = struct.pack('<II', 512, 1024)
        active, disarm = odrive.decode_error(data)
        assert active == 512
        assert disarm == 1024

    def test_no_errors(self):
        data = struct.pack('<II', 0, 0)
        active, disarm = odrive.decode_error(data)
        assert active == 0
        assert disarm == 0

    def test_short_data_raises(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            odrive.decode_error(bytes(7))


class TestDecodeEncoderEstimate:
    def test_valid_data(self):
        data = struct.pack('<ff', 1.5, 0.25)
        pos, vel = odrive.decode_encoder_estimate(data)
        assert pos == pytest.approx(1.5)
        assert vel == pytest.approx(0.25)

    def test_negative_values(self):
        data = struct.pack('<ff', -2.0, -0.5)
        pos, vel = odrive.decode_encoder_estimate(data)
        assert pos == pytest.approx(-2.0)
        assert vel == pytest.approx(-0.5)

    def test_short_data_raises(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            odrive.decode_encoder_estimate(bytes(7))


class TestDecodeIq:
    def test_valid_data(self):
        data = struct.pack('<ff', 5.0, 4.8)
        sp, meas = odrive.decode_iq(data)
        assert sp == pytest.approx(5.0)
        assert meas == pytest.approx(4.8)

    def test_short_data_raises(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            odrive.decode_iq(bytes(7))


class TestDecodeTemps:
    def test_valid_data(self):
        data = struct.pack('<ff', 45.2, 38.1)
        fet, motor = odrive.decode_temps(data)
        assert fet == pytest.approx(45.2, abs=0.1)
        assert motor == pytest.approx(38.1, abs=0.1)

    def test_short_data_raises(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            odrive.decode_temps(bytes(7))


class TestDecodeBusVoltageCurrent:
    def test_valid_data(self):
        data = struct.pack('<ff', 48.0, 2.5)
        v, c = odrive.decode_bus_voltage_current(data)
        assert v == pytest.approx(48.0)
        assert c == pytest.approx(2.5)

    def test_short_data_raises(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            odrive.decode_bus_voltage_current(bytes(7))


class TestDecodeSdoResponse:
    def test_valid_data(self):
        ep_id = proto.ENDPOINT_COMMUTATION_MAPPER_POS_ABS
        data = struct.pack('<BHBf', 0, ep_id, 0, 1.23)
        decoded_ep, val = odrive.decode_sdo_response(data)
        assert decoded_ep == ep_id
        assert val == pytest.approx(1.23, abs=1e-5)

    def test_nan_value(self):
        ep_id = proto.ENDPOINT_COMMUTATION_MAPPER_POS_ABS
        data = struct.pack('<BHBf', 0, ep_id, 0, float('nan'))
        _, val = odrive.decode_sdo_response(data)
        assert math.isnan(val)

    def test_short_data_raises(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            odrive.decode_sdo_response(bytes(7))


# ════════════════════════════════════════════════════════════════
# clip_position()
# ════════════════════════════════════════════════════════════════


class TestClipPosition:
    def test_leg_within_range(self):
        result = odrive.clip_position(0, 2.0)
        assert result == pytest.approx(2.0)

    def test_leg_below_zero_clips(self):
        result = odrive.clip_position(0, -1.0)
        assert result == 0.0

    def test_leg_above_max_clips(self):
        result = odrive.clip_position(0, 100.0)
        assert result == hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS

    def test_leg_at_zero(self):
        result = odrive.clip_position(0, 0.0)
        assert result == 0.0

    def test_leg_at_max(self):
        result = odrive.clip_position(0, hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS)
        assert result == hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS

    def test_hand_within_range(self):
        result = odrive.clip_position(6, 5.0)
        assert result == pytest.approx(5.0)

    def test_hand_above_max_clips(self):
        result = odrive.clip_position(6, 100.0)
        assert result == hw.GEOM_HAND_MOTOR_MAX_POSITION_REVS

    def test_invalid_axis_raises(self):
        with pytest.raises(ValueError, match="Invalid Jugglebot axis"):
            odrive.clip_position(99, 1.0)

    def test_bb_axis_raises(self):
        with pytest.raises(ValueError, match="Invalid Jugglebot axis"):
            odrive.clip_position(7, 1.0)

    def test_all_leg_axes_accepted(self):
        for axis_id in odrive.LEG_AXES:
            result = odrive.clip_position(axis_id, 1.0)
            assert result == pytest.approx(1.0)

    def test_logger_called_on_clip(self):
        from unittest.mock import MagicMock
        logger = MagicMock()
        odrive.clip_position(0, -1.0, logger=logger)
        logger.warning.assert_called_once()


# ════════════════════════════════════════════════════════════════
# error_names()
# ════════════════════════════════════════════════════════════════


class TestErrorNames:
    def test_single_error(self):
        names = odrive.error_names(512)
        assert names == ["DC_BUS_UNDER_VOLTAGE"]

    def test_multiple_errors(self):
        names = odrive.error_names(512 | 1024)
        assert "DC_BUS_UNDER_VOLTAGE" in names
        assert "DC_BUS_OVER_CURRENT" in names
        assert len(names) == 2

    def test_no_errors(self):
        assert odrive.error_names(0) == []

    def test_unknown_bits_ignored(self):
        # Bit 7 (128) is not in ERROR_CODES
        names = odrive.error_names(128)
        assert names == []

    def test_all_known_codes(self):
        # OR all known error codes together and check we get them all back
        all_mask = 0
        for code in odrive.ERROR_CODES:
            all_mask |= code
        names = odrive.error_names(all_mask)
        assert len(names) == len(odrive.ERROR_CODES)
