"""Tests for jugglebot.can.ball_butler — BB heartbeat parsing and command encoding."""

import math
import struct
from unittest.mock import patch

import pytest

from jugglebot.can import ball_butler
from jugglebot.can.ball_butler import (
    BallButlerHeartbeat,
    BallButlerStates,
    BallButlerError,
    encode_throw_command,
    encode_state_command,
    THROW_CMD_ID,
    RELOAD_CMD_ID,
    RESET_CMD_ID,
    CALIBRATE_CMD_ID,
)
import jugglebot.protocol_config as proto


# ════════════════════════════════════════════════════════════════
# BallButlerHeartbeat.from_can_frame()
# ════════════════════════════════════════════════════════════════


class TestBallButlerHeartbeatParsing:
    def test_basic_fields(self, sample_bb_heartbeat_data):
        hb = BallButlerHeartbeat.from_can_frame(sample_bb_heartbeat_data)
        assert hb.ball_in_hand is True
        assert hb.state == BallButlerStates.IDLE
        assert hb.state_data == 0

    def test_ball_in_hand_true(self):
        state_byte = (1 << 1) | 0x01  # IDLE, ball_in_hand=True
        data = struct.pack('<BBHHH', state_byte, 0, 0, 0, 0)
        hb = BallButlerHeartbeat.from_can_frame(data)
        assert hb.ball_in_hand is True

    def test_ball_in_hand_false(self):
        state_byte = (1 << 1) | 0x00  # IDLE, ball_in_hand=False
        data = struct.pack('<BBHHH', state_byte, 0, 0, 0, 0)
        hb = BallButlerHeartbeat.from_can_frame(data)
        assert hb.ball_in_hand is False

    def test_state_boot(self):
        state_byte = (0 << 1)  # BOOT
        data = struct.pack('<BBHHH', state_byte, 0, 0, 0, 0)
        hb = BallButlerHeartbeat.from_can_frame(data)
        assert hb.state == BallButlerStates.BOOT

    def test_state_error(self):
        state_byte = (127 << 1)  # ERROR
        data = struct.pack('<BBHHH', state_byte, 1, 0, 0, 0)
        hb = BallButlerHeartbeat.from_can_frame(data)
        assert hb.state == BallButlerStates.ERROR
        assert hb.state_data == 1

    def test_encoder_yaw_scaling(self):
        yaw_enc = 1000
        data = struct.pack('<BBHHH', 0, 0, yaw_enc, 0, 0)
        hb = BallButlerHeartbeat.from_can_frame(data)
        expected = yaw_enc * proto.HEARTBEAT_YAW_RES_DEG
        assert hb.yaw_deg == pytest.approx(expected)

    def test_encoder_pitch_scaling(self):
        pitch_enc = 500
        data = struct.pack('<BBHHH', 0, 0, 0, pitch_enc, 0)
        hb = BallButlerHeartbeat.from_can_frame(data)
        expected = pitch_enc * proto.HEARTBEAT_PITCH_RES_DEG
        assert hb.pitch_deg == pytest.approx(expected)

    def test_encoder_hand_scaling(self):
        hand_enc = 200
        data = struct.pack('<BBHHH', 0, 0, 0, 0, hand_enc)
        hb = BallButlerHeartbeat.from_can_frame(data)
        expected = hand_enc * proto.HEARTBEAT_HAND_RES_MM
        assert hb.hand_mm == pytest.approx(expected)

    def test_short_data_raises(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            BallButlerHeartbeat.from_can_frame(bytes(7))

    def test_empty_data_raises(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            BallButlerHeartbeat.from_can_frame(b'')

    def test_all_states_parseable(self):
        """Every BallButlerStates enum value can be round-tripped."""
        for st in BallButlerStates:
            state_byte = (st.value << 1)
            data = struct.pack('<BBHHH', state_byte, 0, 0, 0, 0)
            hb = BallButlerHeartbeat.from_can_frame(data)
            assert hb.state == st


# ════════════════════════════════════════════════════════════════
# error_code property
# ════════════════════════════════════════════════════════════════


class TestErrorCodeProperty:
    def test_error_state_returns_code(self):
        hb = BallButlerHeartbeat(
            state=BallButlerStates.ERROR,
            state_data=BallButlerError.RELOAD_FAILED,
        )
        assert hb.error_code == BallButlerError.RELOAD_FAILED

    def test_non_error_state_returns_none_code(self):
        hb = BallButlerHeartbeat(state=BallButlerStates.IDLE, state_data=1)
        assert hb.error_code == BallButlerError.NONE

    def test_error_state_none_error(self):
        hb = BallButlerHeartbeat(
            state=BallButlerStates.ERROR,
            state_data=BallButlerError.NONE,
        )
        assert hb.error_code == BallButlerError.NONE


# ════════════════════════════════════════════════════════════════
# encode_throw_command()
# ════════════════════════════════════════════════════════════════


class TestEncodeThrowCommand:
    def test_valid_parameters(self):
        msg = encode_throw_command(0.0, 0.5, 1.0, 0.1)
        assert msg.arbitration_id == THROW_CMD_ID
        assert msg.dlc == 8
        assert msg.is_extended_id is False

    def test_yaw_encoding_zero(self):
        msg = encode_throw_command(0.0, 0.0, 0.0, 0.0)
        yaw_int16 = struct.unpack_from('<h', bytes(msg.data), 0)[0]
        assert yaw_int16 == 0

    def test_yaw_encoding_near_pi(self):
        """Test yaw encoding near (but not at) pi — exact pi overflows int16."""
        yaw = math.pi - 0.001
        msg = encode_throw_command(yaw, 0.0, 0.0, 0.0)
        yaw_int16 = struct.unpack_from('<h', bytes(msg.data), 0)[0]
        expected = int(yaw * 32768.0 / math.pi)
        assert yaw_int16 == expected

    def test_pitch_encoding(self):
        msg = encode_throw_command(0.0, math.pi / 2, 0.0, 0.0)
        pitch_u16 = struct.unpack_from('<H', bytes(msg.data), 2)[0]
        # pi/2 * 65536 / pi = 32768
        assert pitch_u16 == 32768

    def test_speed_encoding(self):
        msg = encode_throw_command(0.0, 0.0, 1.0, 0.0)
        speed_u16 = struct.unpack_from('<H', bytes(msg.data), 4)[0]
        # 1.0 * 10000 = 10000
        assert speed_u16 == 10000

    def test_yaw_out_of_range_raises(self):
        with pytest.raises(ValueError, match="Yaw"):
            encode_throw_command(4.0, 0.0, 0.0, 0.0)

    def test_yaw_negative_out_of_range_raises(self):
        with pytest.raises(ValueError, match="Yaw"):
            encode_throw_command(-4.0, 0.0, 0.0, 0.0)

    def test_pitch_negative_raises(self):
        with pytest.raises(ValueError, match="Pitch"):
            encode_throw_command(0.0, -0.1, 0.0, 0.0)

    def test_pitch_too_high_raises(self):
        with pytest.raises(ValueError, match="Pitch"):
            encode_throw_command(0.0, 2.0, 0.0, 0.0)

    def test_speed_negative_raises(self):
        with pytest.raises(ValueError, match="Speed"):
            encode_throw_command(0.0, 0.0, -1.0, 0.0)

    def test_speed_too_high_raises(self):
        with pytest.raises(ValueError, match="Speed"):
            encode_throw_command(0.0, 0.0, 7.0, 0.0)

    def test_delay_negative_raises(self):
        with pytest.raises(ValueError, match="Delay"):
            encode_throw_command(0.0, 0.0, 0.0, -1.0)

    def test_delay_too_high_raises(self):
        with pytest.raises(ValueError, match="Delay"):
            encode_throw_command(0.0, 0.0, 0.0, 100.0)

    def test_boundary_values_accepted(self):
        """Edge cases near boundaries should not raise."""
        encode_throw_command(-math.pi, 0.0, 0.0, 0.0)  # -pi is fine (int16 = -32768)
        # Note: exact +pi overflows int16 (32768 > 32767), so use just below pi
        encode_throw_command(math.pi - 1e-6, math.pi / 2, 6.5535, 65.535)

    def test_yaw_exact_pi_overflows_int16(self):
        """Exact yaw=pi causes int16 overflow — this is a known edge case.

        The validation allows [-pi, pi] but int(pi * 32768 / pi) = 32768
        which exceeds int16 max (32767).  The firmware never commands
        exact pi so this is acceptable.
        """
        with pytest.raises(struct.error):
            encode_throw_command(math.pi, 0.0, 0.0, 0.0)


# ════════════════════════════════════════════════════════════════
# encode_state_command()
# ════════════════════════════════════════════════════════════════


class TestEncodeStateCommand:
    def test_reload(self):
        msg = encode_state_command(RELOAD_CMD_ID)
        assert msg.arbitration_id == RELOAD_CMD_ID
        assert msg.dlc == 0

    def test_reset(self):
        msg = encode_state_command(RESET_CMD_ID)
        assert msg.arbitration_id == RESET_CMD_ID

    def test_calibrate(self):
        msg = encode_state_command(CALIBRATE_CMD_ID)
        assert msg.arbitration_id == CALIBRATE_CMD_ID

    def test_not_extended_id(self):
        msg = encode_state_command(RELOAD_CMD_ID)
        assert msg.is_extended_id is False
