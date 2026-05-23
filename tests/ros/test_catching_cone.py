"""Unit tests for jugglebot.can.catching_cone — Catching Cone CAN decoding.

Mirrors tests/ros/test_ball_butler.py. The decode helper has no ROS2
dependency, so these run on Windows via the mocks in tests/ros/conftest.py.
"""

import struct

import pytest

import jugglebot.protocol_config as proto
from jugglebot.can import catching_cone as cc


# ── frame-packing helpers (mirror the firmware encoder) ──────────────────────

def pack_catch_event(low32: int, sequence: int, flags: int, reserved: int = 0) -> bytes:
    """Build an 8-byte CATCH_EVENT frame ('<IBBH')."""
    return struct.pack('<IBBH', low32, sequence, flags, reserved)


def pack_heartbeat(state: int, state_data: int, last_catch_seq: int,
                   ms_since_last_catch: int, flags: int, reserved: int = 0) -> bytes:
    """Build an 8-byte CONE_HEARTBEAT frame."""
    return (bytes([state, state_data, last_catch_seq])
            + int(ms_since_last_catch).to_bytes(3, 'little')
            + bytes([flags, reserved]))


# ── CAN IDs ──────────────────────────────────────────────────────────────────

class TestCanIds:
    def test_catch_event_id(self):
        assert cc.CATCH_EVENT_ID == 0x7E0
        assert cc.CATCH_EVENT_ID == proto.CAN_ID_CC_CATCH_EVENT

    def test_heartbeat_id(self):
        assert cc.HEARTBEAT_ID == 0x7E1
        assert cc.HEARTBEAT_ID == proto.CAN_ID_CC_HEARTBEAT

    def test_ids_do_not_collide_with_ball_butler(self):
        bb_ids = {proto.CAN_ID_BB_THROW_CMD, proto.CAN_ID_BB_HEARTBEAT,
                  proto.CAN_ID_BB_RELOAD_CMD, proto.CAN_ID_BB_RESET_CMD,
                  proto.CAN_ID_BB_CALIBRATE_LOC_CMD}
        assert cc.CATCH_EVENT_ID not in bb_ids
        assert cc.HEARTBEAT_ID not in bb_ids


# ── CatchEvent ───────────────────────────────────────────────────────────────

class TestCatchEvent:
    def test_round_trip(self):
        frame = pack_catch_event(low32=123_456_789, sequence=42, flags=0x01)
        evt = cc.CatchEvent.from_can_frame(frame)
        assert evt.catch_time_us_low32 == 123_456_789
        assert evt.sequence == 42

    def test_flag_time_synced_only(self):
        evt = cc.CatchEvent.from_can_frame(pack_catch_event(0, 0, flags=0x01))
        assert evt.time_synced is True
        assert evt.retrigger_suppressed is False

    def test_flag_retrigger_only(self):
        evt = cc.CatchEvent.from_can_frame(pack_catch_event(0, 0, flags=0x02))
        assert evt.time_synced is False
        assert evt.retrigger_suppressed is True

    def test_flags_both(self):
        evt = cc.CatchEvent.from_can_frame(pack_catch_event(0, 0, flags=0x03))
        assert evt.time_synced is True
        assert evt.retrigger_suppressed is True

    def test_flags_none(self):
        evt = cc.CatchEvent.from_can_frame(pack_catch_event(0, 0, flags=0x00))
        assert evt.time_synced is False
        assert evt.retrigger_suppressed is False

    def test_reserved_bits_in_flags_ignored(self):
        # bits 2-7 are reserved — must not affect the decoded booleans
        evt = cc.CatchEvent.from_can_frame(pack_catch_event(0, 0, flags=0xFC))
        assert evt.time_synced is False
        assert evt.retrigger_suppressed is False

    def test_sequence_max(self):
        evt = cc.CatchEvent.from_can_frame(pack_catch_event(0, sequence=255, flags=0))
        assert evt.sequence == 255

    def test_low32_max(self):
        evt = cc.CatchEvent.from_can_frame(pack_catch_event(0xFFFFFFFF, 0, 0x01))
        assert evt.catch_time_us_low32 == 0xFFFFFFFF

    def test_rejects_short_frame(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            cc.CatchEvent.from_can_frame(b'\x00' * 7)

    def test_rejects_empty_frame(self):
        with pytest.raises(ValueError):
            cc.CatchEvent.from_can_frame(b'')


# ── ConeHeartbeat ────────────────────────────────────────────────────────────

class TestConeHeartbeat:
    def test_round_trip(self):
        frame = pack_heartbeat(state=2, state_data=12, last_catch_seq=7,
                               ms_since_last_catch=123_456, flags=0x01)
        hb = cc.ConeHeartbeat.from_can_frame(frame)
        assert hb.state == cc.CatchingConeStates.READY
        assert hb.state_data == 12
        assert hb.last_catch_seq == 7
        assert hb.ms_since_last_catch == 123_456
        assert hb.time_synced is True

    @pytest.mark.parametrize("raw,expected", [
        (0, cc.CatchingConeStates.BOOT),
        (1, cc.CatchingConeStates.UNSYNCED),
        (2, cc.CatchingConeStates.READY),
        (127, cc.CatchingConeStates.ERROR),
    ])
    def test_state_enum_mapping(self, raw, expected):
        hb = cc.ConeHeartbeat.from_can_frame(pack_heartbeat(raw, 0, 0, 0, 0))
        assert hb.state == expected

    def test_uint24_max(self):
        hb = cc.ConeHeartbeat.from_can_frame(
            pack_heartbeat(2, 0, 0, ms_since_last_catch=0xFFFFFF, flags=0))
        assert hb.ms_since_last_catch == 16_777_215

    def test_time_synced_flag_clear(self):
        hb = cc.ConeHeartbeat.from_can_frame(pack_heartbeat(1, 0, 0, 0, flags=0x00))
        assert hb.time_synced is False

    def test_have_any_catch_flag_set(self):
        hb = cc.ConeHeartbeat.from_can_frame(pack_heartbeat(2, 0, 5, 100, flags=0x02))
        assert hb.have_any_catch is True

    def test_have_any_catch_flag_clear_at_boot(self):
        hb = cc.ConeHeartbeat.from_can_frame(pack_heartbeat(1, 0, 0, 0, flags=0x00))
        assert hb.have_any_catch is False

    def test_have_any_catch_with_time_synced(self):
        hb = cc.ConeHeartbeat.from_can_frame(pack_heartbeat(2, 0, 5, 100, flags=0x03))
        assert hb.time_synced is True
        assert hb.have_any_catch is True

    def test_sync_rms_property_when_synced(self):
        hb = cc.ConeHeartbeat.from_can_frame(
            pack_heartbeat(state=2, state_data=18, last_catch_seq=0,
                           ms_since_last_catch=0, flags=0x01))
        assert hb.sync_rms_us == 18

    def test_sync_rms_property_zero_in_error(self):
        # In ERROR, state_data is an error code, not jitter — sync_rms_us must be 0
        hb = cc.ConeHeartbeat.from_can_frame(
            pack_heartbeat(state=127, state_data=5, last_catch_seq=0,
                           ms_since_last_catch=0, flags=0x00))
        assert hb.state == cc.CatchingConeStates.ERROR
        assert hb.state_data == 5
        assert hb.sync_rms_us == 0

    def test_rejects_short_frame(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            cc.ConeHeartbeat.from_can_frame(b'\x00' * 7)


# ── reconstruct_catch_time_us ────────────────────────────────────────────────

_BOUNDARY = 100 << 32  # a clean multiple of 2^32 to build wrap cases around


class TestReconstructCatchTimeUs:
    def test_no_wrap_exact(self):
        host = 5_000_000_000
        low32 = host & 0xFFFFFFFF
        assert cc.reconstruct_catch_time_us(low32, host) == host

    def test_recent_past(self):
        host = _BOUNDARY + 10_000_000
        catch = host - 1_000_000
        low32 = catch & 0xFFFFFFFF
        assert cc.reconstruct_catch_time_us(low32, host) == catch

    def test_wrap_forward(self):
        # host clock just past a 32-bit boundary; catch happened just before it
        host = _BOUNDARY + 1_000_000
        catch = _BOUNDARY - 1_000_000
        low32 = catch & 0xFFFFFFFF          # large (~2^32 - 1e6)
        assert cc.reconstruct_catch_time_us(low32, host) == catch

    def test_wrap_backward(self):
        # host clock just before a boundary; low32 small (as if from after it)
        host = _BOUNDARY - 1_000_000
        catch = _BOUNDARY + 500_000
        low32 = catch & 0xFFFFFFFF          # small (500_000)
        assert cc.reconstruct_catch_time_us(low32, host) == catch

    def test_full_64bit_value_preserved(self):
        # reconstruction must keep the high bits, not just the low 32
        host = (1234 << 32) + 777_000
        catch = host - 250_000
        low32 = catch & 0xFFFFFFFF
        result = cc.reconstruct_catch_time_us(low32, host)
        assert result == catch
        assert result >> 32 == 1234
