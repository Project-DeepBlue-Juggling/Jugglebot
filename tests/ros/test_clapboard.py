"""Unit tests for jugglebot.can.clapboard — Electronic Clapboard CAN decoding.

Mirrors tests/ros/test_catching_cone.py (the clapboard shares that device's bus
and, deliberately, its decoder shape).  No ROS2 dependency, so these run
anywhere the mocks in tests/ros/conftest.py are installed.

The frame-packing helpers here are hand-written from the NORMATIVE source —
``Electronic-Clapboard docs/protocol.md`` §8.6-8.8 — rather than from the
decoder, so a decoder that reads the wrong byte fails instead of agreeing with
itself.  They are byte-for-byte the peer's ``can_frames.h`` encoders.
"""

import struct

import pytest

import jugglebot.protocol_config as proto
from jugglebot.can import catching_cone as cc
from jugglebot.can import clapboard as cb


# ── frame-packing helpers (mirror the peer's can_frames.h encoders) ──────────

def pack_heartbeat(state: int = 1, flags: int = 0, fires_since_boot: int = 0,
                   rail_mv: int = 0, active_template_id: int = 0,
                   last_error: int = 0) -> bytes:
    """Build an 8-byte CLAP_HEARTBEAT frame (§8.7)."""
    return (bytes([state, flags])
            + int(fires_since_boot).to_bytes(2, 'little')
            + int(rail_mv).to_bytes(2, 'little')
            + bytes([active_template_id, last_error]))


def pack_fire_event(wall_us: int, fires_since_boot: int) -> bytes:
    """Build an 8-byte CLAP_FIRE_EVENT frame (§8.8).

    Masks to 48 bits exactly as the peer's ``encode_fire_event`` does — that
    mask is the whole reason the host has to reconstruct the high bits.
    """
    return ((wall_us & 0x0000FFFFFFFFFFFF).to_bytes(6, 'little')
            + int(fires_since_boot).to_bytes(2, 'little'))


def pack_ack(txn_id: int = 0, outcome: int = 0, state: int = 1,
             render_ms: int = 0) -> bytes:
    """Build an 8-byte CLAP_ACK frame (§8.6)."""
    return (bytes([txn_id, outcome, state])
            + int(render_ms).to_bytes(2, 'little')
            + b'\x00\x00\x00')


# ── CAN IDs ──────────────────────────────────────────────────────────────────

class TestCanIds:
    """The id block is a cross-repo contract (protocol.md §8.2) — pin it hard."""

    def test_uplink_ids(self):
        assert cb.ACK_ID == 0x7EB
        assert cb.HEARTBEAT_ID == 0x7EC
        assert cb.FIRE_EVENT_ID == 0x7ED

    def test_downlink_ids(self):
        assert cb.FIELD_ID == 0x7E8
        assert cb.COMMIT_ID == 0x7E9
        assert cb.LINK_ID == 0x7EA

    def test_ids_come_from_the_generated_constants(self):
        # Not literals in the module: protocol_config.yaml is the single source,
        # shared with the firmware's is_clapboard_id().
        assert cb.HEARTBEAT_ID == proto.CAN_ID_CLAP_HEARTBEAT
        assert cb.FIRE_EVENT_ID == proto.CAN_ID_CLAP_FIRE_EVENT
        assert cb.ACK_ID == proto.CAN_ID_CLAP_ACK
        assert cb.FIELD_ID == proto.CAN_ID_CLAP_FIELD
        assert cb.COMMIT_ID == proto.CAN_ID_CLAP_COMMIT
        assert cb.LINK_ID == proto.CAN_ID_CLAP_LINK

    def test_block_bounds_span_the_whole_allocation(self):
        # 0x7EE-0x7EF are reserved but still clapboard territory: a frame there
        # can only have come from a clapboard.
        assert cb.BLOCK_FIRST_ID == 0x7E8
        assert cb.BLOCK_LAST_ID == 0x7EF
        for ident in (cb.FIELD_ID, cb.COMMIT_ID, cb.LINK_ID, cb.ACK_ID,
                      cb.HEARTBEAT_ID, cb.FIRE_EVENT_ID):
            assert cb.BLOCK_FIRST_ID <= ident <= cb.BLOCK_LAST_ID

    def test_downlink_only_set_is_exactly_the_three_j_to_c_ids(self):
        assert cb.DOWNLINK_ONLY_IDS == frozenset(
            (cb.FIELD_ID, cb.COMMIT_ID, cb.LINK_ID))

    def test_ids_do_not_collide_with_the_catching_cone(self):
        # The two devices share one physical bus. A collision would decode a
        # clapboard heartbeat as a catch event and inject phantom catches into
        # catch_correlation_node — the exact failure distinct ids prevent.
        cone_ids = {cc.CATCH_EVENT_ID, cc.HEARTBEAT_ID}
        clap_ids = {cb.FIELD_ID, cb.COMMIT_ID, cb.LINK_ID, cb.ACK_ID,
                    cb.HEARTBEAT_ID, cb.FIRE_EVENT_ID}
        assert cone_ids.isdisjoint(clap_ids)
        for ident in cone_ids:
            assert not (cb.BLOCK_FIRST_ID <= ident <= cb.BLOCK_LAST_ID)

    def test_ids_do_not_collide_with_the_time_sync_broadcast(self):
        # 0x7DD rides the same bus at 100 Hz and is what anchors the clapboard.
        assert proto.CAN_ID_SHARED_TIME_SYNC not in {
            cb.FIELD_ID, cb.COMMIT_ID, cb.LINK_ID, cb.ACK_ID,
            cb.HEARTBEAT_ID, cb.FIRE_EVENT_ID}


# ── ClapboardHeartbeat ───────────────────────────────────────────────────────

class TestClapboardHeartbeat:
    def test_round_trip(self):
        hb = cb.ClapboardHeartbeat.from_can_frame(pack_heartbeat(
            state=2, flags=0x1F, fires_since_boot=1234, rail_mv=11800,
            active_template_id=3, last_error=0x02))
        assert hb.state == int(cb.ClapboardStates.RENDERING)
        assert hb.fires_since_boot == 1234
        assert hb.rail_mv == 11800
        assert hb.active_template_id == 3
        assert hb.last_error == 0x02

    @pytest.mark.parametrize("raw,expected", [
        (0, cb.ClapboardStates.BOOT),
        (1, cb.ClapboardStates.IDLE),
        (2, cb.ClapboardStates.RENDERING),
        (3, cb.ClapboardStates.SCREENSAVER),
        (4, cb.ClapboardStates.ERROR),
    ])
    def test_state_values_match_the_contract(self, raw, expected):
        hb = cb.ClapboardHeartbeat.from_can_frame(pack_heartbeat(state=raw))
        assert hb.state == int(expected)
        assert hb.state_name == expected.name

    def test_error_state_is_4_not_the_cone_s_127(self):
        # The two devices share a bus and a decoder shape but NOT this enum.
        assert int(cb.ClapboardStates.ERROR) == 4
        assert int(cc.CatchingConeStates.ERROR) == 127

    def test_unknown_state_passes_through_rather_than_raising(self):
        # Deliberate deviation from ConeHeartbeat: coercing to the enum would
        # raise, the RX callback would drop the frame, and 500 ms later the node
        # would report the clapboard DISCONNECTED — a confidently wrong verdict
        # where "state 9" is merely uninformative.
        hb = cb.ClapboardHeartbeat.from_can_frame(pack_heartbeat(state=9))
        assert hb.state == 9
        assert hb.state_name == "UNKNOWN(9)"

    @pytest.mark.parametrize("flag,attr", [
        (0x01, 'fire_ready'),
        (0x02, 'template_loaded'),
        (0x04, 'wifi_up'),
        (0x08, 'rail_ok'),
        (0x10, 'time_synced'),
    ])
    def test_each_flag_bit_maps_to_exactly_one_field(self, flag, attr):
        hb = cb.ClapboardHeartbeat.from_can_frame(pack_heartbeat(flags=flag))
        for name in ('fire_ready', 'template_loaded', 'wifi_up', 'rail_ok',
                     'time_synced'):
            assert getattr(hb, name) is (name == attr), (
                f"flag 0x{flag:02X} should set only {attr}")

    def test_all_flags_clear(self):
        hb = cb.ClapboardHeartbeat.from_can_frame(pack_heartbeat(flags=0x00))
        assert not any((hb.fire_ready, hb.template_loaded, hb.wifi_up,
                        hb.rail_ok, hb.time_synced))

    def test_reserved_flag_bits_ignored(self):
        # bits 5-7 are unallocated — they must not disturb the five defined ones
        hb = cb.ClapboardHeartbeat.from_can_frame(pack_heartbeat(flags=0xE0))
        assert not any((hb.fire_ready, hb.template_loaded, hb.wifi_up,
                        hb.rail_ok, hb.time_synced))

    def test_u16_fields_at_maximum(self):
        hb = cb.ClapboardHeartbeat.from_can_frame(pack_heartbeat(
            fires_since_boot=65535, rail_mv=65535))
        assert hb.fires_since_boot == 65535
        assert hb.rail_mv == 65535

    def test_u16_fields_are_little_endian(self):
        # 0x0102 must decode as 258, not 513 — a byte-order slip here would
        # misreport the flash counter by orders of magnitude.
        hb = cb.ClapboardHeartbeat.from_can_frame(
            bytes([1, 0]) + b'\x02\x01' + b'\x04\x03' + bytes([0, 0]))
        assert hb.fires_since_boot == 0x0102
        assert hb.rail_mv == 0x0304

    def test_defaults_are_a_safe_disconnected_placeholder(self):
        hb = cb.ClapboardHeartbeat()
        assert hb.state == int(cb.ClapboardStates.BOOT)
        assert hb.time_synced is False
        assert hb.fires_since_boot == 0

    def test_rejects_short_frame(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            cb.ClapboardHeartbeat.from_can_frame(b'\x00' * 7)

    def test_rejects_long_frame(self):
        # Stricter than the cone's `< 8`: the peer requires len == 8 exactly.
        with pytest.raises(ValueError, match="expected 8 bytes"):
            cb.ClapboardHeartbeat.from_can_frame(b'\x00' * 9)

    def test_rejects_empty_frame(self):
        with pytest.raises(ValueError):
            cb.ClapboardHeartbeat.from_can_frame(b'')


# ── ClapboardFireEvent ───────────────────────────────────────────────────────

class TestClapboardFireEvent:
    def test_round_trip(self):
        fe = cb.ClapboardFireEvent.from_can_frame(
            pack_fire_event(0x0000_1234_5678_9ABC, 77))
        assert fe.wall_us_low48 == 0x1234_5678_9ABC
        assert fe.fires_since_boot == 77

    def test_overflow_cannot_bleed_into_the_sequence_number(self):
        # The device masks to 48 bits; the decoder must read exactly six bytes.
        # A u64 unpack over this frame would swallow fires_since_boot whole.
        fe = cb.ClapboardFireEvent.from_can_frame(
            pack_fire_event(0xFFFF_FFFF_FFFF_FFFF, 5))
        assert fe.wall_us_low48 == 0x0000_FFFF_FFFF_FFFF
        assert fe.fires_since_boot == 5

    def test_max_48_bit_timestamp(self):
        fe = cb.ClapboardFireEvent.from_can_frame(
            pack_fire_event(0xFFFF_FFFF_FFFF, 0))
        assert fe.wall_us_low48 == (1 << 48) - 1

    def test_fields_are_little_endian(self):
        fe = cb.ClapboardFireEvent.from_can_frame(
            b'\x01\x02\x03\x04\x05\x06' + b'\x08\x07')
        assert fe.wall_us_low48 == 0x060504030201
        assert fe.fires_since_boot == 0x0708

    def test_zero_frame(self):
        fe = cb.ClapboardFireEvent.from_can_frame(b'\x00' * 8)
        assert fe.wall_us_low48 == 0
        assert fe.fires_since_boot == 0

    def test_rejects_short_frame(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            cb.ClapboardFireEvent.from_can_frame(b'\x00' * 7)

    def test_rejects_long_frame(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            cb.ClapboardFireEvent.from_can_frame(b'\x00' * 9)


# ── ClapboardAck ─────────────────────────────────────────────────────────────

class TestClapboardAck:
    def test_round_trip(self):
        ack = cb.ClapboardAck.from_can_frame(
            pack_ack(txn_id=17, outcome=0x00, state=2, render_ms=2750))
        assert ack.txn_id == 17
        assert ack.outcome == int(cb.ClapboardAckOutcome.OK)
        assert ack.state == int(cb.ClapboardStates.RENDERING)
        assert ack.render_ms == 2750

    @pytest.mark.parametrize("raw,expected", [
        (0x00, cb.ClapboardAckOutcome.OK),
        (0x01, cb.ClapboardAckOutcome.REJECTED),
        (0x02, cb.ClapboardAckOutcome.CRC_MISMATCH),
        (0x03, cb.ClapboardAckOutcome.INCOMPLETE),
        (0x04, cb.ClapboardAckOutcome.BUSY),
        (0x05, cb.ClapboardAckOutcome.NO_TEMPLATE),
        (0x06, cb.ClapboardAckOutcome.BAD_FIELD_ID),
    ])
    def test_outcome_values_match_the_contract(self, raw, expected):
        ack = cb.ClapboardAck.from_can_frame(pack_ack(outcome=raw))
        assert ack.outcome == int(expected)
        assert ack.outcome_name == expected.name

    def test_unknown_outcome_passes_through(self):
        ack = cb.ClapboardAck.from_can_frame(pack_ack(outcome=0x7F))
        assert ack.outcome == 0x7F
        assert ack.outcome_name == "UNKNOWN(127)"

    def test_render_ms_is_little_endian_at_bytes_3_4(self):
        # protocol.md §8.6 puts render_ms at bytes 3-4, NOT 4-5 — an off-by-one
        # here would report a plausible-looking but wrong panel time.
        ack = cb.ClapboardAck.from_can_frame(
            bytes([1, 0, 1]) + b'\xE8\x03' + b'\x00\x00\x00')
        assert ack.render_ms == 1000

    def test_reserved_bytes_ignored(self):
        ack = cb.ClapboardAck.from_can_frame(
            bytes([9, 0, 1]) + b'\x00\x00' + b'\xFF\xFF\xFF')
        assert ack.txn_id == 9
        assert ack.render_ms == 0

    def test_max_render_ms(self):
        ack = cb.ClapboardAck.from_can_frame(pack_ack(render_ms=65535))
        assert ack.render_ms == 65535

    def test_rejects_short_frame(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            cb.ClapboardAck.from_can_frame(b'\x00' * 7)

    def test_rejects_long_frame(self):
        with pytest.raises(ValueError, match="expected 8 bytes"):
            cb.ClapboardAck.from_can_frame(b'\x00' * 9)


# ── reconstruct_fire_time_us ─────────────────────────────────────────────────

_BOUNDARY = 6 << 48   # a clean multiple of 2^48 to build wrap cases around


class TestReconstructFireTimeUs:
    def test_a_2026_unix_us_value_does_not_fit_in_48_bits(self):
        """The premise the whole reconstruction rests on.

        protocol.md §8.8 reads as though the wall clock fits in the field; it
        does not, and the peer masks. Without reconstruction every flash would be
        stamped somewhere in 1978.
        """
        unix_us_2026 = 1_786_000_000 * 1_000_000
        assert unix_us_2026 > (1 << 48)
        assert unix_us_2026.bit_length() == 51

    def test_no_wrap_exact(self):
        host = 1_786_000_000_000_000
        low48 = host & ((1 << 48) - 1)
        assert cb.reconstruct_fire_time_us(low48, host) == host

    def test_recent_past(self):
        host = _BOUNDARY + 10_000_000
        fired = host - 1_000_000
        low48 = fired & ((1 << 48) - 1)
        assert cb.reconstruct_fire_time_us(low48, host) == fired

    def test_wrap_forward(self):
        # host just past a 2^48 boundary; the flash happened just before it
        host = _BOUNDARY + 1_000_000
        fired = _BOUNDARY - 1_000_000
        low48 = fired & ((1 << 48) - 1)
        assert cb.reconstruct_fire_time_us(low48, host) == fired

    def test_wrap_backward(self):
        # host just before a boundary; low48 small (as if from after it)
        host = _BOUNDARY - 1_000_000
        fired = _BOUNDARY + 500_000
        low48 = fired & ((1 << 48) - 1)
        assert cb.reconstruct_fire_time_us(low48, host) == fired

    def test_high_bits_preserved(self):
        host = (6 << 48) + 777_000_000
        fired = host - 250_000
        low48 = fired & ((1 << 48) - 1)
        result = cb.reconstruct_fire_time_us(low48, host)
        assert result == fired
        assert result >> 48 == 6

    def test_end_to_end_a_present_day_flash_stamps_in_the_present(self):
        """Frame → decode → reconstruct lands within a second of the host clock.

        This is the assertion that would have caught a naive implementation:
        taking the wire field at face value puts the stamp ~1.5e15 µs (47 years)
        away, so the tolerance below is not remotely tight.
        """
        host = 1_786_000_123_456_789
        frame = pack_fire_event(host, 42)
        fe = cb.ClapboardFireEvent.from_can_frame(frame)
        assert fe.wall_us_low48 != host          # the mask really did bite
        assert cb.reconstruct_fire_time_us(fe.wall_us_low48, host) == host

    def test_struct_helpers_agree_with_the_decoder_widths(self):
        # Guard against a future edit changing one packer and not the other.
        assert len(pack_heartbeat()) == 8
        assert len(pack_fire_event(0, 0)) == 8
        assert len(pack_ack()) == 8
        assert struct.calcsize('<BBHHBB') == 8
