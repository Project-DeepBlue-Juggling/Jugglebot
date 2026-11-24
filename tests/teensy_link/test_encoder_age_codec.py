"""EncoderAge (0x96) codec pins: can-bridge FW 50, FEAT-64.

22 B = u64 t_teensy_us (IDENTICAL to the paired TELEMETRY) + u16 age_us[7], additive
(no PROTOCOL_VERSION bump). age_us saturates at ENCODER_AGE_MAX; ENCODER_AGE_NONE means
no encoder frame has arrived on that axis yet this boot.
"""

from __future__ import annotations

import struct

import pytest

from teensy_link import EncoderAge, MsgType
from teensy_link import protocol as p
from teensy_link.protocol import ENCODER_AGE_MAX, ENCODER_AGE_NONE, ENCODER_AGE_SIZE


def test_msg_type_size_and_sentinels_are_frozen():
    assert int(MsgType.ENCODER_AGE) == 0x96
    assert ENCODER_AGE_SIZE == 22
    assert ENCODER_AGE_NONE == 0xFFFF
    assert ENCODER_AGE_MAX == 0xFFFE
    assert p.PROTOCOL_VERSION == 14      # additive: no bump


def test_round_trip_with_distinct_per_axis_values():
    """Distinct values on every axis (and both sentinels): a transposition fails."""
    ages = (4100, 5500, 0, 9999, ENCODER_AGE_MAX, ENCODER_AGE_NONE, 123)
    e = EncoderAge(t_teensy_us=1_764_000_000_123_456, age_us=ages)
    raw = e.pack()
    assert len(raw) == ENCODER_AGE_SIZE
    back = EncoderAge.unpack(raw)
    assert back.t_teensy_us == 1_764_000_000_123_456
    assert tuple(back.age_us) == ages


def test_wire_bytes_are_little_endian_u64_then_seven_u16():
    raw = EncoderAge(t_teensy_us=0x0102030405060708,
                     age_us=(1, 2, 3, 4, 5, 6, 0xFFFF)).pack()
    assert raw[:8] == bytes([8, 7, 6, 5, 4, 3, 2, 1])
    assert struct.unpack("<7H", raw[8:]) == (1, 2, 3, 4, 5, 6, 0xFFFF)


def test_pairs_with_telemetry_by_the_shared_timestamp():
    """The host's pairing key is t_teensy_us; the two decoders must agree on it."""
    t = 1_764_000_000_000_777
    tel = p.Telemetry(t_teensy_us=t, pos_rev=(0.0,) * 7, vel_rps=(0.0,) * 7)
    age = EncoderAge(t_teensy_us=t, age_us=(5000,) * 7)
    assert p.Telemetry.unpack(tel.pack()).t_teensy_us == EncoderAge.unpack(age.pack()).t_teensy_us


def test_short_payload_is_refused():
    good = EncoderAge(t_teensy_us=1, age_us=(1,) * 7).pack()
    with pytest.raises(struct.error):
        EncoderAge.unpack(good[:ENCODER_AGE_SIZE - 1])
    assert EncoderAge.unpack(good[:ENCODER_AGE_SIZE]) is not None
