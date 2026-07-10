"""Round-trip + edge-case tests for the wire-protocol codec."""

from __future__ import annotations

import struct

import pytest

from controller.teensy_link import protocol as p


def test_constants_match_firmware_spec():
    # These are load-bearing — if they ever drift we've broken the protocol.
    assert p.PROTOCOL_VERSION == 3   # bumped for the HeartbeatT2J deviation-telemetry
    #                                  fields (per-leg live deviation, lead_clamp_mask,
    #                                  MAX_DEVIATION latch snapshot — 2026-07-10)
    assert p.MAGIC == 0x4A42  # "JB" little-endian
    assert p.HEADER_SIZE == 8
    assert p.CRC_SIZE == 2
    assert p.PORT_STREAM == 5005
    assert p.PORT_RPC == 5006
    assert p.NUM_LEGS == 6
    assert p.NUM_AXES == 7
    assert p.HEARTBEAT_HZ == 10


def test_crc16_canonical_check_value():
    # CRC-16/CCITT-FALSE canonical test vector
    assert p.crc16_ccitt(b"123456789") == 0x29B1


def test_heartbeat_j2t_roundtrip():
    hb = p.HeartbeatJ2T(t_jetson_us=0x0123_4567_89AB_CDEF, flags=0xDEAD_BEEF)
    blob = hb.pack()
    assert len(blob) == p.HEARTBEAT_J2T_SIZE
    decoded = p.HeartbeatJ2T.unpack(blob)
    assert decoded.t_jetson_us == hb.t_jetson_us
    assert decoded.flags == hb.flags


def test_cmd_result_frame_roundtrip():
    # The loud command-outcome relay: a raw BB CAN1 frame (0x7D5) wrapped
    # verbatim. The decoded throw outcome lives inside `data` (cmd_type, outcome,
    # detail0/1 int16 LE) — the codec just preserves the 8 payload bytes.
    cr = p.CmdResultFrame(
        t_bridge_us=0x0011_2233_4455_6677,
        can_id=0x7D5, dlc=8,
        data=(0, 0x29, 0x01, 0x00, 0xFA, 0x00, 0, 0),  # THROW, ABORTED_NOT_SETTLED
    )
    blob = cr.pack()
    assert len(blob) == p.CMD_RESULT_FRAME_SIZE
    decoded = p.CmdResultFrame.unpack(blob)
    assert decoded.t_bridge_us == cr.t_bridge_us
    assert decoded.can_id == cr.can_id
    assert decoded.dlc == cr.dlc
    assert tuple(decoded.data) == cr.data


def test_telemetry_roundtrip():
    tm = p.Telemetry(
        t_teensy_us=12345678,
        pos_rev=tuple(0.1 * i for i in range(p.NUM_AXES)),
        vel_rps=tuple(-0.2 * i for i in range(p.NUM_AXES)),
    )
    blob = tm.pack()
    assert len(blob) == p.TELEMETRY_SIZE
    decoded = p.Telemetry.unpack(blob)
    assert decoded.t_teensy_us == tm.t_teensy_us
    assert decoded.pos_rev == pytest.approx(tm.pos_rev)
    assert decoded.vel_rps == pytest.approx(tm.vel_rps)


def test_setpoint_roundtrip():
    sp = p.Setpoint(
        u0=tuple(float(i) for i in range(6)),
        u1=tuple(float(i + 10) for i in range(6)),
        u2=tuple(float(i + 20) for i in range(6)),
        v0=tuple(float(i + 30) for i in range(6)),
        accel=tuple(float(i + 40) for i in range(6)),
        torque_ff=tuple(float(i + 50) for i in range(6)),
        flags=0b101,
        t_origin_us=987654321,
    )
    blob = sp.pack()
    assert len(blob) == p.SETPOINT_SIZE
    decoded = p.Setpoint.unpack(blob)
    assert decoded.flags == sp.flags
    assert decoded.t_origin_us == sp.t_origin_us
    assert decoded.u0 == pytest.approx(sp.u0)
    assert decoded.u1 == pytest.approx(sp.u1)
    assert decoded.u2 == pytest.approx(sp.u2)
    assert decoded.v0 == pytest.approx(sp.v0)
    assert decoded.accel == pytest.approx(sp.accel)
    assert decoded.torque_ff == pytest.approx(sp.torque_ff)


def test_encode_then_decode_full_frame():
    hb = p.HeartbeatJ2T(t_jetson_us=42, flags=0)
    frame = p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 7, hb.pack())
    assert len(frame) == p.HEADER_SIZE + p.HEARTBEAT_J2T_SIZE + p.CRC_SIZE
    mt, seq, payload = p.decode_frame(frame)
    assert mt == int(p.MsgType.HEARTBEAT_J2T)
    assert seq == 7
    assert p.HeartbeatJ2T.unpack(payload).t_jetson_us == 42


def test_decode_rejects_bad_magic():
    frame = bytearray(p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 0, p.HeartbeatJ2T().pack()))
    frame[0] = 0  # corrupt magic
    with pytest.raises(ValueError, match="magic"):
        p.decode_frame(bytes(frame))


def test_decode_rejects_bad_crc():
    frame = bytearray(p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 0, p.HeartbeatJ2T().pack()))
    frame[-1] ^= 0xFF  # flip a CRC bit
    with pytest.raises(ValueError, match="CRC"):
        p.decode_frame(bytes(frame))


def test_decode_raises_typed_crc_error():
    # CRC corruption raises the typed CrcError (a ValueError subclass, so any
    # legacy `except ValueError` still catches it). The host RX path counts this
    # as a crc_error rather than a structural decode_error.
    frame = bytearray(p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 0, p.HeartbeatJ2T().pack()))
    frame[-1] ^= 0xFF  # flip a CRC bit
    assert issubclass(p.CrcError, ValueError)
    with pytest.raises(p.CrcError, match="CRC"):
        p.decode_frame(bytes(frame))


def test_decode_structural_error_is_not_crc_error():
    # A structural failure (bad magic — checked before the CRC) is a plain
    # ValueError, NOT a CrcError, so the two RX-stat buckets never
    # cross-contaminate (the bug the old `"CRC" in str(e)` match risked).
    frame = bytearray(p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 0, p.HeartbeatJ2T().pack()))
    frame[0] ^= 0xFF  # corrupt magic
    with pytest.raises(ValueError) as exc:
        p.decode_frame(bytes(frame))
    assert not isinstance(exc.value, p.CrcError)


def test_decode_rejects_inconsistent_length():
    # Header says length=99 but frame too short to actually carry it
    blob = b"\x00" * 4
    head = struct.pack("<HBBHH", p.MAGIC, p.PROTOCOL_VERSION, int(p.MsgType.HEARTBEAT_J2T), 0, 99)
    crc = p.crc16_ccitt(head + blob)
    bogus = head + blob + struct.pack("<H", crc)
    with pytest.raises(ValueError, match="length"):
        p.decode_frame(bogus)


def test_seq_wraps_at_16_bits():
    # The encoded seq field truncates to uint16 — make sure a request for seq=70000
    # still produces a valid frame whose decoded seq is the low 16 bits.
    blob = b"\x00" * 4
    frame = p.encode_frame(int(p.MsgType.HEARTBEAT_J2T), 70_000, p.HeartbeatJ2T().pack())
    _, seq, _ = p.decode_frame(frame)
    assert seq == 70_000 & 0xFFFF
