"""T-U7 leg-lane byte-exactness + T-R2 version darkness, against CAPTURED v5 fixtures.

``tests/teensy_link/data/v5_pump_wire_fixtures.json`` was captured at commit
2aaaae1 (2026-09-01, the last pre-v6 checkout) by
``tools/probes/capture_v5_wire_fixtures.py`` — READ-ONLY ground truth, never
regenerate it (the capture script refuses to run post-v6 by design). Each case
replays recorded ``mpc_cmd`` dicts through a recorded pump configuration and
pins the exact v5 ``Setpoint`` payload and full UDP frame bytes.

What this file asserts:

* **T-U7 / T-R1 — leg lanes are byte-identical across the v6 widening.** With
  no hand keys, today's pump must produce EXACTLY the recorded leg bytes: for
  each of the seven v6 arrays' leg lanes (the first 24 bytes of each 28-byte
  block) a bit-for-bit compare against the recorded v5 payload's corresponding
  24-byte block, plus the flags/t_origin_us tail — float compares via raw
  struct bytes, never ``==`` on floats. The hand lane (bytes 24:28 of each
  block) and the whole v1 block must be zero bytes with the flags' hand/v1
  bits clear.

* **T-R2 — version skew is loud, with REAL recorded bytes.** The recorded v5
  wire frames (header version byte 5, valid CRC) are exactly what a
  not-yet-reflashed FW ≤ 16 board's traffic looks like; the live v6
  ``decode_frame`` must reject EVERY one of them on the version check (a
  structural ValueError, before the CRC is even consulted). The recorded
  header/CRC are additionally verified here by a frozen v5-layout reader —
  the live codec can no longer play the v5 side, which is the point.

The v5 payload layout is FROZEN inside this file (36 f32 + u32 + u64 = 156 B):
it is a historical wire format, so it must never track the live generator.
"""

from __future__ import annotations

import json
import struct
from pathlib import Path

import pytest

from teensy_link import protocol as p
from teensy_link.setpoint_pump import (
    SetpointPump, FLAG_HAS_HAND, FLAG_HAS_V1,
)

_FIXTURE = Path(__file__).resolve().parent / 'data' / 'v5_pump_wire_fixtures.json'

# ── The FROZEN v5 Setpoint layout (pre-2026-09-01) — do not derive from the
# live module, whose layout has moved on. 6 lanes × 6 arrays of f32, then
# u32 flags + u64 t_origin_us.
_V5_SETPOINT_SIZE = 156
_V5_ARRAYS = 6          # u0, u1, u2, v0, accel, torque_ff
_V5_LANES = 6
_V5_HEADER_FMT = '<HBBHH'   # magic, version, msg_type, seq, length (unchanged in v6)

# The live v6 layout constants this file slices with (pinned, not derived, so
# a layout regression fails HERE too rather than being sliced around).
_V6_LANES = 7
_V6_ARRAYS = 7          # + v1


def _fixture():
    with open(_FIXTURE) as f:
        return json.load(f)


def _v5_payload_blocks(payload: bytes):
    """(list of six 24-byte array blocks, 12-byte flags+t_origin tail)."""
    assert len(payload) == _V5_SETPOINT_SIZE
    blocks = [payload[k * 24:(k + 1) * 24] for k in range(_V5_ARRAYS)]
    return blocks, payload[_V5_ARRAYS * 24:]


def _v6_payload_blocks(payload: bytes):
    """(leg-lane 24-byte slices, hand-lane 4-byte slices, v1 block, tail)."""
    assert len(payload) == p.SETPOINT_SIZE == 208
    legs, hands = [], []
    for k in range(_V6_ARRAYS):
        base = k * 4 * _V6_LANES
        legs.append(payload[base:base + 24])
        hands.append(payload[base + 24:base + 28])
    v1_block = payload[6 * 28:7 * 28]
    return legs, hands, v1_block, payload[_V6_ARRAYS * 28:]


def test_fixture_provenance_is_v5():
    fx = _fixture()
    assert fx['protocol_version'] == 5
    assert fx['setpoint_size'] == _V5_SETPOINT_SIZE
    assert fx['captured_at_commit'].startswith('2aaaae1')


@pytest.mark.parametrize('case_name', [
    c['case'] for c in _fixture()['cases']])
def test_leg_lanes_byte_identical_to_v5_capture(case_name):
    fx = _fixture()
    case = next(c for c in fx['cases'] if c['case'] == case_name)
    pump = SetpointPump(**case['pump_args'])
    for i, fr in enumerate(case['frames']):
        sp, reason = pump.build(fr['cmd'], t_origin_us=fr['t_origin_us'])
        assert reason is None and sp is not None, (
            f"{case_name}[{i}] rejected: {reason}")
        v6 = sp.pack()
        v5 = bytes.fromhex(fr['setpoint_payload_hex'])
        v5_blocks, v5_tail = _v5_payload_blocks(v5)
        v6_legs, v6_hands, v6_v1, v6_tail = _v6_payload_blocks(v6)
        # Every leg lane of every pre-existing array: bit-for-bit.
        for k in range(_V5_ARRAYS):
            assert v6_legs[k] == v5_blocks[k], (
                f"{case_name}[{i}] array {k} leg lanes differ from the v5 capture")
        # flags + t_origin_us: bit-for-bit (no hand/v1 bits with no hand keys).
        assert v6_tail == v5_tail
        assert sp.flags == fr['flags']
        assert not (sp.flags & (FLAG_HAS_HAND | FLAG_HAS_V1))
        # The new lanes are all-zero bytes: hand lane of each array + all of v1.
        for k, hb in enumerate(v6_hands):
            assert hb == b'\x00\x00\x00\x00', (
                f"{case_name}[{i}] array {k} hand lane not zero")
        assert v6_v1 == b'\x00' * 28


def test_recorded_v5_wire_frames_verify_under_the_frozen_reader():
    # The frozen reader proves the recorded frames really are well-formed v5:
    # magic/version/length coherent, CRC valid over header+payload, payload
    # exactly the recorded Setpoint bytes. This is the half the live codec can
    # no longer perform.
    fx = _fixture()
    checked = 0
    for case in fx['cases']:
        for fr in case['frames']:
            frame = bytes.fromhex(fr['wire_frame_hex'])
            magic, version, msg_type, seq, length = struct.unpack_from(
                _V5_HEADER_FMT, frame, 0)
            assert magic == p.MAGIC
            assert version == 5
            assert msg_type == fx['msg_type_setpoint']
            assert seq == fr['wire_seq']
            assert length == _V5_SETPOINT_SIZE
            assert len(frame) == 8 + length + 2
            want = p.crc16_ccitt(frame[:8 + length])       # CRC alg is unchanged
            (got,) = struct.unpack_from('<H', frame, 8 + length)
            assert want == got
            assert frame[8:8 + length] == bytes.fromhex(fr['setpoint_payload_hex'])
            checked += 1
    assert checked >= 12    # every recorded frame, all six cases


def test_live_v6_decoder_rejects_every_recorded_v5_frame():
    # T-R2: total link darkness, demonstrated on genuine v5 traffic. The
    # reject must be the structural version ValueError (not CrcError — version
    # is checked before the CRC), for EVERY frame.
    fx = _fixture()
    assert p.PROTOCOL_VERSION == 6      # the premise of the darkness
    for case in fx['cases']:
        for fr in case['frames']:
            frame = bytes.fromhex(fr['wire_frame_hex'])
            with pytest.raises(ValueError, match='version') as exc:
                p.decode_frame(frame)
            assert not isinstance(exc.value, p.CrcError)


def test_v6_frame_is_undecodable_as_v5_by_construction():
    # The inverse skew (v6 host → v5 board): a v5 decoder rejects on its
    # version check exactly as ours does, but even a version-blind v5 reader
    # cannot mistake the frame — the header length field says 208 where the
    # exact-size v5 unpack demands 156. Assert that structural mismatch here
    # so the "fail-closed in BOTH directions" claim rests on bytes, not prose.
    sp_pump = SetpointPump(mm_to_rev=(1.0,) * 6)
    sp, reason = sp_pump.build(
        {'type': 'mpc_cmd', 'motor_rev': [0.1] * 6, 'vel_mm_s': [0.0] * 6},
        t_origin_us=1)
    assert reason is None
    frame = p.encode_frame(int(p.MsgType.SETPOINT), 1, sp.pack())
    _, version, _, _, length = struct.unpack_from(_V5_HEADER_FMT, frame, 0)
    assert version == 6 != 5
    assert length == 208 != _V5_SETPOINT_SIZE
