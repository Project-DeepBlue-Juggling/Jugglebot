"""Cross-reference the host HAND_TRAJ_CMD encoders against can_node's 0x6D0 wire.

Phase 5 (canbridge-foundation-coldstart-parity) restores the hand catch-trajectory
+ smooth-move surface. can_node built two 8-byte PLATFORM_TRAJ_CMD (0x6D0) payloads
on the Jetson and put them on CAN3 directly (_send_hand_traj_cmd, _smooth_move_hand,
can_node.py:1626-1661). The can-bridge builds the SAME 8 bytes host-side
(rpc_args.encode_hand_traj_cmd / encode_smooth_move_hand) and the firmware forwards
them on the FIRMWARE-OWNED 0x6D0 id after the CLOSED_LOOP + POSITION/PASSTHROUGH
preamble. So byte-parity of the PAYLOAD is the contract — the Platform Teensy's
PLATFORM_TRAJ_CMD decoder is unchanged, so a byte drift would silently mis-arm a
catch. These tests transcribe can_node's payload construction 1:1 and assert the
host encoders produce byte-identical output for fixed inputs.

Determinacy: the ABSOLUTE wall_time_ms is PASSED IN (the service handler computes
now_ms + event_delay*1000, exactly as can_node did; the encoder is pure), so the
byte layout is testable with fixed inputs and NO wall-clock. The firmware never
re-stamps the deadline — an absolute deadline is immune to Jetson→bridge→CAN3
transit jitter (the temporal-accuracy contract; the BallButler 2026-06-18 arc).

Ground truth: can_node.py:1639-1644 (_send_hand_traj_cmd), :1658 (_smooth_move_hand).
Pure stdlib + the host encoder module (no ROS2, no g++, no hardware).
"""

from __future__ import annotations

import struct

import pytest

from controller.teensy_link import rpc_args


# ── can_node reference (transcribed 1:1) ──────────────────────────────────────

def cannode_traj_payload(event_vel: float, wall_time_ms: int, traj_type: int) -> bytes:
    """can_node.py:1639-1644 — the 8-byte 0x6D0 payload for a catch trajectory."""
    wall_time_ms_low32 = wall_time_ms & 0xFFFFFFFF
    vel_u16 = int(round(event_vel * 100))
    payload = bytes([traj_type & 0xFF, vel_u16 & 0xFF, (vel_u16 >> 8) & 0xFF])
    payload += struct.pack('<I', wall_time_ms_low32) + bytes([0])
    return payload


def cannode_smooth_payload(target_rev: float) -> bytes:
    """can_node.py:1658 — the 8-byte 0x6D0 payload for a smooth-move."""
    return bytes([3]) + struct.pack('<f', target_rev) + bytes(3)


# ── catch-trajectory payload byte-parity ──────────────────────────────────────

@pytest.mark.parametrize("traj_type", [0, 1, 2])
@pytest.mark.parametrize("event_vel,wall_time_ms", [
    (0.3, 0),
    (3.75, 1723456789123),
    (7.0, 0xFFFFFFFF + 5),   # exercises the low-32 mask
    (1.0, 42),
])
def test_traj_payload_byte_identical(traj_type, event_vel, wall_time_ms):
    got = rpc_args.encode_hand_traj_cmd(traj_type, event_vel, wall_time_ms)
    ref = cannode_traj_payload(event_vel, wall_time_ms, traj_type)
    assert got == ref, (got.hex(), ref.hex())
    assert len(got) == 8


def test_traj_payload_layout_fields():
    """Spell out the layout the Platform Teensy decodes: byte 0 = traj_type,
    bytes 1-2 = vel_u16 (LE), bytes 3-6 = wall_time_ms low32 (LE), byte 7 = 0."""
    p = rpc_args.encode_hand_traj_cmd(2, 5.55, 0x11223344)
    assert p[0] == 2
    assert (p[1] | (p[2] << 8)) == int(round(5.55 * 100))     # 555
    assert struct.unpack('<I', p[3:7])[0] == 0x11223344
    assert p[7] == 0


def test_wall_time_truncates_to_low32():
    """can_node masks wall_time_ms to the low 32 bits (& 0xFFFFFFFF); a value above
    2^32 wraps identically on both sides (the Platform Teensy compares mod 2^32)."""
    big = (1 << 40) | 0x0BADF00D
    got = rpc_args.encode_hand_traj_cmd(1, 3.0, big)
    assert struct.unpack('<I', got[3:7])[0] == (big & 0xFFFFFFFF) == 0x0BADF00D


# ── smooth-move payload byte-parity ───────────────────────────────────────────

@pytest.mark.parametrize("target_rev", [0.0, 0.1, 5.5, 9.858, 11.1])
def test_smooth_payload_byte_identical(target_rev):
    got = rpc_args.encode_smooth_move_hand(target_rev)
    ref = cannode_smooth_payload(target_rev)
    assert got == ref, (got.hex(), ref.hex())
    assert len(got) == 8


def test_smooth_discriminator_is_3():
    """Byte 0 = 3 disambiguates a smooth-move from a catch traj (0/1/2) for the
    Platform Teensy's PLATFORM_TRAJ_CMD decoder."""
    p = rpc_args.encode_smooth_move_hand(4.2)
    assert p[0] == 3
    assert abs(struct.unpack('<f', p[1:5])[0] - 4.2) < 1e-6
    assert p[5:8] == b"\x00\x00\x00"


def test_traj_and_smooth_distinguished_by_byte0():
    """A catch traj (byte0 ∈ {0,1,2}) is never confused with a smooth-move (byte0=3)
    — the single-RPC discriminator contract."""
    for tt in (0, 1, 2):
        assert rpc_args.encode_hand_traj_cmd(tt, 3.0, 1000)[0] == tt
    assert rpc_args.encode_smooth_move_hand(1.0)[0] == 3


def test_arghandtraj_is_the_raw_payload():
    """ArgHandTraj(payload[8]).pack() IS the 8 payload bytes verbatim — the firmware
    attaches the 0x6D0 id and forwards them, so the RPC arg carries no wrapper."""
    p = rpc_args.encode_smooth_move_hand(2.0)
    a = rpc_args.ArgHandTraj.unpack(p)
    assert bytes(a.payload) == p