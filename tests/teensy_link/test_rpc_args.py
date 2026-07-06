"""Byte-layout + round-trip tests for controller/teensy_link/rpc_args.py.

Validates the codegen-hoisted RPC argument encoders against the EXACT byte
layouts the firmware rpc.h / rpc.cpp expect (packed, little-endian). These are
the same generated structs the firmware consumes via ``using JbUdp::RpcArgs::*``,
so a mismatch here is a mismatch on the wire.

Reference layouts (from rpc.h, pre-hoist):
    ArgAxisState     { u8 axis; u32 state; }                 -> 5 B  <BI
    ArgControllerMode{ u8 axis; u32 ctrl; u32 input; }       -> 9 B  <BII
    ArgVelCurr       { u8 axis; f32 vel_limit; f32 curr; }   -> 9 B  <Bff
    ArgPosGain       { u8 axis; f32 pos_gain; }              -> 5 B  <Bf
    ArgVelGains      { u8 axis; f32 vel_gain; f32 vel_int; } -> 9 B  <Bff
    ArgAbsPosition   { u8 axis; f32 position; }              -> 5 B  <Bf
    ArgAxisOnly      { u8 axis; }                            -> 1 B  <B
    ArgSdoRead       { u8 axis; u16 endpoint; }              -> 3 B  <BH
    ArgSdoWrite      { u8 axis; u16 endpoint; f32 value; }   -> 7 B  <BHf
    ResultTimeOfDay  { u64 jetson_wall_us; }                 -> 8 B  <Q
"""

from __future__ import annotations

import struct

import pytest

from controller.teensy_link import rpc_args as ra
from controller.teensy_link.protocol import (
    ArgAxisState, ArgControllerMode, ArgVelCurr, ArgPosGain, ArgVelGains,
    ArgAbsPosition, ArgAxisOnly, ArgSdoRead, ArgSdoWrite, ResultTimeOfDay,
    ArgBbThrow,
)


def test_axis_all_sentinel():
    assert ra.AXIS_ALL == 0xFF


# ── Exact byte layouts (the ground truth the firmware memcpy's) ───────────────

def test_set_axis_state_exact_bytes():
    blob = ra.encode_set_axis_state(axis=3, state=8)
    assert blob == struct.pack('<BI', 3, 8)
    assert len(blob) == 5
    a = ArgAxisState.unpack(blob)
    assert a.axis == 3 and a.state == 8


def test_set_controller_mode_exact_bytes():
    blob = ra.encode_set_controller_mode(axis=2, ctrl=3, input_mode=1)
    assert blob == struct.pack('<BII', 2, 3, 1)
    assert len(blob) == 9


def test_set_vel_curr_limits_exact_bytes():
    blob = ra.encode_set_vel_curr_limits(axis=1, vel_limit=4.0, curr_limit=20.0)
    assert blob == struct.pack('<Bff', 1, 4.0, 20.0)
    assert len(blob) == 9


def test_set_pos_gain_exact_bytes():
    blob = ra.encode_set_pos_gain(axis=0, pos_gain=35.0)
    assert blob == struct.pack('<Bf', 0, 35.0)
    assert len(blob) == 5


def test_set_vel_gains_exact_bytes():
    blob = ra.encode_set_vel_gains(axis=5, vel_gain=0.007, vel_int_gain=0.07)
    assert blob == struct.pack('<Bff', 5, 0.007, 0.07)
    assert len(blob) == 9


def test_set_absolute_position_exact_bytes():
    blob = ra.encode_set_absolute_position(axis=4, position=2.19)
    assert blob == struct.pack('<Bf', 4, 2.19)
    assert len(blob) == 5


def test_clear_errors_and_reboot_default_broadcast():
    assert ra.encode_clear_errors() == struct.pack('<B', 0xFF)
    assert ra.encode_reboot() == struct.pack('<B', 0xFF)
    assert ra.encode_clear_errors(axis=2) == struct.pack('<B', 2)
    assert len(ra.encode_clear_errors()) == 1


def test_encoder_search_home_use_axis_only():
    assert ra.encode_encoder_search() == struct.pack('<B', 0xFF)
    assert ra.encode_home() == struct.pack('<B', 0xFF)


def test_sdo_read_exact_bytes():
    blob = ra.encode_sdo_read(axis=1, endpoint=0x1234)
    assert blob == struct.pack('<BH', 1, 0x1234)
    assert len(blob) == 3


def test_sdo_write_exact_bytes():
    blob = ra.encode_sdo_write(axis=1, endpoint=0x00AB, value=1.5)
    assert blob == struct.pack('<BHf', 1, 0x00AB, 1.5)
    assert len(blob) == 7


def test_time_of_day_result_roundtrip():
    blob = ResultTimeOfDay(jetson_wall_us=1_700_000_000_000_000).pack()
    assert blob == struct.pack('<Q', 1_700_000_000_000_000)
    assert ra.decode_time_of_day_result(blob) == 1_700_000_000_000_000


# ── Float round-trips with f32 tolerance ──────────────────────────────────────

def test_float_fields_roundtrip_f32():
    a = ArgVelGains.unpack(ra.encode_set_vel_gains(2, 0.007, 0.07))
    assert a.axis == 2
    assert a.vel_gain == pytest.approx(0.007, rel=1e-6)
    assert a.vel_int_gain == pytest.approx(0.07, rel=1e-6)


# ── Ball Butler ─────────────────────────────────────────────────────

def test_bb_throw_exact_bytes():
    blob = ra.encode_bb_throw(yaw_rad=0.5, pitch_rad=0.3,
                              speed_mps=3.0, delay_s=0.1)
    assert blob == struct.pack('<ffff', 0.5, 0.3, 3.0, 0.1)
    assert len(blob) == 16
    a = ArgBbThrow.unpack(blob)
    assert a.yaw_rad == pytest.approx(0.5, rel=1e-6)
    assert a.pitch_rad == pytest.approx(0.3, rel=1e-6)
    assert a.speed_mps == pytest.approx(3.0, rel=1e-6)
    assert a.delay_s == pytest.approx(0.1, rel=1e-6)


def test_state_write_exact_bytes():
    # STATE_WRITE (Platform-Teensy relay): flags byte (is_homed/levelling
    # are encoded into the 0x6E0 CAN frame by the FIRMWARE; the RPC arg carries
    # them as two u8 + the two pose floats). Layout '<BBff'.
    blob = ra.encode_state_write(is_homed=True, levelling_complete=False,
                                 pose_offset_tiltX=0.01, pose_offset_tiltY=-0.02)
    assert blob == struct.pack('<BBff', 1, 0, 0.01, -0.02)
    assert len(blob) == 10
    a = ra.ArgRobotState.unpack(blob)
    assert a.is_homed == 1 and a.levelling_complete == 0
    assert a.pose_offset_tiltX == pytest.approx(0.01, rel=1e-5)
    assert a.pose_offset_tiltY == pytest.approx(-0.02, rel=1e-5)


def test_bb_payloadless_commands_are_empty():
    # RELOAD/RESET/CALIBRATE_LOC ride a 0-byte args payload — matches NOP shape.
    # The Teensy ignores any trailing bytes on these methods anyway, but the
    # bridge sends b"" for clarity.
    assert ra.encode_bb_reload()        == b""
    assert ra.encode_bb_reset()         == b""
    assert ra.encode_bb_calibrate_loc() == b""


def test_axis_versions_result_roundtrip_and_layout():
    """GET_AXIS_VERSIONS result blob: mask byte + axis-major 8-byte raw
    payloads. encode/decode round-trip, the set bits match the supplied axes, and
    the byte layout is exactly mask(1) + NUM_AXES*8 (the firmware version_fill_blob
    mirror)."""
    v0 = bytes([0x00, 0x03, 0x06, 0x00, 0x00, 0x06, 0x0B, 0x00])  # axis 0 (leg)
    v6 = bytes([0x00, 0x04, 0x04, 0x3A, 0x00, 0x06, 0x0B, 0x00])  # axis 6 (hand)
    blob = ra.encode_axis_versions_result({0: v0, 6: v6})

    # Exact layout: 57 bytes = 1 mask + 7*8 raw; mask has bits 0 and 6.
    assert len(blob) == 1 + ra._NUM_VERSION_AXES * 8 == 57
    assert blob[0] == (1 << 0) | (1 << 6)
    assert blob[1:9] == v0                          # axis 0 slot
    assert blob[1 + 6 * 8:1 + 7 * 8] == v6          # axis 6 slot
    assert blob[1 + 1 * 8:1 + 2 * 8] == bytes(8)    # axis 1 slot zero-filled (unreceived)

    got = ra.decode_axis_versions_result(blob)
    assert set(got.keys()) == {0, 6}
    assert got[0] == v0 and got[6] == v6

    # Empty (nothing swept yet) round-trips to no axes.
    assert ra.decode_axis_versions_result(ra.encode_axis_versions_result({})) == {}

    # Out-of-range axis / wrong-length payload are rejected.
    with pytest.raises(ValueError):
        ra.encode_axis_versions_result({99: v0})
    with pytest.raises(ValueError):
        ra.encode_axis_versions_result({0: b"\x00" * 7})


def test_method_arg_association_covers_all_commandable_methods():
    """NON-tautological: partition the WHOLE RpcMethod enum into
    payloadless vs arg-carrying and freeze it, rather than hand-copying
    ``METHOD.keys()`` (which only caught "someone edited METHOD"). A NEW arg-carrying
    method added to the enum without a METHOD entry now lands in the computed
    'missing' set below and FAILS — the useful direction."""
    from controller.teensy_link import RpcMethod
    # The authoritative protocol fact: methods that carry NO request args (correctly
    # absent from METHOD). TIME_OF_DAY_QUERY is Teensy→Jetson (server-side); the
    # reads (TILT_READ/STATE_READ) + BB reload/reset/calibrate + GET_AXIS_VERSIONS
    # are payloadless requests.
    payloadless = {
        RpcMethod.NOP, RpcMethod.TIME_OF_DAY_QUERY,
        RpcMethod.BB_RELOAD, RpcMethod.BB_RESET, RpcMethod.BB_CALIBRATE_LOC,
        RpcMethod.GET_AXIS_VERSIONS, RpcMethod.TILT_READ, RpcMethod.STATE_READ,
    }
    have = set(ra.METHOD.keys())
    missing = {m for m in RpcMethod if m not in payloadless and m not in have}
    unexpected = {m for m in have if m in payloadless}
    assert not missing, f"arg-carrying methods missing a METHOD entry: {missing}"
    assert not unexpected, f"payloadless methods wrongly in METHOD: {unexpected}"
    # Full-partition freeze: every enum member is classified exactly once.
    assert have | payloadless == set(RpcMethod)
    assert not (have & payloadless)
    # DEACTIVATE is axis-only, like ACTIVATE/HOME.
    assert ra.METHOD[RpcMethod.DEACTIVATE] is ra.METHOD[RpcMethod.ACTIVATE]
