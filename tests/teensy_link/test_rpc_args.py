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


def test_method_arg_association_covers_all_commandable_methods():
    from controller.teensy_link import RpcMethod
    expected = {
        RpcMethod.SET_AXIS_STATE, RpcMethod.SET_CONTROLLER_MODE,
        RpcMethod.SET_VEL_CURR_LIMITS, RpcMethod.SET_POS_GAIN,
        RpcMethod.SET_VEL_GAINS, RpcMethod.SET_ABSOLUTE_POSITION,
        RpcMethod.CLEAR_ERRORS, RpcMethod.REBOOT_ODRIVES,
        RpcMethod.ENCODER_SEARCH, RpcMethod.HOME,
        RpcMethod.SDO_READ, RpcMethod.SDO_WRITE,
    }
    assert set(ra.METHOD.keys()) == expected
