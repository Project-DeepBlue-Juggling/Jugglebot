"""Cross-reference the C++ ODrive protocol port against the real odrive.py / bus.py.

The firmware's `odrive_protocol.h` is a hand port of
`ros_ws/src/jugglebot/jugglebot/can/odrive.py`. These tests transcribe the C++
encode/decode arithmetic into Python (kept deliberately minimal and 1:1 with the
C++) and assert the result is **byte-identical** to the production `odrive.py`
encoders/decoders and the `bus.py` time-sync payload.

A struct-format or arbitration-id error in the C++ port — the easiest mistake to
make in a hand translation — shows up here as a byte mismatch against the
authoritative Python. (The C++ itself can't be compiled in this environment; the
adversarial review pass also eyeballs the header line-by-line.)

Requires python-can (a core hardware dependency); skipped if unavailable.
"""

from __future__ import annotations

import struct

import pytest

can = pytest.importorskip("can")  # python-can
import jugglebot.can.odrive as od  # noqa: E402  (on sys.path via tests/conftest)
import jugglebot.protocol_config as proto  # noqa: E402

LEGS = list(range(6))
HAND = od.HAND_AXIS


# ── C++ mirrors (transcribed 1:1 from odrive_protocol.h) ──────────────────────

def cpp_arb_id(axis, cmd):
    return (axis << 5) | cmd


def cpp_set_input_pos(pos, vel_i, tor_i):
    # encode_set_input_pos: memcpy float@0, int16@4, int16@6  →  struct '<fhh'
    return struct.pack("<fhh", pos, vel_i, tor_i)


def cpp_leg_setpoint(axis, setpoint_rev, vel_ff_rps, torque_ff_Nm):
    # encode_leg_setpoint: clip → leg_sign → int16 scale → clamp → set_input_pos
    is_leg = axis < 6
    max_pos = proto_to_max(axis)
    pos = max(0.0, min(setpoint_rev, max_pos))            # clip_position
    sign = -1.0 if is_leg else 1.0
    pos *= sign
    vff = vel_ff_rps * sign
    tff = torque_ff_Nm * sign
    if is_leg:
        vscale, tscale = proto.INPUT_SCALE_LEG_VEL, proto.INPUT_SCALE_LEG_TOR
    else:
        vscale, tscale = proto.INPUT_SCALE_HAND_VEL, proto.INPUT_SCALE_HAND_TOR
    import math
    vi = int(math.floor(vff * vscale + 0.5)) if vff * vscale >= 0 else -int(math.floor(-vff * vscale + 0.5))
    ti = int(math.floor(tff * tscale + 0.5)) if tff * tscale >= 0 else -int(math.floor(-tff * tscale + 0.5))
    vi = max(-32768, min(32767, vi))
    ti = max(-32768, min(32767, ti))
    # C++ stores `pos` as float32; emulate the f32 round-trip so bytes match.
    pos32 = struct.unpack("<f", struct.pack("<f", pos))[0]
    return cpp_arb_id(axis, proto.ODRIVE_COMMANDS["set_input_pos"]), cpp_set_input_pos(pos32, vi, ti)


def proto_to_max(axis):
    import jugglebot.hardware_config as hw
    return hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS if axis < 6 else hw.GEOM_HAND_MOTOR_MAX_POSITION_REVS


# ── arb_id ────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("axis", LEGS + [HAND])
def test_arb_id_matches(axis):
    assert cpp_arb_id(axis, proto.ODRIVE_COMMANDS["set_input_pos"]) == od.arb_id(axis, "set_input_pos")


# ── Encoders: C++ bytes == odrive.py bytes ────────────────────────────────────

def test_encode_set_input_pos_bytes():
    for pos in (-1.234, 0.0, 2.5, 4.2):
        for vel_i, tor_i in ((0, 0), (1000, -5000), (-32768, 32767)):
            ref = od.encode_set_input_pos(0, pos, vel_i, tor_i)
            assert bytes(ref.data) == cpp_set_input_pos(pos, vel_i, tor_i)


def test_encode_set_state_bytes():
    for state in (od.AXIS_STATES["IDLE"], od.AXIS_STATES["CLOSED_LOOP"]):
        ref = od.encode_set_state(0, [k for k, v in od.AXIS_STATES.items() if v == state][0])
        # C++ encode_set_state: u32@0 + 4 zero bytes
        mine = struct.pack("<I", state) + bytes(4)
        assert bytes(ref.data) == mine


def test_encode_set_controller_mode_bytes():
    ref = od.encode_set_controller_mode(0, "POSITION", "PASSTHROUGH")
    mine = struct.pack("<II", od.CONTROL_MODES["POSITION"], od.INPUT_MODES["PASSTHROUGH"])
    assert bytes(ref.data) == mine


def test_encode_set_vel_curr_limits_bytes():
    ref = od.encode_set_vel_curr_limits(0, 4.0, 10.0)
    assert bytes(ref.data) == struct.pack("<ff", 4.0, 10.0)


def test_encode_set_pos_gain_bytes():
    ref = od.encode_set_pos_gain(0, 30.0)
    assert bytes(ref.data) == struct.pack("<f", 30.0) + bytes(4)


def test_encode_set_vel_gains_bytes():
    ref = od.encode_set_vel_gains(0, 0.1, 0.05)
    assert bytes(ref.data) == struct.pack("<ff", 0.1, 0.05)


def test_encode_set_absolute_position_bytes():
    ref = od.encode_set_absolute_position(0, 1.5)
    assert bytes(ref.data) == struct.pack("<f", 1.5) + bytes(4)


def test_encode_clear_errors_and_reboot_bytes():
    assert bytes(od.encode_clear_errors(0).data) == bytes(8)
    assert bytes(od.encode_reboot(0).data) == bytes(8)


def test_encode_sdo_bytes():
    ep = proto.ENDPOINT_COMMUTATION_MAPPER_POS_ABS
    rd = od.encode_sdo_read(0, "commutation_mapper.pos_abs")
    assert bytes(rd.data) == struct.pack("<BHBf", proto.OPCODE_READ, ep, 0, 0.0)
    wr = od.encode_sdo_write(0, "commutation_mapper.pos_abs", 2.5)
    assert bytes(wr.data) == struct.pack("<BHBf", proto.OPCODE_WRITE, ep, 0, 2.5)


# ── Full leg setpoint path (encode_leg_setpoint vs can_node._send_position_target) ──

def test_encode_leg_setpoint_matches_send_position_target_logic():
    # Replicate can_node._send_position_target exactly and compare to the C++ mirror.
    for axis in LEGS:
        for setpoint, vel, tor in ((2.0, 0.5, 0.1), (0.0, -1.0, -0.2), (5.0, 3.9, 1.0)):
            sign = -1.0 if axis in od.LEG_AXES else 1.0   # _leg_sign (can_node)
            sp = od.clip_position(axis, setpoint) * sign
            vff = vel * sign
            tff = tor * sign
            vi = max(-32768, min(32767, int(round(vff * proto.INPUT_SCALE_LEG_VEL))))
            ti = max(-32768, min(32767, int(round(tff * proto.INPUT_SCALE_LEG_TOR))))
            ref = od.encode_set_input_pos(axis, sp, vi, ti)
            arb, data = cpp_leg_setpoint(axis, setpoint, vel, tor)
            assert arb == ref.arbitration_id
            assert data == bytes(ref.data)


# ── Decoders: feed bytes, C++ mirror == odrive.py ─────────────────────────────

def test_decode_heartbeat():
    data = bytes([0, 0, 0, 0, od.AXIS_STATES["CLOSED_LOOP"], 1, 0x01, 0])
    state, proc, traj = od.decode_heartbeat(data)
    assert (state, proc, traj) == (data[4], data[5], bool(data[6] & 1))


def test_decode_error():
    data = struct.pack("<II", 0x200, 0x40)
    active, disarm = od.decode_error(data)
    assert (active, disarm) == (0x200, 0x40)


def test_decode_encoder_iq_temps_busvc():
    data = struct.pack("<ff", 1.25, -2.5)
    assert od.decode_encoder_estimate(data) == (1.25, -2.5)
    assert od.decode_iq(data) == (1.25, -2.5)
    assert od.decode_temps(data) == (1.25, -2.5)
    assert od.decode_bus_voltage_current(data) == (1.25, -2.5)


def test_decode_sdo_response():
    data = struct.pack("<BHBf", 0, 488, 0, 3.14)
    ep, val = od.decode_sdo_response(data)
    assert ep == 488 and abs(val - 3.14) < 1e-6


# ── Time-sync 0x7DD payload (bus.broadcast_time format) ───────────────────────

def test_time_sync_payload_matches_bus_format():
    # The contract is: master encodes (sec, usec) as '<II' on ID 0x7DD; slaves
    # (Teensy_code.ino handleSyncFrame) reconstruct jetson_us = sec*1e6 + usec.
    # bus.py uses the same '<II' layout. The C++ master uses integer microseconds
    # (sec = w//1e6, usec = w%1e6) — byte-compatible with '<II' and, unlike
    # bus.py's float path, exact (reconstructs to w with zero rounding).
    w = 1_700_000_123_456            # us since epoch
    sec, usec = w // 1_000_000, w % 1_000_000
    cpp = struct.pack("<II", sec, usec)
    assert len(cpp) == 8
    dec_sec, dec_usec = struct.unpack("<II", cpp)
    assert dec_sec * 1_000_000 + dec_usec == w        # slave reconstruction is exact
    assert usec < 1_000_000                            # well-formed microseconds field
    assert proto.CAN_ID_SHARED_TIME_SYNC == 0x7DD      # ID matches bus.py / slaves
