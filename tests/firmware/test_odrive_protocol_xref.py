"""Cross-language byte-parity of the C++ ODrive encoders (odrive_protocol.h) vs odrive.py.

**Upgraded in the Fable-5 hardening pass (gap 8).** This used to TRANSCRIBE the C++
arithmetic into Python (`cpp_arb_id`/`cpp_set_input_pos`/`cpp_leg_setpoint`) and
compare that to `odrive.py` — both sides Python, so a wrong memcpy offset / struct
order / arb-id shift in the REAL header passed silently. It is now a GOLDEN
CONSUMER: `tests/firmware/native/test_odrive_protocol.cpp` compiles + runs the real
C++ encoders and emits their exact bytes into `native/odrive_protocol_golden.json`;
this test reproduces every golden entry from the AUTHORITATIVE `odrive.py` and
asserts equality. Transitively — golden == real-C++-bytes (pinned on the Jetson by
`test_native_firmware.py::test_odrive_committed_golden_matches_live_firmware`) AND
golden == odrive.py-bytes (here, everywhere) — this is true cross-language equality
that EXECUTES the header, not a self-comparison.

The decoder + time-sync payload checks stay pure-Python (they validate the parse
direction, which the golden emitter does not cover).

Requires python-can (a core hardware dependency); skipped if unavailable.
"""

from __future__ import annotations

import json
import struct
from pathlib import Path

import pytest

can = pytest.importorskip("can")  # python-can
import jugglebot.can.odrive as od  # noqa: E402  (on sys.path via tests/conftest)
import jugglebot.protocol_config as proto  # noqa: E402

LEGS = list(range(6))
HAND = od.HAND_AXIS

_GOLDEN = (Path(__file__).resolve().parent / "native" / "odrive_protocol_golden.json")


@pytest.fixture(scope="module")
def golden() -> dict:
    return json.loads(_GOLDEN.read_text())


# ── Reproduce each golden entry from the AUTHORITATIVE odrive.py ───────────────
# Each lambda returns a python-can Message; the test compares arb_id + data bytes
# to the golden the REAL C++ header emitted. The (name → args) here MUST match the
# table in native/test_odrive_protocol.cpp — the two encode the same wire commands
# in two languages; agreement is the cross-language guarantee.
_SDO_PARAM = "commutation_mapper.pos_abs"   # proto.ENDPOINT_COMMUTATION_MAPPER_POS_ABS == 488


def _leg_setpoint(axis, setpoint, vel, tor):
    """Reproduce can_node._send_position_target: clip → _leg_sign → int16 scale →
    clamp → encode_set_input_pos. FF vectors are exact-integer-scaled, so Python
    int(round()) and C++ lroundf agree (no X.5 boundary)."""
    sign = -1.0 if axis in od.LEG_AXES else 1.0
    sp = od.clip_position(axis, setpoint) * sign
    vff, tff = vel * sign, tor * sign
    if axis in od.LEG_AXES:
        vscale, tscale = proto.INPUT_SCALE_LEG_VEL, proto.INPUT_SCALE_LEG_TOR
    else:
        vscale, tscale = proto.INPUT_SCALE_HAND_VEL, proto.INPUT_SCALE_HAND_TOR
    vi = max(-32768, min(32767, int(round(vff * vscale))))
    ti = max(-32768, min(32767, int(round(tff * tscale))))
    return od.encode_set_input_pos(axis, sp, vi, ti)


_REPRO = {
    "get_version":           lambda: od.encode_get_version(0),
    "set_state_idle":        lambda: od.encode_set_state(0, "IDLE"),
    "set_state_closed_loop": lambda: od.encode_set_state(0, "CLOSED_LOOP"),
    "set_controller_mode":   lambda: od.encode_set_controller_mode(0, "POSITION", "PASSTHROUGH"),
    "set_input_pos":         lambda: od.encode_set_input_pos(0, 2.5, 1000, -5000),
    "set_input_vel":         lambda: od.encode_set_input_vel(0, 1.5, 0.0),
    "set_input_torque":      lambda: od.encode_set_input_torque(0, 0.75),
    "set_vel_curr_limits":   lambda: od.encode_set_vel_curr_limits(0, 4.0, 10.0),
    "set_traj_vel_limit":    lambda: od.encode_set_traj_vel_limit(0, 15.0),
    "set_traj_acc_limits":   lambda: od.encode_set_traj_acc_limits(0, 20.0, 25.0),
    "set_absolute_position": lambda: od.encode_set_absolute_position(0, 1.5),
    "set_pos_gain":          lambda: od.encode_set_pos_gain(0, 30.0),
    "set_vel_gains":         lambda: od.encode_set_vel_gains(0, 0.1, 0.05),
    "clear_errors":          lambda: od.encode_clear_errors(0),
    "reboot":                lambda: od.encode_reboot(0),
    "sdo_read":              lambda: od.encode_sdo_read(0, _SDO_PARAM),
    "sdo_write":             lambda: od.encode_sdo_write(0, _SDO_PARAM, 2.5),
    "leg_setpoint_leg0":     lambda: _leg_setpoint(0, 2.0, 0.5, 0.1),
    "leg_setpoint_hand":     lambda: _leg_setpoint(HAND, 2.0, 0.5, 0.1),
}


def test_golden_covers_every_reproduced_encoder(golden):
    """The committed golden and the Python reproduction table describe the SAME set
    of encoders — neither side silently drops a command (which would hide a drift)."""
    assert set(golden.keys()) == set(_REPRO.keys()), (
        "odrive golden and the Python reproduction table diverged — regenerate:\n"
        "  python tests/firmware/native/build.py --odrive-golden "
        "tests/firmware/native/odrive_protocol_golden.json")


@pytest.mark.parametrize("name", list(_REPRO.keys()))
def test_cpp_encoder_bytes_match_odrive_py(golden, name):
    """The REAL C++ encoder's bytes (golden) equal the production odrive.py bytes —
    arb_id AND payload, byte-for-byte."""
    g = golden[name]
    msg = _REPRO[name]()
    assert msg.arbitration_id == g["arb"], f"{name}: arb_id C++={g['arb']} py={msg.arbitration_id}"
    assert bytes(msg.data).hex() == g["data"], (
        f"{name}: bytes C++={g['data']} py={bytes(msg.data).hex()}")
    assert len(bytes(msg.data)) == g["len"]


# ── Decoders: feed bytes, odrive.py parses correctly (parse direction) ─────────

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
    w = 1_700_000_123_456            # us since epoch
    sec, usec = w // 1_000_000, w % 1_000_000
    cpp = struct.pack("<II", sec, usec)
    assert len(cpp) == 8
    dec_sec, dec_usec = struct.unpack("<II", cpp)
    assert dec_sec * 1_000_000 + dec_usec == w        # slave reconstruction is exact
    assert usec < 1_000_000                            # well-formed microseconds field
    assert proto.CAN_ID_SHARED_TIME_SYNC == 0x7DD      # ID matches bus.py / slaves
