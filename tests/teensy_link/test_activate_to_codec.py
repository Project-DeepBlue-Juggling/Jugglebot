"""ACTIVATE_TO (can-bridge FW 51, BUG-130): the host codec and the band twin.

The firmware (``leg_activate.cpp`` ``activate_to_request``) is the enforcement
point for the band; these pin the Python side to it: the wire layout (six f32,
24 B, method 0x66), the band constant equal to ``canbridge_config.h``, and the
host's pre-send band check agreeing with the firmware's at the edges.
"""
from __future__ import annotations

import math
import re
from pathlib import Path

import pytest

import jugglebot.hardware_config as hw
from teensy_link import protocol as p
from teensy_link import rpc_args
from teensy_link.rpc import NON_IDEMPOTENT_METHODS

_REPO = Path(__file__).resolve().parents[2]
_CFG_H = _REPO / "ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h"
_CONST = list(hw.JB_OP_ACTIVATE_POSITION_REVS)


def test_method_id_and_wire_layout():
    assert int(p.RpcMethod.ACTIVATE_TO) == 0x0066
    t = [c + 0.01 * i for i, c in enumerate(_CONST)]
    blob = rpc_args.encode_activate_to(t)
    assert len(blob) == 24
    import struct
    assert struct.unpack("<6f", blob) == pytest.approx(t, abs=1e-6)


def test_encode_refuses_a_wrong_count():
    with pytest.raises(ValueError):
        rpc_args.encode_activate_to(_CONST[:5])


def test_band_twin_equals_the_firmware_constant():
    m = re.search(r"constexpr float ACTIVATE_TO_BAND_REV\s*=\s*([0-9.]+)f;",
                  _CFG_H.read_text())
    assert m, "ACTIVATE_TO_BAND_REV not found in canbridge_config.h"
    assert float(m.group(1)) == rpc_args.ACTIVATE_TO_BAND_REV


@pytest.mark.parametrize("sign", [+1.0, -1.0])
def test_band_edges_match_the_firmware_rule(sign):
    band = rpc_args.ACTIVATE_TO_BAND_REV
    for leg in range(6):
        inside = list(_CONST)
        inside[leg] += sign * (band - 0.001)
        assert rpc_args.activate_to_out_of_band(inside, _CONST) == []
        outside = list(_CONST)
        outside[leg] += sign * (band + 0.001)
        assert rpc_args.activate_to_out_of_band(outside, _CONST) == [leg]


def test_non_finite_is_out_of_band():
    for bad in (math.nan, math.inf, -math.inf):
        t = list(_CONST)
        t[2] = bad
        assert rpc_args.activate_to_out_of_band(t, _CONST) == [2]


def test_activate_to_is_never_blind_retried():
    assert int(p.RpcMethod.ACTIVATE_TO) in NON_IDEMPOTENT_METHODS
