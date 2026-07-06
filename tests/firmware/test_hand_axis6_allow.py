"""Hand axis-6 allow-table — single-source consistency.

The can-bridge replaced its blanket `axis == HAND_AXIS` reject (rpc.cpp) with a
NARROW (method, axis) allow-table: which RpcMethods it forwards to the hand ODrive
(axis 6) on CAN3. The table is generated ONCE (config/generate_udp_protocol.py)
into both a C++ predicate (JbUdp::hand_axis6_permitted, consumed by rpc.cpp's
send_axis_frame) and a Python frozenset (controller.teensy_link). This test pins:

  1. the Python set == the operator-locked permit list (a rename/drop fails here,
     not silently on the bench);
  2. the generated C++ predicate lists EXACTLY the same RpcMethod names (so the
     firmware gate and the Jetson mirror can never drift);
  3. the reject list (ENCODER_SEARCH / ACTIVATE / DEACTIVATE) is excluded.

The compiled C++ predicate's runtime behaviour is additionally asserted in the
native harness (tests/firmware/native/test_platform_relay.cpp).
"""

from __future__ import annotations

import re
from pathlib import Path

from controller.teensy_link import RpcMethod
from controller.teensy_link import hand_axis6_permitted
from controller.teensy_link.protocol import HAND_AXIS6_PERMITTED

_FIRMWARE_H = (Path(__file__).resolve().parents[2]
               / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge"
               / "udp_protocol.h")

# The operator-locked policy (plan "Hand axis-6 allow-table", locked 2026-06-29).
_PERMIT = {
    "SET_AXIS_STATE", "SET_CONTROLLER_MODE", "SET_POS_GAIN", "SET_VEL_GAINS",
    "SET_VEL_CURR_LIMITS", "CLEAR_ERRORS", "REBOOT_ODRIVES", "HOME",
    "SET_ABSOLUTE_POSITION",
}
_REJECT = {"ENCODER_SEARCH", "ACTIVATE", "DEACTIVATE"}


def test_python_allow_set_matches_locked_policy():
    permitted_names = {RpcMethod(m).name for m in HAND_AXIS6_PERMITTED}
    assert permitted_names == _PERMIT


def test_python_predicate_permits_and_rejects():
    for name in _PERMIT:
        assert hand_axis6_permitted(getattr(RpcMethod, name)), name
    for name in _REJECT:
        assert not hand_axis6_permitted(getattr(RpcMethod, name)), name
    # A few extra non-hand ops must also be rejected (default-false).
    for name in ("SDO_READ", "SDO_WRITE", "NOP", "BB_THROW"):
        assert not hand_axis6_permitted(getattr(RpcMethod, name)), name


def test_generated_cpp_predicate_matches_python_set():
    """The C++ hand_axis6_permitted() switch lists exactly the Python permit set —
    the single-source guard (both are generated; this catches a hand-edit of one)."""
    text = _FIRMWARE_H.read_text()
    body = re.search(
        r"inline bool hand_axis6_permitted\(uint16_t method\) \{(.*?)\n\}",
        text, re.DOTALL)
    assert body, "hand_axis6_permitted() not found in the delivered udp_protocol.h"
    cpp_cases = set(re.findall(r"case RpcMethod::(\w+):", body.group(1)))
    assert cpp_cases == _PERMIT
