"""Source scan of the Hand fan wiring in ``Teensy_code_platform.ino`` and its config.

``tests/firmware/native/test_platform_hand_fan.cpp`` runs the decision logic in
``hand_fan.h`` as a binary. It cannot see the sketch, which is where the failures
this file pins would live: a fan fed by the wrong CAN id or left out of ``loop()``
compiles and stays silently off, a pin shared with the inclinometer's SPI or with
CAN takes the board's only other functions down, and a pin left undriven until
after the inclinometer wait is uncontrolled for as long as that wait lasts.
"""
from __future__ import annotations

import os
import re
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_INO = os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_platform",
                    "Teensy_code_platform.ino")
sys.path.insert(0, os.path.join(_REPO, "config", "generated"))

import hardware_config as hc  # noqa: E402
import protocol_config as pc  # noqa: E402


def _code() -> str:
    with open(_INO, encoding="utf-8") as fh:
        text = fh.read()
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", text)


def _function_body(name: str) -> str:
    code = _code()
    start = re.search(r"\bvoid\s+%s\s*\([^)]*\)\s*\{" % name, code)
    assert start is not None, f"no {name}() in the sketch"
    depth, i = 1, start.end()
    while depth:
        depth += {"{": 1, "}": -1}.get(code[i], 0)
        i += 1
    return code[start.end():i]


def test_the_pin_is_free_on_this_board():
    """Pins 10-13 are the SCL3300's SPI, 22/23 are this board's CAN controller."""
    assert hc.TEENSY_HAND_FAN_PIN not in (10, 11, 12, 13, 22, 23)


def test_the_thresholds_leave_a_gap():
    """A zero or inverted gap makes the fan chatter on a reading at the threshold."""
    assert hc.TEENSY_HAND_FAN_OFF_BELOW_C < hc.TEENSY_HAND_FAN_ON_ABOVE_C


def test_the_timeout_rides_through_missed_frames():
    """The Hand ODrive broadcasts every 100 ms; a few lost frames must not stop the fan."""
    assert hc.TEENSY_HAND_FAN_TEMP_TIMEOUT_MS >= 1000


def test_the_can_id_is_the_hand_get_temperature_frame():
    assert pc.ODRIVE_COMMANDS["get_temps"] == 0x15
    code = _code()
    assert re.search(
        r"HAND_TEMPS_ID\s*=\s*\(uint32_t\(NodeId::JUGGLEBOT_HAND\)\s*<<\s*5\)"
        r"\s*\|\s*ODriveCmd::get_temps\s*;", code), "the frame id must come from the protocol"


def test_can_callback_feeds_the_controller_with_the_remote_flag():
    assert re.search(
        r"msg\.id\s*==\s*HAND_TEMPS_ID\s*\)\s*\{?\s*"
        r"handFan\.on_temps_frame\(msg\.buf,\s*msg\.len,\s*msg\.flags\.remote,\s*millis\(\)\)",
        _function_body("canSniff"))


def test_command_is_applied_and_always_answered():
    """The status reply is how the host sees a refused command."""
    body = _function_body("canSniff")
    assert re.search(
        r"msg\.id\s*==\s*HAND_FAN_CMD_ID\s*\)\s*\{\s*"
        r"handFan\.on_command_frame\(msg\.buf,\s*msg\.len,\s*msg\.flags\.remote\);\s*"
        r"updateHandFan\(\);\s*sendHandFanStatus\(\);", body)


def test_defaults_sit_inside_the_runtime_bounds():
    """A default the board would refuse if sent back to it at runtime is a config error."""
    assert (hc.TEENSY_HAND_FAN_THRESHOLD_MIN_C <= hc.TEENSY_HAND_FAN_OFF_BELOW_C
            < hc.TEENSY_HAND_FAN_ON_ABOVE_C <= hc.TEENSY_HAND_FAN_THRESHOLD_MAX_C)


def test_status_interval_lets_a_bag_record_every_stretch():
    assert 0 < hc.TEENSY_HAND_FAN_STATUS_INTERVAL_MS <= 5000


def test_the_bridge_forwards_the_status_frame():
    """Off the list, the frame is counted as a bad axis and dropped, silently."""
    path = os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_canbridge",
                        "can_buses.h")
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    start = text.index("inline bool is_platform_reply_id")
    assert "PlatformCanId::HAND_FAN_STATUS" in text[start:text.index("}", start)]


def test_loop_drives_the_fan():
    assert "updateHandFan()" in _function_body("loop")


def test_setup_owns_the_pin_before_anything_that_can_block():
    """The inclinometer wait in setup() never returns if the SCL3300 is absent."""
    body = _function_body("setup")
    pin = body.find("pinMode(HAND_FAN_PIN, OUTPUT)")
    wait = body.find("inclinometer.begin()")
    assert 0 <= pin < wait
