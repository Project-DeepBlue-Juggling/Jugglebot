"""Firmware-version handshake tests for teensy_bridge_node (Phase 3).

Phase 3 restores can_node's Get_Version handshake on the can-bridge: the firmware
sweeps Get_Version + caches the raw versions; the bridge pulls them via the
GET_AXIS_VERSIONS RPC (a bridge-LOCAL cache read — no CAN3 round-trip) and runs
the EXISTING tested ``MotorStateTracker.validate_group`` against
``EXPECTED_HW_VERSIONS``, latching:

* ``firmware_validated=True`` on a clean match → un-gates the orchestrator's
  ``is_homed`` skip (state_machine.py:228-235) so BOOT proceeds;
* a mismatch string → kept ``firmware_validated=False`` AND surfaced on
  ``robot_state.error`` → orchestrator force-FAULT (exact can_node:489-492 / 1085
  parity);
* incomplete / old-firmware / never-answers → ``firmware_validated`` stays False
  (BOOT waits, then FAULTs on BOOT_TIMEOUT_S — can_node's behaviour when versions
  never arrive).

These drive the real ``_version_check_poll`` through a FakeTeensy GET_AXIS_VERSIONS
responder. ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

import time

from controller.teensy_link import RpcMethod, RpcStatus, Telemetry
from controller.teensy_link import rpc_args
from controller.teensy_link import protocol as p

from jugglebot.can.motor_state import EXPECTED_HW_VERSIONS, JUGGLEBOT_AXES

from tests.ros.test_teensy_bridge_node_read import _build_paired_node, _wait_until


def _node():
    teensy, client, node = _build_paired_node(boot_state_read=False)
    return teensy, client, node


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


def _raw_version(hw, fw=(0, 6, 11)):
    """An 8-byte ODrive Get_Version reply: proto, hw(product,ver,variant),
    fw(major,minor,rev), unreleased — the layout odrive.decode_get_version reads."""
    return bytes([0, hw[0], hw[1], hw[2], fw[0], fw[1], fw[2], 0])


def _wire_versions(teensy, per_axis_hw, fw_per_axis=None):
    """Register a GET_AXIS_VERSIONS responder returning the given per-axis hw
    versions (fw consistent unless overridden)."""
    fw_per_axis = fw_per_axis or {}
    raw = {aid: _raw_version(hw, fw_per_axis.get(aid, (0, 6, 11)))
           for aid, hw in per_axis_hw.items()}
    blob = rpc_args.encode_axis_versions_result(raw)

    def handler(req_id, args):
        return (int(RpcStatus.OK), blob)
    teensy.on_rpc(int(RpcMethod.GET_AXIS_VERSIONS), handler)


def _poll_until_resolved(node, timeout=2.0):
    """Drive _version_check_poll until it latches (validated or mismatch), or
    timeout. Robust to the cold-process first-RPC socket warmup."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        node._version_check_poll()
        with node._lock:
            if node._firmware_validated or node._firmware_mismatch_error:
                return
        time.sleep(0.02)


def _publish(node, teensy):
    pos = tuple(float(i) for i in range(p.NUM_AXES))
    teensy.send_telemetry(pos_rev=pos, vel_rps=tuple(0.0 for _ in range(p.NUM_AXES)))
    assert _wait_until(lambda: node._latest_telemetry is not None)
    node._publish_robot_state()
    return node.robot_state_pub.published[-1]


# ── Match → firmware_validated=True ────────────────────────────

def test_matching_versions_validate_firmware():
    """All Jugglebot axes report the EXPECTED_HW_VERSIONS tuple → firmware_validated
    latches True and surfaces on robot_state (un-gating the orchestrator BOOT)."""
    teensy, client, node = _node()
    try:
        per_axis = {aid: EXPECTED_HW_VERSIONS[aid] for aid in JUGGLEBOT_AXES}
        _wire_versions(teensy, per_axis)
        _poll_until_resolved(node)
        assert node._firmware_validated is True
        assert node._firmware_mismatch_error is None

        msg = _publish(node, teensy)
        assert msg.firmware_validated is True
        assert msg.has_fatal_odrive_error is False
        assert msg.error == [] or "firmware" not in " ".join(msg.error).lower()
    finally:
        _teardown(teensy, client, node)


def test_validation_is_idempotent_once_resolved():
    """Once validated, _version_check_poll is a no-op — it does NOT re-pull (the
    cheap steady state). Dropping the responder afterwards must not flip the latch."""
    teensy, client, node = _node()
    try:
        _wire_versions(teensy, {aid: EXPECTED_HW_VERSIONS[aid] for aid in JUGGLEBOT_AXES})
        _poll_until_resolved(node)
        assert node._firmware_validated is True
        # Make any further pull fail; the latch must hold.
        teensy.on_rpc(int(RpcMethod.GET_AXIS_VERSIONS),
                      lambda rid, a: (int(RpcStatus.ERR_BUS_DOWN), b""))
        node._version_check_poll()
        assert node._firmware_validated is True
    finally:
        _teardown(teensy, client, node)


# ── Mismatch → fatal + error string, firmware_validated stays False ───────────

def test_version_mismatch_forces_fault():
    """A wrong hw version on one axis → firmware_validated stays False, the mismatch
    string is latched and surfaced on robot_state.error (force-FAULT), and
    has_fatal_odrive_error is set (exact can_node:489-492 / 1085 parity)."""
    teensy, client, node = _node()
    try:
        per_axis = {aid: EXPECTED_HW_VERSIONS[aid] for aid in JUGGLEBOT_AXES}
        bad = JUGGLEBOT_AXES[0]
        exp = EXPECTED_HW_VERSIONS[bad]
        per_axis[bad] = (exp[0], exp[1], exp[2] + 1)   # wrong hw revision
        _wire_versions(teensy, per_axis)
        _poll_until_resolved(node)

        assert node._firmware_validated is False
        assert node._firmware_mismatch_error is not None
        assert f"Axis {bad} hw mismatch" in node._firmware_mismatch_error

        msg = _publish(node, teensy)
        assert msg.firmware_validated is False
        assert msg.has_fatal_odrive_error is True
        assert any("hw mismatch" in e for e in msg.error)   # force-FAULT via ctx.errors
    finally:
        _teardown(teensy, client, node)


# ── Cannot-validate (incomplete / old firmware) → stays False, no crash ───────

def test_incomplete_versions_stay_unvalidated():
    """Only some Jugglebot axes reported → all_jugglebot_versions_received is False,
    so the bridge keeps trying (firmware_validated stays False, no mismatch latched
    — the BOOT_TIMEOUT governs the give-up)."""
    teensy, client, node = _node()
    try:
        partial = {aid: EXPECTED_HW_VERSIONS[aid] for aid in JUGGLEBOT_AXES[:3]}
        _wire_versions(teensy, partial)
        # Poll a few times; it must never falsely validate or latch a mismatch.
        for _ in range(5):
            node._version_check_poll()
        assert node._firmware_validated is False
        assert node._firmware_mismatch_error is None
        assert _publish(node, teensy).firmware_validated is False
    finally:
        _teardown(teensy, client, node)


def test_old_firmware_unknown_method_does_not_crash():
    """An old firmware that does not implement GET_AXIS_VERSIONS answers
    ERR_UNKNOWN_METHOD → the poll cannot validate (stays False), never crashes."""
    teensy, client, node = _node()
    try:
        teensy.on_rpc(int(RpcMethod.GET_AXIS_VERSIONS),
                      lambda rid, a: (int(RpcStatus.ERR_UNKNOWN_METHOD), b""))
        for _ in range(3):
            node._version_check_poll()
        assert node._firmware_validated is False
        assert node._firmware_mismatch_error is None
        assert _publish(node, teensy).firmware_validated is False
    finally:
        _teardown(teensy, client, node)


def test_no_responder_default_false():
    """With no GET_AXIS_VERSIONS responder (the FakeTeensy answers
    ERR_UNKNOWN_METHOD), firmware_validated is the conservative default False — the
    rewritten 'conservative False' contract (it is no longer hardcoded; it is the
    not-yet-validated state)."""
    teensy, client, node = _node()
    try:
        node._version_check_poll()
        assert node._firmware_validated is False
        assert _publish(node, teensy).firmware_validated is False
    finally:
        _teardown(teensy, client, node)
