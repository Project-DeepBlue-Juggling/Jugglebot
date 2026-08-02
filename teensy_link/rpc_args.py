"""RPC method argument encoders for the can-bridge link.

Thin, typed wrappers over the codegen-emitted argument dataclasses (hoisted into
``config/generate_udp_protocol.py``). The
packed structs in the generated ``udp_protocol`` module are the single source of
truth; this module gives the bridge (and any future Jetson-side client) a clean
per-method encode API plus the ``AXIS_ALL`` broadcast sentinel, so the encoding
lives in one tested place.

Each ``encode_*`` returns the packed args ``bytes`` ready for
``RpcClient.call(method, args)``. The firmware ``rpc.h`` consumes the *same*
generated structs (via ``using JbUdp::RpcArgs::...``); byte layouts are validated
against the firmware in ``tests/teensy_link/test_rpc_args.py``.
"""

from __future__ import annotations

import struct

from . import protocol as p
from .protocol import (
    AXIS_ALL,
    ArgAxisState,
    ArgControllerMode,
    ArgVelCurr,
    ArgPosGain,
    ArgVelGains,
    ArgAbsPosition,
    ArgAxisOnly,
    ArgSdoRead,
    ArgSdoWrite,
    ResultTimeOfDay,
    ArgBbThrow,
    ArgRobotState,
    ArgHandTraj,
    ResultAxisVersions,
)

RpcMethod = p.RpcMethod

__all__ = [
    "AXIS_ALL",
    # dataclasses (re-exported)
    "ArgAxisState", "ArgControllerMode", "ArgVelCurr", "ArgPosGain",
    "ArgVelGains", "ArgAbsPosition", "ArgAxisOnly", "ArgSdoRead", "ArgSdoWrite",
    "ResultTimeOfDay", "ArgBbThrow", "ArgRobotState", "ArgHandTraj",
    # encoders
    "encode_set_axis_state", "encode_set_controller_mode",
    "encode_set_vel_curr_limits", "encode_set_pos_gain", "encode_set_vel_gains",
    "encode_set_absolute_position", "encode_clear_errors", "encode_reboot",
    "encode_encoder_search", "encode_home", "encode_activate", "encode_deactivate",
    "encode_sdo_read", "encode_sdo_write", "encode_state_write",
    "encode_hand_traj_cmd", "encode_smooth_move_hand", "decode_hand_cmd_echo",
    "encode_bb_throw", "encode_bb_reload", "encode_bb_reset",
    "encode_bb_calibrate_loc",
    "decode_time_of_day_result", "ResultAxisVersions",
    "encode_axis_versions_result", "decode_axis_versions_result",
    # method association
    "METHOD",
]


def encode_set_axis_state(axis: int, state: int) -> bytes:
    """SET_AXIS_STATE: set an ODrive axis's requested state (e.g. CLOSED_LOOP)."""
    return ArgAxisState(axis=int(axis), state=int(state)).pack()


def encode_set_controller_mode(axis: int, ctrl: int, input_mode: int) -> bytes:
    """SET_CONTROLLER_MODE: set control_mode + input_mode."""
    return ArgControllerMode(axis=int(axis), ctrl=int(ctrl),
                             input=int(input_mode)).pack()


def encode_set_vel_curr_limits(axis: int, vel_limit: float,
                               curr_limit: float) -> bytes:
    """SET_VEL_CURR_LIMITS: velocity (rev/s) + current (A) limits."""
    return ArgVelCurr(axis=int(axis), vel_limit=float(vel_limit),
                      curr_limit=float(curr_limit)).pack()


def encode_set_pos_gain(axis: int, pos_gain: float) -> bytes:
    """SET_POS_GAIN: position gain."""
    return ArgPosGain(axis=int(axis), pos_gain=float(pos_gain)).pack()


def encode_set_vel_gains(axis: int, vel_gain: float, vel_int_gain: float) -> bytes:
    """SET_VEL_GAINS: velocity gain + velocity integrator gain."""
    return ArgVelGains(axis=int(axis), vel_gain=float(vel_gain),
                       vel_int_gain=float(vel_int_gain)).pack()


def encode_set_absolute_position(axis: int, position: float) -> bytes:
    """SET_ABSOLUTE_POSITION: absolute position (rev), post-homing."""
    return ArgAbsPosition(axis=int(axis), position=float(position)).pack()


def encode_clear_errors(axis: int = AXIS_ALL) -> bytes:
    """CLEAR_ERRORS: clear errors on one axis or all (AXIS_ALL)."""
    return ArgAxisOnly(axis=int(axis)).pack()


def encode_reboot(axis: int = AXIS_ALL) -> bytes:
    """REBOOT_ODRIVES: reboot one axis or all (AXIS_ALL)."""
    return ArgAxisOnly(axis=int(axis)).pack()


def encode_encoder_search(axis: int = AXIS_ALL) -> bytes:
    """ENCODER_SEARCH (firmware stub — returns ERR_NOT_IMPL today)."""
    return ArgAxisOnly(axis=int(axis)).pack()


def encode_home(axis: int = AXIS_ALL) -> bytes:
    """HOME (firmware stub — returns ERR_NOT_IMPL today)."""
    return ArgAxisOnly(axis=int(axis)).pack()


def encode_activate(axis: int = AXIS_ALL) -> bytes:
    """ACTIVATE: TRAP_TRAJ move to the active pose.

    ``AXIS_ALL`` activates every present leg in parallel (even platform rise); a
    single leg index activates just that leg iff present.
    """
    return ArgAxisOnly(axis=int(axis)).pack()


def encode_deactivate(axis: int = AXIS_ALL) -> bytes:
    """DEACTIVATE: TRAP_TRAJ controlled lower to the STOW pose, then IDLE.

    ``AXIS_ALL`` deactivates every present leg in parallel (even platform descent);
    a single leg index deactivates just that leg iff present.
    """
    return ArgAxisOnly(axis=int(axis)).pack()


def encode_sdo_read(axis: int, endpoint: int) -> bytes:
    """SDO_READ: arbitrary parameter read (response returns async on TxSdo)."""
    return ArgSdoRead(axis=int(axis), endpoint=int(endpoint)).pack()


def encode_sdo_write(axis: int, endpoint: int, value: float) -> bytes:
    """SDO_WRITE: arbitrary parameter write."""
    return ArgSdoWrite(axis=int(axis), endpoint=int(endpoint),
                       value=float(value)).pack()


def decode_time_of_day_result(blob: bytes) -> int:
    """Decode a TIME_OF_DAY_QUERY result blob → Jetson wall-clock µs."""
    return int(ResultTimeOfDay.unpack(blob).jetson_wall_us)


# ── Hand trajectory / smooth-move ─────────────────────────────────────────────
# set_hand_traj_cmd + smooth_move_hand both ride the ONE HAND_TRAJ_CMD RPC,
# discriminated by byte 0 of an 8-byte payload that is BYTE-IDENTICAL to can_node's
# 0x6D0 PLATFORM_TRAJ_CMD frame (can_node.py:1626-1661). The payload IS the
# ArgHandTraj struct (u8 payload[8]); the firmware attaches the firmware-owned 0x6D0
# id and forwards it after the CLOSED_LOOP + POSITION/PASSTHROUGH preamble. These
# encoders are PURE (no time.time()) so the byte layout is deterministically testable
# (tests/firmware/test_hand_traj_xref.py); the CALLER computes the ABSOLUTE
# wall_time_ms (now_ms + event_delay*1000, exactly as can_node did) and passes it in —
# the absolute deadline is what makes the hand timing immune to bridge transit jitter,
# and the firmware never re-stamps it.

def encode_hand_traj_cmd(traj_type: int, event_vel: float, wall_time_ms: int) -> bytes:
    """HAND_TRAJ_CMD (catch trajectory). Builds can_node._send_hand_traj_cmd's exact
    0x6D0 payload: byte 0 = traj_type (0/1/2), bytes 1-2 = round(event_vel*100) as a
    u16 (LE), bytes 3-6 = (wall_time_ms & 0xFFFFFFFF) as a u32 (LE), byte 7 = 0.
    ``wall_time_ms`` is the ABSOLUTE Jetson wall-clock deadline (ms) — the caller adds
    event_delay to time.time() (the firmware does NOT re-stamp)."""
    vel_u16 = int(round(event_vel * 100))
    payload = bytes([int(traj_type) & 0xFF, vel_u16 & 0xFF, (vel_u16 >> 8) & 0xFF])
    payload += struct.pack('<I', int(wall_time_ms) & 0xFFFFFFFF) + bytes([0])
    return ArgHandTraj(payload=tuple(payload)).pack()


def encode_smooth_move_hand(target_rev: float) -> bytes:
    """HAND_TRAJ_CMD (smooth-move). Builds can_node._smooth_move_hand's exact 0x6D0
    payload: byte 0 = 3 (discriminator), bytes 1-4 = target_rev as f32 (LE),
    bytes 5-7 = 0."""
    payload = bytes([3]) + struct.pack('<f', float(target_rev)) + bytes(3)
    return ArgHandTraj(payload=tuple(payload)).pack()


def decode_hand_cmd_echo(data: bytes, vel_scale: float, tor_scale: float):
    """Decode a HAND_CMD_ECHO frame's 8-byte ``data`` (the sniffed ODrive
    Set_Input_Pos payload) → (pos_rev, vel_ff, tor_ff), byte-identical to
    can_node._handle_hand_input_pos: unpack ``<f h h>`` then divide vel/tor by
    INPUT_SCALE_HAND_VEL / INPUT_SCALE_HAND_TOR. The caller passes the scales (from
    protocol_config) so this module stays free of the ROS/codegen import graph."""
    pos, vel_ff, tor_ff = struct.unpack('<fhh', bytes(data[:8]))
    return float(pos), vel_ff / vel_scale, tor_ff / tor_scale


# ── Firmware version pull ─────────────────────────────────────────────────────
# GET_AXIS_VERSIONS takes NO args (the request is empty) and returns the
# ResultAxisVersions blob SYNCHRONOUSLY in the RPC response (a bridge-local cache
# read — no CAN3 round-trip). The firmware fills it with the raw 8-byte ODrive
# Get_Version payload per Jugglebot axis (axis-major) + a received bitmask. The
# bridge decodes the set-bit axes via jugglebot.can.odrive.decode_get_version and
# runs MotorStateTracker.validate_group — version semantics live in tested Python.

_VERSION_BYTES_PER_AXIS = 8
# Axis count = the raw-array length / 8 (NUM_AXES = 7 today), read from the struct
# so it tracks the codegen rather than a hardcoded literal.
_NUM_VERSION_AXES = len(ResultAxisVersions().raw) // _VERSION_BYTES_PER_AXIS


def encode_axis_versions_result(per_axis_raw: dict) -> bytes:
    """Pack a GET_AXIS_VERSIONS result blob from {axis: raw8} — the firmware-side
    mirror (used by the FakeTeensy responder + the round-trip test). Axes absent
    from the dict are zero-filled and their received bit stays clear."""
    n_axes = _NUM_VERSION_AXES
    mask = 0
    raw = bytearray(n_axes * _VERSION_BYTES_PER_AXIS)
    for axis, payload in per_axis_raw.items():
        if not (0 <= int(axis) < n_axes):
            raise ValueError(f"axis {axis} out of range [0,{n_axes})")
        b = bytes(payload)
        if len(b) != _VERSION_BYTES_PER_AXIS:
            raise ValueError(f"axis {axis} version payload must be 8 bytes, got {len(b)}")
        off = int(axis) * _VERSION_BYTES_PER_AXIS
        raw[off:off + _VERSION_BYTES_PER_AXIS] = b
        mask |= (1 << int(axis))
    return ResultAxisVersions(received_mask=mask, raw=tuple(raw)).pack()


def decode_axis_versions_result(blob: bytes) -> dict:
    """Decode a GET_AXIS_VERSIONS result blob → {axis: raw8} for every axis whose
    received bit is set. The caller decodes each 8-byte payload via
    jugglebot.can.odrive.decode_get_version (keeping ODrive semantics in one place)."""
    r = ResultAxisVersions.unpack(blob)
    raw = bytes(r.raw)
    out = {}
    n_axes = len(raw) // _VERSION_BYTES_PER_AXIS
    for axis in range(n_axes):
        if r.received_mask & (1 << axis):
            off = axis * _VERSION_BYTES_PER_AXIS
            out[axis] = raw[off:off + _VERSION_BYTES_PER_AXIS]
    return out


# ── Platform-Teensy relay ─────────────────────────────────────────────────────
# TILT_READ / STATE_READ carry NO args — they only trigger a Platform-Teensy
# reply on CAN3 (0x7DE / 0x6E0); the reply arrives async as a PLATFORM_FRAME the
# bridge correlates by (can_id, dlc). STATE_WRITE carries the whole RobotState
# (the bridge is the sole writer and read-modify-writes through its cache so a
# homing write preserves levelling and vice versa); the firmware encodes the
# 0x6E0 frame itself (least-privilege — never a Jetson-supplied raw frame).

def encode_state_write(is_homed: bool, levelling_complete: bool,
                       pose_offset_tiltX: float = 0.0,
                       pose_offset_tiltY: float = 0.0) -> bytes:
    """STATE_WRITE: write the Platform-Teensy RobotState (is_homed / levelling /
    pose offset). The firmware re-encodes the 0x6E0 RobotState CAN frame.

    Note the asymmetry with :func:`decode_platform_fw_version` below: the write
    path carries NO version. Bytes 5-6 of the 0x6E0 frame are the Platform
    Teensy's own identity and are meaningful only Teensy→host; the can-bridge's
    ``state_write`` zero-fills them and the Teensy's ``decodeStateCANMessage``
    never reads them."""
    return ArgRobotState(
        is_homed=1 if is_homed else 0,
        levelling_complete=1 if levelling_complete else 0,
        pose_offset_tiltX=float(pose_offset_tiltX),
        pose_offset_tiltY=float(pose_offset_tiltY),
    ).pack()


# ── Platform-Teensy firmware identity (0x6E0 reply, bytes 5-6) ────────────────
# The Platform Teensy carries `FW_VERSION` in Teensy_code_platform/Teensy_code_platform.ino and
# reports it in the RobotState reply it already sends. THE HOST'S EXPECTED VALUE
# IS DELIBERATELY A SECOND, INDEPENDENTLY-AUTHORED CONSTANT rather than a shared
# generated one: the skew being detected is *board vs tree*, and a single codegen'd
# value would move in the source tree without the board ever being flashed — i.e.
# it would agree with itself in exactly the situation the check exists to catch.
# The two are pinned together by tests/firmware/test_platform_fw_version_xref.py.

#: What a pre-2026-07-27 (un-versioned) Platform Teensy reports. NOT a release
#: number — every firmware built before the identity block existed zero-filled
#: bytes 5-7 of the reply unconditionally, so this value arrives on the wire from
#: a real board rather than standing for its silence.
PLATFORM_FW_VERSION_UNVERSIONED = 0

#: The Platform Teensy firmware version this host tree expects.
#: Keep in lockstep with `FW_VERSION` in Teensy_code_platform/Teensy_code_platform.ino — that file
#: carries the bump history explaining what each version means.
#: 2 (2026-07-28) = the post-release deceleration feedforward, contract C-HAND-2
#: (ros_ws/docs/hand_decel_feedforward.md). A board still on 1 is not unsafe — it
#: simply coasts past the stroke end as it did before — but it invalidates every
#: § CHECK HAND-7 bench row, which is why the check warns loudly.
PLATFORM_FW_VERSION_EXPECTED = 2


def decode_platform_fw_version(data: bytes) -> int:
    """Read the Platform Teensy's ``FW_VERSION`` out of a 0x6E0 RobotState reply.

    Bytes 5-6, uint16 little-endian, exactly as ``Teensy_code_platform.ino``
    ``createStateCANMessage`` packs it.

    A frame too short to carry the field decodes to
    :data:`PLATFORM_FW_VERSION_UNVERSIONED` rather than raising. That default is
    the LOUD direction, not the convenient one: an unversioned answer is reported
    as a skew, whereas raising would take down the caller's read (and with it the
    ``is_homed`` the same frame carries), and returning the expected version would
    manufacture false reassurance from a malformed frame. A real board cannot
    produce a short frame here — the dlc is 8 in every firmware ever flashed — so
    this branch is defence, not a supported path.
    """
    if len(data) < 7:
        return PLATFORM_FW_VERSION_UNVERSIONED
    return int.from_bytes(data[5:7], "little")


# ── Can-bridge firmware identity (BRIDGE_IDENTITY 0x8E, fw_version) ───────────
# The can-bridge Teensy has carried `FW_VERSION` in canbridge_config.h since the
# beginning, but it reached only the USB serial boot banner — so from a Jetson
# session there was no way to tell WHICH firmware answered, and every bench
# result was implicitly attributed to whatever the tree happened to say. The
# 0x8E uplink puts it on the wire; this is the host's side of the comparison.
#
# SAME "two independently-authored constants" reasoning as
# PLATFORM_FW_VERSION_EXPECTED above, and for the same reason: the skew being
# detected is BOARD vs TREE. A value shared with the firmware via codegen would
# move in the source tree without the board ever being flashed — i.e. it would
# agree with itself in exactly the situation the check exists to catch. The two
# are pinned together by tests/firmware/test_bridge_fw_version_xref.py.
#
# There is deliberately NO `UNVERSIONED` sentinel here, unlike the Platform
# Teensy: a bridge too old to know about 0x8E does not send a zero, it sends
# NOTHING, and absence is already the honest signal (rendered as
# 'unknown (never seen)'). Reserving a numeric sentinel would invent a wire
# value no firmware can produce.

#: The can-bridge Teensy firmware version this host tree expects.
#: Keep in lockstep with `FW_VERSION` in
#: ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h — that line
#: carries the bump history explaining what each version means.
#: 9 (2026-08-02) = the ERR_TIMEOUT-attribution instrumentation: the additive
#: BridgeTxDiag (0x8D) and BridgeIdentity (0x8E) uplinks. A board still on 8 is
#: not unsafe — it simply sends neither frame, so both rows read never-seen —
#: but every conclusion drawn from tx_deferred / hand-stage attribution needs a
#: board that actually has the counters, which is why the check is loud.
EXPECTED_BRIDGE_FW_VERSION = 9


# ── Ball Butler ─────────────────────────────────────────────────────────────
# Firmware-owned encoding: the bridge passes typed args; the can-bridge Teensy
# range-checks + frame-builds before TX on CAN1.
# RELOAD/RESET/CALIBRATE_LOC are payloadless on the BB wire; the RPC carries no
# args either (caller sends b"" — matches the NOP shape).

def encode_bb_throw(yaw_rad: float, pitch_rad: float,
                    speed_mps: float, delay_s: float) -> bytes:
    """BB_THROW: typed throw command. Firmware validates ranges and returns
    ERR_BAD_ARGS for malformed throws (yaw outside [-pi, pi), pitch outside
    [0, pi/2], speed > 6.5535, delay > 65.535)."""
    return ArgBbThrow(yaw_rad=float(yaw_rad), pitch_rad=float(pitch_rad),
                      speed_mps=float(speed_mps), delay_s=float(delay_s)).pack()


def encode_bb_reload() -> bytes:
    """BB_RELOAD: payloadless reload command."""
    return b""


def encode_bb_reset() -> bytes:
    """BB_RESET: payloadless reset command."""
    return b""


def encode_bb_calibrate_loc() -> bytes:
    """BB_CALIBRATE_LOC: payloadless calibrate-locations command."""
    return b""


# Method → arg-dataclass association (introspection / tests).
METHOD = {
    RpcMethod.SET_AXIS_STATE: ArgAxisState,
    RpcMethod.SET_CONTROLLER_MODE: ArgControllerMode,
    RpcMethod.SET_VEL_CURR_LIMITS: ArgVelCurr,
    RpcMethod.SET_POS_GAIN: ArgPosGain,
    RpcMethod.SET_VEL_GAINS: ArgVelGains,
    RpcMethod.SET_ABSOLUTE_POSITION: ArgAbsPosition,
    RpcMethod.CLEAR_ERRORS: ArgAxisOnly,
    RpcMethod.REBOOT_ODRIVES: ArgAxisOnly,
    RpcMethod.ENCODER_SEARCH: ArgAxisOnly,
    RpcMethod.HOME: ArgAxisOnly,
    RpcMethod.ACTIVATE: ArgAxisOnly,
    RpcMethod.DEACTIVATE: ArgAxisOnly,   # was missing (axis-only, like ACTIVATE)
    RpcMethod.SDO_READ: ArgSdoRead,
    RpcMethod.SDO_WRITE: ArgSdoWrite,
    RpcMethod.BB_THROW: ArgBbThrow,
    RpcMethod.STATE_WRITE: ArgRobotState,
    RpcMethod.HAND_TRAJ_CMD: ArgHandTraj,   # host builds the 8-byte 0x6D0 payload
    # BB_RELOAD/RESET/CALIBRATE_LOC are payloadless — no entry (matches NOP).
    # TILT_READ/STATE_READ are payloadless too (reply arrives as a PLATFORM_FRAME).
}
