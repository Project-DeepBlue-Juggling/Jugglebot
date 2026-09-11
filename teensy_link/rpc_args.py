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
    ResultAxisVersions,
    ResultBbAxisVersions,
    ArgPlatformFwBegin,
    ArgPlatformFwData,
    ArgPlatformFwVerify,
)

RpcMethod = p.RpcMethod

__all__ = [
    "AXIS_ALL",
    # dataclasses (re-exported)
    "ArgAxisState", "ArgControllerMode", "ArgVelCurr", "ArgPosGain",
    "ArgVelGains", "ArgAbsPosition", "ArgAxisOnly", "ArgSdoRead", "ArgSdoWrite",
    "ResultTimeOfDay", "ArgBbThrow", "ArgRobotState",
    # encoders
    "encode_set_axis_state", "encode_set_controller_mode",
    "encode_set_vel_curr_limits", "encode_set_pos_gain", "encode_set_vel_gains",
    "encode_set_absolute_position", "encode_clear_errors", "encode_reboot",
    "encode_encoder_search", "encode_home", "encode_activate", "encode_deactivate",
    "encode_sdo_read", "encode_sdo_write", "encode_state_write",
    "decode_hand_cmd_echo",
    "encode_bb_throw", "encode_bb_reload", "encode_bb_reset",
    "encode_bb_calibrate_loc",
    "decode_time_of_day_result", "ResultAxisVersions",
    "ResultBbAxisVersions", "decode_bb_axis_versions_result", "BB_FIRST_NODE",
    "encode_axis_versions_result", "decode_axis_versions_result",
    # Platform firmware-over-CAN (2026-09-09)
    "encode_platform_fw_begin", "encode_platform_fw_data",
    "encode_platform_fw_verify", "encode_platform_fw_commit",
    "decode_platform_fw_reply", "platform_fw_rewind_frame",
    "PLATFORM_FW_STATUS_NAMES",
    "PLATFORM_FW_STATUS_OK", "PLATFORM_FW_STATUS_BUSY",
    "PLATFORM_FW_STATUS_BAD_STATE", "PLATFORM_FW_STATUS_BAD_SEQ",
    "PLATFORM_FW_STATUS_TOO_BIG", "PLATFORM_FW_STATUS_BAD_CRC",
    "PLATFORM_FW_STATUS_BAD_IDENTITY", "PLATFORM_FW_STATUS_FLASH_ERR",
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
    """SDO_READ: arbitrary parameter read — FIRE-AND-FORGET. The value never
    reaches the Jetson, so this CANNOT be used to read a register back.

    The ODrive does answer on TxSdo, but nothing correlates that reply to the
    caller: ``rpc.cpp``'s SDO_READ case returns an empty result blob, and
    ``can_buses.cpp``'s TxSdo decode consumes a reply ONLY for the hand's
    ``get_gpio_states`` (the ball sensor), discarding every other. No uplink frame
    carries an arbitrary SDO value. The hand is refused outright — SDO_READ is
    absent from ``hand_axis6_permitted``, so axis 6 gets ERR_REJECTED before
    anything leaves the Teensy.

    Reading a register back needs a firmware change, not a caller change; the
    analysis is in ``odrive-config-drift-assertion.md``.
    """
    return ArgSdoRead(axis=int(axis), endpoint=int(endpoint)).pack()


def encode_sdo_write(axis: int, endpoint: int, value: float) -> bytes:
    """SDO_WRITE: arbitrary parameter write."""
    return ArgSdoWrite(axis=int(axis), endpoint=int(endpoint),
                       value=float(value)).pack()


def decode_time_of_day_result(blob: bytes) -> int:
    """Decode a TIME_OF_DAY_QUERY result blob → Jetson wall-clock µs."""
    return int(ResultTimeOfDay.unpack(blob).jetson_wall_us)


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


#: First Ball Butler CAN node id. The GET_BB_AXIS_VERSIONS mask is BB-RELATIVE
#: (bit i ⇒ the i-th axis of that blob), mirroring ResultAxisVersions bit-for-bit
#: so one decoder shape serves both; this is the base that turns it back into an
#: absolute axis id. Kept as a module constant rather than read from
#: protocol_config so teensy_link stays free of the ROS package's imports.
BB_FIRST_NODE = 7


def decode_bb_axis_versions_result(blob: bytes) -> dict:
    """Decode a GET_BB_AXIS_VERSIONS result blob → {absolute_axis: raw8} for
    every Ball Butler axis whose received bit is set.

    The Ball Butler twin of :func:`decode_axis_versions_result`, and the same
    contract: the caller decodes each 8-byte payload via
    ``jugglebot.can.odrive.decode_get_version`` (ODrive semantics stay in one
    place). The only difference is the mask base — the wire mask counts from
    ``BB_FIRST_NODE``, so bit 0 is axis 7 — which is converted here rather than
    at the call site, so no consumer has to remember the offset.
    """
    r = ResultBbAxisVersions.unpack(blob)
    raw = bytes(r.raw)
    out = {}
    n_axes = len(raw) // _VERSION_BYTES_PER_AXIS
    for i in range(n_axes):
        if r.received_mask & (1 << i):
            off = i * _VERSION_BYTES_PER_AXIS
            out[BB_FIRST_NODE + i] = raw[off:off + _VERSION_BYTES_PER_AXIS]
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
#: 3 (2026-08-18) = the hand END-STOP correction, Geometry::HAND_MOTOR_HARD_STOP_REVS
#: 11.1 -> 10.8 rev (operator-measured metal contact). The commanded profiles are
#: unchanged — SMOOTH_MOVE_POS_CEIL_REV holds at 10.60 rev because the margin moved
#: 0.5 -> 0.2 with the base — but smoothMoveMaxDuration() moves 0.80054 -> 0.78964 s,
#: so a board on 2 still emits preludes up to 0.8005 s and bench row H4.10 scores it
#: as unflashed.
#: 4 (2026-09-08) = FW 18 bundle hand-clip re-measurement, a host-side ripple of the
#: can-bridge's FW 18 (see ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h).
#: Geometry::HAND_MOTOR_HARD_STOP_REVS 10.8 -> 10.701 rev, this time with NO
#: compensating margin widening, so SMOOTH_MOVE_POS_CEIL_REV moves too: 10.60 ->
#: 10.501 rev, and smoothMoveMaxDuration() 0.78964 -> 0.78602 s. A board on 3 emits
#: preludes past the new ceiling.
#: 5 (2026-09-09) = FIRMWARE UPDATE OVER CAN: the board's USB port is damaged, so
#: every image after this one has to arrive over the 0x6F0/0x6F1 seam (see the
#: PLATFORM_FW_* encoders below) instead of a USB reflash. No behavioural change
#: to the flight code — a board on 4 flies identically — it simply has no
#: 0x6F0 listener yet, so it cannot be handed its own successor and the one
#: flash to 5 must happen over USB.
#: 6 (2026-09-09) = the first image built on the Jetson (`pio run -e teensy40
#: -t upload`) and flashed over CAN; no code change beyond the number, which is
#: the receipt: the STATE_READ version going 5 -> 6 is the only visible proof the
#: copy landed now that the boot banner cannot be read.
#: 7 (2026-09-11) = skill-stack R1: the stroke engine (Trajectory.h), the 0x6D0
#: decode and the 0x0C9 hand-encoder cache are deleted. The Platform never
#: transmits to node 6 (the hand ODrive) again — the bridge's 500 Hz interp is
#: the one hand master. Inclinometer, time-sync slave and 0x6E0 cold-start
#: state are unchanged; a board on 6 still answers STATE_READ/TILT_READ
#: identically, so this skew is advisory only, not a dark link.
PLATFORM_FW_VERSION_EXPECTED = 7


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


# ── Platform firmware-over-CAN (2026-09-09) ───────────────────────────────────
# The Platform Teensy's USB port is damaged, so every image after FW 5 arrives
# over CAN through the relay seam: PLATFORM_FW_BEGIN/DATA/VERIFY/COMMIT ride
# 0x6F0 (host -> board), and the board answers every op on 0x6F1 (dlc 8), which
# the bridge forwards verbatim as a PLATFORM_FRAME — the SAME (can_id, dlc)
# correlation TILT_READ/STATE_READ already use (see PlatformFrameWaiter in
# teensy_link/rpc.py). Wire contract (little-endian throughout) is the single
# document in Teensy_code_platform.ino's "FIRMWARE UPDATE OVER CAN" header
# comment; this module mirrors it, never re-derives it.

def encode_platform_fw_begin(image_len: int) -> bytes:
    """PLATFORM_FW_BEGIN: declare the image length (bytes) about to be staged."""
    return ArgPlatformFwBegin(image_len=int(image_len)).pack()


def encode_platform_fw_data(seq: int, payload: bytes) -> bytes:
    """PLATFORM_FW_DATA: one image chunk. ``seq`` counts DATA frames from 0 and
    wraps mod 65536 (the receiver tracks the absolute byte offset separately, so
    the wrap is invisible to the staged image). ``payload`` is 1..5 bytes —
    refused HOST-SIDE outside that range (ValueError) so a malformed chunk never
    reaches the wire; the firmware would ERR_BAD_ARGS the same call, but this is
    the cheaper place to catch it. Short payloads are zero-padded to the fixed
    5-byte wire slot; ``n`` (not the padding) tells the firmware how many of
    those bytes are real."""
    n = len(payload)
    if not (1 <= n <= 5):
        raise ValueError(f"platform fw data payload must be 1..5 bytes, got {n}")
    padded = bytes(payload) + bytes(5 - n)
    return ArgPlatformFwData(seq=int(seq) & 0xFFFF, n=n, payload=tuple(padded)).pack()


def encode_platform_fw_verify(crc32: int) -> bytes:
    """PLATFORM_FW_VERIFY: CRC-32 (zlib/``binascii.crc32`` flavour) over the
    whole staged image, computed host-side over the same bytes it sent."""
    return ArgPlatformFwVerify(crc32=int(crc32) & 0xFFFFFFFF).pack()


def encode_platform_fw_commit() -> bytes:
    """PLATFORM_FW_COMMIT: copy the verified staged image over program flash and
    reboot. Payloadless — matches the BB_RELOAD/RESET/CALIBRATE_LOC NOP shape."""
    return b""


#: Status codes, byte 1 of the 0x6F1 reply — mirrors
#: Teensy_code_platform.ino FwUpdate::ST_* VERBATIM (never renumber, only
#: append) and Teensy_code_canbridge/rpc.cpp's RpcStatus is a SEPARATE code
#: space: these travel inside the RPC result blob, once the RPC itself has
#: already returned OK.
PLATFORM_FW_STATUS_OK = 0
PLATFORM_FW_STATUS_BUSY = 1            # trajectory engine not idle (BEGIN, COMMIT)
PLATFORM_FW_STATUS_BAD_STATE = 2       # no session open, wrong phase, or malformed dlc
PLATFORM_FW_STATUS_BAD_SEQ = 3         # out-of-order DATA; seq field = the expected seq
PLATFORM_FW_STATUS_TOO_BIG = 4         # image_len > staging capacity, or DATA past image_len
PLATFORM_FW_STATUS_BAD_CRC = 5
PLATFORM_FW_STATUS_BAD_IDENTITY = 6    # staged image is not a jugglebot-platform build
PLATFORM_FW_STATUS_FLASH_ERR = 7       # staging write failed its read-back

PLATFORM_FW_STATUS_NAMES = {
    PLATFORM_FW_STATUS_OK: "OK",
    PLATFORM_FW_STATUS_BUSY: "BUSY",
    PLATFORM_FW_STATUS_BAD_STATE: "BAD_STATE",
    PLATFORM_FW_STATUS_BAD_SEQ: "BAD_SEQ",
    PLATFORM_FW_STATUS_TOO_BIG: "TOO_BIG",
    PLATFORM_FW_STATUS_BAD_CRC: "BAD_CRC",
    PLATFORM_FW_STATUS_BAD_IDENTITY: "BAD_IDENTITY",
    PLATFORM_FW_STATUS_FLASH_ERR: "FLASH_ERR",
}


def decode_platform_fw_reply(data: bytes):
    """Decode a FW_UPDATE_REPLY (0x6F1, dlc 8) frame:
    ``[opcode][status][seq u16 LE][detail u32 LE]``. Returns
    ``(opcode, status, seq, detail)`` as raw ints — the caller maps ``status``
    via :data:`PLATFORM_FW_STATUS_NAMES`. ``seq`` is the last accepted DATA seq
    on OK, or the EXPECTED seq on BAD_SEQ; ``detail`` is the staged byte count
    (BEGIN/DATA), the computed CRC (VERIFY), or 0."""
    if len(data) < 8:
        raise ValueError(f"platform fw reply frame too short ({len(data)} bytes, need 8)")
    opcode, status, seq, detail = struct.unpack('<BBHI', bytes(data[:8]))
    return opcode, status, seq, detail


def platform_fw_rewind_frame(current_frame_idx: int, expected_seq: int) -> int:
    """On a BAD_SEQ reply, return the absolute DATA frame index to resume
    sending from. ``current_frame_idx`` is the host's own unwrapped frame
    counter (never wraps — only the wire ``seq`` does, mod 65536); the board's
    ``expected_seq`` is mod-65536, so recovering the absolute index means
    finding the frame at or before ``current_frame_idx`` whose wire seq matches.
    A NAK only ever looks back one or two windows (16-32 frames, per the
    ACK-every-16th cadence), so the true rewind distance is always tiny —
    assuming it is under 32768 frames (half the wrap period) resolves the
    ambiguity correctly with enormous margin, including across a seq wrap."""
    delta = (int(current_frame_idx) - int(expected_seq)) & 0xFFFF
    return int(current_frame_idx) - delta


PLATFORM_FW_ACK_CADENCE = 16
"""The Platform receiver ACKs a DATA frame only when ``seq % 16 == 15`` (and on
the frame that completes the image); every other accepted frame is silent."""


def platform_fw_window_end(window_start_frame: int, total_frames: int) -> int:
    """The exclusive end of the DATA window that starts at ``window_start_frame``
    (absolute, unwrapped): the next ACK point, i.e. the smallest multiple of 16
    above the start, capped at the image's last frame — so every window ends on a
    frame the board will answer.

    2026-09-09, first flash attempt: after a genuine BAD_SEQ rewind (a sector
    flush overran the board's RX queue) the host resumed with full 16-frame
    windows from the arbitrary seq the board named, so the window's last frame
    was no longer ``seq % 16 == 15``, the one ACK inside the window was thrown
    away, the host waited a full second for an ACK the board would never send,
    re-sent, took the retry's NAK as the rewind, and repeated that once per
    window until a retry's NAK was lost as well. A partial first window up to
    the next ACK point is the whole fix; 65536 is a multiple of 16, so the
    cadence is continuous across the seq wrap."""
    start = int(window_start_frame)
    aligned_end = (start // PLATFORM_FW_ACK_CADENCE + 1) * PLATFORM_FW_ACK_CADENCE
    return min(int(total_frames), aligned_end)


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
#: 10 (2026-08-09) = the ERR_TIMEOUT FIX: 16 TX mailboxes on the Jugglebot bus
#: (setMaxMB 16→24) + the console-only [handphase] diagnostic. This bump is
#: WIRE-INVISIBLE — no MsgType, no payload, PROTOCOL_VERSION still 5 — so a board
#: still on 9 decodes identically and stays perfectly usable; what it does NOT
#: have is the fix, so it keeps failing ~37 % of hand dispatches under the 500 Hz
#: leg stream. That is exactly why the skew must be loud: the symptom of running
#: FW 9 here is a hand that intermittently does not stroke, not a dark link.
#: 11 (2026-08-11) = the bridge-temporal-trustworthiness P1 instrumentation: the
#: additive ClockDiag (0x8F) uplink — per-anchor clock-discipline sample plus the
#: 500 Hz interp occupancy census. Additive, so a board on 10 decodes every other
#: frame identically and stays fully usable; it simply sends no 0x8F, and
#: /clock_diag records EMPTY rather than erroring.
#: EXPECTED IS BUMPED WHILE THE BOARD IS STILL ON 10, ON PURPOSE. FW 11 is
#: committed unflashed until after the S1 aged-bridge experiment (a flash is a
#: reboot, and the aged state IS the experiment), so a loud
#: `10 (SKEW — expected v11)` row on /link_status for the whole pre-flash window
#: is the CORRECT report: the tree has FW 11, the board does not. Reporting
#: agreement in that window would be exactly the failure mode the
#: 0x8E frame was added to prevent — inferring a flash instead of confirming it.
#: 12 (2026-08-12) = the CacheDiag (0x91) encoder-cache freshness census: the
#: confirmation instrument for the one question the S1 aged-bridge experiment
#: left open (is the encoder cache the lead clamp measures against going STALE
#: with uptime, or does the leg genuinely trail?). Additive again, so an FW 11
#: board decodes every other frame identically and stays fully usable; it simply
#: sends no 0x91, and /cache_diag records EMPTY rather than erroring. The same
#: bumped-while-the-board-is-behind situation as 11 above applies until the
#: operator flashes: /link_status will read `11 (SKEW — expected v12)`, which is
#: the CORRECT report — the tree has FW 12, the board does not — and it is
#: advisory everywhere, never enforced.
#: 13 (2026-08-14) = the RingDiag (0x92) CAN RX-ring true-occupancy census: the
#: conviction instrument for the FlexCAN_T4 `_available` leak, which is the
#: surviving candidate mechanism after S2 killed the cache-AGE hypothesis. The
#: library's ``events()`` pops the RX ring before its ``NVIC_DISABLE_IRQ`` guard,
#: so the ISR's ``_available++`` races the task-side ``_available--``
#: one-directionally, the count under-reports, and the bridge's drain loop leaves
#: a residue that makes every delivery N frames old. ``getRXQueueCount()`` returns
#: that same corrupted count, so every counter the bridge already had is blind to
#: it — hence a new frame rather than a new field. Additive again, so an FW 12
#: board decodes every other frame identically and stays fully usable; it simply
#: sends no 0x92, and ``/ring_diag`` records EMPTY rather than erroring. The same
#: bumped-while-the-board-is-behind situation as 11 and 12 above applies until the
#: operator flashes: ``/link_status`` will read `12 (SKEW — expected v13)`, which
#: is the CORRECT report — the tree has FW 13, the board does not — and it is
#: advisory everywhere, never enforced.
#: 14 (2026-08-14) = THE FIX for that leak, now convicted on hardware rather than
#: suspected: FW 13's ``/ring_diag`` on a 4.03 h board read ``leak_jb`` = 247–248
#: (``true_depth`` 247–248 against ``avail_reported`` 0, hwm 249 ≈ 97 % of one
#: 256-slot lap) on the 500 Hz-loaded jugglebot bus, against 1 on bb and 0 on
#: cone — the arrival × pop ordering the mechanism predicts — with end-to-end leg
#: lag 19.9 ms fresh vs 252.2 ms at 3.80 h. Two vendored-library patches in
#: ``lib/FlexCAN_T4/FlexCAN_T4.tpp``'s ``events()``: the RX pop now runs inside the
#: bus's ``NVIC_DISABLE_IRQ`` mask (with ``dsb; isb``), so the ISR's
#: ``_available++`` can no longer be swallowed; and the dormant ``mb == -1``
#: TX-deferral refill loop gets its missing ``break`` (unreachable at
#: ``tx_deferred == 0``, so it cannot change live behaviour).
#: WIRE-INVISIBLE, like 9→10 — no MsgType, no payload, PROTOCOL_VERSION still 5 —
#: so a board on 13 decodes identically and keeps sending 0x92; what it does NOT
#: have is the fix, so its RX ring keeps ratcheting and its leg lag keeps growing
#: with uptime. RingDiag is retained UNCHANGED on purpose: it is the fix's own
#: proof, and post-fix acceptance is ``leak`` ≡ 0 on all three buses at any
#: uptime. Same bumped-while-the-board-is-behind situation until the operator
#: flashes: ``/link_status`` will read `13 (SKEW — expected v14)`, which is the
#: CORRECT report, and it is advisory everywhere, never enforced.
#: 15 (2026-08-18) = the hand END-STOP correction, and NOTHING ELSE:
#: ``hand_motor_hard_stop_revs`` 11.1 → 10.8 rev (the operator-measured metal
#: contact), which the bridge consumes as ``HAND_MOTOR_MAX_POSITION`` in
#: ``clip_position``, so an FW 14 board passes commanded setpoints up to 0.3 rev
#: (9.5 mm) PAST metal and an FW 15 board does not.  WIRE-INVISIBLE again —
#: no MsgType, no payload, PROTOCOL_VERSION still 5.  **FLASHED ~2026-08-20**:
#: the board has self-reported ``bridge_fw_version`` 15 on BRIDGE_IDENTITY ever
#: since, which is what makes 16 below a bump rather than a fold.
#: 16 (2026-08-24) = the hand ball-sensor POLLER CADENCE fix (consume-and-send in
#: one tick + an absolute schedule with a half-tick early-fire band, so the
#: poller finally reaches its configured 50 Hz instead of 20/30 ms bimodal) and
#: TRI-STATE TX ACCOUNTING (``TxResult{FAILED, MAILBOX, DEFERRED}`` — FlexCAN_T4's
#: ``write()`` returns −1 for QUEUED-and-will-transmit, which every caller had
#: been reading as failure: the mechanism behind the 2026-08-09 lying-ack).
#: WIRE-INVISIBLE once more, so an FW 15 board decodes identically; what it does
#: NOT have is a poller at cadence or a truthful TX verdict.
#: THIS ONE WAS BRIEFLY FOLDED INTO 15 (commit 2995855) ON A FALSE PREMISE —
#: that 15 was still unflashed.  It was not; the end-stop image went aboard
#: ~2026-08-20 and the board has been reporting 15 since, so a fold would have
#: made ``bridge_fw_version`` unable to tell the two images apart, which is the
#: single job this constant has.  Owner's re-decision 2026-08-24: bump to 16.
#: Same bumped-while-the-board-is-behind situation as 11–14, and here it is the
#: WHOLE POINT: until the operator flashes, ``/link_status`` reads
#: `15 (SKEW — expected v16)`, which is the CORRECT report — the tree has FW 16,
#: the bench has FW 15 — and it is advisory everywhere, never enforced (a
#: BRIDGE_FW_CHECK log line and a ``link_status`` row; no gate, no refusal —
#: pinned by ``tests/firmware/test_bridge_fw_version_xref.py``).
#: 17 (2026-09-02) = the unified-7dof HAND LANE (plan Phase 3) — and the first
#: INCOMPATIBLE wire bump since UDP protocol 4→5: FW 17 decodes the v6 Setpoint
#: (208 B, seven lanes + the exact-v1 array; PROTOCOL_VERSION 5→6), so against
#: an FW ≤ 16 board this host tree is in TOTAL LINK DARKNESS in both directions
#: until the lockstep flash sitting — loud and fail-closed by design
#: (decode_frame hard-rejects on version; there is no silent-struct-mismatch
#: failure mode). Content: the 500 Hz interp's 7th (hand) lane behind
#: HAS_HAND && hand_source == STREAMED, the owner-signed hand guard constants
#: (MAX_DEVIATION_HAND_REV 2.5 observe-first / MAX_LEAD_HAND_REV 2.0
#: freshness-aware + the lead-duty counter / HAND_VELFF_LIMIT_RPS 300 / hand
#: overspeed 345), the additive HAND_SOURCE_SET RPC + ERR_HAND_SOURCE status,
#: and the HeartbeatT2J HAND_SOURCE_STREAMED flag bit. Until the flash the live
#: board reporting ≤ 16 raises the BRIDGE_FW_CHECK skew advisory on every
#: launch — advisory only, never enforced, and CORRECT (the tree has FW 17, the
#: board does not); unlike the wire-invisible bumps 9→16 the skew here is ALSO
#: a dark link, so a v6 host against an old board shows no telemetry at all —
#: roll back with the pre-v6 host checkout, or flash FW 17, never half.
#:
#: 18 (2026-09-06) = the FW 18 bundle: the hand setpoint clip stands off the
#: metal (HAND_MOTOR_MAX_POSITION = hand_motor_hard_stop_revs 10.701 −
#: hand_clip_margin_rev 0.2 = 10.501 rev, where FW 17 clipped AT a stop that
#: itself read 0.099 rev high); homing restores axis 6 to POSITION/PASSTHROUGH
#: on the shipped vel/curr limits and every mode-commanding site now records
#: controller_mode/input_mode, which read 0 forever before; the cumulative hand
#: lead / dev_over counters count only ticks that actually transmitted; and
#: `hand7 reset` zeroes them without a Teensy reboot. **NO WIRE CHANGE —
#: PROTOCOL_VERSION STAYS 6**, so unlike 16→17 this skew is NOT a dark link: an
#: FW 17 board and this host tree still talk in both directions, and the only
#: symptom before the flash is the BRIDGE_FW_CHECK advisory. That makes a
#: healthy link no evidence at all that FW 18 is aboard — read
#: link_status/bridge_fw_version or the boot banner.
#: 19 (2026-09-09) = the Platform firmware-over-CAN RELAY: four ADDITIVE
#: RpcMethods (PLATFORM_FW_BEGIN/DATA/VERIFY/COMMIT) that the bridge turns into
#: 0x6F0 frames (byte 0 = opcode) and answers via the existing PLATFORM_FRAME
#: uplink of the board's 0x6F1 reply — see the PLATFORM_FW_* encoders and
#: decode_platform_fw_reply above. NO WIRE CHANGE of any kind — no MsgType, no
#: payload move, PROTOCOL_VERSION stays 6 — so an FW 18 board and an FW 19
#: board are wire-identical and a host that predates these ids is NOT dark
#: against either; what an FW 18 board does NOT have is the four methods, so
#: it answers them with ERR_UNKNOWN_METHOD. Same as every wire-invisible bump
#: before it: a healthy link is not evidence this build is aboard.
#: 20 (2026-09-09) = the BALL BUTLER Get_Version sweep on CAN1, restoring the
#: half of can_node's BOOT firmware check that commit 5875531 dropped: BB moved
#: to teensy_bridge_node over the now-removed USB-CAN and BB ODrive validation
#: was deferred to a "phase B ... by decoding axes 7+8 on CAN1 and surfacing the
#: result via teensy_bridge_node (a new T2J flag or RPC)". The RX decode landed
#: long ago; this is the version half, arriving as exactly the RPC that message
#: anticipated. ONE ADDITIVE RpcMethod (GET_BB_AXIS_VERSIONS) plus a new
#: ResultBbAxisVersions blob — NO wire change to any existing frame, so
#: PROTOCOL_VERSION stays 6 and an FW 19 board is wire-identical. What an FW 19
#: board lacks is the method, so it answers ERR_UNKNOWN_METHOD and both BB axes
#: read never-seen — the honest report, and the reason this bump is safe to land
#: unflashed. Deliberately NOT a widening of ResultAxisVersions: that blob is a
#: fixed NUM_AXES*8 array, so growing it would be an incompatible change needing
#: a lockstep flash to add two display rows.
#: 21 (2026-09-11) = skill-stack R1: the hand mastery latch (hand_source.cpp)
#: and hand_ops.cpp are deleted, so the hand lane is ACTIVE whenever a HAS_HAND
#: Setpoint frame is latched — no HAND_SOURCE_SET step. The hand deviation
#: guard now boots ARMED (was observe-first). HAND_TRAJ_CMD (0x54),
#: HAND_SOURCE_SET (0x55) and ERR_HAND_SOURCE (0x07) are deleted wire ids and
#: HeartbeatT2J flags bit 6 (HAND_SOURCE_STREAMED) is retired — an INCOMPATIBLE
#: wire change, so PROTOCOL_VERSION bumps 6 -> 7 alongside this and a board on
#: 20 is in TOTAL LINK DARKNESS against this host tree until the lockstep
#: flash, loud and fail-closed by design (decode_frame hard-rejects on
#: version).
EXPECTED_BRIDGE_FW_VERSION = 21


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
    # HAND_TRAJ_CMD / HAND_SOURCE_SET removed at PROTOCOL_VERSION 7 (2026-09-11,
    # skill-stack R1) with the hand-mastery latch and stroke-engine conduit they
    # served — the bridge is the one hand master and the lane follows the
    # HAS_HAND Setpoint bit alone.
    RpcMethod.PLATFORM_FW_BEGIN: ArgPlatformFwBegin,
    RpcMethod.PLATFORM_FW_DATA: ArgPlatformFwData,
    RpcMethod.PLATFORM_FW_VERIFY: ArgPlatformFwVerify,
    # BB_RELOAD/RESET/CALIBRATE_LOC are payloadless — no entry (matches NOP).
    # TILT_READ/STATE_READ are payloadless too (reply arrives as a PLATFORM_FRAME).
    # PLATFORM_FW_COMMIT is payloadless too — no entry (matches NOP).
}
