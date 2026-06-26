"""RPC method argument encoders for the can-bridge link.

Thin, typed wrappers over the codegen-emitted argument dataclasses (hoisted into
``config/generate_udp_protocol.py`` at Phase 10b — firmware handoff D8). The
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
)

RpcMethod = p.RpcMethod

__all__ = [
    "AXIS_ALL",
    # dataclasses (re-exported)
    "ArgAxisState", "ArgControllerMode", "ArgVelCurr", "ArgPosGain",
    "ArgVelGains", "ArgAbsPosition", "ArgAxisOnly", "ArgSdoRead", "ArgSdoWrite",
    "ResultTimeOfDay", "ArgBbThrow",
    # encoders
    "encode_set_axis_state", "encode_set_controller_mode",
    "encode_set_vel_curr_limits", "encode_set_pos_gain", "encode_set_vel_gains",
    "encode_set_absolute_position", "encode_clear_errors", "encode_reboot",
    "encode_encoder_search", "encode_home", "encode_activate",
    "encode_sdo_read", "encode_sdo_write",
    "encode_bb_throw", "encode_bb_reload", "encode_bb_reset",
    "encode_bb_calibrate_loc",
    "decode_time_of_day_result",
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
    """ENCODER_SEARCH (firmware Phase 9 — returns ERR_NOT_IMPL today)."""
    return ArgAxisOnly(axis=int(axis)).pack()


def encode_home(axis: int = AXIS_ALL) -> bytes:
    """HOME (firmware Phase 9 — returns ERR_NOT_IMPL today)."""
    return ArgAxisOnly(axis=int(axis)).pack()


def encode_activate(axis: int = AXIS_ALL) -> bytes:
    """ACTIVATE: TRAP_TRAJ move to the active pose (Phase 11 U5).

    ``AXIS_ALL`` activates every present leg in parallel (even platform rise); a
    single leg index activates just that leg iff present.
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


# ── Ball Butler ─────────────────────────────────────────────────────────────
# Firmware-owned encoding: the bridge passes typed args; the can-bridge Teensy
# range-checks + frame-builds before TX on CAN1 (HANDOFF-firmware-three-bus D2).
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
    RpcMethod.SDO_READ: ArgSdoRead,
    RpcMethod.SDO_WRITE: ArgSdoWrite,
    RpcMethod.BB_THROW: ArgBbThrow,
    # BB_RELOAD/RESET/CALIBRATE_LOC are payloadless — no entry (matches NOP).
}
