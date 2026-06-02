"""RPC method argument encoders for the leg-bridge link.

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
)

RpcMethod = p.RpcMethod

__all__ = [
    "AXIS_ALL",
    # dataclasses (re-exported)
    "ArgAxisState", "ArgControllerMode", "ArgVelCurr", "ArgPosGain",
    "ArgVelGains", "ArgAbsPosition", "ArgAxisOnly", "ArgSdoRead", "ArgSdoWrite",
    "ResultTimeOfDay",
    # encoders
    "encode_set_axis_state", "encode_set_controller_mode",
    "encode_set_vel_curr_limits", "encode_set_pos_gain", "encode_set_vel_gains",
    "encode_set_absolute_position", "encode_clear_errors", "encode_reboot",
    "encode_encoder_search", "encode_home", "encode_sdo_read", "encode_sdo_write",
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
    RpcMethod.SDO_READ: ArgSdoRead,
    RpcMethod.SDO_WRITE: ArgSdoWrite,
}
