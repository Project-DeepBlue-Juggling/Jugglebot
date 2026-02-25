"""ODrive CAN protocol: message encoding and decoding.

All ODrive CAN communication uses an 11-bit arbitration ID:
    arb_id = (node_id << 5) | command_id

Node IDs and command IDs come from protocol_config.py (generated from YAML).
Encoding uses struct.pack directly — no cantools/DBC dependency.
"""

import struct
from typing import Tuple

import can
import jugglebot.protocol_config as proto
import jugglebot.hardware_config as hw

# ═══════════════════════════════════════════════════════════════
# Constants re-exported from protocol_config
# ═══════════════════════════════════════════════════════════════

COMMANDS = proto.ODRIVE_COMMANDS
COMMAND_ID_TO_NAME = {v: k for k, v in COMMANDS.items()}

LEG_AXES = proto.NODE_ID_LEGS
NUM_LEGS = len(LEG_AXES)
HAND_AXIS = proto.NODE_ID_JUGGLEBOT_HAND
JUGGLEBOT_AXES = LEG_AXES + [HAND_AXIS]
BB_AXES = [proto.NODE_ID_BB_PITCH, proto.NODE_ID_BB_HAND]
ALL_AXES = JUGGLEBOT_AXES + BB_AXES

# ═══════════════════════════════════════════════════════════════
# ODrive enums — imported from protocol_config (source of truth)
# ═══════════════════════════════════════════════════════════════

AXIS_STATES = proto.ODRIVE_STATES
CONTROL_MODES = proto.ODRIVE_CONTROL_MODES
INPUT_MODES = proto.ODRIVE_INPUT_MODES

# ═══════════════════════════════════════════════════════════════
# Operational defaults
# ═══════════════════════════════════════════════════════════════

LEG_MOTOR_MAX_POSITION = hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS
HAND_MOTOR_MAX_POSITION = hw.GEOM_HAND_MOTOR_MAX_POSITION_REVS

DEFAULT_TRAP_TRAJ = {
    'vel_limit': hw.ODRIVE_TRAP_VEL_LIMIT_RPS,
    'acc_limit': hw.ODRIVE_TRAP_ACC_LIMIT_RPS2,
    'dec_limit': hw.ODRIVE_TRAP_DEC_LIMIT_RPS2,
}
DEFAULT_VEL_CURR = {
    'leg_vel': hw.ODRIVE_LEG_VEL_LIMIT_RPS,
    'leg_curr': hw.ODRIVE_LEG_CURR_LIMIT_A,
    'hand_vel': hw.ODRIVE_HAND_VEL_LIMIT_RPS,
    'hand_curr': hw.ODRIVE_HAND_CURR_LIMIT_A,
}
DEFAULT_HAND_GAINS = {
    'pos_gain': hw.ODRIVE_HAND_POS_GAIN,
    'vel_gain': hw.ODRIVE_HAND_VEL_GAIN,
    'vel_int_gain': hw.ODRIVE_HAND_VEL_INT_GAIN,
}

# ═══════════════════════════════════════════════════════════════
# SDO endpoint IDs (for arbitrary parameter read/write)
# ═══════════════════════════════════════════════════════════════

ENDPOINT_IDS = {
    'commutation_mapper.pos_abs': proto.ENDPOINT_COMMUTATION_MAPPER_POS_ABS,
    'get_gpio_states': proto.ENDPOINT_GPIO_STATES,
}

# ═══════════════════════════════════════════════════════════════
# Error code bitmask → human-readable name
# ═══════════════════════════════════════════════════════════════

ERROR_CODES = {
    1: "INITIALIZING",
    2: "SYSTEM_LEVEL",
    4: "TIMING_ERROR",
    8: "MISSING_ESTIMATE",
    16: "BAD_CONFIG",
    32: "DRV_FAULT",
    64: "MISSING_INPUT",
    256: "DC_BUS_OVER_VOLTAGE",
    512: "DC_BUS_UNDER_VOLTAGE",
    1024: "DC_BUS_OVER_CURRENT",
    2048: "DC_BUS_OVER_REGEN_CURRENT",
    4096: "CURRENT_LIMIT_VIOLATION",
    8192: "MOTOR_OVER_TEMP",
    16384: "INVERTER_OVER_TEMP",
    32768: "VELOCITY_LIMIT_VIOLATION",
    65536: "POSITION_LIMIT_VIOLATION",
    16777216: "WATCHDOG_TIMER_EXPIRED",
    33554432: "ESTOP_REQUESTED",
    67108864: "SPINOUT_DETECTED",
    134217728: "BRAKE_RESISTOR_DISARMED",
    268435456: "THERMISTOR_DISCONNECTED",
    1073741824: "CALIBRATION_ERROR",
}

# Frequently-referenced error bitmasks
ERR_DC_BUS_UNDER_VOLTAGE = 512

# Ball Butler motors — only process these command IDs from BB axes
BB_MOTOR_IDS = set(BB_AXES)
BB_RELEVANT_CMD_IDS = {
    COMMANDS['heartbeat_message'],
    COMMANDS['get_encoder_estimate'],
    COMMANDS['get_iq'],
    COMMANDS['get_temps'],
    COMMANDS['get_bus_voltage_current'],
}


def arb_id(axis_id: int, command_name: str) -> int:
    """Compute CAN arbitration ID: (node_id << 5) | command_id."""
    return (axis_id << 5) | COMMANDS[command_name]


# ═══════════════════════════════════════════════════════════════
# Encoding — outgoing commands to ODrives
# ═══════════════════════════════════════════════════════════════

def encode_set_state(axis_id: int, state: str) -> can.Message:
    """Set requested axis state (e.g. 'IDLE', 'CLOSED_LOOP')."""
    data = struct.pack('<I', AXIS_STATES[state]) + bytes(4)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_requested_state'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_controller_mode(axis_id: int, control_mode: str, input_mode: str) -> can.Message:
    """Set control mode and input mode for an axis."""
    data = struct.pack('<II', CONTROL_MODES[control_mode], INPUT_MODES[input_mode])
    return can.Message(arbitration_id=arb_id(axis_id, 'set_controller_mode'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_input_pos(axis_id: int, position: float,
                         vel_ff: int = 0, torque_ff: int = 0) -> can.Message:
    """Set input position with optional velocity and torque feedforward.

    Note: caller is responsible for leg inversion (negate for legs 0-5).
    vel_ff and torque_ff are raw int16 values as the ODrive expects.
    """
    data = struct.pack('<fhh', position, vel_ff, torque_ff)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_input_pos'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_input_vel(axis_id: int, velocity: float,
                         torque_ff: float = 0.0) -> can.Message:
    """Set input velocity with optional torque feedforward."""
    data = struct.pack('<ff', velocity, torque_ff)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_input_vel'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_input_torque(axis_id: int, torque: float) -> can.Message:
    """Set input torque (Nm) for torque control mode."""
    data = struct.pack('<f', torque) + bytes(4)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_input_torque'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_vel_curr_limits(axis_id: int, vel_limit: float,
                                curr_limit: float) -> can.Message:
    """Set absolute velocity and current limits."""
    data = struct.pack('<ff', vel_limit, curr_limit)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_vel_curr_limits'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_traj_vel_limit(axis_id: int, vel_limit: float) -> can.Message:
    """Set trapezoidal trajectory velocity limit."""
    data = struct.pack('<f', vel_limit) + bytes(4)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_traj_vel_limit'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_traj_acc_limits(axis_id: int, acc_limit: float,
                                dec_limit: float) -> can.Message:
    """Set trapezoidal trajectory acceleration and deceleration limits."""
    data = struct.pack('<ff', acc_limit, dec_limit)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_traj_acc_limits'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_absolute_position(axis_id: int, position: float) -> can.Message:
    """Set the absolute encoder position (used after homing)."""
    data = struct.pack('<f', position) + bytes(4)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_absolute_position'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_pos_gain(axis_id: int, pos_gain: float) -> can.Message:
    """Set position gain."""
    data = struct.pack('<f', pos_gain) + bytes(4)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_pos_gain'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_vel_gains(axis_id: int, vel_gain: float,
                          vel_int_gain: float) -> can.Message:
    """Set velocity gain and velocity integrator gain."""
    data = struct.pack('<ff', vel_gain, vel_int_gain)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_vel_gains'),
                       data=data, dlc=8, is_extended_id=False)


def encode_clear_errors(axis_id: int) -> can.Message:
    """Clear errors on a single axis."""
    return can.Message(arbitration_id=arb_id(axis_id, 'clear_errors'),
                       dlc=8, is_extended_id=False, data=bytes(8))


def encode_reboot(axis_id: int) -> can.Message:
    """Reboot a single ODrive axis."""
    return can.Message(arbitration_id=arb_id(axis_id, 'reboot_odrives'),
                       dlc=8, is_extended_id=False, data=bytes(8))


def encode_sdo_read(axis_id: int, param_name: str) -> can.Message:
    """Encode an SDO read request for an arbitrary parameter."""
    endpoint_id = ENDPOINT_IDS[param_name]
    data = struct.pack('<BHBf', proto.OPCODE_READ, endpoint_id, 0, 0.0)
    return can.Message(arbitration_id=arb_id(axis_id, 'RxSdo'),
                       data=data, dlc=8, is_extended_id=False)


def encode_sdo_write(axis_id: int, param_name: str, value: float) -> can.Message:
    """Encode an SDO write request for an arbitrary parameter."""
    endpoint_id = ENDPOINT_IDS[param_name]
    data = struct.pack('<BHBf', proto.OPCODE_WRITE, endpoint_id, 0, value)
    return can.Message(arbitration_id=arb_id(axis_id, 'RxSdo'),
                       data=data, dlc=8, is_extended_id=False)


# ═══════════════════════════════════════════════════════════════
# Decoding — incoming messages from ODrives
# ═══════════════════════════════════════════════════════════════

def _check_len(data: bytes, expected: int, name: str):
    """Raise ValueError if CAN frame data is too short for the expected unpack."""
    if len(data) < expected:
        raise ValueError(f"{name}: expected {expected} bytes, got {len(data)}")


def decode_heartbeat(data: bytes) -> Tuple[int, int, bool]:
    """Returns (current_state, procedure_result, trajectory_done)."""
    _check_len(data, 7, "heartbeat")
    return data[4], data[5], bool(data[6] & 0x01)


def decode_error(data: bytes) -> Tuple[int, int]:
    """Returns (active_errors, disarm_reason) as bitmasks."""
    _check_len(data, 8, "error")
    active_errors = struct.unpack_from('<I', data, 0)[0]
    disarm_reason = struct.unpack_from('<I', data, 4)[0]
    return active_errors, disarm_reason


def decode_encoder_estimate(data: bytes) -> Tuple[float, float]:
    """Returns (pos_estimate, vel_estimate) in rev and rev/s."""
    _check_len(data, 8, "encoder_estimate")
    return struct.unpack_from('<ff', data)


def decode_iq(data: bytes) -> Tuple[float, float]:
    """Returns (iq_setpoint, iq_measured) in amps."""
    _check_len(data, 8, "iq")
    return struct.unpack_from('<ff', data)


def decode_temps(data: bytes) -> Tuple[float, float]:
    """Returns (fet_temp, motor_temp) in deg C."""
    _check_len(data, 8, "temps")
    return struct.unpack_from('<ff', data)


def decode_bus_voltage_current(data: bytes) -> Tuple[float, float]:
    """Returns (bus_voltage, bus_current)."""
    _check_len(data, 8, "bus_voltage_current")
    return struct.unpack_from('<ff', data)


def decode_sdo_response(data: bytes) -> Tuple[int, float]:
    """Returns (endpoint_id, value) from an SDO read response."""
    _check_len(data, 8, "sdo_response")
    _, endpoint_id, _, value = struct.unpack_from('<BHBf', data)
    return endpoint_id, value


# ═══════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════

def clip_position(axis_id: int, setpoint: float, logger=None) -> float:
    """Validate and clip a Jugglebot position setpoint (pre-inversion).

    Caller should negate the return value for leg axes (0-5) before
    passing to encode_set_input_pos, since ODrive uses negative = extension.
    """
    if axis_id in LEG_AXES:
        max_pos = LEG_MOTOR_MAX_POSITION
    elif axis_id == HAND_AXIS:
        max_pos = HAND_MOTOR_MAX_POSITION
    else:
        raise ValueError(f"Invalid Jugglebot axis ID for position: {axis_id}")

    if setpoint < 0.0 or setpoint > max_pos:
        if logger:
            logger.warning(f"Setpoint {setpoint:.2f} for axis {axis_id} clipped to [0, {max_pos}]")
        setpoint = max(0.0, min(setpoint, max_pos))

    return setpoint


def error_names(bitmask: int) -> list:
    """Extract human-readable error names from a bitmask."""
    return [name for code, name in ERROR_CODES.items() if bitmask & code]
