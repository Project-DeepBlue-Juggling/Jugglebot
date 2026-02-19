"""Ball Butler CAN protocol: heartbeat parsing and command encoding.

The Ball Butler is a separate subsystem with its own Teensy controller.
Communication with the host uses custom CAN IDs defined in protocol_config.
"""

import math
import struct
import time
from dataclasses import dataclass
from typing import ClassVar

import can
import jugglebot.protocol_config as proto

# CAN arbitration IDs
THROW_CMD_ID = proto.CAN_ID_BB_THROW_CMD
HEARTBEAT_ID = proto.CAN_ID_BB_HEARTBEAT
RELOAD_CMD_ID = proto.CAN_ID_BB_RELOAD_CMD
RESET_CMD_ID = proto.CAN_ID_BB_RESET_CMD
CALIBRATE_CMD_ID = proto.CAN_ID_BB_CALIBRATE_LOC_CMD

# Re-export enums from protocol_config
BallButlerStates = proto.BallButlerStates
BallButlerError = proto.BallButlerError


@dataclass
class BallButlerHeartbeat:
    """Decoded Ball Butler heartbeat from an 8-byte CAN frame.

    Frame layout (little-endian):
        Byte 0: bit 0 = ball_in_hand, bits 1-7 = state enum
        Byte 1: state_data (error code when state == ERROR, else 0)
        Bytes 2-3: yaw encoder (uint16, scaled by yaw_resolution)
        Bytes 4-5: pitch encoder (uint16, scaled by pitch_resolution)
        Bytes 6-7: hand encoder (uint16, scaled by hand_resolution)
    """
    yaw_resolution: ClassVar[float] = proto.HEARTBEAT_YAW_RES_DEG
    pitch_resolution: ClassVar[float] = proto.HEARTBEAT_PITCH_RES_DEG
    hand_resolution: ClassVar[float] = proto.HEARTBEAT_HAND_RES_MM

    ball_in_hand: bool = False
    state: BallButlerStates = BallButlerStates.BOOT
    state_data: int = 0
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    hand_mm: float = 0.0

    @classmethod
    def from_can_frame(cls, data: bytes) -> 'BallButlerHeartbeat':
        """Unpack from 8-byte CAN frame."""
        state_byte, state_data, yaw_enc, pitch_enc, hand_enc = struct.unpack('<BBHHH', data)
        return cls(
            ball_in_hand=bool(state_byte & 0x01),
            state=BallButlerStates(state_byte >> 1),
            state_data=state_data,
            yaw_deg=yaw_enc * cls.yaw_resolution,
            pitch_deg=pitch_enc * cls.pitch_resolution,
            hand_mm=hand_enc * cls.hand_resolution,
        )

    @property
    def error_code(self) -> BallButlerError:
        """Get error code (only meaningful when state == ERROR)."""
        if self.state == BallButlerStates.ERROR:
            return BallButlerError(self.state_data)
        return BallButlerError.NONE


# ═══════════════════════════════════════════════════════════════
# Command encoding
# ═══════════════════════════════════════════════════════════════

def encode_throw_command(yaw_rad: float, pitch_rad: float,
                         speed_mps: float, delay_s: float) -> can.Message:
    """Encode a throw command for the Ball Butler.

    Args:
        yaw_rad: Yaw angle in radians (-π to π).
        pitch_rad: Pitch angle in radians (0 to π/2).
        speed_mps: Throw speed in m/s (0 to 6.5535).
        delay_s: Relative delay before throwing (0 to 65.535 s).
                 Converted to absolute epoch milliseconds internally.

    CAN frame layout (8 bytes, little-endian):
        Bytes 0-1: yaw (int16, π/32768 rad/LSB)
        Bytes 2-3: pitch (uint16, π/65536 rad/LSB)
        Bytes 4-5: speed (uint16, 0.1 mm/s per LSB)
        Bytes 6-7: throw time (uint16, lower 16 bits of epoch ms)
    """
    if not (-math.pi <= yaw_rad <= math.pi):
        raise ValueError(f"Yaw {yaw_rad:.3f} rad outside [-π, π]")
    if not (0 <= pitch_rad <= math.pi / 2):
        raise ValueError(f"Pitch {pitch_rad:.3f} rad outside [0, π/2]")
    if not (0 <= speed_mps <= 6.5535):
        raise ValueError(f"Speed {speed_mps:.2f} m/s outside [0, 6.5535]")
    if not (0 <= delay_s <= 65.535):
        raise ValueError(f"Delay {delay_s:.2f} s outside [0, 65.535]")

    # Convert relative delay to absolute epoch ms, extract lower 16 bits.
    # This conversion MUST happen here because float32 ROS service fields
    # would destroy the lower bits of large epoch timestamps.
    throw_time_ms = int((time.time() + delay_s) * 1000.0) & 0xFFFF

    data = struct.pack('<hHHH',
                       int(yaw_rad * 32768.0 / math.pi),
                       int(pitch_rad * 65536.0 / math.pi),
                       int(speed_mps * 10000.0),
                       throw_time_ms)

    return can.Message(arbitration_id=THROW_CMD_ID, data=data,
                       dlc=8, is_extended_id=False)


def encode_state_command(cmd_id: int) -> can.Message:
    """Encode a reload, reset, or calibrate command (no payload)."""
    return can.Message(arbitration_id=cmd_id, data=[], dlc=0, is_extended_id=False)
