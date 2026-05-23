"""Catching Cone CAN protocol: catch-event and heartbeat frame decoding.

The instrumented catching cone is a Teensy 4.x sensor node on the CAN bus.
A piezo contact sensor on the cup detects a ball landing; the cone broadcasts
two custom-ID frames — CATCH_EVENT (per impact) and CONE_HEARTBEAT (liveness).
The host only ever *decodes* these; the cone takes no commands.

See the logbook entry ``2026-05-23-catching-cone-hardware-bringup.md`` Design
section for protocol/wiring background; the firmware ``CatchingCone_code.ino``
header has the wiring sketch.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import jugglebot.protocol_config as proto

# CAN arbitration IDs
CATCH_EVENT_ID = proto.CAN_ID_CC_CATCH_EVENT
HEARTBEAT_ID = proto.CAN_ID_CC_HEARTBEAT

# Re-export the state enum from protocol_config
CatchingConeStates = proto.CatchingConeStates

# flags bit positions. _FLAG_RETRIGGER_SUPPRESSED and _FLAG_ANY_CATCH share
# bit value 0x02 but live in different frames (CATCH_EVENT byte 5 vs
# CONE_HEARTBEAT byte 6) so there is no clash.
_FLAG_TIME_SYNCED          = 0x01   # CATCH_EVENT byte 5 / CONE_HEARTBEAT byte 6
_FLAG_RETRIGGER_SUPPRESSED = 0x02   # CATCH_EVENT byte 5 only
_FLAG_ANY_CATCH            = 0x02   # CONE_HEARTBEAT byte 6 only

# 32-bit wrap span for timestamp reconstruction
_WRAP = 1 << 32          # 0x1_0000_0000
_MASK32 = _WRAP - 1      # 0xFFFF_FFFF
_HALF_WRAP = 1 << 31


@dataclass
class CatchEvent:
    """Decoded catch event from an 8-byte CATCH_EVENT CAN frame.

    Frame layout (little-endian, struct '<IBBH'):
        Bytes 0-3: catch_time_us_low32 — low 32 bits of the cone's wall-clock
                   microsecond timestamp, latched in the piezo ISR
        Byte 4:    sequence — monotonic uint8, wraps at 256
        Byte 5:    flags — bit0 time_synced, bit1 retrigger_suppressed
        Bytes 6-7: reserved

    ``time_synced`` is the load-bearing flag: when False the cone had not yet
    locked to the global clock, so ``catch_time_us_low32`` is a raw local
    timestamp and must NOT be compared against wall-clock times.
    """
    catch_time_us_low32: int = 0
    sequence: int = 0
    time_synced: bool = False
    retrigger_suppressed: bool = False

    @classmethod
    def from_can_frame(cls, data: bytes) -> 'CatchEvent':
        """Unpack from an 8-byte CAN frame. Raises ValueError if too short."""
        if len(data) < 8:
            raise ValueError(f"catch event: expected 8 bytes, got {len(data)}")
        low32, sequence, flags, _reserved = struct.unpack('<IBBH', data[:8])
        return cls(
            catch_time_us_low32=low32,
            sequence=sequence,
            time_synced=bool(flags & _FLAG_TIME_SYNCED),
            retrigger_suppressed=bool(flags & _FLAG_RETRIGGER_SUPPRESSED),
        )


@dataclass
class ConeHeartbeat:
    """Decoded heartbeat from an 8-byte CONE_HEARTBEAT CAN frame.

    Frame layout (little-endian):
        Byte 0:    state — CatchingConeStates enum
        Byte 1:    state_data — sync-jitter RMS in µs (clamped 0-255),
                   or an error code when state == ERROR
        Byte 2:    last_catch_seq — sequence of the most recent catch
        Bytes 3-5: ms_since_last_catch — uint24, clamped
        Byte 6:    flags — bit0 time_synced, bit1 have_any_catch
        Byte 7:    reserved

    ``have_any_catch`` disambiguates the boot state (no catches yet,
    ``last_catch_seq`` and ``ms_since_last_catch`` are placeholders) from a
    real long-elapsed reading.
    """
    state: CatchingConeStates = CatchingConeStates.BOOT
    state_data: int = 0
    last_catch_seq: int = 0
    ms_since_last_catch: int = 0
    time_synced: bool = False
    have_any_catch: bool = False

    @classmethod
    def from_can_frame(cls, data: bytes) -> 'ConeHeartbeat':
        """Unpack from an 8-byte CAN frame. Raises ValueError if too short."""
        if len(data) < 8:
            raise ValueError(f"cone heartbeat: expected 8 bytes, got {len(data)}")
        flags = data[6]
        return cls(
            state=CatchingConeStates(data[0]),
            state_data=data[1],
            last_catch_seq=data[2],
            ms_since_last_catch=int.from_bytes(data[3:6], 'little'),
            time_synced=bool(flags & _FLAG_TIME_SYNCED),
            have_any_catch=bool(flags & _FLAG_ANY_CATCH),
        )

    @property
    def sync_rms_us(self) -> int:
        """Time-sync jitter RMS in µs (0 when in ERROR — state_data is then an error code)."""
        return 0 if self.state == CatchingConeStates.ERROR else self.state_data


def reconstruct_catch_time_us(low32: int, host_now_us: int) -> int:
    """Reconstruct a full 64-bit wall-clock µs timestamp from the low 32 bits.

    The CATCH_EVENT frame carries only the low 32 bits of the cone's
    microsecond wall clock (it wraps every ~71.6 min). The host rebuilds the
    high bits from its own clock, assuming the true catch time is within
    ±2^31 µs (~35.8 min) of ``host_now_us`` — the µs-domain mirror of the
    firmware's reconstructWallMs().

    Args:
        low32: catch_time_us_low32 field from the frame (0 .. 2^32-1).
        host_now_us: host wall clock in µs at decode time, e.g.
                     ``int(time.time() * 1e6)``.

    Returns:
        Full 64-bit wall-clock microsecond timestamp.
    """
    candidate = (host_now_us & ~_MASK32) | (low32 & _MASK32)
    if candidate > host_now_us + _HALF_WRAP:
        candidate -= _WRAP
    elif candidate + _HALF_WRAP < host_now_us:
        candidate += _WRAP
    return candidate
