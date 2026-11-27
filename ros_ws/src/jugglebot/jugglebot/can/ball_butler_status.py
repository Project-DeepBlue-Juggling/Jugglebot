"""Ball Butler receive-path status frame (BB FW >= 10, BUG-133): pure decode.

Kept apart from ``ball_butler.py`` (which imports python-can for its command
encoders) so ``teensy_bridge_node`` can decode the frame without adding
python-can to its import graph, like ``catching_cone.py``.
"""

from __future__ import annotations

from dataclasses import dataclass

import jugglebot.protocol_config as proto


@dataclass
class BallButlerStatus:
    """Decoded Ball Butler receive-path status (BB FW >= 10, BUG-133).

    An unsolicited CMD_RESULT (0x7D5) frame of command_type STATUS, every
    500 ms; the can-bridge relays it verbatim. Its own 8-byte layout, not
    outcome/detail0/detail1 (protocol_config.yaml ball_butler.command_types):

        Byte 0: command_type == STATUS
        Byte 1: rx_ring_depth_min  smallest true RX ring occupancy in the window
                (frames, 255 sat). Above 0: the ring never emptied for 500 ms,
                a standing queue (the BUG-133 delay line).
        Byte 2: rx_ring_depth_max  largest, same probe (frames, 255 sat)
        Byte 3: rx_ring_leak_max   largest occupancy minus ``_available``
                (stranded frames, 255 sat; 0 with the patched FlexCAN_T4)
        Bytes 4-6: uptime_s        uint24 LE, seconds since boot
        Byte 7: fifo_overflows     hardware RX FIFO overruns since boot (255 sat)

    An image older than FW 10 never sends the frame: absence of a decoded
    status is how the host reports "not available", never zeros.
    """

    rx_ring_depth_min: int
    rx_ring_depth_max: int
    rx_ring_leak_max: int
    uptime_s: int
    fifo_overflows: int

    @classmethod
    def from_cmd_result(cls, data: bytes) -> 'BallButlerStatus':
        """Unpack an 8-byte CMD_RESULT frame whose byte 0 is STATUS.

        Raises ValueError for a short frame or another command_type, so a
        caller can route every CMD_RESULT through ``is_status_frame`` first.
        """
        if len(data) < 8:
            raise ValueError(f"BB status: expected 8 bytes, got {len(data)}")
        if data[0] != int(proto.BallButlerCommandType.STATUS):
            raise ValueError(f"BB status: command_type {data[0]} is not STATUS")
        return cls(
            rx_ring_depth_min=data[1],
            rx_ring_depth_max=data[2],
            rx_ring_leak_max=data[3],
            uptime_s=data[4] | (data[5] << 8) | (data[6] << 16),
            fifo_overflows=data[7],
        )

    @staticmethod
    def is_status_frame(data: bytes) -> bool:
        """True iff a CMD_RESULT payload is a STATUS frame this decoder reads."""
        return len(data) >= 8 and data[0] == int(proto.BallButlerCommandType.STATUS)
