"""Ball Butler STATUS frame decode (BB FW 10, BUG-133): jugglebot.can.ball_butler_status.

The FW 10 board sends an unsolicited CMD_RESULT (0x7D5) of command_type STATUS
every 500 ms: RX ring depth min/max, leak, uptime and FIFO overruns. An FW 9
board sends only THROW / CANCEL CMD_RESULTs; those must never decode as status.
Layout: config/protocol_config.yaml ball_butler.command_types.STATUS and
BallButler CanInterface.cpp maybePublishStatus_.
"""

from __future__ import annotations

import pytest

import jugglebot.protocol_config as proto
from jugglebot.can.ball_butler_status import BallButlerStatus


def _status(depth_min=0, depth_max=0, leak=0, uptime_s=0, fifo=0):
    return bytes([int(proto.BallButlerCommandType.STATUS), depth_min, depth_max, leak,
                  uptime_s & 0xFF, (uptime_s >> 8) & 0xFF, (uptime_s >> 16) & 0xFF, fifo])


def test_status_command_type_is_five():
    # The BB firmware writes BallButlerCommandType::STATUS from the same YAML.
    assert int(proto.BallButlerCommandType.STATUS) == 5


def test_fw10_status_frame_decodes_every_field():
    data = _status(depth_min=1, depth_max=7, leak=0, uptime_s=0x011234, fifo=3)
    assert BallButlerStatus.is_status_frame(data)
    st = BallButlerStatus.from_cmd_result(data)
    assert st == BallButlerStatus(rx_ring_depth_min=1, rx_ring_depth_max=7,
                                  rx_ring_leak_max=0, uptime_s=0x011234,
                                  fifo_overflows=3)


def test_uptime_is_uint24_little_endian():
    st = BallButlerStatus.from_cmd_result(_status(uptime_s=0xFFFFFF))
    assert st.uptime_s == 16_777_215          # 194 days, the wrap point
    st = BallButlerStatus.from_cmd_result(_status(uptime_s=3600))
    assert st.uptime_s == 3600


def test_saturated_bytes_pass_through():
    st = BallButlerStatus.from_cmd_result(
        _status(depth_min=255, depth_max=255, leak=255, fifo=255))
    assert (st.rx_ring_depth_min, st.rx_ring_depth_max, st.rx_ring_leak_max,
            st.fifo_overflows) == (255, 255, 255, 255)


@pytest.mark.parametrize('cmd_type', ['THROW', 'RELOAD', 'CALIBRATE', 'HOME', 'CANCEL'])
def test_fw9_cmd_results_are_not_status_frames(cmd_type):
    # An FW 9 board's CMD_RESULTs: [type, outcome, d0 lo/hi, d1 lo/hi, 0, 0].
    data = bytes([int(getattr(proto.BallButlerCommandType, cmd_type)), 0x21,
                  0x10, 0x00, 0xF4, 0x01, 0, 0])
    assert not BallButlerStatus.is_status_frame(data)
    with pytest.raises(ValueError):
        BallButlerStatus.from_cmd_result(data)


def test_short_frame_is_not_a_status_frame():
    data = _status()[:6]
    assert not BallButlerStatus.is_status_frame(data)
    with pytest.raises(ValueError):
        BallButlerStatus.from_cmd_result(data)
