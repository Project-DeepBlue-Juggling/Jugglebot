"""BUG-132: the FW 15 receive-path quality fields in the capture header and the ``clock_suspect`` flag.

FW 15 exports ``sync_rx_delay_10us``, ``rx_ring_depth_max`` and ``rx_ring_leak_max`` in header bytes 58-61 (FW <= 14
wrote 0 there: they decode as None, and the capture is flagged suspect, because that board's RX queue could delay the
sync and put the Platform clock early). A capture is also suspect on the saturated/unmatched sentinel or any ring leak. A finite sync
delay is informational: FW 15 stamps the sync in the interrupt, so a loop() stall cannot move the clock.
"""
from __future__ import annotations

import json

import pytest

from teensy_link import platform_imu as pi
from teensy_link import platform_imu_capture as pcap
from tests.teensy_link.fake_platform_imu import build_image


def _hdr(**kw):
    return pi.decode_header(build_image(n=200, n_anchors=4, **kw)["header"])


def test_fw15_header_round_trips_the_three_fields():
    h = _hdr(fw_version=15, sync_rx_delay_10us=37, rx_ring_depth_max=5, rx_ring_leak_max=0)
    assert (h.sync_rx_delay_10us, h.rx_ring_depth_max, h.rx_ring_leak_max) == (37, 5, 0)
    assert pcap.header_to_dict(h)["sync_rx_delay_ms"] == pytest.approx(0.37)
    assert pcap.clock_quality(h) == (False, "")


def test_fw14_header_has_none_and_is_suspect():
    h = _hdr(fw_version=14)
    assert (h.sync_rx_delay_10us, h.rx_ring_depth_max, h.rx_ring_leak_max) == (None, None, None)
    d = pcap.header_to_dict(h)
    assert d["sync_rx_delay_ms"] is None
    suspect, reason = pcap.clock_quality(h)
    assert suspect and reason == "fw<15: Platform RX queue could delay the sync, BUG-132"
    json.dumps(d)                                    # None serialises


@pytest.mark.parametrize("delay,leak,suspect", [
    (200, 0, False),
    (10300, 0, False),                               # a 103 ms loop() stall: the clock is unaffected on FW 15
    (pi.SYNC_RX_DELAY_SATURATED, 0, True),
    (pi.SYNC_RX_DELAY_NONE, 0, False),               # no beacon dispatched in the capture: nothing to judge
    (30, 1, True),
])
def test_thresholds_and_sentinels(delay, leak, suspect):
    h = _hdr(fw_version=15, sync_rx_delay_10us=delay, rx_ring_leak_max=leak)
    got, reason = pcap.clock_quality(h)
    assert got is suspect
    assert bool(reason) is suspect
    if delay in (pi.SYNC_RX_DELAY_SATURATED, pi.SYNC_RX_DELAY_NONE):
        assert pcap.header_to_dict(h)["sync_rx_delay_ms"] is None


def test_diag_decode_carries_the_peaks_and_keeps_them_out_of_the_deltas():
    blob = bytes([1, 1]) + bytes(8) + bytes(10) + bytes([0, 0]) + bytes(4) + (77).to_bytes(2, "little") + bytes([4, 2])
    d = pi.decode_diag(blob)
    assert (d["sync_rx_delay_10us"], d["rx_ring_depth_max"], d["rx_ring_leak_max"]) == (77, 4, 2)
    assert "rx_ring_leak_max" not in pi.diag_deltas(d, d)
