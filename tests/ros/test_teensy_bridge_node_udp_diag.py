"""/udp_diag — the per-message-type UDP link census (F2/R2, 2026-08-22).

Until this topic existed the ONLY per-type information about the Jetson ↔
can-bridge UDP link lived in ``teensy_link.LinkStats`` in the bridge node's
memory: nothing published it, so no rosbag ever carried it and the GUI could
only show ROS-topic rates — which, for spied topics, are the browser's
*received* rate behind a 200 ms rosbridge throttle, not the wire rate.

Two properties this file pins, both chosen because getting them wrong is
SILENT:

* **counters, not rates.** A rate published here would be a rate over whatever
  interval this timer actually fired on, with the divisor discarded before it
  reached the bag. Cumulative counters difference offline at any window.
* **every type, every tick.** ``rx_<TYPE>``/``tx_<TYPE>`` rows are emitted for
  the whole MsgType inventory, so a type that is not flowing reads as a real
  ``0`` instead of vanishing — the same "absent row reads as no problem" false
  negative the BRIDGE_TX_DIAG instrument exists to avoid.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticStatus

from teensy_link import MsgType

from tests.ros._bridge_harness import (
    _build_paired_node,
    _teardown,
    _wait_until,
)


def _diag_kv(node):
    """Publish one /udp_diag and return its key→value dict."""
    node._publish_udp_diag()
    return {v.key: v.value for v in node.udp_diag_pub.published[-1].values}


def test_udp_diag_publishes_every_msgtype_both_directions():
    teensy, client, node = _build_paired_node()
    try:
        kv = _diag_kv(node)
        for t in MsgType:
            assert f'rx_{t.name}' in kv, f'no rx row for {t.name}'
            assert f'tx_{t.name}' in kv, f'no tx row for {t.name}'
        # Names, not ids: a bag consumer must not have to carry an enum table.
        assert 'rx_129' not in kv and 'rx_TELEMETRY' in kv
    finally:
        _teardown(teensy, client, node)


def test_udp_diag_counts_are_cumulative_not_rates():
    """Send more frames; the counter must GROW, not re-report a per-tick rate."""
    teensy, client, node = _build_paired_node()
    try:
        teensy.send_telemetry()
        assert _wait_until(
            lambda: client.stats.rx_count_by_type.get(int(MsgType.TELEMETRY), 0) >= 1)
        first = int(_diag_kv(node)[f'rx_{MsgType.TELEMETRY.name}'])
        assert first >= 1

        for _ in range(3):
            teensy.send_telemetry()
        assert _wait_until(
            lambda: client.stats.rx_count_by_type.get(int(MsgType.TELEMETRY), 0)
            >= first + 3)
        second = int(_diag_kv(node)[f'rx_{MsgType.TELEMETRY.name}'])
        assert second >= first + 3, 'counter did not accumulate across ticks'
    finally:
        _teardown(teensy, client, node)


def test_udp_diag_counts_tx_by_type():
    teensy, client, node = _build_paired_node()
    try:
        before = int(_diag_kv(node)[f'tx_{MsgType.HEARTBEAT_J2T.name}'])
        for _ in range(2):
            client.send_heartbeat(t_jetson_us=1)
        after = int(_diag_kv(node)[f'tx_{MsgType.HEARTBEAT_J2T.name}'])
        assert after == before + 2
    finally:
        _teardown(teensy, client, node)


def test_udp_diag_omits_zero_gap_rows_and_emits_nonzero_ones():
    """21 permanently-zero gap rows would bury the one that matters."""
    teensy, client, node = _build_paired_node()
    try:
        kv = _diag_kv(node)
        assert not any(k.startswith('gap_') for k in kv), \
            'zero gap rows must be omitted'

        teensy.send_telemetry()          # seq 0
        teensy._tx_seq_stream = 5        # skip ahead → one gap event
        teensy.send_telemetry()          # seq 5
        assert _wait_until(
            lambda: client.stats.seq_gaps_by_type.get(int(MsgType.TELEMETRY), 0) >= 1)
        kv = _diag_kv(node)
        assert int(kv[f'gap_{MsgType.TELEMETRY.name}']) >= 1
    finally:
        _teardown(teensy, client, node)


def test_udp_diag_carries_the_aggregates():
    """rx_frames counts EVERY datagram, so it is >= the sum of the per-type
    rows; the difference (crc + decode failures, unknown types) is only
    readable if the aggregates ride in the same message."""
    teensy, client, node = _build_paired_node()
    try:
        teensy.send_telemetry()
        assert _wait_until(lambda: client.stats.rx_frames >= 1)
        kv = _diag_kv(node)
        for key in ('rx_frames', 'tx_frames', 'crc_errors', 'decode_errors',
                    'drain_capped'):
            assert key in kv, f'missing aggregate {key}'
        per_type_rx = sum(int(v) for k, v in kv.items() if k.startswith('rx_')
                          and k != 'rx_frames')
        assert per_type_rx <= int(kv['rx_frames'])
    finally:
        _teardown(teensy, client, node)


def test_udp_diag_level_is_delta_based_not_cumulative():
    """A single corrupted frame at boot must not latch WARN for the session.

    The level reads the CRC+decode DELTA between ticks, so it clears itself on
    the next clean tick.  (Advisory either way — nothing gates on this topic.)
    """
    teensy, client, node = _build_paired_node()
    try:
        _diag_kv(node)                       # baseline tick
        client.stats.crc_errors += 1         # a corrupted frame lands
        _diag_kv(node)
        assert node.udp_diag_pub.published[-1].level == DiagnosticStatus.WARN, \
            'expected WARN on new errors'
        _diag_kv(node)                       # nothing new since
        assert node.udp_diag_pub.published[-1].level == DiagnosticStatus.OK, \
            'cumulative errors latched the level — it must follow the delta'
    finally:
        _teardown(teensy, client, node)
