"""LEG_CLAMP_DIAG (0x94) -> the leg_clamp_diag /link_status row (2026-10-24).

Additive MsgType (can-bridge FW 31, no PROTOCOL_VERSION bump), so never-seen is
the NORMAL state against an older bridge and must not render as zeros: an
all-zero row would read as "the lead clamp never engaged", the false negative
this census exists to prevent. ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

from teensy_link import LegClampDiag, MsgType

from tests.ros._bridge_harness import (
    _build_paired_node,
    _link_kv,
    _teardown,
    _wait_until,
)


def _node():
    return _build_paired_node(boot_state_read=False)


def _send(teensy, node, clamp, stale):
    d = LegClampDiag(lead_clamp_ticks=tuple(clamp), stale_anchor_ticks=tuple(stale))
    teensy.send_to_jetson(int(MsgType.LEG_CLAMP_DIAG), d.pack())
    assert _wait_until(
        lambda: node._latest_leg_clamp_diag is not None
        and tuple(node._latest_leg_clamp_diag.lead_clamp_ticks) == tuple(clamp)
        and tuple(node._latest_leg_clamp_diag.stale_anchor_ticks) == tuple(stale))
    return d


def test_row_renders_every_counter_in_leg_order():
    teensy, client, node = _node()
    try:
        _send(teensy, node, (11, 22, 33, 44, 55, 66), (101, 202, 303, 404, 505, 606))
        row = _link_kv(node)['leg_clamp_diag']
        assert row == 'clamp 11 22 33 44 55 66 | stale 101 202 303 404 505 606'
    finally:
        _teardown(teensy, client, node)


def test_never_seen_is_explicit_not_silent():
    teensy, client, node = _node()
    try:
        assert node._latest_leg_clamp_diag is None
        kv = _link_kv(node)
        assert 'leg_clamp_diag' in kv, "the row must be published unconditionally"
        assert kv['leg_clamp_diag'] == 'unknown (never seen)'
        assert 'clamp' not in kv['leg_clamp_diag']  # not an all-zeros impersonation
    finally:
        _teardown(teensy, client, node)


def test_malformed_frame_is_dropped_and_rx_thread_survives():
    teensy, client, node = _node()
    try:
        teensy.send_to_jetson(int(MsgType.LEG_CLAMP_DIAG), b'\x01\x02\x03')
        _send(teensy, node, (1, 2, 3, 4, 5, 6), (6, 5, 4, 3, 2, 1))
        assert _link_kv(node)['leg_clamp_diag'] == 'clamp 1 2 3 4 5 6 | stale 6 5 4 3 2 1'
    finally:
        _teardown(teensy, client, node)


def test_row_is_published_unconditionally_beside_bridge_tx_diag():
    teensy, client, node = _node()
    try:
        keys = list(_link_kv(node).keys())
        assert 'bridge_tx_diag' in keys and 'leg_clamp_diag' in keys
        assert keys.index('leg_clamp_diag') == keys.index('bridge_tx_diag') + 1
    finally:
        _teardown(teensy, client, node)
