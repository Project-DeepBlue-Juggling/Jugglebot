"""CAN3 wire-error instrument on teensy_bridge_node (2026-07-29 CAN3 flap, FIX B).

The can-bridge has computed per-class CAN wire-error counters on every 1 kHz
service tick since the 2026-07-05 marginal-CAN3 work, and then thrown them away
to the USB serial console — ``can_buses.h`` said outright that they are "NOT on
the UDP uplink". That is precisely why the 2026-07-29 CAN3 bus-health flap could
not be root-caused from an 8 MB rosbag: the bag proved CAN3 was entering
error-passive at 42.4 % duty but carried nothing that could name WHICH error
class was driving it, so layer 2 of that investigation stayed open pending a
serial-console bench session
(``logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md``).

``MsgType.CAN_ERRORS`` (0x8C, additive — no PROTOCOL_VERSION bump) closes that
gap. This module drives the frame through the real ``TeensyLinkClient`` RX
thread into the node and asserts the single ROS observable: the ``can3_errors``
row on ``/link_status``.

Two states are covered, and the second is the one with teeth:

1. **fresh** — a frame arrived; every counter reaches the row verbatim, so an
   operator differencing two captures (poller on vs off) is reading the
   bridge's numbers and not a lossy re-derivation;
2. **never-seen** — no frame has ever arrived, which is the NORMAL steady state
   against any bridge older than FW 5. The row must render
   ``unknown (never seen)`` rather than disappearing or rendering zeros: a
   missing row, or a row of zeros, would read as "CAN3 is clean" — the exact
   false-negative this instrument exists to prevent.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

from controller.teensy_link import CanErrors, MsgType

from tests.ros._bridge_harness import (
    _build_paired_node,
    _link_kv,
    _teardown,
    _wait_until,
)


def _node():
    teensy, client, node = _build_paired_node(boot_state_read=False)
    return teensy, client, node


def _send(teensy, node, **kw):
    """Inject one CAN_ERRORS frame and wait for the RX thread to cache it."""
    ce = CanErrors(**kw)
    teensy.send_to_jetson(int(MsgType.CAN_ERRORS), ce.pack())
    assert _wait_until(lambda: node._latest_can_errors is not None
                       and int(node._latest_can_errors.tec_live) == int(ce.tec_live)
                       and int(node._latest_can_errors.ack_cnt) == int(ce.ack_cnt))
    return ce


def test_can3_errors_row_renders_every_counter():
    """A fresh frame reaches /link_status with every field intact.

    Distinct non-zero values per field on purpose: a transposed or duplicated
    field would still "look plausible" with a uniform payload, and this row is
    the input to a discriminator table that decides whether the 2026-07-29 wire
    errors are TX-side (tecInc/txctx/ack/bit1) or RX-side (recInc/rxctx/
    crc/form/stuff). Getting two of those swapped would point the next bench
    session at the wrong half of the bus.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node,
              ack_cnt=11, crc_cnt=22, form_cnt=33, stuff_cnt=44,
              bit0_cnt=55, bit1_cnt=66, err_tx_ctx=77, err_rx_ctx=88,
              tec_inc_sum=999, rec_inc_sum=1010, tx_gated=12,
              tec_live=130, rec_live=7, flt_live=1, flt_sustained=1)
        row = _link_kv(node)['can3_errors']
        # The two an operator reads first: flt=1 is an instantaneous
        # error-passive reading (transient, normal); sust=1 means it persisted
        # past CAN_PASSIVE_SUSTAIN_US and the command gate is actually refusing.
        assert 'flt=1' in row
        assert 'sust=1' in row
        assert 'tec=130' in row and 'rec=7' in row
        assert 'tecInc=999' in row and 'recInc=1010' in row
        assert 'ack=11' in row and 'crc=22' in row
        assert 'form=33' in row and 'stuff=44' in row
        assert 'bit0=55' in row and 'bit1=66' in row
        assert 'txctx=77' in row and 'rxctx=88' in row
        assert 'gated=12' in row
    finally:
        _teardown(teensy, client, node)


def test_can3_errors_distinguishes_healthy_from_gating():
    """flt/sust track the frame, so a transient never reads as an outage.

    The whole point of the FIX A split is that instantaneous error-passive
    (flt=1) is a normal, self-healing transient while SUSTAINED confinement
    (sust=1) is the state that actually refuses CAN3 commands. If the row
    collapsed those two, it would be useless for telling "the bus blipped" from
    "half your commands are being dropped".
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, ack_cnt=1, tec_live=0, flt_live=0, flt_sustained=0)
        row = _link_kv(node)['can3_errors']
        assert 'flt=0' in row and 'sust=0' in row

        _send(teensy, node, ack_cnt=2, tec_live=129, flt_live=1, flt_sustained=0)
        row = _link_kv(node)['can3_errors']
        assert 'flt=1' in row and 'sust=0' in row    # passive, gate NOT refusing

        _send(teensy, node, ack_cnt=3, tec_live=140, flt_live=1, flt_sustained=1)
        row = _link_kv(node)['can3_errors']
        assert 'flt=1' in row and 'sust=1' in row    # gate refusing
    finally:
        _teardown(teensy, client, node)


def test_can3_errors_never_seen_is_explicit_not_silent():
    """No frame ever ⇒ the row still exists and says so.

    This is the steady state against any bridge older than FW 5, so it must be
    unmistakable. A vanished row or a row of zeros would both read as "CAN3 is
    clean", which is the false negative the instrument exists to prevent.
    """
    teensy, client, node = _node()
    try:
        assert node._latest_can_errors is None
        row = _link_kv(node)
        assert 'can3_errors' in row, "the row must be published unconditionally"
        assert row['can3_errors'] == 'unknown (never seen)'
        # Explicitly NOT the all-zeros rendering: 'flt=0 sust=0 tec=0 ...' is what
        # a healthy FW 5 bridge looks like, and never-seen must not impersonate it.
        assert 'flt=' not in row['can3_errors']
        assert 'tec=' not in row['can3_errors']
    finally:
        _teardown(teensy, client, node)


def test_can3_errors_survives_a_malformed_frame():
    """A truncated payload is dropped without killing the RX thread.

    Same contract as every other RX callback: the bridge link must never die on
    a bad frame. Verified by landing a GOOD frame afterwards — if the RX thread
    had died, the cache would stay None.
    """
    teensy, client, node = _node()
    try:
        teensy.send_to_jetson(int(MsgType.CAN_ERRORS), b'\x01\x02\x03')
        _send(teensy, node, ack_cnt=5, tec_live=42, flt_live=0, flt_sustained=0)
        assert 'ack=5' in _link_kv(node)['can3_errors']
    finally:
        _teardown(teensy, client, node)
