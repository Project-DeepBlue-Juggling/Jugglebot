"""LEG_CMD uplink → /leg_cmd_executed on teensy_bridge_node (P0, bridge-temporal arc).

The can-bridge has emitted ``MsgType.LEG_CMD`` (0x88, 100 Hz) since the frame
was added for the float32 interp-residual bench work: the Teensy's own report of
what its 500 Hz interp ladder commanded to the leg ODrives —
``axes[i].target_pos_rev`` / ``target_vel_rps`` AFTER the lead and stroke clamps
(``telemetry.cpp::send_leg_cmd``) — stamped with the bridge wall clock. Until
now the Jetson decoded it nowhere: ``teensy_bridge_node`` simply did not
subscribe it, so the frame reached the box and died there.

That absence is the first of the three telemetry gaps named in the Addendum to
``logbook/2026-07-18-teensy-uptime-tracking-degradation.md``. Without this
timeline a bag carries only the two ENDS of the command chain —
``/leg_setpoint_echo`` (what the Jetson asked for) and ``robot_state`` (what the
encoders did) — so a lag that grows with bridge uptime (~10 ms fresh → ~240 ms
at 30 h) can be measured but never ATTRIBUTED to Jetson→Teensy transport vs the
interp ladder vs the ODrives. The middle timeline is the discriminator.

What this module pins:

1. **the mapping** — a decoded frame reaches ``/leg_cmd_executed`` as a
   ``JointState`` with per-leg positions/velocities in the right slots and the
   Teensy's own wall stamp on the header (not the drain time);
2. **the threading contract** — the RX callback QUEUES and does not publish;
   the executor-side timer drains. Publishing from the RX thread would put DDS
   work on the socket thread that also carries the 100 Hz telemetry and every
   RPC reply, which is the standing determinism rule for this node and the
   reason ``_on_bb_estimates`` is shaped the same way;
3. **the timer is actually registered** — a queue nobody drains is a leak, and
   the mock's ``create_timer`` is a no-op MagicMock, so a missing registration
   would otherwise be invisible to every other assertion here.

ROS 2 is mocked by tests/ros/conftest.py — except ``sensor_msgs``, which is the
REAL Foxy package on this box, so the ``position``/``velocity`` coercion below
runs against the real field types (``array('d')`` with real type checks). The
header does NOT get that treatment: the real ``JointState.__init__`` builds it
from the mocked ``std_msgs.msg.Header``, so the stamp assertions pin the
µs→(sec, nanosec) arithmetic by exact equality, not via builtin_interfaces'
int32 enforcement.
"""

from __future__ import annotations

import pytest

from teensy_link import LegCmd, MsgType

from tests.ros._bridge_harness import (
    _build_paired_node,
    _teardown,
    _wait_until,
)

# A realistic 2026 wall stamp in microseconds — deliberately NOT a small
# synthetic number. t_teensy_us is the bridge's wall clock (now_wall_us(),
# time-synced to the Jetson), so the seconds half lands in the 1.75e9 range —
# sized so a forgot-to-divide bug produces a value no int32 stamp.sec could
# hold on hardware; here it is caught by the exact-equality assertions below.
# A stamp built from micros64() uptime instead would pass a small-number test
# and fail on hardware; this one would not.
_T_US = 1_754_000_000_123_456

# Distinct per-leg values on purpose: with a uniform payload a transposed pair,
# a pos/vel swap or a truncated slice all still "look plausible". These are
# float32 on the wire, so compare with approx, not ==.
_POS = (0.125, -1.25, 2.5, -3.75, 4.0625, -5.5)
_VEL = (-10.0, 20.5, -30.25, 40.75, -50.125, 60.0)


def _node():
    return _build_paired_node(boot_state_read=False)


def _send(teensy, node, *, t_us=_T_US, pos=_POS, vel=_VEL):
    """Inject one LEG_CMD frame; wait for the RX thread to queue it."""
    before = len(node._leg_cmd_queue)
    c = LegCmd(t_teensy_us=t_us, cmd_pos_rev=pos, cmd_vel_rps=vel)
    teensy.send_to_jetson(int(MsgType.LEG_CMD), c.pack())
    assert _wait_until(lambda: len(node._leg_cmd_queue) > before), (
        'LEG_CMD frame never reached the node — is MsgType.LEG_CMD subscribed?')
    return c


def test_leg_cmd_frame_publishes_joint_state():
    """A decoded LEG_CMD lands on /leg_cmd_executed with the mapping intact.

    name=[leg0..leg5], position=cmd_pos_rev (motor rev, Jugglebot convention),
    velocity=cmd_vel_rps, header.stamp from t_teensy_us. The stamp is the
    load-bearing field: it is the TEENSY's clock, which is the only thing that
    makes a Jetson-vs-Teensy one-way delay measurable from a bag at all.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node)
        node._publish_leg_cmd_executed()

        published = node.leg_cmd_executed_pub.published
        assert len(published) == 1
        js = published[0]
        assert list(js.name) == ['leg0', 'leg1', 'leg2', 'leg3', 'leg4', 'leg5']
        assert list(js.position) == pytest.approx(list(_POS), rel=1e-6)
        assert list(js.velocity) == pytest.approx(list(_VEL), rel=1e-6)
        # 1_754_000_000_123_456 us → 1_754_000_000 s + 123_456_000 ns.
        assert js.header.stamp.sec == _T_US // 1_000_000
        assert js.header.stamp.nanosec == (_T_US % 1_000_000) * 1000
    finally:
        _teardown(teensy, client, node)


def test_rx_callback_queues_and_does_not_publish():
    """The RX thread never publishes; the executor-side drain does.

    The frame is driven through the REAL TeensyLinkClient RX thread, so this is
    the production path, not a direct callback call. After it lands the queue is
    non-empty and the publisher has seen nothing — the split the node's
    determinism rule requires. Only after the timer body runs does the message
    appear, and the queue is left empty (a drain that published without clearing
    would republish the same sample every 10 ms forever).
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node)
        assert node.leg_cmd_executed_pub.published == [], (
            'the RX callback published — publishing must happen on the '
            'executor thread (drain timer), never on the socket RX thread')

        node._publish_leg_cmd_executed()
        assert len(node.leg_cmd_executed_pub.published) == 1
        assert node._leg_cmd_queue == []

        # A second drain with nothing queued is a no-op, not a repeat.
        node._publish_leg_cmd_executed()
        assert len(node.leg_cmd_executed_pub.published) == 1
    finally:
        _teardown(teensy, client, node)


def test_every_sample_keeps_its_own_stamp_and_order():
    """Two frames in one drain → two messages, in order, each with ITS stamp.

    Per-sample stamping (not drain time) is what makes the 100 Hz series usable
    for a cross-correlation against /leg_setpoint_echo: a batch stamped at drain
    time would collapse to the drain cadence and manufacture up to a full timer
    period of phantom lag — precisely the quantity this topic exists to measure.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, t_us=_T_US, pos=_POS, vel=_VEL)
        second = tuple(p + 1.0 for p in _POS)
        _send(teensy, node, t_us=_T_US + 10_000, pos=second, vel=_VEL)
        node._publish_leg_cmd_executed()

        published = node.leg_cmd_executed_pub.published
        assert len(published) == 2
        assert published[0].header.stamp.nanosec == (_T_US % 1_000_000) * 1000
        assert (published[1].header.stamp.nanosec
                == ((_T_US + 10_000) % 1_000_000) * 1000)
        assert list(published[1].position) == pytest.approx(list(second), rel=1e-6)
    finally:
        _teardown(teensy, client, node)


def test_malformed_frame_is_dropped_silently():
    """A short/garbled payload must not raise on the RX thread.

    An exception escaping an RX callback poisons the shared socket thread — the
    one that also carries telemetry, heartbeats and RPC replies — so a truncated
    frame has to die inside the callback. Called directly (not over the wire) so
    the negative is deterministic rather than a timing race.
    """
    teensy, client, node = _node()
    try:
        node._on_leg_cmd(int(MsgType.LEG_CMD), 1, b'\x00' * 10, None)
        assert node._leg_cmd_queue == []
        node._publish_leg_cmd_executed()
        assert node.leg_cmd_executed_pub.published == []
    finally:
        _teardown(teensy, client, node)


def test_drain_timer_is_registered_at_the_telemetry_rate():
    """The 100 Hz drain timer exists.

    The mock's create_timer is a no-op MagicMock, so nothing else in this file
    would notice its absence — the queue would grow to its 4000-sample cap and
    the topic would stay silent on hardware while every test here passed.
    """
    from rclpy.node import Node as _MockNodeBase   # conftest-injected mock

    calls = []
    original = _MockNodeBase.create_timer

    def _recording_create_timer(self, *a, **kw):
        calls.append(a)
        return original(self, *a, **kw)

    _MockNodeBase.create_timer = _recording_create_timer
    try:
        teensy, client, node = _node()
    finally:
        _MockNodeBase.create_timer = original
    try:
        periods = [a[0] for a in calls
                   if len(a) > 1 and a[1] == node._publish_leg_cmd_executed]
        assert periods == [0.01], (
            'the /leg_cmd_executed drain is not registered at 100 Hz — '
            f'timer registrations seen: {calls}')
    finally:
        _teardown(teensy, client, node)
