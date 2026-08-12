"""CLOCK_DIAG uplink → /clock_diag on teensy_bridge_node (P1, bridge-temporal arc).

The can-bridge emits ``MsgType.CLOCK_DIAG`` (0x8F) once per accepted
time-of-day anchor from FW 11 onward: the wall-clock discipline state
``time_base.cpp::set_wall_anchor`` has always computed and then DISCARDED
(pre-slew offset error, the exchange RTT, the implied crystal frequency error),
plus the 500 Hz interp ladder's recover-slew and Mode-2-extrapolation occupancy
over the window since the previous emit. Those are the last two telemetry gaps
named in the Addendum to
``logbook/2026-07-18-teensy-uptime-tracking-degradation.md``, and the clock half
is the raw material ``plans/active/bridge-clock-frequency-discipline.md``
Phase 1 needs before its servo is designed.

**FW 11 is committed UNFLASHED** — a flash reboots the Teensy and destroys the
aged uptime state the S1 isolation experiment exists to interrogate — so for the
whole pre-flash window no board emits this frame and no bag can contradict the
host decode. These tests are the only thing that keeps the surface honest until
then, which is why they pin behaviour and not merely arrival.

What this module pins:

1. **the mapping** — a decoded frame reaches ``/clock_diag`` as a
   ``DiagnosticStatus`` with every wire field present as a KeyValue, and the
   flags rendered as separate, independently-readable rows;
2. **the FREQ_VALID contract** — ``freq_ppb`` renders as ``n/a`` when the bit is
   clear. The wire carries 0 there and a bare ``0`` reads as "no frequency
   error", which is the precise opposite of "no measurement". Closing that at
   the render site is cheaper than trusting every future consumer to check a
   bit;
3. **the threading contract** — the RX callback QUEUES and does not publish; the
   executor-side timer drains. Publishing from the RX thread would put DDS work
   on the socket thread that also carries the 100 Hz telemetry and every RPC
   reply;
4. **every sample survives** — anchors are ~30 s apart, so a latest-wins cache
   would delete data points from a fit that has only ~120 of them per hour;
5. **an FW 10 board is silent, not broken** — no 0x8F ever arrives, the queue
   stays empty, and the drain publishes nothing rather than raising.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

from teensy_link import ClockDiag, MsgType
from teensy_link.protocol import ClockDiagFlags

from tests.ros._bridge_harness import (
    _build_paired_node,
    _teardown,
    _wait_until,
)

# A realistic steady-state sample: ~30 s since the previous anchor, a sub-ms
# RTT, a sub-ms pre-slew error and a single-digit-ppm crystal — deliberately NOT
# round synthetic numbers, so a transposed field or a unit slip is visible.
_STEADY = dict(
    t_local_us=57_600_123_456,          # ~16 h of bridge uptime, the S1 target age
    jetson_wall_us=1_754_000_000_123_456,
    dt_local_us=30_000_017,
    rtt_us=812,
    err_us=-437,
    freq_ppb=-9_140,                    # -9.14 ppm
    anchor_seq=1_921,
    interp_ticks=15_000,                # 30 s x 500 Hz
    recover_slew_ticks=0,
    extrap_ticks=37,
    flags=int(ClockDiagFlags.FREQ_VALID),
)


def _node():
    return _build_paired_node(boot_state_read=False)


def _send(teensy, node, **overrides):
    """Inject one CLOCK_DIAG frame; wait for the RX thread to queue it."""
    before = len(node._clock_diag_queue)
    fields = dict(_STEADY)
    fields.update(overrides)
    cd = ClockDiag(**fields)
    teensy.send_to_jetson(int(MsgType.CLOCK_DIAG), cd.pack())
    assert _wait_until(lambda: len(node._clock_diag_queue) > before), (
        'CLOCK_DIAG frame never reached the node — is MsgType.CLOCK_DIAG '
        'subscribed?')
    return cd


def _kv(msg):
    return {kv.key: kv.value for kv in msg.values}


def test_clock_diag_frame_publishes_every_wire_field():
    """A decoded CLOCK_DIAG lands on /clock_diag with the whole payload intact.

    Every field is asserted individually. A partial surface would be worse than
    none: the analysis this frame exists for re-derives the measured offset
    (``jetson_wall_us - t_local_us``) and re-checks ``freq_ppb`` against
    ``dt_local_us``, so a silently-dropped field turns a falsifiable instrument
    into one that has to be trusted.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node)
        node._publish_clock_diag()

        published = node.clock_diag_pub.published
        assert len(published) == 1
        msg = published[0]
        assert msg.name == 'teensy/clock_diag'
        vals = _kv(msg)
        assert vals['t_local_us'] == '57600123456'
        assert vals['jetson_wall_us'] == '1754000000123456'
        assert vals['dt_local_us'] == '30000017'
        assert vals['rtt_us'] == '812'
        assert vals['err_us'] == '-437'
        assert vals['freq_ppb'] == '-9140'
        assert vals['anchor_seq'] == '1921'
        assert vals['interp_ticks'] == '15000'
        assert vals['recover_slew_ticks'] == '0'
        assert vals['extrap_ticks'] == '37'
        assert vals['stepped'] == '0'
        assert vals['first_anchor'] == '0'
        assert vals['freq_valid'] == '1'
    finally:
        _teardown(teensy, client, node)


def test_freq_ppb_renders_na_when_the_sample_is_refused():
    """FREQ_VALID clear ⇒ ``freq_ppb`` is ``n/a``, never ``0``.

    The firmware zeroes ``freq_ppb`` whenever the interval is not a crystal
    measurement (a step, the first anchor, a saturated interval). On the wire
    that 0 is indistinguishable from a perfectly-disciplined oscillator, and the
    only thing separating the two is the bit. Rendering the number unqualified
    would hand a downstream ppm fit a fabricated zero — a data point that looks
    like the best sample in the set.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, freq_ppb=0, flags=int(ClockDiagFlags.STEPPED))
        node._publish_clock_diag()
        vals = _kv(node.clock_diag_pub.published[0])
        assert vals['freq_ppb'] == 'n/a'
        assert vals['freq_valid'] == '0'
        assert 'n/a' in node.clock_diag_pub.published[0].message
    finally:
        _teardown(teensy, client, node)


def test_step_vs_slew_is_distinguishable_and_only_a_real_step_warns():
    """STEPPED / FIRST_ANCHOR are separate rows, and the level follows them.

    A step means the measured offset moved further in one interval than any
    crystal can explain (>20 ms in 30 s is >666 ppm), so it marks a link gap or
    a host clock jump — worth a WARN. The FIRST anchor of a session also steps,
    by construction (it establishes the epoch), and warning on it would fire
    once every launch and train the operator to ignore the level. The two are
    therefore separate bits and the render keeps them separate.
    """
    teensy, client, node = _node()
    try:
        from diagnostic_msgs.msg import DiagnosticStatus

        # Steady state: slewed → OK.
        _send(teensy, node)
        # First anchor after boot: stepped AND first → OK, not a fault.
        _send(teensy, node, anchor_seq=1, dt_local_us=0, err_us=0, freq_ppb=0,
              flags=int(ClockDiagFlags.STEPPED) | int(ClockDiagFlags.FIRST_ANCHOR))
        # Mid-session re-acquisition: stepped but NOT first → WARN.
        _send(teensy, node, anchor_seq=97, err_us=2_147_483_647, freq_ppb=0,
              flags=int(ClockDiagFlags.STEPPED))
        node._publish_clock_diag()

        steady, first, regained = node.clock_diag_pub.published
        assert steady.level == DiagnosticStatus.OK
        assert _kv(steady)['stepped'] == '0'

        assert first.level == DiagnosticStatus.OK
        assert _kv(first)['stepped'] == '1'
        assert _kv(first)['first_anchor'] == '1'

        assert regained.level == DiagnosticStatus.WARN
        assert _kv(regained)['stepped'] == '1'
        assert _kv(regained)['first_anchor'] == '0'
        # err_us saturates at the int32 rail on a step — it must survive as the
        # rail, not wrap into a plausible small number with the wrong sign.
        assert _kv(regained)['err_us'] == '2147483647'
    finally:
        _teardown(teensy, client, node)


def test_rx_callback_queues_and_does_not_publish():
    """The RX thread never publishes; the executor-side drain does.

    Driven through the REAL TeensyLinkClient RX thread, so this is the
    production path. The drain must also CLEAR the queue: a drain that published
    without clearing would republish every anchor once a second forever, which
    for a series meant to be fitted is not a cosmetic defect.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node)
        assert node.clock_diag_pub.published == [], (
            'the RX callback published — publishing must happen on the '
            'executor thread (drain timer), never on the socket RX thread')

        node._publish_clock_diag()
        assert len(node.clock_diag_pub.published) == 1
        assert node._clock_diag_queue == []

        # A second drain with nothing queued is a no-op, not a repeat.
        node._publish_clock_diag()
        assert len(node.clock_diag_pub.published) == 1
    finally:
        _teardown(teensy, client, node)


def test_every_anchor_survives_the_drain_in_order():
    """Two anchors in one drain window → two messages, in order.

    Anchors arrive ~30 s apart, so each one is ~1/120th of an hour's evidence.
    A latest-wins cache (the shape /profile uses, and the wrong shape here)
    would silently delete samples whenever two landed inside one drain period —
    which is exactly what the 500 ms fast-retry cadence produces at boot.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, anchor_seq=1_921)
        _send(teensy, node, anchor_seq=1_922, t_local_us=57_630_123_456,
              err_us=511, freq_ppb=-9_002)
        node._publish_clock_diag()

        published = node.clock_diag_pub.published
        assert len(published) == 2
        assert _kv(published[0])['anchor_seq'] == '1921'
        assert _kv(published[1])['anchor_seq'] == '1922'
        assert _kv(published[1])['err_us'] == '511'
    finally:
        _teardown(teensy, client, node)


def test_an_fw10_bridge_is_silent_not_broken():
    """No 0x8F ever arrives ⇒ empty queue, no message, no exception.

    This is the LIVE state of the bench: FW 11 is committed and deliberately not
    flashed until after the S1 aged-bridge experiment. Every session until then
    runs this exact path, and it has to be quiet — a drain that raised on an
    empty queue, or a publisher that emitted a placeholder, would put a defect
    (or a fabricated sample) into every bag recorded before the flash.
    """
    teensy, client, node = _node()
    try:
        assert node._clock_diag_queue == []
        node._publish_clock_diag()
        assert node.clock_diag_pub.published == []
    finally:
        _teardown(teensy, client, node)


def test_malformed_frame_is_dropped_silently():
    """A short/garbled payload must not raise on the RX thread.

    An exception escaping an RX callback poisons the shared socket thread — the
    one that also carries telemetry, heartbeats and RPC replies. Called directly
    (not over the wire) so the negative is deterministic rather than a race. The
    concrete way this happens in the field: a bridge built against an older
    header sends a short payload, which raises ``struct.error`` — not a
    ``ValueError`` — so a narrow except clause would let it escape.
    """
    teensy, client, node = _node()
    try:
        node._on_clock_diag(int(MsgType.CLOCK_DIAG), 1, b'\x00' * 10, None)
        assert node._clock_diag_queue == []
        node._publish_clock_diag()
        assert node.clock_diag_pub.published == []
    finally:
        _teardown(teensy, client, node)


def test_drain_timer_is_registered():
    """The 1 Hz drain timer exists.

    The mock's create_timer is a no-op MagicMock, so nothing else in this file
    would notice its absence — on hardware the queue would fill to its cap and
    the topic would stay silent while every test here passed. 1 Hz against a
    ~30 s anchor cadence (500 ms at worst, during the boot fast-retry) leaves
    ample headroom.
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
                   if len(a) > 1 and a[1] == node._publish_clock_diag]
        assert periods == [1.0], (
            'the /clock_diag drain is not registered at 1 Hz — '
            f'timer registrations seen: {calls}')
    finally:
        _teardown(teensy, client, node)
