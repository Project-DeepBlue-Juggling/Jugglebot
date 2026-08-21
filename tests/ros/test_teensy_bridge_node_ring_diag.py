"""RING_DIAG uplink → /ring_diag on teensy_bridge_node (can-bridge FW 13).

The can-bridge emits ``MsgType.RING_DIAG`` (0x92) once a second from FW 13
onward: per bus, the CAN RX ring's TRUE occupancy — derived from the ring's
head/tail indices and probed the instant after the drain loop finishes — beside
the ``_available`` count the rest of the firmware believes, the leak's
high-water mark over every 1 kHz service tick, and the hardware FIFO
overflow/warning counts; plus two jugglebot-bus cross-checks (delivery lag off
the FlexCAN hardware capture timestamp, and the hand-sensor SDO round-trip).

WHY IT EXISTS. S2 (2026-08-13) killed the encoder-cache AGE hypothesis — the
cache read FRESHER at 28 h than at 1.3 h — and left the observed mechanism as
~100-150 ms bit-identical per-axis telemetry freezes that the freshness-blind
lead clamp amplifies into commanded stops at every move onset. The surviving
candidate cause is a concurrency defect in the vendored FlexCAN_T4, verified
against this project's toolchain on 2026-08-14: ``events()`` pops the RX ring
BEFORE its ``NVIC_DISABLE_IRQ`` guard, so the CAN ISR's ``_available++`` races
the unguarded task-side ``_available--``. The race is one-directional (the ISR
preempts the task, never the reverse), so ISR increments are swallowed,
``_available`` monotonically UNDER-counts, and the bridge's drain loop exits
believing the ring is empty while a residue D is stranded — after which every
frame it delivers is D frames old, ratcheting with uptime and capping at the
256-slot ring (~114-135 ms at jugglebot-bus rates).

WHY IT COULD NOT BE A FIELD ON /cache_diag. That topic already carries
``rx_depth_hwm`` and ``rx_cap_hits``, but both come from ``getRXQueueCount()``,
which returns ``_available`` — the very count the race corrupts. They are blind
to this failure BY CONSTRUCTION and read healthy through a fully-leaked ring.

What this module pins:

1. **the mapping** — a decoded frame reaches ``/ring_diag`` as a
   ``DiagnosticStatus`` with every wire field present;
2. **the leak is computed and rendered FIRST** — ``leak_jb`` =
   ``true_depth_jb - avail_reported_jb`` is the single number that convicts, and
   it leads the row list rather than hiding among per-bus noise;
3. **the level follows the leak and the FIFO overflow** — and specifically does
   NOT fire on a one-frame reading, which is the probe's own documented
   task-vs-ISR race rather than a defect;
4. **the flags gate the lag rows** — an unseeded arrival clock renders ``n/a``,
   never a 0 that reads as "no lag";
5. **``sdo_rtt_count`` gates the RTT rows** — 0 round trips renders ``n/a``,
   never a 0 µs round trip;
6. **the threading contract** — the RX callback QUEUES and does not publish;
7. **every window survives** — each frame reduces ~1000 probes that exist
   nowhere else;
8. **an FW 12 board is silent, not broken** — no 0x92 ever arrives, the queue
   stays empty, and the drain publishes nothing rather than raising.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticStatus

from teensy_link import MsgType, RingDiag
from teensy_link import protocol as p

from tests.ros._bridge_harness import (
    _build_paired_node,
    _teardown,
    _wait_until,
)

_SEEDED = int(p.RingDiagFlags.LAG_SEEDED)
_RESEEDED = int(p.RingDiagFlags.LAG_RESEED_IN_WINDOW)

# A HEALTHY window: the ring drains to empty, so true depth and the reported
# count agree and the leak is 0. Deliberately non-round elsewhere so a
# transposed field is visible.
_HEALTHY = dict(
    t_local_us=4_680_000_000,          # 1.3 h — the S2 fresh-bridge reference
    true_depth_jb=0, true_depth_bb=0, true_depth_cone=0,
    avail_reported_jb=0, avail_reported_bb=0, avail_reported_cone=0,
    leak_hwm_jb=0, leak_hwm_bb=0, leak_hwm_cone=0,
    true_depth_hwm_jb=4, true_depth_hwm_bb=2, true_depth_hwm_cone=1,
    fifo_overflows_jb=0, fifo_overflows_bb=0, fifo_overflows_cone=0,
    fifo_warns_jb=0, fifo_warns_bb=0, fifo_warns_cone=0,
    probe_ticks=999,
    lag_now_us=41, lag_hwm_us=380,
    cap_span_us=999_959, lag_frames=2_241, lag_reseeds=0,
    sdo_rtt_min_us=1_104, sdo_rtt_max_us=2_870, sdo_rtt_last_us=1_331,
    sdo_rtt_count=50,
    seq=4_680, window_us=1_000_011,
    flags=_SEEDED,
)

# THE CONVICTED window: 131 frames stranded in a ring whose own counter says 0.
_LEAKED = dict(
    _HEALTHY,
    t_local_us=226_800_000_000,        # 63 h — the S1 aged-bridge point
    true_depth_jb=131, avail_reported_jb=0,
    leak_hwm_jb=134, true_depth_hwm_jb=137,
    lag_now_us=131_400, lag_hwm_us=142_900,
    sdo_rtt_min_us=131_910, sdo_rtt_max_us=148_220, sdo_rtt_last_us=132_705,
    seq=226_800,
)


def _node():
    return _build_paired_node(boot_state_read=False)


def _send(teensy, node, base=None, **overrides):
    """Inject one RING_DIAG frame; wait for the RX thread to queue it."""
    before = len(node._ring_diag_queue)
    fields = dict(_HEALTHY if base is None else base)
    fields.update(overrides)
    rd = RingDiag(**fields)
    teensy.send_to_jetson(int(MsgType.RING_DIAG), rd.pack())
    assert _wait_until(lambda: len(node._ring_diag_queue) > before), (
        'RING_DIAG frame never reached the node — is MsgType.RING_DIAG '
        'subscribed?')
    return rd


def _kv(msg):
    return {kv.key: kv.value for kv in msg.values}


def test_ring_diag_frame_publishes_every_wire_field():
    """A decoded RING_DIAG lands on /ring_diag with the whole payload intact.

    Field-by-field rather than spot checks: the frame's three per-bus groups
    repeat the same five field names with different suffixes, so the realistic
    defect is a suffix slip in the renderer (jb's depth published under bb's
    key), which any partial assertion would sail past.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, base=_LEAKED)
        node._publish_ring_diag()
        assert len(node.ring_diag_pub.published) == 1
        kv = _kv(node.ring_diag_pub.published[0])

        assert kv['seq'] == '226800'
        assert kv['t_local_us'] == '226800000000'
        assert kv['window_us'] == '1000011'
        assert kv['probe_ticks'] == '999'
        # Per-bus occupancy, all three buses.
        assert kv['true_depth_jb'] == '131'
        assert kv['true_depth_bb'] == '0'
        assert kv['true_depth_cone'] == '0'
        assert kv['avail_reported_jb'] == '0'
        assert kv['avail_reported_bb'] == '0'
        assert kv['avail_reported_cone'] == '0'
        assert kv['leak_hwm_jb'] == '134'
        assert kv['leak_hwm_bb'] == '0'
        assert kv['leak_hwm_cone'] == '0'
        assert kv['true_depth_hwm_jb'] == '137'
        assert kv['true_depth_hwm_bb'] == '2'
        assert kv['true_depth_hwm_cone'] == '1'
        # Hardware FIFO counters, rendered RAW and cumulative.
        assert kv['fifo_overflows_jb'] == '0'
        assert kv['fifo_warns_jb'] == '0'
        assert kv['fifo_overflows_bb'] == '0'
        assert kv['fifo_warns_cone'] == '0'
        # Delivery lag + its bookkeeping.
        assert kv['lag_seeded'] == '1'
        assert kv['lag_now_us'] == '131400'
        assert kv['lag_hwm_us'] == '142900'
        assert kv['lag_frames'] == '2241'
        assert kv['cap_span_us'] == '999959'
        assert kv['lag_reseeds'] == '0'
        assert kv['lag_reseed_in_window'] == '0'
        # SDO round trip.
        assert kv['sdo_rtt_count'] == '50'
        assert kv['sdo_rtt_min_us'] == '131910'
        assert kv['sdo_rtt_max_us'] == '148220'
        assert kv['sdo_rtt_last_us'] == '132705'
    finally:
        _teardown(teensy, client, node)


def test_the_leak_is_computed_and_rendered_first():
    """``leak_jb`` = true_depth - avail_reported, and it LEADS the row list.

    This is the number the whole hypothesis stands or falls on, and it exists
    nowhere on the wire — the firmware sends the two terms and the subtraction
    happens here, so if this renderer got it wrong the frame would carry the
    evidence and the topic would not show it. Rendering it first is not cosmetic:
    a DiagnosticStatus with two dozen rows is read top-down in a bag viewer, and
    burying the verdict among per-bus rows is how an instrument gets ignored.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, base=_LEAKED)
        node._publish_ring_diag()
        msg = node.ring_diag_pub.published[0]
        assert msg.values[0].key == 'leak_jb'
        assert msg.values[0].value == '131'
        assert msg.values[1].key == 'leak_hwm_jb'
        # And in the headline string, so a console watcher sees it without
        # expanding the KeyValue list.
        assert 'leak_jb=131' in msg.message
        kv = _kv(msg)
        assert kv['leak_bb'] == '0'
        assert kv['leak_cone'] == '0'
    finally:
        _teardown(teensy, client, node)


def test_healthy_window_is_ok_and_reports_a_zero_leak():
    """A ring that drains to empty: depth == avail, leak 0, level OK.

    The negative case matters as much as the positive one — if this WARNed, the
    instrument would convict a healthy bridge and the measurement would be
    worthless in the direction that REFUTES the hypothesis.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node)
        node._publish_ring_diag()
        msg = node.ring_diag_pub.published[0]
        assert msg.level == DiagnosticStatus.OK
        assert _kv(msg)['leak_jb'] == '0'
        assert msg.name == 'teensy/ring_diag'
        assert msg.hardware_id == 'can_bridge_teensy'
    finally:
        _teardown(teensy, client, node)


def test_a_one_frame_reading_does_not_warn_but_a_real_leak_does():
    """The threshold is calibrated on the probe's own race, not picked round.

    The probe reads head/tail and then ``_available`` from task context while the
    CAN ISR can push concurrently — and the accessor's read ordering makes that
    skew UNDER-reporting (head/tail first, ``_available`` last: a straddled push
    can only shrink the reported leak). A reported leak of 1 is therefore
    already real; the threshold of 2 keeps one frame of margin against the sole
    over-report path (the full-ring overwrite edge, unreachable before every
    threshold has long fired). The predicted signature is two orders of
    magnitude above the line, so nothing real is lost by the margin.
    """
    teensy, client, node = _node()
    try:
        # 1 frame: the documented race. Must stay OK.
        _send(teensy, node, true_depth_jb=1, avail_reported_jb=0)
        # 2 frames: still at the boundary, still not a conviction.
        _send(teensy, node, true_depth_jb=2, avail_reported_jb=0, seq=4_681)
        # 3 frames: past anything the race can produce.
        _send(teensy, node, true_depth_jb=3, avail_reported_jb=0, seq=4_682)
        node._publish_ring_diag()
        levels = [m.level for m in node.ring_diag_pub.published]
        assert levels == [DiagnosticStatus.OK, DiagnosticStatus.OK,
                          DiagnosticStatus.WARN]
    finally:
        _teardown(teensy, client, node)


def test_the_high_water_mark_alone_can_raise_the_warning():
    """A leak that has since drained still WARNs, off ``leak_hwm``.

    The instantaneous pair is one sample out of ~1000 probes in the window, so a
    leak that only appears under load could sit between two 1 Hz reads forever.
    The high-water mark is maximised over every service tick since boot and
    cannot be missed that way — so the level has to consider it, or the frame
    would carry the evidence in a field the level ignored.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, true_depth_jb=0, avail_reported_jb=0,
              leak_hwm_jb=97)
        node._publish_ring_diag()
        msg = node.ring_diag_pub.published[0]
        assert msg.level == DiagnosticStatus.WARN
        assert _kv(msg)['leak_jb'] == '0'
        assert _kv(msg)['leak_hwm_jb'] == '97'
    finally:
        _teardown(teensy, client, node)


def test_a_fifo_overflow_warns_even_with_a_clean_ring():
    """A hardware FIFO overflow is a LOST frame, and it warns on its own.

    Not redundant with the leak: a leaked frame is merely LATE, an overflowed one
    is GONE — dropped inside the peripheral, upstream of the software ring and of
    every counter the bridge kept before FW 13's vendored patch counted it. It is
    both the worse failure and the one that was completely invisible, so it must
    not need a concurrent leak to be surfaced.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, fifo_overflows_jb=1)
        _send(teensy, node, fifo_overflows_cone=3, seq=4_681)
        node._publish_ring_diag()
        published = node.ring_diag_pub.published
        assert [m.level for m in published] == [DiagnosticStatus.WARN,
                                                DiagnosticStatus.WARN]
        assert _kv(published[0])['leak_jb'] == '0'  # clean ring, still WARN
        assert _kv(published[1])['fifo_overflows_cone'] == '3'
    finally:
        _teardown(teensy, client, node)


def test_an_unseeded_arrival_clock_renders_the_lag_as_na():
    """No reference ⇒ ``n/a``, never a 0 that reads as "no delivery lag".

    The wire carries 0 in the lag fields before the arrival clock is seeded (and
    the firmware sets them to 0 deliberately, rather than shipping an
    uninitialised value). A bare ``0`` in a bag is indistinguishable from a
    measured zero lag — the healthiest possible reading — which is the one thing
    this frame must never fabricate. ``flags`` is the positive assertion that
    resolves it, the same discipline ClockDiag's ``FREQ_VALID`` uses.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, flags=0, lag_now_us=0, lag_hwm_us=0, lag_frames=0)
        node._publish_ring_diag()
        kv = _kv(node.ring_diag_pub.published[0])
        assert kv['lag_seeded'] == '0'
        assert kv['lag_now_us'] == 'n/a'
        assert kv['lag_hwm_us'] == 'n/a'
        # lag_frames is NOT blanked: 0 there is a measurement (nothing arrived
        # this window), which is itself diagnostic.
        assert kv['lag_frames'] == '0'
        assert 'lag_now=n/aus' in node.ring_diag_pub.published[0].message
    finally:
        _teardown(teensy, client, node)


def test_a_reseed_is_surfaced_per_window_not_only_cumulatively():
    """A reseed moves the lag series' ZERO, so the consumer must segment there.

    ``lag_reseeds`` is cumulative and says a reseed happened at some point;
    ``lag_reseed_in_window`` says it happened in THIS sample, which is the only
    form that tells a trend fit where to cut. Without it, a reseed's step in
    ``lag_now_us`` reads as a sudden improvement in delivery lag.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, flags=_SEEDED | _RESEEDED, lag_reseeds=1,
              cap_span_us=0)
        node._publish_ring_diag()
        kv = _kv(node.ring_diag_pub.published[0])
        assert kv['lag_reseed_in_window'] == '1'
        assert kv['lag_reseeds'] == '1'
        assert kv['lag_seeded'] == '1'
        # Still seeded, so the lag itself is a real (post-reseed) measurement.
        assert kv['lag_now_us'] == '41'
    finally:
        _teardown(teensy, client, node)


def test_zero_sdo_round_trips_blank_the_rtt_rows():
    """``sdo_rtt_count == 0`` ⇒ the three RTT rows render ``n/a``.

    A 0 µs round trip is not a thing that can happen, so publishing one would be
    a fabricated measurement. Count-zero is the live state whenever the hand
    ball-sensor poller is toggled off, parked on its version gate, or timing out
    — none of which are the same as a fast bus, and all of which the RTT
    cross-check must refuse to answer for.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, sdo_rtt_count=0, sdo_rtt_min_us=0,
              sdo_rtt_max_us=0, sdo_rtt_last_us=0)
        node._publish_ring_diag()
        kv = _kv(node.ring_diag_pub.published[0])
        assert kv['sdo_rtt_count'] == '0'
        assert kv['sdo_rtt_min_us'] == 'n/a'
        assert kv['sdo_rtt_max_us'] == 'n/a'
        assert kv['sdo_rtt_last_us'] == 'n/a'
    finally:
        _teardown(teensy, client, node)


def test_a_negative_leak_renders_as_zero_not_as_a_negative_number():
    """The probe's ±1 race can read avail one ahead of depth; clamp it.

    The mechanism cannot produce a negative leak — a ring cannot hold fewer
    frames than its counter claims for any reason other than the probe catching
    a push mid-flight. Rendering ``-1`` would put a number in the bag that an
    operator would try to explain, so it clamps to 0 and stays out of the way.
    The raw terms are still published beside it, so nothing is hidden.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, true_depth_jb=0, avail_reported_jb=1)
        node._publish_ring_diag()
        msg = node.ring_diag_pub.published[0]
        kv = _kv(msg)
        assert kv['leak_jb'] == '0'
        assert msg.level == DiagnosticStatus.OK
        # The unclamped terms remain visible.
        assert kv['true_depth_jb'] == '0'
        assert kv['avail_reported_jb'] == '1'
    finally:
        _teardown(teensy, client, node)


def test_rx_callback_queues_and_does_not_publish():
    """The RX thread never publishes; the executor-side drain does.

    Driven through the REAL TeensyLinkClient RX thread, so this is the
    production path. Publishing here would put DDS work on the socket thread
    that also carries the 100 Hz telemetry and every RPC reply. The drain must
    also CLEAR the queue, or every window would be republished once a second
    forever and a trend fit would see each sample N times.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node)
        assert node.ring_diag_pub.published == [], (
            'the RX callback published — publishing must happen on the '
            'executor thread (drain timer), never on the socket RX thread')

        node._publish_ring_diag()
        assert len(node.ring_diag_pub.published) == 1
        assert node._ring_diag_queue == []

        # A second drain with nothing queued is a no-op, not a repeat.
        node._publish_ring_diag()
        assert len(node.ring_diag_pub.published) == 1
    finally:
        _teardown(teensy, client, node)


def test_every_window_survives_the_drain_in_order():
    """Two windows in one drain period → two messages, in order.

    Each frame reduces ~1000 on-chip service-tick probes that exist nowhere else
    once the window closes, so a latest-wins cache (the shape ``/profile`` uses,
    and the wrong shape here) would delete a second of evidence whenever the emit
    and the drain beat against each other — which two 1 Hz timers with no phase
    relationship do routinely. The verdict is a ratchet over hours; holes in the
    series are exactly what a ratchet fit cannot tolerate.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, seq=4_680)
        _send(teensy, node, seq=4_681, true_depth_jb=7)
        node._publish_ring_diag()

        published = node.ring_diag_pub.published
        assert len(published) == 2
        assert _kv(published[0])['seq'] == '4680'
        assert _kv(published[1])['seq'] == '4681'
        assert _kv(published[1])['true_depth_jb'] == '7'
    finally:
        _teardown(teensy, client, node)


def test_an_fw12_bridge_is_silent_not_broken():
    """No 0x92 ever arrives ⇒ empty queue, no message, no exception.

    This is the LIVE state of the bench until the operator flashes FW 13. Every
    session until then runs this exact path and it has to be quiet — a drain that
    raised on an empty queue, or a publisher that emitted a placeholder window,
    would put a defect (or a fabricated sample) into every bag recorded before
    the flash.
    """
    teensy, client, node = _node()
    try:
        assert node._ring_diag_queue == []
        node._publish_ring_diag()
        assert node.ring_diag_pub.published == []
    finally:
        _teardown(teensy, client, node)


def test_malformed_frame_is_dropped_silently():
    """A short/garbled payload must not raise on the RX thread.

    An exception escaping an RX callback poisons the shared socket thread — the
    one that also carries telemetry, heartbeats and RPC replies. Called directly
    (not over the wire) so the negative is deterministic rather than a race. The
    concrete field case: a bridge built against an older header sends a short
    payload, which raises ``struct.error`` — NOT a ``ValueError`` — so a narrow
    except clause would let it escape.
    """
    teensy, client, node = _node()
    try:
        node._on_ring_diag(int(MsgType.RING_DIAG), 1, b'\x00' * 10, None)
        assert node._ring_diag_queue == []
        node._publish_ring_diag()
        assert node.ring_diag_pub.published == []
    finally:
        _teardown(teensy, client, node)


def test_drain_timer_is_registered():
    """The 1 Hz drain timer exists.

    The mock's create_timer is a no-op MagicMock, so nothing else in this file
    would notice its absence — on hardware the queue would fill to its cap and
    the topic would stay silent while every test here passed. 1 Hz matches the
    frame's own emit cadence, so at most one window is ever waiting.
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
                   if len(a) > 1 and a[1] == node._publish_ring_diag]
        assert periods == [1.0], (
            'the /ring_diag drain is not registered at 1 Hz — '
            f'timer registrations seen: {calls}')
    finally:
        _teardown(teensy, client, node)
