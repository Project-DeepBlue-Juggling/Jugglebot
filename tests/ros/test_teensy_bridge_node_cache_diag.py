"""CACHE_DIAG uplink → /cache_diag on teensy_bridge_node (can-bridge FW 12).

The can-bridge emits ``MsgType.CACHE_DIAG`` (0x91) once a second from FW 12
onward: the per-axis ENCODER-CACHE AGE floor and peak over the window just
closed (reduced on-chip from one sample per 100 Hz telemetry tick), the
seen-mask that says which axes have ever been cached at all, the window length
and sample count, and the CAN RX-ring depth/cap-hit and decode-discard counters
that have existed on the Teensy since the 2026-06-04 drain-to-empty fix and the
2026-07-05 marginal-CAN3 work but were never uplinked.

WHY IT EXISTS. The S1 aged-bridge experiment (2026-08-12) localized the uptime
command-latency drift of
``logbook/2026-07-18-teensy-uptime-tracking-degradation.md`` to the Teensy: at
63 h of uptime the end-to-end leg lag is 290-340 ms with the ``leg_interp`` lead
clamp pinning the executed command at ``fb + MAX_LEAD_REV`` on 44-74 % of ticks
(5.8 % fresh), while RTT is flat at 1-3 ms, ``interp_deadline_misses`` is 0, the
heap is flat, CAN throughput is flat, and the ODrives and the Jetson-side link
are exonerated. Two candidates survive: the encoder cache the lead clamp
measures ``fb`` against is going stale, or the leg genuinely trails.
``/robot_state`` and ``/leg_cmd_executed`` both read that same cache, so under a
stale cache they move together and are indistinguishable from real lag.
``age_min_us`` is the one observable that separates them.

The bag forensics then sharpened the question: the per-axis cache VALUE stalls
for 30-500 ms in a fat tail (9-18 % of refresh intervals > 30 ms aged against
4.3 % fresh) while every AGGREGATE CAN RX counter stays flat and the uplink
cadence stays perfect — because an aggregate cannot see one axis of seven go
quiet. ``enc_frames`` is the per-axis counter that closes it: differenced across
a stall, still advancing ⇒ frames arrived and the decode ran; paused ⇒ nothing
arrived at all.

What this module pins:

1. **the mapping** — a decoded frame reaches ``/cache_diag`` as a
   ``DiagnosticStatus`` with every wire field present, per-axis ages as
   individually-readable rows;
2. **the seen_mask contract** — an axis that has never been cached renders
   ``n/a`` rather than the u32 saturation rail, and never raises the level. A
   bare ``4294967295`` in a bag reads as a measured age, and an absent axis
   (the single-leg bench rig, a dark hand ODrive) warning forever would train
   the operator to ignore the row;
3. **the level follows the FLOOR, not the peak** — the peak rises with ordinary
   jitter and a missed frame; only the floor can be raised by the cache
   actually not being written;
3b. **enc_frames splits a stall's two causes**, is rendered RAW and cumulative
   (the consumer differences it), and is NEVER blanked by ``seen_mask`` — 0 is
   a measurement there, not a missing one;
4. **the threading contract** — the RX callback QUEUES and does not publish;
5. **every window survives** — each frame is a reduction over ~100 samples that
   exist nowhere else;
6. **an FW 11 board is silent, not broken** — no 0x91 ever arrives, the queue
   stays empty, and the drain publishes nothing rather than raising.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

from teensy_link import CacheDiag, MsgType

from tests.ros._bridge_harness import (
    _build_paired_node,
    _teardown,
    _wait_until,
)

_RAIL = 0xFFFF_FFFF

# A realistic HEALTHY window at the S1 aged-bridge point: 63 h of uptime, a full
# 1 s window of 100 samples, ages sitting at the ODrive broadcast period. If the
# stale-cache hypothesis is FALSE this is what a degraded session looks like too
# — which is exactly why the frame is worth having. Deliberately non-round
# numbers, so a transposed field or an off-by-one array slice is visible.
_HEALTHY = dict(
    t_local_us=226_800_000_000,        # 63 h
    age_min_us=(101, 202, 303, 404, 505, 606, 707),
    age_max_us=(3_701, 3_802, 3_903, 4_004, 4_105, 4_206, 4_307),
    # Cumulative since boot: ~272 frames/s/axis x 63 h ~= 6.17e7, staggered per
    # axis so a transposed slot is visible.
    enc_frames=(61_000_001, 61_000_002, 61_000_003, 61_000_004,
                61_000_005, 61_000_006, 61_000_007),
    seq=3_600,
    window_us=1_000_013,
    rx_cap_hits_jb=0,
    rx_cap_hits_bb=7,
    rx_cap_hits_cone=13,
    decode_short=2,
    decode_bad_axis=5,
    rx_depth_hwm_jb=9,
    rx_depth_hwm_bb=3,
    rx_depth_hwm_cone=1,
    samples=100,
    seen_mask=0x7F,                    # all six legs + the hand
)


def _node():
    return _build_paired_node(boot_state_read=False)


def _send(teensy, node, **overrides):
    """Inject one CACHE_DIAG frame; wait for the RX thread to queue it."""
    before = len(node._cache_diag_queue)
    fields = dict(_HEALTHY)
    fields.update(overrides)
    cd = CacheDiag(**fields)
    teensy.send_to_jetson(int(MsgType.CACHE_DIAG), cd.pack())
    assert _wait_until(lambda: len(node._cache_diag_queue) > before), (
        'CACHE_DIAG frame never reached the node — is MsgType.CACHE_DIAG '
        'subscribed?')
    return cd


def _kv(msg):
    return {kv.key: kv.value for kv in msg.values}


def test_cache_diag_frame_publishes_every_wire_field():
    """A decoded CACHE_DIAG lands on /cache_diag with the whole payload intact.

    Every field is asserted individually, and every array slot separately. The
    two 7-wide age arrays are adjacent on the wire, so an off-by-one slice would
    attribute each leg's age to its neighbour and still look entirely plausible
    in a plot — the analysis that follows is per-leg, because the lead clamp is
    per-leg.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node)
        node._publish_cache_diag()

        published = node.cache_diag_pub.published
        assert len(published) == 1
        msg = published[0]
        assert msg.name == 'teensy/cache_diag'
        vals = _kv(msg)
        assert vals['t_local_us'] == '226800000000'
        assert vals['seq'] == '3600'
        assert vals['window_us'] == '1000013'
        assert vals['samples'] == '100'
        assert vals['seen_mask'] == '127'
        for i, expected in enumerate((101, 202, 303, 404, 505, 606, 707)):
            assert vals[f'age_min_us_{i}'] == str(expected)
        for i, expected in enumerate((3_701, 3_802, 3_903, 4_004, 4_105,
                                      4_206, 4_307)):
            assert vals[f'age_max_us_{i}'] == str(expected)
        for i, expected in enumerate((61_000_001, 61_000_002, 61_000_003,
                                      61_000_004, 61_000_005, 61_000_006,
                                      61_000_007)):
            assert vals[f'enc_frames_{i}'] == str(expected)
        assert vals['rx_depth_hwm_jb'] == '9'
        assert vals['rx_depth_hwm_bb'] == '3'
        assert vals['rx_depth_hwm_cone'] == '1'
        assert vals['rx_cap_hits_jb'] == '0'
        assert vals['rx_cap_hits_bb'] == '7'
        assert vals['rx_cap_hits_cone'] == '13'
        assert vals['decode_short'] == '2'
        assert vals['decode_bad_axis'] == '5'
        # The one derived row, so an operator watching a soak has a single
        # number to watch instead of seven.
        assert vals['worst_age_floor_us'] == '707'
    finally:
        _teardown(teensy, client, node)


def test_a_healthy_window_is_ok_and_a_raised_floor_warns():
    """The level follows the FLOOR, and only the floor.

    A peak can be raised by ordinary jitter, a single dropped CAN frame, or a
    late telemetry tick. A MINIMUM over ~100 samples cannot: the only way every
    sample in a second is old is that the cache was not written. So the level
    keys on ``age_min_us`` while ``age_max_us`` rides along as evidence, and a
    window with a healthy floor stays OK no matter how tall its peak is.
    """
    teensy, client, node = _node()
    try:
        from diagnostic_msgs.msg import DiagnosticStatus

        # Healthy: floor at the broadcast period.
        _send(teensy, node)
        # Healthy floor, TALL peak — one gap, not a stale cache. Still OK.
        _send(teensy, node, seq=3_601,
              age_max_us=(90_000,) * 7)
        # The stale-cache signature: the FLOOR itself has moved to the lag class
        # S1 measured, on one leg. WARN.
        _send(teensy, node, seq=3_602,
              age_min_us=(101, 202, 303, 404, 505, 310_000, 707))
        node._publish_cache_diag()

        healthy, spiky, stale = node.cache_diag_pub.published
        assert healthy.level == DiagnosticStatus.OK
        assert spiky.level == DiagnosticStatus.OK, (
            'a tall PEAK with a healthy floor is a transport gap, not a stale '
            'cache — warning on it would bury the signal this topic exists for')
        assert stale.level == DiagnosticStatus.WARN
        assert _kv(stale)['worst_age_floor_us'] == '310000'
        assert _kv(stale)['age_min_us_5'] == '310000'
    finally:
        _teardown(teensy, client, node)


def test_enc_frames_split_arrived_from_never_arrived_across_a_stall():
    """The two halves of a value-stall are distinguishable from the bag alone.

    This is the discriminator the 2026-08-12 bag forensics asked for. The stall
    signature is a per-axis cache VALUE frozen for 30-500 ms while every
    aggregate CAN RX counter stays flat — which an aggregate genuinely cannot
    resolve, because six healthy axes keep the total moving.

    Two consecutive windows are published here with leg 3 stalling in both, and
    the counters differenced exactly as a consumer would. Leg 3's count is
    FROZEN across the pair (nothing arrived — the ODrive's broadcast or a
    per-axis bus loss) while leg 4's advances at the healthy rate despite an
    equally raised age (frames arrived and the decode ran, so the frozen value
    came in over the wire). Those are different faults with different owners,
    and this is the only field that tells them apart.
    """
    teensy, client, node = _node()
    try:
        base = (61_000_000,) * 7
        _send(teensy, node, seq=3_600,
              age_min_us=(101, 202, 303, 310_000, 305_000, 606, 707),
              enc_frames=base)
        # One second later: axis 3 received NOTHING, axis 4 received a healthy
        # ~272 frames, everything else likewise.
        nxt = tuple(v + 272 for v in base)
        nxt = nxt[:3] + (base[3],) + nxt[4:]
        _send(teensy, node, seq=3_601,
              age_min_us=(101, 202, 303, 310_000, 305_000, 606, 707),
              enc_frames=nxt)
        node._publish_cache_diag()

        first, second = node.cache_diag_pub.published
        a, b = _kv(first), _kv(second)

        def delta(key):
            return int(b[key]) - int(a[key])

        # Axis 3: stalled value AND stalled counter ⇒ upstream (nothing arrived).
        assert delta('enc_frames_3') == 0
        # Axis 4: stalled value but the counter kept advancing ⇒ frames arrived
        # and the cache write completed, so the stale value came in on the wire.
        assert delta('enc_frames_4') == 272
        # A healthy axis for contrast — the aggregate view that hid all of this.
        assert delta('enc_frames_0') == 272
        # Both stalled axes look IDENTICAL on every other field, which is the
        # whole point: without enc_frames the bag cannot separate them.
        assert a['age_min_us_3'] != 'n/a' and a['age_min_us_4'] != 'n/a'
        assert b['rx_cap_hits_jb'] == a['rx_cap_hits_jb']
        assert b['decode_short'] == a['decode_short']
    finally:
        _teardown(teensy, client, node)


def test_enc_frames_are_raw_cumulative_and_never_blanked_by_seen_mask():
    """A zero frame count is a MEASUREMENT, so it is never rendered ``n/a``.

    The ``n/a`` discipline exists because an unseen axis's AGE is the u32
    saturation rail — a missing measurement dressed as a number. A frame count
    has no such rail: 0 means "this axis received nothing", which is the single
    most diagnostic value the field can carry and the upstream half of the split
    above. Blanking it on the same mask bit would erase the answer precisely in
    the case the operator most needs it.

    They are also rendered RAW rather than as a node-computed rate: the node
    holds no previous sample, so a node-side delta would silently widen across a
    dropped frame — the failure ``seq`` exists to expose, not to hide.
    """
    teensy, client, node = _node()
    try:
        # Bench rig: only leg 0 on the bus, so axes 1-6 are outside seen_mask AND
        # have never received a frame.
        _send(teensy, node,
              age_min_us=(101,) + (_RAIL,) * 6,
              age_max_us=(3_701,) + (_RAIL,) * 6,
              enc_frames=(61_000_001, 0, 0, 0, 0, 0, 0),
              seen_mask=0x01)
        node._publish_cache_diag()

        vals = _kv(node.cache_diag_pub.published[0])
        assert vals['enc_frames_0'] == '61000001'
        for i in range(1, 7):
            # Blanked ages, but a real zero count beside them.
            assert vals[f'age_min_us_{i}'] == 'n/a'
            assert vals[f'enc_frames_{i}'] == '0'
    finally:
        _teardown(teensy, client, node)


def test_an_unseen_axis_renders_na_and_never_warns():
    """seen_mask clear ⇒ ``n/a``, and no WARN.

    The wire carries the u32 saturation rail for an axis whose cache has never
    been written — the single-leg bench rig, a dark hand ODrive, or any frame
    emitted before the first encoder broadcast lands. That rail is honest but
    reads exactly like a catastrophically stale cache, so rendering it as a
    number would put a fabricated 4294967295 µs age into every bench bag and a
    permanent WARN on the row. One mask bit closes both.
    """
    teensy, client, node = _node()
    try:
        from diagnostic_msgs.msg import DiagnosticStatus

        # Bench rig: only leg 0 is on the bus.
        _send(teensy, node,
              age_min_us=(101,) + (_RAIL,) * 6,
              age_max_us=(3_701,) + (_RAIL,) * 6,
              seen_mask=0x01)
        node._publish_cache_diag()

        msg = node.cache_diag_pub.published[0]
        vals = _kv(msg)
        assert vals['age_min_us_0'] == '101'
        assert vals['age_max_us_0'] == '3701'
        for i in range(1, 7):
            assert vals[f'age_min_us_{i}'] == 'n/a'
            assert vals[f'age_max_us_{i}'] == 'n/a'
        assert vals['worst_age_floor_us'] == '101'
        assert msg.level == DiagnosticStatus.OK
    finally:
        _teardown(teensy, client, node)


def test_a_frame_with_nothing_cached_at_all_is_ok_not_a_fault():
    """seen_mask == 0 is the pre-arm shape, and must not WARN.

    The FIRST frame of a session can legitimately arrive before any ODrive has
    broadcast an encoder estimate — every age is the rail and the mask is empty.
    A level derived by taking a max over an empty set has to have a defined
    answer, and the honest one is OK: the frame is reporting "no measurement
    yet", not "the cache is 71 minutes stale".
    """
    teensy, client, node = _node()
    try:
        from diagnostic_msgs.msg import DiagnosticStatus

        _send(teensy, node, seq=1, window_us=0, samples=1,
              age_min_us=(_RAIL,) * 7, age_max_us=(_RAIL,) * 7, seen_mask=0)
        node._publish_cache_diag()

        msg = node.cache_diag_pub.published[0]
        assert msg.level == DiagnosticStatus.OK
        vals = _kv(msg)
        assert vals['seen_mask'] == '0'
        assert vals['worst_age_floor_us'] == '0'
        assert all(vals[f'age_min_us_{i}'] == 'n/a' for i in range(7))
        # window_us 0 with samples 1 is the honest first-frame shape (a
        # zero-length window closed by its own single sample), not a defect.
        assert vals['window_us'] == '0'
        assert vals['samples'] == '1'
    finally:
        _teardown(teensy, client, node)


def test_the_hand_axis_is_carried_and_judged_like_a_leg():
    """Axis 6 is the hand, and it is not a second-class row.

    The hand shares ``axes[]`` with the legs, and the 2026-07-28 anomaly-fixes
    sitting measured the same uptime curve on the hand DISPATCH shift
    (+14.5 ms at 0.24 h → +54-63 ms at 1.57-1.74 h). A census that stopped at
    leg 5 would have excluded the axis that first showed the drift escaping the
    leg path.
    """
    teensy, client, node = _node()
    try:
        from diagnostic_msgs.msg import DiagnosticStatus

        _send(teensy, node,
              age_min_us=(101, 202, 303, 404, 505, 606, 275_000))
        node._publish_cache_diag()

        msg = node.cache_diag_pub.published[0]
        assert _kv(msg)['age_min_us_6'] == '275000'
        assert _kv(msg)['worst_age_floor_us'] == '275000'
        assert msg.level == DiagnosticStatus.WARN
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
        assert node.cache_diag_pub.published == [], (
            'the RX callback published — publishing must happen on the '
            'executor thread (drain timer), never on the socket RX thread')

        node._publish_cache_diag()
        assert len(node.cache_diag_pub.published) == 1
        assert node._cache_diag_queue == []

        # A second drain with nothing queued is a no-op, not a repeat.
        node._publish_cache_diag()
        assert len(node.cache_diag_pub.published) == 1
    finally:
        _teardown(teensy, client, node)


def test_every_window_survives_the_drain_in_order():
    """Two windows in one drain period → two messages, in order.

    Each frame is a reduction over ~100 on-chip samples that exist nowhere else
    once the window closes, so a latest-wins cache (the shape ``/profile`` uses,
    and the wrong shape here) would delete a second of evidence whenever the
    emit and the drain beat against each other — which two 1 Hz timers with no
    phase relationship do routinely.
    """
    teensy, client, node = _node()
    try:
        _send(teensy, node, seq=3_600)
        _send(teensy, node, seq=3_601, t_local_us=226_801_000_000,
              age_min_us=(111, 212, 313, 414, 515, 616, 717))
        node._publish_cache_diag()

        published = node.cache_diag_pub.published
        assert len(published) == 2
        assert _kv(published[0])['seq'] == '3600'
        assert _kv(published[1])['seq'] == '3601'
        assert _kv(published[1])['age_min_us_0'] == '111'
    finally:
        _teardown(teensy, client, node)


def test_an_fw11_bridge_is_silent_not_broken():
    """No 0x91 ever arrives ⇒ empty queue, no message, no exception.

    This is the LIVE state of the bench until the operator flashes FW 12. Every
    session until then runs this exact path and it has to be quiet — a drain
    that raised on an empty queue, or a publisher that emitted a placeholder
    window, would put a defect (or a fabricated sample) into every bag recorded
    before the flash.
    """
    teensy, client, node = _node()
    try:
        assert node._cache_diag_queue == []
        node._publish_cache_diag()
        assert node.cache_diag_pub.published == []
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
        node._on_cache_diag(int(MsgType.CACHE_DIAG), 1, b'\x00' * 10, None)
        assert node._cache_diag_queue == []
        node._publish_cache_diag()
        assert node.cache_diag_pub.published == []
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
                   if len(a) > 1 and a[1] == node._publish_cache_diag]
        assert periods == [1.0], (
            'the /cache_diag drain is not registered at 1 Hz — '
            f'timer registrations seen: {calls}')
    finally:
        _teardown(teensy, client, node)
