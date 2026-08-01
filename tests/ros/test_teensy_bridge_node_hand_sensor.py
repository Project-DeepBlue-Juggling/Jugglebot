"""Hand ball-present sensor surface on teensy_bridge_node (plan Phase 5).

The can-bridge polls the hand ODrive's G02 over an SDO pair and uplinks the
decoded cache as ``MsgType.HAND_SENSOR``. This module drives that frame through
the real ``TeensyLinkClient`` RX thread into the node and asserts the two ROS
observables:

* ``/hand_telemetry`` gains ``ball_held`` / ``ball_held_raw`` /
  ``ball_held_valid`` / ``ball_held_stamp``;
* ``/link_status`` gains the ``hand_ball_sensor`` row (state word + miss count
  + the RAW ``get_gpio_states`` word in hex — the Phase 7 step 2 gate's
  observable).

The signal is TRI-STATE by contract (plans/active/hand-ball-sensor.md
§ Architecture): ``ball_held`` means nothing unless ``ball_held_valid``, and
UNKNOWN must NEVER render as "empty". Three distinct ways to be not-valid are
covered here and all three must yield ``ball_held_valid is False``:

1. **stale-by-flags** — the bridge itself won't vouch for the reading
   (``VALID`` clear, or ``TIME_SYNCED`` clear so ``t_bridge_us`` is unanchored);
2. **stale-by-RX-age** — a good frame arrived and then the FRAME SOURCE died.
   The publisher is a free-running 100 Hz timer over a cache, so without this
   gate a dead bridge republishes "ball held, valid" forever. Faked by
   backdating the node's host-monotonic RX stamp past ``_HAND_SENSOR_RX_FRESH_S``
   (same technique as ``test_teensy_bridge_node_bb.py``'s ``_latest_bb_est_mono
   -= 10.0``) — the gate reads ``time.monotonic()``, never the wire stamp;
3. **never-seen** — boot, or a bridge running firmware older than the Phase 3/4
   poller, which never sends the frame at all.

ROS 2 is mocked by tests/ros/conftest.py.
"""

from __future__ import annotations

from teensy_link import HandSensor, MsgType
from teensy_link import protocol as p

from jugglebot.teensy_bridge_node import _HAND_SENSOR_RX_FRESH_S

from tests.ros._bridge_harness import (
    _build_paired_node,
    _link_kv,
    _teardown,
    _wait_until,
)


_RAW = int(p.HandSensorFlags.RAW_HELD)
_DEB = int(p.HandSensorFlags.DEBOUNCED_HELD)
_VALID = int(p.HandSensorFlags.VALID)
_STALE = int(p.HandSensorFlags.STALE)
_SYNCED = int(p.HandSensorFlags.TIME_SYNCED)

_FRESH_HELD = _RAW | _DEB | _VALID | _SYNCED

# A plausible bridge wall-clock stamp: Jetson-epoch microseconds.
_T_US = 1_500_000_123_456_789
_T_SEC = 1_500_000_123
_T_NANOSEC = 456_789_000

# G02 is bit 2 of get_gpio_states, ACTIVE-LOW (a seated ball shorts the pin to
# GND): held ⇒ bit 2 CLEAR, empty ⇒ bit 2 SET — the word the Phase 7 gate
# watches (gpio_poll.cpp decodes raw_held = !((states >> pin) & 1)).
_RAW_WORD_HELD = 0x00000000
_RAW_WORD_EMPTY = 0x00000004


def _node():
    teensy, client, node = _build_paired_node(boot_state_read=False)
    return teensy, client, node


def _send_sensor(teensy, node, flags, *, raw=_RAW_WORD_HELD, miss=0, t_us=_T_US):
    """Inject one HAND_SENSOR frame and wait for the RX thread to cache it."""
    hs = HandSensor(t_bridge_us=t_us, raw_states=raw, flags=flags, miss_count=miss)
    teensy.send_to_jetson(int(MsgType.HAND_SENSOR), hs.pack())
    assert _wait_until(lambda: node._latest_hand_sensor is not None
                       and int(node._latest_hand_sensor.flags) == flags
                       and int(node._latest_hand_sensor.miss_count) == miss)


def _hand_msg(teensy, node):
    """Publish one /hand_telemetry message and return it (needs a Telemetry frame:
    the publisher no-ops until axis 6's measured side exists)."""
    teensy.send_telemetry()
    assert _wait_until(lambda: node._latest_telemetry is not None)
    node._publish_hand_telemetry()
    return node.hand_telemetry_pub.published[-1]


def _age_out(node):
    """Fake RX age: backdate the host-monotonic arrival stamp past the window."""
    with node._lock:
        node._latest_hand_sensor_mono -= (_HAND_SENSOR_RX_FRESH_S + 1.0)


# ── /hand_telemetry ────────────────────────────────────────────

def test_fresh_frame_populates_hand_telemetry():
    """A frame the bridge vouches for (VALID + TIME_SYNCED), freshly arrived →
    all four fields populated, ball_held_valid True, stamp converted from
    t_bridge_us with NO Jetson-side offset (the bridge is the sync master)."""
    teensy, client, node = _node()
    try:
        _send_sensor(teensy, node, _FRESH_HELD)
        msg = _hand_msg(teensy, node)
        assert msg.ball_held is True
        assert msg.ball_held_raw is True
        assert msg.ball_held_valid is True
        assert msg.ball_held_stamp.sec == _T_SEC
        assert msg.ball_held_stamp.nanosec == _T_NANOSEC
    finally:
        _teardown(teensy, client, node)


def test_raw_and_debounced_bits_are_independent():
    """The raw per-sample bit survives to ROS separately from the debounced
    verdict (approved decision 4): a mid-carry contact dropout shows as
    raw=False while the debounced verdict still holds."""
    teensy, client, node = _node()
    try:
        _send_sensor(teensy, node, _DEB | _VALID | _SYNCED,
                     raw=_RAW_WORD_EMPTY)   # raw bit clear ⇒ bit 2 SET (active-low)
        msg = _hand_msg(teensy, node)
        assert msg.ball_held is True
        assert msg.ball_held_raw is False
        assert msg.ball_held_valid is True
    finally:
        _teardown(teensy, client, node)


# ── Not-valid case 1: stale-by-flags ───────────────────────────

def test_valid_flag_clear_is_not_valid():
    """VALID clear = the bridge's own cache is UNKNOWN (no gated good reply)."""
    teensy, client, node = _node()
    try:
        _send_sensor(teensy, node, _RAW | _DEB | _SYNCED)
        assert _hand_msg(teensy, node).ball_held_valid is False
    finally:
        _teardown(teensy, client, node)


def test_time_synced_clear_is_not_valid():
    """TIME_SYNCED clear = the bridge's wall anchor is unset, so t_bridge_us is
    meaningless — the reading is not trustworthy even though VALID is set."""
    teensy, client, node = _node()
    try:
        _send_sensor(teensy, node, _RAW | _DEB | _VALID)
        assert _hand_msg(teensy, node).ball_held_valid is False
    finally:
        _teardown(teensy, client, node)


# ── Not-valid case 2: stale-by-RX-age ──────────────────────────

def test_rx_age_beyond_window_is_not_valid():
    """Good frame, then the frame source dies: past _HAND_SENSOR_RX_FRESH_S on
    the HOST monotonic clock the cached reading stops being valid, even though
    the frame's own flags still say VALID + TIME_SYNCED. Without this the
    free-running publisher would republish 'held, valid' forever."""
    teensy, client, node = _node()
    try:
        _send_sensor(teensy, node, _FRESH_HELD)
        assert _hand_msg(teensy, node).ball_held_valid is True
        _age_out(node)
        msg = _hand_msg(teensy, node)
        assert msg.ball_held_valid is False
        # The last-known bits and stamp are retained as diagnostics — the
        # validity flag, not a wiped payload, is what marks them untrustworthy.
        assert msg.ball_held is True
        assert msg.ball_held_stamp.sec == _T_SEC
    finally:
        _teardown(teensy, client, node)


# ── Not-valid case 3: never seen ───────────────────────────────

def test_never_seen_is_not_valid_with_zero_stamp():
    """No HAND_SENSOR ever arrived (boot, or a bridge older than the Phase 3/4
    poller) → UNKNOWN: every bool False and a ZEROED stamp, so a consumer can
    never mistake silence for a confident reading."""
    teensy, client, node = _node()
    try:
        msg = _hand_msg(teensy, node)
        assert msg.ball_held_valid is False
        assert msg.ball_held is False
        assert msg.ball_held_raw is False
        assert msg.ball_held_stamp.sec == 0
        assert msg.ball_held_stamp.nanosec == 0
    finally:
        _teardown(teensy, client, node)


# ── /link_status hand_ball_sensor row ──────────────────────────

def test_link_status_never_seen_renders_unknown_not_empty():
    """Never-seen renders explicitly as unknown — and MUST NOT read as 'empty'
    (§ Architecture tri-state: no frame is 'we don't know', not 'no ball')."""
    teensy, client, node = _node()
    try:
        value = _link_kv(node)['hand_ball_sensor']
        assert value == 'unknown (never seen)'
    finally:
        _teardown(teensy, client, node)


def test_link_status_held_carries_miss_count_and_raw_hex():
    """A vouched-for held reading renders held + miss count + the RAW word in
    hex (Phase 7 step 2 watches bit 2 of exactly this word move)."""
    teensy, client, node = _node()
    try:
        _send_sensor(teensy, node, _FRESH_HELD, raw=_RAW_WORD_HELD, miss=0)
        assert _link_kv(node)['hand_ball_sensor'] == 'held miss=0 raw=0x00000000'
    finally:
        _teardown(teensy, client, node)


def test_link_status_empty_is_only_rendered_for_a_valid_reading():
    """'empty' is reserved for a reading the bridge vouches for; the same frame
    aged out on the RX clock flips to 'stale' and never back to 'empty'."""
    teensy, client, node = _node()
    try:
        _send_sensor(teensy, node, _VALID | _SYNCED, raw=_RAW_WORD_EMPTY, miss=3)
        assert _link_kv(node)['hand_ball_sensor'] == 'empty miss=3 raw=0x00000004'
        _age_out(node)
        value = _link_kv(node)['hand_ball_sensor']
        assert value == 'stale miss=3 raw=0x00000004'
    finally:
        _teardown(teensy, client, node)


def test_link_status_flag_states_render_stale_vs_unknown():
    """The bridge's own STALE flag renders 'stale'; a frame the bridge vouches
    for locally but whose wall anchor is unset (VALID set, TIME_SYNCED clear —
    the un-anchored-boot case, the one reachable not-stale not-valid combo the
    firmware can emit) renders 'unknown'. Neither is ever 'empty'."""
    teensy, client, node = _node()
    try:
        _send_sensor(teensy, node, _DEB | _STALE | _SYNCED,
                     raw=_RAW_WORD_EMPTY, miss=5)
        assert _link_kv(node)['hand_ball_sensor'] == 'stale miss=5 raw=0x00000004'

        _send_sensor(teensy, node, _RAW | _DEB | _VALID, raw=_RAW_WORD_HELD, miss=0)
        assert _link_kv(node)['hand_ball_sensor'] == 'unknown miss=0 raw=0x00000000'
    finally:
        _teardown(teensy, client, node)
