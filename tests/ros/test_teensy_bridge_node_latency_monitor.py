"""The alarmed command-latency monitor on teensy_bridge_node (2026-08-15).

WHY THIS EXISTS. ``logbook/2026-07-18-teensy-uptime-tracking-degradation.md``
§ Addendum (2026-07-24) makes closing that entry conditional on TWO deliverables,
not one: the fix, AND *"a continuously-measured, alarmed end-to-end
command-latency monitor (logged with uptime_ms)"*. The class it names is not
"the RX ring leaked" — it is **latency drift is invisible until a session is
already degraded**. Four separate sittings ended with an operator saying "janky"
before any instrument said anything.

By 2026-08-15 every MEASUREMENT layer that class needs existed and was bagged
beside ``uptime_ms``: RING_DIAG's leak (the fixed defect's earliest precursor,
WARN above 2 frames), CACHE_DIAG's encoder-cache age floor (WARN at 20 ms), and
the heartbeat's ``lead_clamp_mask``. All of them were still SILENT — a WARN
level on a 1 Hz diagnostics topic is invisible unless somebody is watching that
topic, and nobody watches a topic during a sitting. This module pins the LOUD
half.

What it pins:

1. **each condition raises a log line AND the summary row** — ring leak,
   cache-age floor, and a sustained lead-clamp duty;
1b. **every input is LIVE** — ``leak_hwm`` is a since-boot maximum, so it is
   consumed only where it ADVANCES. Keyed on its level, one excursion would pin
   the alarm for the rest of the session and, since the ring ranks first in the
   precedence, would mask CACHE_AGE and CLAMP_DUTY along with it;
2. **clean reads OK and says nothing** — an alarm that fires on a healthy
   bridge is an alarm the operator learns to ignore;
3. **the duty is sampled honestly** — once per HEARTBEAT (never the same latched
   frame twice) and only while setpoints are actually streaming, with a floor on
   the sample count so a two-sample burst cannot read as duty 1.0;
4. **the warning is throttled** while the condition persists, off a monotonic
   stamp rather than rclpy's ``throttle_duration_sec`` (whose first-call
   bookkeeping can silently drop the initial repeat, and which a test cannot
   observe) — but an ESCALATION to a more upstream cause speaks immediately;
5. **precedence is causal** — ring leak, then cache age, then clamp duty, so the
   token names the most upstream active condition;
6. **a stale input stops asserting** — a 1 Hz frame that stopped arriving must
   not hold its last verdict in either direction;
7. **it gates nothing** — no level change, no refusal, no fault. Same policy as
   BRIDGE_FW_CHECK and the install-skew check: a latency monitor wired into the
   leg command path would convert a reporting gap into an outage.

ROS 2 is mocked by tests/ros/conftest.py. All sockets are ephemeral and the only
shared state is per-node, so this file is xdist-parallel-safe.
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock

from diagnostic_msgs.msg import DiagnosticStatus

from teensy_link import BusHealth, CacheDiag, FaultState, HeartbeatT2J
from teensy_link import LinkState, MsgType, RingDiag
from teensy_link import protocol as p

from jugglebot.teensy_bridge_node import (
    CACHE_AGE_FLOOR_WARN_US,
    LATENCY_MONITOR_CACHE_AGE,
    LATENCY_MONITOR_CLAMP_DUTY,
    LATENCY_MONITOR_OK,
    LATENCY_MONITOR_RING_LEAK,
    _CLAMP_DUTY_MIN_SAMPLES,
    _LATENCY_MONITOR_INPUT_STALE_S,
    _LATENCY_MONITOR_LOG_PERIOD_S,
)

from tests.ros._bridge_harness import (
    _build_paired_node,
    _messages,
    _teardown,
    _wait_until,
)

_SEEDED = int(p.RingDiagFlags.LAG_SEEDED)

# Enough ticks to clear _CLAMP_DUTY_MIN_SAMPLES with margin (the first tick
# never samples — it is the one that establishes the pump baseline).
_TICKS = _CLAMP_DUTY_MIN_SAMPLES + 15


def _node():
    teensy, client, node = _build_paired_node(boot_state_read=False)
    node._logger = MagicMock()      # MockNode.get_logger() returns this
    return teensy, client, node


def _hb(mask=0, uptime_ms=3_600_000):
    return HeartbeatT2J(t_teensy_us=1, link_state=int(LinkState.UP),
                        bus1_health=int(BusHealth.OK),
                        bus2_health=int(BusHealth.OK),
                        fault_state=int(FaultState.NONE), flags=0,
                        uptime_ms=uptime_ms, lead_clamp_mask=mask,
                        max_dev_leg=0xFF)


def _tick(node, *, mask=0, streaming=True, uptime_ms=3_600_000):
    """One 10 Hz link_status tick with a FRESH heartbeat (and maybe a stream)."""
    with node._lock:
        node._latest_heartbeat = _hb(mask=mask, uptime_ms=uptime_ms)
        node._heartbeat_gen += 1
    if streaming:
        node._sp_pump.frames_built += 20      # ~40 Hz of setpoints per 10 Hz tick
    node._publish_link_status()
    return {kv.key: kv.value
            for kv in node.link_status_pub.published[-1].values}


def _alarms(node):
    return [m for m in _messages(node._logger.warning) if 'LATENCY MONITOR' in m]


def _send_ring(node, **overrides):
    """One RING_DIAG window through the node's real decode + drain."""
    fields = dict(
        t_local_us=14_400_000_000,
        true_depth_jb=0, true_depth_bb=0, true_depth_cone=0,
        avail_reported_jb=0, avail_reported_bb=0, avail_reported_cone=0,
        leak_hwm_jb=0, leak_hwm_bb=0, leak_hwm_cone=0,
        true_depth_hwm_jb=4, true_depth_hwm_bb=2, true_depth_hwm_cone=1,
        fifo_overflows_jb=0, fifo_overflows_bb=0, fifo_overflows_cone=0,
        fifo_warns_jb=0, fifo_warns_bb=0, fifo_warns_cone=0,
        probe_ticks=999, lag_now_us=41, lag_hwm_us=380,
        cap_span_us=999_959, lag_frames=2_241, lag_reseeds=0,
        sdo_rtt_min_us=1_104, sdo_rtt_max_us=2_870, sdo_rtt_last_us=1_331,
        sdo_rtt_count=50, seq=14_400, window_us=1_000_011, flags=_SEEDED)
    fields.update(overrides)
    # Each call is a distinct window; the alarm's high-water logic differences
    # consecutive windows, so they must not all carry the same seq/stamp.
    fields['seq'] = int(fields['seq']) + _send_ring.calls
    fields['t_local_us'] = int(fields['t_local_us']) + _send_ring.calls * 1_000_000
    _send_ring.calls += 1
    node._on_ring_diag(int(MsgType.RING_DIAG), 1, RingDiag(**fields).pack(),
                       None)
    node._publish_ring_diag()


#: Monotonic across the module so consecutive windows are never identical. Each
#: test builds a fresh node, so nothing carries over that matters.
_send_ring.calls = 0


def _send_cache(node, *, floor_us, seen_mask=0x7F):
    peak_us = min(floor_us + 1000, 0xFFFF_FFFF)   # u32 saturation rail
    cd = CacheDiag(t_local_us=14_400_000_000, seen_mask=seen_mask,
                   age_min_us=(floor_us,) * 7, age_max_us=(peak_us,) * 7,
                   enc_frames=(1000,) * 7, seq=14_400, window_us=1_000_000,
                   samples=100)
    node._on_cache_diag(int(MsgType.CACHE_DIAG), 1, cd.pack(), None)
    node._publish_cache_diag()


# ══════════════════════════════════════════════════════════════════════════════
#  Clean
# ══════════════════════════════════════════════════════════════════════════════

def test_a_clean_bridge_reads_ok_and_logs_nothing():
    """No condition ⇒ ``latency_monitor=OK`` and silence.

    The negative case is the load-bearing one for an ADVISORY alarm: it has no
    gate behind it, so its only enforcement mechanism is the operator's
    attention, and an alarm that cries wolf spends that in a week.
    """
    teensy, client, node = _node()
    try:
        _send_ring(node)                                  # leak 0
        _send_cache(node, floor_us=3_000)                 # 3 ms floor, healthy
        kv = None
        for i in range(_TICKS):
            kv = _tick(node, mask=0)
        assert kv['latency_monitor'] == LATENCY_MONITOR_OK
        assert _alarms(node) == []
    finally:
        _teardown(teensy, client, node)


def test_the_summary_row_is_published_before_any_heartbeat_arrives():
    """The row exists from the first publish, not from the first heartbeat.

    ``uptime_ms`` and the rest of the heartbeat block vanish when no frame has
    arrived. A summary row that vanished with them would read as "no data" in a
    bag whose whole subject is a link that went quiet — so it is published
    unconditionally, exactly like ``install_skew``.
    """
    teensy, client, node = _node()
    try:
        node._publish_link_status()
        kv = {v.key: v.value
              for v in node.link_status_pub.published[-1].values}
        assert kv['latency_monitor'] == LATENCY_MONITOR_OK
        assert 'uptime_ms' not in kv       # no heartbeat yet — the row still is
    finally:
        _teardown(teensy, client, node)


# ══════════════════════════════════════════════════════════════════════════════
#  Each condition is loud
# ══════════════════════════════════════════════════════════════════════════════

def test_a_ring_leak_raises_the_alarm_and_the_summary_row():
    """The leak is the fixed defect's earliest precursor — it must be loud.

    The whole S3 chain (247 stranded frames ⇒ a 115 ms delay line ⇒ a clamp
    pinning commanded motion to stale feedback ⇒ "janky") starts here, hours
    before the behaviour is visible. This is the one input that can warn while
    the robot still feels fine.
    """
    teensy, client, node = _node()
    try:
        _send_ring(node, true_depth_jb=131, avail_reported_jb=0,
                   leak_hwm_jb=134)
        kv = _tick(node)
        assert kv['latency_monitor'] == LATENCY_MONITOR_RING_LEAK
        alarms = _alarms(node)
        assert len(alarms) == 1
        assert LATENCY_MONITOR_RING_LEAK in alarms[0]
        # The SPOT leak, 131. The high-water (134) is a since-boot maximum and
        # this is the first window, so no advance can be observed and none is
        # claimed — see test_a_historical_high_water_does_not_latch_the_alarm
        # for why the alarm refuses to read a cumulative field as a live one.
        assert '131' in alarms[0]
        # The contract says "logged with uptime_ms" — the tag is what makes a
        # growth trend attributable rather than merely visible.
        assert 'uptime_ms=3600000' in alarms[0]
    finally:
        _teardown(teensy, client, node)


def test_a_historical_high_water_does_not_latch_the_alarm():
    """One excursion must not pin RING_LEAK — nor mask what comes after it.

    ``leak_hwm`` is a maximum since BOOT; ``can_buses_ring_probe`` never resets
    it. Read as a level, a single >2-frame excursion would hold the alarm and a
    30 s WARNING for the rest of the session with no way to observe recovery —
    and because the ring ranks FIRST in the causal precedence, it would also
    suppress CACHE_AGE and CLAMP_DUTY for that whole session. "Continuously
    measured" in the 2026-07-24 contract has to mean recovery is visible too.
    """
    teensy, client, node = _node()
    try:
        # The excursion, then two clean windows carrying the SAME (historical)
        # high-water: the ring drained and stayed drained.
        _send_ring(node, true_depth_jb=131, avail_reported_jb=0,
                   leak_hwm_jb=134)
        assert _tick(node)['latency_monitor'] == LATENCY_MONITOR_RING_LEAK
        _send_ring(node, true_depth_jb=0, avail_reported_jb=0, leak_hwm_jb=134)
        assert _tick(node)['latency_monitor'] == LATENCY_MONITOR_OK, (
            'the alarm latched on a since-boot maximum')

        # And a genuinely different condition is now visible rather than masked.
        _send_cache(node, floor_us=CACHE_AGE_FLOOR_WARN_US + 5_000)
        _send_ring(node, true_depth_jb=0, avail_reported_jb=0, leak_hwm_jb=134)
        assert _tick(node)['latency_monitor'] == LATENCY_MONITOR_CACHE_AGE

        # The bottom of the chain is reachable too. Both downstream conditions
        # are checked because the precedence makes RING_LEAK mask BOTH, and a
        # latch that only ever surfaced as a missing CACHE_AGE would still hide
        # the clamp duty on a bridge with no FW >= 13 cache frames at all.
        _send_cache(node, floor_us=0)
        for i in range(_TICKS):
            _send_ring(node, true_depth_jb=0, avail_reported_jb=0,
                       leak_hwm_jb=134)
            _tick(node, mask=(0x3 if i % 5 < 2 else 0))        # 40 % duty
        assert _tick(node, mask=0x3)['latency_monitor'] == (
            LATENCY_MONITOR_CLAMP_DUTY)
    finally:
        _teardown(teensy, client, node)


def test_a_fresh_high_water_advance_alarms_even_with_a_clean_spot_read():
    """A leak that appeared and drained between two 1 Hz reads still alarms.

    The spot pair is one sample out of ~1000 service-tick probes per window, so
    a short excursion under load can hide between reads — which is exactly what
    the high-water exists for. It is admissible as an EVENT (it advanced in this
    window) even though it is inadmissible as a level.
    """
    teensy, client, node = _node()
    try:
        _send_ring(node, true_depth_jb=0, avail_reported_jb=0, leak_hwm_jb=1)
        assert _tick(node)['latency_monitor'] == LATENCY_MONITOR_OK
        _send_ring(node, true_depth_jb=0, avail_reported_jb=0, leak_hwm_jb=97)
        kv = _tick(node)
        assert kv['latency_monitor'] == LATENCY_MONITOR_RING_LEAK
        assert '97' in _alarms(node)[0]
        # ...and it clears again once the advance stops.
        _send_ring(node, true_depth_jb=0, avail_reported_jb=0, leak_hwm_jb=97)
        assert _tick(node)['latency_monitor'] == LATENCY_MONITOR_OK
    finally:
        _teardown(teensy, client, node)


def test_a_cache_age_floor_breach_raises_the_alarm():
    """A cache floor above 20 ms means the feedback the clamp reads is stale.

    A FLOOR, not a peak: jitter, a missed frame and a late tick can all raise a
    maximum, but none of them can raise a minimum, so this cannot be a sampling
    artefact.
    """
    teensy, client, node = _node()
    try:
        _send_cache(node, floor_us=CACHE_AGE_FLOOR_WARN_US + 5_000)
        kv = _tick(node)
        assert kv['latency_monitor'] == LATENCY_MONITOR_CACHE_AGE
        alarms = _alarms(node)
        assert len(alarms) == 1
        assert LATENCY_MONITOR_CACHE_AGE in alarms[0]
    finally:
        _teardown(teensy, client, node)


def test_an_unseen_axis_cannot_raise_the_cache_age_alarm():
    """An axis that has never been cached rides the u32 rail. That is not staleness.

    The single-leg bench rig and a dark hand ODrive both look like this. If the
    monitor alarmed on them it would be alarming continuously on the bench,
    which is how an advisory dies.
    """
    teensy, client, node = _node()
    try:
        _send_cache(node, floor_us=0xFFFF_FFFF, seen_mask=0)
        kv = _tick(node)
        assert kv['latency_monitor'] == LATENCY_MONITOR_OK
        assert _alarms(node) == []
    finally:
        _teardown(teensy, client, node)


def test_a_sustained_clamp_duty_raises_the_alarm():
    """~0.40 duty over the trailing window ⇒ CLAMP_DUTY.

    Calibrated on this exact statistic (mask-any, streaming-gated, trailing
    10 s, ≥ the sample floor) rather than picked: a LEAKING FW 13 bridge at
    4.04 h measures 0.4441 with a worst window of 0.68, while FW 14 at
    5.80 / 5.84 / 15.19 h measures 0.0183 / 0.0198 / 0.0039 with worst windows
    0.0667 / 0.0333 / 0.0200. This input needs no firmware support beyond the
    heartbeat, which is why it is the one that still fires on a board too old to
    send RING_DIAG.
    """
    teensy, client, node = _node()
    try:
        kv = None
        for i in range(_TICKS):
            kv = _tick(node, mask=(0x3 if i % 5 < 2 else 0))   # 40 %
        assert kv['latency_monitor'] == LATENCY_MONITOR_CLAMP_DUTY
        alarms = _alarms(node)
        assert len(alarms) == 1
        assert LATENCY_MONITOR_CLAMP_DUTY in alarms[0]
    finally:
        _teardown(teensy, client, node)


def test_a_healthy_clamp_duty_stays_quiet():
    """~0.06 duty stays quiet: move onsets clamp, and that is normal.

    Deliberately set an order of magnitude ABOVE the measured healthy means
    (0.004-0.02) and at the worst healthy 10 s window on record (0.0667), so the
    threshold is shown to have margin against the real distribution's tail
    rather than against its average.
    """
    teensy, client, node = _node()
    try:
        kv = None
        for i in range(_TICKS):
            kv = _tick(node, mask=(0x1 if i % 15 == 0 else 0))   # ~0.067
        assert kv['latency_monitor'] == LATENCY_MONITOR_OK
        assert _alarms(node) == []
    finally:
        _teardown(teensy, client, node)


# ══════════════════════════════════════════════════════════════════════════════
#  How the duty is sampled
# ══════════════════════════════════════════════════════════════════════════════

def test_the_duty_is_only_sampled_while_setpoints_are_streaming():
    """No stream ⇒ no samples, whatever the mask says.

    The lead clamp acts on the leg interpolator's setpoint stream. With nothing
    streaming there is nothing to clamp, and folding those ticks in as "not
    clamped" would dilute a short move's duty to nothing — precisely the moves
    where the clamp does its damage.
    """
    teensy, client, node = _node()
    try:
        kv = None
        for _ in range(_TICKS):
            kv = _tick(node, mask=0x3F, streaming=False)
        assert len(node._lm_clamp_samples) == 0
        assert kv['latency_monitor'] == LATENCY_MONITOR_OK
        assert _alarms(node) == []
    finally:
        _teardown(teensy, client, node)


def test_a_latched_heartbeat_is_not_resampled():
    """``_latest_heartbeat`` is a latch; the duty must count FRAMES, not ticks.

    Without the generation counter, a link that died mid-clamp would have its
    last frame resampled at 10 Hz forever and the duty would pin at 1.0 — an
    alarm about a link that is not delivering heartbeats at all, reported as a
    clamp problem.
    """
    teensy, client, node = _node()
    try:
        with node._lock:
            node._latest_heartbeat = _hb(mask=0x3F)
            node._heartbeat_gen += 1
        for _ in range(_TICKS):
            node._sp_pump.frames_built += 20      # streaming, but no new frame
            node._publish_link_status()
        assert len(node._lm_clamp_samples) <= 1
        kv = {v.key: v.value
              for v in node.link_status_pub.published[-1].values}
        assert kv['latency_monitor'] == LATENCY_MONITOR_OK
    finally:
        _teardown(teensy, client, node)


def test_a_short_burst_cannot_fire_the_duty_alarm():
    """Fewer than _CLAMP_DUTY_MIN_SAMPLES ⇒ no verdict, even at duty 1.0.

    3 s of streaming is the floor. The first two clamped samples of any stream
    are duty 1.0, and firing on them would make the alarm a move-onset detector.
    """
    teensy, client, node = _node()
    try:
        kv = None
        for _ in range(_CLAMP_DUTY_MIN_SAMPLES - 5):
            kv = _tick(node, mask=0x3F)
        assert len(node._lm_clamp_samples) < _CLAMP_DUTY_MIN_SAMPLES
        assert kv['latency_monitor'] == LATENCY_MONITOR_OK
        assert _alarms(node) == []
    finally:
        _teardown(teensy, client, node)


def test_the_duty_window_ages_samples_out():
    """The duty is a TRAILING window, so a fixed session cannot poison it forever.

    A degraded stretch must stop asserting once it is out of the window —
    otherwise the alarm latches for the rest of the session and the operator
    cannot tell a fix from a stuck instrument.
    """
    teensy, client, node = _node()
    try:
        for i in range(_TICKS):
            _tick(node, mask=0x3F)
        assert len(node._lm_clamp_samples) > 0
        # Rewind every sample past the trailing window (monotonic stamps are
        # the only state involved, so this is exact rather than a sleep).
        aged = [(t - 3600.0, c) for t, c in node._lm_clamp_samples]
        node._lm_clamp_samples.clear()
        node._lm_clamp_samples.extend(aged)
        kv = _tick(node, mask=0)
        assert len(node._lm_clamp_samples) == 1
        assert kv['latency_monitor'] == LATENCY_MONITOR_OK
    finally:
        _teardown(teensy, client, node)


# ══════════════════════════════════════════════════════════════════════════════
#  Throttling, precedence, staleness, and the advisory contract
# ══════════════════════════════════════════════════════════════════════════════

def test_the_warning_is_throttled_while_the_condition_persists():
    """One line per period, not one per 10 Hz tick.

    An unthrottled alarm at the link_status rate would emit 36 000 lines an hour
    and bury the guard-latch and link-loss lines that DO need immediate
    attention. Enforced off a monotonic stamp rather than rclpy's
    ``throttle_duration_sec``, whose first-call bookkeeping can drop the initial
    repeat — and which no test could observe.
    """
    teensy, client, node = _node()
    try:
        _send_ring(node, true_depth_jb=131, avail_reported_jb=0,
                   leak_hwm_jb=134)
        for _ in range(20):
            _tick(node)
        assert len(_alarms(node)) == 1, 'the alarm is not throttled'

        # Rewind the throttle stamp by one full period: the condition still
        # holds, so it must speak again.
        node._lm_last_log_t -= (_LATENCY_MONITOR_LOG_PERIOD_S + 1.0)
        _send_ring(node, true_depth_jb=131, avail_reported_jb=0,
                   leak_hwm_jb=134)
        _tick(node)
        assert len(_alarms(node)) == 2, (
            'the alarm went silent for good — a persisting condition must keep '
            'saying so')
    finally:
        _teardown(teensy, client, node)


def test_an_escalation_speaks_immediately_rather_than_waiting_for_the_throttle():
    """CLAMP_DUTY then RING_LEAK: the cause is announced without a 30 s wait.

    A throttle keyed on "some condition is active" would swallow the escalation
    — the operator would be looking at a clamp-duty line while the ring leak
    that explains it, and changes what they should do about it, went unsaid for
    half a minute. Keyed on RANK, so the door only opens going UP the causal
    chain: a de-escalation or a flap between two conditions cannot reuse it.
    """
    teensy, client, node = _node()
    try:
        for i in range(_TICKS):
            _tick(node, mask=(0x3 if i % 5 < 2 else 0))    # 40 % duty
        assert len(_alarms(node)) == 1
        assert LATENCY_MONITOR_CLAMP_DUTY in _alarms(node)[0]

        _send_ring(node, true_depth_jb=131, avail_reported_jb=0,
                   leak_hwm_jb=134)
        _tick(node, mask=0x3)
        alarms = _alarms(node)
        assert len(alarms) == 2, (
            'the escalation to RING_LEAK was swallowed by the throttle')
        assert LATENCY_MONITOR_RING_LEAK in alarms[1]

        # A de-escalation does NOT reuse the door: the ring input goes stale and
        # CLAMP_DUTY takes over, quietly, until the period elapses.
        node._lm_ring_t -= (_LATENCY_MONITOR_INPUT_STALE_S + 1.0)
        for _ in range(5):
            _tick(node, mask=0x3)
        assert len(_alarms(node)) == 2
    finally:
        _teardown(teensy, client, node)


def test_precedence_is_causal_ring_then_cache_then_clamp():
    """The token names the most UPSTREAM active condition.

    All three are links in one chain: a stranded ring makes the cached feedback
    old, and old feedback is what the lead clamp pins commanded motion to. If
    the row named the loudest symptom instead of the earliest cause, the
    operator would chase the clamp while the ring was the thing to fix.
    """
    teensy, client, node = _node()
    try:
        _send_cache(node, floor_us=CACHE_AGE_FLOOR_WARN_US + 5_000)
        _send_ring(node, true_depth_jb=131, avail_reported_jb=0,
                   leak_hwm_jb=134)
        kv = None
        for i in range(_TICKS):
            kv = _tick(node, mask=0x3F)          # duty ~1.0 as well
        assert kv['latency_monitor'] == LATENCY_MONITOR_RING_LEAK

        # Ring input goes stale (the board stopped sending 0x92): the next
        # cause down the chain takes over, not OK.
        node._lm_ring_t -= (_LATENCY_MONITOR_INPUT_STALE_S + 1.0)
        kv = _tick(node, mask=0x3F)
        assert kv['latency_monitor'] == LATENCY_MONITOR_CACHE_AGE

        node._lm_cache_t -= (_LATENCY_MONITOR_INPUT_STALE_S + 1.0)
        kv = _tick(node, mask=0x3F)
        assert kv['latency_monitor'] == LATENCY_MONITOR_CLAMP_DUTY
    finally:
        _teardown(teensy, client, node)


def test_a_stale_diagnostics_input_stops_asserting_its_last_verdict():
    """A 1 Hz frame that stopped arriving must not keep raising the alarm.

    Both directions matter. A held verdict would alarm forever after a single
    bad window, and a zeroed one would read as a clean bridge. Skipping the
    input is the only honest third option — and it is why the clamp-duty input,
    which needs no diagnostics frame at all, exists.
    """
    teensy, client, node = _node()
    try:
        _send_ring(node, true_depth_jb=131, avail_reported_jb=0,
                   leak_hwm_jb=134)
        assert _tick(node)['latency_monitor'] == LATENCY_MONITOR_RING_LEAK
        node._lm_ring_t -= (_LATENCY_MONITOR_INPUT_STALE_S + 1.0)
        assert _tick(node)['latency_monitor'] == LATENCY_MONITOR_OK
    finally:
        _teardown(teensy, client, node)


def test_the_monitor_never_gates_refuses_or_colours_the_level():
    """Advisory by contract: the DiagnosticStatus level is untouched.

    /link_status's level is the link/fault channel the orchestrator reads. A
    latency advisory that coloured it would turn a REPORTING gap into a
    behaviour change — strictly worse than the failure being reported, and the
    same reasoning that keeps BRIDGE_FW_CHECK and the install-skew check
    advisory.
    """
    teensy, client, node = _node()
    try:
        # A real heartbeat over the wire so the link reads UP and the level is
        # OK — otherwise NO_HEARTBEAT would mask the assertion.
        teensy.send_to_jetson(int(MsgType.HEARTBEAT_T2J), _hb().pack())
        assert _wait_until(lambda: node._latest_heartbeat is not None)
        node._publish_link_status()
        clean = node.link_status_pub.published[-1]
        assert clean.level == DiagnosticStatus.OK

        _send_ring(node, true_depth_jb=131, avail_reported_jb=0,
                   leak_hwm_jb=134)
        _send_cache(node, floor_us=CACHE_AGE_FLOOR_WARN_US + 5_000)
        node._publish_link_status()
        alarmed = node.link_status_pub.published[-1]
        kv = {v.key: v.value for v in alarmed.values}
        assert kv['latency_monitor'] == LATENCY_MONITOR_RING_LEAK
        assert alarmed.level == DiagnosticStatus.OK, (
            'the latency monitor coloured /link_status — it must be advisory')
        assert alarmed.message == clean.message
    finally:
        _teardown(teensy, client, node)


def test_the_monitor_never_raises_out_of_the_link_status_timer():
    """A monitor that threw would take the whole 10 Hz message down with it.

    /link_status is the operator's only view of link state and guard faults, so
    every renderer on it is defensive. Driven with the pathological inputs a
    pre-flash or half-dead bridge produces: no heartbeat, no diagnostics frames,
    an empty duty window.
    """
    teensy, client, node = _node()
    try:
        node._publish_link_status()
        node._latency_monitor_step(None, 0)
        node._lm_ring_leak = 999
        node._lm_ring_t = time.monotonic()
        assert node._latency_monitor_step(None, 0) == LATENCY_MONITOR_RING_LEAK
        # uptime is unknown without a heartbeat, and that must not be an error.
        assert 'uptime_ms=n/a' in _alarms(node)[0]
    finally:
        _teardown(teensy, client, node)
