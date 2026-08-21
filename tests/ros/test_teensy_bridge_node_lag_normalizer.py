"""Normalising RING_DIAG's delivery-lag integral (S3 residual (a), 2026-08-15).

WHY THIS EXISTS. FW 13's ``lag_now_us`` is an INTEGRAL of
``(decode elapsed) - (capture elapsed)`` since the arrival clock's seed. On the
S3 conviction bag it read 151-183 ms against a naive 129 ms and CREPT
~0.35 ms/s, and the entry recorded that creep as unexplained
(``logbook/2026-08-14-s3-conviction-ring-leak-measured.md`` residual (a)).

The 2026-08-15 forensics named it, across four bags agreeing to <1 %: the
FlexCAN hardware capture clock runs SLOW against ``micros64()`` — ~237 ppm with
no setpoint stream (bag ``10-06-14``) against ~582 ppm while streaming (bag
``10-08-06``), with the same step visible inside a single bag (``00-44-59``:
0.117 µs/frame over windows 0-27, 0.34-0.46 over windows 28-38). That is why
the integral sailed past the 135 ms one-lap physical cap on a plant whose leak
was identically 0: past a few minutes it is measuring the crystal, not the
robot.

THE ARTEFACT IS LOAD-DEPENDENT AND THE FOLD RATE IS NOT. ``lag_frames`` sits at
~1950 per window in every bag, streaming or not, because ``jb_lag_fold`` folds
jugglebot-bus RX frames (ODrive broadcasts) while the 500 Hz setpoint stream is
TX and is never folded. So ppm and µs/frame step by the SAME factor across a
load change (×2.46 and ×2.47 between the two bags above) — they are one
normaliser up to the constant ~1950, and neither is the load-independent one.
The consequence for this code is the TRAILING calibration: the rate has to be
re-estimated continuously. A rate pooled since node start applies a
quiet-plant 0.1215 µs/frame to a streaming plant whose true rate is 0.3000,
under-correcting by ~348 ppm — which over a segment running to the firmware's
|lag| > 200 ms reseed (~344 s at 582 ppm) leaves ~120 ms of uncorrected
artefact, i.e. it MANUFACTURES a one-lap ring delay in the row labelled "the
row to trend".

WHY THE CORRECTION IS NOT THE OBVIOUS SUBTRACTION. ``window_us`` and
``cap_span_us`` are the per-window advances of the same two clocks
``lag_now_us`` differences, so ``lag_now - lag_prev == window_us - cap_span_us``
is an IDENTITY (up to one inter-frame gap of sampling offset). Subtracting the
measured per-window divergence from the measured per-window lag change yields
~0 for every possible input — including a fully-leaked ring, because stranding
N frames means N frames' worth of capture time is not folded in that window and
lands in exactly the same difference. The two S3 numbers (0.35 ms/s creep,
230-665 ppm ratio) agree because they are algebraically one quantity.

So the correction is calibrated instead: the artefact rate is learned ONLY from
windows whose ring is provably clean (spot leak within ``RING_LEAK_WARN_FRAMES``
and no fresh high-water advance — an occupancy channel derived from ring indices
and not from any clock), over a TRAILING span so it tracks the load state, and
``rate x frames folded`` is subtracted per window and accumulated. A leaking
window is excluded from the calibration but still accumulated into the
correction, which is what makes a real delivery-lag step survive while the clock
drift does not.

What this module pins:

1. **a pure clock-drift stream corrects to ~0** while its raw integral climbs;
2. **a load transition is tracked** — the regression barrier for the pooled-rate
   defect above;
3. **a real lag step survives the correction** — the property the whole
   instrument exists for, and the one the tautological formulation loses;
4. **a leaking plant never calibrates** and says ``n/a`` rather than
   manufacturing a corrected number from a contaminated rate — but a plant that
   leaked ONCE and recovered does calibrate, because the gate is live;
5. **segmentation** — a reseed, a dropped uplink frame (``seq`` gap) and an
   unseeded arrival clock each restart the series rather than imputing across
   the hole;
6. **the calibration warm-up renders ``n/a``**, never a 0 that reads as a
   measured zero lag;
7. **the node wiring** — the ring-clean gate is the JUGGLEBOT bus's spot leak at
   the same threshold ``/ring_diag``'s own WARN uses plus a fresh-advance test on
   the high-water, and the RAW ``lag_now_us`` row is published unchanged beside
   the corrected one.

ROS 2 is mocked by tests/ros/conftest.py. Sections 1-4 drive the normaliser
directly (it is pure Python and takes no bridge); section 5 drives the real
decode + render path.
"""

from __future__ import annotations

import pytest

from teensy_link import MsgType, RingDiag
from teensy_link import protocol as p

from jugglebot.teensy_bridge_node import (
    _LAG_CAL_MIN_WINDOWS,
    _LAG_CAL_TRAIL_WINDOWS,
    LagClockNormalizer,
)

from tests.ros._bridge_harness import _build_paired_node, _teardown

_SEEDED = int(p.RingDiagFlags.LAG_SEEDED)
_RESEEDED = int(p.RingDiagFlags.LAG_RESEED_IN_WINDOW)

#: The forensics' measured QUIET-plant artefact, µs/frame (bag 10-06-14,
#: 2026-08-15). Streams below are built from it, so the tests assert that the
#: normaliser RECOVERS a rate it was never told.
_K_QUIET = 0.1215

#: The same plant STREAMING (bag 10-08-06) — 2.47x the quiet figure, on an
#: unchanged fold rate. The load transition between the two is the regression
#: barrier for the pooled-calibration defect.
_K_STREAM = 0.3000

#: Frames folded per window, quiet or streaming: the setpoint stream is TX and
#: is never folded, so this does NOT move with load (measured 1864-1989 across
#: both bags). At ~1950 the quiet rate is ~237 ppm and the streaming one
#: ~582 ppm.
_FRAMES_NOMINAL = 1950


def _drift_windows(n, *, start_seq=1000, start_lag=0, t0=4_680_000_000,
                   k=_K_QUIET):
    """``n`` consecutive windows of PURE capture-clock drift, no stranding.

    Deliberately not a constant stream: the frame count and the window length
    both wobble, so a normaliser that assumed a fixed 1 s window or a fixed
    frame rate would drift visibly instead of cancelling. ``lag_now`` is
    accumulated by the exact firmware identity (each window's raw lag change IS
    that window's decode-vs-capture divergence), which is what makes these
    fixtures a faithful stand-in for the wire rather than a convenient one.

    ``k`` selects the load state: the fold count deliberately does NOT change
    with it, matching the bags.
    """
    out = []
    lag = start_lag
    t = t0
    for i in range(n):
        frames = _FRAMES_NOMINAL + (i % 7) * 3
        window_us = 1_000_000 + (i % 5) * 137
        div = int(round(k * frames))
        lag += div
        t += window_us
        out.append(dict(seq=start_seq + i, t_local_us=t, window_us=window_us,
                        cap_span_us=window_us - div, lag_now_us=lag,
                        lag_frames=frames))
    return out


def _feed(norm, windows, *, ring_clean=True, reseed_in_window=False,
          seeded=True):
    result = None
    for w in windows:
        result = norm.update(seeded=seeded, reseed_in_window=reseed_in_window,
                             ring_clean=ring_clean, **w)
    return result


# ══════════════════════════════════════════════════════════════════════════════
#  1. Pure clock drift corrects to ~0, and the rate tracks the load
# ══════════════════════════════════════════════════════════════════════════════

def test_a_pure_clock_drift_stream_corrects_to_about_zero():
    """60 clean windows: the raw integral climbs ~14 ms, the corrected one ~0.

    This is the S3 residual in miniature. The raw number is not wrong — it is
    faithfully reporting that two clocks disagree — it simply is not a delivery
    lag, and the whole failure mode of the S3 bag was reading it as one.
    """
    norm = LagClockNormalizer()
    windows = _drift_windows(60)
    result = _feed(norm, windows)

    raw = windows[-1]['lag_now_us'] - windows[0]['lag_now_us']
    assert raw > 12_000, 'fixture is not drifting; the test proves nothing'
    assert result['corrected_us'] is not None
    assert abs(result['corrected_us']) < 50, (
        f'corrected {result["corrected_us"]:.1f} µs against a raw drift of '
        f'{raw} µs — the clock artefact is not being removed')
    assert result['state'] == 'ok'


def test_the_calibrated_rate_recovers_the_measured_constants():
    """The rate the node learns is the one the forensics measured.

    Published in BOTH units on purpose: µs/frame is the internal normaliser,
    ppm is the forensics' language. Neither is the load-independent one — the
    fold rate does not move with load, so they step together — which is why the
    estimator has to be trailing rather than pooled.
    """
    norm = LagClockNormalizer()
    _feed(norm, _drift_windows(_LAG_CAL_TRAIL_WINDOWS))
    assert norm.rate_us_per_frame == pytest.approx(_K_QUIET, abs=0.001)
    # ~1959 folded frames per ~1 s window x 0.1215 µs ≈ 238 µs/s ≈ 238 ppm.
    assert norm.rate_ppm == pytest.approx(237, abs=6)
    assert norm.cal_windows == _LAG_CAL_TRAIL_WINDOWS


def test_the_rate_estimator_is_trailing_not_pooled():
    """It holds a bounded trailing span, so old load states age out.

    The distinction is the whole blocking defect: a pooled estimator keeps a
    quiet plant's 0.1215 µs/frame in the average forever and under-corrects a
    streaming plant by ~348 ppm — enough to manufacture a full ring lap of
    apparent delay over one segment.
    """
    norm = LagClockNormalizer()
    _feed(norm, _drift_windows(_LAG_CAL_TRAIL_WINDOWS * 3))
    assert norm.cal_windows == _LAG_CAL_TRAIL_WINDOWS, (
        'the calibration is pooling every window it has ever seen')


def test_a_load_transition_is_tracked_by_the_trailing_calibration():
    """Quiet → streaming (0.1215 → 0.3000 µs/frame): corrected stays near 0.

    THE REGRESSION BARRIER for the pooled-calibration defect. The fold count is
    deliberately unchanged across the transition, exactly as the bags show, so
    the only thing that moves is the rate — and a normaliser that pooled it
    would carry the quiet-plant rate into the streaming phase and accumulate
    ~348 ppm of uncorrected artefact for as long as the segment ran.

    The residual is bounded by the estimator's own lag (it must re-learn over
    the trailing span), not by the length of the session: ~10 ms here against a
    raw drift of ~70 ms, and it stops growing once the estimator has refilled.
    """
    norm = LagClockNormalizer()
    quiet = _drift_windows(_LAG_CAL_TRAIL_WINDOWS)
    _feed(norm, quiet)
    last = quiet[-1]
    streaming = _drift_windows(150, start_seq=last['seq'] + 1,
                               start_lag=last['lag_now_us'],
                               t0=last['t_local_us'], k=_K_STREAM)
    result = _feed(norm, streaming)

    raw = streaming[-1]['lag_now_us'] - quiet[0]['lag_now_us']
    assert raw > 60_000, 'fixture is not drifting; the test proves nothing'
    assert norm.rate_us_per_frame == pytest.approx(_K_STREAM, abs=0.005), (
        'the estimator never followed the load change')
    assert abs(result['corrected_us']) < 15_000, (
        f'corrected {result["corrected_us"]:.0f} µs against a raw drift of '
        f'{raw} µs — a pooled rate would sit near 30 000 µs here and grow')


def test_the_correction_reads_na_until_it_is_calibrated():
    """Below the warm-up the row is ``n/a``, never 0.

    A 0 in this row reads as "measured, and healthy" — the single most
    dangerous thing an uncalibrated instrument can say, and the same trap the
    unseeded ``lag_now_us`` row already closes with ``n/a``.
    """
    norm = LagClockNormalizer()
    windows = _drift_windows(_LAG_CAL_MIN_WINDOWS)
    result = _feed(norm, windows[:-1])
    assert result['corrected_us'] is None
    assert result['state'] == 'uncalibrated'
    assert result['rate_us_per_frame'] is None
    assert result['rate_ppm'] is None

    result = _feed(norm, windows[-1:])     # the window that closes the warm-up
    assert result['corrected_us'] is not None
    assert result['state'] == 'ok'


# ══════════════════════════════════════════════════════════════════════════════
#  2. A real lag step survives
# ══════════════════════════════════════════════════════════════════════════════

def test_a_real_delivery_lag_step_survives_the_correction():
    """131 ms of stranding shows up as 131 ms corrected, not as 0.

    THE discriminating test. The naive correction — subtracting the measured
    per-window divergence — would return ~0 here, because a delay line hides in
    exactly the same two fields the clock artefact does. The occupancy channel
    (leak, from the ring's own indices) is what tells them apart: this window is
    barred from the calibration but still accumulated into the correction.

    The fixture is physical, not arbitrary: stranding 131 ms of frames means
    131 ms of capture time is NOT folded this window, so ``lag_frames`` drops by
    the same fraction and the divergence carries the step on top of the
    artefact.
    """
    norm = LagClockNormalizer()
    clean = _drift_windows(40)
    _feed(norm, clean)
    rate_before = norm.rate_us_per_frame
    cal_before = norm.cal_windows

    step_us = 131_400
    frames = int(_FRAMES_NOMINAL * (1 - 0.1314))     # deliveries fall behind
    window_us = 1_000_000
    div = step_us + int(round(_K_QUIET * frames))
    last = clean[-1]
    stepped = dict(seq=last['seq'] + 1, t_local_us=last['t_local_us'] + window_us,
                   window_us=window_us, cap_span_us=window_us - div,
                   lag_now_us=last['lag_now_us'] + div, lag_frames=frames)
    result = norm.update(seeded=True, reseed_in_window=False,
                         ring_clean=False, **stepped)

    assert result['corrected_us'] == pytest.approx(step_us, abs=400), (
        'a real delivery-lag step was absorbed by the clock correction — the '
        'instrument now reads healthiest exactly when it is broken')
    # And the contaminated window never touched the calibration.
    assert norm.cal_windows == cal_before
    assert norm.rate_us_per_frame == pytest.approx(rate_before, abs=1e-9)


def test_a_stepped_plateau_holds_rather_than_decaying_back():
    """After the step, a saturated (constant-delay) plateau keeps reading 131 ms.

    A delay line that has stopped growing looks EXACTLY like a healthy bus in
    the divergence channel — same frame rate in, same frame rate out. If the
    correction were rate-only it would walk the plateau back toward zero over
    the following minute and the operator would watch a real 131 ms delay fade
    out of the instrument.
    """
    norm = LagClockNormalizer()
    clean = _drift_windows(40)
    _feed(norm, clean)
    step_us = 131_400
    last = clean[-1]
    frames = int(_FRAMES_NOMINAL * (1 - 0.1314))
    div = step_us + int(round(_K_QUIET * frames))
    lag = last['lag_now_us'] + div
    seq = last['seq'] + 1
    t = last['t_local_us'] + 1_000_000
    norm.update(seeded=True, reseed_in_window=False, ring_clean=False,
                seq=seq, t_local_us=t, window_us=1_000_000,
                cap_span_us=1_000_000 - div, lag_now_us=lag, lag_frames=frames)

    result = None
    for _ in range(10):        # saturated: delay constant, artefact continues
        seq += 1
        t += 1_000_000
        div = int(round(_K_QUIET * _FRAMES_NOMINAL))
        lag += div
        result = norm.update(seeded=True, reseed_in_window=False,
                             ring_clean=False, seq=seq, t_local_us=t,
                             window_us=1_000_000, cap_span_us=1_000_000 - div,
                             lag_now_us=lag, lag_frames=_FRAMES_NOMINAL)
    assert result['corrected_us'] == pytest.approx(step_us, abs=400)


# ══════════════════════════════════════════════════════════════════════════════
#  3. A leaking plant refuses to calibrate; a recovered one resumes
# ══════════════════════════════════════════════════════════════════════════════

def test_a_leaking_plant_never_calibrates_and_says_so():
    """No clean window ⇒ no rate ⇒ ``n/a``, not a rate learned from the leak.

    On a pre-FW-14 board the ring leaks from boot, so every window is
    contaminated. Calibrating anyway would fold the leak's own ratchet into the
    "clock rate" and then subtract the very signal being hunted — the
    manufactured-reassurance failure this arc has already been burned by. The
    raw integral is still published for that plant; this row abstains.
    """
    norm = LagClockNormalizer()
    result = _feed(norm, _drift_windows(60), ring_clean=False)
    assert result['corrected_us'] is None
    assert result['state'] == 'uncalibrated'
    assert norm.cal_windows == 0
    assert norm.rate_us_per_frame is None


def test_a_plant_that_leaked_once_and_recovered_still_calibrates():
    """A dirty stretch delays the calibration; it does not end it.

    The gate is a LIVE verdict, so recovery is real recovery. Keyed on a
    since-boot quantity instead, one excursion would bar every later window and
    the corrected row would read ``n/a`` for the whole boot — the instrument
    going dark exactly at the uptime it exists for.
    """
    norm = LagClockNormalizer()
    dirty = _drift_windows(20)
    _feed(norm, dirty, ring_clean=False)
    assert norm.rate_us_per_frame is None

    last = dirty[-1]
    clean = _drift_windows(_LAG_CAL_MIN_WINDOWS + 5, start_seq=last['seq'] + 1,
                           start_lag=last['lag_now_us'],
                           t0=last['t_local_us'])
    result = _feed(norm, clean)
    assert norm.cal_windows == _LAG_CAL_MIN_WINDOWS + 5
    assert result['corrected_us'] is not None
    assert result['state'] == 'ok'


# ══════════════════════════════════════════════════════════════════════════════
#  4. Segmentation: reseed, dropped frame, unseeded
# ══════════════════════════════════════════════════════════════════════════════

def test_a_reseed_restarts_the_corrected_series_but_not_the_calibration():
    """The reseed moves the LAG series' zero. It does not change the crystal.

    Two different lifetimes in one object, and conflating them is a real bug in
    either direction: keeping the segment across a reseed would read the
    firmware's own reset to 0 as a 131 ms improvement, while resetting the
    calibration would throw away minutes of rate estimate every time the bus
    went quiet for 50 ms.
    """
    norm = LagClockNormalizer()
    clean = _drift_windows(40)
    _feed(norm, clean)
    rate_before = norm.rate_us_per_frame
    cal_before = norm.cal_windows
    last = clean[-1]

    # The firmware zeroed lag_now at the reseed and folded a few frames after
    # it; cap_span is 0 for a reseeded window by contract.
    result = norm.update(seeded=True, reseed_in_window=True, ring_clean=True,
                         seq=last['seq'] + 1,
                         t_local_us=last['t_local_us'] + 1_000_000,
                         window_us=1_000_000, cap_span_us=0, lag_now_us=41,
                         lag_frames=_FRAMES_NOMINAL)
    assert result['state'] == 'segment_start'
    assert result['corrected_us'] == pytest.approx(0.0)
    assert result['segment_frames'] == 0
    assert norm.cal_windows == cal_before          # reseeded window not eligible
    assert norm.rate_us_per_frame == pytest.approx(rate_before, abs=1e-9)

    # And the new segment accumulates from the post-reseed reference.
    after = _feed(norm, _drift_windows(5, start_seq=last['seq'] + 2,
                                       start_lag=41,
                                       t0=last['t_local_us'] + 1_000_000))
    assert abs(after['corrected_us']) < 50


def test_a_dropped_uplink_frame_segments_rather_than_imputing():
    """A ``seq`` gap restarts the series instead of guessing the missing frames.

    ``lag_frames`` is per-window: a dropped RING_DIAG frame takes its window's
    frame count with it, so continuing to accumulate would under-count the
    correction's denominator and re-introduce drift that reads as delivery lag.
    Segmenting costs one visible restart; imputing costs a wrong number that
    looks right.
    """
    norm = LagClockNormalizer()
    clean = _drift_windows(40)
    _feed(norm, clean)
    last = clean[-1]

    result = norm.update(seeded=True, reseed_in_window=False, ring_clean=True,
                         seq=last['seq'] + 2,          # one frame lost
                         t_local_us=last['t_local_us'] + 2_000_000,
                         window_us=1_000_000, cap_span_us=999_750,
                         lag_now_us=last['lag_now_us'] + 500,
                         lag_frames=_FRAMES_NOMINAL)
    assert result['state'] == 'segment_start'
    assert result['corrected_us'] == pytest.approx(0.0)
    assert result['segment_frames'] == 0


def test_an_unseeded_window_reports_nothing_and_drops_the_base():
    """Unseeded ⇒ no reference on the wire, so none here either.

    The wire carries 0 in the lag fields before the arrival clock is seeded.
    Correcting that 0 would produce a corrected 0 — a measured-looking healthy
    reading manufactured out of an absent measurement.
    """
    norm = LagClockNormalizer()
    _feed(norm, _drift_windows(40))
    result = norm.update(seeded=False, reseed_in_window=False, ring_clean=True,
                         seq=2000, t_local_us=5_000_000_000, window_us=1_000_000,
                         cap_span_us=0, lag_now_us=0, lag_frames=0)
    assert result['corrected_us'] is None
    assert result['state'] == 'unseeded'

    # The next seeded window starts a fresh segment rather than differencing
    # against a base from before the gap.
    result = norm.update(seeded=True, reseed_in_window=False, ring_clean=True,
                         seq=2001, t_local_us=5_001_000_000, window_us=1_000_000,
                         cap_span_us=999_750, lag_now_us=93_000,
                         lag_frames=_FRAMES_NOMINAL)
    assert result['state'] == 'segment_start'
    assert result['corrected_us'] == pytest.approx(0.0)


# ══════════════════════════════════════════════════════════════════════════════
#  5. Node wiring: the gate, the rows, and the untouched raw value
# ══════════════════════════════════════════════════════════════════════════════

_HEALTHY = dict(
    t_local_us=4_680_000_000,
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


def _node():
    return _build_paired_node(boot_state_read=False)


def _feed_node(node, windows, **overrides):
    """Decode ``windows`` through the node's real RX callback, then drain.

    Called directly rather than over the loopback socket: these streams are 35+
    frames long and the property under test is the RENDER, not the transport
    (which ``test_teensy_bridge_node_ring_diag.py`` already drives over the real
    RX thread). The decode path is the production one either way.
    """
    for w in windows:
        fields = dict(_HEALTHY)
        fields.update(overrides)
        fields.update(w)
        node._on_ring_diag(int(MsgType.RING_DIAG), 1, RingDiag(**fields).pack(),
                           None)
    node._publish_ring_diag()
    return {kv.key: kv.value for kv in node.ring_diag_pub.published[-1].values}


def test_the_raw_lag_row_is_published_unchanged_beside_the_corrected_one():
    """Both rows, always. The raw one is what the firmware actually sent.

    The correction rests on a calibration this node performs live; a reader who
    distrusts it — or who is re-deriving the artefact from a bag months later —
    must be able to see the untouched number. Hiding it behind the corrected
    value would make the instrument unfalsifiable.
    """
    teensy, client, node = _node()
    try:
        kv = _feed_node(node, [{}])
        assert kv['lag_now_us'] == '41'              # raw, untouched
        assert kv['lag_now_corrected_us'] == 'n/a'   # not yet calibrated
        assert kv['lag_corr_state'] == 'uncalibrated'
        assert kv['lag_corr_windows'] == '1'
        assert 'lag_corr=n/aus' in node.ring_diag_pub.published[-1].message
    finally:
        _teardown(teensy, client, node)


def test_the_node_calibrates_on_clean_windows_and_renders_the_correction():
    """A clean stream through the real render path produces a corrected row."""
    teensy, client, node = _node()
    try:
        kv = _feed_node(node, _drift_windows(_LAG_CAL_MIN_WINDOWS + 5))
        assert kv['lag_corr_windows'] == str(_LAG_CAL_MIN_WINDOWS + 5)
        assert kv['lag_corr_state'] == 'ok'
        assert abs(int(kv['lag_now_corrected_us'])) < 50
        assert float(kv['lag_corr_rate_us_per_frame']) == pytest.approx(
            _K_QUIET, abs=0.001)
        assert float(kv['lag_corr_rate_ppm']) == pytest.approx(237, abs=6)
        assert int(kv['lag_corr_frames']) > 0
        assert int(kv['lag_corr_segment_us']) > 0
        assert f"lag_corr={kv['lag_now_corrected_us']}us" in (
            node.ring_diag_pub.published[-1].message)
    finally:
        _teardown(teensy, client, node)


def test_the_ring_clean_gate_is_the_jugglebot_leak_at_the_warn_threshold():
    """Calibration eligibility and the WARN line are ONE definition.

    Two thresholds for "the ring is clean" would drift apart the first time
    either was retuned, and the failure would be silent: a leak below the
    calibration gate but above the WARN line (or vice versa) reads as a healthy
    bridge whose corrected lag is quietly learned from a leaking one. A leak of
    2 is the documented probe race (under-reporting by construction), so it
    calibrates; 3 is past anything the race can produce, so it does not.
    """
    teensy, client, node = _node()
    try:
        kv = _feed_node(node, _drift_windows(_LAG_CAL_MIN_WINDOWS + 5),
                        true_depth_jb=2, avail_reported_jb=0, leak_hwm_jb=2)
        assert kv['lag_corr_windows'] == str(_LAG_CAL_MIN_WINDOWS + 5)
        assert kv['lag_now_corrected_us'] != 'n/a'
    finally:
        _teardown(teensy, client, node)

    teensy, client, node = _node()
    try:
        kv = _feed_node(node, _drift_windows(_LAG_CAL_MIN_WINDOWS + 5),
                        true_depth_jb=3, avail_reported_jb=0, leak_hwm_jb=3)
        assert kv['lag_corr_windows'] == '0'
        assert kv['lag_now_corrected_us'] == 'n/a'
    finally:
        _teardown(teensy, client, node)


def test_a_leak_on_another_bus_does_not_bar_the_jugglebot_calibration():
    """The arrival clock folds JUGGLEBOT frames; bb/cone leaks are irrelevant.

    S3 measured leak_bb = 1 and leak_cone = 0 beside leak_jb = 247. Gating the
    jugglebot lag's calibration on another bus's occupancy would abstain for a
    reason that has nothing to do with the quantity being corrected.
    """
    teensy, client, node = _node()
    try:
        kv = _feed_node(node, _drift_windows(_LAG_CAL_MIN_WINDOWS + 5),
                        true_depth_bb=40, avail_reported_bb=0, leak_hwm_bb=40)
        assert kv['lag_corr_windows'] == str(_LAG_CAL_MIN_WINDOWS + 5)
        assert kv['lag_now_corrected_us'] != 'n/a'
    finally:
        _teardown(teensy, client, node)


def test_a_boot_that_leaked_once_and_recovered_still_calibrates_at_the_node():
    """The NODE's ring_clean gate reads the high-water as an EVENT, not a level.

    Companion to the normaliser-level recovery test above, and the one that
    actually pins the gate: that test hands ``ring_clean`` in directly, so it is
    blind to how the node COMPUTES it. This one drives the real decode path with
    the firmware's own semantics — ``leak_hwm_jb`` is a maximum since BOOT and
    ``can_buses_ring_probe`` never clears it, so after any excursion it stays
    high for the rest of the session no matter how healthy the ring becomes.

    Gate that on the high-water's LEVEL and the corrected row reads ``n/a`` from
    the first excursion to the next reboot — the instrument going dark at
    exactly the uptime it was built for, on a bridge whose ring has already
    recovered. The gate is therefore the SPOT leak plus a fresh-ADVANCE test,
    and this test is what stops that pair regressing to the level form.
    """
    teensy, client, node = _node()
    try:
        # The excursion: 97 frames stranded, high-water advancing with it.
        dirty = _drift_windows(5)
        kv = _feed_node(node, dirty, true_depth_jb=97, avail_reported_jb=0,
                        leak_hwm_jb=97)
        assert kv['lag_corr_windows'] == '0', 'a leaking window calibrated'

        # ...then the ring drains and STAYS drained, while the high-water keeps
        # reporting 97 forever — the situation the level form cannot survive.
        last = dirty[-1]
        clean = _drift_windows(_LAG_CAL_MIN_WINDOWS + 5,
                               start_seq=last['seq'] + 1,
                               start_lag=last['lag_now_us'],
                               t0=last['t_local_us'])
        kv = _feed_node(node, clean, true_depth_jb=0, avail_reported_jb=0,
                        leak_hwm_jb=97)

        assert kv['leak_jb'] == '0'
        assert kv['leak_hwm_jb'] == '97', (
            'the fixture dropped the historical high-water; it must persist or '
            'this test proves nothing')
        assert kv['lag_corr_windows'] == str(_LAG_CAL_MIN_WINDOWS + 5), (
            'the calibration gate latched on a since-boot maximum')
        assert kv['lag_now_corrected_us'] != 'n/a'
    finally:
        _teardown(teensy, client, node)


def test_an_unseeded_window_renders_the_corrected_row_as_na():
    """Same ``n/a`` discipline as the raw lag rows it sits beside."""
    teensy, client, node = _node()
    try:
        kv = _feed_node(node, [{}], flags=0, lag_now_us=0, lag_hwm_us=0,
                        lag_frames=0)
        assert kv['lag_now_us'] == 'n/a'
        assert kv['lag_now_corrected_us'] == 'n/a'
        assert kv['lag_corr_state'] == 'unseeded'
    finally:
        _teardown(teensy, client, node)


def test_a_reseed_window_renders_a_segment_start_not_a_recovery():
    """The corrected row restarts at 0 and SAYS it restarted.

    Without the state row a bag reader sees the corrected series drop to 0 and
    reads a recovered link. ``lag_corr_state`` is what makes the 0 a definition
    rather than a measurement — the same job ``lag_reseed_in_window`` does for
    the raw series.
    """
    teensy, client, node = _node()
    try:
        _feed_node(node, _drift_windows(_LAG_CAL_MIN_WINDOWS + 5))
        kv = _feed_node(node, [{}], flags=_SEEDED | _RESEEDED, cap_span_us=0,
                        lag_reseeds=1, seq=9_999, lag_now_us=41)
        assert kv['lag_corr_state'] == 'segment_start'
        assert kv['lag_now_corrected_us'] == '0'
        assert kv['lag_reseed_in_window'] == '1'
    finally:
        _teardown(teensy, client, node)
