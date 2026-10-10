"""Synthetic-trace tests for the minimum-latency QTM -> ROS clock estimator.

``jugglebot.qtm_clock_sync.MinLatencyClockSync`` replaced the EMA-of-latency in
``MocapInterface`` on 2026-10-10 (logbook 2026-10-10-mocap-min-latency-clock-sync):
under Jetson load the EMA put 4-59 ms of queueing wander into every mocap frame
stamp within one 7 s BB calibration sweep.

Each trace is QTM's 300 Hz frame clock (integer µs, as ``packet.timestamp``),
a true QTM->ROS offset (optionally drifting), and a receive latency = a fixed
floor + small unqueued jitter + a load term that only ever ADDS delay. The
estimator should report offset + floor (the lower envelope): the error below is
``output - (true offset + floor)``.

Bounds, and why they are what they are:
* unloaded (steady) error <= 0.3 ms: the envelope sits on the jitter's minimum;
* under load, within-window peak-to-peak <= 1 ms and mean bias vs the unloaded
  trace <= 0.5 ms (the requirement the BB calibration needs: 1 ms at 67 deg/s
  is 0.07 deg);
* drift of +-50 ppm tracked within 1 ms (a plain window minimum would lag by
  drift x window = 0.5 ms at 10 s, and a mean-based estimate would not care -
  the drift term is what is under test);
* the output never moves faster than ``max_slew`` after startup.
"""

from __future__ import annotations

import numpy as np
import pytest

from jugglebot.qtm_clock_sync import MinLatencyClockSync

FPS = 300.0
FLOOR_NS = 2_000_000          # 2 ms network + QTM processing floor
OFFSET0_NS = 1_791_550_000 * 10**9   # a realistic absolute ROS-vs-QTM offset


def _trace(duration_s, *, seed=0, drift_ppm=0.0, load=None, jitter_ms=0.3, wobble_ms=0.5,
           qtm0_us=10_000_000):
    """(qtm_us, ros_ns, truth_ns) per packet. truth = true offset + FLOOR (the envelope).

    Unqueued latency = FLOOR + a floor wobble, U(0, wobble_ms) held for 0.25 s at a
    time, + per-packet Exp(jitter_ms). The wobble reproduces what the bags show: the
    minima of 0.5 s bins of the reconstructed latency scatter by 0.2-0.4 ms even
    unloaded (bin-min successive-difference SD; ~/bb_calibration_sessions/
    clock_sync_20261010/binmin_cmp.py), so the envelope itself is that uncertain."""
    rng = np.random.default_rng(seed)
    n = int(duration_s * FPS)
    q_us = qtm0_us + np.round(np.arange(n) * 1e6 / FPS).astype(np.int64)
    t_s = (q_us - qtm0_us) / 1e6
    true_off = OFFSET0_NS + (drift_ppm * 1e-6 * t_s * 1e9)
    seg = (t_s / 0.25).astype(int)
    wobble = rng.uniform(0.0, wobble_ms * 1e6, seg.max() + 1)[seg]
    lat = FLOOR_NS + wobble + rng.exponential(jitter_ms * 1e6, n)
    if load is not None:
        lat = lat + load(t_s, rng)
    ros = q_us * 1000 + true_off + lat
    # a receiver processes packets in order: arrival times are non-decreasing
    ros = np.maximum.accumulate(ros)
    return q_us, ros.astype(np.int64), (true_off + FLOOR_NS)


def _bursty(t_s, rng):
    """1-3 stalls per second of 20-80 ms: every packet sent during a stall is
    delivered when the stall ends (then drained at 50 us per packet)."""
    extra = np.zeros(len(t_s))
    t = 0.0
    while t < t_s[-1]:
        t += rng.uniform(1 / 3.0, 1.0)
        dur = rng.uniform(0.020, 0.080)
        k = (t_s >= t) & (t_s < t + dur)
        idx = np.where(k)[0]
        extra[idx] = (t + dur - t_s[idx]) * 1e9 + np.arange(len(idx)) * 50e3
    return extra


def _heavy(t_s, rng):
    """Bag-2-like load (2026-10-10_00-24-06: the EMA offset wandered 4-59 ms within a
    7 s sweep). The receiving thread alternates between stalls and short runs: a
    stall's length is U(0.5, 1.5) x 2 x level(t), with level(t) wandering between 4
    and 59 ms on a 3.5 s scale; a run (thread scheduled, socket drained at 50 us per
    packet) lasts Exp(3 ms) - about one frame. Packets sent during a stall are
    delivered when it ends; only those sent during a run arrive unqueued."""
    knots = np.arange(0, t_s[-1] + 3.5, 3.5)
    level = rng.uniform(4e-3, 59e-3, len(knots))
    extra = np.zeros(len(t_s))
    t = 0.0
    while t < t_s[-1]:
        t += rng.exponential(0.003)                       # run
        dur = rng.uniform(0.5, 1.5) * 2 * float(np.interp(t, knots, level))
        i0, i1 = np.searchsorted(t_s, [t, t + dur])
        idx = np.arange(i0, i1)
        extra[idx] = (t + dur - t_s[idx]) * 1e9 + np.arange(len(idx)) * 50e3
        t += dur
    return extra


def _run(q_us, ros, **kw):
    est = MinLatencyClockSync(**kw)
    out = np.empty(len(q_us), dtype=np.int64); events = []
    for i, (q, r) in enumerate(zip(q_us.tolist(), ros.tolist())):
        ev = est.update(q, r)
        if ev:
            events.append((i, ev))
        out[i] = est.offset_ns
    return out, est, events


def _err(out, truth):
    return (out - OFFSET0_NS - (truth - OFFSET0_NS)) / 1e6     # ms (differences taken in int first)


def _window_ptp(err, t_s, start_s, win_s=7.0):
    worst = 0.0
    t = start_s
    while t + win_s <= t_s[-1]:
        k = (t_s >= t) & (t_s < t + win_s)
        worst = max(worst, float(np.ptp(err[k])))
        t += win_s / 2
    return worst


def _assert_slew(out, ros, est, start_s):
    t = (ros - ros[0]) / 1e9
    k = np.where(t > start_s + est.window_ns / 1e9 + 0.01)[0]
    d_out = np.abs(np.diff(out[k]))
    d_t = np.diff(ros[k]).astype(float)
    assert np.all(d_out <= est.max_slew * d_t + 2.0), 'slew limit violated'


# ── steady / startup ─────────────────────────────────────────────────

def test_first_packet_initialises_from_its_own_measurement():
    est = MinLatencyClockSync()
    assert est.offset_ns is None
    assert est.update(1_000_000, 5_000_000_000) == 'init'
    assert est.offset_ns == 5_000_000_000 - 1_000_000_000


def test_steady_trace_sits_on_the_floor():
    q, ros, truth = _trace(40)
    out, est, events = _run(q, ros)
    t = (q - q[0]) / 1e6
    err = _err(out, truth)
    assert events == [(0, 'init')]
    assert np.max(np.abs(err[t > 1.0])) <= 0.3          # usable within the first second
    assert _window_ptp(err, t, 2.0) <= 0.3
    _assert_slew(out, ros, est, 0.0)


def test_startup_is_usable_from_the_first_packets_and_never_above_them():
    q, ros, truth = _trace(3, load=_bursty, seed=4)
    out, est, _ = _run(q, ros)
    # until the drift fit engages (min_fit_span_s = 2 s) the startup output is the
    # running minimum: never above any packet seen so far
    meas = ros - q * 1000
    t = (q - q[0]) / 1e6
    early = t < 1.9
    assert np.all(out[early] <= np.minimum.accumulate(meas)[early] + 1)
    assert np.max(np.abs(_err(out, truth)[t > 0.5])) <= 1.0


# ── drift ────────────────────────────────────────────────────────────

@pytest.mark.parametrize('ppm', [50.0, -50.0])
def test_drift_is_tracked(ppm):
    q, ros, truth = _trace(60, drift_ppm=ppm, seed=1)
    out, est, events = _run(q, ros)
    t = (q - q[0]) / 1e6
    err = _err(out, truth)
    assert len(events) == 1
    assert np.max(np.abs(err[t > 12.0])) <= 1.0
    assert abs(est.drift * 1e6 - ppm) < 25.0
    _assert_slew(out, ros, est, 0.0)


@pytest.mark.parametrize('ppm', [50.0, -50.0])
def test_drift_under_heavy_load_is_tracked(ppm):
    q, ros, truth = _trace(60, drift_ppm=ppm, load=_heavy, seed=2)
    out, est, _ = _run(q, ros)
    t = (q - q[0]) / 1e6
    assert np.max(np.abs(_err(out, truth)[t > 12.0])) <= 1.0


# ── load ─────────────────────────────────────────────────────────────

@pytest.mark.parametrize('load,seed', [(_bursty, 3), (_bursty, 13), (_heavy, 5), (_heavy, 15)])
def test_load_neither_wanders_nor_biases_the_offset(load, seed):
    q, ros, truth = _trace(60, load=load, seed=seed)
    out, est, events = _run(q, ros)
    q0, ros0, truth0 = _trace(60, seed=seed)                  # same jitter, no load
    out0, _, _ = _run(q0, ros0)
    t = (q - q[0]) / 1e6
    err, err0 = _err(out, truth), _err(out0, truth0)
    settled = t > 10.0
    assert len(events) == 1                                  # no spurious re-anchor
    assert _window_ptp(err, t, 10.0) <= 1.0
    assert abs(np.mean(err[settled]) - np.mean(err0[settled])) <= 0.5
    assert np.max(np.abs(err[settled])) <= 1.0
    _assert_slew(out, ros, est, 0.0)


def test_heavy_load_ema_baseline_would_have_wandered():
    """The trace is a fair test: the 53b0e0a6 estimator (EMA, alpha 0.01) on it
    wanders by tens of ms, as on the real bag."""
    q, ros, truth = _trace(60, load=_heavy, seed=5)
    meas = (ros - q * 1000).astype(float)
    ema = np.empty(len(meas)); x = meas[0]
    for i, m in enumerate(meas):
        a = 1.0 / (i + 1) if i < 100 else 0.01
        x = x + a * (m - x); ema[i] = x
    t = (q - q[0]) / 1e6
    assert _window_ptp(_err(ema, truth), t, 10.0) > 10.0


def test_isolated_sub_floor_sample_is_clipped():
    q, ros, truth = _trace(40, seed=6)
    ros = ros.copy()
    k = int(25 * FPS)
    ros[k] -= 5_000_000                     # one timestamp anomaly 5 ms below the floor
    out, est, events = _run(q, ros)
    t = (q - q[0]) / 1e6
    assert est.outlier_clips >= 1
    assert len(events) == 1
    # bounded by the clip (0.5 ms) plus the floor wobble, instead of the 5 ms anomaly
    assert np.max(np.abs(_err(out, truth)[t > 2.0])) <= 0.7


# ── restarts / re-anchoring ──────────────────────────────────────────

def test_qtm_restart_reanchors_cleanly():
    q1, r1, t1 = _trace(20, seed=7)
    # QTM restarts: its clock begins again near zero, with a new offset
    q2, r2, t2 = _trace(20, seed=8, qtm0_us=50_000)
    shift = int(r1[-1] - r2[0]) + 3_000_000          # ROS time continues
    r2 = r2 + shift; t2 = t2 + shift
    q = np.r_[q1, q2]; ros = np.r_[r1, r2]; truth = np.r_[t1, t2]
    out, est, events = _run(q, ros)
    assert [e for _, e in events] == ['init', 'restart']
    assert events[1][0] == len(q1)
    err = _err(out, truth)
    after = np.arange(len(q)) >= len(q1) + int(1.0 * FPS)
    assert np.max(np.abs(err[after])) <= 0.3


def test_qtm_forward_jump_beyond_5s_is_a_restart():
    est = MinLatencyClockSync()
    est.update(1_000_000, 5_000_000_000)
    est.update(1_003_333, 5_003_333_000)
    assert est.update(7_100_000, 11_100_000_000) == 'restart'
    assert est.offset_ns == 11_100_000_000 - 7_100_000_000


@pytest.mark.parametrize('step_ns,expected', [(-50_000_000, 'reanchor_below'),
                                              (1_000_000_000, 'reanchor_above')])
def test_clock_step_far_outside_the_window_reanchors(step_ns, expected):
    q, ros, truth = _trace(40, seed=9, load=_bursty)
    k = int(20 * FPS)
    ros = ros.copy(); ros[k:] += step_ns; truth = truth.copy(); truth[k:] += step_ns
    out, est, events = _run(q, ros)
    assert [e for _, e in events] == ['init', expected]
    t = (q - q[0]) / 1e6
    assert np.max(np.abs(_err(out, truth)[t > 24.0])) <= 1.0


def test_heavy_load_never_triggers_a_reanchor():
    q, ros, _ = _trace(120, load=_heavy, seed=10)
    _, est, events = _run(q, ros)
    assert len(events) == 1 and est.reanchors == 0 and est.restarts == 0


def test_diagnostics_report_the_documented_fields():
    q, ros, _ = _trace(15, load=_bursty, seed=11)
    _, est, _ = _run(q, ros)
    diag = est.diagnostics()
    assert set(diag) == {'sample_count', 'last_excess_latency_ms', 'envelope_minus_output_ms',
                         'window_fill', 'drift_ppm', 'slew_clamps', 'outlier_clips',
                         'reanchors', 'restarts'}
    assert diag['sample_count'] == len(q)
    assert 0.9 <= diag['window_fill'] <= 1.0
    assert diag['last_excess_latency_ms'] >= -0.5
