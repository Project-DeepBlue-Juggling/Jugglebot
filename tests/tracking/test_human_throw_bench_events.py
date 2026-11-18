"""Scoring function of `tools/probes/human_throw_bench.py --events` on synthetic input (PWAY-11 P1, U3)."""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO / 'tools' / 'probes'))
sys.path.insert(0, str(_REPO / 'ros_ws' / 'src' / 'jugglebot'))

import human_throw_bench as B  # noqa: E402


def _truth(t_rel, t_cross, xy=(100.0, 50.0)):
    return dict(t_rel=t_rel, t_end=t_cross + 0.2, t_cross=t_cross, xy_cross=np.array(xy))


def _ev(tid, seq, rev, stamp, xy, t_land, start, final=False, speed=900.0):
    return SimpleNamespace(track_id=tid, flight_seq=seq, revision=rev, stamp=stamp,
                           flight_start=start, landing_position=np.array([xy[0], xy[1], 830.0]),
                           landing_time=t_land, final=final, start_speed_mm_s=speed)


def test_score_events_leads_errors_and_phantoms():
    th = [_truth(10.0, 11.0), _truth(20.0, 21.0), _truth(30.0, 31.0)]
    serve = {0: {1}, 1: {2}}                       # Throw 2 is never served
    ev = [
        _ev(1, 0, 0, 10.30, (130.0, 50.0), 11.010, 10.15),      # first: 30 mm, +10 ms, lead 0.70
        _ev(1, 0, 1, 10.60, (105.0, 50.0), 11.002, 10.15),      # last before 0.30 s: 5 mm
        _ev(1, 0, 2, 10.90, (100.0, 50.0), 11.000, 10.15, final=True),   # after the deadline
        _ev(2, 0, 0, 20.80, (100.0, 120.0), 21.0, 20.2, final=True),    # frozen-only, 70 mm off
        _ev(9, 0, 0, 50.0, (0.0, 0.0), 51.0, 49.9, speed=700.0),         # phantom, 18 s from a release
        _ev(1, 1, 0, 12.0, (0.0, 0.0), 13.0, 11.4),                      # re-Throw of id 1 outside every Throw window
    ]
    out = B.score_events(ev, th, serve, span_s=60.0)
    s = out['summary']
    rows = out['rows']
    assert (s['n_served'], s['n_served_cross'], s['n_throws']) == (2, 2, 3)
    assert abs(rows[0]["lead"] - 0.70) < 1e-9
    assert abs(rows[0]['err_xy'] - 30.0) < 1e-9 and abs(rows[0]['err_t'] - 10.0) < 1e-6
    assert rows[0]['n_rev'] == 3 and not rows[0]['frozen_only']
    # last revision stamped at or before crossing - 0.30 s (10.70): revision 1
    assert abs(rows[0]['last_err_xy'] - 5.0) < 1e-9
    assert rows[1]['frozen_only'] and abs(rows[1]['err_xy'] - 70.0) < 1e-9
    assert rows[1]['last_err_xy'] is None            # first event came after crossing - 0.30 s
    assert not rows[2]['served']
    assert s['n_frozen_only'] == 1 and s['n_good_first'] == 1
    # phantoms: id 9 (idle) and the id-1 event at t=11.9 (no Throw window, 1.4 s from a release)
    assert s['n_phantom'] == 2 and s['n_phantom_idle'] == 1
    assert abs(s['phantom_per_min'] - 2.0) < 1e-9


def test_classify_flights_start_speed_vs_truth():
    th = [_truth(10.0, 11.0)]
    fl = {(1, 0): dict(speed=900.0, start=10.1), (1, 1): dict(speed=300.0, start=10.4),
          (5, 0): dict(speed=40.0, start=3.0), (6, 0): dict(speed=None, start=None)}
    out = B.classify_flights(fl, th, {0: {1}})
    by = {(f['id'], f['seq']): f for f in out}
    assert len(out) == 3                              # the flight with no start is dropped
    assert by[(1, 0)]['k'] == 0 and not by[(1, 0)]['rejected']
    assert by[(1, 1)]['k'] == 0 and by[(1, 1)]['rejected']      # a real re-Throw the rule would lose
    assert by[(5, 0)]['k'] is None and by[(5, 0)]['rejected']


def test_d2_variants_names_and_values():
    v = B.d2_variants()
    assert v['default'].rms_mm == 6.0 and v['default'].stable_mm == 30.0
    assert v['rms4/agree50'].rms_mm == 4.0 and v['rms4/agree50'].stable_mm == 50.0
    assert v['no-speed-rule'].min_start_speed_mm_s == 0.0
