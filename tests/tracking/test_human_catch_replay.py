"""Scoring function of `tools/probes/human_catch_replay.py` on synthetic input (PWAY-11 P2, U6)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO / 'tools' / 'probes'))
sys.path.insert(0, str(_REPO / 'config' / 'generated'))
sys.path.insert(0, str(_REPO / 'ros_ws' / 'src' / 'jugglebot'))

import human_catch_replay as H  # noqa: E402


def _row(k, outcome, reason='OK', xy=(0.0, 0.0), dist=None, dt=None, cross=True, reaims=None):
    r = dict(k=k, outcome=outcome, reason=reason, cross=cross, truth_xy=np.array(xy))
    if outcome == 'PLANNED':
        r.update(dist_mm=dist, dt_ms=dt, reaims=reaims or {})
    return r


def test_score_rows_counts_planned_declined_and_phantoms():
    rows = [
        _row(0, 'PLANNED', dist=10.0, dt=5.0, xy=(20.0, 0.0), reaims={'requested': 2, 'admitted': 1}),
        _row(1, 'PLANNED', dist=40.0, dt=-5.0, xy=(100.0, 100.0), reaims={'requested': 1,
                                                                          'refused_LIMIT_JERK': 1}),
        _row(2, 'PLANNED', dist=25.0, dt=30.0, xy=(0.0, 0.0)),            # late: not within
        _row(3, 'DECLINED', 'LEAD_TOO_SHORT', xy=(10.0, 10.0)),
        _row(4, 'REFUSED', 'SETTLE_SITE', xy=(400.0, 0.0)),               # out of the box
        _row(5, 'NO_EVENT', 'NO_EVENT', xy=(0.0, 0.0)),
        _row(6, 'DECLINED', 'BUSY', xy=(0.0, 0.0), cross=False),
    ]
    ph = [dict(outcome='PLANNED'), dict(outcome='REFUSED'), dict(outcome='NO_EVENT')]
    s = H.score_rows(rows, ph, cap_mm=35.0, time_tol_ms=20.0)
    assert (s['n_throws'], s['n_cross'], s['n_planned']) == (7, 6, 3)
    assert s['dist_p50'] == 25.0
    assert s['n_within_cap'] == 1            # row 0 only: row 1 is 40 mm, row 2 is 30 ms late
    assert s['n_d10_good'] == 1
    # in reach = crossing and inside the +-150 mm box: rows 0, 1, 2, 3, 5
    assert s['n_in_reach'] == 5 and s['in_reach_planned'] == 3 and s['n_in_reach_d10'] == 1
    assert s['declined'] == {'LEAD_TOO_SHORT': 1, 'SETTLE_SITE': 1, 'NO_EVENT': 1, 'BUSY': 1}
    assert s['reaims'] == {'requested': 3, 'admitted': 1, 'refused_LIMIT_JERK': 1}
    assert (s['n_phantom'], s['n_phantom_planned'], s['n_phantom_refused']) == (3, 1, 1)


def test_score_rows_empty_is_nan_not_error():
    s = H.score_rows([], [])
    assert s['n_planned'] == 0 and np.isnan(s['dist_p50']) and np.isnan(s['frac_within_cap'])


def test_hand_frame_puts_direction_mean_at_home():
    xf = H.Xform('hand', mean_cross=(500.0, 100.0))
    np.testing.assert_allclose(xf.xy((530.0, 90.0, 830.0)), np.array(H.HOME_XY) + [30.0, -10.0])
    np.testing.assert_allclose(xf.xy((500.0, 100.0, 830.0)), H.HOME_XY)
