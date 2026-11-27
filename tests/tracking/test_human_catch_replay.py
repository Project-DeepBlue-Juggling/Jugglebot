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


# ---- BUG-126: the time window bounds the read; a run with nothing to report fails loudly ----

import pickle  # noqa: E402
import time  # noqa: E402

import pytest  # noqa: E402

import human_throw_bench as B  # noqa: E402

_WALL0 = time.mktime((2026, 11, 20, 12, 49, 25, 0, 0, -1)) + 0.0


def _clock(h, m, s):
    return time.mktime((2026, 11, 20, h, m, 0, 0, 0, -1)) + s


def test_window_bounds_the_bag_read(monkeypatch, tmp_path):
    seen = {}

    def fake_iter(mcap_path, topics, start_s=None, end_s=None):
        seen['args'] = (start_s, end_s)
        return iter([])

    monkeypatch.setattr(B.R, '_find_mcap', lambda bag: tmp_path / 'x.mcap')
    monkeypatch.setattr(B.R, '_iter_bag', fake_iter)
    monkeypatch.setattr(B, 'bag_start_wall', lambda bag: _WALL0)
    with pytest.raises(SystemExit) as e:                  # nothing in the window: loud, not silent
        H.load_frames(tmp_path, None, '12:53:20', '12:56:45')
    assert 'no /mocap_data frames' in str(e.value)
    assert seen['args'] == (_clock(12, 53, 20), _clock(12, 56, 45))


def test_default_read_is_unbounded(monkeypatch, tmp_path):
    seen = {}
    monkeypatch.setattr(B.R, '_find_mcap', lambda bag: tmp_path / 'x.mcap')
    monkeypatch.setattr(B.R, 'make_tracker', lambda: type('T', (), {})())
    monkeypatch.setattr(B, '_REPO', tmp_path)             # keep the whole-bag cache out of the repo

    def fake_iter(mcap_path, topics, start_s=None, end_s=None):
        seen['args'] = (start_s, end_s)
        return iter([])
    monkeypatch.setattr(B.R, '_iter_bag', fake_iter)
    assert B.load_frames(tmp_path / 'bag', with_wall=True) == []
    assert seen['args'] == (None, None)


def _pkl(tmp_path, with_wall=True):
    rows = []
    for i in range(10):
        r = (100.0 + i, [np.zeros(3)], ['a'])
        rows.append(r + (_WALL0 + 60.0 * i,) if with_wall else r)
    p = tmp_path / 'f.pkl'
    p.write_bytes(pickle.dumps(rows))
    return p


def test_window_applies_to_frames_pickle_with_wall_times(tmp_path):
    p = _pkl(tmp_path)
    fr, wall0 = H.load_frames('bag', p, '12:51:25', '12:53:25')   # +120 s .. +240 s -> i = 2..4
    assert [f[0] for f in fr] == [102.0, 103.0, 104.0]
    assert wall0 == pytest.approx(_WALL0 + 120.0)


def test_window_on_pickle_without_wall_times(tmp_path):
    p = _pkl(tmp_path, with_wall=False)
    with pytest.raises(SystemExit) as e:                  # clock time cannot be mapped: say so
        H.load_frames('bag', p, '12:51:25', None)
    assert 'carries no wall times' in str(e.value)
    fr, _ = H.load_frames('bag', p, '2', '4')             # bare seconds map by frame time
    assert [f[0] for f in fr] == [102.0, 103.0, 104.0]


def test_run_with_nothing_to_report_exits_nonzero(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(H, 'load_frames', lambda *a: ([(0.0, [], [])], None))
    monkeypatch.setattr(H, 'build_stream', lambda fr: dict(events=[], truth=[]))
    monkeypatch.setattr(H, 'run_bag', lambda *a: ([], [], []))
    rc = H.main([str(tmp_path / 'bag'), '--frame', 'hand', '--out-dir', str(tmp_path / 'o'),
                 '--t-start', '12:53:20'])
    assert rc == 1
    assert 'no truth Throws' in capsys.readouterr().err
    assert (tmp_path / 'o' / 'bag_125320_replay.md').exists()   # the window is in the file name


def test_d10_xy_bar_is_35_mm_decided_2026_11_21():
    assert H.D10_XY_MM == 35.0
    rows = [_row(0, 'PLANNED', dist=34.0, dt=5.0, xy=(0.0, 0.0)),
            _row(1, 'PLANNED', dist=36.0, dt=5.0, xy=(0.0, 0.0))]
    assert H.score_rows(rows, [])['n_d10_good'] == 1


# -- transport delay (PWAY-11 P3 readiness, 2026-11-27) --------------------
# The live `/human_throws` reached the bag recorder a median 43.8 ms after its frame stamp on
# the 2026-11-27 lob bag; at 0 ms the replay plans from a revision no live node has yet.

def _ev(stamp, rev=0):
    return dict(track_id=1, flight_seq=0, revision=rev, stamp=stamp, landing=[0.0, 0.0, 830.0],
                vel=[0.0, 0.0, -5000.0], t_land=stamp + 0.6, fit_age=0.0, final=False, k=0)


def test_delay_events_moves_only_the_stamp():
    evs = [_ev(10.0), _ev(10.04, rev=1)]
    out = H.delay_events(evs, 0.044)
    assert [e['stamp'] for e in out] == pytest.approx([10.044, 10.084])
    assert [e['t_land'] for e in out] == [e['t_land'] for e in evs]     # the Fit is untouched
    assert [e['fit_age'] for e in out] == [0.0, 0.0]
    assert evs[0]['stamp'] == 10.0                                      # copies, input unchanged
    with pytest.raises(ValueError):
        H.delay_events(evs, -0.01)


def test_transport_ms_reaches_the_run_and_names_the_report(monkeypatch, tmp_path):
    seen = {}
    monkeypatch.setattr(H, 'load_frames', lambda *a: ([(0.0, [], [])], None))
    monkeypatch.setattr(H, 'build_stream', lambda fr: dict(
        events=[_ev(5.0)], truth=[dict(t_cross=5.6)], t0=0.0, span=1.0))

    def fake_run_bag(S, *a):
        seen['stamps'] = [e['stamp'] for e in S['events']]
        return [], [], {}
    monkeypatch.setattr(H, 'run_bag', fake_run_bag)
    H.main([str(tmp_path / 'bag'), '--frame', 'hand', '--out-dir', str(tmp_path / 'o'),
            '--transport-ms', '44'])
    assert seen['stamps'] == pytest.approx([5.044])
    rep = tmp_path / 'o' / 'bag_tr44ms_replay.md'
    assert rep.exists()
    assert 'transport delay 44.0 ms' in rep.read_text()
    cached = pickle.load(open(tmp_path / 'o' / 'bag_stream.pkl', 'rb'))
    assert cached['events'][0]['stamp'] == 5.0                          # the cache stays undelayed


def _one_throw(fit_x, truth_x, t_land=10.62):
    """One synthetic Throw: two revisions of one key (both at ``fit_x``) and its truth crossing,
    in the Hand frame with the direction mean at home (so the frame is the identity)."""
    evs = [dict(stamp=s, revision=r, track_id=1, flight_seq=0, landing=(fit_x, 0.0, 830.0),
                vel=(0.0, 0.0, -4580.0), t_land=t_land, fit_age=0.0, final=False, k=0)
           for r, s in ((0, 9.9), (1, 10.0))]
    truth = [dict(t_cross=t_land, xy_cross=np.array([truth_x, 0.0]), hand=0)]
    return evs, truth


def test_the_replay_scores_the_policys_edge_aim():
    """The 2026-11-27 rule through the replay's own chain (policy, compile, the real gate): a
    Fit 10 mm outside the box is planned at the edge and scored there; with the edge aim off
    (``edge_aim_mm`` 0) the same Throw is refused ``SETTLE_SITE`` as before."""
    xf = H.Xform('hand', mean_cross=H.HOME_XY)
    evs, truth = _one_throw(160.0, 140.0)
    rows, _ph, log = H.run_attempt(evs, truth, xf, {})
    assert log['edge_aims'] == 1
    assert rows[0]['outcome'] == 'PLANNED' and rows[0]['edge_aim_outside_mm'] == 10.0
    assert abs(rows[0]['dist_mm'] - 10.0) < 1e-9          # Cup at 150, Ball at 140
    rows, _ph, log = H.run_attempt(evs, truth, xf, dict(edge_aim_mm=0.0))
    assert log['edge_aims'] == 0
    assert (rows[0]['outcome'], rows[0]['reason']) == ('REFUSED', 'SETTLE_SITE')
