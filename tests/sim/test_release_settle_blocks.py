"""The `blocks` report of tools/probes/release_settle_bag_probe.py (FEAT-67, learner-efficacy E1) on synthetic
per-Throw rows built here: no bag, no files, fixed seeds, parallel-safe."""
from __future__ import annotations

import importlib.util
import os

import numpy as np

_P = os.path.join(os.path.dirname(__file__), "..", "..", "tools", "probes", "release_settle_bag_probe.py")
_spec = importlib.util.spec_from_file_location("release_settle_bag_probe", _P)
rs = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rs)

REG = rs.BLOCK_REGRESSORS
K_TILT = 4000.0      # mm/s of lateral launch velocity per unit of Platform normal tilt (about 4.4 m/s upward)
K_VC = 1.2           # mm/s per mm/s of Platform centroid velocity (18 mm/s at the 15 mm/s spread below)


def _row(bag, t, dwell=0.27, t_seat=0.3, sens=0.2, site=(0.0, 0.0), **kw):
    r = dict(bag=bag, trel=repr(t), dwell_s=repr(dwell), t_seat=repr(t_seat), sens_t_held=repr(sens),
             site_x=repr(site[0]), site_y=repr(site[1]), release_kind='free_cup', v0x='0', v0y='0')
    r.update({k: repr(v) if isinstance(v, float) else v for k, v in kw.items()})
    return r


def _labels(rows, **kw):
    out, att, idx, lab = rs.label_blocks(rows, **kw)
    return [(float(r['trel']), int(a), int(i), str(b)) for r, a, i, b in zip(out, att, idx, lab)]


def test_block_labels_by_goal():
    rows = [_row('b', 0.0, t_seat=1.3, sens=1.3),                        # one Throw, seated: S
            _row('b', 10.0, t_seat=0.5, sens=1.2),                       # one Throw, seated by the Hand ball sensor: S
            _row('b', 20.0, t_seat=0.3, sens=0.2),                       # one Throw, not seated: other
            _row('b', 30.0, t_seat=1.3, sens=1.3), _row('b', 31.5), _row('b', 33.0),
            _row('b', 50.0, dwell=1.0, t_seat=1.3, sens=1.3),
            _row('b', 51.9, dwell=1.0, t_seat=1.3, sens=1.3),            # from_rest would fire; the goal says D100
            _row('b', 53.8, dwell=1.0, t_seat=1.3, sens=1.3)]
    lab = [x[3] for x in _labels(rows[::-1])]                            # input order does not matter
    assert lab == ['S', 'S', 'other', 'first', 'D27', 'D27', 'first', 'D100', 'D100']


def test_attempt_gap_rule():
    rows = [_row('a', 0.0), _row('a', 2.9), _row('a', 6.0), _row('b', 6.5)]
    assert [x[1:3] for x in _labels(rows)] == [(0, 1), (0, 2), (1, 1), (2, 1)]      # 3.1 s > 3.0 s; a new bag
    assert [x[1:3] for x in _labels(rows, gap_s=6.0)] == [(0, 1), (0, 2), (0, 3), (1, 1)]
    assert [x[3] for x in _labels(rows, skip_attempts=1)][:2] == ['skipped', 'skipped']


def _sitting(sd, seed, platform=True, n_s=30, n_att=6, n_per=12):
    """A synthetic Sitting: block S of n_s one-Throw Attempts and, per Dwell in `sd`, n_att Attempts of n_per
    Throws. Each block's noise is standardised to sample sd exactly sd[block] per axis; a Platform term (tilt on
    x, centroid velocity on y) is added on top when `platform`."""
    rng = np.random.RandomState(seed)
    rows, t = [], 0.0
    plan = [('S', None, 1)] * n_s + [(b, d, n_per) for b, d in (('D27', 0.27), ('D50', 0.50), ('D100', 1.00)) if b in sd] * n_att
    rng.shuffle(plan)
    by_block = {}
    for b, d, n in plan:
        for i in range(n):
            t += 1.5
            r = _row('bag', t, dwell=d if d else 0.27, t_seat=1.3 if i == 0 else 0.3, sens=1.3 if i == 0 else 0.2)
            r.update({k: repr(float(x)) for k, x in zip(REG, rng.randn(len(REG)) * [50, 50, 15, 15, 3e-3, 3e-3, 2e-2, 2e-2])})
            r['entry_r'] = repr(float(rng.uniform(0, 30)))
            rows.append(r)
            by_block.setdefault('S' if b == 'S' else (b if i else 'first'), []).append(r)
        t += 20.0
    for b, rr in by_block.items():
        z = rng.randn(len(rr), 2)
        z = (z - z.mean(0)) / z.std(0, ddof=1) * sd.get(b, 10.0)
        for r, (zx, zy) in zip(rr, z):
            px = K_TILT * float(r['mnx']) if platform else 0.0
            py = K_VC * float(r['mvcy']) if platform else 0.0
            r['v0x'], r['v0y'] = repr(float(zx + px)), repr(float(zy + py))
    return rows


def test_pooled_regression_removes_the_platform_term():
    sigma = 10.0
    rep = rs.blocks_report(_sitting({'S': sigma, 'D27': sigma}, seed=1), n_boot=200)
    for b in ('S', 'D27'):
        r = rep['blocks'][b]
        tol = 2 * sigma / np.sqrt(2 * (r['n'] - 1))                     # twice the sampling se of an sd
        assert min(r['sd_raw']) > sigma + 2 * tol                          # the injected term is there before the fit
        assert np.allclose(r['sd_corr'], sigma, atol=tol), (b, r['sd_corr'], tol)
    assert rep['regression_rank'] == len(REG) and rep['n_raw_only'] == 0


def test_raw_only_rows_are_counted():
    rows = _sitting({'S': 5.0, 'D27': 8.0}, seed=2)
    rows[3]['mnx'] = ''
    rep = rs.blocks_report(rows, n_boot=50)
    assert rep['n_raw_only'] == 1 and rep['n_corrected'] == len(rows) - 1


def test_verdict_seating():
    rep = rs.blocks_report(_sitting({'S': 4.0, 'D27': 10.0, 'D50': 7.0, 'D100': 5.0}, seed=3), n_boot=500)
    assert rep['verdict'] == 'Seating', rep['verdict_clause']
    assert rep['ratio_ref'] == 'S' and len(rep['d27_by_entry']) == 3


def test_verdict_separation():
    rep = rs.blocks_report(_sitting({'S': 15.0, 'D27': 12.0, 'D50': 12.0, 'D100': 12.0}, seed=4), n_boot=500)
    assert rep['verdict'] == 'Separation', rep['verdict_clause']


def test_verdict_undecided_when_the_interval_straddles():
    # D27 at exactly twice S's variance: the bootstrap interval of the deciding ratio straddles 2
    rep = rs.blocks_report(_sitting({'S': 5.0, 'D27': 5.0 * np.sqrt(2.0)}, seed=5), n_boot=500)
    assert rep['verdict'] == 'UNDECIDED' and 'Seating clause' in rep['verdict_clause'], rep['verdict_clause']
    lo, hi = rep['blocks']['D27']['ratio_corr_ci']
    assert lo < 2.0 < hi


def test_verdict_clause_logic():
    assert rs.verdict(None, [])[0] == 'n/a'
    assert rs.verdict([5, 5], [('D27', 3.0, 2.2, 4.0), ('D50', 2.0, 1.5, 3.0)])[0] == 'Seating'
    assert rs.verdict([5, 5], [('D27', 2.0, 1.5, 3.0), ('D50', 3.0, 2.2, 4.0)])[0] == 'UNDECIDED'
    assert rs.verdict([5, 5], [('D27', 3.0, 2.2, 4.0), ('D50', 4.0, 3.0, 5.0)])[0] == 'Mixed'     # not falling
    assert rs.verdict([13, 14], [('D27', 1.0, 0.7, 1.3)])[0] == 'Separation'
    assert rs.verdict([13, 14], [('D27', 1.4, 1.0, 2.0)])[0] == 'UNDECIDED'
    assert rs.verdict([13, 14], [('D27', 2.5, 1.8, 3.5)])[0] == 'Mixed'
    assert rs.verdict([9, 9], [('D27', 1.5, 1.0, 2.0)])[0] == 'Mixed'


def test_no_settled_block_and_determinism():
    rows = [r for r in _sitting({'D27': 10.0}, seed=6, n_s=0)]
    a = rs.blocks_report(rows, n_boot=300)
    b = rs.blocks_report(rows, n_boot=300)
    assert a['verdict'] == 'n/a' and a['ratio_ref'] == 'first'
    assert rs.format_blocks(a) == rs.format_blocks(b)
    assert 'no settled block: verdict not available' in rs.format_blocks(a)
