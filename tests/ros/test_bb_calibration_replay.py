"""BB calibration replay probe: fixture re-solve, bands arithmetic, CLI smoke.

(a) ``tools/probes/data/bb_calibration_replay_fixture.json`` is ONE sweep's raw inputs (703 mocap frames,
    stamped yaw, the 4 shelf markers) captured by the replay probe. Re-solving it through the same
    ``run_calibration`` / ``estimate_base_pose`` / ``bb_in_base`` calls the replay (and ``mocap_node``) makes must
    give back the recorded outputs. The solver is deterministic; the only slack is the emitter's 4-decimal input
    rounding (measured 2026-10-15: yaw 8e-7 deg, kappa 2e-7 deg, residual 6.5e-6 mm, lag 4e-6 ms), so the
    tolerances below are 10-50x that, still 3-4 orders below any gate (kappa gate 0.09 deg, residual cap 0.5 mm).
    Recipe (the emitter writes it into the fixture's top-level "recipe" key)::

        python tools/probes/bb_calibration_replay.py --bags "$HOME/Desktop/rosbags/2026-10-10_12-26-22" \\
            --emit-fixture 2026-10-10_12-26-22:3 --out tools/probes/data/bb_calibration_replay_fixture.json

    The fixture is a 2026-10-10 sweep: the test pins the solver, not that night's verdict.
(b) ``bb_calibration_bands.summarise`` grouping arithmetic on synthetic rows.
(c) the fixture loads and carries the recipe; the replay CLI parses ``--emit-fixture``.
"""
from __future__ import annotations

import json
import math
import os
import re
import sys

import numpy as np
import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

import tools.probes.bb_calibration_replay as R  # noqa: E402

FIXTURE = os.path.join(REPO, 'tools', 'probes', 'data', 'bb_calibration_replay_fixture.json')


@pytest.fixture(scope='module')
def fixture():
    with open(FIXTURE) as fh:
        return json.load(fh)


def test_fixture_loads_and_has_recipe(fixture):
    assert '--emit-fixture 2026-10-10_12-26-22:3' in fixture['recipe']
    assert set(fixture) >= {'inputs', 'row', 'recipe'}
    assert fixture['row']['verdict_intrinsic'] == 'ACCEPTED'
    assert len(fixture['inputs']['marker_frames']) == 731 and fixture['row']['n_frames'] == 703
    assert os.path.getsize(FIXTURE) < 1.5e6


def test_replay_cli_parses_emit_fixture(monkeypatch, tmp_path, capsys):
    """main() parses --emit-fixture/--out (no bag needed: the bag glob is empty, so it exits 2, 'not found')."""
    seen = {}

    def fake_write(outdir, rows, unmatched, args, elapsed):
        seen['args'] = args
        return 'score', 0, 0
    monkeypatch.setattr(R, 'parse_launch_logs', lambda d: [])
    monkeypatch.setattr(R, 'write_outputs', fake_write)
    monkeypatch.setattr(sys, 'argv', ['bb_calibration_replay.py', '--bags', str(tmp_path / 'none*'),
                                      '--emit-fixture', 'some_bag:3', '--out', str(tmp_path / 'f.json')])
    assert R.main() == 2
    assert seen['args'].emit_fixture == 'some_bag:3'
    assert seen['args'].out == str(tmp_path / 'f.json')
    assert not (tmp_path / 'f.json').exists()


def test_fixture_resolves_to_recorded_outputs(fixture):
    inp, row = fixture['inputs'], fixture['row']
    template = R.load_marker_template(os.path.join(R.RES_DIR, 'bb_marker_template.json'))
    base_model = R.load_base_frame(os.path.join(R.RES_DIR, 'bb_base_frame.json'))
    assert inp['params']['bb_yaw_source'] == 'auto' and row['yaw_source'] == 'stamped'
    assert len(inp['stamped_yaw']) >= R.MIN_STAMPED_YAW_SAMPLES

    res = R.run_calibration(
        calibration_data={int(k): [np.array(p) for p in v] for k, v in inp['calibration_data'].items()},
        yaw_readings_deg=inp['yaw_readings_deg'],
        pitch_z_offset_mm=inp['params']['pitch_z_offset_mm'],
        marker_frames=[(t, np.array(p)) for t, p in inp['marker_frames']],
        yaw_samples=[tuple(s) for s in inp['stamped_yaw']],
        template=template, yaw_source='stamped')
    est = res.yaw_estimate
    assert res.yaw_method == 'constellation'
    assert math.degrees(res.yaw_offset_rad) == pytest.approx(row['yaw_offset_deg'], abs=1e-5)
    assert res.yaw_offset_std_deg == pytest.approx(row['sigma_deg'], abs=1e-5)
    assert est.lag_s * 1e3 == pytest.approx(row['lag_ms'], abs=1e-4)
    assert est.template_residual_mm == pytest.approx(row['template_residual_mm'], abs=3e-5)
    assert est.sweep_end_deg == pytest.approx(row['sweep_end_deg'], abs=1e-9)
    assert np.asarray(res.bb_position_mm) == pytest.approx(np.asarray(row['axis_point_mm']), abs=1e-3)

    pose = R.estimate_base_pose([(t, np.array(p)) for t, p in inp['base_points']], base_model.template_mm)
    kappa, p_b = R.bb_in_base(pose, math.degrees(res.yaw_offset_rad), res.bb_position_mm)
    assert pose.heading_deg == pytest.approx(row['base_heading_deg'], abs=1e-5)
    assert pose.residual_mm == pytest.approx(row['base_residual_mm'], abs=1e-4)
    assert kappa == pytest.approx(row['kappa_deg'], abs=1e-5)
    assert p_b == pytest.approx(np.asarray(row['p_b_mm']), abs=1e-3)


# ---------------------------------------------------------------- (a2) Phase 1 path: bumped BB

def _phase1(fixture, rot_deg, n_records=1):
    """The fixture sweep through the replay's Phase 1 path (``NodeSim.phase1``: assess_base_frame ->
    assess_sweep_vs_reference -> append_record -> format_outcome_body) against an in-memory state holding ONE
    epoch at the fixture's own recorded kappa, seeded with ``n_records`` identical records (0 = empty pool).
    ``rot_deg`` rotates every BB marker position about the vertical
    through the fitted axis point (the 2 mm bump of the Phase 0 entry = 0.80 deg)."""
    import argparse
    inp, row = fixture['inputs'], fixture['row']
    template = R.load_marker_template(os.path.join(R.RES_DIR, 'bb_marker_template.json'))
    base_model = R.load_base_frame(os.path.join(R.RES_DIR, 'bb_base_frame.json'))
    c, sn = math.cos(math.radians(rot_deg)), math.sin(math.radians(rot_deg))
    ax = np.asarray(row['axis_point_mm'], float)

    def rot(p):
        p = np.array(p, float)
        d = p[:2] - ax[:2]
        out = p.copy()
        out[0], out[1] = ax[0] + c * d[0] - sn * d[1], ax[1] + sn * d[0] + c * d[1]
        return out
    res = R.run_calibration(
        calibration_data={int(k): [rot(p) for p in v] for k, v in inp['calibration_data'].items()},
        yaw_readings_deg=inp['yaw_readings_deg'], pitch_z_offset_mm=inp['params']['pitch_z_offset_mm'],
        marker_frames=[(t, np.array([rot(q) for q in np.array(p)])) for t, p in inp['marker_frames']],
        yaw_samples=[tuple(x) for x in inp['stamped_yaw']], template=template, yaw_source='stamped')
    gate = R.GateState(base_model)
    seeds = [R.KappaRecord(kappa_deg=row['kappa_deg'], sigma_deg=0.04, axis_point_b_mm=tuple(row['p_b_mm']),
                           pin_deg=template.pinned_yaw_offset_deg, accepted_at='2026-10-10T00:00:00Z')
             for _ in range(n_records)]
    gate.state = R.seed_base_state(seeds, None)
    sim = R.NodeSim(argparse.Namespace(bb_yaw_source='auto', clip=None), template, base_model, gate)
    sim.t_start = 0.0
    sim.base_points = [(t, np.array(p)) for t, p in inp['base_points']]
    out = {'t_start': 0.0}
    sim.phase1(out, None, res, 1.0)
    out['template_residual_mm'] = res.yaw_estimate.template_residual_mm   # the solver's, for the callers
    return out, gate


def test_phase1_bumped_bb_opens_epoch_but_unbumped_is_silent(fixture):
    """A 2 mm bump of the BB (0.80 deg of kappa) is ACCEPTED with KAPPA_EXCURSION and a NEW epoch; the same
    sweep unrotated against the same state carries no flag and stays in the epoch (nothing refuses on history)."""
    quiet, g0 = _phase1(fixture, 0.0)
    assert quiet['advisory_flags'] == '' and quiet['epoch_opened'] is False
    assert quiet['epoch_id'] == 0 and len(g0.state['epochs']) == 1
    assert abs(quiet['delta_kappa_deg']) < 1e-3

    bumped, g1 = _phase1(fixture, 0.80)
    assert 'KAPPA_EXCURSION' in bumped['advisory_flags'].split(';')
    assert bumped['epoch_opened'] is True and bumped['epoch_id'] == 1
    assert abs(abs(bumped['delta_kappa_deg']) - 0.80) < 0.01
    assert len(g1.state['epochs']) == 2 and len(g1.state['records']) == 2
    assert 'ADVISORY' in bumped['outcome_line'] and 'K1' in bumped['outcome_line']


# ---------------------------------------------------------------- (b) bands

def _row(bag, idx, t, kappa, yaw, heading, intrinsic='ACCEPTED', origin=(0.0, 0.0, 0.0)):
    return {'bag': bag, 'sweep_idx': idx, 't_start': t, 'verdict_intrinsic': intrinsic, 'kappa_deg': kappa,
            'yaw_offset_deg': yaw, 'base_heading_deg': heading, 'base_origin_mm': list(origin),
            'base_seen': True, 'lag_ms': 2.0, 'template_residual_mm': 0.3, 'base_residual_mm': 0.05,
            'sigma_deg': 0.04, 'sweep_end_deg': 122.0, 'axis_point_mm': [0.0, 0.0, 0.0],
            'p_b_mm': [0.0, 0.0, 0.0]}


def test_bands_summarise_grouping_and_exclusion():
    """Sittings are bags; kappa epochs are the module's fixed date-based ones (A,B in K0 = 10-10, C in K2 =
    10-13 22:00 on); WORLD epochs (yaw) split on a base-pose jump (> 2 mm / 0.05 deg), here between B and C."""
    bands = pytest.importorskip('tools.probes.bb_calibration_bands')
    A, B, C = '2026-10-10_09-00-00', '2026-10-10_12-00-00', '2026-10-13_22-30-00'
    rows = [_row(A, 1, 1.0, 179.30, 0.30, -179.05), _row(A, 2, 2.0, 179.32, 0.32, -179.05),
            _row(A, 3, 3.0, 179.34, 0.34, -179.05),
            _row(B, 1, 11.0, 179.40, 0.40, -179.05), _row(B, 2, 12.0, 179.42, 0.42, -179.05),
            _row(B, 3, 13.0, 179.44, 0.44, -179.05),
            _row(C, 1, 21.0, 179.60, 1.70, -177.50, origin=(5.0, 0.0, 0.0)),
            _row(C, 2, 22.0, 179.62, 1.72, -177.50, origin=(5.0, 0.0, 0.0)),
            _row(C, 3, 23.0, 999.0, 99.0, -177.50, intrinsic='CIRCLE_CENTRES', origin=(5.0, 0.0, 0.0))]
    out = bands.summarise(rows)
    assert out['n_rows'] == 9 and out['n_accepted_intrinsic'] == 8 and out['n_sittings'] == 3
    # refused-intrinsic row never contributes
    qs = out['quantities']
    assert qs['kappa_deg']['all_sweeps']['max'] == pytest.approx(179.62)
    assert qs['yaw_offset_deg']['all_sweeps']['max'] == pytest.approx(1.72)

    k = out['quantities']['kappa_deg']
    assert k['all_sweeps']['n'] == 6 + 2
    per = k['per_sitting']
    assert per[A]['mean'] == pytest.approx(179.32) and per[B]['mean'] == pytest.approx(179.42)
    assert per[C]['mean'] == pytest.approx(179.61) and per[C]['n'] == 2
    means = np.array([179.32, 179.42, 179.61])
    assert k['cross_sitting']['mean'] == pytest.approx(float(means.mean()))
    assert k['cross_sitting']['sd'] == pytest.approx(float(means.std(ddof=1)))
    assert k['cross_sitting']['n'] == 3
    assert k['within_sitting_pooled_sd'] == pytest.approx(math.sqrt((2 * 0.0008 + 0.0002) / 5))
    # kappa epochs (fixed by bag date)
    ke = k['kappa_epochs']['per_epoch']
    assert ke['K0 (10-10)']['sweeps']['mean'] == pytest.approx(179.37)
    assert ke['K0 (10-10)']['sweeps']['n'] == 6
    assert ke['K2 (10-13 22:00 on)']['sweeps']['mean'] == pytest.approx(179.61)
    # world epochs (base-pose jump between B and C)
    assert list(out['world_epochs']) == ['W0', 'W1']
    ye = out['quantities']['yaw_offset_deg']['epochs']['per_epoch']
    assert ye['W0']['sweeps']['mean'] == pytest.approx(0.37) and ye['W0']['sweeps']['n'] == 6
    assert ye['W1']['sweeps']['mean'] == pytest.approx(1.71) and ye['W1']['sweeps']['n'] == 2


# ---------------------------------------------------------------- (a3) Phase 1: pool-size independence

@pytest.mark.parametrize('n_records', [0, 1, 5, 50])
def test_template_residual_verdict_is_independent_of_pool_size(fixture, monkeypatch, n_records):
    """Pins plan Phase 2 'the same data gives the same residual verdict on any pool size'. The fixture's template
    residual is ~0.3 mm (the row value), under the shipped 1.0 mm TEMPLATE_RESIDUAL advisory band, so no pool of
    0/1/5/50 seeded kappa records may flag it: the residual is a property of the sweep, never of the history.
    With the band patched to 0.25 mm (below the residual) EVERY pool size must flag it and the outcome line must
    carry 'template residual'."""
    import jugglebot.bb_calibration as BC
    row = fixture['row']
    out, _ = _phase1(fixture, 0.0, n_records)
    assert out['template_residual_mm'] == pytest.approx(row['template_residual_mm'], abs=3e-5)
    assert 0.2 < row['template_residual_mm'] < 0.4
    assert 'TEMPLATE_RESIDUAL' not in out['advisory_flags'].split(';')
    monkeypatch.setattr(BC, 'TEMPLATE_RESIDUAL_ADVISORY_MM', 0.25)
    tight, _ = _phase1(fixture, 0.0, n_records)
    assert tight['template_residual_mm'] == out['template_residual_mm']
    assert 'TEMPLATE_RESIDUAL' in tight['advisory_flags'].split(';')
    assert 'template residual' in tight['outcome_line']


# ---------------------------------------------------------------- (a4) Phase 2 measurements

def _solve_fixture(fixture, marker_frames=None):
    """Production solve of the fixture -> (inputs, template, frames, yaw, result, track)."""
    inp = fixture['inputs']
    template = R.load_marker_template(os.path.join(R.RES_DIR, 'bb_marker_template.json'))
    frames = [(t, np.array(p)) for t, p in (marker_frames or inp['marker_frames'])]
    yaw = [tuple(x) for x in inp['stamped_yaw']]
    res = R.run_calibration(
        calibration_data={int(k): [np.array(p) for p in v] for k, v in inp['calibration_data'].items()},
        yaw_readings_deg=inp['yaw_readings_deg'], pitch_z_offset_mm=inp['params']['pitch_z_offset_mm'],
        marker_frames=frames, yaw_samples=yaw, template=template, yaw_source='stamped')
    return inp, template, frames, yaw, res, R.track_constellation(frames, template)


def test_phase2_marker_stats_on_fixture(fixture):
    """The Phase 2 measurements on the fixture sweep (a 2026-10-10 sweep). Measured 2026-10-16 on this fixture (recorded for the reader; the assertions below are bounds, not these values):
    label fractions Q1..Q7 = 0.82 1.00 0.59 1.00 0.45 0.88 0.72; per-marker vis 0.79 0.99 0.91 1.00 0.67 0.89
    0.61, rms 0.54-0.85 mm; worst pair Q1-Q4 0.2895 mm (equal to the solver's template residual: it IS that
    statistic, by pair); out/ret yaw 0.318 / 0.293 deg against the production 0.284 deg (split 0.024 deg)."""
    inp, template, frames, yaw, res, trk = _solve_fixture(fixture)
    est, row = res.yaw_estimate, fixture['row']
    ct = {int(k): [1] * len(v) for k, v in inp['calibration_data'].items()}
    fr = R.marker_label_fractions(ct, len(frames))
    assert len(fr) == 7 and all(0.0 <= f <= 1.0 for f in fr) and max(fr) >= 0.9
    mask = R.moving_mask(trk, yaw, est.lag_s)
    ms = R.marker_fit_stats(trk, mask)
    assert len(ms) == 7
    for m in ms:
        assert 0.0 <= m['vis'] <= 1.0
        if math.isfinite(m['rms_mm']):
            assert m['rms_mm'] < 1.0
    name, val = R.worst_pair(trk, mask)
    assert re.fullmatch(r'Q[1-7]-Q[1-7]', name)
    assert val == pytest.approx(est.template_residual_mm, abs=1e-6)       # the SAME statistic
    assert val == pytest.approx(row['template_residual_mm'], abs=3e-5)    # and the recorded row (4-dp input rounding)
    yo, yr = R.yaw_by_direction(trk, yaw, template, est.lag_s, est.phi_raw_deg)
    prod = math.degrees(res.yaw_offset_rad)
    assert abs(R.wrap_deg(yo - prod)) < 0.25 and abs(R.wrap_deg(yr - prod)) < 0.25
    assert abs(R.wrap_deg(yo - yr)) < 0.25


def _window_all(frames, extra=None):
    """window_all rows for the fixture frames: the BB points get placeholder labels 'Ball Butler - i' (the
    fixture keeps no per-point labels), plus ``extra`` {stamp: xyz} as UNLABELLED points."""
    out = []
    for t, p in frames:
        p = np.asarray(p, float)
        labels = ['Ball Butler - %d' % (i + 1) for i in range(len(p))]
        pts = p
        if extra and t in extra:
            pts = np.vstack([p, np.asarray(extra[t], float).reshape(-1, 3)])
            labels = labels + [''] * (len(pts) - len(p))
        out.append((t, labels, pts.astype(np.float32)))
    return out


def test_labelfree_pickup_recovers_an_unlabelled_marker_and_ignores_shelf_points(fixture):
    """Mechanism test for the label-free pickup, a case the 2026-10-16 bags never showed (0 of ~80k near-axis
    unlabelled points were BB markers). (i) The extra unlabelled pool = the fixture's shelf markers: the
    label-free solve equals the production yaw within 1e-3 deg and no pool point is within 3 mm of a posed BB
    marker. (ii) QTM2 (index 1, labelled ~100 %) is stripped from every labelled frame (the point nearest the
    posed QTM2, within 3 mm) and put in the unlabelled pool: the labelled-only solve on the stripped frames
    still succeeds, and the label-free solve on stripped + pool recovers QTM2 (visibility back within 0.02)
    and the original yaw within 5e-3 deg."""
    inp, template, frames, yaw, res, trk = _solve_fixture(fixture)
    est = res.yaw_estimate
    prod = math.degrees(res.yaw_offset_rad)
    posed = {float(t): (Rm, o) for t, Rm, o in zip(trk.t, trk.R, trk.origin)}
    ax = est.axis_point_mm
    mask = R.moving_mask(trk, yaw, est.lag_s)
    vis0 = [m['vis'] for m in R.marker_fit_stats(trk, mask)]

    # (i) shelf markers as the unlabelled pool (radius large enough to include them: they must still be ignored)
    shelf = {float(t): np.array(p) for t, p in inp['base_points']}
    wa = _window_all(frames, extra=shelf)
    lf, st = R.labelfree_frames(wa, -1e18, 1e18, ax, 1e6, posed, template.points_mm)
    assert st['n_unlab'] > 0 and st['n_within_3mm'] == 0
    v = R.solve_variant(lf, yaw, 'stamped', template)
    assert not isinstance(v, str)
    assert abs(R.wrap_deg(math.degrees(v[0].yaw_offset_rad) - prod)) < 1e-3

    # (ii) strip QTM2 from every frame and put it in the unlabelled pool
    stripped, pool = [], {}
    for t, p in frames:
        Rm, o = posed[float(t)] if float(t) in posed else (None, None)
        if Rm is None:
            stripped.append((t, p))
            continue
        q2 = o + Rm @ template.points_mm[1]
        d = np.linalg.norm(p - q2, axis=1)
        j = int(np.argmin(d))
        if d[j] <= 3.0:
            pool[t] = p[j]
            p = np.delete(p, j, axis=0)
        stripped.append((t, p))
    assert len(pool) > 0.9 * len(frames)
    assert not isinstance(R.solve_variant(stripped, yaw, 'stamped', template), str)
    wa2 = [(t, ['Ball Butler - %d' % (i + 1) for i in range(len(p))], np.asarray(p, np.float32)) for t, p in stripped]
    wa2 = [(t, lab + ([''] if t in pool else []),
            np.vstack([pts, np.asarray(pool[t], np.float32).reshape(1, 3)]) if t in pool else pts)
           for t, lab, pts in wa2]
    lf2, st2 = R.labelfree_frames(wa2, -1e18, 1e18, ax, R.LABELFREE_RADIUS_MM, posed, template.points_mm)
    assert st2['n_within_3mm'] > 0.9 * len(frames)        # these ARE BB markers the labels missed
    out2 = R.solve_variant(lf2, yaw, 'stamped', template)
    assert not isinstance(out2, str)
    est2, trk2 = out2
    vis2 = [m['vis'] for m in R.marker_fit_stats(trk2, R.moving_mask(trk2, yaw, est2.lag_s))]
    assert abs(vis2[1] - vis0[1]) < 0.02
    assert abs(R.wrap_deg(math.degrees(est2.yaw_offset_rad) - prod)) < 5e-3


def test_replay_cli_parses_subset_and_loo(monkeypatch, tmp_path):
    """main() parses --subset 1,2,4,6,7 and --loo (empty bag glob: main() returns 0 after the faked write_outputs captures args)."""
    seen = {}

    def fake_write(outdir, rows, unmatched, args, elapsed):
        seen['args'] = args
        return 'score', 0, 0
    monkeypatch.setattr(R, 'parse_launch_logs', lambda d: [])
    monkeypatch.setattr(R, 'write_outputs', fake_write)
    monkeypatch.setattr(sys, 'argv', ['bb_calibration_replay.py', '--bags', str(tmp_path / 'none*'),
                                      '--subset', '1,2,4,6,7', '--loo'])
    assert R.main() == 0
    assert seen['args'].subset == '1,2,4,6,7' and seen['args'].loo is True
    monkeypatch.setattr(sys, 'argv', ['bb_calibration_replay.py', '--bags', str(tmp_path / 'none*')])
    assert R.main() == 0
    assert seen['args'].subset is None and seen['args'].loo is False
