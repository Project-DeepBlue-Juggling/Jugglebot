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
    monkeypatch.setattr(R, 'parse_launch_logs', lambda d: ([], []))
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
