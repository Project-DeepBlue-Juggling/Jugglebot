"""Tests for ``tools/toss_cal_analyse.py`` — the phase-2c corpus/map analyser.

The analyser never writes a calibration, so these tests are not about refusals;
they are about the four things a *wrong* analyser gets silently wrong and an
operator cannot see:

1. **The quiver points the wrong way.** The stored quantity is a commanded
   TILT and the operator checks it against a ball on the floor, so the arrows
   are drawn in LANDING space (``J·aim``). Since ``J`` is a 90° rotation,
   plotting ``(rx, ry)`` directly would draw every arrow at right angles to the
   correction it represents — and the picture would still look plausible.
2. **A diff across two different grids.** Interpolating one would report a number
   for a node that was never captured, which is the opposite of what a
   re-capture invariance check is for.
3. **A per-hour slope fitted over nine minutes.** The uptime panel exists for an
   hours-scale phenomenon (+118–133 ms at ~16 h); extrapolating a short capture
   to "mm per hour" prints a large number that is entirely noise.
4. **Grid axes reconstructed from the CLI while a map is loaded.** Then
   ``[iy][ix]`` means two different poses in one report.

Everything else (the tables, the HTML) is checked for *existence and content*
rather than layout, on the principle that a report nobody can regenerate is worse
than one that is ugly.
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import pytest

_TESTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(_TESTS)
for _p in (os.path.join(_TESTS, 'hardware'), os.path.join(_REPO, 'tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import toss_fit_lib as fit                                       # noqa: E402
import toss_cal_analyse as ana                                   # noqa: E402

from jugglebot import toss_record                                # noqa: E402
from jugglebot.motion import toss_cal                            # noqa: E402


GRID_X = [-150.0, 0.0, 150.0]
GRID_Y = [-150.0, 0.0, 150.0]
Z_MM = 170.0

_CAPTURED = {'date': '2026-08-11', 'git_sha': 'deadbee', 'tool': fit.TOOL_NAME}
_REQUIRES = {'tilt_map_version': 'synthetic-tilt/1',
             'estimator_version': toss_cal.ESTIMATOR_VERSION}
_JACOBIAN = {'S': [[0.0, 1.0], [-1.0, 0.0]], 'gain_mm_per_rad': 3126.53}


def _corpus(**kwargs):
    params = dict(plant_bias_rad=lambda x, y: (4e-5 * x, 2e-5 * y),
                  n_per_node=10, sigma_mm=0.0, z_mm=Z_MM)
    params.update(kwargs)
    return fit.synthetic_corpus(GRID_X, GRID_Y, **params)


def _write(tmp_path, records, name='corpus.jsonl'):
    path = tmp_path / name
    with open(path, 'w') as handle:
        for rec in records:
            handle.write(toss_record.encode(rec) + '\n')
    return str(path)


def _map(tmp_path, records, name='map.yaml', x=GRID_X, y=GRID_Y):
    fits, _adm, _exc = fit.fit_nodes(records, x, y)
    anchor = fit.anchor_estimate(fit.anchor_visits(records, x, y))
    doc = fit.build_map_document(x, y, Z_MM, fits, anchor,
                                 captured=dict(_CAPTURED),
                                 requires=dict(_REQUIRES),
                                 jacobian=dict(_JACOBIAN))
    path = tmp_path / name
    path.write_text(fit.dump_map_yaml(doc))
    return str(path)


# ── the corpus panels ────────────────────────────────────────────────────────


def test_analyse_corpus_returns_every_panels_data():
    data = ana.analyse_corpus(_corpus(), GRID_X, GRID_Y, n_min=fit.N_MIN)
    assert data['n_records'] == 90 and data['n_admitted'] == 90
    assert data['rx'].shape == (3, 3) and data['n_grid'].sum() == 90
    assert data['anchors'] and data['anchor_n'] == 10
    assert data['sigma_L_mm'] is not None
    assert set(data['partitions'][0]) >= set(fit.PARTITION_KEYS)


def test_the_analyser_home_references_the_grid_it_prints():
    """The tool prints the SHIPPED quantity, not the raw reduction — otherwise
    the picture on screen is not the picture the machine will fly."""
    data = ana.analyse_corpus(_corpus(), GRID_X, GRID_Y, n_min=fit.N_MIN)
    hx, hy = fit.home_index(GRID_X, GRID_Y)
    assert data['rx'][hy][hx] == pytest.approx(0.0, abs=1e-9)
    assert data['ry'][hy][hx] == pytest.approx(0.0, abs=1e-9)


def test_a_thin_node_shows_its_n_rather_than_vanishing():
    records = [r for r in _corpus()
               if r['goal_catch_xyz_stow_mm'][:2] != [150.0, 150.0]]
    data = ana.analyse_corpus(records, GRID_X, GRID_Y, n_min=fit.N_MIN)
    assert data['n_grid'][2][2] == 0
    assert math.isfinite(float(data['rx'][2][2]))


def test_the_uptime_slope_is_withheld_on_a_span_too_short_to_see_it():
    """The panel exists for an hours-scale effect. A slope fitted over a
    9-minute capture and reported per hour is a ~7x extrapolation that prints a
    large number made entirely of noise."""
    data = ana.analyse_corpus(_corpus(), GRID_X, GRID_Y, n_min=fit.N_MIN)
    assert data['uptime_span_h'] < ana.MIN_UPTIME_SPAN_H
    assert data['uptime_trend_mm_per_h'] is None
    assert data['uptime_scatter'], 'the scatter is always kept; only the slope goes'


def test_the_uptime_slope_is_reported_once_the_span_is_long_enough():
    records = _corpus(dt_s=60.0)          # 90 tosses x 60 s = 1.5 h
    data = ana.analyse_corpus(records, GRID_X, GRID_Y, n_min=fit.N_MIN)
    assert data['uptime_span_h'] > ana.MIN_UPTIME_SPAN_H
    assert data['uptime_trend_mm_per_h'] is not None


# ── the map-vs-map diff ──────────────────────────────────────────────────────


def test_the_diff_refuses_two_different_grids_rather_than_interpolating():
    a = ana.map_field(fit.validate_map_document(_doc(_corpus())))
    other = fit.synthetic_corpus([-100.0, 0.0, 100.0], [-100.0, 0.0, 100.0],
                                 plant_bias_rad=(0.004, 0.0), n_per_node=10,
                                 sigma_mm=0.0)
    b = ana.map_field(fit.validate_map_document(
        _doc(other, x=[-100.0, 0.0, 100.0], y=[-100.0, 0.0, 100.0])))
    assert ana.diff_maps(a, b) == {'same_axes': False}


def test_the_diff_finds_the_worst_node_of_two_maps_on_one_grid():
    base = _doc(_corpus())
    moved = json.loads(json.dumps(base))
    moved['aim_rad']['rx'][0][2] += 0.002
    a = ana.map_field(fit.validate_map_document(base))
    b = ana.map_field(fit.validate_map_document(moved))
    diff = ana.diff_maps(b, a)
    assert diff['same_axes'] is True
    assert diff['max_delta_rad'] == pytest.approx(0.002, abs=1e-9)
    assert diff['worst_node'] == {'ix': 2, 'iy': 0, 'x_mm': 150.0,
                                  'y_mm': -150.0}


def _doc(records, x=GRID_X, y=GRID_Y):
    fits, _adm, _exc = fit.fit_nodes(records, x, y)
    anchor = fit.anchor_estimate(fit.anchor_visits(records, x, y))
    return fit.build_map_document(x, y, Z_MM, fits, anchor,
                                  captured=dict(_CAPTURED),
                                  requires=dict(_REQUIRES),
                                  jacobian=dict(_JACOBIAN))


# ── the CLI ──────────────────────────────────────────────────────────────────


def test_the_cli_writes_an_html_report_and_json_without_plots(tmp_path, capsys):
    corpus = _write(tmp_path, _corpus())
    report = tmp_path / 'report'
    payload = tmp_path / 'out.json'
    rc = ana.main(['--corpus', corpus, '--report-dir', str(report),
                   '--json', str(payload), '--no-plot'])
    assert rc == 0
    html = (report / 'index.html').read_text()
    assert 'HOME-REFERENCED' in html and 'C-TOSS-CAL-1' in html
    data = json.loads(payload.read_text())
    assert data['n_records'] == 90
    assert data['gain_mm_per_rad'] == pytest.approx(3126.53, rel=1e-3)
    assert 'PER-NODE aim residual' in capsys.readouterr().out


def test_the_cli_scores_groups_on_the_continuous_observables(tmp_path, capsys):
    tight = _corpus(sigma_mm=8.0, seed=1)
    loose = _corpus(sigma_mm=30.0, seed=2)
    corpus = _write(tmp_path, tight + loose)
    rc = ana.main(['--corpus', corpus, '--report-dir', str(tmp_path / 'r'),
                   '--no-plot', '--group', 'A=1-90', '--group', 'B=91-180'])
    out = capsys.readouterr().out
    assert rc == 0 and 'A/B' in out
    payload = tmp_path / 'g.json'
    ana.main(['--corpus', corpus, '--report-dir', str(tmp_path / 'r'),
              '--no-plot', '--json', str(payload),
              '--group', 'A=1-90', '--group', 'B=91-180'])
    groups = json.loads(payload.read_text())['groups']
    assert groups[0]['sigma_mm'] < groups[1]['sigma_mm']


def test_the_cli_reads_the_grid_from_the_map_it_was_given(tmp_path):
    """Axes come from the map when one is loaded. Reconstructing them from the
    CLI's ``--box-mm/--nodes`` while a map is open is how ``[iy][ix]`` comes to
    mean two different poses in a single report."""
    path = _map(tmp_path, fit.synthetic_corpus(
        [-100.0, 0.0, 100.0], [-100.0, 0.0, 100.0],
        plant_bias_rad=lambda x, y: (6e-5 * x, 0.0), n_per_node=10,
        sigma_mm=0.0, z_mm=Z_MM), x=[-100.0, 0.0, 100.0],
        y=[-100.0, 0.0, 100.0])
    payload = tmp_path / 'm.json'
    rc = ana.main(['--map', path, '--report-dir', str(tmp_path / 'r'),
                   '--no-plot', '--json', str(payload),
                   '--box-mm', '150', '--nodes', '5'])
    assert rc == 0
    assert json.loads(payload.read_text())['x_mm'] == [-100.0, 0.0, 100.0]


def test_the_cli_needs_something_to_analyse():
    assert ana.main(['--no-plot']) == 2


def test_the_plot_path_runs_end_to_end(tmp_path):
    """Plotting is the riskiest code here (pcolormesh edge arrays, a quiver in a
    rotated frame, an anchor series that may hold one point) and it is skipped
    everywhere else in this file, so it gets exactly one full run."""
    corpus = _write(tmp_path, _corpus(sigma_mm=10.0, dt_s=60.0))
    report = tmp_path / 'plots'
    assert ana.main(['--corpus', corpus, '--report-dir', str(report)]) == 0
    written = sorted(os.path.basename(p) for p in os.listdir(str(report)))
    for name in ('aim_magnitude.png', 'aim_quiver.png', 'aim_rx.png',
                 'aim_ry.png', 'anchor_series.png', 'index.html',
                 'node_n.png', 'uptime_scatter.png'):
        assert name in written, name


# ── the geometry the picture depends on ──────────────────────────────────────


def test_cell_edges_put_every_node_strictly_inside_its_own_cell():
    """``pcolormesh`` with node coordinates as edges would offset every colour
    cell by half a cell from its label — and reading an outlier off such a plot
    points the operator at the wrong node."""
    nodes = [-150.0, -20.0, 0.0, 90.0, 150.0]
    edges = ana.cell_edges(nodes)
    assert len(edges) == len(nodes) + 1
    assert all(b > a for a, b in zip(edges, edges[1:]))
    for i, node in enumerate(nodes):
        assert edges[i] < node < edges[i + 1]


def test_the_quiver_is_drawn_in_LANDING_space_not_tilt_space():
    """``J`` is a 90° rotation, so an arrow drawn as ``(rx, ry)`` sits at right
    angles to the landing shift it commands — and it would still look like a
    plausible field. The tool draws ``J·aim``; this pins the pairing."""
    T = math.sqrt(8.0 * 0.78 / 9.80665)
    J = fit.aim_landing_jacobian(T, Z_MM)
    gain = float(np.linalg.norm(J[:, 1]))
    aim = np.array([0.004, -0.002])
    want = J @ aim
    # the tool's own arithmetic, lifted verbatim from make_plots
    got = np.array([aim[1] * gain, -aim[0] * gain])
    assert got[0] == pytest.approx(float(want[0]), rel=1e-9)
    assert got[1] == pytest.approx(float(want[1]), rel=1e-9)
