"""Unit tests for the offline tilt-calibration analyser.

Tool: ``tools/tilt_cal_analyse.py``.
Contract: ``ros_ws/docs/levelling_frame.md`` § **C-LEVEL-2**.
Plan: ``plans/active/tilt-calibration-grid.md`` § Phase 3 (rung C2 consumes the
``--diff`` mode).

Lives in ``tests/sim/`` because that is where the existing test of a ``tools/``
script lives (``test_check_vel_ff_plumbing.py``), using the same ``sys.path``
idiom. The analyser is pure offline analysis — no ROS, no hardware, no motion.

The tests that matter most are the two around **outlier detection**, because
that is where this tool makes a judgement rather than a calculation:

* :func:`test_curvature_ignores_a_pure_gradient` and
  :func:`test_smooth_curved_field_produces_no_flags` pin the reason the
  statistic is a *second* difference. A first-order "departs from its
  neighbours" test flags the entire boundary of a good capture, because an edge
  node's neighbours all lie on its interior side and the residual field is
  genuinely curved (the 2026-07-28 table refutes only a *linear* fit).
* :func:`test_top_over_median_curvature_ratio_separates_a_pin` pins the
  scale-free discriminator the report leads with. The absolute flag line is
  PROVISIONAL until rung C1 measures a real field; this ratio needs no
  threshold at all.
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import pytest
import yaml

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
_TOOLS_DIR = os.path.join(_REPO_ROOT, 'tools')
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

import tilt_cal_analyse as tca  # noqa: E402


def _doc(x, y, tx, ty, sd=1e-5, z_mm=170.0, date='2026-08-03'):
    n_y, n_x = len(y), len(x)
    return {
        'version': 1,
        'captured': {'date': date, 'git_sha': 'test', 'tool': 'unit-test',
                     'base_condition': 'bench'},
        'grid': {'z_mm': z_mm, 'orientation': 'level',
                 'x_mm': list(x), 'y_mm': list(y)},
        'residual_rad': {
            'tx': [[float(tx[iy][ix]) for ix in range(n_x)]
                   for iy in range(n_y)],
            'ty': [[float(ty[iy][ix]) for ix in range(n_x)]
                   for iy in range(n_y)]},
        'stats': {
            'n_reads': 8,
            'sd_tx_rad': [[sd] * n_x for _ in range(n_y)],
            'sd_ty_rad': [[sd] * n_x for _ in range(n_y)]},
    }


def _write(tmp_path, name, doc):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(doc, default_flow_style=False,
                                   sort_keys=False))
    return str(path)


def _smooth_field(x, y, peak_deg=0.6):
    """A genuinely smooth residual field, shaped like the 2026-07-28 table:
    small near the middle, largest at a corner, curved but not kinked."""
    tx = np.zeros((len(y), len(x)))
    ty = np.zeros((len(y), len(x)))
    for iy, yv in enumerate(y):
        for ix, xv in enumerate(x):
            u, v = xv / 150.0, yv / 150.0
            tx[iy][ix] = math.radians(peak_deg) * (0.55 * u * u - 0.35 * v
                                                   + 0.10 * u * v)
            ty[iy][ix] = math.radians(peak_deg * 0.6) * (v + 0.25 * u * u)
    return tx, ty


AXIS5 = [-150.0, -75.0, 0.0, 75.0, 150.0]


# ── loading ──────────────────────────────────────────────────────────────


def test_load_map_field_uses_the_production_parser(tmp_path):
    tx, ty = _smooth_field(AXIS5, AXIS5)
    path = _write(tmp_path, 'm.yaml', _doc(AXIS5, AXIS5, tx, ty))
    field = tca.load_map_field(path)
    assert field.shape == (5, 5)
    assert list(field.x_mm) == AXIS5
    assert field.z_mm == pytest.approx(170.0)
    assert field.version.startswith('2026-08-03-')
    assert field.metadata['base_condition'] == 'bench'
    assert field.sd_tx is not None and field.sd_tx.shape == (5, 5)


def test_load_map_field_refuses_what_the_node_would_refuse(tmp_path):
    """The analyser rejects exactly what ``trajectory_node`` rejects, so
    "the analyser was happy" is a statement about the machine."""
    tx, ty = _smooth_field(AXIS5, AXIS5)
    doc = _doc(AXIS5, AXIS5, tx, ty)
    doc['residual_rad']['tx'][2][2] = 0.5     # way over MAX_ABS_RESIDUAL_RAD
    path = _write(tmp_path, 'bad.yaml', doc)
    from jugglebot.motion import tilt_map
    with pytest.raises(tilt_map.TiltMapError):
        tca.load_map_field(path)


def test_magnitude_is_the_vector_magnitude_in_degrees(tmp_path):
    tx = [[math.radians(0.3)]] * 2
    tx = np.array([[math.radians(0.3), 0.0], [0.0, 0.0]])
    ty = np.array([[math.radians(0.4), 0.0], [0.0, 0.0]])
    path = _write(tmp_path, 'm.yaml', _doc([0.0, 10.0], [0.0, 10.0], tx, ty))
    field = tca.load_map_field(path)
    # 0.3 and 0.4 degrees on the two axes -> 0.5 degrees of tilt magnitude,
    # which is what converts to landing displacement at 41.9 mm/deg.
    assert field.magnitude_deg[0][0] == pytest.approx(0.5, abs=1e-9)


# ── the kink statistic ───────────────────────────────────────────────────


def test_curvature_is_a_second_difference_and_skips_corners():
    grid = np.zeros((3, 3))
    grid[1][1] = 1.0
    curvature = tca.node_curvatures(grid)
    # Centre: 0 - 2(1) + 0 along both axes.
    assert curvature[(1, 1)] == pytest.approx(2.0)
    # Corners have no second difference along either axis — an honest limit,
    # not a silent pass.
    for corner in ((0, 0), (2, 0), (0, 2), (2, 2)):
        assert corner not in curvature
    # Edge midpoints are testable along the one axis they are interior to.
    assert curvature[(1, 0)] == pytest.approx(0.0)


def test_curvature_ignores_a_pure_gradient():
    """A ramp has zero second difference. This is the property that makes the
    boundary of a curved capture stop producing false positives."""
    grid = np.array([[float(ix) for ix in range(5)] for _ in range(5)])
    curvature = tca.node_curvatures(grid)
    assert max(curvature.values()) == pytest.approx(0.0)


def test_smooth_curved_field_produces_no_flags(tmp_path):
    tx, ty = _smooth_field(AXIS5, AXIS5)
    path = _write(tmp_path, 'm.yaml', _doc(AXIS5, AXIS5, tx, ty))
    field = tca.load_map_field(path)
    assert tca.flag_outliers(field) == []


def test_a_large_pin_is_flagged(tmp_path):
    """A leg pinned on its stroke clamp holds the platform off the commanded
    pose: the value can be ordinary, only the local discontinuity shows it."""
    tx, ty = _smooth_field(AXIS5, AXIS5)
    tx[1][3] += math.radians(0.5)
    path = _write(tmp_path, 'm.yaml', _doc(AXIS5, AXIS5, tx, ty))
    field = tca.load_map_field(path)
    flags = tca.flag_outliers(field)
    assert flags, 'a 0.5 deg pin must trip the flag line'
    assert any(f['x_mm'] == 75.0 and f['y_mm'] == -75.0 for f in flags)
    assert any('kink' in reason
               for flag in flags for reason in flag['reasons'])


def test_top_over_median_curvature_ratio_separates_a_pin(tmp_path):
    """The scale-free discriminator the report leads with.

    On a smooth field every node's second difference is nearly equal, so the
    ratio sits near 1. A single pinned node raises only its own (and mildly its
    neighbours'), so the ratio jumps — with no tuned threshold anywhere, which
    is why this is the number to read first while the flag line stays
    PROVISIONAL until rung C1.
    """
    def ratio(pin_deg):
        tx, ty = _smooth_field(AXIS5, AXIS5)
        if pin_deg:
            tx[1][3] += math.radians(pin_deg)
        path = _write(tmp_path, 'm{}.yaml'.format(pin_deg),
                      _doc(AXIS5, AXIS5, tx, ty))
        curvature = tca.curvature_by_node(tca.load_map_field(path))
        values = [v for v in curvature.values() if math.isfinite(v)]
        return max(values) / float(np.median(values))

    clean = ratio(0.0)
    assert clean < 1.2, 'a smooth field must sit near 1.0, got {}'.format(clean)
    assert ratio(0.2) > 2.0
    assert ratio(0.5) > 4.0
    # Monotone in the size of the fault.
    assert ratio(0.2) < ratio(0.3) < ratio(0.5)


def test_an_inflated_read_spread_is_flagged_independently(tmp_path):
    """Second detector, different physics: the platform was still moving when
    it was read, or the read path was unhealthy at that pose."""
    tx, ty = _smooth_field(AXIS5, AXIS5)
    doc = _doc(AXIS5, AXIS5, tx, ty, sd=1e-5)
    doc['stats']['sd_tx_rad'][2][1] = 1e-3          # 100x the median
    path = _write(tmp_path, 'm.yaml', doc)
    flags = tca.flag_outliers(tca.load_map_field(path))
    assert any(f['x_mm'] == -75.0 and f['y_mm'] == 0.0 for f in flags)
    assert any('sd_tx' in reason
               for flag in flags for reason in flag['reasons'])


def test_curvature_flag_line_takes_the_larger_of_floor_and_median_multiple():
    quiet = {(0, 0): 0.01, (1, 0): 0.02, (2, 0): 0.03}      # 4 x 0.02 < floor
    limit, basis = tca.curvature_limit(quiet)
    assert limit == pytest.approx(tca.CURVATURE_FLOOR_DEG)
    assert 'PROVISIONAL' in basis, (
        'the basis string must keep saying so until rung C1 measures a real '
        'field — this threshold is not yet knowable')

    curved = {(0, 0): 0.05, (1, 0): 0.06, (2, 0): 0.07}     # 4 x 0.06 > floor
    limit, _ = tca.curvature_limit(curved)
    assert limit == pytest.approx(tca.CURVATURE_FLAG_MULT * 0.06)


def test_curvature_flag_line_can_be_overridden():
    curvature = {(0, 0): 0.05, (1, 0): 0.06, (2, 0): 0.07}
    limit, basis = tca.curvature_limit(curvature, override=0.01)
    assert limit == pytest.approx(0.01)
    assert 'override' in basis


# ── diff mode (rung C2) ──────────────────────────────────────────────────


def test_diff_of_a_map_with_itself_is_zero(tmp_path):
    tx, ty = _smooth_field(AXIS5, AXIS5)
    path = _write(tmp_path, 'm.yaml', _doc(AXIS5, AXIS5, tx, ty))
    field = tca.load_map_field(path)
    diff = tca.diff_fields(field, tca.load_map_field(path))
    assert diff['same_axes'] is True
    assert diff['max_delta_deg'] == pytest.approx(0.0, abs=1e-12)


def test_diff_reports_a_known_offset_and_names_the_worst_node(tmp_path):
    tx, ty = _smooth_field(AXIS5, AXIS5)
    a = _write(tmp_path, 'a.yaml', _doc(AXIS5, AXIS5, tx, ty))
    shifted = tx.copy()
    shifted[3][4] += math.radians(0.2)
    b = _write(tmp_path, 'b.yaml', _doc(AXIS5, AXIS5, shifted, ty))
    diff = tca.diff_fields(tca.load_map_field(b), tca.load_map_field(a))
    assert diff['max_delta_deg'] == pytest.approx(0.2, abs=1e-6)
    assert diff['worst_node']['x_mm'] == 150.0
    assert diff['worst_node']['y_mm'] == 75.0


def test_diff_across_different_grids_interpolates_and_says_so(tmp_path):
    """A linear field interpolates exactly, so a coarse-vs-fine diff of the
    same physics is ~0 — and the report must still declare the mode, because an
    interpolated diff carries the other map's interpolation error."""
    coarse = [-150.0, 0.0, 150.0]
    def linear(x, y):
        tx = np.array([[math.radians(0.004) * xv for xv in x] for _ in y])
        ty = np.array([[math.radians(0.002) * yv for _ in x] for yv in y])
        return tx, ty

    ta, tb = linear(coarse, coarse)
    a = _write(tmp_path, 'a.yaml', _doc(coarse, coarse, ta, tb))
    tc, td = linear(AXIS5, AXIS5)
    b = _write(tmp_path, 'b.yaml', _doc(AXIS5, AXIS5, tc, td))
    diff = tca.diff_fields(tca.load_map_field(b), tca.load_map_field(a))
    assert diff['same_axes'] is False
    assert diff['max_delta_deg'] == pytest.approx(0.0, abs=1e-9)


def test_c2_threshold_is_max_of_twice_noise_and_the_floor(tmp_path):
    tx, ty = _smooth_field(AXIS5, AXIS5)
    quiet = tca.load_map_field(
        _write(tmp_path, 'q.yaml', _doc(AXIS5, AXIS5, tx, ty, sd=1e-6)))
    noisy = tca.load_map_field(
        _write(tmp_path, 'n.yaml', _doc(AXIS5, AXIS5, tx, ty, sd=0.01)))

    limit, basis = tca.c2_threshold_deg(quiet, quiet)
    assert limit == pytest.approx(tca.DIFF_FLOOR_DEG), basis

    limit, basis = tca.c2_threshold_deg(noisy, quiet)
    # 0.01 rad = 0.573 deg; twice that dominates the 0.05 deg floor.
    assert limit == pytest.approx(2.0 * math.degrees(0.01), rel=1e-6)
    assert 'median per-read sd' in basis

    limit, basis = tca.c2_threshold_deg(quiet, quiet, override=0.02)
    assert limit == pytest.approx(0.02)
    assert 'override' in basis


# ── CSV mode ─────────────────────────────────────────────────────────────


def _write_capture_csv(tmp_path, name='cap.csv'):
    """Build a capture CSV using the acquisition tool's own row writer, so this
    test breaks if the two ends of the schema ever drift apart."""
    import csv as _csv
    hw_dir = os.path.join(_REPO_ROOT, 'tests', 'hardware')
    if hw_dir not in sys.path:
        sys.path.insert(0, hw_dir)
    import tilt_cal_grid as tcg

    offset = tcg.mounting_offset_rad()
    path = tmp_path / name
    x = [-150.0, 0.0, 150.0]
    y = [-150.0, 0.0, 150.0]
    with open(str(path), 'w', newline='') as handle:
        writer = _csv.DictWriter(handle, fieldnames=list(tcg.CSV_COLUMNS))
        writer.writeheader()
        for iy, yv in enumerate(y):
            for ix, xv in enumerate(x):
                for k in range(4):
                    raw_tx = 0.001 * ix - offset[0]
                    raw_ty = 0.002 * iy - offset[1]
                    writer.writerow(tcg.csv_row(
                        'iso', 0.0, 'capture', 0, ix, iy, xv, yv, 170.0, k,
                        raw_tx, raw_ty, offset, 1234))
        # one verification pose, and one failed read that must be ignored
        writer.writerow(tcg.csv_row('iso', 0.0, 'verify', 0, -1, -1,
                                    75.0, 75.0, 170.0, 0,
                                    -offset[0], -offset[1], offset, 1234))
        writer.writerow(tcg.csv_row('iso', 0.0, 'capture', 0, 0, 0,
                                    -150.0, -150.0, 170.0, 9,
                                    float('nan'), float('nan'), offset, 1234))
    return str(path)


def test_csv_mode_reduces_reads_to_a_field(tmp_path):
    field = tca.load_csv_field(_write_capture_csv(tmp_path))
    assert field.shape == (3, 3)
    assert list(field.x_mm) == [-150.0, 0.0, 150.0]
    # residual = raw + offset, and the raws were built as (value - offset)
    assert field.tx[0][2] == pytest.approx(0.002)
    assert field.ty[2][0] == pytest.approx(0.004)
    assert field.version == '(from CSV, unwritten)'


def test_csv_mode_ignores_failed_reads_and_separates_verify_rows(tmp_path):
    field = tca.load_csv_field(_write_capture_csv(tmp_path))
    # 4 good reads at (0,0); the NaN row must not be counted or averaged in.
    assert int(field.n_reads[0][0]) == 4
    assert math.isfinite(field.tx[0][0])
    checks = field.metadata['check_poses']
    assert len(checks) == 1
    assert checks[0]['x_mm'] == 75.0
    assert math.hypot(checks[0]['res_tx_rad'],
                      checks[0]['res_ty_rad']) == pytest.approx(0.0, abs=1e-12)


def test_csv_mode_refuses_a_file_with_no_usable_rows(tmp_path):
    path = tmp_path / 'empty.csv'
    path.write_text('not,a,tilt,csv\n1,2,3,4\n')
    with pytest.raises(SystemExit, match='no usable capture rows'):
        tca.load_csv_field(str(path))


def test_csv_mode_excludes_home_end_and_verify_home_from_the_field(tmp_path):
    """The 2026-08-10 tool writes 'home_end' (drift-gate re-measure) and
    'verify_home' (verification reference) rows at the home coordinates. If
    they averaged into the field, the home node would silently mix its start
    and end measurements — and mix pre-/post-apply reads — exactly the
    corruption the phase filter's own comment describes. verify_home must
    instead surface as metadata (the tool scores checks against it)."""
    import csv as _csv
    hw_dir = os.path.join(_REPO_ROOT, 'tests', 'hardware')
    if hw_dir not in sys.path:
        sys.path.insert(0, hw_dir)
    import tilt_cal_grid as tcg

    offset = tcg.mounting_offset_rad()
    path = tmp_path / 'phases.csv'
    with open(str(path), 'w', newline='') as handle:
        writer = _csv.DictWriter(handle, fieldnames=list(tcg.CSV_COLUMNS))
        writer.writeheader()
        # a 1x2 grid: home (0,0) plus one neighbour, 2 reads each
        for k in range(2):
            writer.writerow(tcg.csv_row(
                'iso', 0.0, 'capture', 0, 0, 0, 0.0, 0.0, 170.0, k,
                0.001 - offset[0], 0.001 - offset[1], offset, 1))
            writer.writerow(tcg.csv_row(
                'iso', 0.0, 'capture', 1, 1, 0, 150.0, 0.0, 170.0, k,
                0.002 - offset[0], 0.002 - offset[1], offset, 1))
        # grossly different home_end and verify_home rows at (0, 0)
        writer.writerow(tcg.csv_row(
            'iso', 9.0, 'home_end', 2, 0, 0, 0.0, 0.0, 170.0, 0,
            0.5 - offset[0], 0.5 - offset[1], offset, 1))
        writer.writerow(tcg.csv_row(
            'iso', 9.5, 'verify_home', -1, -1, -1, 0.0, 0.0, 170.0, 0,
            0.25 - offset[0], 0.25 - offset[1], offset, 1))

    field = tca.load_csv_field(str(path))
    # capture-only mean at home: the 0.5/0.25 rad rows must not participate
    assert field.tx[0][0] == pytest.approx(0.001, abs=1e-12)
    assert int(field.n_reads[0][0]) == 2
    # ...and verify_home surfaces as the scoring reference, not as field data
    assert field.metadata['verify_home_rad'] == pytest.approx([0.25, 0.25])


# ── CLI ──────────────────────────────────────────────────────────────────


def test_main_writes_json_and_exits_zero(tmp_path, capsys):
    tx, ty = _smooth_field(AXIS5, AXIS5)
    path = _write(tmp_path, 'm.yaml', _doc(AXIS5, AXIS5, tx, ty))
    out = str(tmp_path / 'out' / 'summary.json')
    assert tca.main([path, '--no-plot', '--json', out]) == 0
    payload = json.loads(open(out).read())
    assert payload['x_mm'] == AXIS5
    assert payload['diff'] is None
    assert payload['outliers'] == []
    assert payload['median_sd_deg'] == pytest.approx(math.degrees(1e-5))
    report = capsys.readouterr().out
    assert 'top/median curvature ratio' in report
    assert 'residual magnitude [deg]' in report


def test_main_diff_exit_code_reflects_the_c2_verdict(tmp_path, capsys):
    tx, ty = _smooth_field(AXIS5, AXIS5)
    a = _write(tmp_path, 'a.yaml', _doc(AXIS5, AXIS5, tx, ty))
    shifted = tx.copy()
    shifted[3][4] += math.radians(0.5)          # far beyond the C2 bound
    b = _write(tmp_path, 'b.yaml', _doc(AXIS5, AXIS5, shifted, ty))

    assert tca.main([a, '--diff', a, '--no-plot']) == 0
    assert 'VERDICT: PASS' in capsys.readouterr().out

    assert tca.main([b, '--diff', a, '--no-plot']) == 1
    assert 'VERDICT: FAIL' in capsys.readouterr().out


def test_main_reports_a_missing_file_rather_than_raising(tmp_path, capsys):
    assert tca.main([str(tmp_path / 'nope.yaml'), '--no-plot']) == 1
    assert 'no such file' in capsys.readouterr().err


# ── heat-map cell geometry ───────────────────────────────────────
#
# The plots are how an operator FINDS an outlier node. A colour cell that is not
# where its label says it is sends them to re-measure the wrong node, so the
# cell/label co-location is a correctness property of this tool, not styling.


def test_cell_edges_bracket_every_node():
    """n nodes ⇒ n+1 strictly-increasing edges, each node inside its own cell."""
    for nodes in (AXIS5, [-150.0, -20.0, 0.0, 5.0, 150.0], [0.0, 1.0]):
        edges = tca.cell_edges(nodes)
        assert len(edges) == len(nodes) + 1
        assert all(edges[i] < edges[i + 1] for i in range(len(edges) - 1)), \
            'a non-monotonic edge array silently reorders the colour cells'
        for i, node in enumerate(nodes):
            assert edges[i] < float(node) < edges[i + 1], \
                'node {} is not inside cell {}'.format(node, i)


def test_cell_edges_centre_each_node_on_a_uniform_axis():
    """The default grid is uniform, so there each node IS the cell centre.

    This is the case the old `imshow(extent=[first, last])` got wrong by half a
    cell: it spread 5 pixels across [-150, 150], centring the first at -120
    while its label sat at -150.
    """
    edges = tca.cell_edges(AXIS5)
    for i, node in enumerate(AXIS5):
        assert 0.5 * (edges[i] + edges[i + 1]) == pytest.approx(float(node))


def test_cell_edges_handle_a_non_uniform_axis_that_imshow_cannot():
    """On unequal spacing an even pixel spread puts a node in the WRONG cell.

    `--x-nodes` accepts an arbitrary list, so this is a supported grid, and it
    is the case where the old rendering stopped being merely offset and started
    being wrong: the node at 5.0 fell inside the pixel belonging to 0.0.
    """
    nodes = [-150.0, -20.0, 0.0, 5.0, 150.0]
    edges = tca.cell_edges(nodes)
    # The fix: every node lands in its own cell.
    for i, node in enumerate(nodes):
        assert edges[i] < node < edges[i + 1]

    # The bug, reproduced: imshow's even spread across [first, last].
    lo, hi = nodes[0], nodes[-1]
    width = (hi - lo) / len(nodes)
    def imshow_cell(value):
        return min(int((value - lo) / width), len(nodes) - 1)
    assert imshow_cell(5.0) == imshow_cell(0.0), \
        'precondition: the even spread collapses two nodes into one pixel'
    assert imshow_cell(5.0) != nodes.index(5.0)


def test_make_plots_places_each_label_inside_its_own_colour_cell(tmp_path):
    """End-to-end on the real plotting path: labels sit in the cells they name.

    Drives `make_plots` under the Agg backend and reads the QuadMesh's own
    coordinates back out, so it tests what is rendered rather than restating
    `cell_edges`. A non-uniform y-axis is used because that is where the two
    disagreed most.
    """
    matplotlib = pytest.importorskip('matplotlib')
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    x = [-150.0, -20.0, 0.0, 5.0, 150.0]
    y = [-150.0, -75.0, 0.0, 75.0, 150.0]
    tx, ty = _smooth_field(x, y)
    field = tca.Field(x_mm=x, y_mm=y, tx=np.asarray(tx), ty=np.asarray(ty))

    written = tca.make_plots(field, str(tmp_path))
    assert written, 'matplotlib is installed, so plots must have been written'
    assert all(os.path.exists(p) for p in written)

    # Rebuild one mesh the way make_plots does and check the rendered geometry.
    fig, ax = plt.subplots()
    mesh = ax.pcolormesh(np.asarray(tca.cell_edges(x)),
                         np.asarray(tca.cell_edges(y)),
                         field.magnitude_deg, shading='flat')
    # QuadMesh cell VERTICES, shape (ny+1, nx+1, 2) — NOT centres. The four
    # corners of cell (iy, ix) are [iy][ix], [iy][ix+1], [iy+1][ix],
    # [iy+1][ix+1], which is what the bracketing assertions below read.
    centres = mesh.get_coordinates()
    for iy in range(len(y)):
        for ix in range(len(x)):
            corners = [centres[iy][ix], centres[iy][ix + 1],
                       centres[iy + 1][ix], centres[iy + 1][ix + 1]]
            xs = [float(c[0]) for c in corners]
            ys = [float(c[1]) for c in corners]
            assert min(xs) < x[ix] < max(xs), \
                'label x={} is outside the cell drawn for it'.format(x[ix])
            assert min(ys) < y[iy] < max(ys), \
                'label y={} is outside the cell drawn for it'.format(y[iy])
    plt.close(fig)
