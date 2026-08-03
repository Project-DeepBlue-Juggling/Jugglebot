"""Unit tests for the pure core of the tilt-calibration acquisition tool.

Tool: ``tests/hardware/tilt_cal_grid.py`` (operator-run rclpy CLI).
Contract: ``ros_ws/docs/levelling_frame.md`` § **C-LEVEL-2**.
Plan: ``plans/active/tilt-calibration-grid.md`` § Phase 3.

The tool splits into a **pure core** — grid-spec generation, the residual
reduction, map-document assembly, CSV rows — and a thin rclpy runtime. Only the
core is tested here, and it is importable with no ROS at all: the runtime
imports ``rclpy`` and the interface packages *inside* :func:`tilt_cal_grid.run`
precisely so this file can exist.

This lives in ``tests/motion/`` rather than ``tests/ros/`` or ``tests/sim/``
because that is where every existing importer of a ``tests/hardware/`` script
already lives (``test_bench_sysid_logic.py``, ``test_bench_sysid_bridge.py``,
``test_kt_lib.py``), using the same ``_HW_DIR`` ``sys.path`` idiom.

Two tests carry more weight than the rest:

* :func:`test_residual_matches_the_real_levelling_handler` drives the **actual**
  ``state_machine.LevellingHandler`` phase and compares its output to the tool's
  reduction on the same reading. The residual is *defined* as that formula, so a
  restatement of it in an assertion would drift in lockstep with a bug. This
  fails if either side changes its sign, its constant, or its axis order.
* :func:`test_map_document_round_trips_through_the_production_loader` pushes the
  writer's own output through ``tilt_map.parse_tilt_map`` — the same function
  ``trajectory_node`` loads with. "The tool writes a valid map" is otherwise a
  claim about a schema doc rather than about the machine.

The grid fixtures are deliberately **asymmetric** (3 x-nodes vs 4 y-nodes,
unequal spacing, no two residuals equal) for the reason ``test_tilt_map.py``
gives: a transposed ``[iy][ix]`` index is invisible on a square symmetric grid.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest
import yaml

_HW_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import tilt_cal_grid as tcg  # noqa: E402

import jugglebot.hardware_config as hw  # noqa: E402
from jugglebot.motion import tilt_map  # noqa: E402


OFFSET = tcg.mounting_offset_rad()


# ── grid spec ────────────────────────────────────────────────────────────


def test_build_axis_box_form_spans_the_box_inclusively():
    assert tcg.build_axis(150.0, 5, 'x') == [-150.0, -75.0, 0.0, 75.0, 150.0]
    assert tcg.build_axis(100.0, 3, 'y') == [-100.0, 0.0, 100.0]


def test_build_axis_rejects_degenerate_specs():
    with pytest.raises(tcg.TiltCalError, match='at least 2'):
        tcg.build_axis(150.0, 1, 'x')
    with pytest.raises(tcg.TiltCalError, match='positive half-width'):
        tcg.build_axis(0.0, 5, 'x')
    with pytest.raises(tcg.TiltCalError, match='positive half-width'):
        tcg.build_axis(-10.0, 5, 'x')


def test_parse_node_list_accepts_commas_and_spaces():
    assert tcg.parse_node_list('-150,-75,0,75,150', 'x') == \
        [-150.0, -75.0, 0.0, 75.0, 150.0]
    assert tcg.parse_node_list('-100 0 100', 'y') == [-100.0, 0.0, 100.0]


def test_parse_node_list_rejects_non_monotonic_and_short_axes():
    with pytest.raises(tcg.TiltCalError, match='strictly increasing'):
        tcg.parse_node_list('0,100,50', 'x')
    with pytest.raises(tcg.TiltCalError, match='strictly increasing'):
        tcg.parse_node_list('0,0,100', 'x')       # a repeat is a 0-width cell
    with pytest.raises(tcg.TiltCalError, match='at least 2'):
        tcg.parse_node_list('0', 'x')
    with pytest.raises(tcg.TiltCalError, match='not a list of numbers'):
        tcg.parse_node_list('0,banana', 'x')


def test_home_index_locates_the_origin():
    assert tcg.home_index([-150.0, 0.0, 150.0], [-75.0, 0.0, 75.0]) == (1, 1)


def test_home_index_refuses_a_grid_without_the_origin():
    """An even --nodes over a symmetric box has no (0, 0) node.

    The home node is the stale-level-reference gate — the one failure that
    corrupts every node identically while leaving the map looking plausible —
    so a grid that cannot check it is refused rather than captured.
    """
    axis = tcg.build_axis(150.0, 4, 'x')
    assert 0.0 not in axis
    with pytest.raises(tcg.TiltCalError, match='home node'):
        tcg.home_index(axis, axis)


def test_serpentine_alternates_direction_each_row():
    x = [-1.0, 0.0, 1.0]
    y = [-1.0, 0.0, 1.0]
    order = tcg.serpentine_order(x, y)
    row0 = [n.x_mm for n in order if n.iy == 0]
    row1 = [n.x_mm for n in order if n.iy == 1]
    row2 = [n.x_mm for n in order if n.iy == 2]
    assert row0 == [-1.0, 0.0, 1.0]
    assert row1 == [1.0, 0.0, -1.0]      # reversed — this is the whole point
    assert row2 == [-1.0, 0.0, 1.0]
    # Rows are visited in increasing y, so the sweep never jumps rows twice.
    assert [n.iy for n in order] == sorted(n.iy for n in order)


def test_serpentine_visits_every_node_exactly_once():
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    order = tcg.serpentine_order(x, y)
    assert len(order) == 25
    assert len({(n.ix, n.iy) for n in order}) == 25


def test_visit_order_measures_the_home_node_first_and_only_once():
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    order = tcg.visit_order(x, y)
    assert (order[0].x_mm, order[0].y_mm) == (0.0, 0.0)
    assert len(order) == 25
    assert len({(n.ix, n.iy) for n in order}) == 25
    # Exactly one visit to the origin: two readings for one node would force an
    # arbitrary choice between them.
    assert sum(1 for n in order if n.x_mm == 0.0 and n.y_mm == 0.0) == 1


def test_visit_order_indices_address_their_own_coordinates():
    """``ix``/``iy`` must index ``x_mm``/``y_mm`` — swapping them is silent on a
    square grid, so this uses different axis lengths."""
    x = [-10.0, 0.0, 10.0]
    y = [-20.0, -5.0, 0.0, 30.0]
    for node in tcg.visit_order(x, y):
        assert x[node.ix] == node.x_mm
        assert y[node.iy] == node.y_mm


# ── check poses ──────────────────────────────────────────────────────────


def test_check_poses_are_off_node_and_inside_the_hull():
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    poses = tcg.check_poses(x, y, 6)
    assert len(poses) == 6
    for cx, cy in poses:
        assert x[0] < cx < x[-1]
        assert y[0] < cy < y[-1]
        assert cx not in x, 'a check pose on a node measures the stored value'
        assert cy not in y


def test_check_poses_are_deterministic():
    """Rung C2 re-verifies the *same* poses after a re-level without
    recapturing, so this ordering is load-bearing, not cosmetic."""
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    assert tcg.check_poses(x, y, 6) == tcg.check_poses(x, y, 6)


def test_check_poses_cover_all_four_quadrants_before_doubling_up():
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    first_four = tcg.check_poses(x, y, 4)
    quadrants = {((cx >= 0), (cy >= 0)) for cx, cy in first_four}
    assert len(quadrants) == 4


def test_check_poses_prefer_the_outermost_cells():
    """The 2026-07-28 table puts the largest residual at the corner, so that is
    where an interpolation error costs a ball."""
    x = tcg.build_axis(150.0, 5, 'x')
    y = tcg.build_axis(150.0, 5, 'y')
    first_four = tcg.check_poses(x, y, 4)
    for cx, cy in first_four:
        assert math.hypot(cx, cy) == pytest.approx(math.hypot(112.5, 112.5))


def test_check_poses_handles_zero_and_oversized_counts():
    x = tcg.build_axis(150.0, 3, 'x')
    y = tcg.build_axis(150.0, 3, 'y')
    assert tcg.check_poses(x, y, 0) == []
    # 2x2 = 4 cells; asking for more returns all of them, not a crash.
    assert len(tcg.check_poses(x, y, 99)) == 4


# ── residual math ────────────────────────────────────────────────────────


def test_mounting_offset_comes_from_the_generated_constant():
    expected = tuple(math.radians(d) for d in hw.JB_OP_INCLINOMETER_OFFSET_DEG)
    assert tcg.mounting_offset_rad() == expected


class _LevellingCtx:
    """Minimal stand-in for the orchestrator context LevellingHandler mutates."""

    def __init__(self, tilt_reading):
        self.tilt_reading = list(tilt_reading)
        self.pose_offset_rad = None
        self.control_mode = ''
        self.request = None
        self.operation_result = True

    def clear_commands(self):
        pass


def test_residual_matches_the_real_levelling_handler():
    """Drive the production handler and compare, rather than restating it.

    C-LEVEL-2 defines a residual as *the exact LevellingHandler formula*
    (``state_machine.py:455-460``). Asserting against a hand-written
    ``raw + radians(offset)`` here would pass just as happily if both this tool
    and the handler flipped the sign together, and a sign flip inverts every
    node in the map — aiming the machine roughly twice as badly as no map at all.
    """
    from jugglebot.state_machine import LevellingHandler

    handler = LevellingHandler(
        inclinometer_offset_deg=hw.JB_OP_INCLINOMETER_OFFSET_DEG, settle_s=0.0)
    handler._phase_idx = LevellingHandler._PHASES.index('level_get_tilt')

    raw = (0.01234, -0.04567)
    ctx = _LevellingCtx(raw)
    handler.execute(ctx)
    assert ctx.pose_offset_rad is not None, 'handler phase did not run'

    stats = tcg.residual_from_reads([raw], tcg.mounting_offset_rad())
    assert stats.res_tx == pytest.approx(ctx.pose_offset_rad[0], abs=1e-15)
    assert stats.res_ty == pytest.approx(ctx.pose_offset_rad[1], abs=1e-15)


def test_residual_adds_the_offset_it_does_not_subtract():
    """Belt and braces on the sign, with the real (negative) constants."""
    stats = tcg.residual_from_reads([(0.0, 0.0)], OFFSET)
    assert stats.res_tx == pytest.approx(OFFSET[0])
    assert stats.res_ty == pytest.approx(OFFSET[1])
    assert OFFSET[0] < 0.0 and OFFSET[1] < 0.0, (
        'the shipped offsets are negative; if that changes, confirm this test '
        'still distinguishes an add from a subtract')


def test_residual_averages_and_reports_sample_sd():
    stats = tcg.residual_from_reads([(1.0, 5.0), (3.0, 5.0)], (0.0, 0.0))
    assert stats.raw_mean_tx == pytest.approx(2.0)
    # Sample sd (ddof=1) = sqrt(2); the population sd would be 1.0. ddof=0
    # understates the spread at the small N this tool uses, and the spread
    # gates the home node.
    assert stats.sd_tx == pytest.approx(math.sqrt(2.0))
    assert stats.sd_ty == pytest.approx(0.0)
    assert (stats.n_ok, stats.n_total) == (2, 2)


def test_residual_drops_nan_reads_but_counts_them():
    nan = float('nan')
    stats = tcg.residual_from_reads(
        [(1.0, 1.0), (nan, nan), (3.0, 3.0), (nan, 2.0)], (0.0, 0.0))
    assert (stats.n_ok, stats.n_total) == (2, 4)
    assert stats.raw_mean_tx == pytest.approx(2.0)


def test_residual_with_every_read_failed_is_all_nan():
    nan = float('nan')
    stats = tcg.residual_from_reads([(nan, nan)] * 4, OFFSET)
    assert stats.n_ok == 0
    assert stats.n_total == 4
    assert math.isnan(stats.res_tx) and math.isnan(stats.res_ty)
    assert not tcg.node_is_good(stats, 4)


def test_residual_accepts_a_generator():
    """Regression: a generator was consumed by the finite-filter, leaving
    ``n_total = 0`` — which made ``node_is_good`` pass a node with no reads."""
    stats = tcg.residual_from_reads(((1.0, 1.0) for _ in range(4)), (0.0, 0.0))
    assert (stats.n_ok, stats.n_total) == (4, 4)


def test_node_is_good_needs_an_sd_and_half_the_requested_reads():
    def _stats(n_ok):
        return tcg.ReadStats(n_ok=n_ok, n_total=8, raw_mean_tx=0.0,
                             raw_mean_ty=0.0, res_tx=0.0, res_ty=0.0,
                             sd_tx=0.0, sd_ty=0.0)
    assert not tcg.node_is_good(_stats(1), 8)    # no sd from one sample
    assert not tcg.node_is_good(_stats(3), 8)    # under half
    assert tcg.node_is_good(_stats(4), 8)
    assert tcg.node_is_good(_stats(8), 8)


# ── home-node gate ───────────────────────────────────────────────────────


def _home_stats(res_tx, res_ty, sd):
    return tcg.ReadStats(n_ok=8, n_total=8, raw_mean_tx=0.0, raw_mean_ty=0.0,
                         res_tx=res_tx, res_ty=res_ty, sd_tx=sd, sd_ty=sd)


def test_home_gate_passes_a_fresh_reference():
    ok, message = tcg.home_node_verdict(_home_stats(1e-5, -2e-5, 2e-5))
    assert ok, message


def test_home_gate_fails_a_stale_reference_and_says_so():
    ok, message = tcg.home_node_verdict(_home_stats(0.02, 0.0, 2e-5))
    assert not ok
    assert 'STALE LEVEL REFERENCE' in message
    assert 'level' in message


def test_home_gate_floor_does_not_false_abort_on_a_quiet_sensor():
    """3 x sd alone is degenerate when the sensor is quiet: the Teensy
    persistence quantum is 1 mrad/axis on its own, so a tolerance of 3e-9 rad
    would abort a perfectly good capture — and a gate that fires on good data
    is a gate the third operator disables."""
    ok, _ = tcg.home_node_verdict(_home_stats(1e-3, 0.0, 1e-9))
    assert ok, 'residual inside the absolute floor must pass'


def test_home_gate_ceiling_beats_a_wide_sd():
    """The mirror failure: a *noisy* sensor makes 3 x sd wide enough to launder
    a genuinely stale reference. The absolute ceiling refuses it anyway."""
    stats = _home_stats(0.02, 0.0, 0.01)     # 3 x sd = 0.03 > 0.02 residual
    assert 3.0 * stats.sd_tx > abs(stats.res_tx), 'fixture must exercise this'
    ok, message = tcg.home_node_verdict(stats)
    assert not ok
    assert 'ceiling' in message


def test_home_gate_fails_when_the_sensor_barely_answered():
    stats = tcg.ReadStats(n_ok=1, n_total=8, raw_mean_tx=0.0, raw_mean_ty=0.0,
                          res_tx=0.0, res_ty=0.0, sd_tx=0.0, sd_ty=0.0)
    ok, message = tcg.home_node_verdict(stats)
    assert not ok
    assert 'finite reads' in message


# ── map document ─────────────────────────────────────────────────────────


def _asymmetric_capture():
    """3 x-nodes, 4 y-nodes, unequal spacing, no two residuals equal."""
    x = [-100.0, -25.0, 150.0]
    y = [-150.0, -10.0, 30.0, 120.0]
    results = {}
    for ix in range(len(x)):
        for iy in range(len(y)):
            results[(ix, iy)] = tcg.ReadStats(
                n_ok=8, n_total=8, raw_mean_tx=0.0, raw_mean_ty=0.0,
                res_tx=0.001 * (ix + 1) + 0.0001 * iy,
                res_ty=-0.002 * (iy + 1) - 0.0003 * ix,
                sd_tx=1e-5, sd_ty=2e-5)
    return x, y, results


_CAPTURED = {
    'date': '2026-08-03',
    'git_sha': 'abc123',
    'tool': tcg.TOOL_NAME,
    'args': [],
    'uptime_ms_first': 1000,
    'uptime_ms_last': 240000,
    'level_offset_rad': [0.001, -0.002],
    'base_condition': 'unit test',
}


def test_map_document_round_trips_through_the_production_loader():
    """The writer's output must be acceptable to the loader the node uses.

    ``parse_tilt_map`` is the exact function ``trajectory_node._load_tilt_map``
    calls, so this is a statement about the machine and not about a schema doc.
    """
    x, y, results = _asymmetric_capture()
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)
    parsed = tilt_map.parse_tilt_map(doc)

    assert parsed.shape == (len(y), len(x))
    assert list(parsed.x_mm) == x
    assert list(parsed.y_mm) == y
    assert parsed.z_mm == pytest.approx(170.0)
    # The version the node will publish on TrajectoryStatus.tilt_map_version is
    # the one the tool computes locally to verify the auto-apply readback.
    assert parsed.version == tilt_map.map_version(doc)

    # Every node is recoverable at its own coordinates (lookup is exact at nodes).
    for ix in range(len(x)):
        for iy in range(len(y)):
            tx, ty = tilt_map.lookup(parsed, x[ix], y[iy])
            assert tx == pytest.approx(results[(ix, iy)].res_tx, abs=1e-9)
            assert ty == pytest.approx(results[(ix, iy)].res_ty, abs=1e-9)


def test_map_document_is_indexed_iy_ix():
    """A transposed grid is invisible on a square capture and fatal on a real
    one, so the fixture is 3 x 4."""
    x, y, results = _asymmetric_capture()
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)
    tx = doc['residual_rad']['tx']
    assert len(tx) == len(y)
    assert all(len(row) == len(x) for row in tx)
    for ix in range(len(x)):
        for iy in range(len(y)):
            assert tx[iy][ix] == pytest.approx(results[(ix, iy)].res_tx,
                                               abs=1e-9)


def test_map_document_refuses_an_unusable_node():
    x, y, results = _asymmetric_capture()
    results[(1, 2)] = tcg.ReadStats(n_ok=0, n_total=8, raw_mean_tx=float('nan'),
                                    raw_mean_ty=float('nan'),
                                    res_tx=float('nan'), res_ty=float('nan'),
                                    sd_tx=float('nan'), sd_ty=float('nan'))
    with pytest.raises(tcg.TiltCalError) as excinfo:
        tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)
    message = str(excinfo.value)
    assert 'unusable node' in message
    # The refusal must name the node, in operator coordinates.
    assert '-25.0' in message and '30.0' in message
    assert 'never interpolated' in message


def test_map_document_refuses_a_missing_node():
    x, y, results = _asymmetric_capture()
    del results[(0, 0)]
    with pytest.raises(tcg.TiltCalError, match='unusable node'):
        tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)


def test_validate_refuses_an_out_of_bound_residual_with_a_physical_hint():
    """> MAX_ABS_RESIDUAL_RAD is a capture fault, not a calibration — and the
    operator reading the refusal is standing at a robot, so it names causes."""
    x, y, results = _asymmetric_capture()
    results[(2, 3)] = results[(2, 3)]._replace(
        res_tx=tilt_map.MAX_ABS_RESIDUAL_RAD * 2.0)
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)
    with pytest.raises(tcg.TiltCalError) as excinfo:
        tcg.validate_map_document(doc)
    message = str(excinfo.value)
    assert 'REJECTED' in message
    assert 'stale level reference' in message
    assert 'stroke clamp' in message


def test_validate_accepts_a_residual_at_the_measured_envelope():
    """0.604 deg is the largest residual the 2026-07-28 seed table measured; a
    real capture must not trip the sanity bound."""
    x, y, results = _asymmetric_capture()
    results[(2, 3)] = results[(2, 3)]._replace(res_tx=math.radians(0.604))
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)
    assert tcg.validate_map_document(doc) is not None


def test_dumped_yaml_reloads_and_carries_a_do_not_edit_header():
    x, y, results = _asymmetric_capture()
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)
    text = tcg.dump_map_yaml(doc)
    assert 'MACHINE-WRITTEN' in text
    assert 'C-LEVEL-2' in text
    reloaded = yaml.safe_load(text)
    assert tilt_map.map_version(reloaded) == tilt_map.map_version(doc)
    tilt_map.parse_tilt_map(reloaded)


def test_written_file_loads_through_load_tilt_map(tmp_path):
    x, y, results = _asymmetric_capture()
    doc = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)
    path = tmp_path / 'tilt_calibration.yaml'
    path.write_text(tcg.dump_map_yaml(doc))
    loaded = tilt_map.load_tilt_map(str(path))
    assert loaded.version == tilt_map.map_version(doc)
    assert loaded.metadata['base_condition'] == 'unit test'


def test_map_version_is_stable_across_a_rewrite_of_identical_numbers():
    """The tool re-emits this file every run; a version that churns when
    nothing was recalibrated sends an operator hunting a difference that does
    not exist."""
    x, y, results = _asymmetric_capture()
    first = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)
    later = dict(_CAPTURED, git_sha='different', uptime_ms_last=999999)
    second = tcg.build_map_document(x, y, 170.0, results, later, 8)
    assert tilt_map.map_version(first) == tilt_map.map_version(second)


def test_map_version_changes_when_a_single_node_changes():
    x, y, results = _asymmetric_capture()
    first = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)
    results[(0, 1)] = results[(0, 1)]._replace(res_tx=0.00777)
    second = tcg.build_map_document(x, y, 170.0, results, _CAPTURED, 8)
    assert tilt_map.map_version(first) != tilt_map.map_version(second)


# ── CSV schema ───────────────────────────────────────────────────────────


def test_csv_columns_are_stable():
    """The analyser reads these names; changing one silently breaks
    ``tilt_cal_analyse.py --csv`` on every historical capture."""
    assert tcg.CSV_COLUMNS == (
        'iso', 't_s', 'phase', 'visit', 'ix', 'iy',
        'x_mm', 'y_mm', 'z_mm', 'read_index',
        'raw_tx_rad', 'raw_ty_rad', 'res_tx_rad', 'res_ty_rad',
        'ok', 'uptime_ms', 'note',
    )


def test_csv_row_keys_match_the_declared_columns_exactly():
    row = tcg.csv_row('2026-08-03T00:00:00', 1.0, 'capture', 0, 1, 2,
                      10.0, 20.0, 170.0, 3, 0.01, 0.02, OFFSET, 1234)
    assert set(row) == set(tcg.CSV_COLUMNS)


def test_csv_row_applies_the_mounting_offset_to_the_res_columns():
    row = tcg.csv_row('t', 0.0, 'capture', 0, 0, 0, 0.0, 0.0, 170.0, 0,
                      0.01, 0.02, OFFSET, None)
    assert row['ok'] == 1
    assert row['res_tx_rad'] == pytest.approx(0.01 + OFFSET[0])
    assert row['res_ty_rad'] == pytest.approx(0.02 + OFFSET[1])
    assert row['uptime_ms'] == ''


def test_csv_row_marks_a_failed_read_and_does_not_launder_nan():
    nan = float('nan')
    row = tcg.csv_row('t', 0.0, 'capture', 0, 0, 0, 0.0, 0.0, 170.0, 0,
                      nan, nan, OFFSET, 55)
    assert row['ok'] == 0
    assert math.isnan(row['res_tx_rad'])
    assert math.isnan(row['res_ty_rad'])


def test_verification_rows_are_marked_off_grid():
    row = tcg.csv_row('t', 0.0, 'verify', 2, -1, -1, 37.5, 112.5, 170.0, 0,
                      0.0, 0.0, OFFSET, 1)
    assert (row['ix'], row['iy']) == (-1, -1)
    assert row['phase'] == 'verify'


# ── write target ─────────────────────────────────────────────────────────


def test_write_target_honours_the_authoritative_env_override(monkeypatch):
    """When $JUGGLEBOT_TILT_CAL is set it is the ONLY candidate, so the tool
    must write there — writing anywhere else would apply a different
    calibration than the operator named while reporting success."""
    monkeypatch.setenv(tilt_map.TILT_MAP_ENV, '/tmp/explicit_tilt_cal.yaml')
    assert tcg.write_target_path() == '/tmp/explicit_tilt_cal.yaml'


def test_write_target_is_the_first_production_candidate(monkeypatch):
    monkeypatch.delenv(tilt_map.TILT_MAP_ENV, raising=False)
    candidates = tilt_map.tilt_map_candidates()
    if not candidates:
        pytest.skip('no candidate paths in this environment')
    assert tcg.write_target_path() == candidates[0]


def test_source_tree_detection_distinguishes_a_share_tree():
    assert tcg.looks_like_source_tree(
        '/home/x/Jugglebot/config/tilt_calibration.yaml') is False
    assert tcg.looks_like_source_tree(
        '/opt/ros/install/jugglebot/share/jugglebot/config/'
        'tilt_calibration.yaml') is False


def test_estimate_duration_includes_the_return_to_centre():
    per_visit = 3.0 + 1.0 + 2 * (tcg.EST_READ_S + 0.15)
    assert tcg.estimate_duration_s(25, 6, 3.0, 1.0, 2, 0.15) == \
        pytest.approx(32 * per_visit)


# ── CLI wiring ───────────────────────────────────────────────────────────


def test_defaults_match_the_documented_capture():
    args = tcg.build_parser().parse_args([])
    assert args.box == 150.0 and args.nodes == 5 and args.z == 170.0
    assert args.lean_gain == 0.0, 'lean must be explicitly OFF by default'
    assert args.on_fail == 'continue'
    assert args.check_poses == 6
    assert args.threshold_deg == pytest.approx(0.15)
    assert args.read_gap_s > 0.1, (
        'reads must be spaced beyond the SCL3300 10 Hz LPF period or the '
        'per-read sd measures the filter rather than the sensor')


def test_apply_is_the_default_and_no_apply_serves_rung_c0():
    assert tcg.build_parser().parse_args([]).no_apply is False
    assert tcg.build_parser().parse_args(['--no-apply']).no_apply is True


def test_verify_only_and_no_apply_are_refused_together(capsys):
    assert tcg.main(['--verify-only', '--no-apply']) == 2
    assert 'contradictory' in capsys.readouterr().err


def test_resolve_grid_prefers_explicit_lists_over_the_box_form():
    args = tcg.build_parser().parse_args(
        ['--x=-50,0,50', '--y=-60,0,60', '--box', '999'])
    x, y = tcg.resolve_grid(args)
    assert x == [-50.0, 0.0, 50.0]
    assert y == [-60.0, 0.0, 60.0]


def test_negative_node_list_without_the_equals_form_explains_itself(capsys):
    """``--x -150,...`` cannot work (argparse reads the leading '-' as an
    option) and its native error says only "expected one argument". This is the
    tool's own documented usage form, so the fix is in the message."""
    with pytest.raises(SystemExit):
        tcg.build_parser().parse_args(['--x', '-150,-75,0,75,150'])
    assert '--x=-150' in capsys.readouterr().err


def test_resolve_grid_refuses_an_even_node_count():
    args = tcg.build_parser().parse_args(['--nodes', '4'])
    with pytest.raises(tcg.TiltCalError, match='home node'):
        tcg.resolve_grid(args)


def test_return_to_centre_guard_catches_base_exception():
    """The return-to-centre guard must catch ``BaseException``, not ``Exception``.

    Structural, because ``run()`` cannot be exercised without a robot — and this
    is exactly where the one blocking defect of this phase lived. A plain
    ``except Exception`` does **not** catch ``KeyboardInterrupt``, and the second
    Ctrl-C is the reflex when a program does not die on the first. The
    return-to-centre blocks for up to ``--timeout-s`` plus the move, so that
    second interrupt lands inside the guard: with ``Exception`` it propagated out
    of the ``finally``, skipping the artefact writes and the rclpy shutdown, and
    left the platform parked at a raised displaced pose **with no message** —
    silently contradicting the one absolute safety claim the tool and its runbook
    both make.

    This test exists to stop that being "tidied" back to ``except Exception``.
    """
    source = open(tcg.__file__.replace('.pyc', '.py')).read()
    marker = 'RETURN TO CENTRE FAILED'
    assert marker in source
    # rindex, not index: the module docstring quotes this operator-facing string
    # when explaining the one case the return cannot honour (a wire disarmed
    # mid-sweep). The occurrence that matters is the `print` in the guard, which
    # is the LAST one — anchoring on the first made the test fail on a prose
    # mention rather than on the handler it exists to pin.
    guard = source[:source.rindex(marker)]
    handler = guard.rsplit('except ', 1)[1].split('\n')[0]
    assert handler.startswith('BaseException'), (
        'the return-to-centre handler must catch BaseException so a second '
        'Ctrl-C cannot skip it silently; found "except {}"'.format(handler))


def test_dry_run_makes_no_ros_calls(capsys, monkeypatch):
    """--dry-run must construct nothing: it is the rehearsal an operator runs
    on a machine that is not even powered."""
    def _explode(*_args, **_kwargs):
        raise AssertionError('dry-run imported/used rclpy')

    monkeypatch.setitem(sys.modules, 'rclpy', _explode)
    assert tcg.main(['--dry-run']) == 0
    out = capsys.readouterr().out
    assert 'home node (stale-level gate)' in out
    assert 'no ROS calls made' in out
    assert 'ETA' in out


# ── the disarmed-wire guard ──────────────────────────────────────────────
#
# The BLOCKING class this section pins: `trajectory/go_to_pose` is ACCEPTED
# while the wire is disarmed (streaming-while-disarmed is the legal pre-arm
# phase of the ARMING CONTRACT), the platform does not move, every node
# re-measures one stationary pose, and the tool writes + applies + "verifies" an
# all-zeros calibration that verifies perfectly BECAUSE nothing moved. Precedent
# for the silent-no-op shape: the 2026-07-15 ramp battery.


def test_disarmed_marker_is_detected_in_an_accepted_response():
    """The real suffix `trajectory_node._wire_state_suffix` appends."""
    accepted = ('move accepted: target (x=150.0 y=0.0 z=170.0 mm) over 3.000s '
                '(lean_gain=0.00) [wire DISARMED — setpoints not reaching the '
                'legs]')
    assert tcg.response_reports_disarmed(accepted) is True


def test_a_normal_accepted_response_is_not_flagged():
    """No false positives, or the first operator to hit one disables the check."""
    accepted = ('move accepted: target (x=150.0 y=0.0 z=170.0 mm) over 3.000s '
                '(lean_gain=0.00)')
    assert tcg.response_reports_disarmed(accepted) is False
    assert tcg.response_reports_disarmed('') is False
    assert tcg.response_reports_disarmed(None) is False


def test_disarmed_detection_is_case_insensitive():
    """Keyed on the WORD, not the wording: the sentence around it is a log
    string that may be reworded, the marker itself is the contract."""
    assert tcg.response_reports_disarmed('accepted [wire disarmed]') is True


def _kv(**overrides):
    """A /link_status key-value mapping in the encodings the bridge publishes.

    Verified against `teensy_bridge_node._publish_link_status`, not assumed:
    `mpc_active` is `str(int(bool))`, `fault_state` is a FaultState enum NAME,
    `bridge_link` is 'UP'/'LOST'/'NO_HEARTBEAT'.
    """
    kv = {'mpc_active': '1', 'fault_state': 'NONE', 'bridge_link': 'UP',
          'uptime_ms': '12345'}
    kv.update(overrides)
    return kv


def test_wire_verdict_passes_on_an_armed_link():
    ok, message = tcg.wire_armed_verdict(_kv())
    assert ok is True
    assert 'ARMED' in message


@pytest.mark.parametrize('overrides,needle', [
    ({'mpc_active': '0'}, 'DISARMED'),
    ({'fault_state': 'MAX_DEVIATION'}, 'MAX_DEVIATION'),
    # 'UNKNOWN' is what the bridge publishes when it has seen NO heartbeat —
    # not a clear guard, and it must not read as one.
    ({'fault_state': 'UNKNOWN'}, 'UNKNOWN'),
    ({'bridge_link': 'LOST'}, 'bridge_link=LOST'),
    ({'bridge_link': 'NO_HEARTBEAT'}, 'NO_HEARTBEAT'),
])
def test_wire_verdict_refuses_every_unsafe_link_state(overrides, needle):
    ok, message = tcg.wire_armed_verdict(_kv(**overrides))
    assert ok is False
    assert needle in message


def test_wire_verdict_refuses_an_empty_or_partial_cache():
    """An absent key is a FAILURE, never a pass.

    This runs against a cache that is empty until the first message arrives.
    Defaulting to "armed" there would reintroduce the exact hole it closes —
    a preflight that passes because nothing has been heard yet.
    """
    ok, message = tcg.wire_armed_verdict({})
    assert ok is False and 'no /link_status' in message
    ok, message = tcg.wire_armed_verdict(None)
    assert ok is False

    # ALL THREE keys are required, not just mpc_active. A bridge that publishes
    # mpc_active publishes the other two in the same message, so a missing
    # companion means this is not the publisher the tool thinks it is reading —
    # and falling through on it would silently skip a real refusal (a latched
    # guard fault, or a downed UDP link).
    for key in ('mpc_active', 'fault_state', 'bridge_link'):
        partial = _kv()
        del partial[key]
        ok, message = tcg.wire_armed_verdict(partial)
        assert ok is False, '{} must be required'.format(key)
        assert key in message


def _stats(res_tx, res_ty, n_ok=8, n_total=8, sd=1e-6):
    """A ReadStats with the raw means filled in consistently.

    `raw_mean_*` are the un-offset sensor means; the flat-field check reads only
    `res_*` and `n_ok`, but constructing a NamedTuple partially would drift the
    moment a field is added.
    """
    return tcg.ReadStats(res_tx=res_tx, res_ty=res_ty, sd_tx=sd, sd_ty=sd,
                         raw_mean_tx=res_tx - OFFSET[0],
                         raw_mean_ty=res_ty - OFFSET[1],
                         n_ok=n_ok, n_total=n_total)


def test_flat_field_is_detected_as_the_disarmed_signature():
    """Every node the same number ⇒ the platform never moved."""
    results = {(ix, iy): _stats(0.0021, -0.0013)
               for ix in range(3) for iy in range(3)}
    is_flat, message = tcg.flat_field_verdict(results)
    assert is_flat is True
    assert 'FLAT FIELD' in message
    assert 'NEVER MOVED' in message


def test_a_real_field_is_not_flagged_flat():
    """A real field spans ~1e-2 rad across the workspace — two orders above the
    tolerance — so the check must never fire on a good capture."""
    results = {}
    for ix in range(3):
        for iy in range(3):
            results[(ix, iy)] = _stats(0.001 * ix, -0.001 * iy)
    is_flat, _ = tcg.flat_field_verdict(results)
    assert is_flat is False


def test_flat_field_is_undefined_below_two_usable_nodes():
    """One node cannot be 'flat'; an unusable node must not be counted."""
    one = {(0, 0): _stats(0.0, 0.0)}
    assert tcg.flat_field_verdict(one)[0] is False
    plus_dead = dict(one)
    plus_dead[(1, 0)] = _stats(0.0, 0.0, n_ok=0)
    assert tcg.flat_field_verdict(plus_dead)[0] is False


def test_the_wire_check_runs_before_the_per_node_try_block():
    """Structural: `assert_wire_armed` must sit OUTSIDE the per-node try.

    `run()` needs a robot, so this is checked in the source. The placement is
    the whole point: the try below converts a `TiltCalError` into a failed
    *node* and, under the default `--on-fail continue`, carries on. A wire that
    disarmed mid-sweep is never "this one node failed" — every remaining node
    would re-measure the same stationary pose — so demoting it there would
    reinstate the bug this guard exists to close.
    """
    source = open(tcg.__file__.replace('.pyc', '.py')).read()
    marker = 'stats, raws = runner.measure(nd.x_mm, nd.y_mm, z_mm, label,'
    assert marker in source
    prologue = source[:source.index(marker)]
    tail = prologue.rsplit('runner.assert_wire_armed(', 1)[1]
    assert tail.count('try:') == 1, (
        'assert_wire_armed must be called before the per-node `try:`, so a '
        'mid-sweep disarm aborts the run instead of scoring one bad node')


def test_go_to_checks_the_response_for_the_disarmed_marker():
    """Structural: every accepted `go_to_pose` is screened, in `go_to` itself.

    Screening at the single choke point rather than per call site is what makes
    the guarantee total — capture nodes, verification poses and the
    return-to-centre all route through this one method.
    """
    source = open(tcg.__file__.replace('.pyc', '.py')).read()
    body = source.split('def go_to(', 1)[1].split('\n    def ', 1)[0]
    assert 'response_reports_disarmed(response.message)' in body


# ── --force-uninstall: the ament-share shadow ────────────────────────────


def test_uninstall_plan_moves_every_existing_candidate(monkeypatch, tmp_path):
    """Source tree AND ament share — moving only the source file is a no-op.

    `setup.py` installs `config/tilt_calibration.yaml` into
    `share/jugglebot/config/` whenever it exists at build time, and the resolver
    order is env -> source tree -> share. So after ANY colcon build with the
    file present, removing just the source copy falls through to the share copy:
    the reload succeeds, the OLD map stays loaded, and the capture is refused
    with a message naming neither the cause nor the remedy.
    """
    src = tmp_path / 'src' / 'tilt_calibration.yaml'
    share = tmp_path / 'share' / 'tilt_calibration.yaml'
    absent = tmp_path / 'gone' / 'tilt_calibration.yaml'
    for path in (src, share):
        path.parent.mkdir(parents=True)
        path.write_text('x')

    monkeypatch.delenv(tilt_map.TILT_MAP_ENV, raising=False)
    monkeypatch.setattr(tilt_map, 'tilt_map_candidates',
                        lambda environ=None: (str(src), str(share),
                                              str(absent)))
    to_move, env_override = tcg.uninstall_plan()
    assert env_override is None
    assert to_move == [str(src), str(share)], \
        'every EXISTING candidate must be moved aside, not just the first'


def test_uninstall_plan_refuses_to_touch_an_env_pointed_file(monkeypatch,
                                                            tmp_path):
    """An operator's `$JUGGLEBOT_TILT_CAL` file is never renamed behind them.

    When the override is set it is the ONLY candidate, and it names a file the
    operator chose — possibly outside the repo, possibly the artefact they are
    protecting. The caller refuses with instructions instead.
    """
    named = tmp_path / 'my_calibration.yaml'
    named.write_text('x')
    monkeypatch.setenv(tilt_map.TILT_MAP_ENV, str(named))
    to_move, env_override = tcg.uninstall_plan()
    assert to_move == []
    assert env_override == str(named)
    assert named.exists()
