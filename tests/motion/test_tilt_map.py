"""Unit tests for the pose-dependent residual tilt map (`motion/tilt_map.py`)
and its single application entry point (`levelling.correction_for_pose`).

Contract: `ros_ws/docs/levelling_frame.md` (**C-LEVEL-2**, which composes with
C-LEVEL-1). No ROS mocking: `motion/` is pure Python by design.

Two deliberate choices in the fixtures, both load-bearing:

* The map is **asymmetric in every way** — 3 x-nodes vs 4 y-nodes, unequal
  spacing on both axes, and no two residual values equal. A transposed
  `[iy][ix]` index, a swapped axis, or an x/y-swapped query is invisible on a
  symmetric grid and this is the shape of map that catches all three.
* The interpolation expectations are **hand-computed weighted sums written out
  in the test**, not recomputed with the production formula, so a rewritten
  `lookup` cannot drag its tests along with it.
"""

from __future__ import annotations

import copy
import os
import math

import numpy as np
import pytest
import yaml

from jugglebot.motion import levelling, tilt_map
from jugglebot.motion.ik_solver import rot_matrix_to_rotvec, rotvec_to_rot_matrix


# ── fixtures ─────────────────────────────────────────────────────

# Asymmetric on purpose: 3 x-nodes, 4 y-nodes, unequal spacing on both axes.
_X_MM = [-100.0, 0.0, 50.0]
_Y_MM = [-60.0, 0.0, 40.0, 120.0]

# Indexed [iy][ix]. Every value distinct, both signs present in ty.
_TX = [[0.001, 0.002, 0.003],
       [0.004, 0.005, 0.006],
       [0.007, 0.008, 0.009],
       [0.010, 0.011, 0.012]]
_TY = [[-0.001, 0.0015, 0.0000],
       [0.002, -0.003, 0.004],
       [0.005, 0.006, -0.007],
       [0.008, -0.009, 0.010]]


def _doc() -> dict:
    """A fresh, valid schema-v1 document (deep-copied — tests mutate it)."""
    return copy.deepcopy({
        'version': 1,
        'captured': {
            'date': '2026-08-02',
            'git_sha': 'deadbeef',
            'tool': 'tests/hardware/tilt_cal_grid.py',
            'uptime_ms_first': 41234,
            'uptime_ms_last': 902311,
            'level_offset_rad': [0.0136, 0.0012],
            'base_condition': 'nominal, no shims',
        },
        'grid': {
            'z_mm': 170.0,
            'orientation': 'level',
            'x_mm': list(_X_MM),
            'y_mm': list(_Y_MM),
        },
        'residual_rad': {'tx': [list(r) for r in _TX],
                         'ty': [list(r) for r in _TY]},
        'stats': {'n_reads': 30, 'failed_nodes': []},
    })


def _map() -> tilt_map.TiltMap:
    return tilt_map.parse_tilt_map(_doc())


def _zero_map() -> tilt_map.TiltMap:
    doc = _doc()
    doc['residual_rad'] = {
        'tx': [[0.0] * len(_X_MM) for _ in _Y_MM],
        'ty': [[0.0] * len(_X_MM) for _ in _Y_MM]}
    return tilt_map.parse_tilt_map(doc)


# ── parse: the happy path ────────────────────────────────────────


def test_parse_builds_the_expected_arrays():
    tm = _map()
    assert np.array_equal(tm.x_mm, _X_MM)
    assert np.array_equal(tm.y_mm, _Y_MM)
    assert tm.shape == (4, 3), "shape is (n_y, n_x) — rows are y"
    assert np.array_equal(tm.tx_rad, _TX)
    assert np.array_equal(tm.ty_rad, _TY)
    assert tm.z_mm == 170.0
    assert tm.metadata['git_sha'] == 'deadbeef'
    assert tm.metadata['base_condition'] == 'nominal, no shims'


def test_parsed_arrays_are_read_only():
    """One in-place write on the shared map would re-frame every later pose."""
    tm = _map()
    for array in (tm.x_mm, tm.y_mm, tm.tx_rad, tm.ty_rad):
        with pytest.raises(ValueError):
            array[0] = 99.0


def test_parse_does_not_freeze_the_callers_own_arrays():
    """A caller self-validating its working arrays must not find them read-only.

    `np.asarray` returns the SAME object for a float64 ndarray, so freezing in
    the parser would reach back into the caller's array. The Phase-3 acquisition
    tool builds the document from its own numpy grids, so this is a real path.
    """
    caller_tx = np.array(_TX, dtype=float)
    doc = _doc()
    doc['residual_rad']['tx'] = caller_tx
    tm = tilt_map.parse_tilt_map(doc)
    assert caller_tx.flags.writeable, "parse_tilt_map froze an array it does not own"
    assert not tm.tx_rad.flags.writeable, 'the map''s own copy must be frozen'
    caller_tx[0][0] = 99.0
    assert tm.tx_rad[0][0] == _TX[0][0], 'the map must not alias the caller'


def test_hand_built_map_with_a_degenerate_axis_raises_tilt_map_error():
    """The invariant travels with the TYPE, not only with `parse_tilt_map`.

    `lookup` divides by a cell width, so a 1-node axis used to raise
    `ZeroDivisionError` — a type no caller is told to expect, and one that walks
    straight through a handler catching `TiltMapError`.
    """
    with pytest.raises(tilt_map.TiltMapError, match='at least 2 nodes'):
        tilt_map.TiltMap(x_mm=np.array([0.0]), y_mm=np.array(_Y_MM),
                         tx_rad=np.zeros((4, 1)), ty_rad=np.zeros((4, 1)),
                         version='hand-built')


def test_hand_built_map_with_mismatched_grids_raises_tilt_map_error():
    with pytest.raises(tilt_map.TiltMapError, match=r'\[iy\]\[ix\]'):
        tilt_map.TiltMap(x_mm=np.array(_X_MM), y_mm=np.array(_Y_MM),
                         tx_rad=np.zeros((3, 4)), ty_rad=np.zeros((4, 3)),
                         version='hand-built')


def test_hand_built_map_with_non_monotonic_axis_raises_tilt_map_error():
    with pytest.raises(tilt_map.TiltMapError, match='strictly increasing'):
        tilt_map.TiltMap(x_mm=np.array([0.0, 0.0, 50.0]), y_mm=np.array(_Y_MM),
                         tx_rad=np.zeros((4, 3)), ty_rad=np.zeros((4, 3)),
                         version='hand-built')


def test_hand_built_map_from_lists_is_usable():
    """Construction coerces to float arrays, so a list-built map still looks up."""
    tm = tilt_map.TiltMap(x_mm=_X_MM, y_mm=_Y_MM, tx_rad=_TX, ty_rad=_TY,
                          version='hand-built')
    assert tilt_map.lookup(tm, -50.0, -30.0)[0] == pytest.approx(0.003, abs=1e-15)


def test_missing_optional_blocks_are_tolerated():
    """`captured` and `stats` are provenance — a map without them still loads."""
    doc = _doc()
    del doc['captured']
    del doc['stats']
    del doc['grid']['z_mm']
    tm = tilt_map.parse_tilt_map(doc)
    assert tm.metadata == {}
    assert tm.z_mm is None
    assert tm.version.startswith('undated-')


# ── lookup: exactness at nodes ───────────────────────────────────


def test_lookup_is_exact_at_every_node():
    """Every grid node returns its stored pair bit-identically.

    Exactness matters beyond tidiness: the home node is ~0 by construction and
    the verification pass re-reads at nodes, so an interpolation that is merely
    *close* at a node would show up as calibration error that isn't there.
    """
    tm = _map()
    for iy, y in enumerate(_Y_MM):
        for ix, x in enumerate(_X_MM):
            assert tilt_map.lookup(tm, x, y) == (_TX[iy][ix], _TY[iy][ix]), (
                f"node (ix={ix}, iy={iy}) is not exact")


def test_lookup_does_not_transpose_the_grid():
    """A swapped [iy][ix] index would still pass a symmetric-grid test.

    (x=50, y=-60) is the +x edge of the FIRST y-row ⇒ tx = 0.003. The
    transposed read at the mirrored query (x=-100, y=40) is 0.007 — different,
    so this pair distinguishes the two indexings.
    """
    tm = _map()
    assert tilt_map.lookup(tm, 50.0, -60.0)[0] == 0.003
    assert tilt_map.lookup(tm, -100.0, 40.0)[0] == 0.007


# ── lookup: interpolation ────────────────────────────────────────


def test_lookup_at_cell_midpoint_is_the_four_corner_mean():
    """Cell x∈[-100, 0], y∈[-60, 0] — midpoint (-50, -30), all weights 1/4.

    tx = (0.001 + 0.002 + 0.004 + 0.005)/4 = 0.003
    ty = (-0.001 + 0.0015 + 0.002 - 0.003)/4 = -0.000125
    """
    tm = _map()
    tx, ty = tilt_map.lookup(tm, -50.0, -30.0)
    assert tx == pytest.approx(0.003, abs=1e-15)
    assert ty == pytest.approx(-0.000125, abs=1e-15)


def test_lookup_at_a_second_cell_midpoint():
    """Cell x∈[0, 50], y∈[40, 120] — midpoint (25, 80), unequal cell size.

    tx = (0.008 + 0.009 + 0.011 + 0.012)/4 = 0.010
    ty = (0.006 - 0.007 - 0.009 + 0.010)/4 = 0.0
    """
    tm = _map()
    tx, ty = tilt_map.lookup(tm, 25.0, 80.0)
    assert tx == pytest.approx(0.010, abs=1e-15)
    assert ty == pytest.approx(0.0, abs=1e-15)


def test_lookup_at_an_off_centre_interior_point():
    """(x=-25, y=80): fx = 75/100 = 0.75 in cell ix=0, fy = 40/80 = 0.5 in iy=2.

    Weights (w00, w10, w01, w11) = (0.125, 0.375, 0.125, 0.375). Written out
    by hand rather than recomputed, so the test cannot follow a rewrite of the
    production formula.
    """
    tm = _map()
    expected_tx = (0.125 * 0.007 + 0.375 * 0.008
                   + 0.125 * 0.010 + 0.375 * 0.011)
    expected_ty = (0.125 * 0.005 + 0.375 * 0.006
                   + 0.125 * 0.008 + 0.375 * -0.009)
    tx, ty = tilt_map.lookup(tm, -25.0, 80.0)
    assert tx == pytest.approx(expected_tx, abs=1e-15)
    assert ty == pytest.approx(expected_ty, abs=1e-15)
    # ...and the pinned decimal values, so a sign error in the weights above
    # cannot be "corrected" into agreement with a broken implementation.
    assert tx == pytest.approx(0.00925, abs=1e-12)
    assert ty == pytest.approx(0.00050, abs=1e-12)


def test_lookup_along_an_edge_reduces_to_1d_interpolation():
    """On the y = -60 edge at x = -50 (fx = 0.5): tx = (0.001 + 0.002)/2."""
    tm = _map()
    tx, ty = tilt_map.lookup(tm, -50.0, -60.0)
    assert tx == pytest.approx(0.0015, abs=1e-15)
    assert ty == pytest.approx((-0.001 + 0.0015) / 2.0, abs=1e-15)


# ── lookup: clamp to hull, never extrapolate ─────────────────────


def test_lookup_clamps_on_all_four_edges():
    """Outside the hull the nearest hull point is returned, not an extrapolation.

    An edge extrapolation with the wrong sign aims a throw *worse* than no map,
    which is why the contract forbids it outright.
    """
    tm = _map()
    # −x and +x, at an interior y (mid-cell so the clamp cannot hide in a node).
    assert tilt_map.lookup(tm, -1e6, -30.0) == tilt_map.lookup(tm, -100.0, -30.0)
    assert tilt_map.lookup(tm, +1e6, -30.0) == tilt_map.lookup(tm, 50.0, -30.0)
    # −y and +y, at an interior x.
    assert tilt_map.lookup(tm, -50.0, -1e6) == tilt_map.lookup(tm, -50.0, -60.0)
    assert tilt_map.lookup(tm, -50.0, +1e6) == tilt_map.lookup(tm, -50.0, 120.0)


def test_lookup_clamps_at_all_four_corners():
    tm = _map()
    corners = {(-1e6, -1e6): (_TX[0][0], _TY[0][0]),
               (+1e6, -1e6): (_TX[0][-1], _TY[0][-1]),
               (-1e6, +1e6): (_TX[-1][0], _TY[-1][0]),
               (+1e6, +1e6): (_TX[-1][-1], _TY[-1][-1])}
    for (x, y), expected in corners.items():
        assert tilt_map.lookup(tm, x, y) == expected, f"corner ({x}, {y})"


def test_lookup_is_bounded_by_the_node_extremes_everywhere():
    """A property check: no query anywhere on the plane may leave the map's range.

    This is the extrapolation guard stated as an invariant rather than as four
    edge cases — including well outside the hull, where extrapolation would be
    unbounded.
    """
    tm = _map()
    lo_tx, hi_tx = float(np.min(tm.tx_rad)), float(np.max(tm.tx_rad))
    lo_ty, hi_ty = float(np.min(tm.ty_rad)), float(np.max(tm.ty_rad))
    rng = np.random.default_rng(20260802)
    for x, y in rng.uniform(-5000.0, 5000.0, size=(400, 2)):
        tx, ty = tilt_map.lookup(tm, float(x), float(y))
        assert lo_tx <= tx <= hi_tx
        assert lo_ty <= ty <= hi_ty


def test_lookup_refuses_a_non_finite_query():
    """NaN would be summed into the offset and negated into the commanded
    rotation, surfacing only downstream as a feasibility rejection."""
    tm = _map()
    for bad in (float('nan'), float('inf'), float('-inf')):
        with pytest.raises(tilt_map.TiltMapError):
            tilt_map.lookup(tm, bad, 0.0)
        with pytest.raises(tilt_map.TiltMapError):
            tilt_map.lookup(tm, 0.0, bad)


# ── validation: every failure mode is a loud TiltMapError ────────


@pytest.mark.parametrize('bad_version', [2, 0, '1', 1.0, None, True])
def test_rejects_unknown_schema_version(bad_version):
    doc = _doc()
    doc['version'] = bad_version
    with pytest.raises(tilt_map.TiltMapError, match='version'):
        tilt_map.parse_tilt_map(doc)


def test_rejects_missing_version():
    doc = _doc()
    del doc['version']
    with pytest.raises(tilt_map.TiltMapError, match="'version'"):
        tilt_map.parse_tilt_map(doc)


@pytest.mark.parametrize('block', ['grid', 'residual_rad'])
def test_rejects_missing_required_block(block):
    doc = _doc()
    del doc[block]
    with pytest.raises(tilt_map.TiltMapError, match=block):
        tilt_map.parse_tilt_map(doc)


@pytest.mark.parametrize('axis', ['x_mm', 'y_mm'])
def test_rejects_missing_axis(axis):
    doc = _doc()
    del doc['grid'][axis]
    with pytest.raises(tilt_map.TiltMapError, match=axis):
        tilt_map.parse_tilt_map(doc)


@pytest.mark.parametrize('axis,values', [
    ('x_mm', [-100.0, 50.0, 0.0]),      # descending step
    ('y_mm', [-60.0, 0.0, 0.0, 120.0]),  # repeated node ⇒ zero-width cell
])
def test_rejects_non_monotonic_axis(axis, values):
    """A repeated node is a zero-width cell — a divide-by-zero in the weights."""
    doc = _doc()
    doc['grid'][axis] = values
    with pytest.raises(tilt_map.TiltMapError, match='increasing'):
        tilt_map.parse_tilt_map(doc)


def test_rejects_single_node_axis():
    """One node is a constant offset masquerading as a map, not a grid."""
    doc = _doc()
    doc['grid']['x_mm'] = [0.0]
    doc['residual_rad']['tx'] = [[0.001], [0.004], [0.007], [0.010]]
    doc['residual_rad']['ty'] = [[0.001], [0.004], [0.007], [0.010]]
    with pytest.raises(tilt_map.TiltMapError, match='at least 2 nodes'):
        tilt_map.parse_tilt_map(doc)


@pytest.mark.parametrize('field_name', ['tx', 'ty'])
def test_rejects_nan_node(field_name):
    doc = _doc()
    doc['residual_rad'][field_name][2][1] = float('nan')
    with pytest.raises(tilt_map.TiltMapError, match='non-finite'):
        tilt_map.parse_tilt_map(doc)


@pytest.mark.parametrize('field_name', ['tx', 'ty'])
def test_rejects_infinite_node(field_name):
    doc = _doc()
    doc['residual_rad'][field_name][0][0] = float('inf')
    with pytest.raises(tilt_map.TiltMapError, match='non-finite'):
        tilt_map.parse_tilt_map(doc)


def test_rejects_shape_mismatch():
    """A grid that is [ix][iy] instead of [iy][ix] is caught by the shape check
    (3x4 vs the required 4x3) — the transposition is not silently accepted."""
    doc = _doc()
    doc['residual_rad']['tx'] = [list(col) for col in zip(*_TX)]
    with pytest.raises(tilt_map.TiltMapError, match=r'shape'):
        tilt_map.parse_tilt_map(doc)


def test_rejects_short_row():
    doc = _doc()
    doc['residual_rad']['ty'][1] = [0.002, -0.003]
    with pytest.raises(tilt_map.TiltMapError):
        tilt_map.parse_tilt_map(doc)


def test_rejects_over_bound_residual():
    """0.06 rad = 3.4°, against a measured signal of 0.04–0.6°: a capture fault."""
    doc = _doc()
    doc['residual_rad']['tx'][3][2] = 0.06
    with pytest.raises(tilt_map.TiltMapError, match='sanity bound'):
        tilt_map.parse_tilt_map(doc)


def test_accepts_residual_exactly_at_the_bound():
    """The bound is inclusive — a map is refused for exceeding it, not meeting it."""
    doc = _doc()
    doc['residual_rad']['tx'][3][2] = tilt_map.MAX_ABS_RESIDUAL_RAD
    doc['residual_rad']['ty'][0][0] = -tilt_map.MAX_ABS_RESIDUAL_RAD
    assert tilt_map.parse_tilt_map(doc).shape == (4, 3)


@pytest.mark.parametrize('doc', [None, [], 'a string', 42])
def test_rejects_non_mapping_document(doc):
    with pytest.raises(tilt_map.TiltMapError):
        tilt_map.parse_tilt_map(doc)


def test_rejects_non_numeric_node():
    doc = _doc()
    doc['residual_rad']['tx'][0][0] = 'zero'
    with pytest.raises(tilt_map.TiltMapError):
        tilt_map.parse_tilt_map(doc)


def test_rejects_non_mapping_captured_block():
    doc = _doc()
    doc['captured'] = ['2026-08-02']
    with pytest.raises(tilt_map.TiltMapError, match='captured'):
        tilt_map.parse_tilt_map(doc)


def test_tilt_map_error_is_a_value_error():
    """Callers funnelling malformed config through `except ValueError` keep working."""
    assert issubclass(tilt_map.TiltMapError, ValueError)


# ── version string ───────────────────────────────────────────────


def test_map_version_is_date_plus_content_hash():
    version = tilt_map.map_version(_doc())
    assert version.startswith('2026-08-02-')
    assert version.count('-') == 3, 'ISO date + 8-hex digest'
    assert len(version.split('-')[-1]) == 8
    assert tilt_map.map_version(_doc()) == version, 'must be deterministic'


def test_map_version_changes_with_any_residual_edit():
    before = tilt_map.map_version(_doc())
    doc = _doc()
    doc['residual_rad']['ty'][2][2] += 1e-9
    assert tilt_map.map_version(doc) != before


def test_map_version_ignores_provenance_only_edits():
    """Same numbers ⇒ same applied calibration ⇒ same reported version.

    Includes the two provenance fields that live INSIDE the `grid` block —
    `z_mm` and `orientation`. Hashing the whole block would churn the version on
    either, and `z_mm` is documented as never keyed on.
    """
    before = tilt_map.map_version(_doc())
    doc = _doc()
    doc['captured']['base_condition'] = 'shimmed 1.5deg about +x'
    doc['stats']['n_reads'] = 60
    doc['grid']['orientation'] = 'level (rechecked)'
    doc['grid']['z_mm'] = 171.0
    assert tilt_map.map_version(doc) == before


def test_map_version_is_stable_across_int_float_serialisation():
    """`170` and `170.0` are the same calibration written by two serializers.

    The Phase-3 tool re-emits this file; a version that changes when nothing was
    recalibrated sends an operator hunting a difference that does not exist.
    """
    before = tilt_map.map_version(_doc())
    doc = _doc()
    doc['grid']['z_mm'] = 170
    doc['grid']['x_mm'] = [-100, 0, 50]          # ints, same values
    doc['grid']['y_mm'] = [-60, 0, 40, 120]
    assert tilt_map.map_version(doc) == before


def test_map_version_changes_when_an_axis_moves():
    """The axes ARE applied content — a moved node changes what is interpolated."""
    before = tilt_map.map_version(_doc())
    doc = _doc()
    doc['grid']['x_mm'] = [-100.0, 1.0, 50.0]
    assert tilt_map.map_version(doc) != before


# ── load_tilt_map: file-level failures are the same type ─────────


def test_load_round_trips_a_written_file(tmp_path):
    path = tmp_path / 'tilt_calibration.yaml'
    path.write_text(yaml.safe_dump(_doc()))
    tm = tilt_map.load_tilt_map(str(path))
    assert np.array_equal(tm.tx_rad, _TX)
    assert tm.version == tilt_map.map_version(_doc())


def test_load_rejects_a_missing_file(tmp_path):
    with pytest.raises(tilt_map.TiltMapError):
        tilt_map.load_tilt_map(str(tmp_path / 'nope.yaml'))


def test_load_rejects_malformed_yaml(tmp_path):
    path = tmp_path / 'broken.yaml'
    path.write_text('version: 1\n  grid: [oops\n')
    with pytest.raises(tilt_map.TiltMapError):
        tilt_map.load_tilt_map(str(path))


def test_load_rejects_an_empty_file(tmp_path):
    path = tmp_path / 'empty.yaml'
    path.write_text('')
    with pytest.raises(tilt_map.TiltMapError, match='empty'):
        tilt_map.load_tilt_map(str(path))


def test_load_error_names_the_path(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump({'version': 7}))
    with pytest.raises(tilt_map.TiltMapError, match='bad.yaml'):
        tilt_map.load_tilt_map(str(path))


# ── path resolution ──────────────────────────────────────────────
#
# Pure, so it is tested here rather than through the node. What it answers is
# "WHICH file", and getting that wrong applies a real calibration that is simply
# not the one the operator meant — a failure with a green status line.


def test_env_override_wins_and_is_the_only_candidate(tmp_path):
    """The override is AUTHORITATIVE — it does not fall through when missing.

    Falling through would apply a *different* calibration than the operator
    named while reporting `tilt_map_loaded=True` and a plausible version. Since
    C-LEVEL-2 is non-gating, loading nothing costs only precision, and loading
    the wrong map costs a mis-aimed throw. So a typo degrades to offset-only.
    """
    real = tmp_path / 'tilt_calibration.yaml'
    real.write_text(yaml.safe_dump(_doc()))
    env = {tilt_map.TILT_MAP_ENV: str(real)}
    assert tilt_map.tilt_map_candidates(env) == (str(real),)
    assert tilt_map.resolve_tilt_map_path(env) == str(real)

    typo = {tilt_map.TILT_MAP_ENV: str(tmp_path / 'tpyo.yaml')}
    assert tilt_map.tilt_map_candidates(typo) == (str(tmp_path / 'tpyo.yaml'),)
    assert tilt_map.resolve_tilt_map_path(typo) is None


def test_source_tree_is_tried_before_the_ament_share_dir():
    """The inversion of `friction_ff_params.py`'s order, pinned.

    The acquisition tool rewrites the SOURCE file at runtime and reloads it live;
    share-first would serve the previous `colcon build`'s copy — stale, silent,
    and reported as loaded.

    The `isdir` assertion is the load-bearing one and is NOT decoration: an
    earlier draft of this resolver used a fixed five-level `__file__` walk, which
    from the colcon install tree lands on `ros_ws/install/jugglebot` — so
    candidate 0 was `install/jugglebot/config/tilt_calibration.yaml`, a path
    nothing ever creates (`setup.py` installs into `share/jugglebot/config/`).
    Every assertion about ORDER still passed while the source-tree candidate was
    incapable of existing in production and the share copy always won.
    """
    candidates = tilt_map.tilt_map_candidates({})
    assert candidates, 'there must always be at least the source-tree candidate'
    assert candidates[0] == tilt_map._SRC_TILT_YAML
    assert os.path.isdir(os.path.dirname(candidates[0])), (
        f'candidate 0 must name a REAL config dir, not a path that can never '
        f'exist: {candidates[0]}')
    shares = [i for i, c in enumerate(candidates)
              if os.sep + 'share' + os.sep in c]
    assert all(i > 0 for i in shares), (
        f'the ament share copy must never precede the source tree: {candidates}')


def test_repo_root_is_found_from_an_install_tree_copy(tmp_path):
    """`colcon` COPIES the package, so resolution must survive relocation.

    From `ros_ws/install/jugglebot/lib/python3.8/site-packages/jugglebot/motion/`
    a fixed five-level walk yields `ros_ws/install/jugglebot`, whose `config/`
    directory is never created by anything. The consequence is not a crash but a
    silent downgrade: the source-tree candidate can never exist, so every load in
    production resolves the build-time share copy, and the acquisition tool's
    write-then-reload cycle re-applies the PREVIOUS build's calibration while
    reporting `tilt_map_loaded=true` with a plausible version string. Searching
    for the repo-root marker instead makes the answer independent of how deep the
    package was installed.
    """
    repo = tmp_path / 'Jugglebot'
    (repo / 'config').mkdir(parents=True)
    (repo / 'config' / 'hardware_config.yaml').write_text('friction_ff: {}\n')

    installed = (repo / 'ros_ws' / 'install' / 'jugglebot' / 'lib'
                 / 'python3.8' / 'site-packages' / 'jugglebot' / 'motion')
    installed.mkdir(parents=True)
    assert tilt_map._find_repo_root(str(installed / 'tilt_map.py')) == str(repo)

    source = repo / 'ros_ws' / 'src' / 'jugglebot' / 'jugglebot' / 'motion'
    source.mkdir(parents=True)
    assert tilt_map._find_repo_root(str(source / 'tilt_map.py')) == str(repo)

    # The fixed five-level walk the search replaced, shown failing from the
    # install tree — so this test cannot pass by the two trees coinciding.
    fixed = os.path.normpath(
        os.path.join(str(installed), *(['..'] * 5)))
    assert fixed != str(repo)
    assert not os.path.isdir(os.path.join(fixed, 'config'))


def test_a_deployment_outside_the_repo_has_no_source_candidate(tmp_path):
    """No repo root ⇒ no source-tree candidate, and no bogus one either.

    A genuinely detached deployment must fall through to the ament share copy
    (which IS the right answer there) rather than offering a fabricated path that
    can never exist, and `$JUGGLEBOT_TILT_CAL` remains the escape hatch.
    """
    stray = tmp_path / 'opt' / 'jugglebot' / 'motion'
    stray.mkdir(parents=True)
    assert tilt_map._find_repo_root(str(stray / 'tilt_map.py')) is None


def test_an_absent_map_resolves_to_none_rather_than_raising(tmp_path):
    """`friction_ff_params` fail-fasts with FileNotFoundError; this must not.

    Its YAML is a hard runtime dependency. This one is a refinement, and
    C-LEVEL-2 makes an absent map a silent, non-gating state — raising here
    would turn "uncalibrated" into "cannot start".
    """
    assert tilt_map.resolve_tilt_map_path(
        {tilt_map.TILT_MAP_ENV: str(tmp_path / 'absent.yaml')}) is None


# ── levelling.correction_for_pose — the C-LEVEL-2 entry point ────

_LEVEL_OFFSET = (0.010, -0.006)


def test_no_map_is_bit_identical_to_c_level_1():
    """C-LEVEL-2 is non-gating: no map ⇒ exactly today's behaviour, to the bit."""
    pose = np.array([120.0, -95.0, 170.0, 0.02, -0.01, 0.0])
    expected = levelling.correction_from_offset(*_LEVEL_OFFSET)
    got = levelling.correction_for_pose(_LEVEL_OFFSET, None, pose)
    assert np.array_equal(got, expected)


def test_zero_map_is_bit_identical_to_c_level_1():
    """The home-node identity property: a residual of 0 changes nothing at all.

    The home node is ≈ 0 by construction (the capture tool asserts it), so this
    is the behaviour at the pose the single-offset `level` routine measured.
    """
    tm = _zero_map()
    expected = levelling.correction_from_offset(*_LEVEL_OFFSET)
    for pose in (np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]),
                 np.array([-100.0, 120.0, 170.0, 0.1, -0.05, 0.0]),
                 np.array([9e3, -9e3, 170.0, 0.0, 0.0, 0.0])):
        assert np.array_equal(
            levelling.correction_for_pose(_LEVEL_OFFSET, tm, pose), expected)


def test_composition_is_additive_in_the_rotation_vector():
    """offset (0.010, −0.006) + residual (0.004, 0.002) ⇒ exactly (0.014, −0.004).

    The residual pair is placed at the (x=0, y=0) node so the query is exact and
    the assertion is about composition, not interpolation.
    """
    doc = _doc()
    doc['residual_rad']['tx'][1][1] = 0.004
    doc['residual_rad']['ty'][1][1] = 0.002
    tm = tilt_map.parse_tilt_map(doc)
    pose = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])

    got = levelling.correction_for_pose(_LEVEL_OFFSET, tm, pose)
    expected = levelling.correction_from_offset(0.010 + 0.004, -0.006 + 0.002)
    assert np.array_equal(got, expected)

    # And the applied tilt really is the sum, not one term or the other.
    applied = levelling.apply_gravity_correction(np.zeros(3), got)
    assert applied[0] == pytest.approx(-0.014, abs=1e-15)
    assert applied[1] == pytest.approx(0.004, abs=1e-15)


def _composition_delta(offset, residual) -> float:
    """‖rotvec(R_additive⁻¹ · R(map)·R(level))‖ — the exact composition error."""
    additive = levelling.correction_from_offset(offset[0] + residual[0],
                                                offset[1] + residual[1])
    two_rotations = (levelling.correction_from_offset(*residual)
                     @ levelling.correction_from_offset(*offset))
    return float(np.linalg.norm(
        rot_matrix_to_rotvec(additive.T @ two_rotations)))


def _ring(magnitude_deg: float, n: int = 72):
    """`n` tilt pairs of the given magnitude, sweeping the full azimuth.

    The error term is a cross product, so it is zero for parallel tilts and
    maximal for orthogonal ones — a test that fixes the *direction* measures
    whatever the author happened to pick. Sweeping the ring is what makes the
    worst case, rather than a lucky case, the thing being pinned.
    """
    m = math.radians(magnitude_deg)
    return [(m * math.cos(t), m * math.sin(t))
            for t in np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)]


def test_additive_composition_second_order_bound_over_the_regime():
    """Sweep the whole claimed regime, not one point inside it.

    C-LEVEL-2 sums the two tilts and passes them through the ONE existing
    Rodrigues; the exact alternative composes two matrices, `R(map) @ R(level)`.
    The difference is exactly `|offset × residual| / 2` — which means the bound
    depends on BOTH magnitudes and on the angle between them, so "< 1e-4 rad at
    sub-degree tilts" is false at its own boundary. An earlier draft of the
    contract said exactly that, and a single-point test at (0.010, −0.006) +
    (0.004, 0.002) — 2.2e-5 rad, 7× inside the bound — was what let it pass.

    Pinned here (probe /tmp/probe_tilt_bound.py, run 2026-08-02, confirmed
    against this same `rot_matrix_to_rotvec` ground truth):
      * measured envelope 0.78185° × 0.604°  → 7.1925e-05 rad  (holds)
      * 1° × 1°, the corner of "sub-degree"  → 1.5231e-04 rad  (does NOT hold)
    """
    measured_worst = max(_composition_delta(off, res)
                         for off in _ring(0.78185) for res in _ring(0.604))
    assert measured_worst == pytest.approx(7.1925e-05, rel=1e-3)
    assert measured_worst < 1e-4, (
        'the bound must hold across the whole MEASURED envelope — this is the '
        'claim the contract actually relies on')

    sub_degree_worst = max(_composition_delta(off, res)
                           for off in _ring(1.0) for res in _ring(1.0))
    assert sub_degree_worst == pytest.approx(1.5231e-04, rel=1e-3), (
        'the sub-degree worst case is 1.52e-4 rad, NOT the < 1e-4 an earlier '
        'draft of the contract claimed — if this moves, fix the regime table '
        'in levelling_frame.md, do not widen the tolerance')
    assert sub_degree_worst > 1e-4, (
        'if this ever holds, the closed form changed and the contract table '
        'is now wrong in the other direction')


def test_composition_error_matches_the_closed_form_cross_product():
    """The contract states the term as `|offset × residual| / 2`. Pin the formula,
    not just its value: that is what lets a future reader re-derive the table."""
    for offset, residual in ((_LEVEL_OFFSET, (0.004, 0.002)),
                             ((0.0136, 0.0), (0.0, 0.01054)),
                             ((0.02, -0.01), (-0.003, 0.007))):
        a = np.array([-offset[0], -offset[1], 0.0])
        b = np.array([-residual[0], -residual[1], 0.0])
        closed_form = float(np.linalg.norm(np.cross(a, b)) / 2.0)
        assert _composition_delta(offset, residual) == pytest.approx(
            closed_form, rel=1e-3)


def test_composition_error_at_the_validation_ceiling_is_documented_as_large():
    """`MAX_ABS_RESIDUAL_RAD` alone does NOT keep the additive form under 1e-4.

    A future reader must not infer "it passed validation, therefore the additive
    composition is free". At the 0.05 rad ceiling against a 0.05 rad offset the
    error is 1.25e-3 rad — 12.5× the figure the contract's headline number
    suggests, which is why the contract states the regime rather than a bound.
    """
    assert _composition_delta((0.05, 0.0), (0.0, 0.05)) == pytest.approx(
        1.2498e-03, rel=1e-3)
    assert _composition_delta((0.0136, 0.0), (0.0, 0.05)) == pytest.approx(
        3.3997e-04, rel=1e-3)


def test_additive_and_two_rotation_forms_are_genuinely_different_matrices():
    """The bound is not vacuous — a future 'simplification' to matrix
    composition would be a real behavioural change, not a refactor."""
    doc = _doc()
    doc['residual_rad']['tx'][1][1] = 0.004
    doc['residual_rad']['ty'][1][1] = 0.002
    tm = tilt_map.parse_tilt_map(doc)
    pose = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])

    additive = levelling.correction_for_pose(_LEVEL_OFFSET, tm, pose)
    two_rotations = (levelling.correction_from_offset(0.004, 0.002)
                     @ levelling.correction_from_offset(*_LEVEL_OFFSET))
    assert not np.allclose(additive, two_rotations, atol=1e-12)
    assert _composition_delta(_LEVEL_OFFSET, (0.004, 0.002)) == pytest.approx(
        2.2000e-05, rel=1e-3)


def test_lookup_keys_on_x_y_only_and_ignores_orientation():
    """The map is captured at the level orientation and applied additively at
    every commanded orientation — the rotation components never key the lookup."""
    tm = _map()
    flat = np.array([-25.0, 80.0, 170.0, 0.0, 0.0, 0.0])
    tilted = np.array([-25.0, 80.0, 190.0, 0.10, -0.05, 0.03])
    assert np.array_equal(
        levelling.correction_for_pose(_LEVEL_OFFSET, tm, flat),
        levelling.correction_for_pose(_LEVEL_OFFSET, tm, tilted))


def test_correction_for_pose_accepts_a_list_pose_and_offset():
    tm = _map()
    got = levelling.correction_for_pose([0.010, -0.006], tm,
                                        [-25.0, 80.0, 170.0, 0.0, 0.0, 0.0])
    assert got.shape == (3, 3)
    assert np.allclose(got @ got.T, np.eye(3), atol=1e-12)


def test_correction_for_pose_uses_the_clamped_residual_outside_the_hull():
    """A pose beyond the calibrated grid gets the hull residual, never NaN and
    never an extrapolation."""
    tm = _map()
    outside = np.array([9999.0, 9999.0, 170.0, 0.0, 0.0, 0.0])
    corner = np.array([50.0, 120.0, 170.0, 0.0, 0.0, 0.0])
    assert np.array_equal(
        levelling.correction_for_pose(_LEVEL_OFFSET, tm, outside),
        levelling.correction_for_pose(_LEVEL_OFFSET, tm, corner))
    assert np.all(np.isfinite(
        levelling.correction_for_pose(_LEVEL_OFFSET, tm, outside)))


def test_correction_for_pose_matches_a_hand_built_rotation():
    """End-to-end against Rodrigues built directly from the summed rotvec."""
    tm = _map()
    pose = np.array([-25.0, 80.0, 170.0, 0.0, 0.0, 0.0])
    res_x, res_y = tilt_map.lookup(tm, -25.0, 80.0)
    expected = rotvec_to_rot_matrix(np.array([
        -(_LEVEL_OFFSET[0] + res_x), -(_LEVEL_OFFSET[1] + res_y), 0.0]))
    assert np.allclose(
        levelling.correction_for_pose(_LEVEL_OFFSET, tm, pose),
        expected, atol=1e-15)
