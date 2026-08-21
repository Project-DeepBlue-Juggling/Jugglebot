"""The layer-3 critical-point ILC artifact — ``jugglebot/motion/toss_ilc.py``.

``plans/active/critical-point-ilc.md`` **Phase 2**. This file covers the PURE
half: the schema, the all-or-nothing validation, the provenance-dormancy verdict,
the key quantisation, the exact-zero miss, and the constants this module shares
with the offline fit. The NODE half — byte-identical OFF, the single apply point,
the clamp REFUSAL, the record fields — is ``tests/ros/test_toss_ilc_node.py``.

THE CROSS-CHECK THIS FILE EXISTS FOR
------------------------------------
``tests/hardware/ilc_fit_lib.py`` (the offline fit) and this module (the online
apply) must agree, number for number, about the command channels, their
authorities and the key quantisation. They cannot share an import — a test-tree
module may not be a production dependency, and ``toss_ilc`` may not import
``toss_cal`` either (``test_toss_cal.py::test_nothing_else_imports_the_loader``
pins ``reload_coordinator_node`` as the aim map's single owner). So the constants
are stated twice and **pinned equal here**, which is the repo's standing answer
to "no silent second copies": the copy is allowed, the drift is not.

The failure this closes is quiet and total: a fit that writes cells on a 150 mm
grid and a loader that looks them up on a 100 mm grid produces an artifact that
loads clean, reports applied, and commands nothing on every goal forever.
"""

from __future__ import annotations

import ast
import copy
import math
import os
import sys

import numpy as np
import pytest
import yaml

_TESTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_HW_DIR = os.path.join(_TESTS, 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import ilc_fit_lib as fitlib                                       # noqa: E402

import jugglebot.hardware_config as hw                             # noqa: E402
from jugglebot import toss_record, toss_trim                       # noqa: E402
from jugglebot.motion import toss_cal, toss_ilc                    # noqa: E402
from jugglebot.motion.toss_ilc import (                            # noqa: E402
    ILC_AIM_MAX_RAD,
    ILC_SPEED_AUTHORITY,
    ZERO_CORRECTION,
    IlcCorrection,
    TossIlcError,
    artifact_version,
    goal_key,
    load_toss_ilc,
    lookup,
    model_config_identity,
    parse_toss_ilc,
    resolve_toss_ilc_path,
    toss_ilc_candidates,
)

_PKG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'ros_ws', 'src', 'jugglebot', 'jugglebot')

_TILT_V = '2026-08-10-deadbeef'
_CAL_V = '2026-08-11-abcd1234'


# ── Fixtures ──────────────────────────────────────────────────────────────────

def _doc(cells=None, **over):
    """A minimal VALID schema-v1 artifact.

    Two cells, with **deliberately different, deliberately asymmetric** values in
    all three channels: a swapped ``aim_rx``/``aim_ry`` or a key-order transpose
    is invisible on a symmetric fixture, and both are mistakes this schema
    invites (the aim→landing Jacobian is a 90° rotation, so the axes are already
    easy to confuse).
    """
    if cells is None:
        cells = [
            {'key': [0.0, 150.0, 170.0, 0.9],
             'aim_rx': 0.0011, 'aim_ry': -0.0023, 'event_vel_trim': -0.1076},
            {'key': [-150.0, 0.0, 170.0, 0.9],
             'aim_rx': -0.0007, 'aim_ry': 0.0031, 'event_vel_trim': -0.096},
        ]
    doc = {
        'version': 1,
        'captured': {'date': '2026-08-13', 'tool': 'test'},
        'requires': {'tilt_map_version': _TILT_V,
                     'toss_cal_version': _CAL_V,
                     'estimator_version': toss_ilc.ESTIMATOR_VERSION,
                     'model_config_identity': model_config_identity()},
        'units': {'aim': 'rad', 'event_vel_trim': 'dimensionless'},
        'key': {'pose_cell_mm': 150.0, 'pose_z_cell_mm': 10.0,
                'flight_time_cell_s': 0.050},
        'cells': cells,
    }
    doc.update(over)
    return doc


def _live(**over):
    kw = {'tilt_map_version': _TILT_V, 'toss_cal_version': _CAL_V}
    kw.update(over)
    return kw


# ══════════════════════════════════════════════════════════════════════════════
# 1. The cross-check against the offline fit (see the module docstring)
# ══════════════════════════════════════════════════════════════════════════════

def test_the_command_channels_match_the_fits_first_three_columns():
    """Order is load-bearing: it indexes every ``u`` array and every column of
    ``F``. An artifact written in a different order puts a radian in a
    dimensionless slot, which is a ~10x wrong launch speed, not a rounding
    error."""
    assert toss_ilc.ILC_CHANNELS == fitlib.U_LABELS[:3]
    assert list(toss_ilc.ILC_CHANNELS) == ['aim_rx', 'aim_ry', 'event_vel_trim']


def test_the_fourth_fit_channel_is_the_one_this_module_refuses():
    """``release_timing_offset`` is REFUSED by name, not merely unimplemented.
    Phase 1's screen found its column of ``F`` structurally zero, so nothing
    measured it; the quantity a dispatch shift really moves belongs to
    ``toss_trim.release_latency_ms`` and to gate G-1."""
    assert fitlib.U_LABELS[3] == 'release_timing_offset'
    assert 'release_timing_offset' in toss_ilc.REFUSED_CHANNELS
    assert set(toss_ilc.REFUSED_CHANNELS) & set(toss_ilc.ILC_CHANNELS) == set()


def test_the_speed_authority_is_the_fits_own_and_not_the_session_trims():
    """±0.15, the owner's Gate-1 decision of 2026-08-13 — and
    ``toss_trim.SPEED_AUTHORITY`` is deliberately NOT changed to match. That
    constant bounds a different loop with a different update law, measurand and
    operator model; widening it here would silently widen that one."""
    assert ILC_SPEED_AUTHORITY == fitlib.ILC_SPEED_AUTHORITY == 0.15
    assert toss_trim.SPEED_AUTHORITY == 0.10
    assert ILC_SPEED_AUTHORITY > toss_trim.SPEED_AUTHORITY


def test_the_aim_authority_is_the_D7_total_and_the_fit_agrees():
    """The per-cell aim bound is ``toss_cal.TOTAL_MAX_RAD`` restated (this module
    may not import the aim map's loader — see the module docstring) and the fit
    imports the same number as its ``AUTHORITY[0..1]``."""
    assert ILC_AIM_MAX_RAD == toss_cal.TOTAL_MAX_RAD == math.radians(1.0)
    assert float(fitlib.AUTHORITY[0]) == ILC_AIM_MAX_RAD
    assert float(fitlib.AUTHORITY[1]) == ILC_AIM_MAX_RAD


def test_the_key_quantisation_matches_the_fit():
    """Gate-1's approved cells: pose xy on the aim map's own 150 mm grid nodes,
    z at 10 mm (2x the miner's plane tolerance), flight time at 50 ms (inside the
    ±28 ms gain-vs-offset discrimination bound)."""
    assert toss_ilc.POSE_CELL_MM == fitlib.POSE_CELL_MM == 150.0
    assert toss_ilc.POSE_Z_CELL_MM == fitlib.POSE_Z_CELL_MM == 10.0
    assert toss_ilc.FLIGHT_TIME_CELL_S == fitlib.FLIGHT_TIME_CELL_S == 0.050


@pytest.mark.parametrize('x', [-380.0, -150.0, -76.0, -74.0, 0.0, 74.0, 76.0,
                               150.0, 223.0])
@pytest.mark.parametrize('t', [0.55, 0.7749, 0.7751, 0.9032314457914598, 1.10])
def test_the_key_arithmetic_is_bit_identical_to_the_fits(x, t):
    """The failure this closes is total and silent: a cell the fit wrote under
    one rounding rule and this loader looks up under another is a permanent MISS
    — the artifact loads, reports applied, and commands nothing.

    Swept rather than spot-checked, and deliberately across the half-cell
    boundaries (74/76 mm about the 0 node, 0.7749/0.7751 s about the 0.775 s cell
    edge) where a floor-vs-round or a half-to-even disagreement would first show.
    ``ilc_fit_lib.goal_key`` takes a mined record, so the comparison goes through
    a row it can recover a goal from.
    """
    y, z = 150.0, 170.0
    plane = z + float(hw.GEOM_INITIAL_HEIGHT_MM) + float(hw.HAND_CATCH_OFFSET_MM)
    row = {'cmd_flight_time_s': t,
           'land_xy_global_mm': [x + 7.0, y - 3.0],
           'land_err_mm': [7.0, -3.0],
           'land_plane_mm': plane}
    assert fitlib.goal_key(row) == goal_key(x, y, z, t)


# ══════════════════════════════════════════════════════════════════════════════
# 2. Parse / validate — all-or-nothing and loud
# ══════════════════════════════════════════════════════════════════════════════

def test_a_valid_document_parses_with_both_cells():
    ilc = parse_toss_ilc(_doc())
    assert ilc.n_cells == 2
    got = ilc.cells[(0.0, 150.0, 170.0, 0.9)]
    assert got == IlcCorrection(0.0011, -0.0023, -0.1076)
    # The asymmetric fixture pays off here: a transposed key or a swapped aim
    # pair would return the OTHER cell's values, and both are non-zero.
    other = ilc.cells[(-150.0, 0.0, 170.0, 0.9)]
    assert other.aim_rx == -0.0007 and other.aim_ry == 0.0031


@pytest.mark.parametrize('mutate,fragment', [
    (lambda d: d.pop('version'), 'version'),
    (lambda d: d.update(version=2), 'schema version'),
    (lambda d: d.update(version=True), 'schema version'),
    (lambda d: d.pop('units'), 'units'),
    (lambda d: d['units'].update(aim='mm'), 'units.aim'),
    (lambda d: d['units'].update(event_vel_trim='percent'), 'event_vel_trim'),
    (lambda d: d.pop('key'), 'key'),
    (lambda d: d['key'].update(pose_cell_mm=100.0), 'key quantisation'),
    (lambda d: d['key'].update(pose_z_cell_mm=150.0), 'key quantisation'),
    (lambda d: d['key'].update(flight_time_cell_s=0.1), 'key quantisation'),
    (lambda d: d.pop('requires'), 'requires'),
    (lambda d: d['requires'].pop('tilt_map_version'), 'tilt_map_version'),
    (lambda d: d['requires'].pop('toss_cal_version'), 'toss_cal_version'),
    (lambda d: d['requires'].pop('estimator_version'), 'estimator_version'),
    (lambda d: d['requires'].pop('model_config_identity'), 'model_config'),
    (lambda d: d['requires'].update(estimator_version=''), 'may not be empty'),
    (lambda d: d['requires'].update(model_config_identity=''), 'may not be empty'),
    (lambda d: d.pop('cells'), 'cells'),
    (lambda d: d.update(cells=[]), 'empty'),
    (lambda d: d.update(cells={}), 'must be a list'),
    (lambda d: d['cells'][0].pop('aim_rx'), 'aim_rx'),
    (lambda d: d['cells'][0].pop('key'), 'key'),
    (lambda d: d['cells'][0].update(key=[0.0, 150.0, 170.0]), 'key must be'),
    (lambda d: d['cells'][0].update(aim_rx=float('nan')), 'not finite'),
    (lambda d: d['cells'][0].update(aim_rx='0.001'), 'must be a number'),
    (lambda d: d['cells'][0].update(event_vel_trim=True), 'must be a number'),
    (lambda d: d['cells'][0].update(release_timing_offset=0.0), 'REFUSED'),
    (lambda d: d['cells'][0].update(catch_pose_dx=1.0), 'unknown key'),
    (lambda d: d['cells'][0].update(key=[10.0, 150.0, 170.0, 0.9]),
     'not a cell of this artifact'),
    (lambda d: d['cells'][1].update(key=[0.0, 150.0, 170.0, 0.9]), 'repeats'),
])
def test_every_malformed_document_is_refused_by_name(mutate, fragment):
    """All-or-nothing: there is no partial load, no per-cell repair and no silent
    zero-fill. A half-trusted correction is indistinguishable at the machine from
    a correct one until a ball misses."""
    doc = _doc()
    mutate(doc)
    with pytest.raises(TossIlcError) as excinfo:
        parse_toss_ilc(doc)
    assert fragment in str(excinfo.value), str(excinfo.value)


def test_an_off_grid_key_is_refused_where_it_is_written():
    """A stored key that is not a cell of the document's own quantisation could
    never be hit by a lookup — the artifact would load clean, report applied and
    command nothing. Refused at parse, where the fix is cheap."""
    doc = _doc(cells=[{'key': [40.0, 150.0, 170.0, 0.9], 'aim_rx': 0.0,
                       'aim_ry': 0.0, 'event_vel_trim': -0.05}])
    with pytest.raises(TossIlcError) as excinfo:
        parse_toss_ilc(doc)
    assert 'quantises onto' in str(excinfo.value)
    assert '0.0' in str(excinfo.value)


def test_the_aim_authority_is_enforced_on_the_MAGNITUDE_not_per_axis():
    """A per-axis box would admit ``hypot`` = 1.414x the bound on the diagonal —
    41 % more authority than every downstream argument (the C-LEVEL-2 composition
    regime, the cup swing, the 55 mm landing shift) is sized on."""
    edge = ILC_AIM_MAX_RAD / math.sqrt(2.0)
    ok = _doc(cells=[{'key': [0.0, 0.0, 170.0, 0.9],
                      'aim_rx': edge * 0.999, 'aim_ry': edge * 0.999,
                      'event_vel_trim': 0.0}])
    assert parse_toss_ilc(ok).n_cells == 1
    bad = _doc(cells=[{'key': [0.0, 0.0, 170.0, 0.9],
                       'aim_rx': edge * 1.01, 'aim_ry': edge * 1.01,
                       'event_vel_trim': 0.0}])
    with pytest.raises(TossIlcError) as excinfo:
        parse_toss_ilc(bad)
    assert 'authority' in str(excinfo.value)
    # Each axis on its own is well INSIDE the bound — which is exactly the case a
    # per-axis check would wave through.
    assert edge * 1.01 < ILC_AIM_MAX_RAD


@pytest.mark.parametrize('dv,ok', [
    (-0.15, True), (0.15, True), (-0.1076, True), (0.0, True),
    (-0.1501, False), (0.16, False), (-1.0, False),
])
def test_the_event_vel_trim_is_bounded_by_the_ilc_speed_authority(dv, ok):
    """REFUSED past ±0.15, never clamped into range. A silently clamped trim is
    not the correction that was solved for, and widening a safety bound is an
    operator decision — which is precisely how ±0.15 came to exist (Gate 1,
    2026-08-13)."""
    doc = _doc(cells=[{'key': [0.0, 0.0, 170.0, 0.9], 'aim_rx': 0.0,
                       'aim_ry': 0.0, 'event_vel_trim': dv}])
    if ok:
        assert parse_toss_ilc(doc).cells[
            (0.0, 0.0, 170.0, 0.9)].event_vel_trim == dv
    else:
        with pytest.raises(TossIlcError) as excinfo:
            parse_toss_ilc(doc)
        assert 'speed authority' in str(excinfo.value)


def test_a_single_bad_cell_loads_no_artifact_at_all():
    """The all-or-nothing rule, stated as behaviour rather than as a docstring:
    the first cell is perfectly good and it is still not loaded."""
    doc = _doc()
    doc['cells'][1]['event_vel_trim'] = -0.9
    with pytest.raises(TossIlcError):
        parse_toss_ilc(doc)


def test_load_reports_unreadable_empty_and_invalid_files_as_one_type(tmp_path):
    """Every failure is a ``TossIlcError`` because the caller's contract is
    identical in all of them: load no artifact, log loudly, throw exactly as the
    pre-Phase-2 machine did."""
    missing = tmp_path / 'nope.yaml'
    with pytest.raises(TossIlcError):
        load_toss_ilc(str(missing))
    empty = tmp_path / 'empty.yaml'
    empty.write_text('')
    with pytest.raises(TossIlcError) as excinfo:
        load_toss_ilc(str(empty))
    assert 'empty' in str(excinfo.value)
    broken = tmp_path / 'broken.yaml'
    broken.write_text('cells: [\n  - key: [\n')
    with pytest.raises(TossIlcError) as excinfo:
        load_toss_ilc(str(broken))
    assert 'not valid YAML' in str(excinfo.value)


def test_a_trim_proposal_document_is_structurally_refused(tmp_path):
    """The same guard ``test_toss_trim_node`` puts on the aim map, one layer up:
    an operator who ``cp``s some other YAML into ``config/toss_ilc.yaml`` must
    get a loud refusal, not an artifact that means something else."""
    doc = {'schema': toss_trim.TRIM_SCHEMA, 'trim': {'c_rad': [0.001, 0.0]}}
    path = tmp_path / 'toss_ilc.yaml'
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(TossIlcError):
        load_toss_ilc(str(path))


# ══════════════════════════════════════════════════════════════════════════════
# 3. Provenance dormancy (design constraint 5)
# ══════════════════════════════════════════════════════════════════════════════

def test_a_matching_artifact_has_no_mismatch():
    assert parse_toss_ilc(_doc()).provenance_mismatch(**_live()) is None


@pytest.mark.parametrize('live_over,fragment', [
    ({'tilt_map_version': '2026-01-01-00000000'}, 'tilt_map_version'),
    ({'tilt_map_version': ''}, 'UNKNOWN'),
    ({'toss_cal_version': '2020-01-01-ffffffff'}, 'toss_cal_version'),
    ({'toss_cal_version': ''}, 'APPLIED aim map is NONE'),
])
def test_a_live_layer_mismatch_makes_the_artifact_dormant(live_over, fragment):
    """Loaded, valid, and DOING NOTHING. An ILC residual fitted under layer 0/1 A
    double-counts B's delta, and an UNKNOWN live version is a mismatch
    deliberately — "I cannot verify what is underneath me" is not "the right
    thing is underneath me"."""
    reason = parse_toss_ilc(_doc()).provenance_mismatch(**_live(**live_over))
    assert reason and fragment in reason


def test_an_artifact_fitted_with_no_aim_map_is_dormant_under_one():
    """The other direction of the same gate. ``requires.toss_cal_version = ''``
    declares "fitted with NO aim map applied"; running it on a machine that IS
    applying one would add a learned residual on top of a correction that was not
    there when it was learnt."""
    doc = _doc()
    doc['requires']['toss_cal_version'] = ''
    ilc = parse_toss_ilc(doc)
    assert ilc.provenance_mismatch(**_live(toss_cal_version='')) is None
    reason = ilc.provenance_mismatch(**_live())
    assert reason and 'fitted with NO aim map' in reason


def test_the_estimator_version_gates_the_meaning_of_the_fitted_numbers():
    doc = _doc()
    doc['requires']['estimator_version'] = 'ilc-critical-point/0'
    reason = parse_toss_ilc(doc).provenance_mismatch(**_live())
    assert reason and 'estimator_version' in reason


def test_the_model_config_identity_gates_the_sensitivity_F():
    """``F`` is computed through the production geometry constants; under a
    different geometry the same ``du`` points somewhere else. The identity is
    re-derived live, so a config edit makes a shipped artifact dormant with no
    further plumbing."""
    doc = _doc()
    doc['requires']['model_config_identity'] = 'deadbeef0000'
    reason = parse_toss_ilc(doc).provenance_mismatch(**_live())
    assert reason and 'model_config_identity' in reason
    assert 'GEOM_INITIAL_HEIGHT_MM' in reason


def test_the_model_identity_moves_when_a_forward_chain_constant_moves(
        monkeypatch):
    """Not a hash of nothing: perturbing ONE of the named constants changes it,
    and perturbing a constant the forward chain does not read does not."""
    before = model_config_identity()
    monkeypatch.setattr(hw, 'HAND_CATCH_OFFSET_MM',
                        float(hw.HAND_CATCH_OFFSET_MM) + 1.0)
    assert model_config_identity() != before
    monkeypatch.undo()
    assert model_config_identity() == before


def test_the_identity_covers_every_constant_the_forward_chain_reads(monkeypatch):
    for name in toss_ilc.MODEL_CONFIG_KEYS:
        before = model_config_identity()
        monkeypatch.setattr(hw, name, float(getattr(hw, name)) + 1.0)
        assert model_config_identity() != before, name
        monkeypatch.undo()


# ══════════════════════════════════════════════════════════════════════════════
# 4. Lookup — the exact-zero miss
# ══════════════════════════════════════════════════════════════════════════════

def test_a_hit_returns_the_stored_correction_and_says_so():
    ilc = parse_toss_ilc(_doc())
    got = lookup(ilc, 0.0, 150.0, 170.0, 0.9)
    assert got.hit is True
    assert got.key == (0.0, 150.0, 170.0, 0.9)
    assert got.correction == IlcCorrection(0.0011, -0.0023, -0.1076)


@pytest.mark.parametrize('query', [
    (0.0, 150.0, 170.0, 0.60),          # right pose, wrong flight time
    (0.0, 150.0, 220.0, 0.9),           # right xy + T, wrong plane
    (300.0, 150.0, 170.0, 0.9),         # a pose the fit never visited
    (150.0, 150.0, 170.0, 0.9),
])
def test_a_key_MISS_is_exactly_zero_and_never_an_interpolation(query):
    """No hull clamp, no nearest neighbour, no bilinear blend — unlike the aim
    map, deliberately. The value is a whole command vector whose channels have
    different units and different authorities, so "half way between two
    corrections" is not a correction anybody solved for; and the corpora are
    sparse, so a hull clamp would spread one cell's correction over the whole
    workspace.

    ``exactly`` means identity with the module's zero, not ``approx``: the node's
    byte-identical-OFF path keys on ``== (0.0, 0.0)`` and ``!= 0.0``.
    """
    ilc = parse_toss_ilc(_doc())
    got = lookup(ilc, *query)
    assert got.hit is False
    assert got.correction is ZERO_CORRECTION
    assert got.correction.aim_rad == (0.0, 0.0)
    assert got.correction.event_vel_trim == 0.0


def test_a_query_inside_the_cell_still_hits_it():
    """The cell is a KEY, not a point: a goal 40 mm off the node and 12 ms off the
    flight-time centre is the same goal cell, which is what makes the artifact
    usable at poses an operator typed by hand."""
    ilc = parse_toss_ilc(_doc())
    got = lookup(ilc, 40.0, 138.0, 173.0, 0.912)
    assert got.hit is True
    assert got.correction.event_vel_trim == -0.1076


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_a_non_finite_query_raises_rather_than_returning_nan(bad):
    """A NaN correction would become a NaN virtual target, a NaN launch velocity
    and a NaN ``event_vel`` on the wire — surfacing only as a downstream rejection
    after a goal has already claimed the platform."""
    ilc = parse_toss_ilc(_doc())
    with pytest.raises(TossIlcError):
        lookup(ilc, bad, 150.0, 170.0, 0.9)
    with pytest.raises(TossIlcError):
        lookup(ilc, 0.0, 150.0, 170.0, bad)


def test_the_parsed_cells_cannot_be_mutated_behind_a_running_goal():
    """The table is a shared per-process single reference every toss goal reads.
    ``parse_toss_ilc`` copies, so a caller that keeps its own dict (the fit tool
    self-validating before it writes) cannot re-command every subsequent throw by
    editing it afterwards."""
    cells = {(0.0, 0.0, 170.0, 0.9): IlcCorrection(0.0, 0.0, -0.05)}
    ilc = toss_ilc.TossIlc(cells=cells, version='v', requires_tilt_map_version='',
                           requires_toss_cal_version='',
                           requires_estimator_version=toss_ilc.ESTIMATOR_VERSION,
                           requires_model_config_identity='x')
    cells[(150.0, 0.0, 170.0, 0.9)] = IlcCorrection(0.0, 0.0, -0.9)
    assert ilc.n_cells == 1


# ══════════════════════════════════════════════════════════════════════════════
# 5. The version string
# ══════════════════════════════════════════════════════════════════════════════

def test_two_documents_with_identical_APPLIED_numbers_share_a_version():
    """The rule, restated from C-TOSS-CAL-1: two files whose applied numbers are
    identical report the same version; an edit to a single applied number changes
    it. Provenance blocks are excluded, so re-stamping ``captured`` or fixing a
    ``requires`` typo does not churn the version and send an operator hunting a
    difference that does not exist."""
    a = _doc()
    b = _doc()
    b['captured']['tool'] = 'something else'
    b['requires']['tilt_map_version'] = 'a-different-tilt-map'
    assert artifact_version(a) == artifact_version(b)


def test_one_edited_applied_number_changes_the_version():
    a = _doc()
    b = copy.deepcopy(a)
    b['cells'][0]['event_vel_trim'] = -0.1077
    assert artifact_version(a) != artifact_version(b)


def test_the_units_and_the_key_block_are_inside_the_hash():
    """Both change what the machine DOES without changing a single cell value —
    the units decide what the numbers mean and the key block decides which goal
    reads which cell. Outside the hash, an edit to either would be invisible."""
    a = _doc()
    for mutate in (lambda d: d['units'].update(aim='mm'),
                   lambda d: d['key'].update(pose_cell_mm=100.0)):
        b = copy.deepcopy(a)
        mutate(b)
        assert artifact_version(a) != artifact_version(b)


def test_the_version_ignores_cell_ORDER_but_not_cell_CONTENT():
    """Two writers that emit the same cells in different orders describe one
    artifact. Sorting inside the hash says so; without it a re-fit that happened
    to iterate a dict differently would look like a new calibration."""
    a = _doc()
    b = copy.deepcopy(a)
    b['cells'].reverse()
    assert artifact_version(a) == artifact_version(b)


def test_the_version_survives_an_int_vs_float_serializer_difference():
    """``170`` and ``170.0`` are one artifact written by two YAML writers."""
    a = _doc()
    b = copy.deepcopy(a)
    b['cells'][0]['key'] = [0, 150, 170, 0.9]
    assert artifact_version(a) == artifact_version(b)


def test_a_malformed_document_still_hashes_deterministically():
    assert artifact_version({}) == artifact_version({})
    assert artifact_version('not a document') .startswith('undated-')


# ══════════════════════════════════════════════════════════════════════════════
# 6. Path resolution
# ══════════════════════════════════════════════════════════════════════════════

def test_the_env_override_is_the_only_candidate_when_set(tmp_path):
    """Falling through would apply A DIFFERENT CORRECTION THAN THE OPERATOR NAMED
    while reporting a plausible version. It is also the Phase-3 A/B's baseline
    arm: naming a path that does not exist must degrade to zero correction, not
    to whatever is in ``config/``."""
    named = str(tmp_path / 'named.yaml')
    assert toss_ilc_candidates({toss_ilc.ILC_ENV: named}) == (named,)
    assert resolve_toss_ilc_path({toss_ilc.ILC_ENV: named}) is None
    (tmp_path / 'named.yaml').write_text(yaml.safe_dump(_doc()))
    assert resolve_toss_ilc_path({toss_ilc.ILC_ENV: named}) == named


def test_the_source_tree_is_searched_before_the_ament_share():
    candidates = toss_ilc_candidates({})
    assert candidates, 'the repo source tree must always be a candidate here'
    assert candidates[0].endswith(os.path.join('config', 'toss_ilc.yaml'))
    assert os.path.isdir(os.path.join(
        os.path.dirname(os.path.dirname(candidates[0])), 'ros_ws'))


def test_absence_is_silent_not_an_error():
    assert resolve_toss_ilc_path({toss_ilc.ILC_ENV: '/nonexistent/ilc.yaml'}) \
        is None


def test_the_repo_root_is_found_by_marker_search_not_a_file_walk():
    """A fixed ``__file__`` walk is correct only from the source checkout: colcon
    COPIES the package into ``install/.../site-packages/``, where the same number
    of levels lands on a directory nothing ever creates — and the source-tree
    candidate would then be unable to exist in production."""
    with open(os.path.join(_PKG_DIR, 'motion', 'toss_ilc.py'),
              encoding='utf-8') as fh:
        source = fh.read()
    assert 'find_repo_root(__file__)' in source


def test_the_repo_ships_no_ilc_artifact_yet():
    """Phase 2 lands the plumbing APPLIED AT ZERO. A committed artifact here
    would mean an un-A/B'd learned correction shipped to hardware, and Phase 3 is
    the gate that decides whether it should be — on evidence, against an
    aim-map-only baseline."""
    repo = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    assert os.path.isdir(os.path.join(repo, 'ros_ws')), (
        'the repo root walk is wrong — this guard would pass vacuously')
    assert not os.path.exists(os.path.join(repo, 'config', 'toss_ilc.yaml')), (
        'config/toss_ilc.yaml exists — if Phase 3 has genuinely run and the '
        'operator chose to ship it, delete this test in the same commit that '
        'lands the artifact, and say so in the logbook')


# ══════════════════════════════════════════════════════════════════════════════
# 7. The tripwire, and the module's purity
# ══════════════════════════════════════════════════════════════════════════════

def test_shipped_config_has_the_feature_off():
    """THE TRIPWIRE. ``jugglebot_operational.toss_ilc_enabled`` ships FALSE.

    This is the ``dynamics.torque_ff_enabled`` pattern before its 2026-07-16
    arming session, and it is a CONFIG flag rather than a node parameter for one
    root cause: this arms a *learned* correction, so which build applied it has
    to be answerable from git alone, months later, when a corpus fitted under it
    is being read back. A ``ros2 param set`` puts that fact in a shell history
    nobody keeps.

    If this fails, someone ARMED the ILC via a commit. That is a legitimate
    thing to do — after the Phase-3 A/B — but it must be a deliberate, logged
    decision with the A/B evidence behind it, not a drive-by config edit. Turn
    this into ``test_shipped_config_has_the_feature_on`` in the same commit and
    write down which sitting decided it.
    """
    assert hw.JB_OP_TOSS_ILC_ENABLED is False, (
        'jugglebot_operational.toss_ilc_enabled ships false until the Phase-3 '
        'A/B says otherwise — arming it is a reviewed, logged config commit')


def test_the_loader_imports_no_ros():
    """Pure Python, consumed by a ROS node AND by tests/motion with no ROS
    mocking at all — ``ament_index`` is imported lazily inside the resolver."""
    with open(os.path.join(_PKG_DIR, 'motion', 'toss_ilc.py'),
              encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    top_level = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            top_level += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            top_level.append(node.module or '')
    banned = [m for m in top_level
              if m.split('.')[0] in ('rclpy', 'std_msgs', 'std_srvs',
                                     'geometry_msgs', 'jugglebot_interfaces',
                                     'ament_index_python')]
    assert banned == [], 'toss_ilc.py imports ROS at module scope: {}'.format(
        banned)


def test_the_loader_does_not_import_the_aim_maps_loader():
    """``test_toss_cal.py::test_nothing_else_imports_the_loader`` pins
    ``reload_coordinator_node`` as the aim map's SINGLE owner, and a second
    importer inside the package is a second owner. That is why
    :data:`ILC_AIM_MAX_RAD` is restated and pinned equal by test rather than
    imported — the same shape ``toss_trim`` already follows for its own bound."""
    with open(os.path.join(_PKG_DIR, 'motion', 'toss_ilc.py'),
              encoding='utf-8') as fh:
        tree = ast.parse(fh.read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [(node.module or '')] + [
                (node.module or '') + '.' + a.name for a in node.names]
        else:
            continue
        assert not any(n.endswith('toss_cal') or '.toss_cal' in n
                       for n in names), ast.dump(node)


# ══════════════════════════════════════════════════════════════════════════════
# 8. The record's two new fields
# ══════════════════════════════════════════════════════════════════════════════

def test_the_record_declares_the_applied_ilc_contribution():
    """Additive fields, origin 'D' — the node is the only witness of what it
    commanded. Purely additive, so ``toss_record.SCHEMA`` does NOT bump (the
    record's own § 3.7 rule); ``tests/motion/test_toss_record.py`` pins the whole
    list."""
    by_name = {f.name: f for f in toss_record.FIELDS}
    assert by_name['ilc_aim_rad'].origin == 'D'
    assert by_name['ilc_aim_rad'].kind == 'f2'
    assert by_name['ilc_vel_trim'].origin == 'D'
    assert by_name['ilc_vel_trim'].kind == 'f'
    assert by_name['ilc_aim_rad'].block == by_name['ilc_vel_trim'].block \
        == 'calibration'
    assert toss_record.SCHEMA == 'toss_record/1'
    blank = toss_record.blank_record()
    assert blank['ilc_aim_rad'] is None and blank['ilc_vel_trim'] is None


def test_the_new_fields_round_trip_through_the_wire_form():
    rec = toss_record.blank_record()
    rec.update({'toss_uid': 'u', 'action': 'toss', 'outcome': 'CAUGHT',
                'ilc_aim_rad': [0.0011, -0.0023], 'ilc_vel_trim': -0.1076})
    decoded = toss_record.decode(toss_record.encode(rec))
    assert decoded['ilc_aim_rad'] == [0.0011, -0.0023]
    assert decoded['ilc_vel_trim'] == -0.1076
    assert toss_record.validate(decoded) == ()


# ══════════════════════════════════════════════════════════════════════════════
# 9. The writer round trip (tests/hardware/ilc_fit.py --write-artifact)
# ══════════════════════════════════════════════════════════════════════════════

import ilc_fit                                                     # noqa: E402

_NO_NOISE = [0.0] * fitlib.N_E


def _synthetic_rows(*, dv=-0.08, n=12, cell_y=150.0):
    """A corpus the fit will admit, carrying a known plant speed error.

    Built through ``ilc_fit_lib.synthetic_corpus`` — the FORWARD production model
    — so the round trip exercises the real sign convention in both directions
    rather than restating one of them. Noiseless (``sigma_e`` all zero) because
    the assertion below is about the WRITER, not about the estimator: any scatter
    here would turn a schema test into a statistics test. The declaration-half
    provenance fields are stamped on afterwards, because the synthetic generator
    makes mined-only rows.
    """
    goal = fitlib.TossGoal(catch_pose_stow_mm=(0.0, cell_y, 170.0),
                           flight_time_s=fitlib.CORPUS_FLIGHT_TIME_S)
    rows = fitlib.synthetic_corpus(goal, u_plant=[0.0, 0.0, dv, 0.0], n=n,
                                   sigma_e=_NO_NOISE)
    for row in rows:
        row['tilt_map_version'] = _TILT_V
        row['toss_cal_version'] = _CAL_V
        row['toss_cal_applied'] = True
    return rows


def _args(tmp_path, extra=()):
    """Parse through the CLI's OWN parser, so this test cannot drift from the
    flags an operator actually types."""
    path = tmp_path / 'toss_ilc.yaml'
    argv = (['--write-artifact', str(path), '--allow-cross-partition']
            + list(extra))
    return path, ilc_fit.build_parser().parse_args(argv)


def test_the_writer_round_trips_through_the_PRODUCTION_loader(tmp_path):
    """THE Phase-2 writer gate: write → load through
    ``jugglebot.motion.toss_ilc`` → the applied numbers are the fitted ones.

    Not "a YAML file appeared": the file has to be one THIS BUILD's loader
    accepts and looks up at the goal the corpus was thrown at. A writer carrying
    its own idea of the schema passes a file-exists check and fails this one.
    """
    path, args = _args(tmp_path)
    rows = _synthetic_rows()
    assert ilc_fit.write_artifact(rows, args) == 0

    loaded = load_toss_ilc(str(path))
    assert loaded.provenance_mismatch(**_live()) is None
    goal = fitlib.goal_of(rows[0])
    got = lookup(loaded, goal.catch_pose_stow_mm[0], goal.catch_pose_stow_mm[1],
                 goal.catch_pose_stow_mm[2], goal.flight_time_s)
    assert got.hit is True
    # The plant throws SLOW by |dv| here (u_plant = −0.08), so the correction
    # must be POSITIVE. A sign flip does not merely fail this assertion, it fails
    # it by doubling the error — which is why the sign is asserted separately
    # from the magnitude.
    assert got.correction.event_vel_trim > 0.0
    # And it is exactly ONE TRUST-REGION STEP, not the whole 0.08: the loop is
    # iterative by construction (Phase 3's criterion is k <= 3 iterations) and
    # tau bounds what one hardware sitting may change. A writer that emitted the
    # converged correction here would silently discard the trust region — the one
    # thing standing between a wrong F and a large wrong command.
    assert got.correction.event_vel_trim == pytest.approx(
        float(fitlib.TAU0[2]), rel=1e-9)
    assert got.correction.aim_rad == (0.0, 0.0)


def test_a_second_iteration_ACCUMULATES_onto_the_first(tmp_path):
    """``--from-artifact``: the artifact stores the accumulated ``u``, not the
    last step. Without this seam every write would restart from zero and the
    trust region would cap the correction at ``tau`` forever — the loop would
    look convergent and never converge.

    Iteration 2 is fed the SAME residual (the synthetic plant does not know it
    was corrected), so the accumulated value must be exactly ``2*tau``. On
    hardware the residual shrinks instead, which is what makes ``k <= 3``
    reachable against the measured −0.1076 requirement.
    """
    path, args = _args(tmp_path)
    rows = _synthetic_rows()
    assert ilc_fit.write_artifact(rows, args) == 0
    first = load_toss_ilc(str(path))

    second_path, second_args = _args(tmp_path / 'two')
    (tmp_path / 'two').mkdir()
    second_args.from_artifact = str(path)
    assert ilc_fit.write_artifact(rows, second_args) == 0
    second = load_toss_ilc(str(second_path))

    key = (0.0, 150.0, 170.0, 0.9)
    assert first.cells[key].event_vel_trim == pytest.approx(
        float(fitlib.TAU0[2]), rel=1e-9)
    assert second.cells[key].event_vel_trim == pytest.approx(
        2.0 * float(fitlib.TAU0[2]), rel=1e-9)


def test_the_accumulated_command_is_re_validated_by_the_REAL_gates(tmp_path):
    """Design constraint 3, at the seam that emits. ``propose_step`` validated
    ``0 + du``; the ACCUMULATED vector is a different command and gets its own
    ``admit_command`` pass. Seeded here from an artifact already at the ±0.15
    authority, so one more step must be refused rather than written."""
    seed_path = tmp_path / 'seed.yaml'
    seed_path.write_text(yaml.safe_dump(_doc(cells=[{
        'key': [0.0, 150.0, 170.0, 0.9], 'aim_rx': 0.0, 'aim_ry': 0.0,
        'event_vel_trim': ILC_SPEED_AUTHORITY}])))
    path, args = _args(tmp_path)
    args.from_artifact = str(seed_path)
    with pytest.raises(ilc_fit.WriteRefused) as excinfo:
        ilc_fit.write_artifact(_synthetic_rows(), args)
    assert 'ACCUMULATED' in str(excinfo.value)
    assert not path.exists()


def test_the_writer_refuses_a_saturated_fit(tmp_path):
    """A correction the residual asks for that exceeds an authority is an
    OPERATOR decision — exactly the one Gate 1 had to take for
    ``event_vel_trim`` — not something a writer may quietly saturate into a
    file. ``dv = 0.30`` puts the required trim at twice the ±0.15 authority."""
    path, args = _args(tmp_path)
    with pytest.raises(ilc_fit.WriteRefused) as excinfo:
        ilc_fit.write_artifact(_synthetic_rows(dv=0.30), args)
    message = str(excinfo.value)
    assert 'authority' in message or 'trust region' in message, message
    assert not path.exists(), 'a refused write must leave NOTHING on disk'


def test_the_writer_refuses_unprovable_provenance(tmp_path):
    """A mined-only corpus (every bag predating ``/toss/record``) cannot say what
    aim map was applied. Stamping ``''`` there would declare "fitted with NO aim
    map" on evidence that says only "nobody wrote it down" — and the artifact's
    whole dormancy gate rests on that string, so the guess would be a gate that
    fails open forever."""
    path, args = _args(tmp_path)
    rows = _synthetic_rows()
    for row in rows:
        row['toss_cal_version'] = None
        row['toss_cal_applied'] = None
    with pytest.raises(ilc_fit.WriteRefused) as excinfo:
        ilc_fit.write_artifact(rows, args)
    assert 'toss_cal' in str(excinfo.value)
    assert not path.exists()


def test_an_operator_declaration_is_stamped_not_laundered(tmp_path):
    """The escape hatch exists, and using it is RECORDED: the artifact says its
    provenance was asserted rather than measured, which is the difference a later
    reader needs."""
    path, args = _args(tmp_path, extra=['--declare-toss-cal', 'NONE'])
    rows = _synthetic_rows()
    for row in rows:
        row['toss_cal_version'] = None
        row['toss_cal_applied'] = None
    assert ilc_fit.write_artifact(rows, args) == 0
    doc = yaml.safe_load(path.read_text())
    assert doc['requires']['toss_cal_version'] == ''
    assert doc['captured']['toss_cal_version_source'] == 'operator-declared'
    assert doc['captured']['tilt_map_version_source'] == 'corpus'
    # ... and the artifact it wrote is DORMANT on a machine applying an aim map,
    # which is the whole point of recording the declaration rather than a guess.
    loaded = load_toss_ilc(str(path))
    assert loaded.provenance_mismatch(**_live()) is not None
    assert loaded.provenance_mismatch(**_live(toss_cal_version='')) is None


def test_the_writer_refuses_a_document_it_could_not_load_back(tmp_path,
                                                              monkeypatch):
    """The round trip runs BEFORE a byte reaches the disk. Writing first and
    validating later would leave an unloadable artifact in ``config/`` on every
    failure — and the operator would find out at the next launch, not at the
    write."""
    path, args = _args(tmp_path)
    monkeypatch.setattr(toss_ilc, 'POSE_CELL_MM', 100.0)
    with pytest.raises(TossIlcError):
        ilc_fit.write_artifact(_synthetic_rows(), args)


# ══════════════════════════════════════════════════════════════════════════════
# 9a. The POOLED write gates each cell at its OWN goal
# ══════════════════════════════════════════════════════════════════════════════

def _two_cell_rows(*, dv=-0.08, n_modal=12, n_other=4, other_flight_s=1.30):
    """A corpus whose two goal cells DIFFER — same pose, different flight time.

    The modal cell is the corpus flight time; the second is deliberately outside
    the sequencer's ``[0.55, 1.10]`` s band, which is the cheapest of the several
    goal-dependent gates ``admit_command`` runs (the others being the workspace
    box, the tilt feasibility and the bridge's ``event_vel`` band). Any of them
    would do — the point under test is not which gate fires but WHICH GEOMETRY it
    is asked about.
    """
    rows = _synthetic_rows(dv=dv, n=n_modal)
    other = fitlib.TossGoal(catch_pose_stow_mm=(0.0, 150.0, 170.0),
                            flight_time_s=float(other_flight_s))
    extra = fitlib.synthetic_corpus(other, u_plant=[0.0, 0.0, dv, 0.0],
                                    n=n_other, sigma_e=_NO_NOISE)
    for i, row in enumerate(extra):
        row['toss_uid'] = 'syn-ilc-other-{}'.format(i)
        row['tilt_map_version'] = _TILT_V
        row['toss_cal_version'] = _CAL_V
        row['toss_cal_applied'] = True
    return rows + extra


def test_a_POOLED_write_gates_every_cell_at_its_OWN_goal_not_the_modal_one(
        tmp_path):
    """**The pooled write's admission is PER CELL even though its fit is not.**

    ``--pool`` computes one ``du`` and writes it to every cell the corpus
    visited. That is a deliberate, stamped statistical choice. It is NOT a
    licence to validate those cells against the modal cell's geometry: every gate
    ``ilc_fit_lib.admit_command`` runs is goal-dependent — the sequencer's
    flight-time band and workspace box, ``compute_release_state_tilted``'s tilt
    feasibility, and the bridge's ``event_vel`` acceptance band, which is the one
    that actually bites (the same trim is a different absolute launch speed at a
    different flight time). Gating cell B at cell A's geometry is a gate that can
    pass for a command the machine would reject at B.

    Driven by the *whole cell* being inadmissible rather than a hair-splitting
    one, because that makes the failure unambiguous: the second cell's goal is
    outside the sequencer band, so NOTHING may be written for it — and the modal
    cell's geometry says it is fine.
    """
    path, args = _args(tmp_path, extra=['--pool'])
    with pytest.raises(ilc_fit.WriteRefused) as excinfo:
        ilc_fit.write_artifact(_two_cell_rows(), args)
    message = str(excinfo.value)
    assert 'REJECTED_FLIGHT_TIME' in message, message
    # Named as the OFFENDING cell, not as "the pooled fit" — an operator has to
    # be able to see which goal is unflyable.
    assert '1.3' in message, message
    assert not path.exists(), 'a refused write must leave NOTHING on disk'


def test_the_pooled_and_per_cell_paths_agree_about_an_inadmissible_cell(
        tmp_path):
    """The two write paths must refuse the SAME corpus for the SAME reason.

    The per-cell path has always refused this cell — ``fit_corpus`` fits it at
    its own geometry, so ``propose_step``'s very first ``admit_command`` says no.
    A pooled path that accepted it would mean the ``--pool`` flag quietly widened
    the admission rule as well as the statistics, which is not what it is
    documented to do and not what an operator asking for one number across three
    poses is agreeing to.
    """
    rows = _two_cell_rows()
    _, pooled_args = _args(tmp_path / 'a', extra=['--pool'])
    (tmp_path / 'a').mkdir()
    _, per_cell_args = _args(tmp_path / 'b')
    (tmp_path / 'b').mkdir()

    with pytest.raises((ilc_fit.WriteRefused, fitlib.IlcFitError)) as pooled:
        ilc_fit.write_artifact(rows, pooled_args)
    with pytest.raises((ilc_fit.WriteRefused, fitlib.IlcFitError)) as per_cell:
        ilc_fit.write_artifact(rows, per_cell_args)
    assert 'REJECTED_FLIGHT_TIME' in str(pooled.value)
    assert 'REJECTED_FLIGHT_TIME' in str(per_cell.value)


def test_the_modal_cell_alone_still_writes_so_the_refusal_is_the_CELLS(tmp_path):
    """The control for the two tests above: the same pooled fit over the modal
    cell ALONE writes cleanly. Without this, a refusal that came from the pooling
    itself — or from the fit being bad — would look identical to a refusal that
    came from the second cell's geometry."""
    path, args = _args(tmp_path, extra=['--pool'])
    assert ilc_fit.write_artifact(_synthetic_rows(), args) == 0
    assert load_toss_ilc(str(path)).n_cells == 1


# ══════════════════════════════════════════════════════════════════════════════
# 9b. --from-artifact: the SEED is validated, not assumed
# ══════════════════════════════════════════════════════════════════════════════

def _seed(tmp_path, name='seed.yaml', *, cells=None, **requires_over):
    seed_path = tmp_path / name
    doc = _doc(cells=cells or [{'key': [0.0, 150.0, 170.0, 0.9],
                                'aim_rx': 0.0, 'aim_ry': 0.0,
                                'event_vel_trim': -0.05}])
    doc['requires'] = dict(doc['requires'], **requires_over)
    seed_path.write_text(yaml.safe_dump(doc))
    return seed_path


@pytest.mark.parametrize('requires_over,fragment', [
    ({'tilt_map_version': '2020-01-01-00000000'}, 'tilt_map_version'),
    ({'toss_cal_version': '2020-01-01-ffffffff'}, 'toss_cal_version'),
    ({'estimator_version': 'ilc-critical-point/0'}, 'estimator_version'),
    ({'model_config_identity': 'deadbeefcafe'}, 'model_config_identity'),
])
def test_a_seed_artifact_whose_provenance_does_not_match_is_REFUSED(
        tmp_path, requires_over, fragment):
    """**``--from-artifact`` may not accumulate onto an unchecked prior.**

    ``parse_toss_ilc`` proves the seed document is well-formed and inside the
    per-cell authorities. It does NOT prove the seed was fitted on the plant this
    write is fitting — nothing in the file's syntax could. And the write is
    ``u_prev + du``, so an unchecked prior imports its whole command vector into
    a document that is then stamped with THIS corpus's provenance: a correction
    learnt under aim map A, shipped under a header declaring aim map B.

    That is precisely the double-count ``requires`` exists to make dormant, and
    routing it through the writer is worse than defeating the gate at the robot —
    the robot's gate would have caught it, and after this laundering there is
    nothing left for it to catch. The estimator case is the same failure in the
    measurand: a residual fitted from the PRE-E-1 per-branch lateral velocities
    accumulated onto post-E-1 whole-arc ones.

    Enforced through the PRODUCTION comparison (``TossIlc.provenance_mismatch``),
    never a second copy of it: the writer and the machine must agree about what
    "matches" means, or an artifact refuses at the robot after it has shipped.
    """
    seed_path = _seed(tmp_path, **requires_over)
    path, args = _args(tmp_path)
    args.from_artifact = str(seed_path)
    with pytest.raises(ilc_fit.WriteRefused) as excinfo:
        ilc_fit.write_artifact(_synthetic_rows(), args)
    message = str(excinfo.value)
    assert fragment in message, message
    assert not path.exists(), 'a refused write must leave NOTHING on disk'


def test_a_MATCHING_seed_is_still_accumulated(tmp_path):
    """The control: the provenance gate must not be a blanket refusal of
    ``--from-artifact``. A seed whose four ``requires`` agree with what this
    write will declare accumulates exactly as before."""
    seed_path = _seed(tmp_path)
    path, args = _args(tmp_path)
    args.from_artifact = str(seed_path)
    assert ilc_fit.write_artifact(_synthetic_rows(), args) == 0
    got = load_toss_ilc(str(path)).cells[(0.0, 150.0, 170.0, 0.9)]
    assert got.event_vel_trim == pytest.approx(-0.05 + float(fitlib.TAU0[2]),
                                               rel=1e-9)


def test_a_SEED_the_real_gates_refuse_is_never_accumulated_onto(tmp_path,
                                                                monkeypatch):
    """**"The sum is admissible" is not "the seed was".**

    ``admit_command`` is not monotone in ``u``, so an inadmissible seed can add
    to an admissible sum — a prior past the authority plus a ``du`` pointing back
    inside it lands in bounds, while the machine would have refused the prior on
    every toss it ever flew. Design constraint 3 says every emitted command
    passes the exact gates, and the seed is one of the commands the emitted
    vector is made of.

    **Driven by a NARROWED authority, and that is the honest driver here.** On the
    shipped constants the parse bound (``toss_ilc.ILC_SPEED_AUTHORITY``) and the
    apply bound (``ilc_fit_lib.AUTHORITY``) are the same number, so a seed that
    parses is admissible at any admissible goal — a test that reached this path
    through an unmodified pair would be evidence those two had drifted apart. The
    reachable form is the one modelled here: the authority is TIGHTENED (Gate 1
    revisits it) after an artifact was written at the old wider bound, and the
    old file is then handed to ``--from-artifact``.
    """
    narrowed = np.array(fitlib.AUTHORITY, dtype=float)
    narrowed[2] = 0.10
    monkeypatch.setattr(fitlib, 'AUTHORITY', narrowed)

    seed_path = _seed(tmp_path, cells=[{'key': [0.0, 150.0, 170.0, 0.9],
                                        'aim_rx': 0.0, 'aim_ry': 0.0,
                                        'event_vel_trim': -0.115}])
    path, args = _args(tmp_path)
    args.from_artifact = str(seed_path)
    # The SUM would have been admissible: -0.115 + tau0 = -0.075, inside 0.10.
    assert abs(-0.115 + float(fitlib.TAU0[2])) < narrowed[2]

    with pytest.raises(ilc_fit.WriteRefused) as excinfo:
        ilc_fit.write_artifact(_synthetic_rows(), args)
    message = str(excinfo.value)
    assert 'SEED' in message, message
    assert 'authority' in message, message
    assert not path.exists(), 'a refused write must leave NOTHING on disk'
    assert not path.exists()


# ══════════════════════════════════════════════════════════════════════════════
# 9. C1 — the anchor prior and the session-local common mode
# ══════════════════════════════════════════════════════════════════════════════
#
# Owner decision 2 of the 2026-08-21 ILC-primary fold-in. The failure being
# closed is measured, not hypothetical: `level()` is one int16 SCL3300 sample
# with 1.2-1.7 mrad/axis of session-to-session scatter, D3 says a re-`level`
# deliberately does NOT invalidate a persisted map, and the measured per-cell
# |aim| is 9.1-10.5 mrad. That is 11-19 % of every persisted cell being one
# sitting's inclinometer noise, frozen forever and re-applied on every future
# session that never took that draw -- and none of layer 3's four provenance
# keys can see a re-`level`, so the fence has to be structural.
#
# The node's half (composition, the D7 refusal, the goal-end discard) is in
# `tests/ros/test_toss_ilc_node.py` section 9.


def _anchor(rx=math.radians(0.55), ry=math.radians(-0.24), n=7,
            se=(0.0005, 0.0005)):
    """An anchor that CLEARS the gate: 9.6 mrad at n=7, far outside both the
    deadband and 2.5 se. The magnitude is the measured one (9.1-10.5 mrad), so a
    fixture that stopped clearing the gate would be telling us something real."""
    return toss_ilc.IlcAnchor(aim_rad=(rx, ry), n=int(n),
                              se_rad=(float(se[0]), float(se[1])))


def _anchor_doc(anchor=None, cells=None, **over):
    """`_doc` plus an `anchor` block, written the way `build_document` writes it."""
    doc = _doc(cells=cells, **over)
    a = _anchor() if anchor is None else anchor
    doc['anchor'] = {'aim_rad': [a.aim_rad[0], a.aim_rad[1]], 'n': a.n,
                     'se_rad': [a.se_rad[0], a.se_rad[1]]}
    return doc


def test_the_anchor_gate_constants_are_the_session_trims_own():
    """The gate is TRANSPOSED from `toss_trim`, not invented here, and the three
    constants are pinned equal to it — the same restate-don't-import shape
    ``ILC_AIM_MAX_RAD`` follows for the same reason (a second importer of a
    module is a second owner of it).

    That matters beyond tidiness: each of the three carries a MEASUREMENT.
    ``N_MIN_APPLY = 6`` came from a 300-400 session probe that found the design's
    ``n >= 3`` gate let 45.7 % of ZERO-bias sessions command a non-zero
    correction; ``SE_GATE = 2.5`` was chosen on the measured expected residual
    aim error in mm rather than on a false-action rate; ``DEADBAND_RAD`` is
    0.10 deg = 5.46 mm, ~1/6 of the capture radius. Re-deriving any of them here
    would be re-deriving a probe nobody re-ran.
    """
    assert toss_ilc.ANCHOR_N_MIN == toss_trim.N_MIN_APPLY == 6
    assert toss_ilc.ANCHOR_SE_GATE == toss_trim.SE_GATE == 2.5
    assert toss_ilc.ANCHOR_DEADBAND_RAD == toss_trim.DEADBAND_RAD
    assert toss_ilc.ANCHOR_DEADBAND_RAD == math.radians(0.10)


def test_an_absent_anchor_block_is_a_DECLARATION_and_composes_to_zero():
    """No ``anchor`` block means "this artifact's cells are not
    anchor-referenced", which is the pre-C1 shape, and it must compose to
    EXACTLY zero rather than to a small number.

    It is legal rather than refused on purpose: no ``config/toss_ilc.yaml`` has
    ever existed in either tree, so there is no installed base to protect — the
    thing being protected is the fit tool's ability to write a usable artifact
    before it has learnt to compute an anchor. The reason is NAMED
    (``no_anchor``, not ``no_artifact``), because "layer 3 is off" and "layer 3
    is on but this file predates C1" are different operator situations.
    """
    parsed = parse_toss_ilc(_doc())
    assert parsed.anchor is None
    session = toss_ilc.IlcSessionCommonMode(parsed)
    assert session.aim() == (0.0, 0.0)
    assert session.applied is False
    assert session.reason == toss_ilc.SESSION_NO_ANCHOR
    assert session.n == 0
    # ... and no artifact at all is a THIRD, distinguishable state.
    assert toss_ilc.IlcSessionCommonMode(None).reason == \
        toss_ilc.SESSION_NO_ARTIFACT


@pytest.mark.parametrize('drop', ['aim_rad', 'n', 'se_rad'])
def test_a_present_anchor_block_is_ALL_OR_NOTHING(drop):
    """Declare an anchor and you declare its evidence. The gate that decides
    whether to apply the prior is made of all three fields, so an anchor without
    its ``n`` or its ``se_rad`` is a number this loader could only apply on
    faith — and the module's posture everywhere else is to refuse what it cannot
    check rather than to guess a permissive default."""
    doc = _anchor_doc()
    del doc['anchor'][drop]
    with pytest.raises(TossIlcError) as excinfo:
        parse_toss_ilc(doc)
    assert 'anchor.{}'.format(drop) in str(excinfo.value)


def test_the_anchor_n_counts_SESSIONS_not_tosses():
    """``n`` is independent evidence units — ``level()`` draws — so it must be an
    integer count and a float is refused by name.

    The distinction is the whole gate: sixty tosses in one sitting are ONE draw,
    and a gate keyed on the toss count would read that as overwhelming evidence
    for a number whose entire error budget is between-session. A schema that
    accepted ``n: 60.0`` invites exactly that misreading.
    """
    doc = _anchor_doc()
    doc['anchor']['n'] = 7.0
    with pytest.raises(TossIlcError) as excinfo:
        parse_toss_ilc(doc)
    message = str(excinfo.value)
    assert 'INDEPENDENT' in message and 'sessions' in message, message
    doc['anchor']['n'] = -1
    with pytest.raises(TossIlcError):
        parse_toss_ilc(doc)


def test_the_anchor_is_bounded_by_the_SAME_authority_the_cells_are():
    """A prior past 1.0 deg is a fit fault for exactly the reasons a cell past it
    is, so it is refused where it is written — not clamped. A truncated prior is
    not the prior that was fitted."""
    over = ILC_AIM_MAX_RAD * 1.01
    with pytest.raises(TossIlcError) as excinfo:
        parse_toss_ilc(_anchor_doc(_anchor(rx=over, ry=0.0)))
    assert 'authority' in str(excinfo.value)
    # Exactly AT the authority is admitted: the bound is a refusal above it, not
    # a strict inequality that would make the last legal value illegal.
    parse_toss_ilc(_anchor_doc(_anchor(rx=ILC_AIM_MAX_RAD, ry=0.0),
                               cells=[{'key': [0.0, 150.0, 170.0, 0.9],
                                       'aim_rx': 0.0, 'aim_ry': 0.0,
                                       'event_vel_trim': 0.0}]))


def test_cell_PLUS_anchor_is_bounded_and_that_is_a_THIRD_check():
    """THE composed bound, and the one neither half can see.

    ``_parse_cell`` bounds the spatial residual and ``_parse_anchor`` bounds the
    common mode, but layer 3 APPLIES their sum, and two separately-legal halves
    sum to 2.0 deg. This is the same argument D7 makes for clamping the total at
    the node's apply seam rather than at any layer's own update, one level down —
    and it is checked at PARSE because that is where both numbers first exist
    together.
    """
    half = ILC_AIM_MAX_RAD * 0.6
    cells = [{'key': [0.0, 150.0, 170.0, 0.9],
              'aim_rx': half, 'aim_ry': 0.0, 'event_vel_trim': 0.0}]
    doc = _anchor_doc(_anchor(rx=half, ry=0.0), cells=cells)
    # Each half is legal on its own ...
    assert half < ILC_AIM_MAX_RAD
    # ... and the SUM is what the machine would command.
    with pytest.raises(TossIlcError) as excinfo:
        parse_toss_ilc(doc)
    message = str(excinfo.value)
    assert 'plus anchor' in message, message
    assert 'their SUM is what layer 3 applies' in message, message


def test_a_THIN_anchor_commands_nothing():
    """Fewer than ``ANCHOR_N_MIN`` independent ``level()`` draws and the prior is
    not applied at all.

    Root cause, measured: the trim's own probe found a thin-evidence
    significance gate let 45.7 % of ZERO-bias sessions command a non-zero
    correction, because a single-look test re-evaluated as evidence trickles in
    is a sequential multiple-comparison problem. Two sessions of scatter at
    1.2-1.7 mrad/axis is enough to mint a confident-looking common mode out of
    nothing.
    """
    for n in range(toss_ilc.ANCHOR_N_MIN):
        parsed = parse_toss_ilc(_anchor_doc(_anchor(n=n)))
        session = toss_ilc.IlcSessionCommonMode(parsed)
        assert session.aim() == (0.0, 0.0), n
        assert session.reason == toss_ilc.SESSION_INSUFFICIENT_EVIDENCE
        assert session.n == n
    # ... and exactly AT the gate it applies.
    at = toss_ilc.IlcSessionCommonMode(
        parse_toss_ilc(_anchor_doc(_anchor(n=toss_ilc.ANCHOR_N_MIN))))
    assert at.applied is True
    assert at.aim() != (0.0, 0.0)


def test_the_SE_gate_is_PER_AXIS_and_zeroes_only_the_axis_it_refuses():
    """Per axis, like the trim's, because the two axes carry independent
    evidence: a significant ``rx`` must not be suppressed by an insignificant
    ``ry``, and an insignificant ``ry`` must not ride along on ``rx``'s
    significance.

    A whole-vector gate would fail both ways at once, and the second direction is
    the dangerous one — it commands an axis nothing measured.
    """
    aim = (math.radians(0.55), math.radians(0.05))
    # ry's se is wide enough that 2.5*se swallows it; rx's is not.
    se = (0.0005, math.radians(0.05) / toss_ilc.ANCHOR_SE_GATE * 1.01)
    session = toss_ilc.IlcSessionCommonMode(
        parse_toss_ilc(_anchor_doc(_anchor(rx=aim[0], ry=aim[1], se=se))))
    assert session.applied is True
    assert session.aim()[0] == pytest.approx(aim[0])
    assert session.aim()[1] == 0.0, 'the insignificant axis must be zeroed'

    # Both axes insignificant -> nothing at all, and the reason says so.
    wide = (aim[0] / toss_ilc.ANCHOR_SE_GATE * 1.01,
            aim[1] / toss_ilc.ANCHOR_SE_GATE * 1.01)
    none = toss_ilc.IlcSessionCommonMode(
        parse_toss_ilc(_anchor_doc(_anchor(rx=aim[0], ry=aim[1], se=wide))))
    assert none.aim() == (0.0, 0.0)
    assert none.reason == toss_ilc.SESSION_BELOW_SE_GATE


def test_the_deadband_judges_what_SURVIVES_the_se_gate_not_the_raw_anchor():
    """Ordering. The per-axis significance gate runs FIRST and zeroes the axes it
    refuses; the deadband then judges the magnitude of what is left.

    The other order lets a single significant axis drag an insignificant one over
    the deadband on the hypotenuse — the vector clears 0.10 deg only because of a
    component the evidence just refused.
    """
    small = math.radians(0.08)          # each axis alone is inside the deadband
    assert small < toss_ilc.ANCHOR_DEADBAND_RAD
    assert math.hypot(small, small) > toss_ilc.ANCHOR_DEADBAND_RAD
    # ry insignificant, rx significant: what survives is (small, 0), which is
    # INSIDE the deadband even though the raw pair's magnitude is outside it.
    se = (0.0001, small / toss_ilc.ANCHOR_SE_GATE * 1.01)
    session = toss_ilc.IlcSessionCommonMode(
        parse_toss_ilc(_anchor_doc(_anchor(rx=small, ry=small, se=se))))
    assert session.aim() == (0.0, 0.0)
    assert session.reason == toss_ilc.SESSION_INSIDE_DEADBAND


def test_an_anchor_inside_the_deadband_commands_nothing():
    """0.10 deg = 5.46 mm at the reference geometry, ~1/6 of the capture radius.
    Below it the loop churns the commanded pose for no measurable catch
    benefit."""
    tiny = toss_ilc.ANCHOR_DEADBAND_RAD * 0.5
    session = toss_ilc.IlcSessionCommonMode(
        parse_toss_ilc(_anchor_doc(_anchor(rx=tiny, ry=0.0, se=(0.0, 0.0)))))
    assert session.aim() == (0.0, 0.0)
    assert session.reason == toss_ilc.SESSION_INSIDE_DEADBAND


def test_the_anchor_is_inside_the_artifact_VERSION_hash():
    """The hash covers every number the machine ACTS ON — and all three anchor
    fields are acted on, including the two that are never commanded. Editing
    ``n`` from 5 to 6 turns a dormant prior into a live commanded correction
    without touching a single aim value, so a version that did not move would be
    reporting two materially different machines as the same one."""
    base = artifact_version(_anchor_doc())
    assert base != artifact_version(_doc()), 'adding an anchor must move it'
    for field, value in (('n', 9), ('se_rad', [0.002, 0.002]),
                         ('aim_rad', [0.001, 0.001])):
        moved = _anchor_doc()
        moved['anchor'][field] = value
        assert artifact_version(moved) != base, field


def test_the_session_component_is_SEEDED_AND_HELD_with_no_update_path():
    """The honest limitation, pinned so it cannot be quietly "fixed" into an
    unvalidated live loop.

    There is no live-admissible observable to update this from: ``land_err_mm``
    is mined offline and both live candidates (``catch_error_mm``,
    ``BallState.landing_position``) are refused on their own merits by D5 —
    ``toss_trim``'s ``no_mocap_fit`` refusal is the same wall, hit by the same
    machine. So the component exposes no observe/update/write path at all, and
    its value is constant for the goal's whole life by construction.

    That constancy is also why there is no CUSUM and no freeze-never-zero here:
    both are defences against an UPDATING estimator (a shift in a stream, and an
    authority-sized step injected into the next commanded pose), and with no
    update path there is no stream and no step. Porting them would be dead code
    wearing a safety argument.
    """
    session = toss_ilc.IlcSessionCommonMode(parse_toss_ilc(_anchor_doc()))
    first = session.aim()
    for _ in range(5):
        assert session.aim() == first
    for name in ('observe', 'update', 'step', 'write', 'save', 'proposal'):
        assert not hasattr(session, name), name


def test_the_component_never_writes_and_the_cells_never_learn_the_common_mode():
    """**THE C1 fence, stated as the property it guarantees.**

    Seeding, gating and applying the prior must leave the persisted artifact
    byte-identical — a session's ``level()`` draw is applied and then dropped, and
    nothing about it reaches a cell. Two different session components built from
    one artifact leave that artifact, its version and every cell unchanged.
    """
    parsed = parse_toss_ilc(_anchor_doc())
    before_cells = dict(parsed.cells)
    before_version = parsed.version
    before_anchor = parsed.anchor

    applied = toss_ilc.IlcSessionCommonMode(parsed, goal_id='goal-1')
    assert applied.applied is True and applied.aim() != (0.0, 0.0)
    refused = toss_ilc.IlcSessionCommonMode(parsed, goal_id='goal-2',
                                            n_min=99)
    assert refused.aim() == (0.0, 0.0)

    assert parsed.cells == before_cells
    assert parsed.version == before_version
    assert parsed.anchor == before_anchor
    # ... and the applied value is nowhere in any cell.
    for correction in parsed.cells.values():
        assert correction.aim_rad != applied.aim()


def test_build_document_round_trips_an_anchor_and_subtracts_NOTHING():
    """The writer lives in the production module so there is one schema, and it
    does NOT re-reference its caller's cells: a writer that silently subtracted
    the anchor would make ``cells`` mean one thing in the fit and another on
    disk, which is the C1 double-count wearing the opposite hat."""
    anchor = _anchor()
    cells = [([0.0, 150.0, 170.0, 0.9], [0.0011, -0.0023, -0.1076])]
    doc = toss_ilc.build_document(cells, tilt_map_version=_TILT_V,
                                  toss_cal_version=_CAL_V,
                                  date='2026-08-22', tool='test',
                                  anchor=anchor)
    parsed = parse_toss_ilc(doc)
    assert parsed.anchor == anchor
    key = goal_key(0.0, 150.0, 170.0, 0.9)
    assert parsed.cells[key].aim_rx == pytest.approx(0.0011)
    assert parsed.cells[key].aim_ry == pytest.approx(-0.0023)
    # Omitting the anchor writes no block at all — absent is a declaration.
    bare = toss_ilc.build_document(cells, tilt_map_version=_TILT_V,
                                   toss_cal_version=_CAL_V,
                                   date='2026-08-22', tool='test')
    assert 'anchor' not in bare
    assert parse_toss_ilc(bare).anchor is None


def test_an_unknown_key_in_the_anchor_block_is_refused_by_name():
    """Same rule the cells follow: a field this build does not understand may be
    a field that changes what the others MEAN, and best-effort parsing it is a
    silent command error."""
    doc = _anchor_doc()
    doc['anchor']['aim_mm'] = [1.0, 2.0]
    with pytest.raises(TossIlcError) as excinfo:
        parse_toss_ilc(doc)
    assert 'aim_mm' in str(excinfo.value)
