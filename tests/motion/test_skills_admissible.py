"""``motion/skills/admissible`` -- the offline admissible box (plan § 2.6).

Round-trip (dump/load), :func:`admissible.clip`, and the two loader refusals
(a limits mismatch, a gate-hash mismatch) are pure-Python and unmarked. The
one end-to-end case runs the REAL sweep (``tools/admissible_sweep.py``) on a
tiny 2x2 grid -- module-scoped, ~10 QP solves, the same shape
``tests/motion/test_skills_segments.py`` uses for its own real-solve fixtures.

Plan: ``plans/active/two-ball-skill-stack.md`` § 2.6.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.skills import admissible as ab
from jugglebot.motion.skills import sites as st
from jugglebot.motion.trajectory.limits import TrajectoryLimits

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SWEEP_PATH = os.path.join(_REPO, 'tools', 'admissible_sweep.py')


def _load_sweep_module():
    spec = importlib.util.spec_from_file_location('admissible_sweep', _SWEEP_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_LIMITS_DICT = dict(leg_vel_mmps=300.0, leg_acc_mmps2=5000.0,
                    leg_jerk_mmps3=200000.0, hand_acc_rps2=3500.0)
#: The REAL live gate hash -- so a box built with the default here matches
#: what ``check_limits``'s gate check compares against, and the mismatch test
#: below only has to change ONE field to exercise the refusal.
_GATE_HASH = ab.gate_hash()


#: The default release/target xy every ``_box()`` / ``_empty_box()`` carries
#: unless overridden -- arbitrary but finite (most tests below exercise
#: validation / dump / load / clip / check_limits, which never look at the
#: xy's physical meaning); the ``select`` xy-matching tests pass explicit
#: ``release_site_xy_mm`` / ``target_site_xy_mm`` on both sides.
_DEFAULT_XY_MM = (-50.0, 50.0)


def _box(**overrides):
    kwargs = dict(
        site_pair=('P1', 'P2'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((-0.02, 0.02), (-0.01, 0.01)), apex_m=(0.87, 0.93),
        pattern='hop', release_site_xy_mm=_DEFAULT_XY_MM,
        target_site_xy_mm=_DEFAULT_XY_MM,
        limits=dict(_LIMITS_DICT), gate_hash=_GATE_HASH, swept_at='2026-09-12')
    kwargs.update(overrides)
    return ab.AdmissibleBox(**kwargs)


def _empty_box(**overrides):
    kwargs = dict(
        site_pair=('P1', 'P2'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((float('nan'), float('nan')), (float('nan'), float('nan'))),
        apex_m=(float('nan'), float('nan')), pattern='hop',
        release_site_xy_mm=_DEFAULT_XY_MM, target_site_xy_mm=_DEFAULT_XY_MM,
        limits=dict(_LIMITS_DICT),
        gate_hash=_GATE_HASH, swept_at='2026-09-12')
    kwargs.update(overrides)
    return ab.AdmissibleBox(**kwargs)


# ---------------------------------------------------------------------------
# AdmissibleBox validation
# ---------------------------------------------------------------------------

def test_box_rejects_a_bad_site_pair():
    with pytest.raises(ValueError, match='site_pair'):
        _box(site_pair=('P1',))


def test_box_rejects_an_inverted_range():
    with pytest.raises(ValueError, match='apex_band_m'):
        _box(apex_band_m=(0.95, 0.85))


def test_box_rejects_a_half_empty_sentinel():
    """landing_xy_m and apex_m must be empty (nan) TOGETHER, never one alone
    -- a half-empty box is a bug in whatever built it, not a physical state."""
    with pytest.raises(ValueError, match='empty'):
        _box(apex_m=(float('nan'), float('nan')))


def test_box_rejects_missing_limit_keys():
    with pytest.raises(ValueError, match='leg_vel_mmps'):
        _box(limits={'leg_acc_mmps2': 5000.0, 'leg_jerk_mmps3': 200000.0,
                     'hand_acc_rps2': 3500.0})


def test_box_rejects_a_malformed_gate_hash():
    with pytest.raises(ValueError, match='gate_hash'):
        _box(gate_hash='not-hex!!')


def test_box_rejects_a_pattern_outside_the_vocabulary():
    with pytest.raises(ValueError, match='pattern'):
        _box(pattern='crossing')


def test_box_rejects_a_non_finite_site_xy():
    with pytest.raises(ValueError, match='release_site_xy_mm'):
        _box(release_site_xy_mm=(float('nan'), 0.0))
    with pytest.raises(ValueError, match='target_site_xy_mm'):
        _box(target_site_xy_mm=(0.0, float('inf')))


def test_empty_box_reports_empty():
    assert _box().empty is False
    assert _empty_box().empty is True


# ---------------------------------------------------------------------------
# gate_hash
# ---------------------------------------------------------------------------

def test_gate_hash_is_twelve_lowercase_hex_chars_and_deterministic():
    h1 = ab.gate_hash()
    h2 = ab.gate_hash()
    assert h1 == h2
    assert len(h1) == 12
    assert all(c in '0123456789abcdef' for c in h1)


_GATE_TREE_FILES = ('segments.py', 'unified_cycle.py',
                    os.path.join('trajectory', 'feasibility.py'),
                    os.path.join('trajectory', 'cup_cycle.py'),
                    os.path.join('trajectory', 'cup_realize.py'),
                    os.path.join('trajectory', 'tilt_geometry.py'),
                    'hardware_config.py')


def _write_gate_tree(root, contents='x'):
    os.makedirs(os.path.join(root, 'trajectory'), exist_ok=True)
    for rel in _GATE_TREE_FILES:
        with open(os.path.join(root, rel), 'w') as handle:
            handle.write(contents)


def test_gate_tree_mirrors_the_live_gated_file_list():
    """The tmp-tree list above IS the live `_GATED_FILES` list, flattened --
    a file added to one and not the other would leave the parametrised test
    below silently not covering it."""
    live = {os.path.join(*parts) for parts in ab._GATED_FILES}
    assert live == set(_GATE_TREE_FILES)


@pytest.mark.parametrize('which', _GATE_TREE_FILES)
def test_gate_hash_changes_when_any_of_the_gated_files_change(tmp_path, which):
    """R4 widened the gate from {feasibility, segments}.py alone to all six
    files that shape the real solve (cup_cycle.py / cup_realize.py /
    unified_cycle.py / tilt_geometry.py too) -- an edit to ANY of them must
    change the hash, or a box swept against a stale one of the four newly
    added files goes undetected (plan R4 carried item (d)). The kinematic
    calibration (2026-09-27) added the generated `hardware_config.py`, the IK
    geometry: a geometry change must force the re-sweep, not rely on it being
    remembered (plans/active/kinematic-calibration.md § 7)."""
    root = str(tmp_path)
    _write_gate_tree(root)
    before = ab.gate_hash(root=root)
    with open(os.path.join(root, which), 'a') as handle:
        handle.write('y')
    after = ab.gate_hash(root=root)
    assert before != after


# ---------------------------------------------------------------------------
# dump / load round trip
# ---------------------------------------------------------------------------

def test_round_trip_dump_then_load_is_equal(tmp_path):
    boxes = [_box(), _box(site_pair=('P2', 'P1'),
                          landing_xy_m=((-0.01, 0.01), (-0.02, 0.02)))]
    path = str(tmp_path / 'admissible_box.yaml')
    ab.dump(path, boxes)
    loaded = ab.load(path)
    assert len(loaded) == len(boxes)
    for original, back in zip(boxes, loaded):
        assert back == original


def test_round_trip_dump_then_load_preserves_dwell_s(tmp_path):
    """R5 (D4, 2026-09-30): dwell_s is a per-box stamp (schedule.Pattern.dwell_s
    / skill_node's own dwell_s parameter were an unenforced timing twin before
    this field existed -- plan § 0)."""
    box = _box(dwell_s=0.25)
    path = str(tmp_path / 'x.yaml')
    ab.dump(path, [box])
    loaded = ab.load(path)
    assert loaded[0].dwell_s == pytest.approx(0.25)


def test_round_trip_dump_then_load_tolerates_an_unstamped_dwell_s(tmp_path):
    """A box built without dwell_s (dataclass default None -- the Python-level
    escape hatch for callers outside this unit's file list, e.g.
    tests/ros/test_skill_node.py, that construct AdmissibleBox directly and
    never set it) round-trips as None, not refused and not guessed."""
    box = _box(dwell_s=None)
    path = str(tmp_path / 'x.yaml')
    ab.dump(path, [box])
    loaded = ab.load(path)
    assert loaded[0].dwell_s is None


def test_box_rejects_a_non_positive_dwell_s():
    with pytest.raises(ValueError, match='dwell_s'):
        _box(dwell_s=0.0)
    with pytest.raises(ValueError, match='dwell_s'):
        _box(dwell_s=-0.1)


def test_check_limits_passes_when_the_dwell_matches():
    ab.check_limits([_box(dwell_s=0.30)], _live_limits(), dwell_s=0.30)


def test_check_limits_refuses_on_a_dwell_mismatch():
    with pytest.raises(ab.LimitsMismatch, match='dwell_s'):
        ab.check_limits([_box(dwell_s=0.30)], _live_limits(), dwell_s=0.20)


def test_check_limits_refuses_an_unstamped_dwell_against_a_live_one():
    """A box that never carried a dwell_s (None) is treated exactly like a
    numeric mismatch when the caller actually cares about the live dwell --
    it is not silently assumed to match."""
    with pytest.raises(ab.LimitsMismatch, match='dwell_s'):
        ab.check_limits([_box(dwell_s=None)], _live_limits(), dwell_s=0.30)


def test_check_limits_skips_the_dwell_check_when_not_given():
    """The default (`dwell_s=None` on the call) is a no-op -- every EXISTING
    caller (skill_node.py's two call sites, tests/hardware/skills_plan_bench.py)
    keeps working unchanged until it is wired to pass its own live dwell."""
    ab.check_limits([_box(dwell_s=None)], _live_limits())
    ab.check_limits([_box(dwell_s=0.20)], _live_limits())


def test_dump_refuses_a_mix_of_provenance(tmp_path):
    boxes = [_box(), _box(swept_at='2026-09-13')]
    with pytest.raises(ValueError, match='swept_at|gate_hash|limits'):
        ab.dump(str(tmp_path / 'x.yaml'), boxes)


def test_dump_refuses_zero_boxes(tmp_path):
    with pytest.raises(ValueError):
        ab.dump(str(tmp_path / 'x.yaml'), [])


@pytest.mark.parametrize('missing_key', ['swept_at', 'gate_hash', 'limits', 'boxes'])
def test_load_refuses_a_missing_top_level_key(tmp_path, missing_key):
    boxes = [_box()]
    path = str(tmp_path / 'admissible_box.yaml')
    ab.dump(path, boxes)
    import yaml
    with open(path) as handle:
        doc = yaml.safe_load(handle)
    del doc[missing_key]
    with open(path, 'w') as handle:
        yaml.safe_dump(doc, handle)
    with pytest.raises(ab.AdmissibleError, match=missing_key):
        ab.load(path)


def test_load_refuses_a_non_mapping_document(tmp_path):
    path = str(tmp_path / 'x.yaml')
    with open(path, 'w') as handle:
        handle.write('- just\n- a\n- list\n')
    with pytest.raises(ab.AdmissibleError, match='mapping'):
        ab.load(path)


# ---------------------------------------------------------------------------
# clip
# ---------------------------------------------------------------------------

def test_clip_inside_the_box_is_unchanged():
    box = _box()
    u = (np.array([0.005, -0.002]), 0.90)
    xy, apex = ab.clip(u, box)
    np.testing.assert_allclose(xy, [0.005, -0.002])
    assert apex == pytest.approx(0.90)


def test_clip_outside_the_box_saturates_to_the_nearest_edge():
    box = _box()
    u = (np.array([1.0, -1.0]), 5.0)
    xy, apex = ab.clip(u, box)
    np.testing.assert_allclose(xy, [0.02, -0.01])
    assert apex == pytest.approx(0.93)


def test_clip_on_an_empty_box_refuses_naming_the_site_pair():
    box = _empty_box(site_pair=('P2', 'P1'))
    with pytest.raises(ab.AdmissibleError, match='P2'):
        ab.clip((np.array([0.0, 0.0]), 0.90), box)


def test_load_refuses_a_pre_r4_file_missing_pattern_and_site_xy(tmp_path):
    """A file swept before R4 (unit U0, 2026-09-23) has every _REQUIRED_BOX
    key but none of pattern / release_site_xy_mm / target_site_xy_mm --
    refused naming the missing fields and pointing at the sweep command,
    never reinterpreted with a guessed pattern or xy."""
    import yaml as _yaml
    doc = {
        'swept_at': '2026-09-12', 'gate_hash': _GATE_HASH,
        'limits': dict(_LIMITS_DICT),
        'boxes': [{'site_pair': ['P1', 'P2'], 'apex_band_m': [0.85, 0.95],
                   'landing_xy_m': [[-0.02, 0.02], [-0.01, 0.01]],
                   'apex_m': [0.87, 0.93]}],
    }
    path = str(tmp_path / 'pre_r4.yaml')
    with open(path, 'w') as handle:
        _yaml.safe_dump(doc, handle)
    with pytest.raises(ab.AdmissibleError,
                       match='pattern.*admissible_sweep|admissible_sweep.*pattern'):
        ab.load(path)


def test_load_refuses_a_box_missing_apex_m():
    """2026-09-21: the pre-2026-09-18 ``flight_s`` compatibility branch was
    retired (no other YAML/fixture depended on it), so ``apex_m`` is now a
    required field like ``site_pair`` / ``apex_band_m`` / ``landing_xy_m`` --
    a box missing it refuses naming it, full stop."""
    import tempfile, yaml as _yaml
    doc = {
        'swept_at': '2026-09-12', 'gate_hash': _GATE_HASH,
        'limits': dict(_LIMITS_DICT),
        'boxes': [{'site_pair': ['P1', 'P2'], 'apex_band_m': [0.85, 0.95],
                   'landing_xy_m': [[-0.02, 0.02], [-0.01, 0.01]]}],
    }
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as fh:
        _yaml.safe_dump(doc, fh)
        path = fh.name
    with pytest.raises(ab.AdmissibleError, match='apex_m'):
        ab.load(path)


# ---------------------------------------------------------------------------
# select
# ---------------------------------------------------------------------------

def _sel(boxes, pattern, pair, apex, xy=_DEFAULT_XY_MM):
    return ab.select(boxes, pattern, pair, apex, release_site_xy_mm=xy,
                     target_site_xy_mm=xy)


def test_select_hits_a_box_covering_the_apex():
    box = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95), pattern='self_toss')
    assert _sel([box], 'self_toss', ('P1', 'P1'), 0.90) is box


def test_select_hits_at_the_inclusive_boundary():
    box = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95), pattern='self_toss')
    assert _sel([box], 'self_toss', ('P1', 'P1'), 0.85) is box
    assert _sel([box], 'self_toss', ('P1', 'P1'), 0.95) is box


def test_select_misses_an_apex_outside_the_band():
    box = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95), pattern='self_toss')
    assert _sel([box], 'self_toss', ('P1', 'P1'), 0.50) is None


def test_select_misses_the_right_apex_at_the_wrong_site_pair():
    box = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95), pattern='self_toss')
    assert _sel([box], 'self_toss', ('P1', 'P2'), 0.90) is None


def test_select_misses_the_right_pair_at_the_wrong_pattern():
    """A columns box and a self-toss box may legitimately share a site_pair
    -- select must not confuse them (the latent defect a shared key would
    reopen: a self-toss silently reusing a columns box's bounds)."""
    box = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95), pattern='columns')
    assert _sel([box], 'self_toss', ('P1', 'P1'), 0.90) is None
    assert _sel([box], 'columns', ('P1', 'P1'), 0.90) is box


def test_select_misses_when_the_live_site_xy_does_not_match_the_swept_xy():
    """R4: a box carries no site geometry beyond its own xy stamp -- a box
    swept at one separation (e.g. a hop at 250 mm) must refuse when applied
    at another (e.g. a schedule at 100 mm), even with a matching pattern and
    site-name pair."""
    box = _box(site_pair=('P1', 'P2'), apex_band_m=(0.85, 0.95), pattern='hop',
              release_site_xy_mm=(-125.0, 0.0), target_site_xy_mm=(125.0, 0.0))
    hit = ab.select([box], 'hop', ('P1', 'P2'), 0.90,
                    release_site_xy_mm=(-125.0, 0.0), target_site_xy_mm=(125.0, 0.0))
    assert hit is box
    miss = ab.select([box], 'hop', ('P1', 'P2'), 0.90,
                     release_site_xy_mm=(-50.0, 0.0), target_site_xy_mm=(50.0, 0.0))
    assert miss is None


def test_select_picks_the_right_box_among_several_apex_bands():
    lo = _box(site_pair=('P1', 'P1'), apex_band_m=(0.45, 0.55), pattern='self_toss')
    hi = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95), pattern='self_toss')
    assert _sel([lo, hi], 'self_toss', ('P1', 'P1'), 0.50) is lo
    assert _sel([lo, hi], 'self_toss', ('P1', 'P1'), 0.90) is hi
    assert _sel([lo, hi], 'self_toss', ('P1', 'P1'), 0.70) is None


def test_dump_refuses_two_boxes_for_one_pair_with_overlapping_apex_bands(tmp_path):
    boxes = [_box(site_pair=('P1', 'P1'), apex_band_m=(0.45, 0.60)),
            _box(site_pair=('P1', 'P1'), apex_band_m=(0.55, 0.70))]
    with pytest.raises(ValueError, match='overlapping apex_band_m'):
        ab.dump(str(tmp_path / 'x.yaml'), boxes)


def test_dump_allows_touching_apex_bands_for_one_pair(tmp_path):
    boxes = [_box(site_pair=('P1', 'P1'), apex_band_m=(0.45, 0.55)),
            _box(site_pair=('P1', 'P1'), apex_band_m=(0.55, 0.65))]
    ab.dump(str(tmp_path / 'x.yaml'), boxes)  # must not raise


def test_dump_allows_overlapping_apex_bands_for_different_pairs(tmp_path):
    boxes = [_box(site_pair=('P1', 'P1'), apex_band_m=(0.45, 0.60)),
            _box(site_pair=('P1', 'P2'), apex_band_m=(0.45, 0.60))]
    ab.dump(str(tmp_path / 'x.yaml'), boxes)  # must not raise


def test_dump_allows_overlapping_apex_bands_for_the_same_pair_different_pattern(
        tmp_path):
    """R4: a 'columns' box and a 'self_toss' box may share a site_pair of
    (P1, P1) (both key release==target) -- overlap is only ambiguous within
    one pattern, since select() always filters by pattern first."""
    boxes = [_box(site_pair=('P1', 'P1'), apex_band_m=(0.45, 0.60), pattern='columns'),
            _box(site_pair=('P1', 'P1'), apex_band_m=(0.45, 0.60), pattern='self_toss')]
    ab.dump(str(tmp_path / 'x.yaml'), boxes)  # must not raise


def test_load_refuses_overlapping_apex_bands_for_one_pair(tmp_path):
    path = str(tmp_path / 'x.yaml')
    boxes = [_box(site_pair=('P1', 'P1'), apex_band_m=(0.45, 0.60))]
    ab.dump(path, boxes)
    import yaml
    with open(path) as handle:
        doc = yaml.safe_load(handle)
    extra = dict(doc['boxes'][0])
    extra['apex_band_m'] = [0.55, 0.70]
    doc['boxes'].append(extra)
    with open(path, 'w') as handle:
        yaml.safe_dump(doc, handle)
    with pytest.raises(ab.AdmissibleError, match='overlapping apex_band_m'):
        ab.load(path)


# ---------------------------------------------------------------------------
# check_limits
# ---------------------------------------------------------------------------

def _live_limits(**overrides):
    kwargs = dict(leg_vel_mmps=300.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
                 hand_acc_rps2=3500.0)
    kwargs.update(overrides)
    return TrajectoryLimits.from_config(hw).with_session_limits(**kwargs)


def test_check_limits_passes_when_the_box_matches_the_live_session():
    ab.check_limits([_box()], _live_limits())


def test_check_limits_refuses_naming_the_differing_field():
    live = _live_limits(leg_jerk_mmps3=100000.0)
    with pytest.raises(ab.LimitsMismatch, match='leg_jerk_mmps3'):
        ab.check_limits([_box()], live)


def test_check_limits_refuses_on_a_stale_gate_hash():
    stale = _box(gate_hash='f' * 12)
    with pytest.raises(ab.LimitsMismatch, match='gate_hash'):
        ab.check_limits([stale], _live_limits())


def test_check_limits_gate_check_can_be_disabled_for_a_stale_hash():
    """The ``check_gate=False`` escape hatch: a working-tree edit to
    ``feasibility.py`` / ``segments.py`` mid-session must not fail a caller
    that only cares about the numeric limits."""
    stale = _box(gate_hash='f' * 12)
    ab.check_limits([stale], _live_limits(), check_gate=False)


# ---------------------------------------------------------------------------
# End-to-end: a tiny real sweep (module-scoped, ~10 solves)
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def sweep_mod():
    return _load_sweep_module()


#: The session limits ``tiny_sweep`` is pinned at (see its docstring).
_TINY_SWEEP_LIMITS = dict(leg_vel_mmps=300.0, leg_acc_mmps2=5000.0,
                          leg_jerk_mmps3=150000.0)


@pytest.fixture(scope='module')
def tiny_sweep(sweep_mod):
    """R5 (D4, 2026-09-30): the real cell needs NO ``pre_release_hold_s``
    override -- the old override existed only to work around the transit-baked
    cell's incompatibility with the default 100 ms hold (see
    ``test_the_cross_site_branch_gates_the_real_catch_with_throw_segment``
    below for what replaced it). Re-pinned from a probe
    (``scratchpad/probe_unitB_sweep5.py``, 2026-09-30) of what the tiny grid
    actually admits AT THE DEFAULT HOLD: apex 0.90 m (the owner operating
    point) fails MARGIN at every offset tried at dwell 0.30, but apex 0.85 m
    (still inside D4's grid) passes at the identity command AND at a small +/-
    5 mm y offset (x stays pinned at 0 -- +/-5 mm x refuses CHAIN_CATCH:MARGIN
    / MARGIN) -- a real, non-empty, non-degenerate-in-y box.

    The limits are PINNED to the R2/R3 point (300/5000/150000) the probe ran
    at, not read from the tool's defaults: those follow the YAML launch
    default, which moved to the R5 point 350/5000/200000 on 2026-10-05, and at
    the roomier limits the -5 mm x cell passes (the planner is simply more
    permissive) and the "x pinned at 0" characterisation below is false. A
    characterisation test freezes the conditions it characterised."""
    site0, site1 = st.columns_sites(100.0)
    boxes, rows = sweep_mod.sweep(
        apexes_m=(0.85,), offsets_mm=(-5.0, 0.0, 5.0), dwell_s=0.30,
        leg_vel=_TINY_SWEEP_LIMITS['leg_vel_mmps'],
        leg_acc=_TINY_SWEEP_LIMITS['leg_acc_mmps2'],
        leg_jerk=_TINY_SWEEP_LIMITS['leg_jerk_mmps3'],
        site_pairs=[(site0, site1)])
    return boxes, rows


def test_the_cross_site_branch_gates_the_real_catch_with_throw_segment(
        sweep_mod, monkeypatch):
    """R5 (D4, 2026-09-30) replaces
    ``test_the_default_pre_release_hold_empties_the_columns_cell``: the
    cross-site (columns) branch must be gated on the segment
    ``schedule.compile_columns`` / ``_fold_catch_throw_pairs`` actually
    dispatches -- a CATCH carrying ``then_throw`` -- not the old transit-baked
    THROW that pattern never flies. Spy on ``segments.plan_segment`` the same
    way ``test_single_site_sweep_uses_the_carried_throw_segment`` does for the
    same-site branch, and assert at least one CATCH call in the cross-site
    sweep carried ``then_throw``."""
    calls = []
    orig = sweep_mod.sg.plan_segment

    def _spy(kind, seed, terminal, cfg, limits, geom, **kw):
        if kind == sweep_mod.sg.CATCH:
            calls.append(terminal.then_throw)
        return orig(kind, seed, terminal, cfg, limits, geom, **kw)

    monkeypatch.setattr(sweep_mod.sg, 'plan_segment', _spy)
    site0, site1 = st.columns_sites(100.0)
    boxes, rows = sweep_mod.sweep(apexes_m=(0.85,), offsets_mm=(0.0,),
                                  dwell_s=0.30, site_pairs=[(site0, site1)])
    assert calls, 'expected at least one CATCH plan_segment call from the sweep'
    assert any(tt is not None for tt in calls), (
        'the cross-site sweep never planned a CATCH with then_throw -- it '
        'regressed to the old transit-baked THROW, the segment shape '
        'schedule.compile_columns never flies')
    assert len(boxes) == 1
    assert boxes[0].pattern == 'columns'
    assert not boxes[0].empty, (
        'the identity command at apex 0.85 m / dwell 0.30 s must be '
        'admissible -- it is the tiny_sweep fixture\'s own passing cell')
    assert len(rows) >= 2


def test_tiny_sweep_produces_one_box_inside_the_swept_grid(tiny_sweep):
    boxes, rows = tiny_sweep
    assert len(boxes) == 1
    box = boxes[0]
    # R4 (2026-09-23): a columns box is keyed on (TARGET, TARGET) -- the
    # THROW releases AT the destination site (the pre-throw transit is baked
    # into the segment), which is what the executor actually looks up by.
    assert box.pattern == 'columns'
    assert box.site_pair == ('P2', 'P2')
    assert box.release_site_xy_mm == box.target_site_xy_mm
    assert not box.empty, 'apex 0.85 m must admit at least the identity-' \
                          'prior (0, 0) command at the default pre-release hold'
    (xlo, xhi), (ylo, yhi) = box.landing_xy_m
    # x is pinned at 0 in this tiny grid (+/-5 mm x refuses) -- only y widens.
    assert xlo == pytest.approx(0.0) and xhi == pytest.approx(0.0)
    assert -0.005 - 1e-9 <= ylo <= yhi <= 0.005 + 1e-9
    alo, ahi = box.apex_m
    # Only ONE apex (0.85 m) is in this tiny grid, so the band is a point.
    assert alo == pytest.approx(0.85) and ahi == pytest.approx(0.85)
    assert box.dwell_s == pytest.approx(0.30)
    # Every row actually reached a verdict -- the grid ran, not merely built.
    assert len(rows) >= 6


def test_tiny_sweep_yaml_round_trips_and_validates(tmp_path, tiny_sweep,
                                                   sweep_mod):
    boxes, _rows = tiny_sweep
    path = str(tmp_path / 'admissible_box.yaml')
    ab.dump(path, boxes)
    loaded = ab.load(path)
    assert loaded == boxes
    # "Live" here is the limits the tiny sweep actually ran at: the pinned
    # `_TINY_SWEEP_LIMITS` (since 2026-10-05; before that the tool's defaults,
    # which follow the YAML launch default) plus the tool's hand cap.
    live = _live_limits(hand_acc_rps2=sweep_mod.HAND_ACC_RPS2, **_TINY_SWEEP_LIMITS)
    ab.check_limits(loaded, live)


# ---------------------------------------------------------------------------
# End-to-end: a tiny REAL cross-site HOP sweep -- R4
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def tiny_hop_sweep(sweep_mod):
    return sweep_mod.hop_sweep(apexes_m=(0.90,), offsets_mm=(-10.0, 0.0),
                               separation_mm=250.0)


def test_tiny_hop_sweep_produces_a_hop_box_both_directions_containing_the_origin(
        tiny_hop_sweep):
    boxes, rows = tiny_hop_sweep
    assert len(boxes) == 2
    by_pair = {b.site_pair: b for b in boxes}
    assert set(by_pair) == {('P1', 'P2'), ('P2', 'P1')}
    for pair, box in by_pair.items():
        assert box.pattern == 'hop'
        assert box.release_site_xy_mm != box.target_site_xy_mm, (
            'a hop box must carry two DIFFERENT sites -- release != target, '
            'unlike columns/self_toss')
        assert not box.empty, (
            '%r must admit at least the identity-prior (0, 0) command' % (pair,))
        (xlo, xhi), (ylo, yhi) = box.landing_xy_m
        assert xlo <= 0.0 <= xhi
        assert ylo <= 0.0 <= yhi
    assert len(rows) >= 6


# ---------------------------------------------------------------------------
# R5 (D4, 2026-09-30): the hop's 0.80 m row and its own per-apex band
# ---------------------------------------------------------------------------

def test_hop_sweep_default_apex_grid_admits_the_080_row(sweep_mod):
    """D4: the hop grid gains a 0.80 m row, independent of the columns/
    self_toss APEXES_M -- probed real (scratchpad/probe_unitB_hop80.py,
    2026-09-30): apex 0.80 m at the default HOP_SEPARATION_MM passes with
    margin, both directions, at every offset in a small +/-10 mm grid."""
    assert 0.80 in sweep_mod.HOP_APEXES_M
    boxes, rows = sweep_mod.hop_sweep(apexes_m=(0.80,), offsets_mm=(-10.0, 0.0))
    assert len(boxes) == 2
    for box in boxes:
        assert box.pattern == 'hop'
        assert not box.empty, '%r must admit apex 0.80 m' % (box.site_pair,)
        alo, ahi = box.apex_m
        assert alo == pytest.approx(0.80) and ahi == pytest.approx(0.80)
        assert box.dwell_s == pytest.approx(sweep_mod.DWELL_S)
    assert len(rows) >= 6


def test_single_apex_boxes_pattern_hop_records_the_requested_band_at_080(
        sweep_mod):
    """`_single_apex_boxes` generalised to `pattern='hop'` (R5, D4): the
    resulting box's `apex_band_m` is the REQUESTED `(apex - h, apex + h)`, not
    the grid-derived band `hop_sweep` would otherwise compute -- the same
    R3-single-site defect this machinery already closes, now covering hop
    too. Real solve, kept to a single flight/offset point for wall time."""
    boxes, rows = sweep_mod._single_apex_boxes(
        [0.80], flight_frac=[0.0], halfwidth_m=0.05, offsets_mm=[0.0],
        dwell_s=0.30, separation_mm=250.0, leg_vel=300.0, leg_acc=5000.0,
        leg_jerk=150000.0, hand_acc=3500.0, pattern='hop', log=lambda r: None)
    assert len(boxes) == 2
    for box in boxes:
        assert box.pattern == 'hop'
        assert box.apex_band_m == pytest.approx((0.75, 0.85))
        assert not box.empty
    assert len(rows) >= 2


# ---------------------------------------------------------------------------
# End-to-end: a tiny REAL single-site (P1, P1) sweep -- R3
# ---------------------------------------------------------------------------

def test_single_site_sweep_uses_the_carried_throw_segment(sweep_mod, monkeypatch):
    """R3's single-site (P1, P1) box must be judged by the STEADY
    catch-with-``then_throw`` segment R3's schedule actually dispatches
    (``segments.ThenThrow``'s docstring / plan § 2.2's R2 amendment: a split
    LANDING-then-THROW refuses at every cell of a 480-cell grid), not the
    two-site cell's standalone LANDING catch. Spy on ``segments.plan_segment``
    and assert at least one CATCH call in the sweep carried ``then_throw`` --
    this fails if the single-site cell regresses to the standalone form.
    """
    calls = []
    orig = sweep_mod.sg.plan_segment

    def _spy(kind, seed, terminal, cfg, limits, geom, **kw):
        if kind == sweep_mod.sg.CATCH:
            calls.append(terminal.then_throw)
        return orig(kind, seed, terminal, cfg, limits, geom, **kw)

    monkeypatch.setattr(sweep_mod.sg, 'plan_segment', _spy)
    site = st.columns_sites(100.0)[0]
    boxes, rows = sweep_mod.sweep(flights_s=(0.80, 0.8570), offsets_mm=(-10.0, 0.0),
                                  site_pairs=[(site, site)])
    assert calls, 'expected at least one CATCH plan_segment call from the sweep'
    assert any(tt is not None for tt in calls), (
        'the single-site sweep never planned a CATCH with then_throw -- it '
        'regressed to the standalone LANDING form the two-site cell uses, '
        'which plan § 2.2 measured as infeasible for a same-site chain')
    assert len(boxes) == 1
    box = boxes[0]
    assert box.pattern == 'self_toss'
    assert box.site_pair == ('P1', 'P1')
    assert not box.empty, 'the (P1, P1) chain must admit at least the ' \
                          'identity-prior (0, 0) command at its centre flight'
    assert len(rows) >= 6


# ---------------------------------------------------------------------------
# tools/admissible_sweep.py's --single-apex CLI -- band construction and the
# argument-parsing overlap refusal (no real sweep: milliseconds)
# ---------------------------------------------------------------------------

_RING_XS = [-40.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0]
#: The 2026-09-14 --single-apex sweep's failure shape at flights near 0.64 s:
#: every small offset around the origin fails the chained catch's margin,
#: while (0, 0) itself and the large offsets pass.
_RING_FAIL = {(x, y) for x in (-20.0, -10.0, 0.0, 10.0, 20.0)
              for y in (-20.0, -10.0, 0.0, 10.0, 20.0) if (x, y) != (0.0, 0.0)}


def _contains_origin(rect):
    return rect[0] <= 0.0 <= rect[1] and rect[2] <= 0.0 <= rect[3]


def test_max_rectangle_must_contain_restricts_to_rectangles_through_the_point(
        sweep_mod):
    """Without the constraint the largest all-passing rectangle routes around
    a ring of failures and excludes the origin; with it, the returned
    rectangle contains the origin and every cell in it passes."""
    def pass_fn(x, y):
        return (x, y) not in _RING_FAIL
    free = sweep_mod._max_rectangle(_RING_XS, _RING_XS, pass_fn)
    assert not _contains_origin(free)
    rect = sweep_mod._max_rectangle(_RING_XS, _RING_XS, pass_fn,
                                    must_contain=(0.0, 0.0))
    assert _contains_origin(rect)
    assert all(pass_fn(x, y) for x in _RING_XS if rect[0] <= x <= rect[1]
               for y in _RING_XS if rect[2] <= y <= rect[3])


def test_flight_band_and_rect_always_admits_the_identity_offset(sweep_mod):
    """The box's landing rectangle must contain (0, 0): that is the command a
    cold learner issues, and ``admissible.clip`` would otherwise move it off
    the cup. The first --single-apex sweep (2026-09-14) wrote boxes whose
    rectangles excluded the origin at every apex from 0.5 to 0.9 m."""
    flights = [0.60, 0.64, 0.70]
    pass_grid = {}
    for T in flights:
        for x in _RING_XS:
            for y in _RING_XS:
                ring = T == 0.64 and (x, y) in _RING_FAIL
                pass_grid[(T, x, y)] = not ring
    band, rect = sweep_mod._flight_band_and_rect(flights, _RING_XS, pass_grid,
                                                 center_flight=0.64)
    assert band == (0.60, 0.70)
    assert rect is not None and _contains_origin(rect)


def test_refuse_overlapping_single_apex_bands_flags_an_overlap(sweep_mod):
    ap = argparse.ArgumentParser()
    with pytest.raises(SystemExit):
        sweep_mod._refuse_overlapping_single_apex_bands([0.50, 0.53], 0.05, ap)


def test_refuse_overlapping_single_apex_bands_allows_touching_bands(sweep_mod):
    ap = argparse.ArgumentParser()
    sweep_mod._refuse_overlapping_single_apex_bands([0.5, 0.6], 0.05, ap)  # no raise


def test_single_apex_boxes_records_the_requested_band_not_the_grid_derived_one(
        sweep_mod, monkeypatch):
    """`_single_apex_boxes` must override `sweep`'s own (grid-derived)
    ``apex_band_m`` with exactly ``(apex - h, apex + h)`` -- neighbouring
    apexes' grid-derived bands overlap, which is the whole reason this CLI
    exists. `sweep` itself is stubbed so this stays a unit test of the band
    construction, not a second copy of the real-solver sweep test above."""
    calls = []
    pattern_flight_calls = []

    def _fake_sweep(*, flights_s, offsets_mm, dwell_s, separation_mm, leg_vel,
                    leg_acc, leg_jerk, hand_acc, center_flight_s, site_pairs,
                    pattern_flight_s, log):
        calls.append(list(flights_s))
        pattern_flight_calls.append(pattern_flight_s)
        site = site_pairs[0][0]
        site_xy = (float(site.cup_mm[0]), float(site.cup_mm[1]))
        box = sweep_mod.ab.AdmissibleBox(
            site_pair=(site.name, site.name), apex_band_m=(0.0, 99.0),
            landing_xy_m=((-0.01, 0.01), (-0.01, 0.01)),
            apex_m=(sweep_mod.sc.apex_m(min(flights_s)),
                    sweep_mod.sc.apex_m(max(flights_s))),
            pattern='self_toss', release_site_xy_mm=site_xy,
            target_site_xy_mm=site_xy,
            limits=dict(leg_vel_mmps=leg_vel, leg_acc_mmps2=leg_acc,
                       leg_jerk_mmps3=leg_jerk, hand_acc_rps2=hand_acc),
            gate_hash=sweep_mod.ab.gate_hash(), swept_at='2026-09-14')
        return [box], []

    monkeypatch.setattr(sweep_mod, 'sweep', _fake_sweep)
    site = sweep_mod.st.columns_sites(100.0)[0]
    boxes, _rows = sweep_mod._single_apex_boxes(
        [0.5, 0.6], flight_frac=[-0.1, 0.0, 0.1], halfwidth_m=0.05,
        offsets_mm=[0.0], dwell_s=0.30, separation_mm=100.0, leg_vel=300.0,
        leg_acc=5000.0, leg_jerk=150000.0, hand_acc=3500.0, site=site,
        log=lambda r: None)
    assert len(boxes) == 2
    assert boxes[0].apex_band_m == pytest.approx((0.45, 0.55))
    assert boxes[1].apex_band_m == pytest.approx((0.55, 0.65))
    centre0 = sweep_mod.sc.flight_s(0.5)
    assert calls[0] == pytest.approx([centre0 * 0.9, centre0, centre0 * 1.1])
    # R5 (2026-09-30 fix): `_single_apex_boxes` passes the apex's OWN flight
    # as `pattern_flight_s` on every call -- the schedule timing a ladder
    # rung's fraction must NOT move with the commanded flight.
    centre1 = sweep_mod.sc.flight_s(0.6)
    assert pattern_flight_calls == pytest.approx([centre0, centre1])


# ---------------------------------------------------------------------------
# R5 fix (2026-09-30): `pattern_flight_s` separates the SCHEDULE's timing
# (tau/t_land/t_release, and the incoming ball's arrival velocity) from the
# COMMANDED flight (the carried throw's own ballistic solve) -- see
# `_chained_catch_cell` / `_columns_catch_throw_cell`'s docstrings for the
# full "Why". Before this fix every cell derived BOTH roles from the same
# `flight_s`, so a ladder rung below the pattern apex wrongly shortened the
# schedule window it was judged against and every `--single-apex` band
# collapsed to a point at the pattern centre
# (scratchpad/probe_pattern_flight_timing.py and
# scratchpad/probe_ladder_below_pattern.py, 2026-09-30, confirm both the
# defect and the fix -- see handoff_L.md's probe table).
# ---------------------------------------------------------------------------

def test_chained_catch_cell_timing_is_the_patterns_not_the_commanded(
        sweep_mod, monkeypatch):
    """`_chained_catch_cell`'s CatchTerminal must carry the PATTERN's timing
    (`t_land_s`, `then_throw.t_release_s`) while the carried throw's own
    `then_throw.flight_s` follows the COMMANDED flight -- production's own
    split (`schedule.compile_columns` places every CATCH at
    `t_throw + flight_s(pattern.apex_m)`; `executor._throw_terminal` gives the
    carried throw `flight_s = sch.flight_s(u_apex)`). `sg.plan_segment` is
    stubbed to capture the terminal without a real solve -- this is an
    argument-construction check, not a threshold, so no probe/real-solve is
    needed for it (see the module docstring's Discussion on when a probe is
    required). Confirmed FAILING against the pre-fix body verbatim
    (`scratchpad/probe_pattern_flight_timing.py`, 2026-09-30): the old
    ``_chained_catch_cell`` set ``t_land_s = flight_s`` directly, so this
    assertion would have read ``0.771193`` (Tc) instead of ``0.856881``
    (Tp)."""
    captured = {}

    def _stub(kind, seed, terminal, cfg, limits, geom, **kw):
        captured['terminal'] = terminal
        raise sweep_mod.uc.CycleInfeasible('STUB')

    monkeypatch.setattr(sweep_mod.sg, 'plan_segment', _stub)
    site = st.columns_sites(100.0)[0]
    Tp = sweep_mod.sc.flight_s(0.90)
    Tc = Tp * 0.90  # f = -0.10, a ladder rung below the pattern
    takeoff = np.array([0.0, 0.0, 5000.0])
    sweep_mod._chained_catch_cell(
        site, object(), takeoff, Tc, 0.30, (0.0, 0.0),
        object(), object(), object(), pattern_flight_s=Tp)
    terminal = captured['terminal']
    assert terminal.t_land_s == pytest.approx(Tp)
    assert terminal.then_throw.t_release_s == pytest.approx(Tp + 0.30)
    assert terminal.then_throw.flight_s == pytest.approx(Tc)


def test_columns_catch_throw_cell_timing_is_the_patterns_not_the_commanded(
        sweep_mod, monkeypatch):
    """Same split as the test above, for the cross-site columns cell:
    `tau`/`t_release` are `sc.transit_s(pattern_flight_s, dwell_s)`-derived,
    never the commanded flight's transit."""
    captured = {}

    def _stub(kind, seed, terminal, cfg, limits, geom, **kw):
        captured['terminal'] = terminal
        raise sweep_mod.uc.CycleInfeasible('STUB')

    monkeypatch.setattr(sweep_mod.sg, 'plan_segment', _stub)
    site0, site1 = st.columns_sites(100.0)
    Tp = sweep_mod.sc.flight_s(0.90)
    Tc = Tp * 0.90  # f = -0.10
    takeoff = np.array([0.0, 0.0, 5000.0])
    dwell = 0.30
    sweep_mod._columns_catch_throw_cell(
        site1, object(), takeoff, Tc, dwell, (0.0, 0.0),
        object(), object(), object(), pattern_flight_s=Tp)
    terminal = captured['terminal']
    expected_tau = sweep_mod.sc.transit_s(Tp, dwell)
    assert terminal.t_land_s == pytest.approx(expected_tau)
    assert terminal.then_throw.t_release_s == pytest.approx(expected_tau + dwell)
    assert terminal.then_throw.flight_s == pytest.approx(Tc)
    # The pre-fix body used `sc.transit_s(flight_s, dwell_s)` -- Tc, not Tp --
    # so this would have read `sc.transit_s(Tc, dwell)` instead, a shorter tau.
    assert expected_tau != pytest.approx(sweep_mod.sc.transit_s(Tc, dwell))


def test_single_apex_columns_ladder_admits_a_commanded_apex_below_the_pattern(
        sweep_mod):
    """The whole point of the fix: at the owner's historical leg_jerk=200k
    reference point (the R5 diagnosis's ``runA``,
    ``temp/probes/admissible_box_runA_200k_columns.yaml``), the columns
    ladder around apex 0.90 m must admit commands BELOW 0.90 m -- before the
    fix every band was a single point at the pattern centre (the defect this
    unit closes). Real solve, pinned from a probe
    (``scratchpad/probe_ladder_below_pattern.py``, 2026-09-30, identity offset
    only for wall time -- 1.3 s for the whole 7-rung x 2-direction ladder):
    the f=-0.20 rung (apex 0.576 m, the LOWEST rung tried) already passes at
    zero offset, both directions; f=+0.05/+0.10 (above the pattern) refuse
    ``HAND_LIMIT_ACC`` on the carried/launch throw -- a genuinely asymmetric
    band, not a symmetric one an unfixed sweep could also produce by luck."""
    boxes, rows = sweep_mod._single_apex_boxes(
        [0.90], flight_frac=list(sweep_mod.SINGLE_APEX_FLIGHT_FRAC),
        halfwidth_m=0.05, offsets_mm=[0.0], dwell_s=0.30, separation_mm=100.0,
        leg_vel=300.0, leg_acc=5000.0, leg_jerk=200000.0, hand_acc=3500.0,
        pattern='columns', log=lambda r: None)
    assert len(boxes) == 2
    for box in boxes:
        assert not box.empty
        alo, ahi = box.apex_m
        assert alo == pytest.approx(0.576)
        assert ahi == pytest.approx(0.90)
        assert alo < 0.90, (
            '%r admits no commanded apex below the pattern -- the R5 defect '
            'regressed: a ladder rung is shortening the schedule timing '
            'along with the command again' % (box.site_pair,))
    assert len(rows) >= 2


def test_single_site_sweep_also_gates_the_launch_throw_from_rest(
        sweep_mod, monkeypatch):
    """R3-h2 (2026-09-13): R3's cold-start policy runs single-throw attempts
    (THROW from rest -> CATCH -> REST), so the LAUNCH THROW carries the
    learner's command too -- not only the chained STEADY catch
    ``_chained_catch_cell`` checks. A rehearsal found a warm command inside
    the (unfixed) box refusing ``LIMIT_JERK`` on that launch throw. Spy on
    ``_throw_cell`` and assert it is called with a NONZERO offset at least
    once: if the launch gate regresses to only the shared zero-offset
    per-flight seeding call, this fails.
    """
    calls = []
    orig = sweep_mod._throw_cell

    def _spy(*args, **kwargs):
        calls.append(tuple(kwargs.get('offset_mm', (0.0, 0.0))))
        return orig(*args, **kwargs)

    monkeypatch.setattr(sweep_mod, '_throw_cell', _spy)
    site = st.columns_sites(100.0)[0]
    sweep_mod.sweep(flights_s=(0.8570,), offsets_mm=(-10.0, 0.0),
                    site_pairs=[(site, site)])
    nonzero = [c for c in calls if c != (0.0, 0.0)]
    assert nonzero, (
        'the launch THROW from rest was never planned with a nonzero '
        'offset -- the (P1, P1) box is not gating the segment a cold-start '
        'attempt actually carries the learner command on (R3-h2)')


def test_describe_miss_names_an_apex_outside_every_swept_band():
    """``describe_miss``'s second branch (audit 2026-09-29): no box for the
    pair covers the apex at all -- the message says so and lists what WAS
    swept, and does not blame the site positions."""
    msg = ab.describe_miss([_box()], 'hop', ('P1', 'P2'), 1.10,
                           release_site_xy_mm=_DEFAULT_XY_MM,
                           target_site_xy_mm=_DEFAULT_XY_MM)
    assert 'outside the range' in msg
    assert '0.850-0.950 m' in msg
    assert 'separation_mm' not in msg


def test_describe_miss_names_a_site_separation_mismatch():
    """``describe_miss``'s first branch, called directly (the skill_node path
    is pinned in ``tests/ros/test_skill_node.py``): the apex IS covered, the
    sites are not the swept ones -- both separations are named."""
    msg = ab.describe_miss([_box()], 'hop', ('P1', 'P2'), 0.90,
                           release_site_xy_mm=(-50.0, 0.0),
                           target_site_xy_mm=(50.0, 0.0))
    assert 'IS covered' in msg and 'separation_mm' in msg
    assert '100.0 mm apart' in msg


# ── the committed box agrees with the launch (2026-10-05) ────────────────────

_COMMITTED_BOX = os.path.join(_REPO, 'config', 'generated', 'admissible_box.yaml')


def test_the_committed_box_is_swept_at_the_launch_defaults_and_the_live_gate():
    """R5 sitting 5 (2026-10-05) lost three launches to a box swept at
    350/5000/200000 while the launch default was still 300/5000/150000:
    ``check_limits`` refused every pattern by name until the operator ramped
    ``trajectory/set_limits`` by hand, and nothing in the suite could see the
    divergence because the launch default and the swept limits were pinned in
    two places with only a runsheet row between them. This is the one
    enforcement point: the committed box carries the limits the launch starts
    at (``hw.JB_TRAJ_*`` -- the YAML session defaults) and the gate hash of
    the gated files as committed. Moving the YAML working point, editing a
    gated file, or re-sweeping at other limits without the matching change
    fails here, not at the first JUGGLE goal of a sitting.

    The dwell is pinned separately where ``skill_node``'s default lives
    (``tests/ros/test_skill_node.py``)."""
    boxes = ab.load(_COMMITTED_BOX)
    assert boxes, 'no boxes in the committed admissible_box.yaml'
    launch = TrajectoryLimits.from_config(hw)
    # No gate escape hatch: the committed box must match the committed gate.
    ab.check_limits(boxes, launch, check_gate=True)
    for box in boxes:
        assert box.limits['leg_vel_mmps'] == pytest.approx(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS)
        assert box.limits['leg_acc_mmps2'] == pytest.approx(hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2)
        assert box.limits['leg_jerk_mmps3'] == pytest.approx(hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3)
        assert box.limits['hand_acc_rps2'] == pytest.approx(hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)
        assert box.gate_hash == ab.gate_hash()
