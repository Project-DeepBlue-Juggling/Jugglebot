"""``motion/skills/admissible`` -- the offline admissible box (plan § 2.6).

Round-trip (dump/load), :func:`admissible.clip`, and the two loader refusals
(a limits mismatch, a gate-hash mismatch) are pure-Python and unmarked. The
one end-to-end case runs the REAL sweep (``tools/admissible_sweep.py``) on a
tiny 2x2 grid -- module-scoped, ~10 QP solves, the same shape
``tests/motion/test_skills_segments.py`` uses for its own real-solve fixtures.

Plan: ``plans/active/two-ball-skill-stack.md`` § 2.6.
"""

from __future__ import annotations

import importlib.util
import math
import os

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.skills import admissible as ab
from jugglebot.motion.skills import schedule as sc
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


def _box(**overrides):
    kwargs = dict(
        site_pair=('P1', 'P2'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((-0.02, 0.02), (-0.01, 0.01)), flight_s=(0.83, 0.88),
        limits=dict(_LIMITS_DICT), gate_hash=_GATE_HASH, swept_at='2026-09-12')
    kwargs.update(overrides)
    return ab.AdmissibleBox(**kwargs)


def _empty_box(**overrides):
    kwargs = dict(
        site_pair=('P1', 'P2'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((float('nan'), float('nan')), (float('nan'), float('nan'))),
        flight_s=(float('nan'), float('nan')), limits=dict(_LIMITS_DICT),
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
    """landing_xy_m and flight_s must be empty (nan) TOGETHER, never one alone
    -- a half-empty box is a bug in whatever built it, not a physical state."""
    with pytest.raises(ValueError, match='empty'):
        _box(flight_s=(float('nan'), float('nan')))


def test_box_rejects_missing_limit_keys():
    with pytest.raises(ValueError, match='leg_vel_mmps'):
        _box(limits={'leg_acc_mmps2': 5000.0, 'leg_jerk_mmps3': 200000.0,
                     'hand_acc_rps2': 3500.0})


def test_box_rejects_a_malformed_gate_hash():
    with pytest.raises(ValueError, match='gate_hash'):
        _box(gate_hash='not-hex!!')


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
    u = (np.array([0.005, -0.002]), 0.857)
    xy, flight = ab.clip(u, box)
    np.testing.assert_allclose(xy, [0.005, -0.002])
    assert flight == pytest.approx(0.857)


def test_clip_outside_the_box_saturates_to_the_nearest_edge():
    box = _box()
    u = (np.array([1.0, -1.0]), 5.0)
    xy, flight = ab.clip(u, box)
    np.testing.assert_allclose(xy, [0.02, -0.01])
    assert flight == pytest.approx(0.88)


def test_clip_on_an_empty_box_refuses_naming_the_site_pair():
    box = _empty_box(site_pair=('P2', 'P1'))
    with pytest.raises(ab.AdmissibleError, match='P2'):
        ab.clip((np.array([0.0, 0.0]), 0.857), box)


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


@pytest.fixture(scope='module')
def tiny_sweep(sweep_mod):
    site0, site1 = st.columns_sites(100.0)
    boxes, rows = sweep_mod.sweep(
        apexes_m=(0.85, 0.90), offsets_mm=(-10.0, 0.0), site_pairs=[(site0, site1)])
    return boxes, rows


def test_tiny_sweep_produces_one_box_inside_the_swept_grid(tiny_sweep):
    boxes, rows = tiny_sweep
    assert len(boxes) == 1
    box = boxes[0]
    assert box.site_pair == ('P1', 'P2')
    assert not box.empty, 'the owner operating point (0.90 m apex) must admit ' \
                          'at least the identity-prior (0, 0) command'
    (xlo, xhi), (ylo, yhi) = box.landing_xy_m
    assert -0.010 - 1e-9 <= xlo <= xhi <= 0.010 + 1e-9
    assert -0.010 - 1e-9 <= ylo <= yhi <= 0.010 + 1e-9
    flo, fhi = box.flight_s
    swept_flights = [sc.flight_s(a) for a in (0.85, 0.90)]
    assert min(swept_flights) - 1e-9 <= flo <= fhi <= max(swept_flights) + 1e-9
    # Every row actually reached a verdict -- the grid ran, not merely built.
    assert len(rows) >= 6


def test_tiny_sweep_yaml_round_trips_and_validates(tmp_path, tiny_sweep):
    boxes, _rows = tiny_sweep
    path = str(tmp_path / 'admissible_box.yaml')
    ab.dump(path, boxes)
    loaded = ab.load(path)
    assert loaded == boxes
    live = _live_limits()
    ab.check_limits(loaded, live)
