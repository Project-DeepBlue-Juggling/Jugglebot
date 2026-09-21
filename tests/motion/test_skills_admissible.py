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


def _box(**overrides):
    kwargs = dict(
        site_pair=('P1', 'P2'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((-0.02, 0.02), (-0.01, 0.01)), apex_m=(0.87, 0.93),
        limits=dict(_LIMITS_DICT), gate_hash=_GATE_HASH, swept_at='2026-09-12')
    kwargs.update(overrides)
    return ab.AdmissibleBox(**kwargs)


def _empty_box(**overrides):
    kwargs = dict(
        site_pair=('P1', 'P2'), apex_band_m=(0.85, 0.95),
        landing_xy_m=((float('nan'), float('nan')), (float('nan'), float('nan'))),
        apex_m=(float('nan'), float('nan')), limits=dict(_LIMITS_DICT),
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

def test_select_hits_a_box_covering_the_apex():
    box = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95))
    assert ab.select([box], ('P1', 'P1'), 0.90) is box


def test_select_hits_at_the_inclusive_boundary():
    box = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95))
    assert ab.select([box], ('P1', 'P1'), 0.85) is box
    assert ab.select([box], ('P1', 'P1'), 0.95) is box


def test_select_misses_an_apex_outside_the_band():
    box = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95))
    assert ab.select([box], ('P1', 'P1'), 0.50) is None


def test_select_misses_the_right_apex_at_the_wrong_site_pair():
    box = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95))
    assert ab.select([box], ('P1', 'P2'), 0.90) is None


def test_select_picks_the_right_box_among_several_apex_bands():
    lo = _box(site_pair=('P1', 'P1'), apex_band_m=(0.45, 0.55))
    hi = _box(site_pair=('P1', 'P1'), apex_band_m=(0.85, 0.95))
    assert ab.select([lo, hi], ('P1', 'P1'), 0.50) is lo
    assert ab.select([lo, hi], ('P1', 'P1'), 0.90) is hi
    assert ab.select([lo, hi], ('P1', 'P1'), 0.70) is None


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
    alo, ahi = box.apex_m
    # The grid is swept in flight time; the box bounds the APEX (2026-09-18),
    # so the bounds must land inside the swept grid's own apexes.
    assert 0.85 - 1e-9 <= alo <= ahi <= 0.90 + 1e-9
    # Every row actually reached a verdict -- the grid ran, not merely built.
    assert len(rows) >= 6


def test_tiny_sweep_yaml_round_trips_and_validates(tmp_path, tiny_sweep,
                                                   sweep_mod):
    boxes, _rows = tiny_sweep
    path = str(tmp_path / 'admissible_box.yaml')
    ab.dump(path, boxes)
    loaded = ab.load(path)
    assert loaded == boxes
    # "Live" here is the limits the tiny sweep actually ran at. Since 2026-09-21
    # the tool's defaults are the GENERATED launch constants (300/5000/150000),
    # not the 200 000 literal `_live_limits` still carries for the hand-built
    # `_box()` cases above, so the two are read from the same place.
    live = _live_limits(leg_vel_mmps=sweep_mod.LEG_VEL_MMPS,
                        leg_acc_mmps2=sweep_mod.LEG_ACC_MMPS2,
                        leg_jerk_mmps3=sweep_mod.LEG_JERK_MMPS3,
                        hand_acc_rps2=sweep_mod.HAND_ACC_RPS2)
    ab.check_limits(loaded, live)


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

    def _fake_sweep(*, flights_s, offsets_mm, dwell_s, separation_mm, leg_vel,
                    leg_acc, leg_jerk, hand_acc, center_flight_s, site_pairs,
                    log):
        calls.append(list(flights_s))
        site = site_pairs[0][0]
        box = sweep_mod.ab.AdmissibleBox(
            site_pair=(site.name, site.name), apex_band_m=(0.0, 99.0),
            landing_xy_m=((-0.01, 0.01), (-0.01, 0.01)),
            apex_m=(sweep_mod.sc.apex_m(min(flights_s)),
                    sweep_mod.sc.apex_m(max(flights_s))),
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
