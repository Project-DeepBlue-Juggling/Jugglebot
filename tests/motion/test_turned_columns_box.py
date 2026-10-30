"""JP-10 turned Columns: straight and turned cells coexist in one box YAML and
are never selected for each other's pair; the sweep tool plumbs ``--angle-deg``.

Unmarked, parallel-safe (``tmp_path`` only, no sweep is run).
"""

from __future__ import annotations

import os
import sys

import pytest

from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import sites as st

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'tools'))
import admissible_sweep as sweep_tool  # noqa: E402


def _box(site, planner_mode='pose'):
    xy = (float(site.cup_mm[0]), float(site.cup_mm[1]))
    return adm.AdmissibleBox(
        site_pair=(site.name, site.name), apex_band_m=(0.875, 0.925),
        landing_xy_m=((-0.05, 0.05), (-0.05, 0.05)), apex_m=(0.6, 1.2),
        pattern='columns', release_site_xy_mm=xy, target_site_xy_mm=xy,
        limits={'leg_vel_mmps': 600.0, 'leg_acc_mmps2': 5000.0,
               'leg_jerk_mmps3': 200000.0, 'hand_acc_rps2': 30.0},
        gate_hash=adm.gate_hash(), swept_at='2026-10-30',
        dwell_s=0.27, planner_mode=planner_mode)


def _select(boxes, site):
    xy = site.cup_mm[:2]
    return adm.select(boxes, 'columns', (site.name, site.name), 0.9,
                      release_site_xy_mm=xy, target_site_xy_mm=xy,
                      dwell_s=0.27, planner_mode='pose')


def test_straight_and_turned_cells_coexist_and_never_cross_select(tmp_path):
    straight = st.columns_sites(125.0)
    turned = st.columns_sites(125.0, 45.0)
    boxes = [_box(s) for s in straight + turned]
    path = str(tmp_path / 'box.yaml')
    adm.dump(path, boxes)                 # the overlap check must not fire
    loaded = adm.load(path)
    for s in straight + turned:
        got = _select(loaded, s)
        assert got is not None and got.site_pair == (s.name, s.name)
    # A turned pair never matches a straight cell, nor the reverse -- even
    # if someone reused the straight NAME at the turned xy.
    clash = st.Site('P1', turned[0].cup_mm)
    assert _select(loaded, clash) is None


def test_same_name_cells_at_different_xy_would_collide_in_one_yaml(tmp_path):
    """The reason turned Sites are renamed: names, not xy, key the file."""
    a = _box(st.columns_sites(125.0)[0])
    b = _box(st.Site('P1', st.columns_sites(125.0, 45.0)[0].cup_mm))
    with pytest.raises(Exception):
        adm.dump(str(tmp_path / 'box.yaml'), [a, b])


def test_sweep_cli_plumbs_angle_deg_to_the_columns_sweep(monkeypatch, tmp_path):
    seen = {}

    def fake_single(apexes, **kw):
        seen.update(kw)
        return [], []

    monkeypatch.setattr(sweep_tool, '_single_apex_boxes', fake_single)
    monkeypatch.setattr(sweep_tool.ab, 'dump', lambda *a, **k: None)
    rc = sweep_tool.main(['--site-pairs', 'columns', '--single-apex', '0.9',
                          '--angle-deg', '45', '--planner-mode', 'pose',
                          '--quiet', '--out', str(tmp_path / 'x.yaml')])
    assert rc == 0 and seen['angle_deg'] == 45.0
    seen.clear()
    sweep_tool.main(['--site-pairs', 'columns', '--single-apex', '0.9',
                     '--quiet', '--out', str(tmp_path / 'x.yaml')])
    assert seen['angle_deg'] == 0.0


def test_sweep_cli_refuses_angle_with_level_or_other_families(tmp_path):
    for argv in (['--site-pairs', 'columns', '--angle-deg', '45',
                  '--planner-mode', 'level'],
                 ['--site-pairs', 'hop', '--angle-deg', '45',
                  '--planner-mode', 'pose']):
        with pytest.raises(SystemExit):
            sweep_tool.main(argv + ['--out', str(tmp_path / 'x.yaml')])
