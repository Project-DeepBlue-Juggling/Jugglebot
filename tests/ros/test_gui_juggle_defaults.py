"""Pin the GUI juggle panel's per-pattern apex / separation pre-fill
(``PATTERN_POINTS`` in ros_ws/gui/js/juggle-panel.js) to the committed
admissible box.

Root cause (R5 sitting 7b): the panel flew skill_node's single 0.9 m / 250 mm
default and kept a typed apex across pattern switches, but the box is swept at
a different point per pattern (columns 125 mm / apex band 0.925-0.975, hop
only 250 mm), so operator inputs were refused or silently flew the wrong
geometry.  A pre-fill that drifts out of the box recreates that refusal, so
this test fails if any row is outside it.
"""
from __future__ import annotations

import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
JS = (ROOT / "ros_ws" / "gui" / "js" / "juggle-panel.js").read_text()
BOX = yaml.safe_load((ROOT / "config" / "generated" / "admissible_box.yaml").read_text())
PATTERNS = re.findall(r"^\s*\{ value: '(\w+)'", JS, re.M)


def _points():
    block = re.search(r"export const PATTERN_POINTS = \{(.*?)\n\};", JS, re.S).group(1)
    out = {}
    for m in re.finditer(
            r"(\w+):\s*\{ box: '(\w+)',\s*apex_m: ([\d.]+),\s*separation_mm: (null|[\d.]+) \}", block):
        out[m.group(1)] = (m.group(2), float(m.group(3)),
                           None if m.group(4) == "null" else float(m.group(4)))
    return out


def test_every_panel_pattern_has_a_point():
    assert set(_points()) == set(PATTERNS) and len(PATTERNS) == 5


def test_points_sit_inside_the_committed_box():
    for pattern, (kind, apex, sep) in _points().items():
        rows = [b for b in BOX["boxes"] if b["pattern"] == kind]
        assert rows, pattern
        if sep is None:   # one-site pattern: apex only
            hits = [b for b in rows if b["apex_band_m"][0] <= apex <= b["apex_band_m"][1]]
        else:
            hits = [b for b in rows
                    if abs(2 * abs(b["release_site_xy_mm"][0]) - sep) < 1e-6
                    and b["apex_band_m"][0] <= apex <= b["apex_band_m"][1]]
        assert hits, "%s: apex %.3f / sep %s is not covered by any %s box" % (
            pattern, apex, sep, kind)


def test_panel_switches_points_on_pattern_change():
    assert "applyPatternPoint(ui.pattern);" in JS
    assert JS.count("applyPatternPoint(") >= 4   # def + init + change + reset
