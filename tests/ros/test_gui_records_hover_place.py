# -*- coding: utf-8 -*-
"""Records panel: the hover parameters line (JB-90) and the placement beside the
State Machine card (JB-103).

``tests/ros/js/records_hover_place_harness.js`` runs the real DOM-free
``records-model.js`` under node; the source-scan half pins the wiring a node
harness cannot see (records-panel.js placing the panel, the CSS default and the
dodge class).
"""
from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
JS = GUI / "js"
HARNESS = REPO / "tests" / "ros" / "js" / "records_hover_place_harness.js"


def _find_node():
    found = shutil.which("node") or shutil.which("nodejs")
    if found:
        return found
    for pat in (os.path.expanduser("~/.nvm/versions/node/*/bin/node"), "/usr/local/bin/node", "/usr/bin/node"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[-1]
    return None


NODE = _find_node()


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("recordshoverplace")
    shutil.copy(JS / "records-model.js", sb / "records-model.js")
    shutil.copy(HARNESS, sb / "harness.js")
    (sb / "package.json").write_text('{"type": "module"}')
    res = subprocess.run([NODE, str(sb / "harness.js")], capture_output=True, text=True, timeout=60)
    assert res.returncode == 0, res.stderr
    return json.loads(res.stdout)


def test_hover_title_carries_the_run_parameters_on_its_second_line(out):
    first, second = out["titles"]["columns"].split("\n")
    assert first == "Ended: stopped (STOPPED)"
    assert second == "Apex 0.95 m · Separation 125 mm · Dwell 0.27 s · Planner pose"


def test_record_without_parameters_says_so(out):
    assert out["titles"]["hop"].endswith("\nParameters not recorded")
    assert out["titles"]["oval_1ball"].endswith("\nParameters not recorded")
    assert out["none"] == ["Parameters not recorded"] * 4


def test_parameter_labels_and_units(out):
    assert out["full"] == ("Apex 0.95 m · Separation 125 mm · Dwell 0.27 s · Planner pose"
                           " · Turned angle 30° · Ball 2 apex 0.85 m")
    assert out["labels"] == [["Apex", "m"], ["Separation", "mm"], ["Dwell", "s"],
                             ["Turned angle", "°"], ["Ball 2 apex", "m"], ["Planner", ""],
                             ["Hold", "s"]]


def test_dodge_only_when_the_collapsed_card_reaches_the_panel(out):
    d = out["dodge"]
    assert d["tallPane"] is False      # the usual case: hard left
    assert d["reaches"] is True        # a short pane: the card's top is inside the panel's band
    assert d["withinGap"] is True      # touching within one --gap also dodges
    assert d["expanded"] is False      # expanded: the CSS minimap-expanded rule places it
    assert d["cardHidden"] is False
    assert d["panelHidden"] is False
    assert d["noRects"] is False


def test_panel_places_itself_and_css_defaults_hard_left():
    js = (JS / "records-panel.js").read_text()
    assert "dodgesMinimap(" in js
    assert "classList.toggle('dodge-minimap'" in js
    assert "card.classList.contains('collapsed')" in js
    for node in ("'records-panel'", "'state-minimap'", "'viewer-pane'"):
        assert node in js[js.index("function watchPlacement"):]
    assert "watchPlacement();" in js[js.index("export function initRecordsPanel"):]
    assert "placePanel();" in js[js.index("function render()"):js.index("function placePanel")]
    css = (GUI / "css" / "records-panel.css").read_text()
    base = css[css.index("#records-panel {"):css.index("}", css.index("#records-panel {"))]
    assert "left: var(--gap);" in base
    assert "#records-panel.dodge-minimap { left: calc(var(--gap) * 2 + 210px); }" in css
    # the expanded card still wins (higher specificity than the dodge class)
    assert "#viewer-pane.minimap-expanded #records-panel" in css
