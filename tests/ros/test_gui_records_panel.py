# -*- coding: utf-8 -*-
"""Records panel (``ros_ws/gui/js/records-panel.js`` + ``records-model.js``, JB-35).

``tests/ros/js/records_panel_harness.js`` runs the real DOM-free model under
node and prints JSON this file asserts on: the row model from the
``/api/records`` JSON, the View-menu visibility filter, the localStorage
round trip (and its failure modes), and the juggle_end ``record_beaten``
read. The source-scan half pins the wiring a node harness cannot see: the
markup and CSS link in ``index.html``, the init and View-menu calls in
``main.js``, the skills/attempt forward in ``state-minimap.js``, the fetch
route and the replay fence.
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
HARNESS = REPO / "tests" / "ros" / "js" / "records_panel_harness.js"


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
    sb = tmp_path_factory.mktemp("recordspanel")
    shutil.copy(JS / "records-model.js", sb / "records-model.js")
    shutil.copy(HARNESS, sb / "harness.js")
    (sb / "package.json").write_text('{"type": "module"}')
    res = subprocess.run([NODE, str(sb / "harness.js")], capture_output=True, text=True, timeout=60)
    assert res.returncode == 0, res.stderr
    return json.loads(res.stdout)


def test_one_row_per_pattern_in_ledger_order(out):
    rows = out["all"]
    assert [r["pattern"] for r in rows] == ["hop", "self_toss", "columns"]
    hop, st, col = rows
    assert (hop["score"], hop["date"], hop["empty"], hop["title"]) == ("—", "", True, "No record yet")
    assert (st["label"], st["score"], st["date"], st["empty"]) == ("Self-throw", "12/14", "2026-10-21", False)
    assert st["title"].startswith("12 Catches in 14 Throws, 2026-10-21 10:05, drop (MISSED_CATCH)")
    assert "warm box" in st["title"] and "beaten 2× before" in st["title"]
    assert (col["score"], col["balls"]) == ("32/33", 2)


def test_hidden_patterns_drop_out_and_garbage_renders_nothing(out):
    assert out["filtered"] == ["self_toss"]
    assert out["garbage"] == [[], [], []]


def test_view_menu_entries_and_storage(out):
    assert out["menu"] == [
        {"pattern": "hop", "label": "Records: Hop", "checked": True},
        {"pattern": "self_toss", "label": "Records: Self-throw", "checked": True},
        {"pattern": "columns", "label": "Records: Columns", "checked": False},
    ]
    assert json.loads(out["stored"]) == ["columns", "self_toss"]
    assert sorted(out["reloaded"]) == ["columns", "self_toss"]
    assert out["badJson"] == out["notArray"] == out["blocked"] == []   # unreadable = all shown
    assert out["blockedSave"] == "ok"


def test_record_beaten_is_read_from_juggle_end_only(out):
    assert out["ends"] == [
        {"pattern": "columns", "beaten": True},
        {"pattern": "hop", "beaten": False},
        {"pattern": "hop", "beaten": False},       # an older skill_node: no field, no flash
        None, None,
    ]


# ── Source scan: the wiring ────────────────────────────────────────────────

def test_index_html_carries_the_panel_and_its_stylesheet():
    html = (GUI / "index.html").read_text()
    assert '<link rel="stylesheet" href="css/records-panel.css">' in html
    assert html.index('id="connection-status"') < html.index('id="records-panel"') < html.index('id="scene-menu"')
    assert (GUI / "css" / "records-panel.css").exists()


def test_main_inits_the_panel_and_its_view_menu_section():
    main = (JS / "main.js").read_text()
    assert "import { initRecordsPanel, attachRecordsMenu } from './records-panel.js';" in main
    assert "initRecordsPanel();" in main
    assert main.index("initCameraPresets();\n        attachRecordsMenu(dropdown);") > 0


def test_juggle_end_reaches_the_panel_through_the_minimap_forward():
    mm = (JS / "state-minimap.js").read_text()
    assert "import { recordsPanelOnSkillAttempt } from './records-panel.js';" in mm
    assert "    jugglePanelOnSkillAttempt(msg);\n    recordsPanelOnSkillAttempt(msg);\n" in mm


def test_panel_fetches_the_route_and_respects_the_replay_fence():
    panel = (JS / "records-panel.js").read_text()
    assert "fetch('/api/records'" in panel
    assert "isReplayFenced()" in panel
    assert "body.replay #records-panel { display: none; }" in (GUI / "css" / "replay.css").read_text()
