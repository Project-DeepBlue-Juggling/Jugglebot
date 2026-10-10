# -*- coding: utf-8 -*-
"""Replay hide-input-panels (owner ask 2026-10-10): runs the real replay/ui/hide.js; source contracts for the juggle panel."""
from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
UI = GUI / "js" / "replay" / "ui"
HARNESS = REPO / "tests" / "ros" / "js" / "replay_ui_hide_harness.js"
NODE = shutil.which("node") or shutil.which("nodejs")
SELECTORS = ["#minimap-juggle", "#bb-throw-content", "#panel-jog", "#panel-speed-limits"]


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("uihide")
    (sb / "replay" / "ui").mkdir(parents=True)
    shutil.copy(UI / "hide.js", sb / "replay" / "ui" / "hide.js")
    shutil.copy(HARNESS, sb / "h.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    p = subprocess.run([NODE, str(sb / "h.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert p.returncode == 0, p.stderr.decode()
    return json.loads(p.stdout.decode().strip().splitlines()[-1])


def test_selector_set_is_the_four_regions(out):
    assert sorted(out["selectors"]) == sorted(SELECTORS)


def test_four_selectors_exist_in_sources():
    html = (GUI / "index.html").read_text()
    minimap = (GUI / "js" / "state-minimap.js").read_text()
    assert 'id="panel-jog"' in html and 'id="panel-speed-limits"' in html and 'id="bb-throw-content"' in html
    assert "juggle.id = 'minimap-juggle'" in minimap


def test_hidden_on_entry_and_restored_on_exit(out):
    assert out["initial"] == []
    assert out["onHidden"] == sorted(SELECTORS)
    assert out["offHidden"] == [] and out["hiddenFlag"] is False


def test_late_built_element_is_hidden_and_bb_status_kept(out):
    assert "#minimap-juggle" in out["onHidden"]      # element appeared after construction, hidden on the next state event
    assert out["bbKept"] is True


def test_inline_display_untouched_so_restore_is_exact(out):
    assert out["jogInline"] == "none" and out["jogInlineAfter"] == "none"


def test_juggle_hook_called_once_per_transition(out):
    assert out["onCalls"] == [True] and out["repeatCalls"] == [True] and out["offCalls"] == [True, False]


def test_css_class_is_display_none_important():
    css = (GUI / "css" / "replay.css").read_text()
    assert re.search(r"\.replay-hidden\s*\{\s*display:\s*none\s*!important", css)


def test_juggle_panel_gates_render_and_raf_on_replay_hidden():
    src = (GUI / "js" / "juggle-panel.js").read_text()
    assert re.search(r"export function setJugglePanelReplayHidden\(", src)
    m = re.search(r"export function renderJugglePanel\(g\) \{(.*?)\n    if \(!dom", src, re.S)
    assert m and "if (replayHidden) return;" in m.group(1)
    assert re.search(r"function animShouldRun\(\) \{[^}]*!replayHidden", src, re.S)
    assert re.search(r"if \(on\) \{\s*if \(rafId\) \{ cancelAnimationFrame\(rafId\); rafId = 0; \}", src)
    assert re.search(r"else \{\s*if \(lastGate\) renderJugglePanel\(lastGate\);\s*ensureAnim\(\);", src)


def test_index_wires_hide_to_juggle_hook():
    src = (UI / "index.js").read_text()
    assert "createHide" in src and "setJuggleHidden: setJugglePanelReplayHidden" in src
