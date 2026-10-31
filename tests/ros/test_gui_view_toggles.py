# -*- coding: utf-8 -*-
"""View-menu scene toggles persist across a refresh (JB-89).

``tests/ros/js/view_toggles_harness.js`` runs the real DOM-free
``ros_ws/gui/js/view-toggles.js`` under node; the source-text half pins that
``main.js`` loads on build and saves on every change (no browser pass).
"""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
JS = REPO / "ros_ws" / "gui" / "js"
HARNESS = REPO / "tests" / "ros" / "js" / "view_toggles_harness.js"
NODE = shutil.which("node") or shutil.which("nodejs")


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("viewtoggles")
    shutil.copy(JS / "view-toggles.js", sb / "view-toggles.js")
    shutil.copy(HARNESS, sb / "harness.js")
    (sb / "package.json").write_text('{"type": "module"}')
    res = subprocess.run([NODE, str(sb / "harness.js")], capture_output=True, text=True, timeout=60)
    assert res.returncode == 0, res.stderr
    return json.loads(res.stdout)


def test_round_trip_and_single_json_key(out):
    assert out["key"] == "jugglebot-view-toggles"
    assert json.loads(out["stored"]) == {"Jugglebot model": False, "Ball": True}
    assert out["roundTrip"] == {"Jugglebot model": False, "Ball": True}


def test_absent_garbage_and_wrong_shapes_give_no_saved_choice(out):
    assert out["absent"] == {}
    assert out["garbage"] == {}
    assert out["array"] == {}
    assert out["nullStorage"] == {}
    assert out["throwingLoad"] == {}
    assert out["nonBool"] == {"b": False}


def test_blocked_storage_save_is_not_an_error(out):
    assert out["saveThrew"] is False


def test_saved_choice_beats_scene_default_else_default_holds(out):
    assert out["savedOff"] is False
    assert out["savedOn"] is True
    assert out["defaultTrue"] is True
    assert out["defaultFalse"] is False
    assert out["protoName"] is False


def test_main_js_wires_load_and_save():
    src = (JS / "main.js").read_text()
    assert "from './view-toggles.js'" in src
    assert "loadViewToggles(" in src
    assert "toggleState(viewSaved, name, obj.visible)" in src
    assert "saveViewToggles(viewStore, viewSaved)" in src
    # the save sits inside the checkbox change handler, after the scene is updated
    change = src.index("checkbox.addEventListener('change'", src.index("function initSceneMenu"))
    assert change < src.index("saveViewToggles(viewStore, viewSaved)") < src.index("legend.className")
