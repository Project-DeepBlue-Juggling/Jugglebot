# -*- coding: utf-8 -*-
"""Static-markup contract for the Robot | Map tab bar (ros_ws/gui/js/app-tabs.js)."""
from __future__ import annotations

from pathlib import Path

GUI = Path(__file__).resolve().parents[2] / "ros_ws" / "gui"
HTML = (GUI / "index.html").read_text()
JS = (GUI / "js" / "app-tabs.js").read_text()
CSS = (GUI / "css" / "app-tabs.css").read_text()


def test_index_has_tablist_with_robot_and_map_tabs():
    assert 'id="app-tabs" role="tablist"' in HTML
    assert 'role="tab" data-tab="robot"' in HTML and 'role="tab" data-tab="map"' in HTML
    assert 'id="map-pane"' in HTML


def test_index_loads_tab_script_and_css():
    assert 'js/app-tabs.js' in HTML and 'css/app-tabs.css' in HTML


def test_map_iframe_is_lazy_and_never_destroyed():
    assert "createElement('iframe')" in JS
    assert "mapCreated" in JS
    assert "removeChild(frame" not in JS and "frame.remove" not in JS


def test_bar_steals_height_from_app_and_is_slim():
    assert "calc(100vh - var(--app-tabs-height))" in CSS
    assert "--app-tabs-height: 32px" in CSS


def test_create_map_clears_pane_and_uses_no_local_storage():
    assert "pane.replaceChildren()" in JS
    assert "localStorage" not in JS


def test_app_is_inert_while_map_shows_and_never_hidden():
    assert "appEl.inert" in JS
    assert "appEl.hidden" not in JS and "'app').hidden" not in JS
    assert "appEl.style" not in JS and "display = 'none'" not in JS and "display: none" not in JS


def test_initial_tab_is_robot_unless_hash_map():
    assert "hash === 'map' ? 'map' : 'robot'" in JS
