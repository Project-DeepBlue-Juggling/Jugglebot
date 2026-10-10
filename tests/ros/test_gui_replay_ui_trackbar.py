# -*- coding: utf-8 -*-
"""Replay UI unit 2 (Phase 3): trackbar, transport, hotkeys, overview, zoom.

``tests/ros/js/replay_ui_trackbar_harness.js`` runs the real ``replay/ui/{dom,overview,trackbar}.js`` under node
with a minimal fake DOM, a fake engine (state object + recorded calls) and a fake mode, then prints JSON this
file asserts on. Times in the harness output are seconds relative to the recording start where noted.
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
UI = REPO / "ros_ws" / "gui" / "js" / "replay" / "ui"
HARNESS = REPO / "tests" / "ros" / "js" / "replay_ui_trackbar_harness.js"


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
pytestmark = pytest.mark.skipif(NODE is None, reason="node not installed")


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    sb = tmp_path_factory.mktemp("uitrackbar")
    (sb / "replay" / "ui").mkdir(parents=True)
    for name in ("dom.js", "overview.js", "trackbar.js"):
        shutil.copy(UI / name, sb / "replay" / "ui" / name)
    shutil.copy(HARNESS, sb / "replay_ui_trackbar_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "replay_ui_trackbar_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


def test_mount_follows_mode_state_events(out):
    assert out["idle"] == {"mounted": False, "dock_kids": 0, "keydown": 0}
    # OPENING (no engine yet) already mounts the bar; hotkey listener installed exactly once
    assert out["opening"] == {"mounted": True, "dock_kids": 1, "keydown": 1}
    assert out["replay"]["keydown"] == 1 and out["replay"]["mousemove"] == 1


def test_unmount_removes_listeners_and_dock_content(out):
    e = out["exit"]
    assert e["mounted"] is False and e["dock_kids"] == 0
    assert e["keydown"] == 0 and e["mousemove"] == 0 and e["mouseup"] == 0
    assert e["key_calls"] == 0                      # hotkeys are dead outside replay
    assert out["remount"] == {"mounted": True, "kids": 1}


def test_readouts_for_playhead_and_range(out):
    r = out["readouts"]
    assert r["clock"] == r["expect_clock"]          # wall-clock time of day HH:MM:SS
    assert r["elapsed"] == "00:10 / 01:40"
    assert r["speed"] == "×1" and r["play_btn"] == "Play"
    assert r["head_left"] == "10.00%" and r["conv_w"] == "100.00%"
    assert r["date"] == "2026-10-09"
    assert r["buffering_hidden"] is True and r["front_hidden"] is True


def test_frontier_fill_and_buffering_flag_follow_state(out):
    p = out["progressive"]
    assert p["buffering_hidden"] is False           # engine mode 'buffering'
    assert p["front_hidden"] is False               # source still converting: frontier marker shown
    assert p["elapsed"] == "00:40 / ?"              # duration unknown until complete
    assert p["play_btn"] == "Pause"                 # buffering counts as playing intent


def test_buttons_map_to_engine_calls(out):
    b = out["buttons"]
    assert b["play"] == ["play"] and b["pause"] == ["pause"]
    assert b["home"] == ["seek:1791532800"] and b["end"] == ["seek:1791532900"]
    assert b["stepb"] == ["step:-1"] and b["stepf"] == ["step:1"]
    assert b["exit"] == ["exitReplay:closed"]


def test_speed_ladder_and_reverse(out):
    lad = out["ladder"]
    assert lad["ff_from_1"] == [2, 4, 8, 8, 8]                 # x1 -> x2 -> x4 -> x8, capped
    assert lad["rw_from_1"] == [-1, -2, -4, -8, -8, -8]        # RW enters reverse at x1, then faster
    assert lad["ff_from_rev"] == [-1, -0.5, -0.25, 1]          # FF slows the reverse, then returns forward
    assert lad["ff_click"] == ["setSpeed:2", "play"]
    assert lad["rw_click"] == ["setSpeed:-1", "play"]


def test_hotkeys_map_to_calls_and_space_prevents_default(out):
    k = out["keys"]
    assert k["space_play"] == {"calls": ["play"], "prevented": True}
    assert k["space_pause"]["calls"] == ["pause"]
    assert k["left"]["calls"] == ["seek:49"] and k["right"]["calls"] == ["seek:51"]
    assert k["sleft"]["calls"] == ["seek:40"] and k["sright"]["calls"] == ["seek:60"]
    assert k["comma"]["calls"] == ["step:-1"] and k["period"]["calls"] == ["step:1"]
    assert k["home"]["calls"] == ["seek:0"] and k["end"]["calls"] == ["seek:100"]
    assert k["right_clamped"]["calls"] == ["seek:100"] and k["left_clamped"]["calls"] == ["seek:0"]
    assert k["unrelated"] == {"calls": [], "prevented": False}


def test_hotkeys_ignored_in_text_inputs_and_with_modifiers(out):
    assert out["keys"]["in_input"] == {"calls": 0, "prevented": False}
    assert out["keys"]["ctrl"] == 0


def test_drag_scrubs_then_release_seeks(out):
    # bar spans x=100..1100 over 0..100 s: 600 -> 50 s, 700 -> 60 s, release at 800 -> 70 s
    assert out["drag"] == ["scrub:50", "scrub:60", "seek:70"]


def test_zoom_keys_about_playhead_and_reset(out):
    assert out["zoom_in"]["view"] == {"v0": 25, "v1": 75}
    assert "Esc resets" in out["zoom_in"]["tag"]               # a zoomed bar labels its range edges
    assert out["zoom_out"] is None and out["esc"] is None and out["dbl"] is None


def test_overview_drag_select_zooms_and_click_seeks(out):
    assert out["drag_select"] == {"v0": 20, "v1": 40}
    assert out["drag_select_ticks"] == ["skill attempt (hop) @ " + out["drag_select_ticks"][0].split("@ ")[1]]
    assert out["ov_click"] == ["seek:30"]                       # x=500 of 1000 over the 20..40 view -> 30 s


def test_zoom_range_math_clamped_to_recording(out):
    m = out["math"]
    assert m["clamp_lo"] == {"v0": 0, "v1": 20} and m["clamp_hi"] == {"v0": 80, "v1": 100}
    assert m["clamp_all"] == {"v0": 0, "v1": 100}
    assert m["clamp_min"] == {"v0": 50, "v1": 52}               # MIN_SPAN
    assert m["zoom_about"] == {"v0": 12.5, "v1": 62.5}          # playhead 25 keeps its screen fraction
    assert m["zoom_edge"] == {"v0": 0, "v1": 25}
    assert m["zoom_out"] == {"v0": 30, "v1": 70}
    assert m["drag"] == {"v0": 20, "v1": 40} and m["drag_rev"] == {"v0": 60, "v1": 70}
    assert m["drag_click"] is None


def test_overview_placement_at_two_zooms(out):
    m = out["math"]
    assert m["bands_full"] == [["IDLE", 0, 0.2], ["ACTIVE", 0.2, 0.4], ["FAULT", 0.6, 0.4]]
    assert m["bands_zoom"] == [["IDLE", 0, 0.1667], ["ACTIVE", 0.1667, 0.6667], ["FAULT", 0.8333, 0.1667]]
    # homed is never a tick; only the five drawn kinds
    assert m["ticks_full"] == [["skill_attempt", 0.3], ["catch_event", 0.5], ["fault", 0.8]]
    assert m["ticks_zoom"] == [["skill_attempt", 0.1667], ["catch_event", 0.8333]]
    r = out["readouts"]
    assert r["bands"] == 3 and r["ticks"] == 3
    assert r["tick_titles"][0].startswith("skill attempt (hop) @ ")


def test_trackbar_source_state_names_are_the_real_vocabulary():
    """Frontier-marker logic compares source.status().state to names the backend/sources.js really emit:
    schema STATUS_* plus 'none' (api status route, no manifest yet) and 'live' (session source)."""
    import re
    import sys
    sys.path.insert(0, str(REPO / "ros_ws" / "gui"))
    from replay import schema
    src = (UI / "trackbar.js").read_text()
    used = set(re.findall(r"sstate\s*[!=]==\s*'(\w+)'", src))
    real = {schema.STATUS_CONVERTING, schema.STATUS_COMPLETE, schema.STATUS_FAILED, "none", "live"}
    assert used and used <= real, used - real
    harness = HARNESS.read_text()
    assert "srcState = 'converting'" in harness and "srcState = 'complete'" in harness


def test_space_does_not_double_fire_the_live_chart_pause(out):
    """Audit W1: the trackbar listens in the capture phase and stops handled keys, so the live GUI's bubble-phase
    Space shortcut (chart pause) never also fires; unhandled keys and keys typed in inputs still reach it."""
    d = out["space_double_fire"]
    assert d["trackbar_calls"] == ["play"] and d["prevented"] is True
    assert d["live_pause"] == 0 and d["live_unhandled_q"] == 1
    assert d["live_pause_in_input"] == 1


def test_failing_overview_is_not_refetched_every_frame(out):
    """Audit W3: a partial/failed /overview is retried on a frame-count backoff, not per animation frame."""
    assert out["overview_refetch"]["fetches"] <= 2
