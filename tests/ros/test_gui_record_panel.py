# -*- coding: utf-8 -*-
"""Wave camera record panel (``ros_ws/gui/js/camera-panel.js``; s8 of
ros_ws/docs/wave_camera_can_adapter.md).

``tests/ros/js/record_panel_harness.js`` runs the real module under node, with
``replay/fence.js`` and ``replay/ui/format.js`` copied verbatim and a fake
``ros-bridge.js`` (connection state + a scriptable ``callService``), then prints
JSON this file asserts on: the enable truth table, the status text per state,
fault names, the elapsed interpolation (+1 s clamp = the stale freeze), and the
DOM glue (in-flight, failure flash, replay fence, disconnect blank).

The source-scan half pins the wiring a node harness cannot see: the
``camera/heartbeat`` subscribe literal, its ``GUI_SUBSCRIBED_TOPICS`` entry,
the init call, the disconnect blank, the CSS link and the markup.
"""
from __future__ import annotations

import glob
import json
import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
JS = GUI / "js"
HARNESS = REPO / "tests" / "ros" / "js" / "record_panel_harness.js"

FAKE_ROS = """\
export function getConnectionState() { return globalThis.__fakeRos.state; }
export function callService(name, type, req) { return globalThis.__fakeRos.callService(name, type, req); }
export function withTimeout(p) { return p; }
"""


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
needs_node = pytest.mark.skipif(NODE is None, reason="node not installed")


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("recordpanel")
    (sb / "replay" / "ui").mkdir(parents=True)
    shutil.copy(JS / "camera-panel.js", sb / "camera-panel.js")
    shutil.copy(JS / "replay" / "fence.js", sb / "replay" / "fence.js")
    shutil.copy(JS / "replay" / "ui" / "format.js", sb / "replay" / "ui" / "format.js")
    (sb / "ros-bridge.js").write_text(FAKE_ROS)
    shutil.copy(HARNESS, sb / "record_panel_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "record_panel_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


# ---------------------------------------------------------------- core

@needs_node
def test_enable_truth_table(out):
    e = out["enable"]
    assert e["idle"] is True and e["recording"] is True
    assert e["unsynced_and_faulted_idle"] is True     # warnings never disable; only the listed gates do
    for k in ("starting", "stopping", "boot", "fault", "adapter_down", "camera_off", "console_owns",
              "ros_down", "fenced", "pending", "no_hb", "stale"):
        assert e[k] is False, k
    assert e["just_fresh"] is True                     # exactly 1000 ms old is still fresh; > 1000 is stale


@needs_node
def test_status_text_per_state(out):
    s = out["status"]
    assert s["no_hb"] == "No heartbeat"
    assert s["idle0"] == "Idle · 0 clips"
    assert s["idle1"] == "Idle · 1 clip"
    assert s["idle3"] == "Idle · 3 clips"
    assert s["recording"] == "Recording"
    assert s["starting"] == "Starting…" and s["stopping"] == "Stopping…"
    assert s["boot"] == "Booting"
    assert s["fault"] == "Fault: TICK_LOST_MID_REC, INTENT_TIMEOUT_AUTOSTOP"
    assert s["fault_none"] == "Fault: unknown"
    assert s["adapter_down"] == "Adapter disconnected"   # precedes camera-off
    assert s["camera_off"] == "Camera off"               # precedes the FAULT state
    assert s["faults_alias"] == "Fault: GPI2_REFUSED"    # `faults` accepted as well as the design doc's `fault_bits`


@needs_node
def test_fault_names_and_warnings(out):
    f = out["faultNames"]
    assert f["all"] == ["GPI2_REFUSED", "TICK_LOST_MID_REC", "CLIP_MISSING", "INTENT_TIMEOUT_AUTOSTOP",
                        "BUS_OFF_SEEN", "SYNC_STEP", "EDGE_RING_OVERFLOW"]
    assert f["none"] == [] and f["b0"] == ["GPI2_REFUSED"] and f["b5b6"] == ["SYNC_STEP", "EDGE_RING_OVERFLOW"]
    assert f["b7"] == ["BIT7"]
    w = out["warn"]
    assert w["clean"] == [] and w["adapter_down"] == []
    assert len(w["unsynced"]) == 1 and "not synced" in w["unsynced"][0] and "25500" in w["unsynced"][0]
    assert w["faults"] == ["Faults: GPI2_REFUSED, INTENT_TIMEOUT_AUTOSTOP"]
    assert len(w["console"]) == 1 and "Console owns" in w["console"][0]


@needs_node
def test_elapsed_interpolation_and_clamp(out):
    e = out["elapsed"]
    assert e["at_hb"] == 61500 and e["plus250"] == 61750 and e["plus999"] == 62499
    assert e["plus5000"] == 62500          # clamped at +1 s: the stale freeze
    assert e["before_hb"] == 61500
    assert e["idle"] == 0 and e["none"] is None
    f = out["fmt"]
    assert f == {"t0": "00:00", "t999": "00:00", "t61500": "01:01", "t_hour": "1:01:02", "t_null": "--:--"}


@needs_node
def test_view_stale_freeze_snap_and_blank(out):
    v = out["view"]
    assert v["fresh"]["elapsedText"] == "01:02" and v["fresh"]["stale"] is False
    assert v["fresh"]["label"] == "STOP" and v["fresh"]["recording"] is True and v["fresh"]["enabled"] is True
    assert v["stale"]["stale"] is True and v["stale"]["enabled"] is False
    assert v["stale"]["elapsedText"] == v["stale_later"]["elapsedText"] == "01:02"   # frozen
    assert v["stale"]["statusText"] == "Recording (stale)"
    assert v["snapped"]["elapsedText"] == "01:02" and v["snapped"]["stale"] is False
    for k in ("ros_down", "no_hb"):
        assert v[k]["blank"] is True and v[k]["elapsedText"] == "--:--" and v[k]["enabled"] is False, k
    assert v["ros_down"]["statusText"] == "ROS disconnected"
    assert v["no_hb"]["statusText"] == "No heartbeat"
    assert v["pending"]["label"] == "…" and v["pending"]["enabled"] is False
    assert v["flash"]["statusText"] == "Start failed: camera off"
    assert v["idle_warn"]["warn"] is True and v["idle_warn"]["enabled"] is False
    assert v["fresh"]["warn"] is False


# ---------------------------------------------------------------- glue

@needs_node
def test_glue_repaints_at_4hz_and_snaps_on_heartbeat(out):
    g = out["glue"]
    assert g["intervals"] == [250]
    assert g["initial"]["disabled"] is True and g["initial"]["elapsed"] == "--:--"
    assert g["idle"] == {**g["idle"], "disabled": False, "label": "REC", "red": False, "dot": False,
                         "elapsed": "00:00", "status": "Idle · 2 clips", "blank": False}
    assert g["recording"]["label"] == "STOP" and g["recording"]["red"] is True and g["recording"]["dot"] is True
    assert g["recording"]["elapsed"] == "00:04" and g["recording_plus500"]["elapsed"] == "00:04"
    st = g["recording_stale"]
    assert st["stale"] is True and st["disabled"] is True and st["dot"] is False and st["elapsed"] == "00:05"


@needs_node
def test_glue_click_in_flight_failure_flash_and_fence(out):
    g = out["glue"]
    assert g["inflight_calls"] == [{"name": "camera/record_start", "type": "std_srvs/srv/Trigger"}]
    assert g["inflight"]["label"] == "…" and g["inflight"]["disabled"] is True
    assert g["calls_after_double_click"] == 1
    assert g["stop_call"] == {"name": "camera/record_stop", "type": "std_srvs/srv/Trigger"}
    assert g["failed"]["status"] == "Stop failed: timeout waiting for STOP" and g["failed"]["disabled"] is False
    assert g["after_flash"]["status"] == "Recording"
    assert g["rejected"]["status"] == "Error: Not connected to ROS"
    assert g["fenced"]["disabled"] is True and g["fenced_click_calls"] == 0
    d = g["disconnected"]
    assert d["blank"] is True and d["elapsed"] == "--:--" and d["disabled"] is True
    assert d["status"] == "ROS disconnected" and d["warnHidden"] is True


# ---------------------------------------------------------------- source wiring

def _src(rel):
    return (GUI / rel).read_text(encoding="utf-8")


def test_main_subscribes_camera_heartbeat_with_the_wave_type():
    src = _src("js/main.js")
    assert re.search(r"ros\.subscribe\('camera/heartbeat',\s*'jugglebot_interfaces/msg/WaveCameraHeartbeat',"
                     r"\s*cameraPanelOnHeartbeat,\s*0\)", src)


def test_camera_heartbeat_is_in_gui_subscribed_topics():
    block = re.search(r"const GUI_SUBSCRIBED_TOPICS = new Set\(\[(.*?)\]\)", _src("js/main.js"), re.S).group(1)
    assert "'camera/heartbeat'" in block


def test_main_inits_the_panel_and_blanks_it_on_disconnect():
    src = _src("js/main.js")
    assert re.search(r"import \{[^}]*\binitCameraPanel\b[^}]*\} from '\./camera-panel\.js';", src)
    init = re.search(r"function init\(\) \{(.*?)\n\}", src, re.S).group(1)
    assert "initCameraPanel();" in init
    assert init.index("initStateMinimap();") < init.index("initCameraPanel();")
    blank = re.search(r"export function blankDisconnectedState\(\) \{(.*?)\n\}", src, re.S).group(1)
    assert "setCameraPanelDisconnected();" in blank


def test_panel_click_checks_the_replay_fence_first():
    src = _src("js/camera-panel.js")
    body = re.search(r"function onRecClick\(\) \{(.*?)\n\}", src, re.S).group(1)
    assert body.lstrip().startswith("if (isReplayFenced()) return;")
    assert "'camera/record_start'" in src and "'camera/record_stop'" in src


def test_index_links_css_and_mounts_markup_after_the_minimap():
    html = _src("index.html")
    assert re.search(r'<link rel="stylesheet" href="css/state-minimap\.css">\s*'
                     r'<link rel="stylesheet" href="css/camera-panel\.css">', html)
    pane = html.index('<div id="viewer-pane">')
    minimap = html.index('<div id="state-minimap"')
    panel = html.index('<div id="camera-panel"')
    assert pane < minimap < panel < html.index('<div id="command-overlay">')
    for el_id in ("camera-rec-btn", "camera-rec-dot", "camera-rec-elapsed", "camera-rec-warn", "camera-rec-status"):
        assert 'id="%s"' % el_id in html, el_id
    assert re.search(r'<button class="cmd-btn[^"]*" id="camera-rec-btn"', html)


def test_css_sits_right_of_collapsed_minimap_and_shifts_when_expanded():
    css = _src("css/camera-panel.css")
    rule = re.search(r"#camera-panel \{(.*?)\}", css, re.S).group(1)
    assert "bottom: calc(var(--command-strip-h, 56px) + var(--gap) * 2);" in rule
    assert "left: calc(var(--gap) + 210px + var(--gap));" in rule
    assert "background: var(--bg-overlay);" in rule and "backdrop-filter: blur(8px);" in rule
    assert not re.search(r"transition:[^;]*\b(left|bottom)\b", rule)
    shift = re.search(r"#viewer-pane\.minimap-expanded #camera-panel \{(.*?)\}", css, re.S).group(1)
    assert "left: calc(var(--minimap-width) + var(--gap));" in shift
    # the collapsed minimap width this offset assumes
    assert "width: 210px;" in _src("css/state-minimap.css")
