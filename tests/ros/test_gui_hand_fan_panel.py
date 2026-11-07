"""The Hand motor fan's GUI controls: the System pill and the Platform Control toggle.

``tests/ros/js/hand_fan_panel_harness.js`` runs the real module under node, with a fake
ros-bridge and a fake DOM. The source scan pins the wiring a harness cannot see.
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

ROOT = Path(__file__).resolve().parents[2]
GUI = ROOT / "ros_ws" / "gui"
JS = GUI / "js"
HARNESS = Path(__file__).resolve().parent / "js" / "hand_fan_panel_harness.js"

FAKE_ROS = """
export const getConnectionState = () => globalThis.__fakeRos.state;
export const callService = (n, t, r) => globalThis.__fakeRos.callService(n, t, r);
export const withTimeout = (p) => p;
"""


def _find_node():
    found = shutil.which("node") or shutil.which("nodejs")
    if found:
        return found
    for pat in (os.path.expanduser("~/.nvm/versions/node/*/bin/node"),
                "/usr/local/bin/node", "/usr/bin/node"):
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[-1]
    return None


NODE = _find_node()


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("handfanpanel")
    (sb / "replay").mkdir()
    shutil.copy(JS / "hand-fan-panel.js", sb / "hand-fan-panel.js")
    shutil.copy(JS / "replay" / "fence.js", sb / "replay" / "fence.js")
    (sb / "ros-bridge.js").write_text(FAKE_ROS)
    shutil.copy(HARNESS, sb / "hand_fan_panel_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "hand_fan_panel_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


def test_the_request_keeps_the_thresholds(out):
    """0 means keep: the GUI must never overwrite thresholds set by service call."""
    assert out["request"] == {"mode": "off", "on_above_c": 0.0, "off_below_c": 0.0}
    assert out["service"] == ["hand_fan/set", "jugglebot_interfaces/srv/SetHandFan"]


def test_no_status_is_unknown_and_commands_nothing(out):
    v = out["views"]["no_status"]
    assert v["pillText"] == "UNKNOWN" and v["pillClass"] == "unknown"
    assert not v["offEnabled"] and not v["onEnabled"] and v["autoHidden"]


def test_auto_lights_the_real_fan_state_and_offers_no_auto_button(out):
    off, on = out["views"]["auto_off"], out["views"]["auto_on"]
    assert off["pillText"] == "OFF" and off["offActive"] and not off["onActive"]
    assert on["pillText"] == "ON" and on["onActive"] and not on["offActive"]
    for v in (off, on):
        assert v["modeText"] == "AUTO" and v["autoHidden"]
        assert v["offEnabled"] and v["onEnabled"]


def test_forced_shows_forced_and_offers_auto(out):
    on, off = out["views"]["forced_on"], out["views"]["forced_off"]
    assert on["modeText"] == off["modeText"] == "FORCED"
    assert on["modeClass"] == "forced"
    assert not on["autoHidden"] and on["autoEnabled"]
    assert not on["onEnabled"] and on["offEnabled"]      # already forced that way
    assert not off["offEnabled"] and off["onEnabled"]


def test_the_tooltip_names_the_reading_and_thresholds(out):
    assert "28.2" in out["views"]["auto_off"]["title"] or "28.3" in out["views"]["auto_off"]["title"]
    assert "35.0" in out["views"]["auto_off"]["title"]
    assert "no reading" in out["views"]["no_reading"]["title"]


def test_a_stale_status_stops_claiming_a_fan_state(out):
    assert out["views"]["just_fresh"]["known"] is True
    v = out["views"]["stale"]
    assert v["pillText"] == "UNKNOWN" and not v["onActive"] and not v["onEnabled"]


def test_ros_down_and_pending_disable_every_command_but_keep_the_state(out):
    for name in ("ros_down", "pending"):
        v = out["views"][name]
        assert v["pillText"] == "ON"
        assert not v["offEnabled"] and not v["onEnabled"] and not v["autoEnabled"]


def test_replay_shows_the_recorded_state_read_only_and_never_goes_stale(out):
    v = out["views"]["replay"]
    assert v["pillText"] == "ON" and v["modeText"] == "FORCED"
    assert not v["offEnabled"] and not v["onEnabled"] and not v["autoEnabled"]


def test_glue_is_born_unknown_then_follows_status(out):
    assert out["born"]["pill"] == "UNKNOWN" and out["born"]["autoHidden"] is True
    assert out["after_status"]["pill"] == "OFF"
    assert out["after_status"]["pillClass"] == "flag-ball-pill empty"
    assert out["after_status"]["mode"] == "AUTO"


def test_a_click_sends_the_mode_and_the_toggle_waits_for_the_board(out):
    calls = out["click_on"]["calls"]
    assert len(calls) == 1 and calls[0]["req"]["mode"] == "on"
    assert out["click_on"]["snap"]["onActive"] is False
    after = out["after_forced_status"]
    assert after["onActive"] is True and after["mode"] == "FORCED" and after["autoHidden"] is False


def test_auto_click_restores_auto(out):
    assert [c["req"]["mode"] for c in out["click_auto"]] == ["auto"]


def test_one_command_at_a_time(out):
    assert out["while_pending"]["disabled"] is True
    assert out["while_pending"]["calls"] == 1


def test_a_refusal_and_a_lost_reply_are_shown_then_cleared(out):
    assert out["refused"] == "REFUSED"
    assert out["flash_cleared"] == "AUTO"
    assert out["no_reply"] == "NO REPLY"


def test_disconnect_blanks_the_controls(out):
    d = out["disconnected"]
    assert d["pill"] == "UNKNOWN" and d["onDisabled"] and d["offDisabled"] and d["autoHidden"]


def test_main_wires_the_module():
    text = (JS / "main.js").read_text()
    assert re.search(r"ros\.subscribe\('hand_fan/status',\s*"
                     r"'jugglebot_interfaces/msg/HandFanStatus',\s*handFanPanelOnStatus,\s*0\)", text)
    assert "initHandFanPanel();" in text
    blank = text[text.index("function blankDisconnectedState"):]
    assert "setHandFanPanelDisconnected();" in blank[:blank.index("\n}\n")]


def test_status_is_in_gui_subscribed_topics():
    block = re.search(r"const GUI_SUBSCRIBED_TOPICS = new Set\(\[(.*?)\]\)",
                      (JS / "main.js").read_text(), re.S).group(1)
    assert "'hand_fan/status'" in block


def test_the_system_panel_has_a_read_only_pill():
    text = (JS / "panels.js").read_text()
    assert re.search(r"id: 'flag-hand-fan',\s*type: 'pill',\s*label: 'Hand Motor Fan'", text)


def test_the_toggle_sits_in_platform_control():
    html = (GUI / "index.html").read_text()
    start = html.index('id="panel-motion"')
    panel = html[start:html.index('id="panel-bb"')]
    for element in ("hand-fan-off", "hand-fan-on", "hand-fan-auto", "hand-fan-mode"):
        assert 'id="%s"' % element in panel
    assert re.search(r'id="hand-fan-auto"[^>]*\bhidden\b', panel)
