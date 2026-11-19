"""The Platform IMU upload toggle and progress (IDEA-20 P3c).

``tests/ros/js/imu_upload_panel_harness.js`` runs the real module under node with a fake ros-bridge and DOM;
the source scan pins the wiring a harness cannot see. There is no gui_server endpoint: the toggle is a
rosbridge service call and the progress is the platform_imu/status topic, as for the Hand fan.
"""
from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
GUI = ROOT / "ros_ws" / "gui"
JS = GUI / "js"
HARNESS = Path(__file__).resolve().parent / "js" / "imu_upload_panel_harness.js"

FAKE_ROS = """
export const getConnectionState = () => globalThis.__fakeRos.state;
export const callService = (n, t, r) => globalThis.__fakeRos.callService(n, t, r);
export const withTimeout = (p) => p;
"""

from tests.ros._node import find_node as _find_node  # noqa: E402

NODE = _find_node()


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("imuuploadpanel")
    (sb / "replay").mkdir()
    shutil.copy(JS / "imu-upload-panel.js", sb / "imu-upload-panel.js")
    shutil.copy(JS / "replay" / "fence.js", sb / "replay" / "fence.js")
    (sb / "ros-bridge.js").write_text(FAKE_ROS)
    shutil.copy(HARNESS, sb / "imu_upload_panel_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "imu_upload_panel_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


def test_the_request_is_a_setbool_on_the_node_service(out):
    assert out["request"] == {"data": False}
    assert out["service"] == ["platform_imu/set_upload_after_attempt", "std_srvs/srv/SetBool"]


def test_no_status_is_unknown_and_offers_nothing(out):
    v = out["views"]["no_status"]
    assert not v["known"] and not v["toggleEnabled"] and v["stateText"] == "--" and v["progress"] is None


def test_recording_and_no_imu_read_differently(out):
    assert out["views"]["recording"]["stateText"] == "recording"
    assert out["views"]["recording"]["toggleChecked"] and out["views"]["recording"]["toggleEnabled"]
    no_imu = out["views"]["no_imu"]
    assert no_imu["stateText"] == "no IMU detected" and "no sensor" in no_imu["title"]


def test_an_upload_shows_phase_percent_and_seconds_left(out):
    v = out["views"]["uploading"]
    assert v["stateText"] == "uploading samples 50%, 24 s left"
    assert v["progress"] == 50 and v["stateClass"] == "busy"
    assert "refused" in v["title"] and "deactivate" in v["title"]


def test_the_toggle_state_is_the_nodes_not_the_click(out):
    assert out["views"]["off"]["toggleChecked"] is False
    assert out["click_off"]["calls"][0]["req"] == {"data": False}
    assert out["after_off_status"]["checked"] is False
    assert out["after_status"]["checked"] is True


def test_the_last_capture_result_shows_flags(out):
    v = out["views"]
    assert v["incomplete"]["lastText"] == "last capture: INCOMPLETE" and v["incomplete"]["lastClass"] == "warn"
    assert v["crc"]["lastText"] == "last capture: CRC FAILED" and v["crc"]["lastClass"] == "failed"
    assert v["failed"]["lastText"] == "last capture failed: reboot"
    assert v["complete"]["lastText"] == "last capture: complete"
    assert v["recording"]["lastText"] == ""


def test_a_stale_status_and_ros_down_and_replay_disable_the_toggle(out):
    assert out["views"]["just_fresh"]["known"] is True
    assert out["views"]["stale"]["known"] is False and not out["views"]["stale"]["toggleEnabled"]
    assert out["views"]["ros_down"]["toggleEnabled"] is False
    assert out["views"]["replay"]["known"] is False


def test_progress_bar_shows_only_while_uploading(out):
    assert out["after_status"]["barHidden"] is True
    assert out["uploading"]["barHidden"] is False and out["uploading"]["bar"] == 50
    assert out["uploading"]["state"].startswith("uploading samples 50%")


def test_a_refusal_and_a_lost_reply_are_flashed(out):
    assert out["refused"] == "REFUSED" and out["no_reply"] == "NO REPLY"


def test_disconnect_blanks_the_controls(out):
    d = out["disconnected"]
    assert d["disabled"] is True and d["state"] == "--" and d["barHidden"] is True


def test_main_wires_the_module_and_the_topic_is_gui_subscribed():
    text = (JS / "main.js").read_text()
    assert re.search(r"ros\.subscribe\('platform_imu/status',\s*'diagnostic_msgs/msg/DiagnosticStatus',\s*"
                     r"imuUploadPanelOnStatus,\s*0\)", text)
    assert "initImuUploadPanel();" in text
    blank = text[text.index("function blankDisconnectedState"):]
    assert "setImuUploadPanelDisconnected();" in blank[:blank.index("\n}\n")]
    block = re.search(r"const GUI_SUBSCRIBED_TOPICS = new Set\(\[(.*?)\]\)", text, re.S).group(1)
    assert "'platform_imu/status'" in block


def test_the_row_sits_in_platform_control_with_its_ids():
    html = (GUI / "index.html").read_text()
    start = html.index('id="panel-motion"')
    panel = html[start:html.index('id="panel-bb"')]
    for element in ("imu-upload-toggle", "imu-upload-state", "imu-upload-bar", "imu-upload-last"):
        assert 'id="%s"' % element in panel
    assert "Upload IMU data after each Attempt" in panel
