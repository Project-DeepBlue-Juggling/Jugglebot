"""The IMU chart group in the GUI chart panel (frontend half; the server routes are test_gui_imu_api.py).

``tests/ros/js/imu_charts_harness.js`` runs the REAL ``imu-charts-core.js`` (binary parse, gaps, derivative,
decimation, selection / following, the do-not-yank rule), ``imu-charts.js`` (the glue, against a fake fetch,
DOM and uPlot) and the external-chart hook of ``telemetry-charts.js`` under node. The capture fixture is
written here with ``struct`` from the spec's byte layout, so the parser is checked against the layout, not
against itself. The source scans pin the wiring a harness cannot see.
"""
from __future__ import annotations

import json
import math
import re
import shutil
import struct
import subprocess
from pathlib import Path

import pytest

from tests.ros._node import find_node as _find_node  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
GUI = ROOT / "ros_ws" / "gui"
JS = GUI / "js"
HARNESS = Path(__file__).resolve().parent / "js" / "imu_charts_harness.js"
NODE = _find_node()

T0 = 1795175860.0
F32 = lambda v: struct.unpack("<f", struct.pack("<f", v))[0]  # noqa: E731
T = [T0, T0 + 0.001, float("nan"), T0 + 0.003, T0 + 0.004, T0 + 0.005]
ACC = [[0.1 * i, -0.1 * i, 1.0] for i in range(6)]
GYR = [[float(i), 2.0 * i, 3.0 * i] for i in range(6)]
VALID = [1, 1, 1, 0, 1, 1]


def _fixture(path: Path) -> None:
    n = len(T)
    blob = b"JBIMU1\0\0" + struct.pack("<II", n, 0)
    blob += struct.pack("<%dd" % n, *T)
    blob += struct.pack("<%df" % (3 * n), *[v for row in ACC for v in row])
    blob += struct.pack("<%df" % (3 * n), *[v for row in GYR for v in row])
    blob += bytes(VALID)
    path.write_bytes(blob)


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("imucharts")
    for name in ("telemetry-charts.js", "axis-ticks.js", "clock.js", "event-store.js", "geometry-config.js",
                 "callout-layout.js", "imu-charts.js", "imu-charts-core.js"):
        shutil.copy(JS / name, sb / name)
    (sb / "stewart-model.js").write_text("export function setStewartHighlight() {}\n")
    (sb / "ball-butler-model.js").write_text("export function setBallButlerHighlight() {}\n")
    shutil.copy(HARNESS, sb / "imu_charts_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    _fixture(sb / "capture.bin")
    proc = subprocess.run([NODE, str(sb / "imu_charts_harness.js"), str(sb / "capture.bin")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120, env={"TZ": "UTC"})
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout.decode().strip().splitlines()[-1])


# ---- binary parse and plot columns ----

def test_the_binary_layout_parses_to_the_values_written(out):
    p = out["parse"]
    assert p["n"] == 6
    assert p["t"][:2] == T[:2] and p["t"][2] is None and p["t"][3:] == T[3:]
    assert p["accel"] == pytest.approx([F32(v) for row in ACC for v in row])
    assert p["gyro"] == pytest.approx([F32(v) for row in GYR for v in row])
    assert p["valid"] == VALID


def test_a_wrong_or_truncated_buffer_is_refused_in_words(out):
    assert "magic" in out["parse_bad_magic"]
    assert "does not match" in out["parse_truncated"]
    assert "not binary" in out["parse_not_binary"]


def test_invalid_samples_and_nonfinite_times_are_gaps_never_bridged(out):
    c = out["columns"]
    # the NaN time is dropped and marked by one NaN sample between its neighbours
    assert c["nGaps"] == 1 and len(c["t"]) == 6
    assert c["t"][2] == pytest.approx(T0 + 0.002)
    assert c["ax"][2] is None            # gap marker
    assert c["ax"][3] is None            # valid == 0
    assert c["ax"][4] == pytest.approx(F32(0.4)) and c["ax"][1] == pytest.approx(F32(0.1))
    assert c["amag"][4] == pytest.approx(math.sqrt(F32(0.4) ** 2 + F32(-0.4) ** 2 + 1.0))


# ---- decimation ----

def test_decimation_keeps_every_pixel_columns_min_and_max(out):
    d = out["decimate"]
    assert d["decimated"] and d["points"] <= 3 * d["W"] + 2
    assert d["colsBad"] == 0 and d["colsOk"] == d["W"]
    assert d["ascending"] and d["edgeBefore"] and d["edgeAfter"]


def test_decimation_keeps_a_gap_visible(out):
    assert out["decimate"]["gapNaN"] >= 1


def test_few_samples_in_view_are_full_resolution_zero_copy(out):
    f = out["full"]
    assert not f["decimated"] and f["zeroCopy"]
    assert f["len"] == 503 and f["first"] < 1002 and f["last"] > 1002.5


# ---- angular acceleration ----

def test_angular_acceleration_of_a_sine_is_its_derivative(out):
    d = out["deriv"]
    assert d["finite"] == d["interior"] and d["edgeNaN"]
    assert d["maxErr"] < 0.01 * d["peak"]          # 10 ms boxcar at 2 Hz attenuates < 0.1 %


def test_angular_acceleration_is_never_smoothed_or_differenced_across_a_gap(out):
    assert out["deriv_gap"]["nanNear"] == 22         # 10 NaN samples + the 5-sample half window + 1 either side
    assert out["deriv_gap"]["finiteFar"] == 80
    assert out["deriv_hole"]["nanAtHole"]


def test_the_label_states_the_smoothing_window(out):
    assert out["ang_title"] == "Angular acceleration (deg/s^2, 10 ms boxcar)"


# ---- lookup, labels, launch window, status ----

def test_nearest_sample_and_coverage(out):
    assert out["nearest"] == [2, -1, 3, -1]


def test_dropdown_label(out):
    assert out["label"] == "09:50:14 · columns attempt 3 · 32.4 s · incomplete"
    assert out["label_complete"] == "09:50:14 · no label · 32.4 s"


def test_live_launch_window_is_the_newest_recording(out):
    assert out["launch"] == {"from": pytest.approx(1795175619.424748), "fallback": False}
    # an in-progress bag has no start_ns yet: its id (local time, TZ=UTC here) is used
    assert out["launch_inprogress"] == {"from": 1795177800, "fallback": False}
    assert out["launch_none"] == {"from": 1795180000 - 12 * 3600, "fallback": True}


def test_list_failures_are_plain_words(out):
    assert "restart the GUI server to enable IMU charts" in out["list_msgs"][0]
    assert "not reachable" in out["list_msgs"][1] and "HTTP 500" in out["list_msgs"][2]


def test_status_refresh_triggers(out):
    # first message: no edge; busy falls: refresh; last_label moves: refresh; unchanged: nothing
    assert out["status"] == [False, True, True, False]


# ---- selection / following (item 4) and the yank rule (item 5) ----

def test_live_following(out):
    assert out["live"] == [
        ["B", True, True],
        ["D", True, True],
        ["A", False, "pick"],
        ["A", False, False],
        ["E", True, "pick"],
        ["F", True, True],
        ["B", True, True],
    ]


def test_replay_selection_follows_the_playhead_and_holds_a_hand_pick(out):
    r = out["replay"]
    assert r[:5] == ["A", "A", "B", "D", None]       # inside, nearest (each side), far after, empty list
    assert json.loads(r[5]) == {"stem": "D", "at": "A"}
    assert r[6:] == ["D", "D", "B", "D", "A", "D"]


def test_auto_follow_does_not_yank_a_moved_view(out):
    assert out["yank"] == ["jump", "jump", "offer", "offer", "offer"]
    assert out["in_view"] == [True, False]


# ---- the telemetry-charts.js hook ----

def test_hook_broadcasts_and_requests(out):
    assert out["hook_n_odrive"] == 9 and out["hook_sync_key"] == "telemetry"
    assert out["hook_show"] is True
    a = out["hook_after_show"]
    assert a["got"] == [100, 132] and a["odriveX"]
    assert a["view"]["paused"] and a["view"]["manual"] and a["view"]["min"] == 100 and a["view"]["max"] == 132
    assert out["hook_after_select"] == {"got": [110, 120], "odriveX": True}


def test_an_external_wheel_and_dblclick_drive_every_chart(out):
    w = out["hook_after_wheel"]
    assert w["got"] == pytest.approx([109.5, 120.5]) and w["odrive"] == pytest.approx([109.5, 120.5])
    r = out["hook_after_reset"]
    assert not r["view"]["manual"] and r["got"][1] - r["got"][0] == pytest.approx(10)


def test_unsubscribe_stops_the_broadcast(out):
    assert out["hook_after_unsub"] == out["hook_listener_count_before_unsub"]


def test_replay_playhead_is_broadcast_and_a_jump_is_a_seek(out):
    r = out["replay_hook"]
    assert r["got"] == [495, 505]
    assert r["show"] is False and r["seek"] is True and r["seeks"] == [600] and r["last"] == [595, 605]
    assert r["view"]["replay"] and not r["view"]["paused"]
    assert out["seek_live"] is False


# ---- imu-charts.js glue ----

def test_hidden_by_default_and_nothing_fetched(out):
    assert out["glue_live_before"]["paused"] is False and out["glue_live_before"]["manual"] is False
    h = out["glue_hidden"]
    assert h == {"fetches": 0, "groupHidden": True, "selectHidden": True, "shownKey": True, "intervals": 0}


def test_showing_follows_the_newest_capture_and_moves_every_chart(out):
    s = out["glue_shown"]
    assert s["shownKey"] == "true" and s["groupHidden"] is False
    assert s["fetches"][0] == "/api/replay/recordings"
    assert s["fetches"][1] == "/api/imu/captures?from=1795162000"
    assert s["fetches"][2] == "/api/imu/captures/20261120T115806Z_AEDE/data"
    assert s["options"] == ["20261120T115806Z_AEDE", "20261120T095142Z_C33B"]      # newest first
    assert s["selected"] == "20261120T115806Z_AEDE"
    assert s["view"]["paused"] and s["view"]["min"] == 1795175854.0 and s["view"]["max"] == 1795175886.0
    assert s["imuCharts"] == 2
    assert [d["x"] for d in out["glue_draw"]] == [[1795175854.0, 1795175886.0]] * 2
    assert [d["nSeries"] for d in out["glue_draw"]] == [4, 3]
    assert all(d["n"] == 6 and d["sync"] == "telemetry" for d in out["glue_draw"])


def test_series_pill_hides_the_series_and_persists(out):
    assert out["glue_pill"]["show"] is False and "amag" not in out["glue_pill"]["stored"]


def test_a_new_capture_after_the_operator_moved_the_view_only_changes_the_dropdown(out):
    n = out["glue_no_yank"]
    assert n["selected"] == "20261120T120305Z_AEDF"
    assert n["view"]["min"] == 1795175860 and n["view"]["max"] == 1795175870
    assert n["jumpHidden"] is False
    j = out["glue_jump"]
    assert j["view"]["min"] == 1795176185.0 and j["view"]["max"] == 1795176217.0 and j["jumpHidden"] is True


def test_hiding_stops_fetching(out):
    assert out["glue_hide"] == {"fetchesAfter": 0, "groupHidden": True, "shownKey": "false"}


def test_imu_row_height_is_clamped_and_the_3d_view_keeps_its_minimum(out):
    h = out["imu_height"]
    assert h["clamp"] == [100, 500, 333, 200, 200, 250]
    assert h["free"] == 300
    assert h["capped"] == 300      # IMU row shrinks first so the 3D view keeps 150 px
    assert h["floor"] == 100 and h["noMeasure"] == 400


def test_imu_charts_draw_the_motor_charts_playhead(out):
    assert out["imu_playhead_hook"] >= 2


# ---- source scans ----

def test_the_group_is_a_row_above_the_motor_grid_hidden_by_default():
    html = (GUI / "index.html").read_text()
    panel = html[html.index('<div id="chart-panel">'):]
    group, grid = panel.index('<div id="imu-chart-group" hidden>'), panel.index('<div id="chart-grid">')
    assert group < grid
    toolbar = panel[:panel.index('<div id="imu-chart-group"')]
    for el in ("imu-charts-toggle-btn", "imu-capture-select", "imu-jump-btn"):
        assert 'id="%s"' % el in toolbar
    row = panel[group:grid]
    for el in ("imu-accel-plot", "imu-gyro-plot", "imu-gyro-mode-btn", "imu-chart-message"):
        assert 'id="%s"' % el in row
    assert "Linear acceleration (g)" in row and "Angular rate (deg/s)" in row and "IMU x/y/z (sensor frame)" in row
    css = (GUI / "css" / "charts.css").read_text()
    assert "#imu-chart-group[hidden]" in css


def test_main_wires_the_module():
    text = (JS / "main.js").read_text()
    assert "import { initImuCharts, attachImuChartsReplay, imuChartsOnStatus } from './imu-charts.js';" in text
    assert re.search(r"initTelemetryCharts\(\);\s*initImuCharts\(\);", text)
    assert "attachImuChartsReplay(replayMode);" in text
    assert re.search(r"ros\.subscribe\('platform_imu/status',\s*'diagnostic_msgs/msg/DiagnosticStatus',\s*"
                     r"imuChartsOnStatus,\s*0\)", text)


def test_storage_key_and_hook_exports():
    core = (JS / "imu-charts-core.js").read_text()
    assert "'jugglebot-imu-charts-shown'" in core and "ANG_ACCEL_SMOOTH_MS = 10" in core
    tc = (JS / "telemetry-charts.js").read_text()
    for name in ("onSharedXRange", "getSharedXRange", "getSharedViewState", "showRangeOnAllCharts",
                 "selectRangeOnAllCharts", "seekReplayTo", "attachZoomPanControls"):
        assert re.search(r"export function %s\(" % name, tc), name
    assert "export const SYNC_KEY = 'telemetry';" in tc


def test_height_divider_wiring_and_the_old_fixed_200px_is_gone():
    css = (GUI / "css" / "charts.css").read_text()
    assert "200px" not in css.split("IMU charts (imu-charts.js)")[1] and "220px" not in css
    assert css.count("var(--imu-chart-height)") >= 3
    html = (GUI / "index.html").read_text()
    assert html.index('id="imu-chart-divider"') < html.index('<div id="chart-grid">')
    assert "jugglebot-imu-chart-height" in (JS / "imu-charts-core.js").read_text()


def test_motor_grid_button_is_named_odrive_charts():
    js = (JS / "telemetry-charts.js").read_text()
    assert "'Show ODrive charts' : 'Hide ODrive charts'" in js
    assert "'Show charts'" not in js and "'Hide charts'" not in js
