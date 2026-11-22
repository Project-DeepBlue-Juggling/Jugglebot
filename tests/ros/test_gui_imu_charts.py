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
    (sb / "replay").mkdir()
    shutil.copy(JS / "ball-events.js", sb / "ball-events.js")
    shutil.copy(JS / "replay" / "span-events.js", sb / "replay" / "span-events.js")
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
    assert s["fetches"][2] == "/api/imu/captures/20261120T115806Z_AEDE/data?frame=imu"
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


def test_panel_height_model_clamps_and_splits(out):
    h = out["imu_height"]
    assert h["clamp"] == [100, 9999, 333, 200, 200, 250]
    # IMU share: both groups keep >= 100 px of the shared space; too little room splits it evenly
    assert h["share"] == [300, 100, 500, 150, 75, 200]
    assert h["even"] == [300, 250, 100]
    assert h["both_default"] == 500
    # top handle: both shown = toolbar + divider + border + 2 x 100; max leaves the 3D view 150 px
    assert h["bounds_both"] == {"min": 244, "max": 804}
    assert h["bounds_single"] == {"min": 150, "max": 804}
    assert h["bounds_tiny_window"]["max"] == h["bounds_tiny_window"]["min"]    # never max < min
    assert h["clamp_panel"] == [244, 804, 500, "NaN"]
    assert h["applied_stored"] == [700, 400, 300, "NaN", "NaN"]


def test_frame_labels_query_and_csv_header_say_which_frame(out):
    """A reader can never confuse the raw sensor axes with the Platform axes: labels, the chart-head note, the
    request and the CSV columns all carry the frame. A row without a frame (an older server) is the IMU frame."""
    f = out["frames"]
    assert f["q_platform"] == "?frame=platform" and f["q_imu"] == "?frame=imu" and f["q_unknown"] == "?frame=imu"
    assert f["frame_row"] == ["platform", "imu", "imu", "imu"]
    assert (f["label_p"], f["label_i"]) == ("x (Platform)", "x (IMU)")
    assert f["note_p"] == "Platform x/y/z (mount calibration 2026-11-22)"
    assert f["note_i"] == "IMU x/y/z (sensor frame, not mount-calibrated)"
    assert f["csv_p"] == ["plat_ax_g", "plat_ay_g", "plat_az_g", "plat_gx_dps", "plat_gy_dps", "plat_gz_dps"]
    assert f["csv_i"] == ["imu_ax_g", "imu_ay_g", "imu_az_g", "imu_gx_dps", "imu_gy_dps", "imu_gz_dps"]


def test_csv_merge_is_the_time_sorted_union(out):
    m = out["csv_merge"]
    assert m["header"] == ["imu_ax_g", "imu_ay_g", "imu_az_g", "imu_gx_dps", "imu_gy_dps", "imu_gz_dps"]
    # IMU samples at 9.0 (before the motor range), 11.0 (NaN gap marker) and 14.0 (after) are dropped;
    # 10.0 and 12.0 coincide with motor rows k=0 and k=2 and share them; 10.5 and 11.25 are IMU-only rows
    assert m["rows"] == [[10, 0, 1], [10.5, -1, 2], [11, 1, -1], [11.25, -1, 4], [12, 2, 5], [13, 3, -1]]
    assert m["no_imu"] == [[10, 0, -1], [11, 1, -1], [12, 2, -1], [13, 3, -1]]
    assert m["no_motor"] == 0


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
    assert "Linear acceleration (g)" in row and "Angular rate (deg/s)" in row and "IMU x/y/z (sensor frame, not mount-calibrated)" in row
    css = (GUI / "css" / "charts.css").read_text()
    assert "#imu-chart-group[hidden]" in css


def test_main_wires_the_module():
    text = (JS / "main.js").read_text()
    assert "import { initImuCharts, attachImuChartsReplay, imuChartsOnStatus, imuChartsRelayout } from './imu-charts.js';" in text
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
    # the old fixed 200 px IMU row is gone; the only 200px left is the both-shown default (single + 200)
    assert css.split("IMU charts (imu-charts.js)")[1].count("200px") == 1 and "220px" not in css
    assert css.count("var(--imu-chart-height)") >= 2
    # both shown: the panel has its own total height (the top handle's), the grid takes what the IMU row leaves
    assert "var(--chart-panel-both-height" in css and "jugglebot-chart-height-both" in (JS / "imu-charts-core.js").read_text()
    html = (GUI / "index.html").read_text()
    assert html.index('id="imu-chart-divider"') < html.index('<div id="chart-grid">')
    assert "jugglebot-imu-chart-height" in (JS / "imu-charts-core.js").read_text()


def test_motor_grid_button_is_named_odrive_charts():
    js = (JS / "telemetry-charts.js").read_text()
    assert "'Show ODrive charts' : 'Hide ODrive charts'" in js
    assert "'Show charts'" not in js and "'Hide charts'" not in js


def test_chart_handle_label_and_collapse_are_gone():
    """The "Charts" label/chevron on the top handle and the collapse flag it persisted are removed; a stored
    'jugglebot-chart-collapsed' is cleared at startup instead of honoured."""
    html = (GUI / "index.html").read_text()
    assert "chart-toggle" not in html
    main = (JS / "main.js").read_text()
    assert "chart-toggle" not in main and "classList.add('collapsed')" not in main.split("Chart panel resize handle")[1].split("Font size control")[0]
    assert re.search(r"localStorage\.removeItem\(CHART_COLLAPSED_STORAGE_KEY_RETIRED\)", main)
    for f in (GUI / "css").glob("*.css"):
        t = f.read_text()
        assert "chart-toggle" not in t and "#chart-panel.collapsed" not in t, f.name
    assert "contains('collapsed')" not in (JS / "telemetry-charts.js").read_text()


def test_export_csv_hook_is_the_only_telemetry_charts_change_for_imu():
    tc = (JS / "telemetry-charts.js").read_text()
    assert "export function setCsvExtraProvider(" in tc
    assert "setCsvExtraProvider(csvExtra)" in (JS / "imu-charts.js").read_text()


# ---- low-pass filter (core) ----

def test_lowpass_is_a_zero_phase_butterworth_at_the_data_rate(out):
    for key in ("filt_gain_1k", "filt_gain_2k"):
        g5, g15, g30, g100 = out[key]
        assert abs(g5["g"] - 0.988) < 0.01              # |H|^2 of a 2nd-order Butterworth, forward-backward
        assert abs(g15["g"] - 0.5) < 0.01               # -6 dB at the cutoff (-3 dB per pass)
        assert abs(g30["g"] - 1 / 17) < 0.01            # 4th-order magnitude: 1 / (1 + 2^4)
        assert g100["g"] < 1e-3
        assert all(x["quad"] < 0.01 for x in (g5, g15, g30))    # no quadrature part: zero phase
    g20, g100, g300 = out["filt_gain_100"]
    assert g20["g"] > 0.99 and abs(g100["g"] - 0.5) < 0.01 and g300["g"] < 0.02
    assert abs(out["filt_fs_from_data"] - 994) < 0.5


def test_lowpass_keeps_a_pulse_in_place_and_symmetric(out):
    p = out["filt_pulse"]
    assert p["imax"] == 1000 and p["asym"] < 1e-9 and 0 < p["peak"] < 0.1


def test_lowpass_never_filters_across_a_gap(out):
    g = out["filt_gap"]
    assert g["before"] < 1e-3 and g["after"] < 1e-3     # the +100 step after the gap does not leak backwards
    assert g["inGap"] == 10                               # the gap stays a gap


def test_short_runs_become_gaps_and_a_time_hole_splits_a_run(out):
    s = out["filt_short"]
    assert s["minRun"] == 16 and s["shortNaN"] and s["longFinite"]
    assert out["filt_hole"]["finite"] and out["filt_hole"]["split"]


def test_filter_off_or_unusable_returns_the_raw_array_and_settings_parse(out):
    assert out["filt_raw"] == [True, True, True, 15, 0, 0, "low-pass 15 Hz", ""]


def test_amag_comes_from_the_filtered_components_and_raw_columns_are_untouched(out):
    c = out["filt_cols"]
    assert c["off"] and c["rawSpike"] == pytest.approx(50.2494, abs=1e-3) and c["rawKept"] == 50
    assert c["amag100"] == pytest.approx(c["want"], abs=1e-9) and c["spikeCut"]


# ---- spectrum (core) ----

def test_welch_peak_frequency_and_power_of_known_sines(out):
    for key in ("s994", "g994", "s2k", "s2k_hi"):
        r = out["spec_sine"][key]
        assert r["bin"] < 0.5, key                        # within half a bin (the parabola does far better)
        assert abs(r["powerRatio"] - 1) < 0.03, key       # integral of the PSD = A^2/2 within 3 %
    assert out["spec_sine"]["s994"]["seg"] == 4096 and abs(out["spec_sine"]["s994"]["fs"] - 994) < 0.5
    assert out["spec_sine"]["s2k"]["df"] == pytest.approx(2000 / 4096)


def test_fft_bins_and_length_check(out):
    f = out["fft_bins"]
    assert f["k5"] == pytest.approx(32) and f["kN5"] == pytest.approx(32) and f["other"] < 1e-9
    assert "power of two" in out["fft_bad"]


def test_white_noise_density_is_sigma_sqrt_two_over_fs(out):
    n = out["spec_noise"]
    assert n["seg"] == 4096 and n["nSeg"] > 15
    assert abs(n["rms"] / n["want"] - 1) < 0.05


def test_spectrum_uses_only_samples_in_the_range_and_range_indices_are_inclusive(out):
    r = out["spec_range"]
    assert abs(r["a"] - 12.5) < 0.1 and abs(r["b"] - 40) < 0.1
    assert r["idx"] == [1000, 1005] and r["idxEmpty"][0] == r["idxEmpty"][1] and r["idxAll"] == [0, 30000]


def test_segment_length_and_the_too_few_samples_message(out):
    s = out["spec_seg"]
    assert "Fewer than 256" in s["n255"]
    assert s["n256"] == [256, 1] and s["n300"][0] == 256 and s["n5000"][0] == 4096 and s["n20000"][0] == 4096
    assert s["n5000"][1] == 1 and s["n20000"][1] > 4


def test_welch_segments_never_straddle_a_gap(out):
    g = out["spec_gap"]
    assert g["seg"] == 512 and g["nSeg"] == 2 and g["finite"]   # one segment in each 600-sample run
    assert "Fewer than 256" in out["spec_fragmented"]


def test_peak_search_starts_at_one_hz_and_interpolates(out):
    p = out["spec_peak"]
    assert abs(p["f"] - 7.37) < 0.02 and abs(p["f"] - 7.37) < p["df"] / 4
    assert p["fullF"] < 1                                  # the 0.4 Hz tone would win without the 1 Hz floor
    assert p["none"] is None
    assert p["xmax"] == [10, 50, 497, 30] and p["parse"] == [10, 0, 50, 50]


# ---- filter and spectrum (glue) ----

def test_filter_selector_persists_and_badges_both_titles(out):
    assert out["glue_filter"] == {"stored": "15", "badge": "low-pass 15 Hz", "badgeGyro": "low-pass 15 Hz"}
    assert out["glue_filter_off"] == {"stored": "0", "badge": ""}


def test_spectrum_mode_leaves_the_shared_cursor_zoom_and_time_axis(out):
    s = out["glue_spectrum"]
    assert s["stored"] == "true" and s["n"] == 2
    assert s["sync"] == [False, False] and s["wheel"] == [False, False]
    assert s["logY"] == [3, 3] and s["timeX"] == [False, False]
    assert "Fewer than 256" in s["msg"] and not s["msgHidden"]       # the 6-sample fixture
    assert "spectrum" in s["accelTitle"] and s["gyroModeHidden"] and not s["maxHidden"] and s["filterDisabled"]
    assert out["glue_spec_max"] == "10"


def test_leaving_spectrum_mode_restores_sync_zoom_and_markers(out):
    o = out["glue_spectrum_off"]
    assert o["stored"] == "false" and o["sync"] == [True, True] and o["wheel"] == [True, True]
    assert o["markerHook"] == [True, True] and o["accelTitle"] == "Linear acceleration (g)"


def test_source_wiring_of_the_event_markers_and_the_spectrum_debounce():
    imu = (JS / "imu-charts.js").read_text()
    tel = (JS / "telemetry-charts.js").read_text()
    assert "drawEventMarkers, drawReplayPlayhead" in imu and "attachEventMarkerHover(u)" in imu
    assert "export function drawEventMarkers" in tel and "paintEventMarkers(u);" in tel
    # trailing debounce WITH a max wait (a range that moves every frame must not starve the pass forever)
    assert "setTimeout(() => { specTimer = null; computeSpectrum(); }, wait)" in imu
    assert "core.SPEC_DEBOUNCE_MS, specFirstReq + SPEC_MAX_WAIT_MS - now" in imu and "SPEC_MAX_WAIT_MS = 500" in imu
    assert "SPEC_DEBOUNCE_MS = 150" in (JS / "imu-charts-core.js").read_text()
