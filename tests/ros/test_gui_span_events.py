"""Throw / Catch markers on the IMU charts across the whole capture, and the hidden ODrive grid staying hidden.

Frontend halves, REAL modules under node (``tests/ros/js/span_events_harness.js``): ``replay/span-events.js`` reads the
/balls of a time span straight from the recording (filtered, lite chunk loads) through a fresh ``ball-events.js`` mapper,
and ``event-store.js`` keeps the result in a span list that only the IMU charts merge in (the Event Log never sees it).
Source scans pin the wiring a harness cannot see.
"""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest

from tests.ros._node import find_node as _find_node  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
JS = ROOT / "ros_ws" / "gui" / "js"
HARNESS = Path(__file__).resolve().parent / "js" / "span_events_harness.js"
NODE = _find_node()


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    if NODE is None:
        pytest.skip("node not installed")
    sb = tmp_path_factory.mktemp("span_events")
    (sb / "replay").mkdir()
    for name in ("event-store.js", "clock.js", "ball-events.js"):
        shutil.copy(JS / name, sb / name)
    shutil.copy(JS / "replay" / "span-events.js", sb / "replay" / "span-events.js")
    shutil.copy(HARNESS, sb / "harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "harness.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def test_span_scan_marks_every_throw_and_catch_inside_the_span(out):
    # Throws at 4 + 3i s, Catches 1.2 s later: the span [10, 25] holds Throws at 10, 13, 16, 19, 22, 25
    assert out["midThrows"] == [10, 13, 16, 19, 22, 25]
    assert out["midCatches"] == [11.2, 14.2, 17.2, 20.2, 23.2]
    assert out["midSorted"] is True
    assert out["allCounts"] == [12, 12]


def test_span_scan_reads_only_balls_through_lite_loads(out):
    assert out["onlyBallsLite"] is True
    assert out["chunksTouched"] == [0, 1, 2]         # [7, 28] s: span + the 3 s margin on both sides


def test_span_scan_cancel_and_empty_span(out):
    assert out["cancelled"] is None and out["reversed"] == 0


def test_event_store_span_list_is_merged_only_on_request_and_deduped(out):
    assert out["plainCount"] == 2                     # the ring: the one past Throw and the command
    assert out["spannedCount"] == 25                  # 24 span markers + the command; the shared first Throw counts once
    assert out["spannedSorted"] is True and out["spanIdsUnique"] is True
    assert "catch" not in out["logTypes"] and out["logTypes"].count("throw") == 0   # chart-only types stay out of the Event Log list


def test_span_list_survives_a_backward_seek_trim_and_dies_with_replay(out):
    assert out["afterTrimSpanned"] == 25              # 24 span markers survive; the trim only drops ring events after t (the command at +1 s stays)
    assert out["cleared"] == 1 and out["afterRestore"] == 0


# ---- source scans ----

def test_imu_charts_load_the_span_markers_per_capture_and_only_imu_charts_draw_them():
    imu = (JS / "imu-charts.js").read_text()
    assert "scanBallEvents(" in imu and "setSpanEvents([])" in imu
    assert imu.count("syncSpanMarkers();") >= 2       # selection change (loadSelected) and mode reset (resetSelection)
    tc = (JS / "telemetry-charts.js").read_text()
    assert "spanMarkerCharts" in tc
    # The ODrive charts' marker pass must not opt in: only the externally built IMU charts register through these two exports
    assert tc.count("spanMarkerCharts.add(u)") == 2


def test_top_handle_drag_does_not_unhide_the_odrive_grid_while_the_imu_group_is_shown():
    main = (JS / "main.js").read_text()
    assert "if (!panel.classList.contains('imu-shown')) clearChartGridHidden();" in main
    assert main.count("clearChartGridHidden()") == 1  # no other caller re-reveals the grid behind the operator's back


def test_imu_only_panel_is_sized_by_the_top_handle_not_by_its_content():
    css = (ROOT / "ros_ws" / "gui" / "css" / "charts.css").read_text()
    assert "#chart-panel.charts-hidden.imu-shown {\n    height: var(--chart-panel-height);" in css
    assert "#chart-panel.charts-hidden.imu-shown #imu-chart-group {\n    flex: 1 1 0;" in css
