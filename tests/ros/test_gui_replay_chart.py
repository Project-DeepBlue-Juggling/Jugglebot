# -*- coding: utf-8 -*-
"""Replay chart store (Phase 2 unit U4): chunk-derived columns == live onTelemetryData.

A synthetic bag is converted with the production ``replay.convert.convert``; node loads
VERBATIM copies of telemetry-charts.js / chart-store.js / chunk.js (+ clock, event-store,
geometry-config, stub 3D models, fake DOM/uPlot) via ``tests/ros/js/replay_chart_harness.js``.
The harness feeds the same records through the real live path (``onTelemetryData``, with the
main.js legs/hand latches) and through the replay store and compares every series.
"""
from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from tests.ros._replay_fixture import write_bag

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))

from replay.convert import convert  # noqa: E402

HARNESS = REPO / "tests" / "ros" / "js" / "replay_chart_harness.js"


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
    d = tmp_path_factory.mktemp("chart")
    write_bag(d / "bag_0.mcap", 35.0, seed=5)
    cache = d / "cache"
    m = convert(str(d / "bag_0.mcap"), str(cache))
    assert m["status"] == "complete" and len(m["chunks"]) == 4

    sb = d / "sandbox"
    (sb / "replay").mkdir(parents=True)
    for name in ("telemetry-charts.js", "clock.js", "event-store.js", "geometry-config.js"):
        shutil.copy(GUI / "js" / name, sb / name)
    for name in ("chunk.js", "chart-store.js"):
        shutil.copy(GUI / "js" / "replay" / name, sb / "replay" / name)
    (sb / "stewart-model.js").write_text("export function setStewartHighlight() {}\n")
    (sb / "ball-butler-model.js").write_text("export function setBallButlerHighlight() {}\n")
    shutil.copy(GUI / "lib" / "msgpack.min.js", sb / "msgpack.min.cjs")
    shutil.copy(HARNESS, sb / "replay_chart_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')

    proc = subprocess.run([NODE, str(sb / "replay_chart_harness.js"), str(cache)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=180)
    if proc.returncode != 0:
        pytest.fail("node harness failed in %s\n%s" % (sb, proc.stderr.decode("utf-8", "replace")))
    return json.loads(proc.stdout.decode("utf-8"))


def test_node_check_module():
    for rel in ("js/replay/chart-store.js", "js/telemetry-charts.js"):
        r = subprocess.run([NODE, "--check", str(GUI / rel)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        assert r.returncode == 0, r.stderr.decode()


def test_replay_columns_equal_live_on_telemetry_data(out):
    assert out["n_charts"] == 9
    assert out["live_lengths"] == out["replay_lengths"]
    assert out["live_lengths"][0] == 3500 and out["live_lengths"][8] == 0
    assert out["equality"]["cmp"] > 50000
    assert out["equality"]["bad"] == 0


def test_join_rules(out):
    j = out["join"]
    # leg echo stale (> 1 s since the last echo) -> NaN, and only inside the gap
    assert j["nanInGap"] > 20 and j["finiteInGap"] == 0
    assert j["nanOutside"] == 0
    # hand is latest-before with no staleness: never NaN once a hand sample exists
    assert j["handNan"] == 0
    # the fixture bag has 7 motors: BB axes (7, 8) are never fed, like the live push loop
    assert j["axis7Samples"] == 0 or j["bbNan"] == j["axis7Samples"]


def test_store_swap_and_restore_are_exact(out):
    assert out["swap_equality"]["bad"] == 0
    assert out["inert"] is True
    assert out["exit_equality"]["bad"] == 0
    assert out["live_resumes"] is True


def test_buffer_is_immutable_across_rebuilds(out):
    im = out["immut"]
    assert all(im.values()), im
    assert out["open_chunk"]["second"] > out["open_chunk"]["first"] == 5


def test_playhead_moves_only_the_scale_and_span_is_clamped(out):
    assert out["window"]["span"] == 120 and out["window"]["cap"] == 120
    p = out["playhead"]
    assert p["centred"] and p["only_scale"] and p["span"] == pytest.approx(10.0)
    assert out["wheel_span"] == pytest.approx(120.0)
    assert out["wheel_centre_offset"] == pytest.approx(0.0, abs=1e-6)
    assert out["select"]["span"] == pytest.approx(8.0) and out["select"]["centred"]
    assert out["select"]["seek"] == pytest.approx(34.0, abs=1e-6)
    assert all(out["invalidate"].values())


def test_unit_toggle_during_replay_converts_parked_live_store(out):
    ut = out["unit_toggle"]
    assert ut["converted"] is True  # the toggle really changes pos/vel columns
    assert ut["vs_direct"] == pytest.approx(0.0, abs=1e-6)
