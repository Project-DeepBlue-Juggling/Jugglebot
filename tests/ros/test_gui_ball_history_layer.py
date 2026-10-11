# -*- coding: utf-8 -*-
"""js/ball-history-layer.js (far-tier ball flights) under node with the vendored Three.js.

Sandbox: shipped ball-history-layer.js / marker-palette.js plus ``node_modules/three`` (a copy of the vendored
three.module.js with a package.json), so the bare ``import 'three'`` resolves exactly as the browser importmap does.
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
JS = REPO / "ros_ws" / "gui" / "js"
THREE = REPO / "ros_ws" / "gui" / "lib" / "three" / "build" / "three.module.js"
HARNESS = REPO / "tests" / "ros" / "js" / "ball_history_layer_harness.js"


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
    sb = tmp_path_factory.mktemp("ball_history_layer")
    for name in ("ball-history-layer.js", "marker-palette.js"):
        shutil.copy(JS / name, sb / name)
    t = sb / "node_modules" / "three"
    t.mkdir(parents=True)
    shutil.copy(THREE, t / "three.module.js")
    (t / "package.json").write_text('{"name": "three", "type": "module", "main": "three.module.js"}\n')
    shutil.copy(HARNESS, sb / "ball_history_layer_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "ball_history_layer_harness.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def test_segments_join_same_id_samples_within_150ms_only(out):
    j = out["join"]
    assert j["segments"] == 4 and j["vertices"] == 8 and j["draw"] == 8
    assert j["xs"] == [0, 1, 5, 6, 1, 2, 3, 4] or sorted(zip(j["xs"][::2], j["xs"][1::2])) == [(0, 1), (1, 2), (3, 4), (5, 6)]


def test_robot_mm_maps_to_three_metres_with_ball_colour_and_epoch_relative_time(out):
    m = out["map"]
    assert m["p"] == pytest.approx([1.0, 3.0, -2.0], abs=1e-6)
    assert m["colour_ok"] is True
    assert m["epoch"] == 50 and m["t"] == pytest.approx([0.0, 0.01], abs=1e-6)


def test_segments_join_across_slot_boundaries_and_slots_may_arrive_unsorted(out):
    c = out["cross"]
    assert c["segments"] == 3 and c["slots"] == 2 and c["epoch"] == 9.98


def test_render_is_uniform_only(out):
    r = out["render"]
    assert r["versions_same"] is True and r["data_same"] is True and r["rebuilds_same"] is True    # no buffer write, no rebuild
    assert r["v1"] == 4 and r["v2"] == 4 and r["visible"] is True
    assert r["uNow"] == pytest.approx(0.3, abs=1e-6) and r["uTail"] == 8.0
    assert r["hidden_at_tail0"] is True and r["cleared"] == [0, False, 0]
    assert r["shader_discards"] is True


def test_overflow_drops_the_oldest_slots_first_and_counts(out):
    o = out["overflow"]
    assert o["droppedSlots"] == 1 and o["droppedSegments"] == 2
    assert o["slots"] == 2 and o["segments"] == 4 and o["vertices"] == 8 and o["epoch"] == 20


def test_max_vertices_is_even(out):
    assert out["ctor"]["odd_max"] == 6


def test_buffers_are_lazy_sized_to_the_data_and_released_on_clear(out):
    # no 28 MB at page load: capacity 0 until flights arrive, then the next power of two >= 4096 that holds them
    z = out["lazy"]
    assert z["before"] == 0
    assert z["after"] == 8192 and z["posLen"] == 8192 * 3
    assert z["cleared"] == 0 and z["hasPos"] is False


def test_markers_have_own_key_space_and_palette_colour(out):
    m = out["markers"]
    assert m["vertices"] == 4          # ball id 1 (1 seg) + label 1 (1 seg); label 0 has one sample (no seg): ids never cross-join
    assert m["ball_colour"] is True and m["marker_colour"] is True and m["palette_differs"] is True
    assert m["marker_x"] == [100, 200]


def test_marker_only_slot_draws_and_hides_at_tail_zero(out):
    m = out["marker_only"]
    assert m["segments"] == 1 and m["epoch"] == 5 and m["r0"] == 0 and m["r5"] == 2


def test_cap_drops_oldest_marker_rows_before_any_ball(out):
    c = out["cap"]
    assert c["a"] == [7, 3, 1, 2, 0]    # cap 7 of 9: only the OLDEST slot's marker rows (2 segs) go; all 3 ball slots kept
    assert c["b"] == [3, 3, 0]          # cap 3: every marker row dropped (oldest first), the 3 ball segments survive
    assert c["c"] == [2, 2, 3, 1]       # cap 2: markers all gone, only then the oldest whole slot
