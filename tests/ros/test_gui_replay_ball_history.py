# -*- coding: utf-8 -*-
"""Far-tier ball history (ros_ws/gui/js/replay/ball-history.js) over real typed /balls chunks with fake source/engine/cache/layer.

``ball_history_harness.js`` drives the shipped module (verbatim copies in the sandbox).
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
HARNESS = REPO / "tests" / "ros" / "js" / "ball_history_harness.js"


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
    sb = tmp_path_factory.mktemp("ball_history")
    (sb / "replay").mkdir()
    for name in ("chunk.js", "ball-history.js"):
        shutil.copy(JS / "replay" / name, sb / "replay" / name)
    shutil.copy(HARNESS, sb / "ball_history_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "ball_history_harness.js")], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def test_node_check_modules():
    for rel in ("js/replay/ball-history.js", "js/ball-history-layer.js"):
        r = subprocess.run([NODE, "--check", str(JS.parent / rel)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        assert r.returncode == 0, (rel, r.stderr.decode())


def test_fill_is_playhead_slot_then_backward_then_slack_and_every_load_is_a_filtered_lite(out):
    f = out["fill"]
    assert f["order"] == [15, 14, 13, 12, 11, 10, 9, 8, 16]   # window [centre-60-10, centre+10] around slot 15
    assert f["all_lite"] is True and f["all_filtered"] is True
    assert f["status"]["slots"] == 9 and f["status"]["inflight"] is None


def test_the_layer_gets_the_slot_set_once_per_frame_sorted(out):
    b = out["batched"]
    assert b["frames_queued"] == 1 and b["calls_before_frame"] == 0     # nine arrivals, ONE queued publish
    assert b["calls"] == [[8, 9, 10, 11, 12, 13, 14, 15, 16]]
    assert out["fill"]["sorted_publish"] is True


def test_rows_are_compact_typed_and_decimated_to_10ms_per_id(out):
    d = out["decimation"]
    assert d["types"] == ["Float64Array", "Int32Array", "Float32Array"]
    assert d["ids"] == [1, 2] and d["mono"] is True
    assert d["n"] == 1000 + 500                                          # id 1 for 10 s + id 2 for 5 s, at 100 Hz
    assert d["min_dt_per_id"] >= 0.01 - 1e-6


def test_retarget_only_after_the_playhead_moves_more_than_10_s(out):
    r = out["retarget"]
    assert r["small"] == {"loads": 0, "centre": 1150}                    # 5 s: nothing happens
    assert r["big"]["centre"] == 1165 and r["big"]["new"] == [17]       # 15 s: retargets, one new forward slot
    assert r["far"] == [0, 1, 2, 3]                                      # a far seek keeps only [p-70, p+10] = slots 0..3 (clamped), the rest evicted
    assert r["status"]["slots"] == 4


def test_inactive_at_5s_or_less_clears_and_loads_nothing_then_refills_when_the_tail_grows(out):
    t = out["tail"]
    assert t["inactive"]["active"] is False and t["inactive"]["slots"] == 0 and t["inactive"]["last"] == "clear"
    assert t["inactive"]["loads_while_inactive"] == 0
    assert t["regrown_loads"][:1] == [10] and sorted(t["regrown"]) == t["regrown"]   # centre 1100: nearest-first backward from slot 10


def test_paused_while_buffering_or_the_cache_is_loading(out):
    p = out["paused"]
    assert p["idle_loads"] == 0 and p["resumed"] is True
    assert p["busy_loads"] == 0 and p["busy_resumed"] is True


def test_a_resident_full_chunk_is_read_in_place(out):
    r = out["resident"]
    assert 15 not in r["loaded"] and 14 not in r["loaded"]
    assert 15 in r["slots"] and 14 in r["slots"]


def test_dispose_clears_the_layer_and_stops_work(out):
    d = out["dispose"]
    assert d["last"] == "clear" and d["after_move_loads"] == 0


def test_a_load_in_flight_across_a_far_retarget_is_ignored(out):
    c = out["cancel"]
    assert c["inflight"] == 15 and 15 not in (c["last"] or [])
