# -*- coding: utf-8 -*-
"""Replay chunk cache (ros_ws/gui/js/replay/cache.js), design § 6, Phase 2 unit U5.

``tests/ros/js/replay_cache_harness.js`` drives the real ``cache.js`` (and, for the last
property, the real ``engine.js``) against a scripted source whose ``load`` latency, held chunks
and growing frontier the harness controls, then prints JSON that this file asserts on.

Properties:
1. ahead bias: forward at p the loads go current -> ahead -> behind, covering +20 s / -10 s
   (+1 chunk behind); reverse mirrors;
2. cap: 30 chunks wanted -> 16 resident, the farthest never kept, a far jump evicts the old set;
3. serial fetch: never more than one load in flight, in priority order;
4. bufferedRanges: merged resident / loading runs in time order;
5. frontier: nothing at or past the frontier chunk is requested; growth fetches the missing
   chunks and only then resolves a pending ensure(); a FINAL source resolves past its end;
6. failures surface through onChange({error}); a null load is an EMPTY chunk, never null;
7. through the engine: play into a held chunk -> buffering, chunk arrives -> playing.
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
HARNESS = REPO / "tests" / "ros" / "js" / "replay_cache_harness.js"


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
    sb = tmp_path_factory.mktemp("cache")
    (sb / "replay").mkdir()
    shutil.copy(JS / "clock.js", sb / "clock.js")
    for name in ("chunk.js", "policy.js", "engine.js", "cache.js"):
        shutil.copy(JS / "replay" / name, sb / "replay" / name)
    shutil.copy(HARNESS, sb / "replay_cache_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    proc = subprocess.run([NODE, str(sb / "replay_cache_harness.js")],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    assert proc.returncode == 0, proc.stderr.decode()
    return json.loads(proc.stdout)


def test_ahead_bias_forward_and_reverse(out):
    a = out["ahead"]
    # p = chunk 50 + 3 s, span 0: forward window [493, 523] -> 49..52, +1 behind -> 48
    assert a["1"]["order"] == [50, 51, 52, 49, 48]
    assert a["1"]["resident"] == [48, 49, 50, 51, 52]
    # reverse mirrors: current, then earlier chunks first, +1 chunk later than the window
    assert a["-1"]["order"] == [50, 49, 48, 51, 52]
    # ahead 40 s / behind 10 s: forward reaches +40 s ahead and only -10 s (+1 chunk) behind
    assert a["wideFwd"] == [48, 49, 50, 51, 52, 53, 54]
    assert a["wideRev"] == [46, 47, 48, 49, 50, 51, 52]


def test_cap_keeps_nearest_and_evicts_farthest(out):
    c = out["cap"]
    assert len(c["first"]) == 16 and c["size"] == 16
    assert all(abs(i - c["cur1"]) <= 8 for i in c["first"]), c["first"]
    assert c["cur1"] in c["first"]
    assert len(c["second"]) == 16
    assert not set(c["first"]) & set(c["second"])  # a far jump evicts the whole old set
    assert all(abs(i - c["cur2"]) <= 8 for i in c["second"])


def test_serial_fetch_in_priority_order(out):
    s = out["serial"]
    assert s["maxInflight"] == 1
    assert s["order"] == [50, 51, 52, 49, 48]
    assert all(st["inflight"] <= 1 for st in s["steps"])
    assert s["steps"][0]["parked"] == [50]  # only the current chunk was requested first


def test_buffered_ranges_merge_and_order(out):
    r = out["ranges"]
    assert r["resident"] == [50]  # chunk 51 is held in flight, so nothing after it has loaded
    ranges = r["ranges"]
    t0 = 1000
    # time order: loading 48..49 (merged), resident 50, loading 51..52 (in flight + queued, merged)
    assert [(x["state"], x["t0"] - t0, x["t1"] - t0) for x in ranges] == [
        ("loading", 480, 500), ("resident", 500, 510), ("loading", 510, 530)], ranges


def test_frontier_gates_requests_and_growth_refetches(out):
    f = out["frontier"]
    assert max(f["beforeLoads"] + f["loadsAtFrontier"]) <= 2  # chunk 3 starts AT the frontier: never asked
    assert f["stillPending"] is True
    assert f["resolved"] is True
    assert {3, 4, 5} <= set(f["after"])
    assert f["resident"] == [0, 1, 2, 3, 4, 5]


def test_final_source_resolves_past_its_end(out):
    assert out["finalEnd"]["resident"] == [2, 3]
    assert max(out["finalEnd"]["loads"]) <= 3


def test_failure_surfaces_and_gap_is_empty_not_null(out):
    f = out["failure"]
    assert any(e["msg"] == "boom 1" for e in f["events"])
    assert f["peek1"] is True      # the failed chunk is simply not resident
    assert f["gapEmpty"] is True   # a null load -> empty chunk (engine would buffer forever on null)
    assert f["peek0"] is True
    assert any("failed" in s for s in out["sourceFailed"])


def test_engine_buffers_at_held_chunk_then_resumes(out):
    e = out["engine"]
    assert e["stuck"]["mode"] == "buffering"
    assert e["stuck"]["parked"] == [1]
    assert 1009 < e["stuck"]["playhead"] <= 1010.0001  # held at the chunk-1 boundary
    assert abs(e["still"] - e["stuck"]["playhead"]) < 1e-9
    assert e["resumed"]["mode"] == "playing" and e["resumed"]["playhead"] > 1010
    assert e["bufEvents"][:2] == [True, False] or e["bufEvents"][-2:] == [True, False]
