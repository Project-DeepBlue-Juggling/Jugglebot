# -*- coding: utf-8 -*-
"""Replay browser feed: chunk shape + SessionBufferSource (+ McapSource agreeing with it).

Phase 2 unit U1. Python is ground truth: a synthetic bag is written with
``tests/ros/_replay_fixture.py::write_bag`` (which also returns the plain dict
of every message), converted with the production ``replay.convert.convert``,
and ``js/replay/chunk.js`` / ``sources.js`` are run under node over the
oracle's chunks exported as JSON (``_replay_chunks.py``; ``tests/ros/js/replay_feed_harness.js``).
The sandbox holds VERBATIM copies of the shipped modules. The McapSource contract (slot math,
worker protocol, overview poll, errors) is ``test_gui_replay_mcap_source.py``.

Non-finite floats: rosbridge_library/internal/message_conversion.py (Foxy,
``_from_inst``) maps NaN/+-Inf floats to None -> JSON null. The decoder keeps
them as real NaN/Inf in ``cols`` and ``hydrate`` mirrors rosbridge (null); the
test builds a chunk record carrying NaN/Inf (tagged for JSON) and pins it.
"""
from __future__ import annotations

import glob
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from tests.ros._replay_chunks import export_chunks, tagged
from tests.ros._replay_fixture import write_bag

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))

from replay.convert import convert  # noqa: E402

HARNESS = REPO / "tests" / "ros" / "js" / "replay_feed_harness.js"
DURATION = 35.0
T_START_NS = 1_791_532_600_000_000_000


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


def _close(a, b):
    """Deep equality with exact scalars (msgpack/CDR doubles round-trip exactly)."""
    if isinstance(a, dict):
        return isinstance(b, dict) and a.keys() == b.keys() and all(_close(a[k], b[k]) for k in a)
    if isinstance(a, list):
        return isinstance(b, list) and len(a) == len(b) and all(_close(x, y) for x, y in zip(a, b))
    return a == b


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    d = tmp_path_factory.mktemp("feed")
    info = write_bag(d / "bag_0.mcap", DURATION, seed=3)
    cache = d / "cache"
    m = convert(str(d / "bag_0.mcap"), str(cache))
    assert m["status"] == "complete" and len(m["chunks"]) == 4

    sandbox = d / "sandbox"
    sandbox.mkdir()
    for name in ("chunk.js", "sources.js", "slot.js"):
        shutil.copy(GUI / "js" / "replay" / name, sandbox / name)
    shutil.copy(HARNESS.parent / "replay_test_support.js", sandbox / "replay_test_support.js")
    chunks = d / "chunks"
    assert export_chunks(cache, chunks) == 4
    shutil.copy(HARNESS, sandbox / "replay_feed_harness.js")
    (sandbox / "package.json").write_text('{"type": "module"}\n')
    nan_chunk = {"format": 2, "i": 0, "t0": 0.0, "t1": 10.0, "topics": {"/nan": {
        "type": "x/msg/Y", "n": 2, "t": [0.0, 1.0],
        "cols": {"x": [float("nan"), 1.5], "y.z": [float("inf"), float("-inf")],
                 "arr": [[1.0, float("nan")], [2.0, 3.0]],
                 "objs": [[{"v": float("nan"), "w": 1}], []]}}}}
    (sandbox / "nan_chunk.json").write_text(json.dumps(tagged(nan_chunk)))

    proc = subprocess.run([NODE, str(sandbox / "replay_feed_harness.js"), str(chunks), str(sandbox)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=180)
    if proc.returncode != 0:
        pytest.fail("node harness failed in %s\n%s" % (sandbox, proc.stderr.decode("utf-8", "replace")))
    return info, m, json.loads(proc.stdout.decode("utf-8"))


def test_hydrate_equals_python_dict_for_every_record(run):
    info, _m, out = run
    for topic in ("/robot_state", "/mocap_data", "/orchestrator_state", "/skills/attempt"):
        want = info["topics"][topic]
        got = out["hydrated"][topic]
        assert len(got) == len(want) > 0, topic
        for (t_ns, plain), row in zip(want, got):
            assert abs(row["t"] - t_ns / 1e9) < 1e-6, topic
            assert _close(json.loads(json.dumps(plain)), dict(row["msg"])), (topic, t_ns)


def test_flatten_of_hydrate_roundtrips_the_columns(run):
    _i, _m, out = run
    assert out["roundtrip"]["rows"] > 2000
    assert out["roundtrip"]["mismatches"] == 0


def test_hydrator_and_flatten_rules(run):
    out = run[2]
    # dotted paths nest; array columns are assigned as-is; undefined entries are omitted
    assert out["hydrator"] == {"a": {"b": 1, "c": [1, 2]}}
    # typed arrays -> plain arrays, message arrays stay ONE column, null is a scalar
    assert out["flatten_typed"] == {"a.b": [1, 2], "l": [{"x": 1}], "n": None}


def test_index_latest_before_edges(run):
    idx = run[2]["index"]
    assert idx["before"] == -1
    assert idx["exact"] == 5
    assert idx["between"] == 5
    assert idx["after"] == idx["last_k"]


def test_non_finite_floats_become_null_like_rosbridge(run):
    nan = run[2]["nan"]
    assert nan[0] == {"x": None, "y": {"z": None}, "arr": [1.0, None], "objs": [{"v": None, "w": 1}]}
    assert nan[1] == {"x": 1.5, "y": {"z": None}, "arr": [2.0, 3.0], "objs": []}
    assert run[2]["nan_cols_untouched"] is True  # decoded columns keep the real NaN


def test_session_buffer_ring_horizon_and_sidecar(run):
    r = run[2]["session"]
    assert r["horizon"] == 600
    assert r["max_chunks"] <= 61
    assert r["chunks_final"] <= 61
    assert r["oldest_age"] <= 600 + 10 and r["oldest_age"] >= 590
    assert r["range_t1_is_now"] is True
    # on-change topic recorded once at base, 700 s ago, still resolves via the sidecar
    assert r["sidecar"] == {"t": 0, "msg": {"data": "JOG"}}
    assert r["before_ring"] is None
    mid = r["mid"]
    assert mid["x"] == mid["expect"] and mid["b"] == 2 * mid["x"]
    assert mid["arr"] == [mid["x"], mid["x"] + 1] and mid["motors"] == [{"p": mid["x"]}]
    assert r["window_chunks"] == sorted(r["window_chunks"]) and len(r["window_chunks"]) == 3
    assert r["window_sealed"] == [True, True, False]


def test_session_snapshot_is_frozen_and_clear(run):
    r = run[2]["session"]
    assert r["snapshot_rows"][0] == r["snapshot_rows"][1]      # later records/eviction/write on the snapshot do not change it
    assert r["live_chunks_after_jump"] <= 61
    assert r["cleared"] == [0, 0]
    assert r["snapshot_survives_clear"] is True


def test_session_and_recording_agree_on_the_same_data(run):
    r = run[2]["agree"]
    assert r["windows_equal"] is True
    assert all(r["lb_equal"]) and len(r["lb_equal"]) == 15
    assert r["session_band"] == ["IDLE", "LEVELLING", "ACTIVE"]
    assert r["session_presence_topics"] == ["/orchestrator_state", "/robot_state", "/skills/attempt"]
