# -*- coding: utf-8 -*-
"""McapSource (Phase 4 design § 3/§ 4): the Source over a module worker, under node.

A scripted fake worker serves the Python oracle's chunk records (``convert()`` output, exported as
JSON by ``_replay_chunks.py``) through the real ``mcap-worker.js`` message protocol, so everything
here is the SHIPPED ``sources.js``/``chunk.js``/``slot.js`` against records shaped exactly like
the real decode's output (the decode itself is ``test_replay_mcap_oracle.py``). No browser: the
real Worker/Range path is covered by the oracle's http readable and the box smoke test.
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

from tests.ros._replay_chunks import export_chunks
from tests.ros._replay_fixture import write_bag

REPO = Path(__file__).resolve().parents[2]
GUI = REPO / "ros_ws" / "gui"
JS = REPO / "tests" / "ros" / "js"
if str(GUI) not in sys.path:
    sys.path.insert(0, str(GUI))

from replay import schema  # noqa: E402
from replay.convert import convert  # noqa: E402

DURATION = 100.0   # 10 slots: /skills/attempt's last message (27.7 s) is > 60 s before slot 9


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
    d = tmp_path_factory.mktemp("mcapsrc")
    info = write_bag(d / "bag_0.mcap", DURATION, seed=4)
    cache = d / "cache"
    m = convert(str(d / "bag_0.mcap"), str(cache))
    assert m["status"] == "complete" and len(m["chunks"]) == 10
    sb = d / "sandbox"
    sb.mkdir()
    for name in ("chunk.js", "sources.js", "slot.js"):
        shutil.copy(GUI / "js" / "replay" / name, sb / name)
    shutil.copy(JS / "replay_test_support.js", sb / "replay_test_support.js")
    shutil.copy(JS / "replay_mcap_source_harness.js", sb / "replay_mcap_source_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    chunks = d / "chunks"
    export_chunks(cache, chunks)
    proc = subprocess.run([NODE, str(sb / "replay_mcap_source_harness.js"), str(chunks)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    if proc.returncode != 0:
        pytest.fail("node harness failed in %s\n%s" % (sb, proc.stderr.decode("utf-8", "replace")))
    return info, m, json.loads(proc.stdout.decode("utf-8"))


def test_open_range_status_and_topics(out):
    _info, m, o = out
    r = o["open"]
    assert r["before"] == {"t0": 0, "t1": 0, "frontier": 0}
    assert r["range"] == {"t0": m["t0"], "t1": m["t1"], "frontier": m["t1"]}   # a sealed source: frontier == t1
    assert r["status"] == {"state": "complete", "chunksDone": 10, "chunksTotal": 10, "dropped": {}}
    assert r["topic_set"] == r["manifest_topics"]
    assert r["log"][0] == "open"
    assert r["events"] and set(r["events"]) == {"complete"}


def test_slot_math_equals_the_oracles_chunk_bounds(out):
    r = out[2]["open"]
    assert r["bounds_ok"] is True and r["index_ok"] is True


def test_load_shapes_dedupes_and_peeks(out):
    r = out[2]["load"]
    assert r["same_object"] is True and r["loads_posted"] == 1     # in-flight dedupe + memo
    assert r["peek_hit"] is True and r["peek_miss"] is None
    assert r["t_is_f64"] is True and r["n"] == r["want_n"] > 0
    assert r["t_equal"] is True and r["cols_equal"] is True
    assert r["hydrate0"]                             # a hydrated nested row
    assert r["range_error"] == "RangeError" and r["neg_error"] == "RangeError"
    assert [c["i"] for c in r["window"]] == [0, 1] and all(c["topics"] == ["/robot_state"] for c in r["window"])


def test_latest_before_reaches_past_sixty_seconds_via_the_worker(out):
    info, m, o = out
    r = o["latest"]
    assert r["far_dist"] > 60.0                      # the old 6-slot scan-back would have given up
    assert r["far_t"] == r["last_skill"] and r["posted_latest"] == 1
    want = max(t for t, _ in info["topics"]["/skills/attempt"]) / 1e9
    assert abs(r["far_t"] - want) < 1e-6
    assert r["unknown_topic"] is None and r["before_t0"] is None


def test_latest_before_is_served_from_a_resident_slot_without_the_worker(out):
    r = out[2]["latest"]
    assert r["memo_served"] is True and r["memo_t"] is not None


def test_timeline_202_202_200_fires_on_change_once_and_never_fetches_itself(out):
    r = out[2]["timeline"]
    assert r["calls_after_open"] == 1                # open() fires the first GET (it enqueues the pass)
    assert r["calls_after_timeline"] == 1            # timeline() itself never touches the network
    assert r["partial_first"] is True
    assert r["poll_ms"] == [1234]                    # the source re-polls at overviewPollMs on a 202
    assert r["calls_final"] == 3 and r["queue_left"] == 0
    assert r["timeline_is_overview"] is True
    assert r["on_change_during_poll"] == 1


def test_overview_refusal_is_terminal_and_reported(out):
    r = out[2]["timeline_refused"]
    assert r == {"partial": True, "unavailable": "no_index", "queued": 0}


def test_overview_network_error_backs_off_then_recovers(out):
    r = out[2]["timeline_backoff"]
    assert r["backoff"] == [100, 200] and r["got_overview"] is True and r["calls"] == 3


def test_worker_errors_become_reason_and_refused_status(out):
    e = out[2]["errors"]
    for code in ("no_index", "compressed", "in_progress", "changed", "empty"):
        assert e[code] == {"reason": code, "status": "refused"}, code
    assert e["load"] == {"reason": "changed"} and e["load_ok_after"] == 0


def test_close_rejects_pending_and_terminates_the_worker(out):
    c = out[2]["close"]
    assert c == {"pending_reason": "closed", "terminated": True, "load_after_close": "closed"}


def test_main_thread_slot_math_equals_the_worker_copy_and_python(tmp_path):
    sb = tmp_path / "sb"
    (sb / "js" / "replay").mkdir(parents=True)
    (sb / "lib").mkdir()
    for name in ("slot.js", "mcap-decode.js"):
        shutil.copy(GUI / "js" / "replay" / name, sb / "js" / "replay" / name)
    shutil.copy(GUI / "lib" / "mcap-bundle.min.js", sb / "lib" / "mcap-bundle.min.js")
    shutil.copy(JS / "replay_slot_pin_harness.js", sb / "replay_slot_pin_harness.js")
    (sb / "package.json").write_text('{"type": "module"}\n')
    vec = [[1791532610.0, 1791532600.0], [1.0000000000000002e9 + 10, 1e9], [0.3, 0.1], [29.999999999, 0],
           [30, 0], [9.999999999999998, 0], [1791532609.9999998, 1791532600.003], [1791532630.003, 1791532600.003],
           [40, 0], [-0.5, 0]]
    proc = subprocess.run([NODE, str(sb / "replay_slot_pin_harness.js"), json.dumps(vec)],
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=60)
    assert proc.returncode == 0, proc.stderr.decode()
    got = json.loads(proc.stdout.decode())
    for (t, t0), (main, worker) in zip(vec, got):
        assert main == worker == schema.chunk_index(t, t0), (t, t0, main, worker)
