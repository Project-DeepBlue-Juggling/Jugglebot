"""Design 06 § 5(c): the overview pass (replay/overview.py) and its route.

The pass is compared with the test oracle (convert.py's overview.json) on the
synthetic fixture; the route is driven through the REAL worker subprocess
(``sys.executable`` = the venv) on an ephemeral-port server.
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from _replay_fixture import write_bag  # noqa: E402
from _replay_srv import GUI, Srv, make_bag  # noqa: E402

from replay import schema  # noqa: E402
from replay.convert import convert  # noqa: E402
from replay.overview import compute, robot_state_flags  # noqa: E402

A, B = "2026-10-01_10-00-00", "2026-10-02_10-00-00"


def _put_bag(root, rid, duration=35.0, seed=1, **kw):
    d = Path(root) / rid
    d.mkdir(parents=True, exist_ok=True)
    p = d / (rid + "_0.mcap")
    write_bag(p, duration, seed=seed, **kw)
    return str(p)


def ov_url(rid):
    return "/api/replay/recordings/{}/overview".format(rid)


def poll(srv, rid, timeout=120.0):
    """GET until the status is no longer 202; returns (code, body, seen_statuses)."""
    seen = []
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        code, j = srv.jreq(ov_url(rid))
        if code != 202:
            return code, j, seen
        seen.append(j["status"])
        time.sleep(0.1)
    raise AssertionError("overview still 202 after %s s: %r" % (timeout, seen[-3:]))


# ---- the pass vs the oracle ---------------------------------------------

@pytest.fixture(scope="module")
def indexed(tmp_path_factory):
    d = tmp_path_factory.mktemp("ov")
    p = d / "bag_0.mcap"
    write_bag(p, 35.0, seed=1)
    convert(str(p), str(d / "cache"))
    oracle = json.loads((d / "cache" / schema.OVERVIEW).read_text())
    return str(p), oracle


def test_pass_equals_the_oracle_overview(indexed):
    path, oracle = indexed
    got = compute(path)
    src = got.pop("source")
    assert src["size_bytes"] == os.path.getsize(path)
    assert got == oracle  # pins t0, presence slot binning, tick order, band segments


def test_tick_kinds_are_the_contract_vocabulary(indexed):
    got = compute(indexed[0])
    kinds = {t["kind"] for t in got["ticks"]}
    assert kinds <= set(schema.OVERVIEW_TICK_KINDS)
    assert kinds == set(schema.OVERVIEW_TICK_KINDS)  # the fixture exercises every kind


def test_robot_state_flag_walk_matches_full_decode(tmp_path):
    """The hand CDR walk (no full decode of /robot_state) agrees with rosbags on
    every fixture message, including non-empty ``error`` string arrays."""
    from mcap.reader import SeekingReader
    from replay.decode import Decoder
    p = tmp_path / "b.mcap"
    write_bag(p, 12.0, seed=3)
    dec = Decoder()
    n = 0
    with open(p, "rb") as fh:
        for sch, ch, msg in SeekingReader(fh).iter_messages():
            if ch.topic != "/robot_state":
                continue
            dec.ensure_schema(sch)
            o = dec.decode(msg.data, sch.name)
            want = tuple(bool(getattr(o, f)) for f in (
                "has_fatal_odrive_error", "has_fatal_can_error", "has_undervoltage",
                "is_homed", "levelling_complete"))
            assert robot_state_flags(msg.data) == want
            n += 1
    assert n == 1200


def test_unindexed_bag_is_refused_by_the_pass(tmp_path):
    p = tmp_path / "bag_0.mcap"
    write_bag(p, 35.0, seed=2, unindexed=True, chunk_size=16 * 1024)
    import subprocess
    r = subprocess.run([sys.executable, "-m", "replay.overview", "--bag", str(p),
                        "--out", str(tmp_path / "o.json")], cwd=GUI,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    assert r.returncode == 2 and "no_index" in r.stderr
    assert not (tmp_path / "o.json").exists()


# ---- the route, through the real worker ------------------------------------

@pytest.fixture
def srv(tmp_path):
    s = Srv(tmp_path, worker_python=sys.executable)
    assert s.backend.worker_available
    yield s
    s.close()


def test_route_202_then_200_cached_then_recomputed_on_mtime_change(srv):
    path = _put_bag(srv.root, A)
    assert srv.jreq("/api/replay/recordings")[1]["recordings"][0]["overview"] == "none"
    code, j = srv.jreq(ov_url(A))
    assert code == 202 and j["status"] in ("queued", "computing")  # the first GET starts the pass
    code, j, seen = poll(srv, A)
    assert code == 200 and seen, "never saw a 202 before the 200"
    want = compute(path)
    assert j == want
    assert json.loads(Path(srv.backend._ov_path(A)).read_text()) == want  # cache file on disk
    assert srv.jreq("/api/replay/recordings")[1]["recordings"][0]["overview"] == "ready"

    # second GET is served from the cache: a pass here would blow up
    def boom(info):
        raise AssertionError("second GET recomputed")
    srv.backend._run_pass = boom
    assert srv.jreq(ov_url(A)) == (200, want)
    del srv.backend._run_pass

    st = os.stat(path)
    os.utime(path, (st.st_atime, st.st_mtime - 1000))  # source changed -> cache is stale
    assert srv.jreq("/api/replay/recordings")[1]["recordings"][0]["overview"] == "none"
    code, j = srv.jreq(ov_url(A))
    assert code == 202
    code, j, _ = poll(srv, A)
    assert code == 200 and j["source"]["mtime"] == os.stat(path).st_mtime


def test_route_409_for_killed_and_in_progress(srv):
    make_bag(srv.root, A, kind="killed", age_s=3600.0)
    assert srv.jreq(ov_url(A)) == (409, {"status": "refused", "reason": "no_index"})
    make_bag(srv.root, B, kind="killed", age_s=1.0)
    assert srv.jreq(ov_url(B)) == (409, {"status": "refused", "reason": "recording_in_progress"})
    assert srv.jreq(ov_url("2030-01-01_00-00-00"))[0] == 404


def test_route_503_when_worker_missing(tmp_path):
    s = Srv(tmp_path, worker_python="/nonexistent/python")
    try:
        _put_bag(s.root, A, duration=11.0)
        assert s.jreq(ov_url(A)) == (503, {"status": "unavailable", "reason": "worker_unavailable"})
        assert s.jreq("/api/replay/recordings")[1]["overview_available"] is False
    finally:
        s.close()


_SLOW = '''\
import argparse, json, os, sys, time
sys.path.insert(0, {gui!r})
from replay import schema
p = argparse.ArgumentParser(); p.add_argument("--bag"); p.add_argument("--out")
a = p.parse_args()
log = os.environ["OV_LOG"]
with open(log, "a") as f: f.write("start %s\\n" % os.path.basename(a.bag))
if os.environ.get("OV_FAIL"):
    sys.stderr.write("synthetic failure\\n"); sys.exit(1)
time.sleep(0.6)
st = os.stat(a.bag)
with open(a.out, "w") as f:
    json.dump({{"format": schema.FORMAT_VERSION, "source": {{"size_bytes": st.st_size, "mtime": st.st_mtime}}}}, f)
with open(log, "a") as f: f.write("end %s\\n" % os.path.basename(a.bag))
'''


def _slow_srv(tmp_path, monkeypatch, fail=False):
    mod = tmp_path / "mods"
    mod.mkdir()
    (mod / "slow_ov.py").write_text(_SLOW.format(gui=GUI))
    log = tmp_path / "ov.log"
    monkeypatch.setenv("PYTHONPATH", str(mod))
    monkeypatch.setenv("OV_LOG", str(log))
    if fail:
        monkeypatch.setenv("OV_FAIL", "1")
    import argparse
    import gui_server
    root = tmp_path / "bags"
    root.mkdir()
    args = argparse.Namespace(port=0, host="127.0.0.1", rosbags_dir=str(root),
                              overview_dir=str(tmp_path / "overview"),
                              worker_python=sys.executable, worker_module="slow_ov")
    server, backend = gui_server.make_server(args)
    import threading
    threading.Thread(target=server.serve_forever, daemon=True).start()
    s = Srv.__new__(Srv)
    s.root, s.server, s.backend, s.port = str(root), server, backend, server.server_address[1]
    for rid in (A, B):
        make_bag(s.root, rid)
    return s, log


def test_one_slot_queue_runs_passes_one_at_a_time(tmp_path, monkeypatch):
    s, log = _slow_srv(tmp_path, monkeypatch)
    try:
        assert s.jreq(ov_url(A))[0] == 202
        code_b, jb = s.jreq(ov_url(B))
        assert code_b == 202 and jb["status"] == "queued"  # A holds the slot
        rows = {r["id"]: r["overview"] for r in s.jreq("/api/replay/recordings")[1]["recordings"]}
        assert rows[A] == "computing" and rows[B] == "queued"
        assert poll(s, B)[0] == 200 and poll(s, A)[0] == 200
        lines = log.read_text().split()
        names = [x for x in lines if x.startswith("2026")]
        events = lines[0::2]
        assert events == ["start", "end", "start", "end"], lines  # never two in flight
        assert names[0] == names[1] and names[2] == names[3] and names[0] != names[2]
    finally:
        s.close()


def test_failed_pass_is_500_with_the_error_and_sticks(tmp_path, monkeypatch):
    s, log = _slow_srv(tmp_path, monkeypatch, fail=True)
    try:
        assert s.jreq(ov_url(A))[0] == 202
        code, j, _ = poll(s, A)
        assert code == 500 and j["status"] == "failed" and "synthetic failure" in j["error"]
        assert s.jreq(ov_url(A))[0] == 500  # not silently retried
        rows = {r["id"]: r["overview"] for r in s.jreq("/api/replay/recordings")[1]["recordings"]}
        assert rows[A] == "failed"
        assert log.read_text().count("start") == 1
    finally:
        s.close()
