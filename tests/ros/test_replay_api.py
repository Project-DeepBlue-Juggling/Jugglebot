"""Replay API tests: real gui_server on an ephemeral port, stub conversion worker."""
from __future__ import annotations

import argparse
import gzip
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

import msgpack
import pytest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
GUI = os.path.join(REPO, "ros_ws", "gui")
if GUI not in sys.path:
    sys.path.insert(0, GUI)

import gui_server  # noqa: E402
from replay import cache, schema  # noqa: E402

MAGIC = b"\x89MCAP0\r\n"

STUB = '''
import argparse, gzip, json, os, sys, time
import msgpack
sys.path.insert(0, {gui!r})
from replay import schema
ap = argparse.ArgumentParser()
ap.add_argument("--bag"); ap.add_argument("--out")
a = ap.parse_args()
FAIL = {fail!r}
SLEEP = {sleep!r}
AFTER = {after!r}
st = os.stat(a.bag)
rid = os.path.basename(os.path.dirname(a.bag))
os.makedirs(a.out, exist_ok=True)
man = schema.new_manifest(rid, a.bag, st.st_size, st.st_mtime, True,
                          "2026-10-10T00:00:00", "stub", 3)
def w():
    tmp = os.path.join(a.out, "manifest.tmp")
    with open(tmp, "w") as f: json.dump(man, f)
    os.replace(tmp, os.path.join(a.out, schema.MANIFEST))
w()
if FAIL:
    man["status"] = "failed"; man["error"] = "boom"; w(); sys.exit(1)
for i in range(3):
    time.sleep(SLEEP)
    rec = {{"format": 1, "i": i, "t0": 10.0*i, "t1": 10.0*(i+1), "topics": {{}}}}
    with open(os.path.join(a.out, schema.chunk_name(i)), "wb") as f:
        f.write(gzip.compress(msgpack.packb(rec)))
    man["chunks_done"] = i + 1; man["t0"] = 0.0; w()
man["status"] = "complete"; man["t1"] = 30.0; man["chunks_total"] = 3
man["completed_at"] = "2026-10-10T00:00:01"; man["chunks"] = []
with open(os.path.join(a.out, schema.OVERVIEW), "w") as f:
    json.dump({{"format": 1, "t0": 0.0, "t1": 30.0, "bands": [], "ticks": [], "presence": {{}}}}, f)
w()
time.sleep(AFTER)
'''


def write_stub(d, name="stub_worker", fail=False, sleep=0.05, after=0.0):
    os.makedirs(str(d), exist_ok=True)
    with open(os.path.join(str(d), name + ".py"), "w") as f:
        f.write(STUB.format(gui=GUI, fail=fail, sleep=sleep, after=after))
    return name


def make_bag(root, rid, closed=True, age_s=3600.0, metadata=False, extra=b""):
    d = os.path.join(root, rid)
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, rid + "_0.mcap")
    with open(p, "wb") as f:
        f.write(b"x" * 50 + extra + (MAGIC if closed else b"zz"))
    t = time.time() - age_s
    os.utime(p, (t, t))
    if metadata:
        with open(os.path.join(d, "metadata.yaml"), "w") as f:
            f.write("rosbag2_bagfile_information:\n  duration:\n    nanoseconds: 12500000000\n"
                    "  starting_time:\n    nanoseconds_since_epoch: 1700000000000000000\n"
                    "  message_count: 4242\n")
    return p


class Srv:
    def __init__(self, tmp_path, worker_python=None, stub_kwargs=None, min_free_gb=0.0,
                 cap_gb=10.0):
        self.root = str(tmp_path / "bags")
        self.cache = str(tmp_path / "cache")
        os.makedirs(self.root)
        self.gui_dir = str(tmp_path / "guidir")
        mod = write_stub(self.gui_dir, **(stub_kwargs or {}))
        # the stub imports replay.schema from the real GUI dir via sys.path insert
        args = argparse.Namespace(
            port=0, host="127.0.0.1", rosbags_dir=self.root, cache_dir=self.cache,
            worker_python=worker_python or sys.executable, cache_cap_gb=cap_gb,
            min_free_gb=min_free_gb, worker_module=mod, gui_dir=self.gui_dir)
        self.server, self.backend = gui_server.make_server(args)
        self.port = self.server.server_address[1]
        self.t = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.t.start()

    def req(self, path, method="GET"):
        r = urllib.request.Request("http://127.0.0.1:{}{}".format(self.port, path),
                                   method=method, data=b"" if method == "POST" else None)
        try:
            with urllib.request.urlopen(r, timeout=10) as resp:
                return resp.status, resp.headers, resp.read()
        except urllib.error.HTTPError as e:
            return e.code, e.headers, e.read()

    def jreq(self, path, method="GET"):
        code, h, body = self.req(path, method)
        return code, json.loads(body.decode())

    def wait_status(self, rid, want, timeout=5.0):
        end = time.time() + timeout
        st = None
        while time.time() < end:
            _c, st = self.jreq("/api/replay/recordings/{}/status".format(rid))
            if st["status"] in want:
                return st
            time.sleep(0.05)
        raise AssertionError("status never reached {}: {}".format(want, st))

    def wait_idle(self, rid, timeout=10.0):
        """Block until the worker queue no longer holds `rid` (process exited)."""
        end = time.time() + timeout
        while time.time() < end:
            if self.backend.queue_position(rid) is None:
                return
            time.sleep(0.02)
        raise AssertionError("worker never went idle for " + rid)

    def close(self):
        self.server.shutdown()
        self.server.server_close()


@pytest.fixture
def srv(tmp_path):
    s = Srv(tmp_path)
    yield s
    s.close()


A, B, C = "2026-10-01_10-00-00", "2026-10-02_10-00-00", "2026-10-03_10-00-00"


def test_listing_shape_order_exclusions(srv):
    make_bag(srv.root, A, metadata=True)
    make_bag(srv.root, B)
    make_bag(srv.root, C, closed=False, age_s=1.0)
    make_bag(srv.root, "not-a-recording")
    d = os.path.join(srv.root, "2026-10-04_10-00-00")
    make_bag(srv.root, "2026-10-04_10-00-00")
    open(os.path.join(d, "second.mcap"), "wb").close()
    code, h, body = srv.req("/api/replay/recordings")
    assert code == 200
    assert h["Content-Type"] == "application/json; charset=utf-8"
    assert h["Cache-Control"] == "no-cache"
    assert h["Access-Control-Allow-Origin"] == "*"
    j = json.loads(body)
    assert [r["id"] for r in j["recordings"]] == [C, B, A]
    assert set(j) == {"recordings", "cache", "worker_available"}
    assert set(j["cache"]) == {"bytes", "cap_bytes", "disk_free_bytes"}
    assert j["worker_available"] is True
    by = {r["id"]: r for r in j["recordings"]}
    assert by[A]["closed"] and not by[A]["in_progress"]
    assert by[A]["duration_s"] == pytest.approx(12.5)
    assert by[A]["message_count"] == 4242
    assert by[A]["start_ns"] == 1700000000000000000
    assert by[B]["duration_s"] is None and by[B]["cache"] == "none"
    assert not by[C]["closed"] and by[C]["in_progress"]
    for k in ("path", "size_bytes", "mtime"):
        assert k in by[A]


def test_not_closed_old_file_is_not_in_progress(srv):
    make_bag(srv.root, A, closed=False, age_s=100.0)
    _c, j = srv.jreq("/api/replay/recordings")
    r = j["recordings"][0]
    assert not r["closed"] and not r["in_progress"]


def test_open_convert_serve_and_reopen(srv):
    make_bag(srv.root, A)
    code, st = srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    assert code == 200 and st["status"] in ("queued", "converting")
    st = srv.wait_status(A, ("complete",))
    assert st["chunks_done"] == 3
    code, h, body = srv.req("/api/replay/recordings/{}/manifest".format(A))
    assert code == 200 and json.loads(body)["status"] == "complete"
    code, h, body = srv.req("/api/replay/recordings/{}/overview".format(A))
    assert code == 200 and json.loads(body)["t1"] == 30.0
    code, h, body = srv.req("/api/replay/recordings/{}/chunks/0".format(A))
    assert code == 200
    assert h["Content-Encoding"] == "gzip"
    assert h["Content-Type"] == "application/msgpack"
    assert h["Cache-Control"] == "no-cache"
    assert msgpack.unpackb(gzip.decompress(body), raw=False)["i"] == 0
    assert srv.req("/api/replay/recordings/{}/chunks/3".format(A))[0] == 404
    assert srv.req("/api/replay/recordings/{}/chunks/2".format(A))[0] == 200
    # reopen: complete, .opened advances
    mark = os.path.join(srv.cache, A, schema.OPENED_MARK)
    os.utime(mark, (1000, 1000))
    code, st = srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    assert code == 200 and st["status"] == "complete"
    assert os.stat(mark).st_mtime > 1e9
    _c, j = srv.jreq("/api/replay/recordings")
    assert j["recordings"][0]["cache"] == "complete"
    assert j["cache"]["bytes"] > 0


def test_status_none_and_unknown(srv):
    make_bag(srv.root, A)
    _c, st = srv.jreq("/api/replay/recordings/{}/status".format(A))
    assert st["status"] == "none"
    assert srv.jreq("/api/replay/recordings/{}/open".format(B), "POST")[0] == 404
    assert srv.req("/api/replay/recordings/{}/manifest".format(A))[0] == 404


def test_in_progress_refused(srv):
    make_bag(srv.root, A, closed=False, age_s=1.0)
    code, j = srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    assert code == 409 and j == {"status": "refused", "reason": "recording_in_progress"}


def test_worker_unavailable(tmp_path):
    s = Srv(tmp_path, worker_python="/nonexistent/python")
    try:
        make_bag(s.root, A)
        assert s.jreq("/api/replay/recordings")[1]["worker_available"] is False
        code, j = s.jreq("/api/replay/recordings/{}/open".format(A), "POST")
        assert code == 409 and j["reason"] == "worker_unavailable"
    finally:
        s.close()


def test_disk_low(srv, monkeypatch):
    make_bag(srv.root, A)
    monkeypatch.setattr(cache, "disk_free_bytes", lambda root: 10)
    srv.backend.min_free_bytes = 1000
    code, j = srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    assert code == 409 and j == {"status": "refused", "reason": "disk_low"}
    assert srv.jreq("/api/replay/recordings/{}/status".format(A))[1]["status"] == "none"


def _fake_complete(cache_root, rid, opened, size=1000):
    d = os.path.join(cache_root, rid)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, schema.MANIFEST), "w") as f:
        json.dump({"status": "complete", "completed_at": "2026-01-01T00:00:00"}, f)
    with open(os.path.join(d, "blob"), "wb") as f:
        f.write(b"0" * size)
    m = os.path.join(d, schema.OPENED_MARK)
    open(m, "w").close()
    os.utime(m, (opened, opened))


def test_evict_lru_unit(tmp_path):
    root = str(tmp_path)
    _fake_complete(root, A, 1000)
    _fake_complete(root, B, 2000)
    _fake_complete(root, C, 3000)
    removed = cache.evict_lru(root, 1500, keep_id=C)
    assert removed == [A, B]
    assert os.path.isdir(os.path.join(root, C))
    assert not os.path.exists(os.path.join(root, A))


def test_eviction_on_open(tmp_path):
    s = Srv(tmp_path, cap_gb=1e-6)  # 1000 byte cap
    try:
        _fake_complete(s.cache, A, 1000)
        _fake_complete(s.cache, B, 2000)
        _fake_complete(s.cache, C, 3000)
        make_bag(s.root, "2026-10-05_10-00-00")
        code, _ = s.jreq("/api/replay/recordings/2026-10-05_10-00-00/open", "POST")
        assert code == 200
        assert not os.path.exists(os.path.join(s.cache, A))
        assert not os.path.exists(os.path.join(s.cache, B))
        assert os.path.isdir(os.path.join(s.cache, "2026-10-05_10-00-00"))
    finally:
        s.close()


def test_failing_worker(tmp_path):
    s = Srv(tmp_path, stub_kwargs={"fail": True})
    try:
        make_bag(s.root, A)
        assert s.jreq("/api/replay/recordings/{}/open".format(A), "POST")[0] == 200
        st = s.wait_status(A, ("failed",))
        assert st["error"] == "boom"
        assert s.jreq("/api/replay/recordings")[1]["recordings"][0]["cache"] == "failed"
    finally:
        s.close()


def test_stale_partial_discarded_and_requeued(srv):
    make_bag(srv.root, A)
    d = os.path.join(srv.cache, A)
    os.makedirs(d)
    with open(os.path.join(d, schema.MANIFEST), "w") as f:
        json.dump({"status": "converting", "chunks_done": 1}, f)
    with open(os.path.join(d, "stale-junk"), "w") as f:
        f.write("x")
    assert srv.jreq("/api/replay/recordings/{}/status".format(A))[1]["status"] == "failed"
    code, st = srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    assert code == 200 and st["status"] in ("queued", "converting")
    srv.wait_status(A, ("complete",))
    assert not os.path.exists(os.path.join(d, "stale-junk"))


def test_bad_id(srv):
    for p in ("/api/replay/recordings/..%2Fx/open", "/api/replay/recordings/../x/status", "/api/replay/recordings/x/status"):
        code, _h, _b = srv.req(p, "POST" if p.endswith("open") else "GET")
        assert code in (400, 404)
    assert srv.req("/api/replay/recordings/..%2Fx/status")[0] == 400
    assert srv.req("/api/replay/recordings/x/chunks/0")[0] == 400


def test_static_still_served(srv):
    code, h, body = srv.req("/index.html")
    assert code == 200 and b"<html" in body.lower()
    assert h["Access-Control-Allow-Origin"] == "*"
    assert h["Cache-Control"] == "no-cache"
    code, h, _b = srv.req("/js/main.js")
    assert code == 200 and h["Content-Type"] == "application/javascript"
    assert srv.req("/api/other")[0] == 404


def test_concurrent_gets_while_worker_runs(tmp_path):
    s = Srv(tmp_path, stub_kwargs={"sleep": 0.3})
    try:
        make_bag(s.root, A)
        s.jreq("/api/replay/recordings/{}/open".format(A), "POST")
        results = []

        def go(path):
            results.append(s.req(path)[0])

        ts = [threading.Thread(target=go, args=(p,)) for p in
              ("/api/replay/recordings", "/api/replay/recordings/{}/status".format(A))]
        t0 = time.time()
        for t in ts:
            t.start()
        for t in ts:
            t.join(5)
        assert sorted(results) == [200, 200]
        assert time.time() - t0 < 2.0
        s.wait_status(A, ("complete",), timeout=8)
    finally:
        s.close()


def test_queue_runs_one_at_a_time(tmp_path):
    s = Srv(tmp_path, stub_kwargs={"sleep": 0.2})
    try:
        make_bag(s.root, A)
        make_bag(s.root, B)
        s.jreq("/api/replay/recordings/{}/open".format(A), "POST")
        _c, st = s.jreq("/api/replay/recordings/{}/open".format(B), "POST")
        assert st["status"] == "queued"
        assert s.backend.queue_position(B) in (1, 0)
        s.wait_status(B, ("complete",), timeout=8)
        assert s.jreq("/api/replay/recordings/{}/status".format(A))[1]["status"] == "complete"
    finally:
        s.close()


def _chunk_files(cache_root, rid):
    d = os.path.join(cache_root, rid)
    return [n for n in os.listdir(d) if n.startswith("chunk-") and n.endswith(".msgpack.gz")]


def test_concurrent_open_same_id(srv):
    make_bag(srv.root, A)
    results = []
    barrier = threading.Barrier(2)

    def go():
        barrier.wait()
        results.append(srv.jreq("/api/replay/recordings/{}/open".format(A), "POST"))

    ts = [threading.Thread(target=go) for _ in range(2)]
    for t in ts:
        t.start()
    for t in ts:
        t.join(15)
    assert len(results) == 2
    for code, st in results:
        assert code == 200 and st["status"] in ("queued", "converting", "complete")
    srv.wait_status(A, ("complete",))
    man = cache.read_manifest(srv.cache, A)
    assert man["status"] == "complete"
    assert len(_chunk_files(srv.cache, A)) == 3


def test_chunk_index_non_digit_is_400(srv):
    make_bag(srv.root, A)
    base = "/api/replay/recordings/{}/chunks/".format(A)
    assert srv.req(base + "%C2%B2")[0] == 400  # superscript two: isdigit() True, int() raises
    assert srv.req(base + "abc")[0] == 400


def test_source_changed_discards_complete_cache(srv):
    p = make_bag(srv.root, A)
    srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    srv.wait_status(A, ("complete",))
    srv.wait_idle(A)
    old = cache.read_manifest(srv.cache, A)["source"]["size_bytes"]
    with open(p, "ab") as f:
        f.write(b"more-bytes")
    t = time.time() - 3600
    os.utime(p, (t, t))
    new_size = os.stat(p).st_size
    assert new_size != old
    code, st = srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    assert code == 200 and st["status"] in ("queued", "converting")
    srv.wait_status(A, ("complete",))
    man = cache.read_manifest(srv.cache, A)
    assert man["source"]["size_bytes"] == new_size
    assert os.path.exists(os.path.join(srv.cache, A, schema.OPENED_MARK))


def test_midrun_manifest_invariant(tmp_path):
    s = Srv(tmp_path, stub_kwargs={"sleep": 0.2})
    try:
        make_bag(s.root, A)
        s.jreq("/api/replay/recordings/{}/open".format(A), "POST")
        seen = 0
        end = time.time() + 10
        while time.time() < end:
            st = s.jreq("/api/replay/recordings/{}/status".format(A))[1]
            if st["status"] == "complete":
                break
            code, _h, body = s.req("/api/replay/recordings/{}/manifest".format(A))
            if code != 200:
                time.sleep(0.02)
                continue
            man = json.loads(body.decode())
            if man["status"] != "converting":
                continue
            nfiles = len(_chunk_files(s.cache, A))  # read after the manifest
            assert man["chunks"] == []
            # the stub seals the file before it publishes chunks_done
            assert man["chunks_done"] <= nfiles
            seen += 1
            time.sleep(0.05)
        assert seen >= 2
        s.wait_status(A, ("complete",))
        s.wait_idle(A)
    finally:
        s.close()


def test_stale_open_while_finished_worker_still_exiting(tmp_path):
    # The worker writes its complete manifest, then lingers 1 s before exiting.
    s = Srv(tmp_path, stub_kwargs={"after": 1.0})
    try:
        p = make_bag(s.root, A)
        s.jreq("/api/replay/recordings/{}/open".format(A), "POST")
        end = time.time() + 10
        while time.time() < end:
            m = cache.read_manifest(s.cache, A)
            if m and m.get("status") == schema.STATUS_COMPLETE:
                break
            time.sleep(0.02)
        assert s.backend.queue_position(A) is not None  # still lingering
        with open(p, "ab") as f:
            f.write(b"more-bytes")
        t = time.time() - 3600
        os.utime(p, (t, t))
        new_size = os.stat(p).st_size
        code, st = s.jreq("/api/replay/recordings/{}/open".format(A), "POST")
        assert code == 200 and st["status"] in ("queued", "converting")
        s.wait_status(A, ("complete",), timeout=10)
        s.wait_idle(A)
        assert cache.read_manifest(s.cache, A)["source"]["size_bytes"] == new_size
    finally:
        s.close()


def _set_manifest_format(cache_root, rid, fmt):
    p = os.path.join(cache_root, rid, schema.MANIFEST)
    with open(p) as f:
        man = json.load(f)
    man["format"] = fmt
    with open(p, "w") as f:
        json.dump(man, f)


def test_old_format_cache_is_stale_discarded_and_reconverted(srv):
    make_bag(srv.root, A)
    srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    srv.wait_status(A, ("complete",))
    srv.wait_idle(A)
    _set_manifest_format(srv.cache, A, 1)
    _c, j = srv.jreq("/api/replay/recordings")
    assert j["recordings"][0]["cache"] == "stale"
    assert srv.jreq("/api/replay/recordings/{}/status".format(A))[1]["status"] != "complete"
    code, st = srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    assert code == 200 and st["status"] in ("queued", "converting")
    srv.wait_status(A, ("complete",))
    assert cache.read_manifest(srv.cache, A)["format"] == schema.FORMAT_VERSION == 2


def test_current_format_complete_cache_returns_complete_immediately(srv):
    make_bag(srv.root, A)
    srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    srv.wait_status(A, ("complete",))
    assert cache.read_manifest(srv.cache, A)["format"] == schema.FORMAT_VERSION
    code, st = srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    assert code == 200 and st["status"] == "complete"


def test_old_format_cache_chunk_and_files_404(srv):
    make_bag(srv.root, A)
    srv.jreq("/api/replay/recordings/{}/open".format(A), "POST")
    srv.wait_status(A, ("complete",))
    srv.wait_idle(A)
    base = "/api/replay/recordings/{}/".format(A)
    assert srv.req(base + "chunks/0")[0] == 200
    _set_manifest_format(srv.cache, A, 1)
    assert srv.req(base + "chunks/0")[0] == 404
    assert srv.req(base + "manifest")[0] == 404
