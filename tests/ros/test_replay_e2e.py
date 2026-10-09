"""End to end: the real GUI server drives the REAL converter worker
(``sys.executable -m replay.convert``) on the synthetic fixture bag, and the
cache it produces is served back through the API and decodes to the values the
fixture wrote. Unit 1c of plans/active/gui-rosbag-replay.md Phase 1."""
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

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
GUI = os.path.join(REPO, "ros_ws", "gui")
for _p in (GUI, os.path.dirname(__file__)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import gui_server  # noqa: E402
from replay import schema  # noqa: E402
from _replay_fixture import write_bag  # noqa: E402

RID = "2026-01-01_00-00-00"
DURATION_S = 25.0  # -> 3 chunks


def _req(port, path, method="GET"):
    r = urllib.request.Request("http://127.0.0.1:{}{}".format(port, path), method=method,
                               data=b"" if method == "POST" else None)
    try:
        with urllib.request.urlopen(r, timeout=30) as resp:
            return resp.status, resp.headers, resp.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers, e.read()


def _wait_status(port, want, timeout):
    end = time.time() + timeout
    st = None
    while time.time() < end:
        _c, _h, body = _req(port, "/api/replay/recordings/{}/status".format(RID))
        st = json.loads(body.decode())
        if st["status"] in want:
            return st
        assert st["status"] != "failed", st
        time.sleep(0.1)
    raise AssertionError("status never reached {}: {}".format(want, st))


def test_real_worker_through_the_api(tmp_path):
    root = tmp_path / "bags"
    bag_dir = root / RID
    bag_dir.mkdir(parents=True)
    bag = bag_dir / (RID + "_0.mcap")
    written = write_bag(str(bag), duration_s=DURATION_S, seed=3)
    old = time.time() - 3600
    os.utime(str(bag), (old, old))

    args = argparse.Namespace(
        port=0, host="127.0.0.1", rosbags_dir=str(root), cache_dir=str(tmp_path / "cache"),
        worker_python=sys.executable, cache_cap_gb=10.0, min_free_gb=0.0)
    server, backend = gui_server.make_server(args)
    assert backend.worker_available, "the venv interpreter must import mcap, rosbags, msgpack"
    port = server.server_address[1]
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    try:
        code, _h, body = _req(port, "/api/replay/recordings")
        assert code == 200
        listing = json.loads(body.decode())
        rec = [r for r in listing["recordings"] if r["id"] == RID][0]
        assert rec["closed"] and not rec["in_progress"] and rec["cache"] == "none"
        assert rec["duration_s"] is not None and abs(rec["duration_s"] - DURATION_S) < 1.0

        code, _h, body = _req(port, "/api/replay/recordings/{}/open".format(RID), "POST")
        assert code == 200, body
        assert json.loads(body.decode())["status"] in ("queued", "converting")

        st = _wait_status(port, ("complete",), timeout=120.0)
        assert st["chunks_done"] == st["chunks_total"] == 3

        code, _h, body = _req(port, "/api/replay/recordings/{}/manifest".format(RID))
        man = json.loads(body.decode())
        assert man["format"] == schema.FORMAT_VERSION
        assert man["status"] == schema.STATUS_COMPLETE
        assert man["chunks_done"] == len(man["chunks"]) == 3
        assert man["source"]["indexed"] is True
        assert "/robot_state" in man["topics"] and "/orchestrator_state" in man["topics"]
        assert man["topics"]["/robot_state"]["count"] == len(written["topics"]["/robot_state"])

        code, h, body = _req(port, "/api/replay/recordings/{}/chunks/0".format(RID))
        assert code == 200
        assert h["Content-Encoding"] == "gzip"
        assert h["Content-Type"].startswith("application/msgpack")
        rec0 = msgpack.unpackb(gzip.decompress(body), raw=False)
        assert rec0["i"] == 0 and abs(rec0["t0"] - man["t0"]) < 1e-9
        rs = rec0["topics"]["/robot_state"]
        assert rs["n"] == man["chunks"][0]["topics"]["/robot_state"] == len(rs["t"])
        assert rs["t"] == sorted(rs["t"])
        first_written = written["topics"]["/robot_state"][0][1]
        assert rs["cols"]["is_homed"][0] == first_written["is_homed"]
        assert rs["cols"]["timestamp.sec"][0] == first_written["timestamp"]["sec"]
        assert not any(k.endswith("__msgtype__") for k in rs["cols"])

        code, _h, body = _req(port, "/api/replay/recordings/{}/chunks/3".format(RID))
        assert code == 404

        code, _h, body = _req(port, "/api/replay/recordings/{}/overview".format(RID))
        assert code == 200
        ov = json.loads(body.decode())
        assert ov["bands"] and ov["bands"][0]["topic"] == schema.OVERVIEW_BAND_TOPIC
        assert {t["kind"] for t in ov["ticks"]} >= {"homed", "levelled"}

        code, _h, body = _req(port, "/api/replay/recordings/{}/open".format(RID), "POST")
        assert code == 200 and json.loads(body.decode())["status"] == "complete"

        code, _h, body = _req(port, "/api/replay/recordings")
        rec = [r for r in json.loads(body.decode())["recordings"] if r["id"] == RID][0]
        assert rec["cache"] == "complete"
    finally:
        server.shutdown()
        server.server_close()
