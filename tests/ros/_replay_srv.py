"""Shared helpers for the replay route tests: ephemeral-port server + fake MCAPs."""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import threading
import time
import urllib.error
import urllib.request

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
GUI = os.path.join(REPO, "ros_ws", "gui")
if GUI not in sys.path:
    sys.path.insert(0, GUI)

import gui_server  # noqa: E402

MAGIC = b"\x89MCAP0\r\n"


def footer(summary_start=1000):
    """A syntactically valid MCAP footer record + closing magic."""
    return struct.pack("<BQQQI", 0x02, 20, summary_start, 0, 0) + MAGIC


def make_bag(root, rid, kind="indexed", age_s=3600.0, body=None, metadata=None):
    """kind: indexed | nosummary (closed, summary_start 0) | killed (no magic)."""
    d = os.path.join(root, rid)
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, rid + "_0.mcap")
    body = body if body is not None else bytes(range(256)) * 8
    tail = {"indexed": footer(1000), "nosummary": footer(0), "killed": b"zz"}[kind]
    with open(p, "wb") as f:
        f.write(MAGIC + body + tail)
    t = time.time() - age_s
    os.utime(p, (t, t))
    if metadata is not None:
        with open(os.path.join(d, "metadata.yaml"), "w") as f:
            f.write(metadata)
    return p


class Srv:
    def __init__(self, tmp_path, worker_python="/nonexistent/python"):
        self.root = str(tmp_path / "bags")
        os.makedirs(self.root)
        args = argparse.Namespace(
            port=0, host="127.0.0.1", rosbags_dir=self.root,
            overview_dir=str(tmp_path / "overview"), worker_python=worker_python)
        self.server, self.backend = gui_server.make_server(args)
        self.port = self.server.server_address[1]
        self.t = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.t.start()

    def req(self, path, method="GET", headers=None):
        r = urllib.request.Request("http://127.0.0.1:{}{}".format(self.port, path),
                                   method=method, headers=headers or {})
        try:
            with urllib.request.urlopen(r, timeout=10) as resp:
                return resp.status, resp.headers, resp.read()
        except urllib.error.HTTPError as e:
            return e.code, e.headers, e.read()

    def jreq(self, path, method="GET", headers=None):
        code, _h, body = self.req(path, method, headers)
        return code, json.loads(body.decode())

    def close(self):
        self.server.shutdown()
        self.server.server_close()
