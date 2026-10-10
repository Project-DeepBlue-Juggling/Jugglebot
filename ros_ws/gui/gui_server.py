#!/usr/bin/env python3
"""Standalone HTTP server for the Jugglebot GUI.

Serves static files from this directory on all interfaces and the
``/api/replay/`` rosbag-replay routes (replay/api.py). Pure stdlib, no ROS2 -
start once and leave running. The browser reads recordings over HTTP Range; the
overview pass runs in a niced subprocess under the venv interpreter
(``--worker-python``); the server itself needs nothing.

Usage:
    python3 gui_server.py [--port 8081] [--rosbags-dir DIR] [--overview-dir DIR]
"""
from __future__ import annotations

import argparse
import http.server
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from replay.api import ReplayBackend  # noqa: E402


class CORSHandler(http.server.SimpleHTTPRequestHandler):
    """SimpleHTTPRequestHandler with CORS, correct MIME types and the replay API."""

    backend = None  # set per server in make_server
    _replay_headers_sent = False

    def end_headers(self):
        if not self._replay_headers_sent:  # the API adds these itself
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def do_GET(self):
        if self.backend is not None and self.backend.handle("GET", self.path, self):
            return
        super().do_GET()

    def do_HEAD(self):
        if self.backend is not None and self.backend.handle("HEAD", self.path, self):
            return
        super().do_HEAD()

    def do_POST(self):
        if self.backend is not None and self.backend.handle("POST", self.path, self):
            return
        self.send_error(404)

    def guess_type(self, path):
        # Ensure .js files are served as ES modules
        if path.endswith(".js"):
            return "application/javascript"
        if path.endswith(".mjs"):
            return "application/javascript"
        return super().guess_type(path)


class ReusableHTTPServer(http.server.ThreadingHTTPServer):
    """Threaded server with SO_REUSEADDR (no 'Address already in use')."""
    allow_reuse_address = True
    daemon_threads = True


def build_parser():
    parser = argparse.ArgumentParser(description="Jugglebot GUI server")
    parser.add_argument("--port", type=int, default=8081, help="Port to serve on")
    parser.add_argument("--rosbags-dir", default="~/Desktop/rosbags")
    parser.add_argument("--overview-dir",
                        default=os.path.join(_HERE, "..", "..", "temp", "replay_overview"))
    parser.add_argument("--worker-python", default="~/Desktop/PDJ_venv/venv/bin/python")
    return parser


def make_server(args):
    """Build (server, backend). ``args`` may carry optional ``host`` and
    ``worker_module`` attributes (tests)."""
    backend = ReplayBackend(
        rosbags_root=os.path.abspath(os.path.expanduser(args.rosbags_dir)),
        overview_dir=os.path.abspath(os.path.expanduser(args.overview_dir)),
        worker_python=os.path.expanduser(args.worker_python),
        gui_dir=getattr(args, "gui_dir", None) or _HERE,
        worker_module=getattr(args, "worker_module", "replay.overview"))

    class Handler(CORSHandler):
        pass

    Handler.backend = backend
    handler = lambda *a, **kw: Handler(*a, directory=_HERE, **kw)  # noqa: E731
    server = ReusableHTTPServer((getattr(args, "host", "0.0.0.0"), args.port), handler)
    return server, backend


def main():
    args = build_parser().parse_args()
    server, backend = make_server(args)
    print("Serving Jugglebot GUI on http://0.0.0.0:{}".format(args.port))
    print("  Directory: {}".format(_HERE))
    print("  Replay: rosbags={} overview={} worker_available={}".format(
        backend.rosbags_root, backend.overview_dir, backend.worker_available))
    print("  Press Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        server.server_close()


if __name__ == "__main__":
    main()
