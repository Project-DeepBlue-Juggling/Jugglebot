"""HTTP server for the live telemetry dashboard.

Runs entirely in daemon threads so it never blocks the sim loop.
Uses Server-Sent Events (SSE) for data push — plain HTTP, no WebSocket
handshake complexity, and EventSource auto-reconnects natively.
"""

import functools
import http.server
import json
import os
import queue
import threading

from dataclasses import asdict

_STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


def _record_to_payload(record) -> dict:
    """Convert a StepRecord to a compact JSON-serialisable dict."""
    d = asdict(record)
    return {
        "t": d["time"],
        "ref": [d["ref_pose_x"], d["ref_pose_y"], d["ref_pose_z"],
                d["ref_pose_rx"], d["ref_pose_ry"], d["ref_pose_rz"]],
        "ref_tw": [d["ref_twist_vx"], d["ref_twist_vy"], d["ref_twist_vz"],
                   d["ref_twist_wx"], d["ref_twist_wy"], d["ref_twist_wz"]],
        "act": [d["actual_pose_x"], d["actual_pose_y"], d["actual_pose_z"],
                d["actual_pose_rx"], d["actual_pose_ry"], d["actual_pose_rz"]],
        "act_tw": [d["actual_twist_vx"], d["actual_twist_vy"], d["actual_twist_vz"],
                   d["actual_twist_wx"], d["actual_twist_wy"], d["actual_twist_wz"]],
        "cmd_ext": [d["cmd_ext_0"], d["cmd_ext_1"], d["cmd_ext_2"],
                    d["cmd_ext_3"], d["cmd_ext_4"], d["cmd_ext_5"]],
        "act_ext": [d["actual_ext_0"], d["actual_ext_1"], d["actual_ext_2"],
                    d["actual_ext_3"], d["actual_ext_4"], d["actual_ext_5"]],
        "leg_vel": [d["leg_vel_0"], d["leg_vel_1"], d["leg_vel_2"],
                    d["leg_vel_3"], d["leg_vel_4"], d["leg_vel_5"]],
        "mpc": {
            "solve_ms": d["solve_time_ms"],
            "status": d["solve_status"],
            "cost": d["cost"],
            "cv": d["constraint_violation"],
        },
        "err": {"mm": d["tracking_error_mm"], "deg": d["tracking_error_deg"]},
    }


# ---------------------------------------------------------------------------
# HTTP handler with SSE endpoint
# ---------------------------------------------------------------------------

class _DashboardHandler(http.server.SimpleHTTPRequestHandler):
    """Serves static files and streams telemetry via Server-Sent Events."""

    protocol_version = "HTTP/1.1"

    def do_GET(self):
        if self.path == "/events":
            self._handle_sse()
        else:
            super().do_GET()

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def guess_type(self, path):
        if path.endswith(".js"):
            return "application/javascript"
        return super().guess_type(path)

    def log_message(self, format, *args):
        pass

    def handle(self):
        try:
            super().handle()
        except (ConnectionAbortedError, ConnectionResetError,
                BrokenPipeError, OSError):
            pass

    # ---- Server-Sent Events -----------------------------------------------

    def _handle_sse(self):
        """Stream telemetry as SSE.  Blocks until the client disconnects."""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")  # disable nginx buffering
        self.end_headers()

        q: queue.Queue[str | None] = queue.Queue(maxsize=10)
        server: _TelemetryServer = self.server  # type: ignore[assignment]
        server.register_client(q)

        try:
            while True:
                try:
                    msg = q.get(timeout=1.0)
                except queue.Empty:
                    # Send SSE comment as keep-alive to detect dead connections
                    try:
                        self.wfile.write(b":\n\n")
                        self.wfile.flush()
                    except Exception:
                        break
                    continue
                if msg is None:
                    break
                try:
                    self.wfile.write(f"data: {msg}\n\n".encode())
                    self.wfile.flush()
                except Exception:
                    break
        finally:
            server.unregister_client(q)
            self.close_connection = True


class _TelemetryServer(http.server.ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._clients: set[queue.Queue] = set()
        self._lock = threading.Lock()

    def register_client(self, q: queue.Queue):
        with self._lock:
            self._clients.add(q)

    def unregister_client(self, q: queue.Queue):
        with self._lock:
            self._clients.discard(q)

    def broadcast(self, msg: str):
        with self._lock:
            for q in self._clients:
                try:
                    q.put_nowait(msg)
                except queue.Full:
                    pass


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

class DashboardServer:
    """Live telemetry dashboard server.

    Parameters
    ----------
    port : int
        HTTP + SSE port (default 8082).
    """

    def __init__(self, port: int = 8082):
        self._port = port
        handler = functools.partial(_DashboardHandler, directory=_STATIC_DIR)
        self._httpd = _TelemetryServer(("0.0.0.0", port), handler)
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """Start the server in a daemon thread."""
        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            daemon=True,
            name="dashboard-server",
        )
        self._thread.start()
        print(f"Dashboard: http://localhost:{self._port}")

    def broadcast(self, record) -> None:
        """Push a StepRecord to all connected clients (non-blocking)."""
        payload = json.dumps(_record_to_payload(record))
        self._httpd.broadcast(payload)

    def stop(self) -> None:
        """Shut down the server."""
        self._httpd.shutdown()
