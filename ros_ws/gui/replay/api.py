"""``/api/replay/`` routes (design 06 § 1): listing, Range file serving, overview.

STDLIB ONLY - runs inside gui_server.py under the system interpreter. The
browser reads the MCAP bytes itself over HTTP Range (``McapSource``); the only
venv work is the overview pass (``replay.overview``, subprocess).
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import threading
from collections import deque
from typing import Any, Dict, Optional, Tuple
from urllib.parse import urlsplit

try:
    from . import recordings, schema
except ImportError:  # pragma: no cover - flat import
    import recordings  # type: ignore
    import schema  # type: ignore

PREFIX = "/api/replay/"
CORS = "*"
BLOCK = 1 << 20  # file bodies stream in 1 MiB blocks
EXPOSE = "Content-Range, ETag, Accept-Ranges, X-Replay-Reason"
_RANGE_RE = re.compile(r"^bytes=(\d*)-(\d*)$")


def parse_range(header: str, size: int) -> Optional[Tuple[int, int]]:
    """Inclusive ``(first, last)`` for a single satisfiable byte range, else None
    (multi-range, malformed, or unsatisfiable - all answered 416)."""
    m = _RANGE_RE.match(header.strip())
    if not m:
        return None
    a, b = m.group(1), m.group(2)
    if a == "" and b == "":
        return None
    if a == "":  # suffix: the last n bytes
        n = int(b)
        if n <= 0 or size == 0:
            return None
        return max(0, size - n), size - 1
    first = int(a)
    last = size - 1 if b == "" else min(int(b), size - 1)
    if first >= size or first > last:
        return None
    return first, last


class ReplayBackend:
    def __init__(self, rosbags_root: str, overview_dir: str, worker_python: str,
                 gui_dir: str, worker_module: str = "replay.overview") -> None:
        self.rosbags_root = rosbags_root
        self.overview_dir = overview_dir
        self.worker_python = worker_python
        self.gui_dir = gui_dir
        self.worker_module = worker_module
        self.worker_available = self._probe_worker()
        # Overview queue (design 06 § 1): one pass at a time, FIFO, one daemon thread.
        self._lock = threading.Lock()
        self._queue = deque()          # recording ids waiting
        self._running = None           # id being computed
        self._failed = {}              # id -> (size_bytes, mtime_ns, error)
        self._thread = None
        self._valid_memo = {}          # overview path -> ((st_size, st_mtime_ns), doc head)

    def _probe_worker(self) -> bool:
        if not self.worker_python or not os.path.exists(self.worker_python):
            return False
        try:
            r = subprocess.run([self.worker_python, "-c", "import mcap, rosbags"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
            return r.returncode == 0
        except (OSError, subprocess.SubprocessError):
            return False

    # ---- responses ----------------------------------------------------
    @staticmethod
    def _send(handler: Any, code: int, body: bytes, ctype: str,
              extra: Optional[Dict[str, str]] = None, cache: str = "no-cache") -> None:
        handler.send_response(code)
        handler.send_header("Content-Type", ctype)
        handler.send_header("Content-Length", str(len(body)))
        handler.send_header("Cache-Control", cache)
        handler.send_header("Access-Control-Allow-Origin", CORS)
        for k, v in (extra or {}).items():
            handler.send_header(k, v)
        handler._replay_headers_sent = True
        handler.end_headers()
        if getattr(handler, "command", "GET") != "HEAD":
            handler.wfile.write(body)

    def _json(self, handler: Any, code: int, obj: Any,
              extra: Optional[Dict[str, str]] = None, cache: str = "no-cache") -> None:
        self._send(handler, code, json.dumps(obj).encode("utf-8"),
                   "application/json; charset=utf-8", extra, cache)

    # ---- routing ------------------------------------------------------
    def handle(self, method: str, path: str, handler: Any) -> bool:
        path = urlsplit(path).path
        if not path.startswith(PREFIX):
            return False
        # Routes (design 06 § 1):
        #   GET       recordings
        #   GET|HEAD  recordings/<id>/file       (Range)
        #   GET       recordings/<id>/overview
        parts = path[len(PREFIX):].split("/")
        try:
            if parts == ["recordings"] and method == "GET":
                self._recordings(handler)
            elif len(parts) == 3 and parts[0] == "recordings":
                rid, action = parts[1], parts[2]
                if not recordings.valid_id(rid):
                    self._json(handler, 400, {"error": "bad recording id"})
                elif action == "file" and method in ("GET", "HEAD"):
                    self._file(handler, rid)
                elif action == "overview" and method == "GET":
                    self._overview(handler, rid)
                else:
                    self._json(handler, 404, {"error": "not found"})
            else:
                self._json(handler, 404, {"error": "not found"})
        except (BrokenPipeError, ConnectionResetError):
            pass
        return True

    # ---- overview cache + queue --------------------------------------
    def _ov_path(self, rid: str) -> str:
        return os.path.join(self.overview_dir, rid + ".json")

    def _cached_overview(self, info: Dict[str, Any]) -> Optional[bytes]:
        """The cached overview bytes when format and source match the bag, else None."""
        path = self._ov_path(info["id"])
        try:
            st = os.stat(path)
            key = (st.st_size, st.st_mtime_ns)
            memo = self._valid_memo.get(path)
            if memo is not None and memo[0] == key:
                head, raw = memo[1], None
            else:
                with open(path, "rb") as fh:
                    raw = fh.read()
                doc = json.loads(raw.decode("utf-8"))
                head = (doc.get("format"), doc.get("source"))
                self._valid_memo[path] = (key, head)
        except (OSError, ValueError):
            return None
        src = head[1] or {}
        if head[0] != schema.FORMAT_VERSION or src.get("size_bytes") != info["size_bytes"] \
                or src.get("mtime") != info["mtime"]:
            return None
        if raw is None:
            try:
                with open(path, "rb") as fh:
                    raw = fh.read()
            except OSError:
                return None
        return raw

    def overview_state(self, rid: str, info: Optional[Dict[str, Any]] = None) -> str:
        """ready|none|queued|computing|failed (listing row field). ``info`` skips a second discovery."""
        if info is None:
            info = recordings.find_recording(self.rosbags_root, rid)
        if info is None:
            return "none"
        with self._lock:
            if self._running == rid:
                return "computing"
            if rid in self._queue:
                return "queued"
            f = self._failed.get(rid)
        if f is not None and f[:2] == (info["size_bytes"], info["mtime_ns"]):
            return "failed"
        return "ready" if self._cached_overview(info) is not None else "none"

    def _enqueue(self, info: Dict[str, Any]) -> str:
        """Queue ``info`` unless already queued/running; returns the 202 status word."""
        rid = info["id"]
        with self._lock:
            if self._running == rid:
                return "computing"
            if rid not in self._queue:
                self._queue.append(rid)
            if self._thread is None:
                self._thread = threading.Thread(target=self._drain, daemon=True,
                                                name="replay-overview")
                self._thread.start()
            return "queued"

    def _drain(self) -> None:
        while True:
            with self._lock:
                if not self._queue:
                    self._running = None
                    self._thread = None  # under the lock, so _enqueue never trusts a dying thread
                    return
                rid = self._queue.popleft()
                self._running = rid
            err = None
            info = None
            try:
                info = recordings.find_recording(self.rosbags_root, rid)
                err = "recording vanished" if info is None else self._run_pass(info)
            except Exception as exc:  # noqa: BLE001 - the queue must keep draining
                err = "%s: %s" % (type(exc).__name__, exc)
            with self._lock:
                if err is None:
                    self._failed.pop(rid, None)
                elif info is not None:
                    self._failed[rid] = (info["size_bytes"], info["mtime_ns"], err)
                self._running = None

    def _run_pass(self, info: Dict[str, Any]) -> Optional[str]:
        """Run the niced venv pass; None on success, else an error string."""
        out = self._ov_path(info["id"])
        try:
            os.makedirs(self.overview_dir, exist_ok=True)
            r = subprocess.run(
                ["nice", "-n", "10", self.worker_python, "-m", self.worker_module,
                 "--bag", info["path"], "--out", out],
                cwd=self.gui_dir, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE, timeout=6 * 3600)
        except (OSError, subprocess.SubprocessError) as exc:
            return "%s: %s" % (type(exc).__name__, exc)
        if r.returncode != 0:
            tail = r.stderr.decode("utf-8", "replace").strip().splitlines()[-3:]
            return "overview pass exited %d: %s" % (r.returncode, " | ".join(tail))
        return None

    def _recordings(self, handler: Any) -> None:
        rows = []
        for info in recordings.list_recordings(self.rosbags_root):
            row = {k: v for k, v in info.items() if k not in ("path", "mtime_ns")}
            row["overview"] = self.overview_state(info["id"], info)
            rows.append(row)
        self._json(handler, 200, {"recordings": rows,
                                  "overview_available": self.worker_available})

    def _overview(self, handler: Any, rid: str) -> None:
        info = recordings.find_recording(self.rosbags_root, rid)
        if info is None:
            self._json(handler, 404, {"error": "unknown recording"})
            return
        if info["in_progress"]:
            self._refuse(handler, "recording_in_progress")
            return
        if not info["indexed"]:
            self._refuse(handler, "no_index")
            return
        cached = self._cached_overview(info)
        if cached is not None:
            self._send(handler, 200, cached, "application/json; charset=utf-8")
            return
        if not self.worker_available:
            self._json(handler, 503, {"status": "unavailable", "reason": "worker_unavailable"})
            return
        with self._lock:
            f = self._failed.get(rid)
        if f is not None and f[:2] == (info["size_bytes"], info["mtime_ns"]):
            self._json(handler, 500, {"status": "failed", "error": f[2]})
            return
        self._json(handler, 202, {"status": self._enqueue(info)})

    def _refuse(self, handler: Any, reason: str) -> None:
        # The reason also rides a header: a HEAD response has no body, and the browser's first request
        # (the size probe) is a HEAD, so a body-only reason would be lost and a killed recording would
        # read as "in progress".
        self._json(handler, 409, {"status": "refused", "reason": reason}, cache="no-store",
                   extra={"X-Replay-Reason": reason, "Access-Control-Expose-Headers": EXPOSE})

    def _file(self, handler: Any, rid: str) -> None:
        info = recordings.find_recording(self.rosbags_root, rid)
        if info is None:
            self._json(handler, 404, {"error": "unknown recording"})
            return
        if info["in_progress"]:
            self._refuse(handler, "recording_in_progress")
            return
        if not info["indexed"]:
            self._refuse(handler, "no_index")
            return
        size = info["size_bytes"]
        tag = recordings.etag(info)
        base = {"Accept-Ranges": "bytes", "ETag": tag,
                "Access-Control-Expose-Headers": EXPOSE}
        im = handler.headers.get("If-Match")
        if im is not None and im.strip() != "*" \
                and tag not in [t.strip() for t in im.split(",")]:
            self._json(handler, 412, {"error": "recording changed"}, base, "no-store")
            return
        rng = handler.headers.get("Range")
        first, last, code = 0, size - 1, 200
        extra = dict(base)
        if rng is not None:
            parsed = parse_range(rng, size)
            if parsed is None:
                extra["Content-Range"] = "bytes */{}".format(size)
                self._json(handler, 416, {"error": "range not satisfiable"}, extra, "no-store")
                return
            first, last = parsed
            code = 206
            extra["Content-Range"] = "bytes {}-{}/{}".format(first, last, size)
        length = last - first + 1 if size else 0
        body = None
        if getattr(handler, "command", "GET") != "HEAD" and length:
            # Open BEFORE the headers and compare the open file with the listing the ETag came from:
            # a rotate/rewrite between find_recording and open() must not stream a mixed file under 200/206.
            body = open(info["path"], "rb")
            st = os.fstat(body.fileno())
            if st.st_size != info["size_bytes"] or st.st_mtime_ns != info["mtime_ns"]:
                body.close()
                self._json(handler, 412, {"error": "recording changed"}, base, "no-store")
                return
        handler.send_response(code)
        handler.send_header("Content-Type", "application/octet-stream")
        handler.send_header("Content-Length", str(length))
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Access-Control-Allow-Origin", CORS)
        for k, v in extra.items():
            handler.send_header(k, v)
        handler._replay_headers_sent = True
        handler.end_headers()
        if body is None:
            return
        with body as f:
            f.seek(first)
            left = length
            while left > 0:
                block = f.read(min(BLOCK, left))
                if not block:
                    break
                handler.wfile.write(block)
                left -= len(block)
