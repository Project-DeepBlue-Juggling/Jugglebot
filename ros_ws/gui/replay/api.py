"""``/api/replay/`` routes and the single-slot conversion worker queue.

STDLIB ONLY - runs inside gui_server.py under the system interpreter. The
conversion itself happens in a subprocess (``replay.convert``) under the venv
interpreter, niced, one at a time.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional
from urllib.parse import urlsplit

try:
    from . import cache, schema
except ImportError:  # pragma: no cover - flat import
    import cache  # type: ignore
    import schema  # type: ignore

PREFIX = "/api/replay/"
CORS = "*"


class ReplayBackend:
    def __init__(self, rosbags_root: str, cache_root: str, worker_python: str,
                 gui_dir: str, cap_bytes: int, min_free_bytes: int,
                 worker_module: str = "replay.convert") -> None:
        self.rosbags_root = rosbags_root
        self.cache_root = cache_root
        self.worker_python = worker_python
        self.gui_dir = gui_dir
        self.cap_bytes = int(cap_bytes)
        self.min_free_bytes = int(min_free_bytes)
        self.worker_module = worker_module
        self._lock = threading.Lock()
        self._open_lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._waiting: List[str] = []
        self._running: Optional[str] = None
        self._proc: Optional[subprocess.Popen] = None
        self._exit_error: Dict[str, str] = {}
        self.worker_available = self._probe_worker()
        self._thread = threading.Thread(target=self._loop, name="replay-worker", daemon=True)
        self._thread.start()

    # ---- worker queue -------------------------------------------------
    def _probe_worker(self) -> bool:
        if not self.worker_python or not os.path.exists(self.worker_python):
            return False
        try:
            r = subprocess.run([self.worker_python, "-c", "import mcap, rosbags, msgpack"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
            return r.returncode == 0
        except (OSError, subprocess.SubprocessError):
            return False

    def _loop(self) -> None:
        while True:
            with self._cv:
                while not self._waiting:
                    self._cv.wait()
                rid = self._waiting.pop(0)
                self._running = rid
            try:
                self._run_one(rid)
            except Exception as exc:  # never let the queue thread die
                self._exit_error[rid] = "worker launch failed: {}".format(exc)
            finally:
                with self._lock:
                    self._running = None
                    self._proc = None

    def _run_one(self, rid: str) -> None:
        info = cache.find_recording(self.rosbags_root, rid, self.cache_root)
        if info is None:
            self._exit_error[rid] = "recording vanished"
            return
        out = cache.cache_dir(self.cache_root, rid)
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "worker.log"), "ab") as log:
            # No preexec_fn (unsafe in a threaded process); niceness via the
            # `nice` binary where it exists (absent on Windows).
            nice = shutil.which("nice")
            cmd = [self.worker_python, "-m", self.worker_module,
                   "--bag", info["path"], "--out", out]
            if nice:
                cmd = [nice, "-n", "10"] + cmd
            proc = subprocess.Popen(
                cmd, cwd=self.gui_dir,
                stdout=subprocess.DEVNULL, stderr=log)
            with self._lock:
                self._proc = proc
            code = proc.wait()
        if code != 0:
            self._exit_error[rid] = "worker exited (code {})".format(code)

    def enqueue(self, rid: str) -> None:
        with self._cv:
            self._exit_error.pop(rid, None)
            if rid != self._running and rid not in self._waiting:
                self._waiting.append(rid)
                self._cv.notify()

    def queue_position(self, rid: str) -> Optional[int]:
        """1-based place in the waiting line, 0 if running, None if absent."""
        with self._lock:
            if rid == self._running:
                return 0
            if rid in self._waiting:
                return self._waiting.index(rid) + 1
            return None

    def is_running(self, rid: str) -> bool:
        with self._lock:
            return rid == self._running

    def worker_alive(self, rid: str) -> bool:
        with self._lock:
            return rid == self._running and self._proc is not None and self._proc.poll() is None

    # ---- responses ----------------------------------------------------
    @staticmethod
    def _send(handler: Any, code: int, body: bytes, ctype: str,
              extra: Optional[Dict[str, str]] = None) -> None:
        handler.send_response(code)
        handler.send_header("Content-Type", ctype)
        handler.send_header("Content-Length", str(len(body)))
        handler.send_header("Cache-Control", "no-cache")
        handler.send_header("Access-Control-Allow-Origin", CORS)
        for k, v in (extra or {}).items():
            handler.send_header(k, v)
        handler._replay_headers_sent = True
        handler.end_headers()
        if getattr(handler, "command", "GET") != "HEAD":
            handler.wfile.write(body)

    def _json(self, handler: Any, code: int, obj: Any) -> None:
        self._send(handler, code, json.dumps(obj).encode("utf-8"),
                   "application/json; charset=utf-8")

    # ---- routing ------------------------------------------------------
    def handle(self, method: str, path: str, handler: Any) -> bool:
        path = urlsplit(path).path
        if not path.startswith(PREFIX):
            return False
        # Routes (plans/active/gui-rosbag-replay.md § HTTP API):
        #   GET  recordings
        #   POST recordings/<id>/open      GET recordings/<id>/status
        #   GET  recordings/<id>/manifest  GET recordings/<id>/overview
        #   GET  recordings/<id>/chunks/<i>
        parts = path[len(PREFIX):].split("/")
        try:
            if parts == ["recordings"] and method == "GET":
                self._recordings(handler)
            elif len(parts) >= 3 and parts[0] == "recordings":
                parts = parts[1:]
                rid, action = parts[0], parts[1]
                if not cache.valid_id(rid):
                    self._json(handler, 400, {"error": "bad recording id"})
                elif action == "open" and method == "POST" and len(parts) == 2:
                    self._open(handler, rid)
                elif action == "status" and method == "GET" and len(parts) == 2:
                    self._status_route(handler, rid)
                elif action in ("manifest", "overview") and method == "GET" and len(parts) == 2:
                    self._file_route(handler, rid, action)
                elif action == "chunks" and method == "GET" and len(parts) == 3:
                    self._chunk(handler, rid, parts[2])
                else:
                    self._json(handler, 404, {"error": "not found"})
            else:
                self._json(handler, 404, {"error": "not found"})
        except (BrokenPipeError, ConnectionResetError):
            pass
        return True

    def _recordings(self, handler: Any) -> None:
        rows = cache.list_recordings(self.rosbags_root, self.cache_root)
        for r in rows:  # a converting/failed manifest with no worker is not "converting"
            if r["cache"] in (schema.STATUS_CONVERTING,) and not self.worker_alive(r["id"]) \
                    and self.queue_position(r["id"]) is None:
                r["cache"] = schema.STATUS_FAILED
        self._json(handler, 200, {
            "recordings": rows,
            "cache": {"bytes": cache.cache_bytes(self.cache_root),
                      "cap_bytes": self.cap_bytes,
                      "disk_free_bytes": cache.disk_free_bytes(self.cache_root)},
            "worker_available": self.worker_available,
        })

    def status(self, rid: str) -> Dict[str, Any]:
        man = cache.read_manifest(self.cache_root, rid)
        pos = self.queue_position(rid)
        out: Dict[str, Any] = {"id": rid, "status": "none", "error": None,
                               "chunks_done": 0, "chunks_total": None,
                               "t0": None, "t1": None, "queue_position": pos}
        if man:
            for k in ("chunks_done", "chunks_total", "t0", "t1", "error"):
                out[k] = man.get(k, out[k])
        mstat = man.get("status") if man else None
        if mstat == schema.STATUS_COMPLETE:
            out["status"] = "complete"
        elif pos is not None and pos > 0:
            out["status"] = "queued"
        elif mstat == schema.STATUS_FAILED:
            out["status"] = "failed"
        elif mstat == schema.STATUS_CONVERTING:
            if self.worker_alive(rid):
                out["status"] = "converting"
            else:
                out["status"] = "failed"
                out["error"] = self._exit_error.get(rid, "worker exited")
        elif pos == 0:
            out["status"] = "converting"  # worker started, no manifest yet
        elif rid in self._exit_error:
            out["status"] = "failed"
            out["error"] = self._exit_error[rid]
        return out

    def _status_route(self, handler: Any, rid: str) -> None:
        self._json(handler, 200, self.status(rid))

    def _file_route(self, handler: Any, rid: str, name: str) -> None:
        fn = schema.MANIFEST if name == "manifest" else schema.OVERVIEW
        try:
            with open(os.path.join(cache.cache_dir(self.cache_root, rid), fn), "rb") as f:
                body = f.read()
        except OSError:
            self._json(handler, 404, {"error": name + " not available"})
            return
        self._send(handler, 200, body, "application/json; charset=utf-8")

    def _chunk(self, handler: Any, rid: str, idx: str) -> None:
        if not re.fullmatch(r"[0-9]+", idx):
            self._json(handler, 400, {"error": "bad chunk index"})
            return
        i = int(idx)
        man = cache.read_manifest(self.cache_root, rid)
        if not man or i >= int(man.get("chunks_done", 0)):
            self._json(handler, 404, {"error": "chunk not ready"})
            return
        try:
            with open(os.path.join(cache.cache_dir(self.cache_root, rid),
                                   schema.chunk_name(i)), "rb") as f:
                body = f.read()
        except OSError:
            self._json(handler, 404, {"error": "chunk missing"})
            return
        self._send(handler, 200, body, "application/msgpack", {"Content-Encoding": "gzip"})

    def _refuse(self, handler: Any, reason: str) -> None:
        self._json(handler, 409, {"status": "refused", "reason": reason})

    def _open(self, handler: Any, rid: str) -> None:
        with self._open_lock:
            self._open_locked(handler, rid)

    def _open_locked(self, handler: Any, rid: str) -> None:
        info = cache.find_recording(self.rosbags_root, rid, self.cache_root)
        if info is None:
            self._json(handler, 404, {"error": "unknown recording"})
            return
        man = cache.read_manifest(self.cache_root, rid)
        if man and man.get("status") == schema.STATUS_COMPLETE:
            src = man.get("source", {})
            if src.get("size_bytes") == info["size_bytes"] and \
                    abs(float(src.get("mtime", -1)) - info["mtime"]) < 1e-3:
                cache.touch_opened(self.cache_root, rid)
                self._json(handler, 200, self.status(rid))
                return
        if info["in_progress"]:
            self._refuse(handler, "recording_in_progress")
            return
        if self.queue_position(rid) is not None:
            self._json(handler, 200, self.status(rid))
            return
        # Anything left on disk now is a stale partial or a changed source.
        if os.path.isdir(cache.cache_dir(self.cache_root, rid)):
            cache.discard_cache(self.cache_root, rid)
        if not self.worker_available:
            self.worker_available = self._probe_worker()  # lazy re-probe
        if not self.worker_available:
            self._refuse(handler, "worker_unavailable")
            return
        os.makedirs(self.cache_root, exist_ok=True)
        cache.evict_lru(self.cache_root, self.cap_bytes, keep_id=rid)
        if cache.disk_free_bytes(self.cache_root) < self.min_free_bytes:
            self._refuse(handler, "disk_low")
            return
        cache.touch_opened(self.cache_root, rid)
        self.enqueue(rid)
        st = self.status(rid)
        if st["status"] == "none":
            st["status"] = "queued"
        self._json(handler, 200, st)
