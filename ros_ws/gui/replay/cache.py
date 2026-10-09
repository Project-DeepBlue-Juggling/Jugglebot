"""Recording listing and cache bookkeeping for the GUI replay backend.

STDLIB ONLY - imported by gui_server.py under the system interpreter.
The cache layout is defined by ``schema.py``.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import time
from typing import Any, Dict, List, Optional

try:  # package import (tests, ``-m replay.x``)
    from . import schema
except ImportError:  # pragma: no cover - flat import
    import schema  # type: ignore

ID_RE = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}$")
MCAP_MAGIC = b"\x89MCAP0\r\n"
IN_PROGRESS_WINDOW_S = 10.0

_DURATION_RE = re.compile(r"duration:\s*\n\s*nanoseconds:\s*(\d+)")
_COUNT_RE = re.compile(r"message_count:\s*(\d+)")
_START_RE = re.compile(r"starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)")


def valid_id(recording_id: str) -> bool:
    return bool(ID_RE.match(recording_id or ""))


def cache_dir(cache_root: str, recording_id: str) -> str:
    return os.path.join(cache_root, recording_id)


def read_manifest(cache_root: str, recording_id: str) -> Optional[Dict[str, Any]]:
    try:
        with open(os.path.join(cache_dir(cache_root, recording_id), schema.MANIFEST),
                  "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else None
    except (OSError, ValueError):
        return None


def touch_opened(cache_root: str, recording_id: str) -> None:
    d = cache_dir(cache_root, recording_id)
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, schema.OPENED_MARK)
    with open(p, "a"):
        pass
    os.utime(p, None)


def is_closed(mcap_path: str) -> bool:
    try:
        with open(mcap_path, "rb") as f:
            f.seek(0, os.SEEK_END)
            if f.tell() < len(MCAP_MAGIC):
                return False
            f.seek(-len(MCAP_MAGIC), os.SEEK_END)
            return f.read(len(MCAP_MAGIC)) == MCAP_MAGIC
    except OSError:
        return False


def _parse_metadata(path: str) -> Dict[str, Optional[float]]:
    out = {"duration_s": None, "message_count": None, "start_ns": None}
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            text = f.read()
    except OSError:
        return out
    m = _DURATION_RE.search(text)
    if m:
        out["duration_s"] = int(m.group(1)) / 1e9
    m = _COUNT_RE.search(text)
    if m:
        out["message_count"] = int(m.group(1))
    m = _START_RE.search(text)
    if m:
        out["start_ns"] = int(m.group(1))
    return out


def find_recording(rosbags_root: str, recording_id: str,
                   cache_root: str = "", now: Optional[float] = None) -> Optional[Dict[str, Any]]:
    """One recording's info dict (same shape as a listing row) or None.

    ``cache`` is the manifest status ("none", "converting", "complete",
    "failed") or "stale" for a complete cache whose format is not current
    (it is discarded and reconverted on the next open)."""
    if not valid_id(recording_id):
        return None
    d = os.path.join(rosbags_root, recording_id)
    if not os.path.isdir(d):
        return None
    try:
        mcaps = sorted(n for n in os.listdir(d) if n.endswith(".mcap"))
    except OSError:
        return None
    if len(mcaps) != 1:
        return None
    path = os.path.join(d, mcaps[0])
    try:
        st = os.stat(path)
    except OSError:
        return None
    if now is None:
        now = time.time()
    closed = is_closed(path)
    info: Dict[str, Any] = {
        "id": recording_id,
        "path": path,
        "size_bytes": st.st_size,
        "mtime": st.st_mtime,
        "closed": closed,
        "in_progress": (not closed) and (now - st.st_mtime < IN_PROGRESS_WINDOW_S),
    }
    info.update(_parse_metadata(os.path.join(d, "metadata.yaml")))
    man = read_manifest(cache_root, recording_id) if cache_root else None
    info["cache"] = man.get("status", "none") if man else "none"
    if man and info["cache"] == schema.STATUS_COMPLETE and not is_format_current(man):
        info["cache"] = "stale"
    return info


def list_recordings(rosbags_root: str, cache_root: str,
                    now: Optional[float] = None) -> List[Dict[str, Any]]:
    if now is None:
        now = time.time()
    try:
        names = os.listdir(rosbags_root)
    except OSError:
        return []
    rows = []
    for name in names:
        if not valid_id(name):
            continue
        info = find_recording(rosbags_root, name, cache_root, now)
        if info is not None:
            rows.append(info)
    rows.sort(key=lambda r: r["id"], reverse=True)
    return rows


def _dir_bytes(d: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(d):
        for n in files:
            try:
                total += os.path.getsize(os.path.join(root, n))
            except OSError:
                pass
    return total


def cache_bytes(cache_root: str) -> int:
    try:
        names = os.listdir(cache_root)
    except OSError:
        return 0
    return sum(_dir_bytes(os.path.join(cache_root, n)) for n in names
               if os.path.isdir(os.path.join(cache_root, n)))


def _lru_key(cache_root: str, recording_id: str, manifest: Dict[str, Any]) -> float:
    d = cache_dir(cache_root, recording_id)
    try:
        return os.stat(os.path.join(d, schema.OPENED_MARK)).st_mtime
    except OSError:
        pass
    done = manifest.get("completed_at")
    if done:
        try:
            import datetime
            return datetime.datetime.fromisoformat(done).timestamp()
        except (ValueError, TypeError):
            pass
    try:
        return os.stat(d).st_mtime
    except OSError:
        return 0.0


def discard_cache(cache_root: str, recording_id: str) -> None:
    if not valid_id(recording_id):
        return
    shutil.rmtree(cache_dir(cache_root, recording_id), ignore_errors=True)


def evict_lru(cache_root: str, cap_bytes: int, keep_id: Optional[str] = None) -> List[str]:
    """Delete least-recently-opened COMPLETE caches until total <= cap."""
    removed: List[str] = []
    total = cache_bytes(cache_root)
    if total <= cap_bytes:
        return removed
    try:
        names = os.listdir(cache_root)
    except OSError:
        return removed
    cands = []
    for n in names:
        if n == keep_id or not valid_id(n):
            continue
        man = read_manifest(cache_root, n)
        if not man or man.get("status") != schema.STATUS_COMPLETE:
            continue
        cands.append((_lru_key(cache_root, n, man), n))
    cands.sort()
    for _k, n in cands:
        if total <= cap_bytes:
            break
        total -= _dir_bytes(cache_dir(cache_root, n))
        discard_cache(cache_root, n)
        removed.append(n)
    return removed


def disk_free_bytes(cache_root: str) -> int:
    p = cache_root
    while p and not os.path.exists(p):
        parent = os.path.dirname(p)
        if parent == p:
            break
        p = parent
    return shutil.disk_usage(p or ".").free


def is_format_current(manifest: Optional[Dict[str, Any]]) -> bool:
    """True when the manifest was written under the current chunk schema."""
    return bool(manifest) and manifest.get("format") == schema.FORMAT_VERSION


def is_stale_partial(manifest: Optional[Dict[str, Any]], worker_alive: bool) -> bool:
    if not manifest:
        return False
    return manifest.get("status") in (schema.STATUS_CONVERTING, schema.STATUS_FAILED) \
        and not worker_alive
