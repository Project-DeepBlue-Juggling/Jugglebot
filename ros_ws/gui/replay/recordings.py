"""Recording discovery for the GUI replay backend (design 06 § 1).

STDLIB ONLY - imported by gui_server.py under the system interpreter, so the
MCAP footer is read with ``struct`` and ``metadata.yaml`` with regexes.
"""
from __future__ import annotations

import os
import re
import struct
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

try:  # package import (tests, ``-m replay.x``)
    from . import schema
except ImportError:  # pragma: no cover - flat import
    import schema  # type: ignore

ID_RE = re.compile(r"[0-9]{4}-[0-9]{2}-[0-9]{2}_[0-9]{2}-[0-9]{2}-[0-9]{2}").fullmatch
MCAP_MAGIC = b"\x89MCAP0\r\n"
IN_PROGRESS_WINDOW_S = 10.0

# Footer record: opcode(1)=0x02, body length(8)=20, summary_start(8),
# summary_offset_start(8), summary_crc(4); then the closing magic.
_FOOTER_BODY = 20
_FOOTER_TAIL = 1 + 8 + _FOOTER_BODY + len(MCAP_MAGIC)

_DURATION_RE = re.compile(r"duration:\s*\n\s*nanoseconds:\s*(\d+)")
_COUNT_RE = re.compile(r"^  message_count:\s*(\d+)", re.MULTILINE)
_START_RE = re.compile(r"starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)")
_TOPIC_NAME_RE = re.compile(r"name:\s*(\S+)")
_TOPIC_TYPE_RE = re.compile(r"type:\s*(\S+)")
_TOPIC_COUNT_RE = re.compile(r"message_count:\s*(\d+)")

_memo_lock = threading.Lock()
_footer_memo: Dict[Tuple[str, int, int], Tuple[bool, bool]] = {}
_meta_memo: Dict[Tuple[str, int, int], Dict[str, Any]] = {}
_MEMO_MAX = 4096


def valid_id(recording_id: str) -> bool:
    return bool(ID_RE(recording_id or ""))


def read_footer(mcap_path: str) -> Tuple[bool, bool]:
    """``(closed, indexed)`` from the file's last bytes.

    closed  - the trailing magic is present (the writer finished).
    indexed - closed AND the footer record's summary_start is non-zero, so a
              reader can find the summary section (chunk index, statistics)."""
    try:
        with open(mcap_path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            if size < len(MCAP_MAGIC):
                return False, False
            f.seek(-len(MCAP_MAGIC), os.SEEK_END)
            if f.read(len(MCAP_MAGIC)) != MCAP_MAGIC:
                return False, False
            if size < _FOOTER_TAIL:
                return True, False
            f.seek(-_FOOTER_TAIL, os.SEEK_END)
            tail = f.read(_FOOTER_TAIL)
    except OSError:
        return False, False
    opcode, length, summary_start = struct.unpack_from("<BQQ", tail, 0)
    if opcode != 0x02 or length != _FOOTER_BODY:
        return True, False
    return True, summary_start > 0


def _footer_cached(path: str, st: os.stat_result) -> Tuple[bool, bool]:
    key = (path, st.st_size, st.st_mtime_ns)
    with _memo_lock:
        hit = _footer_memo.get(key)
    if hit is not None:
        return hit
    val = read_footer(path)
    if val[0]:  # only a finished file is stable enough to memoise
        with _memo_lock:
            if len(_footer_memo) > _MEMO_MAX:
                _footer_memo.clear()
            _footer_memo[key] = val
    return val


def is_closed(mcap_path: str) -> bool:
    return read_footer(mcap_path)[0]


def parse_metadata_text(text: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {"duration_s": None, "message_count": None,
                           "start_ns": None, "topics": None}
    m = _DURATION_RE.search(text)
    if m:
        out["duration_s"] = int(m.group(1)) / 1e9
    m = _COUNT_RE.search(text)
    if m:
        out["message_count"] = int(m.group(1))
    m = _START_RE.search(text)
    if m:
        out["start_ns"] = int(m.group(1))
    idx = text.find("topics_with_message_count:")
    if idx >= 0:
        allow = set(schema.ALLOWLIST)
        topics: Dict[str, Dict[str, Any]] = {}
        for item in text[idx:].split("topic_metadata:")[1:]:
            n, t, c = (_TOPIC_NAME_RE.search(item), _TOPIC_TYPE_RE.search(item),
                       _TOPIC_COUNT_RE.search(item))
            if n and t and c and n.group(1) in allow and int(c.group(1)) >= 1:
                topics[n.group(1)] = {"type": t.group(1), "count": int(c.group(1))}
        out["topics"] = topics
    return out


def _metadata_cached(path: str) -> Dict[str, Any]:
    try:
        st = os.stat(path)
    except OSError:
        return parse_metadata_text("")
    key = (path, st.st_size, st.st_mtime_ns)
    with _memo_lock:
        hit = _meta_memo.get(key)
    if hit is not None:
        return dict(hit)
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            val = parse_metadata_text(f.read())
    except OSError:
        return parse_metadata_text("")
    with _memo_lock:
        if len(_meta_memo) > _MEMO_MAX:
            _meta_memo.clear()
        _meta_memo[key] = val
    return dict(val)


def find_recording(rosbags_root: str, recording_id: str,
                   now: Optional[float] = None) -> Optional[Dict[str, Any]]:
    """One recording's info dict (listing row plus ``path`` and ``mtime_ns``,
    which the HTTP layer strips/uses) or None."""
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
    closed, indexed = _footer_cached(path, st)
    info: Dict[str, Any] = {
        "id": recording_id,
        "path": path,
        "mtime_ns": st.st_mtime_ns,
        "size_bytes": st.st_size,
        "mtime": st.st_mtime,
        "closed": closed,
        "indexed": indexed,
        # Growing-file rule: no closing magic AND written within the window.
        "in_progress": (not closed) and (now - st.st_mtime < IN_PROGRESS_WINDOW_S),
    }
    info.update(_metadata_cached(os.path.join(d, "metadata.yaml")))
    return info


def list_recordings(rosbags_root: str, now: Optional[float] = None) -> List[Dict[str, Any]]:
    if now is None:
        now = time.time()
    try:
        names = os.listdir(rosbags_root)
    except OSError:
        return []
    rows = []
    for name in names:
        if valid_id(name):
            info = find_recording(rosbags_root, name, now)
            if info is not None:
                rows.append(info)
    rows.sort(key=lambda r: r["id"], reverse=True)
    return rows


def etag(info: Dict[str, Any]) -> str:
    return '"{}-{}"'.format(info["size_bytes"], info["mtime_ns"])
