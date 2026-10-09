"""Convert one MCAP recording into the per-recording replay cache.

The cache layout, chunk record, manifest and overview shapes are defined in
``schema.py`` (the contract); this module only produces them. Runs under the
project VENV (needs ``mcap``, ``rosbags``, ``msgpack``) — the GUI server spawns
it as a subprocess and only ever reads what it writes.

Usage (``cwd=ros_ws/gui`` so ``-m`` resolves the package)::

    python -m replay.convert --bag <file.mcap> --out <cache_dir> [--chunk-s 10]
    python -m replay.convert --bulk <recordings_root> --cache-root <dir> [--newest N]

Exit 0 when every conversion completed, 1 otherwise (the manifest then says
``status: failed`` with ``error`` set). Progress goes to stderr, never stdout.

One pass over the bag, undecoded iteration; only allow-listed channels are
deserialised (rosbags typestore built from the schemas embedded in the MCAP).
Messages are binned into ``CHUNK_S`` chunks keyed off the first message's log
time, with two chunks open at once as a reorder buffer: chunk ``i`` is sealed
when a message for chunk ``i + 2`` arrives (or at the end), and a message for
an already-sealed chunk is counted in ``dropped_late`` and not written. The
overview is built incrementally from sealed (time-sorted) rows, so the bag is
never re-read.
"""
from __future__ import annotations

import argparse
import dataclasses
import datetime as _dt
import gzip
import json
import math
import os
import re
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import msgpack
import numpy as np
from mcap.reader import NonSeekingReader, SeekingReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from . import schema

MCAP_MAGIC = b"\x89MCAP0\r\n"
SCHEMA_SEPARATOR = "=" * 80
RECORDING_DIR_RE = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}$")
MANIFEST_MIN_INTERVAL_S = 1.0

_FAULT_FLAGS = ("has_fatal_odrive_error", "has_fatal_can_error", "has_undervoltage")


def _now_iso() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat()


def _converter_version() -> str:
    try:
        from importlib.metadata import version
        return "rosbags " + version("rosbags")
    except Exception:  # pragma: no cover - metadata missing
        return "rosbags unknown"


def _log(msg: str) -> None:
    sys.stderr.write("[replay.convert] " + msg + "\n")
    sys.stderr.flush()


def _atomic_write_bytes(path: str, data: bytes) -> None:
    tmp = path + ".tmp"
    with open(tmp, "wb") as fh:
        fh.write(data)
    os.replace(tmp, path)


def _write_json(path: str, obj: Dict) -> None:
    _atomic_write_bytes(path, json.dumps(obj, indent=1).encode("utf-8"))


def is_indexed(path: str) -> bool:
    """True when the file ends with the MCAP footer magic (a finished recording)."""
    with open(path, "rb") as fh:
        fh.seek(0, os.SEEK_END)
        if fh.tell() < len(MCAP_MAGIC):
            return False
        fh.seek(-len(MCAP_MAGIC), os.SEEK_END)
        return fh.read(len(MCAP_MAGIC)) == MCAP_MAGIC


# --------------------------------------------------------------------------
# Typestore + flattening
# --------------------------------------------------------------------------

def parse_embedded_schema(name: str, text: str) -> Dict:
    """Split rosbag2's concatenated ``ros2msg`` text into rosbags type defs.

    The first (un-separated) block is ``name`` itself; every later block starts
    with ``MSG: pkg/msg/Dep`` after an 80-``=`` separator line.
    """
    blocks = text.split(SCHEMA_SEPARATOR)
    types = dict(get_types_from_msg(blocks[0], name))
    for block in blocks[1:]:
        block = block.strip()
        if not block:
            continue
        first, _, body = block.partition("\n")
        if not first.startswith("MSG: "):
            raise ValueError("schema %s: malformed dependency block %r" % (name, first))
        types.update(get_types_from_msg(body, first[5:].strip()))
    return types


class _Decoder:
    """Lazily registers MCAP schemas into one typestore and flattens messages."""

    def __init__(self) -> None:
        self.ts = get_typestore(Stores.ROS2_FOXY)
        self._registered = set()  # schema ids
        self._flatteners = {}  # typename -> list of (path, attr chain)

    def ensure_schema(self, sch) -> None:
        if sch.id in self._registered:
            return
        if sch.encoding != "ros2msg":
            raise ValueError("schema %s has unsupported encoding %r" % (sch.name, sch.encoding))
        types = parse_embedded_schema(sch.name, sch.data.decode("utf-8"))
        new = {k: v for k, v in types.items() if k not in self.ts.types}
        if new:
            self.ts.register(new)
        self._registered.add(sch.id)

    def decode(self, data: bytes, typename: str):
        return self.ts.deserialize_cdr(data, typename)

    def flatten(self, msg, typename: str) -> Tuple[List[str], List[Any]]:
        spec = self._flatteners.get(typename)
        if spec is None:
            spec = []
            _build_spec(msg, (), spec)
            self._flatteners[typename] = spec
        values = []
        for _path, chain in spec:
            v = msg
            for attr in chain:
                v = getattr(v, attr)
            values.append(_to_plain(v))
        return [p for p, _ in spec], values


def _is_message(v) -> bool:
    return hasattr(v, "__dataclass_fields__")


def _field_names(msg) -> List[str]:
    # rosbags message classes carry a ``__msgtype__`` dataclass field (the
    # type name) after the real ones; it is not part of the definition. IDL
    # CONSTANTS (DiagnosticStatus OK/WARN/ERROR/STALE) are also dataclass fields,
    # distinguished only by carrying a default (real fields have none).
    return [f.name for f in dataclasses.fields(msg)
            if not f.name.startswith("__")
            and f.default is dataclasses.MISSING
            and f.default_factory is dataclasses.MISSING]


def _build_spec(msg, chain: Tuple[str, ...], out: List) -> None:
    for name in _field_names(msg):
        v = getattr(msg, name)
        sub = chain + (name,)
        if _is_message(v):
            _build_spec(v, sub, out)
        else:
            out.append((".".join(sub), sub))


def _to_plain(v):
    """Leaf -> msgpack-able value; message (inside arrays) -> plain nested dict."""
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, np.generic):
        return v.item()
    if _is_message(v):
        return {k: _to_plain(getattr(v, k)) for k in _field_names(v)}
    if isinstance(v, (list, tuple)):
        return [_to_plain(x) for x in v]
    return v


# --------------------------------------------------------------------------
# Overview (built from sealed, time-sorted rows)
# --------------------------------------------------------------------------

class _Overview:
    def __init__(self) -> None:
        self.segments = []  # [t_start, t_end, value]
        self.ticks = []
        self.presence = {}  # topic -> list of chunk indices with n >= 1
        self._rs_prev = None  # (fault, homed, levelled)

    def feed(self, topic: str, ts: List[float], cols: Dict[str, List]) -> None:
        if topic == schema.OVERVIEW_BAND_TOPIC and "data" in cols:
            for t, val in zip(ts, cols["data"]):
                val = str(val)
                if self.segments and self.segments[-1][2] == val:
                    continue
                if self.segments:
                    self.segments[-1][1] = t
                self.segments.append([t, None, val])
        elif topic == "/robot_state":
            flags = [cols.get(f) for f in _FAULT_FLAGS]
            homed = cols.get("is_homed")
            lev = cols.get("levelling_complete")
            for k, t in enumerate(ts):
                fault = any(bool(f[k]) for f in flags if f is not None)
                h = bool(homed[k]) if homed is not None else False
                lv = bool(lev[k]) if lev is not None else False
                prev = self._rs_prev
                if prev is not None:
                    if fault and not prev[0]:
                        names = [n for n, f in zip(_FAULT_FLAGS, flags) if f is not None and f[k]]
                        self.ticks.append({"t": t, "kind": "fault", "label": ",".join(names)})
                    elif prev[0] and not fault:
                        self.ticks.append({"t": t, "kind": "fault_cleared", "label": ""})
                    if h and not prev[1]:
                        self.ticks.append({"t": t, "kind": "homed", "label": ""})
                    if lv and not prev[2]:
                        self.ticks.append({"t": t, "kind": "levelled", "label": ""})
                self._rs_prev = (fault, h, lv)
        elif topic == "/cone/catch_event":
            seq = cols.get("sequence")
            for k, t in enumerate(ts):
                self.ticks.append({"t": t, "kind": "catch_event",
                                   "label": "" if seq is None else "seq %s" % seq[k]})
        elif topic == "/skills/attempt":
            msgs = cols.get("message") or [""] * len(ts)
            names = cols.get("name") or [""] * len(ts)
            for t, m, n in zip(ts, msgs, names):
                self.ticks.append({"t": t, "kind": "skill_attempt", "label": m or n or ""})
        elif topic == "/bb/calibration_result":
            for t in ts:
                self.ticks.append({"t": t, "kind": "bb_calibration", "label": ""})

    def note_chunk(self, i: int, topics: Dict[str, int]) -> None:
        for topic, n in topics.items():
            if n >= 1:
                self.presence.setdefault(topic, []).append(i)

    def build(self, t0: float, t1: float, chunk_s: float) -> Dict:
        segs = [list(s) for s in self.segments]
        if segs:
            segs[-1][1] = t1
        presence = {}
        for topic, idx in self.presence.items():
            runs = []
            for i in idx:
                start, end = t0 + i * chunk_s, min(t0 + (i + 1) * chunk_s, t1)
                if runs and runs[-1][2] == i - 1:
                    runs[-1][1] = end
                    runs[-1][2] = i
                else:
                    runs.append([start, end, i])
            presence[topic] = [[a, b] for a, b, _ in runs]
        return {
            "format": schema.FORMAT_VERSION,
            "t0": t0,
            "t1": t1,
            "bands": [{"topic": schema.OVERVIEW_BAND_TOPIC, "segments": segs}],
            "ticks": sorted(self.ticks, key=lambda d: d["t"]),
            "presence": presence,
        }


# --------------------------------------------------------------------------
# The conversion pass
# --------------------------------------------------------------------------

def _default_recording_id(bag_path: str) -> str:
    parent = os.path.basename(os.path.dirname(os.path.abspath(bag_path)))
    return parent or os.path.splitext(os.path.basename(bag_path))[0]


def convert(bag_path: str, out_dir: str, chunk_s: float = schema.CHUNK_S,
            recording_id: Optional[str] = None,
            on_manifest: Optional[Callable[[Dict], None]] = None) -> Dict:
    """Convert ``bag_path`` into ``out_dir``; return the final manifest.

    Never raises for a conversion failure: the returned (and written) manifest
    carries ``status: failed`` and ``error``. ``on_manifest`` (tests) is called
    with a copy of every manifest written.
    """
    os.makedirs(out_dir, exist_ok=True)
    manifest_path = os.path.join(out_dir, schema.MANIFEST)
    rec_id = recording_id or _default_recording_id(bag_path)

    def write_manifest(m: Dict) -> None:
        _write_json(manifest_path, m)
        if on_manifest is not None:
            on_manifest(json.loads(json.dumps(m)))

    try:
        st = os.stat(bag_path)
        indexed = is_indexed(bag_path)
    except OSError as exc:
        m = schema.new_manifest(rec_id, os.path.abspath(bag_path), 0, 0.0, False,
                                _now_iso(), _converter_version(), None)
        m["status"] = schema.STATUS_FAILED
        m["chunks"] = []  # "[] until complete" (schema.py); chunks_done stays
        m["error"] = "%s: %s" % (type(exc).__name__, exc)
        m["completed_at"] = _now_iso()
        write_manifest(m)
        _log("FAILED %s: %s" % (bag_path, m["error"]))
        return m

    m = schema.new_manifest(rec_id, os.path.abspath(bag_path), st.st_size, st.st_mtime,
                            indexed, _now_iso(), _converter_version(), None)
    m["chunk_s"] = float(chunk_s)
    try:
        with open(bag_path, "rb") as fh:
            _run(fh, st.st_size, indexed, out_dir, float(chunk_s), m, write_manifest)
    except Exception as exc:  # noqa: BLE001 - any failure is reported via the manifest
        m["status"] = schema.STATUS_FAILED
        m["chunks"] = []  # "[] until complete" (schema.py); chunks_done stays
        m["error"] = "%s: %s" % (type(exc).__name__, exc)
        m["completed_at"] = _now_iso()
        write_manifest(m)
        _log("FAILED %s: %s" % (bag_path, m["error"]))
    return m


def _run(fh, size: int, indexed: bool, out_dir: str, chunk_s: float,
         m: Dict, write_manifest: Callable[[Dict], None]) -> None:
    allow = set(schema.ALLOWLIST)
    dec = _Decoder()
    ov = _Overview()

    if indexed:
        reader = SeekingReader(fh)
        summary = reader.get_summary()
        stats = summary.statistics if summary is not None else None
        if stats is not None and stats.message_count:
            dur_s = (stats.message_end_time - stats.message_start_time) / 1e9
            m["chunks_total"] = int(dur_s // chunk_s) + 1
    else:
        reader = NonSeekingReader(fh)
    write_manifest(m)

    open_chunks = {}  # i -> {topic: {"type": str, "rows": [(t, values)], "paths": [...]}}
    state = {"next_seal": 0, "last_write": time.monotonic(), "progress": 0}
    t0_holder = [None]
    t_last = [None]

    def seal(i: int) -> None:
        t0 = t0_holder[0]
        chunk = open_chunks.pop(i, {})
        topics_out = {}
        counts = {}
        for topic in sorted(chunk):
            entry = chunk[topic]
            rows = sorted(entry["rows"], key=lambda r: r[0])
            ts = [r[0] for r in rows]
            cols = {}
            for k, path in enumerate(entry["paths"]):
                cols[path] = [r[1][k] for r in rows]
            topics_out[topic] = {"type": entry["type"], "n": len(rows), "t": ts, "cols": cols}
            counts[topic] = len(rows)
            ov.feed(topic, ts, cols)
        lo, hi = t0 + i * chunk_s, t0 + (i + 1) * chunk_s
        record = {"format": schema.FORMAT_VERSION, "i": i, "t0": lo, "t1": hi,
                  "topics": topics_out}
        blob = gzip.compress(msgpack.packb(record, use_bin_type=True), 6)
        _atomic_write_bytes(os.path.join(out_dir, schema.chunk_name(i)), blob)
        m["chunks"].append({"i": i, "t0": lo, "t1": hi, "n": sum(counts.values()),
                            "topics": counts})
        ov.note_chunk(i, counts)
        m["chunks_done"] = i + 1
        now = time.monotonic()
        if now - state["last_write"] >= MANIFEST_MIN_INTERVAL_S:
            # Mid-run manifests carry the running census; the chunk index is
            # published only at completion (schema: "[] until complete").
            snap = dict(m)
            snap["chunks"] = []
            write_manifest(snap)
            state["last_write"] = now

    # Indexed: log-time order (the seeking reader merges overlapping chunks
    # lazily). Unindexed: FILE order — the non-seeking reader's log-time mode
    # sorts the whole bag in memory and loses everything on a truncated tail;
    # the two-chunk reorder buffer below absorbs the cross-channel disorder.
    it = iter(reader.iter_messages(log_time_order=indexed))
    while True:
        try:
            item = next(it)
        except StopIteration:
            break
        except Exception as exc:  # noqa: BLE001
            if indexed:
                raise
            # An unindexed bag is a killed recording: its tail may be truncated
            # mid-record. Everything before the damage is kept.
            _log("unindexed bag ends without a footer (%s%s); keeping what was read"
                 % (type(exc).__name__, (": %s" % exc) if str(exc) else ""))
            break
        sch, ch, msg = item
        topic = ch.topic
        typename = sch.name if sch is not None else ""
        if topic not in allow or sch is None or sch.encoding != "ros2msg":
            sk = m["skipped_topics"].setdefault(topic, {"type": typename, "count": 0})
            sk["count"] += 1
            continue
        t = msg.log_time / 1e9
        if t0_holder[0] is None:
            t0_holder[0] = t
            m["t0"] = t
        i = int((t - t0_holder[0]) // chunk_s)
        if i < state["next_seal"]:
            m["dropped_late"] += 1
            continue
        while i >= state["next_seal"] + 2:
            seal(state["next_seal"])
            state["next_seal"] += 1
        dec.ensure_schema(sch)
        obj = dec.decode(msg.data, typename)
        paths, values = dec.flatten(obj, typename)
        entry = open_chunks.setdefault(i, {}).setdefault(
            topic, {"type": typename, "rows": [], "paths": paths})
        entry["rows"].append((t, values))
        tc = m["topics"].setdefault(topic, {"type": typename, "count": 0})
        tc["count"] += 1
        if t_last[0] is None or t > t_last[0]:
            t_last[0] = t

        if size > 0:
            pct = int(10 * fh.tell() / size)
            if pct > state["progress"]:
                state["progress"] = pct
                _log("%d%% (%d chunks sealed)" % (min(pct * 10, 100), m["chunks_done"]))

    # Skipped-only bag: t0 from nothing allow-listed -> no chunks at all.
    if t0_holder[0] is not None:
        last = max(open_chunks) if open_chunks else state["next_seal"] - 1
        while state["next_seal"] <= last:
            seal(state["next_seal"])
            state["next_seal"] += 1
        t1 = t_last[0]
    else:
        t1 = None
    m["t1"] = t1
    m["chunks_total"] = len(m["chunks"])
    if t1 is not None:
        _write_json(os.path.join(out_dir, schema.OVERVIEW),
                    ov.build(t0_holder[0], t1, chunk_s))
    m["status"] = schema.STATUS_COMPLETE
    m["completed_at"] = _now_iso()
    write_manifest(m)
    _log("complete: %d chunks, %d topics, %d skipped topics, dropped_late=%d"
         % (len(m["chunks"]), len(m["topics"]), len(m["skipped_topics"]), m["dropped_late"]))


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def _bulk(root: str, cache_root: str, newest: Optional[int], chunk_s: float) -> int:
    dirs = sorted(d for d in os.listdir(root)
                  if RECORDING_DIR_RE.match(d) and os.path.isdir(os.path.join(root, d)))
    if newest is not None:
        dirs = dirs[-newest:] if newest > 0 else []
    rc = 0
    for d in dirs:
        mcaps = [f for f in os.listdir(os.path.join(root, d)) if f.endswith(".mcap")]
        if len(mcaps) != 1:
            _log("skip %s: %d .mcap files" % (d, len(mcaps)))
            continue
        bag = os.path.join(root, d, mcaps[0])
        out = os.path.join(cache_root, d)
        try:
            with open(os.path.join(out, schema.MANIFEST)) as fh:
                old = json.load(fh)
            st = os.stat(bag)
            src = old.get("source", {})
            if (old.get("status") == schema.STATUS_COMPLETE
                    and old.get("format") == schema.FORMAT_VERSION
                    and src.get("size_bytes") == st.st_size
                    and src.get("mtime") == st.st_mtime):
                _log("skip %s: already complete" % d)
                continue
        except (OSError, ValueError):
            pass
        _log("converting %s" % d)
        res = convert(bag, out, chunk_s=chunk_s, recording_id=d)
        if res["status"] != schema.STATUS_COMPLETE:
            rc = 1
    return rc


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="python -m replay.convert", description=__doc__.split("\n")[0])
    p.add_argument("--bag")
    p.add_argument("--out")
    p.add_argument("--chunk-s", type=float, default=schema.CHUNK_S)
    p.add_argument("--recording-id")
    p.add_argument("--bulk")
    p.add_argument("--cache-root")
    p.add_argument("--newest", type=int)
    a = p.parse_args(argv)
    if a.bulk:
        if not a.cache_root:
            p.error("--bulk needs --cache-root")
        return _bulk(a.bulk, a.cache_root, a.newest, a.chunk_s)
    if not (a.bag and a.out):
        p.error("--bag and --out are required (or --bulk/--cache-root)")
    res = convert(a.bag, a.out, chunk_s=a.chunk_s, recording_id=a.recording_id)
    return 0 if res["status"] == schema.STATUS_COMPLETE else 1


if __name__ == "__main__":
    sys.exit(main())
