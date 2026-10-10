"""TEST ORACLE: convert one MCAP recording into the per-recording replay cache.

Since Phase 4 (design 06) the browser reads the MCAP directly and nothing in
production calls this module; it survives so the JS decoder
(``js/replay/mcap-decode.js``) and the overview pass (``overview.py``) have an
independent Python reference to be compared against, value for value.

The cache layout, chunk record, manifest and overview shapes are defined in
``schema.py`` (the contract); this module only produces them. Runs under the
project VENV (needs ``mcap``, ``rosbags``, ``msgpack``).

Usage (``cwd=ros_ws/gui`` so ``-m`` resolves the package)::

    python -m replay.convert --bag <file.mcap> --out <cache_dir> [--chunk-s 10]

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
import datetime as _dt
import gzip
import json
import math
import os
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import msgpack
from mcap.reader import NonSeekingReader, SeekingReader

from . import schema
from .decode import (Decoder as _Decoder, FAULT_FLAGS, Overview as _Overview,  # noqa: F401
                     _field_names, is_indexed, parse_embedded_schema)  # noqa: F401

MANIFEST_MIN_INTERVAL_S = 1.0


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

def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="python -m replay.convert", description=__doc__.split("\n")[0])
    p.add_argument("--bag")
    p.add_argument("--out")
    p.add_argument("--chunk-s", type=float, default=schema.CHUNK_S)
    p.add_argument("--recording-id")
    a = p.parse_args(argv)
    if not (a.bag and a.out):
        p.error("--bag and --out are required")
    res = convert(a.bag, a.out, chunk_s=a.chunk_s, recording_id=a.recording_id)
    return 0 if res["status"] == schema.STATUS_COMPLETE else 1


if __name__ == "__main__":
    sys.exit(main())
