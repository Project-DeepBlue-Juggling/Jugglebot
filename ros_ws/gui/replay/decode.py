"""Venv-only decode helpers shared by the overview pass (``overview.py``) and the
test oracle (``convert.py``): the embedded-schema typestore, the flatten rule
of ``schema.py``, the footer check, and the ``Overview`` builder.

Needs ``rosbags`` and ``numpy``; never imported by the stdlib GUI server.
"""
from __future__ import annotations

import dataclasses
import os
from typing import Any, Dict, List, Tuple

import numpy as np
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from . import schema

MCAP_MAGIC = b"\x89MCAP0\r\n"
SCHEMA_SEPARATOR = "=" * 80

FAULT_FLAGS = ("has_fatal_odrive_error", "has_fatal_can_error", "has_undervoltage")


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


class Decoder:
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

class Overview:
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
            flags = [cols.get(f) for f in FAULT_FLAGS]
            homed = cols.get("is_homed")
            lev = cols.get("levelling_complete")
            for k, t in enumerate(ts):
                fault = any(bool(f[k]) for f in flags if f is not None)
                h = bool(homed[k]) if homed is not None else False
                lv = bool(lev[k]) if lev is not None else False
                prev = self._rs_prev
                if prev is not None:
                    if fault and not prev[0]:
                        names = [n for n, f in zip(FAULT_FLAGS, flags) if f is not None and f[k]]
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
        elif topic == "/bb/calibration_attempt":
            # every sweep (success and failure); /bb/calibration_result only
            # carries the calibration in force since keep-last-good 2026-10-10
            ok = cols.get("success")
            for k, t in enumerate(ts):
                good = True if ok is None else bool(ok[k])
                self.ticks.append({"t": t, "kind": "bb_calibration",
                                   "label": "ok" if good else "failed"})

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
