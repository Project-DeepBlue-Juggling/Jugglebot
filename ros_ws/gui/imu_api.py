"""IMU capture routes for the GUI server (stdlib only, no numpy, Python 3.8).

``GET /api/imu/captures[?from=&to=]`` lists the Platform IMU captures
(``<stem>.json`` sidecars written by ``teensy_link/platform_imu_capture.py``);
``GET /api/imu/captures/<stem>/data`` streams one capture's samples as a small
binary blob the browser reads into typed arrays:

    8 bytes  magic  b"JBIMU1\\0\\0"
    uint32   n
    uint32   reserved (0)
    float64  t[n]            epoch SECONDS
    float32  accel_g[n*3]    row-major
    float32  gyro_dps[n*3]   row-major
    uint8    valid[n]

All little-endian. The .npz is read with zipfile and a minimal .npy parser.
"""
from __future__ import annotations

import array
import json
import math
import os
import re
import struct
import sys
import zipfile

MAGIC = b"JBIMU1\0\0"
STEM_RE = re.compile(r"^[0-9]{8}T[0-9]{6}Z_[0-9A-Fa-f]{4}(_[0-9]+)?$")
LIST_ROUTE = "/api/imu/captures"
_DATA_RE = re.compile(r"^/api/imu/captures/([^/]+)/data$")


class NpyError(ValueError):
    pass


def _parse_npy(buf):
    """(descr, shape, data offset) of a C-order .npy (v1/v2/v3 header)."""
    if buf[:6] != b"\x93NUMPY":
        raise NpyError("not an npy file")
    major = buf[6]
    if major == 1:
        hlen, off = struct.unpack("<H", buf[8:10])[0], 10
    elif major in (2, 3):
        hlen, off = struct.unpack("<I", buf[8:12])[0], 12
    else:
        raise NpyError("unsupported npy version %d" % major)
    header = buf[off:off + hlen].decode("latin1")
    m = re.search(r"'descr'\s*:\s*'([^']*)'", header)
    f = re.search(r"'fortran_order'\s*:\s*(True|False)", header)
    s = re.search(r"'shape'\s*:\s*\(([^)]*)\)", header)
    if not (m and f and s):
        raise NpyError("unparseable npy header")
    if f.group(1) == "True":
        raise NpyError("fortran-order npy unsupported")
    shape = tuple(int(x) for x in s.group(1).split(",") if x.strip())
    return m.group(1), shape, off + hlen


_ITEM = {"<f8": ("d", 8), "<f4": ("f", 4), "|b1": ("B", 1), "|u1": ("B", 1)}


def _read_array(zf, name, want_descr, want_cols):
    """(n, array.array) from member ``name``; little-endian, C-order only."""
    buf = zf.read(name + ".npy")
    descr, shape, off = _parse_npy(buf)
    if descr not in _ITEM or (descr in ("|b1", "|u1")) != (want_descr == "B"):
        raise NpyError("%s: unsupported dtype %s" % (name, descr))
    code, size = _ITEM[descr]
    if want_cols == 1 and len(shape) != 1 or want_cols > 1 and shape[1:] != (want_cols,):
        raise NpyError("%s: unexpected shape %r" % (name, shape))
    count = 1
    for d in shape:
        count *= d
    if len(buf) - off < count * size:
        raise NpyError("%s: truncated" % name)
    a = array.array(code)
    a.frombytes(buf[off:off + count * size])
    if sys.byteorder == "big":
        a.byteswap()
    return shape[0], a


def encode_capture(npz_path):
    """The JBIMU1 blob for one .npz (raises NpyError/KeyError/OSError/BadZipFile)."""
    with zipfile.ZipFile(npz_path) as zf:
        n, t_us = _read_array(zf, "t_wall_us", "d", 1)
        na, acc = _read_array(zf, "accel_g", "f", 3)
        ng, gyr = _read_array(zf, "gyro_dps", "f", 3)
        nv, val = _read_array(zf, "valid", "B", 1)
    if not (n == na == ng == nv):
        raise NpyError("array length mismatch")
    t = array.array("d", (x / 1e6 for x in t_us))
    out = [MAGIC, struct.pack("<II", n, 0)]
    for a in (t, acc, gyr, val):
        if sys.byteorder == "big":
            a = array.array(a.typecode, a)
            a.byteswap()
        out.append(a.tobytes())
    return b"".join(out)


def _num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _summary(stem, side):
    """One list row from a parsed sidecar dict, or None if it lacks the times."""
    h = side.get("header") or {}
    fit = side.get("fit") or {}
    flags = side.get("flags") or {}
    n = side.get("n_samples_written", h.get("n_samples"))
    freeze = h.get("freeze_wall_us")
    rate = fit.get("rate_hz")
    if not (_num(n) and _num(freeze) and _num(rate) and rate > 0):
        return None
    # span_wall_s in the sidecar is the REQUESTED window (it can end after the
    # freeze, and is [null, null] for operator captures), so it is not the
    # sample window; the sample window is anchored on the freeze instant.
    t_end = freeze / 1e6
    t_start = t_end - n / rate
    return {
        "stem": stem,
        "capture_id": h.get("capture_id"),
        "label": side.get("label"),
        "attempt_id": side.get("attempt_id"),
        "t_start": t_start,
        "t_end": t_end,
        "n_samples": int(n),
        "complete": bool(flags.get("complete")),
        "crc_ok": bool(flags.get("crc_ok")),
        "wall_ok": bool(fit.get("wall_ok")),
        "temp_c_freeze": h.get("temp_c_freeze") if _num(h.get("temp_c_freeze")) else None,
    }


def list_captures(directory, t_from=None, t_to=None):
    rows = []
    try:
        names = sorted(os.listdir(directory))
    except OSError:
        return rows
    for name in names:
        if not name.endswith(".json"):
            continue
        stem = name[:-5]
        if not STEM_RE.match(stem):
            continue
        try:
            with open(os.path.join(directory, name), "r") as f:
                side = json.load(f)
            row = _summary(stem, side) if isinstance(side, dict) else None
        except Exception:  # a malformed sidecar is skipped, never a failed list
            row = None
        if row is None:
            continue
        if t_from is not None and row["t_end"] < t_from:
            continue
        if t_to is not None and row["t_start"] > t_to:
            continue
        rows.append(row)
    rows.sort(key=lambda r: r["t_start"])
    return rows


def _qfloat(query, key):
    for part in query.split("&"):
        k, _, v = part.partition("=")
        if k == key and v:
            try:
                x = float(v)
            except ValueError:
                return None
            return x if math.isfinite(x) else None
    return None


def handle(path, directory):
    """(status, content_type, body bytes) for an IMU route, or None if ``path``
    is not one."""
    route, _, query = path.partition("?")
    if route == LIST_ROUTE:
        rows = list_captures(directory, _qfloat(query, "from"), _qfloat(query, "to"))
        return 200, "application/json", json.dumps({"captures": rows}).encode("utf-8")
    m = _DATA_RE.match(route)
    if not m:
        return None
    stem = m.group(1)
    if not STEM_RE.match(stem):
        return _err(404, "unknown capture")
    npz = os.path.join(directory, stem + ".npz")
    if not os.path.isfile(npz):
        return _err(404, "unknown capture")
    try:
        return 200, "application/octet-stream", encode_capture(npz)
    except Exception as exc:  # any unreadable capture is a JSON 500, never a dropped connection
        return _err(500, "cannot read capture: {}".format(exc))


def _err(status, msg):
    return status, "application/json", json.dumps({"error": msg}).encode("utf-8")
