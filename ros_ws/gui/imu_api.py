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

Frames. The .npz on disk is always the raw IMU frame. When the mount calibration
(``config/platform_imu_mount.json``, loaded through ``teensy_link/platform_imu_mount.py`` by file path, which
is also stdlib only) is present, ``/data`` serves accel and gyro rotated into the PLATFORM frame by default;
``?frame=imu`` returns the raw arrays and ``?frame=platform`` insists on the rotated ones (409 if there is
no calibration). The list rows carry ``frame`` (what ``/data`` serves by default) and ``cal_date``; the
browser asks for exactly the frame it labels. Rotation only: no gravity or gyro-bias subtraction. A malformed
calibration file is a 500 on ``/data`` (loud) and the list reports ``frame: "imu"`` with ``cal_error``.
"""
from __future__ import annotations

import array
import importlib.util
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


_HERE = os.path.dirname(os.path.abspath(__file__))
_MOUNT_MODULE_PATH = os.path.join(_HERE, "..", "..", "teensy_link", "platform_imu_mount.py")
DEFAULT_MOUNT_PATH = os.path.join(_HERE, "..", "..", "config", "platform_imu_mount.json")
_mount_mod = None


def _mount_module():
    """teensy_link/platform_imu_mount.py loaded by path (not through the teensy_link package)."""
    global _mount_mod
    if _mount_mod is None:
        spec = importlib.util.spec_from_file_location("_jb_platform_imu_mount", _MOUNT_MODULE_PATH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _mount_mod = mod
    return _mount_mod


def load_mount(mount_path=None):
    """(calibration or None, error string or None); None calibration and no error when the file is absent."""
    mod = _mount_module()
    try:
        return mod.load_mount(os.path.abspath(mount_path or DEFAULT_MOUNT_PATH)), None
    except mod.MountCalError as exc:
        return None, str(exc)


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


def encode_capture(npz_path, matrix=None):
    """The JBIMU1 blob for one .npz (raises NpyError/KeyError/OSError/BadZipFile). ``matrix`` (3x3, IMU to
    Platform) rotates accel and gyro; None leaves them raw."""
    with zipfile.ZipFile(npz_path) as zf:
        n, t_us = _read_array(zf, "t_wall_us", "d", 1)
        na, acc = _read_array(zf, "accel_g", "f", 3)
        ng, gyr = _read_array(zf, "gyro_dps", "f", 3)
        nv, val = _read_array(zf, "valid", "B", 1)
    if not (n == na == ng == nv):
        raise NpyError("array length mismatch")
    if matrix is not None:
        rot = _mount_module().rotate_xyz
        acc, gyr = rot(acc, matrix), rot(gyr, matrix)
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


def _summary(stem, side, frame="imu", cal_date=None):
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
        "frame": frame,          # what /data serves by default: "platform" or "imu"
        "cal_date": cal_date,    # the mount calibration's date when frame is "platform"
    }


def list_captures(directory, t_from=None, t_to=None, mount_path=None):
    rows = []
    cal, _err = load_mount(mount_path)
    frame, cal_date = ("platform", cal.date) if cal else ("imu", None)
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
            row = _summary(stem, side, frame, cal_date) if isinstance(side, dict) else None
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


def _qstr(query, key):
    for part in query.split("&"):
        k, _, v = part.partition("=")
        if k == key:
            return v
    return None


_blob_cache = {}     # (npz path, npz mtime ns, size, frame, cal path, cal mtime ns) -> blob; newest 4 kept
_BLOB_CACHE_MAX = 4


def _file_sig(p):
    try:
        st = os.stat(p)
        return (st.st_mtime_ns, st.st_size)
    except OSError:
        return None


def handle(path, directory, mount_path=None):
    """(status, content_type, body bytes) for an IMU route, or None if ``path``
    is not one. ``mount_path`` is the mount calibration file (default: the repo's)."""
    route, _, query = path.partition("?")
    if route == LIST_ROUTE:
        rows = list_captures(directory, _qfloat(query, "from"), _qfloat(query, "to"), mount_path)
        body = {"captures": rows}
        _cal, err = load_mount(mount_path)
        if err:
            body["cal_error"] = err
        return 200, "application/json", json.dumps(body).encode("utf-8")
    m = _DATA_RE.match(route)
    if not m:
        return None
    stem = m.group(1)
    if not STEM_RE.match(stem):
        return _err(404, "unknown capture")
    npz = os.path.join(directory, stem + ".npz")
    if not os.path.isfile(npz):
        return _err(404, "unknown capture")
    want = _qstr(query, "frame")
    if want not in (None, "", "imu", "platform"):
        return _err(400, "frame must be imu or platform")
    cal, cal_err = load_mount(mount_path)
    if cal_err and want != "imu":
        return _err(500, "mount calibration unusable: {}".format(cal_err))
    if want == "platform" and cal is None:
        return _err(409, "no mount calibration: the Platform frame is not available")
    use_cal = cal if want in (None, "", "platform") else None
    key = (npz, _file_sig(npz), "platform" if use_cal else "imu",
           use_cal.path if use_cal else None, _file_sig(use_cal.path) if use_cal else None)
    blob = _blob_cache.get(key)
    if blob is not None:
        return 200, "application/octet-stream", blob
    try:
        blob = encode_capture(npz, use_cal.matrix if use_cal else None)
        _blob_cache[key] = blob
        while len(_blob_cache) > _BLOB_CACHE_MAX:
            _blob_cache.pop(next(iter(_blob_cache)))
        return 200, "application/octet-stream", blob
    except Exception as exc:  # any unreadable capture is a JSON 500, never a dropped connection
        return _err(500, "cannot read capture: {}".format(exc))


def _err(status, msg):
    return status, "application/json", json.dumps({"error": msg}).encode("utf-8")
