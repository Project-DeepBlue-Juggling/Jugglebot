"""Platform IMU capture: timestamp reconstruction, SI scaling and the capture-file writer.

Pure Python + numpy (no ROS). Contract: ``ros_ws/docs/platform_imu_protocol.md`` sections 3 and 5.
The uploader (``teensy_link.platform_imu``) hands this module the decoded header, the anchors and the
raw int16 sample words; this module turns them into per-sample wall-clock timestamps and writes them.

Thread model: ``build_capture`` runs on the uploader's worker thread, ``ImuCaptureWriter`` owns its own
writer thread behind a bounded queue (the ``HandCaptureWriter`` precedent). ``submit`` never blocks and
never raises, so no node hot path can be hurt by a slow disk.

File format (why npz, not hand_capture's CSV): a capture is 32000 x 6 int16 plus timestamps; CSV would
be about 4 MB of text per Attempt and lose the raw int16, while ``numpy.load`` reads an npz in one call
with exact dtypes. A small JSON sidecar with the same metadata sits beside it for humans and the GUI.
"""

from __future__ import annotations

import json
import logging
import math
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# Anchor flag bits (protocol_config.PlatformImuAnchorFlag); duplicated as plain ints so this module
# is importable without the generated config. tests pin them against the generated enum.
ANCHOR_DISCONTINUITY = 0x01
ANCHOR_WALL_INVALID = 0x02
ANCHOR_SYNC_STEP = 0x04

_U32 = 1 << 32


# ── least squares helpers ────────────────────────────────────────────────────────────────────────

def _ls_line(x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
    """Centered least squares line. Returns ``(slope, x0, y0)`` with ``y = y0 + slope * (x - x0)``.

    Centering matters here: wall_us is about 1.7e15 and local_us about 4e9, so an uncentered normal
    equation would lose the low digits the fit exists to recover."""
    x0 = float(np.mean(x))
    y0 = float(np.mean(y))
    dx = x - x0
    den = float(np.dot(dx, dx))
    if x.size < 2 or den <= 0.0:
        return 0.0, x0, y0
    return float(np.dot(dx, y - y0) / den), x0, y0


def _unwrap_u32(vals: Sequence[int]) -> np.ndarray:
    """Unwrap a monotone u32 microsecond counter (it wraps every 71.6 min)."""
    out = np.empty(len(vals), dtype=np.float64)
    off = 0
    prev = None
    for i, v in enumerate(vals):
        v = int(v)
        if prev is not None and v - prev < -(_U32 // 2):
            off += _U32
        out[i] = v + off
        prev = v
    return out


# ── the time model ───────────────────────────────────────────────────────────────────────────────

@dataclass
class Segment:
    """One run of samples between discontinuities, with its fitted ``local_us(k) = a + b k``."""
    k_lo: int                      # first retained-sample index this segment owns
    k_hi: int                      # one past the last
    n_anchors: int
    a: float = float("nan")
    b: float = float("nan")        # microseconds per sample (measured, not assumed)
    resid_rms_us: float = float("nan")   # rms of anchor residuals about the least-squares line
    lag_us: float = 0.0            # how far the lower envelope sits below that line
    note: str = ""

    @property
    def rate_hz(self) -> float:
        return 1e6 / self.b if self.b and self.b > 0 else float("nan")


class TimeModel:
    """Sample index to wall time, from the anchors (spec section 5).

    ``k`` is the retained-sample index (0..N-1). An anchor with ``index`` I belongs to sample
    ``I - 1 - first_index``. Per segment between DISCONTINUITY anchors the slope of
    ``local_us(k)`` is the least-squares slope and the intercept is the LOWER ENVELOPE of the
    residuals, because every anchor is an upper bound on its sample's time (drain jitter only adds
    lag). Local time then maps to wall time through a straight-line fit of ``wall_us - local_us``
    against ``local_us`` per run between SYNC_STEP anchors, which keeps the 0x7DD slave's dispatch
    jitter out of the rate fit.
    """

    def __init__(self, header: Any, anchors: Sequence[Any]):
        self.header = header
        self.anchors = list(anchors)
        self.n = int(header.n_samples)
        self.nominal_us = 1e6 / float(header.odr_hz or 1000)
        self.segments: List[Segment] = []
        self._boundaries: np.ndarray = np.zeros(0)
        self.wall_ok = False
        self.wall_resid_rms_us = float("nan")
        self._wall_groups: List[Tuple[float, float, float]] = []   # (slope, x0, y0) per run
        self._step_locals: np.ndarray = np.zeros(0)
        self._fit()

    # -- fitting --
    def _fit(self) -> None:
        h = self.header
        anchors = self.anchors
        if not anchors:
            self.segments = [Segment(0, self.n, 0, note="no anchors")]
            return
        local = _unwrap_u32([a.local_us for a in anchors])
        ks = np.array([a.index - 1 - h.first_index for a in anchors], dtype=np.float64)
        disc = [i for i, a in enumerate(anchors) if (a.flags & ANCHOR_DISCONTINUITY) and i >= 1]
        # Segment s owns anchors [starts[s], ends[s]] and samples [bounds[s-1], bounds[s]).
        starts = [0] + disc
        ends = [j - 1 for j in disc] + [len(anchors) - 1]
        bounds = [int(anchors[j - 1].index - h.first_index) for j in disc]
        self._boundaries = np.array(bounds, dtype=np.float64)
        lows = [0] + bounds
        highs = bounds + [self.n]
        for s, e, lo, hi in zip(starts, ends, lows, highs):
            seg = Segment(max(lo, 0), min(hi, self.n), e - s + 1)
            x = ks[s:e + 1]
            y = local[s:e + 1]
            self._fit_segment(seg, x, y)
            self.segments.append(seg)
        self._fit_wall(local)

    def _fit_segment(self, seg: Segment, x: np.ndarray, y: np.ndarray) -> None:
        nominal = self.nominal_us
        if x.size >= 2 and float(np.ptp(x)) > 0:
            b, x0, y0 = _ls_line(x, y)
            if not (0.5 * nominal <= b <= 2.0 * nominal):
                seg.note = "slope outside 0.5-2x nominal; nominal rate used"
                b = nominal
                resid = y - (y0 + b * (x - x0))
                seg.resid_rms_us = float(math.sqrt(float(np.mean((resid - resid.mean()) ** 2))))
            else:
                resid = y - (y0 + b * (x - x0))
                seg.resid_rms_us = float(math.sqrt(float(np.mean(resid ** 2))))
            seg.b = b
            a_env = float(np.min(y - b * x))
            seg.lag_us = float(np.mean(y - b * x) - a_env)
            seg.a = a_env
        else:
            seg.note = "fewer than 2 anchors; nominal rate used"
            seg.b = nominal
            seg.a = float(y[0] - nominal * x[0]) if x.size else float("nan")

    def _fit_wall(self, local: np.ndarray) -> None:
        wall = np.array([a.wall_us for a in self.anchors], dtype=np.float64)
        valid = np.array([not (a.flags & ANCHOR_WALL_INVALID) for a in self.anchors])
        step = np.array([bool(a.flags & ANCHOR_SYNC_STEP) for a in self.anchors])
        if not valid.any():
            return
        gid = np.cumsum(step & valid)           # a new run starts AT each SYNC_STEP anchor
        step_locals = [float(local[i]) for i in range(len(local)) if step[i] and valid[i]]
        self._step_locals = np.array(step_locals, dtype=np.float64)
        groups: List[Tuple[float, float, float]] = []
        sq = []
        for g in range(int(gid.max()) + 1):
            m = valid & (gid == g)
            if not m.any():
                groups.append(groups[-1] if groups else (0.0, 0.0, 0.0))
                continue
            x = local[m]
            y = wall[m] - local[m]
            slope, x0, y0 = _ls_line(x, y)
            if x.size < 2:
                slope = 0.0
            groups.append((slope, x0, y0))
            sq.append((y - (y0 + slope * (x - x0))) ** 2)
        self._wall_groups = groups
        self.wall_ok = True
        self.wall_resid_rms_us = float(math.sqrt(float(np.mean(np.concatenate(sq))))) if sq else float("nan")

    # -- evaluation --
    def local_us(self, k: np.ndarray) -> np.ndarray:
        k = np.asarray(k, dtype=np.float64)
        seg_id = np.searchsorted(self._boundaries, k, side="right") if self._boundaries.size else np.zeros(k.shape, int)
        a = np.array([s.a for s in self.segments])
        b = np.array([s.b for s in self.segments])
        return a[seg_id] + b[seg_id] * k

    def wall_us(self, k: np.ndarray) -> np.ndarray:
        k = np.asarray(k, dtype=np.float64)
        if not self.wall_ok:
            return np.full(k.shape, np.nan)
        loc = self.local_us(k)
        gid = np.searchsorted(self._step_locals, loc, side="right") if self._step_locals.size else np.zeros(loc.shape, int)
        slope = np.array([g[0] for g in self._wall_groups])
        x0 = np.array([g[1] for g in self._wall_groups])
        y0 = np.array([g[2] for g in self._wall_groups])
        gid = np.minimum(gid, len(self._wall_groups) - 1)
        return loc + y0[gid] + slope[gid] * (loc - x0[gid])

    def select_range(self, t0_wall_us: float, t1_wall_us: float) -> Optional[Tuple[int, int]]:
        """Retained-sample range ``[k0, k1)`` whose wall time lies in ``[t0, t1]``, or None when the
        capture has no usable wall clock (caller then uploads everything)."""
        if not self.wall_ok or self.n <= 0:
            return None
        w = self.wall_us(np.arange(self.n))
        hit = np.nonzero((w >= t0_wall_us) & (w <= t1_wall_us))[0]
        if hit.size == 0:
            return (0, 0)
        return int(hit[0]), int(hit[-1]) + 1

    def summary(self) -> Dict[str, Any]:
        big = max(self.segments, key=lambda s: s.k_hi - s.k_lo) if self.segments else None
        return {
            "wall_ok": self.wall_ok,
            "wall_resid_rms_us": self.wall_resid_rms_us,
            "n_segments": len(self.segments),
            "rate_hz": big.rate_hz if big else float("nan"),
            "resid_rms_us": big.resid_rms_us if big else float("nan"),
            "segments": [
                {"k_lo": s.k_lo, "k_hi": s.k_hi, "n_anchors": s.n_anchors, "us_per_sample": s.b,
                 "rate_hz": s.rate_hz, "resid_rms_us": s.resid_rms_us, "lag_us": s.lag_us, "note": s.note}
                for s in self.segments],
        }


# ── the finished capture ─────────────────────────────────────────────────────────────────────────

@dataclass
class Capture:
    """Everything one capture file holds (the writer serialises this)."""
    header: Dict[str, Any]
    anchors: np.ndarray                 # structured: index u4, local_us u4, wall_us u8, flags u1
    raw: np.ndarray                     # int16 [n, 6] ax ay az gx gy gz, chip axes, covering [k0, k1)
    valid: np.ndarray                   # bool [n]: both words of the sample arrived
    k0: int
    t_wall_us: np.ndarray               # float64 [n]; NaN when the board never had wall time
    accel_g: np.ndarray                 # float32 [n, 3]
    gyro_dps: np.ndarray                # float32 [n, 3]
    fit: Dict[str, Any]
    flags: Dict[str, Any]
    meta: Dict[str, Any] = field(default_factory=dict)


ANCHOR_DTYPE = np.dtype([("index", "<u4"), ("local_us", "<u4"), ("wall_us", "<u8"), ("flags", "u1")])


def build_capture(header: Any, anchors: Sequence[Any], sample_words: bytes, word_present: bytes,
                  k0: int, flags: Dict[str, Any], meta: Optional[Dict[str, Any]] = None) -> Capture:
    """Assemble a :class:`Capture`. ``sample_words`` holds the words of samples ``k0..`` (6 B each,
    accel word then gyro word per sample); ``word_present`` is one byte per word (0 = never arrived)."""
    n_words = len(sample_words) // 6
    n = n_words // 2
    raw = np.frombuffer(bytes(sample_words[:12 * n]), dtype="<i2").reshape(n, 6).copy() if n else np.zeros((0, 6), "<i2")
    pres = np.frombuffer(bytes(word_present[:2 * n]), dtype=np.uint8).reshape(n, 2) if n else np.zeros((0, 2), np.uint8)
    valid = (pres != 0).all(axis=1) if n else np.zeros(0, bool)
    model = TimeModel(header, anchors)
    t = model.wall_us(np.arange(k0, k0 + n)) if n else np.zeros(0)
    acc_lsb = float(header.accel_lsb_per_g or 2048)
    gyr_lsb = float((header.gyro_lsb_per_dps_x10 or 328)) / 10.0
    anc = np.zeros(len(anchors), dtype=ANCHOR_DTYPE)
    for i, a in enumerate(anchors):
        anc[i] = (a.index, a.local_us, a.wall_us, a.flags)
    return Capture(
        header=header_to_dict(header), anchors=anc, raw=raw, valid=valid, k0=int(k0), t_wall_us=t,
        accel_g=(raw[:, :3] / acc_lsb).astype(np.float32), gyro_dps=(raw[:, 3:] / gyr_lsb).astype(np.float32),
        fit=model.summary(), flags=dict(flags), meta=dict(meta or {}))


def header_to_dict(header: Any) -> Dict[str, Any]:
    return {k: (int(v) if isinstance(v, (int, np.integer)) else v) for k, v in header.__dict__.items()}


# ── the writer ───────────────────────────────────────────────────────────────────────────────────

def _jsonable(o: Any) -> Any:
    if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
        return None
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return _jsonable(float(o))
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def capture_stem(capture: Capture, now: Optional[float] = None) -> str:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(now if now is not None else time.time()))
    return "%s_%04X" % (stamp, int(capture.header.get("capture_id", 0)) & 0xFFFF)


def write_capture(directory: str, capture: Capture, now: Optional[float] = None) -> str:
    """Write ``<UTC stamp>_<capture_id hex>.npz`` and ``.json`` (atomically: temp then rename).
    Returns the npz path. Raises on I/O failure; the writer thread catches it."""
    directory = os.path.expanduser(directory)
    os.makedirs(directory, exist_ok=True)
    stem = capture_stem(capture, now)
    path = os.path.join(directory, stem + ".npz")
    n = 1
    while os.path.exists(path):
        path = os.path.join(directory, "%s_%d.npz" % (stem, n))
        n += 1
    meta = _jsonable({"format": "platform_imu_capture/1", "header": capture.header, "flags": capture.flags,
                      "fit": capture.fit, "k0": capture.k0, "n_samples_written": int(capture.raw.shape[0]),
                      "columns": ["ax", "ay", "az", "gx", "gy", "gz"], "units": {"accel": "g", "gyro": "dps"},
                      **capture.meta})
    tmp = path + ".tmp"
    with open(tmp, "wb") as fh:
        np.savez_compressed(
            fh, raw=capture.raw, valid=capture.valid, t_wall_us=capture.t_wall_us,
            accel_g=capture.accel_g, gyro_dps=capture.gyro_dps, anchors=capture.anchors,
            k0=np.int64(capture.k0), meta_json=np.array(json.dumps(meta)))
    os.replace(tmp, path)
    with open(path[:-4] + ".json", "w") as fh:
        json.dump(meta, fh, indent=1, sort_keys=True)
    return path


class ImuCaptureWriter:
    """Bounded queue + writer thread, as ``HandCaptureWriter``: ``submit`` only does ``put_nowait``."""

    def __init__(self, directory: str, maxsize: int = 4):
        self.directory = directory
        self._q: "queue.Queue" = queue.Queue(maxsize=maxsize)
        self.dropped = 0
        self.written = 0
        self.errors = 0
        self.last_path: Optional[str] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="platform_imu_writer", daemon=True)
        self._started = False

    def start(self) -> "ImuCaptureWriter":
        if not self._started:
            self._started = True
            self._thread.start()
        return self

    def submit(self, capture: Capture) -> bool:
        try:
            self._q.put_nowait(capture)
            return True
        except Exception:  # noqa: BLE001 -- full or closed: a diagnostic never breaks its caller
            self.dropped += 1
            return False

    def _run(self) -> None:
        while True:
            try:
                cap = self._q.get(timeout=0.2)
            except queue.Empty:
                if self._stop.is_set():
                    return
                continue
            try:
                self.last_path = write_capture(self.directory, cap)
                self.written += 1
                logger.info("platform IMU capture written: %s", self.last_path)
            except Exception:  # noqa: BLE001
                self.errors += 1
                logger.exception("platform IMU capture write failed")

    def close(self, timeout: float = 10.0) -> None:
        self._stop.set()
        if self._started:
            self._thread.join(timeout)
