"""Ball Butler base-marker frame: BB's pose relative to four fixed markers on its shelf.

Why. QTM's world frame moves between sittings (0.3–1.5 mm, non-rigidly) and is
occasionally recalibrated, while BB itself is bolted to a shelf and does not
move. The sweep estimator (:mod:`jugglebot.bb_calibration`) measures BB in the
WORLD frame, so every QTM shift reads as a BB move: the consistency gate
refuses it, or accepts it with no way to tell "QTM moved"
from "BB moved". Four markers fastened to the shelf (the base frame) are
static in BB's frame; posing them every sitting separates the two:

    BB-in-base   (κ, p_b)   constant while BB and the base stay on the shelf
    base-in-world (h, o, R)  changes when QTM's frame changes

and :func:`compose_bb_pose` is the inverse of :func:`bb_in_base` (kept for tests
and tools; since Phase 1 the published BB pose is always the sweep's own, and
kappa feeds only the epoch advisory).

Geometry (owner's nominal, 3D-printed, fastened to the shelf). Four coplanar
markers in an L: corner C; short span C–S, |CS| = 115 mm; long span C–M–E
collinear, |CM| = 80 mm, |CE| = 240 mm. They need no QTM label: they are
identified by their pairwise distances (:func:`identify_base_markers`) among
ALL the frame's markers except BB's own yaw-stage markers
(:func:`base_candidate_points`), so a QTM rigid body that grabs (labels) a
base marker cannot hide it — on 2026-10-10 14:10 the re-enabled Base /
Catching Cone bodies labelled two of the four and the unlabelled-only
identifier saw 0 of 805 frames. A planar template's mirror image is reachable
by a proper rotation (a half-turn about an in-plane axis), so the handedness is
fixed by requiring the posed plane normal to point UP (world +z).

Base frame (the canonical frame of the as-built template): origin C; z the
plane normal, up; x along C→E with its normal component removed; y = z × x.
Heading h = the angle of the posed x axis projected on world xy (deg).

BB-in-base constants per accepted sweep: κ = φ − h, the sweep's PUBLISHED yaw
offset φ (in the pinned gauge, about world z, as the estimator defines it)
minus the base heading; p_b = Rᵀ(p_w − o), the axis point in the base frame.

Gauge. κ is measured from the published, pinned offset, so at the sitting it
is measured in the base-derived offset h + κ equals the constellation's
offset: same gauge, the frame the deployed aim correction was fitted in. Each
κ record carries the marker template's ``gauge.pinned_yaw_offset_deg`` it was
measured under; pooling moves each to the CURRENT pin (κ + pin_now −
pin_record). So a landing-derived re-pin (BallButler ``settle_yaw_gauge.py``:
"add δ to gauge.pinned_yaw_offset_deg") moves κ by δ too, with no re-sweep.
The pin's own uncertainty (``gauge.pin_uncertainty_deg``) is carried along and
reported, not folded into σ (it is common to every sitting).

A tilted base plane. The yaw offset stays defined about world z. With the
plane tilted by τ (1.04° on 2026-10-10, in the same direction as BB's yaw axis:
the shelf, or QTM's z, is tilted), a rotation by Δψ about the plane normal
changes the projected heading by Δψ·(1 + O(τ²)) (≈ 1.7e-4 relative), and a
QTM rotation about a horizontal axis by α changes h and φ only at O(α·τ), so
κ is invariant to well below the sweep's precision.

All functions are pure Python + numpy — no ROS2 dependency.
See ``logbook/2026-10-10-bb-base-marker-frame.md``.
"""

from __future__ import annotations

import dataclasses
import itertools
import json
import math
import os
import re
import shutil
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .bb_calibration import kabsch, _assign, wrap_pi

#: Base marker ids, template order: corner, middle (long span), end (long
#: span), short-span end.
BASE_MARKER_NAMES = ('C', 'M', 'E', 'S')
#: Owner's nominal spans (mm).
NOMINAL_CS_MM = 115.0
NOMINAL_CM_MM = 80.0
NOMINAL_CE_MM = 240.0

#: Pair-distance tolerance (mm) of the label-free identification, and the
#: re-assignment radius of a tracked frame. The as-built spans differ from the
#: nominal by ≤ 0.33 mm (2026-10-10) and the closest nominal pair distances
#: (115 vs 140, 140 vs 160) are ≥ 20 mm apart.
BASE_MATCH_TOL_MM = 4.0
#: Fewest base markers for a pose (three give a residual; the four are
#: coplanar, so three non-collinear ones fix the pose).
BASE_MIN_MATCHED = 3
#: The matched template markers must span a plane: the second singular value
#: of their centred coordinates at least this (mm). C, M and E are collinear,
#: so a match without S fixes no rotation about the C–E line (and so no
#: handedness either: a mirrored L matches C–M–E with the normal up) — S is
#: required, plus any two of C, M, E.
BASE_MIN_SPREAD_MM = 10.0
#: A frame whose rigid fit RMS exceeds this (mm) gives no pose. 2026-10-10:
#: median 0.011 mm, p99 0.026 mm.
BASE_FRAME_MAX_RMS_MM = 1.0
#: The posed plane normal must be within this of world +z (deg): fixes the
#: handedness, and refuses a frame that is not lying on the shelf.
BASE_MAX_TILT_DEG = 10.0
#: Fewest posed frames for a static base pose (~0.5 s at the ~100 distinct
#: frames/s mocap_node sees).
BASE_MIN_FRAMES = 50
#: Static average: a frame whose origin is more than this (mm) or whose
#: heading is more than BASE_OUTLIER_DEG from the median is an outlier
#: (per-frame scatter 2026-10-10: 0.012–0.033 mm, 0.005°).
BASE_OUTLIER_MM = 1.0
BASE_OUTLIER_DEG = 0.05
#: Block length (s) for the standard error of the static mean: frame noise is
#: correlated over tenths of a second, so SE = SD(block means)/√n_blocks.
BASE_SE_BLOCK_S = 2.0
#: Base template residual (mm): the largest |observed - as-built| pair distance
#: (window mean). Above it the base frame's pose is not trusted: the sweep's kappa
#: is NOT pooled and NOT judged (flag BASE_FRAME_RESIDUAL; never a refusal).
#: 2026-10-10: <= 0.035 mm SD per pair, mean 0 by construction.
BASE_MAX_RESIDUAL_MM = 0.5
#: Advisory bands. Fixed numbers: scaling a cross-session band by one sitting's
#: sigma was root cause C2 (logbook 2026-10-15-bb-calibration-phase0-characterise,
#: "Advisory bands (input to Phase 1)"). None of them refuses a sweep.
#: kappa epoch half-width (deg): the day-to-day kappa step is ~0.25-0.31 deg, one
#: sitting's spread ~0.03 deg (Phase 0 entry, section "Advisory bands").
KAPPA_EPOCH_BAND_DEG = 0.20
#: BB-in-base axis-point shift (mm) vs the assigned epoch's median (Phase 0 entry,
#: section "Advisory bands").
BB_IN_BASE_SHIFT_ADVISORY_MM = 1.0
#: Base-in-world shift vs base_ref beyond either = the QTM frame moved (Phase 0
#: entry, section "Advisory bands"; QTM re-alignment steps were >2 mm / >0.05 deg).
WORLD_EPOCH_SHIFT_MM = 2.0
WORLD_EPOCH_SHIFT_DEG = 0.05
#: Per-sweep κ SD by yaw source (deg), measured 2026-10-10: stamped 0.030°
#: (13:45 sitting, 8 sweeps), heartbeat 0.050° (12:26 sitting, 10 live sweeps).
#: An unknown source gets the larger.
KAPPA_SWEEP_SD_DEG = {'stamped': 0.030, 'heartbeat': 0.050}
#: Robust pool (:func:`kappa_outliers`): a record more than CLIP_N·σ_g from the
#: pool median is excluded, σ_g = max(1.4826·MAD of its yaw-source group about
#: the median, SIGMA_FLOOR[source]) for a group of ≥ MIN_GROUP records, else
#: max(SIGMA_FLOOR, KAPPA_SWEEP_SD_DEG)[source]. The floor bounds the false
#: exclusions a small group's MAD (which reads low) would make. Stamped 0.020°
#: = its per-sweep SD without the two suspect sweeps (12:26 offline stamped
#: replay 0.021°, 13:45 sitting less its +0.07° sweep 0.020°); heartbeat 0.050°
#: (its measured SD — no cleaner estimate exists). If the stamped σ is really
#: 0.030°, a good record is excluded with p ≈ 4.6 % (|z| > 2), not 0.3 %.
KAPPA_POOL_CLIP_N = 3.0
KAPPA_POOL_SIGMA_FLOOR_DEG = {'stamped': 0.020, 'heartbeat': 0.050}
KAPPA_POOL_MIN_GROUP = 5
#: QTM rigid-body names of BB's own yaw-stage markers (labels ``<body> - i``):
#: the only markers the base identifier does not search (they turn with yaw).
BB_MARKER_BODIES = ('Ball Butler', 'Ball_Butler')
#: Raw records kept PER EPOCH (oldest dropped; they survive in the state backups).
MAX_POOLED_RECORDS = 200


def nominal_base_template(cs_mm: float = NOMINAL_CS_MM, cm_mm: float = NOMINAL_CM_MM,
                          ce_mm: float = NOMINAL_CE_MM) -> np.ndarray:
    """(4, 3) nominal C, M, E, S in the base frame (S on +y with z up)."""
    return np.array([[0.0, 0.0, 0.0], [cm_mm, 0.0, 0.0], [ce_mm, 0.0, 0.0],
                     [0.0, cs_mm, 0.0]])


def heading_deg(R: np.ndarray) -> float:
    """Heading (deg) of the frame's x axis projected on world xy."""
    return math.degrees(math.atan2(float(R[1, 0]), float(R[0, 0])))


def wrap_deg(a: float) -> float:
    return math.degrees(wrap_pi(math.radians(a)))


# ---------------------------------------------------------------------------
#  Identification and tracking
# ---------------------------------------------------------------------------

def base_candidate_points(unlabelled, labelled) -> np.ndarray:
    """(n, 3) points the base identifier searches in one frame: every
    unlabelled marker (rows x, y, z[, residual]) plus every labelled one
    (``(label, x, y, z, residual)``) except BB's own (BB_MARKER_BODIES).
    Labels play no other part: the base is found by geometry."""
    U = np.asarray(unlabelled if unlabelled is not None else np.empty((0, 3)), dtype=float)
    U = U.reshape(-1, U.shape[1] if U.ndim == 2 and U.shape[1] else 3)[:, :3]
    L = [(float(x), float(y), float(z)) for label, x, y, z, *_ in (labelled or ())
         if str(label).rsplit(' - ', 1)[0] not in BB_MARKER_BODIES]
    if not L:
        return U.copy()
    return np.vstack([U, np.array(L, dtype=float)])


@dataclass
class BaseFrameMatch:
    pairs: list                 # [(template_index, point_index), ...]
    R: np.ndarray               # base → world rotation
    t: np.ndarray               # world position of the base origin (C)
    rms_mm: float
    points: Optional[np.ndarray] = None   # the (finite) frame points the pair indices refer to


def _pose_from_pairs(T, P, pairs, tol_mm, cos_tilt):
    """Kabsch on ``pairs``, re-assign once with the 3D pose, refit. None if the
    posed normal is not up, fewer than BASE_MIN_MATCHED remain, or the matched
    markers are collinear (BASE_MIN_SPREAD_MM)."""
    for _ in range(2):
        ti = [p[0] for p in pairs]
        pi = [p[1] for p in pairs]
        R, t, rms = kabsch(T[ti], P[pi])
        if R[2, 2] < cos_tilt:
            return None
        new = _assign(T @ R.T + t, P, tol_mm)
        if len(new) < BASE_MIN_MATCHED or new == pairs:
            break
        pairs = new
    ti = [p[0] for p in pairs]
    pi = [p[1] for p in pairs]
    sv = np.linalg.svd(T[ti] - T[ti].mean(axis=0), compute_uv=False)
    if len(sv) < 2 or sv[1] < BASE_MIN_SPREAD_MM:
        return None
    R, t, rms = kabsch(T[ti], P[pi])
    if R[2, 2] < cos_tilt:
        return None
    return BaseFrameMatch(pairs=sorted(pairs), R=R, t=t, rms_mm=rms, points=P)


def identify_base_markers(points, template, tol_mm: float = BASE_MATCH_TOL_MM,
                          max_tilt_deg: float = BASE_MAX_TILT_DEG) -> Optional[BaseFrameMatch]:
    """Find the base markers among ``points`` by geometry alone.

    Every point triangle whose three distances match a template triangle
    within ``tol_mm`` proposes a pose (Kabsch, proper rotation); the posed
    plane normal must be within ``max_tilt_deg`` of world +z (this is what
    fixes the handedness of the planar L). The proposal matching the most
    markers, then the smallest RMS, wins. Extra scene markers only add
    candidates; a missing marker leaves three. None when no ≥ 3 match.
    """
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    P = P[np.all(np.isfinite(P), axis=1)]
    T = np.asarray(template, dtype=float).reshape(-1, 3)
    if len(P) < BASE_MIN_MATCHED:
        return None
    cos_tilt = math.cos(math.radians(max_tilt_deg))
    dP = np.linalg.norm(P[:, None] - P[None], axis=2)
    dT = np.linalg.norm(T[:, None] - T[None], axis=2)
    best = None
    n = len(P)
    for a, b, c in itertools.combinations(range(len(T)), 3):
        for i in range(n):
            for j in range(n):
                if j == i or abs(dP[i, j] - dT[a, b]) > tol_mm:
                    continue
                for k in range(n):
                    if k in (i, j) or abs(dP[i, k] - dT[a, c]) > tol_mm \
                            or abs(dP[j, k] - dT[b, c]) > tol_mm:
                        continue
                    m = _pose_from_pairs(T, P, [(a, i), (b, j), (c, k)], tol_mm, cos_tilt)
                    if m is None:
                        continue
                    key = (-len(m.pairs), m.rms_mm)
                    if best is None or key < best[0]:
                        best = (key, m)
    return None if best is None else best[1]


@dataclass
class BaseFrameTrack:
    """Per-frame rigid poses of the base template over a window (posed frames only)."""
    t: np.ndarray               # (n,) frame stamps, s
    R: np.ndarray               # (n, 3, 3) base → world
    origin: np.ndarray          # (n, 3) world position of C
    rms_mm: np.ndarray
    n_matched: np.ndarray
    points: np.ndarray          # (n, 4, 3) matched points in template order (nan: unseen)
    n_frames_in: int = 0
    n_rejected: int = 0

    @property
    def heading_deg(self) -> np.ndarray:
        return np.degrees(np.arctan2(self.R[:, 1, 0], self.R[:, 0, 0]))


class BaseFrameTracker:
    """Stateful frame-to-frame tracker: re-assigns from the previous pose, and
    falls back to :func:`identify_base_markers` after a loss or a gap."""

    def __init__(self, template, tol_mm: float = BASE_MATCH_TOL_MM,
                 max_rms_mm: float = BASE_FRAME_MAX_RMS_MM,
                 max_tilt_deg: float = BASE_MAX_TILT_DEG, max_gap_s: float = 1.0):
        self.T = np.asarray(template, dtype=float).reshape(-1, 3)
        self.tol_mm = tol_mm
        self.max_rms_mm = max_rms_mm
        self.max_tilt_deg = max_tilt_deg
        self.cos_tilt = math.cos(math.radians(max_tilt_deg))
        self.max_gap_s = max_gap_s
        self._prev = None       # (t, R, origin)

    def update(self, t: float, points) -> Optional[BaseFrameMatch]:
        P = np.asarray(points, dtype=float).reshape(-1, 3)
        P = P[np.all(np.isfinite(P), axis=1)]
        m = None
        if self._prev is not None and t - self._prev[0] <= self.max_gap_s and len(P) >= BASE_MIN_MATCHED:
            pairs = _assign(self.T @ self._prev[1].T + self._prev[2], P, self.tol_mm)
            if len(pairs) >= BASE_MIN_MATCHED:
                m = _pose_from_pairs(self.T, P, pairs, self.tol_mm, self.cos_tilt)
        if m is None:
            m = identify_base_markers(P, self.T, self.tol_mm, self.max_tilt_deg)
        if m is None or m.rms_mm > self.max_rms_mm:
            self._prev = None
            return None
        self._prev = (t, m.R, m.t)
        return m


def track_base_frame(frames, template, **kw) -> BaseFrameTrack:
    """Pose the base template in every frame. ``frames``: (t_s, (n, 3) points)
    — the frame's candidate markers (:func:`base_candidate_points`)."""
    T = np.asarray(template, dtype=float).reshape(-1, 3)
    K = len(T)
    tr = BaseFrameTracker(T, **kw)
    rows = sorted(((float(t), f) for t, f in frames), key=lambda r: r[0])
    ts, Rs, os_, rms, nm, pts = [], [], [], [], [], []
    rejected = 0
    for t, f in rows:
        m = tr.update(t, f)
        if m is None:
            rejected += 1
            continue
        Pm = np.full((K, 3), np.nan)
        for a, i in m.pairs:
            Pm[a] = m.points[i]
        ts.append(t); Rs.append(m.R); os_.append(m.t); rms.append(m.rms_mm)
        nm.append(len(m.pairs)); pts.append(Pm)
    return BaseFrameTrack(t=np.array(ts), R=np.array(Rs).reshape(-1, 3, 3),
                          origin=np.array(os_).reshape(-1, 3), rms_mm=np.array(rms),
                          n_matched=np.array(nm, dtype=int), points=np.array(pts).reshape(-1, K, 3),
                          n_frames_in=len(rows), n_rejected=rejected)


def canonical_base_frame(points4: np.ndarray):
    """(R, origin) of the canonical base frame of four points C, M, E, S:
    origin C, z = plane normal (SVD) pointing up, x = C→E with its normal
    component removed, y = z × x."""
    Q = np.asarray(points4, dtype=float).reshape(4, 3)
    _, _, vt = np.linalg.svd(Q - Q.mean(axis=0))
    nz = vt[2] * (1.0 if vt[2][2] >= 0 else -1.0)
    x = Q[2] - Q[0]
    x = x - nz * float(nz @ x)
    x /= np.linalg.norm(x)
    y = np.cross(nz, x)
    return np.c_[x, y, nz], Q[0].copy()


def learn_as_built(track: BaseFrameTrack, iterations: int = 3) -> np.ndarray:
    """As-built (4, 3) template in the canonical base frame: generalised
    Procrustes over the track's frames that see all four markers."""
    full = np.all(np.isfinite(track.points[:, :, 0]), axis=1)
    if full.sum() < 1:
        raise ValueError('BASE_FRAME_NOT_SEEN: no frame with all four base markers')
    P = track.points[full]
    R0, o0 = canonical_base_frame(P[0])
    T = (P[0] - o0) @ R0
    for _ in range(iterations):
        loc = []
        for Q in P:
            R, o, _ = kabsch(T, Q)
            loc.append((Q - o) @ R)
        T = np.mean(loc, axis=0)
        Rc, oc = canonical_base_frame(T)
        T = (T - oc) @ Rc
    return T


# ---------------------------------------------------------------------------
#  Static base pose
# ---------------------------------------------------------------------------

@dataclass
class BasePose:
    """The base frame's static pose over a window."""
    origin_mm: np.ndarray       # world position of C
    R: np.ndarray               # base → world (SVD-projected mean rotation)
    heading_deg: float
    tilt_deg: float             # plane normal from world +z
    heading_sd_deg: float       # per-frame scatter
    heading_se_deg: float       # SE of the mean (block means)
    origin_sd_mm: np.ndarray
    origin_se_mm: np.ndarray
    n_frames: int               # frames in the average
    n_frames_in: int
    n_rejected: int             # frames without a pose
    n_outliers: int
    visibility: tuple           # per marker, fraction of the input frames
    residual_mm: float          # largest |observed − template| mean pair distance
    t_span_s: tuple = (0.0, 0.0)

    def summary(self) -> str:
        vis = ' '.join(f'{n} {v * 100:.0f}%' for n, v in zip(BASE_MARKER_NAMES, self.visibility))
        return (f'base frame: {self.n_frames} frames ({self.n_outliers} outliers, '
                f'{self.n_rejected} unposed of {self.n_frames_in}), heading '
                f'{self.heading_deg:.4f}° ±{self.heading_se_deg:.4f}° (frame SD '
                f'{self.heading_sd_deg:.4f}°), origin ({self.origin_mm[0]:.2f}, '
                f'{self.origin_mm[1]:.2f}, {self.origin_mm[2]:.2f}) mm, tilt '
                f'{self.tilt_deg:.2f}°, residual {self.residual_mm:.2f} mm, visible {vis}')

    def to_dict(self) -> dict:
        return {'origin_mm': [float(v) for v in self.origin_mm],
                'R': [[float(v) for v in r] for r in self.R],
                'heading_deg': float(self.heading_deg), 'tilt_deg': float(self.tilt_deg),
                'heading_se_deg': float(self.heading_se_deg), 'n_frames': int(self.n_frames)}


def _block_se(t, v, block_s):
    """SE of mean(v) from block means (rows of v), ≥ 3 blocks; else SD/√n."""
    v = np.asarray(v, dtype=float)
    if len(v) < 2:
        return np.zeros(v.shape[1:]) if v.ndim > 1 else 0.0
    b = np.floor((np.asarray(t) - t[0]) / block_s).astype(int)
    keys = np.unique(b)
    if len(keys) >= 3:
        means = np.array([v[b == k].mean(axis=0) for k in keys])
        return means.std(axis=0, ddof=1) / math.sqrt(len(keys))
    return v.std(axis=0, ddof=1) / math.sqrt(len(v))


def mean_rotation(Rs: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(np.mean(Rs, axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def estimate_base_pose(frames, template, *, track: Optional[BaseFrameTrack] = None,
                       min_frames: int = BASE_MIN_FRAMES,
                       outlier_mm: float = BASE_OUTLIER_MM,
                       outlier_deg: float = BASE_OUTLIER_DEG,
                       block_s: float = BASE_SE_BLOCK_S) -> BasePose:
    """Static base pose over a window of candidate-marker frames
    (:func:`base_candidate_points`).

    Raises ValueError ``BASE_FRAME_NOT_SEEN`` when fewer than ``min_frames``
    frames pose the base (after outlier rejection)."""
    T = np.asarray(template, dtype=float).reshape(-1, 3)
    if track is None:
        track = track_base_frame(frames, T)
    n = len(track.t)
    if n < min_frames:
        raise ValueError(f'BASE_FRAME_NOT_SEEN: {n} of {track.n_frames_in} frames posed the '
                         f'base markers (need {min_frames})')
    h = track.heading_deg
    h_med = float(np.median(h))
    dh = np.array([wrap_deg(v - h_med) for v in h])
    o_med = np.median(track.origin, axis=0)
    keep = (np.abs(dh) <= outlier_deg) & (np.linalg.norm(track.origin - o_med, axis=1) <= outlier_mm)
    if keep.sum() < min_frames:
        raise ValueError(f'BASE_FRAME_NOT_SEEN: {int(keep.sum())} of {n} posed base frames '
                         f'agree with their median (need {min_frames})')
    R = mean_rotation(track.R[keep])
    o = track.origin[keep].mean(axis=0)
    hk = h_med + dh[keep]
    tk = track.t[keep]
    vis = tuple(float(np.isfinite(track.points[:, k, 0]).sum()) / max(track.n_frames_in, 1)
                for k in range(len(T)))
    # Template residual: mean pair distance over the window vs the template.
    dT = np.linalg.norm(T[:, None] - T[None], axis=2)
    Pk = track.points[keep]
    res = 0.0
    for a, b in itertools.combinations(range(len(T)), 2):
        d = np.linalg.norm(Pk[:, a] - Pk[:, b], axis=1)
        d = d[np.isfinite(d)]
        if len(d):
            res = max(res, abs(float(d.mean()) - dT[a, b]))
    return BasePose(
        origin_mm=o, R=R, heading_deg=heading_deg(R),
        tilt_deg=math.degrees(math.acos(min(1.0, float(R[2, 2])))),
        heading_sd_deg=float(hk.std()), heading_se_deg=float(_block_se(tk, hk, block_s)),
        origin_sd_mm=track.origin[keep].std(axis=0),
        origin_se_mm=np.asarray(_block_se(tk, track.origin[keep], block_s)),
        n_frames=int(keep.sum()), n_frames_in=track.n_frames_in, n_rejected=track.n_rejected,
        n_outliers=int(n - keep.sum()), visibility=vis, residual_mm=res,
        t_span_s=(float(track.t[0]), float(track.t[-1])))


# ---------------------------------------------------------------------------
#  BB-in-base constants
# ---------------------------------------------------------------------------

def bb_in_base(base: BasePose, yaw_offset_deg: float, position_mm):
    """(κ deg, p_b (3,) mm): a world BB pose expressed in the base frame."""
    kappa = wrap_deg(float(yaw_offset_deg) - base.heading_deg)
    p_b = base.R.T @ (np.asarray(position_mm, dtype=float) - base.origin_mm)
    return kappa, p_b


def compose_bb_pose(base: BasePose, kappa_deg: float, p_b_mm):
    """(yaw offset deg, position (3,) mm) in the world: the inverse of :func:`bb_in_base`."""
    return (wrap_deg(base.heading_deg + float(kappa_deg)),
            base.origin_mm + base.R @ np.asarray(p_b_mm, dtype=float))


@dataclass
class KappaRecord:
    """One accepted sweep, in the base frame."""
    kappa_deg: float
    sigma_deg: float            # the sweep's published σ
    axis_point_b_mm: tuple
    pin_deg: float              # template gauge.pinned_yaw_offset_deg at the time
    accepted_at: str = ''
    yaw_offset_deg: float = float('nan')     # the sweep's world value (published gauge)
    base_heading_deg: float = float('nan')
    source: str = ''
    epoch: int = 0              # kappa epoch id (state schema 2; not in the resource)

    def to_dict(self) -> dict:
        return {'kappa_deg': self.kappa_deg, 'sigma_deg': self.sigma_deg,
                'axis_point_b_mm': [float(v) for v in self.axis_point_b_mm],
                'pin_deg': self.pin_deg, 'accepted_at': self.accepted_at,
                'yaw_offset_deg': self.yaw_offset_deg,
                'base_heading_deg': self.base_heading_deg, 'source': self.source}

    @classmethod
    def from_dict(cls, d: dict) -> 'KappaRecord':
        return cls(kappa_deg=float(d['kappa_deg']), sigma_deg=float(d['sigma_deg']),
                   axis_point_b_mm=tuple(float(v) for v in d['axis_point_b_mm']),
                   pin_deg=float(d['pin_deg']), accepted_at=str(d.get('accepted_at', '')),
                   yaw_offset_deg=float(d.get('yaw_offset_deg', float('nan'))),
                   base_heading_deg=float(d.get('base_heading_deg', float('nan'))),
                   source=str(d.get('source', '')), epoch=int(d.get('epoch', 0)))


@dataclass
class PooledKappa:
    kappa_deg: float            # in the CURRENT pin; mean of the kept records
    sd_deg: float               # sweep-to-sweep SD of the kept records (0 for n = 1)
    se_deg: float               # SE of the mean
    n: int                      # records kept (outliers excluded)
    axis_point_b_mm: np.ndarray
    axis_point_sd_mm: np.ndarray
    n_total: int = 0            # records in the pool, outliers included
    outliers: tuple = ()        # indices (into the records given) of the excluded ones

    @property
    def n_outliers(self) -> int:
        return len(self.outliers)

    def count_note(self) -> str:
        """'n 20' or 'n 20 of 22, 2 outliers excluded' — the pool on one line."""
        if not self.outliers:
            return f'n {self.n}'
        return f'n {self.n} of {self.n_total}, {self.n_outliers} outliers excluded'


def kappa_record_yaw_source(rec: KappaRecord) -> str:
    """'stamped' or 'heartbeat': the yaw source a record was measured with
    (the node writes ``mocap_node (<source>)``; the 12:26 seed records are the
    live heartbeat values)."""
    return 'stamped' if 'stamped' in rec.source else 'heartbeat'


def kappa_sweep_sd_deg(yaw_source: str) -> float:
    """Per-sweep κ SD (deg) of a yaw source; the larger for an unknown one."""
    return KAPPA_SWEEP_SD_DEG.get(yaw_source, max(KAPPA_SWEEP_SD_DEG.values()))


def kappa_outliers(kappas, sources, clip_n: float = KAPPA_POOL_CLIP_N,
                   min_group: int = KAPPA_POOL_MIN_GROUP) -> np.ndarray:
    """Boolean mask of the records more than ``clip_n``·σ_g from the median κ
    (``kappas`` unwrapped, in one pin). σ_g per yaw-source group: the group's
    1.4826·MAD about the common median, floored at KAPPA_POOL_SIGMA_FLOOR_DEG
    (a group of ≥ ``min_group``), else the larger of that floor and the
    source's per-sweep SD. Never excludes every record."""
    ks = np.asarray(kappas, dtype=float)
    src = np.asarray(list(sources))
    out = np.zeros(len(ks), dtype=bool)
    if len(ks) < 2:
        return out
    med = float(np.median(ks))
    dev = np.abs(ks - med)
    for g in sorted(set(src.tolist())):
        sel = src == g
        floor = KAPPA_POOL_SIGMA_FLOOR_DEG.get(g, max(KAPPA_POOL_SIGMA_FLOOR_DEG.values()))
        if sel.sum() >= min_group:
            sig = max(1.4826 * float(np.median(dev[sel])), floor)
        else:
            sig = max(floor, kappa_sweep_sd_deg(g))
        out[sel] = dev[sel] > clip_n * sig
    return np.zeros(len(ks), dtype=bool) if out.all() else out


def pool_kappa(records, pin_now_deg: float, repeatability_deg: float = 0.0) -> Optional[PooledKappa]:
    """Pool κ records (each moved to the current pin), robustly: the records
    :func:`kappa_outliers` flags are excluded (and listed in ``outliers``);
    κ and p_b are the means of the rest. SE = max(SD, the kept records' RMS σ
    while n < 5, ``repeatability_deg``)/√n — a small pool's sample SD is itself
    uncertain, so it never claims better than the per-sweep σ allows. None for
    no records."""
    recs = list(records)
    if not recs:
        return None
    k0 = recs[0].kappa_deg + pin_now_deg - recs[0].pin_deg
    ks_all = np.array([k0 + wrap_deg(r.kappa_deg + pin_now_deg - r.pin_deg - k0) for r in recs])
    bad = kappa_outliers(ks_all, [kappa_record_yaw_source(r) for r in recs])
    keep = ~bad
    ks = ks_all[keep]
    pb = np.array([r.axis_point_b_mm for r in recs], dtype=float)[keep]
    n = len(ks)
    sd = float(ks.std(ddof=1)) if n > 1 else 0.0
    sig = float(np.sqrt(np.mean([r.sigma_deg ** 2 for r, k in zip(recs, keep) if k])))
    floor = max(sd, sig if n < 5 else 0.0, float(repeatability_deg))
    return PooledKappa(kappa_deg=wrap_deg(float(ks.mean())), sd_deg=sd,
                       se_deg=floor / math.sqrt(n), n=n,
                       axis_point_b_mm=pb.mean(axis=0),
                       axis_point_sd_mm=pb.std(axis=0, ddof=1) if n > 1 else np.zeros(3),
                       n_total=len(recs), outliers=tuple(int(i) for i in np.flatnonzero(bad)))


# ---------------------------------------------------------------------------
#  Model (resource) and runtime state
# ---------------------------------------------------------------------------

@dataclass
class BaseFrameModel:
    """``resources/bb_base_frame.json`` (schema 1): the as-built base template
    and the seed BB-in-base records."""
    template_mm: np.ndarray
    nominal_mm: np.ndarray
    records: list = field(default_factory=list)
    pin_uncertainty_deg: Optional[float] = None
    source: str = ''
    #: The base pose at the build sitting ({'origin_mm', 'heading_deg'}): the
    #: first QTM-frame-shift reference before the node has stored one.
    base_ref: Optional[dict] = None


def load_base_frame(path: str) -> BaseFrameModel:
    """Load the base-frame resource. Raises ValueError if malformed."""
    with open(path, 'r') as f:
        data = json.load(f)
    if data.get('schema') != 1:
        raise ValueError(f'BB base frame {path}: schema {data.get("schema")!r}, need 1')
    try:
        names = [m['id'] for m in data['as_built']['markers']]
        T = np.array([m['xyz_mm'] for m in data['as_built']['markers']], dtype=float)
        nom = data['nominal']
        N = nominal_base_template(float(nom['cs_mm']), float(nom['cm_mm']), float(nom['ce_mm']))
        recs = [KappaRecord.from_dict(r) for r in data['bb_in_base'].get('records', [])]
        pin_sd = data.get('gauge', {}).get('pin_uncertainty_deg')
        ref = data.get('provenance', {}).get('base_pose_at_build')
        if ref is not None:
            ref = {'origin_mm': [float(v) for v in ref['origin_mm']],
                   'heading_deg': float(ref['heading_deg']), 'accepted_at': 'build'}
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f'malformed BB base frame {path}: {e!r}')
    if tuple(names) != BASE_MARKER_NAMES or T.shape != (4, 3) or not np.all(np.isfinite(T)):
        raise ValueError(f'BB base frame {path}: need markers {BASE_MARKER_NAMES} with xyz')
    return BaseFrameModel(template_mm=T, nominal_mm=N, records=recs,
                          pin_uncertainty_deg=None if pin_sd is None else float(pin_sd),
                          source=path, base_ref=ref)


def _corrected_kappas(recs, pin_now_deg: float):
    """Each record's kappa moved to ``pin_now_deg`` (kappa + pin_now - pin_rec, the
    :func:`pool_kappa` convention), unwrapped about the first."""
    recs = list(recs)
    k0 = recs[0].kappa_deg + pin_now_deg - recs[0].pin_deg
    return np.array([k0 + wrap_deg(r.kappa_deg + pin_now_deg - r.pin_deg - k0) for r in recs])


def _epoch_summary(recs, pin_now_deg: Optional[float] = None) -> dict:
    """{n, median_kappa_deg, sd_deg, pin_deg, median_axis_point_b_mm} of
    KappaRecords. ``median_kappa_deg`` is stated AT ``pin_deg`` (= ``pin_now_deg``,
    else the newest record's pin): each record's kappa is first moved to that pin,
    because kappas measured under different template gauges are not comparable."""
    recs = list(recs)
    pin = float(recs[-1].pin_deg if pin_now_deg is None else pin_now_deg)
    ks = _corrected_kappas(recs, pin)
    pb = np.array([r.axis_point_b_mm for r in recs], dtype=float)
    return {'n': len(recs), 'median_kappa_deg': wrap_deg(float(np.median(ks))),
            'sd_deg': float(ks.std(ddof=1)) if len(ks) > 1 else 0.0,
            'pin_deg': pin,
            'median_axis_point_b_mm': [float(v) for v in np.median(pb, axis=0)]}


def _new_epoch(epoch_id: int, opened_at: str, reason: str, recs,
               pin_now_deg: Optional[float] = None) -> dict:
    d = {'id': int(epoch_id), 'opened_at': opened_at, 'reason': reason}
    d.update(_epoch_summary(recs, pin_now_deg))
    return d


def seed_base_state(records, base_ref) -> dict:
    """A fresh state: every record in epoch 0 (reason 'seed'), e.g. from the
    resource's records. No records: no epochs (the first sweep opens epoch 0)."""
    recs = [dataclasses.replace(r, epoch=0) for r in records]
    epochs = [_new_epoch(0, recs[0].accepted_at, 'seed', recs)] if recs else []
    return {'records': recs, 'base_ref': base_ref, 'epochs': epochs}


def load_base_state(path: str):
    """Runtime state ``{'records': [KappaRecord], 'base_ref': {...} | None,
    'epochs': [dict]}`` or None if the file does not exist (the caller seeds it,
    :func:`seed_base_state`). Reads schema 1 (every record -> epoch 0, reason
    'migrated') and schema 2. Raises (OSError, ValueError, KeyError, TypeError)
    if unreadable - the caller decides."""
    try:
        with open(path, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        return None
    recs = [KappaRecord.from_dict(r) for r in data['records']]
    ref = data.get('base_ref')
    if ref is not None:
        ref = {'origin_mm': [float(v) for v in ref['origin_mm']],
               'heading_deg': float(ref['heading_deg']),
               'accepted_at': str(ref.get('accepted_at', ''))}
    schema = data.get('schema')
    if schema == 1:
        recs = [dataclasses.replace(r, epoch=0) for r in recs]
        epochs = [_new_epoch(0, recs[0].accepted_at, 'migrated', recs)] if recs else []
    elif schema == 2:
        epochs = []
        for e in data['epochs']:
            epochs.append({'id': int(e['id']), 'opened_at': str(e.get('opened_at', '')),
                           'reason': str(e.get('reason', '')), 'n': int(e['n']),
                           'median_kappa_deg': float(e['median_kappa_deg']),
                           'sd_deg': float(e['sd_deg']),
                           **({'pin_deg': float(e['pin_deg'])} if 'pin_deg' in e else {}),
                           'median_axis_point_b_mm': [float(v) for v in e['median_axis_point_b_mm']]})
        ids = {e['id'] for e in epochs}
        if any(r.epoch not in ids for r in recs):
            raise ValueError('BB base state: a record names an epoch with no summary')
    else:
        raise ValueError(f'BB base state {path}: schema {schema!r}, need 1 or 2')
    return {'records': recs, 'base_ref': ref, 'epochs': epochs}


def pool_summary(records, pooled: Optional[PooledKappa]) -> Optional[dict]:
    """The pool as written next to its records (derived, for a reader: the
    node and the build tool recompute it): κ, SD, SE, counts, and which records
    the robust pool excludes."""
    if pooled is None:
        return None
    recs = list(records)
    return {'kappa_deg': round(float(pooled.kappa_deg), 5), 'sd_deg': round(pooled.sd_deg, 5),
            'se_deg': round(pooled.se_deg, 5), 'n_kept': pooled.n, 'n_total': pooled.n_total,
            'axis_point_mm': [round(float(v), 4) for v in pooled.axis_point_b_mm],
            'outliers': [{'index': i, 'accepted_at': recs[i].accepted_at,
                          'kappa_deg': recs[i].kappa_deg, 'source': recs[i].source}
                         for i in pooled.outliers],
            'rule': (f'excluded: |κ − median| > {KAPPA_POOL_CLIP_N:g}·σ_g, σ_g = max(1.4826·MAD '
                     f'of the yaw-source group, floor {KAPPA_POOL_SIGMA_FLOOR_DEG}) for a group '
                     f'of ≥ {KAPPA_POOL_MIN_GROUP}, else max(floor, per-sweep SD '
                     f'{KAPPA_SWEEP_SD_DEG}) — bb_base_frame.kappa_outliers')}


def current_epoch_id(state) -> Optional[int]:
    """The epoch of the newest record (None for a state with no records)."""
    return state['records'][-1].epoch if state['records'] else None


def epoch_median_at_pin(state, epoch: dict, pin_now_deg: float) -> float:
    """An epoch's median kappa at the CURRENT pin. From its held raw records, each
    moved to ``pin_now_deg``; with none held (per-epoch cap) its stored summary
    moved by pin_now - summary pin. A summary with no ``pin_deg`` (written before
    the key existed) is taken at the pin of the epoch's newest record, else at
    ``pin_now_deg`` (no correction)."""
    held = [r for r in state['records'] if r.epoch == epoch['id']]
    if held:
        return wrap_deg(float(np.median(_corrected_kappas(held, pin_now_deg))))
    pin_rec = epoch.get('pin_deg')
    if pin_rec is None:
        pin_rec = pin_now_deg
    return wrap_deg(epoch['median_kappa_deg'] + pin_now_deg - float(pin_rec))


def _epoch_by_id(state, epoch_id) -> Optional[dict]:
    for e in state['epochs']:
        if e['id'] == epoch_id:
            return e
    return None


def _restate_at_pin(state, epoch: dict, pin_now_deg: float) -> dict:
    """The epoch summary with its median stated at ``pin_now_deg`` (``pin_deg`` set)."""
    return dict(epoch, median_kappa_deg=epoch_median_at_pin(state, epoch, pin_now_deg),
                pin_deg=float(pin_now_deg))


def base_state_to_json(state, pin_now_deg: Optional[float] = None,
                       repeatability_deg: float = 0.0) -> dict:
    """The state file, schema 2: every epoch summary (never removed), the raw
    records (each tagged with its epoch; at most MAX_POOLED_RECORDS per epoch,
    kept whole - outliers included), base_ref, and - given the current pin - the
    derived ``pool`` of the CURRENT epoch (:func:`pool_summary`; ignored on read).
    Given the current pin, every epoch summary's median is written AT that pin
    (its ``pin_deg``); without it the summaries are written as held."""
    recs = list(state['records'])
    out = {'schema': 2,
           'description': 'mocap_node runtime state for the BB base-marker frame: kappa epochs '
                          '(one summary each, never removed), the BB-in-base records (kappa per '
                          'sweep, each tagged with its epoch) and the last base pose in the '
                          'world. Seeded from resources/bb_base_frame.json; '
                          'tools/bb_base_frame_build.py --from-state folds it back. '
                          '"pool" is derived (the robust pool of the current epoch).',
           'epochs': [dict(e) for e in state['epochs']],
           'records': [dict(r.to_dict(), epoch=r.epoch) for r in recs],
           'base_ref': state['base_ref']}
    cur = current_epoch_id(state)
    if pin_now_deg is not None:
        out['epochs'] = [_restate_at_pin(state, e, pin_now_deg) for e in state['epochs']]
    if pin_now_deg is not None and cur is not None:
        cur_recs = [r for r in recs if r.epoch == cur]
        out['pool'] = pool_summary(cur_recs, pool_kappa(cur_recs, pin_now_deg, repeatability_deg))
    return out


#: Timestamped copies of the state file kept next to it (newest N).
STATE_BACKUPS_KEPT = 20
_BAK_RE = re.compile(r'\.(\d{8}-\d{6})(?:\.(\d+))?\.bak$')


def _prune_backups(path: str, keep: int = STATE_BACKUPS_KEPT) -> None:
    base = os.path.basename(path)
    found = []
    for name in os.listdir(os.path.dirname(os.path.abspath(path))):
        if name.startswith(base + '.'):
            m = _BAK_RE.fullmatch(name[len(base):])
            if m:
                found.append(((m.group(1), int(m.group(2) or 0)), name))
    found.sort()
    for _, name in found[:-keep] if keep > 0 else found:
        try:
            os.remove(os.path.join(os.path.dirname(os.path.abspath(path)), name))
        except OSError:
            pass


def write_base_state(path: str, state, pin_now_deg: Optional[float] = None,
                     repeatability_deg: float = 0.0) -> None:
    """THE ONLY writer of the state file. The existing file is first copied to
    ``<path>.<YYYYmmdd-HHMMSS>.bak`` (newest STATE_BACKUPS_KEPT kept), then the new
    one is written to a .tmp and os.replace'd. Nothing here removes a record or an
    epoch beyond the per-epoch MAX_POOLED_RECORDS cap, and the dropped ones
    survive in the backups."""
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    if os.path.exists(path):
        stamp = time.strftime('%Y%m%d-%H%M%S')
        # Same-second writes: number above the highest existing one (pruning
        # deletes the low ones, so "first free" would reuse a name and break order).
        base = os.path.basename(path)
        seqs = []
        for name in os.listdir(d):
            m = _BAK_RE.fullmatch(name[len(base):]) if name.startswith(base + '.') else None
            if m and m.group(1) == stamp:
                seqs.append(int(m.group(2) or 0) + 1)
        bak = f'{path}.{stamp}.bak' if not seqs else f'{path}.{stamp}.{max(seqs)}.bak'
        shutil.copy2(path, bak)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(base_state_to_json(state, pin_now_deg, repeatability_deg), f, indent=1)
    os.replace(tmp, path)
    _prune_backups(path)


# ---------------------------------------------------------------------------
#  Kappa epochs and the advisory (nothing here refuses a sweep)
# ---------------------------------------------------------------------------

@dataclass
class EpochAssignment:
    epoch_id: int
    opened: bool                      # a new epoch was opened for this sweep
    rejoined: bool                    # an earlier (non-current) epoch was rejoined
    delta_vs_current_deg: float       # kappa - the CURRENT epoch's median, before assignment
    current_epoch_id_before: Optional[int]
    median_before: Optional[float]    # that median (None with no epochs)
    reason: str = 'excursion'         # of a newly opened epoch ('seed' for the very first)


def assign_epoch(kappa_deg: float, state, pin_now_deg: float,
                 band_deg: float = KAPPA_EPOCH_BAND_DEG) -> EpochAssignment:
    """Which kappa epoch a sweep belongs to. Within ``band_deg`` of the CURRENT
    epoch's median -> that epoch. Else the nearest OTHER epoch within the band ->
    rejoin it (it becomes current, being newest). Else a new epoch (excursion).
    Flip-flop: 179.27 (K0), 179.59 opens K1, 179.27 again rejoins K0, not K2.
    The comparison runs on every sweep (a one-record epoch's median is that
    record). ``kappa_deg`` is at ``pin_now_deg``; every epoch median is moved to
    that pin first (:func:`epoch_median_at_pin`), so a re-pin from landings does
    not look like a kappa excursion. Pure: the state is not changed
    (:func:`append_record` does that)."""
    cur = current_epoch_id(state)
    cur_e = _epoch_by_id(state, cur) if cur is not None else None
    if cur_e is None:
        nxt = 1 + max((e['id'] for e in state['epochs']), default=-1)
        return EpochAssignment(nxt, True, False, 0.0, None, None, reason='seed')
    med = epoch_median_at_pin(state, cur_e, pin_now_deg)
    d = wrap_deg(kappa_deg - med)
    if abs(d) <= band_deg:
        return EpochAssignment(cur, False, False, d, cur, med)
    others = [(abs(wrap_deg(kappa_deg - epoch_median_at_pin(state, e, pin_now_deg))), e['id'])
              for e in state['epochs'] if e['id'] != cur]
    others = [o for o in others if o[0] <= band_deg]
    if others:
        return EpochAssignment(min(others)[1], False, True, d, cur, med)
    nxt = 1 + max(e['id'] for e in state['epochs'])
    return EpochAssignment(nxt, True, False, d, cur, med)


def append_record(state, record: KappaRecord, assignment: EpochAssignment,
                  pin_now_deg: float) -> dict:
    """The only growth path: a NEW state with ``record`` appended in the assigned
    epoch (opened if the assignment says so), that epoch's summary refreshed, and
    the epoch's raw records capped at MAX_POOLED_RECORDS (oldest dropped; they
    survive in the backups). Epoch summaries are never removed; the refreshed one
    is stated at ``pin_now_deg``."""
    rec = dataclasses.replace(record, epoch=assignment.epoch_id)
    recs = list(state['records']) + [rec]
    epochs = [dict(e) for e in state['epochs']]
    eid = assignment.epoch_id
    prior_n = next((e['n'] for e in epochs if e['id'] == eid), 0)
    mine = [r for r in recs if r.epoch == eid]
    if len(mine) > MAX_POOLED_RECORDS:
        drop = mine[0]
        recs = [r for r in recs if r is not drop]
        mine = mine[1:]
    summ = _epoch_summary(mine, pin_now_deg)
    summ['n'] = prior_n + 1                     # sweeps ever assigned, not just held
    for i, e in enumerate(epochs):
        if e['id'] == eid:
            epochs[i] = dict(e, **summ)
            break
    else:
        epochs.append(_new_epoch(eid, rec.accepted_at, assignment.reason, mine, pin_now_deg))
        epochs[-1]['n'] = 1
    return {'records': recs, 'base_ref': state['base_ref'], 'epochs': epochs}


@dataclass
class BaseAdvisory:
    pooled: bool                              # False: BASE_FRAME_RESIDUAL, nothing pooled/judged
    assignment: Optional[EpochAssignment]
    delta_kappa_deg: Optional[float]          # vs the current epoch median, before assignment
    delta_axis_mm: Optional[float]            # p_b vs the assigned (else previous current) epoch
    base_shift_mm: Optional[float]            # base origin vs base_ref (None without one)
    base_shift_deg: Optional[float]
    same_world_epoch: Optional[bool]          # not shifted; None without base_ref
    flags: list = field(default_factory=list)
    lines: list = field(default_factory=list)


def assess_base_frame(kappa_deg: float, sigma_deg: float, axis_point_b_mm, base: BasePose,
                      state, pin_now_deg: float) -> BaseAdvisory:
    """Advisory for a sweep in the base frame. NEVER refuses; pure (the state is
    not mutated - the caller then does :func:`append_record` +
    :func:`write_base_state`). ``kappa_deg`` is at ``pin_now_deg`` (the template
    gauge current for this sweep); epochs are compared at that pin. ``sigma_deg`` is the sweep's per-sweep kappa scatter
    (informational; the bands are fixed numbers).

    Flags: BASE_FRAME_RESIDUAL (the shelf-marker fit is poor: kappa not pooled, not
    judged), KAPPA_EXCURSION (|delta kappa| > KAPPA_EPOCH_BAND_DEG vs the current
    epoch: the line says which epoch was opened or rejoined), BB_IN_BASE_SHIFT
    (|delta p_b| > BB_IN_BASE_SHIFT_ADVISORY_MM), QTM_FRAME_SHIFT (base moved
    beyond WORLD_EPOCH_SHIFT_MM / _DEG vs base_ref)."""
    flags: list = []
    lines: list = []
    shift_mm = shift_deg = same = None
    ref = state.get('base_ref')
    if ref is not None:
        shift_mm = float(np.linalg.norm(base.origin_mm - np.asarray(ref['origin_mm'], float)))
        shift_deg = wrap_deg(base.heading_deg - float(ref['heading_deg']))
        same = not (shift_mm > WORLD_EPOCH_SHIFT_MM or abs(shift_deg) > WORLD_EPOCH_SHIFT_DEG)
        if not same:
            flags.append('QTM_FRAME_SHIFT')
            lines.append(f'QTM frame moved since the last sweep ({shift_mm:.1f} mm, '
                         f'{shift_deg:+.2f} deg); world deltas are not comparable')
    if base.residual_mm > BASE_MAX_RESIDUAL_MM:
        flags.append('BASE_FRAME_RESIDUAL')
        lines.insert(0, f'shelf markers fit {base.residual_mm:.2f} mm (> {BASE_MAX_RESIDUAL_MM:.1f}), '
                        'base kappa not used - check QTM sees the 4 shelf markers')
        return BaseAdvisory(False, None, None, None, shift_mm, shift_deg, same, flags, lines)
    asg = assign_epoch(kappa_deg, state, pin_now_deg)
    dk = asg.delta_vs_current_deg if asg.median_before is not None else None
    ref_e = _epoch_by_id(state, asg.epoch_id) or (
        _epoch_by_id(state, asg.current_epoch_id_before)
        if asg.current_epoch_id_before is not None else None)
    da = None
    if ref_e is not None:
        da = float(np.linalg.norm(np.asarray(axis_point_b_mm, float)
                                  - np.asarray(ref_e['median_axis_point_b_mm'], float)))
    if asg.opened and asg.median_before is not None:
        flags.append('KAPPA_EXCURSION')
        lines.append(f'kappa {dk:+.2f} deg from epoch K{asg.current_epoch_id_before}: '
                     f'opened epoch K{asg.epoch_id}')
    elif asg.rejoined:
        flags.append('KAPPA_EXCURSION')
        lines.append(f'kappa {dk:+.2f} deg from epoch K{asg.current_epoch_id_before}: '
                     f'rejoined epoch K{asg.epoch_id}')
    if da is not None and da > BB_IN_BASE_SHIFT_ADVISORY_MM:
        flags.append('BB_IN_BASE_SHIFT')
        lines.append(f'BB axis point moved {da:.1f} mm in the base frame '
                     f'(advisory > {BB_IN_BASE_SHIFT_ADVISORY_MM:.1f})')
    return BaseAdvisory(True, asg, dk, da, shift_mm, shift_deg, same, flags, lines)


# ---------------------------------------------------------------------------
#  Slow continuous estimate
# ---------------------------------------------------------------------------

class BaseFrameMonitor:
    """A slow running base pose for the node: ``update`` a few times a second
    with a frame's candidate markers (:func:`base_candidate_points`); ``pose``
    is the static pose over the poses of the last ``window_s`` (None if fewer
    than ``min_frames``)."""

    def __init__(self, template, window_s: float = 30.0, min_frames: int = 20):
        self.T = np.asarray(template, dtype=float).reshape(-1, 3)
        self.window_s = float(window_s)
        self.min_frames = int(min_frames)
        self._tracker = BaseFrameTracker(self.T, max_gap_s=2.0)
        self._buf = deque()      # (t, R, origin, (4,3) points)
        self._seen = 0
        self._frames = 0

    def update(self, t: float, points) -> bool:
        self._frames += 1
        m = self._tracker.update(t, points)
        while self._buf and self._buf[0][0] < t - self.window_s:
            self._buf.popleft()
        if m is None:
            return False
        Pm = np.full((len(self.T), 3), np.nan)
        for a, i in m.pairs:
            Pm[a] = m.points[i]
        self._buf.append((float(t), m.R, m.t, Pm))
        self._seen += 1
        return True

    def pose(self, now: Optional[float] = None) -> Optional[BasePose]:
        rows = [r for r in self._buf if now is None or r[0] >= now - self.window_s]
        if len(rows) < self.min_frames:
            return None
        track = BaseFrameTrack(t=np.array([r[0] for r in rows]), R=np.array([r[1] for r in rows]),
                               origin=np.array([r[2] for r in rows]), rms_mm=np.zeros(len(rows)),
                               n_matched=np.array([int(np.isfinite(r[3][:, 0]).sum()) for r in rows]),
                               points=np.array([r[3] for r in rows]), n_frames_in=len(rows))
        try:
            return estimate_base_pose(None, self.T, track=track, min_frames=self.min_frames)
        except ValueError:
            return None
