"""Ball Butler base-marker frame: BB's pose relative to four fixed markers on its shelf.

Why. QTM's world frame moves between sittings (0.3–1.5 mm, non-rigidly) and is
occasionally recalibrated, while BB itself is bolted to a shelf and does not
move. The sweep estimator (:mod:`jugglebot.bb_calibration`) measures BB in the
WORLD frame, so every QTM shift reads as a BB move: the consistency gate
refuses it, or (with ``bb_moved``) accepts it with no way to tell "QTM moved"
from "BB moved". Four markers fastened to the shelf (the base frame) are
static in BB's frame; posing them every sitting separates the two:

    BB-in-base   (κ, p_b)   constant while BB and the base stay on the shelf
    base-in-world (h, o, R)  changes when QTM's frame changes

and the published BB world pose is their composition (:func:`compose_bb_pose`).

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

import itertools
import json
import math
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
#: Base template residual gate (mm): the largest |observed − as-built| pair
#: distance (window mean). Above it a base marker moved or the frame was
#: knocked: rebuild the base frame (bb_moved does not override).
#: 2026-10-10: ≤ 0.035 mm SD per pair, mean 0 by construction.
BASE_MAX_RESIDUAL_MM = 0.5
#: Base-in-world change vs the stored base pose above which it is reported as
#: a QTM frame shift (it is accepted either way).
BASE_SHIFT_REPORT_MM = 0.2
BASE_SHIFT_REPORT_DEG = 0.01
#: BB-in-base gate: |Δκ| > max(N·√(σ_src² + SE_pool²), MIN) or |Δp_b| > MAX,
#: Δκ against the ROBUST pooled κ (:func:`pool_kappa`). σ_src is the measured
#: per-sweep κ scatter of the new sweep's yaw source (KAPPA_SWEEP_SD_DEG), not
#: the sweep's published σ (which over-states it: 0.036–0.045° stamped,
#: 0.07–0.09° heartbeat). MIN 0.09° = 3 × the stamped per-sweep SD; it was
#: 0.15° (the world gate's floor) until 2026-10-10, which let two sweeps
#: +0.07–0.09° off into the pool (logbook 2026-10-10-bb-base-frame-robustness).
KAPPA_GATE_N_SIGMA = 3.0
KAPPA_GATE_MIN_DEG = 0.09
BB_IN_BASE_MAX_SHIFT_MM = 1.5
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
#: Pooled sweeps the ``auto`` pose source needs before it uses the base frame.
DEFAULT_MIN_POOLED_SWEEPS = 5
#: Records kept in the pool (oldest dropped): ~a week of sittings.
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
                   source=str(d.get('source', '')))


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


def load_base_state(path: str):
    """Runtime state ``{'records': [...], 'base_ref': {...}}`` or None if the
    file does not exist. Raises (OSError, ValueError, KeyError, TypeError) if
    unreadable — the caller decides."""
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
    return {'records': recs, 'base_ref': ref}


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


def base_state_to_json(records, base_ref, pin_now_deg: Optional[float] = None,
                       repeatability_deg: float = 0.0) -> dict:
    """The state file: the newest MAX_POOLED_RECORDS records, kept whole
    (outliers included: the exclusion is recomputed at every pooling), and —
    given the current pin — the derived pool summary (:func:`pool_summary`)."""
    recs = list(records)[-MAX_POOLED_RECORDS:]
    out = {'schema': 1,
           'description': 'mocap_node runtime state for the BB base-marker frame: the pooled '
                          'BB-in-base records (κ per accepted sweep) and the last base pose in '
                          'the world. Seeded from resources/bb_base_frame.json; '
                          'tools/bb_base_frame_build.py --from-state folds it back. '
                          '"pool" is derived (the robust pool of these records).',
           'records': [r.to_dict() for r in recs],
           'base_ref': base_ref}
    if pin_now_deg is not None:
        out['pool'] = pool_summary(recs, pool_kappa(recs, pin_now_deg, repeatability_deg))
    return out


# ---------------------------------------------------------------------------
#  Gate
# ---------------------------------------------------------------------------

@dataclass
class BaseGateVerdict:
    accepted: bool
    message: str
    delta_kappa_deg: float = 0.0
    threshold_deg: float = 0.0
    axis_shift_mm: float = 0.0
    base_shift_mm: float = 0.0          # base-in-world origin change vs base_ref
    base_shift_deg: float = 0.0         # base heading change vs base_ref
    qtm_shift: bool = False             # base-in-world changed (reported, accepted)
    reset_pool: bool = False            # bb_moved: this sweep starts a new pool
    overridden: bool = False
    no_reference: bool = False


def check_base_frame_consistency(
    kappa_deg: float, sigma_deg: float, axis_point_b_mm, base: BasePose,
    pooled: Optional[PooledKappa], base_ref: Optional[dict], bb_moved: bool = False,
    n_sigma: float = KAPPA_GATE_N_SIGMA, min_deg: float = KAPPA_GATE_MIN_DEG,
    max_axis_shift_mm: float = BB_IN_BASE_MAX_SHIFT_MM,
    max_residual_mm: float = BASE_MAX_RESIDUAL_MM,
) -> BaseGateVerdict:
    """Gate a sweep in the base frame. ``sigma_deg`` is the new sweep's
    per-sweep κ scatter (:func:`kappa_sweep_sd_deg` of its yaw source).

    BB-in-base (invariant to QTM frame shifts) is compared with the pool: a
    change means BB or the base frame moved ON THE SHELF — refused
    (``BB_IN_BASE_MOVED``) unless ``bb_moved``, which starts a new pool.
    Base-in-world is compared with ``base_ref``: a change means QTM's frame
    moved — reported as the frame shift and accepted. A base template residual
    above ``max_residual_mm`` refuses (``BASE_FRAME_RESIDUAL``: a base marker
    moved or the frame was knocked; rebuild the base frame — bb_moved does not
    override). No pool: accepted (it seeds one)."""
    shift_mm = shift_deg = 0.0
    qtm = False
    if base_ref is not None:
        shift_mm = float(np.linalg.norm(base.origin_mm - np.asarray(base_ref['origin_mm'], float)))
        shift_deg = wrap_deg(base.heading_deg - float(base_ref['heading_deg']))
        qtm = shift_mm > BASE_SHIFT_REPORT_MM or abs(shift_deg) > BASE_SHIFT_REPORT_DEG
    frame_note = (f'; QTM frame shift: base moved {shift_mm:.2f} mm, {shift_deg:+.3f}° in the '
                  'world (accepted)' if qtm else '')
    if base.residual_mm > max_residual_mm:
        return BaseGateVerdict(
            False, f'BASE_FRAME_RESIDUAL: a base marker sits {base.residual_mm:.2f} mm off the '
                   f'base template > {max_residual_mm:.2f} mm — the base frame was knocked or a '
                   'marker moved; rebuild bb_base_frame.json (bb_moved does not override this)',
            base_shift_mm=shift_mm, base_shift_deg=shift_deg, qtm_shift=qtm)
    if pooled is None:
        return BaseGateVerdict(True, f'base gate: no pooled BB-in-base constant — this sweep '
                                     f'seeds it{frame_note}', base_shift_mm=shift_mm,
                               base_shift_deg=shift_deg, qtm_shift=qtm, no_reference=True)
    dk = wrap_deg(kappa_deg - pooled.kappa_deg)
    thr = max(n_sigma * math.hypot(sigma_deg, pooled.se_deg), min_deg)
    da = float(np.linalg.norm(np.asarray(axis_point_b_mm, float) - pooled.axis_point_b_mm))
    head = (f'base gate: Δκ {dk:+.3f}° (limit ±{thr:.3f}°), Δp_b {da:.2f} mm vs the pooled '
            f'BB-in-base ({pooled.count_note()})')
    bad = []
    if abs(dk) > thr:
        bad.append(f'Δκ {dk:+.3f}° exceeds ±{thr:.3f}°')
    if da > max_axis_shift_mm:
        bad.append(f'axis point moved {da:.2f} mm > {max_axis_shift_mm:.1f} mm in the base frame')
    if not bad:
        return BaseGateVerdict(True, f'{head} ok{frame_note}', dk, thr, da, shift_mm, shift_deg, qtm)
    if bb_moved:
        return BaseGateVerdict(True, f'{head} — BB-in-base changed, ACCEPTED on the bb_moved '
                                     f'override (new pool; refit the aim correction){frame_note}',
                               dk, thr, da, shift_mm, shift_deg, qtm, reset_pool=True,
                               overridden=True)
    return BaseGateVerdict(
        False, f'BB_IN_BASE_MOVED: {", ".join(bad)} (pooled {pooled.count_note()}) — BB or the '
               'base frame moved on the shelf; set bb_moved:=true if BB was moved',
        dk, thr, da, shift_mm, shift_deg, qtm)


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
