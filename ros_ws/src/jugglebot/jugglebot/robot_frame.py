"""Robot frame: Jugglebot's base frame, measured in software from its labelled markers.

Why. QTM's world frame is re-aligned by hand to the robot base, and every
consumer has assumed "world == robot frame" ever since. That assumption is
broken by any QTM re-definition (2026-10-10 23:32) or by moving the robot
without re-aligning, and nothing noticed. Seven markers fixed to the robot base
are labelled by QTM (``Base - 1`` .. ``Base - 7``); their positions in the
robot frame were learned once and frozen
(``resources/jugglebot_robot_frame.json``), so posing them every frame gives
the robot frame in world coordinates directly.

The robot frame IS the kinematic-calibration frame: the QTM global frame of bag
``2026-09-27_14-31-13`` (kincal capture, "frame: Base = QTM global"). The
template's coordinates are the markers' world coordinates in that bag, so on
that bag the fit is the identity by construction. The ~0.8° lever-arm tilt
between the QTM Base body and the machine's base plane (logbook 2026-09-23) is
inside that frame by definition.

Convention (``RobotFrame``)::

    p_world = R @ p_robot + t        p_robot = R.T @ (p_world - t)
    velocities rotate only

``R`` is robot -> world, ``t`` the robot origin in world (mm). With the exact
identity (R == I, t == 0) every transform returns its input bit for bit, so a
QTM frame aligned to the robot reproduces today's numbers exactly.

Name. "Robot frame" everywhere; "base frame" in this codebase already means the
Ball Butler SHELF marker frame (:mod:`jugglebot.bb_base_frame`).

All functions are pure Python + numpy — no ROS2 dependency, no clock reads
(stamps come from the caller).
"""

from __future__ import annotations

import json
import math
import re
from collections import deque
from dataclasses import dataclass, field
from typing import ClassVar, Dict, Iterable, Optional, Tuple

import numpy as np

from .bb_base_frame import mean_rotation
from .bb_calibration import kabsch

# ── Constants ────────────────────────────────────────────────────────────────

#: QTM labels of the robot-base markers: 'Base - 1' .. 'Base - 7'.
ROBOT_MARKER_LABEL_PREFIX = 'Base - '
#: Fewest matched labels for a fit.
ROBOT_FRAME_MIN_MARKERS = 4
#: Fit RMS cap (mm); above it the frame is UNKNOWN. Recipe:
#: ``python -I tools/probes/robot_frame_residual_probe.py`` (2026-10-19,
#: ``temp/probes/robot_frame/20261019-094945``): per-frame Kabsch RMS of
#: Base - 1..7, 3000 frames per bag. Against a template learned from the kincal
#: bag 2026-09-27_14-31-13: that bag median 0.019 / p99 0.057 / max 0.32 mm;
#: bags 2026-10-16_16-23-01 and _22-01-50 median 0.665 / 0.655, p99 0.68 /
#: 0.68, max 0.70 / 1.15 mm. The markers have NOT moved (owner): that 0.66 mm
#: is the reconstruction's systematic shape scatter between QTM wand
#: calibrations (re-wanded 2026-10-16; the BB shelf markers show the same
#: sitting-to-sitting pair-length variation, 0.05-0.48 mm). So the template
#: SHAPE is learned from the freshest bag (live fit ~0.03 mm) and registered
#: once into the kincal frame (:func:`register_template`), and the cap stays at
#: 1.0 mm because the next re-wand can put ~0.66 mm of scatter back: 1.0 mm
#: holds that with 1.5x margin and still refuses the isolated mislabel frames
#: (max 1.15 mm). One marker displaced by d mm adds ~0.33 d to the RMS.
#: Weak direction: the markers are near-coplanar (z spans 43 mm), so a VERTICAL
#: error on Base - 7 (or 1 / 4) is mostly absorbed as tilt: 1 mm of Base - 7 z
#: adds only 0.11 mm RMS but tilts the frame 0.12 deg; spending the whole cap
#: on one marker's error tilts it up to 1.10 deg. The RMS cap is therefore not
#: a tilt guard; ``ROBOT_FRAME_MAX_TILT_FROM_BUILD_DEG`` is.
ROBOT_FRAME_MAX_RMS_MM = 1.0
#: Largest angle (deg) between the live fit's robot z axis (R[:, 2]) and the
#: template's ``z_axis_world_at_build``. The floor is fixed, so sliding the
#: robot keeps its tilt; a tilt change means a bad fit (or a QTM frame that is
#: no longer level-aligned). 0.2 deg moves the cup by 0.9 mm per 250 mm of
#: height above the rest plane. Measured 2026-10-19 (the three probe bags,
#: shape learned from 2026-10-16_22-01-50 and registered into the kincal
#: frame, 3000 frames each): window-frame z axes differ by 0.022-0.036 deg
#: between bags (0.0035 / 0.0375 / 0.0198 deg from world z); per-frame
#: scatter p99 0.006-0.011 deg, single-frame max 0.13 deg (kincal bag, an
#: isolated frame the tracker window absorbs). 0.2 deg is ~5x the
#: between-bag spread.
ROBOT_FRAME_MAX_TILT_FROM_BUILD_DEG = 0.2
#: Second principal RMS spread (mm) the matched markers must have: Base - 3,
#: 5, 6 and 7 are near-collinear (x ≈ 0; spread 3.3 mm), so that subset alone
#: cannot fix the rotation about its own line. Every other 4-subset of the
#: kincal template is >= 19 mm ({2, 3, 5, 6}); 5+ markers are larger still.
ROBOT_FRAME_MIN_SPREAD_MM = 10.0
#: Window agreement for :class:`RobotFrameTracker`: a fit further than this
#: from the window's medoid fit is left out of the current frame.
ROBOT_FRAME_OUTLIER_MM = 1.0
ROBOT_FRAME_OUTLIER_DEG = 0.05

#: ``RobotFrameFit.reason`` values ('' when the fit is OK).
REASON_TOO_FEW_MARKERS = 'TOO_FEW_MARKERS'
REASON_DEGENERATE = 'DEGENERATE_GEOMETRY'
REASON_RMS_OVER_CAP = 'RMS_OVER_CAP'
REASON_TILT_FROM_BUILD = 'TILT_FROM_BUILD'

_LABEL_NUM_RE = re.compile(r'(\d+)$')


def _label_key(label: str):
    m = _LABEL_NUM_RE.search(label)
    return (0, int(m.group(1)), label) if m else (1, 0, label)


def rotation_angle_deg(R: np.ndarray) -> float:
    """Rotation angle of a proper rotation matrix (deg)."""
    c = (float(np.trace(R)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def _readonly(a) -> np.ndarray:
    a = np.array(a, dtype=float, copy=True)
    a.flags.writeable = False
    return a


# ── The frame ────────────────────────────────────────────────────────────────

@dataclass(frozen=True, eq=False)
class RobotFrame:
    """The robot frame in world: ``p_world = R @ p_robot + t`` (mm)."""
    R: np.ndarray                   # (3, 3) robot -> world, proper rotation
    t: np.ndarray                   # (3,) robot origin in world, mm
    rms_mm: float = 0.0
    n_matched: int = 0
    stamp_ns: Optional[int] = None
    _exact_identity: bool = field(init=False, repr=False, default=False)

    IDENTITY: ClassVar['RobotFrame']

    def __post_init__(self):
        R = _readonly(self.R)
        t = _readonly(self.t).reshape(3)
        t.flags.writeable = False
        if R.shape != (3, 3) or not (np.all(np.isfinite(R)) and np.all(np.isfinite(t))):
            raise ValueError(f'RobotFrame: need a finite (3,3) R and (3,) t, got {R.shape}')
        object.__setattr__(self, 'R', R)
        object.__setattr__(self, 't', t)
        object.__setattr__(self, '_exact_identity',
                           bool(np.array_equal(R, np.eye(3)) and not np.any(t)))

    @classmethod
    def identity(cls, *, stamp_ns: Optional[int] = None) -> 'RobotFrame':
        return cls(np.eye(3), np.zeros(3), stamp_ns=stamp_ns)

    # Transforms accept a (3,) point or an (n, 3) array of points.
    def to_world(self, p_robot) -> np.ndarray:
        p = np.array(p_robot, dtype=float)
        return p if self._exact_identity else p @ self.R.T + self.t

    def to_robot(self, p_world) -> np.ndarray:
        p = np.array(p_world, dtype=float)
        return p if self._exact_identity else (p - self.t) @ self.R

    def vel_to_robot(self, v_world) -> np.ndarray:
        v = np.array(v_world, dtype=float)
        return v if self._exact_identity else v @ self.R

    def vel_to_world(self, v_robot) -> np.ndarray:
        v = np.array(v_robot, dtype=float)
        return v if self._exact_identity else v @ self.R.T

    def quaternion_xyzw(self) -> Tuple[float, float, float, float]:
        """R (robot -> world) as a unit quaternion (x, y, z, w), w >= 0."""
        return _matrix_to_quat_xyzw(self.R)

    @classmethod
    def from_pose(cls, position_xyz, quat_xyzw, *, rms_mm: float = 0.0, n_matched: int = 0,
                  stamp_ns: Optional[int] = None) -> 'RobotFrame':
        """From a pose message's position (robot origin in world, mm) and
        orientation (robot -> world). Raises ValueError on a zero/non-finite
        quaternion."""
        return cls(_quat_xyzw_to_matrix(quat_xyzw), np.asarray(position_xyz, dtype=float),
                   rms_mm=float(rms_mm), n_matched=int(n_matched), stamp_ns=stamp_ns)

    def is_identity(self, tol: float = 0.0) -> bool:
        """Exact identity at ``tol`` = 0; else every |R - I| and |t| entry <= tol."""
        if tol <= 0.0:
            return self._exact_identity
        return bool(np.max(np.abs(self.R - np.eye(3))) <= tol and np.max(np.abs(self.t)) <= tol)

    def moved_from(self, other: 'RobotFrame', pos_tol_mm: float, ang_tol_deg: float) -> bool:
        """True if the origin moved more than ``pos_tol_mm`` or the relative
        rotation exceeds ``ang_tol_deg``."""
        if float(np.linalg.norm(self.t - other.t)) > pos_tol_mm:
            return True
        return rotation_angle_deg(other.R.T @ self.R) > ang_tol_deg


RobotFrame.IDENTITY = RobotFrame.identity()


def _matrix_to_quat_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0.0:
        q = -q
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _quat_xyzw_to_matrix(q) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(4)
    n = float(np.linalg.norm(q))
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError(f'quaternion {q.tolist()} is zero or non-finite')
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ── Template ─────────────────────────────────────────────────────────────────

@dataclass(frozen=True, eq=False)
class RobotFrameTemplate:
    """The robot-base markers' coordinates in the robot frame."""
    labels: Tuple[str, ...]
    points_mm: np.ndarray           # (n, 3), rows in ``labels`` order
    source: str = ''
    provenance: dict = field(default_factory=dict)
    #: Unit robot z axis in world (the fitted R[:, 2]) when the template was
    #: built; None skips the tilt-from-build check (optional: schema stays 1).
    z_axis_world_at_build: Optional[np.ndarray] = None

    def __post_init__(self):
        z = self.z_axis_world_at_build
        if z is not None:
            z = np.asarray(z, dtype=float).reshape(3)
            n = float(np.linalg.norm(z))
            if not (math.isfinite(n) and n > 0.5):
                raise ValueError(f'z_axis_world_at_build {z.tolist()} is not a unit vector')
            object.__setattr__(self, 'z_axis_world_at_build', _readonly(z / n))

    def as_dict(self, frame: str = 'robot') -> dict:
        """JSON schema 1 (what :func:`load_robot_frame_template` reads)."""
        d = {'schema': 1, 'frame': frame,
             'markers': [{'label': lb, 'xyz_mm': [float(v) for v in p]}
                         for lb, p in zip(self.labels, self.points_mm)],
             'provenance': dict(self.provenance)}
        if self.z_axis_world_at_build is not None:
            d['z_axis_world_at_build'] = [float(v) for v in self.z_axis_world_at_build]
        return d


def load_robot_frame_template(path: str) -> RobotFrameTemplate:
    """Load ``resources/jugglebot_robot_frame.json``. Raises ValueError, naming
    the file, if it is unreadable or malformed."""
    try:
        with open(path, 'r') as f:
            data = json.load(f)
    except (OSError, ValueError) as e:
        raise ValueError(f'robot frame template {path}: unreadable: {e!r}')
    if not isinstance(data, dict) or data.get('schema') != 1:
        sch = data.get('schema') if isinstance(data, dict) else None
        raise ValueError(f'robot frame template {path}: schema {sch!r}, need 1')
    try:
        labels = tuple(str(m['label']) for m in data['markers'])
        P = np.array([[float(v) for v in m['xyz_mm']] for m in data['markers']], dtype=float)
        prov = dict(data.get('provenance') or {})
        z = data.get('z_axis_world_at_build')
        z = None if z is None else np.array([float(v) for v in z], dtype=float)
        if z is not None and (z.shape != (3,) or not np.all(np.isfinite(z))
                              or abs(float(np.linalg.norm(z)) - 1.0) > 1e-3):
            raise ValueError(f'z_axis_world_at_build {z.tolist()} is not a finite unit 3-vector')
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f'malformed robot frame template {path}: {e!r}')
    if P.ndim != 2 or P.shape != (len(labels), 3) or not np.all(np.isfinite(P)):
        raise ValueError(f'robot frame template {path}: every marker needs a finite xyz_mm')
    if len(set(labels)) != len(labels):
        raise ValueError(f'robot frame template {path}: duplicate labels {labels}')
    if not all(lb.startswith(ROBOT_MARKER_LABEL_PREFIX) for lb in labels):
        raise ValueError(f'robot frame template {path}: labels must start with '
                         f'{ROBOT_MARKER_LABEL_PREFIX!r}: {labels}')
    if len(labels) < ROBOT_FRAME_MIN_MARKERS:
        raise ValueError(f'robot frame template {path}: {len(labels)} markers, need '
                         f'>= {ROBOT_FRAME_MIN_MARKERS}')
    return RobotFrameTemplate(labels=labels, points_mm=_readonly(P), source=str(path),
                              provenance=prov, z_axis_world_at_build=z)


def register_template(shape: RobotFrameTemplate, reference: RobotFrameTemplate):
    """Express ``shape`` (e.g. learned from the freshest bag) in ``reference``'s
    frame (the kincal frame): Kabsch of the shape's points onto the reference's
    over their common labels, applied to every shape point. Returns
    ``(template, registration_rms_mm, per_marker_mm)``; the template keeps the
    shape's labels, records the registration in ``provenance['registration']``
    and carries no ``z_axis_world_at_build`` (the builder sets it). Raises
    ValueError with fewer than ``ROBOT_FRAME_MIN_MARKERS`` common labels."""
    ref = {lb: p for lb, p in zip(reference.labels, reference.points_mm)}
    common = [lb for lb in shape.labels if lb in ref]
    if len(common) < ROBOT_FRAME_MIN_MARKERS:
        raise ValueError(f'register_template: {len(common)} common labels, need '
                         f'>= {ROBOT_FRAME_MIN_MARKERS}')
    idx = {lb: i for i, lb in enumerate(shape.labels)}
    A = np.array([shape.points_mm[idx[lb]] for lb in common])
    B = np.array([ref[lb] for lb in common])
    R, t, rms = kabsch(A, B)
    P = np.asarray(shape.points_mm) @ R.T + t
    per = {lb: float(d) for lb, d in zip(common, np.linalg.norm(A @ R.T + t - B, axis=1))}
    prov = dict(shape.provenance)
    prov['registration'] = {
        'reference': reference.source, 'shape': shape.source, 'labels': common,
        'rms_mm': float(rms), 'per_marker_mm': per,
        'rotation_deg': rotation_angle_deg(R), 'translation_mm': [float(v) for v in t]}
    out = RobotFrameTemplate(labels=tuple(shape.labels), points_mm=_readonly(P),
                             source=shape.source, provenance=prov)
    return out, float(rms), per


def write_robot_frame_template(path: str, template: RobotFrameTemplate,
                               frame: str = 'robot') -> None:
    with open(path, 'w') as f:
        json.dump(template.as_dict(frame), f, indent=2)
        f.write('\n')


def _iter_prefixed(labelled, prefix: str = ROBOT_MARKER_LABEL_PREFIX):
    """(label, xyz) of finite, prefixed labels; the first occurrence of a label wins."""
    seen = set()
    for row in labelled or ():
        try:
            label = row[0]
            xyz = (float(row[1]), float(row[2]), float(row[3]))
        except (IndexError, TypeError, ValueError):
            continue
        if not isinstance(label, str) or not label.startswith(prefix) or label in seen:
            continue
        if not all(math.isfinite(v) for v in xyz):
            continue
        seen.add(label)
        yield label, xyz


def learn_robot_frame_template(frames: Iterable, labels=None, *, inlier_mm: float = 1.0,
                               source: str = '') -> RobotFrameTemplate:
    """Per-label robust mean over frames of labelled tuples: the median, then
    the mean of the samples within ``inlier_mm`` of it. ``labels`` None = every
    prefixed label seen. The frames must be in the frame the template defines
    (the kincal bag's world). Raises ValueError with fewer than
    ``ROBOT_FRAME_MIN_MARKERS`` labels."""
    want = None if labels is None else set(labels)
    samples: Dict[str, list] = {}
    n_frames = 0
    for fr in frames:
        n_frames += 1
        for label, xyz in _iter_prefixed(fr):
            if want is None or label in want:
                samples.setdefault(label, []).append(xyz)
    order = sorted(samples, key=_label_key) if labels is None else \
        [lb for lb in labels if lb in samples]
    if len(order) < ROBOT_FRAME_MIN_MARKERS:
        raise ValueError(f'learn_robot_frame_template: {len(order)} labels seen in '
                         f'{n_frames} frames, need >= {ROBOT_FRAME_MIN_MARKERS}')
    pts, stats = [], {}
    for lb in order:
        S = np.asarray(samples[lb], dtype=float)
        med = np.median(S, axis=0)
        keep = np.linalg.norm(S - med, axis=1) <= inlier_mm
        mean = S[keep].mean(axis=0) if keep.any() else med
        pts.append(mean)
        stats[lb] = {'n': int(len(S)), 'n_inliers': int(keep.sum()),
                     'sd_mm': [float(v) for v in (S[keep].std(axis=0) if keep.sum() > 1
                                                  else np.zeros(3))]}
    prov = {'n_frames': n_frames, 'inlier_mm': float(inlier_mm), 'per_marker': stats}
    return RobotFrameTemplate(labels=tuple(order), points_mm=_readonly(pts), source=source,
                              provenance=prov)


# ── Fit ──────────────────────────────────────────────────────────────────────

@dataclass(frozen=True, eq=False)
class RobotFrameFit:
    """One frame's fit; ``frame`` is None unless ``reason`` is ''."""
    frame: Optional[RobotFrame]
    rms_mm: float
    n_matched: int
    per_marker_mm: dict             # label -> |R a + t - b| (mm), empty if not fitted
    reason: str = ''

    @property
    def ok(self) -> bool:
        return self.frame is not None


def fit_robot_frame(labelled, template: RobotFrameTemplate, *,
                    max_rms_mm: float = ROBOT_FRAME_MAX_RMS_MM,
                    min_markers: int = ROBOT_FRAME_MIN_MARKERS,
                    min_spread_mm: float = ROBOT_FRAME_MIN_SPREAD_MM,
                    max_tilt_from_build_deg: float = ROBOT_FRAME_MAX_TILT_FROM_BUILD_DEG,
                    stamp_ns: Optional[int] = None) -> RobotFrameFit:
    """Kabsch fit (proper rotation) of the template onto a frame's labelled
    markers. ``labelled`` is ``MocapFrame.labelled``: (label, x, y, z[, residual])
    rows; non-prefixed, unknown, non-finite and repeated labels are ignored.
    Never raises on bad input: the fit comes back with a reason instead."""
    index = {lb: i for i, lb in enumerate(template.labels)}
    A, B, names = [], [], []
    for label, xyz in _iter_prefixed(labelled):
        i = index.get(label)
        if i is not None:
            A.append(template.points_mm[i])
            B.append(xyz)
            names.append(label)
    n = len(names)
    if n < max(int(min_markers), 3):
        return RobotFrameFit(None, math.nan, n, {}, REASON_TOO_FEW_MARKERS)
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    sv = np.linalg.svd(A - A.mean(axis=0), compute_uv=False)
    if sv[1] / math.sqrt(n) < min_spread_mm:
        return RobotFrameFit(None, math.nan, n, {}, REASON_DEGENERATE)
    if np.array_equal(A, B):
        # Exactly the template's own frame: the identity, not SVD round-off.
        R, t, rms = np.eye(3), np.zeros(3), 0.0
    else:
        R, t, rms = kabsch(A, B)
    per = {lb: float(d) for lb, d in zip(names, np.linalg.norm(A @ R.T + t - B, axis=1))}
    if not (math.isfinite(rms) and rms <= max_rms_mm):
        return RobotFrameFit(None, float(rms), n, per, REASON_RMS_OVER_CAP)
    z0 = template.z_axis_world_at_build
    if z0 is not None:
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(R[:, 2] @ z0)))))
        if not tilt <= max_tilt_from_build_deg:
            return RobotFrameFit(None, float(rms), n, per, REASON_TILT_FROM_BUILD)
    return RobotFrameFit(RobotFrame(R, t, rms_mm=float(rms), n_matched=n, stamp_ns=stamp_ns),
                         float(rms), n, per, '')


# ── Tracker (the node's slow running frame) ──────────────────────────────────

class RobotFrameTracker:
    """Fed a few times a second by mocap_node (like
    :class:`jugglebot.bb_base_frame.BaseFrameMonitor`). ``current`` is the frame
    over the last ``window`` OK fits: fits that disagree with the window's
    medoid by more than ``ROBOT_FRAME_OUTLIER_MM`` / ``_DEG`` are left out, then
    the median origin and the SVD-projected mean rotation. After ``window``
    consecutive failed fits the window is cleared: the frame is UNKNOWN again
    (a marker knocked or the robot moved must not leave a stale frame live)."""

    def __init__(self, template: RobotFrameTemplate, *, window: int = 10,
                 max_rms_mm: float = ROBOT_FRAME_MAX_RMS_MM,
                 min_markers: int = ROBOT_FRAME_MIN_MARKERS,
                 max_tilt_from_build_deg: float = ROBOT_FRAME_MAX_TILT_FROM_BUILD_DEG,
                 outlier_mm: float = ROBOT_FRAME_OUTLIER_MM,
                 outlier_deg: float = ROBOT_FRAME_OUTLIER_DEG):
        self.template = template
        self.window = max(1, int(window))
        self.max_rms_mm = float(max_rms_mm)
        self.min_markers = int(min_markers)
        self.max_tilt_from_build_deg = float(max_tilt_from_build_deg)
        self.outlier_mm = float(outlier_mm)
        self.outlier_deg = float(outlier_deg)
        self._buf = deque(maxlen=self.window)      # OK RobotFrame fits
        self._last: Optional[RobotFrameFit] = None
        self._last_stamp_ns: Optional[int] = None
        self._fails = 0
        self._current: Optional[RobotFrame] = None

    def feed(self, labelled, stamp_ns: Optional[int]) -> RobotFrameFit:
        fit = fit_robot_frame(labelled, self.template, max_rms_mm=self.max_rms_mm,
                              min_markers=self.min_markers,
                              max_tilt_from_build_deg=self.max_tilt_from_build_deg,
                              stamp_ns=stamp_ns)
        self._last = fit
        if stamp_ns is not None:
            self._last_stamp_ns = int(stamp_ns)
        if fit.ok:
            self._fails = 0
            self._buf.append(fit.frame)
            self._current = None                    # recomputed lazily
        else:
            self._fails += 1
            if self._fails >= self.window and self._buf:
                self._buf.clear()
                self._current = None
        return fit

    def current(self) -> Optional[RobotFrame]:
        if not self._buf:
            return None
        if self._current is None:
            self._current = self._combine(list(self._buf))
        return self._current

    def _combine(self, frames) -> RobotFrame:
        n = len(frames)
        Rs = np.array([f.R for f in frames])
        ts = np.array([f.t for f in frames])
        ang = np.array([[rotation_angle_deg(Rs[i].T @ Rs[j]) for j in range(n)] for i in range(n)])
        dist = np.linalg.norm(ts[:, None] - ts[None], axis=2)
        # Medoid: the fit closest to all others (mm + deg scaled to mm by the caps).
        cost = (dist / self.outlier_mm + ang / self.outlier_deg).sum(axis=1)
        m = int(np.argmin(cost))
        keep = (dist[m] <= self.outlier_mm) & (ang[m] <= self.outlier_deg)
        R = mean_rotation(Rs[keep])
        t = np.median(ts[keep], axis=0)
        last = frames[-1]
        return RobotFrame(R, t, rms_mm=float(np.median([f.rms_mm for f in frames])),
                          n_matched=int(last.n_matched), stamp_ns=last.stamp_ns)

    def status(self, now_ns: Optional[int] = None) -> dict:
        """{'ok', 'rms_mm', 'n_matched', 'reason', 'age_s', 'n_window'}; ``ok``
        is the LAST fit; ``age_s`` is ``now_ns`` minus the newest OK fit's stamp
        (None without both)."""
        last = self._last
        cur = self.current()
        age = None
        if now_ns is not None and cur is not None and cur.stamp_ns is not None:
            age = (int(now_ns) - cur.stamp_ns) / 1e9
        return {'ok': bool(last is not None and last.ok),
                'rms_mm': None if last is None or not math.isfinite(last.rms_mm)
                else float(last.rms_mm),
                'n_matched': 0 if last is None else int(last.n_matched),
                'reason': 'NO_FIT_YET' if last is None else last.reason,
                'age_s': age, 'n_window': len(self._buf)}
