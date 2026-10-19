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

Yaw-only fit (owner, 2026-10-19). The robot's z axis is pinned to world z:
the design already assumes it (the ball tracker's gravity is world -z, the
landing plane is the world z of robot (0, 0, 830)), and the fitted 6-DoF tilt
turned out to be QTM reconstruction bias that varies with platform pose, not
the robot (``temp/probes/robot_frame_diag/report.md``). So the fit solves yaw
about world z + a 3-D translation; ``R`` is still a 3x3 (a pure yaw rotation)
and any residual tilt of the QTM world shows up in the RMS, not in ``R``.

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
#: Fewest markers a fit may use: a frame with fewer matched labels is
#: TOO_FEW_MARKERS, and drop-worst outlier rejection (:func:`fit_robot_frame`)
#: never drops below it. With 5 retained markers one displaced marker cannot be
#: dropped, so the RMS cap still bounds it there. Also the floor for a template
#: (loader / learner / registration).
ROBOT_FRAME_MIN_FIT_MARKERS = 5
#: Fit RMS cap (mm), 3-D residuals, on the RETAINED markers after drop-worst.
#: Above it the fit fails (RMS_OVER_CAP). Measured 2026-10-19
#: (``temp/probes/robot_frame_diag/report.md``, the three 2026-10-19 bags, every
#: /mocap_data frame): the RMS is a QTM reconstruction BIAS that steps with the
#: configuration, not noise (at-rest per-frame scatter 0.02-0.05 mm). Wherever
#: the robot was moved in the volume it sits at 1.00-1.06 mm; at home with the
#: platform parked raised (occluding the base) it reaches 1.62 mm. The previous
#: 1.0 mm cap sat ON that floor and flapped on all three bags. (History, the
#: 1.5 mm cut of the same day:) 1.5 mm is above
#: the moved-robot plateau; the occluded-home stretches (one marker, Base - 5,
#: reconstructing 1.8-2.9 mm off) are handled by drop-worst, and a failed fit
#: that still agrees with the running frame is held by the tracker
#: (``ROBOT_FRAME_HOLD_MM``). Replay of cap 1.5 + drop-worst to 5 + hold 2 mm
#: (report variant E): locks at 0 s and holds 100 % on all three bags, origin
#: p95 deviation 1.4 mm, yaw p95 up to 0.06 deg. One marker displaced by d mm
#: adds ~0.33 d to the 7-marker RMS.
#: Raised 1.5 -> 2.0 (owner, 2026-10-19): the 14:24 sitting (bag
#: 2026-10-19_14-24-15, temp/probes/robot_frame_sitting_1424/report.md) ran on
#: 5 markers after a 48.5 mm / 14.5 deg hand move, Base - 5 and Base - 6
#: dropped on almost every feed, RMS 1.25-1.33 mm and a max of 1.491 mm against
#: 1.5: a slightly worse spot would not lock. At 2.0 a 5 mm displaced marker
#: passes 7-wide with a 0.84 mm origin error (inside the frame's own ~1 mm
#: scatter) while a 10 mm one is still dropped
#: (temp/probes/robot_frame/replay_2026-10-19/replay.md).
ROBOT_FRAME_MAX_RMS_MM = 2.0
#: Second principal RMS spread (mm) the retained markers must have (checked on
#: the full matched set and on every drop-worst subset): Base - 3, 5, 6 and 7
#: are near-collinear (x ≈ 0; spread 3.3 mm). Every other 4-subset of the
#: kincal template is >= 19 mm ({2, 3, 5, 6}) and every 5-subset larger still,
#: so on the real template this only bites a caller passing a lower
#: ``min_markers``.
ROBOT_FRAME_MIN_SPREAD_MM = 10.0
#: Window agreement for :class:`RobotFrameTracker`: a fit further than this
#: from the window's medoid fit is left out of the current frame.
ROBOT_FRAME_OUTLIER_MM = 1.0
ROBOT_FRAME_OUTLIER_DEG = 0.05
#: Hold rule for :class:`RobotFrameTracker`: a FAILED fit whose uncapped pose
#: is within this of the running frame (origin distance, mm; yaw, deg) does not
#: count toward the window clear. From the report's variant E replay
#: (``temp/probes/robot_frame_diag/report.md`` § 6): the reconstruction bias
#: moves the origin by ~1 mm at home as occlusion changes (p95 1.4 mm), yaw
#: p95 0.03-0.05 deg; a real robot move today was 40-108 mm and 4-12 deg.
ROBOT_FRAME_HOLD_MM = 2.0
ROBOT_FRAME_HOLD_DEG = 0.3

#: Settle spread AND move announcement threshold (owner, 2026-10-19): the
#: running frame is SETTLED only when every fit in a full window agrees with it
#: within this much, and a settled frame that differs from the last announced
#: one by more is a MOVE. One pair: origin scatter p95 is 1.45 mm at home with
#: the platform parking high, per-configuration bias steps ~1 mm, yaw p95
#: 0.06 deg; 2 mm would false-trip during active juggling.
ROBOT_FRAME_MOVE_MM = 5.0
ROBOT_FRAME_MOVE_DEG = 0.3

#: ``RobotFrameFit.reason`` values ('' when the fit is OK).
REASON_TOO_FEW_MARKERS = 'TOO_FEW_MARKERS'
REASON_DEGENERATE = 'DEGENERATE_GEOMETRY'
REASON_RMS_OVER_CAP = 'RMS_OVER_CAP'

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

    def as_dict(self, frame: str = 'robot') -> dict:
        """JSON schema 1 (what :func:`load_robot_frame_template` reads)."""
        return {'schema': 1, 'frame': frame,
                'markers': [{'label': lb, 'xyz_mm': [float(v) for v in p]}
                            for lb, p in zip(self.labels, self.points_mm)],
                'provenance': dict(self.provenance)}


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
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f'malformed robot frame template {path}: {e!r}')
    if P.ndim != 2 or P.shape != (len(labels), 3) or not np.all(np.isfinite(P)):
        raise ValueError(f'robot frame template {path}: every marker needs a finite xyz_mm')
    if len(set(labels)) != len(labels):
        raise ValueError(f'robot frame template {path}: duplicate labels {labels}')
    if not all(lb.startswith(ROBOT_MARKER_LABEL_PREFIX) for lb in labels):
        raise ValueError(f'robot frame template {path}: labels must start with '
                         f'{ROBOT_MARKER_LABEL_PREFIX!r}: {labels}')
    if len(labels) < ROBOT_FRAME_MIN_FIT_MARKERS:
        raise ValueError(f'robot frame template {path}: {len(labels)} markers, need '
                         f'>= {ROBOT_FRAME_MIN_FIT_MARKERS}')
    return RobotFrameTemplate(labels=labels, points_mm=_readonly(P), source=str(path),
                              provenance=prov)


def register_template(shape: RobotFrameTemplate, reference: RobotFrameTemplate):
    """Express ``shape`` (e.g. learned from the freshest bag) in ``reference``'s
    frame (the kincal frame): Kabsch of the shape's points onto the reference's
    over their common labels, applied to every shape point. Returns
    ``(template, registration_rms_mm, per_marker_mm)``; the template keeps the
    shape's labels and records the registration in ``provenance['registration']``.
    This is a full 6-DoF Kabsch (it expresses a learned SHAPE in another
    frame; only the per-frame fit is yaw-only). Raises
    ValueError with fewer than ``ROBOT_FRAME_MIN_FIT_MARKERS`` common labels."""
    ref = {lb: p for lb, p in zip(reference.labels, reference.points_mm)}
    common = [lb for lb in shape.labels if lb in ref]
    if len(common) < ROBOT_FRAME_MIN_FIT_MARKERS:
        raise ValueError(f'register_template: {len(common)} common labels, need '
                         f'>= {ROBOT_FRAME_MIN_FIT_MARKERS}')
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
    ``ROBOT_FRAME_MIN_FIT_MARKERS`` labels."""
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
    if len(order) < ROBOT_FRAME_MIN_FIT_MARKERS:
        raise ValueError(f'learn_robot_frame_template: {len(order)} labels seen in '
                         f'{n_frames} frames, need >= {ROBOT_FRAME_MIN_FIT_MARKERS}')
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

def yaw_fit(A, B):
    """Least-squares yaw-about-z + 3-D translation taking ``A`` onto ``B``
    ((n, 3) each): 2-D Kabsch on xy after centroid removal; the z translation
    is the mean z difference. Returns ``(R, t, rms_mm)`` with ``R`` a pure yaw
    (R[2, 2] == 1 and R[0:2, 2] == R[2, 0:2] == 0 exactly) and the 3-D RMS of
    ``A @ R.T + t - B``."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    a, b = A - ca, B - cb
    # maximise sum b.(R a) = cos(th) * sum(ax bx + ay by) + sin(th) * sum(ax by - ay bx)
    th = math.atan2(float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0])),
                    float(np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1])))
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    t = cb - R @ ca
    rms = float(np.sqrt(np.mean(np.sum((A @ R.T + t - B) ** 2, axis=1))))
    return R, t, rms


@dataclass(frozen=True, eq=False)
class RobotFrameFit:
    """One frame's fit; ``frame`` is None unless ``reason`` is ''.

    ``n_matched`` and ``per_marker_mm`` describe the RETAINED markers (the ones
    the final fit used); ``dropped`` names the markers drop-worst removed, in
    drop order. ``uncapped`` is the final fit's pose whether or not it passed
    the cap (the tracker's hold rule compares it); None when no fit was made
    (too few markers, degenerate geometry)."""
    frame: Optional[RobotFrame]
    rms_mm: float
    n_matched: int
    per_marker_mm: dict             # label -> |R a + t - b| (mm), empty if not fitted
    reason: str = ''
    dropped: Tuple[str, ...] = ()
    uncapped: Optional[RobotFrame] = None

    @property
    def ok(self) -> bool:
        return self.frame is not None


def _spread_ok(A: np.ndarray, min_spread_mm: float) -> bool:
    sv = np.linalg.svd(A - A.mean(axis=0), compute_uv=False)
    return bool(sv[1] / math.sqrt(len(A)) >= min_spread_mm)


def fit_robot_frame(labelled, template: RobotFrameTemplate, *,
                    max_rms_mm: float = ROBOT_FRAME_MAX_RMS_MM,
                    min_markers: int = ROBOT_FRAME_MIN_FIT_MARKERS,
                    min_spread_mm: float = ROBOT_FRAME_MIN_SPREAD_MM,
                    stamp_ns: Optional[int] = None) -> RobotFrameFit:
    """Yaw-only fit (:func:`yaw_fit`) of the template onto a frame's labelled
    markers, with drop-worst outlier rejection. ``labelled`` is
    ``MocapFrame.labelled``: (label, x, y, z[, residual]) rows; non-prefixed,
    unknown, non-finite and repeated labels are ignored (first occurrence wins).

    Rule: fit every matched marker; while the RMS is over ``max_rms_mm`` and
    more than ``min_markers`` remain, drop the single worst-residual marker and
    refit. The fit is OK when the RMS of the retained set is within the cap.
    Fewer than ``min_markers`` matched -> TOO_FEW_MARKERS; the matched set's
    spread under ``min_spread_mm`` -> DEGENERATE_GEOMETRY; a drop that would
    leave a degenerate set is not made (the fit then ends RMS_OVER_CAP). Never
    raises on bad input: the fit comes back with a reason instead."""
    index = {lb: i for i, lb in enumerate(template.labels)}
    A, B, names = [], [], []
    for label, xyz in _iter_prefixed(labelled):
        i = index.get(label)
        if i is not None:
            A.append(template.points_mm[i])
            B.append(xyz)
            names.append(label)
    n = len(names)
    floor = max(int(min_markers), 3)
    if n < floor:
        return RobotFrameFit(None, math.nan, n, {}, REASON_TOO_FEW_MARKERS)
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    if not _spread_ok(A, min_spread_mm):
        return RobotFrameFit(None, math.nan, n, {}, REASON_DEGENERATE)
    keep = list(range(n))
    dropped = []
    while True:
        Ak, Bk = A[keep], B[keep]
        if np.array_equal(Ak, Bk):
            # Exactly the template's own frame: the identity, not round-off.
            R, t, rms = np.eye(3), np.zeros(3), 0.0
        else:
            R, t, rms = yaw_fit(Ak, Bk)
        res = np.linalg.norm(Ak @ R.T + t - Bk, axis=1)
        within = math.isfinite(rms) and rms <= max_rms_mm
        if within or len(keep) <= floor:
            break
        cand = [k for j, k in enumerate(keep) if j != int(np.argmax(res))]
        if not _spread_ok(A[cand], min_spread_mm):
            break
        dropped.append(names[keep[int(np.argmax(res))]])
        keep = cand
    m = len(keep)
    per = {names[k]: float(d) for k, d in zip(keep, res)}
    pose = RobotFrame(R, t, rms_mm=float(rms), n_matched=m, stamp_ns=stamp_ns)
    if not within:
        return RobotFrameFit(None, float(rms), m, per, REASON_RMS_OVER_CAP,
                             tuple(dropped), pose)
    return RobotFrameFit(pose, float(rms), m, per, '', tuple(dropped), pose)


# ── Tracker (the node's slow running frame) ──────────────────────────────────

class RobotFrameTracker:
    """Fed a few times a second by mocap_node (like
    :class:`jugglebot.bb_base_frame.BaseFrameMonitor`). ``current`` is the frame
    over the last ``window`` OK fits: fits that disagree with the window's
    medoid by more than ``ROBOT_FRAME_OUTLIER_MM`` / ``_DEG`` are left out, then
    the median origin and the SVD-projected mean rotation.

    Clear rule: after ``window`` consecutive COUNTED failed fits the window is
    cleared and the frame is UNKNOWN again (a marker knocked or the robot moved
    must not leave a stale frame live). Hold rule: a failed fit whose
    ``uncapped`` pose (the final drop-worst fit on the retained markers, see
    :class:`RobotFrameFit`) agrees with the current running frame within
    ``ROBOT_FRAME_HOLD_MM`` = 2.0 mm (origin) and ``ROBOT_FRAME_HOLD_DEG`` =
    0.3 deg (rotation, a pure yaw difference) is HELD: it neither counts toward
    the clear nor resets the count, and it never enters the window. That rides
    out the configuration-dependent QTM reconstruction bias (RMS up to 1.62 mm
    with the platform parked raised, ``temp/probes/robot_frame_diag/report.md``)
    without letting a stale frame survive a real move: a failed fit that
    disagrees (a real move is tens of mm / degrees) or has no pose (too few
    markers, degenerate) counts, so a moved robot still clears within
    ``window`` feeds (10 = 2 s at mocap_node's 5 Hz). A held feed does not
    refresh the newest-OK stamp, so ``status()['age_s']`` (a display key on
    mocap/status, gated by nothing) grows through a long hold by design; the
    skill_node staleness WARN measures the latched topic's receive time via
    the 5 s republish, not this key."""

    def __init__(self, template: RobotFrameTemplate, *, window: int = 10,
                 max_rms_mm: float = ROBOT_FRAME_MAX_RMS_MM,
                 min_markers: int = ROBOT_FRAME_MIN_FIT_MARKERS,
                 outlier_mm: float = ROBOT_FRAME_OUTLIER_MM,
                 outlier_deg: float = ROBOT_FRAME_OUTLIER_DEG,
                 hold_mm: float = ROBOT_FRAME_HOLD_MM,
                 hold_deg: float = ROBOT_FRAME_HOLD_DEG):
        self.template = template
        self.window = max(1, int(window))
        self.max_rms_mm = float(max_rms_mm)
        self.min_markers = int(min_markers)
        self.outlier_mm = float(outlier_mm)
        self.outlier_deg = float(outlier_deg)
        self.hold_mm = float(hold_mm)
        self.hold_deg = float(hold_deg)
        self._buf = deque(maxlen=self.window)      # OK RobotFrame fits
        self._last: Optional[RobotFrameFit] = None
        self._last_stamp_ns: Optional[int] = None
        self._fails = 0
        self._held = False
        self._current: Optional[RobotFrame] = None
        self._announced: Optional[RobotFrame] = None
        self._event = None
        self.move_count = 0
        self.last_move: Optional[Tuple[float, float, int]] = None

    def feed(self, labelled, stamp_ns: Optional[int]) -> RobotFrameFit:
        fit = self._feed(labelled, stamp_ns)
        self._update_event(stamp_ns)
        return fit

    @property
    def settled(self) -> bool:
        """True iff the window is FULL and every fit in it agrees with the
        running frame within ROBOT_FRAME_MOVE_MM / _DEG."""
        if len(self._buf) < self.window:
            return False
        cur = self.current()
        if cur is None:
            return False
        return not any(f.moved_from(cur, ROBOT_FRAME_MOVE_MM, ROBOT_FRAME_MOVE_DEG)
                       for f in self._buf)

    def _update_event(self, stamp_ns) -> None:
        self._event = None
        if not self.settled:
            return
        cur = self.current()
        if self._announced is None:
            self._announced = cur
            self._event = ('settled', cur, None)
            return
        if cur.moved_from(self._announced, ROBOT_FRAME_MOVE_MM, ROBOT_FRAME_MOVE_DEG):
            d_mm = float(np.linalg.norm(cur.t - self._announced.t))
            d_deg = float(rotation_angle_deg(self._announced.R.T @ cur.R))
            frm = self._announced
            self._announced = cur
            self.move_count += 1
            self.last_move = (d_mm, d_deg, int(stamp_ns) if stamp_ns is not None else 0)
            self._event = ('moved', cur, (d_mm, d_deg, frm))

    def pop_event(self):
        """The event the last feed produced, once: ``('settled', frame, None)``
        (the initial lock) or ``('moved', frame, (d_mm, d_yaw_deg, from_frame))``;
        None otherwise."""
        ev, self._event = self._event, None
        return ev

    def _feed(self, labelled, stamp_ns: Optional[int]) -> RobotFrameFit:
        fit = fit_robot_frame(labelled, self.template, max_rms_mm=self.max_rms_mm,
                              min_markers=self.min_markers, stamp_ns=stamp_ns)
        self._last = fit
        self._held = False
        if stamp_ns is not None:
            self._last_stamp_ns = int(stamp_ns)
        if fit.ok:
            self._fails = 0
            self._buf.append(fit.frame)
            self._current = None                    # recomputed lazily
            return fit
        cur = self.current()
        if (cur is not None and fit.uncapped is not None
                and not fit.uncapped.moved_from(cur, self.hold_mm, self.hold_deg)):
            self._held = True                       # bias excursion: not a fail
            return fit
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
        """{'ok', 'rms_mm', 'n_matched', 'reason', 'age_s', 'n_window',
        'dropped', 'held'}; ``ok``, ``rms_mm``, ``n_matched``, ``reason`` and
        ``dropped`` are the LAST fit's; ``held`` is True when that fit failed
        but was held (:class:`RobotFrameTracker` hold rule); ``age_s`` is
        ``now_ns`` minus the newest OK fit's stamp (None without both)."""
        last = self._last
        lm = self.last_move
        cur = self.current()
        age = None
        if now_ns is not None and cur is not None and cur.stamp_ns is not None:
            age = (int(now_ns) - cur.stamp_ns) / 1e9
        return {'ok': bool(last is not None and last.ok),
                'rms_mm': None if last is None or not math.isfinite(last.rms_mm)
                else float(last.rms_mm),
                'n_matched': 0 if last is None else int(last.n_matched),
                'reason': 'NO_FIT_YET' if last is None else last.reason,
                'age_s': age, 'n_window': len(self._buf),
                'dropped': () if last is None else tuple(last.dropped),
                'held': bool(self._held),
                'settled': bool(self.settled),
                'move_count': int(self.move_count),
                'last_move_mm': 0.0 if lm is None else float(lm[0]),
                'last_move_deg': 0.0 if lm is None else float(lm[1]),
                'last_move_stamp_ns': 0 if lm is None else int(lm[2])}
