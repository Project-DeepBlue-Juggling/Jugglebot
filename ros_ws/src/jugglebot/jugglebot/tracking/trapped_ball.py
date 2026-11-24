"""A marker dwelling under the Hand: pure geometry plus an ONLINE dwell detector.

Why this exists (FEAT-50, BUG-84): the Hand stalled three times on 2026-11-13,
each time with a Ball trapped under the Cup that the Tracker had already dropped
as a Ball track (no ballistic fit once it jostles on the Platform). The raw
unlabelled marker on ``mocap_data`` is still there. ``tools/trapped_ball_finder.py``
finds those offline; this module is the shared geometry (moved here from the
finder, which now imports it) and the live detector the Tracker node feeds.

LOG-ONLY. Nothing here touches the Hand path; the Hand's safety authority stays
the Teensy guard and the jam detector. The node just logs a WARN.

Frames and constants. Markers are in the Platform frame (``to_platform_frame``:
pose x, y as published, z + ``GEOM_INITIAL_HEIGHT_MM``). The Hand-marker fit
(z = -47 + 32.567 mm/rev x rev), the Cup seat and the dwell thresholds live in
``config/hardware_config.yaml`` (``ball_tracking.trapped_*``), read here through
``jugglebot.hardware_config`` as ``TRACKING_TRAPPED_*``. One exception stays a module
constant: ``FIXTURE_MIN_S``, the history the rolling fixture filter needs before it
retires anything (a start-up guard, not a fitted threshold).

Online dwell. ``OnlineDwellDetector.update(t, markers, hand_rev)`` takes one frame
of UNLABELLED markers. Clustering is the batch tool's, through the SAME functions
(``cluster_add`` / ``cluster_region``): every marker that is not None, HAND_MARKER
or IN_CUP joins a cluster within ``dwell_radius`` of its anchor, a gap shorter than
``gap_s`` does not end it, and the cluster's region is the MAJORITY region of its
samples. A cluster warns when it has lasted ``min_s`` and its majority region is
UNDER_HAND (a resting Ball in the rim pocket near (-55, 57) is ON_PLATFORM with
the Hand low: harmless, never warns). A persisting dwell is reported again every
``repeat_s``. The one deliberate difference from the batch tool is that the warning
is restricted to UNDER_HAND clusters; the batch listing also lists ON_PLATFORM ones.

Fixture filter (the choice, 2026-11-14; neighbour merge 2026-11-25). The batch tool
drops any 10 mm voxel occupied in more than 15 % of the WHOLE bag. Online there is
no whole bag, so the detector keeps the voxel occupancy of the last
``fixture_window_s`` (30 s) of FED frames and treats a voxel occupied in more than
``fixture_frac`` (90 %) of them as a fixture. Occupancy is DILATED: a marker marks
its voxel and the 26 neighbours, so a static marker whose jitter straddles a voxel
boundary still makes both voxels read ~100 % (before, it split ~50/50, escaped the
90 % bar and warned every second). Cost: a fixture's footprint is +-1 voxel, so a
Ball within ~10-20 mm of a long-static marker is also retired. The rolling counter
is pruned of dead voxels every ``PRUNE_PERIOD_S`` of data time, not per frame. A fixture needs at least ``fixture_min_s`` (10 s) of history before it
can be declared, otherwise every marker is "occupied in 100 % of 3 frames". Costs
of the choice, accepted: a real trapped Ball is only reclassified as a fixture
after sitting ~27 s of 30 s (a Hand stall ends far sooner), and for the first
10 s after node start a static marker can still warn.

NOT done: the finder's lower ``STALL_DWELL_MIN_S`` = 0.08 s bar AT a stall needs
stall detection (|iq| clamp, command-vs-position error); the live WARN uses the
0.30 s listing threshold only.

No ROS2 imports: pure Python, Python 3.8.
"""
from __future__ import annotations

import math
from collections import Counter, deque
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

import jugglebot.hardware_config as hw

# ---------------------------------------------------------------------------
# Geometry constants. Config-backed ones come from hardware_config; the region
# tolerances below are the finder's classification bounds (not physical
# parameters of a single measured quantity) and stay named here.
# ---------------------------------------------------------------------------
INITIAL_HEIGHT_MM = float(hw.GEOM_INITIAL_HEIGHT_MM)
HAND_MM_PER_REV = float(hw.GEOM_HAND_MM_PER_REV)
HAND_MARKER_Z0_MM = float(hw.TRACKING_TRAPPED_HAND_MARKER_Z0_MM)
HAND_MARKER_XY_MM = (float(hw.TRACKING_TRAPPED_HAND_MARKER_X_MM),
                     float(hw.TRACKING_TRAPPED_HAND_MARKER_Y_MM))
HAND_MARKER_RADIAL_TOL_MM = 25.0
HAND_MARKER_AXIAL_TOL_MM = 25.0
CUP_SEAT_XY_MM = (float(hw.TRACKING_TRAPPED_CUP_SEAT_X_MM),
                  float(hw.TRACKING_TRAPPED_CUP_SEAT_Y_MM))
CUP_SEAT_Z0_MM = float(hw.TRACKING_TRAPPED_CUP_SEAT_Z0_MM)
CUP_SEAT_RADIAL_TOL_MM = 20.0
CUP_SEAT_AXIAL_TOL_MM = 22.0
UNDER_R_MM = 95.0             # arm opening radius 70 + ~25 mm ball radius
UNDER_Z_MIN_MM = -250.0       # lowest believable resting height in the tube
PLATFORM_R_MM = 230.0         # plat_radius_mm 219.075 + margin
ON_PLATFORM_Z_MM = (-60.0, 80.0)  # a resting Ball's centre; excludes airborne Balls

DWELL_RADIUS_MM = float(hw.TRACKING_TRAPPED_DWELL_RADIUS_MM)
DWELL_MIN_S = float(hw.TRACKING_TRAPPED_DWELL_MIN_S)
DWELL_GAP_S = float(hw.TRACKING_TRAPPED_DWELL_GAP_S)
WARN_REPEAT_S = float(hw.TRACKING_TRAPPED_WARN_REPEAT_S)
STRIDE = int(hw.TRACKING_TRAPPED_STRIDE)
FIXTURE_FRAC_ONLINE = float(hw.TRACKING_TRAPPED_FIXTURE_FRAC)
FIXTURE_WINDOW_S = float(hw.TRACKING_TRAPPED_FIXTURE_WINDOW_S)
#: Not in config: a start-up guard for the rolling filter, not a fitted threshold.
FIXTURE_MIN_S = 10.0
HAND_STALE_S = float(hw.TRACKING_TRAPPED_HAND_STALE_S)
#: Platform pose older than this {s} (wall clock at receipt) is treated as missing:
#: a pose from before a Mocap dropout would put the "under the Hand" region wrong.
POSE_STALE_S = HAND_STALE_S
#: Dead voxels (count 0) are swept from the rolling counter this often {s of data time}.
PRUNE_PERIOD_S = 5.0
#: Rev-independent zone a marker must be in to be worth recording in the fixture history
#: (Platform frame, mm): everything the region test could ever call UNDER_HAND / ON_PLATFORM
#: for any Hand position. Markers outside (airborne Balls, Base clutter) cost nothing.
RECORD_Z_MM = (UNDER_Z_MIN_MM, 300.0)
#: Markers in these regions never join a cluster (shared by the batch tool and the live detector).
SKIP_REGIONS = (None, "HAND_MARKER", "IN_CUP")


# ---------------------------------------------------------------------------
# Geometry (pure)
# ---------------------------------------------------------------------------
def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_platform_frame(world_pts: np.ndarray, pose: Sequence[float]) -> np.ndarray:
    """World markers (N,3) mm -> Platform frame. pose = (x, y, z_rel, qx, qy, qz, qw)."""
    R = quat_to_matrix(*pose[3:7])
    c = np.array([pose[0], pose[1], pose[2] + INITIAL_HEIGHT_MM])
    return (np.asarray(world_pts, float) - c) @ R


def hand_marker_local_z(rev: float) -> float:
    """Platform-frame z of the Hand's own marker at Hand position ``rev``."""
    return HAND_MARKER_Z0_MM + HAND_MM_PER_REV * rev


def region_of(local_xyz: Sequence[float], hand_rev: float) -> Optional[str]:
    """Where a marker (Platform frame, mm) sits relative to the Cup.

    "HAND_MARKER" - it is the Hand's own marker (to be dropped, never a Ball);
    "IN_CUP"      - a Ball seated in the Cup, moving with the Hand (harmless, never a dwell);
    "UNDER_HAND"  - inside the arm tube, below the Hand marker;
    "ON_PLATFORM" - resting on the Platform top outside the tube;
    None          - anywhere else (airborne, on the floor, on the Base...).
    """
    x, y, z = float(local_xyz[0]), float(local_xyz[1]), float(local_xyz[2])
    r_hand = math.hypot(x - HAND_MARKER_XY_MM[0], y - HAND_MARKER_XY_MM[1])
    if r_hand <= HAND_MARKER_RADIAL_TOL_MM and abs(z - hand_marker_local_z(hand_rev)) <= HAND_MARKER_AXIAL_TOL_MM:
        return "HAND_MARKER"
    if (math.hypot(x - CUP_SEAT_XY_MM[0], y - CUP_SEAT_XY_MM[1]) <= CUP_SEAT_RADIAL_TOL_MM
            and abs(z - (CUP_SEAT_Z0_MM + HAND_MM_PER_REV * hand_rev)) <= CUP_SEAT_AXIAL_TOL_MM):
        return "IN_CUP"
    r = math.hypot(x, y)
    if r <= UNDER_R_MM and UNDER_Z_MIN_MM <= z < hand_marker_local_z(hand_rev):
        return "UNDER_HAND"
    if r <= PLATFORM_R_MM and ON_PLATFORM_Z_MM[0] <= z <= ON_PLATFORM_Z_MM[1]:
        return "ON_PLATFORM"
    return None


def _voxel(p: Sequence[float]) -> Tuple[int, int, int]:
    return (math.floor(float(p[0]) / 10.0), math.floor(float(p[1]) / 10.0),
            math.floor(float(p[2]) / 10.0))


_NEIGHBOURS = tuple((i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1))


def cluster_add(open_: List[dict], p: np.ndarray, t: float, region: str,
                dwell_radius: float) -> dict:
    """Join ``p`` (Platform frame) to the first open cluster within ``dwell_radius``
    of its anchor, or open a new one. The ONE clustering rule of the batch tool
    (``find_dwells``) and the live detector. Returns the cluster."""
    for c in open_:
        if np.linalg.norm(p - c["anchor"]) <= dwell_radius:
            c["t1"] = t
            c["regions"][region] += 1
            c["sum"] += p
            c["n"] += 1
            return c
    c = {"anchor": p, "t0": t, "t1": t, "regions": Counter({region: 1}), "sum": p.copy(), "n": 1}
    open_.append(c)
    return c


def cluster_region(c: dict) -> str:
    """The cluster's MAJORITY region."""
    rc = c["regions"]
    return max(rc, key=rc.get)


# ---------------------------------------------------------------------------
# Online detector
# ---------------------------------------------------------------------------
class OnlineDwellDetector:
    """One-frame-at-a-time UNDER_HAND dwell detector (see module docstring)."""

    def __init__(self, dwell_radius: float = DWELL_RADIUS_MM, gap_s: float = DWELL_GAP_S,
                 min_s: float = DWELL_MIN_S, repeat_s: float = WARN_REPEAT_S,
                 fixture_frac: float = FIXTURE_FRAC_ONLINE,
                 fixture_window_s: float = FIXTURE_WINDOW_S,
                 fixture_min_s: float = FIXTURE_MIN_S) -> None:
        self.dwell_radius = dwell_radius
        self.gap_s = gap_s
        self.min_s = min_s
        self.repeat_s = repeat_s
        self.fixture_frac = fixture_frac
        self.fixture_window_s = fixture_window_s
        self.fixture_min_s = fixture_min_s
        self._open: List[dict] = []
        self._hist: Deque[Tuple[float, set]] = deque()
        self._vox: Counter = Counter()
        self._last_prune = -float("inf")

    @staticmethod
    def _recordable(p: np.ndarray) -> bool:
        return (RECORD_Z_MM[0] <= p[2] <= RECORD_Z_MM[1]
                and math.hypot(p[0], p[1]) <= PLATFORM_R_MM)

    def _push_history(self, t: float, pts: Sequence[np.ndarray]) -> None:
        seen = set()
        for p in pts:
            i, j, k = _voxel(p)
            for di, dj, dk in _NEIGHBOURS:
                seen.add((i + di, j + dj, k + dk))
        self._hist.append((t, seen))
        self._vox.update(seen)
        while self._hist and t - self._hist[0][0] > self.fixture_window_s:
            _t0, old = self._hist.popleft()
            self._vox.subtract(old)
        if t - self._last_prune >= PRUNE_PERIOD_S:
            self._last_prune = t
            for v in [v for v, n in self._vox.items() if n <= 0]:
                del self._vox[v]

    def _is_fixture(self, p: np.ndarray) -> bool:
        if len(self._hist) < 2 or self._hist[-1][0] - self._hist[0][0] < self.fixture_min_s:
            return False
        return self._vox.get(_voxel(p), 0) / len(self._hist) > self.fixture_frac

    def update(self, t: float, markers: Sequence[Sequence[float]],
               hand_rev: Optional[float]) -> List[Dict[str, object]]:
        """Feed one frame; return the WARN events due now (usually none).

        markers: UNLABELLED markers, Platform frame, mm. hand_rev None or
        non-finite (no telemetry / stale): nothing is evaluated, nothing returned.
        Event: {t, dur_s, local_mean (x,y,z), hand_rev, region}.
        """
        if hand_rev is None or not np.isfinite(hand_rev):
            return []
        pts = [np.asarray(m, float) for m in markers]
        pts = [p for p in pts if p.shape == (3,) and np.all(np.isfinite(p))]
        self._push_history(t, [p for p in pts if self._recordable(p)])
        self._open = [c for c in self._open if t - c["t1"] <= self.gap_s]
        events: List[Dict[str, object]] = []
        touched: List[dict] = []
        for p in pts:
            if self._is_fixture(p):
                continue
            reg = region_of(p, hand_rev)
            if reg in SKIP_REGIONS:
                continue
            best = cluster_add(self._open, p, t, reg, self.dwell_radius)
            if not any(best is c for c in touched):
                touched.append(best)
        for c in touched:
            dur = c["t1"] - c["t0"]
            if (dur >= self.min_s and cluster_region(c) == "UNDER_HAND"
                    and (c.get("last_warn") is None or t - c["last_warn"] >= self.repeat_s)):
                c["last_warn"] = t
                events.append({"t": t, "dur_s": dur, "region": "UNDER_HAND",
                               "local_mean": tuple(float(v) for v in c["sum"] / c["n"]),
                               "hand_rev": float(hand_rev)})
        return events
