#!/usr/bin/env python3
"""Kinematic-calibration capture: the mocap pose sweep the geometry fit reads (OPERATOR-run).

Plan: ``plans/active/kinematic-calibration.md`` (§ 5 the sweep, § 3 the
two-direction repeats, § 4 the re-home subset, § 6 step 2 this tool). The fit
is ``tools/kincal_fit.py``; its module docstring is the CSV contract this tool
writes.

What it does
------------
Drives the platform through a generated list of poses with
``trajectory/go_to_pose``, dwells at each, and records one row per dwell: the
mocap ``Platform`` body pose (position mm and attitude, **Base frame = QTM
global**) and the six leg **encoder** positions (``/robot_state``
``motor_states[0..5].pos_estimate``, revolutions, positive = extension). The
commanded pose is recorded alongside but the fit never reads it: servo tracking
error must not leak into the geometry (§ 2).

Modes::

    --dry-run     generate the sweep, check every pose AND every move against
                  the stroke margin and the production move gate
                  (``planner.build_move``, the same code go_to_pose runs), print
                  every refusal at once plus the ETA, write the plan CSV. No ROS.
    --rehearse    the dry-run, plus a live preflight against the running stack
                  (topics flowing, Platform body tracked, mode, wire, hand
                  parked). Every problem is reported at once. No motion.
    (default)     the capture.
    --check       the short per-session homing check (§ 4, ~8 poses, under a
                  minute), for ``kincal_fit.py --offsets-only``.

The sweep (§ 5)
---------------
Five z-levels over 100–250 mm (STOW-relative, the ``go_to_pose`` convention),
about 25 poses each, filling ``--fill`` (0.9) of the reachable radius in a
random direction, weighted toward the edge. **Every pose is tilted** 6–10° in a
random direction, backing off toward 6° where 10° is out of reach: the fit
cannot otherwise separate the registration tilt from the six leg zero lengths
(§ 5.1). Reach is judged with the config geometry and a 5 mm stroke margin at
both ends, on the pose and on straight-line samples of every move into it.
About 10 poses are held out (``role = holdout``). About 10 poses get the § 3
repeats: approached twice from each of two opposite horizontal directions
through a via pose 30 mm away. A 10-pose subset is visited just before and just
after an operator-performed re-home (§ 4). Yaw is 0 throughout (``--yaw-deg``
widens it).

Frames — one conversion this tool owns
--------------------------------------
``mocap_node`` publishes non-Base bodies with z SHIFTED DOWN by
``GEOM_INITIAL_HEIGHT_MM`` (``mocap_interface.py``: ``z - base_to_platform``,
frame ``platform_start``). The fit wants QTM-global z, so this tool adds the
same constant back, read from the same generated config ``mocap_node``
imports, and records it in the CSV header and the meta. The calibration will
later CHANGE that constant (§ 6 step 5), which is exactly why the capture is
stored in the Base frame and never in ``platform_start``.

A **frame sanity check** runs at the first dwell, before anything else is
recorded: mocap position within ``FRAME_POS_TOL_MM`` of the commanded pose
(the known error is ~8.5 mm), attitude within ``FRAME_ATT_TOL_DEG``, and
encoder revolutions within ``FRAME_REV_TOL`` of the config IK. The first pose
is tilted, so a transposed or wrong-order quaternion (12°+ off), a missing z
shift (~500 mm off) or a leg sign error all abort here, not in the fit.

Safety posture
--------------
Request-only, like ``tilt_cal_grid.py``: this tool never arms, never changes
mode, never touches limits, never commands the hand. Homing is the operator's
(the tool pauses and says exactly what to do). Every accepted move is checked
for the ``DISARMED`` marker, and the cached ``/link_status`` is re-checked
through every dwell (``tilt_cal_grid.wire_armed_verdict``). A disarmed wire
would record a stationary platform against a whole sweep of commands.

Moves are requested at ``lean_gain = 0`` (OFF) with a slow duration (at least
``--min-move-s``, 60 mm/s, 4°/s). Lean shapes the transit only, never the
terminal pose; turning it off makes the dry-run's gate call the SAME plan the
node will build, so a rehearsal refusal list is exact. The slow duration keeps
the leg peaks far below the unshaped-traverse latch canary
(``logbook/2026-07-17-wobble-latch-unshaped-traverse.md``). Every exit path
returns to the centre pose (0, 0, 170, level) unless the wire has disarmed.

Usage (stack up, ACTIVE, TRAJECTORY mode, ARMED; hand parked; QTM tracking the
``Platform`` and ``Base`` bodies)::

    python3 tests/hardware/kincal_capture.py --dry-run
    python3 tests/hardware/kincal_capture.py --rehearse
    python3 tests/hardware/kincal_capture.py 2>&1 | tee temp/logs/kincal_capture_console.log
    python3 tests/hardware/kincal_capture.py --check

Runs under the system python3.8 with ROS 2 sourced (like every script here),
or the project venv for ``--dry-run``. Output: ``temp/logs/kincal_<mode>_<ts>.csv``
plus ``_meta.json``; rows are appended and flushed as they are taken, so an
abort keeps everything recorded so far.

The pure core (generation, checks, reduction, CSV) imports no ROS and is tested
in ``tests/sim/test_kincal_capture.py``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field, replace
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (os.path.join(_REPO_ROOT, 'tools'), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np  # noqa: E402  (kincal_fit pins OPENBLAS threads first)
import kincal_fit as kf  # noqa: E402

# ═════════════════════════════════════════════════════════════════════════════
# Constants
# ═════════════════════════════════════════════════════════════════════════════

Z_LEVELS_MM = (100.0, 137.5, 175.0, 212.5, 250.0)
POSES_PER_LEVEL = 25
TILT_MIN_DEG = 6.0
TILT_MAX_DEG = 10.0
BOX_MM = 300.0                 # the owner's ±300 mm x/y region
STROKE_MARGIN_MM = 5.0
FILL = 0.9
EDGE_EXPONENT = 0.4            # r = fill·r_max·u^0.4: weighted toward the edge
N_HOLDOUT = 10
N_REPEAT = 10
N_REHOME = 10
REPEATS_PER_DIRECTION = 2
VIA_OFFSET_MM = 30.0
PATH_SAMPLES = 11

CENTRE_Z_MM = 170.0            # JB_OP_DEFAULT_ACTIVE_Z_MM: where every exit goes
MIN_MOVE_S = 3.0
MOVE_SPEED_MMPS = 60.0
TILT_RATE_DEGPS = 4.0
SETTLE_S = 1.0
WINDOW_S = 1.0
MAX_WINDOWS = 3

# A dwell is static if its window is this still (encoder 0.003 rev ≈ 0.2 mm).
MOCAP_SPREAD_MM = 0.5
REV_SPREAD = 0.003
MIN_MOCAP_SAMPLES = 20
MIN_LEG_SAMPLES = 20
MOCAP_FRESH_S = 0.25

FRAME_POS_TOL_MM = 25.0
FRAME_ATT_TOL_DEG = 2.0
FRAME_REV_TOL = 0.3
HAND_PARKED_REV = 0.3

PLATFORM_BODY = 'Platform'
N_LEGS = 6
HAND_AXIS = 6


# ═════════════════════════════════════════════════════════════════════════════
# Poses and the sequence
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class Target:
    """One commanded pose. ``x/y/z`` mm with z STOW-relative (the go_to_pose
    convention); ``rv`` the attitude as a rotation vector, rad."""
    pose_id: str
    x: float
    y: float
    z: float
    rv: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    record: bool = True
    role: str = 'fit'
    repeat_group: str = ''
    direction: str = ''
    phase: str = ''

    @property
    def pose6(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, *self.rv], dtype=float)

    @property
    def tilt_deg(self) -> float:
        return math.degrees(math.hypot(self.rv[0], self.rv[1]))


@dataclass
class Step:
    """``kind`` is ``move`` (go to ``target``; record it if ``target.record``)
    or ``rehome`` (the operator re-homes; the tool waits)."""
    kind: str
    target: Optional[Target] = None


def centre_target() -> Target:
    return Target('centre', 0.0, 0.0, CENTRE_Z_MM, record=False)


class Reach:
    """Stroke-margin reach under the config geometry — the same IK the
    emitter ships (``tests/sim/test_kincal_fit.py`` pins the equality)."""

    def __init__(self, margin_mm: float = STROKE_MARGIN_MM):
        hw = kf._load_hw()
        self.g = kf.nominal_geometry()
        self.h0 = float(hw.GEOM_INITIAL_HEIGHT_MM)
        self.stroke = float(hw.GEOM_LEG_STROKE_MM)
        self.margin = float(margin_mm)

    def extensions(self, pose6s: np.ndarray) -> np.ndarray:
        p = np.atleast_2d(np.asarray(pose6s, dtype=float))
        c = p[:, :3] + np.array([0.0, 0.0, self.h0])
        R = np.array([kf._exp(rv) for rv in p[:, 3:6]])
        return kf.leg_lengths(c, R, self.g.base, self.g.plat) - self.g.L0

    def ok(self, pose6s: np.ndarray) -> np.ndarray:
        e = self.extensions(pose6s)
        return np.all((e >= self.margin) & (e <= self.stroke - self.margin), axis=1)

    def revs(self, pose6: np.ndarray) -> np.ndarray:
        return self.extensions(pose6)[0] * self.g.k

    def max_radius(self, z: float, phi: float, rv, lo: float = 0.0,
                   hi: float = 430.0, tol: float = 0.5) -> float:
        """Largest r along direction ``phi`` still reachable (0 if the axis
        point itself is out)."""
        def at(r):
            return np.array([r * math.cos(phi), r * math.sin(phi), z, *rv])
        if not self.ok(at(lo))[0]:
            return 0.0
        if self.ok(at(hi))[0]:
            return hi
        while hi - lo > tol:
            mid = 0.5 * (lo + hi)
            if self.ok(at(mid))[0]:
                lo = mid
            else:
                hi = mid
        return lo


def _tilt_rv(mag_deg: float, psi: float, yaw_deg: float = 0.0):
    m = math.radians(mag_deg)
    return (m * math.cos(psi), m * math.sin(psi), math.radians(yaw_deg))


def _draw_pose(rng, reach: Reach, z: float, pose_id: str, *, fill: float,
               tilt_min: float, tilt_max: float, yaw_max: float,
               box: float, tries: int = 200) -> Optional[Target]:
    for _ in range(tries):
        phi = rng.uniform(0.0, 2.0 * math.pi)
        psi = rng.uniform(0.0, 2.0 * math.pi)
        yaw = rng.uniform(-yaw_max, yaw_max) if yaw_max > 0 else 0.0
        mag = rng.uniform(tilt_min, tilt_max)
        # Back off toward tilt_min until the axis point is reachable.
        while True:
            rv = _tilt_rv(mag, psi, yaw)
            r_max = reach.max_radius(z, phi, rv)
            if r_max > 0.0 or mag <= tilt_min:
                break
            mag = max(tilt_min, mag - 1.0)
        if r_max <= 0.0:
            continue
        r = fill * r_max * rng.uniform() ** EDGE_EXPONENT
        x, y = r * math.cos(phi), r * math.sin(phi)
        s = min(1.0, box / max(abs(x), abs(y), 1e-9))
        t = Target(pose_id, x * s, y * s, z, rv)
        if reach.ok(t.pose6)[0]:
            return t
    return None


def _nn_order(targets: List[Target], start_xy=(0.0, 0.0)) -> List[Target]:
    left = list(targets)
    out = []
    cur = np.array(start_xy, dtype=float)
    while left:
        d = [np.hypot(t.x - cur[0], t.y - cur[1]) for t in left]
        t = left.pop(int(np.argmin(d)))
        out.append(t)
        cur = np.array([t.x, t.y])
    return out


def _spread_pick(rng, pool: List[Target], n: int) -> List[Target]:
    """``n`` targets spread over the list (one per equal slice, random inside)."""
    if n <= 0 or not pool:
        return []
    n = min(n, len(pool))
    edges = np.linspace(0, len(pool), n + 1).astype(int)
    return [pool[int(rng.integers(edges[i], max(edges[i] + 1, edges[i + 1])))]
            for i in range(n)]


def _via(reach: Reach, t: Target, u: np.ndarray, offset: float) -> Optional[Target]:
    for off in (offset, offset / 2.0, offset / 3.0):
        v = replace(t, pose_id=t.pose_id + '_via', x=t.x + off * u[0],
                    y=t.y + off * u[1], record=False, role='fit',
                    repeat_group='', direction='', phase='')
        if reach.ok(v.pose6)[0]:
            return v
    return None


def generate_sweep(seed: int = 1, *, reach: Optional[Reach] = None,
                   z_levels: Sequence[float] = Z_LEVELS_MM,
                   per_level: int = POSES_PER_LEVEL, fill: float = FILL,
                   tilt_min: float = TILT_MIN_DEG, tilt_max: float = TILT_MAX_DEG,
                   yaw_max: float = 0.0, box: float = BOX_MM,
                   n_holdout: int = N_HOLDOUT, n_repeat: int = N_REPEAT,
                   n_rehome: int = N_REHOME,
                   via_offset: float = VIA_OFFSET_MM) -> List[Step]:
    """The § 5 sweep as an ordered step list (see the module docstring)."""
    rng = np.random.default_rng(seed)
    reach = reach or Reach()
    main: List[Target] = []
    for z in z_levels:
        level = []
        for i in range(per_level):
            t = _draw_pose(rng, reach, float(z), 'z%03d_%02d' % (round(z), i),
                           fill=fill, tilt_min=tilt_min, tilt_max=tilt_max,
                           yaw_max=yaw_max, box=box)
            if t is not None:
                level.append(t)
        start = (main[-1].x, main[-1].y) if main else (0.0, 0.0)
        main.extend(_nn_order(level, start))

    for t in _spread_pick(rng, main, n_holdout):
        t.role = 'holdout'
    fit_pool = [t for t in main if t.role == 'fit']

    steps = [Step('move', t) for t in main]

    # § 3 repeats: two opposite horizontal directions, REPEATS_PER_DIRECTION each.
    # Candidates in spread order first, then the rest: a pose whose vias are out
    # of reach falls through to the next one rather than shrinking the set.
    first = _spread_pick(rng, fit_pool, n_repeat)
    rest = [fit_pool[i] for i in rng.permutation(len(fit_pool))
            if fit_pool[i] not in first]
    k = 0
    for t in first + rest:
        if k >= n_repeat:
            break
        a = rng.uniform(0.0, 2.0 * math.pi)
        u = np.array([math.cos(a), math.sin(a)])
        vias = {'A': _via(reach, t, u, via_offset), 'B': _via(reach, t, -u, via_offset)}
        if vias['A'] is None or vias['B'] is None:
            continue
        k += 1
        grp = 'R%02d' % k
        for _ in range(REPEATS_PER_DIRECTION):
            for d in ('A', 'B'):
                steps.append(Step('move', vias[d]))
                steps.append(Step('move', replace(t, repeat_group=grp, direction=d)))

    # § 4 re-home: the same subset just before and just after a re-home.
    sub = _nn_order(_spread_pick(rng, fit_pool, n_rehome),
                    (steps[-1].target.x, steps[-1].target.y))
    if sub:
        steps += [Step('move', replace(t, phase='pre_home')) for t in sub]
        steps.append(Step('rehome'))
        steps += [Step('move', replace(t, phase='post_home')) for t in sub]
    return steps


def generate_check(seed: int = 1, *, reach: Optional[Reach] = None,
                   n: int = 8, z: float = CENTRE_Z_MM) -> List[Step]:
    """The § 4 per-session check: ``n`` tilted poses at one height."""
    rng = np.random.default_rng(seed)
    reach = reach or Reach()
    out = []
    for i in range(n):
        t = _draw_pose(rng, reach, z, 'chk_%02d' % i, fill=0.6,
                       tilt_min=TILT_MIN_DEG, tilt_max=TILT_MIN_DEG + 2.0,
                       yaw_max=0.0, box=BOX_MM)
        if t is not None:
            out.append(t)
    return [Step('move', t) for t in _nn_order(out)]


# ═════════════════════════════════════════════════════════════════════════════
# Checks — all refusals at once
# ═════════════════════════════════════════════════════════════════════════════

def move_duration_s(a: Target, b: Target, *, min_s: float = MIN_MOVE_S,
                    speed: float = MOVE_SPEED_MMPS,
                    tilt_rate: float = TILT_RATE_DEGPS) -> float:
    d = float(np.linalg.norm(b.pose6[:3] - a.pose6[:3]))
    ang = math.degrees(float(np.linalg.norm(
        np.asarray(b.rv) - np.asarray(a.rv))))
    return max(min_s, d / speed, ang / tilt_rate)


def _moves(steps: Sequence[Step]) -> List[Tuple[int, Target, Target]]:
    """(step index, from, to) for every move, starting from the centre pose.
    A re-home puts the platform back at the centre (ACTIVE)."""
    out = []
    cur = centre_target()
    for i, s in enumerate(steps):
        if s.kind == 'rehome':
            cur = centre_target()
            continue
        out.append((i, cur, s.target))
        cur = s.target
    out.append((len(steps), cur, centre_target()))
    return out


def reach_problems(steps: Sequence[Step], reach: Reach,
                   samples: int = PATH_SAMPLES) -> List[str]:
    """Every pose and every straight-line move sample outside the stroke margin."""
    probs = []
    for i, a, b in _moves(steps):
        s = np.linspace(0.0, 1.0, samples)[:, None]
        path = a.pose6[None, :] * (1.0 - s) + b.pose6[None, :] * s
        bad = ~reach.ok(path)
        if bad.any():
            e = reach.extensions(path[bad])
            probs.append('step %d %s -> %s: %d/%d path samples outside the %.0f mm '
                         'stroke margin (extension %.1f..%.1f mm)'
                         % (i, a.pose_id, b.pose_id, int(bad.sum()), samples,
                            reach.margin, e.min(), e.max()))
    return probs


def gate_problems(steps: Sequence[Step], limits=None) -> List[str]:
    """Run the production move gate on every move at the duration the capture
    will request — ``planner.build_move`` is exactly what ``go_to_pose`` runs,
    and lean is OFF on both sides, so this list is what the node would refuse.
    """
    from jugglebot.motion.trajectory.planner import build_move
    from jugglebot.motion.trajectory.limits import TrajectoryLimits
    from jugglebot.motion.trajectory.feasibility import TrajectoryInfeasible
    from jugglebot.motion.geometry import StewartGeometry
    import jugglebot.hardware_config as hw
    lim = limits or TrajectoryLimits.from_config(hw)
    geom = StewartGeometry()
    z6 = np.zeros(6)
    probs = []
    for i, a, b in _moves(steps):
        T = move_duration_s(a, b)
        try:
            build_move((a.pose6, z6, z6), b.pose6, T, lim, geom, shaper=None)
        except TrajectoryInfeasible as e:
            probs.append('step %d %s -> %s at %.1f s: %s %s'
                         % (i, a.pose_id, b.pose_id, T, e.code, e))
    return probs


def sequence_summary(steps: Sequence[Step]) -> Dict[str, object]:
    rec = [s.target for s in steps if s.kind == 'move' and s.target.record]
    moves = _moves(steps)
    t_move = sum(move_duration_s(a, b) for _, a, b in moves)
    t_dwell = len(rec) * (SETTLE_S + WINDOW_S) + (
        len(moves) - len(rec)) * 0.5
    return {
        'recorded': len(rec),
        'moves': len(moves),
        'holdout': sum(t.role == 'holdout' for t in rec),
        'repeat_groups': len(set(t.repeat_group for t in rec if t.repeat_group)),
        'pre_home': sum(t.phase == 'pre_home' for t in rec),
        'post_home': sum(t.phase == 'post_home' for t in rec),
        'rehomes': sum(s.kind == 'rehome' for s in steps),
        'tilt_deg_min': min((t.tilt_deg for t in rec), default=0.0),
        'tilt_deg_max': max((t.tilt_deg for t in rec), default=0.0),
        'eta_min': (t_move + t_dwell) / 60.0,
    }


# ═════════════════════════════════════════════════════════════════════════════
# Dwell reduction and the frame sanity check
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class Dwell:
    pos: np.ndarray            # (3,) Base frame mm
    quat: np.ndarray           # (4,) wxyz, body -> Base
    rev: np.ndarray            # (6,)
    n_mocap: int
    n_leg: int
    mocap_spread_mm: float
    rev_spread: float
    problems: List[str] = field(default_factory=list)


def mean_quat_wxyz(q: np.ndarray) -> np.ndarray:
    """Sign-aligned normalised mean — exact enough over a static window."""
    q = np.atleast_2d(np.asarray(q, dtype=float))
    ref = q[0]
    s = np.where((q @ ref) < 0.0, -1.0, 1.0)[:, None]
    m = (q * s).sum(axis=0)
    m = m / np.linalg.norm(m)
    return m if m[0] >= 0.0 else -m


def reduce_dwell(mocap_pos: np.ndarray, mocap_quat: np.ndarray,
                 revs: np.ndarray, *, min_mocap: int = MIN_MOCAP_SAMPLES,
                 min_leg: int = MIN_LEG_SAMPLES,
                 mocap_spread: float = MOCAP_SPREAD_MM,
                 rev_spread: float = REV_SPREAD) -> Dwell:
    """Means over one dwell window, with every reason it is not usable."""
    P = np.asarray(mocap_pos, dtype=float).reshape(-1, 3)
    Q = np.asarray(mocap_quat, dtype=float).reshape(-1, 4)
    V = np.asarray(revs, dtype=float).reshape(-1, N_LEGS)
    probs = []
    if len(P) < min_mocap:
        probs.append('%d mocap Platform samples (need %d) — body occluded or '
                     'untracked?' % (len(P), min_mocap))
    if len(V) < min_leg:
        probs.append('%d /robot_state samples (need %d)' % (len(V), min_leg))
    if len(P) == 0 or len(V) == 0:
        nan = np.full(3, np.nan)
        return Dwell(nan, np.full(4, np.nan), np.full(N_LEGS, np.nan),
                     len(P), len(V), float('nan'), float('nan'), probs)
    ms = float(np.max(np.linalg.norm(P - P.mean(axis=0), axis=1)) * 2.0)
    rs = float(np.max(V.max(axis=0) - V.min(axis=0)))
    if not np.all(np.isfinite(P)) or not np.all(np.isfinite(V)):
        probs.append('non-finite samples in the window')
    if ms > mocap_spread:
        probs.append('mocap spread %.2f mm > %.2f (still moving?)' % (ms, mocap_spread))
    if rs > rev_spread:
        probs.append('encoder spread %.4f rev > %.4f (still moving?)' % (rs, rev_spread))
    return Dwell(P.mean(axis=0), mean_quat_wxyz(Q), V.mean(axis=0),
                 len(P), len(V), ms, rs, probs)


def frame_problems(t: Target, d: Dwell, reach: Reach, *,
                   pos_tol: float = FRAME_POS_TOL_MM,
                   att_tol: float = FRAME_ATT_TOL_DEG,
                   rev_tol: float = FRAME_REV_TOL) -> List[str]:
    """Is the recorded dwell in the frames the fit assumes? (module docstring)"""
    probs = []
    cmd_c = t.pose6[:3] + np.array([0.0, 0.0, reach.h0])
    dp = d.pos - cmd_c
    if not np.linalg.norm(dp) <= pos_tol:
        probs.append('mocap Platform is %s mm from the commanded pose in the Base '
                     'frame (tolerance %.0f): a missing z shift, a wrong body, or a '
                     'QTM frame that is not the Base'
                     % (np.round(dp, 1).tolist(), pos_tol))
    R_cmd = kf._exp(np.asarray(t.rv))
    R_m = kf.quat_wxyz_to_matrix(d.quat[None])[0]
    ang = math.degrees(float(np.linalg.norm(
        kf.Rotation.from_matrix(R_cmd.T @ R_m).as_rotvec())))
    if not ang <= att_tol:
        probs.append('mocap attitude is %.2f deg from the commanded %.1f deg tilt '
                     '(tolerance %.1f): quaternion order/transpose, or the body '
                     'axes are not parallel to the Base' % (ang, t.tilt_deg, att_tol))
    dr = d.rev - reach.revs(t.pose6)
    if not np.max(np.abs(dr)) <= rev_tol:
        probs.append('encoder revs differ from the config IK by %s rev (tolerance '
                     '%.2f): leg order, sign, or homing'
                     % (np.round(dr, 3).tolist(), rev_tol))
    return probs


# ═════════════════════════════════════════════════════════════════════════════
# CSV
# ═════════════════════════════════════════════════════════════════════════════

EXTRA_COLS = (['t_iso', 'cmd_x_mm', 'cmd_y_mm', 'cmd_z_mm', 'cmd_rx_rad',
               'cmd_ry_rad', 'cmd_rz_rad', 'n_mocap', 'n_leg', 'mocap_spread_mm',
               'rev_spread', 'hand_rev', 'uptime_ms'])
CSV_COLS = (['pose_id', 'role', 'repeat_group', 'direction', 'phase',
             'x_mm', 'y_mm', 'z_mm', 'qw', 'qx', 'qy', 'qz']
            + ['rev_%d' % i for i in range(N_LEGS)] + EXTRA_COLS)


def csv_row(t: Target, d: Dwell, *, hand_rev=float('nan'), uptime_ms='') -> list:
    return ([t.pose_id, t.role, t.repeat_group, t.direction, t.phase]
            + ['%.4f' % v for v in d.pos] + ['%.9f' % v for v in d.quat]
            + ['%.6f' % v for v in d.rev]
            + [datetime.now().isoformat(timespec='milliseconds')]
            + ['%.4f' % v for v in t.pose6[:3]] + ['%.7f' % v for v in t.pose6[3:]]
            + [d.n_mocap, d.n_leg, '%.3f' % d.mocap_spread_mm,
               '%.5f' % d.rev_spread, '%.4f' % hand_rev, uptime_ms])


class CaptureWriter:
    """Append-and-flush writer; ``kincal_fit.read_capture`` skips '#' lines."""

    def __init__(self, path: str, header: Dict[str, object]):
        self.path = path
        self.fh = open(path, 'w', newline='', encoding='utf-8')
        for k, v in header.items():
            self.fh.write('# %s: %s\n' % (k, v))
        self.w = csv.writer(self.fh)
        self.w.writerow(CSV_COLS)
        self.fh.flush()
        self.rows = 0

    def write(self, row: list) -> None:
        self.w.writerow(row)
        self.fh.flush()
        self.rows += 1

    def close(self) -> None:
        self.fh.close()


def write_plan(path: str, steps: Sequence[Step]) -> None:
    with open(path, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['step', 'kind', 'pose_id', 'record', 'role', 'repeat_group',
                    'direction', 'phase', 'x_mm', 'y_mm', 'z_mm', 'rx_rad',
                    'ry_rad', 'rz_rad', 'tilt_deg'])
        for i, s in enumerate(steps):
            if s.kind == 'rehome':
                w.writerow([i, 'rehome'] + [''] * 13)
                continue
            t = s.target
            w.writerow([i, 'move', t.pose_id, int(t.record), t.role, t.repeat_group,
                        t.direction, t.phase] + ['%.3f' % v for v in t.pose6[:3]]
                       + ['%.6f' % v for v in t.pose6[3:]] + ['%.2f' % t.tilt_deg])


# ═════════════════════════════════════════════════════════════════════════════
# Live preflight (pure verdict over cached state)
# ═════════════════════════════════════════════════════════════════════════════

def preflight_problems(*, link_kv: Optional[Dict[str, str]], status_mode: Optional[str],
                       robot_state_age_s: Optional[float],
                       leg_errors: Sequence[int], hand_rev: Optional[float],
                       platform_age_s: Optional[float],
                       service_ok: bool) -> List[str]:
    """Every reason the capture cannot start, at once."""
    from tilt_cal_grid import wire_armed_verdict
    probs = []
    if not service_ok:
        probs.append('/trajectory/go_to_pose is not available (stack up? rebuilt?)')
    ok, msg = wire_armed_verdict(link_kv)
    if not ok:
        probs.append('wire: ' + msg)
    if status_mode is None:
        probs.append('no /trajectory/status seen (trajectory_node not running?)')
    elif status_mode != 'TRAJECTORY':
        probs.append('mode is %s: publish "trajectory" on /orchestrator_command '
                     'from ACTIVE' % status_mode)
    if robot_state_age_s is None or robot_state_age_s > 0.5:
        probs.append('/robot_state not flowing (age %s)' % robot_state_age_s)
    bad = [i for i, e in enumerate(leg_errors) if e]
    if bad:
        probs.append('ODrive errors on axes %s' % bad)
    if hand_rev is None:
        probs.append('hand position unknown (no motor_states[6])')
    elif abs(hand_rev) > HAND_PARKED_REV:
        probs.append('hand at %.2f rev — park it (plan § 5: hand parked, disarmed)'
                     % hand_rev)
    if platform_age_s is None:
        probs.append('mocap body "%s" never seen on /rigid_body_poses (QTM '
                     'running? body enabled?)' % PLATFORM_BODY)
    elif platform_age_s > MOCAP_FRESH_S:
        probs.append('mocap body "%s" last seen %.2f s ago (occluded?)'
                     % (PLATFORM_BODY, platform_age_s))
    return probs


# ═════════════════════════════════════════════════════════════════════════════
# ROS layer
# ═════════════════════════════════════════════════════════════════════════════

class CaptureAbort(RuntimeError):
    pass


class _Ros:
    """rclpy client: one service, four subscriptions. Nothing here arms, changes
    mode, sets limits or commands the hand."""

    def __init__(self, node, timeout_s: float):
        from jugglebot_interfaces.srv import GoToPose
        from jugglebot_interfaces.msg import (TrajectoryStatus, RobotState,
                                              RigidBodyPoses)
        from diagnostic_msgs.msg import DiagnosticStatus
        self.node = node
        self.timeout_s = timeout_s
        self._GoToPose = GoToPose
        self.cli_go = node.create_client(GoToPose, '/trajectory/go_to_pose')
        self.status = None
        self.link_kv: Dict[str, str] = {}
        self.uptime_ms = ''
        self.rs_t = None
        self.leg_errors = [0] * N_LEGS
        self.hand_rev = None
        self.plat_t = None
        self.buf_mocap: List[Tuple[np.ndarray, np.ndarray]] = []
        self.buf_rev: List[np.ndarray] = []
        self.collecting = False
        node.create_subscription(TrajectoryStatus, '/trajectory/status',
                                 self._on_status, 10)
        node.create_subscription(DiagnosticStatus, '/link_status', self._on_link, 10)
        node.create_subscription(RobotState, '/robot_state', self._on_rs, 50)
        node.create_subscription(RigidBodyPoses, '/rigid_body_poses',
                                 self._on_bodies, 50)

    def _on_status(self, msg):
        self.status = msg

    def _on_link(self, msg):
        self.link_kv = {v.key: v.value for v in msg.values}
        self.uptime_ms = self.link_kv.get('uptime_ms', '')

    def _on_rs(self, msg):
        ms = list(msg.motor_states)
        if len(ms) < N_LEGS:
            return
        self.rs_t = time.monotonic()
        self.leg_errors = [int(ms[i].active_errors) | int(ms[i].disarm_reason)
                           for i in range(N_LEGS)]
        if len(ms) > HAND_AXIS:
            self.hand_rev = float(ms[HAND_AXIS].pos_estimate)
        if self.collecting:
            self.buf_rev.append(np.array([ms[i].pos_estimate for i in range(N_LEGS)],
                                         dtype=float))

    def _on_bodies(self, msg):
        for b in msg.bodies:
            if b.name != PLATFORM_BODY:
                continue
            p, o = b.pose.pose.position, b.pose.pose.orientation
            vals = (p.x, p.y, p.z, o.w, o.x, o.y, o.z)
            if not all(math.isfinite(v) for v in vals):
                return
            self.plat_t = time.monotonic()
            if self.collecting:
                self.buf_mocap.append((np.array([p.x, p.y, p.z]),
                                       np.array([o.w, o.x, o.y, o.z])))

    def spin(self, seconds: float) -> None:
        import rclpy
        end = time.monotonic() + max(0.0, seconds)
        while True:
            rem = end - time.monotonic()
            if rem <= 0.0:
                break
            rclpy.spin_once(self.node, timeout_sec=min(0.05, rem))

    def _age(self, t):
        return None if t is None else round(time.monotonic() - t, 3)

    def preflight(self) -> List[str]:
        svc = self.cli_go.wait_for_service(timeout_sec=self.timeout_s)
        self.spin(1.0)
        return preflight_problems(
            link_kv=self.link_kv,
            status_mode=getattr(self.status, 'mode', None),
            robot_state_age_s=self._age(self.rs_t), leg_errors=self.leg_errors,
            hand_rev=self.hand_rev, platform_age_s=self._age(self.plat_t),
            service_ok=svc)

    def wire_check(self) -> None:
        from tilt_cal_grid import wire_armed_verdict
        ok, msg = wire_armed_verdict(self.link_kv)
        if not ok:
            raise CaptureAbort('wire: ' + msg)
        bad = [i for i, e in enumerate(self.leg_errors) if e]
        if bad:
            raise CaptureAbort('ODrive errors on axes %s' % bad)

    def go(self, t: Target, duration_s: float) -> float:
        """Request the move; return the planned duration. Raises on refusal."""
        import rclpy
        from tilt_cal_grid import response_reports_disarmed
        req = self._GoToPose.Request()
        req.pose.position.x, req.pose.position.y, req.pose.position.z = (
            float(t.x), float(t.y), float(t.z))
        q = kf.matrix_to_quat_wxyz(kf._exp(np.asarray(t.rv))[None])[0]
        (req.pose.orientation.w, req.pose.orientation.x,
         req.pose.orientation.y, req.pose.orientation.z) = (float(v) for v in q)
        req.lean_gain = 0.0
        for attempt in (0, 1):
            req.duration_s = float(duration_s)
            fut = self.cli_go.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=self.timeout_s)
            res = fut.result()
            if res is None:
                raise CaptureAbort('go_to_pose did not answer in %.1f s' % self.timeout_s)
            if res.accepted:
                if response_reports_disarmed(res.message):
                    raise CaptureAbort('go_to_pose accepted on a DISARMED wire: %s'
                                       % res.message)
                return float(res.planned_duration_s)
            if res.code == 'TOO_FAST' and attempt == 0 and res.min_duration_s > 0:
                duration_s = 1.2 * float(res.min_duration_s)
                continue
            raise CaptureAbort('go_to_pose refused %s: %s %s'
                               % (t.pose_id, res.code, res.message))
        raise CaptureAbort('unreachable')

    def wait_arrival(self, planned_s: float, settle_s: float) -> None:
        """Sleep through the move with the wire watched, then until the node
        reports a hold, then the settle."""
        end = time.monotonic() + planned_s
        while time.monotonic() < end:
            self.spin(min(0.2, end - time.monotonic()))
            self.wire_check()
        deadline = time.monotonic() + 5.0
        while getattr(self.status, 'plan_kind', 'hold') != 'hold':
            if time.monotonic() > deadline:
                raise CaptureAbort('no hold 5 s after the planned end (plan_kind=%s)'
                                   % getattr(self.status, 'plan_kind', None))
            self.spin(0.1)
        self.spin(settle_s)
        self.wire_check()

    def sample(self, window_s: float, z_shift_mm: float) -> Dwell:
        d = None
        for _ in range(MAX_WINDOWS):
            self.buf_mocap, self.buf_rev = [], []
            self.collecting = True
            self.spin(window_s)
            self.collecting = False
            self.wire_check()
            P = np.array([p for p, _ in self.buf_mocap]).reshape(-1, 3)
            P = P + np.array([0.0, 0.0, z_shift_mm])
            Q = np.array([q for _, q in self.buf_mocap]).reshape(-1, 4)
            d = reduce_dwell(P, Q, np.array(self.buf_rev).reshape(-1, N_LEGS))
            if not d.problems:
                return d
        return d


def _operator_rehome(ros: _Ros) -> None:
    print('\n' + '=' * 72)
    print('RE-HOME (plan § 4). The platform is at the centre pose. At the robot:')
    print('  1. /orchestrator_command "deactivate"  (platform to STOW)')
    print('  2. /orchestrator_command "home"        (wait for IDLE, is_homed)')
    print('  3. /orchestrator_command "activate", then "trajectory", then arm')
    print('     as usual; park the hand.')
    print('Then press Enter here. The tool re-runs the preflight until it passes.')
    print('=' * 72)
    while True:
        input('[Enter when re-homed and armed] ')
        probs = ros.preflight()
        if not probs:
            return
        print('Not ready yet:')
        for p in probs:
            print('  - ' + p)


def run_capture(steps: List[Step], args, reach: Reach, out_stem: str) -> int:
    import rclpy
    rclpy.init()
    node = rclpy.create_node('kincal_capture')
    ros = _Ros(node, args.timeout_s)
    meta = {'mode': args.mode, 'seed': args.seed, 'started': datetime.now().isoformat(),
            'mocap_z_shift_mm': reach.h0, 'summary': sequence_summary(steps),
            'skipped': [], 'abort_reason': None}
    writer = None
    cur = centre_target()
    rc = 1
    try:
        probs = ros.preflight()
        if probs:
            for p in probs:
                print('PREFLIGHT: ' + p)
            meta['abort_reason'] = 'preflight: ' + '; '.join(probs)
            return 2
        writer = CaptureWriter(out_stem + '.csv', {
            'tool': 'tests/hardware/kincal_capture.py', 'mode': args.mode,
            'seed': args.seed, 'frame': 'Base = QTM global; z = published + shift',
            'mocap_z_shift_mm': reach.h0, 'started': meta['started']})
        # Start from the centre so the first move is the one the dry-run checked.
        planned = ros.go(centre_target(), MIN_MOVE_S)
        ros.wait_arrival(planned, 0.2)
        n_rec = sum(1 for s in steps if s.kind == 'move' and s.target.record)
        k = 0
        framed = False
        t0 = time.monotonic()
        for s in steps:
            if s.kind == 'rehome':
                planned = ros.go(centre_target(), move_duration_s(cur, centre_target()))
                ros.wait_arrival(planned, 0.2)
                cur = centre_target()
                _operator_rehome(ros)
                continue
            t = s.target
            planned = ros.go(t, move_duration_s(cur, t))
            cur = t
            ros.wait_arrival(planned, args.settle_s if t.record else 0.3)
            if not t.record:
                continue
            d = ros.sample(args.window_s, reach.h0)
            k += 1
            if d.problems:
                meta['skipped'].append({'pose_id': t.pose_id, 'phase': t.phase,
                                        'repeat_group': t.repeat_group,
                                        'problems': d.problems})
                print('[%3d/%d] %-10s SKIPPED: %s' % (k, n_rec, t.pose_id,
                                                      '; '.join(d.problems)))
                continue
            if not framed:
                fp = frame_problems(t, d, reach)
                if fp:
                    raise CaptureAbort('frame sanity check failed at %s:\n  - %s'
                                       % (t.pose_id, '\n  - '.join(fp)))
                framed = True
            writer.write(csv_row(t, d, hand_rev=ros.hand_rev or float('nan'),
                                 uptime_ms=ros.uptime_ms))
            el = time.monotonic() - t0
            print('[%3d/%d] %-10s %-7s %s%s  err %5.1f mm  spread %.2f mm  %4.1f min'
                  % (k, n_rec, t.pose_id, t.role, t.phase or '',
                     (' %s%s' % (t.repeat_group, t.direction)) if t.repeat_group else '',
                     float(np.linalg.norm(d.pos - t.pose6[:3]
                                          - np.array([0, 0, reach.h0]))),
                     d.mocap_spread_mm, el / 60.0))
        rc = 0
    except CaptureAbort as e:
        meta['abort_reason'] = str(e)
        print('\nABORT: %s' % e)
    except KeyboardInterrupt:
        meta['abort_reason'] = 'KeyboardInterrupt'
        print('\nInterrupted.')
    finally:
        from tilt_cal_grid import wire_armed_verdict
        ok, why = wire_armed_verdict(ros.link_kv)
        if meta['abort_reason'] is not None and meta['abort_reason'].startswith('preflight'):
            pass
        elif ok:
            try:
                planned = ros.go(centre_target(), move_duration_s(cur, centre_target()))
                ros.wait_arrival(planned, 0.2)
                print('Returned to centre.')
            except Exception as e:  # noqa: BLE001 — report, never mask the abort
                print('RETURN TO CENTRE FAILED (%s) — bring it home manually.' % e)
        else:
            print('RETURN TO CENTRE REFUSED: %s — bring it home manually.' % why)
        if writer is not None:
            writer.close()
            meta['rows'] = writer.rows
            print('Wrote %d rows: %s' % (writer.rows, writer.path))
        meta['ended'] = datetime.now().isoformat()
        with open(out_stem + '_meta.json', 'w', encoding='utf-8') as fh:
            json.dump(meta, fh, indent=2, default=str)
        node.destroy_node()
        rclpy.shutdown()
    if rc == 0:
        print('Next: python tools/kincal_fit.py %s.csv%s' % (
            out_stem, ' --offsets-only --geometry <fitted proposed_geometry.yaml>'
            if args.mode == 'check' else ''))
    return rc


# ═════════════════════════════════════════════════════════════════════════════
# CLI
# ═════════════════════════════════════════════════════════════════════════════

def _print_summary(steps: Sequence[Step]) -> None:
    s = sequence_summary(steps)
    print('Sequence: %(recorded)d recorded dwells over %(moves)d moves; '
          '%(holdout)d hold-out, %(repeat_groups)d repeat groups, '
          '%(pre_home)d pre-home + %(post_home)d post-home, %(rehomes)d re-home' % s)
    print('Tilt %.1f..%.1f deg; ETA %.1f min (excluding the re-home)'
          % (s['tilt_deg_min'], s['tilt_deg_max'], s['eta_min']))


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    mx = ap.add_mutually_exclusive_group()
    mx.add_argument('--dry-run', action='store_true')
    mx.add_argument('--rehearse', action='store_true')
    ap.add_argument('--check', action='store_true',
                    help='the short per-session homing check (§ 4)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--per-level', type=int, default=POSES_PER_LEVEL)
    ap.add_argument('--yaw-deg', type=float, default=0.0)
    ap.add_argument('--no-gate', action='store_true',
                    help='dry-run: skip the production move gate (reach only)')
    ap.add_argument('--settle-s', type=float, default=SETTLE_S)
    ap.add_argument('--window-s', type=float, default=WINDOW_S)
    ap.add_argument('--timeout-s', type=float, default=10.0)
    ap.add_argument('--out-dir', default=os.path.join(_REPO_ROOT, 'temp', 'logs'))
    args = ap.parse_args(argv)
    args.mode = 'check' if args.check else 'sweep'

    reach = Reach()
    steps = (generate_check(args.seed, reach=reach) if args.check else
             generate_sweep(args.seed, reach=reach, per_level=args.per_level,
                            yaw_max=args.yaw_deg))
    _print_summary(steps)
    probs = reach_problems(steps, reach)
    if not args.no_gate:
        try:
            probs += gate_problems(steps)
        except ImportError as e:
            probs.append('production gate not importable (%s) — build jugglebot or '
                         'pass --no-gate' % e)
    for p in probs:
        print('REFUSED: ' + p)
    print('%d refusal(s).' % len(probs))

    os.makedirs(args.out_dir, exist_ok=True)
    stem = os.path.join(args.out_dir, 'kincal_%s_%s' % (
        args.mode, datetime.now().strftime('%Y%m%d_%H%M%S')))
    if args.dry_run or args.rehearse:
        write_plan(stem + '_plan.csv', steps)
        print('Plan: %s_plan.csv' % stem)
    if args.dry_run:
        return 0 if not probs else 1
    if args.rehearse:
        import rclpy
        rclpy.init()
        node = rclpy.create_node('kincal_rehearse')
        try:
            live = _Ros(node, args.timeout_s).preflight()
        finally:
            node.destroy_node()
            rclpy.shutdown()
        for p in live:
            print('PREFLIGHT: ' + p)
        print('Rehearsal: %d sequence refusal(s), %d preflight problem(s).'
              % (len(probs), len(live)))
        return 0 if not (probs or live) else 1
    if probs:
        print('Refusing to move: fix the sequence (or change --seed) first.')
        return 1
    return run_capture(steps, args, reach, stem)


if __name__ == '__main__':
    sys.exit(main())
