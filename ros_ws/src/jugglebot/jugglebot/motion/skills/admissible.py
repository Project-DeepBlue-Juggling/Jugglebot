"""The offline admissible box for a THROW command (plan § 2.2 / § 2.6).

R2 moves the two remaining per-cycle-time-critical gates on a THROW's command
``u = (landing_xy_m, apex_m)`` -- I-CATCH-3's closed-form quintic reach
frontier (the platform's transit between two sites) and ``REJECTED_DISPLACEMENT``
(the pre-throw A->B displacement cap) -- off the beat path entirely. Instead,
:mod:`tools.admissible_sweep` runs the REAL QP + gate
(``unified_cycle.plan_launch`` / ``plan_landing``, gated by
``feasibility.validate_cycle`` exactly as ``skills.segments.plan_segment``
calls it) over a grid of commands at the session limits, offline, and records
the box of ``u`` for which every grid point passed with margin. At runtime the
node only has to :func:`clip` the learner's (or, at R2, the identity prior's)
command into that box -- an O(1) bounds check, not a QP solve.

**Why a box and not a per-throw call.** The old per-cycle call
(``catch_reach.catch_reach_verdict``) cost 1.5-18.9 ms on the FSM tick
(``catch_reach.py``'s own measurement, 2026-08-29) and its own docstring notes
that cost "does not move the needle" only because the FSM tick already blocks
on a synchronous ``go_to_pose`` round trip -- a luxury the schedule-driven
skill stack does not have (every segment installs against a wall-clock
deadline). Swept offline once per limits change, the runtime cost collapses to
four float comparisons.

**What ships in this box, and what does not (C-HAND-3 / throw_envelope.py).**
``motion.trajectory.throw_envelope.evaluate`` is the ONE enforcement point for
C-HAND-3's six hand-metal bounds (END_STOP, DECEL_AUTHORITY, DECEL_FF_HEADROOM,
ACCEL_AUTHORITY, REGEN, WIRE_BAND) and this module does not re-derive any of
them -- it is a straight input, called once per (site pair, apex) grid cell on
the THROW's actual commanded release speed and flight time. That is a BOUNDED
check on takeoff speed only: it says nothing about how a landing-xy OFFSET
inside the box would change the release speed of a Tier-8b *displaced* throw
(a vertical self-toss's release speed is a function of flight time alone, so
at R2 -- columns only, ``Skill.y_d`` always ``(zeros(2), apex_m)`` -- the
offset never reaches the THROW's own release speed; it only ever perturbs the
CATCH's landing target). Wiring the full six-bound envelope against a
DISPLACED throw's release speed (a function of both the offset and the flight
time) is left for R3, when the learner's command can move the THROW's own
target off self.

Pure Python + numpy + PyYAML. No ROS2 imports (the ``motion/`` rule).
"""

from __future__ import annotations

import dataclasses
import hashlib
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

__all__ = [
    'AdmissibleBox', 'AdmissibleError', 'LimitsMismatch',
    'MARGIN_FRAC', 'PATTERNS', 'clip', 'dump', 'load', 'check_limits',
    'describe_miss', 'gate_hash', 'select',
]

#: The skill patterns an admissible box may be swept for (R4,
#: brief_U0_admissible.md § A). 'self_toss' and 'columns' both key
#: :attr:`AdmissibleBox.site_pair` as (release site name, release site name)
#: -- a columns throw's own release/target site IS the site a pre-throw
#: transit lands it at, baked into the swept segment (see the module
#: docstring's "Why" #1); 'hop' is the genuinely cross-site case, release
#: site name != target site name.
PATTERNS = ('self_toss', 'hop', 'columns')

#: Every grid point in the box must clear the session limit by this fraction
#: before it is admitted -- 10% headroom above the swept operating point for
#: solver tolerance and the 80-sample-vs-dense-400 mesh-coarsening gap
#: (I-PLAN-3) so a point the box calls "admissible" cannot be one the live
#: gate, sampled slightly differently on the day, refuses.
MARGIN_FRAC = 0.9

#: Required keys of the ``limits`` dict every box is stamped with -- the
#: session limits it was swept under (plan § 2.6).  Hand VELOCITY is
#: deliberately excluded: no swept segment here nears it (the hand's
#: constraining axis at these apexes is acceleration, per
#: ``tools/probes/skills_sizing_sweep.py``'s frontier runs).
_LIMIT_KEYS = ('leg_vel_mmps', 'leg_acc_mmps2', 'leg_jerk_mmps3', 'hand_acc_rps2')


class AdmissibleError(Exception):
    """The command / box combination cannot be resolved -- names the site pair."""


class LimitsMismatch(AdmissibleError):
    """A loaded box was swept under limits that differ from the live session's."""


def _is_empty_range(lo: float, hi: float) -> bool:
    return math.isnan(lo) or math.isnan(hi)


@dataclasses.dataclass(frozen=True)
class AdmissibleBox:
    """Per (pattern, site pair, apex band) bounds on a THROW command, plus the
    provenance a loader needs to refuse a stale sweep (plan § 2.6).

    An EMPTY box (no grid point in the sweep passed with margin) is
    represented by ``float('nan')`` on every bound of ``landing_xy_m`` and
    ``apex_m`` -- the same fail-closed sentinel
    ``throw_envelope._find_flight_band`` uses for "no admitted set", so a
    caller who forgets to check emptiness gets a loud ``nan`` comparison
    rather than a silently-passing bound.

    **R4: ``pattern`` + the two site xy stamps (2026-09-23).** ``site_pair``
    is ``(RELEASE site name, TARGET site name)`` -- THE key
    ``executor.SkillExecutor._command_u`` looks a box up by
    (``adm.select(self.boxes, self.schedule.pattern, (site.name,
    target.name), apex, ...)``). A box carries no other site geometry, so a
    box swept at one separation (e.g. columns' 100 mm) would otherwise be
    silently applied at another (a hop swept at 250 mm applied to a 100 mm
    schedule) -- :attr:`release_site_xy_mm` / :attr:`target_site_xy_mm` close
    that, and :func:`select` refuses unless both match the live sites to
    within 1e-6 mm. See ``tools/admissible_sweep.py``'s module docstring for
    why 'columns' and 'self_toss' both key ``site_pair`` as (X, X) while
    'hop' keys (release, target).
    """

    site_pair: Tuple[str, str]
    apex_band_m: Tuple[float, float]
    #: ``((xmin, xmax), (ymin, ymax))``, metres, relative to the target site.
    landing_xy_m: Tuple[Tuple[float, float], Tuple[float, float]]
    #: ``(lo, hi)`` bounds on the commanded APEX (m above the catch plane) --
    #: the third component of the command this box clips (2026-09-18; it was
    #: a flight time in seconds until the learner's outcome became an apex).
    #: Distinct from :attr:`apex_band_m`, which says which commands this box
    #: is the right one to LOOK UP, not what it clips them to.
    apex_m: Tuple[float, float]
    #: One of :data:`PATTERNS` -- which skill shape this box was swept for.
    pattern: str
    #: ``(x, y)`` mm, platform frame -- the RELEASE site's ``cup_mm[:2]`` this
    #: box was swept at (:func:`select`'s xy match).
    release_site_xy_mm: Tuple[float, float]
    #: ``(x, y)`` mm, platform frame -- the TARGET site's ``cup_mm[:2]``.
    target_site_xy_mm: Tuple[float, float]
    #: The session limits this box was swept under -- see ``_LIMIT_KEYS``.
    limits: Dict[str, float]
    #: sha256(feasibility.py text + segments.py text)[:12] -- see :func:`gate_hash`.
    gate_hash: str
    #: ISO date the sweep that produced this box was run.
    swept_at: str
    #: R5 (D4, 2026-09-30): the dwell (s) this box was swept at -- ``skill_node``'s
    #: ``dwell_s`` parameter (default 0.30) and ``admissible_sweep.DWELL_S`` were
    #: an UNENFORCED timing twin (plan § 0: "no second copy of any constant or
    #: timing") before this field existed -- a box swept at dwell 0.30 could be
    #: silently selected for a live session running at 0.20 with nothing to
    #: catch the mismatch, and the columns cell's admitted set is dwell-shaped
    #: (a shorter dwell widens the transit tau and loosens the catch-with-throw
    #: gate). ``None`` is a box built before this field existed -- tolerated
    #: (not refused) so the existing Python-constructed fixtures in
    #: ``tests/motion/test_skills_executor.py`` / ``tests/ros/test_skill_node.py``
    #: (out of this unit's file list, plan token-budget rule) keep working
    #: unstamped; :func:`check_limits`'s ``dwell_s`` kwarg is the ACTIVE check
    #: that matters (the operator-visible refusal at runtime) and treats a
    #: ``None`` box exactly like a numeric mismatch when a live dwell is given.
    dwell_s: Optional[float] = None

    def __post_init__(self):
        if (not isinstance(self.site_pair, tuple) or len(self.site_pair) != 2
                or not all(isinstance(s, str) and s for s in self.site_pair)):
            raise ValueError('site_pair must be a 2-tuple of non-empty strings,'
                              ' got %r' % (self.site_pair,))
        lo, hi = float(self.apex_band_m[0]), float(self.apex_band_m[1])
        if not (math.isfinite(lo) and math.isfinite(hi) and lo <= hi):
            raise ValueError('apex_band_m must be a finite (lo <= hi) pair, '
                              'got %r' % (self.apex_band_m,))
        (xlo, xhi), (ylo, yhi) = self.landing_xy_m
        xlo, xhi, ylo, yhi = float(xlo), float(xhi), float(ylo), float(yhi)
        for name, a, b in (('x', xlo, xhi), ('y', ylo, yhi)):
            if _is_empty_range(a, b):
                continue
            if not (math.isfinite(a) and math.isfinite(b) and a <= b):
                raise ValueError('landing_xy_m.%s must be a finite (lo <= hi) '
                                  'pair or the (nan, nan) empty sentinel, got '
                                  '%r' % (name, (a, b)))
        alo, ahi = float(self.apex_m[0]), float(self.apex_m[1])
        if not _is_empty_range(alo, ahi):
            if not (math.isfinite(alo) and math.isfinite(ahi) and alo <= ahi):
                raise ValueError('apex_m must be a finite (lo <= hi) pair or '
                                  'the (nan, nan) empty sentinel, got %r'
                                  % (self.apex_m,))
        # x/y and apex emptiness must agree -- a half-empty box is a bug in
        # whatever built it, not a physical state.
        if _is_empty_range(xlo, xhi) != _is_empty_range(alo, ahi):
            raise ValueError('landing_xy_m and apex_m must be empty (nan) '
                              'together or not at all, got landing_xy_m=%r '
                              'apex_m=%r' % (self.landing_xy_m, self.apex_m))
        if self.pattern not in PATTERNS:
            raise ValueError('pattern must be one of %r, got %r'
                              % (PATTERNS, self.pattern))
        for name, xy in (('release_site_xy_mm', self.release_site_xy_mm),
                         ('target_site_xy_mm', self.target_site_xy_mm)):
            x, y = float(xy[0]), float(xy[1])
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError('%s must be a finite (x, y) pair, got %r'
                                  % (name, xy))
        missing = [k for k in _LIMIT_KEYS if k not in self.limits]
        if missing:
            raise ValueError('limits is missing %r (required: %r)'
                              % (missing, _LIMIT_KEYS))
        if len(self.gate_hash) != 12 or not all(
                c in '0123456789abcdef' for c in self.gate_hash):
            raise ValueError('gate_hash must be 12 lowercase hex chars, got %r'
                              % (self.gate_hash,))
        if not isinstance(self.swept_at, str) or not self.swept_at:
            raise ValueError('swept_at must be a non-empty ISO date string, '
                              'got %r' % (self.swept_at,))
        if self.dwell_s is not None:
            dwell = float(self.dwell_s)
            if not (math.isfinite(dwell) and dwell > 0.0):
                raise ValueError('dwell_s must be None or a finite positive '
                                  'float, got %r' % (self.dwell_s,))

    @property
    def empty(self) -> bool:
        """True when no grid point in the sweep passed with margin."""
        return _is_empty_range(*self.apex_m)


#: The gated files, IN THE FIXED ORDER :func:`gate_hash` reads them --
#: everything a working-tree edit to which must invalidate a swept box,
#: because it changes what the real QP + gate (``unified_cycle.plan_launch``/
#: ``plan_landing``, gated by ``feasibility.validate_cycle`` as
#: ``skills.segments.plan_segment`` calls it) admits. R4 (2026-09-23) widened
#: this from ``{feasibility, segments}.py`` alone: ``cup_cycle.py`` /
#: ``cup_realize.py`` / ``unified_cycle.py`` / ``tilt_geometry.py`` all shape
#: the SAME solve and a stale box against an edit to any of them would go
#: undetected (plan R4 carried item (d)). The kinematic calibration
#: (2026-09-27) added the generated ``hardware_config.py``: the IK geometry
#: (nodes, leg zero lengths, mm_to_rev, initial height) is as much an input to
#: the solve as any planner file, and without it the geometry commit would
#: have left every committed box "valid" while swept under the old IK -- the
#: re-sweep would be remembered, not forced. Each entry is the path components
#: relative to the ``motion/`` directory -- except ``segments.py``, which
#: (like the live import ``skills.segments``) lives one level down, in
#: ``motion/skills/`` alongside this module, and ``hardware_config.py``, the
#: generated constants module one level UP, in the package root.
_GATED_FILES = (
    ('trajectory', 'feasibility.py'),
    ('segments.py',),
    ('trajectory', 'cup_cycle.py'),
    ('trajectory', 'cup_realize.py'),
    ('unified_cycle.py',),
    ('trajectory', 'tilt_geometry.py'),
    ('hardware_config.py',),
)


def gate_hash(root: str = None) -> str:
    """sha256 of the seven :data:`_GATED_FILES`' TEXT, in order, first 12 hex
    chars.

    Deliberately NOT the commit hash: a working-tree edit to any gated file
    (the common case while iterating on a sweep) must invalidate a box that
    was swept against the old gate, and a git hash only changes at commit.

    ``root`` (test-only) maps EVERY one of the seven paths directly under it
    (``root/trajectory/feasibility.py``, ``root/segments.py``,
    ``root/unified_cycle.py``, ``root/hardware_config.py``, ...) -- a flat
    tmp tree, not a repo layout -- rather than replicating the live tree's
    ``motion/`` vs ``motion/skills/`` vs package-root split. Left ``None``,
    each path resolves against the live tree: this module's own directory
    (``motion/skills``) for ``segments.py``, the package root
    (``jugglebot/``) for ``hardware_config.py``, and ``motion/`` for the rest.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    motion_dir = os.path.join(here, os.pardir)
    package_dir = os.path.join(motion_dir, os.pardir)
    digest = hashlib.sha256()
    for parts in _GATED_FILES:
        if root is not None:
            path = os.path.join(root, *parts)
        elif parts == ('segments.py',):
            path = os.path.join(here, *parts)
        elif parts == ('hardware_config.py',):
            path = os.path.join(package_dir, *parts)
        else:
            path = os.path.join(motion_dir, *parts)
        with open(path, 'rb') as handle:
            digest.update(handle.read())
    return digest.hexdigest()[:12]


def clip(u, box: AdmissibleBox):
    """Clip a THROW command ``u = (landing_xy_m, apex_m)`` into ``box``.

    ``landing_xy_m`` is a 2-vector (metres, relative to the target site);
    ``apex_m`` a scalar (metres above the catch plane). Returns
    ``(clipped_xy, clipped_apex_m)``. An EMPTY box (see
    :attr:`AdmissibleBox.empty`) refuses outright -- there is nothing to clip
    into -- naming the site pair that has no admissible throw.
    """
    landing_xy_m, apex_ = u
    if box.empty:
        raise AdmissibleError(
            'admissible box for site pair %r (apex band %.3f-%.3f m) is EMPTY '
            '-- no grid point passed the sweep with margin, so no throw is '
            'admissible for this pair' % (box.site_pair, box.apex_band_m[0],
                                          box.apex_band_m[1]))
    (xlo, xhi), (ylo, yhi) = box.landing_xy_m
    alo, ahi = box.apex_m
    cx = min(max(float(landing_xy_m[0]), xlo), xhi)
    cy = min(max(float(landing_xy_m[1]), ylo), yhi)
    ca = min(max(float(apex_), alo), ahi)
    return np.array([cx, cy]), ca


def select(boxes: List[AdmissibleBox], pattern: str, site_pair: Tuple[str, str],
          apex_m: float, *, release_site_xy_mm: Tuple[float, float],
          target_site_xy_mm: Tuple[float, float]) -> Optional[AdmissibleBox]:
    """The box in ``boxes`` whose ``pattern`` matches ``pattern``, whose
    ``site_pair`` matches ``site_pair``, whose ``apex_band_m`` contains
    ``apex_m`` (inclusive, 1e-9 m tolerance), and whose ``release_site_xy_mm``
    / ``target_site_xy_mm`` match the live sites to within 1e-6 mm.

    ``None`` when no box covers it -- the caller's cue to refuse rather than
    fall back to some OTHER apex's box (the defect this closes: a self-toss
    at 0.5 m apex silently reusing a box swept for 0.9 m and having its
    apex clipped up to it) or some OTHER geometry's box (R4: a hop swept at
    250 mm separation silently applied to a 100 mm schedule -- a box carries
    no site geometry of its own, so the caller's live sites are the only
    check). Pattern, then site pair, then xy, then apex band -- the same
    four-key lookup :meth:`AdmissibleBox` is keyed by."""
    tol = 1e-9
    xy_tol_mm = 1e-6
    pair = tuple(site_pair)
    rel_x, rel_y = float(release_site_xy_mm[0]), float(release_site_xy_mm[1])
    tgt_x, tgt_y = float(target_site_xy_mm[0]), float(target_site_xy_mm[1])
    for box in boxes:
        if box.pattern != pattern or box.site_pair != pair:
            continue
        if (abs(box.release_site_xy_mm[0] - rel_x) > xy_tol_mm
                or abs(box.release_site_xy_mm[1] - rel_y) > xy_tol_mm
                or abs(box.target_site_xy_mm[0] - tgt_x) > xy_tol_mm
                or abs(box.target_site_xy_mm[1] - tgt_y) > xy_tol_mm):
            continue
        lo, hi = box.apex_band_m
        if lo - tol <= float(apex_m) <= hi + tol:
            return box
    return None


def describe_miss(boxes: List[AdmissibleBox], pattern: str,
                  site_pair: Tuple[str, str], apex_m: float, *,
                  release_site_xy_mm: Tuple[float, float],
                  target_site_xy_mm: Tuple[float, float]) -> str:
    """Plain-language reason :func:`select` returned ``None`` for these exact
    arguments -- for an operator-facing refusal message, not for control flow.

    A miss has two distinct physical causes and conflating them misleads the
    operator (the owner-reported 0.95 m hop refusal at R4 sitting 3, plan
    ``.scratch/r4-throw-precision/issues/04-plain-language-refusal-messages.md``):

    1. A box for this ``(pattern, site_pair)`` DOES cover ``apex_m``, but it
       was swept for different release/target site positions -- a non-default
       ``separation_mm``. Reported first: this is the more surprising cause,
       and a bands-only message hides it entirely because the apex band the
       message names IS satisfied.
    2. No box for this ``(pattern, site_pair)`` covers ``apex_m`` at all,
       regardless of site xy -- reported with whatever bands were swept for
       this pair (if any), so the operator knows what apex range to ask for.
    """
    xy_tol_mm = 1e-6
    tol = 1e-9
    apex = float(apex_m)
    pair = tuple(site_pair)
    rel_x, rel_y = float(release_site_xy_mm[0]), float(release_site_xy_mm[1])
    tgt_x, tgt_y = float(target_site_xy_mm[0]), float(target_site_xy_mm[1])
    same_pair = [b for b in boxes if b.pattern == pattern and b.site_pair == pair]
    for box in same_pair:
        lo, hi = box.apex_band_m
        if not (lo - tol <= apex <= hi + tol):
            continue
        if (abs(box.release_site_xy_mm[0] - rel_x) <= xy_tol_mm
                and abs(box.release_site_xy_mm[1] - rel_y) <= xy_tol_mm
                and abs(box.target_site_xy_mm[0] - tgt_x) <= xy_tol_mm
                and abs(box.target_site_xy_mm[1] - tgt_y) <= xy_tol_mm):
            continue  # xy also matches -- select() would have returned it
        swept_sep = math.hypot(
            box.target_site_xy_mm[0] - box.release_site_xy_mm[0],
            box.target_site_xy_mm[1] - box.release_site_xy_mm[1])
        live_sep = math.hypot(tgt_x - rel_x, tgt_y - rel_y)
        return (
            'apex %.3f m IS covered by a swept box for this site pair, but '
            'the box was swept with the release site at (%.1f, %.1f) mm and '
            'the target site at (%.1f, %.1f) mm -- %.1f mm apart. This '
            'request has them at (%.1f, %.1f) mm and (%.1f, %.1f) mm -- %.1f '
            'mm apart. The box does not carry over to a different '
            'separation_mm: re-sweep for this separation, or run the pattern '
            'at the swept one (%.1f mm).'
            % (apex, box.release_site_xy_mm[0], box.release_site_xy_mm[1],
               box.target_site_xy_mm[0], box.target_site_xy_mm[1], swept_sep,
               rel_x, rel_y, tgt_x, tgt_y, live_sep, swept_sep))
    bands = sorted(b.apex_band_m for b in same_pair)
    bands_str = (', '.join('%.3f-%.3f m' % (lo, hi) for lo, hi in bands)
                if bands else 'none swept for this pair')
    return ('apex %.3f m is outside the range this site pair was swept for '
           '(swept bands: %s) -- re-sweep to cover this apex, or request an '
           'apex inside the swept range' % (apex, bands_str))


def _apex_bands_overlap(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    """True when ``a`` and ``b`` share more than a boundary point (1e-9 m
    tolerance) -- two bands that only TOUCH (e.g. 0.45-0.55 and 0.55-0.65)
    are adjacent, not ambiguous, and must not refuse."""
    tol = 1e-9
    lo1, hi1 = a
    lo2, hi2 = b
    return lo1 < hi2 - tol and lo2 < hi1 - tol


def _refuse_overlapping_apex_bands(boxes: List[AdmissibleBox], error_cls) -> None:
    """Refuse (raising ``error_cls``) when two boxes share a
    ``(pattern, site_pair)`` and their ``apex_band_m`` ranges overlap --
    ambiguous at :func:`select` time, so it is a contract violation in
    whatever built the file, not a tie for the caller to break. Keyed on
    pattern too (R4): a 'hop' and a 'columns' box may legitimately share a
    ``site_pair`` of ``(P1, P1)`` / ``(P2, P2)`` at overlapping apex bands --
    :func:`select` never confuses them, so this must not refuse them either."""
    by_key: Dict[Tuple[str, Tuple[str, str]], List[Tuple[float, float]]] = {}
    for box in boxes:
        key = (box.pattern, box.site_pair)
        for band in by_key.get(key, ()):
            if _apex_bands_overlap(band, box.apex_band_m):
                raise error_cls(
                    'two boxes for pattern %r site pair %r have overlapping '
                    'apex_band_m %r and %r -- select() could not resolve '
                    'which one a command in the overlap belongs to'
                    % (box.pattern, box.site_pair, band, box.apex_band_m))
        by_key.setdefault(key, []).append(box.apex_band_m)


def _limits_dict(limits) -> Dict[str, float]:
    """Build the comparison dict from a live ``TrajectoryLimits``."""
    return {
        'leg_vel_mmps': float(limits.leg_vel_mmps),
        'leg_acc_mmps2': float(limits.leg_acc_mmps2),
        'leg_jerk_mmps3': float(limits.leg_jerk_mmps3),
        'hand_acc_rps2': float(limits.hand_acc_limit_rps2),
    }


def check_limits(boxes: List[AdmissibleBox], limits, *, check_gate: bool = True,
                 dwell_s: Optional[float] = None) -> None:
    """Refuse ``boxes`` against the LIVE session ``limits`` AND the live gate.

    ``limits`` is a ``TrajectoryLimits`` (or anything exposing the same
    ``leg_vel_mmps`` / ``leg_acc_mmps2`` / ``leg_jerk_mmps3`` /
    ``hand_acc_limit_rps2`` attributes). Raises :class:`LimitsMismatch` naming
    the first field that differs -- a box swept under yesterday's limits, or
    against yesterday's ``feasibility.py`` / ``segments.py`` (see
    :func:`gate_hash`), is a box gating against a machine that no longer
    exists. ``check_gate`` is a test-only escape hatch (a working-tree edit to
    either gated file mid-session must not fail every caller that does not
    care).

    ``dwell_s`` (R5, D4, 2026-09-30): the LIVE session dwell, e.g.
    ``schedule.Pattern.dwell_s`` / ``skill_node``'s own ``dwell_s`` parameter.
    ``None`` (the default) skips the dwell check entirely -- a caller that
    does not know its dwell (``tests/hardware/skills_plan_bench.py``) keeps
    its behaviour; ``skill_node.py``'s three call sites pass the live
    ``dwell_s`` (same rung, 2026-09-30). When given, a box whose
    ``dwell_s`` is ``None`` (unstamped) OR numerically different is refused
    naming ``'dwell_s'`` -- the same treatment ``leg_vel_mmps`` etc. get.
    """
    live = _limits_dict(limits)
    live_hash = gate_hash() if check_gate else None
    for box in boxes:
        for key, live_val in live.items():
            swept_val = box.limits.get(key)
            if swept_val is None or not math.isclose(
                    float(swept_val), live_val, rel_tol=1e-9, abs_tol=1e-9):
                raise LimitsMismatch(
                    'admissible box for site pair %r was swept with %s=%r but '
                    'the live session limit is %r -- regenerate '
                    'config/generated/admissible_box.yaml (tools/'
                    'admissible_sweep.py)' % (box.site_pair, key, swept_val,
                                              live_val))
        if dwell_s is not None:
            live_dwell = float(dwell_s)
            if box.dwell_s is None or not math.isclose(
                    float(box.dwell_s), live_dwell, rel_tol=1e-9, abs_tol=1e-9):
                raise LimitsMismatch(
                    'admissible box for site pair %r was swept with '
                    'dwell_s=%r but the live session dwell is %r -- '
                    'regenerate config/generated/admissible_box.yaml (tools/'
                    'admissible_sweep.py --dwell-s %.4f)'
                    % (box.site_pair, box.dwell_s, live_dwell, live_dwell))
        if check_gate and box.gate_hash != live_hash:
            raise LimitsMismatch(
                'admissible box for site pair %r was swept against '
                'gate_hash=%r but the live gate (feasibility.py + segments.py) '
                'hashes to %r -- regenerate config/generated/admissible_box.yaml '
                '(tools/admissible_sweep.py)'
                % (box.site_pair, box.gate_hash, live_hash))


def dump(path: str, boxes: List[AdmissibleBox]) -> None:
    """Write ``boxes`` to ``path`` in the schema :func:`load` reads.

    Every box in one file MUST share ``swept_at`` / ``gate_hash`` / ``limits``
    -- they are one sweep's provenance, hoisted to the top level rather than
    repeated per box.
    """
    if not boxes:
        raise ValueError('cannot write an admissible box file with zero boxes')
    swept_at, ghash, limits = boxes[0].swept_at, boxes[0].gate_hash, boxes[0].limits
    for box in boxes:
        if (box.swept_at, box.gate_hash, box.limits) != (swept_at, ghash, limits):
            raise ValueError(
                'all boxes written to one file must share swept_at/gate_hash/'
                'limits (one sweep run) -- got a mix; site pair %r differs'
                % (box.site_pair,))
    _refuse_overlapping_apex_bands(boxes, ValueError)
    doc = {
        'swept_at': swept_at,
        'gate_hash': ghash,
        'limits': {k: float(limits[k]) for k in _LIMIT_KEYS},
        'boxes': [
            {
                'pattern': str(box.pattern),
                'site_pair': list(box.site_pair),
                'apex_band_m': [float(box.apex_band_m[0]), float(box.apex_band_m[1])],
                'landing_xy_m': [
                    [float(box.landing_xy_m[0][0]), float(box.landing_xy_m[0][1])],
                    [float(box.landing_xy_m[1][0]), float(box.landing_xy_m[1][1])],
                ],
                'apex_m': [float(box.apex_m[0]), float(box.apex_m[1])],
                'release_site_xy_mm': [float(box.release_site_xy_mm[0]),
                                       float(box.release_site_xy_mm[1])],
                'target_site_xy_mm': [float(box.target_site_xy_mm[0]),
                                      float(box.target_site_xy_mm[1])],
                'dwell_s': (float(box.dwell_s) if box.dwell_s is not None
                           else None),
            }
            for box in boxes
        ],
    }
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(path, 'w') as handle:
        yaml.safe_dump(doc, handle, sort_keys=False)


_REQUIRED_TOP = ('swept_at', 'gate_hash', 'limits', 'boxes')
_REQUIRED_BOX = ('site_pair', 'apex_band_m', 'landing_xy_m', 'apex_m')
#: R4 (2026-09-23) box-level fields -- checked SEPARATELY from
#: :data:`_REQUIRED_BOX` so a pre-R4 file (which has every ``_REQUIRED_BOX``
#: key but none of these) gets the dedicated "regenerate the sweep" refusal
#: below rather than a generic "missing %r" that reads like a typo.
_REQUIRED_BOX_R4 = ('pattern', 'release_site_xy_mm', 'target_site_xy_mm')


def load(path: str) -> List[AdmissibleBox]:
    """Read + validate an admissible-box YAML file. Strict: a missing field is
    a refusal naming it, never a best-effort partial parse.

    Every box on disk bounds ``apex_m`` directly (2026-09-18). The pre-apex
    ``flight_s`` compatibility branch (read through ``schedule.apex_m``, the
    exact inverse of the ``schedule.flight_s`` the sweep once used) was
    retired 2026-09-21 once the U4 re-sweep wrote every tracked box as
    ``apex_m`` and a grep of the tree found no other YAML/fixture still
    writing ``flight_s`` (only this module's own loader and one now-deleted
    test exercised it) -- see ``logbook/2026-09-18-learn-the-apex-aim-from-
    the-tracker.md`` ("retire it then, not before").

    **R4 (2026-09-23): a pre-R4 file (no ``pattern`` / site xy on its boxes)
    is REFUSED, never reinterpreted.** There is no default pattern or xy a
    loader could safely infer -- a box with no site geometry stamped on it is
    exactly the defect this unit closes (a box swept at one separation
    silently applied at another), so guessing would resurrect it one loader
    away. Regenerate with ``tools/admissible_sweep.py``.
    """
    try:
        with open(path, 'r') as handle:
            doc = yaml.safe_load(handle)
    except OSError as exc:
        raise AdmissibleError('cannot read admissible box file %s: %s'
                              % (path, exc)) from exc
    except yaml.YAMLError as exc:
        raise AdmissibleError('%s is not valid YAML: %s' % (path, exc)) from exc
    if not isinstance(doc, dict):
        raise AdmissibleError('%s does not parse to a mapping' % (path,))
    for key in _REQUIRED_TOP:
        if key not in doc:
            raise AdmissibleError("%s is missing the top-level '%s' key"
                                  % (path, key))
    swept_at = str(doc['swept_at'])
    ghash = str(doc['gate_hash'])
    limits = doc['limits']
    if not isinstance(limits, dict):
        raise AdmissibleError("%s: 'limits' must be a mapping" % (path,))
    missing_limits = [k for k in _LIMIT_KEYS if k not in limits]
    if missing_limits:
        raise AdmissibleError("%s: limits is missing %r" % (path, missing_limits))
    raw_boxes = doc['boxes']
    if not isinstance(raw_boxes, list) or not raw_boxes:
        raise AdmissibleError("%s: 'boxes' must be a non-empty list" % (path,))
    limits_out = {k: float(limits[k]) for k in _LIMIT_KEYS}
    out = []
    for i, raw in enumerate(raw_boxes):
        if not isinstance(raw, dict):
            raise AdmissibleError('%s: boxes[%d] is not a mapping' % (path, i))
        missing = [k for k in _REQUIRED_BOX if k not in raw]
        if missing:
            raise AdmissibleError("%s: boxes[%d] is missing %r"
                                  % (path, i, missing))
        missing_r4 = [k for k in _REQUIRED_BOX_R4 if k not in raw]
        if missing_r4:
            raise AdmissibleError(
                "%s: boxes[%d] is missing %r -- this is a PRE-R4 admissible "
                "box file (unit U0, 2026-09-23), swept before a box carried "
                "its own pattern/site geometry, and is refused rather than "
                "reinterpreted (a box with no site xy is exactly the defect "
                "this unit closes). Regenerate: python tools/"
                "admissible_sweep.py" % (path, i, missing_r4))
        xy = raw['landing_xy_m']
        apex = (float(raw['apex_m'][0]), float(raw['apex_m'][1]))
        rel_xy = raw['release_site_xy_mm']
        tgt_xy = raw['target_site_xy_mm']
        # R5 (D4, 2026-09-30): `dwell_s` is tolerated ABSENT or null -- a box
        # swept before this field existed, or one round-tripped from a
        # Python-built AdmissibleBox that never set it (see the field's own
        # docstring). Unlike `pattern` / the site-xy fields (refused outright
        # by `_REQUIRED_BOX_R4`), an unstamped dwell is not silently
        # reinterpreted as some default dwell -- it stays `None`, and
        # `check_limits(..., dwell_s=<live>)` refuses a `None` box exactly
        # like a numeric mismatch when the live dwell actually matters to
        # the caller.
        raw_dwell = raw.get('dwell_s')
        dwell = float(raw_dwell) if raw_dwell is not None else None
        out.append(AdmissibleBox(
            site_pair=tuple(raw['site_pair']),
            apex_band_m=(float(raw['apex_band_m'][0]), float(raw['apex_band_m'][1])),
            landing_xy_m=((float(xy[0][0]), float(xy[0][1])),
                          (float(xy[1][0]), float(xy[1][1]))),
            apex_m=apex,
            pattern=str(raw['pattern']),
            release_site_xy_mm=(float(rel_xy[0]), float(rel_xy[1])),
            target_site_xy_mm=(float(tgt_xy[0]), float(tgt_xy[1])),
            limits=limits_out,
            gate_hash=ghash,
            swept_at=swept_at,
            dwell_s=dwell,
        ))
    _refuse_overlapping_apex_bands(out, AdmissibleError)
    return out
