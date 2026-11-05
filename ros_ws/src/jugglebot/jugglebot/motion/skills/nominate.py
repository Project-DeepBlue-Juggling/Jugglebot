"""Nomination-time admissibility check (learner-generalisation P3b, owner D8).

A NOMINATION answers, before anything moves: "can this Pattern configuration
be flown at these Sitting limits with the 10 % margin?"  :func:`nominate`
builds the same ``Pattern`` / ``OneBallPattern`` skill_node builds
(:func:`build_pattern`), compiles a SHORT Schedule from it with a dummy ``t0``,
and plans EVERY nominal Segment (``schedule.nominal_segments``) with the REAL
planner and gate (``segments.plan_segment``) at each rung of the
commanded-flight ladder (:data:`RUNGS`).  The SHORT Schedule ignores the
operator's Throw count: :func:`short_n_throws` is the launch plus two periods
of the Site cycle, five Throws for Columns and the hop, three for the
self-throw and the oval, enough for every steady Segment shape to appear
seeded from a carried release.  REST Segments are counted, not gated (bend 7).  It is a pure function of configuration, limits, geometry and
planner config: no live state, no ROS.  It reports every refusal at once
(:meth:`NominationReport.lines`) and, when refused, names the nearest viable
configuration on the operator's own grid (:data:`SEARCH_STEPS` steps of
:data:`APEX_STEP_M` / :data:`SEPARATION_STEP_MM` / :data:`DWELL_STEP_S`).

A verdict's ``margin`` is ``admissible.margin`` (headroom against
``MARGIN_FRAC`` x the leg limits); on a Throw or a carried Throw the planned
takeoff is also judged by ``throw_envelope.evaluate`` (C-HAND-3).

What a nomination does NOT see
------------------------------
* the live feed arrival: Ball Butler's real ball, and any ball in flight
  before ``t0``, is replaced by a synthesis (bend 1 below);
* Ball Butler's position: ``columns_turned`` takes the fed Site from
  :attr:`NominationConfig.feed_site` (the straight-pair rule), not from the
  live geometry skill_node uses;
* the Hand beyond its refusal codes: the margin is LEG-only, a Hand limit
  shows only when the planner refuses with ``HAND_LIMIT_*``;
* the Learner: every Throw is commanded at the identity (``u = y_d``), moved
  only by the rung's flight fraction.

Accepted bends (``logbook/2026-11-06-learner-generalisation-p3b-nomination-check.md``)
-----------------------------------------------------------------------------------
1. **Phantom synthesis for a ball in flight before t0.** ``compile_columns``'
   first CATCH has no producing Throw and no landing prior under a dummy-t0
   compile.  Reporting it NOT_NOMINATED would leave every later Columns
   Segment NOT_PLANNED, so the walker uses the executor's phantom synthesis
   (identity self-toss at that Site at the ball's own pattern flight) and says
   so in the verdict's message.
2. **PROXY seeds and their taint.** A Segment whose seed producer was refused
   is still planned, so every refusal shows at once: from the last good
   release of the same shape if there is one, else from the producer's own
   release if it planned and was refused only on MARGIN, else from a PROXY
   (the sweep's seed: a Throw from rest at the producer's release Site at the
   identity command and nominal flight).  Such verdicts say so in ``seed``,
   and every descendant is tainted; a PROXY never makes a refused
   configuration viable (the refusal that forced it stands).
3. **Rung-identical refusals share a line** (same Segment, code, leg and
   margin to 1e-3): ``rungs -20/+0/+10 %``.
4. **Stop and hop-entry Throws fly at the nominal flight on every rung**: the
   executor sends ``u = y_d`` for a ``shadow_landing`` / ``entry_hop`` Throw,
   so the rung never moves them.
5. **Envelope on carried Throws too**, not only on THROW Skills (the P3a
   resolver judges both).
6. **Workspace half-extent 0 in the separation search**
   (:data:`WORKSPACE_HALF_EXTENT_MM`): a trial separation is viable only if
   every Site's ``|xy|`` is inside ``CupCycleConfig().workspace_xy_m``; a
   30 mm authority half-extent would refuse the GUI's 250 mm hop default.
7. **REST is counted, not gated**: every REST in these Schedules is a
   rest-terminal settle planned from rest or a rest-terminal catch, and the
   opening REST starts from the live Hand, which a nomination does not have.

Segment clock: a seeded Segment's origin is its producer's release instant;
from rest it is ``t_abs_s - window_s`` (``plan_columns_first_cycle``'s rule).
Times go onto the Segment clock through ``executor._terminal_on_segment_clock``.

The incoming ball of every CATCH arrives ballistically from its producer's
NOMINAL release (pattern apex, identity command): only the CARRIED Throw's
commanded flight moves with the rung (the sweep's rule).

Python 3.8; no ROS imports (the ``motion/`` rule).
"""
from __future__ import annotations

import collections
import dataclasses
import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.trajectory import ballistics_bc as bal
from jugglebot.motion.trajectory import cup_cycle as cc
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory import throw_envelope
from jugglebot.motion.trajectory.limits import TrajectoryLimits
from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as st
from jugglebot.motion.skills.executor import (SkillExecutor, _previous_release,
                                              _terminal_on_segment_clock,
                                              release_site_of)

THROW, CATCH, REST = sc.THROW, sc.CATCH, sc.REST

# ── seeds and limits (moved here from tools/admissible_sweep.py) ────────────


def rest_state(cup_mm) -> uc.CycleState:
    """A plan seed at rest at ``cup_mm`` -- identical construction to
    ``tests/motion/test_skills_segments.py::_rest_state`` and
    ``tools/probes/skills_sizing_sweep.py::_rest_state``.  Moved from
    ``tools/admissible_sweep.py`` (which imports it back as ``_rest_state``)."""
    rcfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - rcfg.cup_z_base_mm
    rev = ((slider_mm - rcfg.slider_rev_zero_mm) / 1000.0) * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], rcfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, rcfg)


def sitting_limits(leg_vel, leg_acc, leg_jerk, hand_acc) -> TrajectoryLimits:
    """The planner's limits at a Sitting's leg vel / acc / jerk and Hand acc.
    Moved from ``tools/admissible_sweep.py`` (imported back as ``_limits``)."""
    return TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=float(leg_vel), leg_acc_mmps2=float(leg_acc),
        leg_jerk_mmps3=float(leg_jerk), hand_acc_rps2=float(hand_acc))


def default_limits() -> TrajectoryLimits:
    """:func:`sitting_limits` at the generated launch constants."""
    return sitting_limits(hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS,
                          hw.JB_TRAJ_LEG_ACC_LIMIT_MMPS2,
                          hw.JB_TRAJ_LEG_JERK_LIMIT_MMPS3,
                          hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2)


# ── constants ────────────────────────────────────────────────────────────────

#: The commanded-flight ladder's two edges and its centre.  ORIGIN:
#: ``tools/admissible_sweep.py`` ``SINGLE_APEX_FLIGHT_FRAC`` = (-0.20 ..
#: +0.10), the ladder the box was swept on; a nomination plans its edges and
#: the nominal (the margin is smooth between them, logbook 2026-11-04).
RUNGS = (-0.20, 0.0, 0.10)

#: Patterns a nomination plans: skill_node ``_PATTERNS`` minus the two below.
NOMINATED_PATTERNS = ('self_toss', 'hop', 'columns', 'columns_1ball',
                      'columns_1ball_fed', 'columns_turned', 'columns_highlow',
                      'oval_1ball', 'oval_1ball_fed')
#: The refusal of a nomination that walked no non-REST Segment.
NOTHING_TO_PLAN = 'NOT_NOMINATED: nothing to plan'
#: Patterns refused ``NOT_NOMINATED`` by name, with the reason.
NOT_NOMINATED_PATTERNS = {'magazine': 'formulaic (a MAGAZINE throw from rest)',
                          'reload': 'needs a live landing (Ball Butler)'}

_ONE_PERIOD_PATTERNS = ('self_toss', 'oval_1ball', 'oval_1ball_fed')
COLUMNS_FED = ('columns', 'columns_turned', 'columns_highlow',
                'columns_1ball_fed')

#: skill_node's self-toss Site (``_DEFAULT_SITE_NAME`` / ``_DEFAULT_SITE_X_MM``
#: / ``_DEFAULT_SITE_Y_MM``), the site the self-toss pin compares against.
SELF_TOSS_SITE_NAME = 'P1'
SELF_TOSS_SITE_XY_MM = (-50.0, 0.0)

#: Nearest-viable search (``motion/skills/INVARIANTS.md`` I-ADMIT-2;
#: ``logbook/2026-11-06-learner-generalisation-p3b-nomination-check.md``):
#: a deliberate GRID, not bisection:
#: the answer must be a value the operator can type in the GUI Juggle panel,
#: the margin is smooth in each parameter (logbook 2026-11-04), and a bounded
#: number of steps bounds the cost).  One parameter at a time, outward from
#: the commanded value, first viable value wins.
SEARCH_STEPS = 4
APEX_STEP_M = 0.05          # downward; apex_m_ball2 moves with it (gap kept)
SEPARATION_STEP_MM = 25.0   # downward, never <= 0, workspace rule below
DWELL_STEP_S = 0.05         # oval: upward only; others: down and up, alternating
#: Site half-extent the separation search adds to ``|site xy|`` against
#: ``CupCycleConfig().workspace_xy_m`` (bend 6: 0, not the 30 mm authority).
WORKSPACE_HALF_EXTENT_MM = 0.0
#: Nominations kept by the module LRU (select-then-Start must not solve twice).
CACHE_SIZE = 64


# ── the configuration ────────────────────────────────────────────────────────

@dataclasses.dataclass(frozen=True)
class NominationConfig:
    """Everything a nomination depends on besides limits and geometry: the
    goal fields and the node parameters skill_node resolves at accept."""
    pattern: str
    apex_m: float
    separation_mm: float
    dwell_s: float
    #: ``columns_highlow`` only (skill_node param ``apex_m_ball2``)
    apex_m_ball2: Optional[float] = None
    #: ``columns_turned`` only (skill_node param ``columns_angle_deg``)
    angle_deg: float = 0.0
    planner_mode: str = 'pose'
    carry_zvd: bool = False
    #: None = the SHORT pattern (:func:`short_n_throws`); set only to override
    n_throws: Optional[int] = None
    #: the fed Columns Site's name (skill_node param ``columns_feed_site``;
    #: for a turned pair the name of the geometric pick, ``P1@<angle>`` or
    #: ``P2@<angle>``, or its bare ``P1`` / ``P2`` prefix)
    feed_site: str = 'P1'
    #: the self-toss Site's xy (skill_node params ``site_x_mm`` / ``site_y_mm``)
    self_toss_site_xy_mm: Tuple[float, float] = SELF_TOSS_SITE_XY_MM
    #: 't0': compile with a dummy t0 (the design).  'hop': what skill_node
    #: dispatches for a FED Columns (``compile_columns(feed=..., hop_entry=
    #: True)``) with a VERTICAL identity feed at the fed Site.
    entry: str = 't0'

    def key(self) -> tuple:
        return dataclasses.astuple(self)


def short_n_throws(pattern: str) -> int:
    """The SHORT pattern a nomination compiles: the launch plus two full
    periods of the Site cycle, so every steady Segment shape appears seeded
    from a CARRIED release, not only from the launch.  Two Throws per period
    for Columns / hop (-> 5; Columns' 5th Throw is the Stop, carried by a
    steady catch; the standalone last CATCH and the REST follow), one for
    self-toss / oval (-> 3)."""
    per = 1 if pattern in _ONE_PERIOD_PATTERNS else 2
    return 1 + 2 * per


def _feed_index(sites, feed_site: str) -> int:
    """0 if ``feed_site`` names ``sites[0]`` (in full or by its ``P1``-style
    prefix), else 1 -- skill_node's ``== 'P1'`` / ``== sites[0].name`` pick."""
    name0 = sites[0].name
    return 0 if feed_site in (name0, name0.split('@')[0]) else 1


def build_pattern(config: NominationConfig, n_throws: Optional[int] = None):
    """THE ``Pattern`` / ``OneBallPattern`` for ``config`` -- the one builder
    skill_node and a nomination share.  ``n_throws`` = the operator's count
    (skill_node) or None -> ``config.n_throws`` or :func:`short_n_throws`.

    skill_node's goal path calls this builder (I-ADMIT-2); it reproduces what
    these ``skill_node.py`` paths built before it existed:
    * ``_run_columns`` (``columns``, ``columns_turned``, ``columns_highlow``,
      ``columns_1ball_fed``): ``columns_sites(separation_mm, angle_deg)``, the
      fed Site by ``columns_feed_site`` (turned: by the geometric pick, which
      the caller passes as ``feed_site``), ``Pattern(sites=(a_site,
      feed_site), ..., phantom_balls=(0,) if columns_1ball_fed,
      apex_m_ball_b=apex_m_ball2 if columns_highlow)``;
    * ``_run_columns_1ball``: ``columns_sites(separation_mm)``, the phantom
      Site = the ``columns_feed_site`` one, ``Pattern(sites=(a_site,
      phantom_site), ..., phantom_balls=(1,))``;
    * ``_run_one_ball`` (``self_toss``, ``hop``, ``oval_1ball``,
      ``oval_1ball_fed``): self-toss ``(Site('P1', [site_x_mm, site_y_mm,
      CATCH_CUP_Z_MM]),)``, else ``columns_sites(separation_mm)``;
      ``OneBallPattern(..., roles='fixed' if oval else 'alternate')``.

    Raises ``ValueError`` (``columns_sites``, the Pattern constructors, or an
    un-nominated pattern name)."""
    p = config.pattern
    n = int(n_throws if n_throws is not None
            else (config.n_throws or short_n_throws(p)))
    if p in COLUMNS_FED:
        sites = st.columns_sites(config.separation_mm,
                                 config.angle_deg if p == 'columns_turned' else 0.0)
        fi = _feed_index(sites, config.feed_site)
        feed_site, a_site = sites[fi], sites[1 - fi]
        return sc.Pattern(sites=(a_site, feed_site), apex_m=config.apex_m,
                          dwell_s=config.dwell_s, n_throws=n,
                          phantom_balls=((0,) if p == 'columns_1ball_fed' else ()),
                          apex_m_ball_b=(config.apex_m_ball2
                                         if p == 'columns_highlow' else None))
    if p == 'columns_1ball':
        sites = st.columns_sites(config.separation_mm)
        fi = _feed_index(sites, config.feed_site)
        phantom_site, a_site = sites[fi], sites[1 - fi]
        return sc.Pattern(sites=(a_site, phantom_site), apex_m=config.apex_m,
                          dwell_s=config.dwell_s, n_throws=n, phantom_balls=(1,))
    if p == 'self_toss':
        x, y = config.self_toss_site_xy_mm
        sites = (st.Site(SELF_TOSS_SITE_NAME,
                         np.array([float(x), float(y), st.CATCH_CUP_Z_MM])),)
    elif p in ('hop', 'oval_1ball', 'oval_1ball_fed'):
        sites = st.columns_sites(config.separation_mm)
    else:
        raise ValueError('pattern %r is not nominated' % (p,))
    is_oval = p in ('oval_1ball', 'oval_1ball_fed')
    return sc.OneBallPattern(sites=sites, apex_m=config.apex_m,
                             dwell_s=config.dwell_s, n_throws=n,
                             roles='fixed' if is_oval else 'alternate')


#: The dummy ``t0`` a nomination compiles at (any instant: nothing is wall-
#: clock bound offline).
DUMMY_T0_S = 10.0


def compile_schedule(config: NominationConfig, t0: float = DUMMY_T0_S):
    """The SHORT Schedule a nomination walks.  Raises ``ValueError`` (the
    compiler's own text: no transit, window under the floor, dispatch not
    monotone) -> a ``COMPILE`` refusal."""
    pat = build_pattern(config)
    if isinstance(pat, sc.Pattern):
        if config.entry == 'hop' and config.pattern != 'columns_1ball':
            feed_site = pat.sites[1]
            tm = sc.columns_ball_timing(pat.apex_for(0), pat.apex_for(1),
                                        pat.dwell_s)
            pos = np.asarray(feed_site.catch_site_mm(), dtype=float)
            lv = bal.launch_velocity(
                np.asarray(feed_site.throw_site_mm(), dtype=float), pos,
                tm.flights[1])
            feed = sc.LandingPrior(
                pos_mm=pos,
                vel_mm_s=np.asarray(bal.arrival_velocity(lv, tm.flights[1]),
                                    dtype=float),
                t_land_abs_s=float(t0) + tm.transits[1])
            return sc.compile_columns(pat, feed=feed, hop_entry=True)
        return sc.compile_columns(pat, t0_abs_s=t0)
    return sc.compile_one_ball(pat, t0)


def segment_config(config: NominationConfig) -> sg.SegmentConfig:
    """The planner config: ``carry_zvd`` exists only in the pose planner."""
    zvd = bool(config.carry_zvd) and config.planner_mode == 'pose'
    return sg.SegmentConfig(planner_mode=config.planner_mode, carry_zvd=zvd)


# ── the walker ───────────────────────────────────────────────────────────────

@dataclasses.dataclass
class SegmentVerdict:
    """One Skill's verdict at one rung.  ``ok`` = planned, margin >= 0 and
    envelope ok.  ``code``: 'OK' | 'MARGIN' | planner / gate code |
    'ENVELOPE:<bound>' | 'UNREACHABLE' | 'NOT_PLANNED' (no seed: the producer
    and its PROXY both refused) | 'REST' (counted, not gated).  ``binding``: which of vel / acc / jerk has
    the least headroom.  ``leg`` / ``t_s``: the report's ``peak_leg_index`` /
    ``peak_leg_t_s`` (set on LIMIT_* refusals only)."""
    idx: int
    kind: str
    ball: str
    site: str
    target: Optional[str]
    flight_frac: float
    ok: bool
    code: str
    margin: float
    binding: Optional[str]
    leg: Optional[int]
    t_s: Optional[float]
    message: str
    release: Optional[str] = None
    seed: str = ''
    peaks: Optional[Tuple[float, float, float]] = None
    solved: bool = False

    def label(self) -> str:
        s = 'skill %d %s %s at %s' % (self.idx, self.kind, self.ball, self.site)
        if self.release and self.release != self.site:
            s += ' (release %s)' % self.release
        if self.target:
            s += ' -> %s' % self.target
        return s

    def line(self) -> str:
        rung = 'rung %s' % ('0 %' if abs(self.flight_frac) < 1e-12
                            else '%+d %%' % int(round(self.flight_frac * 100)))
        what = self.code
        if self.leg is not None:
            what += ' leg %d at t=%.3f s' % (self.leg, self.t_s)
        if not math.isnan(self.margin):
            what += ', margin %.3f' % self.margin
            if self.binding:
                what += ' (%s)' % self.binding
        if self.message and self.code not in ('OK', 'MARGIN'):
            what += ' [%s]' % self.message
        return '%s, %s: %s' % (self.label(), rung, what)


def _binding(report, limits):
    """``(binding limit name, (vel, acc, jerk) peaks)`` or ``(None, None)``."""
    if report is None:
        return None, None
    f = adm.MARGIN_FRAC
    peaks = (float(report.peak_leg_vel_mmps), float(report.peak_leg_acc_mmps2),
             float(report.peak_leg_jerk_mmps3))
    lims = (limits.leg_vel_mmps, limits.leg_acc_mmps2, limits.leg_jerk_mmps3)
    head = [(f * L - p) / (f * L) for p, L in zip(peaks, lims)]
    return ('vel', 'acc', 'jerk')[int(np.argmin(head))], peaks


def _commanded_target_mm(target_site, y_d) -> np.ndarray:
    """The identity command's landing point (``SkillExecutor.
    _commanded_target_mm`` at ``u = y_d``)."""
    return SkillExecutor._commanded_target_mm(target_site, y_d)


def _cmd_frac(flight_frac: float, bypass: bool) -> float:
    """Bend 4: a Stop / hop-entry Throw bypasses the Learner (``u = y_d``,
    executor ``_command_u``), so its commanded flight never moves with a rung."""
    return 0.0 if bypass else float(flight_frac)


def _ball(sk) -> str:
    return sc.ball_label(sk.ball_id)


def plan_nominal(schedule, limits, geom, cfg, flight_frac: float = 0.0, *,
                 stop_after: Optional[int] = None,
                 counter: Optional[list] = None,
                 proxy_seeds: bool = True) -> List[SegmentVerdict]:
    """Walk ``schedule.nominal_segments(schedule)`` keeping the chain of
    release states, planning each Segment with ``segments.plan_segment`` at
    commanded flight ``flight_s(apex) * (1 + flight_frac)`` (the schedule's
    timing and every incoming ball stay at the pattern flight).  One verdict
    per Skill, in dispatch order; ``stop_after`` ends the walk after that
    Skill index; ``counter[0]`` counts solves."""
    verdicts: List[SegmentVerdict] = []
    segs: Dict[int, object] = {}         # idx -> planned Segment (ok or MARGIN)
    good_by_shape: Dict[tuple, object] = {}
    status: Dict[int, str] = {}          # idx -> 'ok' | its refusal code
    tainted = set()                      # seed chain runs through a PROXY
    skills = schedule.skills

    def shape(sk):
        return (sk.kind, sk.site.name, release_site_of(sk).name,
                sk.then_throw is not None)

    for ns in sc.nominal_segments(schedule):
        if stop_after is not None and ns.idx > stop_after:
            break
        sk = ns.skill
        rel = release_site_of(sk)
        tgt = (sk.target if sk.kind == THROW else
               (sk.then_throw.target if sk.then_throw is not None else None))
        v = SegmentVerdict(idx=ns.idx, kind=sk.kind, ball=_ball(sk),
                           site=sk.site.name, target=tgt.name if tgt else None,
                           flight_frac=float(flight_frac), ok=True, code='OK',
                           margin=float('nan'), binding=None, leg=None, t_s=None,
                           message='', release=rel.name)
        if sk.kind == REST:
            v.code, v.message = 'REST', 'rest-terminal settle, not gated'
            verdicts.append(v)
            status[ns.idx] = 'ok'
            continue
        # ── the seed ──
        if ns.seed_from is None:
            prev = [segs[k] for k in sorted(segs)]
            rest_mm = (prev[-1].rest_site_mm if prev else sk.site.rest_site_mm())
            seed = rest_state(rest_mm)
            v.seed = 'rest'
            t_origin = float(sk.t_abs_s) - float(sk.window_s)
        else:
            k = ns.seed_from
            src = skills[k]
            t_origin = float(sc._release_instants(src)[0])
            if status.get(k) == 'ok':
                seed = segs[k].release_state
                v.seed = 'release of skill %d' % k
                if k in tainted:
                    v.seed += ' [chain via PROXY]'
                    tainted.add(ns.idx)
            else:
                alt = good_by_shape.get(shape(src))
                seed = None
                if alt is not None:
                    seed = alt.release_state
                    v.seed = ('release of the last good %s %s at %s (skill %d '
                              'refused %s)' % (src.kind, _ball(src),
                                               src.site.name, k, status.get(k)))
                elif k in segs and segs[k].release_state is not None:
                    # Bend 2: a producer refused AFTER planning (MARGIN or
                    # ENVELOPE) has a release: the machine's state there had
                    # it been flown.  The label names its actual refusal.
                    seed = segs[k].release_state
                    v.seed = 'release of skill %d (itself refused %s)' % (
                        k, status.get(k))
                    tainted.add(ns.idx)
                elif proxy_seeds:
                    seed = _proxy_release(src, limits, geom, cfg, counter)
                    if seed is not None:
                        tainted.add(ns.idx)
                        v.seed = ('PROXY: launch from rest at %s (skill %d '
                                  'refused %s)' % (release_site_of(src).name, k,
                                                   status.get(k)))
                if seed is None:
                    v.ok, v.code = False, 'NOT_PLANNED'
                    v.message = 'seed refused at skill %d (%s)' % (
                        k, status.get(k, '?'))
                    verdicts.append(v)
                    status[ns.idx] = v.code
                    continue
        # ── the terminal ──
        try:
            if sk.kind == THROW:
                frac = _cmd_frac(flight_frac, sk.shadow_landing or sk.entry_hop)
                cmd_flight = sc.flight_s(float(sk.y_d[1])) * (1.0 + frac)
                term = sg.ThrowTerminal(
                    site_mm=sk.site.throw_site_mm(),
                    target_mm=_commanded_target_mm(sk.target, sk.y_d),
                    flight_s=cmd_flight, t_release_s=float(sk.t_abs_s))
            else:
                pos_mm, vel_mm_s, t_land, note = _landing(schedule, ns.idx, sk)
                if note:
                    v.message = note
                then = None
                cmd_flight = None
                tt = sk.then_throw
                if tt is not None:
                    if rel is sk.site:
                        release_mm = np.array([pos_mm[0], pos_mm[1],
                                               float(rel.throw_site_mm()[2])])
                    else:
                        release_mm = np.asarray(rel.throw_site_mm(), dtype=float)
                    frac = _cmd_frac(flight_frac, tt.shadow_landing)
                    cmd_flight = sc.flight_s(float(tt.y_d[1])) * (1.0 + frac)
                    then = sg.ThrowAfterCatch(
                        t_release_s=float(tt.t_release_abs_s), site_mm=release_mm,
                        target_mm=_commanded_target_mm(tt.target, tt.y_d),
                        flight_s=cmd_flight)
                if sk.rest_site_mm is not None:
                    rest_mm = sg.hold_axis_site(pos_mm, sk.hold_tilt,
                                                float(sk.rest_site_mm[2]))
                else:
                    rest_mm = rel.rest_site_mm()
                term = sg.CatchTerminal(landing_mm=pos_mm, landing_vel_mm_s=vel_mm_s,
                                        t_land_s=float(t_land), rest_site_mm=rest_mm,
                                        then_throw=then, hold_tilt=sk.hold_tilt,
                                        receive_tilt=sk.receive_tilt)
            term = _terminal_on_segment_clock(sk.kind, term, t_origin)
        except ValueError as exc:
            v.ok, v.code, v.message = False, 'UNREACHABLE', str(exc)
            verdicts.append(v)
            status[ns.idx] = v.code
            continue
        # ── the solve ──
        if counter is not None:
            counter[0] += 1
        v.solved = True
        try:
            seg = sg.plan_segment(sk.kind, seed, term, cfg, limits, geom)
        except uc.CycleInfeasible as exc:
            rep = exc.report
            v.ok, v.code = False, exc.code
            v.margin = adm.margin(rep, limits)
            v.binding, v.peaks = _binding(rep, limits)
            if rep is not None and getattr(rep, 'peak_leg_index', None) is not None:
                v.leg, v.t_s = int(rep.peak_leg_index), float(rep.peak_leg_t_s)
            v.message = '; '.join(exc.reasons)
            verdicts.append(v)
            status[ns.idx] = v.code
            continue
        rep = seg.meta.report
        v.margin = adm.margin(rep, limits)
        v.binding, v.peaks = _binding(rep, limits)
        segs[ns.idx] = seg
        if not v.margin >= 0.0:
            v.ok, v.code = False, 'MARGIN'
            status[ns.idx] = v.code
            verdicts.append(v)
            continue
        # bend 5: the envelope on every release this Segment carries
        if seg.takeoff_vel_mm_s is not None and cmd_flight is not None:
            speed = float(np.linalg.norm(seg.takeoff_vel_mm_s)) / 1000.0
            ev = throw_envelope.evaluate(cmd_flight, speed)
            if not ev.ok:
                v.ok, v.code, v.message = False, 'ENVELOPE:%s' % ev.bound, ev.detail
                status[ns.idx] = v.code
                verdicts.append(v)
                continue
        status[ns.idx] = 'ok'
        good_by_shape[shape(sk)] = seg
        verdicts.append(v)
    return verdicts


def _proxy_release(src, limits, geom, cfg, counter):
    """Bend 2's PROXY: the release state of a Throw from rest at ``src``'s
    release Site toward the Site it releases to, at the identity command and
    nominal flight, over the launch window (the sweep's per-pair seed).  None
    if that refuses too."""
    rel = release_site_of(src)
    if src.kind == THROW:
        tgt, y_d = src.target, src.y_d
    else:
        tgt, y_d = src.then_throw.target, src.then_throw.y_d
    term = sg.ThrowTerminal(site_mm=rel.throw_site_mm(),
                            target_mm=_commanded_target_mm(tgt, y_d),
                            flight_s=sc.flight_s(float(y_d[1])),
                            t_release_s=float(sc.Pattern.launch_s))
    if counter is not None:
        counter[0] += 1
    try:
        seg = sg.plan_segment(THROW, rest_state(rel.rest_site_mm()), term, cfg,
                              limits, geom)
    except uc.CycleInfeasible:
        return None
    return seg.release_state


def _landing(schedule, idx, sk):
    """``(pos_mm, vel_mm_s, t_land_abs_s, note)`` at the identity command.
    Always built: from the Skill's landing prior, else from the producing
    release in the Schedule, else (a ball in flight before t0) by bend 1's
    synthesis.  A Pattern that needs a live landing (``reload``) is refused
    ``NOT_NOMINATED`` by name before any walk."""
    if sk.landing_prior is not None:
        lp = sk.landing_prior
        return (np.asarray(lp.pos_mm, dtype=float),
                np.asarray(lp.vel_mm_s, dtype=float),
                float(lp.t_land_abs_s), 'landing prior')
    prev = _previous_release(schedule, idx, sk.ball_id)
    if prev is None:
        # Bend 1: a ball in flight before t0 (compile_columns' initial CATCH):
        # the executor's phantom synthesis, an identity self-toss at this Site
        # at the ball's own pattern flight (``_predicted_landing``).
        flight = schedule.flight_for(sk.ball_id)
        pos = np.asarray(sk.site.catch_site_mm(), dtype=float)
        lv = bal.launch_velocity(np.asarray(sk.site.throw_site_mm(), dtype=float),
                                 pos, flight)
        return (pos, bal.arrival_velocity(lv, flight), float(sk.t_abs_s),
                'pre-t0 ball: identity self-toss assumed')
    _site, t_release, y_d, target, ridx = prev
    flight = sc.flight_s(float(y_d[1]))       # NOMINAL: the pattern apex, y_d
    pos = _commanded_target_mm(target, y_d)
    # The release the plan uses: a THROW at its Site; a same-site fold at the
    # caught xy (identity: the Site); a carried fold at the release Site. At
    # the identity command all three are throw_site_mm().
    rel_mm = np.asarray(release_site_of(schedule.skills[ridx]).throw_site_mm(),
                        dtype=float)
    lv = bal.launch_velocity(rel_mm, pos, flight)
    return pos, bal.arrival_velocity(lv, flight), t_release + flight, ''


# ── the report ───────────────────────────────────────────────────────────────

@dataclasses.dataclass
class NominationReport:
    """The verdict on one configuration.  ``refusals`` = one line per refusal
    (rung-identical refusals collapsed, bend 3); ``nearest`` = ``[(parameter,
    value, summary of that configuration's nomination)]``, parameter in
    ``apex`` / ``separation`` / ``dwell``; ``solves`` = the planner calls of
    this nomination (the search's are ``search_solves``)."""
    config: NominationConfig
    ok: bool
    verdicts: List[SegmentVerdict]
    refusals: List[str]
    nearest: List[Tuple[str, float, str]]
    wall_s: float
    solves: int
    #: True once the nearest-viable search ran (cached reports search once)
    search_done: bool = False
    #: full (three-rung) nominations the search ran
    searched: int = 0
    search_solves: int = 0
    search_wall_s: float = 0.0
    search_note: str = ''
    search_trace: List[tuple] = dataclasses.field(default_factory=list)

    def worst(self) -> Optional[SegmentVerdict]:
        vs = [v for v in self.verdicts if v.solved and not math.isnan(v.margin)]
        return min(vs, key=lambda v: v.margin) if vs else None

    def config_text(self) -> str:
        c = self.config
        s = '%s apex %.2f' % (c.pattern, c.apex_m)
        if c.pattern == 'columns_highlow' and c.apex_m_ball2 is not None:
            s += '/%.2f' % c.apex_m_ball2
        s += ' m'
        if c.pattern != 'self_toss':
            s += ', separation %.0f mm' % c.separation_mm
        s += ', dwell %.2f s' % c.dwell_s
        if c.pattern == 'columns_turned':
            s += ', angle %.0f deg' % c.angle_deg
        s += ', %s%s' % (c.planner_mode, ', carry_zvd' if c.carry_zvd else '')
        return s

    def summary(self) -> str:
        """ONE line: ``nomination OK: <config>; worst margin 0.310 (jerk, CATCH
        Ball 2 at P2, rung +10 %)`` or ``nomination REFUSED: <config>; 3
        refusals; nearest viable: separation 100 mm (or dwell 0.22 s,
        shorter)``."""
        if self.ok:
            w = self.worst()
            where = ''
            if w is not None:
                where = ' (%s, %s %s at %s, rung %+d %%)' % (
                    w.binding, w.kind, w.ball, w.site,
                    int(round(w.flight_frac * 100)))
            return ('nomination OK: %s; worst margin %.3f%s'
                    % (self.config_text(), w.margin if w else float('nan'),
                       where))
        n = len(self.refusals)
        s = 'nomination REFUSED: %s; %d refusal%s' % (
            self.config_text(), n, '' if n == 1 else 's')
        if self.nearest:
            texts = [_fmt_param(p, val, self.config) for p, val, _ in self.nearest]
            s += '; nearest viable: ' + texts[0] + ''.join(
                ' (or %s)' % t for t in texts[1:])
        elif self.search_note:
            s += '; ' + self.search_note
        return s

    def lines(self) -> List[str]:
        """Every refusal, one line each, in the ``skills/check`` style (plain
        text, '; '-joinable)."""
        return list(self.refusals)


def _fmt_param(param: str, val: float, config: NominationConfig) -> str:
    if param == 'apex':
        return 'apex %.2f m' % val
    if param == 'separation':
        return 'separation %.0f mm' % val
    return 'dwell %.2f s, %s' % (val, 'shorter' if val < config.dwell_s
                                 else 'longer')


def _collapse(bad: List[SegmentVerdict]) -> List[str]:
    """Bend 3: one line per refusing Segment; rungs that refuse it
    IDENTICALLY (same code, leg, margin to 1e-3, seed kind) share the line."""
    groups: 'collections.OrderedDict' = collections.OrderedDict()
    for v in bad:
        k = (v.idx, v.code, v.leg,
             None if math.isnan(v.margin) else round(v.margin, 3),
             v.seed.startswith('PROXY'))
        groups.setdefault(k, []).append(v)
    out = []
    for vs in groups.values():
        line = vs[0].line()
        if len(vs) > 1:
            rr = '/'.join('%+d' % int(round(v.flight_frac * 100)) for v in vs)
            head, _, tail = line.partition(': ')
            head = head.rsplit(', rung', 1)[0] + ', rungs %s %%' % rr
            line = head + ': ' + tail
        if 'PROXY' in vs[0].seed or 'itself refused' in vs[0].seed:
            line += ' {seed: %s}' % vs[0].seed
        out.append(line)
    return out


# ── nominate, the cache, the nearest-viable search ───────────────────────────

_CACHE: 'collections.OrderedDict' = collections.OrderedDict()


def limits_key(limits: TrajectoryLimits) -> tuple:
    """Every field of the planner's limits (the cache key's limits part)."""
    return tuple(float(x) for x in dataclasses.astuple(limits))


_GEOM: List[StewartGeometry] = []


def _default_geom() -> StewartGeometry:
    """The module's one default ``StewartGeometry`` (the cache's geometry)."""
    if not _GEOM:
        _GEOM.append(StewartGeometry())
    return _GEOM[0]


def clear_cache() -> None:
    _CACHE.clear()


def nominate(config: NominationConfig, limits: Optional[TrajectoryLimits] = None,
             geom: Optional[StewartGeometry] = None, *, rungs=RUNGS,
             search: bool = True) -> NominationReport:
    """Nominate ``config`` at ``limits`` (default: the generated launch
    constants) and, when refused and ``search``, find the nearest viable
    configuration.  Cached (LRU of :data:`CACHE_SIZE`, keyed on the config,
    every limits field and the rungs; the config fixes the planner config) for
    the default geometry only: a second call returns the SAME report object
    without solving, and a refused report searches once."""
    limits = default_limits() if limits is None else limits
    geom = _default_geom() if geom is None else geom
    use_cache = geom is _default_geom()
    key = (config.key(), limits_key(limits), tuple(rungs))
    rep = _CACHE.get(key) if use_cache else None
    if rep is not None:
        _CACHE.move_to_end(key)
    else:
        rep = _nominate_once(config, limits, geom, rungs)
        if use_cache:
            _CACHE[key] = rep
            while len(_CACHE) > CACHE_SIZE:
                _CACHE.popitem(last=False)
    if (search and not rep.ok and not rep.search_done
            and config.pattern not in NOT_NOMINATED_PATTERNS
            and NOTHING_TO_PLAN not in rep.refusals):
        _search_nearest(rep, limits, geom, rungs)
    return rep


def _nominate_once(config, limits, geom, rungs) -> NominationReport:
    t0 = time.perf_counter()
    if config.pattern in NOT_NOMINATED_PATTERNS:
        msg = 'NOT_NOMINATED: %s' % NOT_NOMINATED_PATTERNS[config.pattern]
        return NominationReport(config, False, [], [msg], [], 0.0, 0)
    try:
        schedule = compile_schedule(config)
    except ValueError as exc:
        return NominationReport(config, False, [], ['COMPILE: %s' % exc], [],
                                time.perf_counter() - t0, 0)
    cfg = segment_config(config)
    counter = [0]
    verdicts: List[SegmentVerdict] = []
    for f in rungs:
        verdicts.extend(plan_nominal(schedule, limits, geom, cfg, f,
                                     counter=counter))
    refusals = _collapse([v for v in verdicts if not v.ok])
    if not any(v.kind != REST for v in verdicts):
        # Nothing was walked (empty rungs, or a Schedule with no non-REST
        # Skill): never vacuously viable.
        refusals = [NOTHING_TO_PLAN] + refusals
    return NominationReport(config, not refusals, verdicts, refusals, [],
                            time.perf_counter() - t0, counter[0])


def _first_refusal(rep) -> Optional[SegmentVerdict]:
    for v in rep.verdicts:
        if not v.ok:
            return v
    return None


def _quick_pass(config, limits, geom, focus: SegmentVerdict, counter) -> bool:
    """The trial at the focus rung only, up to and including the focus
    Segment: a refusal anywhere on that prefix fails (only a pass here earns
    the full nomination that confirms it)."""
    try:
        schedule = compile_schedule(config)
    except ValueError:
        return False
    vs = plan_nominal(schedule, limits, geom, segment_config(config),
                      focus.flight_frac, stop_after=focus.idx, counter=counter)
    return all(v.ok for v in vs)


_PARAM_FIELD = {'apex': 'apex_m', 'separation': 'separation_mm',
                'dwell': 'dwell_s'}


def search_trials(config: NominationConfig) -> Dict[str, List[tuple]]:
    """``{parameter: [(direction, trial config), ...]}`` on the operator's
    grid, in search order (see the ``SEARCH_STEPS`` constants)."""
    out: Dict[str, List[tuple]] = {}
    c = config
    apex_tr = []
    for k in range(1, SEARCH_STEPS + 1):
        a = round(c.apex_m - APEX_STEP_M * k, 3)
        a2 = (None if c.apex_m_ball2 is None
              else round(c.apex_m_ball2 - APEX_STEP_M * k, 3))
        if a <= APEX_STEP_M or (a2 is not None and a2 <= APEX_STEP_M):
            break
        apex_tr.append(('down', dataclasses.replace(c, apex_m=a, apex_m_ball2=a2)))
    out['apex'] = apex_tr
    if c.pattern != 'self_toss':
        sep_tr = []
        for k in range(1, SEARCH_STEPS + 1):
            s = round(c.separation_mm - SEPARATION_STEP_MM * k, 3)
            if s <= 0.0:
                break
            sep_tr.append(('down', dataclasses.replace(c, separation_mm=s)))
        out['separation'] = sep_tr
    tf = sc.flight_s(min(c.apex_m, c.apex_m_ball2 or c.apex_m))
    ups = [round(c.dwell_s + DWELL_STEP_S * k, 3)
           for k in range(1, SEARCH_STEPS + 1)]
    downs = [round(c.dwell_s - DWELL_STEP_S * k, 3)
             for k in range(1, SEARCH_STEPS + 1)]
    if c.pattern in ('oval_1ball', 'oval_1ball_fed'):
        order = [('up', d) for d in ups]
    else:
        order = []
        for d_, u_ in zip(downs, ups):
            order += [('down', d_), ('up', u_)]
    out['dwell'] = [(direction, dataclasses.replace(c, dwell_s=d))
                    for direction, d in order if 0.0 < d < tf]
    return out


def workspace_ok(config: NominationConfig) -> bool:
    """Every Site's ``|xy| + WORKSPACE_HALF_EXTENT_MM`` inside
    ``CupCycleConfig().workspace_xy_m`` (``columns_sites`` ValueError ->
    False).  Self-toss: True (its Site does not move with a parameter)."""
    if config.pattern == 'self_toss':
        return True
    try:
        sites = st.columns_sites(
            config.separation_mm,
            config.angle_deg if config.pattern == 'columns_turned' else 0.0)
    except ValueError:
        return False
    lim = float(cc.CupCycleConfig().workspace_xy_m) * 1000.0
    return all(math.hypot(s.cup_mm[0], s.cup_mm[1]) + WORKSPACE_HALF_EXTENT_MM
               <= lim + 1e-9 for s in sites)


def _search_nearest(rep: NominationReport, limits, geom, rungs) -> None:
    """Fill ``rep.nearest`` (one entry per parameter for which a viable value
    exists within :data:`SEARCH_STEPS`) and the search bookkeeping."""
    t0 = time.perf_counter()
    focus = _first_refusal(rep) if rep.verdicts else None  # COMPILE: no focus
    counter = [0]
    n_full = 0
    nearest = []
    trace = []
    for param, trials in search_trials(rep.config).items():
        for direction, trial in trials:
            val = getattr(trial, _PARAM_FIELD[param])
            if param == 'separation' and not workspace_ok(trial):
                trace.append((param, val, 'workspace'))
                continue
            if focus is not None and not _quick_pass(trial, limits, geom,
                                                     focus, counter):
                trace.append((param, val, 'quick-refused'))
                continue
            full = nominate(trial, limits, geom, rungs=rungs, search=False)
            n_full += 1
            counter[0] += full.solves
            trace.append((param, val, 'OK' if full.ok else 'refused'))
            if full.ok:
                nearest.append((param, val, full.summary()))
                break
    rep.nearest = nearest
    rep.search_done = True
    rep.searched = n_full
    rep.search_solves = counter[0]
    rep.search_wall_s = time.perf_counter() - t0
    rep.search_trace = trace
    rep.search_note = ('' if nearest else
                       'no viable configuration within %d steps of apex / '
                       'separation / dwell' % SEARCH_STEPS)
