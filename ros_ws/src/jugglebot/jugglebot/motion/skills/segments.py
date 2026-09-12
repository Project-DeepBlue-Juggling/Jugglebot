"""One skill, turned into a rest-terminal ``CyclePlan`` (plan § 2.2).

A :class:`Segment` is the trajectory the shared chain (``unified_cycle.
plan_cycle`` / ``extend``) produces for one skill: THROW = a LAUNCH into a
release, then a SETTLE tail; CATCH = a single LANDING, or — when it carries the
next same-site throw (``CatchTerminal.then_throw``) — a STEADY window holding
both the touch-down and the release, then the same SETTLE tail; REST = a single
SETTLE.
All three are ALWAYS rest-terminal — the machine never streams a skill that
ends mid-motion, because there is nothing scheduled to chain onto it yet at
this rung. Pure Python + numpy, no ROS imports.
"""

from __future__ import annotations

import dataclasses
from typing import Optional

import numpy as np

from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.trajectory import cup_cycle as cc
from jugglebot.motion.trajectory.cycle_plan import CyclePlan

THROW = 'THROW'
CATCH = 'CATCH'
REST = 'REST'
KINDS = (THROW, CATCH, REST)

#: The SETTLE/runway length (s) after a THROW's release or a CATCH's touch-down
#: before the segment is rest-terminal.
#:
#: PROBED, not guessed: the smallest of ``{0.30, 0.40, 0.50, 0.60}`` for which
#: all three of the plan's reference scenarios pass ``validate_cycle`` at the
#: owner's R2 operating point (leg 300/5000/200000 mm, hand acc 3500 rev/s²) —
#: (1) a THROW's LAUNCH (0.4 s, 0.9 m apex) from rest at 750 mm, plus this
#: SETTLE tail; (2) a THROW's LAUNCH (0.30 s, the dwell) from a POST-CATCH rest
#: seed (cup at ``unified_cycle.SETTLE_CUP_Z_MM``), plus this SETTLE tail;
#: (3) a CATCH's LANDING, touch-down at 0.278 s with the arrival velocity of a
#: 0.857 s vertical flight, whose own period already embeds this tail.
#:
#: (date, command, result): 2026-09-12,
#: a one-off ``probe_rest_tail.py`` (uncommitted and not preserved — the table
#: below IS the result; venv interpreter) — **every** candidate ACCEPTs all three
#: scenarios:
#:
#: ==========  ================  ======================  =====
#: rest_tail_s  throw_from_rest   throw_from_post_catch    catch
#: ==========  ================  ======================  =====
#: 0.30         ACCEPT            ACCEPT                  ACCEPT
#: 0.40         ACCEPT            ACCEPT                  ACCEPT
#: 0.50         ACCEPT            ACCEPT                  ACCEPT
#: 0.60         ACCEPT            ACCEPT                  ACCEPT
#: ==========  ================  ======================  =====
#:
#: so the smallest candidate (0.30 s) is CHOSEN — the owner's R2 operating
#: point already has enough leg/hand headroom that the runway length is not
#: the binding constraint; nothing shorter than 0.30 s was probed because
#: 0.30 s is also the dwell (plan § 0) and a shorter tail has no scenario that
#: needs it yet.
REST_TAIL_S = 0.30


def _vec3(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (3,):
        raise ValueError('%s must be a 3-vector, got shape %s'
                          % (name, np.shape(value)))
    if not np.all(np.isfinite(arr)):
        raise ValueError('%s must be finite, got %r' % (name, arr.tolist()))
    return arr


@dataclasses.dataclass(frozen=True)
class ThrowTerminal:
    """A THROW's release: site, ballistic target, flight time, and the LAUNCH
    window length (seed → release, seconds)."""

    site_mm: np.ndarray
    target_mm: np.ndarray
    flight_s: float
    t_release_s: float

    def __post_init__(self):
        object.__setattr__(self, 'site_mm', _vec3(self.site_mm, 'site_mm'))
        object.__setattr__(self, 'target_mm', _vec3(self.target_mm, 'target_mm'))
        if not float(self.flight_s) > 0.0:
            raise ValueError('flight_s must be > 0, got %r' % (self.flight_s,))
        if not float(self.t_release_s) > 0.0:
            raise ValueError('t_release_s must be > 0, got %r'
                              % (self.t_release_s,))


@dataclasses.dataclass(frozen=True)
class ThrowAfterCatch:
    """The release a CATCH carries out of the ball it just seated.

    Times are on the SEGMENT clock, like every other terminal here.  The
    measured reason this is one window rather than two segments is on
    ``schedule.ThenThrow``; the shape is a :data:`~jugglebot.motion.
    unified_cycle.STEADY` window (catch and release solved together) plus the
    same SETTLE tail a THROW carries.
    """

    t_release_s: float
    site_mm: np.ndarray
    target_mm: np.ndarray
    flight_s: float

    def __post_init__(self):
        object.__setattr__(self, 'site_mm', _vec3(self.site_mm, 'site_mm'))
        object.__setattr__(self, 'target_mm', _vec3(self.target_mm, 'target_mm'))
        if not float(self.flight_s) > 0.0:
            raise ValueError('flight_s must be > 0, got %r' % (self.flight_s,))


@dataclasses.dataclass(frozen=True)
class CatchTerminal:
    """A CATCH's touch-down: the ball's observed arrival, and the site the
    cup rests at afterwards."""

    landing_mm: np.ndarray
    landing_vel_mm_s: np.ndarray
    t_land_s: float
    rest_site_mm: np.ndarray
    #: The same-site throw this catch carries, or ``None`` for a standalone
    #: catch (the last catch of an attempt, and R4's reload).  With it the
    #: segment is a STEADY window + SETTLE tail; without it, a LANDING.
    then_throw: Optional[ThrowAfterCatch] = None

    def __post_init__(self):
        object.__setattr__(self, 'landing_mm', _vec3(self.landing_mm, 'landing_mm'))
        object.__setattr__(self, 'landing_vel_mm_s',
                            _vec3(self.landing_vel_mm_s, 'landing_vel_mm_s'))
        object.__setattr__(self, 'rest_site_mm',
                            _vec3(self.rest_site_mm, 'rest_site_mm'))
        if not float(self.t_land_s) > 0.0:
            raise ValueError('t_land_s must be > 0, got %r' % (self.t_land_s,))
        if (self.then_throw is not None
                and not float(self.then_throw.t_release_s) > float(self.t_land_s)):
            raise ValueError(
                'the carried release (%.4f s) must come after the touch-down '
                '(%.4f s) — the ball has to be in the cup before it can be '
                'thrown' % (self.then_throw.t_release_s, self.t_land_s))


@dataclasses.dataclass(frozen=True)
class RestTerminal:
    """A standalone REST: hold at ``rest_site_mm`` by ``t_rest_s``."""

    rest_site_mm: np.ndarray
    t_rest_s: float

    def __post_init__(self):
        object.__setattr__(self, 'rest_site_mm',
                            _vec3(self.rest_site_mm, 'rest_site_mm'))
        if not float(self.t_rest_s) > 0.0:
            raise ValueError('t_rest_s must be > 0, got %r' % (self.t_rest_s,))


@dataclasses.dataclass(frozen=True)
class SegmentConfig:
    """The one knob a segment build needs beyond the terminal itself."""

    rest_tail_s: float = REST_TAIL_S


@dataclasses.dataclass(frozen=True)
class Segment:
    """A rest-terminal ``CyclePlan`` plus the facts the executor needs off it.

    ``splice_k`` is the knot on the ACTIVE (already-installed) plan this
    segment replaces from; :func:`plan_segment` always returns 0 (a freshly
    built segment has not been spliced against an installed plan yet — that is
    the executor's job, against the live plan it is holding).
    """

    kind: str
    plan: CyclePlan
    meta: uc.CycleMeta
    splice_k: int
    #: Release/catch instant on the SEGMENT clock; ``None`` for REST.
    event_t_s: Optional[float]
    takeoff_vel_mm_s: Optional[np.ndarray]
    rest_site_mm: np.ndarray
    #: A CATCH-with-throw's RELEASE instant on the segment clock (its
    #: ``event_t_s`` is the touch-down — a segment with two events needs both
    #: named).  ``None`` for every other shape, including a standalone CATCH.
    release_t_s: Optional[float] = None
    #: The QP working set from the segment's own solve, to hand to the next
    #: ``plan_segment`` call as its ``warm_start`` (owner decision: carried on
    #: the ``Segment``, no cache).
    warm_start: Optional['cc.SolverState'] = None


def plan_segment(kind: str, seed: uc.CycleState, terminal, cfg: SegmentConfig,
                  limits, geom, *, warm_start=None) -> Segment:
    """Plan one skill into a rest-terminal :class:`Segment`.

    ``terminal`` is a :class:`ThrowTerminal` / :class:`CatchTerminal` /
    :class:`RestTerminal` matching ``kind``. Refusals from the underlying chain
    (``unified_cycle.CycleInfeasible``) propagate UNCHANGED — the layer's own
    code is the refusal, so nothing here wraps or re-codes it.
    """
    if kind == THROW:
        return _plan_throw(seed, terminal, cfg, limits, geom, warm_start)
    if kind == CATCH:
        return _plan_catch(seed, terminal, cfg, limits, geom, warm_start)
    if kind == REST:
        return _plan_rest(seed, terminal, cfg, limits, geom, warm_start)
    raise ValueError('unknown skill kind %r (expected one of %s)'
                      % (kind, KINDS))


def _plan_throw(seed, terminal: ThrowTerminal, cfg: SegmentConfig,
                 limits, geom, warm_start) -> Segment:
    # The REST site of a THROW is its own site at REST_CUP_Z (unified_cycle's
    # settle clamp) — same xy the ball just left from.
    rest_mm = np.array([terminal.site_mm[0], terminal.site_mm[1],
                        uc.SETTLE_CUP_Z_MM])
    goals_a = uc.CycleGoals(period_s=terminal.t_release_s,
                            throw_site_mm=terminal.site_mm,
                            throw_target_mm=terminal.target_mm,
                            flight_s=terminal.flight_s,
                            settle_site_mm=rest_mm)
    plan_a, meta_a = uc.plan_launch(goals_a, seed, limits, geom,
                                    warm_start=warm_start)
    seed_b = uc.release_state_from_meta(meta_a, plan_a)
    goals_b = uc.CycleGoals(period_s=cfg.rest_tail_s, settle_site_mm=rest_mm)
    plan_b, meta_b = uc.plan_settle(goals_b, seed_b, limits, geom)
    plan, meta = uc.extend(plan_a, meta_a, plan_b, meta_b, limits, geom)
    return Segment(kind=THROW, plan=plan, meta=meta, splice_k=0,
                   event_t_s=meta.t_release_s,
                   takeoff_vel_mm_s=meta.release_vel_mm_s,
                   rest_site_mm=rest_mm, warm_start=meta.warm_start)


def _plan_catch(seed, terminal: CatchTerminal, cfg: SegmentConfig,
                 limits, geom, warm_start) -> Segment:
    if terminal.then_throw is not None:
        return _plan_catch_throw(seed, terminal, cfg, limits, geom, warm_start)
    goals = uc.CycleGoals(period_s=terminal.t_land_s + cfg.rest_tail_s,
                          catch_site_mm=terminal.landing_mm,
                          catch_vel_mm_s=terminal.landing_vel_mm_s,
                          catch_t_s=terminal.t_land_s,
                          settle_site_mm=terminal.rest_site_mm)
    plan, meta = uc.plan_landing(goals, seed, limits, geom,
                                 warm_start=warm_start)
    return Segment(kind=CATCH, plan=plan, meta=meta, splice_k=0,
                   event_t_s=meta.t_catch_s, takeoff_vel_mm_s=None,
                   rest_site_mm=terminal.rest_site_mm, warm_start=meta.warm_start)


def _plan_catch_throw(seed, terminal: CatchTerminal, cfg: SegmentConfig,
                       limits, geom, warm_start) -> Segment:
    """A CATCH that carries the next same-site throw: ONE STEADY window (the
    touch-down and the release solved together) plus the same SETTLE tail
    ``_plan_throw`` uses — still rest-terminal, like every other segment."""
    tt = terminal.then_throw
    goals_a = uc.CycleGoals(period_s=tt.t_release_s,
                            throw_site_mm=tt.site_mm,
                            throw_target_mm=tt.target_mm,
                            flight_s=tt.flight_s,
                            catch_site_mm=terminal.landing_mm,
                            catch_vel_mm_s=terminal.landing_vel_mm_s,
                            catch_t_s=terminal.t_land_s)
    plan_a, meta_a = uc.plan_steady(goals_a, seed, limits, geom,
                                    warm_start=warm_start)
    seed_b = uc.release_state_from_meta(meta_a, plan_a)
    goals_b = uc.CycleGoals(period_s=cfg.rest_tail_s,
                            settle_site_mm=terminal.rest_site_mm)
    plan_b, meta_b = uc.plan_settle(goals_b, seed_b, limits, geom)
    plan, meta = uc.extend(plan_a, meta_a, plan_b, meta_b, limits, geom)
    return Segment(kind=CATCH, plan=plan, meta=meta, splice_k=0,
                   event_t_s=meta.t_catch_s,
                   takeoff_vel_mm_s=meta.release_vel_mm_s,
                   rest_site_mm=terminal.rest_site_mm,
                   release_t_s=meta.t_release_s, warm_start=meta.warm_start)


def _plan_rest(seed, terminal: RestTerminal, cfg: SegmentConfig,
                limits, geom, warm_start) -> Segment:
    goals = uc.CycleGoals(period_s=terminal.t_rest_s,
                          settle_site_mm=terminal.rest_site_mm)
    plan, meta = uc.plan_settle(goals, seed, limits, geom,
                                warm_start=warm_start)
    return Segment(kind=REST, plan=plan, meta=meta, splice_k=0,
                   event_t_s=None, takeoff_vel_mm_s=None,
                   rest_site_mm=terminal.rest_site_mm, warm_start=meta.warm_start)
