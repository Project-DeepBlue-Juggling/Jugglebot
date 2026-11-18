"""The ``human_catch`` Pattern's policy: when to plan a Catch for a human Throw,
where the Platform waits, and what ends the Attempt (PWAY-11 P2, U4;
``design-human-catch.md`` sections 2.2-2.3).

PURE: no ROS, no clock of its own (every instant is passed in), no executor and
no planner.  The node glue (``skill_node._run_human_catch`` / ``_on_human_throw``,
unit U5a) and the offline replay (``tools/probes/human_catch_replay.py``, U6)
both drive this one class, so the rules the replay scores are the rules the
node runs.

**Rules (section 2.3).**

* A, association: ignore an event whose Landing is farther than
  ``assoc_radius_mm`` from the wait Site W ("not thrown at the robot"), and
  ignore every event unless the phase is WAITING (``BUSY``: one Ball at a
  time; a second Ball or a phantom during a Catch never steals it).
* B, first plan: only revision 0 of a key (``(track_id, flight_seq)``) may
  start a Catch, and only with ``lead = landing_time - now >=
  first_lead_s`` and ``fit_age_s <= max_fit_age_s``.  A key whose revision 0
  arrived too late is never planned: a later revision does not get a second
  chance (the lead only shrinks).
* D, no admissible Catch: the install's own gate is the admissibility check
  (I-ADMIT-1); the policy never pre-checks.  A refused install moves nothing.
* E, continue or end: an ALLOWLIST of per-Throw end codes continues the
  Attempt; every other code, an unknown one included, ENDS it.
* F, HOLDING waits on the DEBOUNCED cup: EMPTY continuously for
  ``clear_s``; UNKNOWN never counts as EMPTY (C-POSSESS-1) and restarts the
  debounce, so a dead sensor leaves the Cup holding.
* Wait Site (section 2.2): the mean of the last ``mean_n`` Landings that passed
  association, clamped to within ``wait_radius_mm`` of ``home_xy_mm`` and into the cup workspace box; applied
  only when the Platform returns (RETURNING) and only when it moved at least
  ``wait_deadband_mm``.
* G, Memory: this Pattern writes none.  :attr:`HumanCatchPolicy.writes_memory`
  is ``False`` and the node builds no Learner for it.

**Every number below is an OWNER DECISION still to be taken.**  Each is a
constructor parameter whose default is the design's PROPOSED value (named
PROPOSED in its docstring line); nothing here is a limit, margin, guard or
gain of the planner or the machine.

Python 3.8; no ROS imports (the ``motion/`` rule).
"""
from __future__ import annotations

import collections
import dataclasses
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

# -- phases --------------------------------------------------------------

OPENING = 'OPENING'
WAITING = 'WAITING'
CATCHING = 'CATCHING'
HOLDING = 'HOLDING'
RETURNING = 'RETURNING'
PHASES = (OPENING, WAITING, CATCHING, HOLDING, RETURNING)

# -- decisions on an event ------------------------------------------------

PLAN = 'PLAN'
IGNORE = 'IGNORE'

#: Decline reasons (``Decision.reason``).
NOT_AT_ROBOT = 'NOT_AT_ROBOT'          # rule A: farther than assoc_radius_mm
BUSY = 'BUSY'                          # rule A: phase is not WAITING
NOT_FIRST_REVISION = 'NOT_FIRST_REVISION'   # rule B: revision > 0 of a new key
LEAD_TOO_SHORT = 'LEAD_TOO_SHORT'      # rule B: lead < first_lead_s
STALE_FIT = 'STALE_FIT'                # rule B: fit_age_s > max_fit_age_s
KEY_DECLINED = 'KEY_DECLINED'          # a later revision of a declined key
KEY_CLAIMED = 'KEY_CLAIMED'            # a later revision of the claimed key

# -- per-Throw end codes ---------------------------------------------------

#: Rule E's ALLOWLIST: a per-Throw executor ending in one of these CONTINUES
#: the Attempt.  The first two are verdicts; the rest are what a refused first
#: install returns (``executor.install_segment`` / ``unified_cycle``
#: outcomes).  The design leaves the executor's exact end code for a refused
#: first install unverified (section 7), so BOTH the bare gate codes and the
#: executor's ``REJECTED_CYCLE_INFEASIBLE`` wrapper are listed; anything not
#: listed (STOPPED, ABORTED_*, LANE_REFUSED, every other REJECTED_*
#: precondition, a code nobody has seen) ENDS the Attempt.
CONTINUE_CODES = frozenset((
    'MISSED_CATCH', 'NO_LANDING',
    'REJECTED_CYCLE_INFEASIBLE',
    'LIMIT_VEL', 'LIMIT_ACC', 'LIMIT_JERK', 'WORKSPACE', 'UNREACHABLE',
    'WINDOW_TOO_SHORT', 'ORIGIN_TOO_LATE', 'SPLICE_TOO_LATE',
    'STALE_STATE', 'CATCH_CLOSING_SPEED', 'CATCH_AXIS', 'SETTLE_SITE',
    'INFEASIBLE', 'MARGIN',
))

#: What a CAUGHT per-Throw executor ends with.
CAUGHT_CODES = frozenset(('COMPLETED', 'CAUGHT', 'OK'))

# -- per-Throw outcomes -----------------------------------------------------

TO_HOLDING = 'HOLDING'
TO_RETURNING = 'RETURNING'
TO_WAITING = 'WAITING'
END_ATTEMPT = 'END_ATTEMPT'

# -- cup states (the I-POSS-1 seam's vocabulary) ----------------------------

CUP_EMPTY = 'EMPTY'
CUP_SEATED = 'SEATED'
CUP_UNKNOWN = 'UNKNOWN'


@dataclasses.dataclass(frozen=True)
class HumanThrowEvt:
    """One Tracker event, as the policy needs it (``HumanThrowEvent.msg``
    after the node has put the Landing in the schedule frame and the wall
    clock): ``landing_mm`` is the Fit's Landing xyz, ``landing_time_s`` its
    Catch-plane crossing, ``fit_age_s`` the age of the Fit's last sample."""

    track_id: int
    flight_seq: int
    revision: int
    landing_mm: Tuple[float, float, float]
    landing_vel_mm_s: Tuple[float, float, float]
    landing_time_s: float
    fit_age_s: float = 0.0
    final: bool = False

    @property
    def key(self) -> Tuple[int, int]:
        return (int(self.track_id), int(self.flight_seq))


@dataclasses.dataclass(frozen=True)
class Decision:
    """What the policy decided for one event.  ``kind`` is :data:`PLAN`
    (compile and swap in a per-Throw Schedule for this event) or
    :data:`IGNORE`; ``reason`` names why (always set for IGNORE); ``log`` is
    the ONE line to log for this decision or None when this key's decline was
    already logged (every decline is logged once per key, with its reason).
    ``lead_s`` / ``dist_mm`` are the lead and the distance from W the rule
    read (``nan`` when not computed)."""

    kind: str
    reason: str
    key: Tuple[int, int]
    lead_s: float = float('nan')
    dist_mm: float = float('nan')
    log: Optional[str] = None


@dataclasses.dataclass(frozen=True)
class Outcome:
    """What the policy decided when a per-Throw executor ended."""

    next: str            # TO_HOLDING | TO_RETURNING | TO_WAITING | END_ATTEMPT
    reason: str
    log: Optional[str] = None


class HumanCatchPolicy:
    """The ``human_catch`` Pattern's decisions, one instance per Attempt.

    Every keyword is an owner decision; the default is the design's PROPOSED
    value (``design-human-catch.md`` section 4).  None is a planner or machine
    constant.

    :param home_xy_mm: the home Site's xy (default the Pattern's Site 0, the
        columns Pattern's P1 at (-50, 0) mm in the Platform frame).  The wait
        Site starts here and never leaves ``wait_radius_mm`` of it.
    :param first_lead_s: D3, PROPOSED 0.55 s.  The shortest lead (landing time
        minus now) at which revision 0 may start a Catch.  The sweep
        (``tools/probes/human_catch_reach.py``) is the evidence: below the
        planner's own floor the install refuses everything.
    :param max_fit_age_s: rule B, PROPOSED 0.05 s: the oldest Fit a first plan
        may start from.
    :param assoc_radius_mm: D4, PROPOSED 400 mm: farther from W than this is
        "not thrown at the robot".
    :param reaim_authority_mm: D5, PROPOSED 50 mm per axis about the first
        event's Landing: the node passes it as the per-Throw executor's
        ``lateral_authority_m``.  The policy only carries it.
    :param resend_max: D6, PROPOSED 2: re-sends per Catch (the node passes it
        as ``resend_max_per_catch``).  Carried, not used here.
    :param mean_n: D7, PROPOSED 5: Landings in the wait-Site mean.
    :param wait_radius_mm: D7, PROPOSED 150 mm: the wait Site stays within
        this of home (0 = a fixed Site).
    :param wait_deadband_mm: D7, PROPOSED 20 mm: W moves only when the
        clamped mean differs from the current W by at least this.
    :param clear_s: D8, PROPOSED 0.5 s: continuous EMPTY before RETURNING.
    :param return_s: D8, PROPOSED 1.0 s: the RETURNING REST's duration
        (carried for the node's compile, not used here).
    :param nominal_apex_m: D9, PROPOSED 1.9 m absolute (1.07 m above the
        Catch plane, 4.58 m/s): the arrival the nomination plans (U5b reads
        :meth:`nominal_arrival_vel_mm_s`).
    :param n_throws: the goal's claimed-event count (0 = unbounded; the Rung
        uses 15).
    :param workspace_half_mm: the planner's per-axis cup workspace box about
        the Platform centre (``CupCycleConfig.workspace_xy_m`` = 0.15 m; a test
        pins the two together).  NOT in the design: the reach sweep found that
        every Landing or settle Site outside it is refused ``SETTLE_SITE``
        whatever the lead, so the wait Site is additionally clipped into it
        (a W outside the box would make its own REST unplannable).
    """

    #: Rule G: this Pattern writes no Memory row (no Throw, no row).
    writes_memory = False

    def __init__(self, *, home_xy_mm=(-50.0, 0.0),
                 first_lead_s: float = 0.55,
                 max_fit_age_s: float = 0.05,
                 assoc_radius_mm: float = 400.0,
                 reaim_authority_mm: float = 50.0,
                 resend_max: int = 2,
                 mean_n: int = 5,
                 wait_radius_mm: float = 150.0,
                 wait_deadband_mm: float = 20.0,
                 clear_s: float = 0.5,
                 return_s: float = 1.0,
                 nominal_apex_m: float = 1.9,
                 n_throws: int = 0,
                 catch_plane_z_mm: float = 830.0,
                 workspace_half_mm: float = 150.0):
        self.home_xy_mm = np.asarray(home_xy_mm, dtype=float)[:2].copy()
        self.first_lead_s = float(first_lead_s)
        self.max_fit_age_s = float(max_fit_age_s)
        self.assoc_radius_mm = float(assoc_radius_mm)
        self.reaim_authority_mm = float(reaim_authority_mm)
        self.resend_max = int(resend_max)
        self.mean_n = int(mean_n)
        self.wait_radius_mm = float(wait_radius_mm)
        self.wait_deadband_mm = float(wait_deadband_mm)
        self.clear_s = float(clear_s)
        self.return_s = float(return_s)
        self.nominal_apex_m = float(nominal_apex_m)
        self.n_throws = int(n_throws)
        self._plane_z_mm = float(catch_plane_z_mm)
        self.workspace_half_mm = float(workspace_half_mm)
        if not self.workspace_half_mm > 0.0:
            raise ValueError('workspace_half_mm must be > 0')
        if np.any(np.abs(self.home_xy_mm) > self.workspace_half_mm):
            raise ValueError('home_xy_mm %r is outside the +-%.0f mm cup '
                             'workspace' % (tuple(self.home_xy_mm),
                                            self.workspace_half_mm))
        for name in ('first_lead_s', 'max_fit_age_s', 'assoc_radius_mm',
                     'reaim_authority_mm', 'wait_radius_mm',
                     'wait_deadband_mm', 'clear_s', 'return_s'):
            v = getattr(self, name)
            if not (math.isfinite(v) and v >= 0.0):
                raise ValueError('%s must be finite and >= 0, got %r'
                                 % (name, v))
        if self.mean_n < 1:
            raise ValueError('mean_n must be >= 1, got %r' % (mean_n,))
        if self.resend_max < 0 or self.n_throws < 0:
            raise ValueError('resend_max and n_throws must be >= 0')
        if not self.nominal_apex_m * 1000.0 > self._plane_z_mm:
            raise ValueError('nominal_apex_m must be above the Catch plane')

        self.phase = OPENING
        self.wait_xy_mm = self.home_xy_mm.copy()
        #: key -> latest association-passing Landing xy, oldest first.
        self._landings: 'collections.OrderedDict' = collections.OrderedDict()
        self._seen: Dict[Tuple[int, int], str] = {}   # key -> first verdict
        self._logged = set()
        self.claimed_key: Optional[Tuple[int, int]] = None
        self.claimed = 0
        self._empty_since_s: Optional[float] = None
        self.declines: List[Decision] = []

    # -- the rules ---------------------------------------------------------

    def nominal_arrival_vel_mm_s(self) -> np.ndarray:
        """The vertical arrival the nomination plans (D9)."""
        rise = self.nominal_apex_m * 1000.0 - self._plane_z_mm
        return np.array([0.0, 0.0, -math.sqrt(2.0 * 9810.0 * rise)])

    def _decline(self, evt, reason, now_s, lead, dist, text) -> Decision:
        first = (evt.key, reason) not in self._logged
        self._logged.add((evt.key, reason))
        d = Decision(IGNORE, reason, evt.key, lead, dist,
                     ('HUMAN-CATCH-DECLINED id %d seq %d: %s' % (
                         evt.key[0], evt.key[1], text)) if first else None)
        self.declines.append(d)
        return d

    def on_event(self, evt: HumanThrowEvt, now_s: float) -> Decision:
        """Rules A and B for one Tracker event.  Returns PLAN (the caller
        compiles ``schedule.compile_human_catch`` from the event and claims
        the key; the policy has already moved to CATCHING) or IGNORE with the
        reason.  Every event that passes association (rule A's radius) updates
        the key's Landing in the running mean, whatever else it decides."""
        now = float(now_s)
        xy = np.asarray(evt.landing_mm, dtype=float)[:2]
        dist = float(np.linalg.norm(xy - self.wait_xy_mm))
        lead = float(evt.landing_time_s) - now
        if dist > self.assoc_radius_mm:
            return self._decline(evt, NOT_AT_ROBOT, now, lead, dist,
                                 'Landing %.0f mm from the wait Site > %.0f '
                                 '(not thrown at the robot)'
                                 % (dist, self.assoc_radius_mm))
        # a real Throw at the robot: it informs the wait Site whatever happens
        self._landings[evt.key] = xy.copy()
        self._landings.move_to_end(evt.key)
        while len(self._landings) > max(self.mean_n, 1):
            self._landings.popitem(last=False)

        if evt.key == self.claimed_key:
            return self._decline(evt, KEY_CLAIMED, now, lead, dist,
                                 'a later revision of the claimed Throw')
        if evt.key in self._seen:
            return self._decline(evt, KEY_DECLINED, now, lead, dist,
                                 'a later revision of a declined Throw (%s)'
                                 % self._seen[evt.key])
        if self.phase != WAITING:
            # not remembered as a verdict: BUSY is about the phase, and a
            # revision 0 that arrives busy is never planned later (B)
            self._seen[evt.key] = BUSY
            return self._decline(evt, BUSY, now, lead, dist,
                                 'phase %s, one Ball at a time' % self.phase)
        if int(evt.revision) != 0:
            self._seen[evt.key] = NOT_FIRST_REVISION
            return self._decline(evt, NOT_FIRST_REVISION, now, lead, dist,
                                 'revision %d is not the first of its key'
                                 % evt.revision)
        if float(evt.fit_age_s) > self.max_fit_age_s:
            self._seen[evt.key] = STALE_FIT
            return self._decline(evt, STALE_FIT, now, lead, dist,
                                 'Fit age %.3f s > %.3f s'
                                 % (evt.fit_age_s, self.max_fit_age_s))
        if not lead >= self.first_lead_s:
            self._seen[evt.key] = LEAD_TOO_SHORT
            return self._decline(evt, LEAD_TOO_SHORT, now, lead, dist,
                                 'lead %.3f s < first-plan lead %.3f s'
                                 % (lead, self.first_lead_s))
        self._seen[evt.key] = PLAN
        self.claimed_key = evt.key
        self.claimed += 1
        self.phase = CATCHING
        return Decision(PLAN, 'OK', evt.key, lead, dist, None)

    # -- phase transitions --------------------------------------------------

    def opening_done(self) -> None:
        """The opening REST is at rest at W: OPENING -> WAITING."""
        if self.phase != OPENING:
            raise ValueError('opening_done in phase %s' % self.phase)
        self.phase = WAITING

    def finished(self) -> bool:
        """True once ``n_throws`` events have been claimed AND the machine is
        back in WAITING (a bounded Attempt ends between Throws, never in one)."""
        return (self.n_throws > 0 and self.claimed >= self.n_throws
                and self.phase == WAITING)

    def on_executor_done(self, code: str, moved: bool = True) -> Outcome:
        """The per-Throw executor ended with ``code`` (``moved`` False when
        the install was refused and nothing was commanded).  CATCHING ->
        HOLDING (caught), RETURNING (missed or refused after a move), WAITING
        (refused, nothing moved), or END_ATTEMPT (rule E: any code outside
        the allowlist)."""
        if self.phase != CATCHING:
            raise ValueError('on_executor_done in phase %s' % self.phase)
        code = str(code)
        if code in CAUGHT_CODES:
            self.phase = HOLDING
            self._empty_since_s = None
            return Outcome(TO_HOLDING, code)
        if code not in CONTINUE_CODES:
            self.phase = WAITING
            return Outcome(END_ATTEMPT, code,
                           'HUMAN-CATCH-ENDED %s: not an allowlisted per-Throw '
                           'code (rule E)' % code)
        if moved:
            self.phase = RETURNING
            return Outcome(TO_RETURNING, code)
        self.phase = WAITING
        return Outcome(TO_WAITING, code,
                       'HUMAN-CATCH-REFUSED %s (nothing moved)' % code)

    def observe_cup(self, state: str, now_s: float) -> bool:
        """Rule F: feed the debounced cup in HOLDING.  Returns True (and
        moves to RETURNING) once the cup has read EMPTY continuously for
        ``clear_s``.  SEATED restarts the debounce; UNKNOWN is NOT empty and
        restarts it too (C-POSSESS-1)."""
        if self.phase != HOLDING:
            return False
        if state != CUP_EMPTY:
            self._empty_since_s = None
            return False
        if self._empty_since_s is None:
            self._empty_since_s = float(now_s)
        if float(now_s) - self._empty_since_s >= self.clear_s:
            self.phase = RETURNING
            self._empty_since_s = None
            return True
        return False

    def mean_landing_xy(self) -> Optional[np.ndarray]:
        """The mean of the last ``mean_n`` recorded Landings, or None."""
        if not self._landings:
            return None
        pts = list(self._landings.values())[-self.mean_n:]
        return np.mean(np.asarray(pts, dtype=float), axis=0)

    def target_wait_xy_mm(self) -> np.ndarray:
        """Where W WOULD go now: the mean (home with no Landings) clamped to
        ``wait_radius_mm`` about home, then into the workspace box."""
        m = self.mean_landing_xy()
        if m is None:
            return self.home_xy_mm.copy()
        off = m - self.home_xy_mm
        n = float(np.linalg.norm(off))
        if n > self.wait_radius_mm:
            off = off * (self.wait_radius_mm / n)
        return np.clip(self.home_xy_mm + off, -self.workspace_half_mm,
                       self.workspace_half_mm)

    def return_done(self) -> None:
        """The RETURNING REST is at rest: RETURNING -> WAITING."""
        if self.phase != RETURNING:
            raise ValueError('return_done in phase %s' % self.phase)
        self.phase = WAITING

    def begin_return(self) -> Tuple[np.ndarray, bool]:
        """On entering RETURNING: ``(xy the REST should go to, moved)``.  W is
        updated only here and only when the clamped mean moved at least
        ``wait_deadband_mm`` from the current W; otherwise the REST returns to
        the current W."""
        tgt = self.target_wait_xy_mm()
        if float(np.linalg.norm(tgt - self.wait_xy_mm)) >= self.wait_deadband_mm:
            self.wait_xy_mm = tgt
            return tgt.copy(), True
        return self.wait_xy_mm.copy(), False
