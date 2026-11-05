"""The online admissibility check for a THROW command (plan
``plans/active/learner-generalisation.md`` P3a/P3c; ``motion/skills/INVARIANTS.md``
I-ADMIT-1 / I-ADMIT-2).

A THROW's command is ``u = (landing_xy_m, apex_m)``. Before it reaches the
platform it is judged by ONE real plan + gate of a stand-in Segment per
evaluated command (``executor.check_segment``, run in the solver worker), and
this module is the pure half of that check: :func:`margin` (the gate report's
leg headroom against :data:`MARGIN_FRAC` of the sitting limits, ``>= 0`` iff
the plan is admitted), :class:`CommandVerdict` (one evaluation),
:func:`resolve_command` (the two-solve step: the Learner's candidate, else the
model command, else a point on the line between them -- at most three
evaluations, never a loop) and :class:`AdmissibleError` (nothing admissible
could be resolved; the executor ends the attempt, fail closed).

History. Until 2026-11-06 the same question was answered offline: a sweep
(``tools/admissible_sweep.py``) wrote a box of admissible commands per (site
pair, apex band) at the sitting limits and the runtime only clipped into it.
That box, its loader, its stamps and its hash gate were retired with P3c
(``logbook/2026-11-06-learner-generalisation-p3c-box-retirement.md``): the
check above evaluates the command that is actually about to fly, so there is
no swept grid to go stale. C-HAND-3's six hand-metal bounds are still
enforced by ``motion.trajectory.throw_envelope.evaluate`` (not re-derived
here).

Pure Python + numpy. No ROS2 imports (the ``motion/`` rule).
"""

from __future__ import annotations

import dataclasses
import math
from typing import Tuple

import numpy as np

__all__ = [
    'AdmissibleError', 'MARGIN_FRAC',
    'CommandVerdict', 'Resolution', 'margin', 'resolve_command',
]

#: A plan is admitted only if every leg peak (velocity, acceleration, jerk)
#: clears the sitting limit by this fraction -- 10% headroom for solver
#: tolerance and the 80-sample-vs-dense-400 mesh-coarsening gap (I-PLAN-3), so
#: a command the check calls admissible cannot be one the live gate, sampled
#: slightly differently on the day, refuses.
MARGIN_FRAC = 0.9


class AdmissibleError(Exception):
    """No admissible command could be resolved (names the refused command)."""


# ---------------------------------------------------------------------------
# The command-time (online) check: margin + the resolution rule (P3a)
# ---------------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class CommandVerdict:
    """One REAL evaluation of a command ``u`` (at most one worker solve).

    ``ok`` = the plan succeeded, :func:`margin` ``>= 0`` and the throw envelope
    accepted; ``margin`` is ``nan`` when no gate report exists (a planner
    refusal without one, an envelope refusal); ``code`` is ``'OK'``,
    ``'MARGIN'``, a planner/gate code, ``'ENVELOPE:<bound>'`` or a transport
    code; ``evaluated`` is False when the check could not be evaluated at all
    (worker down, deadline, guard, service timeout) -- fail closed."""

    ok: bool
    margin: float
    code: str
    evaluated: bool


@dataclasses.dataclass(frozen=True)
class Resolution:
    """What :func:`resolve_command` settled on.

    ``u`` = ``(xy ndarray (2,) in metres, apex float in metres)``; ``how`` is
    ``'candidate'`` (the Learner's command passed), ``'stepped'`` (a point on
    the line toward the model command, confirmed by its own evaluation) or
    ``'model'`` (the model command, verified in this resolution); ``solves`` is
    the number of ``verdict_fn`` calls (1, 2 or 3); ``verdicts`` are those
    calls' verdicts in call order (candidate, model, stepped) for the log."""

    u: Tuple[np.ndarray, float]
    how: str
    solves: int
    verdicts: Tuple[CommandVerdict, ...]


def margin(report, limits, frac: float = MARGIN_FRAC) -> float:
    """The gate report's leg headroom against ``frac`` of the sitting limits:
    ``min over leg vel/acc/jerk of (frac L - peak) / (frac L)``; ``>= 0`` iff
    ``planner_cells.within_margin`` (the cell helpers' twin) admits the plan.  ``nan`` for a missing
    report.  The same expression as the probe's ``margin``
    (``online_check_replay.py``), so the two agree bit for bit."""
    if report is None:
        return float('nan')
    f = float(frac)
    return min(
        (f * limits.leg_vel_mmps - report.peak_leg_vel_mmps)
        / (f * limits.leg_vel_mmps),
        (f * limits.leg_acc_mmps2 - report.peak_leg_acc_mmps2)
        / (f * limits.leg_acc_mmps2),
        (f * limits.leg_jerk_mmps3 - report.peak_leg_jerk_mmps3)
        / (f * limits.leg_jerk_mmps3))


def _fmt_u(u) -> str:
    xy = np.asarray(u[0], dtype=float)
    return '(%.1f, %.1f) mm, apex %.3f m' % (xy[0] * 1e3, xy[1] * 1e3,
                                             float(u[1]))


def resolve_command(verdict_fn, u_cand, u_model, *,
                    backoff_m: float = 0.001) -> Resolution:
    """Resolve the Learner's candidate command against the online check.

    ``verdict_fn(u) -> CommandVerdict`` is ONE real evaluation.  The rule
    (`motion/skills/INVARIANTS.md` I-ADMIT-1; ``online_check_replay.py`` part B is the recipe):

    1. the candidate passes -> it, after 1 call;
    2. the candidate's check could not be evaluated -> :class:`AdmissibleError`
       (fail closed: an unchecked command never reaches the platform);
    3. the candidate IS the model command -> :class:`AdmissibleError` (the
       model command itself fails);
    4. the model command is evaluated (call 2); refused ->
       :class:`AdmissibleError`;
    5. step from the model command toward the candidate to the zero of the
       margin, linearly interpolated between the two REAL margins
       (``s = m0 / (m0 - m_u)``; no usable slope -- a nan margin, ``m_u >= 0``
       or ``m0 <= 0`` -- halves, ``s = 0.5``), clamped to [0, 1], backed off
       ``backoff_m`` along the line (apex interpolated with the same ``s``);
       ``s == 0`` after the back-off -> the model command (already verified);
    6. the stepped point is evaluated (call 3): passes -> it, else the model
       command.  Never a fourth call, never a loop.

    The candidate's own REAL margin stands in for the probe's affinely
    PREDICTED one (they agree to 0.003 on the probe's 15 refused candidates).
    A candidate that differs from the model command in apex only has no xy
    line to back off along, so its back-off consumes the whole line and the
    model command is returned after two calls.
    """
    xy_c = np.asarray(u_cand[0], dtype=float).reshape(2)
    xy_m = np.asarray(u_model[0], dtype=float).reshape(2)
    ap_c, ap_m = float(u_cand[1]), float(u_model[1])
    u_cand = (xy_c, ap_c)
    u_model = (xy_m, ap_m)

    v = verdict_fn(u_cand)
    if v.ok:
        return Resolution(u_cand, 'candidate', 1, (v,))
    if not v.evaluated:
        raise AdmissibleError('check unavailable: %s' % (v.code,))
    if np.array_equal(xy_c, xy_m) and ap_c == ap_m:
        raise AdmissibleError(
            'the model command %s is refused: %s (margin %.3f)'
            % (_fmt_u(u_model), v.code, v.margin))
    v0 = verdict_fn(u_model)
    if not v0.ok:
        raise AdmissibleError(
            'candidate %s refused %s (margin %.3f) and the model command %s '
            'refused %s (margin %.3f)%s'
            % (_fmt_u(u_cand), v.code, v.margin, _fmt_u(u_model), v0.code,
               v0.margin, '' if v0.evaluated else ' (check unavailable)'))
    m_u, m0 = float(v.margin), float(v0.margin)
    if math.isnan(m_u) or math.isnan(m0) or m_u >= 0.0 or m0 <= 0.0:
        s = 0.5                         # no usable slope: halve
    else:
        s = m0 / (m0 - m_u)             # the margin's zero along the line
    s = max(0.0, min(1.0, s))
    dist = float(np.linalg.norm(xy_c - xy_m))
    s2 = max(0.0, s - float(backoff_m) / max(dist, 1e-9))
    if s2 == 0.0:
        return Resolution(u_model, 'model', 2, (v, v0))
    u_s = (xy_m + s2 * (xy_c - xy_m), ap_m + s2 * (ap_c - ap_m))
    v2 = verdict_fn(u_s)
    if v2.ok:
        return Resolution(u_s, 'stepped', 3, (v, v0, v2))
    return Resolution(u_model, 'model', 3, (v, v0, v2))
