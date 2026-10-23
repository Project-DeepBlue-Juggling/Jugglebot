"""What the operator reads about an attempt: one line as it starts, one per
throw, one as it ends.

The executor's ``tick()`` lines (dispatches, splices, catch aims, re-aim
decisions, OUTCOME rows) are the RECORD — ``skill_node`` logs them at DEBUG,
which the launch console keeps off the screen while ``launch.log``, the
per-process log file and ``/rosout`` in the bag keep every one. This module
is the SCREEN: the executor fills a :class:`ThrowReport` per finalised
release, and the functions below turn those into the few lines an operator
reads live.

Timing words, all in the wall-clock seconds the schedule runs on:

- **release** — when the ball's flight actually began, minus when it was
  commanded to (:func:`release_error_s`): the fitted parabola traced back
  to the RELEASE HEIGHT on the way up, minus the THROW's release instant.
  The planner commands exactly that flight — release at the site's throw
  height (860 mm), land on the catch plane (830 mm -- R5 sitting 6's
  catch-high decision, 2026-10-05, briefly raised this to 930; the owner
  returned it to 830 on 2026-10-06, restoring the original contact speed)
  ``schedule.flight_s`` later (the ``|v| 4.166 m/s`` a 0.9 m skill announces
  today is that 860 -> 830 ballistic, not a same-height one; during the
  catch-high window it was 4.283 m/s for 860 -> 930) — so a throw that
  leaves on time reads 0 ms. It
  is read where the flight is known best (the tracker's converged
  ballistic fit), not from a telemetry sample; a 1 % error in the fitted
  arrival speed moves it about 9 ms at a 0.9 m apex.
- **arrival** — when the ball reached the catch plane, minus when the catch
  that met it was aimed to (its final aim, after any re-aims): how well the
  catch was timed.
- **seat** — the first SEATED possession reading, minus the TRACKER'S
  crossing of the catch plane (the fit's ``t_land``; since 2026-10-22 -- it
  was the SCHEDULED landing, whose zero used the COMMANDED apex and so carried
  the learner's ~40 ms low-apex bias on every self-throw).  The OUTCOME
  DEBUG line's ``seat=`` keeps the scheduled zero.  ``--`` with no fit.
- **contact** — when the cup was struck (the hand's measured velocity kicked
  below its feedforward, ``skill_node._contact_time``) minus the same
  crossing; + = the ball met the cup after the tracker's crossing time.
- **settle** — contact to the first SEATED reading: how long the ball
  bounces or rattles before it rests on the sensor (2026-10-21: BB feeds
  119 ms in the PLA cup, 252 ms in the ASA one at the same closing speed).
  ``--`` when either end is unobserved.

Pure Python: no ROS, no config imports.
"""

from __future__ import annotations

import dataclasses
import math
import re
from typing import List, Optional, Sequence, Tuple

from jugglebot.motion.trajectory.ballistics_bc import GRAVITY_MMS2


@dataclasses.dataclass(frozen=True)
class ThrowReport:
    """One finalised release, as the operator's throw line reports it.

    Every ``Optional`` is ``None`` when the thing was not observed (no
    converged fit, no possession observer wired, no catch aimed at this
    flight) — the line then leaves that field out rather than print a
    guess."""

    throw_no: int                    # 1-based release order in the attempt
    n_throws: int                    # releases the attempt's schedule plans
    ball_id: int
    caught: Optional[bool]           # the possession verdict latch
    row: bool                        # a learner row was written
    no_row_reason: str               # why not, when ``row`` is False
    apex_m: Optional[float]          # observed, above the catch plane
    landing_err_mm: Optional[Tuple[float, float]]   # landing - target (x, y)
    release_err_s: Optional[float]
    arrival_err_s: Optional[float]
    seat_s: Optional[float]          # seat - the tracker's crossing
    reaims: int = 0                  # accepted re-aims of the catch
    reaim_refused: str = ''          # code of the re-aim that was refused
    contact_s: Optional[float] = None   # cup struck - the tracker's crossing
    settle_s: Optional[float] = None    # first SEATED - cup struck
    #: The two-ball columns Stop (D2): the last throw is aimed at the ball the
    #: cup already holds, BY DESIGN, so it is not a catch attempt -- it is
    #: excluded from the operator's tally (:func:`end_line`, :func:`summarise`)
    #: and its line says so (2026-10-23, owner: the 47/48 read of the 48-run).
    stop: bool = False


@dataclasses.dataclass(frozen=True)
class FeedCatchReport:
    """One Ball Butler feed catch that seated (the feed has no THROW row of
    its own, so no :class:`ThrowReport`).  Same zero as the throw line: the
    tracker's crossing.  ``None`` fields print ``--``."""

    ball_id: int
    arrival_err_s: Optional[float]   # crossing - the catch's dispatched aim
    seat_s: Optional[float]
    contact_s: Optional[float]
    settle_s: Optional[float]


def release_error_s(t_land_s: float, z_land_mm: float, vz_land_mm_s: float,
                    z_release_mm: float,
                    t_release_cmd_s: float) -> Optional[float]:
    """The fitted flight's release instant minus the commanded one.

    The flight is the parabola through the catch-plane crossing
    ``(t_land_s, z_land_mm)`` arriving at ``vz_land_mm_s`` (< 0), under the
    planner's own gravity (``ballistics_bc.GRAVITY_MMS2``); it passed
    ``z_release_mm`` on the way up at ``t_land_s + tau``, ``tau`` the earlier
    root of ``z_land + vz·tau - g·tau²/2 = z_release``. ``None`` for a
    non-descending or non-physical estimate (one that never reached the
    release height)."""
    v = float(vz_land_mm_s)
    disc = v * v + 2.0 * GRAVITY_MMS2 * (float(z_land_mm) - float(z_release_mm))
    if not (v < 0.0) or not math.isfinite(disc) or disc < 0.0:
        return None
    tau = (v - math.sqrt(disc)) / GRAVITY_MMS2
    return float(t_land_s) + tau - float(t_release_cmd_s)


def _signed(fmt: str, value: float) -> str:
    """``fmt % value`` with a value that ROUNDS to zero printed as ``+0``,
    never the ``-0`` a tiny negative formats as."""
    text = fmt % (value,)
    return fmt % (0.0,) if float(text) == 0.0 else text


def _ms(seconds: float) -> str:
    return _signed('%+.0f', seconds * 1000.0) + ' ms'


def _seat_text(seat_s: Optional[float]) -> str:
    return ('--' if seat_s is None
            else '%s s (vs crossing)' % (_signed('%+.2f', seat_s),))


def _contact_text(contact_s: Optional[float]) -> str:
    return '--' if contact_s is None else _ms(contact_s)


def _settle_text(settle_s: Optional[float]) -> str:
    return ('--' if settle_s is None
            else '%s ms' % (_signed('%.0f', settle_s * 1000.0),))


def feed_line(r: FeedCatchReport) -> str:
    """``feed Ball 2 CAUGHT  arrival +12 ms  seat +0.26 s (vs crossing)
    contact +14 ms  settle 252 ms`` (INFO)."""
    parts = ['feed ball %d CAUGHT' % (r.ball_id,)]
    parts.append('arrival %s' % ('--' if r.arrival_err_s is None
                                 else _ms(r.arrival_err_s),))
    parts.append('seat %s' % (_seat_text(r.seat_s),))
    parts.append('contact %s' % (_contact_text(r.contact_s),))
    parts.append('settle %s' % (_settle_text(r.settle_s),))
    return '  '.join(parts)


def throw_line(r: ThrowReport) -> Tuple[str, str]:
    """``(severity, text)`` — WARN for a miss, INFO otherwise.

    ``throw 3/5 CAUGHT  apex 0.917 m  landed -4, +9 mm  release +15 ms
    arrival -8 ms  seat +0.08 s (vs crossing)  contact +14 ms  settle 82 ms
    re-aimed 1x``.  ``seat`` is referenced to the tracker's crossing time
    (not the scheduled landing); ``--`` marks a field with no evidence."""
    verdict = {True: 'CAUGHT', False: 'MISSED', None: 'released'}[r.caught]
    if r.stop:
        verdict = 'STOP (landed on the held ball, by design; not counted)'
    parts = ['throw %d/%d %s' % (r.throw_no, r.n_throws, verdict)]
    if r.apex_m is not None:
        parts.append('apex %.3f m' % (r.apex_m,))
    if r.landing_err_mm is not None:
        parts.append('landed %s, %s mm' % (_signed('%+.0f', r.landing_err_mm[0]),
                                           _signed('%+.0f', r.landing_err_mm[1])))
    if r.release_err_s is not None:
        parts.append('release %s' % (_ms(r.release_err_s),))
    if r.arrival_err_s is not None:
        parts.append('arrival %s' % (_ms(r.arrival_err_s),))
    parts.append('seat %s' % (_seat_text(r.seat_s),))
    parts.append('contact %s' % (_contact_text(r.contact_s),))
    parts.append('settle %s' % (_settle_text(r.settle_s),))
    if r.reaims or r.reaim_refused:
        text = 're-aimed %dx' % (r.reaims,)
        if r.reaim_refused:
            text += ', then refused (%s)' % (r.reaim_refused,)
        parts.append(text)
    if not r.row:
        parts.append('no learner row: %s' % (r.no_row_reason,))
    return (('WARN' if (r.caught is False and not r.stop) else 'INFO'),
            '  '.join(parts))


_PEAK = re.compile(r'peak ([a-z ]+?) ([-+\d.eE]+) ?(\S+) > ([-+\d.eE]+)')
# Appended by feasibility._where_suffix AFTER the peak/limit prefix (2026-10-11):
# "(leg 4 at t=0.200 s on the gated plan's clock)". Leg numbering is 0-based.
_WHERE = re.compile(r'\(leg (\d+) at t=([-+\d.]+) s')
_REJECTED = re.compile(r'^REJECTED_CYCLE_INFEASIBLE\((\w+): (.*)\)$', re.S)


def _compact(value: float) -> str:
    """``257921`` -> ``258k``; small values unchanged."""
    if abs(value) >= 10000.0:
        return '%.0fk' % (value / 1000.0,)
    return ('%.1f' % (value,)).rstrip('0').rstrip('.')


def short_refusal(message: str, limit: int = 90) -> str:
    """A planner refusal in a few words: its code and the number that broke.

    ``REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: <clock preamble>; peak leg jerk
    257921 mm/s³ > 150000 (leg 4 at t=0.200 s on the gated plan's clock))`` ->
    ``LIMIT_JERK: leg jerk 258k > 150k mm/s³ (leg 4 @ +0.200 s)``.
    The whole message stays in the DEBUG record; this is the screen's copy."""
    text = str(message).strip()
    m = _REJECTED.match(text)
    code, detail = (m.group(1), m.group(2)) if m else ('', text)
    detail_full = detail
    peak = _PEAK.search(detail)
    if peak is not None:
        what, value, unit, limit_v = peak.groups()
        try:
            detail = '%s %s > %s %s' % (what, _compact(float(value)),
                                        _compact(float(limit_v)), unit)
            where = _WHERE.search(detail_full)
            if where is not None:
                detail += ' (leg %s @ +%.3f s)' % (where.group(1),
                                                   float(where.group(2)))
        except ValueError:
            pass
    elif len(detail) > limit:
        detail = detail[:limit - 1].rstrip() + '…'
    return '%s: %s' % (code, detail) if code else detail


def start_line(label: str, n_throws: int, apex_m: float, *,
               memory_rows: Optional[int] = None,
               frame_offset_mm: Optional[float] = None,
               catch_time_offset_ms: float = 0.0,
               extra: str = '') -> str:
    """``self_toss started: 5 throws at apex 0.90 m · memory 4 rows ·
    mocap frame offset 1.5 mm``"""
    parts = ['%s started: %d throw%s at apex %.2f m%s'
             % (label, n_throws, '' if n_throws == 1 else 's', apex_m,
                (' ' + extra) if extra else '')]
    if memory_rows is not None:
        parts.append('memory %d rows' % (memory_rows,))
    if frame_offset_mm is not None:
        parts.append('mocap frame offset %.1f mm' % (frame_offset_mm,))
    if catch_time_offset_ms:
        parts.append('DEBUG catch_time_offset %s ms'
                     % (_signed('%+.0f', catch_time_offset_ms),))
    return ' · '.join(parts)


def end_line(label: str, end_code: str, end_message: str,
             reports: Sequence[ThrowReport],
             end_kind: str = '',
             shadow_landing_stop: bool = False) -> Tuple[str, str]:
    """``(severity, text)`` for the attempt's last line.

    INFO when it completed or the operator stopped it, ERROR when anything
    else ended it — with the reason in a few words (:func:`short_refusal`).
    Always the catch count, and the spread of what flew.

    ``shadow_landing_stop`` (owner decision D2, 2026-09-30): the columns
    pattern's own end state — the schedule's last throw was aimed at the
    ball already held at the OTHER site (:func:`~jugglebot.motion.skills.
    schedule.compile_columns`'s docstring), so a CLEAN finish (``end_code``
    empty — nothing refused, every skill dispatched) leaves the platform
    holding both balls, not at an ordinary rest. Only meaningful when
    ``end_code`` is empty: a D3 drop or any other refusal already has its
    own, more specific, reason and takes priority."""
    # The Stop throw is not a catch attempt (ThrowReport.stop): the tally is
    # over the throws the cup was asked to catch.  The end code below still
    # rides beside it, so a run that ENDED on a real miss says so.
    thrown, caught = summarise(reports)
    tally = '%d/%d caught' % (caught, thrown)
    # The ranges share the count's base: the Stop's "landing" is a ball
    # striking the held ball by design, so its landing and release errors
    # would otherwise set the worst figure of a clean run (audit 2026-10-23).
    counted = [r for r in reports if not r.stop]
    apexes = [r.apex_m for r in counted if r.apex_m is not None]
    if apexes:
        tally += ' · apex %.2f–%.2f m' % (min(apexes), max(apexes))
    worst = [max(abs(r.landing_err_mm[0]), abs(r.landing_err_mm[1]))
             for r in counted if r.landing_err_mm is not None]
    if worst:
        tally += ' · worst landing %.0f mm' % (max(worst),)
    releases = [r.release_err_s for r in counted
                if r.release_err_s is not None]
    if releases:
        tally += ' · release %s..%s' % (_ms(min(releases)), _ms(max(releases)))
    if not end_code:
        if shadow_landing_stop:
            return ('INFO',
                    '%s done (one ball held, the last ball rests on the '
                    'platform — not resumable, cone delivery is a later '
                    'rung): %s' % (label, tally))
        return 'INFO', '%s done: %s' % (label, tally)
    if end_code == 'STOPPED':
        return 'INFO', '%s stopped by the operator: %s' % (label, tally)
    at = ' at the %s' % (end_kind,) if end_kind else ''
    reason = short_refusal(end_message) if end_message else ''
    if reason.startswith(end_code + ': '):
        reason = reason[len(end_code) + 2:]      # the code is already said
    if not reason or reason == end_code:
        return 'ERROR', '%s ENDED (%s)%s · %s' % (label, end_code, at, tally)
    return 'ERROR', '%s ENDED (%s)%s: %s · %s' % (label, end_code, at, reason,
                                                  tally)


def summarise(reports: List[ThrowReport]) -> Tuple[int, int]:
    """``(throws, caught)`` for the Juggle result — from the reports, never
    from log text.  The columns Stop (:attr:`ThrowReport.stop`) is neither a
    throw nor a catch here: it was never a catch attempt."""
    counted = [r for r in reports if not r.stop]
    return len(counted), sum(1 for r in counted if r.caught)
