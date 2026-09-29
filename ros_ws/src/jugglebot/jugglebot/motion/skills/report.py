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
  height (860 mm), land on the catch plane (830 mm) ``schedule.flight_s``
  later (the ``|v| 4.166 m/s`` a 0.9 m skill announces is that 860 -> 830
  ballistic, not a same-height one) — so a throw that leaves on time reads
  0 ms. It is read where the flight is known best (the tracker's converged
  ballistic fit), not from a telemetry sample; a 1 % error in the fitted
  arrival speed moves it about 9 ms at a 0.9 m apex.
- **arrival** — when the ball reached the catch plane, minus when the catch
  that met it was aimed to (its final aim, after any re-aims): how well the
  catch was timed.
- **seat** — the first SEATED possession reading, minus the SCHEDULED
  landing (the OUTCOME line's ``seat=``: +0.10 s on a smooth catch at the
  bottom of the dive, +0.02 s met at the top, larger on a bounce).

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
    seat_s: Optional[float]
    reaims: int = 0                  # accepted re-aims of the catch
    reaim_refused: str = ''          # code of the re-aim that was refused


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


def throw_line(r: ThrowReport) -> Tuple[str, str]:
    """``(severity, text)`` — WARN for a miss, INFO otherwise.

    ``throw 3/5 CAUGHT  apex 0.917 m  landed -4, +9 mm  release +15 ms
    arrival -8 ms  seat +0.08 s  re-aimed 1x``"""
    verdict = {True: 'CAUGHT', False: 'MISSED', None: 'released'}[r.caught]
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
    if r.seat_s is not None:
        parts.append('seat %s s' % (_signed('%+.2f', r.seat_s),))
    if r.reaims or r.reaim_refused:
        text = 're-aimed %dx' % (r.reaims,)
        if r.reaim_refused:
            text += ', then refused (%s)' % (r.reaim_refused,)
        parts.append(text)
    if not r.row:
        parts.append('no learner row: %s' % (r.no_row_reason,))
    return ('WARN' if r.caught is False else 'INFO'), '  '.join(parts)


_PEAK = re.compile(r'peak ([a-z ]+?) ([-+\d.eE]+) ?(\S+) > ([-+\d.eE]+)')
_REJECTED = re.compile(r'^REJECTED_CYCLE_INFEASIBLE\((\w+): (.*)\)$', re.S)


def _compact(value: float) -> str:
    """``257921`` -> ``258k``; small values unchanged."""
    if abs(value) >= 10000.0:
        return '%.0fk' % (value / 1000.0,)
    return ('%.1f' % (value,)).rstrip('0').rstrip('.')


def short_refusal(message: str, limit: int = 90) -> str:
    """A planner refusal in a few words: its code and the number that broke.

    ``REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: <clock preamble>; peak leg jerk
    257921 mm/s³ > 150000)`` -> ``LIMIT_JERK: leg jerk 258k > 150k mm/s³``.
    The whole message stays in the DEBUG record; this is the screen's copy."""
    text = str(message).strip()
    m = _REJECTED.match(text)
    code, detail = (m.group(1), m.group(2)) if m else ('', text)
    peak = _PEAK.search(detail)
    if peak is not None:
        what, value, unit, limit_v = peak.groups()
        try:
            detail = '%s %s > %s %s' % (what, _compact(float(value)),
                                        _compact(float(limit_v)), unit)
        except ValueError:
            pass
    elif len(detail) > limit:
        detail = detail[:limit - 1].rstrip() + '…'
    return '%s: %s' % (code, detail) if code else detail


def start_line(label: str, n_throws: int, apex_m: float, *,
               memory_rows: Optional[int] = None,
               frame_offset_mm: Optional[float] = None,
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
    return ' · '.join(parts)


def end_line(label: str, end_code: str, end_message: str,
             reports: Sequence[ThrowReport],
             end_kind: str = '') -> Tuple[str, str]:
    """``(severity, text)`` for the attempt's last line.

    INFO when it completed or the operator stopped it, ERROR when anything
    else ended it — with the reason in a few words (:func:`short_refusal`).
    Always the catch count, and the spread of what flew."""
    thrown = len(reports)
    caught = sum(1 for r in reports if r.caught)
    tally = '%d/%d caught' % (caught, thrown)
    apexes = [r.apex_m for r in reports if r.apex_m is not None]
    if apexes:
        tally += ' · apex %.2f–%.2f m' % (min(apexes), max(apexes))
    worst = [max(abs(r.landing_err_mm[0]), abs(r.landing_err_mm[1]))
             for r in reports if r.landing_err_mm is not None]
    if worst:
        tally += ' · worst landing %.0f mm' % (max(worst),)
    releases = [r.release_err_s for r in reports
                if r.release_err_s is not None]
    if releases:
        tally += ' · release %s..%s' % (_ms(min(releases)), _ms(max(releases)))
    if not end_code:
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
    from log text."""
    return len(reports), sum(1 for r in reports if r.caught)
