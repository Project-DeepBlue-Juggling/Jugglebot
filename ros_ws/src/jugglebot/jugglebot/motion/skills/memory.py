"""``motion/skills/memory`` -- append-only throw memory (plan § 2.2 / § 2.5).

An :class:`Experience` is one throw's (state, command, observed outcome)
triple; a :class:`Memory` is the append-only CSV of them a site's learner
queries. File I/O (``Memory.__init__``'s load, ``Memory.append``'s write)
happens on the orchestrator thread ONCE PER THROW (plan § 0: "no ... blocking
I/O ... on the 40 Hz emitter" -- never call either from the emitter thread).

Pure Python + numpy. No ROS2 imports (the ``motion/`` rule).
"""

from __future__ import annotations

import csv
import dataclasses
import logging
import math
import os

import numpy as np

from jugglebot.motion.skills import learner as lr

__all__ = ['Experience', 'Memory', 'APEX_RATIO_BAND', 'apex_in_band',
           'memory_path']

_LOG = logging.getLogger(__name__)

#: The PHYSICAL band an observed apex ``y[2]`` may occupy, as a multiple of
#: the commanded apex ``u[2]``, for the row to be a row about THIS throw.
#:
#: **The square of the old flight band** (0.5, 1.6), retired 2026-09-18 with
#: the flight outcome itself: a flight is ``t = 2v/g``, so a ratio in flight
#: IS the ratio in release speed, and an apex is ``h = v^2 / 2g`` -- the ratio
#: in apex is that speed ratio SQUARED. ``0.5^2 = 0.25`` and ``1.6^2 = 2.56``
#: therefore admit and refuse exactly the same physical throws the flight band
#: did; nothing about the plant was re-judged here.
#:
#: **Measured** (2026-09-16, both sittings' ten ``temp/learn/*-20260916``
#: memories replayed against their bags' ``/balls``,
#: ``tools/probes/outcome_landing_replay.py``, in flight-ratio terms): the
#: estimates taken while the ball was ACTUALLY IN FLIGHT sit at ``r = 0.97 ..
#: 1.53`` (n = 22, i.e. 0.94 .. 2.34 in apex) and the CONTAMINATED rows the
#: old rule wrote at ``r = 2.21 .. 2.99`` (4.88 .. 8.94 in apex). The two
#: clusters are separated by a factor of 2.1 in apex and nothing lies between:
#:
#: * 2.56 clears the largest in-flight ratio measured and admits a plant
#:   throwing an apex 2.56x the commanded one -- well past the 1.53 this one
#:   does (apex 1.38 m on a 0.9 m command, 2026-09-16) -- while rejecting
#:   every contaminated row by at least a factor of 1.9. It is deliberately a
#:   PHYSICAL gate, not a scatter gate: a genuinely wild throw is exactly what
#:   the learner needs to see.
#: * 0.25 is an apex a QUARTER of the commanded one: a ball that leaves the
#:   cup at half the commanded speed is a failed release, not a throw to learn
#:   from. Nothing in flight measured below 0.94.
#:
#: **What this band CANNOT do**, and why ``executor._consider_landing``'s
#: freeze is the primary fix rather than the band: a ball sitting in the cup
#: after its catch has its "landing" predicted at ~now, and the apex implied
#: by the arrival speed of such an estimate is not bounded away from the
#: commanded one either. Only "the ball has already landed, stop looking"
#: separates that class.
#:
#: This is the ONE definition of the band (``executor`` imports it, and it is
#: applied on ``Memory`` load and append as well) -- a second copy would be
#: exactly the "timing twin" class plan § 0 forbids.
APEX_RATIO_BAND = (0.25, 2.56)


def apex_in_band(u_apex: float, y_apex: float) -> bool:
    """True when ``y_apex`` is inside :data:`APEX_RATIO_BAND` x ``u_apex``.

    A non-finite or non-positive commanded apex has no band (nothing to
    scale), and is treated as ADMITTING the observation: the band exists to
    reject a landing that belongs to another flight, and with no commanded
    apex to compare against there is no such evidence either way. Callers
    validate ``u`` on their own path (``Experience`` requires it finite).
    """
    u = float(u_apex)
    y = float(y_apex)
    if not (math.isfinite(u) and u > 0.0):
        return True
    if not math.isfinite(y):
        return False
    lo, hi = APEX_RATIO_BAND
    return lo * u <= y <= hi * u


#: Column order of the CSV -- x (4), u (3), y (3), t_abs_s, ball_id, caught.
#:
#: The apex columns are NAMED (``u2_apex_m`` / ``y2_apex_m``) because the
#: third component changed MEANING on 2026-09-18 -- it was a flight time in
#: seconds (``u2`` / ``y2``) -- and a file whose numbers mean something else
#: parses perfectly. :meth:`Memory._load` refuses any other header, so the
#: name in the file is the schema check.
_HEADER = ['x0', 'x1', 'x2', 'x3', 'u0', 'u1', 'u2_apex_m',
           'y0', 'y1', 'y2_apex_m', 't_abs_s', 'ball_id', 'caught']

#: The pre-2026-09-18 header, whose ``u2``/``y2`` were flight times (s).
_LEGACY_FLIGHT_HEADER = ['x0', 'x1', 'x2', 'x3', 'u0', 'u1', 'u2',
                         'y0', 'y1', 'y2', 't_abs_s', 'ball_id', 'caught']
_N_NUMERIC = 11  # x(4) + u(3) + y(3) + t_abs_s(1); ball_id/caught parsed separately

#: ``repr``-exact float formatting -- a round-tripped ROS-epoch ``t_abs_s``
#: (~1.79e9 s) must come back bit-identical, not merely close.
_FLOAT_FMT = '%.17g'


def _finite_vec(value, n: int, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.shape != (n,):
        raise ValueError('%s must be a %d-vector, got shape %s'
                          % (name, n, np.shape(value)))
    if not np.all(np.isfinite(arr)):
        raise ValueError('%s must be finite, got %r' % (name, arr.tolist()))
    return arr


@dataclasses.dataclass(frozen=True)
class Experience:
    """One throw's record (plan § 2.2). SI units.

    ``x`` (4,) = (site xy, seat-offset xy of the ball just caught); ``u`` (3,)
    = commanded (landing xy [m], APEX above the catch plane [m]); ``y`` (3,)
    = the SAME three quantities observed, all read off the tracker's
    converged gravity-fixed fit; ``t_abs_s`` the CAN wall-clock time;
    ``ball_id`` the tracked ball; ``caught`` whether the resulting catch
    succeeded.

    **Command and outcome are the same physical quantity** (2026-09-18, after
    Lee et al. § 4C): the third component was a flight TIME until then, and a
    time is measured from the commanded release knot, which the physical
    release lags by 0.02-0.14 s throw to throw, through a crossing estimate
    itself extrapolated to +-40 ms. Both biases drove the learner ~20 % low
    in apex while its own metric read on target. An apex is invariant to the
    release instant and to filter lag, and repeats to +-0.02-0.04 m at a
    fixed command (2026-09-17).
    """

    x: np.ndarray
    u: np.ndarray
    y: np.ndarray
    t_abs_s: float
    ball_id: int
    caught: bool

    def __post_init__(self):
        object.__setattr__(self, 'x', _finite_vec(self.x, 4, 'x'))
        object.__setattr__(self, 'u', _finite_vec(self.u, 3, 'u'))
        object.__setattr__(self, 'y', _finite_vec(self.y, 3, 'y'))
        t_abs_s = float(self.t_abs_s)
        if not math.isfinite(t_abs_s):
            raise ValueError('t_abs_s must be finite, got %r' % (self.t_abs_s,))
        object.__setattr__(self, 't_abs_s', t_abs_s)
        object.__setattr__(self, 'ball_id', int(self.ball_id))
        object.__setattr__(self, 'caught', bool(self.caught))


def memory_path(root: str, plant_id: str) -> str:
    """``<root>/temp/learn/<plant_id>/memory.csv``.

    ``plant_id`` must be exactly one safe path segment -- no separators, no
    ``.``/``..``, non-empty -- since it becomes a directory name under a path
    a caller controls (a ROS param at R4+).
    """
    if (not isinstance(plant_id, str) or not plant_id
            or plant_id in ('.', '..')
            or os.sep in plant_id or (os.altsep and os.altsep in plant_id)
            or '/' in plant_id):
        raise ValueError('plant_id must be one safe path segment (non-empty, '
                          'no "/" and not "." or "..") got %r' % (plant_id,))
    return os.path.join(root, 'temp', 'learn', plant_id, 'memory.csv')


class Memory:
    """Append-only kNN memory for one site's learner (plan § 2.2 / § 2.5).

    Loads ``path`` (if present) at construction; a malformed row (wrong
    column count, unparsable, non-finite) is DROPPED with a ``logging``
    warning naming the line number -- never fatal, so one corrupted row does
    not lose an entire session's memory. Rows are kept in file / call order.

    **A row outside :data:`APEX_RATIO_BAND` is not a row about its own
    throw** and is refused on BOTH paths -- dropped at load (so an already
    written contaminated memory, such as the ten ``temp/learn/*-20260916``
    files, cannot poison the next sitting even if it is read again) and raised
    on ``append`` (so the executor's own band check, ``executor.
    _finalise_outcome``, is belt and braces rather than the only guard).

    ``append`` and construction do file I/O; the plan's determinism rule
    means both must run on the orchestrator thread, once per throw -- never
    on the 40 Hz emitter thread.
    """

    def __init__(self, path: str):
        self._path = path
        self._X = np.zeros((0, 4))
        self._U = np.zeros((0, 3))
        self._Y = np.zeros((0, 3))
        self._load()

    def _load(self) -> None:
        if not os.path.exists(self._path):
            return
        with open(self._path, newline='') as handle:
            reader = csv.reader(handle)
            header = next(reader, None)
            if header is None:
                return
            self._check_header(header)
            for lineno, row in enumerate(reader, start=2):
                exp = self._parse_row(row, lineno)
                if exp is not None:
                    self._append_arrays(exp)

    def _check_header(self, header) -> None:
        """Refuse a file written to a different schema -- the numbers of a
        pre-2026-09-18 memory parse cleanly and mean a flight TIME, so the
        header is the only thing that can tell the two apart."""
        cols = [str(c).strip() for c in header]
        if cols == _HEADER:
            return
        if cols == _LEGACY_FLIGHT_HEADER:
            raise ValueError(
                '%s was written with the pre-2026-09-18 flight-time schema '
                '(u2/y2 are seconds, not the apex metres this learner now '
                'commands): move it aside as temp/learn/_quarantine_<date>/ '
                'rather than loading it' % (self._path,))
        raise ValueError('%s has header %r, expected %r'
                          % (self._path, cols, _HEADER))

    def _parse_row(self, row, lineno: int):
        try:
            if len(row) != len(_HEADER):
                raise ValueError('expected %d columns, got %d'
                                  % (len(_HEADER), len(row)))
            vals = [float(v) for v in row[:_N_NUMERIC]]
            if not all(math.isfinite(v) for v in vals):
                raise ValueError('non-finite numeric field')
            x, u, y = vals[0:4], vals[4:7], vals[7:10]
            t_abs_s = vals[10]
            ball_id = int(row[11])
            caught = row[12].strip().lower() in ('true', '1')
            if not apex_in_band(u[2], y[2]):
                raise ValueError(
                    'observed apex %.4f m outside [%.4f, %.4f] of the '
                    'commanded %.4f m -- this row is another flight'
                    % (y[2], APEX_RATIO_BAND[0] * u[2],
                       APEX_RATIO_BAND[1] * u[2], u[2]))
            return Experience(x=x, u=u, y=y, t_abs_s=t_abs_s, ball_id=ball_id,
                               caught=caught)
        except (ValueError, TypeError) as exc:
            _LOG.warning('%s: dropping malformed row at line %d: %s',
                          self._path, lineno, exc)
            return None

    def _append_arrays(self, exp: Experience) -> None:
        self._X = np.vstack([self._X, exp.x])
        self._U = np.vstack([self._U, exp.u])
        self._Y = np.vstack([self._Y, exp.y])

    def append(self, exp: Experience) -> None:
        """Append one row: writes the header on first create, flushes, and
        updates the in-memory arrays -- in call order."""
        if not isinstance(exp, Experience):
            raise ValueError('append expects an Experience, got %r' % (exp,))
        if not apex_in_band(exp.u[2], exp.y[2]):
            raise ValueError(
                'refusing a row whose observed apex %.4f m is outside '
                '[%.4f, %.4f] of the commanded %.4f m: the landing belongs '
                'to another flight'
                % (exp.y[2], APEX_RATIO_BAND[0] * exp.u[2],
                   APEX_RATIO_BAND[1] * exp.u[2], exp.u[2]))
        out_dir = os.path.dirname(self._path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        write_header = not os.path.exists(self._path)
        with open(self._path, 'a', newline='') as handle:
            writer = csv.writer(handle)
            if write_header:
                writer.writerow(_HEADER)
            writer.writerow(
                [_FLOAT_FMT % v for v in exp.x]
                + [_FLOAT_FMT % v for v in exp.u]
                + [_FLOAT_FMT % v for v in exp.y]
                + [_FLOAT_FMT % exp.t_abs_s, str(exp.ball_id), str(exp.caught)])
            handle.flush()
        self._append_arrays(exp)

    def __len__(self) -> int:
        return self._X.shape[0]

    def arrays(self):
        """The current ``(X, U, Y)`` arrays, shapes ``(n, 4)``/``(n, 3)``/``(n, 3)``."""
        return self._X, self._U, self._Y

    def command(self, x, y_d, cfg: lr.LearnerConfig = None) -> np.ndarray:
        """``learner.command`` over this memory's current arrays."""
        if cfg is None:
            cfg = lr.LearnerConfig()
        return lr.command(x, y_d, self._X, self._U, self._Y, cfg)
