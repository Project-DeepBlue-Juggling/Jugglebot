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

__all__ = ['Experience', 'Memory', 'memory_path']

_LOG = logging.getLogger(__name__)

#: Column order of the CSV -- x (4), u (3), y (3), t_abs_s, ball_id, caught.
_HEADER = ['x0', 'x1', 'x2', 'x3', 'u0', 'u1', 'u2', 'y0', 'y1', 'y2',
           't_abs_s', 'ball_id', 'caught']
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
    = commanded (landing xy, flight); ``y`` (3,) = observed (landing xy,
    flight); ``t_abs_s`` the CAN wall-clock time; ``ball_id`` the tracked
    ball; ``caught`` whether the resulting catch succeeded.
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
            for lineno, row in enumerate(reader, start=2):
                exp = self._parse_row(row, lineno)
                if exp is not None:
                    self._append_arrays(exp)

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
