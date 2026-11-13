"""``motion/skills/memory`` -- append-only throw memory, schema v2.

An :class:`Experience` is one throw's (release state, command, observed
outcome) record; a :class:`Memory` is the append-only CSV of them the
Learner is fed from. The Memory only LOADS, HOLDS and APPENDS rows: the
law that reads them lives in ``learner.Learner`` (which takes each row
through ``observe``).

**Why v2** (``plans/active/learner-generalisation.md`` section 2.3): the
residual memory is keyed on the throw's RELEASE position, release velocity
and release kind, not on the old (site xy, seat offset) 4-vector, which
cannot tell an oval's carried release from a free-cup one at the same Site.
So a row now stores ``site_xy_m``, ``release_m`` (3), ``release_vel_mps``
(3), ``dwell_s`` (nan = unknown), ``seat_offset_m``, ``u``, ``y``,
``t_abs_s``, ``ball_id``, ``release_kind``, ``caught`` and ``migrated``
(1 = the release geometry was reconstructed from a v1 row by
``tools/migrate_memory_schema_v2.py``, 0 = recorded live).

**File and load rules** (each refusal is a ValueError an operator can act on;
the node turns it into "memory refused: ..." and rejects the goal):

* ``<root>/temp/learn/<plant_id>/memory_v2.csv`` present: its header must
  equal :data:`HEADER` exactly (a v1 or legacy header inside it is refused
  by name).
* v2 absent and the v1 ``memory.csv`` beside it present: REFUSED, naming the
  migration command with both paths -- never a silent cold start beside an
  unmigrated corpus.
* neither present: an empty memory (a new plant_id starts cold).

Rows that fail to parse are dropped with a logged warning and counted in
:attr:`Memory.n_dropped`. File I/O (load, ``append``) happens on the
orchestrator thread ONCE PER THROW, never on the 40 Hz emitter.

Pure Python + numpy. No ROS2 imports (the ``motion/`` rule).
"""

from __future__ import annotations

import csv
import dataclasses
import logging
import math
import os

import numpy as np

__all__ = ['Experience', 'Memory', 'APEX_RATIO_BAND', 'apex_in_band',
           'memory_path', 'legacy_memory_path', 'HEADER', 'HEADER_V1',
           'LEGACY_FLIGHT_HEADER', 'SCHEMA_VERSION', 'RELEASE_KINDS']

_LOG = logging.getLogger(__name__)

#: The memory file schema this module reads and writes.
SCHEMA_VERSION = 2

#: The two release kinds a row can carry (``executor._is_carried``).
RELEASE_KINDS = ('free_cup', 'carried')

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


#: v2 column order: site (2), release (3), release velocity (3), dwell,
#: seat offset (2), u (3), y (3), then the bookkeeping columns. The apex
#: columns are NAMED (``u2_apex_m`` / ``y2_apex_m``) because a file whose
#: numbers mean something else parses perfectly: the header is the schema
#: check (see the v1 history in :data:`HEADER_V1`).
HEADER = ['site_x', 'site_y', 'rel_x', 'rel_y', 'rel_z',
          'vel_x', 'vel_y', 'vel_z', 'dwell_s', 'seat_x', 'seat_y',
          'u0', 'u1', 'u2_apex_m', 'y0', 'y1', 'y2_apex_m',
          't_abs_s', 'ball_id', 'release_kind', 'caught', 'migrated']

#: The FROZEN schema-v1 header (``x`` = site xy + seat offset xy). Kept for
#: the migration tools only: nothing in this module reads or writes it.
HEADER_V1 = ['x0', 'x1', 'x2', 'x3', 'u0', 'u1', 'u2_apex_m',
             'y0', 'y1', 'y2_apex_m', 't_abs_s', 'ball_id', 'caught']

#: The pre-2026-09-18 header, whose ``u2``/``y2`` were flight times (s).
LEGACY_FLIGHT_HEADER = ['x0', 'x1', 'x2', 'x3', 'u0', 'u1', 'u2',
                        'y0', 'y1', 'y2', 't_abs_s', 'ball_id', 'caught']

_N_NUMERIC_HEAD = 17    # site(2) rel(3) vel(3) dwell seat(2) u(3) y(3)

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


@dataclasses.dataclass(frozen=True, eq=False)
class Experience:
    """One throw's record (schema v2). SI units.

    ``site_xy_m`` (2) the release Site's cup xy; ``release_m`` (3) where the
    ball left; ``release_vel_mps`` (3) the PLANNED release velocity of the
    model command; ``dwell_s`` the dwell (nan = unknown, the only field
    allowed to be non-finite); ``seat_offset_m`` (2) of the ball just caught;
    ``u`` (3) = commanded (landing xy [m], APEX above the release plane [m]);
    ``y`` (3) = the same three quantities observed; ``t_abs_s`` the CAN
    wall-clock time; ``ball_id``; ``release_kind`` in :data:`RELEASE_KINDS`;
    ``caught``; ``migrated`` (reconstructed from a v1 row).

    Command and outcome are the same physical quantity (an apex, invariant
    to the release instant and filter lag: 2026-09-18).
    """

    site_xy_m: np.ndarray
    release_m: np.ndarray
    release_vel_mps: np.ndarray
    dwell_s: float
    seat_offset_m: np.ndarray
    u: np.ndarray
    y: np.ndarray
    t_abs_s: float
    ball_id: int
    release_kind: str
    caught: bool
    migrated: bool = False

    def __post_init__(self):
        for name, n in (('site_xy_m', 2), ('release_m', 3),
                        ('release_vel_mps', 3), ('seat_offset_m', 2),
                        ('u', 3), ('y', 3)):
            object.__setattr__(self, name,
                               _finite_vec(getattr(self, name), n, name))
        dwell_s = float(self.dwell_s)
        if math.isinf(dwell_s):
            raise ValueError('dwell_s must be finite or nan, got %r'
                              % (self.dwell_s,))
        object.__setattr__(self, 'dwell_s', dwell_s)
        t_abs_s = float(self.t_abs_s)
        if not math.isfinite(t_abs_s):
            raise ValueError('t_abs_s must be finite, got %r' % (self.t_abs_s,))
        object.__setattr__(self, 't_abs_s', t_abs_s)
        object.__setattr__(self, 'ball_id', int(self.ball_id))
        if self.release_kind not in RELEASE_KINDS:
            raise ValueError('release_kind must be one of %r, got %r'
                              % (RELEASE_KINDS, self.release_kind))
        object.__setattr__(self, 'caught', bool(self.caught))
        object.__setattr__(self, 'migrated', bool(self.migrated))


def _check_plant_id(plant_id) -> None:
    if (not isinstance(plant_id, str) or not plant_id
            or plant_id in ('.', '..')
            or os.sep in plant_id or (os.altsep and os.altsep in plant_id)
            or '/' in plant_id):
        raise ValueError('plant_id must be one safe path segment (non-empty, '
                          'no "/" and not "." or "..") got %r' % (plant_id,))


def memory_path(root: str, plant_id: str) -> str:
    """``<root>/temp/learn/<plant_id>/memory_v2.csv`` (the schema-v2 file).

    ``plant_id`` must be exactly one safe path segment.
    """
    _check_plant_id(plant_id)
    return os.path.join(root, 'temp', 'learn', plant_id, 'memory_v2.csv')


def legacy_memory_path(root: str, plant_id: str) -> str:
    """``<root>/temp/learn/<plant_id>/memory.csv`` (the schema-v1 file)."""
    _check_plant_id(plant_id)
    return os.path.join(root, 'temp', 'learn', plant_id, 'memory.csv')


def _row_for(exp: Experience):
    return ([_FLOAT_FMT % v for v in exp.site_xy_m]
            + [_FLOAT_FMT % v for v in exp.release_m]
            + [_FLOAT_FMT % v for v in exp.release_vel_mps]
            + [_FLOAT_FMT % exp.dwell_s]
            + [_FLOAT_FMT % v for v in exp.seat_offset_m]
            + [_FLOAT_FMT % v for v in exp.u]
            + [_FLOAT_FMT % v for v in exp.y]
            + [_FLOAT_FMT % exp.t_abs_s, str(exp.ball_id), exp.release_kind,
               str(exp.caught), '1' if exp.migrated else '0'])


def format_row(exp: Experience):
    """The CSV fields of ``exp`` in :data:`HEADER` order (shared with the
    migration tool so both write byte-identical rows)."""
    return _row_for(exp)


class Memory:
    """Append-only row store (schema v2) the Learner is fed from.

    Loads ``path`` (if present) at construction; a malformed row (wrong
    column count, unparsable, non-finite, unknown release kind, apex outside
    :data:`APEX_RATIO_BAND`) is DROPPED with a ``logging`` warning naming the
    line, counted in :attr:`n_dropped`, never fatal. Rows keep file / call
    order. A row outside the apex band is refused on ``append`` too.

    If ``path`` is ``<dir>/memory_v2.csv`` and it is absent while
    ``<dir>/memory.csv`` (schema v1) exists, construction raises ValueError
    naming the migration command.
    """

    def __init__(self, path: str):
        self._path = path
        self._rows = []
        self.n_dropped = 0
        self._load()

    def _load(self) -> None:
        if not os.path.exists(self._path):
            legacy = os.path.join(os.path.dirname(self._path), 'memory.csv')
            if (os.path.basename(self._path) == 'memory_v2.csv'
                    and os.path.exists(legacy)):
                raise ValueError(
                    'found a schema-v1 memory at %s and no memory_v2.csv: '
                    'run `python tools/migrate_memory_schema_v2.py --memory '
                    '%s --out %s` (it writes a new file and never changes '
                    'the v1 one)' % (legacy, legacy, self._path))
            return
        with open(self._path, newline='') as handle:
            reader = csv.reader(handle)
            header = next(reader, None)
            if header is None:
                return
            self._check_header(header)
            for lineno, row in enumerate(reader, start=2):
                exp = self._parse_row(row, lineno)
                if exp is None:
                    self.n_dropped += 1
                else:
                    self._rows.append(exp)

    def _check_header(self, header) -> None:
        cols = [str(c).strip() for c in header]
        if cols == HEADER:
            return
        if cols == HEADER_V1:
            raise ValueError(
                '%s is a schema-v1 memory (header x0..x3): run `python '
                'tools/migrate_memory_schema_v2.py --memory %s --out <v2 '
                'path>` and load the v2 file' % (self._path, self._path))
        if cols == LEGACY_FLIGHT_HEADER:
            raise ValueError(
                '%s was written with the pre-2026-09-18 flight-time schema '
                '(u2/y2 are seconds, not the apex metres this learner now '
                'commands): move it aside as temp/learn/_quarantine_<date>/ '
                'rather than loading it' % (self._path,))
        raise ValueError('%s has header %r, expected %r'
                          % (self._path, cols, HEADER))

    def _parse_row(self, row, lineno: int):
        try:
            if len(row) != len(HEADER):
                raise ValueError('expected %d columns, got %d'
                                  % (len(HEADER), len(row)))
            vals = [float(v) for v in row[:_N_NUMERIC_HEAD]]
            t_abs_s = float(row[17])
            ball_id = int(row[18])
            kind = row[19].strip()
            caught = row[20].strip().lower() in ('true', '1')
            migrated = row[21].strip().lower() in ('true', '1')
            u, y = vals[11:14], vals[14:17]
            if not apex_in_band(u[2], y[2]):
                raise ValueError(
                    'observed apex %.4f m outside [%.4f, %.4f] of the '
                    'commanded %.4f m -- this row is another flight'
                    % (y[2], APEX_RATIO_BAND[0] * u[2],
                       APEX_RATIO_BAND[1] * u[2], u[2]))
            return Experience(
                site_xy_m=vals[0:2], release_m=vals[2:5],
                release_vel_mps=vals[5:8], dwell_s=vals[8],
                seat_offset_m=vals[9:11], u=u, y=y, t_abs_s=t_abs_s,
                ball_id=ball_id, release_kind=kind, caught=caught,
                migrated=migrated)
        except (ValueError, TypeError) as exc:
            _LOG.warning('%s: dropping malformed row at line %d: %s',
                          self._path, lineno, exc)
            return None

    def append(self, exp: Experience) -> None:
        """Append one row (v2 only): writes the header when the file is
        missing OR empty, flushes, and keeps the row in call order.

        If the file's last byte is not a newline (a power cut mid-row), a
        newline is written first, so only that torn row is dropped at load
        rather than the torn row AND this one glued onto it."""
        if not isinstance(exp, Experience):
            raise ValueError('append expects an Experience, got %r' % (exp,))
        if os.path.basename(self._path) == 'memory.csv':
            raise ValueError(
                'refusing to append a schema-v2 row to %s: memory.csv is the '
                'schema-v1 file name (open memory_v2.csv, `memory_path`)'
                % (self._path,))
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
        size = (os.path.getsize(self._path)
                if os.path.exists(self._path) else 0)
        write_header = size == 0
        torn = False
        if size > 0:
            with open(self._path, 'rb') as raw:
                raw.seek(-1, os.SEEK_END)
                torn = raw.read(1) != b'\n'
        with open(self._path, 'a', newline='') as handle:
            writer = csv.writer(handle)
            if torn:
                handle.write('\n')
            if write_header:
                writer.writerow(HEADER)
            writer.writerow(_row_for(exp))
            handle.flush()
        self._rows.append(exp)

    def __len__(self) -> int:
        return len(self._rows)

    @property
    def path(self) -> str:
        return self._path

    def rows(self):
        """The ``Experience`` objects in file / call order (a new list)."""
        return list(self._rows)

    def arrays(self):
        """``(U, Y)`` arrays, shapes ``(n, 3)``."""
        n = len(self._rows)
        return (np.array([e.u for e in self._rows]).reshape(n, 3),
                np.array([e.y for e in self._rows]).reshape(n, 3))

    @property
    def ball_ids(self) -> np.ndarray:
        return np.array([e.ball_id for e in self._rows], dtype=int)

    @property
    def release_kinds(self) -> np.ndarray:
        return np.array([e.release_kind for e in self._rows], dtype=object)
