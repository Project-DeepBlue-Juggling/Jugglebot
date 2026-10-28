"""Jugglebot records: the ledger of the most Catches per Pattern (JB-35).

``docs/records.md`` (repo root) is the single source of truth. This module
RENDERS that whole file from a data model and PARSES it back; nothing else
writes it. Pure Python (stdlib only, no ROS, Python 3.8): ``skill_node``
imports it as ``jugglebot.records`` and the GUI server loads it by path.

The rule (owner, 2026-10-28)
----------------------------
* A record is the most Catches in ONE continuous Attempt ON THE ROBOT (never
  the sim; ``skill_node`` only writes when its ``records_path`` parameter is
  set, and only ``jugglebot_launch.py`` sets it).
* A Pattern's record is beaten only by STRICTLY more Catches (a first record
  needs at least one Catch). Each time it is beaten one row is appended to
  that Pattern's table, so a table is the history of its record, newest last.
* Every Pattern the skill node accepts except the Feed (``reload`` and
  ``magazine`` move balls, they are not juggling): :data:`PATTERNS`.
* Tables are grouped and ordered by number of balls, then Pattern id. Each
  heading carries ``<!-- pattern: <id> balls: <n> -->`` so the parser reads the
  Pattern and its ball count from a marker, never from prose.

How it ended (the end-code map)
-------------------------------
The "How it ended" cell is a human category plus the raw end code, e.g.
``drop (MISSED_CATCH)``. The end code is the Juggle result's ``outcome``
(``skill_node._juggle_execute``: ``STOPPED``, the executor's ``end_code``, or
``COMPLETED``). :func:`category` maps it:

* ``completed`` -- ``COMPLETED``, ``THROWN``.
* ``stopped``   -- ``STOPPED`` (an operator stop or cancel).
* ``drop``      -- a missed or lost ball: ``MISSED_CATCH``,
  ``ABORTED_NO_RELEASE``, ``DROPPED_SURVIVOR_STOPPED``, ``NO_LANDING``.
* ``compute``   -- the planner could not stand behind a segment in time or in
  limits: ``SPLICE_TOO_LATE``, ``ORIGIN_TOO_LATE``, ``REBASE_DISCONTINUOUS``,
  ``WINDOW_TOO_SHORT``, ``UNREACHABLE``, ``NO_ADMISSIBLE_COMMAND``, any
  ``LIMIT_*``, and any code naming ``INFEASIBLE``, ``BUDGET`` or ``SOLVE``.
* ``other``     -- everything else (``ABORTED_MODE_CHANGED``, ``LANE_REFUSED``,
  ``REJECTED_*``, ``ATTEMPT_ENDED``, ...).

The literals are restated, not imported, from ``motion/skills/executor.py`` so
the GUI server can load this file without the motion package;
``tests/motion/test_records.py`` pins them against the executor's constants.

Writes
------
:func:`record_attempt` is the one writer: it parses the existing file (a
malformed file raises :class:`RecordsFormatError` and is NEVER overwritten),
applies the Attempt, and writes only when a record was beaten or the file did
not exist, atomically (temp file in the same directory + ``os.replace``).
Rendering is deterministic, so a re-render of the same model is
byte-identical.

CLI::

    python -m jugglebot.records rebuild --attempts attempts.csv --out docs/records.md
    python -m jugglebot.records show [--path docs/records.md]
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import os
import re
import sys
import tempfile
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

#: (pattern id, label, balls). Ids are ``skill_node.SkillNode._PATTERNS``
#: minus :data:`FEED_PATTERNS` (pinned by tests/motion/test_records.py).
#: Labels follow GLOSSARY.md (self-throw, not self-toss); balls = the REAL
#: balls in the Attempt (a phantom ball is motion only).
PATTERNS: Tuple[Tuple[str, str, int], ...] = (
    ('self_toss', 'Self-throw', 1),
    ('hop', 'Hop', 1),
    ('oval_1ball', 'Oval — Ball 1', 1),
    ('oval_1ball_fed', 'Oval — fed', 1),
    ('columns_1ball', 'Columns — Ball 1', 1),
    ('columns_1ball_fed', 'Columns — Ball 2 (fed)', 1),
    ('columns', 'Columns', 2),
)
#: Goals that are Feeds, never records.
FEED_PATTERNS = frozenset(('reload', 'magazine'))

_LABELS = {p: label for p, label, _b in PATTERNS}
_BALLS = {p: b for p, _l, b in PATTERNS}

COMPLETED, STOPPED, DROP, COMPUTE, OTHER = (
    'completed', 'stopped', 'drop', 'compute', 'other')
CATEGORIES = (COMPLETED, STOPPED, DROP, COMPUTE, OTHER)
_COMPLETED_CODES = frozenset(('COMPLETED', 'THROWN'))
_STOPPED_CODES = frozenset(('STOPPED',))
_DROP_CODES = frozenset(('MISSED_CATCH', 'ABORTED_NO_RELEASE',
                         'DROPPED_SURVIVOR_STOPPED', 'NO_LANDING'))
_COMPUTE_CODES = frozenset(('SPLICE_TOO_LATE', 'ORIGIN_TOO_LATE',
                            'REBASE_DISCONTINUOUS', 'WINDOW_TOO_SHORT',
                            'UNREACHABLE', 'NO_ADMISSIBLE_COMMAND'))

CSV_COLUMNS = ('when_local', 'pattern', 'throws', 'catches', 'end_code',
               'source')
WHEN_FORMAT = '%Y-%m-%d %H:%M'
_WHEN_RE = re.compile(r'^\d{4}-\d{2}-\d{2} \d{2}:\d{2}$')
_MARKER_RE = re.compile(r'^<!-- pattern: ([a-z0-9_]+) balls: (\d+) -->$')
_ENDED_RE = re.compile(r'^([a-z]+) \((.+)\)$')
_TABLE_HEAD = '| When | Throws | Catches | How it ended | Note |'
_TABLE_RULE = '|---|---:|---:|---|---|'
_NO_RECORD = '_No record yet._'
_FILE_MARKER = '<!-- jugglebot-records v1 -->'

RULE = ('The most Catches in one continuous Attempt on the robot (never the '
        'sim), per Pattern. A record is beaten only by strictly more Catches.')


class RecordsFormatError(ValueError):
    """The ledger file does not parse; it must not be overwritten."""


def category(end_code: str) -> str:
    """The human category of an Attempt's end code (module docstring)."""
    c = str(end_code or '').strip()
    if c in _COMPLETED_CODES:
        return COMPLETED
    if c in _STOPPED_CODES:
        return STOPPED
    if c in _DROP_CODES:
        return DROP
    if (c in _COMPUTE_CODES or c.startswith('LIMIT_')
            or any(w in c for w in ('INFEASIBLE', 'BUDGET', 'SOLVE'))):
        return COMPUTE
    return OTHER


@dataclasses.dataclass(frozen=True)
class Attempt:
    """One finished Attempt as skill_node (or the seed CSV) reports it."""
    when_local: str      # local wall clock, 'YYYY-MM-DD HH:MM'
    pattern: str
    throws: int
    catches: int
    end_code: str
    note: str = ''


@dataclasses.dataclass(frozen=True)
class Record:
    """One row of a Pattern's table: the Attempt that set that record."""
    when_local: str
    throws: int
    catches: int
    end_code: str
    note: str = ''


@dataclasses.dataclass(frozen=True)
class Table:
    pattern: str
    label: str
    balls: int
    rows: Tuple[Record, ...] = ()

    @property
    def best(self) -> Optional[Record]:
        return self.rows[-1] if self.rows else None


@dataclasses.dataclass(frozen=True)
class Ledger:
    """pattern id -> :class:`Table`; :meth:`ordered` is the file order."""
    tables: Tuple[Tuple[str, Table], ...]

    def table(self, pattern: str) -> Optional[Table]:
        for p, t in self.tables:
            if p == pattern:
                return t
        return None

    def ordered(self) -> List[Table]:
        return sorted((t for _p, t in self.tables),
                      key=lambda t: (t.balls, t.pattern))

    def replace(self, table: Table) -> 'Ledger':
        rest = [t for p, t in self.tables if p != table.pattern]
        return _ledger(rest + [table])


def _ledger(tables: Iterable[Table]) -> Ledger:
    """The canonical (file-order) tuple, so equal ledgers compare equal."""
    return Ledger(tables=tuple((t.pattern, t) for t in sorted(
        tables, key=lambda t: (t.balls, t.pattern))))


def empty() -> Ledger:
    """Every Pattern heading, no rows."""
    return _ledger(Table(p, label, balls) for p, label, balls in PATTERNS)


def _cell(text: str) -> str:
    return ' '.join(str(text).replace('|', '/').split())


def render(model: Ledger) -> str:
    """The whole ``docs/records.md`` text (deterministic)."""
    out = [
        '# Jugglebot records',
        '',
        _FILE_MARKER,
        '',
        RULE + ' Each time a record is beaten a row is appended to that '
        'Pattern\'s table, so a table is the history of its record, newest '
        'last. The Feed is not a Pattern here.',
        '',
        '`skill_node` writes this file live at the end of each Attempt (ROS '
        'parameter `records_path`, set by `jugglebot_launch.py`) and the '
        'Sitting\'s close-out commits it. The tables are rendered and parsed '
        'by `ros_ws/src/jugglebot/jugglebot/records.py`; a Note cell may be '
        'edited by hand, nothing else. "How it ended" is completed, stopped, '
        'drop (a missed or lost ball), compute (a solve or limit refusal) or '
        'other, with the raw end code in parentheses.',
    ]
    last_balls = None
    for t in model.ordered():
        if t.balls != last_balls:
            last_balls = t.balls
            out += ['', '## %d ball%s' % (t.balls, '' if t.balls == 1 else 's')]
        out += ['', '<!-- pattern: %s balls: %d -->' % (t.pattern, t.balls),
                '### %s' % (_cell(t.label),), '']
        if not t.rows:
            out.append(_NO_RECORD)
            continue
        out += [_TABLE_HEAD, _TABLE_RULE]
        for r in t.rows:
            ended = '%s (%s)' % (category(r.end_code), _cell(r.end_code))
            note = _cell(r.note)
            out.append('| %s | %d | %d | %s | %s |'
                       % (r.when_local, r.throws, r.catches, ended, note))
    return '\n'.join(out) + '\n'


def _parse_row(line: str, where: str) -> Record:
    cells = [c.strip() for c in line.strip().split('|')]
    if len(cells) != 7 or cells[0] or cells[-1]:
        raise RecordsFormatError('%s: a row needs 5 cells: %r' % (where, line))
    when, throws, catches, ended, note = cells[1:6]
    if not _WHEN_RE.match(when):
        raise RecordsFormatError('%s: bad time %r' % (where, when))
    try:
        n_throws, n_catches = int(throws), int(catches)
    except ValueError:
        raise RecordsFormatError('%s: bad counts in %r' % (where, line))
    if n_throws < 0 or n_catches < 0:
        raise RecordsFormatError('%s: negative count in %r' % (where, line))
    m = _ENDED_RE.match(ended)
    if not m or m.group(1) not in CATEGORIES:
        raise RecordsFormatError('%s: bad "How it ended" %r' % (where, ended))
    return Record(when, n_throws, n_catches, m.group(2), note)


def parse(text: str) -> Ledger:
    """Parse a rendered file. Raises :class:`RecordsFormatError`."""
    lines = text.splitlines()
    if _FILE_MARKER not in lines:
        raise RecordsFormatError('not a records file (no %s)' % _FILE_MARKER)
    tables: Dict[str, Table] = {}
    i = 0
    while i < len(lines):
        m = _MARKER_RE.match(lines[i].strip())
        if not m:
            if lines[i].lstrip().startswith('|'):
                raise RecordsFormatError('line %d: table row outside a '
                                         'Pattern' % (i + 1,))
            i += 1
            continue
        pattern, balls = m.group(1), int(m.group(2))
        if pattern in tables:
            raise RecordsFormatError('Pattern %r appears twice' % (pattern,))
        if pattern in FEED_PATTERNS:
            raise RecordsFormatError('%r is a Feed, not a Pattern' % (pattern,))
        i += 1
        if i >= len(lines) or not lines[i].startswith('### '):
            raise RecordsFormatError('line %d: Pattern %r has no heading'
                                     % (i + 1, pattern))
        label = lines[i][4:].strip()
        i += 1
        rows: List[Record] = []
        seen_head = False
        while i < len(lines):
            s = lines[i].strip()
            if _MARKER_RE.match(s) or s.startswith('#'):
                break
            if s == _TABLE_HEAD and not seen_head:
                seen_head = True
                if i + 1 >= len(lines) or lines[i + 1].strip() != _TABLE_RULE:
                    raise RecordsFormatError('line %d: table rule missing'
                                             % (i + 2,))
                i += 2
                continue
            if s.startswith('|'):
                if not seen_head:
                    raise RecordsFormatError('line %d: row before the table '
                                             'header' % (i + 1,))
                rows.append(_parse_row(s, 'line %d' % (i + 1,)))
            i += 1
        for a, b in zip(rows, rows[1:]):
            if b.catches <= a.catches:
                raise RecordsFormatError(
                    'Pattern %r: a record row (%d Catches) does not beat the '
                    'one before it (%d)' % (pattern, b.catches, a.catches))
        tables[pattern] = Table(pattern, label, balls, tuple(rows))
    model = empty()
    for p, t in tables.items():
        if p in _BALLS and t.balls != _BALLS[p]:
            raise RecordsFormatError('Pattern %r: balls %d, expected %d'
                                     % (p, t.balls, _BALLS[p]))
        model = model.replace(t)
    return model


def apply_attempt(model: Ledger, attempt: Attempt) -> Tuple[Ledger, bool]:
    """(new model, beaten). Beaten iff strictly more Catches than the
    Pattern's record (or than 0 when it has none). A Feed or an unknown
    Pattern id is never a record (``ValueError``)."""
    pattern = str(attempt.pattern)
    if pattern in FEED_PATTERNS:
        raise ValueError('%r is a Feed, not a Pattern' % (pattern,))
    table = model.table(pattern)
    if table is None:
        raise ValueError('unknown Pattern %r (records.PATTERNS)' % (pattern,))
    if not _WHEN_RE.match(str(attempt.when_local)):
        raise ValueError('bad when_local %r (want %s)'
                         % (attempt.when_local, WHEN_FORMAT))
    best = table.best.catches if table.best else 0
    if int(attempt.catches) <= best:
        return model, False
    row = Record(str(attempt.when_local), int(attempt.throws),
                 int(attempt.catches), str(attempt.end_code or 'COMPLETED'),
                 str(attempt.note or ''))
    return model.replace(dataclasses.replace(table, rows=table.rows + (row,))), True


def rebuild(attempts: Iterable[Attempt]) -> Ledger:
    """Replay Attempts from empty in chronological order (stable sort on
    ``when_local``); Feeds are skipped."""
    model = empty()
    for a in sorted(attempts, key=lambda a: a.when_local):
        if a.pattern in FEED_PATTERNS:
            continue
        model, _beaten = apply_attempt(model, a)
    return model


def read_attempts_csv(path: str) -> List[Attempt]:
    """The seed CSV: exactly :data:`CSV_COLUMNS`. Rows whose ``source`` is
    the sim are refused (a record is the robot's)."""
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        if tuple(reader.fieldnames or ()) != CSV_COLUMNS:
            raise ValueError('%s: columns %r, want %r'
                             % (path, reader.fieldnames, CSV_COLUMNS))
        out = []
        for n, row in enumerate(reader, start=2):
            if str(row['source']).strip().lower().startswith('sim'):
                raise ValueError('%s:%d: a sim Attempt is never a record'
                                 % (path, n))
            out.append(Attempt(row['when_local'].strip(),
                               row['pattern'].strip(),
                               int(row['throws']), int(row['catches']),
                               row['end_code'].strip(),
                               seed_note(row['source'])))
    return out


def seed_note(source: str) -> str:
    """The Note a seeded row carries, from the CSV's ``source`` text. A row
    reconstructed from per-throw lines (logs before 2026-09-29 have no Attempt
    summary line, so the Attempt span and its tally are approximate) says so;
    a fed goal says so; anything else is a plain log-seeded row."""
    src = str(source or '')
    if 'per-throw' in src:
        return 'seed: reconstructed from per-throw lines, no summary line (approximate)'
    if '[reload]' in src:
        return 'seed: fed goal'
    return 'seed: log'


def to_json(model: Ledger) -> dict:
    """The GUI's view (``GET /api/records``), in file order."""
    pats = []
    for t in model.ordered():
        b = t.best
        pats.append({
            'pattern': t.pattern, 'label': t.label, 'balls': t.balls,
            'history': len(t.rows),
            'record': None if b is None else {
                'when_local': b.when_local, 'throws': b.throws,
                'catches': b.catches, 'end_code': b.end_code,
                'category': category(b.end_code), 'note': b.note},
        })
    return {'rule': RULE, 'patterns': pats}


def load(path: str) -> Ledger:
    """Parse ``path``; a missing file is the empty ledger."""
    if not os.path.exists(path):
        return empty()
    with open(path, encoding='utf-8') as f:
        return parse(f.read())


def write_atomic(path: str, text: str) -> None:
    """Temp file in the same directory, fsync, ``os.replace``."""
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.records-', suffix='.tmp', dir=d)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def record_attempt(path: str, attempt: Attempt) -> Tuple[bool, int]:
    """The one writer. Returns (beaten, the Pattern's record Catches after
    this Attempt). Raises :class:`RecordsFormatError` on a malformed file,
    which is left untouched; writes only when beaten or the file is new."""
    existed = os.path.exists(path)
    model = load(path)
    model, beaten = apply_attempt(model, attempt)
    if beaten or not existed:
        write_atomic(path, render(model))
    best = model.table(attempt.pattern).best
    return beaten, (best.catches if best else 0)


def _show(model: Ledger) -> str:
    lines = []
    for t in model.ordered():
        b = t.best
        lines.append('%-26s %d ball%s  %s' % (
            t.label, t.balls, ' ' if t.balls == 1 else 's',
            'no record' if b is None else '%d/%d  %s  %s (%s)' % (
                b.catches, b.throws, b.when_local, category(b.end_code),
                b.end_code)))
    return '\n'.join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog='python -m jugglebot.records',
                                 description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    rb = sub.add_parser('rebuild', help='replay a seed CSV into a fresh file')
    rb.add_argument('--attempts', default=None,
                    help='CSV with columns %s (omit: every heading, no rows)'
                    % ','.join(CSV_COLUMNS))
    rb.add_argument('--out', required=True)
    sh = sub.add_parser('show', help='one line per Pattern')
    sh.add_argument('--path', default=os.path.join('docs', 'records.md'))
    args = ap.parse_args(argv)
    if args.cmd == 'rebuild':
        attempts = read_attempts_csv(args.attempts) if args.attempts else []
        write_atomic(args.out, render(rebuild(attempts)))
        print('wrote %s from %d Attempts' % (args.out, len(attempts)))
        return 0
    try:
        print(_show(load(args.path)))
    except RecordsFormatError as exc:
        print('%s: malformed: %s' % (args.path, exc), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
