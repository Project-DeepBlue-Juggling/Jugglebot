"""Jugglebot records: the ledger of the most Catches per Pattern (IDEA-3).

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
* Ledger format v2 (FEAT-29, 2026-11-01): a last ``Parameters`` column records
  what the run used, ``key=value`` pairs joined by ``; `` in insertion order
  (e.g. ``apex_m=0.95; separation_mm=125; dwell_s=0.27; planner=pose``); an
  empty cell means not recorded. :func:`parse` still accepts a v1 file (five
  cells, no Parameters); :func:`render` always writes v2, so the first write
  after the upgrade migrates the file. Keys and values never contain ``|``,
  ``;`` or ``=`` (:func:`_param` strips them).
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
* ``other``     -- everything else (``ABORTED_MODE_CHANGED``, ``ABORTED_FAULT``, ``LANE_REFUSED``,
  ``REJECTED_*``, ``ATTEMPT_ENDED``, ...).

The literals are restated, not imported, from ``motion/skills/executor.py`` so
the GUI server can load this file without the motion package;
``tests/motion/test_records.py`` pins them against the executor's constants.

Drops to date (FEAT-47, 2026-11-10)
-----------------------------------
A section near the top counts every drop: a ball thrown to be caught by
Jugglebot that was not caught (``motion/skills/report.py::drop_count``; the
columns Stop and an unverifiable final Catch are not drops). It is parsed from
the machine marker ``<!-- drops: live=<n> since=<YYYY-MM-DD> history=<m|none> -->``
and the total shown is ``history + live``: ``live`` is counted by this module
from ``since`` on, ``history`` is the figure :func:`set_drops_history` writes
(``tools/set_drops_history.py``; never by hand; ``none`` = not yet counted, read
as 0). A history written with its dates carries them as the optional marker
fields ``history_from=<date> history_through=<date>`` (a marker without them
parses as before, and every live write preserves them).

The same section also keeps the CAUGHT side so a percentage can be shown (owner,
2026-11-11): optional ``caught=<n>`` (live, after ``since``) and
``history_caught=<m>`` (after ``history``). A Throw or Feed "to catch" is a ball
thrown to be caught by Jugglebot, so ``throws = caught + drops`` BY
CONSTRUCTION (``report.caught_count`` and ``report.drop_count`` share one base)
and the denominator can never disagree with the drops figure. A marker without
the fields parses and re-renders byte-identically; the figures are exposed
(:attr:`Drops.throws_total`, ``to_json`` ``caught``/``throws``) only when every
part that has drops also has its caught figure, else ``None``: never a
percentage built on a denominator that misses a part. A file without the section is "not counted yet"
(:attr:`Ledger.drops` is ``None``; the v2 file parses unchanged) and the first
write after the upgrade adds it. Feeds (``reload``/``magazine``) are not
records but their drops count: :func:`record_attempt` books them without a
table row.

Writes
------
:func:`record_attempt` is the one writer: it parses the existing file (a
malformed file raises :class:`RecordsFormatError` and is NEVER overwritten),
applies the Attempt, and writes only when a record was beaten, the file did
not exist, the Attempt carried drops or catches, or the drops section is not there yet
(the first write after the upgrade), atomically (temp file in the same
directory + ``os.replace``).
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
import datetime
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
    ('columns_turned', 'Columns — turned', 2),
    ('columns_highlow', 'Columns — high-low', 2),
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
_TABLE_HEAD = '| When | Throws | Catches | How it ended | Note | Parameters |'
_TABLE_RULE = '|---|---:|---:|---|---|---|'
_TABLE_HEAD_V1 = '| When | Throws | Catches | How it ended | Note |'
_TABLE_RULE_V1 = '|---|---:|---:|---|---|'
_DROPS_RE = re.compile(r'^<!-- drops: live=(\d+) since=(\d{4}-\d{2}-\d{2}) '
                       r'(?:caught=(\d+) )?history=(\d+|none)'
                       r'(?: history_caught=(\d+))?'
                       r'(?: history_from=(\d{4}-\d{2}-\d{2}) '
                       r'history_through=(\d{4}-\d{2}-\d{2}))? -->$')
_SCORES_RE = re.compile(r'^<!-- scores: pattern=([a-z0-9_]+) -->$')
_FEED_SCORES_MARK = '<!-- feed-scores -->'
_SCORES_HEAD = ('| When | Scored | Median mm | Without a Fit | Scores mm |')
_SCORES_RULE = '|---|---:|---:|---:|---|'
_FEED_SCORES_HEAD = '| When | Pattern | Score mm |'
_FEED_SCORES_RULE = '|---|---|---:|'
_NO_RECORD = '_No record yet._'
_FILE_MARKER = '<!-- jugglebot-records v2 -->'
_FILE_MARKER_V1 = '<!-- jugglebot-records v1 -->'

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
    #: (key, value) pairs the run used, in insertion order (FEAT-29).
    params: Tuple[Tuple[str, str], ...] = ()
    #: Balls thrown to be caught that were not (``report.drop_count``).
    drops: int = 0
    #: Catches that pair with ``drops`` (``report.caught_count``); ``None`` =
    #: the writer does not report them (the ledger's caught figure is untouched).
    caught: Optional[int] = None
    #: Landing scores (IDEA-21, owner's decision in TKT-63): per Throw in
    #: Throw order, mm (``report.landing_scores``); ``no_fit`` the Throws
    #: with no Fit (no score); ``feed_scores`` one per Ball Butler Feed that
    #: seated, kept apart from the Throws. Written whether or not a record
    #: is beaten; none of them touches a record or a drops figure.
    scores: Tuple[float, ...] = ()
    no_fit: int = 0
    feed_scores: Tuple[float, ...] = ()


@dataclasses.dataclass(frozen=True)
class ScoreRow:
    """One Attempt's Throw landing scores for a Pattern: whole mm, in Throw
    order; ``median_mm`` over them (``None`` when none was scored)."""
    when_local: str
    pattern: str
    scores: Tuple[int, ...]
    no_fit: int = 0

    @property
    def median_mm(self) -> Optional[int]:
        return _median_int(self.scores)


@dataclasses.dataclass(frozen=True)
class FeedScore:
    """One Ball Butler Feed's landing score, whole mm."""
    when_local: str
    pattern: str
    score_mm: int


def _median_int(values: Sequence[int]) -> Optional[int]:
    v = sorted(values)
    n = len(v)
    if not n:
        return None
    m = v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])
    return int(m + 0.5)


@dataclasses.dataclass(frozen=True)
class Drops:
    """The "Drops to date" figure: ``live`` counted from ``since`` (a date),
    plus a ``history`` count a backfill sets (``None`` = not yet counted)."""
    live: int
    since: str
    history: Optional[int] = None
    #: First and last date the history figure counts (``set_drops_history``);
    #: ``None`` on a marker written before they existed.
    history_from: Optional[str] = None
    history_through: Optional[str] = None
    #: Catches paired with ``live`` / ``history`` (``report.caught_count``);
    #: ``None`` = not recorded (a marker written before they existed).
    caught: Optional[int] = None
    history_caught: Optional[int] = None

    @property
    def total(self) -> int:
        return self.live + (self.history or 0)

    @property
    def caught_total(self) -> Optional[int]:
        """Catches, live + history; ``None`` when a part that has drops lacks
        its caught figure (a part with no drops and no figure counts 0)."""
        live = self.caught if self.caught is not None else (
            0 if self.live == 0 else None)
        hist = self.history_caught if self.history_caught is not None else (
            0 if not self.history else None)
        if live is None or hist is None:
            return None
        return live + hist

    @property
    def throws_total(self) -> Optional[int]:
        """Balls thrown to be caught: caught + drops; ``None`` as above."""
        c = self.caught_total
        return None if c is None else c + self.total


@dataclasses.dataclass(frozen=True)
class Record:
    """One row of a Pattern's table: the Attempt that set that record."""
    when_local: str
    throws: int
    catches: int
    end_code: str
    note: str = ''
    params: Tuple[Tuple[str, str], ...] = ()


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
    #: ``None`` = the file has no drops section ("not counted yet").
    drops: Optional[Drops] = None
    #: Landing scores (IDEA-21); empty = the file has no such section.
    scores: Tuple[ScoreRow, ...] = ()
    feed_scores: Tuple[FeedScore, ...] = ()

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
        return dataclasses.replace(
            _ledger(rest + [table], self.drops),
            scores=self.scores, feed_scores=self.feed_scores)


def _ledger(tables: Iterable[Table], drops: Optional[Drops] = None) -> Ledger:
    """The canonical (file-order) tuple, so equal ledgers compare equal."""
    return Ledger(tables=tuple((t.pattern, t) for t in sorted(
        tables, key=lambda t: (t.balls, t.pattern))), drops=drops)


def empty() -> Ledger:
    """Every Pattern heading, no rows."""
    return _ledger(Table(p, label, balls) for p, label, balls in PATTERNS)


def _cell(text: str) -> str:
    return ' '.join(str(text).replace('|', '/').split())


def _param(text) -> str:
    """One parameter key or value: whitespace collapsed, the cell and pair
    separators (``|``, ``;``, ``=``) replaced by ``/`` so the cell parses back."""
    out = str(text)
    for ch in '|;=':
        out = out.replace(ch, '/')
    return ' '.join(out.split())


def _params_cell(params) -> str:
    pairs = []
    for k, v in params:
        k, v = _param(k), _param(v)
        if k:
            pairs.append('%s=%s' % (k, v))
    return '; '.join(pairs)


def _parse_params(cell: str, where: str) -> Tuple[Tuple[str, str], ...]:
    if not cell:
        return ()
    out = []
    for part in cell.split(';'):
        k, eq, v = part.partition('=')
        k, v = k.strip(), v.strip()
        if not eq or not k:
            raise RecordsFormatError('%s: bad Parameters %r' % (where, cell))
        out.append((k, v))
    return tuple(out)


def _drops_sentence(d: Drops) -> str:
    """The one human line under the marker (never parsed)."""
    if d.history is None:
        hist = 'history not yet counted'
    elif d.history_from and d.history_through:
        hist = '%d from history, %s to %s' % (
            d.history, d.history_from, d.history_through)
    else:
        hist = '%d from history' % d.history
    t = d.throws_total
    pct = ''
    if t:
        p = round(100.0 * d.caught_total / t, 1)
        pct = ' %s Throws and Feeds to catch, %.1f %% caught, %.1f %% dropped.' % (
            format(t, ','), p, round(100.0 - p, 1))
    return ('Drops to date: %d (%d counted live since %s; %s).%s A drop is a '
            'ball thrown to be caught by Jugglebot that was not caught.'
            % (d.total, d.live, d.since, hist, pct))


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
        '`skill_node` writes a gitignored runtime copy of this file, '
        '`temp/runtime/records.md`, at the end of each Attempt (ROS '
        'parameter `records_path`, set by `jugglebot_launch.py`); the '
        'Sitting debrief copies it back here (`tools/runtime_records.py '
        'merge`) and commits it. The tables are rendered and parsed '
        'by `ros_ws/src/jugglebot/jugglebot/records.py`; a Note cell may be '
        'edited by hand, nothing else. "Parameters" is what that run used '
        '(`key=value` pairs; empty = not recorded). "How it ended" is completed, stopped, '
        'drop (a missed or lost ball), compute (a solve or limit refusal) or '
        'other, with the raw end code in parentheses.',
    ]
    d = model.drops
    if d is not None:
        out += ['', '## Drops to date', '',
                '<!-- drops: live=%d since=%s%s history=%s%s%s -->'
                % (d.live, d.since,
                   '' if d.caught is None else ' caught=%d' % d.caught,
                   'none' if d.history is None else d.history,
                   '' if d.history_caught is None
                   else ' history_caught=%d' % d.history_caught,
                   (' history_from=%s history_through=%s'
                    % (d.history_from, d.history_through))
                   if d.history_from and d.history_through else ''),
                '', _drops_sentence(d)]
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
            out.append('| %s | %d | %d | %s | %s | %s |'
                       % (r.when_local, r.throws, r.catches, ended, note,
                          _params_cell(r.params)))
    if model.scores or model.feed_scores:
        out += ['', '## Landing scores', '',
                'A landing score is the distance in mm from the Fit\'s '
                'landing to the target Site, one per Throw (Throw order); '
                '"Without a Fit" counts the Throws with no landing, which '
                'have no score. A Feed\'s score is kept apart. Written at '
                'the end of every Attempt, whether or not a record is '
                'beaten.']
        for t in model.ordered():
            rows = [r for r in model.scores if r.pattern == t.pattern]
            if not rows:
                continue
            out += ['', '<!-- scores: pattern=%s -->' % (t.pattern,),
                    '### %s' % (_cell(t.label),), '',
                    _SCORES_HEAD, _SCORES_RULE]
            for r in rows:
                med = r.median_mm
                out.append('| %s | %d | %s | %d | %s |' % (
                    r.when_local, len(r.scores),
                    '-' if med is None else med, r.no_fit,
                    ' '.join(str(x) for x in r.scores) or '-'))
        if model.feed_scores:
            out += ['', _FEED_SCORES_MARK, '### Feeds', '',
                    _FEED_SCORES_HEAD, _FEED_SCORES_RULE]
            for f in model.feed_scores:
                out.append('| %s | %s | %d |'
                           % (f.when_local, f.pattern, f.score_mm))
    return '\n'.join(out) + '\n'


def _table_rows(lines: List[str], i: int, head: str, rule: str,
                where: str) -> Tuple[List[List[str]], int]:
    """The cell lists of the table that follows the marker at ``lines[i]``,
    and the index after it."""
    i += 1
    while i < len(lines) and lines[i].strip() != head:
        s = lines[i].strip()
        if s and not s.startswith('### '):
            raise RecordsFormatError('%s: expected the table header, got %r'
                                     % (where, s))
        i += 1
    if i + 1 >= len(lines) or lines[i + 1].strip() != rule:
        raise RecordsFormatError('%s: table header or rule missing' % (where,))
    i += 2
    rows = []
    while i < len(lines) and lines[i].strip().startswith('|'):
        cells = [c.strip() for c in lines[i].strip().split('|')]
        rows.append(cells[1:-1])
        i += 1
    return rows, i


def _check_when(when: str, where: str) -> str:
    if not _WHEN_RE.match(when):
        raise RecordsFormatError('%s: bad time %r' % (where, when))
    return when


def _parse_score_row(pattern: str, cells: List[str], where: str) -> ScoreRow:
    if len(cells) != 5:
        raise RecordsFormatError('%s: a score row needs 5 cells' % (where,))
    when, scored, med, no_fit, scores = cells
    try:
        vals = (() if scores == '-'
                else tuple(int(x) for x in scores.split()))
        n_scored, n_no_fit = int(scored), int(no_fit)
    except ValueError:
        raise RecordsFormatError('%s: bad score row %r' % (where, cells))
    if (min(vals, default=0) < 0 or n_no_fit < 0 or n_scored != len(vals)):
        raise RecordsFormatError('%s: scores disagree with Scored %r'
                                 % (where, scored))
    row = ScoreRow(_check_when(when, where), pattern, vals, n_no_fit)
    if med != ('-' if row.median_mm is None else str(row.median_mm)):
        raise RecordsFormatError('%s: Median mm %r is not the median of the '
                                 'scores' % (where, med))
    return row


def _parse_feed_score_row(cells: List[str], where: str) -> FeedScore:
    if len(cells) != 3:
        raise RecordsFormatError('%s: a Feed score row needs 3 cells'
                                 % (where,))
    try:
        mm = int(cells[2])
    except ValueError:
        raise RecordsFormatError('%s: bad Feed score %r' % (where, cells[2]))
    if mm < 0:
        raise RecordsFormatError('%s: negative Feed score' % (where,))
    return FeedScore(_check_when(cells[0], where), cells[1], mm)


def _parse_row(line: str, where: str, v1: bool = False) -> Record:
    cells = [c.strip() for c in line.strip().split('|')]
    n = 5 if v1 else 6
    if len(cells) != n + 2 or cells[0] or cells[-1]:
        raise RecordsFormatError('%s: a row needs %d cells: %r'
                                 % (where, n, line))
    when, throws, catches, ended, note = cells[1:6]
    params = () if v1 else _parse_params(cells[6], where)
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
    return Record(when, n_throws, n_catches, m.group(2), note, params)


def parse(text: str) -> Ledger:
    """Parse a rendered file. Raises :class:`RecordsFormatError`."""
    lines = text.splitlines()
    if _FILE_MARKER in lines:
        v1 = False
    elif _FILE_MARKER_V1 in lines:
        v1 = True
    else:
        raise RecordsFormatError('not a records file (no %s)' % _FILE_MARKER)
    head, rule = ((_TABLE_HEAD_V1, _TABLE_RULE_V1) if v1
                  else (_TABLE_HEAD, _TABLE_RULE))
    tables: Dict[str, Table] = {}
    drops: Optional[Drops] = None
    scores: List[ScoreRow] = []
    feed_scores: List[FeedScore] = []
    i = 0
    while i < len(lines):
        m = _MARKER_RE.match(lines[i].strip())
        sm = _SCORES_RE.match(lines[i].strip())
        if sm or lines[i].strip() == _FEED_SCORES_MARK:
            where = 'line %d' % (i + 1,)
            if sm:
                pat = sm.group(1)
                if pat not in _BALLS:
                    raise RecordsFormatError('%s: scores for unknown Pattern '
                                             '%r' % (where, pat))
                cells, i = _table_rows(lines, i, _SCORES_HEAD, _SCORES_RULE,
                                       where)
                scores += [_parse_score_row(pat, c, where) for c in cells]
            else:
                cells, i = _table_rows(lines, i, _FEED_SCORES_HEAD,
                                       _FEED_SCORES_RULE, where)
                feed_scores += [_parse_feed_score_row(c, where)
                                for c in cells]
            continue
        if not m:
            if lines[i].strip().startswith('<!-- drops:'):
                dm = _DROPS_RE.match(lines[i].strip())
                if not dm or drops is not None:
                    raise RecordsFormatError('line %d: bad or repeated drops '
                                             'marker' % (i + 1,))
                drops = Drops(
                    int(dm.group(1)), dm.group(2),
                    None if dm.group(4) == 'none' else int(dm.group(4)),
                    dm.group(6), dm.group(7),
                    None if dm.group(3) is None else int(dm.group(3)),
                    None if dm.group(5) is None else int(dm.group(5)))
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
            if (_MARKER_RE.match(s) or s.startswith('#')
                    or _SCORES_RE.match(s) or s == _FEED_SCORES_MARK):
                break
            if s == head and not seen_head:
                seen_head = True
                if i + 1 >= len(lines) or lines[i + 1].strip() != rule:
                    raise RecordsFormatError('line %d: table rule missing'
                                             % (i + 2,))
                i += 2
                continue
            if s.startswith('|'):
                if not seen_head:
                    raise RecordsFormatError('line %d: row before the table '
                                             'header' % (i + 1,))
                rows.append(_parse_row(s, 'line %d' % (i + 1,), v1))
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
    order = {p: n for n, (p, _t) in enumerate(model.tables)}
    return dataclasses.replace(
        model, drops=drops, scores=_sorted_scores(scores, order),
        feed_scores=tuple(feed_scores))


def _sorted_scores(rows: Iterable[ScoreRow],
                   order: Dict[str, int]) -> Tuple[ScoreRow, ...]:
    """Grouped by Pattern in file order, chronological within one (a stable
    sort), so a ledger equals its own parse."""
    return tuple(sorted(rows, key=lambda r: order.get(r.pattern, 0)))


def apply_scores(model: Ledger, when_local: str, pattern: str,
                 scores: Sequence[float] = (), no_fit: int = 0,
                 feed_scores: Sequence[float] = ()) -> Ledger:
    """Append one Attempt's landing scores (IDEA-21): a Throw row for a
    Pattern when it has scores or Throws without a Fit (never for a Feed
    goal), a Feed row per Feed score. Scores are rounded to whole mm;
    negative or non-finite ones are ignored. ``ValueError`` on a bad time."""
    if not _WHEN_RE.match(str(when_local)):
        raise ValueError('bad when_local %r (want %s)'
                         % (when_local, WHEN_FORMAT))

    def ok(v) -> bool:
        return isinstance(v, (int, float)) and 0.0 <= v < float('inf')

    ints = tuple(int(round(float(v))) for v in scores if ok(v))
    n_no_fit = max(0, int(no_fit))
    rows = model.scores
    if pattern in _BALLS and pattern not in FEED_PATTERNS and (
            ints or n_no_fit):
        order = {p: n for n, (p, _t) in enumerate(model.tables)}
        rows = _sorted_scores(
            rows + (ScoreRow(str(when_local), pattern, ints, n_no_fit),),
            order)
    feeds = model.feed_scores + tuple(
        FeedScore(str(when_local), str(pattern), int(round(float(v))))
        for v in feed_scores if ok(v))
    return dataclasses.replace(model, scores=rows, feed_scores=feeds)


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
                 str(attempt.note or ''),
                 tuple((_param(k), _param(v)) for k, v in attempt.params
                       if _param(k)))
    return model.replace(dataclasses.replace(table, rows=table.rows + (row,))), True


def apply_drops(model: Ledger, when_local: str, drops: int,
                caught: Optional[int] = None) -> Ledger:
    """Add ``drops`` (and the paired ``caught``) to the live counts; the first
    call on a ledger without the section starts it (``since`` the Attempt's
    date, history ``none``). A live caught figure that was never recorded
    stays unrecorded when live drops already exist (it would miss them), and
    starts at 0 + ``caught`` while live is still 0; every other field is
    preserved. ``caught=None`` leaves the caught figure exactly as it is."""
    d = model.drops
    n = max(0, int(drops))
    if d is None:
        d = Drops(0, str(when_local)[:10], None)
    if caught is None:
        return dataclasses.replace(model, drops=dataclasses.replace(
            d, live=d.live + n))
    c = max(0, int(caught))
    base = d.caught if d.caught is not None else (0 if d.live == 0 else None)
    return dataclasses.replace(model, drops=dataclasses.replace(
        d, live=d.live + n, caught=None if base is None else base + c))


def _check_date(name: str, value: str) -> str:
    if (not isinstance(value, str)
            or not re.match(r'^\d{4}-\d{2}-\d{2}$', value)):
        raise ValueError('%s must be a YYYY-MM-DD date, got %r' % (name, value))
    try:
        datetime.datetime.strptime(value, '%Y-%m-%d')
    except ValueError:
        raise ValueError('%s must be a YYYY-MM-DD date, got %r' % (name, value))
    return value


def set_drops_history(path: str, history: int, through: str, *,
                      since: Optional[str] = None,
                      first: Optional[str] = None,
                      history_caught: Optional[int] = None) -> Ledger:
    """Write the backfilled history figure into ``path``'s drops section.

    ``history`` drops counted from ``first`` through ``through`` (dates). The
    section is created when absent (live 0, ``since`` = ``since`` or
    ``through``); when present ONLY the history fields change (live and
    ``since`` untouched). ``ValueError`` (``RecordsFormatError`` is one) and
    the file untouched on a malformed existing marker, a negative or
    non-integer figure, a bad date or ``first`` after ``through``. The dates
    are stored only when ``first`` is given. ``history_caught`` is the Catches
    paired with ``history`` (``None`` = not recorded, which also clears an
    older figure: it belonged to the older history); a negative or
    non-integer one is refused."""
    if isinstance(history, bool) or not isinstance(history, int) or history < 0:
        raise ValueError('history must be a non-negative integer, got %r'
                         % (history,))
    if history_caught is not None and (
            isinstance(history_caught, bool)
            or not isinstance(history_caught, int) or history_caught < 0):
        raise ValueError('history_caught must be a non-negative integer, '
                         'got %r' % (history_caught,))
    through = _check_date('through', through)
    if first is not None:
        first = _check_date('first', first)
        if first > through:
            raise ValueError('first %s is after through %s' % (first, through))
    if since is not None:
        since = _check_date('since', since)
    model = load(path)               # a malformed marker raises, file untouched
    d = model.drops
    if d is None:
        d = Drops(0, since or through, None)
    d = dataclasses.replace(d, history=history, history_caught=history_caught,
                            history_from=first,
                            history_through=through if first else None)
    model = dataclasses.replace(model, drops=d)
    write_atomic(path, render(model))
    return model


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
                'category': category(b.end_code), 'note': b.note,
                'params': {k: v for k, v in b.params}},
        })
    d = model.drops
    drops = None if d is None else {
        'total': d.total, 'live': d.live, 'since': d.since,
        'history': d.history, 'history_from': d.history_from,
        'history_through': d.history_through,
        'caught': d.caught_total, 'throws': d.throws_total}
    return {'rule': RULE, 'patterns': pats, 'drops': drops}


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
    this Attempt; 0 for a Feed). Raises :class:`RecordsFormatError` on a
    malformed file, which is left untouched. Writes when a record was beaten,
    the file is new, the Attempt carried drops or catches, or the drops section is not
    in the file yet. A Feed books its drops only, never a record row."""
    existed = os.path.exists(path)
    has_scores = bool(attempt.scores or attempt.no_fit or attempt.feed_scores)
    if (attempt.pattern in FEED_PATTERNS and int(attempt.drops) <= 0
            and int(attempt.caught or 0) <= 0 and not has_scores):
        return False, 0            # a Feed with no drops or catches touches nothing
    model = load(path)
    if attempt.pattern in FEED_PATTERNS:
        beaten, best = False, 0
    else:
        model, beaten = apply_attempt(model, attempt)
        t = model.table(attempt.pattern).best
        best = t.catches if t else 0
    n = max(0, int(attempt.drops))
    c = None if attempt.caught is None else max(0, int(attempt.caught))
    if has_scores:
        model = apply_scores(model, attempt.when_local, attempt.pattern,
                             attempt.scores, attempt.no_fit,
                             attempt.feed_scores)
    if (beaten or not existed or n > 0 or (c or 0) > 0 or has_scores
            or model.drops is None):
        model = apply_drops(model, attempt.when_local, n, c)
        write_atomic(path, render(model))
    return beaten, best


def total_drops(path: str) -> Optional[int]:
    """The "Drops to date" total in ``path`` (``None`` = not counted yet or
    unreadable). For the log line only."""
    try:
        d = load(path).drops
    except (OSError, RecordsFormatError):
        return None
    return None if d is None else d.total


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
