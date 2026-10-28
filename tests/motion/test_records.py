"""The records ledger (``jugglebot/records.py``, JB-35): the file format
round-trips, the strictly-more rule, ball-count ordering, the end-code
category map (pinned against the executor's own constants), the seed CSV
rebuild, the malformed-file refusal and the atomic write."""
from __future__ import annotations

import os
import re
from pathlib import Path

import pytest

from jugglebot import records as rec
from jugglebot.motion.skills import executor as ex

REPO = Path(__file__).resolve().parents[2]
SKILL_NODE = REPO / 'ros_ws' / 'src' / 'jugglebot' / 'jugglebot' / 'skill_node.py'


def _a(when, pattern, catches, throws=None, code='COMPLETED'):
    return rec.Attempt(when, pattern, catches + 1 if throws is None else throws,
                       catches, code)


# ── The Pattern set and its order ──────────────────────────────────────────

def test_patterns_are_exactly_the_skill_nodes_minus_the_feed():
    """A new Pattern in skill_node must get a ball count here, or it would
    never be recorded (apply_attempt refuses unknown ids)."""
    m = re.search(r"_PATTERNS = \(([^)]*)\)", SKILL_NODE.read_text())
    node = set(re.findall(r"'([a-z0-9_]+)'", m.group(1)))
    assert rec.FEED_PATTERNS <= node
    assert {p for p, _l, _b in rec.PATTERNS} == node - rec.FEED_PATTERNS


def test_tables_are_ordered_by_balls_then_pattern():
    order = [(t.balls, t.pattern) for t in rec.empty().ordered()]
    assert order == sorted(order)
    assert order[-1] == (2, 'columns')
    text = rec.render(rec.empty())
    assert text.index('## 1 ball') < text.index('## 2 balls')
    assert '<!-- pattern: columns balls: 2 -->' in text


def test_the_committed_file_parses_and_is_a_fixed_point_of_render():
    path = REPO / 'docs' / 'records.md'
    text = path.read_text(encoding='utf-8')
    assert rec.render(rec.parse(text)) == text


# ── Round trip and the strictly-more rule ──────────────────────────────────

def test_round_trip_is_byte_identical():
    model = rec.rebuild([
        _a('2026-10-20 10:00', 'columns', 3, code='MISSED_CATCH'),
        _a('2026-10-21 11:30', 'columns', 9, code='STOPPED'),
        _a('2026-10-21 12:00', 'hop', 4, code='SPLICE_TOO_LATE'),
    ])
    model, _ = rec.apply_attempt(model, rec.Attempt(
        '2026-10-22 09:15', 'hop', 7, 5, 'LIMIT_JERK', note='a | note\nsplit'))
    text = rec.render(model)
    again = rec.parse(text)
    assert rec.render(again) == text
    assert again == rec.parse(rec.render(again))
    assert '| 2026-10-22 09:15 | 7 | 5 | compute (LIMIT_JERK) | a / note split |' in text
    assert '| 2026-10-21 11:30 | 10 | 9 | stopped (STOPPED) |  |' in text


def test_only_strictly_more_catches_beats_a_record():
    model, beaten = rec.apply_attempt(rec.empty(), _a('2026-10-20 10:00', 'hop', 0))
    assert not beaten                                   # 0 Catches is no record
    model, beaten = rec.apply_attempt(model, _a('2026-10-20 10:01', 'hop', 5))
    assert beaten
    model, beaten = rec.apply_attempt(model, _a('2026-10-20 10:02', 'hop', 5, throws=5))
    assert not beaten                                   # a tie is not a record
    model, beaten = rec.apply_attempt(model, _a('2026-10-20 10:03', 'hop', 4))
    assert not beaten
    model, beaten = rec.apply_attempt(model, _a('2026-10-20 10:04', 'hop', 6))
    assert beaten
    assert [r.catches for r in model.table('hop').rows] == [5, 6]   # history, newest last
    assert model.table('columns').rows == ()


@pytest.mark.parametrize('pattern', sorted(rec.FEED_PATTERNS) + ['nope'])
def test_a_feed_or_unknown_pattern_is_never_a_record(pattern):
    with pytest.raises(ValueError):
        rec.apply_attempt(rec.empty(), _a('2026-10-20 10:00', pattern, 3))


# ── The end-code category map ──────────────────────────────────────────────

@pytest.mark.parametrize('code, cat', [
    ('COMPLETED', 'completed'), ('THROWN', 'completed'), ('STOPPED', 'stopped'),
    (ex.MISSED_CATCH, 'drop'), (ex.ABORTED_NO_RELEASE, 'drop'),
    (ex.DROPPED_SURVIVOR_STOPPED, 'drop'), (ex.NO_LANDING, 'drop'),
    (ex.SPLICE_TOO_LATE, 'compute'), (ex.ORIGIN_TOO_LATE, 'compute'),
    (ex.REBASE_DISCONTINUOUS, 'compute'), (ex.WINDOW_TOO_SHORT, 'compute'),
    (ex.UNREACHABLE, 'compute'), (ex.NO_ADMISSIBLE_COMMAND, 'compute'),
    ('LIMIT_JERK', 'compute'), ('CYCLE_INFEASIBLE', 'compute'),
    ('SOLVE_BUDGET_EXCEEDED', 'compute'),
    (ex.ABORTED_MODE_CHANGED, 'other'), (ex.LANE_REFUSED, 'other'),
    (ex.REJECTED_NO_BALL, 'other'), ('REJECTED_BB(busy)', 'other'),
    ('ATTEMPT_ENDED', 'other'), ('', 'other'),
])
def test_category_map(code, cat):
    """The executor constants are imported, so a renamed code fails here."""
    assert rec.category(code) == cat


# ── Seed CSV ───────────────────────────────────────────────────────────────

def _csv(tmp_path, rows, header=','.join(rec.CSV_COLUMNS)):
    p = tmp_path / 'attempts.csv'
    p.write_text(header + '\n' + ''.join(r + '\n' for r in rows))
    return str(p)


def test_rebuild_from_csv_replays_chronologically_and_skips_the_feed(tmp_path):
    path = _csv(tmp_path, [
        '2026-10-21 11:00,columns,12,11,STOPPED,bag',
        '2026-10-20 09:00,columns,6,4,MISSED_CATCH,bag',     # earlier: replayed first
        '2026-10-20 09:30,reload,1,1,COMPLETED,bag',          # a Feed
        '2026-10-22 10:00,columns,11,11,COMPLETED,logbook',   # tie with 11: no row
    ])
    model = rec.rebuild(rec.read_attempts_csv(path))
    assert [(r.when_local, r.catches) for r in model.table('columns').rows] == [
        ('2026-10-20 09:00', 4), ('2026-10-21 11:00', 11)]
    out = tmp_path / 'records.md'
    assert rec.main(['rebuild', '--attempts', path, '--out', str(out)]) == 0
    assert rec.parse(out.read_text()) == model


def test_csv_refuses_wrong_columns_and_sim_rows(tmp_path):
    with pytest.raises(ValueError):
        rec.read_attempts_csv(_csv(tmp_path, [], header='when,pattern,throws'))
    with pytest.raises(ValueError):
        rec.read_attempts_csv(_csv(tmp_path, ['2026-10-20 09:00,hop,3,3,COMPLETED,sim']))


# ── Writes ─────────────────────────────────────────────────────────────────

def test_record_attempt_writes_only_when_beaten(tmp_path):
    path = str(tmp_path / 'records.md')
    assert rec.record_attempt(path, _a('2026-10-20 10:00', 'hop', 0)) == (False, 0)
    assert rec.parse(Path(path).read_text()) == rec.empty()     # new file: headings only
    assert rec.record_attempt(path, _a('2026-10-20 10:01', 'hop', 3)) == (True, 3)
    before = os.stat(path).st_mtime_ns, Path(path).read_bytes()
    assert rec.record_attempt(path, _a('2026-10-20 10:02', 'hop', 2)) == (False, 3)
    assert (os.stat(path).st_mtime_ns, Path(path).read_bytes()) == before
    assert os.listdir(str(tmp_path)) == ['records.md']          # no temp file left


@pytest.mark.parametrize('damage', [
    lambda t: t.replace('<!-- jugglebot-records v1 -->', ''),
    lambda t: t.replace('| 3 | compute', '| x | compute'),
    lambda t: t.replace('compute (SPLICE_TOO_LATE)', 'SPLICE_TOO_LATE'),
    lambda t: t.replace('balls: 2', 'balls: 1'),
    lambda t: t + '\n| 2026-10-20 10:00 | 1 | 1 | completed (COMPLETED) |  |\n',
    lambda t: t.replace('| 2026-10-21 10:00 | 9 | 8 |', '| 2026-10-21 10:00 | 9 | 2 |'),
])
def test_a_malformed_file_is_never_overwritten(tmp_path, damage):
    model = rec.rebuild([_a('2026-10-20 10:00', 'columns', 3, code='SPLICE_TOO_LATE'),
                         _a('2026-10-21 10:00', 'columns', 8)])
    bad = damage(rec.render(model))
    path = tmp_path / 'records.md'
    path.write_text(bad)
    with pytest.raises(rec.RecordsFormatError):
        rec.record_attempt(str(path), _a('2026-10-22 10:00', 'columns', 20))
    assert path.read_text() == bad


def test_write_atomic_replaces_through_a_temp_file(tmp_path, monkeypatch):
    path = tmp_path / 'records.md'
    path.write_text('old')
    seen = []
    real = os.replace

    def spy(src, dst):
        seen.append((os.path.dirname(src), Path(src).read_text(), str(dst)))
        real(src, dst)
    monkeypatch.setattr(rec.os, 'replace', spy)
    rec.write_atomic(str(path), 'new\n')
    assert seen == [(str(tmp_path), 'new\n', str(path))]
    assert path.read_text() == 'new\n'


def test_to_json_is_the_guis_view_in_file_order():
    model = rec.rebuild([_a('2026-10-21 10:00', 'columns', 8, throws=10, code='MISSED_CATCH')])
    j = rec.to_json(model)
    assert [p['pattern'] for p in j['patterns']] == [t.pattern for t in model.ordered()]
    col = j['patterns'][-1]
    assert col['balls'] == 2 and col['history'] == 1
    assert col['record'] == {'when_local': '2026-10-21 10:00', 'throws': 10,
                             'catches': 8, 'end_code': 'MISSED_CATCH',
                             'category': 'drop', 'note': ''}
    assert j['patterns'][0]['record'] is None


def test_seed_note_marks_reconstructed_and_fed_rows():
    """Seeded rows keep their provenance in the Note column: a per-throw
    reconstruction (pre-2026-09-29 logs, no summary line) is flagged as
    approximate, a fed goal is named, anything else is a plain log row."""
    from jugglebot.records import seed_note
    assert 'approximate' in seed_note('~/.ros/log/x/launch.log [per-throw OUTCOME lines counted; no summary line]')
    assert seed_note('~/.ros/log/x/launch.log [reload]') == 'seed: fed goal'
    assert seed_note('~/.ros/log/x/launch.log') == 'seed: log'
