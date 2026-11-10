"""The records ledger (``jugglebot/records.py``, IDEA-3): the file format
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
    # Two-ball Patterns last (columns and its variants, e.g. columns_highlow).
    assert (2, 'columns') in order and order[-1][0] == 2
    assert (2, 'columns_highlow') in order
    assert (2, 'columns_turned') in order
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
    assert '| 2026-10-22 09:15 | 7 | 5 | compute (LIMIT_JERK) | a / note split |  |' in text
    assert '| 2026-10-21 11:30 | 10 | 9 | stopped (STOPPED) |  |  |' in text


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
    (ex.ABORTED_MODE_CHANGED, 'other'), (ex.ABORTED_FAULT, 'other'), (ex.LANE_REFUSED, 'other'),
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
    # new file: headings only, plus the drops section FEAT-47 starts on the first write
    assert rec.parse(Path(path).read_text()) == rec.apply_drops(rec.empty(), '2026-10-20 10:00', 0)
    assert rec.record_attempt(path, _a('2026-10-20 10:01', 'hop', 3)) == (True, 3)
    before = os.stat(path).st_mtime_ns, Path(path).read_bytes()
    assert rec.record_attempt(path, _a('2026-10-20 10:02', 'hop', 2)) == (False, 3)
    assert (os.stat(path).st_mtime_ns, Path(path).read_bytes()) == before
    assert os.listdir(str(tmp_path)) == ['records.md']          # no temp file left


@pytest.mark.parametrize('damage', [
    lambda t: t.replace('<!-- jugglebot-records v2 -->', ''),
    lambda t: t.replace('| 3 | compute', '| x | compute'),
    lambda t: t.replace('compute (SPLICE_TOO_LATE)', 'SPLICE_TOO_LATE'),
    lambda t: t.replace('balls: 2', 'balls: 1'),
    lambda t: t + '\n| 2026-10-20 10:00 | 1 | 1 | completed (COMPLETED) |  |  |\n',
    lambda t: t + '\n<!-- pattern: hop balls: 1 -->\n### Hop\n\n' + rec._TABLE_HEAD + '\n' + rec._TABLE_RULE + '\n| 2026-10-20 10:00 | 1 | 1 | completed (COMPLETED) |  | no_equals |\n',
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
    col = next(p for p in j['patterns'] if p['pattern'] == 'columns')
    assert col['balls'] == 2 and col['history'] == 1
    assert col['record'] == {'when_local': '2026-10-21 10:00', 'throws': 10,
                             'catches': 8, 'end_code': 'MISSED_CATCH',
                             'category': 'drop', 'note': '', 'params': {}}
    assert j['patterns'][0]['record'] is None


def test_seed_note_marks_reconstructed_and_fed_rows():
    """Seeded rows keep their provenance in the Note column: a per-throw
    reconstruction (pre-2026-09-29 logs, no summary line) is flagged as
    approximate, a fed goal is named, anything else is a plain log row."""
    from jugglebot.records import seed_note
    assert 'approximate' in seed_note('~/.ros/log/x/launch.log [per-throw OUTCOME lines counted; no summary line]')
    assert seed_note('~/.ros/log/x/launch.log [reload]') == 'seed: fed goal'
    assert seed_note('~/.ros/log/x/launch.log') == 'seed: log'


# ── Ledger v2: the Parameters column (FEAT-29) ───────────────────────────────

_PARAMS = (('apex_m', '0.95'), ('separation_mm', '125'), ('dwell_s', '0.27'),
           ('planner', 'pose'))


def test_v2_round_trips_params_in_insertion_order():
    model, beaten = rec.apply_attempt(rec.empty(), rec.Attempt(
        '2026-11-01 10:00', 'columns', 9, 8, 'STOPPED', params=_PARAMS))
    assert beaten
    text = rec.render(model)
    assert '<!-- jugglebot-records v2 -->' in text
    assert ('| 2026-11-01 10:00 | 9 | 8 | stopped (STOPPED) |  | '
            'apex_m=0.95; separation_mm=125; dwell_s=0.27; planner=pose |' in text)
    again = rec.parse(text)
    assert again.table('columns').rows[0].params == _PARAMS
    assert rec.render(again) == text


def test_a_v1_file_parses_with_empty_params_and_rerenders_as_v2():
    v2 = rec.render(rec.rebuild([
        _a('2026-10-20 10:00', 'columns', 3, code='MISSED_CATCH'),
        rec.Attempt('2026-10-21 10:00', 'columns', 9, 8, 'STOPPED', note='keep me')]))
    v1 = (v2.replace('jugglebot-records v2', 'jugglebot-records v1')
            .replace(rec._TABLE_HEAD, rec._TABLE_HEAD_V1)
            .replace(rec._TABLE_RULE, rec._TABLE_RULE_V1)
            .replace(' |  |\n', ' |\n').replace(' | keep me |  |\n', ' | keep me |\n'))
    assert '| Parameters |' not in v1
    model = rec.parse(v1)
    rows = model.table('columns').rows
    assert [r.params for r in rows] == [(), ()]
    assert [r.note for r in rows] == ['', 'keep me']
    assert rec.render(model) == v2


def test_v1_file_is_migrated_by_the_next_write(tmp_path):
    path = tmp_path / 'records.md'
    v2 = rec.render(rec.rebuild([_a('2026-10-20 10:00', 'hop', 3)]))
    path.write_text(v2.replace('v2 -->', 'v1 -->')
                    .replace(rec._TABLE_HEAD, rec._TABLE_HEAD_V1)
                    .replace(rec._TABLE_RULE, rec._TABLE_RULE_V1)
                    .replace(' |  |\n', ' |\n'))
    assert rec.record_attempt(str(path), rec.Attempt(
        '2026-10-21 10:00', 'hop', 9, 8, 'COMPLETED', params=_PARAMS)) == (True, 8)
    text = path.read_text()
    assert 'jugglebot-records v2' in text and 'v1' not in text
    rows = rec.load(str(path)).table('hop').rows
    assert [r.params for r in rows] == [(), _PARAMS]


def test_params_with_forbidden_characters_are_sanitised_and_still_parse():
    model, _ = rec.apply_attempt(rec.empty(), rec.Attempt(
        '2026-11-01 10:00', 'hop', 5, 4, 'COMPLETED',
        params=(('a|b', 'x;y=z'), ('k', 'two  words'), ('', 'dropped'))))
    text = rec.render(model)
    row = rec.parse(text).table('hop').rows[0]
    assert row.params == (('a/b', 'x/y/z'), ('k', 'two words'))
    assert model.table('hop').rows[0].params == row.params
    assert rec.render(rec.parse(text)) == text


def test_a_bad_parameters_cell_is_a_format_error():
    text = rec.render(rec.rebuild([_a('2026-10-20 10:00', 'hop', 3)]))
    with pytest.raises(rec.RecordsFormatError):
        rec.parse(text.replace('COMPLETED) |  |  |', 'COMPLETED) |  | nokey |'))


def test_to_json_carries_params_as_an_ordered_dict():
    model, _ = rec.apply_attempt(rec.empty(), rec.Attempt(
        '2026-11-01 10:00', 'columns', 9, 8, 'STOPPED', params=_PARAMS))
    col = next(p for p in rec.to_json(model)['patterns'] if p['pattern'] == 'columns')
    assert list(col['record']['params'].items()) == list(_PARAMS)


# ── Drops to date (FEAT-47) ───────────────────────────────────────────────

def _ad(when, pattern, catches, drops, throws=None):
    return rec.Attempt(when, pattern, catches + 1 if throws is None else throws,
                       catches, 'COMPLETED', drops=drops)


def _with_history(model, hist):
    import dataclasses
    return dataclasses.replace(model, drops=dataclasses.replace(model.drops, history=hist))


def test_a_file_without_the_drops_section_is_not_counted_yet():
    text = rec.render(rec.empty())
    assert 'drops:' not in text
    assert rec.parse(text).drops is None
    assert rec.to_json(rec.parse(text))['drops'] is None


def test_drops_section_round_trips_byte_identically():
    for hist in (None, 40):
        model = _with_history(rec.apply_drops(rec.empty(), '2026-11-10 09:00', 3), hist)
        text = rec.render(model)
        assert '<!-- drops: live=3 since=2026-11-10 history=%s -->' % (
            'none' if hist is None else hist) in text
        assert text.index('drops:') < text.index('<!-- pattern:')   # near the top
        assert rec.parse(text) == model
        assert rec.render(rec.parse(text)) == text


def test_total_is_history_plus_live_and_to_json_carries_it():
    model = _with_history(rec.apply_drops(rec.empty(), '2026-11-10 09:00', 5), 100)
    assert rec.to_json(model)['drops'] == {'total': 105, 'live': 5,
                                           'since': '2026-11-10', 'history': 100}
    none = rec.apply_drops(rec.empty(), '2026-11-10 09:00', 5)
    assert rec.to_json(none)['drops']['total'] == 5
    assert rec.to_json(none)['drops']['history'] is None


def test_the_section_and_tables_survive_each_other_through_a_record():
    model = rec.apply_drops(rec.empty(), '2026-11-10 09:00', 2)
    model, beaten = rec.apply_attempt(model, _a('2026-11-10 09:05', 'hop', 4))
    assert beaten and model.drops.live == 2
    assert rec.parse(rec.render(model)) == model


def test_record_attempt_writes_when_drops_even_if_no_record_is_beaten(tmp_path):
    path = str(tmp_path / 'records.md')
    assert rec.record_attempt(path, _ad('2026-11-10 10:00', 'hop', 3, 0)) == (True, 3)
    assert rec.load(path).drops == rec.Drops(0, '2026-11-10', None)
    before = (os.stat(path).st_mtime_ns, Path(path).read_bytes())   # no drops, no record: no write
    assert rec.record_attempt(path, _ad('2026-11-10 10:01', 'hop', 2, 0)) == (False, 3)
    assert (os.stat(path).st_mtime_ns, Path(path).read_bytes()) == before
    assert rec.record_attempt(path, _ad('2026-11-11 10:02', 'hop', 2, 2)) == (False, 3)
    assert rec.load(path).drops == rec.Drops(2, '2026-11-10', None)
    assert rec.record_attempt(path, _ad('2026-11-11 10:03', 'hop', 1, 1)) == (False, 3)
    assert rec.total_drops(path) == 3
    assert os.listdir(str(tmp_path)) == ['records.md']


def test_the_first_write_migrates_a_v2_file_without_the_section(tmp_path):
    path = tmp_path / 'records.md'
    path.write_text(rec.render(rec.rebuild([_a('2026-10-20 10:00', 'hop', 4)])))
    assert rec.record_attempt(str(path), _ad('2026-11-10 10:00', 'hop', 1, 1)) == (False, 4)
    model = rec.load(str(path))
    assert model.drops == rec.Drops(1, '2026-11-10', None)
    assert model.table('hop').best.catches == 4


def test_a_feed_books_its_drops_but_never_a_record_row(tmp_path):
    path = str(tmp_path / 'records.md')
    rec.record_attempt(path, _ad('2026-11-10 10:00', 'hop', 1, 0))
    before = rec.load(path)
    assert rec.record_attempt(path, _ad('2026-11-10 10:04', 'reload', 5, 0)) == (False, 0)
    assert rec.load(path) == before                # a Feed with no drops writes nothing
    assert rec.record_attempt(path, _ad('2026-11-10 10:05', 'reload', 5, 2)) == (False, 0)
    after = rec.load(path)
    assert after.drops.live == 2
    assert after.tables == before.tables
    assert 'pattern: reload' not in Path(path).read_text()


def test_a_malformed_drops_marker_is_never_overwritten(tmp_path):
    good = rec.render(rec.apply_drops(rec.empty(), '2026-11-10 10:00', 1))
    dup = '<!-- drops: live=1 since=2026-11-10 history=none -->'
    for bad in (good.replace('live=1', 'live=x'),
                good.replace('history=none', 'history=-3'),
                good.replace(dup, dup + '\n' + dup)):
        path = tmp_path / 'records.md'
        path.write_text(bad)
        with pytest.raises(rec.RecordsFormatError):
            rec.record_attempt(str(path), _ad('2026-11-11 10:00', 'hop', 1, 1))
        assert path.read_text() == bad


def test_skill_node_passes_the_drop_count_and_feeds_reach_the_ledger():
    src = SKILL_NODE.read_text()
    assert 'rp.drop_count(self._goal_reports)' in src
    assert "0 if pattern == 'magazine' else" in src   # a magazine Feed books only Feed misses
    body = src.split('def _offer_record')[1].split('def _juggle_execute')[0]
    assert 'FEED_PATTERNS' not in body        # a Feed books drops too
