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
                                           'since': '2026-11-10', 'history': 100,
                                           'history_from': None,
                                           'history_through': None,
                                           'caught': None, 'throws': None}
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
    assert 'rp.drop_count(self._goal_reports,' in src
    assert 'feed_missed=feed_missed' in src          # Feed misses are counted (FEAT-47)
    assert 'missed_catches=missed_catches' in src    # so are Throws lost to an EMPTY Cup
    assert 'rp.caught_count(' in src                 # the caught side shares drop_count's base
    assert 'ball_caught=ball_caught' in src
    assert "ball_caught = (0 if pattern == 'magazine'" in src
    assert "drops = (0 if pattern == 'magazine'" in src   # a magazine run books ZERO drops (owner, TKT-64)
    body = src.split('def _offer_record')[1].split('def _juggle_execute')[0]
    assert 'FEED_PATTERNS' not in body        # a Feed books drops too


# ── Drops history backfill (FEAT-47 slice 4) ──────────────────────────────

HIST = dict(history=828, through='2026-11-11', first='2026-07-23')


def test_set_history_creates_the_section_when_absent(tmp_path):
    legacy = str(tmp_path / 'legacy.md')
    Path(legacy).write_text(rec.render(rec.empty()))   # pre-upgrade shape
    assert rec.load(legacy).drops is None
    rec.set_drops_history(legacy, **HIST)
    out = Path(legacy).read_text()
    assert ('<!-- drops: live=0 since=2026-11-11 history=828 '
            'history_from=2026-07-23 history_through=2026-11-11 -->') in out
    assert ('Drops to date: 828 (0 counted live since 2026-11-11; '
            '828 from history, 2026-07-23 to 2026-11-11).') in out
    assert rec.parse(out).drops.total == 828
    rec.set_drops_history(str(tmp_path / 'b.md'), since='2026-11-12', **HIST)
    assert rec.load(str(tmp_path / 'b.md')).drops.since == '2026-11-12'


def test_set_history_on_a_present_section_leaves_live_and_since(tmp_path):
    path = str(tmp_path / 'records.md')
    rec.record_attempt(path, _ad('2026-11-12 10:00', 'hop', 1, 12))
    rec.set_drops_history(path, **HIST)
    d = rec.load(path).drops
    assert (d.live, d.since, d.history) == (12, '2026-11-12', 828)
    assert d.total == d.history + d.live == 840
    assert 'Drops to date: 840 (12 counted live since 2026-11-12; 828 from history, 2026-07-23 to 2026-11-11)' \
        in Path(path).read_text()
    assert rec.to_json(rec.load(path))['drops'] == {
        'total': 840, 'live': 12, 'since': '2026-11-12', 'history': 828,
        'history_from': '2026-07-23', 'history_through': '2026-11-11',
        'caught': None, 'throws': None}


def test_a_live_write_after_a_history_write_keeps_history_and_dates(tmp_path):
    path = str(tmp_path / 'records.md')
    rec.record_attempt(path, _ad('2026-11-12 10:00', 'hop', 1, 1))
    rec.set_drops_history(path, **HIST)
    rec.record_attempt(path, _ad('2026-11-13 10:00', 'hop', 2, 3))
    d = rec.load(path).drops
    assert d == rec.Drops(4, '2026-11-12', 828, '2026-07-23', '2026-11-11')


def test_an_old_marker_without_the_new_fields_parses_and_round_trips():
    model = _with_history(rec.apply_drops(rec.empty(), '2026-11-10 09:00', 3), 100)
    text = rec.render(model)
    assert 'history_from' not in text and 'history=100 -->' in text
    d = rec.parse(text).drops
    assert d == rec.Drops(3, '2026-11-10', 100) and d.history_from is None
    assert rec.render(rec.parse(text)) == text


@pytest.mark.parametrize('kw', [
    dict(history=-1, through='2026-11-11', first='2026-07-23'),
    dict(history=5, through='2026-13-40', first='2026-07-23'),
    dict(history=5, through='2026-11-11', first='yesterday'),
    dict(history=5, through='2026-07-01', first='2026-07-23'),
    dict(history=5, through='2026-11-11', first='2026-07-23', since='11/12'),
])
def test_bad_figure_or_date_is_refused_and_the_file_untouched(tmp_path, kw):
    path = tmp_path / 'records.md'
    rec.record_attempt(str(path), _ad('2026-11-12 10:00', 'hop', 1, 2))
    before = path.read_bytes()
    with pytest.raises(ValueError):
        rec.set_drops_history(str(path), **kw)
    assert path.read_bytes() == before


def test_a_malformed_existing_marker_is_refused_and_the_file_untouched(tmp_path):
    path = tmp_path / 'records.md'
    good = rec.render(rec.apply_drops(rec.empty(), '2026-11-10 10:00', 1))
    bad = good.replace('history=none', 'history=none history_from=2026-01-01')
    path.write_text(bad)
    with pytest.raises(ValueError):
        rec.set_drops_history(str(path), **HIST)
    assert path.read_text() == bad


def test_the_tools_script_writes_the_figure(tmp_path):
    import subprocess
    import sys
    path = tmp_path / 'records.md'
    cmd = [sys.executable, str(REPO / 'tools' / 'set_drops_history.py'),
           '--history', '828', '--from', '2026-07-23', '--through',
           '2026-11-11', '--path', str(path)]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    assert res.returncode == 0, res.stderr
    assert rec.load(str(path)).drops == rec.Drops(
        0, '2026-11-11', 828, '2026-07-23', '2026-11-11')
    bad = subprocess.run(cmd[:-4] + ['--through', 'nope', '--path', str(path)],
                         capture_output=True, text=True, timeout=60)
    assert bad.returncode == 1 and 'refused' in bad.stderr
    assert rec.load(str(path)).drops.history == 828


# ── Caught alongside drops: the Throws total and the percentage (owner, 2026-11-11) ──

OLD_MARKER = '<!-- drops: live=0 since=2026-11-11 history=819 history_from=2026-07-23 history_through=2026-11-10 -->'


def _ac(when, pattern, catches, drops, caught):
    return rec.Attempt(when, pattern, catches + 1, catches, 'COMPLETED', drops=drops, caught=caught)


def test_a_marker_without_caught_parses_and_rerenders_byte_identically():
    raw = rec.render(rec.apply_drops(rec.empty(), '2026-11-11 09:00', 0)).replace(
        'history=none -->', 'history=819 history_from=2026-07-23 history_through=2026-11-10 -->')
    text = rec.render(rec.parse(raw))      # the sentence is derived from the marker
    assert OLD_MARKER in text
    d = rec.parse(text).drops
    assert (d.caught, d.history_caught) == (None, None)
    assert rec.render(rec.parse(text)) == text
    assert 'Throws and Feeds to catch' not in text           # sentence unchanged until a caught figure is written
    # a drops-only apply_drops (caught=None) leaves the marker's shape alone
    assert 'caught=' not in rec.render(rec.apply_drops(rec.parse(text), '2026-11-11 10:00', 2))


def test_caught_fields_parse_render_and_survive_apply_drops():
    d = rec.Drops(3, '2026-11-11', 100, '2026-07-23', '2026-11-10', caught=7, history_caught=900)
    model = rec.apply_drops(rec.dataclasses.replace(rec.empty(), drops=d), '2026-11-12 08:00', 2, 5)
    text = rec.render(model)
    assert ('<!-- drops: live=5 since=2026-11-11 caught=12 history=100 history_caught=900 '
            'history_from=2026-07-23 history_through=2026-11-10 -->') in text
    assert rec.parse(text).drops == model.drops and rec.render(rec.parse(text)) == text
    again = rec.apply_drops(model, '2026-11-12 09:00', 0, 0)         # nothing new: every field preserved
    assert again.drops == model.drops


def test_totals_and_the_null_rule():
    d = rec.Drops(3, '2026-11-11', 100, caught=7, history_caught=900)
    assert (d.caught_total, d.throws_total, d.total) == (907, 1010, 103)
    assert rec.Drops(0, '2026-11-11', 819, history_caught=6485).throws_total == 7304   # live has no drops
    assert rec.Drops(0, '2026-11-11', None, caught=4).throws_total == 4                # no history at all
    assert rec.Drops(3, '2026-11-11', 100, history_caught=900).throws_total is None    # live drops, no live caught
    assert rec.Drops(3, '2026-11-11', 100, caught=7).throws_total is None              # history drops, no history caught
    assert rec.Drops(3, '2026-11-11', 100).caught_total is None
    j = rec.to_json(rec.dataclasses.replace(rec.empty(), drops=d))['drops']
    assert (j['caught'], j['throws'], j['total']) == (907, 1010, 103)
    assert rec.to_json(rec.dataclasses.replace(rec.empty(), drops=rec.Drops(3, 'x', 100)))['drops']['throws'] is None


def test_live_caught_starts_only_while_live_is_zero_and_never_misses_existing_drops():
    m = rec.apply_drops(rec.empty(), '2026-11-11 09:00', 0, 4)           # live 0, no figure: starts at 4
    assert m.drops.caught == 4
    old = rec.dataclasses.replace(rec.empty(), drops=rec.Drops(5, '2026-11-01', None))
    assert rec.apply_drops(old, '2026-11-11 09:00', 1, 4).drops.caught is None   # would miss the 5 old drops


def test_the_human_sentence_gains_throws_and_percent_only_when_complete(tmp_path):
    path = str(tmp_path / 'records.md')
    rec.set_drops_history(path, history=819, through='2026-11-10', first='2026-07-23',
                          since='2026-11-11', history_caught=6485)
    text = Path(path).read_text()
    assert ('<!-- drops: live=0 since=2026-11-11 history=819 history_caught=6485 '
            'history_from=2026-07-23 history_through=2026-11-10 -->') in text
    assert ('Drops to date: 819 (0 counted live since 2026-11-11; 819 from history, 2026-07-23 to 2026-11-10). '
            '7,304 Throws and Feeds to catch, 88.8 % caught, 11.2 % dropped. A drop is') in text
    rec.set_drops_history(path, history=819, through='2026-11-10', first='2026-07-23')   # without: cleared, sentence as before
    assert 'Throws and Feeds to catch' not in Path(path).read_text()
    assert rec.load(path).drops.history_caught is None


def test_set_history_refuses_a_negative_caught_and_leaves_the_file(tmp_path):
    path = tmp_path / 'records.md'
    rec.set_drops_history(str(path), history=5, through='2026-11-10')
    before = path.read_bytes()
    for bad in (-1, 2.5, True):
        with pytest.raises(ValueError):
            rec.set_drops_history(str(path), history=5, through='2026-11-10', history_caught=bad)
    assert path.read_bytes() == before


def test_the_cli_takes_history_caught_and_refuses_a_negative(tmp_path):
    import subprocess
    import sys
    path = tmp_path / 'records.md'
    cmd = [sys.executable, str(REPO / 'tools' / 'set_drops_history.py'), '--history', '819',
           '--from', '2026-07-23', '--through', '2026-11-10', '--path', str(path)]
    ok = subprocess.run(cmd + ['--history-caught', '6485'], capture_output=True, text=True, timeout=60)
    assert ok.returncode == 0, ok.stderr
    assert rec.load(str(path)).drops.throws_total == 7304
    before = path.read_bytes()
    bad = subprocess.run(cmd + ['--history-caught', '-1'], capture_output=True, text=True, timeout=60)
    assert bad.returncode == 1 and 'refused' in bad.stderr and path.read_bytes() == before


def test_a_catch_only_attempt_still_writes_the_live_caught(tmp_path):
    path = str(tmp_path / 'records.md')
    rec.record_attempt(path, _ac('2026-11-11 10:00', 'hop', 3, 0, 3))
    assert rec.load(path).drops.caught == 3
    assert rec.record_attempt(path, _ac('2026-11-11 10:01', 'hop', 2, 0, 2)) == (False, 3)   # no record, no drops
    assert rec.load(path).drops.caught == 5
    rec.record_attempt(path, _ac('2026-11-11 10:02', 'reload', 1, 0, 1))                     # a Feed with catches only
    assert rec.load(path).drops.caught == 6
    rec.record_attempt(path, _ac('2026-11-11 10:03', 'reload', 0, 0, 0))                     # nothing: no write
    assert rec.load(path).drops.caught == 6


# -- Landing scores (IDEA-21, owner's decision in TKT-63) --------------------

def _scored(when, pattern, catches, scores=(), no_fit=0, feeds=(), throws=None):
    return rec.Attempt(when, pattern, catches + 1 if throws is None else throws,
                       catches, 'COMPLETED', scores=tuple(scores),
                       no_fit=no_fit, feed_scores=tuple(feeds))


def test_scores_render_and_parse_round_trip_with_throws_per_pattern_and_feeds():
    m = rec.empty()
    m = rec.apply_scores(m, '2026-11-12 09:00', 'columns', [12.4, 14.6, 30.2], 2,
                         [8.0])
    m = rec.apply_scores(m, '2026-11-12 09:05', 'self_toss', [20.0])
    m = rec.apply_scores(m, '2026-11-12 09:10', 'columns', [5.0, 7.0])
    text = rec.render(m)
    assert '## Landing scores' in text
    assert '| 2026-11-12 09:00 | 3 | 15 | 2 | 12 15 30 |' in text
    assert '| 2026-11-12 09:10 | 2 | 6 | 0 | 5 7 |' in text
    assert '| 2026-11-12 09:00 | columns | 8 |' in text
    back = rec.parse(text)
    assert back == m and rec.render(back) == text
    file_order = [t.pattern for t in back.ordered()]
    idx = [file_order.index(r.pattern) for r in back.scores]
    assert idx == sorted(idx)


def test_a_ledger_with_no_scores_renders_unchanged_and_the_committed_file_still_parses():
    # The committed ledger is live data: a Sitting books scores into it, so
    # this asserts only that it parses and round-trips, never what it holds.
    text = (REPO / 'docs' / 'records.md').read_text()
    m = rec.parse(text)
    assert m.tables and rec.render(m) == text
    bare = rec.Ledger(tables=m.tables, drops=m.drops)
    assert not bare.scores and not bare.feed_scores
    assert 'Landing scores' not in rec.render(bare)
    assert rec.parse(rec.render(bare)) == bare
    assert rec.render(rec.parse(rec.render(rec.empty()))) == rec.render(rec.empty())


def test_scores_with_no_scored_throw_still_book_the_throws_without_a_fit():
    m = rec.apply_scores(rec.empty(), '2026-11-12 09:00', 'columns', [], 3)
    text = rec.render(m)
    assert '| 2026-11-12 09:00 | 0 | - | 3 | - |' in text
    assert rec.parse(text) == m


def test_a_feed_goal_books_feed_rows_never_a_throw_row_and_junk_scores_are_dropped():
    m = rec.apply_scores(rec.empty(), '2026-11-12 09:00', 'reload', [5.0], 1,
                         [10.0, float('nan'), -1.0, float('inf')])
    assert not m.scores
    assert [f.score_mm for f in m.feed_scores] == [10]
    assert rec.parse(rec.render(m)) == m
    with pytest.raises(ValueError):
        rec.apply_scores(m, 'yesterday', 'columns', [1.0])


@pytest.mark.parametrize('damage', [
    lambda t: t.replace('| 3 | 15 |', '| 4 | 15 |'),       # Scored != scores
    lambda t: t.replace('| 3 | 15 |', '| 3 | 16 |'),       # wrong median
    lambda t: t.replace('pattern=columns', 'pattern=nope'),
    lambda t: t.replace('| 2026-11-12 09:00 | columns | 8 |',
                        '| 2026-11-12 09:00 | columns | eight |'),
])
def test_a_damaged_scores_section_is_a_format_error(damage):
    m = rec.apply_scores(rec.empty(), '2026-11-12 09:00', 'columns',
                         [12.4, 14.6, 30.2], 2, [8.0])
    with pytest.raises(rec.RecordsFormatError):
        rec.parse(damage(rec.render(m)))


def test_record_attempt_writes_scores_whether_or_not_a_record_is_beaten(tmp_path):
    path = str(tmp_path / 'records.md')
    beaten, _b = rec.record_attempt(path, _scored(
        '2026-11-12 09:00', 'columns', 5, [10.0, 20.0], 1))
    assert beaten
    # 3 catches does not beat 5: the scores are still written
    beaten, best = rec.record_attempt(path, _scored(
        '2026-11-12 09:05', 'columns', 3, [30.0], 0, feeds=[7.0]))
    assert (beaten, best) == (False, 5)
    m = rec.load(path)
    assert [(r.when_local, r.scores, r.no_fit) for r in m.scores] == [
        ('2026-11-12 09:00', (10, 20), 1), ('2026-11-12 09:05', (30,), 0)]
    assert [f.score_mm for f in m.feed_scores] == [7]
    # a Feed-only goal with nothing but a Feed score writes the Feed row
    rec.record_attempt(path, _scored('2026-11-12 09:10', 'reload', 0, feeds=[4.0]))
    assert [f.score_mm for f in rec.load(path).feed_scores] == [7, 4]
    # no scores, nothing beaten: the file is untouched
    before = Path(path).read_text()
    rec.record_attempt(path, _a('2026-11-12 09:15', 'columns', 2))
    assert Path(path).read_text() == before


def test_replacing_a_table_keeps_the_scores():
    m = rec.apply_scores(rec.empty(), '2026-11-12 09:00', 'columns', [9.0])
    m, beaten = rec.apply_attempt(m, _a('2026-11-12 09:05', 'columns', 4))
    assert beaten and len(m.scores) == 1
