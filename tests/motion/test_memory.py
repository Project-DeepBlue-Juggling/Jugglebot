"""``motion/skills/memory`` schema v2: Experience, Memory, paths, load rules."""

from __future__ import annotations

import logging
import os

import numpy as np
import pytest

from jugglebot.motion.skills import memory as mem


def _exp(i: int, **over) -> mem.Experience:
    kw = dict(
        site_xy_m=[0.01 * i, -0.02 * i], release_m=[0.01 * i, -0.02 * i, 0.86],
        release_vel_mps=[0.1 * i, 0.0, 2.0 + 0.01 * i], dwell_s=0.3 + 0.01 * i,
        seat_offset_m=[0.001 * i, 0.0], u=[0.0, 0.0, 0.85 + 0.001 * i],
        y=[0.001 * i, -0.001 * i, 0.85 + 0.0011 * i],
        t_abs_s=1_700_000_000.0 + i, ball_id=i % 2,
        release_kind='carried' if i % 2 else 'free_cup', caught=(i % 3 != 0),
        migrated=(i % 4 == 0))
    kw.update(over)
    return mem.Experience(**kw)


def _same(a: mem.Experience, b: mem.Experience) -> None:
    for f in ('site_xy_m', 'release_m', 'release_vel_mps', 'seat_offset_m',
              'u', 'y'):
        np.testing.assert_array_equal(getattr(a, f), getattr(b, f))
    assert np.array_equal(a.dwell_s, b.dwell_s, equal_nan=True)
    assert (a.t_abs_s, a.ball_id, a.release_kind, a.caught, a.migrated) == \
        (b.t_abs_s, b.ball_id, b.release_kind, b.caught, b.migrated)


def _line(exp) -> str:
    return ','.join(mem.format_row(exp))


def test_header_is_pinned():
    assert mem.SCHEMA_VERSION == 2
    assert ','.join(mem.HEADER) == (
        'site_x,site_y,rel_x,rel_y,rel_z,vel_x,vel_y,vel_z,dwell_s,seat_x,'
        'seat_y,u0,u1,u2_apex_m,y0,y1,y2_apex_m,t_abs_s,ball_id,'
        'release_kind,caught,migrated')
    assert ','.join(mem.HEADER_V1) == (
        'x0,x1,x2,x3,u0,u1,u2_apex_m,y0,y1,y2_apex_m,t_abs_s,ball_id,caught')


def test_paths_and_plant_id_rule():
    assert mem.memory_path('/root', 'P1') == '/root/temp/learn/P1/memory_v2.csv'
    assert mem.legacy_memory_path('/root', 'P1') == '/root/temp/learn/P1/memory.csv'
    for bad in ['', '..', '.', 'a/b', 'a/../b']:
        with pytest.raises(ValueError, match='plant_id'):
            mem.memory_path('/root', bad)
        with pytest.raises(ValueError, match='plant_id'):
            mem.legacy_memory_path('/root', bad)


@pytest.mark.parametrize('field,bad', [
    ('site_xy_m', [0.0]), ('release_m', [0.0, 0.0]),
    ('release_vel_mps', [0.0, 0.0, float('nan')]),
    ('seat_offset_m', [float('inf'), 0.0]), ('u', [0.0, float('nan'), 0.9]),
    ('y', [0.0, 0.0, float('inf')]), ('t_abs_s', float('nan')),
    ('dwell_s', float('inf')), ('release_kind', 'tossed'),
])
def test_experience_rejects_bad_fields_by_name(field, bad):
    with pytest.raises(ValueError, match=field):
        _exp(1, **{field: bad})


def test_experience_allows_nan_dwell():
    assert np.isnan(_exp(1, dwell_s=float('nan')).dwell_s)


def test_missing_files_are_an_empty_memory(tmp_path):
    m = mem.Memory(mem.memory_path(str(tmp_path), 'new'))
    assert len(m) == 0 and m.n_dropped == 0 and m.rows() == []
    assert m.ball_ids.shape == (0,) and m.release_kinds.shape == (0,)


def test_append_reload_round_trips_bitwise_in_file_order(tmp_path):
    path = mem.memory_path(str(tmp_path), 'plant1')
    m = mem.Memory(path)
    exps = [_exp(i) for i in range(6)]
    exps[2] = _exp(2, t_abs_s=1789263419.5 + 0.123456789,
                   dwell_s=float('nan'))
    for e in exps:
        m.append(e)
    r = mem.Memory(path)
    assert len(r) == 6 and r.n_dropped == 0
    for a, b in zip(exps, r.rows()):
        _same(a, b)
    np.testing.assert_array_equal(r.ball_ids, [e.ball_id for e in exps])
    assert list(r.release_kinds) == [e.release_kind for e in exps]
    U, Y = r.arrays()
    np.testing.assert_array_equal(U[:, 2], [e.u[2] for e in exps])
    with open(path) as fh:
        assert fh.readline().strip().split(',') == mem.HEADER


def test_malformed_rows_are_dropped_counted_and_logged(tmp_path, caplog):
    path = tmp_path / 'p' / 'memory_v2.csv'
    path.parent.mkdir()
    nan_row = _line(_exp(2)).replace(mem._FLOAT_FMT % _exp(2).site_xy_m[0],
                                     'nan', 1)
    bad_kind = _line(_exp(3)).replace('carried', 'tossed')
    text_row = ','.join(['abc'] * len(mem.HEADER))
    path.write_text('\n'.join([','.join(mem.HEADER), _line(_exp(0)), '1,2,3',
                               nan_row, bad_kind, text_row, _line(_exp(1))])
                    + '\n')
    caplog.set_level(logging.WARNING)
    m = mem.Memory(str(path))
    assert len(m) == 2 and m.n_dropped == 4
    _same(m.rows()[0], _exp(0))
    _same(m.rows()[1], _exp(1))
    msgs = [r.getMessage() for r in caplog.records]
    for ln in (3, 4, 5, 6):
        assert any('line %d' % ln in s for s in msgs)


@pytest.mark.parametrize('u_apex,y_apex,want', [
    (0.9, 0.9, True), (0.9, 1.38, True), (0.9, 0.9 * 2.605 ** 2, False),
    (0.9, 0.9 * 0.2, False), (0.9, 2.56 * 0.9, True), (0.9, 0.25 * 0.9, True)])
def test_apex_in_band(u_apex, y_apex, want):
    assert mem.apex_in_band(u_apex, y_apex) is want


def test_apex_band_and_no_commanded_apex():
    assert mem.APEX_RATIO_BAND == pytest.approx((0.25, 2.56))
    assert mem.apex_in_band(0.0, 5.0) is True
    assert mem.apex_in_band(float('nan'), 5.0) is True
    assert mem.apex_in_band(0.9, float('nan')) is False


def test_append_refuses_and_load_drops_a_row_outside_the_band(tmp_path):
    path = str(tmp_path / 'memory_v2.csv')
    m = mem.Memory(path)
    with pytest.raises(ValueError, match='another flight'):
        m.append(_exp(1, u=[0, 0, 0.8569], y=[0, 0, 2.2317]))
    assert len(m) == 0
    m.append(_exp(1, u=[0, 0, 0.8569], y=[0, 0, 1.0597]))
    with open(path, 'a') as fh:
        fh.write(_line(_exp(2, u=[0, 0, 0.9], y=[0, 0, 6.1])) + '\n')
    r = mem.Memory(path)
    assert len(r) == 1 and r.n_dropped == 1


# ── T-U6: load rules ───────────────────────────────────────────────────────

def _write_v1(path, header=None):
    os.makedirs(os.path.dirname(str(path)), exist_ok=True)
    with open(str(path), 'w') as fh:
        fh.write(','.join(header or mem.HEADER_V1) + '\n')
        fh.write('-0.05,0,0,0,0,0,0.85,0,0,0.9,1789540417.3,0,True\n')


def test_v1_file_at_the_v2_path_is_refused_by_name(tmp_path):
    p = tmp_path / 'memory_v2.csv'
    _write_v1(p)
    with pytest.raises(ValueError, match='schema-v1'):
        mem.Memory(str(p))


def test_v1_beside_a_missing_v2_is_refused_with_the_command(tmp_path):
    v1 = tmp_path / 'plant' / 'memory.csv'
    _write_v1(v1)
    v2 = tmp_path / 'plant' / 'memory_v2.csv'
    with pytest.raises(ValueError) as ei:
        mem.Memory(str(v2))
    msg = str(ei.value)
    assert 'tools/migrate_memory_schema_v2.py' in msg
    assert '--memory %s' % v1 in msg and '--out %s' % v2 in msg
    assert not v2.exists()


def test_v2_present_beside_v1_loads(tmp_path):
    _write_v1(tmp_path / 'memory.csv')
    (tmp_path / 'memory_v2.csv').write_text(','.join(mem.HEADER) + '\n')
    m = mem.Memory(str(tmp_path / 'memory_v2.csv'))
    m.append(_exp(1))
    assert len(mem.Memory(str(tmp_path / 'memory_v2.csv'))) == 1


def test_legacy_flight_time_header_is_still_refused(tmp_path):
    p = tmp_path / 'memory_v2.csv'
    _write_v1(p, header=mem.LEGACY_FLIGHT_HEADER)
    with pytest.raises(ValueError, match='quarantine'):
        mem.Memory(str(p))


def test_unknown_header_is_refused(tmp_path):
    p = tmp_path / 'memory_v2.csv'
    p.write_text('a,b,c\n')
    with pytest.raises(ValueError, match='header'):
        mem.Memory(str(p))


def test_append_refuses_the_schema_v1_file_name(tmp_path):
    """A Memory opened at an ABSENT ``memory.csv`` (the schema-v1 name) loads
    empty, and must not then create a v2 file under the v1 name (audit
    2026-11-06): the node always opens ``memory_path`` (``memory_v2.csv``), so
    this is defence in depth for a tool or a test that passes the old path."""
    m = mem.Memory(str(tmp_path / "memory.csv"))
    with pytest.raises(ValueError, match='schema-v1 file name'):
        m.append(_exp(0))
    assert not (tmp_path / 'memory.csv').exists() and len(m) == 0
