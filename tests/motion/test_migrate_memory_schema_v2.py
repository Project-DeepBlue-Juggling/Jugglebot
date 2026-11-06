"""``tools/migrate_memory_schema_v2.py``: v1 memory -> v2."""

from __future__ import annotations

import csv
import datetime as dt
import importlib.util
import math
import os
import stat
import sys

import numpy as np
import pytest

from jugglebot.motion.skills import memory as mem
from jugglebot.motion.skills import schedule as sch
from jugglebot.motion.skills import sites
from jugglebot.motion.trajectory import ballistics_bc as bbc

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SCRIPT = os.path.join(_ROOT, 'tools', 'migrate_memory_schema_v2.py')
_CORPUS = os.path.join(_ROOT, 'temp', 'learn', 'jugglebot', 'memory.csv')


@pytest.fixture(scope='module')
def tool():
    spec = importlib.util.spec_from_file_location('migrate_memory_schema_v2', _SCRIPT)
    m = importlib.util.module_from_spec(spec)
    sys.modules['migrate_memory_schema_v2'] = m
    spec.loader.exec_module(m)
    return m


def _v1(path, rows, header=None):
    with open(str(path), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(header or mem.HEADER_V1)
        w.writerows(rows)


#                x0     x1 x2    x3  u0 u1 u2     y0   y1   y2    t           ball caught
_CARRIED = ['-0.075', '0', '0.002', '0.001', '0', '0', '0.9', '0.01', '0.02', '1.0', '1790000001.5', '1', 'True']
_HOP = ['0.125', '0', '0', '0', '0', '0', '0.85', '0.0', '0.0', '0.9', '1790000002.5', '0', 'False']
_VERT = ['-0.05', '0.01', '0', '0', '0', '0', '0.8', '0.0', '0.0', '0.85', '1790000003.5', '0', 'True']


def _expect_vel(site, seat, target_x, target_y, apex):
    rel = np.array([(site[0] + seat[0]) * 1000, (site[1] + seat[1]) * 1000,
                    sites.RELEASE_CUP_Z_MM])
    tgt = np.array([target_x * 1000, target_y * 1000, sites.RELEASE_CUP_Z_MM])
    return rel / 1000.0, bbc.launch_velocity(rel, tgt, sch.flight_s(apex)) / 1000.0


def test_one_row_of_each_class(tool, tmp_path):
    src, out = tmp_path / 'memory.csv', tmp_path / 'memory_v2.csv'
    _v1(src, [_CARRIED, _HOP, _VERT])
    assert tool.main(['--memory', str(src), '--out', str(out)]) == 0
    m = mem.Memory(str(out))
    c, h, v = m.rows()
    # a carried row releases AT its Site exactly: the seat offset (the catch
    # offset at the other Site) is kept on the row but is not in the release
    rel, vel = _expect_vel((-0.075, 0), (0, 0), 0.075, 0.0, 0.9)
    np.testing.assert_allclose(c.release_m, rel, atol=1e-12)
    np.testing.assert_allclose(c.seat_offset_m, (0.002, 0.001), atol=1e-12)
    np.testing.assert_allclose(c.release_vel_mps, vel, atol=1e-12)
    assert c.release_kind == 'carried' and c.migrated and np.isnan(c.dwell_s)
    rel, vel = _expect_vel((0.125, 0), (0, 0), -0.125, 0.0, 0.85)
    np.testing.assert_allclose(h.release_vel_mps, vel, atol=1e-12)
    np.testing.assert_allclose(h.release_m, rel, atol=1e-12)
    assert h.release_kind == 'free_cup' and h.migrated and not h.caught
    rel, vel = _expect_vel((-0.05, 0.01), (0, 0), -0.05, 0.01, 0.8)
    np.testing.assert_allclose(v.release_vel_mps, vel, atol=1e-12)
    assert v.release_vel_mps[0] == 0.0 and v.release_vel_mps[1] == 0.0
    assert v.release_vel_mps[2] == pytest.approx(0.5 * 9.80665 * sch.flight_s(0.8), rel=1e-3)
    assert v.release_kind == 'free_cup'
    np.testing.assert_array_equal(c.site_xy_m, [-0.075, 0.0])
    np.testing.assert_array_equal(c.seat_offset_m, [0.002, 0.001])
    assert (c.ball_id, c.t_abs_s) == (1, 1790000001.5)


def test_idempotent_and_refuses_a_different_output(tool, tmp_path, capsys):
    src, out = tmp_path / 'memory.csv', tmp_path / 'memory_v2.csv'
    _v1(src, [_CARRIED, _HOP, _VERT])
    assert tool.main(['--memory', str(src), '--out', str(out)]) == 0
    first = out.read_bytes()
    capsys.readouterr()
    assert tool.main(['--memory', str(src), '--out', str(out)]) == 0
    assert out.read_bytes() == first
    assert 'byte-identical' in capsys.readouterr().out
    out.write_bytes(first + b'extra\n')
    assert tool.main(['--memory', str(src), '--out', str(out)]) == 1
    assert out.read_bytes() == first + b'extra\n'


def test_never_writes_its_input(tool, tmp_path):
    src, out = tmp_path / 'memory.csv', tmp_path / 'memory_v2.csv'
    _v1(src, [_CARRIED, _VERT])
    before = src.read_bytes()
    os.chmod(str(src), stat.S_IRUSR)
    try:
        assert tool.main(['--memory', str(src), '--out', str(out)]) == 0
    finally:
        os.chmod(str(src), stat.S_IRUSR | stat.S_IWUSR)
    assert src.read_bytes() == before
    assert tool.main(['--memory', str(src), '--out', str(src)]) == 1
    assert src.read_bytes() == before


def test_dry_run_writes_nothing(tool, tmp_path, capsys):
    src, out = tmp_path / 'memory.csv', tmp_path / 'sub' / 'memory_v2.csv'
    _v1(src, [_CARRIED])
    assert tool.main(['--memory', str(src), '--out', str(out), '--dry-run']) == 0
    assert not out.exists() and not out.parent.exists()
    assert 'would write 1 of 1' in capsys.readouterr().out


def test_rows_the_v1_loader_would_drop_are_dropped_and_counted(tool, tmp_path, capsys):
    src, out = tmp_path / 'memory.csv', tmp_path / 'memory_v2.csv'
    bad_band = list(_VERT); bad_band[9] = '9.0'          # apex far outside band
    nan_row = list(_VERT); nan_row[0] = 'nan'
    no_flight = list(_VERT); no_flight[6] = '0'; no_flight[9] = '0.5'
    _v1(src, [_VERT, ['1', '2'], bad_band, nan_row, ['abc'] * 13, no_flight, _HOP])
    assert tool.main(['--memory', str(src), '--out', str(out)]) == 0
    text = capsys.readouterr().out
    assert 'wrote 2 of 7 rows read (5 dropped' in text and '1 have no flight time' in text
    assert len(mem.Memory(str(out))) == 2


def test_refuses_a_non_v1_header(tool, tmp_path):
    src = tmp_path / 'memory.csv'
    _v1(src, [], header=mem.HEADER)
    assert tool.main(['--memory', str(src), '--out', str(tmp_path / 'o.csv')]) == 1


#: 2026-10-23 00:00 UTC+11 (AEDT): the day the oval's rows begin.
_OVAL_EPOCH = dt.datetime(2026, 10, 23, tzinfo=dt.timezone(dt.timedelta(hours=11))).timestamp()


@pytest.mark.skipif(not os.path.exists(_CORPUS), reason='committed corpus not present')
def test_committed_corpus(tool, tmp_path):
    out = tmp_path / 'memory_v2.csv'
    assert tool.main(['--memory', _CORPUS, '--out', str(out)]) == 0
    with open(_CORPUS, newline='') as fh:
        v1 = list(csv.reader(fh))[1:]
    with open(str(out), newline='') as fh:
        r = csv.reader(fh)
        assert next(r) == mem.HEADER
        v2 = list(r)
    # 3276 data rows, the v1 loader's own drops = 0 -> 3276 written.
    assert len(v1) == 3276 and len(v2) == 3276
    ix = {n: i for i, n in enumerate(mem.HEADER)}
    carried = [i for i, row in enumerate(v2) if row[ix['release_kind']] == 'carried']
    assert len(carried) == 977
    for i in carried:
        assert float(v2[i][ix['t_abs_s']]) >= _OVAL_EPOCH
        assert abs(float(v1[i][0]) + 0.075) < 1e-9
    assert sum(1 for row in v1 if abs(abs(float(row[0])) - 0.125) < 1e-9) == 175
    for a, b in zip(v1, v2):
        for c1, c2 in zip(['u0', 'u1', 'u2_apex_m', 'y0', 'y1', 'y2_apex_m'],
                          ['u0', 'u1', 'u2_apex_m', 'y0', 'y1', 'y2_apex_m']):
            assert b[ix[c2]] == '%.17g' % float(a[mem.HEADER_V1.index(c1)])
    assert len(mem.Memory(str(out))) == 3276
