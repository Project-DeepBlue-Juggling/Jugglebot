"""``tools/migrate_memory_catch_plane.py`` — the catch-plane memory migration
(R5 sitting 6, 2026-10-05: CATCH HIGH, ``sites.CATCH_CUP_Z_MM`` 830 -> 930).

Offline and read-only except for the file this tool is explicitly given
(``tmp_path`` only, never a real ``temp/learn/`` tree). Pins: a dry run
writes nothing; a real run writes a timestamped backup and a sidecar marker
and touches only rows before ``--cutoff-epoch``; a second run against the
same file refuses outright; and a PERFECT-PLANT row (whose old ``y2_apex_m``
is exactly what the planner's own model would have produced for a known
commanded apex ``u`` at the old rise) round-trips back to ``u``.
"""

from __future__ import annotations

import csv
import importlib.util
import math
import os
import sys

import pytest

from jugglebot.motion.skills import schedule as sch
from jugglebot.motion.skills.memory import _FLOAT_FMT, _HEADER

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SCRIPT_PATH = os.path.join(_PROJECT_ROOT, 'tools', 'migrate_memory_catch_plane.py')


def _load_script():
    spec = importlib.util.spec_from_file_location(
        'migrate_memory_catch_plane', _SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    # The module is a dataclass-bearing script under `from __future__ import
    # annotations`: the dataclass decorator resolves its (string) field
    # annotations via `sys.modules[cls.__module__]`, so it must be
    # registered there BEFORE exec_module runs the class body.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def tool():
    return _load_script()


def _row(x=(0.0, 0.0, 0.0, 0.0), u=(0.0, 0.0, 0.9), y=(0.0, 0.0, 0.92),
         t_abs_s=1000.0, ball_id=0, caught=True):
    return (list(x) + list(u) + list(y)
            + [t_abs_s, ball_id, caught])


def _write_csv(path, rows):
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(_HEADER)
        for row in rows:
            formatted = ([_FLOAT_FMT % v for v in row[:11]]
                        + [str(row[11]), str(row[12])])
            writer.writerow(formatted)


def _read_rows(path):
    with open(path, newline='') as handle:
        reader = csv.reader(handle)
        header = next(reader)
        return header, [row for row in reader]


def test_dry_run_writes_nothing(tool, tmp_path):
    path = str(tmp_path / 'memory.csv')
    _write_csv(path, [_row(t_abs_s=100.0), _row(t_abs_s=200.0)])
    before = open(path).read()

    result = tool.migrate(path, cutoff_epoch=150.0, dry_run=True)

    assert open(path).read() == before
    assert not os.path.exists(tool.marker_path(path))
    assert not any(f.startswith('memory.csv.bak-') for f in os.listdir(tmp_path))
    assert result.dry_run is True
    assert result.n_rows == 2
    assert result.n_migrated == 1


def test_real_run_writes_backup_and_marker_and_only_touches_rows_before_cutoff(
        tool, tmp_path):
    path = str(tmp_path / 'memory.csv')
    old_row = _row(t_abs_s=100.0, y=(1.0, 2.0, 0.92))
    new_row = _row(t_abs_s=200.0, y=(3.0, 4.0, 0.92))
    _write_csv(path, [old_row, new_row])
    original_text = open(path).read()

    result = tool.migrate(path, cutoff_epoch=150.0)

    assert result.n_migrated == 1
    assert os.path.exists(tool.marker_path(path))
    backups = [f for f in os.listdir(tmp_path) if f.startswith('memory.csv.bak-')]
    assert len(backups) == 1
    assert open(os.path.join(str(tmp_path), backups[0])).read() == original_text

    header, rows = _read_rows(path)
    assert header == _HEADER
    # Row 0 (before cutoff): y2_apex_m (index 9) changed; every other field
    # (x, u, y0, y1, t_abs_s, ball_id, caught) is untouched.
    assert float(rows[0][9]) != pytest.approx(0.92)
    for i in (0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12):
        assert rows[0][i] == (_FLOAT_FMT % old_row[i] if i < 11 else str(old_row[i]))
    # Row 1 (at/after cutoff): every field is byte-identical to the input.
    expected_new = ([_FLOAT_FMT % v for v in new_row[:11]]
                    + [str(new_row[11]), str(new_row[12])])
    assert rows[1] == expected_new


def test_second_run_refuses(tool, tmp_path):
    path = str(tmp_path / 'memory.csv')
    _write_csv(path, [_row(t_abs_s=100.0), _row(t_abs_s=200.0)])

    tool.migrate(path, cutoff_epoch=150.0)
    after_first = open(path).read()

    with pytest.raises(tool.AlreadyMigrated):
        tool.migrate(path, cutoff_epoch=150.0)

    assert open(path).read() == after_first


def test_a_perfect_plant_row_migrates_to_u_within_1e9(tool, tmp_path):
    """Construct the OLD y2_apex_m a perfect plant would have written for a
    known commanded apex ``u`` at the pre-catch-high rise (-0.030 m): solve
    the planner's own model for the crossing speed at T = flight_s(u), then
    read it back through the OLD (rise-unaware) ``apex_from_vz``. Migrating
    that row must recover ``u`` to within 1e-9."""
    g = sch._G_SI
    rise_old = -0.030
    u = 0.92
    t_f = sch.flight_s(u)
    v_c = rise_old / t_f - g * t_f / 2.0
    assert v_c < 0.0
    y2_old = sch.apex_from_vz(v_c)

    path = str(tmp_path / 'memory.csv')
    row = _row(u=(0.01, -0.02, u), y=(0.015, -0.018, y2_old), t_abs_s=50.0)
    _write_csv(path, [row])

    result = tool.migrate(path, cutoff_epoch=100.0, rise_old_m=rise_old)
    assert result.n_migrated == 1

    _header, rows = _read_rows(path)
    y2_new = float(rows[0][9])
    assert y2_new == pytest.approx(u, abs=1e-9)
    # u2_apex_m (the command) and the lateral y0/y1 are untouched.
    assert float(rows[0][6]) == pytest.approx(u)
    assert float(rows[0][7]) == pytest.approx(0.015)
    assert float(rows[0][8]) == pytest.approx(-0.018)


def test_migrated_y2_matches_apex_from_crossing_directly(tool):
    """``migrated_y2`` is exactly the compose of the inverse-``apex_from_vz``
    recovery and ``apex_from_crossing`` -- pinned directly, independent of
    the CSV round trip above."""
    y2_old = 0.8
    rise = -0.03
    g = sch._G_SI
    vz = -math.sqrt(2.0 * g * y2_old)
    expected = sch.apex_from_crossing(vz, rise)
    assert tool.migrated_y2(y2_old, rise) == pytest.approx(expected, abs=1e-12)


def test_migrate_refuses_a_header_mismatched_file(tool, tmp_path):
    path = str(tmp_path / 'memory.csv')
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['x0', 'x1'])
        writer.writerow([1.0, 2.0])
    with pytest.raises(ValueError, match='header'):
        tool.migrate(path, cutoff_epoch=100.0)


def test_migrate_raises_on_missing_file(tool, tmp_path):
    with pytest.raises(FileNotFoundError):
        tool.migrate(str(tmp_path / 'does_not_exist.csv'), cutoff_epoch=100.0)
