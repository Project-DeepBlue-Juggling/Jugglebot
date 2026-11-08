"""BUG-58: the GC pause probe's pure helper (jugglebot/gc_probe.py)."""
from __future__ import annotations

import gc

from jugglebot.gc_probe import GcProbe, format_summary


def test_deque_is_bounded():
    p = GcProbe(maxlen=4)
    p.install()
    try:
        for _ in range(12):
            gc.collect()
        s = p.summary()
    finally:
        p.uninstall()
    assert s['count'] == 4


def test_summary_drains():
    p = GcProbe()
    p.install()
    try:
        gc.collect()
        assert p.summary()['count'] >= 1
        assert p.summary() == {'count': 0, 'max_ms': 0.0, 'total_ms': 0.0,
                               'worst': None}
    finally:
        p.uninstall()


def test_forced_collect_is_recorded_as_generation_2():
    p = GcProbe()
    p.install()
    try:
        p.summary()
        gc.collect()
        s = p.summary()
    finally:
        p.uninstall()
    gen, ms, wall = s['worst']
    assert gen == 2 and ms >= 0.0 and wall > 1e9
    assert s['max_ms'] == ms <= s['total_ms'] + 1e-9


def test_uninstall_removes_hook_and_install_is_idempotent():
    p = GcProbe()
    n0 = len(gc.callbacks)
    p.install()
    p.install()
    assert len(gc.callbacks) == n0 + 1 and p.installed
    p.uninstall()
    p.uninstall()
    assert len(gc.callbacks) == n0 and not p.installed
    gc.collect()
    assert p.summary()['count'] == 0


def test_format_summary_is_grep_friendly():
    line = format_summary('x', {'count': 2, 'max_ms': 12.5, 'total_ms': 13.0,
                                'worst': (1, 12.5, 1.7e9)})
    assert line.startswith('GC_PROBE x n=2 max_ms=12.500 ')
