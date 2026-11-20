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


# ---- BUG-58 appended fields (collected/uncollectable, collector state) ----

def test_worst_collection_size_from_stop_info():
    p = GcProbe()
    p._callback('start', {'generation': 2})
    p._callback('stop', {'generation': 2, 'collected': 7, 'uncollectable': 1})
    s = p.summary()
    assert s['worst'][0] == 2
    assert s['worst_collected'] == 7 and s['worst_uncollectable'] == 1


def test_callback_tolerates_missing_info_keys():
    p = GcProbe()
    p._callback('start', {})
    p._callback('stop', {})          # must not raise
    s = p.summary()
    assert s['worst_collected'] == -1 and s['worst_uncollectable'] == -1


def test_state_fields_and_objects_knob_off_by_default(monkeypatch):
    monkeypatch.delenv('JUGGLEBOT_GC_PROBE_OBJECTS_EVERY_S', raising=False)
    p = GcProbe()
    p.install()
    try:
        st = p.state()
    finally:
        p.uninstall()
    assert len(st['count']) == 3 and len(st['colls']) == 3
    assert st['frozen'] >= 0 and st['objects0'] > 0
    assert st['objects'] == -1          # the multi-ms call stays off 1 Hz


def test_objects_knob_samples_when_requested():
    p = GcProbe(objects_every_s=0.001)
    st = p.state()
    assert st['objects'] > 0


def test_format_summary_appends_and_keeps_old_prefix():
    s = {'count': 2, 'max_ms': 12.5, 'total_ms': 13.0,
         'worst': (1, 12.5, 1.7e9), 'worst_collected': 40, 'worst_uncollectable': 0}
    old = format_summary('x', s)
    st = {'count': (1, 2, 3), 'colls': (10, 20, 30), 'frozen': 5,
          'objects0': 99, 'objects': -1}
    new = format_summary('x', s, st)
    assert new.startswith(old + ' ')
    assert new.endswith('worst_collected=40 worst_uncollectable=0 '
                        'gc_count=1,2,3 gc_colls=10,20,30 gc_frozen=5 '
                        'gc_objects0=99')
    assert 'gc_objects=' not in new.replace('gc_objects0=', '')
    assert format_summary('x', s, dict(st, objects=1234)).endswith(
        'gc_objects0=99 gc_objects=1234')


def test_off_path_unchanged_no_callback_without_install():
    n0 = len(gc.callbacks)
    p = GcProbe()
    assert not p.installed and len(gc.callbacks) == n0
    assert p.objects_at_install == -1   # no get_objects() unless installed
