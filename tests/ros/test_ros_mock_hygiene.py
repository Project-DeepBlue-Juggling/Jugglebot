"""The ROS2 stand-ins in ``tests/ros/conftest.py`` must not hoard call records.

WHAT THIS FILE DEFENDS
----------------------
``conftest.py`` replaces ``rclpy`` and the message packages with plain modules
built ONCE, at import, and therefore alive for the whole worker process. Any
attribute of one of those modules that is a :mod:`unittest.mock` object records
every call it ever receives in ``mock_calls``, and nothing in the session ever
drops that record: it is not a per-test fixture, no ``reset_mock()`` runs
between tests, and the module outlives every file the worker picks up under
``--dist loadfile``.

That is not a slow leak, it is an unbounded one, because production spin loops
call these functions as fast as the CPU allows. ``reload_coordinator_node`` has
twelve ``while rclpy.ok():`` waits, and the tests that drive them patch
``time.sleep`` to a no-op while the loop's exit condition stays a REAL
wall-clock deadline — so one test evaluates the condition millions of times.
Measured 2026-09-07 before the fix (``pytest tests/ros/test_toss_coordinator.py
-q``, venv interpreter): **2.41 M retained ``unittest.mock._Call`` objects and a
1.21 GB peak RSS in that single file**, ~960 MB of it from four tier-8b tests.
Afterwards the same file peaks at **170 MB**. On the gate that difference is the
whole failure: ``./run_tests.sh --full`` runs four xdist workers under
``--dist loadfile``, so a file's entire hoard lands in ONE worker, and on
2026-09-06/07 that worker reached 2.43 GB anon-rss and was OOM-killed — the
``[gwN] node down: Not properly terminated`` that wedged the gate twice.

The invariant, stated once so the whole class stays closed rather than the four
attributes that happened to be Mocks in September 2026:

    **No attribute of a conftest-created stand-in module may record its calls.**

Nothing is given up by it. A process-lifetime mock's call record is shared by
every test in every file the worker runs, so a call-count assertion on one could
never have been sound in the first place; a test that needs different behaviour
replaces the attribute for its own duration (``test_toss_coordinator.py`` does
exactly that with ``monkeypatch.setattr(rclpy, 'ok', _ok)``), which works
identically against a plain function.

The enforcement point is ``tests/ros/conftest.py``'s ``rclpy`` block; this file
is the test that fails when it drifts.

ROS 2 is mocked by ``tests/ros/conftest.py``. Nothing here binds a port or
touches the filesystem, so the file is xdist-parallel-safe.
"""

from __future__ import annotations

import sys
import types
from unittest import mock

import pytest

import rclpy


#: The package roots ``tests/ros/conftest.py`` replaces wholesale. Every module
#: registered under one of these is a ``types.ModuleType`` this suite built, so
#: it lives for the worker process and its attributes are process-global state.
_MOCKED_ROOTS = (
    'rclpy',
    'tf2_ros',
    'std_msgs',
    'std_srvs',
    'geometry_msgs',
    'jugglebot_interfaces',
)

#: The `rclpy` entry points production spin loops call. Named explicitly so the
#: test still says something if a future conftest stops defining one of them.
_SPIN_LOOP_ENTRY_POINTS = ('ok', 'init', 'shutdown', 'spin_once')


def _stand_in_modules():
    """Every conftest-created stand-in module currently in ``sys.modules``.

    Identified by "has no ``__file__``": ``_create_mock_module`` builds a bare
    ``types.ModuleType``, while an importable module always carries one. That is
    a property of how the stand-ins are made rather than a list to maintain, so
    a package added to conftest tomorrow is covered without touching this file.
    """
    out = []
    for name, mod in list(sys.modules.items()):
        if not isinstance(mod, types.ModuleType):
            continue
        if getattr(mod, '__file__', None) is not None:
            continue
        root = name.split('.')[0]
        if root in _MOCKED_ROOTS:
            out.append((name, mod))
    return out


@pytest.mark.parametrize('name', _SPIN_LOOP_ENTRY_POINTS)
def test_the_rclpy_spin_loop_entry_points_are_not_mocks(name):
    """``ok`` / ``init`` / ``shutdown`` / ``spin_once`` record nothing.

    ``ok`` is the one that actually bit (a 5 s no-op-sleep wait spins it
    millions of times), but the other three are one driver away from the same
    thing — ``spacemouse_handler.main`` calls ``spin_once`` inside a
    ``while rclpy.ok():`` loop — so the guard covers the entry points rather
    than the single incident.
    """
    fn = getattr(rclpy, name)
    assert not isinstance(fn, mock.NonCallableMock), (
        'rclpy.%s is a %s. It is defined once per worker process and never '
        'reset, so every call a spin loop makes is retained for the life of '
        'the run (2.41 M _Call objects / 1.21 GB in one file, measured '
        '2026-09-07). Use a plain function; see the rclpy block in '
        'tests/ros/conftest.py.' % (name, type(fn).__name__))
    assert not hasattr(fn, 'mock_calls')


def test_no_stand_in_module_attribute_records_its_calls():
    """The invariant itself, swept over every stand-in module.

    Deliberately broader than the four names above: the cost of a
    process-lifetime recorder is paid by whichever production loop happens to
    call it, which is not knowable from conftest, so the rule is about the
    LIFETIME of the object and not about any particular function.
    """
    offenders = []
    for mod_name, mod in _stand_in_modules():
        for attr in dir(mod):
            if attr.startswith('__'):
                continue
            try:
                value = getattr(mod, attr)
            except Exception:                      # pragma: no cover - defensive
                continue
            if isinstance(value, mock.NonCallableMock):
                offenders.append('%s.%s (%s)'
                                 % (mod_name, attr, type(value).__name__))
    assert offenders == [], (
        'these stand-in module attributes are mock objects and will retain '
        'every call made against them for the whole worker process: %s'
        % ', '.join(offenders))


def test_ok_is_still_true_and_still_replaceable(monkeypatch):
    """The fix must not cost the two behaviours the stand-in exists for.

    A spin loop has to see True by default, and a test has to be able to make it
    say False for its own duration — that is how the shutdown paths are driven
    (``test_toss_coordinator.py``). Restoration matters as much as the override:
    a leaked False here would silently turn every later ``while rclpy.ok():``
    into a no-op, which reads as a passing test that ran nothing.
    """
    assert rclpy.ok() is True
    monkeypatch.setattr(rclpy, 'ok', lambda *a, **k: False)
    assert rclpy.ok() is False
    monkeypatch.undo()
    assert rclpy.ok() is True


def test_hammering_ok_retains_nothing():
    """A million calls cost no retained objects — the property that was broken.

    Counted with ``gc``, over ``unittest.mock._Call`` specifically, because that
    is the object the old MagicMock accumulated (one per call, ~130 B) and it is
    what any future recorder would accumulate too. An RSS or ``len(gc
    .get_objects())`` reading would be a machine measurement and would belong in
    the ``serial`` phase; this one is exact and holds under parallel load.
    """
    import gc

    def _calls():
        return sum(1 for o in gc.get_objects()
                   if type(o) is mock._Call)          # noqa: SLF001

    gc.collect()
    before = _calls()
    for _ in range(1_000_000):
        rclpy.ok()
    gc.collect()
    assert _calls() - before < 1000
