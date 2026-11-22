"""BUG-129: the stall watchdog (``jugglebot/stall_watchdog.py``) dumps every
Python thread's stack while a stall is in progress and reports it once.

The stall driver is ``ctypes.PyDLL(None).usleep``: a ``PyDLL`` call does NOT
release the GIL, so it freezes every Python thread of the process for exactly
the requested time, the signature of the 2026-11-22 bridge freeze (every
thread stopped, the kernel kept buffering UDP). Confirmed by probe 2026-11-23
on the Jetson (Python 3.8 and 3.11): a 300 ms hold against a 100 ms timeout
yields one dump naming ``_hold_gil`` in the holder thread.

Margins are wide on purpose (300 ms stall vs 100 ms timeout, reports polled
for up to 5 s) so the tests hold on a loaded box; a load-induced extra dump
is tolerated, never required.
"""
from __future__ import annotations

import ctypes
import faulthandler
import os
import threading
import time

import pytest

from jugglebot.stall_watchdog import StallWatchdog, measure_rearm_cost

_LIBC = ctypes.PyDLL(None)


def _hold_gil(ms: int) -> None:
    _LIBC.usleep(int(ms * 1000))


def _wait(pred, timeout_s=5.0):
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        if pred():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def warns():
    return []


def _wd(tmp_path, warns, **kw):
    kw.setdefault('timeout_s', 0.10)
    kw.setdefault('period_s', 0.03)
    kw.setdefault('names_every', 1)
    return StallWatchdog(str(tmp_path), 'unit', log_warn=warns.append, **kw)


def test_gil_held_stall_is_dumped_mid_stall_and_named(tmp_path, warns):
    wd = _wd(tmp_path, warns).start()
    try:
        holder = threading.Thread(target=_hold_gil, args=(300,), name='gil_holder')
        time.sleep(0.1)
        holder.start()
        holder.join()
        assert _wait(lambda: wd.reports), warns
        text = open(wd.reports[0]).read()
        # The holder thread's frame is in the dump, and its id maps to its name.
        assert '_hold_gil' in text
        assert 'gil_holder' in text
        assert 'Thread 0x' in text
        assert warns and 'stall watchdog: unit stalled' in warns[0]
        assert wd.reports[0] in warns[0]
    finally:
        wd.stop()


def test_no_stall_no_dump(tmp_path, warns):
    wd = _wd(tmp_path, warns).start()
    try:
        time.sleep(0.4)
        wd.tick()
    finally:
        wd.stop()
    assert wd.stalls == 0 and wd.reports == [] and warns == []
    assert os.listdir(str(tmp_path)) == []


def test_watched_loop_stall_alone_is_dumped(tmp_path, warns):
    """The process runs, only the watched loop is stuck: re-arm stops, the timer fires."""
    stuck = {'age': None}
    wd = _wd(tmp_path, warns, heartbeat=lambda: stuck['age']).start()
    try:
        time.sleep(0.1)
        stuck['age'] = 1.0           # the loop reports a 1 s old tick from now on
        assert _wait(lambda: wd.reports), warns
        assert 'largest gap 1000.0 ms' in warns[0]
    finally:
        wd.stop()


def test_report_cap_still_warns(tmp_path, warns):
    wd = _wd(tmp_path, warns, max_reports=0).start()
    try:
        time.sleep(0.05)
        _hold_gil(250)
        assert _wait(lambda: wd.stalls >= 1), warns
    finally:
        wd.stop()
    assert wd.reports == [] and os.listdir(str(tmp_path)) == []
    assert 'not written: cap of 0 reports' in warns[0]


def test_stop_is_idempotent_and_disarms(tmp_path, warns):
    wd = _wd(tmp_path, warns).start()
    wd.stop()
    wd.stop()
    assert not wd.running
    # Disarmed: a stall after stop() must not write to the (closed) file. If the
    # timer were still armed it would write into a closed fd; nothing must happen.
    _hold_gil(200)
    assert wd.stalls == 0 and os.listdir(str(tmp_path)) == []


def test_rejects_timeout_not_above_two_periods(tmp_path):
    with pytest.raises(ValueError):
        StallWatchdog(str(tmp_path), 'unit', timeout_s=0.1, period_s=0.05)


def test_measure_rearm_cost_leaves_timer_cancelled():
    cost = measure_rearm_cost(n=50)
    assert cost > 0.0
    # cancel is safe to repeat; nothing armed is left behind for other tests
    faulthandler.cancel_dump_traceback_later()
