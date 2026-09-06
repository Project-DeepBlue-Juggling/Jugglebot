"""test_blas_threads.py — the planner nodes' BLAS thread-pool self-check.

WHY THIS EXISTS (measured 2026-09-06, 60+ reps, dose-response —
``logbook/2026-09-06-uh3-first-attempt-refusals-and-estop.md`` § Diagnosis,
"Cause pinned (2026-09-06 evening)"):

A unified ``plan_cycle`` is thousands of SMALL numpy calls, each fanning out to
OpenBLAS's DEFAULT six-worker pool whose workers busy-spin between calls. Idle,
the pool is free (194-223 ms default vs 195-207 ms capped — identical). At THREE
busy cores of six the same solve takes **1350-2314 ms** and gaps
``trajectory_node``'s 40 Hz emitter **225-942 ms**, past the can-bridge's 250 ms
``MPC_STALE`` watchdog — which latches and E-STOPs the machine mid-rung. Capped
to one thread: 214-217 ms at ANY load.

The cap itself lives in ``jugglebot_launch.py``'s ``additional_env`` (it has to
be set before numpy is imported), pinned by ``tests/ros/test_launch_nodes.py``.
THIS file pins the other half: the runtime read-back that tells an operator
whether the cap actually landed in the process. Without it the failure is
invisible — an uncapped pool looks exactly like a capped one until the box gets
busy, at which point it looks like an E-STOP.

**The WARN is deliberately not a refusal.** A developer running ``ros2 run
jugglebot trajectory_node`` by hand has no launch file and must still be able to
work, and an uncapped pool is a slow solve, not an unsafe command. But an
UNKNOWN pool warns too: fail-closed means a node that cannot prove it is capped
must not read as capped.
"""

from __future__ import annotations

import pytest

from jugglebot.motion import blas_threads as bt


class _Log:
    """Minimal stand-in for an rclpy logger — records what was said."""

    def __init__(self):
        self.info_lines = []
        self.warn_lines = []

    def info(self, msg):
        self.info_lines.append(str(msg))

    def warning(self, msg):
        self.warn_lines.append(str(msg))


def _reader(n, source='test'):
    return lambda: (n, source)


# ── the INFO line ────────────────────────────────────────────────────────

def test_one_info_line_names_the_count_and_its_source():
    log = _Log()
    n, src = bt.check_blas_threads(log, 'trajectory_node',
                                   reader=_reader(1, 'threadpoolctl:openblas'))
    assert (n, src) == (1, 'threadpoolctl:openblas')
    assert log.info_lines == ['blas threads: 1 (threadpoolctl:openblas)']


def test_a_capped_pool_does_not_warn():
    """N == 1 is the shipped configuration and must be silent past the INFO."""
    log = _Log()
    bt.check_blas_threads(log, 'trajectory_node', reader=_reader(1))
    assert log.warn_lines == [], (
        'A correctly capped node warned — operators learn to ignore a warning '
        'that fires on the good path, which is how the real one gets missed.')


# ── the WARN ─────────────────────────────────────────────────────────────

@pytest.mark.parametrize('n', [2, 4, 6, 12])
def test_an_uncapped_pool_warns_loudly(n):
    """N > 1 is the E-STOP condition and must be impossible to miss."""
    log = _Log()
    bt.check_blas_threads(log, 'trajectory_node', reader=_reader(n))
    assert len(log.warn_lines) == 1
    assert len(log.info_lines) == 1, 'the INFO line is emitted either way'


def test_the_warning_names_the_node_the_launch_file_and_the_entry():
    """A warning that does not say what to DO is a warning that gets ignored."""
    log = _Log()
    bt.check_blas_threads(log, 'reload_coordinator_node', reader=_reader(6))
    warn = log.warn_lines[0]
    assert 'reload_coordinator_node' in warn
    assert 'jugglebot_launch.py' in warn
    assert '2026-09-06-uh3-first-attempt-refusals-and-estop' in warn
    # The two variables an operator has to set, spelled out.
    assert 'OPENBLAS_NUM_THREADS=1' in warn
    assert 'OMP_NUM_THREADS=1' in warn
    # The measured numbers, so the reader can judge the stakes rather than
    # taking the warning's word for it.
    assert '250 ms' in warn and 'MPC_STALE' in warn


def test_an_unknown_pool_warns_too():
    """Fail-closed: unprovable is not the same as fine.

    ``read_blas_threads`` returns ``None`` when neither threadpoolctl nor an
    env var can answer. Treating that as "probably capped" would restore
    exactly the silence this check exists to break.
    """
    log = _Log()
    n, _ = bt.check_blas_threads(log, 'trajectory_node',
                                 reader=_reader(None, 'unknown'))
    assert n is None
    assert len(log.warn_lines) == 1
    assert log.info_lines == ['blas threads: unknown (unknown)']


# ── the reader itself ────────────────────────────────────────────────────

def test_the_env_fallback_reads_the_cap(monkeypatch):
    """With threadpoolctl unavailable, the env var is the answer.

    threadpoolctl is 3.5.0 in BOTH interpreters on this box (the venv AND
    ``/usr/bin/python3``, which is the one the launch file runs), so the
    fallback is not the live path today — but it is what a fresh Jetson image or
    a stripped deployment gets, and an ImportError there must degrade to the
    env read rather than to ``None`` (which would warn on a correctly capped
    node forever).
    """
    monkeypatch.setitem(__import__('sys').modules, 'threadpoolctl', None)
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '1')
    monkeypatch.delenv('OMP_NUM_THREADS', raising=False)
    n, source = bt.read_blas_threads()
    assert (n, source) == (1, 'env:OPENBLAS_NUM_THREADS')


def test_the_env_fallback_reports_an_uncapped_value_honestly(monkeypatch):
    monkeypatch.setitem(__import__('sys').modules, 'threadpoolctl', None)
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '6')
    n, source = bt.read_blas_threads()
    assert n == 6 and source == 'env:OPENBLAS_NUM_THREADS'


def test_nothing_to_read_is_reported_as_unknown_not_as_one(monkeypatch):
    monkeypatch.setitem(__import__('sys').modules, 'threadpoolctl', None)
    for var in bt.BLAS_ENV_VARS:
        monkeypatch.delenv(var, raising=False)
    n, source = bt.read_blas_threads()
    assert n is None
    assert 'unknown' in source


def test_threadpoolctl_reports_the_worst_pool_in_the_process(monkeypatch):
    """Two BLAS libraries loaded → the LARGER pool is the one that starves us.

    numpy can pull in more than one threaded backend. Reporting the smallest
    (or the first) would let a six-thread pool hide behind a one-thread one.
    """
    class _Fake:
        @staticmethod
        def threadpool_info():
            return [
                {'user_api': 'blas', 'internal_api': 'openblas',
                 'num_threads': 1},
                {'user_api': 'openmp', 'internal_api': 'openmp',
                 'num_threads': 6},
                {'user_api': 'other', 'internal_api': 'ignored',
                 'num_threads': 99},
            ]

    monkeypatch.setitem(__import__('sys').modules, 'threadpoolctl', _Fake)
    n, source = bt.read_blas_threads()
    assert n == 6, 'the worst pool wins — it is the one that evicts the emitter'
    assert source.startswith('threadpoolctl:')
    assert 'ignored' not in source, 'non-BLAS/OpenMP pools are not our business'


def test_the_live_process_can_be_read():
    """Whatever this interpreter has, the reader answers without raising.

    No assertion on the VALUE: the test suite runs uncapped by design (it is not
    on the setpoint stream), so pinning a number here would pin the test
    environment, not the contract.
    """
    n, source = bt.read_blas_threads()
    assert n is None or int(n) >= 1
    assert isinstance(source, str) and source
