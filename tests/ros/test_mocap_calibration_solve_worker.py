"""The BB calibration solve runs OFF mocap_node's executor thread (2026-10-16).

Invariant (``jugglebot/calibration_solve_worker.py``): no executor callback of
mocap_node runs the calibration solve. The bb/heartbeat exit edge snapshots
the sweep and returns; one worker thread solves; an executor-thread timer
(``_apply_calibration_outcomes``) applies. Before this the solve ran inside
the heartbeat callback for 4.3-7.2 s and starved the 5 ms frame drain
(logbook 2026-10-16-mocap-frame-queue-starvation).

Pinned here: (a) the heartbeat callback returns while a solve is still
blocked, and the solve is not on the calling thread; (b) the result is applied
exactly once with the stubbed values; (c) a second sweep queues behind the
first, never concurrently, each applied once in order with its OWN inputs;
(d) a solve that raises is reported once, changes no calibration state, and
the worker solves the next sweep; (e) shutdown with a solve in flight joins
with a bounded timeout and drops the result.

The stub solves block on ``threading.Event``s, not sleeps, so the ordering is
deterministic; the one wall-clock bound (a) is 50 ms against a solve that does
not return until the test releases it.

See ``logbook/2026-10-16-mocap-calibration-solve-off-executor.md``.
"""
from __future__ import annotations

import threading
import time
from unittest.mock import MagicMock, patch

import numpy as np

from jugglebot.calibration_solve_worker import CalibrationSolveWorker
from jugglebot.protocol_config import BallButlerStates
from tests.ros.test_mocap_node_keep_last_good import _hb, _rec, _result

WAIT_S = 5.0


def _node(tmp_path):
    import jugglebot.mocap_node as mn
    iface = MagicMock()
    iface.is_receiving.return_value = True
    with patch.object(mn, 'MocapInterface', return_value=iface):
        node = mn.MocapNode()                      # the LIVE (threaded) solve
    node._params['bb_calibration_state_file'] = str(tmp_path / 'last_accepted.json')
    node._params['bb_base_frame_state_file'] = str(tmp_path / 'base_state.json')
    return node


def _end_sweep(node, n_points):
    node._on_bb_heartbeat(_hb(BallButlerStates.CALIBRATING))
    node._calib_data = {0: [np.zeros(3)] * n_points}
    t0 = time.monotonic()
    node._on_bb_heartbeat(_hb(BallButlerStates.IDLE))
    return time.monotonic() - t0


def _apply_until(node, pred):
    deadline = time.monotonic() + WAIT_S
    while time.monotonic() < deadline:
        node._apply_calibration_outcomes()
        if pred():
            return True
        time.sleep(0.005)
    return False


class _Stub:
    """run_calibration stand-in: blocks on a per-call Event, records the
    calling thread, the input size and the peak concurrency."""

    def __init__(self, results):
        self.results = list(results)
        self.release = [threading.Event() for _ in self.results]
        self.started = [threading.Event() for _ in self.results]
        self.calls = []
        self.threads = []
        self._active = 0
        self.peak = 0
        self._lock = threading.Lock()

    def __call__(self, **kw):
        with self._lock:
            i = len(self.calls)
            self.calls.append(len(kw['calibration_data'][0]))
            self.threads.append(threading.current_thread())
            self._active += 1
            self.peak = max(self.peak, self._active)
        self.started[i].set()
        self.release[i].wait(WAIT_S)
        with self._lock:
            self._active -= 1
        r = self.results[i]
        if isinstance(r, Exception):
            raise r
        return r


def _successes(node):
    return [m for m in node.pub_calibration.published if m.success]


def test_heartbeat_returns_while_the_solve_runs_and_applies_once(tmp_path):
    import jugglebot.mocap_node as mn
    node = _node(tmp_path)
    stub = _Stub([_result(0.926)])
    with patch.object(mn, 'run_calibration', side_effect=stub):
        dt = _end_sweep(node, 3)
        assert stub.started[0].wait(WAIT_S)
        # (a) the callback returned with the solve still blocked, off-thread
        # 0.25 s: xdist load headroom; still far under the ~4 s solve
        assert dt < 0.25, f'heartbeat callback took {dt * 1e3:.1f} ms'
        assert stub.threads[0] is not threading.current_thread()
        node._apply_calibration_outcomes()
        assert node.pub_calibration.published == []
        assert node._calibrating is False           # collection closed at once
        stub.release[0].set()
        assert _apply_until(node, lambda: _successes(node))
    # (b) exactly once, with the stubbed values
    for _ in range(5):
        node._apply_calibration_outcomes()
    ok = _successes(node)
    assert len(ok) == 1
    assert abs(ok[0].yaw_offset_rad - _result(0.926).yaw_offset_rad) < 1e-12
    assert node._last_good_calibration is ok[0]
    assert (tmp_path / 'last_accepted.json').exists()
    node._calib_solver.stop(1.0)


def test_a_second_sweep_queues_behind_with_its_own_inputs(tmp_path):
    import jugglebot.mocap_node as mn
    node = _node(tmp_path)
    stub = _Stub([_result(0.900), _result(0.950)])
    with patch.object(mn, 'run_calibration', side_effect=stub):
        _end_sweep(node, 3)
        assert stub.started[0].wait(WAIT_S)
        # A new sweep STARTS and ENDS while the first solve is in flight.
        _end_sweep(node, 7)
        time.sleep(0.05)
        assert len(stub.calls) == 1, 'the second solve ran concurrently'
        stub.release[0].set()
        assert stub.started[1].wait(WAIT_S)
        stub.release[1].set()
        assert _apply_until(node, lambda: len(_successes(node)) == 2)
    assert stub.peak == 1
    assert stub.calls == [3, 7]          # each solve saw its own window's copy
    ok = _successes(node)
    assert [round(m.yaw_offset_rad, 9) for m in ok] == [
        round(_result(0.900).yaw_offset_rad, 9), round(_result(0.950).yaw_offset_rad, 9)]
    node._calib_solver.stop(1.0)


def test_a_solve_that_raises_changes_nothing_and_the_worker_lives(tmp_path):
    import jugglebot.mocap_node as mn
    node = _node(tmp_path)
    lines = _rec(node)
    stub = _Stub([RuntimeError('boom'), _result(0.926)])
    for e in stub.release:
        e.set()
    with patch.object(mn, 'run_calibration', side_effect=stub):
        _end_sweep(node, 3)
        assert _apply_until(node, lambda: node.pub_calibration_attempt.published)
        attempt = node.pub_calibration_attempt.published[-1]
        assert not attempt.success
        assert attempt.message.startswith('CALIBRATION_SOLVE_ERROR: RuntimeError: boom')
        assert [lvl for lvl, m in lines if 'CALIBRATION_SOLVE_ERROR' in m] == ['error']
        assert _successes(node) == [] and node._last_good_calibration is None
        assert not (tmp_path / 'last_accepted.json').exists()
        assert not (tmp_path / 'base_state.json').exists()
        # the worker survived: the next sweep is solved and applied
        _end_sweep(node, 3)
        assert _apply_until(node, lambda: _successes(node))
    node._calib_solver.stop(1.0)


def test_shutdown_with_a_solve_in_flight_is_bounded_and_drops_it():
    gate = threading.Event()
    started = threading.Event()

    def solve(job):
        started.set()
        gate.wait(WAIT_S)
        return job

    w = CalibrationSolveWorker(solve)
    assert w.submit('sweep')
    assert started.wait(WAIT_S)
    t0 = time.monotonic()
    assert w.stop(timeout=0.05) is False           # still running: not waited out
    assert time.monotonic() - t0 < 0.5
    assert w.submit('late') is False
    gate.set()
    time.sleep(0.05)
    assert w.drain() == []                          # the in-flight result is dropped


def test_inline_mode_solves_inside_submit():
    w = CalibrationSolveWorker(lambda job: job * 2, inline=True)
    assert w.submit(21)
    (o,) = w.drain()
    assert (o.value, o.error) == (42, None) and w.drain() == []


def test_an_apply_that_raises_does_not_lose_the_rest_of_the_batch(tmp_path):
    import jugglebot.mocap_node as mn
    node = _node(tmp_path)
    lines = _rec(node)
    stub = _Stub([_result(0.900), _result(0.950)])
    for e in stub.release:
        e.set()
    real = node._apply_sweep_solve
    calls = {'n': 0}

    def flaky(snap, solve):
        calls['n'] += 1
        if calls['n'] == 1:
            raise RuntimeError('apply boom')
        return real(snap, solve)

    with patch.object(mn, 'run_calibration', side_effect=stub):
        _end_sweep(node, 3)
        _end_sweep(node, 7)
        assert stub.started[1].wait(WAIT_S)
        deadline = time.monotonic() + WAIT_S
        while len(node._calib_solver._outcomes) < 2:    # both finished, undrained
            assert time.monotonic() < deadline
            time.sleep(0.005)
        node._apply_sweep_solve = flaky
        node._apply_calibration_outcomes()    # ONE tick drains both
    assert calls['n'] == 2
    assert len(_successes(node)) == 1
    assert [m for lv, m in lines if lv == 'error'
            and m == 'BB calibration apply failed: RuntimeError: apply boom'] != []
    assert len([1 for lv, m in lines if 'BB calibration apply failed' in m]) == 1
    node._calib_solver.stop(1.0)


def test_node_shutdown_with_a_blocked_solve_warns_once_and_destroys(tmp_path):
    import jugglebot.mocap_node as mn
    node = _node(tmp_path)
    lines = _rec(node)
    stub = _Stub([_result(0.926)])
    node.destroy_node = MagicMock()
    with patch.object(mn, 'run_calibration', side_effect=stub), \
            patch.object(mn, 'CALIBRATION_SOLVE_JOIN_TIMEOUT_S', 0.05):
        _end_sweep(node, 3)
        assert stub.started[0].wait(WAIT_S)
        node.on_shutdown()
        stub.release[0].set()
    warns = [m for lv, m in lines if lv == 'warn']
    assert warns == ['BB calibration solve still running at shutdown: result dropped']
    node.destroy_node.assert_called_once()
