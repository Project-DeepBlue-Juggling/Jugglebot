"""Byte-equivalence gate for the deduplicated ROS-clock → perf offset estimator.

``jugglebot/clock_offset.py`` (2026-08-01) replaced two character-identical
copies in ``trajectory_node`` and ``catch_coordinator_node``.  A dedup of
*timing* code has to prove it changed nothing: a drift here does not fail a
functional test, it silently biases catch deadlines.

The frozen reference below is the OLD implementation, transcribed verbatim
from ``trajectory_node.py`` at 2013–2033 / ``catch_coordinator_node.py`` at
395–411 (both were the same code).  Every test drives both through the same
scripted clock and asserts exact float equality — not ``approx``.
"""
from __future__ import annotations

import numpy as np
import pytest

from jugglebot import clock_offset


# ───────────────────────────────────────────────────────────────────
# Frozen reference: the pre-2026-08-01 node-local implementation
# ───────────────────────────────────────────────────────────────────

class _FrozenReferenceNode:
    """Verbatim transcription of the old duplicated methods."""

    def __init__(self, perf_clock, ros_clock_s):
        self._perf = perf_clock
        self._ros = ros_clock_s
        self._ros_to_perf_offset = self._measure_clock_offset()
        self._clock_offset_history = [self._ros_to_perf_offset]

    def _measure_clock_offset(self) -> float:
        offsets = []
        for _ in range(10):
            t_perf = self._perf()
            t_ros = self._ros()
            offsets.append(t_perf - t_ros)
        return float(np.median(offsets))

    def _refresh_clock_offset(self) -> None:
        self._clock_offset_history.append(self._measure_clock_offset())
        if len(self._clock_offset_history) > 20:
            self._clock_offset_history.pop(0)
        self._ros_to_perf_offset = float(np.median(self._clock_offset_history))


# ───────────────────────────────────────────────────────────────────
# Scripted clocks
# ───────────────────────────────────────────────────────────────────

class _ScriptedClock:
    """Deterministic clock: yields a fixed sequence, then repeats the last value."""

    def __init__(self, values):
        self._values = list(values)
        self._i = 0

    def __call__(self) -> float:
        v = self._values[min(self._i, len(self._values) - 1)]
        self._i += 1
        return v


def _clock_pair(seed, n_pairs, jitter_every=7, jitter=0.031):
    """A perf/ros read sequence with a realistic constant offset + hiccups.

    The hiccup is a preemption between the perf and ros reads of one sample —
    exactly the failure mode the median exists to reject.
    """
    rng = np.random.RandomState(seed)
    true_offset = -1234.567891
    perf, ros = [], []
    t = 100.0
    for i in range(n_pairs):
        t += 1e-5 + float(rng.rand()) * 1e-6
        perf.append(t)
        lag = jitter if (i % jitter_every == 0 and i) else 0.0
        ros.append(t - true_offset + lag)
    return _ScriptedClock(perf), _ScriptedClock(ros)


# ───────────────────────────────────────────────────────────────────
# Equivalence
# ───────────────────────────────────────────────────────────────────

def test_measure_offset_matches_frozen_reference_exactly():
    for seed in range(8):
        perf_a, ros_a = _clock_pair(seed, 10)
        perf_b, ros_b = _clock_pair(seed, 10)

        ref = _FrozenReferenceNode(perf_a, ros_a)._ros_to_perf_offset
        new = clock_offset.measure_offset(ros_b, perf_b)

        assert new == ref, 'seed %d: %r != %r' % (seed, new, ref)


def test_refresh_sequence_matches_frozen_reference_exactly():
    """Drive 30 refreshes — past the 20-deep trim — and compare every step."""
    n_refresh = 30
    n_pairs = 10 * (n_refresh + 1) + 5

    perf_a, ros_a = _clock_pair(42, n_pairs)
    perf_b, ros_b = _clock_pair(42, n_pairs)

    ref = _FrozenReferenceNode(perf_a, ros_a)
    history = [clock_offset.measure_offset(ros_b, perf_b)]
    assert history[0] == ref._ros_to_perf_offset

    for step in range(n_refresh):
        ref._refresh_clock_offset()
        new_offset = clock_offset.refresh_offset(history, ros_b, perf_b)

        assert new_offset == ref._ros_to_perf_offset, 'step %d' % step
        assert history == ref._clock_offset_history, 'step %d history' % step
        assert len(history) <= 20


def test_history_is_trimmed_from_the_front_and_capped_at_20():
    perf, ros = _clock_pair(7, 10 * 40)
    history = [clock_offset.measure_offset(ros, perf)]
    seen = list(history)
    for _ in range(35):
        clock_offset.refresh_offset(history, ros, perf)
        seen.append(history[-1])
    assert len(history) == 20
    assert history == seen[-20:]          # oldest dropped, order preserved


def test_median_rejects_a_single_preempted_sample():
    """One 31 ms hiccup inside a measurement must not move the offset."""
    clean_perf, clean_ros = _clock_pair(3, 10, jitter_every=10 ** 6)
    hiccup_perf, hiccup_ros = _clock_pair(3, 10, jitter_every=9, jitter=0.031)

    clean = clock_offset.measure_offset(clean_ros, clean_perf)
    hiccup = clock_offset.measure_offset(hiccup_ros, hiccup_perf)

    assert hiccup == pytest.approx(clean, abs=1e-9)


def test_read_order_is_perf_then_ros():
    """The pair is not atomic; a swapped order flips the sign of the bias."""
    order = []

    def perf():
        order.append('perf')
        return 10.0

    def ros():
        order.append('ros')
        return 1.0

    clock_offset.measure_offset(ros, perf, samples=3)
    assert order == ['perf', 'ros'] * 3


def test_shared_constants_match_both_node_call_sites():
    """The constants are the values the nodes ran with before the dedup.

    ``DEFAULT_SAMPLES`` / ``DEFAULT_HISTORY`` flow through the shared
    functions, so the equalities above pin them everywhere.  ``REFRESH_PERIOD_S``
    does not — it is consumed at the nodes' ``create_timer`` call sites — so
    this test reads those two call sites out of the source and fails if either
    goes back to a bare literal.  Otherwise the constant is free to drift from
    the cadence it claims to describe with nothing going red, which is the
    duplicated-timing-constant shape this unit set out to kill.
    """
    import ast
    import os

    assert clock_offset.DEFAULT_SAMPLES == 10
    assert clock_offset.DEFAULT_HISTORY == 20
    assert clock_offset.REFRESH_PERIOD_S == 30.0

    pkg_dir = os.path.dirname(os.path.abspath(clock_offset.__file__))
    for node_file in ('trajectory_node.py', 'catch_coordinator_node.py'):
        with open(os.path.join(pkg_dir, node_file)) as fh:
            tree = ast.parse(fh.read(), node_file)
        wired = []
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'create_timer'
                    and node.args):
                first = node.args[0]
                callback = node.args[1] if len(node.args) > 1 else None
                is_refresh = (isinstance(callback, ast.Attribute)
                              and callback.attr == '_refresh_clock_offset')
                if not is_refresh:
                    continue
                wired.append(ast.dump(first))
                assert (isinstance(first, ast.Attribute)
                        and first.attr == 'REFRESH_PERIOD_S'), (
                    f"{node_file}:{node.lineno} passes a literal period to "
                    f"create_timer(..., _refresh_clock_offset); it must pass "
                    f"clock_offset.REFRESH_PERIOD_S so the constant is real")
        assert len(wired) == 1, (
            f"{node_file}: expected exactly one clock-offset refresh timer, "
            f"found {len(wired)}")


def test_module_is_pure_no_rclpy_import():
    """clock_offset must stay importable without ROS2 (pure-module boundary)."""
    import ast
    import inspect

    tree = ast.parse(inspect.getsource(clock_offset))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name.split('.')[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split('.')[0])
    assert 'rclpy' not in imported
    assert 'jugglebot' not in imported          # no node/ROS coupling either


def test_history_list_object_is_mutated_in_place():
    """Nodes expose the history as an attribute; refresh must not rebind it."""
    perf, ros = _clock_pair(11, 200)
    history = []
    same = history
    for _ in range(3):
        clock_offset.refresh_offset(history, ros, perf)
    assert history is same
    assert len(history) == 3
