"""BUG-96: an emitter that cannot bind :5557 must not leave the node looking
healthy.  The emitter thread records the failure and shuts the context down;
``main()`` exits non-zero.  ROS is mocked (tests/ros/conftest)."""

from __future__ import annotations

import pytest

from jugglebot import trajectory_node as tn


def _boom():
    raise OSError('Address already in use')


def test_bind_failure_flags_node_and_requests_shutdown(monkeypatch):
    calls = []
    monkeypatch.setattr(tn.rclpy, 'shutdown', lambda *a, **k: calls.append(1))
    node = tn.TrajectoryNode(command_pub_factory=_boom, start_emitter=False)
    assert node.emitter_bind_failed is False
    node._emitter_loop()          # runs the bind inline; returns on failure
    assert node.emitter_bind_failed is True
    assert calls == [1]


def test_main_exits_nonzero_on_bind_failure(monkeypatch):
    class _Exec:
        def __init__(self, *a, **k):
            pass

        def add_node(self, n):
            pass

        def spin(self):
            pass

    monkeypatch.setattr(tn, 'MultiThreadedExecutor', _Exec)
    monkeypatch.setattr(tn.rclpy, 'shutdown', lambda *a, **k: None)
    real = tn.TrajectoryNode

    def make(*a, **k):
        n = real(command_pub_factory=_boom, start_emitter=False)
        n._emitter_loop()
        return n
    monkeypatch.setattr(tn, 'TrajectoryNode', make)
    with pytest.raises(SystemExit) as ei:
        tn.main()
    assert ei.value.code == 1


def test_main_exits_cleanly_when_emitter_ok(monkeypatch):
    class _Exec:
        def __init__(self, *a, **k):
            pass

        def add_node(self, n):
            pass

        def spin(self):
            pass

    monkeypatch.setattr(tn, 'MultiThreadedExecutor', _Exec)
    monkeypatch.setattr(tn.rclpy, 'shutdown', lambda *a, **k: None)
    real = tn.TrajectoryNode
    monkeypatch.setattr(
        tn, 'TrajectoryNode',
        lambda *a, **k: real(command_pub_factory=lambda: object(),
                             start_emitter=False))
    tn.main()   # no SystemExit


def _patch_node(monkeypatch, *, bind_inline, closes, shutdown_raises=False):
    """Real node whose segment-solver worker close() is recorded (on_shutdown
    is NOT stubbed)."""
    monkeypatch.setattr(tn.rclpy, 'shutdown', lambda *a, **k: None)
    real = tn.TrajectoryNode
    holder = []

    def make(*a, **k):
        n = real(command_pub_factory=_boom, start_emitter=False)
        if bind_inline:
            n._emitter_loop()
        orig = n._segment_solver.close

        def close():
            closes.append(1)
            orig()
            if shutdown_raises:
                raise RuntimeError('close failed')
        monkeypatch.setattr(n._segment_solver, 'close', close)
        holder.append(n)
        return n
    monkeypatch.setattr(tn, 'TrajectoryNode', make)
    return holder


def test_main_early_exit_when_bind_fails_before_executor(monkeypatch):
    """Real-Foxy ordering: the emitter thread fails to bind and shuts the
    context down inside the constructor, so building the executor raises.
    main() must clean up (worker close() really runs) and exit 1 without ever
    constructing it."""
    built = []
    closes = []

    class _Exec:
        def __init__(self, *a, **k):
            built.append(1)
            raise RuntimeError('context already shut down')

    monkeypatch.setattr(tn, 'MultiThreadedExecutor', _Exec)
    _patch_node(monkeypatch, bind_inline=True, closes=closes)
    with pytest.raises(SystemExit) as ei:
        tn.main()
    assert ei.value.code == 1
    assert built == []
    assert closes == [1]


def test_early_exit_still_destroys_node_when_on_shutdown_raises(monkeypatch):
    closes = []
    destroyed = []

    class _Exec:
        def __init__(self, *a, **k):
            raise AssertionError('executor must not be built')

    monkeypatch.setattr(tn, 'MultiThreadedExecutor', _Exec)
    holder = _patch_node(monkeypatch, bind_inline=True, closes=closes,
                         shutdown_raises=True)
    orig_make = tn.TrajectoryNode

    def make(*a, **k):
        n = orig_make(*a, **k)
        monkeypatch.setattr(n, 'destroy_node',
                            lambda: destroyed.append(1))
        return n
    monkeypatch.setattr(tn, 'TrajectoryNode', make)
    with pytest.raises(RuntimeError, match='close failed'):
        tn.main()
    assert closes == [1]
    assert destroyed == [1]


def test_bind_failure_during_spin_exits_nonzero_and_cleans_up(monkeypatch):
    """The bind lands after the early check (e.g. verdict-wait timeout): the
    executor is built and spins; the flag flips mid-spin and the context dies.
    main() must run cleanup and exit 1 via the post-spin path."""
    closes = []

    class _Exec:
        def __init__(self, *a, **k):
            pass

        def add_node(self, n):
            self.n = n

        def spin(self):
            self.n._emitter_loop()   # late bind failure while spinning

    monkeypatch.setattr(tn, 'MultiThreadedExecutor', _Exec)
    _patch_node(monkeypatch, bind_inline=False, closes=closes)
    with pytest.raises(SystemExit) as ei:
        tn.main()
    assert ei.value.code == 1
    assert closes == [1]


def test_late_bind_failure_killing_executor_construction_still_exits_1(monkeypatch):
    """Failure lands between the early check and executor construction: the
    constructor raises, cleanup still runs, exit is 1 (not a traceback)."""
    closes = []

    class _Exec:
        def __init__(self, *a, **k):
            holder[0]._emitter_loop()
            raise RuntimeError('context already shut down')

    monkeypatch.setattr(tn, 'MultiThreadedExecutor', _Exec)
    holder = _patch_node(monkeypatch, bind_inline=False, closes=closes)
    with pytest.raises(SystemExit) as ei:
        tn.main()
    assert ei.value.code == 1
    assert closes == [1]


def test_bind_done_event_set_on_both_outcomes(monkeypatch):
    monkeypatch.setattr(tn.rclpy, 'shutdown', lambda *a, **k: None)
    bad = tn.TrajectoryNode(command_pub_factory=_boom, start_emitter=False)
    assert not bad._emit_bind_done.is_set()
    bad._emitter_loop()
    assert bad._emit_bind_done.is_set()
