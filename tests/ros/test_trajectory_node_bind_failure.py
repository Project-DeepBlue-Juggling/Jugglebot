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
