"""BUG-58: the bridge's opt-in ``gc_probe`` parameter installs the GC hook only
when true.  ROS 2 is mocked by tests/ros/conftest.py."""

from __future__ import annotations

import gc

from tests.ros._bridge_harness import _build_paired_node, _teardown


def test_gc_probe_off_by_default_installs_nothing():
    n0 = len(gc.callbacks)
    teensy, client, node = _build_paired_node()
    try:
        assert node._gc_probe is None
        assert len(gc.callbacks) == n0
    finally:
        _teardown(teensy, client, node)


def test_gc_probe_on_installs_hook_and_tick_logs(monkeypatch):
    from jugglebot.teensy_bridge_node import TeensyBridgeNode
    orig = TeensyBridgeNode.declare_parameter

    def declare(self, name, default_value=None, *a, **kw):
        if name == 'gc_probe':
            default_value = True
        return orig(self, name, default_value, *a, **kw)
    monkeypatch.setattr(TeensyBridgeNode, 'declare_parameter', declare)
    n0 = len(gc.callbacks)
    teensy, client, node = _build_paired_node()
    try:
        assert node._gc_probe is not None and node._gc_probe.installed
        assert len(gc.callbacks) == n0 + 1
        lines = []
        node.get_logger().info = lambda m, *a, **k: lines.append(str(m))
        node._gc_probe_tick()
        assert lines and lines[0].startswith('GC_PROBE teensy_bridge_node ')
    finally:
        node._gc_probe.uninstall()
        _teardown(teensy, client, node)
