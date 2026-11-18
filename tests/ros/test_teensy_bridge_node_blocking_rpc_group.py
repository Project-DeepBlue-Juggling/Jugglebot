"""BUG-98: services/timers that block in _call_rpc must not share a callback group
with the 100 Hz publishers or the 1 Hz link-loss _health_check.

The node runs a MultiThreadedExecutor (main()), so the node-default
MutuallyExclusiveCallbackGroup is what serializes _publish_robot_state,
_publish_link_status and _health_check with anything else left in it. A callback
parked in _call_rpc (0.5 s x 3 attempts by default) therefore stalls all of them
unless it lives in another group. The mocked rclpy (tests/ros/conftest.py) records
each service's ``callback_group`` (None = node default); create_timer records
nothing, so the timer test wraps it on the class before construction.
"""

from __future__ import annotations

from tests.ros._bridge_harness import _build_paired_node, _teardown

# Services whose handler reaches _call_rpc directly (or via _reboot_odrives) and
# that are quick-but-blocking, not multi-second moves.
BLOCKING_SERVICES = (
    'reboot_odrives', 'set_hand_state', 'set_hand_gains',
    'bb/reload', 'bb/reset', 'bb/calibrate',
)
PUBLISHER_TIMERS = ('_publish_robot_state', '_publish_link_status', '_health_check')
RPC_TIMERS = ('_version_check_poll', '_bb_version_check_poll')


def test_blocking_services_are_not_in_the_default_group():
    teensy, client, node = _build_paired_node()
    try:
        svcs = node._services
        grp = node._blocking_rpc_cbgroup
        assert grp is not None
        for name in BLOCKING_SERVICES:
            assert name in svcs, name
            assert svcs[name].callback_group is grp, name
        # odrive_command stays in the /recover group (it reroutes clear_errors).
        assert svcs['odrive_command'].callback_group is node._recover_cbgroup
    finally:
        _teardown(teensy, client, node)


def test_no_service_is_left_in_the_default_group():
    """The class pin: any NEW service lands in a non-default group or is named
    here on purpose. An empty allowlist today."""
    teensy, client, node = _build_paired_node()
    try:
        in_default = sorted(n for n, s in node._services.items()
                            if s.callback_group is None)
        assert in_default == [], in_default
    finally:
        _teardown(teensy, client, node)


def test_rpc_timers_are_off_the_publishers_group(monkeypatch):
    from jugglebot.teensy_bridge_node import TeensyBridgeNode

    seen = {}
    orig = TeensyBridgeNode.create_timer

    def _rec(self, period, callback, *a, **kw):
        seen[callback.__name__] = kw.get('callback_group')
        return orig(self, period, callback, *a, **kw)

    monkeypatch.setattr(TeensyBridgeNode, 'create_timer', _rec)
    teensy, client, node = _build_paired_node()
    try:
        for name in PUBLISHER_TIMERS:
            assert name in seen, name
            assert seen[name] is None, name          # node default group
        for name in RPC_TIMERS:
            assert seen[name] is node._blocking_rpc_cbgroup, name
    finally:
        _teardown(teensy, client, node)
