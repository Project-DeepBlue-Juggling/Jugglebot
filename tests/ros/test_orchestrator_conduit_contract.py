"""Runtime drift-guard for the orchestrator ↔ bridge conduit.

The bridge (``teensy_bridge_node``) is the drop-in successor to ``can_node`` from
the LOCKED ``orchestrator_node``'s view. The bridge wires the four cold-start
interfaces the orchestrator drives — the ``home_motors`` action, the
``activate_or_deactivate`` / ``get_platform_tilt`` services, and the
``set_level_state`` subscriber — onto the bridge, with ZERO edits to the
orchestrator / state machine (a locked design decision).

This test is the CONTRACT that keeps them aligned: it constructs BOTH nodes and
asserts, by ``(name, type)``, that the bridge SERVES every service/action the
orchestrator declares as a client, and that the ``robot_state`` / ``set_level_state``
topics agree in both directions. A future orchestrator interface rename (or type
change) fails THIS test, not the powered bench.

Why introspection is deterministic here: the mock rclpy layer
(``tests/ros/conftest.py``) records ``(srv_name, srv_type)`` on each service/client,
``(topic_name, msg_type)`` on each publisher/subscription, and registers each
action's ``(name, type)`` on the node — so both nodes' declared interface sets are
readable offline, with no DDS discovery and no executor spin. Both nodes import the
SAME mock interface classes, so ``srv_type`` / ``action_type`` compare by identity.
"""

from __future__ import annotations

from jugglebot_interfaces.msg import RobotState
from jugglebot_interfaces.action import Reload
from std_msgs.msg import Float64MultiArray

from tests.ros.test_teensy_bridge_node_read import _build_paired_node


# Orchestrator action clients that are DELIBERATELY not bridge-served — they
# target a different node, so the bridge-drift contract below correctly excludes
# them. jugglebot/reload is served by reload_coordinator_node (the GUI reaches it
# through orchestrator_node's jugglebot/reload_request relay; rosbridge on Foxy has
# no action transport). This is a scoping of the contract to its actual root cause
# (orchestrator↔bridge drift), NOT a weakening: every bridge-conduit action client
# is still fully guarded.
_NON_BRIDGE_ORCH_ACTIONS = {('jugglebot/reload', Reload)}


def _teardown(teensy, client, node):
    node.on_shutdown()
    client.stop()
    teensy.stop()


def _orch_service_clients(orch):
    return {(c.srv_name, c.srv_type) for c in orch._clients.values()}


def _orch_action_clients(orch):
    return {(a.action_name, a.action_type) for a in orch._action_clients.values()}


def _bridge_services(node):
    return {(s.srv_name, s.srv_type) for s in node._services.values()}


def _bridge_actions(node):
    return {(a.action_name, a.action_type) for a in node._action_servers.values()}


def _bridge_pubs(node):
    return {(p.topic_name, p.msg_type) for p in node._publishers.values()}


def _bridge_subs(node):
    return {(s.topic_name, s.msg_type) for s in node._subscriptions.values()}


def _build_both():
    """Construct OrchestratorNode (mock rclpy) + a paired TeensyBridgeNode."""
    from jugglebot.orchestrator_node import OrchestratorNode
    orch = OrchestratorNode()
    teensy, client, bridge = _build_paired_node(boot_state_read=False)
    return orch, teensy, client, bridge


# ── The core drift-guard: bridge serves every orchestrator-declared client ──

def test_bridge_serves_every_orchestrator_service_client():
    """Every service the orchestrator CLIENTS must be SERVED by the bridge with a
    matching (name, type). This is the whole cold-start conduit contract for
    services: encoder_search / odrive_command / bb/calibrate (pre-existing) +
    activate_or_deactivate / get_platform_tilt (added by the conduit)."""
    orch, teensy, client, bridge = _build_both()
    try:
        missing = _orch_service_clients(orch) - _bridge_services(bridge)
        assert not missing, (
            f"bridge does not serve orchestrator service clients (name,type): {missing}")
    finally:
        _teardown(teensy, client, bridge)


def test_bridge_serves_every_orchestrator_action_client():
    """Every BRIDGE-CONDUIT action client the orchestrator declares must have a
    matching bridge ActionServer (name, type) — currently home_motors. Action
    clients targeting other nodes (jugglebot/reload → reload_coordinator_node) are
    excluded via _NON_BRIDGE_ORCH_ACTIONS: they are outside this contract's root
    cause (orchestrator↔bridge drift)."""
    orch, teensy, client, bridge = _build_both()
    try:
        missing = (_orch_action_clients(orch)
                   - _bridge_actions(bridge)
                   - _NON_BRIDGE_ORCH_ACTIONS)
        assert not missing, (
            f"bridge does not serve orchestrator action clients (name,type): {missing}")
    finally:
        _teardown(teensy, client, bridge)


def test_reload_action_client_is_not_bridge_served():
    """jugglebot/reload is the orchestrator's one NON-bridge action client: the GUI
    reload relay (jugglebot/reload_request Trigger → this client) targets
    reload_coordinator_node, NOT the bridge. Guards against someone wiring the reload
    onto the bridge by mistake, and documents that the exemption above is real."""
    orch, teensy, client, bridge = _build_both()
    try:
        assert ('jugglebot/reload', Reload) in _orch_action_clients(orch)
        assert ('jugglebot/reload', Reload) not in _bridge_actions(bridge)
    finally:
        _teardown(teensy, client, bridge)


# ── Explicit per-interface presence (clear failure messages) ──

def test_home_motors_action_present():
    from jugglebot_interfaces.action import HomeMotors
    orch, teensy, client, bridge = _build_both()
    try:
        assert ('home_motors', HomeMotors) in _bridge_actions(bridge)
        # And the orchestrator declares exactly this action client.
        assert ('home_motors', HomeMotors) in _orch_action_clients(orch)
    finally:
        _teardown(teensy, client, bridge)


def test_activate_or_deactivate_service_present():
    from jugglebot_interfaces.srv import ActivateOrDeactivate
    orch, teensy, client, bridge = _build_both()
    try:
        assert ('activate_or_deactivate', ActivateOrDeactivate) in _bridge_services(bridge)
        assert ('activate_or_deactivate', ActivateOrDeactivate) in _orch_service_clients(orch)
    finally:
        _teardown(teensy, client, bridge)


def test_get_platform_tilt_service_present():
    from jugglebot_interfaces.srv import GetTiltReadingService
    orch, teensy, client, bridge = _build_both()
    try:
        assert ('get_platform_tilt', GetTiltReadingService) in _bridge_services(bridge)
        assert ('get_platform_tilt', GetTiltReadingService) in _orch_service_clients(orch)
    finally:
        _teardown(teensy, client, bridge)


# ── Topic agreement (both directions) ──

def test_robot_state_topic_agrees():
    """The orchestrator subscribes robot_state; the bridge publishes it (RobotState)."""
    orch, teensy, client, bridge = _build_both()
    try:
        assert ('robot_state', RobotState) in _bridge_pubs(bridge)
        assert 'robot_state' in orch._subscriptions
    finally:
        _teardown(teensy, client, bridge)


def test_set_level_state_topic_agrees():
    """The orchestrator publishes set_level_state (Float64MultiArray); the bridge
    subscribes to it (the conduit adds this — the gravity-offset persist was silently discarded
    before)."""
    orch, teensy, client, bridge = _build_both()
    try:
        assert ('set_level_state', Float64MultiArray) in _bridge_subs(bridge)
        assert 'set_level_state' in orch._publishers
    finally:
        _teardown(teensy, client, bridge)
