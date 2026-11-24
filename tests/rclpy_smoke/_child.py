"""Child-process entry points for the real-rclpy smoke tests (never imported by pytest).

Run as ``python _child.py <mode> [args]`` inside a sourced Foxy environment, from
tests/rclpy_smoke/test_rclpy_smoke.py. Modes:

  catch                      run CatchCorrelationNode's real main()
  bridge                     run teensy_bridge_node's real main(), with the transport
                             client FORCED onto loopback (see below)
  probe TOPIC TIMEOUT        exit 0 once TOPIC (or SERVICE) shows up in the graph
  homegoal TIMEOUT           send one home_motors goal, exit 0 once it is accepted

SAFETY (the real can-bridge board is on this box's network): the bridge node builds
its own TeensyLinkClient from the config constants (peer 192.168.x.x:5005/5006,
local bind 0.0.0.0:5005/5006). This file replaces that constructor with one that
ignores every address the node passes and uses the loopback peer ports in
JUGGLEBOT_SMOKE_STREAM_PORT / JUGGLEBOT_SMOKE_RPC_PORT, binding 127.0.0.1 on
ephemeral local ports. If those variables are missing or not loopback the child
exits 99 before any socket exists.
"""
from __future__ import annotations

import os
import sys
import time


def _run_bridge() -> None:
    try:
        stream_port = int(os.environ['JUGGLEBOT_SMOKE_STREAM_PORT'])
        rpc_port = int(os.environ['JUGGLEBOT_SMOKE_RPC_PORT'])
    except (KeyError, ValueError):
        sys.exit(99)
    if os.environ.get('JUGGLEBOT_SMOKE_PEER_HOST') != '127.0.0.1':
        sys.exit(99)
    import faulthandler
    import signal
    # ptrace_scope=1 blocks py-spy from a non-parent: the test sends SIGUSR1 for an
    # in-process all-thread stack dump (to stderr, which the test captures).
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    import jugglebot.teensy_bridge_node as tbn
    real = tbn.TeensyLinkClient

    def loopback_only(*_a, **_k):
        return real(teensy_addr=('127.0.0.1', stream_port), rpc_port=rpc_port,
                    local_bind_stream=0, local_bind_rpc=0, bind_host='127.0.0.1')

    tbn.TeensyLinkClient = loopback_only
    tbn.main(['--ros-args', '-p', 'teensy_ip:=127.0.0.1'])


def _probe(name: str, timeout: float) -> int:
    import rclpy
    rclpy.init()
    node = rclpy.create_node('smoke_probe')
    want = '/' + name.lstrip('/')
    end = time.monotonic() + timeout
    try:
        while time.monotonic() < end:
            names = {n for n, _ in node.get_topic_names_and_types()}
            names |= {n for n, _ in node.get_service_names_and_types()}
            if want in names:
                return 0
            time.sleep(0.2)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _homegoal(timeout: float) -> int:
    import rclpy
    from rclpy.action import ActionClient
    from jugglebot_interfaces.action import HomeMotors
    rclpy.init()
    node = rclpy.create_node('smoke_home_client')
    client = ActionClient(node, HomeMotors, 'home_motors')
    try:
        if not client.wait_for_server(timeout_sec=timeout):
            return 2
        fut = client.send_goal_async(HomeMotors.Goal())
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0 if fut.done() and fut.result().accepted else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    mode = sys.argv[1]
    if mode == 'catch':
        from jugglebot.catch_correlation_node import main
        main()
    elif mode == 'bridge':
        _run_bridge()
    elif mode == 'probe':
        sys.exit(_probe(sys.argv[2], float(sys.argv[3])))
    elif mode == 'homegoal':
        sys.exit(_homegoal(float(sys.argv[2])))
    else:
        sys.exit(98)
