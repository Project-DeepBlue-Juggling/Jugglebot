#!/usr/bin/env python3
"""Probe: service RESPONSES starve when every MultiThreadedExecutor thread is
blocked (skill_node, sitting 7 re-fly 2026-10-11).

Run with the SYSTEM python and ROS sourced (NOT the project venv):
    bash -c 'source /opt/ros/foxy/setup.bash && python3 tools/probes/skill_node_response_starvation_probe.py'

A node on MultiThreadedExecutor(num_threads=4) has 4 callbacks parked in
time.sleep (stand-ins for `_wait_future` polls / `_tick_lock` waits) plus a
stub service. (a) A client on the SAME node: its response needs a pool thread,
so the future does not complete while the blockers hold. (b) A client on a
helper node spun by its own SingleThreadedExecutor thread: completes fast.
(The stub service REQUEST also needs a pool thread, so the server lives on a
SEPARATE node/executor -- as trajectory_node / ball_butler_node do in prod.)
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

BLOCK_S = 4.0


def main():
    rclpy.init()
    server = Node('probe_server')
    server.create_service(Trigger, 'probe/svc',
                          lambda req, resp: (setattr(resp, 'success', True), resp)[1])
    server_ex = SingleThreadedExecutor()
    server_ex.add_node(server)
    threading.Thread(target=server_ex.spin, daemon=True).start()

    victim = Node('probe_victim')
    helper = Node('probe_victim_clients')
    grp = ReentrantCallbackGroup()
    same_cli = victim.create_client(Trigger, 'probe/svc', callback_group=grp)
    helper_cli = helper.create_client(Trigger, 'probe/svc')

    def blocker():
        time.sleep(BLOCK_S)
    for _ in range(4):
        victim.create_timer(0.01, blocker, callback_group=grp)
    pool = MultiThreadedExecutor(num_threads=4)
    pool.add_node(victim)
    threading.Thread(target=pool.spin, daemon=True).start()

    helper_ex = SingleThreadedExecutor()
    helper_ex.add_node(helper)
    threading.Thread(target=helper_ex.spin, daemon=True).start()

    assert same_cli.wait_for_service(timeout_sec=5.0)
    assert helper_cli.wait_for_service(timeout_sec=5.0)
    time.sleep(0.5)          # let all four pool threads park in blocker()

    def timed(cli, limit):
        t0 = time.perf_counter()
        fut = cli.call_async(Trigger.Request())
        while not fut.done() and time.perf_counter() - t0 < limit:
            time.sleep(0.002)
        return fut.done(), (time.perf_counter() - t0) * 1e3

    done_a, ms_a = timed(same_cli, 1.0)
    done_b, ms_b = timed(helper_cli, 1.0)
    print('(a) same-node client  (pool threads all blocked): done=%s after %.0f ms'
          % (done_a, ms_a))
    print('(b) helper-node client (own executor thread)    : done=%s after %.0f ms'
          % (done_b, ms_b))
    ok = (not done_a) and done_b and ms_b < 100.0
    print('RESULT: %s' % ('REPRODUCED + FIX CONFIRMED' if ok else 'UNEXPECTED'))
    import os, sys
    sys.stdout.flush()
    os._exit(0 if ok else 1)     # daemon spin threads would trace on shutdown


if __name__ == '__main__':
    main()
