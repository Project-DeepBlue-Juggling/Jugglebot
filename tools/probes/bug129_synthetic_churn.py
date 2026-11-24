#!/usr/bin/env python3
"""BUG-129 synthetic reproduction: can discovery churn alone freeze an rclpy publish() for >100 ms?

A bare Foxy rclpy node (NOT the bridge) publishes jugglebot RobotState at 100 Hz from a timer on a
MultiThreadedExecutor and times every publish() call. A driver then churns DDS discovery on the same
isolated domain: `ros2 topic echo /robot_state` started and killed (SIGKILL on even cycles, SIGINT on
odd ones), an extra pub+sub process SIGKILLed every 20 cycles (stale Fast DDS shm files accumulate),
the ros2 daemon restarted every 50 cycles. A HIT is any publish() over 100 ms. The run is repeated
with FASTRTPS_DEFAULT_PROFILES_FILE pointing the PUBLISHER (as the launch does for the bridge) at
ros_ws/src/jugglebot/resources/fastdds_bridge_udp_only.xml (no shm transport).

A NULL RESULT IS UNINFORMATIVE about the real stack: this node has none of the bridge's 14+ other
threads, GIL load, 1.6 kHz IMU traffic, rosbag subscriber or CAN-side work, so it can only say
"discovery churn alone, on an otherwise idle node, does / does not freeze publish()".

Safety: ROS_DOMAIN_ID=142 + ROS_LOCALHOST_ONLY=1, no hardware, no jugglebot node. The processes are
named bug129_synthetic_churn (no robot-shaped argv, so tools/lib/live_session.sh stays "clear").

Run (Foxy env then venv python, both 3.8):
  bash -c 'source /opt/ros/foxy/setup.bash && source ~/Desktop/Jugglebot/ros_ws/install/setup.bash &&
           source ~/Desktop/PDJ_venv/venv/bin/activate &&
           python tools/probes/bug129_synthetic_churn.py --out DIR [--cycles 300] [--budget-s 1200]'
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time

HIT_MS = 100.0
DOMAIN = '142'
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
UDP_PROFILE = os.path.join(REPO, 'ros_ws', 'src', 'jugglebot', 'resources', 'fastdds_bridge_udp_only.xml')


# ── role: publisher node ─────────────────────────────────────────────────────────────────────
def run_node(out_path: str) -> int:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from jugglebot_interfaces.msg import RobotState

    rclpy.init()
    node = Node('bug129_synth_pub')
    pub = node.create_publisher(RobotState, '/robot_state', 10)
    msg = RobotState()
    stats = {'n': 0, 'max_ms': 0.0, 'over_20ms': 0, 'over_50ms': 0, 'hits': [], 'timer_gap_max_ms': 0.0}
    last = [None]
    stop = threading.Event()

    def tick():
        now = time.perf_counter()
        if last[0] is not None:
            stats['timer_gap_max_ms'] = max(stats['timer_gap_max_ms'], (now - last[0]) * 1e3)
        last[0] = now
        t = time.perf_counter()
        pub.publish(msg)
        dt = (time.perf_counter() - t) * 1e3
        stats['n'] += 1
        stats['max_ms'] = max(stats['max_ms'], dt)
        if dt > 20:
            stats['over_20ms'] += 1
        if dt > 50:
            stats['over_50ms'] += 1
        if dt > HIT_MS:
            stats['hits'].append({'wall': time.time(), 'publish_ms': round(dt, 1), 'load1': os.getloadavg()[0]})

    node.create_timer(0.01, tick)

    def writer():
        while not stop.wait(2.0):
            with open(out_path + '.tmp', 'w') as f:
                json.dump(stats, f)
            os.replace(out_path + '.tmp', out_path)

    threading.Thread(target=writer, daemon=True).start()
    signal.signal(signal.SIGTERM, lambda *_: _term(stats, out_path))
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    _term(stats, out_path, exit_now=False)
    return 0


def _term(stats, out_path, exit_now=True):
    with open(out_path, 'w') as f:
        json.dump(stats, f)
    if exit_now:
        os._exit(0)


# ── role: extra pub+sub process (to be SIGKILLed) ────────────────────────────────────────────
def run_extra() -> int:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    n = Node('bug129_synth_extra')
    p = n.create_publisher(String, '/bug129_extra', 10)
    n.create_subscription(String, '/bug129_extra', lambda m: None, 10)
    n.create_timer(0.05, lambda: p.publish(String(data='x')))
    rclpy.spin(n)
    return 0


# ── driver ───────────────────────────────────────────────────────────────────────────────────
def _env(profile: str | None) -> dict:
    e = dict(os.environ)
    e['ROS_DOMAIN_ID'] = DOMAIN
    e['ROS_LOCALHOST_ONLY'] = '1'
    e.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    if profile:
        e['FASTRTPS_DEFAULT_PROFILES_FILE'] = profile
    return e


def _sh(cmd: str, env: dict, timeout: float = 30.0) -> int:
    try:
        return subprocess.run(['bash', '-c', cmd], env=env, stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, timeout=timeout).returncode
    except subprocess.TimeoutExpired:
        return -1


def live_session_state() -> str:
    live = os.path.join(REPO, 'tools', 'lib', 'live_session.sh')
    r = subprocess.run(['bash', '-c', 'source "%s"; live_session && echo LIVE || echo clear' % live],
                       capture_output=True, text=True)
    return r.stdout.strip()


def shm_count() -> int:
    try:
        return len(os.listdir('/dev/shm'))
    except OSError:
        return -1


def wait_for_quiet(max_wait_s: float, limit: float = 6.0) -> float:
    """Wait while the 1-min load average is above limit; return the load at the end."""
    t0 = time.time()
    while True:
        load = os.getloadavg()[0]
        if load <= limit or time.time() - t0 >= max_wait_s:
            return load
        time.sleep(15)


def run_arm(name: str, profile: str | None, cycles: int, budget_s: float, out_dir: str,
            alive_s: float, check_live: bool) -> dict:
    env = _env(profile)
    node_json = os.path.join(out_dir, 'node_%s.json' % name)
    events = open(os.path.join(out_dir, 'events_%s.jsonl' % name), 'w')
    res = {'arm': name, 'profile': profile, 'cycles_done': 0, 'live_session': None,
           'shm_before': shm_count(), 'load_start': os.getloadavg()[0]}
    _sh('ros2 daemon stop', env)
    node = subprocess.Popen([sys.executable, os.path.abspath(__file__), '--role', 'node', '--node-out', node_json],
                            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    t_start = time.time()
    try:
        time.sleep(4.0)
        if check_live:
            res['live_session'] = live_session_state()
        for i in range(cycles):
            if time.time() - t_start > budget_s:
                break
            sig = signal.SIGKILL if i % 2 == 0 else signal.SIGINT
            p = subprocess.Popen(['ros2', 'topic', 'echo', '/robot_state'], env=env, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL, start_new_session=True)
            time.sleep(alive_s)
            os.killpg(p.pid, sig)
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
            events.write(json.dumps({'cycle': i, 'wall': time.time(), 'sig': signal.Signals(sig).name}) + '\n')
            if i % 20 == 19:
                x = subprocess.Popen([sys.executable, os.path.abspath(__file__), '--role', 'extra'], env=env,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                time.sleep(3.0)
                os.killpg(x.pid, signal.SIGKILL)
                x.wait()
                events.write(json.dumps({'cycle': i, 'wall': time.time(), 'extra_pubsub_sigkill': True}) + '\n')
            if i % 50 == 49:
                _sh('ros2 daemon stop', env)
                _sh('ros2 daemon start', env)
                events.write(json.dumps({'cycle': i, 'wall': time.time(), 'daemon_restart': True}) + '\n')
            events.flush()
            res['cycles_done'] = i + 1
            time.sleep(0.3)
    finally:
        try:
            os.killpg(node.pid, signal.SIGTERM)
            node.wait(timeout=10)
        except Exception:  # noqa: BLE001
            try:
                os.killpg(node.pid, signal.SIGKILL)
            except Exception:  # noqa: BLE001
                pass
        _sh('ros2 daemon stop', env)
        events.close()
    try:
        with open(node_json) as f:
            res['node'] = json.load(f)
    except Exception as e:  # noqa: BLE001
        res['node'] = {'error': repr(e)}
    res['wall_s'] = round(time.time() - t_start, 1)
    res['shm_after'] = shm_count()
    res['load_end'] = os.getloadavg()[0]
    n = res['node']
    res['hit'] = bool(n.get('hits'))
    return res


def cleanup_shm() -> str:
    import importlib.util
    path = os.path.join(REPO, 'ros_ws', 'src', 'jugglebot', 'jugglebot', 'dds_shm.py')
    spec = importlib.util.spec_from_file_location('dds_shm_probe', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m.summary_line(m.clean_stale_shm())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--role', choices=['driver', 'node', 'extra'], default='driver')
    ap.add_argument('--node-out')
    ap.add_argument('--out', help='output directory (driver)')
    ap.add_argument('--cycles', type=int, default=300)
    ap.add_argument('--budget-s', type=float, default=1200.0, help='wall-clock cap PER ARM')
    ap.add_argument('--alive-s', type=float, default=2.5, help='seconds each echo lives before the signal')
    ap.add_argument('--quiet-wait-s', type=float, default=1800.0)
    ap.add_argument('--arms', default='default,udp')
    a = ap.parse_args()
    if a.role == 'node':
        return run_node(a.node_out)
    if a.role == 'extra':
        return run_extra()
    os.makedirs(a.out, exist_ok=True)
    load = wait_for_quiet(a.quiet_wait_s)
    out = {'started': time.strftime('%Y-%m-%d %H:%M:%S'), 'load_at_start': load, 'cycles': a.cycles,
           'hit_threshold_ms': HIT_MS, 'domain': DOMAIN, 'arms': []}
    first = True
    for arm in a.arms.split(','):
        out['arms'].append(run_arm(arm, UDP_PROFILE if arm == 'udp' else None, a.cycles, a.budget_s, a.out,
                                   a.alive_s, check_live=first))
        first = False
        print(json.dumps(out['arms'][-1]), flush=True)
    out['shm_cleanup'] = cleanup_shm()
    with open(os.path.join(a.out, 'summary.json'), 'w') as f:
        json.dump(out, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
