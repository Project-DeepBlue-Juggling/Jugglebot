#!/usr/bin/env python3
"""
rosbridge_cpu_probe.py — CPU probe for the GUI's rosbridge (NO hardware, NO
real robot nodes; runs entirely on a private ROS_DOMAIN_ID).

Characterises the two fixes in `rosbridge_websocket_lean.py` (a leaked
service-client destroyed on the executor thread; a dedicated spin thread
replacing stock rosbridge_server's 1 ms tornado timer) and the GUI's
`compression: 'cbor-raw'` raw-subscription spies, against a synthetic
publisher that mimics the live launch's topic census. Promotes four scratch
prototypes validated 2026-09-13 (`pub_load.py`, `ws_client.py`,
`run_load_probe.sh`, `leak_client.py`) into one committed, self-contained
tool. Motivating logbook entry:
`logbook/2026-09-14-rosbridge-cpu-leak-and-spin.md`.

CONTAINMENT (safety rails, enforced in code — same pattern as
`gui_synthetic_stack.py`'s, adapted for a one-shot batch-scenario runner
rather than a persistent stdin-driven server, PLUS an extra rail:
`gui_synthetic_stack.py` runs on the default/robot ROS domain and relies on
the graph-empty check + watchdog alone; this probe additionally runs on a
private, non-default `ROS_DOMAIN_ID` (87), so even a race between the
graph-empty check and a publisher coming up cannot reach a real robot node
living on domain 0):
  * Sets `ROS_DOMAIN_ID` itself (default 87) in the environment of every
    child process it spawns, and REFUSES to run on domain 0.
  * REFUSES to start if that domain's ROS graph already has any (non-hidden)
    node in it — a real stack, or a leftover probe, might be using it.
  * A ~1 Hz watchdog re-checks the graph for the whole run: any foreign node
    appearing mid-run kills every child process group immediately and exits
    non-zero (mirrors `gui_synthetic_stack._containment_watchdog`, but a
    batch tool has no persistent state to degrade — it tears down outright).
  * Publishes ONLY synthetic telemetry topic names copied from the live
    launch's idle census (robot_state, hand_telemetry, leg_cmd_executed,
    bb/axis_estimates, orchestrator_state, control_mode_topic, link_status,
    a handful of probe_diag* names, gui/robot_state, gui/hand_telemetry) —
    never a command topic.
  * SIGINT/SIGTERM/atexit teardown kills every spawned process GROUP
    (SIGINT, then SIGKILL after an 8 s grace period).

Self-contained: sources `/opt/ros/foxy/setup.bash`, then
`Jugglebot-skills/ros_ws/install/setup.bash` (for `jugglebot_interfaces`),
then the project venv, via a one-time re-exec — so a bare
`python3 tools/probes/rosbridge_cpu_probe.py ...` works even if the caller
forgot to source anything. All children inherit that sourced environment.

Subcommands:
  load    --server {stock,lean} --client {none,gui,rawspy,nospy}
          [--seconds 35] [--warm 8] [--port 9391] [--domain 87]
      Synthetic publisher + the chosen rosbridge server + the chosen
      websocket client. Reports mean server %CPU (after warm-up), publisher
      %CPU, and the client's received Hz per topic (via `pidstat -u -p
      <server_pid>,<publisher_pid> 1 N`).

  leak    --server {stock,lean} [--calls 1200] [--port 9391] [--domain 87]
          [--idle-seconds 20]
      Server + a real `rosapi_node`. Idle %CPU for `--idle-seconds`, then
      `--calls` sequential `/rosapi/topics` service calls (latency mean of
      the first 50 and last 50 calls), then idle %CPU again.

  churn   [--server lean] [--cycles 50] [--hold 2.0] [--steady-seconds 20]
          [--port 9391] [--domain 87]
      D2 acceptance recipe: the publisher runs throughout; `--cycles` times,
      a client connects, subscribes the full GUI set (spies included), holds
      `--hold` seconds, disconnects — then one steady client subscribes and
      listens for `--steady-seconds`. Reports whether the server stayed
      alive, the final client's received Hz (compare against `load`'s), and
      counts `rosbridge_spin: unhandled exception` lines (+ exception types)
      in the server log.

  compare --scenario {load,leak} [same flags as the scenario] [--domain 87]
      Runs stock then lean back-to-back in ONE invocation (so load drift
      between separate runs cannot masquerade as a saving). Prints `uptime`
      at the start and end.

Outputs: a timestamped directory per invocation under
`temp/probes/rosbridge_cpu/` holding pidstat/server/client/publisher logs.
Run from anywhere; the repo root is derived from `__file__`.
"""
from __future__ import annotations

import argparse
import atexit
import collections
import json
import os
import re
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT_DIR = os.path.join(REPO_ROOT, 'temp', 'probes', 'rosbridge_cpu')
THIS_FILE = os.path.abspath(__file__)

DEFAULT_PORT = 9391
DEFAULT_DOMAIN = 87
_SOURCED_SENTINEL = '_ROSBRIDGE_CPU_PROBE_SOURCED'

# Nodes legitimately in the graph while a scenario runs: ourselves, our
# publisher, and whichever server + rosapi we launched. rosapi_params is
# spawned internally by Foxy's rosapi_node (see gui_synthetic_stack.py's
# note on the same). Hidden nodes (leading underscore, e.g. the ros2cli
# daemon) are tolerated below, not listed here.
ALLOWED_NODES = {'rosbridge_cpu_probe', 'rosbridge_cpu_probe_pub',
                 'rosbridge_websocket', 'rosapi', 'rosapi_params'}

# rosbridge subscribe sets mimicking the GUI (ros_ws/gui/js/main.js:297-347,
# 1087), lifted from the validated ws_client.py prototype.
_DIAG = 'diagnostic_msgs/msg/DiagnosticStatus'
FULL_LO = [('/orchestrator_state', 'std_msgs/msg/String', 0),
           ('/control_mode_topic', 'std_msgs/msg/String', 0),
           ('/link_status', _DIAG, 0),
           ('/probe_diag10_a', _DIAG, 0),
           ('/probe_diag10_b', _DIAG, 0)]
FULL_HI = [('/robot_state', 'jugglebot_interfaces/msg/RobotState', 50),
           ('/hand_telemetry', 'jugglebot_interfaces/msg/HandTelemetryMessage', 100)]
COPIES = [('/gui/robot_state', 'jugglebot_interfaces/msg/RobotState', 0),
          ('/gui/hand_telemetry', 'jugglebot_interfaces/msg/HandTelemetryMessage', 0)]
SPY_HI = [('/leg_cmd_executed', 'sensor_msgs/msg/JointState'),
          ('/bb/axis_estimates', 'sensor_msgs/msg/JointState')]
SPY_LO = [('/probe_diag5_a', _DIAG), ('/probe_diag5_b', _DIAG)]


def log(msg: str) -> None:
    sys.stderr.write('[rosbridge_cpu_probe] %s\n' % msg)
    sys.stderr.flush()


# ---------------------------------------------------------------------------
# Self-contained environment sourcing
# ---------------------------------------------------------------------------

def _maybe_reexec_with_ros_env() -> None:
    """Re-exec once under `bash -c 'source ... && source ... && source ... &&
    exec python3 <original argv>'` so this probe is runnable as a bare
    `python3 tools/probes/rosbridge_cpu_probe.py ...` (the README's
    "Self-contained" convention), without the caller having to remember the
    three-file source chain. Guarded by an env sentinel so a spawned child
    (which inherits the already-sourced environment) does not re-source."""
    if os.environ.get(_SOURCED_SENTINEL) == '1':
        return
    setup_files = [
        '/opt/ros/foxy/setup.bash',
        '/home/jetson/Desktop/Jugglebot-skills/ros_ws/install/setup.bash',
        '/home/jetson/Desktop/PDJ_venv/venv/bin/activate',
    ]
    missing = [p for p in setup_files if not os.path.isfile(p)]
    if missing:
        log('WARNING: expected setup file(s) missing (%s) -- continuing with '
            'the inherited environment as-is.' % missing)
        os.environ[_SOURCED_SENTINEL] = '1'
        return
    source_chain = ' && '.join('source %s' % shlex.quote(p) for p in setup_files)
    argv = ' '.join(shlex.quote(a) for a in sys.argv)
    env = os.environ.copy()
    env[_SOURCED_SENTINEL] = '1'
    os.execvpe('bash', ['bash', '-c', '%s && exec python3 %s' % (source_chain, argv)], env)


# ---------------------------------------------------------------------------
# Safety rails
# ---------------------------------------------------------------------------

def set_domain(domain: int) -> None:
    if domain == 0:
        raise SystemExit(
            'refusing to run on ROS_DOMAIN_ID 0 (the robot\'s live domain) -- '
            'pass --domain with a private id (default 87).')
    os.environ['ROS_DOMAIN_ID'] = str(domain)


def check_graph_empty(ctrl) -> None:
    """REFUSE to start if any other real node is up on this domain. Two
    discovery samples 1.5 s apart, union — same recipe as
    `gui_synthetic_stack.check_graph_empty`."""
    seen = set()
    for _ in range(2):
        time.sleep(1.5)
        seen.update(ctrl.get_node_names())
    others = sorted(n for n in seen if n != ctrl.get_name() and not n.startswith('_'))
    if others:
        raise SystemExit(
            'ROS graph on domain %s is NOT empty: %s -- refusing to start. '
            'Shut down whatever is on this domain first.'
            % (os.environ.get('ROS_DOMAIN_ID'), ', '.join(others)))


def containment_watchdog(ctrl, stop_event: threading.Event) -> None:
    """~1 Hz: any foreign node in the graph => kill every child process
    group and exit non-zero. A batch tool has no persistent state to
    degrade gracefully (contrast `gui_synthetic_stack`'s per-topic disable),
    so containment here means "stop everything now"."""
    while not stop_event.wait(1.0):
        foreign = sorted(n for n in set(ctrl.get_node_names())
                          if n not in ALLOWED_NODES and n != ctrl.get_name()
                          and not n.startswith('_'))
        if foreign:
            log('FATAL: containment breach -- foreign node(s) appeared in the '
                'graph mid-run: %s. Killing every child and exiting.' % foreign)
            teardown()
            os._exit(1)


# ---------------------------------------------------------------------------
# Child process management
# ---------------------------------------------------------------------------

_children = []  # (name, Popen, filehandle-or-None)
_torn_down = False


def spawn(name, cmd, env_extra=None, out_path=None):
    env = os.environ.copy()
    if env_extra:
        env.update(env_extra)
    fh = open(out_path, 'w') if out_path else None
    proc = subprocess.Popen(cmd, stdout=(fh or subprocess.DEVNULL),
                             stderr=subprocess.STDOUT, env=env, start_new_session=True)
    _children.append((name, proc, fh))
    log('launched %s (pid %d): %s' % (name, proc.pid, ' '.join(cmd)))
    return proc


def teardown() -> None:
    global _torn_down
    if _torn_down:
        return
    _torn_down = True
    for name, proc, _fh in _children:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except OSError:
                pass
    deadline = time.time() + 8.0
    for name, proc, fh in _children:
        while proc.poll() is None and time.time() < deadline:
            time.sleep(0.2)
        if proc.poll() is None:
            log('SIGKILL %s (pid %d)' % (name, proc.pid))
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                pass
        proc.wait()
        if fh:
            fh.close()
        log('%s exited rc=%s' % (name, proc.returncode))


atexit.register(teardown)
for _sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(_sig, lambda *_a: sys.exit(1))


def wait_port(port: int, timeout: float) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        try:
            if s.connect_ex(('127.0.0.1', port)) == 0:
                return
        finally:
            s.close()
        time.sleep(0.25)
    raise SystemExit('server did not bind :%d within %ds' % (port, timeout))


def make_out_dir(label: str) -> str:
    ts = time.strftime('%Y%m%d-%H%M%S')
    d = os.path.join(OUT_DIR, '%s_%s' % (ts, label))
    os.makedirs(d, exist_ok=True)
    return d


def find_ros_executable(package: str, executable: str) -> str:
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
        if not prefix:
            continue
        path = os.path.join(prefix, 'lib', package, executable)
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    raise SystemExit('%s/%s not found on AMENT_PREFIX_PATH -- is the ROS2 env sourced?'
                      % (package, executable))


def server_command(kind: str, port: int):
    """Returns (argv, env_extra, description). `description` always names
    the file that will be imported/executed."""
    ping_params = ['-p', 'retry_startup_delay:=5.0', '-p', 'websocket_ping_interval:=10',
                   '-p', 'websocket_ping_timeout:=30']
    if kind == 'stock':
        exe = find_ros_executable('rosbridge_server', 'rosbridge_websocket')
        cmd = [sys.executable, exe, '--ros-args', '-p', 'port:=%d' % port] + ping_params
        return cmd, {}, 'stock: %s' % exe
    if kind == 'lean':
        pkg_dir = os.path.join(REPO_ROOT, 'ros_ws', 'src', 'jugglebot')
        mod_path = os.path.join(pkg_dir, 'jugglebot', 'rosbridge_websocket_lean.py')
        if not os.path.isfile(mod_path):
            raise SystemExit('lean module not found at %s' % mod_path)
        env_extra = {'PYTHONPATH': pkg_dir + os.pathsep + os.environ.get('PYTHONPATH', '')}
        cmd = [sys.executable, '-m', 'jugglebot.rosbridge_websocket_lean', '--ros-args',
               '-p', 'port:=%d' % port] + ping_params
        return cmd, env_extra, 'lean: %s (PYTHONPATH led by %s)' % (mod_path, pkg_dir)
    raise ValueError(kind)


def rosapi_command():
    exe = find_ros_executable('rosapi', 'rosapi_node')
    return [sys.executable, exe, '--ros-args', '-r', '__node:=rosapi'], exe


def parse_pidstat_mean(path: str, pid: int, warm: int) -> float:
    """Mean %CPU (pidstat's column 8) for `pid`, dropping the first `warm`
    samples. Matches the awk recipe validated in run_load_probe.sh
    2026-09-13 (field 1 = HH:MM:SS, field 3 = PID, field 8 = %CPU)."""
    vals = []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) < 8 or not re.match(r'^\d{2}:\d{2}:\d{2}$', parts[0]):
                continue
            if not parts[2].lstrip('-').isdigit() or int(parts[2]) != pid:
                continue
            try:
                vals.append(float(parts[7]))
            except ValueError:
                continue
    if not vals:
        return float('nan')
    kept = vals[warm:] if len(vals) > warm else vals
    return sum(kept) / len(kept)


def count_spin_exceptions(log_path: str):
    """Counts `rosbridge_spin: unhandled exception` occurrences in a lean
    server log and extracts the exception type from each traceback."""
    text = open(log_path, errors='replace').read()
    count = text.count('rosbridge_spin: unhandled exception')
    types = collections.Counter()
    for block in text.split('rosbridge_spin: unhandled exception')[1:]:
        lines = block.splitlines()[:60]
        for line in reversed(lines):
            m = re.match(r'^([A-Za-z_][\w.]*(?:Error|Exception))\b', line.strip())
            if m:
                types[m.group(1)] += 1
                break
    return count, types


# ---------------------------------------------------------------------------
# Hidden child-process entry points (invoked as `<this file> _pub ...` etc.)
# ---------------------------------------------------------------------------

def _hidden_pub(argv):
    """Synthetic publisher mimicking the idle launch's topic census (bag
    2026-09-13_11-57-45): 4 topics at 100 Hz (robot_state, hand_telemetry,
    leg_cmd_executed, bb/axis_estimates), ~6 at 10 Hz, 2 at 5 Hz, plus
    GUI-rate copies gui/robot_state (20 Hz) and gui/hand_telemetry (10 Hz).
    Ported from the validated pub_load.py prototype."""
    dur = float(argv[0])
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from jugglebot_interfaces.msg import RobotState, MotorStateSingle, HandTelemetryMessage

    rclpy.init()
    n = Node('rosbridge_cpu_probe_pub')

    def diag():
        d = DiagnosticStatus(name='probe', message='ok', hardware_id='x')
        d.values = [KeyValue(key='k%d' % i, value='%.3f' % (i * 1.1)) for i in range(8)]
        return d

    rs = RobotState()
    rs.motor_states = [MotorStateSingle(pos_estimate=1.0, vel_estimate=0.1, bus_voltage=24.0)
                       for _ in range(9)]
    rs.pose_offset_rad = [0.001, -0.002]
    ht = HandTelemetryMessage(pos_cmd=1.0, pos_meas=1.0)
    lce = JointState(name=['leg%d' % i for i in range(6)], position=[0.1] * 6, velocity=[0.0] * 6)
    bba = JointState(name=['bb_pitch', 'bb_hand'], position=[0.1, 0.2], velocity=[0.0, 0.0])
    dg = diag()
    st = String(data='IDLE')

    p100 = [(n.create_publisher(RobotState, 'robot_state', 10), rs),
            (n.create_publisher(HandTelemetryMessage, 'hand_telemetry', 10), ht),
            (n.create_publisher(JointState, 'leg_cmd_executed', 10), lce),
            (n.create_publisher(JointState, 'bb/axis_estimates', 10), bba)]
    p10 = [(n.create_publisher(String, t, 10), st) for t in ('orchestrator_state', 'control_mode_topic')]
    p10 += [(n.create_publisher(DiagnosticStatus, t, 10), dg)
            for t in ('link_status', 'probe_diag10_a', 'probe_diag10_b')]
    p5 = [(n.create_publisher(DiagnosticStatus, t, 10), dg) for t in ('probe_diag5_a', 'probe_diag5_b')]
    g_rs = n.create_publisher(RobotState, 'gui/robot_state', 10)
    g_ht = n.create_publisher(HandTelemetryMessage, 'gui/hand_telemetry', 10)

    cnt = {'t100': 0}

    def t100():
        stamp = n.get_clock().now().to_msg()
        rs.timestamp = stamp
        ht.timestamp = stamp
        for p, m in p100:
            p.publish(m)
        cnt['t100'] += 1
        if cnt['t100'] % 5 == 0:
            g_rs.publish(rs)
        if cnt['t100'] % 10 == 0:
            g_ht.publish(ht)

    n.create_timer(0.01, t100)
    n.create_timer(0.1, lambda: [p.publish(m) for p, m in p10])
    n.create_timer(0.2, lambda: [p.publish(m) for p, m in p5])

    t0 = time.monotonic()
    end = t0 + dur
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    print('publisher: 100 Hz timer achieved %.1f Hz over %.0f s'
          % (cnt['t100'] / (time.monotonic() - t0), dur))
    n.destroy_node()
    rclpy.shutdown()


def _hidden_client(argv):
    """rosbridge client mimicking the GUI's subscription set. Modes: none,
    gui (full subs + 200 ms cbor-raw spies), nospy (gui minus the 100 Hz
    spies), rawspy (kept for symmetry; spies are always cbor-raw here,
    matching the shipped GUI fix). Ported from the validated ws_client.py
    prototype (its 'copies' mode is retained for the historical
    source-decimation comparison but is not one of the --client choices)."""
    mode, port, dur = argv[0], int(argv[1]), float(argv[2])
    import asyncio
    from tornado.websocket import websocket_connect

    async def main_coro():
        ws = await websocket_connect('ws://localhost:%d' % port)

        def sub(t, ty, thr, comp='none'):
            ws.write_message(json.dumps({'op': 'subscribe', 'topic': t, 'type': ty,
                                         'throttle_rate': thr, 'queue_length': 0,
                                         'compression': comp}))

        if mode != 'none':
            for t, ty, thr in (COPIES if mode == 'copies' else FULL_HI) + FULL_LO:
                sub(t, ty, thr)
            spies = SPY_LO + ([] if mode in ('nospy', 'copies') else SPY_HI)
            for t, ty in spies:
                # 'gui' reproduces the PRE-FIX baseline (JSON spies, as the
                # GUI did before ros-bridge.js's subscribeSpy fix); 'rawspy'
                # is the shipped fix (cbor-raw). Collapsing this distinction
                # would make acceptance test B's gui-vs-rawspy comparison
                # meaningless -- keep them distinct.
                sub(t, ty, 200, 'cbor-raw' if mode == 'rawspy' else 'none')

        counts = collections.Counter()
        end = time.monotonic() + dur
        while True:
            left = end - time.monotonic()
            if left <= 0:
                break
            try:
                m = await asyncio.wait_for(ws.read_message(), timeout=left)
            except asyncio.TimeoutError:
                break
            if m is None:
                print('socket closed')
                break
            counts['<cbor>' if isinstance(m, bytes) else json.loads(m).get('topic', '?')] += 1
        print('client %s rx Hz: %s' % (mode, {k: round(v / dur, 1) for k, v in sorted(counts.items())}))

    asyncio.run(main_coro())


def _hidden_leakclient(argv):
    """N sequential `/rosapi/topics` service calls, latency mean of the
    first 50 and last 50. Ported from the validated leak_client.py
    prototype."""
    port, n_calls, cap = int(argv[0]), int(argv[1]), float(argv[2])
    import asyncio
    from tornado.websocket import websocket_connect

    async def main_coro():
        ws = await websocket_connect('ws://localhost:%d' % port)
        lat = []
        t_end = time.monotonic() + cap
        for i in range(n_calls):
            if time.monotonic() > t_end:
                break
            t = time.perf_counter()
            ws.write_message(json.dumps({'op': 'call_service', 'service': '/rosapi/topics',
                                         'args': {}, 'id': 'c%d' % i}))
            while True:
                m = json.loads(await ws.read_message())
                if m.get('op') == 'service_response':
                    break
            lat.append((time.perf_counter() - t) * 1e3)
        k = min(50, len(lat))
        print('calls done: %d; latency first %d mean %.1f ms, last %d mean %.1f ms'
              % (len(lat), k, sum(lat[:k]) / k, k, sum(lat[-k:]) / k))
        ws.close()

    asyncio.run(main_coro())


def _hidden_churnclient(argv):
    """D2 recipe: `cycles` connect -> full GUI subscribe set (spies
    included) -> hold `hold` s -> disconnect, then one steady client for
    `steady` s reporting received Hz per topic (same shape as _hidden_client
    'gui' mode, so it is directly comparable to `load`'s numbers)."""
    port, cycles, hold, steady = int(argv[0]), int(argv[1]), float(argv[2]), float(argv[3])
    import asyncio
    from tornado.websocket import websocket_connect

    def _sub_all(ws):
        for t, ty, thr in FULL_HI + FULL_LO:
            ws.write_message(json.dumps({'op': 'subscribe', 'topic': t, 'type': ty,
                                         'throttle_rate': thr, 'queue_length': 0,
                                         'compression': 'none'}))
        for t, ty in SPY_LO + SPY_HI:
            ws.write_message(json.dumps({'op': 'subscribe', 'topic': t, 'type': ty,
                                         'throttle_rate': 200, 'queue_length': 0,
                                         'compression': 'cbor-raw'}))

    async def one_cycle(i):
        try:
            ws = await websocket_connect('ws://localhost:%d' % port, connect_timeout=10)
        except Exception as e:
            print('cycle %d: CONNECT FAILED: %r' % (i, e))
            return
        _sub_all(ws)
        await asyncio.sleep(hold)
        ws.close()

    async def steady_client():
        ws = await websocket_connect('ws://localhost:%d' % port)
        _sub_all(ws)
        counts = collections.Counter()
        end = time.monotonic() + steady
        while True:
            left = end - time.monotonic()
            if left <= 0:
                break
            try:
                m = await asyncio.wait_for(ws.read_message(), timeout=left)
            except asyncio.TimeoutError:
                break
            if m is None:
                print('steady client: socket closed early')
                break
            counts['<cbor>' if isinstance(m, bytes) else json.loads(m).get('topic', '?')] += 1
        print('steady client rx Hz: %s' % {k: round(v / steady, 1) for k, v in sorted(counts.items())})

    async def run_all():
        for i in range(cycles):
            await one_cycle(i)
        print('churn: %d connect/subscribe/disconnect cycles done' % cycles)
        await steady_client()

    asyncio.run(run_all())


_HIDDEN = {'_pub': _hidden_pub, '_client': _hidden_client,
           '_leakclient': _hidden_leakclient, '_churnclient': _hidden_churnclient}


# ---------------------------------------------------------------------------
# Scenario commands
# ---------------------------------------------------------------------------

def _start_ctrl_and_watchdog(domain):
    set_domain(domain)
    import rclpy
    rclpy.init()
    ctrl = rclpy.create_node('rosbridge_cpu_probe')
    check_graph_empty(ctrl)
    stop_wd = threading.Event()
    wd = threading.Thread(target=containment_watchdog, args=(ctrl, stop_wd), daemon=True)
    wd.start()
    return rclpy, ctrl, stop_wd


def cmd_load(args):
    rclpy, ctrl, stop_wd = _start_ctrl_and_watchdog(args.domain)
    try:
        out_dir = make_out_dir('load_%s_%s' % (args.server, args.client))
        total = args.seconds + args.warm
        pub_proc = spawn('publisher', [sys.executable, THIS_FILE, '_pub', str(total + 20)],
                          out_path=os.path.join(out_dir, 'publisher.log'))
        server_cmd, server_env, desc = server_command(args.server, args.port)
        log('server: %s' % desc)
        server_proc = spawn('server', server_cmd, env_extra=server_env,
                             out_path=os.path.join(out_dir, 'server.log'))
        wait_port(args.port, 30)
        time.sleep(2)  # let discovery / executor settle before the client hits it
        client_log_path = os.path.join(out_dir, 'client.log')
        client_proc = spawn('client', [sys.executable, THIS_FILE, '_client', args.client,
                                       str(args.port), str(total)], out_path=client_log_path)
        pidstat_path = os.path.join(out_dir, 'pidstat.txt')
        subprocess.run(['pidstat', '-u', '-p', '%d,%d' % (server_proc.pid, pub_proc.pid),
                        '1', str(int(total))], stdout=open(pidstat_path, 'w'))
        client_proc.wait(timeout=30)
        server_mean = parse_pidstat_mean(pidstat_path, server_proc.pid, warm=int(args.warm))
        pub_mean = parse_pidstat_mean(pidstat_path, pub_proc.pid, warm=int(args.warm))
        client_result = open(client_log_path).read().strip()
        print('load server=%s client=%s: server %%CPU=%.1f (after %ds warm-up), '
              'publisher %%CPU=%.1f' % (args.server, args.client, server_mean, int(args.warm), pub_mean))
        print(client_result)
        print('logs: %s' % out_dir)
    finally:
        stop_wd.set()
        teardown()
        ctrl.destroy_node()
        rclpy.shutdown()


def cmd_leak(args):
    rclpy, ctrl, stop_wd = _start_ctrl_and_watchdog(args.domain)
    try:
        out_dir = make_out_dir('leak_%s' % args.server)
        server_cmd, server_env, desc = server_command(args.server, args.port)
        log('server: %s' % desc)
        server_proc = spawn('server', server_cmd, env_extra=server_env,
                             out_path=os.path.join(out_dir, 'server.log'))
        wait_port(args.port, 30)
        rosapi_cmd, rosapi_exe = rosapi_command()
        log('rosapi: %s' % rosapi_exe)
        spawn('rosapi', rosapi_cmd, out_path=os.path.join(out_dir, 'rosapi.log'))
        time.sleep(3)  # let rosapi register its services

        idle_before = os.path.join(out_dir, 'idle_before.txt')
        subprocess.run(['pidstat', '-u', '-p', str(server_proc.pid), '1',
                        str(int(args.idle_seconds))], stdout=open(idle_before, 'w'))
        idle_before_mean = parse_pidstat_mean(idle_before, server_proc.pid, warm=0)

        leak_log = os.path.join(out_dir, 'leak_client.log')
        cap = args.calls * 0.2 + 60
        subprocess.run([sys.executable, THIS_FILE, '_leakclient', str(args.port),
                        str(args.calls), str(cap)], stdout=open(leak_log, 'w'),
                       stderr=subprocess.STDOUT, timeout=cap + 30)
        leak_result = open(leak_log).read().strip()

        idle_after = os.path.join(out_dir, 'idle_after.txt')
        subprocess.run(['pidstat', '-u', '-p', str(server_proc.pid), '1',
                        str(int(args.idle_seconds))], stdout=open(idle_after, 'w'))
        idle_after_mean = parse_pidstat_mean(idle_after, server_proc.pid, warm=0)

        print('leak server=%s calls=%d: idle before=%.2f%%, idle after=%.2f%%'
              % (args.server, args.calls, idle_before_mean, idle_after_mean))
        print(leak_result)
        print('logs: %s' % out_dir)
    finally:
        stop_wd.set()
        teardown()
        ctrl.destroy_node()
        rclpy.shutdown()


def cmd_churn(args):
    rclpy, ctrl, stop_wd = _start_ctrl_and_watchdog(args.domain)
    try:
        out_dir = make_out_dir('churn_%s' % args.server)
        pub_total = args.cycles * (args.hold + 1.0) + args.steady_seconds + 30
        pub_proc = spawn('publisher', [sys.executable, THIS_FILE, '_pub', str(pub_total)],
                          out_path=os.path.join(out_dir, 'publisher.log'))
        server_cmd, server_env, desc = server_command(args.server, args.port)
        log('server: %s' % desc)
        server_proc = spawn('server', server_cmd, env_extra=server_env,
                             out_path=os.path.join(out_dir, 'server.log'))
        wait_port(args.port, 30)
        time.sleep(2)
        churn_log = os.path.join(out_dir, 'churn_client.log')
        churn_proc = spawn('churn_client', [sys.executable, THIS_FILE, '_churnclient',
                                            str(args.port), str(args.cycles), str(args.hold),
                                            str(args.steady_seconds)], out_path=churn_log)
        server_died = False
        while churn_proc.poll() is None:
            if server_proc.poll() is not None:
                server_died = True
                log('FATAL: server process died mid-churn (rc=%s)' % server_proc.returncode)
                break
            time.sleep(1.0)
        if not server_died:
            churn_proc.wait(timeout=30)
        server_alive = (not server_died) and server_proc.poll() is None
        count, types = count_spin_exceptions(os.path.join(out_dir, 'server.log'))
        client_log = open(churn_log).read().strip()
        print('churn server=%s cycles=%d: server_alive=%s, unhandled exceptions in '
              'server log=%d %s' % (args.server, args.cycles, server_alive, count, dict(types)))
        print(client_log)
        print('logs: %s' % out_dir)
        if not server_alive:
            print('DATA STOPPED FLOWING / SERVER DIED -- this is a design question '
                  'for the main session, not something to patch here.')
    finally:
        stop_wd.set()
        teardown()
        ctrl.destroy_node()
        rclpy.shutdown()


def cmd_compare(args):
    print('=== uptime at start ===')
    os.system('uptime')
    if args.scenario == 'load':
        for server in ('stock', 'lean'):
            ns = argparse.Namespace(server=server, client=args.client, seconds=args.seconds,
                                    warm=args.warm, port=args.port, domain=args.domain)
            cmd_load(ns)
            time.sleep(1)
    else:
        for server in ('stock', 'lean'):
            ns = argparse.Namespace(server=server, calls=args.calls, port=args.port,
                                    domain=args.domain, idle_seconds=args.idle_seconds)
            cmd_leak(ns)
            time.sleep(1)
    print('=== uptime at end ===')
    os.system('uptime')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_parser():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    p_load = sub.add_parser('load')
    p_load.add_argument('--server', choices=['stock', 'lean'], required=True)
    p_load.add_argument('--client', choices=['none', 'gui', 'rawspy', 'nospy'], required=True)
    p_load.add_argument('--seconds', type=float, default=35)
    p_load.add_argument('--warm', type=float, default=8)
    p_load.add_argument('--port', type=int, default=DEFAULT_PORT)
    p_load.add_argument('--domain', type=int, default=DEFAULT_DOMAIN)
    p_load.set_defaults(func=cmd_load)

    p_leak = sub.add_parser('leak')
    p_leak.add_argument('--server', choices=['stock', 'lean'], required=True)
    p_leak.add_argument('--calls', type=int, default=1200)
    p_leak.add_argument('--port', type=int, default=DEFAULT_PORT)
    p_leak.add_argument('--domain', type=int, default=DEFAULT_DOMAIN)
    p_leak.add_argument('--idle-seconds', type=float, default=20)
    p_leak.set_defaults(func=cmd_leak)

    p_churn = sub.add_parser('churn')
    p_churn.add_argument('--server', choices=['stock', 'lean'], default='lean')
    p_churn.add_argument('--cycles', type=int, default=50)
    p_churn.add_argument('--hold', type=float, default=2.0)
    p_churn.add_argument('--steady-seconds', type=float, default=20)
    p_churn.add_argument('--port', type=int, default=DEFAULT_PORT)
    p_churn.add_argument('--domain', type=int, default=DEFAULT_DOMAIN)
    p_churn.set_defaults(func=cmd_churn)

    p_cmp = sub.add_parser('compare')
    p_cmp.add_argument('--scenario', choices=['load', 'leak'], required=True)
    p_cmp.add_argument('--client', choices=['none', 'gui', 'rawspy', 'nospy'], default='gui')
    p_cmp.add_argument('--calls', type=int, default=1200)
    p_cmp.add_argument('--seconds', type=float, default=35)
    p_cmp.add_argument('--warm', type=float, default=8)
    p_cmp.add_argument('--idle-seconds', type=float, default=20)
    p_cmp.add_argument('--port', type=int, default=DEFAULT_PORT)
    p_cmp.add_argument('--domain', type=int, default=DEFAULT_DOMAIN)
    p_cmp.set_defaults(func=cmd_compare)

    return p


def main():
    _maybe_reexec_with_ros_env()
    if len(sys.argv) > 1 and sys.argv[1] in _HIDDEN:
        _HIDDEN[sys.argv[1]](sys.argv[2:])
        return
    args = _build_parser().parse_args(sys.argv[1:])
    args.func(args)


if __name__ == '__main__':
    main()
