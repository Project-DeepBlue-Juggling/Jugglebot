"""Real-rclpy smoke tests (FEAT-55): one node per test, real rclpy, child process.

Everything under tests/ros/ runs on mocks (tests/ros/conftest.py); nothing there can
see executor threading, QoS, real message types or shutdown behaviour. These tests
start a REAL node in a child process under real Foxy rclpy and check that it comes
up (a topic or service appears in the graph) and exits cleanly on SIGINT.

Isolation: a random high ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1, so nothing here
can see or be seen by a live robot's nodes. The bridge node's UDP transport is
forced onto loopback by tests/rclpy_smoke/_child.py and pointed at a fake peer
owned by the test: the real can-bridge board is never addressed (see _child.py).

Skipped cleanly when Foxy or the built jugglebot_interfaces are absent, and refuses
to run (skip) while tools/lib/live_session.sh reports a live session. Marked
nightly: slow (seconds of DDS discovery per node) and environment-dependent.
"""
from __future__ import annotations

import os
import random
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

# serial as well as nightly: the live_session guard matches ANY process whose command
# line names a node (other tests spawn such processes under xdist), so the guard only
# gives a stable answer in the serial tail, where nothing else runs.
pytestmark = [pytest.mark.nightly, pytest.mark.serial]

REPO = Path(__file__).resolve().parents[2]
CHILD = Path(__file__).with_name('_child.py')
FOXY = Path('/opt/ros/foxy/setup.bash')


def _interfaces_setup():
    for root in (REPO / 'ros_ws' / 'install',
                 Path.home() / 'Desktop' / 'Jugglebot' / 'ros_ws' / 'install'):
        f = root / 'jugglebot_interfaces' / 'share' / 'jugglebot_interfaces' / 'local_setup.bash'
        if f.is_file():
            return f
    return None


def _live_session() -> bool:
    r = subprocess.run(
        ['bash', '-c', f'source {shlex.quote(str(REPO / "tools/lib/live_session.sh"))}; live_session'],
        capture_output=True)
    return r.returncode == 0


@pytest.fixture(scope='module')
def rclpy_env():
    iface = _interfaces_setup()
    if not FOXY.is_file():
        pytest.skip('ROS 2 Foxy is not installed')
    if iface is None:
        pytest.skip('jugglebot_interfaces is not built (ros_ws/install)')
    if _live_session():
        pytest.skip('live_session reports a live robot session; refusing to start nodes')
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(random.randint(80, 101))
    env['ROS_LOCALHOST_ONLY'] = '1'
    # Node source from THIS tree (not install/), plus the repo root for teensy_link.
    env['PYTHONPATH'] = os.pathsep.join(
        [str(REPO / 'ros_ws' / 'src' / 'jugglebot'), str(REPO), env.get('PYTHONPATH', '')])
    env['_SMOKE_SETUP'] = f'source {shlex.quote(str(FOXY))} && source {shlex.quote(str(iface))}'
    env.pop('JUGGLEBOT_REPO', None)
    return env


_started = []


def _spawn(env, *args, extra_env=None):
    e = dict(env)
    e.update(extra_env or {})
    cmd = (f'{e["_SMOKE_SETUP"]} && exec {shlex.quote(sys.executable)} '
           f'{shlex.quote(str(CHILD))} ' + ' '.join(shlex.quote(a) for a in args))
    proc = subprocess.Popen(['bash', '-c', cmd], env=e, cwd=str(REPO),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            start_new_session=True)
    proc.lines = []   # merged stdout/stderr, drained by a thread so the pipe never fills

    def _drain():
        for raw in proc.stdout:
            proc.lines.append(raw.decode(errors='replace'))

    threading.Thread(target=_drain, daemon=True).start()
    _started.append(proc)
    return proc


def _wait_log(p, needle, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if any(needle in ln for ln in p.lines):
            return True
        if p.poll() is not None:
            return False
        time.sleep(0.1)
    return False


def _log(p):
    time.sleep(0.2)
    return ''.join(p.lines)


def _run(env, *args, timeout=60):
    p = _spawn(env, *args)
    try:
        p.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        _kill(p)
        return -999, _log(p)
    return p.returncode, _log(p)


def _kill(p):
    if p.poll() is None:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        p.wait()


@pytest.fixture(autouse=True)
def _reap():
    yield
    for p in _started:
        _kill(p)
    _started.clear()


def _sigint_and_time(p, bound):
    """SIGINT the node (the exec'd python is the group leader's only process)."""
    t0 = time.monotonic()
    p.send_signal(signal.SIGINT)
    try:
        p.wait(timeout=bound)
    except subprocess.TimeoutExpired:
        _kill(p)
        return None
    return time.monotonic() - t0


def test_catch_correlation_node_comes_up_and_exits_on_sigint(rclpy_env):
    node = _spawn(rclpy_env, 'catch')
    rc, out = _run(rclpy_env, 'probe', 'cone/timing_result', '30', timeout=60)
    assert rc == 0, f'topic never appeared:\n{out}'
    dt = _sigint_and_time(node, 10.0)
    log = _log(node)
    assert dt is not None, f'no exit within 10 s of SIGINT:\n{log}'
    assert node.returncode == 0, f'unclean exit {node.returncode}:\n{log}'


def _fake_peer(answer_home=False, unpowered=False):
    """Loopback can-bridge stand-in (the shared FakeTeensy), ephemeral ports.

    Silent by default (every RPC times out). With answer_home it ACKs HOME like the
    firmware does, which puts the bridge's homing loop into its monitor phase, and
    once the bridge has spoken it pumps a DIAGNOSTIC per Leg at 5 Hz (HB_SEEN, axis
    state Idle) so the host Leg IDLE rule (ARMING_CONTRACT A7, 2026-11-25) sees six
    fresh Idle Legs and lets the Home start (BUG-145: without them every Leg reads
    unknown and /home_motors is refused before Homing begins).
    With unpowered=True it behaves like the robot with the motor supply off (BUG-47,
    BUG-129 series 2026-11-25): every RPC is answered ERR_REJECTED, and once the
    bridge has spoken it pumps HEARTBEAT_T2J frames with fault_state=ODRIVE_FATAL
    (the guard latched on INITIALIZING + DC_BUS_UNDER_VOLTAGE) at 10 Hz.
    """
    from tests.teensy_link.conftest import FakeTeensy
    from teensy_link import protocol as p
    peer = FakeTeensy(bind_host='127.0.0.1', stream_port=0, rpc_port=0)
    peer.rpc_log = []   # (monotonic, method, args) of the RPCs the answer_home fake accepted
    if answer_home:
        peer.on_rpc(int(p.RpcMethod.HOME), lambda _rid, _args: (int(p.RpcStatus.OK), b''))

        def _record(method):
            def _h(_rid, args):
                peer.rpc_log.append((time.monotonic(), method, bytes(args)))
                return int(p.RpcStatus.OK), b''
            return _h
        # BUG-47: an aborted homing sends IDLE then persists is_homed=False (STATE_WRITE).
        peer.on_rpc(int(p.RpcMethod.SET_AXIS_STATE), _record('SET_AXIS_STATE'))
        peer.on_rpc(int(p.RpcMethod.STATE_WRITE), _record('STATE_WRITE'))
    stop = threading.Event()

    def _learn_jetson_addr():
        """Block until the bridge's first frame names its address, then aim
        send_to_jetson at it. False if the peer was closed first."""
        while not stop.is_set():
            got = peer.received()
            if got:
                peer.set_jetson_addr(got[0].from_addr)
                return True
            time.sleep(0.05)
        return False

    if answer_home:
        idle = 1        # ODrive AXIS_STATE_IDLE
        hb_seen = 0x2   # DIAGNOSTIC flags bit 1, teensy_bridge_node._DIAG_FLAG_HB_SEEN

        def _pump_leg_diagnostics():
            if not _learn_jetson_addr():
                return
            while not stop.is_set():
                try:
                    for leg in range(p.NUM_LEGS):
                        peer.send_to_jetson(int(p.MsgType.DIAGNOSTIC), p.Diagnostic(
                            axis_id=leg, axis_state=idle, active_errors=0,
                            flags=hb_seen).pack())
                except OSError:
                    pass
                time.sleep(0.2)

        threading.Thread(target=_pump_leg_diagnostics, daemon=True).start()
    if unpowered:
        for m in p.RpcMethod:
            peer.on_rpc(int(m), lambda _rid, _args: (int(p.RpcStatus.ERR_REJECTED), b''))

        def _pump():
            if not _learn_jetson_addr():
                return
            while not stop.is_set():
                try:
                    peer.send_heartbeat_t2j(fault_state=int(p.FaultState.ODRIVE_FATAL))
                except OSError:
                    pass
                time.sleep(0.1)

        threading.Thread(target=_pump, daemon=True).start()
    peer.start()
    peer.ports = (peer.stream_port, peer.rpc_port)

    def _close():
        stop.set()
        peer.stop()

    peer.close = _close
    return peer


def _bridge(env, peer):
    return _spawn(env, 'bridge', extra_env={
        'JUGGLEBOT_SMOKE_PEER_HOST': '127.0.0.1',
        'JUGGLEBOT_SMOKE_STREAM_PORT': str(peer.ports[0]),
        'JUGGLEBOT_SMOKE_RPC_PORT': str(peer.ports[1]),
    })


def test_bridge_node_comes_up_against_fake_peer_and_exits_on_sigint(rclpy_env):
    peer = _fake_peer()
    try:
        node = _bridge(rclpy_env, peer)
        rc, out = _run(rclpy_env, 'probe', 'robot_state', '60', timeout=90)
        assert rc == 0, f'robot_state never appeared:\n{out}'
        # The constructor's boot-state read blocks on RPC timeouts against the silent
        # peer; SIGINT before it returns lands outside main()'s try. Wait it out.
        assert _wait_log(node, 'TeensyBridgeNode up', 60), f'never finished init:\n{_log(node)}'
        time.sleep(1.0)
        assert len(peer.received()) > 0, 'bridge never spoke to the fake peer (heartbeat)'
        dt = _sigint_and_time(node, 30.0)
        log = _log(node)
        assert dt is not None, f'no exit within 30 s of SIGINT:\n{log[-3000:]}'
        assert node.returncode == 0, f'unclean exit {node.returncode}:\n{log[-3000:]}'
    finally:
        peer.close()


def test_bridge_node_exit_with_home_action_in_flight_bug47(rclpy_env):
    """BUG-47: SIGINT while a home_motors callback is running must still exit promptly.

    The fake peer ACKs HOME and then goes silent, so _run_home sits in its monitor
    loop (20 s timeout): a long-running action callback in flight. on_shutdown() sets
    the shutdown event, the loop sees it, logs the abort and commands the axis IDLE.
    Before the fix the process lingered ~21 s (strict xfail, 2026-11-24/25).
    A SIGUSR1 faulthandler dump at SIGINT+1.4 s lands in the node log on failure.
    """
    peer = _fake_peer(answer_home=True)
    try:
        node = _bridge(rclpy_env, peer)
        assert _wait_log(node, 'TeensyBridgeNode up', 60), f'never finished init:\n{_log(node)}'
        rc, out = _run(rclpy_env, 'homegoal', '60', timeout=90)
        assert rc == 0, f'home goal not accepted (rc={rc}):\n{out}'
        assert node.poll() is None
        # SIGINT straight after the goal is accepted: the homing callback is in flight.
        t0 = time.monotonic()
        node.send_signal(signal.SIGINT)
        time.sleep(1.4)   # past on_shutdown (~1 s): what is main() waiting on now?
        if node.poll() is None:
            node.send_signal(signal.SIGUSR1)   # faulthandler all-thread dump, see _child.py
        try:
            node.wait(timeout=60.0)
        except subprocess.TimeoutExpired:
            _kill(node)
            node.returncode = None
        dt = None if node.returncode is None else time.monotonic() - t0
        log = _log(node)
        print(f'BUG-47 exit time after SIGINT: {dt}\n--- node log ---\n{log}')
        assert dt is not None and dt < 8.0, (
            f'bridge did not exit within 8 s of SIGINT (exit time {dt}):\n{log[-3000:]}')
        assert 'homing: aborted on axis' in log, log[-3000:]
        assert node.returncode == 0, f'unclean exit {node.returncode}:\n{log[-3000:]}'
        # The not-homed write must reach the peer BEFORE the transport closed (it can
        # only arrive while the socket is open): IDLE for the homing axis, then the
        # STATE_WRITE with is_homed=False. A write that raced the close never arrives.
        from teensy_link import rpc_args
        names = [m for _t, m, _a in peer.rpc_log]
        idle = [a for _t, m, a in peer.rpc_log if m == 'SET_AXIS_STATE']
        writes = [a for _t, m, a in peer.rpc_log if m == 'STATE_WRITE']
        assert rpc_args.encode_set_axis_state(0, 1) in idle, (names, log[-3000:])
        assert writes, f'no is_homed=False STATE_WRITE reached the peer: {names}\n{log[-3000:]}'
        assert writes[-1] == rpc_args.encode_state_write(False, False), writes[-1]
        assert names.index('STATE_WRITE') > names.index('SET_AXIS_STATE'), names
    finally:
        peer.close()


def test_bridge_node_exits_promptly_on_sigint_when_robot_unpowered_bug47(rclpy_env):
    """BUG-47 (unpowered case): drives INITIALIZING + guard latched ODRIVE_FATAL, every
    RPC rejected. SIGINT after 'TeensyBridgeNode up' must run on_shutdown and exit
    within 8 s. A regression pin: it passes before and after the 2026-11-25 change,
    because the 24/24 'no exit' of the BUG-129 series was the harness starting the
    launch with SIGINT ignored (see the logbook entry), not the node."""
    peer = _fake_peer(unpowered=True)
    try:
        node = _bridge(rclpy_env, peer)
        assert _wait_log(node, 'TeensyBridgeNode up', 60), f'never finished init:\n{_log(node)}'
        time.sleep(2.0)   # heartbeats with the latched fault reach the node
        t0 = time.monotonic()
        node.send_signal(signal.SIGINT)
        time.sleep(2.0)
        if node.poll() is None:
            node.send_signal(signal.SIGUSR1)   # all-thread stack dump into the log
        try:
            node.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            _kill(node)
            node.returncode = None
        dt = None if node.returncode is None else time.monotonic() - t0
        log = _log(node)
        print(f'unpowered exit time after SIGINT: {dt}')
        assert dt is not None and dt < 8.0, f'exit time {dt}:\n{log[-3000:]}'
        assert 'Shutting down TeensyBridgeNode' in log, log[-3000:]
        assert node.returncode == 0, f'unclean exit {node.returncode}:\n{log[-3000:]}'
    finally:
        peer.close()


def test_bridge_node_sigint_during_boot_read_runs_shutdown_bug141(rclpy_env):
    """BUG-141: SIGINT while the constructor's boot-state read is blocked on RPC
    timeouts (silent peer) used to die with a KeyboardInterrupt traceback (exit -2,
    no on_shutdown). It must now take the normal shutdown path and exit 0."""
    peer = _fake_peer()
    try:
        node = _bridge(rclpy_env, peer)
        assert _wait_log(node, 'LEG TORQUE FEEDFORWARD', 60), f'ctor never got going:\n{_log(node)}'
        assert not any('TeensyBridgeNode up' in ln for ln in node.lines), 'boot read already over'
        t0 = time.monotonic()
        node.send_signal(signal.SIGINT)
        try:
            node.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            _kill(node)
            node.returncode = None
        log = _log(node)
        dt = None if node.returncode is None else time.monotonic() - t0
        assert dt is not None and dt < 8.0, f'exit time {dt}:\n{log[-3000:]}'
        assert 'Traceback' not in log, log[-3000:]
        assert 'Shutting down TeensyBridgeNode' in log, log[-3000:]
        assert node.returncode == 0, f'exit {node.returncode}:\n{log[-3000:]}'
    finally:
        peer.close()


def test_bridge_child_refuses_without_loopback_override(rclpy_env):
    """The safety latch: with no loopback peer in the environment the bridge child
    exits 99 before any socket exists (it must never fall back to the real bridge)."""
    rc, out = _run(rclpy_env, 'bridge', timeout=30)
    assert rc == 99, out
    assert 'TeensyBridgeNode' not in out
