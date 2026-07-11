#!/usr/bin/env python3
"""
gui_synthetic_stack.py — Fake ROS2 stack for live GUI behavioural verification
(NO hardware, NO real robot nodes).

Publishes scripted GUI-consumed telemetry through a REAL rosbridge websocket so
that headless chromium (driven by ``tools/probes/gui_dom_probe.py``) can load
the real GUI from the systemd server on :8081 and assert DOM state — the
software gate for (a) the per-bus CAN traffic panel, (b) the leg_setpoint_echo
pos_cmd datasource, and (c) the upcoming orchestrator state-minimap.

CONTAINMENT (safety rails, enforced in code):
  * REFUSES to start if the ROS graph is non-empty (any real node running) or
    if :9090 is already bound (a real rosbridge).  Single-instance lockfile.
  * CONTINUOUSLY re-checks the graph (~1 Hz) while running: if any foreign
    node appears mid-run (operator launching the real stack — whose
    trajectory_node would ingest our fabricated is_homed=True telemetry),
    ALL publishers are disabled immediately and the harness tears itself
    down (``_containment_watchdog``).
  * Launches ONLY ``rosbridge_websocket`` + ``rosapi`` (replicating
    ``jugglebot_launch.py``'s direct-Node params: port 9090,
    retry_startup_delay 5.0, websocket_ping_interval 10,
    websocket_ping_timeout 30).  NO hardware nodes — no teensy_bridge_node,
    no orchestrator, no trajectory_node.
  * PUBLISHES only GUI-consumed telemetry topics: ``orchestrator_state``,
    ``control_mode_topic``, ``robot_state``, ``link_status``, ``profile``,
    ``hand_telemetry``, ``leg_setpoint_echo``.  It NEVER publishes to a
    command topic the robot acts on.
  * SERVES ``set_setpoint_output`` / ``clear_errors`` / ``home`` /
    ``trajectory/go_home`` / ``trajectory/hold`` as RECORDERS that only
    mutate fake in-process state and log the call — nothing reaches hardware
    (there is no hardware node in the graph to reach; see rail #1).
  * SUBSCRIBES to ``orchestrator_command`` purely as a recorder (so future
    interactive scenarios can assert what the GUI sent).
  * SIGINT/SIGTERM/atexit teardown kills rosbridge + rosapi process groups.

Run environment: system python3 (3.8) with the ROS2 Foxy env sourced::

    bash -c 'source /opt/ros/foxy/setup.bash \
             && source ros_ws/install/setup.bash \
             && python3 tools/probes/gui_synthetic_stack.py'

Normally you do NOT run this directly — ``gui_dom_probe.py`` spawns it and
drives it over the stdin/stdout line-JSON protocol below.  For manual browser
inspection: ``--stage can-slot-mapping`` applies a stage at startup.

stdin/stdout protocol (one JSON object per line; logs go to stderr):
  in : {"cmd": "stage", "name": "<stage>"}   -> out: {"ok": true, "stage": ...}
  in : {"cmd": "records"}                    -> out: {"ok": true, "records": ...}
  in : {"cmd": "ping"}                       -> out: {"ok": true}
  in : {"cmd": "quit"}                       -> teardown + exit
  out (unsolicited): {"event": "ready", ...} once rosbridge accepts on :9090.

Stages are DECLARATIVE full-state configs (each stage fully replaces the
publisher config), so any stage sequence is deterministic.  Stage names used
by scenario1 of the DOM probe: connect, can-slot-mapping, can-health,
can-linkdown, can-staleness, pos-cmd-echo, pos-cmd-echo-stale,
orchestrator-state.

Motivating logbook entries:
  * logbook/2026-07-11-gui-leg-setpoint-echo-poscmd.md  (pos_cmd datasource)
  * logbook/2026-07-11-gui-can-traffic-per-bus-panel.md (per-bus CAN panel)
"""
from __future__ import annotations

import argparse
import atexit
import copy
import fcntl
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LOCKFILE = os.path.join(REPO_ROOT, "temp", "probes", "gui_synthetic_stack.lock")
ROSBRIDGE_PORT = 9090

# ---------------------------------------------------------------------------
# Scripted values (ground truth for the DOM probe's assertions)
# ---------------------------------------------------------------------------

# KEYSTONE slot-mapping fixture: DISTINCT per-wire-slot values so a future
# slot swap in the GUI is caught end-to-end.  Wire slot can1_* is physical
# CAN3 (Jugglebot core), wire slot can2_* is physical CAN1 (Ball Butler) —
# see ros_ws/gui/js/can-traffic.js BUSES registry.
PROFILE_DISTINCT = {
    "can1_rx": "100", "can1_tx": "20", "can1_util_pct": "13.3",
    "can2_rx": "7", "can2_tx": "3", "can2_util_pct": "1.1",
}

# link_status fixtures model the REAL bridge (teensy_bridge_node.py
# _publish_link_status, ~:2020): bridge_link is ALWAYS present (UP / LOST /
# NO_HEARTBEAT) alongside heartbeat_age_ms and the per-bus healths, published
# continuously at 10 Hz.  The panel treats a missing/non-'UP' bridge_link as
# link-down (honest-UNKNOWN over frozen-green), so a fixture omitting the key
# would be modelling a bridge that doesn't exist.
LINK_UP_UNKNOWN = {"bridge_link": "UP", "heartbeat_age_ms": "55",
                   "bus1_health": "UNKNOWN", "bus2_health": "UNKNOWN"}
LINK_UP_HEALTH = {"bridge_link": "UP", "heartbeat_age_ms": "55",
                  "bus1_health": "WARN", "bus2_health": "OK"}
# The cached-republish deception case: uplink LOST but the bridge keeps
# republishing its frozen last-heartbeat healths at 10 Hz (and its cached
# profile at 1 Hz) — the panel must show '--'/UNKNOWN/STALE despite
# fresh-looking messages flowing.
LINK_LOST_FROZEN = {"bridge_link": "LOST", "heartbeat_age_ms": "4023",
                    "bus1_health": "WARN", "bus2_health": "OK"}

LEG_POS_REV = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]           # measured (robot_state)
LEG_CMD_REV = [p + 0.02 for p in LEG_POS_REV]            # commanded (echo);
# 0.02 rev / MM_TO_REV (~0.01418 rev/mm) ~= 1.4 mm tracking error per leg.

# All-off baseline; every stage starts from a deep copy of this.
BASELINE_CFG = {
    "profile":            {"period": 1.0,  "enabled": False, "values": dict(PROFILE_DISTINCT)},
    "link_status":        {"period": 0.1,  "enabled": False, "values": dict(LINK_UP_UNKNOWN)},
    "robot_state":        {"period": 0.04, "enabled": False, "leg_pos": list(LEG_POS_REV),
                           "hand_pos": 0.2, "bus_voltage": 25.1},
    "leg_setpoint_echo":  {"period": 0.04, "enabled": False, "data": list(LEG_CMD_REV)},
    "hand_telemetry":     {"period": 0.1,  "enabled": False, "pos_cmd": 0.25, "pos_meas": 0.2},
    "orchestrator_state": {"period": 0.5,  "enabled": False, "data": "IDLE"},
    "control_mode_topic": {"period": 0.5,  "enabled": False, "data": ""},
}

# Declarative stages: {topic: {field: value, ...}} overrides on the baseline.
#
# link_status is enabled from the very first stage, matching the real bridge
# (it publishes at 10 Hz from node start, healths UNKNOWN until the first
# heartbeat) — without it the panel's healthStale watchdog legitimately shows
# the STALE badge, which is honest GUI behaviour, not a scenario condition.
STAGES = {
    # rosbridge up, GUI connects; bridge alive (link_status UP, healths
    # UNKNOWN) but no profile yet.
    "connect": {
        "link_status": {"enabled": True},
    },
    # 1 Hz profile with DISTINCT slot values (keystone slot-mapping check).
    "can-slot-mapping": {
        "profile": {"enabled": True},
        "link_status": {"enabled": True},
    },
    # link_status healths switch to bus1 (CAN3) WARN, bus2 (CAN1) OK.
    "can-health": {
        "profile": {"enabled": True},
        "link_status": {"enabled": True, "values": dict(LINK_UP_HEALTH)},
    },
    # Teensy-uplink loss with the bridge node still alive: profile keeps
    # flowing at 1 Hz with the SAME frozen values and link_status keeps
    # flowing at 10 Hz, but bridge_link='LOST' — the panel must refuse the
    # cached-republish deception ('--' readouts, STALE badge, UNKNOWN dots).
    "can-linkdown": {
        "profile": {"enabled": True},
        "link_status": {"enabled": True, "values": dict(LINK_LOST_FROZEN)},
    },
    # BOTH publishers stop (bridge node / rosbridge / subscription death) ->
    # the >3 s message watchdogs must fire: '--' + STALE badge + UNKNOWN dots.
    "can-staleness": {},
    # robot_state + leg echo flowing -> tracking-error panel shows values.
    "pos-cmd-echo": {
        "robot_state": {"enabled": True},
        "leg_setpoint_echo": {"enabled": True},
    },
    # echo stops (>1 s watchdog), robot_state keeps ticking -> legs go '--'.
    "pos-cmd-echo-stale": {
        "robot_state": {"enabled": True},
    },
    # orchestrator_state 'IDLE' -> right-sidebar state badge (minimap pre-wire).
    "orchestrator-state": {
        "robot_state": {"enabled": True},
        "orchestrator_state": {"enabled": True, "data": "IDLE"},
    },
}


def log(msg: str) -> None:
    sys.stderr.write("[gui_synthetic_stack] %s\n" % msg)
    sys.stderr.flush()


def emit(obj) -> None:
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


def port_bound(port: int) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.5)
    try:
        return s.connect_ex(("127.0.0.1", port)) == 0
    finally:
        s.close()


def find_ros_executable(package: str, executable: str) -> str:
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(":"):
        if not prefix:
            continue
        path = os.path.join(prefix, "lib", package, executable)
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    raise RuntimeError(
        "%s/%s not found on AMENT_PREFIX_PATH — is the ROS2 env sourced?"
        % (package, executable))


# ---------------------------------------------------------------------------
# Safety rails
# ---------------------------------------------------------------------------

_lock_fh = None


def acquire_lock() -> None:
    """Single-instance lockfile (flock, auto-released on process death).

    Opened O_CREAT|O_RDWR — NOT truncating — so a refused second instance
    cannot wipe the running holder's pid breadcrumb; the file is truncated
    and the pid written only once the lock is actually held.
    """
    global _lock_fh
    os.makedirs(os.path.dirname(LOCKFILE), exist_ok=True)
    fd = os.open(LOCKFILE, os.O_CREAT | os.O_RDWR, 0o644)
    _lock_fh = os.fdopen(fd, "r+")
    try:
        fcntl.flock(_lock_fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        holder = _lock_fh.readline().strip()
        _lock_fh.close()
        raise RuntimeError(
            "another gui_synthetic_stack instance (pid %s) holds %s — "
            "refusing to start" % (holder or "?", LOCKFILE))
    _lock_fh.truncate(0)
    _lock_fh.seek(0)
    _lock_fh.write(str(os.getpid()) + "\n")
    _lock_fh.flush()


def check_graph_empty(rclpy) -> None:
    """REFUSE to start if any real ROS2 node is up.

    Uses a throwaway rclpy node's discovery (two samples 1.5 s apart, union)
    rather than the slow ros2 CLI.  Hidden nodes (leading underscore — e.g.
    the ros2cli daemon) are ignored; anything else means a real stack is
    running and we must not inject synthetic telemetry into it.
    """
    node = rclpy.create_node("_gui_synth_precheck")
    try:
        seen = set()
        for _ in range(2):
            time.sleep(1.5)
            seen.update(node.get_node_names())
        others = sorted(n for n in seen
                        if n != "_gui_synth_precheck" and not n.startswith("_"))
        if others:
            raise RuntimeError(
                "ROS graph is NOT empty (real stack running?): %s — refusing "
                "to start. Shut the real stack down first." % ", ".join(others))
    finally:
        node.destroy_node()


# ---------------------------------------------------------------------------
# rosbridge / rosapi child processes
# ---------------------------------------------------------------------------

_children = []  # list of (name, Popen)


def launch_rosbridge(log_dir: str) -> None:
    """Replicates jugglebot_launch.py's rosbridge_websocket + rosapi Nodes
    (and ONLY those two — no hardware nodes)."""
    os.makedirs(log_dir, exist_ok=True)
    specs = [
        ("rosbridge_websocket",
         [find_ros_executable("rosbridge_server", "rosbridge_websocket"),
          "--ros-args", "-r", "__node:=rosbridge_websocket",
          "-p", "port:=%d" % ROSBRIDGE_PORT,
          "-p", "retry_startup_delay:=5.0",
          "-p", "websocket_ping_interval:=10",
          "-p", "websocket_ping_timeout:=30"]),
        ("rosapi",
         [find_ros_executable("rosapi", "rosapi_node"),
          "--ros-args", "-r", "__node:=rosapi"]),
    ]
    for name, cmd in specs:
        out = open(os.path.join(log_dir, name + ".log"), "w")
        proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT,
                                start_new_session=True)
        _children.append((name, proc))
        log("launched %s (pid %d)" % (name, proc.pid))

    deadline = time.time() + 30.0
    while time.time() < deadline:
        if _children[0][1].poll() is not None:
            raise RuntimeError("rosbridge_websocket exited early — see %s"
                               % os.path.join(log_dir, "rosbridge_websocket.log"))
        if port_bound(ROSBRIDGE_PORT):
            log("rosbridge accepting on :%d" % ROSBRIDGE_PORT)
            return
        time.sleep(0.25)
    raise RuntimeError("rosbridge did not bind :%d within 30 s" % ROSBRIDGE_PORT)


_torn_down = False


def teardown() -> None:
    """Idempotent: kill rosbridge + rosapi process groups (SIGINT, then
    SIGKILL after a grace period)."""
    global _torn_down
    if _torn_down:
        return
    _torn_down = True
    for name, proc in _children:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except OSError:
                pass
    deadline = time.time() + 8.0
    for name, proc in _children:
        while proc.poll() is None and time.time() < deadline:
            time.sleep(0.2)
        if proc.poll() is None:
            log("SIGKILL %s (pid %d)" % (name, proc.pid))
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                pass
        proc.wait()
        log("%s exited (rc %s)" % (name, proc.returncode))


# ---------------------------------------------------------------------------
# The fake-stack node
# ---------------------------------------------------------------------------

def make_node(rclpy):
    from builtin_interfaces.msg import Time  # noqa: F401  (via clock .to_msg())
    from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
    from jugglebot_interfaces.msg import (HandTelemetryMessage,
                                          MotorStateSingle, RobotState)
    from std_msgs.msg import Float64MultiArray, String
    from std_srvs.srv import SetBool, Trigger

    class GuiSyntheticStack(rclpy.node.Node):  # type: ignore[name-defined]
        # Nodes that are LEGITIMATELY in the graph while the harness runs:
        # ourselves + the two processes we launch.  rosapi_params is created
        # INTERNALLY by Foxy's rosapi_node alongside 'rosapi' (empirically
        # tripped the watchdog as a false positive on 2026-07-11 until
        # allowed).  Hidden nodes (leading underscore, e.g. the ros2cli
        # daemon) are also tolerated.
        ALLOWED_NODES = {"gui_synthetic_stack", "rosbridge_websocket",
                         "rosapi", "rosapi_params"}

        def __init__(self):
            super().__init__("gui_synthetic_stack")
            self._cfg_lock = threading.Lock()
            self.cfg = copy.deepcopy(BASELINE_CFG)
            self._next_due = {}
            self._breached = False
            self.records = {"services": [], "orchestrator_command": []}
            self.fake_state = {"setpoint_output_enabled": False,
                               "errors_cleared_count": 0}

            # GUI-consumed telemetry publishers ONLY (containment rail #3).
            self.pub = {
                "robot_state": self.create_publisher(RobotState, "robot_state", 10),
                "orchestrator_state": self.create_publisher(String, "orchestrator_state", 10),
                "control_mode_topic": self.create_publisher(String, "control_mode_topic", 10),
                "profile": self.create_publisher(DiagnosticStatus, "profile", 10),
                "link_status": self.create_publisher(DiagnosticStatus, "link_status", 10),
                "hand_telemetry": self.create_publisher(HandTelemetryMessage, "hand_telemetry", 10),
                "leg_setpoint_echo": self.create_publisher(Float64MultiArray, "leg_setpoint_echo", 10),
            }
            self._msg = {
                "robot_state": self._make_robot_state,
                "orchestrator_state": self._make_string("orchestrator_state"),
                "control_mode_topic": self._make_string("control_mode_topic"),
                "profile": self._make_diag("profile", "can_hub_profile"),
                "link_status": self._make_diag("link_status", "can_hub_link"),
                "hand_telemetry": self._make_hand_telemetry,
                "leg_setpoint_echo": self._make_leg_echo,
            }
            self._DiagnosticStatus = DiagnosticStatus
            self._KeyValue = KeyValue
            self._RobotState = RobotState
            self._MotorStateSingle = MotorStateSingle
            self._HandTelemetry = HandTelemetryMessage
            self._String = String
            self._Float64MultiArray = Float64MultiArray

            # Recorder services (mutate fake state only; see module docstring).
            self.create_service(SetBool, "set_setpoint_output", self._svc_set_setpoint)
            for name in ("clear_errors", "home", "trajectory/go_home", "trajectory/hold"):
                self.create_service(Trigger, name, self._make_trigger_recorder(name))

            # Recorder subscription: what did the GUI command?
            self.create_subscription(String, "orchestrator_command",
                                     self._on_orchestrator_command, 10)

            self.create_timer(0.01, self._tick)  # 100 Hz publish scheduler

            # Continuous containment re-check (~1 Hz): the startup
            # graph-empty rail alone would not stop an operator launching
            # the REAL stack mid-run — trajectory_node genuinely subscribes
            # robot_state + link_status and would ingest our fabricated
            # is_homed=True telemetry.  On any foreign node: publishers off
            # first, then FATAL + full teardown.
            self.create_timer(1.0, self._containment_watchdog)

        # ---- scenario engine -------------------------------------------
        def apply_stage(self, name: str) -> None:
            if name not in STAGES:
                raise KeyError("unknown stage %r (have: %s)"
                               % (name, ", ".join(sorted(STAGES))))
            cfg = copy.deepcopy(BASELINE_CFG)
            for topic, patch in STAGES[name].items():
                cfg[topic].update(copy.deepcopy(patch))
            with self._cfg_lock:
                if self._breached:
                    raise RuntimeError(
                        "containment breached — refusing to re-enable publishers")
                self.cfg = cfg
                self._next_due = {}  # publish enabled topics immediately
            log("stage applied: %s" % name)

        def _containment_watchdog(self) -> None:
            """~1 Hz: any foreign node in the graph => publishers off, FATAL,
            teardown (via SIGTERM to ourselves -> signal handler -> atexit)."""
            if self._breached:
                return
            foreign = sorted(
                n for n in set(self.get_node_names())
                if n not in self.ALLOWED_NODES and not n.startswith("_"))
            if not foreign:
                return
            with self._cfg_lock:
                self._breached = True          # hard-stops _tick publishing
                for c in self.cfg.values():
                    c["enabled"] = False
            log("FATAL: containment breach — foreign node(s) appeared in the "
                "graph mid-run: %s. All publishers disabled; tearing down."
                % ", ".join(foreign))
            emit({"event": "containment_breach", "nodes": foreign})
            # SIGTERM ourselves: the registered handler sys.exit()s the main
            # thread out of the stdin loop -> finally/atexit -> teardown().
            os.kill(os.getpid(), signal.SIGTERM)

        def _tick(self) -> None:
            now = time.monotonic()
            with self._cfg_lock:
                if self._breached:
                    return
                cfg = self.cfg
                due = [t for t, c in cfg.items()
                       if c["enabled"] and now >= self._next_due.get(t, 0.0)]
                for t in due:
                    self._next_due[t] = now + cfg[t]["period"]
            for t in due:
                try:
                    self.pub[t].publish(self._msg[t](cfg[t]))
                except Exception as exc:  # pragma: no cover — surface, don't die
                    log("publish %s failed: %r" % (t, exc))

        # ---- message builders ------------------------------------------
        def _make_diag(self, topic, diag_name):
            def build(c):
                msg = self._DiagnosticStatus()
                msg.level = self._DiagnosticStatus.OK  # byte field on Foxy
                msg.name = diag_name
                msg.hardware_id = "gui_synthetic_stack"
                msg.values = [self._KeyValue(key=str(k), value=str(v))
                              for k, v in c["values"].items()]
                return msg
            return build

        def _make_string(self, topic):
            def build(c):
                return self._String(data=str(c["data"]))
            return build

        def _make_leg_echo(self, c):
            return self._Float64MultiArray(data=[float(v) for v in c["data"]])

        def _make_hand_telemetry(self, c):
            msg = self._HandTelemetry()
            msg.timestamp = self.get_clock().now().to_msg()
            msg.pos_cmd = float(c["pos_cmd"])
            msg.pos_meas = float(c["pos_meas"])
            return msg

        def _make_robot_state(self, c):
            msg = self._RobotState()
            msg.timestamp = self.get_clock().now().to_msg()
            positions = list(c["leg_pos"]) + [float(c["hand_pos"]), 0.0, 0.0]
            for pos in positions:  # 6 legs + hand + 2 BB axes = 9 motors
                m = self._MotorStateSingle()
                m.active_errors = 0
                m.disarm_reason = 0
                m.current_state = 8       # CLOSED_LOOP_CONTROL
                m.procedure_result = 0
                m.trajectory_done = True
                m.pos_estimate = float(pos)
                m.vel_estimate = 0.0
                m.iq_setpoint = 0.1
                m.iq_measured = 0.1
                m.fet_temp = 35.0
                m.motor_temp = 0.0
                m.bus_voltage = float(c["bus_voltage"])
                m.bus_current = 0.2
                msg.motor_states.append(m)
            msg.error = []
            msg.has_fatal_odrive_error = False
            msg.has_fatal_can_error = False
            msg.has_undervoltage = False
            msg.firmware_validated = True
            msg.encoder_search_complete = True
            msg.is_homed = True
            msg.levelling_complete = True
            msg.pose_offset_rad = [0.0, 0.0]
            msg.pose_offset_quat.w = 1.0
            return msg

        # ---- recorders ---------------------------------------------------
        def _svc_set_setpoint(self, request, response):
            self.fake_state["setpoint_output_enabled"] = bool(request.data)
            self.records["services"].append(
                {"t": time.time(), "service": "set_setpoint_output",
                 "data": bool(request.data)})
            log("recorder service: set_setpoint_output(%s)" % request.data)
            response.success = True
            response.message = "gui_synthetic_stack recorder (fake state only)"
            return response

        def _make_trigger_recorder(self, name):
            def handle(request, response):
                if name == "clear_errors":
                    self.fake_state["errors_cleared_count"] += 1
                self.records["services"].append(
                    {"t": time.time(), "service": name})
                log("recorder service: %s()" % name)
                response.success = True
                response.message = "gui_synthetic_stack recorder (fake state only)"
                return response
            return handle

        def _on_orchestrator_command(self, msg):
            self.records["orchestrator_command"].append(
                {"t": time.time(), "data": msg.data})
            log("recorded orchestrator_command: %r" % msg.data)

    return GuiSyntheticStack()


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fake ROS2 stack + rosbridge for GUI verification "
                    "(refuses to start beside a real stack)")
    parser.add_argument("--log-dir",
                        default=os.path.join(REPO_ROOT, "temp", "probes", "gui_dom"),
                        help="where rosbridge/rosapi child logs go")
    parser.add_argument("--stage", default=None,
                        help="apply this stage at startup (manual use)")
    args = parser.parse_args()

    try:
        acquire_lock()
    except RuntimeError as exc:
        log("FATAL: %s" % exc)
        return 2
    if port_bound(ROSBRIDGE_PORT):
        log("FATAL: :%d already bound (real rosbridge running?)" % ROSBRIDGE_PORT)
        return 2

    import rclpy  # deferred: clearer error if env not sourced
    import rclpy.node  # noqa: F401
    rclpy.init()

    try:
        check_graph_empty(rclpy)
    except RuntimeError as exc:
        log("FATAL: %s" % exc)
        rclpy.shutdown()
        return 2

    atexit.register(teardown)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: sys.exit(1))  # -> atexit runs

    launch_rosbridge(args.log_dir)

    node = make_node(rclpy)
    if args.stage:
        node.apply_stage(args.stage)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    emit({"event": "ready",
          "rosbridge_pid": _children[0][1].pid,
          "rosapi_pid": _children[1][1].pid,
          "stages": sorted(STAGES)})
    log("ready — reading stdin protocol")

    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                req = json.loads(line)
            except ValueError:
                emit({"ok": False, "error": "bad json: %r" % line})
                continue
            cmd = req.get("cmd")
            try:
                if cmd == "stage":
                    node.apply_stage(req["name"])
                    emit({"ok": True, "stage": req["name"]})
                elif cmd == "records":
                    emit({"ok": True, "records": node.records,
                          "fake_state": node.fake_state})
                elif cmd == "ping":
                    emit({"ok": True})
                elif cmd == "quit":
                    emit({"ok": True, "quitting": True})
                    break
                else:
                    emit({"ok": False, "error": "unknown cmd %r" % cmd})
            except Exception as exc:
                emit({"ok": False, "error": repr(exc)})
        # stdin EOF (probe died) also lands here -> teardown below.
    finally:
        log("shutting down")
        rclpy.shutdown()          # stops the spin thread first ...
        spin_thread.join(timeout=5.0)
        try:
            node.destroy_node()   # ... so destroy can't race the executor
        except Exception:
            pass
        teardown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
