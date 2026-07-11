#!/usr/bin/env python3
"""
gui_dom_probe.py — Live behavioural verification of the Jugglebot GUI against
a REAL rosbridge websocket, with NO hardware and NO real robot nodes.

One command runs the whole gate::

    python3 tools/probes/gui_dom_probe.py --scenario scenario1

What it does:
  1. Spawns ``tools/probes/gui_synthetic_stack.py`` through the ROS2 Foxy env
     (that script enforces the containment rails: refuses to start unless the
     ROS graph is EMPTY and :9090 is free; launches ONLY rosbridge_websocket +
     rosapi; publishes ONLY GUI-consumed telemetry; serves recorder services).
  2. Launches headless chromium (snap, ``--headless=new``) with a fresh
     profile and connects to it over the Chrome DevTools Protocol.  Transport
     is tornado's websocket client (present in both the system python3 and the
     project venv — no pip installs).
  3. Loads the REAL GUI from the already-running systemd server on :8081
     (never restarts it), steps the synthetic stack through scripted scenario
     stages, and polls DOM state via ``Runtime.evaluate`` until each assertion
     passes or times out.  Console errors + uncaught page exceptions are
     captured for the whole run (``Runtime`` + ``Log`` domains).
  4. Writes a PASS/FAIL report + per-stage DOM snapshots + console log to
     ``temp/probes/gui_dom/<timestamp>/``.  Exit 0 = all assertions passed.

Interactive tier: CDP ``Input.dispatchMouseEvent`` dispatches TRUSTED pointer
events, so ``pointer_hold()``/``click_element()`` drive hold-to-confirm
buttons and SVG minimap nodes (``ros_ws/gui/js/hold-to-confirm.js`` listens
for pointerdown/pointerup).  ``--scenario minimap`` exercises the orchestrator
state-minimap end-to-end with a REACTIVE fake orchestrator — the stack's
'set' beats respond to the GUI's recorded commands — pinning the
safety-critical teardown order (standby -> go_home -> disarm+VERIFY ->
deactivate; deactivate hard-gated on mpc_active=0) plus its abort NEGATIVE,
Arm gating, HOMING command-discarding, greyed reasons, and disconnect
greying.  See ``scenario_minimap()``.

Environment dependencies (verified 2026-07-11 on the Jetson):
  * /snap/bin/chromium supports ``--headless=new`` + CDP.
  * three.js is loaded from cdn.jsdelivr.net (index.html importmap) — the GUI
    cannot boot offline; the probe pre-checks CDN reachability and fails fast.
  * GUI static server live on :8081 (systemd jugglebot-gui.service).

Motivating logbook entries:
  * logbook/2026-07-11-gui-leg-setpoint-echo-poscmd.md  (pos_cmd datasource)
  * logbook/2026-07-11-gui-can-traffic-per-bus-panel.md (per-bus CAN panel)
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import select
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STACK_SCRIPT = os.path.join(REPO_ROOT, "tools", "probes", "gui_synthetic_stack.py")
DEFAULT_OUT_ROOT = os.path.join(REPO_ROOT, "temp", "probes", "gui_dom")
DEFAULT_GUI_URL = "http://localhost:8081"
DEFAULT_CHROMIUM = "/snap/bin/chromium"
THREE_CDN_URL = "https://cdn.jsdelivr.net/npm/three@0.170.0/build/three.module.js"
ROS_SETUP = "/opt/ros/foxy/setup.bash"
WS_SETUP = os.path.join(REPO_ROOT, "ros_ws", "install", "setup.bash")

POLL_INTERVAL_S = 0.3


def log(msg: str) -> None:
    sys.stderr.write("[gui_dom_probe] %s\n" % msg)
    sys.stderr.flush()


def port_bound(port: int) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.5)
    try:
        return s.connect_ex(("127.0.0.1", port)) == 0
    finally:
        s.close()


# ---------------------------------------------------------------------------
# CDP client (tornado websocket transport)
# ---------------------------------------------------------------------------

class CDPError(Exception):
    pass


class CDPClient:
    """Minimal Chrome DevTools Protocol client over a tornado websocket."""

    def __init__(self):
        self._conn = None
        self._reader = None
        self._next_id = 0
        self._pending = {}
        self._event_waiters = {}
        self.page_exceptions = []   # Runtime.exceptionThrown
        self.console_errors = []    # console.error / console.assert
        self.console_all = []       # every consoleAPICalled
        self.log_entries = []       # Log.entryAdded (level >= warning)

    async def connect(self, ws_url: str) -> None:
        from tornado.websocket import websocket_connect
        self._conn = await websocket_connect(
            ws_url, connect_timeout=10, max_message_size=64 * 1024 * 1024)
        self._reader = asyncio.ensure_future(self._read_loop())

    async def _read_loop(self) -> None:
        while True:
            raw = await self._conn.read_message()
            if raw is None:
                for fut in self._pending.values():
                    if not fut.done():
                        fut.set_exception(CDPError("CDP socket closed"))
                self._pending.clear()
                return
            msg = json.loads(raw)
            if "id" in msg:
                fut = self._pending.pop(msg["id"], None)
                if fut is not None and not fut.done():
                    fut.set_result(msg)
            elif "method" in msg:
                self._on_event(msg["method"], msg.get("params", {}))

    def _on_event(self, method: str, params: dict) -> None:
        if method == "Runtime.exceptionThrown":
            det = params.get("exceptionDetails", {})
            exc = det.get("exception", {}) or {}
            self.page_exceptions.append({
                "t": time.time(),
                "text": det.get("text", ""),
                "description": exc.get("description", exc.get("value", "")),
                "url": det.get("url", ""),
                "line": det.get("lineNumber"),
            })
        elif method == "Runtime.consoleAPICalled":
            args = params.get("args", [])
            text = " ".join(
                str(a.get("value", a.get("description", "<obj>"))) for a in args)
            entry = {"t": time.time(), "type": params.get("type"), "text": text}
            self.console_all.append(entry)
            if params.get("type") in ("error", "assert"):
                self.console_errors.append(entry)
        elif method == "Log.entryAdded":
            e = params.get("entry", {})
            if e.get("level") in ("warning", "error"):
                self.log_entries.append({
                    "t": time.time(), "level": e.get("level"),
                    "source": e.get("source"), "text": e.get("text", ""),
                    "url": e.get("url", "")})
        waiters = self._event_waiters.pop(method, [])
        for fut in waiters:
            if not fut.done():
                fut.set_result(params)

    async def call(self, method: str, params: dict = None, timeout: float = 20.0):
        self._next_id += 1
        msg_id = self._next_id
        fut = asyncio.get_event_loop().create_future()
        self._pending[msg_id] = fut
        payload = {"id": msg_id, "method": method}
        if params:
            payload["params"] = params
        await self._conn.write_message(json.dumps(payload))
        msg = await asyncio.wait_for(fut, timeout)
        if "error" in msg:
            raise CDPError("%s -> %s" % (method, msg["error"]))
        return msg.get("result", {})

    def event_future(self, method: str):
        """Register BEFORE triggering the action that fires the event."""
        fut = asyncio.get_event_loop().create_future()
        self._event_waiters.setdefault(method, []).append(fut)
        return fut

    def close(self) -> None:
        if self._reader:
            self._reader.cancel()
        if self._conn:
            self._conn.close()


async def evaluate(cdp: CDPClient, expression: str):
    res = await cdp.call("Runtime.evaluate",
                         {"expression": expression, "returnByValue": True})
    det = res.get("exceptionDetails")
    if det:
        exc = det.get("exception", {}) or {}
        return {"__js_error__": exc.get("description", det.get("text", "js error"))}
    return res.get("result", {}).get("value")


async def _element_center(cdp: CDPClient, selector: str):
    rect = await evaluate(cdp, (
        "(() => { const e = document.querySelector(%s); if (!e) return null;"
        " if (e.scrollIntoView) e.scrollIntoView({block: 'center'});"
        " const r = e.getBoundingClientRect();"
        " return {x: r.x + r.width / 2, y: r.y + r.height / 2}; })()"
        % json.dumps(selector)))
    if not rect or (isinstance(rect, dict) and "__js_error__" in rect):
        raise CDPError("element %r not found (%r)" % (selector, rect))
    return rect


async def pointer_hold(cdp: CDPClient, selector: str, hold_ms: int = 1000) -> None:
    """Trusted synthetic pointer hold on an element's centre (Tier B).

    CDP Input.dispatchMouseEvent goes through Chrome's real input pipeline,
    so the page receives trusted pointerdown -> (hold) -> pointerup — exactly
    what hold-to-confirm.js needs (fires at 800 ms; the 1000 ms default
    comfortably exceeds it).  Works on HTML buttons and SVG <g> nodes alike.
    Holds on elements with a truthy `disabled` (including the minimap's
    expando property) are inert by hold-to-confirm's own gate — that
    inertness is itself a behaviour under test (scenario 'minimap' M4)."""
    c = await _element_center(cdp, selector)
    base = {"x": c["x"], "y": c["y"], "button": "left",
            "clickCount": 1, "pointerType": "mouse"}
    await cdp.call("Input.dispatchMouseEvent",
                   dict(base, type="mousePressed", buttons=1))
    await asyncio.sleep(hold_ms / 1000.0)
    await cdp.call("Input.dispatchMouseEvent",
                   dict(base, type="mouseReleased", buttons=0))


async def click_element(cdp: CDPClient, selector: str) -> None:
    """Plain trusted click (press+release, no hold) — e.g. #minimap-header."""
    c = await _element_center(cdp, selector)
    base = {"x": c["x"], "y": c["y"], "button": "left",
            "clickCount": 1, "pointerType": "mouse"}
    await cdp.call("Input.dispatchMouseEvent",
                   dict(base, type="mousePressed", buttons=1))
    await asyncio.sleep(0.05)
    await cdp.call("Input.dispatchMouseEvent",
                   dict(base, type="mouseReleased", buttons=0))


# ---------------------------------------------------------------------------
# Synthetic-stack subprocess controller
# ---------------------------------------------------------------------------

class SyntheticStack:
    """Spawns gui_synthetic_stack.py through the ROS env and drives its
    stdin/stdout line-JSON protocol."""

    def __init__(self, run_dir: str):
        self.run_dir = run_dir
        self.proc = None
        self.stderr_path = os.path.join(run_dir, "stack-stderr.log")
        self.ready_info = None

    def start(self, timeout: float = 120.0) -> None:
        bash_cmd = (
            'source "%s" && source "%s" && exec python3 "%s" --log-dir "%s"'
            % (ROS_SETUP, WS_SETUP, STACK_SCRIPT, self.run_dir))
        self._stderr_fh = open(self.stderr_path, "w")
        self.proc = subprocess.Popen(
            ["bash", "-c", bash_cmd],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=self._stderr_fh, text=True, bufsize=1,
            start_new_session=True)
        log("synthetic stack spawned (pid %d), waiting for ready ..." % self.proc.pid)
        deadline = time.monotonic() + timeout
        while True:
            msg = self._read_json(deadline)
            if msg.get("event") == "ready":
                self.ready_info = msg
                log("synthetic stack ready (rosbridge pid %s)"
                    % msg.get("rosbridge_pid"))
                return

    def _read_json(self, deadline: float) -> dict:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("synthetic stack: timeout waiting for reply%s"
                                   % self._stderr_tail())
            if self.proc.poll() is not None:
                raise RuntimeError(
                    "synthetic stack exited early (rc %s)%s"
                    % (self.proc.returncode, self._stderr_tail()))
            rl, _, _ = select.select([self.proc.stdout], [], [], min(remaining, 0.5))
            if not rl:
                continue
            line = self.proc.stdout.readline()
            if line == "":
                raise RuntimeError("synthetic stack: stdout EOF%s" % self._stderr_tail())
            line = line.strip()
            if not line:
                continue
            try:
                return json.loads(line)
            except ValueError:
                log("stack emitted non-JSON line: %r" % line)

    def _stderr_tail(self) -> str:
        try:
            with open(self.stderr_path) as fh:
                tail = fh.readlines()[-15:]
            return "\n--- stack stderr tail ---\n" + "".join(tail)
        except OSError:
            return ""

    def _request(self, obj: dict, timeout: float = 15.0) -> dict:
        self.proc.stdin.write(json.dumps(obj) + "\n")
        self.proc.stdin.flush()
        deadline = time.monotonic() + timeout
        while True:
            msg = self._read_json(deadline)
            if "ok" in msg:
                if not msg["ok"]:
                    raise RuntimeError("stack refused %r: %s" % (obj, msg.get("error")))
                return msg
            log("stack event while waiting for ack: %r" % msg)

    def stage(self, name: str) -> None:
        self._request({"cmd": "stage", "name": name})
        log("stack stage -> %s" % name)

    def set(self, topic: str, config: dict) -> None:
        """Reactive mid-sequence beat: merge a live config patch into one
        topic (publishes immediately)."""
        self._request({"cmd": "set", "topic": topic, "config": config})
        log("stack set %s <- %s" % (topic, config))

    def records(self) -> dict:
        return self._request({"cmd": "records"})

    def stop_rosbridge(self) -> None:
        """Kill rosbridge only (GUI-disconnect scenario); the stack node and
        rosapi stay up so records remain queryable."""
        self._request({"cmd": "stop_rosbridge"})
        log("stack stop_rosbridge requested")

    def quit_and_wait(self, timeout: float = 10.0) -> bool:
        """Graceful quit with a teardown-preserving escalation ladder.

        1. protocol quit -> the stack's own finally-block teardown.
        2. SIGTERM -> the stack's signal handler sys.exit()s, so its atexit
           teardown STILL kills rosbridge/rosapi.
        3. SIGKILL (last resort) BYPASSES the stack's teardown — and
           rosbridge/rosapi run in their OWN sessions (start_new_session), so
           killing the stack's group misses them: after any SIGKILL we
           directly killpg the rosbridge/rosapi pids recorded at 'ready'.
        Returns True if the stack exited without needing SIGKILL.
        """
        if self.proc is None:
            return True
        if self.proc.poll() is not None:
            return True
        clean = False
        try:
            self._request({"cmd": "quit"}, timeout=10.0)
        except Exception as exc:
            log("quit request failed (%r) — escalating" % exc)
        try:
            self.proc.wait(timeout=timeout)
            clean = True
        except subprocess.TimeoutExpired:
            log("stack ignored quit — SIGTERM (its atexit teardown still runs)")
            try:
                os.killpg(self.proc.pid, signal.SIGTERM)
            except OSError:
                pass
            try:
                self.proc.wait(timeout=10.0)
                clean = True
            except subprocess.TimeoutExpired:
                log("stack ignored SIGTERM — SIGKILL + direct child cleanup")
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except OSError:
                    pass
                self.proc.wait()
                # SIGKILL bypassed the stack's teardown: reap its children
                # ourselves (each is its own session leader -> pgid == pid).
                for key in ("rosbridge_pid", "rosapi_pid"):
                    pid = (self.ready_info or {}).get(key)
                    if pid is None:
                        continue
                    try:
                        os.killpg(int(pid), signal.SIGKILL)
                        log("killed orphaned %s (pgid %s)" % (key, pid))
                    except OSError:
                        pass  # already gone
        return clean


# ---------------------------------------------------------------------------
# Chromium controller
# ---------------------------------------------------------------------------

class Chromium:
    def __init__(self, binary: str, run_dir: str):
        self.binary = binary
        self.profile_dir = os.path.join(run_dir, "chrome-profile")
        self.log_path = os.path.join(run_dir, "chromium.log")
        self.proc = None
        self.port = None

    def start(self, timeout: float = 45.0) -> None:
        os.makedirs(self.profile_dir, exist_ok=True)
        cmd = [self.binary, "--headless=new", "--disable-gpu", "--no-sandbox",
               "--remote-debugging-port=0", "--remote-allow-origins=*",
               "--user-data-dir=" + self.profile_dir,
               "--window-size=1600,1000", "--mute-audio", "--no-first-run",
               "--disable-extensions", "about:blank"]
        out = open(self.log_path, "w")
        self.proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        port_file = os.path.join(self.profile_dir, "DevToolsActivePort")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("chromium exited early (rc %s) — see %s"
                                   % (self.proc.returncode, self.log_path))
            if os.path.isfile(port_file) and os.path.getsize(port_file) > 0:
                with open(port_file) as fh:
                    self.port = int(fh.readline().strip())
                log("chromium CDP on 127.0.0.1:%d (pid %d)"
                    % (self.port, self.proc.pid))
                return
            time.sleep(0.25)
        raise RuntimeError("chromium: DevToolsActivePort never appeared")

    def page_ws_url(self, timeout: float = 20.0) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen(
                        "http://127.0.0.1:%d/json/list" % self.port, timeout=5) as r:
                    targets = json.loads(r.read().decode())
                pages = [t for t in targets if t.get("type") == "page"]
                if pages:
                    return pages[0]["webSocketDebuggerUrl"]
            except (urllib.error.URLError, OSError):
                pass
            time.sleep(0.5)
        raise RuntimeError("chromium: no page target on /json/list")

    def kill(self) -> None:
        if self.proc is None or self.proc.poll() is not None:
            return
        try:
            os.killpg(self.proc.pid, signal.SIGTERM)
        except OSError:
            pass
        try:
            self.proc.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except OSError:
                pass
            self.proc.wait()


# ---------------------------------------------------------------------------
# Assertion framework
# ---------------------------------------------------------------------------

# Null-safe DOM getter shared by every probe expression.
JS_GETTER = (
    "const g = (id) => { const e = document.getElementById(id);"
    " return e ? { cls: e.className, txt: e.textContent,"
    " disp: e.style.display, title: e.title || '',"
    " dis: ('disabled' in e) ? !!e.disabled : null }"
    " : null; };")


def js_probe(body: str) -> str:
    return "(() => { %s return %s; })()" % (JS_GETTER, body)


class Assertion:
    def __init__(self, name, js, check, timeout=8.0, kind="dom"):
        self.name = name
        self.js = js
        self.check = check          # value -> (ok: bool, evidence: str)
        self.timeout = timeout
        self.kind = kind            # 'dom' | 'page-errors'


class StageSpec:
    def __init__(self, name, stack_stage, assertions):
        self.name = name
        self.stack_stage = stack_stage
        self.assertions = assertions


def _ev(value) -> str:
    s = json.dumps(value, default=repr)
    return s if len(s) <= 400 else s[:400] + "...<truncated>"


def _safe(check):
    """Wrap a check so a None/missing-key during polling counts as
    not-yet-passing instead of crashing the poll loop."""
    def wrapped(value):
        try:
            return check(value)
        except (TypeError, KeyError, AttributeError, IndexError):
            return False, _ev(value)
    return wrapped


async def poll_dom(cdp, assertion):
    deadline = time.monotonic() + assertion.timeout
    started = time.monotonic()
    while True:
        value = await evaluate(cdp, assertion.js)
        ok, evidence = assertion.check(value)
        elapsed = time.monotonic() - started
        if ok:
            return True, evidence, elapsed
        if time.monotonic() >= deadline:
            return False, evidence, elapsed
        await asyncio.sleep(POLL_INTERVAL_S)


# ---------------------------------------------------------------------------
# scenario1 — connection, CAN panel (slot mapping / health / staleness),
#             pos-cmd echo tracking panel, orchestrator state badge
# ---------------------------------------------------------------------------

# Expected values derive from the synthetic stack's scripted fixtures
# (gui_synthetic_stack.PROFILE_DISTINCT / LINK_UP_UNKNOWN / LINK_UP_HEALTH /
# LINK_LOST_FROZEN / LEG_*_REV) and the GUI's own derivations
# (can-traffic.js: msg/s = (rx+tx)/1 s, kbit/s = msg/s * 111 / 1000;
# panels.js updateTrackingError: mm formatting).

JS_CONN = js_probe("{ dot: g('conn-dot'), txt: g('conn-text') }")
JS_CAN = js_probe(
    "{ can3: {m: g('can-msgs-can3'), k: g('can-kbits-can3'), u: g('can-util-can3'),"
    "  h: g('can-health-can3')},"
    " can1: {m: g('can-msgs-can1'), k: g('can-kbits-can1'), u: g('can-util-can1'),"
    "  h: g('can-health-can1')},"
    " can2: {m: g('can-msgs-can2'), row: g('can-row-can2'), label: g('can-label-can2'),"
    "  h: g('can-health-can2')},"
    " badge: g('can-stale-badge') }")
JS_TRACK = js_probe(
    "{ vals: [0,1,2,3,4,5,6].map(i => g('track-val-' + i)) }")
JS_STATE = js_probe("{ badge: g('state-badge'), sub: g('state-sub-mode') }")

MM_RE = re.compile(r"^\d+\.\d mm$")


def check_connected(v):
    ok = ("connected" in v["dot"]["cls"].split()
          and v["txt"]["txt"] == "Connected")
    return ok, _ev(v)


def check_can3_slot(v):
    c = v["can3"]
    ok = (c["m"]["txt"] == "120" and c["k"]["txt"] == "13.3"
          and c["u"]["txt"] == "13.3")
    return ok, _ev({"can3": c})


def check_can1_slot(v):
    c = v["can1"]
    ok = (c["m"]["txt"] == "10" and c["k"]["txt"] == "1.1"
          and c["u"]["txt"] == "1.1")
    return ok, _ev({"can1": c})


def check_can2_na(v):
    c = v["can2"]
    ok = (c["m"]["txt"] == "n/a" and "can-row-na" in c["row"]["cls"].split()
          and c["label"]["dis"] is True)
    return ok, _ev({"can2": c})


def check_badge_hidden(v):
    return v["badge"]["disp"] == "none", _ev({"badge": v["badge"]})


def check_health_dots(v):
    ok = (v["can3"]["h"]["cls"] == "can-health-dot warn"
          and v["can1"]["h"]["cls"] == "can-health-dot ok"
          and v["can2"]["h"]["cls"] == "can-health-dot unknown")
    return ok, _ev({b: v[b]["h"] for b in ("can3", "can1", "can2")})


def check_stale_readouts(v):
    ok = all(v[b][f]["txt"] == "--"
             for b in ("can3", "can1") for f in ("m", "k", "u"))
    return ok, _ev({b: {f: v[b][f]["txt"] for f in ("m", "k", "u")}
                    for b in ("can3", "can1")})


def check_badge_visible(v):
    return v["badge"]["disp"] != "none", _ev({"badge": v["badge"]})


def check_linkdown_badge(v):
    """Badge visible AND its tooltip names the bridge_link cause — proves the
    linkDown path specifically (not a message watchdog) raised it."""
    ok = (v["badge"]["disp"] != "none"
          and "bridge_link=LOST" in v["badge"]["title"])
    return ok, _ev({"badge": v["badge"]})


def check_watchdog_badge(v):
    """Badge visible AND its tooltip names BOTH >3 s message watchdog causes.
    Discriminating on the tooltip matters here: scenario order enters
    can-staleness from can-linkdown, where badge/'--'/UNKNOWN are already
    showing — only the tooltip proves the profile and link_status watchdogs
    themselves fired after the streams stopped."""
    ok = (v["badge"]["disp"] != "none"
          and "no 'profile' from bridge >3 s" in v["badge"]["title"]
          and "no 'link_status' from bridge >3 s" in v["badge"]["title"])
    return ok, _ev({"badge": v["badge"]})


def check_dots_unknown(v):
    ok = all(v[b]["h"]["cls"] == "can-health-dot unknown" for b in ("can3", "can1"))
    return ok, _ev({b: v[b]["h"] for b in ("can3", "can1")})


def check_tracking_live(v):
    legs = [e["txt"] for e in v["vals"][:6]]
    ok = all(MM_RE.match(t) for t in legs)
    return ok, _ev({"legs": legs})


def check_hand_unknown(v):
    return v["vals"][6]["txt"] == "--", _ev({"hand": v["vals"][6]["txt"]})


def check_tracking_cleared(v):
    legs = [e["txt"] for e in v["vals"][:6]]
    return all(t == "--" for t in legs), _ev({"legs": legs})


def check_state_idle(v):
    ok = (v["badge"]["txt"] == "IDLE"
          and "idle" in v["badge"]["cls"].split()
          and v["sub"]["txt"] == "")
    return ok, _ev(v)


def check_no_page_errors(cdp):
    ok = not cdp.page_exceptions and not cdp.console_errors
    return ok, _ev({"exceptions": cdp.page_exceptions[:3],
                    "console_errors": cdp.console_errors[:3],
                    "counts": {"exceptions": len(cdp.page_exceptions),
                               "console_errors": len(cdp.console_errors)}})


SCENARIO1 = [
    StageSpec("connect", "connect", [
        # GUI shows connected within N s of rosbridge being up (page just
        # loaded; three.js CDN fetch + ROSLIB connect are inside this window).
        Assertion("gui-connected", JS_CONN, _safe(check_connected), timeout=30.0),
    ]),
    StageSpec("can-slot-mapping", "can-slot-mapping", [
        # KEYSTONE: wire slot can1_* (100+20 msg/s, util 13.3) must land on
        # the CAN3 'Jugglebot core' row; wire slot can2_* (7+3, util 1.1) on
        # the CAN1 'Ball Butler' row.  Catches any future slot swap.
        Assertion("can3-shows-can1-slot-values", JS_CAN, _safe(check_can3_slot), timeout=12.0),
        Assertion("can1-shows-can2-slot-values", JS_CAN, _safe(check_can1_slot), timeout=6.0),
        Assertion("can2-na-and-disabled", JS_CAN, _safe(check_can2_na), timeout=6.0),
        Assertion("stale-badge-hidden-while-fresh", JS_CAN, _safe(check_badge_hidden), timeout=6.0),
    ]),
    StageSpec("can-health", "can-health", [
        # link_status (bridge_link=UP) bus1=WARN -> CAN3 dot warn; bus2=OK ->
        # CAN1 dot ok.  Dots only show received healths while the uplink is UP.
        Assertion("health-dots-warn-ok-unknown", JS_CAN, _safe(check_health_dots), timeout=8.0),
    ]),
    StageSpec("can-linkdown", "can-linkdown", [
        # Cached-republish deception (the HIGH fix this stage pins): profile
        # keeps flowing at 1 Hz with the SAME values and link_status at 10 Hz
        # with frozen WARN/OK healths, but bridge_link=LOST — the panel must
        # blank readouts, raise the badge, and force dots UNKNOWN even though
        # fresh-looking messages are arriving.
        Assertion("linkdown-readouts-dashed", JS_CAN, _safe(check_stale_readouts), timeout=8.0),
        Assertion("linkdown-badge-names-cause", JS_CAN, _safe(check_linkdown_badge), timeout=6.0),
        Assertion("linkdown-dots-unknown-despite-fresh-healths", JS_CAN,
                  _safe(check_dots_unknown), timeout=6.0),
    ]),
    StageSpec("can-staleness", "can-staleness", [
        # BOTH streams stopped >3 s (bridge-node death) -> readouts '--',
        # STALE badge whose tooltip names both watchdog causes, dots unknown.
        Assertion("readouts-dashed-after-3s", JS_CAN, _safe(check_stale_readouts), timeout=12.0),
        Assertion("stale-badge-names-watchdog-causes", JS_CAN, _safe(check_watchdog_badge), timeout=12.0),
        Assertion("health-dots-unknown-after-stale", JS_CAN, _safe(check_dots_unknown), timeout=6.0),
    ]),
    StageSpec("pos-cmd-echo", "pos-cmd-echo", [
        # robot_state + leg_setpoint_echo flowing -> leg tracking slots show
        # real mm values (fixture: 0.02 rev / MM_TO_REV ~= 1.4 mm).
        Assertion("tracking-legs-live", JS_TRACK, _safe(check_tracking_live), timeout=12.0),
        # No hand_telemetry published -> hand slot must stay '--' (never 0).
        Assertion("tracking-hand-unknown", JS_TRACK, _safe(check_hand_unknown), timeout=4.0),
        # Charts must ingest without any page error (uncaught exception or
        # console.error) across everything so far.
        Assertion("no-page-errors", None, check_no_page_errors, kind="page-errors"),
    ]),
    StageSpec("pos-cmd-echo-stale", "pos-cmd-echo-stale", [
        # echo stopped >1 s (robot_state still ticking) -> legs return to '--'
        # (the committed staleness-clear behaviour, c6075db).
        Assertion("tracking-legs-cleared", JS_TRACK, _safe(check_tracking_cleared), timeout=8.0),
    ]),
    StageSpec("orchestrator-state", "orchestrator-state", [
        # orchestrator_state 'IDLE' -> right-sidebar badge (minimap pre-wire).
        Assertion("state-badge-idle", JS_STATE, _safe(check_state_idle), timeout=8.0),
    ]),
]


# ---------------------------------------------------------------------------
# scenario 'minimap' — Tier B interactive verification of the orchestrator
# state-minimap (ros_ws/gui/js/state-minimap.js): trusted pointer holds via
# CDP, a REACTIVE fake orchestrator (the stack's 'set' beats respond to the
# GUI's recorded commands so the sequencer sees states actually move), and
# record-order assertions that pin the SAFETY-CRITICAL teardown ordering
# (runbook Sharp Edges #1/#6: standby -> go_home -> disarm+VERIFY ->
# deactivate, with deactivate hard-gated on a fresh mpc_active=0).
# ---------------------------------------------------------------------------

# Fixture single-source-of-truth: import the stack module (stdlib-only at
# module level; rclpy imports are deferred inside its main()).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gui_synthetic_stack as stack_fixtures  # noqa: E402

MM_NODE_IDS = ["BOOT", "HOMING", "IDLE", "LEVELLING", "ACTIVE", "FAULT",
               "ACTIVE:STANDBY", "ACTIVE:TRAJECTORY", "ACTIVE:SPACEMOUSE",
               "ACTIVE:SHELL", "ACTIVE:GUI", "ACTIVE:CATCH"]

# SVG-safe DOM getters (className on SVG elements is SVGAnimatedString, so
# getAttribute('class'); node `disabled` is the hold-gating expando).
JS_MM_NODES = (
    "(() => { const out = {}; "
    "document.querySelectorAll('#minimap-svg [data-node]').forEach(g => { "
    "const t = g.querySelector('title'); "
    "out[g.getAttribute('data-node')] = { cls: g.getAttribute('class') || '', "
    "title: t ? t.textContent : '', dis: !!g.disabled }; }); return out; })()")
JS_MM_PANEL = js_probe(
    "{ root: { cls: document.getElementById('state-minimap') ? "
    "document.getElementById('state-minimap').getAttribute('class') || '' : null }, "
    " pane: { cls: document.getElementById('viewer-pane') ? "
    "document.getElementById('viewer-pane').getAttribute('class') || '' : null }, "
    " status: g('minimap-status'), action: g('minimap-action'), "
    " armed: g('minimap-armed-badge'), conn: g('conn-dot') }")


def _mm_rec_norm(records: dict):
    """Merge command + service records into one timestamp-ordered timeline."""
    ev = []
    for r in records.get("orchestrator_command", []):
        ev.append({"t": r["t"], "kind": "cmd", "name": r["data"]})
    for r in records.get("services", []):
        ev.append({"t": r["t"], "kind": "svc", "name": r["service"],
                   "data": r.get("data")})
    ev.sort(key=lambda e: e["t"])
    return ev


class MinimapRunner:
    """State + helpers for the reactive minimap scenario."""

    def __init__(self, stack, cdp, run_dir):
        self.stack = stack
        self.cdp = cdp
        self.run_dir = run_dir
        self.results = []
        self._snap_idx = 0

    def add(self, stage, name, ok, evidence, elapsed=0.0):
        self.results.append({"stage": stage, "assertion": name, "ok": ok,
                             "elapsed_s": round(elapsed, 2),
                             "evidence": evidence if isinstance(evidence, str)
                             else _ev(evidence)})
        log("    [%s] %s (%.1fs)" % ("PASS" if ok else "FAIL", name, elapsed))

    async def snapshot(self, name):
        html = await evaluate(self.cdp, "document.documentElement.outerHTML")
        self._snap_idx += 1
        if isinstance(html, str):
            path = os.path.join(self.run_dir,
                                "snapshot-mm%02d-%s.html" % (self._snap_idx, name))
            with open(path, "w") as fh:
                fh.write(html)

    async def poll(self, js, check, timeout):
        a = Assertion("_", js, _safe(check), timeout=timeout)
        return await poll_dom(self.cdp, a)

    def rec_after(self, t0):
        recs = self.stack.records().get("records", {})
        return [e for e in _mm_rec_norm(recs) if e["t"] > t0]

    async def wait_record(self, t0, pred, timeout):
        """Poll the stack's records until one after t0 matches pred."""
        deadline = time.monotonic() + timeout
        while True:
            for e in self.rec_after(t0):
                if pred(e):
                    return e
            if time.monotonic() >= deadline:
                return None
            await asyncio.sleep(0.2)


async def scenario_minimap(stack, cdp, gui_url, run_dir):
    R = MinimapRunner(stack, cdp, run_dir)
    fx = stack_fixtures

    # ---- M1: connect + healthy IDLE renders ------------------------------
    stack.stage("mm-idle-healthy")
    ok, ev, el = await R.poll(JS_CONN, check_connected, 30.0)
    R.add("mm-connect", "gui-connected", ok, ev, el)
    ok, ev, el = await R.poll(
        JS_MM_NODES,
        lambda v: ("current" in v["IDLE"]["cls"].split()
                   and len(v) == len(MM_NODE_IDS), _ev(
                       {"IDLE": v.get("IDLE"), "node_count": len(v)})),
        10.0)
    R.add("mm-connect", "minimap-renders-idle-current", ok, ev, el)

    # ---- M2: expansion toggle (plain clicks, no hold) ---------------------
    await click_element(cdp, "#minimap-header")
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: ("expanded" in v["root"]["cls"].split()
                   and "minimap-expanded" in v["pane"]["cls"].split(),
                   _ev({"root": v["root"], "pane": v["pane"]})),
        3.0)
    R.add("expansion-toggle", "header-click-expands-pane", ok, ev, el)
    await click_element(cdp, "#minimap-header")
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: ("collapsed" in v["root"]["cls"].split()
                   and "minimap-expanded" not in v["pane"]["cls"].split(),
                   _ev({"root": v["root"], "pane": v["pane"]})),
        3.0)
    R.add("expansion-toggle", "header-click-collapses-back", ok, ev, el)
    await R.snapshot("expansion")

    # ---- M5(a): greyed reasons — IDLE unhomed -----------------------------
    stack.stage("mm-idle-unhomed")
    ok, ev, el = await R.poll(
        JS_MM_NODES,
        lambda v: ("unreachable" in v["LEVELLING"]["cls"].split()
                   and re.search(r"hom", v["LEVELLING"]["title"], re.I) is not None
                   and v["LEVELLING"]["dis"] is True,
                   _ev({"LEVELLING": v["LEVELLING"]})),
        8.0)
    R.add("greyed-reasons", "levelling-unreachable-mentions-homing", ok, ev, el)
    await R.snapshot("greyed-unhomed")

    # ---- M5(b): uplink-lost greying — bridge_link=LOST while healthy IDLE.
    # The bridge keeps publishing FRESH link_status (frozen fault_state), so
    # only the G_BRIDGE guard catches this: all commandable nodes must grey
    # with the uplink-lost reason despite fresh messages flowing.
    stack.stage("mm-idle-healthy")
    ok, ev, el = await R.poll(
        JS_MM_NODES,
        lambda v: ("current" in v["IDLE"]["cls"].split()
                   and v["ACTIVE"]["dis"] is False,
                   _ev({"IDLE": v["IDLE"], "ACTIVE": v["ACTIVE"]})),
        8.0)
    R.add("greyed-reasons", "healthy-idle-baseline-before-uplink-loss", ok, ev, el)
    stack.set("link_status", {"values": dict(
        fx.LINK_MM_DISARMED, bridge_link="LOST", heartbeat_age_ms="4023")})
    lost_marker = "Teensy uplink lost (bridge_link=LOST"
    ok, ev, el = await R.poll(
        JS_MM_NODES,
        lambda v: (all("unreachable" in v[n]["cls"].split()
                       and lost_marker in v[n]["title"]
                       for n in ("HOMING", "LEVELLING", "ACTIVE",
                                 "ACTIVE:TRAJECTORY")),
                   _ev({"HOMING": v["HOMING"], "ACTIVE": v["ACTIVE"]})),
        8.0)
    R.add("greyed-reasons", "uplink-lost-greys-commandable-nodes", ok, ev, el)
    await R.snapshot("greyed-uplink-lost")

    # ---- M4: HOMING discards — hold EVERY node, expect zero records -------
    stack.stage("mm-homing")
    ok, ev, el = await R.poll(
        JS_MM_NODES,
        lambda v: ("discards" in v["IDLE"]["title"]
                   and all(v[n]["dis"] for n in MM_NODE_IDS),
                   _ev({"IDLE": v["IDLE"]})),
        8.0)
    R.add("homing-discard", "all-nodes-inert-in-homing", ok, ev, el)
    t0 = time.time()
    await click_element(cdp, "#minimap-header")   # expand: chips clickable
    await asyncio.sleep(0.3)
    for node in MM_NODE_IDS:
        await pointer_hold(cdp, '[data-node="%s"]' % node, 1000)
    await click_element(cdp, "#minimap-header")   # collapse back
    await asyncio.sleep(1.0)                      # any stray publish lands here
    recs = R.rec_after(t0)
    R.add("homing-discard", "zero-commands-zero-services-recorded",
          len(recs) == 0, {"records_in_window": recs})
    await R.snapshot("homing-discard")

    # ---- M3: Arm button gating -------------------------------------------
    stack.stage("mm-active-standby")   # control_mode NOT published
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: (v["action"]["disp"] != "none"
                   and v["action"]["txt"] == "Arm Setpoints"
                   and v["action"]["dis"] is True,
                   _ev({"action": v["action"]})),
        8.0)
    R.add("arm-gating", "arm-disabled-until-controlmode-echo", ok, ev, el)
    stack.set("control_mode_topic", {"enabled": True, "data": "STANDBY"})
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: (v["action"]["txt"] == "Arm Setpoints"
                   and v["action"]["dis"] is False,
                   _ev({"action": v["action"]})),
        5.0)
    R.add("arm-gating", "arm-enabled-after-controlmode-echo", ok, ev, el)
    t0 = time.time()
    await pointer_hold(cdp, "#minimap-action", 1000)
    e = await R.wait_record(
        t0, lambda e: e["kind"] == "svc" and e["name"] == "set_setpoint_output"
        and e.get("data") is True, 6.0)
    R.add("arm-gating", "hold-records-set-setpoint-output-true",
          e is not None, {"record": e})
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: ("OK" in v["status"]["txt"] and "Arm setpoints" in v["status"]["txt"],
                   _ev({"status": v["status"]["txt"]})),
        6.0)
    R.add("arm-gating", "status-shows-arm-success", ok, ev, el)
    await R.snapshot("arm-gating")

    # ---- M3(b): worst-case armed staleness — mpc_active=1 then link_status
    # STOPS entirely.  A stale link must never render as disarmed: the badge
    # switches to the amber 'ARMED?' variant and the action stays an ENABLED
    # Disarm (the escape hatch), never degrading to a disabled Arm.
    stack.stage("mm-active-traj-armed")
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: (v["armed"]["disp"] != "none" and v["armed"]["txt"] == "ARMED"
                   and "stale" not in v["armed"]["cls"].split(),
                   _ev({"armed": v["armed"]})),
        8.0)
    R.add("armed-stale", "armed-badge-fresh-baseline", ok, ev, el)
    stack.set("link_status", {"enabled": False})   # bridge node "dies"
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: (v["armed"]["disp"] != "none" and v["armed"]["txt"] == "ARMED?"
                   and "stale" in v["armed"]["cls"].split(),
                   _ev({"armed": v["armed"]})),
        8.0)   # LINK_STALE_MS = 2 s
    R.add("armed-stale", "stale-flips-badge-to-armed-question", ok, ev, el)
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: (v["action"]["disp"] != "none" and v["action"]["txt"] == "Disarm"
                   and v["action"]["dis"] is False,
                   _ev({"action": v["action"]})),
        5.0)
    R.add("armed-stale", "worst-case-keeps-enabled-disarm", ok, ev, el)
    await R.snapshot("armed-stale")

    # ---- M6: TEARDOWN — NEGATIVE variant first (mpc_active never flips) ---
    # The verify-gate must ABORT and 'deactivate' must NEVER be published.
    stack.stage("mm-active-traj-armed")
    ok, ev, el = await R.poll(
        JS_MM_NODES,
        lambda v: ("current" in v["ACTIVE:TRAJECTORY"]["cls"].split()
                   and v["IDLE"]["dis"] is False,
                   _ev({"traj": v["ACTIVE:TRAJECTORY"], "IDLE": v["IDLE"]})),
        8.0)
    R.add("teardown-negative", "armed-trajectory-renders-idle-holdable", ok, ev, el)
    t0 = time.time()
    await pointer_hold(cdp, '[data-node="IDLE"]', 1000)
    e = await R.wait_record(
        t0, lambda e: e["kind"] == "cmd" and e["name"] == "standby", 8.0)
    R.add("teardown-negative", "standby-published-first", e is not None, {"record": e})
    # React: the fake orchestrator moves to ACTIVE:STANDBY.
    stack.set("orchestrator_state", {"data": "ACTIVE:STANDBY"})
    stack.set("control_mode_topic", {"enabled": True, "data": "STANDBY"})
    e = await R.wait_record(
        t0, lambda e: e["kind"] == "svc" and e["name"] == "set_setpoint_output"
        and e.get("data") is False, 15.0)
    R.add("teardown-negative", "disarm-requested", e is not None, {"record": e})
    # Deliberately DO NOT flip mpc_active: the 3 s verify window must abort.
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: ("ABORTED" in v["status"]["txt"]
                   and "mpc_active" in v["status"]["txt"],
                   _ev({"status": v["status"]["txt"]})),
        8.0)
    R.add("teardown-negative", "abort-within-verify-window-names-gate", ok, ev, el)
    neg_timeline = R.rec_after(t0)
    R.add("teardown-negative", "deactivate-NEVER-published-while-armed",
          all(not (e["kind"] == "cmd" and e["name"] == "deactivate")
              for e in neg_timeline),
          {"timeline": neg_timeline})
    await R.snapshot("teardown-negative")

    # ---- M7: TEARDOWN — POSITIVE variant (keystone safety order) ----------
    stack.stage("mm-active-traj-armed")
    ok, ev, el = await R.poll(
        JS_MM_NODES,
        lambda v: ("current" in v["ACTIVE:TRAJECTORY"]["cls"].split()
                   and v["IDLE"]["dis"] is False,
                   _ev({"traj": v["ACTIVE:TRAJECTORY"]})),
        8.0)
    R.add("teardown-order", "reset-to-armed-trajectory", ok, ev, el)
    t0 = time.time()
    await pointer_hold(cdp, '[data-node="IDLE"]', 1000)
    e_standby = await R.wait_record(
        t0, lambda e: e["kind"] == "cmd" and e["name"] == "standby", 8.0)
    stack.set("orchestrator_state", {"data": "ACTIVE:STANDBY"})
    stack.set("control_mode_topic", {"enabled": True, "data": "STANDBY"})
    e_gohome = await R.wait_record(
        t0, lambda e: e["kind"] == "svc" and e["name"] == "trajectory/go_home", 10.0)
    e_disarm = await R.wait_record(
        t0, lambda e: e["kind"] == "svc" and e["name"] == "set_setpoint_output"
        and e.get("data") is False, 15.0)
    # React: only NOW does the fake bridge report mpc_active=0.
    t_flip = time.time()
    stack.set("link_status", {"values": dict(fx.LINK_MM_DISARMED)})
    e_deact = await R.wait_record(
        t0, lambda e: e["kind"] == "cmd" and e["name"] == "deactivate", 8.0)
    stack.set("orchestrator_state", {"data": "IDLE"})
    stack.set("control_mode_topic", {"data": ""})
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: ("OK" in v["status"]["txt"]
                   and "safe teardown" in v["status"]["txt"],
                   _ev({"status": v["status"]["txt"]})),
        8.0)
    R.add("teardown-order", "completion-status-shown", ok, ev, el)
    pos_timeline = R.rec_after(t0)
    expected = [("cmd", "standby"), ("svc", "trajectory/go_home"),
                ("svc", "set_setpoint_output"), ("cmd", "deactivate")]
    got = [(e["kind"], e["name"]) for e in pos_timeline]
    # Keystone evidence: NEVER truncate the safety-order timelines (they are
    # the report's core artefact) — pass pre-serialized strings, which
    # MinimapRunner.add stores verbatim.
    R.add("teardown-order", "exact-safe-order-standby-gohome-disarm-deactivate",
          got == expected,
          json.dumps({"expected": expected, "timeline": pos_timeline}))
    R.add("teardown-order", "deactivate-only-after-mpc0-flip",
          e_deact is not None and e_deact["t"] > t_flip,
          {"deactivate_t": e_deact and e_deact["t"], "mpc0_flip_t": t_flip,
           "delta_s": e_deact and round(e_deact["t"] - t_flip, 3)})
    await R.snapshot("teardown-positive")
    # Stash both full timelines for the report (untruncated).
    R.results.append({"stage": "teardown-order", "assertion": "TIMELINES (info)",
                      "ok": True, "elapsed_s": 0.0,
                      "evidence": json.dumps({"negative": neg_timeline,
                                              "positive": pos_timeline,
                                              "mpc0_flip_t": t_flip})})

    # ---- M8: mid-sequence FAULT abort -------------------------------------
    stack.stage("mm-idle-healthy")
    ok, ev, el = await R.poll(
        JS_MM_NODES,
        lambda v: ("current" in v["IDLE"]["cls"].split()
                   and v["ACTIVE"]["dis"] is False,
                   _ev({"IDLE": v["IDLE"], "ACTIVE": v["ACTIVE"]})),
        8.0)
    R.add("fault-abort", "healthy-idle-active-holdable", ok, ev, el)
    t0 = time.time()
    await pointer_hold(cdp, '[data-node="ACTIVE"]', 1000)
    e_act = await R.wait_record(
        t0, lambda e: e["kind"] == "cmd" and e["name"] == "activate", 8.0)
    R.add("fault-abort", "activate-published", e_act is not None, {"record": e_act})
    # React: FAULT mid-sequence, with a visible error string.  Deterministic
    # two-beat: publish the error FIRST and wait until the GUI has provably
    # ingested it (the HOMING node greys with the G_ERRORS reason) BEFORE
    # flipping orchestrator_state to FAULT — otherwise FAULT can win the
    # cross-topic race and the abort message honestly lacks the error string.
    stack.set("robot_state", {"error": ["injected test fault"]})
    ok, ev, el = await R.poll(
        JS_MM_NODES,
        lambda v: ("active errors: injected test fault" in v["HOMING"]["title"],
                   _ev({"HOMING": v["HOMING"]})),
        6.0)
    R.add("fault-abort", "error-ingested-before-fault-flip", ok, ev, el)
    stack.set("orchestrator_state", {"data": "FAULT"})
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: ("ABORTED" in v["status"]["txt"]
                   and "injected test fault" in v["status"]["txt"],
                   _ev({"status": v["status"]["txt"]})),
        8.0)
    R.add("fault-abort", "abort-names-the-fault", ok, ev, el)
    await asyncio.sleep(2.0)   # any late step would land in this window
    recs = R.rec_after(t0)
    R.add("fault-abort", "no-commands-after-fault",
          [(e["kind"], e["name"]) for e in recs] == [("cmd", "activate")],
          {"timeline": recs})
    await R.snapshot("fault-abort")

    # ---- M9: disconnect (LAST — kills the GUI's websocket) ----------------
    stack.stop_rosbridge()
    def _disc_check(v):
        non_auto = [n for n in MM_NODE_IDS if n not in ("BOOT", "FAULT")]
        ok = (all("unreachable" in v[n]["cls"].split() for n in MM_NODE_IDS)
              and all(v[n]["title"] == "rosbridge disconnected" for n in non_auto))
        return ok, _ev({"IDLE": v["IDLE"], "ACTIVE:GUI": v["ACTIVE:GUI"],
                        "BOOT": v["BOOT"]})
    ok, ev, el = await R.poll(JS_MM_NODES, _disc_check, 15.0)
    R.add("disconnect", "all-nodes-greyed-with-disconnected-reason", ok, ev, el)
    ok, ev, el = await R.poll(
        JS_MM_PANEL,
        lambda v: ("connected" not in v["conn"]["cls"].split(),
                   _ev({"conn": v["conn"]})),
        10.0)
    R.add("disconnect", "connection-dot-disconnected", ok, ev, el)
    await R.snapshot("disconnected")

    return R.results


SCENARIOS = {"scenario1": SCENARIO1, "minimap": scenario_minimap}


# ---------------------------------------------------------------------------
# Pre-checks (probe-side; the stack re-checks the ROS-side rails itself)
# ---------------------------------------------------------------------------

def prechecks(gui_url: str, skip_cdn: bool) -> None:
    if port_bound(9090):
        raise RuntimeError(":9090 already bound — a real rosbridge (real stack?) "
                           "is running; refusing to start")
    # (No pgrep-by-name check: it false-positives on any command line that
    # merely mentions the name. A rosbridge on a non-default port is caught
    # by the synthetic stack's ROS-graph-empty rail instead.)
    try:
        with urllib.request.urlopen(gui_url, timeout=5) as r:
            if r.status != 200:
                raise RuntimeError("GUI server %s returned %s" % (gui_url, r.status))
    except (urllib.error.URLError, OSError) as exc:
        raise RuntimeError("GUI server %s unreachable (%r) — is "
                           "jugglebot-gui.service up?" % (gui_url, exc))
    if not skip_cdn:
        try:
            req = urllib.request.Request(THREE_CDN_URL, method="HEAD")
            with urllib.request.urlopen(req, timeout=10) as r:
                if r.status != 200:
                    raise RuntimeError("CDN HEAD returned %s" % r.status)
        except (urllib.error.URLError, OSError) as exc:
            raise RuntimeError(
                "three.js CDN unreachable (%r) — index.html's importmap loads "
                "three.js from cdn.jsdelivr.net, so the GUI cannot boot "
                "offline. Fix connectivity or vendor three.js locally. "
                "(--skip-cdn-check to override.)" % exc)


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

async def run_scenario(scenario_name, stages, stack, cdp, gui_url, run_dir):
    results = []  # dicts: stage, assertion, ok, evidence, elapsed

    load_fut = cdp.event_future("Page.loadEventFired")
    await cdp.call("Page.enable")
    await cdp.call("Runtime.enable")
    await cdp.call("Log.enable")
    await cdp.call("Page.navigate", {"url": gui_url})
    await asyncio.wait_for(load_fut, timeout=60.0)
    log("GUI page loaded: %s" % gui_url)

    # Custom (reactive/interactive) scenarios are async callables; declarative
    # scenarios are StageSpec lists.
    if callable(stages):
        return await stages(stack, cdp, gui_url, run_dir)

    for idx, stage in enumerate(stages):
        log("--- stage %d/%d: %s" % (idx + 1, len(stages), stage.name))
        if stage.stack_stage:
            stack.stage(stage.stack_stage)
        for a in stage.assertions:
            if a.kind == "page-errors":
                ok, evidence = a.check(cdp)
                elapsed = 0.0
            else:
                ok, evidence, elapsed = await poll_dom(cdp, a)
            results.append({"stage": stage.name, "assertion": a.name,
                            "ok": ok, "elapsed_s": round(elapsed, 2),
                            "evidence": evidence})
            log("    [%s] %s (%.1fs)" % ("PASS" if ok else "FAIL", a.name, elapsed))
        html = await evaluate(cdp, "document.documentElement.outerHTML")
        snap = os.path.join(run_dir, "snapshot-%02d-%s.html" % (idx + 1, stage.name))
        if isinstance(html, str):
            with open(snap, "w") as fh:
                fh.write(html)
    return results


def write_report(run_dir, scenario_name, gui_url, results, cdp, teardown_info,
                 stack_records):
    console_dump = {
        "page_exceptions": cdp.page_exceptions if cdp else [],
        "console_errors": cdp.console_errors if cdp else [],
        "console_all": cdp.console_all if cdp else [],
        "log_entries": cdp.log_entries if cdp else [],
    }
    with open(os.path.join(run_dir, "console_log.json"), "w") as fh:
        json.dump(console_dump, fh, indent=2, default=repr)

    n_pass = sum(1 for r in results if r["ok"])
    overall = "PASS" if results and n_pass == len(results) else "FAIL"
    lines = []
    lines.append("GUI DOM probe report — %s" % scenario_name)
    lines.append("run: %s   gui: %s   time: %s"
                 % (run_dir, gui_url, datetime.now().isoformat(timespec="seconds")))
    lines.append("")
    cur_stage = None
    for r in results:
        if r["stage"] != cur_stage:
            cur_stage = r["stage"]
            lines.append("STAGE %s" % cur_stage)
        lines.append("  [%s] %s (%.1fs)" % ("PASS" if r["ok"] else "FAIL",
                                            r["assertion"], r["elapsed_s"]))
        lines.append("         evidence: %s" % r["evidence"])
    lines.append("")
    if cdp:
        lines.append("page errors: %d uncaught exceptions, %d console.error, "
                     "%d warn/error log entries (console_log.json)"
                     % (len(cdp.page_exceptions), len(cdp.console_errors),
                        len(cdp.log_entries)))
    lines.append("stack records: %s" % json.dumps(stack_records, default=repr))
    lines.append("teardown: %s" % json.dumps(teardown_info))
    lines.append("")
    lines.append("OVERALL: %s (%d/%d assertions)" % (overall, n_pass, len(results)))
    report = "\n".join(lines) + "\n"
    with open(os.path.join(run_dir, "report.txt"), "w") as fh:
        fh.write(report)
    with open(os.path.join(run_dir, "report.json"), "w") as fh:
        json.dump({"scenario": scenario_name, "overall": overall,
                   "results": results, "teardown": teardown_info,
                   "stack_records": stack_records}, fh, indent=2, default=repr)
    print(report)
    return overall == "PASS"


def verify_teardown(stack, chromium) -> dict:
    """PID-liveness checks against the ACTUAL processes this run spawned
    (pattern-based ``pgrep -f`` is fragile: any unrelated process whose
    command line merely mentions the script name — e.g. an operator's
    grep/editor — would false-flag)."""
    deadline = time.monotonic() + 12.0
    while port_bound(9090) and time.monotonic() < deadline:
        time.sleep(0.5)

    def pid_dead(pid):
        if pid is None:
            return True  # never started -> nothing to leak
        try:
            os.kill(int(pid), 0)
            return False
        except OSError:
            return True

    ready = stack.ready_info or {}
    return {
        "port_9090_free": not port_bound(9090),
        "rosbridge_dead": pid_dead(ready.get("rosbridge_pid")),
        "rosapi_dead": pid_dead(ready.get("rosapi_pid")),
        "stack_dead": stack.proc is None or stack.proc.poll() is not None,
        "chromium_dead": chromium.proc is None or chromium.proc.poll() is not None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Headless-chromium DOM verification of the Jugglebot GUI "
                    "against a synthetic ROS2 stack (no hardware)")
    parser.add_argument("--scenario", default="scenario1",
                        choices=sorted(SCENARIOS))
    parser.add_argument("--gui-url", default=DEFAULT_GUI_URL)
    parser.add_argument("--chromium", default=DEFAULT_CHROMIUM)
    parser.add_argument("--out-root", default=DEFAULT_OUT_ROOT)
    parser.add_argument("--skip-cdn-check", action="store_true")
    args = parser.parse_args()

    stages = SCENARIOS[args.scenario]

    run_dir = os.path.join(args.out_root,
                           datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    log("run dir: %s" % run_dir)

    try:
        prechecks(args.gui_url, args.skip_cdn_check)
    except RuntimeError as exc:
        log("PRECHECK FAILED: %s" % exc)
        return 2

    stack = SyntheticStack(run_dir)
    chromium = Chromium(args.chromium, run_dir)
    cdp = CDPClient()
    results = []
    stack_records = {}
    rc = 2
    loop = asyncio.get_event_loop()
    try:
        stack.start()
        chromium.start()
        ws_url = chromium.page_ws_url()
        loop.run_until_complete(cdp.connect(ws_url))
        results = loop.run_until_complete(
            run_scenario(args.scenario, stages, stack, cdp, args.gui_url, run_dir))
        try:
            stack_records = stack.records().get("records", {})
        except Exception as exc:
            stack_records = {"error": repr(exc)}
        rc = 0
    except Exception as exc:
        log("RUN FAILED: %r" % exc)
        rc = 2
    finally:
        try:
            cdp.close()
        except Exception:
            pass
        stack.quit_and_wait()
        chromium.kill()
        teardown_info = verify_teardown(stack, chromium)
        log("teardown: %s" % teardown_info)
        all_pass = write_report(run_dir, args.scenario, args.gui_url, results,
                                cdp, teardown_info, stack_records)
        if rc == 0:
            rc = 0 if (all_pass and all(teardown_info.values())) else 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
