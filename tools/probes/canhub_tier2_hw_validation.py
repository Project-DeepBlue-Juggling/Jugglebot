#!/usr/bin/env python3
"""canhub_tier2_hw_validation.py — interactive operator runner for the Fable-5
Tier-2 can-bridge hardware-validation checklist.

Companion to ``logbook/2026-07-02-canhub-hardening-tier2.md`` ("Hardware
validation checklist"). It walks an operator through the 7 powered checks one at
a time: it prints WHAT the test validates and WHAT PASS looks like, waits for the
operator to press Enter, tells the operator exactly when to perform any external
action (induce a fault, restart run_mpc, unplug CAN3, ...), watches the Teensy's
live serial diagnostics to help judge the result, and records a PASS/FAIL/SKIP
verdict + a serial snapshot per check into a timestamped Markdown report under
``temp/logs/``.

WHY SERIAL (not the UDP link): the checklist's streaming checks (2/3/4) assume
run_mpc drives setpoints through the production ROS2 stack, and the
``teensy_bridge_node`` OWNS the Jetson<->Teensy UDP link (binds ports 5005/5006).
A second UDP peer cannot coexist. The Teensy's USB-serial debug console
(``/dev/ttyACM0`` @ 115200) emits a 1 Hz block — ``[guard]`` / ``[diag]`` /
``[axes]`` / ``[canhealth]`` / ``[bb]`` — that is READ-ONLY, always-on, and
available in EVERY configuration (bridge up or down, run_mpc streaming or not).
It exposes guard_mode, the interp output-gate, fault_state, drain_cap, and
per-axis ODrive state directly — nearly every checklist observable.

WHAT THIS SCRIPT DOES NOT DO: it never commands robot motion and never owns the
UDP link. Per project convention the OPERATOR runs the robot-actuating commands
(homing / activate / run_mpc / clear_errors / fault induction); this script preps
the exact steps, observes read-only, and records. The single active thing it can
do is fire a UDP junk-flood at the Teensy's ports for check 3 — sendto() only, no
port binding, so it does not contend with the live bridge — and only after an
explicit operator confirm.

The observable behaviour CHANGES this firmware introduces (INTENDED, not faults):
  (a) the guard E-STOP now LATCHES until CLEAR_ERRORS (no auto-recovery);
  (b) a HAND ODrive fault now E-STOPs the legs (guard ESTOP, output gated);
  (c) a dead Jetson<->Teensy link reads fatal on robot_state;
  (d) a cross-axis disarm reads fatal.

Firmware under test: canhub-hardening Tier-2 (plan items 13-17 + 18B + review
fixes) — commits 0ad3d25 138aa11 b562825 c8ba247 83ac938 6fe1ec9 192e6af,
flashed 2026-07-03.

Usage:
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/canhub_tier2_hw_validation.py            # all 7 checks
    python tools/probes/canhub_tier2_hw_validation.py --check 1  # one check
    python tools/probes/canhub_tier2_hw_validation.py --check 1,6
    python tools/probes/canhub_tier2_hw_validation.py --list
    python tools/probes/canhub_tier2_hw_validation.py --port /dev/ttyACM1
    python tools/probes/canhub_tier2_hw_validation.py --no-serial # narrate only

Read-only. Safe to run during a powered sitting alongside the live ROS2 stack.
"""

from __future__ import annotations

import argparse
import os
import re
import socket
import sys
import threading
import time
from datetime import datetime

# ── Enum decode tables (mirror config/generated/udp_protocol.py) ──────────────
FAULT_STATE = {
    0: "NONE", 1: "MPC_STALE", 2: "LINK_LOST", 3: "MOTOR_OVERSPEED",
    4: "MAX_DEVIATION", 5: "ODRIVE_FATAL", 6: "CAN_BUS_DOWN", 7: "MOTOR_FB_STALE",
}
GUARD_MODE = {0: "DISABLED", 1: "ENABLED", 2: "ESTOP"}
LINK_STATE = {0: "INIT", 1: "UP", 2: "DEGRADED", 3: "LOST"}
AXIS_STATE = {1: "IDLE", 8: "CLOSED_LOOP"}   # the two we care about; others print raw

# Teensy point-to-point link (config/generated/udp_protocol.py)
TEENSY_IP = "192.168.42.2"
PORT_STREAM = 5005
PORT_RPC = 5006

# ── serial line grammar (Teensy_code_canbridge.ino task_diag, 1 Hz) ───────────
RE_DIAG = re.compile(
    r"\[diag\]\s+link=(\d+)\s+fault=(\d+)\s+rx=(\d+)\s+tx=(\d+)\s+"
    r"crc_err=(\d+)\s+seq_gaps=(\d+)\s+drain_cap=(\d+)\s+synced=(-?\d+)\s+heap=(\d+)")
RE_GUARD = re.compile(
    r"\[guard\]\s+mpc_active=(\d+)\s+guard_mode=(\d+)\s+output=(\d+)\s+"
    r"sp_age_ms=(\d+)\s+u0=(-?[\d.]+)")
RE_AXES_HEAD = re.compile(r"\[axes\]\s+fresh=(\d+)/(\d+)(.*)")
# Mark is ONLY a status symbol (*=error/disarm, !=stale, ?=never-seen) or absent
# (' '=ok). Matching a bare `.` here backtracks on the last axis — whose trailing
# ' ' mark is stripped before the newline — and steals a digit off the age.
RE_AXIS = re.compile(r"([0-9H]):s(\d+)/(\d+)([*!?]?)")
RE_CANHEALTH = re.compile(
    r"\[canhealth\]\s+(\S+)\s+sync=(\d+)\s+hwm=(\d+)\s+capHit=(\d+)\s+err=(\d+)\s+"
    r"flags=0x([0-9a-fA-F]+)\s+rec=(\d+)\s+tec=(\d+)\s+flt=(\w+)")


# ── ANSI (auto-disabled off a TTY or with --no-color) ─────────────────────────
class C:
    ok = "\033[92m"
    bad = "\033[91m"
    warn = "\033[93m"
    hd = "\033[96m"
    dim = "\033[90m"
    bold = "\033[1m"
    rst = "\033[0m"

    @classmethod
    def off(cls):
        for k in ("ok", "bad", "warn", "hd", "dim", "bold", "rst"):
            setattr(cls, k, "")


class SerialMonitor:
    """Background reader of the Teensy 1 Hz serial debug block.

    Parses [diag]/[guard]/[axes]/[canhealth] lines into ``self.state`` under a
    lock and counts diagnostic blocks (``self.frames``) so callers can wait for
    fresh data after an action. Read-only: never writes to the serial port.
    """

    def __init__(self, port: str, baud: int = 115200):
        import serial  # pyserial; imported here so --no-serial needs no dep
        self._ser = serial.Serial(port, baud, timeout=1.0)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.state: dict = {}
        self.frames = 0            # number of [diag] blocks seen (block counter)
        self.last_line_ts = 0.0
        self._raw_tail: list = []  # last N raw lines, for the report
        self._t = threading.Thread(target=self._loop, name="serial-mon", daemon=True)
        self._t.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
            except Exception:
                time.sleep(0.2)
                continue
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", "replace").strip()
            except Exception:
                continue
            if not line:
                continue
            self._parse(line)

    def _parse(self, line: str):
        with self._lock:
            self.last_line_ts = time.monotonic()
            self._raw_tail.append(line)
            if len(self._raw_tail) > 40:
                self._raw_tail.pop(0)
            m = RE_DIAG.search(line)
            if m:
                self.frames += 1
                (link, fault, rx, tx, crc, gaps, drain, synced, heap) = m.groups()
                self.state.update(
                    link=int(link), fault=int(fault), rx=int(rx), tx=int(tx),
                    crc_err=int(crc), seq_gaps=int(gaps), drain_cap=int(drain),
                    synced=int(synced), heap=int(heap))
                return
            m = RE_GUARD.search(line)
            if m:
                mpc, gm, out, age, u0 = m.groups()
                self.state.update(
                    mpc_active=int(mpc), guard_mode=int(gm), output=int(out),
                    sp_age_ms=int(age), u0=float(u0))
                return
            m = RE_AXES_HEAD.search(line)
            if m:
                fresh, total, tail = m.groups()
                axes = [(t, int(s), int(a), (mk if mk else " "))
                        for (t, s, a, mk) in RE_AXIS.findall(tail)]
                self.state.update(axes_fresh=int(fresh), axes_total=int(total),
                                  axes=axes)
                return
            m = RE_CANHEALTH.search(line)
            if m:
                name = m.group(1)
                buses = self.state.setdefault("buses", {})
                buses[name] = dict(sync=int(m.group(2)), hwm=int(m.group(3)),
                                   cap_hits=int(m.group(4)), err=int(m.group(5)),
                                   flags=int(m.group(6), 16), rec=int(m.group(7)),
                                   tec=int(m.group(8)), flt=m.group(9))
                return

    # ── accessors ────────────────────────────────────────────────────────────
    def snapshot(self) -> dict:
        with self._lock:
            import copy
            return copy.deepcopy(self.state)

    def frame_count(self) -> int:
        with self._lock:
            return self.frames

    def raw_tail(self, n: int = 12) -> list:
        with self._lock:
            return list(self._raw_tail[-n:])

    def stale(self) -> bool:
        with self._lock:
            return (time.monotonic() - self.last_line_ts) > 3.0 if self.last_line_ts else True

    def wait_frames(self, n: int = 2, timeout: float = 6.0) -> bool:
        """Block until ``n`` fresh [diag] blocks arrive (or timeout)."""
        start = self.frame_count()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.frame_count() - start >= n:
                return True
            time.sleep(0.1)
        return False

    def close(self):
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass


def fmt_guard(s: dict) -> str:
    if "guard_mode" not in s:
        return f"{C.dim}[guard] (no data yet){C.rst}"
    gm = s["guard_mode"]
    gmn = GUARD_MODE.get(gm, str(gm))
    col = C.bad if gm == 2 else (C.ok if gm == 1 else C.warn)
    out = s.get("output", "?")
    outc = C.ok if out == 1 else C.dim
    return (f"mpc_active={s.get('mpc_active','?')}  "
            f"guard_mode={col}{gm}:{gmn}{C.rst}  "
            f"output={outc}{out}{C.rst}  "
            f"sp_age_ms={s.get('sp_age_ms','?')}  u0={s.get('u0','?')}")


def fmt_diag(s: dict) -> str:
    if "fault" not in s:
        return f"{C.dim}[diag] (no data yet){C.rst}"
    f = s["fault"]
    fn = FAULT_STATE.get(f, str(f))
    fc = C.ok if f == 0 else C.bad
    ln = LINK_STATE.get(s.get("link"), s.get("link"))
    return (f"link={ln}  fault={fc}{f}:{fn}{C.rst}  "
            f"drain_cap={s.get('drain_cap','?')}  seq_gaps={s.get('seq_gaps','?')}  "
            f"crc_err={s.get('crc_err','?')}  synced={s.get('synced','?')}")


def fmt_axes(s: dict) -> str:
    if "axes" not in s:
        return f"{C.dim}[axes] (no data yet){C.rst}"
    parts = []
    for (tag, st, age, mk) in s["axes"]:
        stn = AXIS_STATE.get(st, f"s{st}")
        markname = {"*": f"{C.bad}*ERR{C.rst}", "!": f"{C.warn}!STALE{C.rst}",
                    "?": f"{C.dim}?GONE{C.rst}", " ": ""}.get(mk, "")
        parts.append(f"{tag}={stn}/{age}ms{markname}")
    return f"fresh={s.get('axes_fresh','?')}/{s.get('axes_total','?')}  " + " ".join(parts)


class Runner:
    def __init__(self, mon, report_path: str, no_serial: bool):
        self.mon = mon
        self.no_serial = no_serial
        self.report_path = report_path
        self.results: list = []

    # ── UI primitives ─────────────────────────────────────────────────────────
    def hr(self, ch="─"):
        print(f"{C.dim}{ch * 78}{C.rst}")

    def head(self, text):
        self.hr("═")
        print(f"{C.hd}{C.bold}{text}{C.rst}")
        self.hr("═")

    def expect(self, lines):
        print(f"{C.bold}Expected — PASS if:{C.rst}")
        for ln in lines:
            print(f"  {C.ok}✓{C.rst} {ln}")

    def note(self, text):
        print(f"{C.warn}▸ {text}{C.rst}")

    def show_live(self, label="live"):
        if self.no_serial:
            return
        if self.mon.stale():
            print(f"  {C.warn}(serial: no fresh diagnostics — is /dev/ttyACM0 connected "
                  f"and no other monitor open?){C.rst}")
            return
        s = self.mon.snapshot()
        print(f"  {C.dim}{label}:{C.rst} {fmt_guard(s)}")
        print(f"       {fmt_diag(s)}")
        print(f"       {fmt_axes(s)}")

    def enter(self, prompt="Press Enter to continue"):
        try:
            input(f"{C.bold}↳ {prompt}...{C.rst} ")
        except (EOFError, KeyboardInterrupt):
            print()

    def operator_action(self, lines):
        print(f"{C.warn}{C.bold}>>> OPERATOR ACTION <<<{C.rst}")
        for ln in lines:
            print(f"    {ln}")

    def wait_until(self, cond, desc, timeout=180.0):
        """Poll the serial snapshot until cond(snapshot) is True.

        Prints a live status line each ~1.5 s. Returns True when met, or False on
        timeout / Ctrl-C (operator may then force-proceed). In --no-serial mode it
        just prompts the operator to confirm manually.
        """
        if self.no_serial:
            self.enter(f"When {desc}, press Enter")
            return True
        print(f"  {C.dim}waiting until {desc}  (Ctrl-C to proceed anyway){C.rst}")
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                s = self.mon.snapshot()
                if s and cond(s):
                    print(f"  {C.ok}✓ {desc}{C.rst}")
                    self.show_live()
                    return True
                if not self.mon.stale():
                    print(f"  {C.dim}· {fmt_guard(s)}{C.rst}")
                time.sleep(1.5)
        except KeyboardInterrupt:
            print(f"\n  {C.warn}(proceeding without the condition){C.rst}")
            return False
        print(f"  {C.warn}(timed out after {int(timeout)} s){C.rst}")
        return False

    def snapshot_after(self, label, n_frames=2):
        """Grab a fresh snapshot after an action (waits for new serial blocks)."""
        if self.no_serial:
            return {}
        self.mon.wait_frames(n_frames)
        s = self.mon.snapshot()
        print(f"  {C.dim}{label}:{C.rst} {fmt_guard(s)}")
        print(f"       {fmt_diag(s)}")
        print(f"       {fmt_axes(s)}")
        return s

    def verdict(self, check_id, title, suggestion=None):
        if suggestion:
            print(f"{C.bold}Auto-assessment:{C.rst} {suggestion}")
        while True:
            try:
                ans = input(f"{C.bold}Verdict for check {check_id} "
                            f"[p]ass / [f]ail / [s]kip:{C.rst} ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                ans = "s"
                print()
            if ans in ("p", "pass"):
                v = "PASS"
            elif ans in ("f", "fail"):
                v = "FAIL"
            elif ans in ("s", "skip", ""):
                v = "SKIP"
            else:
                print("  (enter p, f, or s)")
                continue
            break
        try:
            note = input("  optional note (Enter to skip): ").strip()
        except (EOFError, KeyboardInterrupt):
            note = ""
            print()
        snap = {} if self.no_serial else self.mon.snapshot()
        tail = [] if self.no_serial else self.mon.raw_tail(10)
        self.results.append(dict(id=check_id, title=title, verdict=v, note=note,
                                 snapshot=snap, serial_tail=tail,
                                 ts=datetime.now().strftime("%H:%M:%S")))
        col = {"PASS": C.ok, "FAIL": C.bad, "SKIP": C.warn}[v]
        print(f"  recorded: {col}{v}{C.rst}")
        self._write_report()   # incremental — survive a mid-sitting abort
        return v

    # ── the 7 checks ──────────────────────────────────────────────────────────
    def check1(self):
        self.head("CHECK 1 — Guard E-STOP LATCH  [plan item 13]")
        print("Validates: once tripped, the guard E-STOP now STAYS latched (guard_mode=ESTOP,")
        print("output gated off) until an explicit CLEAR_ERRORS — it no longer auto-recovers")
        print("when the triggering transient clears. This mirrors motor_guard semantics.")
        print()
        self.expect([
            "guard_mode goes 2:ESTOP and output goes 0 when the E-STOP fires",
            "guard_mode STAYS 2:ESTOP + output STAYS 0 AFTER the condition clears (the crux)",
            "CLEAR_ERRORS returns guard_mode to 1:ENABLED, output re-enables, fault=0:NONE",
            "clearing THROUGH a still-live condition re-latches (guard_mode back to 2)",
        ])
        self.enter("Press Enter to begin check 1")

        self.operator_action([
            "Arm the robot and start the MPC setpoint stream so the Teensy is ENABLED with",
            "fresh setpoints, e.g. (your normal arming procedure):",
            f"  {C.hd}python run_mpc.py --pose 0,0,170,0,0,0 --duration 180{C.rst}",
            "I'll wait until [guard] shows mpc_active=1 and guard_mode=1:ENABLED.",
        ])
        self.wait_until(lambda s: s.get("mpc_active") == 1 and s.get("guard_mode") == 1,
                        "mpc_active=1 and guard_mode=1:ENABLED")

        print()
        self.operator_action([
            "INDUCE a guard E-STOP. Either:",
            "  - stop the setpoint stream for > 0.25 s (Ctrl-C run_mpc) → MPC_STALE, or",
            "  - briefly exceed the overspeed limit → MOTOR_OVERSPEED.",
        ])
        self.enter("Press Enter once you've induced it")
        s = self.snapshot_after("after inducing")
        tripped = s.get("guard_mode") == 2 and s.get("output") == 0

        print()
        self.operator_action([
            "Now let the triggering condition CLEAR (restart the stream / bring speed down),",
            "but do NOT clear errors yet. The E-STOP must remain latched.",
        ])
        self.enter("Press Enter once the condition has cleared")
        s2 = self.snapshot_after("after condition cleared (should STILL be ESTOP)")
        latched = s2.get("guard_mode") == 2 and s2.get("output") == 0

        print()
        self.operator_action([
            "Issue CLEAR_ERRORS (release the latch):",
            f"  {C.hd}ros2 service call /clear_errors std_srvs/srv/Trigger{C.rst}",
            "(adjust the namespace to your launch if needed)",
        ])
        self.enter("Press Enter once CLEAR_ERRORS has been issued")
        s3 = self.snapshot_after("after CLEAR_ERRORS (should re-enable cleanly)")
        cleared = s3.get("guard_mode") in (0, 1) and s3.get("fault") == 0

        print()
        self.note("OPTIONAL re-latch check: issue CLEAR_ERRORS while the E-STOP condition is")
        self.note("still live → it should immediately re-latch (guard_mode returns to 2).")
        self.enter("Press Enter when done (or to skip the optional step)")

        sug = None
        if not self.no_serial:
            good = tripped and latched and cleared
            sug = (f"{'looks PASS' if good else 'REVIEW'} — tripped→ESTOP:{tripped}, "
                   f"stayed-latched-after-clear:{latched}, clean-reenable:{cleared} "
                   f"(latched-after-clear is the load-bearing one).")
        return self.verdict(1, "Guard E-STOP latch", sug)

    def check2(self):
        self.head("CHECK 2 — Monotonic clock / clock-step + run_mpc-restart seq-guard  [item 14]")
        print("Validates: (A) a wall-clock ANCHOR STEP must not perturb control (all interval")
        print("arithmetic is on micros64() now); (B) restarting run_mpc must be accepted")
        print("IMMEDIATELY — pre-fix, a persisted seq vs a host-reset stream bricked control")
        print("for minutes after ~half of restarts (phantom MPC_STALE E-STOP).")
        print()
        self.expect([
            "(A) across the bridge (re)connect, fault stays 0:NONE — no spurious MPC_STALE",
            "    /LINK_LOST, no deferred stow armed, and u0 / leg positions do not jump",
            "(B) after a run_mpc restart, sp_age_ms drops back small and guard_mode returns",
            "    to 1:ENABLED within ~2 s, fault stays 0:NONE (NOT stuck at MPC_STALE/ESTOP)",
        ])
        self.enter("Press Enter to begin check 2")

        print(f"{C.bold}Part A — wall-clock anchor step:{C.rst}")
        self.operator_action([
            "With run_mpc streaming, restart the bridge/time-anchor: bring the",
            "teensy_bridge_node DOWN then UP (this forces a fresh wall-clock anchor step),",
            "e.g. Ctrl-C the bridge and relaunch:",
            f"  {C.hd}ros2 launch jugglebot teensy_bridge_launch.py{C.rst}",
            "Watch [diag] synced flip to 1 while fault stays 0 and u0 does not jump.",
        ])
        self.enter("Press Enter once the bridge has reconnected")
        sA = self.snapshot_after("after bridge reconnect (anchor step)")
        a_ok = sA.get("fault") == 0

        print()
        print(f"{C.bold}Part B — run_mpc restart (seq-guard gap-reset):{C.rst}")
        self.operator_action([
            "Restart run_mpc: Ctrl-C it, then relaunch the SAME command:",
            f"  {C.hd}python run_mpc.py --pose 0,0,170,0,0,0 --duration 180{C.rst}",
            "The stream must be accepted immediately (no minutes-long MPC_STALE brick).",
        ])
        self.enter("Press Enter once run_mpc has restarted")
        self.wait_until(lambda s: s.get("sp_age_ms", 9999) < 500 and s.get("fault") == 0,
                        "sp_age_ms small again and fault=0:NONE (stream re-accepted)",
                        timeout=30.0)
        sB = self.snapshot_after("after run_mpc restart")
        b_ok = sB.get("fault") == 0 and sB.get("sp_age_ms", 9999) < 500

        sug = None
        if not self.no_serial:
            sug = (f"{'looks PASS' if (a_ok and b_ok) else 'REVIEW'} — "
                   f"anchor-step-no-fault:{a_ok}, restart-reaccepted:{b_ok}.")
        return self.verdict(2, "Monotonic clock + seq-guard restart", sug)

    def check3(self):
        self.head("CHECK 3 — NetLock / UDP RX flood  [item 15]")
        print("Validates: a UDP flood on the STREAM/RPC ports must NOT hardfault the Teensy")
        print("(NetLock guards the shared lwIP pool under concurrent RX/TX); the bounded RX drain")
        print("keeps the 500 Hz interp undisturbed (deadline_misses/max_jitter stay 0); RTT bounded.")
        print("The junk flood is rejected on bad-magic → crc_err climbs (the 'flood landed' proof).")
        print()
        self.expect([
            "the scheduler stays alive — the Teensy LED keeps blinking and [diag] keeps updating"
            " (no NetLock hardfault under concurrent RX/TX)",
            "crc_err CLIMBS (junk rejected) — proves the flood is reaching + being handled",
            "interp_deadline_misses / interp_max_jitter_us stay 0 (via /teensy/profile) — the"
            " bounded RX drain keeps the 500 Hz interp undisturbed",
            "fault stays 0:NONE — the flood does not starve the real 40 Hz stream into MPC_STALE",
            "drain_cap MAY climb (soft signal — only moves under bursty >8-deep delivery)",
        ])
        self.note("Note: junk-flood packets bump crc_err (bad magic → rejected), NOT rx (rx counts"
                  " valid frames only). crc_err climbing is the reliable 'flood landed' signal.")
        self.enter("Press Enter to begin check 3")

        self.operator_action([
            "Have run_mpc streaming at 40 Hz (mpc_active=1) so the flood competes with real",
            "traffic. In a SEPARATE terminal you can watch the interp counters + RTT:",
            f"  {C.hd}ros2 topic echo /teensy/profile{C.rst}",
            "  (KeyValues interp_deadline_misses / interp_max_jitter_us should stay 0;",
            "   udp_rtt_us stays bounded)",
        ])
        self.wait_until(lambda s: s.get("mpc_active") == 1,
                        "mpc_active=1 (run_mpc streaming)")

        before = self.mon.snapshot() if not self.no_serial else {}
        dc0 = before.get("drain_cap")
        crc0 = before.get("crc_err")
        print(f"  {C.dim}before flood: crc_err={crc0}  drain_cap={dc0}{C.rst}")

        dur = _ask_float("Flood duration seconds", 6.0)
        rate = _ask_float("Flood aggregate packets/sec", 12000.0)
        print(f"{C.warn}About to fire a UDP junk-flood at {TEENSY_IP}:{PORT_STREAM} and "
              f":{PORT_RPC}{C.rst}")
        print(f"{C.warn}for {dur:.0f}s at ~{rate:.0f} pkt/s (sendto only — no port binding, "
              f"safe alongside the bridge).{C.rst}")
        try:
            go = input(f"{C.bold}Type 'flood' to fire (anything else skips):{C.rst} ").strip()
        except (EOFError, KeyboardInterrupt):
            go = ""
            print()
        if go == "flood":
            sent = _flood_udp(TEENSY_IP, [PORT_STREAM, PORT_RPC], dur, rate, self)
            print(f"  {C.dim}flood done — {sent} packets sent{C.rst}")
        else:
            self.note("flood skipped — induce your own flood now if you prefer, then continue")
            self.enter("Press Enter when the flood is complete")

        after = self.snapshot_after("after flood")
        self.note("Confirm the Teensy LED kept blinking throughout (no hardfault).")
        self.enter("Press Enter to confirm the LED stayed alive")

        sug = None
        if not self.no_serial and crc0 is not None and after.get("crc_err") is not None:
            landed = after["crc_err"] > crc0             # junk rejected → flood reached the Teensy
            drain_moved = dc0 is not None and after.get("drain_cap", dc0) > dc0
            alive = not self.mon.stale()                 # [diag] still updating → no hardfault
            no_estop = after.get("fault") == 0           # flood didn't starve into MPC_STALE
            if not landed:
                self.note("crc_err did NOT climb — the flood may not be reaching the Teensy. "
                          "Confirm the Jetson<->Teensy link is up (bridge running / mpc_active=1).")
            sug = (f"{'looks PASS' if (landed and alive and no_estop) else 'REVIEW'} — "
                   f"flood-reached-Teensy(crc_err {crc0}->{after['crc_err']}):{landed}, "
                   f"no-hardfault(diag-updating):{alive}, fault-stayed-NONE:{no_estop}, "
                   f"drain_cap {dc0}->{after.get('drain_cap')}(climbed:{drain_moved}). "
                   f"Confirm LED + interp counters (/teensy/profile) manually.")
        return self.verdict(3, "NetLock / RX flood", sug)

    def check4(self):
        self.head("CHECK 4 — MPC ↔ cold-start mutual exclusion  [item 16]")
        print("Validates: while the MPC stream is driving (mpc_active=1), cold-start moves")
        print("(HOME/ACTIVATE/DEACTIVATE) are REJECTED; a DEACTIVATE during an in-progress")
        print("deferred stow is rejected (no gravity drop); and a cold-start move is not")
        print("co-driven by the 40 Hz stream (no leg fight / jerk).")
        print()
        self.expect([
            "with mpc_active=1, a HOME/ACTIVATE/DEACTIVATE service returns failure (ERR_REJECTED)",
            "  and NO cold-start move starts — guard_mode stays ENABLED, legs do not move",
            "a DEACTIVATE issued during an in-progress deferred stow returns ERR_REJECTED",
            "  and the platform keeps its profiled descent (no gravity drop)",
            "a cold-start move (mpc_active=0) runs cleanly with no co-drive fight/jerk",
        ])
        self.enter("Press Enter to begin check 4")

        self.operator_action([
            "With mpc_active=1 (run_mpc streaming), issue a cold-start command and note the",
            "service RESULT — it must be REJECTED:",
            f"  {C.hd}ros2 service call /home std_srvs/srv/Trigger{C.rst}   (or /activate, /deactivate)",
            "Confirm from the service response that it was rejected, and that the legs did",
            "NOT move (watch [axes] — states unchanged, guard_mode stays 1:ENABLED).",
        ])
        self.enter("Press Enter once you've observed the rejection")
        self.snapshot_after("during rejected cold-start (legs unchanged)")

        print()
        self.operator_action([
            "Deferred-stow interlock: arm a deferred stow (induce a CAN3 loss→reconnect), and",
            "while it is in progress issue a DEACTIVATE → it must return ERR_REJECTED and the",
            "platform must keep its profiled descent (NO gravity drop).",
        ])
        self.enter("Press Enter once you've observed the deferred-stow DEACTIVATE rejection")

        print()
        self.operator_action([
            "Co-drive check: with mpc_active=0, run a normal cold-start move and confirm the",
            "40 Hz stream does not co-drive it — no leg fight or jerk.",
        ])
        self.enter("Press Enter once you've observed a clean, un-fought cold-start move")
        return self.verdict(4, "MPC/cold-start mutual exclusion",
                            "Assess from the service return codes + physical no-jerk "
                            "observation (this check is mostly operator-judged).")

    def check5(self):
        self.head("CHECK 5 — ISR priority / stow barrier  [item 17, items 2-3]")
        print("Validates: over a CAN-loss→reconnect deferred stow, the platform makes a SMOOTH")
        print("profiled descent (no position jump from a torn stow base captured across the")
        print("PRIMASK barrier), and interp jitter is unchanged/improved vs before.")
        print()
        self.expect([
            "during CAN3 loss: [canhealth] jugglebot sync 1→0 and fault→6:CAN_BUS_DOWN, stow armed",
            "on reconnect: a SMOOTH profiled descent — NO sudden position jump",
            "interp_max_jitter_us unchanged/improved (via /teensy/profile)",
        ])
        self.enter("Press Enter to begin check 5")
        self.operator_action([
            "With the legs homed+active holding a pose, induce a CAN3 loss then reconnect,",
            "e.g. briefly drop the CAN3 link (ip link) or power-cycle the ODrive supply.",
            "Watch the platform: the deferred stow must be a smooth profiled descent with no jump.",
            "In a separate terminal, note interp_max_jitter_us from /teensy/profile.",
        ])
        self.enter("Press Enter once you've observed the loss→reconnect→stow descent")
        self.snapshot_after("after stow descent")
        return self.verdict(5, "ISR priority / stow barrier",
                            "Judge smoothness of the descent physically + interp jitter "
                            "from /teensy/profile.")

    def check6(self):
        self.head("CHECK 6 — Hand-axis fault-eval  [item 17, item 5]")
        print("Validates the NEW behaviour: a HAND ODrive fault (active-error or disarm while")
        print("CLOSED_LOOP) now E-STOPs the LEGS (guard ESTOP, output gated) — fault-eval was")
        print("extended to NUM_AXES. But a stale hand HEARTBEAT alone must NOT arm the leg")
        print("watchdog/stow (the hand is never stowed; that path stays legs-only).")
        print()
        self.expect([
            "on a HAND fault: [axes] H shows *ERR, guard_mode→2:ESTOP, output→0, fault→5:ODRIVE_FATAL",
            "on a stale hand HEARTBEAT alone: [axes] H shows !STALE but guard_mode stays 1:ENABLED",
            "  (the legs are NOT E-STOPped/stowed by hand-heartbeat staleness)",
        ])
        self.enter("Press Enter to begin check 6")

        self.operator_action([
            "Induce a HAND ODrive FAULT — an active error, or disarm the hand while it is in",
            "CLOSED_LOOP. Watch the H column go *ERR and the guard latch to ESTOP.",
        ])
        self.enter("Press Enter once you've induced the hand fault")
        s = self.snapshot_after("after hand fault (legs should E-STOP)")
        hand_estop = s.get("guard_mode") == 2

        print()
        self.note("Clear the fault before the next sub-step (CLEAR_ERRORS + re-home hand as needed).")
        self.enter("Press Enter when ready for the stale-heartbeat sub-step")

        self.operator_action([
            "Now make the hand HEARTBEAT go STALE without an active error — e.g. disconnect",
            "just the hand ODrive node. [axes] H should show !STALE, but the legs must be",
            "unaffected (guard_mode stays 1:ENABLED — no E-STOP/stow from staleness alone).",
        ])
        self.enter("Press Enter once the hand heartbeat is stale")
        s2 = self.snapshot_after("after hand heartbeat stale (legs should be unaffected)")
        stale_ok = s2.get("guard_mode") != 2

        sug = None
        if not self.no_serial:
            sug = (f"{'looks PASS' if (hand_estop and stale_ok) else 'REVIEW'} — "
                   f"hand-fault-E-STOPs-legs:{hand_estop}, "
                   f"stale-hand-HB-does-not-E-STOP:{stale_ok}.")
        return self.verdict(6, "Hand-axis fault-eval", sug)

    def check7(self):
        self.head("CHECK 7 — Cold-start regression sweep  [Phase-4 parity]")
        print("Validates: no Tier-2 change regressed the Phase-4 cold-start parity — a normal")
        print("BOOT→HOMING→IDLE, a levelling cycle, and an activate/deactivate all still work")
        print("cleanly, with no spurious E-STOP.")
        print()
        self.expect([
            "BOOT→HOMING→IDLE completes; [axes] settle to s1:IDLE, fresh=7/7, no *ERR marks",
            "a levelling cycle completes normally",
            "activate → [axes] legs go s8:CLOSED_LOOP; deactivate → back to s1:IDLE",
            "guard_mode/fault stay clean throughout — no spurious E-STOP",
        ])
        self.enter("Press Enter to begin check 7")
        self.operator_action([
            "Run a normal cold-start sequence via your usual orchestrator/services:",
            "  home → level → activate → deactivate.",
            "Watch [axes] progress through the expected states with no *ERR / spurious E-STOP.",
        ])
        self.enter("Press Enter once the cold-start sweep is complete")
        self.snapshot_after("end of cold-start sweep")
        return self.verdict(7, "Cold-start regression sweep",
                            "Judge from the [axes] progression + a clean guard/fault throughout.")

    # ── report ────────────────────────────────────────────────────────────────
    def _write_report(self):
        lines = []
        lines.append("# Can-hub Tier-2 hardware-validation report")
        lines.append("")
        lines.append(f"- Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append("- Firmware: canhub-hardening Tier-2 (items 13-17 + 18B + review fixes), "
                     "flashed 2026-07-03")
        lines.append("- Checklist: `logbook/2026-07-02-canhub-hardening-tier2.md`")
        lines.append("- Runner: `tools/probes/canhub_tier2_hw_validation.py` "
                     "(read-only serial observer)")
        lines.append("")
        npass = sum(1 for r in self.results if r["verdict"] == "PASS")
        nfail = sum(1 for r in self.results if r["verdict"] == "FAIL")
        nskip = sum(1 for r in self.results if r["verdict"] == "SKIP")
        lines.append(f"**Summary: {npass} PASS / {nfail} FAIL / {nskip} SKIP "
                     f"({len(self.results)} recorded)**")
        lines.append("")
        lines.append("| # | Check | Verdict | Note |")
        lines.append("|---|-------|---------|------|")
        for r in self.results:
            note = (r["note"] or "").replace("|", "\\|")
            lines.append(f"| {r['id']} | {r['title']} | **{r['verdict']}** | {note} |")
        lines.append("")
        for r in self.results:
            lines.append(f"## Check {r['id']} — {r['title']} — {r['verdict']}  ({r['ts']})")
            if r["note"]:
                lines.append(f"- Note: {r['note']}")
            s = r.get("snapshot") or {}
            if s:
                lines.append("- Serial snapshot at verdict:")
                if "guard_mode" in s:
                    lines.append(f"    - guard: mpc_active={s.get('mpc_active')} "
                                 f"guard_mode={s.get('guard_mode')}:"
                                 f"{GUARD_MODE.get(s.get('guard_mode'))} "
                                 f"output={s.get('output')} sp_age_ms={s.get('sp_age_ms')}")
                if "fault" in s:
                    lines.append(f"    - diag: fault={s.get('fault')}:"
                                 f"{FAULT_STATE.get(s.get('fault'))} "
                                 f"link={s.get('link')} drain_cap={s.get('drain_cap')} "
                                 f"seq_gaps={s.get('seq_gaps')} crc_err={s.get('crc_err')} "
                                 f"synced={s.get('synced')}")
                if "axes" in s:
                    axs = " ".join(f"{t}:s{st}/{a}{mk.strip()}" for (t, st, a, mk) in s["axes"])
                    lines.append(f"    - axes: fresh={s.get('axes_fresh')}/"
                                 f"{s.get('axes_total')}  {axs}")
            tail = r.get("serial_tail") or []
            if tail:
                lines.append("- Raw serial tail:")
                lines.append("  ```")
                for t in tail:
                    lines.append(f"  {t}")
                lines.append("  ```")
            lines.append("")
        with open(self.report_path, "w") as f:
            f.write("\n".join(lines) + "\n")


def _ask_float(prompt, default):
    try:
        raw = input(f"  {prompt} [{default}]: ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return default
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _flood_udp(host, ports, duration, rate, runner):
    """Fire junk UDP datagrams at host:ports for `duration` s at ~`rate` pkt/s.

    sendto() only — never binds a receive port, so it does not contend with the
    live bridge that owns 5005/5006. Junk (bad magic) → dropped on the Teensy's
    CRC/magic check; the point is to exercise the bounded RX drain (drain_cap).
    """
    payload = bytes([0xAB]) * 40
    socks = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in ports]
    stop = time.monotonic() + duration
    burst = 80
    sleep = max(0.0, burst * len(ports) / max(rate, 1.0))
    sent = 0
    last_print = 0.0
    try:
        while time.monotonic() < stop:
            for _ in range(burst):
                for sk, pt in zip(socks, ports):
                    try:
                        sk.sendto(payload, (host, pt))
                        sent += 1
                    except OSError:
                        pass
            now = time.monotonic()
            if now - last_print > 1.0:
                s = runner.mon.snapshot() if not runner.no_serial else {}
                print(f"  {C.dim}flooding... sent={sent} "
                      f"crc_err={s.get('crc_err','?')} drain_cap={s.get('drain_cap','?')}{C.rst}")
                last_print = now
            time.sleep(sleep)
    finally:
        for sk in socks:
            sk.close()
    return sent


CHECKS = {1: "check1", 2: "check2", 3: "check3", 4: "check4",
          5: "check5", 6: "check6", 7: "check7"}
TITLES = {
    1: "Guard E-STOP latch [13]",
    2: "Monotonic clock + seq-guard restart [14]",
    3: "NetLock / RX flood [15]",
    4: "MPC/cold-start mutual exclusion [16]",
    5: "ISR priority / stow barrier [17 items 2-3]",
    6: "Hand-axis fault-eval [17 item 5]",
    7: "Cold-start regression sweep [Phase-4 parity]",
}


def parse_checks(arg):
    if not arg:
        return list(range(1, 8))
    out = []
    for tok in arg.split(","):
        tok = tok.strip()
        if tok.isdigit() and int(tok) in CHECKS:
            out.append(int(tok))
    return out or list(range(1, 8))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0", help="Teensy USB serial device")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--check", default=None,
                    help="comma-separated check numbers (default: all 1-7)")
    ap.add_argument("--list", action="store_true", help="list the checks and exit")
    ap.add_argument("--no-serial", action="store_true",
                    help="narrate only; operator observes serial manually (no pyserial)")
    ap.add_argument("--no-color", action="store_true")
    ap.add_argument("--report", default=None, help="report path (default: temp/logs/...)")
    args = ap.parse_args()

    if args.no_color or not sys.stdout.isatty():
        C.off()

    if args.list:
        print("Can-hub Tier-2 hardware-validation checks:")
        for i in range(1, 8):
            print(f"  {i}. {TITLES[i]}")
        return 0

    selected = parse_checks(args.check)

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    if args.report:
        report_path = args.report
    else:
        logdir = os.path.join(repo_root, "temp", "logs")
        os.makedirs(logdir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = os.path.join(logdir, f"canhub_tier2_validation_{stamp}.md")

    mon = None
    no_serial = args.no_serial
    if not no_serial:
        try:
            mon = SerialMonitor(args.port, args.baud)
        except Exception as e:
            print(f"{C.warn}Could not open serial {args.port}: {e}{C.rst}")
            print(f"{C.warn}Falling back to --no-serial (narrate only). Close any other "
                  f"serial monitor (screen / pio device monitor) and re-run to enable "
                  f"live observation.{C.rst}")
            no_serial = True

    runner = Runner(mon, report_path, no_serial)

    # ── intro banner ──────────────────────────────────────────────────────────
    runner.head("Can-hub Tier-2 hardware validation  (Fable-5 hardening)")
    print("Firmware under test: canhub-hardening Tier-2 (items 13-17 + 18B + review fixes),")
    print("flashed 2026-07-03. Checklist: logbook/2026-07-02-canhub-hardening-tier2.md")
    print()
    print(f"{C.bold}NEW behaviours to EXPECT (intended, not faults):{C.rst}")
    print("  (a) the guard E-STOP now LATCHES until CLEAR_ERRORS (no auto-recovery)")
    print("  (b) a HAND ODrive fault now E-STOPs the legs (guard ESTOP, output gated)")
    print("  (c) a dead Jetson<->Teensy link reads fatal on robot_state")
    print("  (d) a cross-axis disarm reads fatal")
    print()
    print(f"This runner is READ-ONLY: it observes the Teensy serial debug console and records")
    print(f"verdicts. YOU run the robot-actuating commands when prompted.")
    print(f"  Report: {C.hd}{report_path}{C.rst}")
    if no_serial:
        print(f"  Serial: {C.warn}DISABLED (narrate only){C.rst}")
    else:
        print(f"  Serial: {C.ok}{args.port} @ {args.baud}{C.rst}  "
              f"(waiting for the 1 Hz [diag]/[guard] block...)")
        time.sleep(2.0)
        runner.show_live("current")
    print()
    print(f"Selected checks: {', '.join(str(i) for i in selected)}")
    runner.enter("Press Enter to begin")

    try:
        for i in selected:
            print()
            getattr(runner, CHECKS[i])()
    except KeyboardInterrupt:
        print(f"\n{C.warn}Interrupted — writing partial report.{C.rst}")
    finally:
        runner._write_report()
        if mon:
            mon.close()

    # ── final summary ─────────────────────────────────────────────────────────
    print()
    runner.head("Validation summary")
    for r in runner.results:
        col = {"PASS": C.ok, "FAIL": C.bad, "SKIP": C.warn}[r["verdict"]]
        print(f"  {r['id']}. {r['title']:<44} {col}{r['verdict']}{C.rst}"
              + (f"  — {r['note']}" if r["note"] else ""))
    npass = sum(1 for r in runner.results if r["verdict"] == "PASS")
    nfail = sum(1 for r in runner.results if r["verdict"] == "FAIL")
    print()
    print(f"  {C.ok}{npass} PASS{C.rst} / {C.bad}{nfail} FAIL{C.rst} / "
          f"{len(runner.results)} recorded")
    print(f"  Report written: {C.hd}{report_path}{C.rst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
