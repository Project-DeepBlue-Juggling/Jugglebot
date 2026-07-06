#!/usr/bin/env python3
"""teensy_guard_validation.py — MPC-free, ZERO-MOTION guard/pipeline validation for
the can-hub hardening checks 1-3 (E-STOP latch, monotonic-clock/seq-guard, NetLock flood).

It owns the Teensy UDP link directly (the ROS2 teensy_bridge_node MUST be stopped —
it binds 5005/6) and drives the firmware guard state machine with a 40 Hz HOLD
setpoint stream, arming ``mpc_active`` at RUNTIME — the correct
cold-start-independent order (stream first, THEN arm), the inverse of launching the
bridge with ``enable_setpoint_output:=true`` (which arms before any stream → the
MPC_STALE self-E-STOP).

ZERO MOTION BY CONSTRUCTION: the legs are left IDLE and the tool HARD-REFUSES to arm
if any leg is CLOSED_LOOP. The firmware's MPC-staleness watchdog, seq-guard, and
NetLock/flood behaviour run identically whether the legs are IDLE or driving, so this
validates the *primary* guard criteria of checks 1-3 with nothing physically moving.
(The *secondary* "no physical jump/jerk" observations would need the full armed-leg
sequence — out of scope here.)

Observation is the read-only serial monitor from
``tools/probes/canhub_tier2_hw_validation.py`` (serial + UDP are different transports,
zero contention). Checks:

  1  E-STOP latch: arm+stream → STARVE the stream >250 ms (heartbeats KEPT alive
     so it's MPC_STALE, not LINK_LOST) → guard latches ESTOP; RESUME the stream and
     confirm it STAYS latched (pre-fix it auto-recovered); CLEAR_ERRORS → clean recover.
  2  Seq-guard restart: tear down the client + rebuild the stream from a fresh seq
     (== a run_mpc restart) → confirm immediate re-accept (fault stays NONE, sp_age
     drops) — the review-caught regression that bricked control after ~half of restarts.
     (The wall-anchor clock-step half of this check is covered by the native clock-step tests.)
  3  NetLock flood: flood the UDP ports (sendto-only) while streaming → no
     hardfault ([diag] keeps updating), crc_err climbs, fault stays NONE.

MOTION SAFETY: arming ``mpc_active=1`` enables the firmware output gate. With every leg
IDLE this drives nothing, but it IS motor authority — run it per the operator
convention: workspace clear, E-stop in hand, ROS2 bridge DOWN.

Usage:
    source ~/Desktop/PDJ_venv/venv/bin/activate
    # stop the ROS2 stack first (teensy_bridge_node owns UDP 5005/6)
    python tests/hardware/teensy_guard_validation.py
    python tests/hardware/teensy_guard_validation.py --check 1
    python tests/hardware/teensy_guard_validation.py --list
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _p in (_REPO, os.path.join(_REPO, "tools", "probes")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Observation + display reused from the read-only serial probe.
import canhub_tier2_hw_validation as probe  # noqa: E402
from canhub_tier2_hw_validation import C, SerialMonitor, fmt_guard, fmt_diag, fmt_axes  # noqa: E402

from controller.teensy_link import (  # noqa: E402
    TeensyLinkClient, RpcClient, RpcServer, RpcError, RpcTimeout, TimeOfDayServer,
    rpc_args,
)
from controller.teensy_link import protocol as p  # noqa: E402
from controller.teensy_link.protocol import MsgType, RpcMethod, RpcStatus, Setpoint, Telemetry  # noqa: E402

CLOSED_LOOP = 8
_FLAG_HOLD = 0x1 | 0x2   # FLAG_HAS_U1 | FLAG_HAS_U2
_STREAM_HZ = 40.0
MPC_STALENESS_MS = 250   # firmware MPC_CMD_STALENESS_US = 250000


class HoldStreamer:
    """Owns the Teensy UDP link; streams a 40 Hz HOLD at the measured pose and
    arms/disarms mpc_active at runtime. Never commands motion (legs stay IDLE)."""

    def __init__(self):
        self._client = None
        self._rpc_srv = None
        self._tod = None
        self._rpc = None
        self._pose = [0.0] * p.NUM_LEGS
        self._have_pose = False
        self._lock = threading.Lock()
        self._stream_on = threading.Event()   # set → send frames
        self._stream_stop = threading.Event()  # set → thread exits
        self._thread = None
        self.armed = False

    # ── link ──────────────────────────────────────────────────────────────────
    def connect(self):
        self._client = TeensyLinkClient(
            teensy_addr=(p.TEENSY_IP, p.PORT_STREAM), bind_host="0.0.0.0")
        self._client.start()
        self._rpc_srv = RpcServer(self._client)
        self._tod = TimeOfDayServer(self._rpc_srv)
        self._rpc = RpcClient(self._client, default_timeout=0.5, default_retries=2)
        self._client.subscribe(int(MsgType.TELEMETRY), self._on_telem)
        # DISARMED heartbeat (mpc_active=0): keeps the link UP + the MPC_STALE
        # watchdog un-armed + cold-start permitted. Structural double-write matches
        # the bridge.
        self._client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)
        self._client.set_heartbeat_flags(0)
        self.armed = False

    def _on_telem(self, mt, seq, payload, addr):
        try:
            t = Telemetry.unpack(payload)
        except Exception:
            return
        with self._lock:
            self._pose = [float(t.pos_rev[i]) for i in range(p.NUM_LEGS)]
            self._have_pose = True

    def wait_pose(self, timeout=3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._have_pose:
                    return True
            time.sleep(0.05)
        return False

    def pose(self):
        with self._lock:
            return list(self._pose)

    # ── hold stream ────────────────────────────────────────────────────────────
    def start_stream(self):
        self._stream_on.set()
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, name="hold-stream",
                                            daemon=True)
            self._thread.start()

    def _loop(self):
        period = 1.0 / _STREAM_HZ
        nxt = time.monotonic()
        while not self._stream_stop.is_set():
            if self._stream_on.is_set():
                pose = tuple(self.pose())
                sp = Setpoint(u0=pose, u1=pose, u2=pose,
                              v0=(0.0,) * p.NUM_LEGS, accel=(0.0,) * p.NUM_LEGS,
                              torque_ff=(0.0,) * p.NUM_LEGS, flags=_FLAG_HOLD,
                              t_origin_us=int(time.monotonic() * 1e6))
                try:
                    self._client.send_stream(int(MsgType.SETPOINT), sp.pack())
                except OSError:
                    pass
            nxt += period
            slp = nxt - time.monotonic()
            if slp > 0:
                time.sleep(slp)
            else:
                nxt = time.monotonic()

    def starve(self):
        """Stop SETPOINT frames but KEEP the heartbeat (→ MPC_STALE, not LINK_LOST)."""
        self._stream_on.clear()

    def resume(self):
        self._stream_on.set()

    # ── arm / disarm ───────────────────────────────────────────────────────────
    def arm(self):
        self._client.set_heartbeat_flags(0x1)   # bit0 = mpc_active
        self.armed = True

    def disarm(self):
        self._client.set_heartbeat_flags(0)
        self.armed = False

    def clear_errors(self):
        return self._rpc.call(int(RpcMethod.CLEAR_ERRORS),
                              rpc_args.encode_clear_errors(rpc_args.AXIS_ALL),
                              timeout=0.5, retries=2)

    def try_activate_while_armed(self):
        """For a manual cross-check: fire ACTIVATE and return the RpcStatus int
        (expect ERR_REJECTED while armed). Never actually cold-starts here."""
        try:
            self._rpc.call(int(RpcMethod.ACTIVATE),
                          rpc_args.encode_activate(rpc_args.AXIS_ALL),
                          timeout=0.5, retries=0)
            return int(RpcStatus.OK)
        except RpcError as e:
            return int(e.args[1]) if len(e.args) > 1 else -1
        except RpcTimeout:
            return int(RpcStatus.ERR_TIMEOUT)

    def restart_link(self):
        """Tear down the client (resets the tx seq to 0) and rebuild — a faithful
        run_mpc restart. Returns after the fresh stream is flowing again."""
        was_armed = self.armed
        self._stream_on.clear()
        self._stream_stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._thread = None
        self._stream_stop.clear()
        try:
            self._tod.close(); self._rpc.close(); self._client.stop()
        except Exception:
            pass
        self._have_pose = False
        self.connect()               # fresh TeensyLinkClient → tx seq restarts at 0
        self.wait_pose(3.0)
        self.start_stream()
        time.sleep(0.3)              # let a few fresh frames land
        if was_armed:
            self.arm()

    def close(self):
        try:
            self.disarm()
        except Exception:
            pass
        self._stream_on.clear()
        self._stream_stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            self._tod.close(); self._rpc.close(); self._client.stop()
        except Exception:
            pass


class Runner:
    def __init__(self, mon: SerialMonitor, streamer: HoldStreamer, report_path):
        self.mon = mon
        self.s = streamer
        self.report_path = report_path
        self.results = []

    def hr(self, ch="─"): print(f"{C.dim}{ch * 78}{C.rst}")
    def head(self, t): self.hr("═"); print(f"{C.hd}{C.bold}{t}{C.rst}"); self.hr("═")

    def show(self, label="live"):
        s = self.mon.snapshot()
        print(f"  {C.dim}{label}:{C.rst} {fmt_guard(s)}")
        print(f"       {fmt_diag(s)}")

    def enter(self, prompt):
        try:
            input(f"{C.bold}↳ {prompt}...{C.rst} ")
        except (EOFError, KeyboardInterrupt):
            print()

    def after(self, label, n=2):
        self.mon.wait_frames(n)
        s = self.mon.snapshot()
        self.show(label)
        return s

    def verdict(self, cid, title, suggestion=None):
        if suggestion:
            print(f"{C.bold}Auto-assessment:{C.rst} {suggestion}")
        try:
            ans = input(f"{C.bold}Verdict {cid} [p]ass/[f]ail/[s]kip:{C.rst} ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            ans = "s"; print()
        v = {"p": "PASS", "f": "FAIL", "s": "SKIP", "": "SKIP"}.get(ans, "SKIP")
        self.results.append((cid, title, v))
        print(f"  recorded: {v}")
        self._write()
        return v

    # ── SAFETY GATE ─────────────────────────────────────────────────────────────
    def assert_legs_idle(self) -> bool:
        if self.mon.stale():
            # Fail-safe: a snapshot() returns the LAST-parsed state regardless of
            # age. If the serial console froze after a good IDLE readout, a stale
            # snapshot could still show IDLE and wrongly pass — so refuse when the
            # serial is stale (the zero-motion guarantee rests on a FRESH readout).
            print(f"{C.bad}REFUSING TO ARM — serial diagnostics are STALE; cannot confirm "
                  f"the legs are IDLE. Check /dev/ttyACM0 and retry.{C.rst}")
            return False
        s = self.mon.snapshot()
        axes = s.get("axes") or []
        leg_axes = [a for a in axes if a[0] != "H"]
        if len(leg_axes) < p.NUM_LEGS:
            # Fail-safe: without a fresh [axes] readout we cannot confirm the legs
            # are IDLE, so we must NOT arm. (Serial down / no diagnostics yet.)
            print(f"{C.bad}REFUSING TO ARM — no fresh [axes] serial readout to confirm the "
                  f"legs are IDLE. Check the Teensy serial (/dev/ttyACM0) and retry.{C.rst}")
            return False
        cl = [tag for (tag, st, age, mk) in leg_axes if st == CLOSED_LOOP]
        if cl:
            print(f"{C.bad}REFUSING TO ARM — legs {cl} are CLOSED_LOOP. This tool must not "
                  f"drive a holding platform. Deactivate first (ros2 service call "
                  f"/deactivate) and relaunch this tool.{C.rst}")
            return False
        return True

    def arm_sequence(self) -> bool:
        """Disarmed → verify legs IDLE → stream hold → arm. Zero motion."""
        print(f"{C.bold}Arming (zero-motion): stream hold @ current pose, then mpc_active=1{C.rst}")
        if not self.s.wait_pose(3.0):
            print(f"{C.bad}No telemetry — is the Teensy powered and the bridge stopped?{C.rst}")
            return False
        if not self.assert_legs_idle():
            return False
        pose = self.s.pose()
        print(f"  hold pose (rev): {[round(x, 3) for x in pose]}")
        self.s.start_stream()
        time.sleep(0.4)
        self.after("streaming (disarmed)")
        self.s.arm()
        s = self.after("after arm (mpc_active should be 1, guard ENABLED)")
        ok = s.get("mpc_active") == 1 and s.get("guard_mode") == 1 and s.get("fault") == 0
        print(f"  {'✓ armed' if ok else '⚠ arm not confirmed — check serial'}")
        return True

    # ── checks ──────────────────────────────────────────────────────────────────
    def check1(self):
        self.head("CHECK 1 — E-STOP LATCH  (zero-motion)")
        print("arm+stream → STARVE (heartbeats kept) → MPC_STALE → guard latches ESTOP;")
        print("RESUME the stream → confirm it STAYS latched (pre-fix it auto-recovered);")
        print("CLEAR_ERRORS → clean recover.")
        self.enter("Enter to begin (arms mpc_active=1 — legs stay IDLE)")
        if not self.arm_sequence():
            return self.verdict(1, "E-STOP latch", "arm failed")
        print(f"\n{C.warn}Starving the setpoint stream (>250 ms), heartbeats kept alive...{C.rst}")
        self.s.starve()
        time.sleep(0.6)
        s1 = self.after("after starve (expect fault=MPC_STALE, guard=ESTOP, output=0)")
        tripped = s1.get("guard_mode") == 2 and s1.get("fault") == 1

        print(f"\n{C.warn}Resuming the stream — the STALE condition clears, but the E-STOP "
              f"must REMAIN latched...{C.rst}")
        self.s.resume()
        time.sleep(0.6)
        s2 = self.after("after resume (should STILL be guard=ESTOP — the latch)")
        latched = s2.get("guard_mode") == 2

        print(f"\n{C.warn}Issuing CLEAR_ERRORS RPC...{C.rst}")
        try:
            self.s.clear_errors()
            print("  CLEAR_ERRORS → OK")
        except Exception as e:
            print(f"  CLEAR_ERRORS raised: {e}")
        time.sleep(0.6)
        s3 = self.after("after CLEAR_ERRORS (stream fresh + armed → should re-ENABLE)")
        cleared = s3.get("guard_mode") in (0, 1) and s3.get("fault") == 0

        sug = (f"{'looks PASS' if (tripped and latched and cleared) else 'REVIEW'} — "
               f"tripped→ESTOP:{tripped}, stayed-latched-after-resume:{latched} (the crux), "
               f"clean-clear:{cleared}.")
        v = self.verdict(1, "E-STOP latch", sug)
        self.s.disarm()
        return v

    def check2(self):
        self.head("CHECK 2 — SEQ-GUARD RESTART  (zero-motion)")
        print("Tear down the link + rebuild the stream from a FRESH seq (== a run_mpc")
        print("restart) → confirm immediate re-accept: fault stays NONE, sp_age drops.")
        print("(Pre-fix: persisted seq vs host-reset stream → phantom MPC_STALE for minutes.)")
        self.enter("Enter to begin")
        if not self.arm_sequence():
            return self.verdict(2, "Seq-guard restart", "arm failed")
        base = self.after("armed + streaming (baseline)")
        base_ok = base.get("fault") == 0 and base.get("sp_age_ms", 9999) < 500

        print(f"\n{C.warn}Restarting the link (fresh TeensyLinkClient → tx seq resets to 0), "
              f"re-streaming, re-arming...{C.rst}")
        self.s.restart_link()
        time.sleep(0.6)
        s = self.after("after restart (fresh seq — should be re-accepted immediately)")
        reaccepted = s.get("fault") == 0 and s.get("sp_age_ms", 9999) < 500

        sug = (f"{'looks PASS' if (base_ok and reaccepted) else 'REVIEW'} — "
               f"baseline-clean:{base_ok}, fresh-seq-reaccepted(fault=NONE, sp_age small):"
               f"{reaccepted} — NOT stuck at MPC_STALE.")
        v = self.verdict(2, "Seq-guard restart", sug)
        self.s.disarm()
        return v

    def check3(self):
        self.head("CHECK 3 — NetLock / UDP FLOOD  (zero-motion)")
        print("Flood the UDP ports (sendto-only) while streaming → no hardfault ([diag]")
        print("keeps updating), crc_err climbs (flood lands), fault stays NONE.")
        self.enter("Enter to begin")
        if not self.arm_sequence():
            return self.verdict(3, "NetLock flood", "arm failed")
        before = self.mon.snapshot()
        crc0 = before.get("crc_err")
        print(f"  crc_err before: {crc0}")
        dur = probe._ask_float("Flood duration seconds", 6.0)
        rate = probe._ask_float("Flood aggregate packets/sec", 12000.0)

        class _R:  # _flood_udp needs .mon / .no_serial
            def __init__(self, mon): self.mon = mon; self.no_serial = False
        sent = probe._flood_udp(probe.TEENSY_IP, [probe.PORT_STREAM, probe.PORT_RPC],
                                dur, rate, _R(self.mon))
        print(f"  flood done — {sent} packets sent")
        after = self.after("after flood")
        landed = (crc0 is not None and after.get("crc_err", 0) > crc0)
        alive = not self.mon.stale()
        no_estop = after.get("fault") in (0, None)
        self.enter("Confirm the Teensy LED kept blinking (no hardfault) — Enter")
        sug = (f"{'looks PASS' if (landed and alive and no_estop) else 'REVIEW'} — "
               f"flood-landed(crc_err {crc0}→{after.get('crc_err')}):{landed}, "
               f"diag-still-updating:{alive}, fault-stayed-NONE:{no_estop}. "
               f"Also check interp deadline_misses/max_jitter via /teensy/profile if desired.")
        v = self.verdict(3, "NetLock flood", sug)
        self.s.disarm()
        return v

    def _write(self):
        import datetime
        lines = ["# Can-hub hardening guard-validation report (checks 1-3, zero-motion)", "",
                 f"- Generated: {datetime.datetime.now():%Y-%m-%d %H:%M:%S}",
                 "- Tool: tests/hardware/teensy_guard_validation.py (MPC-free, legs IDLE)", "",
                 "| # | Check | Verdict |", "|---|-------|---------|"]
        for (cid, title, v) in self.results:
            lines.append(f"| {cid} | {title} | **{v}** |")
        with open(self.report_path, "w") as f:
            f.write("\n".join(lines) + "\n")


CHECKS = {1: "check1", 2: "check2", 3: "check3"}
TITLES = {1: "E-STOP latch", 2: "Seq-guard restart", 3: "NetLock flood"}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--check", default=None, help="comma-separated (default all 1-3)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--no-color", action="store_true")
    ap.add_argument("--report", default=None)
    args = ap.parse_args()

    if args.no_color or not sys.stdout.isatty():
        C.off()
    if args.list:
        for i in (1, 2, 3):
            print(f"  {i}. {TITLES[i]}")
        return 0

    sel = [int(x) for x in args.check.split(",")] if args.check else [1, 2, 3]
    sel = [i for i in sel if i in CHECKS] or [1, 2, 3]

    import datetime
    report = args.report or os.path.join(
        _REPO, "temp", "logs",
        f"teensy_guard_validation_{datetime.datetime.now():%Y%m%d_%H%M%S}.md")
    os.makedirs(os.path.dirname(report), exist_ok=True)

    try:
        mon = SerialMonitor(args.port, args.baud)
    except Exception as e:
        print(f"Cannot open serial {args.port}: {e}")
        return 1

    streamer = HoldStreamer()
    runner = Runner(mon, streamer, report)

    runner.head("Can-hub hardening guard validation (MPC-free, ZERO-MOTION)")
    print(f"{C.bad}{C.bold}MOTION-AUTHORITY TOOL{C.rst} — it arms mpc_active=1 (enables the")
    print("firmware output gate). Legs are kept IDLE and it REFUSES to arm if any leg is")
    print("CLOSED_LOOP, so nothing should move — but treat it as arming the hardware:")
    print(f"  {C.warn}workspace clear · E-stop in hand · ROS2 teensy_bridge_node STOPPED{C.rst}")
    print(f"  Report: {report}")
    time.sleep(2.0)
    if mon.stale():
        print(f"{C.warn}No serial diagnostics — Teensy powered? another monitor open?{C.rst}")
    runner.show("current")
    print(f"\n{C.bold}Precondition:{C.rst} stop the ROS2 stack first (this tool binds UDP 5005/6).")
    try:
        input(f"{C.bold}Enter to connect the UDP link + begin, or Ctrl-C to abort...{C.rst} ")
    except (EOFError, KeyboardInterrupt):
        print(); mon.close(); return 0

    try:
        streamer.connect()
    except OSError as e:
        print(f"{C.bad}UDP bind failed ({e}) — is teensy_bridge_node still running? "
              f"Stop it and retry.{C.rst}")
        mon.close()
        return 1

    try:
        for i in sel:
            print()
            getattr(runner, CHECKS[i])()
    except KeyboardInterrupt:
        print(f"\n{C.warn}Interrupted.{C.rst}")
    finally:
        streamer.close()
        runner._write()
        mon.close()

    print()
    runner.head("Summary")
    for (cid, title, v) in runner.results:
        print(f"  {cid}. {title:<28} {v}")
    print(f"  Report: {report}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
