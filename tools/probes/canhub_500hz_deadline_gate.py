#!/usr/bin/env python3
"""canhub_500hz_deadline_gate.py — automated 500 Hz interp deadline/jitter
PASS/ABORT soak gate for the can-bridge Teensy.

Closes plan item 19 of ``plans/archived/2026-07-05 canhub-hardening.md`` (Fable-5 Tier-2): the
interactive checklist runners (``tools/probes/canhub_tier2_hw_validation.py`` +
``tests/hardware/teensy_guard_validation.py``) covered the powered checks by eye;
this is the AUTOMATED gate the plan still asked for — a machine PASS/ABORT verdict
over a soak window, so the 500 Hz real-time path can be regression-checked without
an operator squinting at a serial log.

WHAT IT MEASURES (from the 1 Hz PROFILE UDP frame — see the ``Profile`` struct in
``config/generate_udp_protocol.py`` and the reference consumer
``tools/probes/teensy_link_profiling/jetson/profile_monitor.py``):

  * ``interp_deadline_misses`` — a *cumulative* count (firmware ``leg_interp.cpp``:
    a tick later than ``INTERP_PERIOD_US + JITTER_MISS_US`` = 2000 + 500 = 2500 us
    is a miss). The gate keys on the DELTA over the soak window, not the absolute
    value: the absolute count can carry boot-history misses (e.g. the ISR settling
    at power-on), so only "did the ISR miss a deadline DURING a steady soak?"
    matters. Default threshold: delta == 0.
  * ``interp_max_jitter_us`` — the worst interp-tick jitter in each 1 s window
    (firmware resets it after every PROFILE emit). The gate keys on the worst
    window over the soak, which must stay at or under 500 us = ``JITTER_MISS_US``
    (the firmware's own miss boundary — exceed it and that tick counts as a miss).
    Note: the firmware jitter is the ABSOLUTE tick deviation (early OR late,
    ``leg_interp.cpp``), whereas a firmware deadline MISS counts only late ticks
    (``dt > period + boundary``) — so this jitter gate is intentionally a hair
    stricter than the miss counter (a rare >500 us EARLY tick trips it too; a
    periodic hardware timer does not fire that early, so in practice they agree).

READ-ONLY / NO MOTION. The probe only *listens*: it binds the STREAM port and
ingests PROFILE frames. It never sends a setpoint, a heartbeat, or any command, so
it cannot move the robot. The 500 Hz interp ISR runs continuously regardless of
whether a setpoint stream is present (it holds/extrapolates position), so its
real-time health is observable with the legs held. Because it OWNS the STREAM port
(5005), the ROS2 ``teensy_bridge_node`` must be DOWN while it runs (same
constraint as ``profile_monitor.py``). It touches no firmware and no flash, so it
MAY run alongside a concurrent CAN3-diagnosis session.

ISR / stow-re-arm soak: run a long ``--duration`` and, if you want the deferred-stow
path exercised too, have the operator induce a CAN-loss -> reconnect deferred stow
(the Tier-2 check-5 mechanic) mid-window — the cumulative deadline counter persists
across the stow, so any miss it introduces is caught by the same delta gate.

Usage:
    source ~/Desktop/PDJ_venv/venv/bin/activate
    # Bridge DOWN, Teensy powered:
    python tools/probes/canhub_500hz_deadline_gate.py --duration 120
    python tools/probes/canhub_500hz_deadline_gate.py --duration 300 --bind-ip 192.168.42.1
    python tools/probes/canhub_500hz_deadline_gate.py --max-jitter-us 400 --max-deadline-misses 0
    python tools/probes/canhub_500hz_deadline_gate.py --self-test   # offline logic check, no hardware

Exit code: 0 = PASS, 1 = FAIL (a threshold exceeded), 2 = ABORT (no/too-little data).
A timestamped report is written under temp/probes/ (gitignored).
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", ".."))
# The generated wire protocol is the single source of truth (decode_frame,
# Profile, MsgType, PORT_STREAM, CrcError). Mirror protocol.py's mechanism.
sys.path.insert(0, os.path.join(_REPO, "config", "generated"))
import udp_protocol as p  # noqa: E402

# The firmware's own miss boundary (leg_interp.cpp: JITTER_MISS_US = INTERP_PERIOD_US / 4).
JITTER_MISS_US = 500
INTERP_PERIOD_US = 2000


def _default_out_dir() -> str:
    return os.path.join(_REPO, "temp", "probes")


@dataclass
class SoakResult:
    frames: int = 0
    baseline_misses: Optional[int] = None
    final_misses: Optional[int] = None
    max_jitter_us: int = 0
    jitter_samples: List[int] = field(default_factory=list)
    crc_errors: int = 0
    decode_errors: int = 0
    first_t_teensy_us: Optional[int] = None
    last_t_teensy_us: Optional[int] = None

    @property
    def miss_delta(self) -> int:
        if self.baseline_misses is None or self.final_misses is None:
            return 0
        return self.final_misses - self.baseline_misses

    def ingest_profile(self, pr) -> None:
        self.frames += 1
        if self.baseline_misses is None:
            self.baseline_misses = pr.interp_deadline_misses
            self.first_t_teensy_us = pr.t_teensy_us
        self.final_misses = pr.interp_deadline_misses
        self.last_t_teensy_us = pr.t_teensy_us
        self.jitter_samples.append(pr.interp_max_jitter_us)
        if pr.interp_max_jitter_us > self.max_jitter_us:
            self.max_jitter_us = pr.interp_max_jitter_us


def evaluate(result: SoakResult, max_jitter_us: int, max_deadline_misses: int,
             min_frames: int):
    """Pure PASS/FAIL/ABORT decision — shared by the live path and --self-test.

    Returns (verdict, reasons): verdict in {"PASS", "FAIL", "ABORT"}.
    """
    if result.frames < min_frames:
        return "ABORT", [
            f"only {result.frames} PROFILE frame(s) in the window (need >= {min_frames}) "
            f"— is the ROS2 bridge DOWN and the Teensy powered + on the link?"
        ]
    reasons = []
    if result.miss_delta > max_deadline_misses:
        reasons.append(
            f"interp deadline misses +{result.miss_delta} over the soak "
            f"(threshold {max_deadline_misses}; absolute {result.baseline_misses}->{result.final_misses})"
        )
    if result.max_jitter_us > max_jitter_us:
        reasons.append(
            f"interp worst-window jitter {result.max_jitter_us} us "
            f"(threshold {max_jitter_us} us; period {INTERP_PERIOD_US} us)"
        )
    return ("PASS" if not reasons else "FAIL"), reasons


def _summarize(result: SoakResult, max_jitter_us: int, max_deadline_misses: int,
               min_frames: int, duration: float) -> str:
    verdict, reasons = evaluate(result, max_jitter_us, max_deadline_misses, min_frames)
    js = result.jitter_samples
    jmin = min(js) if js else 0
    jmean = (sum(js) / len(js)) if js else 0.0
    span_s = None
    if result.first_t_teensy_us is not None and result.last_t_teensy_us is not None:
        span_s = (result.last_t_teensy_us - result.first_t_teensy_us) / 1e6
    lines = [
        f"verdict           : {verdict}",
        f"soak window        : {duration:.0f} s requested"
        + (f", {span_s:.1f} s of Teensy time observed" if span_s is not None else ""),
        f"PROFILE frames     : {result.frames} (min required {min_frames})",
        f"deadline misses    : baseline {result.baseline_misses} -> final {result.final_misses}"
        f"  (delta {result.miss_delta}, threshold {max_deadline_misses})",
        f"interp jitter (us) : min {jmin}  mean {jmean:.0f}  max {result.max_jitter_us}"
        f"  (threshold {max_jitter_us}, miss boundary {JITTER_MISS_US})",
        f"UDP crc/decode err : {result.crc_errors} / {result.decode_errors} (informational)",
    ]
    if reasons:
        lines.append("fail reasons       :")
        lines.extend(f"  - {r}" for r in reasons)
    return "\n".join(lines)


def _write_report(out_dir: str, ts: str, body: str, argv: List[str]) -> str:
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"canhub_500hz_gate_{ts}.md")
    with open(path, "w") as f:
        f.write(f"# can-bridge 500 Hz deadline/jitter soak gate — {ts}\n\n")
        f.write(f"Command: `{' '.join(argv)}`\n\n")
        f.write("```\n")
        f.write(body)
        f.write("\n```\n")
    return path


def run_live(bind_ip: str, port: int, duration: float, max_jitter_us: int,
             max_deadline_misses: int, min_frames: int, out_dir: str,
             argv: List[str]) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((bind_ip, port))
    except OSError as e:
        # Port busy is the common setup mistake — the ROS2 teensy_bridge_node still
        # holds 5005. Route it to ABORT (exit 2), NOT an uncaught crash: an unhandled
        # exception exits Python with code 1, which is THIS gate's FAIL verdict, so a
        # bridge-still-up mistake would masquerade as "the 500 Hz path failed".
        sock.close()
        print(f"[gate] ABORT: cannot bind {bind_ip}:{port} ({e}). Is the ROS2 "
              "teensy_bridge_node still up? It must be DOWN to run this gate.")
        return 2
    sock.settimeout(0.5)
    print(f"[gate] listening on {bind_ip}:{port} for PROFILE frames — soak {duration:.0f} s")
    print(f"[gate] thresholds: deadline-miss delta <= {max_deadline_misses}, "
          f"worst-window jitter <= {max_jitter_us} us; min frames {min_frames}")
    print("[gate] READ-ONLY (no setpoints/heartbeats sent). Bridge must be DOWN. Ctrl-C to stop early.")

    result = SoakResult()
    t0 = time.time()
    try:
        while (time.time() - t0) < duration:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            try:
                mt, _seq, payload = p.decode_frame(data)
            except p.CrcError:
                result.crc_errors += 1
                continue
            except ValueError:
                result.decode_errors += 1
                continue
            if mt != int(p.MsgType.PROFILE):
                continue
            try:
                pr = p.Profile.unpack(payload)
            except struct.error:
                # A PROFILE-typed frame that passed CRC but carried a short/garbled
                # payload — count as a decode error rather than crashing the soak.
                result.decode_errors += 1
                continue
            result.ingest_profile(pr)
            print(f"[gate] t={time.time()-t0:5.0f}s  frames={result.frames:3d}  "
                  f"misses={pr.interp_deadline_misses} (d{result.miss_delta})  "
                  f"jitter={pr.interp_max_jitter_us}us (max {result.max_jitter_us})")
    except KeyboardInterrupt:
        print("\n[gate] stopped early by operator")
    finally:
        sock.close()

    body = _summarize(result, max_jitter_us, max_deadline_misses, min_frames, duration)
    print("\n" + body)
    report = _write_report(out_dir, time.strftime("%Y%m%d_%H%M%S"), body, argv)
    print(f"\n[gate] report -> {report}")
    verdict, _ = evaluate(result, max_jitter_us, max_deadline_misses, min_frames)
    return {"PASS": 0, "FAIL": 1, "ABORT": 2}[verdict]


# ── Offline self-test — proves the PASS/FAIL/ABORT logic without hardware ──────

class _FakeProfile:
    def __init__(self, misses, jitter, t):
        self.interp_deadline_misses = misses
        self.interp_max_jitter_us = jitter
        self.t_teensy_us = t


def _soak_from(frames) -> SoakResult:
    r = SoakResult()
    for f in frames:
        r.ingest_profile(f)
    return r


def self_test() -> int:
    ok = True

    def check(name, got, want):
        nonlocal ok
        status = "ok" if got == want else "FAIL"
        if got != want:
            ok = False
        print(f"  [{status}] {name}: got {got}, want {want}")

    # A: clean 30 s soak — misses flat at a nonzero boot baseline, jitter well under.
    clean = [_FakeProfile(7, 120 + (i % 30), 1_000_000 * i) for i in range(30)]
    v, _ = evaluate(_soak_from(clean), JITTER_MISS_US, 0, 20)
    check("clean-soak (flat misses, low jitter)", v, "PASS")

    # B: a deadline miss appears mid-window (baseline 7 -> 9) — FAIL on delta,
    #    even though the absolute baseline was already nonzero.
    creep = [_FakeProfile(7 if i < 15 else 9, 150, 1_000_000 * i) for i in range(30)]
    v, reasons = evaluate(_soak_from(creep), JITTER_MISS_US, 0, 20)
    check("deadline-miss-delta", v, "FAIL")
    check("delta value", _soak_from(creep).miss_delta, 2)

    # C: one window spikes jitter to the miss boundary — FAIL on jitter.
    spike = [_FakeProfile(7, 120, 1_000_000 * i) for i in range(30)]
    spike[17].interp_max_jitter_us = 620
    v, _ = evaluate(_soak_from(spike), JITTER_MISS_US, 0, 20)
    check("jitter-spike over boundary", v, "FAIL")

    # D: too few frames (bridge up / Teensy off) — ABORT, not a false PASS.
    v, _ = evaluate(_soak_from(clean[:5]), JITTER_MISS_US, 0, 20)
    check("too-few-frames -> ABORT", v, "ABORT")

    # E: jitter exactly at the boundary passes; one over fails (boundary is strict-<).
    at_bound = [_FakeProfile(7, JITTER_MISS_US, 1_000_000 * i) for i in range(25)]
    v, _ = evaluate(_soak_from(at_bound), JITTER_MISS_US, 0, 20)
    check("jitter == threshold passes", v, "PASS")

    print("self-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="Automated 500 Hz interp deadline/jitter soak gate")
    ap.add_argument("--bind-ip", default="0.0.0.0",
                    help="local IP to bind (default 0.0.0.0; the teensy-link is 192.168.42.1)")
    ap.add_argument("--port", type=int, default=p.PORT_STREAM)
    ap.add_argument("--duration", type=float, default=120.0, help="soak window seconds (default 120)")
    ap.add_argument("--max-jitter-us", type=int, default=JITTER_MISS_US,
                    help=f"FAIL if worst-window interp jitter EXCEEDS this (default {JITTER_MISS_US} = "
                         "firmware miss boundary; a window exactly at the boundary passes)")
    ap.add_argument("--max-deadline-misses", type=int, default=0,
                    help="FAIL if the interp deadline-miss delta over the window exceeds this (default 0)")
    ap.add_argument("--min-frames", type=int, default=0,
                    help="minimum PROFILE frames to trust the verdict (default: max(10, duration*0.5))")
    ap.add_argument("--out-dir", default=_default_out_dir())
    ap.add_argument("--self-test", action="store_true",
                    help="run the offline PASS/FAIL/ABORT logic check (no hardware) and exit")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    min_frames = args.min_frames or max(10, int(args.duration * 0.5))
    return run_live(args.bind_ip, args.port, args.duration, args.max_jitter_us,
                    args.max_deadline_misses, min_frames, args.out_dir, sys.argv)


if __name__ == "__main__":
    sys.exit(main())
