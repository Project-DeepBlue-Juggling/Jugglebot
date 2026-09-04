#!/usr/bin/env python3
"""Bench driver: FIRST streamed hand lane over the can-bridge (FW 17, T-H1..T-H3).

Drives the unified-7dof Phase 3 bench ladder
(``tests/hardware/session_unified7_hand_bringup.md``): the hand ODrive (axis 6)
commanded by the can-bridge's 500 Hz 7th interpolated lane, fed by this
driver's 40 Hz v6 knot stream. Every frame goes through the REAL
``SetpointPump`` (production-in-the-loop — the host hand step gate T-H3
exercises IS this pump), and the leg lanes are PINNED HOLDS at the legs' live
encoder positions so the six leg ODrives — CLOSED_LOOP or IDLE — see a
zero-deviation no-op throughout.

Lives in tests/hardware/ (not tools/probes/) because it COMMANDS THE MOTOR —
per ``tools/probes/README.md``; the ``*_bench.py`` name keeps it out of pytest
collection. Modeled on ``teensy_setpoint_bench.py`` (the leg equivalent): same
single-owner-link discipline, same stream-then-arm ordering, same
disarm-on-fault belt.

⚠️  MOTION WARNING — the hand tracks a streamed setpoint for as long as a stage
runs. NO BALL, E-STOP IN HAND. Defence in depth, all active:
  • driver: frames built through the production pump (hand step gate 5.0 rev,
    NaN reject), hand trajectory bounded to ``[0, HAND_MOTOR_MAX_POSITION]``
    minus margin, deviation belt (``--max-dev``, per-stage default ≤ the
    firmware lead clamp 2.0; VELOCITY-COMPENSATED like the firmware residual —
    ``|cmd − (enc + vel·age)|`` — because a static compare against the
    10-45 ms-stale telemetry cache reads ~0.95-4.3 rev of pure latency at the
    3 m/s stroke's ~95 rev/s plateau and would spuriously abort mid-stroke,
    commanding a stop into a fast hand), instant disarm on fault/Ctrl-C,
    bounded duration, first frame within the stated park gate of the commanded
    start (any residual is walked by the firmware recovery slew at ≤ 1 rev/s),
    and the 40 Hz hold stream runs in a BACKGROUND thread through the
    ARM prompt + verify window (true stream-then-arm — a stalled stream at the
    mpc_active edge latches MPC_STALE before the first loop frame).
  • firmware (FW 17): hand lead clamp ±2.0 rev against the age-extrapolated
    encoder, vel_ff cap 300 rev/s, stroke clip [0, 10.8] rev, hand overspeed
    E-STOP at 345 rev/s, MPC staleness E-STOP, and the observe-first
    ``MAX_DEVIATION_HAND_REV`` residual census on the ``[hand7]`` console line.

PRECONDITIONS (checked; the driver refuses to arm otherwise):
  • the ROS launch is DOWN — this driver is the SOLE owner of the UDP link;
  • can-bridge FW 17 aboard (BRIDGE_IDENTITY reports 17 — v6 link, else dark);
  • the hand ODrive ENERGISED: ``axis_state == CLOSED_LOOP`` (**checked** — at
    entry, after the ``--close-loop`` bring-up, and every tick of the run) in
    POSITION/PASSTHROUGH (**asserted, NEVER checked** — ``controller_mode`` /
    ``input_mode`` (``axis_state.h:42-43``) are never written by any firmware
    code, so the DIAGNOSTIC reports 0 forever; only ``--close-loop`` sending the
    frame establishes the mode). The streamed lane only moves an energised axis;
    an IDLE hand ignores every ``set_input_pos`` the lane puts on the wire and
    looks EXACTLY like a perfect hold (the 2026-09-03 first-sitting failure —
    the ``hold`` stage "passed" de-energised, T-H2a/b then aborted on the
    deviation belt with the encoder dead flat under a ramping command). Pass
    ``--close-loop`` to bring it up here; the driver verifies the bring-up took;
  • hand homed (encoder reference valid) and parked near a rest position —
    the firmware ``hand_source`` settle gate enforces this;
  • ``hand_source == STREAMED`` (this driver switches it via HAND_SOURCE_SET
    before arming; the switch is refused while armed or unsettled). With
    ``--no-source-switch`` the latch is left as-is — the T-H4(b) discard run;
  • hand axis error-free, ``fault_state == NONE``.

Stages (one per invocation; the runbook sequences them):
  hold      T-H1: stream the captured hand position for --duration (600 s for
            the full T-H1 soak). Pass: zero motion, zero guard trips, lead
            duty 0.
  triangle  T-H2a: ±(--tri-span/2) rev around (start + span/2) at --tri-speed
            rev/s (defaults: 2.0 rev span at 0.5 rev/s), repeated for
            --duration.
  stroke    T-H2b: the Phase-0-probe-2 replay — the LEGACY closed-form throw
            stroke (``hand_stroke.HandStrokeModel``) at --event-vel (default
            3.0 m/s), sampled to 40 Hz knots with ANALYTIC piecewise
            velocities (v6 HAS_V1 carries them). Tolerance context: the
            Phase 0 desk probe measured ≤ 3.25 mm worst-case reconstruction
            error for exactly this stroke with v1 carriage (≤ 11.84 mm
            without) — the HONEST prediction; do NOT expect float-exactness
            here (that applies to knot-aligned planner output only).
            PRECONDITION: hand parked within the ±0.10 rev settle band of 0
            (the stroke commands absolute positions from 0, like a kind-0
            throw; any residual inside the band is walked by the firmware
            recovery slew at ≤ 1 rev/s).
  step      T-H3a: attempt ONE hand knot --step-rev (default 6.0) past the
            pump gate mid-hold. Pass: the PUMP refuses host-side (reject
            counted, reason printed, nothing reaches the wire) and the hold
            continues unbroken.
  gap       T-H3c (run BEFORE ``hand7 arm``): hold with hand knots for
            --gap-pre s, drop the hand channel for --gap-s (legs keep
            streaming — the firmware lane must DECAY, the normative falling
            edge), then re-enter with a hand knot --gap-delta rev away
            (default +1.0). The pump's gap re-baseline accepts it as a first
            frame; the firmware Mode-1s onto it inside the ±2.0 rev lead band.
            Watch the hand: one brisk, bounded ~32 mm catch-up move is the
            expected picture. The re-entry is PROVEN, not assumed — the driver
            aborts if the firmware's hand echo has not moved within
            ``GAP_ECHO_DEADLINE_S`` of the re-entry knot (2026-09-04: a dropped
            ``hand_override`` made this stage measure a fiction, silently).

Source-latch / recovery utilities (no streaming, no arm):
  --source-only streamed|legacy   switch the firmware hand_source latch and
            exit — the T-H4 setup / close-out verb.
  --clear-errors   send CLEAR_ERRORS (all axes) and wait for fault_state NONE —
            the recovery verb after a latched guard E-STOP (MPC_STALE /
            MAX_DEVIATION from T-H3b). The next re-arm's output-enable edge
            runs the firmware recovery slew (bounded walk-back, ≤ 1 rev/s).
  --no-source-switch   stream WITHOUT forcing hand_source to STREAMED — the
            T-H4(b) verb: a v6 hand-bearing stream against a LEGACY latch must
            increment [hand7] discard_legacy and move nothing.

Instruments: the driver also subscribes CacheDiag (0x91) and logs a 1 Hz
windowed ``enc_frames`` deficit CSV beside the stage CSV (the headroom
runbook's row-21 recipe) — T-H1's drop-episode criterion reads it.

Usage (from the repo root; every streaming stage wants --close-loop unless the
hand is ALREADY in CLOSED_LOOP — with the launch down nothing else energises it):
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tests/hardware/hand_stream_bench.py --stage hold --duration 600 --close-loop
    python tests/hardware/hand_stream_bench.py --stage triangle --duration 60 --close-loop
    python tests/hardware/hand_stream_bench.py --stage stroke --event-vel 3.0 --close-loop
    python tests/hardware/hand_stream_bench.py --stage step --close-loop
    python tests/hardware/hand_stream_bench.py --stage gap --close-loop
    python tests/hardware/hand_stream_bench.py --source-only legacy
    python tests/hardware/hand_stream_bench.py --clear-errors
    python tests/hardware/hand_stream_bench.py --stage hold --duration 3 --no-source-switch --close-loop
"""
import argparse
import csv
import datetime
import os
import sys
import threading
import time

# LIVE tree first (incl. ros_ws/src — the installed colcon copy of
# jugglebot.hardware_config can lag the generated keys this driver needs).
for _p in ("/home/jetson/Desktop/Jugglebot",
           "/home/jetson/Desktop/Jugglebot/ros_ws/src/jugglebot"):
    if _p not in sys.path:
        sys.path.insert(0, _p)
from teensy_link import (  # noqa: E402
    TeensyLinkClient, RpcClient, RpcServer, TimeOfDayServer, MsgType,
    Telemetry, Diagnostic, HandCmdEcho, HeartbeatT2J, BridgeIdentity,
    CacheDiag, RpcMethod, RpcStatus, FaultState, RpcError,
)
from teensy_link import protocol as p  # noqa: E402
from teensy_link import rpc_args  # noqa: E402
from teensy_link.setpoint_pump import SetpointPump  # noqa: E402
import jugglebot.hardware_config as hw  # noqa: E402
import jugglebot.protocol_config as pc  # noqa: E402
from jugglebot.motion.trajectory.hand_stroke import (  # noqa: E402
    HandStrokeModel, LINEAR_GAIN_REV_PER_M, HAND_SETTLE_BAND_REV,
)

TEENSY_IP = "192.168.42.2"
HAND = int(p.NUM_LEGS)                    # axis 6
CLOSED_LOOP = 8
SETPOINT_HZ = 40.0
SEG_T = 0.025                             # SEGMENT_T_S — knot cadence
HAND_MAX_POS = float(hw.GEOM_HAND_MOTOR_HARD_STOP_REVS)   # 10.8, the metal
HAND_MARGIN = 0.2                         # driver keeps commands this far off the metal
ARM_VERIFY_GRACE_S = 0.7
DIAG_WAIT_S = 2.5                         # TWO forced-refresh opportunities + margin.
# The firmware's forced refresh is an AND of two INDEPENDENT clocks
# (telemetry.cpp:676-678): (now − last_sent_us >= DIAG_FORCE_PERIOD_US) && (slot
# == i·slots_per_axis). When the elapsed measurement lands just under 1 000 000 µs
# at axis 6's slot 84 the send is VETOED and the next opportunity is a whole
# second later — so the worst-case quiescent inter-diagnostic interval is ~2 s,
# not ~1 s. A 1.5 s wait would fail closed on a HEALTHY link; 2.5 s covers two
# opportunities plus margin. See _wait_hand_diag.
BRINGUP_VERIFY_S = 3.0                    # post---close-loop CLOSED_LOOP poll (leg bench's 3.0)
_T2J_FLAG_TIME_SYNCED = 0x1               # HeartbeatT2J flags bit 0
_T2J_FLAG_MPC_ACTIVE = 0x8
_T2J_FLAG_HAND_SOURCE_STREAMED = 0x40     # HeartbeatT2J flags bit 6 (FW 17)
MM_PER_REV = 1000.0 / float(LINEAR_GAIN_REV_PER_M)   # ≈ 31.63 mm per hand rev
ENC_BROADCAST_HZ = 100.0                  # ODrive get_encoder_estimate cadence (row-21 recipe)
CD_EPISODE_DEFICIT = -20.0                # headroom runbook row 21: a window past −20 = episode

# ── The gap stage's re-entry proof (2026-09-04 fix) ──────────────────────────
# ROOT CAUSE this pair exists for: the gap stage cannot tell "the hand tracked
# the re-entry" from "the re-entry was never sent" — both look like a flat
# encoder under a CSV that logs the displacement, and the phantom's deviation is
# exactly |gap_delta| while the stage's own belt is |gap_delta| + 0.5, so the
# silent case sits UNDER every other bar in the loop and passes. Under STREAMED
# the echo re-sources from axes[6].target_pos_rev and is emitted only when the
# interp actually TXed (telemetry.cpp:250-281), so a re-entry that reached the
# wire MUST move it: the stage asserts its premise instead of assuming it.
#   MOVE: the gap stage rides a Hold, so the lane has zero velocity when the
#     gap's decay ends and the pre-re-entry echo is STATIC by construction
#     (2026-09-04: bit-identical 0.00018 rev for all 1200 rows). 0.05 rev
#     (≈ 1.6 mm) is 5 % of the default 1.0 rev re-entry — unmissable if the knot
#     reached the firmware, unreachable if it did not.
#   DEADLINE: uplink ≤ ~1.5 ms (RTT 1-3 ms on this link) + the firmware Mode-1s
#     onto the re-entry over ONE knot (SEG_T 25 ms), with the first 500 Hz tick
#     after arrival (≤ 2 ms) already moving the target + one telemetry tick to
#     emit the echo (TELEM_RATE_HZ 100 ⇒ ≤ 10 ms) + downlink ≤ ~1.5 ms ≈ 40 ms
#     honest worst case. 0.25 s is ten knots — ~6× that — so the check can only
#     fire on a re-entry that never happened, never on a slow one.
GAP_ECHO_MOVE_REV = 0.05
GAP_ECHO_DEADLINE_S = 0.25

# Per-stage deviation-belt defaults (rev), used when --max-dev is not given.
# Derivation (2026-09-02 review fix — the belt is velocity-compensated,
# |cmd − (enc + vel·age)|, so encoder AGE no longer contributes):
#   hold/triangle/step: motion ≤ 0.5 rev/s ⇒ every latency term ≤ ~0.01 rev —
#     a tight 0.5 rev static-ish belt holds.
#   stroke: residual budget at the ~95 rev/s plateau = vel × (transport 1-3 ms
#     + servo lag a few ms) ≈ 0.3-0.6 rev, plus vel-staleness extrapolation
#     error during the ~1500 rev/s² accel/decel phases (stale vel × 10-45 ms
#     age) ≤ ~1.0 rev worst-case ⇒ 1.5 rev clears the honest terms and still
#     fires BELOW the firmware guards (lead clamp 2.0, deviation 2.5).
#   gap: the sanctioned re-entry catch-up is itself ≈ |--gap-delta| of honest
#     deviation, so the default widens to |gap_delta| + 0.5 (≤ the 2.0 cap).
_MAX_DEV_DEFAULT = {'hold': 0.5, 'triangle': 0.5, 'step': 0.5, 'stroke': 1.5}

_lock = threading.Lock()
_cache = {"telem": None, "telem_mono": 0.0, "diag": {}, "hb": None,
          "ident": None, "echo": None, "cd_prev": None, "cd_rows": []}


def _on_telem(mt, seq, payload, addr):
    tm = Telemetry.unpack(payload)
    with _lock:
        _cache["telem"] = tm
        _cache["telem_mono"] = time.monotonic()   # receive stamp → belt age


def _on_diag(mt, seq, payload, addr):
    d = Diagnostic.unpack(payload)
    with _lock:
        _cache["diag"][int(d.axis_id)] = d


def _on_hb(mt, seq, payload, addr):
    with _lock:
        _cache["hb"] = HeartbeatT2J.unpack(payload)


def _on_ident(mt, seq, payload, addr):
    with _lock:
        _cache["ident"] = BridgeIdentity.unpack(payload)


def _on_echo(mt, seq, payload, addr):
    # HAND_CMD_ECHO: under STREAMED the bridge re-sources this from
    # axes[6].target_* — i.e. the firmware's OWN reconstructed command, the
    # datum the stroke stage's reconstruction-error report compares against.
    # t_bridge_us is KEPT (2026-09-02 review fix): it is the bridge's WALL
    # stamp, which the driver's own TimeOfDayServer disciplines to Jetson
    # time.time() — the stroke stage evaluates the analytic AT that stamp
    # (time-aligned) instead of at NOW, removing the 30-100 mm latency smear
    # a NOW-comparison bakes in at stroke speed. Since the 2026-09-03 audit
    # fix the firmware age-corrects that stamp to the interp tick that WROTE
    # the echoed bytes (not the telemetry emit), so the residual error left
    # in the reconstruction read is wall-sync noise only.
    e = HandCmdEcho.unpack(payload)
    import struct as _s
    pos, vel_i, tor_i = _s.unpack('<fhh', bytes(e.data[:8]))
    with _lock:
        _cache["echo"] = (int(e.t_bridge_us), float(pos))


def _on_cachediag(mt, seq, payload, addr):
    # CacheDiag 0x91, 1 Hz: windowed per-axis enc_frames deltas → the headroom
    # runbook row-21 deficit (Δenc_frames_i − 100 × window_us/1e6). This is
    # T-H1's drop-episode instrument (2026-09-02 review fix: the criterion was
    # unmeasurable — nothing subscribed 0x91 with the launch down).
    d = CacheDiag.unpack(payload)
    with _lock:
        prev = _cache["cd_prev"]
        _cache["cd_prev"] = d
        if prev is None:
            return
        expect = ENC_BROADCAST_HZ * (float(d.window_us) / 1e6)
        deltas = [int(d.enc_frames[i]) - int(prev.enc_frames[i]) for i in range(7)]
        deficits = [(deltas[i] - expect) if (d.seen_mask >> i) & 1 else None
                    for i in range(7)]
        # t_local_us rides along (2026-09-04): the `t` column is the DRIVER's
        # monotonic arrival stamp, which carries UDP delivery smear, so a
        # cachediag row cannot be placed on the bridge's own timebase — the one
        # the stage CSV's echoes are stamped in. Carrying the frame's own stamp
        # makes a drop episode alignable against the stroke it belongs to.
        _cache["cd_rows"].append(
            (time.monotonic(), int(d.window_us), int(d.samples), deficits,
             int(d.rx_depth_hwm_jb), int(d.rx_cap_hits_jb), int(d.t_local_us)))


def _telem():
    with _lock:
        return _cache["telem"]


def _telem_with_age():
    """(Telemetry, receive-age seconds) — the velocity-compensated belt's pair."""
    with _lock:
        tm = _cache["telem"]
        mono = _cache["telem_mono"]
    if tm is None:
        return None, 0.0
    return tm, max(0.0, time.monotonic() - mono)


def _hand_pos():
    tm = _telem()
    return None if tm is None else float(tm.pos_rev[HAND])


def _hb():
    with _lock:
        return _cache["hb"]


def _fault():
    hb = _hb()
    return None if hb is None else int(hb.fault_state)


def _fault_name(fs):
    try:
        return FaultState(fs).name
    except ValueError:
        return str(fs)


def _hand_source_streamed():
    hb = _hb()
    return None if hb is None else bool(int(hb.flags) & _T2J_FLAG_HAND_SOURCE_STREAMED)


def _hand_diag():
    """The cached axis-6 DIAGNOSTIC, read UNDER ``_lock`` — the leg bench's
    ``axis_diag()`` (teensy_setpoint_bench.py:129-131). Returns None until one
    has arrived; the caches are written from the client's RX thread, so a
    lock-free ``_cache["diag"].get(...)`` is the odd one out in this file."""
    with _lock:
        return _cache["diag"].get(HAND)


def _wait_hand_diag(timeout_s=DIAG_WAIT_S):
    """Bounded wait for an axis-6 DIAGNOSTIC (returns it, or None on timeout).

    The cache being EMPTY is normal for up to a second and says NOTHING about
    the axis: the firmware emits a Diagnostic on-change, or on a staggered
    ~1 Hz forced refresh (``telemetry.cpp`` send-slot 84 for axis 6), and a
    quiescent IDLE hand changes nothing. The startup wait loop only waits for
    TELEMETRY + BRIDGE_IDENTITY, so whether a diagnostic has landed by the time
    the preconditions are checked is a coin flip — which is precisely how the
    2026-09-03 sitting ran a whole ladder against a de-energised hand, one run
    printing ``axis_state=?`` and the next ``axis_state=1``. Waiting long enough
    to cover TWO forced-refresh opportunities plus margin (``DIAG_WAIT_S``, and
    see its derivation: the refresh is an AND of two independent clocks, so a
    vetoed slot pushes the next send a whole second out) is what turns that gate
    from a coin flip into a verdict."""
    deadline = time.time() + float(timeout_s)
    while True:
        d = _hand_diag()
        if d is not None:
            return d
        if time.time() >= deadline:
            return None
        time.sleep(0.05)


def _diagnose_source_refusal():
    """Host-side gate diagnosis for a HAND_SOURCE_SET ERR_REJECTED (2026-09-03
    audit fix). The firmware's refusal is ONE opaque status by design (a single
    enforcement point, hand_source.cpp::hand_source_request), but every gate it
    checks has a proxy in the caches this driver already holds — so the driver
    disambiguates before printing rather than asking the operator to guess.
    Mirrors the firmware gates: mpc_active, axis-6 telemetry seen + fresh
    (MOTOR_FB_STALENESS_US = 150 ms), |pos − rest| ≤ HAND_SETTLE_BAND_REV for
    rest ∈ {retract, catch-prime}, |vel| ≤ HAND_SOURCE_SETTLE_VEL_RPS = 0.5.
    Returns the likely gate(s) as strings (empty = no gate implicated by the
    driver's caches — the truth is firmware-side; read [hand7])."""
    likely = []
    hb = _hb()
    if hb is not None and (int(hb.flags) & _T2J_FLAG_MPC_ACTIVE):
        likely.append("mpc_active — the setpoint stream is ARMED (disarm first)")
    tm, age = _telem_with_age()
    if tm is None:
        likely.append("axis-6 telemetry never seen by this driver (the firmware "
                      "refuses an unseen/zero-timestamp encoder cache)")
    else:
        if age > 0.15:
            likely.append(f"axis-6 telemetry stale (driver receive age "
                          f"{age * 1e3:.0f} ms vs the firmware's 150 ms "
                          f"MOTOR_FB_STALENESS gate)")
        pos = float(tm.pos_rev[HAND])
        vel = float(tm.vel_rps[HAND])
        band = HAND_SETTLE_BAND_REV
        retract_lo = float(hw.HOMING_HAND_ABS_POS_REV) - band   # homed rest, −0.1
        retract_hi = float(hw.JB_OP_HAND_RETRACT_REV) + band    # retract, 0.0
        prime = float(hw.JB_OP_HAND_CATCH_PRIME_REV)            # 9.9594
        if not ((retract_lo <= pos <= retract_hi)
                or abs(pos - prime) <= band):
            likely.append(f"not settled at a rest position (pos {pos:+.3f} rev "
                          f"vs retract [{retract_lo:+.2f}, {retract_hi:+.2f}] "
                          f"or catch-prime {prime:.2f}±{band:.2f})")
        if abs(vel) > 0.5:
            likely.append(f"hand moving (|vel| = {abs(vel):.2f} rev/s > the "
                          f"0.5 rev/s settle gate)")
    return likely


def _fw_version():
    with _lock:
        bi = _cache["ident"]
    return None if bi is None else int(bi.fw_version)


# ── Stage trajectories: (pos_rev, vel_rps) at t seconds after stage start ─────

class Hold:
    def __init__(self, start_rev):
        self.p0 = float(start_rev)

    def sample(self, t):
        return self.p0, 0.0


class Triangle:
    """start → start+span → start at ±speed, repeating. C0, with the bounded
    velocity steps at the vertices absorbed exactly the way a 40 Hz replan is."""

    def __init__(self, start_rev, span_rev, speed_rps):
        self.p0 = float(start_rev)
        self.span = float(span_rev)
        self.v = float(speed_rps)
        self.period = 2.0 * self.span / self.v

    def sample(self, t):
        ph = t % self.period
        half = self.period / 2.0
        if ph < half:
            return self.p0 + self.v * ph, self.v
        return self.p0 + self.span - self.v * (ph - half), -self.v


class Stroke:
    """The legacy closed-form throw stroke (HandStrokeModel) with ANALYTIC
    piecewise velocities (central differencing is wrong at the phase
    boundaries — Phase 0 decision 2). Starts from rest at 0, ends holding x3."""

    def __init__(self, v_mps, lead_s=1.0):
        self.m = HandStrokeModel(v_mps)
        self.lead = float(lead_s)          # settle time at 0 before the stroke
        self.t_total = (self.m.t_acc + self.m.t_vel + self.m.t_dec)
        self.gain = float(LINEAR_GAIN_REV_PER_M)

    def sample(self, t):
        ts = t - self.lead                 # 0 = accel start
        m = self.m
        if ts <= 0.0:
            return 0.0, 0.0
        if ts <= m.t_acc:
            return (0.5 * m.throwA * ts * ts * self.gain,
                    m.throwA * ts * self.gain)
        if ts <= m.t_acc + m.t_vel:
            tau = ts - m.t_acc
            return ((m.x1_m + m.v * tau) * self.gain, m.v * self.gain)
        if ts <= self.t_total:
            tau = ts - (m.t_acc + m.t_vel)
            return ((m.x2_m + m.v * tau + 0.5 * m.throwD * tau * tau) * self.gain,
                    (m.v + m.throwD * tau) * self.gain)
        return m.x3_m * self.gain, 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", choices=["hold", "triangle", "stroke", "step", "gap"],
                    default="hold")
    ap.add_argument("--duration", type=float, default=30.0,
                    help="run time after arm (s); T-H1 uses 600")
    ap.add_argument("--tri-span", type=float, default=2.0, help="triangle span (rev)")
    ap.add_argument("--tri-speed", type=float, default=0.5, help="triangle speed (rev/s)")
    ap.add_argument("--event-vel", type=float, default=3.0,
                    help="stroke stage: legacy event_vel (m/s)")
    ap.add_argument("--step-rev", type=float, default=6.0,
                    help="step stage: hand knot jump (rev) — must exceed the pump "
                         "gate (5.0) to demonstrate the host-side refusal")
    ap.add_argument("--gap-pre", type=float, default=3.0, help="gap stage: hand-bearing lead-in (s)")
    ap.add_argument("--gap-s", type=float, default=1.0, help="gap stage: hand-less window (s)")
    ap.add_argument("--gap-delta", type=float, default=1.0,
                    help="gap stage: re-entry displacement (rev, ≤ 1.5)")
    ap.add_argument("--max-dev", type=float, default=None, metavar="REV",
                    help="driver deviation belt |cmd − (enc + vel·age)| abort (rev), "
                         "velocity-compensated like the firmware residual; ≤ 2.0 "
                         "(the firmware hand lead clamp) so the belt fires first. "
                         "Default is per-stage (hold/triangle/step 0.5, stroke 1.5, "
                         "gap |gap-delta|+0.5) — see _MAX_DEV_DEFAULT's derivation")
    ap.add_argument("--close-loop", action="store_true",
                    help="bring the hand ODrive up first: POSITION/PASSTHROUGH + the "
                         "operational hand gains + CLOSED_LOOP (auto-holds at the "
                         "current position — no jolt), then VERIFY it took. Needed "
                         "for every streaming stage unless the hand is already in "
                         "CLOSED_LOOP: the driver refuses to arm an IDLE hand, which "
                         "ignores the streamed lane and mimics a perfect hold")
    ap.add_argument("--arm", action="store_true", help="skip the interactive ARM prompt")
    ap.add_argument("--source-only", choices=["streamed", "legacy"], default=None,
                    help="just switch the hand_source latch and exit (no stream, no arm)")
    ap.add_argument("--clear-errors", action="store_true",
                    help="send CLEAR_ERRORS (all axes) and exit — the recovery verb "
                         "after a latched guard E-STOP (no stream, no arm)")
    ap.add_argument("--no-source-switch", action="store_true",
                    help="do NOT force hand_source to STREAMED before streaming — "
                         "the T-H4(b) verb (a hand-bearing stream against a LEGACY "
                         "latch must be discarded + counted, and move nothing)")
    ap.add_argument("--csv", default=None, help="per-tick CSV (default: auto under temp/logs/)")
    args = ap.parse_args()
    if abs(args.gap_delta) > 1.5:
        ap.error("--gap-delta beyond ±1.5 rev — keep the re-entry catch-up bounded")
    if args.max_dev is None:
        args.max_dev = (max(0.5, abs(args.gap_delta) + 0.5) if args.stage == "gap"
                        else _MAX_DEV_DEFAULT[args.stage])
    if args.max_dev <= 0 or args.max_dev > 2.0:
        ap.error("--max-dev out of (0, 2.0] (2.0 = firmware MAX_LEAD_HAND_REV)")

    client = TeensyLinkClient(teensy_addr=(TEENSY_IP, p.PORT_STREAM), bind_host="0.0.0.0")
    client.start()
    rpc_server = RpcServer(client)
    tod = TimeOfDayServer(rpc_server)  # noqa: F841 — keeps the bridge time-synced
    rpc = RpcClient(client)
    client.subscribe(int(MsgType.TELEMETRY), _on_telem)
    client.subscribe(int(MsgType.DIAGNOSTIC), _on_diag)
    client.subscribe(int(MsgType.HEARTBEAT_T2J), _on_hb)
    client.subscribe(int(MsgType.BRIDGE_IDENTITY), _on_ident)
    client.subscribe(int(MsgType.HAND_CMD_ECHO), _on_echo)
    client.subscribe(int(MsgType.CACHE_DIAG), _on_cachediag)   # T-H1 drop-episode instrument
    client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)   # DISARMED
    client.set_heartbeat_flags(0)

    armed = False
    csv_f = None
    cd_f = None
    try:
        # ── Telemetry + identity ─────────────────────────────────────────────
        t0 = time.time()
        while (_hand_pos() is None or _fw_version() is None) and time.time() - t0 < 4.0:
            time.sleep(0.05)
        if _hand_pos() is None:
            print("ABORT: no telemetry from the Teensy (launch running? link dark = "
                  "version-skewed board — FW 17 required)."); return 2
        fw = _fw_version()
        if fw is not None and fw != rpc_args.EXPECTED_BRIDGE_FW_VERSION:
            print(f"ABORT: bridge_fw_version {fw} != expected "
                  f"{rpc_args.EXPECTED_BRIDGE_FW_VERSION} — lockstep flash first."); return 2
        start = _hand_pos()
        d0 = _hand_diag()               # non-blocking snapshot; '?' until one lands
        print(f"hand baseline: pos={start:+.4f} rev  "
              f"axis_state={d0.axis_state if d0 else '?'}  "
              f"active_errors={d0.active_errors if d0 else '?'}  "
              f"fault={_fault_name(_fault())}  hand_source_streamed={_hand_source_streamed()}")
        # ── Recovery verb: CLEAR_ERRORS + wait for fault_state NONE ──────────
        # Runs BEFORE the fault-latched abort below — it exists precisely for
        # that state (e.g. a MAX_DEVIATION latched in T-H3b, or an MPC_STALE).
        # The next re-arm's output-enable edge then runs the firmware recovery
        # slew (bounded walk-back toward the streamed command).
        if args.clear_errors:
            try:
                rpc.call(int(RpcMethod.CLEAR_ERRORS), rpc_args.encode_clear_errors())
            except RpcError as e:
                print(f"CLEAR_ERRORS refused: {e}"); return 2
            deadline = time.time() + 2.0
            while time.time() < deadline:
                if _fault() == int(FaultState.NONE):
                    print("CLEAR_ERRORS OK — fault_state NONE."); return 0
                time.sleep(0.05)
            print(f"CLEAR_ERRORS acked but fault_state still "
                  f"{_fault_name(_fault())} — a live condition re-latches "
                  f"(clear the CAUSE first).")
            return 2

        # ── Preconditions (refuse to arm otherwise) ──────────────────────────
        # ROOT CAUSE this block exists for (2026-09-03, the first FW 17 hand
        # sitting): the hand ODrive sat in IDLE for the WHOLE ladder, so it
        # ignored every set_input_pos the streamed lane put on the wire. The
        # hold stage "passed" — a de-energised axis holds beautifully — and
        # T-H2a/T-H2b then aborted on the deviation belt with the encoder DEAD
        # FLAT under a command ramping to +0.498 / +2.344 rev (the knots the
        # aborts name; the last knot actually written to CSV is one tick
        # earlier, because the belt breaks before the row is written). The
        # firmware was blameless.
        # The leg bench has refused to arm an axis that does not read
        # CLOSED_LOOP since it was written (teensy_setpoint_bench.py:337-356);
        # this driver dropped that gate on the ONE axis that must be energised,
        # AND inverted its fail-closed sense — `if d is not None and ...`
        # silently SKIPPED the only hand-ODrive health check whenever no
        # DIAGNOSTIC had arrived, which is a coin flip at startup (see
        # _wait_hand_diag). Both halves are closed here, leg-bench shape:
        # accumulate every unmet precondition so one run shows the operator all
        # of them.
        #
        # --source-only is EXEMPT from the energisation half: it is a latch verb
        # that commands no motion and never arms (--clear-errors, the other such
        # verb, has already returned above), and the gate exists only to stop a
        # STREAM running against a dead axis. Its pre-existing error/fault gates
        # are unchanged, off the same non-blocking snapshot as before.
        will_stream = args.source_only is None
        d = _wait_hand_diag() if will_stream else d0
        problems = []
        if d is None:
            # Gate B (the post---close-loop verification below) is the AUTHORITY
            # for a --close-loop run, so this entry abort stands down for one:
            # the IDLE→CLOSED_LOOP (1→8) transition is itself a diag_changed()
            # trigger (telemetry.cpp:61), so a --close-loop bring-up PRODUCES a
            # diagnostic, and Gate B's own `dcl is None` arm already fails closed
            # for exactly this shape. Keeping the abort here would make the
            # runbook's ONLY invocation shape (every streaming row carries
            # --close-loop) the likeliest spurious abort of the sitting.
            if will_stream and not args.close_loop:
                problems.append(
                    f"no DIAGNOSTIC for the hand (axis {HAND}) in {DIAG_WAIT_S:.1f} s "
                    f"— its axis_state and errors are UNKNOWN, and this gate fails "
                    f"CLOSED rather than arm blind. The likeliest cause is a stale "
                    f"or silent axis-6 DIAGNOSTIC slot, not a dead link: a quiescent "
                    f"axis changes nothing, and the forced refresh can be vetoed a "
                    f"slot at a time (see DIAG_WAIT_S). RE-RUN first; if it repeats, "
                    f"pass --close-loop (the 1→8 transition forces a diagnostic and "
                    f"hands the verdict to the post-bring-up check) and read [hand7] "
                    f"on the console")
        else:
            if will_stream and int(d.axis_state) != CLOSED_LOOP and not args.close_loop:
                problems.append(
                    f"axis_state={d.axis_state}, need CLOSED_LOOP={CLOSED_LOOP} — "
                    f"an IDLE hand ignores every streamed setpoint and its encoder "
                    f"stays flat under a ramping command (pass --close-loop to bring "
                    f"the hand up here: POSITION/PASSTHROUGH + gains + CLOSED_LOOP)")
            if int(d.active_errors) != 0:
                problems.append(
                    f"active_errors={d.active_errors}, need 0 (run --clear-errors "
                    f"first, or /clear_errors when the launch is up)")
        fs = _fault()
        if fs not in (None, int(FaultState.NONE)):
            problems.append(
                f"fault_state={_fault_name(fs)}, need NONE — a latch (run "
                f"--clear-errors first, or /clear_errors when the launch is up)")
        if problems:
            print("ABORT: preconditions not met:")
            for pb in problems:
                print(f"  • {pb}")
            return 2

        # ── Source-latch switch (before any arm — the gate requires !mpc) ────
        def set_source(streamed: bool) -> bool:
            try:
                rpc.call(int(RpcMethod.HAND_SOURCE_SET),
                         rpc_args.encode_hand_source_set(streamed))
            except RpcError as e:
                print(f"HAND_SOURCE_SET refused: {e}")
                gates = _diagnose_source_refusal()
                if gates:
                    print("  likely gate(s), from the driver's own caches: "
                          + "; ".join(gates))
                else:
                    print("  no gate implicated by the driver's caches — the "
                          "truth is firmware-side; read [hand7] on the console")
                # Actionable exit (2026-09-04 audit fix). This switch runs
                # BEFORE the --close-loop bring-up and returns hard, so a
                # de-energised hand that has DRIFTED out of the firmware settle
                # band can never reach the bring-up row 11b calls "the only
                # route" — and the operator is left with no on-screen way out.
                # (Not hypothetical: the 2026-09-03 baselines were −0.108 /
                # +0.209 / −0.076 rev, and +0.209 sits outside the retract band.)
                # The bring-up is deliberately NOT reordered ahead of this
                # switch: that changes when a 48 V motor energises relative to
                # the latch switch, which is an owner decision, not a bug fix.
                print("  NEXT STEP: with the launch down an IDLE hand is "
                      "BACKDRIVABLE and drifts — park it by hand inside the "
                      "retract band, then re-run. --close-loop cannot run until "
                      "this switch succeeds (it is sequenced after it).")
                return False
            deadline = time.time() + 1.0
            want = bool(streamed)
            while time.time() < deadline:
                if _hand_source_streamed() == want:
                    return True
                time.sleep(0.05)
            print("HAND_SOURCE_SET acked but the heartbeat bit never followed — "
                  "treat as failed.")
            return False

        if args.source_only is not None:
            ok = set_source(args.source_only == "streamed")
            print(f"hand_source → {args.source_only.upper()}: {'OK' if ok else 'FAILED'}")
            return 0 if ok else 2

        if args.no_source_switch:
            # T-H4(b): stream against WHATEVER the latch currently reads. Under
            # LEGACY the firmware must discard Setpoint index 6 ([hand7]
            # discard_legacy climbs) and move nothing.
            print(f"hand_source: NOT switching (--no-source-switch) — latch reads "
                  f"{'STREAMED' if _hand_source_streamed() else 'LEGACY'}")
        elif not set_source(True):
            return 2

        # ── Optional hand bring-up (mode + gains BEFORE closing the loop) ────
        if args.close_loop:
            print("hand bring-up: POSITION/PASSTHROUGH + gains → CLOSED_LOOP")
            rpc.call(int(RpcMethod.SET_CONTROLLER_MODE),
                     rpc_args.encode_set_controller_mode(
                         HAND, pc.ODRIVE_CONTROL_MODES['POSITION'],
                         pc.ODRIVE_INPUT_MODES['PASSTHROUGH']))
            rpc.call(int(RpcMethod.SET_POS_GAIN),
                     rpc_args.encode_set_pos_gain(HAND, hw.ODRIVE_HAND_POS_GAIN))
            rpc.call(int(RpcMethod.SET_VEL_GAINS),
                     rpc_args.encode_set_vel_gains(HAND, hw.ODRIVE_HAND_VEL_GAIN,
                                                   hw.ODRIVE_HAND_VEL_INT_GAIN))
            rpc.call(int(RpcMethod.SET_AXIS_STATE),
                     rpc_args.encode_set_axis_state(HAND, CLOSED_LOOP))
            time.sleep(0.5)
            # Verify the bring-up TOOK (mirrors teensy_setpoint_bench.py:326-335).
            # An RPC ack is the BRIDGE acking the CAN send, not the ODrive
            # entering CLOSED_LOOP: an axis with an active error, a missing
            # encoder index or a live undervoltage acks and stays in IDLE — the
            # same silent de-energised state the precondition block above
            # exists for, just reached one step later. The baseline printed at
            # startup is a PRE-bring-up snapshot and is never refreshed, so
            # re-read the cache; the 1→8 transition is itself a diag_changed()
            # trigger (telemetry.cpp:61), so a healthy bring-up resolves in
            # tens of ms and only a REAL failure spends the full poll.
            t_cl = time.time()
            while time.time() - t_cl < BRINGUP_VERIFY_S:
                dcl = _hand_diag()
                if dcl is not None and int(dcl.axis_state) == CLOSED_LOOP:
                    break
                time.sleep(0.05)
            dcl = _hand_diag()
            # Gate B gates on BOTH halves (2026-09-04 audit fix). The
            # `post-bring-up: … ENERGISED and holding` line below is what runbook
            # row 13 now instructs the operator to read as the sitting's POSITIVE
            # evidence, so it must never print with a live error latched — an
            # axis can report CLOSED_LOOP with active_errors set, and that line
            # would then certify exactly the condition it exists to refuse.
            bad_state = dcl is None or int(dcl.axis_state) != CLOSED_LOOP
            bad_err = dcl is not None and int(dcl.active_errors) != 0
            if bad_state or bad_err:
                if dcl is None:
                    why = "no DIAGNOSTIC arrived (axis_state and errors UNKNOWN)"
                else:
                    why = "; ".join(
                        ([f"axis_state={dcl.axis_state}, need CLOSED_LOOP={CLOSED_LOOP}"]
                         if bad_state else [])
                        + ([f"active_errors={dcl.active_errors}, need 0"]
                           if bad_err else []))
                print(f"ABORT: hand bring-up did not take — {why} after "
                      f"{BRINGUP_VERIFY_S:.0f} s (the RPC was acked by the BRIDGE, "
                      f"not by the ODrive: an axis with an active error, a missing "
                      f"encoder index or a live undervoltage acks and stays IDLE). "
                      f"Read the hand ODrive's errors on the console and clear them "
                      f"with --clear-errors before re-running.")
                return 2
            ep = _hand_pos()
            print(f"post-bring-up: axis_state={dcl.axis_state} "
                  f"(CLOSED_LOOP={CLOSED_LOOP})  active_errors={dcl.active_errors}  "
                  f"pos={'?' if ep is None else f'{ep:+.4f}'} rev — the hand is "
                  f"ENERGISED and holding")

        # ── Build the stage trajectory ───────────────────────────────────────
        start = _hand_pos()
        if args.stage == "hold":
            traj = Hold(start)
        elif args.stage == "triangle":
            if start + args.tri_span > HAND_MAX_POS - HAND_MARGIN:
                print(f"ABORT: triangle would reach "
                      f"{start + args.tri_span:.2f} rev (> "
                      f"{HAND_MAX_POS - HAND_MARGIN:.2f}). Park lower."); return 2
            traj = Triangle(start, args.tri_span, args.tri_speed)
        elif args.stage == "stroke":
            # Park gate = the settle band (2026-09-03 audit fix): the stroke's
            # first frame commands 0.0 rev absolute, NOT the live encoder, so
            # the gate is what bounds the arm-edge offset. Any residual inside
            # it is walked by the firmware recovery slew at <= 1 rev/s.
            if abs(start) > HAND_SETTLE_BAND_REV:
                print(f"ABORT: stroke stage needs the hand parked within the "
                      f"settle band ±{HAND_SETTLE_BAND_REV:.2f} rev of 0 "
                      f"(kind-0 basis); it is at {start:+.3f}."); return 2
            traj = Stroke(args.event_vel)
        else:                               # step / gap ride a hold
            traj = Hold(start)

        # ── The pump (production-in-the-loop) + the pinned leg hold ──────────
        pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV, num_legs=p.NUM_LEGS,
                            max_step_hand_rev=(float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
                                               * float(hw.JB_TRAJ_KNOT_DT_S)))
        tm = _telem()
        leg_rev = [float(tm.pos_rev[i]) for i in range(6)]
        mmrev = [float(x) for x in hw.GEOM_MM_TO_REV]
        leg_ext = [leg_rev[i] / mmrev[i] for i in range(6)]

        def frame(t, with_hand=True, hand_override=None):
            hp, hv = traj.sample(t)
            hp1, hv1 = traj.sample(t + SEG_T)
            hp2, _ = traj.sample(t + 2 * SEG_T)
            if hand_override is not None:
                hp = hp1 = hp2 = hand_override
                hv = hv1 = 0.0
            cmd = {'type': 'mpc_cmd', 'seq': 0,
                   'ext_mm': leg_ext, 'pose_6dof': [0.0, 0.0, 170.0, 0.0, 0.0, 0.0],
                   'motor_rev': leg_rev, 'vel_mm_s': [0.0] * 6,
                   'cmd_next_mm': leg_ext, 'cmd_next2_mm': leg_ext,
                   'vel_next_mm_s': [0.0] * 6,
                   'torque_Nm': [0.0] * 6, 'acc_mm_s2': [0.0] * 6}
            if with_hand:
                cmd.update({'hand_rev': hp, 'hand_vel_rps': hv,
                            'hand_next_rev': hp1, 'hand_next2_rev': hp2,
                            'hand_next_vel_rps': hv1})
            return cmd

        # ── Continuous 40 Hz hold stream from BEFORE the arm to the loop ─────
        # 2026-09-02 review fix: the interactive path used to stop streaming at
        # input(), so by the time mpc_active rose the last setpoint was >250 ms
        # old and — with ever_cmd already set by the pre-stream — the firmware
        # latched MPC_STALE at the arm edge, before the first loop frame. The
        # hold stream now runs in a BACKGROUND thread through the prompt AND
        # the arm-verify window (the production stream-then-arm contract), and
        # is joined just before the main loop takes the stream over — the pump
        # is never built from two threads at once.
        period = 1.0 / SETPOINT_HZ
        hold_stop = threading.Event()
        hold_state = {'err': None, 'sent': 0}

        def _hold_pump():
            nxt = time.monotonic()
            while not hold_stop.is_set():
                sph, whyh = pump.build(frame(0.0), t_origin_us=int(time.time() * 1e6))
                if sph is None:
                    hold_state['err'] = whyh
                    return
                client.send_stream(int(MsgType.SETPOINT), sph.pack())
                hold_state['sent'] += 1
                nxt += period
                lag = nxt - time.monotonic()
                if lag > 0:
                    time.sleep(lag)
                else:
                    nxt = time.monotonic()

        hold_thread = threading.Thread(target=_hold_pump, daemon=True)
        hold_thread.start()
        time.sleep(0.5)                     # ≥ 0.5 s of stream before arming
        if hold_state['err'] is not None:
            print(f"ABORT: pre-arm frame refused by the pump: {hold_state['err']}")
            hold_stop.set(); hold_thread.join(timeout=1.0); return 2

        if not args.arm:
            ans = input(f"ARM stage '{args.stage}' for {args.duration:.0f}s? "
                        f"E-stop in hand. [yes/NO] ")   # hold stream keeps running
            if ans.strip().lower() != "yes":
                hold_stop.set(); hold_thread.join(timeout=1.0)
                print("not armed; exiting."); return 0
        if hold_state['err'] is not None:
            print(f"ABORT: hold frame refused by the pump: {hold_state['err']}")
            hold_stop.set(); hold_thread.join(timeout=1.0); return 2
        client.set_heartbeat_flags(1)       # mpc_active=1 (stream still flowing)
        t_arm = time.time()
        while time.time() - t_arm < ARM_VERIFY_GRACE_S:
            hb = _hb()
            if hb is not None and (int(hb.flags) & _T2J_FLAG_MPC_ACTIVE):
                break
            time.sleep(0.05)
        else:
            print("ABORT: firmware never reported mpc_active — arm did not take.")
            client.set_heartbeat_flags(0)
            hold_stop.set(); hold_thread.join(timeout=1.0); return 2
        armed = True
        hold_stop.set(); hold_thread.join(timeout=1.0)   # main loop takes over NOW
        print(f"ARMED — streaming (hold thread sent {hold_state['sent']} frames).")

        csv_path = args.csv or os.path.join(
            "/home/jetson/Desktop/Jugglebot/temp/logs",
            f"hand_stream_{args.stage}_"
            f"{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        csv_f = open(csv_path, "w", newline="")
        w = csv.writer(csv_f)
        # enc_age is the telemetry receive age the deviation belt extrapolates
        # the cached encoder forward by, and WITHOUT IT THE BELT IS NOT
        # RECONSTRUCTIBLE FROM THIS CSV. |cmd − (enc + vel·age)| carries a
        # vel·age term worth ~0.7 rev at the stroke's ~95 rev/s plateau against
        # a 1.5 rev bar, so an analyst re-deriving a run's deviation has to
        # SOLVE for age rather than read it — which is precisely what blocked
        # attributing the 0.5387 → 1.2348 rev stroke-deviation growth between
        # two runs of one stage in the 2026-09-04 sitting forensics.
        w.writerow(["t", "cmd_rev", "enc_rev", "vel_rps", "echo_rev",
                    "fault", "lead_mask", "recon_mm", "axis_state", "enc_age"])
        # CacheDiag companion CSV — the T-H1 drop-episode instrument (1 Hz
        # windowed enc_frames deficits, the headroom runbook row-21 recipe).
        cd_path = csv_path[:-4] + "_cachediag.csv" if csv_path.endswith(".csv") \
            else csv_path + "_cachediag.csv"
        cd_f = open(cd_path, "w", newline="")
        wcd = csv.writer(cd_f)
        wcd.writerow(["t", "window_us", "samples"]
                     + [f"deficit_{i}" for i in range(7)]
                     + ["rx_depth_hwm_jb", "rx_cap_hits_jb", "t_local_us"])
        cd_stats = {'windows': 0, 'episodes': 0, 'worst': 0.0}

        def _drain_cd():
            with _lock:
                rows = _cache["cd_rows"]
                _cache["cd_rows"] = []
            for (mono, win_us, samples, deficits, hwm, cap, t_local) in rows:
                cd_stats['windows'] += 1
                seen = [x for x in deficits if x is not None]
                if seen:
                    worst = min(seen)
                    cd_stats['worst'] = min(cd_stats['worst'], worst)
                    if worst < CD_EPISODE_DEFICIT:
                        cd_stats['episodes'] += 1
                wcd.writerow([f"{mono - t_start_mono:.3f}", win_us, samples]
                             + ["" if x is None else f"{x:.1f}" for x in deficits]
                             + [hwm, cap, t_local])

        # ── The streamed run ─────────────────────────────────────────────────
        t_start = time.time()               # wall — echoes time-align against this
        t_start_mono = time.monotonic()
        next_t = t_start
        max_dev = 0.0
        max_recon_mm = 0.0
        last_echo_key = None                # dedupe: one recon sample per echo frame
        echo_unsynced = 0                   # echoes skipped: bridge wall not TIME_SYNCED
        step_done = False
        pump_rejects_at_step = None
        gap_reentry_t = None                # gap: stage time the re-entry was first commanded
        gap_echo_ref = None                 # ... and the echo value it must move away from
        gap_echo_moved = False              # ... proven to have reached the wire
        while True:
            t = time.time() - t_start
            if t > args.duration:
                break

            with_hand = True
            hand_override = None
            if args.stage == "gap":
                if args.gap_pre <= t < args.gap_pre + args.gap_s:
                    with_hand = False               # the falling edge
                elif t >= args.gap_pre + args.gap_s:
                    # Clamp BOTH ends (2026-09-03 audit fix): --gap-delta may be
                    # negative (±1.5 accepted), and a hand parked near 0 would
                    # otherwise command below the firmware's [0, 10.8] clip.
                    hand_override = min(max(start + args.gap_delta, 0.0),
                                        HAND_MAX_POS - HAND_MARGIN)
                    # Arm the re-entry proof on the FIRST re-entry tick: snapshot
                    # the echo the re-entry must move away from. Skipped for a
                    # re-entry too small to be distinguishable from a static echo
                    # (see GAP_ECHO_MOVE_REV's derivation).
                    if (gap_reentry_t is None
                            and abs(args.gap_delta) > 4.0 * GAP_ECHO_MOVE_REV):
                        with _lock:
                            e0 = _cache["echo"]
                        gap_reentry_t = t
                        gap_echo_ref = None if e0 is None else e0[1]
            if args.stage == "step" and not step_done and t >= 3.0:
                # ONE deliberately-oversized knot: the pump MUST refuse it.
                bad = frame(t, hand_override=None)
                bad['hand_rev'] = start + args.step_rev
                bad['hand_next_rev'] = start + args.step_rev
                bad['hand_next2_rev'] = start + args.step_rev
                bad['hand_next_vel_rps'] = 0.0
                before = pump.frames_rejected
                spx, whyx = pump.build(bad, t_origin_us=int(time.time() * 1e6))
                step_done = True
                pump_rejects_at_step = (spx is None, whyx,
                                        pump.frames_rejected - before)
                print(f"step probe: pump refused={spx is None} reason={whyx!r}")
                # nothing sent for this knot — the hold continues below.

            # hand_override MUST reach frame() (2026-09-04 fix). ROOT CAUSE: it
            # was computed above and consumed below — the CSV's cmd_rev column
            # and the deviation belt both read it — but was never passed HERE, so
            # the gap stage streamed a constant Hold while the CSV logged a
            # re-entry that never happened and the belt compared against that
            # phantom. It failed SILENTLY because the phantom's deviation is
            # |gap_delta| and the gap belt is |gap_delta| + 0.5: the fiction sits
            # under the stage's own bar. Measured cost, 2026-09-04: echo frozen
            # bit-identical at 0.00018 rev for all 1200 rows and 0.017 mm of
            # encoder excursion, against a logged 1.0 rev step.
            sp, why = pump.build(frame(t, with_hand=with_hand,
                                       hand_override=hand_override),
                                 t_origin_us=int(time.time() * 1e6))
            if sp is None:
                print(f"ABORT: pump refused a stage frame: {why}"); break
            client.send_stream(int(MsgType.SETPOINT), sp.pack())

            # belts + logging (10 Hz worth of work at 40 Hz cost is fine)
            tm_snap, enc_age = _telem_with_age()
            enc = None if tm_snap is None else float(tm_snap.pos_rev[HAND])
            encv = 0.0 if tm_snap is None else float(tm_snap.vel_rps[HAND])
            cmd_now = traj.sample(t)[0] if hand_override is None else hand_override
            if enc is not None and with_hand:
                # Velocity-compensated belt (2026-09-02 review fix), mirroring
                # the firmware residual: extrapolate the cached encoder forward
                # by its receive age before comparing. A static |cmd − enc|
                # against a 10-45 ms-stale cache reads ~0.95-4.3 rev of pure
                # latency at the stroke's ~95 rev/s plateau — the exact
                # static-bound arithmetic Phase 0 Decision 4 rejected for the
                # firmware guard — and would abort a HEALTHY stroke (a
                # commanded stop into a fast hand).
                enc_ex = enc + encv * enc_age
                dev = abs(cmd_now - enc_ex)
                max_dev = max(max_dev, dev)
                if dev > args.max_dev:
                    print(f"ABORT: driver deviation belt |{cmd_now:.3f} - "
                          f"({enc:.3f} + {encv:.1f}·{enc_age * 1e3:.0f}ms)| "
                          f"= {dev:.3f} rev > {args.max_dev}"); break
            fs = _fault()
            if fs not in (None, int(FaultState.NONE)):
                print(f"ABORT: firmware fault {_fault_name(fs)} — disarming.")
                # Trip trio (2026-09-03 audit fix): with the launch DOWN the
                # HeartbeatT2J latch snapshot is the ONLY surface that carries
                # an armed MAX_DEVIATION trip's own exceed-tick residuals —
                # the [hand7] dev_cmd/dev_fb pair is the BOOT-CUMULATIVE worst
                # tick, which can predate the trip by a whole observe block.
                # The latch persists until CLEAR_ERRORS, so the cached
                # heartbeat cannot lose it; leg 0xFF = no MAX_DEVIATION latch
                # (some other fault tripped).
                hb_snap = _hb()
                if hb_snap is not None:
                    print(f"  max_dev latch: leg={int(hb_snap.max_dev_leg)} "
                          f"dev={float(hb_snap.max_dev_value):+.4f} rev  "
                          f"u0={float(hb_snap.max_dev_u0):+.4f}  "
                          f"enc={float(hb_snap.max_dev_enc):+.4f}  "
                          f"(leg 255 = not a MAX_DEVIATION trip)")
                break
            # Mid-stage energisation check (2026-09-04 audit fix): the
            # precondition block and the --close-loop verification are both
            # ENTRY-only, and nothing re-read axis_state after them — not across
            # the UNBOUNDED interactive ARM prompt, not during the run. A hand
            # that drops to IDLE mid-stage makes a `hold` produce zero deviation,
            # so the belt never fires and T-H1 passes on a dead axis again, one
            # step later. Cached read; a state change is itself a diag_changed()
            # trigger, so the verdict lands within a refresh.
            dlive = _hand_diag()
            if dlive is not None and int(dlive.axis_state) != CLOSED_LOOP:
                print(f"ABORT: hand left CLOSED_LOOP mid-stage — axis_state={dlive.axis_state}, "
                      f"active_errors={dlive.active_errors}; every reading after this point is a "
                      f"de-energised axis (the 2026-09-03 class, one step later)"); break
            hb = _hb()
            with _lock:
                echo = _cache["echo"]
            # Did the gap re-entry actually reach the wire? (2026-09-04 fix, the
            # POSITIVE half.) A dropped hand_override is invisible to every other
            # belt in this loop — the deviation belt compares the phantom against
            # itself — so the one surface that can tell a tracked re-entry from an
            # unsent one is the firmware's own echo. See GAP_ECHO_MOVE_REV.
            if gap_reentry_t is not None and not gap_echo_moved:
                cur = None if echo is None else echo[1]
                if (cur is not None and gap_echo_ref is not None
                        and abs(cur - gap_echo_ref) > GAP_ECHO_MOVE_REV):
                    gap_echo_moved = True
                elif t > gap_reentry_t + GAP_ECHO_DEADLINE_S:
                    print(f"ABORT: the gap re-entry never reached the wire — the hand "
                          f"echo has not moved {GAP_ECHO_DEADLINE_S:.2f} s after the "
                          f"re-entry knot. Observed echo "
                          f"{'none' if cur is None else f'{cur:+.5f} rev'} vs "
                          f"{'none' if gap_echo_ref is None else f'{gap_echo_ref:+.5f} rev'}"
                          f" pre-re-entry; expected > {GAP_ECHO_MOVE_REV:.3f} rev of "
                          f"movement for a commanded {args.gap_delta:+.3f} rev step. "
                          f"Under STREAMED the echo re-sources from "
                          f"axes[6].target_pos_rev and is emitted only when the interp "
                          f"TXed, so a re-entry that reached the firmware MUST move it: "
                          f"this stage has measured NOTHING and its CSV logs a re-entry "
                          f"that never happened. REMEDY: read [hand7] — src=STREAMED "
                          f"lane=active with sent= climbing means the knot reached the "
                          f"firmware and the driver dropped hand_override on the way to "
                          f"frame() again (the 2026-09-04 defect); src=LEGACY with "
                          f"discard_legacy climbing means the latch, not the driver "
                          f"(--source-only streamed first)."); break
            recon = None
            if args.stage == "stroke" and echo is not None and echo[0] != last_echo_key:
                # Reconstruction error, TIME-ALIGNED (2026-09-02 review fix):
                # evaluate the analytic AT the echo's t_bridge_us — the bridge
                # wall clock this driver's own TimeOfDayServer disciplines —
                # instead of at NOW. The old NOW-comparison smeared 30-100 mm
                # of pure echo latency over the 3.25 mm bar at stroke speed.
                # Residual noise ≈ 3 mm per ms of wall-sync error at the
                # ~95 rev/s plateau — judge the trend + [hand7] dev_max
                # together. Only TIME_SYNCED echoes are scored.
                last_echo_key = echo[0]
                if hb is not None and (int(hb.flags) & _T2J_FLAG_TIME_SYNCED):
                    t_echo = echo[0] / 1e6 - t_start
                    cmd_at_echo = traj.sample(t_echo)[0]
                    recon = abs(echo[1] - cmd_at_echo) * MM_PER_REV
                    max_recon_mm = max(max_recon_mm, recon)
                else:
                    echo_unsynced += 1
            w.writerow([f"{t:.3f}", f"{cmd_now:.5f}",
                        "" if enc is None else f"{enc:.5f}",
                        "" if tm_snap is None else f"{encv:.3f}",
                        "" if echo is None else f"{echo[1]:.5f}",
                        "" if fs is None else fs,
                        "" if hb is None else int(hb.lead_clamp_mask),
                        "" if recon is None else f"{recon:.2f}",
                        # live cached axis_state — empty until a DIAGNOSTIC has
                        # arrived, same convention as echo_rev / recon_mm. The
                        # energisation record is then auditable after the fact.
                        "" if dlive is None else int(dlive.axis_state),
                        # the belt's own age term (see the header comment).
                        # Empty on the same convention when no telemetry is
                        # cached — _telem_with_age() returns 0.0 there, which
                        # would otherwise log as a REAL zero-age reading.
                        # µs resolution: at 95 rev/s that is 1e-4 rev, below
                        # the 5th decimal enc_rev is logged to, so the belt
                        # re-derives exactly from this row.
                        "" if tm_snap is None else f"{enc_age:.6f}"])
            _drain_cd()                     # 1 Hz CacheDiag rows → companion CSV

            next_t += period
            lag = next_t - time.time()
            if lag > 0:
                time.sleep(lag)

        # ── Report ───────────────────────────────────────────────────────────
        _drain_cd()
        print(f"stage '{args.stage}' done: max |cmd−(enc+vel·age)| = {max_dev:.4f} rev "
              f"({max_dev * MM_PER_REV:.2f} mm)  [belt {args.max_dev} rev]")
        if args.stage == "stroke":
            print(f"  worst TIME-ALIGNED echo-vs-analytic reconstruction: "
                  f"{max_recon_mm:.2f} mm (bar: 3.25 mm + wall-sync noise only, "
                  f"~3 mm/ms at the plateau — the echo-age term is corrected at "
                  f"source since the 2026-09-03 audit fix (firmware stamps the "
                  f"interp tick that wrote the bytes, not the emit); "
                  f"{echo_unsynced} echoes skipped unsynced — judge the trend "
                  f"+ [hand7] dev_max together)")
        if args.stage == "step":
            print(f"  step probe result: {pump_rejects_at_step}")
        print(f"  cachediag: {cd_stats['windows']} windows, "
              f"{cd_stats['episodes']} episode(s) past {CD_EPISODE_DEFICIT:.0f}, "
              f"worst deficit {cd_stats['worst']:.1f} frames "
              f"(row-21 recipe; clean baseline −3.6 ± 10.3)")
        hb = _hb()
        if hb is not None and int(hb.lead_clamp_mask) & 0x40:
            print("  ⚠ hand lead clamp engaged on the LAST tick — read the [hand7] "
                  "lead= counter; non-zero duty during a throw is a hard abort.")
        print(f"csv: {csv_path}")
        print(f"cachediag csv: {cd_path}")
        return 0
    finally:
        if armed:
            client.set_heartbeat_flags(0)   # disarm — firmware gates output off
            time.sleep(0.2)
        if csv_f:
            csv_f.close()
        if cd_f:
            cd_f.close()
        rpc.close()
        client.stop()


if __name__ == "__main__":
    raise SystemExit(main())
