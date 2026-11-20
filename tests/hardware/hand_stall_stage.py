"""Pure-python half of ``hand_stream_bench.py --stage stall`` (BUG-123, can-bridge FW 47).

The stage proves the FW 47 Hand stall guard on the BENCH Hand: outside the ACTIVATE /
HAND_MOVE_TO ladder, |iq| >= 40 A with |vel| <= 1 rev/s for 150 ms of real samples commands
the Hand IDLE and latches fault HAND_STALL (8) with Diagnostic flags bit3
(``leg_activate.cpp`` ``hand_lane_stall_guard``, INVARIANTS I-FW-17). Everything here is
link-free so ``tests/motion/test_hand_stream_bench_stall.py`` drives it with a fake; the
link adapter is ``_StallBenchIO`` in ``hand_stream_bench.py``.

WHAT THE HAND IS PUSHED INTO. The bottom stop: the same metal stop the bench ``home``
(``bench_hand_frame_bench.py``) drives into at 3 rev/s and detects at an Iq EMA of 8 A
(cap 11 A), which then reads -0.10 rev. The stage lowers the drive's current limit to 45 A
(``--stall-current-limit``, 41..50) and pushes the Hand into that stop until the guard
cuts it: about 5x the homing push, for the guard's 150 ms. That is the point of the test
and it is said plainly: 45 A against the bottom stop for about 0.2 s, and for at most
``CLAMP_BOUND_S`` (0.715 s) if the guard fails. The same robot mechanism took 68-70 A
against its TOP stop for 4.4 s on 2026-11-20; the damage was thermal (88 C), not to the
stop. The 0.2-0.7 s here heat the motor by a few degrees at most.
Why 45 A: the guard needs EVERY 9 ms sample >= 40 A for 150 ms, and one sample below 40
restarts the window. The ODrive clamps the Iq SETPOINT at current_soft_max; the measured
Iq ripples around it by about 1-2 A at stall. 45 A leaves a 5 A margin over that ripple;
42 A would let a single low sample restart the window, 50 A buys nothing. The motor
thermistor derates the current from 60 C (``temp_limit_lower``), so the stage refuses a
motor at or above ``TEMP_MAX_C`` (50 C).

WHY THE FRAME IS SHIFTED. The streamed lane clips every Hand command to
[0, HAND_MOTOR_MAX_POSITION] (``leg_interp.cpp``, "Stroke clip"), and after homing the
bottom stop reads -0.10 rev: the lane can never command the Hand INTO the bottom stop,
so a ramp to ``--stall-target`` -0.6 rev would be clipped to 0.0 and stop 0.1 rev above
the metal. The stage therefore re-references the encoder by ``shift = RAMP_END_REV -
target`` (0.65 rev at the default) while the Hand is IDLE, with SET_ABSOLUTE_POSITION
(the RPC a home ends with), so the ramp ends at +0.05 in the lane's frame, which is the
homed -0.60: 0.5 rev past the stop. This is the 2026-11-20 failure on purpose (a Hand whose
reference is wrong, stalled by the lane), on the stop gravity already loads. The shift is
undone in ``finally`` (measured position minus the measured shift, Hand IDLE), checked by
reading the position back, and the sheet re-homes after every run anyway.

WHY THE LANE GUARD, AND NOTHING ELSE, SHOULD ACT.
- The ladder guard (25 A / 0.5 rev/s / 300 ms, ``hand_stall_guard``) runs only while a
  ladder targets the Hand (``s_targets & HB``). This stage never runs a ladder: the Hand
  is brought up with the raw SET_AXIS_STATE, exactly as ``--close-loop`` does, so the
  ladder guard is never armed and the lane guard is armed from the first sample
  (``hand_lane_stall_guard`` returns early only while a ladder targets the Hand).
- MAX_DEVIATION_HAND_REV 2.5 (E-STOP) and MAX_LEAD_HAND_REV 2.0 (silent clamp),
  ``canbridge_config.h``: the command ends 0.5 rev past the stop, well inside both.
- At the clamp the position loop holds |vel| ~ 0, so the lane guard's predicate holds.
  With the hand gains (pos 35, vel 0.007, vel_int 0.07, torque_constant 0.0055) the
  integrator reaches 45 A about 0.35 s after contact at 1 rev/s, before the ramp ends.

WHAT HAPPENS TO THE STREAM AFTER THE TRIP. The latch raises guard ESTOP and gates the
output, and the stream only ever sends Set_Input_Pos, which cannot change an ODrive axis
state. The stage still stops streaming and disarms on the first sign of the trip, so a
host bug can never fight the latch. It does NOT clear the fault: the operator runs
``--clear-errors`` and re-homes.

THE INDEPENDENT ABORT, and the bound on time at the clamp. If no trip is seen
``ABORT_AFTER_40_S`` after the first sample at >= 40 A, the stage stops streaming and
commands the Hand IDLE itself (SET_AXIS_STATE IDLE, ``IDLE_ATTEMPTS`` tries of
``IDLE_RPC_TIMEOUT_S``). The time the motor can sit at >= 40 A on that path is
  OBS_LAG_S (Get_Iq 9 ms + Diagnostic tick 10 ms + UDP)      0.020
+ ABORT_AFTER_40_S                                            0.600
+ POLL_S (this loop)                                          0.005
+ IDLE_ATTEMPTS x IDLE_RPC_TIMEOUT_S                          0.080
+ JITTER_S (a loaded Jetson)                                  0.010
= CLAMP_BOUND_S                                               0.715 s  (< 0.75, asserted)
The Diagnostic is on-change (Iq SETPOINT moves > 0.5 A) or 1 Hz, so the stage keys "first
40 A" on max(|iq_measured|, |iq_setpoint|): the setpoint crosses first, so the clock
starts early, never late. If the current never reaches 40 A, ``NO_CLAMP_S`` (3 s) after
the Hand reaches the stop (or the ramp ends) the same IDLE fires: the clamp or the wind-up
did not get there, and the motor was below 40 A throughout. Stale telemetry
(``TELEM_STALE_S``) aborts the same way. Ctrl-C and any exception reach the ``finally``,
which IDLEs an energised Hand BEFORE it raises the current limit back.

THE CURRENT LIMIT. Lowered and restored with SET_VEL_CURR_LIMITS over the bridge (the RPC
homing uses; the ODrive maps it to vel_limit + current_soft_max). The bridge cannot read
the limit back (no Set_Limits reply, and SDO reads of axis 6 are refused), so the stage
READS it over the drive's USB link, which the bench home already uses: before (the value
it restores, expected 70 A), after lowering (must read the test value, or the stage stops
before energising) and after restoring (must read the original). The restore runs in
``finally`` on every path, Hand IDLE first. The restore RPC also resets the bridge's cache
of the operator's limit, which the next bridge home pushes back.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

# ── Mirrored firmware / config constants (pinned by the stall test file) ──────
STALL_FW_MIN = 47                 # canbridge_config.h FW_VERSION that added the guard
GUARD_IQ_A = 40.0                 # canbridge_config.h HAND_LANE_STALL_IQ_A
GUARD_VEL_RPS = 1.0               # canbridge_config.h HAND_LANE_STALL_VEL_RPS
GUARD_HOLD_S = 0.150              # canbridge_config.h HAND_LANE_STALL_HOLD_US
HOME_REF_REV = -0.10              # jugglebot_homing.hand_abs_pos_rev (the bottom stop)
FAULT_HAND_STALL = 8              # FaultState.HAND_STALL
FAULT_NONE = 0
DIAG_FLAG_LANE_STALL = 0x8        # teensy_link.activate.DIAG_FLAG_LANE_STALL
AXIS_IDLE = 1
AXIS_CLOSED_LOOP = 8

# ── The stage's own numbers ──────────────────────────────────────────────────
PASS_IDLE_LIMIT_S = 0.200         # IDLE within this of the first 40 A sample
LIMIT_DEFAULT_A = 45.0
LIMIT_MIN_A, LIMIT_MAX_A = 41.0, 50.0
ORIG_LIMIT_MAX_A = 80.0           # the bench drive's current_hard_max
TARGET_DEFAULT_REV = -0.6         # homed frame: 0.5 rev past the -0.10 stop
TARGET_MIN_REV, TARGET_MAX_REV = -1.1, -0.3
SPEED_DEFAULT_RPS = 1.0
SPEED_MAX_RPS = 2.0
RAMP_ACCEL_RPS2 = 4.0
RAMP_END_REV = 0.05               # lane frame: just inside the [0, 10.501] clip
PARK_MIN_REV, PARK_MAX_REV = 0.3, 0.7
TEMP_MAX_C = 50.0                 # the thermistor derates from 60 C
AT_STOP_BAND_REV = 0.05
SHIFT_TOL_REV = 0.05
STILL_VEL_RPS = 0.05

ABORT_AFTER_40_S = 0.600
NO_CLAMP_S = 3.0
TELEM_STALE_S = 0.200
POLL_S = 0.005
SEND_PERIOD_S = 0.025             # the 40 Hz knot cadence
OBS_LAG_S = 0.020
IDLE_RPC_TIMEOUT_S = 0.040
IDLE_ATTEMPTS = 2
JITTER_S = 0.010
CLAMP_BOUND_S = (OBS_LAG_S + ABORT_AFTER_40_S + POLL_S
                 + IDLE_ATTEMPTS * IDLE_RPC_TIMEOUT_S + JITTER_S)
assert CLAMP_BOUND_S <= 0.75, CLAMP_BOUND_S

POST_TRIP_WATCH_S = 1.0
POST_TRIP_IQ_A = 5.0              # "near zero" after the IDLE
IQ_COLLAPSE_A = 5.0               # the push has stopped
PROBE_VEL_RPS = 0.5               # HAND_MOVE_TO cruise for the refusal probe (gentle cap 2.5)


@dataclass
class StallConfig:
    limit_a: float = LIMIT_DEFAULT_A
    target_rev: float = TARGET_DEFAULT_REV
    speed_rps: float = SPEED_DEFAULT_RPS

    @property
    def shift_rev(self) -> float:
        return RAMP_END_REV - self.target_rev

    @property
    def stop_lane_rev(self) -> float:
        """Where the bottom stop reads once the frame is shifted."""
        return HOME_REF_REV + self.shift_rev


def config_refusals(limit_a, target_rev, speed_rps):
    out = []
    if not (LIMIT_MIN_A <= limit_a <= LIMIT_MAX_A):
        out.append(f"--stall-current-limit {limit_a} A outside {LIMIT_MIN_A:.0f}..{LIMIT_MAX_A:.0f} A")
    if not (TARGET_MIN_REV <= target_rev <= TARGET_MAX_REV):
        out.append(f"--stall-target {target_rev} rev outside {TARGET_MIN_REV}..{TARGET_MAX_REV} "
                   f"(homed frame; the stop is at {HOME_REF_REV})")
    if not (0.0 < speed_rps <= SPEED_MAX_RPS):
        out.append(f"--stall-speed {speed_rps} rev/s outside (0, {SPEED_MAX_RPS}]")
    return out


def stall_fw_refusal(fw):
    if fw is None:
        return "no BridgeIdentity: the bridge firmware version is unknown"
    if int(fw) < STALL_FW_MIN:
        return (f"bridge reports firmware {fw}; the Hand stall guard exists from firmware "
                f"{STALL_FW_MIN} (BUG-123). Flash the night-shift/2026-11-20-BUG-123 build first")
    return None


def preflight_refusals(pre, cfg):
    """Every reason not to start, all at once. ``pre`` is the adapter's preflight dict."""
    out = []
    why = stall_fw_refusal(pre.get("fw"))
    if why:
        out.append(why)
    if pre.get("fault") != FAULT_NONE:
        out.append(f"fault_state={pre.get('fault_name')}, need NONE (run --clear-errors)")
    st = pre.get("axis_state")
    if st is None:
        out.append("no Hand Diagnostic: axis_state, errors and temperature unknown (re-run)")
    else:
        if st != AXIS_IDLE:
            out.append(f"axis_state={st}, need IDLE ({AXIS_IDLE}): the stage re-references the "
                       f"encoder, which needs an IDLE Hand, as home leaves it")
        if pre.get("active_errors"):
            out.append(f"active_errors={pre.get('active_errors')}, need 0")
        t = pre.get("temp_motor")
        if t is None or not math.isfinite(t) or t >= TEMP_MAX_C:
            out.append(f"motor temperature {t} C, need below {TEMP_MAX_C:.0f} C (let it cool)")
    pos = pre.get("pos")
    if pos is None or not (PARK_MIN_REV <= pos <= PARK_MAX_REV):
        out.append(f"Hand at {pos} rev, need {PARK_MIN_REV}..{PARK_MAX_REV} (home, then park low)")
    return out


def limits_refusal(vel0, curr0, limit_a):
    if vel0 is None or not math.isfinite(vel0) or vel0 <= 0.0:
        return f"drive vel_limit reads {vel0}, not a positive number"
    if curr0 is None or not math.isfinite(curr0):
        return f"drive current_soft_max reads {curr0}"
    if not (limit_a < curr0 <= ORIG_LIMIT_MAX_A):
        return (f"drive current_soft_max reads {curr0:.1f} A: the test limit {limit_a:.1f} A "
                f"must be below it, and it must not exceed {ORIG_LIMIT_MAX_A:.0f} A")
    return None


class StallRamp:
    """Trapezoidal (accel-limited) ramp p0 -> p1, then a hold at p1. sample(t) -> (pos, vel).

    Position is C1 and velocity is continuous: never a step (CLAUDE.md: profiled moves only).
    """

    def __init__(self, p0, p1, vmax, accel=RAMP_ACCEL_RPS2):
        self.p0, self.p1 = float(p0), float(p1)
        self.dir = 1.0 if p1 >= p0 else -1.0
        d = abs(self.p1 - self.p0)
        self.a = float(accel)
        v = min(float(vmax), math.sqrt(d * self.a))
        self.v = v
        self.ta = v / self.a if v > 0 else 0.0
        da = 0.5 * self.a * self.ta ** 2
        self.tc = (d - 2 * da) / v if v > 0 else 0.0
        self.duration = 2 * self.ta + self.tc

    def sample(self, t):
        t = max(0.0, float(t))
        if t >= self.duration:
            return self.p1, 0.0
        if t < self.ta:
            s, v = 0.5 * self.a * t * t, self.a * t
        elif t < self.ta + self.tc:
            s = 0.5 * self.a * self.ta ** 2 + self.v * (t - self.ta)
            v = self.v
        else:
            r = self.duration - t
            s = abs(self.p1 - self.p0) - 0.5 * self.a * r * r
            v = self.a * r
        return self.p0 + self.dir * s, self.dir * v


@dataclass
class Sample:
    pos: Optional[float] = None
    vel: Optional[float] = None
    telem_age: float = 0.0
    iq_meas: float = 0.0
    iq_set: float = 0.0
    axis_state: Optional[int] = None
    flags: int = 0
    fault: Optional[int] = None
    temp_motor: Optional[float] = None


class StallWatch:
    """Watches the push; feed() returns 'run', 'trip' or 'abort' (reason in .abort)."""

    def __init__(self, stop_lane_rev, bit3_before=False):
        self.stop = float(stop_lane_rev)
        self.bit3_before = bool(bit3_before)
        self.t_first40 = None
        self.t_trip = None
        self.trip_how = None
        self.t_ref = None            # the Hand at the stop, or the ramp done
        self.peak = 0.0
        self.abort = None

    def feed(self, t, s, ramp_done):
        iq = max(abs(s.iq_meas), abs(s.iq_set))
        self.peak = max(self.peak, abs(s.iq_meas))
        if self.t_first40 is None and iq >= GUARD_IQ_A:
            self.t_first40 = t
        how = []
        if s.fault == FAULT_HAND_STALL:
            how.append("fault HAND_STALL")
        if (s.flags & DIAG_FLAG_LANE_STALL) and not self.bit3_before:
            how.append("bit3")
        if s.axis_state == AXIS_IDLE:
            how.append("axis IDLE")
        if self.t_first40 is not None and iq < IQ_COLLAPSE_A:
            how.append("iq collapsed")
        if how:
            self.t_trip, self.trip_how = t, ", ".join(how)
            return "trip"
        if self.t_ref is None and (ramp_done or (s.pos is not None
                                                 and s.pos <= self.stop + AT_STOP_BAND_REV)):
            self.t_ref = t
        if s.pos is None or s.telem_age > TELEM_STALE_S:
            self.abort = f"telemetry stale ({s.telem_age * 1e3:.0f} ms)"
        elif self.t_first40 is not None and t - self.t_first40 >= ABORT_AFTER_40_S:
            self.abort = (f"no trip {ABORT_AFTER_40_S * 1e3:.0f} ms after the first "
                          f"{GUARD_IQ_A:.0f} A sample")
        elif self.t_first40 is None and self.t_ref is not None and t - self.t_ref >= NO_CLAMP_S:
            self.abort = (f"current never reached {GUARD_IQ_A:.0f} A within {NO_CLAMP_S:.0f} s "
                          f"of the stop (peak {self.peak:.1f} A): the clamp or the wind-up "
                          f"did not get there")
        return "abort" if self.abort else "run"


@dataclass
class StallResult:
    t_first40: Optional[float] = None
    t_trip: Optional[float] = None
    trip_how: Optional[str] = None
    peak_a: float = 0.0
    fault_end: Optional[int] = None
    fault_name: str = "?"
    bit3: bool = False
    bit3_before: bool = False
    second_rise: Optional[str] = None
    probe: Optional[str] = None
    abort: Optional[str] = None
    limit_orig: Optional[float] = None
    limit_readback: Optional[float] = None
    limit_ok: bool = False
    frame_ok: Optional[bool] = None      # None = never shifted
    temp_before: Optional[float] = None
    temp_after: Optional[float] = None
    fails: list = field(default_factory=list)

    @property
    def idle_ms(self):
        if self.t_first40 is None or self.t_trip is None:
            return None
        return (self.t_trip - self.t_first40) * 1e3


def stall_verdict(r: StallResult):
    """(passed, the one verdict line)."""
    reasons = list(r.fails)
    if r.abort:
        reasons.append(r.abort)
    ms = r.idle_ms
    if ms is None:
        reasons.append("no 40 A sample and trip pair")
    elif ms > PASS_IDLE_LIMIT_S * 1e3:
        reasons.append(f"IDLE {ms:.0f} ms after the first 40 A sample, limit "
                       f"{PASS_IDLE_LIMIT_S * 1e3:.0f}")
    if r.fault_end != FAULT_HAND_STALL:
        reasons.append(f"fault {r.fault_name}, need HAND_STALL")
    if not r.bit3:
        reasons.append("Diagnostic bit3 clear")
    if r.second_rise:
        reasons.append(r.second_rise)
    if r.probe is not None and not r.probe.startswith("refused"):
        reasons.append(f"HAND_MOVE_TO while latched: {r.probe}")
    if not r.limit_ok:
        reasons.append(f"current limit read back {r.limit_readback}, need {r.limit_orig}")
    if r.frame_ok is False:
        reasons.append("encoder frame not restored: re-home before anything else")
    bit3 = "bit3 set" + (" (already set before this run)" if r.bit3_before else "")
    if not reasons:
        return True, (f"STALL-GUARD PASS: IDLE {ms:.0f} ms after the first 40 A sample "
                      f"(limit {PASS_IDLE_LIMIT_S * 1e3:.0f}), peak {r.peak_a:.1f} A, "
                      f"fault HAND_STALL, {bit3}")
    tail = (f"IDLE {'n/a' if ms is None else f'{ms:.0f} ms'} after the first 40 A sample, "
            f"peak {r.peak_a:.1f} A, fault {r.fault_name}, "
            f"{bit3 if r.bit3 else 'bit3 clear'}")
    return False, "STALL-GUARD FAIL: " + "; ".join(reasons) + f" ({tail})"


def command_idle(io, say):
    """SET_AXIS_STATE IDLE, IDLE_ATTEMPTS tries. True when one was acked."""
    for i in range(IDLE_ATTEMPTS):
        if io.idle_once(IDLE_RPC_TIMEOUT_S):
            return True
        say(f"  IDLE attempt {i + 1} not acked")
    say("!! IDLE NOT ACKED: PRESS THE E-STOP BUTTON NOW")
    return False


def _wait(io, cond, timeout_s):
    t0 = io.clock()
    while io.clock() - t0 < timeout_s:
        if cond():
            return True
        io.sleep(0.02)
    return cond()


def run_stall(io, cfg: StallConfig, say=print):
    """The stage. Returns 0 on PASS, 1 on FAIL, 2 when refused before anything was commanded."""
    pre = io.preflight()
    refusals = preflight_refusals(pre, cfg)
    if refusals:
        say("ABORT: stall stage refused (nothing was commanded):")
        for x in refusals:
            say(f"  - {x}")
        return 2
    try:
        vel0, curr0 = io.read_limits()
    except Exception as e:  # noqa: BLE001
        say(f"ABORT: cannot read the Hand drive's limits over USB ({e!r}); the restore could "
            f"not be verified, so nothing was commanded")
        return 2
    why = limits_refusal(vel0, curr0, cfg.limit_a)
    if why:
        say(f"ABORT: {why} (nothing was commanded)")
        return 2
    r = StallResult(limit_orig=curr0, temp_before=pre.get("temp_motor"),
                    bit3_before=bool(pre.get("flags", 0) & DIAG_FLAG_LANE_STALL))
    say(f"stall plan: firmware {pre['fw']}, Hand IDLE at {pre['pos']:+.3f} rev, motor "
        f"{pre['temp_motor']:.1f} C; limit {curr0:.1f} -> {cfg.limit_a:.1f} A "
        f"(vel_limit {vel0:.0f} kept); frame shift {cfg.shift_rev:+.3f} rev; ramp at "
        f"{cfg.speed_rps:.2f} rev/s to homed {cfg.target_rev:+.2f} rev, "
        f"{-cfg.target_rev + HOME_REF_REV:.2f} rev past the stop")
    if r.bit3_before:
        say("note: Diagnostic bit3 is already set (sticky from an earlier trip; only a reboot "
            "or a Hand ladder clears it), so this run times the trip on the current, the "
            "fault and the axis state")
    if not io.confirm(f"ARM stall: push the Hand into the bottom stop at {cfg.limit_a:.0f} A "
                      f"until the guard IDLEs it. Cup empty, E-stop in hand. [yes/NO] "):
        say("not armed; nothing was commanded.")
        return 2
    st = {"lowered": False, "shift": None, "energised": False}
    try:
        _body(io, cfg, say, pre, vel0, r, st)
    except BaseException as e:  # noqa: BLE001 (Ctrl-C included)
        r.abort = r.abort or f"interrupted: {e!r}"
        raise
    finally:
        _finish(io, cfg, say, vel0, curr0, r, st)
    return 0 if stall_verdict(r)[0] else 1


def _body(io, cfg, say, pre, vel0, r, st):
    st["lowered"] = True       # before the call: a lost reply may still have reached the wire
    io.set_limits(vel0, cfg.limit_a)
    rb = io.read_limits()[1]
    say(f"current limit lowered: drive reads {rb:.1f} A over USB")
    if abs(rb - cfg.limit_a) > 0.5:
        r.fails.append(f"the lowered limit reads {rb:.1f} A, not {cfg.limit_a:.1f}: "
                       f"stopped before energising")
        return
    p_before = io.pos()
    io.set_abs_pos(p_before + cfg.shift_rev)
    st["shift"] = 0.0          # restore by the MEASURED shift, whatever it turns out to be
    _wait(io, lambda: io.pos() is not None
          and abs(io.pos() - (p_before + cfg.shift_rev)) < SHIFT_TOL_REV, 0.5)
    applied = io.pos() - p_before
    st["shift"] = applied
    if abs(applied - cfg.shift_rev) > SHIFT_TOL_REV:
        r.fails.append(f"frame shift read {applied:+.3f} rev, wanted {cfg.shift_rev:+.3f}: "
                       f"stopped before energising")
        return
    say(f"frame shifted {applied:+.3f} rev: the stop now reads {cfg.stop_lane_rev:+.3f}")
    st["energised"] = True
    ok, why = io.bring_up()
    if not ok:
        r.fails.append(f"bring-up: {why}")
        return
    p0 = io.pos()
    ok, why = io.arm(p0)
    if not ok:
        r.fails.append(f"arm: {why}")
        return
    ramp = StallRamp(p0, RAMP_END_REV, cfg.speed_rps)
    watch = StallWatch(cfg.stop_lane_rev, bit3_before=r.bit3_before)
    say(f"ARMED: ramp {p0:+.3f} -> {RAMP_END_REV:+.3f} rev (lane frame) over "
        f"{ramp.duration:.2f} s")
    t0 = io.clock()
    next_send = t0
    while True:
        now = io.clock()
        t = now - t0
        if now >= next_send:
            io.send(ramp, t)
            next_send += SEND_PERIOD_S
        ev = watch.feed(t, io.sample(), t >= ramp.duration)
        if ev == "trip":
            io.disarm()                    # stop streaming: nothing may fight the latch
            break
        if ev == "abort":
            command_idle(io, say)
            io.disarm()
            break
        io.sleep(POLL_S)
    r.t_first40, r.t_trip, r.trip_how = watch.t_first40, watch.t_trip, watch.trip_how
    r.peak_a, r.abort = watch.peak, watch.abort
    if watch.abort:
        say(f"ABORT: {watch.abort}; the stage commanded IDLE itself")
        return
    say(f"trip seen at t={watch.t_trip:.3f} s ({watch.trip_how}); first 40 A at "
        f"{'n/a' if watch.t_first40 is None else f'{watch.t_first40:.3f} s'}")
    # Nothing re-energises: iq near zero and the axis IDLE for POST_TRIP_WATCH_S.
    t_end = io.clock() + POST_TRIP_WATCH_S
    while io.clock() < t_end:
        s = io.sample()
        r.peak_a = max(r.peak_a, abs(s.iq_meas))
        if s.flags & DIAG_FLAG_LANE_STALL:
            r.bit3 = True
        if s.fault is not None:
            r.fault_end = s.fault
        iq = max(abs(s.iq_meas), abs(s.iq_set))
        if r.second_rise is None and (iq > POST_TRIP_IQ_A or s.axis_state == AXIS_CLOSED_LOOP):
            r.second_rise = (f"current rose again after the trip ({iq:.1f} A, axis_state "
                             f"{s.axis_state})")
            command_idle(io, say)
        io.sleep(0.01)
    s = io.sample()
    r.temp_after = s.temp_motor
    if r.second_rise is None and s.axis_state != AXIS_IDLE:
        r.second_rise = f"axis_state {s.axis_state} after the trip, need IDLE"
    # A Hand ladder must be REFUSED while latched (firmware activate_allowed: guard ESTOP).
    # Target = where the Hand rests, so a wrongly accepted move has nowhere to go; the
    # stage IDLEs at once if it is accepted. A raw SET_AXIS_STATE is NOT probed: the
    # firmware does not gate it on the latch (only the host arm path refuses it).
    if s.pos is not None and s.pos >= 0.0:
        r.probe = io.hand_move_probe(s.pos)
        if not r.probe.startswith("refused"):
            command_idle(io, say)
        say(f"HAND_MOVE_TO while latched: {r.probe}")
    else:
        say("HAND_MOVE_TO probe skipped (the Hand reads below 0 in the lane frame)")


def _finish(io, cfg, say, vel0, curr0, r, st):
    """IDLE, then the limit, then the frame; each step on its own so one failure skips none.

    Each step catches BaseException: a second Ctrl-C during the restores must not skip
    the ones after it (the interrupt that brought us here is re-raised by the caller)."""
    if st["energised"]:
        try:
            io.disarm()
        except BaseException as e:  # noqa: BLE001
            say(f"!! disarm failed: {e!r}")
        try:
            command_idle(io, say)
        except BaseException as e:  # noqa: BLE001
            say(f"!! IDLE failed: {e!r}: PRESS THE E-STOP BUTTON")
    if st["lowered"]:
        try:
            io.set_limits(vel0, curr0)
            r.limit_readback = io.read_limits()[1]
            r.limit_ok = abs(r.limit_readback - curr0) <= 0.5
        except BaseException as e:  # noqa: BLE001
            say(f"!! limit restore failed: {e!r}")
        say(f"current limit restore: sent {curr0:.1f} A, drive reads "
            f"{r.limit_readback if r.limit_readback is None else f'{r.limit_readback:.1f}'} A "
            f"over USB -> {'OK' if r.limit_ok else 'NOT RESTORED: power-cycle the drive'}")
    else:
        r.limit_ok = True
        r.limit_readback = curr0
    if st["shift"]:
        r.frame_ok = False
        try:
            _wait(io, lambda: (io.sample().axis_state == AXIS_IDLE
                               and abs(io.sample().vel or 0.0) < STILL_VEL_RPS), 2.0)
            s = io.sample()
            if s.axis_state != AXIS_IDLE or s.pos is None:
                say("!! encoder frame NOT restored (Hand not IDLE): re-home before anything else")
            else:
                want = s.pos - st["shift"]
                io.set_abs_pos(want)
                _wait(io, lambda: io.pos() is not None and abs(io.pos() - want) < 0.02, 0.5)
                got = io.pos()
                r.frame_ok = got is not None and abs(got - want) < 0.02
                say(f"encoder frame restored: Hand reads "
                    f"{'?' if got is None else f'{got:+.3f}'} rev (homed frame, want "
                    f"{want:+.3f}); re-home anyway")
        except BaseException as e:  # noqa: BLE001
            say(f"!! encoder frame restore failed: {e!r}: re-home before anything else")
    if r.temp_after is not None:
        say(f"motor temperature {r.temp_before} -> {r.temp_after} C")
    say(stall_verdict(r)[1])
