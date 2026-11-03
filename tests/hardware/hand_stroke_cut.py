"""Pure helpers for ``hand_stream_bench.py --stage stroke_cut`` (JB-145, FW 37/38).

No I/O, no link, no ROS: the stroke profile, the cut-frame selection, the model
of the can-bridge's own Hand stop after the scheduled stream dies, the
envelope refusals and the per-run verdict. The driver imports this module; so
does ``tests/motion/test_hand_stream_bench_stroke_cut.py`` (``tests/hardware``
is ``--ignore``d by pytest, and the driver itself opens a UDP link).

THE MODEL IS THE TWIN, NOT A RE-DERIVATION. Every stop formula below calls the
firmware twin ``tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py``
(``span_eval``, ``hand_c1_stop_eval``, ``hand_c1_stop_duration``,
``stop_duration``, ``stop_eval`` and its constants), which mirrors
``leg_interp.cpp`` line for line and is pinned against the firmware header by
``tests/firmware/``. What this module adds is only the SEQUENCING of the hand
group after its last frame (``_sched_advance``): the frame's span 1, its span 2,
then at cover end + HAND_C1_STOP_GRACE_US (6 ms) the FW 37 decision on the
CONTINUED velocity: above HAND_C1_STOP_MIN_UP_VEL_RPS (50 rev/s, strict) the C1
stop from that state, otherwise the rest of the 20 ms grace and the C2 stop.
"""
from __future__ import annotations

import math
import os
import sys

_TWIN_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "tools", "probes", "teensy_link_profiling", "hermite_xref")
if _TWIN_DIR not in sys.path:
    sys.path.insert(0, _TWIN_DIR)
import teensy_interp as ti  # noqa: E402

SEG_T = ti.SEGMENT_T_S                                   # 0.025 s knot
C1_MIN_UP_VEL = ti.HAND_C1_STOP_MIN_UP_VEL_RPS           # 50 rev/s
C1_DECIDE_S = ti.HAND_C1_DECIDE_US * 1e-6                # cover + 6 ms = 0.056 s
EXHAUST_S = ti.SCHED_EXHAUST_US * 1e-6                   # cover + 20 ms = 0.070 s
STOP_A = ti.STREAM_STOP_HAND_ACCEL_RPS2                  # 4500 rev/s^2
STOP_J = ti.STREAM_STOP_HAND_JERK_RPS3                   # 350000 rev/s^3
HAND_CLIP_REV = ti.HAND_MOTOR_MAX_POSITION               # 10.501 (firmware clip)
MIN_BRIDGE_FW = 38          # FW 37 = the C1 stop; FW 38 = sched_hand_c1_stops uplinked

# Stroke-profile defaults (rev, rev/s, rev/s^2, s). Derivation in the driver's
# docstring and logbook 2026-11-03-jb145-hand-stall-stop-options § Bench-hand rung.
STROKE_ACCEL_DEFAULT = 800.0     # plateau accel: inside a 10 A bench drive (~900-1050 up)
STROKE_JERK_T = 0.020            # jerk ramp time: jerk = accel / 0.02 = 40k rev/s^3
SLOW_PEAK_DEFAULT = 40.0         # the slow control's peak: below the 50 rev/s threshold
SLOW_PEAK_MAX = 45.0             # refuse a "slow" stroke closer than 5 rev/s to 50
RETURN_VEL = 10.0                # the post-cut return move back to the start
RETURN_ACCEL = 200.0
RETURN_JERK_T = 0.050
PRE_HOLD_S = 0.5                 # hand-bearing hold before each stroke
TOP_DWELL_S = 0.2                # dwell at the top between the up and the down leg
TAIL_HOLD_S = 0.5
SETTLE_S = 0.6                   # leg-only frames after the cut: stop (<= ~40 ms) + settle
RESUME_HOLD_S = 0.3              # hand frames AT the firmware hold before the return
BENCH_OVERSHOOT_REV = 1.0        # room left above/below a modelled stop for drive lag
ABORT_FF_POS_GAIN = 35.0         # hw ODRIVE_HAND_POS_GAIN: a disarm leaves vel_ff applied,
                                 # the hand settles at input_pos + vel_ff / pos_gain


class StrokeCutRefusal(ValueError):
    """The stage refuses to run; ``reasons`` lists every unmet condition."""

    def __init__(self, reasons):
        self.reasons = list(reasons)
        super().__init__("; ".join(self.reasons))


# ── Profiles ─────────────────────────────────────────────────────────────────

class PiecewiseJerk:
    """A position profile built from (duration, jerk) pieces, from rest at p0.

    Constant jerk inside each piece, so acceleration is continuous everywhere
    (C2 position). ``sample(t)`` -> (p, v, a); before 0 the start state, past
    the end the end state (rest-to-rest pieces end at v = a = 0)."""

    def __init__(self, p0, pieces):
        self.pieces = [(float(d), float(j)) for d, j in pieces if d > 0.0]
        self.p0 = float(p0)
        self._starts = []
        t, p, v, a = 0.0, self.p0, 0.0, 0.0
        for d, j in self.pieces:
            self._starts.append((t, p, v, a))
            p, v, a = _advance(p, v, a, j, d)
            t += d
        self.duration = t
        self._end = (p, v, a)

    def sample(self, t):
        if t <= 0.0 or not self.pieces:
            return (self.p0, 0.0, 0.0) if not self._starts else self._starts[0][1:]
        if t >= self.duration:
            return self._end
        lo, hi = 0, len(self._starts) - 1
        while lo < hi:                                   # last piece starting <= t
            mid = (lo + hi + 1) // 2
            if self._starts[mid][0] <= t:
                lo = mid
            else:
                hi = mid - 1
        t0, p, v, a = self._starts[lo]
        return _advance(p, v, a, self.pieces[lo][1], t - t0)

    @property
    def end(self):
        return self._end


def _advance(p, v, a, j, d):
    return (p + v * d + a * d * d / 2.0 + j * d ** 3 / 6.0,
            v + a * d + j * d * d / 2.0,
            a + j * d)


def scurve_min_distance(v_pk, accel, t_j):
    """Shortest rest-to-rest move reaching v_pk at plateau accel with t_j ramps."""
    return v_pk * (v_pk / accel + t_j)


def scurve_pieces(dist, v_pk, accel, t_j):
    """(duration, jerk) pieces of a rest-to-rest jerk-limited move of ``dist``
    (signed) that peaks at ``v_pk``: 7 pieces, a cruise only if dist exceeds the
    minimum. Raises ValueError when dist is too short for v_pk at accel."""
    D = abs(float(dist))
    s = 1.0 if dist >= 0 else -1.0
    if v_pk <= 0 or accel <= 0 or t_j <= 0:
        raise ValueError("peak velocity, accel and jerk time must be positive")
    if v_pk < accel * t_j:
        raise ValueError(f"v_pk {v_pk} below accel*t_j {accel * t_j}: the plateau "
                         f"is never reached")
    d_min = scurve_min_distance(v_pk, accel, t_j)
    if D < d_min - 1e-9:
        raise ValueError(f"a {D:.3f} rev move cannot reach {v_pk:.1f} rev/s at "
                         f"{accel:.0f} rev/s^2 (needs {d_min:.3f} rev)")
    j = accel / t_j
    t_a = v_pk / accel - t_j
    t_c = (D - d_min) / v_pk
    return [(t_j, s * j), (t_a, 0.0), (t_j, -s * j), (t_c, 0.0),
            (t_j, -s * j), (t_a, 0.0), (t_j, s * j)]


def stroke_profile(start, top, v_pk, accel=STROKE_ACCEL_DEFAULT, t_j=STROKE_JERK_T,
                   pre_hold=PRE_HOLD_S, dwell=TOP_DWELL_S, tail=TAIL_HOLD_S):
    """hold at start -> up to top (peak v_pk) -> dwell -> back down to start -> hold."""
    up = scurve_pieces(top - start, v_pk, accel, t_j)
    down = scurve_pieces(start - top, v_pk, accel, t_j)
    return PiecewiseJerk(start, [(pre_hold, 0.0)] + up + [(dwell, 0.0)] + down
                         + [(tail, 0.0)])


def return_profile(hold_rev, start, hold_s=RESUME_HOLD_S, tail=TAIL_HOLD_S):
    """The post-cut return: hand frames AT the firmware's hold (a continuous
    resume), then a gentle move back to the stroke start."""
    d = start - hold_rev
    pieces = [(hold_s, 0.0)]
    if abs(d) > 1e-6:
        v = min(RETURN_VEL, math.sqrt(abs(d) * RETURN_ACCEL / 2.0))
        v = max(v, RETURN_ACCEL * RETURN_JERK_T)
        if abs(d) < scurve_min_distance(v, RETURN_ACCEL, RETURN_JERK_T):
            # too short for the plateau: a single slow ramp-only move
            a = abs(d) / (2.0 * RETURN_JERK_T ** 2) * 0.999
            v = a * RETURN_JERK_T
            pieces += scurve_pieces(d, v, a, RETURN_JERK_T)
        else:
            pieces += scurve_pieces(d, v, RETURN_ACCEL, RETURN_JERK_T)
    return PiecewiseJerk(hold_rev, pieces + [(tail, 0.0)])


# ── Frames and the firmware's own stop ───────────────────────────────────────

def frame_knots(traj, k, knot_dt=SEG_T):
    """The hand knots of streamed frame k (u0 at stage time k*T):
    (u0, v0, u1, v1, u2, v2, a0)."""
    t = k * knot_dt
    p0, v0, a0 = traj.sample(t)
    p1, v1, _ = traj.sample(t + knot_dt)
    p2, v2, _ = traj.sample(t + 2.0 * knot_dt)
    return (p0, v0, p1, v1, p2, v2, a0)


def sched_setpoint_cmd(leg_ext, leg_rev, epoch_us, kn=None):
    """The stage's SetpointPump command: the full v8 knot set + knot_epoch_us,
    so the pump sets HAS_SCHED and the firmware plays the frame on the
    SCHEDULED lane (the lane FW 37 changed); the legs are a pinned hold.
    kn = hand (u0, v0, u1, v1, u2, v2, a0); None = a leg-only frame (HAS_HAND
    clear: the hand group receives no newer frame, which IS the stream cut)."""
    cmd = {'type': 'setpoint_cmd', 'seq': 0,
           'ext_mm': list(leg_ext), 'pose_6dof': [0.0, 0.0, 170.0, 0.0, 0.0, 0.0],
           'motor_rev': list(leg_rev), 'vel_mm_s': [0.0] * 6,
           'cmd_next_mm': list(leg_ext), 'cmd_next2_mm': list(leg_ext),
           'vel_next_mm_s': [0.0] * 6, 'vel_next2_mm_s': [0.0] * 6,
           'torque_Nm': [0.0] * 6, 'acc_mm_s2': [0.0] * 6,
           'knot_epoch_us': int(epoch_us)}
    if kn is not None:
        u0, v0, u1, v1, u2, v2, a0 = kn
        cmd.update({'hand_rev': u0, 'hand_vel_rps': v0,
                    'hand_next_rev': u1, 'hand_next2_rev': u2,
                    'hand_next_vel_rps': v1, 'hand_next2_vel_rps': v2,
                    'hand_acc_rps2': a0})
    return cmd


def decision_velocity(kn):
    """The span-2 cubic continued to cover end + 6 ms: the velocity FW 37 tests."""
    _, _, p1, v1, p2, v2, _ = kn
    return ti.span_eval(p1, v1, p2, v2, ti.HAND_C1_S)[1]


def model_stop(kn):
    """Where the firmware's Hand target goes after frame ``kn`` if no newer
    hand frame arrives (times from the frame's stamp, s). Returns a dict:
    path 'C1' | 'C2', g_entry, entry (p, v, a), D, end_rev, g_end."""
    _, _, p1, v1, p2, v2, _ = kn
    pd, vd, ad = ti.span_eval(p1, v1, p2, v2, ti.HAND_C1_S)
    if vd > C1_MIN_UP_VEL:
        D = ti.hand_c1_stop_duration(vd, STOP_A, STOP_J)
        end = ti.hand_c1_stop_eval(pd, vd, STOP_A, STOP_J, D)[0]
        return {'path': 'C1', 'g_entry': C1_DECIDE_S, 'entry': (pd, vd, -STOP_A),
                'D': D, 'end_rev': end, 'g_end': C1_DECIDE_S + D}
    pe, ve, ae = ti.span_eval(p1, v1, p2, v2, ti.SCHED_S_MAX)
    D, _capped = ti.stop_duration([ve], [ae], STOP_A, STOP_J)
    end = ti.stop_eval(pe, ve, ae, D, D)[0]
    return {'path': 'C2', 'g_entry': EXHAUST_S, 'entry': (pe, ve, ae),
            'D': D, 'end_rev': end, 'g_end': EXHAUST_S + D}


def model_stop_fw36(kn):
    """The FW 36 path for the same frame (20 ms grace, then the C2 stop) — the
    discriminator the verdict reads against: what the hold would be had the
    C1 decision not fired."""
    _, _, p1, v1, p2, v2, _ = kn
    pe, ve, ae = ti.span_eval(p1, v1, p2, v2, ti.SCHED_S_MAX)
    D, _ = ti.stop_duration([ve], [ae], STOP_A, STOP_J)
    return ti.stop_eval(pe, ve, ae, D, D)[0]


def model_stop_early(kn):
    """A C2 stop started at the C1 decision instant (cover end + 6 ms) instead
    of after the full 20 ms grace: the slow control's discriminator. A hold
    near this value would mean the grace was cut short below the gate."""
    _, _, p1, v1, p2, v2, _ = kn
    pe, ve, ae = ti.span_eval(p1, v1, p2, v2, ti.HAND_C1_S)
    D, _ = ti.stop_duration([ve], [ae], STOP_A, STOP_J)
    return ti.stop_eval(pe, ve, ae, D, D)[0]


def model_target(kn, g):
    """The firmware Hand target (p, v, a) ``g`` s after frame ``kn``'s stamp,
    no newer hand frame (the belt's reference inside the cut window)."""
    p0, v0, p1, v1, p2, v2, _ = kn
    if g < SEG_T:
        return ti.span_eval(p0, v0, p1, v1, max(g, 0.0) / SEG_T)
    st = model_stop(kn)
    if g < st['g_entry']:
        return ti.span_eval(p1, v1, p2, v2, (g - SEG_T) / SEG_T)
    tau = g - st['g_entry']
    ep, ev, ea = st['entry']
    if st['path'] == 'C1':
        return ti.hand_c1_stop_eval(ep, ev, STOP_A, STOP_J, tau)
    return ti.stop_eval(ep, ev, ea, st['D'], tau)


def select_cut(traj, mode, cut_at_vel, n_frames=None):
    """Index of the LAST hand-bearing frame. upstroke / slow: the first frame
    whose decision velocity exceeds ``cut_at_vel`` upward; downstroke: the first
    whose decision velocity is below -cut_at_vel. None if the profile never
    gets there."""
    n = n_frames if n_frames is not None else int(traj.duration / SEG_T)
    for k in range(n):
        vd = decision_velocity(frame_knots(traj, k))
        if mode == 'downstroke':
            if vd < -cut_at_vel:
                return k
        elif vd > cut_at_vel:
            return k
    return None


def stroke_envelope(traj, n_frames=None):
    """Worst cases over EVERY frame of the planned stroke, as if the stream died
    after any of them (the probe's sweep, applied to the bench stroke):
    (max stop end, min stop end, max abort hold). The abort hold is where a
    driver disarm leaves the hand: input_pos + vel_ff/pos_gain (the bridge
    does not flush vel_ff on a guard DISABLED, leg_interp.cpp JB-73 note)."""
    n = n_frames if n_frames is not None else int(traj.duration / SEG_T)
    hi, lo, ab = -1e9, 1e9, -1e9
    for k in range(n):
        e = model_stop(frame_knots(traj, k))['end_rev']
        hi, lo = max(hi, e), min(lo, e)
        p, v, _ = traj.sample(k * SEG_T)
        ab = max(ab, p + max(v, 0.0) / ABORT_FF_POS_GAIN)
    return hi, lo, ab


class StrokeCutPlan:
    """One validated stroke_cut run: profile, cut frame, modelled stop."""

    def __init__(self, start, top, mode, peak_vel, cut_at_vel, accel, limit_rev):
        self.start, self.top, self.mode = float(start), float(top), mode
        self.peak_vel, self.cut_at_vel, self.accel = peak_vel, cut_at_vel, accel
        self.traj = stroke_profile(start, top, peak_vel, accel)
        self.n_frames = int(self.traj.duration / SEG_T)
        self.cut = select_cut(self.traj, mode, cut_at_vel, self.n_frames)
        self.knots = frame_knots(self.traj, self.cut)
        self.stop = model_stop(self.knots)
        self.fw36_end = model_stop_fw36(self.knots)
        self.early_end = model_stop_early(self.knots)
        self.env_hi, self.env_lo, self.abort_hi = stroke_envelope(self.traj, self.n_frames)
        self.limit_rev = limit_rev

    @property
    def expected_path(self):
        return 'C1' if self.mode == 'upstroke' else 'C2'


def plan_stroke_cut(start, top, mode='upstroke', peak_vel=70.0, cut_at_vel=55.0,
                    accel=STROKE_ACCEL_DEFAULT, limit_rev=HAND_CLIP_REV):
    """Build and validate a run; raise StrokeCutRefusal listing EVERY reason."""
    reasons = []
    if mode not in ('upstroke', 'downstroke', 'slow'):
        raise StrokeCutRefusal([f"unknown --cut-mode {mode!r}"])
    if mode == 'slow':
        if peak_vel >= SLOW_PEAK_MAX:
            reasons.append(f"slow control peak {peak_vel:.1f} rev/s must stay below "
                           f"{SLOW_PEAK_MAX:.0f} (the C1 threshold is "
                           f"{C1_MIN_UP_VEL:.0f}; the control must run the FW 36 path)")
        if not 0.0 < cut_at_vel < peak_vel:
            reasons.append(f"--cut-at-vel {cut_at_vel:.1f} must be inside (0, peak "
                           f"{peak_vel:.1f}) for the slow control")
    else:
        if cut_at_vel < C1_MIN_UP_VEL + 2.0:
            reasons.append(f"--cut-at-vel {cut_at_vel:.1f} must be >= "
                           f"{C1_MIN_UP_VEL + 2.0:.0f} rev/s: the cut has to land "
                           f"clearly on one side of the {C1_MIN_UP_VEL:.0f} rev/s gate")
        if peak_vel < cut_at_vel + 5.0:
            reasons.append(f"--stroke-peak-vel {peak_vel:.1f} must exceed "
                           f"--cut-at-vel {cut_at_vel:.1f} by >= 5 rev/s")
    if start < 0.0:
        reasons.append(f"hand start {start:+.3f} rev below 0")
    if top > limit_rev - BENCH_OVERSHOOT_REV:
        reasons.append(f"--stroke-top {top:.3f} rev leaves less than "
                       f"{BENCH_OVERSHOOT_REV:.1f} rev under the {limit_rev:.3f} clip")
    try:
        scurve_pieces(top - start, peak_vel, accel, STROKE_JERK_T)
    except ValueError as e:
        reasons.append(f"stroke {start:+.3f} -> {top:.3f} rev: {e} — park the hand "
                       f"lower, raise --stroke-top or lower --stroke-peak-vel")
    if reasons:
        raise StrokeCutRefusal(reasons)
    plan = StrokeCutPlan(start, top, mode, peak_vel, cut_at_vel, accel, limit_rev)
    if plan.cut is None:
        reasons.append(f"the stroke never reaches the cut condition "
                       f"({mode}, {cut_at_vel:.1f} rev/s)")
    else:
        vd = decision_velocity(plan.knots)
        if plan.mode == 'slow' and vd > C1_MIN_UP_VEL - 5.0:
            reasons.append(f"slow control: decision velocity {vd:.1f} rev/s is "
                           f"within 5 rev/s of the {C1_MIN_UP_VEL:.0f} rev/s gate — "
                           f"lower --cut-at-vel")
        if plan.stop['path'] != plan.expected_path:
            reasons.append(f"the modelled stop for cut frame {plan.cut} runs the "
                           f"{plan.stop['path']} path, the {mode} run needs "
                           f"{plan.expected_path}")
        if plan.stop['end_rev'] + BENCH_OVERSHOOT_REV > limit_rev:
            reasons.append(f"modelled stop end {plan.stop['end_rev']:.3f} + "
                           f"{BENCH_OVERSHOOT_REV:.1f} rev passes the {limit_rev:.3f} clip")
        if plan.stop['end_rev'] - BENCH_OVERSHOOT_REV < 0.0:
            reasons.append(f"modelled stop end {plan.stop['end_rev']:.3f} leaves less "
                           f"than {BENCH_OVERSHOOT_REV:.1f} rev above the floor (0)")
    if plan.env_hi + BENCH_OVERSHOOT_REV > limit_rev:
        reasons.append(f"a stream death anywhere on this stroke could stop at "
                       f"{plan.env_hi:.3f} rev; + {BENCH_OVERSHOOT_REV:.1f} passes the "
                       f"{limit_rev:.3f} clip — lower --stroke-top")
    if plan.env_lo < 0.0:
        reasons.append(f"a stream death on the down leg could stop at "
                       f"{plan.env_lo:.3f} rev, below the 0 rev floor clip — park "
                       f"the hand higher")
    if plan.abort_hi > limit_rev:
        reasons.append(f"a driver disarm mid-stroke could leave the hand at "
                       f"{plan.abort_hi:.3f} rev (input_pos + vel_ff/pos_gain), past "
                       f"the {limit_rev:.3f} clip")
    if reasons:
        raise StrokeCutRefusal(reasons)
    return plan


def stroke_cut_fw_refusal(fw):
    """None when the bridge can run the stage, else the reason."""
    if fw is None:
        return ("no BridgeIdentity received — the bridge FW is unknown; stroke_cut "
                f"needs FW >= {MIN_BRIDGE_FW}")
    if int(fw) < MIN_BRIDGE_FW:
        return (f"bridge FW {int(fw)} < {MIN_BRIDGE_FW}: FW 37 carries the Hand C1 "
                f"stop and FW 38 uplinks sched_hand_c1_stops; on an older board "
                f"this stage would measure the FW 36 path and read no counter")
    return None


# ── The verdict ──────────────────────────────────────────────────────────────

CMD_TOL_REV = 0.10          # commanded hold vs the modelled stop end
CMD_CEILING_REV = 10.0      # absolute ceiling on the commanded peak
SETTLE_BAND_REV = 0.05      # encoder within 1.6 mm of the hold = settled
SETTLE_BAR_S = 0.150


def settle_time(enc_samples, hold_rev, t_ref, band=SETTLE_BAND_REV):
    """Seconds from t_ref until the encoder enters +-band of hold_rev and stays
    there for the rest of the samples; None if it never settles."""
    last_out = None
    seen = False
    for t, p in enc_samples:
        if t < t_ref:
            continue
        seen = True
        if abs(p - hold_rev) > band:
            last_out = t
    if not seen:
        return None
    if last_out is None:
        return 0.0
    after = [t for t, _ in enc_samples if t > last_out]
    return None if not after else max(0.0, after[0] - t_ref)


def evaluate_stroke_cut(plan, echo_samples, enc_samples, t_stop_end, d_c1, d_stops,
                        d_refused, fault_name=None):
    """Score one run. PURE. echo/enc samples are (wall_s, rev) inside the window
    from the cut to the end of the settle; t_stop_end = wall time the model says
    the stop ends. Returns (verdict, criteria list, summary dict)."""
    crit = []
    s = plan.stop
    end = s['end_rev']
    up = plan.mode != 'downstroke'
    cmd_ext = None
    if echo_samples:
        vals = [e for _, e in echo_samples]
        cmd_ext = max(vals) if up else min(vals)
    hold = echo_samples[-1][1] if echo_samples else None
    enc_ext = None
    if enc_samples:
        ev = [p for _, p in enc_samples]
        enc_ext = max(ev) if up else min(ev)
    overshoot = None if (enc_ext is None or hold is None) else (
        enc_ext - hold if up else hold - enc_ext)
    st = None if hold is None else settle_time(enc_samples, hold, t_stop_end)

    def add(cid, name, ok, detail):
        crit.append({'id': cid, 'name': name,
                     'verdict': 'SKIP' if ok is None else ('PASS' if ok else 'FAIL'),
                     'detail': detail})

    if cmd_ext is None:
        add('S1', 'commanded stop', None, 'no HAND_CMD_ECHO in the window')
    elif up:
        mid = 0.5 * (end + plan.fw36_end)
        if plan.mode == 'upstroke':
            ok = cmd_ext <= end + CMD_TOL_REV and cmd_ext < CMD_CEILING_REV and cmd_ext < mid
        else:                                       # slow: the FW 36 path, grace visible
            mid = 0.5 * (end + plan.early_end)
            ok = abs(cmd_ext - end) <= CMD_TOL_REV and cmd_ext > mid
        alt = (f"FW 36 path would hold {plan.fw36_end:.3f}" if plan.mode == 'upstroke'
               else f"a stop at cover + 6 ms (grace cut short) would hold "
                    f"{plan.early_end:.3f}")
        add('S1', 'commanded stop', ok,
            f"peak {cmd_ext:.3f} rev vs modelled {s['path']} end {end:.3f} "
            f"(tol {CMD_TOL_REV}); {alt}")
    else:
        ok = abs(cmd_ext - end) <= CMD_TOL_REV
        add('S1', 'commanded stop', ok,
            f"lowest {cmd_ext:.3f} rev vs modelled {s['path']} end {end:.3f} "
            f"(tol {CMD_TOL_REV})")
    want_c1 = 1 if plan.mode == 'upstroke' else 0
    add('S2', 'sched_hand_c1_stops delta', None if d_c1 is None else d_c1 == want_c1,
        f"{d_c1} (expected {want_c1})")
    add('S3', 'sched_stops delta', None if d_stops is None else d_stops == 1,
        f"{d_stops} (expected 1: the Hand group only; the legs keep streaming)")
    add('S4', 'no guard trip / latch',
        fault_name is None and (d_refused in (None, 0)),
        f"fault={fault_name or 'NONE'} sched_refused delta={d_refused}")
    add('S5', 'settled hold', None if st is None else st <= SETTLE_BAR_S,
        'never settled inside the window' if (st is None and hold is not None)
        else (f"{'n/a' if st is None else f'{st * 1e3:.0f} ms'} to +-{SETTLE_BAND_REV} "
              f"rev of the hold (bar {SETTLE_BAR_S * 1e3:.0f} ms)"))
    fails = [c for c in crit if c['verdict'] == 'FAIL']
    verdict = 'FAIL' if fails else 'PASS'
    summary = {'cmd_peak': cmd_ext, 'enc_peak': enc_ext, 'hold': hold,
               'overshoot': overshoot, 'settle_s': st}
    return verdict, crit, summary


def verdict_line(run_i, plan, verdict, summary, d_c1, d_stops, events):
    """The one-line per-run verdict the Sitting reads."""
    def f(x, fmt):
        return 'n/a' if x is None else format(x, fmt)
    s = plan.stop
    return (f"[stroke_cut {plan.mode} #{run_i}] {verdict}: cut frame {plan.cut} "
            f"entry v {s['entry'][1]:+.1f} rev/s ({s['path']}) modelled end "
            f"{s['end_rev']:.3f} | cmd peak {f(summary['cmd_peak'], '.3f')} "
            f"enc peak {f(summary['enc_peak'], '.3f')} hold {f(summary['hold'], '.3f')} "
            f"overshoot {f(summary['overshoot'], '+.3f')} settle "
            f"{f(None if summary['settle_s'] is None else summary['settle_s'] * 1e3, '.0f')} ms "
            f"| c1_stops {f(d_c1, '+d')} sched_stops {f(d_stops, '+d')} | "
            f"events: {events or 'none'}")
