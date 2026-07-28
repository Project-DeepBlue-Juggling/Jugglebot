#!/usr/bin/env python3
"""Characterise the hand's POST-RELEASE deceleration: authority vs tracking.

WHAT IT ANSWERS
---------------
When the hand overshoots the top of its throw stroke, is the plant

  * **AUTHORITY-limited** — the achieved deceleration saturates, so commanding a
    steeper ramp changes nothing and the only lever is torque; or
  * **TRACKING-limited** — the achieved deceleration follows the commanded one
    with a shortfall, so a bigger commanded braking torque buys margin directly?

The answer decides whether a fix reshapes the commanded profile or corrects the
feedforward, so it is worth measuring rather than assuming.

WHY IT EXISTS
-------------
The hand made **light physical contact with its mechanical end stop** on ~1.2 m
throws during the 2026-07-27 validation sitting.  See
``logbook/2026-07-28-anomaly-fixes-validation-sitting.md`` (§ Discussion → *the
hand's end-stop margin above 0.78 m*, and the 2026-07-28 amendment) and plan
``plans/active/hand-command-continuity.md`` Phase 7.

WHAT IT UNDERPINS
-----------------
Every empirical number in contract **C-HAND-2**
(``ros_ws/docs/hand_decel_feedforward.md``) and in its two test files:
``tests/sim/test_hand_throw_decel_ff.py`` and
``tests/firmware/test_hand_throw_decel_xref.py`` — specifically the DECEL-SIDE
reflected-inertia lower bound that bounds the declared feedforward value, and
the current-headroom argument that rejected steepening the commanded ramp.

It is committed rather than left in ``/tmp`` because the **bench ladder re-runs
it**: ``tests/hardware/session_anomaly_fixes.md`` § CHECK HAND-7 scores the
post-flash capture with exactly this command, and the whole point of the ladder
is comparing the new numbers against the pre-fix ones this probe produced.

WHAT IT MEASURES, PER SELF-TOSS
-------------------------------
Aligned on the commanded profile re-derived from the SHIPPED firmware header
(``Teensy_code/hardware_config.h``, namespace ``TeensyTraj``) — not from a copy:

  ``peak``       max ``pos_meas`` from the stroke through the settle
  ``over_x3``    ``peak - x3``; what has to fit under the end stop
  ``eta``        achieved/commanded decel, inferred from the stopping distance
                 ``eta = d_dec / (d_dec + over_x3)`` with ``d_dec`` the
                 commanded 4.046 rev (velocity-independent, see C-HAND-2)
  ``iq_ramp``    max |q-axis current| over the decel ramp, with ``iq_ramp_brake``
                 the signed braking extreme and ``fr`` the count of FRESH iq
                 samples in that window
  ``t_anchor``   commanded stroke end minus the announcement ``throw_time``
  ``vff_hold``   the residual velocity feedforward standing after the stroke
  ``tor_hold``   the terminal torque feedforward the drive LATCHES
  ``settle``     ``pos_meas - pos_cmd`` once the hand has settled

and, per tier, the decel-side reflected-inertia lower bound and the authority
ceiling it implies at ``jugglebot_operational.hand_curr_limit_a``.

WHICH CHANNEL TO TRUST — they are not equal
-------------------------------------------
The **kinematic** channel (``peak`` / ``over_x3`` / ``eta``) is well sampled and
is what every gated HAND-7 row should rest on.  The **current** channel is not:
see the caveat below.  Where the two disagree, the kinematics win — the decel-side
inertia bound this tool prints deliberately uses no ``iq`` measurement at all.

THE DECEL WINDOW IS ANCHORED ON THE PROFILE, NOT THE ANNOUNCEMENT
-----------------------------------------------------------------
Corrected 2026-07-29.  Until then the ramp window was ``[throw_time,
throw_time + t_dec]``.  On the 2026-07-27 capture the commanded stroke end
lands **120.5-166.9 ms** after ``throw_time`` while ``t_dec`` is only
**52.7-93.3 ms**, so that window had **zero overlap** with the real
deceleration on **13 of 17** tosses and reported the ASCENT current (10-23 A)
where the true decel current is 3.6-11.6 A.  Bench row H7.5 was reading the
wrong phase of the stroke.  The window now ends at the last commanded frame of
the stroke and runs ``t_dec`` backwards from it; ``t_anchor`` is printed per
toss so the dispatch offset stays visible instead of silent.

CAVEAT THAT BOUNDS EVERY CONCLUSION — READ IT
---------------------------------------------
``hand_telemetry`` is a **~100 Hz snapshot of a 500 Hz frame stream**, and
``iq_meas`` is worse: its median repeat-run length across the 2026-07-27 capture
is **4 samples** (mean 7.8, max 918), i.e. an effective refresh of ~13-25 Hz.
Peak currents are therefore UNDER-reported and the true peak of a 47-93 ms decel
ramp may never be sampled.  Treat every current number as a **lower bound**, and
do not fit a plant model tighter than that supports — a cascade simulation using
the shipped ODrive gains under-predicts the measured 4.86 m/s peak by 0.87 rev
while over-predicting the current by 40 %, which is exactly the accuracy this
instrument can defend.

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/hand_decel_authority.py --self-check              # INST-6
    python tools/probes/hand_decel_authority.py --trace temp/logs/toss_trace_*.jsonl
    python tools/probes/hand_decel_authority.py --trace <f> --json      # -> temp/probes/

``--self-check`` is the TWO-SIDED desk gate: it scores a synthetic PRE-fix
capture (overshoot growing with speed, as 2026-07-27 measured) and a synthetic
POST-fix one (overshoot velocity-independent, at the C-HAND-2 pessimistic
bracket) through the same ``analyse()`` the bench uses, and requires the FLAG
verdict on the first and the ACCEPT verdict on the second.  An instrument
validated only against the broken capture scores a working fix as a failure,
which burns a powered sitting — see the block above ``_self_check``.

Offline and read-only: it opens a recorded jsonl and nothing else (``--self-check``
writes its two synthetic traces to ``temp/probes/``).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_HEADER = (_REPO_ROOT / 'ros_ws' / 'src' / 'jugglebot' / 'Teensy_code'
           / 'hardware_config.h')
_OUT_DIR = _REPO_ROOT / 'temp' / 'probes'

#: The hand ODrive's flashed ``torque_constant`` — ``config/ODrive config
#: Files/odrive_pro_hand_config.json``.  TEN TIMES smaller than the legs'
#: (this is a ~1500 Kv motor), which is why the hand's amps look nothing like
#: the legs'.  Not in hardware_config.yaml, so it is named here explicitly.
HAND_KT_NM_PER_A = 0.00551333324983716

#: Measured quiescent hold current of the hand axis, ball-free.  Gravity ASSISTS
#: an upward deceleration, so it is part of the open-loop braking torque and
#: belongs in every decel torque balance.  Read off the 2026-07-27 capture's
#: quiescent samples; it varies 0.83-2.16 A with stroke position (cable weight +
#: stiction), so treat it as +-50 %.
GRAVITY_HOLD_A = 1.50

#: The hand ODrive's flashed NEGATIVE torque clamp — ``config/ODrive config
#: Files/odrive_pro_hand_config.json`` ``axis0.config.torque_soft_min``.  It is
#: **asymmetric**: -0.0551 N.m is exactly -10.00 A at this Kt, while
#: ``torque_soft_max`` is +0.5 N.m (+90.7 A, so the 50 A current limit binds on
#: the motoring side).  If that clamp is live on the flashed drive it truncates
#: the decel feedforward — legacy AND corrected — at every tier above ~0.49 m,
#: which would make this whole phase a no-op.  See C-HAND-2 § The negative
#: torque clamp for the counter-evidence and the bench pre-flight that settles
#: it.  Named here so the number is greppable from the instrument.
HAND_TORQUE_SOFT_MIN_NM = -0.055133331567049026


def _parse_namespace(ns: str) -> dict:
    """Constants of one ``namespace`` block of the SHIPPED firmware header."""
    src = _HEADER.read_text()
    start = src.index('namespace %s {' % ns)
    depth, i = 0, start
    while True:
        if src[i] == '{':
            depth += 1
        elif src[i] == '}':
            depth -= 1
            if depth == 0:
                break
        i += 1
    out = {}
    for m in re.finditer(
            r'constexpr\s+\w+\s+(\w+)\s*=\s*([-+0-9.eE]+)[fu]?\s*;',
            src[start:i]):
        out[m.group(1)] = float(m.group(2))
    return out


class Model:
    """``calcThrow`` from the shipped header — the commanded profile."""

    def __init__(self):
        tt = _parse_namespace('TeensyTraj')
        self.tt = tt
        self.gain = tt['LINEAR_GAIN_FACTOR'] / (math.pi
                                                * tt['HAND_SPOOL_RADIUS_M'] * 2)
        self.total = tt['HAND_STROKE_M'] - 2.0 * tt['STROKE_MARGIN_M']
        self.vel_hold = tt['THROW_VEL_HOLD_PCT'] * self.total
        self.accel_st = self.total - self.vel_hold
        self.ir = tt['INERTIA_RATIO']
        self.x3_rev = self.total * self.gain
        #: Commanded decel distance — velocity-INDEPENDENT, 4.046 rev.
        self.d_dec_rev = (self.ir * self.accel_st / (1.0 + self.ir)) * self.gain

    def at(self, v: float) -> dict:
        t_acc = 2.0 / (self.ir + 1.0) * self.accel_st / v
        t_dec = t_acc * self.ir
        a_acc = v / t_acc
        a_dec = -a_acc / self.ir
        return dict(t_acc=t_acc, t_vel=self.vel_hold / v, t_dec=t_dec,
                    a_acc_rev=a_acc * self.gain,
                    a_dec_rev=abs(a_dec) * self.gain,
                    v_rev=v * self.gain,
                    tor_legacy=abs(a_dec) * self.tt['INERTIA_HAND_ONLY_KG']
                    * self.tt['HAND_SPOOL_RADIUS_M'],
                    tor_corrected=abs(a_dec)
                    * self.tt['THROW_DECEL_REFLECTED_INERTIA_KGM2']
                    * 2.0 * math.pi * self.gain)


def load(path: Path):
    hand, ann = [], []
    with path.open() as fh:
        for line in fh:
            if '"hand_telemetry"' in line:
                r = json.loads(line)
                d = r['d']
                hand.append((r['t_ros'], d['pos_meas'], d['vel_meas'],
                             d['pos_cmd'], d['vel_ff_cmd'], d['tor_ff_cmd'],
                             d['iq_meas']))
            elif '"throw_announcements"' in line:
                ann.append(json.loads(line)['d'])
    hand.sort(key=lambda s: s[0])
    return hand, ann


def _telemetry_staleness(hand):
    """Effective refresh of each field, as repeat-run lengths."""
    out = {}
    for name, idx in (('iq_meas', 6), ('vel_meas', 2), ('pos_meas', 1)):
        runs, cur = [], 1
        for i in range(1, len(hand)):
            if hand[i][idx] == hand[i - 1][idx]:
                cur += 1
            else:
                runs.append(cur)
                cur = 1
        runs.append(cur)
        out[name] = dict(mean=round(statistics.mean(runs), 2),
                         median=statistics.median(runs), max=max(runs))
    return out


def analyse(path: Path, robot: str = 'jugglebot'):
    model = Model()
    hand, ann = load(path)
    rows = []
    for a in ann:
        if a.get('thrower_name') != robot:
            continue
        iv = a['initial_velocity']
        v = math.sqrt(sum(c * c for c in iv)) / 1000.0
        m = model.at(v)
        t0 = a['throw_time']
        w = [s for s in hand if t0 - 0.7 <= s[0] <= t0 + 1.2]
        if len(w) < 20:
            continue
        # commanded stroke end: first sample at/above x3 - one wire LSB
        stop = next((k for k in range(len(w))
                     if w[k][3] >= model.x3_rev - 3e-3), None)
        if stop is None:
            continue                                    # no stroke in window
        peak = max(s[1] for s in w[:stop + 40])
        over = peak - model.x3_rev
        eta = model.d_dec_rev / (model.d_dec_rev + over) if over > -1 else None
        # ── the decel ramp window ────────────────────────────────────────
        # Anchored on the COMMANDED PROFILE (the `stop` sample, i.e. the last
        # commanded frame of the stroke), NOT on the announcement's
        # `throw_time`.  Anchoring on the announcement is wrong and was wrong
        # in this probe until 2026-07-29: `t_stop - t0` measures 120.5-166.9 ms
        # on the 2026-07-27 capture while `t_dec` is only 52.7-93.3 ms, so
        # `[t0, t0+t_dec]` had ZERO overlap with the real decel on 13 of 17
        # tosses and reported the ASCENT current instead (10-23 A, against a
        # true decel 3.6-11.6 A).  Bench row H7.5 read the wrong phase of the
        # stroke entirely.  `t_anchor_ms` is reported per toss so the offset
        # stays visible rather than silent.
        t_stop = w[stop][0]
        ramp = [s for s in w[:stop + 1] if s[0] >= t_stop - m['t_dec']]
        arrest = w[stop:stop + 12]
        hold = [s for s in w[stop:] if abs(s[3] - model.x3_rev) < 3e-3]
        settled = [s for s in hold if abs(s[2]) < 1.0]
        # How many iq samples inside the ramp are actually FRESH.  iq_meas
        # repeats (median run 4), so a window can contain many samples and
        # almost no information; without this the current rows look far more
        # trustworthy than they are.
        fresh = sum(1 for i in range(1, len(ramp)) if ramp[i][6] != ramp[i - 1][6])
        rows.append(dict(
            v_cmd=round(v, 4),
            a_cmd_rev=round(m['a_dec_rev'], 1),
            t_dec_ms=round(m['t_dec'] * 1000, 1),
            t_anchor_ms=round((t_stop - t0) * 1000, 1),
            peak=round(peak, 4),
            over_x3=round(over, 4),
            eta=round(eta, 4) if eta else None,
            iq_ramp=round(max((abs(s[6]) for s in ramp), default=0.0), 2),
            iq_ramp_brake=round(-min((s[6] for s in ramp), default=0.0), 2),
            iq_ramp_fresh=fresh,
            iq_arrest=round(max((abs(s[6]) for s in arrest), default=0.0), 2),
            vff_hold_max=round(max((s[4] for s in hold), default=0.0), 3),
            vff_hold_med=round(statistics.median([s[4] for s in hold]), 3)
            if hold else None,
            # The terminal torque the drive LATCHES: buildSegment stops one
            # 500 Hz sample short of t3, so the last commanded frame carries the
            # FULL decel feedforward and Set_Input_Pos holds it until the next
            # hand command.  This phase multiplies that latched value by 1.289x
            # even though the position/velocity streams do not move, so it is
            # reported (H7.6) rather than left unmeasured.
            tor_hold_max=round(max((abs(s[5]) for s in hold), default=0.0), 4),
            settle_offset=round(statistics.median([s[1] - s[3]
                                                   for s in settled]), 4)
            if settled else None,
            tor_ff_cmd_min=round(min((s[5] for s in w), default=0.0), 4),
        ))
    return model, rows, _telemetry_staleness(hand)


def report(model: Model, rows, stale, curr_limit_a: float = 50.0):
    tt = model.tt
    print(f'x3 = {model.x3_rev:.4f} rev   commanded decel distance = '
          f'{model.d_dec_rev:.4f} rev (velocity-INDEPENDENT)')
    print(f'declared feedforward inertia = '
          f'{tt["THROW_DECEL_REFLECTED_INERTIA_KGM2"]:.3e} kg m^2   '
          f'legacy implied = '
          f'{tt["INERTIA_HAND_ONLY_KG"]*tt["HAND_SPOOL_RADIUS_M"]/(2*math.pi*model.gain):.4e}')
    print('telemetry staleness (repeat-run length; >1 means the field is '
          'aliased):')
    for k, s in stale.items():
        print(f'   {k:9s} mean {s["mean"]:6.2f}  median {s["median"]:4}  '
              f'max {s["max"]}')
    print()
    hdr = ('  v_cmd  a_cmd   t_dec  t_anch |  peak    over_x3   eta  | iq_ramp '
           'brake fr iq_arr | vff_hold(max/med) tor_hold  settle')
    print(hdr)
    print('-' * len(hdr))
    for r in rows:
        print(f'  {r["v_cmd"]:5.3f} {r["a_cmd_rev"]:6.0f} {r["t_dec_ms"]:6.1f} '
              f'{r["t_anchor_ms"]:7.1f} |'
              f'{r["peak"]:8.4f} {r["over_x3"]:+8.4f} '
              f'{(r["eta"] if r["eta"] is not None else float("nan")):6.3f} |'
              f'{r["iq_ramp"]:7.2f} {r["iq_ramp_brake"]:6.2f} '
              f'{r["iq_ramp_fresh"]:2d} {r["iq_arrest"]:6.2f} |'
              f'{r["vff_hold_max"]:9.2f} {(r["vff_hold_med"] or 0.0):7.2f} '
              f'{r["tor_hold_max"]:9.3f} '
              f'{(r["settle_offset"] if r["settle_offset"] is not None else float("nan")):+8.4f}')
    print('  t_anch = commanded stroke end MINUS the announcement throw_time; '
          'the decel')
    print('  window is anchored on the former.  fr = FRESH iq samples inside '
          'that window;')
    print('  at fr <= 1 the current columns are a lower bound and nothing more.')

    tiers = {}
    for r in rows:
        tiers.setdefault(round(r['v_cmd'], 2), []).append(r)
    print()
    print('per-tier means, and the DECEL-SIDE reflected-inertia LOWER BOUND:')
    print('  J_lower = (tau_FF_wire + tau_grav) / (2*pi * a_achieved).')
    print('  It is a genuine bound, not a fit: through the whole overshoot the')
    print('  hand is AHEAD of pos_cmd, so the position loop is braking too')
    print('  (tau_loop >= 0), and friction only adds.  A lower bound is a lower')
    print('  bound, so the LARGEST across tiers is the binding one — that is the')
    print('  number C-HAND-2 anchors its safety clause on.  It does NOT use any')
    print('  |iq| measurement, so the telemetry aliasing above cannot corrupt it.')
    print('  ros_ws/docs/hand_decel_feedforward.md § The declared inertia.')
    print('  v_cmd  n  over_mean  a_ach   a_cmd    eta  | tau_FF+grav  '
          'J_lower_bound')
    j_bound = 0.0
    for v, rs in sorted(tiers.items()):
        m = model.at(v)
        om = statistics.mean(r['over_x3'] for r in rs)
        a_ach = m['v_rev'] ** 2 / (2.0 * (model.d_dec_rev + om))
        # the wire actually carries int16(lrintf(tor*100)), so quantise
        tau_wire = round(m['tor_legacy'] * 100.0) / 100.0
        tau_tot = tau_wire + GRAVITY_HOLD_A * HAND_KT_NM_PER_A
        j_lb = tau_tot / (a_ach * 2 * math.pi)
        j_bound = max(j_bound, j_lb)
        print(f'  {v:5.2f} {len(rs):2d} {om:10.4f} {a_ach:7.0f} '
              f'{m["a_dec_rev"]:7.0f} {a_ach/m["a_dec_rev"]:7.3f} | '
              f'{tau_tot:11.4f}  {j_lb:.4e}')
    print(f'  => binding decel-side lower bound J_true >= {j_bound:.4e} kg m^2')

    # The flatness statistic bench row H7.3 gates R5 on.
    if len(tiers) >= 2:
        means = [statistics.mean(r['over_x3'] for r in rs)
                 for _v, rs in sorted(tiers.items())]
        allover = [r['over_x3'] for r in rows]
        print(f'  flatness (H7.3): spread of per-tier mean over_x3 = '
              f'{max(means)-min(means):.4f} rev   '
              f'(per-toss spread {max(allover)-min(allover):.4f} rev)')

    print()
    clamp_a = HAND_TORQUE_SOFT_MIN_NM / HAND_KT_NM_PER_A
    print(f'!! hand ODrive axis0.config.torque_soft_min = '
          f'{HAND_TORQUE_SOFT_MIN_NM:.6f} Nm = {clamp_a:.2f} A (asymmetric; '
          f'torque_soft_max = +0.5 Nm)')
    print('   If that clamp is LIVE on the flashed drive it truncates the decel')
    print('   feedforward at every tier above ~0.49 m and this phase is a no-op.')
    print('   Counter-evidence from this capture: a hard clamp would force one')
    print('   SINGLE achieved decel at every tier, and the a_ach column above')
    print('   grows 2.6x across the band — so it is very probably NOT binding.')
    print('   Settle it at the bench: read torque_soft_min off the live axis')
    print('   BEFORE the flash (§ CHECK HAND-7 row H7.0c).')
    print()
    print('authority — the reason a steeper commanded ramp was rejected:')
    for j in (1.0126e-5, 1.050e-5):
        ceil_rev = curr_limit_a * HAND_KT_NM_PER_A / (j * 2 * math.pi)
        print(f'  J={j:.3e}: axis decel ceiling at {curr_limit_a:.0f} A = '
              f'{ceil_rev:7.0f} rev/s^2')
        for T in (0.55, 0.80, 1.10):
            m = model.at(9.81 * T / 2.0)
            print(f'      T={T:.2f}s  a_cmd={m["a_dec_rev"]:6.0f} '
                  f'({100*m["a_dec_rev"]/ceil_rev:5.1f} % of ceiling)   '
                  f'decel FF {m["tor_legacy"]/HAND_KT_NM_PER_A:5.1f} A -> '
                  f'{m["tor_corrected"]/HAND_KT_NM_PER_A:5.1f} A')


# ══════════════════════════════════════════════════════════════════════════
#  --self-check — the TWO-SIDED desk gate (bench row INST-6)
# ══════════════════════════════════════════════════════════════════════════
#
# WHY THIS EXISTS, and why one side would not have been enough.
#
# This probe produces three of the four GATED verdicts of § CHECK HAND-7
# (H7.2 peak, H7.3 flatness, H7.5 decel current) and § THE RUN SHEET stage 8
# sends the operator to it as the sole authority for the ladder.  Until
# 2026-07-29 nothing exercised it: `grep -rn hand_decel_authority tests/`
# returned only docstring citations.
#
# An instrument validated only against the BROKEN capture scores a working fix
# as a failure — it routes correct work back for rework and burns a powered
# sitting, which is worse than having no instrument.  So the check is two-sided
# by construction:
#
#   PRE-FIX synthetic  — over_x3 grows with speed exactly as 2026-07-27
#                        measured it.  The probe must FLAG it: flatness spread
#                        well above the H7.3 gate, peak above the R4 band.
#   POST-FIX synthetic — over_x3 velocity-INDEPENDENT at the pessimistic
#                        bracket.  The probe must ACCEPT it: spread under the
#                        gate, every peak inside the ladder's bands.
#
# Each synthetic carries a DISTINCT ascent current and decel current, and the
# ANNOUNCEMENT-TIME OFFSET the real capture has (the announcement lands
# 120-167 ms before the commanded stroke end).  That single property is what
# makes the check able to catch the window bug this file shipped with: with the
# old `[t0, t0+t_dec]` window the probe reported the ASCENT current, so
# `iq_ramp` read ~22 A on a synthetic constructed with a 9 A brake.

_SELF_CHECK_TIERS = (2.742, 3.440, 3.969, 4.858)
#: Pre-fix per-tier overshoot, the 2026-07-27 group means (rev past x3).
_SELF_CHECK_PRE_OVER = (0.0739, 0.0629, 0.3465, 1.0204)
#: Post-fix: the C-HAND-2 pessimistic bracket, velocity-INDEPENDENT by
#: construction (d_dec * (J_true/J_ff - 1) at J_true = 1.050e-5).
_SELF_CHECK_POST_OVER = (0.426, 0.426, 0.426, 0.426)
_SELF_CHECK_ASCENT_A = 22.0
_SELF_CHECK_BRAKE_A = 9.0
#: § CHECK HAND-7 row H7.3 — the flatness gate this instrument feeds.
H7_3_FLATNESS_GATE_REV = 0.35


def _synth_session(model: Model, overs, ann_lead_s: float = 0.140):
    """A synthetic capture with a KNOWN answer, in the recorder's schema.

    ``overs`` is the per-tier overshoot past ``x3`` to build in.  The hand is
    driven along the commanded profile up to ``x3`` and then coasts the
    constructed excess, so ``analyse`` must recover ``over_x3`` to the sample
    resolution.  ``ann_lead_s`` reproduces the real capture's announcement lead.
    """
    dt = 0.010                                   # ~100 Hz, as hand_telemetry is
    hand, ann, t = [], [], 10.0
    for v, over in zip(_SELF_CHECK_TIERS, overs):
        m = model.at(v)
        t_stop = t + ann_lead_s
        ann.append(dict(thrower_name='jugglebot', throw_time=t,
                        initial_velocity=[0.0, 0.0, v * 1000.0]))
        # ascent: pos_cmd climbs to x2, current at the ascent value
        n_up = int(round((t_stop - m['t_dec'] - (t - 0.30)) / dt))
        x2 = model.x3_rev - model.d_dec_rev
        for k in range(n_up):
            frac = k / max(1, n_up - 1)
            hand.append((t - 0.30 + k * dt, x2 * frac, m['v_rev'] * frac,
                         x2 * frac, 0.0, 0.10, _SELF_CHECK_ASCENT_A))
        # decel: pos_cmd runs x2 -> x3 over t_dec, brake current negative
        n_dec = int(round(m['t_dec'] / dt))
        for k in range(n_dec + 1):
            tau = k * dt
            frac = min(1.0, tau / m['t_dec'])
            pc = x2 + model.d_dec_rev * (2 * frac - frac * frac)
            hand.append((t_stop - m['t_dec'] + tau, pc, m['v_rev'] * (1 - frac),
                         pc, m['v_rev'] * (1 - frac) if frac < 1 else 0.0,
                         -m['tor_legacy'], -_SELF_CHECK_BRAKE_A))
        # coast past x3 by `over`, then settle back onto x3
        n_c = 12
        for k in range(n_c):
            frac = (k + 1) / n_c
            hand.append((t_stop + (k + 1) * dt,
                         model.x3_rev + over * (2 * frac - frac * frac),
                         0.0, model.x3_rev, 0.0, -m['tor_legacy'], -2.0))
        for k in range(30):
            hand.append((t_stop + (n_c + k + 1) * dt, model.x3_rev, 0.0,
                         model.x3_rev, 0.0, 0.0, 1.5))
        t = t_stop + 1.5
    return hand, ann


def _self_check() -> int:
    model = Model()
    ok = True

    def emit(label, cond, detail):
        nonlocal ok
        ok = ok and cond
        print(f'  [{"OK " if cond else "BAD"}] {label}: {detail}')

    _OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, overs, must_flag in (('PRE-FIX (must FLAG)',
                                    _SELF_CHECK_PRE_OVER, True),
                                   ('POST-FIX (must ACCEPT)',
                                    _SELF_CHECK_POST_OVER, False)):
        print(f'{name}')
        hand, ann = _synth_session(model, overs)
        # Write a real trace and score it through the SHIPPED analyse(), so the
        # check gates the code path the bench uses.  A self-check that
        # reimplements the window it is checking would have passed on the very
        # bug this check exists to catch.
        path = _OUT_DIR / ('selfcheck_%s.jsonl'
                           % ('prefix' if must_flag else 'postfix'))
        with path.open('w') as fh:
            for s in hand:
                fh.write(json.dumps(dict(
                    t=s[0], t_ros=s[0], topic='hand_telemetry', n=0,
                    d=dict(pos_meas=s[1], vel_meas=s[2], pos_cmd=s[3],
                           vel_ff_cmd=s[4], tor_ff_cmd=s[5], iq_meas=s[6]))) + '\n')
            for a in ann:
                fh.write(json.dumps(dict(t=a['throw_time'],
                                         t_ros=a['throw_time'],
                                         topic='throw_announcements', n=0,
                                         d=a)) + '\n')
        _model, rows, _stale = analyse(path)
        emit('every synthetic toss was found', len(rows) == len(overs),
             f'{len(rows)} of {len(overs)} scored from {path.name}')
        if len(rows) != len(overs):
            continue

        # 1. the kinematic channel must reproduce what was built in
        worst = max(abs(r['over_x3'] - o) for r, o in zip(rows, overs))
        emit('over_x3 recovered', worst < 0.02,
             f'max |recovered - built| = {worst:.4f} rev over {len(rows)} tiers')

        # 2. THE WINDOW.  The decel window must see the BRAKE current, not the
        #    ascent current.  This is the assertion the pre-2026-07-29 window
        #    fails: `[t0, t0+t_dec]` reported the ~22 A ascent here.
        brake = [r['iq_ramp_brake'] for r in rows]
        stray = [r['iq_ramp'] for r in rows]
        emit('decel window sees the brake, not the ascent',
             all(abs(b - _SELF_CHECK_BRAKE_A) < 0.6 for b in brake)
             and all(s < _SELF_CHECK_ASCENT_A - 5.0 for s in stray),
             f'braking |iq| {min(brake):.1f}-{max(brake):.1f} A against a '
             f'constructed {_SELF_CHECK_BRAKE_A:.1f} A brake / '
             f'{_SELF_CHECK_ASCENT_A:.1f} A ascent')

        # 3. the H7.3 verdict must come out on the correct side
        spread = (max(r['over_x3'] for r in rows)
                  - min(r['over_x3'] for r in rows))
        flagged = spread > H7_3_FLATNESS_GATE_REV
        emit('H7.3 flatness verdict', flagged == must_flag,
             f'spread {spread:.4f} rev vs gate {H7_3_FLATNESS_GATE_REV} '
             f'-> {"FLAG" if flagged else "ACCEPT"} '
             f'(expected {"FLAG" if must_flag else "ACCEPT"})')

        # 4. and so must the H7.2 peak verdict at the top tier
        top = rows[-1]['peak']
        emit('H7.2 peak verdict at the top tier', (top > 10.60) == must_flag,
             f'peak {top:.4f} rev vs the 10.60 hard-abort line')
        print()

    print('SELF-CHECK: %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--self-check', action='store_true',
                    help='two-sided synthetic gate; no trace needed (INST-6)')
    ap.add_argument('--trace', nargs='*', default=[],
                    help='toss_trace_*.jsonl file(s) under temp/logs/')
    ap.add_argument('--robot', default='jugglebot',
                    help='thrower_name to score (default: jugglebot)')
    ap.add_argument('--curr-limit-a', type=float, default=50.0,
                    help='hand current limit for the authority table')
    ap.add_argument('--json', action='store_true',
                    help='also write a timestamped JSON to temp/probes/')
    args = ap.parse_args(argv)

    if args.self_check:
        return _self_check()
    if not args.trace:
        ap.error('give --trace <file> ... or --self-check')

    payload = []
    for p in args.trace:
        path = Path(p)
        if not path.exists():
            print(f'!! missing: {path}', file=sys.stderr)
            continue
        print(f'=== {path.name} ===')
        model, rows, stale = analyse(path, args.robot)
        report(model, rows, stale, args.curr_limit_a)
        print()
        payload.append(dict(trace=str(path), rows=rows, staleness=stale,
                            x3_rev=model.x3_rev, d_dec_rev=model.d_dec_rev))

    if args.json:
        _OUT_DIR.mkdir(parents=True, exist_ok=True)
        out = _OUT_DIR / ('hand_decel_authority_%s.json'
                          % time.strftime('%Y%m%d_%H%M%S'))
        out.write_text(json.dumps(payload, indent=2))
        print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
