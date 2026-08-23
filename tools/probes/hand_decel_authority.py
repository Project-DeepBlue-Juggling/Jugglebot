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
``plans/archived/hand-command-continuity.md`` Phase 7.

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
(``Teensy_code_platform/hardware_config.h``, namespace ``TeensyTraj``) — not from a copy:

  ``peak``       max ``pos_meas`` from the stroke through the settle — the
                 END-STOP SAFETY column, largest excursion of ANY cause
  ``cst_pk``     max ``pos_meas`` inside the THROW'S OWN COAST (below)
  ``cst_min``    min ``pos_meas`` in the same window — the OVER-BRAKE side
  ``over_x3``    ``cst_pk - x3``; how far past the stroke top the throw went
  ``under``      ``x3 - cst_min``; how far under it the throw ended up
  ``eta``        achieved/commanded decel, inferred from the stopping distance
                 ``eta = d_dec / (d_dec + over_x3)`` with ``d_dec`` the
                 commanded 4.046 rev (velocity-independent, see C-HAND-2).
                 ``eta > 1`` means the hand stopped SHORT of x3
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

WHY THE COAST WINDOW EXISTS — AND WHAT IT COST BEFORE IT DID
------------------------------------------------------------
Added 2026-08-23.  Until then ``peak`` was ``max(pos_meas)`` over the 40
samples (~400 ms) after the commanded stroke end, and ``over_x3`` / ``eta`` /
the inertia bound were all derived from it.  **That window routinely contains a
LATER COMMANDED MOVE.**  C-HAND-1's gated catch arm dispatches its prelude
37-183 ms after the stroke completes, and that prelude re-seeds ``pos_cmd`` at
the live encoder position and climbs back to the stroke top — overshooting it
by a measured 0.046-0.222 rev.  When the throw's own coast tops out lower than
that, the reported ``peak`` is the ARM's excursion, ~200 ms after the throw
ended, and every number derived from it describes the arm.

Measured contamination, scoring the recorded captures through this file:

  * 2026-07-27 (the sitting **every empirical number in C-HAND-2 is drawn
    from**): 10 of 17 tosses.  Both LOW tiers are contaminated on every toss —
    the 2.742 and 3.440 m/s rows read ``over_x3`` **+0.074 / +0.063** rev while
    the throws' own coasts topped out **-0.051 / -0.092** rev, i.e. BELOW x3.
    The contract's "the plant already tracks to eta = 0.98 at the bottom of the
    band" rests on those two rows.
  * 2026-08-23 (the first HAND-7 ladder on the unclamped drive): **15 of 15**.
    Reported ``over_x3`` +0.10..+0.24 rev; the coasts actually finished
    **0.058-0.292 rev BELOW x3** and sagged to **0.113-0.468 rev below** before the
    arm arrived.  The sign of the whole result was inverted.

So the window now ends at the first sample whose ``pos_cmd`` leaves the latched
stroke top — the arm prelude's own first frame, the same event
``hand_stroke_timeline.py`` reports as ``post_stroke_cmd``.  ``peak`` KEEPS the
wide window, because the end-stop question is "what is the largest excursion of
any cause" and the arm's overshoot is often the answer; narrowing that column
would trade a measurement bug for a safety hole.  The identification columns
(``cst_pk`` / ``cst_min`` / ``over_x3`` / ``under`` / ``eta`` / the inertia
bound) are the ones that move onto the coast.

AND THE TORQUE IN THE BALANCE COMES OFF THE WIRE, NOT OFF A MODEL
-----------------------------------------------------------------
Same date, same class of defect.  The per-tier inertia bound multiplied
``tor_legacy`` — ``accelToTorque``, the PRE-C-HAND-2 feedforward — even after
Platform FW 2 put ``throwDecelToTorque`` aboard, understating the shipped
braking torque by 1.289x and the bound with it.  The capture carries the
answer: ``buildSegment`` stops one 500 Hz sample short of ``t3``, so the last
commanded frame latches the FULL decel feedforward across the coast, where
``tor_ff_cmd`` is directly observable and already int16-quantised by the wire.
The 2026-08-23 ladder reads 0.050 / 0.090 / 0.110 / 0.150 / 0.170 N.m against a
corrected model of 0.054 / 0.087 / 0.113 / 0.145 / 0.174 and a legacy model of
0.042 / 0.068 / 0.088 / 0.113 / 0.135 — the two generations are never
confusable, and the probe now prints which one it found.

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
    python tools/probes/hand_decel_authority.py --bag ~/Desktop/rosbags/<stamp>
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
_HEADER = (_REPO_ROOT / 'ros_ws' / 'src' / 'jugglebot' / 'Teensy_code_platform'
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

#: The hand ODrive's NEGATIVE torque clamp as it was flashed UNTIL 2026-08-18.
#: It was **asymmetric**: -0.0551 N.m is exactly -10.00 A at this Kt, against a
#: ``torque_soft_max`` of +0.5 N.m (+90.7 A, so the 50 A current limit bound the
#: motoring side and only the braking side was clamped).  **CONFIRMED LIVE and
#: REMOVED at the bench on 2026-08-18** — H7.0c's pre-registered hypothesis was
#: right and this instrument's own "very probably NOT binding" counter-evidence
#: (achieved decel grew 2.6x across the tiers) was WRONG.  Kept named because
#: every capture taken before 2026-08-18 was measured THROUGH it: the braking
#: iq in those bags is clamp-limited, not plant-limited, so no inertia or
#: authority number derived from them transfers to the restored drive.
HAND_TORQUE_SOFT_MIN_NM_PRE_2026_08_18 = -0.055133331567049026

#: The clamp flashed from 2026-08-18 onward — symmetric +-0.7 N.m = +-127 A at
#: this Kt, i.e. far outside ``current_soft_max`` 50.0 A, so the CURRENT limit
#: is the fence on both sides and the torque clamp no longer truncates anything.
HAND_TORQUE_SOFT_MIN_NM = -0.7
HAND_TORQUE_SOFT_MAX_NM = 0.7

#: How close ``pos_cmd`` has to sit to ``x3`` to count as "still latched at the
#: stroke top".  One wire LSB of the position stream; the same band that finds
#: the commanded stroke end, so the coast opens and closes on one criterion.
_X3_BAND_REV = 3e-3


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


def _sec(x) -> float:
    """``builtin_interfaces/Time`` or a float -> seconds."""
    try:
        return float(x)
    except (TypeError, ValueError):
        return float(getattr(x, 'sec', 0)) + float(getattr(x, 'nanosec', 0)) * 1e-9


def load_bag(bag_dir: Path):
    """The same two channels, out of a session rosbag.

    The operator records BAGS, not ``toss_trace`` jsonl — the § CHECK HAND-7
    recording command is a ``ros2 bag record`` — so a probe that can only read
    a trace cannot score the capture the ladder actually produces.  Same shape
    as ``load`` so every consumer below is untouched.
    """
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError as exc:                                # pragma: no cover
        raise SystemExit(f'ERROR: mcap_ros2 not importable ({exc}); '
                         'bag input needs it (pip install mcap-ros2-support)')
    files = sorted(bag_dir.glob('*.mcap'))
    if not files:
        raise SystemExit(f'ERROR: no .mcap files under {bag_dir}')
    hand, ann = [], []
    for f in files:
        for msg in read_ros2_messages(
                str(f), topics=['/hand_telemetry', '/throw_announcements']):
            t, d = msg.log_time_ns / 1e9, msg.ros_msg
            if msg.channel.topic.endswith('hand_telemetry'):
                hand.append((t, d.pos_meas, d.vel_meas, d.pos_cmd,
                             d.vel_ff_cmd, d.tor_ff_cmd, d.iq_meas))
            else:
                v = d.initial_velocity
                ann.append(dict(thrower_name=str(d.thrower_name),
                                throw_time=_sec(d.throw_time),
                                initial_velocity=[v.x, v.y, v.z]))
    hand.sort(key=lambda s: s[0])
    return hand, ann


def load(path: Path):
    if path.is_dir():
        return load_bag(path)
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
                     if w[k][3] >= model.x3_rev - _X3_BAND_REV), None)
        if stop is None:
            continue                                    # no stroke in window
        # ── the COAST: the throw's own excursion, and nothing else ───────
        # Bounded at the first command that takes `pos_cmd` off the latched
        # stroke top — the gated catch arm's own prelude on a healthy capture.
        # Everything the DECELERATION is identified from has to live inside
        # this window; see the module docstring's WHY THE COAST WINDOW block.
        coast_end = next((k for k in range(stop + 1, len(w))
                          if abs(w[k][3] - model.x3_rev) > _X3_BAND_REV),
                         len(w))
        coast = w[stop:coast_end]
        coast_peak = max(s[1] for s in coast)
        coast_min = min(s[1] for s in coast)
        over = coast_peak - model.x3_rev
        eta = model.d_dec_rev / (model.d_dec_rev + over) if over > -1 else None
        # `peak` stays the END-STOP SAFETY column and keeps its wide window:
        # the largest excursion of ANY cause is what has to fit under the
        # 10.8 rev stop, and on a healthy capture the arm prelude's own
        # overshoot is routinely the largest one.  Narrowing THIS column would
        # trade a measurement bug for a safety hole.
        peak = max(s[1] for s in w[:stop + 40])
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
        hold = [s for s in w[stop:] if abs(s[3] - model.x3_rev) < _X3_BAND_REV]
        settled = [s for s in hold if abs(s[2]) < 1.0]
        # The decel feedforward AS THE WIRE CARRIED IT, read off the capture
        # instead of re-derived from a firmware generation this file has to
        # guess.  `buildSegment` stops one 500 Hz sample short of t3, so the
        # last commanded frame of the stroke carries the FULL decel
        # feedforward and `Set_Input_Pos` latches it across the coast — which
        # makes the coast the one window where the commanded braking torque is
        # directly observable.  Quantised to 0.01 N.m by the wire, so this is
        # the drive's actual input, not a model of it.
        tau_wire = max((abs(s[5]) for s in coast), default=0.0)
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
            coast_ms=round((w[min(coast_end, len(w) - 1)][0] - t_stop) * 1000, 1),
            coast_peak=round(coast_peak, 4),
            coast_min=round(coast_min, 4),
            over_x3=round(over, 4),
            under_x3=round(max(0.0, model.x3_rev - coast_min), 4),
            peak_is_coast=bool(abs(peak - coast_peak) < 1e-9),
            tau_wire=round(tau_wire, 4),
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
    hdr = ('  v_cmd  a_cmd   t_dec  t_anch |  peak   cst_pk  over_x3  cst_min '
           'under   eta  | iq_ramp brake fr iq_arr | vff_hold(max/med) tor_hold'
           '  settle')
    print(hdr)
    print('-' * len(hdr))
    for r in rows:
        print(f'  {r["v_cmd"]:5.3f} {r["a_cmd_rev"]:6.0f} {r["t_dec_ms"]:6.1f} '
              f'{r["t_anchor_ms"]:7.1f} |'
              f'{r["peak"]:8.4f}{"" if r["peak_is_coast"] else "*"}'
              f'{r["coast_peak"]:8.4f} {r["over_x3"]:+8.4f} '
              f'{r["coast_min"]:8.4f} {r["under_x3"]:6.4f} '
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
    stray = [r for r in rows if not r['peak_is_coast']]
    print('  peak = the END-STOP safety column, largest excursion of ANY cause. '
          'cst_pk /')
    print('  cst_min = the THROW\'s own coast, bounded at the first command that '
          'leaves the')
    print('  stroke top; over_x3 and eta are derived from cst_pk, never from '
          'peak.  A `*`')
    print('  marks a toss whose `peak` belongs to a LATER commanded move — '
          f'{len(stray)} of {len(rows)}')
    print('  here.  `under` is how far below x3 the coast ended up: it is the '
          'over-brake')
    print('  side of C-HAND-2, and it is NOT visible in `peak` at all.')

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
    print('  IT IS A BOUND ONLY WHILE THE HAND FINISHES ABOVE x3.  The sign of')
    print('  tau_loop is the sign of (pos_meas - pos_cmd): a coast that tops out')
    print('  BELOW the latched stroke top never caught the command, so the loop')
    print('  was pushing UP through the whole excursion and the SAME expression')
    print('  is an UPPER bound instead.  Flagged per tier rather than averaged')
    print('  away, because the two cases cannot be pooled.')
    print('  v_cmd  n  over_mean  a_ach   a_cmd    eta  | tau_wire+grav  '
          'J_estimate  sense')
    j_bound, j_upper = 0.0, []
    for v, rs in sorted(tiers.items()):
        m = model.at(v)
        om = statistics.mean(r['over_x3'] for r in rs)
        a_ach = m['v_rev'] ** 2 / (2.0 * (model.d_dec_rev + om))
        # The wire torque comes from the CAPTURE (already int16-quantised by
        # the wire), not from a model of whichever firmware generation is
        # aboard.  Deriving it from `tor_legacy` under-reports the shipped
        # feedforward by INERTIA_RATIO-scale factors the moment the board
        # carries the corrected one, and the bound scales with it one-for-one.
        tau_wire = statistics.median(r['tau_wire'] for r in rs)
        tau_tot = tau_wire + GRAVITY_HOLD_A * HAND_KT_NM_PER_A
        j_lb = tau_tot / (a_ach * 2 * math.pi)
        above = om > 0.0
        if above:
            j_bound = max(j_bound, j_lb)
        else:
            j_upper.append(j_lb)
        print(f'  {v:5.2f} {len(rs):2d} {om:10.4f} {a_ach:7.0f} '
              f'{m["a_dec_rev"]:7.0f} {a_ach/m["a_dec_rev"]:7.3f} | '
              f'{tau_tot:13.4f}  {j_lb:.4e}  '
              f'{"J >= this" if above else "J <= this (coast ended BELOW x3)"}')
    if j_bound:
        print(f'  => binding decel-side lower bound J_true >= {j_bound:.4e} '
              'kg m^2')
    if j_upper:
        print(f'  => {len(j_upper)} tier(s) finished BELOW x3 and bound J_true '
              f'<= {min(j_upper):.4e} kg m^2 instead.')
        print('     A tier on each side is not a measurement, it is a '
              'CONTRADICTION:')
        print('     re-read the release-speed cross-check before concluding '
              'anything')
        print('     about the declared inertia from this capture.')
    # Which feedforward generation is actually aboard?  Stated, not assumed:
    # the same capture that carries the kinematics carries the wire torque.
    if rows:
        v0 = rows[0]['v_cmd']
        m0, tw = model.at(v0), rows[0]['tau_wire']
        leg, cor = (round(m0['tor_legacy'] * 100) / 100,
                    round(m0['tor_corrected'] * 100) / 100)
        gen = ('CORRECTED (throwDecelToTorque)' if abs(tw - cor) < abs(tw - leg)
               else 'LEGACY (accelToTorque)')
        print(f'  feedforward aboard, from the wire: {tw:.3f} N.m at '
              f'{v0:.3f} m/s  (legacy would be {leg:.2f}, corrected '
              f'{cor:.2f})  => {gen}')

    # The flatness statistic bench row H7.3 gates R5 on.
    if len(tiers) >= 2:
        means = [statistics.mean(r['over_x3'] for r in rs)
                 for _v, rs in sorted(tiers.items())]
        allover = [r['over_x3'] for r in rows]
        print(f'  flatness (H7.3): spread of per-tier mean over_x3 = '
              f'{max(means)-min(means):.4f} rev   '
              f'(per-toss spread {max(allover)-min(allover):.4f} rev)')

    print()
    old_clamp_a = HAND_TORQUE_SOFT_MIN_NM_PRE_2026_08_18 / HAND_KT_NM_PER_A
    now_clamp_a = HAND_TORQUE_SOFT_MIN_NM / HAND_KT_NM_PER_A
    print(f'!! hand ODrive torque clamp: was '
          f'{HAND_TORQUE_SOFT_MIN_NM_PRE_2026_08_18:.6f} Nm = {old_clamp_a:.2f} A '
          f'(asymmetric, torque_soft_max +0.5 Nm) until 2026-08-18; now '
          f'+-{HAND_TORQUE_SOFT_MAX_NM:.1f} Nm = +-{abs(now_clamp_a):.0f} A.')
    print('   H7.0c CONFIRMED the old clamp was LIVE and it has been removed —')
    print('   the 50 A current_soft_max is the fence on both sides now.')
    print('   This instrument previously argued the clamp was "very probably NOT')
    print('   binding" because a_ach grows 2.6x across the band. That argument')
    print('   was WRONG; do not resurrect it.')
    print('   ==> Any capture dated BEFORE 2026-08-18 has clamp-limited braking')
    print('   iq. Inertia and authority numbers derived from those bags (incl.')
    print("   C-HAND-2's J >= 1.0126e-5) must be re-derived on the restored drive.")
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
#: The gated catch arm's prelude, built into BOTH synthetics: how far it sags
#: under x3 before re-seeding, and how far its climb overshoots x3.  Both sit
#: inside the 2026-07-27 measured spread over 25 re-seeds (sag 0.056-0.172 rev,
#: settle 0.046-0.222 rev above x3).  The overshoot is deliberately LARGER than
#: the post-fix synthetic's own coast (0.426 rev) at no tier and larger than the
#: pre-fix low tiers' (0.063-0.074 rev) at every one, which is exactly the
#: selective contamination the real captures show.
_SELF_CHECK_ARM_SAG = 0.156
_SELF_CHECK_ARM_OVER = 0.222
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
        # decel: pos_cmd runs x2 -> x3 over t_dec, brake current negative.
        # The wire torque is the CORRECTED feedforward, int16-quantised the way
        # the wire carries it — the shipped generation since Platform FW 2, and
        # what the report's generation line has to recognise.
        tau = -round(m['tor_corrected'] * 100.0) / 100.0
        n_dec = int(round(m['t_dec'] / dt))
        for k in range(n_dec + 1):
            el = k * dt
            frac = min(1.0, el / m['t_dec'])
            pc = x2 + model.d_dec_rev * (2 * frac - frac * frac)
            hand.append((t_stop - m['t_dec'] + el, pc, m['v_rev'] * (1 - frac),
                         pc, m['v_rev'] * (1 - frac) if frac < 1 else 0.0,
                         tau, -_SELF_CHECK_BRAKE_A))
        # coast past x3 by `over`, then settle back onto x3
        n_c = 12
        for k in range(n_c):
            frac = (k + 1) / n_c
            hand.append((t_stop + (k + 1) * dt,
                         model.x3_rev + over * (2 * frac - frac * frac),
                         0.0, model.x3_rev, 0.0, tau, -2.0))
        for k in range(6):
            hand.append((t_stop + (n_c + k + 1) * dt, model.x3_rev + over,
                         0.0, model.x3_rev, 0.0, tau, 1.5))
        # ── THE GATED CATCH ARM'S PRELUDE ────────────────────────────────
        # `pos_cmd` leaves the latched stroke top, re-seeds at the live
        # encoder position and climbs back — overshooting x3 by
        # `_SELF_CHECK_ARM_OVER`.  This is the event that made `peak` describe
        # a move 200 ms after the throw on 10 of 17 real tosses; without it in
        # the synthetic the window bug passes its own gate.
        n_a = 24
        for k in range(n_a):
            frac = (k + 1) / n_a
            hand.append((t_stop + (n_c + 6 + k + 1) * dt,
                         model.x3_rev + over
                         + (_SELF_CHECK_ARM_OVER - over) * frac,
                         0.0,
                         model.x3_rev - _SELF_CHECK_ARM_SAG * (1.0 - frac),
                         0.0, 0.0, 1.5))
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

        # 1. the kinematic channel must reproduce what was built in.  With the
        #    arm prelude in the synthetic this is ALSO the window assertion:
        #    an unbounded peak search reports the arm's +0.222 rev on every
        #    tier whose own coast is smaller, which is 2 of the 4 pre-fix
        #    tiers and none of the post-fix ones.
        worst = max(abs(r['over_x3'] - o) for r, o in zip(rows, overs))
        emit('over_x3 recovered from the COAST, not the arm prelude',
             worst < 0.02,
             f'max |recovered - built| = {worst:.4f} rev over {len(rows)} '
             f'tiers (arm overshoots x3 by {_SELF_CHECK_ARM_OVER} rev '
             f'{_SELF_CHECK_ARM_SAG} rev below it)')

        # 1b. and the END-STOP column must still see that arm — narrowing the
        #     coast must not blind the row that guards the 10.8 rev stop.
        armed = [r for r, o in zip(rows, overs) if o < _SELF_CHECK_ARM_OVER]
        emit('peak still sees the arm prelude (end-stop column intact)',
             all(r['peak'] >= model.x3_rev + _SELF_CHECK_ARM_OVER - 0.02
                 for r in armed) and all(not r['peak_is_coast'] for r in armed),
             f'{len(armed)} tier(s) whose coast is smaller than the arm; '
             f'peak {min((r["peak"] for r in armed), default=float("nan")):.4f} '
             f'rev vs x3+arm {model.x3_rev + _SELF_CHECK_ARM_OVER:.4f}')

        # 1c. the wire feedforward has to be read off the capture, and the
        #     generation named.  A bound built on `tor_legacy` while the board
        #     carries `throwDecelToTorque` is understated by 1.289x.
        cor = [round(model.at(v)['tor_corrected'] * 100) / 100
               for v in _SELF_CHECK_TIERS]
        emit('wire feedforward read off the capture',
             all(abs(r['tau_wire'] - c) < 5e-4 for r, c in zip(rows, cor)),
             f'{[r["tau_wire"] for r in rows]} vs the corrected wire {cor}')

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
    # The WHOLE module docstring, with its line breaks — not
    # `__doc__.splitlines()[0]`, which is the truncation 907ed30 swept out of the
    # five other bench CLIs and left here. This one carries the coast-window
    # definition, the "which channel to trust" ruling and the per-column units an
    # operator needs while standing at the robot; a --help that drops them sends
    # them back to a 900-line source file at exactly the wrong moment.
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--self-check', action='store_true',
                    help='two-sided synthetic gate; no trace needed (INST-6)')
    ap.add_argument('--trace', nargs='*', default=[],
                    help='toss_trace_*.jsonl file(s) under temp/logs/')
    ap.add_argument('--bag', nargs='*', default=[],
                    help='rosbag director(ies) containing .mcap — what the '
                         '§ CHECK HAND-7 recording command actually produces')
    ap.add_argument('--robot', default='jugglebot',
                    help='thrower_name to score (default: jugglebot)')
    ap.add_argument('--curr-limit-a', type=float, default=50.0,
                    help='hand current limit for the authority table')
    ap.add_argument('--json', action='store_true',
                    help='also write a timestamped JSON to temp/probes/')
    args = ap.parse_args(argv)

    if args.self_check:
        return _self_check()
    if not args.trace and not args.bag:
        ap.error('give --trace <file> ..., --bag <dir> ... or --self-check')

    payload = []
    for p in list(args.trace) + list(args.bag):
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
