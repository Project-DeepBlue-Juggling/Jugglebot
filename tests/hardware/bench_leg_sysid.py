#!/usr/bin/env python3
"""Bench-leg system-ID harness — Stage 1 of the leg-gain tuning methodology.

Direct-CAN (Path DIRECT) system-ID + gain-escalation on the acceptable-loss
bench leg (ODrive node 0), the missing Stage-1 prerequisite of
``plans/active/leg-gain-tuning-methodology.md`` (Fast-motion tier). It talks
socketcan straight to the ODrive via ``SingleLegTestHarness`` — clean
instantaneous steps/chirps to ``input_pos``/``input_vel`` and instant RAM-only
gain-setting over CAN — the only path that can do faithful system-ID and a
fast escalation ladder. The pure logic (onset detector, ladder state machine,
stroke-cap-bounded stimulus generators, step/freq-response estimators) lives in
``sysid_lib.py`` and is unit-tested in ``tests/motion/test_bench_sysid_logic.py``.

Stages (``--mode``)
-------------------
* ``vel_steps``  Stage 1a-2: velocity-loop step responses (rise/overshoot/settle).
* ``pos_steps``  Stage 1a-3a: position-loop step responses (→ ζ, bandwidth).
* ``chirp``      Stage 1a-3b: log-chirp sine sweep 1→30 Hz (the 6 Hz question),
                 amplitude-bounded by stroke → closed-loop gain/phase estimate.
* ``ladder``     Stage 1b: escalate pos_gain until instability onset, searching
                 vel_gain for ζ ≥ target at each rung, with AUTO-BACKOFF to the
                 last stable triple the instant onset is detected.
* ``all``        vel_steps → pos_steps → chirp (system-ID first; run ``ladder``
                 as a separate deliberate invocation).

Safety profile
--------------
The bench leg is SHORTER than a platform leg and the firmware STROKE_MAX_REV
does NOT protect it, so this DRIVER owns the stroke cap. Every stimulus is
clamped to the stroke bounds before a setpoint reaches the motor; a runtime
excursion check drops the axis to IDLE the instant displacement crosses the
cap; and any fault (heartbeat loss, ODrive error, Ctrl-C, or a detected
instability onset) idles the axis before the script proceeds. Hard caps below
are cited from ``cogging_bench_test.py`` — the sibling harness that established
them for this same bench rig with the borrowed brake resistor attached.

Usage
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate

    # Dry-run: print the full plan (bounds, stimuli, ladder) — no CAN I/O.
    python tests/hardware/bench_leg_sysid.py --mode all --dry-run
    python tests/hardware/bench_leg_sysid.py --mode ladder --dry-run

    # System-ID (home first if the encoder reference is fresh-boot):
    python tests/hardware/bench_leg_sysid.py --mode all --home
    # Escalate-until-unstable gain ladder:
    python tests/hardware/bench_leg_sysid.py --mode ladder --home

Output
------
A per-run directory ``temp/probes/bench_sysid_<ts>/`` containing one CSV per
stimulus plus ``manifest.json`` (gains, caps, bounds, thresholds, timestamps,
and per-stage results). CSVs are gitignored under the umbrella ``temp/`` rule.

Exit codes
----------
0 — normal completion    1 — pre-flight rejection    2 — mid-run fault/abort
"""
from __future__ import annotations

import argparse
import csv
import datetime
import json
import os
import signal
import sys
import time
from dataclasses import asdict
from typing import List, Optional

import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
sys.path.insert(0, _SCRIPT_DIR)

import sysid_lib as sid  # noqa: E402
from single_leg_test import (  # type: ignore  # noqa: E402
    SingleLegTestHarness,
    encode_set_controller_mode,
    encode_set_input_pos,
    encode_set_input_vel,
    encode_set_pos_gain,
    encode_set_state,
    encode_set_vel_curr_limits,
    encode_set_vel_gains,
    error_names,
)

# Load baseline leg gains from generated config (the Level-1 HOLD tier).
sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'config', 'generated'))
import hardware_config as hw  # noqa: E402

# ---------------------------------------------------------------------------
# Hard safety caps — cited from cogging_bench_test.py (same bench rig, same
# brake resistor). This harness may push the leg harder than cogging (it is
# acceptable-loss Stage-1 hardware) but it may NEVER exceed these.
# ---------------------------------------------------------------------------

HARD_STROKE_CAP_REV = 3.0     # cogging_bench_test.py:126 HARD_POSITION_CAP_REV —
                              # the bench leg's full mechanical travel. Crashing
                              # past it can damage the ballscrew/cable/motor.
HARD_STROKE_CEIL_REV = 3.30   # highest software ceiling any source cites
                              # (Phase-11 bench cutover, logbook 2026-06-24 U3-iv).
                              # --stroke-cap may be raised toward this ONLY with
                              # --confirm-stroke (operator has re-measured); never
                              # above it.
HARD_VEL_CAP_RPS = 35.0       # cogging_bench_test.py:105 — brake-resistor survival
                              # cap (2.5 m/s leg-end). NOT a tuning velocity: keep
                              # stimuli near Jugglebot's ~2 rev/s operating point.
HARD_ACCEL_CAP_RPS2 = 250.0   # cogging_bench_test.py:137 — brake-resistor-aware,
                              # ~7 A at 250 rev/s² inside the 10 A limit.
HARD_CURRENT_LIMIT_A = 10.0   # cogging_bench_test.py:117 — same as a platform leg.

STROKE_MARGIN_REV = 0.15      # back the usable stimulus window off both end-stops
                              # so a small PID overshoot near a limit can't crash.
SAFETY_EXCURSION_TOL_REV = 0.05  # runtime abort slack outside the usable stroke window.
SAFETY_VEL_TOL_RPS = 1.0      # slack over the session vel cap before a real-time
                              # velocity-runaway abort fires (F6 chirp guard).
CHIRP_TRACK_MARGIN_REV = 0.10  # gross-tracking-error abort budget ABOVE the chirp
                              # amplitude — a real divergence blows past this, normal
                              # phase-lag error does not (F6 chirp guard).

# Default per-run velocity cap — a TUNING-appropriate value, NOT the survival ceiling.
# 4.0 rev/s is Jugglebot's configured ODrive ceiling (~0.28 m/s leg-end, conservative
# test value per the leg-actuator-limits note); the 35 rev/s HARD_VEL_CAP_RPS survival
# ceiling stays reachable by passing --vel-cap 35 explicitly (F4, 2026-07-11).
DEFAULT_VEL_CAP_RPS = 4.0

HEARTBEAT_STALE_S = 1.5       # matches cogging_bench_test.py

# Baseline (Level-1 HOLD tier) gains — the safe fallback the ladder reverts to
# if instability onset fires before any rung has banked a last-good triple.
BASELINE_GAINS = sid.GainTriple(
    pos_gain=float(hw.ODRIVE_LEG_POS_GAINS[0]),
    vel_gain=float(hw.ODRIVE_LEG_VEL_GAINS[0]),
    vel_int_gain=float(hw.ODRIVE_LEG_VEL_INT_GAINS[0]),
)

# Default stimulus parameters (all overridable on the CLI).
DEFAULT_CHIRP_F0_HZ = 1.0
DEFAULT_CHIRP_F1_HZ = 30.0        # covers the 1–30 Hz band; the 6 Hz question
DEFAULT_CHIRP_DURATION_S = 8.0
DEFAULT_CHIRP_AMP_REV = 0.02      # ~1.4 mm p-p on the 71.57 mm/rev bench leg
DEFAULT_CHIRP_FREQS_HZ = [1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 15, 20, 25, 30]

DEFAULT_POS_STEPS_REV = [0.07, 0.14, 0.28, -0.14]  # 5–20 mm steps, within stroke
DEFAULT_VEL_STEPS_RPS = [0.5, 1.0, 2.0]            # up to Jugglebot's ~2 rev/s
DEFAULT_LADDER_STEP_REV = 0.14                     # ~10 mm center step per rung

# Onset thresholds (rev/s ripple + dimensionless autocorrelation score). These
# are conservative defaults; the operator can tighten them from the measured
# hold noise floor (see the methodology's Stage-1b onset criteria).
DEFAULT_RIPPLE_RMS_THRESHOLD = 0.05
DEFAULT_OSC_SCORE_THRESHOLD = 0.5
DEFAULT_IQ_RIPPLE_THRESHOLD = 0.5      # A — braking-current-cycling ripple gate
OVERSHOOT_ONSET = 0.15                 # >15% overshoot (ζ<0.55) = onset

DEFAULT_SEND_PERIOD_S = 0.005          # 200 Hz setpoint/sample cadence
SETTLE_S = 1.0                         # post-step settle window for tail analysis


class BenchSysID:
    """Direct-CAN Stage-1 runner: system-ID stimuli + gain-escalation ladder."""

    def __init__(self, *, axis_id: int, interface: str, channel: str,
                 stroke_cap_rev: float, center_rev: Optional[float],
                 current_limit_a: float, vel_cap_rps: float,
                 output_dir: str, dry_run: bool,
                 chirp_f0: float, chirp_f1: float, chirp_dur: float,
                 chirp_amp: float, pos_steps: List[float],
                 vel_steps: List[float], ladder_step: float,
                 zeta_target: float, bw_clear_hz: Optional[float],
                 n_vel: int, ripple_threshold: float, osc_threshold: float,
                 iq_ripple_threshold: float, do_home: bool):
        self.axis_id = axis_id
        self.interface = interface
        self.channel = channel
        self.stroke_cap_rev = stroke_cap_rev
        self.current_limit_a = current_limit_a
        self.vel_cap_rps = vel_cap_rps
        self.dry_run = dry_run
        self.do_home = do_home
        self.bounds = sid.stroke_bounds(stroke_cap_rev, STROKE_MARGIN_REV)
        # Default the operating centre to mid-stroke — maximal room both ways.
        self.center_rev = center_rev if center_rev is not None else stroke_cap_rev / 2.0
        self.chirp_f0 = chirp_f0
        self.chirp_f1 = chirp_f1
        self.chirp_dur = chirp_dur
        self.chirp_amp = chirp_amp
        self.pos_steps = pos_steps
        self.vel_steps = vel_steps
        self.ladder_step = ladder_step
        self.zeta_target = zeta_target
        self.bw_clear_hz = bw_clear_hz
        self.n_vel = n_vel
        self.ripple_threshold = ripple_threshold
        self.osc_threshold = osc_threshold
        self.iq_ripple_threshold = iq_ripple_threshold
        self.output_dir = output_dir

        self.harness = SingleLegTestHarness(axis_id=axis_id, interface=interface,
                                            channel=channel)
        self._sigint = False
        self._abort_reason = ''
        self._stage_start_pos: Optional[float] = None
        self._manifest = {
            'created': datetime.datetime.now().isoformat(timespec='seconds'),
            'axis_id': axis_id,
            'interface': interface,
            'channel': channel,
            'caps': {
                'stroke_cap_rev': stroke_cap_rev,
                'vel_cap_rps': vel_cap_rps,
                'accel_cap_rps2': HARD_ACCEL_CAP_RPS2,
                'current_limit_a': current_limit_a,
            },
            'bounds_rev': [self.bounds.lo_rev, self.bounds.hi_rev],
            'center_rev': self.center_rev,
            'baseline_gains': asdict(BASELINE_GAINS),
            'onset_thresholds': {
                'ripple_rms': ripple_threshold,
                'oscillation_score': osc_threshold,
                'iq_ripple_a': iq_ripple_threshold,
                'overshoot': OVERSHOOT_ONSET,
            },
            'zeta_target': zeta_target,
            'stages': {},
        }

    # -- pre-flight ----------------------------------------------------------

    def validate_args(self) -> Optional[str]:
        if not (0.0 < self.stroke_cap_rev <= HARD_STROKE_CEIL_REV):
            return (f"stroke cap {self.stroke_cap_rev} rev outside "
                    f"(0, {HARD_STROKE_CEIL_REV}] — the bench leg has no more "
                    f"stroke than that before an end-stop")
        if not self.bounds.contains(self.center_rev):
            return (f"center {self.center_rev:.3f} rev outside usable bounds "
                    f"[{self.bounds.lo_rev:.3f}, {self.bounds.hi_rev:.3f}]")
        if not (0.0 < self.current_limit_a <= HARD_CURRENT_LIMIT_A):
            return f"current limit {self.current_limit_a} A outside (0, {HARD_CURRENT_LIMIT_A}]"
        if not (0.0 < self.vel_cap_rps <= HARD_VEL_CAP_RPS):
            return f"velocity cap {self.vel_cap_rps} rev/s outside (0, {HARD_VEL_CAP_RPS}]"
        for v in self.vel_steps:
            if abs(v) > self.vel_cap_rps:
                return f"velocity step |{v}| exceeds cap {self.vel_cap_rps} rev/s"
        return None

    def _check_safety(self) -> bool:
        """Runtime abort check — sets self._abort_reason and returns False.

        Primary protection is an ABSOLUTE-position bound against the usable stroke
        window (F3): the old RELATIVE displacement-from-stage-start bound used the
        full 3.0-rev stroke as its constant, so it never fired inside the usable
        window and could not catch a velocity return stroke driving INTO the 0.0
        end-stop (the leg starts at the end-stop after --home, not at centre). Every
        stage now centres the leg first, so the whole stimulus stays inside
        ``bounds`` and this catches any excursion past it (± an abort slack).
        Heartbeat + error bits are the other two abort triggers.
        """
        age = time.time() - self.harness.state.last_heartbeat
        if age > HEARTBEAT_STALE_S:
            self._abort_reason = f"heartbeat stale ({age * 1000:.0f} ms)"
            return False
        if self.harness.state.has_errors:
            self._abort_reason = (
                f"ODrive errors: {error_names(self.harness.state.active_errors)}")
            return False
        pos = self.harness.state.pos_rev
        if not self.bounds.contains(pos, tol=SAFETY_EXCURSION_TOL_REV):
            self._abort_reason = (
                f"position {pos:+.3f} rev outside stroke window "
                f"[{self.bounds.lo_rev:.3f}, {self.bounds.hi_rev:.3f}] "
                f"(±{SAFETY_EXCURSION_TOL_REV} tol)")
            return False
        return True

    # -- CAN command helpers (inverted-extension coordinates) ----------------
    #
    # The harness exposes state.pos_rev / state.vel_rps in the Jugglebot
    # "positive = extension" convention. Raw ODrive setpoints negate that (see
    # single_leg_test.test_pos_smoke: extension = negative raw), so E→raw is a
    # single negation and analysis reads state.pos_rev / state.vel_rps directly.

    def _send_pos(self, extension_rev: float, vel_ff: int = 0) -> float:
        e = self.bounds.clamp(extension_rev)   # defence-in-depth stroke clamp
        self.harness.send(encode_set_input_pos(self.axis_id, -e, vel_ff=vel_ff))
        return e

    def _send_vel(self, extension_vel_rps: float) -> float:
        v = max(-self.vel_cap_rps, min(self.vel_cap_rps, extension_vel_rps))
        self.harness.send(encode_set_input_vel(self.axis_id, -v))
        return v

    def set_gains(self, g: sid.GainTriple):
        """Apply a gain triple over CAN — RAM-only, instant, no rebuild."""
        self.harness.send(encode_set_pos_gain(self.axis_id, g.pos_gain))
        self.harness.send(encode_set_vel_gains(self.axis_id, g.vel_gain,
                                               g.vel_int_gain))

    def _apply_limits(self):
        # ODrive vel_limit = EXACTLY the session vel cap (F4). The old ×1.2 programmed
        # the limit 20% ABOVE the cap (42 rev/s at the 35 rev/s survival cap) so a
        # position-domain stimulus could imply a velocity past the stated cap; setting
        # it to the cap bounds every stimulus (steps, chirp) to the cap in hardware.
        self.harness.send(encode_set_vel_curr_limits(
            self.axis_id, vel_limit=self.vel_cap_rps,
            curr_limit=self.current_limit_a))
        time.sleep(0.05)

    # -- data collection -----------------------------------------------------

    def _collect(self, duration_s: float, send_fn=None,
                 send_period_s: float = DEFAULT_SEND_PERIOD_S,
                 writer=None, track_tol_rev: Optional[float] = None,
                 vel_abort_rps: Optional[float] = None) -> dict:
        """Poll CAN for a window; optionally send a setpoint each tick; sample.

        ``send_fn(t_rel) -> float`` computes+sends one command and returns the
        commanded scalar (for logging as the frequency-response input ``u``).
        Aborts (returns ``aborted=True``) on any safety-check failure or Ctrl-C.

        ``track_tol_rev`` / ``vel_abort_rps`` add REAL-TIME divergence aborts (F6):
        a measured position more than ``track_tol_rev`` from the commanded position
        (gross tracking loss) or a measured |velocity| past ``vel_abort_rps`` idles
        the axis mid-window instead of riding the excursion out. The chirp stage
        (an open-loop sweep with no per-step onset check) sets both.
        """
        h = self.harness
        t0 = time.time()
        t_end = t0 + duration_s
        next_t = t0
        T: List[float] = []
        CMD: List[float] = []
        POS: List[float] = []
        VEL: List[float] = []
        IQ: List[float] = []
        aborted = False
        while True:
            now = time.time()
            if now >= t_end:
                break
            if self._sigint:
                aborted = True
                self._abort_reason = 'SIGINT'
                break
            h._poll(timeout=min(0.002, max(0.0, next_t - now)))
            if not self._check_safety():
                aborted = True
                break
            if vel_abort_rps is not None and abs(h.state.vel_rps) > vel_abort_rps:
                self._abort_reason = (f"velocity runaway "
                                      f"{abs(h.state.vel_rps):.2f} rev/s > "
                                      f"{vel_abort_rps:.2f} rev/s")
                aborted = True
                break
            now = time.time()
            if now < next_t:
                continue
            t_rel = now - t0
            cmd = send_fn(t_rel) if send_fn is not None else float('nan')
            if (track_tol_rev is not None and cmd == cmd  # cmd not NaN
                    and abs(h.state.pos_rev - cmd) > track_tol_rev):
                self._abort_reason = (
                    f"tracking error {abs(h.state.pos_rev - cmd):.3f} rev > "
                    f"{track_tol_rev:.3f} rev (cmd {cmd:.3f}, "
                    f"meas {h.state.pos_rev:.3f})")
                aborted = True
                break
            T.append(t_rel)
            CMD.append(cmd if cmd is not None else float('nan'))
            POS.append(h.state.pos_rev)
            VEL.append(h.state.vel_rps)
            IQ.append(h.state.iq_measured)
            if writer is not None:
                writer.writerow([f"{t_rel:.4f}", f"{CMD[-1]:.6f}",
                                 f"{POS[-1]:.6f}", f"{VEL[-1]:.6f}",
                                 f"{IQ[-1]:.4f}", int(h.state.active_errors)])
            next_t += send_period_s
            if next_t < now:
                next_t = now + send_period_s
        return {
            't': np.array(T), 'cmd': np.array(CMD), 'pos': np.array(POS),
            'vel': np.array(VEL), 'iq': np.array(IQ), 'aborted': aborted,
        }

    def _open_csv(self, name: str):
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, name)
        f = open(path, 'w', newline='')
        w = csv.writer(f)
        w.writerow(['t_s', 'cmd', 'pos_rev', 'vel_rps', 'iq_A', 'active_errors'])
        return f, w, path

    # -- mode entry ----------------------------------------------------------

    def _enter_position_hold_at_center(self):
        """POSITION/PASSTHROUGH holding the operating centre; sets stage origin."""
        self.harness.clear_errors()
        self._apply_limits()
        self.harness.enter_position_mode()   # holds current pos, no jump
        # Ramp to centre so there is no step at stage entry.
        self.harness._poll()
        start_e = self.harness.state.pos_rev
        self._stage_start_pos = start_e
        steps = 40
        for i in range(1, steps + 1):
            self._send_pos(start_e + (self.center_rev - start_e) * i / steps)
            self.harness.poll_for(0.02)
        self.harness.poll_for(0.4)
        self._stage_start_pos = self.harness.state.pos_rev

    def _enter_velocity_mode(self):
        # Centre the leg FIRST (F3). A velocity step runs +v then −v, so it needs room
        # to BOTH end-stops; after --home the leg sits at the ~0.0 end-stop and a −v
        # return from there drives straight into it. Centring gives symmetric room and
        # puts the stage start inside the usable window _check_safety now enforces.
        # _enter_position_hold_at_center() leaves the axis CLOSED_LOOP/POSITION at
        # centre; switch it to VELOCITY/PASSTHROUGH in-place (no re-arm) and hold vel 0.
        self._enter_position_hold_at_center()
        self.harness.send(encode_set_controller_mode(
            self.axis_id, 'VELOCITY', 'PASSTHROUGH'))
        time.sleep(0.05)
        self._send_vel(0.0)                    # hold at centre through the mode switch
        self.harness._poll()
        self._stage_start_pos = self.harness.state.pos_rev

    # -- onset evaluation (combines the sysid_lib pure detectors) ------------

    def _evaluate_onset(self, arrays: dict, tail_from: float) -> dict:
        """Run the sysid_lib detectors on a step's settling tail.

        ``tail_from`` is the time (s, relative) after which the response should
        have settled; onset is judged on that tail so the step transient itself
        doesn't read as oscillation.
        """
        t = arrays['t']
        mask = t >= tail_from
        vel_tail = arrays['vel'][mask]
        iq_tail = arrays['iq'][mask]
        onset = sid.detect_instability_onset(
            vel_tail, ripple_rms_threshold=self.ripple_threshold,
            oscillation_score_threshold=self.osc_threshold)
        iqc = sid.braking_current_cycling(
            iq_tail, ripple_threshold=self.iq_ripple_threshold,
            oscillation_threshold=self.osc_threshold)
        return {'onset': onset, 'iq_cycle': iqc}

    def _auto_backoff(self, last_good: Optional[sid.GainTriple], reason: str):
        """Revert to the last stable triple (or the safe baseline) and IDLE."""
        target = last_good if last_good is not None else BASELINE_GAINS
        print(f"  AUTO-BACKOFF ({reason}) → reverting to "
              f"pos={target.pos_gain} vel={target.vel_gain} "
              f"vel_int={target.vel_int_gain}, IDLE")
        try:
            self.set_gains(target)
        except Exception:
            pass
        self.harness.idle_axis()

    # -- Stage 1a-2: velocity-loop step responses ----------------------------

    def stage_vel_steps(self) -> bool:
        print("\n=== Stage 1a-2: velocity-loop step responses ===")
        self._enter_velocity_mode()
        results = []
        # Budget the +v/−v excursion from where the leg ACTUALLY sits (F3): the room is
        # the SMALLER of the up/down distance to the nearer bound from the stage start,
        # so neither the extension nor the return crosses a bound. (_enter_velocity_mode
        # now centres the leg, so this is the symmetric centre room in the normal case —
        # but it stays correct if the leg is off-centre.)
        start = self._stage_start_pos if self._stage_start_pos is not None else self.center_rev
        room = sid.velocity_step_room(start, self.bounds)
        ok = True
        for v in self.vel_steps:
            # Size the step so |v|·duration stays within that single-side room.
            dur = min(sid.max_velocity_step_duration(v, room, frac=0.85), 2.0)
            f, w, path = self._open_csv(f"vel_step_{v:.2f}rps.csv")
            print(f"  |v|={v:.2f} rev/s for {dur:.2f} s → {path}")
            arrays = self._collect(dur, send_fn=lambda tr, vv=v: self._send_vel(vv),
                                    writer=w)
            self._send_vel(0.0)
            # Return toward centre before the next step — through a SAFETY-CHECKED collect
            # (F3): the old poll_for(dur) return had NO safety check, so an over-budget
            # return could drive into the end-stop unseen. _collect runs _check_safety
            # (absolute position) every tick.
            self.harness.poll_for(0.3)
            ret = self._collect(dur, send_fn=lambda tr, vv=v: self._send_vel(-vv))
            self._send_vel(0.0)
            self.harness.poll_for(0.5)
            f.close()
            if arrays['aborted'] or ret['aborted']:
                self.harness.idle_axis()
                print(f"  ABORT: {self._abort_reason}")
                return False
            m = sid.fit_step_response(arrays['t'], arrays['vel'], 0.0, v)
            ev = self._evaluate_onset(arrays, tail_from=dur * 0.5)
            print(f"    rise={m.rise_time_s * 1e3:.1f} ms  "
                  f"overshoot={m.overshoot * 100:.1f}%  "
                  f"settle={m.settling_time_s * 1e3:.1f} ms  "
                  f"unstable={ev['onset'].unstable}")
            results.append({'v_rps': v, 'csv': os.path.basename(path),
                            'metrics': asdict(m),
                            'unstable': ev['onset'].unstable})
            ok = ok and not ev['onset'].unstable
        self.harness.idle_axis()
        self._manifest['stages']['vel_steps'] = results
        return ok

    # -- Stage 1a-3a: position-loop step responses ---------------------------

    def stage_pos_steps(self) -> bool:
        print("\n=== Stage 1a-3a: position-loop step responses ===")
        self._enter_position_hold_at_center()
        steps = sid.position_step_series(self.center_rev, self.pos_steps,
                                         self.bounds)
        results = []
        ok = True
        for req, ps in zip(self.pos_steps, steps):
            if ps.clamped:
                print(f"  NOTE step {req:+.3f} rev clamped to bound "
                      f"{ps.target_rev:.3f} rev")
            # Return to centre first so each step is a clean, labelled-size
            # response (center → center+step) rather than a step from wherever
            # the previous target left the leg.
            self._send_pos(self.center_rev)
            self.harness.poll_for(0.6)
            f, w, path = self._open_csv(f"pos_step_{req:+.3f}rev.csv")
            y0 = self.harness.state.pos_rev
            print(f"  step {req:+.3f} rev → target {ps.target_rev:.3f} rev  {path}")
            arrays = self._collect(SETTLE_S,
                                   send_fn=lambda tr, tgt=ps.target_rev: self._send_pos(tgt),
                                   writer=w)
            f.close()
            if arrays['aborted']:
                self.harness.idle_axis()
                print(f"  ABORT: {self._abort_reason}")
                return False
            m = sid.fit_step_response(arrays['t'], arrays['pos'], y0, ps.target_rev)
            ev = self._evaluate_onset(arrays, tail_from=SETTLE_S * 0.5)
            f_bw = (0.35 / m.rise_time_s) if m.rise_time_s and m.rise_time_s == m.rise_time_s and m.rise_time_s > 0 else float('nan')
            print(f"    rise={m.rise_time_s * 1e3:.1f} ms  "
                  f"overshoot={m.overshoot * 100:.1f}%  ζ≈{m.zeta:.2f}  "
                  f"f_bw≈{f_bw:.1f} Hz  unstable={ev['onset'].unstable}")
            results.append({'requested_rev': req, 'target_rev': ps.target_rev,
                            'clamped': ps.clamped, 'csv': os.path.basename(path),
                            'metrics': asdict(m), 'f_bw_hz': f_bw,
                            'unstable': ev['onset'].unstable})
            ok = ok and not ev['onset'].unstable
        self.harness.idle_axis()
        self._manifest['stages']['pos_steps'] = results
        return ok

    # -- Stage 1a-3b: log-chirp frequency response ---------------------------

    def _chirp_amplitude_request(self) -> float:
        """Requested chirp amplitude reduced to fit the vel/accel caps (F4).

        Bounds the amplitude by BOTH the implied peak velocity (amp·2πf1) and accel
        (amp·(2πf1)²) so the position-domain chirp cannot imply kinematics past the
        caps; ``chirp_position_series`` then further clamps it to the stroke. A 30 Hz
        sweep is accel-bound — the 0.02 rev default implies ~710 rev/s², cut to fit the
        250 rev/s² brake-resistor cap.
        """
        kin = sid.chirp_amplitude_cap_for_kinematics(
            self.chirp_f1, vel_cap_rps=self.vel_cap_rps,
            accel_cap_rps2=HARD_ACCEL_CAP_RPS2)
        return min(self.chirp_amp, kin)

    def stage_chirp(self) -> bool:
        print("\n=== Stage 1a-3b: log-chirp frequency response ===")
        self._enter_position_hold_at_center()
        t_grid = np.arange(0.0, self.chirp_dur, DEFAULT_SEND_PERIOD_S)
        series, amp_used = sid.chirp_position_series(
            t_grid, self.center_rev, self._chirp_amplitude_request(),
            self.chirp_f0, self.chirp_f1, self.chirp_dur, self.bounds)
        pv, pa = sid.chirp_peak_kinematics(amp_used, self.chirp_f1)
        if amp_used < self.chirp_amp - 1e-9:
            print(f"  NOTE chirp amp reduced {self.chirp_amp:.4f} → {amp_used:.4f} rev "
                  f"to fit stroke + vel/accel caps")
        print(f"  chirp {self.chirp_f0:.1f}→{self.chirp_f1:.1f} Hz, "
              f"amp={amp_used:.4f} rev ({amp_used * hw_mm_per_rev() * 2:.2f} mm p-p), "
              f"peak {pv:.2f} rev/s / {pa:.0f} rev/s², {self.chirp_dur:.1f} s")
        f, w, path = self._open_csv("chirp.csv")

        def send(t_rel: float) -> float:
            idx = min(len(series) - 1, int(t_rel / DEFAULT_SEND_PERIOD_S))
            return self._send_pos(series[idx])

        # Real-time chirp aborts (F6): the chirp is an open-loop sweep with no per-step
        # onset check, so guard it live — a gross tracking loss (measured pos more than
        # amp + margin from the command) or a |velocity| past the cap idles the axis
        # rather than riding the excursion out to the end of the sweep.
        arrays = self._collect(
            self.chirp_dur, send_fn=send, writer=w,
            track_tol_rev=amp_used + CHIRP_TRACK_MARGIN_REV,
            vel_abort_rps=self.vel_cap_rps + SAFETY_VEL_TOL_RPS)
        f.close()
        self.harness.idle_axis()
        if arrays['aborted']:
            print(f"  ABORT: {self._abort_reason}")
            return False
        # Frequency response: input = commanded extension, output = measured.
        gains, phases = sid.estimate_frequency_response(
            arrays['t'], arrays['cmd'], arrays['pos'], DEFAULT_CHIRP_FREQS_HZ)
        print("    f[Hz]  gain   phase[deg]")
        points = []
        for fr, g, p in zip(DEFAULT_CHIRP_FREQS_HZ, gains, phases):
            print(f"    {fr:5.1f}  {g:5.2f}  {p:8.1f}")
            points.append({'freq_hz': fr, 'gain': float(g), 'phase_deg': float(p)})
        self._manifest['stages']['chirp'] = {
            'csv': os.path.basename(path), 'amplitude_rev': amp_used,
            'f0_hz': self.chirp_f0, 'f1_hz': self.chirp_f1,
            'response': points}
        return True

    # -- Stage 1b: escalate-until-unstable gain ladder -----------------------

    def stage_ladder(self) -> bool:
        print("\n=== Stage 1b: escalate-until-unstable gain ladder ===")
        lad = sid.GainLadder(zeta_target=self.zeta_target,
                             bw_clear_hz=self.bw_clear_hz)
        self._enter_position_hold_at_center()
        rungs_log = []
        winner: Optional[sid.GainTriple] = None
        while not lad.stopped:
            rung = lad.current_rung()
            print(f"\n  rung {lad.index}: pos_gain={rung.pos_gain} "
                  f"vel_int={rung.vel_int_gain:.3f} "
                  f"vel_gain∈[{rung.vel_gain_lo}, {rung.vel_gain_hi}]")
            best: Optional[sid.GainTriple] = None
            best_zeta: Optional[float] = None
            best_unstable = False
            stable_found = False
            best_bw = None
            for vg in sid.vel_gain_candidates(rung, self.n_vel):
                triple = sid.GainTriple(rung.pos_gain, float(vg), rung.vel_int_gain)
                self.set_gains(triple)
                f, w, path = self._open_csv(
                    f"ladder_p{rung.pos_gain:.0f}_v{vg:.2f}.csv")
                y0 = self.harness.state.pos_rev
                tgt = self.bounds.clamp(self.center_rev + self.ladder_step)
                arrays = self._collect(
                    SETTLE_S, send_fn=lambda tr, t=tgt: self._send_pos(t),
                    writer=w)
                # Return to centre for the next candidate.
                self._send_pos(self.center_rev)
                self.harness.poll_for(0.4)
                f.close()
                if arrays['aborted']:
                    self._auto_backoff(lad.last_good, self._abort_reason)
                    self._finish_ladder(lad, rungs_log, lad.last_good)
                    return False
                m = sid.fit_step_response(arrays['t'], arrays['pos'], y0, tgt)
                ev = self._evaluate_onset(arrays, tail_from=SETTLE_S * 0.5)
                unstable = (ev['onset'].unstable or ev['iq_cycle'].cycling
                            or m.overshoot > OVERSHOOT_ONSET)
                bw = (0.35 / m.rise_time_s) if m.rise_time_s and m.rise_time_s > 0 else None
                print(f"    vel_gain={vg:.2f}: ζ≈{m.zeta:.2f} "
                      f"overshoot={m.overshoot * 100:.1f}% "
                      f"unstable={unstable}")
                best, best_zeta, best_unstable, best_bw = triple, m.zeta, unstable, bw
                if not unstable and m.zeta >= self.zeta_target:
                    stable_found = True
                    break

            onset_for_record = sid.OnsetResult(
                unstable=best_unstable, ripple_rms=0.0, oscillation_score=0.0,
                reasons=[])
            dec = lad.record(best, onset_for_record, best_zeta,
                             bandwidth_hz=best_bw,
                             stable_vel_gain_found=stable_found)
            rungs_log.append({'pos_gain': rung.pos_gain,
                              'tested': asdict(best) if best else None,
                              'zeta': best_zeta, 'stable_found': stable_found,
                              'action': dec.action, 'reason': dec.reason})
            print(f"    → {dec.action} ({dec.reason})")
            if dec.action == 'backoff':
                self._auto_backoff(dec.gains, dec.reason)
                winner = dec.gains
                break
            if dec.action == 'stop_ok':
                winner = dec.gains
                if winner:
                    self.set_gains(winner)
                self.harness.idle_axis()
                break
            # escalate → loop continues with the new current_rung
        self._finish_ladder(lad, rungs_log, winner)
        return True

    def _finish_ladder(self, lad, rungs_log, winner):
        self._manifest['stages']['ladder'] = {
            'stop_reason': lad.stop_reason,
            'winner': asdict(winner) if winner else None,
            'rungs': rungs_log,
        }
        if winner:
            print(f"\n  Ladder winner: pos_gain={winner.pos_gain} "
                  f"vel_gain={winner.vel_gain} vel_int={winner.vel_int_gain} "
                  f"(stop: {lad.stop_reason})")
        else:
            print(f"\n  Ladder produced no winner (stop: {lad.stop_reason})")

    # -- manifest ------------------------------------------------------------

    def write_manifest(self):
        os.makedirs(self.output_dir, exist_ok=True)
        self._manifest['finished'] = datetime.datetime.now().isoformat(
            timespec='seconds')
        path = os.path.join(self.output_dir, 'manifest.json')
        with open(path, 'w') as f:
            json.dump(self._manifest, f, indent=2)
        print(f"\n  Manifest: {path}")

    # -- dry-run plan --------------------------------------------------------

    def print_plan(self, modes: List[str]):
        print("\n" + "=" * 64)
        print("DRY-RUN — bench-leg system-ID plan (no CAN I/O)")
        print("=" * 64)
        print(f"  axis={self.axis_id}  interface={self.interface}  "
              f"channel={self.channel}")
        print(f"  caps: stroke≤{self.stroke_cap_rev} rev, vel≤{self.vel_cap_rps} "
              f"rev/s, accel≤{HARD_ACCEL_CAP_RPS2} rev/s², "
              f"curr≤{self.current_limit_a} A")
        print(f"  bounds=[{self.bounds.lo_rev:.3f}, {self.bounds.hi_rev:.3f}] rev  "
              f"center={self.center_rev:.3f} rev")
        print(f"  onset: ripple>{self.ripple_threshold} rev/s AND "
              f"osc-score>{self.osc_threshold}; iq-ripple>{self.iq_ripple_threshold} A; "
              f"overshoot>{OVERSHOOT_ONSET * 100:.0f}%")
        # vel_steps centre the leg first, so the planned room is the symmetric
        # centre room (F3: velocity_step_room from the centre == that symmetric value).
        room = sid.velocity_step_room(self.center_rev, self.bounds)

        if 'vel_steps' in modes:
            print("\n  [vel_steps] velocity-loop step responses (centred, "
                  f"room={room:.3f} rev each way):")
            for v in self.vel_steps:
                dur = min(sid.max_velocity_step_duration(v, room, 0.85), 2.0)
                print(f"    |v|={v:.2f} rev/s  dur={dur:.2f} s  "
                      f"predicted disp={sid.predicted_velocity_displacement(v, dur):.3f} rev")
        if 'pos_steps' in modes:
            print("\n  [pos_steps] position-loop step responses:")
            for req, ps in zip(self.pos_steps,
                               sid.position_step_series(self.center_rev,
                                                        self.pos_steps, self.bounds)):
                flag = " (CLAMPED)" if ps.clamped else ""
                print(f"    step {req:+.3f} rev → target {ps.target_rev:.3f} rev{flag}")
        if 'chirp' in modes:
            t_grid = np.arange(0.0, self.chirp_dur, DEFAULT_SEND_PERIOD_S)
            _, amp = sid.chirp_position_series(
                t_grid, self.center_rev, self._chirp_amplitude_request(),
                self.chirp_f0, self.chirp_f1, self.chirp_dur, self.bounds)
            pv, pa = sid.chirp_peak_kinematics(amp, self.chirp_f1)
            print("\n  [chirp] log sine sweep:")
            print(f"    {self.chirp_f0:.1f}→{self.chirp_f1:.1f} Hz, "
                  f"amp={amp:.4f} rev (requested {self.chirp_amp:.4f}), "
                  f"dur={self.chirp_dur:.1f} s, {len(t_grid)} samples")
            print(f"    implied peak: {pv:.2f} rev/s / {pa:.0f} rev/s²  "
                  f"(caps {self.vel_cap_rps} rev/s, {HARD_ACCEL_CAP_RPS2:.0f} rev/s²)")
            print(f"    response bins: {DEFAULT_CHIRP_FREQS_HZ} Hz")
        if 'ladder' in modes:
            print("\n  [ladder] escalate-until-unstable, ζ_target="
                  f"{self.zeta_target}:")
            for i, rung in enumerate(sid.default_ladder()):
                cands = [f"{c:.2f}" for c in sid.vel_gain_candidates(rung, self.n_vel)]
                print(f"    rung {i}: pos_gain={rung.pos_gain} "
                      f"vel_int={rung.vel_int_gain:.3f}  vel_gain∈{{{', '.join(cands)}}}")
            print(f"    center step per rung: {self.ladder_step:+.3f} rev; "
                  f"AUTO-BACKOFF → last-good on onset")
        print("\n  DRY-RUN complete — validation passed, no motor commanded.")

    # -- top-level run -------------------------------------------------------

    def run(self, modes: List[str]) -> int:
        err = self.validate_args()
        if err:
            print(f"REJECT: {err}", file=sys.stderr)
            return 1
        if self.dry_run:
            self.print_plan(modes)
            return 0

        def sigint(signum, frame):
            print("\n[SIGINT] requesting abort → IDLE")
            self._sigint = True
            try:
                self.harness.idle_axis()
            except Exception:
                pass
        signal.signal(signal.SIGINT, sigint)

        stage_fns = {
            'vel_steps': self.stage_vel_steps,
            'pos_steps': self.stage_pos_steps,
            'chirp': self.stage_chirp,
            'ladder': self.stage_ladder,
        }
        rc = 0
        try:
            with self.harness as h:
                print(f"  Heartbeat OK (state: {h.state.state_name})")
                if h.state.has_errors:
                    print(f"  Clearing pre-existing errors: "
                          f"{error_names(h.state.active_errors)}")
                    h.clear_errors()
                if self.do_home:
                    h.home_axis()
                for mode in modes:
                    ok = stage_fns[mode]()
                    self._manifest['stages'].setdefault(mode, {})
                    if not ok:
                        rc = 2
                        print(f"  Stage '{mode}' reported a fault/onset — stopping.")
                        break
        except KeyboardInterrupt:
            rc = 2
        except Exception as exc:
            print(f"\nFATAL: {exc}", file=sys.stderr)
            rc = 1
        finally:
            try:
                self.write_manifest()
            except Exception as exc:
                print(f"  (manifest write failed: {exc})", file=sys.stderr)

        if rc == 0:
            print("\nPASS: all requested stages completed.")
        elif rc == 2:
            print(f"\nABORT: {self._abort_reason or 'stage fault/onset'}",
                  file=sys.stderr)
        return rc


def hw_mm_per_rev() -> float:
    """Bench-leg geometry (mm per motor rev) — for human-readable chirp p-p."""
    return 71.5708  # single_leg_test.py:106 (measured)


MODE_GROUPS = {'all': ['vel_steps', 'pos_steps', 'chirp']}


def _default_output_dir() -> str:
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(_PROJECT_ROOT, 'temp', 'probes', f'bench_sysid_{ts}')


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--mode', default='all',
                   choices=['vel_steps', 'pos_steps', 'chirp', 'ladder', 'all'],
                   help="stage(s) to run (default: all = vel_steps,pos_steps,chirp)")
    p.add_argument('--axis', type=int, default=0, help='CAN node id (default 0)')
    p.add_argument('--interface', default='socketcan')
    p.add_argument('--channel', default='can0')
    p.add_argument('--stroke-cap', type=float, default=HARD_STROKE_CAP_REV,
                   help=f'software stroke cap in rev (default {HARD_STROKE_CAP_REV}; '
                        f'lowering is always allowed, raising above '
                        f'{HARD_STROKE_CAP_REV} needs --confirm-stroke and never '
                        f'exceeds {HARD_STROKE_CEIL_REV})')
    p.add_argument('--confirm-stroke', action='store_true',
                   help='acknowledge an operator-remeasured stroke > 3.0 rev')
    p.add_argument('--center', type=float, default=None,
                   help='operating centre in rev (default: mid-stroke)')
    p.add_argument('--current-limit', type=float, default=HARD_CURRENT_LIMIT_A)
    p.add_argument('--vel-cap', type=float, default=DEFAULT_VEL_CAP_RPS,
                   help=f'per-run velocity cap in rev/s (default '
                        f'{DEFAULT_VEL_CAP_RPS}, the tuning operating point; raise '
                        f'toward the {HARD_VEL_CAP_RPS} rev/s survival ceiling only '
                        f'deliberately, e.g. --vel-cap {HARD_VEL_CAP_RPS:.0f})')
    p.add_argument('--home', action='store_true',
                   help='home the axis (drive to end-stop, set zero) before stages')
    p.add_argument('--chirp-f0', type=float, default=DEFAULT_CHIRP_F0_HZ)
    p.add_argument('--chirp-f1', type=float, default=DEFAULT_CHIRP_F1_HZ)
    p.add_argument('--chirp-duration', type=float, default=DEFAULT_CHIRP_DURATION_S)
    p.add_argument('--chirp-amp', type=float, default=DEFAULT_CHIRP_AMP_REV,
                   help='chirp amplitude in rev (auto-reduced to fit stroke)')
    p.add_argument('--zeta-target', type=float, default=0.7)
    p.add_argument('--bw-clear-hz', type=float, default=None,
                   help='if set, ladder stops once closed-loop bandwidth clears '
                        'this with ζ≥target (default: climb until unstable)')
    p.add_argument('--n-vel', type=int, default=4,
                   help='vel_gain search points per ladder rung (default 4)')
    p.add_argument('--ripple-threshold', type=float,
                   default=DEFAULT_RIPPLE_RMS_THRESHOLD)
    p.add_argument('--osc-threshold', type=float, default=DEFAULT_OSC_SCORE_THRESHOLD)
    p.add_argument('--iq-ripple-threshold', type=float,
                   default=DEFAULT_IQ_RIPPLE_THRESHOLD)
    p.add_argument('--output-dir', default=None,
                   help='output dir (default temp/probes/bench_sysid_<ts>/)')
    p.add_argument('--dry-run', action='store_true',
                   help='print the full plan and exit without touching CAN')
    args = p.parse_args()

    # Stroke-cap gating: lowering is free; raising above the default needs the
    # operator to confirm a re-measured stroke, and never exceeds the ceiling.
    if args.stroke_cap > HARD_STROKE_CAP_REV:
        if not args.confirm_stroke:
            print(f"REJECT: --stroke-cap {args.stroke_cap} > {HARD_STROKE_CAP_REV} "
                  f"rev requires --confirm-stroke (operator has re-measured the "
                  f"bench leg's true stroke)", file=sys.stderr)
            return 1
        if args.stroke_cap > HARD_STROKE_CEIL_REV:
            print(f"REJECT: --stroke-cap {args.stroke_cap} exceeds the absolute "
                  f"ceiling {HARD_STROKE_CEIL_REV} rev — no source cites a longer "
                  f"stroke; crashing the end-stop damages the leg", file=sys.stderr)
            return 1
        print(f"  WARNING: stroke cap raised to {args.stroke_cap} rev "
              f"(operator-confirmed)")

    modes = MODE_GROUPS.get(args.mode, [args.mode])
    output_dir = args.output_dir or _default_output_dir()

    runner = BenchSysID(
        axis_id=args.axis, interface=args.interface, channel=args.channel,
        stroke_cap_rev=args.stroke_cap, center_rev=args.center,
        current_limit_a=args.current_limit, vel_cap_rps=args.vel_cap,
        output_dir=output_dir, dry_run=args.dry_run,
        chirp_f0=args.chirp_f0, chirp_f1=args.chirp_f1,
        chirp_dur=args.chirp_duration, chirp_amp=args.chirp_amp,
        pos_steps=DEFAULT_POS_STEPS_REV, vel_steps=DEFAULT_VEL_STEPS_RPS,
        ladder_step=DEFAULT_LADDER_STEP_REV, zeta_target=args.zeta_target,
        bw_clear_hz=args.bw_clear_hz, n_vel=args.n_vel,
        ripple_threshold=args.ripple_threshold, osc_threshold=args.osc_threshold,
        iq_ripple_threshold=args.iq_ripple_threshold, do_home=args.home)
    return runner.run(modes)


if __name__ == '__main__':
    sys.exit(main())
