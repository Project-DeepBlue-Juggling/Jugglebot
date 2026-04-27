#!/usr/bin/env python3
"""Breakaway-torque ramp test — measures static stiction peak per rotor angle.

Procedure per trial::

    1. Axis in IDLE, rotor settles into nearest cogging detent.
    2. Record starting electrical/mechanical angle.
    3. Switch to TORQUE / PASSTHROUGH, CLOSED_LOOP.
    4. Linearly ramp iq_setpoint 0 → ramp_peak_A over ramp_duration_s.
    5. The instant |vel_rps_raw| > BREAKAWAY_VEL_THRESHOLD_RPS — or the
       position cap is touched — record breakaway iq, ramp iq → 0 over
       0.1 s, drop to IDLE.
    6. Between trials, "advance" the rotor by commanding a brief micro-jog
       so the next trial samples a different starting angle.

Stiction is angle-dependent if cogging modulates it.  Sampling N=10
distinct angles and looking at:

    σ(breakaway_iq) / mean(breakaway_iq)

decides:

    < 10 %       → pure stiction, angle-independent.  FF can be a scalar.
    10–25 %      → mild cogging modulation overlaid on stiction.
    > 25 %       → strong angle-dependence, FF needs angle-indexed table
                  even if the cogging map alone (Test A) showed nothing.

Safety
------
* Per-trial position cap — IDLE the instant displacement exceeds
  ``TRIAL_POS_CAP_REV``.  At default 2.5 A peak with the leg motor's rotor
  inertia (~2.75e-4 kg·m²), the motor accelerates at ~90 rev/s²;
  displacement between breakaway-detection and IDLE-applied is < 10 mrev,
  so the trial cap of 0.2 rev is conservative.
* Total session position cap — accumulated displacement across all trials
  also bounded by ``HARD_POSITION_CAP_REV`` (3 rev).  If we drift past it,
  we abort the session.  In practice each breakaway adds < 50 mrev so
  10 trials use < 0.5 rev.
* Soft-stop on detection: iq is ramped 0 over 100 ms before IDLE, not
  cut to zero — avoids exciting any compliance-mode oscillation.
* All exits go through IDLE.

Usage
-----
    # 10 trials forward, default ramp 0→2.5 A over 5 s
    python tests/hardware/breakaway_ramp_test.py --dry-run    # validate first
    python tests/hardware/breakaway_ramp_test.py

    # Both directions, 8 trials each
    python tests/hardware/breakaway_ramp_test.py --direction both --trials 8

    # Higher peak (only if Test A predicts stiction > 2.5 A)
    python tests/hardware/breakaway_ramp_test.py --ramp-peak 3.5

Output
------
    temp/logs/breakaway_<ts>.csv    — one row per trial:
        trial, direction, start_elec_angle_rad, start_mech_angle_rad,
        breakaway_iq_A, breakaway_time_s, breakaway_pos_displacement_rev,
        outcome  ('breakaway' | 'no_breakaway' | 'pos_cap')

    temp/logs/breakaway_<ts>_traces.csv  — optional per-sample trace
        (only with --record-trace).  Columns:
        trial, t_s, iq_setpoint_A, iq_measured_A, pos_rev_raw, vel_rps_raw

Exit codes
----------
0 — normal completion
1 — pre-flight rejection
2 — mid-run abort (position cap, errors, heartbeat loss)
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Reuse the harness + helpers from the bench-test infrastructure
# ---------------------------------------------------------------------------

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
sys.path.insert(0, _SCRIPT_DIR)

from single_leg_test import (  # type: ignore  # noqa: E402
    SingleLegTestHarness,
    encode_set_controller_mode,
    encode_set_input_torque,
    encode_set_input_vel,
    encode_set_state,
    encode_set_vel_curr_limits,
    error_names,
)


# ---------------------------------------------------------------------------
# Safety constants
# ---------------------------------------------------------------------------

HARD_RAMP_PEAK_CAP_A = 4.0          # never command above this iq peak
HARD_CURRENT_LIMIT_A = 5.0          # ODrive soft-limit (ramp_peak << this)
HARD_POSITION_CAP_REV = 3.0         # session-wide cumulative displacement cap
TRIAL_POS_CAP_REV = 0.2             # per-trial: cut iq the instant we drift this far
HEARTBEAT_STALE_S = 1.5

BREAKAWAY_POS_THRESHOLD_REV = 0.05  # |Δpos| above this = rotor escaped detent.
                                    # 1 cogging period ≈ 1/84 ≈ 0.012 rev → 0.05 = ~4 detents.
                                    # Velocity-based detection was tried (2026-04-27)
                                    # but misfires on detent-compliance: as iq tilts the
                                    # detent equilibrium, the rotor "flexes" within its
                                    # starting detent and produces short |vel| spikes
                                    # without ever escaping.  Position alone is the
                                    # only reliable escape signal.
SOFT_STOP_S = 0.1                   # iq ramp-down duration on detection
INTER_TRIAL_REST_S = 1.5            # IDLE pause between trials
INTER_TRIAL_JOG_VEL_RPS = 0.10      # advance jog speed (within Test-A tracking band)
INTER_TRIAL_JOG_DURATION_S = 0.4    # → ~14° rotor advance per trial

DEFAULT_RAMP_PEAK_A = 2.5           # max iq commanded during ramp
DEFAULT_RAMP_DURATION_S = 5.0       # 0 → ramp_peak over this long
DEFAULT_TRIALS = 10
DEFAULT_LOG_RATE_HZ = 200.0         # finer than the 100 Hz CAN broadcast — see note below


@dataclass
class TrialResult:
    trial: int
    direction: str
    start_elec_angle_rad: float
    start_mech_angle_rad: float
    breakaway_iq_A: float
    breakaway_time_s: float
    breakaway_pos_displacement_rev: float
    outcome: str


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------

class BreakawayRampBench:
    def __init__(self, axis_id: int, interface: str, channel: str,
                 trials: int, ramp_peak_A: float, ramp_duration_s: float,
                 direction: str, output_path: str, traces_path: Optional[str],
                 record_trace: bool, dry_run: bool):
        self.harness = SingleLegTestHarness(axis_id=axis_id,
                                            interface=interface, channel=channel)
        self.trials = trials
        self.ramp_peak_A = ramp_peak_A
        self.ramp_duration_s = ramp_duration_s
        self.direction = direction          # 'forward' | 'reverse' | 'both'
        self.output_path = output_path
        self.traces_path = traces_path
        self.record_trace = record_trace
        self.dry_run = dry_run

        try:
            import yaml  # type: ignore
            with open(os.path.join(_PROJECT_ROOT, 'config', 'hardware_config.yaml')) as f:
                cfg = yaml.safe_load(f) or {}
            self.pole_pairs = cfg.get('dynamics', {}).get('motor_pole_pairs', 7)
        except Exception:
            self.pole_pairs = 7

        self._aborted = False
        self._abort_reason = ''
        self._session_start_pos_raw: Optional[float] = None
        self._results: List[TrialResult] = []
        self._traces: List[Tuple[int, float, float, float, float, float]] = []

    # -- pre-flight validation ----------------------------------------------

    def validate(self) -> Optional[str]:
        if self.ramp_peak_A <= 0 or self.ramp_peak_A > HARD_RAMP_PEAK_CAP_A:
            return (f'ramp_peak_A {self.ramp_peak_A} outside (0, {HARD_RAMP_PEAK_CAP_A}]')
        if self.ramp_duration_s < 0.5 or self.ramp_duration_s > 30.0:
            return f'ramp_duration_s {self.ramp_duration_s} outside [0.5, 30.0]'
        if self.trials < 1 or self.trials > 50:
            return f'trials {self.trials} outside [1, 50]'
        if self.direction not in ('forward', 'reverse', 'both'):
            return f'direction {self.direction!r} invalid'
        return None

    # -- session-wide safety check ------------------------------------------

    def _session_safety_ok(self) -> bool:
        """Heartbeat freshness + cumulative displacement check."""
        s = self.harness.state
        age = time.time() - s.last_heartbeat
        if age > HEARTBEAT_STALE_S:
            self._abort_reason = f'heartbeat stale ({age*1000:.0f} ms)'
            return False
        if s.has_errors:
            self._abort_reason = f'ODrive errors: {error_names(s.active_errors)}'
            return False
        if self._session_start_pos_raw is not None:
            cumulative = abs(s.pos_rev_raw - self._session_start_pos_raw)
            if cumulative > HARD_POSITION_CAP_REV:
                self._abort_reason = (f'session position cap exceeded: '
                                       f'cumulative {cumulative:.3f} rev')
                return False
        return True

    # -- mode helpers -------------------------------------------------------

    def _enter_torque_mode(self):
        h = self.harness
        h.send(encode_set_vel_curr_limits(
            h.axis_id,
            vel_limit=2.0,                      # 4× our ramp-induced max
            curr_limit=HARD_CURRENT_LIMIT_A))
        time.sleep(0.05)
        h.send(encode_set_controller_mode(h.axis_id, 'TORQUE', 'PASSTHROUGH'))
        time.sleep(0.05)
        h.send(encode_set_input_torque(h.axis_id, 0.0))
        time.sleep(0.05)
        h.send(encode_set_state(h.axis_id, 'CLOSED_LOOP'))
        h._wait_for_closed_loop()  # type: ignore[attr-defined]

    def _enter_velocity_mode(self):
        h = self.harness
        h.send(encode_set_vel_curr_limits(
            h.axis_id,
            vel_limit=0.5,
            curr_limit=HARD_CURRENT_LIMIT_A))
        time.sleep(0.05)
        h.send(encode_set_controller_mode(h.axis_id, 'VELOCITY', 'PASSTHROUGH'))
        time.sleep(0.05)
        h.send(encode_set_input_vel(h.axis_id, 0.0))
        time.sleep(0.05)
        h.send(encode_set_state(h.axis_id, 'CLOSED_LOOP'))
        h._wait_for_closed_loop()  # type: ignore[attr-defined]

    def _idle(self):
        self.harness.send(encode_set_state(self.harness.axis_id, 'IDLE'))

    def _soft_stop_torque(self):
        """Ramp iq from current setpoint to 0 over SOFT_STOP_S, then IDLE."""
        h = self.harness
        # We don't read back the last commanded torque from the ODrive (it's
        # not a broadcast field); use the live iq_setpoint as a proxy.
        h._poll()  # type: ignore[attr-defined]
        start_iq = float(h.state.iq_setpoint)
        steps = max(2, int(SOFT_STOP_S / 0.01))
        for i in range(steps + 1):
            frac = 1.0 - (i / steps)
            h.send(encode_set_input_torque(h.axis_id, start_iq * frac))
            time.sleep(SOFT_STOP_S / steps)
        self._idle()

    # -- core: one breakaway trial ------------------------------------------

    def _run_trial(self, trial_idx: int, direction: str) -> Optional[TrialResult]:
        h = self.harness

        # Settle in IDLE so the rotor can find its detent
        self._idle()
        time.sleep(INTER_TRIAL_REST_S)
        h._poll()  # type: ignore[attr-defined]
        if not self._session_safety_ok():
            return None

        # Capture starting angles
        two_pi = 2.0 * math.pi
        start_pos_raw = h.state.pos_rev_raw
        start_mech = (start_pos_raw * two_pi) % two_pi
        start_elec = (start_pos_raw * self.pole_pairs * two_pi) % two_pi

        sign = +1.0 if direction == 'forward' else -1.0
        peak = sign * self.ramp_peak_A

        self._enter_torque_mode()

        # Linear ramp iq 0 → peak over ramp_duration_s, polling tightly
        ramp_start = time.time()
        log_period = 1.0 / DEFAULT_LOG_RATE_HZ
        next_log = ramp_start
        last_iq_cmd = 0.0
        breakaway_iq: Optional[float] = None
        breakaway_t: Optional[float] = None
        breakaway_pos: Optional[float] = None
        outcome = 'no_breakaway'
        ramp_end = ramp_start + self.ramp_duration_s

        while time.time() < ramp_end:
            h._poll(timeout=0.001)  # type: ignore[attr-defined]
            now = time.time()
            t_rel = now - ramp_start
            iq_cmd = peak * (t_rel / self.ramp_duration_s)
            # Send command (re-send every ~50 ms or on >1 mA change for tight tracking)
            if abs(iq_cmd - last_iq_cmd) > 0.001 or (now - next_log) > 0.05:
                h.send(encode_set_input_torque(h.axis_id, iq_cmd))
                last_iq_cmd = iq_cmd

            # Trial-level position check — abort the trial cleanly if drift
            # exceeds TRIAL_POS_CAP_REV before breakaway is even detected
            disp = abs(h.state.pos_rev_raw - start_pos_raw)
            if disp > TRIAL_POS_CAP_REV:
                outcome = 'pos_cap'
                breakaway_iq = float(h.state.iq_setpoint)
                breakaway_t = t_rel
                breakaway_pos = disp
                break

            # Breakaway detection — position only.  The rotor must traverse
            # at least BREAKAWAY_POS_THRESHOLD_REV (~4 cogging detents) to be
            # treated as having escaped its starting detent.  Velocity-based
            # detection is NOT used here — short |vel| spikes from in-detent
            # rotor flex would otherwise produce false positives at iq levels
            # well below the true stiction peak.
            if disp > BREAKAWAY_POS_THRESHOLD_REV:
                outcome = 'breakaway'
                breakaway_iq = float(iq_cmd)              # commanded iq (not measured)
                breakaway_t = t_rel
                breakaway_pos = disp
                break

            # Session-level safety
            if not self._session_safety_ok():
                outcome = 'aborted'
                break

            # Trace logging
            if self.record_trace and now >= next_log:
                self._traces.append((trial_idx, t_rel, iq_cmd,
                                     float(h.state.iq_measured),
                                     float(h.state.pos_rev_raw),
                                     float(h.state.vel_rps_raw)))
                next_log += log_period
                if next_log < now:
                    next_log = now + log_period

        # Always soft-stop and return to IDLE
        self._soft_stop_torque()

        if outcome == 'aborted':
            return None
        if breakaway_iq is None:
            # Hit ramp_end without breakaway — record explicitly
            breakaway_iq = self.ramp_peak_A
            breakaway_t = self.ramp_duration_s
            breakaway_pos = abs(h.state.pos_rev_raw - start_pos_raw)
            outcome = 'no_breakaway'

        return TrialResult(
            trial=trial_idx,
            direction=direction,
            start_elec_angle_rad=round(start_elec, 4),
            start_mech_angle_rad=round(start_mech, 4),
            breakaway_iq_A=round(breakaway_iq, 4),
            breakaway_time_s=round(breakaway_t or 0.0, 4),
            breakaway_pos_displacement_rev=round(breakaway_pos or 0.0, 5),
            outcome=outcome,
        )

    # -- inter-trial micro-jog to vary starting angle ----------------------

    def _advance_rotor(self, sign: float = +1.0):
        h = self.harness
        if not self._session_safety_ok():
            return
        self._enter_velocity_mode()
        h.send(encode_set_input_vel(h.axis_id, sign * INTER_TRIAL_JOG_VEL_RPS))
        deadline = time.time() + INTER_TRIAL_JOG_DURATION_S
        while time.time() < deadline:
            h._poll(timeout=0.005)  # type: ignore[attr-defined]
            if not self._session_safety_ok():
                break
        h.send(encode_set_input_vel(h.axis_id, 0.0))
        time.sleep(0.1)
        self._idle()
        time.sleep(0.3)

    # -- top-level run ------------------------------------------------------

    def run(self) -> int:
        err = self.validate()
        if err:
            print(f'REJECT: {err}', file=sys.stderr)
            return 1

        print('Breakaway-torque ramp test')
        print(f'  axis_id        = {self.harness.axis_id}')
        print(f'  trials         = {self.trials}')
        print(f'  direction      = {self.direction}')
        print(f'  ramp           = 0 → {self.ramp_peak_A:.2f} A over {self.ramp_duration_s:.1f} s')
        print(f'  jog between    = {INTER_TRIAL_JOG_VEL_RPS} rev/s × {INTER_TRIAL_JOG_DURATION_S}s '
              f'(~{INTER_TRIAL_JOG_VEL_RPS * INTER_TRIAL_JOG_DURATION_S * 360:.0f}° advance)')
        print(f'  output         = {self.output_path}')
        if self.traces_path:
            print(f'  traces         = {self.traces_path}')
        print(f'  safety caps    = trial-pos≤{TRIAL_POS_CAP_REV} rev, '
              f'session-pos≤{HARD_POSITION_CAP_REV} rev, '
              f'iq≤{HARD_RAMP_PEAK_CAP_A} A')

        if self.dry_run:
            print('\nDRY-RUN: validation passed, no CAN I/O performed.')
            return 0

        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)

        signal.signal(signal.SIGINT, lambda *_: setattr(self, '_aborted', True)
                      or setattr(self, '_abort_reason', 'SIGINT'))

        try:
            with self.harness as h:
                if h.state.has_errors:
                    print(f'  Pre-existing errors: {error_names(h.state.active_errors)}')
                    h.clear_errors()
                    if h.state.has_errors:
                        print('  REJECT: errors persist after clear', file=sys.stderr)
                        return 1
                self._session_start_pos_raw = h.state.pos_rev_raw
                print(f'  Starting pos_raw = {self._session_start_pos_raw:+.4f} rev\n')

                directions: List[str]
                if self.direction == 'both':
                    directions = (['forward'] * self.trials
                                  + ['reverse'] * self.trials)
                else:
                    directions = [self.direction] * self.trials

                for i, dirn in enumerate(directions):
                    if self._aborted:
                        print(f'\nAborted before trial {i}: {self._abort_reason}',
                              file=sys.stderr)
                        break
                    print(f'  Trial {i:2d} ({dirn}): ', end='', flush=True)
                    res = self._run_trial(i, dirn)
                    if res is None:
                        print(f'aborted ({self._abort_reason})')
                        break
                    self._results.append(res)
                    print(f'iq_breakaway = {res.breakaway_iq_A:+.3f} A  '
                          f'in {res.breakaway_time_s:.2f}s  '
                          f'(start θ_elec = {math.degrees(res.start_elec_angle_rad):5.1f}°, '
                          f'pos drift {res.breakaway_pos_displacement_rev*1000:+.1f} mrev, '
                          f'{res.outcome})')

                    # Inter-trial: advance rotor in the opposite of the trial
                    # direction (so the motor doesn't accumulate position bias)
                    if i < len(directions) - 1:
                        advance_sign = -1.0 if dirn == 'forward' else +1.0
                        self._advance_rotor(advance_sign)

                self._idle()
        except Exception as exc:
            print(f'\nABORT: {exc}', file=sys.stderr)
            self._idle()
            return 2

        # Write results
        with open(self.output_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['trial', 'direction', 'start_elec_angle_rad',
                        'start_mech_angle_rad', 'breakaway_iq_A',
                        'breakaway_time_s', 'breakaway_pos_displacement_rev',
                        'outcome'])
            for r in self._results:
                w.writerow([r.trial, r.direction, r.start_elec_angle_rad,
                            r.start_mech_angle_rad, r.breakaway_iq_A,
                            r.breakaway_time_s, r.breakaway_pos_displacement_rev,
                            r.outcome])
        if self.record_trace and self.traces_path:
            with open(self.traces_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['trial', 't_s', 'iq_setpoint_A', 'iq_measured_A',
                            'pos_rev_raw', 'vel_rps_raw'])
                for row in self._traces:
                    w.writerow([row[0], f'{row[1]:.4f}', f'{row[2]:.4f}',
                                f'{row[3]:.4f}', f'{row[4]:.6f}', f'{row[5]:.6f}'])

        print(f'\nDone. {len(self._results)} trials → {self.output_path}')
        return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _timestamp_slug() -> str:
    import datetime
    return datetime.datetime.now().strftime('%Y%m%d_%H%M%S')


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--axis', type=int, default=0)
    p.add_argument('--interface', default='socketcan')
    p.add_argument('--channel', default='can0')
    p.add_argument('--trials', type=int, default=DEFAULT_TRIALS,
                   help=f'trials per direction (default {DEFAULT_TRIALS})')
    p.add_argument('--ramp-peak', type=float, default=DEFAULT_RAMP_PEAK_A,
                   help=f'iq ramp peak in A (≤ {HARD_RAMP_PEAK_CAP_A}, default {DEFAULT_RAMP_PEAK_A})')
    p.add_argument('--ramp-duration', type=float, default=DEFAULT_RAMP_DURATION_S,
                   help=f'ramp 0→peak duration in s (default {DEFAULT_RAMP_DURATION_S})')
    p.add_argument('--direction', choices=['forward', 'reverse', 'both'],
                   default='forward')
    p.add_argument('--output', default=None,
                   help='breakaway summary CSV (default: temp/logs/breakaway_<ts>.csv)')
    p.add_argument('--record-trace', action='store_true',
                   help='also dump per-sample trace CSV (debugging)')
    p.add_argument('--dry-run', action='store_true')
    args = p.parse_args()

    output = args.output or os.path.join(_PROJECT_ROOT, 'temp', 'logs',
                                          f'breakaway_{_timestamp_slug()}.csv')
    traces = (output.replace('.csv', '_traces.csv')
              if args.record_trace else None)

    bench = BreakawayRampBench(
        axis_id=args.axis, interface=args.interface, channel=args.channel,
        trials=args.trials, ramp_peak_A=args.ramp_peak,
        ramp_duration_s=args.ramp_duration,
        direction=args.direction, output_path=output, traces_path=traces,
        record_trace=args.record_trace, dry_run=args.dry_run)
    return bench.run()


if __name__ == '__main__':
    sys.exit(main())
