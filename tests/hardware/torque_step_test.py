#!/usr/bin/env python3
"""Torque-step terminal-velocity test — clean friction-vs-velocity in TORQUE mode.

Why this exists
---------------
Test A (velocity-mode cruise) was found to be contaminated by velocity-loop
limit-cycle iq.  The motor at 1 rps target has iq oscillating ±1.6 A around
−0.78 A mean — the velocity controller is fighting cogging-detent torque
ripple at 84 cycles/rev (= 84 Hz at 1 rps).  The "mean iq" Test A reports
is therefore not pure friction — it's friction + cogging-fight.

Test B (single torque ramp) revealed that the rotor moves at iq < 0.2 A,
inconsistent with Test A's claimed 1 A friction.

This test resolves the discrepancy by running in TORQUE / PASSTHROUGH mode
(no velocity loop in the way).  At each applied torque level, the motor
accelerates from rest, friction balances applied torque at terminal velocity,
and we record the (iq, v_terminal) pair.  This gives τ_friction(v) directly,
free of controller artifacts.

Protocol per torque level
-------------------------
1. IDLE; rotor settles in nearest cogging detent.
2. Enter TORQUE/PASSTHROUGH; apply +iq.
3. Monitor vel and pos until either (a) vel plateaus (Δvel/Δt < threshold)
   or (b) |Δpos| from start exceeds ``PULSE_POS_LIMIT_REV``.
4. Record (iq, v_terminal, |Δpos|, time_to_terminal).
5. Apply −iq to return the rotor to ≈ start; same termination rules.
6. Settle at iq=0 for ``INTER_STEP_REST_S``; advance to next iq level.

Each iq level produces 2 data points (fwd + rev).  Position is bounded
per pulse, so accumulated displacement across the run stays small.

Safety
------
* Same hard caps as cogging_bench_test.py and breakaway_ramp_test.py.
* Per-pulse position guard at PULSE_POS_LIMIT_REV (0.5 rev) — if |Δpos|
  exceeds this during a pulse, the pulse ends immediately (with whatever
  vel we'd reached).
* Session-wide cumulative displacement cap at HARD_POSITION_CAP_REV
  (3 rev = bench leg full stroke).
* All exits go through IDLE.

Usage
-----
    python tests/hardware/torque_step_test.py --dry-run
    python tests/hardware/torque_step_test.py
    python tests/hardware/torque_step_test.py --iq-levels 0.05,0.1,0.2,0.5,1.0,1.5,2.0
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
from typing import List, Optional

# ---------------------------------------------------------------------------
# Reuse the harness
# ---------------------------------------------------------------------------

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
sys.path.insert(0, _SCRIPT_DIR)

from single_leg_test import (  # type: ignore  # noqa: E402
    SingleLegTestHarness,
    encode_set_controller_mode,
    encode_set_input_torque,
    encode_set_state,
    encode_set_vel_curr_limits,
    error_names,
)


# ---------------------------------------------------------------------------
# Caps
# ---------------------------------------------------------------------------

HARD_IQ_CAP_A = 3.0                 # never command above this
HARD_CURRENT_LIMIT_A = 10.0         # ODrive soft-limit (matches cogging_bench)
HARD_POSITION_CAP_REV = 3.0         # bench leg full stroke
PULSE_POS_LIMIT_REV = 0.5           # per-pulse: stop if rotor drifts beyond this
HEARTBEAT_STALE_S = 1.5

# Per-pulse termination (whichever fires first)
PULSE_MAX_S = 0.6                   # absolute time cap per pulse (~0.5 rev at 1 rps)
TERMINAL_DV_DT_THRESHOLD_RPS_PER_S = 1.0  # |Δvel/Δt| below this = terminal
TERMINAL_WINDOW_S = 0.05            # window over which we average vel for terminal check
TERMINAL_MIN_HOLD_S = 0.05          # minimum pulse duration before checking terminal

# Soft ramp-on at start of each pulse (avoid step torque)
TORQUE_RAMP_S = 0.02                # 20 ms — fast enough to be a near-step

# Inter-step IDLE so rotor can re-settle in detent
INTER_STEP_REST_S = 1.0

# Default iq sweep (small to large)
DEFAULT_IQ_LEVELS = (0.05, 0.10, 0.15, 0.20, 0.30, 0.50, 0.75, 1.00, 1.50, 2.00)


@dataclass
class StepResult:
    iq_set_A: float
    direction: str
    v_terminal_rps: float
    v_max_rps: float
    pulse_duration_s: float
    displacement_rev: float
    outcome: str   # 'terminal' | 'pos_cap' | 'time_cap' | 'aborted'


class TorqueStepBench:
    def __init__(self, axis_id: int, interface: str, channel: str,
                 iq_levels: List[float], output_path: str,
                 record_trace: bool, traces_path: Optional[str],
                 dry_run: bool):
        self.harness = SingleLegTestHarness(axis_id=axis_id,
                                            interface=interface, channel=channel)
        self.iq_levels = iq_levels
        self.output_path = output_path
        self.record_trace = record_trace
        self.traces_path = traces_path
        self.dry_run = dry_run

        self._aborted = False
        self._abort_reason = ''
        self._session_start_pos_raw: Optional[float] = None
        self._results: List[StepResult] = []
        # Each trace row: (step_idx, direction, t_s, iq_cmd, iq_meas, pos_raw, vel_raw)
        self._traces: List[tuple] = []

    def validate(self) -> Optional[str]:
        if not self.iq_levels:
            return 'no iq levels given'
        for iq in self.iq_levels:
            if iq <= 0 or iq > HARD_IQ_CAP_A:
                return f'iq level {iq} outside (0, {HARD_IQ_CAP_A}] A'
        return None

    def _session_safety_ok(self) -> bool:
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
                self._abort_reason = (f'session pos cap exceeded: '
                                       f'cumulative {cumulative:.3f} rev')
                return False
        return True

    def _enter_torque_mode(self):
        h = self.harness
        h.send(encode_set_vel_curr_limits(
            h.axis_id,
            vel_limit=20.0,        # ODrive vel cap; we only ever apply small torques
            curr_limit=HARD_CURRENT_LIMIT_A))
        time.sleep(0.05)
        h.send(encode_set_controller_mode(h.axis_id, 'TORQUE', 'PASSTHROUGH'))
        time.sleep(0.05)
        h.send(encode_set_input_torque(h.axis_id, 0.0))
        time.sleep(0.05)
        h.send(encode_set_state(h.axis_id, 'CLOSED_LOOP'))
        h._wait_for_closed_loop()  # type: ignore[attr-defined]

    def _idle(self):
        self.harness.send(encode_set_state(self.harness.axis_id, 'IDLE'))

    def _soft_ramp_torque(self, target_torque_A: float, current_torque_A: float):
        """Linearly ramp commanded torque from current to target over TORQUE_RAMP_S."""
        h = self.harness
        steps = max(2, int(TORQUE_RAMP_S / 0.005))
        for i in range(1, steps + 1):
            frac = i / steps
            torque = current_torque_A + frac * (target_torque_A - current_torque_A)
            h.send(encode_set_input_torque(h.axis_id, torque))
            time.sleep(TORQUE_RAMP_S / steps)

    def _run_pulse(self, step_idx: int, iq_target: float,
                   direction: str) -> StepResult:
        """Run one torque pulse, return when terminal vel detected or guard fires."""
        h = self.harness
        sign = +1.0 if direction == 'forward' else -1.0
        iq_signed = sign * iq_target

        # Settle
        h._poll()  # type: ignore[attr-defined]
        if not self._session_safety_ok():
            return StepResult(iq_target, direction, 0, 0, 0, 0, 'aborted')
        start_pos = h.state.pos_rev_raw

        # Soft ramp-on (avoid step torque)
        self._soft_ramp_torque(iq_signed, 0.0)

        # Monitor pulse
        t0 = time.time()
        last_log = t0
        log_period = 0.005   # 200 Hz logging during pulse
        # Rolling-window vel buffer
        vel_buf: List[tuple] = []  # (t, vel)
        v_max = 0.0
        outcome = 'time_cap'
        end_t = t0 + PULSE_MAX_S

        while time.time() < end_t:
            h._poll(timeout=0.001)  # type: ignore[attr-defined]
            now = time.time()
            t_rel = now - t0

            disp = abs(h.state.pos_rev_raw - start_pos)
            v = h.state.vel_rps_raw

            if abs(v) > abs(v_max):
                v_max = v

            # Position guard
            if disp > PULSE_POS_LIMIT_REV:
                outcome = 'pos_cap'
                break

            # Session safety
            if not self._session_safety_ok():
                outcome = 'aborted'
                break

            # Trace logging
            if self.record_trace and now - last_log >= log_period:
                self._traces.append((step_idx, direction, t_rel,
                                     iq_signed, float(h.state.iq_measured),
                                     float(h.state.pos_rev_raw),
                                     float(v)))
                last_log = now

            # Terminal-velocity detection
            vel_buf.append((t_rel, v))
            # keep only the last TERMINAL_WINDOW_S of samples
            cutoff = t_rel - TERMINAL_WINDOW_S
            vel_buf = [(t, vv) for t, vv in vel_buf if t >= cutoff]
            if t_rel >= TERMINAL_MIN_HOLD_S and len(vel_buf) >= 4:
                # Fit linear regression vel = a + b*t over the window; b is dvel/dt
                ts = [x[0] for x in vel_buf]
                vs = [x[1] for x in vel_buf]
                t_mean = sum(ts) / len(ts)
                v_mean = sum(vs) / len(vs)
                num = sum((t - t_mean) * (v - v_mean) for t, v in zip(ts, vs))
                den = sum((t - t_mean) ** 2 for t in ts)
                slope = num / den if den > 1e-9 else 0.0
                if abs(slope) < TERMINAL_DV_DT_THRESHOLD_RPS_PER_S:
                    outcome = 'terminal'
                    break

        # Read final state, then ramp torque off softly
        h._poll()  # type: ignore[attr-defined]
        v_terminal = h.state.vel_rps_raw
        pulse_duration = time.time() - t0
        displacement = abs(h.state.pos_rev_raw - start_pos)
        self._soft_ramp_torque(0.0, iq_signed)

        return StepResult(
            iq_set_A=iq_target,
            direction=direction,
            v_terminal_rps=round(float(v_terminal), 4),
            v_max_rps=round(float(v_max), 4),
            pulse_duration_s=round(pulse_duration, 4),
            displacement_rev=round(displacement, 5),
            outcome=outcome,
        )

    def run(self) -> int:
        err = self.validate()
        if err:
            print(f'REJECT: {err}', file=sys.stderr)
            return 1

        print('Torque-step terminal-velocity test')
        print(f'  axis_id        = {self.harness.axis_id}')
        print(f'  iq levels      = {self.iq_levels} A  (n={len(self.iq_levels)})')
        print(f'  output         = {self.output_path}')
        if self.traces_path:
            print(f'  traces         = {self.traces_path}')
        print(f'  per-pulse caps = pos≤{PULSE_POS_LIMIT_REV} rev, '
              f'time≤{PULSE_MAX_S} s, '
              f'terminal at |dv/dt|<{TERMINAL_DV_DT_THRESHOLD_RPS_PER_S} rps²')
        print(f'  session caps   = pos≤{HARD_POSITION_CAP_REV} rev, '
              f'iq≤{HARD_IQ_CAP_A} A')

        if self.dry_run:
            print('\nDRY-RUN: validation passed, no CAN I/O performed.')
            return 0

        os.makedirs(os.path.dirname(os.path.abspath(self.output_path)), exist_ok=True)

        signal.signal(signal.SIGINT,
                      lambda *_: setattr(self, '_aborted', True)
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

                self._enter_torque_mode()

                for step_idx, iq in enumerate(self.iq_levels):
                    if self._aborted:
                        print(f'\nAborted: {self._abort_reason}', file=sys.stderr)
                        break
                    for direction in ('forward', 'reverse'):
                        if self._aborted:
                            break
                        print(f'  Step {step_idx:2d} iq={iq:.3f} A {direction:7s}: ',
                              end='', flush=True)
                        res = self._run_pulse(step_idx, iq, direction)
                        self._results.append(res)
                        print(f'v_term={res.v_terminal_rps:+7.3f} rps  '
                              f'v_max={res.v_max_rps:+7.3f} rps  '
                              f'Δpos={res.displacement_rev*1000:+7.1f} mrev  '
                              f't={res.pulse_duration_s:5.3f} s  '
                              f'[{res.outcome}]')
                    # Inter-step rest (IDLE → settle)
                    self._idle()
                    time.sleep(INTER_STEP_REST_S)
                    self._enter_torque_mode()

                self._idle()
        except Exception as exc:
            print(f'\nABORT: {exc}', file=sys.stderr)
            self._idle()
            return 2

        with open(self.output_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['step', 'iq_set_A', 'direction', 'v_terminal_rps',
                        'v_max_rps', 'pulse_duration_s', 'displacement_rev',
                        'outcome'])
            for i, r in enumerate(self._results):
                w.writerow([i, r.iq_set_A, r.direction, r.v_terminal_rps,
                            r.v_max_rps, r.pulse_duration_s, r.displacement_rev,
                            r.outcome])
        if self.record_trace and self.traces_path:
            with open(self.traces_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['step', 'direction', 't_s', 'iq_cmd_A',
                            'iq_meas_A', 'pos_rev_raw', 'vel_rps_raw'])
                for row in self._traces:
                    w.writerow([row[0], row[1], f'{row[2]:.4f}',
                                f'{row[3]:.4f}', f'{row[4]:.4f}',
                                f'{row[5]:.6f}', f'{row[6]:.6f}'])

        print(f'\nDone. {len(self._results)} pulses → {self.output_path}')
        return 0


def _timestamp_slug() -> str:
    import datetime
    return datetime.datetime.now().strftime('%Y%m%d_%H%M%S')


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--axis', type=int, default=0)
    p.add_argument('--interface', default='socketcan')
    p.add_argument('--channel', default='can0')
    p.add_argument('--iq-levels',
                   default=','.join(f'{x:g}' for x in DEFAULT_IQ_LEVELS),
                   help=f'comma-separated iq levels in A '
                        f'(default: {",".join(f"{x:g}" for x in DEFAULT_IQ_LEVELS)})')
    p.add_argument('--output', default=None,
                   help='summary CSV path (default: temp/logs/torque_step_<ts>.csv)')
    p.add_argument('--record-trace', action='store_true',
                   help='also dump per-sample trace CSV (debugging)')
    p.add_argument('--dry-run', action='store_true')
    args = p.parse_args()

    try:
        iq_levels = [float(x) for x in args.iq_levels.split(',')]
    except ValueError:
        print(f'ERROR: --iq-levels needs comma-separated floats, '
              f'got {args.iq_levels!r}', file=sys.stderr)
        return 1

    output = args.output or os.path.join(_PROJECT_ROOT, 'temp', 'logs',
                                          f'torque_step_{_timestamp_slug()}.csv')
    traces = (output.replace('.csv', '_traces.csv')
              if args.record_trace else None)

    bench = TorqueStepBench(
        axis_id=args.axis, interface=args.interface, channel=args.channel,
        iq_levels=iq_levels, output_path=output,
        record_trace=args.record_trace, traces_path=traces,
        dry_run=args.dry_run)
    return bench.run()


if __name__ == '__main__':
    sys.exit(main())
