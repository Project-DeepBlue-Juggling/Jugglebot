#!/usr/bin/env python3
"""Cogging-torque bench test for a single Jugglebot leg motor.

Drives one ODrive axis at a constant, slow velocity and logs
(time, pos_rev, vel_rps, iq_setpoint, iq_measured) so the offline analyser
can build a cogging-torque map indexed by electrical angle.

Why this test
-------------
At very low velocity the motor is in the *cogging-dominated* regime: the
mechanical inertia term ``J·α`` is negligible, the viscous term ``b·ω`` is
negligible, and the Coulomb/stiction term ``τ_f`` is roughly constant.
Whatever current ``iq`` the controller is fighting to hold the target
velocity is therefore ``iq(θ) ≈ (τ_cogging(θ) + τ_f) / Kt`` — i.e. a
constant offset plus the cogging ripple we care about.  Averaging ``iq``
into electrical-angle bins across many revolutions reveals the cogging
map directly.

Safety profile
--------------
This test is designed for a bench rig with NO brake resistor and a small
PSU: every movement is slow, every movement is bounded, and any fault
(heartbeat loss, ODrive error, Ctrl-C) puts the axis in IDLE before the
script exits.

* HARD_VEL_CAP_RPS      — reject any ``--velocity`` above this
* HARD_DURATION_CAP_S   — reject any ``--duration`` above this
* HARD_CURRENT_LIMIT_A  — current soft-limit applied to the ODrive
* HARD_POSITION_CAP_REV — abort if displacement exceeds this in either direction

Usage
-----
    # Simulated sanity-check (no hardware needed)
    python tests/hardware/cogging_bench_test.py --dry-run

    # Default: +0.05 rev/s for 25 s (captures ~1.25 mech revs → plenty)
    python tests/hardware/cogging_bench_test.py

    # Reverse direction at the same speed
    python tests/hardware/cogging_bench_test.py --velocity -0.05

    # Custom runtime, output path, axis id
    python tests/hardware/cogging_bench_test.py --velocity 0.03 --duration 40 \\
                                                --output /tmp/cogging_slow.csv

    # Back-to-back forward then reverse (single invocation, two runs)
    python tests/hardware/cogging_bench_test.py --bidirectional

Output
------
CSV columns::

    t_s, pos_rev_raw, pos_rev_inv, vel_rps_raw, vel_rps_inv,
    iq_setpoint_A, iq_measured_A,
    elec_angle_rad, mech_angle_rad,
    active_errors

``pos_rev_raw`` / ``vel_rps_raw`` are the unmodified ODrive values;
``pos_rev_inv`` / ``vel_rps_inv`` apply the Jugglebot "positive = extension"
inversion.  Electrical / mechanical angle are derived offline from
``pos_rev_raw × 2π × pole_pairs`` mod 2π (pole_pairs read from
``hardware_config.yaml``).

Exit codes
----------
0 — normal completion, CSV written
1 — pre-flight rejection (bad args, heartbeat timeout, ODrive in error)
2 — mid-run fault (heartbeat loss, error bit, position excursion)
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time
from typing import Optional

import can

# ---------------------------------------------------------------------------
# Reuse the single_leg_test harness for CAN I/O & state tracking.
# ---------------------------------------------------------------------------

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
sys.path.insert(0, _SCRIPT_DIR)

from single_leg_test import (  # type: ignore  # noqa: E402
    SingleLegTestHarness,
    encode_set_controller_mode,
    encode_set_input_vel,
    encode_set_vel_curr_limits,
    encode_set_state,
    error_names,
)

# ---------------------------------------------------------------------------
# Safety caps — conservative by design.  A bench rig with no brake resistor
# cannot dissipate regen energy; the only way to stay safe is to move slowly.
# ---------------------------------------------------------------------------

HARD_VEL_CAP_RPS = 35.0         # rev/s — bench rig now has the platform's brake
                                # resistor attached (2026-04-27), so regen energy
                                # at decel can be safely dumped.  35 rev/s on the
                                # bench leg's 71.57 mm/rev geometry = 2.5 m/s of
                                # leg-end velocity, which is the user-stated
                                # mechanical maximum the bench rig can survive.
                                # Was 0.5 rev/s under the original no-brake
                                # configuration.  This cap should NOT be applied
                                # to platform legs — operationally the platform
                                # never goes anywhere near this speed because of
                                # other (mechanical) downstream concerns.
HARD_DURATION_CAP_S = 120.0     # two minutes — plenty for any cogging sweep
HARD_CURRENT_LIMIT_A = 10.0     # bench rig: brake-resistor protected, sized to
                                # the platform's full operational draw.  10 A is
                                # above the motor's inertial-accel demand at the
                                # raised acceleration cap (~7 A at 250 rev/s²),
                                # leaving margin for friction overhead.  Was 5 A
                                # previously; raised to permit higher acceleration
                                # so high-velocity sweeps can complete inside the
                                # leg's 3 rev stroke.  Still well below the
                                # platform's 20 A nominal limit.
HARD_POSITION_CAP_REV = 3.0     # rev — the bench leg's full mechanical stroke
                                # is 3 rev of motor rotation.  This cap MUST NEVER
                                # be exceeded: doing so means the leg has crashed
                                # into one of its end-stops, which can damage the
                                # ballscrew, the cable termination, or the motor.
                                # The cap is hard-enforced two ways: (1) pre-flight
                                # rejection if predicted displacement (|v|·duration)
                                # exceeds it, and (2) mid-run abort that drops the
                                # axis to IDLE the instant runtime displacement
                                # crosses it.  Per-run --position-cap-rev can lower
                                # this for extra-cautious runs but cannot raise it.
HARD_ACCEL_CAP_RPS2 = 250.0     # rev/s² — soft-start / soft-stop ramp.
                                # Sized so the 10 rev/s target reaches steady
                                # state in 0.04 s, leaving ~0.17 s of cruise
                                # within the 3 rev stroke (the leg's full
                                # mechanical travel — see HARD_POSITION_CAP_REV).
                                # Achievable with the 10 A current limit:
                                #   torque = J × α = 2.75e-4 × 250 × 2π ≈ 0.43 Nm
                                #   current = torque / Kt = 0.43 / 0.062 ≈ 7.0 A
                                # 3 A of margin on top for friction overhead and
                                # leg-load reflected inertia.  Was 100 rev/s²
                                # previously — bumped because 10 rev/s tests
                                # otherwise can't fit within the 3 rev stroke.

# Auto-duration safety margin — predicted_displacement = |v|·duration + ramp_overhead
# stays below HARD_POSITION_CAP_REV × this fraction.  Leaves headroom for any
# per-tick overshoot or PID slip during the run.
AUTO_DURATION_POSITION_FRAC = 0.85

# Heartbeat watchdog: if no heartbeat for this long, abort and IDLE
HEARTBEAT_STALE_S = 1.5

# Default test parameters
DEFAULT_VELOCITY_RPS = 0.05
DEFAULT_DURATION_S = 25.0
DEFAULT_LOG_RATE_HZ = 100.0     # ODrive broadcasts iq/encoder at 100 Hz

# Soft-start: ramp velocity in over this many seconds at the start/end.
# Functions as a *minimum* ramp time at low velocities — the actual ramp
# accel is bounded by HARD_ACCEL_CAP_RPS2, so at higher target velocities
# the ramp time grows to |v_target| / HARD_ACCEL_CAP_RPS2 instead.
#
# At RAMP_S=0.05, low-velocity runs (e.g. 0.075 rev/s) have a ramp_rate
# of 1.5 rev/s² — gentle enough to avoid stiction-release jolt — while
# high-velocity runs (10 rev/s) reach target in 0.04 s, leaving usable
# cruise time within the 3 rev stroke.
# Was 0.3 s previously — shortened for the same reason as HARD_ACCEL_CAP_RPS2:
# 10 rev/s tests need to fit inside 3 rev of motion.
RAMP_S = 0.05


def _clamp(v: float, lo: float, hi: float) -> str:
    if v < lo:
        return f"{v} < {lo}"
    if v > hi:
        return f"{v} > {hi}"
    return ""


def _load_pole_pairs() -> Optional[int]:
    """Read motor_pole_pairs from hardware_config.yaml.  None if unavailable."""
    try:
        import yaml  # type: ignore
    except Exception:
        return None
    path = os.path.join(_PROJECT_ROOT, 'config', 'hardware_config.yaml')
    try:
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)
        return cfg.get('dynamics', {}).get('motor_pole_pairs')
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Bench runner
# ---------------------------------------------------------------------------

class CoggingBench:
    """One-shot velocity-mode runner with CSV logging + hard safety caps."""

    def __init__(self, axis_id: int, interface: str, channel: str,
                 velocity_rps: float, duration_s: float, output_path: str,
                 log_rate_hz: float, dry_run: bool,
                 position_cap_rev: float = HARD_POSITION_CAP_REV):
        self.velocity_rps = velocity_rps
        self.duration_s = duration_s
        self.output_path = output_path
        self.log_period_s = 1.0 / log_rate_hz
        self.dry_run = dry_run
        self.position_cap_rev = position_cap_rev
        self.pole_pairs = _load_pole_pairs() or 7
        self.harness = SingleLegTestHarness(axis_id=axis_id,
                                            interface=interface,
                                            channel=channel)
        self._aborted = False
        self._abort_reason = ''
        self._initial_pos_raw: Optional[float] = None

    # -- pre-flight / safety -------------------------------------------------

    def validate_args(self) -> Optional[str]:
        if err := _clamp(abs(self.velocity_rps), 0.001, HARD_VEL_CAP_RPS):
            return f"|velocity| out of range: {err}"
        # Duration floor is dynamic: must allow 2× ramp_time plus a minimum
        # cruise window of 0.05 s (5 samples at the 100 Hz log rate).
        # Using a fixed floor here would either be too generous at high-v
        # (rejecting runs the cap actually permits) or too tight at low-v
        # (allowing all-ramp runs that yield zero useful steady-state data).
        v = abs(self.velocity_rps)
        ramp_time = max(RAMP_S, v / HARD_ACCEL_CAP_RPS2)
        min_duration = 2.0 * ramp_time + 0.05
        if self.duration_s < min_duration:
            achievable_rev = self.position_cap_rev
            needed_for_min = min_duration * v / AUTO_DURATION_POSITION_FRAC
            return (f"duration {self.duration_s:.3f} s below minimum "
                    f"{min_duration:.3f} s "
                    f"(2× {ramp_time:.3f} s ramp + 0.05 s cruise).  "
                    f"At |v|={v:.3f} rev/s and position cap "
                    f"{achievable_rev:.2f} rev, the cap-derived max duration "
                    f"is {achievable_rev * AUTO_DURATION_POSITION_FRAC / v:.3f} s.  "
                    f"Either reduce |v| or — only if the rig has the stroke "
                    f"headroom — raise the cap to ≥ {needed_for_min:.2f} rev.")
        if self.duration_s > HARD_DURATION_CAP_S:
            return f"duration {self.duration_s} > hard ceiling {HARD_DURATION_CAP_S} s"
        predicted_rev = v * self.duration_s
        if predicted_rev > self.position_cap_rev:
            return (f"velocity × duration = {predicted_rev:.2f} rev "
                    f"exceeds position cap {self.position_cap_rev} rev "
                    f"— shorten --duration")
        return None

    def _check_safety(self) -> bool:
        """Return False if we should abort.  Sets self._abort_reason."""
        # Heartbeat freshness
        age = time.time() - self.harness.state.last_heartbeat
        if age > HEARTBEAT_STALE_S:
            self._abort_reason = f"heartbeat stale ({age*1000:.0f} ms)"
            return False
        # Error bits
        if self.harness.state.has_errors:
            self._abort_reason = f"ODrive errors: {error_names(self.harness.state.active_errors)}"
            return False
        # Position excursion from start
        if self._initial_pos_raw is not None:
            displacement = self.harness.state.pos_rev_raw - self._initial_pos_raw
            if abs(displacement) > self.position_cap_rev:
                self._abort_reason = (f"position cap exceeded: "
                                       f"displacement {displacement:+.3f} rev")
                return False
        return True

    # -- mode entry / exit ---------------------------------------------------

    def enter_velocity_mode(self):
        h = self.harness
        h.clear_errors()
        h.send(encode_set_vel_curr_limits(
            h.axis_id,
            vel_limit=HARD_VEL_CAP_RPS * 1.2,
            curr_limit=HARD_CURRENT_LIMIT_A))
        time.sleep(0.05)
        h.send(encode_set_controller_mode(h.axis_id, 'VELOCITY', 'PASSTHROUGH'))
        time.sleep(0.05)
        # Seed zero velocity before entering closed loop
        h.send(encode_set_input_vel(h.axis_id, 0.0))
        time.sleep(0.05)
        h.send(encode_set_state(h.axis_id, 'CLOSED_LOOP'))
        h._wait_for_closed_loop()  # type: ignore[attr-defined]
        print(f"  Axis in VELOCITY/PASSTHROUGH, CLOSED_LOOP")

    # -- the measurement ----------------------------------------------------

    def _ramped_velocity(self, t: float) -> float:
        """Soft-start / soft-stop velocity profile.

        Linear ramp in over RAMP_S, constant in the middle, linear ramp out
        over RAMP_S at the end.  Ramp rate bounded by HARD_ACCEL_CAP_RPS2.
        """
        v_target = self.velocity_rps
        ramp_rate = abs(v_target) / RAMP_S
        if ramp_rate > HARD_ACCEL_CAP_RPS2:
            ramp_rate = HARD_ACCEL_CAP_RPS2
        sign = 1.0 if v_target >= 0 else -1.0
        v_mag_target = abs(v_target)
        if t < RAMP_S:
            return sign * min(v_mag_target, ramp_rate * t)
        if t > self.duration_s - RAMP_S:
            t_remaining = max(0.0, self.duration_s - t)
            return sign * min(v_mag_target, ramp_rate * t_remaining)
        return v_target

    def run(self) -> int:
        err = self.validate_args()
        if err:
            print(f"REJECT: {err}", file=sys.stderr)
            return 1

        print(f"Cogging bench test")
        print(f"  axis_id       = {self.harness.axis_id}")
        print(f"  velocity      = {self.velocity_rps:+.4f} rev/s "
              f"({'reverse' if self.velocity_rps < 0 else 'forward'})")
        print(f"  duration      = {self.duration_s:.1f} s "
              f"(→ ~{abs(self.velocity_rps) * self.duration_s:.2f} rev)")
        print(f"  log rate      = {1.0 / self.log_period_s:.0f} Hz")
        print(f"  output        = {self.output_path}")
        print(f"  pole_pairs    = {self.pole_pairs}")
        print(f"  safety caps   = vel≤{HARD_VEL_CAP_RPS} rev/s, "
              f"curr≤{HARD_CURRENT_LIMIT_A} A, "
              f"pos≤±{self.position_cap_rev} rev"
              f"{' (default)' if self.position_cap_rev == HARD_POSITION_CAP_REV else ' (overridden)'}")
        # Compute predicted ramp time: max(RAMP_S, |v_target| / accel_cap)
        ramp_time = max(RAMP_S, abs(self.velocity_rps) / HARD_ACCEL_CAP_RPS2)
        cruise_time = self.duration_s - 2.0 * ramp_time
        if cruise_time < 0.3:
            print(f"  WARNING: cruise time = {cruise_time:.2f} s — run is mostly ramp; "
                  f"raise --position-cap-rev for more steady-state data")

        if self.dry_run:
            print("\nDRY-RUN: validation passed, no CAN I/O performed.")
            return 0

        rc = 0
        out_dir = os.path.dirname(os.path.abspath(self.output_path))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        def sigint_handler(signum, frame):
            print("\n[SIGINT] requesting abort...")
            self._aborted = True
            self._abort_reason = 'SIGINT'
        signal.signal(signal.SIGINT, sigint_handler)

        try:
            with self.harness as h:
                print(f"\n  Heartbeat OK (state: {h.state.state_name})")
                if h.state.has_errors:
                    print(f"  Pre-existing errors: {error_names(h.state.active_errors)}")
                    print(f"  Clearing...")
                    h.clear_errors()
                    if h.state.has_errors:
                        print(f"  REJECT: errors persist after clear")
                        return 1

                self.enter_velocity_mode()
                h._poll()  # type: ignore[attr-defined]
                self._initial_pos_raw = h.state.pos_rev_raw
                print(f"  Starting pos_raw = {self._initial_pos_raw:+.4f} rev")

                rc = self._drive_and_log()

        except KeyboardInterrupt:
            print("\n[SIGINT in setup]")
            return 2
        except Exception as exc:
            print(f"\nABORT during setup: {exc}", file=sys.stderr)
            return 1

        if rc == 0:
            print(f"\nDone.  CSV: {self.output_path}")
        else:
            print(f"\nAborted: {self._abort_reason}", file=sys.stderr)
        return rc

    def _drive_and_log(self) -> int:
        h = self.harness
        sample_count = 0
        rc = 0
        two_pi = 2.0 * math.pi

        with open(self.output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                't_s', 'pos_rev_raw', 'pos_rev_inv',
                'vel_rps_raw', 'vel_rps_inv',
                'iq_setpoint_A', 'iq_measured_A',
                'elec_angle_rad', 'mech_angle_rad',
                'active_errors',
                'cmd_vel_rps',
            ])

            t_start = time.time()
            next_log_t = t_start
            t_end = t_start + self.duration_s

            last_cmd_vel = 0.0
            h.send(encode_set_input_vel(h.axis_id, 0.0))

            while time.time() < t_end:
                if self._aborted:
                    rc = 2
                    break

                # Poll CAN (non-blocking) — updates state from broadcasts
                h._poll(timeout=0.001)  # type: ignore[attr-defined]

                if not self._check_safety():
                    rc = 2
                    break

                now = time.time()
                t_rel = now - t_start

                # Re-command velocity at every iteration — ODrive treats
                # velocity commands as persistent, but re-sending keeps the
                # effective command fresh and visible in the log.
                cmd_v = self._ramped_velocity(t_rel)
                if abs(cmd_v - last_cmd_vel) > 1e-4 or (now - next_log_t) > 0.5:
                    h.send(encode_set_input_vel(h.axis_id, cmd_v))
                    last_cmd_vel = cmd_v

                # Log at log_rate_hz
                if now >= next_log_t:
                    pos_raw = h.state.pos_rev_raw
                    mech = (pos_raw * two_pi) % two_pi
                    elec = (pos_raw * self.pole_pairs * two_pi) % two_pi
                    writer.writerow([
                        f"{t_rel:.4f}",
                        f"{pos_raw:.6f}",
                        f"{h.state.pos_rev:.6f}",
                        f"{h.state.vel_rps_raw:.6f}",
                        f"{h.state.vel_rps:.6f}",
                        f"{h.state.iq_setpoint:.4f}",
                        f"{h.state.iq_measured:.4f}",
                        f"{elec:.4f}",
                        f"{mech:.4f}",
                        int(h.state.active_errors),
                        f"{cmd_v:.4f}",
                    ])
                    sample_count += 1
                    next_log_t += self.log_period_s
                    # If we fell behind (e.g. GC pause), don't burst-catch-up
                    if next_log_t < now:
                        next_log_t = now + self.log_period_s

            # Graceful stop: command zero velocity, wait for mechanical settle
            print(f"  Commanding zero velocity...")
            h.send(encode_set_input_vel(h.axis_id, 0.0))
            time.sleep(0.5)
            h._poll()  # type: ignore[attr-defined]

        print(f"  Logged {sample_count} samples "
              f"({sample_count / max(1e-6, time.time() - t_start):.0f} Hz effective)")
        return rc


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _timestamp_slug() -> str:
    import datetime
    return datetime.datetime.now().strftime('%Y%m%d_%H%M%S')


def _safe_duration_for(velocity_rps: float, target_duration_s: float,
                       position_cap_rev: float = HARD_POSITION_CAP_REV) -> float:
    """Cap target_duration so |v|·duration ≤ position_cap_rev × frac.

    The position cap is the binding constraint — never violated.  No
    minimum-duration floor here: if the cap forces the duration below the
    validate_args floor (0.5 s), validation will reject with a message
    pointing the operator at --position-cap-rev.  This is the right
    behaviour: it's better to refuse a run than to silently violate the
    position cap by enforcing a duration floor that contradicts it.
    """
    v = max(1e-4, abs(velocity_rps))
    safe = position_cap_rev * AUTO_DURATION_POSITION_FRAC / v
    return min(target_duration_s, safe)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--axis', type=int, default=0,
                        help='CAN node id of the ODrive (default: 0)')
    parser.add_argument('--interface', default='socketcan',
                        help='python-can interface (default: socketcan)')
    parser.add_argument('--channel', default='can0',
                        help='CAN channel (default: can0)')
    parser.add_argument('--velocity', type=float, default=DEFAULT_VELOCITY_RPS,
                        help=f'target motor velocity in rev/s (sign = direction, '
                             f'|v| ≤ {HARD_VEL_CAP_RPS}). Default: {DEFAULT_VELOCITY_RPS}')
    parser.add_argument('--direction', choices=['forward', 'reverse'], default=None,
                        help='if given, force direction (sign of --velocity ignored)')
    parser.add_argument('--duration', type=float, default=DEFAULT_DURATION_S,
                        help=f'run duration in seconds (≤ {HARD_DURATION_CAP_S}). '
                             f'Default: {DEFAULT_DURATION_S}')
    parser.add_argument('--log-rate', type=float, default=DEFAULT_LOG_RATE_HZ,
                        help=f'CSV log rate in Hz (default: {DEFAULT_LOG_RATE_HZ})')
    parser.add_argument('--output', default=None,
                        help='output CSV path (default: temp/logs/cogging_<ts>.csv)')
    parser.add_argument('--bidirectional', action='store_true',
                        help='run forward then reverse back-to-back (two CSVs)')
    parser.add_argument('--auto-duration', action='store_true',
                        help=f'cap --duration to keep predicted displacement under '
                             f'{AUTO_DURATION_POSITION_FRAC*100:.0f}% of position cap '
                             f'(per-velocity sizing)')
    parser.add_argument('--position-cap-rev', type=float,
                        default=HARD_POSITION_CAP_REV,
                        help=f'lower the per-run position-cap (default: '
                             f'{HARD_POSITION_CAP_REV} rev = bench leg full stroke). '
                             f'Cannot raise above {HARD_POSITION_CAP_REV} rev — the '
                             f'leg has only that much stroke before hitting an end-stop.')
    parser.add_argument('--sweep', default=None,
                        help='comma-separated list of |velocities| in rev/s — runs each '
                             'one bidirectionally, sized via --auto-duration. '
                             'Example: --sweep 0.075,0.10,0.20,0.30,0.40')
    parser.add_argument('--dry-run', action='store_true',
                        help='validate args and print config, do not touch CAN')
    args = parser.parse_args()

    velocity = abs(args.velocity) * (-1.0 if args.direction == 'reverse'
                                     else 1.0 if args.direction == 'forward'
                                     else 1.0 if args.velocity >= 0 else -1.0)

    def _default_output(tag: str) -> str:
        return os.path.join(_PROJECT_ROOT, 'temp', 'logs',
                            f'cogging_{_timestamp_slug()}_{tag}.csv')

    # --- sweep mode: bidirectional runs at each listed velocity, auto-sized
    pos_cap = args.position_cap_rev
    if pos_cap <= 0:
        print(f'ERROR: --position-cap-rev {pos_cap} must be positive',
              file=sys.stderr)
        return 1
    if pos_cap > HARD_POSITION_CAP_REV:
        print(f'ERROR: --position-cap-rev {pos_cap} above bench-leg stroke '
              f'{HARD_POSITION_CAP_REV} rev — exceeding it would crash the leg '
              f'into an end-stop',
              file=sys.stderr)
        return 1

    if args.sweep:
        try:
            sweep_vels = [abs(float(v)) for v in args.sweep.split(',')]
        except ValueError:
            print(f'ERROR: --sweep needs comma-separated floats, got {args.sweep!r}',
                  file=sys.stderr)
            return 1
        if not sweep_vels:
            print('ERROR: --sweep is empty', file=sys.stderr)
            return 1
        # Validate all velocities BEFORE touching hardware so a bad value stops
        # the entire sweep before any motion.
        for v in sweep_vels:
            if v <= 0 or v > HARD_VEL_CAP_RPS:
                print(f'ERROR: sweep velocity {v} out of range '
                      f'(0, {HARD_VEL_CAP_RPS}]', file=sys.stderr)
                return 1
        print(f'=== Sweep: {len(sweep_vels)} velocities, '
              f'pos_cap={pos_cap} rev, '
              f'{"DRY-RUN" if args.dry_run else "live"} ===')
        for v in sweep_vels:
            dur = _safe_duration_for(v, args.duration, pos_cap)
            disp = v * dur
            print(f'\n--- |v|={v:.3f} rev/s, duration={dur:.1f} s '
                  f'(predicted displacement {disp:.2f} rev) ---')
            tag_fwd = f'fwd_{v:.3f}rps'
            tag_rev = f'rev_{v:.3f}rps'
            out_fwd = _default_output(tag_fwd)
            out_rev = _default_output(tag_rev)
            rc = CoggingBench(args.axis, args.interface, args.channel,
                              +v, dur, out_fwd, args.log_rate, args.dry_run,
                              position_cap_rev=pos_cap).run()
            if rc != 0:
                return rc
            if not args.dry_run:
                time.sleep(2.0)
            rc = CoggingBench(args.axis, args.interface, args.channel,
                              -v, dur, out_rev, args.log_rate, args.dry_run,
                              position_cap_rev=pos_cap).run()
            if rc != 0:
                return rc
            if not args.dry_run:
                time.sleep(2.0)
        return 0

    if args.auto_duration:
        capped = _safe_duration_for(velocity, args.duration, pos_cap)
        if capped < args.duration:
            print(f'  --auto-duration: capped {args.duration:.1f} s → {capped:.1f} s '
                  f'(predicted {abs(velocity)*capped:.2f} rev)')
        args.duration = capped

    if args.bidirectional:
        print("=== Bidirectional run: forward → reverse ===")
        tag_fwd = f'fwd_{abs(velocity):.3f}rps'
        tag_rev = f'rev_{abs(velocity):.3f}rps'
        out_fwd = args.output.replace('.csv', f'_{tag_fwd}.csv') \
            if args.output else _default_output(tag_fwd)
        out_rev = args.output.replace('.csv', f'_{tag_rev}.csv') \
            if args.output else _default_output(tag_rev)

        rc = CoggingBench(args.axis, args.interface, args.channel,
                          +abs(velocity), args.duration, out_fwd,
                          args.log_rate, args.dry_run,
                          position_cap_rev=pos_cap).run()
        if rc != 0:
            return rc
        if not args.dry_run:
            print("\n  Pausing 2 s between runs...")
            time.sleep(2.0)
        rc = CoggingBench(args.axis, args.interface, args.channel,
                          -abs(velocity), args.duration, out_rev,
                          args.log_rate, args.dry_run,
                          position_cap_rev=pos_cap).run()
        return rc

    output = args.output or _default_output(
        f'{"rev" if velocity < 0 else "fwd"}_{abs(velocity):.3f}rps')
    rc = CoggingBench(args.axis, args.interface, args.channel,
                      velocity, args.duration, output,
                      args.log_rate, args.dry_run,
                      position_cap_rev=pos_cap).run()
    return rc


if __name__ == '__main__':
    sys.exit(main())
