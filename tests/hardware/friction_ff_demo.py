#!/usr/bin/env python3
"""Friction-FF effectiveness demo on the bench leg.

Compares motion-onset latency, settling time, and tracking quality with
vs without a Stribeck friction feedforward, either via the ODrive's
``input_pos`` torque-feedforward channel (POSITION mode — DEFAULT, mirrors
the platform's ``motor_guard``) or the ``input_vel`` channel (VELOCITY
mode — useful for testing the velocity loop in isolation).

Three modes
-----------
``--mode pos_step``  (default — closest to platform conditions)
    32 paired POSITION-mode trapezoid trials.  At 200 Hz, the script
    sends a pre-computed trapezoidal position trajectory to the ODrive
    in POSITION/PASSTHROUGH mode.  Trajectory shape: ramp 0 → v_target
    over ``--ramp-time`` seconds, hold v_target for ``--hold`` seconds,
    ramp v_target → 0.  This reproduces the platform's motion-onset
    dynamics: the ODrive's position loop generates vel_cmd from pos_err,
    which has to grow large enough that the resulting iq overcomes
    friction — *exactly* the dead-time we're trying to fix.  Half the
    trials run with the Stribeck friction feedforward armed via the
    ``torque_ff`` field of ``set_input_pos``; half without.

``--mode vel_step``
    32 paired VELOCITY-mode step trials.  From rest, command a step to
    ``--v-target`` rps (sign alternates per pair) via VELOCITY/PASSTHROUGH.
    Tests the velocity loop's response to a step input with vs without
    torque_ff.  Useful as an isolation test, but velocity-mode response
    is much faster than position-mode response — onset latency in this
    mode hits the ~10 ms encoder-broadcast resolution floor.

``--mode sweep``
    Triangle-wave velocity profile (0 → +v_peak → −v_peak → 0), one
    period with FF off, one with FF on.  Visual side-by-side comparison;
    least rigorous of the three.

For all step modes, the script reports per-condition mean ± std for:

  * motion-onset latency: time from cmd start to first detectable rotor
    motion (|actual_pos − pos_start| > 5 mrev for pos_step, or
    |actual_v| > 5 % of |v_target| for vel_step).  This is the metric
    that mirrors ``sim/analysis/diagnose.py::analyse_motion_onset``.
  * time-to-90 %: time to reach 90 % of (target − start)
  * overshoot %, settling time, steady-state iq

The Stribeck FF
---------------
Default parameters are the Test A v2+v4 fit (R²=0.954)::

    τ_friction(ω) = τ_c + (τ_s − τ_c)·exp(−(|ω|/ω_s)²) + b·|ω|
    τ_c = 1.07 A, τ_s = 1.88 A, ω_s = 0.26 rev/s, b = 0.005 A/(rev/s)

Plus a constant load offset of +0.15 A (direction-independent gravity/preload
on the bench rig — measured as the half-sum of fwd/rev iq).

Sign convention
---------------
On the bench leg, iq_measured has *opposite* sign to vel_rps_raw (the motor's
positive electrical-current direction is opposite to its positive encoder
direction in this build).  ``--ff-sign`` lets you flip the FF polarity if
your hardware has a different convention; default ``-1`` matches the bench
rig empirically.  If the FF makes things WORSE (longer latency), flip this.

Usage
-----
    python tests/hardware/friction_ff_demo.py --dry-run
    python tests/hardware/friction_ff_demo.py                       # default: pos_step
    python tests/hardware/friction_ff_demo.py --v-target 1.0 --trials 6
    python tests/hardware/friction_ff_demo.py --mode vel_step       # legacy, vel-loop only
    python tests/hardware/friction_ff_demo.py --mode sweep --v-peak 0.5
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field
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
    encode_set_input_pos,
    encode_set_input_vel,
    encode_set_state,
    encode_set_vel_curr_limits,
    error_names,
)


# ODrive int16 scaling for set_input_pos's vel_ff and torque_ff fields.
# These match the values flashed into the leg ODrives' config (mirrored in
# protocol_config.yaml as INPUT_SCALE_LEG_VEL / INPUT_SCALE_LEG_TOR).
INPUT_VEL_SCALE = 1000        # 0.001 rev/s per LSB → ±32.7 rev/s range
INPUT_TORQUE_SCALE = 10000    # 0.0001 Nm per LSB → ±3.27 Nm range


def _torque_ff_to_int16(torque_nm: float) -> int:
    """Quantise a torque-FF value (Nm) to int16 ODrive convention with clamp."""
    raw = int(round(torque_nm * INPUT_TORQUE_SCALE))
    return max(-32768, min(32767, raw))


def _vel_ff_to_int16(vel_rps: float) -> int:
    raw = int(round(vel_rps * INPUT_VEL_SCALE))
    return max(-32768, min(32767, raw))


# ---------------------------------------------------------------------------
# Friction-FF model (Stribeck) — parameters from the Test A v2+v4 fit
# ---------------------------------------------------------------------------

TAU_C = 1.094         # A — kinetic Coulomb floor
TAU_S = 1.953         # A — stiction peak (extrapolated v=0)
OMEGA_S = 0.251       # rev/s — Stribeck breakaway speed scale
B_VISC = 0.0173       # A/(rev/s) — viscous slope
LOAD_OFFSET_A = 0.173 # A — direction-independent constant load (bench rig)
KT_NM_PER_A = 0.0570  # motor torque constant (D6374-150Kv) — follows hardware_config.yaml
                      # motor_kt_nm_per_a (measured 2026-07-15, friction-cancelling traverse,
                      # 0.0570 ± 0.0008; supersedes the stiction-biased 0.0624 this demo
                      # originally shipped with)

# Stiction-boost band — see ``friction_ff_nm`` docstring.  When the commanded
# velocity is below this threshold AND non-zero, the FF applies the full
# stiction peak τ_s instead of the Stribeck-tapered value.  This closes
# the FF-to-stiction gap during the early ramp, where the rotor is still
# trying to break free but the Stribeck term has already started tapering
# toward kinetic friction.
#
# Default 0.10 rps (≈ 40 % of ω_s = 0.251) — covers about 10 ms of a 50 ms
# trapezoid ramp at v_target=0.5, which is the window in which the rotor
# typically breaks free.  At v=0.10, Stribeck would have given ≈1.71 A;
# the boost gives the full 1.95 A — a meaningful ~14 % bump.
#
# A discontinuity exists at the threshold (boost → Stribeck taper).  In
# practice the rotor is moving by then so the discontinuity is absorbed
# by the integrator without producing visible jolts.  Override at runtime
# via --boost-threshold; pass --no-stiction-boost to disable entirely.
STICTION_BOOST_THRESHOLD_RPS = 0.10
# Stribeck fit refitted 2026-04-27 18:21 after the bench leg was re-tensioned.
# Source data: temp/logs/cogging_20260427_18*.csv  (R²=0.983 across 8 valid v).
# Original fit (pre-tensioning, kept for reference):
#   τ_c=1.070, τ_s=1.878, ω_s=0.263, b=0.005, load=0.150
# Notable: parameters barely shifted — re-tensioning increased friction by
# only ~3-4%.  The "moderate FF improvement" we saw in the pos_step demo is
# therefore not a parameter-mismatch problem; it's consistent with the FF
# providing ~88% of the true stiction torque (1.71 A iq-equiv at v=0.075
# vs τ_s=1.95 A needed for breakaway), leaving the integrator to wind up
# the remaining ~12% — about right given the position-loop dynamics.


def stribeck_friction_iq(v_rps: float) -> float:
    """Magnitude of friction current at speed |v|, in Amps."""
    av = abs(v_rps)
    return TAU_C + (TAU_S - TAU_C) * math.exp(-(av / OMEGA_S) ** 2) + B_VISC * av


def friction_ff_nm(v_cmd_rps: float, ff_sign: int = -1,
                   include_load: bool = True,
                   stiction_boost: bool = True,
                   boost_threshold_rps: float = STICTION_BOOST_THRESHOLD_RPS) -> float:
    """Compute the torque feedforward (Nm) for a commanded velocity.

    ``ff_sign`` accounts for the bench-rig wiring convention where positive
    iq produces negative mechanical rotation.  Default −1 matches what we
    measured in cruise-mode iq trajectories.

    ``stiction_boost`` (default True): when |v_cmd| is small but non-zero
    (within ``STICTION_BOOST_THRESHOLD_RPS``), apply the full stiction peak
    τ_s instead of the Stribeck-tapered value.  This is a hold-stiction
    "pulse" at motion-onset: the Stribeck formula evaluates to ≈τ_s only at
    exactly v=0, but rapidly tapers as v rises, undersizing the FF during
    the early ramp where the rotor is still trying to break free.  The
    boost holds full stiction torque for the first ~5 ms of the ramp,
    then hands off smoothly to the Stribeck taper.  Pass False to disable.

    At v_cmd == 0 (rotor commanded to hold), FF returns only the load
    offset — no friction term — because there is no commanded motion to
    overcome friction for.  This avoids injecting stiction torque during
    a hold, which would just be a constant disturbance the integrator has
    to fight.
    """
    if abs(v_cmd_rps) < 1e-4:
        # At rest, FF only compensates the constant load (if any)
        return ((LOAD_OFFSET_A if include_load else 0.0)
                * KT_NM_PER_A * ff_sign)
    s = math.copysign(1.0, v_cmd_rps)
    av = abs(v_cmd_rps)
    if stiction_boost and av < boost_threshold_rps:
        # Full stiction peak in the breakaway band
        iq_friction = TAU_S + B_VISC * av
    else:
        iq_friction = stribeck_friction_iq(v_cmd_rps)
    iq_total = -s * iq_friction + (LOAD_OFFSET_A if include_load else 0.0)
    return iq_total * KT_NM_PER_A * ff_sign


# ---------------------------------------------------------------------------
# Safety
# ---------------------------------------------------------------------------

HARD_VEL_CAP_RPS = 5.0          # demo never exceeds this
HARD_CURRENT_LIMIT_A = 10.0     # ODrive soft-limit (matches cogging_bench)
HARD_POSITION_CAP_REV = 3.0     # bench leg full stroke
HEARTBEAT_STALE_S = 1.5

DEFAULT_V_TARGET_RPS = 0.5      # step-response amplitude
DEFAULT_HOLD_S = 0.5            # steady-state hold per trial
DEFAULT_TRIALS_PER_CONDITION = 8
DEFAULT_LOG_RATE_HZ = 200.0     # higher than CAN broadcast — captures every update

# Step-response thresholds (mirrors sim/analysis/diagnose.py motion_onset)
LATENCY_THRESHOLD_FRAC = 0.05   # |v| > 5% of target = "moved"  (vel_step)
TARGET_THRESHOLD_FRAC = 0.90    # |v| > 90% of target = "reached"
SETTLED_TOL_FRAC = 0.10         # within ±10% = settled

# Position-mode trajectory params (defaults — override via CLI).  Trapezoid:
# 0 → v_target over RAMP, hold v_target for HOLD, v_target → 0 over RAMP.
DEFAULT_POS_RAMP_S = 0.05       # 50 ms ramp — matches motor_guard's tick scale
DEFAULT_POS_HOLD_S = 0.30       # 300 ms cruise at v_target
POS_CMD_RATE_HZ = 200.0         # cmd-send rate in pos_step mode (5 ms tick)

# Pos-mode motion-onset detection: rotor must move at least this far for the
# motion-onset event to count.  5 mrev ≈ 0.36 mm of bench-leg extension —
# tight enough to fire on the first rotor flex past detent compliance, loose
# enough to ignore encoder LSB jitter.  Compares to the platform detector's
# 0.5 mm leg-extension threshold (= 7 mrev at the bench leg's geometry).
POS_LATENCY_THRESHOLD_REV = 0.005

# Inter-trial IDLE (let rotor settle in cogging detent)
INTER_TRIAL_REST_S = 0.5


@dataclass
class TrialResult:
    trial: int
    condition: str       # 'no_FF' | 'FF'
    direction: str       # 'forward' | 'reverse'
    v_target_rps: float
    latency_ms: float
    time_to_90pct_ms: float
    overshoot_pct: float
    settling_time_ms: float
    ss_iq_mean_A: float
    ss_iq_std_A: float
    ss_vel_mean_rps: float
    ss_vel_std_rps: float
    samples: int


@dataclass
class Trace:
    trial: int
    condition: str
    direction: str
    t_s: List[float] = field(default_factory=list)
    cmd_vel: List[float] = field(default_factory=list)
    act_vel: List[float] = field(default_factory=list)
    iq_meas: List[float] = field(default_factory=list)
    torque_ff_nm: List[float] = field(default_factory=list)
    # Populated only in pos_step mode
    cmd_pos: List[float] = field(default_factory=list)
    act_pos: List[float] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Bench runner
# ---------------------------------------------------------------------------

class FFDemoBench:
    def __init__(self, axis_id: int, interface: str, channel: str,
                 mode: str, v_target: float, hold_s: float,
                 trials_per_cond: int, ff_sign: int,
                 record_trace: bool, output_path: str,
                 traces_path: Optional[str], dry_run: bool,
                 # sweep-only
                 v_peak: float, sweep_period_s: float, sweep_cycles: int,
                 # pos_step-only
                 pos_ramp_s: float = DEFAULT_POS_RAMP_S,
                 pos_hold_s: float = DEFAULT_POS_HOLD_S,
                 stiction_boost: bool = True,
                 boost_threshold_rps: float = STICTION_BOOST_THRESHOLD_RPS,
                 vel_ff_enabled: bool = True):
        self.harness = SingleLegTestHarness(axis_id=axis_id,
                                            interface=interface, channel=channel)
        self.mode = mode
        self.v_target = v_target
        self.hold_s = hold_s
        self.trials_per_cond = trials_per_cond
        self.ff_sign = ff_sign
        self.record_trace = record_trace
        self.output_path = output_path
        self.traces_path = traces_path
        self.dry_run = dry_run
        self.v_peak = v_peak
        self.sweep_period_s = sweep_period_s
        self.sweep_cycles = sweep_cycles
        self.pos_ramp_s = pos_ramp_s
        self.pos_hold_s = pos_hold_s
        self.stiction_boost = stiction_boost
        self.boost_threshold_rps = boost_threshold_rps
        self.vel_ff_enabled = vel_ff_enabled

        self._aborted = False
        self._abort_reason = ''
        self._session_start_pos_raw: Optional[float] = None
        self._results: List[TrialResult] = []
        self._traces: List[Trace] = []

    def validate(self) -> Optional[str]:
        if self.mode not in ('pos_step', 'vel_step', 'steps', 'sweep'):
            return f'unknown mode {self.mode!r}'
        if self.mode in ('vel_step', 'steps'):
            if abs(self.v_target) > HARD_VEL_CAP_RPS:
                return f'|v_target| {self.v_target} > cap {HARD_VEL_CAP_RPS}'
            if not 0.05 <= self.hold_s <= 5.0:
                return f'hold_s {self.hold_s} outside [0.05, 5.0]'
            if not 1 <= self.trials_per_cond <= 20:
                return f'trials_per_cond {self.trials_per_cond} outside [1, 20]'
            predicted_drift = abs(self.v_target) * self.hold_s * 2  # one pair
            if predicted_drift > HARD_POSITION_CAP_REV:
                return (f'per-pair drift {predicted_drift:.2f} rev '
                        f'exceeds {HARD_POSITION_CAP_REV} rev cap')
        elif self.mode == 'pos_step':
            if abs(self.v_target) > HARD_VEL_CAP_RPS:
                return f'|v_target| {self.v_target} > cap {HARD_VEL_CAP_RPS}'
            if not 0.005 <= self.pos_ramp_s <= 1.0:
                return f'pos_ramp_s {self.pos_ramp_s} outside [0.005, 1.0]'
            if not 0.05 <= self.pos_hold_s <= 5.0:
                return f'pos_hold_s {self.pos_hold_s} outside [0.05, 5.0]'
            if not 1 <= self.trials_per_cond <= 20:
                return f'trials_per_cond {self.trials_per_cond} outside [1, 20]'
            # Each trial moves |v_target| × (ramp + hold) rev (peak displacement
            # of the trapezoid).  fwd/rev pairs cancel under ideal symmetry, but
            # gain it back as headroom in case of asymmetric tracking error.
            per_trial_disp = abs(self.v_target) * (self.pos_ramp_s + self.pos_hold_s)
            if per_trial_disp > HARD_POSITION_CAP_REV * 0.6:
                return (f'per-trial displacement {per_trial_disp:.2f} rev '
                        f'exceeds 60% of {HARD_POSITION_CAP_REV} rev cap')
        else:  # sweep
            if abs(self.v_peak) > HARD_VEL_CAP_RPS:
                return f'|v_peak| {self.v_peak} > cap {HARD_VEL_CAP_RPS}'
            if not 0.5 <= self.sweep_period_s <= 20.0:
                return f'sweep_period_s {self.sweep_period_s} outside [0.5, 20]'
        if self.ff_sign not in (-1, +1):
            return f'ff_sign {self.ff_sign} must be ±1'
        if self.boost_threshold_rps < 0 or self.boost_threshold_rps > 1.0:
            return (f'boost_threshold_rps {self.boost_threshold_rps} outside '
                    f'(0, 1.0]; values above ω_s={OMEGA_S} rps mean the boost '
                    f'is active during steady cruise too, which causes overshoot')
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

    def _enter_velocity_mode(self):
        h = self.harness
        h.send(encode_set_vel_curr_limits(
            h.axis_id,
            vel_limit=HARD_VEL_CAP_RPS * 1.5,
            curr_limit=HARD_CURRENT_LIMIT_A))
        time.sleep(0.05)
        h.send(encode_set_controller_mode(h.axis_id, 'VELOCITY', 'PASSTHROUGH'))
        time.sleep(0.05)
        h.send(encode_set_input_vel(h.axis_id, 0.0, 0.0))
        time.sleep(0.05)
        h.send(encode_set_state(h.axis_id, 'CLOSED_LOOP'))
        h._wait_for_closed_loop()  # type: ignore[attr-defined]

    def _enter_position_mode_at(self, pos_seed_raw: float):
        """Switch to POSITION/PASSTHROUGH closed-loop, seeded at the given pos.

        The seed is critical: switching to POSITION/PASSTHROUGH without seeding
        input_pos = current pos would cause the rotor to snap to whatever stale
        input_pos value the ODrive last had.
        """
        h = self.harness
        h.send(encode_set_vel_curr_limits(
            h.axis_id,
            vel_limit=HARD_VEL_CAP_RPS * 1.5,
            curr_limit=HARD_CURRENT_LIMIT_A))
        time.sleep(0.05)
        h.send(encode_set_controller_mode(h.axis_id, 'POSITION', 'PASSTHROUGH'))
        time.sleep(0.05)
        h.send(encode_set_input_pos(h.axis_id, pos_seed_raw, 0, 0))
        time.sleep(0.05)
        h.send(encode_set_state(h.axis_id, 'CLOSED_LOOP'))
        h._wait_for_closed_loop()  # type: ignore[attr-defined]

    def _idle(self):
        self.harness.send(encode_set_state(self.harness.axis_id, 'IDLE'))

    # -- position-mode trapezoid trial -------------------------------------

    def _run_pos_step_trial(self, trial_idx: int, condition: str,
                            direction: str) -> TrialResult:
        """One trapezoid trial in POSITION/PASSTHROUGH mode.

        Sends pre-computed pos commands (with optional torque_ff) at
        POS_CMD_RATE_HZ, logs at DEFAULT_LOG_RATE_HZ, computes
        position-domain motion-onset latency, time-to-90 %, overshoot,
        settling time, and steady-state iq.
        """
        h = self.harness
        sign = +1.0 if direction == 'forward' else -1.0
        v_signed = self.v_target * sign

        # Settle in IDLE, let rotor sit in detent
        self._idle()
        time.sleep(INTER_TRIAL_REST_S)
        h._poll()  # type: ignore[attr-defined]
        if not self._session_safety_ok():
            return TrialResult(trial_idx, condition, direction, v_signed,
                               0, 0, 0, 0, 0, 0, 0, 0, 0)

        # Read starting pos and seed POSITION mode there
        pos_start = float(h.state.pos_rev_raw)
        self._enter_position_mode_at(pos_start)

        # Pre-compute trapezoid trajectory: ramp 0→v over t_ramp, hold v
        # for t_hold, ramp v→0 over t_ramp.  pos integrates v_cmd via
        # trapezoidal rule.
        cmd_dt = 1.0 / POS_CMD_RATE_HZ
        t_ramp = self.pos_ramp_s
        t_hold = self.pos_hold_s
        t_total = 2.0 * t_ramp + t_hold
        n_cmds = int(round(t_total / cmd_dt)) + 1

        traj_pos: List[float] = []
        traj_v: List[float] = []
        traj_ff_int: List[int] = []
        pos_i = pos_start
        v_prev = 0.0
        for i in range(n_cmds):
            t = i * cmd_dt
            if t < t_ramp:
                v_i = v_signed * (t / t_ramp)
            elif t < t_ramp + t_hold:
                v_i = v_signed
            elif t < t_total:
                v_i = v_signed * (1.0 - (t - t_ramp - t_hold) / t_ramp)
            else:
                v_i = 0.0
            if i > 0:
                pos_i += 0.5 * (v_i + v_prev) * cmd_dt
            v_prev = v_i
            ff_nm = (friction_ff_nm(v_i, self.ff_sign, stiction_boost=self.stiction_boost, boost_threshold_rps=self.boost_threshold_rps)
                     if condition == 'FF' else 0.0)
            traj_pos.append(pos_i)
            traj_v.append(v_i)
            traj_ff_int.append(_torque_ff_to_int16(ff_nm))

        pos_target = traj_pos[-1]   # endpoint of the trapezoid
        traj_displacement = pos_target - pos_start

        # ---- t=0: send the trajectory ----
        trace = Trace(trial=trial_idx, condition=condition, direction=direction)
        log_period = 1.0 / DEFAULT_LOG_RATE_HZ
        t0 = time.time()
        next_log_t = t0
        cmd_idx = 0
        last_v_cmd = 0.0
        last_pos_cmd = pos_start
        last_ff_int = 0

        # Per-trial position guard — abort if rotor drifts beyond a safe
        # multiple of the planned trajectory amplitude.
        per_trial_pos_cap = abs(traj_displacement) * 3.0 + 0.05

        # Metrics
        pos_thresh_motion = POS_LATENCY_THRESHOLD_REV
        pos_thresh_90 = 0.90 * abs(traj_displacement)
        latency_ms = -1.0
        ttp90_ms = -1.0
        peak_pos_excursion = 0.0
        last_excursion_t = -1.0

        # Settle window: when actual_pos has reached its peak (end of
        # trajectory) for steady-state stats.
        ss_window_start = t_ramp + t_hold + t_ramp + 0.05  # 50 ms after ramp-down

        # Loop until we've sent all commands AND held position briefly.
        end_t = t0 + t_total + 0.15
        while time.time() < end_t:
            h._poll(timeout=0.001)  # type: ignore[attr-defined]
            now = time.time()
            t_rel = now - t0

            # Send next command if its target time has arrived
            if cmd_idx < len(traj_pos) and t_rel >= cmd_idx * cmd_dt:
                last_pos_cmd = traj_pos[cmd_idx]
                last_v_cmd = traj_v[cmd_idx]
                last_ff_int = traj_ff_int[cmd_idx]
                # Velocity feedforward: hand the velocity loop the trajectory
                # velocity directly so it doesn't have to wait for pos_err to
                # accumulate.  Without this, the position loop's steady-state
                # tracking lag = v_target / pos_gain (= 12.5 mrev = 25 ms at
                # 0.5 rps with pos_gain=40), independent of any friction FF.
                vel_ff_int = (_vel_ff_to_int16(last_v_cmd)
                              if self.vel_ff_enabled else 0)
                # Bypass harness.send's 2 ms delay — we're already pacing
                # via the trajectory-time check above.
                msg = encode_set_input_pos(h.axis_id, last_pos_cmd,
                                           vel_ff_int, last_ff_int)
                h._bus.send(msg)  # type: ignore[attr-defined]
                cmd_idx += 1

            # Position-mode metrics (computed against actual_pos − pos_start)
            disp = float(h.state.pos_rev_raw) - pos_start
            disp_signed = disp * sign  # positive when moving in commanded direction

            if latency_ms < 0 and disp_signed >= pos_thresh_motion:
                latency_ms = t_rel * 1000.0
            if ttp90_ms < 0 and abs(disp) >= pos_thresh_90:
                ttp90_ms = t_rel * 1000.0
            if abs(disp) > abs(peak_pos_excursion):
                peak_pos_excursion = disp

            # "Settled" criterion: |pos − target| < 10 % of trajectory
            # amplitude.  Track the last time we exceeded that band.
            if abs(float(h.state.pos_rev_raw) - pos_target) > \
                    SETTLED_TOL_FRAC * abs(traj_displacement):
                last_excursion_t = t_rel

            # Per-trial pos guard
            if abs(disp) > per_trial_pos_cap:
                self._abort_reason = (f'trial pos guard: |Δpos| '
                                       f'{disp:+.3f} > {per_trial_pos_cap:.3f} rev')
                self._aborted = True
                break

            # Trace logging
            if now >= next_log_t:
                trace.t_s.append(t_rel)
                trace.cmd_pos.append(last_pos_cmd)
                trace.act_pos.append(float(h.state.pos_rev_raw))
                trace.cmd_vel.append(last_v_cmd)
                trace.act_vel.append(float(h.state.vel_rps_raw))
                trace.iq_meas.append(float(h.state.iq_measured))
                trace.torque_ff_nm.append(last_ff_int / INPUT_TORQUE_SCALE)
                next_log_t += log_period

            if not self._session_safety_ok():
                break

        # Stop and idle
        h.send(encode_set_input_pos(h.axis_id, last_pos_cmd, 0, 0))
        time.sleep(0.05)
        self._idle()

        # Steady-state stats: samples beyond ss_window_start, after rotor
        # has settled near pos_target
        ss_samples = [(trace.act_vel[i], trace.iq_meas[i])
                      for i, t in enumerate(trace.t_s) if t >= ss_window_start]
        if ss_samples:
            ss_v_mean = sum(v for v, _ in ss_samples) / len(ss_samples)
            ss_v_std = (sum((v - ss_v_mean) ** 2 for v, _ in ss_samples)
                        / len(ss_samples)) ** 0.5
            ss_iq_mean = sum(q for _, q in ss_samples) / len(ss_samples)
            ss_iq_std = (sum((q - ss_iq_mean) ** 2 for _, q in ss_samples)
                         / len(ss_samples)) ** 0.5
        else:
            ss_v_mean = ss_v_std = ss_iq_mean = ss_iq_std = 0.0

        # Overshoot %: peak |Δpos| past target as fraction of trajectory amp
        if abs(traj_displacement) > 1e-6:
            overshoot = max(0.0,
                            (abs(peak_pos_excursion) - abs(traj_displacement))
                            / abs(traj_displacement) * 100.0)
        else:
            overshoot = 0.0
        settling_ms = (last_excursion_t * 1000.0) if last_excursion_t > 0 else 0.0
        if latency_ms < 0:
            latency_ms = t_total * 1000.0   # never moved — penalise
        if ttp90_ms < 0:
            ttp90_ms = t_total * 1000.0

        if self.record_trace:
            self._traces.append(trace)

        return TrialResult(
            trial=trial_idx, condition=condition, direction=direction,
            v_target_rps=v_signed,
            latency_ms=round(latency_ms, 2),
            time_to_90pct_ms=round(ttp90_ms, 2),
            overshoot_pct=round(overshoot, 2),
            settling_time_ms=round(settling_ms, 2),
            ss_iq_mean_A=round(ss_iq_mean, 4),
            ss_iq_std_A=round(ss_iq_std, 4),
            ss_vel_mean_rps=round(ss_v_mean, 4),
            ss_vel_std_rps=round(ss_v_std, 4),
            samples=len(trace.t_s),
        )

    # -- step-response trial -----------------------------------------------

    def _run_step_trial(self, trial_idx: int, condition: str,
                        direction: str) -> TrialResult:
        h = self.harness
        v_signed = self.v_target * (+1.0 if direction == 'forward' else -1.0)
        ff_torque = (friction_ff_nm(v_signed, self.ff_sign, stiction_boost=self.stiction_boost, boost_threshold_rps=self.boost_threshold_rps)
                     if condition == 'FF' else 0.0)

        # Settle in IDLE; let rotor sit in detent
        self._idle()
        time.sleep(INTER_TRIAL_REST_S)
        h._poll()  # type: ignore[attr-defined]
        if not self._session_safety_ok():
            return TrialResult(trial_idx, condition, direction, v_signed,
                               0, 0, 0, 0, 0, 0, 0, 0, 0)

        self._enter_velocity_mode()

        # Trace setup
        trace = Trace(trial=trial_idx, condition=condition, direction=direction)
        log_period = 1.0 / DEFAULT_LOG_RATE_HZ

        # ---- t=0: send the step command ----
        t0 = time.time()
        h.send(encode_set_input_vel(h.axis_id, v_signed, ff_torque))
        last_log = t0

        # Track metrics during the step
        v_thresh_5 = LATENCY_THRESHOLD_FRAC * abs(v_signed)
        v_thresh_90 = TARGET_THRESHOLD_FRAC * abs(v_signed)
        latency_ms = -1.0
        ttp90_ms = -1.0
        v_peak = 0.0
        last_excursion_t = -1.0
        ss_window_start = -1.0
        end_t = t0 + self.hold_s

        while time.time() < end_t:
            h._poll(timeout=0.001)  # type: ignore[attr-defined]
            now = time.time()
            t_rel = now - t0
            v = h.state.vel_rps_raw

            # Latency thresholds
            if latency_ms < 0 and abs(v) >= v_thresh_5:
                latency_ms = t_rel * 1000.0
            if ttp90_ms < 0 and abs(v) >= v_thresh_90:
                ttp90_ms = t_rel * 1000.0

            # Peak overshoot
            if abs(v) > abs(v_peak):
                v_peak = v

            # Settling: track last time |v - v_target| > tol
            if abs(abs(v) - abs(v_signed)) > SETTLED_TOL_FRAC * abs(v_signed):
                last_excursion_t = t_rel

            # Trace
            if now - last_log >= log_period:
                trace.t_s.append(t_rel)
                trace.cmd_vel.append(v_signed)
                trace.act_vel.append(float(v))
                trace.iq_meas.append(float(h.state.iq_measured))
                trace.torque_ff_nm.append(ff_torque)
                last_log = now

            if not self._session_safety_ok():
                break

        # Stop and idle
        h.send(encode_set_input_vel(h.axis_id, 0.0, 0.0))
        time.sleep(0.1)
        self._idle()

        # Steady-state stats: last 30 % of the hold (after settling)
        if trace.t_s:
            t_total = trace.t_s[-1]
            ss_lo = 0.7 * t_total
            ss_idx = [i for i, t in enumerate(trace.t_s) if t >= ss_lo]
            if ss_idx:
                vs = [trace.act_vel[i] for i in ss_idx]
                iqs = [trace.iq_meas[i] for i in ss_idx]
                ss_v_mean = sum(vs) / len(vs)
                ss_v_std = (sum((x - ss_v_mean) ** 2 for x in vs) / len(vs)) ** 0.5
                ss_iq_mean = sum(iqs) / len(iqs)
                ss_iq_std = (sum((x - ss_iq_mean) ** 2 for x in iqs) / len(iqs)) ** 0.5
            else:
                ss_v_mean = ss_v_std = ss_iq_mean = ss_iq_std = 0.0
        else:
            ss_v_mean = ss_v_std = ss_iq_mean = ss_iq_std = 0.0

        overshoot = (abs(v_peak) - abs(v_signed)) / abs(v_signed) * 100.0 \
            if abs(v_signed) > 1e-6 else 0.0
        overshoot = max(0.0, overshoot)
        settling_ms = (last_excursion_t * 1000.0) if last_excursion_t > 0 else 0.0
        if latency_ms < 0:
            latency_ms = self.hold_s * 1000.0  # never reached threshold
        if ttp90_ms < 0:
            ttp90_ms = self.hold_s * 1000.0

        if self.record_trace:
            self._traces.append(trace)

        return TrialResult(
            trial=trial_idx, condition=condition, direction=direction,
            v_target_rps=v_signed,
            latency_ms=round(latency_ms, 2),
            time_to_90pct_ms=round(ttp90_ms, 2),
            overshoot_pct=round(overshoot, 2),
            settling_time_ms=round(settling_ms, 2),
            ss_iq_mean_A=round(ss_iq_mean, 4),
            ss_iq_std_A=round(ss_iq_std, 4),
            ss_vel_mean_rps=round(ss_v_mean, 4),
            ss_vel_std_rps=round(ss_v_std, 4),
            samples=len(trace.t_s),
        )

    # -- sweep trial -------------------------------------------------------

    def _run_sweep(self, condition: str) -> Trace:
        """One triangle-wave cycle 0 → +v_peak → -v_peak → 0 over period_s."""
        h = self.harness
        period = self.sweep_period_s
        T_quarter = period / 4.0

        self._idle()
        time.sleep(INTER_TRIAL_REST_S)
        self._enter_velocity_mode()

        trace = Trace(trial=0, condition=condition, direction='sweep')
        log_period = 1.0 / DEFAULT_LOG_RATE_HZ
        t0 = time.time()
        last_log = t0
        last_cmd = 0.0
        end_t = t0 + period * self.sweep_cycles

        while time.time() < end_t:
            h._poll(timeout=0.001)  # type: ignore[attr-defined]
            now = time.time()
            t_rel = now - t0
            phase = (t_rel % period) / period  # 0..1
            # Triangle: 0..0.25 → 0..+v_peak; 0.25..0.75 → +v_peak..-v_peak;
            # 0.75..1 → -v_peak..0
            if phase < 0.25:
                v_cmd = self.v_peak * (phase / 0.25)
            elif phase < 0.75:
                v_cmd = self.v_peak * (1 - 2 * (phase - 0.25) / 0.5)
            else:
                v_cmd = -self.v_peak * (1 - (phase - 0.75) / 0.25)

            ff = (friction_ff_nm(v_cmd, self.ff_sign, stiction_boost=self.stiction_boost, boost_threshold_rps=self.boost_threshold_rps)
                  if condition == 'FF' else 0.0)

            # Re-send command on change or every 50 ms
            if abs(v_cmd - last_cmd) > 1e-3 or (now - last_log) > 0.05:
                h.send(encode_set_input_vel(h.axis_id, v_cmd, ff))
                last_cmd = v_cmd

            if now - last_log >= log_period:
                trace.t_s.append(t_rel)
                trace.cmd_vel.append(v_cmd)
                trace.act_vel.append(float(h.state.vel_rps_raw))
                trace.iq_meas.append(float(h.state.iq_measured))
                trace.torque_ff_nm.append(ff)
                last_log = now

            if not self._session_safety_ok():
                break

        h.send(encode_set_input_vel(h.axis_id, 0.0, 0.0))
        time.sleep(0.1)
        self._idle()
        return trace

    # -- top-level ---------------------------------------------------------

    def run(self) -> int:
        err = self.validate()
        if err:
            print(f'REJECT: {err}', file=sys.stderr)
            return 1

        print('Friction-FF demonstration')
        print(f'  axis_id        = {self.harness.axis_id}')
        print(f'  mode           = {self.mode}')
        if self.mode in ('vel_step', 'steps'):
            print(f'  v_target       = ±{self.v_target} rps')
            print(f'  hold           = {self.hold_s} s')
            print(f'  trials/cond    = {self.trials_per_cond}')
        elif self.mode == 'pos_step':
            print(f'  v_target       = ±{self.v_target} rps (trapezoid peak)')
            print(f'  trapezoid      = {self.pos_ramp_s} s ramp + '
                  f'{self.pos_hold_s} s hold + {self.pos_ramp_s} s ramp')
            per_trial_disp = abs(self.v_target) * (self.pos_ramp_s + self.pos_hold_s)
            print(f'  per-trial disp = ±{per_trial_disp:.3f} rev')
            print(f'  cmd rate       = {POS_CMD_RATE_HZ} Hz')
            print(f'  trials/cond    = {self.trials_per_cond}')
        else:
            print(f'  v_peak         = {self.v_peak} rps')
            print(f'  period         = {self.sweep_period_s} s × {self.sweep_cycles} cycles')
        print(f'  ff_sign        = {self.ff_sign:+d}')
        print(f'  Stribeck FF    = τ_c={TAU_C} A, τ_s={TAU_S} A, '
              f'ω_s={OMEGA_S} rps, b={B_VISC}')
        print(f'  load offset    = {LOAD_OFFSET_A:+.3f} A')
        print(f'  stiction boost = {"ON" if self.stiction_boost else "off"} '
              f'(threshold={self.boost_threshold_rps} rps)')
        print(f'  vel feedforward= {"ON" if self.vel_ff_enabled else "off"} '
              f'(pos_step only)')
        print(f'  output         = {self.output_path}')
        if self.traces_path:
            print(f'  traces         = {self.traces_path}')

        if self.dry_run:
            print('\nDRY-RUN: validation passed, no CAN I/O performed.')
            # Print FF values across a few velocities for sanity-check.
            # Compare these to measured iq from cogging_bench: at v=+0.5 we
            # measured iq_meas≈-0.93 A in steady state.  The FF should produce
            # an effect equivalent to +0.93 A of "controller output" at that
            # velocity to fully unload the integrator.
            print('\n  FF preview (this is what gets sent to ODrive as torque_ff):')
            print('  Compare to measured cruise iq from cogging_bench data.')
            print(f'  Sample values include the stiction-boost band '
                  f'(|v|<{self.boost_threshold_rps} rps when ON).')
            sample_vs = [-2.0, -0.5, -0.2, -0.10, -0.05, -0.025, 0.0,
                         0.025, 0.05, 0.10, 0.2, 0.5, 2.0]
            print(f'    {"v_cmd":>10}  {"τ_ff [Nm]":>12}  {"iq_eq [A]":>10}  in_boost')
            for v in sample_vs:
                ff = friction_ff_nm(v, self.ff_sign,
                                     stiction_boost=self.stiction_boost,
                                     boost_threshold_rps=self.boost_threshold_rps)
                in_boost = (self.stiction_boost
                            and 1e-4 <= abs(v) < self.boost_threshold_rps)
                print(f'    {v:>+10.3f}  {ff:>+12.5f}  '
                      f'{ff / KT_NM_PER_A:>+10.3f}  {"YES" if in_boost else ""}')
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
                        return 1
                self._session_start_pos_raw = h.state.pos_rev_raw
                print(f'  Starting pos_raw = {self._session_start_pos_raw:+.4f} rev\n')

                if self.mode in ('pos_step', 'vel_step', 'steps'):
                    self._run_steps_mode()
                else:
                    self._run_sweep_mode()

                self._idle()
        except Exception as exc:
            print(f'\nABORT: {exc}', file=sys.stderr)
            self._idle()
            return 2

        self._write_outputs()
        if self.mode in ('pos_step', 'vel_step', 'steps'):
            self._print_summary()
        return 0

    def _run_steps_mode(self):
        # Interleave conditions and directions: each cycle = no_FF/fwd, FF/fwd,
        # no_FF/rev, FF/rev.  This way every trial pair (no_FF, FF) is at the
        # same direction and same temperature/state, so the comparison is
        # apples-to-apples per pair.
        is_pos_mode = self.mode == 'pos_step'
        runner = self._run_pos_step_trial if is_pos_mode else self._run_step_trial
        for i in range(self.trials_per_cond):
            for direction in ('forward', 'reverse'):
                if self._aborted:
                    return
                for condition in ('no_FF', 'FF'):
                    if self._aborted:
                        return
                    trial_idx = len(self._results)
                    res = runner(trial_idx, condition, direction)
                    self._results.append(res)
                    print(f'  Trial {trial_idx:3d} ({condition:>5}, {direction:>7}): '
                          f'lat={res.latency_ms:6.1f} ms  '
                          f'ttp90={res.time_to_90pct_ms:6.1f} ms  '
                          f'OS={res.overshoot_pct:5.1f}%  '
                          f'ss_iq={res.ss_iq_mean_A:+6.3f}±{res.ss_iq_std_A:.3f} A')

    def _run_sweep_mode(self):
        for condition in ('no_FF', 'FF'):
            if self._aborted:
                return
            print(f'\n  Sweep ({condition})...')
            trace = self._run_sweep(condition)
            self._traces.append(trace)
            # Compute simple stats
            if trace.cmd_vel:
                rms_err = (sum((c - a) ** 2 for c, a in zip(trace.cmd_vel, trace.act_vel))
                           / len(trace.cmd_vel)) ** 0.5
                print(f'    samples={len(trace.t_s)}  '
                      f'tracking_rms = {rms_err:.4f} rps')

    def _write_outputs(self):
        if self._results:
            with open(self.output_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['trial', 'condition', 'direction', 'v_target_rps',
                            'latency_ms', 'time_to_90pct_ms', 'overshoot_pct',
                            'settling_time_ms', 'ss_iq_mean_A', 'ss_iq_std_A',
                            'ss_vel_mean_rps', 'ss_vel_std_rps', 'samples'])
                for r in self._results:
                    w.writerow([r.trial, r.condition, r.direction, r.v_target_rps,
                                r.latency_ms, r.time_to_90pct_ms, r.overshoot_pct,
                                r.settling_time_ms, r.ss_iq_mean_A, r.ss_iq_std_A,
                                r.ss_vel_mean_rps, r.ss_vel_std_rps, r.samples])
            print(f'\n  Summary CSV → {self.output_path}')

        if self.record_trace and self.traces_path and self._traces:
            with open(self.traces_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['trial', 'condition', 'direction',
                            't_s', 'cmd_pos', 'act_pos',
                            'cmd_vel', 'act_vel',
                            'iq_meas_A', 'torque_ff_nm'])
                for tr in self._traces:
                    for i in range(len(tr.t_s)):
                        cp = tr.cmd_pos[i] if i < len(tr.cmd_pos) else 0.0
                        ap = tr.act_pos[i] if i < len(tr.act_pos) else 0.0
                        w.writerow([tr.trial, tr.condition, tr.direction,
                                    f'{tr.t_s[i]:.4f}',
                                    f'{cp:.5f}',
                                    f'{ap:.5f}',
                                    f'{tr.cmd_vel[i]:.4f}',
                                    f'{tr.act_vel[i]:.4f}',
                                    f'{tr.iq_meas[i]:.4f}',
                                    f'{tr.torque_ff_nm[i]:.5f}'])
            print(f'  Traces CSV → {self.traces_path}')

    def _print_summary(self):
        if not self._results:
            return
        no_ff = [r for r in self._results if r.condition == 'no_FF']
        ff = [r for r in self._results if r.condition == 'FF']
        if not no_ff or not ff:
            return

        def stats(lst, attr):
            vals = [getattr(r, attr) for r in lst]
            mean = sum(vals) / len(vals)
            std = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
            return mean, std

        print('\n' + '=' * 70)
        print('SUMMARY (lower latency / overshoot / iq is better)')
        print('=' * 70)
        print(f"{'Metric':<25} {'no_FF':>15} {'FF':>15} {'Improvement':>12}")
        for label, attr, fmt in [
            ('Motion-onset latency [ms]', 'latency_ms', '{:.1f}'),
            ('Time to 90% target [ms]',   'time_to_90pct_ms', '{:.1f}'),
            ('Overshoot [%]',             'overshoot_pct', '{:.2f}'),
            ('Settling time [ms]',        'settling_time_ms', '{:.1f}'),
            ('|Steady-state iq| [A]',     'ss_iq_mean_A', '{:.3f}'),
        ]:
            m_no, s_no = stats(no_ff, attr)
            m_ff, s_ff = stats(ff, attr)
            if attr == 'ss_iq_mean_A':
                m_no, m_ff = abs(m_no), abs(m_ff)
            ratio = (m_no / m_ff) if m_ff > 1e-6 else float('inf')
            no_str = f'{fmt.format(m_no)} ± {fmt.format(s_no)}'
            ff_str = f'{fmt.format(m_ff)} ± {fmt.format(s_ff)}'
            print(f'{label:<25} {no_str:>15} {ff_str:>15} {ratio:>11.2f}×')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _ts() -> str:
    import datetime
    return datetime.datetime.now().strftime('%Y%m%d_%H%M%S')


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--axis', type=int, default=0)
    p.add_argument('--interface', default='socketcan')
    p.add_argument('--channel', default='can0')
    p.add_argument('--mode',
                   choices=['pos_step', 'vel_step', 'steps', 'sweep'],
                   default='pos_step',
                   help="'pos_step' (default, mirrors platform) | "
                        "'vel_step' (legacy velocity loop) | "
                        "'steps' (alias for vel_step) | 'sweep'")
    p.add_argument('--v-target', type=float, default=DEFAULT_V_TARGET_RPS,
                   help=f'step amplitude in rev/s (pos_step: trapezoid peak; '
                        f'vel_step: step target). Default {DEFAULT_V_TARGET_RPS}')
    p.add_argument('--hold', type=float, default=DEFAULT_HOLD_S,
                   help=f'vel_step: hold time at target per trial (default {DEFAULT_HOLD_S} s)')
    p.add_argument('--ramp-time', type=float, default=DEFAULT_POS_RAMP_S,
                   help=f'pos_step: ramp time at each end of the trapezoid '
                        f'(default {DEFAULT_POS_RAMP_S} s)')
    p.add_argument('--hold-time', type=float, default=DEFAULT_POS_HOLD_S,
                   help=f'pos_step: hold time at v_target between ramps '
                        f'(default {DEFAULT_POS_HOLD_S} s)')
    p.add_argument('--trials', type=int, default=DEFAULT_TRIALS_PER_CONDITION,
                   help=f'trials per (condition × direction) pair '
                        f'(default {DEFAULT_TRIALS_PER_CONDITION})')
    p.add_argument('--ff-sign', type=int, default=-1, choices=[-1, +1],
                   help='polarity of torque_ff (default −1 for the bench rig)')
    p.add_argument('--no-stiction-boost', action='store_false',
                   dest='stiction_boost', default=True,
                   help='disable the v→0 stiction-boost band (default: ON)')
    p.add_argument('--boost-threshold', type=float,
                   default=STICTION_BOOST_THRESHOLD_RPS,
                   help=f'stiction-boost threshold in rev/s — applies τ_s '
                        f'instead of Stribeck-tapered FF for |v|<threshold. '
                        f'Default {STICTION_BOOST_THRESHOLD_RPS}')
    p.add_argument('--no-vel-ff', action='store_false',
                   dest='vel_ff_enabled', default=True,
                   help='disable velocity feedforward via set_input_pos.vel_ff '
                        '(pos_step only; default: ON to eliminate steady-state '
                        'position-loop tracking lag)')
    p.add_argument('--v-peak', type=float, default=0.5,
                   help='sweep mode: peak velocity (rev/s)')
    p.add_argument('--sweep-period', type=float, default=4.0,
                   help='sweep mode: triangle period in s')
    p.add_argument('--sweep-cycles', type=int, default=2,
                   help='sweep mode: number of cycles per condition')
    p.add_argument('--output', default=None)
    p.add_argument('--record-trace', action='store_true', default=True,
                   help='dump per-sample trace CSV (default ON for plotting)')
    p.add_argument('--no-trace', action='store_false', dest='record_trace')
    p.add_argument('--dry-run', action='store_true')
    args = p.parse_args()

    output = args.output or os.path.join(_PROJECT_ROOT, 'temp', 'logs',
                                          f'friction_ff_demo_{args.mode}_{_ts()}.csv')
    traces = (output.replace('.csv', '_traces.csv')
              if args.record_trace else None)

    bench = FFDemoBench(
        axis_id=args.axis, interface=args.interface, channel=args.channel,
        mode=args.mode, v_target=args.v_target, hold_s=args.hold,
        trials_per_cond=args.trials, ff_sign=args.ff_sign,
        record_trace=args.record_trace, output_path=output,
        traces_path=traces, dry_run=args.dry_run,
        v_peak=args.v_peak, sweep_period_s=args.sweep_period,
        sweep_cycles=args.sweep_cycles,
        pos_ramp_s=args.ramp_time, pos_hold_s=args.hold_time,
        stiction_boost=args.stiction_boost,
        boost_threshold_rps=args.boost_threshold,
        vel_ff_enabled=args.vel_ff_enabled)
    return bench.run()


if __name__ == '__main__':
    sys.exit(main())
