#!/usr/bin/env python3
"""Bench-leg system-ID harness — Stage 1 of the leg-gain tuning methodology.

System-ID + gain-escalation on the acceptable-loss bench leg (ODrive node 0),
the Stage-1 prerequisite of ``plans/active/leg-gain-tuning-methodology.md``
(Fast-motion tier). Two selectable backends (``--path``):

**Path BRIDGE (default)** — the bench ODrive on CAN3 behind the can-bridge Teensy
over the UDP ``teensy_link`` protocol. This is the ONLY path available on this
Jetson: direct CAN was wound down entirely (operator-confirmed 2026-07-11 — the
Jetson no longer sees CAN, there is no USB-CAN adapter). Gains apply via RPC
(``SET_POS_GAIN``/``SET_VEL_GAINS``, session-only, re-applied per point); homing
via the firmware ``HOME(0)`` RPC; arming = streaming 40 Hz position-knots with the
J→T heartbeat ``mpc_active`` flag set (the ``teensy_setpoint_bench.py`` idiom).
Stimuli are POSITION-DOMAIN knots through the firmware 500 Hz Hermite + lead
clamp, so they degrade honestly vs the direct path (documented below).

**Path DIRECT (--path direct)** — socketcan straight to ``input_pos``/``input_vel``
via ``SingleLegTestHarness``: clean instantaneous steps/chirps + instant RAM-only
gain-setting. Retained FOR THE RECORD only; **live runs are refused on this Jetson**
(``--path direct`` errors unless ``--dry-run``, which prints the historical plan).

The pure logic (onset/divergence detectors, ladder state machine, stroke-cap-
bounded stimulus generators, step/freq-response estimators, and the Path-BRIDGE
knot-ramp / telemetry-rate / guard-latch-backoff logic) lives in ``sysid_lib.py``,
unit-tested in ``tests/motion/test_bench_sysid_logic.py`` +
``tests/motion/test_bench_sysid_bridge.py``.

Path-BRIDGE degradations vs Path DIRECT (all documented, all honest)
--------------------------------------------------------------------
* **Steps → 25 ms-quantised knot ramps.** No raw step exists on the bridge; a
  "step" is a knot ramp sized UNDER the 0.10 rev lead clamp (``lead_clamp_frame_step``
  at 2× margin ⇒ 0.05 rev/frame = 2.0 rev/s, Jugglebot's operating point). Rise/
  overshoot are of the leg tracking that fast ramp, not an instantaneous step.
* **Velocity-loop step response is INFERRED, not measured.** The bridge has no raw
  ``input_vel`` step; ζ / velocity-loop damping is inferred from position-step
  overshoot (there is no ``vel_steps`` stage on Path BRIDGE).
* **Chirp top frequency is bounded by the telemetry rate AND the 40 Hz knot rate.**
  Measured FIRST over a warmup window; the honest f1 = min(request, telemetry
  bound, knot-stream bound). At 100 Hz telemetry / 40 Hz knots that is ~8 Hz — the
  bridge honestly reaches the 6 Hz pos-loop question but NOT the 30 Hz direct covers.
* **Telemetry is the UDP TELEMETRY/DIAGNOSTIC stream** (pos/vel @ ~100 Hz, iq from
  DIAGNOSTIC), not a 100 Hz CAN poll.

Stages (``--mode``)
-------------------
* ``vel_steps``  Stage 1a-2: velocity-loop step responses (DIRECT only).
* ``pos_steps``  Stage 1a-3a: position-loop step responses (→ ζ, bandwidth). On
                 Path BRIDGE these are knot ramps and also SOURCE the inferred
                 velocity-loop damping.
* ``chirp``      Stage 1a-3b: log-chirp sine sweep (the 6 Hz question),
                 amplitude-bounded by stroke + kinematics, f1 bounded by the
                 telemetry/knot rate on Path BRIDGE.
* ``ladder``     Stage 1b: escalate pos_gain until instability onset, searching
                 vel_gain for ζ ≥ target at each rung, AUTO-BACKOFF to the last
                 stable triple on onset. On Path BRIDGE the firmware MAX_DEVIATION
                 (0.5 rev) + MPC-staleness E-STOPs are the BACKSTOP: the harness
                 detects a guard latch from the heartbeat ``fault_state``, backs
                 off, and recovers via CLEAR_ERRORS — never fighting the guard.
* ``track``      Sustained-tracking discriminant (BRIDGE): drive a continuously-
                 moving sine/triangle reference (the S4 excitation class) and
                 extract the dominant position-residual frequency vs the robot's
                 ~6 Hz limit cycle. Per-run motion-excited HF metrics (Feature 1).
* ``strokes``    Aggressive realistic-stroke battery (BRIDGE): a min-jerk (quintic)
                 amplitude × peak_vel grid + asymmetric throw/catch + a sustained
                 back-to-back sequence, run at each ``--track-gains`` point. The
                 surface the single 1.75 Hz/4.4 rev/s sine was one corner of;
                 per-stroke tracking err / overshoot / HF-onset / rail duty / clamp.
* ``teleop``     Interactive SpaceMouse + live gains (BRIDGE): the z-axis drives the
                 leg position through a slew planner (vel/accel-limited, never a
                 step) with live gain reconfiguration (``g``/``b``/``r``/``q``,
                 near-rest gate), guard-latch freeze-and-require-re-arm, and live
                 ~1 Hz HF/rail/clamp metrics.
* ``all``        DIRECT: vel_steps → pos_steps → chirp. BRIDGE: pos_steps → chirp
                 (no vel_steps). Run ``ladder``/``track``/``strokes``/``teleop`` as
                 separate deliberate invocations.

Motion-excited HF onset (Feature 1)
-----------------------------------
Every MOVING stage (pos_steps, chirp, track, strokes, teleop) reports an ADVISORY
motion-excited HF metric — the band-RMS of velocity and iq in 20-45 Hz with the
knot staircase notched out, plus |iq| rail duties — because the quiescent-hold
buzz gate and arrive-and-settle step tails are structurally blind to the vibration
that only appears under sustained motion (2026-07-12 review). It is printed + logged
to the manifest, never a hard abort.

Selected knobs
--------------
* ``--gains pg:vg:vint``   apply a tuned triple at bringup so pos_steps/chirp
                           characterize THAT loop, not the production baseline.
* ``--rungs pg:vg:vint,…`` (ladder) explicit off-diagonal gap-fill points through
                           the same quiescent-buzz → step → onset pipeline (survey
                           mode: an unstable point never aborts the others).
* ``--quiescent-secs``     (ladder) flat-hold buzz-check window per rung; stretch
                           to 120-180 s for a winner soak.
* ``--track-gains``        (track/strokes) gain points to run the moving battery at.
* ``--stroke-amps/-vels``  (strokes) override the amplitude / peak-vel grid.
* ``--teleop-range/-max-vel`` (teleop) stick half-range (rev) and slew velocity cap.

Safety profile
--------------
The bench leg is SHORTER than a platform leg and the firmware ``STROKE_MAX_REV``
(3.90 rev, production leg-0) does NOT protect the 3.0-rev bench cap — so THIS
DRIVER owns the stroke cap on both paths. Every stimulus is clamped to the stroke
bounds + the kinematic (vel/accel) caps before a setpoint leaves the harness. On
Path BRIDGE the firmware guards (MAX_DEVIATION, MPC-staleness) are the backstop,
not the primary protection. Hard caps below are cited from ``cogging_bench_test.py``.

Usage
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate

    # Dry-run: print the full plan (bounds, stimuli, ladder) — no socket/CAN I/O.
    python tests/hardware/bench_leg_sysid.py --mode all --dry-run           # BRIDGE
    python tests/hardware/bench_leg_sysid.py --mode ladder --dry-run
    python tests/hardware/bench_leg_sysid.py --path direct --mode all --dry-run  # historical

    # System-ID over the bridge (home first if the encoder reference is fresh):
    python tests/hardware/bench_leg_sysid.py --mode all --home
    # Escalate-until-unstable gain ladder over the bridge:
    python tests/hardware/bench_leg_sysid.py --mode ladder --home

Output
------
A per-run directory ``temp/probes/bench_sysid_<ts>/`` containing one CSV per
stimulus plus ``manifest.json`` (gains, caps, bounds, thresholds, timestamps,
telemetry-rate measurement, and per-stage results). CSVs are gitignored under
the umbrella ``temp/`` rule.

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
import threading
import time
from dataclasses import asdict
from typing import List, Optional

import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
sys.path.insert(0, _SCRIPT_DIR)
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import sysid_lib as sid  # noqa: E402

# Path DIRECT backend (socketcan bench harness). Direct CAN was wound down on this
# Jetson (2026-07-11): no USB-CAN adapter, the Jetson no longer sees CAN. The import
# is GUARDED so Path BRIDGE (the default) works even where python-can / the adapter
# is absent; the direct runner refuses live runs regardless (see main()) and is kept
# only for the record + historical --dry-run planning.
try:
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
    _DIRECT_IMPORT_ERR: Optional[Exception] = None
except Exception as _exc:  # noqa: BLE001 — python-can/adapter absent is expected post-cutover
    SingleLegTestHarness = None  # type: ignore
    _DIRECT_IMPORT_ERR = _exc

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
# 4.0 rev/s (~0.28 m/s leg-end) was Jugglebot's configured ODrive ceiling through
# 2026-07-15; the production ceiling was raised to 6.0 rev/s on 2026-07-16 (lead-clamp
# catch-up headroom), but this bench default STAYS 4.0 as a deliberately conservative
# measurement-step velocity (per the leg-actuator-limits note). The 35 rev/s
# HARD_VEL_CAP_RPS survival ceiling stays reachable by passing --vel-cap 35 explicitly
# (F4, 2026-07-11).
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

DEFAULT_SEND_PERIOD_S = 0.005          # 200 Hz setpoint/sample cadence (DIRECT)
SETTLE_S = 1.0                         # post-step settle window for tail analysis

# ---------------------------------------------------------------------------
# Path BRIDGE constants — the can-bridge Teensy over UDP teensy_link. Firmware
# values MUST match canbridge_config.h (cited) or the knot lookahead / lead clamp
# / staleness assumptions drift from what the Teensy actually does.
# ---------------------------------------------------------------------------
BRIDGE_TEENSY_IP = "192.168.42.2"       # ADR-0007 point-to-point link (teensy_setpoint_bench.py:79)
BRIDGE_SEG_T_S = 0.025                  # canbridge_config.h:116 SEGMENT_T_S (40 Hz knot step)
BRIDGE_SETPOINT_HZ = 40.0               # 1 / SEG_T — the knot stream rate
BRIDGE_MAX_LEAD_REV = 0.10              # canbridge_config.h:139 MAX_LEAD_REV (interp lead clamp)
BRIDGE_MAX_DEVIATION_REV = 1.0          # canbridge_config.h MAX_DEVIATION_REV (E-STOP backstop; raised 0.5→1.0 on 2026-07-16 for velocity-loop-lag headroom under coordinated moves)
BRIDGE_MPC_STALENESS_S = 0.25           # canbridge_config.h:154 MPC_CMD_STALENESS_US (E-STOP backstop)
BRIDGE_LEAD_MARGIN_FRAC = 0.5           # approach/return/settle knot-ramp step at 0.5×lead clamp (2× margin ⇒ 2.0 rev/s)
BRIDGE_STEP_LEAD_MARGIN_FRAC = 0.9      # MEASUREMENT steps (pos_steps + ladder step) ramp at 0.9×lead clamp
BRIDGE_RAMP_VEL_TARGET_RPS = 2.0        # design velocity the 0.5 margin implies at 40 Hz knots — held across --knot-hz
BRIDGE_STEP_RAMP_VEL_TARGET_RPS = 3.6   # design measurement-step velocity (under vel_limit 4.0) — held across --knot-hz
                                        # ⇒ 0.09 rev/frame = 3.6 rev/s (still < MAX_LEAD 0.10, vel_limit 4.0; the
                                        # firmware's 3.5 vel_ff cap only caps the ff TERM). The 0.5-margin ramp is
                                        # too gentle to stress the servo — rise time reflects the ramp, not the loop
                                        # (rates finding #4). Fast ramp only for the step under measurement.
DEFAULT_CHIRP_AMPS_REV = [0.02, 0.06, 0.12]  # multi-amplitude chirp: friction attenuates the small-signal gain
                                             # (~0.5 at 0.02 rev); gain rises toward 1 with amplitude (chirp finding)
DEFAULT_TRACK_GAINS = '40:0.2:0.32'     # sustained-tracking gain point(s): the robot's production gains
QUIESCENT_HOLD_S = 0.75                 # flat-hold buzz-check window before each high-rung step
BRIDGE_TELEM_NOMINAL_HZ = 100.0         # TELEMETRY nominal rate (dry-run assumption; measured live)
BRIDGE_WARMUP_S = 2.0                   # RX-only window to MEASURE telemetry rate before any stimulus
BRIDGE_MAX_RECOVERIES = 2               # guard-latch CLEAR_ERRORS budget before the ladder aborts
BRIDGE_T2J_FLAG_MPC_ACTIVE = 0x8        # HeartbeatT2J.flags bit3 — verify our arm took
BRIDGE_ARM_VERIFY_GRACE_S = 0.7         # allow this long after arm for firmware mpc_active to report
_BRIDGE_STROKE_CEIL_PROD_REV = 3.90     # firmware STROKE_MAX_REV[0] — production clamp, NOT a bench bound
# Stream-then-arm + startup guard-latch clearance (sysid_lib owns the pure defaults).
BRIDGE_ARM_WARMUP_FRAMES = sid.BRIDGE_ARM_WARMUP_FRAMES   # DISARMED knots streamed before EVERY arm
BRIDGE_CLEAR_DEV_TOL_REV = sid.BRIDGE_CLEAR_DEV_TOL_REV   # u0≈enc gate before a startup CLEAR_ERRORS
BRIDGE_STARTUP_CLEAR_TIMEOUT_S = 2.0    # wait this long for fault_state→NONE after a startup CLEAR_ERRORS


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


def _bridge_ports():
    """(stream_port, rpc_port) from the generated protocol — imported lazily so
    the module (and Path-DIRECT dry-run) never needs teensy_link."""
    from controller.teensy_link import protocol as p  # noqa: E402
    return int(p.PORT_STREAM), int(p.PORT_RPC)


class BridgeSysID:
    """Path BRIDGE Stage-1 runner: position-knot stimuli + gain ladder over the
    can-bridge Teensy (UDP teensy_link).

    Design (why a separate runner, not a shared backend behind ``BenchSysID``):
    the two paths share *what to measure and how to judge it* — every onset /
    divergence / ladder / step-metric / freq-response function lives in
    ``sysid_lib`` and is used by BOTH. What they do NOT share is the DRIVE: Path
    DIRECT issues instantaneous ``input_pos``/``input_vel`` setpoints and polls
    100 Hz CAN; Path BRIDGE runs a CONTINUOUS armed 40 Hz knot stream through the
    firmware Hermite + lead clamp, with a live guard machine underneath and RPC
    gains. Forcing those two transports behind one stage-level ABC would be a
    leaky abstraction (the stimulus shaping, telemetry source, homing, and guard
    handling all differ), so the split is: share ``sysid_lib`` (the analysis),
    separate the transport. This is the only runnable path on this Jetson.
    """

    def __init__(self, *, axis_id: int, stroke_cap_rev: float,
                 center_rev: Optional[float], current_limit_a: float,
                 vel_cap_rps: float, output_dir: str, dry_run: bool,
                 chirp_f0: float, chirp_f1: float, chirp_dur: float,
                 chirp_amp: float, pos_steps: List[float], ladder_step: float,
                 zeta_target: float, bw_clear_hz: Optional[float], n_vel: int,
                 ripple_threshold: float, osc_threshold: float,
                 iq_ripple_threshold: float, do_home: bool,
                 max_recoveries: int = BRIDGE_MAX_RECOVERIES,
                 chirp_amps: Optional[List[float]] = None,
                 knot_hz: float = BRIDGE_SETPOINT_HZ,
                 vint_ratio: Optional[float] = None,
                 from_pg: Optional[float] = None,
                 track_wave: str = 'sine',
                 track_peak_vel: float = sid.TRACK_PEAK_VEL_DEFAULT_RPS,
                 track_secs: float = sid.TRACK_SECS_DEFAULT,
                 track_gains: Optional[List[sid.GainTriple]] = None,
                 fast_iq_available: bool = False,
                 custom_rungs: Optional[List[sid.GainTriple]] = None,
                 quiescent_secs: float = QUIESCENT_HOLD_S,
                 gains_override: Optional[sid.GainTriple] = None,
                 stroke_amps: Optional[List[float]] = None,
                 stroke_vels: Optional[List[float]] = None,
                 teleop_range: float = sid.TELEOP_RANGE_DEFAULT,
                 teleop_max_vel: float = sid.TELEOP_MAX_VEL_DEFAULT,
                 teleop_device: Optional[str] = None):
        self.axis_id = axis_id
        self.stroke_cap_rev = stroke_cap_rev
        self.current_limit_a = current_limit_a
        self.vel_cap_rps = vel_cap_rps
        self.dry_run = dry_run
        self.do_home = do_home
        self.bounds = sid.stroke_bounds(stroke_cap_rev, STROKE_MARGIN_REV)
        self.center_rev = center_rev if center_rev is not None else stroke_cap_rev / 2.0
        self.chirp_f0 = chirp_f0
        self.chirp_f1 = chirp_f1
        self.chirp_dur = chirp_dur
        self.chirp_amp = chirp_amp
        self.chirp_amps = list(chirp_amps) if chirp_amps else [chirp_amp]
        self.pos_steps = pos_steps
        self.ladder_step = ladder_step
        self.zeta_target = zeta_target
        self.bw_clear_hz = bw_clear_hz
        self.n_vel = n_vel
        self.ripple_threshold = ripple_threshold
        self.osc_threshold = osc_threshold
        self.iq_ripple_threshold = iq_ripple_threshold
        self.output_dir = output_dir
        self.vint_ratio = vint_ratio
        self.from_pg = from_pg
        self.track_wave = track_wave
        self.track_peak_vel = track_peak_vel
        self.track_secs = track_secs
        self.track_gains = track_gains or [BASELINE_GAINS]
        self.fast_iq_available = fast_iq_available
        self.custom_rungs = list(custom_rungs) if custom_rungs else None
        self.quiescent_secs = float(quiescent_secs)
        self.gains_override = gains_override
        # Feature 2 (strokes) + Feature 3 (teleop) knobs.
        self.stroke_amps = list(stroke_amps) if stroke_amps else list(sid.STROKE_AMPS_DEFAULT)
        self.stroke_vels = list(stroke_vels) if stroke_vels else list(sid.STROKE_VELS_DEFAULT)
        self.teleop_range = float(teleop_range)
        self.teleop_max_vel = float(teleop_max_vel)
        self.teleop_device = teleop_device

        # Knot rate (Feature 6): default 40 Hz. A 100 Hz-knot BENCH_SYSID_BUILD firmware
        # lifts the honest chirp ceiling (knot_stream_top_freq = knot_hz / 5) from 8 → 20 Hz.
        self.knot_hz = float(knot_hz)
        self.seg_t_s = 1.0 / self.knot_hz
        self.setpoint_hz = self.knot_hz

        # Bridge stimulus sizing (all in the harness — the firmware STROKE_MAX_REV
        # is the production 3.90-rev clamp and does NOT protect the 3.0-rev bench cap).
        # Two frame steps: a gentle 0.5-margin ramp for approach/return/settle (never
        # stress the servo on a repositioning move) and a fast 0.9-margin ramp for the
        # step UNDER MEASUREMENT (so rise time reflects the loop, not the ramp).
        # Velocity-anchored: a fixed rev/frame delta scales its velocity with --knot-hz
        # (0.09 rev/frame would be 9.0 rev/s at 100 Hz knots, past vel_limit 4.0), so
        # the sizing holds the 40 Hz design velocities across knot rates.
        self.frame_step_rev = sid.velocity_anchored_frame_step(
            BRIDGE_MAX_LEAD_REV, BRIDGE_LEAD_MARGIN_FRAC,
            BRIDGE_RAMP_VEL_TARGET_RPS, self.seg_t_s)
        self.step_frame_step_rev = sid.velocity_anchored_frame_step(
            BRIDGE_MAX_LEAD_REV, BRIDGE_STEP_LEAD_MARGIN_FRAC,
            BRIDGE_STEP_RAMP_VEL_TARGET_RPS, self.seg_t_s)
        self.hold_frames = int(round(SETTLE_S / self.seg_t_s))
        self.guard = sid.GuardLatchBackoff(max_recoveries=max_recoveries)
        self.telem_rate: Optional[sid.TelemetryRate] = None

        # 100 Hz telemetry-frame CSV logging (Feature 1): decoupled from the 40 Hz send
        # loop. The telemetry callback writes one row per RECEIVED frame; the send loop
        # only publishes the current knot u0 (sample-and-hold cmd). Guarded by _log_lock.
        self._log_active = False
        self._log_writer = None
        self._log_u0 = float('nan')
        self._log_t0 = 0.0
        self._log_buf: List[tuple] = []

        # Transport handles — created in run() (dry-run opens no socket).
        self._client = None
        self._rpc = None
        self._rpc_server = None
        self._tod = None
        self._nlegs = 6
        self._lock = threading.Lock()
        self._cache = {'telem': None, 'telem_ts': None, 'diag': {}, 'hb': None}
        self._sigint = False
        self._abort_reason = ''

        self._manifest = {
            'created': datetime.datetime.now().isoformat(timespec='seconds'),
            'path': 'bridge',
            'axis_id': axis_id,
            'teensy_ip': BRIDGE_TEENSY_IP,
            'caps': {
                'stroke_cap_rev': stroke_cap_rev,
                'vel_cap_rps': vel_cap_rps,
                'accel_cap_rps2': HARD_ACCEL_CAP_RPS2,
                'current_limit_a': current_limit_a,
            },
            'bridge': {
                'knot_hz': self.knot_hz,
                'seg_t_s': self.seg_t_s,
                'setpoint_hz': self.setpoint_hz,
                'max_lead_rev': BRIDGE_MAX_LEAD_REV,
                'max_deviation_rev': BRIDGE_MAX_DEVIATION_REV,
                'mpc_staleness_s': BRIDGE_MPC_STALENESS_S,
                'knot_frame_step_rev': self.frame_step_rev,
                'step_frame_step_rev': self.step_frame_step_rev,
                'lead_margin_frac': BRIDGE_LEAD_MARGIN_FRAC,
                'step_lead_margin_frac': BRIDGE_STEP_LEAD_MARGIN_FRAC,
                'max_recoveries': max_recoveries,
                'arm_settle_frames': sid.BRIDGE_ARM_SETTLE_FRAMES,
                'engage_test_rev': sid.BRIDGE_ENGAGE_TEST_REV,
                'csv_sampling': 'telemetry-frame (~100 Hz); cmd_rev = latest streamed '
                                'knot u0 (sample-and-hold)',
            },
            'bounds_rev': [self.bounds.lo_rev, self.bounds.hi_rev],
            'center_rev': self.center_rev,
            'baseline_gains': asdict(BASELINE_GAINS),
            # The gains every bringup actually applies (pos_steps/chirp measure
            # THESE, never RAM leftovers from a previous stage — the 2026-07-12
            # 18:03 run was nearly misread over exactly this ambiguity).
            'gains_applied_at_bringup': asdict(gains_override or BASELINE_GAINS),
            'quiescent_hold_s': self.quiescent_secs,
            'custom_rungs': ([asdict(t) for t in self.custom_rungs]
                             if self.custom_rungs else None),
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
                    f"(0, {HARD_STROKE_CEIL_REV}]")
        if not self.bounds.contains(self.center_rev):
            return (f"center {self.center_rev:.3f} rev outside usable bounds "
                    f"[{self.bounds.lo_rev:.3f}, {self.bounds.hi_rev:.3f}]")
        if not (0.0 < self.current_limit_a <= HARD_CURRENT_LIMIT_A):
            return f"current limit {self.current_limit_a} A outside (0, {HARD_CURRENT_LIMIT_A}]"
        if not (0.0 < self.vel_cap_rps <= HARD_VEL_CAP_RPS):
            return f"velocity cap {self.vel_cap_rps} rev/s outside (0, {HARD_VEL_CAP_RPS}]"
        # Every pos-step target must land inside bounds after the clamp (they do by
        # construction — position_step_series clamps — but the ladder step must too).
        if abs(self.ladder_step) > self.bounds.width:
            return f"ladder step {self.ladder_step} rev wider than the usable stroke"
        return None

    # -- kinematic bounds (harness-owned) ------------------------------------

    def _chirp_amplitude_request(self, amp: Optional[float] = None) -> float:
        """Requested chirp amplitude reduced to fit the vel/accel caps AND the lead
        clamp at the BRIDGE chirp f1 (itself bounded by the telemetry/knot rate).

        ``amp`` defaults to the primary ``chirp_amp``; the multi-amplitude sweep
        (Feature 3) passes each requested amplitude in turn so every sweep gets its own
        per-amplitude kinematic auto-reduction.

        The vel/accel caps alone leave the chirp riding the 0.10 rev lead clamp — at the
        vel-bound amplitude the peak per-knot step is ~MAX_LEAD exactly (``vel_cap 4.0 ==
        MAX_LEAD/seg_t``), which distorts the frequency response (the streamed u0 stops
        matching the interp-clamped command) and can accumulate to a MAX_DEVIATION latch.
        The lead-clamp bound gives the sweep the same 2× lead margin the position steps get
        and keeps u0 a faithful lock-in reference (sysid_bridge finding #4)."""
        req = self.chirp_amp if amp is None else amp
        f1 = self._chirp_f1_bridge()
        kin = sid.chirp_amplitude_cap_for_kinematics(
            f1, vel_cap_rps=self.vel_cap_rps, accel_cap_rps2=HARD_ACCEL_CAP_RPS2)
        lead = sid.chirp_amplitude_cap_for_lead_clamp(
            f1, frame_step_rev=self.frame_step_rev, seg_t_s=self.seg_t_s)
        return min(req, kin, lead)

    def _telem_effective_hz(self) -> float:
        if self.telem_rate is not None and self.telem_rate.effective_hz > 0:
            return self.telem_rate.effective_hz
        return BRIDGE_TELEM_NOMINAL_HZ

    def _chirp_f1_bridge(self) -> float:
        """The honest bridge chirp top frequency: min(request, telemetry bound,
        knot-stream bound). Uses the measured rate when available, else nominal."""
        return sid.bridge_chirp_top_freq(
            self.chirp_f1, self._telem_effective_hz(), self.seg_t_s)

    def _response_bins(self) -> List[float]:
        f1 = self._chirp_f1_bridge()
        return [f for f in DEFAULT_CHIRP_FREQS_HZ
                if self.chirp_f0 - 1e-9 <= f <= f1 + 1e-9]

    # -- transport (live only) -----------------------------------------------

    def _setup_transport(self):
        from controller.teensy_link import (  # noqa: E402
            TeensyLinkClient, RpcClient, RpcServer, TimeOfDayServer, MsgType,
            Telemetry, Diagnostic, HeartbeatT2J,
        )
        from controller.teensy_link import protocol as pr  # noqa: E402
        self._MsgType = MsgType
        self._Telemetry = Telemetry
        self._Diagnostic = Diagnostic
        self._HeartbeatT2J = HeartbeatT2J
        self._Setpoint = pr.Setpoint
        self._RpcMethod = pr.RpcMethod
        self._nlegs = int(pr.NUM_LEGS)
        from controller.teensy_link import rpc_args  # noqa: E402
        self._rpc_args = rpc_args

        self._client = TeensyLinkClient(
            teensy_addr=(BRIDGE_TEENSY_IP, int(pr.PORT_STREAM)), bind_host="0.0.0.0")
        self._client.start()
        self._rpc_server = RpcServer(self._client)
        self._tod = TimeOfDayServer(self._rpc_server)
        self._rpc = RpcClient(self._client)
        self._client.subscribe(int(MsgType.TELEMETRY), self._on_telem)
        self._client.subscribe(int(MsgType.DIAGNOSTIC), self._on_diag)
        self._client.subscribe(int(MsgType.HEARTBEAT_T2J), self._on_hb)
        # DISARMED heartbeat (mpc_active=0) — RX/RPC only until we deliberately arm.
        self._client.start_heartbeat(hz=float(pr.HEARTBEAT_HZ), flags=0)
        self._client.set_heartbeat_flags(0)

    def _teardown_transport(self):
        try:
            self._disarm()
        except Exception:
            pass
        for closer in (getattr(self, '_tod', None), getattr(self, '_rpc_server', None)):
            try:
                if closer is not None:
                    closer.close()
            except Exception:
                pass
        try:
            if self._client is not None:
                self._client.stop()
        except Exception:
            pass

    def _on_telem(self, mt, seq, payload, addr):
        tm = self._Telemetry.unpack(payload)
        now = time.perf_counter()
        with self._lock:
            self._cache['telem'] = tm
            self._cache['telem_ts'] = now
            if self._log_active:
                self._log_frame(tm, now)

    def _log_frame(self, tm, now):
        """Append one 100 Hz telemetry-frame row (called under ``self._lock`` from the RX
        thread, so CSV sampling is DECOUPLED from the 40 Hz send loop — the old per-knot
        writerow decimated pos/vel to the knot rate and threw away ~60% of the frames that
        already arrive). ``cmd`` is the latest streamed knot u0 (sample-and-hold)."""
        ax = self.axis_id
        t_rel = now - self._log_t0
        u0 = self._log_u0
        try:
            pos = float(tm.pos_rev[ax])
            vel = float(tm.vel_rps[ax])
        except Exception:  # noqa: BLE001 — short/garbled frame
            pos = float('nan')
            vel = float('nan')
        d = self._cache['diag'].get(ax)
        iq = float(d.iq_measured) if d is not None else float('nan')
        hb = self._cache['hb']
        fs = int(hb.fault_state) if hb is not None else None
        lc = None
        if hb is not None:
            try:
                lc = int((int(hb.lead_clamp_mask) >> ax) & 1)
            except Exception:  # noqa: BLE001 — older/short heartbeat without the field
                lc = None
        self._log_buf.append((t_rel, u0, pos, vel, iq,
                              float('nan') if fs is None else float(fs),
                              float('nan') if lc is None else float(lc)))
        if self._log_writer is not None:
            self._log_writer.writerow([
                f"{t_rel:.4f}", f"{u0:.6f}",
                "" if pos != pos else f"{pos:.6f}",
                "" if vel != vel else f"{vel:.6f}",
                "" if iq != iq else f"{iq:.4f}",
                "" if fs is None else int(fs),
                "" if lc is None else int(lc)])

    def _begin_telem_log(self, writer, u0_init):
        with self._lock:
            self._log_writer = writer
            self._log_u0 = float(u0_init)
            self._log_t0 = time.perf_counter()
            self._log_buf = []
            self._log_active = True

    def _set_log_u0(self, u0):
        with self._lock:
            self._log_u0 = float(u0)

    def _end_telem_log(self) -> dict:
        with self._lock:
            self._log_active = False
            buf = self._log_buf
            self._log_buf = []
            self._log_writer = None
        cols = ('t', 'cmd', 'pos', 'vel', 'iq', 'fault', 'lead_clamp')
        if buf:
            arr = np.asarray(buf, float)
            out = {c: arr[:, i] for i, c in enumerate(cols)}
        else:
            out = {c: np.array([], float) for c in cols}
        out['telem_ts'] = out['t']
        return out

    def _on_diag(self, mt, seq, payload, addr):
        d = self._Diagnostic.unpack(payload)
        with self._lock:
            self._cache['diag'][int(d.axis_id)] = d

    def _on_hb(self, mt, seq, payload, addr):
        with self._lock:
            self._cache['hb'] = self._HeartbeatT2J.unpack(payload)

    def _sample(self):
        with self._lock:
            tm = self._cache['telem']
            ts = self._cache['telem_ts']
            d = self._cache['diag'].get(self.axis_id)
        pos = None if tm is None else float(tm.pos_rev[self.axis_id])
        vel = None if tm is None else float(tm.vel_rps[self.axis_id])
        iq = None if d is None else float(d.iq_measured)
        return pos, vel, iq, ts

    def _fault_state(self) -> Optional[int]:
        with self._lock:
            hb = self._cache['hb']
        return None if hb is None else int(hb.fault_state)

    def _fw_mpc_active(self) -> Optional[bool]:
        with self._lock:
            hb = self._cache['hb']
        return None if hb is None else bool(hb.flags & BRIDGE_T2J_FLAG_MPC_ACTIVE)

    def _vec(self, val: float):
        ax = self.axis_id
        return tuple(float(val) if i == ax else 0.0 for i in range(self._nlegs))

    def _arm(self):
        self._client.set_heartbeat_flags(1)   # mpc_active=1 — firmware output gate may ENABLE

    def _disarm(self):
        if self._client is not None:
            self._client.set_heartbeat_flags(0)   # mpc_active=0 — firmware gates output off

    def _apply_gains(self, g: sid.GainTriple):
        """RAM-only, session-only gain apply over CAN3 via RPC (re-applied per point)."""
        self._rpc.call(int(self._RpcMethod.SET_POS_GAIN),
                       self._rpc_args.encode_set_pos_gain(self.axis_id, g.pos_gain))
        self._rpc.call(int(self._RpcMethod.SET_VEL_GAINS),
                       self._rpc_args.encode_set_vel_gains(
                           self.axis_id, g.vel_gain, g.vel_int_gain))

    def _clear_errors(self):
        self._rpc.call(int(self._RpcMethod.CLEAR_ERRORS),
                       self._rpc_args.encode_clear_errors(self.axis_id))

    def _send_knot(self, u0, u1, u2, v0, t_origin_us):
        sp = self._Setpoint(
            u0=self._vec(u0), u1=self._vec(u1), u2=self._vec(u2), v0=self._vec(v0),
            accel=(0.0,) * self._nlegs, torque_ff=(0.0,) * self._nlegs,
            flags=0x3, t_origin_us=int(t_origin_us))
        self._client.send_stream(int(self._MsgType.SETPOINT), sp.pack())

    def _measure_telemetry_rate(self) -> sid.TelemetryRate:
        """RX-ONLY warmup: collect TELEMETRY arrival stamps over BRIDGE_WARMUP_S and
        estimate the rate + irregularity BEFORE any stimulus (it bounds the honest
        chirp top frequency). Never streams a setpoint — cannot command motion."""
        stamps: List[float] = []
        unsub = self._client.subscribe(
            int(self._MsgType.TELEMETRY),
            lambda mt, seq, pl, addr: stamps.append(time.perf_counter()))
        t_end = time.perf_counter() + BRIDGE_WARMUP_S
        while time.perf_counter() < t_end and not self._sigint:
            time.sleep(0.02)
        unsub()
        self.telem_rate = sid.estimate_telemetry_rate(stamps)
        return self.telem_rate

    # -- stream-then-arm + startup guard-latch clearance ---------------------

    def _live_deviation(self) -> Optional[float]:
        """This axis's live interp-base-minus-encoder deviation from the T→J
        heartbeat (``live_deviation`` is packed unconditionally, .ino:134), or None."""
        with self._lock:
            hb = self._cache['hb']
        if hb is None:
            return None
        try:
            return float(hb.live_deviation[self.axis_id])
        except Exception:  # noqa: BLE001 — older/short heartbeat without the field
            return None

    def _lead_clamp_bit(self) -> Optional[bool]:
        """This axis's lead-clamp-engaged bit from the last 500 Hz interp tick
        (HeartbeatT2J.lead_clamp_mask, .ino:135), or None if no/short heartbeat."""
        with self._lock:
            hb = self._cache['hb']
        if hb is None:
            return None
        try:
            return bool((int(hb.lead_clamp_mask) >> self.axis_id) & 1)
        except Exception:  # noqa: BLE001
            return None

    def _abort_diag(self, res: dict) -> str:
        """Build a self-diagnosing abort string from a stream result: which guard
        latched + the live u0(=streamed base)/enc/deviation at the latch, so the
        operator's next report names the fault instead of an opaque 'guard latch'."""
        cmd = res.get('cmd')
        pos = res.get('pos')
        # Prefer the last telemetry-logged u0/enc; fall back to the scalars captured at the
        # latch when no telemetry frame landed during the stream (empty log).
        u0 = (float(cmd[-1]) if cmd is not None and len(cmd)
              else float(res.get('last_u0', float('nan'))))
        enc = (float(pos[-1]) if pos is not None and len(pos)
               else float(res.get('last_pos', float('nan'))))
        fs = self._fault_state()
        ga = res.get('guard_action')
        reason = ga.reason if ga is not None else ''
        return sid.approach_abort_diagnostic(
            fault_state=fs if fs is not None else sid.FAULT_NONE,
            u0_rev=u0, enc_rev=enc, guard_reason=reason,
            lead_clamp=self._lead_clamp_bit(), mpc_active=self._fw_mpc_active())

    def _verify_output_engaged(self, pos: float) -> dict:
        """HARD-VERIFY the firmware output gate is driving the leg: command a tiny
        bounded increment (BRIDGE_ENGAGE_TEST_REV, << lead clamp + MAX_DEVIATION so it
        can NEVER itself latch the guard) and confirm the encoder follows it. On failure
        returns a SPECIFIC unmet-condition reason with the live u0/enc/dev/fault so the
        operator's next run is self-diagnosing. Returns {engaged, guard_latched, reason}.

        The nudge is referenced to the LIVE encoder ``enc0`` sampled HERE, NOT the caller's
        ``pos`` argument. The preceding arm-settle can MOVE the leg: when the homed start
        sits below the firmware STROKE_MIN clamp (canbridge_config.h STROKE_MIN_REV ≈ 0.071
        rev), the firmware stroke clamp drives the leg UP to STROKE_MIN during the flat
        settle. Measuring the commanded delta from the stale ``pos`` while the encoder delta
        is measured from the moved-to ``enc0`` mixes two baselines and understates the
        tracked fraction — the 2026-07-12 FALSE 'not following' abort (home −0.069 → settle
        to +0.071; the 0.03 nudge inflated to 0.219 rev and 0.075 rev of honest tracking read
        as 34 %). ``engagement_probe_target`` clamps only on the high side so the nudge stays
        tiny even when the leg legitimately sits below the harness window."""
        enc0, _, _, _ = self._sample()
        enc0 = enc0 if enc0 is not None else pos
        tgt = sid.engagement_probe_target(
            enc0, sid.BRIDGE_ENGAGE_TEST_REV, self.bounds.hi_rev)
        probe = self._stream_and_sample(
            self._knot_ramp(enc0, tgt, hold_frames=int(0.3 / self.seg_t_s)), guard=True)
        enc1, _, _, _ = self._sample()
        enc1 = enc1 if enc1 is not None else enc0
        if probe['aborted'] or probe['guard_latched']:
            # Even the tiny probe latched — surface exactly which guard + live quantities.
            self._stream_and_sample(self._knot_ramp(tgt, enc0, hold_frames=2), guard=True)
            return {'engaged': False, 'guard_latched': probe['guard_latched'],
                    'reason': self._abort_diag(probe)}
        fs = self._fault_state()
        chk = sid.classify_output_engagement(
            commanded_delta_rev=tgt - enc0, encoder_delta_rev=(enc1 - enc0),
            fault_state=fs if fs is not None else sid.FAULT_NONE,
            fw_mpc_active=self._fw_mpc_active())
        reason = chk.reason
        if not chk.engaged:
            dev = self._live_deviation()
            reason = (f"{chk.reason} [commanded +{tgt - enc0:.4f} rev, encoder moved "
                      f"{enc1 - enc0:+.4f} rev; live_deviation="
                      f"{'?' if dev is None else f'{dev:+.4f}'} rev, "
                      f"lead_clamp={self._lead_clamp_bit()}]")
        # Return to enc0 before the caller's real ramp (tiny, inside the lead clamp).
        self._stream_and_sample(self._knot_ramp(tgt, enc0, hold_frames=2), guard=True)
        return {'engaged': chk.engaged, 'guard_latched': False, 'reason': reason}

    def _stream_series_disarmed(self, series):
        """Stream a flat knot series at 40 Hz with mpc_active=0 (no arm, no guard,
        no record). Output is gated while disarmed so this commands NO motion; it
        exists only to freshen the firmware staleness clock + re-baseline the interp
        base to ``series`` before an arm (leg_interp.cpp:196 stamps on every frame)."""
        period = 1.0 / self.setpoint_hz
        t0 = time.perf_counter()
        n = len(series)
        for i in range(n):
            if self._sigint:
                break
            now_us = int(time.time() * 1_000_000)
            u0 = float(series[i])
            try:
                self._send_knot(u0, u0, u0, 0.0, now_us)
            except OSError:
                break
            target = t0 + (i + 1) * period
            dt = target - time.perf_counter()
            if dt > 0:
                time.sleep(dt)

    def _warm_and_arm(self, pos: float) -> dict:
        """Stream-then-arm-then-settle: DISARMED warmup knots at ``pos`` (freshen the
        staleness clock + re-baseline the interp base to the encoder), raise mpc_active,
        THEN hold FLAT armed at ``pos`` for the arm-settle window. Returns the settle
        stream-result so a caller can detect a latch during the settle itself.

        The DISARMED warmup closes the arm-before-stream MPC_STALE race the known-good
        ``teensy_setpoint_bench.py`` idiom avoids: because the Teensy stays powered on
        Jetson 5V, ``interp_last_setpoint_us`` is stale from a prior run, so arming
        FIRST and streaming SECOND lets the firmware 250 ms staleness E-STOP latch in
        the window before the first fresh knot lands. Warming up first makes the base
        fresh AND ≈ the encoder, so the following arm can neither go stale nor jump.

        The FLAT ARMED settle (2026-07-12 approach-latch fix) then lets the firmware
        re-enable RECOVERY SLEW converge at the encoder BEFORE any climbing ramp: with
        the streamed command stationary at ``pos`` when output enables (up to ~200 ms
        after the arm — 10 Hz heartbeat + 10 Hz fault task), the 1.0 rev/s slew's target
        equals the encoder, so it converges in one tick and hands back to normal
        streaming. Without it, the old immediate 2.0 rev/s ramp outran the slew and
        walked ``u0 - enc`` into the 0.5 rev MAX_DEVIATION latch — the operator's
        "moved up slightly then guard latch" (see sysid_lib.arm_settle_series)."""
        self._stream_series_disarmed(sid.warmup_knot_series(pos, BRIDGE_ARM_WARMUP_FRAMES))
        self._arm()
        return self._stream_and_sample(sid.arm_settle_series(pos), guard=True)

    def _wait_fault_clear(self, timeout: float) -> bool:
        """Poll the heartbeat until fault_state == NONE; abort loudly on timeout
        (a fatal or an immediate re-latch)."""
        t_end = time.perf_counter() + timeout
        while time.perf_counter() < t_end and not self._sigint:
            fs = self._fault_state()
            if fs is not None and int(fs) == sid.FAULT_NONE:
                return True
            time.sleep(0.05)
        fs = self._fault_state()
        self._abort_reason = (
            f"fault_state did not clear to NONE within {timeout:.1f}s "
            f"(still {sid.fault_name(fs) if fs is not None else '?'}) — a fatal or an "
            f"immediate re-latch; power + clear the bench ODrive and retry")
        print(f"  ABORT: {self._abort_reason}")
        return False

    def _startup_clear_latch(self) -> bool:
        """Verify-and-clear a pre-existing STICKY guard E-STOP latch, DISARMED, before
        any stage. The guard latch persists across harness restarts (Teensy on Jetson
        5V, cleared only by CLEAR_ERRORS); left un-cleared it makes every run abort
        'during approach: guard latch' with the leg in CLOSED_LOOP but never moving.
        Returns True to proceed, False to abort the run (sets self._abort_reason)."""
        fs = self._fault_state()
        plan = sid.plan_startup_latch(fs if fs is not None else sid.FAULT_NONE)
        if plan.action == 'proceed':
            return True
        print(f"  STARTUP guard-latch check: {plan.reason}")
        if plan.action == 'abort':
            self._abort_reason = plan.reason
            print(f"  ABORT: {self._abort_reason}", file=sys.stderr)
            return False
        if plan.action == 'wait_recover':
            return self._wait_fault_clear(BRIDGE_STARTUP_CLEAR_TIMEOUT_S)
        # plan.action == 'clear' — a latching guard E-STOP (MAX_DEVIATION / MPC_STALE /
        # MOTOR_OVERSPEED). Clear it DISARMED after re-baselining the interp base to the
        # live encoder, so the clear + any later re-arm cannot lurch the leg.
        self._disarm()   # belt-and-braces: never touch the latch while armed
        pos, _, _, _ = self._sample()
        if pos is None:
            self._abort_reason = "no telemetry — cannot verify the encoder before clearing the latch"
            print(f"  ABORT: {self._abort_reason}", file=sys.stderr)
            return False
        # DISARMED re-baseline: stream warmup knots at the live encoder so the interp
        # base u0 tracks it (a MAX_DEVIATION latch froze u0 ~0.5 rev away). Output is
        # gated by the latch AND by mpc_active=0 — no motion.
        self._stream_series_disarmed(sid.warmup_knot_series(pos, BRIDGE_ARM_WARMUP_FRAMES))
        dev = self._live_deviation()
        if dev is not None and not sid.deviation_within_clear_tol(dev, BRIDGE_CLEAR_DEV_TOL_REV):
            self._abort_reason = (
                f"interp base still {dev:+.3f} rev from the encoder after the warmup "
                f"(> {BRIDGE_CLEAR_DEV_TOL_REV} rev) — refusing to CLEAR into an unknown "
                f"base; is another setpoint authority streaming?")
            print(f"  ABORT: {self._abort_reason}", file=sys.stderr)
            return False
        print(f"  re-baselined u0≈enc (live_deviation="
              f"{'?' if dev is None else f'{dev:+.3f}'} rev); CLEAR_ERRORS (disarmed) ...")
        try:
            self._clear_errors()
        except Exception as e:  # noqa: BLE001
            self._abort_reason = f"CLEAR_ERRORS RPC failed: {e}"
            print(f"  ABORT: {self._abort_reason}", file=sys.stderr)
            return False
        if not self._wait_fault_clear(BRIDGE_STARTUP_CLEAR_TIMEOUT_S):
            return False
        print("  guard latch cleared → fault_state=NONE")
        return True

    # -- streaming + collection ----------------------------------------------

    def _stream_and_sample(self, series, *, guard=True, writer=None,
                           revert_gains: Optional[sid.GainTriple] = None) -> dict:
        """Stream a 40 Hz knot ``u0`` series (already armed); the returned arrays and the
        CSV are sampled at the FULL ~100 Hz telemetry rate (one row per RECEIVED frame,
        Feature 1) — this send loop only publishes the current knot u0 (sample-and-hold
        cmd) and runs the guard machine. On a guard latch the firmware E-STOP is the
        backstop: consult ``self.guard`` — ``backoff_recover`` disarms + CLEAR_ERRORS (+
        reverts gains) and returns ``guard_latched=True``; ``abort`` returns
        ``aborted=True`` (ceding authority on a fatal). Never fights the guard."""
        n = len(series)
        aborted = False
        guard_latched = False
        guard_action = None
        period = 1.0 / self.setpoint_hz
        self._begin_telem_log(writer, series[0] if n else float('nan'))
        t0 = time.perf_counter()
        pos = None
        last_u0 = float('nan')
        for i in range(n):
            if self._sigint:
                aborted = True
                self._abort_reason = 'SIGINT'
                self._disarm()
                break
            t_rel = i * period
            now_us = int(time.time() * 1_000_000)
            u0 = float(series[i])
            u1 = float(series[min(i + 1, n - 1)])
            u2 = float(series[min(i + 2, n - 1)])
            v0 = (u1 - u0) / self.seg_t_s
            self._set_log_u0(u0)   # sample-and-hold cmd for the 100 Hz telemetry logger
            last_u0 = u0
            try:
                self._send_knot(u0, u1, u2, v0, now_us)
            except OSError as e:
                aborted = True
                self._abort_reason = f"setpoint send failed: {e}"
                self._disarm()
                break

            pos, vel, iq, ts = self._sample()
            fs = self._fault_state()
            if guard and fs is not None:
                act = self.guard.observe(fs)
                if act.kind == 'backoff_recover':
                    guard_latched = True
                    guard_action = act
                    _enc = pos if pos is not None else float('nan')
                    print(f"    GUARD LATCH → {act.reason} "
                          f"[u0={u0:+.4f} enc={_enc:+.4f} dev(u0-enc)={u0 - _enc:+.4f} rev, "
                          f"lead_clamp={self._lead_clamp_bit()}]")
                    self._disarm()
                    if revert_gains is not None:
                        try:
                            self._apply_gains(revert_gains)
                        except Exception:
                            pass
                    if act.clear_errors:
                        try:
                            self._clear_errors()
                        except Exception:
                            pass
                    break
                if act.kind == 'abort':
                    aborted = True
                    guard_action = act
                    self._abort_reason = act.reason
                    self._disarm()   # fatal → ceded; firmware deferred stow runs
                    break
            if t_rel > BRIDGE_ARM_VERIFY_GRACE_S and self._fw_mpc_active() is False:
                aborted = True
                self._abort_reason = (
                    "firmware mpc_active=0 after arming — another heartbeat "
                    "authority is overriding it (is a ROS2 teensy_bridge_node "
                    "running? this driver must be the sole wire authority)")
                self._disarm()
                break

            target = t0 + (i + 1) * period
            dt = target - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
        out = self._end_telem_log()
        # Keep the last streamed knot + encoder for _abort_diag when no telemetry frame
        # landed (empty log) — as scalars, so the equal-length telemetry arrays stay
        # consistent for downstream slicing (_abort_diag already tolerates empty cmd/pos).
        out['last_u0'] = last_u0
        out['last_pos'] = pos if pos is not None else float('nan')
        out['aborted'] = aborted
        out['guard_latched'] = guard_latched
        out['guard_action'] = guard_action
        return out

    def _open_csv(self, name: str):
        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir, name)
        f = open(path, 'w', newline='')
        w = csv.writer(f)
        # cmd_rev = latest streamed knot u0 (sample-and-hold); rows are ~100 Hz
        # telemetry frames (Feature 1), NOT the 40 Hz knot rate.
        w.writerow(['t_s', 'cmd_rev', 'pos_rev', 'vel_rps', 'iq_A', 'fault_state',
                    'lead_clamp'])
        return f, w, path

    def _evaluate_onset(self, arrays: dict, tail_from: float) -> dict:
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

    def _knot_ramp(self, start, target, hold_frames=None, frame_step_rev=None):
        # Default to the gentle 0.5-margin ramp (approach/return/settle); the measurement
        # steps pass the fast 0.9-margin step_frame_step so rise time reflects the loop.
        return sid.knot_step_ramp(
            start, target,
            frame_step_rev=self.frame_step_rev if frame_step_rev is None else frame_step_rev,
            hold_frames=self.hold_frames if hold_frames is None else hold_frames,
            bounds=self.bounds)

    def _enter_hold_at_center(self) -> dict:
        """CLOSED_LOOP position bring-up (RPC) → arm → stream a knot ramp from the
        live encoder pos to the operating centre (no jump at arm), then hold.
        Returns the stream result so the caller can stop if the approach latched
        the guard / aborted (it disarms itself on either)."""
        self._bringup_closed_loop()
        pos, _, _, _ = self._sample()
        start = pos if pos is not None else self.center_rev
        # Stream-then-arm-then-settle (finding: the approach must get the same care the
        # disarmed gain-apply windows got). Warm up DISARMED at the measured pos to
        # freshen the firmware staleness clock + re-baseline the interp base, arm, THEN
        # hold flat armed so the firmware re-enable recovery slew converges at the encoder
        # before any climb — arming first and immediately ramping races BOTH the 250 ms
        # MPC_STALE E-STOP and the 1.0 rev/s recovery slew (which the old 2.0 rev/s ramp
        # outran into a MAX_DEVIATION latch — the operator's "moved up slightly, stopped").
        settle = self._warm_and_arm(start)
        if settle['aborted'] or settle['guard_latched']:
            self._abort_reason = "arm-settle latched: " + (self._abort_diag(settle)
                                                           or 'guard latch')
            return settle
        # The arm-settle may have MOVED the leg: a start below the firmware STROKE_MIN
        # clamp (canbridge_config.h STROKE_MIN_REV ≈ 0.071 rev — e.g. a leg homed to the
        # retracted hardstop reference and then relaxed under it) is driven UP to STROKE_MIN
        # during the flat settle. Re-sample so the engagement verify + the approach ramp
        # start from where the leg ACTUALLY sits, not the stale pre-settle home position
        # (2026-07-12 false 'not-following' abort — the stale baseline understated tracking).
        moved, _, _, _ = self._sample()
        start = moved if moved is not None else start
        # HARD-VERIFY output engagement before trusting the big approach ramp: a tiny
        # bounded increment the encoder must follow. Turns an opaque 'guard latch' into a
        # specific unmet-condition report naming the firmware gate that is suppressing output.
        eng = self._verify_output_engaged(start)
        if not eng['engaged']:
            self._abort_reason = "output not engaged after arm — " + eng['reason']
            return {'aborted': True, 'guard_latched': eng['guard_latched'],
                    't': np.array([]), 'cmd': np.array([]), 'pos': np.array([]),
                    'vel': np.array([]), 'iq': np.array([]), 'telem_ts': np.array([]),
                    'guard_action': None}
        # Ramp to centre from where the leg LIVE sits (the verify nudged then returned it,
        # and the settle may have moved it to STROKE_MIN) — never the stale home pos.
        cur, _, _, _ = self._sample()
        series = self._knot_ramp(cur if cur is not None else start,
                                 self.center_rev, hold_frames=int(0.4 / self.seg_t_s))
        res = self._stream_and_sample(series, guard=True)
        if res['aborted'] or res['guard_latched']:
            self._abort_reason = self._abort_diag(res) or self._abort_reason
        return res

    def _bringup_closed_loop(self):
        """Legacy-faithful bring-up to CLOSED_LOOP position mode (like
        teensy_setpoint_bench --close-loop): vel/curr limits → POSITION/PASSTHROUGH
        → operational gains → CLOSED_LOOP (the ODrive auto-holds at the current pos,
        no jolt).

        Every bringup (RE-)APPLIES gains — BASELINE_GAINS from hardware_config, or
        the --gains override. pos_steps/chirp therefore NEVER inherit RAM gains a
        previous stage (e.g. a ladder backoff) left on the ODrive: without --gains
        they always characterize the production baseline loop."""
        import jugglebot.protocol_config as pc  # noqa: E402
        ax = self.axis_id
        vel_lim = float(self.vel_cap_rps)
        curr_lim = float(self.current_limit_a)
        ctrl_pos = int(pc.ODRIVE_CONTROL_MODES['POSITION'])
        in_pass = int(pc.ODRIVE_INPUT_MODES['PASSTHROUGH'])
        CLOSED_LOOP = 8
        ra = self._rpc_args
        self._rpc.call(int(self._RpcMethod.SET_VEL_CURR_LIMITS),
                       ra.encode_set_vel_curr_limits(ax, vel_lim, curr_lim))
        self._rpc.call(int(self._RpcMethod.SET_CONTROLLER_MODE),
                       ra.encode_set_controller_mode(ax, ctrl_pos, in_pass))
        bring = self.gains_override or BASELINE_GAINS
        if self.gains_override is not None:
            print(f"  ** --gains OVERRIDE active: pos_steps/chirp characterize "
                  f"pos={bring.pos_gain} vel={bring.vel_gain} "
                  f"vel_int={bring.vel_int_gain}, NOT the production baseline **")
        self._apply_gains(bring)
        self._rpc.call(int(self._RpcMethod.SET_AXIS_STATE),
                       ra.encode_set_axis_state(ax, CLOSED_LOOP))
        time.sleep(0.3)

    def _home(self):
        """Fire the firmware HOME(axis) RPC (fire-and-monitor; the move runs
        autonomously in the can-bridge HOME handler, velocity-limited + current-
        capped + hardstop-detected) and wait for the axis to settle. Production
        homing drives controller.teensy_link.homing.HomingMonitor from telemetry
        (teensy_home_bench.py); here we fire + wait for a finite, stationary encoder
        with a hard timeout, and VERIFY the leg actually MOVED (encoder travel) so a
        rejected / no-op HOME (e.g. ERR_REJECTED while mpc_active, ERR_BUS_DOWN) is not
        mistaken for success — the old 'settled encoder' check false-succeeded on a leg
        that never moved (it is already 'settled')."""
        ra = self._rpc_args
        p0, _, _, _ = self._sample()
        try:
            self._rpc.call(int(self._RpcMethod.HOME),
                           ra.encode_home(self.axis_id), retries=0)
            print(f"  HOME({self.axis_id}) accepted (OK)")
        except Exception as e:  # noqa: BLE001 — REJECTED/BUS_DOWN raise here
            print(f"  HOME RPC returned: {e} (monitoring telemetry for motion anyway)")
        # HOMING_TRAVEL_MIN_REV: firmware homing drives the leg through its full travel
        # to the hardstop (>> this), so <this seen ⇒ the move never ran (RPC rejected).
        HOMING_TRAVEL_MIN_REV = 0.10
        t_end = time.perf_counter() + 35.0
        last = None
        stable = 0
        pmin = pmax = p0 if p0 is not None else None
        while time.perf_counter() < t_end and not self._sigint:
            pos, vel, _, _ = self._sample()
            if pos is not None:
                pmin = pos if pmin is None else min(pmin, pos)
                pmax = pos if pmax is None else max(pmax, pos)
            if pos is not None and vel is not None and abs(vel) < 0.05:
                if last is not None and abs(pos - last) < 1e-3:
                    stable += 1
                    if stable > 20:
                        travel = (pmax - pmin) if (pmin is not None and pmax is not None) else 0.0
                        if travel < HOMING_TRAVEL_MIN_REV:
                            print(f"  HOME: encoder settled at {pos:+.4f} rev but only "
                                  f"{travel:.4f} rev of travel seen (< {HOMING_TRAVEL_MIN_REV}) "
                                  f"— the HOME move did NOT run (RPC rejected? mpc_active "
                                  f"still set? bus down?). NOT treating this as homed.")
                            return False
                        print(f"  HOME complete: pos={pos:+.4f} rev (travel {travel:.3f} rev)")
                        return True
                last = pos
            else:
                stable = 0
            time.sleep(0.05)
        print("  HOME: timed out waiting for a settled encoder")
        return False

    # -- Stage 1a-3a: position-loop step responses (bridge knot ramps) -------

    def stage_pos_steps(self) -> bool:
        print("\n=== Stage 1a-3a: position-loop step responses (bridge knot ramps) ===")
        print("  NOTE velocity-loop damping is INFERRED from these position-step "
              "overshoots (the bridge has no raw input_vel step).")
        entry = self._enter_hold_at_center()
        if entry['aborted'] or entry['guard_latched']:
            self._disarm()
            print(f"  ABORT during approach: {self._abort_reason or 'guard latch'}")
            return False
        # Measurement steps ramp at the FAST 0.9-margin frame step (3.6 rev/s) so rise
        # time reflects the servo, not the ramp (Feature 2); the return-to-centre ramp
        # keeps the gentle 0.5-margin step.
        plan = sid.bridge_step_plan(self.center_rev, self.pos_steps, self.bounds,
                                    frame_step_rev=self.step_frame_step_rev,
                                    seg_t_s=self.seg_t_s)
        results = []
        ok = True
        for req, sp in zip(self.pos_steps, plan):
            if sp.clamped:
                print(f"  NOTE step {req:+.3f} rev clamped to bound {sp.target_rev:.3f} rev")
            # Return to centre first, ramping from where the leg ACTUALLY sits — NOT a flat
            # series at centre. A flat [center]*(1+hold) latches the interp base at centre in
            # one 25 ms knot while the encoder is a full step (~0.14 rev) away, engaging the
            # very lead clamp the steps avoid and walking the leg back at up to the ODrive
            # vel_limit. Ramping from the measured pos keeps the return on the GENTLE
            # 0.5-margin 2.0 rev/s ramp — deliberately slower than the 0.9-margin outbound
            # measurement step (sysid_bridge finding #2).
            cur, _, _, _ = self._sample()
            self._stream_and_sample(
                self._knot_ramp(cur if cur is not None else self.center_rev,
                                self.center_rev, hold_frames=int(0.4 / self.seg_t_s)),
                guard=True)
            f, w, path = self._open_csv(f"pos_step_{req:+.3f}rev.csv")
            y0, _, _, _ = self._sample()
            y0 = y0 if y0 is not None else self.center_rev
            print(f"  step {req:+.3f} rev → target {sp.target_rev:.3f} rev "
                  f"(ramp {sp.ramp_frames} frames / {sp.ramp_duration_s * 1e3:.0f} ms, "
                  f"peak {sp.peak_frame_step_rev:.4f} rev/frame @ {sp.peak_frame_step_rev / self.seg_t_s:.1f} rev/s)  {path}")
            arrays = self._stream_and_sample(
                self._knot_ramp(self.center_rev, sp.target_rev,
                                frame_step_rev=self.step_frame_step_rev),
                guard=True, writer=w)
            f.close()
            if arrays['aborted'] or arrays['guard_latched']:
                self._disarm()
                diag = self._abort_diag(arrays)
                print(f"  ABORT: {diag or self._abort_reason or 'guard latch at baseline gains'}")
                return False
            m = sid.fit_step_response(arrays['t'], arrays['pos'], y0, sp.target_rev)
            ev = self._evaluate_onset(arrays, tail_from=arrays['t'][-1] * 0.5
                                      if arrays['t'].size else 0.0)
            # f_bw from a bridge step is still shaped by the 3.6 rev/s (0.9-margin) knot
            # ramp: a ~6 Hz pos loop's 10-90% rise (~58 ms) is comparable to or shorter than
            # the ramp for the smaller steps, so 0.35/rise measures the RAMP. Report
            # it flagged as ramp-limited with its ceiling ~1/(0.8·ramp_dur); the true loop
            # bandwidth comes from the chirp, not the step (sysid_bridge finding #3).
            f_bw = (0.35 / m.rise_time_s) if (m.rise_time_s and m.rise_time_s > 0
                                              and m.rise_time_s == m.rise_time_s) else float('nan')
            f_bw_ceiling = (1.0 / (0.8 * sp.ramp_duration_s)
                            if sp.ramp_duration_s > 0 else float('nan'))
            print(f"    rise={m.rise_time_s * 1e3:.1f} ms  overshoot={m.overshoot * 100:.1f}%  "
                  f"ζ≈{m.zeta:.2f}  f_bw≈{f_bw:.1f} Hz (ramp-limited ≤{f_bw_ceiling:.1f} Hz; "
                  f"use chirp for loop BW)  unstable={ev['onset'].unstable}")
            hf = self._motion_hf(arrays)   # Feature 1: motion-excited HF (advisory)
            self._print_hf(hf)
            row = {'requested_rev': req, 'target_rev': sp.target_rev,
                   'clamped': sp.clamped, 'ramp_frames': sp.ramp_frames,
                   'csv': os.path.basename(path), 'metrics': asdict(m),
                   'f_bw_hz': f_bw, 'f_bw_ramp_limited': True,
                   'f_bw_ramp_ceiling_hz': f_bw_ceiling,
                   'unstable': ev['onset'].unstable}
            row.update(self._hf_fields(hf))
            results.append(row)
            ok = ok and not ev['onset'].unstable
        self._disarm()
        self._manifest['stages']['pos_steps'] = results
        return ok

    # -- Stage 1a-3b: log-chirp frequency response (bridge knot stream) ------

    def stage_chirp(self) -> bool:
        print("\n=== Stage 1a-3b: log-chirp frequency response (bridge knot stream) ===")
        f1 = self._chirp_f1_bridge()
        if f1 < self.chirp_f1 - 1e-9:
            print(f"  NOTE chirp f1 bounded {self.chirp_f1:.1f} → {f1:.1f} Hz by the "
                  f"telemetry rate ({self._telem_effective_hz():.0f} Hz) / "
                  f"{self.knot_hz:.0f} Hz knot stream (the bridge cannot honestly probe higher).")
        # Multi-amplitude sweep (Feature 3): friction attenuates the small-signal gain
        # (~0.5 at 0.02 rev); a linear plant would be amplitude-independent, so sweeping
        # amplitudes is the artifact-vs-nonlinearity test — gain rising toward 1 with
        # amplitude IS the Coulomb-friction describing function.
        # Dedupe by the auto-reduced amplitude: at the knot-bounded f1 the lead-clamp
        # cap collapses e.g. 0.06 AND 0.12 to ~0.0398 rev — running both would
        # overwrite one CSV with the other and re-measure an identical stimulus.
        kept, skipped = sid.dedupe_amplitudes(
            self.chirp_amps, self._chirp_amplitude_request)
        for req, used in skipped:
            print(f"  SKIP amp {req:.4f} rev — auto-reduces to {used:.4f} rev, already "
                  f"swept (lower --chirp-f1 to fit larger amplitudes)")
        sweeps = []
        gain_vs_amp = []
        for idx, (amp_req, _) in enumerate(kept):
            res = self._run_one_chirp(amp_req, f1, idx)
            if res is None:
                return False    # aborted / guard latch during the sweep
            sweeps.append(res)
            gain_vs_amp.append({'amplitude_rev': res['amplitude_rev'],
                                'gain_by_freq': {p['freq_hz']: p['gain']
                                                 for p in res['response']}})
        self._manifest['stages']['chirp'] = {
            'f0_hz': self.chirp_f0, 'f1_hz': f1, 'f1_requested_hz': self.chirp_f1,
            'telem_effective_hz': self._telem_effective_hz(),
            'skipped_duplicate_amps': [
                {'requested_rev': r, 'reduced_rev': u} for r, u in skipped],
            'sweeps': sweeps, 'gain_vs_amplitude': gain_vs_amp,
            'small_signal_caveat': (
                "FRF at small amplitude (~0.02 rev ≈ 2.9 mm p-p) is friction/stiction-"
                "attenuated (gain ~0.5), NOT the linear closed-loop gain: it rises toward 1 "
                "with amplitude (see gain_vs_amplitude). Bins with coherence < 0.9 are "
                "unreliable (low-SNR / above the honest chirp ceiling) — do not trust their "
                "gain/phase.")}
        return True

    def _run_one_chirp(self, amp_req: float, f1: float,
                       sweep_idx: int = 0) -> Optional[dict]:
        """Run ONE chirp sweep at requested amplitude ``amp_req`` and return its per-bin
        gain/phase (single-bin lock-in) + Welch H1/coherence, or None on abort/latch.
        Each amplitude gets its own per-amplitude kinematic auto-reduction;
        ``sweep_idx`` keys the CSV so reduced-amplitude collisions cannot overwrite."""
        entry = self._enter_hold_at_center()
        if entry['aborted'] or entry['guard_latched']:
            self._disarm()
            print(f"  ABORT during approach: {self._abort_reason or 'guard latch'}")
            return None
        # Knots on the knot-rate grid; the firmware Hermite interpolates to 500 Hz.
        t_grid = np.arange(0.0, self.chirp_dur, self.seg_t_s)
        series, amp_used = sid.chirp_position_series(
            t_grid, self.center_rev, self._chirp_amplitude_request(amp_req),
            self.chirp_f0, f1, self.chirp_dur, self.bounds)
        pv, pa = sid.chirp_peak_kinematics(amp_used, f1)
        if amp_used < amp_req - 1e-9:
            print(f"  NOTE chirp amp reduced {amp_req:.4f} → {amp_used:.4f} rev "
                  f"to fit stroke + vel/accel/lead caps")
        print(f"  chirp {self.chirp_f0:.1f}→{f1:.1f} Hz, amp={amp_used:.4f} rev "
              f"({amp_used * hw_mm_per_rev() * 2:.2f} mm p-p), peak {pv:.2f} rev/s / "
              f"{pa:.0f} rev/s², {self.chirp_dur:.1f} s, {len(series)} knots")
        f, w, path = self._open_csv(f"chirp_s{sweep_idx}_a{amp_used:.3f}.csv")
        self._arm()
        arrays = self._stream_and_sample(series, guard=True, writer=w)
        f.close()
        self._disarm()
        if arrays['aborted'] or arrays['guard_latched']:
            diag = self._abort_diag(arrays)
            print(f"  ABORT: {diag or self._abort_reason or 'guard latch during chirp'}")
            return None
        bins = self._response_bins()
        gains, phases = sid.estimate_frequency_response(
            arrays['t'], arrays['cmd'], arrays['pos'], bins)
        fs = sid.median_sample_rate(arrays['t'])
        wg, wp, coh = sid.welch_h1_coherence(arrays['cmd'], arrays['pos'], fs, bins)
        print(f"    (Welch fs≈{fs:.0f} Hz)  f[Hz]  gain   phase[deg]  coh  welch_gain")
        points = []
        for fr, g, ph, cg, c in zip(bins, gains, phases, wg, coh):
            flag = " <LOW-COH" if c < 0.9 else ""
            print(f"    {fr:8.1f}  {g:5.2f}  {ph:8.1f}  {c:4.2f}  {cg:6.2f}{flag}")
            points.append({'freq_hz': fr, 'gain': float(g), 'phase_deg': float(ph),
                           'welch_gain': float(cg), 'coherence': float(c),
                           'low_coherence': bool(c < 0.9)})
        hf = self._motion_hf(arrays)       # Feature 1: motion-excited HF (advisory)
        self._print_hf(hf)
        out = {'csv': os.path.basename(path), 'amplitude_rev': amp_used,
               'welch_fs_hz': fs, 'response': points}
        out.update(self._hf_fields(hf))
        return out

    # -- Stage 1b: escalate-until-unstable gain ladder (bridge) --------------

    def stage_ladder(self) -> bool:
        print("\n=== Stage 1b: escalate-until-unstable gain ladder (bridge) ===")
        print(f"  guard backstop ACTIVE: firmware MAX_DEVIATION {BRIDGE_MAX_DEVIATION_REV} "
              f"rev + MPC-staleness {BRIDGE_MPC_STALENESS_S}s E-STOP; a latch → back off "
              f"+ CLEAR_ERRORS (budget {self.guard.max_recoveries}).")
        # Extended ceiling-hunt ladder (Feature 4): base 25→90 + high rungs 110→210.
        # High rungs FREEZE vel_int (windup during current saturation reads as a
        # MAX_DEVIATION latch; the 125:1 ratio rule is a HOLD-tier heuristic, wrong here)
        # unless --vint-ratio restores scaling; their vel_gain climbs with pos_gain.
        # --rungs replaces the table with explicit single-candidate gap-fill points
        # (off-diagonal buzz attribution — the table's first candidates all sit on
        # the ζ-constant diagonal and cannot separate pos_gain from vel_gain).
        if self.custom_rungs:
            rungs = sid.custom_rung_table(self.custom_rungs)
            print(f"  EXPLICIT rungs (--rungs): "
                  f"{[(r.pos_gain, r.vel_gain_lo, r.vel_int_gain) for r in rungs]}")
        else:
            rungs = sid.extended_ladder(vint_ratio=self.vint_ratio,
                                        from_pos_gain=self.from_pg)
            print(f"  rungs: {[r.pos_gain for r in rungs]}  "
                  f"(vel_int {'ratio pos/%.0f' % self.vint_ratio if self.vint_ratio else 'FROZEN %.2f above 90' % sid.FROZEN_VEL_INT_GAIN})")
        survey = bool(self.custom_rungs)
        lad = sid.GainLadder(rungs=rungs, zeta_target=self.zeta_target,
                             bw_clear_hz=self.bw_clear_hz, survey=survey)
        if abs(self.quiescent_secs - QUIESCENT_HOLD_S) > 1e-9:
            print(f"  quiescent-hold window: {self.quiescent_secs:.1f} s "
                  f"(--quiescent-secs; default {QUIESCENT_HOLD_S})")
        entry = self._enter_hold_at_center()
        if entry['aborted'] or entry['guard_latched']:
            self._auto_backoff(None, self._abort_reason or 'guard latch during approach')
            self._finish_ladder(lad, [], None)
            return False
        rungs_log = []
        quiescent_floors = []
        winner: Optional[sid.GainTriple] = None
        while not lad.stopped:
            rung = lad.current_rung()
            cands = self._rung_vel_candidates(rung)
            # Revert target after a latch: at/above pos_gain 130 the firmware re-enable
            # slew must run against SOFT baseline gains (a still-stiff last-good rung
            # re-latches and burns the recovery budget) — ladder_revert_target picks it.
            # Survey points always revert to BASELINE (another survey point's gains
            # are not a validated safe state).
            revert = (BASELINE_GAINS if survey else
                      sid.ladder_revert_target(rung.pos_gain, lad.last_good,
                                               BASELINE_GAINS))
            print(f"\n  rung {lad.index}: pos_gain={rung.pos_gain} "
                  f"vel_int={rung.vel_int_gain:.3f} "
                  f"vel_gain∈{{{', '.join('%.2f' % c for c in cands)}}} "
                  f"revert→pos={revert.pos_gain}")
            best = None
            best_zeta = None
            best_unstable = False
            best_bw = None
            stable_found = False
            tgt = self.bounds.clamp(self.center_rev + self.ladder_step)
            for vg in cands:
                triple = sid.GainTriple(rung.pos_gain, float(vg), rung.vel_int_gain)
                # Apply gains DISARMED: the two blocking SET_*_GAIN RPCs (up to ~1.5 s
                # worst-case with retries) can straddle the firmware 250 ms MPC-staleness
                # window while armed, latching a (now correctly-classified latching)
                # MPC_STALE E-STOP — which would burn a guard recovery and gate the
                # following step's output. Dropping mpc_active=0 across the RPC gap
                # suppresses the staleness check (fault_machine.cpp:351 gates on
                # s_mpc_active), then re-arm to stream (sysid_bridge finding #1).
                self._disarm()
                self._apply_gains(triple)
                # Return to centre for a clean labelled step, ramping from where the leg
                # ACTUALLY sits — a flat series at centre would latch the interp base a step
                # away from the encoder and engage the lead clamp (sysid_bridge finding #2).
                cur, _, _, _ = self._sample()
                cur = cur if cur is not None else self.center_rev
                # Stream-then-arm: warm up DISARMED at the live pos to freshen the staleness
                # clock + re-baseline the interp base before raising mpc_active, so the re-arm
                # after the disarmed gain-apply gap cannot race the 250 ms MPC_STALE E-STOP.
                self._warm_and_arm(cur)
                self._stream_and_sample(
                    self._knot_ramp(cur, self.center_rev,
                                    hold_frames=int(0.4 / self.seg_t_s)),
                    guard=True, revert_gains=revert)
                # Quiescent-hold buzz check (Feature 4): FLAT armed hold at centre BEFORE the
                # step — a vel_gain dragged too high amplifies encoder noise into a hold buzz
                # the post-step tail misses. vel-ripple RMS gates the climb; iq is reported
                # but gated only with fast-iq firmware (stock iq is on-change-aliased).
                q = self._quiescent_hold_check(self.center_rev, revert)
                quiescent_floors.append({'pos_gain': rung.pos_gain, 'vel_gain': float(vg),
                                         'vel_rms': q['vel_rms'], 'iq_rms': q['iq_rms'],
                                         'buzz': q['buzz']})
                if q['buzz']:
                    print(f"    vel_gain={vg:.2f}: QUIESCENT BUZZ "
                          f"(vel_rms={q['vel_rms']:.4f} rev/s, iq_rms={q['iq_rms']:.3f} A) "
                          f"→ rung unstable, stop climb")
                    best = triple
                    best_zeta = None
                    best_unstable = True
                    best_bw = None
                    break
                # vint in the name: --rungs points sharing (pos, vel_gain) at
                # different vel_int otherwise overwrite each other's CSV (the
                # 2026-07-12 de-confound run lost 2 of its 3 step records).
                f, w, path = self._open_csv(
                    f"ladder_p{rung.pos_gain:.0f}_v{vg:.2f}_i{rung.vel_int_gain:.2f}.csv")
                y0, _, _, _ = self._sample()
                y0 = y0 if y0 is not None else self.center_rev
                arrays = self._stream_and_sample(
                    self._knot_ramp(self.center_rev, tgt,
                                    frame_step_rev=self.step_frame_step_rev),
                    guard=True, writer=w, revert_gains=revert)
                f.close()
                if arrays['aborted']:
                    self._auto_backoff(revert, self._abort_reason)
                    self._finish_ladder(lad, rungs_log, lad.last_good)
                    self._manifest['stages'].setdefault('ladder', {})[
                        'quiescent_floors'] = quiescent_floors
                    return False
                m = sid.fit_step_response(arrays['t'], arrays['pos'], y0, tgt)
                ev = self._evaluate_onset(arrays, tail_from=arrays['t'][-1] * 0.5
                                          if arrays['t'].size else 0.0)
                unstable = (arrays['guard_latched'] or ev['onset'].unstable
                            or ev['iq_cycle'].cycling or m.overshoot > OVERSHOOT_ONSET)
                bw = (0.35 / m.rise_time_s) if (m.rise_time_s and m.rise_time_s > 0) else None
                tag = " GUARD-LATCH" if arrays['guard_latched'] else ""
                print(f"    vel_gain={vg:.2f}: ζ≈{m.zeta:.2f} "
                      f"overshoot={m.overshoot * 100:.1f}% unstable={unstable}{tag}")
                best, best_zeta, best_unstable, best_bw = triple, m.zeta, unstable, bw
                if arrays['guard_latched']:
                    # Never keep searching a rung that latched the guard.
                    break
                if not unstable and m.zeta >= self.zeta_target:
                    stable_found = True
                    break

            if survey and best_unstable:
                # Park safe between survey points: disarmed at BASELINE so buzzing
                # gains never hold an armed leg through the bookkeeping gap (the
                # stalled knot stream would otherwise latch MPC_STALE at 250 ms).
                self._disarm()
                self._apply_gains(BASELINE_GAINS)
            onset_for_record = sid.OnsetResult(
                unstable=best_unstable, ripple_rms=0.0, oscillation_score=0.0, reasons=[])
            dec = lad.record(best, onset_for_record, best_zeta, bandwidth_hz=best_bw,
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
                self._disarm()          # disarm before the final apply (no armed RPC gap)
                if winner:
                    self._apply_gains(winner)
                elif survey:
                    self._apply_gains(BASELINE_GAINS)   # survey has no winner — park safe
                break
        self._finish_ladder(lad, rungs_log, winner)
        self._manifest['stages'].setdefault('ladder', {})[
            'quiescent_floors'] = quiescent_floors
        return True

    def _rung_vel_candidates(self, rung) -> List[float]:
        """vel_gain search points for a rung: explicit --rungs points test exactly
        their one value; high (>90) table rungs the ``[lo, 1.25·lo, 1.6·lo]`` climbing
        set; base rungs the linspace search."""
        return sid.rung_vel_gain_candidates(rung, self.n_vel)

    def _quiescent_hold_check(self, center: float,
                              revert: sid.GainTriple) -> dict:
        """Hold FLAT armed at ``center`` for the quiescent window (default 0.75 s;
        --quiescent-secs stretches it for soak checks) and measure vel-ripple RMS
        (buzz gate) + iq-ripple RMS (reported; gated only with fast-iq firmware). A guard
        latch during the hold counts as buzz (the hold itself went unstable)."""
        frames = max(1, int(round(self.quiescent_secs / self.seg_t_s)))
        res = self._stream_and_sample(sid.arm_settle_series(center, frames),
                                      guard=True, revert_gains=revert)
        if res['aborted'] or res['guard_latched']:
            return {'buzz': True, 'vel_rms': float('nan'), 'iq_rms': float('nan'),
                    'latched': True}
        vel = res['vel'][np.isfinite(res['vel'])]
        iq = res['iq'][np.isfinite(res['iq'])]
        vrms = sid.ripple_rms(vel) if vel.size else 0.0
        irms = sid.ripple_rms(iq) if iq.size else 0.0
        q = sid.classify_quiescent_buzz(
            vrms, irms, iq_onset=self.iq_ripple_threshold,
            fast_iq_available=self.fast_iq_available)
        return {'buzz': q.buzz, 'vel_rms': q.vel_rms, 'iq_rms': q.iq_rms,
                'latched': False}

    def _auto_backoff(self, last_good, reason):
        target = last_good if last_good is not None else BASELINE_GAINS
        print(f"  AUTO-BACKOFF ({reason}) → reverting to pos={target.pos_gain} "
              f"vel={target.vel_gain} vel_int={target.vel_int_gain}, disarm")
        self._disarm()
        try:
            self._apply_gains(target)
            self._clear_errors()   # leave the leg un-latched at the safe triple
        except Exception:
            pass

    def _finish_ladder(self, lad, rungs_log, winner):
        self._manifest['stages']['ladder'] = {
            'stop_reason': lad.stop_reason,
            'winner': asdict(winner) if winner else None,
            'guard_recoveries_used': self.guard.recoveries_used,
            'rungs': rungs_log,
        }
        if winner:
            print(f"\n  Ladder winner: pos_gain={winner.pos_gain} "
                  f"vel_gain={winner.vel_gain} vel_int={winner.vel_int_gain} "
                  f"(stop: {lad.stop_reason})")
        else:
            print(f"\n  Ladder produced no winner (stop: {lad.stop_reason})")

    # -- sustained-tracking discriminant (--mode track) ----------------------

    def stage_track(self) -> bool:
        print("\n=== Sustained-tracking discriminant (--mode track) ===")
        print("  the instrument for the robot's ~6 Hz question: drive a CONTINUOUSLY-moving "
              "reference (the S4 excitation class) long enough for a limit cycle to develop, "
              "then extract the dominant residual freq the same way the S4 forensics did.")
        if self.track_peak_vel > sid.TRACK_LEAD_CLAMP_WARN_RPS:
            print(f"  WARNING: track peak vel {self.track_peak_vel:.1f} rev/s > "
                  f"{sid.TRACK_LEAD_CLAMP_WARN_RPS:.0f} — the reference peak deliberately RIDES "
                  f"the firmware lead clamp (pos_gain×MAX_LEAD). That IS the S4 regime and is "
                  f"SAFE on the bench: the clamp itself bounds cmd-enc to {BRIDGE_MAX_LEAD_REV} "
                  f"rev << MAX_DEVIATION {BRIDGE_MAX_DEVIATION_REV} rev.")
        amp = min(sid.TRACK_AMP_MAX_REV,
                  sid.symmetric_amplitude_limit(self.center_rev, self.bounds))
        f_sine = sid.track_sine_frequency(self.track_peak_vel, amp)
        print(f"  reference: {self.track_wave}, peak_vel={self.track_peak_vel:.2f} rev/s, "
              f"amp={amp:.3f} rev about {self.center_rev:.3f}, {self.track_secs:.1f} s"
              + (f", sine f≈{f_sine:.2f} Hz" if self.track_wave == 'sine' else ""))
        entry = self._enter_hold_at_center()
        if entry['aborted'] or entry['guard_latched']:
            self._disarm()
            print(f"  ABORT during approach: {self._abort_reason or 'guard latch'}")
            return False
        results = []
        for gp in self.track_gains:
            # Reuse the ladder's disarmed-gain-apply + warm/arm + revert plumbing.
            self._disarm()
            self._apply_gains(gp)
            cur, _, _, _ = self._sample()
            cur = cur if cur is not None else self.center_rev
            self._warm_and_arm(cur)
            self._stream_and_sample(
                self._knot_ramp(cur, self.center_rev, hold_frames=int(0.4 / self.seg_t_s)),
                guard=True, revert_gains=BASELINE_GAINS)
            series = sid.track_reference_series(
                center_rev=self.center_rev, amplitude_rev=amp,
                peak_vel_rps=self.track_peak_vel, duration_s=self.track_secs,
                seg_t_s=self.seg_t_s, wave=self.track_wave, bounds=self.bounds)
            f, w, path = self._open_csv(
                f"track_p{gp.pos_gain:.0f}_v{gp.vel_gain:.2f}.csv")
            print(f"  gain pt pos={gp.pos_gain} vel={gp.vel_gain} "
                  f"vel_int={gp.vel_int_gain}: {len(series)} knots → {path}")
            arrays = self._stream_and_sample(series, guard=True, writer=w,
                                             revert_gains=BASELINE_GAINS)
            f.close()
            if arrays['aborted'] or arrays['guard_latched']:
                self._disarm()
                diag = self._abort_diag(arrays)
                print(f"  ABORT: {diag or self._abort_reason or 'guard latch during track'}")
                self._manifest['stages']['track'] = {'points': results, 'aborted': True}
                return False
            # Steady window: exclude the C1 ramp in/out (~TRACK_RAMP_S each end).
            t = arrays['t']
            if t.size:
                steady = (t >= sid.TRACK_RAMP_S) & (t <= (t[-1] - sid.TRACK_RAMP_S))
            else:
                steady = np.zeros(0, dtype=bool)
            lc = arrays.get('lead_clamp')
            has_lc = lc is not None and lc.size and np.any(np.isfinite(lc))
            resid = sid.analyze_tracking_residual(
                t[steady], arrays['cmd'][steady], arrays['pos'][steady],
                vel=arrays['vel'][steady], iq=arrays['iq'][steady],
                lead_clamp=(lc[steady] if has_lc else None))
            cf = ('?' if resid.clamp_engaged_frac is None
                  else f"{resid.clamp_engaged_frac:.3f}")
            print(f"    dominant residual {resid.dominant_freq_hz:.2f} Hz (fs≈{resid.fs_hz:.0f}); "
                  f"peaks {[(round(fr, 2), round(a, 4)) for fr, a in resid.peaks]}; "
                  f"reversals={resid.reversals}; iq[min/mean/max]="
                  f"{resid.iq_min:.2f}/{resid.iq_mean:.2f}/{resid.iq_max:.2f} A; "
                  f"clamp_frac={cf}")
            hf = self._motion_hf(arrays, mask=steady)   # Feature 1: motion-excited HF (advisory)
            self._print_hf(hf)
            row = {'gains': asdict(gp), 'csv': os.path.basename(path),
                   'fs_hz': resid.fs_hz,
                   'dominant_freq_hz': resid.dominant_freq_hz,
                   'peaks': resid.peaks, 'reversals': resid.reversals,
                   'iq_min': resid.iq_min, 'iq_mean': resid.iq_mean,
                   'iq_max': resid.iq_max, 'iq_rms': resid.iq_rms,
                   'clamp_engaged_frac': resid.clamp_engaged_frac}
            row.update(self._hf_fields(hf))
            results.append(row)
        self._disarm()
        print("\n  Discriminant (gain point → dominant residual freq); compare to S4 "
              "5.9-6.1 Hz + ~12.3 Hz harmonic:")
        print("    pos_gain  vel_gain  dom_freq[Hz]  reversals  clamp_frac")
        for r in results:
            cf = ('?' if r['clamp_engaged_frac'] is None
                  else f"{r['clamp_engaged_frac']:.3f}")
            print(f"    {r['gains']['pos_gain']:8.0f}  {r['gains']['vel_gain']:8.2f}  "
                  f"{r['dominant_freq_hz']:12.2f}  {r['reversals']:9d}  {cf}")
        self._manifest['stages']['track'] = {
            'wave': self.track_wave, 'peak_vel_rps': self.track_peak_vel,
            'amplitude_rev': amp, 'duration_s': self.track_secs,
            's4_reference': ('5.9-6.1 Hz fundamental, ~12.3 Hz 2nd harmonic '
                             '(logbook/2026-07-10-s4-stutter-guard-forensics-recovery-stack.md)'),
            'points': results}
        return True

    # -- Feature 1: motion-excited HF onset (wired into every moving stage) ---

    def _motion_hf(self, arrays: dict, mask=None) -> sid.MotionHFMetrics:
        """Compute the motion-excited HF metrics on a moving-stage window. ``mask`` (a
        boolean array over the telemetry samples) restricts to the steady window when a
        stage excludes a ramp; else the whole window is used."""
        t = arrays['t']
        vel = arrays['vel']
        iq = arrays['iq']
        if mask is not None and t.size:
            t, vel, iq = t[mask], vel[mask], iq[mask]
        return sid.motion_hf_metrics(t, vel, iq, self.knot_hz)

    def _stroke_hf(self, arrays: dict, spec) -> sid.MotionHFMetrics:
        """HF metrics for one stroke: per moving window (out / return, ring-down
        padded), worst-by-iqHF — the stationary holds are excluded so they cannot
        dilute the band-RMS. Falls back to the whole window for holdless specs or
        windows too short to measure."""
        wins = sid.stroke_moving_windows(spec)
        if wins is None or not arrays['t'].size:
            return self._motion_hf(arrays)
        t = arrays['t']
        best = None
        for lo, hi in wins:
            m = (t >= lo) & (t <= hi)
            if np.count_nonzero(m) < 16:
                continue
            hf = self._motion_hf(arrays, mask=m)
            if best is None or hf.hf_iq_rms > best.hf_iq_rms:
                best = hf
        return best if best is not None else self._motion_hf(arrays)

    def _print_hf(self, hf: sid.MotionHFMetrics, prefix: str = "    "):
        lo, hi, notch = hf.band
        nstr = "" if notch is None else " \\[%.0f-%.0f]" % (notch[0], notch[1])
        flag = "  ** HF-ONSET **" if hf.onset else ""
        print("%sHF[%.0f-%.0f Hz%s] velHF=%.3f rps iqHF=%.3f A  rail>9A=%.2f%% "
              ">9.5A=%.2f%%%s"
              % (prefix, lo, hi, nstr, hf.hf_vel_rms, hf.hf_iq_rms,
                 hf.rail_duty_9 * 100.0, hf.rail_duty_9_5 * 100.0, flag))

    @staticmethod
    def _hf_fields(hf: sid.MotionHFMetrics) -> dict:
        lo, hi, notch = hf.band
        return {'hf_vel_rms': hf.hf_vel_rms, 'hf_iq_rms': hf.hf_iq_rms,
                'hf_band': [lo, hi, (list(notch) if notch is not None else None)],
                'rail_duty_9': hf.rail_duty_9, 'rail_duty_9_5': hf.rail_duty_9_5,
                'hf_onset': hf.onset}

    @staticmethod
    def _clamp_frac(arrays: dict, mask=None) -> Optional[float]:
        lc = arrays.get('lead_clamp')
        if lc is None or not lc.size:
            return None
        if mask is not None:
            lc = lc[mask]
        lc = lc[np.isfinite(lc)]
        return float(np.mean(lc > 0.5)) if lc.size else None

    def _recent_telem(self, secs: float) -> Optional[dict]:
        """Snapshot the last ``secs`` of the live telemetry log buffer (teleop rolling
        metrics). Returns arrays keyed like a stream result, or None if no frames yet."""
        with self._lock:
            buf = list(self._log_buf)
        if not buf:
            return None
        arr = np.asarray(buf, float)
        t = arr[:, 0]
        mask = t >= (t[-1] - secs)
        cols = ('t', 'cmd', 'pos', 'vel', 'iq', 'fault', 'lead_clamp')
        return {c: arr[mask, i] for i, c in enumerate(cols)}

    def _teleop_park(self, cmd_now: float):
        """Gentle slew-to-centre park (bounded 3 s) before disarm — the profiled way out
        of teleop. Bails on SIGINT (a Ctrl-C wants an immediate disarm, like every other
        stage)."""
        period = self.seg_t_s
        cmd = float(cmd_now)
        v = 0.0
        t_end = time.perf_counter() + 3.0
        while time.perf_counter() < t_end and not self._sigint:
            cmd, v = sid.slew_toward(cmd, v, self.center_rev, self.teleop_max_vel,
                                     HARD_ACCEL_CAP_RPS2, period)
            self._set_log_u0(cmd)
            try:
                self._send_knot(cmd, cmd, cmd, v, int(time.time() * 1_000_000))
            except OSError:
                break
            if abs(cmd - self.center_rev) < 1e-3 and abs(v) < 1e-3:
                break
            time.sleep(period)

    # -- Feature 2: aggressive realistic-stroke battery (--mode strokes) ------

    def stage_strokes(self) -> bool:
        print("\n=== Aggressive realistic-stroke battery (--mode strokes) ===")
        print("  min-jerk (quintic) point-to-point strokes — the surface the single "
              "1.75 Hz/4.4 rev/s sine was one corner of. Sweeps amplitude × peak_vel, "
              "asymmetric throw/catch, and a sustained back-to-back sequence (the S4 class).")
        battery = sid.stroke_battery(
            self.stroke_amps, self.stroke_vels, vel_cap=self.vel_cap_rps,
            accel_cap=HARD_ACCEL_CAP_RPS2, bounds=self.bounds, center=self.center_rev)
        if not battery:
            print("  REJECT: no stroke in the battery fits the stroke bounds symmetric "
                  "about centre — lower --stroke-amps or move --center.", file=sys.stderr)
            self._manifest['stages']['strokes'] = {'battery': [], 'points': [],
                                                    'aborted': True}
            return False
        print(f"  battery: {len(battery)} strokes across gains {self.track_gains}")
        entry = self._enter_hold_at_center()
        if entry['aborted'] or entry['guard_latched']:
            self._disarm()
            print(f"  ABORT during approach: {self._abort_reason or 'guard latch'}")
            return False
        results = []
        for gp in self.track_gains:
            # Reuse the ladder/track disarmed-apply + warm/arm + revert plumbing exactly.
            self._disarm()
            self._apply_gains(gp)
            cur, _, _, _ = self._sample()
            cur = cur if cur is not None else self.center_rev
            self._warm_and_arm(cur)
            approach = self._stream_and_sample(
                self._knot_ramp(cur, self.center_rev, hold_frames=int(0.4 / self.seg_t_s)),
                guard=True, revert_gains=BASELINE_GAINS)
            if approach['aborted']:
                self._disarm()
                print(f"  ABORT: {self._abort_diag(approach) or self._abort_reason}")
                self._manifest['stages']['strokes'] = {
                    'battery': [asdict(s) for s in battery], 'points': results,
                    'aborted': True}
                return False
            if approach['guard_latched']:
                # Recovered — skip this gain point's battery (budget checked below).
                if self.guard.recoveries_used > self.guard.max_recoveries:
                    self._auto_backoff(BASELINE_GAINS, 'guard budget exhausted at approach')
                    break
                continue
            print(f"\n  --- gain pt pos={gp.pos_gain} vel={gp.vel_gain} "
                  f"vel_int={gp.vel_int_gain} ---")
            for spec in battery:
                series = sid.stroke_series(spec, self.center_rev, self.seg_t_s, self.bounds)
                # Series-level feasibility gate (closes the sizing-bug class): an
                # infeasible commanded series latches MAX_DEVIATION at ANY gains and
                # burns the recovery budget — skip it loudly instead.
                pk = sid.series_peak_velocity(series, self.seg_t_s)
                if pk > self.vel_cap_rps * 1.05:
                    print(f"  SKIP {spec.label}: commanded peak {pk:.2f} rev/s exceeds "
                          f"the {self.vel_cap_rps} rev/s cap — generator sizing bug, "
                          f"stroke not streamed")
                    results.append({'gains': asdict(gp), 'stroke': asdict(spec),
                                    'skipped_infeasible_peak_rps': pk})
                    continue
                f, w, path = self._open_csv(
                    f"stroke_p{gp.pos_gain:.0f}_{spec.label}.csv")
                arrays = self._stream_and_sample(series, guard=True, writer=w,
                                                 revert_gains=BASELINE_GAINS)
                f.close()
                if arrays['aborted']:
                    self._disarm()
                    print(f"  ABORT: {self._abort_diag(arrays) or self._abort_reason}")
                    self._manifest['stages']['strokes'] = {
                        'battery': [asdict(s) for s in battery], 'points': results,
                        'aborted': True}
                    return False
                if arrays['guard_latched']:
                    print(f"  {spec.label}: GUARD LATCH → backed off to BASELINE "
                          f"(recovery {self.guard.recoveries_used}/{self.guard.max_recoveries})")
                    if self.guard.recoveries_used > self.guard.max_recoveries:
                        self._auto_backoff(BASELINE_GAINS, 'guard budget exhausted')
                        self._manifest['stages']['strokes'] = {
                            'battery': [asdict(s) for s in battery], 'points': results,
                            'aborted': True}
                        return False
                    # Budget remains → move on to the next GAIN point (re-approach at top).
                    break
                # Per-stroke metrics.
                t = arrays['t']
                cmd = arrays['cmd']
                pos = arrays['pos']
                fin = np.isfinite(pos) & np.isfinite(cmd)
                err = np.abs(cmd[fin] - pos[fin]) if np.any(fin) else np.array([0.0])
                err_rms = float(np.sqrt(np.mean(err ** 2)))
                err_peak = float(np.max(err))
                target = self.center_rev + spec.amplitude_rev
                ov = sid.overshoot_fraction(pos[fin], self.center_rev, target) if np.any(fin) else 0.0
                # HF over the MOVING phases only (review finding: the two 0.4 s holds
                # dilute whole-window band-RMS ~0.7x and deflate the onset flag).
                hf = self._stroke_hf(arrays, spec)
                cf = self._clamp_frac(arrays)
                cfs = '?' if cf is None else f"{cf:.3f}"
                print(f"  {spec.label} (T_out={spec.T_out_s * 1e3:.0f}ms "
                      f"pk_a={spec.peak_accel_out_rps2:.0f} rps²): "
                      f"{sid.format_stroke_error(err_rms, err_peak, hw_mm_per_rev())} "
                      f"overshoot={ov * 100:.1f}% clamp={cfs}")
                self._print_hf(hf, prefix="      ")
                row = {'gains': asdict(gp), 'stroke': asdict(spec),
                       'csv': os.path.basename(path),
                       'track_err_rms_rev': err_rms, 'track_err_peak_rev': err_peak,
                       'overshoot': float(ov), 'clamp_engaged_frac': cf}
                row.update(self._hf_fields(hf))
                results.append(row)
        self._disarm()
        self._manifest['stages']['strokes'] = {
            'battery': [asdict(s) for s in battery], 'points': results,
            'hf_advisory': ('hf_onset is ADVISORY (motion-excited 20-45 Hz energy, '
                            'knot-staircase notched) — never a hard abort')}
        return True

    # -- Feature 3: interactive SpaceMouse teleop + live gains (--mode teleop) -

    def _spacemouse_candidates(self, pyspacemouse):
        """``(name, path)`` for each CONNECTED supported 3Dconnexion device, or None
        when HID enumeration is unavailable. ``list_available_devices()`` is the
        SUPPORTED-model catalog (13 entries regardless of what's plugged in), so it
        must be intersected with the live HID enumeration. Two spacemice on this
        bench made the bare ``open()`` grab whichever enumerated first — silently
        reading the wrong stick (operator report, 2026-07-12)."""
        try:
            import easyhid
            cat = {(v, p): n for n, v, p in pyspacemouse.list_available_devices()}
            out = []
            for dev in easyhid.Enumeration().find():
                if (dev.vendor_id, dev.product_id) in cat:
                    path = (dev.path.decode() if isinstance(dev.path, bytes)
                            else str(dev.path))
                    out.append((cat[(dev.vendor_id, dev.product_id)], path))
            return out
        except Exception:  # noqa: BLE001 — enumeration is best-effort
            return None

    def _open_spacemouse(self, pyspacemouse) -> bool:
        """Open the RIGHT spacemouse: enumerate connected supported devices, apply
        --teleop-device (case-insensitive substring), and REFUSE ambiguity instead
        of silently reading the wrong stick. Stores the selection for reconnects."""
        cands = self._spacemouse_candidates(pyspacemouse)
        if cands is None:
            if self.teleop_device:
                print("REJECT: HID enumeration unavailable — cannot honour "
                      "--teleop-device; unplug the other device instead.",
                      file=sys.stderr)
                return False
            print("  NOTE HID enumeration unavailable — bare open() fallback")
            try:
                return bool(pyspacemouse.open())
            except Exception as e:  # noqa: BLE001
                print(f"  SpaceMouse open() raised: {e}")
                return False
        for n, p in cands:
            print(f"  connected: {n} ({p})")
        pick = cands
        if self.teleop_device:
            # Match against the NAME and the PATH: wireless mice behind identical
            # 'Universal Receiver' dongles all share one name, so /dev/hidrawN is
            # often the only usable selector.
            want = self.teleop_device.lower()
            pick = [(n, p) for n, p in cands
                    if want in n.lower() or want in p.lower()]
            if not pick:
                print(f"REJECT: no connected spacemouse matches --teleop-device "
                      f"'{self.teleop_device}'", file=sys.stderr)
                return False
        if not pick:
            print("REJECT: no supported spacemouse connected.", file=sys.stderr)
            return False
        if len(pick) > 1:
            sel = self._probe_spacemice(pyspacemouse, pick)
            if sel is None:
                print("REJECT: no interface responded to the wiggle probe — is the "
                      "desired mouse awake? (wireless mice sleep; nudge it first). "
                      "Or select explicitly: --teleop-device /dev/hidrawN. NOTE: if "
                      "the ROS spacemouse_handler node is running it is reading one "
                      "of them.", file=sys.stderr)
                return False
            self._sm_selected = sel
            return True         # the probe leaves the winning device open
        name, path = pick[0]
        self._sm_selected = (name, path)
        return self._reopen_spacemouse(pyspacemouse)

    def _probe_spacemice(self, pyspacemouse, cands, wiggle_s: float = 3.0):
        """Sequentially open each candidate interface and watch for stick motion —
        the only reliable discriminator when every interface reports the same model
        name. Returns the responding ``(name, path)`` (LEFT OPEN) or None."""
        print(f"  PROBING {len(cands)} interfaces — WIGGLE the spacemouse you want "
              f"to use (each watched {wiggle_s:.0f} s)...")
        for name, path in cands:
            try:
                if not bool(pyspacemouse.open(device=name, path=path)):
                    continue
            except Exception:  # noqa: BLE001
                continue
            print(f"    watching {path} ...")
            t_end = time.perf_counter() + wiggle_s
            hit = False
            while time.perf_counter() < t_end:
                try:
                    st = pyspacemouse.read()
                except Exception:  # noqa: BLE001
                    break
                if st is not None and max(
                        abs(getattr(st, a, 0.0) or 0.0)
                        for a in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')) > 0.15:
                    hit = True
                    break
                time.sleep(0.01)
            if hit:
                print(f"    MOTION on {path} — selected")
                return (name, path)
            try:
                pyspacemouse.close()
            except Exception:  # noqa: BLE001
                pass
        return None

    def _reopen_spacemouse(self, pyspacemouse) -> bool:
        """(Re)open the previously selected device — reconnects must not silently
        drift to the other spacemouse."""
        sel = getattr(self, '_sm_selected', None)
        try:
            if sel:
                name, path = sel
                try:
                    ok = bool(pyspacemouse.open(device=name, path=path))
                except TypeError:      # older pyspacemouse without path kwarg
                    ok = bool(pyspacemouse.open(device=name))
                if ok:
                    print(f"  SpaceMouse OPEN: {name}")
                return ok
            return bool(pyspacemouse.open())
        except Exception as e:  # noqa: BLE001
            print(f"  SpaceMouse open raised: {e}")
            return False

    def stage_teleop(self) -> bool:
        import queue
        print("\n=== Interactive teleop (--mode teleop): SpaceMouse + live gains ===")
        print("  commands (type + Enter): 'g PG VG VINT' apply gains (near-rest only) | "
              "'b' baseline | 'r' re-arm after latch | 'q' quit (ramp to centre + disarm)")
        try:
            import pyspacemouse
        except Exception as e:  # noqa: BLE001 — device lib absent
            print(f"REJECT: pyspacemouse import failed ({e})", file=sys.stderr)
            return False
        if not self._open_spacemouse(pyspacemouse):
            print("REJECT: could not open the intended SpaceMouse.", file=sys.stderr)
            return False
        sm = pyspacemouse
        sm_open = True
        sm_last_open = 0.0

        entry = self._enter_hold_at_center()
        if entry['aborted'] or entry['guard_latched']:
            self._disarm()
            try:
                sm.close()
            except Exception:  # noqa: BLE001
                pass
            print(f"  ABORT during approach: {self._abort_reason or 'guard latch'}")
            return False

        active = self.gains_override or BASELINE_GAINS
        f, w, path = self._open_csv("teleop.csv")
        self._begin_telem_log(w, self.center_rev)
        timeline: List[dict] = []
        self._gain_inflight = False
        gain_lock = threading.Lock()
        apply_lock = threading.Lock()
        # Latch epoch (mutated under gain_lock): a guard latch increments it and
        # re-applies BASELINE, cancelling/superseding any in-flight gain apply —
        # BASELINE must be the LAST gain write after a latch (review HIGHs: the
        # worker's deferred RPCs would otherwise race the latch handler's revert).
        epoch = [0]

        def apply_worker(triple, my_epoch):
            nonlocal active
            status = 'applied'
            try:
                # apply_lock serializes whole gain-apply sequences against the latch
                # handler, so two _apply_gains never interleave on the wire.
                with apply_lock:
                    with gain_lock:
                        stale = my_epoch != epoch[0]
                    if stale:
                        status = 'cancelled_by_latch'
                    else:
                        # RUNS IN A WORKER THREAD: the two blocking SET_*_GAIN RPCs
                        # (:5006, up to ~1.5 s with retries) must NOT stall the knot
                        # stream (:5005) or the 250 ms MPC-staleness E-STOP fires.
                        # The main loop keeps streaming throughout.
                        self._apply_gains(triple)
            except Exception as ex:  # noqa: BLE001
                status = 'failed'
                print(f"  gain apply FAILED: {ex}")
            finally:
                with gain_lock:
                    self._gain_inflight = False
            if status == 'applied':
                with gain_lock:
                    if my_epoch != epoch[0]:
                        # A latch fired DURING our RPCs; its BASELINE lands after us
                        # (it is blocked on apply_lock) — never claim our triple active.
                        status = 'superseded_by_latch'
                    else:
                        active = triple
            timeline.append({'t': time.time(), 'gains': asdict(triple),
                             'status': status})
            if status == 'applied':
                print(f"  gains applied: pos={triple.pos_gain} vel={triple.vel_gain} "
                      f"vel_int={triple.vel_int_gain} — stick live")
            elif status != 'failed':
                print(f"  gain apply {status} — BASELINE is restored by the latch handler")

        cmd_q: 'queue.Queue' = queue.Queue()

        def reader():
            try:
                for line in sys.stdin:
                    cmd_q.put(line)
            except Exception:  # noqa: BLE001
                pass
        threading.Thread(target=reader, daemon=True).start()

        cmd = self.center_rev
        cmdvel = 0.0
        target = self.center_rev
        z = 0.0
        latched = False
        quitting = False
        parked = False
        rc_ok = True
        period = self.seg_t_s
        t0 = time.perf_counter()
        i = 0
        last_metric = time.perf_counter()
        pos = vel = None

        print("  TELEOP LIVE — move the SpaceMouse (z axis). Ctrl-C to park + exit.")
        while not self._sigint:
            # 1. read stick (fast) / reconnect at 1 Hz
            if sm_open and not latched:
                try:
                    st = sm.read()
                    if st is not None:
                        z = float(st.z)
                except Exception:  # noqa: BLE001
                    sm_open = False
                    enc, _, _, _ = self._sample()
                    target = enc if enc is not None else target
                    sm_last_open = time.perf_counter()
                    print("  SpaceMouse read failed — target FROZEN at encoder; "
                          "retrying open at 1 Hz")
            if not sm_open and time.perf_counter() - sm_last_open >= 1.0:
                sm_last_open = time.perf_counter()
                try:
                    if self._reopen_spacemouse(sm):
                        sm_open = True
                        print("  SpaceMouse reconnected — stick live")
                except Exception:  # noqa: BLE001
                    pass

            # 2. target (frozen while latched, disconnected, or a gain apply is in
            #    flight: the near-rest gate runs at ENQUEUE, but the worker's RPCs
            #    land up to ~1.5 s later — freezing the stick until they land keeps
            #    the leg at rest AT APPLY TIME, so a pos_gain step can never hit a
            #    moving leg (review HIGH #1). Quit-parking also waits them out.)
            with gain_lock:
                inflight = self._gain_inflight
            if not latched and sm_open and not inflight:
                target = sid.teleop_target(z, self.center_rev, self.teleop_range,
                                           self.bounds)
            if quitting and not inflight:
                target = self.center_rev

            # 3. slew (never a step) + stream one knot
            cmd, cmdvel = sid.slew_toward(cmd, cmdvel, target, self.teleop_max_vel,
                                          HARD_ACCEL_CAP_RPS2, period)
            now_us = int(time.time() * 1_000_000)
            u1 = cmd + cmdvel * period
            u2 = cmd + 2.0 * cmdvel * period
            self._set_log_u0(cmd)
            try:
                self._send_knot(cmd, u1, u2, cmdvel, now_us)
            except OSError as e:
                print(f"  send failed: {e} — disarm + exit")
                rc_ok = False
                break

            # 4. sample + guard
            pos, vel, iq, ts = self._sample()
            fstate = self._fault_state()
            if fstate is not None and not latched:
                act = self.guard.observe(fstate)
                if act.kind == 'backoff_recover':
                    latched = True
                    enc, _, _, _ = self._sample()
                    target = enc if enc is not None else cmd
                    cmd = target
                    cmdvel = 0.0
                    self._disarm()
                    with gain_lock:
                        epoch[0] += 1   # cancel/supersede any in-flight gain apply
                    # apply_lock waits out an in-flight worker's RPCs (≤ ~1.5 s) so
                    # BASELINE is guaranteed the LAST gain write. Blocking the loop
                    # here is safe: output is E-STOPped and we just disarmed, so the
                    # stalled knot stream has nothing to actuate.
                    with apply_lock:
                        try:
                            self._apply_gains(BASELINE_GAINS)
                            active = BASELINE_GAINS
                        except Exception:  # noqa: BLE001
                            pass
                    if act.clear_errors:
                        try:
                            self._clear_errors()
                        except Exception:  # noqa: BLE001
                            pass
                    timeline.append({'t': time.time(), 'event': 'guard_latch',
                                     'reason': act.reason})
                    print(f"\n  ** GUARD LATCH: {act.reason} **\n"
                          f"  target FROZEN at encoder, reverted to BASELINE, CLEAR_ERRORS.\n"
                          f"  type 'r' + Enter to re-arm (never auto-resumes from the stick).")
                elif act.kind == 'abort':
                    print(f"\n  ABORT: {act.reason} — parking + disarm")
                    rc_ok = False
                    break

            # 5. consume stdin commands
            try:
                while True:
                    parsed = sid.parse_teleop_command(cmd_q.get_nowait())
                    if parsed.kind == 'quit':
                        quitting = True
                        print("  quitting — ramping to centre, then disarm...")
                    elif parsed.kind == 'empty':
                        pass
                    elif parsed.kind == 'invalid':
                        print(f"  ? {parsed.error}")
                    elif parsed.kind == 'rearm':
                        if not latched:
                            print("  not latched — 'r' only re-arms after a guard latch")
                        else:
                            enc, _, _, _ = self._sample()
                            start = enc if enc is not None else self.center_rev
                            settle = self._warm_and_arm(start)
                            if settle['aborted'] or settle['guard_latched']:
                                print(f"  re-arm FAILED: {self._abort_diag(settle)} "
                                      f"— still latched; fix + 'r' again")
                            else:
                                curp, _, _, _ = self._sample()
                                cmd = curp if curp is not None else start
                                cmdvel = 0.0
                                target = cmd
                                latched = False
                                # _warm_and_arm's internal stream ended the session
                                # telemetry log — re-attach the same CSV writer so
                                # logging + rolling HF metrics continue (fresh t/window).
                                self._begin_telem_log(w, cmd)
                                timeline.append({'t': time.time(), 'event': 're-armed'})
                                print("  re-armed — stick live")
                    else:  # 'gains' or 'baseline'
                        triple = (BASELINE_GAINS if parsed.kind == 'baseline'
                                  else parsed.gains)
                        if latched:
                            print("  latched — re-arm ('r') before swapping gains")
                        else:
                            with gain_lock:
                                busy = self._gain_inflight
                            if busy:
                                print("  gain apply in flight — wait for it to finish")
                            else:
                                p2, v2, _, _ = self._sample()
                                dev = cmd - (p2 if p2 is not None else cmd)
                                if not sid.gains_swap_allowed(
                                        v2 if v2 is not None else 0.0, dev):
                                    print("  hold still to swap gains (|vel|<0.05 rps "
                                          "AND |cmd-pos|<0.02 rev)")
                                else:
                                    with gain_lock:
                                        self._gain_inflight = True
                                        my_epoch = epoch[0]
                                    timeline.append({'t': time.time(),
                                                     'gains': asdict(triple),
                                                     'status': 'requested'})
                                    threading.Thread(target=apply_worker,
                                                     args=(triple, my_epoch),
                                                     daemon=True).start()
                                    print("  gains applying — stick FROZEN until "
                                          "they land")
            except queue.Empty:
                pass

            # 6. live metrics ~1 Hz (Feature 1 on the last ~2 s)
            if time.perf_counter() - last_metric >= 1.0:
                last_metric = time.perf_counter()
                rec = self._recent_telem(2.0)
                hf = None
                cf = None
                if rec is not None:
                    hf = sid.motion_hf_metrics(rec['t'], rec['vel'], rec['iq'], self.knot_hz)
                    cf = self._clamp_frac(rec)
                cfs = '?' if cf is None else f"{cf:.2f}"
                stt = 'LATCHED' if latched else ('DISCONN' if not sm_open else 'live')
                pv = pos if pos is not None else float('nan')
                vv = vel if vel is not None else float('nan')
                line = (f"  [{stt}] pos={pv:+.3f} vel={vv:+.2f} tgt={target:+.3f} "
                        f"gains={active.pos_gain}/{active.vel_gain}/{active.vel_int_gain}")
                if hf is not None:
                    lo, hi, _ = hf.band
                    line += (f" | HF[{lo:.0f}-{hi:.0f}] velHF={hf.hf_vel_rms:.3f} "
                             f"iqHF={hf.hf_iq_rms:.3f} rail>9A={hf.rail_duty_9 * 100:.1f}% "
                             f"clamp={cfs}")
                print(line)
                if hf is not None and hf.onset:
                    print("  **** HF-ONSET: motion-excited 20-45 Hz vibration ADVISORY "
                          "(velHF/iqHF over threshold) — consider backing off the gains ****")

            # 7. quit convergence → break once home
            if quitting and abs(cmd - self.center_rev) < 1e-3 and abs(cmdvel) < 1e-3:
                parked = True
                break

            # 8. pace to the knot rate
            i += 1
            tt = t0 + i * period
            dt = tt - time.perf_counter()
            if dt > 0:
                time.sleep(dt)

        # -- teardown: gentle park (unless already home or interrupted), then disarm --
        if not parked and rc_ok and not self._sigint:
            try:
                self._teleop_park(cmd)
            except Exception:  # noqa: BLE001
                pass
        self._end_telem_log()
        try:
            f.close()
        except Exception:  # noqa: BLE001
            pass
        self._disarm()
        try:
            sm.close()
        except Exception:  # noqa: BLE001
            pass
        self._manifest['stages']['teleop'] = {
            'teleop_range_rev': self.teleop_range,
            'teleop_max_vel_rps': self.teleop_max_vel,
            'accel_cap_rps2': HARD_ACCEL_CAP_RPS2,
            'deadband': sid.TELEOP_DEADBAND,
            'csv': os.path.basename(path),
            'gain_timeline': timeline,
            'final_gains': asdict(active)}
        print(f"\n  teleop session ended ({len(timeline)} timeline events).")
        return rc_ok

    # -- manifest ------------------------------------------------------------

    def write_manifest(self):
        os.makedirs(self.output_dir, exist_ok=True)
        if self.telem_rate is not None:
            self._manifest['telemetry_rate'] = asdict(self.telem_rate)
        self._manifest['finished'] = datetime.datetime.now().isoformat(timespec='seconds')
        path = os.path.join(self.output_dir, 'manifest.json')
        with open(path, 'w') as f:
            json.dump(self._manifest, f, indent=2)
        print(f"\n  Manifest: {path}")

    # -- dry-run plan --------------------------------------------------------

    def print_plan(self, modes: List[str]):
        stream_port, rpc_port = _bridge_ports()
        print("\n" + "=" * 64)
        print("DRY-RUN — bench-leg system-ID plan, Path BRIDGE (no socket, no motor)")
        print("=" * 64)
        print(f"  axis={self.axis_id}  peer={BRIDGE_TEENSY_IP}  "
              f"stream={stream_port} rpc={rpc_port}")
        print(f"  caps: stroke≤{self.stroke_cap_rev} rev, vel≤{self.vel_cap_rps} rev/s, "
              f"accel≤{HARD_ACCEL_CAP_RPS2} rev/s², curr≤{self.current_limit_a} A")
        print(f"  bounds=[{self.bounds.lo_rev:.3f}, {self.bounds.hi_rev:.3f}] rev  "
              f"center={self.center_rev:.3f} rev")
        print(f"  knot stream: {self.setpoint_hz:.0f} Hz, seg_t={self.seg_t_s * 1e3:.1f} ms; "
              f"approach step ≤ {self.frame_step_rev:.4f} rev "
              f"(≤{BRIDGE_LEAD_MARGIN_FRAC:.0%} of the {BRIDGE_MAX_LEAD_REV} rev lead clamp, "
              f"velocity-anchored {BRIDGE_RAMP_VEL_TARGET_RPS:.1f} rev/s "
              f"⇒ {self.frame_step_rev / self.seg_t_s:.2f} rev/s); measurement step ≤ "
              f"{self.step_frame_step_rev:.4f} rev (≤{BRIDGE_STEP_LEAD_MARGIN_FRAC:.0%}, "
              f"anchored {BRIDGE_STEP_RAMP_VEL_TARGET_RPS:.1f} rev/s "
              f"⇒ {self.step_frame_step_rev / self.seg_t_s:.2f} rev/s)")
        if self.gains_override is not None:
            g = self.gains_override
            print(f"  ** --gains OVERRIDE: pos_steps/chirp will characterize "
                  f"pos={g.pos_gain} vel={g.vel_gain} vel_int={g.vel_int_gain} "
                  f"(NOT the production baseline) **")
        if self.knot_hz > BRIDGE_SETPOINT_HZ + 1e-9:
            print(f"  ** knot-hz {self.knot_hz:.0f} REQUIRES the BENCH_SYSID_BUILD firmware "
                  f"(a parallel agent is adding it) — the stock 40 Hz-knot firmware will NOT "
                  f"honour a {self.knot_hz:.0f} Hz stream. **")
        print(f"  guard BACKSTOP: firmware MAX_DEVIATION {BRIDGE_MAX_DEVIATION_REV} rev + "
              f"MPC-staleness {BRIDGE_MPC_STALENESS_S}s E-STOP; latch → back off + "
              f"CLEAR_ERRORS (budget {self.guard.max_recoveries})")
        print(f"  telemetry: measured live over {BRIDGE_WARMUP_S:.0f}s RX-only warmup FIRST; "
              f"dry-run assumes nominal {BRIDGE_TELEM_NOMINAL_HZ:.0f} Hz")
        eff = self._telem_effective_hz()
        f1 = self._chirp_f1_bridge()
        print(f"    honest chirp top f1 = min(req {self.chirp_f1:.0f}, telem "
              f"{sid.honest_chirp_top_freq(eff):.1f}, knot "
              f"{sid.knot_stream_top_freq(self.seg_t_s):.1f}) = {f1:.1f} Hz")
        print(f"  onset: ripple>{self.ripple_threshold} rev/s AND osc>{self.osc_threshold}; "
              f"iq-ripple>{self.iq_ripple_threshold} A; overshoot>{OVERSHOOT_ONSET * 100:.0f}%")

        if self.do_home:
            print("\n  [home] firmware HOME(0) RPC (fire-and-monitor) before stages")
        if 'pos_steps' in modes:
            print("\n  [pos_steps] position-loop knot-ramp step responses "
                  "(SOURCE the inferred velocity-loop damping; FAST measurement ramp):")
            for req, sp in zip(self.pos_steps,
                               sid.bridge_step_plan(self.center_rev, self.pos_steps,
                                                    self.bounds,
                                                    frame_step_rev=self.step_frame_step_rev,
                                                    seg_t_s=self.seg_t_s)):
                flag = " (CLAMPED)" if sp.clamped else ""
                print(f"    step {req:+.3f} → target {sp.target_rev:.3f} rev{flag}: "
                      f"ramp {sp.ramp_frames} knots / {sp.ramp_duration_s * 1e3:.0f} ms, "
                      f"peak {sp.peak_frame_step_rev:.4f} rev/frame "
                      f"(≤ step lead {self.step_frame_step_rev:.4f} = "
                      f"{self.step_frame_step_rev / self.seg_t_s:.1f} rev/s)")
        if 'chirp' in modes:
            print(f"\n  [chirp] log sine sweep ({self.setpoint_hz:.0f} Hz knots → 500 Hz "
                  f"Hermite), amplitudes {self.chirp_amps} rev:")
            kept, skipped = sid.dedupe_amplitudes(
                self.chirp_amps, self._chirp_amplitude_request)
            for amp_req, _ in kept:
                amp = self._chirp_amplitude_request(amp_req)
                amp_used = sid.clamp_amplitude(self.center_rev, amp, self.bounds)
                pv, pa = sid.chirp_peak_kinematics(amp_used, f1)
                n_knots = len(np.arange(0.0, self.chirp_dur, self.seg_t_s))
                print(f"    amp {amp_req:.3f}→{amp_used:.4f} rev: {self.chirp_f0:.1f}→"
                      f"{f1:.1f} Hz, dur={self.chirp_dur:.1f}s, {n_knots} knots, "
                      f"peak {pv:.2f} rev/s / {pa:.0f} rev/s²")
            for req, used in skipped:
                print(f"    amp {req:.3f} rev SKIPPED — auto-reduces to {used:.4f} rev, "
                      f"duplicate sweep (lower --chirp-f1 to fit larger amplitudes)")
            print(f"    response bins ≤ f1: {self._response_bins()} Hz "
                  f"(the 6 Hz pos-loop question is inside this band; Welch coherence per bin)")
        if 'ladder' in modes:
            if self.custom_rungs:
                rungs = sid.custom_rung_table(self.custom_rungs)
                print(f"\n  [ladder] EXPLICIT --rungs gap-fill points, "
                      f"ζ_target={self.zeta_target}:")
            else:
                rungs = sid.extended_ladder(vint_ratio=self.vint_ratio,
                                            from_pos_gain=self.from_pg)
                print(f"\n  [ladder] escalate-until-unstable, ζ_target={self.zeta_target} "
                      f"(vel_int {'ratio pos/%.0f' % self.vint_ratio if self.vint_ratio else 'FROZEN %.2f > 90' % sid.FROZEN_VEL_INT_GAIN}):")
            for i, rung in enumerate(rungs):
                cands = [f"{c:.2f}" for c in self._rung_vel_candidates(rung)]
                print(f"    rung {i}: pos_gain={rung.pos_gain} "
                      f"vel_int={rung.vel_int_gain:.3f}  vel_gain∈{{{', '.join(cands)}}}")
            sp = sid.bridge_step_plan(self.center_rev, [self.ladder_step], self.bounds,
                                      frame_step_rev=self.step_frame_step_rev,
                                      seg_t_s=self.seg_t_s)[0]
            print(f"    center step per rung: {self.ladder_step:+.3f} rev → knot ramp "
                  f"{sp.ramp_frames} frames / {sp.ramp_duration_s * 1e3:.0f} ms; "
                  f"quiescent-hold buzz check {self.quiescent_secs:g}s before each step; "
                  f"latch → revert (BASELINE ≥ pg {sid.LADDER_BASELINE_REVERT_POS_GAIN:.0f})")
        if 'track' in modes:
            amp = min(sid.TRACK_AMP_MAX_REV,
                      sid.symmetric_amplitude_limit(self.center_rev, self.bounds))
            print(f"\n  [track] sustained-tracking discriminant ({self.track_wave}, "
                  f"peak_vel {self.track_peak_vel:.2f} rev/s, amp {amp:.3f} rev, "
                  f"{self.track_secs:.1f}s):")
            for gp in self.track_gains:
                print(f"    gain pt pos={gp.pos_gain} vel={gp.vel_gain} "
                      f"vel_int={gp.vel_int_gain}")
            print("    residual periodogram at the TRUE sample rate → dominant freq vs "
                  "S4 5.9-6.1 Hz (+~12.3 Hz harmonic)")
        if 'strokes' in modes:
            print(f"\n  [strokes] aggressive min-jerk stroke battery across gains "
                  f"{self.track_gains}:")
            print(f"    amps {self.stroke_amps} rev × peak_vels {self.stroke_vels} rev/s "
                  f"(vel≤{self.vel_cap_rps}, accel≤{HARD_ACCEL_CAP_RPS2:.0f} rps²); "
                  f"holds {sid.STROKE_HOLD_S_DEFAULT}s; asymmetric slow-return "
                  f"{sid.STROKE_SLOW_RETURN_RPS} rev/s; sustained ×{sid.STROKE_SUSTAINED_N}")
            battery = sid.stroke_battery(
                self.stroke_amps, self.stroke_vels, vel_cap=self.vel_cap_rps,
                accel_cap=HARD_ACCEL_CAP_RPS2, bounds=self.bounds, center=self.center_rev)
            if not battery:
                print("    (NO feasible strokes — every amplitude exceeds the symmetric "
                      f"stroke room {sid.symmetric_amplitude_limit(self.center_rev, self.bounds):.3f} "
                      "rev about centre; lower --stroke-amps or move --center)")
            for spec in battery:
                alim = " ACCEL-LIMITED" if spec.accel_limited else ""
                if spec.kind == 'asymmetric':
                    dur = f"out {spec.T_out_s * 1e3:.0f}ms / return {spec.T_return_s * 1e3:.0f}ms"
                    vinfo = (f"v_out {spec.out_peak_vel_rps:.2f} / v_ret "
                             f"{spec.return_peak_vel_rps:.2f} rev/s")
                elif spec.kind == 'sustained':
                    dur = f"{spec.n_strokes}× {spec.T_out_s * 1e3:.0f}ms"
                    vinfo = f"v {spec.out_peak_vel_rps:.2f} rev/s"
                else:
                    dur = f"{spec.T_out_s * 1e3:.0f}ms each way"
                    vinfo = f"v {spec.out_peak_vel_rps:.2f} rev/s"
                print(f"    {spec.label}: A={spec.amplitude_rev:.2f} rev  {vinfo}  "
                      f"peak_accel {spec.peak_accel_out_rps2:.0f} rps²  {dur}{alim}")
            print("    per-stroke: tracking err RMS/peak, overshoot, HF-onset (advisory), "
                  "rail duty, clamp frac; guard latch → backoff, park BASELINE, next gain pt")
        if 'teleop' in modes:
            lo = self.bounds.lo_rev + sid.TELEOP_CLAMP_MARGIN_REV
            hi = self.bounds.hi_rev - sid.TELEOP_CLAMP_MARGIN_REV
            print("\n  [teleop] interactive SpaceMouse + live gains:")
            print(f"    mapping: z∈[-1,1], deadband {sid.TELEOP_DEADBAND} → target = "
                  f"{self.center_rev:.2f} + z·{self.teleop_range} rev, clamped to "
                  f"[{lo:.3f}, {hi:.3f}] rev (z=0 → centre {self.center_rev:.2f})")
            print(f"    slew planner (never a step): vel ≤ {self.teleop_max_vel} rev/s, "
                  f"accel ≤ {HARD_ACCEL_CAP_RPS2:.0f} rps², streamed at {self.setpoint_hz:.0f} Hz")
            print("    commands: 'g PG VG VINT' (gains, near-rest only: |vel|<%.2f AND "
                  "|cmd-pos|<%.2f) | 'b' baseline | 'r' re-arm | 'q' quit"
                  % (sid.TELEOP_GAIN_SWAP_VEL_TOL_RPS, sid.TELEOP_GAIN_SWAP_DEV_TOL_REV))
            print("    safety: gains applied via a ONE-SHOT worker thread (RPC must not "
                  "stall the knot stream); guard latch → freeze target, revert BASELINE, "
                  "CLEAR_ERRORS, require 'r' (never auto-resume from the stick); "
                  "Ctrl-C → park + disarm")
            try:
                import pyspacemouse as _sm
                cands = self._spacemouse_candidates(_sm)
            except Exception:  # noqa: BLE001
                cands = None
            if cands is not None:
                for n, p in cands:
                    print(f"    connected: {n} ({p})")
                if len(cands) > 1 and not self.teleop_device:
                    print("    ** 2+ interfaces connected — the live run will PROBE: "
                          "wiggle the stick you want when prompted (or pre-select "
                          "with --teleop-device /dev/hidrawN) **")
                elif not cands:
                    print("    (no supported spacemouse currently connected)")
            if self.teleop_device:
                print(f"    device filter: --teleop-device '{self.teleop_device}'")
            print("    live ~1 Hz metrics: pos/vel/gains + rolling HF vel/iq RMS, rail "
                  "duty, clamp frac, LOUD HF-onset warning")
        print("\n  DRY-RUN complete — validation passed, no socket opened, no motor commanded.")

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
            print("\n[SIGINT] requesting abort → disarm")
            self._sigint = True
            try:
                self._disarm()
            except Exception:
                pass
        signal.signal(signal.SIGINT, sigint)

        if self.knot_hz > BRIDGE_SETPOINT_HZ + 1e-9:
            print(f"\n  ****************************************************************\n"
                  f"  ** knot-hz {self.knot_hz:.0f} REQUIRES the BENCH_SYSID_BUILD firmware. **\n"
                  f"  ** The stock 40 Hz-knot firmware will NOT honour a {self.knot_hz:.0f} Hz  **\n"
                  f"  ** stream (a parallel agent is adding that build; this harness   **\n"
                  f"  ** does NOT touch the Teensy firmware).                          **\n"
                  f"  ****************************************************************")

        stage_fns = {
            'pos_steps': self.stage_pos_steps,
            'chirp': self.stage_chirp,
            'ladder': self.stage_ladder,
            'track': self.stage_track,
            'strokes': self.stage_strokes,
            'teleop': self.stage_teleop,
        }
        rc = 0
        try:
            self._setup_transport()
            # Wait for the first telemetry frame.
            t0 = time.time()
            while self._sample()[0] is None and time.time() - t0 < 3.0:
                time.sleep(0.05)
            if self._sample()[0] is None:
                print("ABORT: no telemetry from the Teensy.", file=sys.stderr)
                return 2
            pos, _, _, _ = self._sample()
            fs = self._fault_state()
            print(f"  Link up: pos={pos:+.4f} rev  fault_state={fs} "
                  f"({sid.fault_name(fs) if fs is not None else '?'})")
            # MEASURE the telemetry rate FIRST (it bounds the honest chirp top freq).
            rate = self._measure_telemetry_rate()
            print(f"  Telemetry: mean {rate.mean_hz:.1f} Hz (median {rate.median_hz:.1f}, "
                  f"jitter {rate.jitter_rms_s * 1e3:.2f} ms, p95 gap "
                  f"{rate.p95_interval_s * 1e3:.1f} ms) → effective {rate.effective_hz:.1f} Hz; "
                  f"honest chirp f1 = {self._chirp_f1_bridge():.1f} Hz")
            # Verify-and-clear a pre-existing STICKY guard latch BEFORE any stage — else
            # every run aborts 'during approach: guard latch' with the leg never moving.
            if not self._startup_clear_latch():
                return 2
            if self.do_home:
                self._home()
            for mode in modes:
                ok = stage_fns[mode]()
                self._manifest['stages'].setdefault(mode, {})
                if not ok:
                    rc = 2
                    print(f"  Stage '{mode}' reported a fault/onset/latch — stopping.")
                    break
        except KeyboardInterrupt:
            rc = 2
        except Exception as exc:  # noqa: BLE001
            print(f"\nFATAL: {exc}", file=sys.stderr)
            rc = 1
        finally:
            try:
                self.write_manifest()
            except Exception as exc:  # noqa: BLE001
                print(f"  (manifest write failed: {exc})", file=sys.stderr)
            self._teardown_transport()

        if rc == 0:
            print("\nPASS: all requested stages completed.")
        elif rc == 2:
            print(f"\nABORT: {self._abort_reason or 'stage fault/onset'}", file=sys.stderr)
        return rc


def hw_mm_per_rev() -> float:
    """Bench-leg geometry (mm per motor rev) — for human-readable chirp p-p."""
    return 71.5708  # single_leg_test.py:106 (measured)


# Path-specific mode expansion: Path BRIDGE has NO vel_steps (the velocity-loop
# response is inferred from position-step overshoot — no raw input_vel step exists).
MODE_GROUPS_DIRECT = {'all': ['vel_steps', 'pos_steps', 'chirp']}
MODE_GROUPS_BRIDGE = {'all': ['pos_steps', 'chirp']}


def _default_output_dir() -> str:
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    return os.path.join(_PROJECT_ROOT, 'temp', 'probes', f'bench_sysid_{ts}')


def _parse_float_list(s: str) -> List[float]:
    return [float(x) for x in s.split(',') if x.strip()]


def _parse_track_gains(s: str) -> List[sid.GainTriple]:
    """Parse ``'pg:vg:vint,...'`` into GainTriples (the --track-gains grammar)."""
    triples: List[sid.GainTriple] = []
    for tok in s.split(','):
        tok = tok.strip()
        if not tok:
            continue
        parts = tok.split(':')
        if len(parts) != 3:
            raise ValueError("track gain %r must be pg:vg:vint" % tok)
        triples.append(sid.GainTriple(float(parts[0]), float(parts[1]), float(parts[2])))
    if not triples:
        raise ValueError("--track-gains produced no gain points")
    return triples


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--mode', default='all',
                   choices=['vel_steps', 'pos_steps', 'chirp', 'ladder', 'track',
                            'strokes', 'teleop', 'all'],
                   help="stage(s) to run (default: all = pos_steps,chirp on BRIDGE). "
                        "'track' = sustained-tracking discriminant (~6 Hz question); "
                        "'strokes' = aggressive min-jerk stroke battery; "
                        "'teleop' = interactive SpaceMouse + live gains")
    p.add_argument('--path', default='bridge', choices=['bridge', 'direct'],
                   help="drive path (default: bridge). 'direct' (socketcan) is "
                        "UNAVAILABLE for live runs on this Jetson — direct CAN was "
                        "wound down 2026-07-11; --path direct is accepted only with "
                        "--dry-run (historical plan view).")
    p.add_argument('--axis', type=int, default=0,
                   help='axis/CAN node id (default 0 — the bench ODrive on CAN3)')
    p.add_argument('--interface', default='socketcan', help='(Path DIRECT only)')
    p.add_argument('--channel', default='can0', help='(Path DIRECT only)')
    p.add_argument('--max-recoveries', type=int, default=BRIDGE_MAX_RECOVERIES,
                   help=f'(Path BRIDGE) guard-latch CLEAR_ERRORS budget before the '
                        f'ladder aborts (default {BRIDGE_MAX_RECOVERIES})')
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
                   help='single chirp amplitude in rev (auto-reduced to fit stroke); '
                        'superseded by --chirp-amps for the multi-amplitude sweep')
    p.add_argument('--chirp-amps', default=None,
                   help="comma-separated chirp amplitudes in rev for the multi-amplitude "
                        "sweep (default %s). Friction attenuates the small-signal gain; a "
                        "linear plant would be amplitude-independent."
                        % ','.join(str(a) for a in DEFAULT_CHIRP_AMPS_REV))
    p.add_argument('--knot-hz', type=float, default=BRIDGE_SETPOINT_HZ,
                   choices=[40.0, 100.0],
                   help="(Path BRIDGE) knot stream rate. 100 lifts the honest chirp ceiling "
                        "8→20 Hz but REQUIRES the BENCH_SYSID_BUILD firmware (default 40)")
    p.add_argument('--vint-ratio', type=float, default=None,
                   help="(ladder) restore vel_int = pos_gain/RATIO scaling on the high rungs "
                        "(default: vel_int FROZEN at %.2f above pos_gain 90)"
                        % sid.FROZEN_VEL_INT_GAIN)
    p.add_argument('--from-pg', type=float, default=None,
                   help="(ladder) start at the first rung with pos_gain ≥ this "
                        "(skip re-proving the low rungs)")
    p.add_argument('--rungs', default=None,
                   help="(ladder) EXPLICIT gain points 'pg:vg:vint,...' replacing the "
                        "built-in table — single-candidate rungs through the same "
                        "quiescent-buzz → step → onset pipeline. The off-diagonal "
                        "gap-fill knob (e.g. '130:0.50:0.72,110:0.55:0.72' attributes "
                        "a buzz onset to pos_gain vs vel_gain)")
    p.add_argument('--quiescent-secs', type=float, default=QUIESCENT_HOLD_S,
                   help="(ladder) quiescent-hold buzz window per rung in seconds "
                        "(default %.2f; stretch to 120-180 for a winner soak check)"
                        % QUIESCENT_HOLD_S)
    p.add_argument('--gains', default=None,
                   help="(pos_steps/chirp) 'pg:vg:vint' triple applied at bringup instead "
                        "of the production baseline — REQUIRED to step/chirp-characterize "
                        "a tuned candidate (without it those stages always measure "
                        "baseline %s/%s/%s)" % (BASELINE_GAINS.pos_gain,
                                                BASELINE_GAINS.vel_gain,
                                                BASELINE_GAINS.vel_int_gain))
    p.add_argument('--fast-iq', action='store_true',
                   help="(ladder quiescent-buzz check) gate on iq-ripple too — ONLY valid "
                        "with a fast-iq bench firmware; stock iq is on-change-aliased")
    p.add_argument('--track-wave', default='sine', choices=['sine', 'triangle'],
                   help="(track) reference waveform (default sine, C1 ramped in/out)")
    p.add_argument('--track-peak-vel', type=float, default=sid.TRACK_PEAK_VEL_DEFAULT_RPS,
                   help="(track) peak velocity in rev/s (default %.1f; up to %.1f, where "
                        ">%.1f deliberately rides the firmware lead clamp = the S4 regime)"
                        % (sid.TRACK_PEAK_VEL_DEFAULT_RPS, sid.TRACK_PEAK_VEL_MAX_RPS,
                           sid.TRACK_LEAD_CLAMP_WARN_RPS))
    p.add_argument('--track-secs', type=float, default=sid.TRACK_SECS_DEFAULT,
                   help="(track) duration in s (default %.1f)" % sid.TRACK_SECS_DEFAULT)
    p.add_argument('--track-gains', default=DEFAULT_TRACK_GAINS,
                   help="(track/strokes) gain points 'pg:vg:vint,...' (default '%s' = the "
                        "robot's production gains)" % DEFAULT_TRACK_GAINS)
    p.add_argument('--stroke-amps', default=None,
                   help="(strokes) comma-separated amplitudes in rev (default %s)"
                        % ','.join(str(a) for a in sid.STROKE_AMPS_DEFAULT))
    p.add_argument('--stroke-vels', default=None,
                   help="(strokes) comma-separated peak velocities in rev/s (default %s, "
                        "all under the 4.0 config cap)"
                        % ','.join(str(v) for v in sid.STROKE_VELS_DEFAULT))
    p.add_argument('--teleop-range', type=float, default=sid.TELEOP_RANGE_DEFAULT,
                   help="(teleop) half-range in rev; z=±1 → centre ± this (default %.1f)"
                        % sid.TELEOP_RANGE_DEFAULT)
    p.add_argument('--teleop-device', default=None,
                   help="(teleop) case-insensitive SUBSTRING of the spacemouse model "
                        "name to open (e.g. 'Compact', 'Wireless'). REQUIRED when "
                        "more than one spacemouse is connected — the bare open() "
                        "grabs whichever enumerates first, silently reading the "
                        "wrong stick")
    p.add_argument('--teleop-max-vel', type=float, default=sid.TELEOP_MAX_VEL_DEFAULT,
                   help="(teleop) slew velocity cap in rev/s (default %.1f; up to %.1f "
                        "with a warning)"
                        % (sid.TELEOP_MAX_VEL_DEFAULT, sid.TELEOP_MAX_VEL_CEILING))
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

    output_dir = args.output_dir or _default_output_dir()

    # Parse the multi-value BRIDGE knobs (harmless for the DIRECT dry-run, unused there).
    chirp_amps = (_parse_float_list(args.chirp_amps) if args.chirp_amps
                  else list(DEFAULT_CHIRP_AMPS_REV))
    try:
        track_gains = _parse_track_gains(args.track_gains)
        custom_rungs = _parse_track_gains(args.rungs) if args.rungs else None
        gains_override = None
        if args.gains:
            triples = _parse_track_gains(args.gains)
            if len(triples) != 1:
                raise ValueError("--gains takes exactly ONE pg:vg:vint triple")
            gains_override = triples[0]
    except ValueError as e:
        print(f"REJECT: {e}", file=sys.stderr)
        return 1
    if args.quiescent_secs <= 0.0:
        print(f"REJECT: --quiescent-secs must be positive", file=sys.stderr)
        return 1
    if args.track_peak_vel > sid.TRACK_PEAK_VEL_MAX_RPS:
        print(f"REJECT: --track-peak-vel {args.track_peak_vel} > "
              f"{sid.TRACK_PEAK_VEL_MAX_RPS} rev/s (hard cap — the sine/triangle would "
              f"exceed the safe lead-clamp regime)", file=sys.stderr)
        return 1
    stroke_amps = _parse_float_list(args.stroke_amps) if args.stroke_amps else None
    stroke_vels = _parse_float_list(args.stroke_vels) if args.stroke_vels else None
    # Teleop slew cap: default 2.0, up to 4.0 with a warning, never above (the drive
    # leg_vel_limit is 6.0 rev/s since 2026-07-16; 4.0 is kept as a deliberately
    # conservative stick-slew ceiling).
    if args.teleop_max_vel > sid.TELEOP_MAX_VEL_CEILING:
        print(f"REJECT: --teleop-max-vel {args.teleop_max_vel} > "
              f"{sid.TELEOP_MAX_VEL_CEILING} rev/s (the conservative teleop stick-slew "
              f"ceiling, deliberately below the 6.0 rev/s drive limit)", file=sys.stderr)
        return 1
    if args.teleop_max_vel > sid.TELEOP_MAX_VEL_DEFAULT + 1e-9:
        print(f"  WARNING: --teleop-max-vel {args.teleop_max_vel} rev/s is above the "
              f"{sid.TELEOP_MAX_VEL_DEFAULT} rev/s default (aggressive stick response)")

    # ── Path dispatch ───────────────────────────────────────────────────────
    if args.path == 'direct':
        # Direct CAN was wound down on this Jetson (operator-confirmed 2026-07-11):
        # no USB-CAN adapter, the Jetson no longer sees CAN. Live runs are refused;
        # --dry-run is allowed as a historical/plan-only view of the retired path.
        if not args.dry_run:
            print("REJECT: --path direct is UNAVAILABLE for live runs on this Jetson "
                  "— direct CAN was wound down 2026-07-11 (no USB-CAN adapter, the "
                  "Jetson no longer sees CAN). Use --path bridge, or add --dry-run "
                  "for the historical direct plan.", file=sys.stderr)
            return 1
        if _DIRECT_IMPORT_ERR is not None:
            print(f"REJECT: Path DIRECT unavailable to plan — single_leg_test import "
                  f"failed ({_DIRECT_IMPORT_ERR}). This is expected post-cutover; use "
                  f"--path bridge.", file=sys.stderr)
            return 1
        print("  NOTE Path DIRECT is retired on this Jetson (direct CAN wound down "
              "2026-07-11). This --dry-run is a historical/plan-only view.")
        modes = MODE_GROUPS_DIRECT.get(args.mode, [args.mode])
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

    # Path BRIDGE (default, the only runnable path on this Jetson).
    if args.mode == 'vel_steps':
        print("REJECT: --mode vel_steps is not available on Path BRIDGE — the bridge "
              "has no raw input_vel step; velocity-loop damping is INFERRED from "
              "position-step overshoot (run --mode pos_steps).", file=sys.stderr)
        return 1
    modes = MODE_GROUPS_BRIDGE.get(args.mode, [args.mode])
    runner = BridgeSysID(
        axis_id=args.axis, stroke_cap_rev=args.stroke_cap, center_rev=args.center,
        current_limit_a=args.current_limit, vel_cap_rps=args.vel_cap,
        output_dir=output_dir, dry_run=args.dry_run,
        chirp_f0=args.chirp_f0, chirp_f1=args.chirp_f1,
        chirp_dur=args.chirp_duration, chirp_amp=args.chirp_amp,
        pos_steps=DEFAULT_POS_STEPS_REV, ladder_step=DEFAULT_LADDER_STEP_REV,
        zeta_target=args.zeta_target, bw_clear_hz=args.bw_clear_hz, n_vel=args.n_vel,
        ripple_threshold=args.ripple_threshold, osc_threshold=args.osc_threshold,
        iq_ripple_threshold=args.iq_ripple_threshold, do_home=args.home,
        max_recoveries=args.max_recoveries,
        chirp_amps=chirp_amps, knot_hz=args.knot_hz, vint_ratio=args.vint_ratio,
        from_pg=args.from_pg, track_wave=args.track_wave,
        track_peak_vel=args.track_peak_vel, track_secs=args.track_secs,
        track_gains=track_gains, fast_iq_available=args.fast_iq,
        custom_rungs=custom_rungs, quiescent_secs=args.quiescent_secs,
        gains_override=gains_override,
        stroke_amps=stroke_amps, stroke_vels=stroke_vels,
        teleop_range=args.teleop_range, teleop_max_vel=args.teleop_max_vel,
        teleop_device=args.teleop_device)
    return runner.run(modes)


if __name__ == '__main__':
    sys.exit(main())
