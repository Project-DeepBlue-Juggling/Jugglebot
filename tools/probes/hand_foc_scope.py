#!/usr/bin/env python3
"""Hand FOC scope: 8 kHz current-loop capture from the ODrive's RAM oscilloscope (BUG-57).

The 100 Hz ``hand_capture`` cannot show how the Hand's braking torque is lost, because
iq freezes at its last value before the disarm. This tool records the drive's own
current-loop signals at full rate over USB, then downloads and analyses them.

READ-ONLY FOR THE MOTOR. It never changes the drive's state, limits or configuration:
its only writes are ``oscilloscope.config`` and ``oscilloscope.trigger``, and
``tests/motion/test_hand_foc_scope.py`` pins that. Motion comes from another tool, such
as ``tests/hardware/hand_stream_bench.py --stage stroke_cut``. Connect USB through an
isolator: the drive's ground is shared with hard regenerative braking.

UNITS AND FRAMES (FW 0.6.11, measured on the bench Hand's captures, 2026-11-11). ``foc.phase``
is the electrical angle in TURNS, in [0, 1), and ``foc.phase_vel`` is electrical turns/s: the
phase advances by phase_vel/fs a sample and by exactly pole_pairs turns per motor turn. So the
electrical angular velocity is w = 2*pi*phase_vel. (The first version of this tool took both
as radians. That made the angle about -7 deg flat, the fitted flux linkage 2*pi too high, and
every wrap of the turns signal a "phase jump".) ``pos_vel_mapper.pos_rel`` need not count the
same way as the electrical frame: on the bench Hand it ran opposite (phase_vel / d(pos)/dt =
-6.85 against 7 pole pairs). The analysis therefore takes the frame sign from the data,
works in the drive's electrical frame, and never hard-codes a rig.

THE ANGLE ERROR. The onboard encoder drives both commutation and position, so a slipped
magnet moves ``foc.phase`` and position together and neither shows it. The back-EMF does.
With the drive's feedforward off, ``foc.Vd_setpoint`` and ``foc.Vq_setpoint`` are unused
inputs, so the applied voltage comes from the modulation:

    Vd = mod_d * (2/3) * vbus        Vq = mod_q * (2/3) * vbus
    e_d = Vd - R*Id + w*L*Iq         e_q = Vq - R*Iq - w*L*Id        (w = 2*pi*phase_vel)
    correct commutation: e_d ~ 0, e_q ~ w*lambda
    angle error:         delta = atan2(-e_d * sign(w), e_q * sign(w))

The angle is only reported where the back-EMF expected from the speed (lambda from Kt times
|w|) is well above every term the model subtracts or leaves out:
E >= EMF_RATIO * (R*|I| + |w|*L*|I| + V_step). With the ratio at 3, a 50 % error in any of them
moves the angle by at most about 10 deg. Below that the configured R and L, or the step, decide
the angle, not the magnet. (The measured |e| is not used for this gate: at rest an uncorrected
step alone reads as 0.6 V of "back-EMF".) On the bench the voltage left over after R and L
needed more resistance than configured plus a dead-time-like step of 0.2-0.6 V, so brake-only
angles scattered from -60 to +30 deg and no bench sample passes the gate. The tool fits the
effective R and the step (V_step = the fitted one, else DEFAULT_V_STEP) and REPORTS them; it
does not correct for them. With a near two-level current, as in a C1 stroke, the two trade
against each other and the tool says they are not separately identified.
Read delta against a clean brake, not as an absolute. The 2/3 scaling is checked by the
``power`` preset, which records the drive's own ``electrical_power`` beside the value computed
here.

THE SPINOUT CHECK (model). The drive trips SPINOUT_DETECTED when its first-order filtered
electrical power is above ``spinout_electrical_power_threshold`` (+10 W) while its filtered
mechanical power is below ``spinout_mechanical_power_threshold`` (-10 W), each filtered at its
``spinout_*_power_bandwidth`` (20 rad/s). The tool replays that on the capture. It uses the
drive's own ``motor.electrical_power`` channel when the preset has it. Mechanical power is
torque_constant * Iq * 2*pi*phase_vel / pole_pairs, in the drive's frame. That formula, and
the filter form, are ASSUMPTIONS about the firmware: the ``spinout`` preset records the
drive's own filtered values so a capture can check them.

TRIGGERS. ``brake`` fires once the speed has passed ``--arm-vel`` upward and fallen ``--drop``
below its peak. Both modes fire on a FAULT: an edge to non-zero in ``disarm_reason`` or
``active_errors`` against the previous poll (so a value latched before arming never fires). A
clean exit from CLOSED_LOOP (no new reason) re-arms without saving and without using up
``--count``, unless ``--save-clean`` is given. In ``fault`` mode no timeout saves anything,
and the default is to wait until Ctrl-C. Every saved file name ends in ``_brake``, ``_fault``,
``_clean`` or ``_timeout``. A timeout window in brake mode is the scope's last window, not a
capture of the event, and ``analyze`` says so.

Usage (launch may stay down; the motion tool owns the can-bridge link, this one owns USB):
    python tools/probes/hand_foc_scope.py probe --label bench
    python tools/probes/hand_foc_scope.py capture --trigger brake --label bench_70
    python tools/probes/hand_foc_scope.py capture --trigger fault --preset power --label robot
    python tools/probes/hand_foc_scope.py analyze temp/hand_foc_scope/<file>.csv

THE SCOPE'S RULES (measured on the bench Hand's drive, FW 0.6.11, 2026-11-11). The ring is
65536 bytes, 4 bytes per channel, at most 9 channels. A trigger ends the recording only when
the bytes left to record after it land on a sample boundary, and the ring only wraps on one
when the sample size divides it. Otherwise the drive records for ever and the odrive
package's download waits for ever: the first probe on a drive hung that way. So every
preset has 8 channels or fewer with a sample size that divides the ring
(``scope_grid_point`` refuses one that does not), every trigger point is rounded to a whole
sample, and the download wait is bounded. ``oscilloscope.trigger_high_res()`` dropped the
drive off USB and restarted it, losing its position reference; never call it. The trigger
point is the fraction of the window kept BEFORE the trigger (probe: 0.797 measured for 0.80).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT_DIR = os.path.join(REPO, "temp", "hand_foc_scope")
HAND_CONFIG = os.path.join(REPO, "config", "ODrive config Files", "odrive_pro_hand_config.json")

FOC = "axis0.motor.foc."
MOTOR = "axis0.motor."
CTRL = "axis0.controller."
#: CSV column name -> endpoint. Each preset's sample (4 bytes a channel) must divide the
#: 65536-byte ring, so 8 channels at most, not the scope's 9 (see THE SCOPE'S RULES).
PRESETS = {
    "angle": {
        "mod_d": FOC + "mod_d", "mod_q": FOC + "mod_q", "vbus": "vbus_voltage",
        "id": FOC + "Id_measured", "iq": FOC + "Iq_measured",
        "phase": FOC + "phase", "phase_vel": FOC + "phase_vel",
        "pos": "axis0.pos_vel_mapper.pos_rel",
    },
    "power": {
        "mod_d": FOC + "mod_d", "mod_q": FOC + "mod_q", "vbus": "vbus_voltage",
        "id": FOC + "Id_measured", "iq": FOC + "Iq_measured", "phase_vel": FOC + "phase_vel",
        "pos": "axis0.pos_vel_mapper.pos_rel",
        "p_elec": MOTOR + "electrical_power",
    },
    # The drive's own SPINOUT inputs, raw and filtered, to check the model against. No angle
    # (no modulation) and no position: speed and accel come from phase_vel. Never flown yet.
    "spinout": {
        "vbus": "vbus_voltage", "id": FOC + "Id_measured", "iq": FOC + "Iq_measured",
        "phase_vel": FOC + "phase_vel",
        "p_elec": MOTOR + "electrical_power", "p_mech": MOTOR + "mechanical_power",
        "spin_e": CTRL + "spinout_electrical_power", "spin_m": CTRL + "spinout_mechanical_power",
    },
    "timing": {
        "n_sampling": "n_evt_sampling", "n_control": "n_evt_control_loop",
        "phase": FOC + "phase", "phase_vel": FOC + "phase_vel",
    },
}

SPIN_E_THRESHOLD = CTRL + "config.spinout_electrical_power_threshold"
SPIN_M_THRESHOLD = CTRL + "config.spinout_mechanical_power_threshold"
SPIN_E_BANDWIDTH = CTRL + "config.spinout_electrical_power_bandwidth"
SPIN_M_BANDWIDTH = CTRL + "config.spinout_mechanical_power_bandwidth"
SPINOUT_READS = (SPIN_E_THRESHOLD, SPIN_M_THRESHOLD, SPIN_E_BANDWIDTH, SPIN_M_BANDWIDTH)

#: Values read once for the record. Reads only.
IDENTITY_READS = (
    "serial_number", "hw_version_major", "hw_version_minor", "fw_version_major",
    "fw_version_minor", "fw_version_revision", "vbus_voltage", "oscilloscope.size",
    "axis0.current_state", "axis0.active_errors", "axis0.disarm_reason",
    "axis0.config.commutation_encoder", "axis0.config.load_encoder",
    "axis0.config.motor.pole_pairs", "axis0.config.motor.phase_resistance",
    "axis0.config.motor.phase_inductance", "axis0.config.motor.torque_constant",
    "axis0.config.motor.current_soft_max", "axis0.config.motor.current_hard_max",
    "axis0.config.encoder_bandwidth", "axis0.config.motor.current_control_bandwidth",
    "axis0.motor.foc.p_gain", "axis0.commutation_mapper.config.offset",
    "axis0.commutation_mapper.config.scale", "axis0.pos_vel_mapper.config.scale",
    "axis0.commutation_mapper.working_offset", "onboard_encoder0.status",
    "onboard_encoder0.raw",
) + SPINOUT_READS
FIELD_STRENGTH = "onboard_encoder0.get_field_strength"
#: What a record field is NOT, so nobody reads it as a health constant (FEAT-45, 2026-11-11).
IDENTITY_NOTES = {
    "axis0.commutation_mapper.working_offset":
        "tracks the electrical position (read -3.8 to -29.2 across 2026-11-11, fraction = "
        "the at-rest phase); recorded, not a health constant",
    FIELD_STRENGTH:
        "bit-identical (0.11599999666) in 7 reads on 2026-11-11 with the motor moved between "
        "them; meaning and units unknown in FW 0.6.11; not an encoder-health reading",
}
VEL = "axis0.pos_vel_mapper.vel"
STATE = "axis0.current_state"
DISARM = "axis0.disarm_reason"
ERRORS = "axis0.active_errors"
CLOSED_LOOP = 8

MIN_WINDOW_MS = 60.0       # a C1 brake is about 40 ms; below this a capture cannot hold one
MIN_SAMPLE_HZ = 4000.0     # at 137 rev/s the electrical period is 1 ms
TRIGGER_POINT_TOLERANCE = 0.15
DEFAULT_TRIGGER_POINT = {"brake": 0.5, "fault": 0.9}   # fault: ~230 ms kept before the disarm
DEFAULT_TIMEOUT_S = {"brake": 60.0, "fault": None}      # None: wait until Ctrl-C

SPEED_BINS = (0.0, 20.0, 50.0, 80.0, 110.0, 140.0, 1e9)
MIN_EMF_V = 0.5            # expected back-EMF below this is too small to give an angle
EMF_RATIO = 3.0            # back-EMF over (R|I| + |w|L|I| + step): a 50 % error in any -> <= ~10 deg
PHASE_JUMP_TURNS = 0.08    # one sample's phase step against phase_vel/fs (~29 electrical deg)
FRAME_MIN_SAMPLES = 50     # moving samples needed to tell the frame of pos_rel
FRAME_SCALE_TOLERANCE = 0.10  # |phase_vel / d(pos)/dt| this far from pole_pairs: units suspect
TORQUE_MIN_A = 10.0        # the torque check needs current well above friction and gravity
TORQUE_LOSS_FRACTION = 0.25  # the 2026-11-03 events kept 25-45 % of nominal at best
TORQUE_LOSS_MIN_MS = 2.0
TORQUE_MIN_VEL = 10.0      # at rest, holding current moves nothing and proves nothing
VEL_SMOOTH_MS = 2.0
VFIT_MIN_SAMPLES = 20      # per sign of iq, for the effective-R and step fit
VFIT_DEADBAND_A = 1.0      # |iq| below this has no defined current direction
DEFAULT_V_STEP = 0.3       # step allowance when it cannot be fitted (bench 0.22-0.33 V, FEAT-45)
VFIT_CORR_LIMIT = 0.9      # |corr(R, step)| above this: the two are not separately identified


# ---- analysis (pure) --------------------------------------------------------------------

def wrap_pi(x):
    return (np.asarray(x) + np.pi) % (2.0 * np.pi) - np.pi


def wrap_half(x):
    """Wrap turns into [-0.5, 0.5)."""
    return (np.asarray(x) + 0.5) % 1.0 - 0.5


def _runs(mask):
    """(start, stop) index pairs of the True runs in a boolean array."""
    edges = np.flatnonzero(np.diff(np.r_[False, mask, False].astype(int)))
    return list(zip(edges[::2], edges[1::2]))


def _smooth(x, n):
    """Moving average, padded with the end values so the window's edges read no false drop."""
    return np.convolve(np.pad(x, (n // 2, n - 1 - n // 2), mode="edge"), np.ones(n) / n, mode="valid")


def lowpass(x, bandwidth, fs):
    """First-order low-pass, ``y += min(1, bw/fs) * (x - y)``, started at the first sample.

    The drive's filter has history from before the window; starting at x[0] is a guess whose
    error decays with 1/bandwidth (50 ms at 20 rad/s)."""
    x = np.asarray(x, dtype=float)
    a = min(1.0, float(bandwidth) / float(fs))
    y = np.empty_like(x)
    s = x[0] if len(x) else 0.0
    for i, v in enumerate(x):
        s += a * (v - s)
        y[i] = s
    return y


def frame_from_data(phase_vel, vel_pos, pole_pairs):
    """How pos_rel counts against the electrical frame, from the samples that move.

    Returns (sign, ratio, why): sign +1 or -1 and the median phase_vel / d(pos)/dt, or
    (None, None, why) when too few samples move."""
    m = np.abs(vel_pos) >= TORQUE_MIN_VEL
    if m.sum() < FRAME_MIN_SAMPLES:
        return None, None, "only %d samples above %.0f rev/s" % (int(m.sum()), TORQUE_MIN_VEL)
    ratio = float(np.median(phase_vel[m] / vel_pos[m]))
    return (1.0 if ratio > 0 else -1.0), ratio, None


def spinout_params(ident, config_path=HAND_CONFIG):
    """The four SPINOUT values and where they came from: this drive's identity read, else the
    committed hand config (labelled as assumed), else None."""
    vals = {}
    for k in SPINOUT_READS:
        v = ident.get(k)
        try:
            vals[k] = float(v)
        except (TypeError, ValueError):
            vals = None
            break
    if vals is not None:
        return vals, "read from this drive"
    try:
        with open(config_path) as f:
            cfg = json.load(f)
        vals = {}
        for k in SPINOUT_READS:
            obj = cfg
            for part in k.split("."):
                obj = obj[part]
            vals[k] = float(obj)
        return vals, "NOT read from this drive (capture predates the read); assumed from %s" % (
            os.path.relpath(config_path, REPO),)
    except (OSError, KeyError, TypeError, ValueError):
        return None, "not recorded and no config to assume from"


def _angle(c, params, w, speed, s, out):
    R, L = params["R"], params["L"]
    vd = c["mod_d"] * (2.0 / 3.0) * c["vbus"]
    vq = c["mod_q"] * (2.0 / 3.0) * c["vbus"]
    i_d, i_q = c["id"], c["iq"]
    e_d = vd - R * i_d + w * L * i_q
    e_q = vq - R * i_q - w * L * i_d
    sgn = np.where(w >= 0.0, 1.0, -1.0)
    delta = np.degrees(np.arctan2(-e_d * sgn, e_q * sgn))
    out.update({"vd": vd, "vq": vq, "e_d": e_d, "e_q": e_q, "delta_deg": delta})

    # Effective R and a dead-time-like step, fitted on the q axis where there is back-EMF.
    # Diagnostic only: Vq - R*Iq - w*L*Id = lambda*w + dR*Iq + V_step*sign(Iq).
    moving = speed >= TORQUE_MIN_VEL
    pos_i, neg_i = moving & (i_q >= VFIT_DEADBAND_A), moving & (i_q <= -VFIT_DEADBAND_A)
    if pos_i.sum() < VFIT_MIN_SAMPLES or neg_i.sum() < VFIT_MIN_SAMPLES:
        s["vfit_not_evaluated"] = "needs %d samples above %.0f rev/s with each sign of iq (have %d and %d)" % (
            VFIT_MIN_SAMPLES, TORQUE_MIN_VEL, int(pos_i.sum()), int(neg_i.sum()))
    else:
        sg = np.where(np.abs(i_q) >= VFIT_DEADBAND_A, np.sign(i_q), 0.0)
        X = np.c_[w[moving], i_q[moving], sg[moving]]
        y = e_q[moving]
        coef = np.linalg.lstsq(X, y, rcond=None)[0]
        resid = y - X @ coef
        cov = np.linalg.pinv(X.T @ X)
        corr = (float(cov[1, 2] / math.sqrt(cov[1, 1] * cov[2, 2]))
                if cov[1, 1] > 0 and cov[2, 2] > 0 else float("nan"))
        s["vfit"] = {"lambda": float(coef[0]), "R_eff": float(R + coef[1]), "R_cfg": float(R),
                     "v_step": float(coef[2]), "rms_v": float(resid.std()), "n": int(moving.sum()),
                     "corr_R_step": corr}

    # Which samples may give an angle: the back-EMF expected from the speed (lambda from Kt)
    # must be well above every term the model subtracts or leaves out. The measured |e| is
    # not used for this, because an uncorrected step makes |e| look like back-EMF at rest.
    step = abs(s["vfit"]["v_step"]) if "vfit" in s else DEFAULT_V_STEP
    s["angle_step_allowance"] = (step, "fitted" if "vfit" in s else "default, bench FEAT-45")
    i_abs = np.hypot(i_d, i_q)
    emf = params["kt"] / (1.5 * params["pole_pairs"]) * np.abs(w)
    has_emf = emf >= MIN_EMF_V
    valid = has_emf & (emf >= EMF_RATIO * (R * i_abs + np.abs(w) * L * i_abs + step))
    out["valid"] = valid
    s["emf_samples"] = int(has_emf.sum())
    s["angle_qualified"] = int(valid.sum())
    bins = []
    for lo, hi in zip(SPEED_BINS[:-1], SPEED_BINS[1:]):
        inb = has_emf & (speed >= lo) & (speed < hi)
        m = valid & inb
        if inb.any():
            b = {"lo": lo, "hi": hi, "n": int(m.sum()), "n_emf": int(inb.sum())}
            if m.any():
                b.update({"delta_median": float(np.median(delta[m])),
                          "delta_p5": float(np.percentile(delta[m], 5)),
                          "delta_p95": float(np.percentile(delta[m], 95)),
                          "iq_abs_median": float(np.median(np.abs(i_q[m])))})
            bins.append(b)
    s["delta_by_speed"] = bins
    if valid.sum() >= 2:
        s["lambda_fit"] = float(np.dot(w[valid], e_q[valid]) / np.dot(w[valid], w[valid]))


def _torque(i_q, vel, acc, fs, n_smooth, s):
    """Accel per amp and the torque-loss runs, in whatever frame vel and acc are given.

    The detector normalises each sample's accel per amp by the capture's own median, so it
    runs whichever sign that median has. It is "not evaluated" only when it cannot run."""
    moving = np.abs(vel) >= TORQUE_MIN_VEL
    if moving.sum() >= 3 and np.ptp(i_q[moving]) > 0.0:
        A = np.c_[i_q[moving], np.ones(int(moving.sum()))]
        (K, b), *_ = np.linalg.lstsq(A, acc[moving], rcond=None)
        s["torque_fit"] = {"K": float(K), "b": float(b), "n": int(moving.sum()),
                           "rms": float((acc[moving] - A @ [K, b]).std())}
    loaded = (np.abs(i_q) >= TORQUE_MIN_A) & moving
    s["torque_loss"] = None
    if loaded.sum() < n_smooth:
        s["torque_loss_not_evaluated"] = "%d samples with |iq| >= %.0f A and |vel| >= %.0f rev/s, need %d" % (
            int(loaded.sum()), TORQUE_MIN_A, TORQUE_MIN_VEL, n_smooth)
        return
    ratio = np.where(loaded, acc / np.where(i_q == 0.0, np.inf, i_q), np.nan)
    k = float(np.nanmedian(ratio))
    s["accel_per_amp"] = k
    if not np.isfinite(k) or k == 0.0:
        s["torque_loss_not_evaluated"] = "median accel per amp is %r" % (k,)
        return
    weak = loaded & (ratio / k < TORQUE_LOSS_FRACTION)
    min_n = TORQUE_LOSS_MIN_MS * 1e-3 * fs
    s["torque_loss"] = [
        {"start_ms": 1e3 * a / fs, "ms": 1e3 * (b - a) / fs,
         "iq": float(np.median(i_q[a:b])), "vel": float(np.median(vel[a:b]))}
        for a, b in _runs(weak) if b - a >= min_n]


def _spinout(c, params, fs, spin, p_e_calc, s, out):
    """Replay the drive's SPINOUT check on the capture (see THE SPINOUT CHECK)."""
    if "p_elec" in c:
        p_e, s["spinout_p_elec_source"] = c["p_elec"], "the drive's motor.electrical_power channel"
    elif p_e_calc is not None:
        p_e, s["spinout_p_elec_source"] = p_e_calc, "computed 1.5*(Vd*Id + Vq*Iq)"
    else:
        s["spinout_not_evaluated"] = "no electrical power channel and no modulation"
        return
    if "p_mech" in c:
        p_m, s["spinout_p_mech_source"] = c["p_mech"], "the drive's motor.mechanical_power channel"
    else:
        p_m = params["kt"] * c["iq"] * 2.0 * np.pi * c["phase_vel"] / params["pole_pairs"]
        s["spinout_p_mech_source"] = ("torque_constant*Iq*2*pi*phase_vel/pole_pairs in the drive's frame "
                                      "(ASSUMED: the firmware's exact formula is not verified)")
    vals, src = spin
    s["spinout_param_source"] = src
    if vals is None:
        s["spinout_not_evaluated"] = "SPINOUT thresholds and bandwidths: %s" % src
        return
    the, thm = vals[SPIN_E_THRESHOLD], vals[SPIN_M_THRESHOLD]
    pef = lowpass(p_e, vals[SPIN_E_BANDWIDTH], fs)
    pmf = lowpass(p_m, vals[SPIN_M_BANDWIDTH], fs)
    out["p_elec_filt"], out["p_mech_filt"] = pef, pmf
    # Margins are W past the threshold: > 0 means that side met its trip condition.
    me, mm = pef - the, thm - pmf
    joint = np.minimum(me, mm)
    i = int(np.argmax(joint))
    s["spinout"] = {
        "e_threshold": the, "m_threshold": thm,
        "e_bandwidth": vals[SPIN_E_BANDWIDTH], "m_bandwidth": vals[SPIN_M_BANDWIDTH],
        "p_elec_filt_max": float(pef.max()), "p_mech_filt_min": float(pmf.min()),
        "e_margin": float(me.max()), "m_margin": float(mm.max()),
        "joint_margin": float(joint[i]), "joint_ms": 1e3 * i / fs,
        "joint_p_elec_filt": float(pef[i]), "joint_p_mech_filt": float(pmf[i]),
        "trip_ms": float(1e3 * np.sum(joint > 0.0) / fs),
    }
    for col, model in (("spin_e", pef), ("spin_m", pmf)):
        if col in c:
            s["spinout"][col + "_model_err_max"] = float(np.max(np.abs(c[col] - model)))


def analyse(c, params, fs, spin=None):
    """Derive the angle, torque, encoder and SPINOUT checks from one capture. ``c`` maps the
    column names of a preset to arrays; ``params`` holds R, L, pole_pairs and kt; ``spin`` is
    ``spinout_params(identity)`` (None: the committed hand config, labelled as assumed)."""
    pp = params["pole_pairs"]
    w = 2.0 * np.pi * c["phase_vel"]                 # electrical rad/s, drive frame
    speed = np.abs(c["phase_vel"]) / pp              # rev/s, frame-free
    n_smooth = max(1, int(round(VEL_SMOOTH_MS * 1e-3 * fs)))
    out, s = {}, {"samples": int(len(w)), "window_ms": 1e3 * len(w) / fs,
                  "iq_min": float(c["iq"].min()), "iq_max": float(c["iq"].max()),
                  "vbus_min": float(c["vbus"].min()), "vbus_max": float(c["vbus"].max())}

    # Mechanical velocity and accel in the DRIVE's frame (sign of phase_vel).
    if "pos" in c:
        vel_pos = _smooth(np.gradient(c["pos"]) * fs, n_smooth)
        sign, ratio, why = frame_from_data(c["phase_vel"], vel_pos, pp)
        s["frame_ratio"], s["frame_sign"], s["frame_why"] = ratio, sign, why
        vel = vel_pos * (sign if sign is not None else 1.0)
        s["frame_basis"] = "pos_rel" if sign is not None else "pos_rel, frame UNKNOWN (taken as +1)"
    else:
        vel = _smooth(c["phase_vel"] / pp, n_smooth)
        s["frame_basis"] = "phase_vel / pole_pairs (no pos channel)"
    acc = _smooth(np.gradient(vel) * fs, n_smooth)
    out.update({"vel": vel, "acc": acc, "w": w})
    s["vel_max"], s["vel_min"] = float(vel.max()), float(vel.min())

    p_e_calc = None
    if "mod_d" in c and "mod_q" in c:
        s["mod_max"] = float(np.hypot(c["mod_d"], c["mod_q"]).max())
        _angle(c, params, w, speed, s, out)
        p_e_calc = 1.5 * (out["vd"] * c["id"] + out["vq"] * c["iq"])
        out["p_elec_calc"] = p_e_calc
    s["lambda_from_kt"] = params["kt"] / (1.5 * pp)

    if "phase" in c:
        pv = c["phase_vel"]
        step = wrap_half(np.diff(c["phase"]) - 0.5 * (pv[:-1] + pv[1:]) / fs)
        jumps = np.flatnonzero(np.abs(step) > PHASE_JUMP_TURNS)
        s["phase_jumps"] = int(len(jumps))
        s["phase_jump_first_ms"] = float(1e3 * jumps[0] / fs) if len(jumps) else None
        s["phase_residual_max"] = float(np.abs(step).max()) if len(step) else 0.0

    _torque(c["iq"], vel, acc, fs, n_smooth, s)
    _spinout(c, params, fs, spin if spin is not None else spinout_params({}), p_e_calc, s, out)
    if "p_elec" in c and p_e_calc is not None and np.max(np.abs(c["p_elec"])) > 20.0:
        big = np.abs(c["p_elec"]) > 20.0
        s["p_elec_calc_over_drive"] = float(np.median(p_e_calc[big] / c["p_elec"][big]))
    return out, s


def summary_lines(s, pole_pairs=None):
    """The human summary of one capture."""
    lines = ["window %.1f ms, %d samples; speed %+.0f to %+.0f rev/s (drive frame); iq %+.1f to %+.1f A; "
             "vbus %.1f to %.1f V%s" % (
                 s["window_ms"], s["samples"], s["vel_min"], s["vel_max"], s["iq_min"], s["iq_max"],
                 s["vbus_min"], s["vbus_max"],
                 "; max modulation %.3f" % s["mod_max"] if "mod_max" in s else "")]
    if s.get("frame_ratio") is not None:
        r = s["frame_ratio"]
        lines.append("frame: phase_vel / d(pos_rel)/dt = %+.3f%s: pos_rel runs %s the electrical frame%s" % (
            r, "" if pole_pairs is None else " (pole pairs %d)" % pole_pairs,
            "WITH" if r > 0 else "OPPOSITE to",
            "" if pole_pairs is None or abs(abs(r) / pole_pairs - 1.0) <= FRAME_SCALE_TOLERANCE
            else "; |ratio| is not the pole-pair count: UNITS OR SCALE SUSPECT"))
    elif s.get("frame_why"):
        lines.append("frame: not evaluated: %s; speed and accel signs taken from pos_rel as is" % s["frame_why"])
    else:
        lines.append("frame: %s" % s["frame_basis"])
    if "delta_by_speed" in s:
        lines.append("angle error by speed, only where the back-EMF (lambda from Kt x |w|) >= %.0f x "
                     "(R|I| + |w|L|I| + step %.2f V, %s) (read against a clean brake, not as an absolute):" % (
                         EMF_RATIO, s["angle_step_allowance"][0], s["angle_step_allowance"][1]))
        if s["angle_qualified"] == 0:
            lines.append("  not reported: 0 of %d samples with back-EMF >= %.1f V qualified" % (
                s["emf_samples"], MIN_EMF_V))
        for b in s["delta_by_speed"]:
            hi = ("%.0f" % b["hi"]) if b["hi"] < 1e8 else "up"
            if b["n"]:
                lines.append("  %3.0f-%-5s rev/s  n %5d of %5d  delta %+7.1f deg (p5 %+7.1f, p95 %+7.1f), |iq| %5.1f A" % (
                    b["lo"], hi, b["n"], b["n_emf"], b["delta_median"], b["delta_p5"], b["delta_p95"],
                    b["iq_abs_median"]))
            else:
                lines.append("  %3.0f-%-5s rev/s  n     0 of %5d  (none qualified)" % (b["lo"], hi, b["n_emf"]))
    if "lambda_fit" in s:
        lines.append("flux linkage over qualified samples: fitted %.3e, from Kt %.3e V s/rad" % (
            s["lambda_fit"], s["lambda_from_kt"]))
    if "vfit" in s:
        v = s["vfit"]
        lines.append("voltage model (diagnostic, NOT applied): effective R %.4f ohm (configured %.4f), "
                     "step %+.3f V x sign(iq), lambda %.3e V s/rad; rms %.3f V over %d samples" % (
                         v["R_eff"], v["R_cfg"], v["v_step"], v["lambda"], v["rms_v"], v["n"]))
        if not abs(v["corr_R_step"]) <= VFIT_CORR_LIMIT:
            lines.append("  R and the step trade against each other here (correlation %+.2f): "
                         "not separately identified" % v["corr_R_step"])
    elif "vfit_not_evaluated" in s:
        lines.append("voltage model: not evaluated: %s" % s["vfit_not_evaluated"])
    if "phase_jumps" in s:
        lines.append("encoder phase jumps over %.2f turn against phase_vel: %d%s; largest residual %.4f turn" % (
            PHASE_JUMP_TURNS, s["phase_jumps"],
            "" if s["phase_jump_first_ms"] is None else " (first at %.1f ms)" % s["phase_jump_first_ms"],
            s["phase_residual_max"]))
    if "torque_fit" in s:
        t = s["torque_fit"]
        lines.append("torque: accel = %.1f x iq %+.0f rev/s^2 (drive frame, |vel| >= %.0f rev/s, rms %.0f)" % (
            t["K"], t["b"], TORQUE_MIN_VEL, t["rms"]))
    if "accel_per_amp" in s:
        lines.append("median accel per amp above %.0f A and %.0f rev/s: %.1f rev/s^2/A%s" % (
            TORQUE_MIN_A, TORQUE_MIN_VEL, s["accel_per_amp"],
            "" if s["accel_per_amp"] > 0 else " (NEGATIVE: torque opposes current in this frame; check the frame line)"))
    if s["torque_loss"] is None:
        lines.append("torque loss: not evaluated: %s" % s.get("torque_loss_not_evaluated", "unknown"))
    elif s["torque_loss"]:
        for t in s["torque_loss"]:
            lines.append("TORQUE LOSS at %.1f ms for %.1f ms: iq %+.1f A, %+.0f rev/s" % (
                t["start_ms"], t["ms"], t["iq"], t["vel"]))
    else:
        lines.append("torque loss (under %.0f %% of the capture's median, %.0f ms or longer): none found" % (
            100 * TORQUE_LOSS_FRACTION, TORQUE_LOSS_MIN_MS))
    if "spinout" in s:
        p = s["spinout"]
        lines.append("SPINOUT model (thresholds %s): P_elec from %s; P_mech = %s" % (
            s["spinout_param_source"], s["spinout_p_elec_source"], s["spinout_p_mech_source"]))
        lines.append("  filtered P_elec max %+.1f W vs %+.0f W (%.0f rad/s): margin %+.1f W; "
                     "filtered P_mech min %+.1f W vs %+.0f W (%.0f rad/s): margin %+.1f W" % (
                         p["p_elec_filt_max"], p["e_threshold"], p["e_bandwidth"], p["e_margin"],
                         p["p_mech_filt_min"], p["m_threshold"], p["m_bandwidth"], p["m_margin"]))
        lines.append("  both at once: best margin %+.1f W at %.1f ms (filtered P_elec %+.1f W, P_mech %+.1f W); %s" % (
            p["joint_margin"], p["joint_ms"], p["joint_p_elec_filt"], p["joint_p_mech_filt"],
            "TRIP predicted for %.1f ms" % p["trip_ms"] if p["trip_ms"] > 0 else "no trip predicted"))
        for col, name in (("spin_e", "electrical"), ("spin_m", "mechanical")):
            if col + "_model_err_max" in p:
                lines.append("  drive's own filtered %s power vs the model: max |difference| %.2f W" % (
                    name, p[col + "_model_err_max"]))
    elif "spinout_not_evaluated" in s:
        lines.append("SPINOUT model: not evaluated: %s" % s["spinout_not_evaluated"])
    if "p_elec_calc_over_drive" in s:
        lines.append("computed electrical power / the drive's own: %.3f (1.0 confirms the 2/3 scaling)" % s["p_elec_calc_over_drive"])
    return lines


def record_lines(meta):
    """What kind of window a saved file is. Returns (lines, is_a_triggered_capture)."""
    if "capture" in meta:
        info = meta["capture"]
        kind = info.get("kind") or ("timeout" if not info.get("fired") else "untagged")
        lines = ["record: capture, kind %s, fired %s, reason %s; %s polls at %.2f ms; trigger point %s" % (
            kind, info.get("fired"), info.get("reason"), info.get("polls"), float(info.get("poll_ms") or 0.0),
            info.get("trigger_point"))]
        if info.get("clean_exits"):
            lines.append("record: %d clean exits from CLOSED_LOOP re-armed before this one" % info["clean_exits"])
        if not info.get("fired"):
            secs = float(info.get("polls") or 0) * float(info.get("poll_ms") or 0.0) / 1e3
            lines.append("TIMED OUT after %.1f s: this is the scope's last window, NOT a capture of the event" % secs)
            return lines, False
        return lines, True
    if "probe" in meta:
        reasons = judge_probe(meta["probe"])
        return ["record: probe self-check window (no trigger); re-judged: %s" % (
            "GO" if not reasons else "NO-GO: " + "; ".join(reasons))], False
    return ["record: no capture record in the JSON: cannot tell whether this window was triggered"], False


def identity_lines(ident):
    """The identity for printing, with the fields that are not what they look like labelled."""
    return ["%-48s %s%s" % (k, ident[k], "   [%s]" % IDENTITY_NOTES[k] if k in IDENTITY_NOTES else "")
            for k in sorted(ident)]


class BrakeTrigger:
    """When to freeze the scope, fed one (vel, state, disarm_reason, active_errors) per poll.

    A FAULT is an edge to non-zero in disarm_reason or active_errors against the previous
    poll, so a value latched before arming never fires; it fires in both modes. A CLEAN exit
    is CLOSED_LOOP to any other state with no such edge: it re-arms (and resets the brake peak)
    without firing, unless ``save_clean``. ``brake`` also fires once the speed has passed
    ``arm_vel`` upward and fallen ``drop`` below its peak. ``kind`` names what fired."""

    def __init__(self, mode, arm_vel=50.0, drop=0.15, save_clean=False):
        if mode not in ("brake", "fault"):
            raise ValueError("unknown trigger mode %r" % (mode,))
        self.mode, self.arm_vel, self.drop, self.save_clean = mode, arm_vel, drop, save_clean
        self.peak, self.was_closed_loop, self.reason, self.kind = 0.0, False, None, None
        self.last, self.at_arm, self.clean_exits, self.notes = None, None, 0, []

    def seed(self, disarm_reason, active_errors=0):
        """Take the arm-time baseline now (before the scope fills), so a fault that latches
        while the ring fills still shows as an edge on the first poll."""
        if self.last is None:
            self.last = self.at_arm = (int(disarm_reason), int(active_errors))

    def update(self, vel, state, disarm_reason, active_errors=0):
        r, e = int(disarm_reason), int(active_errors)
        if self.last is None:
            self.last = self.at_arm = (r, e)
        (r0, e0), self.last = self.last, (r, e)
        if (r and r != r0) or (e and e != e0):
            self.kind = "fault"
            self.reason = "fault (state %s, disarm_reason 0x%x, active_errors 0x%x; at arm 0x%x, 0x%x)" % (
                state, r, e, self.at_arm[0], self.at_arm[1])
            return True
        if state == CLOSED_LOOP:
            self.was_closed_loop = True
        elif self.was_closed_loop:
            self.was_closed_loop, self.peak = False, 0.0
            self.clean_exits += 1
            note = "clean exit from CLOSED_LOOP to state %s (disarm_reason 0x%x, active_errors 0x%x, no new value)" % (
                state, r, e)
            self.notes.append(note)
            if self.save_clean:
                self.kind, self.reason = "clean", note
                return True
        if self.mode == "brake" and vel is not None:
            self.peak = max(self.peak, vel)
            if self.peak >= self.arm_vel and vel <= self.peak * (1.0 - self.drop):
                self.kind = "brake"
                self.reason = "brake (peak %.1f rev/s, now %.1f rev/s)" % (self.peak, vel)
                return True
        return False


def judge_probe(r):
    """GO or NO-GO reasons from the scope self-check."""
    reasons = []
    if r["window_ms"] < MIN_WINDOW_MS:
        reasons.append("window %.1f ms is under %.0f ms" % (r["window_ms"], MIN_WINDOW_MS))
    if r["sample_hz"] < MIN_SAMPLE_HZ:
        reasons.append("sample rate %.0f Hz is under %.0f Hz" % (r["sample_hz"], MIN_SAMPLE_HZ))
    if r["counter_gaps"]:
        reasons.append("%d gaps in the sample counter" % r["counter_gaps"])
    # Either meaning of the fraction is usable (kept before or kept after the trigger); the
    # probe requests an asymmetric point to tell them apart. Anything else is not understood.
    want, got = r["trigger_point_requested"], r["trigger_point_measured"]
    if min(abs(got - want), abs(got - (1.0 - want))) > TRIGGER_POINT_TOLERANCE:
        reasons.append("trigger point measured %.2f, requested %.2f" % (got, want))
    if not r["angle_channels_finite"]:
        reasons.append("the angle preset returned non-finite values")
    if not 10.0 <= r["vbus"] <= 60.0:
        reasons.append("vbus %.1f V is implausible" % r["vbus"])
    return reasons


# ---- the drive (USB) ---------------------------------------------------------------------

SCOPE_BYTES_PER_CHANNEL = 4
SCOPE_MAX_CHANNELS = 9
SCOPE_STOP_TIMEOUT_S = 3.0      # a whole 8-channel window at 8 kHz is 256 ms


def scope_grid_point(point, ring_bytes, n_channels):
    """Round a trigger point to a whole sample, or refuse a channel count the ring cannot hold.

    The drive stops recording only when the bytes left after the trigger are a whole number
    of samples, so 0.8 on a 16-byte sample never stops; 1638/2048 does. Points of the form
    k / n are exact in float32, so the drive receives what was computed here.
    """
    sample = SCOPE_BYTES_PER_CHANNEL * int(n_channels)
    if not 1 <= n_channels <= SCOPE_MAX_CHANNELS or ring_bytes % sample:
        raise ValueError("%d channels (%d-byte samples) do not divide the %d-byte scope ring; "
                         "the recording would never stop" % (n_channels, sample, ring_bytes))
    n = ring_bytes // sample
    return min(1.0, max(0.0, round(float(point) * n) / n))


class Drive:
    """The odrive package behind the few calls this tool makes."""

    def __init__(self, odrv, utils):
        self.odrv, self.utils = odrv, utils
        self.n_channels = None

    def _resolve(self, path):
        obj = self.odrv
        for part in path.split("."):
            obj = getattr(obj, part)
        return obj

    def read(self, path):
        return self._resolve(path)

    def call_read_function(self, path):
        return self._resolve(path)()

    def scope_config(self, endpoints):
        endpoints = list(endpoints)
        scope_grid_point(0.0, int(self.read("oscilloscope.size")), len(endpoints))  # refuse first
        self.utils.ram_osci_config(self.odrv, endpoints)
        self.n_channels = len(endpoints)

    def scope_trigger(self, point):
        q = scope_grid_point(point, int(self.read("oscilloscope.size")), self.n_channels)
        self.utils.ram_osci_trigger(self.odrv, q)

    def scope_download(self, endpoints, timeout_s=SCOPE_STOP_TIMEOUT_S, clock=time.monotonic,
                       sleep=time.sleep):
        # The package's own wait has no bound; wait here first so a stuck scope is an error.
        t0 = clock()
        while self.read("oscilloscope.recording"):
            if clock() - t0 > timeout_s:
                self.utils.ram_osci_trigger(self.odrv, 1.0)   # 1.0 stops it at once
                raise RuntimeError("scope still recording %.1f s after the trigger; stopped it. "
                                   "See THE SCOPE'S RULES in this file." % timeout_s)
            sleep(0.02)
        return self.utils.ram_osci_download(self.odrv, list(endpoints))


def connect(serial=None, timeout=10.0):
    import odrive
    import odrive.utils
    return Drive(odrive.find_sync(serial_number=serial, timeout=timeout), odrive.utils)


def identity(drive):
    """Every read is defensive: a missing field is recorded as None plus why, never fatal."""
    out = {}
    for path in IDENTITY_READS:
        try:
            out[path] = drive.read(path)
        except Exception as e:  # a missing field must not lose the capture
            out[path] = None
            out[path + ".unreadable"] = str(e)
    try:
        out[FIELD_STRENGTH] = drive.call_read_function(FIELD_STRENGTH)
    except Exception as e:
        out[FIELD_STRENGTH] = None
        out[FIELD_STRENGTH + ".unreadable"] = str(e)
    return out


def motor_params(ident):
    return {"R": float(ident["axis0.config.motor.phase_resistance"]),
            "L": float(ident["axis0.config.motor.phase_inductance"]),
            "kt": float(ident["axis0.config.motor.torque_constant"]),
            "pole_pairs": int(ident["axis0.config.motor.pole_pairs"])}


def scope_timing(drive, trigger_point=0.8, measure_s=1.0, sleep=time.sleep, clock=time.monotonic):
    """Measure the sample rate, the window and where the trigger really lands."""
    t0, n0 = clock(), drive.read("n_evt_sampling")
    sleep(measure_s)
    t1, n1 = clock(), drive.read("n_evt_sampling")
    event_hz = (n1 - n0) / (t1 - t0)
    preset = PRESETS["timing"]
    drive.scope_config(preset.values())
    sleep(measure_s)                      # let the ring fill
    n_trigger = drive.read("n_evt_sampling")
    drive.scope_trigger(trigger_point)
    cols = dict(zip(preset, drive.scope_download(preset.values())))
    n = np.asarray(cols["n_sampling"], dtype=np.int64)
    step = np.diff(n)
    decimation = float(np.median(step))
    sample_hz = event_hz / decimation if decimation else 0.0
    return {
        "samples": int(len(n)), "event_hz": float(event_hz), "decimation": decimation,
        "sample_hz": float(sample_hz), "window_ms": 1e3 * len(n) / sample_hz if sample_hz else 0.0,
        "counter_gaps": int(np.sum(step != decimation)),
        "trigger_point_requested": float(trigger_point),
        "trigger_point_measured": float(np.searchsorted(n, n_trigger)) / max(1, len(n)),
    }


def wait_trigger(drive, trigger, timeout_s, clock=time.monotonic, on_note=None):
    """Poll over USB until the trigger fires; ``timeout_s`` None waits for ever.
    Returns (fired, polls, seconds). Fault mode does not read the speed."""
    t0, polls, seen = clock(), 0, 0
    while timeout_s is None or clock() - t0 < timeout_s:
        polls += 1
        vel = drive.read(VEL) if trigger.mode == "brake" else None
        fired = trigger.update(vel, drive.read(STATE), drive.read(DISARM), drive.read(ERRORS))
        if on_note is not None:
            for note in trigger.notes[seen:]:
                on_note(note)
            seen = len(trigger.notes)
        if fired:
            return True, polls, clock() - t0
    return False, polls, clock() - t0


def capture_once(drive, preset_name, trigger, trigger_point, timeout_s, fill_s, sleep=time.sleep,
                 clock=time.monotonic, on_note=None):
    """Arm, wait, freeze and download. A brake-mode timeout still saves the last window, tagged
    ``timeout``; a fault-mode timeout downloads nothing and returns cols None."""
    preset = PRESETS[preset_name]
    trigger.seed(drive.read(DISARM), drive.read(ERRORS))
    drive.scope_config(preset.values())
    sleep(fill_s)
    fired, polls, seconds = wait_trigger(drive, trigger, timeout_s, clock=clock, on_note=on_note)
    info = {"fired": fired, "kind": trigger.kind if fired else "timeout", "reason": trigger.reason,
            "mode": trigger.mode, "polls": polls, "poll_ms": 1e3 * seconds / max(1, polls),
            "trigger_point": trigger_point, "clean_exits": trigger.clean_exits,
            "at_arm": trigger.at_arm}
    if not fired and trigger.mode == "fault":
        return None, info
    drive.scope_trigger(trigger_point)
    cols = {k: np.asarray(v, dtype=float) for k, v in zip(preset, drive.scope_download(preset.values()))}
    return cols, info


# ---- files -------------------------------------------------------------------------------

def save_capture(cols, meta, label, out_dir=OUT_DIR):
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.join(out_dir, "%s_%s" % (time.strftime("%Y%m%d_%H%M%S"), label))
    names = list(cols)
    with open(stem + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(names)
        w.writerows(zip(*(cols[n].tolist() for n in names)))
    with open(stem + ".json", "w") as f:
        json.dump(meta, f, indent=1, default=str)
    return stem + ".csv"


def load_capture(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    cols = {name: np.array([float(r[i]) for r in rows[1:]]) for i, name in enumerate(rows[0])}
    with open(os.path.splitext(path)[0] + ".json") as f:
        return cols, json.load(f)


# ---- commands ----------------------------------------------------------------------------

def cmd_probe(args):
    drive = connect(args.serial)
    ident = identity(drive)
    print("\n".join(identity_lines(ident)))
    r = scope_timing(drive)
    preset = PRESETS["angle"]
    drive.scope_config(preset.values())
    time.sleep(0.5)
    drive.scope_trigger(0.5)
    cols = {k: np.asarray(v, dtype=float) for k, v in zip(preset, drive.scope_download(preset.values()))}
    r["angle_channels_finite"] = all(bool(np.isfinite(v).all()) for v in cols.values())
    # The channels share one byte buffer, so the 4-channel timing window overstates a real
    # capture. Judge the window of the preset that captures actually use.
    r["samples"] = int(len(cols["vbus"]))
    r["window_ms"] = 1e3 * r["samples"] / r["sample_hz"] if r["sample_hz"] else 0.0
    r["vbus"] = float(np.median(cols["vbus"]))
    reasons = judge_probe(r)
    print("scope: %d samples, %.0f Hz (decimation %.0f of %.0f Hz events), window %.1f ms, "
          "%.2f of the window before the trigger for a requested --trigger-point %.2f" % (
              r["samples"], r["sample_hz"], r["decimation"], r["event_hz"], r["window_ms"],
              r["trigger_point_measured"], r["trigger_point_requested"]))
    path = save_capture(cols, {"identity": ident, "identity_notes": IDENTITY_NOTES, "probe": r,
                               "sample_hz": r["sample_hz"], "preset": "angle",
                               "label": args.label + "_probe"},
                        args.label + "_probe")
    print("saved", path)
    print("GO" if not reasons else "NO-GO: " + "; ".join(reasons))
    return 0 if not reasons else 2


def cmd_capture(args):
    trigger_point = args.trigger_point if args.trigger_point is not None else DEFAULT_TRIGGER_POINT[args.trigger]
    timeout = args.timeout if args.timeout is not None else DEFAULT_TIMEOUT_S[args.trigger]
    drive = connect(args.serial)
    ident = identity(drive)
    size_note = ident.get("oscilloscope.size")
    sample_hz = scope_timing(drive)["sample_hz"]
    print("armed: preset %s, trigger %s, trigger point %.2f, %s; scope size %s bytes, %.0f Hz measured; "
          "Ctrl-C leaves the drive untouched" % (
              args.preset, args.trigger, trigger_point,
              "waiting until Ctrl-C" if timeout is None else "timeout %.0f s" % timeout, size_note, sample_hz))
    rc, done, n_file = 0, 0, 0
    while done < args.count:
        trig = BrakeTrigger(args.trigger, args.arm_vel, args.drop, save_clean=args.save_clean)
        try:
            cols, info = capture_once(drive, args.preset, trig, trigger_point, timeout, args.fill,
                                      on_note=lambda note: print("  %s; re-armed" % note))
        except KeyboardInterrupt:
            print("stopped by Ctrl-C after %d saved; %d clean exits seen while waiting; the drive is untouched" % (
                n_file, trig.clean_exits))
            return rc
        if cols is None:
            print("no fault in %.0f s; nothing saved (fault mode never saves a timeout)" % timeout)
            return 3
        meta = {"identity": ident, "identity_notes": IDENTITY_NOTES, "capture": info,
                "sample_hz": sample_hz, "preset": args.preset, "label": args.label}
        path = save_capture(cols, meta, "%s_%d_%s" % (args.label, n_file, info["kind"]))
        n_file += 1
        if info["kind"] != "clean":
            done += 1
        print("capture %d: %s; %.1f ms per poll; saved %s" % (
            n_file - 1, info["reason"] or "TIMED OUT, last window saved", info["poll_ms"], path))
        if not info["fired"]:
            rc = 3
        _, s = analyse(cols, motor_params(ident), sample_hz, spinout_params(ident))
        print("\n".join(summary_lines(s, motor_params(ident)["pole_pairs"])))
    return rc


def cmd_analyze(args):
    rc = 0
    for path in args.csv:
        cols, meta = load_capture(path)
        print("==", path)
        lines, triggered = record_lines(meta)
        print("\n".join(lines))
        if "capture" in meta and not triggered:
            rc = 3
        params = motor_params(meta["identity"])
        _, s = analyse(cols, params, float(meta["sample_hz"]), spinout_params(meta["identity"]))
        print("\n".join(summary_lines(s, params["pole_pairs"])))
    return rc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("probe", help="identity (with field notes) and the scope self-check; GO or NO-GO")
    p.add_argument("--serial")
    p.add_argument("--label", default="hand")
    p.set_defaults(func=cmd_probe)
    p = sub.add_parser("capture", help="arm the scope, freeze it on a brake or a fault, save and analyse")
    p.add_argument("--serial")
    p.add_argument("--label", default="hand")
    p.add_argument("--preset", choices=sorted(set(PRESETS) - {"timing"}), default="angle")
    p.add_argument("--trigger", choices=("brake", "fault"), default="brake")
    p.add_argument("--arm-vel", type=float, default=50.0, help="brake trigger: upward speed that arms it (rev/s)")
    p.add_argument("--drop", type=float, default=0.15, help="brake trigger: fall from the peak that fires it (fraction)")
    p.add_argument("--trigger-point", type=float, default=None,
                   help="fraction of the window kept before the trigger, rounded to a whole sample "
                        "(default: brake 0.5, fault 0.9)")
    p.add_argument("--count", type=int, default=1, help="brake or fault captures to save (clean saves do not count)")
    p.add_argument("--timeout", type=float, default=None,
                   help="seconds to wait per capture (default: brake 60, fault none); a fault-mode timeout saves nothing")
    p.add_argument("--save-clean", action="store_true",
                   help="also save a window, tagged _clean, at each clean exit from CLOSED_LOOP")
    p.add_argument("--fill", type=float, default=0.5, help="seconds between arming and watching, so the ring fills")
    p.set_defaults(func=cmd_capture)
    p = sub.add_parser("analyze", help="re-run the analysis on saved captures")
    p.add_argument("csv", nargs="+")
    p.set_defaults(func=cmd_analyze)
    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
