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

THE ANGLE ERROR. The onboard encoder drives both commutation and position, so a slipped
magnet moves ``foc.phase`` and position together and neither shows it. The back-EMF does.
With the drive's feedforward off, ``foc.Vd_setpoint`` and ``foc.Vq_setpoint`` are unused
inputs, so the applied voltage comes from the modulation:

    Vd = mod_d * (2/3) * vbus        Vq = mod_q * (2/3) * vbus
    e_d = Vd - R*Id + w*L*Iq         e_q = Vq - R*Iq - w*L*Id        (w = foc.phase_vel)
    correct commutation: e_d ~ 0, e_q ~ w*lambda
    angle error:         delta = atan2(-e_d * sign(w), e_q * sign(w))

Read delta against speed, not as an absolute. The loop's delay and any error in R and L
give a speed-dependent baseline, and a clean brake measures that baseline. R*I and w*L*I
are as large as the back-EMF near 50 A, so delta is best at low current. A fault shows as
a departure from the baseline. The 2/3 scaling is checked by the ``power`` preset, which
records the drive's own ``electrical_power`` beside the value computed here. ``foc.phase_vel`` is
taken as electrical rad/s; if it were not, nearly every sample at speed would count as a
phase jump and the fitted flux linkage would miss the value from Kt.

Usage (launch may stay down; the motion tool owns the can-bridge link, this one owns USB):
    python tools/probes/hand_foc_scope.py probe --label bench
    python tools/probes/hand_foc_scope.py capture --trigger brake --label bench_70
    python tools/probes/hand_foc_scope.py analyze temp/hand_foc_scope/<file>.csv

THE SCOPE'S RULES (measured on the bench Hand's drive, FW 0.6.11, 2026-11-11). The ring is
65536 bytes, 4 bytes per channel, at most 9 channels. A trigger ends the recording only when
the bytes left to record after it land on a sample boundary, and the ring only wraps on one
when the sample size divides it. Otherwise the drive records for ever and the odrive
package's download waits for ever: the first probe on a drive hung that way. So every
preset has 8 channels or fewer with a sample size that divides the ring
(``scope_grid_point`` refuses one that does not), every trigger point is rounded to a whole
sample, and the download wait is bounded. ``oscilloscope.trigger_high_res()`` dropped the
drive off USB and restarted it, losing its position reference; never call it.
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

FOC = "axis0.motor.foc."
MOTOR = "axis0.motor."
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
    "timing": {
        "n_sampling": "n_evt_sampling", "n_control": "n_evt_control_loop",
        "phase": FOC + "phase", "phase_vel": FOC + "phase_vel",
    },
}

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
    "axis0.commutation_mapper.working_offset", "onboard_encoder0.status",
    "onboard_encoder0.raw",
)
FIELD_STRENGTH = "onboard_encoder0.get_field_strength"
VEL = "axis0.pos_vel_mapper.vel"
STATE = "axis0.current_state"
DISARM = "axis0.disarm_reason"
CLOSED_LOOP = 8

MIN_WINDOW_MS = 60.0       # a C1 brake is about 40 ms; below this a capture cannot hold one
MIN_SAMPLE_HZ = 4000.0     # at 137 rev/s the electrical period is 1 ms
TRIGGER_POINT_TOLERANCE = 0.15

SPEED_BINS = (0.0, 20.0, 50.0, 80.0, 110.0, 140.0, 1e9)
MIN_EMF_V = 0.5            # below this the back-EMF vector is too small to give an angle
PHASE_JUMP_RAD = 0.5       # one sample's phase step against phase_vel
TORQUE_MIN_A = 10.0        # the torque check needs current well above friction and gravity
TORQUE_LOSS_FRACTION = 0.25  # the 2026-11-03 events kept 25-45 % of nominal at best
TORQUE_LOSS_MIN_MS = 2.0
TORQUE_MIN_VEL = 10.0      # at rest, holding current moves nothing and proves nothing
VEL_SMOOTH_MS = 2.0


# ---- analysis (pure) --------------------------------------------------------------------

def wrap_pi(x):
    return (np.asarray(x) + np.pi) % (2.0 * np.pi) - np.pi


def _runs(mask):
    """(start, stop) index pairs of the True runs in a boolean array."""
    edges = np.flatnonzero(np.diff(np.r_[False, mask, False].astype(int)))
    return list(zip(edges[::2], edges[1::2]))


def _smooth(x, n):
    """Moving average, padded with the end values so the window's edges read no false drop."""
    return np.convolve(np.pad(x, (n // 2, n - 1 - n // 2), mode="edge"), np.ones(n) / n, mode="valid")


def analyse(c, params, fs):
    """Derive the angle, torque and encoder checks from one capture. ``c`` maps the column
    names of a preset to arrays; ``params`` holds R, L, pole_pairs and kt."""
    R, L, kt = params["R"], params["L"], params["kt"]
    vd = c["mod_d"] * (2.0 / 3.0) * c["vbus"]
    vq = c["mod_q"] * (2.0 / 3.0) * c["vbus"]
    i_d, i_q, w = c["id"], c["iq"], c["phase_vel"]
    e_d = vd - R * i_d + w * L * i_q
    e_q = vq - R * i_q - w * L * i_d
    sgn = np.where(w >= 0.0, 1.0, -1.0)
    delta = np.degrees(np.arctan2(-e_d * sgn, e_q * sgn))
    valid = np.hypot(e_d, e_q) >= MIN_EMF_V

    n_smooth = max(1, int(round(VEL_SMOOTH_MS * 1e-3 * fs)))
    vel = _smooth(np.gradient(c["pos"]) * fs, n_smooth)
    acc = _smooth(np.gradient(vel) * fs, n_smooth)

    out = {"vd": vd, "vq": vq, "e_d": e_d, "e_q": e_q, "delta_deg": delta, "valid": valid,
           "vel": vel, "acc": acc}
    s = {
        "samples": int(len(w)), "window_ms": 1e3 * len(w) / fs,
        "vel_max": float(vel.max()), "vel_min": float(vel.min()),
        "iq_min": float(i_q.min()), "iq_max": float(i_q.max()),
        "mod_max": float(np.hypot(c["mod_d"], c["mod_q"]).max()),
        "vbus_min": float(c["vbus"].min()), "vbus_max": float(c["vbus"].max()),
    }

    bins = []
    for lo, hi in zip(SPEED_BINS[:-1], SPEED_BINS[1:]):
        m = valid & (np.abs(vel) >= lo) & (np.abs(vel) < hi)
        if m.any():
            bins.append({"lo": lo, "hi": hi, "n": int(m.sum()),
                         "delta_median": float(np.median(delta[m])),
                         "delta_p5": float(np.percentile(delta[m], 5)),
                         "delta_p95": float(np.percentile(delta[m], 95)),
                         "iq_abs_median": float(np.median(np.abs(i_q[m])))})
    s["delta_by_speed"] = bins
    if valid.sum() >= 2:
        s["lambda_fit"] = float(np.dot(w[valid], e_q[valid]) / np.dot(w[valid], w[valid]))
    s["lambda_from_kt"] = kt / (1.5 * params["pole_pairs"])

    if "phase" in c:
        step = wrap_pi(np.diff(c["phase"]) - w[:-1] / fs)
        jumps = np.flatnonzero(np.abs(step) > PHASE_JUMP_RAD)
        s["phase_jumps"] = int(len(jumps))
        s["phase_jump_first_ms"] = float(1e3 * jumps[0] / fs) if len(jumps) else None

    loaded = (np.abs(i_q) >= TORQUE_MIN_A) & (np.abs(vel) >= TORQUE_MIN_VEL)
    s["torque_loss"] = []
    if loaded.sum() >= n_smooth:
        ratio = acc[loaded] / i_q[loaded]
        k = float(np.median(ratio))
        s["accel_per_amp"] = k
        if k > 0.0:
            weak = loaded & (acc / np.where(i_q == 0.0, np.inf, i_q) < TORQUE_LOSS_FRACTION * k)
            min_n = TORQUE_LOSS_MIN_MS * 1e-3 * fs
            s["torque_loss"] = [
                {"start_ms": 1e3 * a / fs, "ms": 1e3 * (b - a) / fs,
                 "iq": float(np.median(i_q[a:b])), "vel": float(np.median(vel[a:b])),
                 "pos": float(np.median(c["pos"][a:b]))}
                for a, b in _runs(weak) if b - a >= min_n]

    # The SPINOUT inputs, unfiltered: the drive low-passes both before it compares.
    p_e = 1.5 * (vd * i_d + vq * i_q)
    p_m = kt * i_q * 2.0 * np.pi * vel
    out["p_elec_calc"], out["p_mech_calc"] = p_e, p_m
    s["spinout_condition_ms"] = float(1e3 * np.sum((p_e > 10.0) & (p_m < -10.0)) / fs)
    if "p_elec" in c and np.max(np.abs(c["p_elec"])) > 20.0:
        big = np.abs(c["p_elec"]) > 20.0
        s["p_elec_calc_over_drive"] = float(np.median(p_e[big] / c["p_elec"][big]))
    return out, s


def summary_lines(s):
    """The human summary of one capture."""
    lines = [
        "window %.1f ms, %d samples; speed %+.0f to %+.0f rev/s; iq %+.1f to %+.1f A; "
        "max modulation %.3f; vbus %.1f to %.1f V" % (
            s["window_ms"], s["samples"], s["vel_min"], s["vel_max"], s["iq_min"], s["iq_max"],
            s["mod_max"], s["vbus_min"], s["vbus_max"]),
        "angle error by speed (read against a clean brake, not as an absolute):",
    ]
    for b in s["delta_by_speed"]:
        lines.append("  %3.0f-%-5s rev/s  n %6d  delta %+7.1f deg (p5 %+7.1f, p95 %+7.1f), |iq| %5.1f A" % (
            b["lo"], ("%.0f" % b["hi"]) if b["hi"] < 1e8 else "up", b["n"], b["delta_median"],
            b["delta_p5"], b["delta_p95"], b["iq_abs_median"]))
    if "lambda_fit" in s:
        lines.append("flux linkage: fitted %.3e, from Kt %.3e V s/rad" % (s["lambda_fit"], s["lambda_from_kt"]))
    if "phase_jumps" in s:
        lines.append("encoder phase jumps over %.1f rad: %d%s" % (
            PHASE_JUMP_RAD, s["phase_jumps"],
            "" if s["phase_jump_first_ms"] is None else " (first at %.1f ms)" % s["phase_jump_first_ms"]))
    if "accel_per_amp" in s:
        lines.append("accel per amp above %.0f A and %.0f rev/s: %.1f rev/s^2/A" % (
            TORQUE_MIN_A, TORQUE_MIN_VEL, s["accel_per_amp"]))
    if s["torque_loss"]:
        for t in s["torque_loss"]:
            lines.append("TORQUE LOSS at %.1f ms for %.1f ms: iq %+.1f A, %+.0f rev/s, %.2f rev" % (
                t["start_ms"], t["ms"], t["iq"], t["vel"], t["pos"]))
    else:
        lines.append("torque loss (under %.0f %% of the capture's median, %.0f ms or longer): none" % (
            100 * TORQUE_LOSS_FRACTION, TORQUE_LOSS_MIN_MS))
    lines.append("SPINOUT condition, unfiltered (P_elec > 10 W and P_mech < -10 W): %.1f ms" % s["spinout_condition_ms"])
    if "p_elec_calc_over_drive" in s:
        lines.append("computed electrical power / the drive's own: %.3f (1.0 confirms the 2/3 scaling)" % s["p_elec_calc_over_drive"])
    return lines


class BrakeTrigger:
    """When to freeze the scope, fed one (vel, state, disarm_reason) per USB poll.

    ``brake`` fires once the speed has passed ``arm_vel`` upward and fallen ``drop`` below its
    peak. ``fault`` waits for a disarm only, for catching a rare event. Both fire on a disarm,
    so a fault during a brake capture is never lost."""

    def __init__(self, mode, arm_vel=50.0, drop=0.15):
        if mode not in ("brake", "fault"):
            raise ValueError("unknown trigger mode %r" % (mode,))
        self.mode, self.arm_vel, self.drop = mode, arm_vel, drop
        self.peak, self.was_closed_loop, self.reason = 0.0, False, None

    def update(self, vel, state, disarm_reason):
        if state == CLOSED_LOOP:
            self.was_closed_loop = True
        if self.was_closed_loop and (state != CLOSED_LOOP or disarm_reason):
            self.reason = "disarm (state %s, disarm_reason 0x%x)" % (state, int(disarm_reason))
            return True
        if self.mode == "brake":
            self.peak = max(self.peak, vel)
            if self.peak >= self.arm_vel and vel <= self.peak * (1.0 - self.drop):
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
    out = {}
    for path in IDENTITY_READS:
        try:
            out[path] = drive.read(path)
        except Exception as e:  # a missing field must not lose the capture
            out[path] = "unreadable: %s" % (e,)
    try:
        out[FIELD_STRENGTH] = drive.call_read_function(FIELD_STRENGTH)
    except Exception as e:
        out[FIELD_STRENGTH] = "unreadable: %s" % (e,)
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


def wait_trigger(drive, trigger, timeout_s, clock=time.monotonic):
    """Poll over USB until the trigger fires. Returns (fired, polls, seconds)."""
    t0, polls = clock(), 0
    while clock() - t0 < timeout_s:
        polls += 1
        if trigger.update(drive.read(VEL), drive.read(STATE), drive.read(DISARM)):
            return True, polls, clock() - t0
    return False, polls, clock() - t0


def capture_once(drive, preset_name, trigger, trigger_point, timeout_s, fill_s, sleep=time.sleep):
    preset = PRESETS[preset_name]
    drive.scope_config(preset.values())
    sleep(fill_s)
    fired, polls, seconds = wait_trigger(drive, trigger, timeout_s)
    drive.scope_trigger(trigger_point)    # a timeout still saves the last window, marked as such
    cols = {k: np.asarray(v, dtype=float) for k, v in zip(preset, drive.scope_download(preset.values()))}
    return cols, {"fired": fired, "reason": trigger.reason, "polls": polls,
                  "poll_ms": 1e3 * seconds / max(1, polls), "trigger_point": trigger_point}


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
    for k in sorted(ident):
        print("%-48s %s" % (k, ident[k]))
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
    path = save_capture(cols, {"identity": ident, "probe": r, "sample_hz": r["sample_hz"],
                               "preset": "angle", "label": args.label + "_probe"},
                        args.label + "_probe")
    print("saved", path)
    print("GO" if not reasons else "NO-GO: " + "; ".join(reasons))
    return 0 if not reasons else 2


def cmd_capture(args):
    drive = connect(args.serial)
    ident = identity(drive)
    size_note = ident.get("oscilloscope.size")
    sample_hz = scope_timing(drive)["sample_hz"]
    print("armed: preset %s, trigger %s, scope size %s bytes, %.0f Hz measured; "
          "Ctrl-C leaves the drive untouched" % (args.preset, args.trigger, size_note, sample_hz))
    rc = 0
    for k in range(args.count):
        trig = BrakeTrigger(args.trigger, args.arm_vel, args.drop)
        cols, info = capture_once(drive, args.preset, trig, args.trigger_point, args.timeout, args.fill)
        meta = {"identity": ident, "capture": info, "sample_hz": sample_hz,
                "preset": args.preset, "label": args.label}
        path = save_capture(cols, meta, "%s_%d" % (args.label, k))
        print("capture %d: %s; %.1f ms per poll; saved %s" % (
            k, info["reason"] or "TIMED OUT, last window saved", info["poll_ms"], path))
        if not info["fired"]:
            rc = 3
        _, s = analyse(cols, motor_params(ident), sample_hz)
        print("\n".join(summary_lines(s)))
    return rc


def cmd_analyze(args):
    rc = 0
    for path in args.csv:
        cols, meta = load_capture(path)
        print("==", path)
        _, s = analyse(cols, motor_params(meta["identity"]), float(meta["sample_hz"]))
        print("\n".join(summary_lines(s)))
    return rc


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("probe", help="identity, magnet field strength and the scope self-check; GO or NO-GO")
    p.add_argument("--serial")
    p.add_argument("--label", default="hand")
    p.set_defaults(func=cmd_probe)
    p = sub.add_parser("capture", help="arm the scope, freeze it on a brake or a disarm, save and analyse")
    p.add_argument("--serial")
    p.add_argument("--label", default="hand")
    p.add_argument("--preset", choices=sorted(set(PRESETS) - {"timing"}), default="angle")
    p.add_argument("--trigger", choices=("brake", "fault"), default="brake")
    p.add_argument("--arm-vel", type=float, default=50.0, help="brake trigger: upward speed that arms it (rev/s)")
    p.add_argument("--drop", type=float, default=0.15, help="brake trigger: fall from the peak that fires it (fraction)")
    p.add_argument("--trigger-point", type=float, default=0.5, help="fraction of the window kept before the trigger")
    p.add_argument("--count", type=int, default=1)
    p.add_argument("--timeout", type=float, default=60.0, help="seconds to wait per capture")
    p.add_argument("--fill", type=float, default=0.5, help="seconds between arming and watching, so the ring fills")
    p.set_defaults(func=cmd_capture)
    p = sub.add_parser("analyze", help="re-run the analysis on saved captures")
    p.add_argument("csv", nargs="+")
    p.set_defaults(func=cmd_analyze)
    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
