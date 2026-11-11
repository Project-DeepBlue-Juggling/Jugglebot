"""Offline pins for tools/probes/hand_foc_scope.py (BUG-57, FEAT-45): the units and frame of the
drive's signals, the angle recovery and its honesty gate, the torque check, the SPINOUT model,
the trigger, the capture bookkeeping, the probe verdict, the read-only contract and the
endpoints it relies on.

The units and frame facts are from the bench Hand's real captures (FW 0.6.11, 2026-11-11):
``foc.phase`` advanced exactly -7.000 turns per pos_rel turn and wrapped in [0, 1);
``foc.phase_vel`` / d(pos_rel)/dt was -6.85. The synthetic captures below reproduce that."""
from __future__ import annotations

import ast
import json
import os
import sys

import numpy as np
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "tools", "probes"))
import hand_foc_scope as hfs  # noqa: E402

FS = 8000.0
PARAMS = {"R": 0.040, "L": 1.089e-5, "kt": 0.00551, "pole_pairs": 7}
PRO_TABLE = os.path.join(REPO, "config", "ODrive config Files",
                         "odrive-pro-4.4.58-0.6.11-1_flat_endpoints.json")
SPIN = ({hfs.SPIN_E_THRESHOLD: 10.0, hfs.SPIN_M_THRESHOLD: -10.0,
         hfs.SPIN_E_BANDWIDTH: 20.0, hfs.SPIN_M_BANDWIDTH: 20.0}, "test")


def synth(vel_rps, iq, delta_deg=0.0, vbus=48.0, frame=1.0, r_extra=0.0, v_step=0.0):
    """Drive the voltage equations forwards, with a commutation error of delta_deg.

    ``vel_rps`` is d(pos_rel)/dt; ``frame`` is how the electrical frame counts against it
    (-1 on the bench Hand). ``iq`` is in the drive's frame. phase is in turns [0, 1) and
    phase_vel in electrical turns/s, as FW 0.6.11 reports them. ``r_extra`` and ``v_step``
    are resistance and a dead-time-like step the analysis does not know about."""
    vel = np.asarray(vel_rps, dtype=float)
    iq = np.broadcast_to(np.asarray(iq, dtype=float), vel.shape).copy()
    pv = frame * vel * PARAMS["pole_pairs"]              # electrical turns/s
    w = 2.0 * np.pi * pv
    lam = PARAMS["kt"] / (1.5 * PARAMS["pole_pairs"])
    d = np.radians(delta_deg)
    e_d, e_q = -w * lam * np.sin(d), w * lam * np.cos(d)
    i_d = np.zeros_like(iq)
    r = PARAMS["R"] + r_extra
    vd = e_d + r * i_d - w * PARAMS["L"] * iq
    vq = e_q + r * iq + w * PARAMS["L"] * i_d + v_step * np.sign(iq)
    scale = (2.0 / 3.0) * vbus
    pos = np.cumsum(vel) / FS
    return {"mod_d": vd / scale, "mod_q": vq / scale, "vbus": np.full_like(vel, vbus),
            "id": i_d, "iq": iq, "phase_vel": pv, "phase": (np.cumsum(pv) / FS) % 1.0, "pos": pos}


def stroke(frame=1.0, k=86.7, n_up=800, n_brake=400, n_rest=400, loss=None):
    """Up at 1600 rev/s^2 to 160 rev/s, brake at -3200 to rest, then rest."""
    acc = np.r_[np.full(n_up, 1600.0), np.full(n_brake, -3200.0), np.zeros(n_rest)]
    vel = np.cumsum(acc) / FS
    iq = frame * acc / k
    if loss is not None:
        a, b = loss
        acc2 = acc.copy()
        acc2[a:b] = acc[a:b] * 0.1
        vel = np.cumsum(acc2) / FS
    return synth(vel, iq, frame=frame)


# ---- units --------------------------------------------------------------------------------

@pytest.mark.parametrize("delta", [0.0, 12.0, 30.0, -45.0])
@pytest.mark.parametrize("sign", [1.0, -1.0])
@pytest.mark.parametrize("frame", [1.0, -1.0])
def test_angle_error_is_recovered_in_both_directions_and_frames(delta, sign, frame):
    """phase_vel in turns/s: w = 2*pi*phase_vel. Taken as rad/s the angle came out ~-7 deg."""
    _, s = hfs.analyse(synth(np.full(800, sign * 100.0), 3.0 * frame * sign, delta, frame=frame), PARAMS, FS)
    b = [b for b in s["delta_by_speed"] if b["n"]][0]
    assert b["delta_median"] == pytest.approx(delta, abs=0.5)


def test_correct_commutation_fits_the_flux_linkage_from_kt():
    _, s = hfs.analyse(synth(np.linspace(60, 130, 800), 1.0), PARAMS, FS)
    assert s["lambda_fit"] == pytest.approx(s["lambda_from_kt"], rel=0.01)


def test_a_clean_phase_in_turns_with_many_wraps_has_no_jumps():
    """At 137 rev/s the turns signal wraps every 8 samples; the radian reading of the first
    version counted each wrap as a jump (22-24 per bench stroke)."""
    c = synth(np.full(2000, 137.0), 10.0, frame=-1.0)
    s = hfs.analyse(c, PARAMS, FS)[1]
    assert s["phase_jumps"] == 0 and s["phase_residual_max"] < 1e-6
    assert np.sum(np.abs(np.diff(c["phase"])) > 0.5) > 200      # it really wraps


def test_an_encoder_jump_is_counted():
    c = synth(np.full(800, 100.0), 10.0)
    c["phase"] = (c["phase"] + np.where(np.arange(800) >= 400, 0.2, 0.0)) % 1.0
    s = hfs.analyse(c, PARAMS, FS)[1]
    assert s["phase_jumps"] == 1 and s["phase_jump_first_ms"] == pytest.approx(399 / 8.0)


# ---- frame and torque -------------------------------------------------------------------

@pytest.mark.parametrize("frame", [1.0, -1.0])
def test_the_frame_is_taken_from_the_data_and_accel_per_amp_is_positive(frame):
    s = hfs.analyse(stroke(frame=frame), PARAMS, FS)[1]
    assert s["frame_sign"] == frame
    assert s["frame_ratio"] == pytest.approx(frame * 7.0, rel=0.01)
    assert s["accel_per_amp"] == pytest.approx(86.7, rel=0.05)
    assert s["torque_fit"]["K"] == pytest.approx(86.7, rel=0.05)
    assert s["torque_loss"] == []


@pytest.mark.parametrize("frame", [1.0, -1.0])
def test_a_torque_loss_is_found_in_either_frame(frame):
    """The first version skipped the detector whenever accel per amp was negative, which the
    bench Hand's opposite frame made it on every capture, and printed "none"."""
    s = hfs.analyse(stroke(frame=frame, loss=(900, 1000)), PARAMS, FS)[1]
    assert len(s["torque_loss"]) == 1
    loss = s["torque_loss"][0]
    assert loss["start_ms"] == pytest.approx(900 / 8.0, abs=3.0)
    assert loss["ms"] == pytest.approx(100 / 8.0, abs=4.0)


def test_the_detector_runs_when_torque_opposes_current_and_says_so():
    c = stroke(frame=1.0, loss=(900, 1000))
    c["iq"] = -c["iq"]                       # current sign flipped against the motion
    s = hfs.analyse(c, PARAMS, FS)[1]
    assert s["accel_per_amp"] < 0 and len(s["torque_loss"]) == 1
    assert "NEGATIVE" in "\n".join(hfs.summary_lines(s))


def test_a_capture_at_rest_says_the_torque_check_was_not_evaluated_never_none():
    s = hfs.analyse(synth(np.zeros(800), 0.3), PARAMS, FS)[1]
    text = "\n".join(hfs.summary_lines(s, 7))
    assert s["torque_loss"] is None
    assert "torque loss: not evaluated:" in text and "none found" not in text
    assert "frame: not evaluated:" in text


def test_holding_current_at_rest_after_a_brake_is_not_a_torque_loss():
    """After the stop the position loop can hold over 10 A with no motion."""
    vel = np.r_[137.0 - 2600.0 * np.arange(400) / FS, np.zeros(200)]
    vel[400:] = 0.0
    iq = np.r_[np.full(400, -30.0), np.full(200, -12.0)]
    assert hfs.analyse(synth(vel, iq), PARAMS, FS)[1]["torque_loss"] == []


# ---- the voltage model's honesty --------------------------------------------------------

def test_the_angle_is_withheld_where_r_and_wl_terms_rival_the_back_emf():
    """30 A at 100 rev/s: R*I 1.2 V and w*L*I 1.4 V against a 2.3 V back-EMF."""
    s = hfs.analyse(synth(np.full(800, 100.0), 30.0, 30.0), PARAMS, FS)[1]
    assert s["angle_qualified"] == 0 and s["emf_samples"] == 800
    assert "not reported: 0 of 800" in "\n".join(hfs.summary_lines(s))


def test_samples_with_too_little_back_emf_give_no_angle():
    out, s = hfs.analyse(synth(np.full(800, 1.0), 0.0, 30.0), PARAMS, FS)
    assert not out["valid"].any() and s["delta_by_speed"] == []


def test_effective_r_and_the_step_are_fitted_and_reported_not_applied():
    n = 3000
    vel = 40.0 + 60.0 * np.abs(np.sin(np.linspace(0, 3 * np.pi, n)))
    ramp = np.linspace(0.0, 5.5, n) % 1.0
    iq = 30.0 * (2.0 * np.abs(2.0 * ramp - 1.0) - 1.0)          # triangle: every current level
    c = synth(vel, iq, frame=-1.0, r_extra=0.018, v_step=0.28)
    out, s = hfs.analyse(c, PARAMS, FS)
    v = s["vfit"]
    assert v["R_eff"] == pytest.approx(PARAMS["R"] + 0.018, abs=0.002)
    assert v["v_step"] == pytest.approx(0.28, abs=0.03)
    assert abs(v["corr_R_step"]) < hfs.VFIT_CORR_LIMIT
    # not applied: e_q is still built with the configured R
    vq = c["mod_q"] * (2.0 / 3.0) * c["vbus"]
    assert np.allclose(out["e_q"], vq - PARAMS["R"] * c["iq"] - out["w"] * PARAMS["L"] * c["id"])
    assert s["angle_step_allowance"][0] == pytest.approx(0.28, abs=0.03)
    assert "NOT applied" in "\n".join(hfs.summary_lines(s))


def test_a_two_level_current_flags_r_and_the_step_as_not_separately_identified():
    """The bench stroke is close to this: about -13 A up, +30 A in the brake."""
    vel = np.linspace(30.0, 90.0, 2000)
    iq = np.where(np.arange(2000) < 1200, -13.0, 30.0)
    s = hfs.analyse(synth(vel, iq, r_extra=0.018, v_step=0.28), PARAMS, FS)[1]
    assert abs(s["vfit"]["corr_R_step"]) > hfs.VFIT_CORR_LIMIT
    assert "not separately identified" in "\n".join(hfs.summary_lines(s))


# ---- the SPINOUT model ------------------------------------------------------------------

def test_lowpass_is_first_order_at_its_bandwidth():
    y = hfs.lowpass(np.r_[0.0, np.ones(8000)], 20.0, FS)
    assert y[0] == 0.0 and y[400] == pytest.approx(1.0 - np.exp(-1.0), abs=0.005)


def _power_capture(p_elec, p_mech, n=4000):
    """phase_vel and iq chosen so kt*iq*2*pi*phase_vel/pp is p_mech."""
    pv = np.full(n, -500.0)
    iq = np.full(n, p_mech) / (PARAMS["kt"] * 2.0 * np.pi * pv / PARAMS["pole_pairs"])
    return {"vbus": np.full(n, 46.0), "id": np.zeros(n), "iq": iq, "phase_vel": pv,
            "p_elec": np.full(n, float(p_elec))}


def test_spinout_margins_use_the_drives_electrical_power_and_say_the_mech_formula_is_assumed():
    s = hfs.analyse(_power_capture(30.0, -5.0), PARAMS, FS, SPIN)[1]
    p = s["spinout"]
    assert p["e_margin"] == pytest.approx(20.0, abs=0.5) and p["m_margin"] == pytest.approx(-5.0, abs=0.5)
    assert p["trip_ms"] == 0.0
    text = "\n".join(hfs.summary_lines(s))
    assert "motor.electrical_power channel" in text and "ASSUMED" in text and "no trip predicted" in text
    s = hfs.analyse(_power_capture(30.0, -20.0), PARAMS, FS, SPIN)[1]
    assert s["spinout"]["joint_margin"] > 0 and "TRIP predicted" in "\n".join(hfs.summary_lines(s))


def test_mechanical_power_is_negative_in_a_brake_whichever_way_pos_counts():
    """The first version used pos_rel's speed, so on the bench Hand it counted the run-up
    (motoring) as generating and was blind in the brake."""
    for frame in (1.0, -1.0):
        out, s = hfs.analyse(stroke(frame=frame), PARAMS, FS, SPIN)
        assert s["spinout"]["p_mech_filt_min"] < 0.0
        assert out["p_mech_filt"][1100] < out["p_mech_filt"][700]   # brake below run-up


def test_spinout_values_come_from_the_drive_else_are_labelled_assumed_else_not_evaluated(tmp_path):
    ident = {k: v for k, v in SPIN[0].items()}
    vals, src = hfs.spinout_params(ident)
    assert vals[hfs.SPIN_M_THRESHOLD] == -10.0 and src == "read from this drive"
    vals, src = hfs.spinout_params({hfs.SPIN_E_THRESHOLD: None})
    assert vals[hfs.SPIN_E_BANDWIDTH] == 20.0 and "NOT read from this drive" in src
    vals, src = hfs.spinout_params({}, config_path=str(tmp_path / "missing.json"))
    assert vals is None
    s = hfs.analyse(_power_capture(30.0, -5.0), PARAMS, FS, (vals, src))[1]
    assert "SPINOUT model: not evaluated" in "\n".join(hfs.summary_lines(s))


def test_power_preset_confirms_the_scaling_against_the_drives_figure():
    c = synth(np.full(800, 100.0), 30.0)
    vd, vq = c["mod_d"] * 32.0, c["mod_q"] * 32.0
    c["p_elec"] = 1.5 * (vd * c["id"] + vq * c["iq"])
    assert hfs.analyse(c, PARAMS, FS)[1]["p_elec_calc_over_drive"] == pytest.approx(1.0)
    c["p_elec"] = c["p_elec"] * 1.5
    assert hfs.analyse(c, PARAMS, FS)[1]["p_elec_calc_over_drive"] == pytest.approx(1.0 / 1.5)


def test_summary_names_a_torque_loss_and_every_speed_bin_with_back_emf():
    accel = np.full(800, -2600.0)
    accel[500:600] = -100.0
    _, s = hfs.analyse(synth(137.0 + np.cumsum(accel) / FS, -30.0), PARAMS, FS)
    text = "\n".join(hfs.summary_lines(s))
    assert "TORQUE LOSS" in text and "80-110" in text and "110-140" in text


# ---- the trigger ------------------------------------------------------------------------

def test_brake_trigger_arms_on_speed_and_fires_on_the_fall():
    t = hfs.BrakeTrigger("brake", arm_vel=50.0, drop=0.15)
    assert not any(t.update(v, 8, 0) for v in (0.0, 30.0, 49.0, 40.0))
    assert not any(t.update(v, 8, 0) for v in (60.0, 70.0, 65.0, 60.0))
    assert t.update(59.0, 8, 0) and "peak 70.0" in t.reason and t.kind == "brake"


def test_brake_trigger_ignores_a_downward_stroke():
    t = hfs.BrakeTrigger("brake")
    assert not any(t.update(v, 8, 0) for v in (0.0, -60.0, -100.0, -60.0, 0.0))


def test_fault_trigger_fires_on_a_disarm_reason_edge_and_never_on_a_brake():
    t = hfs.BrakeTrigger("fault")
    assert not any(t.update(v, 8, 0) for v in (137.0, 60.0, 0.0))
    assert t.update(None, 1, 0x4000000) and "0x4000000" in t.reason and t.kind == "fault"


def test_fault_trigger_fires_on_an_active_errors_edge():
    t = hfs.BrakeTrigger("fault")
    assert not t.update(None, 8, 0, 0)
    assert t.update(None, 8, 0, 0x800) and t.kind == "fault"


@pytest.mark.parametrize("mode", ["brake", "fault"])
def test_a_reason_latched_before_arming_never_fires(mode):
    """The first version fired on any non-zero disarm_reason, so a stale one fired at once."""
    t = hfs.BrakeTrigger(mode)
    assert not any(t.update(0.0, 8, 0x4000000, 0x800) for _ in range(5))
    assert t.at_arm == (0x4000000, 0x800)
    assert t.update(0.0, 8, 0x8000, 0x800) and t.kind == "fault"      # a new value is an edge


@pytest.mark.parametrize("mode", ["brake", "fault"])
def test_a_clean_exit_from_closed_loop_re_arms_without_firing(mode):
    """Park, a Soft E-stop or launch shutdown takes the drive to IDLE with no new reason."""
    t = hfs.BrakeTrigger(mode)
    assert not any(t.update(0.0, s, 0) for s in (1, 8, 8, 1, 1))
    assert t.clean_exits == 1 and t.kind is None and "clean exit" in t.notes[0]
    assert not any(t.update(0.0, s, 0) for s in (8, 8))
    assert t.update(0.0, 1, 0x4000000) and t.kind == "fault"


def test_a_clean_exit_fires_only_behind_save_clean():
    t = hfs.BrakeTrigger("fault", save_clean=True)
    assert not t.update(None, 8, 0)
    assert t.update(None, 1, 0) and t.kind == "clean"


def test_trigger_needs_closed_loop_first_so_an_idle_drive_does_not_fire():
    t = hfs.BrakeTrigger("fault", save_clean=True)
    assert not any(t.update(None, 1, 0) for _ in range(3))


def test_fault_mode_defaults_keep_the_onset_before_the_trigger_and_wait_for_ever():
    assert hfs.DEFAULT_TRIGGER_POINT["fault"] == 0.9 and hfs.DEFAULT_TIMEOUT_S["fault"] is None


def test_the_fault_default_point_still_goes_through_the_scope_grid():
    """0.9 is not a whole sample on 32-byte samples; the drive must receive 1843/2048."""
    scope = _Scope(stops_after=0)
    drive = hfs.Drive(_Odrv(scope), _Utils(scope))
    drive.scope_config(list(hfs.PRESETS["power"].values()))
    drive.scope_trigger(hfs.DEFAULT_TRIGGER_POINT["fault"])
    assert scope.triggers == [1843 / 2048] and ((1.0 - scope.triggers[0]) * 65536) % 32 == 0


GOOD = {"window_ms": 100.0, "sample_hz": 8000.0, "counter_gaps": 0, "trigger_point_measured": 0.8,
        "trigger_point_requested": 0.8, "angle_channels_finite": True, "vbus": 48.0}


def test_probe_goes_when_every_check_holds():
    assert hfs.judge_probe(GOOD) == []


def test_probe_accepts_the_trigger_fraction_read_either_way_round():
    assert hfs.judge_probe({**GOOD, "trigger_point_measured": 0.2}) == []


@pytest.mark.parametrize("change,word", [
    ({"window_ms": 40.0}, "window"), ({"sample_hz": 1000.0}, "sample rate"),
    ({"counter_gaps": 3}, "gaps"), ({"trigger_point_measured": 0.5, "trigger_point_requested": 0.9}, "trigger point"),
    ({"angle_channels_finite": False}, "non-finite"), ({"vbus": 0.0}, "vbus"),
])
def test_probe_refuses_each_failed_check_by_name(change, word):
    reasons = hfs.judge_probe({**GOOD, **change})
    assert len(reasons) == 1 and word in reasons[0]


def test_the_probe_record_labels_working_offset_and_field_strength_as_not_health_constants():
    lines = hfs.identity_lines({"axis0.commutation_mapper.working_offset": -3.8,
                                hfs.FIELD_STRENGTH: 0.116, "vbus_voltage": 45.0})
    text = "\n".join(lines)
    assert "tracks the electrical position" in text and "not a health constant" in text
    assert "bit-identical" in text and "not an encoder-health reading" in text
    assert [ln for ln in lines if ln.startswith("vbus_voltage")][0].rstrip().endswith("45.0")


class FakeDrive:
    """The scope as the odrive package presents it, counting at 8 kHz. ``states``, ``reasons``
    and ``errors`` script the poll reads (the last value repeats)."""

    def __init__(self, size=800, trigger_lands=0.8):
        self.size, self.trigger_lands, self.calls, self.n, self.vels = size, trigger_lands, [], 1000, []
        self.states, self.reasons, self.errors = [8], [0], [0]

    @staticmethod
    def _next(seq):
        return seq.pop(0) if len(seq) > 1 else seq[0]

    def read(self, path):
        if path == "n_evt_sampling":
            self.n += 8000
            return self.n
        if path == hfs.VEL:
            return self.vels.pop(0) if self.vels else 0.0
        if path == hfs.STATE:
            return self._next(self.states)
        if path == hfs.DISARM:
            return self._next(self.reasons)
        if path == hfs.ERRORS:
            return self._next(self.errors)
        return 1.0

    def call_read_function(self, path):
        return 1.0

    def scope_config(self, endpoints):
        self.calls.append(("config", list(endpoints)))

    def scope_trigger(self, point):
        self.calls.append(("trigger", point))

    def scope_download(self, endpoints):
        """One shared buffer of 4-byte values, as odrive.utils.ram_osci_download unpacks it:
        more channels mean fewer samples. ``size`` is the sample count at 4 channels."""
        endpoints = list(endpoints)
        samples = self.size * 4 // len(endpoints)
        start = self.n - int(samples * self.trigger_lands)
        return [list(range(start, start + samples)) if e == "n_evt_sampling"
                else [48.0 for _ in range(samples)] for e in endpoints]


def test_scope_timing_measures_the_rate_window_and_trigger_point():
    now = [0.0]

    def clock():
        return now[0]

    def sleep(s):
        now[0] += s

    r = hfs.scope_timing(FakeDrive(), sleep=sleep, clock=clock)
    assert r["event_hz"] == pytest.approx(8000.0)
    assert r["decimation"] == 1.0 and r["counter_gaps"] == 0
    assert r["window_ms"] == pytest.approx(100.0)
    assert r["trigger_point_requested"] == 0.8
    assert r["trigger_point_measured"] == pytest.approx(0.8, abs=0.01)


def _no_wait(monkeypatch):
    monkeypatch.setattr(hfs.time, "sleep", lambda s: None)
    clock = {"t": 0.0}

    def fake_monotonic():
        clock["t"] += 1.0
        return clock["t"]

    monkeypatch.setattr(hfs.time, "monotonic", fake_monotonic)


def test_probe_judges_the_window_of_the_eight_channel_preset_not_the_timing_preset(monkeypatch, capsys, tmp_path):
    """800 samples at 4 channels is 100 ms and would pass; at 8 channels it is 50 ms and must not."""
    monkeypatch.setattr(hfs, "connect", lambda serial: FakeDrive(size=800))
    monkeypatch.setattr(hfs, "OUT_DIR", str(tmp_path))
    monkeypatch.setattr(hfs, "save_capture", lambda *a, **k: "unused")
    _no_wait(monkeypatch)
    rc = hfs.main(["probe"])
    out = capsys.readouterr().out
    assert rc == 2 and "NO-GO" in out and "window" in out
    assert "not a health constant" in out


def _capture_cli(monkeypatch, drive, argv):
    saved = []
    monkeypatch.setattr(hfs, "connect", lambda serial: drive)
    monkeypatch.setattr(hfs.time, "sleep", lambda s: None)
    monkeypatch.setattr(hfs, "save_capture", lambda c, m, l: saved.append((l, m)) or "unused")
    monkeypatch.setattr(hfs, "scope_timing", lambda d: {"sample_hz": 4000.0})
    monkeypatch.setattr(hfs, "motor_params", lambda i: PARAMS)
    return hfs.main(["capture", "--fill", "0"] + argv), saved


def test_capture_measures_its_own_sample_rate_and_tags_the_file(monkeypatch):
    drive = FakeDrive(size=1600)
    drive.vels = [0.0, 60.0, 70.0, 59.0]
    rc, saved = _capture_cli(monkeypatch, drive, [])
    assert rc == 0 and saved[0][1]["sample_hz"] == 4000.0
    assert saved[0][0] == "hand_0_brake" and saved[0][1]["capture"]["kind"] == "brake"
    assert ("trigger", 0.5) in drive.calls


def test_a_brake_timeout_is_saved_tagged_timeout_and_returns_3(monkeypatch):
    rc, saved = _capture_cli(monkeypatch, FakeDrive(), ["--timeout", "0.000001"])
    assert rc == 3 and saved[0][0] == "hand_0_timeout" and not saved[0][1]["capture"]["fired"]


def test_fault_mode_saves_nothing_on_a_timeout(monkeypatch):
    drive = FakeDrive()
    rc, saved = _capture_cli(monkeypatch, drive, ["--trigger", "fault", "--timeout", "0.000001"])
    assert rc == 3 and saved == [] and not [c for c in drive.calls if c[0] == "trigger"]


def test_fault_mode_clean_exits_do_not_use_up_the_count(monkeypatch):
    """CL, clean IDLE, CL, then a fault: one save, tagged _fault, at the default point 0.9."""
    drive = FakeDrive()
    drive.states = [8, 8, 1, 1, 8, 8, 1]
    drive.reasons = [0, 0, 0, 0, 0, 0, 0x4000000]
    rc, saved = _capture_cli(monkeypatch, drive, ["--trigger", "fault"])
    assert rc == 0 and [s[0] for s in saved] == ["hand_0_fault"]
    assert saved[0][1]["capture"]["clean_exits"] == 1
    assert ("trigger", 0.9) in drive.calls


def test_fault_mode_save_clean_saves_the_clean_exit_too(monkeypatch):
    drive = FakeDrive()
    drive.states = [8, 8, 1, 1, 8, 8, 1]
    drive.reasons = [0, 0, 0, 0, 0, 0, 0x4000000]
    rc, saved = _capture_cli(monkeypatch, drive, ["--trigger", "fault", "--save-clean"])
    assert rc == 0 and [s[0] for s in saved] == ["hand_0_clean", "hand_1_fault"]


def test_capture_freezes_the_scope_after_the_trigger_and_returns_named_columns():
    d = FakeDrive()
    d.vels = [0.0, 60.0, 70.0, 59.0]
    trig = hfs.BrakeTrigger("brake")
    cols, info = hfs.capture_once(d, "angle", trig, 0.5, timeout_s=5.0, fill_s=0.0, sleep=lambda s: None)
    assert info["fired"] and info["polls"] == 4 and "brake" in info["reason"] and info["kind"] == "brake"
    assert [c[0] for c in d.calls] == ["config", "trigger"]
    assert list(cols) == list(hfs.PRESETS["angle"])


def test_a_timed_out_capture_saves_the_window_and_says_it_did_not_fire():
    d = FakeDrive()
    cols, info = hfs.capture_once(d, "angle", hfs.BrakeTrigger("brake"), 0.5, timeout_s=0.01, fill_s=0.0,
                                  sleep=lambda s: None)
    assert not info["fired"] and info["reason"] is None and info["kind"] == "timeout"
    assert len(cols["vbus"]) == d.size * 4 // 8


# ---- bookkeeping in analyze --------------------------------------------------------------

def test_record_lines_never_let_a_timeout_pass_for_a_capture():
    lines, ok = hfs.record_lines({"capture": {"fired": False, "reason": None, "polls": 42732,
                                              "poll_ms": 1.404, "trigger_point": 0.5}})
    assert not ok and "TIMED OUT after 60.0 s" in "\n".join(lines) and "kind timeout" in lines[0]
    lines, ok = hfs.record_lines({"capture": {"fired": True, "kind": "fault", "reason": "fault (...)",
                                              "polls": 10, "poll_ms": 1.4, "trigger_point": 0.9}})
    assert ok and "kind fault" in lines[0] and "fired True" in lines[0]
    assert "no capture record" in hfs.record_lines({})[0][0]
    assert "re-judged: GO" in hfs.record_lines({"probe": GOOD})[0][0]


def _identity():
    return {"axis0.config.motor.phase_resistance": PARAMS["R"], "axis0.config.motor.phase_inductance": PARAMS["L"],
            "axis0.config.motor.torque_constant": PARAMS["kt"], "axis0.config.motor.pole_pairs": 7,
            **SPIN[0]}


def test_analyze_end_to_end_on_a_synthetic_bench_brake(tmp_path, capsys):
    """The bench Hand's shape: pos_rel opposite to the electrical frame, phase in turns."""
    path = hfs.save_capture(stroke(frame=-1.0), {"identity": _identity(), "sample_hz": FS, "preset": "angle",
                                                 "capture": {"fired": True, "kind": "brake", "reason": "brake",
                                                             "polls": 9, "poll_ms": 1.4, "trigger_point": 0.5}},
                            "t_0_brake", out_dir=str(tmp_path))
    assert hfs.main(["analyze", path]) == 0
    out = capsys.readouterr().out
    assert "fired True" in out and "OPPOSITE" in out and "jumps over 0.08 turn against phase_vel: 0" in out
    assert "none found" in out and "read from this drive" in out and "both at once" in out


def test_analyze_returns_3_on_a_timed_out_window(tmp_path, capsys):
    path = hfs.save_capture(synth(np.zeros(400), 0.1), {"identity": _identity(), "sample_hz": FS,
                                                        "capture": {"fired": False, "reason": None, "polls": 5,
                                                                    "poll_ms": 1.0, "trigger_point": 0.5}},
                            "t_0_timeout", out_dir=str(tmp_path))
    assert hfs.main(["analyze", path]) == 3 and "TIMED OUT" in capsys.readouterr().out


# ---- the scope ---------------------------------------------------------------------------

def test_every_preset_fits_the_scope_ring_so_its_recording_can_stop():
    """FW 0.6.11 (bench, 2026-11-11): 9 channels never stopped at any trigger point but 1.0."""
    for name, preset in hfs.PRESETS.items():
        hfs.scope_grid_point(0.5, 65536, len(preset))          # raises if the ring cannot hold it
    with pytest.raises(ValueError):
        hfs.scope_grid_point(0.5, 65536, 9)


def test_trigger_points_round_to_a_whole_sample():
    """0.8 on 4 channels left 13107.2 bytes and recorded for ever; 3277/4096 stopped in 106 ms."""
    q = hfs.scope_grid_point(0.8, 65536, 4)
    assert q == 3277 / 4096 and ((1.0 - q) * 65536) % 16 == 0
    assert hfs.scope_grid_point(0.5, 65536, 8) == 0.5
    assert hfs.scope_grid_point(1.0, 65536, 8) == 1.0 and hfs.scope_grid_point(0.0, 65536, 8) == 0.0


class _Scope:
    def __init__(self, stops_after):
        self.size, self.reads, self.stops_after, self.triggers = 65536, 0, stops_after, []

    @property
    def recording(self):
        self.reads += 1
        return self.stops_after is None or self.reads <= self.stops_after


class _Utils:
    def __init__(self, scope):
        self.scope = scope

    def ram_osci_config(self, odrv, endpoints):
        pass

    def ram_osci_trigger(self, odrv, point):
        self.scope.triggers.append(point)

    def ram_osci_download(self, odrv, endpoints):
        return [[0.0] for _ in endpoints]


class _Odrv:
    def __init__(self, scope):
        self.oscilloscope = scope


def test_the_download_wait_is_bounded_and_stops_a_stuck_scope():
    scope = _Scope(stops_after=None)
    drive = hfs.Drive(_Odrv(scope), _Utils(scope))
    t = [0.0]
    with pytest.raises(RuntimeError, match="still recording"):
        drive.scope_download(["vbus_voltage"], timeout_s=1.0, clock=lambda: t[0],
                             sleep=lambda s: t.__setitem__(0, t[0] + s))
    assert scope.triggers == [1.0]


def test_the_download_waits_for_a_scope_that_stops():
    scope = _Scope(stops_after=3)
    drive = hfs.Drive(_Odrv(scope), _Utils(scope))
    assert drive.scope_download(["vbus_voltage"], sleep=lambda s: None) == [[0.0]]
    assert scope.triggers == []


def test_capture_round_trips_through_the_files(tmp_path):
    cols = synth(np.full(100, 100.0), 10.0)
    meta = {"identity": {"a": 1}, "sample_hz": FS, "preset": "angle"}
    path = hfs.save_capture(cols, meta, "t", out_dir=str(tmp_path))
    back, m = hfs.load_capture(path)
    assert m["sample_hz"] == FS and list(back) == list(cols)
    assert np.allclose(back["iq"], cols["iq"]) and np.allclose(back["phase"], cols["phase"])


# ---- contracts ---------------------------------------------------------------------------

def _source():
    with open(hfs.__file__) as f:
        return f.read()


def test_the_tool_never_writes_to_the_drive_except_the_scope():
    """Read-only for the motor. Only the odrive scope utilities are called, and nothing assigns
    to a drive attribute or names a state, gain, limit or persistence command."""
    tree = ast.parse(_source())
    utils = {n.attr for n in ast.walk(tree)
             if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Attribute) and n.value.attr == "utils"}
    assert utils == {"ram_osci_config", "ram_osci_trigger", "ram_osci_download"}
    assert [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "attr", "") == "setattr"] == []
    assert [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "id", "") == "setattr"] == []
    # Drive.read and call_read_function resolve any path, so pin what reaches the function call.
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and getattr(n.func, "attr", "") == "call_read_function"]
    assert len(calls) >= 1 and all(getattr(c.args[0], "id", "") == "FIELD_STRENGTH" for c in calls)
    assert hfs.FIELD_STRENGTH == "onboard_encoder0.get_field_strength"
    code = _source().split('"""', 2)[2]
    for word in ("requested_state", "input_pos", "input_vel", "input_torque", "save_configuration",
                 "erase_configuration", "clear_errors", "reboot", "calibrat", ".config.motor.current_soft_max ="):
        assert word not in code


def test_every_endpoint_exists_on_the_hands_drive_and_is_readable():
    with open(PRO_TABLE) as f:
        table = json.load(f)
    table = table.get("endpoints", table)
    used = set(hfs.IDENTITY_READS) | {hfs.VEL, hfs.STATE, hfs.DISARM, hfs.ERRORS}
    for preset in hfs.PRESETS.values():
        assert len(preset) <= 8
        used |= set(preset.values())
    for path in sorted(used):
        assert path in table, path
        assert "r" in table[path]["access"], path
    assert table[hfs.FIELD_STRENGTH]["type"] == "function"
    for fn in ("oscilloscope.config", "oscilloscope.trigger", "oscilloscope.get_raw"):
        assert table[fn]["type"] == "function"


def test_the_spinout_reads_are_the_fw_0_6_11_controller_config_paths():
    assert set(hfs.SPINOUT_READS) <= set(hfs.IDENTITY_READS)
    assert all(p.startswith("axis0.controller.config.spinout_") for p in hfs.SPINOUT_READS)


def test_the_installed_odrive_package_has_the_calls_this_tool_makes():
    pytest.importorskip("odrive")
    import inspect

    import odrive
    import odrive.utils
    assert "serial_number" in inspect.signature(odrive.find_sync).parameters
    assert "timeout" in inspect.signature(odrive.find_sync).parameters
    assert list(inspect.signature(odrive.utils.ram_osci_config).parameters) == ["odrv", "expressions"]
    assert list(inspect.signature(odrive.utils.ram_osci_trigger).parameters) == ["odrv", "trigger_point"]
    assert list(inspect.signature(odrive.utils.ram_osci_download).parameters) == ["odrv", "expressions"]


def test_a_fault_latched_while_the_scope_fills_still_fires():
    # The baseline is taken before the fill wait, so the first poll sees the edge.
    t = hfs.BrakeTrigger("fault")
    t.seed(0, 0)
    assert t.update(None, 1, 0x200, 0x200) and t.kind == "fault"
    late = hfs.BrakeTrigger("fault")          # unseeded: a latched value is the baseline
    assert not late.update(None, 1, 0x200, 0x200)
