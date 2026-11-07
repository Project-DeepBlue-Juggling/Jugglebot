"""Offline pins for tools/probes/hand_foc_scope.py (JB-147): the angle recovery, the
trigger, the probe verdict, the read-only contract and the endpoints it relies on."""
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


def synth(vel_rps, iq, delta_deg=0.0, vbus=48.0):
    """Drive the voltage equations forwards, with a commutation error of delta_deg."""
    vel = np.asarray(vel_rps, dtype=float)
    iq = np.broadcast_to(np.asarray(iq, dtype=float), vel.shape).copy()
    w = vel * 2.0 * np.pi * PARAMS["pole_pairs"]
    lam = PARAMS["kt"] / (1.5 * PARAMS["pole_pairs"])
    d = np.radians(delta_deg)
    e_d, e_q = -w * lam * np.sin(d), w * lam * np.cos(d)
    i_d = np.zeros_like(iq)
    vd = e_d + PARAMS["R"] * i_d - w * PARAMS["L"] * iq
    vq = e_q + PARAMS["R"] * iq + w * PARAMS["L"] * i_d
    scale = (2.0 / 3.0) * vbus
    pos = np.cumsum(vel) / FS
    return {"mod_d": vd / scale, "mod_q": vq / scale, "vbus": np.full_like(vel, vbus),
            "id": i_d, "iq": iq, "iq_set": iq.copy(), "phase_vel": w,
            "phase": hfs.wrap_pi(np.cumsum(w) / FS), "pos": pos}


@pytest.mark.parametrize("delta", [0.0, 12.0, 30.0, -45.0])
@pytest.mark.parametrize("sign", [1.0, -1.0])
def test_angle_error_is_recovered_in_both_directions(delta, sign):
    _, s = hfs.analyse(synth(np.full(800, sign * 100.0), 30.0, delta), PARAMS, FS)
    assert s["delta_by_speed"][0]["delta_median"] == pytest.approx(delta, abs=0.5)


def test_correct_commutation_fits_the_flux_linkage_from_kt():
    _, s = hfs.analyse(synth(np.linspace(40, 130, 800), 25.0), PARAMS, FS)
    assert s["lambda_fit"] == pytest.approx(s["lambda_from_kt"], rel=0.01)


def test_samples_with_too_little_back_emf_give_no_angle():
    out, s = hfs.analyse(synth(np.full(800, 1.0), 0.0, 30.0), PARAMS, FS)
    assert not out["valid"].any() and s["delta_by_speed"] == []


def test_an_encoder_jump_is_counted_and_a_clean_phase_is_not():
    c = synth(np.full(800, 100.0), 10.0)
    assert hfs.analyse(c, PARAMS, FS)[1]["phase_jumps"] == 0
    c["phase"] = hfs.wrap_pi(c["phase"] + np.where(np.arange(800) >= 400, 1.2, 0.0))
    s = hfs.analyse(c, PARAMS, FS)[1]
    assert s["phase_jumps"] == 1 and s["phase_jump_first_ms"] == pytest.approx(399 / 8.0)


def test_a_torque_loss_is_found_and_a_clean_brake_has_none():
    """-30 A braking at 2600 rev/s^2, then the deceleration collapses with iq unchanged."""
    n = 800
    accel = np.full(n, -2600.0)
    assert hfs.analyse(synth(137.0 + np.cumsum(accel) / FS, -30.0), PARAMS, FS)[1]["torque_loss"] == []
    accel[500:600] = -100.0
    s = hfs.analyse(synth(137.0 + np.cumsum(accel) / FS, -30.0), PARAMS, FS)[1]
    assert s["accel_per_amp"] == pytest.approx(2600.0 / 30.0, rel=0.05)
    assert len(s["torque_loss"]) == 1
    loss = s["torque_loss"][0]
    assert loss["start_ms"] == pytest.approx(500 / 8.0, abs=3.0)
    assert loss["ms"] == pytest.approx(100 / 8.0, abs=4.0)
    assert loss["iq"] == pytest.approx(-30.0)


def test_holding_current_at_rest_after_a_brake_is_not_a_torque_loss():
    """After the stop the position loop can hold over 10 A with no motion."""
    vel = np.r_[137.0 - 2600.0 * np.arange(400) / FS, np.zeros(200)]
    vel[400:] = 0.0
    iq = np.r_[np.full(400, -30.0), np.full(200, -12.0)]
    assert hfs.analyse(synth(vel, iq), PARAMS, FS)[1]["torque_loss"] == []


def test_power_preset_confirms_the_scaling_against_the_drives_figure():
    c = synth(np.full(800, 100.0), 30.0)
    vd, vq = c["mod_d"] * 32.0, c["mod_q"] * 32.0
    c["p_elec"] = 1.5 * (vd * c["id"] + vq * c["iq"])
    assert hfs.analyse(c, PARAMS, FS)[1]["p_elec_calc_over_drive"] == pytest.approx(1.0)
    c["p_elec"] = c["p_elec"] * 1.5
    assert hfs.analyse(c, PARAMS, FS)[1]["p_elec_calc_over_drive"] == pytest.approx(1.0 / 1.5)


def test_summary_names_a_torque_loss_and_every_populated_speed_bin():
    accel = np.full(800, -2600.0)
    accel[500:600] = -100.0
    _, s = hfs.analyse(synth(137.0 + np.cumsum(accel) / FS, -30.0), PARAMS, FS)
    text = "\n".join(hfs.summary_lines(s))
    assert "TORQUE LOSS" in text and "80-110" in text and "110-140" in text


def test_brake_trigger_arms_on_speed_and_fires_on_the_fall():
    t = hfs.BrakeTrigger("brake", arm_vel=50.0, drop=0.15)
    assert not any(t.update(v, 8, 0) for v in (0.0, 30.0, 49.0, 40.0))
    assert not any(t.update(v, 8, 0) for v in (60.0, 70.0, 65.0, 60.0))
    assert t.update(59.0, 8, 0) and "peak 70.0" in t.reason


def test_brake_trigger_ignores_a_downward_stroke():
    t = hfs.BrakeTrigger("brake")
    assert not any(t.update(v, 8, 0) for v in (0.0, -60.0, -100.0, -60.0, 0.0))


def test_fault_trigger_waits_for_the_disarm_and_never_fires_on_a_brake():
    t = hfs.BrakeTrigger("fault")
    assert not any(t.update(v, 8, 0) for v in (137.0, 60.0, 0.0))
    assert t.update(50.0, 1, 0x4000000) and "0x4000000" in t.reason


def test_trigger_needs_closed_loop_first_so_an_idle_drive_does_not_fire():
    t = hfs.BrakeTrigger("fault")
    assert not t.update(0.0, 1, 0)
    t = hfs.BrakeTrigger("brake")
    assert t.update(0.0, 8, 0x4000000) and "disarm" in t.reason


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


class FakeDrive:
    """The scope as the odrive package presents it, counting at 8 kHz."""

    def __init__(self, size=800, trigger_lands=0.8):
        self.size, self.trigger_lands, self.calls, self.n, self.vels = size, trigger_lands, [], 1000, []

    def read(self, path):
        if path == "n_evt_sampling":
            self.n += 8000
            return self.n
        if path == hfs.VEL:
            return self.vels.pop(0) if self.vels else 0.0
        if path == hfs.STATE:
            return 8
        if path == hfs.DISARM:
            return 0
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


def test_probe_judges_the_window_of_the_nine_channel_preset_not_the_timing_preset(monkeypatch, capsys, tmp_path):
    """800 samples at 4 channels is 100 ms and would pass; at 9 channels it is 44 ms and must not."""
    monkeypatch.setattr(hfs, "connect", lambda serial: FakeDrive(size=800))
    monkeypatch.setattr(hfs, "OUT_DIR", str(tmp_path))
    monkeypatch.setattr(hfs.time, "sleep", lambda s: None)
    monkeypatch.setattr(hfs, "save_capture", lambda *a, **k: "unused")
    clock = {"t": 0.0}

    def fake_monotonic():
        clock["t"] += 1.0
        return clock["t"]

    monkeypatch.setattr(hfs.time, "monotonic", fake_monotonic)
    rc = hfs.main(["probe"])
    out = capsys.readouterr().out
    assert rc == 2 and "NO-GO" in out and "window" in out


def test_capture_measures_its_own_sample_rate_and_stores_it(monkeypatch, tmp_path):
    drive = FakeDrive(size=1600)
    drive.vels = [0.0, 60.0, 70.0, 59.0]
    saved = {}
    monkeypatch.setattr(hfs, "connect", lambda serial: drive)
    monkeypatch.setattr(hfs.time, "sleep", lambda s: None)
    monkeypatch.setattr(hfs, "save_capture", lambda c, m, l: saved.update(m) or "unused")
    monkeypatch.setattr(hfs, "scope_timing", lambda d: {"sample_hz": 4000.0})
    monkeypatch.setattr(hfs, "motor_params", lambda i: PARAMS)
    assert hfs.main(["capture", "--fill", "0"]) == 0
    assert saved["sample_hz"] == 4000.0


def test_capture_freezes_the_scope_after_the_trigger_and_returns_named_columns():
    d = FakeDrive()
    d.vels = [0.0, 60.0, 70.0, 59.0]
    trig = hfs.BrakeTrigger("brake")
    cols, info = hfs.capture_once(d, "angle", trig, 0.5, timeout_s=5.0, fill_s=0.0, sleep=lambda s: None)
    assert info["fired"] and info["polls"] == 4 and "brake" in info["reason"]
    assert [c[0] for c in d.calls] == ["config", "trigger"]
    assert list(cols) == list(hfs.PRESETS["angle"])


def test_a_timed_out_capture_saves_the_window_and_says_it_did_not_fire():
    d = FakeDrive()
    cols, info = hfs.capture_once(d, "angle", hfs.BrakeTrigger("brake"), 0.5, timeout_s=0.01, fill_s=0.0,
                                  sleep=lambda s: None)
    assert not info["fired"] and info["reason"] is None and len(cols["vbus"]) == d.size * 4 // 9


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
    """Read-only for the motor. Only two odrive utilities are called, and nothing assigns to a
    drive attribute or names a state, gain, limit or persistence command."""
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
    used = set(hfs.IDENTITY_READS) | {hfs.VEL, hfs.STATE, hfs.DISARM}
    for preset in hfs.PRESETS.values():
        assert len(preset) <= 9
        used |= set(preset.values())
    for path in sorted(used):
        assert path in table, path
        assert "r" in table[path]["access"], path
    assert table[hfs.FIELD_STRENGTH]["type"] == "function"
    for fn in ("oscilloscope.config", "oscilloscope.trigger", "oscilloscope.get_raw"):
        assert table[fn]["type"] == "function"


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
