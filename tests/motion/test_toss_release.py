"""Toss release-state tests (single-ball-toss Phase 1) — the §7 worked-example pins.

Pins the STOW→global conversion (the 2026-07-23 first-sitting z-double-add
regression), the release/catch plane offsets, the FULL release-plane→catch-plane
launch solution against its idealised g·T/2 limit, the announced-landing fields,
and the 9806-vs-9810 gravity-source guard — a drifted constant or a re-added
active-z lift anywhere in the toss math fails loudly here.

Worked constants: initial_height = 574.3, HAND_THROW_OFFSET_MM = 58.044
(= −129.0 + 187.044), HAND_CATCH_OFFSET_MM = 64.78, Δz = 6.736 mm,
g = 9806.0 mm/s².
"""

from __future__ import annotations

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.trajectory import ballistics_bc as bb
from jugglebot.motion.trajectory import toss_release as tr
from jugglebot.reload_sequencer import compute_catch_point_mm

G = bb.GRAVITY_MMS2


# ── STOW→global conversion (the ONE conversion point) ──

def test_stow_to_global_active_center():
    """ACTIVE center: (0, 0, 170) → (0, 0, 744.3) — one addition of 574.3 to z."""
    assert np.allclose(tr.stow_to_global_mm((0.0, 0.0, 170.0)),
                       (0.0, 0.0, 744.3), atol=1e-9)


@pytest.mark.parametrize('stow, want', [
    ((25.0, -40.0, 140.0), (25.0, -40.0, 714.3)),
    ((60.0, -60.0, 140.0), (60.0, -60.0, 714.3)),
])
def test_stow_to_global_general_xyz(stow, want):
    """General (x, y, z): x/y pass through untouched, z gains exactly 574.3."""
    assert np.allclose(tr.stow_to_global_mm(stow), want, atol=1e-9)


def test_conversion_parity_with_reload_catch_point():
    """The new general conversion + cup offset reproduces the hardware-verified
    reload catch point (reload_sequencer.compute_catch_point_mm) at the ACTIVE
    center — pins both frames' values for one worked example (809.08 mm), the
    plan's mandated regression against the z double-add."""
    cup = tr.stow_to_global_mm((0.0, 0.0, 170.0)) + np.array([0.0, 0.0, 64.78])
    ref = compute_catch_point_mm(574.3, 170.0, landing_z_offset_mm=64.78)
    assert np.allclose(cup, ref, atol=1e-9)
    assert cup[2] == pytest.approx(809.08, abs=1e-9)


def test_hand_throw_offset_pin():
    """HAND_THROW_OFFSET_MM = 58.044, derived from the two generated inputs
    (−129.0 + 0.187044·1000) — a hardware_config regeneration that moves either
    input shifts the release plane and must be re-examined."""
    assert tr.HAND_THROW_OFFSET_MM == pytest.approx(58.044, abs=1e-9)
    assert tr.HAND_THROW_OFFSET_MM == pytest.approx(
        hw.GEOM_HAND_AXIS_BOTTOM_OFFSET_MM + hw.HAND_THROW_POS_M * 1000.0,
        abs=1e-12)


# ── release-state worked examples (full solution) ──

def test_release_state_worked_example_center():
    """Plan sanity example: (0, 0, 170) at T = 0.8 s — release plane 802.344 mm,
    launch (0, 0, 3930.82) mm/s (FULL solution), event_vel 3.931 m/s."""
    rs = tr.compute_release_state((0.0, 0.0, 170.0), 0.8)
    assert np.allclose(rs.release_pos_global_mm, (0.0, 0.0, 802.344), atol=1e-9)
    assert np.allclose(rs.catch_point_global_mm, (0.0, 0.0, 809.08), atol=1e-9)
    assert rs.launch_vel_mms[0] == 0.0 and rs.launch_vel_mms[1] == 0.0
    assert rs.launch_vel_mms[2] == pytest.approx(3930.82, abs=1e-6)
    assert rs.event_vel_mps == pytest.approx(3.93082, abs=1e-6)
    assert rs.flight_time_s == 0.8


def test_full_vs_idealised_launch_identity():
    """The production function returns the FULL release-plane→catch-plane
    solution vz = Δz/T + g·T/2; the plan's idealised v = g·T/2 (3922.4 mm/s at
    T = 0.8) is its Δz→0 limit, offset by exactly Δz/T = 8.42 mm/s. Apex above
    release: idealised g·T²/8 = 784.48 mm, full vz²/2g ≈ 787.85 mm (the plan's
    "≈0.78 m")."""
    T = 0.8
    dz = 64.78 - tr.HAND_THROW_OFFSET_MM          # 6.736 mm, cup above release
    vz = tr.compute_release_state((0.0, 0.0, 170.0), T).launch_vel_mms[2]
    idealised = G * T / 2.0
    assert idealised == pytest.approx(3922.4, abs=1e-9)
    assert vz == pytest.approx(idealised + dz / T, abs=1e-9)
    assert vz - idealised == pytest.approx(8.42, abs=1e-3)
    assert G * T * T / 8.0 == pytest.approx(784.48, abs=1e-9)
    assert vz * vz / (2.0 * G) == pytest.approx(787.85, abs=0.01)


def test_release_state_offcenter_colocated():
    """Off-center (60, −60, 170) at T = 0.6 s: x/y ride through to both planes,
    launch is purely vertical (co-located throw/catch ⇒ zero horizontal)."""
    rs = tr.compute_release_state((60.0, -60.0, 170.0), 0.6)
    assert np.allclose(rs.release_pos_global_mm, (60.0, -60.0, 802.344), atol=1e-9)
    assert np.allclose(rs.catch_point_global_mm, (60.0, -60.0, 809.08), atol=1e-9)
    assert rs.launch_vel_mms[0] == 0.0 and rs.launch_vel_mms[1] == 0.0
    assert rs.launch_vel_mms[2] == pytest.approx(2953.03, abs=0.01)
    assert rs.event_vel_mps == pytest.approx(2.953, abs=1e-3)


@pytest.mark.parametrize('xy', [(0.0, 0.0), (60.0, 60.0), (60.0, -60.0),
                                (-60.0, 60.0), (-60.0, -60.0)])
def test_sweep_edge_speeds_within_teensy_bounds(xy):
    """The plan's sweep envelope (flight 0.55–1.10 s, centre + 4 corners at
    ±60 mm) stays inside the bridge's [0.3, 7.0] m/s event_vel acceptance:
    2.709 / 5.399 m/s at the edges."""
    lo = tr.compute_release_state((xy[0], xy[1], 170.0), 0.55)
    hi = tr.compute_release_state((xy[0], xy[1], 170.0), 1.10)
    assert lo.event_vel_mps == pytest.approx(2.709, abs=1e-3)
    assert hi.event_vel_mps == pytest.approx(5.399, abs=1e-3)
    assert tr.validate_event_vel(lo.event_vel_mps)
    assert tr.validate_event_vel(hi.event_vel_mps)


def test_ballistic_roundtrip_lands_in_cup():
    """Integrating the released ball for T lands it exactly on the nominated
    cup point — the launch solution and the flight model agree."""
    rs = tr.compute_release_state((0.0, 0.0, 170.0), 0.8)
    landed = bb.position_at(rs.release_pos_global_mm, rs.launch_vel_mms, 0.8)
    assert np.allclose(landed, rs.catch_point_global_mm, atol=1e-9)


# ── announced-landing construction ──

def test_announcement_fields_worked_examples():
    """Landing velocity is the ballistic arrival: (0, 0, −3913.98) mm/s for
    example A (3930.82 − 9806·0.8) and (0, 0, −2930.57) for example B."""
    rs_a = tr.compute_release_state((0.0, 0.0, 170.0), 0.8)
    f_a = tr.build_announcement_fields(rs_a, 10.0)
    assert np.allclose(f_a['initial_position'], rs_a.release_pos_global_mm)
    assert np.allclose(f_a['initial_velocity'], rs_a.launch_vel_mms)
    assert np.allclose(f_a['landing_position'], (0.0, 0.0, 809.08), atol=1e-9)
    assert f_a['landing_velocity'][2] == pytest.approx(-3913.98, abs=1e-6)
    rs_b = tr.compute_release_state((60.0, -60.0, 170.0), 0.6)
    f_b = tr.build_announcement_fields(rs_b, 10.0)
    assert f_b['landing_velocity'][2] == pytest.approx(-2930.57, abs=0.01)


def test_announcement_landing_time_semantics():
    """landing_time_s = throw_time_s (absolute release time) + flight time; the
    predicted TOF rides through unchanged."""
    rs = tr.compute_release_state((0.0, 0.0, 170.0), 0.8)
    fields = tr.build_announcement_fields(rs, 100.0)
    assert fields['landing_time_s'] == pytest.approx(100.8, abs=1e-9)
    assert fields['predicted_tof_sec'] == pytest.approx(0.8, abs=1e-12)


# ── gravity-source guard (the locked decision, enforced) ──

def test_gravity_constant_parity():
    """One gravity source: the motion-side copy matches controller/ballistics
    (9806.0) and is never the tracker-side 9810.0 — the two constants must not
    mix in one computation (kept as separate assertions deliberately)."""
    import controller.ballistics as cb
    assert bb.GRAVITY_MMS2 == 9806.0
    assert bb.GRAVITY_MMS2 == cb.GRAVITY_MMS2
    assert bb.GRAVITY_MMS2 != 9810.0


# ── error paths ──

def test_error_paths():
    """Non-positive flight time raises; event_vel bounds are inclusive-accept,
    mirroring the bridge's validation."""
    with pytest.raises(ValueError):
        tr.compute_release_state((0.0, 0.0, 170.0), 0.0)
    with pytest.raises(ValueError):
        tr.compute_release_state((0.0, 0.0, 170.0), -0.5)
    assert tr.validate_event_vel(0.2) is False
    assert tr.validate_event_vel(7.1) is False
    assert tr.validate_event_vel(0.3) is True
    assert tr.validate_event_vel(7.0) is True
