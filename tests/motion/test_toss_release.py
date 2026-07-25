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
from jugglebot.motion.trajectory import tilt_geometry as tg
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


# ── Tier-8b tilted displaced release (single-ball-toss Phase 4) ──

def test_tilted_release_worked_example_displaced():
    """Phase-4 spec worked example: B = (100, 0, 170), A = (0, 0), T = 0.8 s.
    Level inverse gives v_x = d/T = 125 mm/s exactly; the tilted release sits
    at nominal A xy (swing-compensated) and arm·(1−cos θ) ≈ 0.029 mm BELOW the
    802.344 level plane, so v_z rides 0.037 mm/s above the 8a 3930.82. Aim
    θ ≈ 1.82° from vertical, +x displacement ⇒ ry > 0, rx == 0."""
    rs = tr.compute_release_state_tilted((100.0, 0.0, 170.0), 0.8,
                                         throw_site_xy_mm=(0.0, 0.0))
    assert np.allclose(rs.release_pos_global_mm, (0.0, 0.0, 802.31467),
                       atol=1e-4)
    assert rs.launch_vel_mms[0] == pytest.approx(125.0, abs=1e-9)
    assert rs.launch_vel_mms[1] == 0.0
    assert rs.launch_vel_mms[2] == pytest.approx(3930.857, abs=1e-3)
    assert rs.event_vel_mps == pytest.approx(3.93284, abs=1e-5)
    assert np.allclose(rs.catch_point_global_mm, (100.0, 0.0, 809.08),
                       atol=1e-9)
    assert rs.tilt_ry > 0.0 and rs.tilt_rx == 0.0
    assert np.degrees(np.hypot(rs.tilt_rx, rs.tilt_ry)) == pytest.approx(
        1.8214, abs=1e-3)
    assert rs.displacement_mm == pytest.approx(100.0, abs=1e-12)
    assert rs.flight_time_s == 0.8


def test_tilted_pretilt_pose_swing_compensation_magnitude():
    """The commanded centroid pulls back by arm·sin θ = 58.044·sin 1.8214° ≈
    1.845 mm along −x, so the TILTED release point lands exactly AT A = (0, 0)
    — the throw-side lever-arm compensation pin (release-plane arm ≈ +58 mm
    against the fixed 744.3 mm world tilt centre, the catch-side convention)."""
    rs = tr.compute_release_state_tilted((100.0, 0.0, 170.0), 0.8,
                                         throw_site_xy_mm=(0.0, 0.0))
    pose = rs.pretilt_pose_stow
    assert pose.shape == (6,)
    assert pose[0] == pytest.approx(-1.8448, abs=1e-3)
    assert pose[1] == 0.0 and pose[2] == 170.0
    assert pose[3] == rs.tilt_rx and pose[4] == rs.tilt_ry and pose[5] == 0.0
    # The compensation's whole point: the release xy sits AT nominal A.
    assert rs.release_pos_global_mm[0] == 0.0
    assert rs.release_pos_global_mm[1] == 0.0


def test_tilted_aim_parallel_and_roundtrip_lands_at_B():
    """cup_axis(tilt) is PARALLEL to the actual launch (the load-bearing
    Tier-8b invariant — the slider stroke ejects along the tilted axis at the
    aim) and integrating the released ball for T lands it exactly on B's cup
    point: the returned state is self-consistent by construction."""
    rs = tr.compute_release_state_tilted((70.0, -50.0, 170.0), 0.8,
                                         throw_site_xy_mm=(0.0, 0.0))
    axis = tg.cup_axis(rs.tilt_rx, rs.tilt_ry)
    unit = rs.launch_vel_mms / np.linalg.norm(rs.launch_vel_mms)
    assert np.dot(axis, unit) > 1.0 - 1e-9
    landed = bb.position_at(rs.release_pos_global_mm, rs.launch_vel_mms, 0.8)
    assert np.allclose(landed, rs.catch_point_global_mm, atol=1e-9)


def test_tilted_fixed_point_converges_in_one_pass():
    """Phase-4 spec pin: the tilt→release→tilt fixed point converges in ONE
    pass (<0.1 mm / <1 mm/s — measured ~5e-7 mm / ~7e-7 mm/s: the vertical-drop
    correction is ~0.1 %). Re-derives pass 1 by hand from the public pieces and
    compares to the returned (pass-2) state."""
    T = 0.8
    rs = tr.compute_release_state_tilted((100.0, 0.0, 170.0), T,
                                         throw_site_xy_mm=(0.0, 0.0))
    rel_level = tr.stow_to_global_mm((0.0, 0.0, 170.0)) + np.array(
        [0.0, 0.0, tr.HAND_THROW_OFFSET_MM])
    v0 = bb.launch_velocity(rel_level, rs.catch_point_global_mm, T)
    rx0, ry0 = tg.tilt_to_throw(v0)
    rel1 = tr._tilted_release_pos((0.0, 0.0), float(rel_level[2]), rx0, ry0)
    v1 = bb.launch_velocity(rel1, rs.catch_point_global_mm, T)
    assert np.abs(rs.release_pos_global_mm - rel1).max() < 0.1
    assert np.abs(rs.launch_vel_mms - v1).max() < 1.0


def test_tilted_clamp_gate_rejects_loudly():
    """A displacement needing a >12° from-vertical aim raises
    ThrowTiltInfeasible — NEVER a silently clamped, mis-aimed throw (the
    Rung-2a landing bias). (400, 0) @ T = 0.55 needs ≈15.03°. A ValueError
    subclass (generic error paths still catch it), carrying the diagnostics
    the sequencer's REJECTED_TILT_CLAMP reject reports. Inside the Phase-4
    envelope the gate never binds: even a 300 mm displacement at the sweep
    floor needs only ≈11.4°."""
    with pytest.raises(tr.ThrowTiltInfeasible) as exc:
        tr.compute_release_state_tilted((400.0, 0.0, 170.0), 0.55,
                                        throw_site_xy_mm=(0.0, 0.0))
    assert isinstance(exc.value, ValueError)
    assert exc.value.required_deg == pytest.approx(15.03, abs=0.01)
    assert exc.value.max_tilt_deg == 12.0
    # Never-binds-in-envelope evidence: 300 mm @ 0.55 s stays under the clamp.
    rs = tr.compute_release_state_tilted((150.0, 0.0, 170.0), 0.55,
                                         throw_site_xy_mm=(-150.0, 0.0))
    assert np.degrees(np.hypot(rs.tilt_rx, rs.tilt_ry)) == pytest.approx(
        11.376, abs=1e-3)


def test_tilted_degenerates_to_8a_bitwise_when_colocated():
    """throw_site == B_xy ⇒ BITWISE-equal to compute_release_state (tilt
    exactly level, zero shift and drop, same construction order) — the Tier-8a
    regression net for the shared math."""
    a = tr.compute_release_state((60.0, -60.0, 170.0), 0.6)
    b = tr.compute_release_state_tilted((60.0, -60.0, 170.0), 0.6,
                                        throw_site_xy_mm=(60.0, -60.0))
    assert np.array_equal(a.release_pos_global_mm, b.release_pos_global_mm)
    assert np.array_equal(a.launch_vel_mms, b.launch_vel_mms)
    assert a.event_vel_mps == b.event_vel_mps
    assert np.array_equal(a.catch_point_global_mm, b.catch_point_global_mm)
    assert a.flight_time_s == b.flight_time_s
    assert (b.tilt_rx, b.tilt_ry) == (0.0, 0.0)
    assert np.array_equal(b.pretilt_pose_stow,
                          np.array([60.0, -60.0, 170.0, 0.0, 0.0, 0.0]))
    assert b.displacement_mm == 0.0


def test_tilted_announcement_landing_is_B():
    """The Tier-8b announcement's landing IS B's cup point (global) and the
    initial/landing velocities carry the conserved lateral component — what
    the correlation → catch loop keys on at the displaced site.
    build_announcement_fields itself is unchanged (inheritance rides through)."""
    rs = tr.compute_release_state_tilted((100.0, 0.0, 170.0), 0.8,
                                         throw_site_xy_mm=(0.0, 0.0))
    f = tr.build_announcement_fields(rs, 100.0)
    assert np.allclose(f['landing_position'], (100.0, 0.0, 809.08), atol=1e-9)
    assert np.allclose(f['initial_position'], rs.release_pos_global_mm)
    assert f['initial_velocity'][0] == pytest.approx(125.0, abs=1e-9)
    assert f['landing_velocity'][0] == pytest.approx(125.0, abs=1e-9)
    assert f['landing_time_s'] == pytest.approx(100.8, abs=1e-9)
    assert f['predicted_tof_sec'] == pytest.approx(0.8, abs=1e-12)


def test_tilted_default_throw_site_is_config_centre():
    """throw_site_xy_mm=None resolves hw.JB_OP_TOSS_THROW_SITE_MM — shipped
    [0, 0], the workspace centre (the Rung-2a evidence base's throw column,
    making displacement = |B_xy|)."""
    assert list(hw.JB_OP_TOSS_THROW_SITE_MM) == [0.0, 0.0]
    rs = tr.compute_release_state_tilted((50.0, 0.0, 170.0), 0.8)
    assert rs.displacement_mm == pytest.approx(50.0, abs=1e-12)


def test_tilted_error_paths():
    """Non-positive flight time raises (propagated from ballistics_bc), same
    as the 8a path."""
    with pytest.raises(ValueError):
        tr.compute_release_state_tilted((50.0, 0.0, 170.0), 0.0,
                                        throw_site_xy_mm=(0.0, 0.0))
    with pytest.raises(ValueError):
        tr.compute_release_state_tilted((50.0, 0.0, 170.0), -0.5,
                                        throw_site_xy_mm=(0.0, 0.0))


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
