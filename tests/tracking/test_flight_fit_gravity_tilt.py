"""FEAT-27: the Fit with gravity's horizontal part in the Mocap world frame.

The Mocap world frame is tilted 0.55 deg from gravity (measured 2026-11-26
from 1,139 free flights, `temp/probes/feat27-gravity/`: (+43, +83) mm/s^2
horizontal). A Fit that assumes gravity along -z fits a straight line to a
horizontal parabola and lands off; `FlightFit(gravity_xy_mmps2=...)` and
`ball_tracking.gravity_world_xy_mmps2` remove that. Three claims pinned here:

1. with the vector set, a synthetic flight under tilted gravity lands within
   0.5 mm (noise-free: the model is then exact);
2. with (0, 0) the landing is biased by exactly the amount least squares
   predicts (a line fitted to 0.5 g t^2 over the admitted samples,
   extrapolated to the crossing), several mm for a Hand-Throw-like flight;
3. (0, 0) -- the config default -- is the pre-FEAT-27 fit and landing BIT
   FOR BIT, against a frozen copy of the old arithmetic on noisy flight data
   with in-cup samples and outliers.
Plus the wiring: config default, `BallTracker` -> every FlightFit, the live
setter, and the Kalman fallback's landing.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from jugglebot.tracking.ballistics import GRAVITY_MMPS2, predict_landing_state
from jugglebot.tracking.flight_fit import FlightFit
from jugglebot.tracking.matcher import BallTracker

G_XY = (43.0, 83.0)          # the measured value, mm/s^2
PLANE = 830.0
FREEZE = 250.0
DT = 1.0 / 300.0             # mocap_node publishes every QTM frame (~300/s)
T0 = 50.0
POS0 = np.array([120.0, -40.0, 900.0])
VEL0 = np.array([150.0, -80.0, 4600.0])   # apex ~1.08 m above release


def _crossing_tau():
    """Time from T0 to the descending crossing of PLANE (vertical part 9806)."""
    a, b, c = -0.5 * GRAVITY_MMPS2, VEL0[2], POS0[2] - PLANE
    return (-b - math.sqrt(b * b - 4 * a * c)) / (2 * a)


def _pos(tau, g_xy):
    p = POS0 + VEL0 * tau
    p[0] += 0.5 * g_xy[0] * tau * tau
    p[1] += 0.5 * g_xy[1] * tau * tau
    p[2] -= 0.5 * GRAVITY_MMPS2 * tau * tau
    return p


def _flight(g_xy, max_age_s=None):
    """(FlightFit with gravity (0,0), FlightFit with G_XY, admitted taus) fed
    the same noise-free flight under gravity ``g_xy``, optionally cut
    ``max_age_s`` after the first admitted sample (an early Fit)."""
    fits = [FlightFit(T0, PLANE, freeze_above_plane_mm=FREEZE),
            FlightFit(T0, PLANE, freeze_above_plane_mm=FREEZE,
                      gravity_xy_mmps2=G_XY)]
    admitted = []
    for k in range(int(_crossing_tau() / DT) + 1):
        tau = k * DT
        p = _pos(tau, g_xy)
        if p[2] > PLANE + FREEZE:
            if admitted and max_age_s is not None and tau - admitted[0] > max_age_s:
                break
            admitted.append(tau)
        for f in fits:
            f.add(T0 + tau, p)
    return fits[0], fits[1], np.array(admitted)


def _truth():
    T = _crossing_tau()
    return _pos(T, G_XY)[:2], T


@pytest.mark.parametrize('max_age_s', [None, 0.2])
def test_tilted_flight_lands_within_half_a_mm_with_the_vector(max_age_s):
    _, tilted, _ = _flight(G_XY, max_age_s)
    pos, vel, t_abs = tilted.landing(PLANE)
    truth_xy, T = _truth()
    assert np.all(np.abs(pos[:2] - truth_xy) < 0.5), pos[:2] - truth_xy
    assert t_abs == pytest.approx(T0 + T, abs=1e-6)
    # Landing velocity carries the horizontal gravity too.
    assert vel[0] == pytest.approx(VEL0[0] + G_XY[0] * T, abs=0.05)
    assert vel[1] == pytest.approx(VEL0[1] + G_XY[1] * T, abs=0.05)


@pytest.mark.parametrize('max_age_s', [None, 0.2])
def test_tilted_flight_is_biased_by_the_predicted_amount_without_it(max_age_s):
    level, _, taus = _flight(G_XY, max_age_s)
    pos, _, _ = level.landing(PLANE)
    truth_xy, T = _truth()
    bias = pos[:2] - truth_xy
    # Independent prediction: least squares fits a line to the horizontal
    # 0.5 g tau^2 term over the admitted samples; the landing error is that
    # line's extrapolation to the crossing minus the term itself.
    predicted = []
    for g in G_XY:
        q = 0.5 * g * taus ** 2
        slope, icpt = np.polyfit(taus, q, 1)
        predicted.append(slope * T + icpt - 0.5 * g * T * T)
    assert bias == pytest.approx(predicted, abs=0.05)
    # Not a rounding effect: the y bias alone is several mm (-) here.
    assert bias[1] < -2.0


def test_a_level_flight_is_unchanged_by_a_zero_vector_and_biased_by_a_wrong_one():
    level, tilted, _ = _flight((0.0, 0.0))
    p0, _, _ = level.landing(PLANE)
    T = _crossing_tau()
    assert np.allclose(p0[:2], _pos(T, (0.0, 0.0))[:2], atol=1e-6)
    p1, _, _ = tilted.landing(PLANE)
    assert p1[1] - p0[1] > 2.0   # the vector is not a free lunch: it must be measured


# ── (0, 0) is the pre-FEAT-27 arithmetic bit for bit ──────────────────────────

def _old_fit_and_landing(ts, ps, landing_z, min_samples=12, residual_mm=12.0,
                         min_span_s=0.050):
    """Frozen copy of `FlightFit.fit` + `landing` + `predict_landing_state`
    as of main 8afbab7d (before FEAT-27), over already-admitted samples."""
    t_arr = np.asarray(ts, dtype=np.float64)
    pos_arr = np.asarray(ps, dtype=np.float64)
    t_ref = float(t_arr[-1])
    mask = np.ones(len(t_arr), dtype=bool)
    result = None
    for _pass in range(2):
        idx = np.nonzero(mask)[0]
        tau = t_arr[idx] - t_ref
        A = np.vstack([np.ones_like(tau), tau]).T
        targets = pos_arr[idx].copy()
        targets[:, 2] = targets[:, 2] + 0.5 * GRAVITY_MMPS2 * tau ** 2
        coeffs, *_ = np.linalg.lstsq(A, targets, rcond=None)
        pos0 = coeffs[0].copy()
        vel0 = coeffs[1].copy()
        pred = (pos0[None, :] + vel0[None, :] * tau[:, None])
        pred[:, 2] = pred[:, 2] - 0.5 * GRAVITY_MMPS2 * tau ** 2
        residuals = np.linalg.norm(pos_arr[idx] - pred, axis=1)
        result = (pos0, vel0, idx, residuals)
        keep = residuals <= residual_mm
        if keep.all():
            break
        new_mask = np.zeros(len(t_arr), dtype=bool)
        new_mask[idx[keep]] = True
        mask = new_mask
    pos0, vel0, idx, residuals = result
    assert idx.size >= min_samples
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    x, y, z = (float(v) for v in pos0)
    vx, vy, vz = (float(v) for v in vel0)
    a, b, c = -0.5 * GRAVITY_MMPS2, vz, z - landing_z
    sq = math.sqrt(b * b - 4.0 * a * c)
    ttl = min(t for t in ((-b + sq) / (2.0 * a), (-b - sq) / (2.0 * a)) if t > 1e-9)
    land = np.array([x + vx * ttl, y + vy * ttl, landing_z])
    lvel = np.array([vx, vy, vz - GRAVITY_MMPS2 * ttl])
    return pos0, vel0, rms, land, lvel, t_ref + ttl


@pytest.mark.parametrize('seed', [0, 1, 2])
def test_zero_vector_reproduces_the_old_fit_bit_for_bit(seed):
    """Recorded-style data: 3 mm marker noise, a few 40 mm outliers (the
    residual gate's second pass runs), pre-release and in-cup samples (the
    admission rules drop them), the measured tilt in the data itself."""
    rng = np.random.default_rng(seed)
    default = FlightFit(T0 + 0.02, PLANE, freeze_above_plane_mm=FREEZE)
    explicit = FlightFit(T0 + 0.02, PLANE, freeze_above_plane_mm=FREEZE,
                         gravity_xy_mmps2=(0.0, 0.0))
    kept_t, kept_p = [], []
    for k in range(int(_crossing_tau() / DT) + 30):
        tau = k * DT
        p = _pos(tau, G_XY) + rng.normal(0.0, 3.0, 3)
        if k % 37 == 5:
            p = p + np.array([40.0, -25.0, 10.0])
        for f in (default, explicit):
            f.add(T0 + tau, p)
        if T0 + tau >= T0 + 0.02 and p[2] > PLANE + FREEZE:
            kept_t.append(T0 + tau)
            kept_p.append(p)
        if k > 20 and len(kept_t) > 20 and p[2] <= PLANE + FREEZE:
            break    # the ball is back in the Cup: the flight is frozen
    pos0, vel0, rms, land, lvel, t_land = _old_fit_and_landing(kept_t, kept_p, PLANE)
    for f in (default, explicit):
        fr = f.fit()
        assert np.array_equal(fr.pos_ref, pos0) and np.array_equal(fr.vel_ref, vel0)
        assert fr.rms_mm == rms
        p, v, t = f.landing(PLANE)
        assert np.array_equal(p, land) and np.array_equal(v, lvel) and t == t_land


def test_predict_landing_state_zero_vector_is_bit_identical():
    pos, vel = np.array([12.3, -4.5, 1500.7]), np.array([140.1, -77.7, 1234.5])
    ref = predict_landing_state(pos, vel, PLANE)
    for g in (None, (0.0, 0.0), [0.0, 0.0]):
        got = predict_landing_state(pos, vel, PLANE, g_xy_mmps2=g)
        assert np.array_equal(got[0], ref[0]) and np.array_equal(got[1], ref[1])
        assert got[2] == ref[2]


# ── wiring: config -> BallTracker -> every FlightFit and the KF fallback ──────

def test_config_default_is_zero_so_the_fit_is_unchanged_until_switched_on():
    import jugglebot.hardware_config as hw
    assert list(hw.TRACKING_GRAVITY_WORLD_XY_MMPS2) == [0.0, 0.0]


def _confirmed_tracker(g):
    """A tracker that has confirmed one announced ball and fed its fit."""
    tr = BallTracker(dt=DT, landing_z=PLANE, gravity_world_xy_mmps2=g)
    bid = tr.handle_announcement(POS0.copy(), VEL0.copy(), T0)
    for k in range(1, 120):
        tr.process_frame([_pos(k * DT, G_XY)], T0 + k * DT)
    return tr, bid


def test_tracker_hands_its_vector_to_every_fit_and_the_live_setter_updates_them():
    tr, bid = _confirmed_tracker(G_XY)
    assert bid is not None and bid in tr.flight_fits
    assert tr.flight_fits[bid].gravity_xy_mmps2 == G_XY
    tr.set_gravity_world_xy(0.0, 0.0)
    assert tr.gravity_world_xy_mmps2 == (0.0, 0.0)
    assert tr.flight_fits[bid].gravity_xy_mmps2 == (0.0, 0.0)
    tr2, bid2 = _confirmed_tracker((0.0, 0.0))
    assert tr2.flight_fits[bid2].gravity_xy_mmps2 == (0.0, 0.0)


def test_kalman_fallback_landing_uses_the_vector():
    from jugglebot.tracking.ball import Ball
    tr = BallTracker(dt=DT, landing_z=PLANE, gravity_world_xy_mmps2=G_XY)
    ball = Ball(id=999, status=None, tracking=None, source='ball_butler',
                position=POS0.copy(), velocity=VEL0.copy(), timestamp=T0)
    tr._update_landing_prediction(ball)
    assert ball.landing_from_fit is False
    want = predict_landing_state(POS0, VEL0, PLANE, g_xy_mmps2=G_XY)
    assert np.array_equal(ball.landing_position, want[0])
    assert np.array_equal(ball.landing_velocity, want[1])
