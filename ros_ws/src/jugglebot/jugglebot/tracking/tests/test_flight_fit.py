"""Tests for flight_fit.py — the gravity-fixed batch landing estimator.

See `flight_fit.py`'s module docstring for the defect this replaces: the
Kalman filter's `landing_time` runs 0.06-0.20 s LATE against the physical
flight and grows later through the descent (measured 2026-09-17, two
sittings, 37 throws). These tests are pure synthetic ballistics — no ROS2,
no bag data — the bag validation lives in
`temp/probes/tracker_fit_validation_20260917.md` via
`tools/probes/tracker_bag_replay.py`.
"""
from __future__ import annotations

import numpy as np
import pytest

from jugglebot.tracking.ballistics import GRAVITY_MMPS2, predict_landing_state
from jugglebot.tracking.flight_fit import FlightFit


def _free_flight_pos(pos0, vel0, t0, t):
    tau = t - t0
    p = np.array(pos0, dtype=np.float64) + np.array(vel0, dtype=np.float64) * tau
    p[2] -= 0.5 * GRAVITY_MMPS2 * tau * tau
    return p


def _true_crossing_abs(pos0, vel0, t0, landing_z):
    """Analytic absolute crossing time — used to check `FlightFit.landing()`
    without depending on which internal reference instant the fit anchors
    its coefficients to (that is an implementation detail: `fit()` anchors
    at the LAST admitted sample, not the first — see `flight_fit.py` for
    why)."""
    _, _, ttl = predict_landing_state(np.asarray(pos0, dtype=np.float64),
                                       np.asarray(vel0, dtype=np.float64),
                                       landing_z)
    return t0 + ttl


class TestSyntheticAccuracy:
    """192 Hz synthetic flight, 3 mm Gaussian noise, fixed seed for determinism."""

    POS0 = np.array([0.0, 0.0, 700.0])
    VEL0 = np.array([300.0, 150.0, 4200.0])
    T0 = 100.0
    DT = 1.0 / 192.0
    LANDING_Z = 400.0  # below the whole synthetic window: pure extrapolation

    def _true_crossing_abs(self):
        _, _, ttl = predict_landing_state(self.POS0, self.VEL0, self.LANDING_Z)
        return self.T0 + ttl

    def _samples(self, n, noise_std=3.0, seed=42):
        rng = np.random.default_rng(seed)
        ts = self.T0 + self.DT * np.arange(n)
        noise = rng.normal(0.0, noise_std, size=(n, 3))
        pts = [_free_flight_pos(self.POS0, self.VEL0, self.T0, t) + noise[i]
               for i, t in enumerate(ts)]
        return ts, pts

    def _fit_crossing_error_ms(self, n):
        ts, pts = self._samples(n)
        ff = FlightFit(throw_time=self.T0, landing_z=self.LANDING_Z,
                        min_samples=12, residual_mm=12.0,
                        freeze_above_plane_mm=250.0)
        for t, p in zip(ts, pts):
            ff.add(t, p)
        landing = ff.landing(self.LANDING_Z)
        assert landing is not None
        _, _, t_abs = landing
        return (t_abs - self._true_crossing_abs()) * 1000.0

    def test_crossing_error_under_3ms_at_20_samples(self):
        assert abs(self._fit_crossing_error_ms(20)) < 3.0

    def test_crossing_error_under_1p5ms_at_60_samples(self):
        assert abs(self._fit_crossing_error_ms(60)) < 1.5

    def test_more_samples_do_not_need_more_than_min_samples_to_be_valid(self):
        ts, pts = self._samples(12, noise_std=0.0)
        ff = FlightFit(throw_time=self.T0, landing_z=self.LANDING_Z,
                        min_samples=12, residual_mm=12.0,
                        freeze_above_plane_mm=250.0)
        for t, p in zip(ts, pts):
            ff.add(t, p)
        result = ff.fit()
        assert result is not None
        assert result.n == 12
        # Noiseless data: the fitted parabola is exact regardless of which
        # sample `fit()` anchors t_ref at, so the crossing it predicts must
        # match the analytic one to floating-point precision.
        landing = ff.landing(self.LANDING_Z)
        assert landing is not None
        assert abs(landing[2] - self._true_crossing_abs()) < 1e-6


class TestAdmissionRules:
    T0 = 50.0
    DT = 1.0 / 192.0
    POS0 = np.array([0.0, 0.0, 700.0])
    VEL0 = np.array([200.0, 100.0, 3800.0])
    LANDING_Z = 830.0

    def _fresh(self, **over):
        kwargs = dict(throw_time=self.T0, landing_z=self.LANDING_Z,
                      min_samples=12, residual_mm=12.0,
                      freeze_above_plane_mm=250.0)
        kwargs.update(over)
        return FlightFit(**kwargs)

    def test_pre_release_samples_excluded(self):
        """A ball pushed at 1.5 g for 40 ms before release rides the cup —
        those samples must not reach the fit at all."""
        ff = self._fresh()
        n_pre = int(0.040 / self.DT)
        a_push = 1.5 * GRAVITY_MMPS2
        for i in range(n_pre, 0, -1):
            t = self.T0 - i * self.DT
            dtau = t - self.T0  # negative: time before release
            z = self.POS0[2] + self.VEL0[2] * dtau - 0.5 * a_push * dtau * dtau
            x = self.POS0[0] + self.VEL0[0] * dtau
            ff.add(t, np.array([x, 0.0, z]))
        assert len(ff) == 0, "pre-release (cup) samples must be dropped"

        # Free-flight samples after throw_time recover the true trajectory
        # exactly (no noise) — proof the pre-release samples exerted zero
        # influence: the predicted crossing matches the analytic one. Use a
        # plane BELOW the release height (unlike `self.LANDING_Z`, which
        # sits between release and apex on purpose for the freeze-rule
        # test below) so there is only one physically-sensible crossing and
        # the comparison is unambiguous.
        for i in range(40):
            t = self.T0 + i * self.DT
            ff.add(t, _free_flight_pos(self.POS0, self.VEL0, self.T0, t))
        result = ff.fit()
        assert result is not None
        check_landing_z = 400.0
        landing = ff.landing(check_landing_z)
        assert landing is not None
        true_abs = _true_crossing_abs(self.POS0, self.VEL0, self.T0, check_landing_z)
        assert abs(landing[2] - true_abs) < 1e-6

    def test_in_cup_deceleration_after_contact_does_not_move_estimate(self):
        """The cup catches the ball ~170 mm above the 830 mm plane (contact
        z ~ 1000 mm on both measured bags). Once the ball is descending and
        below `landing_z + freeze_above_plane_mm` (1080 mm here), later
        samples — even non-ballistic ones, decelerating at 20 m/s^2 — must
        not move the fit."""
        ff = self._fresh()
        i = 0
        contact_t = contact_z = contact_vz = None
        while True:
            t = self.T0 + i * self.DT
            tau = t - self.T0
            p = _free_flight_pos(self.POS0, self.VEL0, self.T0, t)
            vz = self.VEL0[2] - GRAVITY_MMPS2 * tau
            if p[2] <= 1000.0 and vz < 0:
                contact_t, contact_z, contact_vz = t, p[2], vz
                break
            ff.add(t, p)
            i += 1
            assert i < 2000, "trajectory never reached the contact height"

        n_before = len(ff)
        fit_before = ff.fit()
        assert fit_before is not None

        # In-cup deceleration: 20 m/s^2 opposing the downward velocity.
        z, vz, t = contact_z, contact_vz, contact_t
        for _ in range(8):
            t += self.DT
            a = 20000.0 if vz < 0 else 0.0
            vz += a * self.DT
            z += vz * self.DT
            x = self.POS0[0] + self.VEL0[0] * (t - self.T0)
            ff.add(t, np.array([x, 0.0, z]))

        assert len(ff) == n_before, "in-cup samples must not be admitted"
        fit_after = ff.fit()
        assert fit_after is not None
        np.testing.assert_allclose(fit_after.vel_ref, fit_before.vel_ref)
        np.testing.assert_allclose(fit_after.pos_ref, fit_before.pos_ref)

    def test_one_wild_marker_rejected(self):
        """An 80 mm jump on one sample must not survive the outlier gate."""
        # landing_z 400 keeps the whole synthetic window above the cup floor
        # (the floor is landing_z + 250 on both legs since 2026-09-17).
        ff = self._fresh(landing_z=400.0)
        n = 30
        for i in range(n):
            t = self.T0 + i * self.DT
            p = _free_flight_pos(self.POS0, self.VEL0, self.T0, t)
            if i == 15:
                p = p + np.array([80.0, 0.0, 0.0])
            ff.add(t, p)
        result = ff.fit()
        assert result is not None
        assert result.n == n - 1, "exactly the wild sample should be dropped"
        check_landing_z = 400.0
        landing = ff.landing(check_landing_z)
        assert landing is not None
        true_abs = _true_crossing_abs(self.POS0, self.VEL0, self.T0, check_landing_z)
        assert abs(landing[2] - true_abs) < 1e-6

    def test_fewer_than_min_samples_returns_none(self):
        ff = self._fresh(min_samples=12)
        for i in range(5):
            t = self.T0 + i * self.DT
            ff.add(t, _free_flight_pos(self.POS0, self.VEL0, self.T0, t))
        assert ff.fit() is None
        assert ff.landing(self.LANDING_Z) is None

    def test_span_below_min_span_returns_none(self):
        """12 samples that don't span enough wall-clock time (all crammed
        into one instant) must not produce a fit."""
        ff = self._fresh(min_samples=3, min_span_s=0.050)
        for i in range(3):
            # 1 ms apart — well under the 50 ms span floor
            t = self.T0 + i * 0.001
            ff.add(t, _free_flight_pos(self.POS0, self.VEL0, self.T0, t))
        assert ff.fit() is None


class TestReset:
    T0 = 10.0
    DT = 1.0 / 192.0
    POS0 = np.array([0.0, 0.0, 700.0])
    VEL0 = np.array([100.0, 0.0, 3000.0])
    LANDING_Z = 830.0

    def test_reset_restarts_fit_for_a_new_flight(self):
        """A chained self-toss re-uses the fit lifecycle: reset() clears the
        old flight's samples so the new flight's fit isn't contaminated."""
        check_landing_z = 400.0  # below release: unambiguous single crossing
        # The fit's own plane is the check plane too: the cup floor sits
        # 250 mm above it on both legs, and this 700 mm release must clear it.
        ff = FlightFit(throw_time=self.T0, landing_z=check_landing_z,
                        min_samples=12, residual_mm=12.0,
                        freeze_above_plane_mm=250.0)
        for i in range(30):
            t = self.T0 + i * self.DT
            ff.add(t, _free_flight_pos(self.POS0, self.VEL0, self.T0, t))
        first = ff.fit()
        assert first is not None
        first_landing = ff.landing(check_landing_z)
        assert first_landing is not None
        true1 = _true_crossing_abs(self.POS0, self.VEL0, self.T0, check_landing_z)
        assert abs(first_landing[2] - true1) < 1e-6

        new_t0 = 20.0
        new_vel0 = np.array([-150.0, 50.0, 2500.0])
        ff.reset(throw_time=new_t0)
        assert len(ff) == 0
        assert ff.fit() is None  # nothing accumulated yet

        for i in range(30):
            t = new_t0 + i * self.DT
            ff.add(t, _free_flight_pos(self.POS0, new_vel0, new_t0, t))
        second = ff.fit()
        assert second is not None
        second_landing = ff.landing(check_landing_z)
        assert second_landing is not None
        true2 = _true_crossing_abs(self.POS0, new_vel0, new_t0, check_landing_z)
        assert abs(second_landing[2] - true2) < 1e-6

    def test_reset_drops_samples_from_before_the_old_throw_time_too(self):
        """After reset, a sample timestamped between the old and new
        throw_time is correctly treated as pre-release for the NEW flight."""
        ff = FlightFit(throw_time=self.T0, landing_z=400.0,
                        min_samples=3, residual_mm=12.0,
                        freeze_above_plane_mm=250.0)
        ff.add(self.T0 + 0.001, self.POS0)
        assert len(ff) == 1
        ff.reset(throw_time=self.T0 + 5.0)
        # A sample that was admissible under the OLD throw_time is now
        # pre-release under the NEW one.
        ff.add(self.T0 + 0.001, self.POS0)
        assert len(ff) == 0
