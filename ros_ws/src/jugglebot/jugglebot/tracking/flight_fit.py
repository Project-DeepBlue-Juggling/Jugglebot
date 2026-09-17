"""Gravity-fixed batch least-squares fit of one ball's free-flight — pure Python.

Why this exists (measured 2026-09-17, two sittings, 37 throws): the tracker's
`landing_time` — the Kalman filter's state extrapolated through
`predict_landing_state` — runs late against the physical flight, and the
lateness GROWS through the descent (0.06-0.10 s late near the apex, 0.10-0.20 s
late at the last in-flight sample). A recursive filter is the wrong tool for a
quantity gravity fixes exactly: with g known, a flight is TWO numbers per axis
(position and velocity at some reference time), and a batch least-squares fit
over the free-flight samples recovers the plane crossing to a few ms from
~20 samples. See `matcher.BallTracker` for how one `FlightFit` per CONFIRMED
ball is wired in alongside (not instead of) the Kalman filter — the KF still
owns the published position/velocity; only the LANDING estimate changes.

No ROS2 dependency, so this can be tested standalone and reached from
`tools/probes/tracker_bag_replay.py` without importing rclpy.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .ballistics import GRAVITY_MMPS2, predict_landing_state


@dataclass
class FitResult:
    """One gravity-fixed fit of a ball's free flight, evaluated at `t_ref`."""
    pos_ref: np.ndarray   # [x, y, z] mm at t_ref
    vel_ref: np.ndarray   # [vx, vy, vz] mm/s at t_ref
    t_ref: float          # absolute seconds (the fit's reference instant)
    n: int                # surviving sample count after outlier rejection
    rms_mm: float         # 3D residual RMS over the surviving samples


class FlightFit:
    """Accumulates raw `(t, pos)` samples for ONE ball's free flight and
    produces, on demand, a gravity-fixed least-squares fit:

        x = x0 + vx*tau
        y = y0 + vy*tau
        z = z0 + vz*tau - g*tau^2/2      (tau = t - t_ref)

    Admission rules applied in `add()` (see there for the reasoning):
      1. samples strictly before the ball's announced `throw_time` are
         dropped — they ride the cup, not free flight.
      2. samples at or below `landing_z + freeze_above_plane_mm` are dropped
         on BOTH legs of the flight. Descending: the cup catches the ball
         well above the landing plane (marker ~1000-1010 mm on 2026-09-17,
         plane 830), and in-cup deceleration samples are exactly what drags
         a recursive estimate late. Ascending: the announced `throw_time`
         precedes the physical release by 5-55 ms (the ball is still being
         PUSHED at > g by the cup — 2026-09-17, both sittings), and those
         pushed samples bias the fitted launch speed low, i.e. the crossing
         EARLY; the first bag validation measured -53 ms at the catch
         dispatch instant with ascending samples admitted from the release.
         The floor is the same on both legs because the cup is: the ball
         leaves it and re-enters it at the same height band.

    `fit()` then applies a robust two-pass residual gate on top: any sample
    whose 3D residual against the current fit exceeds `residual_mm` is
    dropped and the fit re-run once. A fit is valid only with at least
    `min_samples` surviving samples spanning at least `min_span_s`.
    """

    def __init__(
        self,
        throw_time: float,
        landing_z: float,
        min_samples: int = 12,
        residual_mm: float = 12.0,
        freeze_above_plane_mm: float = 250.0,
        min_span_s: float = 0.050,
        max_samples: int = 400,
    ):
        self.throw_time = float(throw_time)
        self.landing_z = float(landing_z)
        self.min_samples = int(min_samples)
        self.residual_mm = float(residual_mm)
        self.freeze_floor_z = float(landing_z) + float(freeze_above_plane_mm)
        self.min_span_s = float(min_span_s)
        self.max_samples = int(max_samples)

        self._t: List[float] = []
        self._pos: List[np.ndarray] = []

    def reset(self, throw_time: Optional[float] = None) -> None:
        """Clear all accumulated samples — a fresh flight starts clean.

        Called by `matcher.BallTracker` when a ball is re-announced /
        re-thrown (a chained self-toss re-uses the same fit lifecycle: a new
        FlightFit is normally just created for the new track, but `reset` is
        exposed so a caller than wants to reuse one instance in place can, and
        the tracker uses it defensively at the CAUGHT -> IN_FLIGHT hook).
        """
        if throw_time is not None:
            self.throw_time = float(throw_time)
        self._t = []
        self._pos = []

    def add(self, t: float, pos) -> None:
        """Feed one raw (unfiltered) marker sample.

        Silently drops samples that fail the admission rules — most frames of
        a flight are good data; the ones filtered out are pre-release or
        in-cup. Not an error path.
        """
        t = float(t)
        pos = np.asarray(pos, dtype=np.float64).reshape(3)

        if t < self.throw_time:
            return  # pre-release: still riding the cup/thrower

        z = float(pos[2])
        if z <= self.freeze_floor_z:
            return  # in/near the cup: pushed on the way up, caught on the way down

        self._t.append(t)
        self._pos.append(pos)
        if len(self._t) > self.max_samples:
            self._t.pop(0)
            self._pos.pop(0)

    def __len__(self) -> int:
        return len(self._t)

    def fit(self) -> Optional[FitResult]:
        """Gravity-fixed batch least-squares fit over the admitted samples.

        Returns None if fewer than `min_samples` samples survive (including
        after outlier rejection) or they don't span `min_span_s`.
        """
        if len(self._t) < self.min_samples:
            return None

        t_arr = np.asarray(self._t, dtype=np.float64)
        pos_arr = np.asarray(self._pos, dtype=np.float64)  # (N, 3)
        # t_ref = the MOST RECENT sample, not the first (found 2026-09-17
        # validating against real bags). A real ball has aerodynamic drag,
        # so a batch fit spanning the WHOLE flight from near-release (its
        # highest-speed, highest-drag-curvature point) is measurably biased
        # (~18 mm rms on a 0.4 s window in one bag) — algebraically that bias
        # doesn't change which parabola gets fitted, but it CAN put the
        # evaluated position at t_ref slightly on the wrong side of
        # `landing_z` when t_ref is far from most of the data (as the first
        # sample is once a flight has run for a while). `predict_landing_state`
        # then sees two positive roots (a spurious near-zero one plus the
        # real future one) and picks the SMALLEST — the spurious one — which
        # is exactly the "landing_time snaps to ~throw_time" failure this
        # fixes. Anchoring at the last sample keeps the evaluated position
        # solidly above the landing plane (the freeze rule guarantees the
        # last DESCENDING sample stays `freeze_above_plane_mm` above it), so
        # there is only ever one positive root to find.
        t_ref = float(t_arr[-1])

        mask = np.ones(len(t_arr), dtype=bool)
        result = None
        for _pass in range(2):
            idx = np.nonzero(mask)[0]
            if idx.size < self.min_samples:
                return None
            tau = t_arr[idx] - t_ref
            if (tau.max() - tau.min()) < self.min_span_s:
                return None

            A = np.vstack([np.ones_like(tau), tau]).T  # (n, 2)
            targets = pos_arr[idx].copy()
            targets[:, 2] = targets[:, 2] + 0.5 * GRAVITY_MMPS2 * tau ** 2

            coeffs, *_ = np.linalg.lstsq(A, targets, rcond=None)  # (2, 3)
            pos0 = coeffs[0].copy()
            vel0 = coeffs[1].copy()

            pred = (pos0[None, :] + vel0[None, :] * tau[:, None])
            pred[:, 2] = pred[:, 2] - 0.5 * GRAVITY_MMPS2 * tau ** 2
            residuals = np.linalg.norm(pos_arr[idx] - pred, axis=1)

            result = (pos0, vel0, idx, residuals)

            keep = residuals <= self.residual_mm
            if keep.all():
                break
            new_mask = np.zeros(len(t_arr), dtype=bool)
            new_mask[idx[keep]] = True
            mask = new_mask

        pos0, vel0, idx, residuals = result
        n = int(idx.size)
        if n < self.min_samples:
            return None
        tau_final = t_arr[idx] - t_ref
        if (tau_final.max() - tau_final.min()) < self.min_span_s:
            return None

        rms_mm = float(np.sqrt(np.mean(residuals ** 2)))
        return FitResult(
            pos_ref=pos0,
            vel_ref=vel0,
            t_ref=t_ref,
            n=n,
            rms_mm=rms_mm,
        )

    def landing(
        self, landing_z: float
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """Landing (position, velocity, absolute_time) at `landing_z`, or None.

        Reuses `ballistics.predict_landing_state` on the fitted state at
        `t_ref` — the crossing arithmetic has exactly one definition, shared
        with the Kalman-filter landing path.
        """
        fr = self.fit()
        if fr is None:
            return None
        result = predict_landing_state(fr.pos_ref, fr.vel_ref, landing_z)
        if result is None:
            return None
        landing_pos, landing_vel, time_to_land = result
        return landing_pos, landing_vel, fr.t_ref + time_to_land
