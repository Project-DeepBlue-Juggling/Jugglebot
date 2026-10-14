"""Ball tracking orchestrator — pure Python.

BallTracker is the main class that owns all Ball objects and drives:
  - Throw announcement handling (Ball Butler path)
  - Parabolic motion detection (human throw path; OFF by default)
  - Frame-to-frame marker association for tracked balls
  - Ball lifecycle transitions (status + cleanup)

No ROS2 dependency. The ROS2 node (ball_tracker_node.py) feeds data in
and reads results out.
"""
from __future__ import annotations

import logging
from collections import deque
from typing import Callable, Deque, Dict, List, Optional, Tuple

import numpy as np

from .ball import Ball, BallStatus, TrackingConfidence, TERMINAL_STATUSES
from .ballistics import GRAVITY_MMPS2, predict_landing_state
from .flight_fit import FlightFit
from .kalman import KalmanFilter

_LOG = logging.getLogger(__name__)


def parse_label_prefixes(raw) -> Tuple[str, ...]:
    """Split the generated comma-separated label-prefix list into a tuple.

    The config carries the excluded prefixes as ONE comma-separated string
    rather than a YAML list because `config/generate_config.py` also emits the
    `ball_tracking` section into the firmware header, and its C++ emitter has
    no representation for a list of strings (it would produce unquoted
    identifiers). Empty entries are dropped, so `""` means "exclude nothing".

    Lives here, in the pure-Python layer, so the ROS node stays a thin wrapper
    and so tests and `tools/probes/tracker_bag_replay.py` can reach it without
    importing rclpy.
    """
    if raw is None:
        return ()
    if isinstance(raw, (list, tuple)):
        parts = [str(p) for p in raw]
    else:
        parts = str(raw).split(',')
    return tuple(p.strip() for p in parts if p.strip())


# --- Parabolic detection helpers ---

class _MarkerTrack:
    """Internal: buffered unmatched marker for parabolic detection."""
    __slots__ = ("positions", "timestamps")

    def __init__(self):
        self.positions: list[np.ndarray] = []
        self.timestamps: list[float] = []

    def add(self, pos: np.ndarray, t: float):
        self.positions.append(pos.copy())
        self.timestamps.append(t)
        # Keep last 6 frames max
        if len(self.positions) > 6:
            self.positions.pop(0)
            self.timestamps.pop(0)

    @property
    def last_position(self) -> np.ndarray:
        return self.positions[-1]

    @property
    def last_timestamp(self) -> float:
        return self.timestamps[-1]

    def __len__(self):
        return len(self.positions)


class BallTracker:
    """Owns all Ball objects and processes mocap frames.

    Usage:
        tracker = BallTracker(dt=0.005, landing_z=734.3)

        # Ball Butler path:
        tracker.handle_announcement(position, velocity, throw_time, ...)

        # Each mocap frame — ALL markers, with their mocap labels:
        balls = tracker.process_frame(marker_positions, current_time, labels)
    """

    def __init__(
        self,
        dt: float = 0.005,
        landing_z: float = 734.3,
        match_threshold_base_mm: float = 100.0,
        gate_radius_mm: float = 50.0,
        parabolic_accel_threshold_mmps2: float = 3000.0,
        parabolic_min_frames: int = 3,
        missed_frames_to_lose: int = 10,
        caught_grace_s: float = 0.200,
        terminal_retention_s: float = 2.0,
        max_frames_without_measurement: int = 200,
        process_noise: float = 5.0,
        measurement_noise: float = 1.0,
        announced_gate_mm: float = 200.0,
        excluded_label_prefixes: Optional[Tuple[str, ...]] = None,
        detect_human_throws: bool = False,
        flight_fit_min_samples: int = 12,
        flight_fit_residual_mm: float = 12.0,
        flight_fit_freeze_above_plane_mm: float = 250.0,
        announced_release_lead_s: float = 0.10,
        announced_speed_ratio: Tuple[float, float] = (0.4, 1.8),
        announced_min_direction_cos: float = 0.5,
        announced_motion_steps: int = 2,
        demote_still_speed_mmps: float = 500.0,
        demote_still_s: float = 0.060,
        demote_min_predicted_speed_mmps: float = 2000.0,
        debug_log: Optional[Callable[[str], None]] = None,
    ):
        self.dt = dt
        #: Wall time of the previous processed frame -- the Kalman predict steps by
        #: the MEASURED interval, not the nominal ``dt`` (2026-09-18): the bags run
        #: at ~192 Hz (5.2 ms) against a 5.0 ms nominal, so a nominal step ran the
        #: filter clock 4 % slow and its state drifted behind the ball through
        #: every flight. Clamped to [0.4, 4] x nominal so a dropped frame or a
        #: replay restart never becomes one giant leap.
        self._last_frame_time: Optional[float] = None
        self.landing_z = landing_z
        self.match_threshold_base_mm = match_threshold_base_mm
        self.gate_radius_mm = gate_radius_mm
        self.parabolic_accel_threshold = parabolic_accel_threshold_mmps2
        self.parabolic_min_frames = parabolic_min_frames
        self.missed_frames_to_lose = missed_frames_to_lose
        self.caught_grace_s = caught_grace_s
        self.terminal_retention_s = terminal_retention_s
        self.max_frames_without_measurement = max_frames_without_measurement
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

        # --- Announced-ball gate (2026-09-15) ---
        # An ANNOUNCED ball is promoted to CONFIRMED by the mocap marker
        # nearest its *analytically expected* ballistic position, within
        # `announced_gate_mm`.  This replaced the old "unlabelled markers
        # only, above landing_z + min_height_above_landing_mm, within an
        # adaptive KF-distance threshold" gate, which confirmed nothing at
        # all on 2026-09-15 because QTM labels the ball (see
        # `logbook/2026-09-15-tracker-all-markers-gated-to-expected-ball.md`).
        #
        # A SPHERE, not a box: on the 2026-09-15 bag the platform's own
        # markers sit 204.6 / 206.3 / 219.5 / 220.7 mm from the catch cup, so
        # a ±200 mm axis-aligned box admits three of them while a 200 mm
        # sphere admits none.  Identity exclusion is still the primary
        # defence; the sphere is the second one.
        self.announced_gate_mm = announced_gate_mm

        # --- Announced-ball MOTION test (2026-10-14) ---
        # The sphere alone confirmed every Ball Butler feed of the 2026-10-14
        # two-ball columns sessions on a STATIC Ball Butler body marker:
        # `Ball Butler - 1,2,3,4,6,7` (labels deliberately eligible, see
        # above) sit 78-142 mm from BB's announced start in every frame, the
        # nearest (#7, about (-884,-350,1720)) won 0.02-0.04 s after
        # throw_time on 5/5 feeds read, and the gravity-fixed fit then took a
        # point at rest at z=1720 for an apex: `time_at_land` 0.34-0.36 s
        # before the announced landing, landing xy ~700 mm off. Meanwhile the
        # real ball had left BB's hand ~0.05-0.1 s BEFORE the announced
        # throw_time, faster than announced, and sat 230-380 mm from the
        # analytic position for the whole visible flight -- outside the
        # 200 mm sphere, so it was never a candidate at all.
        # (See the 2026-10-14 two-ball columns sessions entry.)
        #
        # So a candidate must now be CONSISTENT WITH THE ANNOUNCED FLIGHT
        # over consecutive frames, not merely near it on one frame:
        #  - gate radius `announced_gate_mm + |v_pred(t)| * lead_s`: a
        #    release that leads (or lags) the announced instant by up to
        #    `lead_s` displaces the ball ~|v| * lead_s along its path;
        #  - candidate history starts `lead_s` BEFORE throw_time (the ball is
        #    already moving in the hand), confirmation only at/after it;
        #  - motion: the marker's frame-to-frame continuation must move at
        #    [lo, hi] x |v_pred(t)| roughly along v_pred (cos >=
        #    `announced_min_direction_cos`) for `announced_motion_steps`
        #    consecutive steps. A static marker can never confirm an
        #    announcement that expects > ~1 m/s.
        self.announced_release_lead_s = float(announced_release_lead_s)
        self.announced_speed_ratio = (float(announced_speed_ratio[0]),
                                      float(announced_speed_ratio[1]))
        self.announced_min_direction_cos = float(announced_min_direction_cos)
        self.announced_motion_steps = int(announced_motion_steps)
        # Safety net for a wrong confirmation that slips through: a CONFIRMED
        # announced ball whose matched RAW marker moves slower than
        # `demote_still_speed_mmps` for longer than `demote_still_s` while
        # the announcement predicts > `demote_min_predicted_speed_mmps` is
        # released back to ANNOUNCED and that marker position is excluded for
        # that announcement. Raw marker, not KF velocity: the KF velocity is
        # seeded from the announcement and decays toward a static marker's
        # zero over far more than 60 ms. Only before the announced landing
        # (less 0.1 s), so a ball resting in the cup is never demoted.
        self.demote_still_speed_mmps = float(demote_still_speed_mmps)
        self.demote_still_s = float(demote_still_s)
        self.demote_min_predicted_speed_mmps = float(demote_min_predicted_speed_mmps)
        self._debug_log: Callable[[str], None] = (
            debug_log if debug_log is not None else _LOG.debug)
        # Per ANNOUNCED ball: last frame's gated candidates as
        # (frame time, [(position, consecutive passing steps)]).
        self._candidate_history: Dict[int, Tuple[float, List[Tuple[np.ndarray, int]]]] = {}
        # Per announced ball: marker positions released by a demotion.
        self._excluded_positions: Dict[int, List[np.ndarray]] = {}
        # Per CONFIRMED announced ball: (t, raw marker) where its current
        # "still" interval began.
        self._still_anchor: Dict[int, Tuple[float, np.ndarray]] = {}
        # Per announced ball: the announcement's landing time (demotion window).
        self._announced_landing_time: Dict[int, float] = {}
        #: Last identity decisions (confirm / demote), newest last -- for tests
        #: and offline probes; the same text goes to `debug_log`.
        self.identity_events: Deque[dict] = deque(maxlen=64)

        # Marker labels whose prefix means "this marker is bolted to the
        # machine, it can never be a ball".  Everything else — including
        # unlabelled markers AND markers QTM has labelled as part of some
        # other rigid body — stays eligible, because the ball itself carries
        # a rigid-body label (`Ball Butler - 1`) whenever QTM's AIM model
        # claims it.  Exclusion is therefore by rigid-body MEMBERSHIP of the
        # robot, never by "does this look like a ball label".
        self.excluded_label_prefixes: Tuple[str, ...] = tuple(
            excluded_label_prefixes if excluded_label_prefixes is not None
            else ()
        )

        # Human-throw (parabolic) detection.  OFF by default: on a static
        # floor marker it spawned 158 phantom `human_throw` tracks in one
        # 2026-09-15 session, and nothing on the skill stack consumes them
        # yet.  Turn it on when human throws are actually being caught.
        self.detect_human_throws = bool(detect_human_throws)

        # --- Landing: gravity-fixed batch fit (2026-09-17) ---
        # One `FlightFit` per CONFIRMED ball, fed the RAW matched marker in
        # `_associate_confirmed_balls`. `_update_landing_prediction` uses its
        # landing estimate whenever the fit is valid; the KF-extrapolation
        # path below is the fallback for balls the fit hasn't converged on
        # yet (too few free-flight samples). See `flight_fit.py` for why: a
        # recursive filter's `landing_time` runs 0.06-0.20 s late and grows
        # later through the descent, which is why it is not this decision's
        # only estimator any more — see `tracking/flight_fit.py` module doc.
        self.flight_fit_min_samples = int(flight_fit_min_samples)
        self.flight_fit_residual_mm = float(flight_fit_residual_mm)
        self.flight_fit_freeze_above_plane_mm = float(flight_fit_freeze_above_plane_mm)
        self._flight_fits: Dict[int, FlightFit] = {}

        # Active balls keyed by id
        self._balls: Dict[int, Ball] = {}
        # Kalman filters keyed by ball id
        self._filters: Dict[int, KalmanFilter] = {}
        # Announcement initial states for analytical ballistic prediction
        # keyed by ball id: (initial_position, initial_velocity, throw_time)
        self._announcement_states: Dict[int, Tuple[np.ndarray, np.ndarray, float]] = {}
        # Next monotonic ball id
        self._next_id = 1

        # Unlabelled marker buffer for parabolic detection
        # Keyed by a transient track id (not ball id)
        self._marker_tracks: Dict[int, _MarkerTrack] = {}
        self._next_track_id = 1

        # Diagnostic: acceleration residuals from parabolic checks.
        # Each entry is (residual_mmps2, was_promoted). Useful for tuning.
        self.accel_residuals: List[Tuple[float, bool]] = []

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def handle_announcement(
        self,
        initial_position: np.ndarray,
        initial_velocity: np.ndarray,
        throw_time: float,
        source: str = "ball_butler",
        destination: str = "",
        landing_position: Optional[np.ndarray] = None,
        landing_velocity: Optional[np.ndarray] = None,
        landing_time: Optional[float] = None,
    ) -> int:
        """Register an announced throw (Ball Butler path).

        Args:
            initial_position: [x,y,z] mm at throw time.
            initial_velocity: [vx,vy,vz] mm/s at throw time.
            throw_time: Absolute time when ball will be released.
            source: Thrower identifier.
            destination: Which robot should catch this ball.
            landing_position/velocity/time: Pre-computed from announcement.
                If None, computed from ballistics.

        Returns:
            Ball id.
        """
        ball_id = self._next_id
        self._next_id += 1

        ball = Ball(
            id=ball_id,
            status=BallStatus.TO_BE_THROWN,
            tracking=TrackingConfidence.ANNOUNCED,
            source=source,
            destination=destination,
            position=np.array(initial_position, dtype=np.float64),
            velocity=np.array(initial_velocity, dtype=np.float64),
            timestamp=throw_time,  # pos/vel represent state at throw_time
            throw_time=throw_time,
        )

        # Pre-compute or use provided landing prediction
        if landing_position is not None and landing_velocity is not None and landing_time is not None:
            ball.landing_position = np.array(landing_position, dtype=np.float64)
            ball.landing_velocity = np.array(landing_velocity, dtype=np.float64)
            ball.landing_time = landing_time
        else:
            self._update_landing_prediction(ball)

        # Create Kalman filter initialized from announcement
        kf = KalmanFilter(
            dt=self.dt,
            process_noise=self.process_noise,
            measurement_noise=self.measurement_noise,
        )
        kf.initialize_from_state(
            np.array(initial_position, dtype=np.float64),
            np.array(initial_velocity, dtype=np.float64),
            position_cov=500.0,   # High uncertainty from announcement
            velocity_cov=500.0,
        )
        self._filters[ball_id] = kf
        self._balls[ball_id] = ball
        # `ball_id` is fresh (monotonic `_next_id`), so no stale FlightFit can
        # exist for it — pop defensively anyway: a chained self-toss re-uses
        # the physical ball but always gets a NEW ball_id here, so this is
        # the reset hook for "re-announced" even though today it is a no-op.
        self._flight_fits.pop(ball_id, None)
        self._announcement_states[ball_id] = (
            np.array(initial_position, dtype=np.float64),
            np.array(initial_velocity, dtype=np.float64),
            throw_time,
        )
        self._announced_landing_time[ball_id] = float(ball.landing_time)
        return ball_id

    def eligible_markers(
        self,
        marker_positions: List[np.ndarray],
        marker_labels: Optional[List[str]] = None,
    ) -> List[np.ndarray]:
        """Filter a mocap frame down to the markers that could be a ball.

        THE canonical enforcement point for marker eligibility: every stage of
        :meth:`process_frame` sees this list and nothing else.

        A marker is ineligible only if its label starts with one of
        ``excluded_label_prefixes`` — i.e. it is a marker of one of the
        robot's own rigid bodies.  Unlabelled markers are always eligible, and
        so is every label that is not excluded: the ball is routinely labelled
        by the mocap system (QTM's AIM model claimed the flying ball as
        ``Ball Butler - 1`` on all 13 throws of 2026-09-15), so a label is
        never itself evidence that a marker is not a ball.
        """
        if not self.excluded_label_prefixes or marker_labels is None:
            return list(marker_positions)
        out = []
        for idx, pos in enumerate(marker_positions):
            label = marker_labels[idx] if idx < len(marker_labels) else ""
            if label and label.startswith(self.excluded_label_prefixes):
                continue
            out.append(pos)
        return out

    def process_frame(
        self,
        marker_positions: List[np.ndarray],
        current_time: float,
        marker_labels: Optional[List[str]] = None,
    ) -> List[Ball]:
        """Process one mocap frame. Call at mocap rate (~200 Hz).

        Args:
            marker_positions: List of [x,y,z] arrays (mm) — ALL markers in the
                frame, labelled and unlabelled alike.
            current_time: Current time in seconds.
            marker_labels: Optional parallel list of mocap labels (``""`` for
                an unlabelled marker).  When given, markers belonging to the
                robot's own rigid bodies are dropped
                (:meth:`eligible_markers`).  When omitted, every marker is
                treated as eligible — the pre-2026-09-15 behaviour, retained
                so synthetic-marker tests and `tools/tracking_analyzer.py`
                need not invent labels.

        Returns:
            List of all active (non-terminal or recently-terminal) balls.
        """
        used_markers = set()
        markers = self.eligible_markers(marker_positions, marker_labels)

        # 1. Time-based transitions (TO_BE_THROWN → IN_FLIGHT)
        self._check_throw_times(current_time)

        # 2. Predict all Kalman filters forward, by the measured frame interval
        dt = self._frame_dt(current_time)
        for ball_id, kf in self._filters.items():
            ball = self._balls[ball_id]
            if ball.status == BallStatus.IN_FLIGHT:
                kf.predict(dt)

        # 3. Associate markers to existing CONFIRMED in-flight balls
        used_markers = self._associate_confirmed_balls(markers, current_time)

        # 4. Try to match remaining markers to ANNOUNCED balls
        used_markers = self._match_announced_balls(markers, current_time, used_markers)

        # 5. Detect parabolic motion in remaining markers (human throw path)
        if self.detect_human_throws:
            self._detect_parabolic(markers, current_time, used_markers)

        # 6. Lifecycle transitions (CAUGHT, DROPPED, UNKNOWN)
        self._check_lifecycle(current_time)

        # 7. Cleanup terminal balls past retention
        self._cleanup_terminal(current_time)

        self._last_frame_time = float(current_time)
        return list(self._balls.values())

    def _frame_dt(self, current_time: float) -> float:
        """The interval the Kalman filters step by on this frame: the measured
        gap to the previous frame, clamped to ``[0.4, 4] x self.dt``; the
        nominal ``dt`` on the first frame or a non-advancing clock."""
        if self._last_frame_time is None:
            return float(self.dt)
        gap = float(current_time) - self._last_frame_time
        if not gap > 0.0:
            return float(self.dt)
        return float(min(max(gap, 0.4 * self.dt), 4.0 * self.dt))

    @property
    def active_balls(self) -> List[Ball]:
        """All balls not yet cleaned up."""
        return list(self._balls.values())

    def get_ball(self, ball_id: int) -> Optional[Ball]:
        return self._balls.get(ball_id)

    # ------------------------------------------------------------------
    # Internal: Time-based transitions
    # ------------------------------------------------------------------

    def _check_throw_times(self, current_time: float):
        for ball in self._balls.values():
            if ball.status == BallStatus.TO_BE_THROWN and current_time >= ball.throw_time:
                ball.status = BallStatus.IN_FLIGHT
                ball.timestamp = current_time

    # ------------------------------------------------------------------
    # Internal: Frame-to-frame association (CONFIRMED balls)
    # ------------------------------------------------------------------

    def _associate_confirmed_balls(
        self,
        markers: List[np.ndarray],
        current_time: float,
    ) -> set:
        """Match markers to existing CONFIRMED in-flight balls using gated nearest-neighbour."""
        used = set()

        for ball_id, ball in self._balls.items():
            if ball.status != BallStatus.IN_FLIGHT or ball.tracking != TrackingConfidence.CONFIRMED:
                continue

            kf = self._filters.get(ball_id)
            if kf is None:
                continue

            predicted_pos = kf.position
            best_idx = None
            best_dist = float("inf")

            for idx, mpos in enumerate(markers):
                if idx in used:
                    continue
                dist = np.linalg.norm(mpos - predicted_pos)
                if dist < self.gate_radius_mm and dist < best_dist:
                    best_dist = dist
                    best_idx = idx

            if best_idx is not None and self._check_still_confirmed(
                    ball_id, ball, current_time, markers[best_idx]):
                # Demoted: the marker is free for other balls this frame, and
                # this ball rejoins the ANNOUNCED gate from the next frame on.
                continue
            if best_idx is not None:
                kf.update(markers[best_idx])
                used.add(best_idx)
                ball.frames_tracked += 1
                ball.frames_without_measurement = 0
                ball.last_seen = current_time
                # Feed the RAW matched marker (not the KF state) to this
                # ball's gravity-fixed flight fit.
                self._get_or_create_flight_fit(ball_id, ball).add(
                    current_time, markers[best_idx])
            else:
                ball.frames_without_measurement += 1

            # Update ball state from filter
            ball.position = kf.position
            ball.velocity = kf.velocity
            ball.timestamp = current_time

            # Refresh landing prediction
            self._update_landing_prediction(ball, from_filter=True)

        return used

    # ------------------------------------------------------------------
    # Internal: Announced ball matching
    # ------------------------------------------------------------------

    def _expected_announced_position(
        self,
        ball_id: int,
        current_time: float,
        fallback: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """Analytical ballistic position of an announced ball at ``current_time``.

        Drift-free by construction (closed form in ``current_time - throw_time``),
        unlike the Kalman filter, which steps by a fixed ``self.dt`` per frame.
        Returns ``fallback`` when there is no announcement state or the throw
        has not happened yet.
        """
        ann = self._announcement_states.get(ball_id)
        if ann is None:
            return fallback
        init_pos, init_vel, throw_time = ann
        dt = current_time - throw_time
        if dt < 0.0:
            return fallback
        return np.array([
            init_pos[0] + init_vel[0] * dt,
            init_pos[1] + init_vel[1] * dt,
            init_pos[2] + init_vel[2] * dt - 0.5 * GRAVITY_MMPS2 * dt * dt,
        ])

    def _expected_announced_velocity(self, ball_id: int, current_time: float) -> Optional[np.ndarray]:
        """Analytical velocity of an announced ball at ``current_time``; the
        announced ``v0`` itself before ``throw_time`` (no backward gravity)."""
        ann = self._announcement_states.get(ball_id)
        if ann is None:
            return None
        _, init_vel, throw_time = ann
        dt = max(current_time - throw_time, 0.0)
        return np.array([init_vel[0], init_vel[1],
                         init_vel[2] - GRAVITY_MMPS2 * dt])

    def _match_announced_balls(
        self,
        markers: List[np.ndarray],
        current_time: float,
        used: set,
    ) -> set:
        """Promote an ANNOUNCED ball to CONFIRMED from a marker that MOVES WITH
        the announced flight.

        Gate centre: the announcement's *analytical* ballistic prediction for
        ``current_time`` (the announced start before ``throw_time``) -- not the
        Kalman filter's position. The KF advanced by a fixed ``self.dt`` per
        call until 2026-09-18, and on any frame stream whose real period
        differs from ``dt`` its clock drifted away from wall clock (the
        2026-09-15 bag: ~186 Hz against 5 ms); the analytic centre cannot.

        Gate radius: ``announced_gate_mm + |v_pred| * announced_release_lead_s``
        -- a sphere, as before (2026-09-15: platform markers 204.6-220.7 mm
        from the cup sit in a +-200 mm box's corners), widened along the
        flight by the release-timing allowance (2026-10-14: the BB ball sat
        230-380 mm from the analytic position for its whole flight).

        Motion test (2026-10-14, see ``__init__``): each gated marker is
        continued from the nearest marker gated on the previous frame; the
        step must move at ``announced_speed_ratio`` x ``|v_pred|`` roughly
        along ``v_pred``, and only a candidate with
        ``announced_motion_steps`` consecutive passing steps can confirm --
        nearest to the gate centre among those. History is gathered from
        ``announced_release_lead_s`` before ``throw_time``, so a ball already
        moving at the announced instant (every JB self-toss: ~3.8 m/s at
        throw_time on 2026-10-14) confirms on the first in-flight frame,
        exactly as before.

        There is deliberately no height floor here (2026-09-15: the ball sat
        in the cup at 717-742 mm, under the old 880 mm floor, through the
        announced throw instant). Identity exclusion (:meth:`eligible_markers`)
        keeps the robot's own markers out; the motion test keeps every other
        static marker out.
        """
        lo, hi = self.announced_speed_ratio
        for ball_id, ball in self._balls.items():
            if ball.tracking != TrackingConfidence.ANNOUNCED:
                continue
            in_flight = ball.status == BallStatus.IN_FLIGHT
            pre_window = (ball.status == BallStatus.TO_BE_THROWN
                          and current_time >= ball.throw_time - self.announced_release_lead_s)
            if not (in_flight or pre_window):
                continue

            kf = self._filters.get(ball_id)
            if kf is None:
                continue

            ann = self._announcement_states.get(ball_id)
            predicted_pos = self._expected_announced_position(
                ball_id, current_time,
                fallback=(ann[0] if ann is not None else kf.position))
            v_pred = self._expected_announced_velocity(ball_id, current_time)
            if v_pred is None:
                v_pred = kf.velocity
            speed_pred = float(np.linalg.norm(v_pred))
            threshold = self.announced_gate_mm + speed_pred * self.announced_release_lead_s
            excluded = self._excluded_positions.get(ball_id, ())

            prev = self._candidate_history.get(ball_id)
            prev_dt = None
            if prev is not None:
                prev_dt = current_time - prev[0]
                # A gap of more than a few frames breaks "consecutive".
                if not (0.0 < prev_dt <= 4.0 * self.dt + 0.02):
                    prev, prev_dt = None, None

            gated: List[Tuple[np.ndarray, int]] = []
            best = None   # (dist, idx, pos, speed, streak)
            for idx, mpos in enumerate(markers):
                if idx in used:
                    continue
                dist = float(np.linalg.norm(mpos - predicted_pos))
                if dist > threshold:
                    continue
                if any(np.linalg.norm(mpos - e) < 20.0 for e in excluded):
                    continue
                streak, speed = 0, 0.0
                if prev is not None and prev[1]:
                    p_prev, s_prev = min(prev[1], key=lambda c: float(np.linalg.norm(mpos - c[0])))
                    step = mpos - p_prev
                    speed = float(np.linalg.norm(step)) / prev_dt
                    cos = (float(np.dot(step, v_pred)) / (float(np.linalg.norm(step)) * speed_pred)
                           if speed > 0.0 and speed_pred > 0.0 else -1.0)
                    if (lo * speed_pred <= speed <= hi * speed_pred
                            and cos >= self.announced_min_direction_cos):
                        streak = s_prev + 1
                gated.append((np.array(mpos, dtype=np.float64), streak))
                if (in_flight and streak >= self.announced_motion_steps
                        and (best is None or dist < best[0])):
                    best = (dist, idx, mpos, speed, streak)
            self._candidate_history[ball_id] = (float(current_time), gated)

            if best is not None:
                dist, best_idx, mpos, speed, streak = best
                # Match found — promote to CONFIRMED
                kf.update(markers[best_idx])
                used.add(best_idx)
                ball.tracking = TrackingConfidence.CONFIRMED
                ball.frames_tracked += 1
                ball.frames_without_measurement = 0
                ball.last_seen = current_time
                ball.position = kf.position
                ball.velocity = kf.velocity
                ball.timestamp = current_time
                self._update_landing_prediction(ball, from_filter=True)
                self._candidate_history.pop(ball_id, None)
                self._still_anchor[ball_id] = (float(current_time), np.array(mpos, dtype=np.float64))
                self._note_identity(
                    'confirm', ball_id, current_time, mpos, speed, speed_pred,
                    'steps %d, %.0f mm from expected (gate %.0f)' % (streak, dist, threshold))
                # Kill any parabolic marker tracks near this marker to
                # prevent duplicate detection of the same physical ball
                self._kill_tracks_near(markers[best_idx])
            elif in_flight:
                ball.frames_without_measurement += 1

        return used

    def _check_still_confirmed(self, ball_id: int, ball: Ball, current_time: float,
                               marker: np.ndarray) -> bool:
        """Demote a CONFIRMED announced ball whose raw marker has stood still
        while the announcement says it should be flying fast. Returns True if
        the ball was demoted (see ``__init__`` for the rule and why)."""
        if ball_id not in self._announcement_states:
            return False      # human-throw track: no announcement to compare
        anchor = self._still_anchor.get(ball_id)
        if anchor is None:
            self._still_anchor[ball_id] = (float(current_time), np.array(marker, dtype=np.float64))
            return False
        t_a, p_a = anchor
        span = current_time - t_a
        moved = float(np.linalg.norm(marker - p_a))
        if moved > self.demote_still_speed_mmps * span + 3.0:
            self._still_anchor[ball_id] = (float(current_time), np.array(marker, dtype=np.float64))
            return False
        if span <= self.demote_still_s:
            return False
        t_land = self._announced_landing_time.get(ball_id, 0.0)
        if not current_time < t_land - 0.1:
            return False
        v_pred = self._expected_announced_velocity(ball_id, current_time)
        speed_pred = float(np.linalg.norm(v_pred)) if v_pred is not None else 0.0
        if speed_pred <= self.demote_min_predicted_speed_mmps:
            return False
        # Release: back to ANNOUNCED, seeded from the analytic state again.
        p_pred = self._expected_announced_position(ball_id, current_time, fallback=marker)
        kf = self._filters[ball_id]
        kf.initialize_from_state(p_pred, v_pred, position_cov=500.0, velocity_cov=500.0)
        ball.tracking = TrackingConfidence.ANNOUNCED
        ball.position = kf.position
        ball.velocity = kf.velocity
        ball.timestamp = current_time
        self._flight_fits.pop(ball_id, None)
        self._still_anchor.pop(ball_id, None)
        self._candidate_history.pop(ball_id, None)
        self._excluded_positions.setdefault(ball_id, []).append(np.array(marker, dtype=np.float64))
        self._update_landing_prediction(ball)
        self._note_identity(
            'demote', ball_id, current_time, marker, moved / span if span > 0 else 0.0,
            speed_pred, 'still %.0f ms' % (span * 1e3))
        return True

    def _note_identity(self, kind: str, ball_id: int, current_time: float,
                       marker: np.ndarray, speed: float, speed_pred: float, extra: str):
        """One DEBUG line per confirmation / demotion (never per frame)."""
        ball = self._balls.get(ball_id)
        dt_throw = current_time - ball.throw_time if ball is not None else float('nan')
        ev = dict(kind=kind, ball_id=ball_id, t=float(current_time), dt_throw=float(dt_throw),
                  marker=np.array(marker, dtype=np.float64), speed=float(speed),
                  speed_pred=float(speed_pred))
        self.identity_events.append(ev)
        self._debug_log(
            'tracker %s ball %d (%s): marker (%.0f,%.0f,%.0f) at %.0f mm/s vs announced %.0f mm/s, '
            '%s, %+.3f s from throw_time' % (
                kind, ball_id, ball.source if ball is not None else '?',
                marker[0], marker[1], marker[2], speed, speed_pred, extra, dt_throw))

    # ------------------------------------------------------------------
    # Internal: Parabolic motion detection (human throw path)
    # ------------------------------------------------------------------

    def _detect_parabolic(
        self,
        markers: List[np.ndarray],
        current_time: float,
        used: set,
    ):
        """Buffer unmatched markers and detect parabolic motion."""
        remaining = [
            (idx, markers[idx]) for idx in range(len(markers))
            if idx not in used
        ]

        # Associate remaining markers with existing tracks (nearest-neighbour)
        matched_track_ids = set()
        for idx, mpos in remaining:
            best_track_id = None
            best_dist = float("inf")

            for track_id, track in self._marker_tracks.items():
                if track_id in matched_track_ids:
                    continue
                if not track.positions:
                    continue
                # Predict where this marker track should be
                dt = current_time - track.last_timestamp
                if dt <= 0 or dt > 0.1:  # Too old
                    continue
                dist = np.linalg.norm(mpos - track.last_position)
                if dist < self.gate_radius_mm * 2 and dist < best_dist:
                    best_dist = dist
                    best_track_id = track_id

            if best_track_id is not None:
                self._marker_tracks[best_track_id].add(mpos, current_time)
                matched_track_ids.add(best_track_id)
            else:
                # New track
                tid = self._next_track_id
                self._next_track_id += 1
                track = _MarkerTrack()
                track.add(mpos, current_time)
                self._marker_tracks[tid] = track

        # Check tracks for parabolic motion
        promote_ids = []
        for track_id, track in self._marker_tracks.items():
            if len(track) < self.parabolic_min_frames:
                continue
            if self._is_parabolic(track):
                promote_ids.append(track_id)

        for track_id in promote_ids:
            track = self._marker_tracks.pop(track_id)
            # Suppress duplicate: if this track's position is near an
            # existing ball (announced or already tracked), it's the same
            # physical ball — don't create a second Ball object.
            if self._is_near_existing_ball(track.last_position, current_time):
                continue
            self._promote_to_ball(track, current_time)

        # Prune stale tracks (no update for > 50ms)
        stale = [
            tid for tid, t in self._marker_tracks.items()
            if current_time - t.last_timestamp > 0.05
        ]
        for tid in stale:
            del self._marker_tracks[tid]

    def _is_parabolic(self, track: _MarkerTrack) -> bool:
        """Check if a marker track exhibits gravity-consistent acceleration."""
        if len(track) < 3:
            return False

        # Use the last 3-4 points
        positions = track.positions[-4:]
        timestamps = track.timestamps[-4:]

        if len(positions) < 3:
            return False

        # Compute acceleration via finite differences
        accels = []
        for i in range(1, len(positions) - 1):
            dt1 = timestamps[i] - timestamps[i - 1]
            dt2 = timestamps[i + 1] - timestamps[i]
            if dt1 <= 0 or dt2 <= 0:
                return False

            v1 = (positions[i] - positions[i - 1]) / dt1
            v2 = (positions[i + 1] - positions[i]) / dt2
            dt_avg = 0.5 * (dt1 + dt2)
            accel = (v2 - v1) / dt_avg
            accels.append(accel)

        if not accels:
            return False

        avg_accel = np.mean(accels, axis=0)
        expected = np.array([0.0, 0.0, -GRAVITY_MMPS2])
        residual = float(np.linalg.norm(avg_accel - expected))

        is_match = residual < self.parabolic_accel_threshold
        self.accel_residuals.append((residual, is_match))

        return is_match

    def _is_near_existing_ball(
        self, position: np.ndarray, current_time: float = 0.0,
    ) -> bool:
        """Check if a position is close to any existing in-flight ball.

        Used to suppress duplicate detections: if the parabolic detector
        finds a track near an already-tracked ball (e.g. an announced BB
        throw), we discard it rather than creating a second Ball.

        For ANNOUNCED balls, uses analytical ballistic prediction from the
        announcement's initial state — this is far more accurate than the
        KF, which accumulates process noise during predict-only steps.
        """
        threshold = self.match_threshold_base_mm  # ~100mm

        for ball_id, ball in self._balls.items():
            if ball.status not in (BallStatus.IN_FLIGHT, BallStatus.TO_BE_THROWN):
                continue

            # For CONFIRMED balls, use the filtered position
            if ball.tracking == TrackingConfidence.CONFIRMED:
                dist = np.linalg.norm(position - ball.position)
                if dist < threshold:
                    return True
                continue

            # For ANNOUNCED balls, compute analytical ballistic position
            # from the stored announcement initial state
            predicted = self._expected_announced_position(ball_id, current_time)
            if predicted is not None:
                dist = np.linalg.norm(position - predicted)
                if dist < threshold:
                    return True

        return False

    def _kill_tracks_near(self, position: np.ndarray):
        """Remove any marker tracks whose last position is near a given point.

        Called when an announced ball gets confirmed by a marker, to prevent
        stale tracks from later being promoted as duplicate parabolic detections.
        """
        to_remove = []
        for track_id, track in self._marker_tracks.items():
            if not track.positions:
                continue
            dist = np.linalg.norm(track.last_position - position)
            if dist < self.gate_radius_mm * 3:
                to_remove.append(track_id)
        for tid in to_remove:
            del self._marker_tracks[tid]

    def _promote_to_ball(self, track: _MarkerTrack, current_time: float):
        """Create a new Ball from a parabolic marker track."""
        ball_id = self._next_id
        self._next_id += 1

        # Initialize KF from the track positions
        kf = KalmanFilter(
            dt=self.dt,
            process_noise=self.process_noise,
            measurement_noise=self.measurement_noise,
        )
        kf.initialize_from_positions(track.positions, track.timestamps)

        ball = Ball(
            id=ball_id,
            status=BallStatus.IN_FLIGHT,
            tracking=TrackingConfidence.CONFIRMED,
            source="human_throw",
            position=kf.position,
            velocity=kf.velocity,
            timestamp=current_time,
            frames_tracked=len(track),
            last_seen=current_time,
        )

        self._update_landing_prediction(ball, from_filter=True)
        # Human throws have no assigned destination — the coordinator
        # only catches balls explicitly aimed at it (via BB announcements).
        self._balls[ball_id] = ball
        self._filters[ball_id] = kf

    # ------------------------------------------------------------------
    # Internal: Lifecycle transitions
    # ------------------------------------------------------------------

    def _check_lifecycle(self, current_time: float):
        """Transition in-flight balls to terminal states."""
        for ball in self._balls.values():
            if ball.status != BallStatus.IN_FLIGHT:
                continue

            # Check: past expected landing time + grace → CAUGHT or UNKNOWN
            if ball.landing_time > 0 and current_time > ball.landing_time + self.caught_grace_s:
                if ball.tracking == TrackingConfidence.CONFIRMED:
                    if ball.frames_without_measurement >= self.missed_frames_to_lose:
                        # Marker disappeared around landing time → CAUGHT
                        ball.status = BallStatus.CAUGHT
                        ball.terminal_since = current_time
                    # If marker still visible past landing: let it continue
                    # (it might be bouncing — will eventually hit lost-track)
                else:
                    # ANNOUNCED-only ball past landing: UNKNOWN
                    if ball.frames_without_measurement > self.max_frames_without_measurement:
                        ball.status = BallStatus.UNKNOWN
                        ball.terminal_since = current_time

            # Check: lost tracking for too long → UNKNOWN
            if ball.frames_without_measurement >= self.max_frames_without_measurement:
                if ball.status not in TERMINAL_STATUSES:
                    ball.status = BallStatus.UNKNOWN
                    ball.terminal_since = current_time

    def _cleanup_terminal(self, current_time: float):
        """Remove terminal balls past retention period."""
        to_remove = [
            ball_id for ball_id, ball in self._balls.items()
            if ball.status in TERMINAL_STATUSES
            and ball.terminal_since is not None
            and current_time - ball.terminal_since > self.terminal_retention_s
        ]
        for ball_id in to_remove:
            del self._balls[ball_id]
            self._filters.pop(ball_id, None)
            self._announcement_states.pop(ball_id, None)
            self._flight_fits.pop(ball_id, None)
            self._candidate_history.pop(ball_id, None)
            self._excluded_positions.pop(ball_id, None)
            self._still_anchor.pop(ball_id, None)
            self._announced_landing_time.pop(ball_id, None)

    # ------------------------------------------------------------------
    # Internal: Landing prediction
    # ------------------------------------------------------------------

    def _get_or_create_flight_fit(self, ball_id: int, ball: Ball) -> FlightFit:
        """Get this ball's gravity-fixed flight fit, creating it on first use.

        One `FlightFit` per ball id; a chained self-toss always gets a NEW
        ball id (`handle_announcement` — `_next_id` is monotonic and never
        reused), so a fresh id here always means a fresh fit.
        """
        fit = self._flight_fits.get(ball_id)
        if fit is None:
            fit = FlightFit(
                throw_time=ball.throw_time,
                landing_z=self.landing_z,
                min_samples=self.flight_fit_min_samples,
                residual_mm=self.flight_fit_residual_mm,
                freeze_above_plane_mm=self.flight_fit_freeze_above_plane_mm,
            )
            self._flight_fits[ball_id] = fit
        return fit

    def _update_landing_prediction(self, ball: Ball, from_filter: bool = False):
        """Recompute landing prediction from ball's current state.

        Prefers the gravity-fixed batch fit (`FlightFit.landing`, see
        `flight_fit.py`) whenever it has converged (enough surviving
        free-flight samples spanning enough time) — the Kalman filter's
        `landing_time` runs 0.06-0.20 s late and grows later through the
        descent, because a recursive filter is the wrong tool for a quantity
        gravity fixes exactly. Falls back to the KF/announcement ballistic
        extrapolation (the pre-2026-09-17 path) when the fit hasn't
        converged yet, e.g. early in a flight or for an ANNOUNCED-only ball
        with no mocap confirmation.

        `Ball.landing_from_fit` records which of the two paths answered — it
        is published, and the skill stack's learner refuses a row whose
        landing did not come from the fit (2026-09-18).

        The published `position`/`velocity` are untouched here — only the
        LANDING estimate changes.
        """
        fit = self._flight_fits.get(ball.id)
        if fit is not None:
            fit_landing = fit.landing(self.landing_z)
            if fit_landing is not None:
                landing_pos, landing_vel, t_abs = fit_landing
                ball.landing_position = landing_pos
                ball.landing_velocity = landing_vel
                ball.landing_time = t_abs
                ball.landing_from_fit = True
                return

        pos = ball.position
        vel = ball.velocity

        result = predict_landing_state(pos, vel, self.landing_z)
        if result is not None:
            landing_pos, landing_vel, ttl = result
            ball.landing_position = landing_pos
            ball.landing_velocity = landing_vel
            ball.landing_time = ball.timestamp + ttl
            ball.landing_from_fit = False
