"""Master event timeline for the juggling demo.

``MasterTimeline`` builds the absolute-wall-time schedule of every discrete
event the orchestrator dispatches: a single Ball-Butler priming throw,
followed by alternating hand-throw and hand-catch events for the
steady-state two-ball oval. All timestamps are absolute epoch wall-time on
the shared CAN clock (``0x7DD``) — never relative delays. ``due_events``
walks a cursor through the sorted schedule; each event is returned at
most once, in time order.

``abort(t_wall)`` truncates the future schedule and returns an
``ExitTransient``: one C2 quintic-Hermite segment from the platform state
at the abort instant to a stow pose at zero twist and zero acceleration.
The runtime player switches to it via ``TrajectoryPlayer.abort``.

Pure NumPy — no ROS2, no CasADi, no hardware. The plan reference is
``plans/archived/2026-08-15 bb-led-two-ball-juggle-demo.md`` §4 Phase 3.
"""
from __future__ import annotations

from typing import NamedTuple

import numpy as np

from controller.hermite import quintic_interp_with_accel
from sim.juggle_planner.pattern import JugglePattern
from sim.juggle_planner.trajectory import JuggleTrajectory


# Hand CAN `traj_type` values from the ``set_hand_traj_cmd`` service —
# the same constants the orchestrator emits on CAN ``0x6D0`` byte 0. See
# the plan §4 Phase 4 for the full encoding.
HAND_TRAJ_CATCH = 0
HAND_TRAJ_THROW = 1
HAND_TRAJ_NA = -1   # used for BB events (no hand stroke)

# Default exit-transient duration. One platform period is a reasonable
# default — long enough to ramp z down from active to stow within sensible
# velocity / acceleration bounds without dragging the abort out — but the
# constructor accepts an override for Phase 5 hardening.
DEFAULT_EXIT_DURATION_S = 1.0

# Default Ball-Butler priming lead time before ``t0``. The hardware BB
# rejects throws with less than ~100–300 ms lead (``StateMachine.cpp``);
# the priming ball's flight also has to fit in here, so the default
# lead is generous. Phase 3c sets the real value from BB geometry.
DEFAULT_BB_LEAD_S = 1.5


class Event(NamedTuple):
    """One scheduled event on the master timeline.

    Attributes
    ----------
    t_wall : float
        Absolute epoch wall-time, seconds, on the shared CAN ``0x7DD`` clock.
    kind : str
        ``'bb_throw'``, ``'hand_throw'``, or ``'hand_catch'``.
    traj_type : int
        Hand CAN ``traj_type`` (``HAND_TRAJ_THROW`` / ``HAND_TRAJ_CATCH``);
        ``HAND_TRAJ_NA`` for ``bb_throw``.
    event_vel : float
        Release speed (throws) or arrival speed (catches), m/s. Symmetric
        flight makes the two equal for steady-state events.
    ball : int
        Sim ball index this event acts on (``0`` or ``1``). The BB primes
        ball 0; Jugglebot starts holding ball 1.
    """
    t_wall: float
    kind: str
    traj_type: int
    event_vel: float
    ball: int


class ExitTransient:
    """C2 ramp from the periodic trajectory to the stow pose.

    A single quintic-Hermite segment keyed on **trajectory-relative time**
    ``t_rel`` so it composes with :meth:`TrajectoryPlayer.command_at`. At
    ``t_rel == t_rel_start`` the segment matches the periodic trajectory
    in pose, twist and acceleration (built that way by
    :func:`build_exit_transient`). At ``t_rel >= t_rel_end`` it returns
    ``(stow_pose, 0, 0)`` exactly: the quintic Hermite basis is zero at
    ``s == 1`` for every term except ``h3``, which scales the end pose.

    Outside the segment's window ``eval`` clamps — the runtime player can
    keep calling it past the end and will receive a steady stow command.
    """

    def __init__(self, t_rel_start: float, duration: float,
                 pose0: np.ndarray, twist0: np.ndarray, accel0: np.ndarray,
                 stow_pose: np.ndarray):
        if duration <= 0.0:
            raise ValueError("exit-transient duration must be positive")
        pose0 = np.asarray(pose0, dtype=float).copy()
        twist0 = np.asarray(twist0, dtype=float).copy()
        accel0 = np.asarray(accel0, dtype=float).copy()
        stow_pose = np.asarray(stow_pose, dtype=float).copy()
        for name, arr in (("pose0", pose0), ("twist0", twist0),
                          ("accel0", accel0), ("stow_pose", stow_pose)):
            if arr.shape != (6,):
                raise ValueError(f"{name} must be a 6-vector, got {arr.shape}")

        self.t_rel_start = float(t_rel_start)
        self.duration = float(duration)
        self.pose0 = pose0
        self.twist0 = twist0
        self.accel0 = accel0
        self.stow_pose = stow_pose

    @property
    def t_rel_end(self) -> float:
        return self.t_rel_start + self.duration

    def is_done(self, t_rel: float) -> bool:
        return float(t_rel) >= self.t_rel_end

    def eval(self, t_rel: float):
        """Evaluate at ``t_rel``; clamps outside ``[t_rel_start, t_rel_end]``."""
        local = float(t_rel) - self.t_rel_start
        local = max(0.0, min(local, self.duration))
        zero6 = np.zeros(6)
        return quintic_interp_with_accel(
            self.pose0, self.twist0, self.accel0,
            self.stow_pose, zero6, zero6,
            self.duration, local)


def build_exit_transient(trajectory: JuggleTrajectory, t_rel_abort: float,
                         duration: float = DEFAULT_EXIT_DURATION_S,
                         stow_pose: np.ndarray | None = None
                         ) -> ExitTransient:
    """Join an :class:`ExitTransient` onto ``trajectory`` at ``t_rel_abort``."""
    pose, twist, accel = trajectory.eval(t_rel_abort)
    stow = (np.zeros(6) if stow_pose is None
            else np.asarray(stow_pose, dtype=float))
    return ExitTransient(t_rel_abort, duration, pose, twist, accel, stow)


class MasterTimeline:
    """Absolute-wall-time event schedule for the BB-led two-ball juggle.

    The schedule lays out one BB priming throw, then ``n_catches`` hand
    catches interleaved with the hand throws that feed them. Trajectory-
    relative time and absolute wall-time are linked by ``t_wall = t0 +
    t_rel``, with ``t0`` the wall-time of the first ``hand_throw`` event.

    Event placement (``t_rel``)::

        hand_throw  at  k * P                    for k = 0 .. n_catches-1
        hand_catch  at  catch_offset + k * P     for k = 0 .. n_catches-1
        bb_throw    at  -bb_lead_s               (i.e. t_wall = t0 - bb_lead)

    where ``P = platform_period_s``. ``catch_offset_s`` defaults to
    ``P / 2`` — the CATCH point of the analytic oval. Phase 1's hand-tempo
    derivation places the catch instant at ``throw_stroke_s +
    transit_window_s`` (~0.43 s vs the analytic oval's 0.31 s); those
    differ because the analytic oval is the un-optimised baseline. Phase
    3c can override ``catch_offset_s`` to match the optimised trajectory
    once Phase 2's offline solve lands.

    Ball-index assignment alternates throws (``ball = (k + 1) % 2``):
    ball 0 is the BB-primed ball (caught first); ball 1 is the pre-held
    ball (thrown first at ``k=0``). Each catch picks up the ball thrown
    one platform-period earlier — except the first catch, which catches
    the BB-primed ball (``ball=0``).

    Pull events with :meth:`due_events`: cursor-based, in-order, each
    event returned at most once.
    """

    def __init__(self, pattern: JugglePattern, trajectory: JuggleTrajectory,
                 t0: float, *,
                 n_catches: int = 32,
                 bb_lead_s: float = DEFAULT_BB_LEAD_S,
                 catch_offset_s: float | None = None,
                 exit_duration_s: float = DEFAULT_EXIT_DURATION_S,
                 stow_pose: np.ndarray | None = None):
        if n_catches < 1:
            raise ValueError("n_catches must be >= 1")
        if bb_lead_s <= 0.0:
            raise ValueError("bb_lead_s must be positive")
        if exit_duration_s <= 0.0:
            raise ValueError("exit_duration_s must be positive")

        P = pattern.platform_period_s
        if catch_offset_s is None:
            catch_offset_s = 0.5 * P
        if not (0.0 < catch_offset_s < P):
            raise ValueError(
                f"catch_offset_s must lie in (0, platform_period_s={P:.4f}); "
                f"got {catch_offset_s:.4f}")

        self._pattern = pattern
        self._trajectory = trajectory
        self._t0 = float(t0)
        self._n_catches = int(n_catches)
        self._catch_offset_s = float(catch_offset_s)
        self._exit_duration_s = float(exit_duration_s)
        self._stow_pose = (np.zeros(6) if stow_pose is None
                           else np.asarray(stow_pose, dtype=float).copy())

        self._aborted = False
        self._abort_t_wall: float | None = None
        self._exit_transient: ExitTransient | None = None

        self._events: list[Event] = self._build_events(
            pattern, self._t0, self._n_catches, self._catch_offset_s,
            bb_lead_s)
        self._cursor = 0

    @staticmethod
    def _build_events(pattern: JugglePattern, t0: float, n_catches: int,
                      catch_offset_s: float,
                      bb_lead_s: float) -> list[Event]:
        P = pattern.platform_period_s
        v = pattern.throw_speed_mps

        # BB priming throw: issued ``bb_lead_s`` before t0 so the ball is
        # already in the air when the steady-state begins; it lands at
        # the first hand catch (k=0, ball 0).
        events: list[Event] = [Event(
            t_wall=t0 - bb_lead_s, kind='bb_throw',
            traj_type=HAND_TRAJ_NA, event_vel=v, ball=0)]

        for k in range(n_catches):
            # Throw event: ball 1 at k=0, then alternating.
            events.append(Event(
                t_wall=t0 + k * P, kind='hand_throw',
                traj_type=HAND_TRAJ_THROW, event_vel=v,
                ball=(k + 1) % 2))
            # Catch event: ball 0 at k=0 (the BB ball), then alternating.
            events.append(Event(
                t_wall=t0 + catch_offset_s + k * P, kind='hand_catch',
                traj_type=HAND_TRAJ_CATCH, event_vel=v,
                ball=k % 2))

        events.sort(key=lambda e: e.t_wall)
        return events

    # ---- API ---------------------------------------------------------

    @property
    def t0(self) -> float:
        return self._t0

    @property
    def n_events(self) -> int:
        return len(self._events)

    @property
    def n_catches(self) -> int:
        return self._n_catches

    @property
    def catch_offset_s(self) -> float:
        return self._catch_offset_s

    @property
    def aborted(self) -> bool:
        return self._aborted

    @property
    def exit_transient(self) -> ExitTransient | None:
        return self._exit_transient

    def all_events(self) -> list[Event]:
        """Return a copy of the (possibly post-abort) event list."""
        return list(self._events)

    def due_events(self, t_wall: float) -> list[Event]:
        """Events with ``t_wall <= t_wall`` not yet returned, in time order.

        Cursor-based: each event is returned at most once. Calling with a
        monotonically-increasing ``t_wall`` (the natural runtime usage)
        walks the cursor forward.
        """
        due: list[Event] = []
        while (self._cursor < len(self._events)
               and self._events[self._cursor].t_wall <= float(t_wall)):
            due.append(self._events[self._cursor])
            self._cursor += 1
        return due

    def abort(self, t_wall: float) -> ExitTransient:
        """Truncate the schedule at ``t_wall`` and return the exit transient.

        Already-dispatched events (everything before the cursor) are
        preserved verbatim regardless of their timestamps. Un-returned
        events at or beyond ``t_wall`` are dropped from the tail;
        un-returned events strictly before ``t_wall`` are kept and remain
        dispatchable. Idempotent — subsequent calls return the same
        :class:`ExitTransient` (the second call's ``t_wall`` is ignored).
        """
        if self._aborted:
            assert self._exit_transient is not None
            return self._exit_transient

        # Keep head (already returned) verbatim; filter tail (not yet
        # returned) to drop events at or beyond the abort instant.
        head = self._events[:self._cursor]
        tail = [e for e in self._events[self._cursor:]
                if e.t_wall < float(t_wall)]
        self._events = head + tail
        # cursor stays — it points to the first un-returned event in the new tail

        self._aborted = True
        self._abort_t_wall = float(t_wall)
        t_rel_abort = float(t_wall) - self._t0
        self._exit_transient = build_exit_transient(
            self._trajectory, t_rel_abort,
            duration=self._exit_duration_s, stow_pose=self._stow_pose)
        return self._exit_transient
