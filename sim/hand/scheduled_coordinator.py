"""Scheduled coordinator: bridges EventScheduler with hand management.

The EventScheduler owns platform target computation and event sequencing.
This adapter adds hand control (priming, catch/throw sequences, ball
spawning, capture notification) on top.

Usage::

    from controller.scheduler import EventScheduler, ScheduledEvent, EventType
    from hand.scheduled_coordinator import ScheduledCoordinator

    scheduler = EventScheduler(active_pose, cumulative_times)
    coord = ScheduledCoordinator(scheduler, feasibility_checker, plant)

    # Submit a catch event (from ball tracker / catch policy)
    coord.submit_catch(catch_pose, arrival_time, event_vel_mps, ball_spawn)

    # Each control step:
    tc = coord.update(sim_time, current_pose, current_twist, hand_pos_mm)
    # tc is a TargetCommand with .hand_cmd and .ball_spawn attributes

    # When ball is captured:
    coord.notify_capture(sim_time)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np

import os, sys
_sim_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)
_repo_root = os.path.dirname(_sim_dir)
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

from controller.scheduler import (
    EventScheduler, EventType, HandNotification, ScheduledEvent,
    SchedulerOutput, SchedulerPhase, _next_event_id,
)
from controller.target import TargetCommand as BaseTargetCommand
from hand.coordinator import BallSpawn
from hand.trajectory import HandCatchSequence

logger = logging.getLogger(__name__)

# Timing constants
_SETTLE_MARGIN_S = 0.1  # platform arrives early by this amount


class _HandState:
    """Tracks hand state across the catch lifecycle."""

    def __init__(self):
        self.primed = False
        self.catch_seq_sent = False
        self.active_catch_seq: HandCatchSequence | None = None
        self.captured = False
        self.home_pending = False
        self.pending_ball_spawn: BallSpawn | None = None
        self.ball_spawned = False
        # Catch params (sequence built lazily when hand_pos is available)
        self.catch_vel_mps: float | None = None
        self.catch_arrival_time: float | None = None


class ScheduledCoordinator:
    """Bridges EventScheduler (platform) with hand management (sim).

    Parameters
    ----------
    scheduler : EventScheduler
        The event scheduler for platform motion.
    feasibility_checker : optional
        Feasibility checker with ``check(state, pose, time, twist)``.
        If provided, events are checked before submission.
    """

    def __init__(
        self,
        scheduler: EventScheduler,
        feasibility_checker=None,
    ) -> None:
        self._scheduler = scheduler
        self._feasibility = feasibility_checker
        self._hand = _HandState()
        self._events_log: list[dict] = []
        self._active_event_id: int | None = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def scheduler(self) -> EventScheduler:
        return self._scheduler

    @property
    def phase(self) -> SchedulerPhase:
        return self._scheduler.phase

    @property
    def events_log(self) -> list[dict]:
        return self._events_log

    # ------------------------------------------------------------------
    # Event submission
    # ------------------------------------------------------------------

    def submit_catch(
        self,
        catch_pose: np.ndarray,
        arrival_time: float,
        event_vel_mps: float | None = None,
        ball_spawn: BallSpawn | None = None,
        ball_id: int | None = None,
    ) -> ScheduledEvent:
        """Submit a catch event.

        Parameters
        ----------
        catch_pose : (6,) platform pose for catch.
        arrival_time : absolute time ball arrives.
        event_vel_mps : ball landing speed (m/s) for hand trajectory.
        ball_spawn : ball spawn parameters (position, velocity, time).
        ball_id : optional ball ID for tracking.

        Returns
        -------
        The ScheduledEvent that was submitted.
        """
        event = ScheduledEvent(
            event_id=_next_event_id(),
            event_type=EventType.CATCH,
            pose=np.asarray(catch_pose, dtype=np.float64),
            time=arrival_time - _SETTLE_MARGIN_S,
            twist=None,  # stationary catch
            hand_vel_mps=event_vel_mps,
            metadata={'ball_id': ball_id},
        )

        self._scheduler.submit_event(event)
        self._active_event_id = event.event_id

        # Store ball spawn for later
        if ball_spawn is not None:
            self._hand.pending_ball_spawn = ball_spawn
            self._hand.ball_spawned = False

        # Reset hand state for new catch
        self._hand.primed = False
        self._hand.catch_seq_sent = False
        self._hand.active_catch_seq = None
        self._hand.captured = False
        self._hand.home_pending = False

        # Store catch params; sequence built lazily when hand_pos is known
        self._hand.catch_vel_mps = event_vel_mps
        self._hand.catch_arrival_time = arrival_time

        self._events_log.append({
            'event_id': event.event_id,
            'type': 'catch_submitted',
            'pose': catch_pose.copy(),
            'arrival_time': arrival_time,
        })

        return event

    def update_catch(
        self,
        catch_pose: np.ndarray,
        arrival_time: float,
        event_vel_mps: float | None = None,
    ) -> None:
        """Refine the current catch event (continuous ball tracking).

        Updates both the scheduler event and the hand catch sequence.
        """
        if self._active_event_id is None:
            logger.warning("update_catch called with no active event")
            return

        event = ScheduledEvent(
            event_id=self._active_event_id,
            event_type=EventType.CATCH,
            pose=np.asarray(catch_pose, dtype=np.float64),
            time=arrival_time - _SETTLE_MARGIN_S,
            twist=None,
            hand_vel_mps=event_vel_mps,
        )
        self._scheduler.update_current_event(event)

        # Update hand catch params (sequence rebuilt lazily if not yet sent)
        if event_vel_mps is not None and not self._hand.catch_seq_sent:
            self._hand.catch_vel_mps = event_vel_mps
            self._hand.catch_arrival_time = arrival_time
            self._hand.active_catch_seq = None  # force rebuild

    # ------------------------------------------------------------------
    # Capture notification
    # ------------------------------------------------------------------

    def notify_capture(self, sim_time: float) -> None:
        """Signal that the ball has been captured."""
        self._hand.captured = True
        self._events_log.append({
            'event_id': self._active_event_id,
            'type': 'captured',
            'time': sim_time,
        })
        logger.info("ScheduledCoordinator: ball captured at t=%.3f", sim_time)

    # ------------------------------------------------------------------
    # Per-step update
    # ------------------------------------------------------------------

    def update(
        self,
        sim_time: float,
        current_pose: np.ndarray,
        current_twist: np.ndarray,
        hand_pos_mm: float = 0.0,
    ) -> tuple[BaseTargetCommand, object, BallSpawn | None]:
        """Advance coordinator and return MPC target + hand + ball commands.

        Returns
        -------
        target_command : TargetCommand
            MPC target for this step (target_pose, arrival_time, etc.).
        hand_cmd : str, float, HandCatchSequence, or None
            Hand command for the main loop to process.
        ball_spawn : BallSpawn or None
            Ball to spawn this step, or None.
        """
        # Advance scheduler
        sched_out = self._scheduler.update(sim_time, current_pose, current_twist)

        # Process hand notifications from scheduler
        hand_cmd = self._process_hand(sim_time, sched_out, hand_pos_mm)

        # Ball spawning
        ball_spawn = self._process_ball_spawn(sim_time)

        return sched_out.target_command, hand_cmd, ball_spawn

    # ------------------------------------------------------------------
    # Internal: hand management
    # ------------------------------------------------------------------

    def _process_hand(
        self,
        sim_time: float,
        sched_out: SchedulerOutput,
        hand_pos_mm: float,
    ) -> object:
        """Determine hand command based on scheduler phase and notifications."""
        hand_cmd = None
        notification = sched_out.hand_notification

        # Prime hand when we start approaching a catch
        if notification is not None:
            if (notification.notification_type == 'approaching'
                    and notification.event.event_type == EventType.CATCH
                    and not self._hand.primed):
                hand_cmd = 'prime'
                self._hand.primed = True

        # Build catch sequence lazily once primed (need hand_pos_mm)
        if (self._hand.primed
                and self._hand.active_catch_seq is None
                and self._hand.catch_vel_mps is not None
                and self._hand.catch_arrival_time is not None):
            self._hand.active_catch_seq = HandCatchSequence(
                event_vel_mps=self._hand.catch_vel_mps,
                arrival_time=self._hand.catch_arrival_time,
                current_pos_mm=hand_pos_mm,
            )

        # Send catch sequence once during approach (not same step as prime)
        if (hand_cmd is None
                and sched_out.phase == SchedulerPhase.APPROACHING
                and not self._hand.catch_seq_sent
                and self._hand.active_catch_seq is not None
                and self._hand.primed):
            hand_cmd = self._hand.active_catch_seq
            self._hand.catch_seq_sent = True

        # Handle arrival at catch event
        if notification is not None and notification.notification_type == 'arrived':
            if notification.event.event_type == EventType.CATCH:
                self._events_log.append({
                    'event_id': notification.event.event_id,
                    'type': 'arrived',
                    'time': sim_time,
                })

        # Handle capture: retract hand, begin return
        if (sched_out.phase == SchedulerPhase.HOLDING
                and self._hand.captured
                and not self._hand.home_pending):
            hand_cmd = 'home'
            self._hand.home_pending = True
            # Begin return to active pose
            self._scheduler.begin_return()

        return hand_cmd

    def _process_ball_spawn(self, sim_time: float) -> BallSpawn | None:
        """Check if it's time to spawn the ball."""
        spawn = self._hand.pending_ball_spawn
        if spawn is None or self._hand.ball_spawned:
            return None

        if sim_time >= spawn.spawn_time:
            self._hand.ball_spawned = True
            return spawn

        return None

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reset coordinator and scheduler for a new sequence."""
        self._scheduler.clear()
        self._hand = _HandState()
        self._active_event_id = None
