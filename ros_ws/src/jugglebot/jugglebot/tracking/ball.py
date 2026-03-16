"""Ball data model and lifecycle enums.

Pure Python — no ROS2 dependency.
"""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

import numpy as np


class BallStatus(IntEnum):
    """Lifecycle state of a tracked ball."""
    TO_BE_THROWN = 0  # Announced by thrower, not yet thrown
    IN_FLIGHT = 1     # Ball is airborne
    CAUGHT = 2        # Ball landed on platform
    DROPPED = 3       # Ball missed / bounced off
    UNKNOWN = 4       # Lost track, no resolution


class TrackingConfidence(IntEnum):
    """Whether mocap has confirmed this ball."""
    ANNOUNCED = 0     # Only from throw announcement, no mocap confirmation yet
    CONFIRMED = 1     # Matched to mocap marker, Kalman filter is being updated


TERMINAL_STATUSES = frozenset({BallStatus.CAUGHT, BallStatus.DROPPED, BallStatus.UNKNOWN})


@dataclass
class Ball:
    """A single tracked ball.

    Fields split into three groups:
      - Identity & status (published)
      - Current state (Kalman-filtered if CONFIRMED)
      - Predicted landing state (ballistic projection)
      - Internal tracking state (not published in ROS2 messages)
    """
    # Identity
    id: int                                  # Monotonic per session, never reused
    status: BallStatus                       # Current lifecycle state
    tracking: TrackingConfidence             # Whether mocap has confirmed this ball
    source: str                              # "ball_butler" or "human_throw"
    destination: str = ""                    # Robot name, empty = unassigned

    # Current state (Kalman-filtered if CONFIRMED, predicted from announcement if ANNOUNCED)
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))   # [x,y,z] mm, base frame
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))   # [vx,vy,vz] mm/s
    timestamp: float = 0.0                   # Time of last state update (seconds)

    # Predicted landing state (ballistic projection to landing_z plane)
    landing_position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [x,y,z] mm
    landing_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [vx,vy,vz] mm/s
    landing_time: float = 0.0               # Absolute time (seconds)

    # Internal tracking state (not published)
    frames_tracked: int = 0                  # Number of mocap frames matched
    frames_without_measurement: int = 0      # Consecutive missed frames
    last_seen: float = 0.0                   # Time of last mocap marker match
    throw_time: float = 0.0                  # Expected throw time (from announcement)
    terminal_since: Optional[float] = None   # Time when ball entered terminal state
