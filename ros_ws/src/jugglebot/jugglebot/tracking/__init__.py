"""Ball tracking subpackage — pure Python, no ROS2 dependency.

Provides Kalman-filtered ball tracking, ballistic landing prediction,
marker-to-ball matching, and ball lifecycle management.
"""

from .ball import Ball, BallStatus, TrackingConfidence
from .kalman import KalmanFilter
from .ballistics import predict_landing_state
from .matcher import BallTracker

__all__ = [
    "Ball",
    "BallStatus",
    "TrackingConfidence",
    "KalmanFilter",
    "predict_landing_state",
    "BallTracker",
]
