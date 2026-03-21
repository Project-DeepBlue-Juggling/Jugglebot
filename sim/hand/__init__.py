"""Hand coordination: dynamic target tracking, hand priming, ball lifecycle."""

from .coordinator import HandCoordinator, DynamicTarget, HandEvent, BallSpawn, BallRelease, HandPhase
from .feasibility import FeasibilityChecker
from .trajectory import (
    HandCatchTrajectory, HandSmoothMove, HandCatchSequence,
    HandThrowTrajectory, HandThrowSequence, max_throw_speed_mps,
)

__all__ = [
    'HandCoordinator', 'DynamicTarget', 'HandEvent', 'BallSpawn',
    'BallRelease', 'HandPhase',
    'FeasibilityChecker',
    'HandCatchTrajectory', 'HandSmoothMove', 'HandCatchSequence',
    'HandThrowTrajectory', 'HandThrowSequence', 'max_throw_speed_mps',
]
