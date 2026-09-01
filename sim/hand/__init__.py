"""Hand coordination: dynamic target tracking, hand priming, ball lifecycle.

History: ``FeasibilityChecker`` (a coarse-horizon MPC reachability check)
was removed 2026-09-01 with the rest of the MPC chain — dormant since
2026-08-01 and superseded by the unified 7-DoF planner
(``plans/active/unified-7dof-planner.md``) as the per-cycle replanner.
The final implementation lives at git tag ``mpc-final``; see
``logbook/2026-09-01-mpc-chain-removed.md``.  The ``feasibility_checker``
parameters left on the coordinators are now always ``None``.
"""

from .coordinator import HandCoordinator, DynamicTarget, HandEvent, BallSpawn, BallRelease, HandPhase
from .trajectory import (
    HandCatchTrajectory, HandSmoothMove, HandCatchSequence,
    HandThrowTrajectory, HandThrowSequence, max_throw_speed_mps,
)

__all__ = [
    'HandCoordinator', 'DynamicTarget', 'HandEvent', 'BallSpawn',
    'BallRelease', 'HandPhase',
    'HandCatchTrajectory', 'HandSmoothMove', 'HandCatchSequence',
    'HandThrowTrajectory', 'HandThrowSequence', 'max_throw_speed_mps',
]
