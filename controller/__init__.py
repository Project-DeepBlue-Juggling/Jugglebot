"""Pure-Python motion primitives shared by the simulation and the hardware stack.

What lives here: ballistics, catch-pose optimisation, quintic/Hermite
interpolation, reference feasibility (K1–K6), the event scheduler, the
plant interface, target sources, telemetry, and the ZMQ target ingest.
No ROS2 imports; no CasADi.

The MPC controller (``mpc.py``, ``params.py``, ``runner.py``,
``hardware_plant.py``, ``hardware_hooks.py``, ``hot_loop_contract.py``,
``generate_solver.py``) was removed 2026-09-01 — dormant since 2026-08-01
and superseded by the unified 7-DoF planner as the per-cycle replanner.
The final implementation is preserved at git tag ``mpc-final``; see
``logbook/2026-09-01-mpc-chain-removed.md``.
"""

from .catch_optimizer import CatchHeightOptimizer, compute_catch_orientation, compute_catch_pose
from .hermite import quintic_interp, quintic_interp_with_accel, quintic_jerk_integral
from .scheduler import (
    EventScheduler, EventType, HandNotification, ScheduledEvent,
    SchedulerOutput, SchedulerPhase,
)
from .plant import PlantInterface, PlantState
from .target import (
    ReferenceEvent, TargetCommand, TargetSource, sample_ref_fn,
    StaticTargetSource, WaypointTargetSource,
)
from .telemetry import TelemetryLogger, StepRecord, record_from_arrays
from .toss_motion_source import TossMotionSource
