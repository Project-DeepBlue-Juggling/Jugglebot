"""Pure-Python motion primitives shared by the simulation and the hardware stack.

What lives here: ballistics, quintic/Hermite interpolation, reference
feasibility (K1–K6), the plant interface, target sources, and telemetry.
No ROS2 imports; no CasADi.

The MPC controller (``mpc.py``, ``params.py``, ``runner.py``,
``hardware_plant.py``, ``hardware_hooks.py``, ``hot_loop_contract.py``,
``generate_solver.py``) was removed 2026-09-01 — dormant since 2026-08-01
and superseded by the unified 7-DoF planner as the per-cycle replanner.
The final implementation is preserved at git tag ``mpc-final``; see
``logbook/2026-09-01-mpc-chain-removed.md``.

``scheduler``, ``zmq_target``, ``toss_motion_source`` and
``catch_optimizer`` were deleted 2026-09-09 (R0 dead-layer deletion,
``plans/active/two-ball-skill-stack.md`` § 6) — MPC-era sim sources with no
non-test importer left after the MPC chain removal. ``target`` is retained:
``sim/input/toss_loop.py`` (kept — imported by ``sim/viz/reference_plot.py``)
still imports ``ReferenceEvent`` from it.
"""

from .hermite import quintic_interp, quintic_interp_with_accel, quintic_jerk_integral
from .plant import PlantInterface, PlantState
from .target import (
    ReferenceEvent, TargetCommand, TargetSource, sample_ref_fn,
    StaticTargetSource, WaypointTargetSource,
)
from .telemetry import TelemetryLogger, StepRecord, record_from_arrays
