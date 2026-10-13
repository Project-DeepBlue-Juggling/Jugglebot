"""Pure-Python motion primitives shared by the simulation and the hardware stack.

What lives here: ballistics, quintic/Hermite interpolation, reference
closed-form feasibility bounds, the plant interface and
telemetry. No ROS2 imports; no CasADi.

The MPC controller (``mpc.py``, ``params.py``, ``runner.py``,
``hardware_plant.py``, ``hardware_hooks.py``, ``hot_loop_contract.py``,
``generate_solver.py``) was removed 2026-09-01 — dormant since 2026-08-01

The final implementation is preserved at git tag ``mpc-final``; see
``logbook/2026-09-01-mpc-chain-removed.md``.

``scheduler``, ``zmq_target``, ``toss_motion_source`` and
``catch_optimizer`` were deleted 2026-09-09 (R0 dead-layer deletion,
``plans/archived/two-ball-skill-stack.md`` § 6) — MPC-era sim sources with no
non-test importer left after the MPC chain removal. ``target`` (``ReferenceEvent``), the K1-K6 enforcement point
(``make_feasible_events``) and ``REFERENCE_LAYER_CONTRACT.md`` were retired
2026-10-14 (owner: nothing live called them).
"""

from .hermite import quintic_interp, quintic_interp_with_accel, quintic_jerk_integral
from .plant import PlantInterface, PlantState
from .telemetry import TelemetryLogger, StepRecord, record_from_arrays
