"""Abstract plant interface and state dataclass.

The PlantInterface defines the contract between a controller and the physical
or simulated plant. The one live implementation is ``MuJoCoPlant``, which steps
a MuJoCo simulation (``sim/plant/mujoco_plant.py``).

``HardwarePlant`` — the ZMQ/CAN implementation that spoke to ``motor_guard`` —
was removed 2026-09-01 with the rest of the MPC chain (dormant since
2026-08-01, superseded by the unified 7-DoF planner as the per-cycle
replanner). It is preserved at git tag ``mpc-final``; see
``logbook/2026-09-01-mpc-chain-removed.md``. The contract below is unchanged
and still binds any future implementation.

The normative invariants P1 (PlantState aliasing), P2 (can_reset capability),
P3 (trusted-callee command boundary), and P4 (control_dt awareness) are
specified in
[controller/PLANT_INTERFACE_CONTRACT.md](PLANT_INTERFACE_CONTRACT.md).
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np


@dataclass
class PlantState:
    """Snapshot of the plant state at a single instant.

    All quantities use the same conventions as the production motion code:
      - Positions in mm, velocities in mm/s, rotations as rotation vectors (rad).
      - Platform pose is an *offset* from STOW height ([0, 0, initial_height_mm]).
    """
    leg_extensions_mm: np.ndarray     # (6,) STOW-relative: 0 = STOW, ~154.5 = Active, 280 = full
    leg_velocities_mmps: np.ndarray   # (6,) actual leg velocities
    platform_pos_mm: np.ndarray       # (3,) [x, y, z] offset from STOW height
    platform_rot: np.ndarray          # (3,) rotation vector (rad)
    platform_twist: np.ndarray        # (6,) [vx, vy, vz, wx, wy, wz]
    time: float                       # simulation time (s)
    # Hand fields — None when model has no hand (backward-compatible)
    hand_pos_mm: float | None = None      # Hand linear position (mm from bottom of travel)
    hand_vel_mmps: float | None = None    # Hand linear velocity (mm/s)
    # Telemetry freshness — None for sim (always fresh), seconds for hardware
    data_age_s: float | None = None


class PlantInterface(ABC):
    """Abstract interface for controlling a Stewart platform plant.

    See controller/PLANT_INTERFACE_CONTRACT.md for the normative P1–P4
    specification.
    """

    @property
    @abstractmethod
    def can_reset(self) -> bool:
        """P2: True iff ``reset()`` is supported on this implementation.

        Implementations MUST declare this as a class attribute or
        property.  When False, ``reset()`` MUST raise
        ``NotImplementedError`` rather than silently no-op.  Callers
        that may run against either implementation MUST guard with
        ``if plant.can_reset: plant.reset(...)``.

        See controller/PLANT_INTERFACE_CONTRACT.md P2.
        """

    @property
    @abstractmethod
    def control_dt(self) -> float:
        """P4: configured control period in seconds.

        Implementations MUST accept ``control_dt: float`` as a
        constructor keyword argument (default 0.025 s = 40 Hz preserves
        current behaviour).  Internal time-window thresholds — telemetry
        staleness watchdogs, dead-band re-arm intervals, deadlock
        deadlines — MUST derive from ``self._control_dt`` rather than
        hard-coded magic numbers.  External consumers (the runner, the
        scheduler's S1 ``τ_grace`` default) read the canonical period
        from this property.

        See controller/PLANT_INTERFACE_CONTRACT.md P4.
        """

    @abstractmethod
    def command(self, leg_extensions_mm: np.ndarray,
                vel_mm_s: np.ndarray | None = None,
                cmd_next_mm: np.ndarray | None = None,
                cmd_next2_mm: np.ndarray | None = None) -> None:
        """Send 6 leg extension commands (mm, IK convention).

        P3 contract: ``command()`` is a *trusted-callee* boundary.
        Callers MUST guarantee finite, correctly-shaped, in-range
        inputs.  Implementations MUST NOT silently coerce, clip,
        default, or sanitise malformed inputs (silent ``np.clip`` is
        a contract violation).  Implementations MAY raise
        ``ValueError`` on bad inputs (defensive), or MAY pass them
        through to the layer below (trusting); silent correction is
        prohibited because it hides the upstream caller bug.  See
        controller/PLANT_INTERFACE_CONTRACT.md P3.

        Parameters
        ----------
        leg_extensions_mm : (6,) ndarray — STOW-relative extensions in mm
        vel_mm_s : (6,) ndarray or None — forward-looking command velocity
            (mm/s).  When provided by the MPC, this is preferred over the
            backward-difference computed from consecutive commands.
        cmd_next_mm : (6,) ndarray or None — MPC's predicted next-step
            command (u[1]).  When provided, the motor guard interpolates
            between the current command and this waypoint instead of
            extrapolating.  None falls back to Taylor extrapolation.
        cmd_next2_mm : (6,) ndarray or None — MPC's predicted step-after-
            next command (u[2]).  Used with cmd_next_mm to make Hermite
            interpolation C1-continuous across segment boundaries.
        """

    @abstractmethod
    def get_state(self) -> PlantState:
        """Read current platform state.

        P1 contract: implementations MUST return the same
        ``PlantState`` instance on every call.  Each ndarray field MUST
        be the same array object across calls — fields are mutated in
        place via ``np.copyto`` / slice assignment, not rebound.
        Consumers MUST NOT retain ``PlantState`` references across
        ticks: the next ``get_state()`` call mutates the fields they
        hold.  See controller/PLANT_INTERFACE_CONTRACT.md P1.
        """

    @abstractmethod
    def step(self, dt: float) -> None:
        """Advance simulation by dt seconds (no-op for hardware)."""

    @abstractmethod
    def reset(self, pose_6dof: np.ndarray | None = None) -> None:
        """Reset to home (pose_6dof=None) or a specified [x,y,z,rx,ry,rz] pose.

        P2 contract: when ``can_reset is True``, executes the documented
        reset.  When ``can_reset is False``, MUST raise
        ``NotImplementedError`` with a message naming the implementation
        and pointing at the correct lifecycle API.  See
        controller/PLANT_INTERFACE_CONTRACT.md P2.
        """
