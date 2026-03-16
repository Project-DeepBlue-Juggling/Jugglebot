"""Abstract plant interface and state dataclass.

The PlantInterface defines the contract between the controller (MPC) and the
physical or simulated plant. Implementations:
  - MuJoCoPlant: steps a MuJoCo simulation (Phase 1)
  - HardwarePlant: sends commands via CAN to real ODrives (Phase 6)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np


@dataclass
class PlantState:
    """Snapshot of the plant state at a single instant.

    All quantities use the same conventions as the production motion code:
      - Positions in mm, velocities in mm/s, rotations as rotation vectors (rad).
      - Platform pose is an *offset* from home ([0, 0, initial_height_mm]).
    """
    leg_extensions_mm: np.ndarray     # (6,) actual leg positions (IK convention)
    leg_velocities_mmps: np.ndarray   # (6,) actual leg velocities
    platform_pos_mm: np.ndarray       # (3,) [x, y, z] offset from home
    platform_rot: np.ndarray          # (3,) rotation vector (rad)
    platform_twist: np.ndarray        # (6,) [vx, vy, vz, wx, wy, wz]
    time: float                       # simulation time (s)


class PlantInterface(ABC):
    """Abstract interface for controlling a Stewart platform plant."""

    @abstractmethod
    def command(self, leg_extensions_mm: np.ndarray) -> None:
        """Send 6 leg extension commands (mm, IK convention)."""

    @abstractmethod
    def get_state(self) -> PlantState:
        """Read current platform state."""

    @abstractmethod
    def step(self, dt: float) -> None:
        """Advance simulation by dt seconds (no-op for hardware)."""

    @abstractmethod
    def reset(self, pose_6dof: np.ndarray | None = None) -> None:
        """Reset to home (pose_6dof=None) or a specified [x,y,z,rx,ry,rz] pose."""
