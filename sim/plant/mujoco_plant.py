"""MuJoCo simulation plant for the Jugglebot Stewart platform.

Wraps a MuJoCo model + data pair behind the PlantInterface ABC.
Handles coordinate conversions between home-relative extensions (mm) and
MuJoCo slide joint values (m), and extracts platform state from MuJoCo sensors.

Coordinate convention:
    The public interface uses **home-relative extensions** where extension=0
    means "at home position" — matching the real robot's encoder convention.
    Internally, MuJoCo slide=0 also means home, but the IK model uses
    init_leg_lengths_mm (which excludes ball_joint_offset_mm) as its zero
    reference.  This plant handles the offset transparently.
"""

from __future__ import annotations

import logging
import os
import sys

import numpy as np
import mujoco

# Allow importing from the production motion package
_repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot'))
sys.path.insert(0, os.path.join(_repo_root, 'config', 'generated'))

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    pose_to_leg_lengths,
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
)

from .interface import PlantInterface, PlantState

logger = logging.getLogger(__name__)

# Default MJCF model path
_DEFAULT_MODEL_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'model', 'jugglebot.xml'
)


class MuJoCoPlant(PlantInterface):
    """MuJoCo-backed Stewart platform simulation.

    Parameters
    ----------
    model_path : str | None
        Path to the MJCF XML.  Defaults to ``sim/model/jugglebot.xml``.
    geom : StewartGeometry | None
        Stewart geometry instance.  Created automatically if not supplied.
    """

    def __init__(
        self,
        model_path: str | None = None,
        geom: StewartGeometry | None = None,
    ):
        if model_path is None:
            model_path = os.path.abspath(_DEFAULT_MODEL_PATH)

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)
        self._geom = geom or StewartGeometry()

        # Pre-compute geometric home leg lengths (m) — needed for slide ↔ extension conversion
        base_m = self._geom.base_nodes / 1000.0
        plat_m = self._geom.plat_nodes / 1000.0
        height_m = self._geom.init_height_mm / 1000.0
        plat_world_m = plat_m + np.array([0.0, 0.0, height_m])
        self._geom_home_lengths_m = np.linalg.norm(plat_world_m - base_m, axis=1)

        # Home offset: IK extensions at MuJoCo home (slide=0).
        # IK measures from init_leg_lengths_mm (excludes ball_joint_offset),
        # so at geometric home the IK reports ~27-30 mm of extension.
        # We subtract this offset so the public interface sees extension=0 at home.
        self._home_extensions_mm = (
            self._geom_home_lengths_m * 1000.0 - self._geom.init_leg_lengths_mm
        )

        # Cache sensor addresses for fast reads
        self._sensor_adr: dict[str, tuple[int, int]] = {}
        sensor_names = (
            [f'slide_pos_{i}' for i in range(6)]
            + [f'slide_vel_{i}' for i in range(6)]
            + ['platform_pos', 'platform_quat', 'platform_linvel', 'platform_angvel']
        )
        for name in sensor_names:
            sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                raise RuntimeError(
                    f"Sensor '{name}' not found in MJCF model '{model_path}'"
                )
            adr = self._model.sensor_adr[sid]
            dim = self._model.sensor_dim[sid]
            self._sensor_adr[name] = (adr, dim)

        # Reset to home keyframe
        self.reset()

    # ---- PlantInterface implementation ---------------------------------

    def command(self, leg_extensions_mm: np.ndarray) -> None:
        """Set actuator targets from home-relative leg extensions (mm).

        extension=0 → home position.  Range: [0, leg_stroke_mm].
        """
        ext = np.asarray(leg_extensions_mm, dtype=float)
        stroke = self._geom.leg_stroke_mm
        eps = 1.0  # mm tolerance for numerical noise
        if np.any(ext < -eps) or np.any(ext > stroke + eps):
            logger.warning(
                "command() extensions outside [0, %.1f] mm: min=%.2f, max=%.2f",
                stroke, ext.min(), ext.max(),
            )
        # Convert home-relative → IK-convention (add home offset) → slide
        ik_ext = ext + self._home_extensions_mm
        slide_m = self._extensions_to_slide(ik_ext)
        self._data.ctrl[:6] = slide_m

    def get_state(self) -> PlantState:
        """Read current plant state from MuJoCo sensors."""
        # Leg positions & velocities
        slide_pos_m = np.array([self._sensor(f'slide_pos_{i}')[0] for i in range(6)])
        slide_vel_mps = np.array([self._sensor(f'slide_vel_{i}')[0] for i in range(6)])

        ik_extensions_mm = self._slide_to_extensions(slide_pos_m)
        extensions_mm = ik_extensions_mm - self._home_extensions_mm  # home-relative
        velocities_mmps = slide_vel_mps * 1000.0  # m/s → mm/s

        # Platform pose
        pos_m = self._sensor('platform_pos')        # (3,) world position
        quat = self._sensor('platform_quat')         # (4,) wxyz quaternion
        linvel = self._sensor('platform_linvel')      # (3,) m/s
        angvel = self._sensor('platform_angvel')      # (3,) rad/s

        height_m = self._geom.init_height_mm / 1000.0
        pos_offset_mm = np.array([
            pos_m[0] * 1000.0,
            pos_m[1] * 1000.0,
            (pos_m[2] - height_m) * 1000.0,
        ])

        rot_matrix = quat_to_rot_matrix(quat[0], quat[1], quat[2], quat[3])
        rot_vec = rot_matrix_to_rotvec(rot_matrix)

        twist = np.concatenate([linvel * 1000.0, angvel])  # mm/s + rad/s

        return PlantState(
            leg_extensions_mm=extensions_mm,
            leg_velocities_mmps=velocities_mmps,
            platform_pos_mm=pos_offset_mm,
            platform_rot=rot_vec,
            platform_twist=twist,
            time=self._data.time,
        )

    def step(self, dt: float) -> None:
        """Advance simulation by *dt* seconds using internal substeps."""
        model_dt = self._model.opt.timestep
        n_steps = max(1, round(dt / model_dt))
        for _ in range(n_steps):
            mujoco.mj_step(self._model, self._data)

    def reset(self, pose_6dof: np.ndarray | None = None) -> None:
        """Reset to home (default) or to a specified pose.

        Parameters
        ----------
        pose_6dof : array-like of shape (6,), optional
            [x, y, z, rx, ry, rz] in mm / rad.  If *None*, resets to home.
        """
        mujoco.mj_resetDataKeyframe(self._model, self._data, 0)

        if pose_6dof is not None:
            pose = np.asarray(pose_6dof, dtype=float)
            pos_mm = pose[:3]
            rot_vec = pose[3:]
            rot = rotvec_to_rot_matrix(rot_vec)
            ik_extensions_mm = pose_to_leg_lengths(pos_mm, rot, self._geom)
            slide_m = self._extensions_to_slide(ik_extensions_mm)
            self._data.ctrl[:6] = slide_m

        mujoco.mj_forward(self._model, self._data)

    # ---- Public accessors ------------------------------------------------

    @property
    def model(self) -> mujoco.MjModel:
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        return self._data

    @property
    def geom(self) -> StewartGeometry:
        return self._geom

    @property
    def timestep(self) -> float:
        """Internal physics timestep (seconds)."""
        return self._model.opt.timestep

    @property
    def home_extensions_mm(self) -> np.ndarray:
        """IK extensions at MuJoCo home (the offset subtracted from public values)."""
        return self._home_extensions_mm.copy()

    def pose_to_extensions(self, pose_6dof: np.ndarray) -> np.ndarray:
        """Convert a [x,y,z,rx,ry,rz] pose to home-relative leg extensions (mm).

        This is the IK call with the home offset applied — ready to pass
        directly to ``command()``.
        """
        pose = np.asarray(pose_6dof, dtype=float)
        rot = rotvec_to_rot_matrix(pose[3:])
        ik_ext = pose_to_leg_lengths(pose[:3], rot, self._geom)
        return ik_ext - self._home_extensions_mm

    # ---- Internal helpers ------------------------------------------------

    def _sensor(self, name: str) -> np.ndarray:
        """Read sensor data by name (cached address lookup)."""
        adr, dim = self._sensor_adr[name]
        return self._data.sensordata[adr : adr + dim].copy()

    def _extensions_to_slide(self, extensions_mm: np.ndarray) -> np.ndarray:
        """Convert IK extensions (mm) → MuJoCo slide joint values (m).

        IK: extension = abs_length - init_leg_lengths_mm
        MuJoCo: slide = abs_length_m - geom_home_length_m
        """
        abs_length_m = (self._geom.init_leg_lengths_mm + extensions_mm) / 1000.0
        return abs_length_m - self._geom_home_lengths_m

    def _slide_to_extensions(self, slide_m: np.ndarray) -> np.ndarray:
        """Convert MuJoCo slide values (m) → IK extensions (mm)."""
        abs_length_mm = (self._geom_home_lengths_m + slide_m) * 1000.0
        return abs_length_mm - self._geom.init_leg_lengths_mm
