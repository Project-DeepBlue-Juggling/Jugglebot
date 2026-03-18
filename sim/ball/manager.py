"""Ball lifecycle management: spawn, capture detection, release, reset.

The ball is a free-body sphere in MuJoCo.  Capture is proximity-based:
when the ball enters the hand cone frustum with low relative velocity,
a weld equality constraint is activated to lock the ball to the hand.

No contact mechanics — the ball does not bounce or roll inside the cone.
Contact-based capture can be added later by enabling ball-hand collision
and replacing the proximity check with a mj_contact query.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import mujoco


@dataclass
class BallState:
    """Snapshot of the ball's state."""
    position_mm: np.ndarray      # (3,) world frame
    velocity_mms: np.ndarray     # (3,) mm/s
    held: bool                   # True when weld constraint is active
    active: bool                 # True when ball is in the scene (not parked)

    # Ball is considered "parked" (inactive) when Z > 50000 mm (50 m)
    _PARK_Z_THRESHOLD_MM = 50000.0


class BallManager:
    """Manages the ball lifecycle in the MuJoCo simulation.

    Parameters
    ----------
    model : mujoco.MjModel
        The compiled MuJoCo model (must contain 'ball' body and 'ball_catch' weld).
    data : mujoco.MjData
        The MuJoCo data instance.
    capture_radius_mm : float
        XY distance from hand axis for capture (default: hand opening radius).
    capture_height_mm : float
        Height of the capture zone above hand body origin (default: cone height).
    capture_max_rel_vel_mms : float
        Maximum ball-hand relative velocity for capture (mm/s).
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        capture_radius_mm: float = 35.0,
        capture_height_mm: float = 40.0,
        capture_max_rel_vel_mms: float = 2000.0,
    ):
        self._model = model
        self._data = data
        self._capture_radius_m = capture_radius_mm / 1000.0
        self._capture_height_m = capture_height_mm / 1000.0
        self._capture_max_rel_vel = capture_max_rel_vel_mms / 1000.0  # m/s

        # Cache MuJoCo IDs
        self._ball_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'ball')
        self._hand_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'hand')
        self._hand_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')

        # Ball freejoint qpos address
        ball_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_joint')
        self._ball_qpos_adr = model.jnt_qposadr[ball_jnt_id]
        self._ball_qvel_adr = model.jnt_dofadr[ball_jnt_id]

        # Weld constraint ID
        self._weld_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, 'ball_catch')

        # Ball geom ID (for toggling collision)
        self._ball_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'ball_geom')

        # Sensor addresses
        self._ball_pos_adr = self._get_sensor_adr('ball_pos')
        self._ball_vel_adr = self._get_sensor_adr('ball_vel')

        self._held = False

    def _get_sensor_adr(self, name: str) -> int:
        sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        return self._model.sensor_adr[sid]

    def spawn(self, position_mm: np.ndarray, velocity_mms: np.ndarray) -> None:
        """Teleport ball to position and set velocity. Disables weld."""
        pos_m = np.asarray(position_mm, dtype=float) / 1000.0
        vel_mps = np.asarray(velocity_mms, dtype=float) / 1000.0

        # Disable weld
        self._set_weld_active(False)

        # Enable ball collision (disabled when parked)
        self._model.geom_contype[self._ball_geom_id] = 3
        self._model.geom_conaffinity[self._ball_geom_id] = 3

        # Set ball qpos (position xyz + quaternion wxyz)
        adr = self._ball_qpos_adr
        self._data.qpos[adr:adr + 3] = pos_m
        self._data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]  # identity quaternion

        # Set ball qvel (linear xyz + angular xyz)
        vadr = self._ball_qvel_adr
        self._data.qvel[vadr:vadr + 3] = vel_mps
        self._data.qvel[vadr + 3:vadr + 6] = 0.0

        self._held = False

    def check_capture(self) -> bool:
        """Check if ball is inside the hand cone with low relative velocity.

        If captured, activates the weld constraint and returns True.
        Returns False if ball is not in capture zone or already held.
        """
        if self._held:
            return False

        # Get ball position in world frame (m)
        ball_pos = self._data.sensordata[self._ball_pos_adr:self._ball_pos_adr + 3].copy()
        ball_vel = self._data.sensordata[self._ball_vel_adr:self._ball_vel_adr + 3].copy()

        # Get hand opening site position in world frame (m)
        hand_opening_pos = self._data.site_xpos[self._hand_site_id].copy()

        # Get hand body position and orientation
        hand_pos = self._data.xpos[self._hand_body_id].copy()
        hand_rot = self._data.xmat[self._hand_body_id].reshape(3, 3)

        # Transform ball position into hand body frame
        ball_in_hand = hand_rot.T @ (ball_pos - hand_pos)

        # Check XY distance from hand axis (hand Z axis)
        xy_dist = np.sqrt(ball_in_hand[0]**2 + ball_in_hand[1]**2)
        if xy_dist > self._capture_radius_m:
            return False

        # Check Z: ball must be between hand body origin and cone top
        if ball_in_hand[2] < 0 or ball_in_hand[2] > self._capture_height_m:
            return False

        # Check relative velocity
        # Approximate hand velocity from the body
        hand_vel = self._data.cvel[self._hand_body_id][3:6]  # linear velocity (cvel: [angular, linear])
        rel_vel = np.linalg.norm(ball_vel - hand_vel)
        if rel_vel > self._capture_max_rel_vel:
            return False

        # Capture!
        self._set_weld_active(True)
        self._held = True
        return True

    def release(self, velocity_mms: np.ndarray | None = None) -> None:
        """Disable weld constraint. Optionally set ball ejection velocity."""
        self._set_weld_active(False)
        self._held = False

        if velocity_mms is not None:
            vel_mps = np.asarray(velocity_mms, dtype=float) / 1000.0
            vadr = self._ball_qvel_adr
            self._data.qvel[vadr:vadr + 3] = vel_mps

    def get_state(self) -> BallState:
        """Read ball position/velocity from MuJoCo sensors."""
        pos_m = self._data.sensordata[self._ball_pos_adr:self._ball_pos_adr + 3].copy()
        vel_mps = self._data.sensordata[self._ball_vel_adr:self._ball_vel_adr + 3].copy()

        position_mm = pos_m * 1000.0
        velocity_mms = vel_mps * 1000.0
        active = bool(position_mm[2] < BallState._PARK_Z_THRESHOLD_MM)

        return BallState(
            position_mm=position_mm,
            velocity_mms=velocity_mms,
            held=self._held,
            active=active,
        )

    def reset(self) -> None:
        """Park ball far above scene, zero velocity, disable weld and collision.

        Ball is placed at 100 m altitude — far enough that gravity won't
        bring it into view during any realistic simulation duration.
        """
        self._set_weld_active(False)
        self._held = False

        adr = self._ball_qpos_adr
        self._data.qpos[adr:adr + 3] = [0, 0, 100]   # 100 m up
        self._data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]

        vadr = self._ball_qvel_adr
        self._data.qvel[vadr:vadr + 6] = 0.0

        # Disable collision so parked ball doesn't interact with anything
        self._model.geom_contype[self._ball_geom_id] = 0
        self._model.geom_conaffinity[self._ball_geom_id] = 0

    def _set_weld_active(self, active: bool) -> None:
        """Enable or disable the ball_catch weld constraint at runtime."""
        self._data.eq_active[self._weld_id] = 1 if active else 0
