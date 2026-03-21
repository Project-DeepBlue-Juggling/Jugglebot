"""Ball lifecycle management: spawn, capture detection, release, reset.

The ball is a free-body sphere in MuJoCo.  Ball-hand interaction uses
**kinematic hold** — when held, the ball's qpos/qvel are overwritten
each substep to exactly track the hand opening site.  No weld constraint
or contact physics between ball and hand.

The ball only collides with the ground (contype=1, conaffinity=1).
Capture is detected via geometric proximity (XY distance from hand axis
+ relative velocity threshold), and is instant (no settle time).
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
    held: bool                   # True when ball is kinematically held
    active: bool                 # True when ball is in the scene (not parked)

    # Ball is considered "parked" (inactive) when Z > 50000 mm (50 m)
    _PARK_Z_THRESHOLD_MM = 50000.0


# Hand opening site offset in hand body frame (m).
# Ball COM sits 44.4 mm above hand body origin when seated.
_HAND_OPENING_OFFSET = np.array([0.0, 0.0, 0.0444])


class BallManager:
    """Manages the ball lifecycle in the MuJoCo simulation.

    Parameters
    ----------
    model : mujoco.MjModel
        The compiled MuJoCo model (must contain 'ball' body).
    data : mujoco.MjData
        The MuJoCo data instance.
    capture_radius_mm : float
        XY distance from hand axis for capture detection.
    capture_height_mm : float
        Maximum height above hand body origin for the capture zone.
    capture_min_z_mm : float
        Minimum height above hand body origin for the capture zone.
        Slightly negative to allow margin during fast approaches.
    capture_max_rel_vel_mms : float
        Maximum ball-hand relative velocity for capture detection (mm/s).
        Capture is instant — no settle time required.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        capture_radius_mm: float = 30.0,
        capture_height_mm: float = 80.0,
        capture_min_z_mm: float = -10.0,
        capture_max_rel_vel_mms: float = 1500.0,
    ):
        self._model = model
        self._data = data
        self._capture_radius_m = capture_radius_mm / 1000.0
        self._capture_height_m = capture_height_mm / 1000.0
        self._capture_min_z_m = capture_min_z_mm / 1000.0
        self._capture_max_rel_vel = capture_max_rel_vel_mms / 1000.0  # m/s

        # Cache MuJoCo IDs
        self._ball_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'ball')
        self._hand_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'hand')
        self._hand_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')

        # Ball freejoint qpos address
        ball_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_joint')
        self._ball_qpos_adr = model.jnt_qposadr[ball_jnt_id]
        self._ball_qvel_adr = model.jnt_dofadr[ball_jnt_id]

        # Ball geom ID (for toggling collision)
        self._ball_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'ball_geom')

        # Sensor addresses
        self._ball_pos_adr = self._get_sensor_adr('ball_pos')
        self._ball_vel_adr = self._get_sensor_adr('ball_vel')

        self._held = False
        self._release_cooldown = 0  # substeps to skip capture after release
        self._must_exit_zone = False  # after release, ball must leave capture zone before re-capture
        self._capture_pending = False  # set by check_capture(), read by poll_capture()

    def _get_sensor_adr(self, name: str) -> int:
        sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        return self._model.sensor_adr[sid]

    def _hand_site_velocity(self) -> np.ndarray:
        """Compute world-frame linear velocity of the hand_opening site.

        Uses v_site = v_body + omega x r_offset, where r_offset is the
        hand_opening site position relative to the hand body origin,
        expressed in world frame.
        """
        cvel = self._data.cvel[self._hand_body_id]
        omega = cvel[:3]
        v_body = cvel[3:6]
        hand_rot = self._data.xmat[self._hand_body_id].reshape(3, 3)
        r_offset = hand_rot @ _HAND_OPENING_OFFSET
        return v_body + np.cross(omega, r_offset)

    def spawn(self, position_mm: np.ndarray, velocity_mms: np.ndarray) -> None:
        """Teleport ball to position and set velocity."""
        pos_m = np.asarray(position_mm, dtype=float) / 1000.0
        vel_mps = np.asarray(velocity_mms, dtype=float) / 1000.0

        # Enable ball collision (ground only)
        self._model.geom_contype[self._ball_geom_id] = 1
        self._model.geom_conaffinity[self._ball_geom_id] = 1

        # Set ball qpos (position xyz + quaternion wxyz)
        adr = self._ball_qpos_adr
        self._data.qpos[adr:adr + 3] = pos_m
        self._data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]  # identity quaternion

        # Set ball qvel (linear xyz + angular xyz)
        vadr = self._ball_qvel_adr
        self._data.qvel[vadr:vadr + 3] = vel_mps
        self._data.qvel[vadr + 3:vadr + 6] = 0.0

        self._held = False
        self._must_exit_zone = False
        self._capture_pending = False

    def tick_cooldowns(self) -> None:
        """Advance post-release cooldown timer.  Call once per substep."""
        if self._release_cooldown > 0:
            self._release_cooldown -= 1

    def _in_capture_zone(self) -> tuple[bool, float]:
        """Test whether the ball is inside the capture zone.

        Returns (in_zone, rel_vel_mps).  Geometric checks (XY + Z) are
        evaluated first; if out of the zone, rel_vel is returned as inf.
        """
        ball_pos = self._data.sensordata[self._ball_pos_adr:self._ball_pos_adr + 3].copy()
        ball_vel = self._data.sensordata[self._ball_vel_adr:self._ball_vel_adr + 3].copy()

        hand_pos = self._data.xpos[self._hand_body_id].copy()
        hand_rot = self._data.xmat[self._hand_body_id].reshape(3, 3)
        ball_in_hand = hand_rot.T @ (ball_pos - hand_pos)

        # XY distance from hand axis
        xy_dist = np.sqrt(ball_in_hand[0]**2 + ball_in_hand[1]**2)
        if xy_dist > self._capture_radius_m:
            return False, float('inf')

        # Z range
        if ball_in_hand[2] < self._capture_min_z_m or ball_in_hand[2] > self._capture_height_m + 0.001:
            return False, float('inf')

        # Relative velocity
        hand_vel = self._hand_site_velocity()
        rel_vel = float(np.linalg.norm(ball_vel - hand_vel))
        return True, rel_vel

    def check_capture(self) -> bool:
        """Check if ball is within the capture zone of the hand.

        Capture is instant — no settle time.  The ball must be within
        the XY capture radius of the hand axis, within the Z capture
        zone, and have relative velocity below the threshold.

        After a release, the ball must first **exit** the capture zone
        before it can be recaptured.  This prevents immediate re-capture
        when the ball is thrown along the hand axis.

        Returns True on the substep that the ball is first considered held.
        """
        if self._held:
            return False
        if self._release_cooldown > 0:
            return False

        in_zone, rel_vel = self._in_capture_zone()

        # After release, require ball to leave the zone before re-capture
        if self._must_exit_zone:
            if not in_zone:
                self._must_exit_zone = False
            return False

        if not in_zone:
            return False

        if rel_vel > self._capture_max_rel_vel:
            return False

        # Capture — start kinematic hold
        self._held = True
        self._capture_pending = True
        return True

    def poll_capture(self) -> bool:
        """Return True if capture occurred since the last poll, then clear.

        Use this instead of check_capture() from the main loop — step()
        already calls check_capture() every physics substep, so the main
        loop only needs to harvest the result.
        """
        if self._capture_pending:
            self._capture_pending = False
            return True
        return False

    def apply_kinematic_hold(self) -> None:
        """Overwrite ball qpos/qvel to track hand opening site exactly.

        Called every substep when the ball is held.  This runs after
        mj_step so the solver's gravity integration on the ball is
        immediately overwritten — the ball behaves as if rigidly
        attached to the hand.
        """
        # Read hand_opening site world position
        site_pos = self._data.site_xpos[self._hand_site_id].copy()

        # Compute site velocity
        v_site = self._hand_site_velocity()

        # Overwrite ball position
        adr = self._ball_qpos_adr
        self._data.qpos[adr:adr + 3] = site_pos
        self._data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]  # identity quat

        # Overwrite ball velocity
        vadr = self._ball_qvel_adr
        self._data.qvel[vadr:vadr + 3] = v_site
        self._data.qvel[vadr + 3:vadr + 6] = 0.0

    def spawn_in_hand(self) -> None:
        """Place ball at the hand opening and start kinematic hold.

        Used for throw sequences: the ball is kinematically locked to
        the hand, giving zero relative motion until release.
        """
        # Read hand opening site position
        site_pos = self._data.site_xpos[self._hand_site_id].copy()

        # Set ball position to hand opening site
        adr = self._ball_qpos_adr
        self._data.qpos[adr:adr + 3] = site_pos
        self._data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]

        # Match hand site velocity
        v_site = self._hand_site_velocity()
        vadr = self._ball_qvel_adr
        self._data.qvel[vadr:vadr + 3] = v_site
        self._data.qvel[vadr + 3:vadr + 6] = 0.0

        # Enable ground collision (ball may land on ground after release)
        self._model.geom_contype[self._ball_geom_id] = 1
        self._model.geom_conaffinity[self._ball_geom_id] = 1

        self._held = True
        self._release_cooldown = 0
        self._capture_pending = False

    def release(self, velocity_mms: np.ndarray | None = None) -> None:
        """Release ball from kinematic hold.  Optionally set velocity.

        If velocity_mms is provided, it becomes the ball's velocity.
        Otherwise the ball retains the hand site velocity from the
        last kinematic hold step.
        """
        self._held = False
        self._capture_pending = False
        # Ball must leave the capture zone before it can be recaptured.
        # This prevents immediate re-capture when thrown along the hand axis.
        self._must_exit_zone = True
        # Brief cooldown as a secondary guard
        self._release_cooldown = 30  # 30 ms at 1 kHz

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
        """Park ball far above scene, zero velocity, disable collision.

        Ball is placed at 100 m altitude — far enough that gravity won't
        bring it into view during any realistic simulation duration.
        """
        self._held = False
        self._must_exit_zone = False
        self._capture_pending = False

        adr = self._ball_qpos_adr
        self._data.qpos[adr:adr + 3] = [0, 0, 100]   # 100 m up
        self._data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]

        vadr = self._ball_qvel_adr
        self._data.qvel[vadr:vadr + 6] = 0.0

        # Disable collision so parked ball doesn't interact with anything
        self._model.geom_contype[self._ball_geom_id] = 0
        self._model.geom_conaffinity[self._ball_geom_id] = 0
