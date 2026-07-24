"""MuJoCo simulation plant for the Jugglebot Stewart platform.

Wraps a MuJoCo model + data pair behind the PlantInterface ABC.
Handles coordinate conversions between STOW-relative extensions (mm) and
MuJoCo slide joint values (m), and extracts platform state from MuJoCo sensors.

Coordinate convention:
    The public interface uses **STOW-relative extensions** where extension=0
    means STOW (motor = 0 rev).  MuJoCo slide=0 also means STOW (geometric
    home).  With init_leg_lengths_mm set to the geometric home length,
    IK extensions equal STOW-relative extensions directly — no offset needed.
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

# Import BallManager (optional — may not exist in minimal setups)
try:
    from ball.manager import BallManager, BallState
    _HAS_BALL = True
except ImportError:
    _HAS_BALL = False

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
    cmd_margin_mm : float
        Pre-Phase-6 ``np.clip`` margin (in mm) applied to ``command()``
        inputs.  Retained for backward compatibility but UNUSED post-
        Phase-6 — ``command()`` now forwards inputs to MuJoCo without
        clipping per P3 (trusted-callee).  Will be removed in a
        follow-up cleanup once no caller passes the kwarg.
    control_dt : float
        Configured control period in seconds.  Default 0.025 = 40 Hz.
        See controller/PLANT_INTERFACE_CONTRACT.md P4.
    """

    # P2: MuJoCo simulation can be reset to home / arbitrary pose at any
    # time.  See controller/PLANT_INTERFACE_CONTRACT.md P2.
    can_reset: bool = True

    def __init__(
        self,
        model_path: str | None = None,
        geom: StewartGeometry | None = None,
        cmd_margin_mm: float = 0.0,
        control_dt: float = 0.025,
        capture_tolerance_m: float | None = None,
        contact_carry: bool = False,
    ):
        if model_path is None:
            model_path = os.path.abspath(_DEFAULT_MODEL_PATH)

        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)
        self._geom = geom or StewartGeometry()

        # Pre-compute geometric home leg lengths (m) — needed for slide ↔ extension conversion.
        # With init_leg_lengths_mm set to the geometric home length, IK extensions
        # are STOW-relative directly (no offset needed).
        base_m = self._geom.base_nodes / 1000.0
        plat_m = self._geom.plat_nodes / 1000.0
        height_m = self._geom.init_height_mm / 1000.0
        plat_world_m = plat_m + np.array([0.0, 0.0, height_m])
        self._geom_home_lengths_m = np.linalg.norm(plat_world_m - base_m, axis=1)

        # P3: pre-Phase-6 ``command()`` silently clipped extensions to
        # [margin, stroke - margin].  Post-Phase-6 ``command()`` is a
        # trusted-callee boundary (matching ``HardwarePlant``) and
        # forwards inputs to MuJoCo without clipping; the model's slide
        # joint range provides the physics-layer envelope.  This field
        # stays as inert state so existing kwargs-based callers don't
        # break, but it's no longer read by ``command()``.  See
        # controller/PLANT_INTERFACE_CONTRACT.md P3.
        self._cmd_margin_mm = cmd_margin_mm

        # P4: configured control period (seconds).  Stored so the
        # ``control_dt`` property satisfies the abstract declaration on
        # ``PlantInterface``; ``MuJoCoPlant`` itself has no control-period-
        # derived thresholds today (no telemetry-staleness watchdog),
        # but external consumers (the runner, the scheduler's S1
        # τ_grace) read this property for the canonical period.
        # See controller/PLANT_INTERFACE_CONTRACT.md P4.
        self._control_dt = float(control_dt)

        # Detect hand actuator/sensors
        hand_act_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'act_hand')
        self._has_hand = hand_act_id >= 0
        self._hand_act_idx = hand_act_id if self._has_hand else None

        # Hand stroke (mm) and prime position (mm)
        if self._has_hand:
            self._hand_stroke_mm = 355.0  # from hardware_config
            # Catch prime: 9.858 rev × 2π × 5.21 mm/rev ≈ 322.7 mm
            self._hand_prime_mm = 9.858 * 2.0 * np.pi * 5.21

        # Cache sensor addresses for fast reads
        self._sensor_adr: dict[str, tuple[int, int]] = {}
        sensor_names = (
            [f'slide_pos_{i}' for i in range(6)]
            + [f'slide_vel_{i}' for i in range(6)]
            + ['platform_pos', 'platform_quat', 'platform_linvel', 'platform_angvel']
        )
        # Add optional sensors (hand, ball) — don't fail if absent
        optional_sensors = ['hand_slide_pos', 'hand_slide_vel',
                            'ball_pos', 'ball_vel']
        for name in sensor_names:
            sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                raise RuntimeError(
                    f"Sensor '{name}' not found in MJCF model '{model_path}'"
                )
            adr = self._model.sensor_adr[sid]
            dim = self._model.sensor_dim[sid]
            self._sensor_adr[name] = (adr, dim)
        for name in optional_sensors:
            sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid >= 0:
                adr = self._model.sensor_adr[sid]
                dim = self._model.sensor_dim[sid]
                self._sensor_adr[name] = (adr, dim)

        # Detect ball body and create BallManager if present.  The new
        # `capture_tolerance_m` knob is propagated only if the caller
        # set it explicitly — otherwise BallManager's default applies,
        # so existing callers don't have to know about the parameter.
        self._contact_carry = bool(contact_carry)
        self._ball_manager: BallManager | None = None
        if _HAS_BALL:
            ball_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_BODY, 'ball')
            if ball_id >= 0:
                bm_kw = {}
                if capture_tolerance_m is not None:
                    bm_kw['capture_tolerance_m'] = capture_tolerance_m
                self._ball_manager = BallManager(
                    self._model, self._data,
                    contact_carry=self._contact_carry, **bm_kw)

        # P1: pre-allocated PlantState returned (aliased) by every
        # ``get_state()`` call.  Ndarray fields live across ticks; scalar
        # fields (time, hand_pos_mm, hand_vel_mmps) are re-assigned each
        # call.  Mirrors the HardwarePlant pattern at
        # controller/hardware_plant.py:262–272.  See
        # controller/PLANT_INTERFACE_CONTRACT.md P1.
        self._state = PlantState(
            leg_extensions_mm=np.zeros(6),
            leg_velocities_mmps=np.zeros(6),
            platform_pos_mm=np.zeros(3),
            platform_rot=np.zeros(3),
            platform_twist=np.zeros(6),
            time=0.0,
            hand_pos_mm=None,
            hand_vel_mmps=None,
            data_age_s=None,
        )
        # Scratch buffer for the per-tick FK rot-matrix → rot-vec conversion.
        # Mutated in place by ``get_state``; held off the PlantState surface
        # because it is a transient computation, not part of the public state.
        self._slide_pos_buf = np.empty(6)
        self._slide_vel_buf = np.empty(6)

        # Reset to home keyframe
        self.reset()

    # ---- PlantInterface implementation ---------------------------------

    def command(self, leg_extensions_mm: np.ndarray,
                vel_mm_s: np.ndarray | None = None,
                cmd_next_mm: np.ndarray | None = None,
                cmd_next2_mm: np.ndarray | None = None) -> None:
        """Set actuator targets from STOW-relative leg extensions (mm).

        P3: trusted-callee boundary.  Inputs are forwarded to MuJoCo
        without validation, clipping, or coercion.  The MuJoCo model's
        slide-joint range provides the physics-layer envelope at the
        actuator level.  ``vel_mm_s``, ``cmd_next_mm``, and
        ``cmd_next2_mm`` are accepted for API compatibility but ignored
        — MuJoCo actuators handle their own dynamics via kp/kv gains.

        See controller/PLANT_INTERFACE_CONTRACT.md P3.
        """
        ext = np.asarray(leg_extensions_mm, dtype=float)
        # STOW-relative extensions = IK extensions (direct, no offset)
        slide_m = self._extensions_to_slide(ext)
        self._data.ctrl[:6] = slide_m

    def get_state(self) -> PlantState:
        """Read current plant state from MuJoCo sensors.

        P1: returns ``self._state`` — the same PlantState instance every
        call — with ndarray fields mutated in place.  Consumers MUST NOT
        retain references across ticks.  See
        controller/PLANT_INTERFACE_CONTRACT.md P1.
        """
        state = self._state

        # Leg positions & velocities — fill scratch buffers from sensor
        # reads, then convert into the PlantState ndarray fields in place.
        for i in range(6):
            self._slide_pos_buf[i] = self._sensor(f'slide_pos_{i}')[0]
            self._slide_vel_buf[i] = self._sensor(f'slide_vel_{i}')[0]
        # STOW-relative IK extensions: ext_mm = abs_length_mm - init_leg_lengths_mm
        # = (geom_home + slide_m) * 1000 - init_leg_lengths_mm.  In-place via
        # the pre-allocated leg_extensions_mm buffer.
        np.add(self._geom_home_lengths_m, self._slide_pos_buf,
               out=state.leg_extensions_mm)
        state.leg_extensions_mm *= 1000.0
        state.leg_extensions_mm -= self._geom.init_leg_lengths_mm
        # Velocities: slide_vel_mps × 1000 → mm/s.  In place.
        np.multiply(self._slide_vel_buf, 1000.0, out=state.leg_velocities_mmps)

        # Platform pose
        pos_m = self._sensor('platform_pos')        # (3,) world position
        quat = self._sensor('platform_quat')         # (4,) wxyz quaternion
        linvel = self._sensor('platform_linvel')      # (3,) m/s
        angvel = self._sensor('platform_angvel')      # (3,) rad/s

        height_m = self._geom.init_height_mm / 1000.0
        # platform_pos_mm: [x*1000, y*1000, (z - height_m)*1000] in place.
        state.platform_pos_mm[0] = pos_m[0] * 1000.0
        state.platform_pos_mm[1] = pos_m[1] * 1000.0
        state.platform_pos_mm[2] = (pos_m[2] - height_m) * 1000.0

        rot_matrix = quat_to_rot_matrix(quat[0], quat[1], quat[2], quat[3])
        # rot_matrix_to_rotvec returns a fresh (3,) ndarray; copy in place.
        np.copyto(state.platform_rot, rot_matrix_to_rotvec(rot_matrix))

        # Twist: [vx, vy, vz, wx, wy, wz] = [linvel*1000, angvel] in place.
        np.multiply(linvel, 1000.0, out=state.platform_twist[:3])
        np.copyto(state.platform_twist[3:], angvel)

        # Time + hand state (scalars; re-assigned each call).
        state.time = self._data.time
        if self._has_hand and 'hand_slide_pos' in self._sensor_adr:
            state.hand_pos_mm = float(self._sensor('hand_slide_pos')[0] * 1000.0)
            state.hand_vel_mmps = float(self._sensor('hand_slide_vel')[0] * 1000.0)
        else:
            state.hand_pos_mm = None
            state.hand_vel_mmps = None

        return state

    def step(self, dt: float, hand_cmd_fn=None, plat_cmd_fn=None) -> None:
        """Advance simulation by *dt* seconds using internal substeps.

        Each substep: optionally refresh the platform + hand commands,
        physics integration, then ball lifecycle.

        Parameters
        ----------
        hand_cmd_fn : callable(float) -> float | None, optional
            When supplied (and the model has a hand), the hand-actuator
            setpoint is refreshed **every physics substep** by sampling
            ``hand_cmd_fn(t)`` at the substep's sim time. This emulates the
            real 500 Hz / 1 kHz hand controller, so a fast throw stroke is
            tracked faithfully instead of as a coarse 40 Hz staircase held
            across the substeps. ``None`` (the default) leaves the hand at
            whatever ``command_hand`` last set — the original behaviour.
            A return value of ``None`` leaves the setpoint unchanged for
            that substep (e.g. outside a sequence's active window).
        plat_cmd_fn : callable(float) -> (ndarray | None), optional
            When supplied, the platform leg-extension targets are refreshed
            **every physics substep** by sampling ``plat_cmd_fn(t)`` (same
            units as :meth:`command`). This likewise emulates the real
            500 Hz platform controller: without it, a 40 Hz command held
            across the substeps makes the stiff leg position-actuators +
            connect constraints ring at each tick boundary, and the
            sub-tick jerk shakes a contact-carried ball out of the cup.
            ``None`` (the default) leaves the platform at whatever
            :meth:`command` last set — the original behaviour.

        Ball lifecycle per substep depends on the manager's mode:

        * kinematic hold (default): held → teleport to the cup; free →
          capture detection.
        * contact carry: held → ride the cup by physics (monitor for
          slosh-out); free → seat-based capture detection.
        """
        model_dt = self._model.opt.timestep
        n_steps = max(1, round(dt / model_dt))
        bm = self._ball_manager
        carry = self._contact_carry
        use_hand_fn = hand_cmd_fn is not None and self._has_hand
        use_plat_fn = plat_cmd_fn is not None
        for _ in range(n_steps):
            if use_plat_fn:
                ext = plat_cmd_fn(self._data.time)
                if ext is not None:
                    self.command(ext)
            if use_hand_fn:
                pos = hand_cmd_fn(self._data.time)
                if pos is not None:
                    self.command_hand(pos)
            mujoco.mj_step(self._model, self._data)
            if bm is not None:
                bm.tick_cooldowns()
                for ball in bm.balls:
                    if ball.held:
                        if carry:
                            ball.monitor_seat()
                        else:
                            ball.apply_kinematic_hold()
                    else:
                        ball.check_capture()

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

        # Reset hand to bottom of travel
        if self._has_hand:
            self._data.ctrl[self._hand_act_idx] = 0.0

        # Reset ball (park out of scene)
        if self._ball_manager is not None:
            self._ball_manager.reset()

        mujoco.mj_forward(self._model, self._data)

    # ---- Hand control -----------------------------------------------------

    @property
    def has_hand(self) -> bool:
        """Whether the model includes a hand actuator."""
        return self._has_hand

    def command_hand(self, pos_mm: float) -> None:
        """Command the hand to a linear position (mm from bottom of travel).

        No-op if the model has no hand actuator.
        """
        if not self._has_hand:
            return
        pos_m = np.clip(pos_mm / 1000.0, 0.0, self._hand_stroke_mm / 1000.0)
        self._data.ctrl[self._hand_act_idx] = pos_m

    def hand_to_home(self) -> None:
        """Command hand to bottom of travel (position 0)."""
        self.command_hand(0.0)

    def hand_to_prime(self) -> None:
        """Command hand to catch prime position (~323 mm)."""
        self.command_hand(self._hand_prime_mm)

    # ---- Ball management ---------------------------------------------------

    @property
    def has_ball(self) -> bool:
        """Whether the model includes a ball body with BallManager."""
        return self._ball_manager is not None

    @property
    def n_balls(self) -> int:
        """Number of ball bodies in the model (0 if none)."""
        return self._ball_manager.count if self._ball_manager is not None else 0

    @property
    def ball_manager(self) -> BallManager | None:
        """Direct access to BallManager (for advanced use). None if no ball."""
        return self._ball_manager

    def spawn_ball(self, position_mm: np.ndarray, velocity_mms: np.ndarray,
                   ball: int = 0) -> None:
        """Teleport ball *ball* to position and set velocity. No-op if no ball."""
        if self._ball_manager is not None:
            self._ball_manager.ball(ball).spawn(position_mm, velocity_mms)
            mujoco.mj_forward(self._model, self._data)

    def check_and_capture(self, ball: int = 0) -> bool:
        """Return True on the control step that ball *ball* is captured.

        Capture detection runs every physics substep inside step().
        This method harvests the result — it does not run detection
        itself, since no physics has advanced since the last substep.
        """
        if self._ball_manager is not None:
            return self._ball_manager.ball(ball).poll_capture()
        return False

    def get_ball_state(self, ball: int = 0) -> BallState | None:
        """Read ball *ball*'s state. None if no ball in model."""
        if self._ball_manager is not None:
            return self._ball_manager.ball(ball).get_state()
        return None

    def release_ball(self, velocity_mms: np.ndarray | None = None,
                     ball: int = 0) -> None:
        """Release ball *ball* from kinematic hold. Optionally set ejection velocity."""
        if self._ball_manager is not None:
            self._ball_manager.ball(ball).release(velocity_mms)

    def begin_physics_throw(self, ball: int = 0) -> None:
        """Transition contact-carried ball *ball* to a physics-driven throw.

        No velocity is set and ball-hand contact stays enabled — the throw
        emerges from the hand stroke. Used by the juggle demo in
        ``contact_carry`` mode in place of ``release_ball``. No-op if no
        ball in model.
        """
        if self._ball_manager is not None:
            self._ball_manager.ball(ball).begin_physics_throw()

    def ballistic_release(self, velocity_mms: np.ndarray, ball: int = 0) -> None:
        """Kinematic (ballistic) release of contact-carried ball *ball* at a set
        velocity (mm/s), breaking ball-hand contact for a clean separation and
        re-arming the contact-carry seat capture. The Rung-2b throw hand-off —
        see :meth:`ball.manager.Ball.ballistic_release`. No-op if no ball.
        """
        if self._ball_manager is not None:
            self._ball_manager.ball(ball).ballistic_release(velocity_mms)

    @property
    def contact_carry(self) -> bool:
        """Whether balls use contact-carry physics (vs kinematic hold)."""
        return self._contact_carry

    def set_contact_stiffness(self, stiff: bool) -> None:
        """Switch the ball/cup contact between soft (catch/carry) and stiff
        (throw) regimes. No-op if no ball manager. See
        ``BallManager.set_contact_stiffness``.
        """
        if self._ball_manager is not None:
            self._ball_manager.set_contact_stiffness(stiff)

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
    def control_dt(self) -> float:
        """P4: configured control period in seconds.  See
        controller/PLANT_INTERFACE_CONTRACT.md P4."""
        return self._control_dt

    @property
    def timestep(self) -> float:
        """Internal physics timestep (seconds)."""
        return self._model.opt.timestep

    def pose_to_extensions(self, pose_6dof: np.ndarray) -> np.ndarray:
        """Convert a [x,y,z,rx,ry,rz] pose to STOW-relative leg extensions (mm).

        IK extensions are STOW-relative directly — ready to pass to ``command()``.
        """
        pose = np.asarray(pose_6dof, dtype=float)
        rot = rotvec_to_rot_matrix(pose[3:])
        return pose_to_leg_lengths(pose[:3], rot, self._geom)

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
