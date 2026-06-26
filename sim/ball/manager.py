"""Ball lifecycle management: spawn, capture detection, release, reset.

Each ball is a free-body sphere in MuJoCo.  Ball-hand contact uses soft
physics (solref with long time constant) for a cushioned catch.  Capture
is detected via MuJoCo's contact list.  The ball then enters one of two
hold regimes depending on the manager's ``contact_carry`` mode:

* **Kinematic hold** (default, ``contact_carry=False``) — on first
  ball-hand contact the ball's qpos/qvel are overwritten each substep to
  track the hand-opening site exactly, and ``release()`` ejects it by
  setting a velocity directly.  Used by the interactive toss/catch sims
  (``sim/main.py``) and the single-ball unit tests, where "holding" is
  bookkeeping, not physics.

* **Contact carry** (``contact_carry=True``) — the ball physically rests
  in the cup and rides the hand by contact; no teleport.  Capture is
  confirmed only when the ball has *seated* (a seat-based metric: contact
  persists for N substeps AND the ball is within the cup radius of the
  opening AND it is co-moving with the cup), and the throw *emerges* from
  the hand stroke (``begin_physics_throw()`` — no velocity is set).  Used
  by the BB-led juggle demo (``sim/juggle_demo.py``) to raise the catch /
  carry / throw to hardware fidelity.  See logbook
  ``2026-06-26-contact-mechanics-integration`` (concern 1).

A ball collides with both ground (bit 0) and hand (bit 1):
contype=3, conaffinity=3.

The model may contain more than one ball body (``ball``, ``ball2``, ...) —
the two-ball juggling demo needs one ball in hand and one in flight.
:class:`BallManager` discovers every ball body and exposes one :class:`Ball`
per body.  Its single-ball convenience methods (``spawn``, ``check_capture``,
…) operate on ball 0, so existing single-ball callers are unaffected;
multi-ball callers use :meth:`BallManager.ball` / :attr:`BallManager.balls`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import mujoco


@dataclass
class BallState:
    """Snapshot of a ball's state."""
    position_mm: np.ndarray      # (3,) world frame
    velocity_mms: np.ndarray     # (3,) mm/s
    held: bool                   # True when ball is kinematically held
    active: bool                 # True when ball is in the scene (not parked)

    # Ball is considered "parked" (inactive) when Z > 50000 mm (50 m)
    _PARK_Z_THRESHOLD_MM = 50000.0


# Hand opening site offset in hand body frame (m).
# Ball COM sits 44.4 mm above hand body origin when seated.
_HAND_OPENING_OFFSET = np.array([0.0, 0.0, 0.0444])

# Capture-tolerance: an optional centre-to-cup-opening distance check
# (m). When set, a ball is captured only when (a) it's in contact with
# any hand collision geom, AND (b) its centre is within this radius of
# the ``hand_opening`` site in world frame. Without it, MuJoCo's
# contact-anywhere check fires on peripheral cup-rim contacts and the
# kinematic hold then snaps the ball from the rim to the cup centre —
# visually ugly at low-apex juggles. The juggle demo's runner
# (``sim.juggle_demo``) sets this to ``0.030`` (30 mm) by default
# since Phase 2 cuts #4 / #5 make the optimiser produce trajectories
# that land balls at the cup centre rather than the rim. The
# *module-level* default below stays ``None`` (gate off) so non-
# juggle-demo consumers (MPC catch sims, hand-stroke tests, etc.)
# don't see a silent behaviour change — each consumer opts in
# explicitly when its motion is tight enough to survive the gate.
DEFAULT_CAPTURE_TOLERANCE_M: float | None = None


# ---- Seat-based capture metric (contact-carry mode only) --------------
# When ``contact_carry`` is on, capture is NOT the first ball-hand
# contact — a cup rushing through a velocity-matched catch at ~3.5 m/s
# touches the ball off-centre on its way past, and treating that glancing
# contact as "caught" loses the ball (it was never seated). Instead a
# catch is confirmed only when the ball has physically SETTLED into the
# cup: contact persists for ``SEAT_PERSIST_SUBSTEPS`` substeps AND the
# ball centre is within ``SEAT_RADIUS_M`` of the ``hand_opening`` site
# AND its speed relative to the cup is below ``SEAT_REL_VEL_MPS`` (it is
# co-moving, i.e. riding the hand rather than bouncing off it). These
# defaults were tuned empirically against the demo (see logbook
# 2026-06-26 contact-mechanics-integration).
SEAT_RADIUS_M: float = 0.040          # 40 mm centre-to-opening
SEAT_REL_VEL_MPS: float = 0.60        # ball must be near-co-moving with cup
SEAT_PERSIST_SUBSTEPS: int = 15       # ~15 ms at 1 kHz of continuous seating
# A held ball that drifts beyond this offset has sloshed out of the cup
# during the carry — un-hold it so the runner's drop logic can see it.
SEAT_ESCAPE_M: float = 0.070          # 2× ball radius


# ---- Phase-dependent ball/cup contact stiffness (contact-carry) -------
# MuJoCo's soft contact couples two properties we want to separate: a soft
# (long-time-constant, overdamped) contact CUSHIONS an impact but is also
# slightly COHESIVE — its velocity-damping term resists the ball separating
# from the cup, dragging a thrown ball as the stroke decelerates (a sim
# artifact; a real cup cannot pull the ball). A stiff contact separates
# cleanly (unilateral-rigid, no cohesion) but is bouncier on impact. A real
# beanbag-style ball is *both* inelastic (clean catch) and non-cohesive
# (clean throw) — neither fixed setting captures that, but the two phases
# capture one each:
#   * CATCH / carry  -> SOFT: cushions the ~3.5 m/s impact, no bounce-out.
#   * THROW stroke   -> STIFF: rigid tracking puts all the hand/platform
#                       motion into the ball and lets it separate cleanly
#                       and deterministically at the stroke peak.
# The geoms are switched at the *start of the throw up-stroke*, when the cup
# is accelerating up and pressing the ball in (compressive) — so stiffening
# firms the seating rather than ejecting the ball. solref = (timeconst,
# dampratio); solimp = (dmin, dmax, width).
CONTACT_SOFT_SOLREF = (0.05, 2.0)
CONTACT_SOFT_SOLIMP = (0.99, 0.99, 0.001)
CONTACT_STIFF_SOLREF = (0.004, 1.0)
CONTACT_STIFF_SOLIMP = (0.99, 0.999, 0.0005)


def _ball_prefix(index: int) -> str:
    """MJCF name prefix for ball *index*: 'ball', 'ball2', 'ball3', …"""
    return 'ball' if index == 0 else f'ball{index + 1}'


class Ball:
    """Lifecycle of a single ball: spawn, capture, kinematic hold, release.

    All MuJoCo names are derived from a prefix (``ball``, ``ball2``, …) so
    that several balls coexist in one model without aliasing.
    """

    def __init__(self, model, data, *, prefix: str,
                 hand_body_id: int, hand_site_id: int,
                 hand_geom_ids: "set[int]",
                 capture_tolerance_m: "float | None" = DEFAULT_CAPTURE_TOLERANCE_M,
                 contact_carry: bool = False):
        self._model = model
        self._data = data
        self.name = prefix

        self._ball_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, prefix)
        jnt_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f'{prefix}_joint')
        self._ball_qpos_adr = model.jnt_qposadr[jnt_id]
        self._ball_qvel_adr = model.jnt_dofadr[jnt_id]
        self._ball_geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f'{prefix}_geom')
        self._ball_pos_adr = self._sensor_adr(f'{prefix}_pos')
        self._ball_vel_adr = self._sensor_adr(f'{prefix}_vel')

        self._hand_body_id = hand_body_id
        self._hand_site_id = hand_site_id
        self._hand_geom_ids = hand_geom_ids
        self._capture_tolerance_m: "float | None" = (
            None if capture_tolerance_m is None else float(capture_tolerance_m))

        # Contact-carry mode (vs kinematic hold). When on, capture uses the
        # seat metric and the ball rides the cup by physics — see the
        # module docstring.
        self._contact_carry = bool(contact_carry)

        self._held = False
        self._release_cooldown = 0    # substeps to skip capture after release
        self._must_exit_zone = False  # ball must leave capture zone before re-capture
        self._capture_pending = False # set by check_capture(), read by poll_capture()
        self._seat_count = 0          # consecutive substeps meeting the seat condition

    @property
    def held(self) -> bool:
        """True while the ball is kinematically held by the hand."""
        return self._held

    def _sensor_adr(self, name: str) -> int:
        sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        return self._model.sensor_adr[sid]

    def _hand_site_velocity(self) -> np.ndarray:
        """World-frame linear velocity of the hand_opening site.

        v_site = v_body + omega x r_offset, with r_offset the hand_opening
        site position relative to the hand body origin, in world frame.
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

        # Enable ball collision (ground + hand)
        self._model.geom_contype[self._ball_geom_id] = 3
        self._model.geom_conaffinity[self._ball_geom_id] = 3

        adr = self._ball_qpos_adr
        self._data.qpos[adr:adr + 3] = pos_m
        self._data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]  # identity quaternion

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

    def _contacts_hand(self) -> bool:
        """Whether this ball's geom is in contact with any hand collision geom."""
        ball_id = self._ball_geom_id
        for i in range(self._data.ncon):
            c = self._data.contact[i]
            if ((c.geom1 == ball_id and c.geom2 in self._hand_geom_ids)
                    or (c.geom2 == ball_id and c.geom1 in self._hand_geom_ids)):
                return True
        return False

    def check_capture(self) -> bool:
        """Detect ball-hand contact and start a kinematic hold.

        After a release the ball must first **lose** contact before it can
        be recaptured (prevents immediate re-capture on the way out).

        Returns True on the substep the ball is first considered held.
        """
        if self._held:
            return False
        if self._release_cooldown > 0:
            return False

        # After release, wait until the ball is geometrically clear of the
        # hand before re-arming capture.  Distance is used (not the contact
        # list) because in kinematic-release mode contact is disabled
        # (contype=1) during this phase; in contact-carry mode contact stays
        # enabled but the still-rising throw stroke would otherwise be
        # mis-read as an immediate re-catch, so the distance gate guards both.
        if self._must_exit_zone:
            ball_pos = self._data.sensordata[
                self._ball_pos_adr:self._ball_pos_adr + 3]
            hand_pos = self._data.xpos[self._hand_body_id]
            dist = float(np.linalg.norm(ball_pos - hand_pos))
            if dist > 0.10:  # 100 mm clearance
                self._must_exit_zone = False
                self._model.geom_contype[self._ball_geom_id] = 3
                self._model.geom_conaffinity[self._ball_geom_id] = 3
            return False

        if not self._contacts_hand():
            self._seat_count = 0
            return False

        # Contact-carry mode: confirm a *seated* catch, not a glancing
        # rim contact (see :meth:`_confirm_seat`).
        if self._contact_carry:
            return self._confirm_seat()

        # Optional geometric gate (off by default — see
        # ``DEFAULT_CAPTURE_TOLERANCE_M``). When enabled, the ball's
        # centre must be within the configured tolerance of the
        # ``hand_opening`` site. MuJoCo's contact list alone fires on
        # peripheral cup-rim contacts; without this gate the
        # subsequent kinematic hold snaps the ball from wherever it
        # touched to the cup centre. Tightening this is the right
        # thing once the optimiser produces actuator-trackable
        # trajectories (Phase 2 cuts #4/#5); until then keeping it
        # off preserves the demo's headline catch rate.
        if self._capture_tolerance_m is not None:
            ball_pos = self._data.sensordata[
                self._ball_pos_adr:self._ball_pos_adr + 3]
            opening_pos = self._data.site_xpos[self._hand_site_id]
            dist = float(np.linalg.norm(ball_pos - opening_pos))
            if dist > self._capture_tolerance_m:
                return False

        self._held = True
        self._capture_pending = True
        return True

    def _ball_to_opening_m(self) -> float:
        """Distance (m) from the ball centre to the hand-opening site."""
        ball_pos = self._data.sensordata[
            self._ball_pos_adr:self._ball_pos_adr + 3]
        opening_pos = self._data.site_xpos[self._hand_site_id]
        return float(np.linalg.norm(ball_pos - opening_pos))

    def _confirm_seat(self) -> bool:
        """Seat-based capture metric for contact-carry mode.

        A catch is confirmed only when the ball has physically settled
        into the cup: it is within :data:`SEAT_RADIUS_M` of the opening
        AND co-moving with the cup (relative speed below
        :data:`SEAT_REL_VEL_MPS`) for :data:`SEAT_PERSIST_SUBSTEPS`
        consecutive substeps. This replaces the geometric snap gate,
        which cannot judge a velocity-matched (moving-cup) catch — a cup
        rushing through the catch point at ~3.5 m/s touches the ball
        off-centre on its way past, and treating that as "caught" loses
        the ball. The persistence + co-motion test asks the right
        question: did the ball seat and stay?
        """
        dist = self._ball_to_opening_m()
        ball_vel = self._data.sensordata[
            self._ball_vel_adr:self._ball_vel_adr + 3]
        rel_vel = float(np.linalg.norm(ball_vel - self._hand_site_velocity()))
        if dist <= SEAT_RADIUS_M and rel_vel <= SEAT_REL_VEL_MPS:
            self._seat_count += 1
        else:
            self._seat_count = 0
        if self._seat_count >= SEAT_PERSIST_SUBSTEPS:
            self._held = True
            self._capture_pending = True
            self._seat_count = 0
            return True
        return False

    def monitor_seat(self) -> None:
        """Watch a contact-carried held ball; un-hold it if it sloshes out.

        Called each substep (in place of the kinematic hold) while a
        contact-carried ball is held. The ball rides the cup by physics —
        no teleport — so a sufficiently violent carry could throw it out.
        If the ball drifts past :data:`SEAT_ESCAPE_M` from the opening it
        has left the cup; release the hold + arm the re-capture guard so
        the runner's drop detection sees a free ball (and the same hand
        can re-catch it if it falls back in).
        """
        if self._ball_to_opening_m() > SEAT_ESCAPE_M:
            self._held = False
            self._must_exit_zone = True
            self._release_cooldown = 30
            self._seat_count = 0

    def begin_physics_throw(self) -> None:
        """Transition a contact-carried held ball to free flight by physics.

        Sets **no** velocity — the ball flies off with the emergent velocity
        it carries from riding the hand stroke. Contact stays enabled; the
        ball separates *cleanly* because the cup contact is switched to a
        stiff, low-cohesion regime for the duration of the throw stroke (see
        :meth:`BallManager.set_contact_stiffness`). A stiff contact is
        unilateral-rigid: it cannot pull the ball, so the ball separates the
        instant the cup decelerates past g, at the cup's velocity at that
        point — physics-determined and deterministic, with none of the
        soft-contact drag-back that a ~50 ms overdamped spring would impose.
        Clearing ``_held`` re-arms capture; the re-capture guard
        (``_must_exit_zone`` + cooldown) prevents the still-rising stroke
        from being mis-read as an immediate re-catch before separation.
        """
        self._held = False
        self._capture_pending = False
        self._must_exit_zone = True
        self._release_cooldown = 30
        self._seat_count = 0

    def poll_capture(self) -> bool:
        """Return True if capture occurred since the last poll, then clear."""
        if self._capture_pending:
            self._capture_pending = False
            return True
        return False

    def apply_kinematic_hold(self) -> None:
        """Overwrite ball qpos/qvel to track the hand opening site exactly.

        Called every substep while the ball is held — after mj_step, so the
        solver's gravity integration is immediately overwritten and the ball
        behaves as if rigidly attached to the hand.
        """
        site_pos = self._data.site_xpos[self._hand_site_id].copy()
        v_site = self._hand_site_velocity()

        adr = self._ball_qpos_adr
        self._data.qpos[adr:adr + 3] = site_pos
        self._data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]

        vadr = self._ball_qvel_adr
        self._data.qvel[vadr:vadr + 3] = v_site
        self._data.qvel[vadr + 3:vadr + 6] = 0.0

    def spawn_in_hand(self) -> None:
        """Place ball at the hand opening and start a kinematic hold.

        Used for throw sequences: the ball is kinematically locked to the
        hand, with zero relative motion until release.
        """
        site_pos = self._data.site_xpos[self._hand_site_id].copy()

        adr = self._ball_qpos_adr
        self._data.qpos[adr:adr + 3] = site_pos
        self._data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]

        v_site = self._hand_site_velocity()
        vadr = self._ball_qvel_adr
        self._data.qvel[vadr:vadr + 3] = v_site
        self._data.qvel[vadr + 3:vadr + 6] = 0.0

        self._model.geom_contype[self._ball_geom_id] = 3
        self._model.geom_conaffinity[self._ball_geom_id] = 3

        self._held = True
        self._release_cooldown = 0
        self._capture_pending = False

    def release(self, velocity_mms: np.ndarray | None = None) -> None:
        """Release the ball from kinematic hold, optionally setting velocity.

        Temporarily disables ball-hand contact so the soft contact solver
        doesn't decelerate the ball on its way out; contact is re-enabled
        once the ball separates (see :meth:`check_capture`).
        """
        self._held = False
        self._capture_pending = False
        self._must_exit_zone = True
        self._release_cooldown = 30  # 30 ms at 1 kHz — secondary guard

        # Disable hand contact — ground only until the ball separates.
        self._model.geom_contype[self._ball_geom_id] = 1
        self._model.geom_conaffinity[self._ball_geom_id] = 1

        if velocity_mms is not None:
            vel_mps = np.asarray(velocity_mms, dtype=float) / 1000.0
            vadr = self._ball_qvel_adr
            self._data.qvel[vadr:vadr + 3] = vel_mps

    def get_state(self) -> BallState:
        """Read ball position/velocity from MuJoCo sensors."""
        pos_m = self._data.sensordata[
            self._ball_pos_adr:self._ball_pos_adr + 3].copy()
        vel_mps = self._data.sensordata[
            self._ball_vel_adr:self._ball_vel_adr + 3].copy()

        position_mm = pos_m * 1000.0
        velocity_mms = vel_mps * 1000.0
        active = bool(position_mm[2] < BallState._PARK_Z_THRESHOLD_MM)

        return BallState(position_mm=position_mm, velocity_mms=velocity_mms,
                         held=self._held, active=active)

    def reset(self) -> None:
        """Park ball far above the scene, zero velocity, disable collision."""
        self._held = False
        self._must_exit_zone = False
        self._capture_pending = False

        adr = self._ball_qpos_adr
        self._data.qpos[adr:adr + 3] = [0, 0, 100]   # 100 m up
        self._data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]

        vadr = self._ball_qvel_adr
        self._data.qvel[vadr:vadr + 6] = 0.0

        self._model.geom_contype[self._ball_geom_id] = 0
        self._model.geom_conaffinity[self._ball_geom_id] = 0


class BallManager:
    """Registry of the ball bodies present in a MuJoCo model.

    Discovers every ``ball`` / ``ball2`` / … body and exposes one
    :class:`Ball` per body.  The single-ball convenience methods operate on
    ball 0 so existing single-ball callers need no change; multi-ball
    callers use :meth:`ball` and :attr:`balls`.

    Parameters
    ----------
    model : mujoco.MjModel
        The compiled MuJoCo model (must contain at least a 'ball' body).
    data : mujoco.MjData
        The MuJoCo data instance.
    capture_tolerance_m : float or None
        Per-ball maximum centre-to-``hand_opening`` distance (m) at
        capture. Applied to every ball managed here. ``None`` (the
        default) disables the distance check entirely. See the
        module-level ``DEFAULT_CAPTURE_TOLERANCE_M`` for the rationale.
        Ignored in ``contact_carry`` mode (the seat metric governs).
    contact_carry : bool
        When True, every ball uses contact-carry physics (seat-based
        capture, no kinematic teleport, physics throw) instead of the
        kinematic hold. Default False (kinematic hold) for backward
        compatibility with the interactive sims and unit tests. See the
        module docstring.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 *, capture_tolerance_m: "float | None" = DEFAULT_CAPTURE_TOLERANCE_M,
                 contact_carry: bool = False):
        self._model = model
        self._data = data
        self._contact_carry = bool(contact_carry)

        hand_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, 'hand')
        hand_site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, 'hand_opening')

        # Hand collision geoms = geoms on the hand body with contype != 0.
        hand_geom_ids: "set[int]" = set()
        for geom_id in range(model.ngeom):
            if (model.geom_bodyid[geom_id] == hand_body_id
                    and model.geom_contype[geom_id] != 0):
                hand_geom_ids.add(geom_id)
        self._hand_geom_ids = hand_geom_ids
        self._contact_is_stiff = False

        # Discover ball bodies: 'ball', 'ball2', 'ball3', … until one is absent.
        self._balls: "list[Ball]" = []
        index = 0
        while True:
            prefix = _ball_prefix(index)
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, prefix) < 0:
                break
            self._balls.append(Ball(
                model, data, prefix=prefix, hand_body_id=hand_body_id,
                hand_site_id=hand_site_id, hand_geom_ids=hand_geom_ids,
                capture_tolerance_m=capture_tolerance_m,
                contact_carry=self._contact_carry))
            index += 1

        if not self._balls:
            raise ValueError("model contains no 'ball' body")

    # ---- multi-ball access ----------------------------------------------
    @property
    def count(self) -> int:
        """Number of ball bodies in the model."""
        return len(self._balls)

    @property
    def contact_carry(self) -> bool:
        """Whether the managed balls use contact-carry physics."""
        return self._contact_carry

    def set_contact_stiffness(self, stiff: bool) -> None:
        """Switch the ball+cup contact between the soft (catch/carry) and
        stiff (throw) regimes — see the module-level ``CONTACT_*`` presets.

        Writes ``geom_solref``/``geom_solimp`` on every ball geom AND every
        cup (hand-collision) geom, so the ball↔cup contact takes the chosen
        regime regardless of MuJoCo's per-geom mixing. Idempotent: a no-op
        when already in the requested regime (avoids redundant array writes
        every substep). Only meaningful in ``contact_carry`` mode.
        """
        if stiff == self._contact_is_stiff:
            return
        ref = CONTACT_STIFF_SOLREF if stiff else CONTACT_SOFT_SOLREF
        imp = CONTACT_STIFF_SOLIMP if stiff else CONTACT_SOFT_SOLIMP
        geoms = [b._ball_geom_id for b in self._balls] + list(self._hand_geom_ids)
        for gid in geoms:
            self._model.geom_solref[gid][:2] = ref
            self._model.geom_solimp[gid][:3] = imp
        self._contact_is_stiff = stiff

    def __len__(self) -> int:
        return len(self._balls)

    @property
    def balls(self) -> "tuple[Ball, ...]":
        """All balls, in body-declaration order."""
        return tuple(self._balls)

    def ball(self, index: int = 0) -> Ball:
        """The ball at *index* (0 = the original 'ball' body)."""
        return self._balls[index]

    # ---- whole-registry operations --------------------------------------
    def tick_cooldowns(self) -> None:
        """Advance every ball's post-release cooldown.  Once per substep."""
        for b in self._balls:
            b.tick_cooldowns()

    def reset(self) -> None:
        """Park every ball out of the scene."""
        for b in self._balls:
            b.reset()

    # ---- single-ball convenience (ball 0) — backward compatibility ------
    def spawn(self, position_mm: np.ndarray, velocity_mms: np.ndarray) -> None:
        self._balls[0].spawn(position_mm, velocity_mms)

    def check_capture(self) -> bool:
        return self._balls[0].check_capture()

    def poll_capture(self) -> bool:
        return self._balls[0].poll_capture()

    def apply_kinematic_hold(self) -> None:
        self._balls[0].apply_kinematic_hold()

    def spawn_in_hand(self) -> None:
        self._balls[0].spawn_in_hand()

    def release(self, velocity_mms: np.ndarray | None = None) -> None:
        self._balls[0].release(velocity_mms)

    def begin_physics_throw(self) -> None:
        self._balls[0].begin_physics_throw()

    def get_state(self) -> BallState:
        return self._balls[0].get_state()
