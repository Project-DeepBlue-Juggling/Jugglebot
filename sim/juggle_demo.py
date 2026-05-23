"""Standalone runner for the BB-led two-ball oval juggle demo (sim).

Wires every Phase 2/3 component together in one process at 40 Hz:

  - :class:`controller.demo.juggle_optimizer` — solves the periodic
    optimised platform trajectory once at startup.
  - :class:`controller.demo.player.TrajectoryPlayer` — drives the platform
    from the trajectory.
  - :class:`controller.demo.timeline.MasterTimeline` — schedules the BB
    priming throw + alternating hand throws / catches.
  - :class:`sim.plant.mujoco_plant.MuJoCoPlant` — physics, two-ball
    manager, hand actuator.
  - :class:`sim.ball_butler.sim.BallButlerSim` — computes BB release
    state for the priming throw.
  - :class:`sim.hand.trajectory.HandThrowSequence` /
    :class:`sim.hand.trajectory.HandCatchSequence` — drive the hand for
    each throw / catch event.

Plan reference: ``plans/active/bb-led-two-ball-juggle-demo.md`` §4 Phase 3.
Exit criterion: ≥ 30 consecutive catches detected by
``BallManager.check_capture``.

CLI:
    python sim/juggle_demo.py                       # 30 s headless run
    python sim/juggle_demo.py --duration 10
    python sim/juggle_demo.py --viewer              # MuJoCo passive viewer
    python sim/juggle_demo.py --realtime-rate 0.5   # half-speed slowmo
    python sim/juggle_demo.py --dashboard           # http://<jetson-ip>:8082
    python sim/juggle_demo.py --abort-at 5.0        # abort mid-pattern

Viewer keyboard (when ``--viewer`` is on):
    SPACE       pause / resume
    RIGHT       step one 40 Hz tick (when paused)
    ``[``       halve the wall-time playback rate (jumps out of free-running)
    ``]``       double the wall-time playback rate
    LEFT        no-op (sim can't run backwards)

Programmatic:
    from sim.juggle_demo import run, JuggleDemoConfig
    stats = run(JuggleDemoConfig(duration_s=5.0))

Pure-Python, no ROS2. The runner is the **same architecture** the Phase 4
hardware orchestrator will mirror, except the plant is ``HardwarePlant``
instead of ``MuJoCoPlant`` and the hand/BB events go to ``can_node``
services instead of the in-process sim model.

Why the runner has a "schedule lead" for hand events
----------------------------------------------------

A :class:`HandThrowSequence` needs to *start* its prelude smooth-move
well before its release instant — typically 0.5–1.0 s for the hand to
reposition. The event's ``t_wall`` is the release time, so the runner
pulls events from the master timeline early (``t_wall +
SCHEDULE_LEAD_S``) to set the sequences up in time. BB throws have no
such lead: their ``t_wall`` is when the BB releases the ball, so they
are dispatched at the exact moment via a small pending queue.
"""
from __future__ import annotations

import argparse
import dataclasses
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

# Make controller/ and sim/ importable when this script is run directly
# (mirrors the path setup in sim/main.py).
_sim_dir = os.path.dirname(os.path.abspath(__file__))
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)
_repo_root = os.path.dirname(_sim_dir)
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

from controller.ballistics import compute_launch_velocity
from controller.demo.pattern import JugglePattern
from controller.telemetry import record_from_arrays
from controller.demo.player import TrajectoryPlayer
from controller.demo.timeline import (
    DEFAULT_BB_LEAD_S, Event, MasterTimeline,
)
from controller.demo.juggle_optimizer import (
    OptimizerConfig, optimise_juggle_trajectory,
)
from controller.demo.trajectory import JuggleTrajectory, build_analytic_oval
from sim.ball_butler.sim import BallButlerSim
from sim.hand.ballistics import compute_hand_offset_mm
from sim.hand.trajectory import (
    HandCatchSequence, HandCatchTrajectory, HandThrowSequence,
)
from sim.plant.mujoco_plant import MuJoCoPlant


CONTROL_DT_S = 0.025                       # 40 Hz player loop
SCHEDULE_LEAD_S = 1.2                      # hand-event lookahead — exceeds the
                                            # prelude duration for any throw /
                                            # catch this demo schedules. The
                                            # FIRST hand throw needs the most
                                            # lead: hand starts at catch-prime
                                            # (~323 mm), throw start is at the
                                            # stroke bottom (~20 mm), so the
                                            # smooth-move prelude is ~0.75 s
                                            # plus ~0.07 s throw lead. Steady-
                                            # state throws/catches need almost
                                            # no prelude (hand ends at throw-
                                            # start after each catch, vice
                                            # versa) — the lead is mostly for
                                            # the bootstrap.
DROP_Z_FLOOR_MM = -200.0                   # below this, ball is considered dropped
RESET_PARK_Z_MM = 100000.0                 # park a dropped ball out of scene


# --------------------------------------------------------------------------
# Internal state types
# --------------------------------------------------------------------------
@dataclasses.dataclass
class _ThrowState:
    sequence: HandThrowSequence
    event: Event
    released: bool = False


@dataclasses.dataclass
class _CatchState:
    sequence: HandCatchSequence
    event: Event


# --------------------------------------------------------------------------
# Public types
# --------------------------------------------------------------------------
@dataclasses.dataclass
class JuggleDemoConfig:
    """Knobs for :func:`run`. Sensible defaults match the plan's §4 Phase 3
    operating point.
    """
    pattern: JugglePattern = dataclasses.field(default_factory=JugglePattern)
    duration_s: float = 30.0
    n_catches: int = 32
    bb_position_mm: tuple[float, float, float] = (0.0, -1500.0, 1500.0)
    # yaw_offset_rad rotates the BB so its local +x points toward
    # Jugglebot. With BB at (0, -1500, 1500), we want local +x = world +y,
    # which is a +π/2 rotation.
    bb_yaw_offset_rad: float = math.pi / 2.0
    bb_scatter_mm: float = 0.0
    seed: Optional[int] = None
    abort_at_s: Optional[float] = None     # if set, abort at this sim_time
    log_path: Optional[str] = None
    headless: bool = True                  # if False, launch MuJoCo viewer
    use_optimised_trajectory: bool = True  # if False, use build_analytic_oval
    realtime_rate: float = 0.0             # 0 = free-running (fastest); 1.0 =
                                            # wall-clock real-time; 0.5 = half
                                            # speed (slowmo). Wall-paces the
                                            # main loop after each plant.step.
    dashboard: bool = False                # if True, start the telemetry
                                            # dashboard on `dashboard_port`
    dashboard_port: int = 8082


@dataclasses.dataclass
class JuggleDemoStats:
    """End-of-run summary returned by :func:`run`."""
    captures: list[tuple[float, int]]      # (sim_time, ball_idx)
    drops: list[tuple[float, int]]
    events_dispatched: int
    duration_s: float
    aborted: bool
    abort_time_s: Optional[float]

    @property
    def n_captures(self) -> int:
        return len(self.captures)

    @property
    def n_drops(self) -> int:
        return len(self.drops)


# --------------------------------------------------------------------------
# Runner
# --------------------------------------------------------------------------
class _JuggleDemoRunner:
    """Internal runner — one instance per :func:`run` call.

    Tests can construct this directly for white-box inspection (state
    machine, scheduled sequences, etc.); the public :func:`run` is the
    intended entry point.
    """

    def __init__(self, cfg: JuggleDemoConfig):
        if cfg.seed is not None:
            np.random.seed(cfg.seed)

        self.cfg = cfg
        self.pattern = cfg.pattern

        # ---- Trajectory --------------------------------------------------
        if cfg.use_optimised_trajectory:
            opt_cfg = OptimizerConfig(n_knots=16, n_samples=128)
            self.opt_result = optimise_juggle_trajectory(self.pattern, opt_cfg)
            self.trajectory: JuggleTrajectory = self.opt_result.trajectory
            self.catch_offset_s = self.opt_result.catch_offset_s
        else:
            # Fallback: the analytic oval. Doesn't satisfy throw/catch
            # constraints — only useful for debugging the plumbing.
            self.opt_result = None
            self.trajectory = build_analytic_oval(self.pattern)
            self.catch_offset_s = (self.pattern.flight_time_s
                                   - self.pattern.platform_period_s)

        # ---- Plant + player ---------------------------------------------
        self.plant = MuJoCoPlant(control_dt=CONTROL_DT_S)
        self.player = TrajectoryPlayer(self.trajectory,
                                       geom=self.plant.geom,
                                       control_dt=CONTROL_DT_S)

        # ---- Initial state ----------------------------------------------
        # Platform at THROW pose; hand at catch-prime; ball 1 (pre-held)
        # kinematically seated in the hand. Ball 0 (BB-primed) is parked
        # until the BB throws it.
        self.plant.reset(pose_6dof=self.pattern.throw_pose)
        self.plant.hand_to_prime()
        # Settle: a handful of ticks for the hand actuator to converge.
        for _ in range(20):
            self.plant.step(CONTROL_DT_S)
        assert self.plant.n_balls >= 2, (
            f"need >= 2 balls in the model; found {self.plant.n_balls}")
        self.plant.ball_manager.ball(1).spawn_in_hand()

        # ---- BB sim + BB lead-time calibration --------------------------
        # The MasterTimeline schedules ``bb_throw`` at t_wall = t0 - bb_lead.
        # The BB's actual time-of-flight (TOF) for the configured throw is
        # determined by its physics, not by the demo's bb_lead. To make
        # the ball arrive at the first hand catch event (t_wall = t0 +
        # catch_offset), set ``bb_lead = BB_TOF - catch_offset``.
        self.bb_sim = BallButlerSim.from_hardware_config(
            np.array(cfg.bb_position_mm, dtype=float),
            cfg.bb_yaw_offset_rad)
        # World-frame z of the hand opening at the CATCH event instant.
        # The catch event is the *midpoint of the catch velocity-hold
        # phase* (per HandCatchTrajectory's timeline convention) — by
        # that time the hand has descended ~137 mm from the top of
        # stroke (slider at ~198 mm), NOT at the catch-prime position.
        # Querying the catch trajectory at t=0 gives the exact hand-
        # slider position at arrival; the formula is independent of
        # throw_speed (the vC ratio cancels).
        _catch_at_arrival = HandCatchTrajectory(
            self.pattern.throw_speed_mps).sample(0.0)
        hand_offset_at_arrival_mm = compute_hand_offset_mm(_catch_at_arrival)
        self._catch_z_world_mm = (self.plant.geom.init_height_mm
                                  + self.pattern.active_z_mm
                                  + hand_offset_at_arrival_mm)
        target = np.array([self.pattern.catch_pose[0],
                           self.pattern.catch_pose[1],
                           self._catch_z_world_mm])
        _, _, self._bb_tof_s = self.bb_sim.compute_release_state(target)
        bb_lead = self._bb_tof_s - self.catch_offset_s
        if bb_lead <= 0.0:
            # Pathological: BB TOF is shorter than the catch offset, i.e.
            # the BB ball would land before the BB even throws. Fall back
            # to the default lead and warn — the demo will not catch the
            # priming ball cleanly, but downstream throws can still juggle.
            print(f"[juggle_demo] BB TOF {self._bb_tof_s:.3f}s < catch_offset "
                  f"{self.catch_offset_s:.3f}s — using fallback bb_lead", file=sys.stderr)
            bb_lead = DEFAULT_BB_LEAD_S

        # ---- Master timeline --------------------------------------------
        # t0 is the first hand-throw instant on the shared (sim) wall clock.
        # The init buffer above bb_lead leaves time for the first throw's
        # scheduling lead (the hand has to traverse from catch-prime to
        # the throw-stroke start before the throw fires).
        t_now = float(self.plant.get_state().time)
        self._t0 = t_now + max(bb_lead, SCHEDULE_LEAD_S) + 0.4
        self.timeline = MasterTimeline(
            self.pattern, self.trajectory, t0=self._t0,
            n_catches=cfg.n_catches,
            catch_offset_s=self.catch_offset_s,
            bb_lead_s=bb_lead,
        )

        # ---- State machine ----------------------------------------------
        # Hand sequences are queued in time order. The runner pulls events
        # from the timeline up to SCHEDULE_LEAD_S in advance — that gives
        # the bootstrap throw enough prelude time to traverse from
        # catch-prime to throw-start, but it also means a second event of
        # the same kind can be pulled before the first fires. Queueing
        # both throws and catches separately preserves time order; the
        # head's `sample(t_wall)` decides which one is currently driving
        # the hand.
        self._throw_queue: list[_ThrowState] = []
        self._catch_queue: list[_CatchState] = []
        # Deferred BB throws — pulled from the timeline early, executed at
        # their exact t_wall.
        self._pending_bb_throws: list[Event] = []

        # ---- Stats -------------------------------------------------------
        self.captures: list[tuple[float, int]] = []
        self.drops: list[tuple[float, int]] = []
        self.events_dispatched = 0
        self.aborted = False
        self.abort_time_s: Optional[float] = None
        self._t_start = t_now

        # ---- Telemetry CSV -----------------------------------------------
        self._csv_fp = None
        self._csv_writer = None
        if cfg.log_path:
            self._open_csv(cfg.log_path)

        # ---- Live dashboard ---------------------------------------------
        self._dashboard = None
        if cfg.dashboard:
            # Lazy import — the dashboard server pulls in http.server and
            # threading; the headless integration tests don't need them.
            from sim.viz.dashboard.server import DashboardServer
            self._dashboard = DashboardServer(port=cfg.dashboard_port)
            self._dashboard.start()
            # The DashboardServer's own start() prints
            # "Dashboard: http://localhost:<port>" — accurate only when a
            # browser on the *same machine* is reaching it. Print the
            # machine's LAN IPs too so a remote browser can connect.
            _print_reachable_urls(cfg.dashboard_port)
        self._last_hand_cmd_mm = 0.0       # for the StepRecord.hand_cmd_mm field

        # ---- Keyboard / interactive state -------------------------------
        # Driven by the MuJoCo passive viewer's `key_callback`. When the
        # demo runs headless (no viewer) the keys aren't observed and the
        # defaults (not paused, configured rate, no pending step) apply.
        self._paused = False
        self._pending_frame_step = False
        self._effective_realtime_rate = float(cfg.realtime_rate)

    # ---- Keyboard --------------------------------------------------------
    def key_callback(self, keycode: int) -> None:
        """Handle a keypress from the MuJoCo passive viewer.

        Bindings::

            SPACE              toggle pause
            RIGHT_ARROW        when paused, step exactly one 40 Hz tick
            ``[`` / ``]``      halve / double the wall-time playback rate
                               (jumping out of free-running mode if needed)
            LEFT_ARROW         no-op (sim can't run backwards)
        """
        if keycode == _KEY_SPACE:
            self._paused = not self._paused
            state = "paused" if self._paused else "running"
            print(f"[juggle_demo] {state} at sim_time "
                  f"{float(self.plant.get_state().time) - self._t_start:.3f} s")
        elif keycode == _KEY_RIGHT_ARROW:
            if self._paused:
                self._pending_frame_step = True
        elif keycode == _KEY_LEFT_BRACKET:
            # Halve the rate. Free-running (0) jumps to real-time first
            # so the user has somewhere meaningful to step down from.
            if self._effective_realtime_rate <= 0.0:
                self._effective_realtime_rate = 1.0
            else:
                self._effective_realtime_rate = max(
                    self._effective_realtime_rate * 0.5, 0.0625)
            print(f"[juggle_demo] realtime rate = "
                  f"{self._effective_realtime_rate:.3f}x")
        elif keycode == _KEY_RIGHT_BRACKET:
            if self._effective_realtime_rate <= 0.0:
                self._effective_realtime_rate = 1.0
            else:
                self._effective_realtime_rate = min(
                    self._effective_realtime_rate * 2.0, 16.0)
            print(f"[juggle_demo] realtime rate = "
                  f"{self._effective_realtime_rate:.3f}x")
        elif keycode == _KEY_LEFT_ARROW:
            pass   # documented no-op

    # ---- Lifecycle ------------------------------------------------------
    def close(self) -> None:
        if self._csv_fp is not None:
            self._csv_fp.close()
            self._csv_fp = None
        if self._dashboard is not None:
            self._dashboard.stop()
            self._dashboard = None

    # ---- One control tick ----------------------------------------------
    def tick(self, t_wall: float) -> None:
        """One 40 Hz control step."""
        # 1. Look ahead: pre-schedule hand events whose t_wall falls
        #    within SCHEDULE_LEAD_S, defer BB events.
        for event in self.timeline.due_events(t_wall + SCHEDULE_LEAD_S):
            self.events_dispatched += 1
            if event.kind == 'bb_throw':
                self._pending_bb_throws.append(event)
            elif event.kind == 'hand_throw':
                self._schedule_hand_throw(event, t_wall)
            elif event.kind == 'hand_catch':
                self._schedule_hand_catch(event, t_wall)

        # 2. Execute due BB throws (at their exact t_wall).
        while (self._pending_bb_throws
               and self._pending_bb_throws[0].t_wall <= t_wall):
            self._execute_bb_throw(self._pending_bb_throws.pop(0))

        # 3. Drive the hand via the active sequences.
        self._update_active_hand(t_wall)

        # 4. Command the platform.
        t_rel = t_wall - self._t0
        cmd = self.player.command_at(t_rel)
        self.plant.command(cmd.ext_mm)

        # 5. Step physics.
        self.plant.step(CONTROL_DT_S)

        # 6. Poll captures + drops.
        new_t_wall = float(self.plant.get_state().time)
        self._poll_captures(new_t_wall)
        self._poll_drops(new_t_wall)

        # 7. Optional abort.
        sim_time = new_t_wall - self._t_start
        if (self.cfg.abort_at_s is not None
                and not self.aborted
                and sim_time >= self.cfg.abort_at_s):
            self._do_abort(new_t_wall)

        # 8. Telemetry.
        if self._csv_writer is not None:
            self._write_csv_row(new_t_wall, cmd)
        if self._dashboard is not None:
            self._broadcast_dashboard_record(new_t_wall, cmd)

    # ---- Run loop -------------------------------------------------------
    def run(self, viewer_handle=None) -> JuggleDemoStats:
        n_steps = int(round(self.cfg.duration_s / CONTROL_DT_S))
        # Wall-time pacing: when ``_effective_realtime_rate > 0`` the loop
        # sleeps so one tick of sim time takes ``CONTROL_DT_S / rate``
        # seconds of wall time. ``1.0`` = real-time, ``0.5`` = half-speed
        # slowmo, ``2.0`` = 2x fast playback. ``0`` skips the pacing so
        # tests and headless runs go as fast as MuJoCo can integrate
        # (the existing behaviour). The viewer's key_callback can mutate
        # the rate live via ``[`` / ``]``; the loop re-reads it each
        # iteration so the schedule re-targets immediately.
        wall_anchor = time.monotonic()
        steps_taken_for_schedule = 0       # sim ticks consumed since the
                                            # last rate change or unpause —
                                            # the schedule's "step" count
        executed_steps = 0                  # count of ticks actually run;
                                            # advances duration progress
                                            # only when not paused
        last_rate = self._effective_realtime_rate
        while executed_steps < n_steps:
            # Re-anchor the pacing schedule if the rate changed (live key
            # bindings) so the new rate takes effect cleanly without
            # racing to "catch up" against the old schedule.
            if self._effective_realtime_rate != last_rate:
                wall_anchor = time.monotonic()
                steps_taken_for_schedule = 0
                last_rate = self._effective_realtime_rate

            if self._paused and not self._pending_frame_step:
                # Idle: keep the viewer responsive (so key events fire
                # and the user can drag the camera) but do not tick the
                # sim. Re-anchor the schedule too — when we resume we
                # don't want to suddenly run a backlog of "missed" ticks.
                if viewer_handle is not None:
                    viewer_handle.sync()
                time.sleep(0.01)
                wall_anchor = time.monotonic()
                steps_taken_for_schedule = 0
                continue
            # If a single-frame step was requested while paused, do
            # exactly one tick and re-pause.
            consume_frame_step = self._pending_frame_step
            self._pending_frame_step = False

            t_wall = float(self.plant.get_state().time)
            self.tick(t_wall)
            if viewer_handle is not None:
                viewer_handle.sync()
            executed_steps += 1
            steps_taken_for_schedule += 1

            if consume_frame_step:
                # One-shot step; immediately drop back into the paused
                # branch on the next loop iteration. Don't bother pacing
                # — single-frame stepping is meant to be instantaneous.
                continue

            if self._effective_realtime_rate > 0.0:
                # Sleep until the next tick's scheduled wall-time. Using
                # an absolute schedule (anchored at the last rate change
                # / unpause) keeps the pacing accurate over long runs
                # (no drift from cumulative sleep-undershoot).
                wall_dt = CONTROL_DT_S / self._effective_realtime_rate
                target = (wall_anchor
                          + steps_taken_for_schedule * wall_dt)
                remaining = target - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
        return JuggleDemoStats(
            captures=list(self.captures),
            drops=list(self.drops),
            events_dispatched=self.events_dispatched,
            duration_s=float(self.plant.get_state().time) - self._t_start,
            aborted=self.aborted,
            abort_time_s=self.abort_time_s,
        )

    # ---- Event dispatch (private) --------------------------------------
    def _execute_bb_throw(self, event: Event) -> None:
        """BB releases the primed ball at this exact moment."""
        # Landing xy = pose-frame xy of the first catch keyframe.
        # catch_z_world_mm was cached at construction (it depends on the
        # hand-prime position and the platform's init_height + active_z).
        landing_xy = (self.pattern.catch_pose[0], self.pattern.catch_pose[1])
        spawn = self.bb_sim.throw_at_jugglebot(
            spawn_time=event.t_wall,
            landing_xy_mm=np.array(landing_xy, dtype=float),
            catch_z_mm=self._catch_z_world_mm,
            scatter_mm=self.cfg.bb_scatter_mm,
        )
        # Spawn into the sim now (event.t_wall == current t_wall by
        # the pending-queue gating in tick()).
        self.plant.spawn_ball(spawn.position_mm, spawn.velocity_mms,
                              ball=event.ball)

    def _schedule_hand_throw(self, event: Event, t_wall: float) -> None:
        """Queue a HandThrowSequence whose release fires at event.t_wall."""
        # Use the LATEST already-queued sequence's end position as the
        # starting hand position for this new sequence's prelude — not
        # the current sensed hand position. In steady state the hand
        # finishes the previous catch at the stroke bottom; if we used
        # the current sensed position (perhaps the middle of an in-
        # progress catch) the prelude would over- or under-shoot.
        hand_pos = self._predict_hand_pos_at(event.t_wall)
        result = HandThrowSequence.try_create(
            throw_speed_mps=event.event_vel,
            release_time=event.t_wall,
            current_pos_mm=hand_pos,
            current_time=t_wall,
        )
        if result.feasible:
            self._throw_queue.append(_ThrowState(result.sequence, event))
        else:
            # Document the infeasibility; not a runner-fatal error — the
            # ball just isn't thrown this cycle.
            print(f"[juggle_demo] hand throw infeasible at t={t_wall:.3f}: "
                  f"{result.reason}", file=sys.stderr)

    def _schedule_hand_catch(self, event: Event, t_wall: float) -> None:
        """Queue a HandCatchSequence whose midpoint fires at event.t_wall."""
        # See note in :meth:`_schedule_hand_throw` — predict at the
        # event's time, not the scheduling tick's time.
        hand_pos = self._predict_hand_pos_at(event.t_wall)
        result = HandCatchSequence.try_create(
            event_vel_mps=event.event_vel,
            arrival_time=event.t_wall,
            current_pos_mm=hand_pos,
            current_time=t_wall,
        )
        if result.feasible:
            self._catch_queue.append(_CatchState(result.sequence, event))
        else:
            print(f"[juggle_demo] hand catch infeasible at t={t_wall:.3f}: "
                  f"{result.reason}", file=sys.stderr)

    def _analytic_release_velocity_world_mms(self, event: Event,
                                              t_wall: float) -> np.ndarray:
        """Sim-only override: ball release velocity to land at the matched catch.

        Read the BALL's actual world position at release (which equals
        the hand opening site under kinematic hold) and compute the
        launch velocity via :func:`compute_launch_velocity` so the
        ballistic trajectory hits the matched catch pose's world
        position. This is open-loop — it does NOT close the loop on
        sensed platform position; it just routes around MuJoCo's 40 Hz
        actuator-tracking imperfection so the ball goes where the
        Phase 2 optimiser solved for it to go.

        On hardware, the hand and platform are commanded and tracked
        at 500 Hz (motor_guard / hand Teensy) and the kinematic-hold-
        inherited velocity at release IS the analytic throw_speed —
        this override is sim-specific. The matched-catch world target
        is the same in both cases.
        """
        ball_state = self.plant.get_ball_state(ball=event.ball)
        ball_world_mm = ball_state.position_mm.copy()
        # Target: world-frame position of the hand opening at the matched
        # catch event. The platform pose at the matched catch event is
        # pattern.catch_pose (STOW-relative); world centroid = STOW + offset.
        target_world_mm = np.array([
            self.pattern.catch_pose[0],
            self.pattern.catch_pose[1],
            self._catch_z_world_mm,
        ])
        return compute_launch_velocity(
            ball_world_mm, target_world_mm, self.pattern.flight_time_s)

    def _predict_hand_pos_at(self, t_wall: float) -> float:
        """Estimate the hand position at ``t_wall`` for prelude planning.

        Returns the end position of the latest already-queued sequence
        (throw or catch, whichever ends later among those that end before
        ``t_wall``). Falls back to the current sensed hand position when
        no sequence is queued ahead.
        """
        candidates: list[tuple[float, float]] = []
        for st in self._throw_queue:
            if st.sequence.end_time <= t_wall:
                # HandThrowSequence exposes end_pos_mm directly
                candidates.append((st.sequence.end_time, st.sequence.end_pos_mm))
        for st in self._catch_queue:
            if st.sequence.end_time <= t_wall:
                candidates.append(
                    (st.sequence.end_time,
                     st.sequence.catch_trajectory.end_pos_mm))
        if candidates:
            candidates.sort(key=lambda x: x[0])
            return candidates[-1][1]
        return self.plant.get_state().hand_pos_mm or 0.0

    def _update_active_hand(self, t_wall: float) -> None:
        """Drive the hand from whichever queued sequence is currently live.

        Throw takes priority over catch during overlap (the catch's
        END_PROFILE_HOLD tail at the bottom of stroke happens to be the
        throw's prelude start, so giving throw priority is the natural
        ordering). Sequences whose ``end_time`` has passed are popped
        off the head of their respective queues.
        """
        # Prune expired heads.
        while self._throw_queue and t_wall > self._throw_queue[0].sequence.end_time:
            self._throw_queue.pop(0)
        while self._catch_queue and t_wall > self._catch_queue[0].sequence.end_time:
            self._catch_queue.pop(0)

        # Throw head first — only command when we're inside its window.
        if self._throw_queue:
            head = self._throw_queue[0]
            if t_wall >= head.sequence.prelude_start_time:
                pos = head.sequence.sample(t_wall)
                if pos is not None:
                    self.plant.command_hand(pos)
                    self._last_hand_cmd_mm = pos
                    if not head.released and t_wall >= head.sequence.release_time:
                        # Sim-only velocity override (see module docstring
                        # and :meth:`_analytic_release_velocity_world_mms`):
                        # in hardware the hand is commanded and tracked at
                        # **500 Hz** (motor_guard / hand Teensy), so the
                        # slider faithfully tracks the analytic throw stroke
                        # and the kinematic-hold-inherited velocity at
                        # release IS the analytic throw_speed. The sim
                        # runner only commands the hand at 40 Hz (the
                        # platform's loop rate), so the MuJoCo actuator
                        # sees a step-function setpoint stream and its
                        # tracking lags. Override with the inverse-
                        # ballistics value (``compute_launch_velocity``
                        # from the ball's actual world position to the
                        # matched catch position) so the released ball
                        # hits the catch the optimiser solved for. This
                        # *models* the hardware physics; it does not
                        # rewrite them.
                        release_vel = self._analytic_release_velocity_world_mms(
                            head.event, t_wall)
                        self.plant.release_ball(velocity_mms=release_vel,
                                                ball=head.event.ball)
                        head.released = True
                    return

        # No throw is driving the hand right now — try catch head.
        if self._catch_queue:
            head = self._catch_queue[0]
            if t_wall >= head.sequence.prelude_start_time:
                pos = head.sequence.sample(t_wall)
                if pos is not None:
                    self.plant.command_hand(pos)
                    self._last_hand_cmd_mm = pos

    # ---- Capture / drop polling ----------------------------------------
    def _poll_captures(self, t_wall: float) -> None:
        for ball_idx in range(self.plant.n_balls):
            if self.plant.check_and_capture(ball=ball_idx):
                self.captures.append((t_wall - self._t_start, ball_idx))

    def _poll_drops(self, t_wall: float) -> None:
        for ball_idx in range(self.plant.n_balls):
            state = self.plant.get_ball_state(ball=ball_idx)
            if state is None or not state.active or state.held:
                continue
            if state.position_mm[2] < DROP_Z_FLOOR_MM:
                self.drops.append((t_wall - self._t_start, ball_idx))
                # Park the dropped ball out of scene so it doesn't
                # interfere with the rest of the run.
                self.plant.ball_manager.ball(ball_idx).reset()

    # ---- Abort path -----------------------------------------------------
    def _do_abort(self, t_wall: float) -> None:
        """Truncate the master timeline, drop unstarted hand sequences,
        and swap the player to the exit transient.

        Sequences whose prelude has already begun stay in the queue — the
        hand can't safely be yanked mid-trajectory — but any sequence
        still in the queue with ``prelude_start_time > t_wall`` (a
        future throw or catch that hadn't started yet) is dropped so the
        abort actually stops the pattern instead of leaking late
        captures from queued-but-not-yet-fired events.
        """
        exit_transient = self.timeline.abort(t_wall)
        self.player.abort(exit_transient)
        self.aborted = True
        self.abort_time_s = t_wall - self._t_start
        self._throw_queue = [st for st in self._throw_queue
                             if st.sequence.prelude_start_time <= t_wall]
        self._catch_queue = [st for st in self._catch_queue
                             if st.sequence.prelude_start_time <= t_wall]
        self._pending_bb_throws = []

    # ---- Telemetry CSV --------------------------------------------------
    def _open_csv(self, path: str) -> None:
        import csv
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._csv_fp = open(path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_fp)
        self._csv_writer.writerow([
            'sim_time',
            'plat_x_mm', 'plat_y_mm', 'plat_z_mm',
            'hand_pos_mm',
            'ball0_x', 'ball0_y', 'ball0_z', 'ball0_held',
            'ball1_x', 'ball1_y', 'ball1_z', 'ball1_held',
            'cmd_ext0', 'cmd_ext1', 'cmd_ext2',
            'cmd_ext3', 'cmd_ext4', 'cmd_ext5',
        ])

    def _broadcast_dashboard_record(self, t_wall: float, cmd) -> None:
        """Push one StepRecord per tick to the connected dashboard clients."""
        state = self.plant.get_state()
        sim_time = t_wall - self._t_start
        record = record_from_arrays(
            time=sim_time,
            ref_pose=cmd.pose,
            ref_twist=cmd.twist,
            actual_pose=np.concatenate([state.platform_pos_mm, state.platform_rot]),
            actual_twist=state.platform_twist,
            cmd_extensions=cmd.ext_mm,
            actual_extensions=state.leg_extensions_mm,
            leg_velocities=state.leg_velocities_mmps,
            hand_cmd_mm=self._last_hand_cmd_mm,
            hand_pos_mm=state.hand_pos_mm or 0.0,
            hand_vel_mmps=state.hand_vel_mmps or 0.0,
            solve_status="ok",     # no MPC; placeholder for the dashboard schema
            t_ref_s=sim_time,
        )
        self._dashboard.broadcast(record)

    def _write_csv_row(self, t_wall: float, cmd) -> None:
        state = self.plant.get_state()
        b0 = self.plant.get_ball_state(ball=0)
        b1 = self.plant.get_ball_state(ball=1)
        self._csv_writer.writerow([
            f"{t_wall - self._t_start:.6f}",
            *(f"{x:.3f}" for x in state.platform_pos_mm),
            f"{(state.hand_pos_mm or 0.0):.3f}",
            *(f"{x:.3f}" for x in (b0.position_mm if b0 else [np.nan]*3)),
            f"{int(b0.held) if b0 else 0}",
            *(f"{x:.3f}" for x in (b1.position_mm if b1 else [np.nan]*3)),
            f"{int(b1.held) if b1 else 0}",
            *(f"{x:.4f}" for x in cmd.ext_mm),
        ])


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def _print_reachable_urls(port: int) -> None:
    """Print every LAN IPv4 address the dashboard is reachable from.

    The :class:`DashboardServer` itself prints ``http://localhost:<port>``,
    which is accurate only for a browser on the same machine. A remote
    browser (typical sim use: viewer on the Jetson, dashboard on the
    workstation) needs the Jetson's LAN IP. List every non-loopback
    IPv4 we can see so the user can copy-paste whichever one is on the
    right network.
    """
    import socket
    addrs: list[str] = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None,
                                       family=socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith('127.') and ip not in addrs:
                addrs.append(ip)
    except socket.gaierror:
        pass
    # Fallback: UDP socket trick to get the default-route IP.
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        if ip not in addrs and not ip.startswith('127.'):
            addrs.append(ip)
    except (OSError, socket.gaierror):
        pass
    for ip in addrs:
        print(f"Dashboard: http://{ip}:{port}  (LAN)")


# GLFW key codes used by mujoco.viewer's `key_callback`. Defined inline so
# we don't need to import glfw just for the constants.
_KEY_SPACE = 32
_KEY_LEFT_BRACKET = 91     # `[`
_KEY_RIGHT_BRACKET = 93    # `]`
_KEY_RIGHT_ARROW = 262
_KEY_LEFT_ARROW = 263      # accepted but no-op (sim can't run backwards)


# --------------------------------------------------------------------------
# Public entry points
# --------------------------------------------------------------------------
def run(cfg: Optional[JuggleDemoConfig] = None) -> JuggleDemoStats:
    """Run the juggle demo with the given config. Returns end-of-run stats."""
    if cfg is None:
        cfg = JuggleDemoConfig()
    runner = _JuggleDemoRunner(cfg)
    viewer_handle = None
    try:
        if not cfg.headless:
            import mujoco.viewer  # type: ignore
            viewer_handle = mujoco.viewer.launch_passive(
                runner.plant.model, runner.plant.data,
                key_callback=runner.key_callback)
            print("[juggle_demo] keyboard: SPACE pause/resume   "
                  "Right step (when paused)   [ slower / ] faster")
        return runner.run(viewer_handle=viewer_handle)
    finally:
        runner.close()
        if viewer_handle is not None:
            viewer_handle.close()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the BB-led two-ball oval juggle demo in MuJoCo.")
    parser.add_argument('--duration', type=float, default=30.0,
                        help="Simulation duration in seconds (default 30)")
    parser.add_argument('--n-catches', type=int, default=32,
                        help="Target number of catches (default 32)")
    parser.add_argument('--viewer', action='store_true',
                        help="Launch the MuJoCo passive viewer")
    parser.add_argument('--abort-at', type=float, default=None,
                        help="Trigger abort at this sim time (seconds)")
    parser.add_argument('--scatter-mm', type=float, default=0.0,
                        help="Gaussian sigma for BB landing scatter (mm)")
    parser.add_argument('--seed', type=int, default=None,
                        help="RNG seed (for reproducible BB scatter)")
    parser.add_argument('--log',
                        default=None,
                        help="CSV log path (default: temp/logs/juggle_demo_<ts>.csv)")
    parser.add_argument('--no-log', action='store_true',
                        help="Disable CSV logging")
    parser.add_argument('--analytic-baseline', action='store_true',
                        help="Use the un-optimised analytic oval (debugging only)")
    parser.add_argument('--realtime-rate', type=float, default=0.0,
                        help="Wall-clock pacing: 1.0 = real-time, 0.5 = half-"
                             "speed slowmo, 2.0 = 2x, 0 (default) = free-"
                             "running (fastest)")
    parser.add_argument('--dashboard', action='store_true',
                        help="Start the live telemetry dashboard "
                             "(http://localhost:8082)")
    parser.add_argument('--dashboard-port', type=int, default=8082,
                        help="Dashboard HTTP/SSE port (default 8082)")
    args = parser.parse_args(argv)

    if args.no_log:
        log_path = None
    elif args.log:
        log_path = args.log
    else:
        ts = time.strftime('%Y%m%d_%H%M%S')
        log_path = f"temp/logs/juggle_demo_{ts}.csv"

    cfg = JuggleDemoConfig(
        duration_s=args.duration,
        n_catches=args.n_catches,
        bb_scatter_mm=args.scatter_mm,
        seed=args.seed,
        abort_at_s=args.abort_at,
        log_path=log_path,
        headless=not args.viewer,
        use_optimised_trajectory=not args.analytic_baseline,
        realtime_rate=args.realtime_rate,
        dashboard=args.dashboard,
        dashboard_port=args.dashboard_port,
    )
    t_start = time.time()
    stats = run(cfg)
    wall_elapsed = time.time() - t_start

    print(f"[juggle_demo] sim duration:   {stats.duration_s:7.3f} s")
    print(f"[juggle_demo] wall elapsed:   {wall_elapsed:7.3f} s "
          f"(realtime {stats.duration_s / max(wall_elapsed, 1e-6):.2f}x)")
    print(f"[juggle_demo] events:         {stats.events_dispatched}")
    print(f"[juggle_demo] captures:       {stats.n_captures}")
    print(f"[juggle_demo] drops:          {stats.n_drops}")
    if stats.aborted:
        print(f"[juggle_demo] aborted at:     {stats.abort_time_s:.3f} s")
    if log_path:
        print(f"[juggle_demo] log written to: {log_path}")
    return 0 if stats.n_captures >= 1 else 1


if __name__ == '__main__':
    sys.exit(main())
