"""Hardware MPC entry point — run the MPC controller on real hardware.

Usage examples:
    python run_mpc.py                              # ZMQ targets from mpc_bridge_node
    python run_mpc.py --pose 0,0,170,0,0,0         # hold at active pose
    python run_mpc.py --pose 0,0,190,0,0,0 --duration 10
    python run_mpc.py --dashboard                   # with live telemetry dashboard

This is the production entry point for hardware. For simulation, use sim/main.py.

Exit codes:
  0 — normal completion (duration elapsed) or Ctrl-C (KeyboardInterrupt
      is caught, telemetry is flushed, and the finally block parks the
      motor guard via disable() before returning).
  1 — unhandled Python exception (propagates up; set by the interpreter).
  3 — HardwarePlant tripped estop() mid-run (telemetry lost, telemetry
      frozen, or FK convergence failure); CSV is finalised and summary
      printed before exit. motor_guard remains latched in ESTOP with its
      fault_state preserved (disable() is skipped on this path).

Operational notes:
  - The orchestrator must be in ACTIVE (STANDBY or any sub-mode) for motor
    feedback telemetry to flow.  Launching this script in IDLE will fail
    at the `HardwarePlant.enable()` telemetry wait.
  - This process owns the motor guard's enable state.  Startup sends
    `disable+enable` (the leading `disable` clears any lingering ESTOP);
    shutdown (Ctrl-C) sends `disable`.
  - DO NOT launch two `run_mpc.py` processes simultaneously.  Both publish
    commands on :5557 and the motor guard will see interleaved commands
    from two planners, causing jerky or unsafe motion.  No safeguard
    currently prevents this — enforce manually.
  - Ctrl-C this process before clicking Deactivate in the GUI.  Deactivating
    while MPC is running leaves the motor guard in a self-ESTOP state
    (harmless but noisy in telemetry).
"""

from __future__ import annotations

import argparse
import datetime
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

# Ensure repo root is on sys.path for controller/ imports
_repo_root = os.path.dirname(os.path.abspath(__file__))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

# Ensure ros_ws package is importable (for jugglebot.motion.*)
_ros_pkg = os.path.join(_repo_root, 'ros_ws', 'src', 'jugglebot')
if _ros_pkg not in sys.path:
    sys.path.insert(0, _ros_pkg)

from controller.mpc import MPCController
from controller.params import MPCParams
from controller.target import (
    TargetCommand, StaticTargetSource, AutoSequenceTargetSource,
)
from controller.plant import PlantState
from controller.telemetry import TelemetryLogger
from controller.runner import run_mpc_loop, MpcLoopHooks


# MPC control rate (Hz) — must match motor guard expectations
CONTROL_RATE_HZ = 40
CONTROL_DT = 1.0 / CONTROL_RATE_HZ

# Telemetry data age threshold for MPC deceleration (seconds).
# Must be less than motor guard's MPC_CMD_STALENESS_S (0.25 s).
_MPC_STALE_DECEL_S = 0.125


def parse_args():
    p = argparse.ArgumentParser(
        description='Run MPC controller on real hardware')
    p.add_argument('--pose', type=str, default=None,
                   help='Target pose: x,y,z,rx,ry,rz (mm, rad). '
                        'Z is STOW-relative; active position ≈ 170mm.')
    p.add_argument('--sequence', type=str, default=None,
                   help='Timed pose sequence: "pose@time pose@time ..."')
    p.add_argument('--duration', type=float, default=None,
                   help='Run duration in seconds (default: 24h for ZMQ, '
                        '10s for --pose)')
    p.add_argument('--log-dir', type=str, default='temp/logs',
                   dest='log_dir',
                   help='Directory for telemetry CSV logs')
    p.add_argument('--dashboard', action='store_true',
                   help='Start live telemetry dashboard (web browser)')
    p.add_argument('--dashboard-port', type=int, default=8082,
                   help='Dashboard server port (default: 8082)')
    p.add_argument('--no-torque-ff', dest='enable_torque_ff',
                   action='store_false',
                   help='Disable torque feedforward (send zeros to motor_guard). '
                        'For ablation testing.')
    p.add_argument('--no-vel-ff', dest='enable_vel_ff',
                   action='store_false',
                   help='Disable velocity feedforward (send zeros to motor_guard).')
    p.add_argument('--no-acc-ff', dest='enable_acc_ff',
                   action='store_false',
                   help='Disable acceleration feedforward (zeros cubic interp '
                        'acc+jerk terms in motor_guard).')
    p.add_argument('--toss-motion', dest='toss_motion', action='store_true',
                   help='Platform-only back-and-forth ballistic test mode. '
                        'Computes throw/catch 6-DOF poses from the current '
                        'platform pose and --catch-pose (or --sequence). '
                        'No hand, no ball.')
    p.add_argument('--catch-pose', dest='catch_pose', type=str, default=None,
                   help='--toss-motion catch position: x,y,z (mm, STOW-relative). '
                        'The throw position is the platform pose when the '
                        'cycle begins (for cycle 1, that is the pose at enable).')
    p.add_argument('--cycles', type=int, default=1,
                   help='--toss-motion cycle count. N>1 ping-pongs between '
                        'the initial pose and --catch-pose.')
    p.add_argument('--sequence-catches', dest='sequence_catches', type=str,
                   default=None,
                   help='--toss-motion catch sequence: '
                        '"x1,y1,z1 x2,y2,z2 ..." (mm, STOW-relative). Each '
                        'cycle throws from the previous catch position to '
                        'the next listed position. Overrides --catch-pose / '
                        '--cycles.')
    p.add_argument('--apex', type=float, default=1.2,
                   help='--toss-motion apex height above each throw (metres, '
                        'default 1.2).')
    p.add_argument('--hold-s', dest='hold_s', type=float, default=1.0,
                   help='--toss-motion dwell at each catch pose before the '
                        'next cycle (seconds, default 1.0).')
    p.add_argument('--final-hold-s', dest='final_hold_s', type=float,
                   default=2.0,
                   help='--toss-motion observation time at the final catch '
                        'pose after the last cycle completes (seconds, '
                        'default 2.0). The motion profile dictates total '
                        'runtime — --duration is ignored for --toss-motion.')
    p.add_argument('--sim', dest='use_sim_plant', action='store_true',
                   help='Dry-run against MuJoCoPlant instead of HardwarePlant. '
                        'For --toss-motion sanity checks before hardware.')
    return p.parse_args()


def _parse_pose(pose_str: str) -> np.ndarray:
    vals = [float(v) for v in pose_str.split(',')]
    if len(vals) != 6:
        raise ValueError(f"Pose must have 6 values, got {len(vals)}")
    return np.array(vals)


def _parse_sequence(seq_str: str) -> list[tuple[float, np.ndarray]]:
    entries = seq_str.strip().split()
    schedule: list[tuple[float, np.ndarray]] = []
    for entry in entries:
        if '@' not in entry:
            raise ValueError(f"Sequence entry must be 'pose@time', got: {entry}")
        pose_str, time_str = entry.rsplit('@', 1)
        schedule.append((float(time_str), _parse_pose(pose_str)))
    schedule.sort(key=lambda x: x[0])
    return schedule


def _parse_xyz(xyz_str: str) -> np.ndarray:
    """Parse 'X,Y,Z' into a (3,) ndarray."""
    vals = [float(v) for v in xyz_str.split(',')]
    if len(vals) != 3:
        raise ValueError(
            f"Position must have 3 values (x,y,z), got {len(vals)}: {xyz_str}")
    return np.array(vals, dtype=float)


def _parse_catch_sequence(seq_str: str) -> list[np.ndarray]:
    """Parse 'x1,y1,z1 x2,y2,z2 ...' into a list of (3,) ndarrays."""
    entries = seq_str.strip().split()
    return [_parse_xyz(entry) for entry in entries]


class _StdoutTee:
    """Duplicate writes to multiple streams; used to tee stdout/stderr to a log file."""
    def __init__(self, *streams):
        self._streams = streams
    def write(self, data):
        for s in self._streams:
            s.write(data)
            s.flush()
        return len(data)
    def flush(self):
        for s in self._streams:
            s.flush()
    def isatty(self):
        return False


def _start_stdout_tee(log_path: str):
    """Tee stdout/stderr to a companion .log file next to the CSV log_path."""
    stdout_log_path = os.path.splitext(log_path)[0] + '.log'
    os.makedirs(os.path.dirname(stdout_log_path), exist_ok=True)
    fh = open(stdout_log_path, 'w', buffering=1)  # line-buffered
    sys.stdout = _StdoutTee(sys.__stdout__, fh)
    sys.stderr = _StdoutTee(sys.__stderr__, fh)
    return stdout_log_path


def main():
    args = parse_args()

    # --- Target source selection ---
    toss_catch_positions = None  # populated when --toss-motion is set
    if args.toss_motion:
        if args.sequence_catches:
            toss_catch_positions = _parse_catch_sequence(args.sequence_catches)
        elif args.catch_pose:
            first_catch = _parse_xyz(args.catch_pose)
            # Ping-pong: odd-indexed cycles return to the platform's pose at
            # source start (None sentinel), even-indexed cycles go back to
            # --catch-pose.  TossMotionSource captures the start pose on
            # first update() and substitutes it anywhere None appears.
            toss_catch_positions = []
            for i in range(args.cycles):
                toss_catch_positions.append(
                    None if i % 2 == 1 else first_catch.copy())
        else:
            raise SystemExit(
                "--toss-motion requires --catch-pose X,Y,Z or "
                "--sequence-catches 'x1,y1,z1 x2,y2,z2 ...'")
        schedule = None
        source_label = (f"Toss motion: {len(toss_catch_positions)} cycle(s), "
                        f"apex={args.apex:.2f} m, hold={args.hold_s:.1f} s, "
                        f"plant={'sim' if args.use_sim_plant else 'hardware'}")
        # Duration for --toss-motion is derived from the motion profile:
        # N cycles × (approach + flight + hold) + final observation hold.
        # The MPC loop exits when TossMotionSource signals .done; this is
        # a safety cap on total wall-clock time.  --duration is ignored.
        from controller.ballistics import (
            flight_time_from_apex as _flight_time_from_apex,
        )
        _APPROACH_EST_S = 0.5      # K1–K6 ASAP approach, conservative floor
        _SAFETY_PAD_S = 2.0        # extra headroom above nominal estimate
        _flight_est = _flight_time_from_apex(
            np.zeros(3), np.zeros(3), args.apex * 1000.0)
        _cycle_s = _APPROACH_EST_S + _flight_est + args.hold_s
        default_duration = (
            len(toss_catch_positions) * _cycle_s
            + args.final_hold_s + _SAFETY_PAD_S)
        if args.duration is not None:
            print(
                f"NOTE: --duration={args.duration:.1f}s ignored for "
                f"--toss-motion; motion profile dictates runtime "
                f"(~{default_duration:.1f}s).")
        args.duration = None  # force default_duration path below
    elif args.sequence:
        schedule = _parse_sequence(args.sequence)
        source_label = f"Sequence: {len(schedule)} poses"
        default_duration = schedule[-1][0] + 2.0
    elif args.pose:
        target = _parse_pose(args.pose)
        schedule = [(0.0, target)]
        source_label = f"Static pose: {target}"
        default_duration = 10.0
    else:
        schedule = None
        source_label = "ZMQ targets from mpc_bridge_node"
        default_duration = 86400.0  # 24h — runs until Ctrl-C

    duration = args.duration if args.duration is not None else default_duration
    print(f"Hardware MPC [{source_label}]")
    print(f"Duration: {duration:.1f}s, Control rate: {CONTROL_RATE_HZ} Hz")

    # --- Create plant (hardware by default, MuJoCo with --sim) ---
    feedback_pub = None
    if args.use_sim_plant:
        _sim_dir = os.path.join(_repo_root, 'sim')
        if _sim_dir not in sys.path:
            sys.path.insert(0, _sim_dir)
        from plant.mujoco_plant import MuJoCoPlant
        plant = MuJoCoPlant()
        # Hardware tests start from ACTIVE (z=170 mm STOW-relative).  Match
        # that precondition in sim so the dry-run exercises the same motion
        # envelope.  reset() sets actuator targets but does not teleport the
        # joints — step the sim forward so the platform physically settles
        # at ACTIVE before TossMotionSource reads the live pose.
        _active_pose = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
        plant.reset(_active_pose)
        _active_ext = plant.pose_to_extensions(_active_pose)
        plant.command(_active_ext)
        plant.step(2.0)  # sim seconds of physical settling
        _state = plant.get_state()
        print(
            f"MuJoCoPlant: sim dry-run, settled at "
            f"({_state.platform_pos_mm[0]:+.1f}, "
            f"{_state.platform_pos_mm[1]:+.1f}, "
            f"{_state.platform_pos_mm[2]:+.1f}) mm (target ACTIVE z=170)")
    else:
        from controller.hardware_plant import HardwarePlant
        plant = HardwarePlant(
            control_dt=CONTROL_DT,
            enable_torque_ff=args.enable_torque_ff,
            enable_vel_ff=args.enable_vel_ff,
            enable_acc_ff=args.enable_acc_ff,
        )
        print("HardwarePlant: connected to motor_guard via IPC")
        print(
            f"Feedforward: torque={'ON' if args.enable_torque_ff else 'OFF'} "
            f"vel={'ON' if args.enable_vel_ff else 'OFF'} "
            f"acc={'ON' if args.enable_acc_ff else 'OFF'}"
        )

        # Target feedback for catch coordinator (accept/reject on :5559) —
        # hardware only; sim dry-run doesn't publish to ROS2.
        from jugglebot.motion.ipc import TargetFeedbackPub
        feedback_pub = TargetFeedbackPub()
        print("TargetFeedbackPub: catch feedback on :5559")

    # --- Telemetry logging ---
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(args.log_dir, f'mpc_{timestamp}.csv')
    stdout_log_path = _start_stdout_tee(log_path)
    logger = TelemetryLogger(log_path)
    print(f"Logging to: {log_path}")
    print(f"Stdout log: {stdout_log_path}")

    # --- Session metadata for ROS2 log correlation (hardware only) ---
    if not args.use_sim_plant:
        from jugglebot.motion.ipc import SessionMetadataPush
        session_push = SessionMetadataPush()
        time.sleep(0.1)  # ZMQ connection establishment
        session_push.send_session_start(os.path.basename(log_path))
        session_push.close()
        print("Session metadata sent to mpc_bridge_node")

    # --- Optional live dashboard ---
    dashboard = None
    if args.dashboard:
        # Import from sim/viz since dashboard is visualization-specific
        _sim_dir = os.path.join(_repo_root, 'sim')
        if _sim_dir not in sys.path:
            sys.path.insert(0, _sim_dir)
        from viz.dashboard import DashboardServer
        dashboard = DashboardServer(port=args.dashboard_port)
        dashboard.start()

    # --- Build MPC controller ---
    # Hardware real-time budget.  Off-Active operating points inherently need
    # 15–22 ms (3–10 IPOPT iterations) to converge — only the Active pose,
    # which is exactly what prime_solver warm-starts, solves in 0 iterations.
    # An 18 ms cap cuts off that natural distribution and forces the fallback
    # path on every other solve; 22 ms accepts the distribution as-is, leaving
    # the walk-forward fallback to handle the rare excursion past ~22 ms.
    # Safety budget is not the 25 ms MPC period but the motor-guard 75 ms
    # stale-warn / 200 ms E-STOP (see controller/hardware_plant.py).
    if args.use_sim_plant:
        # Sim dry-run: accuracy over speed; no real-time constraint.
        params_kwargs = dict(max_cpu_time=2.0, max_iter=500)
    else:
        params_kwargs = dict(max_cpu_time=0.022, max_iter=100)
    params = MPCParams(**params_kwargs)
    mpc = MPCController.from_plant(params, plant)
    assert abs(CONTROL_DT - mpc.params.dt_fine) < 1e-6, (
        f"CONTROL_DT ({CONTROL_DT}) must match MPC dt_fine "
        f"({mpc.params.dt_fine}) for correct warm-start shifting"
    )
    print(f"MPC controller initialised "
          f"(N={mpc.params.N}, tau={mpc.params.tau*1000:.0f} ms, "
          f"v_max={mpc.params.max_leg_vel_mmps:.0f} mm/s)")

    # --- Build target source ---
    # W4: every source is wired with v_max_mmps + tau_s so make_feasible_events
    # enforces K1–K6 on emitted refs (controller/REFERENCE_LAYER_CONTRACT.md).
    _src_kwargs = dict(
        clamp_start_twist_mmps=mpc.params.max_ref_start_twist_mmps,
        v_max_mmps=mpc.params.max_leg_vel_mmps,
        tau_s=mpc.params.tau,
    )
    if args.toss_motion:
        # Ping-pong via --catch-pose + --cycles: odd cycles go to
        # --catch-pose, even cycles (None sentinel) return to the platform's
        # captured-on-first-update initial pose.  TossMotionSource resolves
        # the None at cycle start — no pre-enable poke needed here.
        from controller.toss_motion_source import TossMotionSource
        source = TossMotionSource(
            catch_positions_mm=toss_catch_positions,
            apex_height_mm=args.apex * 1000.0,  # metres → mm
            hold_s=args.hold_s,
            final_hold_s=args.final_hold_s,
            **_src_kwargs,
        )
        print(
            f"TossMotionSource: {len(toss_catch_positions)} cycle(s), "
            f"apex={args.apex:.2f} m ({args.apex * 1000.0:.0f} mm), "
            f"hold={args.hold_s:.1f}s, "
            f"v_max={mpc.params.max_leg_vel_mmps:.0f} mm/s")
        for i, c in enumerate(toss_catch_positions):
            if c is None:
                print(f"  cycle {i}: return to initial pose (captured on start)")
            else:
                print(f"  cycle {i}: catch at ({c[0]:+.1f}, {c[1]:+.1f}, {c[2]:+.1f}) mm")
        print(
            f"  auto runtime ~{default_duration:.1f}s "
            f"({len(toss_catch_positions)} × ({_APPROACH_EST_S:.1f}s approach + "
            f"{_flight_est:.2f}s flight + {args.hold_s:.1f}s hold) + "
            f"{args.final_hold_s:.1f}s final hold + {_SAFETY_PAD_S:.1f}s pad). "
            f"Loop exits when the plan completes.")

        # Worst-case peak platform speed estimate: furthest horizontal catch
        # divided by the symmetric-throw flight time at the configured apex.
        # When d/T exceeds β·v_max, the K1–K6 layer silently stretches the
        # transit past the ballistic deadline (no_stretch=False default).
        from controller.ballistics import (
            flight_time_from_apex as _flight_time_from_apex,
        )
        _t_est = _flight_time_from_apex(
            np.zeros(3), np.zeros(3), args.apex * 1000.0)
        _nonnull = [c for c in toss_catch_positions if c is not None]
        _max_horizontal = (
            max(float(np.linalg.norm(c[:2])) for c in _nonnull)
            if _nonnull else 0.0)
        _est_plat_speed = _max_horizontal / _t_est if _t_est > 0 else 0.0
        _v_max = mpc.params.max_leg_vel_mmps
        if _est_plat_speed > 0.85 * _v_max:
            print(
                f"WARN: estimated TRANSIT horizontal speed "
                f"{_est_plat_speed:.0f} mm/s exceeds 0.85·v_max "
                f"({0.85 * _v_max:.0f} mm/s). K1–K6 will stretch the "
                f"quintic past the {_t_est:.2f}s ballistic deadline; "
                f"observed transit will run longer than the notional "
                f"ball's flight time. Reduce --apex, shorten --catch-pose, "
                f"or raise max_leg_vel_mmps.")
    elif schedule is not None:
        source = StaticTargetSource(schedule, **_src_kwargs)
    else:
        from controller.zmq_target import ZmqTargetSource
        default_z = (plant.geom.active_extensions_mm[0]
                     if hasattr(plant.geom, 'active_extensions_mm')
                     else 170.0)
        source = ZmqTargetSource(
            default_z_mm=default_z,
            v_max_mmps=mpc.params.max_leg_vel_mmps,
            tau_s=mpc.params.tau,
            clamp_start_twist_mmps=mpc.params.max_ref_start_twist_mmps,
            feedback_pub=feedback_pub,  # W11: catch rejection / stretch warning
        )
        print("ZmqTargetSource: waiting for targets from mpc_bridge_node on :5558")

    # --- Hardware hooks ---
    _fb_last_arrival = [None]

    # Hot-loop contract: pre-allocate buffers in the hook closure so
    # the per-tick ``_on_pre_command`` call does not allocate.  Each
    # tick overwrites these in place via ``np.subtract(..., out=buf)``.
    _twist_buf = np.empty(6)
    _twist_next_buf = np.empty(6)
    _accel_buf = np.empty(6)

    def _on_target_override(state: PlantState, tc: TargetCommand) -> TargetCommand:
        """Hold-in-place when telemetry is stale."""
        if (state.data_age_s is not None
                and state.data_age_s > _MPC_STALE_DECEL_S):
            return TargetCommand(
                target_pose=np.concatenate(
                    [state.platform_pos_mm, state.platform_rot]),
            )
        return tc

    def _on_pre_command(plant_, mpc_, tc, cmd, cmd_vel, diag):
        """Set feedforward torques from MPC's predicted trajectory — hot-loop body.

        No-op on MuJoCoPlant which has no feedforward channel (sim dry-run).
        Uses pre-allocated twist/accel buffers from the enclosing closure.

        Every in-place update uses ``np.divide(buf, scalar, out=buf)``
        rather than ``buf /= scalar`` — the augmented-assignment form
        would bind the buffer name as a local of this function and
        shadow the closure reference, causing an UnboundLocalError on
        the earlier ``np.subtract(..., out=buf)`` read.
        """
        if not hasattr(plant_, 'set_pose'):
            return
        poses = mpc_.predicted_poses_view
        times = mpc_.predicted_times_view
        if poses is not None:
            dt0 = times[1] - times[0]
            dt1 = times[2] - times[1]
            # twist = (poses[1] - poses[0]) / dt0, in place
            np.subtract(poses[1], poses[0], out=_twist_buf)
            np.divide(_twist_buf, dt0, out=_twist_buf)
            # twist_next = (poses[2] - poses[1]) / dt1, in place
            np.subtract(poses[2], poses[1], out=_twist_next_buf)
            np.divide(_twist_next_buf, dt1, out=_twist_next_buf)
            # accel = (twist_next - twist) / (0.5 * (dt0 + dt1)), in place
            np.subtract(_twist_next_buf, _twist_buf, out=_accel_buf)
            np.divide(_accel_buf, 0.5 * (dt0 + dt1), out=_accel_buf)
            plant_.set_pose(poses[0], twist_6dof=_twist_buf, accel_6dof=_accel_buf)

    def _on_post_solve(tc, diag):
        """Publish accept/reject feedback for catch targets."""
        from jugglebot.motion.ipc import make_target_feedback
        source_id = getattr(source, 'source', '')
        if source_id != 'catch' or tc.arrival_time is None:
            return
        if (_fb_last_arrival[0] is not None
                and abs(tc.arrival_time - _fb_last_arrival[0]) < 0.05):
            return
        accepted = diag.get('status', '') in (
            'Solve_Succeeded', 'Solved_To_Acceptable_Level')
        violations = None if accepted else [diag.get('status', 'unknown')]
        feedback_pub.send(make_target_feedback(
            tc.arrival_time, accepted, 'catch', violations))
        _fb_last_arrival[0] = tc.arrival_time

    # Hot-loop contract: pre-allocate the namespace once.  The runner
    # reads fields via ``getattr`` so updating in place is safe.
    _log_extras_ns = SimpleNamespace(fk_iterations=0, ff_torque_max_Nm=0.0)

    def _on_log_extras(plant_):
        """hot-loop body — mutates pre-allocated namespace, returns it."""
        _log_extras_ns.fk_iterations = getattr(plant_, 'last_fk_iterations', 0)
        _log_extras_ns.ff_torque_max_Nm = getattr(
            plant_, 'last_ff_torque_max_Nm', 0.0)
        return _log_extras_ns

    hooks = MpcLoopHooks(
        on_target_override=_on_target_override,
        on_pre_command=_on_pre_command,
        on_post_solve=_on_post_solve,
        on_log_extras=_on_log_extras,
    )

    # --- Run ---
    estop_exit = False
    try:
        # Enable pass-through mode (skip for ZmqTargetSource — the loop
        # manages enable/disable transitions based on mode messages).
        # MuJoCoPlant has no enable()/disable() lifecycle, so guard on the
        # plant too.
        if not hasattr(source, 'enabled') and hasattr(plant, 'enable'):
            plant.enable()

        estop_exit = run_mpc_loop(
            plant, mpc, source, duration, logger,
            control_dt=CONTROL_DT, dashboard=dashboard, hooks=hooks,
        )
    except KeyboardInterrupt:
        print("\nInterrupted — flushing telemetry...")
        logger.flush()
    except Exception:
        logger.flush()
        raise
    finally:
        # On safety-triggered ESTOP (frozen telemetry, FK failure, stale
        # telemetry) leave motor_guard latched in ESTOP with fault_state
        # intact so operators see the trip reason in telemetry / GUI.
        # A disable() here would transition ESTOP → DISABLED and erase
        # _fault_state.  ESTOP already zeros outputs (physical safety
        # unchanged); we only skip the operator-visibility-erasing step.
        if not getattr(plant, 'estop_requested', False) and hasattr(
                plant, 'disable'):
            plant.disable()
        time.sleep(0.05)
        if hasattr(plant, 'close'):
            plant.close()
        if feedback_pub is not None:
            feedback_pub.close()
        if hasattr(source, 'close'):
            source.close()
        if dashboard is not None:
            dashboard.stop()

    print(f"Logged {logger.total_count} records to {log_path}")
    if estop_exit:
        print("Exit 3: stopped by HardwarePlant estop()")
        sys.exit(3)


if __name__ == '__main__':
    main()
