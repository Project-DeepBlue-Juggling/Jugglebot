#!/usr/bin/env python3
"""Bench driver: FIRST armed MPC-style setpoint stream on one can-bridge leg.

This is the highest-risk bench harness in the project so far — the **first
continuous powered setpoint stream** to a leg over the can-bridge. Unlike encoder
search and homing, which are bounded self-terminating moves, this
streams a 40 Hz knot trajectory that the leg tracks for as long as it runs. It
arms the firmware output gate (``mpc_active=1`` via the J→T heartbeat) and drives
the **synthetic** knot source (``controller/teensy_link/synthetic_setpoint.py``)
— a KNOWN, bounded command that exercises the Teensy's 40 Hz-knot Hermite interp
(``leg_interp.cpp`` Mode 1) directly, decoupled from motor_guard / the MPC /
friction-FF. The deployed production bridge is untouched (still the relay-based path).

Lives in tests/hardware/ (not tools/probes/) because it COMMANDS THE MOTOR — per
tools/probes/README.md, motor-commanding harnesses are tests/hardware/ territory.
The ``*_bench.py`` name keeps it out of pytest collection (needs a real robot). The
pure trajectory + bounds logic it drives is unit-tested in
tests/teensy_link/test_synthetic_setpoint.py.

⚠️  MOTION WARNING — CONTINUOUS, NOT bounded. The leg tracks a streamed setpoint
indefinitely (until --duration elapses or you abort). The homed leg sits at the
retracted hardstop (≈ −0.1 rev, below the stroke floor 0.0709 rev), so the first
armed motion is a controlled EXTENSION off the hardstop into the workspace via a
smooth approach ramp. KEEP THE E-STOP IN HAND. Defence in depth, all active:
  • bridge/driver: bounds-validated trajectory (envelope ⊂ workspace, per-frame
    step ≤ lead clamp), explicit operator ARM gate, instant disarm on
    fault/deviation/Ctrl-C, bounded duration, first frame commands the live
    encoder pos (no step at arm).
  • firmware: output gate (mpc_active && link-up && no-fatal), per-step lead clamp
    (0.15 rev), stroke clamp, MAX_DEVIATION E-STOP (0.5 rev), overspeed E-STOP,
    MPC staleness E-STOP (0.25 s).

PRECONDITIONS (the driver CHECKS these and refuses to arm otherwise):
  • encoder search + homing done — valid encoder reference.
  • axis in CLOSED_LOOP **position** mode with the operational gains. Pass
    --close-loop to have the driver do the full legacy-faithful bring-up (vel/curr
    limits -> POSITION/PASSTHROUGH mode -> pos+vel gains from hardware_config ->
    CLOSED_LOOP, which the ODrive auto-holds at the current pos — no jolt), needed
    after homing leaves the leg in VELOCITY mode; or bring it up yourself first.
  • fault_state == NONE, axis active_errors == 0.
  • This driver is the SOLE setpoint/wire authority — do NOT run run_mpc.py or the
    bridge's setpoint output at the same time (two authorities on the wire).

This driver does NOT touch the ROS bridge's ``enable_setpoint_output`` param; it
drives the wire directly, exactly like teensy_home_bench.py.

Usage:
    source ~/Desktop/PDJ_venv/venv/bin/activate
    # Stage (i): approach + hold (no oscillation) — the safest first run.
    python tests/hardware/teensy_setpoint_bench.py --axis 0 --center 0.15 --duration 12
    # Stage (ii): small bounded sinusoid around a workspace centre.
    python tests/hardware/teensy_setpoint_bench.py --axis 0 --center 0.30 \
        --amplitude 0.10 --freq 0.25 --duration 20
"""
import argparse
import csv
import datetime
import os
import sys
import threading
import time

sys.path.insert(0, "/home/jetson/Desktop/Jugglebot")
from controller.teensy_link import (  # noqa: E402
    TeensyLinkClient, RpcClient, RpcServer, TimeOfDayServer, MsgType,
    Telemetry, Diagnostic, LegCmd, RpcMethod, FaultState,
)
from controller.teensy_link import protocol as p  # noqa: E402
from controller.teensy_link import rpc_args  # noqa: E402
from controller.teensy_link.synthetic_setpoint import (  # noqa: E402
    SyntheticKnotSource, TrajectoryParams,
)
from controller.teensy_link.replay_setpoint import (  # noqa: E402
    RecordedThrowSource, load_recorded_axis, scale_to_bench, time_stretch,
)
import jugglebot.hardware_config as hw  # noqa: E402
import jugglebot.protocol_config as pc  # noqa: E402

TEENSY_IP = "192.168.42.2"
CLOSED_LOOP = 8
IDLE = 1

# Per-axis stroke clamp bounds — MUST match canbridge_config.h STROKE_{MIN,MAX}_REV.
STROKE_MIN_REV = [0.070917, 0.070954, 0.070448, 0.070934, 0.071340, 0.071248]
STROKE_MAX_REV = [3.900413, 3.902459, 3.874629, 3.901381, 3.923703, 3.918615]

SETPOINT_HZ = 40.0                  # MUST match the firmware SEGMENT_T_S = 0.025 s
DEVIATION_ABORT_REV = 0.30          # driver belt; firmware MAX_DEVIATION (0.5) is the backstop
VEL_STATIONARY_TOL_RPS = 0.1        # --close-loop refuses to close onto a moving leg
ARM_VERIFY_GRACE_S = 0.7            # after ARM, allow this long for the firmware to report mpc_active
_T2J_FLAG_MPC_ACTIVE = 0x8          # HeartbeatT2J.flags bit3 (firmware-side mpc_active)

_lock = threading.Lock()
_cache = {"telem": None, "diag": {}, "hb": None, "telem_rx_t": None, "legcmd": None}


def on_telem(mt, seq, payload, addr):
    tm = Telemetry.unpack(payload)
    with _lock:
        _cache["telem"] = tm
        _cache["telem_rx_t"] = time.perf_counter()   # Jetson arrival time (freeze diag)


def on_legcmd(mt, seq, payload, addr):
    # The Teensy's COMMANDED float32 interp output (LegCmd, 0x88) — the
    # residual datum: this vs the offline float64 reference is the float32 residual.
    with _lock:
        _cache["legcmd"] = LegCmd.unpack(payload)


def on_diag(mt, seq, payload, addr):
    d = Diagnostic.unpack(payload)
    with _lock:
        _cache["diag"][int(d.axis_id)] = d


def on_hb(mt, seq, payload, addr):
    from controller.teensy_link import HeartbeatT2J
    with _lock:
        _cache["hb"] = HeartbeatT2J.unpack(payload)


def axis_pos(axis):
    with _lock:
        tm = _cache["telem"]
    return None if tm is None else float(tm.pos_rev[axis])


def axis_diag(axis):
    with _lock:
        return _cache["diag"].get(int(axis))


def axis_cmd_teensy(axis):
    """The Teensy's commanded float32 interp output for the axis (rev), or None."""
    with _lock:
        lc = _cache["legcmd"]
    return None if lc is None else float(lc.cmd_pos_rev[axis])


def fault_state():
    with _lock:
        hb = _cache["hb"]
    return None if hb is None else int(hb.fault_state)


def telem_age_ms():
    """Jetson-side age of the last TELEMETRY frame. Freeze diagnostic: if this
    GROWS, the Teensy uplink stalled (no fresh frames); if it stays small while
    enc is frozen, the Teensy's CAN3 RX / axis cache froze (frames arrive, stale)."""
    with _lock:
        t = _cache.get("telem_rx_t")
    return None if t is None else (time.perf_counter() - t) * 1000.0


def hb_uptime_ms():
    """Teensy uptime from the heartbeat — advancing ⇒ the Teensy net task is alive."""
    with _lock:
        hb = _cache["hb"]
    return None if hb is None else int(hb.uptime_ms)


def _fault_name(fs):
    try:
        return FaultState(fs).name
    except ValueError:
        return str(fs)


def firmware_mpc_active():
    """The firmware's mpc_active, from the T2J heartbeat (bit3). Verifies our arm
    actually took: if it's clear after arming, another heartbeat authority (e.g. a
    running ROS2 teensy_bridge_node) is overriding mpc_active back to 0 and the
    firmware will never enable output. Returns None until a heartbeat is seen."""
    with _lock:
        hb = _cache["hb"]
    return None if hb is None else bool(hb.flags & _T2J_FLAG_MPC_ACTIVE)


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--axis", type=int, default=0)
    ap.add_argument("--center", type=float, default=0.15,
                    help="in-workspace hold/oscillation centre (rev)")
    ap.add_argument("--amplitude", type=float, default=0.0,
                    help="sinusoid amplitude (rev); 0 = pure hold (default)")
    ap.add_argument("--freq", type=float, default=0.25, help="sinusoid frequency (Hz)")
    ap.add_argument("--approach", type=float, default=3.0,
                    help="smooth ramp time from the hardstop to the centre (s)")
    ap.add_argument("--duration", type=float, default=12.0, help="total run time after arm (s)")
    ap.add_argument("--close-loop", action="store_true",
                    help="full leg bring-up before arming: vel/curr limits -> "
                         "POSITION/PASSTHROUGH mode -> pos+vel gains (from "
                         "hardware_config) -> CLOSED_LOOP (ODrive auto-holds at "
                         "current pos). Needed after homing, which leaves VELOCITY mode")
    ap.add_argument("--arm", action="store_true",
                    help="skip the interactive ARM prompt (for scripted reruns)")
    ap.add_argument("--observe", action="store_true",
                    help="fault-replay observe mode: do NOT abort+disarm on a "
                         "firmware fault. Log the transition and keep observing so the "
                         "full fault->recovery/stow sequence is recorded; cede authority "
                         "(disarm + stop streaming) on a fatal/CAN fault so the firmware "
                         "stow runs uncontested. The deviation belt stays armed as the "
                         "runaway backstop. Operator drives the physical unplug.")
    ap.add_argument("--csv", default=None,
                    help="per-tick telemetry CSV path (default: auto under temp/logs/)")
    ap.add_argument("--replay", default=None, metavar="MPC_CSV",
                    help="replay a recorded throw (temp/logs/mpc_*.csv) on this "
                         "axis instead of the synthetic trajectory — scaled to the bench "
                         "ceiling (3.30 rev), with the Teensy float32 interp output "
                         "captured via the LegCmd uplink for the residual. Ignores "
                         "--center/--amplitude/--freq.")
    ap.add_argument("--friction-ff", action="store_true",
                    help="motion-onset A/B: inject the motor_guard Stribeck "
                         "friction-FF as per-frame torque_ff (requires --replay). The "
                         "torque is computed by friction_ff_params.friction_ff_torque_nm "
                         "(parity-tested against motor_guard._compute_friction_ff_Nm) and "
                         "the can-bridge applies leg_sign on the way to the ODrive, so the "
                         "bench torque is sign-identical to production. Omit for the "
                         "friction-FF-free path (the other A/B arm).")
    ap.add_argument("--max-dev", type=float, default=DEVIATION_ABORT_REV, metavar="REV",
                    help=f"driver deviation-belt abort threshold (rev). Default "
                         f"{DEVIATION_ABORT_REV}; the firmware MAX_DEVIATION (0.5) is the "
                         f"hard backstop, so values up to ~0.5 stay within it. Raise it for "
                         f"an aggressive replay whose onset transient legitimately exceeds "
                         f"the default before the leg catches up.")
    ap.add_argument("--vel-limit", type=float, default=None, metavar="RPS",
                    help="override the ODrive vel_limit set by --close-loop (default: "
                         "hardware_config ODRIVE_LEG_VEL_LIMIT_RPS). --close-loop ALWAYS "
                         "writes this at arm, so a manual odrivetool bump is otherwise "
                         "overwritten — use this flag to make it stick.")
    ap.add_argument("--replay-stretch", type=float, default=1.0, metavar="FACTOR",
                    help="time-stretch the replay by FACTOR (>1 = slower; default 1.0). "
                         "Preserves the position shape; per-frame velocity and the knot "
                         "velocity-steps both scale 1/FACTOR. Use when an aggressive throw "
                         "onset out-accelerates the leg (deviation-belt abort): the float32 "
                         "residual is speed-independent, so stretching yields a trackable "
                         "trajectory with a valid residual + a clean from-rest onset.")
    args = ap.parse_args()
    if args.friction_ff and not args.replay:
        ap.error("--friction-ff requires --replay (the synthetic source has no torque_ff hook)")
    if args.max_dev <= 0 or args.max_dev > 0.5:
        ap.error(f"--max-dev {args.max_dev} out of (0, 0.5] (0.5 = firmware MAX_DEVIATION backstop)")
    axis = args.axis

    # Fatal/CAN faults: in --observe we cede wire authority so the firmware's
    # deferred stow / fatal handling runs uncontested. Recoverable faults
    # (MPC_STALE, MOTOR_FB_STALE, LINK_LOST) keep streaming to show self-recovery.
    _FATAL_FAULTS = {int(FaultState.CAN_BUS_DOWN), int(FaultState.ODRIVE_FATAL)}

    client = TeensyLinkClient(teensy_addr=(TEENSY_IP, p.PORT_STREAM), bind_host="0.0.0.0")
    client.start()
    rpc_server = RpcServer(client)
    tod = TimeOfDayServer(rpc_server)
    rpc = RpcClient(client)
    client.subscribe(int(MsgType.TELEMETRY), on_telem)
    client.subscribe(int(MsgType.DIAGNOSTIC), on_diag)
    client.subscribe(int(MsgType.HEARTBEAT_T2J), on_hb)
    client.subscribe(int(MsgType.LEG_CMD), on_legcmd)   # Teensy float32 interp output
    client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)   # mpc_active=0 — DISARMED
    client.set_heartbeat_flags(0)

    armed = False
    csv_f = None
    try:
        # ── Wait for telemetry ───────────────────────────────────────────────
        print(f"setpoint bench: axis={axis} peer={TEENSY_IP}")
        t0 = time.time()
        while axis_pos(axis) is None and time.time() - t0 < 3.0:
            time.sleep(0.05)
        start = axis_pos(axis)
        if start is None:
            print("ABORT: no telemetry from the Teensy."); return 2
        d = axis_diag(axis)
        fs = fault_state()
        print(f"baseline: pos={start:+.4f} rev  axis_state={d.axis_state if d else '?'}  "
              f"active_errors={d.active_errors if d else '?'}  fault_state={fs}")

        # ── Optional: full legacy-faithful leg bring-up to CLOSED_LOOP ───────
        # Ports can_node._switch_to_passthrough + _set_vel_curr_limits + _set_gains
        # to the can-bridge RPC surface (can_node is defunct). Homing leaves the
        # leg in VELOCITY mode, so streaming POSITION setpoints first needs
        # position-passthrough control re-established with the operational gains +
        # limits. Gains/limits/mode are set BEFORE closing the loop, so the loop
        # never closes on flash-default gains. The ODrive auto-seeds input_pos =
        # current_pos on CLOSED_LOOP entry, so it engages holding at the present
        # position (the hardstop) — no jolt. Values come from hardware_config
        # (never hardcoded) so they track the canonical leg config.
        if args.close_loop:
            vel_lim = float(args.vel_limit if args.vel_limit is not None
                            else hw.ODRIVE_LEG_VEL_LIMIT_RPS)
            curr_lim = float(hw.ODRIVE_LEG_CURR_LIMIT_A)
            pos_gain = float(hw.ODRIVE_LEG_POS_GAINS[axis])
            vel_gain = float(hw.ODRIVE_LEG_VEL_GAINS[axis])
            vel_int = float(hw.ODRIVE_LEG_VEL_INT_GAINS[axis])
            ctrl_pos = int(pc.ODRIVE_CONTROL_MODES['POSITION'])
            in_pass = int(pc.ODRIVE_INPUT_MODES['PASSTHROUGH'])
            # Refuse to close the loop on a moving leg (post-homing it is parked at
            # the hardstop; this guards against closing onto residual motion).
            with _lock:
                tm = _cache["telem"]
            v = abs(float(tm.vel_rps[axis])) if tm is not None else 1e9
            if v > VEL_STATIONARY_TOL_RPS:
                print(f"ABORT — leg not stationary (|vel|={v:.3f} rev/s) — refusing CLOSED_LOOP.")
                return 2
            print(f">>> bring-up axis {axis}: vel/curr {vel_lim}/{curr_lim}, "
                  f"mode POSITION/PASSTHROUGH, pos_gain {pos_gain}, "
                  f"vel_gains {vel_gain}/{vel_int}, then CLOSED_LOOP "
                  f"(ODrive auto-holds at current pos) ...")
            rpc.call(int(RpcMethod.SET_VEL_CURR_LIMITS),
                     rpc_args.encode_set_vel_curr_limits(axis, vel_lim, curr_lim))
            rpc.call(int(RpcMethod.SET_CONTROLLER_MODE),
                     rpc_args.encode_set_controller_mode(axis, ctrl_pos, in_pass))
            rpc.call(int(RpcMethod.SET_POS_GAIN),
                     rpc_args.encode_set_pos_gain(axis, pos_gain))
            rpc.call(int(RpcMethod.SET_VEL_GAINS),
                     rpc_args.encode_set_vel_gains(axis, vel_gain, vel_int))
            rpc.call(int(RpcMethod.SET_AXIS_STATE),
                     rpc_args.encode_set_axis_state(axis, CLOSED_LOOP))
            t1 = time.time()
            while time.time() - t1 < 3.0:
                d = axis_diag(axis)
                if d is not None and int(d.axis_state) == CLOSED_LOOP:
                    break
                time.sleep(0.05)
            d = axis_diag(axis)
            ep = axis_pos(axis)
            print(f">>> post-bring-up: axis_state={d.axis_state if d else '?'} "
                  f"(expect CLOSED_LOOP={CLOSED_LOOP})  pos="
                  f"{'?' if ep is None else f'{ep:+.4f}'}")

        # ── Preconditions (refuse to arm otherwise) ──────────────────────────
        d = axis_diag(axis)
        fs = fault_state()
        problems = []
        if d is None:
            problems.append("no diagnostic frame for the axis")
        else:
            if int(d.axis_state) != CLOSED_LOOP:
                problems.append(f"axis_state={d.axis_state} (need CLOSED_LOOP={CLOSED_LOOP}; "
                                f"bring the leg to closed-loop position mode, or pass --close-loop)")
            if int(d.active_errors) != 0:
                problems.append(f"active_errors={d.active_errors} (clear first)")
        if fs not in (None, int(FaultState.NONE)):
            problems.append(f"fault_state={fs} (must be NONE)")
        if problems:
            print("ABORT — preconditions not met:")
            for pb in problems:
                print(f"  • {pb}")
            return 2

        # ── Optional friction-FF injection (motion-onset A/B) ────────────────
        # Build a per-axis Stribeck torque_ff(velocity) from the canonical
        # motor_guard params (friction_ff_torque_nm is parity-tested against
        # MotorGuard._compute_friction_ff_Nm). The can-bridge applies leg_sign to
        # torque_ff on the way to the ODrive, so this is sign-identical to the
        # production motor_guard → can_node chain.
        friction_ff_fn = None
        if args.friction_ff:
            from dataclasses import replace as _dc_replace
            from jugglebot.motion.friction_ff_params import (
                load_params, friction_ff_torque_nm)
            _ffp = load_params()
            if not _ffp.enabled:
                # The bench A/B explicitly wants FF active; the YAML enabled flag
                # governs production. Force it on, loudly.
                print("  [friction-ff] NOTE: YAML friction_ff.enabled=False — "
                      "forcing enabled=True for the bench A/B.")
                _ffp = _dc_replace(_ffp, enabled=True)
            friction_ff_fn = lambda v, _p=_ffp, _ax=axis: friction_ff_torque_nm(_p, _ax, v)

        # ── Build the bounds-validated trajectory (synthetic OR recorded replay) ─
        try:
            if args.replay:
                from jugglebot.motion.geometry import StewartGeometry
                mm_to_rev = float(StewartGeometry().mm_to_rev[axis])
                raw = load_recorded_axis(args.replay, axis, mm_to_rev)
                if args.replay_stretch != 1.0:
                    raw = time_stretch(raw, args.replay_stretch)
                scaled, sinfo = scale_to_bench(raw, stroke_min_rev=STROKE_MIN_REV[axis])
                gen = RecordedThrowSource(
                    scaled, axis=axis, start_rev=start,
                    stroke_min_rev=STROKE_MIN_REV[axis], stroke_max_rev=STROKE_MAX_REV[axis],
                    approach_s=args.approach, settle_s=1.0,
                    torque_ff_fn=friction_ff_fn)
                # Replay defines its own length; extend --duration to cover it if needed.
                if args.duration < gen.total_duration_s:
                    args.duration = gen.total_duration_s + 1.0
            else:
                gen = SyntheticKnotSource(
                    TrajectoryParams(axis=axis, start_rev=start, center_rev=args.center,
                                     amplitude_rev=args.amplitude, freq_hz=args.freq,
                                     approach_s=args.approach),
                    stroke_min_rev=STROKE_MIN_REV[axis], stroke_max_rev=STROKE_MAX_REV[axis])
        except (ValueError, FileNotFoundError) as e:
            print(f"ABORT — trajectory rejected: {e}")
            return 2

        print("\n── planned motion ─────────────────────────────────────────────")
        if args.replay:
            stretch_note = "" if args.replay_stretch == 1.0 else f"  [time-stretch {args.replay_stretch}x]"
            print(f"  axis {axis}: REPLAY {args.replay.split('/')[-1]}{stretch_note}")
            print(f"    raw [{sinfo.raw_min_rev:.3f}, {sinfo.raw_max_rev:.3f}] rev → "
                  f"scaled [{sinfo.scaled_min_rev:.3f}, {sinfo.scaled_max_rev:.3f}] "
                  f"(pos_scale {sinfo.pos_scale:.3f}, ceiling {sinfo.ceiling_rev})")
            print(f"    approach {args.approach}s → settle 1.0s → onset at "
                  f"t={gen.onset_t_s:.1f}s → replay (peak {sinfo.peak_vel_rps:.2f} rev/s)")
            if friction_ff_fn is not None:
                print(f"    friction-FF: ON — torque_ff @|v|=0.1/1.0/{sinfo.peak_vel_rps:.2f} "
                      f"rev/s = {friction_ff_fn(0.1):+.4f}/{friction_ff_fn(1.0):+.4f}/"
                      f"{friction_ff_fn(sinfo.peak_vel_rps):+.4f} Nm (motor_guard convention)")
            else:
                print(f"    friction-FF: OFF (plain path)")
        else:
            kind = "HOLD" if args.amplitude == 0.0 else f"SINUSOID ±{args.amplitude} @ {args.freq} Hz"
            print(f"  axis {axis}: extend {start:+.4f} → {args.center:+.4f} rev over "
                  f"{args.approach}s, then {kind} for {args.duration}s")
        print(f"  peak per-frame step {gen.peak_step_rev:.4f} rev (lead clamp 0.15)")
        print(f"  workspace [{STROKE_MIN_REV[axis]:.4f}, {STROKE_MAX_REV[axis]:.4f}] rev")
        print("───────────────────────────────────────────────────────────────")
        print("⚠️  This ARMS the leg (mpc_active=1) and STREAMS continuous motion.")
        print("    E-STOP IN HAND. Ctrl-C disarms instantly.")
        if not args.arm:
            ans = input(">>> type 'ARM' to arm and run, anything else to abort: ").strip()
            if ans != "ARM":
                print("aborted by operator (not armed)."); return 0

        # ── Per-tick telemetry CSV (always on) — the durable record ───────────
        csv_path = args.csv
        if csv_path is None:
            os.makedirs("temp/logs", exist_ok=True)
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            mode = "replay" if args.replay else (
                "observe" if args.observe else ("sin" if args.amplitude else "hold"))
            csv_path = f"temp/logs/setpoint_bench_ax{axis}_{mode}_{ts}.csv"
        csv_f = open(csv_path, "w", newline="")
        csv_w = csv.writer(csv_f)
        # cmd_teensy_rev = the Teensy's float32 interp output (LegCmd uplink) — vs the
        # offline float64 reference of `cmd_rev`'s knots, that is the float32 residual.
        # cmd_tau_ff_Nm = the friction-FF torque injected into the frame this tick
        # (motor_guard convention, the Teensy applies leg_sign downstream); blank
        # when --friction-ff is off — the friction-FF-free arm of the motion-onset A/B.
        csv_w.writerow(["t_s", "cmd_rev", "enc_rev", "cmd_teensy_rev", "vel_rps",
                        "err_rev", "iq_A", "cmd_tau_ff_Nm", "fault_state", "fault_name",
                        "telem_age_ms", "hb_uptime_ms", "fw_mpc_active", "streaming"])
        print(f">>> logging telemetry → {csv_path}")

        # ── ARM + stream at 40 Hz ────────────────────────────────────────────
        client.set_heartbeat_flags(1)              # mpc_active=1 — the firmware guard may ENABLE
        armed = True
        streaming = True
        if args.observe:
            print(">>> OBSERVE MODE: faults are logged, not aborted "
                  "(deviation belt + e-stop stay live).")
        print(">>> ARMED. streaming ... (Ctrl-C to disarm)")
        t_arm = time.perf_counter()
        period = 1.0 / SETPOINT_HZ
        n = 0
        last_print = 0.0
        abort_reason = None
        seen_faults = set()
        while True:
            t = time.perf_counter() - t_arm
            if t >= args.duration:
                break
            now_us = int(time.time() * 1_000_000)
            if streaming:
                frame = gen.frame(t, t_origin_us=now_us)
                try:
                    client.send_stream(int(MsgType.SETPOINT), frame.pack())
                except OSError as e:
                    # Ethernet unplug (link-drop test) can make sendto raise. In
                    # observe mode that's expected — the firmware MPC_STALE-gates
                    # output; keep observing. Otherwise it's a real abort.
                    if not args.observe:
                        abort_reason = f"setpoint send failed: {e}"
                        break

            # ── observation snapshot (logged every tick) ──
            fs = fault_state()
            enc = axis_pos(axis)
            with _lock:
                tm = _cache["telem"]
            vel = None if tm is None else float(tm.vel_rps[axis])
            dg = axis_diag(axis)
            iq = None if dg is None else float(dg.iq_measured)
            cmd = gen.position(t)
            cmd_teensy = axis_cmd_teensy(axis)   # Teensy float32 interp output (LegCmd)
            # The friction-FF torque the frame carried this tick (matches the
            # frame's tau = torque_ff_fn(velocity(t))); blank on the plain arm.
            tau_ff = friction_ff_fn(gen.velocity(t)) if friction_ff_fn is not None else None
            fw_mpc = firmware_mpc_active()
            tage = telem_age_ms()
            csv_w.writerow([
                f"{t:.4f}", f"{cmd:.5f}",
                "" if enc is None else f"{enc:.5f}",
                "" if cmd_teensy is None else f"{cmd_teensy:.6f}",
                "" if vel is None else f"{vel:.5f}",
                "" if enc is None else f"{cmd - enc:.5f}",
                "" if iq is None else f"{iq:.4f}",
                "" if tau_ff is None else f"{tau_ff:.5f}",
                "" if fs is None else fs,
                "" if fs is None else _fault_name(fs),
                "" if tage is None else f"{tage:.0f}",
                hb_uptime_ms(), fw_mpc, int(streaming)])

            # ── safety monitor ──
            if fs not in (None, int(FaultState.NONE)):
                if args.observe:
                    if fs not in seen_faults:
                        seen_faults.add(fs)
                        print(f"  [observe] t={t:5.2f}s  FAULT → {fs} ({_fault_name(fs)})  "
                              f"enc={'?' if enc is None else f'{enc:+.4f}'}  "
                              f"iq={'?' if iq is None else f'{iq:+.2f}A'}")
                        if fs in _FATAL_FAULTS and streaming:
                            # Cede authority so the firmware stow / fatal handling runs
                            # uncontested; keep observing + logging to --duration.
                            client.set_heartbeat_flags(0)
                            streaming = False
                            print("  [observe] fatal/CAN fault → CEDING authority "
                                  "(disarm + stop streaming), still observing.")
                else:
                    abort_reason = f"fault_state={fs} ({_fault_name(fs)})"
                    break
            # Verify our arm took (competing-authority guard) — active while streaming.
            if t > ARM_VERIFY_GRACE_S and streaming and firmware_mpc_active() is False:
                abort_reason = ("firmware mpc_active=0 after arming — another heartbeat "
                                "authority is overriding it. Is the ROS2 launch "
                                "(teensy_bridge_node) running? Stop it and retry — this "
                                "driver must be the sole wire authority.")
                break
            # Deviation belt — runaway backstop, hard abort in BOTH modes.
            if enc is not None:
                expected = _clamp(cmd, STROKE_MIN_REV[axis], STROKE_MAX_REV[axis])
                if abs(expected - enc) > args.max_dev:
                    abort_reason = (f"tracking deviation {abs(expected - enc):.3f} rev "
                                    f"> {args.max_dev} (cmd≈{expected:+.3f}, enc={enc:+.3f})")
                    break

            if t - last_print >= 0.2:              # ~5 Hz status
                iqs = f"{iq:+.2f}A" if iq is not None else "?"
                hbup = hb_uptime_ms()
                print(f"  t={t:5.2f}s  cmd={cmd:+.4f}  "
                      f"enc={'?' if enc is None else f'{enc:+.4f}'}  "
                      f"err={'?' if enc is None else f'{cmd - enc:+.4f}'}  iq={iqs}  "
                      f"fault={fs}  strm={int(streaming)}  "
                      f"tage={'?' if tage is None else f'{tage:.0f}'}ms  hbup={hbup}")
                last_print = t

            n += 1
            # fixed-cadence sleep (drift-free): wake for frame n+1 at t_arm+(n+1)*period
            target = t_arm + (n + 1) * period
            dt = target - time.perf_counter()
            if dt > 0:
                time.sleep(dt)

        if abort_reason:
            print(f"\n!!! ABORT: {abort_reason} — disarming.")
        elif args.observe:
            faults = sorted(_fault_name(f) for f in seen_faults) or ["none"]
            print(f"\n--- duration {args.duration}s elapsed (observe) — disarming. "
                  f"faults seen: {', '.join(faults)}")
        else:
            print(f"\n--- duration {args.duration}s elapsed — disarming.")

    except KeyboardInterrupt:
        print("\n!!! Ctrl-C — disarming.")
    finally:
        # ALWAYS disarm. mpc_active=0 gates the firmware output off (≈ one fault step).
        if armed:
            client.set_heartbeat_flags(0)
            time.sleep(0.1)                        # let one disarmed heartbeat land
        if args.close_loop:
            try:
                rpc.call(int(RpcMethod.SET_AXIS_STATE),
                         rpc_args.encode_set_axis_state(axis, IDLE))
            except Exception:  # noqa: BLE001
                pass
        if csv_f is not None:
            try:
                csv_f.close()
            except Exception:  # noqa: BLE001
                pass
        enc = axis_pos(axis)
        print(f"final: pos={enc if enc is None else f'{enc:+.4f}'} rev  "
              f"fault_state={fault_state()}  (mpc_active=0)")
        tod.close()
        rpc_server.close()
        client.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
