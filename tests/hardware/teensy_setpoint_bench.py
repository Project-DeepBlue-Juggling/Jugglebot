#!/usr/bin/env python3
"""Bench driver: FIRST armed MPC-style setpoint stream on one can-bridge leg (Phase 11).

This is the highest-risk bench harness in the project so far — the **first
continuous powered setpoint stream** to a leg over the can-bridge. Unlike encoder
search (9a) and homing (9b), which are bounded self-terminating moves, this
streams a 40 Hz β-knot trajectory that the leg tracks for as long as it runs. It
arms the firmware output gate (``mpc_active=1`` via the J→T heartbeat) and drives
the **synthetic** β-knot source (``controller/teensy_link/synthetic_setpoint.py``)
— a KNOWN, bounded command that exercises the Teensy's 40 Hz-knot Hermite interp
(``leg_interp.cpp`` Mode 1) directly, decoupled from motor_guard / the MPC /
friction-FF. The deployed production bridge is untouched (still the α relay).

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
  • encoder search (9a) + homing (9b) done — valid encoder reference.
  • axis in CLOSED_LOOP **position** mode with sane gains. The driver does NOT
    configure gains/mode (energising + tuning is the operator's call); bring the
    leg to CLOSED_LOOP first, or pass --close-loop to have the driver issue
    SET_AXIS_STATE(CLOSED_LOOP) (assumes mode+gains already set).
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
import sys
import threading
import time

sys.path.insert(0, "/home/jetson/Desktop/Jugglebot")
from controller.teensy_link import (  # noqa: E402
    TeensyLinkClient, RpcClient, RpcServer, TimeOfDayServer, MsgType,
    Telemetry, Diagnostic, RpcMethod, FaultState,
)
from controller.teensy_link import protocol as p  # noqa: E402
from controller.teensy_link import rpc_args  # noqa: E402
from controller.teensy_link.synthetic_setpoint import (  # noqa: E402
    SyntheticKnotSource, TrajectoryParams,
)

TEENSY_IP = "192.168.42.2"
CLOSED_LOOP = 8
IDLE = 1

# Per-axis stroke clamp bounds — MUST match canbridge_config.h STROKE_{MIN,MAX}_REV.
STROKE_MIN_REV = [0.070917, 0.070954, 0.070448, 0.070934, 0.071340, 0.071248]
STROKE_MAX_REV = [3.900413, 3.902459, 3.874629, 3.901381, 3.923703, 3.918615]

SETPOINT_HZ = 40.0                  # MUST match the firmware SEGMENT_T_S = 0.025 s
DEVIATION_ABORT_REV = 0.30          # driver belt; firmware MAX_DEVIATION (0.5) is the backstop

_lock = threading.Lock()
_cache = {"telem": None, "diag": {}, "hb": None}


def on_telem(mt, seq, payload, addr):
    tm = Telemetry.unpack(payload)
    with _lock:
        _cache["telem"] = tm


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


def fault_state():
    with _lock:
        hb = _cache["hb"]
    return None if hb is None else int(hb.fault_state)


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
                    help="issue SET_AXIS_STATE(CLOSED_LOOP) before arming "
                         "(assumes position mode + gains already configured)")
    ap.add_argument("--arm", action="store_true",
                    help="skip the interactive ARM prompt (for scripted reruns)")
    args = ap.parse_args()
    axis = args.axis

    client = TeensyLinkClient(teensy_addr=(TEENSY_IP, p.PORT_STREAM), bind_host="0.0.0.0")
    client.start()
    rpc_server = RpcServer(client)
    tod = TimeOfDayServer(rpc_server)
    rpc = RpcClient(client)
    client.subscribe(int(MsgType.TELEMETRY), on_telem)
    client.subscribe(int(MsgType.DIAGNOSTIC), on_diag)
    client.subscribe(int(MsgType.HEARTBEAT_T2J), on_hb)
    client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)   # mpc_active=0 — DISARMED
    client.set_heartbeat_flags(0)

    armed = False
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

        # ── Optional: close the loop ─────────────────────────────────────────
        if args.close_loop:
            print(f">>> SET_AXIS_STATE({axis}, CLOSED_LOOP) ...")
            rpc.call(int(RpcMethod.SET_AXIS_STATE),
                     rpc_args.encode_set_axis_state(axis, CLOSED_LOOP))
            t1 = time.time()
            while time.time() - t1 < 3.0:
                d = axis_diag(axis)
                if d is not None and int(d.axis_state) == CLOSED_LOOP:
                    break
                time.sleep(0.05)

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

        # ── Build the bounds-validated synthetic trajectory ──────────────────
        try:
            gen = SyntheticKnotSource(
                TrajectoryParams(axis=axis, start_rev=start, center_rev=args.center,
                                 amplitude_rev=args.amplitude, freq_hz=args.freq,
                                 approach_s=args.approach),
                stroke_min_rev=STROKE_MIN_REV[axis], stroke_max_rev=STROKE_MAX_REV[axis])
        except ValueError as e:
            print(f"ABORT — trajectory rejected: {e}")
            return 2

        kind = "HOLD" if args.amplitude == 0.0 else f"SINUSOID ±{args.amplitude} @ {args.freq} Hz"
        print("\n── planned motion ─────────────────────────────────────────────")
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

        # ── ARM + stream at 40 Hz ────────────────────────────────────────────
        client.set_heartbeat_flags(1)              # mpc_active=1 — the firmware guard may ENABLE
        armed = True
        print(">>> ARMED. streaming ... (Ctrl-C to disarm)")
        t_arm = time.perf_counter()
        period = 1.0 / SETPOINT_HZ
        n = 0
        last_print = 0.0
        abort_reason = None
        while True:
            t = time.perf_counter() - t_arm
            if t >= args.duration:
                break
            now_us = int(time.time() * 1_000_000)
            frame = gen.frame(t, t_origin_us=now_us)
            client.send_stream(int(MsgType.SETPOINT), frame.pack())

            # ── safety monitor ──
            fs = fault_state()
            if fs not in (None, int(FaultState.NONE)):
                abort_reason = f"fault_state={fs}"
                break
            enc = axis_pos(axis)
            if enc is not None:
                expected = _clamp(gen.position(t), STROKE_MIN_REV[axis], STROKE_MAX_REV[axis])
                if abs(expected - enc) > DEVIATION_ABORT_REV:
                    abort_reason = (f"tracking deviation {abs(expected - enc):.3f} rev "
                                    f"> {DEVIATION_ABORT_REV} (cmd≈{expected:+.3f}, enc={enc:+.3f})")
                    break

            if t - last_print >= 0.2:              # ~5 Hz status
                dg = axis_diag(axis)
                iq = f"{dg.iq_measured:+.2f}A" if dg else "?"
                print(f"  t={t:5.2f}s  cmd={gen.position(t):+.4f}  enc={enc:+.4f}  "
                      f"err={(gen.position(t) - enc):+.4f}  iq={iq}  fault={fs}")
                last_print = t

            n += 1
            # fixed-cadence sleep (drift-free): wake for frame n+1 at t_arm+(n+1)*period
            target = t_arm + (n + 1) * period
            dt = target - time.perf_counter()
            if dt > 0:
                time.sleep(dt)

        if abort_reason:
            print(f"\n!!! ABORT: {abort_reason} — disarming.")
        else:
            print(f"\n--- duration {args.duration}s elapsed — disarming.")

    except KeyboardInterrupt:
        print("\n!!! Ctrl-C — disarming.")
    finally:
        # ALWAYS disarm. mpc_active=0 instantly gates the firmware output off.
        if armed:
            client.set_heartbeat_flags(0)
            time.sleep(0.1)                        # let one disarmed heartbeat land
        if args.close_loop:
            try:
                rpc.call(int(RpcMethod.SET_AXIS_STATE),
                         rpc_args.encode_set_axis_state(axis, IDLE))
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
