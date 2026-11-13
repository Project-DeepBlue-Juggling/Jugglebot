#!/usr/bin/env python3
"""Bench Hand checks and a guarded USB home: prove the bench Hand's drive frame, then home it.

The bench Hand is production-identical (owner, 2026-11-11: its motor was moved to the robot's
side of the Hand axis; before that it was mirrored and the firmware HOME drove it UP). The
can-bridge Hand lane is production-frame (the 0..10.501 rev clip, and the C1 brake that fires
only on upward, i.e. positive, velocity, leg_interp.cpp), so the drive must count UP as
positive. After any change to the motor, its wiring or the encoder, prove that before a
closed loop runs:
  1. ``push-check``: Hand IDLE, the operator pushes it DOWN, the count must go NEGATIVE.
  2. ``torque-check``: two OPEN-LOOP torque pulses (no position or velocity loop, so a
     sign error cannot run away): 2 A down, then 4 A up (gravity holds at about 1.5 A).
     Each must move the Hand the commanded way, which proves that a positive torque moves
     the Hand in the positive direction. A failed pulse IDLEs at once.
If ``push-check`` fails, the mechanism or encoder is mirrored: stop, it is hardware. If
``torque-check`` fails, the commutation calibration no longer matches the wiring: run
``calibrate`` (Hand mid-stroke) and ``torque-check`` again. Calibration sets
``motor.direction`` itself (ODrive docs: "Do not modify this manually").

COMMANDS THE MOTOR (calibrate, torque-check, home), so it lives in tests/hardware/ and the
``*_bench.py`` name keeps pytest from collecting it. USB only: the launch stays down and the
bridge is never asked to move the Hand. A drive restart (power cycle, any save) loses the
position reference and latches the bridge's CAN_BUS_DOWN, so home after the last restart and
re-flash the bridge before the stage.

    python tests/hardware/bench_hand_frame_bench.py status
    python tests/hardware/bench_hand_frame_bench.py push-check      # push the Hand DOWN
    python tests/hardware/bench_hand_frame_bench.py torque-check    # Hand mid-stroke
    python tests/hardware/bench_hand_frame_bench.py calibrate       # only on a torque-check FAIL
    python tests/hardware/bench_hand_frame_bench.py home            # -3 rev/s, ref -0.10

``home`` is the can-bridge HOME (leg_homing.cpp, Homing::HAND_*) run over USB with guards
the firmware lacks: VELOCITY/VEL_RAMP at 3 rev/s, current cap 8 + 3 A, stop when the Iq EMA
(weight 0.7) reaches 8 A, IDLE, set the reference. It aborts to IDLE on a wrong-way move,
overspeed, too much travel, a drive error or a timeout, restores the mode and limits it
changed, and saves nothing. ``--down-sign +1 --ref 0.1`` homes a mirrored frame (the bench
before 2026-11-11).
"""
from __future__ import annotations

import argparse
import functools
import time

print = functools.partial(print, flush=True)  # noqa: A001

IDLE, CLOSED_LOOP, ENCODER_OFFSET_CALIBRATION = 1, 8, 7
TORQUE_CONTROL, VELOCITY_CONTROL, PASSTHROUGH, VEL_RAMP = 1, 2, 1, 2

# Homing (Homing::HAND_* in the generated config).
HOME_SPEED_RPS = 3.0
HOME_DETECT_A = 8.0
HOME_HEADROOM_A = 3.0
HOME_EMA_W = 0.7
HOME_STOP_SETTLE_S = 0.005
HOME_DRIVE_SETTLE_S = 0.010
WRONG_WAY_RPS = 0.5
OVERSPEED_RPS = 2.0 * HOME_SPEED_RPS
HOME_MAX_TRAVEL_REV = 12.0
HOME_TIMEOUT_S = 15.0

# Torque check: open loop, so a sign error pushes at a fixed current and cannot run away.
TC_DOWN_A = 2.0
TC_UP_A = 4.0
TC_CAP_A = 6.0
TC_PULSE_S = 0.4
TC_STOP_TRAVEL_REV = 0.3
TC_STOP_VEL_RPS = 2.0
TC_MIN_VEL_RPS = 0.2

PUSH_MIN_REV = 0.5
CAL_TIMEOUT_S = 30.0


# ---- pure decisions (tests/motion/test_bench_hand_frame.py) -------------------------------

def home_abort_reason(down_sign, vel, travel, err, state, t):
    """Why the guarded home must stop now, or None."""
    if err or state != CLOSED_LOOP:
        return "drive left closed loop or raised an error (state %s, err 0x%x)" % (state, err)
    if down_sign * vel < -WRONG_WAY_RPS:
        return "WRONG WAY: %+.2f rev/s against a %+.1f rev/s command" % (vel, down_sign * HOME_SPEED_RPS)
    if abs(vel) > OVERSPEED_RPS:
        return "OVERSPEED: %+.2f rev/s" % vel
    if abs(travel) > HOME_MAX_TRAVEL_REV:
        return "TRAVEL: %+.2f rev without a stall" % travel
    if t > HOME_TIMEOUT_S:
        return "TIMEOUT: no stall in %.0f s" % HOME_TIMEOUT_S
    return None


def ema_step(ema, iq):
    return HOME_EMA_W * ema + (1.0 - HOME_EMA_W) * iq


def push_verdict(delta_rev, expect_down_sign=-1.0):
    """The operator pushed the Hand DOWN; the count must move with expect_down_sign."""
    if abs(delta_rev) < PUSH_MIN_REV:
        return False, "moved only %+.3f rev; push it further (at least %.1f rev)" % (delta_rev, PUSH_MIN_REV)
    if delta_rev * expect_down_sign > 0:
        return True, "DOWN counts %s (%+.3f rev): PASS" % ("negative" if expect_down_sign < 0 else "positive", delta_rev)
    return False, "DOWN counts %s (%+.3f rev): the frame is NOT the expected one" % (
        "positive" if delta_rev > 0 else "negative", delta_rev)


def pulse_stop_reason(travel, vel, torque_sign, t):
    """When a torque pulse ends: it moved far or fast enough, went the wrong way, or timed out."""
    if torque_sign * vel < -TC_MIN_VEL_RPS:
        return "wrong"
    if abs(travel) >= TC_STOP_TRAVEL_REV or abs(vel) >= TC_STOP_VEL_RPS:
        return "moved"
    if t >= TC_PULSE_S:
        return "time"
    return None


def pulse_verdict(torque_sign, peak_signed_vel, travel):
    """A pulse passes when the Hand moved the commanded way by speed and by travel."""
    if torque_sign * peak_signed_vel >= TC_MIN_VEL_RPS and torque_sign * travel > 0:
        return True
    return False


# ---- the drive (USB) -----------------------------------------------------------------------

def connect(timeout=20.0):
    import odrive
    return odrive.find_sync(timeout=timeout)


def describe(d):
    a = d.axis0
    return ("state %d err 0x%x disarm %d | pos %+.4f | pos_vel_mapper.scale %+.0f | motor.direction %+.0f | "
            "commutation scale %+.0f offset %+.4f | node %d | vbus %.1f V" % (
                a.current_state, a.active_errors, a.disarm_reason, a.pos_estimate,
                a.pos_vel_mapper.config.scale, a.config.motor.direction,
                a.commutation_mapper.config.scale, a.commutation_mapper.config.offset,
                a.config.can.node_id, d.vbus_voltage))


def require_idle_clean(d):
    a = d.axis0
    if a.current_state != IDLE or a.active_errors:
        raise SystemExit("REFUSED: axis must be IDLE with no errors (%s)" % describe(d))


def save_and_reconnect(d):
    """save_configuration() restarts the drive (FW 0.6.11) and drops USB; find it again."""
    import odrive
    serial = format(d.serial_number, "X")
    try:
        d.save_configuration()
        print("save returned without a restart")
    except Exception as e:  # DeviceLostException is the expected path
        print("saved; drive restarting (%s)" % type(e).__name__)
    time.sleep(3.0)
    return odrive.find_sync(serial_number=serial, timeout=30.0)


def cmd_status(args):
    print(describe(connect()))


def cmd_push_check(args):
    d = connect()
    a = d.axis0
    require_idle_clean(d)
    print("Recording %.0f s: push the Hand DOWN slowly by a few cm, then let go." % args.seconds)
    p0 = a.pos_estimate
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < args.seconds:
        if time.time() - last > 1.0:
            print("  t=%4.1fs pos=%+.3f (d=%+.3f)" % (time.time() - t0, a.pos_estimate, a.pos_estimate - p0))
            last = time.time()
        time.sleep(0.02)
    ok, msg = push_verdict(a.pos_estimate - p0, args.expect_down_sign)
    print(msg)
    if not ok:
        raise SystemExit(1)


def cmd_calibrate(args):
    d = connect()
    a = d.axis0
    require_idle_clean(d)
    print("before: " + describe(d))
    print("Encoder offset calibration: an open-loop sweep of about 1.1 rev each way at %.0f A. "
          "The Hand must be at least 1.5 rev from both stops." % a.config.calibration_lockin.current)
    p0 = a.pos_estimate
    lo = hi = p0
    a.requested_state = ENCODER_OFFSET_CALIBRATION
    time.sleep(0.2)
    t0 = time.time()
    while a.current_state != IDLE and time.time() - t0 < CAL_TIMEOUT_S:
        x = a.pos_estimate
        lo, hi = min(lo, x), max(hi, x)
        time.sleep(0.02)
    if a.current_state != IDLE:
        a.requested_state = IDLE
        raise SystemExit("FAIL: calibration did not finish in %.0f s; forced IDLE" % CAL_TIMEOUT_S)
    time.sleep(0.2)
    print("swept %+.3f..%+.3f rev; procedure_result %d, errors 0x%x" % (
        lo - p0, hi - p0, a.procedure_result, a.active_errors))
    if a.procedure_result != 0 or a.active_errors:
        raise SystemExit("FAIL: calibration did not succeed; nothing saved")
    d = save_and_reconnect(d)
    print("after (saved): " + describe(d))
    print("NEXT: torque-check again")


def _pulse(d, amps, up_sign):
    a = d.axis0
    torque_sign = up_sign if amps > 0 else -up_sign
    kt = a.config.motor.torque_constant
    p0 = a.pos_estimate
    a.controller.input_torque = torque_sign * abs(amps) * kt
    t0 = time.time()
    peak = 0.0
    reason = None
    while reason is None:
        if a.active_errors or a.current_state != CLOSED_LOOP:
            reason = "error"
            break
        v, x = a.vel_estimate, a.pos_estimate
        if abs(v) > abs(peak):
            peak = v
        reason = pulse_stop_reason(x - p0, v, torque_sign, time.time() - t0)
        time.sleep(0.005)
    a.controller.input_torque = 0.0
    travel = a.pos_estimate - p0
    return torque_sign, peak, travel, reason


def cmd_torque_check(args):
    d = connect()
    a = d.axis0
    c = a.controller.config
    require_idle_clean(d)
    orig = dict(control_mode=c.control_mode, input_mode=c.input_mode,
                tmvl=c.enable_torque_mode_vel_limit, soft_max=a.config.motor.current_soft_max)
    print("before: " + describe(d))
    print("Open-loop torque pulses, up = %+d: %.0f A down, then %.0f A up; cap %.0f A; each ends at "
          "%.1f rev or %.0f rev/s." % (args.up_sign, TC_DOWN_A, TC_UP_A, TC_CAP_A, TC_STOP_TRAVEL_REV, TC_STOP_VEL_RPS))
    results = []
    try:
        a.controller.input_torque = 0.0
        c.enable_torque_mode_vel_limit = False   # pure torque: no velocity feedback at all
        a.config.motor.current_soft_max = TC_CAP_A
        c.control_mode = TORQUE_CONTROL
        c.input_mode = PASSTHROUGH
        a.requested_state = CLOSED_LOOP
        t = time.time()
        while a.current_state != CLOSED_LOOP:
            if time.time() - t > 1.0 or a.active_errors:
                raise RuntimeError("closed loop not entered: " + describe(d))
            time.sleep(0.005)
        for label, amps in (("down", -TC_DOWN_A), ("up", TC_UP_A)):
            sign, peak, travel, reason = _pulse(d, amps, args.up_sign)
            ok = reason != "error" and pulse_verdict(sign, peak, travel)
            results.append(ok)
            print("  %-4s torque %+d: peak vel %+.2f rev/s, travel %+.3f rev, ended on %s -> %s" % (
                label, sign, peak, travel, reason, "PASS" if ok else "FAIL"))
            time.sleep(0.3)
            if not ok:
                break
    except BaseException as e:  # noqa: BLE001 (incl. Ctrl-C)
        print("ABORT: %r" % (e,))
        results.append(False)
    finally:
        a.requested_state = IDLE
        time.sleep(0.05)
        a.controller.input_torque = 0.0
        c.control_mode, c.input_mode = orig["control_mode"], orig["input_mode"]
        c.enable_torque_mode_vel_limit = orig["tmvl"]
        a.config.motor.current_soft_max = orig["soft_max"]
        time.sleep(0.2)
        print("after (nothing saved): " + describe(d))
    if len(results) == 2 and all(results):
        print("TORQUE-CHECK PASS: positive torque moves the Hand the positive (up = %+d) way" % args.up_sign)
    else:
        raise SystemExit("TORQUE-CHECK FAIL: run no closed loop; calibrate (Hand mid-stroke), then torque-check again")


def run_home(d, down_sign, ref, sample=None):
    """Guarded home. Returns (ok, message). ``sample`` overrides the drive reads in tests."""
    a = d.axis0
    c = a.controller.config
    orig = dict(control_mode=c.control_mode, input_mode=c.input_mode,
                vel_limit=c.vel_limit, soft_max=a.config.motor.current_soft_max)
    if sample is None:
        def sample():
            return (a.motor.foc.Iq_measured, a.vel_estimate, a.pos_estimate, a.active_errors, a.current_state)
    p0 = a.pos_estimate
    try:
        a.controller.input_vel = 0.0
        c.vel_limit = 2.0 * HOME_SPEED_RPS
        a.config.motor.current_soft_max = HOME_DETECT_A + HOME_HEADROOM_A
        c.control_mode, c.input_mode = VELOCITY_CONTROL, VEL_RAMP
        a.requested_state = CLOSED_LOOP
        t = time.time()
        while a.current_state != CLOSED_LOOP:
            if time.time() - t > 1.0 or a.active_errors:
                return False, "closed loop not entered (state %d, err 0x%x)" % (a.current_state, a.active_errors)
            time.sleep(0.005)
        time.sleep(HOME_DRIVE_SETTLE_S)
        a.controller.input_vel = down_sign * HOME_SPEED_RPS
        t0 = time.time()
        ema = 0.0
        last = -1.0
        while True:
            now = time.time() - t0
            iq, vel, pos, err, state = sample()
            ema = ema_step(ema, iq)
            if now - last > 0.1:
                print("  t=%5.2fs pos=%+.3f vel=%+.2f iq=%+.2f ema=%+.2f" % (now, pos, vel, iq, ema))
                last = now
            why = home_abort_reason(down_sign, vel, pos - p0, err, state, now)
            if why:
                a.requested_state = IDLE
                return False, why
            if abs(ema) >= HOME_DETECT_A:
                a.requested_state = IDLE
                time.sleep(HOME_STOP_SETTLE_S)
                x_stop = a.pos_estimate
                a.set_abs_pos(ref)
                return True, "STALL at %.2f s, %+.3f rev from the start; reference set to %+.2f" % (
                    now, x_stop - p0, ref)
            time.sleep(0.008)
    except BaseException as e:  # noqa: BLE001 (incl. Ctrl-C)
        a.requested_state = IDLE
        return False, "aborted: %r" % (e,)
    finally:
        if a.current_state != IDLE:
            a.requested_state = IDLE
        time.sleep(0.05)
        a.controller.input_vel = 0.0
        c.control_mode, c.input_mode = orig["control_mode"], orig["input_mode"]
        c.vel_limit = orig["vel_limit"]
        a.config.motor.current_soft_max = orig["soft_max"]


def cmd_home(args):
    d = connect()
    require_idle_clean(d)
    print("before: " + describe(d))
    print(">>> HOME: %+.1f rev/s (down), cap %.0f A, stall at Iq EMA %.0f A, reference %+.2f. E-stop ready." % (
        args.down_sign * HOME_SPEED_RPS, HOME_DETECT_A + HOME_HEADROOM_A, HOME_DETECT_A, args.ref))
    ok, msg = run_home(d, args.down_sign, args.ref)
    time.sleep(0.2)
    print(("HOME OK: " if ok else "HOME FAILED -> IDLE: ") + msg)
    print("after (nothing saved): " + describe(d))
    if not ok:
        raise SystemExit(1)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status").set_defaults(fn=cmd_status)
    pc = sub.add_parser("push-check")
    pc.add_argument("--seconds", type=float, default=20.0)
    pc.add_argument("--expect-down-sign", type=float, default=-1.0,
                    help="-1: DOWN counts negative (production frame); +1 for a mirrored one")
    pc.set_defaults(fn=cmd_push_check)
    sub.add_parser("calibrate").set_defaults(fn=cmd_calibrate)
    tc = sub.add_parser("torque-check")
    tc.add_argument("--up-sign", type=int, choices=(-1, 1), default=1,
                    help="+1: up is positive (production frame); -1 for a mirrored one")
    tc.set_defaults(fn=cmd_torque_check)
    h = sub.add_parser("home")
    h.add_argument("--down-sign", type=float, choices=(-1.0, 1.0), default=-1.0)
    h.add_argument("--ref", type=float, default=-0.1, help="reference at the bottom stop (production -0.10)")
    h.set_defaults(fn=cmd_home)
    args = p.parse_args(argv)
    args.fn(args)


if __name__ == "__main__":
    main()
