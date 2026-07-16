#!/usr/bin/env python3
"""Phase-4 limit-ramp move battery — scripted go_to_pose calls (OPERATOR-run).

A read-mostly ROS2 client that fires the Phase-4 ramp battery through the public
``trajectory/*`` services and prints each move's accept/reject + returned duration.
It does NOT touch limits unless you pass ``--set-*`` (the per-step ramp knob),
and it never arms the bridge — arming (``set_setpoint_output``) and the mode
sequence stay the operator's job. Motion only actually happens if the platform is
already armed and in TRAJECTORY mode; otherwise every move is rejected loudly
(``WRONG_MODE`` / ``STALE_STATE``) and nothing moves.

Per the hardware protocol (``tests/hardware/session_phase4_ramp.md``): raise ONE
session limit ~1.5× via ``--set-*``, run this battery, then ``/diagnose --latest``
to review realized leg peaks + jerk headroom, and only persist the bump to
``hardware_config.yaml`` on an operator PASS.

Battery (all rest-to-rest, at the current session limits):
  z 170→190→170 ; x ±20 ; y ±20 ; tilt rx ±3° ; then one deliberately-infeasible
  ``duration_s = 0.05`` request that MUST come back ``TOO_FAST`` with a
  ``min_duration_s`` (the loud-rejection proof) and move nothing.

Lean A/B (``--lean-gain``): pass ``0.0`` and then ``0.3`` on two runs of the same
battery to compare measured leg jerk / calmness (kept only if jerk drops).

Usage (operator, with the stack up + platform armed in TRAJECTORY mode):
    python3 tests/hardware/traj_ramp_battery.py --lean-gain 0.0
    python3 tests/hardware/traj_ramp_battery.py --set-jerk 12000 --lean-gain 0.0
    python3 tests/hardware/traj_ramp_battery.py --dry-run          # print, no calls

ABORT (operator) at any oscillation / audible snap / E-STOP / tracking > 0.1 rev —
then revert the YAML session limits to the last-good values.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

NEUTRAL_Z = 170.0

# (label, x, y, z, rx_deg, duration_s, expect_accept)
_BATTERY = [
    ('z 170→190',    0.0,   0.0, 220.0, 0.0, 0.0, True),
    ('z 190→170',    0.0,   0.0, 170.0, 0.0, 0.0, True),
    ('x +20',       150.0,   0.0, 170.0, 0.0, 0.0, True),
    ('x -20',      -150.0,   0.0, 170.0, 0.0, 0.0, True),
    ('x 0',          0.0,   0.0, 170.0, 0.0, 0.0, True),
    ('y +20',        0.0,  150.0, 170.0, 0.0, 0.0, True),
    ('y -20',        0.0, -150.0, 170.0, 0.0, 0.0, True),
    ('y 0',          0.0,   0.0, 170.0, 0.0, 0.0, True),
    ('tilt rx +3°',  0.0,   0.0, 170.0,  10.0, 0.0, True),
    ('tilt rx -3°',  0.0,   0.0, 170.0, -10.0, 0.0, True),
    ('tilt rx 0',    0.0,   0.0, 170.0,  0.0, 0.0, True),
    # Deliberately infeasible — must be rejected TOO_FAST, zero motion.
    ('TOO_FAST demo (dur 0.05)', 20.0, 20.0, 185.0, 0.0, 0.05, False),
]


def _quat_rx(rx_deg: float):
    """Quaternion (x, y, z, w) for a rotation of ``rx_deg`` about the world x-axis."""
    h = math.radians(rx_deg) / 2.0
    return (math.sin(h), 0.0, 0.0, math.cos(h))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--lean-gain', type=float, default=0.0,
                    help='per-call lean gain for every move (A/B: 0.0 vs 0.3). '
                         'Negative defers to the node config JB_TRAJ_LEAN_GAIN.')
    ap.add_argument('--set-vel', type=float, default=0.0,
                    help='ramp the session leg velocity limit (mm/s) before the '
                         'battery; 0 = leave unchanged (clamped to the YAML ceiling).')
    ap.add_argument('--set-acc', type=float, default=0.0,
                    help='ramp the session leg acceleration limit (mm/s²); 0 = keep.')
    ap.add_argument('--set-jerk', type=float, default=0.0,
                    help='ramp the session leg jerk limit (mm/s³); 0 = keep.')
    ap.add_argument('--settle-s', type=float, default=1.0,
                    help='wall-clock pause between moves (let each move + hold '
                         'complete before the next is issued).')
    ap.add_argument('--timeout-s', type=float, default=5.0,
                    help='per-service-call timeout.')
    ap.add_argument('--dry-run', action='store_true',
                    help='print the battery plan and exit without any ROS calls.')
    args = ap.parse_args()

    print(f"Phase-4 ramp battery — lean_gain={args.lean_gain}  "
          f"set(vel={args.set_vel}, acc={args.set_acc}, jerk={args.set_jerk})")
    for label, x, y, z, rx, dur, expect in _BATTERY:
        exp = 'accept' if expect else 'reject(TOO_FAST)'
        print(f"  - {label:28s} pose=({x:+.0f},{y:+.0f},{z:.0f}) "
              f"rx={rx:+.0f}° dur={dur:.2f}s → expect {exp}")
    if args.dry_run:
        print("dry-run: no ROS calls made.")
        return 0

    import rclpy
    from jugglebot_interfaces.srv import GoToPose, SetTrajectoryLimits
    from geometry_msgs.msg import Pose, Point, Quaternion

    rclpy.init()
    node = rclpy.create_node('traj_ramp_battery')
    go = node.create_client(GoToPose, '/trajectory/go_to_pose')
    setlim = node.create_client(SetTrajectoryLimits, '/trajectory/set_limits')

    def call(client, req, name):
        if not client.wait_for_service(timeout_sec=args.timeout_s):
            print(f"  !! service {name} unavailable — is the stack up?")
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout_s)
        return fut.result()

    rc = 0
    try:
        if args.set_vel or args.set_acc or args.set_jerk:
            req = SetTrajectoryLimits.Request()
            req.leg_vel_limit_mmps = float(args.set_vel)
            req.leg_acc_limit_mmps2 = float(args.set_acc)
            req.leg_jerk_limit_mmps3 = float(args.set_jerk)
            resp = call(setlim, req, 'set_limits')
            if resp is not None:
                print(f"set_limits applied: {resp.message}")

        for label, x, y, z, rx, dur, expect in _BATTERY:
            req = GoToPose.Request()
            qx, qy, qz, qw = _quat_rx(rx)
            req.pose = Pose(position=Point(x=x, y=y, z=z),
                            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
            req.duration_s = float(dur)
            req.lean_gain = float(args.lean_gain)
            resp = call(go, req, 'go_to_pose')
            if resp is None:
                rc = 1
                continue
            tag = 'OK ' if resp.accepted == expect else 'XX '
            if resp.accepted != expect:
                rc = 1
            # Settle long enough for the move ITSELF to finish before the next
            # request, or the next go_to_pose is rejected BUSY mid-battery. Lean-
            # shaped moves plan LONGER than their unshaped twins (the gate sizes the
            # added tilt), so a fixed settle shorter than a shaped duration cascades
            # BUSY rejections — wait max(settle, planned + 0.5 s). planned_duration_s
            # is printed per move so the A/B operator sees the unequal shaped/unshaped
            # durations directly.
            settle = max(args.settle_s, resp.planned_duration_s + 0.5)
            print(f"[{tag}] {label:28s} accepted={resp.accepted} "
                  f"code={resp.code} dur={resp.planned_duration_s:.3f}s "
                  f"min={resp.min_duration_s:.3f}s settle={settle:.2f}s "
                  f":: {resp.message}")
            time.sleep(settle)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("battery complete — run  /diagnose --latest  to review per-move peaks + "
          "jerk headroom before persisting any limit bump.")
    return rc


if __name__ == '__main__':
    sys.exit(main())
