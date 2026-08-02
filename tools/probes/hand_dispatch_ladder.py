#!/usr/bin/env python3
"""Bench probe: HAND_TRAJ_CMD dispatch ladder with counter snapshots.

Operator-run bench probe (Step 3 of the ERR_TIMEOUT investigation). Actuating
in the narrow sense that each dispatch energises the hand ODrive (preamble = CLOSED_LOOP) — the move
itself is dispatched to the hand's CURRENT position (mechanically null) so the
full 3-send firmware path (set_state, set_controller_mode, 0x6D0 kind-3 frame)
is exercised without commanded motion. See
tests/hardware/session_err_timeout_bench.md for arms, safety and PASS/ABORT.

Why the real stack and not a synthetic UDP client: a stub would hand-craft
ArgHandTraj payloads (deadline semantics = stroke risk), collide with the
teensy_link single-owner UDP constraint, and measure a path production never
takes. This ladder measures the production dispatch path end to end, and reads
the attribution counters straight off /link_status.

Usage (launch running, bridge FW >= 9 for bridge_tx_diag):
    python tools/probes/hand_dispatch_ladder.py --n 40 --gap 2.0 \
        [--label armA] [--out temp/probes/]

Output: one CSV row per dispatch (wall time, uptime_ms, pos_meas, outcome,
message, counter rows verbatim) + a start/end counter snapshot and a tally.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticStatus
from jugglebot_interfaces.msg import HandTelemetryMessage
from jugglebot_interfaces.srv import SetFloat

# /link_status rows captured verbatim per dispatch. can3_errors is the
# invalidation guard (any wire-error tick voids the congestion read);
# uptime_ms is the standing-rule companion to every measurement.
ROWS = ("hand_traj_acks", "bridge_tx_diag", "can3_errors", "uptime_ms",
        "setpoints_sent", "mpc_active", "bridge_fw_version")

# Park-band velocity noise reaches ~5.4 rev/s p99 on a parked-TOP hand
# (probe-validated, 2026-07-24 entry) — warn, don't gate, above this.
VEL_WARN_REV_S = 6.0


class Ladder(Node):
    def __init__(self):
        super().__init__("hand_dispatch_ladder")
        self._pos = None
        self._vel = None
        self._kv = {}
        self.create_subscription(HandTelemetryMessage, "hand_telemetry",
                                 self._on_hand, 10)
        self.create_subscription(DiagnosticStatus, "link_status",
                                 self._on_link, 10)
        self._cli = self.create_client(SetFloat, "smooth_move_hand")

    def _on_hand(self, m):
        self._pos, self._vel = m.pos_meas, m.vel_meas

    def _on_link(self, m):
        self._kv = {v.key: v.value for v in m.values}

    def snapshot(self):
        return {k: self._kv.get(k, "<absent>") for k in ROWS}

    def wait_ready(self, timeout=15.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._pos is not None and self._kv and self._cli.service_is_ready():
                return True
        return False

    def dispatch_null_move(self, timeout=5.0):
        """One smooth_move_hand to the current position; returns (ok, message)."""
        if abs(self._vel or 0.0) > VEL_WARN_REV_S:
            self.get_logger().warning(
                f"hand vel {self._vel:.2f} rev/s at dispatch — not stationary?")
        req = SetFloat.Request()
        req.data = float(self._pos)
        fut = self._cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout:
                return False, "<service timeout>"
        r = fut.result()
        return bool(r.success), str(r.message)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--gap", type=float, default=2.0,
                    help="seconds between dispatches (keep >= 2.0: outside any "
                         "retry/quiet window, and hand-command-continuity safe)")
    ap.add_argument("--label", default="arm")
    ap.add_argument("--out", default="temp/probes")
    args = ap.parse_args()

    rclpy.init()
    node = Ladder()
    if not node.wait_ready():
        print("ABORT: telemetry/link_status/service not ready in 15 s", file=sys.stderr)
        return 2

    os.makedirs(args.out, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    path = os.path.join(args.out, f"hand_dispatch_ladder_{args.label}_{stamp}.csv")

    start = node.snapshot()
    print(f"[ladder] {args.label}: start snapshot: {start}")
    ok_n = fail_n = 0
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_wall", "i", "pos_meas_rev", "ok", "message", *ROWS])
        for i in range(args.n):
            t_next = time.monotonic() + args.gap
            ok, msg = node.dispatch_null_move()
            ok_n += ok
            fail_n += (not ok)
            snap = node.snapshot()
            w.writerow([f"{time.time():.6f}", i, f"{node._pos:.4f}",
                        int(ok), msg, *[snap[k] for k in ROWS]])
            fh.flush()
            print(f"[ladder] {i + 1}/{args.n} {'OK ' if ok else 'FAIL'} {msg}"
                  f"  acks={snap['hand_traj_acks']}")
            while time.monotonic() < t_next:
                rclpy.spin_once(node, timeout_sec=0.1)

    end = node.snapshot()
    print(f"[ladder] {args.label}: end snapshot: {end}")
    print(f"[ladder] tally: {ok_n} OK / {fail_n} FAIL of {args.n} "
          f"({100.0 * fail_n / max(args.n, 1):.1f}% fail)  csv={path}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
