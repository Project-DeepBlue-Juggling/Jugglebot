#!/usr/bin/env python3
"""Live observer for the MVP trajectory generator's 40 Hz :5557 knot stream.

READ-ONLY. Opens a ZMQ SUB on ``tcp://127.0.0.1:5557`` (topic ``mpccmd``) — it
**connects**, never binds, and never sends anything on any wire — so it cannot
command a motor and coexists with the live ``trajectory_node`` / ``run_mpc.py``.
Decodes each ``make_mpc_command`` frame and prints the observed stream rate, the
per-leg ``u0`` (= ``motor_rev``), and the max per-knot ``|Δu0|`` step, so an
operator can confirm during the Phase 1 arming bring-up that:

  * a steady ~40 Hz hold stream is flowing (before arming),
  * ``u0`` sits at the activate revs (~2.19 rev) — i.e. a bumpless hold at the
    active pose,
  * the per-knot step stays well under the pump's 0.3 rev gate (a hold is ~0).

Feeds the exact frame each observed knot carries through a real ``SetpointPump``
so the printed "pump" column mirrors what ``teensy_bridge_node`` would decide —
the same production-in-the-loop check the tests assert, but live on the bench.

Motivated by: ``plans/active/mvp-trajectory-bringup.md`` Phase 1 (hardware
session ``tests/hardware/session_phase1_hold.md``); logbook
``2026-07-07-mvp-phase1-streaming-foundation.md``.

Usage (from the repo root; self-bootstraps sys.path — no venv PYTHONPATH needed):
    python tools/probes/traj_stream_probe.py [--duration S] [--addr tcp://127.0.0.1:5557]

Outputs a one-line-per-second summary to stdout and a CSV to
``temp/probes/traj_stream_probe_<ts>.csv``. Ctrl-C to stop early.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from datetime import datetime

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (_REPO_ROOT,
           os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO_ROOT, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import msgpack  # noqa: E402
import zmq  # noqa: E402

import hardware_config as hw  # noqa: E402  (config/generated on path)
from controller.teensy_link.setpoint_pump import SetpointPump  # noqa: E402


def _out_dir() -> str:
    d = os.path.join(_REPO_ROOT, 'temp', 'probes')
    os.makedirs(d, exist_ok=True)
    return d


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--addr', default='tcp://127.0.0.1:5557')
    ap.add_argument('--topic', default='mpccmd')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='seconds to observe (0 = until Ctrl-C)')
    args = ap.parse_args()

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RECONNECT_IVL, 100)
    sub.setsockopt(zmq.RECONNECT_IVL_MAX, 200)
    sub.setsockopt(zmq.RCVHWM, 200)
    sub.connect(args.addr)                       # CONNECT — never bind.
    sub.setsockopt(zmq.SUBSCRIBE, args.topic.encode())
    sub.setsockopt(zmq.RCVTIMEO, 100)

    pump = SetpointPump(mm_to_rev=hw.GEOM_MM_TO_REV,
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV)

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(_out_dir(), f'traj_stream_probe_{ts}.csv')
    csv = open(csv_path, 'w')
    csv.write('t_wall,rate_hz,u0_mean_rev,max_step_rev,pump_rejects\n')

    print(f"[traj_stream_probe] SUB {args.addr} topic={args.topic!r} "
          f"(READ-ONLY) → {csv_path}")
    print(f"{'time':>8} {'rate_hz':>8} {'u0_mean':>9} {'max_step':>9} "
          f"{'pump_rej':>9}  u0[0..5] (rev)")

    t_start = time.perf_counter()
    window_start = t_start
    count = 0
    prev_u0 = None
    max_step = 0.0
    last_u0 = None
    try:
        while True:
            now = time.perf_counter()
            try:
                frames = sub.recv_multipart()
                cmd = msgpack.unpackb(frames[1], raw=False)
            except zmq.Again:
                cmd = None
            if cmd is not None:
                count += 1
                sp, _reason = pump.build(cmd, int(time.time() * 1e6))
                if sp is not None:
                    u0 = list(sp.u0)
                    last_u0 = u0
                    if prev_u0 is not None:
                        step = max(abs(u0[i] - prev_u0[i]) for i in range(6))
                        max_step = max(max_step, step)
                    prev_u0 = u0

            if now - window_start >= 1.0:
                dt = now - window_start
                rate = count / dt if dt > 0 else 0.0
                u0_mean = (sum(last_u0) / 6.0) if last_u0 else float('nan')
                print(f"{now - t_start:8.1f} {rate:8.1f} {u0_mean:9.4f} "
                      f"{max_step:9.4f} {pump.frames_rejected:9d}  "
                      + (' '.join(f'{v:.3f}' for v in last_u0) if last_u0 else '-'))
                csv.write(f"{now - t_start:.2f},{rate:.1f},{u0_mean:.5f},"
                          f"{max_step:.5f},{pump.frames_rejected}\n")
                csv.flush()
                window_start = now
                count = 0
                max_step = 0.0

            if args.duration and (now - t_start) >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        csv.close()
        sub.close()
        ctx.term()
    print(f"[traj_stream_probe] done — pump rejects total: {pump.frames_rejected}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
