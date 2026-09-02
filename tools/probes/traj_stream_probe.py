#!/usr/bin/env python3
"""Live observer for the MVP trajectory generator's 40 Hz :5557 knot stream.

READ-ONLY. Opens a ZMQ SUB on ``tcp://127.0.0.1:5557`` (topic ``mpccmd``) — it
**connects**, never binds, and never sends anything on any wire — so it cannot
command a motor and coexists with the live ``trajectory_node``.
Decodes each ``make_mpc_command`` frame and prints the observed stream rate, the
per-leg ``u0`` (= ``motor_rev``), the max per-knot leg ``|Δu0|`` step, and — for
a 7-channel unified-cycle stream — the hand lane (``u0[6]``, printed only when
the frame carries ``HAS_HAND``), so an operator can confirm during an arming
bring-up that:

  * a steady ~40 Hz hold stream is flowing (before arming),
  * ``u0`` sits at the activate revs (~2.19 rev) — i.e. a bumpless hold at the
    active pose,
  * the per-knot leg step stays well under the pump's 0.3 rev gate (a hold is
    ~0); the hand lane's own gate is 5.0 rev (see setpoint_pump.py's
    DEFAULT_MAX_STEP_HAND_REV derivation).

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
from teensy_link import protocol as p  # noqa: E402
from teensy_link.setpoint_pump import FLAG_HAS_HAND, SetpointPump  # noqa: E402

_NLEGS = int(p.NUM_LEGS)   # leg lanes of the 7-wide v6 Setpoint (index 6 = hand)


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
                        max_step_rev=hw.JB_OP_MAX_POSITION_STEP_REV,
                        max_step_hand_rev=(float(hw.JB_TRAJ_HAND_VEL_LIMIT_RPS)
                                           * float(hw.JB_TRAJ_KNOT_DT_S)))

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(_out_dir(), f'traj_stream_probe_{ts}.csv')
    csv = open(csv_path, 'w')
    csv.write('t_wall,rate_hz,u0_mean_rev,max_step_rev,pump_rejects,hand_rev\n')

    print(f"[traj_stream_probe] SUB {args.addr} topic={args.topic!r} "
          f"(READ-ONLY) → {csv_path}")
    print(f"{'time':>8} {'rate_hz':>8} {'u0_mean':>9} {'max_step':>9} "
          f"{'pump_rej':>9} {'hand':>8}  u0[0..5] (rev)")

    t_start = time.perf_counter()
    window_start = t_start
    count = 0
    prev_u0 = None
    max_step = 0.0
    last_u0 = None
    last_hand = None            # (rev, HAS_HAND) of the last accepted frame
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
                    # Leg lanes only for the step statistic — the hand lane
                    # (index 6) has its own gate and its own column.
                    u0 = list(sp.u0[:_NLEGS])
                    last_u0 = u0
                    last_hand = (sp.u0[_NLEGS],
                                 bool(sp.flags & FLAG_HAS_HAND))
                    if prev_u0 is not None:
                        step = max(abs(u0[i] - prev_u0[i])
                                   for i in range(_NLEGS))
                        max_step = max(max_step, step)
                    prev_u0 = u0

            if now - window_start >= 1.0:
                dt = now - window_start
                rate = count / dt if dt > 0 else 0.0
                u0_mean = (sum(last_u0) / _NLEGS) if last_u0 else float('nan')
                hand_txt = (f'{last_hand[0]:8.3f}' if last_hand and last_hand[1]
                            else f"{'-':>8}")
                hand_csv = (f'{last_hand[0]:.5f}' if last_hand and last_hand[1]
                            else '')
                print(f"{now - t_start:8.1f} {rate:8.1f} {u0_mean:9.4f} "
                      f"{max_step:9.4f} {pump.frames_rejected:9d} {hand_txt}  "
                      + (' '.join(f'{v:.3f}' for v in last_u0) if last_u0 else '-'))
                csv.write(f"{now - t_start:.2f},{rate:.1f},{u0_mean:.5f},"
                          f"{max_step:.5f},{pump.frames_rejected},{hand_csv}\n")
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
