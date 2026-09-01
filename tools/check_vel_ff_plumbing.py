#!/usr/bin/env python3
"""Pre-flight diagnostic: confirm motor_guard's vel_ff and torque_ff IPC
fields are populated with non-zero values during operational moves.

DORMANT since 2026-08-01 — READ BEFORE USE
------------------------------------------
This tool SUBs ``motor_guard``'s telemetry on ``tcp://127.0.0.1:5556``.
``jugglebot_launch.py`` stopped starting ``motor_guard`` on 2026-08-01, and
since the MPC removal on 2026-09-01 (git tag ``mpc-final``;
``logbook/2026-09-01-mpc-chain-removed.md``) **nothing publishes :5556 in the
production launch at all** — the MPC feeder (``run_mpc.py`` → ``HardwarePlant``)
and the ``motion_bridge_node`` consumer are both deleted.  The live leg path is
``trajectory_node`` → ZMQ :5557 → ``teensy_bridge_node``, which never touches
:5556, so a GUI move or a toss/reload gate run produces **no telemetry here**
and this tool exits 1.

It functions ONLY against a hand-started ``motor_guard`` — the parked fallback —
driven by a hand-written setpoint source.  There is no supported end-to-end
recipe on the current stack; the usage block below is retained as the historical
procedure and is not runnable as written.

Subscribes (read-only) to the motor_guard telemetry stream on
``tcp://127.0.0.1:5556``, captures samples for ``--duration`` seconds, and
reports per-leg statistics on ``leg_vel`` and ``leg_torques``.  ZMQ fanout is
the primary distribution mechanism, so this SUB coexists with any other
subscriber on the same address with no interference.

Usage (HISTORICAL — the MPC chain it drives was removed 2026-09-01, tag mpc-final)
---------------------------------------------------------------------------------
    # Terminal A: start the platform stack as you normally would
    cd ros_ws && colcon build --packages-select jugglebot
    source install/setup.bash
    ros2 launch jugglebot jugglebot_launch.py
    # ... activate the platform via service or GUI ...

    # Terminal B: run a move that exercises motor_guard
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python run_mpc.py --pose 0,0,170,0,0,0 --duration 10

    # Terminal C: run this diagnostic concurrently with terminal B
    python tools/check_vel_ff_plumbing.py --duration 8

The diagnostic reports:

  * sample count and effective rate (motor_guard publishes at 500 Hz)
  * fraction of samples with non-zero ``leg_vel`` per leg
  * peak |leg_vel|, |leg_torques| per leg (bench saw ±1.0 to ±1.5 rev/s
    during cruise of a 0.5 rps trapezoid; platform expectations differ
    but should be non-zero during motion)
  * verdict: ``OK`` if at least one leg shows non-zero vel_ff with peak
    above the noise floor; ``WARN`` if all legs are flat zero (latent
    regression)

Exit codes
----------
0 — non-zero vel_ff confirmed; integration plan §6.1 "vel_ff is non-zero"
    branch applies; PR 1 collapses to just the Kt promotion.
1 — no telemetry received (motor_guard not running, or wrong address).
2 — telemetry received but vel_ff is flat zero across all legs (latent
    regression — vel_ff should be enabled before any friction-FF work
    per integration plan §6.1).
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

import msgpack
import numpy as np
import zmq


TELEMETRY_ADDR = 'tcp://127.0.0.1:5556'
TOPIC_TELEMETRY = b'telem'

# Verdict thresholds.  These are intentionally permissive — the question we
# are answering is "is vel_ff plumbed at all", not "is its value optimal".
NONZERO_VEL_FRAC_THRESHOLD = 0.10        # at least 10% of samples non-zero per leg
PEAK_VEL_THRESHOLD_RPS = 0.001           # |vel_ff| peak above this counts as live
PEAK_TORQUE_THRESHOLD_NM = 1e-5          # ditto for torque_ff (Nm)


@dataclass
class CaptureStats:
    n_samples: int = 0
    n_with_leg_vel: int = 0          # samples where 'leg_vel' key present
    n_with_leg_torques: int = 0
    duration_s: float = 0.0
    # Per-leg stats; populated only for samples with the key present
    leg_vel_nonzero_count: list = field(default_factory=lambda: [0] * 6)
    leg_vel_peak_abs: list = field(default_factory=lambda: [0.0] * 6)
    leg_torques_nonzero_count: list = field(default_factory=lambda: [0] * 6)
    leg_torques_peak_abs: list = field(default_factory=lambda: [0.0] * 6)


def _ingest_sample(stats: CaptureStats, msg: dict) -> None:
    """Update ``stats`` from one decoded telemetry message.

    Pure function — no I/O.  Tested by ``tests/sim/test_check_vel_ff_plumbing.py``.
    """
    stats.n_samples += 1
    leg_vel = msg.get('leg_vel')
    if leg_vel is not None:
        stats.n_with_leg_vel += 1
        for i in range(min(6, len(leg_vel))):
            v = float(leg_vel[i])
            if v != 0.0:
                stats.leg_vel_nonzero_count[i] += 1
            stats.leg_vel_peak_abs[i] = max(stats.leg_vel_peak_abs[i], abs(v))
    leg_torques = msg.get('leg_torques')
    if leg_torques is not None:
        stats.n_with_leg_torques += 1
        for i in range(min(6, len(leg_torques))):
            t = float(leg_torques[i])
            if t != 0.0:
                stats.leg_torques_nonzero_count[i] += 1
            stats.leg_torques_peak_abs[i] = max(stats.leg_torques_peak_abs[i], abs(t))


def _verdict(stats: CaptureStats) -> tuple[int, str]:
    """Decide exit code + human-readable verdict from ``stats``.

    Pure function — tested in ``tests/sim/test_check_vel_ff_plumbing.py``.
    """
    if stats.n_samples == 0:
        return 1, ('NO TELEMETRY received.  Is motor_guard running, and is '
                   f'the platform activated to publish on {TELEMETRY_ADDR}?')

    if stats.n_with_leg_vel == 0:
        return 2, ('Telemetry received but every sample had leg_vel = None '
                   '(motor_guard is in a fallback envelope or has not received '
                   'an MPC command).  Re-run while a move is actively in flight.')

    # Per-leg verdict — at least one leg must have BOTH non-zero peak AND
    # non-zero-fraction above threshold for an "OK".  Flat zeros across all
    # six legs means the vel_ff plumbing is latent-regressed.
    any_leg_live = False
    for i in range(6):
        peak_ok = stats.leg_vel_peak_abs[i] > PEAK_VEL_THRESHOLD_RPS
        frac = (stats.leg_vel_nonzero_count[i]
                / max(1, stats.n_with_leg_vel))
        if peak_ok and frac > NONZERO_VEL_FRAC_THRESHOLD:
            any_leg_live = True
            break

    if not any_leg_live:
        return 2, ('vel_ff is FLAT ZERO across all six legs '
                   f'(threshold: peak |vel_ff| > {PEAK_VEL_THRESHOLD_RPS} rev/s '
                   f'AND non-zero in > {NONZERO_VEL_FRAC_THRESHOLD*100:.0f} % '
                   'of samples).  Latent regression — fix this before any '
                   'friction-FF work (see integration plan §6.1).')

    return 0, ('vel_ff is LIVE and non-zero on at least one leg.  '
               'Integration plan §6.1 "vel_ff is non-zero" branch applies; '
               'PR 1 can collapse to just the Kt promotion.')


def _format_report(stats: CaptureStats) -> str:
    """Render a readable per-leg summary of the captured stats."""
    if stats.n_samples == 0:
        return '(no samples captured)'
    rate_hz = stats.n_samples / max(1e-6, stats.duration_s)
    lines: list[str] = []
    lines.append(f'  samples: {stats.n_samples} captured over '
                 f'{stats.duration_s:.2f} s  (effective rate '
                 f'{rate_hz:.1f} Hz; motor_guard publishes at 500 Hz)')
    lines.append(f'  samples carrying leg_vel:     {stats.n_with_leg_vel}/{stats.n_samples} '
                 f'({100.0 * stats.n_with_leg_vel / stats.n_samples:.1f} %)')
    lines.append(f'  samples carrying leg_torques: {stats.n_with_leg_torques}/{stats.n_samples} '
                 f'({100.0 * stats.n_with_leg_torques / stats.n_samples:.1f} %)')
    lines.append('')
    lines.append(f"  {'Leg':>4}  {'vel_ff peak [rev/s]':>20}  {'vel_ff nonzero %':>18}  "
                 f"{'torque_ff peak [Nm]':>20}  {'torque_ff nonzero %':>20}")
    for i in range(6):
        vel_peak = stats.leg_vel_peak_abs[i]
        vel_frac = (100.0 * stats.leg_vel_nonzero_count[i]
                    / max(1, stats.n_with_leg_vel))
        tor_peak = stats.leg_torques_peak_abs[i]
        tor_frac = (100.0 * stats.leg_torques_nonzero_count[i]
                    / max(1, stats.n_with_leg_torques))
        lines.append(f'  {i:>4d}  {vel_peak:>20.5f}  {vel_frac:>17.1f} %  '
                     f'{tor_peak:>20.6f}  {tor_frac:>19.1f} %')
    return '\n'.join(lines)


def _drain(sub: zmq.Socket, stats: CaptureStats) -> None:
    """Drain all pending messages from sub into stats.  Non-blocking."""
    while True:
        try:
            frames = sub.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        if len(frames) < 2:
            continue
        topic, payload = frames[0], frames[1]
        if topic != TOPIC_TELEMETRY:
            continue
        try:
            msg = msgpack.unpackb(payload, raw=False)
        except Exception:
            # malformed packet — count as a sample but don't break the run
            stats.n_samples += 1
            continue
        if isinstance(msg, dict):
            _ingest_sample(stats, msg)
        else:
            stats.n_samples += 1


def capture(duration_s: float, address: str = TELEMETRY_ADDR,
            connect_grace_s: float = 0.5) -> CaptureStats:
    """Open a SUB socket and capture telemetry for ``duration_s`` seconds.

    Always returns; aborts cleanly on SIGINT.  Caller is expected to inspect
    ``stats.n_samples`` to detect the no-telemetry case.
    """
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    # Match motion_bridge_node's idiom: subscribe to the canonical telemetry
    # topic only, with a small RCVHWM and short LINGER so we don't accumulate
    # back-pressure or block on close.
    sub.setsockopt(zmq.SUBSCRIBE, TOPIC_TELEMETRY)
    sub.setsockopt(zmq.RCVHWM, 100)
    sub.setsockopt(zmq.LINGER, 0)
    sub.connect(address)

    # Brief grace period so the first SUBSCRIBE hits the PUB before we start
    # the timed capture window.  ZMQ slow joiner is real: without this,
    # short-duration captures can miss the early messages.
    time.sleep(connect_grace_s)

    stats = CaptureStats()
    stop = [False]
    prev_handler = signal.signal(signal.SIGINT,
                                 lambda *_: stop.__setitem__(0, True))
    try:
        t0 = time.time()
        deadline = t0 + duration_s
        while not stop[0] and time.time() < deadline:
            # Block briefly with a short poll to keep CPU usage reasonable
            # while still letting us notice SIGINT promptly.
            if sub.poll(50, zmq.POLLIN):
                _drain(sub, stats)
        stats.duration_s = time.time() - t0
    finally:
        signal.signal(signal.SIGINT, prev_handler)
        sub.close()
        # Don't terminate the shared context — it can be reused by callers
        # that import this module.

    return stats


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--duration', type=float, default=8.0,
                   help='capture window in seconds (default 8)')
    p.add_argument('--address', default=TELEMETRY_ADDR,
                   help=f'ZMQ telemetry address (default {TELEMETRY_ADDR})')
    args = p.parse_args()

    print(f'Subscribing to {args.address} for {args.duration:.1f} s...')
    print('(DORMANT: nothing publishes :5556 in the production launch since the '
          '2026-09-01 MPC removal — this needs a hand-started motor_guard plus a '
          'hand-written setpoint source. A GUI or gate-run move will NOT show up here.)')
    print()

    stats = capture(args.duration, args.address)

    print(_format_report(stats))
    print()
    rc, verdict = _verdict(stats)
    label = {0: 'OK', 1: 'NO TELEMETRY', 2: 'WARN'}[rc]
    print(f'Verdict: [{label}]  {verdict}')
    return rc


if __name__ == '__main__':
    sys.exit(main())
