#!/usr/bin/env python3
"""Minimum-viable Jetson-side daemon for the can-bridge Teensy link.

This is the MVP: a runnable single-process Jetson that gives the
Teensy what it needs to clear its `LINK_LOST` fault and lock its wall-clock:

    * Sends J→T heartbeats at 10 Hz (so the Teensy's link watchdog clears).
    * Responds to ``RpcMethod.TIME_OF_DAY_QUERY`` with CLOCK_REALTIME µs
      (so the Teensy's time-sync master can anchor its broadcast clock —
      see ADR-0008).
    * Logs T→J telemetry / diagnostic / heartbeat / profile frames at a
      configurable cadence.

No setpoint stream (that's the heavier half of this work — pulls from MPC), no
ROS 2 plumbing (that's the bridge node rewrite). The point of this MVP is
to verify the protocol layer works end-to-end on real hardware before
investing in the rest.

Usage::

    python tools/teensy_link_bridge.py
    python tools/teensy_link_bridge.py --teensy-ip 192.168.42.2
    python tools/teensy_link_bridge.py --duration 30 --verbose

Exits cleanly on Ctrl-C.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path
from typing import Tuple

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
sys.path.insert(0, str(_REPO))

from teensy_link import (  # noqa: E402
    TeensyLinkClient,
    RpcServer,
    TimeOfDayServer,
    MsgType,
    HeartbeatT2J,
    Telemetry,
    Profile,
    LinkState,
    FaultState,
)
from teensy_link import protocol as p  # noqa: E402


def _setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s.%(msecs)03d %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def _on_heartbeat_t2j(msg_type: int, seq: int, payload: bytes, addr: Tuple[str, int]) -> None:
    hb = HeartbeatT2J.unpack(payload)
    try:
        link_name = LinkState(hb.link_state).name
    except ValueError:
        link_name = f"state_{hb.link_state}"
    try:
        fault_name = FaultState(hb.fault_state).name
    except ValueError:
        fault_name = f"fault_{hb.fault_state}"
    # Python's %-formatter does not support %b for binary (that's a printf-ism);
    # bin() returns the '0b...' prefix already, so pass it as %s.
    logging.info(
        "[T2J] uptime=%dms  link=%s  fault=%s  bus1=%d  bus2=%d  flags=%s",
        hb.uptime_ms, link_name, fault_name,
        hb.bus1_health, hb.bus2_health, bin(hb.flags),
    )


def _on_profile(msg_type: int, seq: int, payload: bytes, addr: Tuple[str, int]) -> None:
    prof = Profile.unpack(payload)
    logging.info(
        "[PROFILE] heap=%dB  interp_misses=%d  rtt=%dus  can1_util=%.1f%%  can2_util=%.1f%%",
        prof.free_heap_bytes,
        prof.interp_deadline_misses,
        prof.udp_rtt_us,
        prof.can1_util_x100 / 100.0,
        prof.can2_util_x100 / 100.0,
    )


class _TelemetryRateLogger:
    """Throttled telemetry summary — telemetry arrives at 100 Hz, log at ~1 Hz."""

    def __init__(self, period_s: float = 1.0) -> None:
        self._period = period_s
        self._next_log = 0.0
        self._count_since = 0

    def __call__(self, msg_type: int, seq: int, payload: bytes, addr: Tuple[str, int]) -> None:
        self._count_since += 1
        now = time.time()
        if now < self._next_log:
            return
        tm = Telemetry.unpack(payload)
        logging.info(
            "[TELEM] rate=%dHz  axis0 pos=%.4f rev  vel=%.3f rps  (showing 1-of-100 throttled)",
            self._count_since,
            tm.pos_rev[0],
            tm.vel_rps[0],
        )
        self._count_since = 0
        self._next_log = now + self._period


def run(teensy_ip: str, duration: float, verbose: bool, bind_host: str = "0.0.0.0") -> int:
    _setup_logging(verbose)
    logging.info("teensy_link_bridge MVP — peer=%s ports stream=%d rpc=%d",
                 teensy_ip, p.PORT_STREAM, p.PORT_RPC)

    client = TeensyLinkClient(
        teensy_addr=(teensy_ip, p.PORT_STREAM),
        bind_host=bind_host,
    )
    client.start()

    rpc_server = RpcServer(client)
    tod = TimeOfDayServer(rpc_server)

    # Subscribe to interesting uplink frames. Telemetry is throttled; the rest are 1-10 Hz.
    client.subscribe(int(MsgType.HEARTBEAT_T2J), _on_heartbeat_t2j)
    client.subscribe(int(MsgType.PROFILE), _on_profile)
    client.subscribe(int(MsgType.TELEMETRY), _TelemetryRateLogger(period_s=1.0))

    # Start sending heartbeats. The HeartbeatJ2T.flags = 0 means mpc_active=0
    # — i.e. the Teensy will know the Jetson is alive but will NOT enable
    # leg-output (the setpoint stream isn't implemented in this MVP).
    client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)

    # Periodic link-status summary
    def _summary_loop_until(stop_at: float) -> int:
        while time.time() < stop_at:
            time.sleep(2.0)
            since_t2j = client.time_since_last_t2j_heartbeat_us()
            t2j_s = "never" if since_t2j is None else f"{since_t2j / 1e6:.1f}s ago"
            logging.info(
                "[summary] tx=%d  rx=%d  crc_err=%d  decode_err=%d  tod_calls=%d  last_t2j=%s",
                client.stats.tx_frames, client.stats.rx_frames,
                client.stats.crc_errors, client.stats.decode_errors,
                tod.call_count, t2j_s,
            )
        return 0

    stop_at = (time.time() + duration) if duration > 0 else float("inf")
    exit_code = 0
    try:
        exit_code = _summary_loop_until(stop_at)
    except KeyboardInterrupt:
        logging.info("interrupted — shutting down")
    finally:
        tod.close()
        rpc_server.close()
        client.stop()
        logging.info(
            "final: tx=%d rx=%d crc_err=%d tod_calls=%d",
            client.stats.tx_frames, client.stats.rx_frames,
            client.stats.crc_errors, tod.call_count,
        )
    return exit_code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--teensy-ip", default=p.TEENSY_IP,
                        help="Teensy IP (default %(default)s)")
    parser.add_argument("--bind-host", default="0.0.0.0",
                        help="Local interface to bind on (default %(default)s)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Seconds to run (default 0 = until Ctrl-C)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="DEBUG-level logging")
    args = parser.parse_args()

    # Graceful Ctrl-C without a stack trace
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))

    return run(args.teensy_ip, args.duration, args.verbose, args.bind_host)


if __name__ == "__main__":
    sys.exit(main())
