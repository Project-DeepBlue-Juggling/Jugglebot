#!/usr/bin/env python3
"""Stub Jetson-side client for the can-bridge Teensy — bench exerciser.

Stands in for the (eventual) can_node UDP bridge so the firmware data path can be
exercised end-to-end without the real MPC:

  * sends a synthetic SETPOINT stream at 40 Hz (a slow sinusoid in motor-rev
    space across all 6 legs, with u1/u2 lookahead and v0/accel),
  * sends HEARTBEAT_J2T at 10 Hz (mpc_active set, so the firmware enables output),
  * answers the time-of-day RPC (RpcMethod.TIME_OF_DAY_QUERY) with CLOCK_REALTIME
    µs — the time-sync master's wall-clock anchor,
  * receives + tallies/prints the uplink (TELEMETRY / DIAGNOSTIC / HEARTBEAT_T2J /
    PROFILE) so you can confirm the round-trip.

This is the "stub Python client: send a fake setpoint stream, log what
comes back" bench step. Run ONE of {setpoint_stub.py, profile_monitor.py} at a time — both
bind the STREAM port.

Usage:
    python setpoint_stub.py [--teensy-ip 192.168.42.2] [--duration S]
                            [--no-output]   # clear mpc_active so the firmware holds
"""

from __future__ import annotations

import argparse
import math
import os
import select
import socket
import struct
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import udp_protocol as p   # noqa: E402

NUM_LEGS = 6
MPC_DT = 0.025             # 40 Hz
HB_DT = 0.1               # 10 Hz


def _setpoint_at(t, mid=2.0, amp=0.5, freq=0.2):
    """Motor-rev position for all 6 legs at time t (sinusoid, phase-staggered)."""
    w = 2.0 * math.pi * freq
    pos = [mid + amp * math.sin(w * t + i * 0.3) for i in range(NUM_LEGS)]
    vel = [amp * w * math.cos(w * t + i * 0.3) for i in range(NUM_LEGS)]
    acc = [-amp * w * w * math.sin(w * t + i * 0.3) for i in range(NUM_LEGS)]
    return pos, vel, acc


def run(teensy_ip, duration, send_output):
    stream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stream.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    stream.bind(("0.0.0.0", p.PORT_STREAM))
    stream.setblocking(False)

    rpc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rpc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rpc.bind(("0.0.0.0", p.PORT_RPC))
    rpc.setblocking(False)

    print(f"[stub] → Teensy {teensy_ip}  ports stream={p.PORT_STREAM} rpc={p.PORT_RPC}  "
          f"mpc_active={send_output}")

    seq_sp = seq_hb = seq_rpc = 0
    t0 = time.time()
    next_sp = next_hb = t0
    counts = {}
    try:
        while True:
            now = time.time()
            if duration and (now - t0) >= duration:
                break

            # 40 Hz setpoint.
            if now >= next_sp:
                next_sp += MPC_DT
                tt = now - t0
                u0, v0, acc = _setpoint_at(tt)
                u1, _, _ = _setpoint_at(tt + MPC_DT)
                u2, _, _ = _setpoint_at(tt + 2 * MPC_DT)
                # v6 frame: 7 lanes — hand lane (index 6) packed 0.0 with
                # HAS_HAND/HAS_V1 clear, so the firmware ignores it.
                pad = (0.0,) * (p.NUM_AXES - NUM_LEGS)
                sp = p.Setpoint(
                    u0=tuple(u0) + pad, u1=tuple(u1) + pad, u2=tuple(u2) + pad,
                    v0=tuple(v0) + pad, accel=tuple(acc) + pad,
                    torque_ff=(0.0,) * p.NUM_AXES, v1=(0.0,) * p.NUM_AXES,
                    flags=0b11, t_origin_us=int(now * 1e6))
                stream.sendto(p.encode_frame(p.MsgType.SETPOINT, seq_sp & 0xFFFF, sp.pack()),
                              (teensy_ip, p.PORT_STREAM))
                seq_sp += 1

            # 10 Hz heartbeat.
            if now >= next_hb:
                next_hb += HB_DT
                hb = p.HeartbeatJ2T(t_jetson_us=int(now * 1e6),
                                    flags=(0x1 if send_output else 0x0))  # bit0 mpc_active
                stream.sendto(p.encode_frame(p.MsgType.HEARTBEAT_J2T, seq_hb & 0xFFFF, hb.pack()),
                              (teensy_ip, p.PORT_STREAM))
                seq_hb += 1

            # Drain RX on both sockets.
            r, _, _ = select.select([stream, rpc], [], [], 0.005)
            for s in r:
                try:
                    data, addr = s.recvfrom(2048)
                except (BlockingIOError, socket.timeout):
                    continue
                try:
                    mt, seq, payload = p.decode_frame(data)
                except ValueError:
                    counts["BAD_CRC"] = counts.get("BAD_CRC", 0) + 1
                    continue
                counts[mt] = counts.get(mt, 0) + 1
                if mt == p.MsgType.RPC_REQUEST and s is rpc:
                    _handle_rpc(rpc, payload, addr, now)
                elif mt == p.MsgType.HEARTBEAT_T2J:
                    h = p.HeartbeatT2J.unpack(payload)
                    if counts[mt] % 10 == 1:
                        print(f"[T2J] link={h.link_state} bus2={h.bus2_health} "
                              f"fault={h.fault_state} flags=0b{h.flags:b}")
    except KeyboardInterrupt:
        print("\n[stub] stopped")
    finally:
        stream.close(); rpc.close()
    print("[stub] frame tally: "
          + ", ".join(f"{_name(k)}={v}" for k, v in sorted(counts.items(), key=str)))


def _handle_rpc(rpc_sock, payload, addr, now):
    # method u16, req_id u16, arg_len u16, pad u16
    if len(payload) < p.RPC_REQUEST_SIZE:
        return
    method, req_id, arg_len, _pad = struct.unpack_from("<HHHH", payload, 0)
    if method == p.RpcMethod.TIME_OF_DAY_QUERY:
        result = struct.pack("<Q", int(now * 1e6))   # ResultTimeOfDay.jetson_wall_us
        head = struct.pack("<HHHH", method, req_id, p.RpcStatus.OK, len(result))
        rpc_sock.sendto(p.encode_frame(p.MsgType.RPC_RESPONSE, 0, head + result), addr)
    else:
        head = struct.pack("<HHHH", method, req_id, p.RpcStatus.ERR_UNKNOWN_METHOD, 0)
        rpc_sock.sendto(p.encode_frame(p.MsgType.RPC_RESPONSE, 0, head), addr)


def _name(mt):
    if mt == "BAD_CRC":
        return "BAD_CRC"
    try:
        return p.MsgType(mt).name
    except ValueError:
        return f"0x{mt:02X}"


def main():
    ap = argparse.ArgumentParser(description="Stub Jetson client for the can-bridge Teensy")
    ap.add_argument("--teensy-ip", default="192.168.42.2")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds (0 = until Ctrl-C)")
    ap.add_argument("--no-output", action="store_true",
                    help="clear mpc_active so the firmware does NOT enable CAN output")
    args = ap.parse_args()
    run(args.teensy_ip, args.duration, not args.no_output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
