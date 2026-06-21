#!/usr/bin/env python3
"""Read the CAN3 ODrive telemetry the can-bridge decodes, over the UDP link.

A reusable bench-bringup observer for the teensy-can-offload work
(plans/active/teensy-can-offload.md). RX-ONLY by construction: it sends J->T
heartbeats with flags=0 (mpc_active=0 -> the Teensy will NOT enable any leg
output) and answers TIME_OF_DAY so the bridge clears LINK_LOST and time-syncs.
There is NO setpoint stream here, so it can never command leg motion.

Prints all 7 Jugglebot axes (legs 0-5, hand=6) pos/vel + per-axis diagnostics
(state, active_errors, bus_voltage, FET temp). DIAGNOSTIC axes 7/8 are the Ball
Butler ODrives on CAN1. Used to confirm: link UP, time-sync anchoring, all CAN3
ODrives decoding, and (during Phase 9a encoder search) leg pos going NaN->finite.

Cold-bench signature (CAN+main power, idle): legs at IDLE (axis_state=1),
active_errors=0; with no main bus power, active_errors=0x201
(DC_BUS_UNDER_VOLTAGE | INITIALIZING) and bus_voltage~=0 — benign/unpowered.

Usage:
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/teensy_can3_telem.py [seconds]   # default 15
"""
import sys
import time
import threading

sys.path.insert(0, "/home/jetson/Desktop/Jugglebot")
from controller.teensy_link import (  # noqa: E402
    TeensyLinkClient, RpcServer, TimeOfDayServer, MsgType,
    HeartbeatT2J, Telemetry, Diagnostic, LinkState, FaultState,
)
from controller.teensy_link import protocol as p  # noqa: E402

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
TEENSY_IP = "192.168.42.2"
AX = ["leg0", "leg1", "leg2", "leg3", "leg4", "leg5", "hand"]

lock = threading.Lock()
state = {"telem": None, "telem_n": 0, "hb": None, "hb_n": 0, "diag": {}}


def _enum(cls, v):
    try:
        return cls(v).name
    except Exception:
        return f"{cls.__name__}_{v}"


def on_telem(mt, seq, payload, addr):
    tm = Telemetry.unpack(payload)
    with lock:
        state["telem"] = tm
        state["telem_n"] += 1


def on_hb(mt, seq, payload, addr):
    hb = HeartbeatT2J.unpack(payload)
    with lock:
        state["hb"] = hb
        state["hb_n"] += 1


def on_diag(mt, seq, payload, addr):
    d = Diagnostic.unpack(payload)
    with lock:
        prev = state["diag"].get(d.axis_id)
        state["diag"][d.axis_id] = (d, (prev[1] + 1) if prev else 1)


client = TeensyLinkClient(teensy_addr=(TEENSY_IP, p.PORT_STREAM), bind_host="0.0.0.0")
client.start()
rpc = RpcServer(client)
tod = TimeOfDayServer(rpc)
client.subscribe(int(MsgType.HEARTBEAT_T2J), on_hb)
client.subscribe(int(MsgType.TELEMETRY), on_telem)
client.subscribe(int(MsgType.DIAGNOSTIC), on_diag)
client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)  # mpc_active=0 -> no leg output

print(f"probe: peer={TEENSY_IP} stream={p.PORT_STREAM} rpc={p.PORT_RPC}  duration={DURATION:.0f}s")
t_end = time.time() + DURATION
try:
    while time.time() < t_end:
        time.sleep(1.0)
        with lock:
            hb = state["hb"]
            tm = state["telem"]
            diag = dict(state["diag"])
            tn = state["telem_n"]; state["telem_n"] = 0
            hn = state["hb_n"]; state["hb_n"] = 0
        print("=" * 74)
        if hb:
            print(f"[T2J] link={_enum(LinkState, hb.link_state)} "
                  f"fault={_enum(FaultState, hb.fault_state)} "
                  f"bus1={hb.bus1_health} bus2={hb.bus2_health} "
                  f"up={hb.uptime_ms}ms  rate={hn}Hz")
        else:
            print(f"[T2J] (none yet)  rate={hn}")
        if tm:
            print(f"[TELEM rate={tn}Hz]")
            for i, n in enumerate(AX):
                print(f"   {n:5s} pos={tm.pos_rev[i]:+10.4f} rev   vel={tm.vel_rps[i]:+8.3f} rps")
        else:
            print(f"[TELEM] (none yet)  rate={tn}")
        if diag:
            print(f"[DIAG] axes seen {sorted(diag)} ({len(diag)}/7)")
            for ax in sorted(diag):
                d, c = diag[ax]
                print(f"   axis{ax} state={d.axis_state} "
                      f"errors=0x{d.active_errors:08x} busV={d.bus_voltage:6.2f} "
                      f"Tfet={d.temp_fet:5.1f} (n={c})")
        else:
            print("[DIAG] (none yet)")
finally:
    tod.close()
    rpc.close()
    client.stop()
    with lock:
        diag = dict(state["diag"])
    print("\n--- final ---")
    print(f"tx={client.stats.tx_frames} rx={client.stats.rx_frames} "
          f"crc_err={client.stats.crc_errors} decode_err={client.stats.decode_errors} "
          f"tod_calls={tod.call_count}")
    print(f"diag axes seen: {sorted(diag)} ({len(diag)}/7)")
