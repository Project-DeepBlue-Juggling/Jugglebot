#!/usr/bin/env python3
"""Bench driver: encoder index search on a single can-bridge leg axis (Phase 9a).

Hardware bench driver for the FIRST observable run of encoder index search over
the can-bridge. It drives the TESTED
controller/teensy_link/encoder_search.EncoderSearch state machine against real
hardware over the UDP link, using the SET_AXIS_STATE primitive — the same
orchestration teensy_bridge_node._run_encoder_search uses; this standalone driver
just gives inline observation. The PRODUCTION path is the
``encoder_search`` ROS service (scoped by the ``encoder_search_axes``
parameter); use this driver for bench bring-up of one leg.

Lives in tests/hardware/ (not tools/probes/) because it COMMANDS THE MOTOR — per
tools/probes/README.md, motor-commanding harnesses are tests/hardware/ territory.
The ``*_bench.py`` name keeps it out of pytest collection (it needs a real robot).

SAFETY: the ONLY commands it issues are SET_AXIS_STATE(axis,
ENCODER_INDEX_SEARCH) and (on retry) CLEAR_ERRORS(axis) — both bounded. There is
NO setpoint/position command path here, so it cannot cause runaway motion; the
J->T heartbeat is flags=0 (mpc_active=0), so the firmware never enables the leg
output path either.

MOTION WARNING: ENCODER_INDEX_SEARCH spins the motor to find the index pulse
(bounded, < ~1 motor rev). Keep the e-stop within reach.

First hardware run (standalone leg, odrv0): 2026-06-21 — SET_AXIS_STATE → state
1->6->1 → success; leg moved ~0.035 motor-rev (the index-search spin), no errors.

Usage:
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tests/hardware/teensy_encoder_search_bench.py [axis]   # default 0
"""
import sys
import time
import threading
import math

sys.path.insert(0, "/home/jetson/Desktop/Jugglebot")
from controller.teensy_link import (  # noqa: E402
    TeensyLinkClient, RpcClient, RpcServer, TimeOfDayServer, MsgType,
    Telemetry, Diagnostic, RpcMethod,
)
from controller.teensy_link import protocol as p  # noqa: E402
from controller.teensy_link import rpc_args  # noqa: E402
from controller.teensy_link.encoder_search import (  # noqa: E402
    EncoderSearch, AxisStatus, AXIS_STATE_ENCODER_INDEX_SEARCH,
)

AXIS = int(sys.argv[1]) if len(sys.argv) > 1 else 0
TEENSY_IP = "192.168.42.2"

_lock = threading.Lock()
_cache = {"telem": None, "diag": {}}


def on_telem(mt, seq, payload, addr):
    tm = Telemetry.unpack(payload)
    with _lock:
        _cache["telem"] = tm


def on_diag(mt, seq, payload, addr):
    d = Diagnostic.unpack(payload)
    with _lock:
        _cache["diag"][int(d.axis_id)] = d


def axis_status(axes):
    out = {}
    with _lock:
        telem = _cache["telem"]
        diag = dict(_cache["diag"])
    if telem is None:
        return out
    for a in axes:
        d = diag.get(a)
        if d is None:
            continue
        out[a] = AxisStatus(
            axis_state=int(d.axis_state),
            pos_finite=math.isfinite(float(telem.pos_rev[a])),
            active_errors=int(d.active_errors))
    return out


def axis_pos(a):
    with _lock:
        telem = _cache["telem"]
    return None if telem is None else float(telem.pos_rev[a])


def rpc_set_state(rpc, axis, state):
    try:
        rpc.call(int(RpcMethod.SET_AXIS_STATE),
                 rpc_args.encode_set_axis_state(axis, state))
        return "OK"
    except Exception as e:  # noqa: BLE001
        return f"ERR {e}"


def rpc_clear(rpc, axis):
    try:
        rpc.call(int(RpcMethod.CLEAR_ERRORS), rpc_args.encode_clear_errors(axis))
        return "OK"
    except Exception as e:  # noqa: BLE001
        return f"ERR {e}"


client = TeensyLinkClient(teensy_addr=(TEENSY_IP, p.PORT_STREAM), bind_host="0.0.0.0")
client.start()
rpc_server = RpcServer(client)
tod = TimeOfDayServer(rpc_server)
rpc = RpcClient(client)
client.subscribe(int(MsgType.TELEMETRY), on_telem)
client.subscribe(int(MsgType.DIAGNOSTIC), on_diag)
client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)  # mpc_active=0

print(f"encoder-search bench: axis={AXIS} peer={TEENSY_IP}")
t0 = time.time()
while axis_status([AXIS]).get(AXIS) is None and time.time() - t0 < 3.0:
    time.sleep(0.05)
base = axis_status([AXIS]).get(AXIS)
print(f"baseline: axis{AXIS} status={base} pos={axis_pos(AXIS)}")

es = EncoderSearch([AXIS], timeout_s=6.0)
print(f">>> commanding ENCODER_INDEX_SEARCH on axis {AXIS} (the motor will spin) ...")
last_state = None
try:
    while not es.done:
        now = time.time()
        res = es.step(now, axis_status([AXIS]))
        for a in res.clear_errors:
            print(f"  CLEAR_ERRORS({a}): {rpc_clear(rpc, a)}")
        for a in res.set_search:
            print(f"  SET_AXIS_STATE({a}, ENCODER_INDEX_SEARCH): "
                  f"{rpc_set_state(rpc, a, AXIS_STATE_ENCODER_INDEX_SEARCH)}")
        st = axis_status([AXIS]).get(AXIS)
        if st is not None and st.axis_state != last_state:
            print(f"  axis{AXIS} state -> {st.axis_state}  pos={axis_pos(AXIS)}")
            last_state = st.axis_state
        if es.done:
            break
        time.sleep(0.05)
finally:
    print("--- result ---")
    print(f"succeeded={es.succeeded}  failed={es.failed}")
    print(f"final_pos={axis_pos(AXIS)}")
    tod.close()
    rpc_server.close()
    client.stop()
