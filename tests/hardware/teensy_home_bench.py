#!/usr/bin/env python3
"""Bench driver: firmware homing on a single can-bridge leg axis (Phase 9b).

Hardware bench driver for the FIRST observable run of the firmware
move-to-hardstop over the can-bridge. Unlike encoder search (Phase 9a, a
Jetson-side orchestration over SET_AXIS_STATE), the homing *move* runs
autonomously in the can-bridge HOME handler — there is no per-leg motion RPC, so
the velocity-limited move-to-hardstop lives in firmware. This driver fires the
HOME RPC (fire-and-monitor) and drives the TESTED
controller/teensy_link/homing.HomingMonitor to observe completion from telemetry,
exactly as teensy_bridge_node._run_home does; this standalone driver just gives
inline observation. The PRODUCTION path is the ``/teensy/home`` ROS service
(scoped by the ``home_axes`` parameter); use this driver for bench bring-up of one
leg.

Lives in tests/hardware/ (not tools/probes/) because it COMMANDS THE MOTOR — per
tools/probes/README.md, motor-commanding harnesses are tests/hardware/ territory.
The ``*_bench.py`` name keeps it out of pytest collection (it needs a real robot).

⚠️  MOTION WARNING — LARGER than encoder search. Homing drives the leg through
its travel into the retracted hardstop in VELOCITY/VEL_RAMP at
HOMING_LEG_SPEED_RPS (1.5 rev/s). The firmware bounds it: the descent is
velocity-limited, the ODrive current is capped at limit+headroom (8 A), the
hardstop is detected by the Iq EMA crossing 5 A, and a 30 s hard timeout aborts
to IDLE if no stop is found — but KEEP THE E-STOP WITHIN REACH and confirm the
standalone leg (odrv0) is the target before running.

PRECONDITION: encoder index search (Phase 9a) must have completed on the axis
(finite encoder estimate), and the leg ODrive must have usable velocity gains
(flash defaults or a prior SET_VEL_GAINS) — this homing sets only state/mode/
limits, like can_node._home_motor_steps.

What success looks like: axis_state cycles IDLE -> CLOSED_LOOP -> IDLE, the leg
descends to the hardstop, and the encoder snaps to the home reference
(set_absolute_position(+0.1) reads back -0.1 rev in Jugglebot-convention
telemetry). On any abort the leg is left IDLE — never driving.

Usage:
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tests/hardware/teensy_home_bench.py [axis]   # default 0
"""
import sys
import time
import threading

sys.path.insert(0, "/home/jetson/Desktop/Jugglebot")
from controller.teensy_link import (  # noqa: E402
    TeensyLinkClient, RpcClient, RpcServer, TimeOfDayServer, MsgType,
    Telemetry, Diagnostic, RpcMethod,
)
from controller.teensy_link import protocol as p  # noqa: E402
from controller.teensy_link import rpc_args  # noqa: E402
from controller.teensy_link.homing import (  # noqa: E402
    HomingMonitor, AxisStatus,
)
import jugglebot.hardware_config as hw  # noqa: E402

AXIS = int(sys.argv[1]) if len(sys.argv) > 1 else 0
TEENSY_IP = "192.168.42.2"
HOME_REF = abs(float(hw.HOMING_LEG_ABS_POS_REV))

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


def axis_status(axis):
    with _lock:
        telem = _cache["telem"]
        d = _cache["diag"].get(int(axis))
    if telem is None or d is None:
        return None
    return AxisStatus(
        axis_state=int(d.axis_state),
        pos_rev=float(telem.pos_rev[int(axis)]),
        active_errors=int(d.active_errors))


def axis_pos(a):
    with _lock:
        telem = _cache["telem"]
    return None if telem is None else float(telem.pos_rev[a])


def rpc_home(rpc, axis):
    try:
        rpc.call(int(RpcMethod.HOME), rpc_args.encode_home(axis))
        return "OK (accepted)"
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

print(f"home bench: axis={AXIS} peer={TEENSY_IP}  home_ref=|{HOME_REF:.3f}| rev")
t0 = time.time()
while axis_status(AXIS) is None and time.time() - t0 < 3.0:
    time.sleep(0.05)
print(f"baseline: axis{AXIS} status={axis_status(AXIS)} pos={axis_pos(AXIS)}")

print(f">>> commanding HOME on axis {AXIS} (the leg WILL drive to the hardstop) ...")
print(">>> E-STOP READY. Ctrl-C aborts the observer (the firmware also self-aborts on timeout).")

mon = HomingMonitor([AXIS], home_ref_rev=HOME_REF,
                    timeout_s=float(hw.HOMING_MOTOR_TIMEOUT_S) + 5.0)
hard_deadline = time.time() + float(hw.HOMING_MOTOR_TIMEOUT_S) + 10.0
last_state = None
try:
    while not mon.done:
        now = time.time()
        st = axis_status(AXIS)
        res = mon.step(now, {AXIS: st} if st is not None else {})
        for a in res.set_home:
            print(f"  HOME({a}): {rpc_home(rpc, a)}")
        if st is not None and st.axis_state != last_state:
            print(f"  axis{AXIS} state -> {st.axis_state}  pos={axis_pos(AXIS):+.4f} "
                  f"iq={_cache['diag'].get(AXIS).iq_measured:+.2f}A")
            last_state = st.axis_state
        if mon.done or now > hard_deadline:
            break
        time.sleep(0.05)
finally:
    print("--- result ---")
    print(f"succeeded={mon.succeeded}  failed={mon.failed}")
    print(f"final_pos={axis_pos(AXIS)}  (expect |pos| ~ {HOME_REF:.3f} on success)")
    tod.close()
    rpc_server.close()
    client.stop()
