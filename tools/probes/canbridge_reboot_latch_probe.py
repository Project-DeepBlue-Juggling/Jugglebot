#!/usr/bin/env python3
"""Can-bridge reboot-latency + clear/reboot gate bench probe (reboot-latch gate validator).

Validates the two open reboot-latch bench decisions (logbook
2026-06-30-canbridge-phase6-reboot-latch):

  PROBE 2 (window) — measure REBOOT_ODRIVES -> first-fresh-leg-heartbeat latency.
    The firmware watchdog trips CAN_BUS_DOWN ~CAN_HEARTBEAT_TIMEOUT (2 s) after the
    legs go silent and clears it the instant all present legs heartbeat fresh again
    (all_present_legs_fresh()). So (CAN_BUS_DOWN clear time - t_reboot) IS the
    reboot-to-first-heartbeat latency the REBOOT_WATCHDOG_SUPPRESS_US window must
    cover. Read over UDP from HeartbeatT2J.fault_state (no serial console needed).

  PROBE 1 (gate basis) — does the staleness gate (the 2026-06-27 deadlock) actually
    refuse a recovery clear during the reboot silence? bus1_health IS health_of(
    jugglebot) — the exact value jugglebot_commands_allowed() reads — so if it goes
    WARN during the silence, a CLEAR_ERRORS would be refused with ERR_BUS_DOWN
    (the deadlock). We send CLEAR_ERRORS probes through the window and record the
    RPC status to demonstrate the gate behaviour directly.

  Before/after design: run this against the pre-reboot-latch firmware (the "before" — CAN_BUS_DOWN
  appears on reboot = the false loss; CLEAR_ERRORS during the WARN window =
  ERR_BUS_DOWN = the deadlock) and again against the reboot-latch firmware (the "after" —
  the reboot latch suppresses CAN_BUS_DOWN within the window; the SYNCH gate lets
  CLEAR_ERRORS return OK during the same window).

RX/command scope: sends J->T heartbeats with flags=0 (mpc_active=0 -> the Teensy
NEVER enables leg output), answers TIME_OF_DAY, and issues exactly two operator
RPCs the operator authorised: REBOOT_ODRIVES and CLEAR_ERRORS. Benign with motor
power OFF (logic-only reboot; the ODrives re-home later). It NEVER streams a
setpoint, so it can never command leg motion.

Run (ROS bridge node must NOT be running — it binds the same UDP ports):
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/canbridge_reboot_latch_probe.py [observe_seconds]
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, "/home/jetson/Desktop/Jugglebot")
sys.path.insert(0, "/home/jetson/Desktop/Jugglebot/ros_ws/src/jugglebot")

from controller.teensy_link import (  # noqa: E402
    TeensyLinkClient, RpcClient, RpcServer, RpcError, RpcTimeout,
    TimeOfDayServer, MsgType, HeartbeatT2J, FaultState, BusHealth,
)
from controller.teensy_link import protocol as p  # noqa: E402
from controller.teensy_link import rpc_args  # noqa: E402

TEENSY_IP = "192.168.42.2"
OBSERVE_S = float(sys.argv[1]) if len(sys.argv) > 1 else 18.0
# CLEAR_ERRORS gate probes spanning the likely reboot window (s after REBOOT).
CLEAR_PROBE_TIMES = [2.5, 3.5, 5.0, 7.0, 9.0, 12.0]
OUT_DIR = Path("/home/jetson/Desktop/Jugglebot/temp/probes")


def _enum(cls, v):
    try:
        return cls(v).name
    except Exception:
        return f"{cls.__name__}_{v}"


def main():
    samples = []   # (t_rel, fault_state, bus1_health)
    latest = {"hb": None}

    def on_hb(mt, seq, payload, addr):
        latest["hb"] = HeartbeatT2J.unpack(payload)

    client = TeensyLinkClient(teensy_addr=(TEENSY_IP, p.PORT_STREAM), bind_host="0.0.0.0")
    client.start()
    rpc_srv = RpcServer(client)
    tod = TimeOfDayServer(rpc_srv)          # answer TOD so the bridge time-syncs / stays UP
    rpc = RpcClient(client, default_timeout=0.5, default_retries=2)
    client.subscribe(int(MsgType.HEARTBEAT_T2J), on_hb)
    client.start_heartbeat(hz=float(p.HEARTBEAT_HZ), flags=0)   # mpc_active=0 -> no leg output

    print(f"probe: peer={TEENSY_IP} observe={OBSERVE_S:.0f}s  (motor power should be OFF)")
    # Settle + baseline.
    time.sleep(2.0)
    hb = latest["hb"]
    if hb is None:
        print("ERROR: no HeartbeatT2J received — is the link up / bridge powered?")
        tod.close(); rpc.close(); client.stop(); return 2
    print(f"baseline: fault={_enum(FaultState, hb.fault_state)} "
          f"bus1(CAN3)={_enum(BusHealth, hb.bus1_health)} link={hb.link_state}")

    # Fire the reboot. AXIS_ALL covers the legs; on the pre-reboot-latch firmware AXIS_ALL drops
    # the hand (i < NUM_LEGS), and the hand's heartbeats keep CAN3 RX fresh, masking
    # the staleness gate. To test the all-ODrives-silent deadlock premise we ALSO
    # reboot the hand per-axis (axis 6 — the RPC allow-table permits REBOOT). On
    # the reboot-latch firmware AXIS_ALL already includes the hand, so the second call is a
    # harmless no-op-equivalent (the hand is already rebooting).
    HAND_AXIS = 6
    print("\n=== sending REBOOT_ODRIVES(AXIS_ALL) + REBOOT_ODRIVES(hand axis 6) ===")
    t0 = time.monotonic()
    for label, axis in (("AXIS_ALL(legs)", rpc_args.AXIS_ALL), ("hand(axis6)", HAND_AXIS)):
        try:
            rpc.call(int(p.RpcMethod.REBOOT_ODRIVES), rpc_args.encode_reboot(axis),
                     timeout=0.5, retries=2)
            print(f"  REBOOT_ODRIVES {label} -> OK")
        except RpcError as e:
            print(f"  REBOOT_ODRIVES {label} -> RpcError status={e.args[1] if len(e.args) > 1 else e}")
        except RpcTimeout as e:
            print(f"  REBOOT_ODRIVES {label} -> TIMEOUT ({e})")

    # Observe fault_state + bus1_health, and probe the CLEAR_ERRORS gate through the window.
    clear_probes = []   # (t_rel, bus1_health, clear_status)
    pending = list(CLEAR_PROBE_TIMES)
    last_print = 0.0
    while True:
        t = time.monotonic() - t0
        if t >= OBSERVE_S:
            break
        hb = latest["hb"]
        if hb is not None:
            samples.append((t, int(hb.fault_state), int(hb.bus1_health)))
            if t - last_print >= 0.5:
                print(f"  t={t:5.2f}s  fault={_enum(FaultState, hb.fault_state):14s} "
                      f"bus1={_enum(BusHealth, hb.bus1_health)}")
                last_print = t
        # Fire a CLEAR_ERRORS gate probe when its scheduled time arrives.
        if pending and t >= pending[0]:
            pending.pop(0)
            bus1 = int(hb.bus1_health) if hb else -1
            try:
                rpc.call(int(p.RpcMethod.CLEAR_ERRORS), rpc_args.encode_clear_errors(rpc_args.AXIS_ALL),
                         timeout=0.5, retries=1)
                status = "OK"
            except RpcError as e:
                status = f"ERR status={e.args[1] if len(e.args) > 1 else '?'}"
            except RpcTimeout:
                status = "TIMEOUT"
            clear_probes.append((t, bus1, status))
            print(f"  >> CLEAR_ERRORS @ t={t:5.2f}s  bus1={_enum(BusHealth, bus1)} -> {status}")
        time.sleep(0.02)

    tod.close(); rpc.close(); client.stop()

    # ── Analysis ──────────────────────────────────────────────────────────────
    CBD = int(FaultState.CAN_BUS_DOWN)
    onset = next((t for (t, f, _b) in samples if f == CBD), None)
    clear = None
    if onset is not None:
        clear = next((t for (t, f, _b) in samples if t > onset and f != CBD), None)
    warn_samples = [t for (t, _f, b) in samples if b == int(BusHealth.WARN)]

    print("\n=== ANALYSIS ===")
    if onset is None:
        print("  CAN_BUS_DOWN never observed within the window.")
        print("  -> reboot-latch firmware: the reboot latch SUPPRESSED the false loss (expected 'after').")
        print("  -> pre-reboot-latch firmware: the legs may have rebooted faster than the 2 s watchdog, OR")
        print("     the reboot did not fire — cross-check the bus1/fault trace above.")
    else:
        print(f"  CAN_BUS_DOWN onset:  t={onset:.2f}s (legs went stale ~2 s after reboot)")
        if clear is not None:
            print(f"  CAN_BUS_DOWN clear:  t={clear:.2f}s")
            print(f"  >>> reboot->first-fresh-heartbeat latency ~= {clear:.2f}s "
                  f"(= the REBOOT_WATCHDOG_SUPPRESS_US window floor)")
        else:
            print("  CAN_BUS_DOWN never cleared within the window (legs did not return — extend OBSERVE_S).")
    if warn_samples:
        print(f"  bus1(CAN3) health == WARN during: t=[{min(warn_samples):.2f}..{max(warn_samples):.2f}]s "
              f"-> jugglebot_commands_allowed()=FALSE here (the staleness gate refuses clear/reboot).")
    else:
        print("  bus1(CAN3) health never WARN -> the Platform Teensy kept CAN3 RX fresh (staleness "
              "gate would NOT refuse here; see logbook Discussion).")
    print("  CLEAR_ERRORS gate probes (t, bus1, status):")
    for (t, b, st) in clear_probes:
        print(f"     t={t:5.2f}s  bus1={_enum(BusHealth, b):8s}  CLEAR -> {st}")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / "canbridge_reboot_latch_probe.txt"
    with out.open("w") as f:
        f.write(f"reboot-latch probe  observe={OBSERVE_S}s\n")
        f.write(f"CAN_BUS_DOWN onset={onset} clear={clear}\n")
        f.write("samples (t_rel, fault_state, bus1_health):\n")
        for s in samples:
            f.write(f"  {s[0]:.3f} {s[1]} {s[2]}\n")
        f.write("clear_probes (t_rel, bus1_health, status):\n")
        for cp in clear_probes:
            f.write(f"  {cp[0]:.3f} {cp[1]} {cp[2]}\n")
    print(f"\n  wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
