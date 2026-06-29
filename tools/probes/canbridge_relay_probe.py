#!/usr/bin/env python3
"""Can-bridge Platform-Teensy relay bench probe (Phase 1 gate validator).

READ-ONLY: issues only STATE_READ / TILT_READ relay RPCs — it NEVER writes
RobotState and NEVER commands a motor (honours the never-command-motors rule in
tools/probes/README.md). Safe to run with the robot at rest.

Validates the two bench gates that block Phase 2 from trusting the relay reply
discriminator on hardware (plan: canbridge-foundation-coldstart-parity Testing
plan probe table; result recorded in
logbook/2026-06-29-canbridge-phase1-platform-relay-seam.md). First run
2026-06-29: SRX_DIS PASS (0/24 self-echo), reply latency <=14 ms (the 0.5 s
timeout placeholder confirmed). Re-run after any relay / FlexCAN3 firmware change.

Run:  python tools/probes/canbridge_relay_probe.py
(needs the can-bridge powered + the Jetson teensy-link iface up; the ROS bridge
node must NOT be running, as it binds the same UDP ports.)

The two gates:

  PROBE 1 (SRX_DIS): the bridge TX a dlc-1 read REQUEST on 0x6E0/0x7DE; the
    Platform replies dlc-8. If CAN3 self-reception is ENABLED, the bridge also
    self-receives its own dlc-1 request and routes it into the reply ring -> we
    see an extra near-instant dlc-1 PLATFORM_FRAME per read. If DISABLED, only the
    dlc-8 reply. (The real hazard is a dlc-8 STATE_WRITE self-echo poisoning the
    (id,dlc) latch; the dlc-1 request echo is the read-only canary for the same
    SRX setting.)
  PROBE 2 (latency): time from the RPC trigger to the genuine dlc-8 reply -> sets
    _RELAY_READ_TIMEOUT_S.

Uses the EXACT production transport (TeensyLinkClient + RpcClient + PlatformFrame
correlation) so the measurement reflects the real path.
"""
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path("/home/jetson/Desktop/Jugglebot")))
from controller.teensy_link import protocol as p
from controller.teensy_link.client import TeensyLinkClient
from controller.teensy_link.rpc import RpcClient, RpcError, RpcTimeout

ID_STATE = 0x6E0
ID_TILT = 0x7DE

frames = []           # (mono, can_id, dlc, bytes)
flock = threading.Lock()


def on_pf(msg_type, seq, payload, addr):
    try:
        pf = p.PlatformFrame.unpack(payload)
    except Exception:
        return
    with flock:
        frames.append((time.monotonic(), int(pf.can_id), int(pf.dlc), bytes(pf.data[: pf.dlc])))


def snap_clear():
    with flock:
        out = list(frames)
        frames.clear()
        return out


def main():
    link = TeensyLinkClient()             # default peer 192.168.42.2:5005/5006
    link.start()
    link.subscribe(int(p.MsgType.PLATFORM_FRAME), on_pf)
    rpc = RpcClient(link, default_timeout=0.5, default_retries=1)
    time.sleep(0.3)

    print("=== PHASE A — baseline (2 s, NO RPCs): unsolicited PLATFORM_FRAMEs? ===")
    snap_clear()
    time.sleep(2.0)
    base = snap_clear()
    bycid = {}
    for _, cid, dlc, _ in base:
        bycid[(cid, dlc)] = bycid.get((cid, dlc), 0) + 1
    print(f"  baseline PLATFORM_FRAMEs in 2 s: {len(base)}  by (can_id,dlc): "
          f"{{ {', '.join(f'(0x{c:X},dlc{d})={n}' for (c, d), n in bycid.items())} }}")

    def probe(method, can_id, label, n=12):
        lat_ms, selfecho, ackfail, noreply = [], 0, 0, 0
        for i in range(n):
            snap_clear()
            t0 = time.monotonic()
            try:
                rpc.call(method, timeout=0.5, retries=1)
            except (RpcError, RpcTimeout) as e:
                ackfail += 1
                print(f"  {label} #{i:2d}: RPC ack FAILED: {e}")
                continue
            time.sleep(0.20)                       # collect the async reply
            got = [f for f in snap_clear() if f[1] == can_id]
            dlc8 = [f for f in got if f[2] == 8]
            other = [f for f in got if f[2] != 8]
            if other:
                selfecho += 1
            if dlc8:
                lat_ms.append((dlc8[0][0] - t0) * 1000.0)
            else:
                noreply += 1
            tag = "".join(f" +dlc{f[2]}@{(f[0]-t0)*1000:.2f}ms" for f in other)
            rep = f"dlc8@{(dlc8[0][0]-t0)*1000:.2f}ms" if dlc8 else "NO-dlc8-REPLY"
            print(f"  {label} #{i:2d}: {rep}{tag}")
            time.sleep(0.05)
        return lat_ms, selfecho, ackfail, noreply

    print("\n=== PHASE B — STATE_READ x12 (0x6E0) ===")
    s_lat, s_echo, s_ackf, s_nore = probe(int(p.RpcMethod.STATE_READ), ID_STATE, "STATE_READ")
    print("\n=== PHASE C — TILT_READ x12 (0x7DE) ===")
    t_lat, t_echo, t_ackf, t_nore = probe(int(p.RpcMethod.TILT_READ), ID_TILT, "TILT_READ")

    def summ(name, lat, echo, ackf, nore):
        print(f"\n  [{name}] ack-fails={ackf}  no-reply={nore}  self-echo(non-dlc8 frames)={echo}/12")
        if lat:
            print(f"  [{name}] reply latency ms: min={min(lat):.2f} mean={statistics.mean(lat):.2f} "
                  f"max={max(lat):.2f} p95={sorted(lat)[int(0.95*len(lat))-1]:.2f}  (n={len(lat)})")
        else:
            print(f"  [{name}] reply latency: NO dlc-8 replies captured")

    print("\n=== SUMMARY ===")
    summ("STATE_READ", s_lat, s_echo, s_ackf, s_nore)
    summ("TILT_READ", t_lat, t_echo, t_ackf, t_nore)
    all_lat = s_lat + t_lat
    total_echo = s_echo + t_echo
    print("\n  VERDICT:")
    if total_echo == 0:
        print("  PROBE 1 (SRX_DIS): PASS — no self-echo of the dlc-1 request observed; CAN3")
        print("    self-reception appears disabled, so the (id,dlc=8) reply discriminator is sound.")
    else:
        print(f"  PROBE 1 (SRX_DIS): FAIL — {total_echo} reads showed a self-echoed request frame;")
        print("    CAN3 self-reception is ENABLED -> set SRX_DIS in can_buses_init() (FlexCAN3)")
        print("    before Phase 2 trusts the relay reads (a dlc-8 STATE_WRITE would also self-echo).")
    if all_lat:
        worst = max(all_lat)
        print(f"  PROBE 2 (latency): genuine reply worst-case {worst:.2f} ms -> recommend")
        print(f"    _RELAY_READ_TIMEOUT_S >= {max(0.05, (worst/1000.0)*3):.3f} s (3x worst + headroom; "
              f"current placeholder 0.5 s).")
    else:
        print("  PROBE 2 (latency): INCONCLUSIVE — no genuine replies (is the Platform Teensy on CAN3?).")
    rpc.close()
    link.stop()


if __name__ == "__main__":
    main()
