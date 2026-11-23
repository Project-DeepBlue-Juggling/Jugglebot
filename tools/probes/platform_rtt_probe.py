"""Platform command round-trip probe (BUG-132): is the Platform Teensy's CAN receive queue a delay line?

Measures the round trip of a Platform-answered STATE_READ (RPC ack, then the Platform's PLATFORM_FRAME 0x6E0
reply) against a bridge-only NOP, and reports the EXCESS (state round trip minus NOP round trip): the extra time a
command spends in and behind the Platform's RX path. The verdict uses the FLOOR excess (minimum minus minimum),
because the Platform's reply waits 0 to 10 ms for the bridge's relay tick on a healthy board (round trips spread
evenly over about 2 to 13 ms), while a queue delay lifts the fastest round trip as well. Measured 2026-11-23:
FW 14 with the leaking FlexCAN_T4 ring, floor excess about 37 ms (median 44); FW 15 just flashed, floor excess 1.6
to 2.3 ms (median 5.7 to 6.9). Exit code 1 when the median over runs of the floor excess is over
``--max-excess-ms`` (default 5), 2 when the link produced no usable round trips, 0 otherwise.

MUST run with the launch DOWN: the UDP link to the can-bridge is single-owner, so a running teensy_bridge_node
would fight this probe for it. Read-only RPCs only (NOP, STATE_READ): nothing is commanded, no motion, no flash.

Usage (repo root, venv active)::

    python tools/probes/platform_rtt_probe.py --runs 3 --n 100 [--max-excess-ms 5] [--out PATH]

Output: ``temp/probes/platform_rtt_<UTC stamp>.json`` (raw samples, per-run stats, the bridge identity, the last
IMU_STATUS and RING_DIAG seen). Motivation and findings: BUG-132 and its logbook entry.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from teensy_link import platform_imu as pi  # noqa: E402
from teensy_link import protocol as p  # noqa: E402
from teensy_link.client import TeensyLinkClient  # noqa: E402
from teensy_link.rpc import RpcClient  # noqa: E402


def stats(x):
    x = sorted(x)
    if not x:
        return None
    return dict(n=len(x), min=x[0], median=statistics.median(x), p90=x[int(0.9 * (len(x) - 1))], max=x[-1])


def excess_ms(state_ms, nop_ms):
    """Median state round trip minus median NOP round trip; None when either has no samples."""
    if not state_ms or not nop_ms:
        return None
    return statistics.median(state_ms) - statistics.median(nop_ms)


def floor_excess_ms(state_ms, nop_ms):
    """Fastest state round trip minus fastest NOP round trip (free of the relay-tick phase); None without samples."""
    if not state_ms or not nop_ms:
        return None
    return min(state_ms) - min(nop_ms)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", type=int, default=3, help="NOP + STATE_READ passes (default 3)")
    ap.add_argument("--n", type=int, default=100, help="calls per pass (default 100, one per 100 ms)")
    ap.add_argument("--max-excess-ms", type=float, default=5.0, help="exit 1 above this floor excess (default 5)")
    ap.add_argument("--out", default=None, help="json path (default temp/probes/platform_rtt_<stamp>.json)")
    args = ap.parse_args(argv)

    out = {"runs": [], "args": vars(args)}
    ident, ring, frames = [], [], []
    lock = threading.Lock()
    link = TeensyLinkClient()
    link.start()

    def on_id(t, s, pl, a):
        ident.append(p.BridgeIdentity.unpack(pl))

    def on_ring(t, s, pl, a):
        ring.append(p.RingDiag.unpack(pl))

    def on_pf(t, s, pl, a):
        now = time.monotonic()
        pf = p.PlatformFrame.unpack(pl)
        with lock:
            frames.append((now, int(pf.can_id), int(pf.dlc), bytes(pf.data[:pf.dlc])))

    link.subscribe(int(p.MsgType.BRIDGE_IDENTITY), on_id)
    link.subscribe(int(p.MsgType.RING_DIAG), on_ring)
    link.subscribe(int(p.MsgType.PLATFORM_FRAME), on_pf)
    rpc = RpcClient(link)
    time.sleep(3.0)                                   # listen: identity and ring diag at 1 Hz, IMU_STATUS relays
    out["identity"] = [(i.fw_version, i.protocol_version) for i in ident[-1:]]
    sts = [pi.decode_imu_status(d) for (_t, cid, dlc, d) in frames if cid == 0x6E5 and dlc == 8]
    if sts:
        s = sts[-1]
        out["imu_status"] = dict(state=s.state, flags=s.flags, temp_c=s.temp_c, fifo_ov=s.fifo_overflows,
                                 i2c=s.i2c_errors, sync_peak=s.sync_peak, fill_s=s.fill_s, boot=s.boot)
    if ring:
        out["ring_diag"] = {k: getattr(ring[-1], k) for k in ring[-1].__dataclass_fields__}

    def run_state():
        rtts, acks, miss = [], [], 0
        for _ in range(args.n):
            with lock:
                frames.clear()
            t0 = time.monotonic()
            try:
                rpc.call(int(p.RpcMethod.STATE_READ), b"", timeout=0.5, retries=0)
            except Exception:  # noqa: BLE001
                miss += 1
                time.sleep(0.1)
                continue
            t_ack = time.monotonic()
            got = None
            while time.monotonic() - t0 < 1.0 and got is None:
                with lock:
                    for (t, cid, dlc, _d) in frames:
                        if cid == 0x6E0 and dlc == 8:
                            got = t
                            break
                time.sleep(0.0005)
            if got is None:
                miss += 1
            else:
                rtts.append((got - t0) * 1e3)
                acks.append((t_ack - t0) * 1e3)
            time.sleep(max(0, 0.1 - (time.monotonic() - t0)))
        return dict(rtt_ms=stats(rtts), ack_ms=stats(acks), missed=miss, raw=[round(x, 2) for x in rtts])

    def run_nop():
        r, miss = [], 0
        for _ in range(args.n):
            t0 = time.monotonic()
            try:
                rpc.call(int(p.RpcMethod.NOP), b"", timeout=0.5, retries=0)
            except Exception:  # noqa: BLE001
                miss += 1
                time.sleep(0.1)
                continue
            r.append((time.monotonic() - t0) * 1e3)
            time.sleep(max(0, 0.1 - (time.monotonic() - t0)))
        return dict(rtt_ms=stats(r), missed=miss, raw=[round(x, 2) for x in r])

    excesses, floors = [], []
    try:
        for _ in range(args.runs):
            nop, st = run_nop(), run_state()
            ex = excess_ms(st["raw"], nop["raw"])
            fl = floor_excess_ms(st["raw"], nop["raw"])
            out["runs"].append({"nop": nop, "state": st, "excess_ms": ex, "floor_excess_ms": fl})
            if ex is not None:
                excesses.append(ex)
                floors.append(fl)
    finally:
        rpc.close()
        link.stop()
    median_excess = statistics.median(excesses) if excesses else None
    out["median_excess_ms"] = median_excess
    floor_excess = statistics.median(floors) if floors else None
    out["floor_excess_ms"] = floor_excess
    out["t_end"] = time.time()
    path = args.out or os.path.join(ROOT, "temp", "probes",
                                    "platform_rtt_%s.json" % time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        json.dump(out, fh, indent=1, default=str)
    print("platform_rtt_probe: floor excess %s ms (limit %.1f), median excess %s ms; wrote %s" % (
        "n/a" if floor_excess is None else "%.2f" % floor_excess, args.max_excess_ms,
        "n/a" if median_excess is None else "%.2f" % median_excess, path))
    if floor_excess is None:
        return 2
    return 1 if floor_excess > args.max_excess_ms else 0


if __name__ == "__main__":
    sys.exit(main())
