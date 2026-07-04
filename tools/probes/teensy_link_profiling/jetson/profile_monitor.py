#!/usr/bin/env python3
"""Jetson-side consumer for the can-bridge Teensy's diagnostic UDP stream.

Binds the STREAM port, ingests PROFILE frames (per-task CPU%, wire-slot bus
utilisation — slot 1 = Jugglebot core CAN3, slot 2 = Ball Butler CAN1 —
UDP RTT/jitter, the 500 Hz interp deadline-miss counter, free heap)
plus a tally of the other uplink frame types, logs PROFILE rows to CSV, and on
exit renders matplotlib plots. Single-script run.

Usage:
    python profile_monitor.py [--duration S] [--bind-ip IP] [--no-plot]
                              [--out-dir DIR]

Outputs (default `temp/probes/teensy_link_profiling/`, gitignored):
    profile_<ts>.csv   — one row per PROFILE frame
    profile_<ts>.png   — CPU / CAN util / RTT / deadline-miss / heap plots

The PROFILE cpu_pct_x100[] slot order is shared with the firmware (profiling.h):
"""

from __future__ import annotations

import argparse
import csv
import os
import socket
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", "..", ".."))
sys.path.insert(0, _HERE)        # delivered udp_protocol.py
import udp_protocol as p         # noqa: E402

# Slot order — MUST match firmware profiling.cpp kTaskSlots.
TASK_SLOTS = ["canrx", "tsync", "net", "fault", "telem", "hb", "diag", "IDLE", "other"]


def _default_out_dir():
    return os.path.join(_REPO, "temp", "probes", "teensy_link_profiling")


def run(bind_ip, port, duration, out_dir, do_plot):
    os.makedirs(out_dir, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(out_dir, f"profile_{ts}.csv")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_ip, port))
    sock.settimeout(0.5)
    print(f"[profile_monitor] listening on {bind_ip}:{port}  → {csv_path}")
    if duration:
        print(f"[profile_monitor] running for {duration}s (Ctrl-C to stop early)")

    cols = (["t_teensy_us"] + [f"cpu_{n}" for n in TASK_SLOTS] +
            ["can1_rx", "can1_tx", "can2_rx", "can2_tx", "can1_util_pct", "can2_util_pct",
             "udp_rtt_us", "udp_jitter_us", "interp_deadline_misses",
             "interp_max_jitter_us", "free_heap_bytes"])
    rows = []
    frame_counts = {}
    t0 = time.time()
    try:
        while True:
            if duration and (time.time() - t0) >= duration:
                break
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            try:
                mt, seq, payload = p.decode_frame(data)
            except ValueError as e:
                frame_counts["BAD_CRC"] = frame_counts.get("BAD_CRC", 0) + 1
                continue
            frame_counts[mt] = frame_counts.get(mt, 0) + 1
            if mt == p.MsgType.PROFILE:
                pr = p.Profile.unpack(payload)
                row = ([pr.t_teensy_us] + [c / 100.0 for c in pr.cpu_pct_x100] +
                       [pr.can1_rx, pr.can1_tx, pr.can2_rx, pr.can2_tx,
                        pr.can1_util_x100 / 100.0, pr.can2_util_x100 / 100.0,
                        pr.udp_rtt_us, pr.udp_jitter_us, pr.interp_deadline_misses,
                        pr.interp_max_jitter_us, pr.free_heap_bytes])
                rows.append(row)
                # can1_* wire slot = Jugglebot core / CAN3 (the busy, safety-relevant
                # bus) after the three-bus remap (HANDOFF D4); show it live.
                print(f"[profile] core_util(CAN3)={pr.can1_util_x100/100:.1f}%  "
                      f"rtt={pr.udp_rtt_us}us  miss={pr.interp_deadline_misses}  "
                      f"heap={pr.free_heap_bytes}")
    except KeyboardInterrupt:
        print("\n[profile_monitor] stopped")
    finally:
        sock.close()

    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        w.writerows(rows)
    print(f"[profile_monitor] wrote {len(rows)} PROFILE rows → {csv_path}")
    print(f"[profile_monitor] frame tally: "
          + ", ".join(f"{_name(k)}={v}" for k, v in sorted(frame_counts.items(), key=str)))

    if do_plot and rows:
        _plot(csv_path, cols, rows)
    return csv_path


def _name(mt):
    if mt == "BAD_CRC":
        return "BAD_CRC"
    try:
        return p.MsgType(mt).name
    except ValueError:
        return f"0x{mt:02X}"


def _plot(csv_path, cols, rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[profile_monitor] matplotlib unavailable — skipping plots")
        return
    import numpy as np
    arr = np.array(rows, dtype=float)
    col = {c: i for i, c in enumerate(cols)}
    t = (arr[:, col["t_teensy_us"]] - arr[0, col["t_teensy_us"]]) / 1e6  # s since first

    fig, ax = plt.subplots(4, 1, figsize=(11, 13), sharex=True)
    # CPU per task (stacked).
    cpu = np.vstack([arr[:, col[f"cpu_{n}"]] for n in TASK_SLOTS])
    ax[0].stackplot(t, cpu, labels=TASK_SLOTS)
    ax[0].set_ylabel("CPU %"); ax[0].set_title("Per-task CPU (interp runs in ISR — see deadline misses)")
    ax[0].legend(loc="upper right", ncol=3, fontsize=7)
    # CAN utilisation.
    # Wire slots were remapped by the three-bus refactor (firmware HANDOFF D4):
    # PROFILE can1_* now sources Jugglebot core (CAN3), can2_* sources Ball Butler
    # (CAN1). Cone (CAN2) util is not yet on the uplink (TODO phase-10b).
    ax[1].plot(t, arr[:, col["can1_util_pct"]], label="Jugglebot core / CAN3 (slot can1)")
    ax[1].plot(t, arr[:, col["can2_util_pct"]], label="Ball Butler / CAN1 (slot can2)")
    ax[1].set_ylabel("bus util %"); ax[1].legend(loc="upper right"); ax[1].grid(True, alpha=0.3)
    # UDP RTT + jitter.
    ax[2].plot(t, arr[:, col["udp_rtt_us"]], label="RTT µs")
    ax[2].plot(t, arr[:, col["udp_jitter_us"]], label="jitter µs")
    ax[2].set_ylabel("UDP µs"); ax[2].legend(loc="upper right"); ax[2].grid(True, alpha=0.3)
    # Interp health + heap.
    ax2b = ax[3].twinx()
    ax[3].plot(t, arr[:, col["interp_deadline_misses"]], "r-", label="deadline misses (cum)")
    ax[3].plot(t, arr[:, col["interp_max_jitter_us"]], "m-", label="interp max jitter µs/win")
    ax2b.plot(t, arr[:, col["free_heap_bytes"]] / 1024.0, "g--", label="free heap KB")
    ax[3].set_ylabel("interp"); ax2b.set_ylabel("heap KB"); ax[3].set_xlabel("time (s)")
    ax[3].legend(loc="upper left"); ax2b.legend(loc="upper right"); ax[3].grid(True, alpha=0.3)

    png = csv_path.replace(".csv", ".png")
    fig.tight_layout(); fig.savefig(png, dpi=110)
    print(f"[profile_monitor] wrote plots → {png}")


def main():
    ap = argparse.ArgumentParser(description="Can-bridge Teensy diagnostic UDP consumer")
    ap.add_argument("--bind-ip", default="0.0.0.0",
                    help="local IP to bind (default 0.0.0.0; the teensy-link is 192.168.42.1)")
    ap.add_argument("--port", type=int, default=p.PORT_STREAM)
    ap.add_argument("--duration", type=float, default=0.0, help="seconds (0 = until Ctrl-C)")
    ap.add_argument("--out-dir", default=_default_out_dir())
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    run(args.bind_ip, args.port, args.duration, args.out_dir, not args.no_plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
