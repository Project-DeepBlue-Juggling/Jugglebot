#!/usr/bin/env python3
"""Per-session `/link_status` CONTROL table — was the bus in a comparable state?

WHAT IT DOES
    One line per rosbag session, derived entirely from `/link_status`:
    session span, the non-OK duty of a chosen health row, and the counter
    RATES that say what the bus was actually doing — `setpoints_sent`/s
    (was the robot being commanded, or idle?), UDP `tx_frames`/s and
    `rx_frames`/s (was the Jetson<->bridge link loaded?) — plus the
    can-bridge `uptime_ms` span, the `mpc_active` values seen, the
    `platform_fw_version` values seen, and which deployment-marker rows were
    present.

WHY IT EXISTS
    Companion to `link_status_health_scan.py`. That probe finds *whether* a
    health duty moved across a firmware flash; this one answers the question
    that immediately follows and decides whether the comparison is even
    admissible: **were the compared sessions in comparable states?**

    Concretely, on 2026-07-29 the operator flashed can-bridge FW_VERSION 4
    (the hand ball-sensor GPIO poller) and the sitting that followed showed
    `bus1_health` flapping OK<->WARN at 42.4 % duty. The attribution "the
    poller's added CAN3 traffic did it" only survives if some PRE-flash
    session was also idle (setpoints frozen, bus quiet) and did NOT flap —
    otherwise "an idle bus flaps on its own" is an equally good explanation
    and the two cannot be separated. Finding that control session is exactly
    a scan of `setpoints_sent`/s against the health duty, per session.

    Motivating bag: `/home/jetson/Desktop/rosbags/2026-07-29_22-37-06/`
    (proves FW4 ran: `/link_status` carries a live `hand_ball_sensor` row,
    which exists only in FW4 + the Phase 5 ROS surface).

THE GENERALISATION
    Every firmware flash produces the same two-part question — "did X change?"
    and "was anything else different?" — and for the can-bridge both halves
    are answerable from `/link_status` alone, offline, from bags that already
    exist. Point this at the sessions either side of any future flash and read
    the rate columns before believing a duty-cycle delta.

    Strictly offline and read-only: opens `.mcap` files only — no node, no
    socket, no hardware — and needs NO ROS2 installed (`mcap_ros2` decodes the
    schemas embedded in the bag). It does need the project venv for
    `mcap_ros2`.

REFERENCES
    Logbook: `logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md`
    Plan:    `plans/archived/2026-08-15 hand-ball-sensor.md` (Phases 3-5, the flash)
    Runbook: `tests/hardware/session_hand_ball_sensor.md` § P5 (the on-board
             serial poller-ON/OFF A/B; this probe is its offline counterpart)
    Sibling: `tools/probes/link_status_health_scan.py`

USAGE
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/archived/link_status_flash_control.py <bag.mcap|session-dir> ...
    python tools/probes/archived/link_status_flash_control.py --sessions 20
    python tools/probes/archived/link_status_flash_control.py --match 2026-07-2 --out

    `--out` writes a CSV under `temp/probes/` (gitignored). stdout is the
    primary human surface.

    Exit 0 = every input scanned; 1 = at least one bag was unreadable or
    carried no `/link_status` (reported per-bag, the sweep continues).

CAVEAT
    The rates are (last - first) / span on MONOTONIC counters. A counter that
    wraps or resets mid-session (a bridge reboot resets `uptime_ms` and the
    frame counters with it) reads as a spuriously low or negative rate — check
    the `uptime` column before trusting a rate, exactly as the standing
    uptime-logging rule requires for any timing number on this robot.
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import re
import sys
from datetime import datetime

# Session dirs are named `YYYY-MM-DD_HH-MM-SS`. The rosbags root also holds
# `cone_test_*` dirs and an `Old naming scheme` dir, which a plain
# reverse-sort puts FIRST. Filter on a date-like prefix before sorting.
_SESSION_RE = re.compile(r"^\d{4}-\d{2}-\d{2}")

DEFAULT_ROOT = "/home/jetson/Desktop/rosbags"
DEFAULT_HEALTH_KEY = "bus1_health"
DEFAULT_MARKERS = ("hand_ball_sensor", "odrive_fw_versions")

_COUNTERS = ("setpoints_sent", "tx_frames", "rx_frames")

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def discover(root: str, n_sessions: int, match: str | None) -> list[str]:
    """Newest `n_sessions` DATED session dirs under `root`, expanded to .mcap paths."""
    try:
        names = os.listdir(root)
    except OSError as exc:
        print(f"ERROR: cannot list {root}: {exc}", file=sys.stderr)
        return []
    dated = [d for d in names if _SESSION_RE.match(d) and os.path.isdir(os.path.join(root, d))]
    if match:
        dated = [d for d in dated if match in d]
    dated.sort(reverse=True)          # dated names sort lexicographically = chronologically
    out: list[str] = []
    for d in dated[:n_sessions]:
        out.extend(sorted(glob.glob(os.path.join(root, d, "*.mcap"))))
    return out


def expand(args_paths: list[str]) -> list[str]:
    """Accept .mcap files, session dirs, or globs; return a flat .mcap list."""
    out: list[str] = []
    for p in args_paths:
        if os.path.isdir(p):
            out.extend(sorted(glob.glob(os.path.join(p, "*.mcap"))))
        else:
            out.append(p)
    return out


def scan(path: str, health_key: str, markers: tuple[str, ...]) -> tuple[dict | None, str | None]:
    from mcap_ros2.reader import read_ros2_messages

    n = 0
    t0 = tlast = None
    first: dict[str, int] = {}
    last: dict[str, int] = {}
    up_first = up_last = None
    health: dict[str, int] = {}
    mpc: set[str] = set()
    pfw: set[str] = set()
    present: set[str] = set()

    try:
        for m in read_ros2_messages(path, topics=["/link_status"]):
            t = m.log_time_ns * 1e-9
            if t0 is None:
                t0 = t
            tlast = t
            kv = {v.key: v.value for v in m.ros_msg.values}
            n += 1
            for k in _COUNTERS:
                try:
                    val = int(kv[k])
                except (KeyError, TypeError, ValueError):
                    continue
                first.setdefault(k, val)
                last[k] = val
            try:
                up = int(kv["uptime_ms"])
            except (KeyError, TypeError, ValueError):
                pass
            else:
                if up_first is None:
                    up_first = up
                up_last = up
            h = kv.get(health_key)
            if h:
                health[h] = health.get(h, 0) + 1
            if "mpc_active" in kv:
                mpc.add(kv["mpc_active"])
            if "platform_fw_version" in kv:
                pfw.add(kv["platform_fw_version"])
            for k in markers:
                if k in kv:
                    present.add(k)
    except Exception as exc:                                  # truncated bag, bad chunk, ...
        return None, f"ERROR {type(exc).__name__}: {exc}"
    if n == 0:
        return None, "no /link_status"

    span = (tlast - t0) if (tlast is not None and t0 is not None) else 0.0
    denom = span or 1.0
    rates = {k: ((last[k] - first[k]) / denom) if k in first else None for k in _COUNTERS}
    tot = sum(health.values()) or 1
    return (
        dict(
            n=n,
            span=span,
            health=health,
            health_key=health_key,
            not_ok_pct=100.0 * (tot - health.get("OK", 0)) / tot if health else None,
            rates=rates,
            uptime_first=up_first,
            uptime_last=up_last,
            mpc=sorted(mpc),
            pfw=sorted(pfw),
            markers=sorted(present),
        ),
        None,
    )


def _fmt(x: float | None, width: int = 8) -> str:
    return f"{x:{width}.1f}" if x is not None else "n/a".rjust(width)


def _default_out_path() -> str:
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return os.path.join(_REPO_ROOT, "temp", "probes", f"link_status_flash_control_{stamp}.csv")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Per-session /link_status control table (bus state either side of a flash).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("bags", nargs="*", help=".mcap files, session dirs, or globs")
    ap.add_argument("--root", default=DEFAULT_ROOT, help=f"rosbags root (default {DEFAULT_ROOT})")
    ap.add_argument("--sessions", type=int, default=14,
                    help="when no bags are named, scan the newest N DATED session dirs (default 14)")
    ap.add_argument("--match", default=None, help="substring filter on the session dir name")
    ap.add_argument("--health-key", default=DEFAULT_HEALTH_KEY,
                    help=f"KeyValue row scored for non-OK duty (default {DEFAULT_HEALTH_KEY})")
    ap.add_argument("--markers", default=None,
                    help="comma-separated rows whose PRESENCE is reported "
                         f"(default: {','.join(DEFAULT_MARKERS)})")
    ap.add_argument("--out", nargs="?", const="", default=None, metavar="PATH",
                    help="write a CSV summary (default: a timestamped file under temp/probes/)")
    args = ap.parse_args(argv)

    markers = (tuple(k.strip() for k in args.markers.split(",") if k.strip())
               if args.markers else DEFAULT_MARKERS)

    paths = expand(args.bags) if args.bags else discover(args.root, args.sessions, args.match)
    if not paths:
        print("no bags to scan", file=sys.stderr)
        return 1

    rows: list[dict] = []
    bad = 0
    for p in paths:
        name = os.path.basename(os.path.dirname(p))
        res, err = scan(p, args.health_key, markers)
        if err:
            print(f"{name:26s}  {err}")
            rows.append(dict(session=name, error=err))
            bad += 1
            continue
        dist = ",".join(f"{v}={c}" for v, c in sorted(res["health"].items(), key=lambda x: -x[1]))
        print(
            f"{name:26s} span={res['span']:6.1f}s  "
            f"{args.health_key}!OK={_fmt(res['not_ok_pct'], 5)}%  "
            f"setpt/s={_fmt(res['rates']['setpoints_sent'])}  "
            f"udp_tx/s={_fmt(res['rates']['tx_frames'])}  "
            f"udp_rx/s={_fmt(res['rates']['rx_frames'])}  "
            f"up={res['uptime_first']}->{res['uptime_last']}  "
            f"mpc={res['mpc']} pfw={res['pfw']}"
        )
        if dist:
            print(f"    {args.health_key}: {dist}")
        if res["markers"]:
            print(f"    markers present: {' '.join(res['markers'])}")
        rows.append(dict(
            session=name,
            span_s=round(res["span"], 3),
            health_key=args.health_key,
            not_ok_pct=None if res["not_ok_pct"] is None else round(res["not_ok_pct"], 2),
            health_dist=dist,
            setpoints_per_s=res["rates"]["setpoints_sent"],
            udp_tx_per_s=res["rates"]["tx_frames"],
            udp_rx_per_s=res["rates"]["rx_frames"],
            uptime_first_ms=res["uptime_first"],
            uptime_last_ms=res["uptime_last"],
            mpc_active=" ".join(res["mpc"]),
            platform_fw_version=" ".join(res["pfw"]),
            markers=" ".join(res["markers"]),
        ))

    if args.out is not None:
        out_path = args.out or _default_out_path()
        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        fields = ["session", "span_s", "health_key", "not_ok_pct", "health_dist",
                  "setpoints_per_s", "udp_tx_per_s", "udp_rx_per_s",
                  "uptime_first_ms", "uptime_last_ms", "mpc_active",
                  "platform_fw_version", "markers", "error"]
        with open(out_path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        print(f"\nCSV summary -> {out_path}")

    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
