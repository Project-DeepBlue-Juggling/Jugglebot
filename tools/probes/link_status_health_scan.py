#!/usr/bin/env python3
"""Per-session value-duty scan of `/link_status` — the "did this change across a flash?" probe.

WHAT IT DOES
    Opens one or more rosbag sessions read-only, reads every `/link_status`
    message, and reports — per session — how long the session ran, the
    can-bridge `uptime_ms` span, and the **value distribution of each watched
    KeyValue row** (e.g. `bus1_health[OK=1234(58%),WARN=902(42%)]`). It also
    reports which *marker* rows were present at all, because a KeyValue that
    only exists in a newer firmware / ROS build is itself a deployment
    fingerprint.

WHY IT EXISTS
    On 2026-07-29 the operator flashed can-bridge FW_VERSION 4 (the hand
    ball-sensor GPIO poller) and ran a live sitting. That session's
    `bus1_health` flapped OK<->WARN at a 42.4 % duty — a CAN3 bus-health
    regression that no single message reveals: it is only visible as a *duty
    cycle over a whole session*, and it only means anything when compared
    against the sessions recorded before the flash. Answering "was CAN3 always
    like this, or did the flash do it?" meant scanning a stack of bags for
    exactly this statistic.

    Motivating bag: `/home/jetson/Desktop/rosbags/2026-07-29_22-37-06/`
    (proves FW4 ran: `/link_status` carries a live `hand_ball_sensor` row,
    which exists only in FW4 + the Phase 5 ROS surface).

THE GENERALISATION
    `/link_status` is the can-bridge's whole observable state surface, and
    almost every row on it is a low-cardinality enum (health, link state,
    validity). So "did behaviour X change across a firmware flash?" reduces,
    for a large class of X, to "did the value duty of some `/link_status` row
    move between the pre-flash and post-flash sessions?" — which is what this
    probe measures. Every future can-bridge flash gets the same question;
    point it at the last N dated sessions and read down the column.

    Sibling: `link_status_flash_control.py`, which answers the *control* half
    ("was the bus in a comparable state in those sessions?") from the same
    topic.

    Strictly offline and read-only: opens `.mcap` files only — no node, no
    socket, no hardware — and needs NO ROS2 installed (`mcap_ros2` decodes the
    schemas embedded in the bag). It does need the project venv for
    `mcap_ros2`.

REFERENCES
    Logbook: `logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md`
    Plan:    `plans/active/hand-ball-sensor.md` (Phases 3-5, the flash)
    Runbook: `tests/hardware/session_hand_ball_sensor.md` § P5 (the on-board
             serial A/B this probe is the offline counterpart to)

USAGE
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/link_status_health_scan.py <bag.mcap|session-dir> ...
    python tools/probes/link_status_health_scan.py --sessions 14
    python tools/probes/link_status_health_scan.py --match 2026-07-29 --out

    `--out` writes a JSON summary under `temp/probes/` (gitignored). stdout is
    the primary human surface.

    Exit 0 = every input scanned; 1 = at least one bag was unreadable or
    carried no `/link_status` (reported per-bag, the sweep continues).
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import sys
from datetime import datetime

# Session dirs are named `YYYY-MM-DD_HH-MM-SS`. The rosbags root also holds
# `cone_test_*` dirs and an `Old naming scheme` dir, which a plain
# reverse-sort puts FIRST — so "newest N sessions" silently returned the
# wrong bags. Filter on a date-like prefix before sorting.
_SESSION_RE = re.compile(r"^\d{4}-\d{2}-\d{2}")

DEFAULT_ROOT = "/home/jetson/Desktop/rosbags"

# Low-cardinality enum rows worth a duty cycle. Bus naming has drifted across
# firmware eras (`bus1_health` vs `can3_health` vs `jugglebot_health`), so the
# default watchlist carries all of them and simply skips the absent ones.
DEFAULT_KEYS = (
    "bus0_health",
    "bus1_health",
    "bus2_health",
    "can1_health",
    "can2_health",
    "can3_health",
    "jugglebot_health",
    "bb_health",
    "cone_health",
    "bridge_link",
)

# Rows whose mere PRESENCE fingerprints a deployment.
DEFAULT_MARKERS = ("hand_ball_sensor", "odrive_fw_versions", "platform_fw_version")

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


def scan(path: str, keys: tuple[str, ...], markers: tuple[str, ...]) -> tuple[dict | None, str | None]:
    from mcap_ros2.reader import read_ros2_messages

    n = 0
    t0 = tlast = None
    counts: dict[str, dict[str, int]] = {}
    present: set[str] = set()
    all_keys: set[str] = set()
    uptime_first = uptime_last = None
    try:
        for m in read_ros2_messages(path, topics=["/link_status"]):
            t = m.log_time_ns * 1e-9
            if t0 is None:
                t0 = t
            tlast = t
            kv = {v.key: v.value for v in m.ros_msg.values}
            all_keys.update(kv)
            n += 1
            for k in keys:
                if k in kv:
                    counts.setdefault(k, {})
                    counts[k][kv[k]] = counts[k].get(kv[k], 0) + 1
            for k in markers:
                if k in kv:
                    present.add(k)
            if "uptime_ms" in kv:
                if uptime_first is None:
                    uptime_first = kv["uptime_ms"]
                uptime_last = kv["uptime_ms"]
    except Exception as exc:                                  # truncated bag, bad chunk, ...
        return None, f"ERROR {type(exc).__name__}: {exc}"
    if n == 0:
        return None, "no /link_status"
    return (
        dict(
            n=n,
            span=(tlast - t0) if (tlast is not None and t0 is not None) else 0.0,
            counts=counts,
            markers=sorted(present),
            n_keys=len(all_keys),
            keys=sorted(all_keys),
            uptime_first=uptime_first,
            uptime_last=uptime_last,
        ),
        None,
    )


def _uptime_str(res: dict) -> str:
    if res["uptime_first"] is None:
        return ""
    try:
        return (
            f"  uptime {int(res['uptime_first']) / 1000.0:.0f}->"
            f"{int(res['uptime_last']) / 1000.0:.0f}s"
        )
    except (TypeError, ValueError):
        return f"  uptime {res['uptime_first']}->{res['uptime_last']}"


def _default_out_path() -> str:
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return os.path.join(_REPO_ROOT, "temp", "probes", f"link_status_health_scan_{stamp}.json")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Per-session /link_status value-duty scan (flash A/B instrument).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("bags", nargs="*", help=".mcap files, session dirs, or globs")
    ap.add_argument("--root", default=DEFAULT_ROOT, help=f"rosbags root (default {DEFAULT_ROOT})")
    ap.add_argument("--sessions", type=int, default=14,
                    help="when no bags are named, scan the newest N DATED session dirs (default 14)")
    ap.add_argument("--match", default=None, help="substring filter on the session dir name")
    ap.add_argument("--keys", default=None,
                    help="comma-separated KeyValue rows to tabulate (default: the bus-health set)")
    ap.add_argument("--markers", default=None,
                    help="comma-separated rows whose PRESENCE is reported "
                         f"(default: {','.join(DEFAULT_MARKERS)})")
    ap.add_argument("--top", type=int, default=6,
                    help="max distinct values printed per key (default 6)")
    ap.add_argument("--list-keys", action="store_true", help="also print every key seen in the bag")
    ap.add_argument("--out", nargs="?", const="", default=None, metavar="PATH",
                    help="write a JSON summary (default: a timestamped file under temp/probes/)")
    args = ap.parse_args(argv)

    keys = tuple(k.strip() for k in args.keys.split(",") if k.strip()) if args.keys else DEFAULT_KEYS
    markers = (tuple(k.strip() for k in args.markers.split(",") if k.strip())
               if args.markers else DEFAULT_MARKERS)

    paths = expand(args.bags) if args.bags else discover(args.root, args.sessions, args.match)
    if not paths:
        print("no bags to scan", file=sys.stderr)
        return 1

    results: list[dict] = []
    bad = 0
    for p in paths:
        name = os.path.basename(os.path.dirname(p))
        res, err = scan(p, keys, markers)
        if err:
            print(f"{name:26s}  {err}")
            results.append(dict(session=name, path=p, error=err))
            bad += 1
            continue
        print(f"{name:26s} n={res['n']:5d} span={res['span']:6.1f}s{_uptime_str(res)}"
              f"  keys={res['n_keys']}")
        for k in sorted(res["counts"]):
            dist = res["counts"][k]
            tot = sum(dist.values()) or 1
            ordered = sorted(dist.items(), key=lambda x: -x[1])
            shown = ordered[: args.top]
            parts = ",".join(f"{v}={c}({100.0 * c / tot:.0f}%)" for v, c in shown)
            more = f",+{len(ordered) - len(shown)} more" if len(ordered) > len(shown) else ""
            print(f"    {k}[{parts}{more}]")
        if res["markers"]:
            print(f"    markers present: {' '.join(res['markers'])}")
        if args.list_keys:
            print(f"    all keys: {' '.join(res['keys'])}")
        row = dict(res)
        if not args.list_keys:
            row.pop("keys", None)
        row.update(session=name, path=p)
        results.append(row)

    if args.out is not None:
        out_path = args.out or _default_out_path()
        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        with open(out_path, "w") as fh:
            json.dump(dict(generated=datetime.now().isoformat(timespec="seconds"),
                           keys=list(keys), markers=list(markers), sessions=results), fh, indent=2)
        print(f"\nJSON summary -> {out_path}")

    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
