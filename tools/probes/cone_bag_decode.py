#!/usr/bin/env python3
"""Offline decoder for the catching cone's rosbag evidence table.

WHAT IT DECODES
---------------
Points at a rosbag (default: the newest bag under ``~/Desktop/rosbags/``;
``--all`` sweeps every bag) and prints one row per session:

    session          bag directory name
    dur              wall duration of the recording (s)
    hb               /cone/heartbeat message count
    connected        n_true/n_total  (the bridge saw a cone CAN heartbeat)
    time_synced      n_true/n_total  (the cone is locked to the global clock)
    sync@1st         did the FIRST published heartbeat already carry
                     connected=True AND time_synced=True?
    catch_seq        last_catch_seq range over heartbeats with have_any_catch
    taps             genuine cone taps DURING the session, reconstructed from
                     the heartbeat seq (wrap-aware, reboot-aware; see NOTE below).
                     A trailing `*` means a cone MCU REBOOT was detected inside
                     the session and the count is per-boot, NOT bridged across it.
    catch_ev         /cone/catch_event message count   (should equal `taps`)
    timing           /cone/timing_result message count (catch<->throw matches)

`taps` vs `catch_ev` is the load-bearing cross-check: `taps` is what the CONE
believes happened (heartbeat state), `catch_ev` is what the ROS2 stack actually
PUBLISHED. A divergence (` !` in the table) means catch frames were dropped
between the cone and the graph.

On the 17 sessions of 2026-07-12 they agree exactly (17/17) — but read that
result with its condition attached: **the agreement is CONDITIONAL on no catch
having followed either in-session cone reboot** (`18-34-09`, `18-44-56`, both
`*`-marked). Before the reboot handling below landed, the delta loop resumed
straight across the seq reset and would have invented ~250 phantom taps had a
catch followed one. 17/17 validates the transport only for the reboot-free
sessions; it is not a general property of the reconstruction.

MOTIVATING LOGBOOK ENTRIES
--------------------------
* ``logbook/2026-07-12-gui-cone-catch-correlation-regression.md`` — cone catches
  never reached the GUI (catch_correlation_node launch orphan + stale INSTALLED
  launch copy). The `timing` column is the ground truth for that fix: it is 0 in
  every session before the rebuild and non-zero after.
* ``logbook/2026-07-12-cone-sync-udp-link-contention.md`` — the follow-up
  "cone only syncs right after a colcon build" report. This probe REFUTED that
  mental model: `sync@1st` is Y for every session from 15:57 onward, across the
  19:17 rebuild boundary. It also supplied the evidence AGAINST that entry's own
  first attribution of the two dead morning sessions (`connected=0/N` for their
  whole duration) to a co-binding bench harness: the `connected` fraction plus the
  `*` reboot marker show the SAME dropout-then-reboot signature in sessions with
  no bench process running at all (`18-34-09`, `18-44-56`, `20-03-36`), which
  points at cone-side power/wiring intermittency rather than a UDP port race.

SAFETY / SCOPE
--------------
Strictly offline and read-only: it opens .mcap files and nothing else. It never
starts a node, never opens a socket, never touches hardware. It does NOT need
ROS2 — `mcap_ros2` decodes the message schemas embedded in the bag itself, so it
also runs on a non-ROS2 clone.

NOTE on `taps`, seq wrap, cone reboots, and the two firmware eras
-----------------------------------------------------------------
`last_catch_seq` is a uint8 that wraps at 256, and it RESETS to 0 when the cone
MCU reboots. Deltas are accumulated modulo 256, so a wrap reads correctly — but a
reboot must NOT be bridged, or the resumed delta reads as a ~250-tap jump. Two
reset detectors run, and either one sets the row's `*` marker, restarts the
accumulator, and drops the bridging step:

1. `have_any_catch` transitions True -> False. The flag can only clear on a
   reboot, so this is unambiguous. (This is the detector that fires on the two
   2026-07-12 reboots.)
2. A single-heartbeat delta above `MAX_PLAUSIBLE_TAPS_PER_HB`. A raw "seq went
   backwards" test is wrap-ambiguous (254 -> 2 is a legitimate 4-tap wrap), so the
   test is on the IMPLAUSIBILITY of the step instead: at ~10 Hz heartbeats no cone
   can register 64 taps in one interval, so a step that large is a restart.

The first catch after each (re)boot is counted explicitly (+1 per False->True
flip), because its own increment is invisible to a delta that starts at it.
`catch_ev` remains the tiebreaker if `taps` and the ROS graph ever disagree.

`taps` is reconstructed differently per firmware era (both verified against
`catch_ev` on real bags — see `taps_basis` in the --json output):

* **have_any_catch era** (bags from ~2026-06 on): before the cone's first catch,
  `last_catch_seq` is a PLACEHOLDER and `have_any_catch` is False. So the delta is
  accumulated only over heartbeats with `have_any_catch=True`, plus one per
  False->True flip. Verified: `taps == catch_ev` on 17/17 sessions of 2026-07-12
  (with the reboot caveat above).
* **legacy era** (the May `cone_test_*` bags): the firmware NEVER set
  `have_any_catch` (it is False in every heartbeat) while `last_catch_seq` advanced
  for real. Gating on the flag would silently report 0 taps, so when the flag is
  never seen the probe falls back to a plain wrap-aware delta over ALL heartbeats
  (detector 2 still guards it; detector 1 has no signal to work with).
  Verified: `taps == catch_ev` on the 3 legacy bags that have catches
  (`cone_test_2026-05-24_00-02-34` 5, `_12-39-19` 3, `_14-52-26` 6).

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/cone_bag_decode.py                    # newest bag
    python tools/probes/cone_bag_decode.py --all              # every bag (SLOW: see below)
    python tools/probes/cone_bag_decode.py --all --match 2026-07-12
    python tools/probes/cone_bag_decode.py --bag ~/Desktop/rosbags/2026-07-12_15-57-59
    python tools/probes/cone_bag_decode.py --all --json       # + temp/probes/ copy

A full ``--all`` sweep of ~230 bags takes several minutes: cost is dominated by
mcap CHUNK DECOMPRESSION (the chunks interleave every topic, so a 94 MB bag costs
~5 s even when only /cone/heartbeat is decoded — measured 2026-07-12). Bags with
no cone topics are skipped from their summary index for free. Narrow with
``--match <date>`` when you know which day you care about; progress goes to stderr.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from datetime import datetime
from typing import Any, Dict, List, Optional

# --- self-contained sys.path setup (repo root on the path; no project imports
#     are required today, but keeps the probe consistent with tools/probes/) ---
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

try:
    from mcap.reader import make_reader
    from mcap_ros2.reader import read_ros2_messages
except ImportError as exc:  # pragma: no cover - environment guard
    sys.exit(
        "ERROR: mcap / mcap_ros2 not importable ({}).\n"
        "Run under the project venv:  source ~/Desktop/PDJ_venv/venv/bin/activate".format(exc)
    )

DEFAULT_ROSBAG_ROOT = os.path.expanduser("~/Desktop/rosbags")
OUT_DIR = os.path.join(REPO_ROOT, "temp", "probes")

HEARTBEAT_TOPIC = "/cone/heartbeat"
CATCH_EVENT_TOPIC = "/cone/catch_event"
TIMING_RESULT_TOPIC = "/cone/timing_result"
THROW_TOPIC = "/throw_announcements"
COUNTED_TOPICS = (CATCH_EVENT_TOPIC, TIMING_RESULT_TOPIC, THROW_TOPIC)

SEQ_MODULO = 256  # last_catch_seq is a uint8

# Heartbeats run at ~10 Hz. A cone cannot register this many taps inside one
# heartbeat interval, so a seq step this large is a restart, not a wrap.
MAX_PLAUSIBLE_TAPS_PER_HB = 64


# --------------------------------------------------------------------------- #
# bag discovery
# --------------------------------------------------------------------------- #
def find_bags(root: str) -> List[str]:
    """Every immediate sub-directory of `root` that contains at least one .mcap."""
    bags = []
    for entry in sorted(os.listdir(root)):
        path = os.path.join(root, entry)
        if os.path.isdir(path) and glob.glob(os.path.join(path, "*.mcap")):
            bags.append(path)
    return bags


def newest_bag(root: str) -> Optional[str]:
    """Most recently written bag (by newest .mcap mtime — robust to bag naming)."""
    bags = find_bags(root)
    if not bags:
        return None
    return max(bags, key=lambda b: max(os.path.getmtime(m) for m in glob.glob(os.path.join(b, "*.mcap"))))


# --------------------------------------------------------------------------- #
# decoding
# --------------------------------------------------------------------------- #
def _summarise(mcap_paths: List[str]) -> Dict[str, Any]:
    """Read the mcap summary indexes: which topics exist + their message counts.

    Cheap (no message decode), so `--all` can skip the ~180 pre-cone bags fast.
    Bags written without a statistics section degrade to `topics=None`, which forces
    the caller onto the decode path for counts. The CHANNEL list is collected
    separately and survives a missing statistics section — it is what lets the
    decode path tell a genuine count of 0 apart from "topic absent from the bag".
    """
    topics: Dict[str, int] = {}
    have_summary = False
    channel_topics: Optional[set] = None
    t_start: Optional[int] = None
    t_end: Optional[int] = None
    for path in mcap_paths:
        with open(path, "rb") as fh:
            summary = make_reader(fh).get_summary()
        if summary is None:
            continue
        if summary.channels:
            if channel_topics is None:
                channel_topics = set()
            channel_topics.update(chan.topic for chan in summary.channels.values())
        if summary.statistics is None:
            continue
        have_summary = True
        chan_topic = {cid: chan.topic for cid, chan in summary.channels.items()}
        for cid, count in summary.statistics.channel_message_counts.items():
            topic = chan_topic.get(cid)
            if topic is not None:
                topics[topic] = topics.get(topic, 0) + count
        ms, me = summary.statistics.message_start_time, summary.statistics.message_end_time
        if ms:
            t_start = ms if t_start is None else min(t_start, ms)
        if me:
            t_end = me if t_end is None else max(t_end, me)
    duration_s = (t_end - t_start) / 1e9 if (t_start and t_end and t_end > t_start) else None
    return {
        "topics": topics if have_summary else None,
        "channel_topics": channel_topics,
        "duration_s": duration_s,
        "start_time": datetime.fromtimestamp(t_start / 1e9).isoformat(timespec="seconds") if t_start else None,
    }


def _count_by_decode(mcap_paths: List[str], topics: List[str]) -> Dict[str, int]:
    """Fallback counter for summary-less bags: iterate and count."""
    counts = {t: 0 for t in topics}
    for path in mcap_paths:
        for msg in read_ros2_messages(path, topics=topics):
            counts[msg.channel.topic] = counts.get(msg.channel.topic, 0) + 1
    return counts


def decode_bag(bag_dir: str) -> Dict[str, Any]:
    """Decode one bag directory into the cone evidence row."""
    mcap_paths = sorted(glob.glob(os.path.join(bag_dir, "*.mcap")))
    row: Dict[str, Any] = {
        "session": os.path.basename(os.path.normpath(bag_dir)),
        "path": bag_dir,
        "error": None,
        "duration_s": None,
        "start_time": None,
        "hb_total": 0,
        "connected_true": 0,
        "time_synced_true": 0,
        "connected_at_first_hb": None,
        "synced_at_first_hb": None,
        "synced_from_first_hb": None,   # connected AND time_synced on the very first HB
        "sync_rms_us_max": None,
        "catch_seq_first": None,
        "catch_seq_last": None,
        "catch_seq_delta": None,
        "taps": None,
        "taps_basis": None,             # have_any_catch | seq_delta_legacy | no_catches
        "seq_reset_detected": False,    # cone MCU rebooted mid-session ('*' in the table)
        "have_any_catch_at_start": None,
        "have_any_catch_ever": None,
        "catch_event_count": None,      # None => topic absent from the bag
        "timing_result_count": None,
        "throw_announcement_count": None,
        "has_cone_heartbeat": False,
    }
    if not mcap_paths:
        row["error"] = "no .mcap in bag dir (legacy sqlite3 bag?)"
        return row

    try:
        summary = _summarise(mcap_paths)
    except Exception as exc:  # noqa: BLE001 - a corrupt bag must not kill the sweep
        row["error"] = "summary read failed: {}".format(exc)
        return row

    row["duration_s"] = summary["duration_s"]
    row["start_time"] = summary["start_time"]
    topics = summary["topics"]
    channel_topics = summary["channel_topics"]

    # --- graceful degradation: topics absent from this bag stay None ---------
    if topics is not None:
        row["has_cone_heartbeat"] = HEARTBEAT_TOPIC in topics
        for topic, key in (
            (CATCH_EVENT_TOPIC, "catch_event_count"),
            (TIMING_RESULT_TOPIC, "timing_result_count"),
            (THROW_TOPIC, "throw_announcement_count"),
        ):
            if topic in topics:
                row[key] = topics[topic]
            elif channel_topics is not None and topic in channel_topics:
                # The bag RECORDED this topic and it carried zero messages. mcap's
                # channel_message_counts omits zero-count channels, so `topic not in
                # topics` conflates "recorded but silent" with "absent from the bag"
                # — and a None catch_ev is skipped by taps_mismatch(). A session where
                # the cone counted N taps and the graph published ZERO catch_events is
                # THE case this probe exists to catch (it is the 2026-07-12
                # catch_correlation_node orphan), so a real zero must read as 0.
                row[key] = 0
            # else: no channel for it at all -> genuinely absent -> stays None ('-')
    else:
        # No statistics index — decode to find out what's there.
        #
        # A genuine count of ZERO must NOT collapse to None. None means "topic
        # absent from the bag" (see the row init above), and `taps_mismatch()`
        # skips a None `catch_ev` — so a session where the cone counted N taps but
        # the ROS graph published ZERO catch_events would print `-`, get no ` !`
        # flag, and be dropped from the "catch frames lost" summary. That is
        # exactly the case this probe exists to catch.
        #
        # Prefer the bag's channel list, which distinguishes absent from zero for
        # real. Where the bag has no summary section at all, fall back to 0: an
        # absent topic then reads `0` rather than `-`, which is the safe direction
        # — a false "lost frames" flag is loud, a suppressed one is silent.
        counts = _count_by_decode(mcap_paths, [HEARTBEAT_TOPIC] + list(COUNTED_TOPICS))
        channel_topics = summary["channel_topics"]
        row["has_cone_heartbeat"] = counts.get(HEARTBEAT_TOPIC, 0) > 0
        for topic, key in (
            (CATCH_EVENT_TOPIC, "catch_event_count"),
            (TIMING_RESULT_TOPIC, "timing_result_count"),
            (THROW_TOPIC, "throw_announcement_count"),
        ):
            if channel_topics is not None and topic not in channel_topics:
                row[key] = None                      # genuinely absent from the bag
            else:
                row[key] = counts.get(topic, 0)      # a real 0 stays a real 0

    if not row["has_cone_heartbeat"]:
        return row  # pre-cone bag: nothing more to decode

    # --- heartbeat decode ---------------------------------------------------
    hb_total = 0
    connected_true = 0
    synced_true = 0
    rms_max = 0
    first: Optional[Dict[str, Any]] = None
    have_any_catch_at_start: Optional[bool] = None
    samples: List[Dict[str, Any]] = []  # (have_any, seq) per heartbeat

    try:
        for path in mcap_paths:
            for decoded in read_ros2_messages(path, topics=[HEARTBEAT_TOPIC]):
                msg = decoded.ros_msg
                connected = bool(getattr(msg, "connected", False))
                synced = bool(getattr(msg, "time_synced", False))
                have_any = bool(getattr(msg, "have_any_catch", False))
                seq = int(getattr(msg, "last_catch_seq", 0))
                rms = int(getattr(msg, "sync_rms_us", 0))

                hb_total += 1
                connected_true += int(connected)
                synced_true += int(synced)
                rms_max = max(rms_max, rms)
                if first is None:
                    first = {"connected": connected, "time_synced": synced}
                    have_any_catch_at_start = have_any
                samples.append({"have_any": have_any, "seq": seq})
    except Exception as exc:  # noqa: BLE001
        row["error"] = "heartbeat decode failed: {}".format(exc)
        return row

    row["hb_total"] = hb_total
    row["connected_true"] = connected_true
    row["time_synced_true"] = synced_true
    row["sync_rms_us_max"] = rms_max if hb_total else None
    if first is not None:
        row["connected_at_first_hb"] = first["connected"]
        row["synced_at_first_hb"] = first["time_synced"]
        row["synced_from_first_hb"] = first["connected"] and first["time_synced"]
    row["have_any_catch_at_start"] = have_any_catch_at_start

    # --- tap reconstruction (see module docstring: two firmware eras) --------
    have_any_ever = any(s["have_any"] for s in samples)
    row["have_any_catch_ever"] = have_any_ever

    # Modern firmware: last_catch_seq is a placeholder until have_any_catch, so the
    # delta is gated on the flag. Legacy firmware never set the flag while
    # last_catch_seq advanced for real, so gating there would report 0 taps for a
    # session full of catches -> fall back to a plain delta over every heartbeat.
    gate_on_have_any = have_any_ever
    basis = "have_any_catch" if have_any_ever else "seq_delta_legacy"

    seq_delta = 0
    bonus = 0
    seq_reset = False
    counted_seqs: List[int] = []
    prev_seq: Optional[int] = None
    prev_have_any: Optional[bool] = None

    for sample in samples:
        have_any = bool(sample["have_any"])
        seq = int(sample["seq"])

        if gate_on_have_any:
            if prev_have_any and not have_any:
                # have_any_catch can only clear on a cone MCU reboot. Break the
                # delta chain here so the resumed seq is not bridged across it.
                seq_reset = True
                prev_seq = None
            if not have_any:
                prev_have_any = have_any
                continue
            if prev_have_any is False:
                # First catch after a (re)boot. Its own increment is invisible to a
                # delta that STARTS at it, so count it explicitly and start fresh.
                bonus += 1
                prev_seq = None

        if prev_seq is not None:
            step = (seq - prev_seq) % SEQ_MODULO
            if step > MAX_PLAUSIBLE_TAPS_PER_HB:
                # Not a wrap (a wrap is a small step) and not reachable at ~10 Hz:
                # the sequence restarted. Drop the step rather than invent ~250 taps.
                seq_reset = True
                step = 0
            seq_delta += step
        counted_seqs.append(seq)
        prev_seq = seq
        prev_have_any = have_any

    row["seq_reset_detected"] = seq_reset
    if counted_seqs:
        row["catch_seq_first"] = counted_seqs[0]
        row["catch_seq_last"] = counted_seqs[-1]
        row["catch_seq_delta"] = seq_delta
        row["taps"] = seq_delta + bonus
        row["taps_basis"] = basis if (have_any_ever or seq_delta) else "no_catches"
    else:
        row["taps"] = 0
        row["taps_basis"] = "no_catches"
    return row


# --------------------------------------------------------------------------- #
# reporting
# --------------------------------------------------------------------------- #
def _fmt_pair(n_true: int, n_total: int) -> str:
    return "{}/{}".format(n_true, n_total)


def _fmt_opt(value: Optional[Any]) -> str:
    return "-" if value is None else str(value)


def header(width: int) -> str:
    return (
        "{sess:<{w}} {dur:>6} {hb:>5}  {conn:>11}  {sync:>11}  {first:^8}  "
        "{seq:>9} {taps:>5}   {ce:>8}  {tr:>6}".format(
            sess="session", w=width, dur="dur(s)", hb="hb", conn="connected",
            sync="time_synced", first="sync@1st", seq="catch_seq", taps="taps",
            ce="catch_ev", tr="timing",
        )
    )


def taps_mismatch(row: Dict[str, Any]) -> bool:
    """Cone-believed taps vs ROS-published catch_events (None = topic absent)."""
    ce = row["catch_event_count"]
    return ce is not None and row["taps"] is not None and ce != row["taps"]


def format_row(row: Dict[str, Any], width: int) -> str:
    sess = row["session"]
    if row["error"]:
        return "{:<{w}} ERROR: {}".format(sess, row["error"], w=width)
    if not row["has_cone_heartbeat"]:
        return "{:<{w}} {:>6} {:>5}  (no {} in bag)".format(
            sess,
            "{:.0f}".format(row["duration_s"]) if row["duration_s"] else "-",
            0,
            HEARTBEAT_TOPIC,
            w=width,
        )
    if row["taps_basis"] == "no_catches" or row["catch_seq_first"] is None:
        seq = "-"
    else:
        seq = "{}..{}".format(row["catch_seq_first"], row["catch_seq_last"])
    # "*"  => a cone MCU reboot was detected inside the session; `taps` is counted
    #         per-boot and NOT bridged across the seq reset.
    # " !" => the cone believed N taps but the ROS graph published M catch_events.
    flag = "*" if row.get("seq_reset_detected") else ""
    flag += " !" if taps_mismatch(row) else ""
    return (
        "{sess:<{w}} {dur:>6} {hb:>5}  {conn:>11}  {sync:>11}  {first:^8}  "
        "{seq:>9} {taps:>5}{flag:<3} {ce:>8}  {tr:>6}".format(
            sess=sess,
            w=width,
            dur="{:.0f}".format(row["duration_s"]) if row["duration_s"] else "-",
            hb=row["hb_total"],
            conn=_fmt_pair(row["connected_true"], row["hb_total"]),
            sync=_fmt_pair(row["time_synced_true"], row["hb_total"]),
            first="Y" if row["synced_from_first_hb"] else "N",
            seq=seq,
            taps=_fmt_opt(row["taps"]),
            flag=flag,
            ce=_fmt_opt(row["catch_event_count"]),
            tr=_fmt_opt(row["timing_result_count"]),
        )
    )


def _summary_line(label: str, rows: List[Dict[str, Any]], cap: int = 8) -> str:
    names = [r["session"] for r in rows]
    if not names:
        return "{}: none".format(label)
    shown = ", ".join(names[:cap])
    if len(names) > cap:
        shown += ", (+{} more)".format(len(names) - cap)
    return "{}: {}".format(label, shown)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Decode the catching cone's /cone/* topics out of rosbags "
        "into a per-session evidence table (offline, read-only).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--bag", action="append", default=None,
                        help="Bag directory to decode (repeatable). Default: newest bag under --root.")
    parser.add_argument("--all", action="store_true",
                        help="Sweep every bag under --root.")
    parser.add_argument("--root", default=DEFAULT_ROSBAG_ROOT,
                        help="Rosbag root directory (default: %(default)s).")
    parser.add_argument("--match", default=None,
                        help="With --all: only bags whose name contains this substring.")
    parser.add_argument("--show-empty", action="store_true",
                        help="With --all: also print bags that have no /cone/heartbeat.")
    parser.add_argument("--json", action="store_true",
                        help="Emit JSON on stdout and save a copy under temp/probes/.")
    args = parser.parse_args(argv)

    # --- select bags --------------------------------------------------------
    if args.bag:
        bags = [os.path.abspath(os.path.expanduser(b)) for b in args.bag]
    else:
        # Both root-driven paths need the root to exist. The default (newest-bag)
        # path used to fall through to os.listdir() and raise a raw
        # FileNotFoundError — and a missing ~/Desktop/rosbags is exactly the
        # no-args case on the non-ROS2 clone this probe advertises support for.
        if not os.path.isdir(args.root):
            sys.exit(
                "ERROR: rosbag root not found: {}\n"
                "Pass --root <dir> (or --bag <bag dir>) to point at your bags.".format(args.root)
            )
        if args.all:
            bags = find_bags(args.root)
            if args.match:
                bags = [b for b in bags if args.match in os.path.basename(b)]
        else:
            newest = newest_bag(args.root)
            if newest is None:
                sys.exit("ERROR: no bags with .mcap files under {}".format(args.root))
            bags = [newest]
    if not bags:
        sys.exit("ERROR: no bags selected")

    rows = []
    for i, bag in enumerate(bags, start=1):
        if len(bags) > 1:
            print("\r[{}/{}] {:<40}".format(i, len(bags), os.path.basename(bag)[:40]),
                  end="", file=sys.stderr, flush=True)
        rows.append(decode_bag(bag))
    if len(bags) > 1:
        print("\r{}\r".format(" " * 60), end="", file=sys.stderr, flush=True)

    # --- JSON ---------------------------------------------------------------
    if args.json:
        payload = {
            "generated": datetime.now().isoformat(timespec="seconds"),
            "root": args.root,
            "sessions": rows,
        }
        blob = json.dumps(payload, indent=2)
        print(blob)
        os.makedirs(OUT_DIR, exist_ok=True)
        out_path = os.path.join(
            OUT_DIR, "cone_bag_decode_{}.json".format(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        )
        with open(out_path, "w") as fh:
            fh.write(blob)
        print("\n[saved] {}".format(out_path), file=sys.stderr)
        return 0

    # --- table --------------------------------------------------------------
    shown = [r for r in rows if r["has_cone_heartbeat"] or r["error"] or args.show_empty]
    hidden = len(rows) - len(shown)
    width = max([len(r["session"]) for r in shown] + [len("session")])

    head = header(width)
    print(head)
    print("-" * len(head))
    for row in shown:
        print(format_row(row, width))
    print("-" * len(head))

    cone_rows = [r for r in rows if r["has_cone_heartbeat"]]
    dead = [r for r in cone_rows if r["hb_total"] and r["connected_true"] == 0]
    late_sync = [r for r in cone_rows if r["connected_true"] and not r["synced_from_first_hb"]]
    mismatched = [r for r in cone_rows if taps_mismatch(r)]
    resets = [r for r in cone_rows if r["seq_reset_detected"]]
    errors = [r for r in rows if r["error"]]

    print("{} bag(s) decoded; {} with /cone/heartbeat.".format(len(rows), len(cone_rows)))
    if hidden:
        print("{} bag(s) without /cone/heartbeat hidden (use --show-empty).".format(hidden))
    print(_summary_line(
        "connected=0 for the WHOLE session (bridge saw no cone CAN frames at all)", dead))
    print(_summary_line("cone connected but NOT synced-from-first-heartbeat", late_sync))
    print(_summary_line(
        "cone MCU REBOOT mid-session (`*`: seq reset; taps counted per-boot, "
        "NOT bridged across it)", resets))
    print(_summary_line(
        "taps != catch_event count (catch frames lost between cone and ROS graph)", mismatched))
    if errors:
        print(_summary_line("unreadable bag(s)", errors))
    return 0


if __name__ == "__main__":
    sys.exit(main())
