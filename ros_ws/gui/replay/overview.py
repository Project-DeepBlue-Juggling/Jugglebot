"""The overview pass: timeline bands, ticks and presence for ONE recording.

Venv-only (needs ``mcap``, ``rosbags``); the GUI server runs it niced as a
subprocess, one at a time (``api.py``). Design 06 § 1. The browser reads the
MCAP itself; this pass exists only because the timeline strip needs the values
of a few low-rate topics plus the ``/robot_state`` flag fields, and the stdlib
server has no CDR decoder.

Usage (``cwd=ros_ws/gui``)::

    python -m replay.overview --bag <file.mcap> --out <overview.json>

One log-time-ordered pass over an INDEXED bag (exit 2 for an unindexed one).
Presence is binned per ``CHUNK_S`` slot from ``log_time`` alone, with t0 = the
first allow-listed message, exactly the browser worker's slot rule. Only the
overview topics are CDR-decoded:

- ``/orchestrator_state``, ``/cone/catch_event``, ``/skills/attempt``,
  ``/bb/calibration_attempt``: low rate, decoded in full and fed to
  ``decode.Overview``;
- ``/robot_state`` (about 100 Hz, the dominant cost): NOT decoded. The five
  flags the ticks need sit after two variable-length arrays, so a hand-written
  CDR walk (``robot_state_flags``) skips ``motor_states`` (fixed 44-byte
  elements) and ``error`` (length-prefixed strings) and reads five bytes. The
  walk is checked against the full rosbags decode on the first message (and
  every ``SELF_CHECK_EVERY``-th after): a first-message mismatch falls back to
  full decode for the whole pass, a later one fails the pass. ``Overview`` is
  fed only when the flag tuple changes (between changes it cannot tick).

Output (atomic) = ``Overview.build`` + ``"source": {"size_bytes", "mtime"}``.
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time
from typing import Dict, List, Optional, Set, Tuple

from mcap.reader import SeekingReader

from . import schema
from .decode import FAULT_FLAGS, Decoder, Overview, is_indexed

SELF_CHECK_EVERY = 20000
ROBOT_STATE = "/robot_state"
# Topics decoded in full (everything else allow-listed contributes presence only).
FULL_DECODE = frozenset((schema.OVERVIEW_BAND_TOPIC, "/cone/catch_event",
                         "/skills/attempt", "/bb/calibration_attempt"))
# The 7-bool run in message order is: the three FAULT_FLAGS, firmware_validated,
# encoder_search_complete, is_homed, levelling_complete; _FLAG_IDX picks the five used.
_FLAG_NAMES = FAULT_FLAGS + ("is_homed", "levelling_complete")
_FLAG_IDX = (0, 1, 2, 5, 6)
_MOTOR_BYTES = 44  # MotorStateSingle: u32 u32 u8 u8 bool(pad 1) + 8 x f32, 4-aligned


def _log(msg: str) -> None:
    sys.stderr.write("[replay.overview] " + msg + "\n")
    sys.stderr.flush()


def robot_state_flags(data: bytes) -> Tuple[bool, ...]:
    """The five overview flags of a CDR-LE RobotState, without a full decode."""
    if data[1] != 1:
        raise ValueError("not little-endian CDR")
    o = 4 + 8  # encapsulation header; Time (int32 sec, uint32 nanosec)
    (n,) = struct.unpack_from("<I", data, o)
    o += 4 + n * _MOTOR_BYTES
    (n,) = struct.unpack_from("<I", data, o)
    o += 4
    for _ in range(n):
        o = (o + 3) & ~3  # string length prefix is 4-aligned (offsets here are absolute; header is 4 bytes)
        (ln,) = struct.unpack_from("<I", data, o)
        o += 4 + ln
    if o + 7 > len(data):
        raise ValueError("short RobotState")
    return tuple(bool(data[o + i]) for i in _FLAG_IDX)


def _flags_from_obj(obj) -> Tuple[bool, ...]:
    return tuple(bool(getattr(obj, f)) for f in _FLAG_NAMES)


def compute(bag_path: str) -> Dict:
    st = os.stat(bag_path)
    if not is_indexed(bag_path):
        raise ValueError("no_index")
    allow = set(schema.ALLOWLIST)
    dec = Decoder()
    ov = Overview()
    presence: Dict[str, Set[int]] = {}
    t0: Optional[float] = None
    t_last: Optional[float] = None
    fast: Optional[bool] = None
    n_rs = 0
    last_flags = None
    chunk_s = schema.CHUNK_S
    with open(bag_path, "rb") as fh:
        reader = SeekingReader(fh)
        for sch, ch, msg in reader.iter_messages(log_time_order=True):
            topic = ch.topic
            if topic not in allow or sch is None or sch.encoding != "ros2msg":
                continue
            t = msg.log_time / 1e9
            if t0 is None:
                t0 = t
            if t_last is None or t > t_last:
                t_last = t
            presence.setdefault(topic, set()).add(int((t - t0) // chunk_s))
            if topic == ROBOT_STATE:
                if fast is None:
                    dec.ensure_schema(sch)
                    full = _flags_from_obj(dec.decode(msg.data, sch.name))
                    try:
                        fast = robot_state_flags(msg.data) == full
                    except Exception:  # noqa: BLE001 - any parse trouble -> slow path
                        fast = False
                    _log("robot_state flags: %s path" % ("fast CDR-walk" if fast else "full-decode"))
                n_rs += 1
                if fast:
                    flags = robot_state_flags(msg.data)
                    if n_rs % SELF_CHECK_EVERY == 0:
                        dec.ensure_schema(sch)
                        full = _flags_from_obj(dec.decode(msg.data, sch.name))
                        if flags != full:
                            raise RuntimeError("robot_state flag walk disagrees with decode at t=%r" % t)
                else:
                    dec.ensure_schema(sch)
                    flags = _flags_from_obj(dec.decode(msg.data, sch.name))
                if flags != last_flags:
                    last_flags = flags
                    ov.feed(ROBOT_STATE, [t], {n: [f] for n, f in zip(_FLAG_NAMES, flags)})
            elif topic in FULL_DECODE:
                dec.ensure_schema(sch)
                obj = dec.decode(msg.data, sch.name)
                paths, values = dec.flatten(obj, sch.name)
                ov.feed(topic, [t], {p: [v] for p, v in zip(paths, values)})
    if t0 is None or t_last is None:
        raise ValueError("no allow-listed messages")
    for topic, slots in presence.items():
        ov.presence[topic] = sorted(slots)
    out = ov.build(t0, t_last, chunk_s)
    out["source"] = {"size_bytes": st.st_size, "mtime": st.st_mtime}
    return out


def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="python -m replay.overview", description=__doc__.split("\n")[0])
    p.add_argument("--bag", required=True)
    p.add_argument("--out", required=True)
    a = p.parse_args(argv)
    t = time.monotonic()
    try:
        res = compute(a.bag)
    except ValueError as exc:
        _log("refused: %s" % exc)
        return 2
    except Exception as exc:  # noqa: BLE001
        _log("FAILED: %s: %s" % (type(exc).__name__, exc))
        return 1
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    tmp = a.out + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(res, fh)
    os.replace(tmp, a.out)
    _log("done in %.1f s: %d ticks, %d segments" % (
        time.monotonic() - t, len(res["ticks"]), len(res["bands"][0]["segments"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
