"""The replay cache contract — the one place the file layout, chunk record,
manifest and overview shapes are defined. STDLIB ONLY (imported by the GUI
server under the system interpreter).

Decided in wayfinder ticket 05 (2026-10-10); see
``plans/active/gui-rosbag-replay.md`` § Architecture.

Cache layout, ``<cache_root>/<recording_id>/``::

    manifest.json           status + range + topic census (written by the worker)
    overview.json           timeline bands / ticks / presence (written at completion)
    chunk-00000.msgpack.gz  chunk i covers [t0 + i*CHUNK_S, t0 + (i+1)*CHUNK_S)
    .opened                 touch file; its mtime is the LRU key (server-owned)

Chunks are sealed strictly in order, so ``manifest["chunks_done"] == k`` means
chunk files 0..k-1 exist and are complete. Every index in 0..N-1 has a file,
empty stretches included, so the sequence is contiguous.

Chunk record (msgpack map, stored gzip-compressed, served with
``Content-Encoding: gzip``)::

    {"format": 1, "i": int, "t0": float, "t1": float,
     "topics": {"/robot_state": {"type": "jugglebot_interfaces/msg/RobotState",
                                 "n": int,
                                 "t": [float, ...],          # MCAP log_time, seconds, ascending
                                 "cols": {"<path>": [..]}},  # one list per flattened field, len n
                ...}}                                        # only topics with n >= 1

Flattening (``cols`` keys) is the normative rule the browser unflattens by:

- a primitive field -> its own column (int / float / bool / str);
- a nested message -> recurse with dotted paths (``timestamp.sec``,
  ``pose_offset_quat.w``);
- an array of primitives, fixed or variable length -> ONE column whose entries
  are lists (numpy arrays via ``.tolist()``);
- an array of messages -> ONE column whose entries are lists of plain nested
  dicts (never flattened further);
- every message of a topic has the same field set, so all columns of a topic
  have length ``n``;
- field names starting with ``__`` (rosbags' dataclass ``__msgtype__``) are
  not fields and are skipped. IDL constants (e.g. DiagnosticStatus
  OK/WARN/ERROR/STALE) are not fields and are never columns.

``t0`` is the log time of the first ALLOW-LISTED message (served data starts
there); ``metadata.yaml``'s duration may exceed the cache span by the lead of
non-allow-listed topics. Indexed bags are read in log-time order; unindexed
bags in file order, so only there can a message arrive more than one chunk
late (``dropped_late``) or precede ``t0`` (also counted in ``dropped_late``).

Manifest::

    {"format": 1, "recording": "<id>",
     "source": {"path": str, "size_bytes": int, "mtime": float, "indexed": bool},
     "status": "converting" | "complete" | "failed", "error": str | None,
     "chunk_s": 10.0,
     "t0": float | None,          # first allow-listed message log_time (s); None until seen
     "t1": float | None,          # last message log_time; None until complete
     "chunks_done": int,          # chunks 0..chunks_done-1 are sealed and readable
     "chunks_total": int | None,  # from the summary for indexed bags, None for unindexed, exact at completion
     "topics": {"/x": {"type": str, "count": int}},          # allow-listed topics present (counts final at completion)
     "skipped_topics": {"/x": {"type": str, "count": int}},  # recorded but not allow-listed
     "dropped_late": int,         # messages that arrived > 1 chunk late and were not written
     "chunks": [{"i": int, "t0": float, "t1": float, "n": int, "topics": {"/x": int}}],  # [] until complete
     "started_at": iso8601, "completed_at": iso8601 | None, "converter": str}

    A ``failed`` manifest keeps ``chunks_done`` (its sealed chunk files are valid) and ``chunks: []``.

Overview (written once, at completion)::

    {"format": 1, "t0": float, "t1": float,
     "bands": [{"topic": "/orchestrator_state", "segments": [[t_start, t_end, value_str], ...]}],
     "ticks": [{"t": float, "kind": str, "label": str}, ...],
     "presence": {"/x": [[t_start, t_end], ...]}}   # merged runs of chunks where the topic has n >= 1

Tick kinds (``OVERVIEW_TICK_KINDS``) are the vocabulary ticket 03 chooses from.
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

FORMAT_VERSION = 2
# 2 (2026-10-10): IDL constants dropped from columns; the server discards caches with an older format on open.
CHUNK_S = 10.0

MANIFEST = "manifest.json"
OVERVIEW = "overview.json"
OPENED_MARK = ".opened"
CHUNK_FMT = "chunk-{:05d}.msgpack.gz"

STATUS_CONVERTING = "converting"
STATUS_COMPLETE = "complete"
STATUS_FAILED = "failed"

# The GUI's live subscribe set — every ros.subscribe(...) in ros_ws/gui/js/
# (main.js subscribeAll()). Every topic the browser consumes live is converted.
# Pinned by tests/ros/test_replay_allowlist.py against the JS source.
SUBSCRIBED: Tuple[str, ...] = (
    "/bb/calibration_attempt",   # every sweep's outcome (keep-last-good, 2026-10-10); the result topic below carries only the calibration in force
    "/bb/calibration_result",
    "/bb/heartbeat",
    "/clock_diag",
    "/cone/heartbeat",
    "/cone/timing_result",
    "/control_mode_topic",
    "/hand_telemetry",
    "/leg_setpoint_echo",
    "/link_status",
    "/mocap_data",
    "/motion/diagnostics",
    "/orchestrator_state",
    "/profile",
    "/rigid_body_poses",
    "/robot_state",
    "/skills/attempt",
    "/udp_diag",
)

# Topics the replay plan adds to the GUI (map decisions 18 and 20: balls in the
# 3D scene, cone catch ticks on the overview). The only extras the allowlist
# test accepts beyond the live subscribe set.
PLANNED: Tuple[str, ...] = (
    "/balls",
    "/cone/catch_event",
)

ALLOWLIST: Tuple[str, ...] = tuple(sorted(set(SUBSCRIBED) | set(PLANNED)))

# Overview sources. The band comes from std_msgs/String .data on
# /orchestrator_state; ticks from flag transitions and event topics.
OVERVIEW_BAND_TOPIC = "/orchestrator_state"
OVERVIEW_TICK_KINDS: Tuple[str, ...] = (
    "fault",            # /robot_state: any of has_fatal_odrive_error / has_fatal_can_error / has_undervoltage rises
    "fault_cleared",    # ... all three fall
    "homed",            # /robot_state is_homed rises
    "levelled",         # /robot_state levelling_complete rises
    "catch_event",      # /cone/catch_event message
    "skill_attempt",    # /skills/attempt message (DiagnosticStatus; label = .message or .name)
    "bb_calibration",   # /bb/calibration_result message
)


def chunk_name(i: int) -> str:
    return CHUNK_FMT.format(i)


def chunk_index(t: float, t0: float) -> int:
    """Index of the chunk containing wall-clock time ``t`` (seconds)."""
    return int((t - t0) // CHUNK_S)


def chunk_bounds(i: int, t0: float) -> Tuple[float, float]:
    return (t0 + i * CHUNK_S, t0 + (i + 1) * CHUNK_S)


def new_manifest(recording_id: str, source_path: str, size_bytes: int,
                 mtime: float, indexed: bool, started_at: str,
                 converter: str, chunks_total: Optional[int]) -> Dict:
    """The manifest the worker writes before the first chunk."""
    return {
        "format": FORMAT_VERSION,
        "recording": recording_id,
        "source": {"path": source_path, "size_bytes": size_bytes,
                   "mtime": mtime, "indexed": indexed},
        "status": STATUS_CONVERTING,
        "error": None,
        "chunk_s": CHUNK_S,
        "t0": None,
        "t1": None,
        "chunks_done": 0,
        "chunks_total": chunks_total,
        "topics": {},
        "skipped_topics": {},
        "dropped_late": 0,
        "chunks": [],
        "started_at": started_at,
        "completed_at": None,
        "converter": converter,
    }
