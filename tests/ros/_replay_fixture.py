"""Synthetic rosbag2-shaped MCAP recordings for the replay-converter tests.

NOT collected by pytest (leading underscore). ``write_bag`` writes an MCAP that
looks like a rosbag2 Foxy recording — ``ros2msg`` schemas in rosbag2's embedded
format, ``cdr`` channels, a ``metadata.yaml`` beside it — and returns exactly
what it wrote so tests can compare round-trips.

Timeline (seconds from the start, defaults):
  /robot_state        100 Hz; is_homed rises at 5, levelling_complete at 8,
                      has_undervoltage True on [20, 22)
  /orchestrator_state 10 Hz; IDLE, LEVELLING from 4, ACTIVE from 9
  /hand_telemetry     100 Hz
  /mocap_data         50 Hz, 2 markers
  /skills/attempt     3 messages (DiagnosticStatus)
  /cone/catch_event   2 messages
  /bb/calibration_attempt 3 messages (one failure); /bb/calibration_result 1 message
  /bb/axis_estimates  50 Hz Float64MultiArray — NOT allow-listed (skipped)
"""
from __future__ import annotations

import io
import os
import random
from pathlib import Path
from typing import Dict, List, Tuple

from mcap.writer import CompressionType, Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

REPO = Path(__file__).resolve().parents[2]
MSG_DIR = REPO / "ros_ws" / "src" / "jugglebot_interfaces" / "msg"
SEP = "=" * 80

T_START_NS = 1_791_532_600_000_000_000
SKILL_TIMES = (6.05, 12.33, 27.71)
CATCH_TIMES = (13.42, 25.18)
CAL_ATTEMPT_TIMES = (7.5, 15.2, 30.1)   # the middle one is a FAILED sweep
CAL_RESULT_TIME = 7.5                    # the calibration in force (no overview tick)

JB = "jugglebot_interfaces/msg/"


def make_typestore():
    ts = get_typestore(Stores.ROS2_FOXY)
    types = {}
    for f in sorted(MSG_DIR.glob("*.msg")):
        types.update(get_types_from_msg(f.read_text(), JB + f.stem))
    ts.register(types)
    return ts


def _full_name(short: str) -> str:
    pkg, _, name = short.partition("/")
    return short if "/msg/" in short else "%s/msg/%s" % (pkg, name)


def _dep_header_name(short: str) -> str:
    """rosbag2's dependency header form: ``pkg/Type`` (no ``/msg/``) - what real
    bags carry (checked against a 2026-10-10 bag); the browser's rosmsg parser
    resolves references by exactly this spelling."""
    return short.replace("/msg/", "/")


def embedded_schema(ts, typename: str) -> str:
    """rosbag2's embedded ros2msg text: own raw text, then MSG: blocks per dep."""
    gen, _ = ts.generate_msgdef(typename, ros_version=2)
    blocks = gen.split(SEP)

    def own_text(name: str, generated: str) -> str:
        if name.startswith(JB):
            return (MSG_DIR / (name[len(JB):] + ".msg")).read_text()
        return generated.strip() + "\n"

    out = [own_text(typename, blocks[0])]
    for b in blocks[1:]:
        b = b.strip()
        first, _, body = b.partition("\n")
        dep = _full_name(first[5:].strip())
        out.append(SEP + "\nMSG: " + _dep_header_name(dep) + "\n" + own_text(dep, body))
    return "\n".join(out)


def _time(ts, t_ns: int):
    T = ts.types["builtin_interfaces/msg/Time"]
    return T(sec=t_ns // 10**9, nanosec=t_ns % 10**9)


EDGE_TIMES = (10.0, 10.0, 20.0, 20.0, 29.9999999)   # slot-edge ties + a hair before an edge
EDGE_VALUES = ("TIE_A", "TIE_B", "TIE_C", "TIE_D", "EDGE_LO")


def _build_messages(ts, duration_s: float, seed: int, edge_messages: bool = False) -> List[Tuple[int, str, str, object, Dict]]:
    """[(t_ns, topic, typename, msg_obj, plain_dict)] in log-time order."""
    import numpy as np

    rng = random.Random(seed)
    T = ts.types
    Header = T["std_msgs/msg/Header"]
    Quat = T["geometry_msgs/msg/Quaternion"]
    Point = T["geometry_msgs/msg/Point"]
    out = []

    def add(t_s, topic, typename, msg, plain):
        out.append((T_START_NS + int(round(t_s * 1e9)), topic, typename, msg, plain))

    def f32(x):
        return float(np.float32(x))

    n_rs = int(duration_s * 100)
    for k in range(n_rs):
        t = k / 100.0
        t_ns = T_START_NS + int(round(t * 1e9))
        motors = []
        for j in range(7):
            motors.append({
                "active_errors": rng.randrange(0, 4), "disarm_reason": 0,
                "current_state": 8, "procedure_result": 0,
                "trajectory_done": bool(rng.randrange(2)),
                "pos_estimate": f32(rng.uniform(-5, 5)), "vel_estimate": f32(rng.uniform(-1, 1)),
                "iq_setpoint": f32(rng.uniform(-3, 3)), "iq_measured": f32(rng.uniform(-3, 3)),
                "fet_temp": f32(30 + j), "motor_temp": f32(25 + j),
                "bus_voltage": f32(48.0), "bus_current": f32(rng.uniform(0, 2)),
            })
        plain = {
            "timestamp": {"sec": t_ns // 10**9, "nanosec": t_ns % 10**9},
            "motor_states": motors,
            "error": ["e%d" % (k % 3)] if k % 50 == 0 else [],
            "has_fatal_odrive_error": False, "has_fatal_can_error": False,
            "has_undervoltage": 20.0 <= t < 22.0,
            "firmware_validated": True,
            "encoder_search_complete": t >= 5.0,
            "is_homed": t >= 5.0,
            "levelling_complete": t >= 8.0,
            "pose_offset_rad": [f32(0.001 * (k % 7)), f32(-0.002)],
            "pose_offset_quat": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "platform_fw_version": 3, "platform_fw_version_read": True,
        }
        MS = T[JB + "MotorStateSingle"]
        msg = T[JB + "RobotState"](
            timestamp=_time(ts, t_ns),
            motor_states=[MS(**d) for d in motors],
            error=list(plain["error"]),
            has_fatal_odrive_error=False, has_fatal_can_error=False,
            has_undervoltage=plain["has_undervoltage"], firmware_validated=True,
            encoder_search_complete=plain["encoder_search_complete"],
            is_homed=plain["is_homed"], levelling_complete=plain["levelling_complete"],
            pose_offset_rad=np.array(plain["pose_offset_rad"], dtype=np.float32),
            pose_offset_quat=Quat(x=0.0, y=0.0, z=0.0, w=1.0),
            platform_fw_version=3, platform_fw_version_read=True)
        add(t, "/robot_state", JB + "RobotState", msg, plain)

    for k in range(int(duration_s * 10)):
        t = k / 10.0 + 0.005
        val = "IDLE" if t < 4.0 else ("LEVELLING" if t < 9.0 else "ACTIVE")
        add(t, "/orchestrator_state", "std_msgs/msg/String",
            T["std_msgs/msg/String"](data=val), {"data": val})

    for k in range(int(duration_s * 100)):
        t = k / 100.0 + 0.003
        t_ns = T_START_NS + int(round(t * 1e9))
        vals = {n: rng.uniform(-10, 10) for n in
                ("pos_cmd", "vel_ff_cmd", "tor_ff_cmd", "pos_meas", "vel_meas", "iq_meas")}
        held = bool(rng.randrange(2))
        stamp = {"sec": t_ns // 10**9, "nanosec": t_ns % 10**9}
        plain = dict({"timestamp": stamp}, **vals)
        plain.update(ball_held=held, ball_held_raw=held, ball_held_valid=True,
                     ball_held_stamp=stamp)
        msg = T[JB + "HandTelemetryMessage"](
            timestamp=_time(ts, t_ns), ball_held=held, ball_held_raw=held,
            ball_held_valid=True, ball_held_stamp=_time(ts, t_ns), **vals)
        add(t, "/hand_telemetry", JB + "HandTelemetryMessage", msg, plain)

    MSingle = T[JB + "MocapDataSingle"]
    for k in range(int(duration_s * 50)):
        t = k / 50.0 + 0.007
        t_ns = T_START_NS + int(round(t * 1e9))
        markers = []
        for j in range(2):
            markers.append({"position": {"x": rng.uniform(-500, 500), "y": rng.uniform(-500, 500),
                                         "z": rng.uniform(0, 2000)},
                            "residual": f32(rng.uniform(0, 2)), "label": "m%d" % j})
        msg = T[JB + "MocapDataMulti"](
            markers=[MSingle(position=Point(**d["position"]), residual=d["residual"],
                             label=d["label"]) for d in markers],
            aligned=True, stamp=_time(ts, t_ns))
        add(t, "/mocap_data", JB + "MocapDataMulti", msg,
            {"markers": markers, "aligned": True,
             "stamp": {"sec": t_ns // 10**9, "nanosec": t_ns % 10**9}})

    DS = T["diagnostic_msgs/msg/DiagnosticStatus"]
    KV = T["diagnostic_msgs/msg/KeyValue"]
    for n, t in enumerate(t for t in SKILL_TIMES if t < duration_s):
        message = "attempt %d CAUGHT" % n if n != 1 else ""
        msg = DS(level=0, name="skill_%d" % n, message=message, hardware_id="",
                 values=[KV(key="k", value=str(n))])
        add(t, "/skills/attempt", "diagnostic_msgs/msg/DiagnosticStatus", msg,
            {"level": 0, "name": "skill_%d" % n, "message": message, "hardware_id": "",
             "values": [{"key": "k", "value": str(n)}]})

    for n, t in enumerate(t for t in CATCH_TIMES if t < duration_s):
        t_ns = T_START_NS + int(round(t * 1e9))
        msg = T[JB + "CatchEvent"](
            header=Header(stamp=_time(ts, t_ns), frame_id="cone"),
            catch_time=_time(ts, t_ns), sequence=n, time_synced=True,
            retrigger_suppressed=False)
        add(t, "/cone/catch_event", JB + "CatchEvent", msg, {"sequence": n})

    CalT = JB + "BallButlerCalibrationResult"
    Point3 = T["geometry_msgs/msg/Point"]
    cal_msgs = [("/bb/calibration_attempt", t, n != 1) for n, t in enumerate(CAL_ATTEMPT_TIMES)]
    cal_msgs.append(("/bb/calibration_result", CAL_RESULT_TIME, True))
    for topic, t, ok in cal_msgs:
        if t >= duration_s:
            continue
        msg = T[CalT](position_mm=Point3(x=1.0, y=2.0, z=3.0), yaw_offset_rad=0.1,
                      yaw_offset_std_deg=0.2, axis_tilt_deg=0.3, success=ok,
                      message="" if ok else "sweep refused")
        add(t, topic, CalT, msg, {"success": ok})

    FMA = T["std_msgs/msg/Float64MultiArray"]
    Layout = T["std_msgs/msg/MultiArrayLayout"]
    for k in range(int(duration_s * 50)):
        t = k / 50.0 + 0.011
        msg = FMA(layout=Layout(dim=[], data_offset=0),
                  data=np.array([rng.uniform(-1, 1) for _ in range(3)], dtype=np.float64))
        add(t, "/bb/axis_estimates", "std_msgs/msg/Float64MultiArray", msg, {})

    if edge_messages:
        # Equal-log-time pairs exactly on slot edges (order = insertion order)
        # and one message a hair before an edge: the slot-assignment and tie cases.
        for t, val in zip(EDGE_TIMES, EDGE_VALUES):
            if t < duration_s:
                add(t, "/orchestrator_state", "std_msgs/msg/String",
                    T["std_msgs/msg/String"](data=val), {"data": val})

    out.sort(key=lambda r: r[0])
    return out


def write_bag(path, duration_s: float = 35.0, *, seed: int = 0, unindexed: bool = False,
              chunk_size: int = 64 * 1024, late_message: bool = False,
              compression: str = "none", edge_messages: bool = False) -> Dict:
    """Write a synthetic rosbag2-style MCAP at ``path``; return what was written.

    ``compression``: "none" (rosbag2's default; the browser reader refuses
    compressed chunks) or "zstd"/"lz4" for the refusal test. ``edge_messages``:
    opt-in equal-timestamp pairs on slot edges (the MCAP oracle test).
    ``unindexed``: the file stops before the footer/summary (a killed recording)
    and one /orchestrator_state message is written 0.3 s late in file order.
    ``late_message``: one /orchestrator_state message is written > 20 s late in
    file order (implies ``unindexed`` — an indexed reader re-sorts by log time),
    so the converter must count it in ``dropped_late``.

    Returns ``{"t0_ns", "t1_ns", "topics": {topic: [(t_ns, plain), ...]},
    "reordered": (t_ns, topic) | None, "late": (t_ns, topic) | None}``; ``topics``
    lists only messages that land in the file (the truncated tail of an
    unindexed bag is excluded).
    """
    path = Path(path)
    if late_message:
        unindexed = True
    ts = make_typestore()
    msgs = _build_messages(ts, duration_s, seed, edge_messages)

    order = list(range(len(msgs)))
    moved = None
    if unindexed:
        orch = [k for k, r in enumerate(msgs) if r[1] == "/orchestrator_state"]
        if late_message:
            victim = next(k for k in orch if msgs[k][0] - msgs[0][0] >= 1.0e9)
            delay_ns = 21.0e9
        else:
            victim = next(k for k in orch if msgs[k][0] - msgs[0][0] >= 15.0e9)
            delay_ns = 0.3e9
        target_t = msgs[victim][0] + delay_ns
        order.remove(victim)
        insert_at = next((p for p, k in enumerate(order) if msgs[k][0] > target_t), len(order))
        order.insert(insert_at, victim)
        moved = victim

    buf = io.BytesIO()
    w = Writer(buf, chunk_size=chunk_size, compression=CompressionType[compression.upper()])
    w.start(profile="ros2", library="jugglebot-test-fixture")
    schema_ids, channel_ids = {}, {}
    written_flags = []
    seq = {}
    prefix_len = None
    for idx in order:
        t_ns, topic, typename, msg, _plain = msgs[idx]
        if typename not in schema_ids:
            schema_ids[typename] = w.register_schema(
                name=typename, encoding="ros2msg",
                data=embedded_schema(ts, typename).encode("utf-8"))
        if topic not in channel_ids:
            channel_ids[topic] = w.register_channel(
                topic=topic, message_encoding="cdr", schema_id=schema_ids[typename],
                metadata={"offered_qos_profiles": ""})
        seq[topic] = seq.get(topic, 0) + 1
        w.add_message(channel_ids[topic], log_time=t_ns,
                      data=bytes(ts.serialize_cdr(msg, typename)),
                      publish_time=t_ns, sequence=seq[topic])
        written_flags.append((idx, buf.tell()))
    if unindexed:
        prefix_len = buf.tell()
    w.finish()
    data = buf.getvalue()
    if unindexed:
        data = data[:prefix_len]
    path.write_bytes(data)

    # Which messages made it into the file: for unindexed, only those whose
    # chunk was flushed before the cut (tell() advanced past them).
    if unindexed:
        flushed = set()
        pending = []
        last_tell = 0
        for idx, tell in written_flags:
            pending.append(idx)
            if tell != last_tell:  # a chunk was flushed containing all pending
                flushed.update(pending)
                pending = []
                last_tell = tell
        in_file = flushed
    else:
        in_file = set(range(len(msgs)))

    topics = {}
    for k in sorted(in_file, key=lambda k: msgs[k][0]):
        t_ns, topic, _tn, _m, plain = msgs[k]
        topics.setdefault(topic, []).append((t_ns, plain))
    all_t = [msgs[k][0] for k in in_file]
    info = {
        "t0_ns": min(all_t), "t1_ns": max(all_t), "topics": topics,
        "reordered": (msgs[moved][0], msgs[moved][1]) if (moved is not None and not late_message) else None,
        "late": (msgs[moved][0], msgs[moved][1]) if late_message else None,
        "n_messages": len(in_file),
        "types": {msgs[k][1]: msgs[k][2] for k in in_file},
    }
    _write_metadata(path, info)
    return info


def _write_metadata(path: Path, info: Dict) -> None:
    text = (
        "rosbag2_bagfile_information:\n"
        "  version: 4\n"
        "  storage_identifier: mcap\n"
        "  relative_file_paths:\n"
        "    - %s\n"
        "  duration:\n"
        "    nanoseconds: %d\n"
        "  starting_time:\n"
        "    nanoseconds_since_epoch: %d\n"
        "  message_count: %d\n"
        "  topics_with_message_count:\n"
    ) % (path.name, info["t1_ns"] - info["t0_ns"], info["t0_ns"], info["n_messages"])
    for topic in sorted(info["topics"]):
        text += (
            "    - topic_metadata:\n"
            "        name: %s\n"
            "        type: %s\n"
            "        serialization_format: cdr\n"
            "        offered_qos_profiles: ''\n"
            "      message_count: %d\n"
        ) % (topic, info["types"][topic], len(info["topics"][topic]))
    (path.parent / "metadata.yaml").write_text(text)
