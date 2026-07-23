#!/usr/bin/env python3
"""
generate_udp_protocol.py -- Single-source generator for the Jetson <-> can-bridge
Teensy UDP wire protocol.

This is the **one** place the UDP protocol is defined. From the SPEC below it
emits three artifacts that are guaranteed byte-for-byte consistent:

    config/generated/udp_protocol.h    (C++ header — packed structs + framing)
    config/generated/udp_protocol.py   (Python module — struct fmts + framing)
    docs/teensy-udp-protocol.md        (human-readable spec)

and delivers copies of the C++ header and Python module to their consumers:

    ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
    tools/probes/teensy_link_profiling/jetson/udp_protocol.py

Run:  python config/generate_udp_protocol.py
Test: tests/firmware/test_udp_protocol_xlang.py  (cross-language consistency)

Design rationale (see docs/teensy-udp-protocol.md and the firmware-WIP handoff):
  * Fixed-length, typed frames — NOT COBS. UDP datagrams already carry message
    boundaries, so the stream-framing COBS solves is redundant here, and a
    fixed layout per message type is zero-allocation and deterministic-timing
    (mandatory for the hard-real-time firmware). COBS would add a byte-stuffing
    scan for no benefit on a datagram transport.
  * CRC-16/CCITT-FALSE over the whole frame except the trailing CRC field.
  * 8-byte header → payload starts 8-byte aligned (clean float access).
  * Little-endian throughout (Cortex-M7 + x86 both native LE; no byte-swaps).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
OUTPUT_DIR = SCRIPT_DIR / "generated"
FIRMWARE_DIR = REPO_ROOT / "ros_ws" / "src" / "jugglebot" / "Teensy_code_canbridge"
JETSON_TOOLS_DIR = REPO_ROOT / "tools" / "probes" / "teensy_link_profiling" / "jetson"
DOCS_PATH = REPO_ROOT / "docs" / "teensy-udp-protocol.md"

# ───────────────────────────────────────────────────────────────────────────
# Type system: maps a spec type → (C++ type, Python struct char, byte width)
# ───────────────────────────────────────────────────────────────────────────

TYPES = {
    "u8":  ("uint8_t",  "B", 1),
    "i8":  ("int8_t",   "b", 1),
    "u16": ("uint16_t", "H", 2),
    "i16": ("int16_t",  "h", 2),
    "u32": ("uint32_t", "I", 4),
    "i32": ("int32_t",  "i", 4),
    "u64": ("uint64_t", "Q", 8),
    "f32": ("float",    "f", 4),
    "f64": ("double",   "d", 8),
}


@dataclass
class Field:
    name: str
    type: str
    count: int = 1
    comment: str = ""

    @property
    def width(self) -> int:
        return TYPES[self.type][2] * self.count

    @property
    def cpp_decl(self) -> str:
        ctype = TYPES[self.type][0]
        if self.count == 1:
            return f"{ctype} {self.name};"
        return f"{ctype} {self.name}[{self.count}];"

    @property
    def struct_fmt(self) -> str:
        return TYPES[self.type][1] * self.count


@dataclass
class Message:
    name: str            # CamelCase struct name (without "Payload" suffix)
    msg_type: str        # MsgType enum member name
    direction: str       # "J2T", "T2J", or "BOTH"
    port: str            # "STREAM" or "RPC"
    fields: list
    summary: str = ""

    @property
    def payload_size(self) -> int:
        return sum(f.width for f in self.fields)

    @property
    def struct_fmt(self) -> str:
        return "<" + "".join(f.struct_fmt for f in self.fields)


# ───────────────────────────────────────────────────────────────────────────
# CONSTANTS
# ───────────────────────────────────────────────────────────────────────────

CONSTANTS = [
    ("PROTOCOL_VERSION", 4,      "u8",  "Bumped on any incompatible wire change"),
    ("MAGIC",            0x4A42, "u16", '"JB" little-endian preamble (bytes 0x42 0x4A)'),
    ("HEADER_SIZE",      8,      "u16", "Bytes before payload"),
    ("CRC_SIZE",         2,      "u16", "Trailing CRC-16 bytes"),
    ("MAX_PAYLOAD",      1024,   "u16", "Largest payload we will encode/accept"),
    ("MAX_FRAME",        1034,   "u16", "HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE"),
    ("PORT_STREAM",      5005,   "u16", "High-rate push: setpoint dn, telem/diag/profile/heartbeat up"),
    ("PORT_RPC",         5006,   "u16", "Request/response with retry-on-timeout"),
    ("TEENSY_IP_0",      192,    "u8",  "Static Teensy IP octet 0 (192.168.42.2)"),
    ("TEENSY_IP_1",      168,    "u8",  ""),
    ("TEENSY_IP_2",      42,     "u8",  ""),
    ("TEENSY_IP_3",      2,      "u8",  ""),
    ("JETSON_IP_3",      1,      "u8",  "Static Jetson teensy-link IP octet 3 (192.168.42.1)"),
    ("NUM_LEGS",         6,      "u8",  "Leg ODrive axes 0..5"),
    ("NUM_AXES",         7,      "u8",  "Legs + hand (cache + telemetry breadth)"),
    ("PROFILE_NUM_TASKS", 9,     "u8",  "Task slots reported in the profiling frame"),
    # Link/heartbeat health timing (mirrors motor_guard / can_node thresholds)
    ("HEARTBEAT_HZ",     10,     "u8",  "Both-direction liveness rate"),
    ("LINK_LOST_MISSES", 5,      "u8",  "Missed heartbeats before declaring link lost"),
    # HeartbeatT2J.flags bit offset of the per-leg torque_ff ingest-clamp mask
    # (HeartbeatT2JFlags::TORQUE_CLAMP_MASK, bits 8-13). Packed into the existing
    # u32 flags field ON PURPOSE: no payload-size change, no PROTOCOL_VERSION bump —
    # an old parser just ignores the new bits, a new parser reads 0 from an old
    # firmware (the clamp is dormant until the can-bridge is reflashed).
    ("HEARTBEAT_TORQUE_CLAMP_SHIFT", 8, "u8",
     "Bit offset of TORQUE_CLAMP_MASK inside HeartbeatT2J.flags (bits 8-13)"),
]

# ───────────────────────────────────────────────────────────────────────────
# ENUMS  (name → list of (member, value, comment))
# ───────────────────────────────────────────────────────────────────────────

# C++ constant width per enum — must match the width of the wire field that
# carries it (RpcMethod/RpcStatus ride a u16 field; the rest ride u8 fields).
ENUM_WIDTH = {
    "MsgType": "u8", "RpcMethod": "u16", "RpcStatus": "u16",
    "LinkState": "u8", "BusHealth": "u8", "FaultState": "u8", "GuardMode": "u8",
    "HeartbeatT2JFlags": "u32",
}

ENUMS = {
    "MsgType": [
        # Downlink (Jetson → Teensy): high bit 0
        ("SETPOINT",       0x01, "40 Hz MPC setpoint waypoints (STREAM, J→T)"),
        ("HEARTBEAT_J2T",  0x02, "Jetson liveness (STREAM, J→T)"),
        ("RPC_REQUEST",    0x10, "RPC request (RPC port, J→T)"),
        # Uplink (Teensy → Jetson): high bit 1
        ("TELEMETRY",      0x81, "100 Hz motor state (STREAM, T→J)"),
        ("DIAGNOSTIC",     0x82, "On-change per-axis diagnostics (STREAM, T→J)"),
        ("HEARTBEAT_T2J",  0x83, "Teensy liveness + link/bus health (STREAM, T→J)"),
        ("PROFILE",        0x84, "1 Hz profiling/instrumentation (STREAM, T→J)"),
        ("CONE_FRAME",     0x85, "Catching-cone CAN2 frame relay (STREAM, T→J)"),
        ("BB_AXIS_ESTIMATES", 0x86, "Ball Butler pitch/hand ODrive pos+vel estimates (STREAM, T→J)"),
        ("CMD_RESULT",     0x87, "Ball Butler command-outcome CAN1 frame relay (STREAM, T→J)"),
        ("LEG_CMD",        0x88, "Teensy commanded leg interp output @100Hz (STREAM, T→J) — float32 interp residual check"),
        # Reserved id block — fill the 0x89–0x8F
        # telemetry gap below RPC_RESPONSE 0x90. Owners noted; consumers land as each is implemented.
        ("PLATFORM_FRAME", 0x89, "Verbatim Platform-Teensy relay-reply uplink (STREAM, T→J)"),
        ("HAND_CMD_ECHO",  0x8A, "Hand command-echo telemetry (STREAM, T→J)"),
        ("RPC_RESPONSE",   0x90, "RPC response (RPC port, T→J)"),
    ],
    "RpcMethod": [
        ("NOP",                0x0000, "No-op (link test)"),
        ("TIME_OF_DAY_QUERY",  0x0001, "Wall-clock bootstrap + drift resync (time-sync master)"),
        ("SET_AXIS_STATE",     0x0010, "ODrive set_requested_state"),
        ("SET_CONTROLLER_MODE", 0x0011, "ODrive set_controller_mode"),
        ("SET_VEL_CURR_LIMITS", 0x0012, "ODrive set_vel_curr_limits"),
        ("SET_POS_GAIN",       0x0013, "ODrive set_pos_gain"),
        ("SET_VEL_GAINS",      0x0014, "ODrive set_vel_gains"),
        ("CLEAR_ERRORS",       0x0015, "ODrive clear_errors"),
        ("REBOOT_ODRIVES",     0x0016, "ODrive reboot"),
        ("SET_ABSOLUTE_POSITION", 0x0017, "ODrive set_absolute_position (post-homing)"),
        ("ENCODER_SEARCH",     0x0020, "Run encoder index search (firmware stub — returns ERR_NOT_IMPL)"),
        ("HOME",               0x0021, "Run homing (firmware stub — returns ERR_NOT_IMPL)"),
        ("ACTIVATE",           0x0022, "Run TRAP_TRAJ move to the active pose"),
        ("DEACTIVATE",         0x0023, "Run TRAP_TRAJ move to the STOW pose, then IDLE"),
        ("SDO_READ",           0x0030, "Arbitrary parameter read"),
        ("SDO_WRITE",          0x0031, "Arbitrary parameter write"),
        ("BB_THROW",           0x0040, "Ball Butler: send THROW_CMD on CAN1 (typed, validated)"),
        ("BB_RELOAD",          0x0041, "Ball Butler: send RELOAD_CMD on CAN1 (no payload)"),
        ("BB_RESET",           0x0042, "Ball Butler: send RESET_CMD on CAN1 (no payload)"),
        ("BB_CALIBRATE_LOC",   0x0043, "Ball Butler: send CALIBRATE_LOC_CMD on CAN1 (no payload)"),
        # Reserved id block — the relay /
        # version / hand-traj seams that later work owns. Allocated in ONE pass so
        # parallel sessions cannot mint colliding ids. rpc.cpp dispatches these to
        # ERR_NOT_IMPL stubs until each is implemented.
        ("GET_AXIS_VERSIONS",  0x0050, "Pull cached raw Get_Version bytes + received bitmask"),
        ("TILT_READ",          0x0051, "Relay: read Platform-Teensy inclinometer tilt"),
        ("STATE_READ",         0x0052, "Relay: read Platform-Teensy RobotState (is_homed/level/pose)"),
        ("STATE_WRITE",        0x0053, "Relay: write Platform-Teensy RobotState (read-modify-write via cache)"),
        ("HAND_TRAJ_CMD",      0x0054, "Hand traj + smooth-move (byte-0 discriminator → 0x6D0)"),
    ],
    "RpcStatus": [
        ("OK",            0x0000, "Success"),
        ("ERR_UNKNOWN_METHOD", 0x0001, "Method id not recognised"),
        ("ERR_BAD_ARGS",  0x0002, "Argument decode failed / wrong length"),
        ("ERR_BUS_DOWN",  0x0003, "Refused: CAN bus is down (never command a dead bus)"),
        ("ERR_TIMEOUT",   0x0004, "Downstream CAN op timed out"),
        ("ERR_REJECTED",  0x0005, "Refused by a safety gate"),
        ("ERR_NOT_IMPL",  0x0006, "Method not implemented in this firmware revision"),
    ],
    "LinkState": [
        ("INIT",     0, "Ethernet up, no Jetson heartbeat yet"),
        ("UP",       1, "Healthy bidirectional link"),
        ("DEGRADED", 2, "Some missed heartbeats, not yet lost"),
        ("LOST",     3, "Link lost → legs to safe behaviour"),
    ],
    "BusHealth": [
        ("UNKNOWN", 0, "No frames seen yet"),
        ("OK",      1, "Frames flowing, no error frames"),
        ("WARN",    2, "Error counters rising"),
        ("BUS_OFF", 3, "Controller bus-off"),
    ],
    # Mirrors motor_guard GuardMode + can_node fault concepts on the Teensy side.
    "FaultState": [
        ("NONE",            0, "Nominal"),
        ("MPC_STALE",       1, "MPC setpoint older than staleness threshold → E-STOP"),
        ("LINK_LOST",       2, "Jetson link lost → safe behaviour on legs"),
        ("MOTOR_OVERSPEED", 3, "A leg exceeded the overspeed limit"),
        ("MAX_DEVIATION",   4, "Commanded pos diverged too far from encoder"),
        ("ODRIVE_FATAL",    5, "Active ODrive error / disarm-while-closed-loop"),
        ("CAN_BUS_DOWN",    6, "CAN3 (leg bus) down — hold, deferred stow armed"),
        ("MOTOR_FB_STALE",  7, "Leg encoder feedback stale → suppress output (recoverable)"),
    ],
    # Mirrors motor_guard GuardMode.
    "GuardMode": [
        ("DISABLED", 0, ""),
        ("ENABLED",  1, ""),
        ("ESTOP",    2, ""),
    ],
    # HeartbeatT2J.flags bitset (bits 0-3 single-bit flags + bits 8-13 the per-leg
    # torque-clamp mask) — generated single source for the bits the firmware
    # producer (Teensy_code_canbridge.ino) sets and the Jetson bridge
    # reads, replacing the prose that previously lived only in the field comment.
    # No new cold-start bit is added: is_homed/levelling/pose ride the relay
    # STATE_READ (a deliberate design decision), not a
    # heartbeat flag.
    "HeartbeatT2JFlags": [
        ("TIME_SYNCED",               0x1, "bit0: Teensy clock synced to the Jetson anchor"),
        ("STOW_PENDING_ON_RECONNECT", 0x2, "bit1: deferred-stow latch armed (awaiting confirmed CAN3 reconnect)"),
        ("ALL_AXIS_HEARTBEATS_OK",    0x4, "bit2: every present axis heartbeat is fresh"),
        ("MPC_ACTIVE",                0x8, "bit3: firmware-side mpc_active (lets a setpoint source verify its arm took)"),
        # Per-leg torque_ff ingest-clamp mask, packed into free bits of the same u32
        # (bits 4-7 stay reserved for future single-bit flags; the mask starts at
        # bit 8 = HEARTBEAT_TORQUE_CLAMP_SHIFT so it stays byte-aligned/readable).
        ("TORQUE_CLAMP_MASK",      0x3F00, "bits 8-13: bit (8+i) set = leg i's |torque_ff| was clamped to "
                                           "TORQUE_FF_FIRMWARE_CLAMP_WIRE_NM at UDP ingest on the last ACCEPTED "
                                           "setpoint frame (mirrors lead_clamp_mask; leg_interp.cpp interp_on_setpoint)"),
    ],
}

# ───────────────────────────────────────────────────────────────────────────
# MESSAGES  (payload layouts)
# ───────────────────────────────────────────────────────────────────────────

MESSAGES = [
    Message(
        "Setpoint", "SETPOINT", "J2T", "STREAM",
        summary=(
            "40 Hz MPC setpoint waypoints. Carries motor-rev-space quantities "
            "(Jugglebot convention: positive = leg extension) exactly as "
            "motor_guard's interpolator consumes them. The Jetson bridge does "
            "the same mm→rev / pose conversions motor_guard does today; the "
            "Teensy interpolator works purely in rev-space. `u1`/`u2` presence "
            "is signalled by the flags bits (NOT NaN sentinels)."),
        fields=[
            Field("u0",        "f32", 6, "Current motor positions (rev)"),
            Field("u1",        "f32", 6, "Next waypoint (rev); valid iff flags bit0"),
            Field("u2",        "f32", 6, "Next-next waypoint (rev); valid iff flags bit1 — C1 continuity"),
            Field("v0",        "f32", 6, "Forward-looking velocity from MPC (rev/s)"),
            Field("accel",     "f32", 6, "Acceleration for Taylor extrapolation (rev/s^2)"),
            Field("torque_ff", "f32", 6, "Gravity+inertia feedforward (Nm)"),
            Field("flags",     "u32", 1, "bit0: u1_present, bit1: u2_present"),
            Field("t_origin_us", "u64", 1, "Jetson-side wall-clock timestamp (us)"),
        ],
    ),
    Message(
        "HeartbeatJ2T", "HEARTBEAT_J2T", "J2T", "STREAM",
        summary="Jetson → Teensy liveness, ~10 Hz.",
        fields=[
            Field("t_jetson_us", "u64", 1, "Jetson wall-clock (us)"),
            Field("flags",       "u32", 1, "bit0: mpc_active (guard ENABLED)"),
        ],
    ),
    Message(
        "Telemetry", "TELEMETRY", "T2J", "STREAM",
        summary=(
            "100 Hz motor state for all axes (legs + hand). pos/vel are in the "
            "Jugglebot convention (positive = extension), already sign-corrected "
            "from the ODrive convention."),
        fields=[
            Field("t_teensy_us", "u64", 1, "Teensy wall-clock at sample (us)"),
            Field("pos_rev", "f32", 7, "Per-axis position (rev), axes 0..5 legs, 6 hand"),
            Field("vel_rps", "f32", 7, "Per-axis velocity (rev/s)"),
        ],
    ),
    Message(
        "Diagnostic", "DIAGNOSTIC", "T2J", "STREAM",
        summary=(
            "Per-axis diagnostics, published on-change (delta over threshold) OR "
            "on a 1 Hz heartbeat. One axis per frame."),
        fields=[
            Field("axis_id",       "u8",  1, "0..8 (0..5 legs, 6 hand — CAN3; 7 bb_pitch, 8 bb_hand — CAN1)"),
            Field("axis_state",    "u8",  1, "ODrive current_state"),
            Field("ctrl_mode",     "u8",  1, "ODrive controller mode"),
            Field("input_mode",    "u8",  1, "ODrive input mode"),
            Field("flags",         "u8",  1, "bit0: heartbeat_stale, bit1: heartbeat_seen (ever, this boot)"),
            Field("homing_result", "u8",  1, "HomingResult for this Jugglebot axis (0 none/1 running/2 ok/3 failed); 0 for non-leg axes"),
            Field("pad",           "u8",  2, "Alignment pad (zero)"),
            Field("active_errors", "u32", 1, "ODrive active_errors bitmask"),
            Field("disarm_reason", "u32", 1, "ODrive disarm_reason bitmask"),
            Field("iq_setpoint",   "f32", 1, "A"),
            Field("iq_measured",   "f32", 1, "A"),
            Field("temp_fet",      "f32", 1, "degC"),
            Field("temp_motor",    "f32", 1, "degC"),
            Field("bus_voltage",   "f32", 1, "V"),
            Field("bus_current",   "f32", 1, "A (DC bus, from ODrive Get_Bus_Voltage_Current)"),
        ],
    ),
    Message(
        "HeartbeatT2J", "HEARTBEAT_T2J", "T2J", "STREAM",
        summary="Teensy → Jetson liveness + link/bus health, ~10 Hz.",
        fields=[
            Field("t_teensy_us", "u64", 1, "Teensy wall-clock (us)"),
            Field("link_state",  "u8",  1, "LinkState enum"),
            Field("bus1_health", "u8",  1, "wire slot 1 = CAN3 (Jugglebot core: legs+hand) BusHealth enum"),
            Field("bus2_health", "u8",  1, "wire slot 2 = CAN1 (Ball Butler) BusHealth enum (cone/CAN2 not yet on uplink)"),
            Field("fault_state", "u8",  1, "FaultState enum"),
            Field("flags",       "u32", 1, "HeartbeatT2JFlags bitset: bits 0-3 TIME_SYNCED|STOW_PENDING_ON_RECONNECT|ALL_AXIS_HEARTBEATS_OK|MPC_ACTIVE; bits 8-13 TORQUE_CLAMP_MASK (per-leg torque_ff ingest clamp, see HEARTBEAT_TORQUE_CLAMP_SHIFT)"),
            Field("uptime_ms",   "u32", 1, "ms since boot"),
            # Ball Butler heartbeat snapshot (CAN1 0x7D1 decoded by the
            # can-bridge into bb_state and forwarded here at heartbeat rate).
            # Replaces the legacy can_node bb/heartbeat publisher; the bridge
            # node reassembles a BallButlerHeartbeat ROS msg from these fields.
            Field("bb_state",      "u8",  1, "BallButlerState enum (0..6, 127=ERROR)"),
            Field("bb_state_data", "u8",  1, "BB error code when bb_state == ERROR, else 0"),
            Field("bb_flags",      "u8",  1, "bit0 ball_in_hand, bit1 heartbeat_seen, bit2 heartbeat_stale"),
            Field("bb_yaw_deg",    "f32", 1, "BB yaw (deg), decoded with HeartbeatEncoding scale"),
            Field("bb_pitch_deg",  "f32", 1, "BB pitch (deg)"),
            Field("bb_hand_mm",    "f32", 1, "BB hand position (mm)"),
            # ── Leg guard-deviation diagnostics (2026-07-10 MAX_DEVIATION-runaway
            # forensics). Per-leg live deviation + lead-clamp state at 10 Hz, plus a
            # latch-event snapshot frozen at the MAX_DEVIATION crossing. The 2026-07-10
            # runaway could not be pinned because the 10 Hz link_status straddled the
            # crossing and no per-leg deviation was recorded; these fields make every
            # future stutter/latch bag self-diagnosing. Bridge publishes them as
            # /link_status KeyValues (recorded in the rosbag).
            Field("live_deviation",  "f32", 6, "Per-leg live deviation u0-encoder (rev) — the MAX_DEVIATION guard quantity"),
            Field("lead_clamp_mask", "u8",  1, "bit i set = leg i's interp lead clamp engaged on the last 500 Hz tick"),
            Field("max_dev_leg",     "u8",  1, "Leg that crossed MAX_DEVIATION at the last latch (0xFF = none since boot)"),
            Field("max_dev_value",   "f32", 1, "Deviation u0-encoder (rev) of max_dev_leg frozen at the latch crossing"),
            Field("max_dev_u0",      "f32", 1, "Commanded base u0 (rev) of max_dev_leg frozen at the latch crossing"),
            Field("max_dev_enc",     "f32", 1, "Encoder position (rev) of max_dev_leg frozen at the latch crossing"),
        ],
    ),
    Message(
        "Profile", "PROFILE", "T2J", "STREAM",
        summary=(
            "1 Hz firmware instrumentation. Per-task CPU%, CAN bus utilisation, "
            "UDP round-trip/jitter, the 500 Hz interp deadline-miss counter, and "
            "free heap. Consumed by tools/probes/teensy_link_profiling/jetson."),
        fields=[
            Field("t_teensy_us",  "u64", 1, "Teensy wall-clock (us)"),
            Field("cpu_pct_x100", "u16", 9, "Per-task CPU load, pct*100 (PROFILE_NUM_TASKS)"),
            Field("can1_rx",      "u32", 1, "wire slot 1 = Jugglebot core (CAN3) frames received this window"),
            Field("can1_tx",      "u32", 1, "wire slot 1 = Jugglebot core (CAN3) frames transmitted this window"),
            Field("can2_rx",      "u32", 1, "wire slot 2 = Ball Butler (CAN1) frames received this window"),
            Field("can2_tx",      "u32", 1, "wire slot 2 = Ball Butler (CAN1) frames transmitted this window"),
            Field("can1_util_x100", "u16", 1, "wire slot 1 = Jugglebot core (CAN3) bus utilisation, pct*100"),
            Field("can2_util_x100", "u16", 1, "wire slot 2 = Ball Butler (CAN1) bus utilisation, pct*100"),
            Field("udp_rtt_us",   "u32", 1, "Last measured Jetson round-trip (us)"),
            Field("udp_jitter_us", "u32", 1, "RTT jitter estimate (us)"),
            Field("interp_deadline_misses", "u32", 1, "Cumulative 500 Hz deadline misses"),
            Field("interp_max_jitter_us", "u32", 1, "Worst interp tick jitter this window (us)"),
            Field("free_heap_bytes", "u32", 1, "FreeRTOS free heap (bytes)"),
        ],
    ),
    Message(
        "ConeFrame", "CONE_FRAME", "T2J", "STREAM",
        summary=(
            "Catching-cone CAN2 frame relay. The "
            "can-bridge forwards every frame received on the cone bus "
            "verbatim — CATCH_EVENT (0x7E0) and CONE_HEARTBEAT (0x7E1) today "
            "— so the Jetson reuses the tested jugglebot.can.catching_cone "
            "decoders unchanged and future cone frames flow without a wire "
            "change. The cone's microsecond impact timestamp travels INSIDE "
            "`data` (it is latched in the cone's piezo ISR); `t_bridge_us` "
            "only stamps bridge-side CAN RX for latency/diagnostic checks."),
        fields=[
            Field("t_bridge_us", "u64", 1, "Bridge wall-clock at CAN2 RX (us)"),
            Field("can_id",      "u32", 1, "CAN arbitration id (0x7E0 CATCH_EVENT / 0x7E1 CONE_HEARTBEAT)"),
            Field("dlc",         "u8",  1, "CAN payload length (0..8)"),
            Field("data",        "u8",  8, "Raw CAN payload bytes (zero-padded past dlc)"),
        ],
    ),
    Message(
        "CmdResultFrame", "CMD_RESULT", "T2J", "STREAM",
        summary=(
            "Ball Butler command-outcome relay (the loud outcome channel). The "
            "can-bridge forwards the BB CMD_RESULT CAN1 frame (0x7D5) verbatim so "
            "the host learns the firmware's terminal outcome of an operator command "
            "(throw today; reload/calibrate/home later) instead of only the "
            "bridge-side RPC ack. The decoded payload lives INSIDE `data`: "
            "byte0=command_type, byte1=command_outcome (shared base 0x00-0x0F + "
            "per-command extension >=0x20), bytes2-3=detail0 (int16 LE), "
            "bytes4-5=detail1 (int16 LE). `t_bridge_us` only stamps bridge-side "
            "CAN1 RX for latency/diagnostic checks."),
        fields=[
            Field("t_bridge_us", "u64", 1, "Bridge wall-clock at CAN1 RX (us)"),
            Field("can_id",      "u32", 1, "CAN arbitration id (0x7D5 CMD_RESULT)"),
            Field("dlc",         "u8",  1, "CAN payload length (0..8)"),
            Field("data",        "u8",  8, "Raw CAN payload bytes (zero-padded past dlc)"),
        ],
    ),
    Message(
        "BbAxisEstimates", "BB_AXIS_ESTIMATES", "T2J", "STREAM",
        summary=(
            "High-rate Ball Butler pitch(node 7)/hand(node 8) ODrive encoder "
            "estimates, forwarded for during-throw diagnostics (launch angle vs "
            "commanded pitch; hand launch speed vs commanded). The can-bridge "
            "decodes the CAN1 get_encoder_estimate frames into its bb_axes cache "
            "(can_buses.cpp) at the ODrive broadcast rate; this message snapshots "
            "that cache at the telemetry-task rate. Pitch position maps to barrel "
            "degrees via deg = 90 + 360*rev (PitchAxis.h); hand velocity maps to "
            "ball speed via v = vel_rps * 2*pi*HAND_SPOOL_RADIUS_M."),
        fields=[
            Field("t_bridge_us",  "u64", 1, "Bridge wall-clock at emit (us, time-synced to Jetson)"),
            Field("pitch_pos_rev", "f32", 1, "BB pitch (node 7) pos_estimate (rev)"),
            Field("pitch_vel_rps", "f32", 1, "BB pitch (node 7) vel_estimate (rev/s)"),
            Field("hand_pos_rev",  "f32", 1, "BB hand (node 8) pos_estimate (rev)"),
            Field("hand_vel_rps",  "f32", 1, "BB hand (node 8) vel_estimate (rev/s)"),
        ],
    ),
    Message(
        "LegCmd", "LEG_CMD", "T2J", "STREAM",
        summary=(
            "The Teensy's COMMANDED leg interp output — the float32 cubic-Hermite "
            "ladder result (after the lead + stroke clamps) that leg_interp.cpp "
            "writes to axes[i].target_pos_rev each 500 Hz tick and would send to "
            "the leg ODrives — snapshotted at the telemetry-task rate. Additive "
            "diagnostic (no existing frame changes, so NO PROTOCOL_VERSION bump): "
            "it exposes the on-Teensy float32 interpolator output so a bench "
            "validation can measure the float32-vs-float64 interp residual "
            "DIRECTLY, rather than inferring it "
            "from the encoder. Written for all legs regardless of the output gate, "
            "so it reflects the interp even when CAN3 TX is suppressed. Jugglebot "
            "convention (positive = extension)."),
        fields=[
            Field("t_teensy_us", "u64", 1, "Teensy wall-clock at snapshot (us)"),
            Field("cmd_pos_rev", "f32", 6, "Per-leg commanded interp position (rev), post lead/stroke clamp"),
            Field("cmd_vel_rps", "f32", 6, "Per-leg commanded interp velocity FF (rev/s)"),
        ],
    ),
    Message(
        "PlatformFrame", "PLATFORM_FRAME", "T2J", "STREAM",
        summary=(
            "Verbatim Platform-Teensy relay-reply uplink. The can-bridge "
            "forwards every CAN3 frame "
            "it receives whose arbitration id is a Platform-Teensy reply "
            "(STATE_UPDATE 0x6E0 RobotState, TILT_READING 0x7DE inclinometer) "
            "verbatim, so the host owns the decode and the bridge stays decoupled "
            "from the Platform-Teensy byte layout (Teensy_code.ino "
            "createStateCANMessage / sendTiltData). The host correlates a reply to "
            "its pending relay read by (can_id, dlc): a STATE_READ awaits "
            "(0x6E0, 8); a TILT_READ awaits (0x7DE, 8). `t_bridge_us` only stamps "
            "bridge-side CAN3 RX for latency/diagnostics. NOTE(bench): the "
            "(id, dlc) discriminator is only sound if CAN3 SRX_DIS is set so the "
            "bridge's own 0x6E0 STATE_WRITE is not looped back as a reply — "
            "verify on the bench before trusting on hardware."),
        fields=[
            Field("t_bridge_us", "u64", 1, "Bridge wall-clock at CAN3 RX (us)"),
            Field("can_id",      "u32", 1, "CAN arbitration id (0x6E0 STATE_UPDATE / 0x7DE TILT_READING)"),
            Field("dlc",         "u8",  1, "CAN payload length (0..8)"),
            Field("data",        "u8",  8, "Raw CAN payload bytes (zero-padded past dlc)"),
        ],
    ),
    Message(
        "HandCmdEcho", "HAND_CMD_ECHO", "T2J", "STREAM",
        summary=(
            "Hand command-echo telemetry. The can-bridge sniffs the Platform "
            "Teensy's Set_Input_Pos "
            "command to the HAND ODrive (axis 6) on CAN3 — arb_id(6, set_input_pos, "
            "cmd 0x0C) — and forwards the raw 8-byte payload verbatim so the host "
            "echoes the hand's COMMANDED pos/vel_ff/tor_ff (can_node._handle_hand_"
            "input_pos parity; the hand_telemetry pos_cmd/vel_ff_cmd/tor_ff_cmd "
            "fields were hardcoded 0 on the bridge until now). The host decodes "
            "`data` as `<f h h>` (float32 pos_rev + int16 vel_ff + int16 tor_ff) and "
            "divides vel/tor by INPUT_SCALE_HAND_VEL / INPUT_SCALE_HAND_TOR (100.0). "
            "Emitted at the telemetry-task rate only when a FRESH command was sniffed "
            "(event-driven; silent while the hand is idle). CAN3 SRX_DIS means the "
            "bridge never sniffs its own TX, so only genuine Platform→hand commands "
            "are echoed. `t_bridge_us` stamps CAN3 RX for latency/diagnostics."),
        fields=[
            Field("t_bridge_us", "u64", 1, "Bridge wall-clock at CAN3 RX of the hand Set_Input_Pos (us)"),
            Field("data",        "u8",  8, "Raw ODrive Set_Input_Pos payload: <f h h> = pos_rev, vel_ff, tor_ff"),
        ],
    ),
    Message(
        "RpcRequest", "RPC_REQUEST", "J2T", "RPC",
        summary=(
            "Generic RPC envelope. `method` selects the operation; `args` is a "
            "method-specific blob (see docs). `req_id` is echoed in the response "
            "for matching independent of the frame sequence counter."),
        fields=[
            Field("method",  "u16", 1, "RpcMethod enum"),
            Field("req_id",  "u16", 1, "Caller-chosen id, echoed in response"),
            Field("arg_len", "u16", 1, "Bytes of args following the fixed head"),
            Field("pad",     "u16", 1, "Alignment pad (zero)"),
            # args[] follows — variable length, not a fixed struct field.
        ],
    ),
    Message(
        "RpcResponse", "RPC_RESPONSE", "T2J", "RPC",
        summary="Response to an RpcRequest. `result` is method-specific.",
        fields=[
            Field("method",  "u16", 1, "Echo of request method"),
            Field("req_id",  "u16", 1, "Echo of request req_id"),
            Field("status",  "u16", 1, "RpcStatus enum"),
            Field("res_len", "u16", 1, "Bytes of result following the fixed head"),
            # result[] follows — variable length.
        ],
    ),
]

# RpcRequest/RpcResponse carry a trailing variable blob, so their "struct" is
# only the fixed head. Mark them so the generator emits head-size constants.
VARIABLE_TAIL = {"RpcRequest", "RpcResponse"}


# ───────────────────────────────────────────────────────────────────────────
# RPC METHOD ARGUMENT LAYOUTS
#
# The per-method argument blobs that ride inside an RpcRequest (and the one
# result blob). Hoisted into this single-source generator when the
# Jetson UDP bridge became the second consumer. Emitted as packed,
# little-endian structs into the C++ header
# (JbUdp::RpcArgs), the Python module (dataclasses), and the markdown spec; the
# firmware's rpc.h consumes the generated C++ structs via `using` declarations,
# and controller/teensy_link/rpc_args.py wraps the generated Python.
#
# Field names MUST match the firmware rpc.cpp dispatch (a.axis, a.state, ...).
# ───────────────────────────────────────────────────────────────────────────

@dataclass
class RpcArg:
    name: str            # struct name, e.g. "ArgAxisState"
    methods: str         # which RpcMethod(s) it serves (doc/comment only)
    fields: list         # list[Field] — scalar, packed little-endian
    summary: str = ""

    @property
    def size(self) -> int:
        return sum(f.width for f in self.fields)

    @property
    def struct_fmt(self) -> str:
        return "<" + "".join(f.struct_fmt for f in self.fields)


AXIS_ALL = 0xFF  # broadcast-to-all-legs sentinel (CLEAR_ERRORS / REBOOT_ODRIVES)

RPC_ARGS = [
    RpcArg("ArgAxisState", "SET_AXIS_STATE", [
        Field("axis",  "u8",  1, "ODrive axis 0..5 (or AXIS_ALL where supported)"),
        Field("state", "u32", 1, "ODrive requested AxisState (AXIS_STATES value)"),
    ]),
    RpcArg("ArgControllerMode", "SET_CONTROLLER_MODE", [
        Field("axis",  "u8",  1, "ODrive axis 0..5"),
        Field("ctrl",  "u32", 1, "ODrive control_mode (CONTROL_MODES value)"),
        Field("input", "u32", 1, "ODrive input_mode (INPUT_MODES value)"),
    ]),
    RpcArg("ArgVelCurr", "SET_VEL_CURR_LIMITS", [
        Field("axis",       "u8",  1, "ODrive axis 0..5"),
        Field("vel_limit",  "f32", 1, "velocity limit (rev/s)"),
        Field("curr_limit", "f32", 1, "current limit (A)"),
    ]),
    RpcArg("ArgPosGain", "SET_POS_GAIN", [
        Field("axis",     "u8",  1, "ODrive axis 0..5"),
        Field("pos_gain", "f32", 1, "position gain"),
    ]),
    RpcArg("ArgVelGains", "SET_VEL_GAINS", [
        Field("axis",         "u8",  1, "ODrive axis 0..5"),
        Field("vel_gain",     "f32", 1, "velocity gain"),
        Field("vel_int_gain", "f32", 1, "velocity integrator gain"),
    ]),
    RpcArg("ArgAbsPosition", "SET_ABSOLUTE_POSITION", [
        Field("axis",     "u8",  1, "ODrive axis 0..5"),
        Field("position", "f32", 1, "absolute position (rev), post-homing"),
    ]),
    RpcArg("ArgAxisOnly", "CLEAR_ERRORS / REBOOT_ODRIVES / ENCODER_SEARCH / HOME / ACTIVATE / DEACTIVATE", [
        Field("axis", "u8", 1, "ODrive axis 0..5, or AXIS_ALL for broadcast"),
    ]),
    RpcArg("ArgSdoRead", "SDO_READ", [
        Field("axis",     "u8",  1, "ODrive axis 0..5"),
        Field("endpoint", "u16", 1, "ODrive SDO endpoint id"),
    ]),
    RpcArg("ArgSdoWrite", "SDO_WRITE", [
        Field("axis",     "u8",  1, "ODrive axis 0..5"),
        Field("endpoint", "u16", 1, "ODrive SDO endpoint id"),
        Field("value",    "f32", 1, "value to write"),
    ]),
    RpcArg("ResultTimeOfDay", "TIME_OF_DAY_QUERY (result)", [
        Field("jetson_wall_us", "u64", 1, "Jetson CLOCK_REALTIME microseconds"),
    ]),
    # GET_AXIS_VERSIONS result.
    # GET_AXIS_VERSIONS takes NO args (like TILT_READ/STATE_READ) but, unlike the
    # relay reads, returns SYNCHRONOUSLY in the RPC response (the versions are a
    # bridge-LOCAL cache filled by the firmware's Get_Version sweep — no CAN3
    # round-trip on the pull). The blob carries the raw 8-byte ODrive Get_Version
    # payload for every Jugglebot axis (axis-major: axis 0..NUM_AXES-1, legs then
    # hand) plus a received bitmask (bit i set ⇒ axis i's reply has been cached).
    # The bridge decodes the set-bit axes via jugglebot.can.odrive.decode_get_version
    # and runs the existing tested MotorStateTracker.validate_group — ZERO version
    # semantics in the firmware. raw is u8[NUM_AXES*8] = u8[56] (NUM_AXES=7).
    RpcArg("ResultAxisVersions", "GET_AXIS_VERSIONS (result)", [
        Field("received_mask", "u8", 1, "bit i set ⇒ axis i Get_Version reply cached"),
        Field("raw", "u8", 56, "raw 8-byte Get_Version payload per axis (NUM_AXES*8, axis-major)"),
    ]),
    # Ball Butler — typed firmware-side encoders own the wire format (the can-bridge
    # refuses a malformed throw before it hits CAN1).
    # The Python encoder in jugglebot.can.ball_butler stays as the test spec; a
    # byte-level cross-reference test pins parity. RELOAD/RESET/CALIBRATE_LOC are
    # payloadless — no Arg struct (caller sends b"" — matches the NOP pattern).
    RpcArg("ArgBbThrow", "BB_THROW", [
        Field("yaw_rad",   "f32", 1, "Yaw angle in radians [-pi, pi)"),
        Field("pitch_rad", "f32", 1, "Pitch angle in radians [0, pi/2]"),
        Field("speed_mps", "f32", 1, "Throw speed in m/s [0, 6.5535]"),
        Field("delay_s",   "f32", 1, "Relative delay before throw (s) [0, 65.535]"),
    ]),
    # STATE_WRITE carries the WHOLE Platform-Teensy RobotState (the can-bridge is
    # the sole writer and does read-modify-write through its cache, so a homing
    # write preserves the levelling fields and vice versa). The firmware encodes
    # the 0x6E0 frame itself (mirroring Teensy_code.ino createStateCANMessage) —
    # the can-bridge never forwards a Jetson-supplied raw frame (least-privilege).
    # (TILT_READ / STATE_READ take
    # no args — they only trigger a Platform-Teensy reply on 0x7DE / 0x6E0.)
    RpcArg("ArgRobotState", "STATE_WRITE", [
        Field("is_homed",          "u8",  1, "Legs+hand homing complete"),
        Field("levelling_complete", "u8", 1, "Platform levelling complete"),
        Field("pose_offset_tiltX", "f32", 1, "Levelling pose offset, tilt about X (rad)"),
        Field("pose_offset_tiltY", "f32", 1, "Levelling pose offset, tilt about Y (rad)"),
    ]),
    # HAND_TRAJ_CMD carries the EXACT 8-byte 0x6D0 PLATFORM_TRAJ_CMD payload, built
    # HOST-side byte-identical to can_node._send_hand_traj_cmd / _smooth_move_hand
    # (byte 0 = discriminator: 0/1/2 = catch-traj type, 3 = smooth-move). The
    # can-bridge attaches the FIRMWARE-OWNED 0x6D0 arbitration id (never a Jetson-
    # supplied raw frame — least-privilege, same principle as STATE_WRITE re-encoding
    # 0x6E0) and forwards the payload after the CLOSED_LOOP + POSITION/PASSTHROUGH
    # preamble. The absolute wall_time_ms deadline is baked into the payload by the
    # Jetson; the firmware forwards OPAQUE bytes and CANNOT re-stamp — an absolute
    # deadline is immune to Jetson→bridge→CAN3 transit jitter (the Platform Teensy
    # fires when its synced clock reaches the deadline).
    RpcArg("ArgHandTraj", "HAND_TRAJ_CMD", [
        Field("payload", "u8", 8, "Exact 8-byte 0x6D0 PLATFORM_TRAJ_CMD payload (host-built; byte-0 discriminator)"),
    ]),
]

# ───────────────────────────────────────────────────────────────────────────
# HAND AXIS-6 ALLOW-TABLE
#
# Which RpcMethods the can-bridge forwards to the HAND ODrive (axis 6) on CAN3,
# replacing the blanket `axis == HAND_AXIS` reject (rpc.cpp). The hand reuses the
# leg encoders behind this NARROW (method, axis) gate; everything not listed is
# rejected on axis 6, and every permitted op is still gated on
# jugglebot_commands_allowed() (the hand is gated like a leg, never ungated).
# Single source → C++ predicate (JbUdp::hand_axis6_permitted, consumed by rpc.cpp)
# + Python frozenset (the mirror test tests/firmware/test_hand_axis6_allow.py).
#
# Permit: the ODrive config + lifecycle ops hand homing needs (SET_AXIS_STATE,
# SET_CONTROLLER_MODE, the three gains, CLEAR_ERRORS, REBOOT_ODRIVES, HOME,
# SET_ABSOLUTE_POSITION). Reject: ENCODER_SEARCH (absolute encoder), ACTIVATE /
# DEACTIVATE (leg-specific cold-start moves), SDO_*, and the BB / time methods.
# ───────────────────────────────────────────────────────────────────────────

HAND_AXIS6_PERMITTED = [
    "SET_AXIS_STATE",
    "SET_CONTROLLER_MODE",
    "SET_POS_GAIN",
    "SET_VEL_GAINS",
    "SET_VEL_CURR_LIMITS",
    "CLEAR_ERRORS",
    "REBOOT_ODRIVES",
    "HOME",
    "SET_ABSOLUTE_POSITION",
]

# Fail the codegen if any allow-table entry is not a real RpcMethod (a rename
# would otherwise silently drop a permission and re-blanket-reject the hand).
_RPC_METHOD_NAMES = {name for name, _v, _c in ENUMS["RpcMethod"]}
for _m in HAND_AXIS6_PERMITTED:
    assert _m in _RPC_METHOD_NAMES, f"HAND_AXIS6_PERMITTED: unknown RpcMethod {_m!r}"


# ───────────────────────────────────────────────────────────────────────────
# CRC-16/CCITT-FALSE  (poly 0x1021, init 0xFFFF, no reflect, no xorout)
# ───────────────────────────────────────────────────────────────────────────

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


# ───────────────────────────────────────────────────────────────────────────
# C++ header generation
# ───────────────────────────────────────────────────────────────────────────

def _const_cpp_type(ctype: str) -> str:
    return TYPES[ctype][0]


def generate_cpp() -> str:
    L = []
    a = L.append
    a("#pragma once")
    a("// AUTO-GENERATED by config/generate_udp_protocol.py — DO NOT EDIT")
    a("// Jetson <-> can-bridge Teensy UDP wire protocol.")
    a("// See docs/teensy-udp-protocol.md. To regenerate: python config/generate_udp_protocol.py")
    a("")
    a("#include <cstdint>")
    a("#include <cstring>")
    a("")
    a("namespace JbUdp {")
    a("")
    a("// ── Constants ──────────────────────────────────────────────────────────")
    for name, val, ctype, comment in CONSTANTS:
        ct = _const_cpp_type(ctype)
        valstr = f"0x{val:X}" if (ctype.startswith("u") and val > 9 and name == "MAGIC") else str(val)
        suffix = "u" if ct.startswith("uint") else ""
        line = f"constexpr {ct} {name} = {valstr}{suffix};"
        if comment:
            line += f"  // {comment}"
        a(line)
    a("")

    a("// ── Enums ──────────────────────────────────────────────────────────────")
    for enum_name, members in ENUMS.items():
        ct = TYPES[ENUM_WIDTH[enum_name]][0]   # u8→uint8_t, u16→uint16_t, u32→uint32_t
        a(f"namespace {enum_name} {{")
        for member, value, comment in members:
            valstr = f"0x{value:04X}u" if ct == "uint16_t" else f"{value}u"
            line = f"  constexpr {ct} {member} = {valstr};"
            if comment:
                line += f"  // {comment}"
            a(line)
        a("}")
    a("")

    a("// ── Frame header (8 bytes) + trailing CRC-16 ───────────────────────────")
    a("// Wire frame = [Header(8)][payload(length)][crc16(2)], all little-endian.")
    a("#pragma pack(push, 1)")
    a("struct Header {")
    a("  uint16_t magic;     // == MAGIC")
    a("  uint8_t  version;   // == PROTOCOL_VERSION")
    a("  uint8_t  msg_type;  // MsgType")
    a("  uint16_t seq;       // per-(channel,direction) sequence counter")
    a("  uint16_t length;    // payload byte count")
    a("};")
    a("#pragma pack(pop)")
    a("static_assert(sizeof(Header) == 8, \"Header must be 8 bytes\");")
    a("")

    a("// ── Payload structs (packed; little-endian native on M7 & x86) ─────────")
    a("#pragma pack(push, 1)")
    for msg in MESSAGES:
        a(f"// {msg.name}: {msg.summary.strip()}")
        sname = f"{msg.name}Payload"
        a(f"struct {sname} {{")
        for f in msg.fields:
            line = f"  {f.cpp_decl}"
            if f.comment:
                line += f"  // {f.comment}"
            a(line)
        a("};")
        if msg.name in VARIABLE_TAIL:
            a(f"// NOTE: {sname} is the FIXED HEAD only; a variable blob follows on the wire.")
        a(f"static_assert(sizeof({sname}) == {msg.payload_size}, "
          f"\"{sname} size drift\");")
        a("")
    a("#pragma pack(pop)")
    a("")

    a("// ── Per-message constants ──────────────────────────────────────────────")
    for msg in MESSAGES:
        a(f"constexpr uint16_t {_screaming(msg.name)}_SIZE = {msg.payload_size}u;")
    a("")

    a("// ── RPC method argument layouts (packed; little-endian) ────────────────")
    a("// Per-method arg blobs riding inside RpcRequest, + the one result blob.")
    a("// The firmware rpc.h consumes these via `using JbUdp::RpcArgs::...`.")
    a("namespace RpcArgs {")
    a(f"constexpr uint8_t AXIS_ALL = 0x{AXIS_ALL:X}u;  // broadcast-to-all-legs sentinel")
    a("#pragma pack(push, 1)")
    for arg in RPC_ARGS:
        a(f"// {arg.name} ({arg.methods})")
        a(f"struct {arg.name} {{")
        for f in arg.fields:
            line = f"  {f.cpp_decl}"
            if f.comment:
                line += f"  // {f.comment}"
            a(line)
        a("};")
        a(f"static_assert(sizeof({arg.name}) == {arg.size}, \"{arg.name} size drift\");")
    a("#pragma pack(pop)")
    for arg in RPC_ARGS:
        a(f"constexpr uint16_t {_screaming(arg.name)}_SIZE = {arg.size}u;")
    a("}  // namespace RpcArgs")
    a("")

    a("// ── Hand axis-6 allow-table ──────────────────────────────────────────────")
    a("// True iff RpcMethod `method` may be forwarded to the HAND ODrive (axis 6)")
    a("// on CAN3, replacing the blanket axis==HAND_AXIS reject. Consumed by")
    a("// rpc.cpp's send_axis_frame; mirrored by tests/firmware/test_hand_axis6_allow.py.")
    a("inline bool hand_axis6_permitted(uint16_t method) {")
    a("  switch (method) {")
    for name in HAND_AXIS6_PERMITTED:
        a(f"    case RpcMethod::{name}:")
    a("      return true;")
    a("    default:")
    a("      return false;")
    a("  }")
    a("}")
    a("")

    a("// ── CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) ──────────────────────")
    a("inline uint16_t crc16_ccitt(const uint8_t* data, uint16_t len) {")
    a("  uint16_t crc = 0xFFFF;")
    a("  for (uint16_t i = 0; i < len; ++i) {")
    a("    crc ^= (uint16_t)data[i] << 8;")
    a("    for (uint8_t b = 0; b < 8; ++b) {")
    a("      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);")
    a("    }")
    a("  }")
    a("  return crc;")
    a("}")
    a("")

    a("// ── Framing helpers (zero-allocation; operate on caller buffers) ───────")
    a("// Encode a frame into `out` (capacity out_cap). Returns total bytes, or 0 on error.")
    a("inline uint16_t encode_frame(uint8_t msg_type, uint16_t seq,")
    a("                             const uint8_t* payload, uint16_t len,")
    a("                             uint8_t* out, uint16_t out_cap) {")
    a("  const uint16_t total = (uint16_t)(HEADER_SIZE + len + CRC_SIZE);")
    a("  if (len > MAX_PAYLOAD || total > out_cap) return 0;")
    a("  if (len && !payload) return 0;   // len>0 with a null payload would else CRC over")
    a("                                   // uninitialized bytes and return a 'valid' garbage frame")
    a("  Header h{ MAGIC, PROTOCOL_VERSION, msg_type, seq, len };")
    a("  memcpy(out, &h, HEADER_SIZE);")
    a("  if (len && payload) memcpy(out + HEADER_SIZE, payload, len);")
    a("  const uint16_t crc = crc16_ccitt(out, (uint16_t)(HEADER_SIZE + len));")
    a("  out[HEADER_SIZE + len]     = (uint8_t)(crc & 0xFF);")
    a("  out[HEADER_SIZE + len + 1] = (uint8_t)(crc >> 8);")
    a("  return total;")
    a("}")
    a("")
    a("// Validate + parse a received frame. On success fills *hdr and points")
    a("// *payload at the in-buffer payload, returning true. Rejects bad magic,")
    a("// version, length overrun, or CRC mismatch.")
    a("inline bool decode_frame(const uint8_t* in, uint16_t in_len,")
    a("                         Header* hdr, const uint8_t** payload) {")
    a("  if (in_len < HEADER_SIZE + CRC_SIZE) return false;")
    a("  memcpy(hdr, in, HEADER_SIZE);")
    a("  if (hdr->magic != MAGIC) return false;")
    a("  if (hdr->version != PROTOCOL_VERSION) return false;")
    a("  if ((uint16_t)(HEADER_SIZE + hdr->length + CRC_SIZE) != in_len) return false;")
    a("  if (hdr->length > MAX_PAYLOAD) return false;")
    a("  const uint16_t want = crc16_ccitt(in, (uint16_t)(HEADER_SIZE + hdr->length));")
    a("  const uint16_t got = (uint16_t)(in[HEADER_SIZE + hdr->length] |")
    a("                                  (in[HEADER_SIZE + hdr->length + 1] << 8));")
    a("  if (want != got) return false;")
    a("  *payload = in + HEADER_SIZE;")
    a("  return true;")
    a("}")
    a("")
    a("}  // namespace JbUdp")
    a("")
    return "\n".join(L)


def _screaming(camel: str) -> str:
    # Insert "_" only before a capital that follows a LOWERCASE LETTER, so a
    # digit→capital boundary is NOT split: HeartbeatJ2T → HEARTBEAT_J2T (not
    # HEARTBEAT_J2_T), RpcRequest → RPC_REQUEST, Setpoint → SETPOINT.
    out = []
    for i, c in enumerate(camel):
        if c.isupper() and i > 0 and camel[i - 1].islower():
            out.append("_")
        out.append(c.upper())
    return "".join(out)


# ───────────────────────────────────────────────────────────────────────────
# Python module generation
# ───────────────────────────────────────────────────────────────────────────

def generate_python() -> str:
    L = []
    a = L.append
    a('"""')
    a("AUTO-GENERATED by config/generate_udp_protocol.py — DO NOT EDIT")
    a("Jetson <-> can-bridge Teensy UDP wire protocol (Python side).")
    a("See docs/teensy-udp-protocol.md. To regenerate: python config/generate_udp_protocol.py")
    a('"""')
    a("from __future__ import annotations")
    a("import struct")
    a("from dataclasses import dataclass, field")
    a("from enum import IntEnum")
    a("")
    a("# ── Constants ──────────────────────────────────────────────────────────")
    for name, val, ctype, comment in CONSTANTS:
        line = f"{name} = {val}"
        if comment:
            line += f"  # {comment}"
        a(line)
    a("")

    a("# ── Enums ──────────────────────────────────────────────────────────────")
    for enum_name, members in ENUMS.items():
        a(f"class {enum_name}(IntEnum):")
        for member, value, comment in members:
            line = f"    {member} = {value}"
            if comment:
                line += f"  # {comment}"
            a(line)
        a("")

    a("# ── Decode errors ──────────────────────────────────────────────────────")
    a("class CrcError(ValueError):")
    a('    """decode_frame() raised this on a CRC-16 mismatch (a corrupted frame).')
    a("")
    a("    Subclasses ValueError so existing ``except ValueError`` handlers still")
    a("    catch it; callers that must distinguish a corrupted frame (bad CRC) from")
    a("    a structurally-malformed one (bad magic/version/length) catch CrcError")
    a("    specifically. See controller/teensy_link/client.py's RX decode path,")
    a("    which counts crc_errors vs decode_errors off this distinction.")
    a('    """')
    a("")

    a("# ── CRC-16/CCITT-FALSE ─────────────────────────────────────────────────")
    a("def crc16_ccitt(data: bytes) -> int:")
    a("    crc = 0xFFFF")
    a("    for b in data:")
    a("        crc ^= b << 8")
    a("        for _ in range(8):")
    a("            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF")
    a("    return crc & 0xFFFF")
    a("")

    a("# ── Frame header ───────────────────────────────────────────────────────")
    a('_HEADER_FMT = "<HBBHH"  # magic, version, msg_type, seq, length')
    a("assert struct.calcsize(_HEADER_FMT) == HEADER_SIZE")
    a("")
    a("def encode_frame(msg_type: int, seq: int, payload: bytes) -> bytes:")
    a('    """Build a full wire frame: header + payload + CRC-16."""')
    a("    if len(payload) > MAX_PAYLOAD:")
    a('        raise ValueError(f"payload {len(payload)} > MAX_PAYLOAD {MAX_PAYLOAD}")')
    a("    head = struct.pack(_HEADER_FMT, MAGIC, PROTOCOL_VERSION, msg_type,")
    a("                       seq & 0xFFFF, len(payload))")
    a("    body = head + payload")
    a("    crc = crc16_ccitt(body)")
    a("    return body + struct.pack('<H', crc)")
    a("")
    a("def decode_frame(frame: bytes):")
    a('    """Validate + parse a wire frame. Returns (msg_type, seq, payload).')
    a("")
    a("    Raises ValueError on bad magic/version/length/CRC.")
    a('    """')
    a("    if len(frame) < HEADER_SIZE + CRC_SIZE:")
    a('        raise ValueError("frame too short")')
    a("    magic, version, msg_type, seq, length = struct.unpack_from(_HEADER_FMT, frame, 0)")
    a("    if magic != MAGIC:")
    a('        raise ValueError(f"bad magic 0x{magic:04X}")')
    a("    if version != PROTOCOL_VERSION:")
    a('        raise ValueError(f"bad version {version}")')
    a("    if HEADER_SIZE + length + CRC_SIZE != len(frame):")
    a('        raise ValueError(f"length {length} inconsistent with frame {len(frame)}")')
    a("    want = crc16_ccitt(frame[:HEADER_SIZE + length])")
    a("    got = struct.unpack_from('<H', frame, HEADER_SIZE + length)[0]")
    a("    if want != got:")
    a('        raise CrcError(f"CRC mismatch want 0x{want:04X} got 0x{got:04X}")')
    a("    payload = frame[HEADER_SIZE:HEADER_SIZE + length]")
    a("    return msg_type, seq, payload")
    a("")

    a("# ── Payload structs ────────────────────────────────────────────────────")
    for msg in MESSAGES:
        sname = msg.name
        a(f"# {msg.name}: {' '.join(msg.summary.split())}")
        a(f"{_screaming(msg.name)}_FMT = {msg.struct_fmt!r}")
        a(f"{_screaming(msg.name)}_SIZE = {msg.payload_size}")
        a(f"_{_screaming(msg.name)}_STRUCT = struct.Struct({_screaming(msg.name)}_FMT)")
        a(f"assert _{_screaming(msg.name)}_STRUCT.size == {msg.payload_size}")
        a("")
        # dataclass
        a("@dataclass")
        a(f"class {sname}:")
        for f in msg.fields:
            if f.count == 1:
                default = "0.0" if f.type in ("f32", "f64") else "0"
                a(f"    {f.name}: {_py_field_type(f)} = {default}")
            else:
                zero = "0.0" if f.type in ("f32", "f64") else "0"
                a(f"    {f.name}: tuple = field("
                  f"default_factory=lambda: ({zero},) * {f.count})")
        # pack
        a("")
        a("    def pack(self) -> bytes:")
        flat = []
        for f in msg.fields:
            if f.count == 1:
                flat.append(f"self.{f.name}")
            else:
                flat.append(f"*self.{f.name}")
        a(f"        return _{_screaming(msg.name)}_STRUCT.pack({', '.join(flat)})")
        a("")
        a("    @classmethod")
        a(f"    def unpack(cls, data: bytes) -> '{sname}':")
        a(f"        vals = _{_screaming(msg.name)}_STRUCT.unpack(data[:{msg.payload_size}])")
        # reconstruct fields, grouping arrays
        a("        it = iter(vals)")
        ctor = []
        for f in msg.fields:
            if f.count == 1:
                ctor.append(f"next(it)")
            else:
                ctor.append(f"tuple(next(it) for _ in range({f.count}))")
        a(f"        return cls({', '.join(ctor)})")
        a("")

    a("# ── RPC method argument layouts ────────────────────────────────────────")
    a(f"AXIS_ALL = {AXIS_ALL}  # broadcast-to-all-legs sentinel")
    a("")
    for arg in RPC_ARGS:
        sname = arg.name
        a(f"# {arg.name} ({arg.methods})")
        a(f"{_screaming(arg.name)}_FMT = {arg.struct_fmt!r}")
        a(f"{_screaming(arg.name)}_SIZE = {arg.size}")
        a(f"_{_screaming(arg.name)}_STRUCT = struct.Struct({_screaming(arg.name)}_FMT)")
        a(f"assert _{_screaming(arg.name)}_STRUCT.size == {arg.size}")
        a("")
        a("@dataclass")
        a(f"class {sname}:")
        for f in arg.fields:
            if f.count == 1:
                default = "0.0" if f.type in ("f32", "f64") else "0"
                a(f"    {f.name}: {_py_field_type(f)} = {default}")
            else:
                zero = "0.0" if f.type in ("f32", "f64") else "0"
                a(f"    {f.name}: tuple = field("
                  f"default_factory=lambda: ({zero},) * {f.count})")
        a("")
        a("    def pack(self) -> bytes:")
        flat = ", ".join(
            (f"*self.{f.name}" if f.count != 1 else f"self.{f.name}")
            for f in arg.fields)
        a(f"        return _{_screaming(arg.name)}_STRUCT.pack({flat})")
        a("")
        a("    @classmethod")
        a(f"    def unpack(cls, data: bytes) -> '{sname}':")
        a(f"        vals = _{_screaming(arg.name)}_STRUCT.unpack(data[:{arg.size}])")
        a("        it = iter(vals)")
        ctor = ", ".join(
            (f"tuple(next(it) for _ in range({f.count}))" if f.count != 1
             else "next(it)")
            for f in arg.fields)
        a(f"        return cls({ctor})")
        a("")

    a("# ── Hand axis-6 allow-table ──────────────────────────────────────────────")
    a("# RpcMethod ids the can-bridge forwards to the hand ODrive (axis 6); the")
    a("# single source mirrored by the firmware JbUdp::hand_axis6_permitted predicate.")
    a("HAND_AXIS6_PERMITTED = frozenset({")
    for name in HAND_AXIS6_PERMITTED:
        a(f"    int(RpcMethod.{name}),")
    a("})")
    a("")
    a("def hand_axis6_permitted(method: int) -> bool:")
    a('    """True iff `method` may be forwarded to the hand ODrive (axis 6)."""')
    a("    return int(method) in HAND_AXIS6_PERMITTED")
    a("")
    return "\n".join(L)


def _py_field_type(f: Field) -> str:
    return "float" if f.type in ("f32", "f64") else "int"


# ───────────────────────────────────────────────────────────────────────────
# Markdown doc generation
# ───────────────────────────────────────────────────────────────────────────

def _md_cell(s: str) -> str:
    # Escape `|` so a comment containing it (e.g. a flags bitset "A|B|C") renders
    # inside one table cell instead of splitting the row across extra columns.
    return s.replace("|", r"\|")


def generate_markdown() -> str:
    L = []
    a = L.append
    a("# Jetson ↔ can-bridge Teensy — UDP protocol")
    a("")
    a("> AUTO-GENERATED by `config/generate_udp_protocol.py` — **do not edit by hand**.")
    a("> Regenerate: `python config/generate_udp_protocol.py`")
    a("")
    a("This is the single source of truth for the wire protocol between the Jetson")
    a("and the new can-bridge Teensy 4.1.")
    a("")
    a("## Framing")
    a("")
    a("Every datagram is one fixed-layout, typed frame. **No COBS** — UDP datagrams")
    a("already carry message boundaries, so stream-framing is redundant; a fixed")
    a("per-type layout is zero-allocation and deterministic-timing, which the hard")
    a("real-time firmware requires.")
    a("")
    a("```")
    a("offset size field     type      notes")
    a("0      2    magic     u16 LE    0x4A42 (\"JB\")")
    a("2      1    version   u8        PROTOCOL_VERSION")
    a("3      1    msg_type  u8        MsgType")
    a("4      2    seq       u16 LE    per-(channel,direction) counter")
    a("6      2    length    u16 LE    payload byte count")
    a("8      N    payload   ...       type-specific (below)")
    a("8+N    2    crc16     u16 LE    CRC-16/CCITT-FALSE over bytes [0 .. 8+N-1]")
    a("```")
    a("")
    a("CRC-16/CCITT-FALSE: poly `0x1021`, init `0xFFFF`, no input/output reflection,")
    a("no final XOR. Computed over the header and payload (everything before the CRC).")
    a("")
    a("## Channels (UDP ports)")
    a("")
    a(f"- **Port {[v for n,v,_,_ in CONSTANTS if n=='PORT_STREAM'][0]} (STREAM)** — high-rate push: setpoint downlink, telemetry/")
    a("  diagnostic/profile/heartbeat uplink.")
    a(f"- **Port {[v for n,v,_,_ in CONSTANTS if n=='PORT_RPC'][0]} (RPC)** — request/response with retry-on-timeout: gain")
    a("  writes, state changes, encoder-search, SDO, homing, and the time-of-day query.")
    a("")
    a("Static IPs: Teensy `192.168.42.2`, Jetson `192.168.42.1` (`/30` point-to-point).")
    a("")
    a("## Constants")
    a("")
    a("| Name | Value | Notes |")
    a("|------|------:|-------|")
    for name, val, ctype, comment in CONSTANTS:
        vstr = f"0x{val:04X}" if name == "MAGIC" else str(val)
        a(f"| `{name}` | {vstr} | {_md_cell(comment)} |")
    a("")
    a("## Enums")
    a("")
    for enum_name, members in ENUMS.items():
        a(f"### {enum_name}")
        a("")
        a("| Member | Value | Notes |")
        a("|--------|------:|-------|")
        hexfmt = ENUM_WIDTH[enum_name] == "u16" or enum_name == "MsgType"
        for member, value, comment in members:
            vstr = (f"0x{value:04X}" if ENUM_WIDTH[enum_name] == "u16"
                    else f"0x{value:02X}" if enum_name == "MsgType" else str(value))
            a(f"| `{member}` | {vstr} | {_md_cell(comment)} |")
        a("")
    a("## Messages")
    a("")
    for msg in MESSAGES:
        a(f"### {msg.name} (`MsgType.{msg.msg_type}`, {msg.direction}, {msg.port} port)")
        a("")
        a(f"{msg.summary.strip()}")
        a("")
        if msg.name in VARIABLE_TAIL:
            a(f"Fixed head **{msg.payload_size} bytes**, followed by a variable blob.")
        else:
            a(f"Payload **{msg.payload_size} bytes**. Python struct fmt: `{msg.struct_fmt}`.")
        a("")
        a("| Field | Type | Count | Notes |")
        a("|-------|------|------:|-------|")
        for f in msg.fields:
            a(f"| `{f.name}` | {f.type} | {f.count} | {_md_cell(f.comment)} |")
        a("")
    a("## RPC method arguments")
    a("")
    a("Per-method argument blobs riding inside an `RpcRequest` (and the one")
    a("result blob). Packed, little-endian. The firmware `rpc.h` consumes the")
    a("generated `JbUdp::RpcArgs::*` structs; `controller/teensy_link/rpc_args.py`")
    a(f"wraps the generated Python. `AXIS_ALL = 0x{AXIS_ALL:X}` broadcasts to all legs.")
    a("")
    for arg in RPC_ARGS:
        a(f"### {arg.name} (`{arg.methods}`)")
        a("")
        a(f"**{arg.size} bytes**. Python struct fmt: `{arg.struct_fmt}`.")
        a("")
        a("| Field | Type | Notes |")
        a("|-------|------|-------|")
        for f in arg.fields:
            a(f"| `{f.name}` | {f.type} | {_md_cell(f.comment)} |")
        a("")
    return "\n".join(L)


# ───────────────────────────────────────────────────────────────────────────
# Main
# ───────────────────────────────────────────────────────────────────────────

def _write(path: Path, content: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    print(f"Generated: {path.relative_to(REPO_ROOT)}")


def _deliver(path: Path, content: str):
    if path.parent.exists():
        path.write_text(content, encoding="utf-8")
        print(f"Delivered: {path.relative_to(REPO_ROOT)}")
    else:
        print(f"Skipped (dir not found): {path}")


def main():
    cpp = generate_cpp()
    py = generate_python()
    md = generate_markdown()

    _write(OUTPUT_DIR / "udp_protocol.h", cpp)
    _write(OUTPUT_DIR / "udp_protocol.py", py)
    _write(DOCS_PATH, md)

    _deliver(FIRMWARE_DIR / "udp_protocol.h", cpp)
    _deliver(JETSON_TOOLS_DIR / "udp_protocol.py", py)

    # Self-report sizes — handy sanity check at generation time.
    print("\nMessage payload sizes:")
    for msg in MESSAGES:
        tag = " (fixed head)" if msg.name in VARIABLE_TAIL else ""
        print(f"  {msg.name:14s} type=0x{dict((m,v) for m,v,_ in ENUMS['MsgType'])[msg.msg_type]:02X} "
              f"{msg.payload_size:4d} B  frame={8 + msg.payload_size + 2:4d} B{tag}")


if __name__ == "__main__":
    main()
