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
    ("PROTOCOL_VERSION", 5,      "u8",  "Bumped on any incompatible wire change (4→5: 2026-07-31 Profile gains the 3rd CAN slot can3_* — cone traffic)"),
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
    # HeartbeatT2J.flags bit offset of the cone (physical CAN2) BusHealth value
    # (HeartbeatT2JFlags::CONE_HEALTH_MASK, bits 4-5). Same no-bump packing as the
    # torque-clamp mask above; BusHealth::UNKNOWN == 0, so a pre-cone-uplink
    # firmware (which never sets these bits) self-describes as UNKNOWN.
    ("HEARTBEAT_CONE_HEALTH_SHIFT", 4, "u8",
     "Bit offset of CONE_HEALTH_MASK inside HeartbeatT2J.flags (bits 4-5)"),
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
    "HandSensorFlags": "u8",
    "ClockDiagFlags": "u8",
    "RingDiagFlags": "u8",
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
        ("HAND_SENSOR",    0x8B, "Hand ball-present sensor state (STREAM, T→J)"),
        ("CAN_ERRORS",     0x8C, "1 Hz CAN3 wire-error + fault-confinement counters (STREAM, T→J)"),
        ("BRIDGE_TX_DIAG", 0x8D, "1 Hz per-bus CAN TX deferral/queue pressure + per-stage hand-send attribution (STREAM, T→J)"),
        ("BRIDGE_IDENTITY", 0x8E, "1 Hz can-bridge firmware identity: FW_VERSION + PROTOCOL_VERSION echo (STREAM, T→J)"),
        # 0x8F is the LAST free uplink id below RPC_RESPONSE (0x90). Anything
        # after this needs a new id block, NOT a renumber of the ids above.
        ("CLOCK_DIAG",     0x8F, "Per-accepted-TOD-anchor wall-clock discipline sample + 500 Hz interp occupancy (STREAM, T→J)"),
        ("RPC_RESPONSE",   0x90, "RPC response (RPC port, T→J)"),
        # ── Uplink id block 2 — ABOVE RpcResponse (0x91-0xFF) ─────────────────
        # The block below 0x90 is FULL. Per the note on 0x8F, a new uplink takes
        # a fresh id here rather than renumbering an existing one: an id change
        # silently re-points every already-recorded frame at another message
        # type, so a bag spanning the change decodes as garbage instead of
        # failing. Nothing routes on the numeric value or on any 0x90 boundary —
        # the STREAM/RPC split is which SOCKET a frame was sent on
        # (udp_link.cpp send_to / drain_socket), and both ends dispatch through a
        # msg_type table, never a range test — so an uplink id above
        # RPC_RESPONSE is ordinary. Verified 2026-08-12 across teensy_link/,
        # the firmware and the generated headers: no comparison against a
        # MsgType value exists anywhere.
        ("CACHE_DIAG",     0x91, "1 Hz encoder-cache freshness census (per-axis age floor/peak) + CAN RX-ring occupancy (STREAM, T→J)"),
        ("RING_DIAG",      0x92, "1 Hz CAN RX-ring TRUE-occupancy census (true_depth vs reported _available = the leak) + jugglebot delivery lag + SDO RTT (STREAM, T→J)"),
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
    # HandSensor.flags bitset — generated single source for the bits the firmware
    # producer (telemetry.cpp hand_sensor_uplink_step) sets and the Jetson bridge
    # reads, replacing prose that would otherwise live only in the field comment
    # (HeartbeatT2JFlags precedent).
    "HandSensorFlags": [
        ("RAW_HELD",       0x01, "bit0: raw per-sample bit (active-low decoded)"),
        ("DEBOUNCED_HELD", 0x02, "bit1: debounced verdict"),
        ("VALID",          0x04, "bit2: not UNKNOWN (fresh, gated good reply)"),
        ("STALE",          0x08, "bit3: no good reply within the staleness window"),
        ("TIME_SYNCED",    0x10, "bit4: bridge wall anchor set at the reply"),
    ],
    "ClockDiagFlags": [
        # ClockDiag.flags. Each bit answers a question a consumer MUST ask before
        # using a field, so none of them is cosmetic.
        ("STEPPED",      0x01, "bit0: this anchor STEPPED the offset (|err_us| > TIME_STEP_THRESHOLD_US, "
                               "or the first anchor) instead of slewing the IIR. A step means the measured "
                               "offset moved further in one interval than any crystal can explain "
                               "(>20 ms in 30 s = >666 ppm), i.e. boot, a re-acquisition after a link gap, "
                               "or a Jetson clock step — so freq_ppb over that interval is NOT a crystal "
                               "reading and FREQ_VALID is cleared with it"),
        ("FIRST_ANCHOR", 0x02, "bit1: the FIRST anchor since boot — there was no prior offset to difference "
                               "against, so err_us is reported as 0 because it does not EXIST, not because "
                               "it was zero"),
        ("FREQ_VALID",   0x04, "bit2: freq_ppb carries a real measurement. When CLEAR, freq_ppb is 0 and that "
                               "0 means NO SAMPLE — never 'zero frequency error'. The read site must branch "
                               "on this bit, which is why it is a positive assertion rather than an "
                               "invalid flag"),
    ],
    "RingDiagFlags": [
        # RingDiag.flags. Both bits exist for the same reason ClockDiag's
        # FREQ_VALID does: a lag field of 0 must never be readable as "no lag"
        # when what it actually means is "no measurement" or "the reference just
        # moved". Read them BEFORE lag_now_us / lag_hwm_us.
        ("LAG_SEEDED",           0x01, "bit0: the jugglebot arrival clock has been seeded (at least one "
                                       "frame has been folded since boot or since the last reseed), so "
                                       "lag_now_us / lag_hwm_us carry a real measurement. When CLEAR they "
                                       "are 0 because no reference exists — not because the lag is zero"),
        ("LAG_RESEED_IN_WINDOW", 0x02, "bit1: the arrival clock was RESEEDED during this window (an "
                                       "inter-delivery gap exceeded the wrap-safety bound, so the 16-bit "
                                       "capture-stamp unwrap could not be trusted). A reseed sets the zero "
                                       "reference to NOW, so lag_now_us in this window is not comparable "
                                       "with the previous window's — the consumer must SEGMENT the series "
                                       "here rather than reading a step as a jump in delivery lag. "
                                       "lag_reseeds is the cumulative counterpart"),
    ],
    "HeartbeatT2JFlags": [
        ("TIME_SYNCED",               0x1, "bit0: Teensy clock synced to the Jetson anchor"),
        ("STOW_PENDING_ON_RECONNECT", 0x2, "bit1: deferred-stow latch armed (awaiting confirmed CAN3 reconnect)"),
        ("ALL_AXIS_HEARTBEATS_OK",    0x4, "bit2: every present axis heartbeat is fresh"),
        ("MPC_ACTIVE",                0x8, "bit3: firmware-side mpc_active (lets a setpoint source verify its arm took)"),
        # Cone (physical CAN2) bus health as a 2-bit BusHealth value in bits 4-5
        # (see HEARTBEAT_CONE_HEALTH_SHIFT). Not a single-bit flag: BusHealth has
        # four states, and UNKNOWN == 0 makes a stale flash self-describing.
        ("CONE_HEALTH_MASK",         0x30, "bits 4-5: cone (CAN2) BusHealth (UNKNOWN=0/OK=1/WARN=2/"
                                           "BUS_OFF=3) << HEARTBEAT_CONE_HEALTH_SHIFT; reads 0 = "
                                           "UNKNOWN from a pre-cone-uplink flash"),
        # Per-leg torque_ff ingest-clamp mask, packed into free bits of the same u32
        # (bits 6-7 stay reserved for future single-bit flags; the mask starts at
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
            Field("flags",       "u32", 1, "HeartbeatT2JFlags bitset: bits 0-3 TIME_SYNCED|STOW_PENDING_ON_RECONNECT|ALL_AXIS_HEARTBEATS_OK|MPC_ACTIVE; bits 4-5 CONE_HEALTH_MASK (cone/CAN2 BusHealth, see HEARTBEAT_CONE_HEALTH_SHIFT); bits 8-13 TORQUE_CLAMP_MASK (per-leg torque_ff ingest clamp, see HEARTBEAT_TORQUE_CLAMP_SHIFT)"),
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
            Field("can1_rx",      "u32", 1, "wire slot 1 = jugglebot role (physical CAN2 since 2026-07-31) frames received this window"),
            Field("can1_tx",      "u32", 1, "wire slot 1 = jugglebot role frames transmitted this window"),
            Field("can2_rx",      "u32", 1, "wire slot 2 = Ball Butler (physical CAN1) frames received this window"),
            Field("can2_tx",      "u32", 1, "wire slot 2 = Ball Butler (physical CAN1) frames transmitted this window"),
            Field("can1_util_x100", "u16", 1, "wire slot 1 = jugglebot role bus utilisation, pct*100"),
            Field("can2_util_x100", "u16", 1, "wire slot 2 = Ball Butler bus utilisation, pct*100"),
            Field("udp_rtt_us",   "u32", 1, "Last measured Jetson round-trip (us)"),
            Field("udp_jitter_us", "u32", 1, "RTT jitter estimate (us)"),
            Field("interp_deadline_misses", "u32", 1, "Cumulative 500 Hz deadline misses"),
            Field("interp_max_jitter_us", "u32", 1, "Worst interp tick jitter this window (us)"),
            Field("free_heap_bytes", "u32", 1, "FreeRTOS free heap (bytes)"),
            # Wire slot 3 (2026-07-31, PROTOCOL_VERSION 5): the cone role.
            # Appended at the END so every pre-existing field keeps its offset.
            Field("can3_rx",      "u32", 1, "wire slot 3 = cone role (physical CAN3 since 2026-07-31) frames received this window"),
            Field("can3_tx",      "u32", 1, "wire slot 3 = cone role frames transmitted this window"),
            Field("can3_util_x100", "u16", 1, "wire slot 3 = cone role bus utilisation, pct*100"),
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
            "from the Platform-Teensy byte layout (Teensy_code_platform.ino "
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
        "HandSensor", "HAND_SENSOR", "T2J", "STREAM",
        summary=(
            "Hand ball-present sensor state. A switch in the hand shorts the hand "
            "ODrive Pro's G02 to GND when a ball is seated; no released ODrive "
            "firmware pushes GPIO state on CANSimple, so the bridge POLLS "
            "get_gpio_states over an RxSdo/TxSdo pair (gpio_poll.cpp) and publishes "
            "the decoded cache here. Additive message — no existing frame changes, "
            "so NO PROTOCOL_VERSION bump (LegCmd precedent); an old Jetson ignores "
            "the unknown msg_type and a new Jetson treats never-seen as UNKNOWN. "
            "Emitted from task_telem: one frame per NEW good reply (so naturally "
            "rate-limited to the poll rate), plus a 1 Hz keepalive while no new "
            "reply lands, so staleness is itself observable on the wire. "
            "plans/archived/2026-08-15 hand-ball-sensor.md § Architecture is NORMATIVE for the "
            "signal semantics; these flags describe the bridge's cache and say "
            "nothing about the link, so the consumer applies its own RX-age gate."),
        fields=[
            Field("t_bridge_us", "u64", 1, "Bridge WALL-clock (now_wall_us()) at the last good TxSdo reply (us)"),
            Field("raw_states",  "u32", 1, "Last raw get_gpio_states word, verbatim (commissioning + diagnostics)"),
            Field("flags",       "u8",  1, "HandSensorFlags bitset (generated enum is the single source)"),
            Field("miss_count",  "u8",  1, "Consecutive EMPTY readings from good replies (saturating)"),
        ],
    ),
    Message(
        "CanErrors", "CAN_ERRORS", "T2J", "STREAM",
        summary=(
            "CAN3 wire-error + fault-confinement counters, 1 Hz. These numbers were "
            "already computed on every 1 kHz service tick (can_buses.cpp "
            "poll_bus_errors / service_bus) and then DISCARDED to the USB serial "
            "console — can_buses.h said outright that they are 'NOT on the UDP "
            "uplink'. That is exactly why the 2026-07-29 CAN3 bus-health flap could "
            "not be root-caused from an 8 MB bag: the bag proved the bus was entering "
            "error-passive at 42.4 % duty but carried nothing that could say WHICH "
            "wire-error class was driving it, so layer 2 of that investigation "
            "(logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md) stayed "
            "open pending a serial-console bench session. This message closes that "
            "gap: it is the Jetson-side half of the discriminator table in that "
            "entry. Additive — no existing frame changes, so NO PROTOCOL_VERSION bump "
            "(the LegCmd / HandSensor precedent): an old Jetson ignores the unknown "
            "msg_type and a new Jetson renders never-seen as unknown. CAN3 "
            "(the Jugglebot core bus) ONLY — CAN1/CAN2 keep their serial-only "
            "counters until something needs them, and a per-bus array would triple "
            "the wire cost for two buses nobody is currently debugging. All counters "
            "are CUMULATIVE SINCE BOOT (u32, free-running; the consumer differences "
            "them) except the three live/derived bytes at the end. Read tec_inc_sum "
            "vs rec_inc_sum to attribute error pressure to TX vs RX even when the "
            "live counters decay between 1 Hz samples."),
        fields=[
            Field("ack_cnt",       "u32", 1, "ACKERR snapshots — TX un-ACKed (no other node received it)"),
            Field("crc_cnt",       "u32", 1, "CRCERR snapshots — RX CRC mismatch (noise / signal integrity)"),
            Field("form_cnt",      "u32", 1, "FRMERR snapshots — RX fixed-form field violated"),
            Field("stuff_cnt",     "u32", 1, "STFERR snapshots — RX >5 equal bits (noise / clocking)"),
            Field("bit0_cnt",      "u32", 1, "BIT0ERR snapshots — TX sent dominant, read back recessive"),
            Field("bit1_cnt",      "u32", 1, "BIT1ERR snapshots — TX sent recessive, read back dominant"),
            Field("err_tx_ctx",    "u32", 1, "Wire-error snapshots with ESR1.TX set (error fired while transmitting)"),
            Field("err_rx_ctx",    "u32", 1, "Wire-error snapshots with ESR1.RX set (error fired while receiving)"),
            Field("tec_inc_sum",   "u32", 1, "Sum of positive inter-tick TEC deltas — TX-side error PRESSURE (+8/error)"),
            Field("rec_inc_sum",   "u32", 1, "Sum of positive inter-tick REC deltas — RX-side error pressure (+1/error)"),
            Field("tx_gated",      "u32", 1, "TX attempts refused by the bus-partner presence gate"),
            Field("tec_live",      "u8",  1, "ECR TXERRCNT at the last service tick (>=128 => error-passive)"),
            Field("rec_live",      "u8",  1, "ECR RXERRCNT at the last service tick"),
            Field("flt_live",      "u8",  1, "Live ESR1 FLTCONF: 0 active / 1 passive / 2 bus-off (INSTANTANEOUS)"),
            Field("flt_sustained", "u8",  1, "1 => flt_live>=1 held >= CAN_PASSIVE_SUSTAIN_US, i.e. the command "
                                             "gate is actually refusing (classify_command_gate). The one bit that "
                                             "distinguishes a harmless transient from a real command outage."),
        ],
    ),
    Message(
        "BridgeTxDiag", "BRIDGE_TX_DIAG", "T2J", "STREAM",
        summary=(
            "CAN TX-path pressure per bus, plus per-stage attribution of the "
            "HAND_TRAJ_CMD conduit's exits, 1 Hz. Built for the 2026-08-01 "
            "ERR_TIMEOUT recount, which could establish THAT the hand arm-ack "
            "fails about half the time (139 of 266 arm dispatches pooled across "
            "16 sessions) but not WHICH of hand_ops' three CAN sends refused, "
            "nor whether a refusal meant a lost frame. "
            "tx_deferred is named for what it measures, and the name is "
            "load-bearing: FlexCAN_T4::write(const CAN_message_t&) returns 1 or "
            "-1 and NEVER 0, and -1 means no TX mailbox was free so the frame was "
            "pushed into the 64-slot software txBuffer that the TX-complete ISR "
            "drains. A refused send is therefore a DEFERRAL of ~0.1-1 ms, not a "
            "drop — which is what makes catch_coordinator's 'the ack lies, frames "
            "were observed transmitted after a failed ack' premise and hand_ops' "
            "ERR_TIMEOUT compatible rather than contradictory. The two paths that "
            "genuinely LOSE a frame are (a) txBuffer overflow, where a 65th "
            "pending entry silently overwrites the oldest, and (b) the vendored "
            "events() TX drain, which writes one peeked frame into every free "
            "mailbox while popping one queue entry per mailbox; tx_q_hwm "
            "approaching 64 is the observable for (a). ALL THREE buses carry both "
            "fields — a deliberate contrast with CanErrors' CAN3-only choice, "
            "whose per-bus cost was 15 fields against these 2. hand_* attribute "
            "every invocation to its exit, so the success count is derivable: "
            "OK = hand_calls - hand_rej_homing - hand_bus_down - hand_pre1_fail - "
            "hand_pre2_fail - hand_traj_fail. All counters are CUMULATIVE SINCE "
            "BOOT (the consumer differences them) except tx_q_hwm_*, which are "
            "high-water marks. Unconditional 1 Hz from task_telem rather than "
            "on-change, for the same reason as CanErrors: an operator "
            "differencing an A/B needs a continuous baseline, and 'silence means "
            "healthy' is exactly the ambiguity that cost the 2026-07-29 "
            "investigation a session. Additive — no existing frame changes, so NO "
            "PROTOCOL_VERSION bump (the LegCmd / HandSensor precedent): an old "
            "Jetson ignores the unknown msg_type and a new Jetson renders "
            "never-seen as unknown."),
        fields=[
            Field("tx_deferred_jb",   "u32", 1, "Jugglebot bus: sends whose write() returned -1 (mailbox full → "
                                                "queued to the software txBuffer). NOT a drop — see the summary"),
            Field("tx_deferred_bb",   "u32", 1, "Ball Butler bus: same deferral count"),
            Field("tx_deferred_cone", "u32", 1, "Cone bus: same deferral count"),
            Field("tx_q_hwm_jb",      "u16", 1, "Jugglebot bus: peak software txBuffer occupancy, sampled at SEND "
                                                "instants inside the send critical section (max 64; at/near 64 ⇒ "
                                                "overwrite-loss occurred or is imminent)"),
            Field("tx_q_hwm_bb",      "u16", 1, "Ball Butler bus: same high-water mark"),
            Field("tx_q_hwm_cone",    "u16", 1, "Cone bus: same high-water mark"),
            Field("hand_calls",       "u32", 1, "hand_traj_cmd invocations, counted at ENTRY before any gate "
                                                "(the denominator the other hand_* fields subtract from)"),
            Field("hand_rej_homing",  "u32", 1, "Exits with ERR_REJECTED: the homing interlock refused"),
            Field("hand_bus_down",    "u32", 1, "Exits with ERR_BUS_DOWN: jugglebot_commands_allowed() refused"),
            Field("hand_pre1_fail",   "u32", 1, "Exits with ERR_TIMEOUT at send #1 (set_state CLOSED_LOOP)"),
            Field("hand_pre2_fail",   "u32", 1, "Exits with ERR_TIMEOUT at send #2 (set_controller_mode)"),
            Field("hand_traj_fail",   "u32", 1, "Exits with ERR_TIMEOUT at send #3 (the 0x6D0 traj frame)"),
        ],
    ),
    Message(
        "BridgeIdentity", "BRIDGE_IDENTITY", "T2J", "STREAM",
        summary=(
            "Can-bridge firmware identity, 1 Hz. FW_VERSION existed only in the "
            "USB serial boot banner, so a Jetson session could not tell WHICH "
            "firmware answered it — the same silent-skew defect the Platform "
            "Teensy's identity block closed on 2026-07-27 "
            "(ros_ws/docs/platform_fw_version.md). fw_version is the ACTIONABLE "
            "field: the host compares it against "
            "teensy_link.rpc_args.EXPECTED_BRIDGE_FW_VERSION and logs "
            "BRIDGE_FW_CHECK on a skew — reported, never enforced. "
            "NOTE on protocol_version: it is self-description, NOT skew "
            "detection. A PROTOCOL_VERSION mismatch makes decode_frame reject "
            "EVERY frame in both directions (the 24608bb total-darkness failure), "
            "including this one, so this field can never report the mismatch it "
            "appears to be about; it documents what the running build was "
            "compiled against once the link decodes at all. Additive — no "
            "existing frame changes, so NO PROTOCOL_VERSION bump (the LegCmd / "
            "HandSensor precedent): an old Jetson ignores the unknown msg_type "
            "and a new Jetson renders never-seen as unknown."),
        fields=[
            Field("fw_version",       "u16", 1, "canbridge_config.h FW_VERSION of the running build"),
            Field("protocol_version", "u8",  1, "PROTOCOL_VERSION the running build was compiled against "
                                                "(self-description — a mismatch makes this frame undecodable)"),
        ],
    ),
    Message(
        "ClockDiag", "CLOCK_DIAG", "T2J", "STREAM",
        summary=(
            "ONE SAMPLE PER ACCEPTED TIME-OF-DAY ANCHOR (~30 s steady state, "
            "500 ms during the pre-first-anchor fast retry), carrying the "
            "wall-clock discipline state that time_base.cpp::set_wall_anchor "
            "has always computed and then DISCARDED, plus the 500 Hz interp "
            "ladder's fallback-mode occupancy over the window since the "
            "previous emit. Built for 'plans/archived/2026-08-15 "
            "bridge-temporal-trustworthiness.md' P1 (arc closed 2026-08-15, "
            "logbook/2026-08-15-fw14-validated-arc-closed.md); it is the raw "
            "material the clock plan's "
            "Phase 1 needs before the servo in "
            "plans/active/bridge-clock-frequency-discipline.md is touched — "
            "the ACTUAL crystal ppm, its thermal coefficient, and the "
            "RTT-jitter floor that sets the achievable Kp/Ki — and the "
            "occupancy half closes the two remaining telemetry gaps named in "
            "the Addendum to logbook/2026-07-18-teensy-uptime-tracking-"
            "degradation.md (recover-slew and extrapolation-mode occupancy "
            "were not uplinked at all). "
            "WHY THE TWO HALVES SHARE A FRAME: they are read together. The "
            "question is whether a session's growing command lag is the "
            "transport (rtt_us climbing), the clock (err_us / freq_ppb "
            "wandering) or the interp ladder falling back to extrapolation — "
            "and a single frame makes those three answers simultaneous by "
            "construction rather than by a join across two cadences. "
            "SELF-CONTAINED BY DESIGN: every derived quantity here can be "
            "re-derived by the consumer from the raw fields on the same "
            "sample (measured offset = jetson_wall_us - t_local_us; "
            "freq_ppb = 1e9 * delta(measured offset) / dt_local_us), so a "
            "firmware arithmetic bug is falsifiable from a bag instead of "
            "having to be trusted. "
            "THE X-AXIS IS t_local_us (micros64, the raw crystal), NOT a wall "
            "stamp: the wall clock is the quantity under measurement and it "
            "STEPS, so stamping these samples with now_wall_us() would fold "
            "the measurand into its own time base. That is also why the frame "
            "needs no header timestamp from the host. "
            "Additive — no existing frame changes, so NO PROTOCOL_VERSION "
            "bump (the LegCmd / HandSensor / BridgeTxDiag precedent): an old "
            "Jetson ignores the unknown msg_type and a new Jetson renders "
            "never-seen as unknown. An FW 10 board never sends this frame, so "
            "its consumer surface must read EMPTY, never error."),
        fields=[
            Field("t_local_us",    "u64", 1,
                  "micros64() captured inside set_wall_anchor at this anchor. The pure-crystal "
                  "monotonic x-axis for every fit made from these samples — see the summary. "
                  "Also the epoch reference: measured offset = jetson_wall_us - t_local_us"),
            Field("jetson_wall_us", "u64", 1,
                  "The anchor value ACTUALLY APPLIED, i.e. the Jetson's stamp already advanced by "
                  "rtt/2 (time_sync_master.cpp::on_tod_response). Carried raw rather than as a "
                  "pre-computed offset so (a) the consumer derives the measured offset itself and "
                  "(b) the anchor can be cross-referenced against the Jetson's own bag clock, "
                  "which is what makes the one-way-delay asymmetry of the plan's coupling section "
                  "checkable at all"),
            Field("dt_local_us",   "u32", 1,
                  "micros64() elapsed since the PREVIOUS accepted anchor (0 on the first). The "
                  "denominator of freq_ppb, carried so the estimate is re-derivable AND so its "
                  "precision is visible: measurement noise is ~RTT-jitter, so implied-ppb noise "
                  "scales as 1/dt and a 500 ms fast-retry interval is orders of magnitude noisier "
                  "than a 30 s steady-state one. Consumers filter on this. Saturates at UINT32_MAX "
                  "(~71.6 min), and a saturating interval also clears FREQ_VALID"),
            Field("rtt_us",        "u32", 1,
                  "Round-trip of the TOD exchange that produced this anchor (micros64() at receipt "
                  "minus at send). THE shared discriminator of the arc: RTT growing with bridge "
                  "uptime implicates the UDP transport for the command-latency half AND invalidates "
                  "the rtt/2 symmetry assumption this very anchor rests on. Also the input to the "
                  "min-RTT anchor gating the clock plan's Phase 4 adds — the least-queued "
                  "round-trip is the most symmetric one"),
            Field("err_us",        "i32", 1,
                  "The offset error BEFORE this anchor was applied: (measured offset) - (offset the "
                  "servo was already holding). This is the `diff` set_wall_anchor computes on every "
                  "anchor and throws away, and it is the phase error a type-2 servo would act on. "
                  "i32 is provably wide enough on the branch that matters: the slew branch is "
                  "entered only when |diff| <= TIME_STEP_THRESHOLD_US (20 ms). On a STEP it "
                  "saturates at INT32_MIN/MAX (+-~35 min) — read STEPPED before reading a large "
                  "value. 0 with FIRST_ANCHOR set means no prior offset existed"),
            Field("freq_ppb",      "i32", 1,
                  "Implied fractional frequency error of the bridge crystal against the Jetson over "
                  "the interval since the previous anchor, in parts per BILLION: "
                  "1e9 * (measured offset now - measured offset then) / dt_local_us. Computed from "
                  "consecutive MEASUREMENTS, not from the servo's slewed state, so it is open-loop "
                  "with respect to the IIR and reports the crystal rather than the filter's "
                  "response. ppb not ppm because the interesting range is single-digit-to-tens of "
                  "ppm and a ppm integer would quantise the whole signal away. VALID ONLY when "
                  "FREQ_VALID is set; 0 otherwise, and that 0 is NO SAMPLE (see ClockDiagFlags)"),
            Field("anchor_seq",    "u32", 1,
                  "Count of accepted anchors since boot (1 on the first). This frame is emitted "
                  "once per anchor with no retransmission on a lossy transport, so without a "
                  "sequence a silently dropped sample would corrupt every interval-based "
                  "conclusion drawn downstream while looking like clean data. It is also the "
                  "de-duplication key for any consumer that re-renders the latest sample"),
            Field("interp_ticks",  "u32", 1,
                  "500 Hz interp ISR ticks in the window since the previous CLOCK_DIAG emit. THE "
                  "DENOMINATOR: the two occupancy counters below are duty cycles, and the window "
                  "length is NOT a constant (30 s steady state, 500 ms during fast retry, wider "
                  "still if a frame was lost), so a raw count without this is uninterpretable. "
                  "Doubles as a tick-loss detector — it should equal 500 * window_seconds, and a "
                  "shortfall is ISR starvation that interp_deadline_misses on the PROFILE frame "
                  "counts a different way. Counts EVERY tick including the ones that return early "
                  "(deferred stow, or before the first setpoint is latched); on an armed session "
                  "streaming setpoints those are zero"),
            Field("recover_slew_ticks", "u32", 1,
                  "Ticks in this window in which the re-enable recovery slew (leg_interp.cpp "
                  "s_recover_slewing, the 2026-07-11 clear-errors-jolt ramp) actually OVERRODE the "
                  "streamed command. Counted at the override, not at the flag, so a slew that is "
                  "armed but held (the cold-start carve-out, where the ramp is pinned to the live "
                  "encoder and skipped) does not inflate the number — the question the counter "
                  "exists to answer is how often the emitted command came from the ramp instead of "
                  "the trajectory"),
            Field("extrap_ticks",  "u32", 1,
                  "Ticks in this window that took the Mode-2 cubic-Taylor EXTRAPOLATION branch, "
                  "i.e. no u1 knot was available and the ladder ran open-loop off the last one. "
                  "This is the discriminator for 'the response shape also slows': a session whose "
                  "extrapolation occupancy rises is one whose setpoint stream is arriving late or "
                  "sparse at the Teensy, which is a different fault from the same lag appearing "
                  "with the ladder fully fed"),
            Field("flags",         "u8",  1,
                  "ClockDiagFlags bitfield: STEPPED / FIRST_ANCHOR / FREQ_VALID. Read this BEFORE "
                  "err_us or freq_ppb"),
        ],
    ),
    Message(
        "CacheDiag", "CACHE_DIAG", "T2J", "STREAM",
        summary=(
            "ENCODER-CACHE FRESHNESS CENSUS, 1 Hz. The confirmation instrument "
            "for the surviving question of "
            "logbook/2026-07-18-teensy-uptime-tracking-degradation.md. "
            "The S1 experiment (2026-08-12) localized the uptime command-latency "
            "drift to the Teensy: at 63 h of bridge uptime the end-to-end leg lag "
            "is 290-340 ms with the leg_interp lead clamp pinning the executed "
            "command at fb+MAX_LEAD_REV for 44-74 % of ticks (5.8 % on a fresh "
            "boot), while udp_rtt_us is flat at 1-3 ms, interp_deadline_misses is "
            "0, the heap is flat and CAN throughput is flat. Setpoints therefore "
            "ARRIVE on time and the ladder EXECUTES on time — so either the leg "
            "genuinely trails, or the `fb` the lead clamp measures against "
            "(axes[i].pos_rev, written by can_buses.cpp decode_into_cache on each "
            "ODrive get_encoder_estimate frame) is STALE and the clamp is pinning "
            "the command to a position the leg left hundreds of milliseconds ago. "
            "TODAY'S TELEMETRY CANNOT TELL THOSE APART: /robot_state and "
            "/leg_cmd_executed both read that same cache, so a stale cache moves "
            "both together and looks exactly like a real lag. This frame measures "
            "the cache's freshness DIRECTLY, which is the one observable that "
            "separates them. "
            "WHAT NOMINAL LOOKS LIKE: the ODrive broadcasts get_encoder_estimate "
            "per axis continuously, so age_min_us should sit near 0 and "
            "age_max_us near one broadcast period (a few ms) FOREVER, at any "
            "uptime. age_min_us is the headline: it is a FLOOR, and a floor "
            "cannot be produced by jitter, scheduling or sampling luck - only by "
            "the cache actually not being written. A floor that grows with uptime "
            "confirms the stale-cache mechanism; a floor that stays at the "
            "broadcast period while the lag grows REFUTES it and sends the hunt "
            "to the ODrive's own position loop. "
            "SAMPLED FROM TASK CONTEXT, NOT THE ISR (telemetry.cpp "
            "cache_diag_uplink_step, on task_telem at TELEM_RATE_HZ): the age is "
            "read through axis_state.h's existing snapshot_pos_vel seqlock, the "
            "same reader send_telemetry already uses for this triple at the same "
            "rate, so the census adds ZERO work to the 500 Hz interp ISR and "
            "opens no new IRQ-off window. Every accumulator behind this frame is "
            "written and read by that one task, so there is no cross-context "
            "counter to get wrong. "
            "WHY A PER-AXIS FRAME COUNTER SITS BESIDE THE AGES (enc_frames, added "
            "2026-08-12). The S1 bag forensics found the per-axis cache VALUE "
            "stalling for 30-500 ms in a fat tail — 9-18 % of refresh intervals "
            "> 30 ms on an aged bridge against 4.3 % fresh — while every "
            "AGGREGATE CAN RX counter stayed flat and the uplink cadence stayed "
            "perfect. That is not a contradiction: an aggregate cannot see ONE "
            "axis of seven go quiet, because the other six keep the total moving. "
            "So the aggregates could observe the stall's consequence and never "
            "its cause, and one question stayed open — did the ODrive pause "
            "broadcasting that axis, or did the frame arrive and the cache not "
            "update? enc_frames answers it directly, and the two answers have "
            "different owners. "
            "The RX-ring fields are the other half: depth_hwm and cap_hits have "
            "been computed on every 1 kHz service tick since the 2026-06-04 "
            "drain-to-empty fix (can_buses.cpp service_bus, CAN_RX_DRAIN_BUDGET) "
            "and were NEVER uplinked — CanErrors 0x8C deliberately carries only "
            "wire-error and fault-confinement fields. They are the direct witness "
            "of the one mechanism that could starve the cache from inside the "
            "bridge, and cap_hits is the documented overflow PRECURSOR that must "
            "stay 0. "
            "Additive — no existing frame changes, so NO PROTOCOL_VERSION bump "
            "(the LegCmd / HandSensor / BridgeTxDiag / ClockDiag precedent): an "
            "old Jetson ignores the unknown msg_type and a new Jetson renders "
            "never-seen as unknown. An FW <= 11 board never sends this frame, so "
            "its consumer surface must read EMPTY, never error. "
            "INSTRUMENTATION ONLY: nothing in the firmware reads any field or "
            "accumulator introduced for this frame, so a wrong value here cannot "
            "move a leg."),
        fields=[
            Field("t_local_us",     "u64", 1,
                  "micros64() at the emit instant — the pure-crystal monotonic clock, never the "
                  "steppable wall (time_base.h's clock-discipline invariant). This IS the bridge's "
                  "power-on uptime, which is the independent variable of the whole investigation, so "
                  "carrying it makes every sample self-locating without a join against /link_status "
                  "uptime_ms. Also the x-axis every age fit is plotted against"),
            Field("age_min_us",     "u32", 7,
                  "THE HEADLINE. Per-axis MINIMUM encoder-cache age over the window just closed, in "
                  "microseconds: min over the window of (micros64() at sample - axes[i].pos_timestamp_us). "
                  "Indices 0-5 are the legs, 6 is the HAND (the hand path shares this cache, and its "
                  "dispatch shift tracked the same uptime curve — 2026-07-28 anomaly-fixes sitting). "
                  "A MINIMUM rather than an instantaneous read because the quantity under test is a "
                  "FLOOR: between two encoder frames the age ramps 0 -> one broadcast period, so any "
                  "single read is a uniform sample of that ramp, whereas the minimum over ~100 reads "
                  "is the floor itself and cannot be explained by sampling luck. Nominal is near 0; "
                  "a floor of hundreds of ms is a cache that is not being written. Saturates at "
                  "UINT32_MAX (~71.6 min) rather than wrapping — a wrapped age would read as a small, "
                  "healthy number, which is the one failure this frame must never produce. An axis "
                  "whose bit is clear in seen_mask has never been cached at all and reads the "
                  "saturation rail; read seen_mask FIRST"),
            Field("age_max_us",     "u32", 7,
                  "Per-axis MAXIMUM cache age over the same window, same units and same saturation. "
                  "The burst detector, and the pair that makes the mechanism legible: min and max both "
                  "rising = the cache has a growing floor (staleness); min flat at ~0 with max rising = "
                  "the broadcast has GAPS but recovers, a different fault with a different owner. "
                  "It also recovers the per-axis frame RATE without touching the CAN RX decode path: "
                  "with the cache written every broadcast, max is one broadcast period, so a rate "
                  "collapse shows up here — though enc_frames below is the direct, unambiguous "
                  "witness of that and is what a stall analysis should read"),
            Field("enc_frames",     "u32", 7,
                  "Per-axis get_encoder_estimate frames DECODED AND CACHED, CUMULATIVE SINCE BOOT "
                  "(the consumer differences two samples — the BridgeTxDiag census idiom). Same "
                  "indexing as the age arrays: 0-5 legs, 6 hand. "
                  "THE SPLIT THIS EXISTS TO MAKE. The 2026-08-12 S1 bag forensics found the per-axis "
                  "cache VALUE stalling for 30-500 ms in a fat tail (9-18 % of refresh intervals "
                  "> 30 ms on an aged bridge against 4.3 % fresh) while every AGGREGATE CAN RX "
                  "counter stayed flat and the uplink cadence stayed perfect. An aggregate CANNOT "
                  "see one axis of seven go quiet — the other six keep the total moving — which is "
                  "why the existing counters could observe the stall's consequence and never its "
                  "cause. Read against a stall window: the counter STILL ADVANCING means frames "
                  "arrived and the decode ran, so the frozen value came in over the wire (an ODrive "
                  "sending a stale estimate) rather than being lost on the way in; the counter "
                  "PAUSED means nothing arrived for that axis, and the fault is upstream — the "
                  "ODrive's broadcast scheduler or a per-axis loss on the bus. Those are different "
                  "faults with different owners and no other field separates them. "
                  "Incremented AFTER write_pos_vel() in can_buses.cpp decode_into_cache, so the "
                  "invariant is one-directional and load-bearing: enc_frames[i] advanced ⇒ that "
                  "axis's cache write COMPLETED. A stalled value against an advancing counter "
                  "therefore places the fault at or above that line, never below it. "
                  "Unlike the ages, 0 here is a MEASUREMENT (that axis received nothing this "
                  "window), not a missing one — so the seen_mask 'n/a' discipline deliberately does "
                  "NOT extend to this field. u32 wraps at ~4.3e9 frames, i.e. ~182 days at the "
                  "~272 frames/s/axis broadcast rate, and unsigned differencing is wrap-correct"),
            Field("seq",            "u32", 1,
                  "Emitted-frame counter since boot (1 on the first). This frame is one-shot per "
                  "window on a lossy transport with no retransmission, so without a sequence a "
                  "silently dropped sample would join two windows into one and inflate an age_max "
                  "while looking like clean data. Also the de-duplication key for a latest-value "
                  "consumer"),
            Field("window_us",      "u32", 1,
                  "micros64() elapsed since the previous emit (0 on the FIRST frame, where no window "
                  "exists). The honest window length: nominal 1 000 000, and a longer one means "
                  "task_telem was delayed — which would itself be a finding on a box whose command "
                  "latency is under investigation. Saturates at UINT32_MAX (~71.6 min)"),
            Field("rx_cap_hits_jb", "u32", 1,
                  "Jugglebot bus (ROLE, not controller number — the jugglebot/cone controllers are "
                  "swapped in the current operating config, can_buses.cpp): service ticks on which the "
                  "per-tick CAN_RX_DRAIN_BUDGET (32) bound with frames STILL QUEUED. can_buses.h calls "
                  "this the overflow PRECURSOR and says it must stay 0. This is the residual-pressure "
                  "witness for the FlexCAN_T4 one-frame-per-events() staleness bug fixed on 2026-06-04 "
                  "by the drain-to-empty loop: nonzero means the fix's budget is now the binding "
                  "constraint, and a value that GROWS with uptime is a bridge-internal path to a stale "
                  "cache. CUMULATIVE SINCE BOOT — the consumer differences two samples"),
            Field("rx_cap_hits_bb",   "u32", 1, "Ball Butler bus: same budget-bound tick count, cumulative"),
            Field("rx_cap_hits_cone", "u32", 1, "Cone bus: same budget-bound tick count, cumulative"),
            Field("decode_short",     "u32", 1,
                  "Jugglebot-bus frames that ARRIVED and were then DISCARDED by the decode because the "
                  "DLC was short (< 8). Carried because it is the only counter that distinguishes "
                  "'no encoder data arriving' from 'encoder data arriving and being thrown away' — the "
                  "second is a stale-cache mechanism that every other field here would render as "
                  "silence. Computed since the 2026-07-05 marginal-CAN3 work, never uplinked. "
                  "CUMULATIVE SINCE BOOT"),
            Field("decode_bad_axis",  "u32", 1,
                  "Same, for frames whose decoded node id was >= NUM_AXES. Cumulative since boot"),
            Field("rx_depth_hwm_jb",  "u16", 1,
                  "Jugglebot bus: peak rxBuffer occupancy observed at a service tick since boot (cap "
                  "256). A HIGH-WATER MARK, not a counter — it never decreases, so it is read as a "
                  "level and not differenced. can_buses.h: single digits in health, climbing toward "
                  "256 means task_can_rx is being starved. Paired with cap_hits it separates 'the "
                  "backlog got deep' from 'the drain gave up with work left'"),
            Field("rx_depth_hwm_bb",   "u16", 1, "Ball Butler bus: same high-water mark"),
            Field("rx_depth_hwm_cone", "u16", 1, "Cone bus: same high-water mark"),
            Field("samples",           "u16", 1,
                  "Telemetry ticks in this window — nominally TELEM_RATE_HZ (100 in the production "
                  "build, 250 in BENCH_SYSID_BUILD), and 1 on the first frame, which closes a "
                  "zero-length window. An UPPER BOUND on any axis's fold count: an axis first seen "
                  "mid-window, or never seen (its rail-painted extrema), folded fewer or zero. "
                  "The denominator that makes the extrema interpretable — a min over 100 samples is a "
                  "floor, a min over 2 is an anecdote — and an independent task_telem starvation "
                  "detector: samples well below window_us x TELEM_RATE_HZ / 1e6 means the telemetry "
                  "task itself was late, which would bias the extrema toward the ramp's low end. "
                  "Saturates rather than wrapping"),
            Field("seen_mask",         "u8",  1,
                  "Bit i set = axis i's pos_timestamp_us was nonzero during this window, i.e. an "
                  "encoder frame has EVER been cached for it. READ THIS BEFORE THE AGES: an absent "
                  "axis (the single-leg bench rig, a dark hand ODrive) has age_min == age_max == the "
                  "saturation rail, which is honest but indistinguishable from a catastrophically "
                  "stale cache. One byte closes that ambiguity at the source instead of asking every "
                  "consumer to cross-reference the heartbeat mask on another frame"),
        ],
    ),
    Message(
        "RingDiag", "RING_DIAG", "T2J", "STREAM",
        summary=(
            "CAN RX-RING TRUE-OCCUPANCY CENSUS, 1 Hz. The CONVICTION INSTRUMENT for the "
            "FlexCAN_T4 `_available` leak — the surviving candidate mechanism of the "
            "bridge-temporal arc after S2 (2026-08-13) killed the cache-AGE hypothesis. "
            "THE DEFECT (assembly-verified 2026-08-14, recorded in "
            "Teensy_code_canbridge/lib/FlexCAN_T4/PROVENANCE.md): FlexCAN_T4::events() pops "
            "the RX ring BEFORE its NVIC_DISABLE_IRQ guard, and Circular_Buffer's "
            "`_available` is read-modify-written non-atomically by BOTH the CAN ISR "
            "(increment, on push) and that unguarded task-side pop (decrement). The race is "
            "ONE-DIRECTIONAL — the ISR preempts the task, never the reverse — so ISR "
            "increments get swallowed and `_available` monotonically UNDER-counts. The "
            "bridge's drain loop (can_buses.cpp service_bus: do { events(); } while "
            "(++n < 32 && rx_remaining)) exits when `_available` reads 0 while the TRUE "
            "occupancy is still D > 0; from then on every frame it delivers is D frames "
            "old. D ratchets with uptime and caps at one ring depth, 256 slots ~= 114-135 ms "
            "at jugglebot-bus rates — which is the right order of magnitude for the "
            "290-340 ms end-to-end lag S1 measured at 63 h, and for the ~100-150 ms "
            "per-axis telemetry freezes S2 measured directly. "
            "WHY THIS FRAME HAS TO EXIST AT ALL: `getRXQueueCount()` returns `_available`, "
            "so the depth_hwm and cap_hits counters the bridge already keeps are computed "
            "from the very number the race corrupts. They are blind to this failure BY "
            "CONSTRUCTION and would read perfectly healthy through a fully-leaked ring. "
            "THE SINGLE NUMBER THAT CONVICTS is `true_depth_jb - avail_reported_jb`, "
            "sampled at the same instant, immediately AFTER the drain loop has run to "
            "completion: at that moment `_available` is 0 by definition (that is why the "
            "loop exited), so the residual true depth IS the leak. Nominal is 0 forever, at "
            "any uptime. Anything else, growing with uptime, is the mechanism. "
            "leak_hwm_* is the same quantity maximised over EVERY 1 kHz service tick since "
            "boot, so the verdict does not depend on the 1 Hz sample landing luckily. "
            "THE TWO CROSS-CHECKS, both on the jugglebot bus, both causal rather than "
            "correlational: (1) lag_now_us measures the delivery lag DIRECTLY from the "
            "FlexCAN hardware capture timestamp, which is stamped at frame reception and is "
            "therefore immune to whatever happens in the ring afterwards — immune to the RING, "
            "note, but NOT to that capture clock's own rate error against micros64(), measured "
            "2026-08-15 at a load-dependent 230 ppm idle to 580-670 ppm streaming, which is why "
            "the row to trend is lag_now_corrected_us and not this one, and why even that row is "
            "a growth channel rather than an absolute lag (see lag_now_us below); (2) "
            "sdo_rtt_min_us measures a real round trip over the same bus (the hand "
            "ball-sensor poll), whose floor must grow by exactly the ring delay. A ring leak "
            "of D predicts all three moving together by the same amount; a bus-level or "
            "ODrive-level fault does not. "
            "INSTRUMENTATION ONLY, AND THE LEAK IS DELIBERATELY NOT FIXED IN THIS FIRMWARE. "
            "Nothing in the bridge reads any field or accumulator introduced for this frame, "
            "so a wrong value here cannot move a leg; and the fix waits on the occupancy "
            "number so it can be judged against a measurement instead of a theory. "
            "Additive — no existing frame changes, so NO PROTOCOL_VERSION bump (the LegCmd / "
            "HandSensor / BridgeTxDiag / ClockDiag / CacheDiag precedent): an old Jetson "
            "ignores the unknown msg_type and a new Jetson renders never-seen as unknown. An "
            "FW <= 12 board never sends this frame, so its consumer surface must read EMPTY, "
            "never error."),
        fields=[
            Field("t_local_us",          "u64", 1,
                  "micros64() at the emit instant — the pure-crystal monotonic clock, never the "
                  "steppable wall (time_base.h's clock-discipline invariant). This IS the bridge's "
                  "power-on uptime, which is the independent variable of the entire arc: the leak "
                  "RATCHETS, so every field here is read as a function of this one. Carrying it "
                  "makes each sample self-locating without a join against /link_status uptime_ms"),
            Field("fifo_overflows_jb",   "u32", 1,
                  "Jugglebot bus (ROLE, not controller number — jugglebot/cone are swapped in the "
                  "current operating config, can_buses.cpp): hardware FIFO OVERFLOW events "
                  "(FlexCAN IFLAG1 bit 7), cumulative since boot. A frame LOST inside the "
                  "peripheral, upstream of the software ring and therefore upstream of every other "
                  "counter the bridge keeps — the one RX loss path that no existing telemetry can "
                  "see. Upstream FlexCAN_T4 clears this flag in the ISR and records nothing; the "
                  "vendored copy counts it at the clear site (PROVENANCE.md patch P2), in ISR "
                  "context, so unlike the `_available` counters it is EXACT and race-free. Nonzero "
                  "means the drain is not keeping up at the peripheral, which is a different and "
                  "worse failure than the ring leak: leaked frames are merely LATE, overflowed "
                  "frames are GONE. Must be 0"),
            Field("fifo_overflows_bb",   "u32", 1, "Ball Butler bus: same hardware-FIFO overflow count, cumulative"),
            Field("fifo_overflows_cone", "u32", 1, "Cone bus: same hardware-FIFO overflow count, cumulative"),
            Field("fifo_warns_jb",       "u32", 1,
                  "Jugglebot bus: hardware FIFO WARNING events (IFLAG1 bit 6 — the FIFO reached its "
                  "almost-full mark), cumulative since boot. The PRECURSOR to fifo_overflows above, "
                  "and carried for the same reason cap_hits is carried beside depth_hwm: it "
                  "separates 'the peripheral got close' from 'the peripheral lost a frame', which "
                  "are different points on the same failure and want different responses. Also the "
                  "only field here that can rise while the software ring still looks perfectly "
                  "drained, because it reports pressure UPSTREAM of the ring"),
            Field("fifo_warns_bb",       "u32", 1, "Ball Butler bus: same FIFO warning count, cumulative"),
            Field("fifo_warns_cone",     "u32", 1, "Cone bus: same FIFO warning count, cumulative"),
            Field("probe_ticks",         "u32", 1,
                  "Service ticks probed during THIS window (not cumulative), i.e. how many "
                  "post-drain occupancy samples the leak_hwm / true_depth_hwm extrema were reduced "
                  "from. Nominally ~1000 at the 1 kHz task_can_rx rate. The denominator that makes "
                  "a high-water mark interpretable — a maximum over 1000 samples is a finding, a "
                  "maximum over 3 is an anecdote — and an independent task_can_rx starvation "
                  "detector: a count far below window_us/1000 means the RX task itself was late, "
                  "which on this box would be a finding in its own right and would also mean the "
                  "ring had longer than usual to accumulate between drains"),
            Field("lag_now_us",          "i32", 1,
                  "JUGGLEBOT DELIVERY LAG at the last frame folded this window, microseconds, "
                  "RELATIVE TO THE FIRST FRAME AFTER SEEDING. Read flags.LAG_SEEDED FIRST. "
                  "HOW IT IS MEASURED. CAN_message_t.timestamp is the FlexCAN free-running timer "
                  "captured BY HARDWARE at frame reception (1 us/tick at 1 Mbit), so it is immune "
                  "to everything that happens to the frame afterwards — including sitting in a "
                  "leaked ring. It is only 16 bits, wrapping every 65.536 ms, so a naive "
                  "now-minus-timestamp is ambiguous. Instead the decode path accumulates "
                  "(uint16_t)(ts - ts_prev) into a 64-bit per-bus ARRIVAL CLOCK: delivery is "
                  "FIFO-ordered and inter-delivery gaps are ~0.45 ms at the jugglebot bus's "
                  "~2240 frames/s, so the unwrap is safe by a factor of ~100, and the one case "
                  "where it would not be (a real gap approaching the wrap) is detected and forces "
                  "a reseed rather than a silent corruption. This field is then "
                  "(micros64_now - micros64_at_seed) - (arrival_clock - arrival_clock_at_seed), "
                  "which telescopes EXACTLY to (delivery lag now) - (delivery lag at seed). "
                  "WHY THIS FORMULATION AND NOT THE PER-WINDOW DIVERGENCE. The alternative — "
                  "differencing capture-span against decode-span within each window — measures the "
                  "leak's RATE, and the estimated ratchet is ~40 ms/h, i.e. ~11 us per 1 s window: "
                  "below the sampling noise of a single window and indistinguishable from jitter. "
                  "This formulation INTEGRATES that same rate instead, so the ratchet appears as an "
                  "unmistakable ramp of tens of ms per hour. (The rate form is still recoverable "
                  "host-side from cap_span_us below, so nothing is lost by choosing the integral.) "
                  "THE ABSOLUTE VALUE HAS A BOOT-TIME OFFSET and is NOT the delivery lag: the "
                  "reference is whatever the lag happened to be at the seed instant. A fresh-boot "
                  "soak supplies that zero. THE TREND IS THE MEASUREMENT — BUT THE RAW TREND IS "
                  "NOT THE LEAK ALONE. Measured 2026-08-15 on FW 14 with leak == 0 on every bus, "
                  "this integral STILL ramps, at 229-323 us/s across four bags, because the "
                  "FlexCAN capture clock runs SLOW against micros64() at a LOAD-DEPENDENT rate: "
                  "~230 ppm with no setpoint stream, ~580-670 ppm while streaming, reverting "
                  "within a second of the stream stopping. The ratio cap_span_us/window_us "
                  "predicts the observed slope to better than 1 % in every bag (0.999761 -> "
                  "238.6 us/s predicted vs 238.8 measured; 0.999708 -> 292.2 vs 293.1; 0.999771 "
                  "-> 228.8 vs 229.0; 0.999678 -> 322.2 vs 323.2) — the two clocks in this very "
                  "frame are what convict the artefact. "
                  "PER-SECOND AND PER-FRAME ARE THE SAME NORMALISATION HERE, so do not read the "
                  "per-frame form as evidence of a per-frame mechanism: jb_lag_fold folds "
                  "jugglebot-bus RX frames (ODrive broadcast traffic, steady at ~1950/s), while "
                  "the 500 Hz setpoint stream is TX and is never folded, so the fold rate is "
                  "LOAD-INVARIANT at ~1950 frames/window in every bag, idle or streaming. ppm and "
                  "us-per-frame therefore rise by the identical factor (x2.46 and x2.47 between "
                  "an idle and a streaming bag). Representative: bag 10-06-14 (no stream) "
                  "236.8 ppm / 0.1215 us-per-frame at 1946-1985 frames/window; bag 10-08-06 "
                  "(streaming) 581.7 ppm / 0.3000 us-per-frame at 1864-1989 frames/window. "
                  "WHY bus load changes the capture clock's rate against micros64() at all is "
                  "UNEXPLAINED; the separate-dividers story below accounts for a constant ratio, "
                  "not a load-varying one. "
                  "Three consequences. (1) THE ABSOLUTE VALUE IS (time since the last reseed) x "
                  "that rate, and must NOT be read as a delivery lag. (2) A ramp here is NOT by "
                  "itself evidence of a leak: S3's '151-183 ms creeping ~0.35 ms/s' was this same "
                  "artefact (0.35 ms/s = 350 ppm) and its proximity to the true ~130 ms ring "
                  "delay was a coincidence — the ring's 135 ms one-lap physical cap is the sanity "
                  "bound this field can and did exceed while true_depth reported the ring empty. "
                  "(3) READ THE CORRECTED ROW. teensy_bridge_node's LagClockNormalizer "
                  "re-estimates the rate CONTINUOUSLY from ring-certified-clean windows "
                  "(abstaining on a leaking plant, where calibrating against the leak would be a "
                  "tautology — and continuously because a single rate pooled across a load "
                  "transition under-corrects by ~350 ppm) and publishes lag_now_corrected_us on "
                  "/ring_diag beside this raw value, with its calibration "
                  "(lag_corr_rate_us_per_frame, lag_corr_rate_ppm, lag_corr_windows) alongside so "
                  "the correction is auditable from the bag alone. Even corrected, that row is a "
                  "GROWTH/DELTA channel since the last reseed, never an absolute lag; leak_* "
                  "above remains the absolute-occupancy channel. This field itself is published "
                  "RAW and unchanged. "
                  "SIGNED because the FlexCAN timer and micros64() are separate dividers off the "
                  "same crystal, so a rate ratio can drift this either way; the measured sign is "
                  "POSITIVE (capture clock slow), and a fresh-boot soak at a FIXED bus load "
                  "measures the artefact as a straight line where the leak is a ratchet"),
            Field("lag_hwm_us",          "i32", 1,
                  "Maximum lag_now_us observed since the last seed (NOT since boot — a reseed "
                  "resets it, because after a reseed the two are not measured against the same "
                  "reference and a max across that boundary would be meaningless). The tail "
                  "detector for the ~100-150 ms per-axis freezes S2 measured: those are far shorter "
                  "than a 1 s window, so an instantaneous end-of-window read can miss every one of "
                  "them while the high-water mark cannot"),
            Field("cap_span_us",         "u32", 1,
                  "Arrival-clock advance across THIS window: the sum of the unwrapped inter-frame "
                  "CAPTURE intervals folded since the previous emit. Its counterpart is window_us, "
                  "the same interval measured in DECODE time, and the difference between the two is "
                  "the per-window lag divergence — the rate formulation named in lag_now_us above. "
                  "Carried despite being derivable from consecutive lag_now_us values because that "
                  "derivation silently breaks across a DROPPED UPLINK FRAME or a reseed, whereas "
                  "this one is self-contained within the window it describes. Four bytes to make "
                  "each sample independently interpretable. 0 with LAG_SEEDED clear"),
            Field("lag_frames",          "u32", 1,
                  "Jugglebot frames folded into the arrival clock during THIS window (not "
                  "cumulative). Nominally ~2240. The denominator for cap_span_us, and the field "
                  "that distinguishes 'lag_now_us did not move' from 'nothing arrived to move it' "
                  "— the same 0-is-not-a-measurement trap seen_mask closes on CacheDiag"),
            Field("lag_reseeds",         "u32", 1,
                  "Arrival-clock reseeds since boot, CUMULATIVE. A reseed happens when the "
                  "inter-delivery gap exceeds the wrap-safety bound, i.e. when the 16-bit capture "
                  "stamp could have wrapped unobserved (bus quiet: ODrives unpowered, or a "
                  "multi-tick task_can_rx stall). Reseeding is deliberately chosen over "
                  "accumulating a wrong unwrap: a corrupted arrival clock poisons every subsequent "
                  "sample forever, while a reseed costs one comparable pair and is VISIBLE here. "
                  "Every increment marks a discontinuity the consumer must segment the lag series "
                  "at; flags.LAG_RESEED_IN_WINDOW says whether one landed in THIS window"),
            Field("sdo_rtt_min_us",      "u32", 1,
                  "MINIMUM hand-ball-sensor SDO round-trip time over this window, microseconds: "
                  "from the request stamp (taken immediately BEFORE the CAN send in "
                  "gpio_poll_step) to the reply's arrival stamp in the RX decode. Read "
                  "sdo_rtt_count FIRST — 0 there means no round trip closed and all three RTT "
                  "fields are 0, not a measurement. "
                  "THE CAUSAL CROSS-CHECK. This is a REAL round trip over the jugglebot bus at "
                  "50 Hz (the poller already runs; nothing new is put on the wire for this), and "
                  "its return path goes through the same RX ring as everything else. If the ring "
                  "is holding D frames, this floor grows by exactly D's worth of delay — "
                  "aged-minus-fresh on this number is the ring delay, measured end-to-end through "
                  "a path that has an independent, physically-fixed floor (the ODrive's own SDO "
                  "service time). A leak predicts it moves with true_depth and lag_now_us; a bus "
                  "or ODrive fault does not move all three together. "
                  "A MINIMUM rather than a mean for the reason the CacheDiag age floor is a "
                  "minimum: the request stamp is taken on task_homing (priority 2, the lowest "
                  "real-work task), so a preemption between the stamp and the wire can only "
                  "INFLATE an individual sample. Inflation cannot move a floor drawn from ~50 "
                  "samples; it would corrupt a mean. Stamping before rather than after the send "
                  "is deliberate for the same reason — the other order would deflate samples and "
                  "put the corruption exactly where it does damage"),
            Field("sdo_rtt_max_us",      "u32", 1,
                  "MAXIMUM SDO round-trip time over the same window. The tail, and the pair that "
                  "makes the shape legible: min and max rising together = the whole path got "
                  "slower (what a ring leak predicts); min flat with max rising = occasional "
                  "stalls on an otherwise healthy path, which is a different fault. Also the "
                  "witness for the ~100-150 ms freeze events, which are too short to move a "
                  "1 Hz spot reading"),
            Field("sdo_rtt_last_us",     "u32", 1,
                  "The LAST SDO round-trip time folded this window. Neither an extremum nor a "
                  "reduction — the spot reading, carried so a live console watcher has an "
                  "unreduced number to sanity-check the pair above against. Cheap (one word, one "
                  "store) and it is the only field here that can be compared against a stopwatch"),
            Field("seq",                 "u32", 1,
                  "Emitted-frame counter since boot (1 on the first). This frame is one-shot per "
                  "window on a lossy transport with no retransmission, so without a sequence a "
                  "silently dropped sample would join two windows into one — inflating a "
                  "per-window count and hiding a reseed — while looking like clean data. Also the "
                  "de-duplication key for a latest-value consumer"),
            Field("window_us",           "u32", 1,
                  "micros64() elapsed since the previous emit (0 on the FIRST frame, where no "
                  "window exists). The honest window length in DECODE time: nominal 1 000 000, and "
                  "a longer one means task_telem was delayed. Paired with cap_span_us it is the "
                  "rate formulation of the delivery lag. Saturates at UINT32_MAX (~71.6 min)"),
            Field("true_depth_jb",       "u16", 1,
                  "THE HEADLINE, first half. Jugglebot bus: TRUE RX-ring occupancy, derived from "
                  "the ring's head/tail indices rather than from `_available`, sampled at the last "
                  "service tick of this window IMMEDIATELY AFTER the drain loop finished. Because "
                  "the drain loop exits when `_available` reads 0, this residual is the stranded "
                  "backlog itself. Nominal is 0 forever, at any uptime; the ring is 256 deep, so "
                  "the leak saturates there at ~114-135 ms of delivery delay at jugglebot-bus "
                  "rates. u16 rather than u32 deliberately: a wider field would imply a range the "
                  "hardware cannot produce"),
            Field("true_depth_bb",       "u16", 1, "Ball Butler bus: same post-drain true occupancy"),
            Field("true_depth_cone",     "u16", 1, "Cone bus: same post-drain true occupancy"),
            Field("avail_reported_jb",   "u16", 1,
                  "THE HEADLINE, second half. Jugglebot bus: the ring's own `_available` counter, "
                  "read at the SAME probe as true_depth_jb (and read AFTER it, so a push racing "
                  "the probe can only shrink the reported leak, never invent one). This is exactly "
                  "what getRXQueueCount() returns and therefore exactly what every pre-FW-13 "
                  "counter — depth_hwm, cap_hits, the drain loop's own exit test — is built on. "
                  "`true_depth_jb - avail_reported_jb` IS THE LEAK: one number, one comparison, and "
                  "the whole hypothesis stands or falls on it. Nominal is equality"),
            Field("avail_reported_bb",   "u16", 1, "Ball Butler bus: same `_available` reading at the same probe"),
            Field("avail_reported_cone", "u16", 1, "Cone bus: same `_available` reading at the same probe"),
            Field("leak_hwm_jb",         "u16", 1,
                  "Jugglebot bus: maximum (true_depth - avail_reported) observed at ANY service "
                  "tick since boot — a HIGH-WATER MARK, read as a level and never differenced. "
                  "The verdict field. true_depth_jb / avail_reported_jb above are a single 1 Hz "
                  "instant out of ~1000 probes per window, so on their own they could miss a leak "
                  "that only shows under load; this maximum cannot. It is also the field that "
                  "makes the ratchet claim testable: the leak is theorised to be monotonic, so "
                  "this and the instantaneous pair should track each other, and a high-water mark "
                  "far above a persistently-zero instantaneous reading would REFUTE the ratchet "
                  "and point at a transient instead"),
            Field("leak_hwm_bb",         "u16", 1, "Ball Butler bus: same leak high-water mark"),
            Field("leak_hwm_cone",       "u16", 1, "Cone bus: same leak high-water mark"),
            Field("true_depth_hwm_jb",   "u16", 1,
                  "Jugglebot bus: maximum post-drain TRUE occupancy since boot. Distinct from "
                  "leak_hwm above, and the pair separates two different failures that the existing "
                  "(blind) depth_hwm renders identically: true depth high WITH `_available` "
                  "tracking it = a genuine backlog the drain budget could not clear, which is a "
                  "throughput problem; true depth high with `_available` at 0 = the leak, which is "
                  "a correctness problem. Cap 256"),
            Field("true_depth_hwm_bb",   "u16", 1, "Ball Butler bus: same true-occupancy high-water mark"),
            Field("true_depth_hwm_cone", "u16", 1, "Cone bus: same true-occupancy high-water mark"),
            Field("sdo_rtt_count",       "u16", 1,
                  "SDO round trips that COMPLETED during this window, nominally ~50 at the "
                  "poller's 20 ms interval. READ THIS BEFORE THE THREE RTT FIELDS: 0 means no "
                  "round trip closed — the poller is off, parked on the version gate, or every "
                  "request timed out — and the three RTT fields are then 0 because there was "
                  "nothing to measure, NOT because the path was instant. Also the denominator that "
                  "makes sdo_rtt_min_us a floor rather than an anecdote. The poller's own health "
                  "is deliberately NOT duplicated here: HAND_SENSOR (0x8B) already carries its "
                  "validity and staleness at this same rate, and a second source of truth for one "
                  "fact is how the two disagree"),
            Field("flags",               "u8",  1,
                  "RingDiagFlags bitfield: LAG_SEEDED / LAG_RESEED_IN_WINDOW. Read this BEFORE "
                  "lag_now_us and lag_hwm_us — both are 0 when unseeded, and a 0 lag must never be "
                  "readable as a healthy measurement when it actually means no reference exists "
                  "(the ClockDiag FREQ_VALID discipline)"),
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
# and teensy_link/rpc_args.py wraps the generated Python.
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
    # the 0x6E0 frame itself (mirroring Teensy_code_platform.ino createStateCANMessage) —
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
    a("    specifically. See teensy_link/client.py's RX decode path,")
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
    a("generated `JbUdp::RpcArgs::*` structs; `teensy_link/rpc_args.py`")
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
