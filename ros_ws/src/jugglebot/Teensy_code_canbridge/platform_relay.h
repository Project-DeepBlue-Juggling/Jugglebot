#pragma once
// =============================================================================
//  platform_relay.h — typed Jetson→CAN3→Platform-Teensy relay write RPCs
// =============================================================================
//  The can-bridge re-establishes the Jetson↔Platform-Teensy conduit that the old
//  can_node owned directly: the inclinometer tilt read (0x7DE), and the cold-start
//  RobotState read/write (0x6E0 — is_homed / levelling_complete / pose offset).
//
//  Shape ("hybrid relay seam", see logbook 2026-06-29-canbridge-phase1-platform-relay-seam):
//   • WRITE direction is TYPED, validated, gated — one RPC per operation, NOT a
//     generic forward-arbitrary-frame primitive (a generic forwarder's only guard
//     is a runtime allow-list, and one careless edit re-opens leg-command
//     injection bypassing the step-gate). The firmware owns the CAN frame layout
//     (mirrors Teensy_code_platform.ino createStateCANMessage), so the Jetson never
//     supplies a raw frame.
//   • READ direction is async: the request frame triggers the Platform Teensy to
//     answer on the SAME id, and on_jugglebot_rx (can_buses.cpp) forwards that
//     reply VERBATIM to the host as a PLATFORM_FRAME. The host owns the decode and
//     correlates the reply by (can_id, dlc) — so this file only SENDS the trigger.
//
//  Every send is gated on jugglebot_commands_allowed() (the shared CAN3 gate) and
//  returns a JbUdp::RpcStatus synchronous ack (OK = trigger/write queued on CAN3,
//  ERR_BUS_DOWN = CAN3 down). The host read RPC aborts on ERR_BUS_DOWN BEFORE its
//  async await loop (the fail-fast pattern).
// =============================================================================

#include <cstdint>
#include "udp_protocol.h"   // JbUdp::RpcStatus, JbUdp::RpcArgs::ArgRobotState

namespace CanBridge {
namespace Relay {

// TILT_READ: trigger a Platform-Teensy inclinometer read (0x7DE). The tilt reply
// (0x7DE, two float32) returns async as a PLATFORM_FRAME. Returns RpcStatus.
uint16_t tilt_read();

// STATE_READ: trigger a Platform-Teensy RobotState read (0x6E0 request, dlc 1,
// byte0 0x01). The RobotState reply (0x6E0, dlc 8) returns async as a
// PLATFORM_FRAME. Returns RpcStatus.
uint16_t state_read();

// STATE_WRITE: write the whole RobotState to the Platform Teensy (0x6E0, dlc 8).
// The firmware encodes the frame from the typed args (flags byte + int16 pose,
// mirroring Teensy_code_platform.ino createStateCANMessage). No reply. Returns RpcStatus.
uint16_t state_write(const JbUdp::RpcArgs::ArgRobotState& s);

// HAND_FAN_SET (FW 41): validate, then encode the HAND_FAN_CMD frame. ERR_BAD_ARGS on an
// unknown mode, a non-finite threshold, or thresholds outside
// TeensyOp::HAND_FAN_THRESHOLD_MIN_C..MAX_C or without a gap. Layout:
// Teensy_code_platform/hand_fan.h. The reply is the Platform's HAND_FAN_STATUS frame.
uint16_t hand_fan_set(const JbUdp::RpcArgs::ArgHandFanSet& a);

// ── Platform IMU capture (FW 45, IDEA-20 P3a) ────────────────────────────────
// Contract: ros_ws/docs/platform_imu_protocol.md sections 2 and 6. Both build the
// 8-byte IMU_CMD frame (PlatformCanId::IMU_CMD, 0x6E3, unused bytes 0) and send it
// through the shared relay gate (send_gated: jugglebot_commands_allowed, else
// ERR_BUS_DOWN; TxCls::SAFETY like every relay write). OK = queued on the bus;
// the Platform Teensy's verdict is its IMU_REPLY (0x6E4), uplinked as a
// PLATFORM_FRAME, exactly as HAND_FAN_SET's verdict is its HAND_FAN_STATUS.
// Both are ALLOWED while Armed (owner, 2026-11-19), and the bridge sends no ABORT
// of its own at arming: the Jetson owns the "no upload during an Attempt" gate.
//
// PLATFORM_IMU_CTRL → [op][reason or capture_id LE][0..]: ENABLE/DISABLE/ABORT
// carry no args, FREEZE carries `reason` in byte 1, RESUME `capture_id` in bytes
// 1-2, SET_ODR (FW 47, Platform FW 13) `reason` as odr_khz in byte 1 (the value is
// the Platform's to judge, as FREEZE's reason is). Any other op, READ included, is
// ERR_BAD_ARGS with nothing sent: a READ can only leave through platform_imu_read
// and its clamp. SET_ODR is allowed while Armed like the rest: the capture rate is
// the Platform Teensy's I2C sampling, not CAN traffic, and a rate change while
// FROZEN or UPLOADING is the Platform's ERR_FROZEN.
uint16_t platform_imu_ctrl(const JbUdp::RpcArgs::ArgPlatformImuCtrl& a);
// PLATFORM_IMU_READ → [0x05][capture_id u16][first_seq u16][count u16][gap u8].
// ERR_BAD_ARGS for count 0 or above PlatformImu::READ_MAX_WORDS. While Armed
// (`armed_now` = fault_mpc_active(), passed in as for the FW ops) the gap is
// raised to PlatformImu::BRIDGE_ARMED_MIN_GAP_10US (63 x 10 us, about 1590
// frames/s), the bus protection the spec puts on the bridge; not Armed it goes out
// as asked and the Platform Teensy's own READ_MIN_GAP_10US floor applies.
uint16_t platform_imu_read(const JbUdp::RpcArgs::ArgPlatformImuRead& a, bool armed_now);
// The gap byte platform_imu_read puts on the wire. Pure, for the native harness.
uint8_t imu_read_effective_gap(uint8_t requested_10us, bool armed_now);

// ── Platform firmware-over-CAN (2026-09-09) ──────────────────────────────────
// The Platform Teensy's USB port is damaged, so its firmware now arrives over
// CAN3 through this same typed seam. Four ops on PlatformCanId::FW_UPDATE_CMD
// (0x6F0), byte 0 = opcode; the Platform answers every op on FW_UPDATE_REPLY
// (0x6F1, dlc 8: [opcode][status][seq LE][detail u32 LE]) and on_jugglebot_rx
// forwards that reply VERBATIM as a PLATFORM_FRAME — so these calls return only
// the synchronous queued-on-CAN3 ack, exactly like the relay writes above, and
// the host correlates the outcome from the uplinked reply.
//
// TYPED, not a raw forwarder: the host supplies image_len / seq+payload / crc32
// and the FIRMWARE lays out the frame (the same least-privilege reason
// state_write re-encodes 0x6E0 rather than forwarding Jetson bytes).
//
// GATE: every op is refused with ERR_REJECTED while `mpc_active_now` — an armed
// setpoint output means the legs are being driven, and a Platform Teensy that is
// erasing flash or rebooting mid-update is not a partner to have on the bus then.
// mpc_active is passed IN (from fault_mpc_active() in rpc.cpp) rather than read
// here, so this TU stays fault-machine-free for the native harness — the same
// wiring STATE_WRITE uses. The CAN3 bus gate (ERR_BUS_DOWN) still applies
// underneath, and PLATFORM_FW_DATA additionally rejects n outside 1..5 with
// ERR_BAD_ARGS before a byte reaches the wire.
//
// One DATA frame per RPC is deliberate: no bulk framing, no queue (~25k RPCs for
// a 120 KB image, under a minute), and every chunk keeps its own ack.

// PLATFORM_FW_BEGIN → 0x6F0 dlc 8: [0x01][image_len u32 LE][0,0,0].
uint16_t platform_fw_begin(const JbUdp::RpcArgs::ArgPlatformFwBegin& a, bool mpc_active_now);

// PLATFORM_FW_DATA → 0x6F0 dlc 3+n: [0x02][seq u16 LE][payload[0..n)].
uint16_t platform_fw_data(const JbUdp::RpcArgs::ArgPlatformFwData& a, bool mpc_active_now);

// PLATFORM_FW_VERIFY → 0x6F0 dlc 8: [0x03][crc32 u32 LE][0,0,0].
uint16_t platform_fw_verify(const JbUdp::RpcArgs::ArgPlatformFwVerify& a, bool mpc_active_now);

// PLATFORM_FW_COMMIT → 0x6F0 dlc 8: [0x04][0 × 7]. No args.
uint16_t platform_fw_commit(bool mpc_active_now);

// ── Ball Butler firmware-over-CAN (2026-09-28, FW 21) ───────────────────────
// The SAME contract as the Platform ops above — identical opcodes, frame layouts
// and arg structs — relayed on CAN1 to BallButlerCanId::FW_UPDATE_CMD (0x7D6).
// BB answers on 0x7D7, which on_bb_rx forwards verbatim through the platform
// reply ring as a PLATFORM_FRAME (the host correlates by can_id). BB_FW_INFO
// (op 0x05, dlc 8, payloadless) is BB-only: BB answers with its running
// FW_VERSION, the receipt that a COMMIT landed.
//
// GATE (passed in from rpc.cpp, like mpc_active above, so this TU stays free of
// bb_state for the native harness):
//   • BEGIN needs a FRESH BB heartbeat (else ERR_BUS_DOWN — the never-command-
//     an-absent-partner rule of send_bb_frame) and a BB state of IDLE or ERROR
//     (else ERR_REJECTED): never open a session mid-throw/reload/calibrate. BB's
//     own receiver re-checks this; the bridge gate is defence in depth.
//   • DATA / VERIFY / COMMIT / INFO need only a heartbeat SEEN since boot (else
//     ERR_BUS_DOWN). Not a fresh one: a 4 KB sector erase stalls BB's loop (and
//     so its 100 Hz heartbeat) for up to ~400 ms, against a 500 ms stale
//     threshold, and a mid-transfer ERR_BUS_DOWN would abort a healthy flash.
struct BbFwGate {
  bool    seen;    // bb_state.heartbeat_seen
  bool    fresh;   // heartbeat_seen && !heartbeat_stale
  uint8_t state;   // bb_state.state (BallButlerState::*)
};

uint16_t bb_fw_begin(const JbUdp::RpcArgs::ArgPlatformFwBegin& a, const BbFwGate& g);
uint16_t bb_fw_data(const JbUdp::RpcArgs::ArgPlatformFwData& a, const BbFwGate& g);
uint16_t bb_fw_verify(const JbUdp::RpcArgs::ArgPlatformFwVerify& a, const BbFwGate& g);
uint16_t bb_fw_commit(const BbFwGate& g);
uint16_t bb_fw_info(const BbFwGate& g);

}  // namespace Relay
}  // namespace CanBridge
