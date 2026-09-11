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

}  // namespace Relay
}  // namespace CanBridge
