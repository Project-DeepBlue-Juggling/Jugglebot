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

}  // namespace Relay
}  // namespace CanBridge
