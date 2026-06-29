// =============================================================================
//  platform_relay.cpp — typed Jetson→CAN3→Platform-Teensy relay write RPCs
// =============================================================================
//  See platform_relay.h for the seam shape. This file stays thin (keeps rpc.cpp
//  thin too): it only builds the trigger/write CAN3 frames and gates each send on
//  the shared jugglebot_commands_allowed() predicate. The reply uplink lives in
//  can_buses.cpp (the relay ring) + telemetry.cpp (PLATFORM_FRAME); the host owns
//  reply decode + correlation.
// =============================================================================

#include "platform_relay.h"

#include "udp_protocol.h"
#include "protocol_config.h"   // PlatformCanId (STATE_UPDATE 0x6E0, TILT_READING 0x7DE)
#include "odrive_protocol.h"   // ODrive::CanFrame
#include "can_buses.h"         // can_jugglebot_send, jugglebot_commands_allowed

namespace CanBridge {
namespace Relay {

// Send one CAN3 frame through the shared gate. Returns the RpcStatus ack.
static uint16_t send_gated(const ODrive::CanFrame& f) {
  if (!jugglebot_commands_allowed()) return JbUdp::RpcStatus::ERR_BUS_DOWN;
  return can_jugglebot_send(f) ? JbUdp::RpcStatus::OK : JbUdp::RpcStatus::ERR_TIMEOUT;
}

uint16_t tilt_read() {
  // Any frame on TILT_READING triggers the Platform Teensy's inclinometer read +
  // reply (Teensy_code.ino canSniff: `if (msg.id == tiltID) sendTiltData(...)`).
  // A 1-byte request keeps it distinguishable (dlc 1) from the dlc-8 reply.
  ODrive::CanFrame f;
  f.id = PlatformCanId::TILT_READING;
  f.len = 1;
  f.buf[0] = 0x01;
  return send_gated(f);
}

uint16_t state_read() {
  // RobotState read request: STATE_UPDATE id, dlc 1, byte0 == 0x01 (the Platform
  // Teensy answers a dlc-1/0x01 frame with the dlc-8 RobotState; a dlc-8 frame is
  // instead decoded as a WRITE — Teensy_code.ino canSniff state branch).
  ODrive::CanFrame f;
  f.id = PlatformCanId::STATE_UPDATE;
  f.len = 1;
  f.buf[0] = 0x01;
  return send_gated(f);
}

uint16_t state_write(const JbUdp::RpcArgs::ArgRobotState& s) {
  // Encode the dlc-8 RobotState frame exactly as Teensy_code.ino
  // createStateCANMessage decodes it: byte0 flags (bit0 is_homed, bit1 levelling),
  // int16 LE pose*1000 about X (bytes 1-2) and Y (bytes 3-4), bytes 5-7 zero.
  ODrive::CanFrame f;
  f.id = PlatformCanId::STATE_UPDATE;
  f.len = 8;
  const uint8_t flags = (uint8_t)((s.is_homed ? 0x1u : 0u) |
                                  (s.levelling_complete ? 0x2u : 0u));
  const int16_t x = (int16_t)(s.pose_offset_tiltX * 1000.0f);
  const int16_t y = (int16_t)(s.pose_offset_tiltY * 1000.0f);
  f.buf[0] = flags;
  f.buf[1] = (uint8_t)(x & 0xFF);
  f.buf[2] = (uint8_t)((x >> 8) & 0xFF);
  f.buf[3] = (uint8_t)(y & 0xFF);
  f.buf[4] = (uint8_t)((y >> 8) & 0xFF);
  f.buf[5] = f.buf[6] = f.buf[7] = 0;
  return send_gated(f);
}

}  // namespace Relay
}  // namespace CanBridge
