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

#include <cmath>               // std::isfinite (STATE_WRITE pose-offset validation)
#include "udp_protocol.h"
#include "protocol_config.h"   // PlatformCanId (STATE_UPDATE 0x6E0, TILT_READING 0x7DE)
#include "odrive_protocol.h"   // ODrive::CanFrame
#include "can_buses.h"         // can_jugglebot_tx / TxCls, jugglebot_commands_allowed

namespace CanBridge {
namespace Relay {

// Send one CAN3 frame through the shared gate. Returns the RpcStatus ack.
static uint16_t send_gated(const ODrive::CanFrame& f) {
  if (!jugglebot_commands_allowed()) return JbUdp::RpcStatus::ERR_BUS_DOWN;
  // TxCls::SAFETY (2026-08-24): relay ops ride the safety-deferral counter. A
  // DEFERRED relay frame is SENT — it reaches the Platform Teensy in order — so
  // the ack is OK, and only TxResult::FAILED (no bus partner) is ERR_TIMEOUT.
  return tx_reached_the_wire(can_jugglebot_tx(f, TxCls::SAFETY))
             ? JbUdp::RpcStatus::OK : JbUdp::RpcStatus::ERR_TIMEOUT;
}

uint16_t tilt_read() {
  // Any frame on TILT_READING triggers the Platform Teensy's inclinometer read +
  // reply (Teensy_code_platform.ino canSniff: `if (msg.id == tiltID) sendTiltData(...)`).
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
  // instead decoded as a WRITE — Teensy_code_platform.ino canSniff state branch).
  ODrive::CanFrame f;
  f.id = PlatformCanId::STATE_UPDATE;
  f.len = 1;
  f.buf[0] = 0x01;
  return send_gated(f);
}

uint16_t state_write(const JbUdp::RpcArgs::ArgRobotState& s) {
  // Trust-boundary float validation: a NaN/Inf pose offset makes the
  // `* 1000.0f` → int16 casts below UNDEFINED, and the corrupt value is then PERSISTED
  // on the Platform Teensy across reboots (it owns the cold-start state). Reject a
  // non-finite pose before the cast — mirrors the BB_THROW throw_args_valid isfinite
  // guard (rpc.cpp). is_homed / levelling are bools; only the two floats need checking.
  if (!(std::isfinite(s.pose_offset_tiltX) && std::isfinite(s.pose_offset_tiltY)))
    return JbUdp::RpcStatus::ERR_BAD_ARGS;
  // Encode the dlc-8 RobotState frame exactly as Teensy_code_platform.ino
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

// ── Platform firmware-over-CAN (2026-09-09) ─────────────────────────────────
// See platform_relay.h for the seam contract. Byte 0 of every FW_UPDATE_CMD
// frame is the opcode; the Platform receiver mirrors these four values.
static constexpr uint8_t FW_OP_BEGIN  = 0x01;
static constexpr uint8_t FW_OP_DATA   = 0x02;
static constexpr uint8_t FW_OP_VERIFY = 0x03;
static constexpr uint8_t FW_OP_COMMIT = 0x04;

// Max image bytes one FW_OP_DATA frame carries (8 - 1 opcode - 2 seq).
static constexpr uint8_t FW_DATA_MAX_N = 5;

// The FW-update gate, in front of the shared CAN3 gate. An armed setpoint output
// means the legs are under 500 Hz command; a Platform Teensy erasing flash or
// rebooting mid-update is not a bus partner to have then, so refuse outright
// (ERR_REJECTED — the same condition and status HAND_SOURCE_SET uses).
static uint16_t send_fw_update(const ODrive::CanFrame& f, bool mpc_active_now) {
  if (mpc_active_now) return JbUdp::RpcStatus::ERR_REJECTED;
  return send_gated(f);
}

// Start an FW_UPDATE_CMD frame: opcode in byte 0, the rest zeroed so every
// unused byte is deterministic on the wire (the receiver reads a fixed layout).
static ODrive::CanFrame fw_cmd_frame(uint8_t opcode) {
  ODrive::CanFrame f;
  f.id = PlatformCanId::FW_UPDATE_CMD;
  f.len = 8;
  f.buf[0] = opcode;
  for (uint8_t i = 1; i < 8; ++i) f.buf[i] = 0;
  return f;
}

static void put_u32_le(uint8_t* dst, uint32_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
  dst[2] = (uint8_t)((v >> 16) & 0xFF);
  dst[3] = (uint8_t)((v >> 24) & 0xFF);
}

uint16_t platform_fw_begin(const JbUdp::RpcArgs::ArgPlatformFwBegin& a, bool mpc_active_now) {
  ODrive::CanFrame f = fw_cmd_frame(FW_OP_BEGIN);
  put_u32_le(&f.buf[1], a.image_len);   // bytes 5-7 stay zero
  return send_fw_update(f, mpc_active_now);
}

uint16_t platform_fw_data(const JbUdp::RpcArgs::ArgPlatformFwData& a, bool mpc_active_now) {
  // Trust-boundary length validation BEFORE the frame exists: n == 0 would emit a
  // payload-less dlc-3 chunk the receiver would have to special-case, and n > 5
  // would read past the 5-byte arg payload into the next struct bytes. Refuse
  // both here so a malformed chunk never reaches CAN3 (the throw_args_valid /
  // state_write isfinite pattern; ERR_BAD_ARGS, checked ahead of the mpc gate
  // exactly as hand_source_request validates its value before !mpc_active).
  if (a.n < 1 || a.n > FW_DATA_MAX_N) return JbUdp::RpcStatus::ERR_BAD_ARGS;
  ODrive::CanFrame f = fw_cmd_frame(FW_OP_DATA);
  f.len = (uint8_t)(3 + a.n);           // short dlc — the receiver takes the length from the frame
  f.buf[1] = (uint8_t)(a.seq & 0xFF);
  f.buf[2] = (uint8_t)((a.seq >> 8) & 0xFF);
  for (uint8_t i = 0; i < a.n; ++i) f.buf[3 + i] = a.payload[i];
  return send_fw_update(f, mpc_active_now);
}

uint16_t platform_fw_verify(const JbUdp::RpcArgs::ArgPlatformFwVerify& a, bool mpc_active_now) {
  ODrive::CanFrame f = fw_cmd_frame(FW_OP_VERIFY);
  put_u32_le(&f.buf[1], a.crc32);       // bytes 5-7 stay zero
  return send_fw_update(f, mpc_active_now);
}

uint16_t platform_fw_commit(bool mpc_active_now) {
  // Payloadless: opcode only, bytes 1-7 zero (dlc 8, so the receiver's frame
  // shape is uniform across BEGIN/VERIFY/COMMIT).
  return send_fw_update(fw_cmd_frame(FW_OP_COMMIT), mpc_active_now);
}

}  // namespace Relay
}  // namespace CanBridge
