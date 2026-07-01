// =============================================================================
//  test_platform_relay.cpp — compiled test of the REAL Platform-Teensy relay seam
// =============================================================================
//  Drives the actual compiled platform_relay.cpp (it #includes the .cpp) against
//  the recording fake HAL, asserting the safety-relevant BEHAVIOURS of the
//  Phase-1 relay write seam:
//
//    * tilt_read() / state_read() emit the right CAN3 TRIGGER frame on the right
//      arbitration id (0x7DE / 0x6E0) and dlc — so a Platform-Teensy reply is
//      actually solicited;
//    * state_write() byte-for-byte re-encodes the 0x6E0 RobotState frame exactly
//      as Teensy_code.ino decodeStateCANMessage unpacks it (flags + int16 pose) —
//      so the can-bridge, as the SOLE writer, cannot corrupt the persisted state;
//    * every relay send is GATED on jugglebot_commands_allowed() (the never-
//      command-a-dead-bus fail-fast: ERR_BUS_DOWN, and NOTHING put on CAN3);
//    * is_platform_reply_id() classifies the two reply ids and nothing else (the
//      CAN3 RX ring filter that routes replies to the verbatim uplink);
//    * the generated hand_axis6_permitted() allow-table permits exactly the hand
//      ODrive ops the plan locks and rejects the rest (the compiled predicate the
//      rpc.cpp axis-6 gate consumes).
//
//  SCOPE: validates DECISION LOGIC, not FreeRTOS/ISR concurrency or 500 Hz
//  timing (the SPSC relay ring + the on-wire SRX_DIS reply-correlation assumption
//  remain on-hardware-replay/bench gaps). See README.md.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "udp_protocol.h"
#include "protocol_config.h"
#include "odrive_protocol.h"
#include "can_buses.h"
#include "rpc.h"                 // Phase 6: method_gates_on_bus_transmittable (gate policy)
#include "fake_hal.h"

#include "platform_relay.cpp"   // the unit under test

using namespace CanBridge;

// Decode a recorded little-endian int16 from a frame buffer (pose field check).
static int16_t le_i16(const uint8_t* b) {
  return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

TEST_CASE("relay reads emit the right CAN3 trigger frame") {
  fake_reset();
  fake_set_commands_allowed(true);

  CHECK(Relay::tilt_read() == JbUdp::RpcStatus::OK);
  REQUIRE(fake_sent_count() == 1);
  CHECK(fake_sent_at(0).id == PlatformCanId::TILT_READING);   // 0x7DE
  CHECK(fake_sent_at(0).len == 1);                            // dlc 1 (≠ the dlc-8 reply)

  fake_clear_sent();
  CHECK(Relay::state_read() == JbUdp::RpcStatus::OK);
  REQUIRE(fake_sent_count() == 1);
  CHECK(fake_sent_at(0).id == PlatformCanId::STATE_UPDATE);   // 0x6E0
  CHECK(fake_sent_at(0).len == 1);
  CHECK(fake_sent_at(0).buf[0] == 0x01);                      // request discriminator
}

TEST_CASE("state_write re-encodes the 0x6E0 RobotState frame (createStateCANMessage parity)") {
  fake_reset();
  fake_set_commands_allowed(true);

  JbUdp::RpcArgs::ArgRobotState s{};
  s.is_homed = 1;
  s.levelling_complete = 0;
  s.pose_offset_tiltX = 0.012f;    // *1000 → 12
  s.pose_offset_tiltY = -0.034f;   // *1000 → -34
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::OK);

  REQUIRE(fake_sent_count() == 1);
  const SentFrame& f = fake_sent_at(0);
  CHECK(f.id == PlatformCanId::STATE_UPDATE);
  CHECK(f.len == 8);
  CHECK(f.buf[0] == 0x01);          // flags: bit0 is_homed=1, bit1 levelling=0
  CHECK(le_i16(&f.buf[1]) == 12);   // pose_offset_tiltX * 1000
  CHECK(le_i16(&f.buf[3]) == -34);  // pose_offset_tiltY * 1000
  CHECK(f.buf[5] == 0);
  CHECK(f.buf[6] == 0);
  CHECK(f.buf[7] == 0);

  // The flags byte ORs both bits when both are set.
  fake_clear_sent();
  s.is_homed = 1; s.levelling_complete = 1;
  s.pose_offset_tiltX = 0.0f; s.pose_offset_tiltY = 0.0f;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::OK);
  REQUIRE(fake_sent_count() == 1);
  CHECK(fake_sent_at(0).buf[0] == 0x03);   // bit0 | bit1
}

TEST_CASE("relay sends fail-fast when CAN3 is down (never command a dead bus)") {
  fake_reset();
  fake_set_commands_allowed(false);

  CHECK(Relay::tilt_read()  == JbUdp::RpcStatus::ERR_BUS_DOWN);
  CHECK(Relay::state_read() == JbUdp::RpcStatus::ERR_BUS_DOWN);
  JbUdp::RpcArgs::ArgRobotState s{};
  s.is_homed = 1;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::ERR_BUS_DOWN);

  // Crucially: nothing reached CAN3 (the bus-down fail-fast is BEFORE the send).
  CHECK(fake_sent_count() == 0);
}

TEST_CASE("is_platform_reply_id classifies exactly the two relay reply ids") {
  CHECK(is_platform_reply_id(PlatformCanId::STATE_UPDATE));   // 0x6E0
  CHECK(is_platform_reply_id(PlatformCanId::TILT_READING));   // 0x7DE
  CHECK_FALSE(is_platform_reply_id(PlatformCanId::TRAJ_CMD)); // 0x6D0 (hand traj — not a reply)
  CHECK_FALSE(is_platform_reply_id(ODrive::arb_id(0, ODriveCmd::heartbeat_message)));  // a leg frame
  CHECK_FALSE(is_platform_reply_id(0x000u));
}

TEST_CASE("hand axis-6 allow-table permits exactly the locked hand ODrive ops") {
  using namespace JbUdp;
  // Permit (the hand homing + command surface).
  CHECK(hand_axis6_permitted(RpcMethod::SET_AXIS_STATE));
  CHECK(hand_axis6_permitted(RpcMethod::SET_CONTROLLER_MODE));
  CHECK(hand_axis6_permitted(RpcMethod::SET_POS_GAIN));
  CHECK(hand_axis6_permitted(RpcMethod::SET_VEL_GAINS));
  CHECK(hand_axis6_permitted(RpcMethod::SET_VEL_CURR_LIMITS));
  CHECK(hand_axis6_permitted(RpcMethod::CLEAR_ERRORS));
  CHECK(hand_axis6_permitted(RpcMethod::REBOOT_ODRIVES));
  CHECK(hand_axis6_permitted(RpcMethod::HOME));
  CHECK(hand_axis6_permitted(RpcMethod::SET_ABSOLUTE_POSITION));
  // Reject (leg-specific cold-start moves + absolute-encoder search + non-axis ops).
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::ENCODER_SEARCH));
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::ACTIVATE));
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::DEACTIVATE));
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::SDO_READ));
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::SDO_WRITE));
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::NOP));
}

TEST_CASE("Phase 6 gate policy: only CLEAR_ERRORS/REBOOT gate on bus-transmittable (SYNCH)") {
  using namespace JbUdp;
  using CanBridge::Rpc::method_gates_on_bus_transmittable;
  // The two operator recovery one-shots gate on the LIVE bus-transmittable (SYNCH)
  // signal so a recovery clear reaches a just-repowered bus (the 2026-06-27 deadlock).
  CHECK(method_gates_on_bus_transmittable(RpcMethod::CLEAR_ERRORS));
  CHECK(method_gates_on_bus_transmittable(RpcMethod::REBOOT_ODRIVES));
  // Every other op keeps the heartbeat-staleness gate (jugglebot_commands_allowed):
  // a stale bus SHOULD withhold a setpoint/config. A future edit that drops a method
  // from the bus-transmittable set (re-gating clear/reboot onto staleness) fails here.
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_AXIS_STATE));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_CONTROLLER_MODE));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_POS_GAIN));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_VEL_GAINS));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_VEL_CURR_LIMITS));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_ABSOLUTE_POSITION));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::HOME));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::ACTIVATE));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::DEACTIVATE));
}
