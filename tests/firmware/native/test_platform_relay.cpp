// =============================================================================
//  test_platform_relay.cpp — compiled test of the REAL Platform-Teensy relay seam
// =============================================================================
//  Drives the actual compiled platform_relay.cpp (it #includes the .cpp) against
//  the recording fake HAL, asserting the safety-relevant BEHAVIOURS of the
//  Platform-Teensy relay write seam:
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
#include <cmath>                 // NAN / INFINITY (state_write float validation)

#include "udp_protocol.h"
#include "protocol_config.h"
#include "odrive_protocol.h"
#include "can_buses.h"
#include "rpc.h"                 // method_gates_on_bus_transmittable (reboot-latch gate policy)
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

TEST_CASE("state_write rejects a non-finite pose offset (ERR_BAD_ARGS, nothing sent)") {
  // A NaN/Inf pose offset makes the `* 1000.0f` → int16 cast UNDEFINED, and the corrupt
  // value would be PERSISTED on the Platform Teensy across reboots. Reject before the cast.
  fake_reset();
  fake_set_commands_allowed(true);

  JbUdp::RpcArgs::ArgRobotState s{};
  s.is_homed = 1;
  s.pose_offset_tiltX = NAN;   s.pose_offset_tiltY = 0.0f;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK(fake_sent_count() == 0);          // rejected before any CAN3 frame

  s.pose_offset_tiltX = 0.0f;  s.pose_offset_tiltY = INFINITY;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK(fake_sent_count() == 0);

  s.pose_offset_tiltX = -INFINITY;  s.pose_offset_tiltY = NAN;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK(fake_sent_count() == 0);

  // A finite pose still encodes + sends (the guard is non-finite ONLY).
  s.pose_offset_tiltX = 0.012f;  s.pose_offset_tiltY = -0.034f;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::OK);
  REQUIRE(fake_sent_count() == 1);
  CHECK(le_i16(&fake_sent_at(0).buf[1]) == 12);
  CHECK(le_i16(&fake_sent_at(0).buf[3]) == -34);
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

TEST_CASE("bus_partner_present pins the TX presence-gate window semantics") {
  // The can_*_send() gate (2026-07-05 marginal-CAN3 fix): TX is allowed only when
  // some partner frame arrived within BUS_PARTNER_STALENESS_US. Pin the window
  // value and the boundary/never-seen semantics so a config drift or an off-by-one
  // in the predicate fails loudly here rather than as silent bus-off pollution.
  constexpr uint64_t W = CanBridge::BUS_PARTNER_STALENESS_US;
  CHECK(W == 5000000u);                                    // 5.0 s contract window
  CHECK_FALSE(CanBridge::bus_partner_present(0, 0));       // never seen ⇒ absent (boot)
  CHECK_FALSE(CanBridge::bus_partner_present(0, 10 * W));  // never seen stays absent
  CHECK(CanBridge::bus_partner_present(1000, 1000));       // same-instant frame ⇒ present
  CHECK(CanBridge::bus_partner_present(1000, 1000 + W));   // exactly at window ⇒ present
  CHECK_FALSE(CanBridge::bus_partner_present(1000, 1001 + W));  // one past ⇒ absent
}

TEST_CASE("classify_bus_health pins the health_of truth table (bus-off wiring 2026-07-05)") {
  // health_of() = classify_bus_health(last_rx, now, flt_live): RX staleness plus
  // the LIVE fault-confinement state (0 active / 1 passive / 2 bus-off). Pin the
  // severity ordering and boundaries — WARN/BUS_OFF refuse motion commands
  // (jugglebot_commands_allowed + the homing/activate/deactivate gates), so a
  // drift here silently changes when the robot refuses to move.
  using CanBridge::classify_bus_health;
  constexpr uint64_t T = CanBridge::CAN_HEARTBEAT_TIMEOUT_US;
  CHECK(T == 2000000u);   // 2.0 s staleness contract (can_node _HEARTBEAT_TIMEOUT_S parity)

  // Never seen a frame ⇒ UNKNOWN, whatever the registers say (bring-up allows
  // commands; the TX presence gate independently refuses sends on such a bus).
  CHECK(classify_bus_health(0, 0, 0) == JbUdp::BusHealth::UNKNOWN);
  CHECK(classify_bus_health(0, 10 * T, 2) == JbUdp::BusHealth::UNKNOWN);

  // Live confinement, RX fresh: active ⇒ OK, passive ⇒ WARN, bus-off ⇒ BUS_OFF.
  CHECK(classify_bus_health(1000, 1000, 0) == JbUdp::BusHealth::OK);
  CHECK(classify_bus_health(1000, 1000, 1) == JbUdp::BusHealth::WARN);
  CHECK(classify_bus_health(1000, 1000, 2) == JbUdp::BusHealth::BUS_OFF);
  CHECK(classify_bus_health(1000, 1000, 3) == JbUdp::BusHealth::BUS_OFF);  // clamped ≥2

  // Staleness boundary (active confinement): exactly at the window ⇒ still OK
  // (strict >, matching the pre-wiring health_of), one past ⇒ WARN.
  CHECK(classify_bus_health(1000, 1000 + T, 0) == JbUdp::BusHealth::OK);
  CHECK(classify_bus_health(1000, 1001 + T, 0) == JbUdp::BusHealth::WARN);

  // Bus-off outranks staleness (a bus-off controller stops receiving, so the two
  // co-occur; report the cause, not the symptom).
  CHECK(classify_bus_health(1000, 1001 + T, 2) == JbUdp::BusHealth::BUS_OFF);
  // Passive + stale still WARN (both terms agree).
  CHECK(classify_bus_health(1000, 1001 + T, 1) == JbUdp::BusHealth::WARN);
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

TEST_CASE("gate policy: only CLEAR_ERRORS/REBOOT gate on bus-transmittable (SYNCH)") {
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
