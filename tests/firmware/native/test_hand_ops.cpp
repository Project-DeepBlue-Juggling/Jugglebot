// =============================================================================
//  test_hand_ops.cpp — compiled test of the REAL hand traj / smooth-move conduit
// =============================================================================
//  Drives the actual compiled hand_ops.cpp (it #includes the .cpp) against the
//  recording fake HAL, asserting the safety-relevant BEHAVIOURS of the hand-conduit
//  HAND_TRAJ_CMD dispatch:
//
//    * a valid traj cmd emits the CLOSED_LOOP + POSITION/PASSTHROUGH PREAMBLE to
//      the hand ODrive (axis 6) — byte-identical to ODrive::encode_set_state /
//      encode_set_controller_mode — THEN the 0x6D0 frame (the dropped precondition
//      the audit flagged, row 37);
//    * the 0x6D0 frame carries the host-built payload VERBATIM on the FIRMWARE-
//      OWNED PlatformCanId::TRAJ_CMD id (never a Jetson-supplied raw frame; the
//      firmware forwards opaque bytes so it CANNOT re-stamp the deadline);
//    * hand_traj_cmd GATES on jugglebot_commands_allowed() (a stale/dead CAN3 →
//      ERR_BUS_DOWN, and NOTHING put on CAN3 — the hand is gated like a leg);
//    * the traj TX ABORTS with NO 0x6D0 frame if a preamble send fails (running a
//      trajectory against a hand not in CLOSED_LOOP/PASSTHROUGH would fault/no-op).
//
//  SCOPE: validates DECISION LOGIC, not FreeRTOS/ISR concurrency or 500 Hz timing.
//  See README.md.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "udp_protocol.h"
#include "protocol_config.h"
#include "odrive_protocol.h"
#include "canbridge_config.h"
#include "can_buses.h"
#include "fake_hal.h"

#include "hand_ops.cpp"   // the unit under test

using namespace CanBridge;

// A representative catch-trajectory payload (byte 0 = traj_type 1, vel_u16, wall
// time low32, pad) — the firmware treats it as opaque bytes.
static JbUdp::RpcArgs::ArgHandTraj make_traj_arg() {
  JbUdp::RpcArgs::ArgHandTraj a{};
  const uint8_t payload[8] = {0x01, 0x2C, 0x01, 0x44, 0x33, 0x22, 0x11, 0x00};
  memcpy(a.payload, payload, 8);
  return a;
}

TEST_CASE("valid hand traj emits the CLOSED_LOOP + PASSTHROUGH preamble then the 0x6D0 frame") {
  fake_reset();
  fake_set_commands_allowed(true);

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);

  // Exactly 3 frames: two preamble ODrive frames to axis 6, then the 0x6D0 frame.
  REQUIRE(fake_sent_count() == 3);

  // Frame 0 = set_state(HAND_AXIS, CLOSED_LOOP) — byte-identical to the encoder.
  const auto pre0 = ODrive::encode_set_state(HAND_AXIS, ODriveState::CLOSED_LOOP);
  CHECK(fake_sent_at(0).id == pre0.id);
  CHECK(memcmp(fake_sent_at(0).buf, pre0.buf, 8) == 0);

  // Frame 1 = set_controller_mode(HAND_AXIS, POSITION, PASSTHROUGH).
  const auto pre1 = ODrive::encode_set_controller_mode(
      HAND_AXIS, ODriveControlMode::POSITION, ODriveInputMode::PASSTHROUGH);
  CHECK(fake_sent_at(1).id == pre1.id);
  CHECK(memcmp(fake_sent_at(1).buf, pre1.buf, 8) == 0);

  // Frame 2 = the 0x6D0 PLATFORM_TRAJ_CMD, id firmware-owned, payload verbatim.
  CHECK(fake_sent_at(2).id == PlatformCanId::TRAJ_CMD);   // 0x6D0
  CHECK(fake_sent_at(2).len == 8);
  CHECK(memcmp(fake_sent_at(2).buf, a.payload, 8) == 0);
}

TEST_CASE("the 0x6D0 payload is forwarded VERBATIM regardless of discriminator") {
  // A smooth-move payload (byte 0 = 3) forwards identically — the firmware does NOT
  // branch on the discriminator (it is meaningful only to the Platform Teensy).
  fake_reset();
  fake_set_commands_allowed(true);

  JbUdp::RpcArgs::ArgHandTraj a{};
  const uint8_t smooth[8] = {0x03, 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00};
  memcpy(a.payload, smooth, 8);

  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);
  REQUIRE(fake_sent_count() == 3);
  CHECK(fake_sent_at(2).id == PlatformCanId::TRAJ_CMD);
  CHECK(memcmp(fake_sent_at(2).buf, smooth, 8) == 0);
}

TEST_CASE("a hand traj is REJECTED while homing is active (interlock), nothing sent") {
  fake_reset();
  fake_set_commands_allowed(true);    // bus is fine — the rejection is the homing interlock, not the gate
  fake_set_homing(true);              // the shared state machine is mid-homing (e.g. axis 6)

  const auto a = make_traj_arg();
  // Checked FIRST — before the bus gate and before any preamble frame reaches CAN3.
  // A catch-traj mid-homing would fight the axis-6 move-to-hardstop and corrupt the
  // just-defined HAND_ABS_POS_REV reference.
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);      // not even the preamble reached the bus
}

TEST_CASE("a dead CAN3 refuses the traj (ERR_BUS_DOWN, nothing sent)") {
  fake_reset();
  fake_set_commands_allowed(false);   // never-command-a-dead-bus

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_BUS_DOWN);
  CHECK(fake_sent_count() == 0);      // not even the preamble reached the bus
}

TEST_CASE("a failed FIRST preamble send aborts the traj (no controller-mode, no 0x6D0)") {
  fake_reset();
  fake_set_commands_allowed(true);
  fake_set_send_fail_index(0);        // set_state fails to enqueue

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT);
  // The failed send is not recorded, and the abort short-circuits before the
  // controller-mode preamble AND the 0x6D0 forward.
  CHECK(fake_sent_count() == 0);
}

TEST_CASE("a failed SECOND preamble send aborts the traj (no 0x6D0 frame)") {
  fake_reset();
  fake_set_commands_allowed(true);
  fake_set_send_fail_index(1);        // set_controller_mode fails to enqueue

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT);
  // Only the first preamble (set_state) was recorded; the controller-mode send
  // failed and the 0x6D0 frame was NEVER attempted — the crux: a trajectory must
  // not run against a hand that is not in CLOSED_LOOP/PASSTHROUGH.
  REQUIRE(fake_sent_count() == 1);
  for (size_t i = 0; i < fake_sent_count(); ++i) {
    CHECK(fake_sent_at(i).id != PlatformCanId::TRAJ_CMD);
  }
}
