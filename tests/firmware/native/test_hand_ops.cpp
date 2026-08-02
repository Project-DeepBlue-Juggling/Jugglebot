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
//      trajectory against a hand not in CLOSED_LOOP/PASSTHROUGH would fault/no-op);
//    * every exit is ATTRIBUTED to the right HandOpsCounters slot (2026-08-02).
//
//  Why the counters are tested here and not only on the wire: hand_traj_cmd has
//  five failure exits and THREE of them return an identical bare ERR_TIMEOUT, so
//  the return value cannot distinguish them and neither could the 2026-08-01
//  recount. The counters are the only thing that can, which makes "the right one
//  incremented" the actual contract — a copy-paste that bumped pre1 from the pre2
//  branch would be invisible in every other test in this file and would send the
//  next bench session after the wrong CAN send. Each case calls
//  hand_ops_counters_reset() so the assertions are absolute rather than deltas
//  and stay order-independent (the version_check_init() isolation pattern).
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

// Assert the WHOLE counter block, not just the field a case cares about: the
// failure mode worth catching is an increment landing in the wrong slot, and
// checking only the expected slot cannot see that. `ok` is derived exactly as
// the /link_status row derives it, so the identity
//   calls - rej - down - pre1 - pre2 - traj == ok
// is under test too.
static void check_counters(uint32_t calls, uint32_t rej, uint32_t down,
                           uint32_t pre1, uint32_t pre2, uint32_t traj,
                           uint32_t ok) {
  const auto c = hand_ops_counters();
  CHECK(c.calls == calls);
  CHECK(c.rej_homing == rej);
  CHECK(c.bus_down == down);
  CHECK(c.pre1_fail == pre1);
  CHECK(c.pre2_fail == pre2);
  CHECK(c.traj_fail == traj);
  CHECK(c.calls - c.rej_homing - c.bus_down - c.pre1_fail - c.pre2_fail
        - c.traj_fail == ok);
}

TEST_CASE("valid hand traj emits the CLOSED_LOOP + PASSTHROUGH preamble then the 0x6D0 frame") {
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);
  // The success path increments ONLY calls — OK is derived, never counted.
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/0, /*ok=*/1);

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
  hand_ops_counters_reset();
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
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);    // bus is fine — the rejection is the homing interlock, not the gate
  fake_set_homing(true);              // the shared state machine is mid-homing (e.g. axis 6)

  const auto a = make_traj_arg();
  // Checked FIRST — before the bus gate and before any preamble frame reaches CAN3.
  // A catch-traj mid-homing would fight the axis-6 move-to-hardstop and corrupt the
  // just-defined HAND_ABS_POS_REV reference.
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);      // not even the preamble reached the bus
  // A refused call still counts as a call: `calls` is the denominator, so a gate
  // that rejects everything must read as traffic, not as silence.
  check_counters(/*calls=*/1, /*rej=*/1, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/0, /*ok=*/0);
}

TEST_CASE("a dead CAN3 refuses the traj (ERR_BUS_DOWN, nothing sent)") {
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(false);   // never-command-a-dead-bus

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_BUS_DOWN);
  CHECK(fake_sent_count() == 0);      // not even the preamble reached the bus
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/1,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/0, /*ok=*/0);
}

TEST_CASE("a failed FIRST preamble send aborts the traj (no controller-mode, no 0x6D0)") {
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  fake_set_send_fail_index(0);        // set_state fails to enqueue

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT);
  // The failed send is not recorded, and the abort short-circuits before the
  // controller-mode preamble AND the 0x6D0 forward.
  CHECK(fake_sent_count() == 0);
  // The whole point of the split: this ERR_TIMEOUT and the next two cases' are
  // byte-identical on the wire, and ONLY the counter says which send refused.
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/0,
                 /*pre1=*/1, /*pre2=*/0, /*traj=*/0, /*ok=*/0);
}

TEST_CASE("a failed SECOND preamble send aborts the traj (no 0x6D0 frame)") {
  fake_reset();
  hand_ops_counters_reset();
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
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/1, /*traj=*/0, /*ok=*/0);
}

TEST_CASE("a failed 0x6D0 send is attributed to the TRAJ stage, not to a preamble") {
  // The third send had NO case at all before 2026-08-02, which is the exit an
  // arm-ack failure most plausibly comes from: by then both preambles have
  // consumed TX mailboxes, so the traj frame is the one most likely to find the
  // queue full. Both preambles must still have gone out — a traj-stage failure
  // is NOT a preamble failure, and conflating them would point a bench session
  // at the hand's CLOSED_LOOP transition instead of at TX-queue pressure.
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  fake_set_send_fail_index(2);        // the 0x6D0 traj frame fails to enqueue

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT);
  REQUIRE(fake_sent_count() == 2);    // both preambles landed; the traj did not
  for (size_t i = 0; i < fake_sent_count(); ++i) {
    CHECK(fake_sent_at(i).id != PlatformCanId::TRAJ_CMD);
  }
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/1, /*ok=*/0);
}

TEST_CASE("counters accumulate across calls and stay attributed per stage") {
  // Cumulative-since-boot is the contract the wire relies on (the consumer
  // differences two captures), so a mixed sequence must sum correctly rather
  // than latch a last-writer-wins verdict — the exact weakness that ruled out
  // a single last_fail_stage byte.
  fake_reset();
  hand_ops_counters_reset();
  const auto a = make_traj_arg();

  fake_set_commands_allowed(true);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);          // ok #1

  fake_set_send_fail_index(2);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT); // traj #1

  fake_set_send_fail_index(0);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT); // pre1 #1

  fake_set_commands_allowed(false);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_BUS_DOWN); // down #1

  fake_set_commands_allowed(true);
  fake_set_send_fail_index(-1);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);          // ok #2

  check_counters(/*calls=*/5, /*rej=*/0, /*down=*/1,
                 /*pre1=*/1, /*pre2=*/0, /*traj=*/1, /*ok=*/2);
}

TEST_CASE("the reset seam zeroes every counter") {
  // Production never calls this; it exists so the cases above are absolute and
  // order-independent. If it silently missed a field, every earlier case would
  // start reading a neighbour's leftovers — a failure that presents as a
  // mis-attributed stage, i.e. as the exact bug these counters exist to catch.
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);
  fake_set_commands_allowed(false);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_BUS_DOWN);

  hand_ops_counters_reset();
  check_counters(/*calls=*/0, /*rej=*/0, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/0, /*ok=*/0);
}
