// =============================================================================
//  test_leg_deactivate.cpp — compiled test of the REAL firmware deactivate ladder
// =============================================================================
//  Drives the actual compiled leg_deactivate.cpp against the cold-start fake HAL,
//  asserting the DEACTIVATE behaviours. It mirrors the
//  activate driver (same request validation + SETUP preamble + abort-to-IDLE) and
//  adds the two behaviours that DISTINGUISH deactivate:
//
//    * COMMAND targets the STOW pose (STOW_OFF_POSE_REV), not the active pose;
//    * on MONITOR completion the leg is dropped to IDLE (deactivate SAFES the robot,
//      ending de-energised — unlike activate, which holds the active pose in
//      CLOSED_LOOP). This IDLE-on-arrival is the one behavioural difference and the
//      whole point of the op.
//
//  Sibling predicates (homing_active / activate_active) inline; the real
//  deactivate_active() comes from the #included module. SCOPE: decision logic only.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "udp_protocol.h"
#include "protocol_config.h"
#include "odrive_protocol.h"
#include "canbridge_config.h"
#include "hardware_config.h"
#include "axis_state.h"
#include "can_buses.h"
#include "coldstart_hal.h"

// ── Sibling cold-start predicates (the two DEACTIVATE excludes) ───────────────
namespace CanBridge {
static bool g_sib_homing = false;
static bool g_sib_activate = false;
bool homing_active()   { return g_sib_homing; }
bool activate_active() { return g_sib_activate; }
}

#include "leg_deactivate.cpp"   // the unit under test

using namespace CanBridge;

// The mask spans all NUM_AXES: bit HAND_AXIS (0x40) marks the hand present.
static void present_only(uint8_t mask) {
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    axes[i].heartbeat_seen = (mask & (uint8_t)(1u << i)) != 0;
    axes[i].active_errors = 0;
    axes[i].pos_rev = 0.0f;
    axes[i].vel_rps = 0.0f;
  }
}

static void full_reset(uint8_t present_mask) {
  cs_reset();
  g_sib_homing = false;
  g_sib_activate = false;
  present_only(present_mask);
  deactivate_init();
}

// ── request validation ────────────────────────────────────────────────────────

TEST_CASE("deactivate_request refuses a dead bus / can-down / E-STOP (ERR_BUS_DOWN)") {
  full_reset(0x01);
  cs_set_jugglebot_health(JbUdp::BusHealth::BUS_OFF);
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::ERR_BUS_DOWN);
  CHECK_FALSE(deactivate_active());

  full_reset(0x01);
  cs_set_can_bus_down(true);
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::ERR_BUS_DOWN);

  full_reset(0x01);
  cs_set_guard_estop(true);
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::ERR_BUS_DOWN);
}

TEST_CASE("deactivate_request rejects a concurrent HOME or ACTIVATE") {
  full_reset(0x01);
  g_sib_homing = true;
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::ERR_REJECTED);

  full_reset(0x01);
  g_sib_activate = true;
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
}

TEST_CASE("deactivate_request rejects while the MPC stream is actively driving (interlock)") {
  full_reset(0x01);
  cs_set_mpc_active(true);                               // guard ENABLED on the Jetson → MPC driving legs
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK_FALSE(deactivate_active());
  cs_set_mpc_active(false);
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::OK);  // OK once the MPC stops driving
  CHECK(deactivate_active());
}

TEST_CASE("deactivate_request rejects while a deferred stow is already safing the platform (review fix)") {
  full_reset(0x01);
  cs_set_stow_pending(true);
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK_FALSE(deactivate_active());
  cs_set_stow_pending(false);
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::OK);
}

TEST_CASE("deactivate_request rejects no-present-leg (ERR_BAD_ARGS) and is idempotent") {
  full_reset(0x00);
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::ERR_BAD_ARGS);

  full_reset(0x01);
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::OK);
  CHECK(deactivate_active());
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::ERR_REJECTED);   // no second latch
}

// ── SETUP preamble byte-parity (same 5-frame preamble as activate) ────────────

TEST_CASE("deactivate SETUP emits the 5-frame per-leg preamble byte-identical to the encoders") {
  full_reset(0x01);
  axes[0].pos_rev = 2.19f;         // seeded at the active pose (descent start)
  REQUIRE(deactivate_request(0) == JbUdp::RpcStatus::OK);

  cs_clear_sent();
  deactivate_step();

  REQUIRE(cs_sent_count() == 5);
  const auto f0 = ODrive::encode_set_input_pos(0, ODrive::leg_sign(0, 2.19f), 0, 0);
  CHECK(cs_sent_at(0).id == f0.id);
  CHECK(memcmp(cs_sent_at(0).buf, f0.buf, 8) == 0);
  const auto f3 = ODrive::encode_set_controller_mode(
      0, ODriveControlMode::POSITION, ODriveInputMode::TRAP_TRAJ);
  CHECK(cs_sent_at(3).id == f3.id);
  CHECK(memcmp(cs_sent_at(3).buf, f3.buf, 8) == 0);
  const auto f4 = ODrive::encode_set_state(0, ODriveState::CLOSED_LOOP);
  CHECK(cs_sent_at(4).id == f4.id);
  CHECK(memcmp(cs_sent_at(4).buf, f4.buf, 8) == 0);
}

// ── COMMAND targets the STOW pose (the activate/deactivate difference #1) ──────

TEST_CASE("deactivate COMMAND descends to STOW_OFF_POSE_REV (not the active pose)") {
  full_reset(0x01);
  axes[0].pos_rev = 2.19f;
  REQUIRE(deactivate_request(0) == JbUdp::RpcStatus::OK);
  deactivate_step();               // SETUP
  deactivate_step();               // SETUP cursor → COMMAND
  cs_advance(20000);               // > settle
  cs_clear_sent();
  deactivate_step();               // COMMAND

  REQUIRE(cs_sent_count() == 1);
  const auto tgt = ODrive::encode_leg_setpoint(0, STOW_OFF_POSE_REV, 0.0f, 0.0f);
  CHECK(cs_sent_at(0).id == tgt.id);
  CHECK(memcmp(cs_sent_at(0).buf, tgt.buf, 8) == 0);
}

// ── MONITOR completion IDLEs the leg (the difference #2 — safes the robot) ─────

TEST_CASE("deactivate IDLEs the leg on arrival at STOW (de-energised end-state)") {
  full_reset(0x01);
  axes[0].pos_rev = 2.19f;
  REQUIRE(deactivate_request(0) == JbUdp::RpcStatus::OK);
  deactivate_step();               // SETUP
  deactivate_step();               // → COMMAND
  cs_advance(20000);
  deactivate_step();               // COMMAND → MONITOR

  // The leg reaches STOW and settles.
  axes[0].pos_rev = STOW_OFF_POSE_REV;
  axes[0].vel_rps = 0.0f;
  cs_clear_sent();
  deactivate_step();               // MONITOR sees arrival → IDLE + OK

  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(0) == DEACTIVATE_OK);
  REQUIRE(cs_sent_count() == 1);
  const auto idle = ODrive::encode_set_state(0, ODriveState::IDLE);
  CHECK(cs_sent_at(0).id == idle.id);
  CHECK(memcmp(cs_sent_at(0).buf, idle.buf, 8) == 0);
}

// ── Safety crux: any abort leaves the leg in IDLE ─────────────────────────────

TEST_CASE("a mid-descent CAN3 loss aborts deactivate and leaves the leg in IDLE") {
  full_reset(0x01);
  REQUIRE(deactivate_request(0) == JbUdp::RpcStatus::OK);
  deactivate_step();
  REQUIRE(deactivate_active());

  cs_set_jugglebot_health(JbUdp::BusHealth::BUS_OFF);
  cs_clear_sent();
  deactivate_step();

  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(0) == DEACTIVATE_FAILED);
  REQUIRE(cs_sent_count() == 1);
  const auto idle = ODrive::encode_set_state(0, ODriveState::IDLE);
  CHECK(cs_sent_at(0).id == idle.id);
}


// ── skill-stack R1: DEACTIVATE de-energises the HAND, it does not lower it ─────
//  ACTIVATE energises axis 6 (owner decision 2026-09-11), so DEACTIVATE owns the
//  other half. The choreography is deliberately ASYMMETRIC with the legs: a leg
//  carries the platform and must descend under profile; the hand's carriage hangs
//  on a spool and gravity takes it to the bottom stop the instant the axis IDLEs.
//  So the hand IDLEs in the first tick, before the legs move, and then leaves the
//  target mask — it is never in SETUP/COMMAND/MONITOR and a later leg abort finds
//  it already safe.

static constexpr uint8_t HAND_BIT = (uint8_t)(1u << HAND_AXIS);

TEST_CASE("deactivate IDLEs the hand in the first tick, before any leg frame") {
  full_reset((uint8_t)(0x01 | HAND_BIT));     // leg 0 + hand
  axes[0].pos_rev = JBOp::ACTIVATE_POSITION_REVS[0];
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);

  cs_clear_sent();
  deactivate_step();                          // consume: hand IDLE, then SETUP leg 0

  REQUIRE(cs_sent_count() >= 1);
  const auto idle = ODrive::encode_set_state(HAND_AXIS, ODriveState::IDLE);
  CHECK(cs_sent_at(0).id == idle.id);         // FIRST frame of the whole op
  CHECK(memcmp(cs_sent_at(0).buf, idle.buf, 8) == 0);
  CHECK(deactivate_result(HAND_AXIS) == DEACTIVATE_OK);
  CHECK(deactivate_active());                 // the legs are still descending
}

TEST_CASE("the hand never enters the descent ladder (no TRAP_TRAJ, no STOW target)") {
  full_reset((uint8_t)(0x01 | HAND_BIT));
  axes[0].pos_rev = JBOp::ACTIVATE_POSITION_REVS[0];
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);

  cs_clear_sent();
  deactivate_step();      // hand IDLE + SETUP leg 0
  deactivate_step();      // cursor off the end → COMMAND
  cs_advance(20000);
  deactivate_step();      // COMMAND fires the leg target

  // Exactly one hand frame in the whole op: the IDLE. No CLOSED_LOOP, no mode
  // change, no STOW setpoint on axis 6.
  const auto hand_cl   = ODrive::encode_set_state(HAND_AXIS, ODriveState::CLOSED_LOOP);
  const auto hand_stow = ODrive::encode_leg_setpoint(HAND_AXIS, STOW_OFF_POSE_REV, 0.0f, 0.0f);
  const auto hand_trap = ODrive::encode_set_controller_mode(
      HAND_AXIS, ODriveControlMode::POSITION, ODriveInputMode::TRAP_TRAJ);
  for (size_t i = 0; i < cs_sent_count(); ++i) {
    CHECK_FALSE((cs_sent_at(i).id == hand_cl.id &&
                 memcmp(cs_sent_at(i).buf, hand_cl.buf, 8) == 0));
    CHECK_FALSE((cs_sent_at(i).id == hand_stow.id &&
                 memcmp(cs_sent_at(i).buf, hand_stow.buf, 8) == 0));
    CHECK_FALSE((cs_sent_at(i).id == hand_trap.id &&
                 memcmp(cs_sent_at(i).buf, hand_trap.buf, 8) == 0));
  }
  // 1 hand IDLE + 5 leg preamble + 1 leg target
  CHECK(cs_sent_count() == 7);
}

TEST_CASE("a hand-only deactivate completes in one tick") {
  full_reset(HAND_BIT);
  REQUIRE(deactivate_request(HAND_AXIS) == JbUdp::RpcStatus::OK);
  cs_clear_sent();
  deactivate_step();

  CHECK_FALSE(deactivate_active());           // nothing left in the ladder
  CHECK(deactivate_result(HAND_AXIS) == DEACTIVATE_OK);
  REQUIRE(cs_sent_count() == 1);
  const auto idle = ODrive::encode_set_state(HAND_AXIS, ODriveState::IDLE);
  CHECK(cs_sent_at(0).id == idle.id);
}

TEST_CASE("deactivate AXIS_ALL on a hand-less bench still lowers the legs") {
  full_reset(0x3F);                            // six legs, no hand
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  deactivate_step();
  CHECK(deactivate_result(HAND_AXIS) == DEACTIVATE_NONE);   // never targeted
  CHECK(deactivate_result(0) == DEACTIVATE_RUNNING);
  CHECK(deactivate_active());
}

TEST_CASE("a leg abort after the hand has IDLE'd leaves the hand safe (already OK)") {
  full_reset((uint8_t)(0x01 | HAND_BIT));
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  deactivate_step();                           // hand IDLE + SETUP leg 0
  cs_set_guard_estop(true);
  deactivate_step();                           // abort path

  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(0) == DEACTIVATE_FAILED);
  CHECK(deactivate_result(HAND_AXIS) == DEACTIVATE_OK);   // de-energised before the abort
}
