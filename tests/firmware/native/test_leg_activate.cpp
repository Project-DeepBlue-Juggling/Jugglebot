// =============================================================================
//  test_leg_activate.cpp — compiled test of the REAL firmware activate ladder
// =============================================================================
//  Drives the actual compiled leg_activate.cpp (#includes the .cpp) against the
//  self-contained cold-start fake HAL, asserting the safety-relevant BEHAVIOURS of
//  the ACTIVATE op that no test compiled before:
//
//    * activate_request VALIDATION: never latches on a dead/WARN CAN3, an E-STOP'd
//      guard, or a fault_can_bus_down; rejects a concurrent HOME/DEACTIVATE; rejects
//      when no target leg is present; is idempotent (re-request while active fails);
//    * the SETUP ladder emits the exact 5-frame per-leg preamble (seed input_pos at
//      the NON-clipped current pos, traj vel/acc limits, POSITION/TRAP_TRAJ mode,
//      CLOSED_LOOP) — byte-identical to the ODrive encoders, ONE leg per tick;
//    * COMMAND emits encode_leg_setpoint(ACTIVATE_POSITION_REVS) after the settle;
//    * the safety crux: ANY abort (bus down / E-STOP / timeout) leaves every target
//      leg in IDLE — the move is NEVER left driving.
//
//  The two sibling cold-start predicates (homing_active / deactivate_active) are
//  supplied inline here (controllable) — the real activate_active() comes from the
//  #included leg_activate.cpp. SCOPE: decision logic, not FreeRTOS/500 Hz timing.
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

// ── Sibling cold-start predicates (the two ACTIVATE excludes) ─────────────────
//  activate_request rejects a concurrent HOME/DEACTIVATE; a test flips these to
//  drive the mutual-exclusion paths. The REAL activate_active() is defined by the
//  #included leg_activate.cpp below.
namespace CanBridge {
static bool g_sib_homing = false;
static bool g_sib_deactivate = false;
bool homing_active()     { return g_sib_homing; }
bool deactivate_active() { return g_sib_deactivate; }
}

#include "leg_activate.cpp"   // the unit under test

using namespace CanBridge;

// Mark axes present (leg_present() == heartbeat_seen) and reset per-axis state.
// The mask spans all NUM_AXES now — bit HAND_AXIS (0x40) marks the hand present,
// so a test can drive the hand-less bench (legs only) and the full robot alike.
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
  g_sib_deactivate = false;
  present_only(present_mask);
  activate_init();
}

// ── request validation ────────────────────────────────────────────────────────

TEST_CASE("activate_request refuses a dead/WARN CAN3 (ERR_BUS_DOWN, nothing latched)") {
  full_reset(0x01);   // leg 0 present
  cs_set_jugglebot_health(JbUdp::BusHealth::BUS_OFF);
  CHECK(activate_request(0) == JbUdp::RpcStatus::ERR_BUS_DOWN);
  CHECK_FALSE(activate_active());

  full_reset(0x01);
  cs_set_jugglebot_health(JbUdp::BusHealth::WARN);
  CHECK(activate_request(0) == JbUdp::RpcStatus::ERR_BUS_DOWN);
  CHECK_FALSE(activate_active());
}

TEST_CASE("activate_request refuses a fault_can_bus_down and an E-STOP'd guard") {
  full_reset(0x01);
  cs_set_can_bus_down(true);
  CHECK(activate_request(0) == JbUdp::RpcStatus::ERR_BUS_DOWN);
  CHECK_FALSE(activate_active());

  full_reset(0x01);
  cs_set_guard_estop(true);
  CHECK(activate_request(0) == JbUdp::RpcStatus::ERR_BUS_DOWN);
  CHECK_FALSE(activate_active());
}

TEST_CASE("activate_request rejects a concurrent HOME or DEACTIVATE (mutual exclusion)") {
  full_reset(0x01);
  g_sib_homing = true;
  CHECK(activate_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK_FALSE(activate_active());

  full_reset(0x01);
  g_sib_deactivate = true;
  CHECK(activate_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK_FALSE(activate_active());
}

TEST_CASE("activate_request rejects while the MPC stream is actively driving (interlock)") {
  full_reset(0x01);
  cs_set_mpc_active(true);                               // guard ENABLED on the Jetson → MPC driving legs
  CHECK(activate_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK_FALSE(activate_active());
  cs_set_mpc_active(false);
  CHECK(activate_request(0) == JbUdp::RpcStatus::OK);    // OK once the MPC stops driving
  CHECK(activate_active());
}

TEST_CASE("activate_request rejects while a deferred stow is pending (review fix)") {
  full_reset(0x01);
  cs_set_stow_pending(true);
  CHECK(activate_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK_FALSE(activate_active());
  cs_set_stow_pending(false);
  CHECK(activate_request(0) == JbUdp::RpcStatus::OK);
}

TEST_CASE("activate_request rejects when no target leg is present (ERR_BAD_ARGS)") {
  full_reset(0x00);   // nothing present
  CHECK(activate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK(activate_request(0) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK_FALSE(activate_active());
}

TEST_CASE("activate_request accepts a valid request and is idempotent") {
  full_reset(0x01);
  CHECK(activate_request(0) == JbUdp::RpcStatus::OK);
  CHECK(activate_active());
  // Re-request while active → rejected (no second latch).
  CHECK(activate_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK(activate_active());
}

// ── SETUP ladder: per-leg preamble byte-parity, one leg per tick ──────────────

TEST_CASE("activate SETUP emits the 5-frame per-leg preamble byte-identical to the encoders") {
  full_reset(0x01);            // leg 0 present
  axes[0].pos_rev = -0.10f;    // seeded at the sub-zero hardstop (NON-clipped)
  REQUIRE(activate_request(0) == JbUdp::RpcStatus::OK);

  cs_clear_sent();
  activate_step();             // consumes the start + configures leg 0 this tick

  REQUIRE(cs_sent_count() == 5);
  const auto f0 = ODrive::encode_set_input_pos(0, ODrive::leg_sign(0, -0.10f), 0, 0);
  CHECK(cs_sent_at(0).id == f0.id);
  CHECK(memcmp(cs_sent_at(0).buf, f0.buf, 8) == 0);
  const auto f1 = ODrive::encode_set_traj_vel_limit(0, JBOp::GENTLE_MOVE_VEL_LIMIT_RPS);
  CHECK(cs_sent_at(1).id == f1.id);
  CHECK(memcmp(cs_sent_at(1).buf, f1.buf, 8) == 0);
  const auto f2 = ODrive::encode_set_traj_acc_limits(
      0, ODriveDefaults::TRAP_ACC_LIMIT_RPS2, ODriveDefaults::TRAP_DEC_LIMIT_RPS2);
  CHECK(cs_sent_at(2).id == f2.id);
  CHECK(memcmp(cs_sent_at(2).buf, f2.buf, 8) == 0);
  const auto f3 = ODrive::encode_set_controller_mode(
      0, ODriveControlMode::POSITION, ODriveInputMode::TRAP_TRAJ);
  CHECK(cs_sent_at(3).id == f3.id);
  CHECK(memcmp(cs_sent_at(3).buf, f3.buf, 8) == 0);
  const auto f4 = ODrive::encode_set_state(0, ODriveState::CLOSED_LOOP);
  CHECK(cs_sent_at(4).id == f4.id);
  CHECK(memcmp(cs_sent_at(4).buf, f4.buf, 8) == 0);
}

TEST_CASE("activate SETUP aborts (leaves the leg errored, no CLOSED_LOOP) on a pre-existing ODrive error") {
  full_reset(0x01);
  axes[0].active_errors = 0x40;   // legacy _gentle_move parity: refuse to arm an errored leg
  REQUIRE(activate_request(0) == JbUdp::RpcStatus::OK);

  cs_clear_sent();
  activate_step();                // SETUP error gate fires before any drive frame

  CHECK_FALSE(activate_active());
  CHECK(activate_result(0) == ACTIVATE_FAILED);
  // Never a CLOSED_LOOP on an errored leg (the error gate fires first).
  const auto cl = ODrive::encode_set_state(0, ODriveState::CLOSED_LOOP);
  bool saw_closed_loop = false;
  for (size_t i = 0; i < cs_sent_count(); ++i)
    if (cs_sent_at(i).id == cl.id && memcmp(cs_sent_at(i).buf, cl.buf, 8) == 0)
      saw_closed_loop = true;
  CHECK_FALSE(saw_closed_loop);
}

// ── COMMAND: the active-pose target after the settle ──────────────────────────

TEST_CASE("activate COMMAND emits the active-pose leg_setpoint after SETUP + settle") {
  full_reset(0x01);
  axes[0].pos_rev = -0.10f;
  REQUIRE(activate_request(0) == JbUdp::RpcStatus::OK);
  activate_step();                 // SETUP configures leg 0
  activate_step();                 // SETUP cursor runs off the end → COMMAND (settle clock starts)
  cs_advance(20000);               // > SETTLE_SETUP_US (10 ms)
  cs_clear_sent();
  activate_step();                 // COMMAND fires the target

  REQUIRE(cs_sent_count() == 1);
  const auto tgt = ODrive::encode_leg_setpoint(0, JBOp::ACTIVATE_POSITION_REVS[0], 0.0f, 0.0f);
  CHECK(cs_sent_at(0).id == tgt.id);
  CHECK(memcmp(cs_sent_at(0).buf, tgt.buf, 8) == 0);
}

// ── Safety crux: any abort leaves the targeted legs in IDLE ────────────────────

TEST_CASE("a mid-ladder CAN3 loss aborts activate and leaves the leg in IDLE") {
  full_reset(0x01);
  REQUIRE(activate_request(0) == JbUdp::RpcStatus::OK);
  activate_step();                 // enter SETUP / configure leg 0
  REQUIRE(activate_active());

  cs_set_jugglebot_health(JbUdp::BusHealth::BUS_OFF);  // bus dies mid-move
  cs_clear_sent();
  activate_step();                 // abort path

  CHECK_FALSE(activate_active());
  CHECK(activate_result(0) == ACTIVATE_FAILED);
  REQUIRE(cs_sent_count() == 1);
  const auto idle = ODrive::encode_set_state(0, ODriveState::IDLE);
  CHECK(cs_sent_at(0).id == idle.id);
  CHECK(memcmp(cs_sent_at(0).buf, idle.buf, 8) == 0);
}

TEST_CASE("an overall timeout aborts activate and leaves the leg in IDLE") {
  full_reset(0x01);
  REQUIRE(activate_request(0) == JbUdp::RpcStatus::OK);
  activate_step();                 // enter SETUP
  REQUIRE(activate_active());

  cs_advance((uint64_t)(JBOp::GENTLE_MOVE_TIMEOUT_S * 1.0e6f) + 1000000ull);
  cs_clear_sent();
  activate_step();                 // timeout abort

  CHECK_FALSE(activate_active());
  CHECK(activate_result(0) == ACTIVATE_FAILED);
  REQUIRE(cs_sent_count() == 1);
  const auto idle = ODrive::encode_set_state(0, ODriveState::IDLE);
  CHECK(cs_sent_at(0).id == idle.id);
}


// ── skill-stack R1: ACTIVATE energises and parks the HAND (axis 6) ─────────────
//  Owner decision 2026-09-11: there is one hand master (the streamed lane in
//  leg_interp.cpp) and ACTIVATE is its handover. The operator's `--close-loop`
//  energise step retires, so these behaviours are now safety-relevant:
//    * AXIS_ALL targets the hand when it is present, and does NOT when it is not
//      (a hand-less bench must still raise its six legs);
//    * the hand's target is JBOp::HAND_ACTIVATE_POSITION_REV, NEVER a leg's
//      ACTIVATE_POSITION_REVS[6] (there is no such element);
//    * the op ENDS with axis 6 in POSITION/PASSTHROUGH — leaving it in TRAP_TRAJ
//      is the FW 18 silently-inert-lane fault in a new dress;
//    * a failed hand-off fails the activate (never a quiet ACTIVATE_OK).

static constexpr uint8_t HAND_BIT = (uint8_t)(1u << HAND_AXIS);

// Run the ladder to the point where the target(s) have been commanded.
static void run_to_monitor(uint8_t n_setup_ticks) {
  for (uint8_t k = 0; k < n_setup_ticks; ++k) activate_step();  // one axis per tick
  activate_step();            // cursor runs off the end → COMMAND (settle starts)
  cs_advance(20000);          // > SETTLE_SETUP_US
  activate_step();            // COMMAND fires the targets
}

TEST_CASE("activate AXIS_ALL includes the hand when present and skips it when absent") {
  full_reset((uint8_t)(0x3F | HAND_BIT));      // six legs + hand
  REQUIRE(activate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  activate_step();                              // consume + SETUP leg 0
  CHECK(activate_result(HAND_AXIS) == ACTIVATE_RUNNING);

  full_reset(0x3F);                             // hand-less bench: six legs only
  REQUIRE(activate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  activate_step();
  CHECK(activate_result(HAND_AXIS) == ACTIVATE_NONE);   // never targeted
  CHECK(activate_result(0) == ACTIVATE_RUNNING);        // the legs still activate
}

TEST_CASE("activate refuses the hand axis when the hand is absent (ERR_BAD_ARGS)") {
  full_reset(0x3F);
  CHECK(activate_request(HAND_AXIS) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK_FALSE(activate_active());
}

TEST_CASE("activate SETUP gives the hand the same gentle TRAP_TRAJ preamble, seeded off the stop") {
  full_reset(HAND_BIT);
  axes[HAND_AXIS].pos_rev = Homing::HAND_ABS_POS_REV;   // homed, on the stop (-0.10)
  REQUIRE(activate_request(HAND_AXIS) == JbUdp::RpcStatus::OK);

  cs_clear_sent();
  activate_step();

  REQUIRE(cs_sent_count() == 5);
  const auto f0 = ODrive::encode_set_input_pos(
      HAND_AXIS, ODrive::leg_sign(HAND_AXIS, Homing::HAND_ABS_POS_REV), 0, 0);
  CHECK(cs_sent_at(0).id == f0.id);
  CHECK(memcmp(cs_sent_at(0).buf, f0.buf, 8) == 0);
  const auto f1 = ODrive::encode_set_traj_vel_limit(HAND_AXIS, JBOp::GENTLE_MOVE_VEL_LIMIT_RPS);
  CHECK(cs_sent_at(1).id == f1.id);
  CHECK(memcmp(cs_sent_at(1).buf, f1.buf, 8) == 0);
  const auto f3 = ODrive::encode_set_controller_mode(
      HAND_AXIS, ODriveControlMode::POSITION, ODriveInputMode::TRAP_TRAJ);
  CHECK(cs_sent_at(3).id == f3.id);
  CHECK(memcmp(cs_sent_at(3).buf, f3.buf, 8) == 0);
  const auto f4 = ODrive::encode_set_state(HAND_AXIS, ODriveState::CLOSED_LOOP);
  CHECK(cs_sent_at(4).id == f4.id);
  CHECK(memcmp(cs_sent_at(4).buf, f4.buf, 8) == 0);
}

TEST_CASE("activate COMMAND parks the hand at HAND_ACTIVATE_POSITION_REV") {
  full_reset(HAND_BIT);
  axes[HAND_AXIS].pos_rev = Homing::HAND_ABS_POS_REV;
  REQUIRE(activate_request(HAND_AXIS) == JbUdp::RpcStatus::OK);
  cs_clear_sent();
  run_to_monitor(1);

  REQUIRE(cs_sent_count() == 6);      // 5 preamble + 1 target
  const auto tgt = ODrive::encode_leg_setpoint(
      HAND_AXIS, JBOp::HAND_ACTIVATE_POSITION_REV, 0.0f, 0.0f);
  CHECK(cs_sent_at(5).id == tgt.id);
  CHECK(memcmp(cs_sent_at(5).buf, tgt.buf, 8) == 0);
}

TEST_CASE("activate ends the hand in POSITION/PASSTHROUGH — the streamed lane's mode") {
  full_reset(HAND_BIT);
  axes[HAND_AXIS].pos_rev = Homing::HAND_ABS_POS_REV;
  REQUIRE(activate_request(HAND_AXIS) == JbUdp::RpcStatus::OK);
  run_to_monitor(1);
  REQUIRE(activate_active());                       // MONITOR: not there yet

  axes[HAND_AXIS].pos_rev = JBOp::HAND_ACTIVATE_POSITION_REV;   // arrived + settled
  axes[HAND_AXIS].vel_rps = 0.0f;
  cs_clear_sent();
  activate_step();

  CHECK_FALSE(activate_active());
  CHECK(activate_result(HAND_AXIS) == ACTIVATE_OK);
  REQUIRE(cs_sent_count() == 1);                    // the handover frame, nothing else
  const auto pt = ODrive::encode_set_controller_mode(
      HAND_AXIS, ODriveControlMode::POSITION, ODriveInputMode::PASSTHROUGH);
  CHECK(cs_sent_at(0).id == pt.id);
  CHECK(memcmp(cs_sent_at(0).buf, pt.buf, 8) == 0);
  CHECK(axes[HAND_AXIS].input_mode == (uint8_t)ODriveInputMode::PASSTHROUGH);
}

TEST_CASE("a failed hand hand-off FAILS the activate and IDLEs the hand") {
  full_reset(HAND_BIT);
  axes[HAND_AXIS].pos_rev = Homing::HAND_ABS_POS_REV;
  REQUIRE(activate_request(HAND_AXIS) == JbUdp::RpcStatus::OK);
  run_to_monitor(1);
  axes[HAND_AXIS].pos_rev = JBOp::HAND_ACTIVATE_POSITION_REV;
  axes[HAND_AXIS].vel_rps = 0.0f;

  cs_set_send_fail_index(0);          // the hand-off mode frame cannot go out
  activate_step();
  cs_set_send_fail_index(-1);

  CHECK_FALSE(activate_active());
  CHECK(activate_result(HAND_AXIS) == ACTIVATE_FAILED);   // never a quiet OK
}

TEST_CASE("the legs' activate is unchanged when the hand rides along") {
  full_reset((uint8_t)(0x01 | HAND_BIT));    // leg 0 + hand
  axes[0].pos_rev = -0.10f;
  axes[HAND_AXIS].pos_rev = Homing::HAND_ABS_POS_REV;
  REQUIRE(activate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  cs_clear_sent();
  run_to_monitor(2);                         // one SETUP tick per present axis

  // 5 + 5 preamble frames, then BOTH targets in the one COMMAND tick.
  REQUIRE(cs_sent_count() == 12);
  const auto leg_tgt = ODrive::encode_leg_setpoint(0, JBOp::ACTIVATE_POSITION_REVS[0], 0.0f, 0.0f);
  CHECK(cs_sent_at(10).id == leg_tgt.id);
  CHECK(memcmp(cs_sent_at(10).buf, leg_tgt.buf, 8) == 0);
  const auto hand_tgt = ODrive::encode_leg_setpoint(
      HAND_AXIS, JBOp::HAND_ACTIVATE_POSITION_REV, 0.0f, 0.0f);
  CHECK(cs_sent_at(11).id == hand_tgt.id);
  CHECK(memcmp(cs_sent_at(11).buf, hand_tgt.buf, 8) == 0);
}

TEST_CASE("an abort with the hand targeted leaves the HAND in IDLE too") {
  full_reset((uint8_t)(0x01 | HAND_BIT));
  REQUIRE(activate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  activate_step();
  cs_set_guard_estop(true);                  // E-STOP mid-ladder
  cs_clear_sent();
  activate_step();

  CHECK_FALSE(activate_active());
  CHECK(activate_result(HAND_AXIS) == ACTIVATE_FAILED);
  const auto idle = ODrive::encode_set_state(HAND_AXIS, ODriveState::IDLE);
  bool saw_hand_idle = false;
  for (size_t i = 0; i < cs_sent_count(); ++i)
    if (cs_sent_at(i).id == idle.id && memcmp(cs_sent_at(i).buf, idle.buf, 8) == 0)
      saw_hand_idle = true;
  CHECK(saw_hand_idle);
}
