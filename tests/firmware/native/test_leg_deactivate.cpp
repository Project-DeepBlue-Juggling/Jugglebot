// =============================================================================
//  test_leg_deactivate.cpp — compiled test of the REAL firmware deactivate ladder
// =============================================================================
//  Drives the actual compiled leg_deactivate.cpp against the cold-start fake HAL,
//  asserting the DEACTIVATE behaviours. It mirrors the
//  activate driver (same request validation + SETUP preamble) and
//  adds the two behaviours that DISTINGUISH deactivate:
//
//    * COMMAND targets the STOW pose (STOW_OFF_POSE_REV), not the active pose;
//    * on MONITOR completion the leg is dropped to IDLE (deactivate SAFES the robot,
//      ending de-energised — unlike activate, which holds the active pose in
//      CLOSED_LOOP). This IDLE-on-arrival is the one behavioural difference and the
//      whole point of the op.
//
//  FW 53 (I-FW-19, owner rule 2026-11-25): every leg state change goes through
//  the Stow gate (leg_stow_gate.cpp, linked), so a leg is IDLE'd only from Stow
//  and an abort HOLDS (fw53_design.md 3.1). The fixture therefore stages every
//  present axis with a FRESH heartbeat and encoder (cs_stage_leg), and a case that
//  reaches arrival re-stages the legs with a heartbeat received after the ladder's
//  CLOSED_LOOP command + CL_ENGAGE_SETTLE_US (confirmed()).
//
//  Sibling predicates (homing_active / activate_active / homing_axis_in_flight)
//  inline; the real deactivate_active() comes from the #included module. SCOPE:
//  decision logic only.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "udp_protocol.h"
#include "protocol_config.h"
#include "odrive_protocol.h"
#include "canbridge_config.h"
#include "hardware_config.h"
#include "axis_state.h"
#include "can_buses.h"
#include "leg_stow_gate.h"
#include "coldstart_hal.h"

// ── Sibling cold-start predicates (the two DEACTIVATE excludes) ───────────────
namespace CanBridge {
static bool g_sib_homing = false;
static bool g_sib_activate = false;
bool homing_active()   { return g_sib_homing; }
bool activate_active() { return g_sib_activate; }
uint8_t homing_axis_in_flight() { return 0xFF; }   // leg_stow_gate.o needs it; no HOME here
}

#include "leg_deactivate.cpp"   // the unit under test

using namespace CanBridge;

static constexpr uint64_t T0 = 100000000ull;   // 100 s: every staged age fits below it
static constexpr int64_t  FRESH_HB  = 50000;   // 50 ms
static constexpr int64_t  FRESH_ENC = 2000;    // 2 ms
static constexpr uint32_t ST_IDLE = ODriveState::IDLE;
static constexpr uint32_t ST_CL   = ODriveState::CLOSED_LOOP;
static constexpr float    POSED   = 2.19f;     // the Active pose (rev)

// The mask spans all NUM_AXES: bit HAND_AXIS (0x40) marks the hand present. Each
// present axis is staged with a fresh heartbeat + encoder in `state` at `pos`.
static void present_only(uint8_t mask, uint32_t state = ST_IDLE, float pos = 0.0f) {
  cs_clear_axes();
  for (uint8_t i = 0; i < NUM_AXES; ++i)
    if (mask & (uint8_t)(1u << i)) cs_stage_leg(i, (uint8_t)state, pos, 0.0f, FRESH_HB, FRESH_ENC);
}

static void full_reset(uint8_t present_mask, uint32_t state = ST_IDLE, float pos = 0.0f) {
  cs_reset();
  cs_set_mono(T0);
  leg_stow_gate_init();
  g_sib_homing = false;
  g_sib_activate = false;
  present_only(present_mask, state, pos);
  deactivate_init();
}

// A heartbeat + encoder sample received NOW (so after the ladder's CL command once
// the clock has moved past it by CL_ENGAGE_SETTLE_US).
static void confirmed(uint8_t i, uint32_t state, float pos, float vel = 0.0f) {
  cs_stage_leg(i, (uint8_t)state, pos, vel, 0, 0);
}

// FW 53 U8: a CLOSED_LOOP heartbeat received NOW for every Leg in `mask`, keeping
// its staged position and velocity: what lets COMMAND's confirmation pass once the
// clock is CL_ENGAGE_SETTLE_US past each Leg's CLOSED_LOOP command.
static void cl_confirm(uint8_t mask) {
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (mask & (uint8_t)(1u << i))
      cs_stage_leg(i, (uint8_t)ST_CL, axes[i].pos_rev, axes[i].vel_rps, 0, 0);
}

// Request + step until the ladder is in COMMAND (one SETUP tick per target leg, one
// to reach COMMAND), sending no confirmation. Returns the clock of the last SETUP
// tick that configured a Leg (that Leg's CLOSED_LOOP command), read from the test's
// own clock, never from the gate. Ticks are 10 ms (HOMING_RATE_HZ).
static uint64_t run_to_command(uint8_t axis) {
  REQUIRE(deactivate_request(axis) == JbUdp::RpcStatus::OK);
  uint64_t t_last_cl = 0;
  for (int k = 0; k < 20 && s_phase != DPhase::COMMAND; ++k) {
    const uint64_t t = micros64();
    deactivate_step();
    // A SETUP tick that configured a Leg (s_setup_idx moved past it); not the
    // cursor-end tick, which enters COMMAND.
    if (s_phase == DPhase::SETUP && s_setup_idx >= 1 && s_setup_idx - 1 < NUM_LEGS) t_last_cl = t;
    cs_advance(10000);
  }
  REQUIRE(s_phase == DPhase::COMMAND);
  return t_last_cl;
}

// Request + step until the ladder is in MONITOR. FW 53 U8: once in COMMAND every
// target Leg is confirmed (a CLOSED_LOOP heartbeat after its CL command + settle),
// so the descent can be commanded.
static void run_to_monitor(uint8_t axis) {
  REQUIRE(deactivate_request(axis) == JbUdp::RpcStatus::OK);
  for (int k = 0; k < 20 && s_phase != DPhase::MONITOR; ++k) {
    deactivate_step();
    cs_advance(10000);
    if (s_phase == DPhase::COMMAND) cl_confirm(s_targets);
  }
  REQUIRE(s_phase == DPhase::MONITOR);
}

static size_t count_cmd(uint8_t cmd) {
  size_t n = 0;
  for (size_t k = 0; k < cs_sent_count(); ++k)
    if (ODrive::cmd_of(cs_sent_at(k).id) == cmd) ++n;
  return n;
}
static size_t count_set_state() { return count_cmd(ODriveCmd::set_requested_state); }

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
  cl_confirm(0x01);                // FW 53 U8: leg 0's post-CL heartbeat
  cs_clear_sent();
  deactivate_step();               // COMMAND

  REQUIRE(cs_sent_count() == 1);
  const auto tgt = ODrive::encode_leg_setpoint(0, STOW_OFF_POSE_REV, 0.0f, 0.0f);
  CHECK(cs_sent_at(0).id == tgt.id);
  CHECK(memcmp(cs_sent_at(0).buf, tgt.buf, 8) == 0);
}

// ── MONITOR completion IDLEs the leg (the difference #2 — safes the robot) ─────

TEST_CASE("deactivate IDLEs the leg on arrival at STOW (de-energised end-state)") {
  full_reset(0x01, ST_CL, POSED);
  REQUIRE(deactivate_request(0) == JbUdp::RpcStatus::OK);
  deactivate_step();               // SETUP
  deactivate_step();               // → COMMAND
  cs_advance(20000);
  cl_confirm(0x01);                // FW 53 U8: leg 0's post-CL heartbeat
  deactivate_step();               // COMMAND → MONITOR

  // The leg reaches STOW and settles; its heartbeat (after the ladder's CL + 10 ms)
  // reports CLOSED_LOOP, so the Stow gate passes (FW 53).
  confirmed(0, ST_CL, STOW_OFF_POSE_REV);
  cs_clear_sent();
  deactivate_step();               // MONITOR sees arrival → IDLE + OK

  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(0) == DEACTIVATE_OK);
  REQUIRE(cs_sent_count() == 1);
  const auto idle = ODrive::encode_set_state(0, ODriveState::IDLE);
  CHECK(cs_sent_at(0).id == idle.id);
  CHECK(memcmp(cs_sent_at(0).buf, idle.buf, 8) == 0);
  CHECK(leg_held_mask() == 0);
}

// ── FW 53 safety crux: no abort IDLEs a Leg above Stow (fw53_design.md 3.1) ────

TEST_CASE("CAN loss: FAILED with no frame, before the descent (BUS_OFF / bus down) and mid-descent") {
  // SETUP, commands-not-allowed (BUS_OFF), posed: the gate refuses the IDLE. FW 53
  // U3: leg 0 (the only present leg) is configured but unconfirmed, so the abort
  // first waits (bounded, nothing sent); its confirming heartbeat then reads it
  // posed, and the refusal ends the ladder with no frame.
  full_reset(0x01, ST_CL, POSED);
  REQUIRE(deactivate_request(0) == JbUdp::RpcStatus::OK);
  deactivate_step();
  REQUIRE(deactivate_active());
  cs_set_jugglebot_health(JbUdp::BusHealth::BUS_OFF);
  cs_clear_sent();
  deactivate_step();
  CHECK(deactivate_active());
  cs_advance(20000);
  confirmed(0, ST_CL, POSED);
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(0) == DEACTIVATE_FAILED);
  CHECK(cs_sent_count() == 0);

  // SETUP, the fault machine's bus-down, from an already-stowed Platform: still no
  // frame (nothing is ever sent to a dead bus, even an IDLE the gate would allow).
  full_reset(0x03, ST_IDLE, -0.1f);
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  deactivate_step();
  cs_set_can_bus_down(true);
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(0) == DEACTIVATE_FAILED);
  CHECK(deactivate_result(1) == DEACTIVATE_FAILED);
  CHECK(cs_sent_count() == 0);

  // MONITOR, mid-descent: no frame, FAILED (the deferred stow takes over).
  full_reset(0x3F, ST_CL, POSED);
  run_to_monitor(JbUdp::RpcArgs::AXIS_ALL);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) confirmed(i, ST_CL, 1.1f, -2.5f);
  cs_set_can_bus_down(true);
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_FAILED);
  CHECK(cs_sent_count() == 0);
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
  cl_confirm(0x01);       // FW 53 U8: leg 0's post-CL heartbeat
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
  deactivate_step();                           // abort path (FW 53 U3: waits for leg 0's CL confirmation)
  cs_advance(20000);
  confirmed(0, ST_CL, 0.0f);                   // confirmed at Stow → the gated IDLE
  deactivate_step();

  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(0) == DEACTIVATE_FAILED);
  CHECK(deactivate_result(HAND_AXIS) == DEACTIVATE_OK);   // de-energised before the abort
}

// FW 53 U3: the pre-descent abort waits (bounded by CL_CONFIRM_WAIT_US) while the
// only refusals are unconfirmed CLOSED_LOOP sends, the same rule as ACTIVATE's
// pre-rise abort. Mutations caught: no wait (holds at once, HELD, no IDLE); an
// unbounded wait (never ends).
TEST_CASE("pre-descent abort inside the CL-confirmation window: IDLE once confirmed; bounded") {
  full_reset(0x3F, ST_IDLE, -0.1f);                // after Homing: six IDLE on the stop
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  deactivate_step(); cs_advance(10000);            // leg 0 configured
  cs_set_guard_estop(true);
  cs_clear_sent();
  deactivate_step();
  CHECK(deactivate_active());
  CHECK(cs_sent_count() == 0);
  cs_advance(20000);
  confirmed(0, ST_CL, -0.1f);
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(count_set_state() == 6);
  CHECK(leg_held_mask() == 0);

  full_reset(0x3F, ST_IDLE, -0.1f);
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  deactivate_step(); cs_advance(10000);
  cs_set_guard_estop(true);
  deactivate_step();
  const uint64_t t_abort = micros64();
  cs_set_mono(t_abort + CL_CONFIRM_WAIT_US - 10000);
  cs_clear_sent();
  deactivate_step();
  CHECK(deactivate_active());
  cs_set_mono(t_abort + CL_CONFIRM_WAIT_US + 10000);
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(cs_sent_count() == 0);
  CHECK(leg_held_mask() == 0x3F);
}

// ── FW 53 (I-FW-19): the abort table of fw53_design.md 3.1 ────────────────────
//  Robot case unless stated: six legs CLOSED_LOOP at the Active pose (POSED).

TEST_CASE("arrival implies the gate: the ladder completes iff every target is in the arrival band") {
  // Grid across the arrival band's edges (TARGET_REACHED_*), both signs. A point the
  // ladder's arrival test accepts must also pass the Stow gate in the same tick, or
  // a normal stow would sit in MONITOR to the 10 s timeout and end FAILED/HELD.
  const float tp = JBOp::TARGET_REACHED_POS_TOL_REV, tv = JBOp::TARGET_REACHED_VEL_TOL_RPS;
  const float ps[] = { -1.5f * tp, -1.001f * tp, -0.999f * tp, -0.5f * tp, 0.0f,
                       0.5f * tp, 0.999f * tp, 1.001f * tp, 1.5f * tp };
  const float vs[] = { -1.5f * tv, -1.001f * tv, -0.999f * tv, 0.0f,
                       0.999f * tv, 1.001f * tv, 1.5f * tv };
  for (float p : ps) {
    for (float v : vs) {
      full_reset(0x03, ST_CL, POSED);
      run_to_monitor(JbUdp::RpcArgs::AXIS_ALL);
      confirmed(0, ST_CL, p, v);
      confirmed(1, ST_CL, 0.0f, 0.0f);
      cs_clear_sent();
      deactivate_step();
      const bool in_band = std::fabs(p - STOW_OFF_POSE_REV) <= tp && std::fabs(v) <= tv;
      INFO("pos=" << p << " vel=" << v);
      CHECK(deactivate_active() == !in_band);
      CHECK(cs_sent_count() == (in_band ? 2u : 0u));
      CHECK(count_set_state() == cs_sent_count());
      CHECK(leg_deenergise_refusals() == 0u);   // the pre-check keeps a non-arrival off the census
    }
  }
}

TEST_CASE("guard ESTOP after COMMAND: no frame until arrival, then IDLE") {
  full_reset(0x3F, ST_CL, POSED);
  run_to_monitor(JbUdp::RpcArgs::AXIS_ALL);
  cs_set_guard_estop(true);                        // e.g. a Hand fault mid-descent
  cs_set_commands_allowed(false);                  // and a transient commands-not-allowed
  for (int k = 0; k < 30; ++k) {                   // 300 ms of descent
    for (uint8_t i = 0; i < NUM_LEGS; ++i) confirmed(i, ST_CL, 1.1f - 0.025f * k, -2.5f);
    cs_clear_sent();
    deactivate_step();
    REQUIRE(deactivate_active());
    REQUIRE(cs_sent_count() == 0);
    cs_advance(10000);
  }
  for (uint8_t i = 0; i < NUM_LEGS; ++i) confirmed(i, ST_CL, 0.0f, 0.0f);
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(cs_sent_count() == 6);
  CHECK(count_set_state() == 6);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_OK);
  CHECK(leg_held_mask() == 0);
}

TEST_CASE("ESTOP in SETUP (tick 3, posed): FAILED, no IDLE frame, every target HELD") {
  full_reset((uint8_t)(0x3F | (1u << HAND_AXIS)), ST_CL, POSED);
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  deactivate_step();                               // hand IDLE + configure leg 0
  cs_advance(10000);
  deactivate_step();                               // configure leg 1
  cs_advance(10000);
  cs_set_guard_estop(true);
  cs_clear_sent();
  deactivate_step();                               // tick 3: the abort
  CHECK_FALSE(deactivate_active());
  CHECK(cs_sent_count() == 0);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_FAILED);
  CHECK(deactivate_result(HAND_AXIS) == DEACTIVATE_OK);
  CHECK(leg_held_mask() == 0x3F);
  CHECK(leg_deenergise_refusals() == 6u);

  // The next accepted DEACTIVATE clears its targets' HELD bits.
  cs_set_guard_estop(false);
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  CHECK(leg_held_mask() == 0);
}

TEST_CASE("errored target / send failure in SETUP: no IDLE (posed)") {
  full_reset(0x3F, ST_CL, POSED);
  axes[2].active_errors = 0x1;
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  cs_clear_sent();
  for (int k = 0; k < 3; ++k) deactivate_step();   // legs 0, 1 configured; leg 2 errored
  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(2) == DEACTIVATE_FAILED);
  CHECK(count_set_state() == 2);                   // only the two CLOSED_LOOP sends
  CHECK(leg_held_mask() == 0x3F);

  full_reset(0x3F, ST_CL, POSED);
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  cs_set_send_fail_index(4);                       // leg 0's CLOSED_LOOP frame fails
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(0) == DEACTIVATE_FAILED);
  CHECK(cs_sent_count() == 4);                     // the four recorded preamble frames, no IDLE
  CHECK(count_set_state() == 0);
}

TEST_CASE("MONITOR timeout mid-descent: freeze frames within [0, seed], no IDLE") {
  full_reset(0x03, ST_CL, POSED);
  run_to_monitor(JbUdp::RpcArgs::AXIS_ALL);
  CHECK(s_seed[0] == doctest::Approx(POSED));
  cs_set_mono(s_t_start_us + D_TIMEOUT_US + 1);
  confirmed(0, ST_CL, 0.8f, 0.0f);                 // leg 0 blocked at 0.8 rev
  confirmed(1, ST_CL, 0.02f, -2.0f);               // leg 1 still moving, 2 cm above the bottom
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(count_set_state() == 0);
  REQUIRE(cs_sent_count() == 2);
  for (uint8_t i = 0; i < 2; ++i) {
    float pos = 0.0f, vel = 0.0f; uint64_t ts = 0;
    snapshot_pos_vel(axes[i], pos, vel, ts);
    const float t = leg_freeze_target(pos, vel, POSED, STOW_OFF_POSE_REV);
    CHECK(t >= 0.0f);
    CHECK(t <= POSED);
    const auto f = ODrive::encode_set_input_pos(i, ODrive::leg_sign(i, t), 0, 0);
    CHECK(cs_sent_at(i).id == f.id);
    CHECK(memcmp(cs_sent_at(i).buf, f.buf, 8) == 0);
    CHECK(deactivate_result(i) == DEACTIVATE_FAILED);
  }
  CHECK(leg_freeze_target(0.02f, -2.0f, POSED, 0.0f) == 0.0f);   // the clamp at Stow bites
  CHECK(leg_held_mask() == 0x03);
}

// FW 53 U1b (owner 2026-11-25): on the one-Leg bench (rig_is_bench) the Stow gate is
// off, so a MONITOR timeout far above Stow IDLEs the Leg as FW 52 did, instead of the
// robot's freeze. Mutation caught: leg_deactivate.cpp's timeout branch reverted to
// platform_at_stow(now).ok (a freeze frame, no IDLE).
TEST_CASE("MONITOR timeout on the one-Leg bench IDLEs the Leg above Stow (FAILED, not held)") {
  full_reset(0x01, ST_CL, POSED);
  rig_note_axis_frame(0, micros64());      // the bench Leg's first frame ...
  rig_note_bus(true, micros64());          // ... on a transmittable bus, both at the start
  run_to_monitor(0);
  cs_set_mono(s_t_start_us + D_TIMEOUT_US + 1);   // 10 s > RIG_BENCH_SETTLE_US
  confirmed(0, ST_CL, 0.8f, 0.0f);                 // blocked at 0.8 rev
  REQUIRE(rig_is_bench(micros64()));
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(count_set_state() == 1);
  CHECK(cs_sent_count() == 1);
  CHECK(deactivate_result(0) == DEACTIVATE_FAILED);
  CHECK(leg_held_mask() == 0);
}

TEST_CASE("MONITOR timeout with the Platform at Stow IDLEs the legs (FAILED)") {
  full_reset(0x03, ST_CL, POSED);
  run_to_monitor(JbUdp::RpcArgs::AXIS_ALL);
  cs_set_mono(s_t_start_us + D_TIMEOUT_US + 1);
  confirmed(0, ST_CL, -0.05f, 0.0f);               // in the Stow window, outside the arrival band
  confirmed(1, ST_CL, 0.0f, 0.0f);
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(count_set_state() == 2);
  CHECK(cs_sent_count() == 2);
  CHECK(deactivate_result(0) == DEACTIVATE_FAILED);
}

TEST_CASE("a target Leg reported IDLE mid-descent freezes the others") {
  full_reset(0x3F, ST_CL, POSED);
  run_to_monitor(JbUdp::RpcArgs::AXIS_ALL);
  // A heartbeat received BEFORE the CL command + CL_ENGAGE_SETTLE_US saying IDLE is
  // a cached pre-CL heartbeat, not a dropped leg: nothing happens.
  // SETUP's CLOSED_LOOP went through the gate, which stamped each leg's CL time
  // (the stamp both this test and the Stow predicate's CL confirmation rely on).
  for (uint8_t i = 0; i < NUM_LEGS; ++i) REQUIRE(leg_cl_cmd_us(i) != 0u);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) confirmed(i, ST_CL, 1.4f, -2.5f);
  cs_stage_leg(2, (uint8_t)ST_IDLE, 1.4f, 0.0f,
               (int64_t)(micros64() - leg_cl_cmd_us(2)) - (int64_t)CL_ENGAGE_SETTLE_US + 1000, 0);
  cs_clear_sent();
  deactivate_step();
  REQUIRE(deactivate_active());
  CHECK(cs_sent_count() == 0);

  confirmed(2, ST_IDLE, 1.4f, 0.0f);               // leg 2 disarmed itself
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(count_set_state() == 0);
  CHECK(cs_sent_count() == 5);
  for (size_t k = 0; k < cs_sent_count(); ++k) {
    CHECK(ODrive::cmd_of(cs_sent_at(k).id) == ODriveCmd::set_input_pos);
    CHECK(ODrive::axis_of(cs_sent_at(k).id) != 2);
  }
  CHECK(leg_freeze_target(1.4f, -2.5f, POSED, 0.0f) == doctest::Approx(1.4f - 6.25f / 60.0f));
  for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_FAILED);
  CHECK(leg_held_mask() == 0x3B);                  // everyone but leg 2
}

// FW 53 audit follow-up (SHOULD-FIX 1): the descent continues through a Soft E-stop
// by design, so before the fix Leg 2's Get_Error (heartbeat still CLOSED_LOOP)
// changed nothing and the five kept descending at up to 2.5 rev/s with Leg 2 free
// until its next heartbeat. Now they freeze in that tick: no IDLE, no new target.
// Mutation caught: the active_errors term dropped from leg_dropped_out.
TEST_CASE("audit: a Leg's error before its heartbeat mid-descent freezes the others in the first tick") {
  full_reset(0x3F, ST_CL, POSED);
  run_to_monitor(JbUdp::RpcArgs::AXIS_ALL);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) confirmed(i, ST_CL, 1.4f, -2.5f);   // all CLOSED_LOOP
  axes[2].active_errors = 0x1000;
  cs_set_guard_estop(true);                        // the Soft E-stop the error raised
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(count_set_state() == 0);
  CHECK(cs_sent_count() == 5);
  for (size_t k = 0; k < cs_sent_count(); ++k) {
    CHECK(ODrive::cmd_of(cs_sent_at(k).id) == ODriveCmd::set_input_pos);
    CHECK(ODrive::axis_of(cs_sent_at(k).id) != 2);
    float f; std::memcpy(&f, cs_sent_at(k).buf, 4);
    const uint8_t ax = ODrive::axis_of(cs_sent_at(k).id);
    CHECK(f == doctest::Approx(ODrive::leg_sign(ax, leg_freeze_target(1.4f, -2.5f, POSED, 0.0f))));
  }
  for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_FAILED);
  CHECK(leg_held_mask() == 0x3B);
}

TEST_CASE("abort from an already-stowed Platform IDLEs as before") {
  full_reset(0x3F, ST_IDLE, -0.1f);                // after Homing: six IDLE on the stop
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  cs_set_guard_estop(true);                        // before the first SETUP tick
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(cs_sent_count() == 6);
  CHECK(count_set_state() == 6);
  for (size_t k = 0; k < cs_sent_count(); ++k) {
    const auto idle = ODrive::encode_set_state((uint8_t)k, ODriveState::IDLE);
    CHECK(memcmp(cs_sent_at(k).buf, idle.buf, 8) == 0);
  }
  for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_FAILED);
  CHECK(leg_held_mask() == 0);
}

TEST_CASE("single-Leg DEACTIVATE refused when another present Leg is posed; accepted on the one-Leg bench") {
  full_reset(0x3F, ST_CL, POSED);                  // robot, all posed
  cs_clear_sent();
  CHECK(deactivate_request(3) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK_FALSE(deactivate_active());
  CHECK(cs_sent_count() == 0);
  CHECK(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);   // the descent itself

  full_reset(0x3F, ST_IDLE, -0.1f);                // robot stowed, only leg 3 posed
  confirmed(3, ST_CL, POSED);
  CHECK(deactivate_request(3) == JbUdp::RpcStatus::OK);

  full_reset(0x01, ST_CL, 1.5f);                   // one-Leg bench
  CHECK(deactivate_request(0) == JbUdp::RpcStatus::OK);
  CHECK(deactivate_active());

  full_reset((uint8_t)(0x3F | (1u << HAND_AXIS)), ST_CL, POSED);
  CHECK(deactivate_request(HAND_AXIS) == JbUdp::RpcStatus::OK);   // the Hand is exempt
}

TEST_CASE("a partial refusal at arrival: the IDLE'd legs leave the arrival test, the rest retry") {
  full_reset(0x03, ST_CL, POSED);
  run_to_monitor(JbUdp::RpcArgs::AXIS_ALL);
  confirmed(0, ST_CL, 0.0f, 0.0f);
  confirmed(1, ST_CL, 0.0f, 0.0f);
  // Leg 1's IDLE frame fails on the wire (attempt index 1 of this tick).
  cs_set_send_fail_index(1);
  cs_clear_sent();
  deactivate_step();
  REQUIRE(deactivate_active());
  CHECK(deactivate_result(0) == DEACTIVATE_OK);
  CHECK(deactivate_result(1) == DEACTIVATE_RUNNING);
  // Leg 0 is IDLE now and its position relaxes into the foam: it must not block.
  confirmed(0, ST_IDLE, -0.08f, 0.0f);
  cs_advance(10000);
  confirmed(1, ST_CL, 0.0f, 0.0f);
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(1) == DEACTIVATE_OK);
  REQUIRE(cs_sent_count() == 1);
  CHECK(ODrive::axis_of(cs_sent_at(0).id) == 1);
}

TEST_CASE("arrival with a stale heartbeat retries next tick without a refusal or a HELD bit") {
  full_reset(0x03, ST_CL, POSED);
  run_to_monitor(JbUdp::RpcArgs::AXIS_ALL);
  confirmed(0, ST_CL, 0.0f, 0.0f);
  cs_stage_leg(1, (uint8_t)ST_CL, 0.0f, 0.0f, 600000, 0);   // heartbeat 0.6 s old, encoder fresh
  cs_clear_sent();
  deactivate_step();
  REQUIRE(deactivate_active());
  CHECK(cs_sent_count() == 0);
  CHECK(leg_deenergise_refusals() == 0u);
  CHECK(leg_held_mask() == 0);
  cs_advance(10000);
  confirmed(0, ST_CL, 0.0f, 0.0f);
  confirmed(1, ST_CL, 0.0f, 0.0f);                 // the heartbeat is back
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(deactivate_result(0) == DEACTIVATE_OK);
  CHECK(deactivate_result(1) == DEACTIVATE_OK);
}

// ── FW 53 U6b: the mixed-case recovery R3 (fw53_design.md 6) ──────────────────
// The host's recovery after one Leg dropped out by itself (its drive disarmed on an
// error) while five held the Platform up: the operator's clear_errors cleared the
// drive errors and the guard latch, then DEACTIVATE(AXIS_ALL). Staged exactly: five
// Legs Closed-loop at the Active pose, Leg 2 IDLE (errors clear) at a sagged length.
// Pins: SETUP re-seeds Leg 2 at its MEASURED length and re-engages it (input_pos =
// measured, so the engage commands no motion), no Leg is ever IDLE'd above Stow,
// all six descend together, and each failure branch holds instead of dropping.
// Mutation caught: SETUP's seed changed to a fixed target (e.g. POSED) or skipped
// for an IDLE Leg (subcase "re-engages"); the DROPPED branch removed from
// stow_descent_step (subcase "re-faults"); the SETUP error gate removed (subcase
// "error not cleared").
TEST_CASE("R3 mixed case: one Leg IDLE at a sagged length, five posed; DEACTIVATE(AXIS_ALL)") {
  const float SAGGED = 1.90f;                      // Leg 2 sagged 0.29 rev while free
  full_reset(0x3F, ST_CL, POSED);
  cs_stage_leg(2, (uint8_t)ST_IDLE, SAGGED, 0.0f, FRESH_HB, FRESH_ENC);   // errors 0 (cleared)

  SUBCASE("error not cleared: SETUP aborts at Leg 2, nothing IDLE'd, every Leg HELD") {
    axes[2].active_errors = 0x1000;                // CURRENT_LIMIT_VIOLATION still active
    REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
    cs_clear_sent();
    for (int k = 0; k < 3; ++k) deactivate_step();
    CHECK_FALSE(deactivate_active());
    CHECK(deactivate_result(2) == DEACTIVATE_FAILED);
    CHECK(count_set_state() == 2);                 // Legs 0, 1 CLOSED_LOOP only, no IDLE
    CHECK(count_cmd(ODriveCmd::set_input_pos) == 2);   // their measured seeds, no move
    CHECK(leg_held_mask() == 0x3F);
  }

  SUBCASE("errors cleared") {
  REQUIRE(deactivate_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::OK);
  cs_clear_sent();
  for (int k = 0; k < 3; ++k) { deactivate_step(); cs_advance(10000); }   // SETUP Legs 0, 1, 2
  // Leg 2's five-frame preamble: seed = its measured (sagged) length, then CLOSED_LOOP.
  REQUIRE(cs_sent_count() == 15);
  const auto seed2 = ODrive::encode_set_input_pos(2, ODrive::leg_sign(2, SAGGED), 0, 0);
  CHECK(cs_sent_at(10).id == seed2.id);
  CHECK(memcmp(cs_sent_at(10).buf, seed2.buf, 8) == 0);
  const auto cl2 = ODrive::encode_set_state(2, ODriveState::CLOSED_LOOP);
  CHECK(cs_sent_at(14).id == cl2.id);
  CHECK(memcmp(cs_sent_at(14).buf, cl2.buf, 8) == 0);
  CHECK(s_seed[2] == doctest::Approx(SAGGED));
  CHECK(s_seed[0] == doctest::Approx(POSED));
  for (int k = 0; k < 20 && s_phase != DPhase::COMMAND; ++k) { deactivate_step(); cs_advance(10000); }
  REQUIRE(s_phase == DPhase::COMMAND);
  cs_advance(10000);

  SUBCASE("re-engages: the six descend together and IDLE only on arrival at Stow") {
    cl_confirm(0x3F);                              // all six confirm Closed-loop (FW 53 U8)
    for (int k = 0; k < 5 && s_phase != DPhase::MONITOR; ++k) { deactivate_step(); cs_advance(10000); }
    REQUIRE(s_phase == DPhase::MONITOR);
    // Nothing but the six preambles and the six STOW targets: no IDLE, no other target.
    CHECK(count_set_state() == 6);                 // six CLOSED_LOOP
    CHECK(count_cmd(ODriveCmd::set_input_pos) == 12);
    CHECK(leg_held_mask() == 0);
    for (uint8_t i = 0; i < NUM_LEGS; ++i) confirmed(i, ST_CL, 1.0f, -2.5f);
    cs_clear_sent();
    deactivate_step();
    REQUIRE(deactivate_active());
    CHECK(cs_sent_count() == 0);
    for (uint8_t i = 0; i < NUM_LEGS; ++i) confirmed(i, ST_CL, STOW_OFF_POSE_REV);
    cs_clear_sent();
    deactivate_step();
    CHECK_FALSE(deactivate_active());
    CHECK(count_set_state() == 6);                 // six IDLE, at Stow
    for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_OK);
    CHECK(leg_held_mask() == 0);
  }

  // FW 53 U8 (owner: no Leg is lowered while another is free). Before U8 the five
  // started descending and froze only on Leg 2's first post-CL heartbeat; now
  // COMMAND waits for every Leg's confirmation, Leg 2 reports IDLE with its error
  // (the re-fault), and the pre-descent abort holds: no Stow target and no freeze
  // frame to any Leg.
  SUBCASE("re-faults on engage: no Leg descends, all held, Leg 2 is not IDLE'd by the ladder") {
    cl_confirm(0x3B);                              // the five confirm Closed-loop at the Active pose
    confirmed(2, ST_IDLE, SAGGED);                 // its post-CL heartbeat: still IDLE ...
    axes[2].active_errors = 0x1000;                // ... and Get_Error reports the re-fault
    deactivate_step();
    CHECK_FALSE(deactivate_active());
    CHECK(count_set_state() == 6);                 // the six CLOSED_LOOP of SETUP, no IDLE
    CHECK(count_cmd(ODriveCmd::set_input_pos) == 6);   // the six measured seeds only: no motion
    for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_FAILED);
    CHECK(leg_held_mask() == 0x3F);                // the refused IDLEs (as "error not cleared")
  }
  }  // SUBCASE errors cleared
}

// ── FW 53 U8: no Stow target to any Leg until every target confirms Closed-loop ─
// (owner: no Leg is lowered while another is free). COMMAND waits for a heartbeat
// from each target Leg received >= its CLOSED_LOOP command + CL_ENGAGE_SETTLE_US
// reporting CLOSED_LOOP with no error, bounded by CL_CONFIRM_WAIT_US from the last
// CLOSED_LOOP command; a failure or the expiry is the pre-descent abort.
static size_t count_stow_targets() {
  size_t n = 0;
  for (size_t k = 0; k < cs_sent_count(); ++k) {
    const auto& f = cs_sent_at(k);
    const uint8_t a = ODrive::axis_of(f.id);
    if (a >= NUM_LEGS) continue;
    const auto stow = ODrive::encode_leg_setpoint(a, STOW_OFF_POSE_REV, 0.0f, 0.0f);
    if (f.id == stow.id && memcmp(f.buf, stow.buf, 8) == 0) ++n;
  }
  return n;
}

// Mutations caught: the confirmation removed (COMMAND after the settle alone); the
// post-CL stamp comparison dropped (the cached pre-CL CLOSED_LOOP of a posed Leg
// confirms it).
TEST_CASE("U8: staggered confirmations; no Stow target to any Leg until the last target confirms") {
  full_reset(0x3F, ST_CL, POSED);                  // posed: every cached heartbeat reads CLOSED_LOOP
  (void)run_to_command(JbUdp::RpcArgs::AXIS_ALL);
  cs_clear_sent();
  deactivate_step();                               // settle met; only pre-CL heartbeats: wait
  CHECK(s_phase == DPhase::COMMAND);
  cs_advance(10000);
  cl_confirm(0x07);                                // Legs 0, 1, 2
  deactivate_step();
  cs_advance(10000);
  cl_confirm(0x18);                                // Legs 3, 4
  deactivate_step();
  CHECK(s_phase == DPhase::COMMAND);
  CHECK(cs_sent_count() == 0);                     // nothing at all in three ticks
  cs_advance(10000);
  cl_confirm(0x20);                                // Leg 5, the last
  deactivate_step();
  CHECK(s_phase == DPhase::MONITOR);
  CHECK(count_stow_targets() == 6);                // all six in the same tick
  CHECK(cs_sent_count() == 6);
  CHECK(leg_held_mask() == 0);
}

// From a stowed Platform (IDLE on the stop, e.g. DEACTIVATE after Homing) every
// cached heartbeat reads IDLE. One received just before its Leg's CL command +
// CL_ENGAGE_SETTLE_US is neither a confirmation nor a failed engage. Mutation
// caught: the post-CL stamp comparison dropped (the cached IDLE fails the ladder).
TEST_CASE("U8: a pre-CL cached IDLE heartbeat neither confirms nor fails") {
  full_reset(0x03, ST_IDLE, -0.1f);
  const uint64_t t_last_cl = run_to_command(JbUdp::RpcArgs::AXIS_ALL);   // Leg 1's SETUP tick
  // Leg 1's heartbeat lands 1 ms before its CL command + settle (staged from the
  // test's own clock of that SETUP tick, not from the gate's stamp).
  cs_stage_leg(1, (uint8_t)ST_IDLE, -0.1f, 0.0f,
               (int64_t)(micros64() - t_last_cl) - (int64_t)CL_ENGAGE_SETTLE_US + 1000, 0);
  cl_confirm(0x01);
  cs_clear_sent();
  deactivate_step();
  CHECK(deactivate_active());
  CHECK(s_phase == DPhase::COMMAND);
  CHECK(cs_sent_count() == 0);
  cs_advance(10000);
  cl_confirm(0x02);
  deactivate_step();
  CHECK(s_phase == DPhase::MONITOR);
  CHECK(count_stow_targets() == 2);
}

// The re-fault of the mixed-case recovery, generalised: a target Leg with an error
// (active_errors, or a cached disarm_reason) ends the ladder FAILED at once with no
// motion frame to any Leg; posed, the gate refuses every IDLE, so all hold (HELD).
// Mutations caught: the active_errors check removed; the disarm_reason check removed.
TEST_CASE("U8: a Leg with an error after its CL command: no motion frame to any Leg, all held") {
  full_reset(0x3F, ST_CL, POSED);
  (void)run_to_command(JbUdp::RpcArgs::AXIS_ALL);
  cs_advance(10000);
  SUBCASE("IDLE with a cached disarm_reason") {
    cl_confirm(0x37);
    confirmed(3, ST_IDLE, POSED);
    axes[3].disarm_reason = 0x1;
  }
  SUBCASE("CLOSED_LOOP with an active error") {
    cl_confirm(0x3F);
    axes[3].active_errors = 0x1;
  }
  cs_clear_sent();
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(cs_sent_count() == 0);                     // no Stow target, no freeze, no IDLE
  for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_FAILED);
  CHECK(leg_held_mask() == 0x3F);
}

// Mutations caught: no expiry (waits forever); the abort given a fresh 250 ms from
// `now` instead of the confirmation window (the stowed variant would wait again).
TEST_CASE("U8: the CL_CONFIRM_WAIT_US expiry ends the ladder with no motion frame") {
  SUBCASE("posed: hold") {
    full_reset(0x3F, ST_CL, POSED);
  }
  SUBCASE("stowed: the unconfirmed Leg blocks the IDLE; the window is not restarted") {
    full_reset(0x3F, ST_IDLE, -0.1f);
  }
  const uint64_t t_last_cl = run_to_command(JbUdp::RpcArgs::AXIS_ALL);
  cl_confirm(0x2F);                                // every Leg but Leg 4
  cs_set_mono(t_last_cl + CL_CONFIRM_WAIT_US - 10000);
  cs_clear_sent();
  deactivate_step();
  CHECK(deactivate_active());
  CHECK(s_phase == DPhase::COMMAND);
  cs_set_mono(t_last_cl + CL_CONFIRM_WAIT_US + 10000);
  cl_confirm(0x2F);                                // the five stay live (fresh encoder, heartbeat)
  // Leg 4's last heartbeat: received before its CL command + settle (Leg 4's SETUP
  // tick was one tick before t_last_cl), still fresh at the expiry: CL_UNCONFIRMED,
  // not STALE_HB, so a restarted abort window would WAIT.
  cs_stage_leg(4, (uint8_t)axes[4].axis_state, axes[4].pos_rev, 0.0f,
               (int64_t)(micros64() - (t_last_cl - 5000)), 0);
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(cs_sent_count() == 0);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_FAILED);
  CHECK(leg_held_mask() == 0x3F);
}

// One lost heartbeat costs one 100 ms period inside the 250 ms bound. Mutation
// caught: the bound cut below one lost heartbeat (<= 210 ms).
TEST_CASE("U8: one lost heartbeat still completes") {
  full_reset(0x3F, ST_CL, POSED);
  const uint64_t t_last_cl = run_to_command(JbUdp::RpcArgs::AXIS_ALL);
  cl_confirm(0x1F);                                // Leg 5's first post-CL heartbeat is lost
  cs_set_mono(t_last_cl + 110000);
  deactivate_step();
  CHECK(s_phase == DPhase::COMMAND);
  cs_set_mono(t_last_cl + 200000);                 // still waiting, inside the bound
  deactivate_step();
  CHECK(s_phase == DPhase::COMMAND);
  cs_set_mono(t_last_cl + 210000);                 // its next one, 100 ms later
  cl_confirm(0x3F);
  cs_clear_sent();
  deactivate_step();
  CHECK(s_phase == DPhase::MONITOR);
  CHECK(count_stow_targets() == 6);
}

// The one-Leg bench: the same wait, for its one Leg; a failed engage IDLEs it where
// it stands (the bench exemption), FAILED, not held. Mutation caught: the
// confirmation removed (the Stow target goes before the heartbeat).
TEST_CASE("U8: the one-Leg bench waits for its Leg; a failed engage IDLEs it (bench)") {
  full_reset(0x01, ST_CL, 1.5f);
  rig_note_axis_frame(0, micros64());
  rig_note_bus(true, micros64());
  cs_advance(RIG_BENCH_SETTLE_US);
  confirmed(0, ST_CL, 1.5f);                       // a fresh heartbeat before the request
  REQUIRE(rig_is_bench(micros64()));
  (void)run_to_command(0);
  cs_clear_sent();
  deactivate_step();
  CHECK(s_phase == DPhase::COMMAND);
  CHECK(cs_sent_count() == 0);
  cs_advance(10000);
  confirmed(0, ST_IDLE, 1.5f);                     // re-faulted on engage ...
  axes[0].active_errors = 0x1;                     // ... with its error
  deactivate_step();
  CHECK_FALSE(deactivate_active());
  CHECK(count_stow_targets() == 0);
  CHECK(count_set_state() == 1);                   // the gated IDLE (bench)
  CHECK(deactivate_result(0) == DEACTIVATE_FAILED);
  CHECK(leg_held_mask() == 0);
}

// U8 follow-up (orchestrator, 2026-11-26): the drive's engage latency is unmeasured,
// so a post-settle heartbeat that still reports IDLE with NO error is a Leg still
// engaging: wait. The 400 ms bound ends a Leg that never engages. Staged: Leg 5
// engages slowly (IDLE at +90 ms, the +189 ms heartbeat lost, CLOSED_LOOP at +288 ms,
// i.e. an engage latency of about 100 ms plus one lost heartbeat). Mutations
// caught: a non-CL no-error heartbeat fails at once (the pre-follow-up rule); a
// non-CL heartbeat counted as confirmed; the bound left at 250 ms.
TEST_CASE("U8: a slow engage (post-settle IDLE, no error) waits; CLOSED_LOOP completes, none expires") {
  full_reset(0x3F, ST_CL, POSED);
  const uint64_t t_last_cl = run_to_command(JbUdp::RpcArgs::AXIS_ALL);
  cl_confirm(0x1F);
  cs_set_mono(t_last_cl + 90000);
  confirmed(5, ST_IDLE, POSED);                    // engaging: IDLE, no error
  cs_clear_sent();
  deactivate_step();
  CHECK(s_phase == DPhase::COMMAND);
  cs_set_mono(t_last_cl + 260000);                 // past the old 250 ms bound
  cl_confirm(0x1F);
  deactivate_step();
  CHECK(s_phase == DPhase::COMMAND);
  CHECK(cs_sent_count() == 0);
  SUBCASE("engages: CLOSED_LOOP at +288 ms, the six descend together") {
    cs_set_mono(t_last_cl + 288000);
    cl_confirm(0x3F);
    deactivate_step();
    CHECK(s_phase == DPhase::MONITOR);
    CHECK(count_stow_targets() == 6);
    CHECK(cs_sent_count() == 6);
  }
  SUBCASE("never engages: IDLE with no error to the bound, the pre-descent abort, no motion") {
    cs_set_mono(t_last_cl + CL_CONFIRM_WAIT_US - 10000);
    cl_confirm(0x1F);
    confirmed(5, ST_IDLE, POSED);
    deactivate_step();
    CHECK(s_phase == DPhase::COMMAND);
    cs_set_mono(t_last_cl + CL_CONFIRM_WAIT_US + 10000);
    cl_confirm(0x1F);
    confirmed(5, ST_IDLE, POSED);
    deactivate_step();
    CHECK_FALSE(deactivate_active());
    CHECK(cs_sent_count() == 0);                   // posed: every IDLE refused, nothing moved
    for (uint8_t i = 0; i < NUM_LEGS; ++i) CHECK(deactivate_result(i) == DEACTIVATE_FAILED);
    CHECK(leg_held_mask() == 0x3F);
  }
}
