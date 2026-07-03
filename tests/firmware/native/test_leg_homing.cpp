// =============================================================================
//  test_leg_homing.cpp — compiled test of the REAL firmware homing ladder
// =============================================================================
//  Drives the actual compiled leg_homing.cpp against the cold-start fake HAL,
//  asserting the Phase-9b/Phase-5 HOME behaviours that no test compiled before
//  (coverage gap 1). Homing detects the hardstop by a CURRENT SPIKE (Iq EMA), not
//  a known position, so the native driver runs the REAL float32 EMA — strictly
//  stronger than the float64 Python transcription the xref used:
//
//    * homing_request VALIDATION: axis range (legs 0..5 + hand 6; rejects AXIS_ALL
//      / out-of-range), bus/E-STOP gate, concurrent ACTIVATE/DEACTIVATE rejection,
//      idempotency;
//    * SETUP emits the CLOSED_LOOP + VELOCITY/VEL_RAMP + vel/curr-limit preamble
//      byte-identical to the encoders;
//    * the HAND vs LEG per-axis dispatch (Homing::HAND_* vs LEG_*) — the actual
//      port-bug surface: a HOME(6) must use the hand params, distinct from HOME(0);
//    * the full DRIVE → MONITOR(EMA trips) → STOP → SET_REF ladder ends by defining
//      the hardstop via set_absolute_position (HomingResult OK);
//    * any abort (bus down / timeout) leaves the axis in IDLE — never left driving.
//
//  Sibling predicates (activate_active / deactivate_active) inline; the real
//  homing_active() comes from the #included module. SCOPE: decision logic only.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>
#include <cmath>

#include "udp_protocol.h"
#include "protocol_config.h"
#include "odrive_protocol.h"
#include "canbridge_config.h"
#include "hardware_config.h"
#include "axis_state.h"
#include "can_buses.h"
#include "coldstart_hal.h"

// ── Sibling cold-start predicates (the two HOME excludes) ─────────────────────
namespace CanBridge {
static bool g_sib_activate = false;
static bool g_sib_deactivate = false;
bool activate_active()   { return g_sib_activate; }
bool deactivate_active() { return g_sib_deactivate; }
}

#include "leg_homing.cpp"   // the unit under test (exposes Phase, s_phase, the
                            // homing_* per-axis accessors, SETTLE_* via #include)

using namespace CanBridge;

static void full_reset() {
  cs_reset();
  g_sib_activate = false;
  g_sib_deactivate = false;
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    axes[i].heartbeat_seen = true;   // present by default — homing now gates on presence (Flash-A item 3)
    axes[i].active_errors = 0;
    axes[i].iq_measured = 0.0f;
  }
  homing_init();
}

// ── request validation ────────────────────────────────────────────────────────

TEST_CASE("homing_request rejects AXIS_ALL and out-of-range axes (ERR_BAD_ARGS)") {
  full_reset();
  CHECK(homing_request(JbUdp::RpcArgs::AXIS_ALL) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK(homing_request(NUM_AXES) == JbUdp::RpcStatus::ERR_BAD_ARGS);       // 7
  CHECK(homing_request(NUM_AXES + 5) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK_FALSE(homing_active());
}

TEST_CASE("homing_request accepts a leg AND the hand axis (single-axis scope)") {
  full_reset();
  CHECK(homing_request(0) == JbUdp::RpcStatus::OK);            // a leg
  full_reset();
  CHECK(homing_request(HAND_AXIS) == JbUdp::RpcStatus::OK);    // the hand (axis 6)
  CHECK(homing_active());
}

TEST_CASE("homing_request refuses a dead bus / E-STOP and a concurrent ACTIVATE/DEACTIVATE") {
  full_reset();
  cs_set_jugglebot_health(JbUdp::BusHealth::BUS_OFF);
  CHECK(homing_request(0) == JbUdp::RpcStatus::ERR_BUS_DOWN);

  full_reset();
  cs_set_guard_estop(true);
  CHECK(homing_request(0) == JbUdp::RpcStatus::ERR_BUS_DOWN);

  full_reset();
  g_sib_activate = true;
  CHECK(homing_request(0) == JbUdp::RpcStatus::ERR_REJECTED);

  full_reset();
  g_sib_deactivate = true;
  CHECK(homing_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
}

TEST_CASE("homing_request is idempotent (re-request while active is rejected)") {
  full_reset();
  REQUIRE(homing_request(0) == JbUdp::RpcStatus::OK);
  CHECK(homing_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
}

TEST_CASE("homing_request rejects an ABSENT axis — leg AND hand (presence gate, item 3)") {
  // Homing an absent axis would stream ~30 s of velocity frames to a phantom node,
  // climbing the FlexCAN TEC toward bus-off. leg_present() covers legs + the hand
  // (heartbeat_seen), so the same gate protects axis 6.
  full_reset();
  axes[0].heartbeat_seen = false;                       // leg 0 not on the bus
  CHECK(homing_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK_FALSE(homing_active());

  full_reset();
  axes[HAND_AXIS].heartbeat_seen = false;               // the hand not on the bus
  CHECK(homing_request(HAND_AXIS) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK_FALSE(homing_active());

  // Present → latches OK (both a leg and the hand).
  full_reset();
  CHECK(homing_request(0) == JbUdp::RpcStatus::OK);
  full_reset();
  CHECK(homing_request(HAND_AXIS) == JbUdp::RpcStatus::OK);
}

TEST_CASE("homing_request rejects while the MPC stream is actively driving (interlock, item 1b)") {
  full_reset();
  cs_set_mpc_active(true);                               // guard ENABLED on the Jetson → MPC driving legs
  CHECK(homing_request(0) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK_FALSE(homing_active());
  // Clears once the MPC stops driving.
  cs_set_mpc_active(false);
  CHECK(homing_request(0) == JbUdp::RpcStatus::OK);
  CHECK(homing_active());
}

// ── SETUP preamble byte-parity ────────────────────────────────────────────────

TEST_CASE("homing SETUP emits the CLOSED_LOOP + VEL_RAMP + vel/curr-limit preamble") {
  full_reset();
  REQUIRE(homing_request(0) == JbUdp::RpcStatus::OK);
  cs_clear_sent();
  homing_step();                // consume + SETUP this tick

  REQUIRE(cs_sent_count() == 3);
  const auto f0 = ODrive::encode_set_state(0, ODriveState::CLOSED_LOOP);
  CHECK(cs_sent_at(0).id == f0.id);
  CHECK(memcmp(cs_sent_at(0).buf, f0.buf, 8) == 0);
  const auto f1 = ODrive::encode_set_controller_mode(
      0, ODriveControlMode::VELOCITY, ODriveInputMode::VEL_RAMP);
  CHECK(cs_sent_at(1).id == f1.id);
  CHECK(memcmp(cs_sent_at(1).buf, f1.buf, 8) == 0);
  // vel = |speed|*2, curr = detect-limit + headroom — computed via the module's
  // OWN per-axis accessors (reached through the #include), so this can't drift.
  const auto f2 = ODrive::encode_set_vel_curr_limits(
      0, fabsf(homing_speed_rps(0)) * 2.0f,
      homing_curr_limit_a(0) + homing_headroom_a(0));
  CHECK(cs_sent_at(2).id == f2.id);
  CHECK(memcmp(cs_sent_at(2).buf, f2.buf, 8) == 0);
}

TEST_CASE("HOME(hand) uses the HAND params, distinct from a leg (the port-bug surface)") {
  // The per-axis accessors must dispatch hand→HAND_*, leg→LEG_*. A firmware edit
  // that reused the leg constant for the hand (the classic port bug) is caught here.
  CHECK(homing_speed_rps(HAND_AXIS) != homing_speed_rps(0));
  CHECK(homing_curr_limit_a(HAND_AXIS) != homing_curr_limit_a(0));

  full_reset();
  REQUIRE(homing_request(HAND_AXIS) == JbUdp::RpcStatus::OK);
  cs_clear_sent();
  homing_step();                // SETUP for the hand
  REQUIRE(cs_sent_count() == 3);
  const auto hand_lim = ODrive::encode_set_vel_curr_limits(
      HAND_AXIS, fabsf(homing_speed_rps(HAND_AXIS)) * 2.0f,
      homing_curr_limit_a(HAND_AXIS) + homing_headroom_a(HAND_AXIS));
  CHECK(cs_sent_at(2).id == hand_lim.id);
  CHECK(memcmp(cs_sent_at(2).buf, hand_lim.buf, 8) == 0);
  // And it is NOT the leg-0 limit frame.
  const auto leg_lim = ODrive::encode_set_vel_curr_limits(
      0, fabsf(homing_speed_rps(0)) * 2.0f,
      homing_curr_limit_a(0) + homing_headroom_a(0));
  CHECK(memcmp(hand_lim.buf, leg_lim.buf, 8) != 0);
}

// ── full ladder: DRIVE → MONITOR(EMA trip) → STOP → SET_REF ───────────────────

TEST_CASE("a sustained current spike trips the EMA, stops, and sets the home reference") {
  full_reset();
  REQUIRE(homing_request(0) == JbUdp::RpcStatus::OK);
  homing_step();                       // SETUP → DRIVE
  cs_advance(SETTLE_DRIVE_US + 1);
  homing_step();                       // DRIVE (settle elapsed) → input_vel → MONITOR
  REQUIRE(s_phase == Phase::MONITOR);

  // Sustained stall current well above the detection limit → the real float32 EMA
  // converges over MONITOR ticks (a single spike would NOT trip — EMA smoothing).
  axes[0].iq_measured = homing_curr_limit_a(0) * 3.0f;
  int guard = 0;
  while (s_phase == Phase::MONITOR && guard++ < 1000) homing_step();
  REQUIRE(s_phase == Phase::STOP_SETTLE);          // EMA tripped

  cs_clear_sent();
  cs_advance(SETTLE_STOP_US + 1);
  homing_step();                       // STOP_SETTLE → SET_REF
  homing_step();                       // SET_REF: define the hardstop reference

  CHECK_FALSE(homing_active());
  CHECK(homing_result(0) == HOMING_OK);
  // The reference-set frame was emitted (encode_set_absolute_position).
  const auto ref = ODrive::encode_set_absolute_position(0, homing_abs_pos_rev(0));
  bool saw_ref = false;
  for (size_t i = 0; i < cs_sent_count(); ++i)
    if (cs_sent_at(i).id == ref.id && memcmp(cs_sent_at(i).buf, ref.buf, 8) == 0)
      saw_ref = true;
  CHECK(saw_ref);
}

TEST_CASE("a brief current spike does NOT false-trip the EMA (smoothing)") {
  full_reset();
  REQUIRE(homing_request(0) == JbUdp::RpcStatus::OK);
  homing_step();
  cs_advance(SETTLE_DRIVE_US + 1);
  homing_step();                       // → MONITOR
  REQUIRE(s_phase == Phase::MONITOR);

  // One big spike, then back to a low draw — must not cross the limit.
  axes[0].iq_measured = homing_curr_limit_a(0) * 3.0f;
  homing_step();
  axes[0].iq_measured = homing_curr_limit_a(0) * 0.1f;
  for (int k = 0; k < 20; ++k) homing_step();
  CHECK(s_phase == Phase::MONITOR);    // still driving — no false hardstop
}

// ── Safety crux: abort leaves the axis in IDLE ────────────────────────────────

TEST_CASE("a mid-homing CAN3 loss aborts and leaves the axis in IDLE") {
  full_reset();
  REQUIRE(homing_request(0) == JbUdp::RpcStatus::OK);
  homing_step();                       // SETUP → DRIVE
  REQUIRE(homing_active());

  cs_set_jugglebot_health(JbUdp::BusHealth::BUS_OFF);
  cs_clear_sent();
  homing_step();                       // abort

  CHECK_FALSE(homing_active());
  CHECK(homing_result(0) == HOMING_FAILED);
  REQUIRE(cs_sent_count() == 1);
  const auto idle = ODrive::encode_set_state(0, ODriveState::IDLE);
  CHECK(cs_sent_at(0).id == idle.id);
  CHECK(memcmp(cs_sent_at(0).buf, idle.buf, 8) == 0);
}
