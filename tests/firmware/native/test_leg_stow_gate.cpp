// =============================================================================
//  test_leg_stow_gate.cpp — native tests for leg_stow_gate.cpp (FW 53, I-FW-19)
// =============================================================================
//  The owner rule (2026-11-25): no Leg is made IDLE (nor rebooted, nor SDO-
//  written) unless the Platform is at Stow; the Hand is exempt. This driver
//  links the REAL leg_stow_gate.o against the cold-start fake HAL (recording
//  CAN3 TX, controllable clock) and stages the axis cache directly with
//  cs_stage_leg(). homing_axis_in_flight() is supplied here, controllable,
//  because leg_homing.cpp (its production definition) is not linked.
//
//  Each case names the mutation of leg_stow_gate.cpp it must catch; every one
//  was run once against a mutated copy and failed (fw53-u1 logbook entry).
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>
#include <cmath>

#include "protocol_config.h"
#include "hardware_config.h"
#include "canbridge_config.h"
#include "odrive_protocol.h"
#include "axis_state.h"
#include "can_buses.h"
#include "time_base.h"
#include "leg_homing.h"
#include "leg_stow_gate.h"
#include "coldstart_hal.h"

namespace CanBridge {
static uint8_t g_homing_axis = 0xFF;
uint8_t homing_axis_in_flight() { return g_homing_axis; }
}  // namespace CanBridge

using namespace CanBridge;

static constexpr uint64_t T0 = 100000000ull;   // 100 s: every staged age fits below it
static constexpr int64_t  FRESH_HB  = 50000;   // 50 ms
static constexpr int64_t  FRESH_ENC = 2000;    // 2 ms
static constexpr uint32_t IDLE = ODriveState::IDLE;
static constexpr uint32_t CL   = ODriveState::CLOSED_LOOP;

static void reset_all() {
  cs_reset();
  cs_clear_axes();
  leg_stow_gate_init();
  g_homing_axis = 0xFF;
  cs_set_mono(T0);
}

// Present Legs 0..5 all IDLE and fresh.
static void stage_all_idle() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i) cs_stage_leg(i, IDLE, -0.1f, 0.0f, FRESH_HB, FRESH_ENC);
}

static uint8_t sent_cmd(size_t k)  { return ODrive::cmd_of(cs_sent_at(k).id); }
static uint8_t sent_axis(size_t k) { return ODrive::axis_of(cs_sent_at(k).id); }
static float   sent_f32(size_t k)  { float f; std::memcpy(&f, cs_sent_at(k).buf, 4); return f; }
static uint32_t sent_u32(size_t k) { uint32_t u; std::memcpy(&u, cs_sent_at(k).buf, 4); return u; }

static bool at_stow() { return platform_at_stow(micros64()).ok; }

// ── 1 ── mutation: dropping clause (a) ─────────────────────────────────────────
TEST_CASE("1: all present Legs IDLE and fresh -> a Leg IDLE is sent") {
  reset_all();
  stage_all_idle();
  REQUIRE(at_stow());
  REQUIRE(odrive_set_state_tx(2, IDLE, TxCls::RPC, Who::RPC) == DeResult::SENT);
  REQUIRE(cs_sent_count() == 1);
  CHECK(sent_axis(0) == 2);
  CHECK(sent_cmd(0) == ODriveCmd::set_requested_state);
  CHECK(sent_u32(0) == IDLE);
  CHECK(leg_deenergise_refusals() == 0);
}

// ── 2 ── mutation: evaluating only the target Leg ──────────────────────────────
TEST_CASE("2: one Leg CL at 2.2 rev refuses IDLE, REBOOT and SDO_WRITE on EVERY Leg; nothing on CAN") {
  reset_all();
  stage_all_idle();
  cs_stage_leg(3, CL, 2.2f, 0.0f, FRESH_HB, FRESH_ENC);
  const StowVerdict v = platform_at_stow(micros64());
  CHECK_FALSE(v.ok);
  CHECK(v.blocking_leg == 3);
  CHECK(v.reason == StowReason::NOT_AT_STOW);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    CHECK(odrive_set_state_tx(i, IDLE, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
    CHECK(odrive_set_state_tx(i, ODriveState::ENCODER_INDEX_SEARCH, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
    CHECK(odrive_reboot_tx(i, TxCls::SAFETY, Who::RPC) == DeResult::REFUSED);
    CHECK(odrive_sdo_write_tx(i, 0x1234, 1.0f, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
  }
  CHECK(cs_sent_count() == 0);
  CHECK(leg_deenergise_refusals() == 4u * NUM_LEGS);
}

// ── 3 ── mutations: two-sided |pos| <= tol; no lower bound; no velocity term ───
TEST_CASE("3: the Stow window edges, position and velocity") {
  struct Row { float pos, vel; bool ok; uint8_t reason; };
  const Row rows[] = {
    { -0.1000f,  0.0f,   true,  StowReason::OK },            // homed on the bottom stop
    {  0.0000f,  0.0f,   true,  StowReason::OK },            // the off pose
    { -0.1099f,  0.0f,   true,  StowReason::OK },
    { -0.1101f,  0.0f,   false, StowReason::NOT_AT_STOW },
    {  0.0099f,  0.0f,   true,  StowReason::OK },
    {  0.0101f,  0.0f,   false, StowReason::NOT_AT_STOW },
    {  0.0000f,  0.099f, true,  StowReason::OK },
    {  0.0000f,  0.101f, false, StowReason::MOVING },
    {  0.0000f, -0.099f, true,  StowReason::OK },
    {  0.0000f, -0.101f, false, StowReason::MOVING },
    {  NAN,      0.0f,   false, StowReason::NOT_AT_STOW },
    {  0.0000f,  NAN,    false, StowReason::MOVING },
  };
  for (const Row& r : rows) {
    reset_all();
    stage_all_idle();
    cs_stage_leg(1, CL, r.pos, r.vel, FRESH_HB, FRESH_ENC);
    const StowVerdict v = platform_at_stow(micros64());
    INFO("pos=" << r.pos << " vel=" << r.vel);
    CHECK(v.ok == r.ok);
    CHECK(v.reason == r.reason);
  }
}

// ── 4 ── mutations: CAN_AXIS_SILENCE_TIMEOUT_US (2 s), or no freshness ─────────
TEST_CASE("4: heartbeat age 0.49 s passes, 0.51 s and a zero stamp refuse STALE_HB") {
  reset_all();
  stage_all_idle();
  cs_stage_leg(4, IDLE, -0.1f, 0.0f, 490000, FRESH_ENC);
  CHECK(at_stow());
  cs_stage_leg(4, IDLE, -0.1f, 0.0f, 510000, FRESH_ENC);
  StowVerdict v = platform_at_stow(micros64());
  CHECK_FALSE(v.ok);
  CHECK(v.blocking_leg == 4);
  CHECK(v.reason == StowReason::STALE_HB);
  cs_stage_leg(4, IDLE, -0.1f, 0.0f, CS_NO_STAMP, FRESH_ENC);
  v = platform_at_stow(micros64());
  CHECK_FALSE(v.ok);
  CHECK(v.reason == StowReason::STALE_HB);
}

// ── 5 ── mutation: ignoring the encoder stamp ──────────────────────────────────
TEST_CASE("5: a CL Leg's encoder age 149 ms passes, 151 ms and a zero stamp refuse STALE_ENC") {
  reset_all();
  stage_all_idle();
  cs_stage_leg(0, CL, 0.0f, 0.0f, FRESH_HB, 149000);
  CHECK(at_stow());
  cs_stage_leg(0, CL, 0.0f, 0.0f, FRESH_HB, 151000);
  CHECK(platform_at_stow(micros64()).reason == StowReason::STALE_ENC);
  cs_stage_leg(0, CL, 0.0f, 0.0f, FRESH_HB, CS_NO_STAMP);
  CHECK(platform_at_stow(micros64()).reason == StowReason::STALE_ENC);
  // An IDLE Leg's encoder is not consulted (clause (a) is position-free).
  cs_stage_leg(0, IDLE, 3.0f, 0.0f, FRESH_HB, CS_NO_STAMP);
  CHECK(at_stow());
}

// ── 6 ── mutation: dropping the error term ─────────────────────────────────────
TEST_CASE("6: a CL Leg at Stow with active_errors refuses ERRORED") {
  reset_all();
  stage_all_idle();
  cs_stage_leg(5, CL, 0.0f, 0.0f, FRESH_HB, FRESH_ENC);
  CHECK(at_stow());
  axes[5].active_errors = 0x1u;
  const StowVerdict v = platform_at_stow(micros64());
  CHECK_FALSE(v.ok);
  CHECK(v.blocking_leg == 5);
  CHECK(v.reason == StowReason::ERRORED);
}

// ── 7 ── mutation: allowing every non-CL state ─────────────────────────────────
TEST_CASE("7: states 0, 3 and 9 refuse UNKNOWN_STATE; ENCODER_INDEX_SEARCH (6) is not-holding") {
  const uint8_t refused[] = { 0, 3, 9 };
  for (uint8_t st : refused) {
    reset_all();
    stage_all_idle();
    cs_stage_leg(2, st, 0.0f, 0.0f, FRESH_HB, FRESH_ENC);
    INFO("state=" << (int)st);
    const StowVerdict v = platform_at_stow(micros64());
    CHECK_FALSE(v.ok);
    CHECK(v.reason == StowReason::UNKNOWN_STATE);
  }
  reset_all();
  stage_all_idle();
  cs_stage_leg(2, ODriveState::ENCODER_INDEX_SEARCH, 1.7f, 0.5f, FRESH_HB, FRESH_ENC);
  CHECK(at_stow());
}

// ── 8 ── mutation: dropping s_cl_cmd_us (the CL-confirmation clause) ───────────
TEST_CASE("8: a cached IDLE older than the last CL command + 10 ms refuses CL_UNCONFIRMED") {
  reset_all();
  stage_all_idle();
  REQUIRE(odrive_set_state_tx(0, CL, TxCls::OTHER, Who::LADDER) == DeResult::SENT);
  const uint64_t t_cl = micros64();
  CHECK(leg_cl_cmd_us(0) == t_cl);
  // The pre-command heartbeat still says IDLE: it must not count as not-holding.
  StowVerdict v = platform_at_stow(micros64());
  CHECK_FALSE(v.ok);
  CHECK(v.blocking_leg == 0);
  CHECK(v.reason == StowReason::CL_UNCONFIRMED);
  // A heartbeat 9.999 ms after the command: still unconfirmed.
  cs_advance(9999);
  cs_stage_leg(0, IDLE, -0.1f, 0.0f, 0, FRESH_ENC);
  CHECK(platform_at_stow(micros64()).reason == StowReason::CL_UNCONFIRMED);
  // At exactly +10 ms the reported state counts (here CL at Stow).
  cs_advance(1);
  cs_stage_leg(0, CL, -0.1f, 0.0f, 0, FRESH_ENC);
  CHECK(at_stow());
}

// ── 9 ── mutation: requiring all six Legs present ──────────────────────────────
TEST_CASE("9: absent Legs are skipped (the one-Leg bench); except_leg is skipped too") {
  reset_all();
  cs_stage_leg(0, CL, 0.0f, 0.0f, FRESH_HB, FRESH_ENC);
  // Garbage in an absent Leg's cache is never read.
  axes[3].axis_state = (uint8_t)CL;
  axes[3].pos_rev = 2.0f;
  CHECK(at_stow());
  CHECK(odrive_set_state_tx(0, IDLE, TxCls::RPC, Who::RPC) == DeResult::SENT);
  // Zero present Legs: vacuously at Stow.
  reset_all();
  CHECK(at_stow());
  // except_leg: Leg 0 posed, Leg 1 IDLE. Only the "every OTHER Leg" question passes.
  reset_all();
  cs_stage_leg(0, CL, 2.2f, 0.0f, FRESH_HB, FRESH_ENC);
  cs_stage_leg(1, IDLE, -0.1f, 0.0f, FRESH_HB, FRESH_ENC);
  CHECK_FALSE(platform_at_stow(micros64()).ok);
  CHECK(platform_at_stow(micros64(), 0).ok);
  CHECK_FALSE(platform_at_stow(micros64(), 1).ok);
}

// ── 10 ── mutation: a raw subtraction instead of rx_older_than ─────────────────
TEST_CASE("10: a stamp after now reads fresh (BUG-93), heartbeat and encoder") {
  reset_all();
  stage_all_idle();
  cs_stage_leg(1, IDLE, -0.1f, 0.0f, -1, FRESH_ENC);                  // 1 us in the future
  CHECK(at_stow());
  cs_stage_leg(1, IDLE, -0.1f, 0.0f, -3600000000ll, FRESH_ENC);       // 1 h in the future
  CHECK(at_stow());
  cs_stage_leg(1, CL, 0.0f, 0.0f, -1, -1);
  CHECK(at_stow());
}

// ── 11 ── mutation: HOMING_OWNER accepting any axis ────────────────────────────
TEST_CASE("11: HOMING_OWNER passes the IDLE only for homing_axis_in_flight()") {
  reset_all();
  stage_all_idle();
  cs_stage_leg(2, CL, -0.6f, -0.3f, FRESH_HB, FRESH_ENC);   // the homing Leg, retracting
  REQUIRE_FALSE(at_stow());
  g_homing_axis = 2;
  CHECK(odrive_set_state_tx(2, IDLE, TxCls::OTHER, Who::HOMING_OWNER) == DeResult::SENT);
  REQUIRE(cs_sent_count() == 1);
  CHECK(sent_axis(0) == 2);
  CHECK(sent_u32(0) == IDLE);
  CHECK(odrive_set_state_tx(1, IDLE, TxCls::OTHER, Who::HOMING_OWNER) == DeResult::REFUSED);
  CHECK(odrive_set_state_tx(2, IDLE, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
  CHECK(odrive_set_state_tx(2, ODriveState::ENCODER_INDEX_SEARCH, TxCls::OTHER, Who::HOMING_OWNER) == DeResult::REFUSED);
  CHECK(odrive_reboot_tx(2, TxCls::SAFETY, Who::HOMING_OWNER) == DeResult::REFUSED);
  g_homing_axis = 0xFF;
  CHECK(odrive_set_state_tx(2, IDLE, TxCls::OTHER, Who::HOMING_OWNER) == DeResult::REFUSED);
  CHECK(cs_sent_count() == 1);
}

// ── 12 ── mutation: HELD set on the RPC caller ─────────────────────────────────
TEST_CASE("12: the Hand passes ungated; Leg refusals bump the census and set HELD for LADDER/STOW, not RPC") {
  reset_all();
  stage_all_idle();
  cs_stage_leg(1, CL, 1.5f, 0.0f, FRESH_HB, FRESH_ENC);
  cs_stage_leg(HAND_AXIS, CL, 3.0f, 0.0f, FRESH_HB, FRESH_ENC);
  REQUIRE_FALSE(at_stow());
  CHECK(odrive_set_state_tx(HAND_AXIS, IDLE, TxCls::RPC, Who::RPC) == DeResult::SENT);
  CHECK(odrive_reboot_tx(HAND_AXIS, TxCls::SAFETY, Who::RPC) == DeResult::SENT);
  CHECK(odrive_sdo_write_tx(HAND_AXIS, 0x0042, 0.0f, TxCls::POLLER, Who::RPC) == DeResult::SENT);
  REQUIRE(cs_sent_count() == 3);
  CHECK(sent_cmd(0) == ODriveCmd::set_requested_state);
  CHECK(sent_cmd(1) == ODriveCmd::reboot_odrives);
  CHECK(sent_cmd(2) == ODriveCmd::RxSdo);
  for (size_t k = 0; k < 3; ++k) CHECK(sent_axis(k) == HAND_AXIS);
  CHECK(leg_deenergise_refusals() == 0);

  CHECK(odrive_set_state_tx(0, IDLE, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
  CHECK(leg_deenergise_refusals() == 1);
  CHECK(leg_held_mask() == 0);
  CHECK(odrive_set_state_tx(2, IDLE, TxCls::OTHER, Who::LADDER) == DeResult::REFUSED);
  CHECK(odrive_set_state_tx(5, IDLE, TxCls::SAFETY, Who::STOW) == DeResult::REFUSED);
  CHECK(leg_deenergise_refusals() == 3);
  CHECK(leg_held_mask() == (uint8_t)((1u << 2) | (1u << 5)));
  leg_held_clear(2);
  CHECK(leg_held_mask() == (uint8_t)(1u << 5));
  leg_held_clear_all();
  CHECK(leg_held_mask() == 0);
  CHECK(cs_sent_count() == 3);
  // Out-of-range axes: refused, nothing sent, not counted.
  CHECK(odrive_set_state_tx(NUM_AXES, IDLE, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
  CHECK(leg_deenergise_refusals() == 3);
}

// ── 13 ── mutations: freezing at the raw position; no clamp; sending on a stale encoder
TEST_CASE("13: the freeze target is the stopping point clamped to the segment; a stale encoder sends nothing") {
  const float d = 2.5f * 2.5f / (2.0f * ODriveDefaults::TRAP_DEC_LIMIT_RPS2);   // 0.104 rev
  CHECK(leg_freeze_target(1.0f, -2.5f, 2.0f, 0.0f) == doctest::Approx(1.0f - d));
  CHECK(leg_freeze_target(1.0f,  2.5f, 0.0f, 2.0f) == doctest::Approx(1.0f + d));
  CHECK(leg_freeze_target(0.05f, -2.5f, 0.0f, 2.0f) == 0.0f);   // clamped to the segment's low end
  CHECK(leg_freeze_target(1.95f,  2.5f, 2.0f, 0.0f) == 2.0f);   // high end, endpoints given reversed

  reset_all();
  cs_stage_leg(3, CL, 1.0f, -2.5f, FRESH_HB, FRESH_ENC);
  REQUIRE(leg_freeze_tx(3, 2.0f, 0.0f));
  REQUIRE(cs_sent_count() == 1);
  CHECK(sent_axis(0) == 3);
  CHECK(sent_cmd(0) == ODriveCmd::set_input_pos);
  CHECK(sent_f32(0) == doctest::Approx(-(1.0f - d)));     // Leg sign: the wire is ODrive convention
  CHECK(leg_held_mask() == (uint8_t)(1u << 3));

  // Stale encoder: no frame, the Leg is still marked HELD.
  reset_all();
  cs_stage_leg(4, CL, 1.0f, -2.5f, FRESH_HB, 151000);
  CHECK_FALSE(leg_freeze_tx(4, 2.0f, 0.0f));
  CHECK(cs_sent_count() == 0);
  CHECK(leg_held_mask() == (uint8_t)(1u << 4));
  // Hand / out of range: nothing.
  CHECK_FALSE(leg_freeze_tx(HAND_AXIS, 0.0f, 1.0f));
  CHECK(cs_sent_count() == 0);
}

// ── 14 ── mutation: diverging the window from DEACTIVATE's arrival constants ───
TEST_CASE("14: DEACTIVATE's arrival band implies the gate (sweep)") {
  for (int ip = -10; ip <= 10; ++ip) {
    for (int iv = -10; iv <= 10; ++iv) {
      const float pos = STOW_OFF_POSE_REV + JBOp::TARGET_REACHED_POS_TOL_REV * (float)ip / 10.0f;
      const float vel = JBOp::TARGET_REACHED_VEL_TOL_RPS * (float)iv / 10.0f;
      reset_all();
      for (uint8_t i = 0; i < NUM_LEGS; ++i) cs_stage_leg(i, CL, pos, vel, FRESH_HB, FRESH_ENC);
      INFO("pos=" << pos << " vel=" << vel);
      REQUIRE(at_stow());
    }
  }
}

// ── 15 ── mutation: counting ENCODER_INDEX_SEARCH or CL as idle in the A6 twin ─
TEST_CASE("15: all_present_legs_idle is strict IDLE under the same freshness rule") {
  reset_all();
  CHECK(all_present_legs_idle(micros64()));                    // none present
  stage_all_idle();
  CHECK(all_present_legs_idle(micros64()));
  cs_stage_leg(3, ODriveState::ENCODER_INDEX_SEARCH, 0.0f, 0.0f, FRESH_HB, FRESH_ENC);
  CHECK_FALSE(all_present_legs_idle(micros64()));
  cs_stage_leg(3, CL, 0.0f, 0.0f, FRESH_HB, FRESH_ENC);        // at Stow but holding
  CHECK_FALSE(all_present_legs_idle(micros64()));
  cs_stage_leg(3, IDLE, 0.0f, 0.0f, 510000, FRESH_ENC);        // stale
  CHECK_FALSE(all_present_legs_idle(micros64()));
  cs_stage_leg(3, IDLE, 0.0f, 0.0f, FRESH_HB, FRESH_ENC);
  CHECK(all_present_legs_idle(micros64()));
}

// =============================================================================
//  The bench exemption, rig_is_bench (FW 53 U1b, owner 2026-11-25). The RX witnesses
//  are driven directly through rig_note_* (can_buses.cpp's hooks are not compiled
//  natively); cs_stage_leg sets heartbeat_seen, which the census also counts.
//  Mutation table: fw53-u1b logbook entry.
// =============================================================================
static constexpr uint64_t SETTLE = RIG_BENCH_SETTLE_US;

// The bench Leg 0, CL at 2 rev (far off Stow), heard at `t_axis`, bus transmittable from `t_bus`.
static void stage_bench_leg(uint64_t t_axis, uint64_t t_bus) {
  reset_all();
  cs_stage_leg(0, CL, 2.0f, 0.0f, FRESH_HB, FRESH_ENC);
  rig_note_axis_frame(0, t_axis);
  rig_note_bus(true, t_bus);
}

// ── 16 ── mutation M1: the wrappers ask platform_at_stow only (bench ignored) ──
TEST_CASE("16: one Leg alone past the settle is a bench: IDLE, REBOOT and SDO_WRITE at 2 rev are SENT") {
  stage_bench_leg(T0, T0);
  cs_set_mono(T0 + SETTLE);
  CHECK(rig_is_bench(micros64()));
  CHECK_FALSE(at_stow());                      // the Platform predicate itself is unchanged
  CHECK(leg_idle_permitted(micros64()));
  CHECK(odrive_set_state_tx(0, IDLE, TxCls::RPC, Who::RPC) == DeResult::SENT);
  CHECK(odrive_reboot_tx(0, TxCls::SAFETY, Who::RPC) == DeResult::SENT);
  CHECK(odrive_sdo_write_tx(0, 0x1234, 1.0f, TxCls::RPC, Who::RPC) == DeResult::SENT);
  CHECK(odrive_set_state_tx(0, IDLE, TxCls::OTHER, Who::LADDER) == DeResult::SENT);
  CHECK(cs_sent_count() == 4);
  CHECK(leg_deenergise_refusals() == 0);
  CHECK(leg_held_mask() == 0);
  CHECK_FALSE(rig_robot_latched());
}

// ── 17 ── mutations M2: no settle; M3: anchor = bus only; M4: anchor = first axis only ──
TEST_CASE("17: before the settle a lone Leg is not a bench; the settle runs from the LATER of first axis and bus") {
  // M2: 1 us short of the settle refuses.
  stage_bench_leg(T0, T0);
  cs_set_mono(T0 + SETTLE - 1);
  CHECK_FALSE(rig_is_bench(micros64()));
  CHECK(odrive_set_state_tx(0, IDLE, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
  CHECK(cs_sent_count() == 0);
  // M3: the robot powered AFTER the bridge booted: the bus was transmittable for
  // 60 s with nothing on it, then the first drive spoke. Its first 100 ms (and the
  // whole settle) must not read as a bench.
  stage_bench_leg(T0 + 60000000ull, T0);
  cs_set_mono(T0 + 60000000ull + 100000ull);
  CHECK_FALSE(rig_is_bench(micros64()));
  cs_set_mono(T0 + 60000000ull + SETTLE - 1);
  CHECK_FALSE(rig_is_bench(micros64()));
  cs_set_mono(T0 + 60000000ull + SETTLE);
  CHECK(rig_is_bench(micros64()));
  // M4: the axis heard long ago, the bus transmittable only since T0 + 60 s.
  stage_bench_leg(T0, T0 + 60000000ull);
  cs_set_mono(T0 + 60000000ull + SETTLE - 1);
  CHECK_FALSE(rig_is_bench(micros64()));
  cs_set_mono(T0 + 60000000ull + SETTLE);
  CHECK(rig_is_bench(micros64()));
}

// ── 18 ── mutations M5: the Platform witness ignored; M6: TRAFFIC_REPORT not a Platform id ──
TEST_CASE("18: one Leg plus a Platform Teensy frame is a robot, latched; the Platform id classifier") {
  stage_bench_leg(T0, T0);
  rig_note_platform_frame();
  cs_set_mono(T0 + 10 * SETTLE);
  CHECK_FALSE(rig_is_bench(micros64()));
  CHECK(rig_robot_latched());
  CHECK(odrive_set_state_tx(0, IDLE, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
  CHECK(cs_sent_count() == 0);
  // Every id the Platform Teensy sends counts; ODrive and bridge-sent ids do not.
  for (uint32_t id : { PlatformCanId::TRAFFIC_REPORT, PlatformCanId::STATE_UPDATE,
                       PlatformCanId::TILT_READING, PlatformCanId::FW_UPDATE_REPLY,
                       PlatformCanId::HAND_FAN_STATUS, PlatformCanId::IMU_REPLY,
                       PlatformCanId::IMU_STATUS, PlatformCanId::IMU_DATA,
                       PlatformCanId::IMU_LIVE_ACCEL, PlatformCanId::IMU_LIVE_GYRO }) {
    INFO("id=" << id);
    CHECK(is_platform_teensy_tx_id(id));
  }
  CHECK_FALSE(is_platform_teensy_tx_id(0x001));                                  // Leg 0 heartbeat
  CHECK_FALSE(is_platform_teensy_tx_id((uint32_t)(HAND_AXIS << 5) | 0x09));      // Hand encoder
  CHECK_FALSE(is_platform_teensy_tx_id(0x7DD));                                  // bridge time sync
}

// ── 19 ── mutation M7: no sticky robot latch ──────────────────────────────────
TEST_CASE("19: two axes heard, then one silent, stays a robot until the next boot") {
  stage_bench_leg(T0, T0);
  cs_stage_leg(1, IDLE, -0.1f, 0.0f, FRESH_HB, FRESH_ENC);   // heartbeat_seen only, no RX note
  cs_set_mono(T0 + 1000);
  CHECK_FALSE(rig_is_bench(micros64()));
  CHECK(rig_robot_latched());
  // Leg 1 silent for a minute (a stale heartbeat still counts as heard) ...
  cs_set_mono(T0 + 60000000ull);
  CHECK_FALSE(rig_is_bench(micros64()));
  // ... and even with its presence latch wiped (a future reboot path), still a robot.
  axes[1].heartbeat_seen = false;
  CHECK(rig_axes_heard_mask() == 0x01);
  CHECK_FALSE(rig_is_bench(micros64()));
  CHECK(odrive_set_state_tx(0, IDLE, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
  // The next boot clears it.
  leg_stow_gate_init();
  rig_note_axis_frame(0, T0 + 60000000ull);
  rig_note_bus(true, T0 + 60000000ull);
  cs_set_mono(T0 + 60000000ull + SETTLE);
  CHECK(rig_is_bench(micros64()));
}

// ── 20 ── mutation M8: only Leg axes counted (the Hand ignored) ───────────────
TEST_CASE("20: one Leg plus the Hand is a robot") {
  stage_bench_leg(T0, T0);
  rig_note_axis_frame(HAND_AXIS, T0 + 500000ull);
  cs_set_mono(T0 + 10 * SETTLE);
  CHECK(rig_axes_heard_mask() == (uint8_t)(0x01 | (1u << HAND_AXIS)));
  CHECK_FALSE(rig_is_bench(micros64()));
  CHECK(rig_robot_latched());
  CHECK(odrive_set_state_tx(0, IDLE, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
}

// ── 21 ── mutations M9: a non-transmittable sample does not restart the clock;
//          M10: rig_is_bench ignores the bus (tx == 0) ─────────────────────────
TEST_CASE("21: the bus not transmittable holds the settle clock") {
  stage_bench_leg(T0, T0);
  // Bus drops 3 s in, returns 4 s in: the settle restarts from the return.
  rig_note_bus(false, T0 + 3000000ull);
  cs_set_mono(T0 + 3500000ull);
  CHECK_FALSE(rig_is_bench(micros64()));
  rig_note_bus(true, T0 + 4000000ull);
  rig_note_bus(true, T0 + 4500000ull);          // later samples do not move the anchor
  cs_set_mono(T0 + 4000000ull + SETTLE - 1);
  CHECK_FALSE(rig_is_bench(micros64()));
  CHECK(odrive_set_state_tx(0, IDLE, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
  cs_set_mono(T0 + 4000000ull + SETTLE);
  CHECK(rig_is_bench(micros64()));
  // Bus down now: not a bench, however long ago the settle completed.
  rig_note_bus(false, T0 + 20000000ull);
  cs_set_mono(T0 + 20000001ull);
  CHECK_FALSE(rig_is_bench(micros64()));
  // Never transmittable since boot: never a bench.
  reset_all();
  cs_stage_leg(0, CL, 2.0f, 0.0f, FRESH_HB, FRESH_ENC);
  rig_note_axis_frame(0, T0);
  cs_set_mono(T0 + 10 * SETTLE);
  CHECK_FALSE(rig_is_bench(micros64()));
}

// ── 22 ── mutations M11: "exactly one" weakened to "any"; M12: no now >= anchor guard ──
TEST_CASE("22: the six-Leg robot past the settle stays gated; a now older than the anchor is not settled") {
  reset_all();
  stage_all_idle();
  for (uint8_t i = 0; i < NUM_LEGS; ++i) rig_note_axis_frame(i, T0);
  rig_note_bus(true, T0);
  cs_stage_leg(3, CL, 2.2f, 0.0f, FRESH_HB, FRESH_ENC);
  cs_set_mono(T0 + 10 * SETTLE);
  CHECK_FALSE(rig_is_bench(micros64()));
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    CHECK(odrive_set_state_tx(i, IDLE, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
    CHECK(odrive_reboot_tx(i, TxCls::SAFETY, Who::RPC) == DeResult::REFUSED);
    CHECK(odrive_sdo_write_tx(i, 0x1234, 1.0f, TxCls::RPC, Who::RPC) == DeResult::REFUSED);
  }
  CHECK(cs_sent_count() == 0);
  // A caller that captured `now` before the first axis frame was stamped.
  stage_bench_leg(T0 + 10 * SETTLE, T0);
  CHECK_FALSE(rig_is_bench(T0));
  // No axis heard at all is not a bench either (nothing to exempt).
  reset_all();
  rig_note_bus(true, T0);
  cs_set_mono(T0 + 10 * SETTLE);
  CHECK_FALSE(rig_is_bench(micros64()));
}
