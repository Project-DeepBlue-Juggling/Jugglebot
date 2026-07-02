// =============================================================================
//  test_rpc_dispatch.cpp — compiled test of the REAL rpc.cpp server dispatch
// =============================================================================
//  Drives the actual compiled rpc.cpp dispatch() + send_axis_frame() (both static,
//  reached via #include) against the recording fake HAL, asserting the (method,axis)
//  ENFORCEMENT POINT + the request codec + the Phase-6 gate-basis routing that no
//  test ever EXECUTED (coverage gap 3 — test_rpc_dispatch_lint.py is a text regex
//  over the cases, not a compile). The safety-relevant behaviours:
//
//    * the hand-axis-6 allow-table (JbUdp::hand_axis6_permitted) gates which methods
//      reach axis 6; a leg axis >= NUM_LEGS is ERR_BAD_ARGS;
//    * the Phase-6 gate-basis SPLIT: CLEAR_ERRORS/REBOOT gate on the live
//      bus-transmittable (SYNCH) signal (reach a just-repowered bus), everything else
//      on heartbeat-staleness (a stale bus withholds a setpoint/config);
//    * arg-decode bounds (arg_len < sizeof(Arg) → ERR_BAD_ARGS);
//    * unknown / server-shouldn't-receive methods → ERR_UNKNOWN_METHOD;
//    * BB presence gate + range-check; AXIS_ALL fans out over legs + hand;
//    * REBOOT arms the watchdog-suppression latch for a LEG but not the hand.
//
//  Routing ISOLATION: the leg/hand/relay/version handlers are STUBBED (recording),
//  so this driver tests dispatch ROUTING, not those modules' internals (which have
//  their own native drivers). SCOPE: decision logic only.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "udp_protocol.h"
#include "udp_link.h"            // FrameHandler typedef (for the udp_on_rpc_request stub)
#include "protocol_config.h"
#include "odrive_protocol.h"
#include "canbridge_config.h"
#include "ball_butler_protocol.h"
#include "ball_butler_state.h"
#include "can_buses.h"
#include "fault_machine.h"
#include "leg_homing.h"
#include "leg_activate.h"
#include "leg_deactivate.h"
#include "platform_relay.h"
#include "version_check.h"
#include "hand_ops.h"
#include "fake_hal.h"

// ── Driver-local leaf/handler stubs (the disjoint set fake_hal does NOT define) ─
//  fake_hal provides jugglebot_commands_allowed / can_jugglebot_send / now_wall_us /
//  micros64. These are the remainder rpc.cpp links against. Everything here records
//  or returns a settable value so a TEST_CASE can assert routing + gate decisions.
namespace CanBridge {

static bool     g_bus_transmittable = true;   // drives jugglebot_bus_transmittable()
static int      g_bb_sent = 0;                // can_bb_send call count
static ODrive::CanFrame g_bb_last{};          // recording CAN1 (BB) TX
static bool     g_bb_send_ok = true;
static int      g_clear_calls = 0;            // fault_notify_clear_errors count
static int      g_reboot_calls = 0;           // fault_notify_reboot_started count
static uint8_t  g_homing_axis = 0xFF, g_activate_axis = 0xFF, g_deactivate_axis = 0xFF;
static uint16_t g_homing_ret = JbUdp::RpcStatus::OK;
static uint16_t g_activate_ret = JbUdp::RpcStatus::OK;
static uint16_t g_deactivate_ret = JbUdp::RpcStatus::OK;
static uint16_t g_hand_traj_ret = JbUdp::RpcStatus::OK;
static int      g_hand_traj_calls = 0;
static uint16_t g_version_len = 8;            // version_fill_blob returned length
static int      g_tilt_calls = 0, g_state_read_calls = 0, g_state_write_calls = 0;

static void drv_reset() {
  g_bus_transmittable = true;
  g_bb_sent = 0; g_bb_last = ODrive::CanFrame{}; g_bb_send_ok = true;
  g_clear_calls = 0; g_reboot_calls = 0;
  g_homing_axis = g_activate_axis = g_deactivate_axis = 0xFF;
  g_homing_ret = g_activate_ret = g_deactivate_ret = JbUdp::RpcStatus::OK;
  g_hand_traj_ret = JbUdp::RpcStatus::OK; g_hand_traj_calls = 0;
  g_version_len = 8;
  g_tilt_calls = g_state_read_calls = g_state_write_calls = 0;
}

// ── can_buses.h ──
bool jugglebot_bus_transmittable() { return g_bus_transmittable; }
bool can_bb_send(const ODrive::CanFrame& f) { g_bb_sent++; g_bb_last = f; return g_bb_send_ok; }

// ── fault_machine.h ──
void fault_notify_clear_errors()  { g_clear_calls++; }
void fault_notify_reboot_started() { g_reboot_calls++; }

// ── leg_homing/activate/deactivate.h (entry points only — record + settable ret) ──
uint16_t homing_request(uint8_t axis)     { g_homing_axis = axis; return g_homing_ret; }
uint16_t activate_request(uint8_t axis)   { g_activate_axis = axis; return g_activate_ret; }
uint16_t deactivate_request(uint8_t axis) { g_deactivate_axis = axis; return g_deactivate_ret; }

// ── platform_relay.h ──
namespace Relay {
uint16_t tilt_read()  { g_tilt_calls++; return JbUdp::RpcStatus::OK; }
uint16_t state_read() { g_state_read_calls++; return JbUdp::RpcStatus::OK; }
uint16_t state_write(const JbUdp::RpcArgs::ArgRobotState&) { g_state_write_calls++; return JbUdp::RpcStatus::OK; }
}  // namespace Relay

// ── hand_ops.h ──
namespace HandOps {
uint16_t hand_traj_cmd(const JbUdp::RpcArgs::ArgHandTraj&) { g_hand_traj_calls++; return g_hand_traj_ret; }
}  // namespace HandOps

// ── version_check.h ──
uint16_t version_fill_blob(uint8_t* out, uint16_t cap) {
  const uint16_t n = g_version_len < cap ? g_version_len : cap;
  for (uint16_t i = 0; i < n; ++i) out[i] = 0;
  return n;
}

// ── udp_link.h (link deps pulled in by rpc_server_init/on_request; never called) ──
bool udp_send_rpc(uint8_t, const uint8_t*, uint16_t) { return true; }
void udp_on_rpc_request(FrameHandler) {}

}  // namespace CanBridge

#include "rpc.cpp"   // the unit under test (reaches static dispatch/send_axis_frame)

using namespace CanBridge;

// Call dispatch with a POD arg struct (raw bytes), returning the RpcStatus.
template <typename Arg>
static uint16_t call(uint16_t method, const Arg& a) {
  uint8_t result[64]; uint16_t res_len = 0;
  return Rpc::dispatch(method, reinterpret_cast<const uint8_t*>(&a), sizeof(a), result, res_len);
}
static uint16_t call_noarg(uint16_t method) {
  uint8_t result[64]; uint16_t res_len = 0;
  return Rpc::dispatch(method, nullptr, 0, result, res_len);
}

static void reset_all() { fake_reset(); drv_reset(); }

using namespace JbUdp;
using JbUdp::RpcArgs::ArgAxisState;
using JbUdp::RpcArgs::ArgAxisOnly;
using JbUdp::RpcArgs::ArgAbsPosition;
using JbUdp::RpcArgs::ArgSdoRead;
using JbUdp::RpcArgs::ArgBbThrow;
using JbUdp::RpcArgs::AXIS_ALL;

// ── routing + arg-decode ──────────────────────────────────────────────────────

TEST_CASE("NOP → OK; unknown method + a server-shouldn't-receive method → ERR_UNKNOWN_METHOD") {
  reset_all();
  CHECK(call_noarg(RpcMethod::NOP) == RpcStatus::OK);
  CHECK(call_noarg(0xBEEF) == RpcStatus::ERR_UNKNOWN_METHOD);
  CHECK(call_noarg(RpcMethod::TIME_OF_DAY_QUERY) == RpcStatus::ERR_UNKNOWN_METHOD);
}

TEST_CASE("a short arg (arg_len < sizeof(Arg)) → ERR_BAD_ARGS, nothing sent") {
  reset_all();
  uint8_t oneb[1] = {0};
  uint8_t result[64]; uint16_t res_len = 0;
  CHECK(Rpc::dispatch(RpcMethod::SET_AXIS_STATE, oneb, 1, result, res_len) == RpcStatus::ERR_BAD_ARGS);
  CHECK(fake_sent_count() == 0);
}

TEST_CASE("a valid leg SET_AXIS_STATE sends the byte-exact ODrive frame") {
  reset_all();
  ArgAxisState a{}; a.axis = 2; a.state = 8;  // CLOSED_LOOP
  CHECK(call(RpcMethod::SET_AXIS_STATE, a) == RpcStatus::OK);
  REQUIRE(fake_sent_count() == 1);
  const auto f = ODrive::encode_set_state(2, 8);
  CHECK(fake_sent_at(0).id == f.id);
  CHECK(memcmp(fake_sent_at(0).buf, f.buf, 8) == 0);
}

TEST_CASE("a non-hand axis >= NUM_LEGS → ERR_BAD_ARGS (send_axis_frame guard)") {
  reset_all();
  ArgAxisState a{}; a.axis = CanBridge::NUM_LEGS + 1; a.state = 8;   // not a leg, not the hand
  CHECK(call(RpcMethod::SET_AXIS_STATE, a) == RpcStatus::ERR_BAD_ARGS);
  CHECK(fake_sent_count() == 0);
}

// ── the hand-axis-6 allow-table ───────────────────────────────────────────────

TEST_CASE("hand axis 6: a permitted method reaches it, a non-permitted one is ERR_REJECTED") {
  reset_all();
  // SET_ABSOLUTE_POSITION is in the hand allow-table → reaches axis 6.
  ArgAbsPosition ap{}; ap.axis = HAND_AXIS; ap.position = 1.5f;
  CHECK(call(RpcMethod::SET_ABSOLUTE_POSITION, ap) == RpcStatus::OK);
  CHECK(fake_sent_count() == 1);
  // SDO_READ is NOT in the hand allow-table → ERR_REJECTED, nothing sent.
  reset_all();
  ArgSdoRead sd{}; sd.axis = HAND_AXIS; sd.endpoint = 488;
  CHECK(call(RpcMethod::SDO_READ, sd) == RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);
}

// ── the Phase-6 gate-basis split (the 2026-06-27 just-repowered-bus crux) ──────

TEST_CASE("stale bus (commands_allowed=false) but transmittable=true: CLEAR reaches it, SET_AXIS_STATE does not") {
  reset_all();
  fake_set_commands_allowed(false);   // heartbeat-stale
  g_bus_transmittable = true;         // but electrically alive (SYNCH=1)
  // CLEAR_ERRORS gates on bus-transmittable → allowed; refills the soft-reset budget.
  ArgAxisOnly c{}; c.axis = 0;
  CHECK(call(RpcMethod::CLEAR_ERRORS, c) == RpcStatus::OK);
  CHECK(g_clear_calls == 1);
  CHECK(fake_sent_count() == 1);
  // A normal command still gates on staleness → refused.
  fake_clear_sent();
  ArgAxisState a{}; a.axis = 0; a.state = 8;
  CHECK(call(RpcMethod::SET_AXIS_STATE, a) == RpcStatus::ERR_BUS_DOWN);
  CHECK(fake_sent_count() == 0);
}

TEST_CASE("a dead bus (both gates closed) refuses SET_AXIS_STATE (ERR_BUS_DOWN, nothing sent)") {
  reset_all();
  fake_set_commands_allowed(false);
  g_bus_transmittable = false;
  ArgAxisState a{}; a.axis = 0; a.state = 8;
  CHECK(call(RpcMethod::SET_AXIS_STATE, a) == RpcStatus::ERR_BUS_DOWN);
  CHECK(fake_sent_count() == 0);
}

// ── AXIS_ALL fan-out + the reboot latch arming ────────────────────────────────

TEST_CASE("CLEAR_ERRORS AXIS_ALL fans out over legs + hand (NUM_AXES frames)") {
  reset_all();
  ArgAxisOnly c{}; c.axis = AXIS_ALL;
  CHECK(call(RpcMethod::CLEAR_ERRORS, c) == RpcStatus::OK);
  CHECK(fake_sent_count() == CanBridge::NUM_AXES);
  CHECK(g_clear_calls == 1);
}

TEST_CASE("REBOOT arms the watchdog-suppression latch for a LEG but NOT for the hand") {
  reset_all();
  ArgAxisOnly leg{}; leg.axis = 0;
  CHECK(call(RpcMethod::REBOOT_ODRIVES, leg) == RpcStatus::OK);
  CHECK(g_reboot_calls == 1);          // leg reboot arms the latch

  reset_all();
  ArgAxisOnly hand{}; hand.axis = HAND_AXIS;
  CHECK(call(RpcMethod::REBOOT_ODRIVES, hand) == RpcStatus::OK);
  CHECK(g_reboot_calls == 0);          // hand-only reboot must NOT arm leg-loss suppression
}

// ── cold-start move + hand-traj routing (entry points stubbed) ────────────────

TEST_CASE("HOME / ACTIVATE / DEACTIVATE route to their *_request with the axis; HAND_TRAJ_CMD to HandOps") {
  reset_all();
  ArgAxisOnly h{}; h.axis = 3;
  CHECK(call(RpcMethod::HOME, h) == RpcStatus::OK);
  CHECK(g_homing_axis == 3);
  ArgAxisOnly ac{}; ac.axis = AXIS_ALL;
  CHECK(call(RpcMethod::ACTIVATE, ac) == RpcStatus::OK);
  CHECK(g_activate_axis == AXIS_ALL);
  CHECK(call(RpcMethod::DEACTIVATE, ac) == RpcStatus::OK);
  CHECK(g_deactivate_axis == AXIS_ALL);
  JbUdp::RpcArgs::ArgHandTraj ht{};
  CHECK(call(RpcMethod::HAND_TRAJ_CMD, ht) == RpcStatus::OK);
  CHECK(g_hand_traj_calls == 1);
}

// ── Ball Butler routing + presence gate + range-check ─────────────────────────

TEST_CASE("BB_THROW: range-check rejects before CAN1; in-range + BB present sends one CAN1 frame") {
  reset_all();
  // Out-of-range speed → ERR_BAD_ARGS, no CAN1 traffic (even before the presence gate).
  ArgBbThrow bad{}; bad.yaw_rad = 0.0f; bad.pitch_rad = 0.5f; bad.speed_mps = 100.0f; bad.delay_s = 0.1f;
  CHECK(call(RpcMethod::BB_THROW, bad) == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_bb_sent == 0);
  // In-range + BB present → one CAN1 frame on the THROW id.
  reset_all();
  bb_state.heartbeat_seen = true; bb_state.heartbeat_stale = false;
  ArgBbThrow ok{}; ok.yaw_rad = 0.0f; ok.pitch_rad = 0.5f; ok.speed_mps = 3.0f; ok.delay_s = 0.1f;
  CHECK(call(RpcMethod::BB_THROW, ok) == RpcStatus::OK);
  CHECK(g_bb_sent == 1);
  CHECK(g_bb_last.id == BallButlerCanId::THROW_CMD);
}

TEST_CASE("BB_RELOAD with BB absent → ERR_BUS_DOWN (presence gate, no CAN1)") {
  reset_all();
  bb_state.heartbeat_seen = false;    // BB not present
  CHECK(call_noarg(RpcMethod::BB_RELOAD) == RpcStatus::ERR_BUS_DOWN);
  CHECK(g_bb_sent == 0);
}

// ── GET_AXIS_VERSIONS result blob ─────────────────────────────────────────────

TEST_CASE("GET_AXIS_VERSIONS returns OK with a filled blob, ERR_BAD_ARGS on an empty blob") {
  reset_all();
  g_version_len = 8;
  CHECK(call_noarg(RpcMethod::GET_AXIS_VERSIONS) == RpcStatus::OK);
  reset_all();
  g_version_len = 0;                  // nothing cached yet
  CHECK(call_noarg(RpcMethod::GET_AXIS_VERSIONS) == RpcStatus::ERR_BAD_ARGS);
}
